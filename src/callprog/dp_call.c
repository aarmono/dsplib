/**
 * @file dp_call.c
 * @brief Call-progress data pump: the glue between the host's data-pump interface and the CALLPROG state machine.
 *
 * dp_call_init() registers the data pump DP_CALLPROG ("call") with the host (modem_dp_register) and dp_call_exit() removes it.  The host then calls
 * call_create() when a call starts, call_run() with blocks of line samples, and call_delete() at the end (the function table is call_op).  The block it
 * works on is CallGlue (callprog.h): the host's `struct dp` first, the resamplers, staging buffers, two byte rings, the CallProg and the pulse-dial state.
 *
 * call_run(): the input is cut into pieces of at most 48 samples (0x60 bytes) and appended to the input ring.  Each time 48 samples are available they are
 * (resampled from the line rate to 8000 Hz into stageIn when needed) run through CALLPROG_Progress(); its output (resampled back through stageOut when the
 * line rate is not 8000 Hz) is appended to the output ring in two 0x60-byte halves.  The host's output is read from the output ring, which starts with 48
 * samples of silence, so the glue has one block of latency.  The result of CALLPROG_Progress() is mapped to the data-pump status that call_run() returns.
 * Once a final message is reported (answer, modem or voice answer, V.8bis answer or one of the tone detections) `ended` is set and only silence is
 * produced from then on.
 */
#include <stdint.h>

#include "callprog.h"

#include "core/dsp_debug_printf.h" /* dsplibs_debug_level, dsplibs_debug_printf, DBG(...) */
#include "core/dsp_constants.h"    /* SAMPLE_RATE_* */

/* S-register reader handed to CALLPROG_Create(): the register number is taken as an unsigned 16-bit value. */
static int call_GetSRegister(struct modem* m, int reg) { return modem_get_sreg(m, (unsigned)(unsigned short)reg); }

/**
 * Create the call-progress data pump (called by the host).
 * @param m       host modem object
 * @param id      data-pump id (stored in the dp)
 * @param caller  not used
 * @param srate   line sample rate: 8000 needs no resampling, 9600 and 48000 get resamplers to and from 8000 Hz
 * @param frag    not used
 * @param op      the operations table to store in the dp
 * @return the dp, or NULL if the allocation failed
 *
 * Initialises the pulse timing from the host, creates the state machine (blind dialling if S-register 0x38 is 0, 1 or 3) and starts the call with the host's
 * dial string; a string that starts with a digit and has at most 62 characters is prefixed with 'p' (S-register 0x10 is 0) or 't'.
 * @warning The dial string pointer from the host is dereferenced without a NULL check.  A line rate other than 8000, 9600 and 48000 gets no resamplers and
 *          its blocks of 48 samples are processed as if they were 8000 Hz.
 */
static struct dp* call_create(struct modem* m, enum DP_ID id, int caller, int srate, int frag, struct dp_operations* op)
{
  CallGlue* g;
  CallProgCfg cfg;
  char buf[0x42];
  char* ds;
  (void)caller;
  (void)frag;
  DBG("call: create...\n");
  g = (CallGlue*)sysdep_malloc((unsigned)sizeof(CallGlue));
  if (!g)
    return NULL;
  sysdep_memset(g, 0, sizeof(CallGlue));
  g->dp.modem = m;
  g->dp.dp_data = g;
  g->dp.id = id;
  g->dp.op = op;
  {
    unsigned sreg = (unsigned)modem_get_sreg(g->dp.modem, 0x38);
    cfg.blind = (sreg <= 1u) | (sreg == 3u);
  }
  cfg.getSReg = call_GetSRegister;
  cfg.modem = g->dp.modem;
  cfg.w0c = 0;

  /* pulse timing from the host, the state machine, the dial string */
  g->pulse.breakTime = (unsigned)modem_get_param(m, GetPulseDialBreakTime);
  g->pulse.makeTime = (unsigned)modem_get_param(m, GetPulseDialMakeTime);
  g->pulse.w5bc = 0;
  CALLPROG_Create(&g->cp, &cfg);
  ds = (char*)(intptr_t)modem_get_param(g->dp.modem, MDMPRM_DIALSTR);
  if ((unsigned char)(*ds - '0') <= 9u)
  {
    if (sysdep_strlen(ds) <= 0x3e)
    {
      buf[0] = modem_get_sreg(g->dp.modem, 0x10) == 0 ? 'p' : 't';
      sysdep_strcpy(buf + 1, ds);
      ds = buf;
    }
  }
  CALLPROG_Dial(&g->cp, ds);
  g->status = CALLPROG_MSG_MAX_MESSAGES;

  /* resamplers and the primed output ring (one silent block) */
  if (srate != SAMPLE_RATE_8000)
  {
    DBG("call: create RC: %d <-> %d...\n", srate, SAMPLE_RATE_8000);
    g->rcIn = srate == SAMPLE_RATE_9600 ? RcFixed_Create(3) : srate == SAMPLE_RATE_48000 ? RcFixed_Create(5) : NULL;
    g->rcOut = srate == SAMPLE_RATE_9600 ? RcFixed_Create(2) : srate == SAMPLE_RATE_48000 ? RcFixed_Create(4) : NULL;
  }
  g->outRing.cnt = 0x60;
  g->outRing.i3 = 0x60;
  return &g->dp;
}

/** Delete the call-progress data pump: release the state machine, the resamplers and the block. */
static int call_delete(struct dp* dp)
{
  CallGlue* g = (CallGlue*)dp->dp_data;
  DBG("call: delete...\n");
  CALLPROG_Delete(&g->cp);
  if (g->rcIn)
    RcFixed_Delete(g->rcIn);
  if (g->rcOut)
    RcFixed_Delete(g->rcOut);
  g->dp.dp_data = NULL;
  sysdep_free(g);
  return 0;
}

/**
 * Process a block of samples (called by the host).
 * @param dp    the dp created by call_create()
 * @param inv   input samples (16 bit)
 * @param outv  output samples (16 bit); the same number as the input is written
 * @param cnt   number of samples; nothing is done for cnt <= 0
 * @return the last non-zero status of this call: DPSTAT_NOANSWER (messages NO_RING, NO_ANSWER, ANSWER_STATE_TIMEOUT), DPSTAT_NODIALTONE, DPSTAT_CHANGEDP (any
 *         answer message and the two tone detections; the glue then ends), DPSTAT_BUSY (busy, congestion), DPSTAT_ERROR; DPSTAT_OK otherwise
 */
static int call_run(struct dp* dp, void* inv, void* outv, int cnt)
{
  CallGlue* g = (CallGlue*)dp->dp_data;
  CallRing *ar = &g->outRing, *br = &g->inRing;
  const unsigned char* in = (const unsigned char*)inv;
  unsigned char* out = (unsigned char*)outv;
  int ret = DPSTAT_OK;
  if (cnt <= 0)
    return 0;

  do
  {
    /* append up to 48 input samples to the input ring (limited by the room before the wrap of either ring) */
    unsigned bytes = cnt > 0x2f ? 0x60u : (unsigned)cnt * 2u;
    unsigned room = 0xc0u - (unsigned)br->i1;
    if (bytes > room)
      bytes = room;
    room = 0xc0u - (unsigned)ar->i2;
    if (bytes > room)
      bytes = room;
    sysdep_memcpy(br->data + br->i1, in, bytes);
    br->cnt += (int)bytes;
    in += bytes;
    br->i1 = (int)(((unsigned)br->i1 + bytes) % 0xc0u);

    /* a full block of 48 samples is available: process it */
    if (br->cnt > 0x5f)
    {
      unsigned char* wr = ar->data + ar->i3;
      short* src = (short*)(br->data + br->i3);
      if (g->ended != 0)
      {
        sysdep_memset(wr, 0, 0x60);
      }
      else
      {
        const short* pin;
        short* pout;
        int n2;
        int r;
        if (g->rcIn)
        {
          unsigned cap = 0xa0;
          RcFixed_Resample(g->rcIn, src, 0x30, g->stageIn, &cap);
          n2 = (int)cap;
          pin = g->stageIn;
          pout = g->stageOut;
        }
        else
        {
          pin = src;
          n2 = 0x30;
          pout = (short*)wr;
        }
        r = CALLPROG_Progress(&g->cp, pin, pout, n2);

        /* back to the line rate, then the status */
        if (g->rcOut)
        {
          unsigned cap = 0xa0;
          RcFixed_Resample(g->rcOut, g->stageOut, (unsigned)n2, (short*)wr, &cap);
        }
        if (g->status != r)
        {
          DBG("call: process: msg %d --> %d\n", g->status, r);
          g->status = r;
        }
        switch (r)
        {
        case CALLPROG_MSG_NO_RING:
        case CALLPROG_MSG_NO_ANSWER:
        case CALLPROG_MSG_ANSWER_STATE_TIMEOUT:
          ret = DPSTAT_NOANSWER;
          break;
        case CALLPROG_MSG_NO_DIAL_TONE:
          ret = DPSTAT_NODIALTONE;
          break;
        case CALLPROG_MSG_ANSWER:
        case CALLPROG_MSG_MODEM_ANSWER:
        case CALLPROG_MSG_VOICE_ANSWER:
        case CALLPROG_MSG_V8BIS_MODEM_ANSWER:
        case 16:
        case 17:
          g->ended = 1;
          ret = DPSTAT_CHANGEDP;
          break;
        case CALLPROG_MSG_BUSY:
        case CALLPROG_MSG_CONGESTION:
          ret = DPSTAT_BUSY;
          break;
        case CALLPROG_MSG_ERROR:
          ret = DPSTAT_ERROR;
          break;
        default:
          break;
        }
      }
      br->cnt -= 0x60;
      ar->cnt += 0x60;
      br->i3 = br->i3 == 0 ? 0x60 : 0;
      ar->i3 = ar->i3 == 0 ? 0x60 : 0;
    }

    /* return the same amount from the output ring */
    sysdep_memcpy(out, ar->data + ar->i2, bytes);
    out += bytes;
    ar->cnt -= (int)bytes;
    ar->i2 = (int)(((unsigned)ar->i2 + bytes) % 0xc0u);
    cnt -= (int)bytes >> 1;
  } while (cnt > 0);
  return ret;
}

/* The operations table registered with the host. */
static struct dp_operations call_op = {"call", 0, call_create, call_delete, call_run, 0};

/** @brief Register the call-progress data pump with the host. */
int dp_call_init(void) { return modem_dp_register(DP_CALLPROG, &call_op); }
/** @brief Remove the registration. */
void dp_call_exit(void) { modem_dp_deregister(DP_CALLPROG, &call_op); }
