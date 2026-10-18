/**
 * @file voice_api.c
 * @brief The voice glue slmodemd links: VOICE_create / VOICE_delete / VOICE_command / VOICE_process, and the callbacks it gives the engine.
 *
 * The glue sits between slmodemd's voice path (tty bytes and 16-bit line samples) and the voice engine of voice_engine.c, which works on 8000 Hz float
 * blocks of 160 samples.  Types: voice.h.  Three static callbacks (vce_hook_on(), vce_hook_off(), vce_get_sreg()) are handed to voice_create() as the
 * beep hook and parameter callbacks; they reach the beep generator, the detector and the silence detector.
 *
 * VOICE_process() has the shape of the data-pump wrapper (core/dp_wrapper.c): two 384-sample rings, blocks of srate / 50 samples, one sample-rate
 * converter per direction, and the engine as the block callback.  Per block it
 * - resamples the line input to 8000 Hz (RcFixed_Resample on rcIn) and converts it to float (x 1/16000);
 * - reads the tty bytes the state asks for (transmit state: the count the engine reported; receive state: up to 0xa0 bytes, watching for the
 *   abort sequence DLE '!');
 * - calls voice_modem(), sends the bytes the engine produced to the tty (receive state), and acts on the engine's message: state change, a status
 *   for the caller, or a "DLE <char>" report to the tty;
 * - converts the engine's float output back to 16 bits (x 16000, truncated) and resamples it to the line rate (RcFixed_Resample on rcOut).
 *
 * Supported line rates: 9600 Hz (converters 3 and 2) and 48000 Hz (converters 5 and 4); 8000 Hz is accepted by VOICE_create() but VOICE_process()
 * then calls RcFixed_Resample() with NULL converters (docs/bugs.md, section H).  The float scaling follows the policy of voice_engine.c.
 */
#include <string.h>

#include "voice.h"
#include "core/host_interface.h" /* MDMPRM_VOICEINFO (see voice_get_param in voice.h) */
#include "core/dsp_asm.h"

#include "core/dsp_debug_printf.h" /* dsplibs_debug_level, dsplibs_debug_printf, DBG(...) */

#include "voice_f32.h"
#include "voice_x87.h"
#include "voice_fist16.h"
#include "core/dsp_constants.h" /* SAMPLE_RATE_* */

static const double k_in16 = 0x1.0624dd2f1a9fcp-14;  /* 1/16000: line sample -> engine float */
static const float k_out16 = 0x1.f400000000000p+13f; /* 16000.0: engine float -> line sample */

/* ---- the callbacks VOICE_create() hands to the engine ---- */
/** Beep generator hook-on callback: only logs. */
static void vce_hook_on(struct modem* m) { DBG("voice: vce_hook_on (%p)...\n", (void*)m); }
/** Beep generator hook-off callback: only logs. */
static void vce_hook_off(struct modem* m) { DBG("voice: vce_hook_off (%p)...\n", (void*)m); }
/**
 * @brief Parameter callback of the engine's silence detector, tone detector and beep generator.
 * @param m   the modem (the voice_info block is read through it)
 * @param id  parameter id: 0x52 silence detect level (voice_info sensitivity >> 6, clamped to 3; a non-zero sensitivity below 64 gives 1),
 *            0x53 silence period, 0x49 detector level (3), 0x48 receive gain of #VLS 1 (0x13), 0x18 duration of a '!' beep digit (0x14),
 *            0x8a and 0x8b receive gain from voice_info
 * @return the value, or 0 for an unknown id
 */
static unsigned vce_get_sreg(struct modem* m, int id)
{
  const struct voice_info* vi = (const struct voice_info*)voice_get_param(m, MDMPRM_VOICEINFO);
  unsigned r = 0;
  switch ((unsigned)id)
  {
  case 0x52:
  {
    unsigned s = vi->silence_detect_sensitivity;
    r = s >> 6;
    if (r == 0)
      r = (s != 0);
    else if (r > 3)
      r = 3;
    break;
  }
  case 0x53:
    r = vi->silence_detect_period;
    break;
  case 0x49:
    r = 3;
    break;
  case 0x48:
    r = 0x13;
    break;
  case 0x18:
    r = 0x14;
    break;
  case 0x8a:
  case 0x8b:
    r = vi->rx_gain;
    break;
  default:
    break;
  }
  return r;
}

/* ---- the glue (what slmodemd links) ---- */

/**
 * @brief Create the voice glue and its engine.
 * @param m      the modem
 * @param srate  line sample rate: 8000, 9600 or 48000 Hz (anything else fails)
 * @return the glue object (opaque to the caller), or NULL on a failure
 * @note  The block length is srate / 50 (160, 192 or 960 samples).  The output ring starts with one block of silence.  The engine is created in the
 *        online mode (the initial SET_MODE request names no mode).
 * @warning At 8000 Hz both converters are NULL and VOICE_process() crashes on the first block (docs/bugs.md, section H).  At 48000 Hz the block of
 *          960 samples overflows the two 384-sample rings.
 */
void* VOICE_create(struct modem* m, unsigned srate)
{
  VoiceGlue* g;
  VoiceCfg cfg;
  int arg;
  unsigned blk;
  DBG("voice: VOICE_create...\n");
  g = (VoiceGlue*)sysdep_malloc((unsigned)sizeof(VoiceGlue));
  if (!g)
    return NULL;
  sysdep_memset(g, 0, sizeof(VoiceGlue));
  g->modem = m;

  /* the converters between the line rate and 8000 Hz (an unsupported rate leaves them NULL and fails) */
  if (srate != SAMPLE_RATE_8000)
  {
    RcFixed* rc = NULL;
    if (srate == SAMPLE_RATE_9600)
      rc = RcFixed_Create(3);
    else if (srate == SAMPLE_RATE_48000)
      rc = RcFixed_Create(5);
    g->rcIn = rc;
    rc = NULL;
    if (srate == SAMPLE_RATE_9600)
      rc = RcFixed_Create(2);
    else if (srate == SAMPLE_RATE_48000)
      rc = RcFixed_Create(4);
    g->rcOut = rc;
    if (!g->rcIn || !g->rcOut)
      goto fail;
  }

  g->state = VGLUE_COMMAND;
  g->vi = (const struct voice_info*)voice_get_param(m, MDMPRM_VOICEINFO);

  /* blk = srate * 160 / 8000 = srate / 50, as a multiplication by the reciprocal */
  blk = (unsigned)(((unsigned long long)(srate * 160u) * 0x10624dd3ull) >> 32) >> 9;
  g->blk = blk;
  g->out.count = blk;
  g->out.half = blk;

  /* the engine, with VLS 0 and the initial (no-mode) request */
  cfg.ctx = m;
  cfg.get_sreg = vce_get_sreg;
  cfg.hook_on = vce_hook_on;
  cfg.hook_off = vce_hook_off;
  g->voice = voice_create(&cfg);
  if (!g->voice)
    goto fail;
  arg = 0;
  voice_command(g->voice, VOICE_ECMD_VLS, &arg);
  arg = VOICE_MODE_NONE;
  voice_command(g->voice, VOICE_ECMD_SET_MODE, &arg);
  return g;

fail:
  if (g->rcIn)
    RcFixed_Delete(g->rcIn);
  if (g->rcOut)
    RcFixed_Delete(g->rcOut);
  sysdep_free(g);
  return NULL;
}

/**
 * @brief Delete the glue, its engine and its converters.
 * @param obj  the object VOICE_create() returned
 * @warning `obj` is not checked for NULL.
 */
void VOICE_delete(void* obj)
{
  VoiceGlue* g = (VoiceGlue*)obj;
  DBG("voice: VOICE_delete...\n");
  if (g->voice)
    voice_delete(g->voice);
  if (g->rcIn)
    RcFixed_Delete(g->rcIn);
  if (g->rcOut)
    RcFixed_Delete(g->rcOut);
  sysdep_free(g);
}

/**
 * @brief Execute a command of slmodemd's voice path.
 * @param obj  the object VOICE_create() returned; NULL gives -1
 * @param cmd  VOICE_CMD_STATE_COMMAND / _RX / _TX / _SPEAKER (set the engine mode online, receive, transmit, duplex and clear the pending tty
 *             byte count), VOICE_CMD_BEEP (two frequencies and tone_duration / 10 from the voice_info block), VOICE_CMD_DTMF (a symbol and a
 *             duration, at least one unit) or VOICE_CMD_ABORT
 * @return the engine's answer (0, or VOICE_MSG_ERROR when a beep or DTMF is requested outside the online and duplex modes), or -1 for an unknown command
 */
int VOICE_command(void* obj, enum VOICE_CMD cmd)
{
  VoiceGlue* g = (VoiceGlue*)obj;
  const struct voice_info* vi;
  unsigned c = (unsigned)cmd;
  int args[3];
  int id;
  if (!g)
    return -1;
  vi = g->vi;
  switch (c)
  {
  case VOICE_CMD_STATE_COMMAND:
    DBG("voice: VCE: VOICE_CMD_SET_MODE: COMMAND\n");
    args[0] = VOICE_MODE_ONLINE;
    goto mode;
  case VOICE_CMD_STATE_RX:
    DBG("voice: VCE: VOICE_CMD_SET_MODE: RX\n");
    args[0] = VOICE_MODE_RX;
    goto mode;
  case VOICE_CMD_STATE_TX:
    DBG("voice: VCE: VOICE_CMD_SET_MODE: TX\n");
    args[0] = VOICE_MODE_TX;
    goto mode;
  case VOICE_CMD_STATE_SPEAKER:
    DBG("voice: VCE: VOICE_CMD_SET_MODE: SPEAKER\n");
    args[0] = VOICE_MODE_DUPLEX;
    goto mode;
  case VOICE_CMD_BEEP:
    DBG("voice: VCE: VOICE_CMD_BEEP, %d %d %d\n", (int)vi->tone1_freq, (int)vi->tone2_freq, (int)(vi->tone_duration / 10u));
    args[0] = (int)vi->tone1_freq;
    args[1] = (int)vi->tone2_freq;
    args[2] = (int)(vi->tone_duration / 10u);
    id = VOICE_ECMD_BEEP;
    goto call;
  case VOICE_CMD_DTMF:
  {
    unsigned d;
    DBG("voice: VCE: VOICE_CMD_DTMF, %d %d\n", (int)vi->dtmf_symbol, (int)(vi->tone_duration / 10u));
    args[0] = (int)vi->dtmf_symbol;
    d = vi->tone_duration / 10u;
    args[1] = (int)(d ? d : 1u);
    id = VOICE_ECMD_DTMF;
    goto call;
  }
  case VOICE_CMD_ABORT:
    DBG("voice: VCE: VOICE_CMD_ABORT\n");
    id = VOICE_ECMD_ABORT;
    goto call;
  default:
    DBG("voice: VCE: Unknown command %u\n", c);
    return -1;
  }
mode:
  g->cnt = 0;
  id = VOICE_ECMD_SET_MODE;
call:
  return voice_command(g->voice, id, args);
}

/**
 * @brief Process `count` samples of line audio through the voice engine.
 * @param obj    the object VOICE_create() returned
 * @param in_    16-bit input samples at the line rate
 * @param out_   16-bit output samples at the line rate
 * @param count  number of samples; the data are processed in blocks of srate / 50 samples
 * @return 0, or the status of the last block that produced one: VOICE_STATUS_OK, VOICE_STATUS_CONNECT or VOICE_STATUS_ERROR
 * @note  The engine acts only when its message differs from the previous block's, so a message that repeats is handled once; the message 0 that most
 *        blocks return re-arms it.  The engine always runs a 160-sample block (sampleCount 0xa0), whatever the converter returned.
 * @warning The outer loop advances `in_` and `out_` by the block length in BYTES while the inner copies advance by two bytes per sample, so a call
 *          of more than one block reads and writes the wrong place from the second block on; slmodemd passes exactly one block (docs/bugs.md, section H).
 *          The input and output rings overflow at blocks above 192 samples (48000 Hz).  With NULL converters (8000 Hz) the first block crashes.
 */
int VOICE_process(void* obj, void* in_, void* out_, int count)
{
  VoiceGlue* g = (VoiceGlue*)obj;
  char* in = (char*)in_;
  char* out = (char*)out_;
  int ret = 0;
  while (count > 0)
  {
    unsigned n_outer = ((unsigned)count <= g->blk) ? (unsigned)count : g->blk;
    int flag = 0;
    const char* ip = in;
    char* op = out;
    int rem = (int)n_outer;
    while (rem > 0)
    {
      unsigned blk = g->blk;
      unsigned chunk = (blk <= (unsigned)rem) ? blk : (unsigned)rem;
      unsigned room, cnt, blkNow;

      /* queue as many input samples as fit in the rest of the block and of both rings */
      room = 2 * blk - g->in.wr;
      if (chunk > room)
        chunk = room;
      room = 2 * blk - g->out.rd;
      if (chunk > room)
        chunk = room;
      sysdep_memcpy(&g->in.data[g->in.wr], ip, 2 * (size_t)chunk);
      ip += 2 * (size_t)chunk;
      cnt = g->in.count + chunk;
      {
        unsigned nwr = g->in.wr + chunk;
        g->in.count = cnt;
        g->in.wr = nwr % (2 * g->blk);
      }
      blkNow = g->blk;

      /* a whole block is queued: run the engine on it */
      if (cnt >= blkNow)
      {
        unsigned cap = 0xa0, cap2;
        const short* src = &g->in.data[g->in.half];
        short* dsto = &g->out.data[g->out.half];
        int n_eng, k, msg, mf = 0;
        unsigned short ttyCount, outCount;

        /* line rate -> 8000 Hz -> float */
        RcFixed_Resample(g->rcIn, src, blkNow, g->sbuf, &cap);
        n_eng = (int)cap;
        for (k = 0; k < n_eng; k++)
          g->fin[k] = F(LD(g->sbuf[k]) * LD(k_in16));
        outCount = 0xa0;
        ttyCount = 0;

        /* tty input: the bytes the transmit state asked for; in the receive state a DLE '!' aborts */
        {
          int state = g->state;
          if (state == VGLUE_TX)
            ttyCount = (unsigned short)g->cnt;
          if (ttyCount != 0)
          {
            ttyCount = (unsigned short)modem_recv_from_tty(g->modem, (char*)g->tty, g->cnt);
            state = g->state;
          }
          if (state - 1 == 0)
          {
            int r = modem_recv_from_tty(g->modem, (char*)g->tty, 0xa0);
            if (r > 1)
            {
              DBG("voice: input in RX mode: %d (%d,%d)\n", r, (int)g->tty[0], (int)g->tty[1]);
              if (g->tty[0] == 0x10 && g->tty[1] == 0x21)
                VOICE_command(g, VOICE_CMD_ABORT);
            }
          }
        }

        /* the engine; its tty output goes out in the receive state, ttyCount becomes the byte count it wants next */
        msg = voice_modem(g->voice, g->tty, g->fout, g->fin, g->outb, &ttyCount, &outCount);
        if (g->state == VGLUE_RX)
        {
          if (outCount != 0)
            modem_send_to_tty(g->modem, (const char*)g->outb, outCount);
        }
        g->cnt = (int)ttyCount;

        /* act on a new engine message: change the glue state, set the status for the caller, or report a detected tone to the tty */
        if (g->last != msg)
        {
          switch ((unsigned)msg)
          {
          case VOICE_MSG_NONE:
            DBG("voice: STRM_VCE: VOICE_NO_MESSAGE\n");
            break;
          case VOICE_MSG_OK:
            DBG("voice: STRM_VCE:VOICE_OK\n");
            mf = VOICE_STATUS_OK;
            g->state = VGLUE_COMMAND;
            break;
          case VOICE_MSG_START_ONLINE:
            DBG("voice: STRM_VCE: VOICE_START_ONLINE\n");
            g->state = VGLUE_COMMAND;
            mf = VOICE_STATUS_OK;
            break;
          case VOICE_MSG_START_TX:
            DBG("voice: STRM_VCE:VOICE_START_TX\n");
            g->state = VGLUE_TX;
            mf = VOICE_STATUS_CONNECT;
            g->cnt = 0xa0;
            break;
          case VOICE_MSG_START_RX:
            DBG("voice: STRM_VCE:VOICE_START_RX\n");
            g->state = VGLUE_RX;
            mf = VOICE_STATUS_CONNECT;
            break;
          case VOICE_MSG_START_DUPLEX:
            DBG("voice: STRM_VCE:VOICE_START_DUPLEX\n");
            g->state = VGLUE_DUPLEX;
            mf = VOICE_STATUS_CONNECT;
            break;
          case VOICE_MSG_PURGE:
            DBG("voice: STRM_VCE:VOICE_PURGE\n");
            break;
          case VOICE_MSG_ERROR:
            DBG("voice: STRM_VCE:VOICE_ERROR\n");
            mf = VOICE_STATUS_ERROR;
            g->state = VGLUE_COMMAND;
            break;
          case VOICE_MSG_START_ONLINE_AFTER_ABORT:
            DBG("voice: STRM_VCE: VOICE_START_ONLINE_AFTER_ABORT(1)\n");
            g->state = VGLUE_COMMAND;
            mf = VOICE_STATUS_OK;
            break;
          case VOICE_MSG_CANCEL:
            DBG("voice: STRM_VCE:VOICE_CANCEL\n");
            break;
          case VOICE_MSG_BUSY:
          case VOICE_MSG_DIALTONE:
          case VOICE_MSG_FAX_TONE:
          case VOICE_MSG_UNDERRUN:
          {
            char rep[2];
            unsigned char ch;
            rep[0] = 0x10;
            if (msg == VOICE_MSG_BUSY)
            {
              DBG("voice: STRM_VCE: BUSY\n");
              ch = 0x62;
            }
            else if (msg == VOICE_MSG_DIALTONE)
            {
              DBG("voice: STRM_VCE: DIALTONE\n");
              ch = 0x64;
            }
            else if (msg == VOICE_MSG_FAX_TONE)
            {
              DBG("voice: STRM_VCE: FAX Tone\n");
              ch = 0x63;
            }
            else
            {
              DBG("voice: STRM_VCE: Underrun\n");
              ch = 0x75;
            }
            rep[1] = (char)ch;
            DBG("voice: report char: '%c' (%d)\n", (int)ch, (int)ch);
            modem_send_to_tty(g->modem, rep, 2);
            break;
          }
          default:
            DBG("voice: STRMVCE Monitor: Unknown message %d\n", msg);
            mf = VOICE_STATUS_ERROR;
            g->state = VGLUE_COMMAND;
            break;
          }
          g->last = msg;
        }

        /* float -> 16 bit -> line rate, into the output ring */
        {
          long double kk = LD(k_out16);
          for (k = 0; k < n_eng; k++)
            g->sbuf[k] = fist16(ld_mulf(kk, &g->fout[k]));
        }
        cap2 = blkNow;
        RcFixed_Resample(g->rcOut, g->sbuf, (unsigned)n_eng, dsto, &cap2);
        if (mf != 0)
          flag = mf;

        /* retire the block: swap both ring halves */
        g->in.count -= g->blk;
        g->in.half = (g->in.half == 0) ? g->blk : 0;
        g->out.count += g->blk;
        g->out.half = (g->out.half == 0) ? g->blk : 0;
      }

      /* hand out the processed samples */
      sysdep_memcpy(op, &g->out.data[g->out.rd], 2 * (size_t)chunk);
      op += 2 * (size_t)chunk;
      rem -= (int)chunk;
      g->out.count -= chunk;
      g->out.rd = (g->out.rd + chunk) % (2 * g->blk);
    }

    if (flag != 0)
      ret = flag;
    count -= (int)n_outer;
    in += n_outer;
    out += n_outer;
  }
  return ret;
}
