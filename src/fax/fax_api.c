/**
 * @file fax_api.c
 * @brief The fax entry points slmodemd calls: FAX_create(), FAX_delete(), FAX_class1_command() and FAX_process().
 *
 * The FAX object (FaxGlue, fax_class1.h) is an opaque `void *` to slmodemd.  It is created when a fax session starts (modem.c), receives every audio
 * block through FAX_process() and the AT+F* commands through FAX_class1_command() (modem_at.c).  It is not a `struct dp` block and does not go through
 * the dp_* registry.  The T.30 / class 1 layer behind it is fax_class1.c.
 *
 * FAX_process() is a block loop with fax_class1_progress() as the block callback.  Samples pass through two 384-sample rings (FaxRing); one block is
 * `blk = srate / 50` samples (160 at 8000 Hz, 192 at 9600 Hz, 960 at 48000 Hz).  The line input of a block is converted to 8000 Hz by the RcFixed
 * resampler `rcIn` (NULL at 8000 Hz: the block is then processed in place and the converters are skipped), the class 1 layer must be handed exactly
 * 160 samples per call (otherwise "samples count %d != %d" is printed and the status of the block is -1), and its output is converted back by `rcOut`.
 * While the object is `online` the DTE bytes come from modem_recv_from_tty() and the bytes the class 1 layer produces go to modem_send_to_tty().
 *
 * The class 1 status (0 .. 10) is mapped to the FAX_STATUS_* return value of core/host_interface.h and to the `online` flag:
 * - 0 NO_MESSAGE: no status, `online` unchanged.
 * - 1 OK, 3 OK_NO_CARRIER: FAX_STATUS_OK, `online` = 0.
 * - 2 ERROR, 4 ERROR_NO_CARRIER, 5 ERROR_ON_HOOK: FAX_STATUS_ERROR, `online` = 0.
 * - 6 CONNECT: FAX_STATUS_CONNECT, `online` = 1.
 * - 7 NO_CARRIER: FAX_STATUS_NOCARRIER, `online` = 0.
 * - 8 NO_CARRIER_NO_MESSAGE, 9 OTHER_CARRIER: no status, `online` = 0.
 * - 10 ACCEPT_RATE: no status (debug print only), `online` unchanged.
 * - anything else: FAX_STATUS_ERROR ("Unknown status"), `online` unchanged.
 *
 * The object accepts the sample rates 8000, 9600 and 48000 Hz only; any other rate makes FAX_create() fail.
 */
#include <string.h>

#include "fax_class1.h"
#include "core/rcfixed_resampler.h"
#include "core/host_interface.h" /* enum FAX_CMD, FAX_STATUS_* */
#include "core/dsp_asm.h"
#include "core/dsp_host.h"

#include "core/dsp_debug_printf.h" /* dsplibs_debug_level, dsplibs_debug_printf, DBG(...) */
#include "core/dsp_constants.h"    /* SAMPLE_RATE_* */
/* the order of the loads / stores around it is the same with and without -fno-strict-aliasing (compared on the objects built with and without the flag) */

/* ================================================================================================ FAX_create */
/**
 * @brief Create a fax session object.
 * @param m       the slmodemd modem object (stored in the glue, used for the S7 register and the tty transfers)
 * @param caller  0 = answer side (the class 1 layer starts with the answer tone), non-zero = originate side
 * @param srate   line sample rate in Hz: 8000, 9600 or 48000
 * @return the new object, to be released with FAX_delete(), or NULL on a rejected sample rate or a failed allocation
 *
 * Allocates and clears the glue, creates the input and output resamplers for 9600 / 48000 Hz, sets the block size to `srate / 50`, and creates the class 1
 * layer with the configuration { role from `caller`, S7 register of `m`, high-pass filter on, calling tone on, answer tone 150 frames }.  Any failure
 * releases everything created so far.
 * @note  With a sample rate other than 8000 / 9600 / 48000 no resampler is created and the call fails through the same cleanup path.
 */
void* FAX_create(struct modem* m, unsigned int caller, unsigned int srate)
{
  FaxGlue* g;
  FaxClass1Cfg cfg;
  unsigned int blk;

  DBG("fax: create...\n");
  g = (FaxGlue*)sysdep_malloc((unsigned int)sizeof(FaxGlue));
  if (g == NULL)
    return NULL;
  sysdep_memset(g, 0, sizeof(FaxGlue));
  g->modem = m;

  /* resamplers between the line rate and the 8000 Hz rate of the class 1 layer (none at 8000 Hz) */
  if (srate != SAMPLE_RATE_8000)
  {
    RcFixed* rc = NULL;
    if (srate == SAMPLE_RATE_9600)
      rc = RcFixed_Create(3);
    else if (srate == SAMPLE_RATE_48000)
      rc = RcFixed_Create(5);
    g->rcIn = rc;
    if (rc == NULL)
      goto fail;
    rc = NULL;
    if (srate == SAMPLE_RATE_9600)
      rc = RcFixed_Create(2);
    else if (srate == SAMPLE_RATE_48000)
      rc = RcFixed_Create(4);
    g->rcOut = rc;
    if (rc == NULL)
      goto fail;
  }

  /* block size srate / 50 (computed as a multiply-shift); the output ring starts with one block of silence */
  blk = (unsigned int)(((unsigned long long)(srate * 160u) * 0x10624dd3ull) >> 32) >> 9;
  g->blk = blk;
  g->out.count = blk;
  g->out.half = blk;

  /* the class 1 layer */
  sysdep_memset(&cfg, 0, sizeof cfg);
  cfg.mode = (caller == 0) ? FAXC1_ROLE_ANSWER : FAXC1_ROLE_ORIGINATE;
  cfg.s7 = modem_get_sreg(m, 7);
  cfg.hpFilter = 1;
  DBG("fax: fax_class1 will created (ans_org=%d, s7=%d)\n", cfg.mode, cfg.s7);
  g->c1 = fax_class1_create(NULL, &cfg);
  if (g->c1 != NULL)
    return g;

fail:
  DBG("fax: delete...\n");
  if (g->rcIn != NULL)
    RcFixed_Delete(g->rcIn);
  if (g->rcOut != NULL)
    RcFixed_Delete(g->rcOut);
  if (g->c1 != NULL)
    fax_class1_delete(g->c1);
  g->c1 = NULL;
  sysdep_free(g);
  return NULL;
}

/* ================================================================================================ FAX_delete */
/**
 * @brief Release a fax session object created by FAX_create().
 * @param obj  the object
 * @warning `obj` must not be NULL (it is dereferenced without a check) and must not be used afterwards.
 */
void FAX_delete(void* obj)
{
  FaxGlue* g = (FaxGlue*)obj;
  DBG("fax: delete...\n");
  if (g->rcIn != NULL)
    RcFixed_Delete(g->rcIn);
  if (g->rcOut != NULL)
    RcFixed_Delete(g->rcOut);
  if (g->c1 != NULL)
    fax_class1_delete(g->c1);
  g->c1 = NULL;
  sysdep_free(g);
}

/* ================================================================================================ FAX_class1_command */
/** @brief True when `p` is one of the modem selections AT+FTM / AT+FRM accept (V.27ter 2400 / 4800, V.29 7200 / 9600, V.17 7200 .. 14400, long and short). */
static int modem_ok(unsigned int p)
{
  return p == FAXMOD_V27_2400 || p == FAXMOD_V27_4800 || p == FAXMOD_V29_7200 || p == FAXMOD_V17_7200 || p == FAXMOD_V17_7200_SHORT || p == FAXMOD_V29_9600 ||
         p == FAXMOD_V17_9600 || p == FAXMOD_V17_9600_SHORT || p == FAXMOD_V17_12000 || p == FAXMOD_V17_12000_SHORT || p == FAXMOD_V17_14400 ||
         p == FAXMOD_V17_14400_SHORT;
}

/**
 * @brief Pass an AT+F* class 1 command to the fax layer.
 * @param obj    the object from FAX_create(); NULL is rejected
 * @param cmd    FAX_CMD_FTS, FRS, FTM, FRM, FTH or FRH (core/host_interface.h)
 * @param param  the command parameter: a silence time for FTS / FRS, a FAXMOD_* modem selection (24 .. 144, i.e. 2400 .. 14400 bit/s) for FTM / FRM, the
 *               HDLC modulation (must be 3, V.21 300 bit/s) for FTH / FRH
 * @return 1 when the command was passed on; -1 for a NULL object, an unknown command, a modem selection FTM / FRM does not accept, or an FTH / FRH parameter
 *         other than 3 (nothing is passed on then)
 *
 * The mapping to the class 1 commands is FTS -> TS, FRS -> RS, FTM -> TM (with an extra silence time of 0x50), FRM -> RM, FTH -> TH, FRH -> RH.
 */
int FAX_class1_command(void* obj, unsigned int cmd, unsigned int param)
{
  FaxGlue* g = (FaxGlue*)obj;
  int c1cmd, extra = 0;
  if (g == NULL)
    return -1;
  if (g->c1 == NULL)
    return -1;

  DBG("fax: FAX_class1_command: %x\n", cmd);
  switch (cmd)
  {
  case FAX_CMD_FTS:
    c1cmd = FAXC1_TS;
    DBG("fax:  FAXC1_FTS, %x\n", param);
    break;
  case FAX_CMD_FRS:
    c1cmd = FAXC1_RS;
    DBG("fax:  FAXC1_FRS, %x\n", param);
    break;
  case FAX_CMD_FTM:
    DBG("fax:  FAXC1_FTM, %x, %d\n", param, 0);
    if (!modem_ok(param))
      return -1;
    c1cmd = FAXC1_TM;
    extra = 0x50;
    break;
  case FAX_CMD_FRM:
    DBG("fax:  FAXC1_FRM, %x\n", param);
    if (!modem_ok(param))
      return -1;
    c1cmd = FAXC1_RM;
    break;
  case FAX_CMD_FTH:
    DBG("fax:  FAXC1_FTH, %x\n", param);
    if (param != 3)
      return -1;
    c1cmd = FAXC1_TH;
    break;
  case FAX_CMD_FRH:
    DBG("fax:  FAXC1_FRH %x\n", param);
    c1cmd = FAXC1_RH;
    if (param != 3)
      return -1;
    break;
  default:
    DBG("fax: bad command: %x\n", cmd);
    return -1;
  }
  fax_class1_command(g->c1, c1cmd, (int)param, extra);
  return 1;
}

/* ================================================================================================ FAX_process */
/**
 * @brief Process `count` 16-bit audio samples: run the fax layer on every complete block and exchange the DTE bytes.
 * @param obj    the object from FAX_create(); must not be NULL
 * @param in     line input samples (`short`, `count` of them)
 * @param out    line output samples (`short`, `count` of them), always fully written
 * @param count  number of samples; slmodemd passes exactly one block (`srate / 50`)
 * @return the last non-zero FAX_STATUS_* of the call (statuses of successive blocks replace each other), or 0
 *
 * Each pass of the inner loop copies as many input samples as fit into the input ring, processes a block when one is complete (resample, DTE input,
 * fax_class1_progress(), DTE output, status mapping, resample back) and copies the same number of samples from the output ring to `out`.  The block's
 * status (-1 for a wrong sample count, or the mapped class 1 status) becomes the call's return value when non-zero.
 * @bug  The outer loop advances `in` and `out` by the block length in BYTES, while the inner loop consumes 2 bytes per sample: a call with more than one
 *       block reads and writes the wrong place from the second block on.  slmodemd always calls with exactly one block.
 * @bug  At 48000 Hz the block (960 samples) exceeds the 384-sample rings (the arithmetic wraps at `2 * blk`, past the end of `data`); the object only
 *       supports 8000 and 9600 Hz in practice (slmodemd uses 9600).
 * @warning With a block of other than 160 samples after resampling the status is -1 and the block is not processed.
 */
int FAX_process(void* obj, void* in_, void* out_, int count)
{
  FaxGlue* g = (FaxGlue*)obj;
  const char* in = (const char*)in_;
  char* out = (char*)out_;
  int ret = 0;
  while (count > 0)
  {
    unsigned int n_outer = ((unsigned int)count <= g->blk) ? (unsigned int)count : g->blk;
    int flag = 0;
    const char* ip = in;
    char* op = out;
    int rem = (int)n_outer;
    while (rem > 0)
    {
      unsigned int blk = g->blk;
      unsigned int chunk = (blk <= (unsigned int)rem) ? blk : (unsigned int)rem;
      unsigned int room, cnt;

      /* the chunk is limited by the free space before the end of the input ring and by the data before the end of the output ring */
      room = 2 * blk - g->in.wr;
      if (chunk > room)
        chunk = room;
      room = 2 * blk - g->out.rd;
      if (chunk > room)
        chunk = room;

      /* append the chunk to the input ring */
      sysdep_memcpy(&g->in.data[g->in.wr], ip, 2 * (size_t)chunk);
      ip += 2 * (size_t)chunk;
      cnt = g->in.count + chunk;
      {
        unsigned int nwr = g->in.wr + chunk;
        g->in.count = cnt;
        g->in.wr = nwr % (2 * g->blk);
      }
      blk = g->blk;

      /* a complete block is available: run the fax layer on it */
      if (cnt >= blk)
      {
        short* src = &g->in.data[g->in.half];
        short* dst = &g->out.data[g->out.half];
        short* rxp;
        short* txp;
        unsigned int n_rx;
        int status = 0;

        /* convert the input block to 8000 Hz (or process it in place at 8000 Hz) */
        if (g->rcIn != NULL)
        {
          unsigned int cap = 0xa0;
          RcFixed_Resample(g->rcIn, src, blk, g->rxbuf, &cap);
          n_rx = cap;
          rxp = g->rxbuf;
          txp = g->txbuf;
        }
        else
        {
          n_rx = blk;
          rxp = src;
          txp = dst;
        }

        if (n_rx != 0xa0)
        {
          DBG("fax: process: samples count %d != %d\n", (int)n_rx, 0xa0);
          status = -1;
        }
        else
        {
          int n_in = 0xa0, n_out = 0xa0, p7 = 0, p8;
          int st;

          /* DTE bytes for the class 1 layer, only while online and only when the previous call left room (tty_cnt) */
          if (g->online == 0)
          {
            p8 = 0;
          }
          else if (g->tty_cnt == 0)
          {
            p8 = 0;
          }
          else
          {
            unsigned int want = (unsigned int)g->tty_cnt;
            if (want > 0x1000u)
              want = 0x1000u;
            p8 = modem_recv_from_tty(g->modem, (char*)g->tty_in, (int)want);
            if (p8 > 0)
              DBG("fax: snd: %d (%d)\n", p8, g->tty_cnt);
          }

          st = fax_class1_progress(g->c1, rxp, txp, g->tty_out, g->tty_in, &n_in, &n_out, &p7, &p8);
          g->tty_cnt = p8;

          /* bytes the class 1 layer produced for the DTE */
          if (g->online != 0 && p7 > 0)
          {
            int r = modem_send_to_tty(g->modem, (const char*)g->tty_out, p7);
            DBG("fax: rcv: %d (%d)\n", r, p7);
          }

          /* class 1 status -> return value and online flag (see the file header) */
          switch ((unsigned int)st)
          {
          case FAX_CLASS1_NO_MESSAGE:
            break;
          case FAX_CLASS1_OK:
            DBG("fax: FAX_CLASS1_OK\n");
            status = FAX_STATUS_OK;
            g->online = 0;
            break;
          case FAX_CLASS1_ERROR:
            DBG("fax: FAX_CLASS1_ERROR\n");
            status = FAX_STATUS_ERROR;
            g->online = 0;
            break;
          case FAX_CLASS1_OK_NO_CARRIER:
            DBG("fax: FAX_CLASS1_OK_NO_CARRIER\n");
            status = FAX_STATUS_OK;
            g->online = 0;
            break;
          case FAX_CLASS1_ERROR_NO_CARRIER:
            DBG("fax: FAX_CLASS1_ERROR_NO_CARRIER\n");
            status = FAX_STATUS_ERROR;
            g->online = 0;
            break;
          case FAX_CLASS1_ERROR_ON_HOOK:
            DBG("fax: FAX_CLASS1_ERROR_ON_HOOK\n");
            status = FAX_STATUS_ERROR;
            g->online = 0;
            break;
          case FAX_CLASS1_CONNECT:
            DBG("fax: FAX_CLASS1_CONNECT\n");
            status = FAX_STATUS_CONNECT;
            g->online = 1;
            break;
          case FAX_CLASS1_NO_CARRIER:
            DBG("fax: FAX_CLASS1_NO_CARRIER\n");
            status = FAX_STATUS_NOCARRIER;
            g->online = 0;
            break;
          case FAX_CLASS1_NO_CARRIER_NO_MESSAGE:
            DBG("fax: FAX_CLASS1_NO_CARRIER_NO_MESSAGE\n");
            g->online = 0;
            break;
          case FAX_CLASS1_OTHER_CARRIER:
            DBG("fax: FAX_CLASS1_OTHER_CARRIER\n");
            g->online = 0;
            break;
          case FAX_CLASS1_ACCEPT_RATE:
            DBG("fax: FAX_CLASS1_ACCEPT_RATE\n");
            break;
          default:
            DBG("fax: process: Unknown status %d\n", st);
            status = FAX_STATUS_ERROR;
            break;
          }
        }

        /* convert the output block back to the line rate and advance both rings by one block */
        if (g->rcOut != NULL)
        {
          unsigned int cap = blk;
          RcFixed_Resample(g->rcOut, g->txbuf, n_rx, dst, &cap);
        }
        if (status != 0)
          flag = status;
        g->in.count -= g->blk;
        g->in.half = (g->in.half == 0) ? g->blk : 0;
        g->out.count += g->blk;
        g->out.half = (g->out.half == 0) ? g->blk : 0;
      }

      /* hand the chunk from the output ring to the caller */
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
