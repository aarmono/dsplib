/**
 * @file dp_runtime.c
 * @brief The three per-modem objects slmodemd's modem.c creates for every modem: the data-pump runtime parameter block, the DC remover and the
 *        ring detector wrapper.
 *
 * Entry points (called by the host; the types are in core/dp_glue.h):
 * - dp_runtime_create() / dp_runtime_delete(): the 0x88-byte DpRuntime block (the "modem parameter block", also reachable through dp_param_get()).  It
 *   is filled from the host's `struct dsp_info` (read through the real slmodemd struct, so `long clock_deviation` is narrowed to the block's 32-bit
 *   word) and from the codec type, modem_get_param(m, MDMPRM_CODECTYPE).  No pointer is stored in it.
 * - dcr_create() / dcr_process() / dcr_reset() / dcr_delete(): a DC-offset remover applied to the incoming samples.  It is a state machine
 *   (DcrState): wait n1 = 5760 samples, average n2 = 9600 samples to get a first estimate, then either track with 19200-sample windows and a 0.9 / 0.1
 *   smoothing (DCR_TRACK) or keep the estimate (DCR_HOLD).  dcr_process() optionally subtracts the DC from the samples and reports whether |dc| reached a
 *   threshold.
 * - RD_create() / RD_process() / RD_delete() / RD_ring_details(): the ring detector wrapper `{ modem, RingDetector * }`.  The detector configuration
 *   depends on the sample rate (8000 or 9600 Hz only) and on the codec type.  RD_ring_details() widens the detector's 32-bit results to the `long`
 *   the host declares.
 */
#include "dp_glue.h"
#include "core/dsp_asm.h"
#include "dsp_constants.h" /* SAMPLE_RATE_* */

/* ------------------------------------------------------------------ dp runtime */

/**
 * @brief Allocate and fill the data-pump runtime parameter block of a modem.
 * @param m  host modem object
 * @return the new DpRuntime block (release with dp_runtime_delete()), or NULL if the allocation failed
 *
 * The block is cleared, then set from `struct dsp_info` (qc_lapm -> bit 6 of flags2; qc_index, 0 mapped to 9, -> qcIndex; clock_deviation;
 * connection_type), the codec type (codecType) and a few constant defaults (w4 = 0x3c, w8 = 0x28, txMdTime = 0x2bc, minTxPowerReduction = 6, flags50 = 1); all other words stay 0.
 * The block is cleared a second time after the host call that fetches dsp_info, so the sequence of host calls (clear, get_param, clear) is fixed.
 * @warning The dsp_info pointer returned by the host is not checked for NULL.
 */
void* dp_runtime_create(struct modem* m)
{
  DpRuntime* rt;
  struct dsp_info* di;
  unsigned lapm, qc;
  int dev, conn;

  rt = (DpRuntime*)sysdep_malloc(sizeof(DpRuntime));
  if (!rt)
    return 0;
  sysdep_memset(rt, 0, sizeof(DpRuntime));
  di = (struct dsp_info*)modem_get_param(m, MDMPRM_DSPINFO);
  sysdep_memset(rt, 0, sizeof(DpRuntime));

  /* the dsp_info words are read once, up front: the block and the modem's dsp_info never overlap, but the compiler cannot know it without type-based alias
   * analysis, and the generated code would differ with -fno-strict-aliasing */
  lapm = di->qc_lapm;
  qc = di->qc_index;
  dev = (int)di->clock_deviation;
  conn = (int)di->connection_type;

  /* flag bytes: bit 4 of flags2 set, bit 6 of flags2 = qc_lapm & 1, the other flag bits cleared */
  rt->flags2 = (unsigned char)((((unsigned char)(rt->flags2 | 0x10)) & 0x9f) | (unsigned char)((lapm & 1) << 6));
  if (qc == 0)
    qc = 9;
  rt->flags3 &= 0xf8;
  rt->qcIndex = (int)qc;
  rt->flags2 &= 0x7f;

  /* constant defaults, the dsp_info words and the codec type */
  rt->ansLevel = 0;
  rt->w4 = 0x3c;
  rt->w8 = 0x28;
  rt->txMdTime = 0x2bc;
  rt->powerReduction = 0;
  rt->minTxPowerReduction = 6;
  rt->clockDeviation = dev;
  rt->flags50 = 1;
  rt->connectionType = conn;
  rt->codecType = (int)modem_get_param(m, MDMPRM_CODECTYPE);
  rt->w58 = 0;
  rt->w5c = 0;
  rt->minLevel = 0;
  rt->p2Silence = 0;
  rt->extDelay = 0;
  rt->delay = 0;
  return rt;
}

/** @brief Release a block from dp_runtime_create(). */
void dp_runtime_delete(void* runtime) { sysdep_free(runtime); }

/* ------------------------------------------------------------------ dcr */

/**
 * @brief Allocate a DC remover in state DCR_WAIT.
 * @return the new object (release with dcr_delete()), or NULL if the allocation failed
 *
 * Defaults: all three flag bits set (enabled, subtract, track after the first estimate), n1 = 5760, n2 = 9600, n3 = 19200 samples, threshold 3000.
 */
void* dcr_create(void)
{
  DcrObj* d = (DcrObj*)sysdep_malloc(sizeof(DcrObj));
  if (d)
  {
    sysdep_memset(d, 0, sizeof(DcrObj));
    d->flags |= 7;
    d->state = DCR_WAIT;
    d->n1 = 0x1680;
    d->n2 = 0x2580;
    d->n3 = 0x4b00;
    d->thr = 0xbb8;
  }
  return d;
}

/** @brief Release a DC remover from dcr_create(); NULL is accepted. */
void dcr_delete(void* p)
{
  if (p)
  {
    sysdep_free(p);
    /* deliberately not a tail call: with strict aliasing on, the compiler would store the incoming argument back into its stack slot before a sibling
     * call, which would make the code of this function depend on the aliasing flags for no aliasing reason */
    NO_TAIL_CALL();
  }
}

/** @brief Restart the DC estimate: clear the DC value and the running sum and count.  The state and the flags are kept. */
void dcr_reset(void* p)
{
  DcrObj* d = (DcrObj*)p;
  d->dc = 0;
  d->sum = 0;
  d->count = 0;
}

/**
 * @brief Update the DC estimate with a block of samples, optionally remove the DC from the block, and report a large DC offset.
 * @param p     DC remover from dcr_create(); NULL is accepted (returns 0)
 * @param buf_  `n` 16-bit samples; modified in place when flag bit 1 is set
 * @param n     number of samples
 * @return 1 when |dc| >= thr after the update, else 0 (also 0 when the remover is disabled, flag bit 0 clear, in which case the block is not touched)
 *
 * Per state: DCR_WAIT counts samples until n1 are seen (the block that reaches n1 is not used) and moves to DCR_ESTIMATE.  DCR_ESTIMATE accumulates
 * the sum and count of the samples until n2 are seen, then sets dc = sum / count (integer division) and moves to DCR_TRACK (flag bit 2 set) or DCR_HOLD.
 * DCR_TRACK re-averages over n3-sample windows and updates dc = (0x7333 * dc + 0xccd * mean + 0x4000) >> 15 (0.9 old + 0.1 new).  DCR_HOLD keeps dc.
 * In the TRACK and HOLD states an all-zero block (digital silence) is ignored: nothing changes and 0 is returned.  The subtraction is a 16-bit wrap-around,
 * not a saturating one.  With debug level above 1 the end of the first estimate is printed.
 * @warning `n` must not be negative (the sample loops are skipped but the sample count is still updated).
 */
int dcr_process(void* p, void* buf_, int n)
{
  DcrObj* d = (DcrObj*)p;
  short* buf = (short*)buf_;
  int ret = 0;
  unsigned char fl;
  int i;

  if (!d)
    return 0;
  fl = d->flags;
  if (!(fl & 1))
    return 0;

  /* silence in the steady states: leave the estimate alone */
  if ((unsigned)d->state - 2u <= 1u)
  {
    i = 0;
    if (0 < n)
    {
      do
      {
        if (buf[i] != 0)
          break;
        i++;
      } while (i < n);
    }
    if (i == n)
      return 0;
  }

  /* state machine: advance the estimate with this block */
  if (d->state == DCR_ESTIMATE)
  {
    int sum = 0, count, total;
    for (i = 0; i < n; i++)
      sum += buf[i];
    count = d->count + n;
    total = d->sum + sum;
    if (count >= d->n2)
    {
      int q = total / count;
      d->dc = (short)q;
      if (1 < dsplibs_debug_level)
      {
        dsplibs_debug_printf("DCR: initial DC Evaluation done, DC level %d, %sabled\n", (int)d->dc, (fl & 4) ? "en" : "dis");
        fl = d->flags;
      }
      d->state = (fl & 4) ? DCR_TRACK : DCR_HOLD;
      d->sum = 0;
      d->count = 0;
    }
    else
    {
      d->sum = total;
      d->count = count;
    }
  }
  else if ((unsigned)d->state > 1u)
  {
    if (d->state == DCR_TRACK)
    {
      int sum = 0, count, total;
      for (i = 0; i < n; i++)
        sum += buf[i];
      count = d->count + n;
      total = d->sum + sum;
      if (count >= d->n3)
      {
        int q = total / count;
        d->dc = (short)(((int)d->dc * 0x7333 + q * 0xccd + 0x4000) >> 15);
        d->sum = 0;
        d->count = 0;
      }
      else
      {
        d->sum = total;
        d->count = count;
      }
    }
  }
  else
  {
    int count = d->count + n;
    if (count < d->n1)
    {
      d->count = count;
    }
    else
    {
      d->state = DCR_ESTIMATE;
      d->count = 0;
      d->sum = 0;
    }
  }

  /* report a large offset and remove the current estimate from the block */
  {
    int dc = d->dc;
    int a = dc < 0 ? -dc : dc;
    if (a >= (int)d->thr)
      ret = 1;
    if (fl & 2)
    {
      for (i = 0; i < n; i++)
        buf[i] = (short)((unsigned short)buf[i] - (unsigned short)d->dc);
    }
  }
  return ret;
}

/* ------------------------------------------------------------------ RD */

/**
 * @brief Create the ring detector wrapper for a modem.
 * @param m     host modem object (kept in the wrapper; the codec type is read from it)
 * @param rate  sample rate, Hz; only 8000 and 9600 are accepted
 * @return the wrapper (release with RD_delete()), or NULL for another rate, a failed allocation or a failed RingDetector_Create()
 *
 * The detector configuration is: sample rate `rate`, minimum frequency 15 Hz, maximum 80 Hz, minimum on and off durations 120 ms, and an amplitude
 * threshold chosen from the codec type (modem_get_param(m, MDMPRM_CODECTYPE)): types 4 and 12 give 1000, types 13 and 15 give 650, type 14 gives 850,
 * every other type gives -3000.  (The sign of the threshold selects the confirm / release presets of RingDetector_Create().)
 */
void* RD_create(struct modem* m, unsigned rate)
{
  RdObj* o;
  int cfg[6];
  unsigned idx;

  if (1 < dsplibs_debug_level)
    dsplibs_debug_printf("RD: create...\n");
  if (rate != SAMPLE_RATE_8000 && rate != SAMPLE_RATE_9600)
    return 0;
  o = (RdObj*)sysdep_malloc(sizeof(RdObj));
  if (!o)
    return 0;
  sysdep_memset(o, 0, sizeof(RdObj));
  o->modem = m;

  /* detector configuration, indexed by RD_CFG_*: rate, min / max frequency, min on / off ms, threshold */
  cfg[0] = (int)rate;
  cfg[1] = 0xf;
  cfg[2] = 0x50;
  cfg[3] = 0x78;
  cfg[4] = 0x78;
  idx = (unsigned)modem_get_param(m, MDMPRM_CODECTYPE) - 4u;
  cfg[5] = -3000;
  if (idx <= 0xb)
  {
    switch (idx)
    {
    case 0:
    case 8:
      cfg[5] = 1000;
      break;
    case 9:
    case 11:
      cfg[5] = 0x28a;
      break;
    case 10:
      cfg[5] = 0x352;
      break;
    default:
      break;
    }
  }

  o->rd = RingDetector_Create(cfg);
  if (!o->rd)
  {
    sysdep_free(o);
    return 0;
  }
  return o;
}

/** @brief Delete the ring detector and the wrapper from RD_create(). */
void RD_delete(void* p)
{
  RdObj* o = (RdObj*)p;
  if (1 < dsplibs_debug_level)
    dsplibs_debug_printf("RD: delete...\n");
  RingDetector_Delete(o->rd);
  sysdep_free(o);
}

/**
 * @brief Run the ring detector over a block of line samples.
 * @param p      wrapper from RD_create()
 * @param in     `count` 16-bit samples
 * @param count  number of samples
 * @return the RingDetector_Process() result: 1 on each ring ON / OFF change (then RD_ring_details() has the frequency and duration), else 0
 */
int RD_process(void* p, void* in, int count)
{
  RdObj* o = (RdObj*)p;
  int freq, dur;
  int r = RingDetector_Process(o->rd, (short*)in, (unsigned)count);
  if (r != 0)
  {
    RingDetector_GetLastRing(o->rd, &freq, &dur);
    if (1 < dsplibs_debug_level)
      dsplibs_debug_printf("RD: RD: freq = %d, duration = %d\n", freq, dur);
  }
  return r;
}

/**
 * @brief Fetch the frequency and duration of the ring period that just ended (valid after RD_process() returned 1).
 * @param p         wrapper from RD_create()
 * @param freq      receives the ring frequency, Hz (0 on a ring start)
 * @param duration  receives the duration of the period that just ended, ms
 * @note  RingDetector_GetLastRing() writes 32-bit ints; the values are widened to the host's `long *` here.
 */
void RD_ring_details(void* p, long* freq, long* duration)
{
  RdObj* o = (RdObj*)p;
  int f, d;
  RingDetector_GetLastRing(o->rd, &f, &d);
  *freq = f;
  *duration = d;
}
