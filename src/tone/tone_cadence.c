/**
 * @file tone_cadence.c
 * @brief Cadence: the on / off cadence recogniser that drives a ToneIir, one line sample per cadence_progress() call.
 *
 * Used by the call-progress detectors (callprog/callprog.c: busy and dial tones) and the voice tone detectors (voice/voice_tone.c).  The object
 * and the handle types are in tone_detect.h; the band-pass detector underneath is in tone_iir.c.
 *
 * Phases.  The ToneIir reports once per window (Cadence::bufLen samples) whether the tone is present.  The Cadence counts windows: `state` 0 is
 * "in a tone", 1 is "in silence"; the lengths of the last tone and silence phases are recorded in on[] / off[] (slot `idx`).  Once `maxCycles`
 * slots are collected the cadence is judged against the configured limits (see cadence_progress()).  The dial-tone detector (reportTone) instead
 * reports every tone window, and its maxCycles (100) exceeds the 60 slots, so it never reaches the judgement.
 *
 * Configuration.  cadence_create() reads the detector parameters from the host with modem_get_param(m, id) (ids from core/host_interface.h), picks the
 * coefficient set from the filter index and sub-index, and converts the configured durations to window counts.
 */
#include "tone_detect.h"
#include "core/host_interface.h" /* MDMPRM_*, Get* parameter ids */
#include "core/dsp_host.h"

/* modem_get_param: declared by core/dsp_host.h */

#define DBG(n) (dsplibs_debug_level > (n))

#include "core/dsp_mulw.h"
/* |a - b| with wrapping 32-bit arithmetic */
static int absd(int a, int b)
{
  int d = (int)((unsigned)a - (unsigned)b);
  return d < 0 ? (int)(0u - (unsigned)d) : d;
}
/* a host duration `v` converted to a count of detection windows of `bufLen` samples: GetFP_Value(1, bufLen) * v * 80 >> 14 */
static int scaled(int v, int bufLen)
{
  int g = GetFP_Value(1, (short)bufLen);
  return mulw(mulw(g, v), 80) >> 14;
}

/**
 * @brief Create (or re-initialise) a cadence detector of the type given in `p->type`.
 * @param c      object to initialise, or NULL to allocate and zero one with sysdep_malloc() / sysdep_memset()
 * @param p      parameter record: `type` (CadenceType; any value >= 4 is treated as DIAL and written back as 4), `callerParamA` / `callerParamB`
 *               (stored in the object).  The other fields are not read.
 * @param extra  0 for the call-progress detectors; > 0 (the voice detectors pass 2) turns off the report-every-tone-window mode (reportTone) and
 *               multiplies the ToneIir integration length by extra + 1
 * @param m      host modem object the detector parameters are read from
 * @return the configured cadence detector, or NULL if the allocation failed or the detector is disabled
 *
 * Reads the host parameters (tone thresholds, cadence limits, filter index and sub-index, window length, ...) per tone type: busy, congestion,
 * ringback or dial (the default).  The coefficient set follows from the filter index and, for the dial type only, the filter sub-index: sub-index
 * 1 .. 7 selects that row of Filter_350_500 (filter index 0, 2, 7), Filter_100_550 (1, 4, 5, 6) or Filter_276_504 (3); in every other case the
 * single-row set is used: CP_276_504 (index 3), CP_450_630 (6), CP_100_550 (7), CP_350_600 (all others, including indices above 7).
 * The on / off limits are converted to window counts (scaled()), the ToneIir is created from a ToneIirCfg built
 * from the object, and the object is left in state 1 (tone) with empty slots.  With debug level above 1 the resulting parameters are printed.
 *
 * Congestion and ringback detectors are disabled when any of their four limits (max / min on / off) is 0: all resources are freed and NULL is returned.
 * @warning In that case a caller-supplied `c` is freed as well.
 * @bug   The result of toneiir_create() is stored without a NULL check; if that allocation fails, `ti` is NULL and cadence_progress() dereferences it.
 * @bug   The `allpass` configuration branch tests `c->allpass` after it was cleared at the top of the function, so it can never run.
 * @bug   The host parameters are not range-checked: a maxCycles of 60 or more is never reached (the slots wrap after 59), a negative
 *        maxCycles in the loose busy-tone mode makes cadence_progress() index on[] / off[] out of range (see there), and a bufLen whose low
 *        16 bits are 0 gives GetFP_Value(1, 0) == 0.
 */
Cadence* cadence_create(Cadence* c, CadenceParams* p, int extra, struct modem* m)
{
  ToneIirCfg cfg;
  ToneIir* ti;
  int type, ok = 1, sub = 0, filt, offTimes = 4;
  short level, thr;
  unsigned t, ptype = p->type;
  /* the caller's parameters are read once, up front, and p->type is written last: the result must not depend on whether the compiler assumes
   * *p and *c do not alias */
  int pw18 = p->callerParamB, pw0c = p->callerParamA;

  toneiir_get_default_configuration(&cfg);
  if (!c)
  {
    c = (Cadence*)sysdep_malloc(sizeof(Cadence));
    if (!c)
      return NULL;
    sysdep_memset(c, 0, sizeof(Cadence));
    c->ti = NULL;
  }

  ti = c->ti; /* an existing detector is kept and re-initialised; read before any other store to *c */
  c->allpass = 0;
  c->reportTone = 0;
  c->modem = m;
  type = (int)ptype;

  /* the detection level (a dial-tone parameter, used for every tone type) selects the energy threshold; the result of the first read is unused */
  (void)modem_get_param(c->modem, GetDialToneCallProgressFilterIndex);
  level = (short)modem_get_param(c->modem, GetDialToneDetectionThreshold);
  thr = Get_Detection_Threshold_Table(level);
  if (DBG(1))
    dsplibs_debug_printf("Detection Thresholds: levle_fix=%d,--> LEVEL_THRESHOLD=%d\r\n", (int)level, (int)thr);
  c->thr = thr;
  c->maxCycles = 100;
  c->loose = 0;
  if ((unsigned)type >= 4u)
    type = CADENCE_DIAL;

  /* per tone type: filter index, window length (bufLen), integration length and the on / off limits (host values, converted to windows below) */
  if (type == CADENCE_BUSY)
  {
    filt = (int)modem_get_param(c->modem, GetBusyToneCallProgressFilterIndex);
    offTimes = 3;
    c->bufLen = 0xa0;
    c->reportTone = 0;
    c->integ = 0;
    if (DBG(1))
      dsplibs_debug_printf("============> %d\n", filt);

    c->maxOn = (int)modem_get_param(c->modem, GetMaxBusyCadenceOnTime);
    c->minOn = (int)modem_get_param(c->modem, GetMinBusyCadenceOnTime);
    c->minOff = (int)modem_get_param(c->modem, GetMinBusyCadenceOffTime);
    c->maxOff = (int)modem_get_param(c->modem, GetMaxBusyCadenceOffTime);
    c->maxCycles = (int)modem_get_param(c->modem, GetBusyDetectionCyclesNumber);
    c->loose = (int)modem_get_param(c->modem, GetBusyToneLooseDetectionEnabled);
    if (DBG(1))
      dsplibs_debug_printf("Cadence: Busy Tone loose detection is %d\r\n", c->loose);

    /* an unconfigured (zero) limit selects the built-in busy limits for all four */
    if (c->maxOn == 0 || c->minOn == 0 || c->maxOff == 0 || c->minOff == 0)
    {
      c->maxOn = 0x37;
      c->minOn = 0x14;
      c->maxOff = 0x37;
      c->minOff = 0x14;
    }
  }
  else if (type == CADENCE_CONGESTION || type == CADENCE_RINGBACK)
  {
    int idP, pOn, pMinOn, pMinOff, pMaxOff, pCyc;

    if (type == CADENCE_CONGESTION)
    {
      idP = GetCongestionToneCallProgressFilterIndex, pOn = GetMaxCongestionCadenceOnTime, pMinOn = GetMinCongestionCadenceOnTime,
      pMinOff = GetMinCongestionCadenceOffTime, pMaxOff = GetMaxCongestionCadenceOffTime, pCyc = GetCongestionDetectionCyclesNumber;
    }
    else
    {
      idP = GetRingbackToneCallProgressFilterIndex, pOn = GetMaxRingbackCadenceOnTime, pMinOn = GetMinRingbackCadenceOnTime,
      pMinOff = GetMinRingbackCadenceOffTime, pMaxOff = GetMaxRingbackCadenceOffTime, pCyc = GetRingbackDetectionCyclesNumber;
    }

    filt = (int)modem_get_param(c->modem, (unsigned)idP);
    c->bufLen = 0xa0;
    c->reportTone = 0;
    c->integ = 0;
    if (DBG(1))
      dsplibs_debug_printf("Ringback index====> %d\n", filt);

    c->maxOn = (int)modem_get_param(c->modem, (unsigned)pOn);
    c->minOn = (int)modem_get_param(c->modem, (unsigned)pMinOn);
    c->minOff = (int)modem_get_param(c->modem, (unsigned)pMinOff);
    c->maxOff = (int)modem_get_param(c->modem, (unsigned)pMaxOff);
    c->maxCycles = (int)modem_get_param(c->modem, (unsigned)pCyc);

    /* an unconfigured (zero) limit disables the detector; the limits get placeholder values and create fails below */
    if (c->maxOn == 0 || c->minOn == 0 || c->maxOff == 0 || c->minOff == 0)
    {
      ok = 0;
      if (DBG(2))
        dsplibs_debug_printf(type == CADENCE_CONGESTION ? "Disable CONGESTION detector\n" : "Disable RINGBACK detector\n");
      c->maxOn = 0x37;
      c->minOn = 0x14;
      c->maxOff = 0x37;
      c->minOff = 0x14;
    }
  }
  else
  {
    /* dial tone (and any invalid type): reports every tone window (reportTone), no on / off limits, the window length and integration time
     * come from the host */
    filt = (int)modem_get_param(c->modem, GetDialToneCallProgressFilterIndex);
    sub = (int)modem_get_param(c->modem, GetDialToneFilterSubindex);
    c->bufLen = (int)modem_get_param(c->modem, GetCallProgressSamplesBufferLength);
    if (c->bufLen == 0)
    {
      if (DBG(1))
        dsplibs_debug_printf("BUFFER LENGTH is INVALID!\n");
      c->bufLen = 0x29a;
    }
    c->reportTone = 1;
    c->integ = mulw((int)modem_get_param(c->modem, GetDialToneValidationTime), 100);
    c->maxOff = 0;
    c->maxOn = 0;
  }

  /* select the coefficient set: filter index x sub-index (1 .. 7 = that row of a Filter_* set, otherwise the single-row CP_* set); see the
   * function description */
  {
    int row = (unsigned)(sub - 1) <= 6u;
    const short *fa, *fb, *fs;
    const short *fa0 = NULL, *fb0 = NULL, *fs0 = NULL;
    const short *ca = CP_350_600_a, *cb = CP_350_600_b, *cs = CP_350_600_scales;
    switch ((unsigned)filt <= 7u ? filt : 8)
    {
    case 0:
    case 2:
      if (row)
      {
        fa0 = Filter_350_500_a;
        fb0 = Filter_350_500_b;
        fs0 = Filter_350_500_scales;
      }
      break;
    case 1:
    case 4:
    case 5:
      if (row)
      {
        fa0 = Filter_100_550_a;
        fb0 = Filter_100_550_b;
        fs0 = Filter_100_550_scales;
      }
      break;
    case 3:
      if (row)
      {
        fa0 = Filter_276_504_a;
        fb0 = Filter_276_504_b;
        fs0 = Filter_276_504_scales;
      }
      else
      {
        ca = CP_276_504_a;
        cb = CP_276_504_b;
        cs = CP_276_504_scales;
      }
      break;
    case 6:
      if (row)
      {
        fa0 = Filter_100_550_a;
        fb0 = Filter_100_550_b;
        fs0 = Filter_100_550_scales;
      }
      else
      {
        ca = CP_450_630_a;
        cb = CP_450_630_b;
        cs = CP_450_630_scales;
      }
      break;
    case 7:
      if (row)
      {
        fa0 = Filter_350_500_a;
        fb0 = Filter_350_500_b;
        fs0 = Filter_350_500_scales;
      }
      else
      {
        ca = CP_100_550_a;
        cb = CP_100_550_b;
        cs = CP_100_550_scales;
      }
      break;
    default:
      break;
    }
    c->ordA = 12;
    c->ordB = 12;
    if (fa0)
    {
      fa = fa0 + 12 * (sub - 1);
      fb = fb0 + 12 * (sub - 1);
      fs = fs0 + 5 * (sub - 1);
    }
    else
    {
      fa = ca;
      fb = cb;
      fs = cs;
    }
    c->tabA = fa;
    c->tabB = fb;
    c->tabScales = fs;
  }

  /* convert the host durations to window counts; silence longer than offTimes * maxOff restarts the cycle */
  c->maxOn = scaled(c->maxOn, c->bufLen);
  c->minOn = scaled(c->minOn, c->bufLen);
  c->maxOff = scaled(c->maxOff, c->bufLen);
  c->minOff = scaled(c->minOff, c->bufLen);
  c->resetOff = mulw(offTimes, c->maxOff);

  if (DBG(1))
  {
    dsplibs_debug_printf("TYPE %s\n", cadence_names[type]);
    dsplibs_debug_printf("Filter index %d\n", filt);
    dsplibs_debug_printf("Filter SubIndex %d\n", sub);
    dsplibs_debug_printf("MAX_ON_TIME %d Buffers     MIN_ON_TIME %d Buffers\n", c->maxOn, c->minOn);
    dsplibs_debug_printf("MAX_OFF_TIME %d Buffers    MIN_OFF_TIME %d Buffers\n", c->maxOff, c->minOff);
    dsplibs_debug_printf("OFF_TIME_THAT_RESETS_CYCLE %d\n", c->resetOff);
    dsplibs_debug_printf("BUFFER LENGTH %d samples.\n", c->bufLen);
    dsplibs_debug_printf("INTEGRATION_LENGTH %d[ms]\n", c->integ);
    dsplibs_debug_printf("LEVEL %d\n", (int)c->thr);
  }

  if (c->integ > 100)
    c->integ -= 100;

  /* a disabled detector releases everything it holds, including a caller-supplied object */
  if (!ok)
  {
    if (ti)
      toneiir_delete(ti);
    sysdep_free(c);
    return NULL;
  }

  if (extra > 0)
    c->reportTone = 0;
  c->callerParamB = pw18;
  t = ptype;
  if (t > 4u)
    t = 4;

  /* build the ToneIir configuration from the values chosen above */
  cfg.a_w = tone_word(c->tabA);
  cfg.b_w = tone_word(c->tabB);
  cfg.na = c->ordA;
  cfg.nb = c->ordB;
  cfg.win = c->bufLen;
  cfg.level = c->thr;
  cfg.integ = mulw(extra + 1, c->integ);
  cfg.hold = c->reportTone;
  cfg.scales_w = tone_word(c->tabScales);
  c->name = cadence_names[t]; /* stored after cfg is built from c, so the result does not depend on the compiler's aliasing assumptions */

  /* unreachable: c->allpass was cleared at the top of the function */
  if (c->allpass)
  {
    cfg = toneiir_configuration_allpass;
    cfg.integ = c->bufLen;
  }

  /* create or re-initialise the ToneIir and start in the silence phase */
  c->ti = toneiir_create(ti, &cfg);
  c->idx = 0;
  c->state = 1;
  c->cnt = 0;
  c->scaledLimitB = scaled(c->scaledLimitB, c->bufLen);
  c->scaledLimitA = scaled(c->scaledLimitA, c->bufLen);
  c->seriesFlag = 0;
  c->callerParamA = pw0c;
  p->type = t; /* the clamped type is written back last: no store through p precedes the loads of *c above */
  return c;
}

/**
 * @brief Free a cadence detector and the ToneIir it owns.
 * @param c  detector from cadence_create() (not NULL)
 */
void cadence_delete(Cadence* c)
{
  if (c->ti)
    toneiir_delete(c->ti);
  sysdep_free(c);
}

/**
 * @brief Restart detection: reset the ToneIir and return to the silence phase with empty slots and a zero cycle count.
 * @param c  detector
 */
void cadence_reset(Cadence* c)
{
  toneiir_reset(c->ti);
  c->state = 1;
  c->cnt = 0;
  c->idx = 0;
  c->cycles = 0;
}

/**
 * @brief Feed one line sample to the cadence detector.
 * @param c  detector
 * @param x  line sample
 * @return 0 nothing to report; 1 the cadence was recognised (or, for the dial-tone detector, a tone window was seen); 7 "no answer" (more than
 *         9 cycles were counted)
 *
 * The ToneIir answers once per window: 2 = tone present, 1 = no tone.  Phase bookkeeping (`state` 0 = in a tone, 1 = in silence; `cnt` counts the
 * windows of the current phase):
 *  - tone window: in silence the silence length is stored in off[idx] and the phase becomes "tone"; in a tone `cnt` counts the tone length (but
 *    with maxOn == 0, as for the dial-tone detector, and series mode off, a tone window while already in a tone restarts in silence with empty
 *    slots and returns 1).  With reportTone set every tone window returns 1.
 *  - no-tone window: in silence `cnt` counts (a silence longer than resetOff restarts the slots); in a tone the tone length is stored in on[idx],
 *    idx advances (wrapping to 0 after slot 59), the phase becomes "silence", and storing slot 0 counts a cycle.
 * Once idx reaches maxCycles the slots are judged, with tolerance tol = max(host busy-tone difference time, 3):
 *  - series mode (seriesFlag != 0): on[0], off[1], on[1], off[2] each within tol of ser[0 .. 3], and at least seriesMin slots;
 *  - loose mode (busy tone, loose detection enabled): the latest slot within [min, max]; the older slots up to the oldest of the last maxCycles
 *    agree with it within tol (for the oldest slot: its on within tol, and its off not shorter than the latest off);
 *  - normal mode: the latest slot within [min, max] for both on and off, and either the last three slots agree within tol, or (six or more slots)
 *    slots n, n-2, n-4 agree and slots n-1, n-3, n-5 agree.
 * A successful judgement resets the ToneIir, returns to silence with empty slots and returns 1.  After more than 9 counted cycles every call
 * resets the ToneIir and the phase and returns 7.
 *
 * @bug   `cycles` is only cleared by cadence_reset(), so once it exceeds 9 every following call returns 7.
 * @bug   The slot arrays have 60 entries and are indexed by `idx` and by `n1 - maxCycles + 1` (loose mode) without a range check; a negative
 *        maxCycles from the host makes the loose-mode reads leave the arrays.
 */
int cadence_progress(Cadence* c, short x)
{
  int ret = 0, cond = 0;
  int r = toneiir_progress(c->ti, x);

  /* r == 2: tone window; r == 1: no-tone window; r == 0: mid-window, nothing to do */
  if (r == 2)
  {
    if (c->reportTone != 0)
      ret = 1;
    if (c->state == 0)
    {
      if (c->seriesFlag == 0 && c->maxOn == 0)
      {
        c->state = 1;
        c->cnt = 0;
        ret = 1;
        c->idx = 0;
      }
      else
      {
        c->cnt = c->cnt + 1;
      }
    }
    else if (c->state == 1)
    {
      int v = c->cnt;
      c->state = 0;
      c->cnt = 0;
      c->off[c->idx] = v;
    }
  }
  else if (r == 1)
  {
    if (c->state == 1)
    {
      c->cnt = c->cnt + 1;
      if (c->cnt > c->resetOff)
        c->idx = 0;
    }
    else if (c->state == 0)
    {
      int old = c->idx;
      c->on[old] = c->cnt;
      c->idx = old + 1;
      if (old == 0)
      {
        c->cycles = c->cycles + 1;
        if (DBG(1))
          dsplibs_debug_printf("CYCLES_COUNTER= %d\n", c->cycles);
      }
      /* enough slots collected: judge the cadence (series, normal or loose mode) */
      if (c->idx >= c->maxCycles)
      {
        int tol = (int)modem_get_param(c->modem, GetBusyToneDiffTime);
        int n1 = c->idx - 1;

        if (tol <= 2)
          tol = 3;

        if (c->seriesFlag != 0)
        {
          if (DBG(1))
            dsplibs_debug_printf(" CADENCE SERIRES COMPARISON ========================>\n");
          if (c->seriesMin <= n1 && absd(c->on[0], c->ser[0]) <= tol && absd(c->off[1], c->ser[1]) <= tol && absd(c->on[1], c->ser[2]) <= tol &&
              absd(c->off[2], c->ser[3]) <= tol)
          {
            cond = 1;
            if (DBG(2))
              dsplibs_debug_printf("CADENCE %s: CONDITION -- SERIES --- SATISFIED\n", c->name);
          }
        }
        else if (c->loose == 0)
        {
          if (c->maxCycles - 1 <= n1 && n1 > 1)
          {
            int offn = c->off[n1], onn = c->on[n1];
            if (offn >= c->minOff && offn <= c->maxOff && onn >= c->minOn && onn <= c->maxOn)
            {
              int offp = c->off[n1 - 1], fB, fC = 0;
              fB = absd(offn, offp) < tol && absd(offn, c->off[n1 - 2]) < tol && absd(onn, c->on[n1 - 1]) < tol && absd(onn, c->on[n1 - 2]) < tol;
              if (n1 > 5)
                fC = absd(offn, c->off[n1 - 2]) < tol && absd(offn, c->off[n1 - 4]) < tol && absd(onn, c->on[n1 - 2]) < tol && absd(onn, c->on[n1 - 4]) < tol &&
                     absd(offp, c->off[n1 - 3]) < tol && absd(offp, c->off[n1 - 5]) < tol && absd(c->on[n1 - 1], c->on[n1 - 3]) < tol &&
                     absd(c->on[n1 - 1], c->on[n1 - 5]) < tol;
              if (fB || fC)
              {
                cond = 1;
                if (DBG(2))
                  dsplibs_debug_printf("CADENCE %s: CONDITION B SATISFIED\n", c->name);
              }
            }
          }
        }
        else
        {
          int N = c->maxCycles;
          if (N - 1 <= n1)
          {
            int offn = c->off[n1], onn = c->on[n1], k;
            if (offn >= c->minOff && offn <= c->maxOff && onn >= c->minOn && onn <= c->maxOn)
              cond = 1;
            if (N > 2)
              for (k = 1; k < N - 1; k++)
                if (!(absd(offn, c->off[n1 - k]) < tol && absd(onn, c->on[n1 - k]) < tol))
                  cond = 0;
            if (offn > c->off[n1 - N + 1])
              cond = 0;
            else if (absd(onn, c->on[n1 - N + 1]) >= tol)
              cond = 0;
            else if (cond == 1 && DBG(2))
              dsplibs_debug_printf("CADENCE %s: CONDITION C SATISFIED\n", c->name);
          }
        }
      }

      /* the slot arrays are full: wrap */
      if (c->idx > 0x3b)
        c->idx = 0;
      c->cnt = 0;
      c->state = 1;

      if (cond == 1)
      {
        if (DBG(1))
          dsplibs_debug_printf("BUSY cadence recognized\n");
        toneiir_reset(c->ti);
        c->state = 1;
        c->cnt = 0;
        ret = 1;
        c->idx = 0;
      }
    }
  }
  /* too many cycles without a recognised cadence */
  if ((unsigned)c->cycles > 9u)
  {
    if (DBG(1))
      dsplibs_debug_printf("NO ANSWER state recognized\n");
    toneiir_reset(c->ti);
    ret = 7;
    c->state = 1;
    c->cnt = 0;
    c->idx = 0;
  }
  return ret;
}
