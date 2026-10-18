/**
 * @file tone_iir.c
 * @brief ToneIir: a four-section biquad band-pass tone detector that works one sample per call over a window of cfg.win samples, plus the two
 *        fixed-point helpers its users need (GetFP_Value, Get_Detection_Threshold_Table).
 *
 * The object and its configuration block are declared in tone_detect.h; cadence_create() (tone_cadence.c) is the only creator in the library and
 * cadence_progress() the only caller of toneiir_progress().
 *
 * toneiir_progress(t, x), per sample: the input is shifted right by scales[0] into the history w of section 0; every section computes
 *   acc = sum(k = 0..2) w[k] * b[k] - sum(k = 1..2) y[k-1] * a[k],  out = acc >> 13   (a, b: 3 coefficients per section, 3 shorts further for the next section)
 * shifts w / y, and hands out >> scales[s + 1] to the next section (the last output is shifted by scales[4] when that is positive).
 * Two leaky energies are kept:
 *   e1 = |out * 164 >> 14| + |e1 * 0x3f5c >> 14|  (filtered signal), e0 = the same for the raw input.  After cfg.win samples the window ends:
 *   cond = e1 >= e0 / 4 && e1 >= (p * ratio) >> 14 && p >= (e1 * ratio) >> 14   (p = e1 of the previous window), det = cond && e1 >= level; the hit counters
 *   (hits / run, cfg.miss / cfg.hold) are updated and the result is 2 (tone: hits > integThr && det) or 1; e1 becomes p and the window restarts.
 * Arithmetic details that matter: the shifts use the count modulo 32 (as the x86 `sar` does), and the 32-bit sums wrap (they are computed unsigned).
 */
#include "tone_detect.h"
#include "core/dsp_host.h"

#define DBG(n) (dsplibs_debug_level > (n))

/* wrapping 32-bit add and absolute value (computed unsigned so that overflow wraps instead of being undefined) */
static int addw(int a, int b) { return (int)((unsigned)a + (unsigned)b); }
#include "core/dsp_mulw.h"
static int absi(int v) { return v < 0 ? (int)(0u - (unsigned)v) : v; }

/**
 * @brief Fixed-point ratio a / b in Q14, rounded up in magnitude.
 * @param a  numerator
 * @param b  denominator
 * @return ceil(|a| * 16384 / |b|) truncated to 16 bits, negated unless a * b > 0 (so 0 when a == 0)
 * @note  Used to turn a window length into a Q14 reciprocal (GetFP_Value(1, n) is about 16384 / n).
 * @warning b == 0 with a != 0 returns 0 here.
 */
short GetFP_Value(short a, short b)
{
  int sa = a < 0 ? -(int)a : (int)a, sb = b < 0 ? -(int)b : (int)b;
  int top = sa << 14, n = 0;

  if (top > 0)
  {
    if (sb == 0)
      return 0; /* division by zero: no meaningful quotient */
    n = (top + sb - 1) / sb;
  }
  return (short)((int)a * (int)b > 0 ? n : -n);
}

/**
 * @brief Detection threshold for a configured detection level.
 * @param level  detection level as configured by the host (tone_thr_window[] covers 0 .. 255)
 * @return entry (45 - level) of the 16-entry threshold table (90 ... 560) for level 30 .. 45; the same index into tone_thr_window[] for the rest
 *         of 0 .. 255, which holds values that are not thresholds (a decaying run near 16384 below level 30, filter-coefficient-like words
 *         above 45); 0 for a level outside 0 .. 255
 * @bug   Levels outside 30 .. 45 index past the 16-entry threshold table: tone_thr_window[] reproduces the memory that happens to follow and
 *        precede the table, so the result is not a threshold.  The host parameter is not range-checked by cadence_create().
 */
short Get_Detection_Threshold_Table(short level)
{
  int idx = (short)(0x2d - level);

  if (idx < -210 || idx > 45)
    return 0;
  return tone_thr_window[idx + 210];
}

/**
 * @brief Copy the default configuration block (all-zero coefficient tables, 500-sample window, ratio 0.7 in Q14, level 80, integration 2200, miss 4).
 * @param out  receives the block
 */
void toneiir_get_default_configuration(ToneIirCfg* out) { *out = toneiir_default_cfg; }

/**
 * @brief Create or re-initialise a tone detector.
 * @param t    object to initialise, or NULL to allocate one with sysdep_malloc()
 * @param cfg  configuration block (copied), or NULL for the default configuration
 * @return the initialised detector, or NULL if the allocation failed
 *
 * The state histories (nb entries of w, na of y; both counts clamped to 25) and the window counters are cleared, and the integration threshold
 * is set to cfg.integ scaled by 8 / win (in windows).  At debug level 2 the threshold is printed.
 * @bug   `p` (the previous-window energy) is loaded from the object's old `e1` before it is cleared.  For an object allocated here that memory is
 *        uninitialised, so the first window is compared against an arbitrary `p`.  A caller-supplied object should be zeroed.
 */
ToneIir* toneiir_create(ToneIir* t, const ToneIirCfg* cfg)
{
  short oldE1;
  int i, g;

  if (!t)
  {
    t = (ToneIir*)sysdep_malloc(sizeof(ToneIir));
    if (!t)
      return NULL;
  }
  if (!cfg)
    cfg = &toneiir_default_cfg;

  /* configuration, with the history counts clamped */
  t->cfg = *cfg;
  if (t->cfg.nb > 0x18)
    t->cfg.nb = 0x19;
  if (t->cfg.na > 0x18)
    t->cfg.na = 0x19;

  /* clear the state; p takes the old e1 */
  oldE1 = t->e1;
  t->cnt = 0;
  t->e1 = 0;
  t->p = oldE1;
  t->e0 = 0;
  for (i = 0; i < t->cfg.nb; i++)
    t->w[i] = 0;
  for (i = 0; i < t->cfg.na; i++)
    t->y[i] = 0;

  /* integration threshold in windows */
  g = GetFP_Value(8, (short)t->cfg.win);
  t->integThr = mulw(g, t->cfg.integ) >> 14;
  if (DBG(1))
    dsplibs_debug_printf("INTEGRATION_TIME = %d Buffers.\n", t->integThr);
  t->hits = 0;
  t->run = 0;
  return t;
}

/**
 * @brief Free a detector allocated by toneiir_create().
 * @param t  detector (passed straight to sysdep_free())
 */
void toneiir_delete(ToneIir* t) { sysdep_free(t); }

/**
 * @brief Restart detection: clear the window counters, the energies and the hit counters, keeping the filter histories.
 * @param t  detector
 * @note  The previous-window energy `p` is set to the current `e1`.
 */
void toneiir_reset(ToneIir* t)
{
  short e1 = t->e1;
  t->e1 = 0;
  t->cnt = 0;
  t->p = e1;
  t->e0 = 0;
  t->hits = 0;
  t->run = 0;
}

/**
 * @brief Feed one sample to the detector.
 * @param t  detector
 * @param x  input sample
 * @return 0 in the middle of a window; at a window end 2 if a tone is present (enough consecutive qualifying windows) or 1 if not
 * @note  The coefficient and scale tables are resolved from the table words of cfg (tone_ptr()); a/b must hold 12 shorts and the scales 5.
 *        The filter, the energies and the window decision are described in the file header.
 * @warning A window length cfg.win <= 0 never ends (the window ends only when the sample count reaches cfg.win exactly).  A configuration
 *        whose scales table is NULL (toneiir_configuration_allpass) dereferences NULL here.
 */
int toneiir_progress(ToneIir* t, short x)
{
  const short *a = tone_ptr(t->cfg.a_w), *b = tone_ptr(t->cfg.b_w), *sc = tone_ptr(t->cfg.scales_w);
  int res = 0, s, k, v, e1n, e0n, t1, t2;
  short e1s, e0s, pp, ratio;
  int cond;

  /* the four cascaded biquad sections */
  v = x;
  for (s = 0; s < 4; s++)
  {
    short *w = t->w + 3 * s, *y = t->y + 2 * s;
    const short *as = a + 3 * s, *bs = b + 3 * s;
    int acc = 0;
    short out;
    w[0] = (short)(v >> (sc[s] & 31));
    for (k = 0; k < 3; k++)
      acc = addw(acc, (int)w[k] * (int)bs[k]);
    for (k = 1; k < 3; k++)
      acc = addw(acc, -((int)y[k - 1] * (int)as[k]));
    out = (short)(acc >> 13);
    w[2] = w[1];
    w[1] = w[0];
    y[1] = y[0];
    y[0] = out;
    v = out;
  }
  if (sc[4] > 0)
    v >>= (sc[4] & 31);
  v = (short)v;

  /* leaky energy of the filtered signal and of the raw input */
  t1 = absi(((int)v * 164) >> 14);
  t2 = absi((mulw(t->e1, 0x3f5c)) >> 14);
  e1n = t1 + t2;
  t->e1 = (short)e1n;
  t1 = absi(((int)x * 164) >> 14);
  t2 = absi((mulw(t->e0, 0x3f5c)) >> 14);
  e0n = t1 + t2;
  if (t->cnt + 1 != t->cfg.win)
  {
    t->cnt = t->cnt + 1;
    t->e0 = (short)e0n;
    return 0;
  }
  /* window end: decide whether the window qualifies, update the hit counters, start the next window */
  t->cfg.mode = 2;
  e1s = (short)e1n;
  e0s = (short)e0n;
  pp = t->p;
  ratio = t->cfg.ratio;
  cond = e1s >= (e0s >> 2) && e1s >= (((int)pp * (int)ratio) >> 14) && pp >= (((int)e1s * (int)ratio) >> 14);
  cond = cond && e1s >= t->cfg.level;
  /* hit counters: a qualifying window extends the run; a miss that ends a long enough run still counts one hit; otherwise (hold == 0) forget */
  if (cond)
  {
    t->run++;
    t->hits++;
  }
  else if (t->run > t->cfg.miss)
  {
    t->hits++;
    t->run = 0;
  }
  else if (t->cfg.hold == 0)
  {
    t->run = 0;
    t->hits = 0;
  }
  res = (t->hits > t->integThr && cond) ? 2 : 1;
  t->cnt = 0;
  t->e1 = 0;
  t->e0 = 0;
  t->p = e1s;
  return res;
}
