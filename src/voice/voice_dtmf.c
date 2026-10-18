/**
 * @file voice_dtmf.c
 * @brief The voice DTMF detector (create_dtmf / dtmf_progress / dtmf_detect / dtmf_test) and the beep / DTMF tone generator (beepgen_*).
 *
 * Types: voice_dtmf.h.  Coefficient tables (us_coef, eur_coef, biascoef): voice_dtmf_tables.c.  Users: the detector of voice_tone.c (detector_progress()
 * owns a Dtmf object and calls dtmf_progress()) and the voice engine (voice_engine.c: one BeepGen, driven through beepgen_start_beep() /
 * beepgen_start_dtmf() and sampled with beepgen_sample() once per output sample).
 *
 * Detector.  dtmf_detect() takes every second input sample (4 kHz), runs eight second-order notch filters (the notch() of voice_tone.c, 4 coefficients
 * each) over it and accumulates the energy of each filter's output and of the input; after 40 processed samples (mode 1: 45, with a pre-filter notch
 * on the input) dtmf_test() decides.  The notch filters REJECT their tone, so the tone of a group is the filter with the minimum energy.  A digit must
 * be seen three blocks in a row (after four different results) before the detector holds it; it is reported when the tone ends.
 *
 * Generator.  A BeepGen plays a queue of up to BEEP_QMAX entries {f1, f2, duration}.  The first queued entry starts at once; beepgen_sample() produces
 * sin(phase1) * gain1 + cos(phase2) * gain2 per call and starts the next entry when the current one has run its duration (dur * 8000 / div samples,
 * div 10 for a tone, 100 for the hook marker f1 = -1).  Digits map to the DTMF frequency pairs; '!' is a hook marker whose duration comes from the
 * parameter callback (id 0x18), ',' a pause of three times the duration.
 *
 * Float policy (as voice_tone.c): every intermediate is a `long double` (the x87 format on i386 and x86-64 whatever -O or -mfpmath); a value is rounded
 * to float only where it is stored into a float (F()); `fadd dword [m]`, `fmul dword [m]`, fsin and fcos are inline asm.  dtmf_test() only compares
 * extended intermediates (ordered comparisons are false for NaN, so a NaN energy rejects the block).  Float values are moved between objects as
 * 32-bit integers (copy_f32()) so that the payload of a signalling NaN frequency survives.
 *
 * Behaviour callers must know: beepgen_create() stores the gains from voice_getgain() the other way round from every later start (harmless: the first
 * start overwrites them), and leaves n, limit, div and the queue uninitialised.  A '!' digit with no parameter callback keeps the duration it was given.
 * beepgen_create() with cfg == NULL is undefined.  An entry beyond BEEP_QMAX queued entries is ignored.
 */
#include <limits.h>
#include <math.h>
#include <string.h>

#include "voice_dtmf.h"
#include "core/dsp_asm.h"

#include "voice_f32.h"
#include "voice_x87.h"
#include "core/dsp_constants.h" /* SAMPLE_RATE_* */

/* the x87 transcendental instructions (hardware results: bit-identical on i386 and x86-64 for the same CPU; an operand beyond 2^63 is left unchanged) */
static inline long double x87_fsin(long double a)
{
#if defined(__i386__) || defined(__x86_64__)
  X87_SIN(a);
  return a;
#else
  return sinl(a);
#endif
}
static inline long double x87_fcos(long double a)
{
#if defined(__i386__) || defined(__x86_64__)
  X87_COS(a);
  return a;
#else
  return cosl(a);
#endif
}
/* a float-to-float copy through an INTEGER register: a C float copy may go through the x87 and quiet a signalling NaN at -m32 */
static inline void copy_f32(float* dst, const float* src)
{
  uint32_t u;
  memcpy(&u, src, 4);
  memcpy(dst, &u, 4);
}

#define DBG_LEVEL() (dsplibs_debug_level)

/* ---- constants ---- */
/* phase step per sample and Hz, 7.8494e-4 (not the pi / 4000 = 7.8540e-4 of the tone detector's phasor) */
static const double k_beep_step = 0x1.9b90ea9e6eeb7p-11;
static const float k_m1 = -1.0f;                    /* the hook marker frequency */
static const float k_th1 = 0x1.205bc00000000p-9f;   /* 0.0022: mode 1 input energy threshold */
static const float k_th0 = 0x1.0624dc0000000p-9f;   /* 0.002: mode 0 input energy threshold */
static const float k_sixth = 0x1.5555560000000p-3f; /* 1/6: mean of the six other filters */
static const float k_794 = 0x1.fc28f60000000p+2f;   /* 7.94: twist limit, low group stronger */
static const float k_282 = 0x1.68f5c20000000p+1f;   /* 2.82: twist limit, high group stronger */
static const float k_096 = 0x1.eb851e0000000p-1f;   /* 0.96: separation of the group maximum from the minimum */
static const float k_136 = 0x1.5c28f60000000p+0f;   /* 1.36: upper bound of the other filters against the two minima */
static const float k_079 = 0x1.947ae20000000p-1f;   /* 0.79: lower bound of the other filters against the two minima */

/* ---- DTMF detector ---- */

/**
 * @brief Create or re-initialise a DTMF detector.
 * @param d  an existing detector to reset, or NULL to allocate one
 * @return the detector, or NULL when the allocation failed (a message is printed at debug level 2)
 * @note  The filter states, the energies, the history (USHRT_MAX = none) and the held digit are reset; the unused bytes at 0x80 are left alone.
 */
Dtmf* create_dtmf(Dtmf* d)
{
  int i;
  if (!d)
  {
    d = (Dtmf*)sysdep_malloc(0x98);
    if (!d)
    {
      if (DBG_LEVEL() > 1)
        dsplibs_debug_printf("could not allocate DTMF channel\n");
      return NULL;
    }
  }
  for (i = 0; i < 8; i++)
  {
    d->st[i][0] = 0.0f;
    d->st[i][1] = 0.0f;
    d->e[i] = 0.0f;
  }
  for (i = 0; i < 2; i++)
    d->bias[i] = 0.0f;
  for (i = 0; i < 8; i++)
    d->hist[i] = USHRT_MAX;
  d->eb = 0.0f;
  d->sub = 0;
  d->cnt = 0;
  d->busy = 0;
  d->code = USHRT_MAX;
  d->easy = 0;
  return d;
}

/**
 * @brief Decide whether the eight accumulated filter energies of a block show a DTMF digit.
 * @param e     output energies of the eight notch filters (e[0..3] low group, e[4..7] high group)
 * @param eb    input energy of the block
 * @param mode  1 = the other national plan (a slightly higher input threshold), else 0
 * @return the digit, low index + 4 * high index (0 .. 15), or -1
 * @note  The notch filters reject their tone, so the tone of a group is the filter with the minimum energy.  The block is accepted when the input
 *        energy reaches the threshold, the two minima are within a twist window of each other (7.94 and 2.82), each is below 0.96 of its group's
 *        maximum, and the mean of the other six filters lies between 0.79 / 1.36 times the sum of the two minima.  A NaN anywhere rejects the block.
 */
int dtmf_test(const float* e, float eb, short mode)
{
  static const uint32_t big_bits = 0x7e967699u; /* the "no minimum yet" start value (a float, 1.0e38) */
  float big;
  long double th, smin, m1, m2, sum, e_hi, e_lo, ebv, t, x, z;
  int loIdx = 0, hiIdx = 0, i, c;
  memcpy(&big, &big_bits, 4);
  th = (mode == 1) ? LD(k_th1) : LD(k_th0);

  /* the minimum (its index) and the maximum of the low group */
  smin = LD(big);
  m1 = 0.0L;
  for (i = 0; i < 4; i++)
  {
    long double v = ldl(&e[i]);
    if (!(v >= smin))
    { /* update when below or unordered */
      smin = v;
      loIdx = i;
    }
    if (v > m1)
      m1 = v;
  }

  /* the same for the high group */
  smin = LD(big);
  m2 = 0.0L;
  for (i = 0; i < 4; i++)
  {
    long double v = ldl(&e[4 + i]);
    if (!(v >= smin))
    {
      smin = v;
      hiIdx = i + 4;
    }
    if (v > m2)
      m2 = v;
  }

  sum = 0.0L;
  for (i = 0; i < 8; i++)
    sum = ld_addf(sum, &e[i]);
  e_hi = ldl(&e[hiIdx]);
  e_lo = ldl(&e[loIdx]);
  ebv = ldl(&eb);
  if (!(ebv >= th))
    return -1; /* input too weak (or NaN) */

  /* t = mean of the six other filters; c = all the conditions that must hold */
  t = ((sum - e_hi) - e_lo) * LD(k_sixth);
  c = (e_lo * LD(k_794) >= e_hi);
  c &= (e_hi * LD(k_282) >= e_lo);
  m1 = m1 * LD(k_096);
  m2 = m2 * LD(k_096);
  x = t * LD(k_136);
  c &= (m1 >= e_lo);
  z = e_lo + e_hi;
  c &= (m2 >= e_hi);
  c &= (x >= z);
  if (t * LD(k_079) > z)
    return -1;
  if (!c)
    return -1;
  return (short)(loIdx + hiIdx * 4 - 0x10);
}

/**
 * @brief Feed one input sample to the detector.
 * @param x     input sample
 * @param d     the detector
 * @param mode  1 = the other national plan (45-sample blocks, pre-filter, eur_coef), else 0 (40 samples, us_coef)
 * @return -2 while nothing was evaluated (only every second sample is processed and a block takes 40 / 45 of them), -1 when a block was evaluated
 *         without a digit to report, or the digit (0 .. 15) when a held tone has ended
 * @note  A digit is held after the same non-empty result three blocks in a row that differs from the four blocks before; the digit is reported with the
 *        first block that shows no digit.  `d->easy` (never set in the library) would relax the condition to two blocks.
 */
int dtmf_detect(float x, Dtmf* d, short mode)
{
  int ret = -2, m = mode, i;
  unsigned short sub = (unsigned short)(d->sub + 1);
  unsigned short cnt, lim, code;
  const float* coef;

  /* decimate by two */
  if (sub != 2)
  {
    d->sub = sub;
    return ret;
  }
  d->sub = 0;

  /* mode 1 pre-filter, then the energies of the input and of the eight filters */
  if (m == 1)
    x = F(notch(x, d->bias, biascoef)); /* the pre-filter's unrounded result is rounded to float */
  {
    long double xs = ldl(&x);
    d->eb = F(ld_addf(xs * xs, &d->eb));
  }
  coef = (m == 1) ? eur_coef : us_coef;
  for (i = 0; i < 8; i++)
  {
    long double y = notch(x, d->st[i], coef + 4 * i);
    d->e[i] = F(ld_addf(y * y, &d->e[i]));
  }
  cnt = (unsigned short)(d->cnt + 1);
  d->cnt = cnt;
  lim = (unsigned short)((m == 1) ? 45 : 40);
  if (cnt != lim)
    return ret;

  /* a block is complete: decide, shift the result into the history */
  for (i = 7; i > 0; i--)
    d->hist[i] = d->hist[i - 1];
  code = (unsigned short)dtmf_test(d->e, d->eb, (short)m);
  d->hist[0] = code;
  d->cnt = 0;
  if (d->busy == 0)
  {
    if (code == USHRT_MAX)
    {
      ret = -1;
    }
    else
    {
      const unsigned short* h = d->hist;
      /* the same result three blocks in a row, different from the four before */
      if (code == h[1] && code == h[2] && code != h[3] && code != h[4] && code != h[5] && code != h[6])
      {
        ret = -1;
        d->busy = 1;
        d->code = code;
      }
      if (d->easy != 0)
      {
        /* relaxed validation: two blocks in a row, different from the two before */
        if (code == h[1] && code != h[2] && code != h[3])
        {
          d->busy = 1;
          d->code = code;
          ret = -1;
        }
      }
    }
  }
  else if ((unsigned short)(code + 1u) == 0)
  { /* no digit any more: the tone is gone, the held digit is reported */
    ret = (short)d->code;
    d->busy = 0;
    d->code = USHRT_MAX;
  }
  else
  {
    ret = -1;
  }

  /* start the next block */
  for (i = 0; i < 8; i++)
    d->e[i] = 0.0f;
  d->eb = 0.0f;
  return ret;
}

/**
 * @brief Run the detector over a block of samples.
 * @param d      the detector
 * @param x      input samples
 * @param count  number of samples
 * @param mode   0 or 1, see dtmf_detect()
 * @return the last digit reported by dtmf_detect() during the block (0 .. 15), else -1
 */
short dtmf_progress(Dtmf* d, const float* x, short count, int mode)
{
  int ret = -1, i;
  for (i = 0; i < (int)count; i++)
  {
    int r = (short)dtmf_detect(x[i], d, (short)mode);
    if ((unsigned)(r + 2) > 1u)
      ret = r; /* -2 / -1 are ignored */
  }
  return (short)ret;
}

/* ---- beep / DTMF generator ---- */

/**
 * @brief Create a beep generator, or re-initialise an existing one.
 * @param b    an existing generator, or NULL to allocate one
 * @param cfg  the owner's context and the hook and parameter callbacks (copied)
 * @return the generator, or NULL when the allocation failed
 * @note  The oscillators and the queue are reset (no entry queued); the gains are set from voice_getgain().  `n`, `limit`, `div` and the queue entries
 *        are not initialised.
 * @warning `cfg` is not checked for NULL.
 */
BeepGen* beepgen_create(BeepGen* b, const BeepCfg* cfg)
{
  unsigned lvl = DBG_LEVEL();
  if (lvl > 1)
    dsplibs_debug_printf("beepgen_create\n");
  struct modem* c_ctx;
  BeepHookFn c_11c, c_120;
  VoiceParamFn c_124;
  if (!b)
  {
    b = (BeepGen*)sysdep_malloc(sizeof(BeepGen));
    if (!b)
      return NULL;
  }
  /* the configuration is read up front, so the code does not depend on the aliasing rules between these loads and the float stores below */
  c_ctx = cfg->ctx;
  c_11c = cfg->hookOn;
  c_120 = cfg->hookOff;
  c_124 = cfg->getParam;
  COMPILER_BARRIER();
  b->f1 = 0.0f;
  b->f2 = 0.0f;
  b->ctx = c_ctx;
  b->phase1 = 0.0f;
  b->phase2 = 0.0f;
  COMPILER_BARRIER(); /* keeps the order of the float stores and the pointer / int accesses below whatever -fno-strict-aliasing does */

  /* initial gains (note the argument order: later starts pass gain2, gain1) */
  voice_getgain(b, &b->gain1, &b->gain2);
  lvl = DBG_LEVEL(); /* read after the gain call, before the stores below */
  COMPILER_BARRIER();
  b->count = 0;
  b->cur = 0;
  COMPILER_BARRIER();
  b->hookOn = c_11c;
  b->getParam = c_124;
  b->hookOff = c_120;
  if (lvl > 1)
    dsplibs_debug_printf("%X  %X\n", (unsigned)(uintptr_t)b->hookOn, (unsigned)(uintptr_t)b->hookOff);
  return b;
}

/** @brief Free a beep generator. */
void beepgen_delete(BeepGen* b) { sysdep_free(b); }

/**
 * Queue one entry {f1, f2, dur} (the frequencies are converted to float) for beepgen_start_beep() and beepgen_start_dtmf().
 * An empty queue starts the entry at once: frequencies, gains (0 for a frequency of 0, and for the hook marker f1 = -1, which also calls the hook-on
 * callback and uses the slow time base div = 100), and the length dur * 8000 / div samples.  An entry that does not fit the queue is dropped.
 */
static void beep_queue(BeepGen* b, int f1, int f2, int dur)
{
  float hf1 = F(LD(f1)), hf2 = F(LD(f2));
  int count = b->count;
  if (count == 0)
  {
    b->div = 10;
    b->f1 = hf1;
    b->f2 = hf2;
    voice_getgain(b, &b->gain2, &b->gain1);
    if (f1 == 0)
    {
      b->gain1 = 0.0f;
    }
    else if (f1 == -1)
    {
      b->div = 100;
      b->gain1 = 0.0f;
      if (b->hookOn)
        b->hookOn(b->ctx);
    }
    if (f2 == 0)
      b->gain2 = 0.0f;
    b->n = 0;
    b->cur = 0;
    b->limit = (int)((unsigned)dur * SAMPLE_RATE_8000) / b->div; /* the product wraps in 32 bits */
    count = b->count;                                            /* reloaded: the hook callback may have changed it */
  }
  if (count < 0 || count >= BEEP_QMAX)
    return; /* the queue is full: the entry is dropped */
  b->q[count].dur = dur;
  b->q[count].f1 = hf1;
  b->count = count + 1;
  b->q[count].f2 = hf2;
}

/**
 * @brief Queue a two-tone beep.
 * @param b    the generator
 * @param f1   first frequency in Hz (0 = silent, -1 = hook marker)
 * @param f2   second frequency in Hz
 * @param dur  duration; the entry lasts dur * 800 samples (dur * 80 for the hook marker)
 */
void beepgen_start_beep(BeepGen* b, int f1, int f2, int dur)
{
  if (DBG_LEVEL() > 1)
    dsplibs_debug_printf("start beep %d %d %d\n", f1, f2, dur);
  beep_queue(b, f1, f2, dur);
}

/* the digit table: {row, column} of (ch - 0x21); anything not listed is {0, 0} (the pair of '1'); {5, 5} ('!') is the hook marker pair {-1, -1} */
static const unsigned char dtmf_rc[0x24][2] = {
    [0x21 - 0x21] = {5, 5}, [0x23 - 0x21] = {3, 2}, [0x2a - 0x21] = {3, 0}, [0x30 - 0x21] = {3, 1}, [0x32 - 0x21] = {0, 1}, [0x33 - 0x21] = {0, 2},
    [0x34 - 0x21] = {1, 0}, [0x35 - 0x21] = {1, 1}, [0x36 - 0x21] = {1, 2}, [0x37 - 0x21] = {2, 0}, [0x38 - 0x21] = {2, 1}, [0x39 - 0x21] = {2, 2},
    [0x41 - 0x21] = {0, 3}, [0x42 - 0x21] = {1, 3}, [0x43 - 0x21] = {2, 3}, [0x44 - 0x21] = {3, 3},
};
static const int dtmf_low[6] = {697, 770, 852, 941, 0, -1};
static const int dtmf_high[6] = {1209, 1336, 1477, 1633, 0, -1};

/**
 * @brief Queue a DTMF digit.
 * @param b    the generator
 * @param ch   '0'-'9', '*', '#', 'A'-'D' (their frequency pair), '!' (hook marker; the duration is read from the parameter callback, id 0x18, when there
 *             is one), ',' (a pause of three times `dur`); any other character plays the pair of '1'
 * @param dur  duration, as for beepgen_start_beep()
 */
void beepgen_start_dtmf(BeepGen* b, int ch, int dur)
{
  unsigned lvl, idx;
  int row = 0, col = 0, high, low;
  if (ch == 0x21 && b->getParam)
    dur = (int)b->getParam(b->ctx, 0x18);
  lvl = DBG_LEVEL();
  if (lvl > 1)
  {
    dsplibs_debug_printf("beepgen start dtmf %d %d\n", ch, dur);
    lvl = DBG_LEVEL();
  }
  idx = (unsigned)(ch & 0xff) - 0x21u; /* table index; characters below '!' wrap and fall outside the table */
  if (idx <= 0x23u)
  {
    row = dtmf_rc[idx][0];
    col = dtmf_rc[idx][1];
  }
  if (idx == 0 && lvl > 1)
  {
    dsplibs_debug_printf("start !!!!!!!!!!!!!!!!!!!!!!!!\n");
    lvl = DBG_LEVEL();
  }
  high = dtmf_high[col];
  low = dtmf_low[row];
  if (ch == 0x2c)
  {
    if (lvl > 1)
    {
      dsplibs_debug_printf("\n *** digit ',' => pause");
      lvl = DBG_LEVEL();
    }
    high = 0;
    low = 0;
    dur = (int)((unsigned)dur * 3u); /* a pause lasts three times the duration */
  }
  if (lvl > 1)
    dsplibs_debug_printf("start beep %d %d %d\n", high, low, dur);
  beep_queue(b, high, low, dur);
}

/**
 * @brief Produce the next output sample and advance the queue.
 * @param b    the generator (an entry must have been queued)
 * @param out  receives sin(phase1) * gain1 + cos(phase2) * gain2
 * @return 0, or 1 when the last queued entry has just ended (the queue is then empty)
 * @note  When an entry has run its length the hook-off callback is called if its f1 was the hook marker, and the next entry (if any) is started the
 *        same way beep_queue() starts the first one.
 */
int beepgen_sample(BeepGen* b, float* out)
{
  long double s, c, step, t1, t2, v;
  int n, idx;

  /* the sample and the oscillator phases */
  s = x87_fsin(ldl(&b->phase1));
  c = x87_fcos(ldl(&b->phase2));
  n = b->n + 1;
  b->n = n;
  *out = F(ld_mulf(s, &b->gain1) + ld_mulf(c, &b->gain2));
  step = LD(k_beep_step);
  v = ldl(&b->f1);
  t1 = step * v;
  t2 = ld_mulf(step, &b->f2);
  b->phase1 = F(ld_addf(t1, &b->phase1));
  b->phase2 = F(ld_addf(t2, &b->phase2));
  if (n <= b->limit)
    return 0;

  /* the entry has run its length */
  if (DBG_LEVEL() > 1)
  {
    dsplibs_debug_printf("End of BEEP\n");
    v = ldl(&b->f1);
  }
  if (!(v < LD(k_m1)) && !(v > LD(k_m1)))
  { /* f1 was the hook marker (or NaN) */
    if (DBG_LEVEL() > 1)
      dsplibs_debug_printf("Hook on proc\n");
    if (b->hookOff)
      b->hookOff(b->ctx);
  }

  /* the next entry, or the end of the queue */
  n = b->cur + 1;
  b->cur = n;
  if (n >= b->count)
  {
    b->count = 0;
    return 1;
  }
  if (DBG_LEVEL() > 1)
    dsplibs_debug_printf("Start beep[%02d]\n", n);
  n = b->cur;
  b->div = 10;
  copy_f32(&b->f1, &b->q[n].f1);
  copy_f32(&b->f2, &b->q[n].f2);
  voice_getgain(b, &b->gain2, &b->gain1);
  idx = b->cur;
  {
    long double a = ldl(&b->q[idx].f1);
    if (!(a < 0.0L) && !(a > 0.0L))
    { /* equal to 0 or unordered */
      b->gain1 = 0.0f;
    }
    else
    {
      long double a2 = ldl(&b->q[idx].f1);
      if (!(a2 < LD(k_m1)) && !(a2 > LD(k_m1)))
      {
        b->gain1 = 0.0f;
        b->div = 100;
        if (b->hookOn)
        {
          b->hookOn(b->ctx);
          idx = b->cur; /* reloaded: the callback may have changed it */
        }
      }
    }
  }
  {
    long double a = ldl(&b->q[idx].f2);
    if (!(a < 0.0L) && !(a > 0.0L))
      b->gain2 = 0.0f;
  }
  b->n = 0;
  b->limit = (int)((unsigned)b->q[idx].dur * SAMPLE_RATE_8000) / b->div;
  return 0;
}

#if UINTPTR_MAX == 0xffffffffu
_Static_assert(sizeof(BeepGen) == 0x12c && offsetof(BeepGen, phase1) == 4 && offsetof(BeepGen, phase2) == 8 && offsetof(BeepGen, f1) == 0xc &&
                   offsetof(BeepGen, f2) == 0x10 && offsetof(BeepGen, gain1) == 0x14 && offsetof(BeepGen, gain2) == 0x18 && offsetof(BeepGen, n) == 0x1c &&
                   offsetof(BeepGen, limit) == 0x20 && offsetof(BeepGen, count) == 0x24 && offsetof(BeepGen, cur) == 0x28 && offsetof(BeepGen, q) == 0x2c &&
                   offsetof(BeepGen, hookOn) == 0x11c && offsetof(BeepGen, hookOff) == 0x120 && offsetof(BeepGen, getParam) == 0x124 &&
                   offsetof(BeepGen, div) == 0x128 && sizeof(BeepEntry) == 12 && offsetof(BeepEntry, dur) == 8 && sizeof(BeepCfg) == 0x10 &&
                   offsetof(BeepCfg, getParam) == 0xc,
               "BeepGen");
#endif
_Static_assert(sizeof(Dtmf) == 0x98 && offsetof(Dtmf, e) == 0x40 && offsetof(Dtmf, eb) == 0x60 && offsetof(Dtmf, hist) == 0x64 && offsetof(Dtmf, cnt) == 0x74 &&
                   offsetof(Dtmf, sub) == 0x76 && offsetof(Dtmf, bias) == 0x78 && offsetof(Dtmf, w80) == 0x80 && offsetof(Dtmf, busy) == 0x90 &&
                   offsetof(Dtmf, code) == 0x92 && offsetof(Dtmf, easy) == 0x94,
               "Dtmf (the same layout on every host: detector_progress reads busy at +0x90)");
