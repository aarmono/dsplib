/**
 * @file voice_tone.c
 * @brief The voice tone, phasor, silence and detector layer: TONE_*, MTK_phasor, notch, silence_*, detector_*, _status and voice_getgain.
 *
 * Types: voice_tone.h; tables: voice_tone_tables.c.  Users: the voice engine (voice_engine.c: one Detector and one Silence), the DTMF code
 * (voice_dtmf.c: notch(), voice_getgain()).
 * - TONE_create / TONE_detect / TONE_delete: a single-tone energy detector.  A modulated low-pass FIR (taps `coef`, the oscillator frequency
 *   `freq`) filters the input, a second-order recursion follows, and two smoothed energies are kept: B (the FIR output power) and A (that power minus the
 *   recursion output power).  The tone is present when B reaches the minimum energy and B * fc < A.
 * - MTK_phasor: a table-driven sine / cosine oscillator (quarter-wave tables, linear interpolation) that also advances its phase.
 * - notch: a second-order notch filter section (used by the DTMF detector).
 * - silence_*: an energy detector over 800-sample windows that reports "DLE q" / "DLE s" (silence after / without sound) once the silence has lasted
 *   a configured number of windows.
 * - detector_*: the voice-modem tone detector.  It owns a DTMF detector, four TONE objects (1296, 1100, 2100 and 2225 Hz, reported as 'e', 'c', 'a',
 *   'f') and two cadence detectors (busy and dial tone), and reports each detection either as a "DLE <char>" pair in the byte stream or as a status
 *   code, depending on the output mode.  _status() writes such a pair.
 * - voice_getgain: the gains of the beep generator, from the host's tone level parameters, using dsplibs_pow().
 *
 * Float policy.  The arithmetic keeps every intermediate in x87 extended precision (64-bit mantissa), as the FPU stack does, and rounds to a
 * 24-bit float only where a value is stored into a float.  Only products of two floats (48 bits) are exact in both extended and single precision;
 * sums of such products, products of a double constant and a float, and everything that stays on the stack differ between x87 and SSE code.  So ALL
 * arithmetic here is in `long double` (the x87 instruction on i386 and on x86-64 whatever -O or -mfpmath the translation unit uses) and F() (a
 * volatile float store; a plain cast may be elided at -mfpmath=387 or -fexcess-precision=fast) marks each float store.  A value that stays on the
 * FPU stack across loop iterations (the silence energy accumulator, the TONE_detect sums) is kept in a `long double` too.  A float-to-integer
 * conversion truncates; NaN and out-of-range values give the "integer indefinite" (0x8000 / 0x80000000), reproduced explicitly by trunc16() /
 * trunc32() so that the result does not depend on the FPU control word or on SSE / x87 code generation.  The precision control is assumed to
 * be the default (extended).  The one libm-like dependency is the power function of voice_getgain(), dsplibs_pow().
 */
#include <limits.h>
#include <math.h>
#include <string.h>

#include "voice_tone.h"
#include "voice_dtmf.h"          /* create_dtmf, dtmf_progress */
#include "core/host_interface.h" /* GetDTMFHighToneLevel, ... (the host parameter ids) */
#include "core/dsp_asm.h"

#include "voice_f32.h"

/* float -> 16 / 32-bit integer, truncating toward zero: the integer indefinite for NaN and out of range */
static inline short trunc16(long double v)
{
  if (!(v > -32769.0L && v < 32768.0L))
    return (short)-32768;
  return (short)(int)v;
}
static inline int trunc32(long double v)
{
  if (!(v > -2147483649.0L && v < 2147483648.0L))
    return INT_MIN;
  return (int)v;
}
/* fprem run to completion: a - trunc(a / b) * b exactly (the x87 instruction, so NaN / infinity results are the hardware's) */
static inline long double x87_fprem(long double a, long double b)
{
#if defined(__i386__) || defined(__x86_64__)
  X87_FPREM_LOOP(a, b);
  return a;
#else
  return fmodl(a, b);
#endif
}

static inline float fbits(uint32_t u)
{
  float f;
  memcpy(&f, &u, 4);
  return f;
}

/* f * (-f) in the exact x87 form (fld st0; fchs; fmulp): for a NaN f the instruction picks the destination NaN (the sign of the result is NOT the negated
 * sign a C `-(f * f)` gives, and the compiler folds f * -f to -(f * f)) */
static inline long double mul_by_negated(long double f)
{
#if defined(__i386__) || defined(__x86_64__)
  long double r;
  X87_NEG_SQUARE(r, f);
  return r;
#else
  return f * -f;
#endif
}

#include "voice_x87.h"

#define DBG_ON() (dsplibs_debug_level > 1)

/* ---- constants ---- */
static const double k_pi_4000 = 0x1.9bc65b68b73a0p-11; /* 0.0007853981633975 (= pi / 4000 rounded) */
static const double k_1_92 = 0x1.eb851eb851eb8p+0;     /* 1.92 */
static const double k_c4 = 0x1.99999999999a0p-5;       /* 0.050000000000000044: weight of the new energy sample */
static const double k_c5 = 0x1.e666666666666p-1;       /* 0.95: weight of the old smoothed energy */
static const double k_1_25e3 = 0x1.47ae147ae147bp-10;  /* 0.00125 (= 1 / 800): mean of the silence window */
static const double k_2pi_d = 0x1.921fb54442eeap+2;    /* 6.28318530718 */
static const double k_1024_2pi = 0x1.45f306dc9c883p+7; /* 162.97466172610083 (1024 / 2 pi): phase -> table position */
static const double k_pi_d = 0x1.921fb54442d18p+1;     /* 3.141592653589793 */
static const double k_0_05 = 0x1.999999999999ap-5;     /* 0.05: dB -> exponent of 10 */
static const double k_0_276 = 0x1.1a9fbe76c8b44p-2;    /* 0.276: beep amplitude at 6 dB */
static const float k_m2 = -2.0f;
static const float k_2pi_f = 0x1.921fb60000000p+2f; /* 6.2831854820251465 (2 pi as a float) */
static const float k_1000 = 1000.0f;
static const float k_16000 = 16000.0f; /* float -> 16-bit sample for the cadence detectors */
static const float k_10 = 10.0f;
static const float k_800 = 800.0f;                  /* silence window length */
static const float k_2p30 = 0x1.fff8000000000p+29f; /* 1073676288.0: scale of the energy debug output */

/* ---- MTK_phasor ---- */

/* The sign of quadrant e = (int)(table position) >> 8 (0 .. 3).  A phase that rounds up to 2 pi as a float (any phase in (-6e-8, 0) after the +2 pi)
 * gives e = 4, beyond the four-float sign tables: the sine then takes the values of MTK_cos_sign[0..3] and the cosine 0 (the zero padding behind the
 * tables), written out explicitly here.  A NaN phase gives e = -128 (the indefinite); every value is NaN then, and a NaN times anything is that NaN. */
static float sin_sign_at(int e)
{
  if (e >= 0 && e < 4)
    return MTK_sin_sign[e];
  if (e >= 4 && e < 8)
    return MTK_cos_sign[e - 4];
  return 1.0f;
}
static float cos_sign_at(int e)
{
  if (e >= 0 && e < 4)
    return MTK_cos_sign[e];
  if (e >= 4 && e < 8)
    return 0.0f;
  return 1.0f;
}

/**
 * @brief Table-driven sine / cosine oscillator step.
 * @param p  p[0] = phase in rad (in); p[1] = cos and p[2] = sin of the phase (out); p[3] = phase step in rad (in).  p[0] is replaced by the phase
 *           plus the step, wrapped to below pi.
 * @note  The phase is first reduced to [0, 2 pi); the value comes from 256-entry quarter-wave tables with linear interpolation.
 * @bug   A phase in (-6e-8, 0) rounds to 2 pi as a float and selects quadrant 4, beyond the sign tables; sin_sign_at() / cos_sign_at() give the values
 *        the adjacent data would give (see above).  Callers keep their phases in (-pi, pi), where the case can occur only for tiny negative phases.
 */
void MTK_phasor(float* p)
{
  /* phase = p[0] mod (2 pi as a float), + 2 pi (the double constant) when negative, each stored as a float */
  float ph = F(x87_fprem(ldl(&p[0]), LD(k_2pi_f)));
  long double frac;
  int i, e, idx;
  float x;
  long double sum, cv, sv;

  if (!(ph >= 0.0f))
    ph = F(LD(ph) + LD(k_2pi_d));
  p[0] = ph;

  /* table position: whole part (quadrant and index) and fraction */
  x = F(LD(k_1024_2pi) * LD(ph));
  i = (int)trunc16(LD(x)); /* truncated to 16 bits */
  e = i >> 8;
  frac = LD(x) - LD(i);
  if (e & 1)
  {
    frac = LD(1.0f) - frac;
    idx = (~i) & 0xff;
  }
  else
    idx = i & 0xff;

  /* interpolate cos and sin, apply the quadrant signs */
  cv = LD(MTK_cos_table[idx]) + (LD(MTK_cos_table[idx + 1]) - LD(MTK_cos_table[idx])) * frac;
  p[1] = F(cv * LD(cos_sign_at(e)));
  sv = frac * (LD(MTK_sin_table[idx + 1]) - LD(MTK_sin_table[idx])) + LD(MTK_sin_table[idx]);
  p[2] = F(sv * LD(sin_sign_at(e)));

  /* advance the phase */
  sum = ld_addf(LD(ph), &p[3]);
  if (sum < LD(k_pi_d) || sum != sum) /* below pi, or NaN */
    p[0] = F(sum);
  else
    p[0] = F(sum - LD(k_2pi_d));
}

/* ---- notch ---- */

/**
 * @brief One sample through a second-order notch section.
 * @param x  input sample
 * @param s  state, two floats (updated)
 * @param c  coefficients, four floats
 * @return the output y = c[3] * x + s[0], as an unrounded extended value (the caller squares it)
 * @note  With t = c[3] * x: s[0]' = c[0] * t + s[1] + c[1] * y and s[1]' = t + c[2] * y.
 */
long double notch(float x, float* s, const float* c)
{
  long double t3 = ld_mulf(ldl(&c[3]), &x);
  long double y = ldl(&s[0]) + t3;
  long double z = ld_addf(ldl(&c[0]) * t3, &s[1]) + ldl(&c[1]) * y;
  long double w = t3 + ldl(&c[2]) * y;
  s[0] = F(z);
  s[1] = F(w);
  return y;
}

/* ---- TONE ---- */

/**
 * @brief Create or re-initialise a tone detector.
 * @param obj  an existing object (re-initialised in place; its four blocks must hold cfg->n floats / 5 floats / 2 floats), or NULL to allocate
 * @param cfg  the configuration (copied), or NULL for TONE_CFG
 * @return the object, or NULL when an allocation failed (what was allocated is freed)
 * @note  Computes the oscillator step from cfg->freq, the recursion coefficients k0 = -2 cos(step), k1 = -k0 * f18, k2 = -f18^2, and the FIR taps
 *        coef[i] = 2 cos(phase_i) * lpf[i]; clears the input ring and the state.  With cfg->n <= 0 no block is allocated.
 */
ToneObj* TONE_create(ToneObj* obj, const ToneCfg* cfg)
{
  float p[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  float q[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  int fresh = 0, n, i;
  const float* lpf;
  float *coef, *ring, *b14, *b8;
  float f18;

  if (!obj)
  {
    obj = (ToneObj*)sysdep_malloc((unsigned)sizeof(ToneObj));
    if (!obj)
      return NULL;
    fresh = 1;
  }
  if (!cfg)
    cfg = &TONE_CFG;
  obj->cfg = *cfg;
  n = obj->cfg.n;
  if (fresh)
  {
    /* the four blocks are allocated only when n > 0; with n <= 0 they stay NULL and the stores through them are skipped */
    obj->coef = obj->ring = obj->b14 = obj->b8 = NULL;
    if (n > 0)
    {
      obj->coef = (float*)sysdep_malloc((unsigned)n * 4u);
      obj->ring = (float*)sysdep_malloc((unsigned)n * 4u);
      obj->b14 = (float*)sysdep_malloc(0x14);
      obj->b8 = (float*)sysdep_malloc(8);
      if (!obj->coef || !obj->ring || !obj->b14 || !obj->b8)
      { /* an allocation failed: release the others */
        if (obj->b8)
          sysdep_free(obj->b8);
        if (obj->b14)
          sysdep_free(obj->b14);
        if (obj->ring)
          sysdep_free(obj->ring);
        if (obj->coef)
          sysdep_free(obj->coef);
        sysdep_free(obj);
        return NULL;
      }
    }
  }

  /* the oscillator step: freq * pi / 4000 (double constant * float, rounded to 64 bits, then to float), normalised by one MTK_phasor call with step 0 */
  p[0] = F(ld_mulf(LD(k_pi_4000), &obj->cfg.freq));
  p[3] = 0.0f;
  MTK_phasor(p);
  obj->w30 = 0.0f;
  obj->ph = p[0];
  obj->w38 = 0.0f;
  obj->w64 = 0; /* cleared state words; TONE_detect() does not read them */
  obj->w68 = 0;
  obj->w6c = 0;
  obj->s0 = 0.0f;
  obj->s1 = 0.0f;
  obj->accA = 0.0f;
  obj->accB = 0.0f;
  obj->idx = 0;
  f18 = obj->cfg.f18;

  /* recursion coefficients */
  {
    long double a = ld_mulf(LD(k_m2), &p[1]); /* exact (two floats) */
    obj->k0 = F(a);
    obj->k1 = F(ld_chs(a) * ldl(&f18));
    obj->k2 = F(mul_by_negated(LD(f18)));
  }

  /* FIR taps: 2 * cos(phase_i) * lpf[i], the phase advancing by the oscillator step per tap */
  p[3] = p[0];
  coef = obj->coef;
  ring = obj->ring;
  lpf = obj->cfg.lpf;
  for (i = 0; (short)i < (short)n; i = (short)(i + 1))
  {
    MTK_phasor(p);
    ring[i] = 0.0f;
    {
      long double prod = ld_mulf(ldl(&p[1]), &lpf[i]);
      coef[i] = F(prod + prod); /* fadd st, st(0): exactly twice */
    }
  }

  obj->w1b0 = 0;
  for (i = 0; i < 80; i++)
    obj->z70[i] = 0;
  obj->w1c0 = 0;
  obj->selfk = &obj->k0;
  obj->w1c4 = 0;

  /* two small coefficient blocks (not used by TONE_detect()) */
  q[0] = fbits(0x3d4104fbu);
  MTK_phasor(q);
  b14 = obj->b14;
  b8 = obj->b8;
  if (b14)
  {
    b14[1] = 1.0f;
    b14[0] = fbits(0xbf6bedfau);
    b14[2] = F(ld_mulf(LD(k_1_92), &q[1]));
    b14[3] = F(ld_mulf(LD(k_m2), &q[1]));
    b14[4] = 1.0f;
  }
  if (b8)
  {
    b8[0] = 0.0f;
    b8[1] = 0.0f;
  }
  return obj;
}

/**
 * @brief Free a tone detector and its blocks (the blocks only when cfg.n > 0).
 * @param obj  the object; NULL is ignored
 */
void TONE_delete(ToneObj* obj)
{
  if (!obj)
    return;
  if (obj->cfg.n > 0)
  {
    sysdep_free(obj->b8);
    sysdep_free(obj->b14);
    sysdep_free(obj->ring);
    sysdep_free(obj->coef);
  }
  sysdep_free(obj);
}

/**
 * @brief Feed samples to the tone detector and test for the tone.
 * @param obj    the detector
 * @param in     input samples
 * @param count  number of samples, taken as unsigned 16 bits (a negative count runs 65536 + count samples)
 * @return 2 when the smoothed band energy B is below cfg.f14 (or NaN), else 1 when B * fc >= A (no tone), else 0 (tone present)
 * @note  Per sample: the FIR over the input ring gives S; the recursion (k0, k1, k2) gives W; then A = 0.95 * A + 0.05 * (S^2 - W^2) and
 *        B = 0.95 * B + 0.05 * S^2.  A is clamped at 0 after the block.
 */
int TONE_detect(ToneObj* obj, const float* in, short count)
{
  float* ring = obj->ring;
  const float* coef = obj->coef;
  int n = obj->cfg.n; /* a short */
  int idx = obj->idx;
  unsigned cnt = (unsigned short)count;                 /* the loop counter is 16 bits: a negative count runs 65536 + count times */
  long double A = ldl(&obj->accA), B = ldl(&obj->accB); /* kept in extended precision across the loop */
  const long double k0 = ldl(&obj->k0), k1 = ldl(&obj->k1), k2 = ldl(&obj->k2);
  const long double c4 = LD(k_c4), c5 = LD(k_c5);
  int result;

  while (cnt--)
  {
    int na = (short)(idx + 1), e;
    long double S = 0.0L, s0, s1, T1, U, V, W, SS, E, G;
    const float* cp = coef;
    const float* rp;
    float fe, fg;

    /* store the sample in the ring and run the FIR from the newest sample backwards */
    idx = (na < n) ? na : 0;
    memcpy(&ring[idx], in, 4); /* a raw copy: a signalling NaN keeps its bits */
    in++;
    rp = ring + idx;
    for (e = idx; e >= 0; e--, rp--)
      S += ld_mulf(ldl(cp++), rp);
    rp = rp + n; /* continue at ring + (n - 1) after the first loop ended at ring - 1 */
    for (e = n - 1; e > idx; e--, rp--)
      S += ld_mulf(ldl(cp++), rp);

    /* the second-order recursion on S, then the smoothed energies */
    s0 = ldl(&obj->s0);
    s1 = ldl(&obj->s1);
    T1 = k1 * s1 + S;
    U = k2 * s0 + T1;
    V = s1 * k0 + U;
    W = s0 + V;
    obj->s0 = F(s1);
    obj->s1 = F(U);
    SS = S * S;
    E = A * c5 + (SS - W * W) * c4;
    G = c4 * SS + B * c5;
    fe = F(E); /* E and G are rounded to float and reloaded before the next sample */
    fg = F(G);
    A = ldl(&fe);
    B = ldl(&fg);
  }

  obj->idx = (short)idx;
  if (A < 0.0L)
    A = 0.0L;
  obj->accA = F(A);
  obj->accB = F(B);
  if (!(B >= LD(obj->cfg.f14)))
    return 2; /* below the minimum energy, or NaN */
  result = (ld_mulf(B, &obj->cfg.fc) >= A) ? 1 : 0;
  return result;
}

/* ---- _status / silence ---- */

/**
 * @brief Write the two-byte report "DLE <code>" into a byte stream.
 * @param buf   destination (2 bytes are written at buf; it is NOT advanced)
 * @param len   byte count of the stream, increased by 2
 * @param code  the report character
 */
void _status(unsigned char* buf, unsigned short* len, int code)
{
  if (DBG_ON())
    dsplibs_debug_printf("DLE %d\n", (int)(signed char)code);
  *len = (unsigned short)(*len + 2);
  buf[0] = 0x10;
  buf[1] = (unsigned char)code;
}

/* mean-energy thresholds of the silence detector by level 1 .. 3 (index 0 is unused: level 0 disables it) */
static const float silence_level[4] = {-0x1.0000000000000p+0f, 0x1.5185460000000p-19f, 0x1.d4c7540000000p-18f, 0x1.7bb5ee0000000p-16f};

/**
 * @brief Create or reset a silence detector.
 * @param obj  an existing detector to reset, or NULL to allocate
 * @param ctx  first argument of the callback
 * @param cb   parameter callback (ids 0x52 level, 0x53 silence length in windows)
 * @return the detector, or NULL when the allocation failed
 */
Silence* silence_create(Silence* obj, struct modem* ctx, VoiceParamFn cb)
{
  if (!obj)
  {
    obj = (Silence*)sysdep_malloc((unsigned)sizeof(Silence));
    if (!obj)
      return NULL;
  }
  obj->count = 0;
  obj->ctx = ctx;
  obj->n = 0;
  obj->cb = cb;
  obj->energy = 0.0f;
  obj->heard = 0;
  return obj;
}

/** @brief Free a silence detector. */
void silence_delete(Silence* obj) { sysdep_free(obj); }

/**
 * @brief Test whether the silence has lasted longer than v seconds.
 * @param obj  the detector
 * @param v    time in seconds; a window is 0.1 s, so the test is count > 10 * v (truncated; NaN and overflow give the integer indefinite)
 * @return 1 when the silence counter exceeds 10 * v, else 0
 */
int silence_is_more_then(Silence* obj, float v)
{
  int t = trunc32(LD(k_10) * LD(v));
  return (int)obj->count > t;
}

/**
 * @brief Feed samples to the silence detector.
 * @param s      the detector
 * @param x      input samples
 * @param count  number of samples
 * @param buf    destination of a report ("DLE 0x71" when sound was heard before, else "DLE 0x73"); not advanced
 * @param len    byte count of the stream (increased by 2 per report)
 * @note  Windows of 800 samples: the mean energy against the threshold of the callback's level (id 0x52; 0 or above 3 disables the detector).  A
 *        window above the threshold clears the silence counter and marks sound heard; otherwise the counter grows, and above the callback's length
 *        (id 0x53) a report is written and the counter restarts.
 */
void silence_progress(Silence* s, const float* x, short count, unsigned char* buf, unsigned short* len)
{
  unsigned lvl = (unsigned short)s->cb(s->ctx, 0x52);
  int total = count, i;
  long double E;

  if (lvl > 3 || lvl == 0)
    return;
  if (0 >= total)
    return;

  /* accumulate the energy; at the end of each 800-sample window compare it with the threshold */
  E = ldl(&s->energy);
  for (i = 0; i < total; i++)
  {
    unsigned short c = (unsigned short)(s->n + 1);
    long double xv = ldl(&x[i]);
    s->n = c;
    E = E + xv * xv; /* stays extended while the window is open */
    if (LD(c) < LD(k_800))
    {
      s->energy = F(E);
      continue;
    }
    {
      float e2 = F(E * LD(k_1_25e3));
      float thr = silence_level[lvl];
      s->energy = e2;
      if (DBG_ON())
      {
        int a = trunc32(LD(e2) * LD(k_2p30));
        int b = trunc32(LD(thr) * LD(k_2p30));
        dsplibs_debug_printf("energy = %d, threshold = %d\n", a, b);
      }
      if (LD(s->energy) > LD(thr))
      { /* sound in this window */
        s->count = 0;
        s->heard = 1;
        s->n = 0;
      }
      else
      {
        unsigned r = s->cb(s->ctx, 0x53);
        int limit = (r > 0x7fffffffu) ? INT_MIN : (int)r; /* a value above 2^31 - 1 gives the integer indefinite */
        unsigned short cc = (unsigned short)(s->count + 1);
        s->count = cc;
        if (DBG_ON())
          dsplibs_debug_printf("silence counter %d, level %d\n", (int)cc, limit);
        if ((int)s->count > limit)
        {
          unsigned short heard = s->heard;
          s->count = 0;
          _status(buf, len, heard ? 0x71 : 0x73);
        }
        s->n = 0;
      }
    }
    E = 0.0L;
    s->energy = 0.0f;
  }
}

/* ---- detector ---- */
/* per tone detector: centre frequency (1296, 1100, 2100, 2225 Hz), status code, report character, consecutive blocks needed, enable bit */
static const float detector_tone_freq[4] = {0x1.4500000000000p+10f, 0x1.1300000000000p+10f, 0x1.0680000000000p+11f, 0x1.1620000000000p+11f};
static const int detector_status[4] = {3, 4, 5, 6};
static const char detector_tone_char[4] = {'e', 'c', 'a', 'f'};
static const unsigned short detector_threshold[4] = {25, 21, 12, 25};
static const unsigned detector_mask[4] = {2, 4, 8, 16};
/* the report character by dtmf_test() digit index (low + 4 * high) */
static const char detector_dtmf_char[16] = {'1', '4', '7', '*', '2', '5', '8', '0', '3', '6', '9', '#', 'A', 'B', 'C', 'D'};

/**
 * @brief Create or re-initialise the detector.
 * @param obj  an existing detector (its sub-objects are re-initialised), or NULL to allocate one
 * @param ctx  modem context for the callback and the cadence detectors
 * @param cb   parameter callback (id 0x49 gives the tone level factor); may be NULL
 * @return the detector (all detectors enabled, in-stream output), or NULL when the allocation failed
 * @warning The results of create_dtmf(), TONE_create() and cadence_create() are not checked.
 */
Detector* detector_create(Detector* obj, struct modem* ctx, VoiceParamFn cb)
{
  ToneCfg tc;
  CadenceParams cp;
  int i;

  if (!obj)
  {
    obj = (Detector*)sysdep_malloc((unsigned)sizeof(Detector));
    if (!obj)
      return NULL;
    obj->dtmf = NULL;
    obj->cad8 = NULL;
    obj->cadBusy = NULL;
    obj->cadDial = NULL;
    for (i = 0; i < 4; i++)
      obj->tone[i] = NULL;
  }
  obj->out_mode = 1;
  obj->dtmf = create_dtmf(obj->dtmf);

  /* the four tone detectors */
  tc = tone_amode_cfg;
  tc.lpf = TONE_CFG.lpf;
  for (i = 0; i < 4; i++)
  {
    tc.freq = detector_tone_freq[i];
    obj->tone[i] = TONE_create(obj->tone[i], &tc);
    obj->cnt[i] = 0;
  }
  obj->w30 = cb ? (unsigned)(50u * (unsigned)cb(ctx, 0x49)) >> 2 : 0u;
  obj->w2c = 0;
  obj->enable = 0x3f;

  /* the busy / dial cadences: {50, 50, 3, 0, type busy / dial, 0, 0} */
  memset(&cp, 0, sizeof cp);
  cp.w0 = 50;
  cp.w4 = 50;
  cp.w8 = 3;
  cp.type = CADENCE_BUSY;
  cp.callerParamB = 0;
  obj->cadBusy = cadence_create(obj->cadBusy, &cp, 2, ctx);
  cp.type = CADENCE_DIAL;
  obj->cadDial = cadence_create(obj->cadDial, &cp, 2, ctx);
  return obj;
}

/**
 * @brief Delete the detector and its sub-objects.
 * @param d  the detector (not checked for NULL)
 */
void detector_delete(Detector* d)
{
  int i;
  sysdep_free(d->dtmf);
  for (i = 0; i < 4; i++)
    TONE_delete(d->tone[i]);
  if (d->cad8)
    cadence_delete(d->cad8);
  if (d->cadBusy)
    cadence_delete(d->cadBusy);
  if (d->cadDial)
    cadence_delete(d->cadDial);
  sysdep_free(d);
}

/** @brief Set the enable mask: bit 0 DTMF, bits 1 .. 4 the four tone detectors, bit 5 the busy and dial cadences. */
void detector_set_enable(Detector* d, unsigned short v) { d->enable = v; }
/** @brief Report detections as the return value of detector_progress() (tone codes 3 .. 6, busy 1, dial 2). */
void detector_set_output_status(Detector* d) { d->out_mode = 0; }
/** @brief Report detections as "DLE <char>" pairs in the byte stream (the default). */
void detector_set_output_in_stream(Detector* d) { d->out_mode = 1; }

/**
 * @brief Run the enabled detectors over a block.
 * @param d      the detector
 * @param x      input samples
 * @param count  number of samples
 * @param buf    byte stream for the reports (in stream mode)
 * @param len    byte count of the stream, increased by 2 per report
 * @return in status mode the code of the last detection: 3 .. 6 for a tone, 1 for busy, 2 for dial tone; otherwise 0
 * @note  The DTMF detector reports a digit (when its tone ends) as "DLE <digit>".  The four tone detectors run only while no DTMF digit is held; a
 *        tone is reported once it has been detected in more than detector_threshold blocks in a row, and then on every further block while it lasts.
 *        The DTMF detector runs in mode 2, which dtmf_detect() treats like the mode 0 plan.
 * @bug   In stream mode _status() does not advance `buf`: two tone or cadence reports in one block overwrite each other while `len` counts both,
 *        so the second pair of bytes is stale.  A tone that persists is reported again on every block (the consecutive-block counter is never reset
 *        while the tone lasts).  The caller's buffer must have room for all reports.
 */
int detector_progress(Detector* d, const float* x, short count, unsigned char* buf, unsigned short* len)
{
  int result = 0, n = count, b, i;
  unsigned flags = d->enable;

  if (flags & 1)
  {
    short r = dtmf_progress(d->dtmf, x, (short)n, 2);
    if ((unsigned short)(r + 2) > 1)
    { /* -2 / -1: nothing */
      int code = ((unsigned short)r > 0xf) ? -1 : (int)(signed char)detector_dtmf_char[r];
      short digit = (short)code;
      if (digit >= 0)
      {
        if (DBG_ON())
          dsplibs_debug_printf("detected dtmf %c\n", (int)(unsigned char)digit);
        *len = (unsigned short)(*len + 2);
        buf[0] = 0x10;
        buf[1] = (unsigned char)digit;
        buf += 2;
      }
      else if (DBG_ON())
        dsplibs_debug_printf("false dtmf detect %d\n", (int)r);
    }
    flags = d->enable;
  }

  /* the four tone detectors */
  if (DTMF_W90(d->dtmf) == 0)
  {
    for (b = 0; b < 4; b++)
    {
      if (!(detector_mask[b] & flags))
        continue;
      if (TONE_detect(d->tone[b], x, (short)n) != 0)
        d->cnt[b] = 0;
      else
      {
        unsigned short c = (unsigned short)(d->cnt[b] + 1);
        d->cnt[b] = c;
        if (c > detector_threshold[b])
        {
          if (DBG_ON())
            dsplibs_debug_printf("detected %c\n", (int)detector_tone_char[b]);
          if (d->out_mode != 0)
            _status(buf, len, (int)detector_tone_char[b]);
          else
            result = detector_status[b];
        }
      }
      flags = d->enable;
    }
  }

  /* the busy and dial-tone cadence detectors, per sample */
  if (flags & 0x20)
  {
    for (i = 0; i < n; i++)
    {
      short smp = trunc16(ld_mulf(ldl(&x[i]), &k_16000));
      if ((short)cadence_progress(d->cadBusy, smp) == 1)
      {
        if (DBG_ON())
          dsplibs_debug_printf("busy detected by cadence\n");
        if (d->out_mode != 0)
          _status(buf, len, 0x62);
        else
          result = 1;
      }
      smp = trunc16(ld_mulf(ldl(&x[i]), &k_16000));
      if ((short)cadence_progress(d->cadDial, smp) == 1)
      {
        if (DBG_ON())
          dsplibs_debug_printf("dial detected by cadence\n");
        if (d->out_mode != 0)
          _status(buf, len, 0x64);
        else
          result = 2;
      }
    }
  }
  return result;
}

/* ---- the gains of the beep generator ---- */

/* dsplibs_pow: the library's own x87 power function, the old glibc i386 algorithm: special cases through fxam, an exact square-and-multiply loop for an
 * integer y, otherwise 2^(y * log2 x) = fscale(2^frac - 1 + 1, int) with fyl2x (fyl2xp1 when |x - 1| <= 0.29).  It returns an EXTENDED value (no rounding
 * to double, no double range limit: 10^346.8 is finite).  It is not called `pow` so that it does not replace libm's for the whole process; it uses the
 * x87 instructions (fyl2x / fyl2xp1 / f2xm1 / frndint / fscale / fistp through inline asm: the hardware result, like src/core/dsp_math.h).  Its constant
 * table is {+inf, -inf, -0, +0, 1, 0.29}; the special-case code indexes it as if it were {+inf, +0, -inf, -0}, so pow(+inf, y < 0), pow(|x| < 1, +inf)
 * etc. return -inf.  Only voice_getgain() calls it, with x = 10 and a finite y, so none of the special cases is reached. */
#if defined(__i386__) || defined(__x86_64__)
static inline long double x87_fyl2x(long double x, long double y)
{
  long double r;
  X87_YL2X(r, x, y);
  return r;
}
static inline long double x87_fyl2xp1(long double x, long double y)
{
  long double r;
  X87_YL2XP1(r, x, y);
  return r;
}
static inline long double x87_f2xm1(long double a)
{
  X87_F2XM1(a);
  return a;
}
static inline long double x87_frndint(long double a)
{
  X87_RNDINT(a);
  return a;
}
static inline long double x87_fscale(long double a, long double b)
{
  X87_SCALE(a, b);
  return a;
}
static inline long long x87_fistp64(long double v)
{
  long long r;
  X87_FISTP_I64(r, v);
  return r;
}
#endif
/* inf, -inf, -0, 0, one, limit (0.29) */
static const double pow_T[6] = {__builtin_inf(), -__builtin_inf(), -0.0, 0.0, 1.0, 0x1.28f5c28f5c28fp-2};
static inline int dbl_is_zero(double d) { return d == 0.0; }
static inline int dbl_is_nan(double d) { return d != d; }
static inline int dbl_is_inf(double d) { return d == pow_T[0] || d == pow_T[1]; }
static inline int dbl_neg(double d)
{
  unsigned long long u;
  memcpy(&u, &d, 8);
  return (int)(u >> 63);
}
/**
 * @brief x ^ y in extended precision (see the note above).
 * @return the extended result; special cases as the old i386 glibc, including its swapped infinity table
 */
long double dsplibs_pow(double xd, double yd)
{
#if defined(__i386__) || defined(__x86_64__)
  int ysign = dbl_neg(yd), xsign = dbl_neg(xd);
  long double x = xd, y = yd, ly, lg;
  long long iy;
  if (dbl_is_zero(yd))
    return (long double)pow_T[4];
  if (dbl_is_inf(yd))
  {
    long double absX = x < 0 ? -x : x; /* fabs */
    if (dbl_is_nan(xd))
      return x;
    if (absX == 1.0L)
    {
      volatile double z = pow_T[3];
      return (long double)pow_T[0] * z; /* inf * 0: the default NaN */
    }
    else
    {
      int below = absX < 1.0L; /* C0 of fcomp: |x| < 1 */
      int bit = ysign ^ below;
      return (long double)pow_T[bit ? 1 : 0];
    }
  }
  if (dbl_is_nan(yd))
    return y;
  /* y is finite and non-zero */
  if (dbl_is_zero(xd))
  {
    int iyok;
    if (!ysign)
    { /* y > 0 */
      if (!xsign)
        return (long double)pow_T[3];
      iy = x87_fistp64(y);
      iyok = ((long double)iy == y);
      if (!iyok)
        return (long double)pow_T[3];
      if (!(iy & 1))
        return (long double)pow_T[3];
      if ((unsigned)((unsigned long long)iy >> 32) >= 0xffe00000u)
        return (long double)pow_T[3];
      return (long double)pow_T[2];
    }
    else
    { /* y < 0 */
      volatile double one = pow_T[4], zero = pow_T[3];
      long double inf = (long double)one / (long double)zero;
      if (!xsign)
        return inf;
      iy = x87_fistp64(y);
      iyok = ((long double)iy == y);
      if (!iyok)
        return inf;
      if (!(iy & 1))
        return inf;
      if ((unsigned)((unsigned long long)iy >> 32) <= 0xffe00000u)
        return inf;
      return -inf;
    }
  }
  if (dbl_is_inf(xd))
  {
    if (!xsign)
    { /* +inf: y compared with 0 */
      return (long double)pow_T[y < 0.0L ? 1 : 0];
    }
    iy = x87_fistp64(y);
    if ((long double)iy == y && (iy & 1))
    {
      unsigned hi = (unsigned)((unsigned long long)iy >> 32);
      unsigned a = ((int)hi < 0) ? 0u - hi : hi;
      if (a <= 0x200000u)
        return (long double)pow_T[1 + (hi >> 31)];
    }
    if ((long double)iy == y)
      return (long double)pow_T[(unsigned)((unsigned long long)iy >> 63)]; /* an even integer: the sign of the high dword */
    return (long double)pow_T[ysign];                                      /* a non-integer: the sign bit of y */
  }
  /* finite non-zero x (a NaN x also arrives here) and finite non-zero y */
  iy = x87_fistp64(y);
  if ((long double)iy == y)
  { /* ucomp equal: an integer y (an unordered result cannot occur: y is not a NaN) */
    unsigned long long n = (unsigned long long)iy;
    long double base = x, acc = 1.0L;
    if (iy < 0)
    {
      base = (long double)pow_T[4] / base;
      n = 0ull - n;
    }
    do
    {
      if (n & 1u)
        acc = acc * base;
      base = base * base;
      n >>= 1;
    } while (n);
    return acc;
  }
  ly = y;
  {
    long double d = x - (long double)pow_T[4];
    if (d < 0)
      d = -d;
    /* fcomp + ja: |x - 1| > limit; an unordered compare (x NaN) takes the fyl2xp1 path */
    if (d > (long double)pow_T[5])
      lg = x87_fyl2x(x, (long double)pow_T[4]);
    else
      lg = x87_fyl2xp1(x - (long double)pow_T[4], (long double)pow_T[4]);
  }
  {
    long double prod = lg * ly, n, frac;
    n = x87_frndint(prod);
    frac = prod - n;
    return x87_fscale(x87_f2xm1(frac) + (long double)pow_T[4], n);
  }
#else
  return powl((long double)xd, (long double)yd);
#endif
}

/**
 * @brief Compute the two oscillator gains of a beep generator from the host's tone level parameters.
 * @param beep   the beep generator; its `ctx` is the host modem the level parameters are read from
 * @param out_b  receives gain b = 10^(-levelDiff * 0.05) * a
 * @param out_a  receives gain a = 0.276 * 10^((6 - highLevel) * 0.05 - beepAtten * 0.05)
 * @note  highLevel, levelDiff and beepAtten are the host parameters GetDTMFHighToneLevel, GetDTMFHighAndLowToneLevelDifference and
 *        GetAdditAttenToBeepgenVoice.  The powers are dsplibs_pow(), not libm's.  The parameter values and gains are printed at debug level 2.
 */
void voice_getgain(struct BeepGen* beep, float* out_b, float* out_a)
{
  struct modem* m = beep->ctx;
  int highLevel = modem_get_param(m, GetDTMFHighToneLevel);
  int levelDiff = modem_get_param(m, GetDTMFHighAndLowToneLevelDifference);
  double d1 = (double)(LD((int)(6u - (unsigned)highLevel)) * LD(k_0_05)); /* 6 - highLevel wraps in 32 bits */
  int beepAtten = modem_get_param(m, GetAdditAttenToBeepgenVoice);
  double y1 = (double)(LD(d1) + LD(beepAtten) * LD(-k_0_05));
  long double r1 = dsplibs_pow(10.0, y1);
  float A = F(r1 * LD(k_0_276));
  double y2;
  int dbg_first;
  long double r2;

  *out_a = A;
  y2 = (double)(LD((int)(0u - (unsigned)levelDiff)) * LD(k_0_05)); /* neg wraps */
  r2 = dsplibs_pow(10.0, y2);
  {
    float vb = F(r2 * LD(A));
    unsigned lvl = dsplibs_debug_level; /* read before the store to *out_b; the barrier keeps that order whatever -fno-strict-aliasing does */
    COMPILER_BARRIER();
    *out_b = vb;
    dbg_first = lvl > 1;
  }

  if (dbg_first)
  {
    dsplibs_debug_printf("BeepGen: Additional Attenuation to Beepgen Voice = %d.\n", (int)modem_get_param(m, GetAdditAttenToBeepgenVoice));
    if (DBG_ON())
    {
      dsplibs_debug_printf("BeepGen: GAIN1*1000 = %d.\n", trunc32(LD(*out_b) * LD(k_1000)));
      if (DBG_ON())
        dsplibs_debug_printf("BeepGen: GAIN2*1000 = %d.\n", trunc32(LD(k_1000) * LD(*out_a)));
    }
  }
}

#if UINTPTR_MAX == 0xffffffffu
_Static_assert(sizeof(ToneCfg) == 0x30 && offsetof(ToneCfg, lpf) == 0x1c && offsetof(ToneCfg, n) == 0x20 && offsetof(ToneCfg, w24) == 0x24 &&
                   offsetof(ToneCfg, w2c) == 0x2c,
               "ToneCfg");
_Static_assert(sizeof(ToneObj) == 0x1c8 && offsetof(ToneObj, w30) == 0x30 && offsetof(ToneObj, coef) == 0x3c && offsetof(ToneObj, ring) == 0x40 &&
                   offsetof(ToneObj, idx) == 0x44 && offsetof(ToneObj, k0) == 0x48 && offsetof(ToneObj, s0) == 0x54 && offsetof(ToneObj, accA) == 0x5c &&
                   offsetof(ToneObj, accB) == 0x60 && offsetof(ToneObj, w64) == 0x64 && offsetof(ToneObj, z70) == 0x70 && offsetof(ToneObj, w1b0) == 0x1b0 &&
                   offsetof(ToneObj, b14) == 0x1b4 && offsetof(ToneObj, b8) == 0x1b8 && offsetof(ToneObj, selfk) == 0x1bc && offsetof(ToneObj, w1c4) == 0x1c4,
               "ToneObj");
_Static_assert(sizeof(Silence) == 0x14 && offsetof(Silence, cb) == 4 && offsetof(Silence, count) == 8 && offsetof(Silence, n) == 0xa &&
                   offsetof(Silence, heard) == 0xc && offsetof(Silence, energy) == 0x10,
               "Silence");
_Static_assert(sizeof(Detector) == 0x38 && offsetof(Detector, dtmf) == 4 && offsetof(Detector, cad8) == 8 && offsetof(Detector, cadBusy) == 0xc &&
                   offsetof(Detector, cadDial) == 0x10 && offsetof(Detector, tone) == 0x14 && offsetof(Detector, cnt) == 0x24 &&
                   offsetof(Detector, w2c) == 0x2c && offsetof(Detector, w30) == 0x30 && offsetof(Detector, out_mode) == 0x34,
               "Detector");
#endif
