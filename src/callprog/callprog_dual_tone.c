/**
 * @file callprog_dual_tone.c
 * @brief Small DSP helpers of the call-progress module: cosine and exponential in fixed point, the receive noise pre-filter, the 2100 / 2250 Hz tone detector.
 *
 *   - TONE_read(): Q14 cosine of an 11-bit phase from a quarter-wave table (used by the dialer for DTMF and the calling tone);
 *   - FP_Pow(): e^x in Q14 by a seven-term Taylor series (used by ResetCallingTone() for the calling-tone gain);
 *   - _iir_filter_create() / _progress() / _delete(): four cascaded biquad sections over an IirFilter (callprog.h); CALLPROG_Create() builds one as the
 *     receive-side noise pre-filter when the host parameter MustNoiseFilterBeApplied is set;
 *   - Dual_TONE_create() / _detect() / _delete(): the answer-tone detector used by CALLPROG_Progress() in the automatic states.
 * The coefficient tables are in callprog_tables.c.  Every multiply-accumulate uses 32-bit unsigned arithmetic (wrap-around) with arithmetic shifts of the
 * int result.
 */
#include "callprog.h"
#include "callprog_tables.h"
#include "core/dsp_asm.h"

/* ---------------------------------------------------------------------------------------------------------------- TONE_read / FP_Pow */
/**
 * @brief Cosine from a quarter-wave table.
 * @param idx  phase; only the low 16 bits are used, taken as a signed short and reduced to 11 bits (0x800 = one turn)
 * @return cos(2 * pi * phase / 0x800) in Q14 (16384 = 1.0), from the table callprog_fptone
 */
int TONE_read(int idx)
{
  unsigned v = (unsigned)(short)idx & 0x7ffu;
  unsigned t;
  if ((unsigned short)(v - 0x201u) <= 0x1ffu)
  { /* 0x201 .. 0x400 */
    t = (unsigned short)callprog_fptone[0x400u - v];
    return (short)(0u - t); /* the negated table value as a signed 16-bit number */
  }
  if ((unsigned short)(v - 0x401u) <= 0x1ffu)
  { /* 0x401 .. 0x600 */
    t = (unsigned short)callprog_fptone[v - 0x400u];
    return (short)(0u - t);
  }
  if ((unsigned short)(v - 0x601u) <= 0x1feu) /* 0x601 .. 0x7ff */
    return callprog_fptone[0x800u - v];
  return callprog_fptone[v]; /* 0 .. 0x200 */
}

/**
 * @brief e^x in Q14 by the Taylor series 1 + x + x^2/2 + ... (seven terms, 1/n! from callprog_fact_fp).
 * @param x  exponent in Q14; the series is evaluated for -0x3fff .. 0x3ffe (|x| < 1.0)
 * @return the sum, Q14 (16384 = 1.0), as a full int
 * @note  Outside that range the second loop is used, which is not the exponential series: its result is not e^x.
 */
int FP_Pow(int x)
{
  int sum = 0x4000;
  int order = 0;
  if ((unsigned)x + 0x3fffu <= 0x7ffeu)
  {
    int power = x;
    for (;;)
    {
      int prod = (int)((unsigned)(int)callprog_fact_fp[order] * (unsigned)power);
      int termQ = prod >> 14; /* the term; the loop ends when |termQ| is 0 (the test is on the shifted term, not on the product) */
      int sign = termQ >> 31;
      sum = (int)((unsigned)sum + (unsigned)termQ);
      power = (int)((unsigned)power * (unsigned)x) >> 14;
      if ((int)((unsigned)(termQ ^ sign) - (unsigned)sign) <= 0)
        break; /* |termQ| <= 0: the term vanished */
      if (order++ > 5)
        break;
    }
  }
  else
  {
    int power = x;
    for (;;)
    {
      int powHi = power >> 14;
      int prod;
      power = (int)((unsigned)powHi * (unsigned)x);
      prod = (int)((unsigned)(int)callprog_fact_fp[order] * (unsigned)powHi);
      sum = (int)((unsigned)sum + (unsigned)prod);
      {
        int sign = prod >> 31;
        if ((int)((unsigned)(prod ^ sign) - (unsigned)sign) <= 0)
          break; /* wraps for INT_MIN: the test is then <= 0 too */
      }
      if (order++ > 5)
        break;
    }
  }
  return sum; /* the full eax */
}

/* ---------------------------------------------------------------------------------------------------------------- _iir_filter_* */
#define U(x) ((uint32_t)(x))
/**
 * @brief Create (or re-initialise) the four-section noise filter.
 * @param obj     existing object, or NULL to allocate one
 * @param nA      number of feedback coefficients to copy (12 for four sections)
 * @param nB      number of feed-forward coefficients to copy
 * @param a       feedback coefficients, Q13, 3 per section (the 2nd and 3rd are used)
 * @param b       feed-forward coefficients, Q13, 3 per section
 * @param scales  five right shifts: input, after sections 1 .. 3, after section 4
 * @return the filter with cleared delay lines
 * @warning The sysdep_malloc() result is not checked.  nA and nB above 25 write past the coefficient areas of IirFilter::w.
 *
 * Layout of IirFilter::w: input delay lines w[0 .. 2], w[3 .. 5], w[6 .. 8], w[9 .. 11] (w[0] is the shifted new sample),
 * output delay lines at w[0x19 .. 0x20],
 * feedback coefficients from w[0x32], feed-forward coefficients from w[0x4b], shifts from w[100].
 */
IirFilter* _iir_filter_create(IirFilter* obj, int nA, int nB, const short* a, const short* b, const short* scales)
{
  int i;
  if (!obj)
    obj = (IirFilter*)sysdep_malloc(sizeof(IirFilter));
  for (i = 0; i < 0x19; i++)
  {
    obj->w[i] = 0;
    obj->w[0x19 + i] = 0;
  }
  for (i = 0; i < nA; i++)
    obj->w[50 + i] = a[i];
  for (i = 0; i < nB; i++)
    obj->w[75 + i] = b[i];
  obj->nB = nB;
  obj->nA1 = nA - 1;
  for (i = 0; i < 5; i++)
    obj->w[100 + i] = scales[i];
  return obj;
}

/** @brief Free a filter created by _iir_filter_create(); NULL is ignored. */
void _iir_filter_delete(IirFilter* f)
{
  if (f)
  {
    sysdep_free(f);
    NO_TAIL_CALL(); /* no tail call: with strict aliasing gcc would store the incoming argument back before a sibling call */
  }
}

/**
 * @brief Filter a block in place.
 * @param f    filter
 * @param n    number of samples
 * @param buf  samples, replaced by the filtered samples
 *
 * Per sample: the input is shifted right by scales[0], then each of the four sections computes (b0 * x0 + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2) >> 13 and,
 * for sections 1 .. 3, shifts the result right by the next scale before passing it on; the last section's result is shifted only if scales[4] is positive.
 */
void _iir_filter_progress(IirFilter* f, int n, short* buf)
{
  short* p = f->w;
  int i;
  for (i = 0; i < n; i++)
  {
    uint32_t acc;
    short s, t;
    p[0] = (short)(buf[i] >> (p[100] & 0x1f));
    acc = 0;
    acc += U(p[0] * p[0x4b]);
    acc += U(p[1] * p[0x4c]);
    acc += U(p[2] * p[0x4d]);
    acc -= U(p[0x19] * p[0x33]);
    acc -= U(p[0x1a] * p[0x34]);
    s = (short)((int)acc >> 13);
    t = p[1];
    p[1] = p[0];
    p[2] = t;
    t = p[0x19];
    p[0x19] = s;
    p[0x1a] = t;
    p[3] = (short)(s >> (p[0x65] & 0x1f));
    acc = 0;
    acc += U(p[3] * p[0x4e]);
    acc += U(p[4] * p[0x4f]);
    acc += U(p[5] * p[0x50]);
    acc -= U(p[0x1b] * p[0x36]);
    acc -= U(p[0x1c] * p[0x37]);
    s = (short)((int)acc >> 13);
    p[5] = p[4];
    p[4] = p[3];
    t = p[0x1b];
    p[0x1b] = s;
    p[0x1c] = t;
    p[6] = (short)(s >> (p[0x66] & 0x1f));
    acc = 0;
    acc += U(p[6] * p[0x51]);
    acc += U(p[7] * p[0x52]);
    acc += U(p[8] * p[0x53]);
    acc -= U(p[0x1d] * p[0x39]);
    acc -= U(p[0x1e] * p[0x3a]);
    s = (short)((int)acc >> 13);
    t = p[7];
    p[7] = p[6];
    p[8] = t;
    t = p[0x1d];
    p[0x1d] = s;
    p[0x1e] = t;
    p[9] = (short)(s >> (p[0x67] & 0x1f));
    acc = 0;
    acc += U(p[9] * p[0x54]);
    acc += U(p[10] * p[0x55]);
    acc += U(p[11] * p[0x56]);
    acc -= U(p[0x1f] * p[0x3c]);
    acc -= U(p[0x20] * p[0x3d]);
    s = (short)((int)acc >> 13);
    p[0xb] = p[10];
    p[10] = p[9];
    t = p[0x1f];
    p[0x1f] = s;
    p[0x20] = t;
    if (p[0x68] > 0)
      s = (short)(s >> (p[0x68] & 0x1f));
    buf[i] = s;
  }
}

/* ---------------------------------------------------------------------------------------------------------------- Dual_TONE_* */
/** @brief Allocate a cleared detector with the default ratio 0xe2 and minimum energy 1; NULL if the allocation fails. */
DualTone* Dual_TONE_create(void)
{
  DualTone* d = (DualTone*)sysdep_malloc(sizeof(DualTone));
  if (d)
  {
    sysdep_memset(d, 0, sizeof(DualTone));
    d->ratio = 0xe2;
    d->thr = 1;
  }
  return d;
}

/** @brief Free a detector. */
void Dual_TONE_delete(DualTone* d) { sysdep_free(d); }

/**
 * @brief Look for the 2100 Hz and 2250 Hz answer tones in a block.
 * @param d   detector
 * @param in  samples (not modified)
 * @param n   number of samples
 * @return 0 = input energy below the minimum; 1 = no tone; 2 = 2100 Hz tone present, not yet long enough; 3 = 2100 Hz tone present for more than 0x4ff
 *         samples since the last decision; 4 = 2250 Hz tone present, not yet long enough; 5 = 2250 Hz tone for 0x500 samples or more
 *
 * Each sample is scaled by 1/8 and run through three pre-filter biquads, then through a 2100 Hz notch, and through a cascade of a 1800 Hz and a 2250 Hz
 * notch.  The energy each notch removes (input energy minus output energy, x * x >> 8) and the input energy are smoothed with weights 0.8 / 0.2.  A
 * notch has found its tone when the removed energy exceeds the level e[2] * ratio >> 8; the 2100 Hz result takes precedence.
 * @note  The second criterion sees the energy removed by both the 1800 Hz and the 2250 Hz notch, so a tone at 1800 Hz also raises it and is reported as
 *        2250 Hz.
 */
int Dual_TONE_detect(DualTone* d, const short* in, int n)
{
  unsigned e0 = d->e[0], e1 = d->e[1], e2 = d->e[2];
  int i = 0;
  d->n1 = (short)(d->n1 + (short)n);
  d->n2 = (short)(d->n2 + (short)n);
  if (0 < n)
  {
    do
    {
      const short* coefRow = callprog_iir_coef;
      int stage = 2;
      int x = (int)*in >> 3;
      short* st = &d->st[0][0];
      short xs, o1, o2, y1, y2;
      int inEnergy, outEnergy, broadOut, lost2100, lostBroad;
      do
      {
        short sc = callprog_iir_scales[3 - stage];
        short s0 = st[0], s1 = st[1], s2 = st[2], s3 = st[3];
        short y;
        uint32_t acc = U(coefRow[4] * x) + U(coefRow[3] * s0) + U(coefRow[1] * s1) - U(coefRow[2] * s2) - U(coefRow[0] * s3);
        st[0] = (short)x;
        y = (short)(((int)acc >> 14) * 2);
        st[1] = s0;
        x = (int)y >> (sc & 0x1f);
        st[2] = y;
        st[3] = s2;
        coefRow += 5;
        st += 4;
        stage = (short)(stage - 1);
      } while (stage >= 0);
      in++;
      xs = (short)x;
      x = (int)xs;

      /* 2100 Hz notch */
      o1 = d->g[0];
      o2 = d->g[1];
      d->g[0] = xs;
      d->g[1] = o1;
      y2 = d->g[3];
      d->g[3] = d->g[2];
      y1 = (short)((int)(U(x * 0x4000) + U(o1 * 0xa0b) + U(o2 * 0x4000) + U(d->g[2] * -0x90a) + U(y2 * -0x33d7)) >> 14);
      d->g[2] = y1;

      /* 1800 Hz notch (the first of the cascade that the second criterion uses) */
      o1 = d->g[8];
      o2 = d->g[9];
      d->g[9] = o1;
      d->g[8] = xs;
      y2 = d->g[11];
      d->g[11] = d->g[10];
      xs = (short)((int)(U(x * 0x4000) + U(o1 * -0x1406) + U(o2 * 0x4000) + U(d->g[10] * 0x1205) + U(y2 * -0x33d7)) >> 14);
      d->g[10] = xs;

      /* 2250 Hz notch, fed with the output of the 1800 Hz notch */
      o1 = d->g[4];
      o2 = d->g[5];
      d->g[5] = o1;
      d->g[4] = xs;
      y2 = d->g[7];
      d->g[7] = d->g[6];
      broadOut = (short)((int)(U((int)xs * 0x4000) + U(o1 * 0x18f9) + U(o2 * 0x4000) + U(d->g[6] * -0x1679) + U(y2 * -0x33d7)) >> 14);
      d->g[6] = (short)broadOut;

      /* energies removed by the notches, smoothed */
      inEnergy = (int)((unsigned)(x * x) >> 8);
      outEnergy = (int)((unsigned)((int)y1 * (int)y1) >> 8);
      lost2100 = (unsigned)outEnergy <= (unsigned)inEnergy ? inEnergy - outEnergy : 0;
      outEnergy = (int)((unsigned)((int)(short)broadOut * (int)(short)broadOut) >> 8);
      lostBroad = (unsigned)outEnergy <= (unsigned)inEnergy ? inEnergy - outEnergy : 0;
      e0 = (e0 * 0xcdu + (unsigned)lost2100 * 0x33u) >> 8;
      e1 = (e1 * 0xcdu + (unsigned)lostBroad * 0x33u) >> 8;
      e2 = (e2 * 0xcdu + (unsigned)inEnergy * 0x33u) >> 8;
      i = (short)(i + 1);
    } while (i < n);
  }

  /* decide */
  d->e[0] = e0;
  d->e[2] = e2;
  d->e[1] = e1;
  if (e2 < (unsigned)(int)d->thr)
  {
    d->n1 = 0;
    d->n2 = 0;
    return 0;
  }
  {
    unsigned lvl = (unsigned)((int)d->ratio) * e2 >> 8;
    if (lvl < e0)
    {
      if (d->n1 > 0x4ff)
      {
        d->n1 = 0;
        d->n2 = 0;
        return 3;
      }
      d->n2 = 0;
      return 2;
    }
    if (e1 <= lvl)
    {
      d->n1 = 0;
      d->n2 = 0;
      return 1;
    }
    if (d->n2 < 0x500)
    {
      d->n1 = 0;
      return 4;
    }
    d->n2 = 0;
    d->n1 = 0;
    return 5;
  }
}
