/**
 * @file fpm_iir.c
 * @brief FPM biquad-cascade IIR filters (16-bit data, Q14 coefficients).
 *
 * Three stateless entry points that run a cascade of second-order sections over caller-owned coefficient and state arrays: FPM_iir_filt() filters one
 * sample through a direct-form-II cascade, FPM_iir_filt_block() does the same over a block, and FPM_iir_filt_II() filters a block in place with a
 * direct-form-I cascade.  Users: FPM_MTD_detect() (fpm_mtd.c, DC blocker and detector filter), the tone generator (fpm_tone.c), FPM_FSD_demodulate()
 * (fpm_fsd.c), the V.23 demodulator, the caller-ID detectors and the fax class 1 high-pass.  Every section has five coefficients; the filters keep no
 * state of their own.
 */
#include "fpm.h"

/**
 * @brief Run one sample through a direct-form-II biquad cascade.
 * @param seed   input sample
 * @param coefs  `count` sections of five Q14 coefficients: [0] and [2] feed back the older / newer state (w[n-2], w[n-1]), [1], [3], [4] weight w[n-2],
 *               w[n-1] and the new state w[n] in the output
 * @param state  `count` sections of two shorts: state[0] = w[n-2], state[1] = w[n-1]; updated
 * @param count  number of sections (0 returns `seed`; must not be negative)
 * @return the output of the last section
 * @note  The new state w[n] = input + feedback terms (feedback products rounded) is saturated to -32767 .. 32767 before it is stored; the output terms are
 *        truncated Q14 products summed as shorts.
 */
short FPM_iir_filt(short seed, const short* coefs, short* state, short count)
{
  short s0;
  short coefTmp;
  short c1;
  short s1;
  int acc;
  const short* c4p;

  acc = (int)seed;
  while (count != 0)
  {
    s0 = *state;
    coefTmp = *coefs;
    c1 = coefs[1];
    s1 = state[1];
    *state = s1;
    acc = acc + (((int)coefTmp * (int)s0 + 0x2000) >> 0xe) + (((int)coefs[2] * (int)s1 + 0x2000) >> 0xe);
    coefTmp = coefs[3];
    c4p = coefs + 4;
    {
      int newState = 0x7fff;
      if (acc < 0x8000 && (newState = -0x7fff, -0x8000 < acc))
      {
        newState = (short)acc;
      }
      state[1] = (short)newState;
      state = state + 2;
      coefs = coefs + 5;
      acc = (short)((short)(((int)c1 * (int)s0) >> 0xe) + (short)(((int)coefTmp * (int)s1) >> 0xe) + (short)(((int)*c4p * newState) >> 0xe));
    }
    count = count + -1;
  }
  return (short)acc;
}

/**
 * @brief Filter a block in place with a direct-form-I biquad cascade.
 * @param data      samples, replaced by the filtered samples
 * @param coefs     `sections` sections of five Q14 coefficients: [0], [1], [2] weight x[n], x[n-2], x[n-1]; [3], [4] weight y[n-2], y[n-1] (subtracted)
 * @param state     `sections` sections of four shorts: x[n-2], x[n-1], y[n-2], y[n-1] of that section; updated
 * @param sections  number of sections (0 leaves the data unchanged; must not be negative)
 * @param count     number of samples (<= 0 does nothing)
 * @note  Products are truncated (>> 14), the section output is not saturated.
 */
void FPM_iir_filt_II(short* data, short* coefs, short* state, short sections, short count)
{
  short* c4p;
  short c1;
  short c0;
  short x2;
  short x1;
  short c2;
  short y2;
  short c3;
  short sig;
  short secLeft;
  short* st;
  short* cf;
  int secDone;
  int i;

  i = 0;
  if (0 < count)
  {
    do
    {
      sig = data[i];
      secDone = (short)(sections + -1) == -1;
      st = state;
      cf = coefs;
      secLeft = sections + -1;
      while (!secDone)
      {
        c1 = cf[1];
        c0 = *cf;
        x2 = *st;
        x1 = st[1];
        *st = x1;
        c2 = cf[2];
        st[1] = sig;
        y2 = st[2];
        c3 = cf[3];
        st[2] = st[3];
        c4p = cf + 4;
        cf = cf + 5;
        sig = (short)(((int)c0 * (int)sig + (int)x2 * (int)c1 + (int)x1 * (int)c2) >> 0xe) - (short)(((int)y2 * (int)c3 + (int)st[3] * (int)*c4p) >> 0xe);
        st[3] = sig;
        st = st + 4;
        secDone = secLeft == 0;
        secLeft = secLeft + -1;
      }
      data[i] = sig;
      i = i + 1;
    } while (i < count);
  }
}

/**
 * @brief Run a block through the direct-form-II cascade of FPM_iir_filt(), in place.
 * @param unusedData  not used
 * @param samples     samples, replaced by the filtered samples
 * @param coefs       `sections` sections of five Q14 coefficients (see FPM_iir_filt())
 * @param state       `sections` sections of two shorts, updated
 * @param sections    number of sections (must not be negative)
 * @param count       number of samples (<= 0 does nothing)
 * @note  Nothing in src/ calls it.
 */
void FPM_iir_filt_block(short* unusedData, short* samples, short* coefs, short* state, short sections, short count)
{
  short s0;
  short coefTmp;
  short c1;
  short s1;
  int acc;
  int newState = 0;
  short secLeft;
  short* st;
  short* c4p;
  short* cf;
  int secDone;
  short n = 0;

  if (0 < count)
  {
    do
    {
      secDone = (short)(sections + -1) == -1;
      acc = (int)*samples;
      st = state;
      cf = coefs;
      secLeft = sections + -1;
      while (!secDone)
      {
        s0 = *st;
        coefTmp = *cf;
        c1 = cf[1];
        s1 = st[1];
        *st = s1;
        acc = acc + (((int)coefTmp * (int)s0 + 0x2000) >> 0xe) + (((int)cf[2] * (int)s1 + 0x2000) >> 0xe);
        coefTmp = cf[3];
        c4p = cf + 4;
        newState = 0x7fff;
        if (acc < 0x8000)
        {
          newState = -0x7fff;
          if (-0x8000 < acc)
          {
            newState = (short)acc;
          }
        }
        st[1] = (short)newState;
        st = st + 2;
        cf = cf + 5;
        acc = (short)((short)(((int)c1 * (int)s0) >> 0xe) + (short)(((int)coefTmp * (int)s1) >> 0xe) + (short)(((int)*c4p * newState) >> 0xe));
        secDone = secLeft == 0;
        secLeft = secLeft + -1;
      }
      n = n + 1;
      *samples = (short)acc;
      samples = samples + 1;
    } while (n < count);
  }
}
