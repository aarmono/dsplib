/**
 * @file fpm_math.c
 * @brief FPM fixed-point math primitives: phasor generation, RMS, square root and reciprocal.
 *
 * Small stateless helpers of the FPM toolkit, built on the lookup tables of fpm_tables.c (FPM_sin_table, FPM_cos_table, FPM_sqrt_table, FPM_div_table and the
 * quadrant sign tables).  Users: FPM_phasor() / FPM_phasor_demod() generate and advance a rotating phasor (the V.22, V.32 and fax equalizers and the FSK
 * demodulators); FPM_rms() and FPM_div() serve the AGC (fpm_agc.c); FPM_sqrt_dp() is the square root behind FPM_rms().  FPM_div_32() (fpm_div32.c) is the
 * 32-bit counterpart of FPM_div().  Phases are 16-bit angles in which a full turn is 0x8000.
 */
#include "fpm.h"
#include "fpm_tables.h"
#include "core/dsp_host.h"

/**
 * @brief Evaluate the phasor of an angle and advance the angle.
 * @param ph  four shorts: ph[0] = angle (in/out; a full turn is 0x8000, valid range 0 .. 0x7fff), ph[1] = cosine out, ph[2] = sine out,
 *            ph[3] = angle increment added to ph[0] (wraps modulo 0x8000 when the sum exceeds 0x7fff)
 *
 * The quadrant is angle >> 13; within a quadrant the value is linearly interpolated between two table entries (32 angle steps per entry, 256 entries per
 * quadrant, mirrored in odd quadrants) and multiplied by the quadrant's sign factor (+-16384 from FPM_cos_sign_ext / FPM_sin_sign_ext), so the outputs are
 * Q14 (16384 = 1.0).
 * @bug   An angle with bit 15 set (negative as a short) selects rows 0 .. 3 of the sign tables.  The interpolated magnitude is right, but the factors of
 *        those rows are not the quadrant signs: the sine factors are 28620, -25834, 12917 and 0, and the cosine factors 16384, 16384, -16384, -16384.
 *        ph[1] / ph[2] are therefore not a unit phasor.  Callers that feed a full-circle angle (the V.32 equalizer passes FPM_atan() output) reach this;
 *        keep the angle in 0 .. 0x7fff for correct values.  A negative increment can also take the stored angle below 0, which is not wrapped.
 */
void FPM_phasor(short* ph)
{
  unsigned short sinLo;
  short frac;
  unsigned int idx;
  int phase;
  unsigned int octSel;

  /* split the angle into table index, interpolation fraction and quadrant; odd quadrants run the table backwards */
  phase = (int)*ph;
  idx = (unsigned int)(phase >> 5);
  octSel = (unsigned int)(phase >> 0xd);
  frac = *ph - (short)(idx << 5);
  if ((octSel & 1) != 0)
  {
    idx = ~idx;
    frac = 0x20 - frac;
  }
  idx = idx & 0xff;

  /* cosine output, then the angle update, then the sine output */
  sinLo = (unsigned short)FPM_sin_table[idx];
  ph[1] = (short)((int)((((int)(((unsigned int)(unsigned short)FPM_cos_table[idx + 1] - (unsigned int)(unsigned short)FPM_cos_table[idx]) * (int)frac) >> 5) +
                             (unsigned int)(unsigned short)FPM_cos_table[idx] &
                         0xffff) *
                        /* the quadrant is signed (-4 .. 3); +4 maps it to the 8-entry sign tables */
                        (int)*(short*)((char*)&FPM_cos_sign_ext + (octSel + 4) * 2)) >>
                  0xf);
  phase = phase + ph[3];
  ph[2] =
      (short)((int)((((int)((int)frac * ((unsigned int)(unsigned short)FPM_sin_table[idx + 1] - (unsigned int)sinLo)) >> 5) + (unsigned int)sinLo & 0xffff) *
                    (int)*(short*)((char*)&FPM_sin_sign_ext + (octSel + 4) * 2)) >>
              0xf);

  /* wrap the angle */
  if (0x7fff < phase)
  {
    phase = phase + -0x8000;
  }
  *ph = (short)phase;
}

/**
 * @brief Like FPM_phasor(), but computes only the cosine and advances the angle.
 * @param ph  as for FPM_phasor(): ph[0] angle (in/out), ph[1] cosine out (Q14), ph[3] increment; ph[2] is not touched
 * @bug   The same out-of-range behaviour as FPM_phasor() for an angle with bit 15 set.
 */
void FPM_phasor_demod(short* ph)
{
  short frac;
  int newPhase;
  unsigned int idx;
  int phase;

  /* split the angle into table index, interpolation fraction and quadrant */
  phase = (int)*ph;
  idx = (unsigned int)(phase >> 5);
  frac = *ph - (short)(idx << 5);
  if (((phase >> 0xd) & 1) != 0)
  {
    idx = ~idx;
    frac = 0x20 - frac;
  }

  /* advance the angle and compute the cosine output */
  newPhase = phase + ph[3];
  ph[1] = (short)((int)((((int)((int)frac *
                                ((unsigned int)(unsigned short)FPM_cos_table[(idx & 0xff) + 1] - (unsigned int)(unsigned short)FPM_cos_table[idx & 0xff])) >>
                          5) +
                             (unsigned int)(unsigned short)FPM_cos_table[idx & 0xff] &
                         0xffff) *
                        /* signed quadrant: the 8-entry sign table is indexed by quadrant + 4 (see FPM_phasor()) */
                        (int)*(short*)((char*)&FPM_cos_sign_ext + ((phase >> 0xd) + 4) * 2)) >>
                  0xf);

  /* wrap the angle */
  if (0x7fff < newPhase)
  {
    newPhase = newPhase + -0x8000;
  }
  *ph = (short)newPhase;
}

/**
 * @brief Root of the scaled sum of squares of a sample block.
 * @param data   samples
 * @param count  number of samples
 * @return FPM_sqrt_dp() of sum(s * s * 910 / 32768), i.e. about sqrt(sum(s * s) / 36): the RMS of a block of 36 samples, scaled by sqrt(count / 36) otherwise
 * @bug   The sum is a 32-bit signed int: it overflows (undefined behaviour; wraps in practice, and the wrapped negative value is then taken as a huge
 *        unsigned number) once sum(s * s) / 36 exceeds 2^31, e.g. more than about 72 full-scale samples.  The AGC calls it with its block length
 *        (40 in the configurations in use).  The result also saturates at 32703 (FPM_sqrt_dp()).
 */
short FPM_rms(const short* data, short count)
{
  short s;
  int sumSq;

  sumSq = 0;
  while (count != 0)
  {
    s = *data;
    data = data + 1;
    sumSq = sumSq + (short)((s * 0x38e) >> 0xf) * (int)s;
    count = count + -1;
  }
  return (short)FPM_sqrt_dp((unsigned int)sumSq);
}

/**
 * @brief Table-driven integer square root.
 * @param x  unsigned 32-bit value
 * @return about sqrt(x) (0 for x == 0); the largest result is 32703
 *
 * x is shifted left in steps of two bits until it is at least 0x20000000, the top bits select an entry of FPM_sqrt_table (with a half step when the
 * number of doublings is odd), and the entry is shifted right by half the doublings.  The accuracy is that of the table (about 0.2 percent).
 * @warning For x >= 0x40000000 the table index is clamped, so the result is 32703 instead of up to 46341; for x >= 0x80000000 the 16-bit truncation of
 *          x >> 15 can select a lower entry, so the result is not monotonic there.  FPM_rms() cannot produce such values from valid input.
 */
unsigned int FPM_sqrt_dp(unsigned int x)
{
  unsigned int root;
  unsigned int tabIdx;
  unsigned short normShift;
  unsigned short mant;

  root = 0;
  if (x != 0)
  {
    /* normalise: shift x left until it is at least 0x20000000, counting the doublings */
    normShift = 0;
    if (x < 0x20000000)
    {
      normShift = 0;
      do
      {
        x = x * 2;
        normShift = normShift + 1;
      } while (x < 0x20000000);
    }

    /* table lookup (half step for an odd shift), then undo half the normalisation */
    mant = (unsigned short)(x >> 0xf);
    if (normShift != (unsigned short)((normShift >> 1) * 2))
    {
      mant = mant >> 1;
    }
    tabIdx = (unsigned int)(((int)(mant + 0x40) >> 7) - 0x40);
    root = tabIdx & 0xffff;
    if (0xbf < (unsigned short)tabIdx)
    {
      root = 0xbf;
    }
    root = ((unsigned int)(unsigned short)FPM_sqrt_table[root] >> ((unsigned char)(normShift >> 1) & 0x1f)) & 0xffff;
  }
  return root;
}

/**
 * @brief Normalising reciprocal of a 16-bit unsigned value.
 * @param x      value (non-zero)
 * @param mant   out: Q15 mantissa of 1/x, 16384 .. 32768, so that mant * x * 2^shift is about 2^30
 * @param shift  out: number of doublings that make bit 15 of x set (0 .. 15)
 * @return 0, or 1 for x == 0 (outputs untouched; at debug level above 1 prints "Fatal error: Division by zero!")
 *
 * The mantissa comes from FPM_div_table, indexed by the top eight bits of the normalised value.
 * @bug   For a normalised value of 0xff80 or more (x just below a power of two, within 1/128 of it) the table index is 128, one past the end of the
 *        128-entry table, and the mantissa is forced to 0 instead of about 16384.  A caller using it as a gain (FPM_AGC_agc()) then gets a gain of 0.
 */
int FPM_div(unsigned short x, short* mant, short* shift)
{
  short sh;

  if (x != 0)
  {
    int idx;

    /* normalise: double x until bit 15 is set */
    sh = 0;
    if ((short)x > -1)
    {
      sh = 0;
      do
      {
        x = x * 2;
        sh = sh + 1;
      } while ((short)x > -1);
    }

    /* table lookup on the top bits */
    idx = ((int)(x + 0x80) >> 8) - 0x80;
    *mant = (idx == 128) ? 0 : FPM_div_table[idx & 0x7f];
    *shift = sh;
    return 0;
  }

  if (1 < dsplibs_debug_level)
  {
    dsplibs_debug_printf("Fatal error: Division by zero!\n");
  }
  return 1;
}
