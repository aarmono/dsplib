/**
 * @file v32_math.c
 * @brief Small fixed-point math leaves shared by the V.32 receive path and the status report: arctangent, single-tap LMS update, base-10 logarithm.
 *
 * - FPM_atan(): quarter-circle arctangent lookup with linear refinement; the carrier-recovery loop (FPM_SRE_recover) and the equaliser
 *   (FPM_FSE_receive) use it to turn a rectangular pair into a phase angle.
 * - FPM_lmsupd(): one LMS coefficient update over a circular history, called by FPM_FSE_receive().
 * - FPM_log10(): scaled base-10 logarithm used by V32FP_status() for the signal-level report.
 *
 * All three are leaves (FPM_atan calls FPM_div, FPM_log10 may print one debug line) and keep no state of their own.  The expressions use fixed
 * intermediate widths; narrowing an intermediate to 16 bits changes the result (see FPM_atan).
 */

#include "fpm/fpm_tables.h"
#include "v32_tables.h"
#include "v32_internal.h"
#include "core/dsp_host.h"
#include "fpm/fpm.h"

/**
 * @brief Quarter-circle arctangent lookup with linear refinement.
 * @param i          in-phase component; only its sign and the ratio |i| / |q| matter
 * @param q          quadrature component, same scale as `i`
 * @param angle_out  receives the angle of (i, q) as a fraction of a full turn, 0x8000 = 360 degrees (range [0, 0x8000))
 *
 * The components are 16-bit; the callers shift their accumulators down into that range first.  The axes are handled directly (q == 0 gives
 * 0x2000 or 0x6000, i == 0 gives 0 or 0x4000; both zero gives 0x2000).  Otherwise the smaller magnitude is divided by the larger with FPM_div(),
 * the ratio indexes FPM_atan_table (for ratios above 0x7e, in steps of 1/128) and the result is scaled by 0x145f / 2^15 (1 / 2 pi).  The
 * octant then places the angle.
 * @note The scale step is evaluated in 32-bit `int`: the product reaches tens of millions, so truncating it to 16 bits before the shift would
 *       corrupt large table values.
 */
void FPM_atan(short i, short q, unsigned short* angle_out)
{
  unsigned short angle;
  unsigned short iAbs16;
  unsigned int qBits;
  unsigned int iBits;
  unsigned short qAbs16;
  unsigned int iAbs;
  unsigned int qAbs;
  short shiftOut;

  qBits = (unsigned int)(unsigned short)q;
  iBits = (unsigned int)(unsigned short)i;
  if (qBits == 0)
  {
    *angle_out = (unsigned short)(((int)i >> 15) & 0x4000) + 0x2000;
    return;
  }
  if (iBits == 0)
  {
    *angle_out = (unsigned short)(((int)q >> 15) & 0x4000);
    return;
  }
  /* |i| and |q| as 16-bit unsigned magnitudes */
  iAbs = (unsigned int)(unsigned short)((((int)i >> 15) ^ i) - ((int)i >> 15));
  iBits = iAbs & 0xffff;
  qBits = (unsigned int)(unsigned short)((((int)q >> 15) ^ q) - ((int)q >> 15));
  qAbs = qBits & 0xffff;
  iAbs16 = (unsigned short)iAbs;
  qAbs16 = (unsigned short)qBits;
  {
    /* quotient of the smaller magnitude by the larger, an unsigned Q15 value (FPM_div stores the same 16 bits through a `short *`; the
     * multiply below must zero-extend it) */
    unsigned short ratio;
    unsigned int prod;
    unsigned int shiftedIdx;
    unsigned short idxLow;

    if (iAbs16 < qAbs16)
    {
      FPM_div((unsigned short)qAbs, (short*)&ratio, &shiftOut);
      prod = (unsigned int)ratio * iBits;
    }
    else
    {
      FPM_div((unsigned short)iBits, (short*)&ratio, &shiftOut);
      prod = (unsigned int)ratio * qAbs;
    }
    /* table index: ratio * magnitude, renormalised by the shift FPM_div reported */
    shiftedIdx = (unsigned int)(((int)prod >> 0xf) << ((unsigned char)shiftOut & 0x1f));
    idxLow = (unsigned short)shiftedIdx;
    if (0x7e < idxLow)
    {
      idxLow = (unsigned short)FPM_atan_table[(shiftedIdx & 0xffff) >> 7];
    }
    /* scale to a fraction of a turn; the whole multiply-add is done in `int` and truncated once, at the end */
    idxLow = (unsigned short)(((int)(short)idxLow * 0x145f + 0x4000) >> 0xf);

    /* place the refined angle in its octant (|i| < |q|: near the q axis; otherwise near the i axis) */
    if (iAbs16 < qAbs16)
    {
      if (q < 0)
      {
        angle = idxLow + 0x4000;
        if (-1 < i)
        {
          angle = 0x4000 - idxLow;
        }
      }
      else
      {
        angle = idxLow;
        if (i < 0)
        {
          angle = 0x7fff - idxLow;
        }
      }
    }
    else if (i < 0)
    {
      angle = idxLow + 0x6000;
      if (q < 0)
      {
        angle = 0x6000 - idxLow;
      }
    }
    else
    {
      angle = idxLow + 0x2000;
      if (-1 < q)
      {
        angle = 0x2000 - idxLow;
      }
    }
  }
  *angle_out = angle;
}

/**
 * @brief LMS coefficient update over a circular history.
 * @param taps         coefficients (Q15), `tapCount` entries, updated in place
 * @param historyBase  base of the circular 16-bit history buffer, `tapCount` entries
 * @param histIdx      index of the newest history entry
 * @param tapCount     number of taps and of history entries
 * @param gain         step size; each tap gains `(history * gain + 0x20000) >> 18`
 *
 * Tap k is paired with history entry `histIdx - k`, wrapping from index 0 to `tapCount - 1`.  Both loops add the rounded product to the
 * existing tap (16-bit wrap, no saturation).
 * @warning No range check: `histIdx` must be below `tapCount` and both buffers must hold `tapCount` entries.
 */
void FPM_lmsupd(short* taps, char* historyBase, short histIdx, short tapCount, short gain)
{
  short histPosNext;
  int histPos;

  histPos = (int)histIdx;
  histPosNext = histIdx;
  while (-1 < histPosNext)
  {
    histPosNext = (short)histPos + -1;
    *taps = (short)(((int)*(short*)(historyBase + histPos * 2) * (int)gain + 0x20000) >> 0x12) + *taps;
    histPos = (int)histPosNext;
    taps = taps + 1;
  }
  while (1)
  {
    tapCount = tapCount + -1;
    if (tapCount <= histIdx)
      break;
    *taps = (short)(((int)*(short*)(historyBase + tapCount * 2) * (int)gain + 0x20000) >> 0x12) + *taps;
    taps = taps + 1;
  }
}

/**
 * @brief Fixed-point base-10 logarithm of `mantissa * 2^exponent`, scaled for the status report.
 * @param mantissa  unsigned 16-bit mantissa; 0 is an error
 * @param exponent  binary exponent applied to the mantissa
 * @return `FPM_log10_table[]` value for the normalised mantissa (shifted right by 3) plus `-0x4cc` per power of two, as a signed 16-bit value;
 *         0 for a zero mantissa (after printing "Fatal error: log10 of zero!" when the debug level is above 1)
 *
 * The mantissa is doubled until bit 14 is set; the number of doublings is added to the exponent.  The table is indexed by the rounded top bits
 * of the normalised mantissa.  V32FP_status() is the only caller.
 */
int FPM_log10(unsigned short mantissa, short exponent)
{
  short doublings;

  if (mantissa != 0)
  {
    doublings = 0;
    if (mantissa < 0x4000)
    {
      doublings = 0;
      do
      {
        mantissa = (unsigned short)(mantissa * 2);
        doublings = doublings + 1;
      } while (mantissa < 0x4000);
    }
    /* (exponent + doublings) stays `int` through the multiply; truncating it to 16 bits would change the result */
    return (int)(short)((FPM_log10_table[((((int)mantissa + 0x40) >> 7) - 0x80) & 0xffff] >> 3) + (exponent + doublings) * -0x4cc);
  }
  if (1 < dsplibs_debug_level)
  {
    dsplibs_debug_printf("Fatal error: log10 of zero!\n");
  }
  return 0;
}
