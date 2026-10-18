/**
 * @file fpm_div32.c
 * @brief FPM_div_32: normalising reciprocal of a 32-bit unsigned value (the 32-bit counterpart of FPM_div() in fpm_math.c).
 */
#include "fpm.h"
#include "fpm_tables.h"
#include "core/dsp_host.h"

/**
 * @brief Normalising reciprocal of a 32-bit unsigned value.
 * @param x      value (non-zero)
 * @param mant   out: Q15 mantissa from FPM_div_table, indexed by the top eight bits of the normalised value; mant * (x << shift) / 65536 is about 2^30
 * @param shift  out: number of doublings that set bit 31 of x (0 when it is already set)
 * @return 0, or 1 for x == 0 (outputs untouched; at debug level above 1 prints "Fatal error: Division by zero!")
 * @bug   When the top 16 bits of the normalised value are 0xff80 or more the table index is 128, one past the end of the table, and the mantissa is
 *        forced to 0 instead of about 16384 (the same defect as FPM_div()).
 */
int FPM_div_32(unsigned int x, short* mant, short* shift)
{
  unsigned short hi;
  short sh = 0;
  int idx;

  if (x == 0)
  {
    if (dsplibs_debug_level > 1)
      dsplibs_debug_printf("Fatal error: Division by zero!\n");
    return 1;
  }

  /* normalise: double x until bit 31 is set */
  if ((int)x >= 0)
  {
    do
    {
      sh = (short)(sh + 1);
      x = x + x;
    } while ((int)x >= 0);
  }

  /* table lookup on the top 16 bits */
  hi = (unsigned short)(x >> 16);
  idx = (int)(((unsigned int)hi + 0x80u) >> 8) - 0x80;
  *mant = (idx == 128) ? 0 : FPM_div_table[idx & 0x7f];
  *shift = sh;
  return 0;
}
