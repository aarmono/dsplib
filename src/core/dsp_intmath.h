/**
 * @file dsp_intmath.h
 * @brief Small integer helpers shared by many files (C and C++): absI32(), neg16(), WADD().
 *
 * They are `static inline` (or macros) and are spelled the way the generated code is wanted at -O0 (a wrapping add, an abs by sign mask), which is why
 * they are not plain `abs()` or casts at the call sites.
 */
#ifndef DSP_INTMATH_H
#define DSP_INTMATH_H

/** abs() of an int by the sign-mask method (`(v ^ m) - m`, m = v >> 31); INT_MIN stays INT_MIN, with no signed-overflow undefined behaviour. */
static inline int absI32(int v)
{
  unsigned m = (unsigned)(v >> 31);
  return (int)(((unsigned)v ^ m) - m);
}

/** The negation of a 16-bit value, truncated to 16 bits (-32768 stays -32768). */
static inline short neg16(short v) { return (short)(-(int)v); }

/** The wrapping 32-bit sum of two ints: formed unsigned, so an overflow wraps instead of being signed-overflow undefined behaviour (fax tier files). */
#define WADD(a, b) ((int)((unsigned int)(a) + (unsigned int)(b)))

#endif /* DSP_INTMATH_H */
