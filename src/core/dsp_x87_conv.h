/**
 * @file dsp_x87_conv.h
 * @brief Float -> integer conversions with the x87 `fistp` semantics (truncating control word), as C++ functions on long double.
 *
 * `fistp WORD` / `fistp QWORD` store the "integer indefinite" (0x8000 / 0x8000000000000000) for NaN and for values out of range, where a C cast is
 * undefined behaviour.  The int-returning form of trunc16() is in dsp_x87_conv_int.h (the two must not be included in one file).
 */
#ifndef DSP_X87_CONV_H
#define DSP_X87_CONV_H

#include <stdint.h>

typedef long double LD;

/** `v` truncated to 16 bits as `fistp WORD` does it with the truncating control word; NaN and values outside (-32769, 32768) give 0x8000. */
static inline int16_t trunc16(LD v)
{
  if (!(v > -32769.0L && v < 32768.0L))
    return (int16_t)0x8000;
  return (int16_t)(int)v;
}

/** The low 32 bits of `v` truncated to 64 bits as `fistp QWORD` does it; NaN and out-of-range values give the integer indefinite, whose low dword is 0. */
static inline uint32_t trunc64lo(LD v)
{
  if (!(v > -9223372036854775809.0L && v < 9223372036854775808.0L))
    return 0u;
  return (uint32_t)(int64_t)v;
}

#endif /* DSP_X87_CONV_H */
