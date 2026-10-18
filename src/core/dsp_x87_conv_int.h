/**
 * @file dsp_x87_conv_int.h
 * @brief trunc16(): `fistp WORD` with the truncating control word, returning int (C++).
 *
 * NaN and values outside (-32769, 32768) give the integer indefinite -32768.  The int16_t form is dsp_x87_conv.h (the two must not be included in
 * one file).
 */
#ifndef DSP_X87_CONV_INT_H
#define DSP_X87_CONV_INT_H

typedef long double LD;

/** `v` truncated to a 16-bit value as `fistp WORD` does it, returned as int; NaN and out-of-range values give -32768. */
static inline int trunc16(LD v)
{
  if (!(v > -32769.0L && v < 32768.0L))
    return -32768;
  return (int)v;
}

#endif /* DSP_X87_CONV_INT_H */
