/**
 * @file vpcm_split_value.h
 * @brief splitValue(): the "%c%d.%0Nd" arguments of a float printed without the C library, and edFix(), which prints them.
 *
 * sign is '+' (0x2b) unless v <= 0 (a NaN prints '+', zero prints '-'), ip = trunc(|v|), fp = |trunc((v - trunc v) * scale)| with the fraction formed in
 * extended precision.  The three-argument form takes the sign from another value `sv`.  All functions are `static inline`, so every including file
 * gets its own copy.
 */
#ifndef VPCM_SPLIT_VALUE_H
#define VPCM_SPLIT_VALUE_H

#include "vpcm_split.h"
#include "core/dsp_math.h"
#include "core/dsp_intmath.h"
#include "v34/v34_api.h"

typedef long double LD;

/**
 * @brief Split v into the arguments of a "%c%d.%0Nd" print.
 * @param v      value
 * @param scale  fraction scale (10^N)
 * @return sign character, integer part and fraction digits
 */
static inline Split splitValue(LD v, LD scale)
{
  Split s;
  int t = dsp_trunc_i32(v); /* truncating conversion */
  s.ip = dsp_trunc_i32(__builtin_fabsl(v));
  LD frac = v - (LD)t;
  s.fp = absI32(dsp_trunc_i32(frac * scale));
  s.sign = (v <= 0.0L) ? 0x2d : 0x2b; /* v > 0 or unordered (NaN) gives '+' */
  return s;
}
/** @brief As above, but the sign character is taken from `sv` instead of `v`. */
static inline Split splitValue(LD v, LD scale, LD sv)
{
  Split s;
  int t = dsp_trunc_i32(v);
  s.ip = dsp_trunc_i32(__builtin_fabsl(v));
  LD frac = v - (LD)t;
  s.fp = absI32(dsp_trunc_i32(frac * scale));
  s.sign = (sv <= 0.0L) ? 0x2d : 0x2b;
  return s;
}
/** @brief Print v through edprintf() with the format `fmt` ("%c%d.%0Nd" style) using splitValue(v, scale). */
static inline void edFix(const char* fmt, LD v, LD scale)
{
  Split s = splitValue(v, scale);
  edprintf(fmt, s.sign, s.ip, s.fp);
}

#endif /* VPCM_SPLIT_VALUE_H */
