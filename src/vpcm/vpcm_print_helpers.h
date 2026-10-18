/**
 * @file vpcm_print_helpers.h
 * @brief Helpers for the debug prints of extended-precision values: the sign character, the integer part and the "%c%d.%0Nd" print (V.90 equalizer reports).
 *
 * All functions are `static inline`, so every including file gets its own copy.  The same job with a Split result is done by vpcm_split_value.h.
 */
#ifndef VPCM_PRINT_HELPERS_H
#define VPCM_PRINT_HELPERS_H

#include "core/dsp_math.h"
#include "core/dsp_intmath.h"
#include "v34/v34_api.h"

typedef long double LD;

/** @brief Sign character of a print: '+' (0x2b) when v > 0 or v is a NaN, else '-' (0x2d). */
static inline int signChar(LD v) { return (v <= 0.0L) ? 0x2d : 0x2b; }

/** @brief |v| truncated to an int, the integer part of a print. */
static inline int intPart(LD v) { return dsp_trunc_i32(v < 0.0L ? -v : v); }

/**
 * @brief Print v through edprintf() as "%c%d.%0Nd".
 * @param fmt      format with a sign character, an integer and a fraction field
 * @param v        value
 * @param signSrc  value whose sign selects the sign character
 * @param scale    fraction scale (10^N)
 */
static inline void printFix(const char* fmt, LD v, LD signSrc, LD scale)
{
  int ip = dsp_trunc_i32(v);
  int aip = dsp_trunc_i32(__builtin_fabsl(v));
  int fr = absI32(dsp_trunc_i32((v - (LD)ip) * scale));
  edprintf(fmt, signChar(signSrc), aip, fr);
}
/** @brief printFix() of a float, taking the sign from the value itself. */
static inline void printFloat(const char* fmt, float v, LD scale) { printFix(fmt, (LD)v, (LD)v, scale); }

#endif /* VPCM_PRINT_HELPERS_H */
