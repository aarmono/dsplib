/**
 * @file dsp_floatbits.h
 * @brief Float cells handled as raw 32-bit words, and float stores that really round (C++).
 *
 * Plain C++ float copies compile to `flds` / `fstps` on i386, which quiets a signalling NaN, and a float expression may stay unrounded in a register.
 * The helpers here move float cells and float arguments as integer words and force a float store to round, so the bit patterns and the rounding
 * points of the float classes are exact.  They are `static inline` and must not be replaced by plain casts or assignments.
 */
#ifndef DSP_FLOATBITS_H
#define DSP_FLOATBITS_H

#include <stdint.h>
#include "dsp_asm.h"

typedef long double LD;

/**
 * Store `v` into the float cell `*dst` and return the stored (rounded) value.  The cell is accessed through a volatile, so the store really rounds to
 * float and the next use reloads the rounded value instead of keeping the unrounded register.
 */
static inline float storeF(float* dst, LD v)
{
  *(volatile float*)dst = (float)v;
  return *(volatile float*)dst;
}
/** The same for a float that only lives in a temporary slot: return `v` rounded to float. */
static inline float storeF(LD v)
{
  volatile float s = (float)v;
  return s;
}
/** Read the float cell `*src` (through a volatile, so the value is loaded from memory). */
static inline float loadF(const float* src) { return *(const volatile float*)src; }

/** The raw 32-bit word of a float / int cell value. */
static inline uint32_t floatBits(float f)
{
  uint32_t w;
  __builtin_memcpy(&w, &f, sizeof w);
  return w;
}

/** The float with the raw 32-bit word `w` as its bit pattern. */
static inline float bitsFloat(uint32_t w)
{
  float f;
  __builtin_memcpy(&f, &w, sizeof f);
  return f;
}

/** Set a float member from a constant word (a 32-bit integer store, no float conversion). */
static inline void setWord(float* dst, unsigned w) { __builtin_memcpy(dst, &w, sizeof w); }

/** Set a float cell from a raw word (a 32-bit integer store). */
static inline void storeWord(float* dst, uint32_t w) { __builtin_memcpy(dst, &w, sizeof w); }

/** Store a float argument as its raw word, kept in an integer register (otherwise the compiler picks `flds` / `fstps` and quiets a signalling NaN). */
static inline void storeArgWord(float* dst, float v)
{
  uint32_t word;
  __builtin_memcpy(&word, &v, sizeof word);
  OPTIMIZER_HIDE_VAR(word);
  __builtin_memcpy(dst, &word, sizeof word);
}

/** Copy a float member as a raw 32-bit word (keeps NaN payloads intact; no floating-point load). */
static inline void copyWordF(float* dst, const float* src)
{
  uint32_t w;
  __builtin_memcpy(&w, src, 4);
  __builtin_memcpy(dst, &w, 4);
}

#endif /* DSP_FLOATBITS_H */
