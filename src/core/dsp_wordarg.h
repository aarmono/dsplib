/**
 * @file dsp_wordarg.h
 * @brief wordOf() / floatOf() / WORDARG(): a float argument that travels as a raw 32-bit word (C++).
 *
 * `static inline __attribute__((always_inline))`, because the attribute is part of the code generated at -O0.  A separate header from
 * dsp_floatbits.h and dsp_wordof_value.h because other files define a differently typed wordOf(float) of their own.
 */
#ifndef DSP_WORDARG_H
#define DSP_WORDARG_H

#include <stdint.h>
#include "dsp_asm.h"

/**
 * The raw 32-bit word of the float `v`, taken through an integer register (the asm barrier) so that the compiler does not copy the float with
 * `flds` / `fstps`, which would quiet a signalling NaN on i386.  On x86-64 the float travels in xmm0 unchanged.
 */
static inline __attribute__((always_inline)) uint32_t wordOf(const float& v)
{
  uint32_t w;
  __builtin_memcpy(&w, &v, sizeof w);
  OPTIMIZER_HIDE_VAR(w);
  return w;
}
/** The float with the bit pattern `w` (the inverse of wordOf()). */
static inline __attribute__((always_inline)) float floatOf(uint32_t w)
{
  float f;
  __builtin_memcpy(&f, &w, sizeof f);
  return f;
}

/** The float `v` as the callee receives it: through the integer register and back, so a signalling NaN keeps its bit pattern. */
#define WORDARG(v) floatOf(wordOf(v))

#endif /* DSP_WORDARG_H */
