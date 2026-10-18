/**
 * @file dsp_wordof_value.h
 * @brief wordOf(float): a float argument as its raw 32-bit word, passed by value (C++).
 *
 * The word goes through an integer register (the asm barrier), so that the compiler does not copy the float with `flds` / `fstps`, which would quiet a
 * signalling NaN on i386.  Plain `static inline`; the by-reference, always_inline form is dsp_wordarg.h (the two must not be included in one file).
 */
#ifndef DSP_WORDOF_VALUE_H
#define DSP_WORDOF_VALUE_H

#include <stdint.h>
#include "dsp_asm.h"

/** The raw 32-bit word of the float `v`. */
static inline uint32_t wordOf(float v)
{
  uint32_t w;
  __builtin_memcpy(&w, &v, sizeof w);
  OPTIMIZER_HIDE_VAR(w);
  return w;
}

#endif /* DSP_WORDOF_VALUE_H */
