/**
 * @file dsp_copyword.h
 * @brief copyWord(float *, const float *): a 32-bit cell copied as an integer word (C++).
 *
 * The word is held in an integer register, so the compiler does not copy it through the x87 (`flds` / `fstps` would quiet a signalling NaN).
 * The variants are separate headers because some files define a different, asm-free copyWord(float *, const float *) of their own and because
 * overloads must not meet in one file: dsp_copyword_plain.h (plain memcpy, no barrier), dsp_copyword_raw.h (untyped `void *` form).
 */
#ifndef DSP_COPYWORD_H
#define DSP_COPYWORD_H

#include <stdint.h>
#include "dsp_asm.h"

/** Copy the 32-bit cell `*src` to `*dst` as an integer word; the bit pattern (including a signalling NaN) is preserved. */
static inline void copyWord(float* dst, const float* src)
{
  uint32_t w;
  __builtin_memcpy(&w, src, sizeof w);
  OPTIMIZER_HIDE_VAR(w);
  __builtin_memcpy(dst, &w, sizeof w);
}

#endif /* DSP_COPYWORD_H */
