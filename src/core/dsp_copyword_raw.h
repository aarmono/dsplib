/**
 * @file dsp_copyword_raw.h
 * @brief copyWord(void *, const void *): the untyped form of dsp_copyword.h (C++).
 *
 * A separate header because the typed and the untyped overload must not meet in one file: a call with float pointers would pick the typed one.
 */
#ifndef DSP_COPYWORD_RAW_H
#define DSP_COPYWORD_RAW_H

#include <stdint.h>
#include "dsp_asm.h"

/** Copy the 32-bit cell at `src` to `dst` as an integer word, so that a float cell (including a signalling NaN) is preserved bit for bit. */
static inline void copyWord(void* dst, const void* src)
{
  uint32_t w;
  __builtin_memcpy(&w, src, sizeof w);
  OPTIMIZER_HIDE_VAR(w);
  __builtin_memcpy(dst, &w, sizeof w);
}

#endif /* DSP_COPYWORD_RAW_H */
