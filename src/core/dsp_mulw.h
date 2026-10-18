/**
 * @file dsp_mulw.h
 * @brief mulw(): 32-bit multiplication with wrap-around, the product formed unsigned.
 *
 * Plain `static` (not `static inline`): an unused static function is still emitted at -O0, so include this header only in a file that calls mulw().
 */
#ifndef DSP_MULW_H
#define DSP_MULW_H

/** `a * b` truncated to 32 bits (two's complement wrap; no signed-overflow undefined behaviour). */
static int mulw(int a, int b) { return (int)((unsigned)a * (unsigned)b); }

#endif /* DSP_MULW_H */
