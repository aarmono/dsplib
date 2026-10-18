/**
 * @file v34_sar32.h
 * @brief sar32(): arithmetic shift right of a 32-bit word, for the V.34 receive files.
 *
 * `static inline`, included by v34_receiver.c and v34_echo_rx_timing.c.  It lives in src/v34/ and is included bare, so a test script that compiles one of
 * these files without -Isrc still finds it.
 */
#ifndef V34_SAR32_H
#define V34_SAR32_H

#include <stdint.h>

/**
 * @brief Arithmetic shift right of a value computed with 32-bit wrap-around.
 * @param v  the value, as built from multiply / add chains that wrap modulo 2^32 (callers build it as uint32_t, because C signed overflow is undefined)
 * @param n  shift count, 0 .. 31
 * @return v interpreted as signed 32-bit, shifted right with sign extension
 */
static inline int32_t sar32(uint32_t v, int n) { return (int32_t)v >> n; }

#endif /* V34_SAR32_H */
