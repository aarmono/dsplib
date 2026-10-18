/**
 * @file v22_us.h
 * @brief us(): a signed 16-bit value as unsigned 16-bit (C).
 *
 * Plain `static` (not inline): an unused static function is still emitted at -O0, so include this header only in a file that calls us().
 */
#ifndef V22_US_H
#define V22_US_H

/** The 16-bit pattern of `v` as an unsigned value (0 .. 65535); the call-setup counters are compared as unsigned 16-bit values. */
static unsigned short us(short v) { return (unsigned short)v; }

#endif /* V22_US_H */
