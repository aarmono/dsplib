/**
 * @file v34_hs_bytes.h
 * @brief ld16 / ld32 / st16 / st32: unaligned little-endian loads and stores at a byte offset of a buffer (C++).
 *
 * The handshake files use these to reach 16- and 32-bit fields of the V.34 object and of its sub-records that have no named member (byte-offset
 * regions), and to read the sample / bit buffers.  Each helper is a `memcpy` of 2 or 4 bytes, so there is no alignment or aliasing requirement; the
 * value has the host byte order (little-endian on every supported target).  `static inline`: each including file gets its own copy.
 */
#ifndef V34_HS_BYTES_H
#define V34_HS_BYTES_H

#include <stdint.h>

static inline uint16_t ld16(const uint8_t* base, unsigned off)
{
  uint16_t v;
  __builtin_memcpy(&v, base + off, 2);
  return v;
}
static inline uint32_t ld32(const uint8_t* base, unsigned off)
{
  uint32_t v;
  __builtin_memcpy(&v, base + off, 4);
  return v;
}
static inline void st16(uint8_t* base, unsigned off, unsigned v)
{
  uint16_t x = (uint16_t)v;
  __builtin_memcpy(base + off, &x, 2);
}
static inline void st32(uint8_t* base, unsigned off, uint32_t v) { __builtin_memcpy(base + off, &v, 4); }

#endif /* V34_HS_BYTES_H */
