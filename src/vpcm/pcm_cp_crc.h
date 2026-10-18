/**
 * @file pcm_cp_crc.h
 * @brief CRC-16 helpers of the V.90 / V.92 CP and MP sequences.
 *
 * Shared by V90MP (v90_mp.cpp), V90CP (v90_cp.cpp) and V92CP (v92_cp.cpp) for calcCRC / evaluateCRC / infoToBits / bitsToInfo.  The CRC register is
 * 16 bytes, one bit (0 or 1) per byte, with crc[0] the output stage.  The helpers are `static inline`, and each user includes this header at a fixed place
 * between the same neighbours, so every object file compiles exactly as it did with a private copy.
 */
#ifndef CP_CRC_H
#define CP_CRC_H

#include <stdint.h>

/**
 * @brief Clock the CRC-16 register over a bit buffer.
 * @param crc    the 16-stage register, one bit per byte, updated in place
 * @param bits   bit buffer, one bit per byte, in frames of 17 slots (a 0 start bit followed by 16 data bits)
 * @param bound  index one past the last bit to process
 * @note Processing starts at index 0x12 (the first frame is skipped).  Every index that is a multiple of 17 (a start bit) is skipped.  The feedback is
 *       (crc[0] + bit) computed in a byte; it is added into stages 3 and 10 (taps x^12 and x^5) and becomes the new stage 15.  If `bound` <= 0x12 the
 *       register is not touched.
 */
static inline void crcRun(uint8_t* crc, const uint8_t* bits, unsigned bound)
{
  unsigned i = 0x12;
  if (i < bound)
  {
    do
    {
      uint8_t fb;
      int k;
      if (i % 17 == 0)
        i++;
      fb = (uint8_t)(crc[0] + bits[i]);
      i++;
      for (k = 0; k < 15; k++)
        crc[k] = crc[k + 1];
      crc[3] = (uint8_t)((crc[3] + fb) & 1);
      crc[10] = (uint8_t)((crc[10] + fb) & 1);
      crc[15] = (uint8_t)(fb & 1);
    } while (i < bound);
  }
}

/**
 * @brief Compare two CRC registers.
 * @return the sum of |crc[k] - other[k]| over the 16 stages; 0 means equal (the sum is at most 16, so the 8-bit accumulator cannot wrap)
 */
static inline uint8_t crcDiffSum(const uint8_t* crc, const uint8_t* other)
{
  uint8_t sum = 0;
  int k;
  for (k = 0; k < 16; k++)
  {
    int d = (int)crc[k] - (int)other[k];
    if (d < 0)
      d = -d;
    sum = (uint8_t)(sum + d);
  }
  return sum;
}

/** @brief Set all 16 stages of the register to 1. */
static inline void crcOnes(uint8_t* crc)
{
  int k;
  for (k = 0; k < 16; k++)
    crc[k] = 1;
}

#endif /* CP_CRC_H */
