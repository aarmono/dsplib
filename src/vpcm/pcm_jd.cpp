/**
 * @file pcm_jd.cpp
 * @brief V.90 / V.92 phase-3 "Jd" frames: the holder objects V90Jd and V92Jd with their bit-serial codecs, and the V92MappingParams array helpers.
 *
 * Two classes and four C functions:
 * - V90Jd (15 member functions, 9 used by the library): holds the V.90 Jd frame (rate mask, constellation sizes, look-ahead, CRC-16) in a byte-per-bit
 *   buffer, builds the transmit frame (getBitVector()) and runs the receive state machine one bit at a time (unPackData(), 1 = frame complete).
 * - V92Jd (21 member functions, 14 used): the same for the V.92 Jd data frame (packJdData() / unPackJdData()) plus a second frame for the Jd phase
 *   (packJdPhaseData() / unPackJdPhaseData(), a 16-bit phase value and two constellation bytes).
 * - V92createConstellations() / V92createFilterCoefficients() / V92deleteConstellations() / V92deleteFilterCoefficients(): allocate and free the
 *   arrays of a V92MappingParams block.
 * Member functions marked "unused" in the comments are ported and tested but nothing in the library calls them.
 *
 * Frame (all four builders): 17 one-bits, a 0 start bit, a row of 16 bits, a 0, a row of 16 bits, a 0, the 16 bits of a CRC-16 (registers preset to
 * ones; parity words, taps 3 and 10), four zeros, one byte per bit.  The receivers drop the start bits and check the CRC over the 32 collected bits.
 * Receive state machine (the state word in the object): 0 count the ones (a run of 17 starts a frame), 1 expect the start bit 0, 2 / 4 / 5 collect rows,
 * 3 / 6 expect start bits, 7 collect the CRC and compare, 8 count four trailing bits and report the frame (return value 1).  V92Jd's Jd data codec has
 * one more state (a single unconditional bit, state 5).  A received bit goes in as `(uint8_t)bit`; the tests on it use the whole int.
 *
 * Object byte offsets in the `+0x..` comments are those of pcm_jd.h; `bits_[k]` is at +2 + k.
 *
 * Hazards:
 * - packJdData() zeroes some of the "send" bytes the setters filled (the V.92 rate-mask bits above bit 21 are not sent).
 * - The shift register `crc_` is shared by both V92Jd codecs.
 * - The receive index is a byte: it wraps after 256 bits in the final state, and the bit buffer is indexed with it unchecked (the state machine keeps
 *   it below 0x34 while a frame is being collected).
 * - A received bit is stored before the frame length is tested.
 * - The V92Jd phase frame ends with a marker bit that must be 1 (data frame: 0); the phase frame's marker (+0x79) is set by the constructor.
 *
 * Real C++, no exceptions, no RTTI, freestanding headers only, C++98.  Declarations: pcm_jd.h.
 */

#include "pcm_jd.h"

/* ---- helpers (static inline) ---- */

/* One step of the 16-register parity CRC: s = bit + r0; every register takes its successor, except r3' = (r4 + s) & 1, r10' = (r11 + s) & 1, r15' = s & 1. */
static inline void crcStep(uint32_t* r, uint32_t bit)
{
  uint32_t s = bit + r[0];
  uint32_t t3 = (r[4] + s) & 1;
  uint32_t t10 = (r[11] + s) & 1;
  for (int k = 0; k < 15; k++)
    r[k] = r[k + 1];
  r[3] = t3;
  r[10] = t10;
  r[15] = s & 1;
}

/* Compute the CRC register after n bytes at p, starting from all ones. */
static inline void crcBytes(uint32_t* r, const uint8_t* p, int n)
{
  for (int k = 0; k < 16; k++)
    r[k] = 1;
  for (int k = 0; k < n; k++)
    crcStep(r, p[k]);
}

/* Store the rate mask as send bytes: bits 0 .. 15 at row0, bits 16 .. 16 + n2 - 1 at row1; getMaskBits() reads 16 + 12 bits back. */
static inline void setMaskBits(uint8_t* row0, uint8_t* row1, int n2, int mask)
{
  for (int k = 0; k <= 15; k++)
    row0[k] = (((mask >> k) & 1) != 0) ? 1 : 0;
  for (int k = 0; k < n2; k++)
    row1[k] = (((mask >> (16 + k)) & 1) != 0) ? 1 : 0;
}
static inline int getMaskBits(const uint8_t* row0, const uint8_t* row1)
{
  int m = 0;
  for (int k = 0; k <= 15; k++)
    if (row0[k] != 0)
      m |= 1 << k;
  for (int k = 0; k <= 11; k++)
    if (row1[k] != 0)
      m |= 1 << (16 + k);
  return m;
}

/* Store / read the two look-ahead bits (bit 0, bit 1) of a byte. */
static inline void setLookBits(uint8_t* dst, unsigned char v)
{
  dst[0] = (uint8_t)(v & 1);
  dst[1] = (uint8_t)((v >> 1) & 1);
}
static inline unsigned char lookBits(const uint8_t* src) { return (unsigned char)((src[1] & 1) * 2 + (src[0] & 1)); }

/* Truncate x to a 64-bit integer and return its low 32 bits; NaN and |x| >= 2^63 give 0 (the low word of the x87 integer-indefinite value
 * 0x8000000000000000). */
static inline uint32_t fistpLow32(long double x)
{
  if (!(x > -9223372036854775809.0L && x < 9223372036854775808.0L))
    return 0;
  return (uint32_t)(int64_t)x;
}

/* Base of the frame buffer (+2) and of the V92Jd phase frame buffer (+0x4a), addressed by the byte receive index. */
static inline uint8_t* rawBuf(void* obj) { return (uint8_t*)obj + 2; }
static inline uint8_t* rawPhase(void* obj) { return (uint8_t*)obj + 0x4a; }

/* Sum of |reg[k] - rxbits[k]| over the 16 CRC bits: 0 when the received CRC matches. */
static inline uint32_t crcMismatch(const uint32_t* reg, const uint8_t* rxbits)
{
  uint32_t sum = 0;
  for (int k = 0; k < 16; k++)
  {
    int d = (int)reg[k] - (int)rxbits[k];
    int m = d >> 31;
    sum += (uint32_t)((d ^ m) - m);
  }
  return sum;
}

/* ---- V90Jd ---- */

/**
 * @brief Build a V.90 Jd holder from the phase-2 parameter block.
 * @param p  parameters: maxSpectralShaperLookahead, v34Phase4Constellation, v34RrnConstellation, digitalRateMask
 * @note  Fills the transmit "send" bytes; the receive state is cleared.
 */
V90Jd::V90Jd(V90Parameters* p)
{
  ones_ = 0;
  idx_ = 0;
  uint8_t la = (uint8_t)p->maxSpectralShaperLookahead;
  state_ = 0;
  setLookBits(&bits_[0x31], la);                                   /* +0x33 / +0x34 */
  bits_[0x2f] = (uint8_t)p->v34Phase4Constellation;                /* +0x31 */
  bits_[0x30] = (uint8_t)p->v34RrnConstellation;                   /* +0x32 */
  setMaskBits(&bits_[0x12], &bits_[0x23], 12, p->digitalRateMask); /* +0x14 .. +0x23, +0x25 .. +0x30 */
}

V90Jd::~V90Jd() {}

/* unused: set the look-ahead bits of the send frame */
void V90Jd::setMaxLookahead(unsigned char v) { setLookBits(&bits_[0x31], v); }

/* unused: set the two constellation-size bytes of the send frame */
void V90Jd::setConstelSize(unsigned char a, unsigned char b)
{
  bits_[0x2f] = a;
  bits_[0x30] = b;
}

/* unused: set the rate-mask bits of the send frame */
void V90Jd::setRatesMask(int mask) { setMaskBits(&bits_[0x12], &bits_[0x23], 12, mask); }

/** @brief Rate mask of the last received frame (28 bits: two rows of 16 and 12). */
int V90Jd::getRatesMask() { return getMaskBits(&bits_[0], &bits_[0x10]); } /* +2 .. +0x11, +0x12 .. +0x1d */

/** @brief The two constellation-size bytes of the last received frame. */
void V90Jd::getConstelationSize(unsigned char* a, unsigned char* b)
{
  *a = bits_[0x1c]; /* +0x1e */
  *b = bits_[0x1d]; /* +0x1f */
}

/** @brief The look-ahead value (two bits) of the last received frame. */
unsigned char V90Jd::getMaxLookahead() { return lookBits(&bits_[0x1e]); } /* +0x20 (bit 0), +0x21 (bit 1) */

/* unused: reset the CRC register to all ones */
void V90Jd::resetCrc()
{
  for (int k = 0; k <= 15; k++)
    crc_[k] = 1;
}

/* unused: build the transmit frame (getBitVector() without the return value): ones, start bit, the two rows (+0x14 and +0x25), start bits, CRC bits at
 * +0x36, four zeros */
void V90Jd::packData()
{
  for (int k = 0; k <= 16; k++)
    bits_[k] = 1;  /* +2 .. +0x12 */
  bits_[0x11] = 0; /* +0x13 */
  bits_[0x22] = 0; /* +0x24 */
  bits_[0x33] = 0; /* +0x35 */
  bits_[0x44] = 0; /* +0x46 .. +0x49 */
  bits_[0x45] = 0;
  bits_[0x46] = 0;
  bits_[0x47] = 0;

  /* CRC over the two rows (the register state continues across them) */
  uint32_t r[16];
  for (int k = 0; k <= 15; k++)
    crc_[k] = 1;
  crcBytes(r, &bits_[0x12], 16);
  for (int k = 0; k < 16; k++)
    crcStep(r, bits_[0x23 + k]);
  for (int k = 0; k < 16; k++)
    crc_[k] = r[k];
  for (int k = 0; k <= 15; k++)
    bits_[0x34 + k] = (uint8_t)crc_[k]; /* +0x36 .. +0x45 */
}

/**
 * @brief Build the transmit Jd frame.
 * @return the frame buffer (`bits_`, 0x48 bytes, one bit per byte)
 */
unsigned char* V90Jd::getBitVector()
{
  packData();
  return &bits_[0];
}

/** @brief Restart frame reception (clears the index, the state and the run length). */
void V90Jd::unPackReset()
{
  idx_ = 0;
  state_ = 0;
  ones_ = 0;
}

/**
 * @brief Feed one received bit to the Jd frame receiver.
 * @param bit  received bit (any non-zero value counts as one)
 * @return 1 when a complete frame with a valid CRC has been received (the fields can then be read with the get functions), otherwise 0
 * @note  A wrong start bit or a CRC mismatch restarts reception (unPackReset()).  After a frame the state stays at 8, so further bits keep the index
 *        counting and the function reports a frame again every 256 bits.
 */
int V90Jd::unPackData(int bit)
{
  uint8_t nextOnes = 0;
  if (bit != 0)
    nextOnes = (uint8_t)(ones_ + 1);
  uint8_t* buf = rawBuf(this);
  uint8_t i, n;

  switch (state_)
  {
  case 0: /* count the ones: a run of 17 ends the preamble */
    ones_ = nextOnes;
    if (nextOnes > 0x10)
      state_ = 1;
    return 0;
  case 1: /* the start bit */
    if (bit == 0)
    {
      ones_ = nextOnes;
      state_ = 2;
    }
    else
    {
      unPackReset();
    }
    return 0;
  case 2: /* 16 bits */
    i = idx_;
    ones_ = nextOnes;
    buf[i] = (uint8_t)bit;
    n = (uint8_t)(i + 1);
    idx_ = n;
    if (n == 0x10)
      state_ = 3;
    return 0;
  case 3:
    if (bit != 0)
    {
      unPackReset();
    }
    else
    {
      ones_ = nextOnes;
      state_ = 4;
    }
    return 0;
  case 4: /* 12 bits (up to index 0x1c) */
    i = idx_;
    ones_ = nextOnes;
    buf[i] = (uint8_t)bit;
    n = (uint8_t)(i + 1);
    idx_ = n;
    if (n == 0x1c)
      state_ = 5;
    return 0;
  case 5: /* 4 bits (constellation sizes, look-ahead; up to index 0x20) */
    i = idx_;
    ones_ = nextOnes;
    buf[i] = (uint8_t)bit;
    n = (uint8_t)(i + 1);
    idx_ = n;
    if (n == 0x20)
      state_ = 6;
    return 0;
  case 6:
    if (bit != 0)
    {
      unPackReset();
    }
    else
    {
      state_ = 7;
      ones_ = nextOnes;
    }
    return 0;
  case 7: /* the 16 CRC bits (up to index 0x30), then the check over the 32 bits */
    i = idx_;
    buf[i] = (uint8_t)bit;
    n = (uint8_t)(i + 1);
    if (n != 0x30)
    {
      idx_ = n;
      ones_ = nextOnes;
      return 0;
    }
    else
    {
      uint32_t r[16];
      crcBytes(r, buf, 32);
      for (int k = 0; k <= 15; k++)
        crc_[k] = r[k];
      if (crcMismatch(crc_, buf + 0x20) != 0)
      {
        unPackReset();
      }
      else
      {
        idx_ = 0x30;
        state_ = 8;
        ones_ = nextOnes;
      }
      return 0;
    }
  case 8: /* four trailing bits; the fourth completes the frame */
    n = (uint8_t)(idx_ + 1);
    ones_ = nextOnes;
    idx_ = n;
    return n == 0x34 ? 1 : 0;
  default:
    ones_ = nextOnes;
    return 0;
  }
}

/* ---- V92Jd ---- */

/**
 * @brief Build a V.92 Jd holder from the phase-2 parameter block.
 * @param p  parameters: v92JdPhase (float, scaled by 65536 into 16 bits), v92MaxSpectralShaperLookahead, v92DigitalRateMask
 * @note  Fills the Jd data frame fields and the Jd phase frame fields; the marker bit of the phase frame (phase_[0x2f]) is set to 1.
 */
V92Jd::V92Jd(V90Parameters* p)
{
  ones_ = 0;
  idx_ = 0;
  bits_[0x2f] = 0;  /* +0x31 */
  phase_[0x2f] = 1; /* +0x79 */
  stateJd_ = 0;
  statePhase_ = 0;

  {
    uint32_t w = fistpLow32((long double)65536.0f * (long double)p->v92JdPhase);
    for (int k = 0; k <= 15; k++)
      phase_[0x12 + k] = (((w >> k) & 1) != 0) ? 1 : 0; /* +0x5c .. +0x6b */
  }

  setLookBits(&bits_[0x31], (uint8_t)p->v92MaxSpectralShaperLookahead); /* +0x33 / +0x34 */
  setMaskBits(&bits_[0x12], &bits_[0x23], 11, p->v92DigitalRateMask);   /* +0x14 .. +0x23, +0x25 .. +0x2f */
  phase_[0x30] = 0;                                                     /* +0x7a */
  phase_[0x31] = 0;                                                     /* +0x7b */
}

V92Jd::~V92Jd() {}

/* unused: set the 16 phase bits of the Jd phase frame (f * 65536, truncated) */
void V92Jd::setJdPhase(float f)
{
  uint32_t w = fistpLow32((long double)65536.0f * (long double)f);
  for (int k = 0; k <= 15; k++)
    phase_[0x12 + k] = (((w >> k) & 1) != 0) ? 1 : 0;
}

/* unused */
void V92Jd::setMaxLookahead(unsigned char v) { setLookBits(&bits_[0x31], v); }

/* unused */
void V92Jd::setRatesMask(int mask) { setMaskBits(&bits_[0x12], &bits_[0x23], 11, mask); }

/* unused: set the two constellation bytes of the Jd phase frame */
void V92Jd::setConstelSize(unsigned char a, unsigned char b)
{
  phase_[0x30] = a; /* +0x7a */
  phase_[0x31] = b; /* +0x7b */
}

/** @brief The phase value of the last received Jd phase frame: the 16 received bits divided by 65536. */
float V92Jd::getJdPhase()
{
  uint32_t w = 0;
  for (int k = 0; k <= 15; k++)
    if (phase_[k] != 0)
      w |= 1u << k;
  return (float)w * 1.52587890625e-05f; /* the exact value w / 65536 */
}

/** @brief Rate mask of the last received Jd data frame (28 bits). */
int V92Jd::getRatesMask() { return getMaskBits(&bits_[0], &bits_[0x10]); }

/** @brief The two constellation-size bytes of the last received Jd phase frame. */
void V92Jd::getConstelationSize(unsigned char* a, unsigned char* b)
{
  *a = phase_[0x1d]; /* +0x67 */
  *b = phase_[0x1e]; /* +0x68 */
}

/** @brief The look-ahead value (two bits) of the last received Jd data frame. */
unsigned char V92Jd::getMaxLookahead() { return lookBits(&bits_[0x1e]); }

/* unused: reset the CRC register to all ones */
void V92Jd::resetCrc()
{
  for (int k = 0; k <= 15; k++)
    crc_[k] = 1;
}

/**
 * @brief Build the Jd phase transmit frame in `phase_` (+0x4a).
 * @note  Layout: ones, start bit, the 16 phase bits (+0x5c), start bit, row 2 (+0x6d: 12 zeros, the marker, two constellation bytes, a zero), start bit,
 *        CRC bits at +0x7e, four zeros.
 */
void V92Jd::packJdPhaseData()
{
  for (int k = 0; k <= 16; k++)
    phase_[k] = 1;  /* +0x4a .. +0x5a */
  phase_[0x11] = 0; /* +0x5b */
  phase_[0x22] = 0; /* +0x6c */
  phase_[0x33] = 0; /* +0x7d */
  phase_[0x44] = 0; /* +0x8e .. +0x91 */
  phase_[0x45] = 0;
  phase_[0x46] = 0;
  phase_[0x47] = 0;
  for (int k = 0; k <= 11; k++)
    phase_[0x23 + k] = 0; /* +0x6d .. +0x78 */
  phase_[0x32] = 0;       /* +0x7c */

  /* CRC over the two rows */
  for (int k = 0; k <= 15; k++)
    crc_[k] = 1;
  uint32_t r[16];
  crcBytes(r, &phase_[0x12], 16);
  for (int k = 0; k < 16; k++)
    crcStep(r, phase_[0x23 + k]);
  for (int k = 0; k < 16; k++)
    crc_[k] = r[k];
  for (int k = 0; k <= 15; k++)
    phase_[0x34 + k] = (uint8_t)crc_[k]; /* +0x7e .. +0x8d */
}

/**
 * @brief Build the Jd data transmit frame in `bits_` (+2).
 * @note  As V90Jd::getBitVector(), but the bytes +0x2b .. +0x30 and +0x32 are zeroed first, so rate-mask bits 22 and above are not sent.
 */
void V92Jd::packJdData()
{
  for (int k = 0; k <= 16; k++)
    bits_[k] = 1;
  bits_[0x11] = 0;
  bits_[0x22] = 0;
  bits_[0x33] = 0;
  bits_[0x44] = 0;
  bits_[0x45] = 0;
  bits_[0x46] = 0;
  bits_[0x47] = 0;
  for (int k = 0; k <= 5; k++)
    bits_[0x29 + k] = 0; /* +0x2b .. +0x30 */
  bits_[0x30] = 0;       /* +0x32 */

  /* CRC over the two rows */
  for (int k = 0; k <= 15; k++)
    crc_[k] = 1;
  uint32_t r[16];
  crcBytes(r, &bits_[0x12], 16);
  for (int k = 0; k < 16; k++)
    crcStep(r, bits_[0x23 + k]);
  for (int k = 0; k < 16; k++)
    crc_[k] = r[k];
  for (int k = 0; k <= 15; k++)
    bits_[0x34 + k] = (uint8_t)crc_[k];
}

/** @brief Restart reception of the Jd data frame (clears the index, the Jd data state and the run length). */
void V92Jd::unPackJdReset()
{
  idx_ = 0;
  stateJd_ = 0;
  ones_ = 0;
}

/** @brief Restart reception of the Jd phase frame. */
void V92Jd::unPackJdPhaseReset()
{
  idx_ = 0;
  statePhase_ = 0;
  ones_ = 0;
}

/**
 * @brief Feed one received bit to the Jd phase frame receiver.
 * @param bit  received bit
 * @return 1 when a complete frame with a valid CRC and marker bit 1 has been received, otherwise 0
 * @note  Same state machine as V90Jd::unPackData(); the frame is collected in `phase_`.  The marker (phase_[0x1c]) is tested on the fourth trailing bit.
 */
int V92Jd::unPackJdPhaseData(int bit)
{
  uint8_t nextOnes = 0;
  if (bit != 0)
    nextOnes = (uint8_t)(ones_ + 1);
  uint8_t* buf = rawPhase(this);
  uint8_t i, n;

  switch (statePhase_)
  {
  case 0: /* count the ones */
    ones_ = nextOnes;
    if (nextOnes > 0x10)
      statePhase_ = 1;
    return 0;
  case 1:
    if (bit == 0)
    {
      ones_ = nextOnes;
      statePhase_ = 2;
    }
    else
    {
      unPackJdPhaseReset();
    }
    return 0;
  case 2:
    i = idx_;
    ones_ = nextOnes;
    buf[i] = (uint8_t)bit;
    n = (uint8_t)(i + 1);
    idx_ = n;
    if (n == 0x10)
      statePhase_ = 3;
    return 0;
  case 3:
    if (bit != 0)
    {
      unPackJdPhaseReset();
    }
    else
    {
      ones_ = nextOnes;
      statePhase_ = 4;
    }
    return 0;
  case 4:
    i = idx_;
    ones_ = nextOnes;
    buf[i] = (uint8_t)bit;
    n = (uint8_t)(i + 1);
    idx_ = n;
    if (n == 0x1c)
      statePhase_ = 5;
    return 0;
  case 5:
    i = idx_;
    ones_ = nextOnes;
    buf[i] = (uint8_t)bit;
    n = (uint8_t)(i + 1);
    idx_ = n;
    if (n == 0x20)
      statePhase_ = 6;
    return 0;
  case 6:
    if (bit != 0)
    {
      unPackJdPhaseReset();
    }
    else
    {
      ones_ = nextOnes;
      statePhase_ = 7;
    }
    return 0;
  case 7:
    i = idx_;
    buf[i] = (uint8_t)bit;
    n = (uint8_t)(i + 1);
    if (n != 0x30)
    {
      idx_ = n;
      ones_ = nextOnes;
      return 0;
    }
    else
    {
      uint32_t r[16];
      crcBytes(r, buf, 32);
      for (int k = 0; k <= 15; k++)
        crc_[k] = r[k];
      if (crcMismatch(crc_, buf + 0x20) != 0)
      {
        unPackJdPhaseReset();
      }
      else
      {
        idx_ = 0x30;
        statePhase_ = 8;
        ones_ = nextOnes;
      }
      return 0;
    }
  case 8: /* the fourth trailing bit completes the frame if the marker bit (index 0x1c) is 1 */
    n = (uint8_t)(idx_ + 1);
    if (n == 0x34)
    {
      if (phase_[0x1c] != 1)
      { /* +0x66 */
        unPackJdPhaseReset();
        return 0;
      }
      idx_ = 0x34;
      ones_ = nextOnes;
      return 1;
    }
    idx_ = n;
    ones_ = nextOnes;
    return 0;
  default:
    ones_ = nextOnes;
    return 0;
  }
}

/**
 * @brief Feed one received bit to the Jd data frame receiver.
 * @param bit  received bit
 * @return 1 when a complete frame with a valid CRC and marker bit 0 has been received, otherwise 0
 * @note  Ten states: as V90Jd::unPackData() with state 5 a single bit stored unconditionally.  The frame is collected in `bits_`.
 */
int V92Jd::unPackJdData(int bit)
{
  uint8_t nextOnes = 0;
  if (bit != 0)
    nextOnes = (uint8_t)(ones_ + 1);
  uint8_t* buf = rawBuf(this);
  uint8_t i, n;

  switch (stateJd_)
  {
  case 0: /* count the ones */
    ones_ = nextOnes;
    if (nextOnes > 0x10)
      stateJd_ = 1;
    return 0;
  case 1:
    if (bit == 0)
    {
      ones_ = nextOnes;
      stateJd_ = 2;
    }
    else
    {
      unPackJdReset();
    }
    return 0;
  case 2:
    i = idx_;
    ones_ = nextOnes;
    buf[i] = (uint8_t)bit;
    n = (uint8_t)(i + 1);
    idx_ = n;
    if (n == 0x10)
      stateJd_ = 3;
    return 0;
  case 3:
    if (bit != 0)
    {
      unPackJdReset();
    }
    else
    {
      ones_ = nextOnes;
      stateJd_ = 4;
    }
    return 0;
  case 4: /* 11 bits (up to index 0x1b) */
    i = idx_;
    ones_ = nextOnes;
    buf[i] = (uint8_t)bit;
    n = (uint8_t)(i + 1);
    idx_ = n;
    if (n == 0x1b)
      stateJd_ = 5;
    return 0;
  case 5: /* one bit, unconditionally on to state 6 */
    i = idx_;
    idx_ = (uint8_t)(i + 1);
    stateJd_ = 6;
    ones_ = nextOnes;
    buf[i] = (uint8_t)bit;
    return 0;
  case 6: /* four bits (up to index 0x20) */
    i = idx_;
    ones_ = nextOnes;
    buf[i] = (uint8_t)bit;
    n = (uint8_t)(i + 1);
    idx_ = n;
    if (n == 0x20)
      stateJd_ = 7;
    return 0;
  case 7:
    if (bit != 0)
    {
      unPackJdReset();
    }
    else
    {
      stateJd_ = 8;
      ones_ = nextOnes;
    }
    return 0;
  case 8: /* the 16 CRC bits, then the check */
    i = idx_;
    buf[i] = (uint8_t)bit;
    n = (uint8_t)(i + 1);
    if (n != 0x30)
    {
      idx_ = n;
      ones_ = nextOnes;
      return 0;
    }
    else
    {
      uint32_t r[16];
      crcBytes(r, buf, 32);
      for (int k = 0; k <= 15; k++)
        crc_[k] = r[k];
      if (crcMismatch(crc_, buf + 0x20) != 0)
      {
        unPackJdReset();
      }
      else
      {
        idx_ = 0x30;
        stateJd_ = 9;
        ones_ = nextOnes;
      }
      return 0;
    }
  case 9: /* the fourth trailing bit completes the frame if the marker bit (index 0x1c) is 0 */
    n = (uint8_t)(idx_ + 1);
    if (n == 0x34)
    {
      if (bits_[0x1c] != 0)
      { /* +0x1e */
        unPackJdReset();
        return 0;
      }
      idx_ = 0x34;
      ones_ = nextOnes;
      return 1;
    }
    idx_ = n;
    ones_ = nextOnes;
    return 0;
  default:
    ones_ = nextOnes;
    return 0;
  }
}

/** @brief Build the Jd data transmit frame; returns `bits_`. */
unsigned char* V92Jd::getJdBitVector()
{
  packJdData();
  return &bits_[0];
}

/** @brief Build the Jd phase transmit frame; returns `phase_`. */
unsigned char* V92Jd::getJdPhaseBitVector()
{
  packJdPhaseData();
  return &phase_[0];
}

/* ---- the V92MappingParams array helpers ---- */

/**
 * @brief Allocate the six constellation arrays (0x200 bytes each) of a mapping-parameter block.
 * @param p  block whose `lcp_` pointers are set
 * @warning The sysdep_malloc() results are not checked for NULL, and existing pointers are overwritten without being freed.
 */
extern "C" void V92createConstellations(V92MappingParams* p)
{
  for (int k = 0; k < 6; k++)
    p->lcp_[k] = (uint32_t*)sysdep_malloc(0x200);
}

/**
 * @brief Allocate the four filter-coefficient arrays (0x600 bytes each) of a mapping-parameter block.
 * @param p  block whose `arr_` pointers are set
 * @warning The sysdep_malloc() results are not checked for NULL.
 */
extern "C" void V92createFilterCoefficients(V92MappingParams* p)
{
  for (int k = 0; k < 4; k++)
    p->arr_[k] = (float*)sysdep_malloc(0x600);
}

/**
 * @brief Free the non-NULL constellation arrays of a block, in order.
 * @param p  block
 * @warning The pointers are not cleared: a second call frees them again.
 */
extern "C" void V92deleteConstellations(V92MappingParams* p)
{
  for (int k = 0; k < 6; k++)
    if (p->lcp_[k] != 0)
      sysdep_free(p->lcp_[k]);
}

/**
 * @brief Free the non-NULL filter-coefficient arrays of a block, in order.
 * @param p  block
 * @warning The pointers are not cleared: a second call frees them again.
 */
extern "C" void V92deleteFilterCoefficients(V92MappingParams* p)
{
  for (int k = 0; k < 4; k++)
    if (p->arr_[k] != 0)
      sysdep_free(p->arr_[k]);
}
