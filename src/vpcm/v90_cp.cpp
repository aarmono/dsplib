/**
 * @file v90_cp.cpp
 * @brief V90CP: the V.90 phase-4 CP / SUV frame holder and bit-serial codec (definitions).
 *
 * Entry points (declared in v90_cp.h): the constructor / destructor (six 0x200-byte heap blocks), reset(), resetDetector(), the CRC helpers (resetCRC(),
 * calcCRC(), evaluateCRC()), getBitVector(), calcSequenceLength(), infoToBits() (info fields -> bit buffer), bitsToInfo() (one received bit -> the receive
 * machine; returns the frame code), evaluateInfo() (decode the part the machine just completed) and printNofRecievedMpMpNot().
 *
 * Bit grid: every index that is a multiple of 17 holds a 0 start bit and is skipped by the CRC; the payload is written in groups of 16 bits and all
 * multi-bit fields are LSB first.  The decoder reads them back with a cursor (`cursor_`) that is left in a state the NEXT evaluateInfo() call depends on
 * (the receive machine calls evaluateInfo() once per completed part), so every cursor store is significant.
 *
 * Receive machine (bitsToInfo): state 0 counts leading ones (17 of them = flag), 1 expects the 0 start bit, 2 stores the type bit, 3 collects the rest of
 * a short frame, 4 the three part flags, 5 the header, 6 part A, 7 the part B counts, 8 the part B arrays, 10 the part C header, 11 the part C arrays,
 * 12 the CRC, 13 waits for the frame to end at the next write index that is a multiple of 6.  The jump table of evaluateInfo() covers states 3 .. 11, that of
 * bitsToInfo() states 0 .. 13.
 *
 * Behaviours to know about:
 * - Info bytes are stored whole (not masked) at their bit positions (type_, flagA_ .. flagC_, extendEu_, cptFlag_, wCa0_).
 * - `trn2dD_` == 0 divides by zero in calcSequenceLength() / infoToBits().
 * - Two function-local statics of bitsToInfo() (`alpha`, `beta`: the bit counts of the part B / part C arrays) are shared by ALL objects.
 * - Integer / byte arithmetic only.  The only external calls are sysdep_malloc / sysdep_free and dsplibs_debug_printf; dsplibs_debug_level is compared
 *   unsigned.
 */

#include "v90_cp.h"
#include "core/dsp_host.h"

/* ---- helpers ---- */

#include "pcm_cp_crc.h" /* crcRun / crcDiffSum / crcOnes */

/** Read `n` bits downwards from bits[pos] (pos = the MSB; bit 0 of every byte) and shift them in on top of `acc`. */
static inline uint32_t readDown(const uint8_t* bits, uint32_t pos, int n, uint32_t acc)
{
  int i;
  for (i = 0; i < n; i++)
    acc = (acc << 1) | (uint32_t)(bits[pos - (uint32_t)i] & 1);
  return acc;
}

/** Write the low `n` bits of `v` (arithmetic shift of a signed value), LSB first, into bits[at ..]. */
static inline void putBits(uint8_t* bits, uint32_t at, int32_t v, int n)
{
  int k;
  for (k = 0; k < n; k++)
  {
    bits[at + (uint32_t)k] = (uint8_t)(v & 1);
    v >>= 1;
  }
}

/* ---- construction / destruction / reset ---- */

/** @warning The six sysdep_malloc() results are not checked for NULL. */
V90CP::V90CP()
{
  int k;
  for (k = 0; k < 6; k++)
    heap_[k] = (uint32_t*)sysdep_malloc(0x200);
  widx_ = 0x12;
  count_ = 0;
  state_ = 0;
  ones_ = 0;
  zeros_ = 0;
  nMp_ = 0;
  nMpNot_ = 0;
  cptFlag_ = 0;
  supp_ = -1;
}

/** Frees the six heap arrays (those that are non-NULL). */
V90CP::~V90CP()
{
  int k;
  for (k = 0; k < 6; k++)
    if (heap_[k])
      sysdep_free(heap_[k]);
}

/** Presets the CRC register to ones. */
void V90CP::resetCRC() { crcOnes(crc_); }

/** Returns the receive machine to idle (write index 0x12, counters, state 0); `nMp_`, `nMpNot_` and `supp_` are kept. */
void V90CP::resetDetector()
{
  widx_ = 0x12;
  count_ = 0;
  state_ = 0;
  ones_ = 0;
  zeros_ = 0;
}

/** As resetDetector(), and also clears the MP counters and the suppression window. */
void V90CP::reset()
{
  widx_ = 0x12;
  count_ = 0;
  state_ = 0;
  nMp_ = 0;
  ones_ = 0;
  zeros_ = 0;
  nMpNot_ = 0;
  supp_ = -1;
}

/* ---- CRC / lengths ---- */

/** Runs the CRC over the frame bits (indices up to `frameLen_ - 0x11`); the register is not preset first. */
void V90CP::calcCRC() { crcRun(crc_, bits_, frameLen_ - 0x11); }

/** Presets the register, runs the CRC over the payload and returns 1 when it equals the 16 CRC bits that follow, else 0. */
int V90CP::evaluateCRC()
{
  uint32_t fl = frameLen_;
  crcOnes(crc_);
  crcRun(crc_, bits_, fl - 0x11);
  return crcDiffSum(crc_, bits_ + (fl - 16)) == 0;
}

/** Returns the bit buffer and sets `length` to the padded frame length `totalLen_`. */
unsigned char* V90CP::getBitVector(unsigned int& length)
{
  length = totalLen_;
  return bits_;
}

/** `totalLen_` = `frameLen_ + 1` rounded up to a multiple of `trn2dD_`.  @warning Divides by zero when `trn2dD_` is 0. */
void V90CP::calcSequenceLength()
{
  uint32_t v = frameLen_ + 1;
  uint32_t d = trn2dD_;
  uint32_t q = v / d;
  uint32_t t = d * q;
  if (t != v)
    t = (q + 1) * d;
  totalLen_ = t;
}

/** Prints the (never incremented) MP / MPNot counters at debug level 2 and above. */
void V90CP::printNofRecievedMpMpNot()
{
  if (dsplibs_debug_level > 1)
    dsplibs_debug_printf("V90MP: received %d MP, %d MPNot\r\n", nMp_, nMpNot_);
}

/* ---- buffer -> info ---- */

/**
 * Decodes the part of the frame that the receive machine has just completed (selected by `state_`) from `bits_` into the info fields.
 * - state 3: short frame, the byte and the CPt flag.
 * - state 5: the long-frame header (drn, trellis state, extend-Eu, CPt flag, prefilter gain); cursor left at 0x32.
 * - state 6: part A, six pairs of bytes; cursor left at 0x98.
 * - state 7: part B, the four 9-bit counts.
 * - state 8: part B, the 16-bit words of the four arrays.
 * - state 10: part C, the six nibbles and six 8-bit counts.
 * - state 11: part C, the 16-bit words of the six heap arrays.
 * - other states: nothing.
 * @bug The counts just decoded from the received frame are not range-checked: a part B count above 384 writes past its 384-word `arrs_` array (into the
 *      next array, or for array 3 into `hcnt_` and the members after it), and a part C count above 128 writes past the 0x200-byte heap block.
 */
void V90CP::evaluateInfo()
{
  uint32_t c, g, k, n;
  switch (state_)
  {
  case 3:
    wCa0_ = bits_[0x20];
    cptFlag_ = bits_[0x21];
    break;
  case 5:
    drn_ = 0;
    drn_ = (uint8_t)readDown(bits_, 0x1a, 5, 0);
    prefilterGain_ = 0;
    extendEu_ = bits_[0x1d];
    trellisState_ = (uint8_t)((bits_[0x1b] & 1) | ((bits_[0x1c] & 1) << 1));
    cptFlag_ = bits_[0x21];
    prefilterGain_ = readDown(bits_, 0x32, 16, 0);
    cursor_ = 0x32;
    break;
  case 6:
    for (k = 0; k < 12; k++)
      pair_[k] = 0;
    c = cursor_;
    for (g = 0; g < 6; g++)
    {
      pair_[2 * g] = readDown(bits_, c + 9, 8, pair_[2 * g]);
      pair_[2 * g + 1] = readDown(bits_, c + 0x11, 8, pair_[2 * g + 1]);
      c += 0x11;
    }
    cursor_ = 0x98;
    break;
  case 7:
    cnt_[0] = 0;
    cnt_[1] = 0;
    cnt_[2] = 0;
    cnt_[3] = 0;
    c = cursor_;
    for (g = 0; g < 4; g++)
      cnt_[g] = readDown(bits_, c + 10 + 17 * g, 9, cnt_[g]);
    cursor_ = c + 0x44;
    break;
  case 8:
    for (g = 0; g < 4; g++)
    {
      n = cnt_[g];
      for (k = 0; k < n; k++)
      {
        c = cursor_;
        cursor_ = c + 0x11;
        arrs_[384 * g + k] = 0;
        arrs_[384 * g + k] = (uint16_t)readDown(bits_, c + 0x11, 16, 0);
        cursor_ = c + 0x11;
      }
    }
    break;
  case 10:
    for (k = 0; k < 6; k++)
      nib_[k] = 0;
    cursor_ = cursor_ + 1;
    for (k = 0; k < 4; k++)
    {
      c = cursor_;
      nib_[k] = readDown(bits_, c + 4, 4, nib_[k]);
      cursor_ = c + 4;
      if (k == 3)
        cursor_ = c + 5;
    }
    for (k = 4; k < 6; k++)
    {
      c = cursor_;
      nib_[k] = readDown(bits_, c + 4, 4, nib_[k]);
      cursor_ = c + 4;
      if (k == 5)
        cursor_ = c + 0xc;
    }
    for (k = 0; k < 6; k++)
      hcnt_[k] = 0;
    for (g = 0; g < 3; g++)
    {
      c = cursor_;
      hcnt_[2 * g] = readDown(bits_, c + 9, 8, hcnt_[2 * g]);
      hcnt_[2 * g + 1] = readDown(bits_, c + 0x11, 8, hcnt_[2 * g + 1]);
      cursor_ = c + 0x11;
    }
    break;
  case 11:
    for (g = 0; g < 6; g++)
    {
      if (hcnt_[g] != 0)
      {
        uint32_t* h = heap_[g];
        for (k = 0; k < hcnt_[g]; k++)
        {
          c = cursor_ + 0x11;
          h[k] = 0;
          h[k] = readDown(bits_, c, 16, 0);
          cursor_ = c - 16 + 0x10;
        }
      }
    }
    break;
  default:
    break;
  }
}

/* ---- info -> buffer ---- */

/**
 * Encodes the info fields into `bits_`: 17 ones and a start bit, the type bit, the header (long frame: part flags, drn, trellis state, extend-Eu, CPt flag,
 * prefilter gain; short frame: zero padding, the byte, the CPt flag), parts A / B / C when flagged, the CRC and a final start bit, then zero padding up to
 * a multiple of `trn2dD_`.  Sets `widx_`, `frameLen_` and `totalLen_`.
 * @warning `trn2dD_` must be non-zero (division).  The part counts (`cnt_`, `hcnt_`) are not range-checked against the array sizes, and the written length
 *          is not checked against the 12000-bit buffer.
 */
void V90CP::infoToBits()
{
  uint32_t c, k, g, n, tl, D, q, t, C;

  /* 17 ones, a start bit, the type bit */
  for (k = 0; k < 0x11; k++)
    bits_[k] = 1;
  bits_[0x11] = 0;
  bits_[0x12] = (uint8_t)type_;

  if (type_ != 0)
  {
    for (k = 0x13; k < 0x20; k++)
      bits_[k] = 0;
    widx_ = 0x22;
    bits_[0x20] = (uint8_t)wCa0_;
    bits_[0x21] = cptFlag_;
    c = 0x22;
  }
  else
  {
    uint32_t fa = flagA_, fb = flagB_, fc = flagC_;
    int8_t f11 = (int8_t)trellisState_;

    /* header: part flags, drn, trellis state (values outside 0 .. 3 leave both bits unchanged), extend-Eu, CPt flag, prefilter gain */
    bits_[0x13] = (uint8_t)fa;
    bits_[0x14] = (uint8_t)fb;
    bits_[0x15] = (uint8_t)fc;
    putBits(bits_, 0x16, (int8_t)drn_, 5);

    if (f11 == 1)
    {
      bits_[0x1b] = 1;
      bits_[0x1c] = 0;
    }
    else if (f11 < 2)
    {
      if (f11 == 0)
      {
        bits_[0x1b] = 0;
        bits_[0x1c] = 0;
      }
    }
    else if (f11 == 2)
    {
      bits_[0x1b] = 0;
      bits_[0x1c] = 1;
    }
    else if (f11 == 3)
    {
      bits_[0x1b] = 1;
      bits_[0x1c] = 1;
    }

    bits_[0x1d] = extendEu_;
    bits_[0x1e] = 0;
    bits_[0x1f] = 0;
    bits_[0x20] = 0;
    bits_[0x21] = cptFlag_;
    bits_[0x22] = 0;
    for (k = 0; k < 16; k++)
      bits_[0x23 + k] = (uint8_t)(((uint32_t)(uint16_t)prefilterGain_ >> k) & 1);
    b0ca8_ = 0;
    c = 0x33;

    /* part A: 6 pairs of bytes, each led by a start bit */
    if (fa != 0)
    {
      for (g = 0; g < 6; g++)
      {
        bits_[c] = 0;
        putBits(bits_, c + 1, (int16_t)pair_[2 * g], 8);
        putBits(bits_, c + 9, (int16_t)pair_[2 * g + 1], 8);
        c += 0x11;
      }
    }

    /* part B: the four counts, then the four arrays of 16-bit words */
    if (fb != 0)
    {
      uint32_t cn[4];
      uint32_t cg0 = cnt_[0], cg1 = cnt_[1], cg2 = cnt_[2], cg3 = cnt_[3];
      cn[0] = cg0;
      cn[1] = cg1;
      cn[2] = cg2;
      cn[3] = cg3;
      for (g = 0; g < 4; g++)
      {
        uint32_t v = cn[g];
        bits_[c] = 0;
        for (k = 0; k < 9; k++)
        {
          bits_[c + 1 + k] = (uint8_t)(v & 1);
          v >>= 1;
        }
        for (k = 0; k < 7; k++)
          bits_[c + 10 + k] = 0;
        c += 0x11;
      }

      for (g = 0; g < 4; g++)
      {
        n = cn[g];
        for (k = 0; k < n; k++)
        {
          bits_[c] = 0;
          putBits(bits_, c + 1, (int16_t)arrs_[384 * g + k], 16);
          c += 0x11;
        }
      }
    }

    /* part C: the six nibbles, the six counts, then the six heap arrays */
    if (fc != 0)
    {
      bits_[c] = 0;
      c++;
      for (k = 0; k < 4; k++)
      {
        putBits(bits_, c, (int16_t)nib_[k], 4);
        c += 4;
      }
      bits_[c] = 0;
      c++;
      for (k = 4; k < 6; k++)
      {
        putBits(bits_, c, (int16_t)nib_[k], 4);
        c += 4;
      }
      for (k = 0; k < 8; k++)
        bits_[c + k] = 0;
      c += 8;

      for (g = 0; g < 3; g++)
      {
        bits_[c] = 0;
        putBits(bits_, c + 1, (int16_t)hcnt_[2 * g], 8);
        putBits(bits_, c + 9, (int16_t)hcnt_[2 * g + 1], 8);
        c += 0x11;
      }

      for (g = 0; g < 6; g++)
      {
        n = hcnt_[g];
        if (n != 0)
        {
          uint32_t* h = heap_[g];
          for (k = 0; k < n; k++)
          {
            bits_[c] = 0;
            putBits(bits_, c + 1, (int16_t)h[k], 16);
            c += 0x11;
          }
        }
      }
    }
  }

  /* tail: the last start bit, the CRC, the final start bit, the zero padding up to a multiple of D */
  C = c;
  bits_[C] = 0;
  widx_ = C + 1;
  frameLen_ = C + 0x11;
  crcOnes(crc_);
  crcRun(crc_, bits_, C);
  for (k = 0; k < 16; k++)
    bits_[C + 1 + k] = crc_[k];
  bits_[C + 17] = 0;
  widx_ = C + 18;

  D = trn2dD_;
  q = (C + 0x12) / D;
  t = D * q;
  if (t != C + 0x12)
    t = (q + 1) * D;
  totalLen_ = t;
  tl = t;
  for (k = widx_; k < tl; k++)
    bits_[k] = 0;
}

/* ---- receive state machine ---- */

/**
 * Bounds-checked store used by states 3 / 8 / 10 / 11 / 12: a bit beyond the 12000-bit buffer is dropped (with a message at debug level 2); the caller's
 * counters still advance.
 */
static inline void storeBitChecked(V90CP* o, uint8_t bit)
{
  uint32_t w = o->widx_;
  if (w < 12000u)
  {
    o->bits_[w] = bit;
    o->widx_ = w + 1;
  }
  else if (dsplibs_debug_level > 1)
  {
    dsplibs_debug_printf("\n *** error CP bit , not enouch memory in the buffer *** \n");
  }
}

/** The state after part B (or after the header / part A when part B is absent): 12 (CRC) when part C is absent, else 10 (part C header). */
static inline uint32_t stateAfterB(const V90CP* o) { return o->flagC_ == 0 ? 12u : 10u; }

/**
 * Feeds one received bit to the receive machine (see the file comment for the states).  A run of `2 * trn2dD_` zero bits while idle reports CPINFO_ED.  When
 * a frame completes (state 13) the result is CPINFO_CP / CPINFO_CPT (long) or CPINFO_SUV / CPINFO_SUVT (short) by `cptFlag_`.  After a result of 1 or 2 a
 * suppression window of 800 calls starts (`supp_`), during which results 2 and 4 are reported as 0.
 * @param bit  the received bit (0 or non-zero)
 * @return a CpInfoCode value, 0 if no frame completed
 * @note A frame ends in state 13 at the next write index that is a multiple of 6 (not at `totalLen_`); a 1 bit there abandons it.  A frame with a bad CRC is
 *       dropped with a message at debug level 2.
 */
int V90CP::bitsToInfo(unsigned char bit)
{
  uint32_t ret = 0;
  uint8_t newOnes;

  /* run counters: a 0 bit clears the ones run, a 1 bit clears the zeros run */
  if (bit == 0)
  {
    zeros_ = (uint8_t)(zeros_ + 1);
    newOnes = 0;
  }
  else
  {
    zeros_ = 0;
    newOnes = (uint8_t)(ones_ + 1);
  }

  /* Ed: 2 * D zero bits while no frame is being collected */
  if ((uint32_t)zeros_ == trn2dD_ * 2u && widx_ == 0x12)
    ret = CPINFO_ED;

  {
    static uint32_t alpha;
    static uint32_t beta;
    switch (state_)
    {
    /* waiting for the 17 leading ones */
    case 0:
      ones_ = newOnes;
      if (newOnes > 0x10)
        state_ = 1;
      break;
    /* the start bit that ends the ones; anything else restarts the search */
    case 1:
      if (bit == 0)
      {
        ones_ = newOnes;
        state_ = 2;
      }
      else
      {
        ones_ = 0;
        widx_ = 0x12;
        count_ = 0;
        state_ = 0;
        zeros_ = 0;
      }
      break;
    /* the type bit: 0 = long frame (flags follow), 1 = short frame */
    case 2:
    {
      uint32_t idx = widx_;
      ones_ = newOnes;
      type_ = bit;
      bits_[idx] = bit;
      widx_ = idx + 1;
      state_ = (bit == 0) ? 4u : 3u;
      count_ = 0;
      break;
    }
    /* short frame: 15 more bits, then decode it and wait for the CRC */
    case 3:
      ones_ = newOnes;
      storeBitChecked(this, bit);
      count_ = count_ + 1;
      if (count_ == 0xf)
      {
        evaluateInfo();
        state_ = 0xc;
        count_ = 0;
      }
      break;
    /* long frame: the three part flags */
    case 4:
    {
      uint32_t idx = count_;
      uint32_t w = widx_;
      ones_ = newOnes;
      bits_[w] = bit;
      widx_ = w + 1;
      if (idx == 1)
        flagB_ = bit;
      else if (idx == 0)
        flagA_ = bit;
      else if (idx == 2)
        flagC_ = bit;
      if (idx + 1 != 3)
      {
        count_ = idx + 1;
      }
      else
      {
        state_ = 5;
        count_ = 0;
      }
      break;
    }
    /* long frame: the header, up to write index 0x33 */
    case 5:
    {
      uint32_t w = widx_;
      ones_ = newOnes;
      bits_[w] = bit;
      w++;
      widx_ = w;
      if (w == 0x33)
      {
        evaluateInfo();
        count_ = 0;
        if (flagA_ == 0)
          state_ = (flagB_ == 0) ? stateAfterB(this) : 7u;
        else
          state_ = 6;
      }
      break;
    }
    /* part A, up to write index 0x99 */
    case 6:
    {
      uint32_t w = widx_;
      ones_ = newOnes;
      bits_[w] = bit;
      w++;
      widx_ = w;
      if (w == 0x99)
      {
        evaluateInfo();
        count_ = 0;
        state_ = (flagB_ == 0) ? stateAfterB(this) : 7u;
      }
      break;
    }
    /* part B counts (0x44 bits); then the array bit count (17 per word) is latched in `alpha` */
    case 7:
    {
      uint32_t w = widx_;
      uint32_t n = count_ + 1;
      ones_ = newOnes;
      bits_[w] = bit;
      widx_ = w + 1;
      count_ = n;
      if (n == 0x44)
      {
        evaluateInfo();
        state_ = 8;
        count_ = 0;
        alpha = (cnt_[1] + cnt_[0] + cnt_[2] + cnt_[3]) * 0x11u;
      }
      break;
    }
    /* part B arrays */
    case 8:
    {
      uint32_t n;
      ones_ = newOnes;
      storeBitChecked(this, bit);
      n = count_ + 1;
      count_ = n;
      if (n == alpha)
      {
        evaluateInfo();
        state_ = stateAfterB(this);
        count_ = 0;
      }
      break;
    }
    /* part C header (0x55 bits); then the array bit count is latched in `beta` */
    case 10:
    {
      uint32_t n;
      ones_ = newOnes;
      storeBitChecked(this, bit);
      n = count_ + 1;
      count_ = n;
      if (n == 0x55)
      {
        evaluateInfo();
        count_ = 0;
        state_ = 0xb;
        beta = (hcnt_[1] + hcnt_[0] + hcnt_[2] + hcnt_[3] + hcnt_[4] + hcnt_[5]) * 0x11u;
      }
      break;
    }
    /* part C arrays */
    case 11:
    {
      uint32_t n;
      ones_ = newOnes;
      storeBitChecked(this, bit);
      n = count_ + 1;
      count_ = n;
      if (n == beta)
      {
        evaluateInfo();
        state_ = 0xc;
        count_ = 0;
      }
      break;
    }
    /* the 17 CRC bits, then the CRC check: good -> state 13, bad -> idle */
    case 12:
    {
      uint32_t n;
      ones_ = newOnes;
      storeBitChecked(this, bit);
      n = count_ + 1;
      if (n != 0x11)
      {
        count_ = n;
      }
      else
      {
        uint32_t fl = widx_;
        frameLen_ = fl;
        crcOnes(crc_);
        crcRun(crc_, bits_, fl - 0x11);
        if (crcDiffSum(crc_, bits_ + (fl - 16)) == 0)
        {
          count_ = 0x11;
          state_ = 0xd;
        }
        else
        {
          unsigned level;
          ones_ = 0;
          zeros_ = 0;
          level = dsplibs_debug_level;
          widx_ = 0x12;
          count_ = 0;
          state_ = 0;
          if (level > 1)
            dsplibs_debug_printf("V90CP: recieved CP with bad CRC\r\n");
        }
      }
      break;
    }
    /* the frame is complete; it ends at the next write index that is a multiple of 6 */
    case 13:
      if (bit == 0)
      {
        widx_ = widx_ + 1;
        ones_ = newOnes;
      }
      else
      {
        ones_ = 0;
        widx_ = 0x12;
        count_ = 0;
        state_ = 0;
        zeros_ = 0;
      }
      if (widx_ % 6u == 0)
      {
        evaluateInfo();
        state_ = 0;
        widx_ = 0x12;
        count_ = 0;
        ones_ = 0;
        zeros_ = 0;
        if (cptFlag_ == 0)
          ret = (type_ == 0) ? CPINFO_CP : CPINFO_SUV;
        else
          ret = (type_ == 0) ? CPINFO_CPT : CPINFO_SUVT;
      }
      break;
    default:
      ones_ = newOnes;
      break;
    }
  }

  /* the suppression window after a frame: while it counts, the codes 2 and 4 are reported as 0 */
  if (supp_ < 0)
  {
    if (ret - 1 > 1)
      return (int)ret;
    supp_ = 0;
    return (int)ret;
  }
  else
  {
    int32_t s = supp_ + 1;
    if (ret == CPINFO_SUVT || ret == CPINFO_CPT)
      ret = 0;
    if (s == 800)
    {
      supp_ = -1;
      return (int)ret;
    }
    supp_ = s;
    return (int)ret;
  }
}
