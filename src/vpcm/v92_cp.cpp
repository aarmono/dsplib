/**
 * @file v92_cp.cpp
 * @brief V92CP, the V.92 CP / SUV frame holder and bit-serial codec, and the functions that convert mapping parameters to and from CP frames.
 *
 * Defines the members of V92CP (declared in v92_cp.h) and four free functions:
 * - setV92CPpckFromParamsInfo(): V90MappingParams + tagV90AdditionalCPinfo -> V92CP (dedupes the six code-point lists into groups and packs each list
 *   variant into mask words); called by VPcmFloModem when it sends a V.92 CP.
 * - V92setParamsInfoFromCPUnPck(): a received V90CP -> V92MappingParams (flags, gain, prefilter arrays, constellations), with debug dumps at debug
 *   level > 1; called by VPcmFloModem.
 * - setParamsInfoFromV92CPUnPck() and setParamsInfoFromCPUnPck(): the reverse conversions (V92CP or CPUnPck -> V90MappingParams); nothing in the
 *   library calls them.
 * Used by other files: the constructor, setSUV(), infoToBits() and getBitVector() (V92Phase4Modulator), and the data tables binaryTable[], fltTable_1[]
 * and fltTable_2[].  bitsToInfo(), evaluateInfo(), evaluateCRC(), calcCRC(), resetCRC(), resetDetector() and reset() are the receive side and are not
 * called by any other file.
 *
 * Frame layout (positions in `bits_`): 0 .. 0x10 flag (17 ones), 0x11 zero, 0x12 type bit.  SUV (type 1): 7 zeros, a zero, 5 bits of suv_, the SUV byte,
 * the flag_ bit, ending at 0x22.  Long frame: mode_ (2 bits), rate_ (5 bits), a zero field, sr_ (2 bits, only when mode_ <= 1), flag_ (0x21); when
 * mode_ <= 1 a float part follows (b03_, id_, trn1dRms_ against fltTable_2, the four shaper coefficients as sign + 7 bits against fltTable_1, the
 * six 4-bit group numbers, flagB_), then arrA_ (and arrB_ when flagB_) as nGrp_ * 8 words of 16 bits, each preceded by a 0 start bit.  Every frame ends
 * with a start bit, a 16-bit CRC, a final start bit and zero padding up to the next multiple ABOVE of 12 * mult_.
 *
 * Behaviour worth knowing before changing anything:
 * - Words are written LSB first by infoToBits() but read MSB first by evaluateInfo(): the pack and unpack functions mirror each other (a bit reversal).
 * - fltTable_2 holds 2^-9 twice and has no 2^-8 entry; the TRN1d RMS conversion therefore has no weight for that bit (see the @bug note at the table).
 * - The float conversions keep their intermediate values in extended precision (`long double`): a value is reduced by each table entry it is not
 *   smaller than, and the float words are moved as integer words (copyWord) so a signalling NaN keeps its payload.  The sign tests are written to match
 *   the x87 compare used on the 32-bit target: "t > v" is false for unordered, and the sign bits of shaperA2_ .. shaperB2_ are set for NaN too.
 * - evaluateInfo() case 6 leaves cursor_ at 0x88 and cases 7 / 8 continue from there; rate_ is sign extended on the wire (the 13 bits after the b03_
 *   byte) but decoded from 5 bits; frameLen_ is stored before the CRC check; a bad CRC prints "V92CP: recieved CP with bad CRC".
 * - bitsToInfo() keeps the expected part lengths of the two array regions in function-local statics shared by ALL V92CP objects.
 */

#include "v92_cp.h"
#include "core/dsp_asm.h"
#include "core/dsp_math.h"
#include "core/dsp_host.h"

typedef long double LD;

/* ---- bit-weight tables ---- */

extern "C"
{
  /** Powers of two 1 .. 0x8000, the weights of the bits of a field read from the frame. */
  uint32_t binaryTable[16] = {1, 2, 4, 8, 0x10, 0x20, 0x40, 0x80, 0x100, 0x200, 0x400, 0x800, 0x1000, 0x2000, 0x4000, 0x8000};

  /**
   * Bit weights of the 16-bit Q3.13 TRN1d RMS field, most significant first: 4, 2, 1, 1/2 ... 1/128, 2^-9, 2^-9, 2^-10 ... 2^-13.
   * @bug The table lists 2^-9 twice and has no 2^-8 entry, so the field cannot represent that bit; the same table is used by float2Bits() in
   *      pcm_cp_packers.cpp, and both are kept as they are.
   */
  float fltTable_2[16] = {4.0f,      2.0f,       1.0f,         0.5f,         0.25f,         0.125f,         0.0625f,         0.03125f,
                          0.015625f, 0.0078125f, 0.001953125f, 0.001953125f, 0.0009765625f, 0.00048828125f, 0.000244140625f, 0.0001220703125f};

  /** Bit weights of the 7-bit magnitude of a shaper coefficient, most significant first: 1, 1/2 ... 1/64. */
  float fltTable_1[7] = {1.0f, 0.5f, 0.25f, 0.125f, 0.0625f, 0.03125f, 0.015625f};
}

/* ---- helpers ---- */

#include "core/dsp_copyword.h"

/** @brief The raw 32-bit word of a float cell (no x87 load, so a signalling NaN keeps its payload). */
static inline uint32_t wordOfCell(const float* src)
{
  uint32_t w;
  __builtin_memcpy(&w, src, sizeof w);
  return w;
}
#include "core/dsp_floatbits.h"
#include "core/dsp_intmath.h"
#include "vpcm_split.h"
#include "vpcm_split_value.h"

#include "pcm_cp_crc.h" /* crcRun / crcDiffSum / crcOnes: the 16-bit CRC shared with V90MP and V90CP */

/** @brief Store the low 16 bits of @p w, LSB first (arithmetic shifts), into bits[at .. at + 15]. */
static inline void putWord16(uint8_t* bits, uint32_t at, int32_t w)
{
  int k;
  for (k = 0; k < 16; k++)
  {
    bits[at + (uint32_t)k] = (uint8_t)(w & 1);
    w >>= 1;
  }
}

/** @brief Store @p n bits of @p v, LSB first (arithmetic shifts), into bits[at .. at + n - 1]. */
static inline void putBitsLsb(uint8_t* bits, uint32_t at, int32_t v, int n)
{
  int k;
  for (k = 0; k < n; k++)
  {
    bits[at + (uint32_t)k] = (uint8_t)(v & 1);
    v >>= 1;
  }
}

/**
 * @brief Greedy conversion of a magnitude to bits against a table of descending weights.
 * @param bits  frame buffer
 * @param top   position of the first (most significant) bit; entry i goes to bits[top - i]
 * @param v     |value|, kept in extended precision (v = v - t at every set bit)
 * @param tab   bit weights
 * @param n     number of table entries
 * @note  A weight larger than @p v gives 0, also when @p v is NaN (the comparison is false for unordered), so a NaN converts to all ones.
 */
static inline void floatToBits(uint8_t* bits, uint32_t top, LD v, const float* tab, int n)
{
  int i;
  for (i = 0; i < n; i++)
  {
    LD t = (LD)tab[i];
    if (t > v)
    {
      bits[top - (uint32_t)i] = 0;
    }
    else
    {
      bits[top - (uint32_t)i] = 1;
      v = v - t;
    }
  }
}

/**
 * @brief Sum the weights of the set bits of a field read upwards from the buffer.
 * @param bits    frame buffer
 * @param cursor  read position; advanced by @p n
 * @param tab     bit weights; the i-th bit read has weight tab[n - 1 - i]
 * @param n       number of bits
 * @return the sum in extended precision
 */
static inline LD bitsToFloat(const uint8_t* bits, uint32_t* cursor, const float* tab, int n)
{
  LD acc = 0.0L;
  int i;
  for (i = 0; i < n; i++)
  {
    uint32_t c = *cursor;
    *cursor = c + 1;
    if (bits[c] != 0)
      acc = acc + (LD)tab[n - 1 - i];
  }
  return acc;
}

/**
 * @brief Find the distinct code-point lists among the six of a mapping-parameters block.
 *
 * Group g duplicates an earlier distinct group when it has the same count and both list variants match element by element.
 *
 * @param mp   mapping parameters; idx_[n] receives the first group of distinct list n
 * @param grp  receives, for each group g, the number of its distinct list
 * @return the number of distinct lists (1 .. 6)
 */
static inline unsigned dedupeGroups(V90MappingParams* mp, uint32_t* grp)
{
  unsigned n = 1;
  unsigned g;
  mp->idx_[0] = 0;
  grp[0] = 0;
  for (g = 1; g <= 5; g++)
  {
    unsigned b;
    unsigned cnt = mp->cnt_[g];
    for (b = 0; b < n; b++)
    {
      unsigned d = mp->idx_[b];
      unsigned i;
      if (cnt != mp->cnt_[d])
        continue;
      for (i = 0; i < cnt; i++)
      {
        if (mp->listA_[d][i] != mp->listA_[g][i])
          break;
        if (mp->listB_[d][i] != mp->listB_[g][i])
          break;
      }
      if (i == cnt)
      {
        grp[g] = grp[d];
        break;
      }
    }
    if (b == n)
    {
      mp->idx_[n] = g;
      grp[g] = n;
      n++;
    }
  }
  return n;
}

/**
 * @brief Set the mask bit of every list element: element pos sets bit (pos & 15) of word (pos >> 4).
 * @param dst   the 8 mask words of a group, cleared by the caller
 * @param list  code-point list
 * @param cnt   list length
 * @warning pos >> 4 is not range-checked: an element >= 128 writes beyond the 8 words of the group.
 */
static inline void listToMask(uint16_t* dst, const uint8_t* list, unsigned cnt)
{
  unsigned e;
  for (e = 0; e < cnt; e++)
  {
    unsigned pos = list[e];
    unsigned hi = pos >> 4;
    dst[hi] = (uint16_t)(dst[hi] | (1u << (pos & 15)));
  }
}

/**
 * @brief Expand the 8 mask words of a group into list elements (the inverse of listToMask()).
 *
 * Words are taken from 7 down to 0 and bits from the least significant up, appending element 16 * j + b for each set bit b of word j.
 *
 * @param list  receives the elements, starting at index @p cnt
 * @param src   the 8 mask words
 * @param cnt   starting count (the callers pass 0)
 * @return the new count
 */
static inline unsigned maskToList(uint8_t* list, const uint16_t* src, unsigned cnt)
{
  int j;
  for (j = 7; j >= 0; j--)
  {
    int32_t w = (int16_t)src[j];
    int b;
    for (b = 15; b >= 0; b--)
    {
      if (w & 1)
      {
        list[cnt] = (uint8_t)(16 * j + b);
        cnt++;
      }
      w >>= 1;
    }
  }
  return cnt;
}

/* ---- construction, destruction, reset ---- */

/** @brief Put the receive machine in its idle state and clear flag_; the other frame fields (type_, mode_, mult_ ...) are left for the owner to fill. */
V92CP::V92CP()
{
  widx_ = 0x12;
  count_ = 0;
  state_ = 0;
  ones_ = 0;
  zeros_ = 0;
  flag_ = 0;
  supp_ = -1;
}

V92CP::~V92CP() {}

/** @brief Preset the CRC register to all ones. */
void V92CP::resetCRC() { crcOnes(crc_); }

/** @brief Return the receive machine to idle; supp_ (the suppression window) is not touched. */
void V92CP::resetDetector()
{
  widx_ = 0x12;
  count_ = 0;
  state_ = 0;
  ones_ = 0;
  zeros_ = 0;
}

/** @brief resetDetector() and end the suppression window (supp_ = -1). */
void V92CP::reset()
{
  widx_ = 0x12;
  count_ = 0;
  state_ = 0;
  ones_ = 0;
  zeros_ = 0;
  supp_ = -1;
}

/**
 * @brief Make the frame an SUV value (the owner sets type_ == 1).
 * @param v  SUV byte, sent whole
 */
void V92CP::setSUV(unsigned int v)
{
  suv_ = 0x10;
  suvVal_ = v;
}

/**
 * @brief Give access to the bit buffer after infoToBits().
 * @param length  receives the padded frame length in bits
 * @return bits_, one bit per byte
 */
unsigned char* V92CP::getBitVector(unsigned int& length)
{
  length = totalLen_;
  return bits_;
}

/** @brief Run the CRC over bits_[0 .. frameLen_ - 0x11) into crc_. */
void V92CP::calcCRC() { crcRun(crc_, bits_, frameLen_ - 0x11); }

/** @brief Recompute the CRC of the received frame and compare it with the 16 bits that end the frame.  @return non-zero when they match */
int V92CP::evaluateCRC()
{
  uint32_t fl;
  crcOnes(crc_);
  fl = frameLen_;
  crcRun(crc_, bits_, fl - 0x11);
  return crcDiffSum(crc_, bits_ + (fl - 16)) == 0;
}

/* ---- frame fields -> bit buffer ---- */

/**
 * @brief Encode the frame fields into bits_ (SUV when type_ == 1, else a long frame) and append CRC and padding.
 *
 * Sets widx_ (end of the encoded frame), frameLen_ (up to and including the CRC), totalLen_ (the padded length that getBitVector() reports) and, when
 * flag_ != 0, cptSent_.  The frame layout is described in the file comment.
 *
 * @warning mult_ must have been set by the owner before the call (a long frame with mode_ == 0 sets it to 1); a value of 0 divides by zero, and
 *          a totalLen_ beyond 2000 writes past bits_.  A long frame with nGrp_ > 6 reads past arrA_ / arrB_.
 */
void V92CP::infoToBits()
{
  uint32_t c, k, g, e;
  uint8_t t;

  /* flag (17 ones), zero, type bit */
  for (k = 0; k <= 0x10; k++)
    bits_[k] = 1;
  bits_[0x11] = 0;
  t = type_;
  bits_[0x12] = t;

  if (t == 1)
  {
    /* SUV: 7 zeros, a zero, 5 bits of the SUV value, the SUV byte (whole), the flag */
    int32_t sv;
    for (k = 0x13; k <= 0x19; k++)
      bits_[k] = 0;
    sv = (int16_t)suv_;
    bits_[0x1a] = 0;
    putBitsLsb(bits_, 0x1b, sv, 5);
    widx_ = 0x22;
    bits_[0x20] = (uint8_t)suvVal_;
    bits_[0x21] = flag_;
  }
  else
  {
    /* long frame header: mode, rate, reserved zeros, sr_ (only without the signed-mode escape), flag */
    uint8_t m = mode_;
    int32_t r;

    bits_[0x13] = (uint8_t)(m & 1);
    bits_[0x14] = (uint8_t)(((int8_t)m >> 1) & 1);
    modeCopy_ = m;
    if (m == 0)
      mult_ = 1;

    r = (int8_t)rate_;
    putBitsLsb(bits_, 0x15, r, 5);
    r >>= 5;
    for (k = 0x1a; k <= 0x20; k++)
      bits_[k] = 0;
    if ((int8_t)m <= 1)
    {
      uint32_t w = sr_;
      bits_[0x1f] = (uint8_t)(w & 1);
      bits_[0x20] = (uint8_t)((w >> 1) & 1);
    }
    widx_ = 0x22;
    bits_[0x21] = flag_;

    if ((int8_t)m <= 1)
    {
      /* float part: b03_, the rest of the sign-extended rate, id_, TRN1d RMS, the four shaper coefficients (sign + 7 bits each), group numbers */
      uint8_t b03 = b03_;
      uint32_t w;
      LD v;
      bits_[0x22] = 0;
      bits_[0x23] = b03;
      putBitsLsb(bits_, 0x24, r, 13);

      w = id_;
      bits_[0x31] = (uint8_t)(w & 1);
      bits_[0x32] = (uint8_t)((w >> 1) & 1);
      bits_[0x33] = 0;

      v = (LD)(float)__builtin_fabsl((LD)trn1dRms_);
      floatToBits(bits_, 0x43, v, fltTable_2, 16);
      bits_[0x44] = 0;

      bits_[0x4c] = (uint8_t)(0.0L > (LD)shaperA1_); /* seta: not for unordered */
      v = (LD)(float)__builtin_fabsl((LD)shaperA1_);
      floatToBits(bits_, 0x4b, v, fltTable_1, 7);

      bits_[0x54] = (uint8_t)!((LD)shaperA2_ >= 0.0L); /* setb: also for unordered */
      v = (LD)(float)__builtin_fabsl((LD)shaperA2_);
      floatToBits(bits_, 0x53, v, fltTable_1, 7);
      bits_[0x55] = 0;

      bits_[0x5d] = (uint8_t)!((LD)shaperB1_ >= 0.0L);
      v = (LD)(float)__builtin_fabsl((LD)shaperB1_);
      floatToBits(bits_, 0x5c, v, fltTable_1, 7);

      bits_[0x65] = (uint8_t)!((LD)shaperB2_ >= 0.0L);
      v = (LD)(float)__builtin_fabsl((LD)shaperB2_);
      floatToBits(bits_, 0x64, v, fltTable_1, 7);
      bits_[0x66] = 0;

      /* group numbers of the six lists, then flagB_ and the array regions */
      for (g = 0; g < 4; g++)
        putBitsLsb(bits_, 0x67 + 4 * g, (int16_t)grp_[g], 4);
      bits_[0x77] = 0;
      for (g = 0; g < 2; g++)
        putBitsLsb(bits_, 0x78 + 4 * g, (int16_t)grp_[4 + g], 4);
      {
        uint8_t fb = flagB_;
        uint16_t n = nGrp_;

        bits_[0x80] = fb;
        for (k = 0x81; k <= 0x87; k++)
          bits_[k] = 0;
        c = 0x88;
        widx_ = c;

        /* region A: nGrp_ groups of 8 words, each word preceded by a 0 start bit */
        if (n != 0)
        {
          for (g = 0; g < n; g++)
          {
            for (e = 0; e < 8; e++)
            {
              bits_[c] = 0;
              putWord16(bits_, c + 1, (int16_t)arrA_[8 * g + e]);
              c += 17;
            }
          }
          widx_ = c;
        }

        /* region B, only when flagB_ is set */
        if (fb != 0 && n != 0)
        {
          for (g = 0; g < n; g++)
          {
            for (e = 0; e < 8; e++)
            {
              bits_[c] = 0;
              putWord16(bits_, c + 1, (int16_t)arrB_[8 * g + e]);
              c += 17;
            }
          }
          widx_ = c;
        }
      }
    }
  }

  /* tail: the last start bit, the CRC, the final start bit, zero padding up to the next multiple ABOVE of 12 * mult_ */
  {
    uint32_t C = widx_;
    uint32_t D, q, total, kk;
    bits_[C] = 0;
    widx_ = C + 1;
    frameLen_ = C + 0x11;

    crcOnes(crc_);
    crcRun(crc_, bits_, C);
    for (kk = 0; kk < 16; kk++)
      bits_[C + 1 + kk] = crc_[kk];
    bits_[C + 17] = 0;
    widx_ = C + 18;

    D = 12u * (uint32_t)mult_;
    q = (C + 18) / D;
    total = q * D + D;
    totalLen_ = total;
    for (kk = C + 18; kk < total; kk++)
      bits_[kk] = 0;
    if (flag_ != 0)
      cptSent_ = 1;
  }
}

/* ---- bit buffer -> frame fields ---- */

/**
 * @brief Decode the part of the frame that the receive machine has just completed.
 *
 * Acts on state_: 3 = SUV fields, 5 = long-frame header, 6 = float part and group numbers (sets nGrp_ = largest group number + 1 and leaves cursor_
 * at 0x88), 7 / 8 = array region A / B (continuing from cursor_).  Other states do nothing.  Words are read MSB first, the mirror of infoToBits().
 *
 * @bug  A received group number is 4 bits, so nGrp_ can reach 16 while arrA_ / arrB_ hold 6 groups: cases 7 / 8 then write past the arrays, and the
 *       state-7 length 136 * nGrp_ can exceed bits_ (2000).  A well-formed peer sends at most 6 groups.  No code in the library feeds received
 *       bits to this object today.
 */
void V92CP::evaluateInfo()
{
  uint32_t k, g, e;

  switch (state_)
  {
  case 3:
    /* SUV: 5-bit value, byte, flag */
    suv_ = 0;
    for (k = 0; k < 5; k++)
      suv_ += (uint32_t)bits_[0x1b + k] * binaryTable[k];
    suvVal_ = bits_[0x20];
    flag_ = bits_[0x21];
    break;
  case 5:
  {
    /* long-frame header: mode (2 bits), rate (5 bits), sr_ when mode_ <= 1, flag */
    int32_t a = (int8_t)bits_[0x14];
    uint8_t bl, cl;

    rate_ = 0;
    bl = (uint8_t)(bits_[0x13] & 1);
    bl = (uint8_t)(bl | (uint8_t)(a + a));
    mode_ = bl;
    modeCopy_ = bl;
    cl = 0;
    for (k = 0x19; k > 0x14; k--)
      cl = (uint8_t)((uint8_t)(cl + cl) | (uint8_t)(bits_[k] & 1));
    rate_ = cl;
    if ((int8_t)bl <= 1)
    {
      uint32_t s = bits_[0x20];
      uint32_t p = bits_[0x1f];
      sr_ = (s + s) | (p & 1);
    }
    flag_ = bits_[0x21];
    break;
  }
  case 6:
  {
    /* float part: id_, b03_, TRN1d RMS, four sign + magnitude shaper coefficients (each field followed by a skipped start bit), group numbers, flagB_ */
    uint32_t b32 = bits_[0x32];
    uint32_t b31 = bits_[0x31];
    int32_t mx = 0;
    LD acc;
    float f;
    uint32_t w;

    cursor_ = 0x34;
    id_ = (b32 + b32) | (b31 & 1);
    b03_ = bits_[0x23];
    acc = bitsToFloat(bits_, &cursor_, fltTable_2, 16);
    trn1dRms_ = (float)acc;
    cursor_ = cursor_ + 1;

    /* shaper A1, A2: magnitude, then the sign bit flips the sign of the float word */
    for (g = 0; g < 2; g++)
    {
      acc = bitsToFloat(bits_, &cursor_, fltTable_1, 7);
      f = (float)acc;
      w = wordOfCell(&f);
      k = cursor_;
      cursor_ = k + 1;
      if (bits_[k] != 0)
        w ^= 0x80000000u;
      storeWord(g == 0 ? &shaperA1_ : &shaperA2_, w);
    }
    cursor_ = cursor_ + 1;

    /* shaper B1, B2 */
    for (g = 0; g < 2; g++)
    {
      acc = bitsToFloat(bits_, &cursor_, fltTable_1, 7);
      f = (float)acc;
      w = wordOfCell(&f);
      k = cursor_;
      cursor_ = k + 1;
      if (bits_[k] != 0)
        w ^= 0x80000000u;
      storeWord(g == 0 ? &shaperB1_ : &shaperB2_, w);
    }
    cursor_ = cursor_ + 1;

    /* group numbers: four, a skipped bit, two more; mx is the largest */
    for (g = 0; g < 4; g++)
    {
      grp_[g] = 0;
      for (k = 0; k < 4; k++)
      {
        uint32_t c = cursor_;
        uint32_t v = (uint32_t)bits_[c] * binaryTable[k];
        cursor_ = c + 1;
        grp_[g] = grp_[g] + v;
      }
      if ((int32_t)grp_[g] > mx)
        mx = (int32_t)grp_[g];
    }
    cursor_ = cursor_ + 1;
    for (g = 4; g < 6; g++)
    {
      grp_[g] = 0;
      for (k = 0; k < 4; k++)
      {
        uint32_t c = cursor_;
        uint32_t v = (uint32_t)bits_[c] * binaryTable[k];
        cursor_ = c + 1;
        grp_[g] = grp_[g] + v;
      }
      if ((int32_t)grp_[g] > mx)
        mx = (int32_t)grp_[g];
    }

    {
      uint32_t c = cursor_;

      flagB_ = bits_[c];
      nGrp_ = (uint16_t)(mx + 1);
      cursor_ = c + 8;
    }
    break;
  }
  case 7:
  case 8:
  {
    /* array region A (state 7) or B (state 8): nGrp_ groups of 8 words, each a skipped start bit and 16 bits MSB first */
    uint16_t* arr = (state_ == 7) ? arrA_ : arrB_;
    uint16_t n = nGrp_;

    if (n == 0)
      break;
    for (g = 0; g < n; g++)
    {
      for (e = 0; e < 8; e++)
      {
        uint32_t acc = 0;
        cursor_ = cursor_ + 1;
        arr[8 * g + e] = 0;
        for (k = 0; k < 16; k++)
        {
          uint32_t c = cursor_;
          uint32_t v = (uint32_t)bits_[c] * binaryTable[15 - k];
          cursor_ = c + 1;
          acc += v;
        }
        arr[8 * g + e] = (uint16_t)acc;
      }
    }
    break;
  }
  default:
    break;
  }
}

/* ---- receive state machine ---- */

/**
 * @brief Feed one received bit to the receive machine.
 *
 * States: 0 counts leading ones (more than 16 = flag), 1 expects the 0 start bit (a one restarts), 2 stores the type bit, 3 collects the rest of an SUV
 * (0x22 bits), 5 the long-frame header (0x22 bits), 6 the float part (up to 0x88), 7 / 8 array regions A / B (136 bits per group), 9 the start bit
 * and CRC (17 bits), 10 the final start bit and zero padding up to a multiple of 12 * mult_.  evaluateInfo() runs when a part completes.  A run of
 * 12 * mult_ zeros while idle reports "Ed".
 *
 * @param bit  received bit (0 or 1)
 * @return 0 nothing, 1 / 2 long frame (flag_ 0 / 1), 3 / 4 SUV (flag_ 0 / 1), 5 "Ed" (CpInfoCode)
 * @note  Codes 3 and 4 are reported as 0 while the suppression window counts (400 calls after a code 1 or 2).
 * @warning The expected lengths of region A and B (`gamma`, `delta`) are function-local statics shared by every V92CP object.
 * @bug   See evaluateInfo(): a frame announcing more than 6 groups overruns arrA_ / arrB_ and bits_.  A frame with a bad CRC is dropped with a debug
 *        message ("recieved CP with bad CRC", debug level > 1).
 */
int V92CP::bitsToInfo(unsigned char bit)
{
  uint32_t ret = 0;
  uint8_t newOnes;
  uint8_t m = mult_;
  static uint32_t gamma;
  static uint32_t delta;

  /* run lengths of zeros and ones */
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
  if ((uint32_t)zeros_ == 12u * (uint32_t)m && widx_ == 0x12)
    ret = CPINFO_ED;

  switch (state_)
  {
  case 0:
    /* idle: wait for the 17-one flag */
    ones_ = newOnes;
    if (newOnes > 0x10)
      state_ = 1;
    break;
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
  case 2:
  {
    /* type bit: 1 = SUV, 0 = long frame */
    uint32_t w = widx_;

    type_ = bit;
    bits_[w] = bit;
    w++;
    ones_ = newOnes;
    widx_ = w;
    state_ = (bit != 1) ? 5u : 3u;
    break;
  }
  case 3:
  {
    uint32_t w = widx_;
    bits_[w] = bit;
    w++;
    ones_ = newOnes;
    widx_ = w;
    if (w == 0x22)
    {
      evaluateInfo();
      state_ = 9;
      count_ = 0;
    }
    break;
  }
  case 4:
    ones_ = newOnes;
    break;
  case 5:
  {
    uint32_t w = widx_;
    bits_[w] = bit;
    w++;
    ones_ = newOnes;
    widx_ = w;
    if (w == 0x22)
    {
      evaluateInfo();
      state_ = ((int8_t)mode_ > 1) ? 9u : 6u;
      count_ = 0;
    }
    break;
  }
  case 6:
  {
    uint32_t w = widx_;
    bits_[w] = bit;
    w++;
    ones_ = newOnes;
    widx_ = w;
    if (w == 0x88)
    {
      uint32_t n;
      evaluateInfo();
      n = nGrp_;
      count_ = 0;
      state_ = 7;
      gamma = n * 136u;
    }
    break;
  }
  case 7:
  {
    uint32_t w = widx_;
    uint32_t n = count_ + 1;
    bits_[w] = bit;
    ones_ = newOnes;
    w++;
    widx_ = w;
    count_ = n;
    if (n == gamma)
    {
      uint32_t fb, ng;
      evaluateInfo();
      fb = flagB_;
      ng = nGrp_;
      count_ = 0;
      state_ = (fb == 0) ? 9u : 8u;
      delta = ng * 136u;
    }
    break;
  }
  case 8:
  {
    uint32_t w = widx_;
    uint32_t n = count_ + 1;
    bits_[w] = bit;
    ones_ = newOnes;
    w++;
    widx_ = w;
    count_ = n;
    if (n == delta)
    {
      evaluateInfo();
      state_ = 9;
      count_ = 0;
    }
    break;
  }
  case 9:
  {
    uint32_t w = widx_;
    uint32_t n;
    bits_[w] = bit;
    n = count_ + 1;
    if (n != 0x11)
    {
      count_ = n;
      widx_ = w + 1;
      ones_ = newOnes;
    }
    else
    {
      /* the frame is complete: check the CRC */
      uint32_t fl = w + 1;

      frameLen_ = fl;
      crcOnes(crc_);
      crcRun(crc_, bits_, fl - 0x11);
      if (crcDiffSum(crc_, bits_ + (fl - 16)) == 0)
      {
        widx_ = fl;
        count_ = 0x11;
        state_ = 10;
        ones_ = newOnes;
      }
      else
      {
        unsigned level;
        ones_ = 0;
        zeros_ = 0;
        widx_ = 0x12;
        level = dsplibs_debug_level;
        count_ = 0;
        state_ = 0;
        if (level > 1)
          dsplibs_debug_printf("V92CP: recieved CP with bad CRC\r\n");
      }
    }
    break;
  }
  case 10:
  {
    uint32_t w = widx_ + 1;
    if (bit == 0)
    {
      widx_ = w;
    }
    else
    {
      newOnes = 0;
      widx_ = 0x12;
      count_ = 0;
      state_ = 0;
      zeros_ = 0;
    }
    if (widx_ % (12u * (uint32_t)m) != 0)
    {
      ones_ = newOnes;
    }
    else
    {
      ones_ = 0;
      ret = 0;
      widx_ = 0x12;
      count_ = 0;
      state_ = 0;
      zeros_ = 0;
      if (type_ == 1)
        ret = (flag_ == 0) ? CPINFO_SUV : CPINFO_SUVT;
      else
        ret = (flag_ == 0) ? CPINFO_CP : CPINFO_CPT;
    }
    break;
  }
  default:
    ones_ = newOnes;
    break;
  }

  /* the suppression window after a frame: codes 3 and 4 are reported as 0 while it counts (400 calls) */
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
    if (ret - 3 <= 1)
      ret = 0;
    if (s == 0x190)
    {
      supp_ = -1;
      return (int)ret;
    }
    supp_ = s;
    return (int)ret;
  }
}

/* ---- conversions between mapping parameters and CP frames ---- */

/**
 * @brief Fill a V92CP from the mapping parameters and the additional CP info.
 *
 * Dedupes the six lists into nGrp_ groups (grp_[] gives each list's group), copies the header, shaper and SUV fields, packs list variant A (and B
 * when mp->flag_ != 0) into the mask arrays arrA_ / arrB_, and sets rate_ = kBits_ - (mode_ ? 0x14 : 8).
 *
 * @param mp  mapping parameters (idx_[] is rewritten)
 * @param in  additional CP info (flag, mode, TRN1d RMS, byte, SUV value)
 * @param cp  frame to fill
 * @warning A list element >= 128 writes beyond its group's mask words (listToMask()).
 */
void setV92CPpckFromParamsInfo(V90MappingParams* mp, tagV90AdditionalCPinfo* in, V92CP* cp)
{
  unsigned n = dedupeGroups(mp, cp->grp_);
  int g2;

  cp->nGrp_ = (uint16_t)n;
  cp->flag_ = (uint8_t)in->flag_;
  cp->mode_ = (uint8_t)in->mode_;
  cp->b03_ = (uint8_t)in->wc_;
  copyWord(&cp->trn1dRms_, &in->trn1dRms_);
  cp->suvVal_ = in->suvVal_;
  copyWord(&cp->shaperA1_, &mp->shaperA1_);
  cp->sr_ = mp->sr_;
  copyWord(&cp->shaperA2_, &mp->shaperA2_);
  cp->id_ = mp->id_;
  copyWord(&cp->shaperB1_, &mp->shaperB1_);
  cp->flagB_ = (uint8_t)mp->flag_;
  copyWord(&cp->shaperB2_, &mp->shaperB2_);

  /* region A: variant A of each distinct list as mask words */
  if ((uint16_t)n != 0)
  {
    for (g2 = 0; (unsigned)g2 < cp->nGrp_; g2++)
    {
      unsigned gi = (g2 > 5) ? 0u : (unsigned)g2;
      unsigned idx = mp->idx_[gi];
      unsigned cnt = mp->cnt_[idx];
      uint16_t* dst = &cp->arrA_[8 * g2];
      int e;
      for (e = 0; e < 8; e++)
        dst[e] = 0;
      listToMask(dst, mp->listA_[idx], cnt);
    }
  }

  /* region B: variant B of each distinct list */
  if (cp->flagB_ != 0 && cp->nGrp_ != 0)
  {
    for (g2 = 0; (unsigned)g2 < cp->nGrp_; g2++)
    {
      unsigned gi = (g2 > 5) ? 0u : (unsigned)g2;
      unsigned idx = mp->idx_[gi];
      unsigned cnt = mp->cnt_[idx];
      uint16_t* dst = &cp->arrB_[8 * g2];
      int e;
      for (e = 0; e < 8; e++)
        dst[e] = 0;
      listToMask(dst, mp->listB_[idx], cnt);
    }
  }

  if (cp->mode_ != 0)
    cp->rate_ = (uint8_t)(mp->kBits_ - 0x14);
  else
    cp->rate_ = (uint8_t)(mp->kBits_ - 8);
}

/**
 * @brief Fill mapping parameters from a V92CP (the reverse of setV92CPpckFromParamsInfo()); not called by the library.
 * @param mp  mapping parameters to fill (shaper words, sr_, id_, flag_, idx_, both list variants and their counts, kBits_)
 * @param cp  decoded frame
 * @warning grp_[] entries above 5 index beyond arrA_ / arrB_.
 */
void setParamsInfoFromV92CPUnPck(V90MappingParams* mp, V92CP* cp)
{
  unsigned k;

  copyWord(&mp->shaperA1_, &cp->shaperA1_);
  mp->sr_ = cp->sr_;
  copyWord(&mp->shaperA2_, &cp->shaperA2_);
  mp->id_ = cp->id_;
  copyWord(&mp->shaperB1_, &cp->shaperB1_);
  copyWord(&mp->shaperB2_, &cp->shaperB2_);
  mp->flag_ = (cp->flagB_ != 0) ? 1u : 0u;
  for (k = 0; k < 6; k++)
    mp->idx_[k] = cp->grp_[k];

  /* list variant A, then B (from region A when the frame has no region B) */
  for (k = 0; k < 6; k++)
  {
    unsigned g = (k < 6) ? k : 0u;
    mp->cnt_[g] = 0;
    mp->cnt_[g] = maskToList(mp->listA_[g], &cp->arrA_[8 * cp->grp_[k]], mp->cnt_[g]);
  }

  for (k = 0; k < 6; k++)
  {
    unsigned g = (k < 6) ? k : 0u;
    const uint16_t* src = (cp->flagB_ != 0) ? &cp->arrB_[8 * cp->grp_[k]] : &cp->arrA_[8 * cp->grp_[k]];
    mp->cnt_[g] = 0;
    mp->cnt_[g] = maskToList(mp->listB_[g], src, mp->cnt_[g]);
  }

  {
    int32_t r = (int8_t)cp->rate_;
    mp->kBits_ = (uint32_t)(r + (cp->mode_ != 0 ? 0x14 : 8));
  }
}

/**
 * @brief Fill mapping parameters from a CPUnPck structure; not called by the library.
 * @param mp  mapping parameters to fill
 * @param u   unpacked CP; u->sub_ must point to a readable structure (its second word selects the rate offset)
 * @warning u->sub_ is dereferenced without a NULL check; group numbers above 5 index beyond the array regions.
 */
void setParamsInfoFromCPUnPck(V90MappingParams* mp, CPUnPck* u)
{
  unsigned k;

  mp->sr_ = u->w_[0];
  mp->id_ = u->w_[1];
  copyWord(&mp->shaperA1_, (const float*)&u->w_[2]);
  copyWord(&mp->shaperA2_, (const float*)&u->w_[3]);
  copyWord(&mp->shaperB1_, (const float*)&u->w_[4]);
  copyWord(&mp->shaperB2_, (const float*)&u->w_[5]);
  mp->flag_ = (u->flag_ != 0) ? 1u : 0u;
  for (k = 0; k < 6; k++)
    mp->idx_[k] = u->grp_[k];

  for (k = 0; k < 6; k++)
  {
    unsigned g = (k < 6) ? k : 0u;
    mp->cnt_[g] = 0;
    mp->cnt_[g] = maskToList(mp->listA_[g], &u->arrA_[8 * (unsigned)u->grp_[k]], mp->cnt_[g]);
  }

  for (k = 0; k < 6; k++)
  {
    unsigned g = (k < 6) ? k : 0u;
    const uint16_t* src = (u->flag_ != 0) ? &u->arrB_[8 * (unsigned)u->grp_[k]] : &u->arrA_[8 * (unsigned)u->grp_[k]];
    mp->cnt_[g] = 0;
    mp->cnt_[g] = maskToList(mp->listB_[g], src, mp->cnt_[g]);
  }

  {
    uint32_t rate = u->rate_;
    const uint32_t* sub = u->sub_;
    if (sub[1] != 0)
      mp->kBits_ = rate + 0x14;
    else
      mp->kBits_ = rate + 8;
  }
}

/**
 * @brief Convert a received V90CP into the V.92 modulator's parameter block.
 *
 * Copies the three presence flags, the trellis state, extendEu and the rate (w0_ = 0x22 + 2 * drn), computes the constellation gain
 * (prefilterGain_ * 2^-18, then * 4000), and, depending on the flags, copies the 12 modulus-encoder words (flag A), the four prefilter arrays scaled
 * to float (flag B; counts clamped to 0x148) and the six constellations (flag C; lengths clamped to 0x80).  Every print is guarded by
 * `dsplibs_debug_level > 1`, which is re-read before each one, so the first block and the four array-length prints stop at the first failed guard.
 *
 * @param d   parameter block to fill; its arr_[] and lcp_[] must already point to the owned arrays (V92createFilterCoefficients() /
 *            V92createConstellations()) when flag B / C is set
 * @param cp  received frame
 * @warning arr_[] / lcp_[] are not checked for NULL.
 */
void V92setParamsInfoFromCPUnPck(V92MappingParams* d, V90CP* cp)
{
  unsigned k;
  float x;
  LD t;

  /* debug dump of the received fields */
  if (dsplibs_debug_level > 1)
  {
    dsplibs_debug_printf("V92 mapping parameters:\r\n");
    do
    {
      if (!(dsplibs_debug_level > 1))
        break;
      dsplibs_debug_printf("CPObj->modulosEncoderPresent = %d\r\n", cp->flagA_);
      if (!(dsplibs_debug_level > 1))
        break;
      dsplibs_debug_printf("CPObj->prefilterPrecoderPresent = %d\r\n", cp->flagB_);
      if (!(dsplibs_debug_level > 1))
        break;
      dsplibs_debug_printf("CPObj->constellationPresent = %d\r\n", cp->flagC_);
      if (!(dsplibs_debug_level > 1))
        break;
      dsplibs_debug_printf("CPObj->trellisState = %d\r\n", (int)(int8_t)cp->trellisState_);
      if (!(dsplibs_debug_level > 1))
        break;
      dsplibs_debug_printf("CPObj->prefilterGain = %d\r\n", cp->prefilterGain_);
      if (!(dsplibs_debug_level > 1))
        break;
      dsplibs_debug_printf("CPObj->extendEu = %d\r\n", (int)(int8_t)cp->extendEu_);
      if (!(dsplibs_debug_level > 1))
        break;
      dsplibs_debug_printf("CPObj->drn = %d\r\n", (int)(int8_t)cp->drn_);
      if (!(dsplibs_debug_level > 1))
        break;
      dsplibs_debug_printf("Upstream rate = %d\r\n", dsp_trunc_i32((LD)((int)(int8_t)cp->drn_ + 0x11) * (LD)1333.3333740234375f));
    } while (0);
  }

  /* flags and scalar fields */
  d->a_ = cp->flagA_;
  d->b_ = cp->flagB_;
  d->c_ = cp->flagC_;
  d->w0_ = 0x22 + 2 * (int)(int8_t)cp->drn_;
  d->trellis_ = (int8_t)cp->trellisState_;
  d->extendEu_ = (int8_t)cp->extendEu_;

  /* constellation gain: (float)(prefilterGain_ * 2^-18), then * 4000 (extended product, stored rounded; the second print shows the UNROUNDED product) */
  x = (float)((LD)(uint64_t)cp->prefilterGain_ * (LD)3.814697265625e-06f);
  if (dsplibs_debug_level > 1)
  {
    Split s;
    d->gain_ = x;
    s = splitValue((LD)x, (LD)1000000.0f);
    dsplibs_debug_printf("V92Modulator: constellation gain (before Lu multiplication) = %c%d.%06d\r\n", s.sign, s.ip, s.fp);
    x = d->gain_;
  }
  t = (LD)x * (LD)4000.0f;
  d->gain_ = (float)t;
  if (dsplibs_debug_level > 1)
  {
    Split s = splitValue(t, (LD)1000000.0f);
    dsplibs_debug_printf("V92Modulator: constellation gain (after Lu multiplication) = %c%d.%06d\r\n", s.sign, s.ip, s.fp);
  }

  /* flag A: the 12 modulus-encoder words */
  if (d->a_ != 0)
  {
    for (k = 0; k < 12; k++)
    {
      if (dsplibs_debug_level > 1)
        dsplibs_debug_printf("CPObj->M[%d] = %d\r\n", k, cp->pair_[k]);
    }
    for (k = 0; k < 12; k++)
      d->m_[k] = cp->pair_[k];
  }

  /* flag B: the four prefilter arrays, 16-bit words scaled to float (arrays 0 and 2 by 2^-15, 1 and 3 by 2^-14) */
  if (d->b_ != 0)
  {
    unsigned n0, n1, n2, n3;

    do
    {
      if (!(dsplibs_debug_level > 1))
        break;
      dsplibs_debug_printf("CPObj->lz1 = %d\r\n", cp->cnt_[0]);
      if (!(dsplibs_debug_level > 1))
        break;
      dsplibs_debug_printf("CPObj->lz2 = %d\r\n", cp->cnt_[2]);
      if (!(dsplibs_debug_level > 1))
        break;
      dsplibs_debug_printf("CPObj->lp1 = %d\r\n", cp->cnt_[1]);
      if (!(dsplibs_debug_level > 1))
        break;
      dsplibs_debug_printf("CPObj->lp2 = %d\r\n", cp->cnt_[3]);
    } while (0);

    d->cnt_[0] = cp->cnt_[0];
    d->cnt_[2] = cp->cnt_[2];
    d->cnt_[1] = cp->cnt_[1];
    d->cnt_[3] = cp->cnt_[3];
    if (d->cnt_[0] > 0x148)
      d->cnt_[0] = 0x148;
    if (d->cnt_[2] > 0x148)
      d->cnt_[2] = 0x148;
    if (d->cnt_[1] > 0x148)
      d->cnt_[1] = 0x148;
    if (d->cnt_[3] > 0x148)
      d->cnt_[3] = 0x148;

    n0 = d->cnt_[0];
    n1 = d->cnt_[1];
    n2 = d->cnt_[2];
    n3 = d->cnt_[3];
    for (k = 0; k < n0; k++)
      d->arr_[0][k] = (float)((LD)(int16_t)cp->arrs_[k] * (LD)3.0517578125e-05f);
    for (k = 0; k < n1; k++)
      d->arr_[1][k] = (float)((LD)(int16_t)cp->arrs_[384 + k] * (LD)6.103515625e-05f);
    for (k = 0; k < n2; k++)
      d->arr_[2][k] = (float)((LD)(int16_t)cp->arrs_[768 + k] * (LD)3.0517578125e-05f);
    for (k = 0; k < n3; k++)
      d->arr_[3][k] = (float)((LD)(int16_t)cp->arrs_[1152 + k] * (LD)6.103515625e-05f);
  }

  /* flag C: the six constellations */
  if (d->c_ == 0)
    return;

  for (k = 0; k < 6; k++)
  {
    if (dsplibs_debug_level > 1)
      dsplibs_debug_printf("CPObj->LC[%d] = %d\r\n", k, cp->hcnt_[k]);
  }
  for (k = 0; k < 6; k++)
  {
    if (dsplibs_debug_level > 1)
      dsplibs_debug_printf("CPObj->indexConstel[%d] = %d\r\n", k, cp->nib_[k]);
  }
  for (k = 0; k < 6; k++)
  {
    d->lc_[k] = cp->hcnt_[k];
  }
  for (k = 0; k < 6; k++)
    if (d->lc_[k] > 0x80)
      d->lc_[k] = 0x80;

  {
    static const char* const hdr[6] = {"======== Constellation LC 1 =============\r\n", "======== Constellation LC 2 =============\r\n",
                                       "======== Constellation LC 3 =============\r\n", "======== Constellation LC 4 =============\r\n",
                                       "======== Constellation LC 5 =============\r\n", "======== Constellation LC 6 =============\r\n"};
    static const char* const elem[6] = {"\tconst1[%d] = %d\r\n", "\tconst2[%d] = %d\r\n", "\tconst3[%d] = %d\r\n",
                                        "\tconst4[%d] = %d\r\n", "\tconst5[%d] = %d\r\n", "\tconst6[%d] = %d\r\n"};
    unsigned j;
    for (j = 0; j < 6; j++)
    {
      if (d->lc_[j] != 0)
      {
        unsigned i;
        if (dsplibs_debug_level > 1)
          dsplibs_debug_printf(hdr[j]);
        for (i = 0; i < d->lc_[j]; i++)
        {
          if (dsplibs_debug_level > 1)
            dsplibs_debug_printf(elem[j], i, cp->heap_[j][i]);
        }
      }
    }
  }
  if (d->lc_[0] != 0 || d->lc_[1] != 0 || d->lc_[2] != 0 || d->lc_[3] != 0 || d->lc_[4] != 0 || d->lc_[5] != 0)
  {
    if (dsplibs_debug_level > 1)
      dsplibs_debug_printf("==================================\r\n");
  }

  for (k = 0; k < 6; k++)
    d->idxc_[k] = cp->nib_[k];

  for (k = 0; k < 6; k++)
  {
    unsigned i;
    if (d->lc_[k] != 0)
    {
      const uint32_t* src = cp->heap_[k];
      uint32_t* dst = d->lcp_[k];
      for (i = 0; i < d->lc_[k]; i++)
        dst[i] = src[i];
    }
  }
}
