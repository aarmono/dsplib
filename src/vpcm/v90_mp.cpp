/**
 * @file v90_mp.cpp
 * @brief V90MP: encoding of the MP / MPnot frame into a bit buffer with CRC (infoToBits), and the bit-serial receive state machine (bitsToInfo).
 *
 * Definitions of the class declared in v90_mp.h.  Everything is integer / byte arithmetic.  The CRC primitives are shared with the CP codecs (pcm_cp_crc.h).
 *
 * Frame layout in `bits_` (one bit per byte, least significant bit of a multi-bit field first; the indices that are multiples of 17 hold a zero bit
 * that the CRC skips).  Indices, hexadecimal:
 * - 00 .. 10: seventeen 1 bits (the frame sync, consumed by the receiver and not stored); 11: start bit 0;
 * - 12: frame type (0 short, 1 long); 13 .. 17: reserved 0; 18 .. 1b: rate; 1c: 0; 1d, 1e: trellis (bit 0, bit 1); 1f: non-linear coding; 20: shaping;
 *   21: CPack; 22, 23: 0; 24 .. 31: rate mask bits 0 .. 13; 32, 33: 0;
 * - long frame only: 34 .. 43 h1 real; 44: 0; 45 .. 54 h1 imaginary; 55: 0; 56 .. 65 h2 real; 66: 0; 67 .. 76 h2 imaginary; 77: 0; 78 .. 87 h3 real; 88: 0;
 *   89 .. 98 h3 imaginary; 99 .. aa: 0; ab .. ba: CRC; bb: 0 (end of the frame, frameLen_ = 0xbb);
 * - short frame: 34 .. 44: 0; 45 .. 54: CRC; 55: 0 (end of the frame, frameLen_ = 0x55);
 * - the frame is followed by zero padding up to totalLen_ bits.
 * The CRC covers the indices 12 .. 43 (short) or 12 .. a9 (long), skipping the multiples of 17 (22, 33, 44, ...).
 *
 * Entry points: infoToBits() (transmit) and bitsToInfo() (receive, one bit per call), used by V90Modulator, V90Phase4Modulator, V90Phase4Demodulator and
 * V90Demodulator.  resetCRC(), calcCRC(), evaluateCRC(), resetDetector(), calcSequenceLength() and PrintBase2() are helpers that no other file calls.
 *
 * Behaviour worth knowing:
 * - the CRC register is 16 bytes, one bit per byte; only bit 0 feeds the taps, shifts copy whole bytes, and a buffer byte is added as a whole byte (so the
 *   fields cPack_, nonLin_, shaping_ must be 0 or 1);
 * - evaluateInfo() copies nonLin_, shaping_ and cPack_ as whole bytes and treats a rate-mask byte as 1 when it is not 0;
 * - infoToBits() leaves bits_[0x1d] / bits_[0x1e] unchanged when trellis_ is outside 0 .. 3 (signed compare);
 * - bitsToInfo() flips bit 0x70 of a long frame whose CRC fails and checks the CRC again (see there); MP frames print the rate-mask and h lines only at debug
 *   level 3 and above, MPnot frames at level 2 and above.  dsplibs_debug_level is unsigned and is re-read before each print.
 */

#include "v90_mp.h"
#include "core/dsp_host.h"

/* ------------------------------------------------------------------------------------------------ helpers */

#include "pcm_cp_crc.h" /* crcRun / crcDiffSum / crcOnes */

/** @brief The CRC loop bound of a frame type byte: 0x44 (the zero bit before the CRC field) for a short frame, 0xaa for a long frame. */
static inline unsigned crcBound(uint8_t type) { return type == 0 ? 0x44u : 0xaau; }

/** @brief Assemble the bits bits[hi] .. bits[lo] (bit 0 of each byte) into a number; bits[lo] is the least significant. */
static inline uint32_t packBits(const uint8_t* bits, int hi, int lo)
{
  uint32_t v = 0;
  int i;
  for (i = hi; i >= lo; i--)
    v = (v << 1) | (uint32_t)(bits[i] & 1);
  return v;
}

/** @brief Store the low `n` bits of `v`, least significant first, as 0 / 1 bytes at bits[at ..].  A negative `v` is sign-extended (arithmetic shift). */
static inline void putBits(uint8_t* bits, int at, int32_t v, int n)
{
  int k;
  for (k = 0; k < n; k++)
  {
    bits[at + k] = (uint8_t)(v & 1);
    v >>= 1;
  }
}

/* ------------------------------------------------------------------------------------------------ construction / reset */

/** Idle receive machine, frame counters cleared; the info fields, bit buffer and CRC register are left as they are. */
V90MP::V90MP()
{
  widx_ = 0x12;
  state_ = 0;
  ones_ = 0;
  zeros_ = 0;
  nMp_ = 0;
  nMpNot_ = 0;
}

V90MP::~V90MP() {}

/** Preset the CRC register to ones. */
void V90MP::resetCRC() { crcOnes(crc_); }

/** Back to idle: the next bit is looked at as a possible frame start.  Frame counters are kept. */
void V90MP::resetDetector()
{
  widx_ = 0x12;
  state_ = 0;
  ones_ = 0;
  zeros_ = 0;
}

/** resetDetector() and clear the MP / MPnot counters. */
void V90MP::reset()
{
  widx_ = 0x12;
  state_ = 0;
  ones_ = 0;
  zeros_ = 0;
  nMp_ = 0;
  nMpNot_ = 0;
}

/* ------------------------------------------------------------------------------------------------ CRC */

/** Run the CRC over the frame bits up to the CRC field (the register is not preset here). */
void V90MP::calcCRC() { crcRun(crc_, bits_, crcBound(frameType_)); }

/**
 * Preset the register, run it over the frame and compare it with the 16 bits that end the frame.
 * @return 1 when the CRC field matches, else 0
 */
int V90MP::evaluateCRC()
{
  crcOnes(crc_);
  crcRun(crc_, bits_, crcBound(frameType_));
  return crcDiffSum(crc_, bits_ + (frameLen_ - 16)) == 0;
}

/**
 * @param length receives totalLen_
 * @return the bit buffer bits_ (not a copy); meaningful after infoToBits()
 */
unsigned char* V90MP::getBitVector(unsigned int& length)
{
  length = totalLen_;
  return bits_;
}

/**
 * totalLen_ = frameLen_ + 1 rounded up to a multiple of trn2dD_ (kept in 8 bits).
 * @warning trn2dD_ == 0 divides by zero.  The result is truncated to 8 bits, but bits_ holds only 0xe6 entries; infoToBits() fills up to totalLen_
 *          (see there).
 */
void V90MP::calcSequenceLength()
{
  unsigned v = (unsigned)frameLen_ + 1;
  unsigned q = v / trn2dD_;
  if (v == q * trn2dD_)
    totalLen_ = (uint8_t)v;
  else
    totalLen_ = (uint8_t)((q + 1) * trn2dD_);
}

/* ------------------------------------------------------------------------------------------------ buffer -> info */

/**
 * Decode the info fields from bits_ of a complete, CRC-checked frame (field positions in the file comment).  The rate-mask bytes count as 1 when they are
 * not 0; nonLin_, shaping_ and cPack_ are copied as whole bytes.  A short frame has no precoder coefficients: h1 .. h3 are set to 0.
 */
void V90MP::evaluateInfo()
{
  uint8_t ft = frameType_;
  int k;

  type_ = ft;
  rate_ = (uint8_t)packBits(bits_, 0x1b, 0x18);
  rateMask_ = 0;
  trellis_ = (uint8_t)((bits_[0x1d] & 1) | ((bits_[0x1e] & 1) << 1));
  nonLin_ = bits_[0x1f];
  shaping_ = bits_[0x20];
  cPack_ = bits_[0x21];
  for (k = 0x24; k <= 0x31; k++)
    if (bits_[k] != 0)
      rateMask_ = (uint16_t)(rateMask_ | (1u << (k - 0x24)));

  h1Re_ = 0;
  h1Im_ = 0;
  h2Re_ = 0;
  h2Im_ = 0;
  h3Re_ = 0;
  h3Im_ = 0;

  if (ft != 0)
  {
    h1Re_ = (int16_t)packBits(bits_, 0x43, 0x34);
    h1Im_ = (int16_t)packBits(bits_, 0x54, 0x45);
    h2Re_ = (int16_t)packBits(bits_, 0x65, 0x56);
    h2Im_ = (int16_t)packBits(bits_, 0x76, 0x67);
    h3Re_ = (int16_t)packBits(bits_, 0x87, 0x78);
    h3Im_ = (int16_t)packBits(bits_, 0x98, 0x89);
  }
}

/* ------------------------------------------------------------------------------------------------ info -> buffer */

/**
 * Build the transmit frame in bits_ from the info fields: sync ones and start bit, the fields (positions in the file comment), the h coefficients (long
 * frames), the CRC, and zero padding up to totalLen_.  Sets frameType_ = type_, frameLen_ (0x55 / 0xbb) and totalLen_ (calcSequenceLength()).
 * @bug   type_ == 0 (short frame): the CRC copied to bits_[0x45 .. 0x54] is wiped again: bits_[0x45] is cleared and the padding fill starts at 0x45, so
 *        the whole CRC field leaves as zeros and a receiver checking it rejects the frame.  The long-frame branch has the correct positions (clears the end
 *        bit 0xbb and fills from 0xbc); the short-frame constants should be 0x55 / 0x56.
 * @warning The padding fill runs up to totalLen_ (at most 255) but bits_ has 0xe6 entries; for totalLen_ above 0xe6 it overwrites crc_, trn2dD_, totalLen_ and
 *          frameLen_ (index 0xfc is totalLen_ itself, which is why the bound is read once).  Callers keep trn2dD_ small.  Also trellis_ outside 0 .. 3
 *          leaves bits_[0x1d] / bits_[0x1e] as they were; cPack_, nonLin_ and shaping_ are stored as whole bytes and must be 0 or 1.
 */
void V90MP::infoToBits()
{
  uint8_t type = type_;
  int k;
  int8_t tr;
  unsigned tl;

  /* sync, start bit, frame type, reserved bits and the rate */
  for (k = 0; k <= 0x10; k++)
    bits_[k] = 1;
  bits_[0x11] = 0;
  bits_[0x12] = type;
  for (k = 0x13; k <= 0x17; k++)
    bits_[k] = 0;
  putBits(bits_, 0x18, (int8_t)rate_, 4);
  bits_[0x1c] = 0;

  /* trellis selector as two bits (a value outside 0 .. 3 leaves them unchanged) */
  tr = (int8_t)trellis_;
  if (tr == 1)
  {
    bits_[0x1d] = 1;
    bits_[0x1e] = 0;
  }
  else if (tr <= 1)
  {
    if (tr == 0)
    {
      bits_[0x1d] = 0;
      bits_[0x1e] = 0;
    }
  }
  else if (tr == 2)
  {
    bits_[0x1d] = 0;
    bits_[0x1e] = 1;
  }
  else if (tr == 3)
  {
    bits_[0x1d] = 1;
    bits_[0x1e] = 1;
  }

  /* option flags, rate mask (14 bits) and the reserved zeros */
  bits_[0x1f] = nonLin_;
  bits_[0x20] = shaping_;
  bits_[0x21] = cPack_;
  bits_[0x22] = 0;
  bits_[0x23] = 0;
  {
    int32_t m = (int16_t)rateMask_;
    for (k = 0; k < 14; k++)
      bits_[0x24 + k] = (uint8_t)((m >> k) & 1);
  }
  bits_[0x32] = 0;
  bits_[0x33] = 0;

  /* frame geometry */
  frameType_ = type;
  frameLen_ = (type == 0) ? 0x55 : 0xbb;
  calcSequenceLength();
  tl = totalLen_; /* the padding bound is read once: the fill below can overwrite totalLen_ (bits_ index 0xfc) and must not follow it */

  if (type != 0)
  {
    /* long frame: the three complex precoder coefficients, each 16 bits followed by a zero bit; then reserved zeros, CRC and padding */
    putBits(bits_, 0x34, h1Re_, 16);
    bits_[0x44] = 0;
    putBits(bits_, 0x45, h1Im_, 16);
    bits_[0x55] = 0;
    putBits(bits_, 0x56, h2Re_, 16);
    bits_[0x66] = 0;
    putBits(bits_, 0x67, h2Im_, 16);
    bits_[0x77] = 0;
    putBits(bits_, 0x78, h3Re_, 16);
    bits_[0x88] = 0;
    putBits(bits_, 0x89, h3Im_, 16);
    bits_[0x99] = 0;
    for (k = 0x9a; k <= 0xa9; k++)
      bits_[k] = 0;
    bits_[0xaa] = 0;

    crcOnes(crc_);
    crcRun(crc_, bits_, 0xaau);
    for (k = 0; k < 16; k++)
      bits_[0xab + k] = crc_[k];
    bits_[0xbb] = 0;
    for (k = 0xbc; (unsigned)k < tl; k++)
      bits_[k] = 0;
  }
  else
  {
    /* short frame: no coefficients, CRC after the zero bit at 0x44 */
    for (k = 0x34; k <= 0x43; k++)
      bits_[k] = 0;
    bits_[0x44] = 0;

    crcOnes(crc_);
    crcRun(crc_, bits_, 0x44u);
    for (k = 0; k < 16; k++)
      bits_[0x45 + k] = crc_[k];
    bits_[0x45] = 0; /* defect (see above): clears the first CRC bit, and the fill below starts at 0x45 as well */
    for (k = 0x45; (unsigned)k < tl; k++)
      bits_[k] = 0;
  }
}

/* ------------------------------------------------------------------------------------------------ small helpers of the receive machine */

/**
 * Write the binary digits of `value`, most significant first, as text.
 * @param out    destination, NUL-terminated on return (up to 33 bytes needed)
 * @param value  the number (only the low 32 bits are used)
 * @param nbits  number of digits to print (the top bit shown is bit nbits - 1, taken modulo 32); 0 prints all digits without leading zeros
 * @warning `out` is not bounded.
 */
void V90MP::PrintBase2(char* out, unsigned long value, unsigned short nbits)
{
  uint32_t v = (uint32_t)value;
  uint32_t mask = 0x80000000u;
  int started = (nbits != 0);
  if (nbits != 0)
    mask = 1u << ((nbits - 1) & 31);
  while (mask != 0)
  {
    if ((v & mask) != 0)
    {
      *out++ = '1';
      started = 1;
    }
    else if (started)
    {
      *out++ = '0';
    }
    mask >>= 1;
  }
  *out = 0;
}

/** Print the number of MP and MPnot frames received (debug level above 1). */
void V90MP::printNofRecievedMpMpNot()
{
  if (dsplibs_debug_level > 1)
    dsplibs_debug_printf("V90MP: received %d MP, %d MPNot\r\n", nMp_, nMpNot_);
}

/** @brief The "Rate Mask" text: the 14 low bits of the mask word as 0 / 1 characters, bit 13 first (`s` needs 15 bytes). */
static inline void rateMaskText(char* s, uint16_t maskWord)
{
  int32_t m = (int16_t)maskWord;
  uint32_t b;
  for (b = 0x2000; b != 0; b >>= 1)
    *s++ = ((m & (int32_t)b) != 0) ? '1' : '0';
  *s = 0;
}

/* ------------------------------------------------------------------------------------------------ receive state machine */

/**
 * Feed one received bit.  States of `state_`:
 * - 0: hunt for a run of more than 16 ones; 1: a 0 bit must follow (else back to idle);
 * - 2: the frame type bit: stored at bits_[0x12], sets frameType_, frameLen_ (0x55 / 0xbb) and totalLen_;
 * - 3: payload bits are stored at widx_ until frameLen_ bits are in; then the CRC is checked.  If it fails and the frame is long, bit 0x70 (inside the h2
 *      imaginary field) is inverted and the CRC checked again; a frame accepted that way is reported at debug level 3.  A second failure drops to idle;
 * - 4: the end bit and padding up to totalLen_ are counted but not stored; at the end evaluateInfo() decodes the frame and the result is MPINFO_MP
 *      (cPack_ == 0) or MPINFO_MPNOT; the machine returns to idle.
 * Separately, an idle machine (widx_ == 0x12) that sees 2 * trn2dD_ zero bits in a row reports MPINFO_ED, whatever state it is in.
 * @param bit  the received bit
 * @return an MpInfoCode; MPINFO_ED is reported on the bit that completes the zero run, and the state machine still runs on that bit
 * @warning trn2dD_ == 0 makes every 1 bit look like the end of data (when idle).  The Ed test compares an 8-bit run count with 2 * trn2dD_.
 */
int V90MP::bitsToInfo(int bit)
{
  int ret = 0;
  int ed;

  /* run-length bookkeeping and end-of-data detection */
  if (bit == 0)
  {
    zeros_++;
    ones_ = 0;
    ed = ((uint32_t)zeros_ == trn2dD_ * 2u);
  }
  else
  {
    zeros_ = 0;
    ones_++;
    ed = (0u == trn2dD_ * 2u);
  }

  if (ed && widx_ == 0x12)
  {
    if (dsplibs_debug_level > 1)
      dsplibs_debug_printf("V90MP: Ed detected\r\n");
    ret = MPINFO_ED;
  }

  /* frame receive machine */
  switch (state_)
  {
  case 0:
    if (ones_ > 0x10)
      state_ = 1;
    break;
  case 1:
    if (bit == 0)
    {
      state_ = 2;
    }
    else
    {
      widx_ = 0x12;
      state_ = 0;
      ones_ = 0;
      zeros_ = 0;
    }
    break;
  case 2:
  {
    uint8_t idx = widx_;
    frameType_ = (uint8_t)bit;
    widx_ = (uint8_t)(idx + 1);
    bits_[idx] = (uint8_t)bit;
    frameLen_ = ((uint8_t)bit == 0) ? 0x55 : 0xbb;
    calcSequenceLength();
    state_ = 3;
    break;
  }
  case 3:
  {
    uint8_t idx = widx_;
    uint8_t n;
    bits_[idx] = (uint8_t)bit;
    n = (uint8_t)(idx + 1);
    if (n != frameLen_)
    {
      widx_ = n;
    }
    else if (evaluateCRC())
    {
      state_ = 4;
      widx_ = n;
    }
    else
    {
      if (frameLen_ > 0x6f)
        bits_[0x70] = (bits_[0x70] == 0);
      if (evaluateCRC())
      {
        state_ = 4;
        widx_ = n;
        if (dsplibs_debug_level > 2)
          dsplibs_debug_printf("V90MP: recieved MP with modified good CRC\r\n");
      }
      else
      {
        widx_ = 0x12;
        ones_ = 0;
        zeros_ = 0;
        state_ = 0;
        if (dsplibs_debug_level > 1)
          dsplibs_debug_printf("V90MP: recieved MP with bad CRC\r\n");
      }
    }
    break;
  }
  case 4:
  {
    uint8_t n = (uint8_t)(widx_ + 1);
    widx_ = n;
    if (n == totalLen_)
    {
      char text[16];
      unsigned level;

      /* a whole frame (with padding) has arrived: decode it, go idle, report MP or MPnot (the first two of each kind are printed) */
      evaluateInfo();
      widx_ = 0x12;
      state_ = 0;
      ones_ = 0;
      zeros_ = 0;
      if (cPack_ == 0)
      {
        ret = MPINFO_MP;
        nMp_++;
        if (nMp_ <= 2)
        {
          level = dsplibs_debug_level;
          if (level > 1)
          {
            dsplibs_debug_printf("V90MP: MP detected. Type%d,Rate%d,Trellis%d,NonLin%d,Shaping%d,CPack%d\r\n", (int)(int8_t)type_, (int)(int8_t)rate_ * 0x960,
                                 (int)(int8_t)trellis_, (int)(int8_t)nonLin_, (int)(int8_t)shaping_, 0);
            level = dsplibs_debug_level;
          }
          /* MP: the details only at level 3 and above */
          rateMaskText(text, rateMask_);
          if (level > 2)
          {
            dsplibs_debug_printf("V90MP: Rate Mask - %s\r\n", text);
            if (dsplibs_debug_level > 2)
            {
              dsplibs_debug_printf("V90MP: h1 real = %d, imag = %d\r\n", (int)h1Re_, (int)h1Im_);
              if (dsplibs_debug_level > 2)
              {
                dsplibs_debug_printf("V90MP: h2 real = %d, imag = %d\r\n", (int)h2Re_, (int)h2Im_);
                if (dsplibs_debug_level > 2)
                  dsplibs_debug_printf("V90MP: h3 real = %d, imag = %d\r\n", (int)h3Re_, (int)h3Im_);
              }
            }
          }
        }
      }
      else
      {
        ret = MPINFO_MPNOT;
        nMpNot_++;
        if (nMpNot_ <= 2)
        {
          level = dsplibs_debug_level;
          if (level > 1)
          {
            dsplibs_debug_printf("V90MP: MPnot detected. Type%d,Rate%d,Trellis%d,NonLin%d,Shaping%d,CPack%d\r\n", (int)(int8_t)type_,
                                 (int)(int8_t)rate_ * 0x960, (int)(int8_t)trellis_, (int)(int8_t)nonLin_, (int)(int8_t)shaping_, (int)(int8_t)cPack_);
            level = dsplibs_debug_level;
          }
          /* MPnot: the details already at level 2 */
          rateMaskText(text, rateMask_);
          if (level > 1)
          {
            dsplibs_debug_printf("V90MP: Rate Mask - %s\r\n", text);
            if (dsplibs_debug_level > 1)
            {
              dsplibs_debug_printf("V90MP: h1 real = %d, imag = %d\r\n", (int)h1Re_, (int)h1Im_);
              if (dsplibs_debug_level > 1)
              {
                dsplibs_debug_printf("V90MP: h2 real = %d, imag = %d\r\n", (int)h2Re_, (int)h2Im_);
                if (dsplibs_debug_level > 1)
                  dsplibs_debug_printf("V90MP: h3 real = %d, imag = %d\r\n", (int)h3Re_, (int)h3Im_);
              }
            }
          }
        }
      }
    }
    break;
  }
  default:
    break;
  }
  return ret;
}
