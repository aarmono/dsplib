/**
 * @file v90_mp.h
 * @brief V90MP: the V.90 / V.92 phase-4 MP (modulation parameters) frame holder and its bit-serial codec (class declaration).
 *
 * The object is both the decoded frame and the codec that converts it to and from the bit stream:
 * - the info fields (type_ .. h3Im_) hold the content of an MP / MPnot frame: frame type, data rate, trellis, non-linear coding, shaping, the CPack flag,
 *   the 14-bit rate mask and the three complex precoder coefficients h1 .. h3;
 * - `bits_` is the frame as a byte-per-bit buffer (one byte = one bit), `crc_` a 16-byte CRC shift register (one bit per byte);
 * - `state_`, `ones_`, `zeros_`, `widx_` are the receive state machine of bitsToInfo(), `nMp_` / `nMpNot_` count the frames received.
 *
 * Transmit side: the owner fills the info fields and calls infoToBits(); the phase-4 modulator then sends the buffer returned by getBitVector().
 * Receive side: the phase-4 demodulator feeds each descrambled bit to bitsToInfo(), which returns MPINFO_MP / MPINFO_MPNOT when a frame with a good CRC
 * has been collected (and evaluateInfo() has filled the info fields) or MPINFO_ED when the end-of-data pattern is seen.  `trn2dD_` (the TRN2d
 * parameter D, set by the owner from the first word of the V90MappingParams) is the padding unit: the frame is padded to a multiple of D bits.
 *
 * The class is a plain class (no virtual functions).  All members are public: the owners (V90Modulator, V90Phase4Modulator, V90Phase4Demodulator,
 * V90Demodulator) read and write them directly, for example V90Modulator stores `cPack_`.  The 32-bit layout of the class is pinned by the generated
 * v90_mp_layout.inc.  The definitions are in v90_mp.cpp.
 */
#ifndef V90_MP_H
#define V90_MP_H

#include "core/dsp_cxx.h"

/** Result of V90MP::bitsToInfo(). */
enum MpInfoCode
{
  MPINFO_NONE = 0,  ///< no frame completed by this bit
  MPINFO_MP = 1,    ///< a complete MP frame with a good CRC and cPack_ == 0 was received
  MPINFO_MPNOT = 2, ///< a complete MPnot frame (cPack_ != 0) was received
  MPINFO_ED = 3     ///< the end-of-data pattern (2 * trn2dD_ zero bits while idle) was seen
};

/** The MP frame: decoded fields, bit buffer, CRC register and receive state machine.  See the file comment. */
class V90MP
{
public:
  /* ---- info: the decoded frame; infoToBits() packs these into bits_, evaluateInfo() fills them from bits_ ---- */
  uint8_t type_;      ///< frame type: 0 = short frame (0x55 bits, no precoder coefficients), else long frame (0xbb bits)
  uint8_t rate_;      ///< data rate code (4 bits); the rate in bit/s is rate_ * 2400
  uint8_t trellis_;   ///< trellis code selector 0 .. 3
  uint8_t nonLin_;    ///< non-linear coding flag (a whole byte, not masked)
  uint8_t shaping_;   ///< constellation shaping flag (a whole byte, not masked)
  uint8_t cPack_;     ///< CPack flag: 0 in a received MP frame, non-zero in an MPnot frame
  uint16_t rateMask_; ///< 14-bit mask of the data rates supported, bit k = rate-mask bit k of the frame
  int16_t h1Re_;      ///< precoder coefficient h1, real part (long frames only)
  int16_t h1Im_;      ///< h1, imaginary part
  int16_t h2Re_;      ///< h2, real part
  int16_t h2Im_;      ///< h2, imaginary part
  int16_t h3Re_;      ///< h3, real part
  int16_t h3Im_;      ///< h3, imaginary part
  /* ---- receive state machine ---- */
  uint32_t state_;     ///< bitsToInfo() state: 0 hunt for 17 ones, 1 wait for the 0 start bit, 2 frame type bit, 3 payload, 4 padding tail
  uint8_t frameType_;  ///< first payload bit of the frame: 0 = short frame (frameLen_ 0x55), else long frame (0xbb)
  uint8_t ones_;       ///< run length of 1 bits seen (state 0 looks for more than 16)
  uint8_t zeros_;      ///< run length of 0 bits seen (the end-of-data pattern is 2 * trn2dD_ of them)
  uint8_t widx_;       ///< write index into bits_ (0x12 = idle: the first payload bit is stored at index 0x12)
  uint8_t bits_[0xe6]; ///< the frame, one bit per byte: 17 ones, a 0 start bit, then the payload (index 0x12 on); see v90_mp.cpp for the field positions
  uint8_t crc_[16];    ///< CRC-16 shift register, one bit per byte (see pcm_cp_crc.h)
  uint8_t unk112_[2];  ///< padding, never accessed
  uint32_t trn2dD_;    ///< TRN2d parameter D: the frame is padded to a multiple of D bits and the end-of-data pattern is 2 * D zeros; set by the owner
  uint8_t totalLen_;   ///< frame length plus padding: frameLen_ + 1 rounded up to a multiple of trn2dD_ (kept in 8 bits)
  uint8_t frameLen_;   ///< payload end index: 0x55 for a short frame, 0xbb for a long frame
  uint8_t unk11a_[2];  ///< padding, never accessed
  uint32_t nMp_;       ///< number of MP frames (cPack_ == 0) received since reset()
  uint32_t nMpNot_;    ///< number of MPnot frames (cPack_ != 0) received since reset()

  /** @brief Construct in the idle state: widx_ = 0x12, run counters and frame counters cleared (the info fields and buffers are not cleared). */
  V90MP();
  ~V90MP();
  /** @brief Preset the CRC register to all ones. */
  void resetCRC();
  /** @brief Run the CRC register over bits_ up to the CRC position of the current frameType_ (the register must have been preset). */
  void calcCRC();
  /** @brief Preset the CRC, run it over the frame and compare with the 16 CRC bits at bits_[frameLen_ - 16].  @return 1 if they are equal, else 0. */
  int evaluateCRC();
  /** @brief Return the receive machine to idle (widx_ = 0x12, state_ = 0, run counters 0); the frame counters are kept. */
  void resetDetector();
  /** @brief resetDetector() and clear nMp_ / nMpNot_. */
  void reset();
  /**
   * @brief Give access to the frame bits.
   * @param length  receives totalLen_, the number of bits to send (frame plus padding)
   * @return bits_ (not a copy)
   */
  unsigned char* getBitVector(unsigned int& length);
  /** @brief Decode the info fields (type_ .. h3Im_) from bits_ of a complete frame. */
  void evaluateInfo();
  /** @brief Set totalLen_ = frameLen_ + 1 rounded up to a multiple of trn2dD_.  @warning trn2dD_ == 0 divides by zero. */
  void calcSequenceLength();
  /** @brief Pack the info fields into bits_ (transmit side), compute the CRC, zero the padding, and set frameType_, frameLen_ and totalLen_. */
  void infoToBits();
  /** @brief Write `value` as text of binary digits into `out`: all nbits digits if nbits != 0, else without leading zeros.  @warning `out` is not bounded. */
  void PrintBase2(char* out, unsigned long value, unsigned short nbits);
  /**
   * @brief Feed one received bit to the frame receive machine.
   * @param bit  the descrambled bit (0 or non-zero)
   * @return MpInfoCode: MPINFO_ED when the end-of-data pattern is seen, MPINFO_MP / MPINFO_MPNOT when a frame completed (info fields valid), else MPINFO_NONE
   */
  int bitsToInfo(int bit);
  /** @brief Print the MP / MPnot counts (debug level above 1). */
  void printNofRecievedMpMpNot();
};

typedef V90MP P09_V90MP;

#pragma GCC diagnostic ignored "-Winvalid-offsetof"
#include "v90_mp_layout.inc"

#endif /* V90_MP_H */
