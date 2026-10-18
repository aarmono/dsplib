/**
 * @file v90_cp.h
 * @brief V90CP: the V.90 phase-4 CP / SUV frame holder and its bit-serial codec.
 *
 * One V90CP object is both the decoded frame (the info fields at the start of the object) and the codec that moves it to and from a byte-per-bit buffer of
 * 12000 bits (`bits_`), with a 16-stage CRC register (`crc_`) and a 14-state receive machine (`state_`).  Definitions: v90_cp.cpp; the generated
 * layout assertions: v90_cp_layout.inc.  The class is plain: no virtual functions, and six heap blocks (`heap_`) that the constructor allocates with
 * sysdep_malloc() and the destructor frees with sysdep_free().
 *
 * A frame is either SHORT (`type_` != 0: a type bit, 2 bytes, CRC) or LONG (`type_` == 0: a header of 5 + 2 + 8 + 3 + 8 + 16 bits and up to three
 * optional parts selected by flagA_ / flagB_ / flagC_: part A = 6 pairs of bytes, part B = 4 counts and 4 arrays of 16-bit words, part C = 6 nibbles,
 * 6 counts and 6 heap arrays of 16-bit words).  Every 17th bit of the grid is a 0 start bit.
 *
 * Users: V90Modulator sets `cptFlag_` = 0 before infoToBits(); V90Modulator::exitRi, V90Phase4Demodulator::reset / getV92Decision and
 * V90Phase4Modulator::generateV92Symbol store `trn2dD_` (the first word of the V90MappingParams, "TRN2d D"), the frame-padding unit.
 */
#ifndef DSP_CLASSES_P10_H
#define DSP_CLASSES_P10_H

#include "core/dsp_cxx.h"

/**
 * Result of V90CP::bitsToInfo() (and of V92CP::bitsToInfo()): a long frame (type 0) is a CP (`cptFlag_` == 0) or a CPt, a short frame (type 1) an SUV or an
 * SUVt; CPINFO_ED is the "Ed" (a run of zero bits).  The names follow the debug prints "CP detected" / "CPnot detected" / "Ed detected" and the
 * V92Phase4Modulator handlers (recivedCP / recivedCPtag / recivedSUV / recivedSUVtag).  The suppression-window tests in bitsToInfo()
 * (`ret - 1 > 1`, `ret - 3 <= 1`) compare the
 * numeric values.
 */
enum CpInfoCode
{
  CPINFO_NONE = 0,
  CPINFO_CP = 1,
  CPINFO_CPT = 2,
  CPINFO_SUV = 3,
  CPINFO_SUVT = 4,
  CPINFO_ED = 5
};

/** V.90 phase-4 CP frame holder and bit-serial codec (see the file comment). */
class V90CP
{
public:
  uint32_t type_;          ///< frame type: the first payload bit (0 = long frame, else short)
  uint32_t flagA_;         ///< long frame: part A present (6 pairs of bytes in pair_[])
  uint32_t flagB_;         ///< long frame: part B present (counts cnt_[] and the four 16-bit arrays arrs_)
  uint32_t flagC_;         ///< long frame: part C present (nibbles, counts hcnt_[], the six heap arrays)
  uint8_t drn_;            ///< 5-bit field "CPObj->drn" of a long frame
  uint8_t trellisState_;   ///< 2-bit field "CPObj->trellisState" of a long frame (0 .. 3; other values leave both bits unchanged in infoToBits())
  uint8_t extendEu_;       ///< one-bit field "CPObj->extendEu" of a long frame
  uint8_t cptFlag_;        ///< 0: the frame is a "CP" (or SUV), else a "CPt" (or SUVt); the V92CP counterpart is flag_
  uint32_t prefilterGain_; ///< 16-bit field "CPObj->prefilterGain" (the V.92 constellation gain is prefilterGain_ * 2^-18 * 4000)
  uint32_t pair_[12];      ///< part A: 6 pairs of bytes
  uint32_t cnt_[4];        ///< part B: four 9-bit counts
  uint16_t arrs_[1536];    ///< part B: four arrays of 384 words (array g starts at 384 * g), cnt_[g] of them used
  uint32_t hcnt_[6];       ///< part C: six 8-bit counts, one per heap array
  uint32_t nib_[6];        ///< part C: six 4-bit values
  uint32_t* heap_[6];      ///< part C: six OWNED heap arrays of 128 words (0x200 bytes each)
  uint32_t wCa0_;          ///< short frame: one byte
  uint32_t state_;         ///< receive state 0 .. 13 (jump table of bitsToInfo(); evaluateInfo() switches on it too)
  uint8_t b0ca8_;          ///< written (0) by infoToBits() of a long frame, never read
  uint8_t ones_;           ///< run counter of 1 bits
  uint8_t zeros_;          ///< run counter of 0 bits
  uint8_t pad0cab_[1];     ///< padding, never touched
  uint32_t widx_;          ///< write index into bits_ (0x12 = idle)
  uint32_t count_;         ///< bits collected in the current part
  uint32_t cursor_;        ///< read cursor of evaluateInfo(); its value after one call is what the next call starts from
  uint8_t bits_[12000];    ///< byte-per-bit frame buffer
  uint8_t crc_[16];        ///< CRC shift register, one byte per stage
  uint32_t trn2dD_;        ///< "D" (TRN2d D): frames are padded to a multiple of it; 0 makes calcSequenceLength() / infoToBits() divide by zero
  uint32_t totalLen_;      ///< frame length rounded up to a multiple of D
  uint32_t frameLen_;      ///< index of the last start bit (= payload end + 17)
  uint32_t nMp_;           ///< count of received MP frames; never incremented (printed by printNofRecievedMpMpNot())
  uint32_t nMpNot_;        ///< count of received MPNot frames; never incremented
  int32_t supp_;           ///< suppression window: -1 idle; 0 .. 799 after a return code 1 / 2, during which codes 2 and 4 are reported as 0

  /** @brief Allocate the six heap arrays and put the receive machine in its idle state (`cptFlag_` = 0, `supp_` = -1). */
  V90CP();
  /** @brief Free the six heap arrays. */
  ~V90CP();
  /** @brief Preset the CRC register to all ones. */
  void resetCRC();
  /** @brief Run the CRC over the frame bits up to the CRC field (register not preset first). */
  void calcCRC();
  /** @brief Recompute the CRC of the received frame; @return 1 if it matches the 16 bits after the payload, else 0. */
  int evaluateCRC();
  /** @brief Return the receive machine to idle (write index, counters, state) without touching `nMp_` / `supp_`. */
  void resetDetector();
  /** @brief Return the receive machine to idle and clear the MP counters and the suppression window. */
  void reset();
  /**
   * @brief Get the frame bit buffer.
   * @param length  set to `totalLen_`, the padded frame length in bits
   * @return the byte-per-bit buffer `bits_`
   */
  unsigned char* getBitVector(unsigned int& length);
  /** @brief Decode the part of the frame that the receive state just completed from `bits_` into the info fields. */
  void evaluateInfo();
  /** @brief Set `totalLen_` to `frameLen_ + 1` rounded up to a multiple of `trn2dD_`. */
  void calcSequenceLength();
  /** @brief Encode the info fields into `bits_` (start bits, payload, CRC, zero padding) and set `widx_`, `frameLen_`, `totalLen_`. */
  void infoToBits();
  /**
   * @brief Feed one received bit to the receive machine.
   * @param bit  the bit (0 or non-zero)
   * @return CpInfoCode: 0 nothing, 1 / 2 long frame (`cptFlag_` == 0 / != 0), 3 / 4 short frame (`cptFlag_` == 0 / != 0), 5 "Ed"; codes 2 and 4 are
   *         reported as 0 while the suppression window `supp_` counts
   */
  int bitsToInfo(unsigned char bit);
  /** @brief Print the MP / MPNot counters when the debug level exceeds 1. */
  void printNofRecievedMpMpNot();
};

typedef V90CP P10_V90CP;

#pragma GCC diagnostic ignored "-Winvalid-offsetof"
#include "v90_cp_layout.inc"

#endif /* DSP_CLASSES_P10_H */
