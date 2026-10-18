/**
 * @file v92_cp.h
 * @brief V92CP, the V.92 CP / SUV frame holder with its bit-serial codec, and the parameter blocks that are packed into and unpacked from CP frames.
 *
 * Declares class V92CP and the structures that move through it: V90MappingParams (the "mapping parameters" block of the V.90 / V.92 phase 3 negotiation),
 * tagV90AdditionalCPinfo, V92MappingParams and CPUnPck, plus the free functions that convert between them:
 * - defined in v92_cp.cpp: setV92CPpckFromParamsInfo() (mapping parameters -> V92CP; called by VPcmFloModem) and V92setParamsInfoFromCPUnPck() (a received
 *   V90CP -> V92MappingParams; called by VPcmFloModem), and the reverse directions setParamsInfoFromV92CPUnPck() and setParamsInfoFromCPUnPck() (nothing
 *   in the library calls these two);
 * - defined in pcm_cp_packers.cpp: V90CPPacker(), the two float2Bits() overloads, displaySpectralParams(), get/setDataBitRate(), getConstellationsIndex()
 *   and get/set(Codec)ConstellationMask().
 * The generated layout assertions are in v92_cp_layout.inc, included at the end of this header.
 *
 * V92CP is a plain class (no virtual functions) and the V.92 counterpart of V90CP (v90_cp.h).  One object is a frame AND its codec.  The frame is an SUV
 * (short frame, `type_` == 1) or a long frame with a header, an optional float part (shaper coefficients, TRN1d RMS) and up to two arrays of
 * constellation code-point masks.  The codec is a byte-per-bit buffer (`bits_`, 2000 bits), a 16-stage CRC register and a receive state machine.
 * Objects are allocated by pcm_modem.cpp (sysdep_malloc(sizeof(V92CP)) and placement new); the owners store type_, mode_, flag_, suvVal_, cptSent_ and
 * mult_ directly.
 *
 * V90MappingParams is the mapping-parameters block (0x650 bytes in the 32-bit layout): a data-rate code, six code-point lists in two variants, their
 * counts, the spectral-shaper words and the indices of the distinct lists.  A few readers (V90ConstellationPower) still reach into it by byte offset.
 */
#ifndef V92_CP_H
#define V92_CP_H

#include "core/dsp_cxx.h"
#include "v90_cp.h" /* V90CP: read by V92setParamsInfoFromCPUnPck */

/**
 * V.92 CP / SUV frame holder and bit-serial codec.
 *
 * Wire layout of `bits_`: positions 0 .. 0x10 are the flag (17 ones), 0x11 a zero, 0x12 the type bit; the rest depends on the frame (see infoToBits()).
 * Multi-bit fields go out LSB first, and every 17th bit of the array parts is a 0 start bit.  The frame ends with a start bit, a 16-bit CRC, a final
 * start bit and zero padding up to the next multiple ABOVE of 12 * mult_ bits.
 */
class V92CP
{
public:
  uint8_t type_;       ///< 1 = SUV (short frame), else a long frame (set by the owners; the receive machine stores the type bit here)
  uint8_t mode_;       ///< two frame bits; the rate offset is 0x14 if non-zero else 8; a signed value > 1 has no float part
  uint8_t rate_;       ///< data-rate field (signed byte): V90MappingParams::kBits_ - (mode_ ? 0x14 : 8)
  uint8_t b03_;        ///< a byte sent whole (not masked) in the float part
  uint8_t flag_;       ///< "CPt" flag (owners clear it before infoToBits(); a received frame reports CP / SUV when it is 0, CPt / SUVt otherwise)
  uint8_t unk05_[3];   ///< padding, never touched
  uint32_t sr_;        ///< two bits, copy of V90MappingParams::sr_ (setV92CPpckFromParamsInfo() / setParamsInfoFromV92CPUnPck() move it)
  uint32_t id_;        ///< two bits, copy of V90MappingParams::id_
  float trn1dRms_;     ///< TRN1d RMS, a 16-bit unsigned Q3.13 value on the wire (greedy against fltTable_2); tagV90AdditionalCPinfo::trn1dRms_
  float shaperA1_;     ///< 7 bits plus sign on the wire (greedy against fltTable_1); V90MappingParams::shaperA1_
  float shaperA2_;     ///< V90MappingParams::shaperA2_
  float shaperB1_;     ///< V90MappingParams::shaperB1_
  float shaperB2_;     ///< V90MappingParams::shaperB2_
  uint8_t flagB_;      ///< the second array region (arrB_) is present
  uint8_t unk25_[3];   ///< padding, never touched
  uint32_t grp_[6];    ///< group number of each of the six lists (4 bits each on the wire)
  uint8_t unk40_[2];   ///< padding, never touched
  uint16_t arrA_[48];  ///< first array region: 6 groups of 8 words (bit `pos & 15` of word `pos >> 4` of a group = one list element)
  uint16_t arrB_[48];  ///< second array region
  uint8_t unk102_[2];  ///< padding, never touched
  uint32_t suv_;       ///< SUV value (setSUV() stores 0x10; 5 bits on the wire)
  uint32_t suvVal_;    ///< SUV byte
  uint16_t nGrp_;      ///< number of distinct groups
  uint8_t unk10e_[2];  ///< padding, never touched
  uint32_t cptSent_;   ///< set to 1 by infoToBits() when flag_ != 0 (cleared by V92Phase4Modulator)
  uint32_t state_;     ///< receive state (bitsToInfo(): 11 states; evaluateInfo() acts on states 3 .. 8)
  uint8_t modeCopy_;   ///< mode_ as last seen by infoToBits() / evaluateInfo()
  uint8_t ones_;       ///< receive machine: current run of one bits
  uint8_t zeros_;      ///< receive machine: current run of zero bits
  uint8_t unk11b_[1];  ///< padding, never touched
  uint32_t widx_;      ///< write index into bits_ (0x12 = idle, i.e. just after the type-bit position)
  uint32_t count_;     ///< receive machine: bits collected in the current part
  uint32_t cursor_;    ///< read cursor of evaluateInfo(); left in a state the next call depends on
  uint8_t mult_;       ///< the frame length is padded to a multiple of 12 * mult_ (owners store it; the constructor does not)
  uint8_t bits_[2000]; ///< byte-per-bit frame buffer
  uint8_t crc_[16];    ///< CRC shift register
  uint8_t unk909_[3];  ///< padding, never touched
  uint32_t totalLen_;  ///< padded frame length in bits (infoToBits(); getBitVector() reports it)
  uint32_t frameLen_;  ///< frame length up to and including the CRC (the CRC field is the last 16 bits)
  int32_t supp_;       ///< suppression counter of bitsToInfo() (-1 idle)

  /** @brief Put the receive machine in its idle state and clear flag_; the other frame fields (type_, mode_, mult_ ...) are left to the owner. */
  V92CP();
  ~V92CP();
  /** @brief Preset the CRC register to all ones. */
  void resetCRC();
  /** @brief Run the CRC over bits_[0 .. frameLen_ - 0x11) into crc_. */
  void calcCRC();
  /** @brief Compute the CRC over a received frame and compare it with the CRC field at the end of frameLen_.  @return non-zero when they match. */
  int evaluateCRC();
  /** @brief Return the receive machine to idle (widx_, count_, state_, ones_, zeros_); supp_ is left alone. */
  void resetDetector();
  /** @brief resetDetector() and also end the suppression window (supp_ = -1). */
  void reset();
  /** @brief Make the frame an SUV value: suv_ = 0x10, suvVal_ = @p v. */
  void setSUV(unsigned int v);
  /**
   * @brief Give access to the transmit bit buffer.
   * @param length  receives the padded frame length in bits (totalLen_)
   * @return bits_, one bit per byte
   */
  unsigned char* getBitVector(unsigned int& length);
  /** @brief Decode the part of the frame that the receive machine has just completed (acts on state_ 3 .. 8). */
  void evaluateInfo();
  /** @brief Encode the frame fields into bits_ with start bits, CRC and padding; sets widx_, frameLen_, totalLen_, and cptSent_ when flag_ != 0. */
  void infoToBits();
  /**
   * @brief Feed one received bit to the receive machine.
   * @param bit  the bit (0 or 1)
   * @return 0 nothing, 1 / 2 long frame (flag_ 0 / 1), 3 / 4 SUV frame (flag_ 0 / 1), 5 "Ed" (CpInfoCode); codes 3 and 4 are reported as 0 during the
   *         400-call suppression window that follows a code 1 or 2
   */
  int bitsToInfo(unsigned char bit);
};

typedef V92CP P11_V92CP;

/** The mapping-parameters block of the V.90 / V.92 negotiation. */
struct V90MappingParams
{
  uint32_t kBits_;        ///< data-rate code (the V.90 K printed as "K = %d" by V90BitsToSymbol; the bit rate is kBits_ * 8000 / 6)
  uint8_t listA_[6][128]; ///< code-point lists (element = 16 * word + bit position), one row per group
  uint8_t listB_[6][128]; ///< second variant of the lists
  uint32_t cnt_[6];       ///< element count of group g (shared by the two lists)
  uint32_t flag_;         ///< the second list variant is distinct from the first (copied to V92CP::flagB_)
  uint32_t sr_;           ///< printed as "V90MappingParams: Sr = %d"; set from the spectralShaperSr parameter
  uint32_t id_;           ///< printed as "V90MappingParams: Id = %d"
  float shaperA1_;        ///< Q-format field (float2Bits); copy of the parameter spectralShaperA1 / germanPbxSpectralShaperA1
  float shaperA2_;        ///< copy of the parameter spectralShaperA2 / germanPbxSpectralShaperA2
  float shaperB1_;        ///< copy of the parameter spectralShaperB1 / germanPbxSpectralShaperB1
  float shaperB2_;        ///< copy of the parameter spectralShaperB2 / germanPbxSpectralShaperB2
  uint32_t idx_[6];       ///< representative group of each distinct list
};
typedef V90MappingParams P11_V90MappingParams;

/** The additional CP information (six words, the modem's addCp_ member) that V90CPPacker() and setV92CPpckFromParamsInfo() read. */
struct tagV90AdditionalCPinfo
{
  uint32_t flag_;   ///< copied to V92CP::flag_ (the CPt marker; the debug header prints a quote for it)
  uint32_t mode_;   ///< copied to V92CP::mode_ (the debug header prints "CP" / "CPt" from it)
  float trn1dRms_;  ///< copied to V92CP::trn1dRms_ (Q3.13); V90Demodulator::exitPhase3() stores the TRN1d RMS it prints here
  uint32_t wc_;     ///< copied to V92CP::b03_
  uint32_t suvVal_; ///< copied to V92CP::suvVal_ (the debug header prints the "s" marker for it)
  uint32_t w14_;    ///< read as a signed 16-bit value by V90CPPacker()
};
typedef tagV90AdditionalCPinfo P11_V90AdditionalCPinfo;

/** The V.92 modulator's parameter block, filled by V92setParamsInfoFromCPUnPck() from a received V90CP. */
struct V92MappingParams
{
  int32_t w0_;       ///< 0x22 + 2 * (int8_t)V90CP::drn_
  uint32_t a_;       ///< V90CP::flagA_ (modulus-encoder parameters m_ present)
  uint32_t b_;       ///< V90CP::flagB_ (prefilter / precoder arrays present)
  uint32_t c_;       ///< V90CP::flagC_ (constellations present)
  int32_t trellis_;  ///< (int8_t)V90CP::trellisState_
  int32_t extendEu_; ///< (int8_t)V90CP::extendEu_
  float gain_;       ///< constellation gain: V90CP::prefilterGain_ * 2^-18 * 4000
  uint32_t m_[12];   ///< V90CP::pair_
  uint32_t cnt_[4];  ///< array lengths (clamped to 0x148)
  float* arr_[4];    ///< OWNED float arrays (0x600 bytes each, V92createFilterCoefficients() / V92deleteFilterCoefficients()); filled from V90CP::arrs_, scaled
  uint32_t lc_[6];   ///< constellation lengths (clamped to 0x80)
  uint32_t* lcp_[6]; ///< OWNED word arrays (0x200 bytes each, V92createConstellations() / V92deleteConstellations()); copies of V90CP::heap_
  uint32_t idxc_[6]; ///< V90CP::nib_
};
typedef V92MappingParams P11_V92MappingParams;

/** An unpacked CP structure of a layout other than V92CP; read by setParamsInfoFromCPUnPck() (which nothing calls). */
struct CPUnPck
{
  uint8_t unk00_[20];   ///< untouched
  uint32_t rate_;       ///< data-rate field
  uint32_t w_[6];       ///< six words: sr, id and the four shaper floats (copied to V90MappingParams)
  uint8_t flag_;        ///< the second array region is present
  uint8_t grp_[6];      ///< group numbers
  uint8_t unk37_[3];    ///< untouched
  uint16_t arrA_[48];   ///< first array region
  uint8_t unk9a_[2];    ///< untouched
  uint16_t arrB_[48];   ///< second array region
  const uint32_t* sub_; ///< points to a structure whose second word selects the rate offset (non-zero: 0x14, else 8)
};
typedef CPUnPck P11_CPUnPck;

extern "C"
{
  /* Bit-weight tables of the float fields of the CP frame: binaryTable are powers of two, fltTable_1 the 7 magnitude bits of a shaper coefficient and
   * fltTable_2 the 16 bits of the TRN1d RMS.  fltTable1 / fltTable2 are the same values, defined in pcm_cp_packers.cpp for float2Bits(). */
  extern uint32_t binaryTable[16];
  extern float fltTable_1[7];
  extern float fltTable_2[16];
  extern float fltTable1[7];
  extern float fltTable2[16];

  /* defined in v92_cp.cpp */

  /** @brief Pack mapping parameters and the additional CP info into a V92CP: dedupe the six lists into groups and build the two mask arrays. */
  void setV92CPpckFromParamsInfo(V90MappingParams* mp, tagV90AdditionalCPinfo* in, V92CP* cp);
  /** @brief Unpack a V92CP into mapping parameters (not called by the library). */
  void setParamsInfoFromV92CPUnPck(V90MappingParams* mp, V92CP* cp);
  /** @brief Unpack a CPUnPck structure into mapping parameters (not called by the library). */
  void setParamsInfoFromCPUnPck(V90MappingParams* mp, CPUnPck* u);
  /** @brief Convert a received V90CP into the V.92 modulator's parameter block, with debug dumps at debug level > 1. */
  void V92setParamsInfoFromCPUnPck(V92MappingParams* d, V90CP* cp);

  /* defined in pcm_cp_packers.cpp */
  int getConstellationsIndex(V90MappingParams* mp, unsigned* out);
  void getConstellationMask(V90MappingParams* mp, int g, unsigned short* out);
  void getCodecConstellationMask(V90MappingParams* mp, int g, unsigned short* out);
  void setConstellationMask(V90MappingParams* mp, int g, const short* in);
  void setCodecConstellationMask(V90MappingParams* mp, int g, const short* in);
  int getDataBitRate(V90MappingParams* mp, int flag);
  void setDataBitRate(V90MappingParams* mp, int flag, int rate);
  void displaySpectralParams(V90MappingParams* mp);
}
void float2Bits(float f, short* dst, int mode);
void float2Bits(float f, unsigned char* dst, int mode);
int V90CPPacker(V90MappingParams* mp, tagV90AdditionalCPinfo* in, short* out, int cleardown);

#pragma GCC diagnostic ignored "-Winvalid-offsetof"
#include "v92_cp_layout.inc"

#endif /* V92_CP_H */
