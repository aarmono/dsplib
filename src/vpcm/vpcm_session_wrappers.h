/**
 * @file vpcm_session_wrappers.h
 * @brief Declarations of the small V.34 session wrappers of vpcm_session_wrappers.cpp and of the diagnostics record V03_DiagRecord.
 *
 * The eight functions are V34GiveINFO1dBits, indicateJaTransmission, VPcmV34GetCurrentRxBitRate, VPcmV34InitMOH, VPcmV34SetMinMaxBitRates,
 * VPcmV34SetDelays, VPcmV34GetVisualDiagnostics and VPcmV34GetDiagnostics.  Only the first three have callers in the library (the V.34 handshake and
 * dp_vpcm.cpp); the other five are exported but nothing in the library calls them.  They take the V.34 module object tagV34Object (v34_object.h),
 * reach the V.90 / V.92 / K56Flex engines through its ctx_ / k56_ members, and are the reason this group is C++.
 *
 * V03_DiagRecord is the caller-owned record that VPcmV34GetDiagnostics() fills; its first part is the TAG_DiagnosticResults that
 * V90Demodulator::getAT_UD() writes, and only the words the wrapper stores are named.
 */
#ifndef VPCM_SESSION_WRAPPERS_H
#define VPCM_SESSION_WRAPPERS_H

#include "v34/v34_object.h"
#include "pcm_bits_to_symbol.h" /* V92Transmitter / V92BitsToSymbol (the V.92 transmit bit count) */

/**
 * @brief Diagnostics record filled by VPcmV34GetDiagnostics(); sizeof 0x22c on i386.
 *
 * The member names carry the i386 byte offset of the word (asserted in vpcm_session_wrappers_layout.inc).  The comments say where each value comes
 * from in the V.34 object; "(short)" marks a value sign-extended from 16 bits.
 */
struct V03_DiagRecord
{
  uint8_t unk0000_[108]; ///< not written by VPcmV34GetDiagnostics (V90Demodulator::getAT_UD() fills part of it)
  float snrDb_;          ///< SNR estimate in dB: -12.0f - (float) w25dc_ (the constant -12.0f in V.92 mode)
  float qcA_;            ///< number of 6 dB / 1 dB steps of the error-ratio estimate (plain V.34 mode only)
  float qcB_;            ///< same value as qcA_
  uint8_t unk0078_[8];   ///< not written
  int32_t dly80_;        ///< (short) bulkDelay_
  int32_t dly84_;        ///< (short) bulkDelay_
  uint8_t unk0088_[40];  ///< not written
  int32_t baud_;         ///< symbol rate: (short) baud_ (8000 = 0x1f40 in V.92 mode)
  int32_t wb4_;          ///< (short) rxBaud_
  int32_t carrier_;      ///< (short) carrier_ (0 in V.92 mode)
  int32_t wbc_;          ///< (short) rxCarrier_
  int32_t wc0_;          ///< dgC04_
  int32_t wc4_;          ///< dgC08_
  uint8_t unk00c8_[32];  ///< not written
  int32_t we8_;          ///< (short) dgC12_
  int32_t wec_;          ///< (short) dgC0e_ + (short) dgC10_
  int32_t wf0_;          ///< (short) dgC12_
  int32_t wf4_;          ///< (short) dgC14_
  int32_t txRate_;       ///< (short) txRateIdx_ * 2400 (V.92 mode: 0, or a rate derived from the transmitter's bit count)
  int32_t rxRate_;       ///< (short) rxRateIdx_ * 2400 (plain V.34 mode only)
  int32_t w100_;         ///< (short) dgC12_
  uint8_t unk0104_[292]; ///< not written
  int32_t w228_;         ///< (short) dgC0c_, or -1 when it is 0; also the function's return value
};

extern "C"
{
  /**
   * @brief Process the INFO1d bits received in V.34 phase 2: decide whether the upstream is PCM.
   * @param obj  V.34 module object
   * @param info the received INFO1d words (10 x 16 bits); bit 5 of word 7 requests PCM upstream
   * @return 1 when it started a retrain to V.34 upstream (V.92Lite), else 0
   */
  int V34GiveINFO1dBits(tagV34Object* obj, uint16_t* info);

  /** @brief The V.34 handshake started to send Ja: let the V.90 / V.92 (or K56Flex) engine enter phase 3.  @param obj V.34 module object */
  void indicateJaTransmission(tagV34Object* obj);

  /** @brief Current receive bit rate: from the V.90 demodulator, the K56Flex handle or the V.34 rate index (x 2400).  @param obj V.34 module object */
  int VPcmV34GetCurrentRxBitRate(tagV34Object* obj);

  /**
   * @brief Start a V.34 session in the MOH (modem-on-hold) state.
   * @param obj V.34 module object
   * @param a1  MOH echo-cancel word; 1 selects the special bypass (MOHreq instead of MOHFRR)
   * @param a2  MOH flag byte
   * @param a3  MOH flag byte (cleared by the bypass)
   */
  void VPcmV34InitMOH(tagV34Object* obj, unsigned a1, unsigned char a2, unsigned char a3);

  /**
   * @brief Visual diagnostics.
   * @param obj V.34 module object
   * @param sel 0 constellation, 1 linear equalizer, 2 DFE, 3 resampler phase, 4 resampler offset, 5 echo canceller, 6 PCM samples, 7 none, 8 decision
   *            errors; larger values give 0
   * @param out receives the points
   * @param n   capacity of `out`
   * @return number of int_complex written (or the engine getter's result)
   */
  unsigned VPcmV34GetVisualDiagnostics(tagV34Object* obj, unsigned sel, int_complex* out, unsigned n);

  /**
   * @brief Fill a diagnostics record from the current session.
   * @param obj V.34 module object
   * @param out record to fill
   * @return the value stored in out->w228_
   */
  int VPcmV34GetDiagnostics(tagV34Object* obj, V03_DiagRecord* out);
}

/* These two have C++ linkage. */

/** @brief Apply the parameter block's minimum / maximum bit rates to the active engine (V.90 constellation designer, K56Flex or the V.34 rate indices). */
void VPcmV34SetMinMaxBitRates(tagV34Object* obj);

/** @brief Derive the V.34 filter delay and DMA delay from the parameter block and set the echo canceller's echo delay. */
void VPcmV34SetDelays(tagV34Object* obj);

#pragma GCC diagnostic ignored "-Winvalid-offsetof"
#include "vpcm_session_wrappers_layout.inc"

#endif /* VPCM_SESSION_WRAPPERS_H */
