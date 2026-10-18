/**
 * @file v90_diagnostics.h
 * @brief TAG_DiagnosticResults: the diagnostics record that V90Demodulator::getAT_UD() fills.
 *
 * The record is owned by the caller (the diagnostics query in vpcm_session_wrappers.cpp hands in a buffer of at least 0x224 bytes).  Only the words that
 * getAT_UD() writes are declared; the gaps are `unk*_` byte arrays.  The class V90Demodulator itself is declared in v90_demodulator.h.  The 32-bit layout
 * is checked by v90_diagnostics_layout.inc (generated).
 */
#ifndef DSP_CLASSES_P44_H
#define DSP_CLASSES_P44_H

#include "core/dsp_cxx.h"

/** Diagnostics record (0x224 bytes in the 32-bit layout); the named fields are filled by V90Demodulator::getAT_UD(). */
struct TAG_DiagnosticResults
{
  unsigned char unk00_[104];  ///< +0x00 not written
  float powerDb_;             ///< +0x68 received power in dB: 10 * log10(AGC power)
  unsigned char unk6c_[4];    ///< +0x6c not written
  float meanErrDb_;           ///< +0x70 mean error energy in dB: 10 * log10(mean error energy)
  unsigned meanErrWord_;      ///< +0x74 raw bit pattern of the mean error energy (float)
  unsigned char unk78_[12];   ///< +0x78 not written
  unsigned rtdUnits_;         ///< +0x84 round-trip delay: (10 * rtd) / 96
  unsigned char unk88_[44];   ///< +0x88 not written
  unsigned symbolRate_;       ///< +0xb4 symbol rate, always 8000
  unsigned char unkb8_[4];    ///< +0xb8 not written
  unsigned zeroBc_;           ///< +0xbc always 0
  unsigned char unkc0_[44];   ///< +0xc0 not written
  unsigned cnt23_;            ///< +0xec copy of V90Demodulator::cnt23_
  unsigned cnt1f_;            ///< +0xf0 copy of V90Demodulator::cnt1f_
  unsigned cnt26_;            ///< +0xf4 copy of V90Demodulator::cnt26_
  unsigned char unkf8_[4];    ///< +0xf8 not written
  unsigned bitRate_;          ///< +0xfc data rate in bit/s
  unsigned char unk100_[288]; ///< +0x100 not written
  unsigned rbs_;              ///< +0x220 the six per-phase RBS-suspicion flags packed into bits 0..5 (phase 0 in bit 0)
};

typedef TAG_DiagnosticResults P44_TAG_DiagnosticResults;

#include "v90_diagnostics_layout.inc"

#endif /* DSP_CLASSES_P44_H */
