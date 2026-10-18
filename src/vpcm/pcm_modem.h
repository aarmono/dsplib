/**
 * @file pcm_modem.h
 * @brief V90Modem and V92Modem: the owner of one side of the PCM modem.
 *
 * Each builds the modulator (V.90 side 0 "Digital", V.92 side 1 "Analog") or the demodulator (V.90 side 1 "Analog"), the V90Parameters / V92Parameters, the
 * phase-2 info, the Jd objects (V.90) or the CP / mapping parameters (V.92), forwards setSessionFlag() / reset() / progress() to the active side and prints
 * the version banner.  VPcmFloModem embeds a V90Modem at +0x1758 and a V92Modem at +0x6124.  Definitions: pcm_modem.cpp.  Layout asserts:
 * pcm_modem_layout.inc (generated).
 */
#ifndef DSP_CLASSES_P45_H
#define DSP_CLASSES_P45_H

#include <limits.h>

#include "core/dsp_cxx.h"
#include "pcm_modulators.h"  /* V90Modulator, V92Modulator, V92Ja and what they include (MP / CP, mapping params, Jd, Parameters, Phase2Info) */
#include "v90_demodulator.h" /* V90Demodulator, V90ComputationalMode, __tHardwareCodecTypes__ */

/* The methods compare the side for equality with 0 / 1 and, in the destructors, unsigned > 1, so the enums must accept every 32-bit value (the enumerator
 * UINT_MAX widens the range). */

/** Which end of the V.90 link the modem plays: the digital (modulating) or analog (demodulating) side. */
enum V90ModemSide
{
  V90_SIDE_DIGITAL = 0,
  V90_SIDE_ANALOG = 1,
  V90_SIDE_ANY_ = UINT_MAX
};
/** Which end of the V.92 link the modem plays; only the analog side builds a modulator. */
enum V92ModemSide
{
  V92_SIDE_NONE = 0,
  V92_SIDE_ANALOG = 1,
  V92_SIDE_ANY_ = UINT_MAX
};
/** V.92 computational mode; only handed to the constructor, which ignores it. */
enum V92ComputationalMode
{
  V92_COMPMODE_0 = 0,
  V92_COMPMODE_1 = 1,
  V92_COMPMODE_ANY_ = UINT_MAX
};

/** One side of the V.90 modem (0x49c0 bytes). */
class V90Modem
{
public:
  V90Modem(V90ModemSide side, _tagModemParameters* params, tagV90DILdescriptor* dil, unsigned blk, V90ComputationalMode mode, unsigned session);
  ~V90Modem();
  void printTitle();
  void reset(unsigned qc); /* qc: the quick-connect flag (the demodulator's reset argument) */
  void setSessionFlag(unsigned f);
  void progress(int* bits, unsigned& nBits, float* samples,
                unsigned n); /* side 0: V90Modulator::progress(in bits, nbits, out samples, n); side 1: V90Demodulator::progress(out, cnt, in, n) */

public:
  V90Modulator* mod_;            ///< +0x0000 modulator (side 0), else NULL
  V90Demodulator* dem_;          ///< +0x0004 demodulator (side 1), else NULL
  V90Phase2Info* info_;          ///< +0x0008 phase-2 info (heap)
  V90Jd* jd_;                    ///< +0x000c V.90 Jd object (heap)
  V92Jd* jd92_;                  ///< +0x0010 V.92 Jd object (heap)
  tagV90DILdescriptor* dil_;     ///< +0x0014 DIL descriptor (not owned)
  V90MappingParams mp1_;         ///< +0x0018 0x650 B, mapping parameters shared with the active side
  V90MappingParams mp2_;         ///< +0x0668
  tagV90AdditionalCPinfo addCp_; ///< +0x0cb8 0x18 B
  V90MP mp_;                     ///< +0x0cd0 0x124 B
  V90CP cp_;                     ///< +0x0df4 0x3bc0 B
  V90Parameters* params_;        ///< +0x49b4 parameter block (heap)
  unsigned session_;             ///< +0x49b8 session flag
  unsigned side_;                ///< +0x49bc side (V90ModemSide)
};

/** The V92Ja block of the V92Modem (+0xc): a length word and the bits V92DILdescriptorPacker() writes (the V92Ja type declares only the first byte of them). */
struct V92JaBlock
{
  uint32_t len_;
  uint8_t bits_[0xa8c];
};

/** One side of the V.92 modem (0xaac bytes). */
class V92Modem
{
public:
  V92Modem(V92ModemSide side, _tagModemParameters* params, unsigned blk, tagV90DILdescriptor* dil, V92ComputationalMode mode);
  ~V92Modem();
  void printTitle();
  void reset();
  void progress(int* bits, unsigned& nBits, float* samples, unsigned n);

public:
  V92Modulator* mod_;         ///< +0x0000 modulator (side 1), else NULL
  V92Parameters* params_;     ///< +0x0004 parameter block (heap)
  V92Phase2Info* info_;       ///< +0x0008 phase-2 info (heap)
  V92JaBlock ja_;             ///< +0x000c 0xa90 B, the DIL frame the V.92 modulator sends
  tagV90DILdescriptor* dil_;  ///< +0x0a9c DIL descriptor (not owned)
  V92MappingParams* mapping_; ///< +0x0aa0 mapping parameters (heap)
  V92CP* cp_;                 ///< +0x0aa4 call-progress object (heap)
  unsigned side_;             ///< +0x0aa8 side (V92ModemSide)
};

typedef V90Modem P45_V90Modem;
typedef V92Modem P45_V92Modem;

#pragma GCC diagnostic ignored "-Winvalid-offsetof"
#include "pcm_modem_layout.inc"

#endif /* DSP_CLASSES_P45_H */
