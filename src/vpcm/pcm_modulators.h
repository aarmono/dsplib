/**
 * @file pcm_modulators.h
 * @brief V90Modulator and V92Modulator: the top-level transmit controllers of the V.90 / V.92 modems.
 *
 * V90Modem / V92Modem own one each.  Definitions: v90_modulator.cpp / v92_modulator.cpp.  Member order is the 32-bit layout (asserts generated from
 * layouts/vpcm/v90_modulator.layout into pcm_modulators_layout.inc by tools/layout_gen.py).
 *
 * Both are plain classes (no virtual functions, no base class).  A modulator is driven by the modem: progress(in, nbits, out, n) produces n output
 * samples per call, phase by phase:
 * - state_ 0 silence;
 * - 1 phase 3 (V90Phase3Modulator / V92Phase3Modulator::generateSymbol() per symbol);
 * - 2 phase 4 (V90Phase4Modulator / V92Phase4Modulator::generateSymbol());
 * - 3 data phase (the caller's bits -> Scrambler<int, uchar> -> V90BitsToSymbol / V92BitsToSymbol -> symbols).
 * The generators raise an event (phase 3: 6 / 8 = the DIL / Su is over, the next phase starts; phase 4: 7 / 9 = Phase4 terminated: data phase) that
 * progress() turns into the next state; the exit*() methods are the modem's answers to the far end's events.  The V.92 modulator additionally runs the
 * symbols through a resampler (ResamplerTimingOffset, 8000 * 5/6 symbol rate to the sample rate), an optional 36-tap FIR (v92TxPreFilter) and a
 * Queue<float> that absorbs the difference between the produced and the requested samples ("Queue is Empty/Full" sets event 1).
 *
 * The sub-objects a modulator builds in its constructor are heap blocks (sysdep_malloc() plus placement new); the destructor destroys them (when
 * non-NULL).
 */
#ifndef DSP_CLASSES_P39_H
#define DSP_CLASSES_P39_H

#include "core/dsp_cxx.h"
#include "dsp/dsp_int_templates.h" /* Scrambler<int, unsigned char>, Queue<float> */
#include "dsp/dsp_float_filters.h" /* FloatFIR */
#include "dsp/dsp_resampler.h"     /* ResamplerTimingOffset */
#include "pcm_leaf_classes.h"      /* V90Phase2Info, V92Phase2Info, PcmType */
#include "pcm_parameters.h"        /* V90Parameters, V92Parameters */
#include "v90_mp.h"                /* V90MP */
#include "v90_cp.h"                /* V90CP */
#include "v92_cp.h"                /* V90MappingParams, V92MappingParams, V92CP, tagV90AdditionalCPinfo, displaySpectralParams */
#include "pcm_dil.h"               /* tagV90DILdescriptor */
#include "pcm_jd.h"                /* V90Jd, V92Jd */
#include "pcm_phase3_modulators.h" /* V90Phase3Modulator, V92Phase3Modulator */
#include "pcm_bits_to_symbol.h"    /* V90BitsToSymbol, V92BitsToSymbol */
#include "v90_phase4_modulator.h"  /* V90Phase4Modulator */
#include "v92_phase4_modulator.h"  /* V92Phase4Modulator */

/** V.90 transmit controller (0x70 bytes); see the file comment. */
class V90Modulator
{
public:
  V90Modulator(unsigned blk, V90Phase2Info* info, V90Jd* jd, V92Jd* v92jd, tagV90DILdescriptor* dil, V90MappingParams* mp1, V90MappingParams* mp2,
               tagV90AdditionalCPinfo* addCp, V90CP* cp, V90MP* mp, V90Parameters* params, unsigned session);
  ~V90Modulator();
  void setSessionFlag(unsigned f);
  void enterPhase3();               /* unused */
  void exitJd();                    /* unused */
  void exitJdPhase();               /* unused */
  void exitRi();                    /* unused */
  void acknowledgeCPReception();    /* unused */
  void acknowledgeCPNotReception(); /* unused */
  void acknowledgeEReception();     /* unused */
  void enterDataPhase();            /* unused (inlined in progress()) */
  void enterPhase4();               /* unused */
  void exitDIL();                   /* unused */
  int initiateRRN();                /* 0 = done, -1 = refused (not in the data phase) */
  int initiateFPE();                /* unused */
  void reset();
  void progress(int* in, unsigned& nbits, float* out, unsigned n);

public:
  V90Phase2Info* info_;               ///< phase-2 info (not owned)
  V90Jd* jd_;                         ///< V.90 Jd object (not owned)
  V92Jd* v92jd_;                      ///< V.92 Jd object (not owned)
  tagV90DILdescriptor* dil_;          ///< DIL descriptor (not owned)
  V90MappingParams* mp1_;             ///< mapping parameters (not owned)
  V90MappingParams* mp2_;             ///< mapping parameters (not owned)
  tagV90AdditionalCPinfo* addCp_;     ///< additional CP info (not owned)
  V90MP* mp_;                         ///< MP frame (not owned)
  V90CP* cp_;                         ///< CP frame (not owned)
  V90Parameters* params_;             ///< parameter block (not owned)
  unsigned session_;                  ///< session flag (setSessionFlag())
  unsigned state_;                    ///< phase: 0 silence, 1 phase 3, 2 phase 4, 3 data
  unsigned cnt_;                      ///< symbol counter of the current phase
  unsigned event_;                    ///< event raised by the phase generator
  V90Phase3Modulator* p3_;            ///< phase 3 modulator (heap)
  V90Phase4Modulator* p4_;            ///< phase 4 modulator (heap)
  V90BitsToSymbol* b2s_;              ///< bits-to-symbol mapper (heap)
  Scrambler<int, unsigned char> scr_; ///< data scrambler
  unsigned blk_;                      ///< block size in samples
  short* symBuf_;                     ///< symbol buffer (heap)
  unsigned char* bitBuf_;             ///< bit buffer (heap)
};

/** V.92 transmit controller (0x90 bytes); see the file comment. */
class V92Modulator
{
public:
  V92Modulator(unsigned blk, V92Phase2Info* info, V92Ja* ja, tagV90DILdescriptor* dil, V92CP* cp, V92MappingParams* mp, V92Parameters* params);
  ~V92Modulator();
  unsigned getV92TxFilterDelay() const; /* unused */
  void enterPhase3();
  void exitJa();
  void exitSilence();
  void exitSuSecond();
  void exitTRN1uSecond();
  void exitCPt();
  int initiateRRN(); /* 0 = done, -1 = refused */
  int initiateFPE();
  void enterPhase4();    /* unused */
  void enterDataPhase(); /* unused */
  void mkResampledSignal(unsigned& outCnt);
  void progress(int* in, unsigned& nbits, float* out, unsigned n);
  void reset();

public:
  unsigned blk_;                      ///< block size in samples
  unsigned half_;                     ///< half the modulator queue length (params_->modulatorQueueLength)
  unsigned cur_;                      ///< number of symbols to produce for the current block
  unsigned char codeA_;               ///< phase 4 start code (state 3 entry)
  unsigned char codeB_;               ///< phase 4 start code (exit*() entries)
  unsigned char pad0e_[2];            ///< unused padding
  V92Phase2Info* info_;               ///< phase-2 info (not owned)
  V92Ja* ja_;                         ///< Ja block (not owned)
  tagV90DILdescriptor* dil_;          ///< DIL descriptor (not owned)
  V92MappingParams* mp_;              ///< MP frame (not owned)
  V92CP* cp_;                         ///< CP frame (not owned)
  float skew_;                        ///< resampler phase correction used for split_ == 2
  float ppm_;                         ///< timing offset in ppm
  unsigned state_;                    ///< phase: 0 silence, 1 phase 3, 2 phase 4, 3 data
  unsigned cnt_;                      ///< symbol counter of the current phase
  unsigned event_;                    ///< event raised by the phase generator
  unsigned split_;                    ///< phase 3 event split of the block (0 none, 1 or 2 selects the phase correction)
  unsigned splitAt_;                  ///< symbol index at which the block is split
  V92Parameters* params_;             ///< parameter block (not owned)
  V92Phase3Modulator* p3_;            ///< phase 3 modulator (heap)
  V92Phase4Modulator* p4_;            ///< phase 4 modulator (heap)
  V92BitsToSymbol* b2s_;              ///< bits-to-symbol mapper (heap)
  ResamplerTimingOffset* rs_;         ///< resampler (heap)
  Scrambler<int, unsigned char> scr_; ///< data scrambler
  Queue<float>* q_;                   ///< sample queue (heap)
  FloatFIR* fir_;                     ///< optional pre-filter (heap)
  short* symBuf_;                     ///< symbol buffer (heap)
  float* fin_;                        ///< resampler input buffer (heap)
  float* fout_;                       ///< resampler output buffer (heap)
  unsigned char* bitBuf_;             ///< bit buffer (heap)
  float* fcarry_;                     ///< resampler output of the second part of a split block (heap)
};

typedef V90Modulator P39_V90Modulator;
typedef V92Modulator P39_V92Modulator;

#pragma GCC diagnostic ignored "-Winvalid-offsetof"
#include "pcm_modulators_layout.inc"

#if defined(__x86_64__)
/* The 64-bit sizes of the sub-objects; a test harness carries local mirrors of Queue<float>, FloatFIR and ResamplerTimingOffset. */
DSP_STATIC_ASSERT(sizeof(Queue<float>) == 40, "x86-64 sizeof Queue<float>");
DSP_STATIC_ASSERT(sizeof(FloatFIR) == 32, "x86-64 sizeof FloatFIR");
DSP_STATIC_ASSERT(sizeof(ResamplerTimingOffset) == 88, "x86-64 sizeof ResamplerTimingOffset");
#endif

#endif /* DSP_CLASSES_P39_H */
