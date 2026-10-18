/**
 * @file v90_equalizer.h
 * @brief V90Equalizer: the adaptive equalizer of the V.90 / V.92 PCM receiver (class declaration and mode enums).
 *
 * V90Equalizer is a plain class (no virtual functions, no base class, no embedded sub-object) that V90Demodulator owns through a pointer.  It equalizes the
 * PCM-level samples the resampler delivers, two samples per symbol: a linear FIR equalizer (float history, newest sample at the LOWEST index) plus a
 * decision feedback equalizer (DFE) that subtracts the intersymbol interference of the past decision errors.  Both are adapted by an LMS rule
 * (setLinearEquBeta() / setDfeBeta() set the step sizes, 0 freezes the adaptation).
 *
 * Data flow.  process() consumes n samples (plus one pending sample of the previous call), forms the soft decision of every symbol, asks the demodulator state
 * machine of the current decision mode (mode_, see V90EqMode) for the hard decision, stores it and adapts.  The events that the phase state machines
 * (V90Phase3Demodulator::event_, V90Phase4Demodulator::event_) raise steer the beta schedule, the resampler's BLL state and the mode switches; the last
 * event is left in event_ for V90Demodulator::progress().  In the data phase the equalizer switches to a 16-bit integer implementation (the "MMX" mode,
 * mmx_ != 0; the code is plain C, only the data format is called MMX): coefficients are kept as 32-bit values split into 16-bit high and low halves, the
 * history as 16-bit samples, and the LMS update is a multiply and shift.  enterPhase4(), enterDataPhase(), convertEqualizerToMmx() and
 * restoreEqualizerToFloat() convert between the two representations.
 *
 * Source files: v90_equalizer.cpp (construction, reset, beta setters, phase entries, statistics), v90_equalizer_phases.cpp (enterPhase4, enterDataPhase,
 * convertEqualizerToMmx) and v90_equalizer_process.cpp (process).  The 32-bit layout of the whole class is pinned by v90_equalizer_layout.inc (generated).
 */
#ifndef V90_EQUALIZER_H
#define V90_EQUALIZER_H

#include "core/dsp_cxx.h"
#include "v90_demod_state.h"          /* V90DemodState: the event / outcome codes of V90Demodulator::state_ and the event_ words of the phase objects */
#include "dsp/dsp_resampler.h"        /* V90Resampler */
#include "pcm_parameters.h"           /* V90Parameters */
#include "v90_connection_evaluator.h" /* V90ConnectionEvaluator */
#include "v90_spectral_shaper.h"      /* V90SpectralVerifier */
#include "v92_echo_canceller.h"       /* V90PreFilter */
#include "v90_mapper.h"               /* V90Demapper */
#include "v90_phase3_demod.h"         /* V90Phase3Demodulator */
#include "v90_phase4_demod.h"         /* V90Phase4Demodulator */

/**
 * Computational mode the demodulator was created for.  The constructor only compares it with V90_COMPMODE_1 (against the modem parameter block word at
 * +0x5c, which decides whether the 16-bit mode is allowed); the meaning of the individual values is not known.
 */
enum V90ComputationalMode
{
  V90_COMPMODE_0 = 0,
  V90_COMPMODE_1 = 1,
  V90_COMPMODE_2 = 2
};

/**
 * V90Equalizer::mode_: the decision mode of process().  The names are the ones printed by the entering functions ("enter Phase 3" ...
 * "enterChannelVerification"); 0 is the value reset() stores (plain rounding, no phase entered yet).  process() stores the decision unchanged for any
 * value above 6, and its adaptation test against 10 .. 16 (`mode - 10 > 2`, `mode != 16` ...) names values mode_ never takes, so that part of the
 * test is always true for modes 0 .. 6.
 */
enum V90EqMode
{
  V90EQ_MODE_RESET = 0,               ///< plain rounding of the soft decision
  V90EQ_MODE_PHASE3 = 1,              ///< phase 3: decisions from V90Phase3Demodulator
  V90EQ_MODE_PHASE4 = 2,              ///< phase 4 (TRN2d): decisions from V90Phase4Demodulator on the clipped soft value
  V90EQ_MODE_DATA = 3,                ///< data phase: decisions from V90Demapper, RRN / FPE detection
  V90EQ_MODE_RRN = 4,                 ///< rate renegotiation: phase-4 decisions on the 16-bit soft value
  V90EQ_MODE_FPE = 5,                 ///< final phase-4 exchange: phase-4 decisions, waits for the end of phase 4
  V90EQ_MODE_CHANNEL_VERIFICATION = 6 ///< channel verification (V.92): phase-3 decisions
};

/**
 * The equalizer object.  Members are public because V90Demodulator reads and writes a few of them directly (event_, statistics, ratio_, flag148_, w8_).
 * "Float mode" is mmx_ == 0, "16-bit mode" is mmx_ != 0.
 */
class V90Equalizer
{
public:
  /**
   * @brief Allocate the float tables (and, when the 16-bit mode is allowed, the 16-bit tables), then reset().
   * @param linLen    linear equalizer length (rounded down to a multiple of 4)
   * @param dfeLen    DFE length (rounded down to a multiple of 4)
   * @param p3        phase-3 demodulator, source of the phase-3 decisions
   * @param p4        phase-4 demodulator
   * @param dem       demapper (hard decisions in the data phase)
   * @param ce        connection evaluator, receives the smoothed error
   * @param sv        spectral verifier (its flag_ == 2 marks the German PBX case)
   * @param params    parameter block (step sizes, lengths, durations)
   * @param resampler resampler whose BLL state and timing offset the events steer
   * @param pf        pre-filter (EIA6 query)
   * @param mode      computational mode, part of the decision whether the 16-bit mode is allowed
   * @warning The sysdep_malloc() results are not checked.
   */
  V90Equalizer(unsigned linLen, unsigned dfeLen, V90Phase3Demodulator* p3, V90Phase4Demodulator* p4, V90Demapper* dem, V90ConnectionEvaluator* ce,
               V90SpectralVerifier* sv, V90Parameters* params, V90Resampler* resampler, V90PreFilter* pf, V90ComputationalMode mode);
  /** @brief Free every table allocated by the constructor; nothing is cleared. */
  ~V90Equalizer();

  /** @brief Return to float mode: both step sizes 0, identity linear equalizer (main tap at `arg`), zero DFE, cleared statistics, new fade windows. */
  void reset(unsigned arg);
  /** @brief Empty the mean-error history (mseIdx_ = mseFull_ = 0).  No caller in the library. */
  void resetMeanErrorEnergyDiagnostics();
  /** @brief Set the linear LMS step size (prints it when it changed; recomputes the 16-bit multiplier and shift in the 16-bit mode). */
  void setLinearEquBeta(float beta);
  /** @brief Set the DFE LMS step size (same behaviour as setLinearEquBeta()). */
  void setDfeBeta(float beta);
  /** @brief Freeze both equalizers (both step sizes 0).  No caller in the library. */
  void freeze();
  /** @brief Enter decision mode 1 (phase 3) once: step sizes 0, event_ cleared. */
  void enterPhase3();
  /** @brief Return the DFE step size.  No caller in the library. */
  float getDfeBeta();
  /** @brief Zero the linear coefficients (float, and the 16-bit halves in the 16-bit mode).  No caller in the library. */
  void zeroLinearEquCoefs();
  /** @brief Zero the DFE coefficients (float, and the 16-bit halves in the 16-bit mode).  No caller in the library. */
  void zeroDfeCoefs();
  /** @brief Copy n float linear coefficients from c (float table only).  No caller in the library. */
  void setLinearEquCoeff(float* c, unsigned n);
  /** @brief Copy n float DFE coefficients from c (float table only).  No caller in the library. */
  void setDfeCoeff(float* c, unsigned n);
  /** @brief Empty stub.  No caller in the library. */
  void loadCoefsFromFile();
  /** @brief Taper both ends of the linear equalizer with the fade windows (through the float form when the 16-bit mode is active). */
  void linearEquFadeEdges();
  /** @brief Empty stub.  No caller in the library. */
  void printCoefsToFile() const;
  /** @brief Convert the 16-bit mode state (coefficients, histories, history position) back to float and leave the 16-bit mode; no-op in float mode. */
  void restoreEqualizerToFloat();
  /**
   * @brief Enter decision mode 5 (FPE) once; freezes or restores the equalizer as the spectral verifier / EIA6 state requires.
   * @return 1 when the equalizer was converted from the 16-bit mode back to float, else 0
   */
  int enterFPE();
  /** @brief Enter decision mode 4 (RRN) once; same behaviour and return value as enterFPE(). */
  int enterRRN();
  /** @brief Enter decision mode 6 (channel verification) once: step sizes 0, resampler BLL state V90BLL_PRE_ANSPCM. */
  void enterChannelVerification();
  /** @brief Empty stub.  No caller in the library. */
  void printEquStuff();
  /**
   * @brief Compute and print mean, standard deviation, variance, minimum and maximum of the mean-error history.
   * @return the standard deviation, or 0 when the history is empty (statMean_, statMin_ and statMax_ are then left alone)
   */
  float calcMeanErrorStatistics();
  /** @brief Rebuild the two fade windows from edge ratios (each clamped to 0 .. 0.5 of the linear length).  No caller outside reset(). */
  void setLinearEquEdgesFadingParams(float left, float right);

  /** @brief Enter decision mode 2 (phase 4) once: freeze step sizes and timing, zero the DFE (German PBX), restart the study, print the coefficient report. */
  void enterPhase4();
  /** @brief Convert both equalizers and their histories to the 16-bit mode (a message and no change when it is not allowed or already active). */
  void convertEqualizerToMmx();
  /**
   * @brief Enter decision mode 3 (data phase) once: reset the phase-4 RRN detector and convert to the 16-bit mode.
   * @return 1 when the equalizer was in float mode on entry and is in the 16-bit mode afterwards, else 0
   */
  int enterDataPhase();

  /**
   * @brief Equalize n input samples and produce the decisions.
   * @param in     input samples from the resampler (float, two per symbol)
   * @param n      number of input samples; the pending sample of the previous call is added when pend_ is set
   * @param dec    receives one hard decision per symbol
   * @param out    receives one equalized soft value per symbol (float)
   * @param outCnt receives the symbol count, (n + pend_) / 2
   * @warning In the 16-bit mode n + 1 samples are staged in mmxIn_ (512 entries) and the soft values in mmxOut_ (256 entries); larger calls overflow them.
   *          The switch to the 16-bit mode in the middle of a call reads two floats beyond in[n - 1].
   */
  void process(float* in, unsigned n, short* dec, float* out, unsigned& outCnt);

public:
  V90Resampler* resampler_;    ///< resampler whose BLL state and timing offset the events steer
  unsigned bllSaved_;          ///< BLL state saved before the timing was frozen
  unsigned short w8_;          ///< DFE protection value stored by the demodulator on the DIL path; 0 = none, else divides the DFE high-ucode DIL beta
  unsigned char pad0a_[2];     ///< unused
  unsigned linLen_;            ///< linear equalizer length (multiple of 4)
  float linBeta_;              ///< linear LMS step size, 0 = frozen
  float* linCoef_;             ///< linear coefficients, linLen_ floats (valid in float mode)
  float* linHist_;             ///< linear sample history, linHistLen_ floats, newest at linPos_ (valid in float mode)
  unsigned linHistLen_;        ///< length of linHist_ (and of the 16-bit history)
  unsigned linPos_;            ///< index of the newest sample in linHist_
  float* fadeWinA_;            ///< window for the leading edge of the linear coefficients
  float* fadeWinB_;            ///< window for the trailing edge of the linear coefficients
  unsigned fadeLenA_;          ///< number of leading coefficients faded
  unsigned fadeLenB_;          ///< number of trailing coefficients faded
  unsigned frameCnt_;          ///< process() calls since the last linearEquFadeEdges()
  unsigned dfeLen_;            ///< DFE length (multiple of 4)
  float dfeBeta_;              ///< DFE LMS step size, 0 = frozen
  float* dfeCoef_;             ///< DFE coefficients, dfeLen_ floats (valid in float mode)
  float* dfeHist_;             ///< DFE history of past errors, dfeLen_ floats, newest at index 0 (valid in float mode)
  V90Phase3Demodulator* p3_;   ///< phase-3 demodulator
  V90Phase4Demodulator* p4_;   ///< phase-4 demodulator
  V90Demapper* dem_;           ///< demapper
  V90ConnectionEvaluator* ce_; ///< connection evaluator
  V90SpectralVerifier* sv_;    ///< spectral verifier
  V90PreFilter* pf_;           ///< pre-filter
  unsigned mode_;              ///< decision mode (V90EqMode)
  unsigned event_;             ///< last non-zero event of the phase state machine, read by V90Demodulator::progress()
  unsigned pend_;              ///< nonzero when an odd input sample is pending in pendSample_
  float pendSample_;           ///< unpaired last input sample of the previous process() call
  unsigned errCnt_;            ///< symbols accumulated in the current error block
  unsigned errLimit_;          ///< error block length in symbols
  unsigned errSum_;            ///< sum of the truncated squared errors of the block
  float rmsErr_;               ///< rms error of the last block
  float meanErr_;              ///< smoothed rms error
  float statMean_;             ///< mean of the mean-error history (calcMeanErrorStatistics())
  float statMin_;              ///< minimum of the mean-error history
  float statMax_;              ///< maximum of the mean-error history
  float errK_;                 ///< smoothing constant of meanErr_ (weight of the previous value)
  unsigned badCnt_;            ///< count of large momentary errors
  float* mseHist_;             ///< history of rmsErr_ values, 300 floats, used as a ring
  unsigned mseIdx_;            ///< next write index in mseHist_
  unsigned mseFull_;           ///< nonzero once mseHist_ has wrapped
  unsigned mseEnable_;         ///< nonzero while rmsErr_ values are appended to mseHist_
  V90Parameters* params_;      ///< parameter block
  unsigned mmxCapable_;        ///< nonzero when the 16-bit tables were allocated
  unsigned mmx_;               ///< nonzero while the 16-bit mode is active
  short* mmxIn_;               ///< staging area for the 16-bit input samples (512 shorts)
  short* mmxOut_;              ///< 16-bit soft values of the symbols of one call (256 shorts)
  float linCoefMax_;           ///< largest |coefficient| of the linear equalizer at the last report
  float linCoefMin_;           ///< smallest |coefficient| of the linear equalizer at the last report
  float linMmxGain_;           ///< scale from float to 32-bit linear coefficients (2^30 / linCoefMax_)
  int linDiv_;                 ///< divisor of the 16-bit linear sum (linMmxGain_ / 2^16)
  int linMul_;                 ///< 16-bit LMS multiplier of the linear equalizer
  int linShift_;               ///< 16-bit LMS right shift of the linear equalizer
  short* rawLinHi_;            ///< allocated block of the linear high halves (linLen_ + 8 shorts)
  short* rawLinLo_;            ///< allocated block of the linear low halves
  short* linHi_;               ///< 8-byte aligned view of rawLinHi_: high 16 bits of the 32-bit linear coefficients
  short* linLo_;               ///< 8-byte aligned view of rawLinLo_: low 16 bits
  unsigned linHiOff_;          ///< alignment offset of linHi_ in shorts
  unsigned linLoOff_;          ///< alignment offset of linLo_ in shorts
  short* rawHist16_;           ///< allocated block of the 16-bit linear history (linHistLen_ + 8 shorts)
  short* hist16_;              ///< 8-byte aligned view of rawHist16_
  unsigned hist16Off_;         ///< alignment offset of hist16_ in shorts
  unsigned histPos16_;         ///< index of the newest sample in hist16_
  float dfeCoefMax_;           ///< largest |coefficient| of the DFE at the last report
  float dfeCoefMin_;           ///< smallest |coefficient| of the DFE at the last report
  float dfeMmxGain_;           ///< scale from float to 32-bit DFE coefficients
  int dfeDiv_;                 ///< divisor of the 16-bit DFE sum
  int dfeMul_;                 ///< 16-bit LMS multiplier of the DFE
  int dfeShift_;               ///< 16-bit LMS right shift of the DFE
  short* rawDfeHi_;            ///< allocated block of the DFE high halves (dfeLen_ + 8 shorts)
  short* rawDfeLo_;            ///< allocated block of the DFE low halves
  short* dfeHi_;               ///< 8-byte aligned view of rawDfeHi_
  short* dfeLo_;               ///< 8-byte aligned view of rawDfeLo_
  unsigned dfeHiOff_;          ///< alignment offset of dfeHi_ in shorts
  unsigned dfeLoOff_;          ///< alignment offset of dfeLo_ in shorts
  short* rawDfeHist16_;        ///< allocated block of the 16-bit DFE history (dfeLen_ + 8 shorts)
  short* dfeHist16_;           ///< 8-byte aligned view of rawDfeHist16_
  unsigned dfeHist16Off_;      ///< alignment offset of dfeHist16_ in shorts
  unsigned char pad138_[4];    ///< unused
  float stat13c_;              ///< statMean_ saved when the first phase-4 study ended
  float ratio_;                ///< stat13c_ / statMean_ after the second study (0 when statMean_ is 0), read by V90Demodulator::progress()
  unsigned short flag144_;     ///< nonzero until the first study of phase 4 has been evaluated
  unsigned short flag146_;     ///< nonzero until the second study of phase 4 has been evaluated
  unsigned flag148_;           ///< retrain / quick-connect flag stored by V90Demodulator::reset(); selects the TRN1d beta schedule
  float timingOffset_;         ///< timing offset applied at TRN1d start when flag148_ is set; never stored by the library
};

typedef V90Equalizer P40_V90Equalizer;

#pragma GCC diagnostic ignored "-Winvalid-offsetof"
#include "v90_equalizer_layout.inc"

#endif /* V90_EQUALIZER_H */
