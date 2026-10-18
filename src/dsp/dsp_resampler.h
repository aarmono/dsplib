/**
 * @file dsp_resampler.h
 * @brief Polyphase resampler classes of the V.90 / V.92 modems: Resampler, ResamplerTimingOffset, ResamplerTiming and V90Resampler.
 *
 * Definitions are in dsp_resampler.cpp.  These are the only classes with virtual functions in the V.90 / V.92 code.  The members are laid out so that the
 * 32-bit layout is fixed (asserts generated into dsp_resampler_layout.inc; the implicit vptr is at offset 0).
 *
 * Class chain and virtual functions (declaration order = vtable order, slots counted from the vptr: 0 destructor, 1 deleting destructor, 2 reset(),
 * 3 timingCorrection(float), 4 reset(unsigned)):
 *   Resampler              ~Resampler, reset(), timingCorrection(float) (empty)
 *   ResamplerTimingOffset  overrides all three: the hook adds a constant phase offset per output sample
 *   ResamplerTiming        overrides ~ and timingCorrection (second-order timing loop); reset() stays ResamplerTimingOffset::reset; ADDS reset(unsigned)
 *   V90Resampler           overrides ~ and reset(); adds the loop gain schedule and a history of the measured timing offset
 * Objects are created with `sysdep_malloc(size)` plus a constructor (placement new) and destroyed with the destructor plus `sysdep_free` in the owner, or
 * through the deleting destructor (slot 1), whose `operator delete` is sysdep_free.
 *
 * Numerics as in dsp_float_filters.cpp: float32 / double in memory, `long double` for intermediate values, one explicit rounding at every store, float
 * words (history, pending samples, loop gains) moved as 32-bit integer words.  The three float constructor arguments (step, cutoff, ppm) are forwarded
 * from a derived constructor to its base as raw 32-bit words through the protected `Words` constructors (a float forwarded as a float is loaded and stored
 * by the x87 at -O0 and would quiet a signalling NaN).
 */
#ifndef DSP_CLASSES_P05_H
#define DSP_CLASSES_P05_H

#include "core/dsp_cxx.h"
#include "core/dsp_math.h"
#include "dsp_window_fft_psd.h" /* LowPassFIR<float>, WindowType, meanLD / VarLD */

/** The V.90 parameter block (src/vpcm/pcm_parameters.h); V90Resampler reads the BLL gains and the timing-history settings from it (pcm_parameters.h defines the members). */
class V90Parameters;

/** The states of the V.90 timing-loop gain schedule (the names are those of the debug messages printed by V90Resampler::setBllState()). */
enum V90BllState
{
  V90BLL_FROZEN = 0,
  V90BLL_SECOND_ORDER_FROZEN = 1,
  V90BLL_INITIAL = 2,
  V90BLL_FAST = 3,
  V90BLL_MEDIUM = 4,
  V90BLL_SLOW = 5,
  V90BLL_SLOW2 = 6,
  V90BLL_DIL = 7,
  V90BLL_TRN2_INITIAL = 8,
  V90BLL_TRN2 = 9,
  V90BLL_STEADY_STATE = 10,
  V90BLL_PRE_ANSPCM = 11,
  V90BLL_TRN1_QC_INITIAL = 12,
  V90BLL_TRN1_QC_FAST = 13,
  V90BLL_TRN1_QC_MEDIUM = 14,
  V90BLL_TRN1_QC_SLOW = 15,
  V90BLL_ANY_ = 0x7fffffff /* keeps the enumeration's range wide: setBllState() tests `state <= 15` */
};

/* The derived constructors call the base constructor; keeping it a call stops the inliner from pasting the whole polyphase-table design into every derived
 * constructor. */
#define P05_NOINLINE __attribute__((noinline))

/**
 * Resampler(L, step, taps, cutoff | coef, histLen): polyphase interpolating resampler.
 *
 * `L` phases of `taps` coefficients: a table of L * taps floats, BORROWED when the constructor is given a pointer, otherwise designed from a windowed-sinc
 * (Blackman) low pass of cutoff / L and decomposed into the time-reversed polyphase rows.  A history of `histLen_` samples is written upwards (when it is
 * full, the newest taps_ samples are copied to the front and writing restarts at `taps_`).  The phase is counted in 1/L of an input sample and grows by `step`
 * per output sample.  resample() produces an output for as long as at least two input samples (counting the pending ones) are available, interpolating
 * linearly between the outputs of two adjacent phases, and calls the VIRTUAL timingCorrection(output sample) after each output (the hook may change phase_).
 * @warning resample() reads one sample beyond the block it is given (the look-ahead sample); see Resampler::resample().  The history allocation is not
 *          checked (a NULL history crashes the first resample()).
 */
class Resampler
{
public:
  P05_NOINLINE Resampler(unsigned L, float step, unsigned taps, float cutoff, unsigned histLen);
  P05_NOINLINE Resampler(unsigned L, float step, unsigned taps, float* coef, unsigned histLen);
  virtual ~Resampler();
  virtual void reset(); ///< zero the history, phase_ = 0, need_ = 0, wr_ = taps_, no pending samples
  virtual void timingCorrection(float) {}
  void resetHistoryIndex(); ///< wr_ = taps_
  void copyHistoryTail();   ///< the newest taps_ history samples to the front
  void resample(const float* in, unsigned n, float* out, unsigned& nOut);
  float getNormalizedPhase() const; ///< phase_ / L, rounded to float
  void setNormalizedPhase(float f); ///< phase_ = f * L for 0 <= f < 1, else 0

  /** The deleting destructor ends by calling sysdep_free(). */
  static inline void operator delete(void* p) __attribute__((always_inline)) { sysdep_free(p); }

protected:
  struct Words
  {
  };
  P05_NOINLINE Resampler(Words, unsigned L, uint32_t stepWord, unsigned taps, uint32_t cutoffWord, unsigned histLen);
  P05_NOINLINE Resampler(Words, unsigned L, uint32_t stepWord, unsigned taps, float* coef, unsigned histLen);

public:
  float* coef_;        ///< L rows of taps_ coefficients; owned (borrowed_ == 0) or borrowed
  float* hist_;        ///< heap, histLen_ floats
  double phase_;       ///< in 1/L input samples
  float pend_[5];      ///< pending input samples (word moves)
  unsigned pendCount_; ///< number of pending samples
  float step_;         ///< phase increment per output sample
  unsigned L_;         ///< number of phases
  unsigned taps_;      ///< taps per phase
  unsigned histLen_;   ///< history length
  unsigned wr_;        ///< write index into hist_
  unsigned need_;      ///< input samples to take before the next output
  unsigned borrowed_;  ///< 1: coef_ is not freed
};

/** Resampler whose timing hook adds a constant phase offset per output sample (step * ppm * 1e-6). */
class ResamplerTimingOffset : public Resampler
{
public:
  P05_NOINLINE ResamplerTimingOffset(unsigned L, float step, unsigned taps, float cutoff, float ppm, unsigned histLen);
  P05_NOINLINE ResamplerTimingOffset(unsigned L, float step, unsigned taps, float* coef, float ppm, unsigned histLen);
  virtual ~ResamplerTimingOffset();
  virtual void reset();                 ///< Resampler::reset(); off_ = 0
  virtual void timingCorrection(float); ///< phase_ += off_ (the argument is ignored)
  void setTimingOffset(float ppm);      ///< off_ = step * ppm * 1e-6
  /**
   * 1e6 * off_ / step_ computed in extended precision.  The float-returning getter leaves the UNROUNDED extended value in st(0) on i386 (dsp_ret_ext_float);
   * code that needs the exact value calls the LD variant.
   */
  DSP_EXTRET_FN float getTimingOffsetPPM() const;
  long double getTimingOffsetPPM_LD() const;

protected:
  P05_NOINLINE ResamplerTimingOffset(Words, unsigned L, uint32_t stepWord, unsigned taps, uint32_t cutoffWord, uint32_t ppmWord, unsigned histLen);
  P05_NOINLINE ResamplerTimingOffset(Words, unsigned L, uint32_t stepWord, unsigned taps, float* coef, uint32_t ppmWord, unsigned histLen);

public:
  float off_; ///< phase offset added per output sample
};

/**
 * Resampler with a second-order timing-recovery loop driven by the output samples (the hook of the V.90 receiver), plus a DFT bin at a quarter of the output
 * sample rate (the half-baud bin) and a band-pass gain normalisation that feed it.
 */
class ResamplerTiming : public ResamplerTimingOffset
{
public:
  P05_NOINLINE ResamplerTiming(unsigned L, float step, unsigned taps, float cutoff, float ppm, unsigned histLen);
  P05_NOINLINE ResamplerTiming(unsigned L, float step, unsigned taps, float* coef, float ppm, unsigned histLen);
  virtual ~ResamplerTiming();
  virtual void timingCorrection(float x);
  virtual void reset(unsigned); ///< clears the loop state; the argument is unused
  using ResamplerTimingOffset::reset;
  void SdHalfBaudDft(float x); ///< accumulates one bin of 256 samples
  void adjustHalfBaudBpfGain(float x);
  void resetSdHalfBaudDft();
  void invertPhase();     ///< not called anywhere in this tree
  void addPhase(float a); ///< not called anywhere in this tree

protected:
  P05_NOINLINE ResamplerTiming(Words, unsigned L, uint32_t stepWord, unsigned taps, uint32_t cutoffWord, uint32_t ppmWord, unsigned histLen);
  P05_NOINLINE ResamplerTiming(Words, unsigned L, uint32_t stepWord, unsigned taps, float* coef, uint32_t ppmWord, unsigned histLen);

public:
  float gainA_;            ///< proportional gain of the loop
  float gainB_;            ///< integral gain of the loop
  float errOut_;           ///< last timing-error estimate
  float f58_;              ///< only cleared
  float lastCorr_;         ///< last phase correction
  unsigned parity_;        ///< bit 0 selects the branch of timingCorrection (the loop updates on every second output)
  unsigned char pad64_[4]; ///< never touched
  float energy_;           ///< energy of the band-passed signal, last sample
  float energyPrev_;       ///< the one before
  float bpf_;              ///< band-pass filter output, last sample
  float bpfPrev_;          ///< band-pass filter state (the sample before the last)
  float errAcc_;           ///< negated error of the first sample of an update pair; the second sample's error is added to it
  unsigned dftN_;          ///< samples accumulated in the DFT bin
  float dftMag_;           ///< magnitude of the DFT bin, valid when dftDone_ is set
  float dftRe_;            ///< real part of the bin
  float dftIm_;            ///< imaginary part of the bin
  unsigned char dftDone_;  ///< 1 when 256 samples were accumulated
  unsigned char pad8d_[3];
  float bpfGain_; ///< gain in front of the band-pass filter (0.03981 initially)
};

/** The V.90 receiver's resampler: a gain schedule for the loop (setBllState) and a history of the measured timing offset. */
class V90Resampler : public ResamplerTiming
{
public:
  V90Resampler(unsigned L, float step, unsigned taps, float cutoff, V90Parameters* params, float ppm, unsigned histLen);
  V90Resampler(unsigned L, float step, unsigned taps, float* coef, V90Parameters* params, float ppm, unsigned histLen);
  virtual ~V90Resampler();
  virtual void reset();
  using ResamplerTiming::reset;
  void setBllState(V90BllState state, unsigned flag);
  void resample(const float* in, unsigned n, float* out, unsigned& nOut); ///< Resampler::resample plus the bookkeeping
  /** Mean / standard deviation of the timing-offset history, extended and UNROUNDED in st(0) on i386 (see getTimingOffsetPPM). */
  DSP_EXTRET_FN float getTimingHistoryMean();
  DSP_EXTRET_FN float getTimingHistoryStd();
  long double getTimingHistoryMeanLD();
  long double getTimingHistoryStdLD();

public:
  unsigned state_;        ///< current gain-schedule state (V90BllState)
  unsigned stateCount_;   ///< input samples since the last state change (counted while stateFlag_ != 0)
  unsigned stateFlag_;    ///< the flag of the last state change
  V90Parameters* params_; ///< borrowed
  float* tHist_;          ///< heap, TIMING_HISTORY_EVALUATION_BUFFER_LENGTH floats: history of getTimingOffsetPPM()
  unsigned tHistLen_;     ///< length of tHist_
  unsigned tHistIdx_;     ///< next write position in tHist_
  unsigned tCount_;       ///< input samples since the last history entry
};

typedef Resampler P05_Resampler;
typedef ResamplerTimingOffset P05_ResamplerTimingOffset;
typedef ResamplerTiming P05_ResamplerTiming;
typedef V90Resampler P05_V90Resampler;

#pragma GCC diagnostic ignored "-Winvalid-offsetof" /* offsetof of members of polymorphic classes: fine for the layout asserts */
#include "dsp_resampler_layout.inc"

#endif /* DSP_CLASSES_P05_H */
