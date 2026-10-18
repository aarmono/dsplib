/**
 * @file dsp_tone_detectors.h
 * @brief Tone detectors of the V.90 / V.92 modems: GenericToneDetector and ANSamToneDetector (the 2100 Hz answer-tone detector).
 *
 * Definitions and the coefficient tables are in dsp_tone_detectors.cpp.  Plain classes: no virtual functions, no vtable, no operator delete.  The members
 * are laid out so that the 32-bit layout is fixed (asserts generated into dsp_tone_detectors_layout.inc).
 *
 * The detector filters the input with a GenericIIR<float,double> (dsp_float_filters.h; a heap object built in the constructor and destroyed in the
 * destructor), accumulates the energy of the input x and of the filtered signal y over blocks of `period` samples, smooths both mean powers over the
 * periods and decides "tone present" from the ratio of the two plus a threshold on the filtered power, with run lengths (hits needed, misses tolerated)
 * counted in periods.  Numerics as in dsp_float_filters.cpp: float32 in memory, extended precision (`long double`) for the intermediate values.
 * Users: the V.90 receivers (ANSamToneDetector).
 */
#ifndef DSP_CLASSES_P04_H
#define DSP_CLASSES_P04_H

#include "core/dsp_cxx.h"
#include "dsp_float_filters.h"

/** Band-pass filter plus power-ratio tone detector. */
class GenericToneDetector
{
public:
  /**
   * @brief Build a detector.
   * @param nDen, nNum, den, num, extra  the GenericIIR: coefficient counts, borrowed coefficient arrays, history extra length
   * @param minSamples      run of tone needed, in samples; rounded UP to whole periods (`needHits_`)
   * @param maxGapSamples   run of non-tone tolerated, in samples; rounded UP to whole periods (`maxMiss_`)
   * @param thresh          minimum mean filtered power per sample
   * @param ratioTest       non-zero enables the "filtered power > input power * ratio" test
   * @param ratio           the ratio of that test
   * @param period          samples per decision period
   * @warning `period` must not be 0 (the run lengths are computed by an unsigned division by it).  The float arguments are stored as 32-bit words (a
   *          signalling NaN survives).  The sysdep_malloc() result of the IIR object is not checked.
   */
  GenericToneDetector(unsigned nDen, unsigned nNum, double* den, double* num, unsigned minSamples, unsigned maxGapSamples, float thresh, unsigned ratioTest,
                      float ratio, unsigned period, unsigned extra);
  ~GenericToneDetector();
  /** @brief Process one sample; returns the detection flag (`detected_`). */
  unsigned process(float x);
  /**
   * @brief Process a block of samples (`in` is only read); returns the detection flag.
   * @note The decision logic differs from the single-sample version: this one has a hysteresis path (see the .cpp).
   */
  unsigned process(float* in, unsigned n);
  /** @brief Clear the filter, the sums, the smoothed powers, the run counters and the detection flag. */
  void reset();

protected:
  /** Tag type selecting the constructor that takes the two float arguments as raw 32-bit words (so they are never loaded as floats). */
  struct Words
  {
  };
  GenericToneDetector(Words, unsigned nDen, unsigned nNum, double* den, double* num, unsigned minSamples, unsigned maxGapSamples, uint32_t threshWord,
                      unsigned ratioTest, uint32_t ratioWord, unsigned period, unsigned extra);

public:
  P02_GenericIIR* iir_; ///< heap object
  float thresh_;        ///< minimum mean filtered power
  float ratio_;         ///< ratio of the filtered / input power test
  float sumX2_;         ///< sum of x^2 in the current period
  float sumY2_;         ///< sum of y^2 in the current period
  float avgX2_;         ///< smoothed input power
  float avgY2_;         ///< smoothed filtered power
  unsigned needHits_;   ///< ceil(minSamples / period)
  unsigned maxMiss_;    ///< ceil(maxGapSamples / period)
  unsigned count_;      ///< samples in the current period
  unsigned period_;     ///< samples per period
  unsigned hits_;       ///< consecutive passing periods
  unsigned misses_;     ///< consecutive failing periods
  unsigned ratioTest_;  ///< non-zero: the ratio test is enabled
  unsigned detected_;   ///< detection flag
};

/**
 * Detector for the 2100 Hz answer tone: a GenericToneDetector with a fixed band-pass whose coefficients depend on the sample rate (8000 Hz: 13
 * coefficients; anything else is taken as 9600 Hz: 11 coefficients).  Adds no members.  Arguments as the base class' constructor, with `sampleRate`
 * replacing the filter arguments.
 */
class ANSamToneDetector : public GenericToneDetector
{
public:
  ANSamToneDetector(unsigned minSamples, unsigned maxGapSamples, float thresh, unsigned ratioTest, float ratio, unsigned sampleRate, unsigned period,
                    unsigned extra);
  ~ANSamToneDetector();
};

typedef GenericToneDetector P04_GenericToneDetector;
typedef ANSamToneDetector P04_ANSamToneDetector;

#include "dsp_tone_detectors_layout.inc"

#endif /* DSP_CLASSES_P04_H */
