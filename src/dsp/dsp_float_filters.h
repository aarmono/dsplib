/**
 * @file dsp_float_filters.h
 * @brief Float block filters of the V.90 / V.92 modems: FloatFIR, FloatIIR, FloatARMA and GenericIIR<float, double>.
 *
 * Definitions and the explicit instantiations are in dsp_float_filters.cpp.  The classes are used by the V.90 / V.92 modulator, demodulator and echo
 * canceller (src/vpcm) and by the tone detectors of dsp_tone_detectors.h.  The members are laid out so that the 32-bit layout is fixed (asserts generated
 * into dsp_float_filters_layout.inc); the 64-bit build derives its own.
 *
 * All four are block filters on a downward-written history: a heap block `hist` of (taps + extra) elements and an index `room` that counts DOWN from
 * `extra`.  Each new sample is stored at hist[room]; when `room` would become negative, the newest `taps - 1` elements hist[0 .. taps-2] are copied
 * (backwards, element by element) to hist[len-taps+1 .. len-1] and `room` restarts at `len - taps`.  The `taps` newest samples are thus contiguous at
 * hist[room .. room+taps-1] (newest first) for the dot product, without a modulo per sample.
 *
 * Numerics.  Samples and coefficients are float32 in memory, but the dot products are accumulated in extended precision (`long double`, two accumulators
 * for the even and odd terms) and rounded to float32 only where a result is stored; accumulating in float would change the low bits of every output.  The
 * input-sample stores into the history and the history shifts are plain 32-bit word moves, so a signalling NaN is kept as it is.
 *
 * Preconditions common to the classes: the constructors do not check the sysdep_malloc() results (FloatFIR / FloatIIR only skip the zero fill), and
 * the history wrap is a do-while over (taps - 1) elements, so an object with fewer than 2 taps must not be run past `extra` samples (the loop would run
 * about 2^32 times).
 */
#ifndef DSP_CLASSES_P02_H
#define DSP_CLASSES_P02_H

#include "core/dsp_cxx.h"

/**
 * FIR filter  y[n] = sum_k coef[k] * x[n-k].
 *
 * `taps_` is the constructor's `ntaps` rounded DOWN to a multiple of 4.  The coefficient pointer is BORROWED (stored, not copied, not freed) and must
 * stay valid and hold at least `taps_` floats.  The history is `taps_ + extra` floats, zero-filled (if the allocation succeeded), `room_` starts at `extra`.
 * process(x) stores x at hist[room], returns sum_k coef[k] * hist[room + k] and decrements `room`.
 */
class FloatFIR
{
public:
  FloatFIR(unsigned ntaps, float* coef, unsigned extra);
  ~FloatFIR();
  void reset();                                     ///< zero the history (if allocated), room_ = histLen_ - taps_
  int setCoefficients(float* coef, unsigned ntaps); ///< change the tap count and coefficient pointer; -1 (nothing changed) if (ntaps & ~3) >= histLen_, else 0
  float process(float x);                           ///< one sample
  void process(const float* in, float* out, unsigned n); ///< `n` samples

  float* coef_;      ///< borrowed coefficients
  float* hist_;      ///< heap block of histLen_ floats
  unsigned taps_;    ///< number of taps (multiple of 4)
  unsigned histLen_; ///< taps_ + extra
  int room_;         ///< write index, counts down
};

/**
 * All-pole (feedback) filter  y[n] = in[n] + sum_k coef[k] * y[n-1-k].
 *
 * Same layout and constructor / reset / setCoefficients behaviour as FloatFIR (including the rounding of the tap count down to a multiple of 4 and the
 * borrowed coefficient pointer), but the history holds past OUTPUTS and `room_` indexes the newest output.  Only the block process exists.
 */
class FloatIIR
{
public:
  FloatIIR(unsigned ntaps, float* coef, unsigned extra);
  ~FloatIIR();
  void reset();
  int setCoefficients(float* coef, unsigned ntaps);
  void process(const float* in, float* out, unsigned n); ///< `n` samples; the output sample is stored in the history

  float* coef_;      ///< borrowed coefficients
  float* hist_;      ///< heap block of histLen_ floats (past outputs)
  unsigned taps_;    ///< number of feedback taps (multiple of 4)
  unsigned histLen_; ///< taps_ + extra
  int room_;         ///< index of the newest output
};

/**
 * ARMA filter  y[n] = (sum_k ma[k] * x[n-k] - sum_{k>=1} ar[k] * y[n-k]) / ar[0].
 *
 * The coefficient arrays are COPIED (the lengths rounded UP to multiples of 4, the padding zeroed) and divided by ar[0] unless ar[0] is 1.0 or NaN;
 * ar_[0] is then set to 0, so the feedback dot product ignores the slot that is about to receive the new output.  process(x) is the moving-average
 * part minus the auto-regressive part, each rounded to float32 into `y_` / `w_`, and the difference rounded again.  Separate input and output
 * histories, each with its own wrap.
 */
class FloatARMA
{
public:
  FloatARMA(unsigned nAr, unsigned nMa, float* ar, float* ma, unsigned extra);
  ~FloatARMA();
  void reset();
  float process(float x);
  void process(const float* in, float* out, unsigned n); ///< block form; not called anywhere in this tree

  float* ar_;     ///< heap, nAr4_ floats (ar_[0] == 0 after construction)
  float* ma_;     ///< heap, nMa4_ floats
  float* xhist_;  ///< heap, xlen_ floats (input history)
  float* yhist_;  ///< heap, ylen_ floats (output history)
  unsigned nAr4_; ///< nAr rounded UP to a multiple of 4
  unsigned nMa4_; ///< nMa rounded UP to a multiple of 4
  unsigned xlen_; ///< nMa4_ + extra
  unsigned ylen_; ///< nAr4_ + extra
  int xr_;        ///< write index of the input history
  int yr_;        ///< write index of the output history
  unsigned idx_;  ///< loop counter left behind by the constructor / reset; its value carries no state (after either it is ylen_)
  float y_;       ///< last moving-average sum, then the last output
  float w_;       ///< last auto-regressive sum
};

/**
 * Direct-form IIR filter with coefficients and history of type C (double) and samples of type T (float):
 *   y = (sum_{i<nNum} num[i] * hnum[rNum + i] - sum_{i=1..nDen-1} den[i] * hden[rDen + i]) / den[0]   (no division when den[0] == 0)
 * accumulated in extended precision.  The input is stored in the numerator history first, the output (as C) in the denominator history afterwards.
 * The coefficient pointers are BORROWED.  process(T) divides by den[0]; the block version multiplies by (1 / den[0]) computed in extended precision, so
 * the two forms can differ in the last bit.  Both `nDen` and `nNum` must be at least 1.
 */
template <class T, class C> class GenericIIR
{
public:
  GenericIIR(unsigned nDen, unsigned nNum, C* den, C* num, unsigned extra);
  ~GenericIIR();
  void reset();
  T process(T x);
  void process(const T* in, T* out, unsigned n);

  C* den_;          ///< borrowed denominator coefficients
  C* num_;          ///< borrowed numerator coefficients
  C* hnum_;         ///< heap, lenNum_ elements (input history)
  C* hden_;         ///< heap, lenDen_ elements (output history)
  unsigned nDen_;   ///< number of denominator coefficients
  unsigned nNum_;   ///< number of numerator coefficients
  unsigned lenNum_; ///< nNum_ + extra
  unsigned lenDen_; ///< nDen_ + extra
  int rNum_;        ///< write index of the input history
  int rDen_;        ///< write index of the output history
  unsigned idx_;    ///< loop counter left behind by the constructor / reset / process; carries no state
  C y_;             ///< last output
};

typedef FloatFIR P02_FloatFIR;
typedef FloatIIR P02_FloatIIR;
typedef FloatARMA P02_FloatARMA;
typedef GenericIIR<float, double> P02_GenericIIR;

#include "dsp_float_filters_layout.inc"

#endif /* DSP_CLASSES_P02_H */
