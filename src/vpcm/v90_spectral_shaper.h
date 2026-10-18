/**
 * @file v90_spectral_shaper.h
 * @brief Classes V90SpectralVerifier (line-condition detector of the V.90 receiver) and V90SpectralShaper (spectral-shaping trellis of the V.90 transmitter).
 *
 * Declares the two classes, the shaper's candidate-code table `V90SpectralShaper::actionLookupTable` (128 ints, read only by advanceTrellis()) and the
 * decimal-power table `pow10Table`.  Definitions are in v90_spectral_shaper.cpp.  Both classes are plain: no virtual functions, no vtable.  Member order is the
 * 32-bit layout, checked on 32-bit builds by the asserts of the generated v90_spectral_shaper_layout.inc (included at the end of this header).
 *
 * V90SpectralVerifier (a member of V90Demodulator, 0x2c bytes in the 32-bit layout) is the "spectral verifier" of the V.90 receiver.  After startAccumulation()
 * it gathers params->spectralVerifierPsdLen samples (process()), runs a Psd over them (window, overlap and FFT length come from the parameter block) and keeps
 * the dB spectrum (fftLen / 2 bins of `resolution_` Hz each). From the spectrum it recognises three line conditions: a German ISDN NT1 box (two peaks next to
 * a
 * null), a German PBX (the same with other frequencies) and a severe codec (two test frequencies far below a reference), sets flag_ to 1 / 2 / 3 and prints the
 * measurements through edprintf() (the floats as "%c%d.%02d", without the C library).  All thresholds and frequencies are named V90Parameters members.
 *
 * V90SpectralShaper (a member of V90Mapper, 0x6c bytes in the 32-bit layout) is the V.90 "spectral shaping" trellis of the transmitter.  Per symbol interval
 * (spf_ = 6 / arg samples) the input sign bits are differentially encoded (a serial encoder on the odd positions, then a ParallelDifferentialEncoder over the
 * lanes), the samples are signed by the result and appended to a history of (depth_ + 1) symbols (histA_).  Once the delay line (delay_ = depth_ symbols) has
 * run down, advanceTrellis() tries the 2^(depth_ + 1) candidate sign patterns of the whole history (the codes of `actionLookupTable`: one decimal digit per
 * symbol, 1 = keep, 2 = negate, 3 / 4 = alternate signs), keeps the one whose V90SpectralShapingFilter metric is smallest and commits the sign action of the
 * OLDEST symbol (the leading digit).  The committed samples go to the output and through the filter, and the history shifts down by one symbol.
 *
 * Numerics: values that the algorithm keeps at extended precision are `long double`; float words are moved as integers where a plain copy is meant; comparisons
 * are written so that a NaN takes the branch that the x87 compare-and-jump sequence takes.
 */
#ifndef V90_SPECTRAL_SHAPER_H
#define V90_SPECTRAL_SHAPER_H

#include "core/dsp_cxx.h"
#include "core/dsp_math.h"
#include "dsp/dsp_int_templates.h"  /* ParallelDifferentialEncoder<uchar>, SerialDifferentialEncoder<uchar> */
#include "dsp/dsp_window_fft_psd.h" /* Psd, WindowType */
#include "pcm_leaf_classes.h"       /* V90SpectralShapingFilter */
#include "pcm_parameters.h"         /* V90Parameters */

/**
 * @brief Line-condition detector: collects a block of receive samples, computes its dB power spectrum and checks it for ISDN NT1 / PBX / severe-codec
 *   signatures.
 */
class V90SpectralVerifier
{
public:
  /**
   * @brief Build the verifier: copy the geometry from the parameter block and allocate the sample buffer, the spectrum and the Psd.
   * @param p  parameter block, borrowed (must outlive the verifier)
   * @warning The allocations (sysdep_malloc) are not checked for failure.  fill_ and state_ are not initialised here; reset() sets them.
   */
  explicit V90SpectralVerifier(V90Parameters* p);
  ~V90SpectralVerifier();
  /** @brief Print "Reset" and clear state_, fill_ and flag_. */
  void reset();
  /** @brief Start collecting samples (state_ = 1, fill_ = 0) unless a collection is already running or the verifier is disabled in the parameters. */
  void startAccumulation();
  /**
   * @brief Convert a frequency to a spectrum bin index.  Nearest = trunc(f / resolution + 0.5), left = trunc(f / resolution), right = left + 1.
   * @note The truncation is 64-bit and only its low 32 bits are returned. None of the three, nor getSpectrumOfBin() / getSpectrumOfNearestBin(), has a
    *        caller.
   */
  unsigned long freqToNearestBin(float f) const;
  unsigned long freqToLeftBin(float f) const;
  unsigned long freqToRightBin(float f) const;
  /** @brief Spectrum value (dB) of bin `bin`, or of the bin nearest to frequency `f`.  @warning No range check against fftLen_ / 2. */
  float getSpectrumOfBin(unsigned long bin) const;
  float getSpectrumOfNearestBin(float f) const;
  /** @brief Print "Spectrum[<Hz>] = <dB>" for every bin, between two dashed lines. */
  void printSpectrum() const;
  /**
   * @brief Set flag_ to 0, then to 1 / 2 / 3 for the ISDN NT1 / German PBX / severe-codec condition (a later one overrides an earlier one). Acts only in
   *   state_ 2.
   */
  void checkSpecialSpectralConditions();
  /**
   * @brief Feed samples; when the buffer is full compute the spectrum, optionally print it and run checkSpecialSpectralConditions().
   * @param in  float samples
   * @param n   number of samples in `in`
   * @return 1 when the spectrum has just been computed (and checked), otherwise 0 (also when not accumulating)
   */
  unsigned process(float* in, unsigned n);

public:
  V90Parameters* params_; ///< parameter block, borrowed
  Psd* psd_;              ///< power-spectral-density engine, heap allocated by the constructor
  float sampleFreq_;      ///< sample rate in Hz
  unsigned fftLen_;       ///< FFT length; the spectrum has fftLen_ / 2 bins
  unsigned psdLen_;       ///< samples gathered per spectrum
  float resolution_;      ///< bin width in Hz: sampleFreq_ / fftLen_
  float* buf_;            ///< heap, psdLen_ samples being gathered
  float* spec_;           ///< heap, fftLen_ / 2 dB values
  unsigned fill_;         ///< samples gathered so far
  unsigned state_;        ///< 0 idle, 1 gathering, 2 spectrum ready
  unsigned flag_;         ///< detected condition: 0 none, 1 ISDN NT1 box, 2 German PBX, 3 severe codec
};

/**
 * @brief Spectral-shaping trellis: signs the transmit samples of each symbol interval so that the V90SpectralShapingFilter metric of the history is minimal.
 */
class V90SpectralShaper
{
public:
  /** Sign pattern applied to one symbol interval of samples by applyFrameAction(). */
  enum ACTIONS
  {
    ACTION_KEEP = 0,
    ACTION_NEGATE = 1,
    ACTION_ALT_NEGATE_EVEN = 2,
    ACTION_ALT_NEGATE_ODD = 3
  };

  /** @brief Build the shaper: allocate the two 24-sample histories and set len_ = 24, fill_ = 0, state_ = 0.  @warning The allocations are not checked. */
  V90SpectralShaper();
  ~V90SpectralShaper();
  /**
   * @brief Configure and restart the trellis.
   * @param depth  trellis depth: the history holds depth + 1 symbols and the delay line depth symbols
   * @param arg    divisor of 6 giving the samples per symbol interval (spf_ = 6 / arg; 0 gives 0)
   * @param c0     filter coefficients of the V90SpectralShapingFilter, passed on as raw float words
   * @param c1     (see c0)
   * @param c2     (see c0)
   * @param c3     (see c0)
   * @warning Nothing is range-checked: histA_ / histB_ hold 24 samples, so (depth + 1) * spf_ > 24 overruns them; actionLookupTable has 8 rows, so
   *          state_ + 2 * depth > 7 (depth > 3) reads past it; pow10Table has 5 entries.
   */
  void reset(unsigned depth, unsigned arg, float c0, float c1, float c2, float c3);
  /** @brief Reset the filter and load new coefficients; the trellis state is untouched. */
  void resetSSFilter(float c0, float c1, float c2, float c3);
  /** @brief Set out[offset .. offset + spf_) from histA_[offset ..] with the sign pattern `action`.  No caller. */
  void applyFrameAction(ACTIONS action, short* out, int offset);
  /** @brief Write the whole history to `out` with the candidate code `actions` applied (digit k acts on symbol depth_ - k).  No caller. */
  void applyAction(int actions, short* out);
  /** @brief Choose the best candidate sign pattern for the history, apply the sign action of the oldest symbol to histA_ and update state_. */
  void advanceTrellis();
  /**
   * @brief Process one symbol interval.
   * @param in    spf_ input samples
   * @param bits  spf_ - 1 sign bits (position 0 is fixed at 0 before the differential encoding)
   * @warning spf_ == 0 (arg 0 in reset()) makes the bit-copy loop run 2^32 - 1 times.
   * @param out   receives spf_ output samples (the oldest symbol of the history)
   */
  void process(short* in, unsigned char* bits, short* out);

  /** Candidate codes: row (state_ + 2 * depth_) of 16 entries, each a decimal number with one digit per symbol; 0 entries are unused. */
  static int actionLookupTable[128];

public:
  unsigned depth_;                                 ///< trellis depth
  unsigned arg_;                                   ///< divisor of 6 (see reset())
  unsigned spf_;                                   ///< samples per symbol interval
  unsigned char bitsIn_[6];                        ///< input sign bits with a leading 0
  unsigned char bitsDiff_[6];                      ///< after the serial differential encoder (odd positions)
  unsigned char bitsOut_[6];                       ///< after the parallel differential encoder; 0 negates the sample
  unsigned char pad1e_[2];                         ///< layout padding, never touched
  unsigned state_;                                 ///< trellis state: 0 or 1 (parity of the last committed action)
  unsigned delay_;                                 ///< symbols still to be collected before the trellis starts deciding
  short* histA_;                                   ///< heap, 24 shorts: the symbol history being signed
  short* histB_;                                   ///< heap, 24 shorts: scratch copy for one candidate
  unsigned fill_;                                  ///< write position in histA_
  unsigned len_;                                   ///< history length in samples: (depth_ + 1) * spf_ (24 after construction)
  SerialDifferentialEncoder<unsigned char> sde_;   ///< differential encoder for the odd positions
  unsigned char pad39_[3];                         ///< layout padding, never touched
  ParallelDifferentialEncoder<unsigned char> pde_; ///< differential encoder across the lanes
  V90SpectralShapingFilter ssf_;                   ///< metric filter
};

/** Powers of ten 1 .. 10000: divisors that pick the leading (oldest-symbol) digit of a candidate code in advanceTrellis(). */
extern unsigned pow10Table[5];

typedef V90SpectralVerifier P21_V90SpectralVerifier;
typedef V90SpectralShaper P21_V90SpectralShaper;

#pragma GCC diagnostic ignored "-Winvalid-offsetof"
#include "v90_spectral_shaper_layout.inc"

#endif /* V90_SPECTRAL_SHAPER_H */
