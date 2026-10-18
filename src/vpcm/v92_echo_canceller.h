/**
 * @file v92_echo_canceller.h
 * @brief V.92 near-end echo canceller (V92EchoCanceller) and V.90 receive pre-filter (V90PreFilter) with its reference-loop tables.
 *
 * Declares two unrelated classes that share one translation unit pair:
 * - V92EchoCanceller: the near-end echo canceller of the V.92 PCM modem, one member (`ec_`) of VPcmFloModem.  updateEchoHistory() pushes the far-end
 *   (transmitted) samples through a FloatARMA pre-filter into a history; process() subtracts the echo estimate (a dot product of the adaptive
 *   coefficients with the history, starting at `pos_`) from the received samples and, in the two training states, adapts the coefficients
 *   (NLMS style: coef += hist * (beta * error), then beta *= decay); setState() sequences the states 0 (filter only), 1 (count the delay before
 *   training), 2 (fast training) and 3 (slow training) from the V92Parameters and prints every change through edprintf().  The two coefficient
 *   sets of the ARMA pre-filter (v92echoPreFilter_a / _b, 12 floats each) are file-local tables of v92_echo_canceller.cpp.
 * - V90PreFilter: the receiver pre-filter of V90Demodulator (`pf_`).  It derives from FloatFIR (20 or 40 taps) and loads its coefficients from the
 *   tables: the hardware codec type (an index into `dataBase`, taken from the V90Parameters or from the constructor argument) selects a table of
 *   reference loops (`refLoopsTypeN`: six loop measurements of V90Phase2Info::l2_ that characterise a codec, plus the coefficient array type, the gain
 *   index and the V.90 capability of each).  autoSelection() picks the nearest reference loop, selectFilter() loads the coefficients (ISDN NT1 / PBX
 *   connection types, a forced type, or the automatic choice) and setFilter(idx) moves to another gain index.  The tables (preFilterCoefType1 / 2 / 3,
 *   refLoopsType1 / 2 / 4 / 5 / 6 / 7 and dataBase) are defined in v90_prefilter_tables.cpp.
 *
 * Member order is the layout the i386 asserts of v92_echo_canceller_layout.inc check; both classes are plain members of larger objects.
 */
#ifndef V92_ECHO_CANCELLER_H
#define V92_ECHO_CANCELLER_H

#include "core/dsp_cxx.h"
#include "core/dsp_math.h"
#include "dsp/dsp_float_filters.h" /* FloatFIR, FloatARMA */
#include "pcm_leaf_classes.h"      /* V90Phase2Info */
#include "pcm_parameters.h"        /* V90Parameters, V92Parameters */

/** @brief State of V92EchoCanceller::state_; V92EchoCanceller::setState() sequences them. */
enum V92EchoCancellerState
{
  V92_ECHO_FILTER_ONLY = 0,   ///< cancel with the current coefficients, no adaptation
  V92_ECHO_COUNT_DELAY = 1,   ///< pass the input through and count `limit_` samples (echo delay + 400) before training
  V92_ECHO_FAST_TRAINING = 2, ///< adapt with the fast beta / decay of the V92Parameters for `limit_` samples
  V92_ECHO_SLOW_TRAINING = 3  ///< adapt with the slow beta / decay of the V92Parameters for `limit_` samples
};

/**
 * @brief Near-end echo canceller of the V.92 modem (FIR echo estimate over an ARMA-pre-filtered far-end history).
 *
 * @warning The constructor does not check the sysdep_malloc() results, divides by `granularity` (0 traps) and applies setEchoDelay()'s update
 *          to the not yet initialised `delay_` / `histFill_` (reset() overwrites `histFill_` at its end).
 */
class V92EchoCanceller
{
public:
  /**
   * @brief Build the canceller: sizes the coefficient and history arrays from the V92Parameters and creates the ARMA pre-filter.
   * @param p            parameter block, borrowed (v92EchoInitialDelay, v92EchoFilterLength, v92EchoDelayOffset, fast / slow beta, decay, duration)
   * @param granularity  block size of the caller; the history is sized to a whole number of blocks plus two
   * @param extra        extra history samples
   */
  V92EchoCanceller(V92Parameters* p, unsigned granularity, unsigned extra);
  /** @brief Frees the coefficient array, the history and the ARMA pre-filter and clears the pointers. */
  ~V92EchoCanceller();
  /** @brief Change the echo delay by moving the fill point of the history; prints the new delay.  @param delay new delay in samples */
  void setEchoDelay(unsigned delay);
  /** @brief Set the adaptation step `beta_` and print it.  @note No caller in the library; kept for external test harnesses. */
  void setEchoBeta(float beta);
  /** @brief Set the per-sample step decay `decay_` and print it.  @note No caller in the library; kept for external test harnesses. */
  void setDecayFactor(float decay);
  /** @brief Set beta, decay and the echo delay in one call.  @note No caller in the library; kept for external test harnesses. */
  void setEchoParams(float beta, float decay, unsigned delay);
  /** @brief Zero the coefficient array.  @note No caller in the library; kept for external test harnesses. */
  void zeroEchoCoeff();
  /** @brief Restart the history: position 0, zero-filled up to the fill point.  @note No caller in the library; kept for external test harnesses. */
  void resetEchoHistory();
  /** @brief Zero the coefficients and the history, go to the filter-only state with beta = decay = 0 and reset the ARMA pre-filter. */
  void reset();
  /** @brief Switch to @p state (loading beta, decay and the duration from the parameters), print the change and clear `count_`; an unknown state only logs. */
  void setState(V92EchoCancellerState state);
  /** @brief Append @p n far-end samples, ARMA pre-filtered, to the history; when it fills up, the newest nCoef_ - 1 samples move to the front. */
  void updateEchoHistory(float* x, unsigned n);
  /**
   * @brief Cancel the echo in @p n received samples, adapting the coefficients in the training states.
   * @param in   received samples
   * @param out  output samples (@p in minus the echo estimate)
   * @param n    sample count
   * @note A caller that stores 177.0f in out[0] before the call asks for a bypass: out = in, no filtering or adaptation, only `pos_` advances.
   */
  void process(float* in, float* out, unsigned n);

public:
  V92Parameters* params_; ///< borrowed parameter block
  FloatARMA* arma_;       ///< far-end pre-filter, heap, created in the constructor
  int state_;             ///< V92EchoCancellerState
  unsigned limit_;        ///< sample count `count_` has to reach to leave the current counting / training state
  unsigned count_;        ///< samples spent in the current state
  unsigned nCoef_;        ///< number of echo-filter coefficients (a multiple of 4)
  unsigned nCoefM1_;      ///< nCoef_ - 1
  unsigned histLen_;      ///< capacity of `hist_` in samples
  float* coef_;           ///< echo-filter coefficients, nCoef_ floats, heap
  float* hist_;           ///< pre-filtered far-end history, histLen_ floats, heap
  unsigned pos_;          ///< read position of the dot product in `hist_`, wraps at histLen_ - nCoefM1_
  unsigned histFill_;     ///< write position in `hist_`
  float beta_;            ///< current adaptation step
  float decay_;           ///< factor `beta_` is multiplied by after every adapted sample
  unsigned delay_;        ///< echo delay in samples
};

/** @brief Coefficient array a reference loop selects (V90PreFilterRefLoop::coefType); 1 and 2 have 20 taps, 3 has 40. */
enum PreFilterCoefType
{
  PREFILTER_COEF_TYPE_1 = 1,
  PREFILTER_COEF_TYPE_2 = 2,
  PREFILTER_COEF_TYPE_3 = 3
};

/** @brief Hardware codec type: the index into V90PreFilter::dataBase (and the values of the hwCodecType parameter). */
enum __tHardwareCodecTypes__
{
  HWCODEC_UNKNOWN = 0,
  HWCODEC_AD1821,
  HWCODEC_LUCENT,
  HWCODEC_SIEMENS,
  HWCODEC_USB_STLC_7550,
  HWCODEC_ALS300_AD1819,
  HWCODEC_ALS300_AKM4542,
  HWCODEC_ALS300_ICE,
  HWCODEC_ALS300_WOLFSON,
  HWCODEC_AMR_SILABS,
  HWCODEC_SIL3052_INTERNAL,
  HWCODEC_SIL3054,
  HWCODEC_PANTHER_AD1803,
  HWCODEC_SQUEEZER_545A_ALC,
  HWCODEC_RAPTOR_SL2800,
  HWCODEC_SQUEEZER_545A_ITE
};

/** @brief One reference loop: the six loop measurements that characterise a codec setup and the pre-filter it selects. */
struct V90PreFilterRefLoop
{
  char name[32];
  float ref[6];
  int coefType;   ///< PreFilterCoefType
  int coefIndex;  ///< gain index (the value autoSelection() returns)
  int capability; ///< 2 = V.90 with EIA6
};

/** @brief One hardware codec: its name and its table of reference loops, which ends with a record whose name starts with NUL. */
struct V90PreFilterDbEntry
{
  char name[32];
  V90PreFilterRefLoop* refLoops;
};

/**
 * @brief Receiver pre-filter of V90Demodulator: a FloatFIR (20 or 40 taps) whose coefficients are chosen from the tables by codec and line.
 * @warning Row indices are computed from the raw or the clamped gain index depending on the path; see setFilter(unsigned) and selectFilter().
 */
class V90PreFilter : public FloatFIR
{
public:
  /**
   * @brief Pick the codec type (the hwCodecType parameter when it is not negative, else @p codec; out of table range gives 0) and load coefficient array 1.
   * @param codec   codec type used when the parameters do not force one
   * @param info    phase 2 measurements (l2_ is read by autoSelection()), borrowed
   * @param params  parameter block, borrowed
   */
  V90PreFilter(__tHardwareCodecTypes__ codec, V90Phase2Info* info, V90Parameters* params);
  ~V90PreFilter();
  /** @brief FloatFIR::reset(), no reference loop, gain index 0 and coefficient array 1 row 0. */
  void reset();
  /** @brief Load row @p idx of coefficient array @p type.  @note No caller in the library; kept for external test harnesses. */
  void setFilter(PreFilterCoefType type, unsigned idx);
  /**
   * @brief Move to gain index @p idx of the coefficient array of the current reference loop (array 1 with none); no-op when already there.
   * @warning The index is clamped to 30 (arrays 1 / 2) or 50 (array 3) only when a reference loop is set; with none it is used raw.
   */
  void setFilter(unsigned idx);
  /** @brief Tap count (20 or 40) of the current reference loop's array.  @note No caller in the library; kept for external test harnesses. */
  unsigned getFilterLength(unsigned idx);
  /** @brief Pointer to the coefficient row for @p idx.  @note No caller in the library; kept for external test harnesses. */
  float* getFilterPointer(unsigned idx);
  /** @brief True when the current reference loop has the EIA6 capability or the loopType parameter is 6. */
  bool isV90WithEia6() const;
  /** @brief Copy the EIA6 variants of the parameters over the normal ones (equalizer, spectral shaper, BLL timing, retrain limits, thresholds). */
  void setParamEia6();
  /** @brief Empty. */
  void displayParamEia6();
  /** @brief Number of reference loops of the codec's table.  @note No caller in the library; kept for external test harnesses. */
  unsigned getNofRefLoops() const;
  /** @brief Pick the reference loop nearest to the measured loop (sum of squared differences), print it and return its gain index. */
  int autoSelection();
  /** @brief Load the coefficients for the line: ISDN NT1 / PBX gains, the forced coefficient array and gain, or the automatic choice; prints the gain. */
  void selectFilter();
  /** @brief V.90 capability of the current reference loop (autoSelection() picks one when none is set); 1 for EIA6 or loopType 6. */
  int getV90Capability();

  static float preFilterCoefType1[31][20]; ///< coefficient array 1, 31 gains of 20 taps
  static float preFilterCoefType2[31][20]; ///< coefficient array 2, 31 gains of 20 taps
  static float preFilterCoefType3[31][40]; ///< coefficient array 3, 31 gains of 40 taps (gain indices 20 .. 50)
  static V90PreFilterRefLoop refLoopsType1[23];
  static V90PreFilterRefLoop refLoopsType2[33];
  static V90PreFilterRefLoop refLoopsType4[36];
  static V90PreFilterRefLoop refLoopsType5[36];
  static V90PreFilterRefLoop refLoopsType6[34];
  static V90PreFilterRefLoop refLoopsType7[34];
  static V90PreFilterDbEntry dataBase[17]; ///< per codec type: name and reference-loop table

public:
  int codecType_;         ///< index into dataBase
  V90Phase2Info* info_;   ///< borrowed
  V90Parameters* params_; ///< borrowed
  int filterIdx_;         ///< current gain index
  int refLoop_;           ///< index into the codec's reference loops, -1 = none
};

typedef V92EchoCanceller P22_V92EchoCanceller;
typedef V90PreFilter P22_V90PreFilter;

#pragma GCC diagnostic ignored "-Winvalid-offsetof"
#include "v92_echo_canceller_layout.inc"

#endif /* V92_ECHO_CANCELLER_H */
