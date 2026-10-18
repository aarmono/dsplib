/**
 * @file v90_connection_evaluator.cpp
 * @brief V.90 connection evaluator: decides when the running V90 connection should change rate, retrain or fall back to V.34.
 *
 * Implements V90ConnectionEvaluator (declared in v90_connection_evaluator.h), owned by the V.90 demodulator.  The equalizer feeds it a weighted running
 * mean of the pre-decision SNR error (updateAvePdsnr()) and the demodulator periodically asks for a verdict:
 * - evaluateConnection(): the data phase.  Handles external demands (error-correction rate-down, remote requests), the rate-up / rate-down / retrain
 *   duration counters against the thresholds supplied by the constellation designer (updateCurrentConstellationData()), the echo-RRN state machine used
 *   when the high-level transmitter is active, the fade counters that forgive old retrains, and the debug demand generators.
 * - evaluatePhase3() / evaluatePhase4(): the shorter checks during the TRN1d / phase 3 and phase 4 training.
 * - indicateLocalRetrain() / indicateRemoteRetrain() / indicateRemoteRateReneg(): event counters; too many retrains give a fall back to V.34.
 * Verdict codes: 0 nothing, 1 rate up, 2 rate down, 3 rate down without restriction, 4 retrain, 5 fall back to V.34.  All limits and durations come from
 * the V90Parameters block, which the evaluator reads and (only in evaluateConnection) writes: rrnSilenceMinEchoEnergyForKeepRate.
 *
 * Arithmetic: averages, thresholds and the parameter values are single-precision floats in memory, but every comparison and every product is carried
 * out in extended precision (long double) and rounded to float only when stored.  Comparison helpers (jmpB / jmpBe / jmpA / jmpAe) reproduce the x87
 * branch conditions: an unordered (NaN) comparison counts as "below" and "below or equal" and as neither "above" nor "above or equal".  The debug
 * prints show floats as sign / integer / fractional parts (splitValue()) without the C library: the sign is '-' when 0 >= v (so zero prints '-'), '+'
 * otherwise (a NaN prints '+'), the integer part is truncated, and the fraction is truncated to the print scale (100, 1000 or 10000).
 *
 * The control flow of evaluateConnection() is deliberately kept as labelled blocks (echoStateCheck, echoRateDown, plainRateDown, noRateDown,
 * retrainCheck, retrainCount, retrainReset, rateRenegCheck, resetInitDmin, clearSilence, retrainInstead) because the paths share tails.
 *
 * Behaviour worth knowing about:
 * - the fade counters divide by retrainCounterFadeCount / remoteRrnCounterFadeCount whenever the matching event counter is non-zero; a zero count traps;
 * - the error-correction branch of evaluateConnection() returns directly, with weight_ = avePdsnr_ = 0 and externalCommand_ = -1;
 * - echoRrnState_ above 2 skips the state-3 block (signed 16-bit compares), and echoRrnState_ + 1 is stored as a 16-bit value;
 * - evaluatePhase3() continues with the pdsnrThresholdInPhase3 test after its large-error fall back (the verdict stays 5 unless that test overrides it).
 */

#include "v90_connection_evaluator.h"
#include "core/dsp_math.h"
#include "core/dsp_asm.h"
#include "core/dsp_host.h"
#include "v34/v34_api.h"

extern "C"
{
}

typedef long double LD;

/* ---- helpers (static inline) ---- */

#include "core/dsp_x87_cmp.h"

#include "core/dsp_intmath.h"

#include "vpcm_split.h"
#include "vpcm_split_value.h"

#include "core/dsp_copyword_raw.h"
#include "core/dsp_floatbits.h"

/**
 * @brief Print helpers for a float shown as "%c%d.%0Nd" (sign, integer part, fraction) through edprintf() or dsplibs_debug_printf().
 * @param fmt    format string whose conversions are the optional leading argument followed by %c %d %d
 * @param n      (edNAve) leading %d argument, normally the retrain counter; (edAve2) the same as a signed int
 * @param v      value to print
 * @param scale  fraction scale: 100, 1000 or 10000 for two, three or four digits
 */
static inline void edAve(const char* fmt, LD v, LD scale)
{
  Split s = splitValue(v, scale);
  edprintf(fmt, s.sign, s.ip, s.fp);
}
static inline void edNAve(const char* fmt, unsigned n, LD v, LD scale)
{
  Split s = splitValue(v, scale);
  edprintf(fmt, n, s.sign, s.ip, s.fp);
}
static inline void edAve2(const char* fmt, int a, LD v, LD scale)
{
  Split s = splitValue(v, scale);
  edprintf(fmt, a, s.sign, s.ip, s.fp);
}
static inline void dbgAve(const char* fmt, LD v, LD scale)
{
  Split s = splitValue(v, scale);
  dsplibs_debug_printf(fmt, s.sign, s.ip, s.fp);
}

/* ---- construction / reset ---- */

/**
 * @brief Create an evaluator bound to a parameter block and reset it.
 * @param p  parameter block, borrowed; must outlive the evaluator
 */
V90ConnectionEvaluator::V90ConnectionEvaluator(V90Parameters* p)
{
  params_ = p;
  reset();
}

/** @brief Destructor; the evaluator owns nothing. */
V90ConnectionEvaluator::~V90ConnectionEvaluator() {}

/**
 * @brief Return the evaluator to its start-of-connection state.
 * @note  Zeroes every counter and the running average, clears the pending commands (externalCommand_ = -1, initDmin_ = -1), and re-reads the
 *        enables, durations and debug switches from the parameter block.  The large-error durations are fixed at 0x640 weights.
 */
void V90ConnectionEvaluator::reset()
{
  if (dsplibs_debug_level > 1)
    dsplibs_debug_printf("V90ConnectionEvaluator reset called !\r\n");
  edprintf(" *********** Echo Rrn Mechanism *********** \n");
  V90Parameters* p = params_;

  /* state and counters */
  rateUpCounter_ = 0;
  echoRrnState_ = 0;
  phase4RatioCheckEnabled_ = 1;
  altDebugState_ = 0;
  copyWord(&pdsnrV34DropThreshPhase4_, &p->phase4ErrorForV34Fallback);
  altRbsDetectedOnQc_ = 0;
  phase3EvalActive_ = 0;
  initDmin_ = -1;
  rrnSilence_ = 0;
  externalCommand_ = -1;
  rateDownCounter_ = 0;
  retrainCounter_ = 0;
  nofRetrainsSoFar_ = 0;
  nofRemoteRetrainsSoFar_ = 0;
  nofRateRenegSoFar_ = 0;
  debugCounter_ = 0;
  evaluatorCounter_ = 0;
  fadeCounter_ = 0;
  retrainInsteadOfRateDown_ = 0;
  delayedRetrainRequest_ = 0;
  delayedRetrainEnable_ = 0;
  weight_ = 0;
  curDmin_ = 0;

  /* thresholds and average cleared; options and durations copied from the parameter block */
  enableRrnDown_ = p->enableRrnDown;
  storeWord(&threshUp_, 0u);
  storeWord(&threshDown_, 0u);
  storeWord(&threshRetrain_, 0u);
  storeWord(&avePdsnr_, 0u);
  enableRrnUp_ = p->enableRrnUp;
  nofRemoteRateRenegBeforeRetrain_ = (unsigned)p->nofRemoteRateRenegBeforeRetrain;
  debugAlternate_ = p->debugConnectionEvaluatorAlternateDebug;
  debugFallBack_ = p->debugConnectionEvaluatorFallBack;
  debugRetrain_ = p->debugConnectionEvaluatorRetrain;
  debugRateUp_ = p->debugConnectionEvaluatorRateUp;
  debugRateDown_ = p->debugConnectionEvaluatorRateDown;
  retrainCounterFadeCount_ = (unsigned)p->retrainCounterFadeCount;
  remoteRrnCounterFadeCount_ = (unsigned)p->remoteRrnCounterFadeCount;
  rateUpDetectDuration_ = (unsigned)p->rateUpDetectDuration;
  minDurationInDataBeforeRrnUp_ = (unsigned)p->minimumDurationInDataBeforeRrnUp;
  rateDownDetectDuration_ = (unsigned)p->rateDownDetectDuration;
  minDurationInDataBeforeRrnDown_ = (unsigned)p->minimumDurationInDataBeforeRrnDown;
  retrainDetectDuration_ = (unsigned)p->retrainDetectDuration;
  largeErrorDuration_ = 0x640;
  largeErrorDurationPhase4_ = 0x640;
  debugPeriod_ = p->debugConnectionEvaluatorPeriod;
}

/* ---- small members ---- */

/**
 * @brief Fold one measurement into the weighted running mean of the pre-decision SNR error.
 * @param x  new measurement
 * @param w  its weight (number of symbols it stands for)
 * @note  avePdsnr_ = (weight_ * avePdsnr_ + w * x) / (weight_ + w) computed in extended precision, then weight_ += w.  The first measurement after a
 *        verdict (weight_ == 0) simply replaces the average.  A verdict function clears weight_ and avePdsnr_ again.
 */
void V90ConnectionEvaluator::updateAvePdsnr(float x, unsigned w)
{
  unsigned n = weight_;
  if (n == 0)
  {
    avePdsnr_ = x;
    weight_ = w;
    return;
  }
  unsigned cnt = n + w;
  LD sum = (LD)x * (LD)w + (LD)n * (LD)avePdsnr_; /* exact products, one extended sum */
  weight_ = cnt;
  avePdsnr_ = (float)(sum / (LD)cnt); /* single rounding on the store */
}

/**
 * @brief Install the thresholds and minimum distance of the constellation that is now in use.
 * @param curDmin       minimum distance of the current constellation
 * @param threshUp      average error below which a rate up is considered
 * @param threshDown    average error above which a rate down is considered
 * @param threshRetrain average error above which a retrain is considered
 * @note  Also clears rrnSilence_ and prints the thresholds scaled by 10.
 */
void V90ConnectionEvaluator::updateCurrentConstellationData(short curDmin, float threshUp, float threshDown, float threshRetrain)
{
  threshUp_ = threshUp;
  threshDown_ = threshDown;
  threshRetrain_ = threshRetrain;
  curDmin_ = curDmin;
  rrnSilence_ = 0;
  LD ten = (LD)10.0f;
  int a = dsp_trunc_i32((LD)threshUp * ten);
  int b = dsp_trunc_i32((LD)threshDown * ten);
  int c = dsp_trunc_i32((LD)threshRetrain * ten);
  edprintf("V90ConnectionEvaluator UPDATE: curDmin = %d, 10*threshUp = %d, 10*threshDown = %d, 10*threshRetrain = %d\r\n", (int)curDmin, a, b, c);
}

/** @brief Record that the far end started a rate renegotiation; counts towards nofRemoteRateRenegBeforeRetrain_ in evaluateConnection(). */
void V90ConnectionEvaluator::indicateRemoteRateReneg()
{
  rrnSilence_ = 0;
  unsigned n = nofRateRenegSoFar_ + 1;
  nofRateRenegSoFar_ = n;
  edprintf("V90ConnectionEvaluator: remote rrn indicated. (rrn no %d).\r\n", n);
}

/**
 * @brief Record a retrain that this side initiated outside the evaluator.
 * @return 4 (retrain), or 5 (fall back to V.34) when the count exceeds maxNofV90Retrains, in which case the count restarts at 0
 * @note  Clears the evaluation and retrain counters and rrnSilence_.
 */
unsigned V90ConnectionEvaluator::indicateLocalRetrain()
{
  unsigned r = 4;
  V90Parameters* p = params_;
  unsigned n = nofRetrainsSoFar_ + 1;
  rrnSilence_ = 0;
  nofRetrainsSoFar_ = n;
  if (n > (unsigned)p->maxNofV90Retrains)
  {
    r = 5;
    edprintf("V90ConnectionEvaluator: initiating fall back to V34 due to %d V90 retrains\r\n", n);
    nofRetrainsSoFar_ = 0;
  }
  evaluatorCounter_ = 0;
  retrainCounter_ = 0;
  return r;
}

/**
 * @brief Record a retrain requested by the far end.
 * @return 4 (retrain), or 5 (fall back to V.34) when the count exceeds maxNofRemoteRetrains, in which case the count restarts at 0
 * @note  Clears the evaluation and retrain counters and rrnSilence_.
 */
unsigned V90ConnectionEvaluator::indicateRemoteRetrain()
{
  unsigned r = 4;
  V90Parameters* p = params_;
  rrnSilence_ = 0;
  unsigned n = nofRemoteRetrainsSoFar_ + 1;
  nofRemoteRetrainsSoFar_ = n;
  if (n > (unsigned)p->maxNofRemoteRetrains)
  {
    r = 5;
    edprintf("V90ConnectionEvaluator: initiating fall back to V34 due to %d remote retrains\r\n", n);
    nofRemoteRetrainsSoFar_ = 0;
  }
  evaluatorCounter_ = 0;
  retrainCounter_ = 0;
  return r;
}

/** @brief Phase-4 mean-error-deviation check; not implemented, the arguments are ignored. @return always 0 */
unsigned V90ConnectionEvaluator::evaluateMeanErrorStdPhase4(float, float) { return 0; }

/** @brief Print the event counters (retrains, rate renegotiations, evaluation and fade counters) through edprintf(). */
void V90ConnectionEvaluator::printStatus() const
{
  edprintf("V90ConnectionEvaluator Status Summary:\r\n");
  edprintf("V90ConnectionEvaluator: nofRetrainsSoFar = %d\r\n", nofRetrainsSoFar_);
  edprintf("V90ConnectionEvaluator: nofRateRenegSoFar = %d\r\n", nofRateRenegSoFar_);
  edprintf("V90ConnectionEvaluator: nofRemoteRetrainsSoFar = %d\r\n", nofRemoteRetrainsSoFar_);
  edprintf("V90ConnectionEvaluator: rateUpCounter = %d\r\n", rateUpCounter_);
  edprintf("V90ConnectionEvaluator: rateDownCounter = %d\r\n", rateDownCounter_);
  edprintf("V90ConnectionEvaluator: retrainCounter = %d\r\n", retrainCounter_);
  edprintf("V90ConnectionEvaluator: evaluatorCounter = %d\r\n", evaluatorCounter_);
  edprintf("V90ConnectionEvaluator: fadeCounter = %d\r\n", fadeCounter_);
}

/**
 * @brief Phase-3 check of the standard deviation of the mean error.
 * @param std  measured deviation
 * @return 5 (fall back to V.34) when trn1dMeanErrorStdEvaluationEnable is set and trn1dMaxMeanErrorStdInPhase3 is above std; otherwise 0
 * @note  On a fall back the retrain count, evaluation counter and rrnSilence_ are cleared.
 */
unsigned V90ConnectionEvaluator::evaluateMeanErrorStdPhase3(float std)
{
  V90Parameters* p = params_;
  if (p->trn1dMeanErrorStdEvaluationEnable == 0)
    return 0;
  if (!jmpBe((LD)p->trn1dMaxMeanErrorStdInPhase3, (LD)std))
    return 0; /* the maximum is not above std (or unordered): no fall back */
  edAve("V90ConnectionEvaluator (phase3): initiating fall back to V34 due to large mean error variance, std = %c%d.%04d\r\n", (LD)std, (LD)10000.0f);
  nofRetrainsSoFar_ = 0;
  evaluatorCounter_ = 0;
  rrnSilence_ = 0;
  return 5;
}

/* ---- evaluatePhase3 ---- */

/**
 * @brief Verdict for the phase-3 / TRN1d training interval.
 * @return 0 nothing, 4 retrain, 5 fall back to V.34
 * @note  Does nothing (returns 0 and keeps the average) while weight_ == 0.  Otherwise exactly one of three checks applies, in priority order:
 *        - altRbsDetectedOnQc_: counts a retrain (or falls back when the count exceeds maxNofV90Retrains);
 *        - trn1dEvalActive_: avePdsnr_ above trn1dErrorForV34Fallback for largeErrorDuration_ weights falls back;
 *        - phase3EvalActive_: the same duration test against phase3ErrorForV34Fallback, then avePdsnr_ above pdsnrThresholdInPhase3 for
 *          retrainDetectDuration_ weights gives a retrain (or fall back).
 *        The average and its weight are cleared on exit unless the retrain duration has not yet been reached.
 */
unsigned V90ConnectionEvaluator::evaluatePhase3()
{
  unsigned r = 0;
  unsigned w = weight_;
  if (w == 0)
    return 0;

  /* alternate RBS pattern found during quick connect: retrain */
  if (altRbsDetectedOnQc_ != 0)
  {
    altRbsDetectedOnQc_ = 0;
    V90Parameters* p = params_;
    unsigned n = nofRetrainsSoFar_ + 1;
    nofRetrainsSoFar_ = n;
    if (n > (unsigned)p->maxNofV90Retrains)
    {
      edprintf("V90ConnectionEvaluator (phase3): altRbsDetectedOnQc => initiating fall back to V34 due to %d V90 retrains\r\n", n);
      r = 5;
      nofRetrainsSoFar_ = 0;
    }
    else
    {
      r = 4;
      edprintf("V90ConnectionEvaluator (phase3): altRbsDetectedOnQc => initiating Retrain (retrain no %d)\r\n", n);
    }
    evaluatorCounter_ = 0;
    rrnSilence_ = 0;
  }
  else if (trn1dEvalActive_ != 0)
  {
    LD ave = (LD)avePdsnr_;
    if (jmpBe(ave, (LD)params_->trn1dErrorForV34Fallback))
    { /* not above the threshold (or unordered): the counter restarts */
      rateUpCounter_ = 0;
    }
    else
    {
      unsigned nw = rateUpCounter_ + w;
      rateUpCounter_ = nw;
      if (!(nw < largeErrorDuration_))
      {
        edAve("V90ConnectionEvaluator (end of TRN1d): initiating fall back to V34 due to large error, avePdsnr = %c%d.%03d\r\n", ave, (LD)1000.0f);
        r = 5;
        nofRetrainsSoFar_ = 0;
        evaluatorCounter_ = 0;
        rrnSilence_ = 0;
      }
    }
  }
  else if (phase3EvalActive_ != 0)
  {
    V90Parameters* p = params_;
    LD ave = (LD)avePdsnr_;
    if (jmpA(ave, (LD)p->phase3ErrorForV34Fallback))
    {
      unsigned nw = rateUpCounter_ + w;
      rateUpCounter_ = nw;
      if (!(nw < largeErrorDuration_))
      {
        edAve("V90ConnectionEvaluator (phase3): initiating fall back to V34 due to large error, avePdsnr = %c%d.%03d\r\n", ave, (LD)1000.0f);
        r = 5;
        ave = (LD)avePdsnr_;
        nofRetrainsSoFar_ = 0;
        evaluatorCounter_ = 0;
        rrnSilence_ = 0;
      }
    }
    else
    {
      rateUpCounter_ = 0;
    }
    if (jmpBe(ave, (LD)p->pdsnrThresholdInPhase3))
    { /* not above the retrain threshold (or unordered): the retrain counter restarts */
      retrainCounter_ = 0;
    }
    else
    {
      unsigned nw = weight_ + retrainCounter_;
      retrainCounter_ = nw;
      if (nw < retrainDetectDuration_)
        goto tail;
      unsigned n = nofRetrainsSoFar_ + 1;
      nofRetrainsSoFar_ = n;
      if (n > (unsigned)p->maxNofV90Retrains)
      {
        r = 5;
        edNAve("V90ConnectionEvaluator (phase3): initiating fall back to V34 due to %d V90 retrains, avePdsnr = %c%d.%03d\r\n", nofRetrainsSoFar_, ave,
               (LD)1000.0f);
        nofRetrainsSoFar_ = 0;
      }
      else
      {
        r = 4;
        edNAve("V90ConnectionEvaluator (phase3): initiating Retrain (retrain no %d), due to avePdsnr = %c%d.%03d\r\n", nofRetrainsSoFar_, ave, (LD)1000.0f);
      }
      evaluatorCounter_ = 0;
      rrnSilence_ = 0;
      retrainCounter_ = 0;
    }
  }
tail:
  weight_ = 0;
  storeWord(&avePdsnr_, 0u);
  return r;
}

/* ---- evaluatePhase4 ---- */

/**
 * @brief Verdict for the phase-4 training interval.
 * @param ratio  mean error before / after the last update (meanErrBefToAftUpdateRatio)
 * @return 0 nothing, 4 retrain, 5 fall back to V.34
 * @note  Does nothing while weight_ == 0.  In order:
 *        1. a one-shot ratio check (phase4RatioCheckEnabled_): a ratio below phase4MeanErrorBefToAftUpdateRatioThresh, while fewer than
 *           maxNofRetrainsForRatioCheck retrains have been counted, retrains (or falls back) and ends the evaluation;
 *        2. avePdsnr_ above pdsnrV34DropThreshPhase4_ for largeErrorDurationPhase4_ weights falls back;
 *        3. avePdsnr_ above pdsnrThresholdInPhase4 for retrainDetectDuration_ weights retrains (or falls back) and switches the drop threshold to
 *           pdsnrV34DropThreshPhase4Retrain;
 *        4. a delayed retrain request (delayedRetrainRequest_ with delayedRetrainEnable_) retrains once.
 *        The average and its weight are cleared on exit.
 */
unsigned V90ConnectionEvaluator::evaluatePhase4(float ratio)
{
  unsigned r = 0;
  unsigned w = weight_;
  if (w == 0)
    return 0;
  V90Parameters* p = params_;

  /* one-shot check of the mean-error ratio: skipped when the threshold is >= ratio (or unordered) */
  if (phase4RatioCheckEnabled_ != 0 && !jmpAe((LD)p->phase4MeanErrorBefToAftUpdateRatioThresh, (LD)ratio) &&
      nofRetrainsSoFar_ < (unsigned)p->maxNofRetrainsForRatioCheck)
  {
    unsigned n = nofRetrainsSoFar_ + 1;
    nofRetrainsSoFar_ = n;
    phase4RatioCheckEnabled_ = 0;
    if (n > (unsigned)p->maxNofV90Retrains)
    {
      r = 5;
      edNAve("V90ConnectionEvaluator (phase4): initiating fall back to V34 due to %d V90 retrains, meanErrBefToAftUpdateRatio = %c%d.%03d\r\n",
             nofRetrainsSoFar_, (LD)ratio, (LD)1000.0f);
      nofRetrainsSoFar_ = 0;
    }
    else
    {
      r = 4;
      edNAve("V90ConnectionEvaluator (phase4): initiating Retrain (retrain no %d), due to meanErrBefToAftUpdateRatio = %c%d.%03d\r\n", nofRetrainsSoFar_,
             (LD)ratio, (LD)1000.0f);
    }
    evaluatorCounter_ = 0;
    rrnSilence_ = 0;
    retrainCounter_ = 0;
    goto tail;
  }

  /* large-error fall back and retrain detection on the running average */
  {
    LD ave = (LD)avePdsnr_;
    if (jmpBe(ave, (LD)pdsnrV34DropThreshPhase4_))
    {
      rateUpCounter_ = 0;
    }
    else
    {
      unsigned nw = rateUpCounter_ + w;
      rateUpCounter_ = nw;
      if (!(nw < largeErrorDurationPhase4_))
      {
        edAve("V90ConnectionEvaluator (phase4): initiating fall back to V34 due to large error, avePdsnr = %c%d.%03d\r\n", ave, (LD)1000.0f);
        nofRetrainsSoFar_ = 0;
        evaluatorCounter_ = 0;
        rrnSilence_ = 0;
        weight_ = 0;
        storeWord(&avePdsnr_, 0u);
        return 5;
      }
    }
    if (jmpA(ave, (LD)p->pdsnrThresholdInPhase4))
    {
      unsigned nw = retrainCounter_ + w;
      retrainCounter_ = nw;
      if (!(nw < retrainDetectDuration_))
      {
        unsigned n = nofRetrainsSoFar_ + 1;
        nofRetrainsSoFar_ = n;
        if (n > (unsigned)p->maxNofV90Retrains)
        {
          r = 5;
          edNAve("V90ConnectionEvaluator (phase4): initiating fall back to V34 due to %d V90 retrains, avePdsnr = %c%d.%03d\r\n", nofRetrainsSoFar_, ave,
                 (LD)1000.0f);
          nofRetrainsSoFar_ = 0;
        }
        else
        {
          copyWord(&pdsnrV34DropThreshPhase4_, &p->pdsnrV34DropThreshPhase4Retrain); /* the drop threshold used after a retrain */
          if (dsplibs_debug_level > 1)
          {
            dbgAve("V90ConnectionEvaluator (phase4): pdsnrCurrentV34DropThreshPhase4 set to = %c%d.%03d\r\n", (LD)pdsnrV34DropThreshPhase4_, (LD)1000.0f);
            ave = (LD)avePdsnr_;
          }
          r = 4;
          edNAve("V90ConnectionEvaluator (phase4): initiating Retrain (retrain no %d), due to avePdsnr = %c%d.%03d\r\n", nofRetrainsSoFar_, ave, (LD)1000.0f);
        }
        evaluatorCounter_ = 0;
        rrnSilence_ = 0;
        retrainCounter_ = 0;
      }
    }
    else
    {
      retrainCounter_ = 0;
    }
  }

  /* a retrain that was requested earlier and is now allowed */
  if (delayedRetrainRequest_ != 0 && delayedRetrainEnable_ != 0)
  {
    edprintf("V90ConnectionEvaluator (phase4): Initiating retrain (delayed)...\r\n");
    delayedRetrainRequest_ = 0;
    delayedRetrainEnable_ = 0;
    unsigned n = nofRetrainsSoFar_ + 1;
    nofRetrainsSoFar_ = n;
    if (n > (unsigned)p->maxNofV90Retrains)
    {
      r = 5;
      /** @bug  the format has a %d conversion but no argument is passed; the printed count is whatever the variadic read finds. */
      edprintf("V90ConnectionEvaluator (phase4): initiating fall back to V34 due to %d V90 retrains (last one delayed)\r\n");
      nofRetrainsSoFar_ = 0;
    }
    else
    {
      r = 4;
      edprintf("V90ConnectionEvaluator (phase4): initiating Retrain (retrain no %d), due to delayed retrain request\r\n", n);
    }
    evaluatorCounter_ = 0;
    rrnSilence_ = 0;
  }
tail:
  weight_ = 0;
  storeWord(&avePdsnr_, 0u);
  return r;
}

/* ---- evaluateConnection ---- */

/** @brief Debug prints of the echo-RRN state machine: the scaled error (avePdsnr_ * echoRrnFactor_, an exact extended product) and threshDown_. */
static inline void dbgEchoPrints(const V90ConnectionEvaluator* e)
{
  if (dsplibs_debug_level > 1)
  {
    dbgAve("error -- = %c%d.%03d\r\n", (LD)e->avePdsnr_ * (LD)e->echoRrnFactor_, (LD)1000.0);
    if (dsplibs_debug_level > 1)
      dbgAve("threshold -- = %c%d.%03d\r\n", (LD)e->threshDown_, (LD)1000.0);
  }
}

/**
 * @brief Verdict for the data phase.
 * @return 0 nothing, 1 one rate up, 2 one rate down, 3 rate down without restriction, 4 retrain, 5 fall back to V.34
 * @note  Does nothing (returns 0) while weight_ == 0.  Steps, in order:
 *        1. initDmin_ latches the first curDmin_ seen; evaluatorCounter_ advances by weight_;
 *        2. fade counters: each time fadeCounter_ crosses a multiple of retrainCounterFadeCount_ (remoteRrnCounterFadeCount_ for rate renegotiations)
 *           the corresponding event count drops by one;
 *        3. an external command (externalCommand_ >= 0): 1 / 4 no-restriction rate down, 2 / 5 rate down (4 and 5 also request silence, 5 sets
 *           noRestrictionRrn_), 3 rate up, 6 ignored (fast parameter exchange is not implemented), 0 or > 6 the error-correction demand (a rate
 *           down, or a retrain when curDmin_ has not fallen below twice initDmin_); this last case returns at once;
 *        4. rate up: avePdsnr_ below threshUp_ for rateUpDetectDuration_ weights and at least minDurationInDataBeforeRrnUp_ in data;
 *        5. rate down: avePdsnr_ above threshDown_ for rateDownDetectDuration_ weights and at least minDurationInDataBeforeRrnDown_ in data.  With
 *           highLevelTxActive the echo-RRN state machine (echoRrnState_ 0..3) scales the threshold by 1.52 / 1.39 and shortens the durations;
 *        6. retrain: avePdsnr_ above threshRetrain_ for retrainDetectDuration_ weights counts a retrain (fall back above maxNofV90Retrains);
 *        7. too many remote rate renegotiations become a retrain, and so does a rate down while curDmin_ is not below twice initDmin_;
 *        8. the debug demands generate one verdict every debugPeriod_ weights.
 *        A retrain-instead-of-rate-down option turns a verdict of 2 into 4.  The average and its weight are cleared on exit.
 * @warning Divides by retrainCounterFadeCount_ and remoteRrnCounterFadeCount_ when the matching event counter is non-zero; a zero parameter traps.
 */
unsigned V90ConnectionEvaluator::evaluateConnection()
{
  unsigned r = 0; /* the verdict */
  unsigned weight, fadeCount, newCount, n, evalCount, echoLimit;
  int ecmd;
  int16_t echoState = 0;
  float echoFactor = 0.0f;
  V90Parameters* P = params_;
  LD minDurDown = 0.0L;

  /* latch the first minimum distance, then advance the in-data time */
  if (initDmin_ == -1)
    initDmin_ = curDmin_;
  weight = weight_;
  if (weight == 0)
    return 0;
  evaluatorCounter_ += weight;

  /* the fade counters: one step down for every multiple of the fade count the counter passes */
  fadeCount = fadeCounter_;
  if (nofRetrainsSoFar_ != 0 && (int)((fadeCount + weight) / retrainCounterFadeCount_) > (int)(fadeCount / retrainCounterFadeCount_))
    nofRetrainsSoFar_ = nofRetrainsSoFar_ - 1;
  if (nofRemoteRetrainsSoFar_ != 0 && (int)((fadeCount + weight) / retrainCounterFadeCount_) > (int)(fadeCount / retrainCounterFadeCount_))
    nofRemoteRetrainsSoFar_ = nofRemoteRetrainsSoFar_ - 1;
  if (nofRateRenegSoFar_ != 0 && (int)((fadeCount + weight) / remoteRrnCounterFadeCount_) > (int)(fadeCount / remoteRrnCounterFadeCount_))
    nofRateRenegSoFar_ = nofRateRenegSoFar_ - 1;
  fadeCounter_ = fadeCount + weight;

  /* external demands (0 = error-correction demand, 1..6 as in the function note) */
  ecmd = externalCommand_;
  if (ecmd > -1)
  {
    if ((unsigned)ecmd < 7u)
    {
      unsigned bit = 1u << ecmd;
      if (bit & 0x36u)
      { /* commands 1, 2, 4, 5: rate down */
        if (ecmd == 1 || ecmd == 4)
        {
          edprintf("V90ConnectionEvaluator: Initiating No Restriction RRN (external demand)\r\n");
          r = 3;
        }
        else
        {
          edprintf("V90ConnectionEvaluator: Initiating RRN Down (external demand)\r\n");
          r = 2;
        }
        rrnSilence_ = (externalCommand_ > 3) ? 1 : 0;
        noRestrictionRrn_ = (externalCommand_ == 5) ? 1 : 0;
        evaluatorCounter_ = 0;
        rateDownCounter_ = 0;
        rateUpCounter_ = 0;
        retrainCounter_ = 0;
        externalCommand_ = -1;
        goto L_rateUp;
      }
      if (bit & 0x8u)
      { /* command 3: rate up */
        edprintf("V90ConnectionEvaluator: Initiating RRN up (external demand)\r\n");
        rrnSilence_ = 0;
        r = 1;
        evaluatorCounter_ = 0;
        rateDownCounter_ = 0;
        rateUpCounter_ = 0;
        retrainCounter_ = 0;
        externalCommand_ = -1;
        goto L_rateUp;
      }
      if (bit & 0x40u)
      { /* command 6: fast parameter exchange */
        edprintf("V90ConnectionEvaluator: Initiating Fast Parameters Exchange called - NOT IMPLEMENTED\r\n");
        externalCommand_ = -1;
        goto L_rateUp;
      }
    }
    /* command 0 or > 6: the error-correction mechanism demands a rate down */
    if (enableRrnDown_ == 0)
    {
      edprintf("V90ConnectionEvaluator: can't responed to V42 RRN down because currently at min allowed rate.\r\n");
      externalCommand_ = -1;
      goto L_rateUp;
    }
    edprintf("V90ConnectionEvaluator: before EC RRN: curDmin = %d, initDmin = %d\r\n", (int)curDmin_, (int)initDmin_);
    if ((int)curDmin_ < (int)initDmin_ * 2)
    {
      edprintf("V90ConnectionEvaluator: initiating One Rate Down, due to error correction mechanism demand.\r\n");
      rateDownCounter_ = 0;
      rateUpCounter_ = 0;
      r = 2;
      evaluatorCounter_ = 0;
      noRestrictionRrn_ = 0;
      rrnSilence_ = P->rrnSilenceRequested;
    }
    else
    {
      nofRetrainsSoFar_ = nofRetrainsSoFar_ + 1;
      edprintf("V90ConnectionEvaluator: error correction mechanism demanded rate down,\r\n");
      edprintf("V90ConnectionEvaluator: initiating retrain (retrain no %d), due to %d rate renegotiations down.\r\n", nofRetrainsSoFar_,
               P->maxNofRatesDiffBeforeRetrain);
      retrainCounter_ = 0;
      r = 4;
      evaluatorCounter_ = 0;
      n = nofRetrainsSoFar_;
      initDmin_ = -1;
      if (n > (unsigned)P->maxNofV90Retrains)
      {
        edprintf("V90ConnectionEvaluator: initiating fall back to V34 due to %d V90 retrains\r\n", n);
        nofRetrainsSoFar_ = 0;
        r = 5;
      }
      rrnSilence_ = 0;
    }
    if (r == 2 && retrainInsteadOfRateDown_ != 0)
    {
      edprintf("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\r\n");
      edprintf("V90ConnectionEvaluator: initiating Retrain instead of One Rate Down !!\r\n");
      edprintf("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\r\n");
      rrnSilence_ = 0;
      r = 4;
    }
    weight_ = 0;
    storeWord(&avePdsnr_, 0u);
    externalCommand_ = -1;
    return r;
  }

L_rateUp:
  /* rate up: the average error stays below the up threshold long enough */
  if (enableRrnUp_ != 0 && jmpB((LD)avePdsnr_, (LD)threshUp_))
  {
    newCount = weight_ + rateUpCounter_;
    if (newCount < rateUpDetectDuration_ || evaluatorCounter_ < minDurationInDataBeforeRrnUp_)
    {
      rateUpCounter_ = newCount;
    }
    else
    {
      rateUpCounter_ = 0;
      r = 1;
      evaluatorCounter_ = 0;
    }
  }
  else
  {
    rateUpCounter_ = 0;
  }

  /* the rate-down / retrain counters (with the echo-RRN state machine when the high-level transmitter is active) */
  if (P->highLevelTxActive == 0)
  {
    if (enableRrnDown_ == 0)
      goto noRateDown;
    goto plainRateDown;
  }

  /* echo-RRN state machine: pick the threshold factor from the evaluation time and the state */
  minDurDown = (LD)minDurationInDataBeforeRrnDown_;
  echoLimit = (unsigned)(long long)(minDurDown * (LD)2.3f);
  evalCount = evaluatorCounter_;
  echoState = echoRrnState_;
  if (evalCount >= echoLimit)
    goto echoStateCheck;
  if (echoState != 0)
    goto echoStateCheck;
  echoFactor = 1.52f;
  goto echoRateDown;

echoStateCheck:
  /* state 1 before the minimum duration still uses the smaller factor */
  if (!(evalCount >= minDurationInDataBeforeRrnDown_) && echoState == 1)
  {
    echoFactor = 1.39f;
    goto echoRateDown;
  }
  if (echoState > 2)
    goto plainRateDown;
  if (dsplibs_debug_level > 1)
  {
    dsplibs_debug_printf("V90-mod3  Echo_Rrn_State = %d ", (int)echoState);
    dbgEchoPrints(this);
  }
  rateDownCounter_ = 0;
  echoRrnState_ = 3;
  storeWord(&P->rrnSilenceMinEchoEnergyForKeepRate, 0x40000000u);
  goto plainRateDown;

echoRateDown:
  /* rate down against the scaled threshold; advance the state when the scaled count is reached */
  echoRrnFactor_ = echoFactor;
  if (jmpBe((LD)echoFactor * (LD)avePdsnr_, (LD)threshDown_))
    goto noRateDown;
  newCount = weight_ + rateDownCounter_;
  rateDownCounter_ = newCount;
  if (newCount >= rateDownDetectDuration_ / 3u && jmpBe(minDurDown * (LD)1.3, (LD)evaluatorCounter_))
    goto L_advance;
  if (newCount < rateDownDetectDuration_ / 5u)
    goto retrainCheck;
  if (echoState != 1)
    goto retrainCheck;
  if (jmpA(minDurDown * (LD)0.6, (LD)evaluatorCounter_))
    goto retrainCheck;

L_advance:
  echoRrnState_ = (int16_t)(echoState + 1);
  if (dsplibs_debug_level > 1)
  {
    dsplibs_debug_printf("V90-mod3 CHANGE echoRrnState = %d", (int)(int16_t)(echoState + 1));
    dbgEchoPrints(this);
  }
  r = 2;
  noRestrictionRrn_ = 0;
  rrnSilence_ = P->rrnSilenceRequested;
  storeWord(&P->rrnSilenceMinEchoEnergyForKeepRate, echoRrnState_ == 1 ? 0x3f266666u : 0x3fe66666u);
  rateDownCounter_ = 0;
  evaluatorCounter_ = 0;
  goto retrainCheck;

plainRateDown:
  /* rate down without the echo state machine */
  if (jmpBe((LD)avePdsnr_, (LD)threshDown_))
    goto noRateDown;
  newCount = weight_ + rateDownCounter_;
  rateDownCounter_ = newCount;
  if (newCount < rateDownDetectDuration_)
    goto retrainCheck;
  if (evaluatorCounter_ < minDurationInDataBeforeRrnDown_)
    goto retrainCheck;
  noRestrictionRrn_ = 0;
  rateDownCounter_ = 0;
  r = 2;
  rrnSilence_ = P->rrnSilenceRequested;
  evaluatorCounter_ = 0;
  goto retrainCheck;

noRateDown:
  /* no rate down pending: the retrain test follows */
  rateDownCounter_ = 0;
  if (jmpA((LD)avePdsnr_, (LD)threshRetrain_))
    goto retrainCount;
  goto retrainReset;
retrainCheck:
  /* retrain: the average error stays above the retrain threshold long enough */
  if (jmpBe((LD)avePdsnr_, (LD)threshRetrain_))
    goto retrainReset;

retrainCount:
  newCount = weight_ + retrainCounter_;
  retrainCounter_ = newCount;
  if (newCount < retrainDetectDuration_)
    goto rateRenegCheck;
  n = nofRetrainsSoFar_ + 1;
  nofRetrainsSoFar_ = n;
  if (n > (unsigned)P->maxNofV90Retrains)
  {
    r = 5;
    edNAve("V90ConnectionEvaluator: initiating fall back to V34 due to %d V90 retrains, avePdsnr = %c%d.%02d\r\n", nofRetrainsSoFar_, (LD)avePdsnr_,
           (LD)100.0f);
    nofRetrainsSoFar_ = 0;
  }
  else
  {
    r = 4;
    edNAve("V90ConnectionEvaluator: initiating Retrain (retrain no %d) due to avePdsnr = %c%d.%02d\r\n", nofRetrainsSoFar_, (LD)avePdsnr_, (LD)100.0f);
  }
  evaluatorCounter_ = 0;
retrainReset:
  retrainCounter_ = 0;
rateRenegCheck:
  /* post-processing of the verdict: remote rate renegotiations, rate-down depth, retrain bookkeeping */
  if (nofRateRenegSoFar_ >= nofRemoteRateRenegBeforeRetrain_)
  {
    /* too many remote rate renegotiations: retrain */
    nofRetrainsSoFar_ = nofRetrainsSoFar_ + 1;
    r = 4;
    edprintf("V90ConnectionEvaluator: initiating Retrain (retrain no %d) due to %d remote rate reneg\r\n", nofRetrainsSoFar_, nofRateRenegSoFar_);
    nofRateRenegSoFar_ = 0;
    evaluatorCounter_ = 0;
    retrainCounter_ = 0;
    initDmin_ = -1;
    goto clearSilence;
  }
  if ((int)r == 2)
  {
    edprintf("V90ConnectionEvaluator: on Rate Down: curDmin = %d, initDmin = %d\r\n", (int)curDmin_, (int)initDmin_);
    if ((int)curDmin_ < (int)initDmin_ * 2)
    {
      edAve("V90ConnectionEvaluator: initiating One Rate Down demand, avePdsnr = %c%d.%02d\r\n", (LD)avePdsnr_, (LD)100.0f);
      goto retrainInstead;
    }
    nofRetrainsSoFar_ = nofRetrainsSoFar_ + 1;
    edAve2("V90ConnectionEvaluator: initiating retrain, due to %d rate renegotiations down, avePdsnr = %c%d.%02d\r\n", P->maxNofRatesDiffBeforeRetrain,
           (LD)avePdsnr_, (LD)100.0f);
    edprintf("V90ConnectionEvaluator: retrain no %d\r\n", nofRetrainsSoFar_);
    retrainCounter_ = 0;
    initDmin_ = -1;
    r = 4;
    if (nofRetrainsSoFar_ > (unsigned)P->maxNofV90Retrains)
    {
      edprintf("V90ConnectionEvaluator: initiating fall back to V34 due to %d V90 retrains\r\n", nofRetrainsSoFar_);
      r = 5;
      nofRetrainsSoFar_ = 0;
    }
    goto clearSilence;
  }
  if ((int)r > 2)
  {
    if (r == 4)
      goto resetInitDmin;
    goto retrainInstead;
  }
  if (r == 1)
  {
    edAve("V90ConnectionEvaluator: initiating One Rate Up demand, avePdsnr = %c%d.%02d\r\n", (LD)avePdsnr_, (LD)100.0f);
    goto clearSilence;
  }
  goto retrainInstead;
resetInitDmin:
  initDmin_ = -1;
clearSilence:
  rrnSilence_ = 0;

retrainInstead:
  if (r == 2 && retrainInsteadOfRateDown_ != 0)
  {
    edprintf("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\r\n");
    edprintf("V90ConnectionEvaluator: initiating Retrain instead of One Rate Down !!\r\n");
    edprintf("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\r\n");
    rrnSilence_ = 0;
    r = 4;
  }

  /* debug demands: every `debugPeriod_` weights one of them fires (alternate sequence, fall back, retrain, rate up, rate down) */
  if (debugAlternate_ != 0)
  {
    newCount = weight_ + (unsigned)debugCounter_;
    debugCounter_ = (int32_t)newCount;
    if ((int)newCount < debugPeriod_)
      goto L_end;
    switch ((unsigned)altDebugState_)
    {
    case 1:
      edprintf("V90ConnectionEvaluator: Alternate Debug RRN up\r\n");
      r = 1;
      altDebugState_ = altDebugState_ + 1;
      debugCounter_ = 0;
      rrnSilence_ = 0;
      break;
    case 0:
    case 2:
      edprintf("V90ConnectionEvaluator: Alternate Debug Retrain\r\n");
      r = 4;
      altDebugState_ = altDebugState_ + 1;
      debugCounter_ = 0;
      rrnSilence_ = 0;
      break;
    case 3:
      edprintf("V90ConnectionEvaluator: Alternate Debug RRN down\r\n");
      debugCounter_ = 0;
      altDebugState_ = 0;
      r = 2;
      noRestrictionRrn_ = 0;
      rrnSilence_ = P->rrnSilenceRequested;
      break;
    default:
      break;
    }
    goto L_end;
  }
  if (debugFallBack_ != 0)
  {
    newCount = weight_ + (unsigned)debugCounter_;
    debugCounter_ = (int32_t)newCount;
    if ((int)newCount < debugPeriod_)
      goto L_end;
    edprintf("V90ConnectionEvaluator: Debug fall back to V34\r\n");
    r = 5;
    debugCounter_ = 0;
    rrnSilence_ = 0;
    goto L_end;
  }
  if (debugRetrain_ != 0)
  {
    newCount = weight_ + (unsigned)debugCounter_;
    debugCounter_ = (int32_t)newCount;
    if ((int)newCount < debugPeriod_)
      goto L_end;
    edprintf("V90ConnectionEvaluator: Debug initiating Retrain\r\n");
    r = 4;
    debugCounter_ = 0;
    rrnSilence_ = 0;
    goto L_end;
  }
  if (debugRateUp_ != 0)
  {
    newCount = weight_ + (unsigned)debugCounter_;
    debugCounter_ = (int32_t)newCount;
    if ((int)newCount < debugPeriod_)
      goto L_end;
    edprintf("V90ConnectionEvaluator: Debug One Rate Up\r\n");
    r = 1;
    debugCounter_ = 0;
    rrnSilence_ = 0;
    goto L_end;
  }
  if (debugRateDown_ != 0)
  {
    newCount = weight_ + (unsigned)debugCounter_;
    debugCounter_ = (int32_t)newCount;
    if ((int)newCount < debugPeriod_)
      goto L_end;
    edprintf("V90ConnectionEvaluator: Debug One Rate Down\r\n");
    r = 2;
    debugCounter_ = 0;
    noRestrictionRrn_ = 0;
    rrnSilence_ = P->rrnSilenceRequested;
  }

L_end:
  weight_ = 0;
  storeWord(&avePdsnr_, 0u);
  return r;
}
