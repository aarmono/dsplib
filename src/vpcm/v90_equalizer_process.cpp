/**
 * @file v90_equalizer_process.cpp
 * @brief V90Equalizer::process(): the per-call symbol loop of the V.90 / V.92 receive equalizer.
 *
 * process() is the only caller of the decision functions of the phase state machines and the single place where the equalizer adapts.
 * V90Demodulator::progress() calls it once per block of resampled samples.  Callees in other files: V90Equalizer::setLinearEquBeta() / setDfeBeta() /
 * enterPhase4() / enterDataPhase() / enterFPE() / enterRRN() / linearEquFadeEdges() / calcMeanErrorStatistics(), V90Resampler::setBllState() /
 * SdHalfBaudDft() / resetSdHalfBaudDft() / setTimingOffset(), V90ConnectionEvaluator::updateAvePdsnr(), V90PreFilter::isV90WithEia6(),
 * V90Demapper::hardDecision() / linearMappingStudy(),
 * V90Phase3Demodulator::getDecision(), V90Phase4Demodulator::getDecision() / detectRRN() / detectFPE(), edprintf() and dsplibs_debug_printf().
 *
 * Structure.  The n input floats plus one pending sample of the previous call (pend_) are consumed two per symbol (outCnt = (n + pend_) / 2 symbols).  Per
 * symbol pair:
 *  1. The two samples enter the linear history (float: linHist_, newest at the lowest index linPos_; 16-bit mode: hist16_).  The DFE sum (dfeCoef_ . dfeHist_)
 *     and the linear sum (linCoef_ . history) are formed: in float mode as two accumulators in extended precision whose sums are rounded to float (Df, Lf);
 *     in the 16-bit mode as 32-bit integer sums divided by dfeDiv_ / linDiv_ (q1, q2).  The soft decision is y = Lf - Df (float: yf, and its truncated 16-bit
 *     value y16; 16-bit mode: y16 = (short)(q2 - q1)).
 *  2. The decision mode (mode_, 0 .. 6) picks the hard decision: 0 trunc(yf); 1 and 6 V90Phase3Demodulator::getDecision(yf); 2 V90Phase4Demodulator::
 *     getDecision() on the clipped soft value (Lf while the phase-4 state is 0 or 1); 3 V90Demapper::hardDecision() (plus the linear mapping study and the
 *     RRN / FPE detection of phase 4); 4 and 5 V90Phase4Demodulator::getDecision(y16).  The events the phase state machines leave in their event_ words
 *     steer the LMS step sizes, the resampler's BLL state, the switch between the two number formats (enterDataPhase() / enterRRN() / enterFPE() convert
 *     the working data of this function right here), the mean-error study and the adaptation flag.
 *  3. The decision is stored in dec[i].  The error e = y - decision drives the LMS update of both equalizers unless adaptation is off: float mode
 *     coef += hist * beta * e with extended intermediate values; 16-bit mode: the 32-bit coefficient (hi << 16 | lo) += (hist * (+-mul * err)) >> shift.
 *     The DFE history shifts and receives the linear error Lf - decision (16-bit mode: e2 = q2 - decision), the history position steps back (at 0 the
 *     newest linLen_ samples are copied to the top of the history), and errSum_ accumulates trunc(e * e).
 *  4. After the loop: in the 16-bit mode the 16-bit soft values become floats in out[]; when the error block is full (errCnt_ >= errLimit_) the rms error is
 *     smoothed into meanErr_ and reported to the connection evaluator (updateAvePdsnr) and, when enabled, appended to the mean-error history; an odd sample
 *     is kept as the pending sample; every params->linearEquFadeEdgesCycle calls (frameCnt_ counts calls, not symbols) linearEquFadeEdges() runs.
 *
 * Control flow.  The function is one long state machine with labels: the per-symbol loop (symbolLoop), a dispatch on the decision mode (decisionDispatch),
 * one handler per mode (mode0Round, mode1Phase3, mode2Phase4, mode3Demapper, mode4Phase4Short, mode5DataWait, mode6Phase3V92), one label per phase event
 * (onTrn1dStarted, onJdNotDetected, ..., onShortFirstRrnEd), the phase tails (phase3Tail, phase4TailA, phase4TailB) that apply the beta schedule from the
 * state counters, the exit labels that reload the cached format flag (reloadCache, reloadAfterEvent, clearMseA/B), the store of the decision and the error
 * handling (storeDecision, smallError16/smallErrorFloat, adapt16/adaptFloat, floatError), and the epilogue (afterLoop).
 *
 * State kept in locals.  `mmxCache` is a cache of mmx_ that is refreshed exactly where the format may have changed; `mode` is reloaded at the exits of the
 * mode handlers; `adapt` (adaptation on/off) starts at 1 for the whole call, is overwritten by p3_->status_ in some phase-3 states, is set to 0 by a large
 * error and is incremented by the "quiet" test; yf / Lf / Df / decF are volatile floats (rounded on every store); q1 / q2, e1 / e2 are the 16-bit mode
 * counterparts; e is the extended-precision error.  The extended error survives the two debug prints as a float (stashed and reloaded).
 *
 * Conversions.  The float to 16-bit / 32-bit conversions truncate towards zero; out-of-range and NaN give the 16-bit / 32-bit "integer indefinite"
 * (0x8000 / 0x80000000).  The squared error is truncated to 64 bits and its low 32 bits are added to errSum_.  Words of the history and of the pending
 * sample are copied as words (copyWord) so that signalling NaNs survive.
 *
 * Hazards (also documented at the places).
 * - toMmx() converts n - 2 * i samples starting at the current input pointer, which is two samples more than are left: it reads two floats beyond the end
 *   of the caller's block.
 * - In the 16-bit mode the handlers of modes 0, 1, 2 and 6 read yf / Lf / Df, which are not computed there (they hold stale or zero values); the
 *   demodulator does not keep those modes in the 16-bit mode.
 * - mmxIn_ holds 512 shorts and mmxOut_ 256 shorts: n + 1 > 512 or more than 256 symbols overflow them in the 16-bit mode.
 */

#include "v90_equalizer.h"
#include "core/dsp_math.h"
#include "core/dsp_asm.h"
#include "core/dsp_host.h"
#include "v34/v34_api.h"

extern "C"
{
}

typedef P40_V90Equalizer Q;
typedef long double LD;

#define PFX "V90Equalizer: "

/* ------------------------------------------------------------------------------------------------ helpers (static inline) */

#include "core/dsp_copyword_plain.h"

#include "core/dsp_wordarg.h"

#include "core/dsp_x87_conv_int.h"

/** @brief Low 32 bits of a 64-bit truncating conversion; out-of-range and NaN give 0 (the 64-bit "integer indefinite" has a zero low word). */
static inline unsigned trunc64lo(LD v)
{
  if (!(v > -9223372036854775809.0L && v < 9223372036854775808.0L))
    return 0;
  return (unsigned)(unsigned long long)(long long)v;
}

#include "core/dsp_intmath.h"

#include "vpcm_print_helpers.h"

/** @brief Fraction digits of a debug print: abs(chop((v - chop(v)) * 1000)). */
static inline int frac1000(LD v)
{
  int ip = dsp_trunc_i32(v);
  return absI32(dsp_trunc_i32((v - (LD)ip) * (LD)1000.0f));
}

/**
 * @brief Dot product with two accumulators, in extended precision.
 * @param a   first vector
 * @param b   second vector
 * @param cnt number of elements
 * @return B + A, where element k goes to accumulator A for k % 4 in {0, 2} and to B for k % 4 in {1, 3}; the 0 .. 3 trailing elements go to A.  Every product
 *         and sum is evaluated in extended precision.
 */
static inline LD dot2(const float* a, const float* b, unsigned cnt)
{
  LD A = 0.0L, B = 0.0L;
  while (cnt > 3)
  {
    A += (LD)a[0] * (LD)b[0];
    B += (LD)a[1] * (LD)b[1];
    A += (LD)a[2] * (LD)b[2];
    B += (LD)a[3] * (LD)b[3];
    a += 4;
    b += 4;
    cnt -= 4;
  }
  while (cnt != 0)
  {
    A += (LD)*a++ * (LD)*b++;
    --cnt;
  }
  return B + A;
}

/**
 * @brief 16-bit mode LMS update of one coefficient array: (hi << 16 | lo) += (hist[k] * mul) >> shift, written back as two halves.
 * @param hi    high halves
 * @param lo    low halves
 * @param hist  16-bit history aligned with the coefficients
 * @param count number of coefficients
 * @param mul   multiplier (already includes the sign and the error)
 * @param shift right shift, masked to 5 bits
 */
static inline void mmxUpdate(short* hi, short* lo, const short* hist, int count, int mul, int shift)
{
  for (int k = 0; k < count; ++k)
  {
    int c = (int)(((unsigned)(int)hi[k] << 16) | (unsigned)(unsigned short)lo[k]);
    int d = (int)((unsigned)(int)hist[k] * (unsigned)mul);
    d >>= (shift & 31);
    c = (int)((unsigned)c + (unsigned)d);
    lo[k] = (short)c;
    hi[k] = (short)(c >> 16);
  }
}

/**
 * @brief Convert the working data of process() from float to the 16-bit mode (called after enterDataPhase() succeeded).
 * @param in        current input pointer (already past the pair of the current symbol)
 * @param n         total sample count of the call (including the pending sample)
 * @param i         index of the current symbol
 * @param out       soft values of the symbols done so far (converted into mmxOut_)
 * @param Lf        linear sum of the current symbol (becomes q2)
 * @param yf        soft decision of the current symbol (becomes y16)
 * @param Df        DFE sum of the current symbol (becomes q1)
 * @param dfBits32  1: q1 keeps 32 bits (modes 3 to 5); 0: q1 is narrowed to 16 bits (mode 2)
 * @param q2        receives the linear sum as an integer
 * @param y16       receives the 16-bit soft value
 * @param q1        receives the DFE sum as an integer
 * @param mmxCursor receives the start of the converted input in mmxIn_
 * @warning Converts n - 2 * i samples starting at `in`, which is two more than are left in the caller's block; the last two are read beyond it.
 */
static inline void toMmx(Q* e, const float* in, unsigned n, unsigned i, const float* out, float Lf, float yf, float Df, int dfBits32, int* q2, int* y16,
                         int* q1, short** mmxCursor)
{
  *q2 = trunc16((LD)Lf);
  *y16 = trunc16((LD)yf);
  *q1 = dfBits32 ? dsp_trunc_i32((LD)Df) : trunc16((LD)Df);
  short* mi = e->mmxIn_;
  unsigned cnt = n - 2 * i;
  { /* cnt >= 2 here */
    short* q = mi;
    const float* s = in;
    for (unsigned k = cnt; k != 0; --k)
      *q++ = (short)trunc16((LD)*s++);
  }
  *mmxCursor = mi;
  for (unsigned k = 0; k < i; ++k)
    e->mmxOut_[k] = (short)trunc16((LD)out[k]);
}

/**
 * @brief Convert the working data of process() from the 16-bit mode back to float (called after enterRRN() / enterFPE() converted the equalizer).
 * @param in  input pointer, advanced past the pairs already consumed
 * @param i   index of the current symbol
 * @param out receives the soft values of the symbols done so far (from mmxOut_)
 * @param q2  linear sum (becomes *Lf)
 * @param y16 16-bit soft value (becomes *yf)
 * @param q1  DFE sum (becomes *Df)
 * @note The next input float is at base + 1 when mmxIn_[0] is nonzero (a pending sample was staged there) and at base + 2 otherwise, where base = *in + 2 * i.
 */
static inline void toFloat(Q* e, float** in, unsigned i, float* out, int q2, int y16, int q1, volatile float* Lf, volatile float* yf, volatile float* Df)
{
  *Lf = (float)q2;
  short* mi = e->mmxIn_;
  float* base = *in + 2 * i;
  *in = (mi[0] != 0) ? base + 1 : base + 2;
  *yf = (float)(short)y16;
  *Df = (float)q1;
  for (unsigned k = 0; k < i; ++k)
    out[k] = (float)e->mmxOut_[k];
}

/** @brief Debug print of the mean-error ratio before and after the second study, "%c%d.%03d" (sign of the float `v`, digits of the extended `ratio`). */
static inline void printRatio(float v, LD ratio)
{
  int s = signChar((LD)v);
  int ip = intPart(ratio);
  int fr = frac1000(ratio);
  dsplibs_debug_printf(PFX "ph4MeanErrorEnergyBeforeToAfterUpdateRatio = %c%d.%03d\r\n", s, ip, fr);
}

/** @brief Debug print of a large momentary error and the soft decision in float mode: "symbol#%d, error = %c%d.%03d,   soft Decision = %c%d.%03d". */
static inline void printHighError(unsigned i, LD e, float yf)
{
  LD ae = e < 0.0L ? -e : e;
  int fi = dsp_trunc_i32((LD)yf);
  LD t2 = (LD)yf - (LD)fi; /* fraction of yf (the integer part is chopped from yf itself) */
  int yInt = intPart((LD)yf);
  int yFrac = absI32(dsp_trunc_i32(t2 * (LD)1000.0f));
  int eInt = dsp_trunc_i32(ae);
  int ei = dsp_trunc_i32(e);
  int eFrac = absI32(dsp_trunc_i32((LD)1000.0f * (e - (LD)ei)));
  int sgnE = signChar(e);
  int sgnY = signChar((LD)yf);
  edprintf(PFX "High momentary error, symbol#%d, error = %c%d.%03d,   soft Decision = %c%d.%03d\r\n", (int)i, sgnE, eInt, eFrac, sgnY, yInt, yFrac);
}

/* ------------------------------------------------------------------------------------------------ V90Equalizer::process */

/**
 * @note See the file comment for the algorithm.  Sets event_ to 0 on entry; a non-zero event of the phase state machines is left in event_ for the caller.
 * @warning In the 16-bit mode mmxIn_ takes n + 1 samples (512 entries) and mmxOut_ takes outCnt soft values (256 entries); the block size is not checked.
 * @warning The conversion to the 16-bit mode in the middle of a call (toMmx()) reads two floats beyond in[n - 1]; the caller's block needs that slack.
 * @bug  After a switch from the 16-bit mode back to float (toFloat(), via enterRRN() / enterFPE()) the position of the next input float is derived from
 *       `mmxIn_[0] != 0`, used as "a pending sample was staged".  A pending sample that truncates to 0 (|pendSample_| < 1) is taken for "none", and the
 *       input pointer is then one float too far.
 * @bug  `timingOffset_` is passed to V90Resampler::setTimingOffset() at TRN1d start in a quick connect (flag148_ != 0); nothing in the library stores it;
 *       its value is whatever the allocation held.
 */
void V90Equalizer::process(float* in, unsigned n, short* decOut, float* outF, unsigned& outCnt)
{
  unsigned adapt;    /* adaptation on (nonzero) / off */
  unsigned mode;     /* decision mode of the current symbol */
  unsigned mmxCache; /* cache of mmx_ */
  unsigned i;        /* index of the current symbol */
  unsigned cnt;      /* the pair count (n + pend) / 2 */
  short* mmxCursor;  /* the read position in mmxIn_ (sample pairs) */
  int y16, decision, q2, q1, e1, e2;
  volatile float yf, Lf, Df, decF;
  LD e;
  unsigned phaseEvent, phaseState, phaseCount, oldPos, k;
  int clipped, newPos;
  short* mmxBase;

  adapt = 1;
  mode = 0;
  y16 = 0;
  decision = 0;
  q2 = 0;
  q1 = 0;
  e1 = 0;
  e2 = 0;
  yf = 0.0f;
  Lf = 0.0f;
  Df = 0.0f;
  decF = 0.0f;
  e = 0.0L;
  phaseEvent = phaseState = phaseCount = oldPos = k = 0;
  clipped = newPos = 0;

  /* prologue: in the 16-bit mode the samples are converted up front, with the pending sample (or a zero) in front */
  mmxBase = mmxIn_;
  mmxCursor = mmxBase + 1;
  event_ = 0;
  if (mmx_ != 0)
  {
    if (n != 0)
    {
      short* q = mmxCursor;
      const float* s = in;
      for (unsigned t = n; t != 0; --t)
        *q++ = (short)trunc16((LD)*s++);
    }
    if (pend_ != 0)
    { /* the pending sample leads the pairs */
      *mmxBase = (short)trunc16((LD)pendSample_);
      mmxCursor = mmxBase;
      pend_ = 0;
      n++;
    }
    else
    { /* no pending sample: the pairs start at the second slot */
      *mmxBase = 0;
      mmxCursor = mmxBase + 1;
    }
  }
  else
  {
    if (pend_ != 0)
      n++; /* the pending sample counts as one more input sample */
  }
  i = 0;
  cnt = n >> 1;
  outCnt = cnt;
  if (cnt > 0)
    goto symbolLoopStart;
  goto afterLoopReload;

  /* ------------------------------------------------------------------------------------------ the per-symbol loop */
symbolLoopStart:
  mmxCache = mmx_;
symbolLoop:
  if (mmxCache == 0)
    goto floatSamples;

  /* 16-bit mode: the two samples enter the history, the sums are integer dot products */
  {
    short* h16 = hist16_;
    unsigned idx = histPos16_;
    h16[idx] = mmxCursor[0];
    idx--;
    histPos16_ = idx;
    h16[idx] = mmxCursor[1];
    mmxCursor += 2;
    int s1 = 0;
    for (int t = 0; t < (int)dfeLen_; ++t)
      s1 = (int)((unsigned)s1 + (unsigned)((int)dfeHi_[t] * (int)dfeHist16_[t]));
    q1 = s1 / dfeDiv_;
    int s2 = 0;
    const short* hp = h16 + idx;
    for (int t = 0; t < (int)linLen_; ++t)
      s2 = (int)((unsigned)s2 + (unsigned)((int)linHi_[t] * (int)hp[t]));
    q2 = s2 / linDiv_;
    y16 = (int)(short)(unsigned short)(unsigned)(q2 - q1);
  }
  goto decisionDispatch;

floatSamples: /* float mode: the two new samples enter the linear history */
  if (pend_ != 0)
  { /* the pending sample and the first input sample */
    unsigned idx = linPos_;
    pend_ = 0;
    copyWord(&linHist_[idx], &pendSample_);
    linPos_ = idx - 1;
    copyWord(&linHist_[idx - 1], in);
    in += 1;
  }
  else
  { /* two input samples */
    unsigned idx = linPos_;
    copyWord(&linHist_[idx], in);
    linPos_ = idx - 1;
    copyWord(&linHist_[idx - 1], in + 1);
    in += 2;
  }

  /* DFE sum, linear sum and the soft decision */
  {
    LD sumD = dot2(dfeHist_, dfeCoef_, dfeLen_);
    Df = (float)sumD;
    LD sumL = dot2(&linHist_[linPos_], linCoef_, linLen_);
    Lf = (float)sumL;
    LD y = (LD)Lf - (LD)Df;
    y16 = trunc16(y);
    yf = (float)y;
  }

decisionDispatch: /* dispatch on the decision mode */
  mode = mode_;
  if (mode > 6)
    goto storeDecision;
  switch (mode)
  {
  case V90EQ_MODE_RESET:
    goto mode0Round;
  case V90EQ_MODE_PHASE3:
    goto mode1Phase3;
  case V90EQ_MODE_PHASE4:
    goto mode2Phase4;
  case V90EQ_MODE_DATA:
    goto mode3Demapper;
  case V90EQ_MODE_RRN:
    goto mode4Phase4Short;
  case V90EQ_MODE_FPE:
    goto mode5DataWait;
  default:
    goto mode6Phase3V92;
  }

  /* ------------------------------------------------------------------------------------------ mode 0: plain rounding */
mode0Round:
  decision = trunc16((LD)yf);
  goto storeDecision;

  /* ------------------------------------------------------------------------------------------ mode 1: phase 3 (float) */
mode1Phase3:
  decision = (int)p3_->getDecision(yf);
  phaseEvent = p3_->event_;
  if (phaseEvent == 0)
    goto phase3Tail;
  event_ = phaseEvent;
  if (phaseEvent - 3 > 0x11)
    goto phase3Tail;
  switch (phaseEvent)
  {
  case V90DEMOD_STATE_TRN1D_STARTED:
    goto onTrn1dStarted;
  case V90DEMOD_STATE_JDNOT_DETECTED:
    goto onJdNotDetected;
  case V90DEMOD_STATE_SD_TONE_DROPPED:
    goto onSdToneDropped;
  case V90DEMOD_STATE_DIL_HIGH_UCODES_STAGE:
    goto onDilHighUcodesStage;
  case V90DEMOD_STATE_DIL_HIGH_UCODES_STAGE_END:
    goto onDilHighUcodesStageEnd;
  case V90DEMOD_STATE_DIL_MED_UCODES_STAGE:
    goto onDilMedUcodesStage;
  case V90DEMOD_STATE_DIL_MED_UCODES_STAGE_END:
    goto onDilMedUcodesStageEnd;
  case V90DEMOD_STATE_DIL_INITIAL_ERROR_RELAXATION:
    goto onDilInitialErrorRelaxation;
  case V90DEMOD_STATE_DIL_INITIAL_ERROR_RELAX_END:
    goto onDilInitialErrorRelaxEnd;
  case V90DEMOD_STATE_DIL_IN_FREEZE:
    goto onDilInFreeze;
  case V90DEMOD_STATE_PHASE3_TERMINATED:
    goto onPhase3Terminated;
  default:
    goto phase3Tail;
  }

  /* TRN1d started: in a quick connect the stored timing offset is applied and the timing loop released; otherwise the linear equalizer is zeroed and the
   * timing loop restarts in its initial state */
onTrn1dStarted:
  if (flag148_ != 0)
  {
    resampler_->setTimingOffset(WORDARG(timingOffset_));
    resampler_->stateFlag_ = 1;
    goto phase3FallThrough;
  }
  for (k = 0; k < linLen_; ++k)
    linCoef_[k] = 0.0f;
  if (mmx_ != 0)
  {
    unsigned t = linLen_ + 8;
    for (k = 0; k < t; ++k)
    {
      rawLinHi_[k] = 0;
      rawLinLo_[k] = 0;
    }
  }
  resampler_->setBllState(V90BLL_INITIAL, 1);
  goto phase3FallThrough;

  /* the Jd sequence was not detected: normal DIL step sizes */
onJdNotDetected:
  setLinearEquBeta(WORDARG(params_->linearEquDilBeta));
  setDfeBeta(WORDARG(params_->dfeDilBeta));
  goto phase3Tail;

onSdToneDropped:
  resampler_->resetSdHalfBaudDft();
  goto phase3Tail;

  /* DIL, high-ucode stage: high-ucode step sizes (the DFE one divided by w8_ when set) and the timing loop frozen */
onDilHighUcodesStage:
  setLinearEquBeta(WORDARG(params_->linearEquDilHighUcodeBeta));
  if (dsplibs_debug_level > 1)
    dsplibs_debug_printf(PFX "DfeProtectionOnDil = %d \r\n", (int)(short)w8_);
  {
    unsigned short dx = w8_;
    if (dx == 0)
      setDfeBeta(WORDARG(params_->dfeDilHighUcodeBeta));
    else
      setDfeBeta((float)((LD)params_->dfeDilHighUcodeBeta / (LD)(short)dx));
  }
  {
    unsigned rs = resampler_->state_;
    if (rs != V90BLL_FROZEN)
    {
      bllSaved_ = rs;
      resampler_->setBllState(V90BLL_FROZEN, 1);
      edprintf(PFX "DemodDilInHighUcodesStage -> freeze timing\r\n");
    }
    edprintf(PFX "DemodDilInHighUcodesStage\r\n");
  }
  goto phase3Tail;

onDilHighUcodesStageEnd:
  setLinearEquBeta(WORDARG(params_->linearEquDilMedUcodeBeta));
  setDfeBeta(WORDARG(params_->dfeDilMedUcodeBeta));
  edprintf(PFX "DemodDilHighUcodesStageTerminated setting medium ucode beta\r\n");
  goto phase3Tail;

onDilMedUcodesStage:
  setLinearEquBeta(WORDARG(params_->linearEquDilMedUcodeBeta));
  setDfeBeta(WORDARG(params_->dfeDilMedUcodeBeta));
  edprintf(PFX "DemodDilInMedUcodesStage\n");
  goto phase3Tail;

onDilMedUcodesStageEnd:
  setLinearEquBeta(WORDARG(params_->linearEquDilBeta));
  setDfeBeta(WORDARG(params_->dfeDilBeta));
  edprintf(PFX "DemodDilInMedUcodesStageTerminated setting normal dil beta\r\n");
  goto phase3Tail;

onDilInitialErrorRelaxation:
  setLinearEquBeta(0.0f);
  setDfeBeta(0.0f);
  edprintf(PFX "DemodDilInitialErrorRelaxation => freeze LE & DFE\r\n");
  goto phase3FallThrough;

onDilInitialErrorRelaxEnd:
  setLinearEquBeta(WORDARG(params_->linearEquDilErrorRelaxBeta));
  setDfeBeta(WORDARG(params_->dfeDilErrorRelaxBeta));
  resampler_->setBllState(V90BLL_DIL, 1);
  edprintf(PFX "DemodDilInitialErrorRelaxTerminated => unfreezing LE & DFE and Timing.\r\n");
  goto phase3Tail;

onDilInFreeze:
  setLinearEquBeta(0.0f);
  setDfeBeta(0.0f);
  {
    unsigned rs = resampler_->state_;
    if (rs != V90BLL_FROZEN)
    {
      bllSaved_ = rs;
      resampler_->setBllState(V90BLL_FROZEN, 1);
    }
  }
  edprintf(PFX "DemodDilInFreeze => freeze LE & DFE & timing.\r\n");
  goto phase3Tail;

onPhase3Terminated:
  enterPhase4();
phase3FallThrough:
  /* falls into the phase-3 tail */

  /* phase-3 tail: the half-baud DFT of the resampler while waiting for the SD tone, the beta schedule of the TRN1d states, the adaptation flag and the
   * mean-error study enable */
phase3Tail:
  phaseState = p3_->state_;
  phaseCount = p3_->cnt_;
  if (phaseState == P3D_WAIT_FOR_SD && p3_->sdToneSeen_ != 0)
  {
    if (mmx_ != 0)
    {
      resampler_->SdHalfBaudDft((float)hist16_[histPos16_ + 1]);
      resampler_->SdHalfBaudDft((float)hist16_[histPos16_]);
    }
    else
    {
      resampler_->SdHalfBaudDft(WORDARG(linHist_[linPos_ + 1]));
      resampler_->SdHalfBaudDft(WORDARG(linHist_[linPos_]));
    }
  }

  /* TRN1d with known data: after the freeze duration switch to the TRN1d step sizes (not in a quick connect) */
  if (p3_->state_ == P3D_TRN1D_KNOWN)
  {
    if ((unsigned)params_->linearEquTrn1dFreezeDuration == phaseCount && flag148_ == 0)
      setLinearEquBeta(WORDARG(params_->linearEquTrn1dBeta));
  }
  if (p3_->state_ == P3D_TRN1D_KNOWN)
  {
    if ((unsigned)params_->dfeTrn1dFreezeDuration == phaseCount && flag148_ == 0)
      setDfeBeta(WORDARG(params_->dfeTrn1dBeta));
  }

  /* TRN1d with decision-directed data in a quick connect: switch to the data step sizes after the quick-connect freeze duration */
  if (flag148_ != 0 && p3_->state_ == P3D_TRN1D_DD)
  {
    if ((unsigned)params_->linearEquQcTrn1dFreezeDuration == phaseCount)
      setLinearEquBeta(WORDARG(params_->linearEquDataBeta));
    if ((unsigned)params_->dfeQcTrn1dFreezeDuration == phaseCount)
      setDfeBeta(WORDARG(params_->dfeDataBeta));
  }

  /* in the study states the phase-3 status decides whether the equalizer adapts */
  phaseState = p3_->state_;
  if (phaseState == P3D_V92_STUDY_UREF || phaseState == P3D_FIRST_STUDY || phaseState == P3D_SECOND_STUDY || phaseState == P3D_THIRD_STUDY ||
      phaseState == P3D_ERROR_RELAXATION || phaseState == P3D_QC_FIRST_STUDY || phaseState == P3D_QC_SECOND_STUDY || phaseState == P3D_QC_THIRD_STUDY)
    adapt = p3_->status_;
  phaseState = p3_->state_;
  if (phaseState == P3D_PROBING_DIL)
    adapt = p3_->status_;
  else if (phaseState == P3D_TRN1D_DD)
  {
    if ((unsigned)params_->nofDdSymbolsBeforeMeanErrorDiagPhase3 == phaseCount)
      mseEnable_ = 1;
  }
  phaseState = p3_->state_;
  if (phaseState == P3D_FIRST_STUDY || phaseState == P3D_QC_FIRST_STUDY)
    goto clearMseB;
  goto reloadCache;

  /* ------------------------------------------------------------------------------------------ mode 2: phase 4 (float, clipped) */
mode2Phase4:
  if (yf > 32767.0f)
    yf = 32767.0f;
  else if (!(yf >= -32767.0f))
    yf = -32767.0f;
  if ((int)p4_->state_ > 1)
    clipped = trunc16((LD)yf);
  else
    clipped = trunc16((LD)Lf); /* before TRN2d the DFE output is not used yet */
  decision = (int)p4_->getDecision((short)clipped);
  phaseEvent = p4_->event_;
  if (phaseEvent == 0)
    goto phase4TailA;
  event_ = phaseEvent;
  if (phaseEvent == V90DEMOD_STATE_MIDDLE_OF_TRN2D)
    goto onMiddleOfTrn2d;
  if ((int)phaseEvent > 0x18)
  {
    if (phaseEvent == V90DEMOD_STATE_ED_DETECTED)
      goto onEdDetected;
    if (phaseEvent == V90DEMOD_STATE_PHASE4_TERMINATED)
      goto onPhase4Terminated;
    goto phase4TailA;
  }
  if (phaseEvent == V90DEMOD_STATE_RINOT_DETECTED)
    goto onRiNotDetected;
  goto phase4TailA;

  /* Ri not detected: TRN2d initial step sizes (the DFE one depends on the German PBX / EIA6 case) and the TRN2d timing loop */
onRiNotDetected:
  setLinearEquBeta(WORDARG(params_->linearEquTrn2dInitialBeta));
  if (sv_->flag_ == 2)
    setDfeBeta(WORDARG(params_->germanPbxDfeTrn2dFastBeta));
  else if (!pf_->isV90WithEia6())
    setDfeBeta(WORDARG(params_->dfeTrn2dBeta));
  else
  {
    edprintf(PFX "RiNot on EIA6 DFE fast\r\n");
    setDfeBeta(WORDARG(params_->eia6DfeTrn2dFastBeta));
  }
  if (flag148_ != 0)
    resampler_->setBllState(V90BLL_TRN2, 1);
  else
    resampler_->setBllState(V90BLL_TRN2_INITIAL, 1);
  goto phase4TailA;

  /* middle of TRN2d: slow DFE step size in the German PBX and EIA6 cases */
onMiddleOfTrn2d:
  if (sv_->flag_ == 2)
  {
    edprintf(PFX "middle of TRN2d\r\n");
    setDfeBeta(WORDARG(params_->germanPbxDfeTrn2dSlowBeta));
  }
  if (pf_->isV90WithEia6())
  {
    edprintf(PFX "middle of TRN2d (DFE EIA6 CONDITION)\r\n");
    setDfeBeta(WORDARG(params_->eia6DfeTrn2dSlowBeta));
  }
  goto phase4TailA;

onEdDetected:
  setLinearEquBeta(WORDARG(params_->linearEquDataBeta));
  setDfeBeta(WORDARG(params_->dfeDataBeta));
  goto phase4TailA;

  /* phase 4 terminated: enter the data phase and, when that converted to the 16-bit mode, convert the working data */
onPhase4Terminated:
  if (enterDataPhase() != 0)
    toMmx(this, in, n, i, outF, Lf, yf, Df, 0, &q2, &y16, &q1, &mmxCursor);
  goto phase4TailA;

  /* phase-4 tail A: TRN2d step size schedule, the mean-error study, and the evaluation of the first and second study */
phase4TailA:
  phaseState = p4_->state_;
  phaseCount = p4_->cnt_;
  if (phaseState == P4D_TRN2D_KNOWN || phaseState == P4D_TRN2D_ENTRY)
  {
    if ((unsigned)params_->linearEquTrn2dInitialDuration == phaseCount)
    {
      setLinearEquBeta(WORDARG(params_->linearEquTrn2dBeta));
      if (flag148_ == 0)
        resampler_->setBllState(V90BLL_TRN2, 1);
    }
  }
  if (p4_->state_ == P4D_TRN2D_KNOWN)
  {
    if (p4_->studyEnd_ == phaseCount)
      mseEnable_ = 1;

    /* the first study ended: keep its mean error and restart the history */
    if (p4_->dem_->studyDone_ != 0 && flag144_ != 0)
    {
      flag144_ = 0;
      (void)calcMeanErrorStatistics();
      mseFull_ = 0;
      copyWord(&stat13c_, &statMean_);
      mseIdx_ = 0;
    }

    /* the second study ended: the ratio of the two mean errors (state_ is read again after the call) */
    if (p4_->state_ == P4D_TRN2D_KNOWN)
    {
      if (p4_->dem_->study2Done_ != 0 && flag146_ != 0)
      {
        flag146_ = 0;
        (void)calcMeanErrorStatistics();
        LD ratio;
        if (statMean_ == 0.0f || statMean_ != statMean_)
          ratio = 0.0L; /* statMean_ equal to zero or NaN */
        else
          ratio = (LD)stat13c_ / (LD)statMean_;
        ratio_ = (float)ratio;
        if (dsplibs_debug_level > 1)
          printRatio(ratio_, ratio);
      }
    }
  }
  phaseState = p4_->state_;
  if (phaseState == P4D_WAIT_MP || phaseState == P4D_WAIT_CP)
    goto clearMseA;
  goto reloadCache;

  /* ------------------------------------------------------------------------------------------ mode 3: data phase / demapper */
mode3Demapper:
  decision = (int)dem_->hardDecision((short)y16);
  if (dem_->linMapStudyOn_ != 0)
    dem_->linearMappingStudy((short)y16, (short)decision);

  /* rate renegotiation detected: stop the mapping study, enter the RRN state (converts back to float when it was in the 16-bit mode) */
  if (p4_->detectRRN((short)decision) != 0)
  {
    event_ = V90DEMOD_STATE_RATE_RENEG_DETECTED;
    dem_->linMapStudyOn_ = 0;
    if (dsplibs_debug_level > 1)
      dsplibs_debug_printf(PFX "disable linear mapping study.\n");
    if (enterRRN() != 0)
      toFloat(this, &in, i, outF, q2, y16, q1, &Lf, &yf, &Df);
  }

  /* final phase-4 exchange detected: same treatment (detectRRN and detectFPE are both asked for every symbol) */
  if (p4_->detectFPE((short)decision) == 0)
    goto reloadCache;
  event_ = V90DEMOD_STATE_FPE_DETECTED;
  dem_->linMapStudyOn_ = 0;
  if (dsplibs_debug_level > 1)
    dsplibs_debug_printf(PFX "disable linear mapping study.\n");
  if (enterFPE() != 0)
    toFloat(this, &in, i, outF, q2, y16, q1, &Lf, &yf, &Df);
  goto reloadAfterEvent;

  /* ------------------------------------------------------------------------------------------ mode 4: phase 4 (16-bit soft value) */
mode4Phase4Short:
  decision = (int)p4_->getDecision((short)y16);
  phaseEvent = p4_->event_;
  if (phaseEvent == 0)
    goto phase4TailB;
  event_ = phaseEvent;
  if (phaseEvent - 0x1c > 0x19)
    goto phase4TailB;
  switch (phaseEvent)
  {
  case V90DEMOD_STATE_ED_DETECTED:
    goto onShortEdDetected;
  case V90DEMOD_STATE_PHASE4_TERMINATED:
    goto onShortPhase4Terminated;
  case V90DEMOD_STATE_RTNOT_DETECTED:
    goto onShortRtNotDetected;
  case V90DEMOD_STATE_RDNOT_DETECTED:
    goto onShortRdNotDetected;
  case V90DEMOD_STATE_FIRST_RRN_ED_DETECTED:
    goto onShortFirstRrnEd;
  default:
    goto phase4TailB;
  }

  /* Ed detected: freeze while the line is silent between Ed and Rt, otherwise the data step sizes */
onShortEdDetected:
  if (ce_->rrnSilence_ != 0 && p4_->rtPending_ != 0)
  {
    edprintf(PFX "Freezing equ & dfe on silence between Ed and Rt\r\n");
    setLinearEquBeta(0.0f);
    setDfeBeta(0.0f);
  }
  else
  {
    setLinearEquBeta(WORDARG(params_->linearEquDataBeta));
    setDfeBeta(WORDARG(params_->dfeDataBeta));
  }
  goto phase4TailB;

onShortPhase4Terminated:
  if (enterDataPhase() != 0)
    toMmx(this, in, n, i, outF, Lf, yf, Df, 1, &q2, &y16, &q1, &mmxCursor);
  goto phase4TailB;

onShortRtNotDetected:
  setLinearEquBeta(WORDARG(params_->linearEquTrn2dBeta));
  if (sv_->flag_ == 2)
    setDfeBeta(WORDARG(params_->germanPbxDfeTrn2dSlowBeta));
  else if (!pf_->isV90WithEia6())
    setDfeBeta(WORDARG(params_->dfeTrn2dBeta));
  else
  {
    edprintf(PFX "Rtnot on  EIA6 DFE SLOW\r\n");
    setDfeBeta(WORDARG(params_->eia6DfeTrn2dSlowBeta));
  }
  goto phase4TailB;

onShortRdNotDetected:
  if (pf_->isV90WithEia6())
  {
    edprintf(PFX "after rrn (EIA6): set fast...\r\n");
    setLinearEquBeta(WORDARG(params_->linearEquDataBeta));
    setDfeBeta(WORDARG(params_->eia6DfeTrn2dRrnBeta));
  }
  goto phase4TailB;

onShortFirstRrnEd:
  edprintf(PFX "Freezing equ & dfe on silence between Ed and Rt\r\n");
  setLinearEquBeta(0.0f);
  setDfeBeta(0.0f);
  goto phase4TailB;

  /* phase-4 tail B: restart the mean-error history when the study ends */
phase4TailB:
  phaseCount = p4_->cnt_;
  if (p4_->state_ == P4D_TRN2D_KNOWN)
  {
    if (p4_->studyEnd_ == phaseCount)
    {
      mseEnable_ = 1;
      mseIdx_ = 0;
      mseFull_ = 0;
    }
  }
  phaseState = p4_->state_;
  if (phaseState == P4D_WAIT_MP || phaseState == P4D_WAIT_CP)
    goto clearMseB;
  goto reloadCache;

  /* ------------------------------------------------------------------------------------------ mode 5: waiting for the end of phase 4 */
mode5DataWait:
  decision = (int)p4_->getDecision((short)y16);
  phaseEvent = p4_->event_;
  if (phaseEvent == 0)
    goto reloadCache;
  event_ = phaseEvent;
  if (phaseEvent != V90DEMOD_STATE_PHASE4_TERMINATED)
    goto reloadAfterEvent;
  if (enterDataPhase() != 0)
    toMmx(this, in, n, i, outF, Lf, yf, Df, 1, &q2, &y16, &q1, &mmxCursor);
  goto reloadCache;

  /* ------------------------------------------------------------------------------------------ mode 6: phase 3 (float, V.92 channel verification) */
mode6Phase3V92:
  decision = (int)p3_->getDecision(yf);
  phaseEvent = p3_->event_;
  if (phaseEvent == 0)
    goto reloadCache;
  event_ = phaseEvent;
  if (phaseEvent == V90DEMOD_STATE_ANSPCM_DEMOD_ENTERED)
    resampler_->setBllState(V90BLL_FROZEN, 1);
  goto reloadCache;

  /* ------------------------------------------------------------------------------------------ exits of the handlers: reload mmxCache / mode */
clearMseA:
clearMseB:
  mseEnable_ = 0;
reloadAfterEvent:
reloadCache:
  mmxCache = mmx_;
  mode = mode_;

  /* ------------------------------------------------------------------------------------------ the decision is stored, the error drives the adaptation */
storeDecision:
  decOut[i] = (short)decision;
  if (mmxCache == 0)
    goto floatError;

  /* ---- 16-bit mode: error, large-error accounting, LMS update, histories */
  e1 = (int)(short)(unsigned short)(unsigned)(y16 - decision);
  if (absI32(e1) <= 300 || (int)mode <= 1)
    goto smallError16;

  /* a large error in a mode above 1: count it (print the first two) and switch adaptation off */
  if (badCnt_ <= 1)
  {
    edprintf(PFX "High momentary error, symbol#%d, error %d, soft Decision %d\r\n", (int)i, e1, y16);
    mmxCache = mmx_;
  }
  badCnt_ = badCnt_ + 1;
  adapt = 0;
  goto adapt16;

  /* a small error after more than two large ones: the quiet counter `adapt` runs; at 4 the large-error count is reported and cleared */
smallError16:
  if (mode - 10 > 2 && mode != 16 && mode != 13 && mode != 14 && mode != 15 && badCnt_ > 2)
  {
    adapt++;
    if (adapt == 4)
    {
      edprintf(PFX "nof consecutive errors = %d\r\n", (int)badCnt_);
      badCnt_ = 0;
      mmxCache = mmx_;
    }
  }

adapt16:
  e2 = (int)(short)(unsigned short)(unsigned)(q2 - decision);
  if (adapt != 0)
  {
    int mul = (int)((0u - (unsigned)linMul_) * (unsigned)e2);
    mmxUpdate(linHi_, linLo_, hist16_ + histPos16_, (int)linLen_, mul, linShift_);
    int mul2 = (int)((unsigned)e1 * (unsigned)dfeMul_);
    mmxUpdate(dfeHi_, dfeLo_, dfeHist16_, (int)dfeLen_, mul2, dfeShift_);
  }

  /* the DFE history shifts by one, the new entry is e2 */
  oldPos = histPos16_;
  for (k = 0; k < dfeLen_ - 1; ++k)
    dfeHist16_[dfeLen_ - 1 - k] = dfeHist16_[dfeLen_ - 2 - k]; /* a DFE length of 1 skips the loop, with the same result */
  dfeHist16_[0] = (short)e2;

  /* the linear history position steps back; at 0 the newest linLen_ samples are copied to the top */
  newPos = (int)(oldPos - 1);
  if (newPos < 0)
  {
    unsigned L = linHistLen_, N = linLen_;
    histPos16_ = L - N - 1;
    for (k = 0; k < N; ++k)
      hist16_[L - 1 - k] = hist16_[N - 1 - k];
  }
  else
  {
    histPos16_ = (unsigned)newPos;
  }
  errSum_ += (unsigned)((unsigned)e1 * (unsigned)e1);
  mmxOut_[i] = (short)y16;
  goto nextSymbol;

  /* ---- float mode: error, large-error accounting, LMS update, histories */
floatError:
  decF = (float)(LD)(short)decision;
  e = (LD)yf - (LD)(short)decision;
  if (300.0f >= (e < 0.0L ? -e : e))
    goto smallErrorFloat; /* |e| <= 300 (NaN: not taken) */
  if ((int)mode <= 1)
    goto smallErrorFloat;

  /* a large error in a mode above 1: count it (print the first two) and switch adaptation off */
  if (badCnt_ <= 1)
  {
    printHighError(i, e, yf);
    e = (LD)(float)e; /* the error is stashed as a float across the print */
    mmxCache = mmx_;
  }
  badCnt_ = badCnt_ + 1;
  adapt = 0;
  goto adaptFloat;

smallErrorFloat:
  if (mode - 10 > 2 && mode != 16 && mode != 13 && mode != 14 && mode != 15 && badCnt_ > 2)
  {
    adapt++;
    if (adapt == 4)
    {
      e = (LD)(float)e;
      edprintf(PFX "nof consecutive errors = %d\r\n", (int)badCnt_);
      badCnt_ = 0;
      mmxCache = mmx_;
    }
  }

adaptFloat:
  {
    LD Lerr = (LD)Lf - (LD)decF;
    if (adapt != 0)
    {
      LD cd = (LD)dfeBeta_ * e;
      for (k = 0; k < dfeLen_; ++k)
        dfeCoef_[k] = (float)((LD)dfeHist_[k] * cd + (LD)dfeCoef_[k]);
      LD cl = -(LD)linBeta_ * Lerr;
      for (k = 0; k < linLen_; ++k)
        linCoef_[k] = (float)((LD)linHist_[linPos_ + k] * cl + (LD)linCoef_[k]);
    }

    /* the DFE history shifts by one (as words), the new entry is the linear error */
    for (k = 0; k < dfeLen_ - 1; ++k)
      copyWord(&dfeHist_[dfeLen_ - 1 - k], &dfeHist_[dfeLen_ - 2 - k]);
    dfeHist_[0] = (float)Lerr;
  }

  /* the linear history position steps back; at 0 the newest linLen_ samples are copied to the top */
  newPos = (int)(linPos_ - 1);
  if (newPos < 0)
  {
    unsigned L = linHistLen_, N = linLen_;
    linPos_ = L - N - 1;
    for (k = 0; k < N; ++k)
      copyWord(&linHist_[L - 1 - k], &linHist_[N - 1 - k]);
  }
  else
  {
    linPos_ = (unsigned)newPos;
  }
  errSum_ += trunc64lo(e * e);
  outF[i] = yf;

nextSymbol:
  i++;
  if (outCnt > i)
    goto symbolLoop;
  goto afterLoop;

  /* ------------------------------------------------------------------------------------------ after the loop */
afterLoopReload:
  mmxCache = mmx_;
afterLoop:
  /* 16-bit mode: the 16-bit soft values become floats */
  if (mmxCache != 0)
  {
    for (k = 0; k < cnt; ++k)
      outF[k] = (float)mmxOut_[k];
  }

  /* error block: sqrt(errSum / errCnt) smoothed with errK_, reported to the connection evaluator, appended to the history ring */
  errCnt_ += cnt;
  if (errCnt_ >= errLimit_)
  {
    LD sum = (LD)(unsigned long long)errSum_;
    LD num = (LD)(unsigned long long)errCnt_;
    LD kk = (LD)errK_;
    LD omk = (LD)1.0f - kk;
    LD rms = dsp_sqrt(sum / num);
    rmsErr_ = (float)rms;
    LD prev = (LD)meanErr_;
    meanErr_ = (float)(omk * rms + kk * prev);
    ce_->updateAvePdsnr(meanErr_, errCnt_);
    errCnt_ = 0;
    errSum_ = 0;
    if (mseEnable_ != 0)
    {
      unsigned ix = mseIdx_;
      copyWord(&mseHist_[ix], &rmsErr_);
      if (ix + 1 == 300)
      {
        mseIdx_ = 0;
        mseFull_ = 1;
      }
      else
      {
        mseIdx_ = ix + 1;
      }
    }
  }

  /* an odd sample is kept for the next call */
  if (n & 1)
  {
    pend_ = 1;
    if (mmx_ != 0)
      pendSample_ = (float)*mmxCursor;
    else
      copyWord(&pendSample_, in);
  }

  /* periodically taper the ends of the linear equalizer */
  frameCnt_ = frameCnt_ + 1;
  if (frameCnt_ == (unsigned)params_->linearEquFadeEdgesCycle)
  {
    frameCnt_ = 0;
    linearEquFadeEdges();
  }
}
