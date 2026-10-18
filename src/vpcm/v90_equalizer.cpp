/**
 * @file v90_equalizer.cpp
 * @brief V90Equalizer: construction, reset, step-size setters, decision-mode entries, mode conversion back to float, and mean-error statistics.
 *
 * Part of the V.90 / V.92 receiver equalizer (class declared in v90_equalizer.h; the per-call processing is in v90_equalizer_process.cpp and the
 * phase-4 / data-phase entries with the float to 16-bit conversion are in v90_equalizer_phases.cpp).  This file owns:
 * - construction and destruction: the constructor sizes and allocates the float tables (coefficients, histories, fade windows, mean-error history) and, when
 *   the 16-bit ("MMX") mode is allowed, the 16-bit tables with their 8-byte aligned views; reset() then clears everything and sets the identity linear
 *   equalizer;
 * - the step-size setters setLinearEquBeta() / setDfeBeta(), which print a changed step size and recompute the 16-bit multiplier and shift;
 * - the decision-mode entries enterPhase3(), enterChannelVerification(), enterFPE() and enterRRN(), which set mode_ and freeze or restore the equalizer;
 * - linearEquFadeEdges(), which windows the ends of the linear coefficients (through the float form when the 16-bit mode is active), and
 *   restoreEqualizerToFloat(), the inverse of convertEqualizerToMmx();
 * - calcMeanErrorStatistics(), which prints statistics of the mean-error history that process() collects;
 * - a few debugging/utility methods with no caller in the library (zero*, set*Coeff, freeze, getDfeBeta, empty stubs).
 *
 * Callees from other files: hamming(), mean(), Std(), Var() (dsp/dsp_window_fft_psd.h), V90Resampler::setBllState(), V90PreFilter::isV90WithEia6(),
 * edprintf(), sysdep_malloc() / sysdep_free().
 *
 * Numeric conventions.  Float intermediate values are kept in extended precision (`long double`, LD) and rounded once at the store to a float member, as
 * the arithmetic has always been evaluated; do not "simplify" these expressions, results would differ in the last bit.
 * - The beta print is skipped when the new beta compares equal to, or unordered with, the old one; a beta of 0 or NaN gives multiplier = shift = 0.
 * - The 16-bit multiplier is chop((beta * gain) * (1 << (shift & 31))) with an extended intermediate (the shift count is masked to 5 bits); the gain comes
 *   from the float member.
 * - The fade clamp is `x >= 0 ? (x <= 0.5 ? x : 0.5) : 0` (NaN gives 0); the fade lengths are the low 32 bits of chop64(x * N); the windows are
 *   hamming(2 * length).
 * - The 16-bit tables are aligned to 8 bytes only ((p + 7) & ~7).
 */

#include "v90_equalizer.h"
#include "dsp/dsp_window_fft_psd.h"
#include "core/dsp_math.h"
#include "core/dsp_asm.h"
#include "v34/v34_api.h"

extern "C"
{
}

typedef long double LD;

/* ------------------------------------------------------------------------------------------------ message strings (debug output through edprintf) */
#define S_ENTER3 "V90Equalizer: enter Phase 3\r\n"
#define S_RESET "V90Equalizer: reset\r\n"
#define S_LEBETA "V90Equalizer: LE Beta = %c%d.%05de-10\r\n"
#define S_DFEBETA "V90Equalizer: DFE Beta = %c%d.%05de-7\r\n"
#define S_RESTORED "V90Equalizer: Equalizer restored to FLOAT mode.\r\n"
#define S_ENTERFPE "V90Equalizer: enter FPE state\r\n"
#define S_DFEZERO "V90Equalizer: Dfe coefs zeroed (GERMAN_PBX) !!\r\n"
#define S_FREEZED "V90Equalizer: Dfe & Linear EQU coefs freezed (RRN : EIA6 )!!\r\n"
#define S_RESTORING "V90Equalizer: restoring Equalizer to float...\r\n"
#define S_ENTERRRN "V90Equalizer: enter RRN state\r\n"
#define S_ENTERCV "V90Equalizer: enterChannelVerification\r\n"
#define S_HASH "####################################################\r\n"
#define S_DEBUG "V90Equalizer: meanErrorEnergy Debug:\r\n"
#define S_OVER "V90Equalizer: calculated over %d mean errors\r\n"
#define S_DASH "--------------------------------------------\r\n"
#define S_MEAN "V90Equalizer: meanErrorEnergy mean  = %c%d.%06d\r\n"
#define S_CUR "V90Equalizer: current meanErrorEnergy  = %c%d.%06d\r\n"
#define S_STD "V90Equalizer: meanErrorEnergy Std  = %c%d.%06d\r\n"
#define S_VAR "V90Equalizer: meanErrorEnergy Variance  = %c%d.%06d\r\n"
#define S_MIN "V90Equalizer: meanErrorEnergy min value  = %c%d.%06d\r\n"
#define S_MAX "V90Equalizer: meanErrorEnergy max value  = %c%d.%06d\r\n"
#define S_MMXON "V90Equalizer: Created - MMX mode is enabled.\r\n"
#define S_MMXOFF "V90Equalizer: Created - MMX mode is disabled.\r\n"

/* ------------------------------------------------------------------------------------------------ helpers (static inline) */

#include "core/dsp_copyword_plain.h"

#include "core/dsp_floatbits.h"

#include "core/dsp_wordarg.h" /* wordOf / floatOf / WORDARG: pass a float argument as its raw 32-bit word (keeps NaN payloads of the parameter block intact) */

/** @brief Low 32 bits of a 64-bit truncating conversion; out-of-range and NaN give 0 (the 64-bit "integer indefinite" has a zero low word). */
static inline unsigned trunc64lo(LD v)
{
  if (!(v > -9223372036854775809.0L && v < 9223372036854775808.0L))
    return 0;
  return (unsigned)(unsigned long long)(long long)v;
}

#include "core/dsp_intmath.h"

#include "vpcm_print_helpers.h"

#include "core/dsp_x87_hide.h"

/** @brief The 32-bit value of a coefficient's two 16-bit halves: (hi << 16) | (unsigned short)lo, the low half zero-extended. */
static inline int join16(short hi, short lo) { return (int)(((unsigned)(int)hi << 16) | (unsigned)(unsigned short)lo); }

/**
 * @brief Float coefficient of a 16-bit high / low pair.
 * @return (1.0 / gain) * join16(hi, lo); the division and the product are evaluated in extended precision, rounded once to float
 */
static inline float unpackCoef(short hi, short lo, float gain)
{
  LD q = (LD)1.0L / (LD)gain;
  return (float)(q * (LD)join16(hi, lo));
}

/** @brief Split a float coefficient into 16-bit halves: v = chop(c * gain); *lo = (short)v, *hi = (short)(v >> 16). */
static inline void packCoef(float c, float gain, short* hi, short* lo)
{
  int v = dsp_trunc_i32((LD)c * (LD)gain);
  *lo = (short)v;
  *hi = (short)(v >> 16);
}

/**
 * @brief 16-bit LMS multiplier and shift for a step size (used by setLinearEquBeta() / setDfeBeta()).
 * @param b      step size
 * @param maxAbs largest |coefficient| at the time of the conversion
 * @param k      scale of the equalizer (2^24 linear, 2^20 DFE)
 * @param gain   float to 32-bit coefficient gain
 * @param mul    receives chop((b * gain) * 2^shift)
 * @param shift  receives chop(log10|maxAbs / (b * k)| / log10(2))
 * @note A step size of 0 or NaN gives mul = shift = 0.
 */
static inline void betaMmx(LD b, float maxAbs, LD k, float gain, int* mul, int* shift)
{
  if (b == 0.0L || b != b)
  { /* equal to zero, or unordered (NaN) */
    *shift = 0;
    *mul = 0;
  }
  else
  {
    LD t = (LD)maxAbs / (b * k);
    LD l = dsp_log10(__builtin_fabsl(t));
    LD l2 = dsp_log10((LD)2.0f);
    int sh = dsp_trunc_i32(l / l2);
    LD p = b * (LD)gain;
    int pw = (int)(1u << (sh & 31));
    *shift = sh;
    *mul = dsp_trunc_i32(p * (LD)pw);
  }
}

/** @brief Clamp a fade ratio to 0 .. 0.5 (NaN gives 0). */
static inline LD clampFade(float x)
{
  LD v = (LD)x;
  if (!(v >= 0.0L))
    return 0.0L;
  if (!(v <= 0.5L))
    return 0.5L;
  return v;
}

/**
 * @brief Set the fade lengths and rebuild both fade windows (used by reset() and setLinearEquEdgesFadingParams()).
 * @param left  fraction of the linear length faded at the leading edge, clamped to 0 .. 0.5
 * @param right fraction faded at the trailing edge, clamped to 0 .. 0.5
 * @note Each window is a hamming(2 * length) window; fadeWinA_ / fadeWinB_ must hold at least linLen_ entries.
 */
static inline void setFade(V90Equalizer* e, float left, float right)
{
  LD l = clampFade(left);
  LD r = clampFade(right);
  LD n = (LD)e->linLen_;
  unsigned la = trunc64lo(l * n);
  unsigned lb;
  e->fadeLenA_ = la;
  lb = trunc64lo(r * n);
  e->fadeLenB_ = lb;
  hamming<float>(e->fadeWinA_, la + la);
  hamming<float>(e->fadeWinB_, lb + lb);
}

/** @brief Zero the DFE float coefficients and, when `was16` is nonzero, the 16-bit DFE halves (dfeLen_ + 8 shorts: the whole allocated blocks). */
static inline void zeroDfe(V90Equalizer* e, unsigned was16)
{
  unsigned m = e->dfeLen_;
  float* c = e->dfeCoef_;
  for (unsigned k = 0; k < m; ++k)
    c[k] = 0.0f;
  if (was16 != 0)
  {
    unsigned n8 = e->dfeLen_ + 8;
    short* hi = e->rawDfeHi_;
    short* lo = e->rawDfeLo_;
    for (unsigned k = 0; k < n8; ++k)
    {
      hi[k] = 0;
      lo[k] = 0;
    }
  }
}

/**
 * @brief Common body of enterFPE() and enterRRN(): enter decision mode `mode` once.
 * @param mode the decision mode to enter (V90EQ_MODE_FPE or V90EQ_MODE_RRN)
 * @param text message printed on entry
 * @return 1 when the equalizer was converted from the 16-bit mode back to float, else 0 (also 0 when `mode` is already active)
 * @note With the spectral verifier flag at 2 (German PBX) the DFE is zeroed and set to the fast German PBX step size.  When the pre-filter reports V.90 with
 *       EIA6, both step sizes are set to 0.  A running 16-bit mode is always converted back to float.
 */
static inline int enterState(V90Equalizer* e, unsigned mode, const char* text)
{
  unsigned was16;
  bool eia6;
  if (e->mode_ == mode)
    return 0;

  edprintf(text);
  e->mode_ = mode;
  was16 = e->mmx_;
  if (e->sv_->flag_ == 2)
  {
    zeroDfe(e, was16);
    edprintf(S_DFEZERO);
    e->setDfeBeta(WORDARG(e->params_->germanPbxDfeTrn2dFastBeta));
    eia6 = e->pf_->isV90WithEia6();
  }
  else
  {
    eia6 = e->pf_->isV90WithEia6();
  }

  if (eia6)
  {
    e->setLinearEquBeta(0.0f);
    e->setDfeBeta(0.0f);
    edprintf(S_FREEZED);
    if (e->mmx_ == 0)
      return 0;
  }
  else if (e->mmx_ == 0)
  {
    return 0;
  }

  edprintf(S_RESTORING);
  e->restoreEqualizerToFloat();
  e->mmx_ = 0;
  return 1;
}

/** @brief Number of shorts (0 .. 3) to add to `p` to reach the next 8-byte boundary. */
static inline unsigned alignOff(void* p)
{
  unsigned long a = (unsigned long)p;
  return (unsigned)(((a + 7ul) & ~7ul) - a) >> 1;
}

/** @brief `p` advanced by `off` shorts (the aligned view of an allocated block). */
static inline short* alignPtr(void* p, unsigned off) { return (short*)((char*)p + off * 2u); }

/**
 * @brief Constructor body: store the collaborators, size and allocate the tables, then reset().
 * @param linLen  requested linear length, rounded down to a multiple of 4
 * @param dfeLen  requested DFE length, rounded down to a multiple of 4
 * @note The linear history holds an even number of floats derived from params->linearEquHistoryLength.  The 16-bit mode is allowed when
 *       params->enableEqualizerMmx is set and the modem parameter block word at +0x5c is not 2 (computational mode 1) or equals 1 (other modes).
 *       It ends with reset(linLen_ >> 1): the main tap of the identity equalizer sits in the middle.
 * @warning The allocation results are not checked.  The 16-bit blocks hold the length plus 8 shorts; the aligned views start up to 3 shorts into them.
 */
static inline void ctorBody(V90Equalizer* e, unsigned linLen, unsigned dfeLen, V90Phase3Demodulator* p3, V90Phase4Demodulator* p4, V90Demapper* dem,
                            V90ConnectionEvaluator* ce, V90SpectralVerifier* sv, V90Parameters* params, V90Resampler* resampler, V90PreFilter* pf,
                            V90ComputationalMode mode)
{
  unsigned n4 = linLen >> 2;
  unsigned d4 = dfeLen >> 2;
  int h;

  /* collaborators and the float tables */
  e->params_ = params;
  e->p3_ = p3;
  e->p4_ = p4;
  e->dem_ = dem;
  e->ce_ = ce;
  e->sv_ = sv;
  e->resampler_ = resampler;
  e->linLen_ = n4 * 4u;
  e->pf_ = pf;
  e->linCoef_ = (float*)sysdep_malloc(n4 << 4);
  h = e->params_->linearEquHistoryLength;
  h = (h + (int)((unsigned)h >> 31)) >> 1;
  e->linHistLen_ = (unsigned)h * 2u;
  e->linHist_ = (float*)sysdep_malloc((unsigned)h << 3);
  e->fadeWinA_ = (float*)sysdep_malloc(e->linLen_ << 2);
  e->fadeWinB_ = (float*)sysdep_malloc(e->linLen_ << 2);
  e->dfeLen_ = d4 * 4u;
  e->dfeCoef_ = (float*)sysdep_malloc(d4 << 4);
  e->dfeHist_ = (float*)sysdep_malloc(e->dfeLen_ << 2);

  /* is the 16-bit mode allowed? */
  e->mmxCapable_ = 0;
  if (e->params_->enableEqualizerMmx != 0)
  {
    const int* modemParams = &e->params_->params->w5c;
    if (mode == V90_COMPMODE_1)
    {
      if (*modemParams != 2)
        e->mmxCapable_ = 1;
    }
    else if (*modemParams == 1)
    {
      e->mmxCapable_ = 1;
    }
  }

  /* the 16-bit tables and their 8-byte aligned views */
  if (e->mmxCapable_ != 0)
  {
    e->mmxOut_ = (short*)sysdep_malloc(0x200);
    e->mmxIn_ = (short*)sysdep_malloc(0x400);
    e->rawLinHi_ = (short*)sysdep_malloc((e->linLen_ + 8u) * 2u);
    e->rawLinLo_ = (short*)sysdep_malloc((e->linLen_ + 8u) * 2u);
    e->rawHist16_ = (short*)sysdep_malloc((e->linHistLen_ + 8u) * 2u);
    e->rawDfeHi_ = (short*)sysdep_malloc((e->dfeLen_ + 8u) * 2u);
    e->rawDfeLo_ = (short*)sysdep_malloc((e->dfeLen_ + 8u) * 2u);
    e->rawDfeHist16_ = (short*)sysdep_malloc((e->dfeLen_ + 8u) * 2u);

    e->linHiOff_ = alignOff(e->rawLinHi_);
    e->linHi_ = alignPtr(e->rawLinHi_, e->linHiOff_);
    e->linLoOff_ = alignOff(e->rawLinLo_);
    e->linLo_ = alignPtr(e->rawLinLo_, e->linLoOff_);
    e->hist16Off_ = alignOff(e->rawHist16_);
    e->hist16_ = alignPtr(e->rawHist16_, e->hist16Off_);
    e->dfeHiOff_ = alignOff(e->rawDfeHi_);
    e->dfeHi_ = alignPtr(e->rawDfeHi_, e->dfeHiOff_);
    e->dfeLoOff_ = alignOff(e->rawDfeLo_);
    e->dfeLo_ = alignPtr(e->rawDfeLo_, e->dfeLoOff_);
    e->dfeHist16Off_ = alignOff(e->rawDfeHist16_);
    e->dfeHist16_ = alignPtr(e->rawDfeHist16_, e->dfeHist16Off_);
    edprintf(S_MMXON);
  }
  else
  {
    edprintf(S_MMXOFF);
  }

  e->mseHist_ = (float*)sysdep_malloc(0x4b0);
  e->reset(e->linLen_ >> 1);
}

/** @brief Free a block if it was allocated. */
static inline void freeBlock(void* p)
{
  if (p != 0)
    sysdep_free(p);
}

/** @brief Destructor body: free every non-NULL table (the 16-bit blocks only when they were allocated); no member is cleared. */
static inline void dtorBody(V90Equalizer* e)
{
  freeBlock(e->linCoef_);
  freeBlock(e->linHist_);
  freeBlock(e->dfeCoef_);
  freeBlock(e->dfeHist_);
  freeBlock(e->fadeWinA_);
  freeBlock(e->fadeWinB_);
  freeBlock(e->mseHist_);
  if (e->mmxCapable_ != 0)
  {
    freeBlock(e->mmxOut_);
    freeBlock(e->mmxIn_);
    freeBlock(e->rawLinHi_);
    freeBlock(e->rawLinLo_);
    freeBlock(e->rawHist16_);
    freeBlock(e->rawDfeHi_);
    freeBlock(e->rawDfeLo_);
    freeBlock(e->rawDfeHist16_);
  }
}

/**
 * @brief Table part of reset(): decision mode 0, identity linear equalizer, zero DFE, zero histories and (when allocated) zero 16-bit tables.
 * @param arg index of the unit coefficient of the linear equalizer, clamped to linLen_ - 1
 * @warning With linLen_ == 0 the clamp wraps and `linCoef_[arg]` is written out of range (the lengths come from the parameter block).
 *          linHistLen_ must be larger than linLen_, otherwise linPos_ wraps.
 */
static inline void resetTables(V90Equalizer* e, unsigned arg)
{
  unsigned h = e->linHistLen_;
  unsigned n = e->linLen_;
  unsigned m;
  e->mode_ = V90EQ_MODE_RESET;
  e->event_ = 0;
  e->linPos_ = h - n - 1u;

  /* linear equalizer: identity, empty history (the history is filled from the top down) */
  {
    float* c = e->linCoef_;
    float* hist = e->linHist_;
    for (unsigned i = 0; i < n; ++i)
    {
      c[i] = 0.0f;
      hist[h - i - 1u] = 0.0f;
    }
    if (n - 1u < arg)
      arg = n - 1u;
    c[arg] = 1.0f;
  }

  /* DFE: all zero */
  m = e->dfeLen_;
  {
    float* dh = e->dfeHist_;
    float* dc = e->dfeCoef_;
    for (unsigned i = 0; i < m; ++i)
    {
      dh[i] = 0.0f;
      dc[i] = 0.0f;
    }
  }

  /* the 16-bit tables (whole allocated blocks, length + 8) */
  if (e->mmxCapable_ != 0)
  {
    unsigned n8 = n + 8u;
    unsigned h8 = h + 8u;
    unsigned m8 = m + 8u;
    {
      short* a = e->rawLinHi_;
      short* b = e->rawLinLo_;
      for (unsigned i = 0; i < n8; ++i)
      {
        a[i] = 0;
        b[i] = 0;
      }
    }
    {
      short* a = e->rawHist16_;
      for (unsigned i = 0; i < h8; ++i)
        a[i] = 0;
    }
    {
      short* a = e->rawDfeHi_;
      short* b = e->rawDfeLo_;
      short* c = e->rawDfeHist16_;
      for (unsigned i = 0; i < m8; ++i)
      {
        a[i] = 0;
        b[i] = 0;
        c[i] = 0;
      }
    }
  }
}

/** @brief State part of reset(): clear the pending sample, statistics and counters, load the smoothing constant and block length, re-arm the studies. */
static inline void resetState(V90Equalizer* e)
{
  V90Parameters* p = e->params_;
  setWord(&e->pendSample_, 0u);
  setWord(&e->rmsErr_, 0u);
  setWord(&e->meanErr_, 0u);
  setWord(&e->statMean_, 0u);
  setWord(&e->statMin_, 0u);
  setWord(&e->statMax_, 0u);
  setWord(&e->stat13c_, 0u);
  setWord(&e->ratio_, 0u);
  copyWord(&e->errK_, &p->errorEnergyMeanK);
  e->pend_ = 0;
  e->errCnt_ = 0;
  e->errLimit_ = (unsigned)p->errorEnergyMeanBlockLen;
  e->mseIdx_ = 0;
  e->errSum_ = 0;
  e->mseFull_ = 0;
  e->mseEnable_ = 0;
  e->badCnt_ = 0;
  e->flag144_ = 1;
  e->flag146_ = 1;
  e->frameCnt_ = 0;
}

/** @brief Rebuild the float linear coefficients from the 16-bit halves (used by linearEquFadeEdges() and restoreEqualizerToFloat()). */
static inline void unpackLinear(V90Equalizer* e)
{
  unsigned n = e->linLen_;
  float* c = e->linCoef_;
  for (unsigned i = 0; i < n; ++i)
    c[i] = unpackCoef(e->linHi_[i], e->linLo_[i], e->linMmxGain_);
}

/** @brief Rebuild the float DFE coefficients from the 16-bit halves. */
static inline void unpackDfe(V90Equalizer* e)
{
  unsigned n = e->dfeLen_;
  float* c = e->dfeCoef_;
  for (unsigned i = 0; i < n; ++i)
    c[i] = unpackCoef(e->dfeHi_[i], e->dfeLo_[i], e->dfeMmxGain_);
}

/** @brief Print block of calcMeanErrorStatistics(): history length, mean, current value, standard deviation, variance, minimum and maximum. */
static inline void printStats(V90Equalizer* e, unsigned n, float s, float v)
{
  edprintf(S_HASH);
  edprintf(S_DEBUG);
  edprintf(S_OVER, n);
  edprintf(S_DASH);
  printFloat(S_MEAN, e->statMean_, (LD)1000000.0);
  printFloat(S_CUR, e->meanErr_, (LD)1000000.0);
  edprintf(S_DASH);
  printFloat(S_STD, s, (LD)1000000.0);
  printFloat(S_VAR, v, (LD)1000000.0);
  edprintf(S_DASH);
  printFloat(S_MIN, e->statMin_, (LD)1000000.0);
  printFloat(S_MAX, e->statMax_, (LD)1000000.0);
  edprintf(S_HASH);
}

/* ------------------------------------------------------------------------------------------------ the methods */

V90Equalizer::V90Equalizer(unsigned linLen, unsigned dfeLen, V90Phase3Demodulator* p3, V90Phase4Demodulator* p4, V90Demapper* dem, V90ConnectionEvaluator* ce,
                           V90SpectralVerifier* sv, V90Parameters* params, V90Resampler* resampler, V90PreFilter* pf, V90ComputationalMode mode)
{
  ctorBody(this, linLen, dfeLen, p3, p4, dem, ce, sv, params, resampler, pf, mode);
}

V90Equalizer::~V90Equalizer() { dtorBody(this); }

/**
 * @brief Return the equalizer to its initial state (see the declaration).
 * @param arg index of the unit coefficient of the linear equalizer
 * @note Leaves the 16-bit mode (mmx_ = 0) without converting anything: the tables are cleared anyway.  The default step sizes stored first
 *       (0x283424dc, about 1.0e-14) are replaced by 0 through the two setters, which print the change.
 */
void V90Equalizer::reset(unsigned arg)
{
  edprintf(S_RESET);
  w8_ = 0;
  setWord(&linBeta_, 0x283424dcu);
  setWord(&dfeBeta_, 0x283424dcu);
  mmx_ = 0;
  setWord(&linCoefMax_, 0u);
  setWord(&linCoefMin_, 0u);
  setWord(&dfeCoefMax_, 0u);
  setWord(&dfeCoefMin_, 0u);
  setLinearEquBeta(0.0f);
  setDfeBeta(0.0f);

  resetTables(this, arg);
  resetState(this);
  setFade(this, params_->linearEquFadeLeftEdgeRatio, params_->linearEquFadeRightEdgeRatio);
}

void V90Equalizer::resetMeanErrorEnergyDiagnostics()
{
  mseIdx_ = 0;
  mseFull_ = 0;
}

/**
 * @note Prints the new value (scaled by 1e10) only when it differs from the old one.  In the 16-bit mode the multiplier and shift are recomputed from the
 *       linear coefficient maximum recorded at the conversion.
 */
void V90Equalizer::setLinearEquBeta(float beta)
{
  if (linBeta_ < beta || linBeta_ > beta) /* a change: equal or unordered values skip the print */
    printFix(S_LEBETA, (LD)10000000000.0f * (LD)beta, (LD)beta, (LD)100000.0f);
  linBeta_ = beta;
  if (mmx_ != 0)
    betaMmx((LD)beta, linCoefMax_, (LD)16777216.0f, linMmxGain_, &linMul_, &linShift_);
}

/** @note Same as setLinearEquBeta() for the DFE (print scaled by 1e7, equalizer scale 2^20). */
void V90Equalizer::setDfeBeta(float beta)
{
  if (dfeBeta_ < beta || dfeBeta_ > beta)
    printFix(S_DFEBETA, (LD)10000000.0f * (LD)beta, (LD)beta, (LD)100000.0f);
  dfeBeta_ = beta;
  if (mmx_ != 0)
    betaMmx((LD)beta, dfeCoefMax_, (LD)1048576.0f, dfeMmxGain_, &dfeMul_, &dfeShift_);
}

void V90Equalizer::freeze()
{
  setLinearEquBeta(0.0f);
  setDfeBeta(0.0f);
}

void V90Equalizer::enterPhase3()
{
  if (mode_ == V90EQ_MODE_PHASE3)
    return;

  edprintf(S_ENTER3);
  setLinearEquBeta(0.0f);
  setDfeBeta(0.0f);
  mode_ = V90EQ_MODE_PHASE3;
  event_ = 0;
}

float V90Equalizer::getDfeBeta() { return dfeBeta_; }

void V90Equalizer::zeroLinearEquCoefs()
{
  unsigned n = linLen_;
  float* c = linCoef_;
  for (unsigned i = 0; i < n; ++i)
    c[i] = 0.0f;
  if (mmx_ != 0)
  {
    unsigned n8 = n + 8u;
    short* hi = rawLinHi_;
    short* lo = rawLinLo_;
    for (unsigned i = 0; i < n8; ++i)
    {
      hi[i] = 0;
      lo[i] = 0;
    }
  }
}

void V90Equalizer::zeroDfeCoefs()
{
  unsigned n = dfeLen_;
  float* c = dfeCoef_;
  for (unsigned i = 0; i < n; ++i)
    c[i] = 0.0f;
  if (mmx_ != 0)
  {
    unsigned n8 = n + 8u;
    short* hi = rawDfeHi_;
    short* lo = rawDfeLo_;
    for (unsigned i = 0; i < n8; ++i)
    {
      hi[i] = 0;
      lo[i] = 0;
    }
  }
}

/** @warning Does not check n against linLen_ and does not update the 16-bit halves. */
void V90Equalizer::setLinearEquCoeff(float* c, unsigned n)
{
  float* d = linCoef_;
  for (unsigned i = 0; i < n; ++i)
    copyWord(&d[i], &c[i]);
}

/** @warning Does not check n against dfeLen_ and does not update the 16-bit halves. */
void V90Equalizer::setDfeCoeff(float* c, unsigned n)
{
  float* d = dfeCoef_;
  for (unsigned i = 0; i < n; ++i)
    copyWord(&d[i], &c[i]);
}

void V90Equalizer::loadCoefsFromFile() {}

/**
 * @note In the 16-bit mode both equalizers are first rebuilt in float (the DFE too, although only the linear one is changed), the first fadeLenA_
 *       coefficients are multiplied by fadeWinA_ and the last fadeLenB_ ones by fadeWinB_ (in reverse order), and only the linear equalizer is converted
 *       back to 16-bit halves.  The DFE keeps its 16-bit halves and its float rebuild is unused afterwards.
 */
void V90Equalizer::linearEquFadeEdges()
{
  unsigned m16 = mmx_;
  short* hi = linHi_;
  short* lo = linLo_;
  if (m16 != 0)
  {
    unpackLinear(this);
    unpackDfe(this);
  }

  /* leading edge */
  {
    unsigned la = fadeLenA_;
    const float* w = fadeWinA_;
    float* c = linCoef_;
    for (unsigned i = 0; i < la; ++i)
      c[i] = mulF(w[i], c[i]);
  }

  /* trailing edge */
  {
    unsigned lb = fadeLenB_;
    unsigned n = linLen_;
    const float* w = fadeWinB_;
    float* c = linCoef_;
    for (unsigned i = 0; i < lb; ++i)
      c[n - i - 1u] = mulF(w[i], c[n - i - 1u]);
  }

  if (m16 != 0)
  {
    unsigned n = linLen_;
    float g = linMmxGain_;
    float* c = linCoef_;
    for (unsigned i = 0; i < n; ++i)
      packCoef(c[i], g, &hi[i], &lo[i]);
  }
}

void V90Equalizer::printCoefsToFile() const {}

/**
 * @note Rebuilds the float coefficients of both equalizers and the float histories from the 16-bit ones.  The float linear history position takes
 *       histPos16_ even when the history is empty.  The history samples are 16-bit values converted to float.
 */
void V90Equalizer::restoreEqualizerToFloat()
{
  if (mmx_ == 0)
    return;

  unpackLinear(this);
  {
    unsigned n = linHistLen_;
    float* d = linHist_;
    const short* s = hist16_;
    for (unsigned i = 0; i < n; ++i)
      d[i] = (float)s[i];
  }
  linPos_ = histPos16_;

  {
    unsigned n = dfeLen_;
    float* c = dfeCoef_;
    float* dh = dfeHist_;
    for (unsigned i = 0; i < n; ++i)
    {
      c[i] = unpackCoef(dfeHi_[i], dfeLo_[i], dfeMmxGain_);
      dh[i] = (float)dfeHist16_[i];
    }
  }
  edprintf(S_RESTORED);
  mmx_ = 0;
}

int V90Equalizer::enterFPE() { return enterState(this, V90EQ_MODE_FPE, S_ENTERFPE); }

int V90Equalizer::enterRRN() { return enterState(this, V90EQ_MODE_RRN, S_ENTERRRN); }

void V90Equalizer::enterChannelVerification()
{
  if (mode_ == V90EQ_MODE_CHANNEL_VERIFICATION)
    return;

  edprintf(S_ENTERCV);
  setLinearEquBeta(0.0f);
  setDfeBeta(0.0f);
  mode_ = V90EQ_MODE_CHANNEL_VERIFICATION;
  event_ = 0;
  resampler_->setBllState(V90BLL_PRE_ANSPCM, 1);
}

void V90Equalizer::printEquStuff() {}

/**
 * @note The history holds mseIdx_ values, or all 300 once it has wrapped (mseFull_).  Mean, standard deviation and variance come from the shared statistics
 *       templates; the minimum and maximum are scanned here.  A NaN in the history is skipped by the maximum scan and replaces the minimum.
 */
float V90Equalizer::calcMeanErrorStatistics()
{
  unsigned n;
  float s, v;

  /* number of valid history entries */
  if (mseIdx_ != 0)
  {
    n = mseIdx_;
    if (mseFull_ != 0)
      n = 300;
  }
  else
  {
    if (mseFull_ == 0)
      return 0.0f; /* no history: nothing is computed or printed */
    n = 300;
  }

  statMean_ = mean<float>(mseHist_, n);
  s = Std<float>(mseHist_, n);
  v = Var<float>(mseHist_, n);

  /* extremes */
  {
    const float* h = mseHist_;
    float mx = h[0];
    float mn = mx;
    statMax_ = mx;
    statMin_ = mx;
    for (unsigned i = 1; i < n; ++i)
    {
      float a = h[i];
      if (a > mx)
      {
        mx = a;
        statMax_ = a;
      } /* a value that is not greater (or is NaN) keeps the maximum */
      if (!(a >= mn))
      {
        mn = a;
        statMin_ = a;
      } /* only a value that is >= the minimum keeps it; a smaller value or NaN replaces it */
    }
  }

  printStats(this, n, s, v);
  return s;
}

void V90Equalizer::setLinearEquEdgesFadingParams(float left, float right) { setFade(this, left, right); }
