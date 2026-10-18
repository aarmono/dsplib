/**
 * @file v90_equalizer_phases.cpp
 * @brief V90Equalizer: entry into phase 4 and the data phase, and the conversion of the equalizer to the 16-bit ("MMX") data format.
 *
 * Three methods of V90Equalizer (class declared in v90_equalizer.h).  The callers are V90Equalizer::process() (enterDataPhase() and, through it,
 * convertEqualizerToMmx()) and V90Demodulator::exitPhase3() (enterPhase4()).  The name "MMX" is only a data format: the code uses no vector instructions,
 * it is plain 16-bit integer arithmetic.  Callees in other files: V90Equalizer::setLinearEquBeta() / setDfeBeta() (v90_equalizer.cpp),
 * V90Resampler::setBllState() and getTimingOffsetPPM_LD(), V90Phase4Demodulator::resetRRNDetector(), edprintf().
 *
 * - enterPhase4(): once only (mode_ != 2).  Sets mode_ = 2, freezes both step sizes, saves the resampler's BLL state in bllSaved_ and freezes the
 *   timing loop, prints the timing offset at the freeze, zeroes the DFE (and its 16-bit halves) when the spectral verifier flag is 2 (the German PBX case),
 *   restarts the mean-error study (mseFull_ = mseIdx_ = 0) and prints the coefficient report.
 * - enterDataPhase(): once only (mode_ != 3).  Sets mode_ = 3 and resets the phase-4 RRN detector.  In float mode (mmx_ == 0) it prints the same
 *   coefficient report as enterPhase4() and converts the equalizer to the 16-bit mode; it returns 1 when the equalizer is in the 16-bit mode afterwards and
 *   was in float mode on entry.
 * - convertEqualizerToMmx(): unless the 16-bit mode is already active or not allowed (mmxCapable_, params->enableEqualizerMmx) it converts, first the
 *   linear and then the DFE equalizer: gain G = 2^30 / max|coefficient| (kept extended, stored as a float in linMmxGain_ / dfeMmxGain_), output divisor
 *   G / 2^16, LMS multiplier and shift (a power of two chosen from a log10 / log10(2) quotient), the coefficients G * c as 32-bit integers split into
 *   16-bit low and high halves, statistics of the result (sums, extremes; all printed) and the history converted to 16-bit shorts (statistics printed).
 *   Finally mmx_ = 1.  The history position handover histPos16_ = linPos_ happens between the linear and the DFE part.
 *
 * Numeric conventions.  Float slots that are updated term by term (the report sums) are volatile float, so every term is rounded to float as it is
 * stored; intermediate products are evaluated in extended precision (`long double`, LD).  Printed values go through the helpers of vpcm_print_helpers.h:
 * the sign character is '+' when 0 < v or v is NaN, otherwise '-'; the integer part is chopped from |v| and the fraction is
 * abs(chop((v - chop(v)) * scale)).  The conversion-factor print takes its sign from the float gain but its digits from the unrounded product 1e-8 * gain.
 * The 16-bit history extremes track abs() of the short, so -32768 stays negative.
 *
 * Ordering requirement.  The order of the four getTimingOffsetPPM calls in enterPhase4() and the position of every store relative to a print are part of
 * the observable behaviour (test harnesses snapshot the whole object at every print).
 */

#include "v90_equalizer.h"
#include "core/dsp_math.h"
#include "v34/v34_api.h"

extern "C"
{
}

typedef long double LD;

/* ------------------------------------------------------------------------------------------------ message strings (debug output through edprintf) */
#define S_TOFF "V90Equalizer: timing offset on freeze (phase4) = %c%d.%04d\r\n"
#define S_BAR55 "=======================================================\r\n"
#define S_MINLE "minLeCoefValue  = %c%d.%010d\r\n"
#define S_MINDFE "minDfeCoefValue  = %c%d.%010d\r\n"
#define S_MAXDFE "maxDfeCoefValue  = %c%d.%06d\r\n"
#define S_DFEZERO "V90Equalizer: Dfe coefs zeroed!!\r\n"
#define S_FLOAT "V90Equalizer: LE & DFE running in FLOAT mode.\r\n"
#define S_BAR62 "==============================================================\r\n"
#define S_LEGAIN "V90Equalizer: linearEquMmxConversionFactor = %c%d.%05de8\r\n"
#define S_LEOUT "V90Equalizer: linearEquMmxOutputConversionFactor = %c%d.%03d\r\n"
#define S_LEBETA "V90Equalizer: linearEquBetaMmx = %d, beta exponent = %d\r\n"
#define S_LEFSUM "V90Equalizer: float LE coeffs sum = %c%d.%04d\r\n"
#define S_LEFABS "V90Equalizer: float LE coeffs abs sum = %c%d.%04d\r\n"
#define S_LEHSUM "V90Equalizer: short high LE coeffs sum = %d\r\n"
#define S_LEHABS "V90Equalizer: short high LE coeffs abs sum = %d\r\n"
#define S_LEHMIN "V90Equalizer: short high LE coeffs min value = %d\r\n"
#define S_LEHMAX "V90Equalizer: short high LE coeffs max value = %d\r\n"
#define S_LEHISTMAX "V90Equalizer: Max LE History = %d\r\n"
#define S_LEHISTMIN "V90Equalizer: Min LE History = %d\r\n"
#define S_DFEGAIN "V90Equalizer: dfeMmxConversionFactor = %c%d.%05de8\r\n"
#define S_DFEOUT "V90Equalizer: dfeMmxOutputConversionFactor = %c%d.%03d\r\n"
#define S_DFEBETA "V90Equalizer: dfeBetaMmx = %d, beta exponent = %d\r\n"
#define S_DFEFSUM "V90Equalizer: float dfe coeffs sum = %c%d.%04d\r\n"
#define S_DFEFABS "V90Equalizer: float dfe coeffs abs sum = %c%d.%04d\r\n"
#define S_DFEHSUM "V90Equalizer: short high dfe coeffs sum = %d\r\n"
#define S_DFEHABS "V90Equalizer: short high dfe coeffs abs sum = %d\r\n"
#define S_DFEHMIN "V90Equalizer: short high dfe coeffs min value = %d\r\n"
#define S_DFEHMAX "V90Equalizer: short high dfe coeffs max value = %d\r\n"
#define S_DFEHISTMAX "V90Equalizer: Max dfe History = %d\r\n"
#define S_DFEHISTMIN "V90Equalizer: Min dfe History = %d\r\n"
#define S_MMX "V90Equalizer: LE & DFE running in MMX mode.\r\n"
#define S_ENTERDATA "V90Equalizer: enter Data Phase\r\n"
#define S_ENTER4 "V90Equalizer: enter Phase 4\r\n"
#define S_LE "Linear Equalizer:\r\n"
#define S_MAXLE "maxLeCoefValue  = %c%d.%06d\r\n"
#define S_SUM "coefs sum  = %c%d.%06d\r\n"
#define S_ABS "abs coefs sum  = %c%d.%06d\r\n"
#define S_DFE "DFE:\r\n"

/* ------------------------------------------------------------------------------------------------ helpers (static inline) */

#include "core/dsp_floatbits.h"

/** @brief |v| evaluated in extended precision (a signalling NaN is quieted, the sign is cleared). */
static inline LD absF(float v) { return __builtin_fabsl((LD)v); }

#include "core/dsp_intmath.h"

#include "vpcm_print_helpers.h"

#define K1E6 ((LD)1000000.0)       /* print scale 1e6 */
#define K1E10 ((LD)10000000000.0f) /* print scale 1e10 (small coefficient minima) */

/**
 * @brief Scan one coefficient table for the coefficient report.
 * @param c    float coefficients
 * @param n    number of coefficients (at least 1; element 0 is always read)
 * @param mx   receives the largest |c|
 * @param mn   receives the smallest |c|
 * @param sum  receives the plain sum of c[1 .. n-1]
 * @param asum receives the sum of |c[1 .. n-1]|
 * @note The extremes start at |c[0]|, but the two sums start at element 1 (element 0 is not included).  The sums are rounded to float after every term.
 *       A NaN never replaces the maximum and always replaces the minimum.
 */
static inline void scanCoefs(const float* c, unsigned n, float* mx, float* mn, float* sum, float* asum)
{
  volatile float S = 0.0f, A = 0.0f; /* volatile: float accumulators, rounded to float on every update */
  float a0 = storeF(mx, absF(c[0]));
  float mxv = a0, mnv = a0;
  *mn = a0;
  if (n > 1)
  {
    const float* p = c + 1;
    for (unsigned k = n - 1; k != 0; --k)
    {
      float a = (float)absF(*p);
      if (a > mxv)
      {
        mxv = a;
        *mx = a;
      }
      if (!(a >= mnv))
      {
        mnv = a;
        *mn = a;
      }
      S = (float)((LD)S + (LD)*p);
      A = (float)(absF(*p) + (LD)A);
      ++p;
    }
  }
  *sum = S;
  *asum = A;
}

/**
 * @brief Print the coefficient report of enterPhase4() / enterDataPhase().
 * @note Scans the linear equalizer (recording linCoefMax_ / linCoefMin_) and prints its minimum, maximum, sum and absolute sum; the DFE scan (dfeCoefMax_ /
 *       dfeCoefMin_) runs after the first block of prints.  The recorded maxima are the ones convertEqualizerToMmx() uses for its gains.  In the 16-bit mode
 *       the float tables are stale, so the report then describes the last float state.
 */
static inline void coefReport(V90Equalizer* e)
{
  float s, a;
  scanCoefs(e->linCoef_, e->linLen_, &e->linCoefMax_, &e->linCoefMin_, &s, &a);
  edprintf(S_BAR55);
  edprintf(S_LE);
  printFloat(S_MINLE, e->linCoefMin_, K1E10);
  printFloat(S_MAXLE, e->linCoefMax_, K1E6);
  printFloat(S_SUM, s, K1E6);
  printFloat(S_ABS, a, K1E6);
  edprintf(S_BAR55);

  scanCoefs(e->dfeCoef_, e->dfeLen_, &e->dfeCoefMax_, &e->dfeCoefMin_, &s, &a);
  edprintf(S_DFE);
  printFloat(S_MINDFE, e->dfeCoefMin_, K1E10);
  printFloat(S_MAXDFE, e->dfeCoefMax_, K1E6);
  printFloat(S_SUM, s, K1E6);
  printFloat(S_ABS, a, K1E6);
  edprintf(S_BAR55);
}

/**
 * @brief Compute the gain, divisor, LMS multiplier and shift of one equalizer and convert its coefficients to 16-bit halves.
 * @param coef     float coefficients, n entries
 * @param hi       receives the high halves of the 32-bit coefficients
 * @param lo       receives the low halves
 * @param maxAbs   largest |coefficient| (from the coefficient report)
 * @param beta     current LMS step size
 * @param k        equalizer scale (linear: 2^24, DFE: 2^20)
 * @param gainSlot receives the gain G = 2^30 / maxAbs as a float
 * @param div      receives chop(G / 2^16), the divisor of the 16-bit sum
 * @param mul      receives the LMS multiplier chop((beta * G) * 2^shift); 0 when the step size is 0 or NaN
 * @param shift    receives chop(log10|maxAbs / (beta * k)| / log10(2)); 0 when the step size is 0 or NaN
 * @note The gain stays an extended value inside the function (only its float copy is stored); each coefficient is chop(c * G), low half = (short)v, high
 *       half = (short)(v >> 16).
 * @warning A zero maxAbs gives an infinite gain; the integer conversions then produce the "integer indefinite" value and a later division by `div`
 *          is by zero if G / 2^16 truncates to 0 (maxAbs above 2^14).
 */
static inline void convertCoefs(const float* coef, unsigned n, short* hi, short* lo, float maxAbs, float beta, LD k, float* gainSlot, int* div, int* mul,
                                int* shift)
{
  LD m = (LD)maxAbs;
  LD g = ((LD)1.0L / m) * (LD)1073741824.0f; /* 1 / max * 2^30 */
  LD h = (LD)1.52587890625e-05f * g;         /* * 2^-16 */
  LD b = (LD)beta;
  storeF(gainSlot, g);
  *div = dsp_trunc_i32(h);

  if (b == 0.0L || b != b)
  {
    *shift = 0;
    *mul = 0;
  }
  else
  {
    LD t5 = (LD)1.0L / (b * k);
    LD t6 = t5 * m;
    LD t8 = dsp_log10(__builtin_fabsl(t6));
    LD t9 = dsp_log10((LD)2.0f);
    LD t10 = t8 / t9;
    LD t11 = b * g;
    int sh = dsp_trunc_i32(t10);
    int pw = (int)(1u << (sh & 31));
    *shift = sh;
    *mul = dsp_trunc_i32(t11 * (LD)pw);
  }

  for (unsigned i = 0; i < n; ++i)
  {
    int v = dsp_trunc_i32((LD)coef[i] * g);
    lo[i] = (short)v;
    hi[i] = (short)(v >> 16);
  }
}

/** @brief Statistics of a converted equalizer: float sums of the coefficients and integer sum / absolute sum / minimum / maximum of the high halves. */
typedef struct
{
  float fsum, fabs_;          ///< float sum of the coefficients and of their absolute values (all elements)
  int isum, iabs, imin, imax; ///< integer sum, absolute sum, minimum and maximum |value| of the high halves
} ConvStats;

/**
 * @brief Statistics of the converted high halves.
 * @param coef float coefficients (summed from element 0, unlike the coefficient report)
 * @param hi   high halves, n entries
 * @param s    receives the statistics; imin starts at 0x10000 and imax at 0
 */
static inline void statsHigh(const float* coef, const short* hi, unsigned n, ConvStats* s)
{
  volatile float S = 0.0f, A = 0.0f;
  int isum = 0, iabs = 0, imax = 0, imin = 0x10000;
  for (unsigned i = 0; i < n; ++i)
  {
    int h = (int)hi[i];
    int a;
    isum += h;
    a = absI32(h);
    if (a > imax)
      imax = a;
    if (a < imin)
      imin = a;
    iabs += a;
    S = (float)((LD)S + (LD)coef[i]);
    A = (float)(absF(coef[i]) + (LD)A);
  }
  s->fsum = S;
  s->fabs_ = A;
  s->isum = isum;
  s->iabs = iabs;
  s->imin = imin;
  s->imax = imax;
}

#include "core/dsp_x87_conv_int.h"

/**
 * @brief Convert a float history to 16-bit shorts and record the extremes of |value|.
 * @param src float history, n entries
 * @param dst receives the shorts (truncated towards zero, out-of-range and NaN give the 16-bit "integer indefinite")
 * @param mx  receives the largest |value| of the shorts
 * @param mn  receives the smallest |value| of the shorts (starts at 0x8000)
 */
static inline void convertHist(const float* src, short* dst, unsigned n, int* mx, int* mn)
{
  int imax = 0, imin = 0x8000;
  for (unsigned i = 0; i < n; ++i)
  {
    int s = trunc16((LD)src[i]);
    int a;
    dst[i] = (short)s;
    a = (int)(short)absI32(s);
    if (a > imax)
      imax = a;
    if (a < imin)
      imin = a;
  }
  *mx = imax;
  *mn = imin;
}

/* ------------------------------------------------------------------------------------------------ V90Equalizer::enterPhase4 */
void V90Equalizer::enterPhase4()
{
  if (mode_ == V90EQ_MODE_PHASE4)
    return;

  edprintf(S_ENTER4);
  mode_ = V90EQ_MODE_PHASE4;
  setLinearEquBeta(0.0f);
  setDfeBeta(0.0f);

  /* remember the BLL state and freeze the timing loop */
  {
    V90Resampler* r = resampler_;
    bllSaved_ = r->state_;
    r->setBllState(V90BLL_FROZEN, 1);
  }

  /* print the timing offset at the freeze: the resampler is asked four times; the value of the first call is rounded to float, the others stay extended */
  {
    int ip;
    int frac;
    int ipAbs;
    int sgn;
    volatile float tmp; /* float slot: rounded at every store */
    tmp = (float)resampler_->getTimingOffsetPPM_LD();
    ip = dsp_trunc_i32(resampler_->getTimingOffsetPPM_LD());
    tmp = (float)((LD)tmp - (LD)ip);
    frac = absI32(dsp_trunc_i32((LD)10000.0f * (LD)tmp));
    ipAbs = dsp_trunc_i32(__builtin_fabsl(resampler_->getTimingOffsetPPM_LD()));
    sgn = signChar(resampler_->getTimingOffsetPPM_LD());
    edprintf(S_TOFF, sgn, ipAbs, frac);
  }

  /* German PBX: start phase 4 with a zero DFE */
  if (sv_->flag_ == 2)
  {
    unsigned m = dfeLen_;
    for (unsigned k = 0; k < m; ++k)
      ((unsigned*)dfeCoef_)[k] = 0;
    if (mmx_ != 0)
    {
      for (unsigned k = 0; k < m + 8; ++k)
      {
        rawDfeHi_[k] = 0;
        rawDfeLo_[k] = 0;
      }
    }
    edprintf(S_DFEZERO);
  }

  /* restart the mean-error study and report the coefficients */
  mseFull_ = 0;
  mseIdx_ = 0;
  coefReport(this);
}

/* ------------------------------------------------------------------------------------------------ V90Equalizer::convertEqualizerToMmx */
void V90Equalizer::convertEqualizerToMmx()
{
  ConvStats cs;
  int hmx, hmn;
  if (mmx_ != 0 || mmxCapable_ == 0 || params_->enableEqualizerMmx == 0)
  {
    edprintf(S_FLOAT);
    mmx_ = 0;
    return;
  }

  /* the linear equalizer */
  convertCoefs(linCoef_, linLen_, linHi_, linLo_, linCoefMax_, linBeta_, (LD)16777216.0f, &linMmxGain_, &linDiv_, &linMul_, &linShift_);
  statsHigh(linCoef_, linHi_, linLen_, &cs);
  edprintf(S_BAR62);
  printFix(S_LEGAIN, (LD)(double)1e-8 * (LD)linMmxGain_, (LD)linMmxGain_, (LD)100000.0f);
  edprintf(S_LEOUT, signChar((LD)linDiv_), dsp_trunc_i32(__builtin_fabsl((LD)linDiv_)), 0);
  edprintf(S_LEBETA, linMul_, linShift_);
  edprintf(S_BAR62);
  printFloat(S_LEFSUM, cs.fsum, (LD)10000.0f);
  printFloat(S_LEFABS, cs.fabs_, (LD)10000.0f);
  edprintf(S_LEHSUM, cs.isum);
  edprintf(S_LEHABS, cs.iabs);
  edprintf(S_LEHMIN, cs.imin);
  edprintf(S_LEHMAX, cs.imax);
  convertHist(linHist_, hist16_, linHistLen_, &hmx, &hmn);
  edprintf(S_LEHISTMAX, hmx);
  edprintf(S_LEHISTMIN, hmn);

  /* the DFE (the position handover histPos16_ = linPos_ sits between the two parts) */
  histPos16_ = linPos_;
  convertCoefs(dfeCoef_, dfeLen_, dfeHi_, dfeLo_, dfeCoefMax_, dfeBeta_, (LD)1048576.0f, &dfeMmxGain_, &dfeDiv_, &dfeMul_, &dfeShift_);
  statsHigh(dfeCoef_, dfeHi_, dfeLen_, &cs);
  edprintf(S_BAR62);
  printFix(S_DFEGAIN, (LD)(double)1e-8 * (LD)dfeMmxGain_, (LD)dfeMmxGain_, (LD)100000.0f);
  edprintf(S_DFEOUT, signChar((LD)dfeDiv_), dsp_trunc_i32(__builtin_fabsl((LD)dfeDiv_)), 0);
  edprintf(S_DFEBETA, dfeMul_, dfeShift_);
  edprintf(S_BAR62);
  printFloat(S_DFEFSUM, cs.fsum, (LD)10000.0f);
  printFloat(S_DFEFABS, cs.fabs_, (LD)10000.0f);
  edprintf(S_DFEHSUM, cs.isum);
  edprintf(S_DFEHABS, cs.iabs);
  edprintf(S_DFEHMIN, cs.imin);
  edprintf(S_DFEHMAX, cs.imax);
  convertHist(dfeHist_, dfeHist16_, dfeLen_, &hmx, &hmn);
  edprintf(S_DFEHISTMAX, hmx);
  edprintf(S_DFEHISTMIN, hmn);

  mmx_ = 1;
  edprintf(S_MMX);
}

/* ------------------------------------------------------------------------------------------------ V90Equalizer::enterDataPhase */
int V90Equalizer::enterDataPhase()
{
  if (mode_ == V90EQ_MODE_DATA)
    return 0;

  edprintf(S_ENTERDATA);
  mode_ = V90EQ_MODE_DATA;
  p4_->resetRRNDetector();
  if (mmx_ != 0)
    return 0;

  coefReport(this);
  convertEqualizerToMmx();
  return mmx_ != 0 ? 1 : 0;
}
