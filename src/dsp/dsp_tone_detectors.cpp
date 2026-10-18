/**
 * @file dsp_tone_detectors.cpp
 * @brief GenericToneDetector and ANSamToneDetector (declarations in dsp_tone_detectors.h) and the 2100 Hz band-pass coefficient tables.
 *
 * Detection scheme.  Every sample goes through the GenericIIR; the energy of the input x and of the filtered output y is summed over `period` samples.
 * At the end of a period the mean powers XR, YR are formed, the smoothed powers avgX2_ / avgY2_ are updated (0.3 new + 0.7 old), and the period counts as a
 * HIT when YR >= thresh_ and (with the ratio test on) the smoothed filtered power exceeds the smoothed input power times `ratio_`; otherwise as a MISS.
 * A hit increments `hits_` and clears `misses_`; a miss increments `misses_` and, once it reaches `maxMiss_`, clears `hits_` and the detection flag.
 * The flag is set when `hits_` reaches `needHits_`.  Example: the V.90 receivers use minSamples 400, maxGapSamples 100, period 50, i.e. 8 hits to
 * detect and 2 consecutive misses to drop the detection.
 *
 * Numerics.  Values are float32 in memory; everything that is combined within one period decision is computed in extended precision (`long double`):
 * x*x + sum, y*y + sum (y is an exact float, so y*y is exact; the sum is rounded to 64 bits), the reciprocal 1.0f / N, the products with the float constants
 * 0.3f / 0.7f, the two smoothed powers (the TESTS use the unrounded extended values; only the stores into avgX2_ / avgY2_ round to float), and the products
 * YR / XR / V14 * ratio.  The float arguments of the constructors are moved as 32-bit words (storeArgWord), the float input sample is converted by the x87.
 *
 * Behaviour notes.
 *  - The two process() functions DISAGREE.  The block version has a hysteresis path (YR below thresh_ but at least thresh_ / 2, with the ratio test on and
 *    the smoothed filtered power above 0.85 times the smoothed input power, still counts as a hit); the single-sample version has no such path.  They also
 *    differ in what a plain miss does afterwards (whether the miss joins the detected_ test) and in the operand order of the comparisons.  NaN data: every
 *    comparison is the IEEE one (unordered = "below" in the block version's tests, "not above-or-equal" in the single version's).
 *  - The constructor stores the ceil'd run lengths with an unsigned division by `period` (0 faults); the sysdep_malloc() result of the IIR object is never
 *    tested (the GenericIIR constructor then runs on a NULL object); the destructor frees only a non-NULL iir_.
 *  - needHits_ == 0 makes the detection test `hits_ >= 0` true on every period that reaches it.
 *  - The IIR is a HEAP object: sysdep_malloc(sizeof(GenericIIR)) plus placement new.
 *  - The second parameter of process(float*, unsigned) is only the count.
 */

#include "dsp_tone_detectors.h"
#include "core/dsp_asm.h"

typedef long double LD;

#include "core/dsp_wordof_value.h"
#include "core/dsp_floatbits.h"

/* ------------------------------------------------------------------------------------------- the ANSam coefficient tables */
/* The 2100 Hz band-pass (the answer tone ANSam) for sample rates of 8000 Hz (13 coefficients) and 9600 Hz (11 coefficients): numerator B, denominator A.
 * Referenced only by the ANSam constructor; the pointers escape into the borrowed den_ / num_ fields of the heap GenericIIR (read only, never written,
 * never freed), so the tables are plain static arrays. */
static double IIR2100_Coef_B_9600[11] = {2.312202029234315e-09,  0.0, -1.156101014617157e-08, 0.0, 2.312202029234314e-08, 0.0,
                                         -2.312202029234314e-08, 0.0, 1.156101014617157e-08,  0.0, -2.312202029234315e-09};

static double IIR2100_Coef_B_8000[13] = {0.002408242450883259, 0.001949855816940553,  0.01166368929396639,  0.007618952888945239, 0.02346701176517511,
                                         0.01204710974546615,  0.02504312108519287,   0.00960400397667947,  0.01492339147837984,  0.003852041169481545,
                                         0.004697867899841702, 0.0006208767756319313, 0.0006086858991153866};

static double IIR2100_Coef_A_9600[11] = {1.0,
                                         -1.943566985186252,
                                         6.46824695457787,
                                         -8.297207613661302,
                                         14.44577546317007,
                                         -12.64954539798627,
                                         14.33753948287288,
                                         -8.173322546547311,
                                         6.323930771614717,
                                         -1.885953470363953,
                                         0.9630862382111866};

static double IIR2100_Coef_A_8000[13] = {1.0,
                                         0.895414910356467,
                                         6.017214854697117,
                                         4.319895336143241,
                                         14.81300311749296,
                                         8.317289565710666,
                                         19.12198197948349,
                                         7.987800922903028,
                                         13.65601668082814,
                                         3.826043070202579,
                                         5.112303726012623,
                                         0.7310565944840918,
                                         0.7824741196348293};

/* ------------------------------------------------------------------------------------------------ GenericToneDetector */

/* a / period rounded UP (period == 0 faults) */
static inline unsigned ceilDiv(unsigned a, unsigned period)
{
  unsigned q = a / period;
  return (q * period < a) ? q + 1u : q;
}

/* The body shared by the two constructors: build the IIR object, then store the fields and clear the state. */
static inline __attribute__((always_inline)) void initDetector(GenericToneDetector* t, unsigned nDen, unsigned nNum, double* den, double* num,
                                                               unsigned minSamples, unsigned maxGapSamples, uint32_t threshWord, unsigned ratioTest,
                                                               uint32_t ratioWord, unsigned period, unsigned extra)
{
  /* the IIR object first (three allocations: the object, then its two histories) ... */
  P02_GenericIIR* p = (P02_GenericIIR*)sysdep_malloc((unsigned)sizeof(P02_GenericIIR));
  new (p) P02_GenericIIR(nDen, nNum, den, num, extra);
  /* ... then the fields */
  t->iir_ = p;
  t->period_ = period;
  storeWord(&t->thresh_, threshWord);
  storeWord(&t->ratio_, ratioWord);
  t->needHits_ = ceilDiv(minSamples, period);
  t->maxMiss_ = ceilDiv(maxGapSamples, period);
  t->ratioTest_ = ratioTest;
  p->reset();
  storeWord(&t->sumX2_, 0u);
  storeWord(&t->sumY2_, 0u);
  storeWord(&t->avgX2_, 0u);
  storeWord(&t->avgY2_, 0u);
  t->hits_ = 0;
  t->misses_ = 0;
  t->count_ = 0;
  t->detected_ = 0;
}

/** @brief Build a detector; see the declaration for the arguments.  The two floats are converted to 32-bit words and handed to initDetector(). */
GenericToneDetector::GenericToneDetector(unsigned nDen, unsigned nNum, double* den, double* num, unsigned minSamples, unsigned maxGapSamples, float thresh,
                                         unsigned ratioTest, float ratio, unsigned period, unsigned extra)
{
  initDetector(this, nDen, nNum, den, num, minSamples, maxGapSamples, wordOf(thresh), ratioTest, wordOf(ratio), period, extra);
}

/** @brief The same with the thresh / ratio arguments already as raw words (used by ANSamToneDetector). */
GenericToneDetector::GenericToneDetector(Words, unsigned nDen, unsigned nNum, double* den, double* num, unsigned minSamples, unsigned maxGapSamples,
                                         uint32_t threshWord, unsigned ratioTest, uint32_t ratioWord, unsigned period, unsigned extra)
{
  initDetector(this, nDen, nNum, den, num, minSamples, maxGapSamples, threshWord, ratioTest, ratioWord, period, extra);
}

/** @brief Destroy the IIR object (if there is one) and free it. */
GenericToneDetector::~GenericToneDetector()
{
  P02_GenericIIR* p = iir_;

  if (p)
  {
    p->~P02_GenericIIR();
    sysdep_free(p);
  }
}

/** @brief Clear the filter history, the period sums, the smoothed powers, the run counters and the detection flag. */
void GenericToneDetector::reset()
{
  iir_->reset();
  hits_ = 0;
  storeWord(&sumX2_, 0u);
  storeWord(&sumY2_, 0u);
  storeWord(&avgX2_, 0u);
  storeWord(&avgY2_, 0u);
  misses_ = 0;
  count_ = 0;
  detected_ = 0;
}

/* The smoothing constants of both process() functions (float).  The block version's hysteresis test also uses the float 0.5f and the DOUBLE 0.85. */
static const float kNew = 0.3f; /* weight of the new period */
static const float kOld = 0.7f; /* weight of the smoothed value */

/**
 * @brief Process one sample.
 * @return the detection flag (`detected_`)
 *
 * Accumulates x^2 and y^2; at the end of a period (`period_` samples) decides hit or miss as described in the file header, updates the run counters and
 * the detection flag and clears the sums.  Hit: YR >= thresh_ and, with the ratio test on, avgY2_ > avgX2_ * ratio_.  Everything else is a miss; a
 * ratio miss with `hits_` still at `needHits_` keeps the detection set until `maxMiss_` misses accumulate.
 */
unsigned GenericToneDetector::process(float x)
{
  float yf = iir_->process(x); /* y rounded to float */
  LD X = (LD)x * (LD)x + (LD)sumX2_;
  LD Y = (LD)yf * (LD)yf + (LD)sumY2_;
  unsigned cnt = count_ + 1;
  count_ = cnt;
  if (cnt != period_)
  {
    sumY2_ = (float)Y;
    sumX2_ = (float)X;
    return detected_;
  }

  /* end of a period: mean powers and their smoothing */
  LD R = (LD)1.0f / (LD)cnt;
  LD XR = X * R;
  LD YR = Y * R;
  LD V14 = XR * (LD)kNew + (LD)avgX2_ * (LD)kOld;
  LD V18 = (LD)kNew * YR + (LD)kOld * (LD)avgY2_;
  avgX2_ = (float)V14;
  avgY2_ = (float)V18;

  /* hit or miss, run counters, detection flag */
  if (YR >= (LD)thresh_)
  {
    int hit = 1;

    if (ratioTest_ != 0)
      hit = (V18 > V14 * (LD)ratio_);
    if (hit)
    {
      hits_ = hits_ + 1;
      misses_ = 0;
    }
    else
    {
      unsigned m = misses_ + 1;
      misses_ = m;
      if (m >= maxMiss_)
      {
        hits_ = 0;
        detected_ = 0;
      }
    }
    if (hits_ >= needHits_)
      detected_ = 1;
  }
  else
  {
    unsigned m = misses_ + 1;
    misses_ = m;
    if (m >= maxMiss_)
    {
      hits_ = 0;
      detected_ = 0;
    }
  }

  count_ = 0;
  storeWord(&sumX2_, 0u);
  storeWord(&sumY2_, 0u);
  return detected_;
}

/**
 * @brief Process `n` samples (`in` is only read).
 * @return the detection flag (`detected_`)
 *
 * Same period accounting as process(float) with a different decision.  Verdict 1 (hit): YR >= thresh_ and the ratio test passes (or is off); or the
 * hysteresis case YR < thresh_ but YR >= thresh_ / 2 with the ratio test on and avgY2_ > 0.85 * avgX2_.  Verdict 2 (ratio miss): YR >= thresh_ and the
 * ratio test fails; counts as a miss, but the detection flag is still set when `hits_` >= `needHits_`.  Verdict 0 (plain miss): everything else; it
 * never sets the flag.
 */
unsigned GenericToneDetector::process(float* in, unsigned n)
{
  if (n == 0)
    return detected_;
  do
  {
    float x = *in;
    ++in;
    float yf = iir_->process(x);
    LD X = (LD)x * (LD)x + (LD)sumX2_;
    LD Y = (LD)yf * (LD)yf + (LD)sumY2_;
    unsigned cnt = count_ + 1;

    count_ = cnt;
    if (cnt != period_)
    {
      sumY2_ = (float)Y;
      sumX2_ = (float)X;
      continue;
    }

    /* end of a period: mean powers and their smoothing */
    LD R = (LD)1.0f / (LD)cnt;
    LD XR = X * R;
    LD YR = Y * R;
    LD V18 = (LD)kNew * YR + (LD)avgY2_ * (LD)kOld;
    LD V14 = XR * (LD)kNew + (LD)avgX2_ * (LD)kOld;
    avgY2_ = (float)V18;
    avgX2_ = (float)V14;

    /* verdict: 0 = plain miss without the detection test, 1 = hit, 2 = ratio miss (joins the detection test) */
    int verdict;

    if (YR >= (LD)thresh_)
    {
      int hit = 1;

      if (ratioTest_ != 0)
        hit = (V18 > V14 * (LD)ratio_);
      verdict = hit ? 1 : 2;
    }
    else if ((LD)thresh_ * (LD)0.5f > YR)
    {
      verdict = 0;
    }
    else if (ratioTest_ == 0)
    {
      verdict = 0;
    }
    else if (V14 * (LD)0.85 >= V18)
    {
      verdict = 0;
    }
    else
    {
      verdict = 1;
    }

    /* run counters and detection flag */
    if (verdict == 1)
    {
      hits_ = hits_ + 1;
      misses_ = 0;
      if (hits_ >= needHits_)
        detected_ = 1;
    }
    else
    {
      unsigned m = misses_ + 1;
      misses_ = m;
      if (m >= maxMiss_)
      {
        hits_ = 0;
        detected_ = 0;
      }
      if (verdict == 2 && hits_ >= needHits_)
        detected_ = 1;
    }
    count_ = 0;
    storeWord(&sumX2_, 0u);
    storeWord(&sumY2_, 0u);
  } while (--n != 0);
  return detected_;
}

/* ------------------------------------------------------------------------------------------------ ANSamToneDetector */

/* The coefficient set of a sample rate: 8000 Hz -> 13 coefficients, anything else -> the 9600 Hz set of 11. */
static inline unsigned ansamCoefs(unsigned sampleRate) { return sampleRate == 8000u ? 13u : 11u; }
static inline double* ansamDen(unsigned sampleRate) { return sampleRate == 8000u ? IIR2100_Coef_A_8000 : IIR2100_Coef_A_9600; }
static inline double* ansamNum(unsigned sampleRate) { return sampleRate == 8000u ? IIR2100_Coef_B_8000 : IIR2100_Coef_B_9600; }

/** @brief Build a 2100 Hz answer-tone detector: the generic detector with the band-pass of `sampleRate` (8000 -> 13 coefficients, anything else -> 11). */
ANSamToneDetector::ANSamToneDetector(unsigned minSamples, unsigned maxGapSamples, float thresh, unsigned ratioTest, float ratio, unsigned sampleRate,
                                     unsigned period, unsigned extra)
    : GenericToneDetector(Words(), ansamCoefs(sampleRate), ansamCoefs(sampleRate), ansamDen(sampleRate), ansamNum(sampleRate), minSamples, maxGapSamples,
                          wordOf(thresh), ratioTest, wordOf(ratio), period, extra)
{
}

ANSamToneDetector::~ANSamToneDetector() {}
