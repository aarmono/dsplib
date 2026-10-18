/**
 * @file v90_spectral_shaper.cpp
 * @brief V90SpectralVerifier (ISDN NT1 / German PBX / severe-codec line detection from a dB spectrum) and V90SpectralShaper (sign-pattern trellis of the V.90
 *        transmitter), plus the shaper's candidate-code table and the decimal-power table.
 *
 * Classes are declared in v90_spectral_shaper.h.  Entry points used by other files: V90SpectralVerifier::reset() / startAccumulation() / process()
 * (V90Demodulator owns a verifier) and V90SpectralShaper::reset() / resetSSFilter() / process() (V90Mapper owns a shaper).  Callees: Psd
 * (dsp/dsp_window_fft_psd.h), V90SpectralShapingFilter (pcm_leaf_classes.h), the differential encoders (dsp/dsp_int_templates.h), edprintf(),
 * sysdep_malloc() / sysdep_free().
 *
 * The functions freqToNearestBin / freqToLeftBin / freqToRightBin / getSpectrumOfBin / getSpectrumOfNearestBin and applyFrameAction / applyAction have no
 * caller.
 *
 * Numeric conventions (do not "tidy" these, results would differ in the last bit or in the NaN case):
 *  - `long double` (LD) holds every value that stays in an extended-precision register; spectrum values, frequencies and thresholds live as floats in memory
 *    and widen exactly. roundF() is the only rounding to float (the value really is a float afterwards).
 *  - Comparisons are written with the branch sense of the x87 compare: `a > b` is false for NaN; `!(a >= b)` is true for NaN.
 *  - Float to bin conversions truncate to 64 bits and use the low 32 bits; NaN and |x| >= 2^63 give 0.  Print arguments truncate to 32 bits (out of range or
 *    NaN gives 0x80000000).
 *
 * Behaviour worth knowing:
 *  - checkSpecialSpectralConditions clears flag_ before it looks at state_; the three conditions are tested in the order ISDN, PBX, severe codec and each
 *    later one OVERWRITES flag_; flag_ is stored BEFORE the "detected" message.  The two delta prints of ISDN / PBX take the integer part of the UNROUNDED
 *    extended difference (left: the integer part of its magnitude, the fraction from the float-rounded difference minus ITS truncation; right: both integer
 *    parts from the unrounded value, the fraction from the rounded one) and the sign from the rounded float; the severe-codec values are plain floats.
 *    "No special conditions" goes through dsplibs_debug_printf behind an unsigned `level > 1` test.
 *  - The verifier constructor never initialises fill_ / state_ (reset() / startAccumulation() do); the resolution is sampleFreq / (unsigned 64-bit fftLen) in
 *    extended arithmetic (fftLen 0 gives inf / NaN).
 *  - process() copies samples as words, stores fill_ only when it copied something, and runs the spectrum and the check once the buffer is full (state_ 2).
 *  - The shaper constructor builds only the encoders, the filter and the two heap histories (uninitialised) with len_ = 0x18; reset() sets the rest.
 *  - advanceTrellis keeps the best metric as a FLOAT (the extended metric is compared with it and rounded when it wins; a NaN metric always wins); the
 *    leading digit of the winning code (code / pow10Table[depth_]) selects the sign action of the committed (oldest) symbol and the new trellis state is that
 *    digit's parity (digits 1 / 3 give 0, 2 / 4 give 1). A leading digit outside 1 .. 4 (or no candidate below 1e38) commits nothing and leaves state_ alone.
 *  - The digit loop of advanceTrellis / applyAction is `for (p = 0; p <= depth_; p++)`: digit p (least significant first) acts on the symbol
 *    [(depth_ - p) * spf_ .. + spf_) of the history: 1 copy, 2 negate, 3 negate the samples at an even distance from the symbol start, 4 negate the odd ones,
 *    anything else leaves the candidate buffer as it was.
 */

#include "v90_spectral_shaper.h"
#include "core/dsp_math.h"
#include "core/dsp_asm.h"
#include "core/dsp_host.h"
#include "v34/v34_api.h"

extern "C"
{
}

typedef long double LD;

/* ------------------------------------------------------------------------------------------------ helpers (static inline) */

/** @brief Low 32 bits of the 64-bit truncation of x; NaN and |x| >= 2^63 give 0 (the 64-bit integer indefinite has a zero low word). */
static inline uint32_t fistpLow32(LD x)
{
  if (!(x > -9223372036854775809.0L && x < 9223372036854775808.0L))
    return 0;
  return (uint32_t)(int64_t)x;
}

#include "core/dsp_intmath.h"

/** @brief Round an extended value to float and force the result through memory so the compiler cannot keep the excess precision. */
static inline float roundF(LD v)
{
  float f = (float)v;
  OPTIMIZER_HIDE_MEM(f);
  return f;
}

/** @brief Sign character of a printed value: '-' (0x2d) when v <= 0 and '+' (0x2b) otherwise, so zero prints as '-' and NaN as '+'. */
static inline int signOf(LD v) { return (v <= 0.0L) ? 0x2d : 0x2b; }

/** @brief The "%c%d.%02d" arguments of a float v: sign, trunc(|v|) and |trunc((v - trunc(v)) * 100)|. */
static inline void splitPlain(LD v, int* sign, int* ip, int* fp)
{
  int t = dsp_trunc_i32(v);
  *ip = dsp_trunc_i32(__builtin_fabsl(v));
  LD frac = v - (LD)t;
  *fp = absI32(dsp_trunc_i32(frac * 100.0L));
  *sign = signOf(v);
}

/** @brief Print arguments of the left-peak delta of the ISDN / PBX checks: `d` is the float-rounded difference, `t8` the unrounded extended difference. */
static inline void splitLeftDelta(LD t8, float d, int* sign, int* ip, int* fp)
{
  int tl = dsp_trunc_i32((LD)d);            /* truncation of the rounded float */
  *ip = dsp_trunc_i32(__builtin_fabsl(t8)); /* integer part from the magnitude of the unrounded value */
  LD x = ((LD)d - (LD)tl) * 100.0L;
  *fp = absI32(dsp_trunc_i32(x));
  *sign = signOf((LD)d);
}
/** @brief Print arguments of the right-peak delta: both integer parts come from the unrounded difference `t1`, the fraction from the float-rounded `d`. */
static inline void splitRightDelta(LD t1, float d, int* sign, int* ip, int* fp)
{
  int tt = dsp_trunc_i32(t1); /* truncation of the unrounded value */
  *ip = dsp_trunc_i32(__builtin_fabsl(t1));
  LD x = 100.0L * ((LD)d - (LD)tt);
  *fp = absI32(dsp_trunc_i32(x));
  *sign = signOf((LD)d);
}

#include "core/dsp_wordof_value.h"
#include "core/dsp_floatbits.h"

/* ================================================================================================ V90SpectralVerifier */

/**
 * @brief Construct from the parameter block: geometry (FFT length, sample rate, PSD length), bin resolution, the sample and spectrum buffers and the Psd.
 * @warning The sysdep_malloc results are not checked.
 */
V90SpectralVerifier::V90SpectralVerifier(V90Parameters* p)
{
  params_ = p;
  fftLen_ = (unsigned)p->spectralVerifierFftLen;
  LD f = (LD)p->spectralVerifierSampleFreq;
  sampleFreq_ = roundF(f);
  psdLen_ = (unsigned)p->spectralVerifierPsdLen;
  resolution_ = roundF(f / (LD)(uint64_t)fftLen_); /* fftLen_ converted as an unsigned 64-bit value */

  buf_ = (float*)sysdep_malloc(psdLen_ << 2);
  spec_ = (float*)sysdep_malloc((fftLen_ >> 1) << 2);

  void* mem = sysdep_malloc(sizeof(Psd));
  Psd* psd = new (mem) Psd(fftLen_, (WindowType)params_->spectralVerifierFftWindow, (unsigned)params_->spectralVerifierPsdOverlapLen);
  psd_ = psd;
  flag_ = 0;
}

/** @brief Free the buffers and destroy and free the Psd. */
V90SpectralVerifier::~V90SpectralVerifier()
{
  if (buf_)
    sysdep_free(buf_);
  if (spec_)
    sysdep_free(spec_);
  if (psd_)
  {
    psd_->~Psd();
    sysdep_free(psd_);
  }
}

/** @brief Print "Reset" and return to the idle state (state_, fill_ and flag_ = 0). */
void V90SpectralVerifier::reset()
{
  edprintf("V90SpectralVerifier: Reset\r\n");
  state_ = 0;
  fill_ = 0;
  flag_ = 0;
}

/** @brief Begin gathering samples, unless already gathering (state_ == 1) or disabled by params->spectralVerifierEnable. */
void V90SpectralVerifier::startAccumulation()
{
  if (state_ == 1)
    return;
  if (params_->spectralVerifierEnable == 0)
    return;

  edprintf("V90SpectralVerifier: start data accumulation\r\n");
  state_ = 1;
  fill_ = 0;
}

/* The const bin-lookup methods below are thin wrappers around these helpers. */
static inline unsigned long binNearest(float f, float res) { return (unsigned long)fistpLow32((LD)f / (LD)res + 0.5L); }
static inline unsigned long binLeft(float f, float res) { return (unsigned long)fistpLow32((LD)f / (LD)res); }
static inline unsigned long binRight(float f, float res) { return (unsigned long)(uint32_t)(fistpLow32((LD)f / (LD)res) + 1u); }
static inline float specAt(const float* spec, unsigned long bin) { return spec[bin]; }
static inline float specAtNearest(const float* spec, float f, float res) { return spec[fistpLow32((LD)f / (LD)res + 0.5L)]; }

unsigned long V90SpectralVerifier::freqToNearestBin(float f) const { return binNearest(f, resolution_); }
unsigned long V90SpectralVerifier::freqToLeftBin(float f) const { return binLeft(f, resolution_); }
unsigned long V90SpectralVerifier::freqToRightBin(float f) const { return binRight(f, resolution_); }
float V90SpectralVerifier::getSpectrumOfBin(unsigned long bin) const { return specAt(spec_, bin); }
float V90SpectralVerifier::getSpectrumOfNearestBin(float f) const { return specAtNearest(spec_, f, resolution_); }

/** @brief Body of printSpectrum(): one line per bin, "Spectrum[<Hz>] = <dB>", between two dashed lines. */
static inline void printSpectrumBody(const V90SpectralVerifier* o)
{
  edprintf("--------------------------------------------------------\r\n");
  for (unsigned i = 0; i < (o->fftLen_ >> 1); i++)
  {
    LD v = (LD)o->spec_[i];
    int sign, ip, fp;
    splitPlain(v, &sign, &ip, &fp);
    int freq = dsp_trunc_i32((LD)(uint64_t)i * (LD)o->resolution_ + 0.5L);
    edprintf("V90SpectralVerifier: Spectrum[%d] = %c%d.%02d\r\n", freq, sign, ip, fp);
  }
  edprintf("--------------------------------------------------------\r\n");
}
void V90SpectralVerifier::printSpectrum() const { printSpectrumBody(this); }

/** @brief Bin indices of a (left peak, null, right peak) frequency triple: trunc(f * r + 0.5) with r = 1 / resolution, evaluated in extended precision. */
static inline void triple(const V90SpectralVerifier* o, float fl, float fn, float fr, uint32_t* il, uint32_t* in_, uint32_t* ir)
{
  LD r = 1.0L / (LD)o->resolution_;
  *il = fistpLow32((LD)fl * r + 0.5L);
  *in_ = fistpLow32((LD)fn * r + 0.5L);
  LD t6 = r * (LD)fr;
  *ir = fistpLow32(0.5L + t6);
}

/**
 * @brief Test the spectrum for the three line signatures and set flag_.
 *
 * ISDN NT1 box and German PBX: both the left-peak and the right-peak level must exceed the null level by more than the configured deltas.  Severe codec: the
 * reference level must exceed BOTH test-frequency levels by more than the configured delta.  Each test prints its measurements first.  Runs only in state_ 2.
 * @warning Bin indices come from the configured frequencies without a range check against fftLen_ / 2; a frequency above the Nyquist rate reads past spec_.
 */
void V90SpectralVerifier::checkSpecialSpectralConditions()
{
  flag_ = 0;
  if (state_ != 2)
    return;
  V90Parameters* p = params_;

  /* ---- German ISDN NT1 box: peaks at the left / right peak frequency, relative to the null */
  {
    uint32_t il, in_, ir;
    triple(this, p->spectralVerifierIsdnLeftPeakFreq, p->spectralVerifierIsdnNullFreq, p->spectralVerifierIsdnRightPeakFreq, &il, &in_, &ir);

    float* spec = spec_;
    float spN = spec[in_];
    LD t8 = (LD)spec[il] - (LD)spN;
    float dL = roundF(t8);
    float spR = spec[ir];

    int sign, ip, fp;
    splitLeftDelta(t8, dL, &sign, &ip, &fp);
    edprintf("V90SpectralVerifier: German ISDN NT1 box: left peak delta = %c%d.%02d\r\n", sign, ip, fp);

    LD t1 = (LD)spR - (LD)spN;
    float dR = roundF(t1);
    splitRightDelta(t1, dR, &sign, &ip, &fp);
    edprintf("V90SpectralVerifier: German ISDN NT1 box: right peak delta = %c%d.%02d\r\n", sign, ip, fp);

    p = params_;
    if ((LD)dL > (LD)p->spectralVerifierIsdnLeftPeakDelta)
    {
      if ((LD)dR > (LD)p->spectralVerifierIsdnRightPeakDelta)
      {
        flag_ = 1;
        edprintf("V90SpectralVerifier: German ISDN NT1 box conditions detected!\r\n");
        p = params_;
      }
    }
  }

  /* ---- German PBX */
  {
    uint32_t il, in_, ir;
    triple(this, p->spectralVerifierGermanPbxLeftPeakFreq, p->spectralVerifierGermanPbxNullFreq, p->spectralVerifierGermanPbxRightPeakFreq, &il, &in_, &ir);

    float* spec = spec_;
    float spN = spec[in_];
    LD t8 = (LD)spec[il] - (LD)spN;
    float dL = roundF(t8);
    float spR = spec[ir];

    int sign, ip, fp;
    splitLeftDelta(t8, dL, &sign, &ip, &fp);
    edprintf("V90SpectralVerifier: German PBX: left peak delta = %c%d.%02d\r\n", sign, ip, fp);

    LD t1 = (LD)spR - (LD)spN;
    float dR = roundF(t1);
    splitRightDelta(t1, dR, &sign, &ip, &fp);
    edprintf("V90SpectralVerifier: German PBX: right peak delta = %c%d.%02d\r\n", sign, ip, fp);

    p = params_;
    if ((LD)dL > (LD)p->spectralVerifierGermanPbxLeftPeakDelta)
    {
      if ((LD)dR > (LD)p->spectralVerifierGermanPbxRightPeakDelta)
      {
        flag_ = 2;
        edprintf("V90SpectralVerifier: German PBX conditions detected!\r\n");
        p = params_;
      }
    }
  }

  /* ---- severe codec: a reference frequency and two test frequencies, each printed as a plain float */
  {
    LD r = 1.0L / (LD)resolution_;
    uint32_t iRef = fistpLow32((LD)p->spectralVerifierSevereCodecRefFreq * r + 0.5L);

    float* spec = spec_;
    float spRef = spec[iRef];
    uint32_t i1 = fistpLow32((LD)p->spectralVerifierSevereCodecTestFreq1 * r + 0.5L);
    LD t6 = r * (LD)p->spectralVerifierSevereCodecTestFreq2;
    float sp1 = spec[i1];
    uint32_t i2 = fistpLow32(0.5L + t6);
    float sp2 = spec[i2];

    int sign, ip, fp;
    splitPlain((LD)spRef, &sign, &ip, &fp);
    edprintf("V90SpectralVerifier: Severe Codec: ref freq (%d)  = %c%d.%02d\r\n", dsp_trunc_i32((LD)p->spectralVerifierSevereCodecRefFreq), sign, ip, fp);
    splitPlain((LD)sp1, &sign, &ip, &fp);
    edprintf("V90SpectralVerifier: Severe Codec: test freq1 (%d)  = %c%d.%02d\r\n", dsp_trunc_i32((LD)params_->spectralVerifierSevereCodecTestFreq1), sign, ip,
             fp);
    splitPlain((LD)sp2, &sign, &ip, &fp);
    edprintf("V90SpectralVerifier: Severe Codec: test freq2 (%d)  = %c%d.%02d\r\n", dsp_trunc_i32((LD)params_->spectralVerifierSevereCodecTestFreq2), sign, ip,
             fp);

    p = params_;
    LD d1 = (LD)spRef - (LD)sp1;
    LD delta = (LD)p->spectralVerifierSevereCodecDelta;
    if (!(delta >= d1))
    { /* also true when either side is NaN */
      LD d2 = (LD)spRef - (LD)sp2;
      if (d2 > delta)
      {
        flag_ = 3;
        edprintf("V90SpectralVerifier: Severe Codec conditions detected!\r\n");
      }
    }
  }

  if (flag_ != 0)
    return;

  if (dsplibs_debug_level > 1)
    dsplibs_debug_printf("V90SpectralVerifier: No special conditions\r\n");
}

/**
 * @brief Append up to the missing number of samples to buf_; when it is full, compute the dB spectrum, print it if params->spectralVerifierPrintSpectrum is set
 *        and run the condition check.
 * @return 1 on the call that completes the spectrum, otherwise 0; extra input samples of that call are dropped
 */
unsigned V90SpectralVerifier::process(float* in, unsigned n)
{
  unsigned rv = 0;
  if (state_ != 1)
    return rv;

  unsigned fill = fill_;
  unsigned need = psdLen_;
  unsigned i = 0;
  if (fill < need && i < n)
  {
    float* buf = buf_;
    do
    {
      __builtin_memcpy(&buf[fill], &in[i], sizeof(float)); /* bit-exact copy (keeps NaN payloads) */
      i++;
      fill++;
    } while (fill < need && i < n);
    fill_ = fill;
  }
  if (fill < need)
    return rv;

  state_ = 2;
  psd_->process(buf_, need, spec_, Psd::kDb);
  edprintf("V90SpectralVerifier: Ready\r\n");
  rv = 1;
  if (params_->spectralVerifierPrintSpectrum != 0)
    printSpectrum();
  checkSpecialSpectralConditions();
  return rv;
}

/* ================================================================================================ V90SpectralShaper */

/**
 * Candidate sign-pattern codes of the trellis: row (state_ + 2 * depth_) of 16 candidates.  Each code is a decimal number with one digit per symbol of the
 * history (digit 1 keep, 2 negate, 3 negate even samples, 4 negate odd samples; the leading digit belongs to the oldest symbol); 0 marks an unused slot.
 * Rows 0 .. 7 cover depths 0 .. 3.
 */
int V90SpectralShaper::actionLookupTable[128] = {
    1,    2,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    3,    4,    0,    0,    0,    0,    0,    0,    0,    0,
    0,    0,    0,    0,    0,    0,    11,   12,   23,   24,   0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    31,   32,   43,   44,
    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    111,  112,  123,  124,  231,  232,  243,  244,  0,    0,    0,    0,    0,    0,
    0,    0,    311,  312,  323,  324,  431,  432,  443,  444,  0,    0,    0,    0,    0,    0,    0,    0,    1111, 1112, 1123, 1124, 1231, 1232, 1243, 1244,
    2311, 2312, 2323, 2324, 2431, 2432, 2443, 2444, 3111, 3112, 3123, 3124, 3231, 3232, 3243, 3244, 4311, 4312, 4323, 4324, 4431, 4432, 4443, 4444,
};
/** Powers of ten 1 .. 10000, indexed by the trellis depth (see v90_spectral_shaper.h). */
unsigned pow10Table[5] = {1, 10, 100, 1000, 10000};

/** @brief One sample, negated when `negate` is set (the 16-bit value is widened unsigned, so -32768 stays -32768). */
static inline short signedSample(short s, int negate) { return (short)(negate ? -(int)(unsigned short)s : (int)(unsigned short)s); }

/**
 * @brief Apply one sign action to the span [base, base + n) of a sample buffer: dst[j] = src[j] with the pattern `kind`.
 * @param kind  0 keep, 1 negate, 2 negate the samples at an even distance from `base`, 3 negate the odd ones; anything else leaves dst untouched
 * @note  An empty span (n == 0) or one whose end wraps does nothing.
 */
static inline void actionSpan(int kind, const short* src, short* dst, unsigned base, unsigned n)
{
  unsigned end = base + n;
  if (end <= base)
    return;
  for (unsigned j = base; j < end; j++)
  {
    int odd = (int)((j - base) & 1u);
    switch (kind)
    {
    case 0:
      dst[j] = src[j];
      break;
    case 1:
      dst[j] = signedSample(src[j], 1);
      break;
    case 2:
      dst[j] = signedSample(src[j], !odd);
      break;
    case 3:
      dst[j] = signedSample(src[j], odd);
      break;
    default:
      break;
    }
  }
}

/**
 * @brief Constructor body: allocate the two histories (24 shorts each) and set len_ = 24, fill_ = 0, state_ = 0. The encoders and the filter are built by the
 *   initialiser list.
 */
static inline void shaperCtorBody(V90SpectralShaper* o)
{
  o->histA_ = (short*)sysdep_malloc(0x30);
  o->histB_ = (short*)sysdep_malloc(0x30);
  o->len_ = 0x18;
  o->fill_ = 0;
  o->state_ = 0;
}

/** @brief Construct the encoders and the filter, then allocate the histories (shaperCtorBody). */
V90SpectralShaper::V90SpectralShaper() : sde_(), pde_(6), ssf_() { shaperCtorBody(this); }

/** @brief Free the two histories. */
V90SpectralShaper::~V90SpectralShaper()
{
  if (histA_)
    sysdep_free(histA_);
  if (histB_)
    sysdep_free(histB_);
}

/**
 * @brief Set depth and samples per symbol, clear the encoders, the filter (new coefficients, N_ = spf_) and histA_, and arm the delay line.
 *
 * len_ = (depth + 1) * spf_ is the history length, fill_ = spf_ * depth the write position (the first depth symbols are collected before the trellis runs).
 */
void V90SpectralShaper::reset(unsigned depth, unsigned arg, float c0, float c1, float c2, float c3)
{
  uint32_t w0 = wordOf(c0), w1 = wordOf(c1), w2 = wordOf(c2), w3 = wordOf(c3); /* the coefficients are passed on as raw words */
  arg_ = arg;
  depth_ = depth;
  spf_ = arg ? 6u / arg : 0u;
  sde_.state_ = 0;
  pde_.reset(spf_, 0);
  ssf_.reset();
  ssf_.setFilterCoeff(bitsFloat(w0), bitsFloat(w1), bitsFloat(w2), bitsFloat(w3));
  ssf_.N_ = spf_;

  len_ = (depth_ + 1) * spf_;
  fill_ = spf_ * depth_;
  for (unsigned i = 0; i < 24; i++)
    histA_[i] = 0;
  delay_ = depth_;
  state_ = 0;
}

/** @brief Reset the metric filter and load the four coefficients (only c0 is passed as a float, the others as words). */
void V90SpectralShaper::resetSSFilter(float c0, float c1, float c2, float c3)
{
  uint32_t w1 = wordOf(c1), w2 = wordOf(c2), w3 = wordOf(c3);
  ssf_.reset();
  ssf_.setFilterCoeff(c0, bitsFloat(w1), bitsFloat(w2), bitsFloat(w3));
}

/** @brief Apply sign pattern `action` to spf_ samples of histA_ starting at `offset`, writing to out[offset ...].  No caller. */
void V90SpectralShaper::applyFrameAction(ACTIONS action, short* out, int offset) { actionSpan((int)action, histA_, out, (unsigned)offset, spf_); }

/**
 * @brief Apply the per-symbol actions encoded in the decimal digits of `actions` (least significant digit = symbol depth_) to histA_, writing to `out`. No
 *   caller.
 */
void V90SpectralShaper::applyAction(int actions, short* out)
{
  unsigned K = depth_;
  for (unsigned p = 0; p <= K; p++)
  {
    int digit = actions % 10;
    actions = actions / 10;
    actionSpan(digit - 1, histA_, out, (K - p) * spf_, spf_); /* digit 1 .. 4 = action 0 .. 3 */
  }
}

/**
 * @brief Pick the candidate sign pattern with the smallest filter metric and commit its action for the oldest symbol.
 *
 * For each of the 2^(depth_ + 1) candidates: copy histA_ to histB_, apply the candidate's per-symbol actions and measure histB_ with the filter.  The winner's
 * leading digit then modifies the oldest symbol of histA_ in place and sets state_ (digits 1 / 3: 0; 2 / 4: 1).
 */
void V90SpectralShaper::advanceTrellis()
{
  unsigned K = depth_;
  unsigned nStates = 1u << ((K + 1) & 31);
  float best = bitsFloat(0x7e967699u); /* 9.99999968e37: initial "worse than anything" metric */
  int bestCode = 0;

  for (unsigned s = 0; s < nStates; s++)
  {
    for (unsigned j = 0; j < len_; j++)
      histB_[j] = histA_[j];
    int cand = actionLookupTable[((state_ + K * 2) << 4) + s];
    int code = cand;
    for (unsigned p = 0; p <= K; p++)
    {
      int digit = code % 10;
      code = code / 10;
      actionSpan(digit - 1, histA_, histB_, (K - p) * spf_, spf_);
    }
    LD m = ssf_.getMetricLD(histB_, K + 1);
    if (!(m >= (LD)best))
    { /* true for a NaN metric too */
      best = roundF(m);
      bestCode = cand;
    }
  }

  unsigned q = (unsigned)bestCode / pow10Table[K];
  int sel = (q >= 1 && q <= 4) ? (int)q - 1 : 4; /* 4: leading digit outside 1 .. 4, no action */
  for (unsigned j = 0; j < spf_; j++)
  {
    short v = histA_[j];
    switch (sel)
    {
    case 1:
      v = signedSample(v, 1);
      break;
    case 2:
      v = signedSample(v, !(j & 1u));
      break;
    case 3:
      v = signedSample(v, (int)(j & 1u));
      break;
    default:
      break;
    }
    histA_[j] = v;
  }
  if (sel == 1 || sel == 3)
    state_ = 1;
  else if (sel == 0 || sel == 2)
    state_ = 0;
}

/**
 * @brief One symbol interval: differentially encode the sign bits, sign the input samples into the history, run the trellis once the delay line is
 *        empty, emit the oldest symbol, advance the metric filter and shift the history down by one symbol.
 */
void V90SpectralShaper::process(short* in, unsigned char* bits, short* out)
{
  /* sign bits: position 0 is fixed at 0, the odd positions go through the serial encoder, then the parallel encoder runs over all positions */
  bitsIn_[0] = 0;
  unsigned sps = spf_;
  for (unsigned i = 0; i < sps - 1; i++)
    bitsIn_[1 + i] = bits[i];
  for (unsigned i = 0; i < sps; i++)
  {
    if (i & 1u)
      bitsDiff_[i] = sde_.process(bitsIn_[i]);
    else
      bitsDiff_[i] = bitsIn_[i];
  }
  pde_.process(bitsDiff_, bitsOut_);

  /* append the signed input samples to the history (a 0 bit negates) */
  sps = spf_;
  if (sps > 0)
  {
    short* buf = histA_;
    unsigned pos = fill_;
    for (unsigned c = 0; c < sps; c++)
    {
      int v = (int)(unsigned short)in[c];
      if (bitsOut_[c] == 0)
        v = -v;
      buf[pos] = (short)v;
      pos++;
    }
    fill_ = pos;
  }

  /* once depth_ symbols have been collected the trellis decides on the oldest symbol; before that the delay line just counts down */
  if (delay_ == 0)
  {
    advanceTrellis();
    sps = spf_;
  }
  else
  {
    delay_ = delay_ - 1;
  }

  /* emit the oldest symbol and feed it to the metric filter */
  if (sps > 0)
  {
    for (unsigned c = 0; c < sps; c++)
      out[c] = histA_[c];
  }
  ssf_.progress(histA_);

  /* shift the history down by one symbol */
  sps = spf_;
  unsigned len = len_;
  if (len > sps)
  {
    short* a = histA_;
    unsigned c = sps;
    do
    {
      a[c - sps] = a[c];
      c++;
    } while (len > c);
  }
  fill_ = fill_ - sps;
}
