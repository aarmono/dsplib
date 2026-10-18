/**
 * @file dsp_window_fft_psd.cpp
 * @brief Window functions, statistics, the Numerical-Recipes FFT (four1, realfft), LowPassFIR<float>, Psd, Agc<float> and SineWave<float, float>
 *        (declarations in dsp_window_fft_psd.h).
 *
 * Transcendental functions.  sin / cos / log10 are evaluated as x87 INSTRUCTIONS through core/dsp_math.h (dsp_fsin / dsp_fcos / dsp_log10), on extended
 * operands, at the places named below; the square root is IEEE.  Everything else follows the numeric rules of dsp_float_filters.cpp: float32 in memory,
 * `long double` for the intermediate values, an explicit `(T)` cast wherever the value is stored into a float slot, doubles where a double slot is kept
 * (realfft's wi / wpr / wpi), and double constants written as DOUBLE literals converted to long double (0.46L is a different number).
 *
 * Behaviour notes.
 *  - sum / mean / sqrSum / Var / Std return an UNROUNDED extended value; Var rounds sqrSum to float32 first, then subtracts the extended mean^2 (so it can come
 *    out slightly negative, and Std then gives NaN).  The exported float-returning versions leave the value in st(0) on i386 (dsp_ret_ext_float); the
 *    `...LD` functions return it as a long double everywhere.  sinc rounds to float32.
 *  - four1 / realfft use ONE-BASED indices (data[1] is the first element): the caller passes an array of at least 2 nn + 1 (n + 1) floats; data[0] is never
 *    touched.  Sizes that are not powers of two are not defined (they index outside the block); the memory-safe ones (four1 nn = 2^k - 1, realfft
 *    n = 2^k - 2 .. 2^k + 1) read and write the 1 to 2 floats after the window.  four1's bit-reversal swap interleaves its loads and stores in a fixed
 *    order that differs from two independent swaps when j == i + 1 (only for those non-power-of-two sizes).  Both keep the sine recurrence in extended
 *    precision, EXCEPT realfft's wi / wpi / wpr, which are rounded to double at every store, and theta, which realfft rounds to double only on the
 *    isign == 1 path (it lives across the four1 call); the isign != 1 path negates the extended theta.
 *  - realfft with isign != 1 (any value, not only -1) is the inverse transform: c2 = +0.5, theta negated, data[1 .. 2] * 0.5, four1 with isign -1.
 *  - hanning: w[k-1] = (1 - cos(2 pi k / (n + 1))) / 2, with the division done as a multiplication by the extended reciprocal 1 / (n + 1); hamming and
 *    blackman use 1 / (n - 1): n == 1 gives NaN.  The double constants are 2 pi = 6.283185307179586, 4 pi, 0.46, 0.54, 0.42, 0.08; four1 uses the truncated
 *    6.28318530717959.
 *  - sinc(NaN) = 1 (an unordered comparison counts as equal to zero).
 *  - LowPassFIR::design validates fc and n BEFORE changing anything (returns 1 for fc < 0, NaN, fc > 1 or n <= 1); then stores n_, frees the old block (the
 *    pointer is not cleared), allocates (win == NULL: hamming; adopt == 0: a copy of the window words) or adopts the window pointer (adopt != 0: the object
 *    frees it later).  The four-argument design() allocates the window block itself and passes it with adopt = 1; it leaks that block when design() rejects the
 *    arguments.  The sinc argument sequence is x0 = -fc (n - 1) / 2, x(i+1) = (float)x(i) + fc (the extended sum of the ROUNDED float and fc); sinc is
 *    called with (float)x(i).  The constructor only sets coef_ = 0 and calls design (n_ stays uninitialised when design fails).
 *  - Psd::process has no checks (N == 0, N == overlap and n < overlap divide by zero or run away).  out[] is zeroed first, every segment adds
 *    re^2 + im^2 + out[k] in extended precision and stores float32; bins use buf_[2k + 1], buf_[2k + 2] (bin 0 = DC + i * Nyquist, the packed output of
 *    realfft); kDbRelativeToMax finds the maximum from +0 with a strict > (NaN never wins) and multiplies by the extended reciprocal; the dB values are
 *    (float)log10(x) * 10 (float32 intermediate).
 *  - Agc::process: acc_ += x * x is rounded to float32 EVERY sample (kept in the object), out = (float)(gain * x), count_ is written back only at the end of
 *    the call; the gain update happens on the sample that ends the period and is used from the NEXT sample; mix_ == 1.0 (frozen) stores power_ and clears
 *    acc_ but leaves the gain; a NaN mix_ counts as frozen; the update needs power > 1e-10 (NaN: no update).
 *  - SineWave::generate: sin() of the EXTENDED phase, the next phase = (float)stored phase + dphi, i.e. the float rounding of the phase accumulates; the
 *    final phase is wrapped by a float -> int32 TRUNCATION of phase / (2 pi) (float constant 1/(2 pi)); an out-of-range quotient gives the integer
 *    indefinite 0x80000000 (dsp_trunc_i32).
 */

#include "dsp_window_fft_psd.h"
#include "core/dsp_math.h"
#include "core/dsp_asm.h"

typedef long double LD;

/* ----------------------------------------------------------------------------------------------------- helpers */

#include "core/dsp_copyword_plain.h"
#include "core/dsp_floatbits.h"
static inline void storeZeroWord(float* dst) { __builtin_memset(dst, 0, sizeof(float)); }

/* double constants: DOUBLE literals converted to extended (an `L` literal would be another number) */
static inline LD cTwoPi() { return (LD)6.283185307179586; }
static inline LD cFourPi() { return (LD)12.566370614359172; }
static inline LD cPi() { return (LD)3.141592653589793; }

/* ----------------------------------------------------------------------------------------------------- windows */

/** @brief w[i] = 1. */
template <class T> void boxcar(T* w, unsigned n)
{
  for (unsigned i = 0; i < n; i++)
    w[i] = (T)1.0f;
}

/** @brief Hanning window, n + 1 flavour: w[k-1] = (1 - cos(2 pi k * (1 / (n + 1)))) * 0.5, k = 1 .. n. */
template <class T> void hanning(T* w, unsigned n)
{
  if (n == 0)
    return;
  const LD twopi = cTwoPi();
  const LD q = (LD)1 / (LD)(uint32_t)(n + 1u);
  unsigned k = 1;
  for (;;)
  {
    unsigned cur = k;
    LD x = ((LD)cur * twopi) * q;
    w[cur - 1] = (T)(((LD)1 - dsp_fcos(x)) * (LD)0.5f);
    k++;
    if (!(cur < n))
      break;
  }
}

/** @brief Hamming window, n - 1 flavour: w[i] = 0.54 - 0.46 cos(2 pi i * (1 / (n - 1))); n == 1 gives NaN. */
template <class T> void hamming(T* w, unsigned n)
{
  if (n == 0)
    return;
  const LD twopi = cTwoPi();
  const LD q = (LD)1 / (LD)(uint32_t)(n - 1u);
  for (unsigned i = 0; i < n; i++)
  {
    LD x = ((LD)i * twopi) * q;
    w[i] = (T)((LD)0.54 - dsp_fcos(x) * (LD)0.46);
  }
}

/** @brief Blackman window, n - 1 flavour: w[i] = 0.08 cos(4 pi i q) + (0.42 - 0.5 cos(2 pi i q)), q = 1 / (n - 1); n == 1 gives NaN. */
template <class T> void blackman(T* w, unsigned n)
{
  if (n == 0)
    return;
  const LD twopi = cTwoPi();
  const LD fourpi = cFourPi();
  const LD q = (LD)1 / (LD)(uint32_t)(n - 1u);
  for (unsigned i = 0; i < n; i++)
  {
    LD x2 = ((LD)i * twopi) * q;
    LD x4 = ((LD)i * fourpi) * q;
    LD c2 = dsp_fcos(x2);
    LD c4 = dsp_fcos(x4);
    w[i] = (T)(c4 * (LD)0.08 + ((LD)0.42 - c2 * (LD)0.5f));
  }
}

/** @brief Fill w[0 .. n-1] with the window of the given type (any unknown type: boxcar). */
template <class T> void designWindow(WindowType type, T* w, unsigned n)
{
  switch (type)
  {
  case WINDOW_HANNING:
    hanning(w, n);
    break;
  case WINDOW_HAMMING:
    hamming(w, n);
    break;
  case WINDOW_BLACKMAN:
    blackman(w, n);
    break;
  default:
    boxcar(w, n);
    break; /* 0 and every other value */
  }
}

/** @brief sin(pi x) / (pi x), 1 for x = +-0 and for NaN, rounded to T. */
template <class T> T sinc(T x)
{
  LD xv = (LD)x;
  if (!(xv < 0 || xv > 0))
    return (T)1; /* +-0 and NaN (unordered) */
  LD y = xv * cPi();
  return (T)(dsp_fsin(y) / y);
}

/* ----------------------------------------------------------------------------------------------------- statistics */

template <class T> LD sumLD(const T* p, unsigned n)
{
  LD s = 0;
  for (unsigned i = 0; i < n; i++)
    s += (LD)p[i];
  return s;
}

template <class T> LD meanLD(const T* p, unsigned n) { return sumLD(p, n) / (LD)(uint32_t)n; }

template <class T> LD sqrSumLD(const T* p, unsigned n)
{
  LD s = 0;
  for (unsigned i = 0; i < n; i++)
  {
    LD v = (LD)p[i];
    s += v * v;
  }
  return s / (LD)(uint32_t)n;
}

template <class T> LD VarLD(const T* p, unsigned n)
{
  T sq = (T)sqrSumLD(p, n); /* rounded to the sample type (fstp DWORD) */
  LD m = meanLD(p, n);
  return (LD)sq - m * m;
}

template <class T> LD StdLD(const T* p, unsigned n) { return dsp_sqrt(VarLD(p, n)); }

/* The float-returning versions: the return value is the unrounded extended result left in st(0) (i386).  DSP_EXTRET_FN makes this work at -O0 too (the asm
 * trick needs the temporary to stay in the register: the attribute puts these five tiny functions at -O2). */
template <class T> DSP_EXTRET_FN T sum(T* p, unsigned n) { return dsp_ret_ext_float(sumLD(p, n)); }
template <class T> DSP_EXTRET_FN T mean(T* p, unsigned n) { return dsp_ret_ext_float(meanLD(p, n)); }
template <class T> DSP_EXTRET_FN T sqrSum(T* p, unsigned n) { return dsp_ret_ext_float(sqrSumLD(p, n)); }
template <class T> DSP_EXTRET_FN T Var(T* p, unsigned n) { return dsp_ret_ext_float(VarLD(p, n)); }
template <class T> DSP_EXTRET_FN T Std(T* p, unsigned n) { return dsp_ret_ext_float(StdLD(p, n)); }

/* ----------------------------------------------------------------------------------------------------- FFT */

/**
 * @brief Complex FFT in place (Numerical Recipes four1).
 * @param data   interleaved re / im, ONE-BASED: data[1 .. 2 nn]; data[0] is not touched
 * @param nn     number of complex points, a power of two
 * @param isign  1 forward, -1 inverse (unscaled)
 *
 * Bit-reversal reordering, then Danielson-Lanczos butterflies with a trigonometric recurrence kept in extended precision.
 */
void four1(float* data, unsigned long nn, int isign)
{
  uint32_t n = (uint32_t)nn * 2u;
  uint32_t i, j = 1, m, mmax, istep;

  /* bit-reversal reordering */
  for (i = 1; i < n; i += 2)
  {
    if (j > i)
    {
      /* swap the pairs d[i], d[i+1] and d[j], d[j+1] with a fixed load / store order, t = d[j]; u = d[i]; d[j] = u; v = d[i+1]; d[i] = t; w = d[j+1];
       * d[j+1] = v; d[i+1] = w; it differs from two independent swaps when the pairs overlap (j == i + 1), which happens only for sizes that are not powers
       * of two */
      float t, u, v, w;
      copyWord(&t, &data[j]);
      copyWord(&u, &data[i]);
      copyWord(&data[j], &u);
      copyWord(&v, &data[i + 1]);
      copyWord(&data[i], &t);
      copyWord(&w, &data[j + 1]);
      copyWord(&data[j + 1], &v);
      copyWord(&data[i + 1], &w);
    }
    m = n;
    for (;;)
    {
      m >>= 1;
      if (!(m > 1 && j > m))
        break;
      j -= m;
    }
    j += m;
  }
  if (n <= 2)
    return;

  /* butterfly passes: mmax doubles each pass, the twiddle factor wr + i wi advances by the recurrence (wpr, wpi) */
  mmax = 2;
  for (;;)
  {
    istep = mmax << 1;
    LD theta = ((LD)1 / (LD)(uint32_t)mmax) * (LD)6.28318530717959;
    theta = theta * (LD)(double)isign;
    LD wpi = dsp_fsin(theta);
    LD wtemp = dsp_fsin(theta * (LD)0.5);
    LD wpr = (wtemp * wtemp) * (LD)(-2.0);
    LD wr = 1, wi = 0;

    for (m = 1; m < mmax; m += 2)
    {
      for (i = m; i <= n; i += istep)
      {
        j = i + mmax;
        float tempr = (float)(((LD)data[j] * wr) - (wi * (LD)data[j + 1]));
        float tempi = (float)(((LD)data[j] * wi) + (wr * (LD)data[j + 1]));
        data[j] = (float)((LD)data[i] - (LD)tempr);
        data[j + 1] = (float)((LD)data[i + 1] - (LD)tempi);
        data[i] = (float)((LD)tempr + (LD)data[i]);
        data[i + 1] = (float)((LD)tempi + (LD)data[i + 1]);
      }

      LD wrn = wr + ((wr * wpr) - (wi * wpi));
      LD win = wi + ((wr * wpi) + (wi * wpr));

      wr = wrn;
      wi = win;
    }
    mmax = istep;
    if (!(n > mmax))
      break;
  }
}

/**
 * @brief Real FFT in place (Numerical Recipes realft): one half-size complex FFT plus a post-processing pass.
 * @param data   ONE-BASED: data[1 .. n]; forward output packs the DC term in data[1] and the Nyquist term in data[2], the other bins as re / im pairs
 * @param nl     number of real points, a power of two
 * @param isign  1 forward; any other value is the inverse (scaled by 1/2, to be completed by the caller's 2/n)
 */
void realfft(float* data, unsigned long nl, int isign)
{
  uint32_t n = (uint32_t)nl;
  LD theta = ((LD)1 / (LD)(uint32_t)(n >> 1)) * cPi();
  float c1 = 0.5f, c2;

  /* forward: complex FFT of the half-size sequence first; inverse: the FFT comes last */
  if (isign == 1)
  {
    double thetaSpill = (double)theta; /* lives across the call in a double slot */
    c2 = -0.5f;
    four1(data, n >> 1, 1);
    theta = (LD)thetaSpill;
  }
  else
  {
    theta = -theta;
    c2 = 0.5f;
  }

  /* twiddle recurrence start values */
  const LD sinHalf = dsp_fsin((LD)c1 * theta);
  const LD sinTheta = dsp_fsin(theta);
  double wpi = (double)sinTheta;
  double wi = (double)sinTheta;
  const LD wprExt = (sinHalf * sinHalf) * (LD)(-2.0f);
  double wpr = (double)wprExt;
  LD wr = (LD)1 + wprExt;
  const float negc2 = -c2;
  const uint32_t np3 = n + 3u;

  /* split the half-size transform into the spectrum of the real sequence, bin pairs (i, n/2 + 2 - i) at a time */
  for (uint32_t i = 2; i <= (n >> 2); i++)
  {
    uint32_t i1 = i + i - 1u, i2 = i1 + 1u, i3 = np3 - i2, i4 = i3 + 1u;
    float a = data[i1], b = data[i2], c = data[i3], d = data[i4];
    LD h2r = ((LD)b + (LD)d) * (LD)negc2;
    LD h1r = ((LD)c + (LD)a) * (LD)c1;
    LD h2i = ((LD)a - (LD)c) * (LD)c2;
    LD h1i = ((LD)b - (LD)d) * (LD)c1;
    data[i1] = (float)((h1r + h2r * wr) - ((LD)wi * h2i));
    data[i2] = (float)(((h2i * wr) + h1i) + (h2r * (LD)wi));
    data[i3] = (float)(((LD)wi * h2i) + (h1r - h2r * wr));
    LD nh1i = -h1i; /* a real negation (fchs): flips the sign of a NaN, which `x - h1i` would not */
    OPTIMIZER_HIDE_MEM(nh1i);
    data[i4] = (float)((nh1i + (h2i * wr)) + (h2r * (LD)wi));
    LD wrn = wr + (((LD)wpr * wr) - ((LD)wi * (LD)wpi));
    double win = (double)(((wr * (LD)wpi) + ((LD)wi * (LD)wpr)) + (LD)wi);
    wr = wrn;
    wi = win;
  }

  /* the first bin pair (DC, Nyquist), then for the inverse the half-size FFT */
  if (isign == 1)
  {
    float d1 = data[1], d2 = data[2];
    data[1] = (float)((LD)d1 + (LD)d2);
    data[2] = (float)((LD)d1 - (LD)d2);
  }
  else
  {
    float d1 = data[1], d2 = data[2];
    data[1] = (float)(((LD)d1 + (LD)d2) * (LD)c1);
    data[2] = (float)(((LD)d1 - (LD)d2) * (LD)c1);
    four1(data, n >> 1, -1);
  }
}

/* ----------------------------------------------------------------------------------------------------- LowPassFIR */

/** @brief Design a low-pass of n taps, normalised cut-off fc (0 .. 1), the given window and a coefficient sum of `gain` (see design()). */
template <class T> LowPassFIR<T>::LowPassFIR(unsigned n, T fc, WindowType wt, T gain)
{
  coef_ = 0;
  design(n, fc, wt, gain);
}

template <class T> LowPassFIR<T>::~LowPassFIR()
{
  if (coef_)
    sysdep_free(coef_);
}

/** @brief Design with a window of the given type; returns 1 if the allocation failed or the arguments are invalid (the window block is then leaked), else 0. */
template <class T> int LowPassFIR<T>::design(unsigned n, T fc, WindowType wt, T gain)
{
  T* buf = (T*)sysdep_malloc(n * 4u);
  if (!buf)
    return 1;
  designWindow(wt, buf, n);
  return design(n, fc, gain, buf, 1);
}

/** @brief Design with a given window (NULL: hamming); see the class description for `adopt`.  Returns 1 for invalid arguments (nothing changed), else 0. */
template <class T> int LowPassFIR<T>::design(unsigned n, T fc, T gain, const T* win, int adopt)
{
  if (!((LD)fc >= 0))
    return 1; /* fc < 0 or NaN */
  if ((LD)fc > (LD)1 || n <= 1)
    return 1;
  n_ = n;
  if (coef_)
    sysdep_free(coef_);
  if (!win)
  {
    coef_ = (T*)sysdep_malloc(n_ * 4u);
    hamming(coef_, n_);
  }
  else if (adopt == 0)
  {
    coef_ = (T*)sysdep_malloc(n * 4u);
    for (unsigned i = 0; i < n_; i++)
      copyWord(&coef_[i], &win[i]);
  }
  else
  {
    coef_ = (T*)win;
  }

  /* multiply the window by fc * sinc(...), then scale the block to the requested sum */
  const LD fcx = (LD)fc;
  LD x = (-fcx * (LD)(uint32_t)(n_ - 1u)) * (LD)0.5f;
  for (unsigned i = 0; i < n_; i++)
  {
    T xf = (T)x;
    LD s = (LD)sinc(xf);
    coef_[i] = (T)((s * fcx) * (LD)coef_[i]);
    x = (LD)xf + fcx;
  }

  LD scale = (LD)gain / sumLD(coef_, n_);
  for (unsigned i = 0; i < n_; i++)
    coef_[i] = (T)((LD)coef_[i] * scale);
  return 0;
}

/* ----------------------------------------------------------------------------------------------------- Psd */

/** @brief Allocate the window (n floats, designed as `wt`) and the FFT buffer (n + 1 floats); the allocations are not checked. */
Psd::Psd(unsigned n, WindowType wt, unsigned overlap)
{
  size_ = n;
  window_ = (float*)sysdep_malloc(n * 4u);
  buf_ = (float*)sysdep_malloc((size_ + 1u) * 4u);
  designWindow(wt, window_, size_);
  overlap_ = overlap;
}

Psd::~Psd()
{
  if (window_)
    sysdep_free(window_);
  if (buf_)
    sysdep_free(buf_);
}

/** @brief Set the number of samples shared by two segments. */
void Psd::setOverlapLength(unsigned overlap) { overlap_ = overlap; }

/** @brief Redesign the analysis window. */
void Psd::setWindowType(WindowType wt) { designWindow(wt, window_, size_); }

/**
 * @brief Averaged periodogram of `n` samples.
 * @param in   input samples
 * @param n    number of input samples
 * @param out  N/2 output bins
 * @param opt  output scaling, see the class description
 * @warning No checks; see the class description for the NaN cases (fewer than N samples, all-zero spectrum in kDbRelativeToMax) and the unsigned arithmetic.
 */
void Psd::process(float* in, unsigned n, float* out, OutputOption opt)
{
  const unsigned N = size_;
  const unsigned nseg = (n - overlap_) / (N - overlap_);
  const unsigned nbins = N >> 1;

  for (unsigned k = 0; k < nbins; k++)
    storeZeroWord(&out[k]);

  /* accumulate the power of every segment */
  unsigned off = 0;

  for (unsigned seg = 0; seg < nseg; seg++)
  {
    for (unsigned j = 0; j < N; j++)
      buf_[j + 1] = (float)((LD)in[off + j] * (LD)window_[j]);
    realfft(buf_, N, 1);
    for (unsigned k = 0; k < nbins; k++)
    {
      LD re = (LD)buf_[2 * k + 1];
      LD im = (LD)buf_[2 * k + 2];
      out[k] = (float)((re * re + im * im) + (LD)out[k]);
    }
    off += N - overlap_;
  }

  /* output scaling */
  if (opt == kDbRelativeToMax)
  {
    LD mx = 0;
    for (unsigned k = 0; k < nbins; k++)
    {
      LD v = (LD)out[k];
      if (v > mx)
        mx = v;
    }
    if (nbins != 0)
    {
      const LD recip = (LD)1 / mx;
      for (unsigned k = 0; k < nbins; k++)
      {
        LD x = (LD)out[k] * recip + (LD)1e-25;
        float l = (float)dsp_log10(x);
        out[k] = (float)((LD)l * (LD)10.0f);
      }
    }
  }
  else if (opt == kDb)
  {
    if (nbins != 0)
    {
      const LD recip = (LD)1 / (LD)(uint32_t)nseg;
      for (unsigned k = 0; k < nbins; k++)
      {
        LD x = (LD)out[k] * recip + (LD)1e-25;
        float l = (float)dsp_log10(x);
        out[k] = (float)((LD)l * (LD)10.0f);
      }
    }
  }
  else if (opt == kLinear)
  {
    if (nbins != 0)
    {
      const LD recip = (LD)1 / (LD)(uint32_t)nseg;
      for (unsigned k = 0; k < nbins; k++)
        out[k] = (float)((LD)out[k] * recip);
    }
  }
}

/** @brief Bin centre frequencies: out[k] = k * fs / N for k < N/2. */
void Psd::getFrequencies(float* out, float fs) const
{
  const unsigned nb = size_ >> 1;
  if (nb == 0)
    return;
  const LD recip = (LD)1 / (LD)(uint32_t)size_;
  for (unsigned k = 0; k < nb; k++)
    out[k] = (float)(((LD)k * (LD)fs) * recip);
}

/* ----------------------------------------------------------------------------------------------------- Agc */

/** @brief period 500 samples, target power 1.0, then reset(). */
template <class T> Agc<T>::Agc()
{
  period_ = 500;
  target_ = (T)1.0f;
  reset();
}

/** @brief Unity gain, unfrozen state (mix 1), clear the power and the accumulator, start a full period. */
template <class T> void Agc<T>::reset()
{
  mix_ = (T)1.0f;
  gain_ = (T)1.0f;
  mixSaved_ = (T)1.0f;
  storeZeroWord(&power_);
  storeZeroWord(&acc_);
  count_ = period_;
}

/** @brief Freeze the gain: remember mix_ in mixSaved_ and set mix_ to 1 (the gain update then stops). */
template <class T> void Agc<T>::freeze()
{
  T old;
  copyWord(&old, &mix_);
  mix_ = (T)1.0f;
  copyWord(&mixSaved_, &old);
}

/**
 * @brief Apply the current gain to `n` samples and, at the end of every period, update it.
 *
 * Per sample: acc += x^2 (rounded to float every time), out = gain * x.  When the period count runs out: power = acc / period, acc = 0, and, if not frozen and
 * power > 1e-10, gain = (1 - mix) * sqrt(target / power) + mix * gain.
 */
template <class T> void Agc<T>::process(const T* in, T* out, unsigned n)
{
  if (n == 0)
    return;
  const LD eps = (LD)1e-10f;
  unsigned count = count_;
  unsigned remaining = n;

  for (;;)
  {
    LD xv = (LD)*in;
    remaining--;
    count--;
    acc_ = (T)(xv * xv + (LD)acc_);
    *out = (T)((LD)gain_ * (LD)*in);
    in++;
    out++;

    /* end of a period: mean power, gain update */
    if (count == 0)
    {
      count = period_;
      LD meanpow = ((LD)1 / (LD)(uint32_t)period_) * (LD)acc_;
      LD m0 = (LD)mix_;
      acc_ = (T)0;
      bool update = (mix_ < (T)1.0f || mix_ > (T)1.0f); /* ZF = 0: ordered and different from 1.0 */
      power_ = (T)meanpow;
      if (update && meanpow > eps)
        gain_ = (T)((((LD)1 - m0) * dsp_sqrt(((LD)1 / meanpow) * (LD)target_)) + (m0 * (LD)gain_));
    }
    if (remaining == 0)
      break;
  }
  count_ = count;
}

/* ----------------------------------------------------------------------------------------------------- SineWave */

/** @brief Store the four parameters (as 32-bit words, so a signalling NaN is kept). */
template <class T, class U> SineWave<T, U>::SineWave(T amp, T freq, T phase, T fs)
{
  storeArgWord(&amp_, amp);
  storeArgWord(&freq_, freq);
  storeArgWord(&phase_, phase);
  storeArgWord(&fs_, fs);
}

template <class T, class U> SineWave<T, U>::~SineWave() {}

/**
 * @brief Generate `n` samples amp * sin(phase), advancing the phase by 2 pi freq / fs per sample, then wrap the phase to (-2 pi, 2 pi).
 *
 * The phase is carried across samples as a float, so its rounding accumulates within a block.
 */
template <class T, class U> void SineWave<T, U>::generate(T* out, unsigned long n)
{
  const LD twopi = (LD)6.2831855f;
  const LD dphi = ((LD)freq_ * twopi) / (LD)fs_;
  LD ph = (LD)phase_;

  for (unsigned long i = 0; i < n; i++)
  {
    if (i != 0)
      phase_ = (T)ph;
    out[i] = (T)(dsp_fsin(ph) * (LD)amp_);
    ph = (LD)phase_ + dphi;
  }

  /* wrap the final phase */
  int k = dsp_trunc_i32(ph * (LD)0.15915494f);
  phase_ = (T)(ph - (LD)k * twopi);
}

/* ----------------------------------------------------------------------------------------------------- explicit instantiations
 * (member-wise: exactly the functions that are used) */

template void boxcar<float>(float*, unsigned);
template void hanning<float>(float*, unsigned);
template void hamming<float>(float*, unsigned);
template void blackman<float>(float*, unsigned);
template void designWindow<float>(WindowType, float*, unsigned);
template float sinc<float>(float);
template float sum<float>(float*, unsigned);
template float mean<float>(float*, unsigned);
template float sqrSum<float>(float*, unsigned);
template float Var<float>(float*, unsigned);
template float Std<float>(float*, unsigned);
template long double sumLD<float>(const float*, unsigned);
template long double meanLD<float>(const float*, unsigned);
template long double sqrSumLD<float>(const float*, unsigned);
template long double VarLD<float>(const float*, unsigned);
template long double StdLD<float>(const float*, unsigned);

template LowPassFIR<float>::LowPassFIR(unsigned, float, WindowType, float);
template LowPassFIR<float>::~LowPassFIR();
template int LowPassFIR<float>::design(unsigned, float, WindowType, float);
template int LowPassFIR<float>::design(unsigned, float, float, const float*, int);

template Agc<float>::Agc();
template void Agc<float>::reset();
template void Agc<float>::freeze();
template void Agc<float>::process(const float*, float*, unsigned);

template SineWave<float, float>::SineWave(float, float, float, float);
template SineWave<float, float>::~SineWave();
template void SineWave<float, float>::generate(float*, unsigned long);
