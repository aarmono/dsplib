/**
 * @file dsp_window_fft_psd.h
 * @brief Window functions, simple statistics, a one-based FFT, and the classes LowPassFIR<float>, Psd, Agc<float> and SineWave<float, float>.
 *
 * Definitions and the explicit instantiations are in dsp_window_fft_psd.cpp.  The members are laid out so that the 32-bit layout is fixed (asserts generated
 * into dsp_window_fft_psd_layout.inc).  Users: LowPassFIR designs the polyphase tables of the resamplers (dsp_resampler.cpp); the V.90 spectral shaper uses Psd
 * (and through it realfft / four1), the V.90 demodulator uses Agc, the floating-point modem uses SineWave, and the V.90 demodulator and equalizer use the
 * statistics and the window functions.
 *
 * Numerics as in dsp_float_filters.cpp: float32 in memory, `long double` (x87 extended) for the intermediate values, and the transcendental functions
 * through core/dsp_math.h.  The five statistics functions (sum / mean / sqrSum / Var / Std) produce an UNROUNDED extended result: the float-returning
 * versions leave it in st(0) on i386, and the `...LD` functions return the same value as a `long double` on every host; code that needs the exact value
 * (e.g. LowPassFIR::design()) calls those.
 */
#ifndef DSP_CLASSES_P03_H
#define DSP_CLASSES_P03_H

#include "core/dsp_cxx.h"
#include "core/dsp_math.h" /* DSP_EXTRET_FN */

/** The window families of designWindow(); every other value gives a boxcar. */
enum WindowType
{
  WINDOW_BOXCAR = 0,
  WINDOW_HANNING = 1,
  WINDOW_HAMMING = 2,
  WINDOW_BLACKMAN = 3
};

/* ---- windows: w[0 .. n-1]; hanning is the n+1 flavour (never 0 / 1 at the ends), hamming / blackman the n-1 flavour (n == 1 gives NaN) ---- */
template <class T> void boxcar(T* w, unsigned n);
template <class T> void hanning(T* w, unsigned n);
template <class T> void hamming(T* w, unsigned n);
template <class T> void blackman(T* w, unsigned n);
template <class T> void designWindow(WindowType type, T* w, unsigned n);
template <class T> T sinc(T x); ///< sin(pi x) / (pi x), 1 for x == 0 (and for NaN); rounded to T

/* ---- statistics (n = 0 divides 0 / 0) ---- */
template <class T> DSP_EXTRET_FN T sum(T* p, unsigned n);    ///< extended-precision sum from +0, UNROUNDED result in st(0)
template <class T> DSP_EXTRET_FN T mean(T* p, unsigned n);   ///< sum / n
template <class T> DSP_EXTRET_FN T sqrSum(T* p, unsigned n); ///< mean of the squares (sum of squares / n)
template <class T> DSP_EXTRET_FN T Var(T* p, unsigned n);    ///< (T)sqrSum - mean^2, extended; can be slightly negative for near-constant data
template <class T> DSP_EXTRET_FN T Std(T* p, unsigned n);    ///< sqrt(Var), extended; NaN when Var is negative
/* the same values as a long double, portable across hosts */
template <class T> long double sumLD(const T* p, unsigned n);
template <class T> long double meanLD(const T* p, unsigned n);
template <class T> long double sqrSumLD(const T* p, unsigned n);
template <class T> long double VarLD(const T* p, unsigned n);
template <class T> long double StdLD(const T* p, unsigned n);

/* ---- FFT (Numerical Recipes, ONE-BASED arrays: data[1 .. 2 nn] / data[1 .. n]; data[0] is never touched) ---- */
void four1(float* data, unsigned long nn, int isign);  ///< complex FFT of nn points (a power of two), interleaved re / im in data[1 .. 2nn]
void realfft(float* data, unsigned long n, int isign); ///< real FFT of n points (n a power of two); isign 1 forward, otherwise inverse (x 1/2)

/**
 * LowPassFIR<T>: windowed-sinc low-pass design.
 *
 * `coef_` is a heap block of n_ coefficients that the object OWNS and frees (also when the caller handed the window in with adopt != 0).
 * design(n, fc, gain, win, adopt): fc in [0, 1] (normalised cut-off), n > 1; the window (a hamming window when win == NULL, copied when adopt == 0, adopted
 * when adopt != 0) is multiplied in place by fc * sinc(...), then the whole block is scaled so that its sum is `gain`.  Returns 0, or 1 for invalid
 * arguments (nothing is changed then).
 * @warning The constructor sets coef_ = 0 and calls design(); n_ stays uninitialised when design() rejects the arguments.  The four-argument design()
 *          allocates its window block itself and leaks it when it then rejects the arguments.
 */
template <class T> class LowPassFIR
{
public:
  LowPassFIR(unsigned n, T fc, WindowType wt, T gain);
  ~LowPassFIR();
  int design(unsigned n, T fc, WindowType wt, T gain); ///< designs the window itself; 1 if the allocation fails or the arguments are invalid
  int design(unsigned n, T fc, T gain, const T* win, int adopt);

  T* coef_;    ///< heap block of n_ coefficients (owned)
  unsigned n_; ///< number of coefficients
};

/**
 * Psd: averaged periodogram.
 *
 * `size_` is the segment length N (the FFT size, a power of two), `overlap_` the number of samples shared by two segments; `window_` is the analysis window,
 * `buf_` the one-based FFT buffer (N + 1 floats).  process(in, n, out, opt): nseg = (n - overlap) / (N - overlap) segments are windowed and transformed,
 * out[0 .. N/2-1] += |FFT|^2 per bin (bin 0 holds DC in its real part and Nyquist in the imaginary part, as realfft packs them), then
 *   kDb: out = 10 log10(out / nseg + 1e-25);  kDbRelativeToMax: out = 10 log10(out / max + 1e-25) (max over the bins, from 0);  kLinear: out = out / nseg;
 *   any other value: the sums are left as they are.
 * @warning process() has no argument checks: n < overlap makes nseg huge, N == overlap divides by zero, and a size that is not a power of two is not a valid
 *          FFT size.  With fewer than N input samples (nseg == 0) kDb / kLinear produce NaN (0 * infinity); an all-zero spectrum does the same in
 *          kDbRelativeToMax.
 */
class Psd
{
public:
  enum OutputOption
  {
    kDb = 0,
    kDbRelativeToMax = 1,
    kLinear = 2
  };

  Psd(unsigned n, WindowType wt, unsigned overlap);
  ~Psd();
  void setOverlapLength(unsigned overlap);
  void setWindowType(WindowType wt);
  void process(float* in, unsigned n, float* out, OutputOption opt);
  void getFrequencies(float* out, float fs) const; ///< out[k] = k * fs / N for k < N/2

  unsigned size_;    ///< segment length N
  unsigned overlap_; ///< samples shared by two segments
  float* window_;    ///< heap, N floats
  float* buf_;       ///< heap, N + 1 floats
};

/**
 * Agc<T>: block automatic gain control.
 *
 * Every period_ samples the mean power of the period is measured (power_); unless the object is frozen (mix_ == 1.0) the gain moves to
 * mix * gain + (1 - mix) * sqrt(target / power), and only when power > 1e-10.  process() applies the CURRENT gain to every sample (the gain changes only
 * between periods; the update on the last sample of a period takes effect from the next sample).
 */
template <class T> class Agc
{
public:
  Agc();
  void reset();  ///< mix = mixSaved = gain = 1, power = acc = 0, count = period
  void freeze(); ///< mixSaved = mix; mix = 1
  void process(const T* in, T* out, unsigned n);

  T mix_;           ///< weight of the old gain (1.0 = frozen)
  T mixSaved_;      ///< mix_ before freeze()
  T gain_;          ///< current gain
  T target_;        ///< target power (1.0 after the constructor)
  T power_;         ///< mean power of the last period
  T acc_;           ///< sum of squares of the current period
  unsigned period_; ///< samples per period (500 after the constructor)
  unsigned count_;  ///< samples left in the current period
};

/** SineWave<T, U>: sin() oscillator; the phase is kept as a float field and wrapped to (-2pi, 2pi) after every block. */
template <class T, class U> class SineWave
{
public:
  SineWave(T amp, T freq, T phase, T fs);
  ~SineWave();
  void generate(T* out, unsigned long n); ///< n samples amp * sin(phase), the phase advancing by 2 pi freq / fs per sample

  T amp_;   ///< amplitude
  T freq_;  ///< frequency
  T phase_; ///< radians
  T fs_;    ///< sample rate (same unit as freq_)
};

typedef LowPassFIR<float> P03_LowPassFIR;
typedef Psd P03_Psd;
typedef Agc<float> P03_Agc;
typedef SineWave<float, float> P03_SineWave;

#include "dsp_window_fft_psd_layout.inc"

#endif /* DSP_CLASSES_P03_H */
