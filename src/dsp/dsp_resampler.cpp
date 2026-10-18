/**
 * @file dsp_resampler.cpp
 * @brief Resampler, ResamplerTimingOffset, ResamplerTiming and V90Resampler (declarations in dsp_resampler.h).
 *
 * Resampler is a polyphase interpolating sample-rate converter: for each output it forms the dot products of the newest `taps` history samples with the
 * coefficient rows of the two phases around the current phase and interpolates linearly between them; the phase then advances by `step`, and the
 * input samples that the integer part of the phase consumed are taken in.  After every output the VIRTUAL timingCorrection() hook may change the phase:
 * ResamplerTimingOffset adds a constant offset (a clock error in ppm), ResamplerTiming runs a second-order timing-recovery loop on a band-passed copy of
 * the output, V90Resampler adds a gain schedule for that loop (setBllState(), gains read from the V90Parameters block) and a history of the measured offset
 * (V90Resampler::resample()).  Dot products are accumulated in extended precision; see dsp_float_filters.cpp for the numeric rules.
 *
 * Behaviour notes.
 *  - resample() reads ONE SAMPLE BEYOND the block it is given: the interpolation needs a look-ahead sample, and the code takes `*in` after the block has
 *    been consumed (it also stores that sample as a pending sample when it runs out of input); with n == 0 it reads in[0].  Callers must keep the sample
 *    after the block readable.
 *  - The look-ahead of an output is the sample at *in even when pending samples exist; the pending samples feed only the history.
 *  - The phase is clamped to 0 when the timing hook leaves it negative (or after the wrap loop); a NaN phase stays NaN and then selects a coefficient row
 *    outside the table.
 *  - V90Resampler::setBllState() stores its flag argument only when the state changes, before the state is looked up; every state above 15 skips the gain
 *    table (and the message); state 1 clears only gainB_.  The messages are printed with edprintf() and end in \r\n.
 *  - The constructors never test a malloc result; the float form designs its table with a LowPassFIR<float> on the stack and uses `taps & ~3`.
 *  - resetSdHalfBaudDft() does not clear the done flag (only ResamplerTiming::reset() does).
 */

#include "dsp_resampler.h"
#include "vpcm/pcm_parameters.h" /* V90Parameters */
#include "core/dsp_asm.h"
#include "core/dsp_host.h"
#include "v34/v34_api.h"

extern "C"
{
}

typedef long double LD;

/* ------------------------------------------------------------------------------------------------ small helpers */

#include "core/dsp_wordof_value.h"
#include "core/dsp_floatbits.h"
#include "core/dsp_copyword.h"
/* float constants (the hexadecimal comments are the bit patterns, which the code relies on being exactly these floats) */
static const float kMicro = 1e-6f;                       /* 0x358637bd: ppm to a fraction */
static const float kTimK = 0.9604f;                      /* 0x3f75dcc6: pole factor of the band-pass resonator */
static const float kTimC1 = 0.00025f;                    /* 0x3983126f: outer weights of the timing-error filter */
static const float kTimC2 = 0.0005f;                     /* 0x3a03126f: middle weight of the timing-error filter */
static const float kGainBase = 0.03981f;                 /* 0x3d230fd0: initial band-pass gain */
static const float kDegToCycle = 0.0055555556900799274f; /* 0x3bb60b61: 1/180 (addPhase) */

/* ------------------------------------------------------------------------------------------------ Resampler */

/* Constructor body, pointer form: the coefficient table is borrowed.  The history is `histLen` samples, or taps * 10 when histLen is not larger than taps. */
static inline __attribute__((always_inline)) void initResamplerP(Resampler* r, unsigned L, uint32_t stepWord, unsigned taps, float* coef, unsigned histLen)
{
  r->coef_ = coef;
  r->L_ = L;
  storeWord(&r->step_, stepWord);
  r->taps_ = taps;
  r->borrowed_ = 1;
  unsigned hl = (histLen > taps) ? histLen : taps * 10u;
  r->histLen_ = hl;
  float* h = 0;
  if (hl != 0)
    h = (float*)sysdep_malloc(hl * 4u);
  r->hist_ = h;
  if (h)
  {
    for (unsigned i = 0; i < hl; i++)
      storeWord(&h[i], 0u);
  }
  r->wr_ = taps;
  r->need_ = 0;
  r->phase_ = 0.0;
  for (unsigned i = 0; i < 5; i++)
    storeWord(&r->pend_[i], 0u);
  r->pendCount_ = 0;
}

/* Constructor body, float form: the table is designed here from a Blackman-windowed sinc low pass (a stack LowPassFIR<float>, dsp_window_fft_psd.h) of
 * cutoff / L and decomposed into its time-reversed polyphase rows; the history is `histLen` samples, or taps * 10 when histLen is smaller than taps. */
static inline __attribute__((always_inline)) void initResamplerF(Resampler* r, unsigned L, uint32_t stepWord, unsigned taps, uint32_t cutoffWord,
                                                                 unsigned histLen)
{
  storeWord(&r->step_, stepWord);
  r->L_ = L;
  unsigned t = taps & ~3u;
  r->taps_ = t;
  unsigned hl = (t < histLen) ? histLen : t * 10u;
  r->histLen_ = hl;
  float* h = 0;
  if (hl != 0)
    h = (float*)sysdep_malloc(hl * 4u);
  r->hist_ = h;
  if (h)
  {
    for (unsigned i = 0; i < hl; i++)
      storeWord(&h[i], 0u);
  }
  r->wr_ = t;
  r->need_ = 0;
  r->phase_ = 0.0;
  for (unsigned i = 0; i < 5; i++)
    storeWord(&r->pend_[i], 0u);
  r->pendCount_ = 0;
  {
    LD Lf = (LD)(uint32_t)L; /* exact */
    float cutoff = bitsFloat(cutoffWord);
    float fc = (float)((LD)cutoff / Lf); /* the cutoff divided by L in extended precision */
    float gain = (float)Lf;              /* passband gain L */
    P03_LowPassFIR lpf(L * t, fc, WINDOW_BLACKMAN, gain);
    float* c = (float*)sysdep_malloc(t * L * 4u);
    r->coef_ = c;
    r->borrowed_ = 0;
    if (c)
    {
      const float* src = lpf.coef_;
      unsigned base = 0;
      for (unsigned p = 0; p < L; p++)
      {
        unsigned tt = r->taps_;
        if (tt > 0)
        {
          unsigned s = (tt - 1) * L + p;
          for (unsigned k = 0; k < tt; k++)
          {
            copyWord(&c[base + k], &src[s]);
            s -= L;
          }
        }
        base += tt;
      }
    }
  } /* ~LowPassFIR */
}

/** @brief Resampler with a designed table (cutoff relative to the input rate): see the class description. */
Resampler::Resampler(unsigned L, float step, unsigned taps, float cutoff, unsigned histLen)
{
  initResamplerF(this, L, wordOf(step), taps, wordOf(cutoff), histLen);
}
/** @brief Resampler with a borrowed coefficient table of L * taps floats. */
Resampler::Resampler(unsigned L, float step, unsigned taps, float* coef, unsigned histLen) { initResamplerP(this, L, wordOf(step), taps, coef, histLen); }
Resampler::Resampler(Words, unsigned L, uint32_t stepWord, unsigned taps, uint32_t cutoffWord, unsigned histLen)
{
  initResamplerF(this, L, stepWord, taps, cutoffWord, histLen);
}
Resampler::Resampler(Words, unsigned L, uint32_t stepWord, unsigned taps, float* coef, unsigned histLen)
{
  initResamplerP(this, L, stepWord, taps, coef, histLen);
}

/** @brief Free the history and, unless it was borrowed, the coefficient table. */
Resampler::~Resampler()
{
  if (hist_)
    sysdep_free(hist_);
  if (!borrowed_)
  {
    if (coef_)
      sysdep_free(coef_);
  }
}

/** @brief Zero the history, restart the phase and the write index, drop the pending samples. */
void Resampler::reset()
{
  if (hist_)
  {
    unsigned n = histLen_;
    for (unsigned i = 0; i < n; i++)
      storeWord(&hist_[i], 0u);
  }
  need_ = 0;
  unsigned t = taps_;
  phase_ = 0.0;
  wr_ = t;
  for (unsigned i = 0; i < 5; i++)
    storeWord(&pend_[i], 0u);
  pendCount_ = 0;
}

/** @brief Restart writing at index taps_. */
void Resampler::resetHistoryIndex() { wr_ = taps_; }

/** @brief Copy the newest taps_ history samples (the last taps_ of the block) to the front. */
void Resampler::copyHistoryTail()
{
  unsigned t = taps_;
  if (t > 0)
  {
    float* h = hist_;
    unsigned d = histLen_ - t;
    for (unsigned i = 0; i < t; i++)
      copyWord(&h[i], &h[d + i]);
  }
}

static inline float normalizedPhase(const Resampler* r) { return (float)(r->phase_ / (LD)(uint32_t)r->L_); }

/** @brief The phase as a fraction of an input sample, [0, 1). */
float Resampler::getNormalizedPhase() const { return normalizedPhase(this); }

/** @brief Set the phase from a fraction of an input sample: 0 <= f < 1 gives f * L, anything else (including NaN) gives 0. */
void Resampler::setNormalizedPhase(float f)
{
  LD v = (LD)f;
  if (!(v >= 0.0L))
  { /* jb after fcom with 0.0: below or unordered */
    phase_ = 0.0;
  }
  else if (v < 1.0L)
  {
    phase_ = (double)(v * (LD)(uint32_t)L_);
  }
  else
  {
    phase_ = 0.0;
  }
}

/* Four-term unrolled dot product: two extended accumulators, `even` takes terms 0 and 2 of every group of four and the whole remainder, `odd` terms 1 and 3;
 * the result is odd + even.  Products of two floats are exact in the extended format. */
static inline LD dotEvenOdd(const float* a, const float* c, unsigned n)
{
  LD even = 0.0L, odd = 0.0L;
  while (n > 3)
  {
    even += (LD)a[0] * (LD)c[0];
    odd += (LD)a[1] * (LD)c[1];
    even += (LD)a[2] * (LD)c[2];
    odd += (LD)a[3] * (LD)c[3];
    a += 4;
    c += 4;
    n -= 4;
  }
  while (n != 0)
  {
    even += (LD)a[0] * (LD)c[0];
    a++;
    c++;
    n--;
  }
  return odd + even;
}

/**
 * @brief Resample a block.
 * @param in    input samples
 * @param n     number of input samples
 * @param out   output samples (the number is not bounded here: about n * L / step_ at most, plus one)
 * @param nOut  receives the number of output samples written
 * @warning Reads in[n], one sample beyond the block (the look-ahead sample; in[0] when n == 0), and keeps a copy of it as a pending sample when the block
 *          runs out.  If the timing hook leaves a NaN phase, the coefficient row selected from it lies outside the table.
 *
 * Loop: while at least two samples (new plus pending) are available, take `need_` samples into the history (the pending ones first), form one output from
 * the two adjacent phases, advance the phase by `step_`, call the timing hook and normalise the phase: whole turns of L become the number of input samples
 * to take before the next output (`need_`), a negative phase is clamped to 0.  When the input runs out in the middle of a step the missing count is
 * remembered and the look-ahead sample is stored as pending.
 */
void Resampler::resample(const float* in, unsigned n, float* out, unsigned& nOut)
{
  nOut = 0;
  unsigned total = n + pendCount_;

  while (total > 1)
  {
    unsigned need = need_;
    unsigned k = (need <= total) ? need : total;

    if (k != 0)
    {
      /* take k samples into the history (the pending ones first); the cached values are loaded once per outer iteration */
      float* h = hist_;
      unsigned pc = pendCount_;
      unsigned hl = histLen_;
      unsigned w = wr_;
      for (unsigned i = 0; i < k; i++)
      {
        if (pc == 0)
        {
          copyWord(&h[w], in);
          ++in;
        }
        else
        {
          copyWord(&h[w], &pend_[0]);
          pc = pc - 1;
          pendCount_ = pc;
          for (int j = 0; j < (int)pc; j++)
            copyWord(&pend_[j], &pend_[j + 1]);
        }
        ++w;
        if (w == hl)
        {
          unsigned t = taps_;
          if (t != 0)
          {
            unsigned d = hl - t;
            for (unsigned j = 0; j < t; j++)
              copyWord(&h[j], &h[d + j]);
          }
          w = t;
        }
      }
      wr_ = w;
    }

    if (need > total)
    {
      need_ = need - total;
      unsigned c = pendCount_;
      pendCount_ = c + 1;
      copyWord(&pend_[c], in); /* *in: one past the consumed block */
      return;
    }
    total -= need;

    /* one output sample: the two adjacent phases and the linear interpolation between them */
    LD phase = (LD)phase_;
    int idx = dsp_trunc_i32(phase);
    unsigned t = taps_;
    const float* coef = coef_;
    unsigned w = wr_;
    float* h = hist_;
    const float* win = h + (w - t);
    LD s0 = dotEvenOdd(win, coef + (unsigned)idx * t, t);
    float y0 = (float)s0;
    LD s1;
    if ((unsigned)(idx + 1) < L_)
    {
      s1 = dotEvenOdd(win, coef + (unsigned)(idx + 1) * t, t);
    }
    else
    {
      copyWord(&h[w], in); /* the look-ahead sample, the write index is not advanced */
      s1 = dotEvenOdd(h + (w - t) + 1, coef, t);
    }
    float y1 = (float)s1;
    float fracf = (float)(phase - (LD)idx);
    LD d = (LD)y1 - (LD)y0;
    LD o = (LD)y0 + d * (LD)fracf;
    float of = (float)o;
    *out = of;
    nOut = nOut + 1;
    ++out;
    phase_ = (double)(phase + (LD)step_);
    this->timingCorrection(of); /* VIRTUAL, vtable slot 3 */

    /* normalise the phase: whole turns of L are input samples to take next */
    need_ = 0;
    LD Lf = (LD)(uint32_t)L_;
    LD p = (LD)phase_; /* the hook may have changed it */
    if (!(p >= Lf))
    { /* jb: below or unordered */
      if (0.0L > p)
        phase_ = 0.0;
    }
    else
    {
      unsigned cnt = 0;
      do
      {
        cnt++;
        p -= Lf;
      } while (p >= Lf);
      need_ = cnt;
      phase_ = (double)p;
      if (0.0L > p)
        phase_ = 0.0;
    }
  }

  /* end of the block: keep the look-ahead sample as pending */
  {
    unsigned c = pendCount_;

    pendCount_ = c + 1;
    copyWord(&pend_[c], in);
  }
}

/* ------------------------------------------------------------------------------------------------ ResamplerTimingOffset */

/* Constructor body: reset the base, then off_ = ppm * step * 1e-6. */
static inline __attribute__((always_inline)) void offsetInit(ResamplerTimingOffset* o, uint32_t ppmWord)
{
  o->Resampler::reset();
  float ppm = bitsFloat(ppmWord);
  o->off_ = (float)(((LD)ppm * (LD)o->step_) * (LD)kMicro);
}

/** @brief Resampler with a designed table and a constant timing offset of `ppm` parts per million of the step. */
ResamplerTimingOffset::ResamplerTimingOffset(unsigned L, float step, unsigned taps, float cutoff, float ppm, unsigned histLen)
    : Resampler(Words(), L, wordOf(step), taps, wordOf(cutoff), histLen)
{
  offsetInit(this, wordOf(ppm));
}
/** @brief The same with a borrowed coefficient table. */
ResamplerTimingOffset::ResamplerTimingOffset(unsigned L, float step, unsigned taps, float* coef, float ppm, unsigned histLen)
    : Resampler(Words(), L, wordOf(step), taps, coef, histLen)
{
  offsetInit(this, wordOf(ppm));
}
ResamplerTimingOffset::ResamplerTimingOffset(Words, unsigned L, uint32_t stepWord, unsigned taps, uint32_t cutoffWord, uint32_t ppmWord, unsigned histLen)
    : Resampler(Resampler::Words(), L, stepWord, taps, cutoffWord, histLen)
{
  offsetInit(this, ppmWord);
}
ResamplerTimingOffset::ResamplerTimingOffset(Words, unsigned L, uint32_t stepWord, unsigned taps, float* coef, uint32_t ppmWord, unsigned histLen)
    : Resampler(Resampler::Words(), L, stepWord, taps, coef, histLen)
{
  offsetInit(this, ppmWord);
}

ResamplerTimingOffset::~ResamplerTimingOffset() {}

/** @brief Resampler::reset() and clear the offset. */
void ResamplerTimingOffset::reset()
{
  Resampler::reset();
  storeWord(&off_, 0u);
}

/** @brief Timing hook: advance the phase by the constant offset (the sample argument is ignored). */
void ResamplerTimingOffset::timingCorrection(float) { phase_ = (double)((LD)phase_ + (LD)off_); }

/** @brief Set the constant offset to ppm * step * 1e-6 phase units per output sample. */
void ResamplerTimingOffset::setTimingOffset(float ppm) { off_ = (float)(((LD)step_ * (LD)ppm) * (LD)kMicro); }

static inline LD timingOffsetPPM(const ResamplerTimingOffset* o) { return ((LD)1000000.0f * (LD)o->off_) / (LD)o->step_; }
/** @brief The offset in ppm of the step (1e6 * off_ / step_), unrounded. */
long double ResamplerTimingOffset::getTimingOffsetPPM_LD() const { return timingOffsetPPM(this); }

/** @brief The same as a float return (the unrounded value in st(0) on i386, see the header). */
float ResamplerTimingOffset::getTimingOffsetPPM() const { return dsp_ret_ext_float(getTimingOffsetPPM_LD()); }

/* ------------------------------------------------------------------------------------------------ ResamplerTiming */

/* The loop-state clear shared by the constructors and reset(unsigned); the band-pass gain returns to its initial 0.03981. */
static inline __attribute__((always_inline)) void timingClear(ResamplerTiming* t)
{
  t->parity_ = 0;
  storeWord(&t->gainA_, 0u);
  storeWord(&t->gainB_, 0u);
  storeWord(&t->errOut_, 0u);
  storeWord(&t->f58_, 0u);
  storeWord(&t->lastCorr_, 0u);
  storeWord(&t->energy_, 0u);
  storeWord(&t->energyPrev_, 0u);
  storeWord(&t->bpf_, 0u);
  storeWord(&t->bpfPrev_, 0u);
  storeWord(&t->errAcc_, 0u);
  t->dftN_ = 0;
  storeWord(&t->dftMag_, 0u);
  storeWord(&t->dftRe_, 0u);
  storeWord(&t->dftIm_, 0u);
  t->dftDone_ = 0;
  storeWord(&t->bpfGain_, 0x3d230fd0u);
}

/* Constructor body: reset the offset class, clear the loop state, set the offset. */
static inline __attribute__((always_inline)) void timingInit(ResamplerTiming* t, uint32_t ppmWord)
{
  t->ResamplerTimingOffset::reset();
  timingClear(t);
  t->setTimingOffset(bitsFloat(ppmWord));
}

/** @brief Timing-recovery resampler with a designed table; `ppm` is the initial constant offset. */
ResamplerTiming::ResamplerTiming(unsigned L, float step, unsigned taps, float cutoff, float ppm, unsigned histLen)
    : ResamplerTimingOffset(Words(), L, wordOf(step), taps, wordOf(cutoff), wordOf(ppm), histLen)
{
  timingInit(this, wordOf(ppm));
}
/** @brief The same with a borrowed coefficient table. */
ResamplerTiming::ResamplerTiming(unsigned L, float step, unsigned taps, float* coef, float ppm, unsigned histLen)
    : ResamplerTimingOffset(Words(), L, wordOf(step), taps, coef, wordOf(ppm), histLen)
{
  timingInit(this, wordOf(ppm));
}
ResamplerTiming::ResamplerTiming(Words, unsigned L, uint32_t stepWord, unsigned taps, uint32_t cutoffWord, uint32_t ppmWord, unsigned histLen)
    : ResamplerTimingOffset(ResamplerTimingOffset::Words(), L, stepWord, taps, cutoffWord, ppmWord, histLen)
{
  timingInit(this, ppmWord);
}
ResamplerTiming::ResamplerTiming(Words, unsigned L, uint32_t stepWord, unsigned taps, float* coef, uint32_t ppmWord, unsigned histLen)
    : ResamplerTimingOffset(ResamplerTimingOffset::Words(), L, stepWord, taps, coef, ppmWord, histLen)
{
  timingInit(this, ppmWord);
}

ResamplerTiming::~ResamplerTiming() {}

/** @brief Reset the offset class and clear the loop state (gains, errors, filter state, DFT bin). */
void ResamplerTiming::reset(unsigned)
{
  ResamplerTimingOffset::reset();
  timingClear(this);
}

/** @brief Restart the half-baud DFT bin (the done flag is not touched). */
void ResamplerTiming::resetSdHalfBaudDft()
{
  dftN_ = 0;
  storeWord(&dftMag_, 0u);
  storeWord(&dftRe_, 0u);
  storeWord(&dftIm_, 0u);
}

/**
 * @brief Timing hook: one step of the second-order timing-recovery loop, called after every output sample `x`.
 *
 * The sample is scaled by `bpfGain_` and passed through a resonator y[n] = gain * x[n] - 0.9604 * y[n-2] (a band-pass at a quarter of the output rate).  The
 * energies e[n] = y[n]^2 give the timing-error estimate R = 0.00025 * e[n] - 0.0005 * e[n-1] + 0.00025 * e[n-2].  The loop works on pairs of samples:
 * on the even call the phase just advances by `off_` and -R is kept in `errAcc_`; on the odd call Q = R + errAcc_ (half of it, H, is the error of the
 * pair) updates the integrator off_ += gainB_ * H and the phase advances by gainA_ * H + off_.
 */
void ResamplerTiming::timingCorrection(float x)
{
  uint32_t oldBpf;

  /* band-pass resonator and its energy */
  __builtin_memcpy(&oldBpf, &bpf_, sizeof oldBpf);
  LD kb = (LD)kTimK * (LD)bpfPrev_;

  storeWord(&bpfPrev_, oldBpf);
  LD gx = (LD)bpfGain_ * (LD)x;
  LD energyOld = (LD)energy_;
  LD E = gx - kb;

  bpf_ = (float)E;
  LD sq = E * E;

  energy_ = (float)sq;

  /* timing-error estimate from three consecutive energies */
  LD t1 = sq * (LD)kTimC1;
  LD t2 = (LD)kTimC1 * (LD)energyPrev_;
  LD t3 = energyOld * (LD)kTimC2;
  energyPrev_ = (float)energyOld;
  LD R = (t1 - t3) + t2;
  errOut_ = (float)R;

  /* even call: advance by the offset; odd call: the pair update of integrator and phase */
  unsigned par = parity_;

  if ((par & 1u) == 0)
  {
    LD off = (LD)off_;
    parity_ = 0;
    lastCorr_ = (float)off;
    errAcc_ = (float)(-R);
    parity_ = 1;
    phase_ = (double)((LD)phase_ + off);
  }
  else
  {
    LD Q = R + (LD)errAcc_;
    parity_ = par + 1;
    errAcc_ = (float)Q;
    LD H = Q * (LD)0.5f;
    LD hb = (LD)gainB_ * H;
    LD ha = (LD)gainA_ * H;
    LD U = hb + (LD)off_;
    LD V = ha + U;
    off_ = (float)U;
    lastCorr_ = (float)V;
    phase_ = (double)((LD)phase_ + V);
  }
}

/**
 * @brief Accumulate one sample into the half-baud DFT bin (a quarter of the output rate): +x, -j x, -x, +j x for successive samples.
 *
 * After 256 samples the magnitude is stored in `dftMag_` and `dftDone_` is set; further samples are ignored until the bin is restarted.
 */
void ResamplerTiming::SdHalfBaudDft(float x)
{
  if (dftDone_ != 0)
    return;
  unsigned n = dftN_;
  LD xv = (LD)x;

  switch (n & 3u)
  {
  case 0:
    dftRe_ = (float)(xv + (LD)dftRe_);
    break;
  case 1:
    dftIm_ = (float)((LD)dftIm_ - xv);
    break;
  case 2:
    dftRe_ = (float)((LD)dftRe_ - xv);
    break;
  default:
    dftIm_ = (float)(xv + (LD)dftIm_);
    break;
  }

  n = n + 1;
  dftN_ = n;
  if (n == 0x100u)
  {
    LD re = (LD)dftRe_, im = (LD)dftIm_;
    dftDone_ = 1;
    dftMag_ = (float)dsp_sqrt(re * re + im * im);
  }
}

/* abs() of an int32 by sign mask: INT_MIN stays INT_MIN */
static inline int absI32(int v)
{
  unsigned u = (unsigned)v;
  unsigned m = (unsigned)(v >> 31);
  return (int)((u ^ m) - m);
}

/* The debug print of one value in the form sign, integer part, three decimals: the sign is '-' when 0 >= f ('+' for 0 < f or unordered, f being the stored
 * float), then |trunc(v)| and |trunc((v - trunc(v)) * 1000)| of the extended value (out of range and NaN convert to 0x80000000). */
static inline void printValue(const char* fmt, LD v, float f)
{
  int sign = (0.0L >= (LD)f) ? '-' : '+';
  LD av = (v < 0.0L) ? -v : v; /* fabs */
  int ip = dsp_trunc_i32(av);
  int tv = dsp_trunc_i32(v);
  int fp = absI32(dsp_trunc_i32((v - (LD)tv) * (LD)1000.0));
  dsplibs_debug_printf(fmt, sign, ip, fp);
}

/**
 * @brief Set the band-pass gain from the half-baud DFT bin, once the bin is complete (nothing happens before).
 * @param x  scale applied to the bin magnitude
 *
 * P = x * dftMag_ replaces dftMag_; N = 350000 / P selects a factor Y: 1 when P is 0 or N > 4, 2 when 2 < N <= 4, otherwise N if N >= 1 (or NaN) and 1 if
 * N < 1.  bpfGain_ = Y * 0.03981.  Prints three debug lines at debug level above 1.
 * @note The mapping is not monotonic: a weak bin (N > 4) gives the factor 1, a stronger one (2 < N <= 4) the factor 2; it reads like an inverted clamp.
 */
void ResamplerTiming::adjustHalfBaudBpfGain(float x)
{
  if (dftDone_ == 0)
    return;

  LD P = (LD)x * (LD)dftMag_;

  dftMag_ = (float)P;

  /* normalisation factor Y from N = 350000 / P */
  LD Y;

  if (P > 0.0L || P < 0.0L)
  {
    LD N = ((LD)1.0L / P) * (LD)350000.0f;
    if (N > 4.0L)
    {
      Y = 1.0L;
    }
    else if (!(N > 2.0L))
    {                                       /* N <= 2 or NaN */
      Y = (1.0L <= N || N != N) ? N : 1.0L; /* fcom: 1 <= N, or unordered, keeps N; N < 1 gives 1 */
    }
    else
    {
      Y = 2.0L;
    }
  }
  else
  {
    Y = 1.0L;
  }
  bpfGain_ = (float)(Y * (LD)kGainBase);

  /* debug output */
  if (dsplibs_debug_level <= 1)
    return;
  /* the printed values: P (extended; the sign from the stored float), (float)Y, the stored bpfGain_ */
  printValue("ResamplerTiming::adjustHalfBaudBpfGain  baud/2 dft bin = %c%d.%03d\r\n", P, dftMag_);
  float yf = (float)Y;
  if (dsplibs_debug_level <= 1)
    return;
  printValue("ResamplerTiming::adjustHalfBaudBpfGain  normFactor = %c%d.%03d\r\n", (LD)yf, yf);
  if (dsplibs_debug_level <= 1)
    return;
  float g = bpfGain_;
  printValue("ResamplerTiming::adjustHalfBaudBpfGain  normBPFhBaudB0coef = %c%d.%03d\r\n", (LD)g, g);
}

/** @brief Advance the phase by one step, normalising it (not called anywhere in this tree). */
void ResamplerTiming::invertPhase()
{
  LD P = (LD)phase_ + (LD)step_;
  LD Lf = (LD)(uint32_t)L_;
  if (P >= Lf)
  { /* fcom + jb: below Lf or unordered skips the loop */
    unsigned n = need_;
    do
    {
      P -= Lf;
      n = n + 1;
    } while (Lf <= P);
    need_ = n;
  }
  phase_ = (double)P;
}

/** @brief Advance the phase by a * step / 180 when a > 0, normalising it (not called anywhere in this tree). */
void ResamplerTiming::addPhase(float a)
{
  LD av = (LD)a;
  if (!(av > 0.0L))
    return; /* jbe after fcom with 0.0: <= or unordered */
  LD t = av * (LD)step_;
  LD P = (LD)phase_ + t * (LD)kDegToCycle;
  LD Lf = (LD)(uint32_t)L_;
  if (P >= Lf)
  { /* fcom + jb: below Lf or unordered skips the loop */
    unsigned n = need_;
    do
    {
      P -= Lf;
      n = n + 1;
    } while (P >= Lf);
    need_ = n;
  }
  phase_ = (double)P;
}

/* ------------------------------------------------------------------------------------------------ V90Resampler */

/* Constructor body: allocate the timing-offset history (timingHistoryEvaluationBufferLength floats), start in the
 * FROZEN gain state and set the initial offset. */
static inline __attribute__((always_inline)) void v90Init(V90Resampler* v, V90Parameters* params, uint32_t ppmWord)
{
  v->params_ = params;
  v->tHist_ = 0;
  unsigned cnt = (unsigned)params->timingHistoryEvaluationBufferLength;
  float* th = (float*)sysdep_malloc(cnt * 4u);
  v->tHist_ = th;
  v->tHistLen_ = (unsigned)params->timingHistoryEvaluationBufferLength;
  v->ResamplerTiming::reset(1);
  v->stateCount_ = 0;
  v->state_ = 1;
  v->setBllState(V90BLL_FROZEN, 1);
  {
    unsigned n = v->tHistLen_;
    float* p = v->tHist_;
    for (unsigned i = 0; i < n; i++)
      storeWord(&p[i], 0u);
  }
  v->tHistIdx_ = 0;
  v->tCount_ = 0;
  v->setTimingOffset(bitsFloat(ppmWord));
}

/** @brief V.90 receiver resampler with a designed table; `params` is the borrowed parameter block (gains, history length). */
V90Resampler::V90Resampler(unsigned L, float step, unsigned taps, float cutoff, V90Parameters* params, float ppm, unsigned histLen)
    : ResamplerTiming(Words(), L, wordOf(step), taps, wordOf(cutoff), wordOf(ppm), histLen)
{
  v90Init(this, params, wordOf(ppm));
}
/** @brief The same with a borrowed coefficient table. */
V90Resampler::V90Resampler(unsigned L, float step, unsigned taps, float* coef, V90Parameters* params, float ppm, unsigned histLen)
    : ResamplerTiming(Words(), L, wordOf(step), taps, coef, wordOf(ppm), histLen)
{
  v90Init(this, params, wordOf(ppm));
}

/** @brief Free the timing-offset history. */
V90Resampler::~V90Resampler()
{
  if (tHist_)
    sysdep_free(tHist_);
}

/** @brief Reset the loop state, enter the FROZEN gain state and clear the timing-offset history. */
void V90Resampler::reset()
{
  ResamplerTiming::reset(1);
  stateCount_ = 0;
  state_ = 1;
  setBllState(V90BLL_FROZEN, 1);
  {
    unsigned n = tHistLen_;
    if (n > 0)
    {
      float* p = tHist_;
      for (unsigned i = 0; i < n; i++)
        storeWord(&p[i], 0u);
    }
  }
  tHistIdx_ = 0;
  tCount_ = 0;
}

/**
 * @brief Switch the timing-loop gain schedule to `state`.
 * @param state  new state; nothing happens when it equals the current one
 * @param flag   stored in `stateFlag_` on a change; while non-zero, resample() counts input samples in `stateCount_`
 *
 * On a change to a state of 0 to 15 the loop gains gainA_ / gainB_ are loaded from the V90Parameters block (K1 and K2 of the state's BLL_<name> keys; FROZEN
 * clears both, SECOND ORDER FROZEN only gainB_) and the state name is printed with edprintf().  A state above 15 changes only `state_` and `stateCount_`.
 * @note The K2 gain of TRN1_QC_SLOW (bllTrn1QcSlowK2) is never loaded from the parameter file, so it keeps its default (src/vpcm/pcm_parameters_load.cpp).
 */
void V90Resampler::setBllState(V90BllState state, unsigned flag)
{
  unsigned s = (unsigned)state;
  if (state_ == s)
    return;
  stateFlag_ = flag;
  if (s <= 15u)
  {
    const V90Parameters* p = params_;
    switch (s)
    {
    case V90BLL_FROZEN:
      storeWord(&gainA_, 0u);
      storeWord(&gainB_, 0u);
      edprintf("V90Resampler: state = FROZEN\r\n");
      break;
    case V90BLL_SECOND_ORDER_FROZEN:
      storeWord(&gainB_, 0u);
      edprintf("V90Resampler: state = SECOND ORDER FROZEN\r\n");
      break;
    case V90BLL_INITIAL:
      storeWord(&gainA_, wordOf(p->bllInitialK1));
      storeWord(&gainB_, wordOf(p->bllInitialK2));
      edprintf("V90Resampler: state = INITIAL\r\n");
      break;
    case V90BLL_FAST:
      storeWord(&gainA_, wordOf(p->bllFastK1));
      storeWord(&gainB_, wordOf(p->bllFastK2));
      edprintf("V90Resampler: state = FAST\r\n");
      break;
    case V90BLL_MEDIUM:
      storeWord(&gainA_, wordOf(p->bllMediumK1));
      storeWord(&gainB_, wordOf(p->bllMediumK2));
      edprintf("V90Resampler: state = MEDIUM\r\n");
      break;
    case V90BLL_SLOW:
      storeWord(&gainA_, wordOf(p->bllSlowK1));
      storeWord(&gainB_, wordOf(p->bllSlowK2));
      edprintf("V90Resampler: state = SLOW\r\n");
      break;
    case V90BLL_SLOW2:
      storeWord(&gainA_, wordOf(p->bllSlow2K1));
      storeWord(&gainB_, wordOf(p->bllSlow2K2));
      edprintf("V90Resampler: state = SLOW2\r\n");
      break;
    case V90BLL_DIL:
      storeWord(&gainA_, wordOf(p->bllDilK1));
      storeWord(&gainB_, wordOf(p->bllDilK2));
      edprintf("V90Resampler: state = DIL\r\n");
      break;
    case V90BLL_TRN2_INITIAL:
      storeWord(&gainA_, wordOf(p->bllTrn2InitialK1));
      storeWord(&gainB_, wordOf(p->bllTrn2InitialK2));
      edprintf("V90Resampler: state = TRN2 INITIAL\r\n");
      break;
    case V90BLL_TRN2:
      storeWord(&gainA_, wordOf(p->bllTrn2K1));
      storeWord(&gainB_, wordOf(p->bllTrn2K2));
      edprintf("V90Resampler: state = TRN2 \r\n");
      break;
    case V90BLL_STEADY_STATE:
      storeWord(&gainA_, wordOf(p->bllSteadyStateK1));
      storeWord(&gainB_, wordOf(p->bllSteadyStateK2));
      edprintf("V90Resampler: state = STEADY_STATE\r\n");
      break;
    case V90BLL_PRE_ANSPCM:
      storeWord(&gainA_, wordOf(p->bllPreAnspcmK1));
      storeWord(&gainB_, wordOf(p->bllPreAnspcmK2));
      edprintf("V90Resampler: state = PRE_ANSPCM\r\n");
      break;
    case V90BLL_TRN1_QC_INITIAL:
      storeWord(&gainA_, wordOf(p->bllTrn1QcInitialK1));
      storeWord(&gainB_, wordOf(p->bllTrn1QcInitialK2));
      edprintf("V90Resampler: state = TRN1_QC_INITIAL\r\n");
      break;
    case V90BLL_TRN1_QC_FAST:
      storeWord(&gainA_, wordOf(p->bllTrn1QcFastK1));
      storeWord(&gainB_, wordOf(p->bllTrn1QcFastK2));
      edprintf("V90Resampler: state = TRN1_QC_FAST\r\n");
      break;
    case V90BLL_TRN1_QC_MEDIUM:
      storeWord(&gainA_, wordOf(p->bllTrn1QcMediumK1));
      storeWord(&gainB_, wordOf(p->bllTrn1QcMediumK2));
      edprintf("V90Resampler: state = TRN1_QC_MEDIUM\r\n");
      break;
    default: /* 15 */
      storeWord(&gainA_, wordOf(p->bllTrn1QcSlowK1));
      storeWord(&gainB_, wordOf(p->bllTrn1QcSlowK2));
      edprintf("V90Resampler: state = TRN1_QC_SLOW\r\n");
      break;
    }
  }
  state_ = s;
  stateCount_ = 0;
}

/**
 * @brief Resampler::resample() plus bookkeeping: count input samples for the state timer, and, when TIMING_HISTORY_EVALUATION_ENABLED, store the current timing
 *        offset in ppm into `tHist_` every TIMING_HISTORY_EVALUATION_PERIOD input samples (circular).
 * @warning The history length is TIMING_HISTORY_EVALUATION_BUFFER_LENGTH from the parameter block: with evaluation enabled and a length of 0 the store goes
 *          into a zero-size block.  Same read-ahead as Resampler::resample().
 */
void V90Resampler::resample(const float* in, unsigned n, float* out, unsigned& nOut)
{
  Resampler::resample(in, n, out, nOut);

  /* state timer */
  if (stateFlag_ != 0)
    stateCount_ += n;

  /* timing-offset history */
  const V90Parameters* p = params_;

  if (p->timingHistoryEvaluationEnabled != 0)
  {
    unsigned c = tCount_ + n;
    tCount_ = c;
    if (c >= (unsigned)p->timingHistoryEvaluationPeriod)
    {
      tCount_ = 0;
      LD ppm = getTimingOffsetPPM_LD();
      unsigned i = tHistIdx_;
      float* th = tHist_;
      th[i] = (float)ppm;
      unsigned ni = i + 1;
      if (ni == tHistLen_)
        ni = 0;
      tHistIdx_ = ni;
    }
  }
}

/** @brief Mean of the timing-offset history (unrounded). */
long double V90Resampler::getTimingHistoryMeanLD() { return meanLD<float>(tHist_, tHistLen_); }

/** @brief Standard deviation of the timing-offset history: the square root of |variance| (unrounded). */
long double V90Resampler::getTimingHistoryStdLD()
{
  LD v = VarLD<float>(tHist_, tHistLen_);
  LD k = (v >= 0.0L) ? 1.0L : -1.0L; /* fcoms 0.0 / jb: below or unordered -> -1.0f */
  OPTIMIZER_HIDE_MEM(k);
  return dsp_sqrt(k * v);
}

/** @brief The mean as a float return (unrounded in st(0) on i386). */
float V90Resampler::getTimingHistoryMean() { return dsp_ret_ext_float(getTimingHistoryMeanLD()); }
/** @brief The standard deviation as a float return (unrounded in st(0) on i386). */
float V90Resampler::getTimingHistoryStd() { return dsp_ret_ext_float(getTimingHistoryStdLD()); }
