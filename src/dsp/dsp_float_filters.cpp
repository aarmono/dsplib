/**
 * @file dsp_float_filters.cpp
 * @brief FloatFIR, FloatIIR, FloatARMA and GenericIIR<float, double>: float block filters with a downward-written history (see dsp_float_filters.h).
 *
 * Numerics (the rules every function below follows):
 *  - Values live as float32 (or double) in memory and are combined in extended precision (`long double`, the x87 format with a 64-bit mantissa).  A value
 *    is rounded to float32 / double only where it is stored into a float / double member, an output buffer or a history element.
 *  - Every dot product is accumulated in `long double` with two accumulators (even and odd terms, see dot2()); a float or double accumulator would round
 *    every partial sum and change the result.  The product of two floats is exact in the 64-bit mantissa, so `acc += (long double)a * b` is exact per term.
 *  - Results are rounded to float32 by an explicit `(float)` cast at the store; values that are read back from the slot just stored (`y_` / `w_` in
 *    FloatARMA::process, `y_` in GenericIIR) are float / double members for that reason.
 *  - The input-sample stores into the history, the history shifts and the by-value argument of FloatFIR::process(float) / FloatARMA::process(float) are 32-bit
 *    word moves (copyWord(), storeArgWord()): a float load and store through the x87 would quiet a signalling NaN.
 *  - GenericIIR moves doubles through floating-point loads and stores, which quiet a signalling NaN double; its histories only ever hold values produced by
 *    arithmetic, so no signalling NaN can get there.
 *  - GenericIIR::process(const T*, T*, n) multiplies by an extended-precision reciprocal 1 / den[0]; the single-sample process(T) divides: the two roundings
 *    differ, and both are kept.
 *
 * Constructor / usage rules:
 *  - Tap counts are rounded DOWN to a multiple of 4 in FloatFIR / FloatIIR and UP in FloatARMA, so the four-term unrolled dot product normally has no
 *    remainder (the remainder loop is kept for completeness).
 *  - The history wrap is a do-while over (taps - 1) elements: with taps - 1 == 0 it runs 2^32 times.  FloatFIR / FloatIIR / FloatARMA objects with fewer
 *    than 2 taps must never wrap.  GenericIIR guards the wrap with n > 1.
 *  - Allocation results are not checked, except by the FloatFIR / FloatIIR constructors' zero fill and reset() (and all destructors).
 *  - FloatARMA / GenericIIR keep a loop counter in the member idx_.  Its final values are kept (FloatARMA constructor and reset(): ylen_; GenericIIR
 *    process(): max(nDen_, 1) once n > 0; GenericIIR constructor and reset(): lenDen_); the intermediate values are not observable.
 *  - The FloatARMA constructor reads ar[0] even when nAr == 0 and then writes ar_[0] = 0 (into a zero-size block when nAr4_ == 0).
 *  - The block process functions keep the history index in a local and store the member once at the end (and only when n > 0).
 *  - FloatFIR::process(float x) sums as `odd + even`; with NaN data the x87 picks the NaN with the larger significand, so the operand order is irrelevant.
 */

#include "dsp_float_filters.h"
#include "core/dsp_asm.h"

typedef long double LD;

/* ------------------------------------------------------------------------------------------------------ helpers */

/**
 * @brief Dot product sum_{k<n} c[k] * h[k] in extended precision.
 *
 * Four-term unrolled: accumulator `ev` takes the terms 0 and 2 of every group of four and the remainder, `od` the terms 1 and 3; the result is `od + ev`.
 * Both start at +0.
 */
static inline LD dot2(const float* h, const float* c, unsigned n)
{
  LD ev = 0, od = 0;

  while (n > 3)
  {
    ev += (LD)h[0] * c[0];
    od += (LD)h[1] * c[1];
    ev += (LD)h[2] * c[2];
    od += (LD)h[3] * c[3];
    h += 4;
    c += 4;
    n -= 4;
  }
  while (n != 0)
  {
    ev += (LD)*h * *c;
    ++h;
    ++c;
    --n;
  }
  return od + ev;
}

#include "core/dsp_copyword_plain.h"
#include "core/dsp_floatbits.h"
static inline void zeroWords(float* p, unsigned n) { __builtin_memset(p, 0, (size_t)n * sizeof(float)); }

/**
 * @brief History wrap: hist[len-1-j] = hist[n-2-j] for j = 0 .. n-2 (the newest n-1 elements move to the end of the block).
 * @warning A do-while over (n - 1) elements: n == 1 runs about 2^32 times, n == 0 about 2^32 as well.
 */
static inline void wrapCopy(float* hist, unsigned len, unsigned n)
{
  float* dst = hist + len - 1;
  const float* src = hist + n - 2;
  unsigned cnt = n - 1;

  do
  {
    copyWord(dst, src);
    --dst;
    --src;
  } while (--cnt != 0);
}

/* ------------------------------------------------------------------------------------------------------ FloatFIR */

/**
 * @brief Build the filter: taps_ = ntaps & ~3, history of taps_ + extra floats, cleared when allocated.
 * @warning The allocation is not checked beyond the clear; process() with a NULL `hist_` writes through NULL.
 */
FloatFIR::FloatFIR(unsigned ntaps, float* coef, unsigned extra)
{
  taps_ = ntaps & ~3u;
  coef_ = coef;
  histLen_ = taps_ + extra;
  hist_ = (float*)sysdep_malloc(histLen_ * 4u);
  if (hist_)
    zeroWords(hist_, histLen_);
  room_ = (int)(histLen_ - taps_);
}

FloatFIR::~FloatFIR()
{
  if (hist_)
    sysdep_free(hist_);
}

void FloatFIR::reset()
{
  if (hist_)
    zeroWords(hist_, histLen_);
  room_ = (int)(histLen_ - taps_);
}

/**
 * @brief Install another coefficient array / tap count.
 * @param coef   new coefficients (borrowed)
 * @param ntaps  new tap count, rounded down to a multiple of 4
 * @return 0, or -1 (nothing changed) when the rounded count is not smaller than the history length
 *
 * A changed tap count clamps `room_` to the new limit; the history is not cleared.
 */
int FloatFIR::setCoefficients(float* coef, unsigned ntaps)
{
  unsigned t = ntaps & ~3u;

  if (histLen_ <= t)
    return -1;
  coef_ = coef;
  if (taps_ != t)
  {
    taps_ = t;
    int limit = (int)(histLen_ - t);

    room_ = (room_ > limit) ? limit : room_;
  }
  return 0;
}

/** @brief Filter one sample: store it at hist[room], return the dot product with the coefficients, advance (wrap when needed). */
float FloatFIR::process(float x)
{
  float* hist = hist_;
  unsigned taps = taps_;
  int room = room_;

  storeArgWord(hist + room, x);
  float y = (float)dot2(hist + room, coef_, taps);

  if (room - 1 >= 0)
  {
    room_ = room - 1;
    return y;
  }
  room_ = (int)(histLen_ - taps);
  wrapCopy(hist, histLen_, taps);
  return y;
}

/** @brief Filter `n` samples from `in` to `out`; no work when n == 0. */
void FloatFIR::process(const float* in, float* out, unsigned n)
{
  if (n == 0)
    return;

  float* hist = hist_;
  const float* coef = coef_;
  unsigned taps = taps_;
  int room = room_;

  do
  {
    copyWord(hist + room, in);
    ++in;
    float y = (float)dot2(hist + room, coef, taps);

    *out = y;
    ++out;
    room = room - 1;
    if (room < 0)
    {
      room = (int)(histLen_ - taps);
      wrapCopy(hist, histLen_, taps);
    }
  } while (--n != 0);
  room_ = room;
}

/* ------------------------------------------------------------------------------------------------------ FloatIIR */

/** @brief Build the filter: same construction as FloatFIR (taps_ = ntaps & ~3, history of taps_ + extra floats, cleared when allocated). */
FloatIIR::FloatIIR(unsigned ntaps, float* coef, unsigned extra)
{
  taps_ = ntaps & ~3u;
  coef_ = coef;
  histLen_ = taps_ + extra;
  hist_ = (float*)sysdep_malloc(histLen_ * 4u);
  if (hist_)
    zeroWords(hist_, histLen_);
  room_ = (int)(histLen_ - taps_);
}

FloatIIR::~FloatIIR()
{
  if (hist_)
    sysdep_free(hist_);
}

void FloatIIR::reset()
{
  if (hist_)
    zeroWords(hist_, histLen_);
  room_ = (int)(histLen_ - taps_);
}

/** @brief As FloatFIR::setCoefficients(). */
int FloatIIR::setCoefficients(float* coef, unsigned ntaps)
{
  unsigned t = ntaps & ~3u;

  if (histLen_ <= t)
    return -1;
  coef_ = coef;
  if (taps_ != t)
  {
    taps_ = t;
    int limit = (int)(histLen_ - t);

    room_ = (room_ > limit) ? limit : room_;
  }
  return 0;
}

/**
 * @brief Filter `n` samples: y = in + dot(coef, past outputs); y is stored to `out` and as the newest history element.
 *
 * The history advance (and wrap) happens before the new output is stored, so the output lands at hist[room] with the previous outputs behind it.
 */
void FloatIIR::process(const float* in, float* out, unsigned n)
{
  if (n == 0)
    return;

  float* hist = hist_;
  const float* coef = coef_;
  unsigned taps = taps_;
  int room = room_;

  do
  {
    LD sum = dot2(hist + room, coef, taps);
    float y = (float)(sum + (LD)*in);

    ++in;
    *out = y;
    ++out;
    room = room - 1;
    if (room < 0)
    {
      room = (int)(histLen_ - taps);
      wrapCopy(hist, histLen_, taps);
    }
    copyWord(hist + room, &y);
  } while (--n != 0);
  room_ = room;
}

/* ----------------------------------------------------------------------------------------------------- FloatARMA */

/**
 * @brief Build the filter: copy and zero-pad the coefficients (lengths rounded up to a multiple of 4), normalise by ar[0], clear the histories.
 * @param nAr    number of AR coefficients (ar[0] is the leading one)
 * @param nMa    number of MA coefficients
 * @param ar     AR coefficients (copied)
 * @param ma     MA coefficients (copied)
 * @param extra  extra history length beyond the coefficient counts
 * @warning The allocations are not checked.  `ar[0]` == 0 divides by zero and fills the copies with infinities / NaNs (the normalisation is skipped only
 *          when it is 1.0 or NaN); `ar[0]` is read even when nAr == 0, and ar_[0] is written even for a zero-length block.
 */
FloatARMA::FloatARMA(unsigned nAr, unsigned nMa, float* ar, float* ma, unsigned extra)
{
  unsigned nb4 = nAr & ~3u;

  if (nb4 < nAr)
    nb4 += 4;
  nAr4_ = nb4;
  unsigned na4 = nMa & ~3u;

  if (na4 < nMa)
    na4 += 4;
  nMa4_ = na4;
  xlen_ = na4 + extra;
  ylen_ = nb4 + extra;
  ar_ = 0;
  ma_ = 0;
  xhist_ = 0;
  yhist_ = 0;
  ar_ = (float*)sysdep_malloc(nAr4_ * 4u);
  ma_ = (float*)sysdep_malloc(nMa4_ * 4u);
  xhist_ = (float*)sysdep_malloc(xlen_ * 4u);
  yhist_ = (float*)sysdep_malloc(ylen_ * 4u);

  /* copy (word moves) and zero-pad the two coefficient arrays */
  for (unsigned i = 0; i < nAr; ++i)
    copyWord(ar_ + i, ar + i);
  if (nb4 > nAr)
    zeroWords(ar_ + nAr, nb4 - nAr);
  for (unsigned i = 0; i < nMa; ++i)
    copyWord(ma_ + i, ma + i);
  if (na4 > nMa)
    zeroWords(ma_ + nMa, na4 - nMa);

  /* normalise by ar[0] (skipped when it equals 1.0 or is a NaN), then ar_[0] = 0 */
  float a0 = ar[0];

  if (a0 < 1.0f || a0 > 1.0f)
  {
    LD recip = (LD)1.0 / (LD)a0; /* the extended reciprocal is NOT rounded to float */

    for (unsigned i = 0; i < nMa; ++i)
      ma_[i] = (float)(recip * (LD)ma_[i]);
    for (unsigned i = 1; i < nAr; ++i)
      ar_[i] = (float)(recip * (LD)ar_[i]);
  }
  ar_[0] = 0;

  /* histories and indices */
  if (xlen_ != 0)
    zeroWords(xhist_, xlen_);
  if (ylen_ != 0)
    zeroWords(yhist_, ylen_);
  idx_ = ylen_;
  xr_ = (int)(xlen_ - nMa4_);
  yr_ = (int)(ylen_ - nAr4_);
}

FloatARMA::~FloatARMA()
{
  if (ar_)
    sysdep_free(ar_);
  if (ma_)
    sysdep_free(ma_);
  if (xhist_)
    sysdep_free(xhist_);
  if (yhist_)
    sysdep_free(yhist_);
}

/** @brief Clear both histories and restart the write indices. */
void FloatARMA::reset()
{
  if (xlen_ != 0)
    zeroWords(xhist_, xlen_);
  idx_ = 0;
  if (ylen_ != 0)
  {
    zeroWords(yhist_, ylen_);
    idx_ = ylen_;
  }
  xr_ = (int)(xlen_ - nMa4_);
  yr_ = (int)(ylen_ - nAr4_);
}

/**
 * @brief Filter one sample: out = (MA dot product) - (AR dot product), each rounded to float32 first, then the difference rounded.
 *
 * The input is stored at xhist[xr], the output at yhist[yr] after both dot products were taken (ar_[0] == 0, so the slot is not part of the sum).
 * Both write indices then advance, wrapping their histories when they run out.
 * @warning With nAr4_ or nMa4_ == 0 the wrap runs about 2^32 times (see the file header).
 */
float FloatARMA::process(float x)
{
  float* xh = xhist_;
  float* yh = yhist_;
  int xr = xr_;
  int yr = yr_;
  unsigned nAr = nAr4_;
  unsigned nMa = nMa4_;

  storeArgWord(xh + xr, x);
  y_ = (float)dot2(xh + xr, ma_, nMa);
  w_ = (float)dot2(yh + yr, ar_, nAr);
  float out = (float)((LD)y_ - (LD)w_);

  y_ = out;
  copyWord(yh + yr, &out);

  if (yr - 1 >= 0)
  {
    yr_ = yr - 1;
  }
  else
  {
    yr_ = (int)(ylen_ - nAr);
    wrapCopy(yh, ylen_, nAr);
  }
  if (xr - 1 >= 0)
  {
    xr_ = xr - 1;
  }
  else
  {
    xr_ = (int)(xlen_ - nMa);
    wrapCopy(xh, xlen_, nMa);
  }
  return y_;
}

/** @brief Block form of process(float); no work when n == 0.  Not called anywhere in this tree. */
void FloatARMA::process(const float* in, float* out, unsigned n)
{
  if (n == 0)
    return;

  float* xh = xhist_;
  float* yh = yhist_;
  const float* ma = ma_;
  const float* ar = ar_;
  int xr = xr_;
  int yr = yr_;
  unsigned nAr = nAr4_;
  unsigned nMa = nMa4_;

  do
  {
    copyWord(xh + xr, in);
    ++in;
    y_ = (float)dot2(xh + xr, ma, nMa);
    w_ = (float)dot2(yh + yr, ar, nAr);
    float o = (float)((LD)y_ - (LD)w_);

    y_ = o;
    copyWord(yh + yr, &o);
    yr = yr - 1;
    if (yr < 0)
    {
      yr = (int)(ylen_ - nAr);
      wrapCopy(yh, ylen_, nAr);
    }
    copyWord(out, &y_);
    ++out;
    xr = xr - 1;
    if (xr < 0)
    {
      xr = (int)(xlen_ - nMa);
      wrapCopy(xh, xlen_, nMa);
    }
  } while (--n != 0);
  xr_ = xr;
  yr_ = yr;
}

/* ------------------------------------------------------------------------------------------- GenericIIR<T, C> */

/** @brief History wrap of the C-element histories: hist[len-1-j] = hist[n-2-j], j = 0 .. n-2; nothing for n <= 1. */
template <class C> static inline void wrapCopyC(C* hist, unsigned len, unsigned n)
{
  if (n > 1)
  {
    C* dst = hist + len - 1;
    const C* src = hist + n - 2;

    for (unsigned j = n - 1; j != 0; --j)
    {
      *dst = *src;
      --dst;
      --src;
    }
  }
}

/**
 * @brief Build the filter.
 * @param nDen   number of denominator coefficients (>= 1; den[0] is the leading one)
 * @param nNum   number of numerator coefficients (>= 1)
 * @param den    denominator coefficients (borrowed)
 * @param num    numerator coefficients (borrowed)
 * @param extra  extra history length
 * @warning The history allocations are not checked and reset() clears them unconditionally.  With nDen == 0 or nNum == 0 the first history store is one
 *          element past the block (the write index starts at `extra` in a block of `extra` elements).
 */
template <class T, class C> GenericIIR<T, C>::GenericIIR(unsigned nDen, unsigned nNum, C* den, C* num, unsigned extra)
{
  nDen_ = nDen;
  lenDen_ = nDen + extra;
  nNum_ = nNum;
  den_ = den;
  lenNum_ = nNum + extra;
  num_ = num;
  hnum_ = 0;
  hden_ = 0;
  hnum_ = (C*)sysdep_malloc(lenNum_ * (unsigned)sizeof(C));
  hden_ = (C*)sysdep_malloc(lenDen_ * (unsigned)sizeof(C));
  reset();
}

template <class T, class C> GenericIIR<T, C>::~GenericIIR()
{
  if (hnum_)
    sysdep_free(hnum_);
  if (hden_)
    sysdep_free(hden_);
}

/** @brief Clear both histories and restart the write indices. */
template <class T, class C> void GenericIIR<T, C>::reset()
{
  if (lenNum_ != 0)
    __builtin_memset(hnum_, 0, (size_t)lenNum_ * sizeof(C));
  idx_ = 0;
  if (lenDen_ != 0)
  {
    __builtin_memset(hden_, 0, (size_t)lenDen_ * sizeof(C));
    idx_ = lenDen_;
  }
  rNum_ = (int)(lenNum_ - nNum_);
  rDen_ = (int)(lenDen_ - nDen_);
}

/**
 * @brief Filter one sample.
 *
 * Stores the input, accumulates the numerator and (from index 1) the denominator terms in extended precision, divides by den[0] when it is non-zero,
 * stores the output (as C) in the denominator history and advances both write indices (wrapping when needed).
 */
template <class T, class C> T GenericIIR<T, C>::process(T x)
{
  unsigned nNum = nNum_;
  unsigned nDen = nDen_;
  int rN = rNum_;
  int rD = rDen_;
  C* hn = hnum_;

  /* numerator and denominator sums; y_ follows the running sum (the last value is the one kept) */
  hn[rN] = (C)x;
  y_ = 0;
  LD acc = 0;
  if (nNum != 0)
  {
    unsigned i = 0;

    do
    {
      acc += (LD)hn[rN + i] * (LD)num_[i];
      y_ = (C)acc;
      ++i;
    } while (i < nNum);
  }
  if (nDen > 1)
  {
    unsigned i = 1;

    do
    {
      acc -= (LD)hden_[rD + i] * (LD)den_[i];
      y_ = (C)acc;
      ++i;
    } while (i < nDen);
  }
  idx_ = (nDen > 1) ? nDen : 1;

  /* normalise by den[0] (when non-zero) and store the output */
  C den0 = den_[0];

  if (den0 < 0 || den0 > 0)
  {
    C q = (C)(acc / (LD)den0);

    y_ = q;
    hden_[rD] = q;
  }
  else
  {
    hden_[rD] = (C)acc;
  }

  /* advance the write indices */
  if (rD - 1 >= 0)
  {
    rDen_ = rD - 1;
  }
  else
  {
    rDen_ = (int)(lenDen_ - nDen);
    wrapCopyC(hden_, lenDen_, nDen);
  }
  if (rN - 1 >= 0)
  {
    rNum_ = rN - 1;
  }
  else
  {
    rNum_ = (int)(lenNum_ - nNum);
    wrapCopyC(hn, lenNum_, nNum);
  }
  return (T)y_;
}

/** @brief Filter `n` samples; as process(T) but multiplying by an extended-precision reciprocal of den[0]; no work when n == 0. */
template <class T, class C> void GenericIIR<T, C>::process(const T* in, T* out, unsigned n)
{
  if (n == 0)
    return;

  unsigned nNum = nNum_;
  unsigned nDen = nDen_;
  int rN = rNum_;
  int rD = rDen_;
  C* hn = hnum_;
  C* hd = hden_;

  do
  {
    hn[rN] = (C)*in;
    ++in;
    y_ = 0;
    LD acc = 0;

    if (nNum != 0)
    {
      unsigned i = 0;

      do
      {
        acc += (LD)hn[rN + i] * (LD)num_[i];
        y_ = (C)acc;
        ++i;
      } while (i < nNum);
    }
    if (nDen > 1)
    {
      unsigned i = 1;

      do
      {
        acc -= (LD)hd[rD + i] * (LD)den_[i];
        y_ = (C)acc;
        ++i;
      } while (i < nDen);
    }

    C den0 = den_[0];

    if (den0 < 0 || den0 > 0)
    {
      LD recip = (LD)1.0 / (LD)den0; /* extended reciprocal, multiplied in (the single-sample version divides) */
      C q = (C)(acc * recip);

      y_ = q;
      hd[rD] = q;
    }
    else
    {
      hd[rD] = (C)acc;
    }

    rD = rD - 1;
    if (rD < 0)
    {
      rD = (int)(lenDen_ - nDen);
      wrapCopyC(hd, lenDen_, nDen);
    }
    *out = (T)y_;
    ++out;
    rN = rN - 1;
    if (rN < 0)
    {
      rN = (int)(lenNum_ - nNum);
      wrapCopyC(hn, lenNum_, nNum);
    }
  } while (--n != 0);
  rNum_ = rN;
  idx_ = (nDen > 1) ? nDen : 1;
  rDen_ = rD;
}

/* Explicit instantiations: the five member functions of GenericIIR<float, double> that are used (weak symbols). */
template GenericIIR<float, double>::GenericIIR(unsigned, unsigned, double*, double*, unsigned);
template GenericIIR<float, double>::~GenericIIR();
template void GenericIIR<float, double>::reset();
template float GenericIIR<float, double>::process(float);
template void GenericIIR<float, double>::process(const float*, float*, unsigned);
