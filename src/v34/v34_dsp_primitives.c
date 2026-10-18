/**
 * @file v34_dsp_primitives.c
 * @brief V.34 leaf DSP primitives: echo canceller, linear equalizer, timing-recovery filters, Hilbert filter, scrambler / descrambler, non-linear encoder.
 *
 * Every function here is a small stand-alone kernel that the larger V.34 files call; none of them calls back into the modem.
 * - Echo canceller (EchoAdaptState): a circular far-end delay line, an N-tap history, and N extended-precision (hi:lo 16-bit halves of one 32-bit
 *   value) coefficients.  V34EchoCleanUp(), V34EchoUpdateDelayLine(), V34EchoFilter(), V34EchoAdapt(), V34EchoEstimateDelayLineEnergy(),
 *   V34EchoReportCoeff() (debug dump); V34EchoPreFilter() / V34EchoPreFilterCopy() work on a PreFilterState (a 42-tap FIR).
 * - Receive equalizer (EqualizerState): a complex 80-tap FIR whose coefficients are hi:lo pairs; V34EqualizerCleanUp(),
 *   V34EqualizerClearCenterTaps(), V34EqualizerUpdateDelayLine(), V34EqualizerFilter(), V34EqualizerAdapt(), V34EqualizerCenterAdapt().
 * - Symbol-timing recovery (TimingState): V34TimingFiltersInit(), V34TimingFilter() (returns the timing-error sample), V34TimingHPFilter(),
 *   V34TimingPrefilter(); the coefficient tables are in v34_timing_hilbert_tables.c.
 * - Hilbert filter: V34InitHilbertFilter(), V34HilbertFilter() (64-tap in-phase / quadrature pair).
 * - Miscellaneous: V34Filter2() (generic FIR; no caller), V34scrambler() / V34descrambler() (the self-synchronising scrambler of the V.34
 *   data stream), V34nlencoder() (non-linear encoding of one (I, Q) point), V34PremptxCopy() (stores the transmit-emphasis coefficient pointer).
 *
 * Four of the state types (EchoAdaptState, PreFilterState, EqualizerState, TimingState) are declared here as real structs with pointer members:
 * the functions receive one of them directly as their first argument.  Each struct overlays a region of the V.34 module object (V34Obj of
 * v34_object_view.h); the _Static_asserts after the struct definitions pin the members that follow a pointer to the corresponding member of
 * the module object, so the overlay holds both with 4-byte and with 8-byte pointers.  V34descrambler() and V34PremptxCopy() take the receive
 * and the modulator sub-object (V34Rx, V34Mod) of the module object instead and reach their members with offsetof().
 *
 * The 16- and 32-bit arithmetic wraps exactly as the fixed-point DSP code expects; the casts through unsigned types are deliberate.
 */

#define V34_PRIMITIVES_TYPED /* this file defines the typed versions of the primitives v34_internal.h declares opaque */
#include <stddef.h>
#include <stdint.h>

#include "v34_timing_hilbert_tables.h"
#include "v34_object_view.h" /* V34Rx / V34Mod (embedded sub-objects of the V.34 module object) */
#include "core/dsp_host.h"
#include "v34_internal.h"

/* ===================== Struct 1: EchoAdaptState (the echo canceller) ===================== */

/** State of one echo canceller (near-end or far-end): circular delay line, tap history and extended-precision coefficients. */
typedef struct EchoAdaptState
{
  short* writePtr;  ///< next write position in the circular delay line [base, base + delayLineLen)
  short* base;      ///< start of the circular delay line
  short* coefHi;    ///< high halves of the hi:lo extended-precision coefficients (N entries); the only half the filter reads
  short* coefLo;    ///< low halves (N entries); read and written by the adaptation only
  short* history;   ///< the N-entry tap history, oldest first
  int reserved14;   ///< not read or written by any function in this file
  int delayLineLen; ///< length of the circular delay line in samples
  int N;            ///< number of taps
} EchoAdaptState;

/* ===================== Struct 2: PreFilterState (V34EchoPreFilter / V34EchoPreFilterCopy) ===================== */

/** State of the 42-tap pre-filter applied to the echo canceller input. */
typedef struct PreFilterState
{
  short delay[42];           ///< inline FIR shift register
  short* coef;               ///< coefficient array (42 entries), set by V34EchoPreFilterCopy()
  unsigned short idx;        ///< not touched by V34EchoPreFilter() / V34EchoPreFilterCopy() (used by V34EchoHistoryBackwardClean())
  unsigned short reserved5a; ///< padding-shaped gap before `cap`
  unsigned int cap;          ///< not touched by the functions in this file
  unsigned int len2;         ///< not touched by the functions in this file
  unsigned char shift;       ///< low byte of a stored 32-bit word: the final right-shift of V34EchoPreFilter(); the only member of this tail it reads
} PreFilterState;

/* ===================== Struct 3: EqualizerState (the complex 80-tap equalizer) ===================== */

/** State of the receive equalizer: delay lines and hi:lo coefficients of the in-phase and quadrature paths. */
typedef struct EqualizerState
{
  short iDelay[80];             ///< in-phase delay line (circular, `idx` is the next write position and the oldest sample)
  short qDelay[80];             ///< quadrature delay line
  short iCoefHi[80];            ///< in-phase coefficients, high halves (Q12; the centre tap 40 is 0x1000 = unity after V34EqualizerCleanUp())
  short qCoefHi[80];            ///< quadrature coefficients, high halves
  short iCoefLo[80];            ///< in-phase coefficients, low halves (adaptation only)
  short qCoefLo[80];            ///< quadrature coefficients, low halves (adaptation only)
  int idx;                      ///< delay-line position, range [0, 80)
  unsigned char reserved3c4[8]; ///< zeroed by V34EqualizerCleanUp() (it clears the whole struct), otherwise unused here
} EqualizerState;

/* ===================== Struct 4: TimingState (the symbol-timing recovery filters) ===================== */

/** State of the symbol-timing recovery: two complex recursive sections, a high-pass FIR and the pre-filter delay chains. */
typedef struct TimingState
{
  short bcoefA[3];   ///< feed-forward history of the low channel (3-tap shift register)
  short bcoefB[3];   ///< feed-forward history of the high channel
  short aPosReal[3]; ///< recursive history of the positive-half-baud section, real part; [0] is this call's output, [1] and [2] the older ones
  short aPosImag[3]; ///< same, imaginary part
  short aNegReal[3]; ///< recursive history of the negative-half-baud section, real part
  short aNegImag[3]; ///< same, imaginary part
  short hpDelay[40]; ///< delay line of the 40-tap high-pass FIR
  struct
  {
    int a;                       ///< delay chain fed from curSample
    int b;                       ///< delay chain fed from prevSample
  } prefilterDelay[20];          ///< delay chains of V34TimingPrefilter(); each entry holds two packed 16-bit channels
  const short* prefilterCoefPtr; ///< set by V34TimingFiltersInit(), not read by any function in this file
  const short* hpCoefPtr;        ///< set by V34TimingFiltersInit(), not read by any function in this file
  int curSample;                 ///< the latest packed two-channel sample (high half: first channel, low half: second channel)
  int prevSample;                ///< curSample as it was one V34TimingFilter() call earlier
} TimingState;

/* Each of these structs overlays a region of the V.34 module object (V34Obj, v34_object_view.h).  The offsets of the members that follow a pointer
   differ between 32-bit and 64-bit builds, but the overlay is consistent in both: */
_Static_assert(offsetof(EchoAdaptState, writePtr) == offsetof(V34Echo, p80b8_) && offsetof(EchoAdaptState, base) == offsetof(V34Echo, p80bc_) &&
                   offsetof(EchoAdaptState, coefHi) == offsetof(V34Echo, echoBuf_) && offsetof(EchoAdaptState, coefLo) == offsetof(V34Echo, p80c4_) &&
                   offsetof(EchoAdaptState, history) == offsetof(V34Echo, p80c8_) && offsetof(EchoAdaptState, reserved14) == offsetof(V34Echo, w80cc_) &&
                   offsetof(EchoAdaptState, delayLineLen) == offsetof(V34Echo, w80d0_) && offsetof(EchoAdaptState, N) == offsetof(V34Echo, echoCnt_),
               "EchoAdaptState overlays V34Echo");
_Static_assert(offsetof(PreFilterState, coef) == offsetof(V34PreFilter, p20cc_) && offsetof(PreFilterState, idx) == offsetof(V34PreFilter, pfIdx_) &&
                   offsetof(PreFilterState, cap) == offsetof(V34PreFilter, pfCap_) && offsetof(PreFilterState, len2) == offsetof(V34PreFilter, pfLen2_) &&
                   offsetof(PreFilterState, shift) == offsetof(V34PreFilter, pfShift_),
               "PreFilterState overlays V34PreFilter");
_Static_assert(offsetof(TimingState, prefilterCoefPtr) == offsetof(V34Obj, rx.p0620_) - offsetof(V34Obj, rx.unk050c_) &&
                   offsetof(TimingState, hpCoefPtr) == offsetof(V34Obj, rx.p0624_) - offsetof(V34Obj, rx.unk050c_) &&
                   offsetof(TimingState, curSample) == offsetof(V34Obj, rx.unk0628_) - offsetof(V34Obj, rx.unk050c_) &&
                   offsetof(TimingState, prevSample) == offsetof(V34Obj, rx.unk0628_) - offsetof(V34Obj, rx.unk050c_) + 4,
               "TimingState overlays V34Obj at 0x50c");

/* ===================== Echo canceller ===================== */

/**
 * @brief Reset an echo canceller: rewind the write pointer and zero the coefficients and the tap history.
 * @param obj  echo canceller state; coefHi, coefLo and history must each have N entries
 * @note   When the debug level is above 1 it also prints a banner line through dsplibs_debug_printf().
 */
void V34EchoCleanUp(EchoAdaptState* obj)
{
  unsigned int n = (unsigned int)obj->N;
  unsigned int i;

  obj->writePtr = obj->base;
  for (i = 0; i < n; i++)
  {
    obj->coefHi[i] = 0;
    obj->coefLo[i] = 0;
    obj->history[i] = 0;
  }
  if (dsplibs_debug_level > 1)
  {
    dsplibs_debug_printf("V34FLO: Echo running in Original Integer...\r\n");
  }
}

/**
 * @brief Append one sample to the circular delay line.
 * @param obj     echo canceller state
 * @param sample  the new sample; it is stored at writePtr, which then advances and wraps to `base` at base + delayLineLen
 */
void V34EchoUpdateDelayLine(EchoAdaptState* obj, short sample)
{
  short* p = obj->writePtr;

  *p = sample;
  p = p + 1;
  if (obj->base + obj->delayLineLen <= p)
  {
    p = obj->base;
  }
  obj->writePtr = p;
}

/**
 * @brief Run the echo canceller filter for one sample and return the echo estimate.
 * @param obj  echo canceller state
 * @param val  delay, in samples, of the newest tap behind writePtr: the new history entry is read from writePtr + (val + N - 1)
 * @return the dot product of coefHi[] and history[] (32-bit, only the high halves of the coefficients take part)
 * @note   The history is shifted down by one entry first, so history[N - 1] is the entry just read from the delay line.
 * @warning The read position is wrapped once (by delayLineLen) only; a `val + N - 1` that is negative or reaches twice the delay-line length
 *          reads outside [base, base + delayLineLen).  N must be at least 1 (history[N - 1] is written).
 */
int V34EchoFilter(EchoAdaptState* obj, short val)
{
  int n = obj->N;
  short* hist = obj->history;
  short* tap;
  int sum = 0;
  int i;

  /* shift the tap history and append the new delay-line sample */
  if (n != 1)
  {
    for (i = 0; i < n - 1; i++)
    {
      hist[i] = hist[i + 1];
    }
  }

  tap = obj->writePtr + (val + n - 1);
  if (obj->base + obj->delayLineLen <= tap)
  {
    tap -= obj->delayLineLen;
  }
  hist[n - 1] = *tap;

  /* echo estimate */
  for (i = 0; i < n; i++)
  {
    sum += (int)obj->coefHi[i] * (int)hist[i];
  }
  return sum;
}

/**
 * @brief LMS update of the echo canceller coefficients with extended precision.
 * @param obj  echo canceller state
 * @param err  error sample (the residual after echo subtraction)
 * @note   Each coefficient is the 32-bit value (coefHi[i] << 16 | coefLo[i]); the update adds history[i] * err and stores the two halves back.
 */
void V34EchoAdapt(EchoAdaptState* obj, short err)
{
  short* coefHi = obj->coefHi;
  short* coefLo = obj->coefLo;
  short* hist = obj->history;
  int n = obj->N;
  int i;

  for (i = 0; i < n; i++)
  {
    int coef32 = (int)(((unsigned int)(unsigned short)coefHi[i] << 16) | (unsigned int)(unsigned short)coefLo[i]) + (int)hist[i] * (int)err;
    coefHi[i] = (short)((unsigned int)coef32 >> 16);
    coefLo[i] = (short)coef32;
  }
}

/**
 * @brief Energy of the tap history.
 * @param obj  echo canceller state
 * @return the sum over the N history entries of (history[i]^2 >> 5)
 */
int V34EchoEstimateDelayLineEnergy(EchoAdaptState* obj)
{
  short* hist = obj->history;
  int n = obj->N;
  int sum = 0;
  int i;

  for (i = 0; i < n; i++)
  {
    sum += ((int)hist[i] * (int)hist[i]) >> 5;
  }
  return sum;
}

/**
 * @brief Filter a block of samples in place with the 42-tap pre-filter.
 * @param x    samples, replaced by the filtered samples
 * @param n    number of samples
 * @param obj  pre-filter state: the shift register `delay` persists between calls; `coef` points at 42 coefficients; `shift` is the output shift
 * @note   Each output is (dot product + 0x4000) >> (shift & 31), truncated to 16 bits.
 */
void V34EchoPreFilter(short* x, short n, PreFilterState* obj)
{
  const short* coef = obj->coef;
  unsigned char shift = obj->shift;
  int i;

  for (i = 0; i < n; i++)
  {
    short carry = x[i];
    int dot = 0;
    int k;
    for (k = 0; k < 42; k++)
    {
      short old = obj->delay[k];
      obj->delay[k] = carry;
      dot += (int)carry * (int)coef[k];
      carry = old;
    }
    x[i] = (short)((dot + 0x4000) >> (shift & 0x1f));
  }
}

/**
 * @brief Select the coefficient array of the pre-filter.
 * @param obj      pre-filter state
 * @param coefPtr  42 coefficients; the pointer is stored, not the data
 */
void V34EchoPreFilterCopy(PreFilterState* obj, const short* coefPtr) { obj->coef = (short*)coefPtr; }

/**
 * @brief Debug dump of the echo canceller coefficients.
 * @param obj  echo canceller state (read only)
 * @note   Looks for a non-zero entry among the first (N / 6) * 6 entries of coefHi.  If one is found, and the debug level is above 1, it prints a
 *         header and 24 rows of six coefHi values; if none is found it prints a single "Nothing to report" line.  There is no other effect.
 * @warning The row loop always walks 144 (0x90) entries of coefHi, whatever N is; with N < 144 it reads past the end of the coefficient array.
 *          Only the debug output is affected, and only when the debug level is above 1.
 */
void V34EchoReportCoeff(EchoAdaptState* obj)
{
  const short* coefHi = obj->coefHi;
  unsigned int bound = ((unsigned int)obj->N / 6) * 6;
  unsigned int i;

  for (i = 0; i < bound; i++)
  {
    if (coefHi[i] != 0)
    {
      unsigned int j;
      if (dsplibs_debug_level > 1)
      {
        dsplibs_debug_printf("?======= Coefficients[1..%ld]=========\n", (long)bound);
      }
      for (j = 0; j < 0x90; j += 6)
      {
        if (dsplibs_debug_level > 1)
        {
          dsplibs_debug_printf("?%d %d %d %d %d %d\n", (int)coefHi[j], (int)coefHi[j + 1], (int)coefHi[j + 2], (int)coefHi[j + 3], (int)coefHi[j + 4],
                               (int)coefHi[j + 5]);
        }
      }
      return;
    }
  }
  if (dsplibs_debug_level > 1)
  {
    dsplibs_debug_printf("?======= Nothing to report =========\n");
  }
}

/* ===================== Equalizer ===================== */

/**
 * @brief Reset the equalizer: clear the whole state and set the centre tap to unity.
 * @param obj  equalizer state
 * @note   Sets iCoefHi[40] = 0x1000 (1.0 in Q12); every other coefficient, both delay lines and idx become 0.
 */
void V34EqualizerCleanUp(EqualizerState* obj)
{
  sysdep_memset(obj, 0, sizeof(*obj));
  obj->iCoefHi[40] = 0x1000;
}

/**
 * @brief Zero the eight centre taps (36..43) of the equalizer.
 * @param obj  equalizer state
 * @note   Only iCoefHi and qCoefHi are cleared; the matching iCoefLo / qCoefLo entries keep their value, so the next adaptation starts from the
 *         low halves left behind.
 */
void V34EqualizerClearCenterTaps(EqualizerState* obj)
{
  int i;
  for (i = 36; i < 44; i++)
  {
    obj->iCoefHi[i] = 0;
    obj->qCoefHi[i] = 0;
  }
}

/**
 * @brief Store one complex input sample in the equalizer delay line and advance the position.
 * @param obj       equalizer state
 * @param i_sample  in-phase sample
 * @param q_sample  quadrature sample
 */
void V34EqualizerUpdateDelayLine(EqualizerState* obj, short i_sample, short q_sample)
{
  int idx = obj->idx;

  obj->iDelay[idx] = i_sample;
  obj->qDelay[idx] = q_sample;
  idx = idx + 1;
  if (idx == 80)
  {
    idx = 0;
  }
  obj->idx = idx;
}

/**
 * @brief Complex 80-tap FIR over the equalizer delay line.
 * @param obj      equalizer state (read only); only the high halves of the coefficients are used
 * @param outReal  receives the real part of the sum (32-bit, no rescaling)
 * @param outImag  receives the imaginary part
 * @note   Tap t pairs with delay-line entry (idx + t) mod 80, so tap 0 sees the oldest sample.  The circular walk is two straight loops
 *         (entries idx..79, then 0..idx-1) rather than a modulo.
 */
void V34EqualizerFilter(EqualizerState* obj, int* outReal, int* outImag)
{
  int idx = obj->idx;
  int sumReal = 0, sumImag = 0;
  int t = 0, k;

  for (k = idx; k < 80; k++, t++)
  {
    int iv = obj->iDelay[k];
    int qv = obj->qDelay[k];
    short ch = obj->iCoefHi[t];
    short cq = obj->qCoefHi[t];
    sumReal += iv * ch - qv * cq;
    sumImag += iv * cq + qv * ch;
  }
  for (k = 0; k < idx; k++, t++)
  {
    int iv = obj->iDelay[k];
    int qv = obj->qDelay[k];
    short ch = obj->iCoefHi[t];
    short cq = obj->qCoefHi[t];
    sumReal += iv * ch - qv * cq;
    sumImag += iv * cq + qv * ch;
  }
  *outReal = sumReal;
  *outImag = sumImag;
}

/**
 * Extended-precision complex LMS update of one equalizer tap: coef -= conj(x) * err, on the 32-bit values (hi << 16 | lo) of both paths.
 * Used by V34EqualizerAdapt() for every tap.
 */
static void EqualizerAdaptTap(EqualizerState* obj, int t, int iv, int qv, int errReal, int errImag)
{
  int iCoef32 = (int)(((unsigned int)(unsigned short)obj->iCoefHi[t] << 16) | (unsigned int)(unsigned short)obj->iCoefLo[t]) - iv * errReal - qv * errImag;
  int qCoef32 = (int)(((unsigned int)(unsigned short)obj->qCoefHi[t] << 16) | (unsigned int)(unsigned short)obj->qCoefLo[t]) - iv * errImag + qv * errReal;
  obj->iCoefHi[t] = (short)((unsigned int)iCoef32 >> 16);
  obj->iCoefLo[t] = (short)iCoef32;
  obj->qCoefHi[t] = (short)((unsigned int)qCoef32 >> 16);
  obj->qCoefLo[t] = (short)qCoef32;
}

/**
 * @brief Extended-precision complex LMS update of all 80 equalizer taps.
 * @param obj      equalizer state
 * @param errReal  real part of the equalizer error
 * @param errImag  imaginary part of the equalizer error
 * @note   Same circular traversal as V34EqualizerFilter(): tap t pairs with delay-line entry (idx + t) mod 80.
 */
void V34EqualizerAdapt(EqualizerState* obj, short errReal, short errImag)
{
  int idx = obj->idx;
  int t = 0, k;

  for (k = idx; k < 80; k++, t++)
  {
    EqualizerAdaptTap(obj, t, obj->iDelay[k], obj->qDelay[k], errReal, errImag);
  }
  for (k = 0; k < idx; k++, t++)
  {
    EqualizerAdaptTap(obj, t, obj->iDelay[k], obj->qDelay[k], errReal, errImag);
  }
}

/**
 * Update of one centre tap by the same formula as EqualizerAdaptTap(), but on the high half only: the new value is the high half of
 * (hi * 0x10000 - delta + 0x8000), i.e. rounded, and no low half is kept.
 */
static void EqualizerCenterAdaptTap(short* iCoefHi, short* qCoefHi, int iv, int qv, int errReal, int errImag)
{
  int iCoef32 = (int)*iCoefHi * 0x10000 - iv * errReal - qv * errImag;
  int qCoef32 = (int)*qCoefHi * 0x10000 - iv * errImag + qv * errReal;
  *iCoefHi = (short)((unsigned int)(iCoef32 + 0x8000) >> 16);
  *qCoefHi = (short)((unsigned int)(qCoef32 + 0x8000) >> 16);
}

/**
 * @brief Adapt only the eight centre taps (36..43) of the equalizer.
 * @param obj      equalizer state
 * @param errReal  real part of the equalizer error
 * @param errImag  imaginary part of the equalizer error
 * @note   The taps pair with delay-line entries (idx + 36 .. idx + 43) mod 80.  The three branches below handle the window lying wholly before the
 *         end of the delay line, straddling it, or wholly wrapped; each covers exactly eight (entry, tap) pairs.  Unlike V34EqualizerAdapt() the
 *         update keeps no low half: the high half is rounded and the matching iCoefLo / qCoefLo entries are left as they are.
 */
void V34EqualizerCenterAdapt(EqualizerState* obj, short errReal, short errImag)
{
  int idx = obj->idx;
  int a = idx + 36;
  int b = idx + 44;
  int firstLo, firstHi, secondHi;
  int t, k;

  /* split the window [idx + 36, idx + 44) of delay-line entries into at most two straight runs */
  if (a < 80)
  {
    if (b > 79)
    {
      /* partial wrap: window starts before the array end but extends past it */
      firstLo = a;
      firstHi = 80;
      secondHi = idx - 36;
    }
    else
    {
      /* no wrap at all */
      firstLo = a;
      firstHi = b;
      secondHi = 0;
    }
  }
  else
  {
    /* full wrap: the whole 8-tap window has already wrapped past the end */
    firstLo = idx - 44;
    firstHi = idx - 36;
    secondHi = 0;
  }

  t = 36;
  for (k = firstLo; k < firstHi; k++, t++)
  {
    EqualizerCenterAdaptTap(&obj->iCoefHi[t], &obj->qCoefHi[t], obj->iDelay[k], obj->qDelay[k], errReal, errImag);
  }
  for (k = 0; k < secondHi; k++, t++)
  {
    EqualizerCenterAdaptTap(&obj->iCoefHi[t], &obj->qCoefHi[t], obj->iDelay[k], obj->qDelay[k], errReal, errImag);
  }
}

/* ===================== Timing filters ===================== */

/**
 * @brief 40-tap high-pass FIR on a single-sample shift register.
 * @param obj     timing state (hpDelay is the shift register)
 * @param sample  new input sample
 * @return (sum of coefficient products + 0x8000) >> 16, against V34TimingHPFilterCoeff[40]
 */
int V34TimingHPFilter(TimingState* obj, short sample)
{
  int sum = 0x8000;
  short cur = sample;
  int i;

  for (i = 0; i < 40; i++)
  {
    short coef = V34TimingHPFilterCoeff[i];
    short old = obj->hpDelay[i];
    obj->hpDelay[i] = cur;
    sum += (int)cur * (int)coef;
    cur = old;
  }
  return sum >> 16;
}

/**
 * @brief Timing-recovery filter: one packed two-channel sample in, one timing-error sample out.
 * @param obj           timing state
 * @param packedSample  two 16-bit channels in one word (high half and low half)
 * @return the high-pass filtered timing-error sample (a signed 16-bit value)
 * @note   The stages, per call:
 *   1. Scale both channels by 0x599a / 2^15 (about 0.7) and push them through a 3-tap complex shift register (bcoefA / bcoefB) against the
 *      posHalfBaud_Bcoef_* and negHalfBaud_Bcoef_* tables at the same time, giving two complex sums (posB, negB).
 *   2. Run the aPosReal / aPosImag history through a 2-tap recursive section (taps 1 and 2; tap 0 seeds the shift and is not summed) against
 *      posHalfBaud_Acoef_*, giving posA.
 *   3. The same for aNegReal / aNegImag against negHalfBaud_Acoef_*, giving negA.
 *   4. New tap 0 of each section = (B sum - A sum + 0x2000) >> 14, stored (truncated to 16 bits) in aPosReal[0] / aPosImag[0] / aNegReal[0] /
 *      aNegImag[0]; the next call consumes it as the seed of steps 2 and 3.
 *   5. The timing error is the rounded (>> 14) cross product of the two new complex tap-0 values, formed from the untruncated 32-bit values of
 *      step 4, then filtered by the same 40-tap high-pass FIR as V34TimingHPFilter() (accumulator seed 0x4000, final shift 15 here).
 *   The table names suggest the two sections select the upper and lower half-baud components of the received signal.
 */
int V34TimingFilter(TimingState* obj, int packedSample)
{
  int sumPosBReal = 0, sumPosBImag = 0, sumNegBReal = 0, sumNegBImag = 0;
  int sumPosAReal = 0, sumPosAImag = 0, sumNegAReal = 0, sumNegAImag = 0;
  short chanHi, chanLo;
  int newPosReal, newPosImag, newNegReal, newNegImag;
  short timingErr;
  int hpSum;
  int t;

  obj->prevSample = obj->curSample;
  obj->curSample = packedSample;

  /* step 1: scale the two channels, feed-forward sections */
  chanHi = (short)((((packedSample >> 16) * 0x599a) + 0x8000) >> 15);
  chanLo = (short)((((short)packedSample * 0x599a) + 0x8000) >> 15);
  for (t = 0; t < 3; t++)
  {
    int hiV = (int)chanHi;
    int loV = (int)chanLo;
    short oldA = obj->bcoefA[t];
    short oldB = obj->bcoefB[t];

    obj->bcoefA[t] = chanLo;
    obj->bcoefB[t] = chanHi;

    sumPosBReal += loV * (int)posHalfBaud_Bcoef_Real[t] - hiV * (int)posHalfBaud_Bcoef_Imag[t];
    sumPosBImag += (int)posHalfBaud_Bcoef_Imag[t] * loV + (int)posHalfBaud_Bcoef_Real[t] * hiV;
    sumNegBReal += (int)negHalfBaud_Bcoef_Real[t] * loV - hiV * (int)negHalfBaud_Bcoef_Imag[t];
    sumNegBImag += loV * (int)negHalfBaud_Bcoef_Imag[t] + hiV * (int)negHalfBaud_Bcoef_Real[t];

    chanHi = oldB;
    chanLo = oldA;
  }

  /* step 2: recursive section of the positive half-baud component */
  {
    short curReal = obj->aPosReal[0];
    short curImag = obj->aPosImag[0];
    for (t = 1; t < 3; t++)
    {
      short oldReal = obj->aPosReal[t];
      short oldImag = obj->aPosImag[t];

      obj->aPosReal[t] = curReal;
      obj->aPosImag[t] = curImag;

      sumPosAReal += (int)curReal * (int)posHalfBaud_Acoef_Real[t] - (int)curImag * (int)posHalfBaud_Acoef_Imag[t];
      sumPosAImag += (int)posHalfBaud_Acoef_Imag[t] * (int)curReal + (int)posHalfBaud_Acoef_Real[t] * (int)curImag;

      curReal = oldReal;
      curImag = oldImag;
    }
  }

  /* step 3: recursive section of the negative half-baud component */
  {
    short curReal = obj->aNegReal[0];
    short curImag = obj->aNegImag[0];
    for (t = 1; t < 3; t++)
    {
      short oldReal = obj->aNegReal[t];
      short oldImag = obj->aNegImag[t];

      obj->aNegReal[t] = curReal;
      obj->aNegImag[t] = curImag;

      sumNegAReal += (int)curReal * (int)negHalfBaud_Acoef_Real[t] - (int)curImag * (int)negHalfBaud_Acoef_Imag[t];
      sumNegAImag += (int)negHalfBaud_Acoef_Imag[t] * (int)curReal + (int)negHalfBaud_Acoef_Real[t] * (int)curImag;

      curReal = oldReal;
      curImag = oldImag;
    }
  }

  /* step 4: combine; the new tap 0 of each section is the seed for the next call */
  newPosReal = (sumPosBReal - sumPosAReal + 0x2000) >> 14;
  obj->aPosReal[0] = (short)newPosReal;
  newPosImag = (sumPosBImag - sumPosAImag + 0x2000) >> 14;
  obj->aPosImag[0] = (short)newPosImag;
  newNegReal = (sumNegBReal - sumNegAReal + 0x2000) >> 14;
  newNegImag = (sumNegBImag - sumNegAImag + 0x2000) >> 14;
  obj->aNegReal[0] = (short)newNegReal;
  obj->aNegImag[0] = (short)newNegImag;

  /* step 5: timing error from the cross product, then the high-pass FIR */
  timingErr = (short)(((newPosImag * newNegReal - newPosReal * newNegImag) + 0x2000) >> 14);
  hpSum = 0x4000;
  {
    short cur = timingErr;
    for (t = 0; t < 40; t++)
    {
      short coef = V34TimingHPFilterCoeff[t];
      short old = obj->hpDelay[t];
      obj->hpDelay[t] = cur;
      hpSum += (int)cur * (int)coef;
      cur = old;
    }
  }
  return (int)(short)(hpSum >> 15);
}

/**
 * @brief Initialise the timing state: clear the section histories and part of the delay lines, and set the two coefficient pointers.
 * @param obj  timing state
 * @bug    Only the first 0xc4 bytes are cleared: the six 3-entry histories, hpDelay[] and prefilterDelay[0..9].  prefilterDelay[10..19], curSample
 *         and prevSample keep whatever they held, so a re-initialisation leaves stale pre-filter taps and samples behind (the objects are normally
 *         zero when first created).
 * @note   prefilterCoefPtr and hpCoefPtr are set to the coefficient tables but no function in this file reads them.  The clear is a flat walk over
 *         the leading 16-bit words of the struct.
 */
void V34TimingFiltersInit(TimingState* obj)
{
  short* p = (short*)obj;
  int i;

  for (i = 0; i < 3; i++)
  {
    p[i] = 0;      /* bcoefA[i] */
    p[i + 3] = 0;  /* bcoefB[i] */
    p[i + 6] = 0;  /* aPosReal[i] */
    p[i + 9] = 0;  /* aPosImag[i] */
    p[i + 12] = 0; /* aNegReal[i] */
    p[i + 15] = 0; /* aNegImag[i] */
  }
  for (i = 0; i < 80; i++)
  {
    p[i + 0x12] = 0; /* hpDelay[] and the first ten prefilterDelay entries: see the @bug note */
  }

  obj->prefilterCoefPtr = V34TimingPrefilterCoeff;
  obj->hpCoefPtr = V34TimingHPFilterCoeff;
}

/**
 * @brief Pre-filter of the timing path: a 20-stage two-channel FIR over the packed samples.
 * @param obj  timing state: curSample / prevSample are the new inputs, prefilterDelay[] the delay chains
 * @return the two filtered channels packed in one word (high half: first channel, low half: second channel), each (sum + 0x2000) >> 14
 * @note   Stage j reads the pair c0 = V34TimingPrefilterCoeff[2j], c1 = V34TimingPrefilterCoeff[2j + 1]; the `a` chain (fed from curSample) is
 *         weighted by c0, the `b` chain (fed from prevSample) by c1, and both channels of a chain share the coefficient.  The table has 40 entries.
 */
unsigned int V34TimingPrefilter(TimingState* obj)
{
  int hiSum = 0x2000;
  int loSum = 0x2000;
  int curSample = obj->curSample;
  int prevSample = obj->prevSample;
  int j;

  for (j = 0; j < 20; j++)
  {
    int oldA = obj->prefilterDelay[j].a;
    int oldB = obj->prefilterDelay[j].b;
    short c0 = V34TimingPrefilterCoeff[2 * j];
    short c1 = V34TimingPrefilterCoeff[2 * j + 1];

    obj->prefilterDelay[j].a = curSample;
    obj->prefilterDelay[j].b = prevSample;

    hiSum += (curSample >> 16) * (int)c0 + (prevSample >> 16) * (int)c1;
    loSum += (int)(short)curSample * (int)c0 + (int)(short)prevSample * (int)c1;

    curSample = oldA;
    prevSample = oldB;
  }

  return (unsigned int)(((hiSum >> 14) << 16) | ((unsigned int)(loSum >> 14) & 0xffffU));
}

/* ===================== Hilbert filter ===================== */

/**
 * @brief Clear a Hilbert filter buffer.
 * @param buf  the 64-entry (128-byte) buffer; it is the whole filter state
 */
void V34InitHilbertFilter(short buf[64]) { sysdep_memset(buf, 0, 0x80); }

/**
 * @brief Run the 64-tap Hilbert filter pair for one sample.
 * @param buf        the 64-entry shift register (state, see V34InitHilbertFilter())
 * @param newSample  the new input sample
 * @param outReal    receives the sum against V34hilbertrealcoef[64]
 * @param outImag    receives the sum against V34hilbertimagcoef[64]
 * @note   Tap 0 is multiplied with newSample; buf[0] is only a scratch slot (it always ends the call as 0), the retained history is buf[1..63].
 */
void V34HilbertFilter(short buf[64], short newSample, int* outReal, int* outImag)
{
  short carry = 0;
  int sumReal = 0, sumImag = 0;
  int i;

  buf[0] = newSample;
  for (i = 0; i < 64; i++)
  {
    short old = buf[i];
    buf[i] = carry;
    sumReal += (int)V34hilbertrealcoef[i] * (int)old;
    sumImag += (int)V34hilbertimagcoef[i] * (int)old;
    carry = old;
  }
  *outReal = sumReal;
  *outImag = sumImag;
}

/* ===================== Miscellaneous ===================== */

/**
 * @brief Generic FIR with a single-sample shift register; all state is passed in.
 * @param sample     new input sample
 * @param delayLine  n-entry shift register, updated in place
 * @param coefTable  n coefficients
 * @param n          number of taps
 * @return the dot product of the new delay line and the coefficients (32-bit, no rescaling)
 * @note   Nothing in the library calls this function.
 */
int V34Filter2(short sample, short* delayLine, const short* coefTable, unsigned int n)
{
  int sum = 0;
  unsigned int i;

  for (i = 0; i < n; i++)
  {
    short old = delayLine[i];
    delayLine[i] = sample;
    sum += (int)sample * (int)coefTable[i];
    sample = old;
  }
  return sum;
}

/**
 * @brief Scramble up to 16 bits of the V.34 data stream with the self-synchronising scrambler.
 * @param state      32-bit shift register of the scrambler, caller owned; the newest scrambled bit is inserted at bit 31 and shifted down
 * @param mode       0: feedback from the bits delayed by 5 and 23 (reg & 0x4000000, reg & 0x100); non-zero: delayed by 18 and 23
 *                   (reg & 0x2000, reg & 0x100)
 * @param inputBits  data bits, least significant bit first
 * @param numBits    how many bits to scramble (the result and the shift use the low 5 bits of this value)
 * @return the `numBits` scrambled bits (the newest `numBits` entries of the register), least significant bit first; with numBits <= 0 the register
 *         is left unchanged and the bits it holds are returned
 * @note   Each output bit is the input bit XOR the two tapped register bits.
 */
short V34scrambler(unsigned int* state, short mode, unsigned short inputBits, short numBits)
{
  unsigned char shiftAmt = (unsigned char)numBits;
  unsigned short outMask = (unsigned short)((1 << (shiftAmt & 0x1f)) - 1);

  if (0 < numBits)
  {
    unsigned int reg = *state;
    short i;

    if (mode == 0)
    {
      for (i = 0; i < numBits; i++)
      {
        int bitIn = (inputBits & 1) != 0;
        inputBits = (unsigned short)((short)inputBits >> 1);
        if ((reg & 0x4000000) != 0)
          bitIn = !bitIn;
        if ((reg & 0x100) != 0)
          bitIn = !bitIn;
        if (bitIn)
          reg |= 0x80000000u;
        reg = reg >> 1;
      }
    }
    else
    {
      for (i = 0; i < numBits; i++)
      {
        int bitIn = (inputBits & 1) != 0;
        inputBits = (unsigned short)((short)inputBits >> 1);
        if ((reg & 0x2000) != 0)
          bitIn = !bitIn;
        if ((reg & 0x100) != 0)
          bitIn = !bitIn;
        if (bitIn)
          reg |= 0x80000000u;
        reg = reg >> 1;
      }
    }
    *state = reg;
    return (short)((unsigned short)(reg >> ((0x1f - shiftAmt) & 0x1f)) & outMask);
  }
  return (short)((unsigned short)(*state >> ((0x1f - shiftAmt) & 0x1f)) & outMask);
}

/**
 * @brief Descramble up to 16 received bits with the self-synchronising descrambler of the receive sub-object.
 * @param rx         the receive sub-object (V34Rx) of the V.34 module object; its `w0408_` is the 32-bit shift register of received bits and bit 2
 *                   of `rxFlags_` selects the feedback taps
 * @param inputBits  received (scrambled) bits, least significant bit first
 * @param numBits    how many bits to process; with numBits <= 0 nothing is read or written and 0 is returned
 * @return the descrambled bits, least significant bit first: each is the input bit XOR the register bits delayed by 23 and by 5 (flag bit 2 clear)
 *         or by 18 (flag bit 2 set)
 * @note   The register is shifted left, with the received bit entering at bit 1 (bit 0 stays 0), so bit k holds the bit received k steps ago.
 */
short V34descrambler(void* rx, unsigned short inputBits, short numBits)
{
  short outBits = 0;

  if (0 < numBits)
  {
    unsigned int reg = *(unsigned int*)((uintptr_t)rx + offsetof(V34Rx, w0408_));
    unsigned char flag = *(unsigned char*)((uintptr_t)rx + offsetof(V34Rx, rxFlags_));
    unsigned short mask = 1;
    short i;

    if ((flag & 4) == 0)
    {
      for (i = 0; i < numBits; i++)
      {
        unsigned int bit = (inputBits & mask) != 0;
        unsigned int hi = (reg >> 0x17) & 1;
        unsigned int lo = (reg >> 5) & 1;
        reg = (reg + bit) * 2;
        if (lo != (hi ^ bit))
        {
          outBits = (short)((unsigned short)outBits | mask);
        }
        mask = (unsigned short)(mask * 2);
      }
    }
    else
    {
      for (i = 0; i < numBits; i++)
      {
        unsigned int bit = (inputBits & mask) != 0;
        unsigned int hi = (reg >> 0x17) & 1;
        unsigned int lo = (reg >> 0x12) & 1;
        reg = (reg + bit) * 2;
        if (lo != (hi ^ bit))
        {
          outBits = (short)((unsigned short)outBits | mask);
        }
        mask = (unsigned short)(mask * 2);
      }
    }
    *(unsigned int*)((uintptr_t)rx + offsetof(V34Rx, w0408_)) = reg;
  }
  return outBits;
}

/**
 * @brief Non-linear encoding of one (I, Q) point.
 * @param in   the point: in[0] = I, in[1] = Q
 * @param out  receives the encoded point, same layout; `in` and `out` may be the same array
 * @note   Computes a scale factor from the point's energy ((I^2 + Q^2) >> 12 rescaled by 0x155, then a squared term weighted by 0x4ccd, then 0x3b17)
 *         and multiplies both coordinates by it (>> 14).  Uses no tables or state.
 */
void V34nlencoder(const short in[2], short out[2])
{
  int i1 = (int)in[0];
  int q1 = (int)in[1];
  short energy, sq, term, scaleSum;
  int scale;

  energy = (short)((((i1 * i1 + 0x800 + q1 * q1) >> 12) * 0x155 + 0x800) >> 12);
  sq = (short)(((int)energy * (int)energy + 0x2000) >> 14);
  term = (short)(((unsigned int)((int)sq * 0x4ccd)) >> 16);
  scaleSum = (short)((int)energy + 0x4000 + (int)term);
  scale = ((int)scaleSum * 0x3b17) >> 14;

  out[0] = (short)((i1 * scale) >> 14);
  out[1] = (short)((q1 * scale) >> 14);
}

/**
 * @brief Store the coefficient pointer of the transmit modulator.
 * @param mod  the transmit modulator sub-object (V34Mod) of the V.34 module object
 * @param ptr  the coefficient table; the pointer is stored in the `p2100_` slot of the sub-object, the data are not copied
 */
void V34PremptxCopy(void* mod, const void* ptr) { *(const void**)((uintptr_t)mod + offsetof(V34Mod, p2100_)) = ptr; }
