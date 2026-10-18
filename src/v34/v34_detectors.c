/**
 * @file v34_detectors.c
 * @brief V.34 tone and FSK detectors: the two-stage IIR tone detector, the DFT bin accumulators and the 600 baud FSK detector / demodulator.
 *
 * Entry points (all declared in v34_internal.h / v34_symbol_tables.h and used by the handshake pieces and the V.34 receive path):
 * - tone_detect(): cascaded two-section IIR band-pass on a block of samples, a leaky peak-energy follower and a small counting state machine that decides
 *   "tone present" / "tone gone".
 * - dftupdate() / dftenergy(): a bank of single-bin correlators (one record of 0x2c bytes per bin) and the energy read-out of those bins.
 * - fskdetect(): interpolates 4 input samples to 12, filters them through a 49-word weighting line and an 80-tap low-pass and produces 3 soft values.
 * - fskdemodulate(): turns the 3 soft values of fskdetect() into bits (zero-crossing timing recovery, shift register of recovered bits).
 *
 * None of the functions prints or calls the host.  The tables they read (costbl, intcoef1/2/3, fsklpfcoeff600) are in v34_symbol_tables.c.  The state
 * blocks are reached through the V34Obj / V34ToneDet members; the DFT bin records, the FSK demodulator state block and the filter windows of the
 * object (an undeclared region) are walked by byte offset.  The DFT bins are typed records (`DftBin`, v34_records.h).
 *
 * ## Floating point in dftupdate() and dftenergy()
 *
 * The bin sums are doubles that are accumulated with x87 extended-precision arithmetic: every intermediate is kept to a 64-bit mantissa and rounded to
 * double only when it is stored.  The code therefore spells these values as `long double` and never lets the compiler choose the precision, so that the
 * result does not depend on the optimisation level or on -mfpmath.  This requires the x87 control word to be in its power-up state (64-bit mantissa,
 * round to nearest even), which holds on Linux x86 and x86-64.
 * - dftupdate: sum = (double)((long double)p + (long double)sum).  The int32 p is exact in extended precision; the add is rounded once to 64 bits and
 *   again to 53 bits by the store.  While |sum| < 2^53 both roundings are exact; beyond that the double rounding differs from a plain double add.
 * - dftenergy: energy = (double)((long double)(sumI * sumI) + (long double)(sumQ * sumQ)).  The squares are rounded to 64 bits (not 53) and stay in
 *   extended precision until the single rounding of the sum.  A plain double implementation already differs once |sumI| or |sumQ| exceeds about 2^26.5.
 * On a target whose `long double` is not the x87 80-bit type (aarch64, MSVC, 32-bit ARM / PowerPC) a software 64-bit-mantissa emulation would be needed.
 *
 * ## Bin record (DftBin, v34_records.h; 0x2c bytes; the doubles are only 4-byte aligned for odd bins, which `#pragma pack(4)` of the struct expresses)
 *
 * | offset | member               | type | meaning                                              |
 * |--------|----------------------|------|------------------------------------------------------|
 * | 0x00   | phase                | u16  | phase (14 bits)                                      |
 * | 0x02   | step                 | u16  | phase step per sample                                |
 * | 0x04   | accI                 | s32  | integer I accumulator                                |
 * | 0x08   | accQ                 | s32  | integer Q accumulator                                |
 * | 0x0c   | energyHi             | s16  | high half of the integer energy (dftenergy)          |
 * | 0x0e   | normCnt              | s16  | normalisation count of the integer energy            |
 * | 0x10   | sumI                 | f64  | I sum                                                |
 * | 0x18   | sumQ                 | f64  | Q sum                                                |
 * | 0x20   | energy               | f64  | energy (dftenergy)                                   |
 * | 0x28   | quietLimit, toneLimit| s16  | two limits of the retrain-request detector; neither function touches them |
 */

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "v34_symbol_tables.h" /* costbl (== v8_costbl), intcoef1/2/3, fsklpfcoeff600 */
#include "v34_internal.h"

#include "v34_access.h"
#include "v34_object_view.h"

/* ---- tone detector ---- */

/**
 * @brief Run the two-stage tone detector over a block of samples and report whether the tone is present.
 * @param rxobj   the receive sub-object (V34Rx) of the V.34 object; only its flag word `rxFlags_` is written: bit 0x200 (AGC freeze) is cleared when the
 *                detector first arms
 * @param detobj  the detector record (V34ToneDet) built by the detector initialiser; it is read and updated.  Layout (byte offsets in the record):
 *                +0x00 coefficient pointer (short[8]), +0x04 mode, +0x06 armed flag, +0x08 counter, +0x0a required count, +0x0c state, +0x0e low threshold,
 *                +0x10 high threshold, +0x12 peak energy E, +0x14/+0x16 stage-1 input history, +0x18/+0x1a stage-2 input history, +0x1c/+0x1e stage-1
 *                output history, +0x20/+0x22 stage-2 output history (all signed 16-bit)
 * @param p       first sample (16-bit PCM)
 * @param end     one past the last sample; the loop runs only while `p < end` (unsigned pointer compare)
 * @return 1 when the counter exceeds the required count, else 0
 *
 * Per sample two cascaded second-order sections run with the coefficient array `c`: section 1 uses c[0..1] (feed-forward on x >> 4) and c[4..5]
 * (feedback); section 2 uses c[2..3] and c[6..7] on the section-1 output >> 4.  Each product is truncated to 16 bits after the >> 14; the running sums
 * are int accumulations of which only the low 16 bits are stored.  The magnitude of the section-2 output (>> 4) feeds a leaky peak follower
 * E = |w| + 0.95 * E (0x3ccd / 2^14).
 *
 * The decision block runs after the loop even when the block was empty.
 * - state 1: the counter is incremented as a settling delay; the function returns 0 while it is negative, then the state becomes 2.
 * - mode 0 (tone-on search): while not armed, E > 0x100 arms the detector (the AGC freeze flag is cleared, the counter is zeroed) and returns 0.  Once
 *   armed, the counter counts the blocks with E above the high threshold and is cleared by a block below it.
 * - mode != 0 (tone-off search): the counter counts the blocks with E below the low threshold and is cleared by a block above the high threshold.
 * @note  The names "tone-on" / "tone-off" for the two modes describe what the code does; no caller-side name for them exists.
 */
int tone_detect(V34Rx* rxobj, V34ToneDet* detobj, const short* p, const short* end)
{

  if ((uintptr_t)p < (uintptr_t)end)
  {
    const short* c = (const short*)detobj->p3564_;
    do
    {
      int x = *p++;
      int y1 = x >> 4;
      int z, w, a, e;
      unsigned int old1c, old14, old18, old20;
      int j;

      /* section 1 */
      for (j = 0; j < 2; j++)
      {
        y1 += (int16_t)((detobj->toneInHist_[j] * (int)c[j]) >> 14);
        y1 -= (int16_t)((detobj->toneOutHist_[j] * (int)c[4 + j]) >> 14);
      }

      /* shift the section-1 histories */
      old1c = AS_U16(detobj->toneOutHist_[0]);
      old14 = AS_U16(detobj->toneInHist_[0]);
      AS_U16(detobj->toneOutHist_[0]) = (uint16_t)y1;
      z = (int16_t)(y1 >> 4); /* y1 is the untruncated int */
      AS_U16(detobj->toneInHist_[0]) = (uint16_t)(x >> 4);
      AS_U16(detobj->toneOutHist_[1]) = (uint16_t)old1c;
      AS_U16(detobj->toneInHist_[1]) = (uint16_t)old14;

      /* section 2 */
      w = z;
      for (j = 0; j < 2; j++)
      {
        w += (int16_t)((detobj->toneInHist_[j + 2] * (int)c[2 + j]) >> 14);
        w -= (int16_t)((detobj->toneOutHist_[j + 2] * (int)c[6 + j]) >> 14);
      }

      /* shift the section-2 histories */
      old18 = AS_U16(detobj->toneInHist_[2]);
      old20 = AS_U16(detobj->toneOutHist_[2]);
      AS_U16(detobj->toneInHist_[2]) = (uint16_t)z;
      AS_U16(detobj->toneInHist_[3]) = (uint16_t)old18;
      AS_U16(detobj->toneOutHist_[3]) = (uint16_t)old20;
      AS_U16(detobj->toneOutHist_[2]) = (uint16_t)w;

      /* leaky peak follower of the output magnitude */
      a = (int16_t)w >> 4; /* abs via (v ^ (v>>31)) - (v>>31) on the sign-extended int */
      a = (a ^ (a >> 31)) - (a >> 31);
      e = a + ((detobj->toneEnergy_ * 0x3ccd) >> 14);
      AS_U16(detobj->toneEnergy_) = (uint16_t)e;
    } while ((uintptr_t)p < (uintptr_t)end);
  }

  /* state 1: settling delay */
  if (detobj->toneState_ == 1)
  {
    uint16_t n = (uint16_t)(AS_U16(detobj->toneCnt_) + 1); /* 16-bit increment, 16-bit store */
    AS_U16(detobj->toneCnt_) = n;
    if ((int16_t)n < 0)
      return 0; /* state stays 1 */
    detobj->toneState_ = 2;
  }

  /* mode 0: tone-on search; otherwise tone-off search */
  if (detobj->toneMode_ == 0)
  {
    if (detobj->toneDetF6_ == 0)
    { /* not yet armed */
      if (detobj->toneEnergy_ <= 0x100)
        return 0;
      detobj->toneDetF6_ = 1;
      rxobj->rxFlags_ = (uint16_t)(rxobj->rxFlags_ & 0xfdff);
      detobj->toneCnt_ = 0;
      return 0;
    }
    if (detobj->toneEnergy_ > detobj->toneHigh_)
      AS_U16(detobj->toneCnt_) = (uint16_t)(AS_U16(detobj->toneCnt_) + 1);
    else
      detobj->toneCnt_ = 0;
  }
  else
  {
    int16_t e = detobj->toneEnergy_;
    if (e < detobj->toneLow_)
      AS_U16(detobj->toneCnt_) = (uint16_t)(AS_U16(detobj->toneCnt_) + 1);
    if (e > detobj->toneHigh_)
      detobj->toneCnt_ = 0;
  }

  return detobj->toneCnt_ > detobj->toneReq_;
}

/* ---- DFT bins ---- */

/**
 * @brief Correlate a block of samples against every bin of a DFT bank.
 * @param bins      array of bin records (DftBin, 0x2c bytes each); updated in place
 * @param nbins     number of bins; only the low 16 bits are used (sign-extended)
 * @param samples   input samples (16-bit PCM)
 * @param nsamples  number of samples; only the low 16 bits are used (sign-extended)
 *
 * For each sample (outer loop) and each bin (inner loop): advance the 14-bit phase by the bin's step, take cos / sin from `costbl` (phase >> 6, and +90
 * degrees), accumulate sample * cos and sample * sin >> 6 into the two wrapping int32 accumulators and the full products into the two doubles (extended-
 * precision add, see the file header).
 */
void dftupdate(DftBin* bins, int nbins, const short* samples, int nsamples)
{
  int nb = (int16_t)nbins;
  int ns = (int16_t)nsamples;
  int n, i;
  for (n = 0; n < ns; n++)
  {
    DftBin* bin = bins;
    for (i = 0; i < nb; i++, bin++)
    {
      unsigned int ph = ((unsigned int)bin->phase + bin->step) & 0x3fff;
      unsigned int idx = ph >> 6;
      int x = samples[n];
      int pp, qq;
      long double t;

      bin->phase = (uint16_t)ph;
      pp = (int)costbl[idx] * x;
      bin->accI = (int32_t)((uint32_t)bin->accI + (uint32_t)(pp >> 6));

      qq = (int)costbl[(idx + 0x40) & 0xff] * x;
      bin->accQ = (int32_t)((uint32_t)bin->accQ + (uint32_t)(qq >> 6));

      t = (long double)pp + (long double)bin->sumI; /* rounded to 64 bits ... */
      bin->sumI = (double)t;                        /* ... then to 53 bits */
      t = (long double)qq + (long double)bin->sumQ;
      bin->sumQ = (double)t;
    }
  }
}

/**
 * @brief Compute the energy of every bin of a DFT bank.
 * @param bins   array of bin records (DftBin, see dftupdate()); the fields `energyHi`, `normCnt` and `energy` are written
 * @param nbins  number of bins; only the low 16 bits are used (sign-extended)
 * @param shift  left shift applied to the integer accumulators first; only the low 5 bits are used
 *
 * Per bin: the integer energy e = a*a + b*b of the top 16 bits of the shifted accumulators (wrapping int32); the high half of e goes to +0x0c; the
 * floating energy sumI^2 + sumQ^2 (extended precision, see the file header) goes to +0x20; the normalisation count at +0x0e is the smallest m + 1
 * (m = 0..0x12) for which bits 31-m and 32-m of e differ (bit 32 being the sign extension), or 0x14 when none does.
 */
void dftenergy(DftBin* bins, int nbins, int shift)
{
  int nb = (int16_t)nbins;
  unsigned int sh = (unsigned int)shift & 31u;
  DftBin* bin = bins;
  int k;

  for (k = 0; k < nb; k++, bin++)
  {
    int32_t a = (int32_t)((uint32_t)bin->accI << sh) >> 16;
    int32_t b2 = (int32_t)((uint32_t)bin->accQ << sh) >> 16;
    uint32_t eu = (uint32_t)a * (uint32_t)a + (uint32_t)b2 * (uint32_t)b2; /* wraps like imul/lea */
    int32_t e = (int32_t)eu;
    long double si = (long double)bin->sumI;
    long double sq = (long double)bin->sumQ;
    long double sqi = si * si; /* each square rounded to 64 bits, kept in extended */
    long double sqq = sq * sq;
    int m;
    int16_t r = 0x14;

    bin->energyHi = (int16_t)(eu >> 16);
    bin->energy = (double)(sqq + sqi); /* one rounding of the sum, then to double */

    /* normalisation count of the integer energy */
    for (m = 0; m <= 0x12; m++)
    {
      if ((((e >> (0x13 - m)) ^ (e >> (0x14 - m))) & 0x1000) != 0)
      {
        r = (int16_t)(m + 1);
        break;
      }
    }
    bin->normCnt = r;
  }
}

/* ---- FSK detector / demodulator ---- */

/**
 * @brief Filter 4 input samples into 3 soft FSK values.
 * @param obj  the V.34 object; its input history (`waae6_` .. +0xaafe, 13 words, newest sample at the LOW end, 12-tap window from `waae6_` to +0xaafc),
 *             the 80-word FIR window `fskWin_` at +0xab00..+0xab9f and the scratch pointer `p80c4_` are used and updated
 * @param in   4 input samples
 * @param out  3 results
 * @param st   `st[0]` selects a tap of the weighting line (0..48), `st[1]` is a bias subtracted from every result
 *
 * The memory behind `obj->p80c4_` (it points into the near echo-adapt record) is used as private scratch: bytes +0x14..+0x75 are a 49-word delay line.
 *
 * Steps:
 * 1. 4 input samples -> 12 interpolated values loc[]: per sample three 12-tap dot products (intcoef1/2/3, seeded with 0x2000, >> 14) against the
 *    history window; the third loop also shifts the history up by one slot (each word is read before it can be overwritten).
 * 2. For each of the 12 values: shift the 49-word delay line up one word, store the value at the head, then loc[m] = (tap * loc[m]) >> 14 where the tap
 *    is read after the store.
 * 3. For i in 0..2: append loc[4i..4i+3] to the end of the 80-word window, out[i] = (dot(window, fsklpfcoeff600) >> 14) - st[1] (no rounding constant),
 *    then slide the window down by 4 words (76 word moves).
 * All 32-bit accumulations wrap; they are done in unsigned arithmetic.
 * @warning `st[0]` is not range-checked; a value outside 0..48 reads outside the delay line.  Callers take it from the FSK state block.
 */
void fskdetect(V34Obj* obj, const uint16_t* in, short* out, const short* st)
{
  short loc[12];
  int n, k, m, i, q;
  int16_t* hist = (int16_t*)&obj->waae6_; /* the 13-word input history: waae6_ followed by fskHist_[12] (newest sample at the low end) */
  unsigned char* scratch;
  const unsigned char* g;

  /* step 1: interpolate */
  for (n = 0; n < 4; n++)
  {
    uint32_t s1 = 0x2000, s2 = 0x2000, s3 = 0x2000;
    obj->waae6_ = *in++;
    for (k = 0; k < 12; k++)
      s1 += (uint32_t)(hist[11 - k] * (int)intcoef1[k]);
    for (k = 0; k < 12; k++)
      s2 += (uint32_t)(hist[11 - k] * (int)intcoef2[k]);
    for (k = 0; k < 12; k++)
    {
      int16_t h = hist[11 - k];
      hist[12 - k] = h;
      s3 += (uint32_t)(h * (int)intcoef3[k]);
    }
    loc[3 * n + 0] = (short)((int32_t)s1 >> 14);
    loc[3 * n + 1] = (short)((int32_t)s2 >> 14);
    loc[3 * n + 2] = (short)((int32_t)s3 >> 14);
  }

  /* step 2: weight through the delay line */
  scratch = (unsigned char*)obj->echo0.p80c4_;
  g = scratch + 0x14 + 2 * (int)st[0]; /* unchecked; computed once, read after the store */
  for (m = 0; m < 12; m++)
  {
    memmove(scratch + 0x16, scratch + 0x14, 0x60);
    U16(scratch, 0x14) = (uint16_t)loc[m];
    loc[m] = (short)(((int)*(const s16_t*)g * (int)loc[m]) >> 14);
  }

  /* step 3: low-pass filter, 4 new values per result */
  q = 0;
  for (i = 0; i < 3; i++)
  {
    uint32_t s = 0;
    for (k = 0; k < 4; k++)
      obj->fskWin_[76 + k] = loc[q++];
    for (k = 0; k < 80; k++)
      s += (uint32_t)(obj->fskWin_[k] * (int)fsklpfcoeff600[k]);
    out[i] = (short)(((int32_t)s >> 14) - st[1]);
    for (k = 0; k < 76; k++)
      obj->fskWin_[k] = obj->fskWin_[k + 4];
  }
}

/**
 * @brief Demodulate 4 input samples of FSK into recovered bits.
 * @param obj  the V.34 object; nothing is done (and nothing is written) when its word `w402_` is non-zero
 * @param in   4 input samples (passed to fskdetect())
 * @param st   demodulator state (byte offsets): +0 s16 tap index and +2 s16 bias (fskdetect()'s st[0] / st[1]), +4 u16 pattern OR-ed into the shift
 *             register when the previous soft value is <= 0, +6 u16 pattern when it is > 0, +8 s16 bit period P, +0x0a s16 reload value, +0x0c s16
 *             counter, +0x0e s16 next sampling point, +0x10 s16 bit count, +0x12 u16 shift register, +0x14 s16 previous soft value
 *
 * Runs fskdetect() and then steps through its 3 soft values: when the counter reaches the sampling point the shift register takes one bit (shifted
 * left, OR-ed with the pattern chosen by the previous value's sign), the bit count is incremented and the sampling point advances by P.  The counter
 * is cleared when it reaches P * 256 and on every sign change (zero crossing in either direction), which also reloads the sampling point.  All sign
 * tests are 16-bit; 0 counts as non-negative for the crossing test and as "<= 0" for the pattern choice.
 */
void fskdemodulate(V34Obj* obj, const uint16_t* in, void* st)
{
  short loc[14];
  int16_t prev;
  int16_t cur = 0;
  int thr;
  int i;

  if (obj->rx.w402_ != 0)
    return;

  fskdetect(obj, in, loc, (const short*)st);
  prev = S16(st, 0x14);
  thr = (int)S16(st, 8) << 8;
  for (i = 0; i < 3; i++)
  {
    uint32_t cnt = U16(st, 0xc);
    uint32_t nxt = U16(st, 0xe);
    uint32_t c1;

    /* sampling point reached: shift in a bit */
    if ((int16_t)cnt >= (int16_t)nxt)
    {
      uint16_t pat = (prev > 0) ? U16(st, 6) : U16(st, 4);
      U16(st, 0x12) = (uint16_t)((uint32_t)U16(st, 0x12) * 2u | pat);
      U16(st, 0x10) = (uint16_t)(U16(st, 0x10) + 1);
      U16(st, 0xe) = (uint16_t)(nxt + (uint16_t)S16(st, 8));
    }

    /* bit-period overflow: restart the counter */
    c1 = cnt + 1u;
    if ((int)(int16_t)c1 >= thr)
    {
      c1 = 0;
      U16(st, 0xe) = U16(st, 0xa);
    }

    /* zero crossing: resynchronise */
    cur = loc[i];
    if ((prev < 0) != (cur < 0))
    { /* zero crossing (either direction) */
      S16(st, 0xc) = 0;
      U16(st, 0xe) = U16(st, 0xa);
    }
    else
    {
      U16(st, 0xc) = (uint16_t)c1;
    }
    prev = cur;
  }

  S16(st, 0x14) = cur;
}
