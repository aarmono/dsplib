/**
 * @file v90_constellation_noise.cpp
 * @brief V90ConstellationDesigner: choosing the ucode lists of the six phases from the measured noise (setConstellationToNoise and its force-rate variant).
 *
 * Both functions are called by V90ConstellationDesigner::process() / constellationDesign() (v90_constellation_dmin.cpp).  From the measured noise energy they
 * set the minimum distance `dMin_` (and, according to `state_`, the pdSnr thresholds for rate up / down / retrain), then fill `mapParams_->listA_` /
 * `listB_` / `cnt_` with the ucode indices of each of the six phases and their G.711 codes:
 * - setConstellationToNoise() scans the ucodes from `params_->unk360` up to `maxU[ph]` and keeps those whose distance exceeds a running half-distance.
 * - setConstellationToNoise_forceRate() distributes a requested number of ucodes over the phases (a base count `s`, scaled by 1/4 and 1/2 for the two kinds
 *   of phases, raised until the product of the counts reaches 2^n) and then iterates, per phase, until the phase holds exactly its count.
 * Both end with printReport(), the debug report of the counts and ucode table.
 *
 * Arithmetic: the 387 FPU is modelled with `long double` for the intermediate products and sums (dMin products, the A / B pair, the two-step reciprocal of
 * forceRate, the 2^n and product accumulators), `float` where a value is stored and reloaded (threshUp_ / threshDown_ / threshRetrain_, the phase
 * distances `phd[]`, the step), 16-bit truncation through trunc16() (out of range = 0x8000) and 32-bit truncation through dsp_trunc_i32().  Comparisons are
 * written as the FPU's flag tests take them (`jmpB` / `jmpBe` are also true for NaN, `jmpA` / `jmpAe` are not).  16-bit compares are signed; the compare
 * against `params_->unk360` is an unsigned 32-bit compare.
 *
 * Behaviours to know about:
 * - The unrestricted branch of setConstellationToNoise() keeps two quiet NaNs as "dMinHighRates" / "dMinLowRates": the debug print shows the 16-bit
 *   truncation of NaN (-32768) twice; the restricted branch prints the two values after rounding them to float.
 * - The scan's half-distance is NOT halved again after an accepted ucode: `half = max(A, B) + dMin`.
 * - forceRate: the retrain threshold print truncates the UNROUNDED extended product noise * 10 (the stored threshold is the float); the reciprocal
 *   1 / (nu - 0.5) is an extended value multiplied into A (two roundings); (1 - step) / (step + 1) are extended sums multiplied into the float phd[]
 *   (one rounding at the store).
 * - forceRate writes the caller's `cur[i]` (the ucode of phase i) when it has to grow a list.
 */

#include "v90_constellation_designer.h"
#include "core/dsp_math.h"
#include "core/dsp_host.h"
#include "v34/v34_api.h"
#include "pcm_leaf_classes.h"

extern "C"
{
}

typedef long double LD;

#include "core/dsp_floatbits.h"

/* ---- helpers ---- */

#include "core/dsp_x87_cmp.h"

#include "core/dsp_x87_conv.h"

#include "core/dsp_intmath.h"

#include "vpcm_split.h"
#include "vpcm_split_value.h"

/** Print a float threshold with the format `fmt` (sign, integer part, two decimals) through edprintf(). */
static inline void edThresh(const char* fmt, const float* pv)
{
  Split s = splitValue((LD)loadF(pv), 100.0L);
  edprintf(fmt, s.sign, s.ip, s.fp);
}

/*
 * Float constants of the designer (literals with enough digits to round-trip the stored float; the hex word is the IEEE-754 bit pattern):
 * kC428 / kC42c: slope and offset of the unrestricted dMin; kC434 / kC43c: slopes of the restricted dMin A / B values; kC44c / kC450: the rate-up / rate-down
 * threshold factors; kC470: the rate-up threshold factor of force-rate; kC478 / kC47c: the initial step and the step decay of force-rate.
 */
static const float kC428 = 6.77620983f;   /* 0x40d8d6b6 */
static const float kC42c = 9.89999962f;   /* 0x411e6666 */
static const float kC434 = 3.33305001f;   /* 0x405550b1 */
static const float kC43c = 8.21358776f;   /* 0x41036adb */
static const float kC44c = 0.449999988f;  /* 0x3ee66666 */
static const float kC450 = 1.41250002f;   /* 0x3fb4cccd */
static const float kC470 = 0.100000001f;  /* 0x3dcccccd */
static const float kC478 = 0.0500000007f; /* 0x3d4ccccd */
static const float kC47c = 0.899999976f;  /* 0x3f666666 */

/** The double 0.0007500001875000469 (bit pattern 0x3f489375237ec81b): the scale from `rateForce` to a count of 1/1000-rate units. */
static inline double rateScale()
{
  uint64_t w = 0x3f489375237ec81bull;
  double d;
  __builtin_memcpy(&d, &w, sizeof d);
  return d;
}

/** The G.711 code of a distance: u-law = ~linear2ulaw(|v|), A-law = linear2alaw(|v|) ^ 0xd5 (A-law when `useAlaw_` is non-zero). */
static inline uint8_t codeOf(const V90ConstellationDesigner* t, int v)
{
  int a = absI32(v);
  if (t->useAlaw_ == 0)
    return (uint8_t)~(uint8_t)linear2ulaw(a);
  return (uint8_t)((uint8_t)linear2alaw(a) ^ 0xd5u);
}

/** Table index `base + u` computed modulo 2^32, so that `u` = UINT_MAX means "one before `base`". */
static inline int ixOf(unsigned base, unsigned u) { return (int)(base + u); }

/**
 * The debug report shared by both functions (debug level 2 and above): the six phase counts, then one line per ucode index with the six ucodes, the six G.711
 * codes and the six distances of table `a`.  The debug level is re-read after every print because a print may change it.
 */
static inline void printReport(V90ConstellationDesigner* t, const int16_t* a)
{
  unsigned maxcnt = (uint16_t)t->mapParams_->cnt_[0];

  /* the largest of the six counts bounds the table dump */
  for (unsigned k = 1; k < 6; k++)
  {
    unsigned e = t->mapParams_->cnt_[k];
    if (e > maxcnt)
      maxcnt = (uint16_t)e;
  }

  unsigned lv = dsplibs_debug_level;
  if (lv > 1)
  {
    dsplibs_debug_printf("\n-----------------------------------------------------------------------------------------------------------------\r\n");
    lv = dsplibs_debug_level;
    if (lv > 1)
    {
      dsplibs_debug_printf("V90 Constellation Designer report:\r\n");
      lv = dsplibs_debug_level;
      if (lv > 1)
      {
        unsigned c5 = t->mapParams_->cnt_[5];
        unsigned c4 = t->mapParams_->cnt_[4];
        unsigned c3 = t->mapParams_->cnt_[3];
        unsigned c2 = t->mapParams_->cnt_[2];
        unsigned c1 = t->mapParams_->cnt_[1];
        unsigned c0 = t->mapParams_->cnt_[0];
        dsplibs_debug_printf("constelation size phase[0..5]  :  %d  %d  %d  %d  %d  %d\r\n", c0, c1, c2, c3, c4, c5);
        lv = dsplibs_debug_level;
        if (lv > 1)
        {
          dsplibs_debug_printf("----------- constel phase 0,1,2,3,4,5\t codec constel phase 0,1,2,3,4,5\tlinearMapping of constel --------------\r\n");
          lv = dsplibs_debug_level;
        }
      }
    }
  }

  for (unsigned i = 0; i < maxcnt; i++)
  {
    if (lv > 1)
    {
      unsigned l1_0 = t->mapParams_->listA_[0][i];
      unsigned l1_1 = t->mapParams_->listA_[1][i];
      unsigned l1_2 = t->mapParams_->listA_[2][i];
      unsigned l1_3 = t->mapParams_->listA_[3][i];
      unsigned l1_4 = t->mapParams_->listA_[4][i];
      unsigned l1_5 = t->mapParams_->listA_[5][i];
      unsigned l2_0 = t->mapParams_->listB_[0][i];
      unsigned l2_1 = t->mapParams_->listB_[1][i];
      unsigned l2_2 = t->mapParams_->listB_[2][i];
      unsigned l2_3 = t->mapParams_->listB_[3][i];
      unsigned l2_4 = t->mapParams_->listB_[4][i];
      unsigned l2_5 = t->mapParams_->listB_[5][i];
      dsplibs_debug_printf("ucode[%d]  :  %d  %d  %d  %d  %d  %d  :  %d  %d  %d  %d  %d  %d  :  %d  %d  %d  %d  %d  %d\r\n", i, l1_0, l1_1, l1_2, l1_3, l1_4,
                           l1_5, l2_0, l2_1, l2_2, l2_3, l2_4, l2_5, (int)a[l1_0], (int)a[l1_1 + 0x80], (int)a[l1_2 + 0x100], (int)a[l1_3 + 0x180],
                           (int)a[l1_4 + 0x200], (int)a[l1_5 + 0x280]);
      lv = dsplibs_debug_level;
    }
  }

  if (lv > 1)
    dsplibs_debug_printf("---------------------------------------------------------------------------------------------------------------------\r\n");
}

/* ---- setConstellationToNoise ---- */

/**
 * Sets `dMin_`, the thresholds and the ucode lists from the measured noise.
 * - dMin: unrestricted (`params_->useRestricedDmin` == 0) noise * 6.776 + 9.9, otherwise the larger of noise * 3.333 + 80 and noise * 8.214 + 12.
 * - Per `state_`: 1 (KeepRate) restores the previous dMin and keeps the thresholds; 0 (no restriction) sets the thresholds (up 0.45 * noise, down
 *   1.4125 * noise, retrain factor * noise with factor 4 unrestricted / 2 restricted) and, when `w24_` is 0, clamps a dMin in
 *   0x3f .. 0x43 to 0x3e; 2 (one rate up) limits dMin to `rrnUpDmin_` (when non-zero) and sets the
 *   thresholds; 3 (one rate down) raises dMin to `rrnDownDmin_` (when non-zero) and sets the
 *   thresholds; other values change nothing.  `state_` is then set to 1.
 * - `dMinAlt_` = 1.25 * dMin; a non-negative `params_->forcedDmin` overrides dMin.
 * - For each phase ph: scans ucodes `params_->unk360` .. maxU[ph]; a ucode is accepted when its distance A (and, for a dual phase, B) exceeds the running
 *   half-distance (initially dMin / 2, dMinAlt_ / 2 for a dual phase) and `valid` is set (dual phases skip that test); an accepted ucode sets the half-distance
 *   to max(A, B) + dMin.  The accepted ucodes are written to `mapParams_->listA_[ph]` in descending order with `cnt_[ph]`; `listB_` holds their G.711 codes
 *   (limited to the ucode itself when `useAlaw_` equals `w28_`).
 * @param noise  measured noise energy
 * @param A      per-phase, per-ucode distances, 6 x 128
 * @param B      second distances used by dual phases, 6 x 128
 * @param dual   per-phase flag: the phase uses both A and B
 * @param maxU   per-phase highest ucode to consider
 * @param valid  per-phase, per-ucode allowed flags, 6 x 128
 * @warning The accepted ucodes of one phase are collected in a 140-byte local buffer; a range `unk360` .. maxU[ph] that accepts more than 140 ucodes would
 *          overflow it, and a maxU above 127 reads past the 128-entry rows.  Callers keep maxU within the table.
 */
void V90ConstellationDesigner::setConstellationToNoise(float noise, short (*A)[128], short (*B)[128], short* dual, unsigned char* maxU,
                                                       unsigned char (*valid)[128])
{
  const int16_t* a = &A[0][0];
  const int16_t* b = &B[0][0];
  const uint8_t* vl = &valid[0][0];
  int old = dMin_;
  float retrainFactor;
  LD rA, rB;

  /* the new dMin and the retrain-threshold factor */
  if (params_->useRestricedDmin == 0)
  {
    dMin_ = trunc16((LD)noise * (LD)kC428 + (LD)kC42c);
    rA = __builtin_nanl("");
    rB = __builtin_nanl("");
    retrainFactor = 4.0f;
  }
  else
  {
    LD lA = (LD)noise * (LD)kC434 + 80.0L;
    LD lB = (LD)noise * (LD)kC43c + 12.0L;
    LD m = jmpAe(lB, lA) ? lB : lA;
    dMin_ = trunc16(m);
    rA = lA;
    rB = lB;
    retrainFactor = 2.0f;
  }

  if (dsplibs_debug_level > 1)
  {
    volatile float sA = (float)rA, sB = (float)rB; /* both values are rounded to float before the first print */
    Split s = splitValue((LD)noise, 100.0L);
    dsplibs_debug_printf("V90ConstellationDesigner: noiseEnergy = %c%d.%02d\r\n", s.sign, s.ip, s.fp);
    if (dsplibs_debug_level > 1)
    {
      int hi = (int)trunc16((LD)sA);
      int lo = (int)trunc16((LD)sB);
      dsplibs_debug_printf("V90ConstellationDesigner: dMinHighRates = %d  dMinLowRates = %d\r\n", hi, lo);
      if (dsplibs_debug_level > 1)
        dsplibs_debug_printf("V90ConstellationDesigner: current dMin = %d\r\n", (int)dMin_);
    }
  }

  /* adjust dMin and the thresholds for the rate-change state */
  int st = (int)state_;
  if (st == 1)
  {
    dMin_ = (int16_t)old;
    if (dsplibs_debug_level > 1)
      dsplibs_debug_printf("V90ConstellationDesigner: KeepRate => keep dMin and pdsnr thresh\r\n");
  }
  else if (st <= 0)
  {
    if (st == 0)
    {
      if (dsplibs_debug_level > 1)
        dsplibs_debug_printf("V90ConstellationDesigner: dMin calc NoRestriction\r\n");
      storeF(&threshUp_, (LD)noise * (LD)kC44c);
      storeF(&threshDown_, (LD)noise * (LD)kC450);
      storeF(&threshRetrain_, (LD)retrainFactor * (LD)noise);
      if (w24_ == 0)
      {
        int d = dMin_;
        if (d <= 0x43 && d > 0x3e)
        {
          if (dsplibs_debug_level > 1)
            dsplibs_debug_printf("V90ConstellationDesigner: adjusting dMin for rate>=53k. Orig dMin=%d Modified dMin=%d\r\n", d, 0x3e);
          dMin_ = 0x3e;
        }
      }
    }
  }
  else if (st == 2)
  {
    int r = rrnUpDmin_;
    int d = dMin_;
    int nd;
    if (r == 0)
      nd = d;
    else
      nd = (r <= d) ? r : d;
    dMin_ = (int16_t)nd;
    if (dsplibs_debug_level > 1)
    {
      dsplibs_debug_printf("V90ConstellationDesigner: dMin calc OneRateUp\r\n");
      if (dsplibs_debug_level > 1)
        dsplibs_debug_printf("V90ConstellationDesigner: rrnUpDmin = %d\r\n", (int)rrnUpDmin_);
    }
    storeF(&threshUp_, (LD)noise * (LD)kC44c);
    storeF(&threshDown_, (LD)noise * (LD)kC450);
    storeF(&threshRetrain_, (LD)retrainFactor * (LD)noise);
  }
  else if (st == 3)
  {
    int r = rrnDownDmin_;
    int d = dMin_;
    int nd;
    if (r == 0)
      nd = d;
    else
      nd = (r >= d) ? r : d;
    dMin_ = (int16_t)nd;
    if (dsplibs_debug_level > 1)
    {
      dsplibs_debug_printf("V90ConstellationDesigner: dMin calc OneRateDown\r\n");
      if (dsplibs_debug_level > 1)
        dsplibs_debug_printf("V90ConstellationDesigner: rrnDownDmin = %d\r\n", (int)rrnDownDmin_);
    }
    storeF(&threshUp_, (LD)noise * (LD)kC44c);
    storeF(&threshDown_, (LD)noise * (LD)kC450);
    storeF(&threshRetrain_, (LD)retrainFactor * (LD)noise);
  }

  /* dMinAlt_, the forced dMin and the status prints */
  state_ = 1;
  dMinAlt_ = trunc16((LD)(int16_t)dMin_ * (LD)1.25f);
  int forced = params_->forcedDmin;
  if (forced > -1)
  {
    dMin_ = (int16_t)forced;
    edprintf("V90ConstellationDesigner: dMin Forced to: %d\r\n", (int)(int16_t)forced);
  }
  edprintf("V90ConstellationDesigner: final dMin = %d\r\n", (int)(int16_t)dMin_);
  edprintf("V90ConstellationDesigner: USE_RESTRICED_DMIN = %d\r\n", (int)params_->useRestricedDmin);
  edThresh("V90ConstellationDesigner: pdSnrThreshForRateUp = %c%d.%02d\r\n", &threshUp_);
  edThresh("V90ConstellationDesigner: pdSnrThreshForRateDown = %c%d.%02d\r\n", &threshDown_);
  edThresh("V90ConstellationDesigner: pdSnrThreshForRetrain = %c%d.%02d\r\n", &threshRetrain_);

  /* the per-phase ucode scan */
  V90MappingParams* mp = mapParams_;
  uint8_t buf[0x8c];
  for (unsigned ph = 0; ph < 6; ph++)
  {
    unsigned cnt = 0;
    bool d = dual[ph] != 0;
    int dm = d ? (int)(int16_t)dMinAlt_ : (int)(int16_t)dMin_;
    int16_t half = (int16_t)(dm / 2);
    unsigned lo = (unsigned)params_->unk360;
    unsigned hi = maxU[ph];
    unsigned base = ph * 0x80u;
    for (unsigned u = lo; hi >= u; u++)
    {
      int ix = ixOf(base, u);
      int16_t av = a[ix];
      if (av <= half)
        continue;
      int16_t bv = 0;
      if (d)
      {
        bv = b[ix];
        if (bv <= half)
          continue;
      }
      if (vl[ix] == 0)
        continue;
      buf[cnt++] = (uint8_t)u;
      int m = d ? ((bv >= av) ? bv : av) : av;
      half = (int16_t)(m + dm);
    }
    mp->cnt_[ph] = cnt;

    /* write the lists in descending ucode order with their G.711 codes */
    for (unsigned j = 0; j < cnt; j++)
    {
      uint8_t uc = buf[cnt - 1 - j];
      mp->listA_[ph][j] = uc;
      if (useAlaw_ == w28_)
      {
        uint8_t code = codeOf(this, a[base + uc]);
        if (code >= uc)
        {
          mp->listB_[ph][j] = uc;
        }
        else
        {
          mp->listB_[ph][j] = codeOf(this, a[base + uc]);
        }
      }
      else
      {
        mp->listB_[ph][j] = codeOf(this, a[base + uc]);
      }
    }
  }

  printReport(this, a);
}

/* ---- setConstellationToNoise_forceRate ---- */

/**
 * Builds the ucode lists so that the six phases together carry the requested rate (`params_->rateForce`).
 * 1. n = rate in units + `mapParams_->sr_` - 6, X = 2^n, Q = X times 4 per dual phase times 2 per fixed phase; `s` = floor(Q^(1/6)).
 * 2. Initial counts nu[i]: s, or s / 4 for a dual phase and s / 2 for a fixed phase; while the (scaled) product is below X the smallest count of a
 *    non-fixed phase is incremented.
 * 3. Phase distances phd[i] = A[i][cur[i]] / (nu[i] - 0.5); thresholds up = 0.1 * noise, down = retrain = 10 * noise.
 * 4. For each phase, repeat (at most 200 times): start the list at `cur[i]` and scan the lower ucodes, accepting one whose distance is at most the previous
 *    accepted distance minus phd (a dual phase tests both A and B; a single phase also needs `valid`), stopping when a distance falls below phd / 2; then
 *    scale phd by (1 - step) when the list is too short or (1 + step) when too long, shrinking `step` by 0.9 when the direction flips.
 * 5. A list that is too long loses entries from the front; one that is too short gets entries inserted at the front (`cur[i]` is incremented for each).
 * Results go to `mapParams_->listA_ / listB_ / cnt_`; the thresholds and the debug report are printed.
 * @param noise   measured noise energy
 * @param A       per-phase, per-ucode distances, 6 x 128
 * @param B       second distances used by dual phases, 6 x 128
 * @param dual    per-phase flag: the phase uses both A and B
 * @param fixedU  per-phase flag: the phase keeps a fixed (halved) ucode count
 * @param cur     per-phase starting ucode; updated when a list is grown
 * @param valid   per-phase, per-ucode allowed flags, 6 x 128
 * @bug With n < 0 the doubling loop counts down from a negative value and runs about 2^32 times.
 * @bug The growth loop (step 5) never terminates when no ucode above the first entry up to 0x74 has a distance exceeding that entry's plus phd: the
 *      `continue` re-enters the loop test with unchanged data.
 * @note The growth step only tests whether a higher candidate exists; the entry it inserts at the front is ucode 0 (with its code), not the candidate.
 */
void V90ConstellationDesigner::setConstellationToNoise_forceRate(float noise, short (*A)[128], short (*B)[128], short* dual, unsigned char* fixedU,
                                                                 unsigned char* cur, unsigned char (*valid)[128])
{
  const int16_t* a = &A[0][0];
  const int16_t* b = &B[0][0];
  const uint8_t* vl = &valid[0][0];

  /* step 1: n and X = 2^n */
  int16_t e16 = trunc16((LD)rateScale() * (LD)(int)params_->rateForce + (LD)0.5f);
  int n = (int16_t)((unsigned)(uint16_t)e16 + mapParams_->sr_ - 6u);
  LD X;
  if (n == 0)
  {
    X = 1.0L;
  }
  else
  {
    X = 2.0L;
    if (n != 1)
    {
      int c = n - 1;
      do
      {
        c--;
        X += X;
      } while (c != 0);
    }
  }

  LD prod = 1.0L;
  for (unsigned i = 0; i < 6; i++)
  {
    if (dual[i] != 0)
      prod *= 4.0L;
    else if (fixedU[i] != 0)
      prod += prod;
  }
  LD Q = prod * X;

  /* the integer sixth root: s = the largest t with t^6 <= Q (0x80 if none exceeds it) */
  int16_t s = 0x80;
  for (unsigned t = 1; t <= 0x7f; t++)
  {
    LD tt = (LD)(int16_t)t;
    LD w = tt;
    for (int k = 0; k < 5; k++)
      w *= tt;
    if (jmpA(w, Q))
    {
      s = (int16_t)(t - 1);
      break;
    }
  }

  /* step 2: the initial counts */
  LD sv = (LD)s;
  LD W = sv;
  for (int k = 0; k < 5; k++)
    W *= sv;
  int16_t c4 = trunc16(sv * (LD)0.25f);
  int16_t c5 = trunc16(sv * (LD)0.5f);
  uint16_t nu[6];
  for (unsigned i = 0; i < 6; i++)
  {
    if (dual[i] != 0)
    {
      nu[i] = (uint16_t)c4;
      W *= (LD)0.25f;
    }
    else if (fixedU[i] != 0)
    {
      nu[i] = (uint16_t)c5;
      W *= (LD)0.5f;
    }
    else
    {
      nu[i] = (uint16_t)s;
    }
  }

  /* raise the smallest count of a non-fixed phase until the product reaches X */
  if (!jmpAe(W, X))
  {
    do
    {
      int best = 32000;
      int idx = 0;
      for (unsigned i = 0; i < 6; i++)
      {
        if (fixedU[i] == 0 && (int)nu[i] < best)
        {
          best = (int16_t)nu[i];
          idx = (int16_t)i;
        }
      }
      nu[idx] = (uint16_t)(nu[idx] + 1);
      LD acc = 1.0L;
      for (unsigned j = 0; j < 6; j++)
        acc *= (LD)(uint32_t)nu[j];
      W = acc; /* only the loop test reads it */
    } while (jmpB(W, X));
  }

  /* step 3: the phase distances and the thresholds */
  volatile float phd[6];
  for (unsigned i = 0; i < 6; i++)
  {
    int16_t Ai = a[i * 0x80u + cur[i]];
    LD r = 1.0L / ((LD)(uint32_t)nu[i] - (LD)0.5f);
    LD v = (LD)Ai * r;
    int16_t q = trunc16(v);
    phd[i] = (float)(LD)(int)q; /* fild WORD; fstp DWORD */
  }

  LD t10 = (LD)noise * (LD)10.0f;
  storeF(&threshUp_, (LD)noise * (LD)kC470);
  storeF(&threshDown_, t10);
  storeF(&threshRetrain_, t10);
  {
    int pRetrain = dsp_trunc_i32(t10);
    int pDown = dsp_trunc_i32((LD)loadF(&threshDown_));
    int pUp = dsp_trunc_i32((LD)loadF(&threshUp_));
    edprintf("V90ConstellationDesigner: pdSnrThreshForRateUp = %d pdSnrThreshForRateDown = %d pdSnrThreshForRetrain = %d\r\n", pUp, pDown, pRetrain);
  }

  edprintf("V90ConstellationDesigner: requested rate force is %d\n", (int)params_->rateForce);
  edprintf("V90ConstellationDesigner: initial nofUcodeInPhase[1..6] = %d %d %d %d %d %d\n", (unsigned)nu[0], (unsigned)nu[1], (unsigned)nu[2], (unsigned)nu[3],
           (unsigned)nu[4], (unsigned)nu[5]);
  {
    int p0 = (int)trunc16((LD)phd[0]);
    int p1 = (int)trunc16((LD)phd[1]);
    int p2 = (int)trunc16((LD)phd[2]);
    int p3 = (int)trunc16((LD)phd[3]);
    int p4 = (int)trunc16((LD)phd[4]);
    int p5 = (int)trunc16((LD)phd[5]);
    edprintf("V90ConstellationDesigner: initial phaseDmin[1..6] = %d %d %d %d %d %d\n", p0, p1, p2, p3, p4, p5);
  }

  /* step 4: per phase, iterate the list construction until it has nu[i] entries */
  V90MappingParams* mp = mapParams_;
  for (unsigned i = 0; i < 6; i++)
  {
    volatile float step = kC478;
    int16_t sV9 = 0;
    int16_t iter = 0;
    unsigned cnt = 1;
    unsigned nui = nu[i];
    unsigned base = i * 0x80u;
    if (nui != 1)
    {
      do
      {
        bool d = dual[i] != 0;
        uint8_t bl = cur[i];
        int ix0 = ixOf(base, bl);
        int c = (int)trunc16((LD)phd[i]);
        int L;

        /* L: the largest distance the next accepted ucode may have */
        if (d)
        {
          int pb = (int)b[ix0] - c;
          int pa = (int)a[ix0] - c;
          L = (int16_t)(pb > pa ? pa : pb);
        }
        else
        {
          L = (int16_t)((int)a[ix0] - c);
        }

        mp->listA_[i][0] = bl;
        mp->listB_[i][0] = codeOf(this, a[base + mp->listA_[i][0]]);
        cnt = 1;
        unsigned u = (unsigned)bl - 1u;
        if (!((unsigned)params_->unk360 > u))
        {
          for (;;)
          {
            LD half = (LD)phd[i] * (LD)0.5f;
            int ix = ixOf(base, u);
            int16_t av = a[ix];
            if (jmpB((LD)av, half))
              break;
            bool skip;
            int16_t bv = 0;
            if (d)
            {
              bv = b[ix];
              if (jmpB((LD)bv, half))
                break;
              skip = (av > L) || (bv > L);
            }
            else
            {
              skip = (av > L) || (vl[ix] == 0);
            }
            if (!skip)
            {
              mp->listA_[i][cnt] = (uint8_t)u;
              mp->listB_[i][cnt] = codeOf(this, a[base + mp->listA_[i][cnt]]);
              cnt++;
              int c2 = (int)trunc16((LD)phd[i]);
              if (d)
              {
                int pb = (int)b[ix] - c2;
                int pa = (int)a[ix] - c2;
                L = (int16_t)(pb > pa ? pa : pb);
              }
              else
              {
                L = (int16_t)((int)a[ix] - c2);
              }
            }
            u--;
            if ((unsigned)params_->unk360 > u)
              break;
          }
        }

        /* too few entries: reduce the spacing; too many: increase it */
        nui = nu[i];
        if (nui > cnt)
        {
          phd[i] = (float)(((LD)1.0f - (LD)step) * (LD)phd[i]);
          if (sV9 <= 0)
            sV9 = (int16_t)(sV9 + 1);
        }
        else if (nui < cnt)
        {
          phd[i] = (float)(((LD)step + (LD)1.0f) * (LD)phd[i]);
          if (sV9 > -1)
            sV9 = (int16_t)(sV9 - 1);
        }
        if (sV9 == 0)
          step = (float)((LD)step * (LD)kC47c);
        iter = (int16_t)(iter + 1);
        mp = mapParams_;
      } while (nui != cnt && iter <= 199);
    }

    /* step 5: shrink to nu entries, then grow */
    if (nui < cnt)
    {
      do
      {
        cnt--;
        for (unsigned j = 0; j < cnt; j++)
        {
          mp->listA_[i][j] = mp->listA_[i][j + 1];
          mp->listB_[i][j] = mp->listB_[i][j + 1];
        }
      } while (nui < cnt);
    }
    if (nui > cnt)
    {
      do
      {
        unsigned u0 = mp->listA_[i][0];
        if (u0 > 0x73)
        {
          edprintf("V90ConstellationDesigner: reached max posible ucode -> Not able to reach requested rate !!!");
          mp = mapParams_;
          break;
        }
        bool found = false;
        for (unsigned cand = u0 + 1; cand <= 0x74; cand++)
        {
          LD pc = (LD)a[base + cand];
          LD pu = (LD)a[base + mp->listA_[i][0]];
          LD sum = pu + (LD)phd[i];
          if (jmpB(sum, pc))
          {
            found = true;
            break;
          }
        }
        if (!found)
          continue; /* no candidate: the loop test sees unchanged data and repeats forever */
        for (unsigned k = cnt; k != 0; k--)
        {
          mp->listA_[i][k] = mp->listA_[i][k - 1];
          mp->listB_[i][k] = mp->listB_[i][k - 1];
        }
        mp->listA_[i][0] = 0;
        mp->listB_[i][0] = codeOf(this, a[base + mp->listA_[i][0]]);
        cur[i] = (uint8_t)(cur[i] + 1);
        mp = mapParams_;
        cnt++;
      } while (nui > cnt);
    }
    mp->cnt_[i] = cnt;
  }

  {
    unsigned c5 = mapParams_->cnt_[5];
    unsigned c4 = mapParams_->cnt_[4];
    unsigned c3 = mapParams_->cnt_[3];
    unsigned c2 = mapParams_->cnt_[2];
    unsigned c1 = mapParams_->cnt_[1];
    unsigned c0 = mapParams_->cnt_[0];
    edprintf("V90ConstellationDesigner: final nofUcodeInPhase[1..6] = %d %d %d %d %d %d\n", c0, c1, c2, c3, c4, c5);
  }

  {
    int p0 = (int)trunc16((LD)phd[0]);
    int p1 = (int)trunc16((LD)phd[1]);
    int p2 = (int)trunc16((LD)phd[2]);
    int p3 = (int)trunc16((LD)phd[3]);
    int p4 = (int)trunc16((LD)phd[4]);
    int p5 = (int)trunc16((LD)phd[5]);
    edprintf("V90ConstellationDesigner: final phaseDmin[1..6] = %d %d %d %d %d %d\n", p0, p1, p2, p3, p4, p5);
  }
  printReport(this, a);
}
