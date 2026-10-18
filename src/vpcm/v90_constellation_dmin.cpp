/**
 * @file v90_constellation_dmin.cpp
 * @brief V90ConstellationDesigner: the top-level design step (process), the rate-step minimum distances, and the small mapping helpers.
 *
 * Part of the V.90 digital-side constellation designer (class declared in v90_constellation_designer.h; its noise-driven ucode selection is in
 * v90_constellation_noise.cpp and its power / redundancy adjustments in v90_constellation_power.cpp).  This file holds:
 * - construction, reset() and setMinMaxRates() (the rate window the design must respect);
 * - process(): the entry point called by the demodulator.  It records its arguments, copies the spectral-shaper constants into the mapping block
 *   (spectralBody(), also exposed as spectralDesign()), clamps the rate limits to 28000 .. 56000, and designs the constellation at most twice:
 *   setConstellationToNoise() (or its forced-rate variant), adjustConstellationsPower(), adjustConstellationsToNewK().  From the resulting
 *   redundancy K it derives D = K - S + 6 and the rate D * 8000 / 6; a rate outside [minRate_, maxRate_] is forced through P->rateForce and the
 *   design is redone.  It also sets enableRrnUp / enableRrnDown so that the connection evaluator cannot step outside the window;
 * - determineDminForRrn(): the minimum distances rrnDownDmin_ / rrnUpDmin_ at which the constellation would be one rate step down / up.  It picks the
 *   phase with the largest count (among the phases the detector does not suspect), normalises the six counts to it, solves for the count M that moves
 *   K by one step (-0.75 / +1.05 bits) with the exponent t = (target - K) / 6, M = trunc(2^n * 1.0069555^q) (n whole doublings, q
 *   hundredths of one), checks the resulting K by the log10 formula and nudges M by one, then scans that phase's distance table with the distance
 *   growing by 2% (down) or shrinking by 5% (up) until the number of
 *   usable ucodes falls below / reaches M (at most 100 iterations);
 * - small helpers on the mapping block (pow6, maxK, calcK, constelBuild, calcMtoMatchKtarget, realK, findConstelMaxValueIndex, findMinValueIndex,
 *   findNextUcodeToAdd, reconstructInitialConditions), which no other source file in this library calls.
 *
 * Arithmetic: K, the log10 products and the M search are carried out in extended precision (long double), with float slots (storeF / loadF) where the
 * values are rounded to single precision, the product of the normalised counts kept as a double, and 16- / 64-bit truncating conversions
 * (trunc16 / trunc64lo).  Comparisons use jmpA / jmpAe, which treat an unordered result like the x87 branches do (jmpA false, jmpAe false).
 *
 * Behaviour worth knowing about:
 * - determineDminForRrn() re-reads dsplibs_debug_level after every print (a print may change it) and re-reads mapParams_ after the prints that can change it;
 * - the rate-down half multiplies by a reciprocal, (1 / log10(2)), the rate-up half divides: the two differ in the last rounding;
 * - the M search loops are counted by UNSIGNED variables; a negative exponent would loop about 2^32 times;
 * - process() counts the product from its mp argument but writes kBits_ through this->mapParams_ (the same object as long as the caller passes mp).
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

/* single-precision constants (kEps6, the 1e-6 guard added before truncating K, comes from v90_mapping_prod.h) */
#define kEps9 (bitsFloat(0x3089705fu))  /* 9.99999972e-10, guard of realK() */
#define kSixth (bitsFloat(0x3e2aaaabu)) /* 0.16666667: M is the sixth root of the six-phase product */
#define kPow (bitsFloat(0x3f80e3ebu))   /* 1.0069555 = 2^(1/100), base of the fractional part of the exponent */

/* ---- helpers (static inline) ---- */

#include "core/dsp_x87_cmp.h" /* jmpA / jmpAe: ordered compares, false when unordered */

#include "core/dsp_x87_conv.h"

#include "v90_mapping_rows.h"

/**
 * The detector's tables the designer reads through flat accessors: linearMapping_ (int16 [phase * 128 + ucode], at offset 0), ucodeAllowed_ (uint8 at
 * 0xd00) and suspectedPhase_ (at 0x280c).  adLin() / adOk() do not bound the ucode: an index above 127 runs on into the next phase's row.
 */
DSP_STATIC_ASSERT(offsetof(V90AutoDigitalImpDetector, linearMapping_) == 0 && offsetof(V90AutoDigitalImpDetector, ucodeAllowed_) == 0xd00 &&
                      offsetof(V90AutoDigitalImpDetector, suspectedPhase_) == 0x280c,
                  "the detector tables the designer reads are at the expected offsets");
/** @brief linearMapping_ entry (phase * 128 + ucode) of the detector. */
static inline int16_t adLin(const V90AutoDigitalImpDetector* a, unsigned i) { return ((const int16_t*)(const void*)a)[i]; }
/** @brief ucodeAllowed_ flag (phase * 128 + ucode) of the detector. */
static inline uint8_t adOk(const V90AutoDigitalImpDetector* a, unsigned i) { return ((const uint8_t*)(const void*)a)[0xd00u + i]; }

#include "v90_mapping_prod.h"
/** @brief log10(x) rounded to single precision (through a float slot) and returned as a float. */
static inline float lg10F(LD x)
{
  float slot;
  return storeF(&slot, dsp_log10(x));
}

/**
 * @brief Integer redundancy K of a mapping block: trunc(log10(prod) / (float)log10(2) + 1e-6f), 0 when the product of the six counts is 0.
 * @note  The log10 stays in extended precision.  Used by maxK().
 */
static inline uint32_t kInt(const V90MappingParams* mp)
{
  LD p = prodCnt(mp);
  if (p == 0.0L)
    return 0u;
  LD lg = dsp_log10(p);
  float den = lg2F();
  return trunc64lo(lg / (LD)den + (LD)kEps6);
}

/**
 * @brief K (in bits) of the constellation whose phase i holds m * nrm[i] points: (float)log10(prod_i m * nrm[i]) * (1 / (float)log10(2)), extended.
 * @param m    count of the reference phase (a 32-bit value or a byte count, zero-extended)
 * @param nrm  six normalised counts (cnt[i] / maxM)
 * @note  The K check of determineDminForRrn().
 */
static inline LD kqMul(uint32_t m, const float* nrm)
{
  LD ml = (LD)m;
  LD acc = 1.0L;
  for (int i = 0; i <= 5; i++)
  {
    LD t = ml * (LD)loadF(&nrm[i]);
    acc = acc * t;
  }
  float lgf = lg10F(acc);
  float den = lg2F();
  LD recip = (LD)1.0L / (LD)den;
  return (LD)lgf * recip;
}

/**
 * @brief Count M that realises an exponent: M = trunc(2^n * 1.0069555^q).
 * @param t  exponent (target - log10(prod) / log10(2)) / 6, in doublings of M
 * @note  n = trunc(t) doublings of a 32-bit value, q = trunc((t - n) * 100) extended multiplications by 1.0069555f.
 * @warning Both loop counts are unsigned: a negative t loops about 2^32 times.
 */
static inline uint32_t mFromT(LD t)
{
  uint32_t n = trunc64lo(t);
  uint32_t p2 = 1u;
  for (uint32_t c = n; c != 0u; c--)
    p2 += p2;
  uint32_t q = trunc64lo((t - (LD)n) * (LD)(100.0f));
  LD pw = 1.0L;
  if (q != 0u)
  {
    LD c = (LD)kPow;
    for (uint32_t k = q; k != 0u; k--)
      pw = pw * c;
  }
  return trunc64lo(pw * (LD)p2);
}

/** @brief Data rate in bit/s for D bits per symbol: (D * 8000) / 6. */
static inline uint32_t rateOfD(uint32_t d) { return (d * 8000u) / 6u; }

/**
 * @brief Copy the spectral-shaper constants of the parameter block into the mapping block.
 * @param P          parameter block
 * @param m          mapping block to fill (id_, sr_, shaperA1_, shaperA2_, shaperB1_, shaperB2_)
 * @param lookahead  upper bound for the shaper id
 * @param special    V90SSC_PBX selects the German-PBX set, anything else the normal set
 * @note  The float constants move as raw 32-bit words.
 */
static inline void spectralBody(V90Parameters* P, V90MappingParams* m, unsigned lookahead, V90SpecialSpectralConditions special)
{
  if (special == V90SSC_PBX)
  {
    uint32_t id = (uint32_t)P->germanPbxSpectralShaperId;
    m->id_ = id > lookahead ? lookahead : id;
    __builtin_memcpy(&m->shaperA1_, &P->germanPbxSpectralShaperA1, 4);
    m->sr_ = (uint32_t)P->germanPbxSpectralShaperSr;
    __builtin_memcpy(&m->shaperA2_, &P->germanPbxSpectralShaperA2, 4);
    __builtin_memcpy(&m->shaperB1_, &P->germanPbxSpectralShaperB1, 4);
    __builtin_memcpy(&m->shaperB2_, &P->germanPbxSpectralShaperB2, 4);
  }
  else
  {
    uint32_t id = (uint32_t)P->spectralShaperId;
    m->id_ = id > lookahead ? lookahead : id;
    __builtin_memcpy(&m->shaperA1_, &P->spectralShaperA1, 4);
    m->sr_ = (uint32_t)P->spectralShaperSr;
    __builtin_memcpy(&m->shaperA2_, &P->spectralShaperA2, 4);
    __builtin_memcpy(&m->shaperB1_, &P->spectralShaperB1, 4);
    __builtin_memcpy(&m->shaperB2_, &P->spectralShaperB2, 4);
  }
}

/**
 * @brief Count the usable ucodes of one phase for a trial minimum distance.
 * @param mp         mapping block (listA[phase][0] is the last ucode considered)
 * @param ad         detector whose linear-mapping and allowed tables are read
 * @param phase      phase 0..5
 * @param lo         first ucode considered
 * @param tempDmin   trial minimum distance
 * @return number of ucodes in [lo, listA[phase][0]] whose table distance exceeds a running half distance (starting at tempDmin / 2, becoming
 *         (int16)(tempDmin + distance) after each accepted ucode) and that the detector allows; a byte counter, so it wraps at 256
 */
static inline uint8_t scanCount(V90MappingParams* mp, const V90AutoDigitalImpDetector* ad, unsigned phase, uint32_t lo, int32_t tempDmin)
{
  uint8_t cnt = 0;
  int32_t half = tempDmin >> 1;
  if (lo <= (uint32_t)LA(mp, phase, 0))
  {
    uint32_t idx = lo;
    do
    {
      uint16_t e = (uint16_t)adLin(ad, phase * 128u + idx);
      if ((int16_t)e > (int16_t)half && adOk(ad, phase * 128u + idx) != 0)
      {
        cnt++;
        half = (int16_t)(uint16_t)(tempDmin + (int32_t)e);
      }
      idx++;
    } while ((uint32_t)LA(mp, phase, 0) >= idx);
  }
  return cnt;
}

/* ---- ctor / dtor / reset / setMinMaxRates ---- */

/**
 * @brief Create a designer.
 * @param p   parameter block, borrowed
 * @param pf  pre-filter object, borrowed
 * @param pw  constellation power calculator, borrowed
 * @note  The rate window starts at 28000 .. 56000 bit/s and the power index limit at 0x16 (22).
 */
V90ConstellationDesigner::V90ConstellationDesigner(V90Parameters* p, V90PreFilter* pf, V90ConstellationPower* pw)
{
  flag8_ = 0;
  state_ = 0;
  powerLimit_ = 0x16;
  power_ = pw;
  params_ = p;
  minRate_ = 28000u;
  maxRate_ = 56000u;
  preFilter_ = pf;
}

/** @brief Destructor; the designer owns nothing. */
V90ConstellationDesigner::~V90ConstellationDesigner() {}

/** @brief Clear the design state: state_, the three minimum distances, and w24_ (reloaded from the parameter block's unk39c). */
void V90ConstellationDesigner::reset()
{
  V90Parameters* p = params_;
  state_ = 0;
  dMin_ = 0;
  w24_ = (uint32_t)p->unk39c;
  rrnDownDmin_ = 0;
  rrnUpDmin_ = 0;
  dMinAlt_ = 0;
}

/**
 * @brief Set the rate window the next process() must respect.
 * @param minRate  lowest acceptable rate in bit/s
 * @param maxRate  highest acceptable rate in bit/s
 * @note  process() clamps both to 28000 .. 56000 and makes minRate_ <= maxRate_.
 */
void V90ConstellationDesigner::setMinMaxRates(unsigned minRate, unsigned maxRate)
{
  unsigned lv = dsplibs_debug_level;
  minRate_ = minRate;
  maxRate_ = maxRate;
  if (lv > 1)
  {
    dsplibs_debug_printf("V90ConstellationDesigner: set min rate to %d\r\n", (int)minRate);
    if (dsplibs_debug_level > 1)
      dsplibs_debug_printf("V90ConstellationDesigner: set max rate to %d\r\n", (int)maxRate_);
  }
}

/* ---- determineDminForRrn ---- */

/**
 * @brief Find the minimum distances at which the constellation moves one rate step down and one rate step up.
 * @param K  redundancy (bits) of the current constellation
 * @note  Reads mapParams_ (the six counts cnt_ and list A heads), adid_ (suspected phases, linear mapping, allowed ucodes), dMin_ and the parameter
 *        block (unk360 = first ucode scanned).  Writes rrnDownDmin_ and rrnUpDmin_.
 *        Rate down: M is solved for K - 0.75 and nudged by one when the K check (K - 0.8, then K - 0.3) fails; if the reference phase already has
 *        no more than M points, rrnDownDmin_ = 1.25 * dMin_ ("arbitrary"), otherwise the distance is raised by 2% per iteration while at least M
 *        ucodes remain, and the last distance that still gives M (or the one after it, by the K check) is stored.
 *        Rate up: M is solved for K + 1.05 and nudged when the K check (K + 1.0, then K + 1.7) fails; failure of the nudge stores dMin_; if the
 *        reference phase already has at least M points, rrnUpDmin_ = 0.9 * dMin_; otherwise the distance is lowered by 5% per iteration until M
 *        ucodes are reached, and dMin_ is stored if that would give two rate steps up.
 * @warning All six counts zero or all phases suspected makes maxM zero: the normalisation divides by it and the result is not meaningful.  The
 *          searches stop after 100 iterations without a fall-back value.
 */
void V90ConstellationDesigner::determineDminForRrn(unsigned K)
{
  unsigned lv = dsplibs_debug_level;
  if (lv > 1)
  {
    dsplibs_debug_printf("V90ConstellationDesigner::determineDminForRrn : pParams->m[1..6] = %d %d %d %d %d %d\r\n", (int)mapParams_->cnt_[0],
                         (int)mapParams_->cnt_[1], (int)mapParams_->cnt_[2], (int)mapParams_->cnt_[3], (int)mapParams_->cnt_[4], (int)mapParams_->cnt_[5]);
    lv = dsplibs_debug_level;
  }
  V90MappingParams* mp = mapParams_;

  /* reference phase: the largest count among the phases the detector does not suspect (strictly larger counts only; unsigned) */
  uint32_t maxM = 0;
  int phase = 0;
  for (unsigned i = 0; i <= 5; i++)
  {
    uint32_t m = mp->cnt_[i];
    if (m > maxM && adid_->suspectedPhase_[i] == 0)
    {
      maxM = m;
      phase = (int16_t)i;
    }
  }

  /* normalised counts and their product */
  float nrm[6];
  LD recip = (LD)1.0L / (LD)maxM;
  LD pn = 1.0L;
  for (unsigned i = 0; i <= 5; i++)
  {
    LD r = (LD)mp->cnt_[i] * recip;
    storeF(&nrm[i], r);
    pn = pn * r;
  }
  volatile double pnD = (double)pn;

  /* ---- rate down: count M that is one step below K ---- */
  uint32_t mDown;
  {
    float lgf = lg10F((LD)pnD);
    float den = lg2F();
    LD recip2 = (LD)1.0L / (LD)den;
    LD t = (((LD)K - (LD)0.75f) - (LD)lgf * recip2) * (LD)kSixth;
    mDown = mFromT(t);
  }
  if (!jmpA((LD)K - (LD)0.8f, kqMul(mDown, nrm)))
  {
    /* K - 0.8 <= kq(mDown) (or unordered): keep mDown; the level test and the print are the common ones below */
  }
  else
  {
    if (lv > 1)
    {
      dsplibs_debug_printf("V90ConstellationDesigner:: rrn down maxM too low - probably will cause more then one rate down\r\n");
      lv = dsplibs_debug_level;
    }
    uint32_t mD1 = mDown + 1u;
    if (!jmpA((LD)K - (LD)0.3f, kqMul(mD1, nrm)))
    {
      mp = mapParams_;
    }
    else
    {
      mDown = mD1;
      if (lv > 1)
      {
        dsplibs_debug_printf("V90ConstellationDesigner:: increment maxM for one rate down\r\n");
        lv = dsplibs_debug_level;
      }
      mp = mapParams_;
    }
  }
  if (lv > 1)
  {
    dsplibs_debug_printf("V90ConstellationDesigner:: current maxM = %d  rrn down maxM = %d\r\n", (int)mp->cnt_[phase], (int)mDown);
    lv = dsplibs_debug_level;
    mp = mapParams_;
  }

  /* the reference phase has no more points than M: no scan needed */
  if (mp->cnt_[phase] <= mDown)
  {
    rrnDownDmin_ = trunc16((LD)(int16_t)dMin_ * (LD)1.25f + (LD)0.5f);
    if (lv > 1)
    {
      dsplibs_debug_printf("V90ConstellationDesigner:: maxM>pParams->m[phase] seting rrnDownDmin arbitrary !!!\r\n");
      lv = dsplibs_debug_level;
      if (lv > 1)
      {
        dsplibs_debug_printf("V90ConstellationDesigner:: rrnDownDmin = %d\r\n", (int)rrnDownDmin_);
        lv = dsplibs_debug_level;
      }
    }
  }
  else
  {
    /* raise the distance by 2% until fewer than M ucodes are usable; the previous distance still gave M or more */
    uint8_t u = (uint8_t)mp->cnt_[phase];
    int32_t tempDmin = (int32_t)dMin_;
    uint32_t iter = 0;
    uint8_t prevCount = 0;
    int32_t prevDmin = 0;
    V90Parameters* P = params_;
    if (u >= mDown)
    {
      do
      {
        prevCount = u;
        prevDmin = tempDmin;
        tempDmin = (int32_t)trunc16((LD)(int16_t)prevDmin * (LD)1.02f + (LD)1.0f);
        uint32_t lo = (uint32_t)P->unk360;
        u = scanCount(mp, adid_, (unsigned)phase, lo, tempDmin);
        iter++;
      } while (u >= mDown && iter <= 99u);
    }
    if (lv > 1)
    {
      dsplibs_debug_printf("V90ConstellationDesigner:: rrn down - nofUcodes = %d  prevNofUcodes = %d\r\n", (int)u, (int)prevCount);
      lv = dsplibs_debug_level;
      if (lv > 1)
      {
        dsplibs_debug_printf("V90ConstellationDesigner:: rrn down - tempDmin  = %d  prevDmin      = %d\r\n", (int)tempDmin, (int)prevDmin);
        lv = dsplibs_debug_level;
      }
    }

    /* choose between the last distance that gave M or more and the first that gave fewer */
    if ((uint32_t)prevCount == mDown)
    {
      rrnDownDmin_ = (int16_t)prevDmin;
    }
    else if (!jmpA((LD)K - (LD)0.8f, kqMul(u, nrm)))
    {
      rrnDownDmin_ = (int16_t)tempDmin;
    }
    else if (!jmpA((LD)K - (LD)0.3f, kqMul(prevCount, nrm)))
    {
      rrnDownDmin_ = (int16_t)tempDmin;
    }
    else
    {
      rrnDownDmin_ = (int16_t)prevDmin;
    }
    if (lv > 1)
    {
      dsplibs_debug_printf("V90ConstellationDesigner:: rrnDownDmin = %d\r\n", (int)rrnDownDmin_);
      lv = dsplibs_debug_level;
    }
  }

  /* ---- rate up: count M that is one step above K ---- */
  uint32_t mUp;
  {
    float lgf = lg10F((LD)pnD);
    float den = lg2F();
    LD t = (((LD)K + (LD)1.05f) - (LD)lgf / (LD)den) * (LD)kSixth;
    mUp = mFromT(t);
  }
  bool failed = false;
  if (!jmpA((LD)K + (LD)1.0L, kqMul(mUp, nrm)))
  {
  }
  else
  {
    if (lv > 1)
    {
      dsplibs_debug_printf("V90ConstellationDesigner:: maxM too high - probably will not cause rate up\r\n");
      lv = dsplibs_debug_level;
    }
    uint32_t mU1 = mUp + 1u;
    if (!jmpA((LD)K + (LD)1.7f, kqMul(mU1, nrm)))
    {
      failed = true;
    }
    else
    {
      mUp = mU1;
      if (lv > 1)
      {
        dsplibs_debug_printf("V90ConstellationDesigner:: increment maxM for one rate up\r\n");
        lv = dsplibs_debug_level;
      }
    }
  }
  if (lv > 1)
  {
    dsplibs_debug_printf("V90ConstellationDesigner:: current maxM = %d  rrn up maxM = %d\r\n", (int)mapParams_->cnt_[phase], (int)mUp);
    lv = dsplibs_debug_level;
  }

  /* the K check failed even with the nudge: no rate-up distance */
  if (failed)
  {
    rrnUpDmin_ = dMin_;
    if (lv > 1)
    {
      dsplibs_debug_printf("V90ConstellationDesigner:: failed to find rrnUpDmin => seting rrnUpDmin = dMin !!!\r\n");
      lv = dsplibs_debug_level;
      if (lv > 1)
        dsplibs_debug_printf("V90ConstellationDesigner:: rrnUpDmin = %d\r\n", (int)rrnUpDmin_);
    }
    return;
  }

  /* the reference phase already has at least M points */
  mp = mapParams_;
  if (mp->cnt_[phase] >= mUp)
  {
    rrnUpDmin_ = trunc16((LD)(int16_t)dMin_ * (LD)0.9f + (LD)0.5f);
    if (lv > 1)
    {
      dsplibs_debug_printf("V90ConstellationDesigner:: maxM<pParams->m[phase] seting rrnUpDmin arbitrary !!!\r\n");
      lv = dsplibs_debug_level;
      if (lv > 1)
        dsplibs_debug_printf("V90ConstellationDesigner:: rrnUpDmin = %d\r\n", (int)rrnUpDmin_);
    }
    return;
  }

  /* lower the distance by 5% until at least M ucodes are usable */
  {
    uint8_t u = (uint8_t)mp->cnt_[phase];
    int32_t tempDmin = (int32_t)dMin_;
    uint32_t iter = 0;
    V90Parameters* P = params_;
    if (u < mUp)
    {
      do
      {
        tempDmin = (int32_t)trunc16((LD)(int16_t)tempDmin * (LD)0.95f);
        uint32_t lo = (uint32_t)P->unk360;
        u = scanCount(mp, adid_, (unsigned)phase, lo, tempDmin);
        iter++;
      } while (u < mUp && iter <= 99u);
    }
    if (lv > 1)
    {
      dsplibs_debug_printf("V90ConstellationDesigner:: rrn up nofUcodes = %d \r\n", (int)u);
      lv = dsplibs_debug_level;
      if (lv > 1)
      {
        dsplibs_debug_printf("V90ConstellationDesigner:: rrn up tempDmin  = %d \r\n", (int)tempDmin);
        lv = dsplibs_debug_level;
      }
    }
    if (jmpAe((LD)K + (LD)1.7f, kqMul(u, nrm)))
    {
      rrnUpDmin_ = (int16_t)tempDmin;
    }
    else
    {
      /* the trial distance may give two rate steps up: fall back to dMin_ (the first store is only visible while the message is printed) */
      if (lv > 1)
      {
        rrnUpDmin_ = (int16_t)tempDmin;
        dsplibs_debug_printf("V90ConstellationDesigner:: tempDmin might cause 2 rates up => rrnUpDmin = dMin\r\n");
        lv = dsplibs_debug_level;
      }
      rrnUpDmin_ = dMin_;
    }
  }
  if (lv > 1)
    dsplibs_debug_printf("V90ConstellationDesigner:: rrnUpDmin = %d\r\n", (int)rrnUpDmin_);
}

/* ---- process ---- */

/**
 * @brief Design the constellation for a measured noise level and the allowed rates.
 * @param lookahead  largest spectral-shaper id accepted
 * @param adid       detector supplying the linear mapping, allowed ucodes, PCM type and companding law
 * @param noise      measured noise energy
 * @param rateMask   bit mask of allowed digital rates, indexed by D - 21
 * @param mp         mapping block to fill (lists A / B, counts, kBits_, sr_, ...); stored as mapParams_
 * @param A          per-phase A distance tables
 * @param B          per-phase B distance tables
 * @param dual       per-phase flag: the phase uses both tables
 * @param fixedU     ucodes fixed by the caller for the forced-rate design
 * @param cur        current ucode limits per phase
 * @param powerLimit power index limit (stored as powerLimit_)
 * @param codec      hardware codec type (stored, not read)
 * @param arg13      thirteenth argument (stored as w40_, not read)
 * @param special    special spectral condition (selects the shaper set)
 * @return 0 on success, 1 when the chosen D is at or below the minimum (20)
 * @note  Stores its arguments in the object, copies the spectral-shaper constants, clamps the rate window, then designs at most twice: when the
 *        resulting rate lies outside [minRate_, maxRate_] it forces the nearest limit through the parameter block (forceRateEnable, rateForce,
 *        power reduction and redundancy optimisation off) and designs again.  Afterwards enableRrnUp / enableRrnDown are set according to whether
 *        the window is narrower than 28000 .. 56000 and whether the rate sits at a limit, and w24_ takes the rate when it is still 0.
 * @warning A rate masked out by rateMask is only reported (a banner is printed); the design continues with that rate.
 */
int V90ConstellationDesigner::process(unsigned lookahead, V90AutoDigitalImpDetector* adid, float noise, int rateMask, V90MappingParams* mp, short (*A)[128],
                                      short (*B)[128], short* dual, unsigned char* fixedU, unsigned char* cur, unsigned char powerLimit,
                                      __tHardwareCodecTypes__ codec, unsigned arg13, V90SpecialSpectralConditions special)
{
  unsigned retv = 0;
  int flag = 0;
  int cnt = 1;
  uint32_t rate;
  uint32_t kk;
  V90MappingParams* m2;

  adid_ = adid;
  w28_ = (uint32_t)adid->pcmType_;
  w40_ = arg13;
  codec_ = (uint32_t)codec;
  useAlaw_ = (uint32_t)adid->codecIsAlaw_;
  powerLimit_ = powerLimit;
  mapParams_ = mp;
  mp->flag_ = 1u;
  edprintf("V90ConstellationDesigner: digital rate mask %d\r\n", rateMask);
  spectralBody(params_, mapParams_, lookahead, special);

  /* clamp the rate window to 28000 .. 56000 and keep minRate_ <= maxRate_ */
  {
    uint32_t mn = minRate_;
    if (mn < 28000u)
    {
      minRate_ = 28000u;
      mn = 28000u;
    }
    else if (mn > 56000u)
    {
      minRate_ = 56000u;
      mn = 56000u;
    }
    uint32_t mx = maxRate_;
    if (mx < 28000u)
    {
      maxRate_ = 28000u;
      mx = 28000u;
    }
    else if (mx > 56000u)
    {
      maxRate_ = 56000u;
      mx = 56000u;
    }
    if (mn > mx)
      minRate_ = mx;
  }

  /* design loop: at most two passes */
  for (;;)
  {
    V90Parameters* P = params_;
    cnt = (int16_t)(cnt + 1);
    unsigned char (*valid)[128] = (unsigned char (*)[128])(void*)adid_->ucodeAllowed_;
    if (P->forceRateEnable == 0)
      setConstellationToNoise(noise, A, B, dual, cur, valid);
    else
      setConstellationToNoise_forceRate(noise, A, B, dual, fixedU, cur, valid);
    P = params_;
    if (P->enableDigitalPowerReduction != 0)
    {
      adjustConstellationsPower();
      P = params_;
    }
    if (P->enableRedundancyOptimization != 0)
      adjustConstellationsToNewK(A, B, dual, valid);

    /* redundancy K of the result, limited to 0x2a - (6 - S) */
    m2 = mapParams_;
    {
      uint32_t limit = (uint8_t)(0x2au - (uint8_t)(6u - (uint8_t)m2->sr_));
      uint32_t k = 0;
      LD prod = (LD)mp->cnt_[0];
      prod = prod * (LD)mp->cnt_[1];
      prod = prod * (LD)mp->cnt_[2];
      prod = prod * (LD)mp->cnt_[3];
      prod = prod * (LD)mp->cnt_[4];
      prod = prod * (LD)mp->cnt_[5];
      if (prod != 0.0L)
      { /* a product of counts is never NaN, so the zero test is exact */
        LD lg = dsp_log10(prod);
        float den = lg2F();
        k = trunc64lo(lg / (LD)den + (LD)kEps6);
      }
      kk = k > limit ? limit : k;
    }

    /* D = K - S + 6, at most 0x2a; above that K is reduced to match */
    {
      uint32_t d = kk - m2->sr_ + 6u;
      if (d <= 0x2au)
      {
        m2->kBits_ = d;
      }
      else
      {
        m2->kBits_ = 0x2au;
        kk = m2->sr_ + 0x24u;
      }
    }

    /* report a rate the provider mask excludes */
    if (((rateMask >> ((m2->kBits_ - 0x15u) & 0x1fu)) & 1) == 0)
    {
      edprintf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n");
      edprintf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n");
      edprintf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n");
      edprintf("!!!!!!!!!!!!!!!!!!    V90ConstellationDesigner: Rate Used Masked By Provider    !!!!!!!!!!!!!!!!!!\r\n");
      edprintf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n");
      edprintf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n");
      edprintf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n");
      m2 = mapParams_;
    }

    /* rate of D, the flag8_ value used by the power adjustment, and the result report */
    {
      uint32_t d = m2->kBits_;
      rate = rateOfD(d);
      uint8_t f = (uint8_t)(0x2au - (uint8_t)d);
      if (f > 4)
        f++;
      flag8_ = f;
      if (m2->kBits_ <= 0x14u)
      {
        edprintf("V90ConstellationDesigner: Connection design ERROR, D choosen is smaller than minimum.\r\n");
        retv = 1;
      }
      else
      {
        edprintf("V90ConstellationDesigner: S = %d\r\n", (int)(6u - m2->sr_));
        edprintf("V90ConstellationDesigner: K = %d\r\n", (int)kk);
        edprintf("V90ConstellationDesigner: final rate chosen - D = %d ( %d )\r\n", (int)mapParams_->kBits_, (int)rate);
      }
    }

    /* outside the rate window: force the limit and design again */
    {
      uint32_t mx = maxRate_;
      if (mx < rate)
      {
        edprintf("V90ConstellationDesigner: currentRate = %d is larger then maxRate = %d => seting currentRate = %d\r\n", (int)rate, (int)mx, (int)mx);
        P = params_;
        P->forceRateEnable = 1;
        P->enableDigitalPowerReduction = 0;
        P->enableRedundancyOptimization = 0;
        P->rateForce = (int)maxRate_;
      }
      else
      {
        uint32_t mn = minRate_;
        if (mn > rate)
        {
          edprintf("V90ConstellationDesigner: currentRate = %d is smaller then minRate = %d => seting currentRate = %d\r\n", (int)rate, (int)mn, (int)mn);
          P = params_;
          P->forceRateEnable = 1;
          P->enableDigitalPowerReduction = 0;
          P->enableRedundancyOptimization = 0;
          P->rateForce = (int)minRate_;
        }
        else
        {
          flag = 1;
        }
      }
    }
    if (!(flag == 0 && cnt <= 2))
      break;
  }

  /* tell the connection evaluator which rate steps are possible */
  if (maxRate_ != 56000u)
    params_->enableRrnUp = 1;
  if (minRate_ != 28000u)
    params_->enableRrnDown = 1;
  if (maxRate_ <= rate)
    params_->enableRrnUp = 0;
  if (rate <= minRate_)
    params_->enableRrnDown = 0;
  if (w24_ == 0u)
    w24_ = rate;
  return (int)retv;
}

/* ---- constellationDesign / spectralDesign ---- */

/**
 * @brief One design pass: noise-driven ucode selection (forced-rate variant when forceRateEnable), then the optional power and redundancy adjustments.
 * @note  Same arguments as the corresponding steps of process(); no other source file in this library calls it (process() performs these steps itself).
 */
void V90ConstellationDesigner::constellationDesign(float noise, short (*A)[128], short (*B)[128], short* dual, unsigned char* fixedU, unsigned char* cur,
                                                   unsigned char (*valid)[128])
{
  if (params_->forceRateEnable == 0)
    setConstellationToNoise(noise, A, B, dual, cur, valid);
  else
    setConstellationToNoise_forceRate(noise, A, B, dual, fixedU, cur, valid);
  if (params_->enableDigitalPowerReduction != 0)
    adjustConstellationsPower();
  if (params_->enableRedundancyOptimization != 0)
    adjustConstellationsToNewK(A, B, dual, valid);
}

/** @brief Copy the spectral-shaper constants into mapParams_ (see spectralBody()). */
void V90ConstellationDesigner::spectralDesign(unsigned lookahead, V90SpecialSpectralConditions special)
{
  V90Parameters* P = params_;
  V90MappingParams* m = mapParams_;
  spectralBody(P, m, lookahead, special);
}

/* ---- mapping helpers (no other source file in this library calls them) ---- */

/** @brief x to the sixth power, returned as an extended-precision value in a float-typed function. */
DSP_EXTRET_FN float V90ConstellationDesigner::pow6(short x)
{
  LD xl = (LD)x;
  LD acc = xl;
  for (int i = 2; i <= 6; i++)
    acc = acc * xl;
  return dsp_ret_ext_float(acc);
}

/** @brief Integer redundancy K of the mapping block (see kInt()). */
unsigned V90ConstellationDesigner::maxK(V90MappingParams* mp) { return kInt(mp); }

/**
 * @brief K in bits of the constellation whose phase i holds m * arr[i] points.
 * @param m    reference count
 * @param arr  six normalised counts
 */
DSP_EXTRET_FN float V90ConstellationDesigner::calcK(unsigned m, float* arr)
{
  LD ml = (LD)m;
  LD acc = 1.0L;
  for (int i = 0; i <= 5; i++)
  {
    LD t = ml * (LD)arr[i];
    acc = acc * t;
  }
  float lgf = lg10F(acc);
  float den = lg2F();
  LD recip = (LD)1.0L / (LD)den;
  return dsp_ret_ext_float((LD)lgf * recip);
}

/**
 * @brief Number of usable ucodes of a phase for a trial minimum distance.
 * @param dmin   trial minimum distance
 * @param phase  phase 0..5
 * @return count of ucodes from the parameter block's unk360 up to listA[phase][0] whose distance exceeds the running half distance and which the
 *         detector allows (a byte counter)
 */
unsigned char V90ConstellationDesigner::constelBuild(short dmin, short phase)
{
  V90MappingParams* mp = mapParams_;
  V90Parameters* P = params_;
  uint8_t cnt = 0;
  int32_t half = (int32_t)dmin >> 1;
  uint32_t lo = (uint32_t)P->unk360;
  if ((uint32_t)LA(mp, (unsigned)(int)phase, 0) >= lo)
  { /* phase is a signed 16-bit value, used as an int row index */
    V90AutoDigitalImpDetector* ad = adid_;
    uint32_t idx = lo;
    unsigned ph = (unsigned)(int)phase;
    do
    {
      uint16_t e = (uint16_t)adLin(ad, ph * 128u + idx);
      if ((int16_t)e > (int16_t)half && adOk(ad, ph * 128u + idx) != 0)
      {
        cnt++;
        half = (int16_t)(uint16_t)((int32_t)dmin + (int32_t)e);
      }
      idx++;
    } while ((uint32_t)LA(mp, ph, 0) >= idx);
  }
  return cnt;
}

/**
 * @brief Count M that brings the constellation to a target K.
 * @param a  target exponent K
 * @param b  product of the six counts
 * @return trunc(2^n * 1.0069555^q) for t = (a - log10(b) / log10(2)) / 6 (see mFromT())
 */
unsigned V90ConstellationDesigner::calcMtoMatchKtarget(float a, float b)
{
  float lgf = lg10F((LD)b);
  float den = lg2F();
  LD t = (((LD)a) - (LD)lgf / (LD)den) * (LD)kSixth;
  return mFromT(t);
}

/** @brief Real-valued K of the mapping block: log10(prod) / log10(2) + 1e-9, 0 when the product of the counts is 0. */
DSP_EXTRET_FN float V90ConstellationDesigner::realK(V90MappingParams* mp)
{
  LD p = prodCnt(mp);
  if (p == 0.0L)
    return dsp_ret_ext_float(0.0L);
  float lgf = lg10F(p);
  float den = lg2F();
  return dsp_ret_ext_float((LD)lgf / (LD)den + (LD)kEps9);
}

/**
 * @brief Phase whose first A ucode is the largest.
 * @return phase 0..5; a tie goes to the phase with the larger count (the earlier phase on a further tie)
 */
int V90ConstellationDesigner::findConstelMaxValueIndex(V90MappingParams* mp)
{
  uint32_t cb = mp->cnt_[0];
  uint32_t best_v = LA(mp, 0, 0);
  int best = 0;
  for (unsigned ph = 1; ph <= 5; ph++)
  {
    uint32_t v = LA(mp, ph, 0);
    if (v > best_v)
    {
      cb = mp->cnt_[ph];
      best_v = v;
      best = (int)ph;
    }
    else if (v == best_v)
    {
      uint32_t c = mp->cnt_[ph];
      if (c > cb)
      {
        cb = c;
        best = (int)ph;
      }
    }
  }
  return best;
}

/**
 * @brief Phase whose first A ucode is the smallest.
 * @return phase 0..5; a tie goes to the phase with the larger count (the earlier phase on a further tie)
 */
int V90ConstellationDesigner::findMinValueIndex(V90MappingParams* mp)
{
  uint32_t cb = mp->cnt_[0];
  uint32_t best_v = LA(mp, 0, 0);
  int best = 0;
  for (unsigned ph = 1; ph <= 5; ph++)
  {
    uint32_t v = LA(mp, ph, 0);
    if (v < best_v)
    {
      cb = mp->cnt_[ph];
      best_v = v;
      best = (int)ph;
    }
    else if (v == best_v)
    {
      uint32_t c = mp->cnt_[ph];
      if (c > cb)
      {
        cb = c;
        best = (int)ph;
      }
    }
  }
  return best;
}

#include "core/dsp_intmath.h"

/**
 * @brief Find the next ucode that can be added at the start of a phase's list.
 * @param out    receives out[0] = the ucode and out[1] = its G.711 code (A-law with the 0xd5 toggle, or inverted mu-law, per useAlaw_)
 * @param phase  phase 0..5
 * @param A      A distance tables
 * @param B      B distance tables
 * @param dual   per-phase flag: the phase uses both tables (compare against A + dMinAlt_ and B + dMinAlt_)
 * @param valid  unused
 * @return 1 when the ucode is within range (signed byte >= 0), 0 when the scan ran off the end
 * @note  Starts at listA[phase][0] and advances until the table distance reaches the threshold (dMin_ for a single-table phase).  The G.711 code is
 *        computed before the range result is reported, so out[1] is also written on overflow.
 * @warning The single-table scan runs on a signed byte and may read up to index 128 of the phase row, one entry into the next row.
 */
int V90ConstellationDesigner::findNextUcodeToAdd(unsigned char* out, unsigned char phase, short (*A)[128], short (*B)[128], short* dual,
                                                 unsigned char (*valid)[128])
{
  (void)valid;
  const int16_t* a = (const int16_t*)(const void*)A;
  const int16_t* b = (const int16_t*)(const void*)B;
  V90MappingParams* mp = mapParams_;
  unsigned ph = phase;
  uint8_t u0 = LA(mp, ph, 0);
  uint8_t u = u0;
  unsigned base = ph * 128u;
  if (dual[ph] != 0)
  {
    if (u <= 0x71)
    {
      int thrA = (int)a[base + u0] + (int)(int16_t)dMinAlt_;
      for (;;)
      {
        int e = (int)a[base + u];
        if (!(e < thrA))
        {
          int thrB = (int)b[base + u0] + (int)(int16_t)dMinAlt_;
          if (e >= thrB)
            break;
        }
        u++;
        if (!(u <= 0x71))
          break;
      }
    }
  }
  else
  {
    if (!((int8_t)u < 0))
    {
      int thr = (int)a[base + u0] + (int)(int16_t)dMin_;
      for (;;)
      {
        int e = (int)a[base + u];
        if (e >= thr)
          break;
        u++;
        if ((int8_t)u < 0)
          break;
      }
    }
  }
  out[0] = u;
  int v = absI32((int)a[base + u]);
  if (useAlaw_ != 0)
    out[1] = (uint8_t)((uint8_t)linear2alaw(v) ^ 0xd5u);
  else
    out[1] = (uint8_t)~(uint8_t)linear2ulaw(v);
  return (int8_t)u >= 0 ? 1 : 0;
}

/**
 * @brief Drop leading list entries so that each phase's list starts at the given ucode.
 * @param mp      mapping block whose lists A / B and counts are edited
 * @param ucodes  six ucodes, one per phase
 * @warning The search for each ucode in list A is not bounded by the count; a ucode that is not in the list reads past the list.
 */
void V90ConstellationDesigner::reconstructInitialConditions(V90MappingParams* mp, unsigned char* ucodes)
{
  for (uint8_t phc = 0; phc <= 5; phc++)
  {
    unsigned ph = phc;
    uint8_t d = 0;
    uint8_t target = ucodes[ph];
    while (LA(mp, ph, d) != target)
      d++;
    if (d != 0)
    {
      uint8_t rem = d;
      do
      {
        uint32_t cnt = mp->cnt_[ph];
        uint8_t i = 0;
        if (0u < cnt)
        {
          do
          {
            unsigned ci = i;
            i++;
            LA(mp, ph, ci) = LA(mp, ph, ci + 1);
            LB(mp, ph, ci) = LB(mp, ph, ci + 1);
          } while ((uint32_t)i < cnt);
        }
        mp->cnt_[ph]--;
        rem--;
      } while (rem != 0);
    }
  }
}
