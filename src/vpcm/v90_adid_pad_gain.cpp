/**
 * @file v90_adid_pad_gain.cpp
 * @brief V90AutoDigitalImpDetector: alternate-phase variance threshold, alternate-phase mapping, maximum ucode and pad-gain / codec identification.
 *
 * The analysis steps that run after the TRN1 study has filled linearMapping_ and variance_ (see v90_adid_study.cpp):
 * - getAltVarThresh(): the variance threshold above which a phase counts as an alternate-RBS phase,
 * - updateAltRbsPhaseInDil(): rebuilds the mappings of the alternate-RBS phases from the stored samples, snapping them to the unsuspected phase,
 * - setQcLinearMapping(): quick-connect variant, taking the mappings from the previous session,
 * - determineMaxUcode(): the highest ucode whose variance is still small, overall and per phase, and the allowed-ucode flags,
 * - findPadGain(): which codec law (mu-law or A-law) and which digital pad gain best explain the linear mapping.
 * All methods belong to the class declared in v90_adid.h; the results are stored in its members.  Callers: the study (getAltVarThresh(),
 * updateAltRbsPhaseInDil()), the phase-3 demodulator decision code (setQcLinearMapping()), and V90Phase3Demodulator::setDigitalImairmentsInfo()
 * (determineMaxUcode(), findPadGain(), then applyPadGainToLinMapp()).
 *
 * Arithmetic: values stored in float members are computed in `long double` (x87 extended precision) and rounded to float on the store; the
 * floating-point comparisons are written with the x87 condition predicates of core/dsp_x87_cmp.h (jmpB, jmpAe, ... : a NaN takes "below" and not "above").
 * Debug prints format a float as `%c%d.%0Nd` (sign, integer part, N fraction digits) through splitValue() instead of the C library's float printing.
 *
 * Behaviours to be aware of:
 * - findPadGain(): the error E = g * g * sum is an unrounded extended product; with the debug level above 1 the "error = ..." print stores it to a float
 *   and reloads it, so E is rounded to float before the selection compares, at lower levels it is not.  The debug level can therefore change the
 *   chosen gain in rare ties.
 * - determineMaxUcode(): the integer part of the first varThresh print (before clamping) comes from the unrounded extended threshold, the
 *   other parts from the float copy.
 * - updateAltRbsPhaseInDil(): the debug level is cached in a local and re-read only after a print.
 */

#include <limits.h>
#include "v90_adid.h"
#include "core/dsp_math.h"
#include "core/dsp_host.h"
#include "v34/v34_api.h"
#include "pcm_leaf_classes.h"

extern "C"
{
}

typedef long double LD;

/* ------------------------------------------------------------------------------------------------ helpers */

#include "core/dsp_x87_cmp.h"

#include "core/dsp_intmath.h"

#include "core/dsp_x87_conv.h"

#include "vpcm_split.h"

/**
 * Split a value into the three arguments of a "%c%d.%0Nd" print.
 * @param vip    value whose integer part is printed (its absolute value, truncated)
 * @param vfrac  value that gives the sign and the fraction; may differ from `vip` by the rounding to float
 * @param scale  10^N, the number of printed fraction digits
 */
static inline Split splitValue2(LD vip, LD vfrac, LD scale)
{
  Split s;
  int t = dsp_trunc_i32(vfrac);               /* truncating conversion */
  s.ip = dsp_trunc_i32(__builtin_fabsl(vip)); /* |vip| truncated */
  LD frac = vfrac - (LD)t;
  s.fp = absI32(dsp_trunc_i32(frac * scale));
  s.sign = (vfrac <= 0.0L) ? 0x2d : 0x2b;
  return s;
}
static inline Split splitValue(LD v, LD scale) { return splitValue2(v, v, scale); }

/** Print "<fmt>" with the sign, integer part and fraction digits of `v` through edprintf(). */
static inline void edVal(const char* fmt, LD v, LD scale)
{
  Split s = splitValue(v, scale);
  edprintf(fmt, s.sign, s.ip, s.fp);
}

#include "core/dsp_debug.h"

#include "core/dsp_floatbits.h"

/** Variance and linear value of one bin (the body of updateLinMappMeanAndVar(), inlined here). */
static inline void updateBin(V90AutoDigitalImpDetector* o, int idx)
{
  if (o->cumCount_[idx] != 0)
  {
    LD r = 1.0L / (LD)o->cumCount_[idx];
    LD m = (LD)o->cumSum_[idx] * r;
    LD q = r * (LD)o->cumSumSq_[idx];
    LD mm = m * m;
    LD var = q - mm;
    o->variance_[idx] = (float)var;
    o->linearMapping_[idx] = trunc16(m + 0.5L);
  }
}

/* ------------------------------------------------------------------------------------------------ getAltVarThresh */

/**
 * @brief Variance threshold that separates alternate-RBS phases from the others.
 * @param var  the six per-phase variances at the reference ucode
 * @param cap  factor applied to the average (altVarFactor_ of the caller)
 * @return cap * (average of the variances that are below the mean of all six), raised to at least altVarThreshMax_ (despite its name it is a floor)
 * @note  Prints the mean, the factor and the initial threshold through edprintf().  When no variance is below the mean (all equal or NaN) the average
 *        divides by zero and the result is NaN or infinite.
 */
float V90AutoDigitalImpDetector::getAltVarThresh(float* var, float cap)
{
  LD sum = 0.0L;
  for (int16_t i = 0; i <= 5; i++)
    sum = sum + (LD)var[i];
  LD mean = sum * (LD)0.16666667f; /* 1/6 as a single-precision constant (0x3e2aaaab) */

  /* average of the variances below the mean: the phases without alternate signalling */
  LD acc = 0.0L;
  int16_t cnt = 0;
  for (int16_t j = 0; j <= 5; j++)
  {
    LD v = (LD)var[j];
    if (jmpB(v, mean))
    {
      acc = acc + v;
      cnt = (int16_t)(cnt + 1);
    }
  }
  LD avg = acc / (LD)cnt;
  LD t = (LD)cap * avg;
  edVal("V90AutoDigitalImpDetector: getAltVarThresh: meanVar = %c%d.%02d\r\n", avg, 100.0L);
  float thr = (float)t;
  edVal("V90AutoDigitalImpDetector: getAltVarThresh: factor = %c%d.%02d\r\n", (LD)cap, 100.0L);
  edVal("V90AutoDigitalImpDetector: getAltVarThresh: initial varThresh = %c%d.%02d\r\n", (LD)thr, 100.0L);

  if (!jmpBe((LD)altVarThreshMax_, (LD)thr))
    thr = altVarThreshMax_;
  return thr;
}

/* ------------------------------------------------------------------------------------------------ updateAltRbsPhaseInDil */

/** Order in which the ucodes of an alternate phase are visited: the order of the TRN1 training sequence (63, 61, ... 3, 2, 4, ... 64, 65 ... 116). */
static const uint8_t kUcodeOrder[115] = {63, 61, 59, 57, 55, 53, 51,  49,  47,  45,  43,  41,  39,  37,  35,  33,  31,  29,  27,  25,  23,  21,  19,
                                         17, 15, 13, 11, 9,  7,  5,   3,   2,   4,   6,   8,   10,  12,  14,  16,  18,  20,  22,  24,  26,  28,  30,
                                         32, 34, 36, 38, 40, 42, 44,  46,  48,  50,  52,  54,  56,  58,  60,  62,  64,  65,  66,  67,  68,  69,  70,
                                         71, 72, 73, 74, 75, 76, 77,  78,  79,  80,  81,  82,  83,  84,  85,  86,  87,  88,  89,  90,  91,  92,  93,
                                         94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116};

/**
 * @brief Rebuild linearMapping_ / linearMappingAlt_ of every alternate-RBS phase from the stored samples.
 *
 * The samples of an alternate phase were stored (addReceivedSampleToStorage()) in the order of the training sequence, which is kUcodeOrder, so the
 * storedCount_[phase][ucode] samples of one ucode are consecutive and cumOff walks through them.  For each ucode the stored samples are
 * 1. snapped to the nearest linear value of the unsuspected phase (ucodes 5..116),
 * 2. searched for their most frequent value that differs from the unsuspected phase's value of that ucode (duplicates are marked USHRT_MAX as consumed),
 * 3. and then the phase's normal mapping takes the unsuspected phase's value while the alternate mapping takes the most frequent differing value (or
 *    the unsuspected phase's value when there is none).
 * With the debug level above 1 the resulting tables are printed.
 *
 * @note  Reads unsuspectedPhase_ (0..5), which porcessSecondStudy() / setQcLinearMapping() set before the call.
 */
void V90AutoDigitalImpDetector::updateAltRbsPhaseInDil()
{
  unsigned lv = dsplibs_debug_level;
  if (lv > 1)
  {
    dsplibs_debug_printf("------------------------------------------------------------------\n");
    lv = dsplibs_debug_level;
  }

  for (uint32_t ph = 0; (uint16_t)ph <= 5; ph = (uint16_t)(ph + 1))
  {
    if (altRbsPhase_[ph] == 0)
      continue;

    const int phStore = (int)ph * 0x83e;
    const int phRow = (int)ph << 7;
    int cumOff = 0;
    for (uint32_t k = 0; (uint16_t)k <= 0x72; k = (uint16_t)(k + 1))
    {
      const int u = kUcodeOrder[k];
      int bestCnt = 0;
      int bestVal = 0; /* a sign-extended word */
      const int cnt = (int)(int16_t)storedCount_[phRow + u];

      if (cnt > 0)
      {
        /* snap every stored sample to the nearest linear value of the unsuspected phase (ucodes 5 .. 116) */
        const int ubase = (int)unsuspectedPhase_ << 7;
        uint32_t j = 0;
        do
        {
          int pos = phStore + (int)j + cumOff;
          int av = absI32((int)storage_[pos]);
          int best = 1000000;
          unsigned bestC = 5;
          for (uint32_t c = 5; (uint16_t)c <= 0x74; c = (uint16_t)(c + 1))
          {
            int d = absI32(av - (int)linearMapping_[ubase + (int)c]);
            if (d < best)
            {
              bestC = c;
              best = d;
            }
          }
          j = (uint16_t)(j + 1);
          storage_[pos] = linearMapping_[ubase + (int)bestC];
        } while ((int)j < cnt);
      }

      /* the most frequent value that differs from the reference (an unsuspected phase) and from the "consumed" marker USHRT_MAX */
      {
        const int cnt2 = (int)(int16_t)storedCount_[phRow + u];
        uint32_t m = 0;
        if ((int)m < cnt2 - 1)
        {
          const int refIdx = ((int)unsuspectedPhase_ << 7) + u;
          const uint16_t refWord = (uint16_t)linearMapping_[refIdx];
          do
          {
            const int posM = phStore + (int)m + cumOff;
            const uint16_t valM = (uint16_t)storage_[posM];
            uint32_t next = (uint16_t)(m + 1);
            if (valM != refWord && valM != USHRT_MAX)
            {
              /* count the later equal samples and mark them consumed */
              uint32_t rc = 1;
              uint32_t q = next;
              if ((int)q < cnt2)
              {
                do
                {
                  int posQ = phStore + (int)q + cumOff;
                  uint16_t w = (uint16_t)storage_[posQ];
                  if (w == (uint16_t)storage_[posM] && w != (uint16_t)linearMapping_[refIdx] && w != USHRT_MAX)
                  {
                    storage_[posQ] = (int16_t)0xffff;
                    rc = (uint16_t)(rc + 1);
                  }
                  q = (uint16_t)(q + 1);
                } while ((int)q < cnt2);
              }
              if ((uint16_t)rc > (uint16_t)bestCnt)
              {
                bestCnt = (int)rc;
                bestVal = (int)storage_[posM];
              }
            }
            m = next;
          } while ((int)m < cnt2 - 1);
        }
      }

      /* the normal mapping takes the unsuspected phase's value, the alternate mapping the most frequent differing value */
      if (storedCount_[phRow + u] != 0)
      {
        uint16_t refLin = (uint16_t)linearMapping_[((int)unsuspectedPhase_ << 7) + u];
        linearMapping_[phRow + u] = (int16_t)refLin;
        if (bestCnt == 0)
          linearMappingAlt_[phRow + u] = (int16_t)refLin;
        else
          linearMappingAlt_[phRow + u] = (int16_t)bestVal;
        cumOff += (int)(int16_t)storedCount_[phRow + u];
      }
    }

    /* debug report of both tables of this phase */
    if (lv > 1)
    {
      dsplibs_debug_printf("\n----  AltPhase %d : linearMapping  linearMappingAlt  ----\n", (int)ph);
      lv = dsplibs_debug_level;
    }
    for (uint32_t u2 = 0; (uint16_t)u2 <= 0x7f; u2 = (uint16_t)(u2 + 1))
    {
      if (lv > 1)
      {
        int idx = phRow + (int)u2;
        dsplibs_debug_printf("AltUcode[%d]  :  %d  %d\n", (int)u2, (int)linearMapping_[idx], (int)linearMappingAlt_[idx]);
        lv = dsplibs_debug_level;
      }
    }
  }

  if (lv > 1)
    dsplibs_debug_printf("------------------------------------------------------------------\n");
}

/* ------------------------------------------------------------------------------------------------ setQcLinearMapping */

/**
 * @brief Quick-connect: set the linear mappings from the previous session and the accumulated sums.
 *
 * For each phase that is not an alternate-RBS phase: an unsuspected phase takes prevSessionLinearMapping_, a suspected phase is converted from its
 * accumulated bins.  Then unsuspectedPhase_ is chosen (the first phase not suspected of RBS), updateAltRbsPhaseInDil() fills the alternate phases and,
 * with the debug level above 1, the six-phase table is printed.
 *
 * @note  When every phase 1..5 is suspected and phase 0 is as well, the search stops at phase 5 and unsuspectedPhase_ is 5 even though that phase is
 *        suspected (see porcessSecondStudy(), which has the same search).
 */
void V90AutoDigitalImpDetector::setQcLinearMapping()
{
  for (int16_t i = 0; i <= 5; i++)
  {
    if (altRbsPhase_[i] != 0)
      continue;

    const int row = (int)i << 7;
    if (suspectedPhase_[i] == 0)
    {
      for (int16_t u = 0; u <= 0x7f; u++)
        linearMapping_[row + u] = prevSessionLinearMapping_[u];
    }
    else
    {
      for (int16_t u = 0; u <= 0x7f; u++)
        updateBin(this, row + u);
    }
  }

  /* first phase that is not suspected of RBS (5 when there is none) */
  unsuspectedPhase_ = 0;
  if (suspectedPhase_[0] != 0)
  {
    for (;;)
    {
      uint16_t p = (uint16_t)(unsuspectedPhase_ + 1);
      bool found = suspectedPhase_[(int)(int16_t)p] == 0;
      unsuspectedPhase_ = (int16_t)p;
      if (found)
        break;
      if (!((int16_t)p <= 4))
        break;
    }
  }

  updateAltRbsPhaseInDil();

  /* debug report of the six mappings, ucodes 0 .. 116 */
  unsigned lv = dsplibs_debug_level;
  if (lv > 1)
  {
    dsplibs_debug_printf("------------------------------------------------------\r\n");
    lv = dsplibs_debug_level;
    if (lv > 1)
    {
      dsplibs_debug_printf("-----------------  Linear Mapping report --------------\r\n");
      lv = dsplibs_debug_level;
    }
  }
  for (int16_t u = 0; u <= 0x74; u++)
  {
    if (lv > 1)
    {
      dsplibs_debug_printf("linearMapping[%d]  :  %d  %d  %d  %d  %d  %d\r\n", (int)u, (int)linearMapping_[u], (int)linearMapping_[0x80 + u],
                           (int)linearMapping_[0x100 + u], (int)linearMapping_[0x180 + u], (int)linearMapping_[0x200 + u], (int)linearMapping_[0x280 + u]);
      lv = dsplibs_debug_level;
    }
  }
  if (lv > 1)
    dsplibs_debug_printf("------------------------------------------------------\r\n");
}

/* ------------------------------------------------------------------------------------------------ determineMaxUcode */

/**
 * @brief Determine the highest usable ucode, per-phase maxima and the allowed-ucode flags from the variance table.
 * @param arg  highest ucode to consider (the impairment info of the phase-3 demodulator)
 *
 * 1. Threshold vt = varThreshFactor_ * mean |variance| of ucodes 40..59 of the unsuspected phase, clamped to 500 .. 100000.
 * 2. maxUcode_ = the highest ucode u (arg down to minMaxUcode_ + 1) for which at least 3 of the variances at u, u-1, ... u-4 are non-zero and below vt;
 *    if none, or if the result is below minMaxUcode_, minMaxUcode_.
 * 3. ucodeAllowed_ is cleared for every ucode of a non-alternate phase that is above `arg` or has |variance| >= 2 * vt; the reference ucode is always allowed.
 * 4. maxUcodeArray_[phase]: an alternate phase gets min(maxUcode_, arg - 2); another phase gets the highest allowed ucode not above maxUcode_.
 * Every step prints through edprintf().
 *
 * @warning  Uses unsuspectedPhase_ as a row index without a range check; the ucode arithmetic is 8-bit, so `arg` must be below 256 and above
 *           minMaxUcode_.  The downward search for an allowed ucode (step 4) has no lower bound other than the reference ucode, which step 3 keeps allowed.
 */
void V90AutoDigitalImpDetector::determineMaxUcode(short arg)
{
  const int a = (int)arg;
  const int us = (int)unsuspectedPhase_;

  /* variance threshold from the middle ucodes of the unsuspected phase */
  LD acc = 0.0L;
  for (int k = 0; k < 20; k++)
  {
    float t = (float)__builtin_fabsl((LD)variance_[(us << 7) + 40 + k]);
    acc = acc + (LD)t;
  }
  LD t1 = acc * (LD)varThreshFactor_;
  LD t2 = t1 * (LD)0.05f;
  float vt = (float)t2;
  {
    Split s = splitValue2(t2, (LD)vt, 100.0L); /* the integer part from the unrounded value */
    edprintf("V90AutoDigitalImpDetector: original varThresh = %c%d.%02d\r\n", s.sign, s.ip, s.fp);
  }
  if (!jmpAe((LD)vt, (LD)500.0f))
    vt = 500.0f;
  else if (jmpA((LD)vt, (LD)100000.0f))
    vt = 100000.0f;
  {
    Split s = splitValue((LD)vt, 100.0L);
    s.sign = jmpA((LD)vt, 0.0L) ? 0x2b : 0x2d; /* only a positive threshold prints '+' */
    edprintf("V90AutoDigitalImpDetector: final varThresh = %c%d.%02d\r\n", s.sign, s.ip, s.fp);
  }

  /* variance report from minMaxUcode_ + 1 up to arg */
  edprintf("--------------------------------------------------------\r\n");
  edprintf("V90AutoDigitalImpDetector: linearMappingVar report:\r\n", us, (int)minMaxUcode_ + 1, a);
  {
    uint8_t bl = (uint8_t)((uint8_t)minMaxUcode_ + 1);
    while ((int)bl <= a)
    {
      int iv = absI32(dsp_trunc_i32((LD)variance_[(us << 7) + bl]));
      edprintf("linearMappingVar[%d][%d] = %d\r\n", us, (int)bl, iv);
      bl = (uint8_t)(bl + 1);
    }
  }
  edprintf("--------------------------------------------------------\r\n");

  /* highest ucode with at least three of its five lower neighbours (itself included) inside 0 < variance < vt */
  const uint8_t min8 = (uint8_t)(uint16_t)minMaxUcode_;
  if ((uint8_t)a > min8)
  {
    uint8_t u = (uint8_t)a;
    bool found = false;
    for (;;)
    {
      uint8_t first = 0;
      uint8_t cnt = 0;
      for (uint8_t k = 0; k <= 4; k++)
      {
        float v = variance_[(us << 7) + (int)u - (int)k];
        if (jmpB((LD)v, (LD)vt))
        {
          bool zf = ((LD)v == 0.0L) || ((LD)v != (LD)v); /* zero or NaN does not count */
          if (!zf)
          {
            if (cnt == 0)
              first = k;
            cnt = (uint8_t)(cnt + 1);
          }
        }
      }
      if (cnt > 2)
      {
        maxUcode_ = (uint8_t)(u - first);
        found = true;
        break;
      }
      u = (uint8_t)(u - 1);
      if (!(u > min8))
        break;
    }
    if (!found)
      maxUcode_ = min8;
  }
  else
  {
    maxUcode_ = min8;
  }

  {
    int mu = (int)maxUcode_;
    int minS = (int)minMaxUcode_;
    if (mu < minS)
    {
      edprintf("V90AutoDigitalImpDetector: original maxUcode = %d  forced minimum maxUcode = %d\n", mu, minS);
      maxUcode_ = (uint8_t)(uint16_t)minMaxUcode_;
    }
  }

  /* allowed ucodes: not above arg, variance below 2 * vt; the reference ucode is always allowed */
  uint8_t cur = maxUcode_;
  const LD twoVt = (LD)vt + (LD)vt;
  const unsigned ref = refUcode_;
  for (uint8_t p = 0; p <= 5; p++)
  {
    const int row = (int)p << 7;
    if (altRbsPhase_[p] == 0)
    {
      for (uint8_t u = 0; u <= 0x7f; u++)
      {
        if ((int)u > a || jmpAe(__builtin_fabsl((LD)variance_[row + u]), twoVt))
          ucodeAllowed_[row + u] = 0;
        else
          ucodeAllowed_[row + u] = 1;
      }
    }
    ucodeAllowed_[row + ref] = 1;
  }

  /* per-phase maximum ucode */
  for (uint8_t p = 0;;)
  {
    if (altRbsPhase_[p] != 0)
    {
      if ((int)cur <= a - 2)
        maxUcodeArray_[p] = cur;
      else
        maxUcodeArray_[p] = (uint8_t)(a - 2);
    }
    else
    {
      maxUcodeArray_[p] = cur;
      if (ucodeAllowed_[((int)p << 7) + cur] == 0)
      {
        uint8_t d = cur;
        do
        {
          d = (uint8_t)(d - 1);
        } while (ucodeAllowed_[((int)p << 7) + d] == 0);
        maxUcodeArray_[p] = d;
      }
    }
    edprintf("V90AutoDigitalImpDetector: phase %d maxUcode is %d\r\n", (int)p, (int)maxUcodeArray_[p]);
    p = (uint8_t)(p + 1);
    if (p > 5)
      return;
    cur = maxUcode_;
  }
}

/* ------------------------------------------------------------------------------------------------ findPadGain */

/**
 * @brief Identify the codec law of the far end and the digital pad gain.
 *
 * The linear mapping of the unsuspected phase is compared with the levels a pad of gain g in front of a mu-law or an A-law codec would produce.  For
 * each law c (0 = mu-law, 1 = A-law):
 * - the projection base ucode is the one with the smallest variance among the five ucodes maxUcode_ - 3 and below, limited to 0x50..0x5f;
 * - every candidate ucode u from the lowest considered ucode (minUcodeScale_ times the base's level, at least 0x28) up to the base gives a candidate gain
 *   g = linearMapping(base) / level(u); its error E is g^2 times the sum over ucodes 0x40..maxUcode_ of the squared quantisation error that remains
 *   when linearMapping / g is rounded and re-encoded in law c;
 * - candidates are kept in three classes (g >= 2.7; A-law only: g < 1.2, meaning "no pad"; the rest) and the class winners are combined: the
 *   high-gain class wins when 0.8 times the middle class's error is above its own; for A-law the "no pad" class (gain 1.0) wins when 0.85 times its error
 *   is below the chosen error.
 * The law with the smaller error wins.  codecIsAlaw_ and padGain_ are set; everything is printed through edprintf() / dsplibs_debug_printf().
 *
 * @note  Needs maxUcode_ (determineMaxUcode()) and unsuspectedPhase_.  If no variance in the base search is below 1e8 the base is ucode 0 clamped to 0x50.
 * @warning  The error E is rounded to float when the debug level is above 1 and not otherwise (see the file header), so the debug level can change the
 *           selection in rare ties.
 */
void V90AutoDigitalImpDetector::findPadGain()
{
  if (dbgOn())
  {
    dsplibs_debug_printf("-------------------------------------------------------------------------\r\n");
    if (dbgOn())
    {
      dsplibs_debug_printf("V90AutoDigitalImpDetector::findPadGain() Report :\r\n");
      if (dbgOn())
        dsplibs_debug_printf("-------------------------------------------------------------------------\r\n");
    }
  }

  /* the projection base ucode: the smallest variance among maxUcode - 3 and the next four lower ucodes (8-bit index) */
  const int ubase = (int)unsuspectedPhase_ << 7;
  LD best = (LD)100000000.0f;
  uint8_t dl = (uint8_t)(maxUcode_ - 3u);
  const int lowBound = (int)dl - 5;
  uint8_t argmin = 0; /* stays 0 when no variance is below 1e8 */
  for (;;)
  {
    LD v = (LD)variance_[ubase + dl];
    if (jmpB(v, best))
    {
      best = v;
      argmin = dl;
    }
    dl = (uint8_t)(dl - 1);
    if (!((int)dl > lowBound))
      break;
  }
  uint8_t base = argmin;
  if (base > 0x5f)
    base = 0x5f;
  else if (base <= 0x4f)
    base = 0x50;

  float gainMu = 0.0f, errMu = 0.0f;
  float gain90 = 0.0f, err98 = 0.0f; /* the A-law result, also the working result of the law being evaluated */
  for (uint8_t c = 0; c <= 1; c++)
  {
    /* lowest candidate ucode: the base's level scaled by minUcodeScale_, re-encoded in law c, at least 0x28 */
    const unsigned codeA = (unsigned)((base & 0x7fu) ^ 0xd5u);
    const unsigned codeMu = (unsigned)(~(unsigned)(base & 0x7fu)) & 0xffu;
    uint8_t minU;
    {
      float scale = minUcodeScale_;
      if (c != 0)
      {
        int lin = alaw2linear((int)codeA);
        int t = absI32(dsp_trunc_i32((LD)lin * (LD)scale));
        minU = (uint8_t)((uint8_t)linear2alaw(t) ^ 0xd5u);
      }
      else
      {
        int lin = ulaw2linear((int)codeMu);
        int t = absI32(dsp_trunc_i32((LD)lin * (LD)scale));
        minU = (uint8_t)~(uint8_t)linear2ulaw(t);
      }
      if (minU <= 0x27)
        minU = 0x28;
    }

    /* class winners: err80 = A-law "no pad" class (g < 1.2), errMid88 / gainMid84 = middle class, errB8c / candB = g >= 2.7 class (bits of the gain) */
    float err80 = 1000000.0f, gainMid84 = 1.0f, errMid88 = 1000000.0f, errB8c = 1000000.0f;
    uint32_t candB = 0x3f800000u;
    if (dbgOn())
    {
      dsplibs_debug_printf("Codec type %d\r\n", (int)c);
      if (dbgOn())
        dsplibs_debug_printf("projectionBaseUcode = %d minUcodeForCodecProjection = %d\r\n", (int)base, (int)minU);
    }

    for (uint8_t u = minU; u <= base; u++)
    {
      /* candidate gain: the base's linear value over the level of ucode u in law c */
      float s0 = (float)(int)linearMapping_[ubase + base];
      float errSum = 0.0f;
      int l = c ? alaw2linear((int)((u & 0x7fu) ^ 0xd5u)) : ulaw2linear((int)((u & 0x7fu) ^ 0xffu));
      float g = (float)((LD)s0 / (LD)l);

      /* quantisation error that remains when the mapping of ucodes 0x40 .. maxUcode is divided by g and re-encoded */
      if (maxUcode_ > 0x3f)
      {
        uint8_t k = 0x40;
        do
        {
          LD r = 1.0L / (LD)g;
          LD x = (LD)(int)linearMapping_[ubase + k] * r + (LD)0.5f;
          int v16 = (int)trunc16(x);
          int back;
          if (c)
            back = (int)(int16_t)alaw2linear((int)(uint8_t)linear2alaw(v16));
          else
            back = (int)(int16_t)ulaw2linear((int)(uint8_t)linear2ulaw(v16));
          int diff = (int)((unsigned)v16 - (unsigned)back);
          k = (uint8_t)(k + 1);
          LD d2 = (LD)diff * (LD)diff;
          errSum = (float)(d2 + (LD)errSum);
        } while (maxUcode_ >= k);
      }
      LD E = (LD)g * (LD)g * (LD)errSum; /* an unrounded extended product */
      if (dbgOn())
      {
        Split s = splitValue((LD)g, (LD)100.0f);
        dsplibs_debug_printf("error = %d, gain = %c%d.%02d\r\n", dsp_trunc_i32(E), s.sign, s.ip, s.fp);
        E = (LD)(float)E; /* the print stores E to a float and reloads it: E is rounded only at debug levels above 1 */
      }

      /* file the candidate in its class */
      if (!jmpB((LD)g, (LD)2.7f))
      {
        if (jmpB(E, (LD)errB8c))
        {
          errB8c = (float)E;
          candB = floatBits(g);
        }
      }
      else
      {
        bool toMid = true;
        if (c == 1)
        {
          if (jmpB((LD)g, (LD)1.2f))
          {
            if (jmpB(E, (LD)err80))
              err80 = (float)E;
            toMid = false;
          }
        }
        if (toMid && jmpB(E, (LD)errMid88))
        {
          errMid88 = (float)E;
          gainMid84 = g;
        }
      }
    }

    /* combine the class winners */
    gain90 = gainMid84;
    err98 = errMid88;
    LD t08 = (LD)errMid88 * (LD)0.8f;
    if (jmpA(t08, (LD)errB8c))
    {
      gain90 = bitsFloat(candB);
      err98 = errB8c;
    }
    if (c != 0)
    {
      LD t085 = (LD)err80 * (LD)0.85f;
      if (jmpB(t085, (LD)err98))
      {
        gain90 = 1.0f;
        err98 = err80;
      }
    }
    else
    {
      gainMu = gain90;
      errMu = err98;
    }
  }

  /* the law with the smaller error wins */
  if (dbgOn())
    dsplibs_debug_printf("------ Final conclusions -------\r\n");
  edVal("minErrorMuLaw = %c%d.%02d\r\n", (LD)errMu, 100.0L);
  edVal("gainValueMuLaw = %c%d.%06d\r\n", (LD)gainMu, 1000000.0L);
  edVal("minErrorALaw = %c%d.%02d\r\n", (LD)err98, 100.0L);
  edVal("gainValueALaw = %c%d.%06d\r\n", (LD)gain90, 1000000.0L);

  float chosen;
  if (jmpA((LD)errMu, (LD)err98))
  {
    codecIsAlaw_ = 1;
    edprintf("Final codec identified is ALaw\r\n");
    chosen = gain90;
  }
  else
  {
    chosen = gainMu;
    codecIsAlaw_ = 0;
    edprintf("Final codec identified is MuLaw\r\n");
  }
  padGain_ = chosen;
  edVal("Final padGain identified is = %c%d.%06d\r\n", (LD)chosen, (LD)1000000.0f);
  if (dbgOn())
    dsplibs_debug_printf("-------------------------------------------------------------------------\r\n");
}
