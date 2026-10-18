/**
 * @file v90_adid_study.cpp
 * @brief V90AutoDigitalImpDetector: the TRN1 study state machine.
 *
 * The study ("uref" = the reference ucode refUcode_, the one linear-mapping entry every phase is compared on) runs while the V.90 TRN1 training
 * sequence is received.  studyUrefHandler() is called once per received symbol with the sample and its PCM phase 0..5 and walks a state machine
 * (studyState_; studyCounter_ counts the symbols of the state, studyLen0_ .. studyLen4_ say how long each state lasts):
 *   0  accumulate |sample|, sample^2 into the (phase, ucode of refLinear_) bin; at the end: variance per phase, the alternate-RBS variance threshold
 *      (getAltVarThresh()); phases whose variance exceeds it become alternate-RBS phases (altRbsPhase_) and the bins are cleared; -> 1
 *   1  accumulate; at the end: mean / variance / linear value per phase, unitePhasesInfoOfUref(), bins cleared, "first update" printed; -> 2
 *   2  like 1, but a sample of an alternate phase that is far from the phase's linear value goes into altSum_ / altCount_ instead (isAltRbs() inlined);
 *      at the end the alternate linear mapping is built and a second pass over the alternate flags removes false detections; studyDone_ = 1; -> 4
 *   4  waits studyLen4_ symbols; -> 3
 *   3  like 2 (including the flag pass), then trn1Sigma_ = mean variance of the non-alternate phases; -> 5
 *   5  like 3 without the flag pass, trn1Sigma_ again; -> 6 and the answer 2 (study finished)
 *   6  answers 2 forever.
 * The answer of studyUrefHandler() is 1 (carry on), 0 (the sample went into the alternate statistics, states 1 / 2) or 2 (study finished).
 * The end-of-study steps porcessFirstStudy() (marks the suspected phases) and porcessSecondStudy() (snaps suspected phases onto an unsuspected one, then
 * updateAltRbsPhaseInDil()) are called by the phase-3 demodulator's decision code; resetStudyUrefHandler() loads the study lengths and thresholds and
 * restarts the machine.
 *
 * The debug level (dsplibs_debug_level above 1) is read once per block and re-read after each print, because a print may change it.
 *
 * Behaviours to be aware of:
 * - the two "trn1Sigma = %d" prints pass a DOUBLE where the format has %d (the extended mean variance rounded to double);
 * - state 2 and state 3 end with the same alternate-flag pass, but state 2 re-reads the debug level at every "false detection" print, state 3 uses the
 *   cached one;
 * - porcessSecondStudy(): the best / second best distances live across ALL iterations: the second best of the previous iteration starts the next one
 *   (NaN at the first), the best starts at 32256.0 every time, and the chosen index survives from earlier iterations when no candidate beats 32256;
 * - resetStudyUrefHandler(0) copies the defaults, resetStudyUrefHandler(!= 0) derives the thresholds from studyParam_ and prints a QC report with edprintf().
 */

#include "v90_adid.h"
#include "core/dsp_math.h"
#include "core/dsp_asm.h"
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

#include "core/dsp_debug.h"

#include "core/dsp_floatbits.h"

#include "vpcm_split.h"
#include "vpcm_split_value.h"

/** Mean, variance and linear value of one bin (the body of updateLinMappMeanAndVar(), inlined all over the state machine). */
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

/** The ucode bin of the reference linear value: every sample of the study goes into the bin of the reference's own ucode. */
static inline int refBin(const V90AutoDigitalImpDetector* o)
{
  int lin = (int)o->refLinear_;
  if (o->pcmType_ != 0)
    return (int)(uint8_t)((uint8_t)linear2alaw(absI32(lin)) ^ 0xd5u);
  return (int)(int16_t)(0xff - (unsigned)(uint8_t)linear2ulaw(absI32(lin)));
}

/** Add the sum, sum of squares and count of one received sample to the reference bin of `phase`. */
static inline void accumulate(V90AutoDigitalImpDetector* o, unsigned phase, int sample)
{
  int idx = (int)((phase << 7) + (unsigned)refBin(o));
  int a = absI32(sample);
  int sq = a * a;
  o->cumSum_[idx] = (float)((LD)a + (LD)o->cumSum_[idx]);
  LD s2 = (LD)sq + (LD)o->cumSumSq_[idx];
  o->cumCount_[idx]++;
  o->cumSumSq_[idx] = (float)s2;
}

/** The alternate-sample test (isAltRbs() inlined): the phase is an alternate one and the sample is far from its linear value at the reference ucode. */
static inline bool altSample(const V90AutoDigitalImpDetector* o, unsigned phase, LD sample)
{
  int ph = (int)(int16_t)phase;
  if (o->altRbsPhase_[ph] == 0)
    return false;
  int t = absI32(dsp_trunc_i32(sample));
  int lin = (int)o->linearMapping_[(ph << 7) + (int)o->refUcode_];
  return absI32(t - lin) > (int)o->altRbsThresh_;
}

/** Add |sample| to the alternate sum and count of `phase`. */
static inline void accumulateAlt(V90AutoDigitalImpDetector* o, unsigned phase, int sample)
{
  int a = absI32(sample);
  LD s = (LD)a + (LD)o->altSum_[phase];
  o->altCount_[phase]++;
  o->altSum_[phase] = (float)s;
}

/* End-of-state steps shared by the states 1, 2, 3 and 5. */

/** Convert the reference bins of the six phases to variance and linear value. */
static inline void binsToMapping(V90AutoDigitalImpDetector* o)
{
  int idx = (int)o->refUcode_;
  for (int16_t k = 0; k <= 5; k++, idx += 128)
    updateBin(o, idx);
}

/** Empty the reference bins of the six phases. */
static inline void clearRefBins(V90AutoDigitalImpDetector* o)
{
  int idx = (int)o->refUcode_;
  for (int16_t k = 0; k <= 5; k++, idx += 128)
  {
    o->cumSum_[idx] = 0.0f;
    o->cumCount_[idx] = 0;
    o->cumSumSq_[idx] = 0.0f;
  }
}

/** Alternate linear value of the reference ucode of every alternate phase from its alternate sum / count; empties the alternate statistics. */
static inline void altToMapping(V90AutoDigitalImpDetector* o)
{
  int idx = (int)o->refUcode_;
  for (int16_t k = 0; k <= 5; k++, idx += 128)
  {
    if (o->altRbsPhase_[k] != 0 && o->altCount_[k] != 0)
    {
      LD r = 1.0L / (LD)o->altCount_[k];
      LD v = r * (LD)o->altSum_[k];
      o->linearMappingAlt_[idx] = trunc16(v + 0.5L);
    }
    o->altSum_[k] = 0.0f;
    o->altCount_[k] = 0;
  }
}

/** Copy the six linear values of the reference ucode (rows 0..5 of linearMapping_ or linearMappingAlt_) for the debug prints. */
static inline void sixValues(const int16_t* base, int ref, int out[6])
{
  for (int k = 0; k < 6; k++)
    out[k] = (int)base[ref + 128 * k];
}

/* ------------------------------------------------------------------------------------------------ resetStudyUrefHandler */

/**
 * @brief Load the study lengths and thresholds and restart the state machine.
 * @param lineMode  0 = defaults: study lengths adidStudyLen0..5 of the parameter block, fixed thresholds (united distance 25, alternate distance 50,
 *                  squared-difference limits 2500 / 6000, alternate variance floor 4000);
 *                  non-zero (retrain) = the lengths adidStudyLenAlt0..5 and thresholds divided by studyParam_, the reference ucode of every phase seeded
 *                  with refLinear_ / studyParam_, and a QC report printed through edprintf()
 * @warning  For lineMode != 0 the object's studyParam_ is divided into the thresholds, but the library never assigns studyParam_ (the constructor leaves
 *           it uninitialised): the value is whatever the heap block held.  The branch is reachable (V90Demodulator passes retrainOk_ as the line mode).
 */
void V90AutoDigitalImpDetector::resetStudyUrefHandler(unsigned lineMode)
{
  if (lineMode == 0)
  {
    V90Parameters* p = params_;
    studyLen0_ = (uint32_t)p->adidStudyLen0;
    studyLen1_ = (uint32_t)p->adidStudyLen1;
    studyLen2_ = (uint32_t)p->adidStudyLen2;
    studyLen3_ = (uint32_t)p->adidStudyLen3;
    studyLen4_ = (uint32_t)p->adidStudyLen4;
    studyLen5_ = (uint32_t)p->adidStudyLen5;
    sqrDiffThreshUref_ = 0x19;
    altRbsThresh_ = 0x32;
    sqrDiffThreshMax_ = 0x1770;
    altVarThreshMax_ = 4000.0f;
    sqrDiffThreshMin_ = 0x9c4;
    studyState_ = 0;
    studyCounter_ = 0;
    return;
  }

  /* line mode != 0: lengths of the alternate set, thresholds scaled by 1 / studyParam_ */
  const LD sp = (LD)studyParam_;
  V90Parameters* p = params_;
  LD r = 1.0L / sp;
  studyLen0_ = (uint32_t)p->adidStudyLenAlt0;
  studyLen1_ = (uint32_t)p->adidStudyLenAlt1;
  studyLen2_ = (uint32_t)p->adidStudyLenAlt2;
  studyLen3_ = (uint32_t)p->adidStudyLenAlt3;
  studyLen4_ = (uint32_t)p->adidStudyLenAlt4;
  studyLen5_ = (uint32_t)p->adidStudyLenAlt5;
  sqrDiffThreshUref_ = trunc16(r * (LD)25.0f + (LD)0.5f);
  LD x = r * (LD)4444.4443359375f;
  if (jmpA(x, (LD)4000.0f))
    altVarThreshMax_ = 4000.0f;
  else
    altVarThreshMax_ = (float)x;
  LD r2 = 1.0L / sp;
  altRbsThresh_ = trunc16(r2 * (LD)50.0f + (LD)0.5f);
  int16_t v = trunc16(r2 * (LD)2777.77783203125f + (LD)0.5f);
  if (v > 0x9c4)
    v = 0x9c4;
  sqrDiffThreshMin_ = v;
  int16_t v2 = trunc16(r2 * (LD)6666.6669921875f + (LD)0.5f);
  if (v2 > 0x1770)
    v2 = 0x1770;
  sqrDiffThreshMax_ = v2;

  /* seed the reference ucode of every phase with the scaled reference value (skipped for studyParam_ == 0 or NaN; any other value, also negative, goes on) */
  if (!(sp == 0.0L) && (sp == sp))
  {
    LD t = r2 * (LD)(int)refLinear_ + (LD)0.5f;
    int16_t cx = trunc16(t);
    int idx = (int)refUcode_;
    for (int16_t i = 0; i <= 5; i++, idx += 128)
    {
      linearMapping_[idx] = cx;
      linearMappingAlt_[idx] = cx;
    }
  }

  /* QC report */
  edprintf("---------------------------------------------------------------\r\n");
  edprintf("V90AutoDigitalImpDetector::resetStudyUrefHandler QC report :\r\n");
  {
    int ref = (int)refUcode_;
    edprintf("prevSession uinfo : %d %d %d %d %d %d\r\n", (int)linearMapping_[ref], (int)linearMapping_[ref + 128], (int)linearMapping_[ref + 256],
             (int)linearMapping_[ref + 384], (int)linearMapping_[ref + 512], (int)linearMapping_[ref + 640]);
  }
  edprintf("uniteUrefDistanceThresh = %d\r\n", (int)sqrDiffThreshUref_);
  edprintf("altRbsDistanceThresh = %d\r\n", (int)altRbsThresh_);
  edprintf("neighborUcodeMinDistance=%d neighborUcodeMaxDistance=%d\r\n", (int)sqrDiffThreshMin_, (int)sqrDiffThreshMax_);
  {
    Split s = splitValue((LD)altVarThreshMax_, (LD)100.0f);
    edprintf("altMinVarThresh = %c%d.%02d\r\n", s.sign, s.ip, s.fp);
  }
  edprintf("---------------------------------------------------------------\r\n");

  studyState_ = 0;
  studyCounter_ = 0;
}

/* ------------------------------------------------------------------------------------------------ porcessFirstStudy */

/**
 * @brief First end-of-study step: mark the phases that are suspected of RBS and clear the statistics.
 *
 * The squared-difference threshold is 2.5 * trn1Sigma_ limited to sqrDiffThreshMin_ .. sqrDiffThreshMax_.  For ucodes 0x40..0x4f (except the reference)
 * the bins are converted to linear values; a phase that is not an alternate-RBS phase is suspected when at most 9 of the 15 neighbouring-ucode steps
 * (counted from 1) have a squared difference above the threshold (RBS merges neighbouring levels); an alternate phase is always suspected.  Afterwards
 * all bins of all phases are emptied and linearMapping_ is zeroed everywhere except at the reference ucode.
 *
 * @note  Sets suspectedPhase_[] and nofSuspectedPhases_; prints the pattern.  The reference ucode is re-read after the print (a print may change state).
 */
void V90AutoDigitalImpDetector::porcessFirstStudy()
{
  /* threshold from the study's variance */
  const LD ts = (LD)trn1Sigma_;
  LD c = ts * (LD)2.5f;
  LD x1 = jmpAe(c, (LD)(int)sqrDiffThreshMin_) ? c : (LD)(int)sqrDiffThreshMin_;
  LD x2 = jmpAe((LD)(int)sqrDiffThreshMax_, x1) ? x1 : (LD)(int)sqrDiffThreshMax_;
  const float thrF = (float)x2;
  int16_t sqrDiffThresh = trunc16(x2 + 0.5L);
  int16_t trn1SigmaX25 = trunc16(ts * (LD)2.5f + 0.5L);
  edprintf("V90AutoDigitalImpDetector::porcessFirstStudy()  2.5*trn1Sigma=%d , sqrDiffThresh=%d\n", (int)trn1SigmaX25, (int)sqrDiffThresh);

  /* linear values of the ucodes 0x40..0x4f of every phase */
  const uint8_t ref0 = refUcode_;
  for (uint8_t p = 0; p <= 5; p++)
  {
    int idx = ((int)p << 7) + 0x40;
    for (uint8_t u = 0x40; u <= 0x4f; u++, idx++)
    {
      if (ref0 != u)
        updateBin(this, idx);
    }
  }

  /* suspected phases: too few well separated neighbouring levels */
  nofSuspectedPhases_ = 0;
  for (uint8_t p = 0; p <= 5; p++)
  {
    bool suspected = true;
    if (altRbsPhase_[p] == 0)
    {
      int idx = ((int)p << 7) + 0x40;
      int16_t cnt = 1;
      for (int t = 0; t < 15; t++, idx++)
      {
        int d = (int)linearMapping_[idx + 1] - (int)linearMapping_[idx];
        LD d2 = (LD)d * (LD)d;
        if (jmpA(d2, (LD)thrF))
          cnt = (int16_t)(cnt + 1);
      }
      suspected = cnt <= 9;
    }
    if (suspected)
    {
      nofSuspectedPhases_ = (uint8_t)(nofSuspectedPhases_ + 1);
      suspectedPhase_[p] = 1;
    }
    else
    {
      suspectedPhase_[p] = 0;
    }
  }

  uint8_t ref = ref0;
  if (dbgOn())
  {
    dsplibs_debug_printf("V90AutoDigitalImpDetector: suspected rbs patern  %d%d%d%d%d%d\r\n", (int)suspectedPhase_[0], (int)suspectedPhase_[1],
                         (int)suspectedPhase_[2], (int)suspectedPhase_[3], (int)suspectedPhase_[4], (int)suspectedPhase_[5]);
    ref = refUcode_;
  }

  /* empty every bin; zero the linear mapping except at the reference ucode */
  for (uint8_t p = 0; p <= 5; p++)
  {
    const int row = (int)p << 7;
    if (ref != 0)
    {
      for (unsigned j = 0; j < ref; j++)
      {
        int idx = row + (int)j;
        cumSum_[idx] = 0.0f;
        cumCount_[idx] = 0;
        cumSumSq_[idx] = 0.0f;
        linearMapping_[idx] = 0;
      }
    }
    cumCount_[row + ref] = 0;
    cumSum_[row + ref] = 0.0f;
    cumSumSq_[row + ref] = 0.0f;
    for (uint8_t u = (uint8_t)(ref + 1); (int8_t)u >= 0; u = (uint8_t)(u + 1))
    {
      int idx = row + (int)u;
      cumCount_[idx] = 0;
      cumSum_[idx] = 0.0f;
      cumSumSq_[idx] = 0.0f;
      linearMapping_[idx] = 0;
    }
  }
}

/* ------------------------------------------------------------------------------------------------ porcessSecondStudy */

/**
 * @brief Second end-of-study step: choose the unsuspected phase, snap the suspected phases onto it and rebuild the alternate phases.
 *
 * unsuspectedPhase_ becomes the first phase not suspected of RBS.  For every ucode 0..116 and every suspected phase that is not an alternate phase, the
 * phase's linear value is compared with three consecutive linear values of the unsuspected phase (starting at the same ucode when the phase's value is
 * above the unsuspected value of that ucode, else two ucodes lower); when the nearest candidate is exact, or the second nearest is more than twice as far,
 * the phase takes that candidate.  Then updateAltRbsPhaseInDil() runs and, with the debug level above 1, the six-phase mapping is printed.
 *
 * @bug  The search for the unsuspected phase stops at phase 5 whether or not phase 5 is suspected, so with every phase suspected the "unsuspected" phase
 *       is 5 and the guard `localPhase > 5` below it can never be true.
 * @bug  For ucodes 0 and 1 the candidate window can start at ucode -2 / -1 (when the phase's value is not above the unsuspected phase's value), which
 *       reads linearMapping_ two entries before the unsuspected phase's row: the previous phase's row, or before the table for phase 0.
 * @note The chosen candidate index survives between iterations (it is not reset), and the second-best distance of one iteration starts the next.
 */
void V90AutoDigitalImpDetector::porcessSecondStudy()
{
  int32_t bestIdx = 0; /* survives from iteration to iteration */

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

  int16_t localPhase = unsuspectedPhase_;
  if (localPhase > 5)
  {
    localPhase = 0;
    unsuspectedPhase_ = 0;
  }
  else
  {
    LD second = (LD)__builtin_nanf(""); /* NaN at first, then the second best distance of the previous iteration */
    const int row = (int)localPhase << 7;
    for (uint8_t u = 0; u <= 0x74; u++)
    {
      for (uint8_t ph2 = 0; ph2 <= 5; ph2++)
      {
        if (suspectedPhase_[ph2] == 0)
          continue;
        if (altRbsPhase_[ph2] != 0)
          continue;

        /* candidate window of three levels of the unsuspected phase */
        const int cur = ((int)ph2 << 7) + (int)u;
        const int16_t refLin = linearMapping_[row + u];
        int cb = (int)u;
        if (!(linearMapping_[cur] > refLin))
          cb = (int)(int16_t)(u - 2);
        const int x = (int)linearMapping_[cur];
        LD best = (LD)32256.0f;
        LD sec = second;
        for (uint8_t k = 0; k <= 2; k++)
        {
          int cand = (int)linearMapping_[row + cb + k];
          LD dist = (LD)absI32(x - cand);
          if (jmpB(dist, best))
          {
            sec = best;
            best = dist;
            bestIdx = (int32_t)(int16_t)(cb + k);
          }
          else if (!jmpAe(dist, sec))
          {
            sec = dist;
          }
        }
        second = sec;

        /* accept the nearest candidate when it is exact or clearly nearer than the second */
        bool accept = false;
        if (best == 0.0L || best != best)
        {
          accept = true;
        }
        else
        {
          LD r = 1.0L / best;
          LD t = r * sec;
          accept = jmpA(t, (LD)2.0f);
        }
        if (accept)
          linearMapping_[cur] = linearMapping_[row + bestIdx];
      }
    }
  }

  edprintf("V90AutoDigitalImpDetector: unSuspectedPhase = %d\r\n", (int)localPhase);
  updateAltRbsPhaseInDil();

  /* debug report of the six mappings, ucodes 0 .. 127 */
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
  for (uint8_t u = 0;;)
  {
    if (lv > 1)
    {
      dsplibs_debug_printf("linearMapping[%d]  :  %d  %d  %d  %d  %d  %d\r\n", (int)u, (int)linearMapping_[u], (int)linearMapping_[0x80 + u],
                           (int)linearMapping_[0x100 + u], (int)linearMapping_[0x180 + u], (int)linearMapping_[0x200 + u], (int)linearMapping_[0x280 + u]);
      lv = dsplibs_debug_level;
    }
    u = (uint8_t)(u + 1);
    if ((int8_t)u < 0)
      break;
  }
  if (lv > 1)
    dsplibs_debug_printf("------------------------------------------------------\r\n");
}

/* ------------------------------------------------------------------------------------------------ studyUrefHandler */

/**
 * @brief Feed one received sample to the study state machine (see the file header for the states).
 * @param sampleF  received sample (float, PCM units)
 * @param phase    PCM phase 0..5 of the sample
 * @return 1 carry on, 0 the sample went into the alternate statistics (states 1 and 2 only), 2 the study is finished
 * @warning  `phase` is used as an array index without a range check.  State 0 divides by the bin counts of the reference ucode: a phase that received no
 *           sample in the first study length gives an infinite or NaN variance (no trap on the FPU, but the threshold computed from it is meaningless).
 *           States 3 and 5 divide by the number of non-alternate phases; if every phase is an alternate phase the result is NaN.
 */
int V90AutoDigitalImpDetector::studyUrefHandler(float sampleF, unsigned phase)
{
  const LD s = (LD)sampleF;
  const int sI = (int)trunc16(s);
  const unsigned st = studyState_;
  int ret = 1;
  if (st > 6)
    return ret;

  switch (st)
  {
  case 0:
  {
    /* initial statistics: which phases have a high variance at the reference ucode (alternate RBS)? */
    studyCounter_++;
    accumulate(this, phase, sI);
    if (studyCounter_ != studyLen0_)
      return ret;

    float tmp[6];
    {
      int idx = (int)refUcode_;
      for (int k = 0; k < 6; k++, idx += 128)
      {
        LD r = 1.0L / (LD)cumCount_[idx];
        LD m = (LD)cumSum_[idx] * r;
        LD q = r * (LD)cumSumSq_[idx];
        LD mm = m * m;
        tmp[k] = (float)(q - mm);
      }
    }
    if (dbgOn())
      dsplibs_debug_printf("V90AutoDigitalImpDetector:  initail var (Trn1):  %d  %d  %d  %d  %d  %d\n", dsp_trunc_i32((LD)tmp[0]), dsp_trunc_i32((LD)tmp[1]),
                           dsp_trunc_i32((LD)tmp[2]), dsp_trunc_i32((LD)tmp[3]), dsp_trunc_i32((LD)tmp[4]), dsp_trunc_i32((LD)tmp[5]));

    uint32_t w = floatBits(altVarFactor_);
    OPTIMIZER_HIDE_VAR(w);
    float thr = getAltVarThresh(tmp, bitsFloat(w));
    if (dbgOn())
      dsplibs_debug_printf("V90AutoDigitalImpDetector:  initail AltRbsVarThresh (Trn1) = %d\n", dsp_trunc_i32((LD)thr));

    int16_t cnt = 0;
    for (int k = 0; k < 6; k++)
    {
      if (jmpA((LD)tmp[k], (LD)thr))
      {
        cnt = (int16_t)(cnt + 1);
        altRbsPhase_[k] = 1;
      }
    }
    if (cnt > 0)
      clearRefBins(this);
    edprintf("V90AutoDigitalImpDetector: Trn1 alternate rbs initial patern  %d%d%d%d%d%d\n", (int)altRbsPhase_[0], (int)altRbsPhase_[1], (int)altRbsPhase_[2],
             (int)altRbsPhase_[3], (int)altRbsPhase_[4], (int)altRbsPhase_[5]);
    studyCounter_ = 0;
    studyState_ = 1;
    return ret;
  }
  case 1:
  {
    /* first linear values; samples of alternate phases are reported as 0 */
    studyCounter_++;
    accumulate(this, phase, sI);
    ret = altRbsPhase_[phase] == 0 ? 1 : 0;
    if (studyCounter_ != studyLen1_)
      return ret;

    binsToMapping(this);
    unitePhasesInfoOfUref((short)refUcode_);
    clearRefBins(this);
    {
      int six[6];
      sixValues(linearMapping_, (int)refUcode_, six);
      edprintf("V90AutoDigitalImpDetector trn1 first update  :  %d  %d  %d  %d  %d  %d\n", six[0], six[1], six[2], six[3], six[4], six[5]);
    }
    studyCounter_ = 0;
    studyState_ = 2;
    return ret;
  }
  case 2:
  {
    /* second linear values, first alternate mapping, removal of false alternate detections */
    studyCounter_++;
    if (altSample(this, phase, s))
    {
      accumulateAlt(this, phase, sI);
      ret = 0;
    }
    else
    {
      accumulate(this, phase, sI);
      ret = 1;
    }
    if (studyCounter_ != studyLen2_)
      return ret;

    binsToMapping(this);
    unitePhasesInfoOfUref((short)refUcode_);
    clearRefBins(this);
    altToMapping(this);
    if (dbgOn())
    {
      int six[6];
      sixValues(linearMapping_, (int)refUcode_, six);
      dsplibs_debug_printf("V90AutoDigitalImpDetector trn1 second update  :  %d  %d  %d  %d  %d  %d\n", six[0], six[1], six[2], six[3], six[4], six[5]);
      if (dbgOn())
      {
        sixValues(linearMappingAlt_, (int)refUcode_, six);
        dsplibs_debug_printf("V90AutoDigitalImpDetector trn1 Alt first update  :  %d  %d  %d  %d  %d  %d\n", six[0], six[1], six[2], six[3], six[4], six[5]);
      }
    }

    /* an alternate phase whose alternate value is within altRbsThresh_ of the normal one was a false detection */
    for (int k = 0; k < 6; k++)
    {
      if (altRbsPhase_[k] != 0)
      {
        int idx = k * 128 + (int)refUcode_;
        int flag = absI32(absI32((int)linearMappingAlt_[idx]) - (int)linearMapping_[idx]) > (int)altRbsThresh_ ? 1 : 0;
        altRbsPhase_[k] = (int16_t)flag;
        if (flag == 0 && dbgOn())
          dsplibs_debug_printf("V90AutoDigitalImpDetector: alternate rbs false detection on phase %d !!!\n", k);
      }
    }
    studyDone_ = 1;
    studyCounter_ = 0;
    studyState_ = 4;
    return ret;
  }
  case 3:
  {
    /* third update, same false-detection pass, then the noise level of the clean phases */
    studyCounter_++;
    if (altSample(this, phase, s))
      accumulateAlt(this, phase, sI);
    else
      accumulate(this, phase, sI);
    if (studyCounter_ != studyLen3_)
      return ret;

    binsToMapping(this);
    unitePhasesInfoOfUref((short)refUcode_);
    clearRefBins(this);
    altToMapping(this);
    unsigned lv = dsplibs_debug_level;
    if (lv > 1)
    {
      int six[6];
      sixValues(linearMapping_, (int)refUcode_, six);
      dsplibs_debug_printf("V90AutoDigitalImpDetector trn1 third update  :  %d  %d  %d  %d  %d  %d\n", six[0], six[1], six[2], six[3], six[4], six[5]);
      lv = dsplibs_debug_level;
      if (lv > 1)
      {
        sixValues(linearMappingAlt_, (int)refUcode_, six);
        dsplibs_debug_printf("V90AutoDigitalImpDetector trn1 Alt second update  :  %d  %d  %d  %d  %d  %d\n", six[0], six[1], six[2], six[3], six[4], six[5]);
        lv = dsplibs_debug_level;
      }
    }

    for (int k = 0; k < 6; k++)
    {
      if (altRbsPhase_[k] != 0)
      {
        int idx = k * 128 + (int)refUcode_;
        int flag = absI32(absI32((int)linearMappingAlt_[idx]) - (int)linearMapping_[idx]) > (int)altRbsThresh_ ? 1 : 0;
        altRbsPhase_[k] = (int16_t)flag;
        if (flag == 0 && lv > 1)
        {
          dsplibs_debug_printf("V90AutoDigitalImpDetector: alternate rbs false detection on phase %d !!!\n", k);
          lv = dsplibs_debug_level;
        }
      }
    }

    /* trn1Sigma_: mean variance of the non-alternate phases */
    LD acc = 0.0L;
    int16_t cnt = 0;
    for (int k = 0; k < 6; k++)
    {
      if (altRbsPhase_[k] == 0)
      {
        cnt = (int16_t)(cnt + 1);
        acc = acc + (LD)variance_[(k << 7) + (int)refUcode_];
      }
    }
    LD avg = acc / (LD)cnt;
    trn1Sigma_ = (float)avg;
    if (lv > 1)
      dsplibs_debug_printf("V90AutoDigitalImpDetector first update : trn1Sigma = %d\n", (double)avg);
    studyCounter_ = 0;
    studyState_ = 5;
    return ret;
  }
  case 4:
  {
    /* waiting state between the second and the third update */
    ret = 1;
    unsigned c = studyCounter_ + 1;
    if (c == studyLen4_)
    {
      studyCounter_ = 0;
      studyState_ = 3;
    }
    else
    {
      studyCounter_ = c;
    }
    return ret;
  }
  case 5:
  {
    /* final update (same length as state 3), no false-detection pass, trn1Sigma_ again */
    studyCounter_++;
    if (altSample(this, phase, s))
      accumulateAlt(this, phase, sI);
    else
      accumulate(this, phase, sI);
    if (studyCounter_ != studyLen3_)
      return ret;

    binsToMapping(this);
    unitePhasesInfoOfUref((short)refUcode_);
    clearRefBins(this);
    altToMapping(this);
    unsigned lv = dsplibs_debug_level;
    if (lv > 1)
    {
      int six[6];
      sixValues(linearMapping_, (int)refUcode_, six);
      dsplibs_debug_printf("V90AutoDigitalImpDetector trn1 final update  :  %d  %d  %d  %d  %d  %d\n", six[0], six[1], six[2], six[3], six[4], six[5]);
      lv = dsplibs_debug_level;
      if (lv > 1)
      {
        sixValues(linearMappingAlt_, (int)refUcode_, six);
        dsplibs_debug_printf("V90AutoDigitalImpDetector trn1 Alt final update  :  %d  %d  %d  %d  %d  %d\n", six[0], six[1], six[2], six[3], six[4], six[5]);
        lv = dsplibs_debug_level;
      }
    }

    LD acc = 0.0L;
    int16_t cnt = 0;
    for (int k = 0; k < 6; k++)
    {
      if (altRbsPhase_[k] == 0)
      {
        cnt = (int16_t)(cnt + 1);
        acc = acc + (LD)variance_[(k << 7) + (int)refUcode_];
      }
    }
    LD avg = acc / (LD)cnt;
    trn1Sigma_ = (float)avg;
    if (lv > 1)
      dsplibs_debug_printf("V90AutoDigitalImpDetector second update : trn1Sigma = %d\n", (double)avg);
    studyCounter_ = 0;
    studyState_ = 6;
    return 2;
  }
  default:
    return 2;
  }
}
