/**
 * @file v90_adid.cpp
 * @brief V90AutoDigitalImpDetector: bin statistics, phase uniting and small accessors.
 *
 * The detector keeps, for each of the six PCM phases and each of the 128 ucodes, a "bin": the count, the sum of |sample| and the sum of sample^2 of the
 * received samples whose nearest codec level is that ucode (cumCount_ / cumSum_ / cumSumSq_), from which mean and variance give the linear value
 * (linearMapping_) and its variance (variance_).  This file holds
 * - the construction and reset() of a study session,
 * - the accumulation of one sample (calculateLinearMeanAndVar(), addReceivedSampleToStorage()) and the conversion of a bin to variance / linear value
 *   (updateLinMappMeanAndVar()),
 * - the uniting of phases that measure the same level (unitePhasesInfoOfUref(), uniteLinMappInfoOfUnsuspectedPhases()),
 * - small accessors and tests (isAltRbs(), isThereAnyAltRbsPhase(), setConnectionType(), adjustUinfoToPhaseOffset(), applyPadGainToLinMapp()),
 * - the alternate-phase counterparts (calculateLinearMeanAndVarAlt(), updateUrefAlt(), ...), which nothing in the library calls.
 * The TRN1 state machine is in v90_adid_study.cpp, the pad-gain / maximum-ucode analysis in v90_adid_pad_gain.cpp.
 *
 * Arithmetic: the object stores sums, sums of squares and variances as `float`.  Intermediate results are computed in `long double` (x87 extended
 * precision) and rounded only when stored to a float member or converted to a 16-bit integer: a mean is `sum * (1 / n)` (the reciprocal is itself an
 * extended value), a variance `(1 / n) * sumsq - mean * mean`, a stored linear value `trunc16(mean + 0.5)` (truncating conversion; out of range or NaN
 * gives 0x8000).  The counters keep the integer widths of the 32-bit layout (16-bit signed counts in unitePhasesInfoOfUref()).  These roundings decide the
 * low bits of the results, so they must not be "simplified".
 *
 * Behaviours to be aware of (not defects of the call sequence the demodulator uses, but surprising):
 * - unitePhasesInfoOfUref() iterates until a termination sum stops changing; that sum adds the SAME word (linearMapping_[5][ucode]) six times (16-bit wrap),
 *   and only phases 0..4 are seeds.  When every phase 0..4 is flagged alternate it uses a best mean of 0.0 (never assigned) for the final value.
 * - uniteLinMappInfoOfUnsuspectedPhases() accumulates its group counter over all seeds (it is not reset per seed), so the winning seed is the last one
 *   that united at least one more phase.
 * - updateUrefAlt() multiplies by the reciprocal where updateLinMappMeanAndVarAlt() divides.
 * - calculateLinearMeanAndVar() accumulates |sample| into the bin of |linearValue| (the two arguments are different quantities).
 */

#include "v90_adid.h"
#include "core/dsp_math.h"
#include "pcm_leaf_classes.h"

extern "C"
{
}

typedef long double LD;

/* ------------------------------------------------------------------------------------------------ helpers */

#include "core/dsp_intmath.h"

#include "core/dsp_x87_conv.h"

/* ------------------------------------------------------------------------------------------------ construction */

V90AutoDigitalImpDetector::V90AutoDigitalImpDetector(V90Parameters* p) { params_ = p; }

V90AutoDigitalImpDetector::~V90AutoDigitalImpDetector() {}

/* ------------------------------------------------------------------------------------------------ small accessors */

/**
 * @brief Empty the bin (phase, ucode): count, sum and sum of squares become 0.
 * @param phase  PCM phase 0..5
 * @param ucode  ucode 0..127
 */
void V90AutoDigitalImpDetector::clearCamulativeVal(short phase, short ucode)
{
  int idx = ((int)phase << 7) + (int)ucode;
  cumCount_[idx] = 0;
  cumSum_[idx] = 0.0f;
  cumSumSq_[idx] = 0.0f;
}

/**
 * @brief Empty the alternate statistics of one phase (not called by the library).
 * @param phase  PCM phase 0..5
 * @note  The second argument is ignored.
 */
void V90AutoDigitalImpDetector::clearCamulativeAltVal(short phase, short)
{
  altCount_[(int)phase] = 0;
  altSum_[(int)phase] = 0.0f;
}

/**
 * @brief Select the thresholds that depend on the connection type.
 * @param type  2 selects the second line type (minMaxUcode_ 0x58, minUcodeScale_ 0.35, varThreshFactor_ 1.75); any other value the default
 *              (0x50, 0.25, 1.5)
 * @note  The default branch does not clear lineType2Flag_ (reset() does), so the flag stays 1 after a call with type 2.  Nothing reads the flag.
 */
void V90AutoDigitalImpDetector::setConnectionType(short type)
{
  if (type != 2)
  {
    minUcodeScale_ = 0.25f;
    minMaxUcode_ = 0x50;
    varThreshFactor_ = 1.5f;
  }
  else
  {
    lineType2Flag_ = 1;
    minMaxUcode_ = 0x58;
    minUcodeScale_ = 0.35f; /* single-precision constant 0x3eb33333 */
    varThreshFactor_ = 1.75f;
  }
}

/**
 * @brief Start a study session: remember the reference ucode, clear all statistics and load the default thresholds.
 * @param ucode    reference ucode of the study ("uref"), the ucode every phase is compared on
 * @param pcmType  codec law of the far end: PCM_MU_LAW or anything else for A-law
 * @param arg3     non-zero selects altVarFactor_ 5.0 instead of 1.5 (stored in resetArg3_)
 * @note  refLinear_ becomes the linear value of the reference ucode in the chosen law; padGain_ becomes 1.0; the connection-type thresholds follow
 *        params_->lineConnectionType (2 = second line type).  Must be called before any other method: the constructor leaves the object uninitialised.
 */
void V90AutoDigitalImpDetector::reset(unsigned char ucode, PcmType pcmType, short arg3)
{
  pcmType_ = (int32_t)pcmType;
  refUcode_ = ucode;
  if (pcmType != PCM_MU_LAW)
    refLinear_ = (int16_t)alaw2linear((int)(ucode & 0x7fu) ^ 0xd5);
  else
    refLinear_ = (int16_t)ulaw2linear((int)(ucode & 0x7fu) ^ 0xff);

  studyDone_ = 0;
  padGain_ = 1.0f;
  resetArg3_ = arg3;

  /* empty every bin of every phase and the per-phase flags and alternate statistics */
  for (uint16_t p = 0; p <= 5; p++)
  {
    int base = (int)(int16_t)p << 7;
    int ubase = (int)((unsigned)p << 7);
    for (uint16_t u = 0; u <= 0x7f; u++)
    {
      int idx = base + (int)(int16_t)u;
      cumCount_[idx] = 0;
      cumSum_[idx] = 0.0f;
      cumSumSq_[idx] = 0.0f;
      variance_[ubase + u] = 0.0f;
      storedCount_[ubase + u] = 0;
      ucodeAllowed_[ubase + u] = 1;
    }
    storageFill_[p] = 0;
    altRbsPhase_[p] = 0;
    altCount_[(int)(int16_t)p] = 0;
    suspectedPhase_[p] = 0;
    altSum_[(int)(int16_t)p] = 0.0f;
  }

  if (resetArg3_ != 0)
  {
    altVarFactor_ = 5.0f;
    altVarFactor974_ = 5.0f;
  }
  else
  {
    altVarFactor_ = 1.5f;
    altVarFactor974_ = 5.0f;
  }

  /* thresholds of the connection type */
  if ((int16_t)params_->lineConnectionType != 2)
  {
    minUcodeScale_ = 0.25f;
    lineType2Flag_ = 0;
    minMaxUcode_ = 0x50;
    varThreshFactor_ = 1.5f;
  }
  else
  {
    lineType2Flag_ = 1;
    minMaxUcode_ = 0x58;
    minUcodeScale_ = 0.35f;
    varThreshFactor_ = 1.75f;
  }
}

/**
 * @brief Zero both linear mappings and seed the reference ucode of each phase with refLinear_.
 */
void V90AutoDigitalImpDetector::resetLinearMapping()
{
  unsigned lin = (uint16_t)refLinear_;
  unsigned u = refUcode_;
  for (int16_t p = 0; p <= 5; p++)
  {
    int base = (int)p << 7;
    for (int16_t k = 0; k <= 0x7f; k++)
    {
      linearMapping_[base + k] = 0;
      linearMappingAlt_[base + k] = 0;
    }
    linearMapping_[base + u] = (int16_t)lin;
    linearMappingAlt_[base + u] = (int16_t)lin;
  }
}

/**
 * @brief Test whether any phase is flagged as alternate RBS.
 * @return 1 when the (16-bit) sum of the six flags is positive, else 0
 */
int V90AutoDigitalImpDetector::isThereAnyAltRbsPhase()
{
  int16_t sum = 0;
  for (int16_t i = 0; i <= 5; i++)
    sum = (int16_t)((int)sum + (int)(uint16_t)altRbsPhase_[i]);
  return sum > 0 ? 1 : 0;
}

/**
 * @brief Test whether a sample of an alternate-RBS phase differs from the phase's linear value.
 * @param phase  PCM phase 0..5
 * @param ucode  ucode the sample was assigned to
 * @param value  received sample
 * @return 1 when `phase` is an alternate phase and | |value| - linearMapping_[phase][ucode] | exceeds altRbsThresh_, else 0
 */
int V90AutoDigitalImpDetector::isAltRbs(short phase, short ucode, float value)
{
  if (altRbsPhase_[(int)phase] == 0)
    return 0;

  int t = absI32(dsp_trunc_i32((LD)value));
  int lin = (int)linearMapping_[((int)phase << 7) + (int)ucode];
  int d = absI32(t - lin);
  return d > (int)altRbsThresh_ ? 1 : 0;
}

/**
 * @brief Add |value| to the alternate sum and count of a phase (not called by the library).
 * @param value  sample
 * @param phase  PCM phase 0..5, not range-checked
 */
void V90AutoDigitalImpDetector::calculateLinearMeanAndVarAlt(short value, unsigned phase)
{
  int a = absI32((int)value);
  LD s = (LD)a + (LD)altSum_[phase];
  altCount_[phase]++;
  altSum_[phase] = (float)s;
}

/* ------------------------------------------------------------------------------------------------ mean / variance of one (phase, ucode) bin */

/**
 * @brief Convert the accumulated sums of the bin (phase, ucode) to its variance and linear value.
 * @param phase  PCM phase 0..5
 * @param ucode  ucode 0..127
 * @note  A bin with a zero count is left unchanged.  mean = sum / n, variance_ = sumsq / n - mean^2 (float), linearMapping_ = trunc16(mean + 0.5).
 */
void V90AutoDigitalImpDetector::updateLinMappMeanAndVar(short phase, short ucode)
{
  int idx = ((int)phase << 7) + (int)ucode;
  if (cumCount_[idx] != 0)
  {
    LD r = 1.0L / (LD)cumCount_[idx];
    LD m = (LD)cumSum_[idx] * r;
    LD q = r * (LD)cumSumSq_[idx];
    LD mm = m * m;
    LD var = q - mm;
    variance_[idx] = (float)var;
    linearMapping_[idx] = trunc16(m + 0.5L);
  }
}

/**
 * @brief Variance and linear value of the reference ucode in all six phases, unite the phases, clear the sums (not called by the library).
 */
void V90AutoDigitalImpDetector::updateUref()
{
  unsigned u = refUcode_;
  int idx = (int)u;
  for (int16_t i = 0; i <= 5; i++, idx += 128)
  {
    if (cumCount_[idx] != 0)
    {
      LD r = 1.0L / (LD)cumCount_[idx];
      LD m = (LD)cumSum_[idx] * r;
      LD q = r * (LD)cumSumSq_[idx];
      LD mm = m * m;
      LD var = q - mm;
      variance_[idx] = (float)var;
      linearMapping_[idx] = trunc16(m + 0.5L);
    }
  }

  unitePhasesInfoOfUref((short)u);

  idx = (int)refUcode_;
  for (int16_t i = 0; i <= 5; i++, idx += 128)
  {
    cumSum_[idx] = 0.0f;
    cumCount_[idx] = 0;
    cumSumSq_[idx] = 0.0f;
  }
}

/**
 * @brief Alternate linear value of the bin (phase, ucode) as the mean of the alternate samples (not called by the library).
 * @param phase  PCM phase 0..5
 * @param ucode  ucode 0..127
 */
void V90AutoDigitalImpDetector::updateLinMappMeanAndVarAlt(short phase, short ucode)
{
  int ph = (int)phase;
  if (altCount_[ph] != 0)
  {
    int idx = (ph << 7) + (int)ucode;
    LD q = (LD)altSum_[ph] / (LD)altCount_[ph];
    linearMappingAlt_[idx] = trunc16(0.5L + q);
  }
}

/**
 * @brief Alternate linear value of the reference ucode for every alternate-RBS phase; clears the alternate sums (not called by the library).
 */
void V90AutoDigitalImpDetector::updateUrefAlt()
{
  for (int16_t i = 0; i <= 5; i++)
  {
    if (altRbsPhase_[i] != 0 && altCount_[i] != 0)
    {
      int idx = ((int)i << 7) + (int)refUcode_;
      LD r = 1.0L / (LD)altCount_[i];
      LD q = r * (LD)altSum_[i]; /* the reciprocal is multiplied in (updateLinMappMeanAndVarAlt() divides) */
      linearMappingAlt_[idx] = trunc16(q + 0.5L);
    }
    altSum_[i] = 0.0f;
    altCount_[i] = 0;
  }
}

/* ------------------------------------------------------------------------------------------------ uniting phases */

/**
 * @brief Merge the measurements of the phases that agree on one ucode.
 * @param ucode  ucode (normally the reference ucode)
 *
 * Phases that are not alternate-RBS phases are grouped greedily: a seed phase i collects every later phase j whose linear value differs from the
 * seed's by less than sqrDiffThreshUref_; the group's sums give one mean / variance, which is written to every member.  The largest group's linear
 * value and variance are remembered.  The pass repeats until the 16-bit termination sum (linearMapping_[5][ucode] added six times) is unchanged.
 * Finally every alternate-RBS phase receives the largest group's linear value and variance.
 *
 * @note  Only phases 0..4 are seeds.  bestMean starts at 0.0 and bestVar at NaN; they keep those values when no phase 0..4 qualifies as seed (all of them
 *        alternate), and the alternate phases then receive 0 and NaN.
 */
void V90AutoDigitalImpDetector::unitePhasesInfoOfUref(short ucode)
{
  const int u = (int)ucode;
  LD bestVar = (LD)(__builtin_nanf("")); /* quiet NaN, replaced as soon as a seed qualifies */
  float bestMean = 0.0f;
  int16_t prevTotal = 0;

  for (;;)
  {
    int16_t bestCnt = 0;
    int16_t group[6];
    for (int16_t k = 0; k <= 5; k++)
      group[k] = -1;

    int i = 0;
    for (;;)
    {
      if (group[i] == -1 && altRbsPhase_[i] == 0)
      {
        /* gather the later phases that agree with seed phase i */
        int idx = (i << 7) + u;
        int n = (int)(int16_t)cumCount_[idx];
        LD sum = (LD)cumSum_[idx];
        LD sumsq = (LD)cumSumSq_[idx];
        for (int j = i + 1; j <= 5; j++)
        {
          if (group[j] != -1 || altRbsPhase_[j] != 0)
            continue;
          int jdx = (j << 7) + u;
          int d = absI32((int)linearMapping_[idx] - (int)linearMapping_[jdx]);
          if (d < (int)sqrDiffThreshUref_)
          {
            group[j] = (int16_t)i;
            sum = sum + (LD)cumSum_[jdx];
            sumsq = sumsq + (LD)cumSumSq_[jdx];
            n = (int)(int16_t)(n + (int)cumCount_[jdx]);
          }
        }

        /* one mean / variance for the whole group, in the seed's bin */
        if (n != 0)
        {
          LD r = 1.0L / (LD)(int16_t)n;
          LD m = sum * r;
          LD q = sumsq * r;
          LD mm = m * m;
          linearMapping_[idx] = trunc16(m + 0.5L);
          variance_[idx] = (float)(q - mm);
        }

        /* copy it to the other members and remember the largest group */
        group[i] = (int16_t)i;
        int16_t cnt = 1;
        for (int j = i + 1; j <= 5; j++)
        {
          if (group[j] == (int16_t)i)
          {
            int jdx = (j << 7) + u;
            linearMapping_[jdx] = linearMapping_[idx];
            variance_[jdx] = variance_[idx];
            cnt = (int16_t)(cnt + 1);
          }
        }
        if (cnt > bestCnt)
        {
          bestCnt = cnt;
          bestMean = (float)(int)linearMapping_[idx];
          bestVar = (LD)variance_[idx];
        }
      }
      i++;
      if (i > 4)
        break;
    }

    /* termination: six times the word linearMapping_[5][ucode] in 16-bit arithmetic, unchanged since the previous pass */
    uint16_t w = (uint16_t)linearMapping_[(5 << 7) + u];
    int16_t total = 0;
    for (int16_t k = 0; k <= 5; k++)
      total = (int16_t)((int)total + (int)w);
    bool same = total == prevTotal;
    prevTotal = total;
    if (same)
      break;
  }

  /* the alternate phases take the value of the largest group */
  int16_t cx = trunc16((LD)bestMean);
  for (int i = 0; i <= 5; i++)
  {
    if (altRbsPhase_[i] != 0)
    {
      int idx = (i << 7) + u;
      linearMapping_[idx] = cx;
      variance_[idx] = (float)bestVar;
    }
  }
}

/**
 * @brief Merge, for one ucode, the phases that are not suspected of RBS into the largest group of agreeing phases and clear the sums of all phases.
 * @param ucode  ucode 0..127
 *
 * A seed phase k (0..4, not suspected) collects every later unsuspected phase m whose squared linear-value difference is below a quarter of the seed's
 * variance.  The seed with the largest (running) group count wins; its group's sums give a common mean / variance that every unsuspected phase receives.
 * The sums of all six phases are cleared.  Nothing changes when the winning group has no samples.
 */
void V90AutoDigitalImpDetector::uniteLinMappInfoOfUnsuspectedPhases(unsigned char ucode)
{
  const unsigned u = ucode;
  int16_t group[6];
  for (unsigned char k = 0; k <= 5; k++)
    group[k] = -1;
  int16_t cnt = 1;
  int16_t bestCnt = 0;
  unsigned bestSeed = 0;

  /* grouping around every unsuspected seed; cnt keeps counting across seeds */
  for (unsigned k = 0; k <= 4; k++)
  {
    if (suspectedPhase_[k] == 0 && group[k] == -1)
    {
      group[k] = (int16_t)k;
      int seedIdx = (int)((k << 7) + u);
      for (unsigned m = k + 1; m <= 5; m++)
      {
        if (suspectedPhase_[m] != 0 || group[m] != -1)
          continue;
        LD thr = (LD)variance_[seedIdx] * (LD)0.25f;
        int diff = (int)linearMapping_[seedIdx] - (int)linearMapping_[(m << 7) + u];
        LD d2 = (LD)diff * (LD)diff;
        /* united when d2 < 0.25 * var; an unordered compare (NaN variance) also unites */
        if (!(d2 >= thr))
        {
          group[m] = (int16_t)k;
          cnt = (int16_t)(cnt + 1);
        }
      }
      if (cnt > bestCnt)
      {
        bestSeed = k;
        bestCnt = cnt;
      }
    }
  }

  /* sums of the winning group */
  LD sum = 0.0L, sumsq = 0.0L;
  int n = 0;
  for (unsigned k = 0; k <= 5; k++)
  {
    if (suspectedPhase_[k] == 0 && group[k] == (int16_t)bestSeed)
    {
      int idx = (int)((k << 7) + u);
      sum = sum + (LD)cumSum_[idx];
      sumsq = sumsq + (LD)cumSumSq_[idx];
      n = (int)(int16_t)(n + (int)cumCount_[idx]);
    }
  }
  if (n == 0)
    return;

  LD r = (LD)1.0f / (LD)(int16_t)n;
  LD m = sum * r;
  LD q = sumsq * r;
  LD mm = m * m;
  LD var = q - mm;
  int16_t t = trunc16(m + (LD)0.5f);

  /* every unsuspected phase gets the common value; all sums are cleared */
  for (unsigned k = 0; k <= 5; k++)
  {
    int idx = (int)((k << 7) + u);
    if (suspectedPhase_[k] == 0)
    {
      linearMapping_[idx] = t;
      variance_[idx] = (float)var;
    }
    cumSum_[idx] = 0.0f;
    cumCount_[idx] = 0;
    cumSumSq_[idx] = 0.0f;
  }
}

/**
 * @brief Find the linear value of `phase` (ucodes 5..116) nearest to |value| (not called by the library).
 * @param value  linear sample value
 * @param phase  PCM phase 0..5
 * @return linearMapping_[phase][u] of the ucode u in 5..116 with the smallest | |value| - linearMapping_ |
 */
int V90AutoDigitalImpDetector::unSuspectedPhaseNearestLinMapp(short value, short phase)
{
  int best = 1000000;
  unsigned bestU = 0; /* stays 0 only if no candidate beats 1,000,000, which |value| <= 32768 excludes */
  int av = absI32((int)value);
  int base = (int)phase << 7;
  for (unsigned u = 5; (uint16_t)u <= 0x74; u = (uint16_t)(u + 1))
  {
    int d = absI32(av - (int)linearMapping_[base + (int)u]);
    if (d < best)
    {
      bestU = u;
      best = d;
    }
  }
  return (int)linearMapping_[base + (int)bestU];
}

/* ------------------------------------------------------------------------------------------------ sample storage, accumulation */

/**
 * @brief Store |round(value)| of one sample of an alternate phase and count it for its ucode.
 * @param phase  PCM phase 0..5
 * @param ucode  ucode the sample belongs to
 * @param value  received sample
 * @warning  The per-phase store holds 2110 samples (storage_ row stride 0x83e) and storageFill_[phase] is not compared with it: the study lengths in the
 *           parameter block must keep the number of calls per phase below 2110, otherwise the writes run into the next phase's row and, for phase 5,
 *           past storage_.
 */
void V90AutoDigitalImpDetector::addReceivedSampleToStorage(short phase, unsigned char ucode, float value)
{
  int ph = (int)phase;
  unsigned fill = storageFill_[ph];
  int pos = ph * 0x83e + (int)fill;
  int16_t t = trunc16((LD)0.5f + (LD)value);
  storageFill_[ph] = fill + 1;
  storage_[pos] = (int16_t)absI32((int)t);
  int bin = (ph << 7) + (int)ucode;
  storedCount_[bin] = (uint16_t)(storedCount_[bin] + 1);
}

/**
 * @brief Accumulate one sample into the bin of the codec level of `linearValue`.
 * @param sample       received sample; |sample| and sample^2 are added
 * @param linearValue  linear value that selects the bin: its ucode in the current codec law (A-law when pcmType_ != 0)
 * @param phase        PCM phase 0..5, not range-checked
 */
void V90AutoDigitalImpDetector::calculateLinearMeanAndVar(short sample, short linearValue, unsigned phase)
{
  unsigned u;
  int lv = absI32((int)linearValue);
  if (pcmType_ != 0)
    u = (uint8_t)linear2alaw(lv) ^ 0xd5u;
  else
    u = (unsigned)(int)(int16_t)(0xff - (uint8_t)linear2ulaw(lv));

  int idx = (int)((phase << 7) + u);
  int a = absI32((int)sample);
  int sq = a * a;
  cumSum_[idx] = (float)((LD)a + (LD)cumSum_[idx]);
  LD s2 = (LD)sq + (LD)cumSumSq_[idx];
  cumCount_[idx]++;
  cumSumSq_[idx] = (float)s2;
}

/**
 * @brief Divide every entry of linearMapping_ and linearMappingAlt_ by padGain_ (rounded to nearest) to undo the pad attenuation.
 * @warning  padGain_ == 0 gives an infinite reciprocal; findPadGain() never produces 0 (candidate gains are ratios of non-zero linear values).
 */
void V90AutoDigitalImpDetector::applyPadGainToLinMapp()
{
  LD r = (LD)1.0f / (LD)padGain_; /* one rounding */
  const LD half = (LD)0.5f;
  for (int16_t p = 0; p <= 5; p++)
  {
    int base = (int)p << 7;
    for (int16_t k = 0; k <= 0x7f; k++)
    {
      int idx = base + k;
      linearMapping_[idx] = trunc16((LD)linearMapping_[idx] * r + half);
      linearMappingAlt_[idx] = trunc16((LD)linearMappingAlt_[idx] * r + half);
    }
  }
}

/**
 * @brief Copy six per-phase maximum ucodes into maxUcodeArray_ (not called by the library).
 * @param a  six bytes
 */
void V90AutoDigitalImpDetector::setMaxUcodeArray(unsigned char* a)
{
  for (int16_t i = 0; i <= 5; i++)
    maxUcodeArray_[i] = a[i];
}

/**
 * @brief Copy a 128-entry linear mapping of a previous session into prevSessionLinearMapping_ (not called by the library).
 * @param m  128 linear values
 */
void V90AutoDigitalImpDetector::setPrevSessionLinearMapping(short* m)
{
  for (int16_t i = 0; i <= 0x7f; i++)
    prevSessionLinearMapping_[i] = m[i];
}

/**
 * @brief Rotate the six linear values of the reference ucode so that the sequence starts at phase `offset`.
 * @param offset  phase 0..5 that becomes phase 0
 * @warning  The phase counter wraps to 0 only when it equals exactly 6, and `offset` is not range-checked: a value above 5 reads past the six rows
 *           (linearMapping_ has 768 entries) and a negative value reads before the table.
 */
void V90AutoDigitalImpDetector::adjustUinfoToPhaseOffset(short offset)
{
  const unsigned u = refUcode_;
  int16_t tmp[6];
  int p = (int)offset;
  for (int16_t k = 0; k <= 5; k++)
  {
    int next = (int)(int16_t)(p + 1);
    tmp[k] = linearMapping_[(p << 7) + (int)u];
    p = ((int16_t)next != 6) ? next : 0;
  }

  for (int16_t k = 0; k <= 5; k++)
    linearMapping_[((int)k << 7) + (int)u] = tmp[k];
}
