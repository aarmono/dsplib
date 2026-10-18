/**
 * @file ring_detector.c
 * @brief Soft ring detector: creation, per-block processing and result query.
 *
 * Implements the four entry points declared in ring_detector.h, which the RD glue object of core/dp_runtime.c calls.  The state object and the
 * algorithm (idle / positive half-wave / negative half-wave, crossing confirmation, period-frequency averaging, ON / OFF decision) are described
 * in ring_detector.h; this file only holds the code.  The only host services used are sysdep_malloc() / sysdep_free() and the debug printer
 * (core/dsp_host.h).
 *
 * RingDetector_Process() is one flat per-sample loop written with labels and gotos: the three states share the "between the levels",
 * "beyond the levels" and "crossing done" tails, which are marked at their labels.
 */

#include "ring_detector.h"
#include "core/dsp_host.h"

/**
 * @brief Allocate a detector and derive its working parameters from the configuration.
 * @param cfg  RD_CFG_COUNT ints indexed by the RD_CFG_* constants (see ring_detector.h)
 * @return the new detector in the idle state, with all counters cleared; ownership passes to the caller (RingDetector_Delete())
 * @note  The configured frequencies and durations are clamped: minFreq >= 14 Hz, maxFreq <= 100 Hz, minOnMs >= 40 ms, minOffMs >= 120 ms.
 *        |threshold| becomes the crossing level.  The crossing confirm count is |threshold| / (sampleRate / 80).  A negative configured
 *        threshold selects confirmAfterStart = 0 / releaseLevel = 200, a non-negative one 2 / 100.
 *        With the debug level above 1 the resulting parameters are printed.
 * @bug   The allocation is not checked: a NULL result of sysdep_malloc() is dereferenced.
 * @bug   maxBetweenSamples is computed from the configured minFreq BEFORE it is clamped to 14 Hz, and by an integer division: a configured
 *        minFreq of 0 divides by zero (and, for any value below 14, the limit is larger than the clamped frequency implies).  The same holds for
 *        a sample rate below 80 Hz (division by sampleRate / 80 == 0).  The RD glue always passes 15 Hz and 8000 / 9600 Hz.
 */
RingDetector* RingDetector_Create(int* cfg)
{
  RingDetector* rd;
  int threshold;
  int fs;
  int minFreqIn;
  int absThreshold;
  short level;

  rd = (RingDetector*)sysdep_malloc(sizeof(RingDetector));
  threshold = cfg[RD_CFG_THRESHOLD];
  fs = cfg[RD_CFG_SAMPLE_RATE];
  minFreqIn = cfg[RD_CFG_MIN_FREQ];

  /* configured parameters and the quantities derived from them */
  rd->ringOn = 0;
  rd->sampleRate = fs;
  absThreshold = threshold < 0 ? -threshold : threshold;
  rd->confirmSamples = (short)(absThreshold / (fs / 0x50));
  rd->minFreq = minFreqIn;
  rd->maxFreq = cfg[RD_CFG_MAX_FREQ];
  rd->minOnMs = cfg[RD_CFG_MIN_ON_MS];
  rd->maxBetweenSamples = (short)((fs * 3) / (minFreqIn * 4));
  rd->minOffMs = cfg[RD_CFG_MIN_OFF_MS];

  /* clamp the configured limits to the range the detector supports */
  if (rd->minFreq < 0xe)
  {
    rd->minFreq = 0xe;
  }
  if (100 < rd->maxFreq)
  {
    rd->maxFreq = 100;
  }
  if (rd->minOnMs < 0x28)
  {
    rd->minOnMs = 0x28;
  }
  if (rd->minOffMs < 0x78)
  {
    rd->minOffMs = 0x78;
  }

  if (1 < (int)dsplibs_debug_level)
  {
    dsplibs_debug_printf("\n Reset Soft Ring: Threshold = %d, Fs = %d, MinFreq = %d, MaxFreq =%d \n\tminOnDur = %d, minOffDur = %d \n", cfg[RD_CFG_THRESHOLD],
                         rd->sampleRate, rd->minFreq, rd->maxFreq, rd->minOnMs, rd->minOffMs);
  }

  /* start idle with all counters cleared; the idle half-wave levels are +/- threshold */
  rd->phaseSamples = 0;
  rd->periodSamples = 0;
  rd->quietSamples = 0;
  rd->freqAvg = 0;
  level = (short)absThreshold;
  rd->threshold = level;
  rd->upperLevel = level;
  rd->lastFreq = 0;
  rd->freqAvgCount = 0;
  rd->prevRingOn = 0;
  rd->state = RING_STATE_IDLE;
  rd->aboveCount = 0;
  rd->belowCount = 0;
  rd->lowerLevel = (short)-level;
  rd->betweenSamples = 0;
  rd->cycleCount = 0;
  rd->halfWaveSamples = 0;
  rd->crossGapSamples = 0;
  rd->riseConfirm = rd->confirmSamples;
  rd->fallConfirm = rd->confirmSamples;

  /* the sign of the configured threshold selects the confirm / release presets */
  if (threshold < 0)
  {
    rd->confirmAfterStart = 0;
    rd->releaseLevel = 200;
    if (rd->minOffMs < 0x78)
    {
      rd->minOffMs = 0x78;
      return rd;
    }
  }
  else
  {
    rd->confirmAfterStart = 2;
    rd->releaseLevel = 100;
  }
  return rd;
}

/**
 * @brief Free a detector.
 * @param rd  detector from RingDetector_Create(), or NULL (ignored)
 */
void RingDetector_Delete(RingDetector* rd)
{
  if (rd != (RingDetector*)0)
  {
    sysdep_free(rd);
  }
}

/**
 * @brief Run the detector over a block of line samples and decide whether the ring turned ON or OFF.
 * @param rd       detector
 * @param samples  `count` 16-bit line samples (read only)
 * @param count    number of samples; 0 is allowed
 * @return 1 if the ring state changed during this call (the duration of the period that ended is then available from
 *         RingDetector_GetLastRing()), else 0
 *
 * Per sample the state machine of ring_detector.h advances: crossings are confirmed, a closed period updates the running frequency mean,
 * a stay between the two levels longer than maxBetweenSamples or a half-wave longer than one period at minFreq returns the detector to idle.
 * After the sample the ON / OFF decision is made: ON when the counted periods (cycleCount * 1000 / freqAvg ms) last longer than minOnMs, OFF when
 * quietSamples or crossGapSamples exceed minOffMs.  At the end of the block, if ringOn differs from the previous call, the duration of the finished
 * period is stored in lastDurationMs: phaseSamples converted to ms and corrected by (minOnMs - minOffMs + 20) after an ON period or
 * (minOffMs - minOnMs - 20) after an OFF period, then raised to at least minOnMs / minOffMs respectively; phaseSamples restarts.
 *
 * The locals are shared between the two phases (the three `x_y` ints hold x in the sample loop and y in the ON / OFF evaluation after it;
 * `tmp` / `tmp2` are short scratch values: state, level, sample or counter).
 *
 * @bug phaseSamples is incremented for every sample and only cleared on an ON / OFF change, and quietSamples for every idle sample and only cleared
 *      when a burst starts: both are plain ints that overflow after about 2^31 samples (roughly 74 hours at 8 kHz) of an unchanged line state.
 */
int RingDetector_Process(RingDetector* rd, short* samples, unsigned int count)
{
  int oldAvgCount;
  short tmp;
  short tmp2;
  int changed;
  int minFreq_durMs;
  int period_freq;
  unsigned int i;
  int offLimit_minOnMs;
  int rate_minOffMs;
  int gap_newCount;

  i = 0;
  if (count != 0)
  {
    /* limits shared by all samples of the block */
    minFreq_durMs = rd->minFreq;
    rate_minOffMs = rd->sampleRate;
    offLimit_minOnMs = (rate_minOffMs * rd->minOffMs) / 1000;

    do
    {
      tmp = rd->state;
      rd->phaseSamples = rd->phaseSamples + 1;
      if (tmp == RING_STATE_IDLE)
      { /* idle: wait for a confirmed crossing of +threshold or -threshold; it sets the half-wave levels */
        tmp = rd->threshold;
        rd->periodSamples = 0;
        tmp2 = samples[i];
        if (tmp < tmp2)
        {
          rd->belowCount = 0;
          tmp2 = rd->aboveCount + 1;
          rd->aboveCount = tmp2;
          if (tmp2 <= rd->riseConfirm)
          {
          /* count_quiet: still idle; the sample counts as quiet */
          count_quiet:
            rd->quietSamples = rd->quietSamples + 1;
            goto sample_done;
          }
          tmp2 = rd->releaseLevel;
          rd->state = RING_STATE_POSITIVE;
          rd->upperLevel = tmp;
          rd->fallConfirm = rd->confirmAfterStart;
        }
        else
        {
          /* sample >= -threshold: not beyond the negative level either; the sum of two shorts cannot overflow an int */
          if (tmp2 + tmp >= 0)
          {
            rd->quietSamples = rd->quietSamples + 1;
          /* clear_cross_counts: the sample is inside (or beyond) the levels without a crossing in progress */
          clear_cross_counts:
            rd->aboveCount = 0;
            rd->belowCount = 0;
            goto sample_done;
          }
          rd->aboveCount = 0;
          tmp2 = rd->belowCount + 1;
          rd->belowCount = tmp2;
          if (tmp2 <= rd->fallConfirm)
            goto count_quiet;
          tmp2 = -tmp;
          rd->state = RING_STATE_NEGATIVE;
          rd->upperLevel = -rd->releaseLevel;
          rd->riseConfirm = rd->confirmAfterStart;
        }
        rd->lowerLevel = tmp2;
        rd->quietSamples = 0;
      }
      else if (tmp == RING_STATE_POSITIVE)
      { /* positive half-wave: wait for a confirmed fall below lowerLevel */
        period_freq = rd->periodSamples + 1;
        rd->halfWaveSamples = rd->halfWaveSamples + 1;
        gap_newCount = rd->crossGapSamples + 1;
        tmp = samples[i];
        if (rd->lowerLevel <= tmp)
        {
          rd->periodSamples = period_freq;
          if (rd->upperLevel <= tmp)
            goto beyond_levels;
        /* between_levels: the sample lies between lowerLevel and upperLevel; too long a stay there drops the detector back to idle */
        between_levels:
          rd->aboveCount = 0;
          rd->belowCount = 0;
          tmp = rd->betweenSamples + 1;
          if (tmp <= rd->maxBetweenSamples)
          {
            rd->betweenSamples = tmp;
            rd->crossGapSamples = gap_newCount;
            goto sample_done;
          }
          rd->cycleCount = 0;
          rd->state = RING_STATE_IDLE;
          rd->quietSamples = (int)rd->maxBetweenSamples;
          rd->betweenSamples = 0;
          rd->riseConfirm = rd->confirmSamples;
          rd->fallConfirm = rd->confirmSamples;
          goto reset_half_wave;
        }
        rd->aboveCount = 0;
        tmp = rd->belowCount + 1;
        rd->belowCount = tmp;
        if (rd->fallConfirm < tmp)
        {
          if (rd->fallConfirm == rd->confirmSamples)
          {
            period_freq = rate_minOffMs / period_freq; /* a full period closed: its frequency */
            if ((period_freq < rd->maxFreq + 1) && (minFreq_durMs <= period_freq))
            {
              oldAvgCount = rd->freqAvgCount;
              gap_newCount = oldAvgCount + 1;
              rd->freqAvgCount = gap_newCount;
              rd->freqAvg = (short)((rd->freqAvg * oldAvgCount + period_freq) / gap_newCount);
            }
            rd->periodSamples = 0;
            rd->cycleCount = rd->cycleCount + 1;
          }
          else
          {
            rd->periodSamples = period_freq;
          }
          rd->state = RING_STATE_NEGATIVE;
          goto crossing_done;
        }
        /* store_period_and_gap: no crossing yet; advance the period and the gap counters */
      store_period_and_gap:
        rd->crossGapSamples = gap_newCount;
        rd->periodSamples = period_freq;
      }
      else if (tmp == RING_STATE_NEGATIVE)
      { /* negative half-wave: wait for a confirmed rise above upperLevel */
        period_freq = rd->periodSamples + 1;
        rd->halfWaveSamples = rd->halfWaveSamples + 1;
        tmp = samples[i];
        gap_newCount = rd->crossGapSamples + 1;
        if (tmp <= rd->upperLevel)
        {
          rd->periodSamples = period_freq;
          if (rd->lowerLevel < tmp)
            goto between_levels;
          /* beyond_levels: the sample is beyond the outer level of the current half-wave (upperLevel when positive, lowerLevel when negative);
           * no crossing is under way, so the crossing counters are cleared */
        beyond_levels:
          rd->crossGapSamples = gap_newCount;
          goto clear_cross_counts;
        }
        rd->belowCount = 0;
        tmp = rd->aboveCount + 1;
        rd->aboveCount = tmp;
        if (tmp <= rd->riseConfirm)
          goto store_period_and_gap;
        if (rd->riseConfirm == rd->confirmSamples)
        {
          period_freq = rate_minOffMs / period_freq; /* a full period closed: its frequency */
          if ((period_freq < rd->maxFreq + 1) && (minFreq_durMs <= period_freq))
          {
            oldAvgCount = rd->freqAvgCount;
            gap_newCount = oldAvgCount + 1;
            rd->freqAvgCount = gap_newCount;
            rd->freqAvg = (short)((rd->freqAvg * oldAvgCount + period_freq) / gap_newCount);
          }
          rd->periodSamples = 0;
          rd->cycleCount = rd->cycleCount + 1;
        }
        else
        {
          rd->periodSamples = period_freq;
        }
        rd->state = RING_STATE_POSITIVE;
        /* crossing_done: a confirmed crossing switched the half-wave; restart the between-levels count */
      crossing_done:
        rd->betweenSamples = 0;
        /* reset_half_wave: restart the per-half-wave sample counters */
      reset_half_wave:
        rd->halfWaveSamples = 0;
        rd->crossGapSamples = 0;
      }
    sample_done:
      /* a half-wave longer than one period at minFreq: back to idle */
      if (rate_minOffMs / minFreq_durMs < (int)rd->halfWaveSamples)
      {
        rd->quietSamples = rate_minOffMs / minFreq_durMs;
        rd->cycleCount = 0;
        rd->betweenSamples = 0;
        rd->halfWaveSamples = 0;
        rd->state = RING_STATE_IDLE;
        rd->riseConfirm = rd->confirmSamples;
        rd->fallConfirm = rd->confirmSamples;
      }

      /* ring ON once the counted periods last longer than minOnMs; OFF after minOffMs without a crossing */
      tmp = rd->freqAvg;
      if (((tmp != 0) && (0 < rd->cycleCount)) && (rd->minOnMs < (rd->cycleCount * 1000) / (int)tmp))
      {
        rd->ringOn = 1;
        rd->lastFreq = 0;
      }

      if ((offLimit_minOnMs < rd->quietSamples) || (offLimit_minOnMs < rd->crossGapSamples))
      {
        if (tmp != 0)
        {
          rd->lastFreq = tmp;
        }
        rd->freqAvg = 0;
        rd->freqAvgCount = 0;
        rd->ringOn = 0;
      }
      i = i + 1;
    } while (i < count);
  }

  /* an ON / OFF change since the previous call: compute the duration of the period that ended (see the function description) */
  changed = 0;
  tmp = rd->prevRingOn;
  if (tmp == rd->ringOn)
    goto store_prev_ring_on;
  if (tmp == 1)
  {
    offLimit_minOnMs = rd->minOnMs;
    rate_minOffMs = rd->minOffMs;
    minFreq_durMs = (offLimit_minOnMs - rate_minOffMs) + 0x14;
  }
  else
  {
    rate_minOffMs = rd->minOffMs;
    offLimit_minOnMs = rd->minOnMs;
    minFreq_durMs = (rate_minOffMs - offLimit_minOnMs) + -0x14;
  }
  minFreq_durMs = (rd->phaseSamples * 1000) / rd->sampleRate + minFreq_durMs;

  if (tmp == 1)
  {
    if (offLimit_minOnMs <= minFreq_durMs)
      goto store_duration;
    rd->lastDurationMs = offLimit_minOnMs;
  }
  else if (minFreq_durMs < rate_minOffMs)
  {
    rd->lastDurationMs = rate_minOffMs;
  }
  else
  {
  store_duration:
    rd->lastDurationMs = minFreq_durMs;
  }
  rd->phaseSamples = 0;
  changed = 1;

store_prev_ring_on:
  rd->prevRingOn = rd->ringOn;
  return changed;
}

/**
 * @brief Report the frequency and duration recorded at the last ON / OFF change.
 * @param rd          detector
 * @param freq        out: lastFreq, Hz (0 while the ring is ON)
 * @param durationMs  out: lastDurationMs, the duration of the ON or OFF period that ended, ms
 */
void RingDetector_GetLastRing(RingDetector* rd, int* freq, int* durationMs)
{
  *freq = (int)rd->lastFreq;
  *durationMs = rd->lastDurationMs;
}
