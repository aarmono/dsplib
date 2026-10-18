/**
 * @file ring_detector.h
 * @brief Soft ring detector: state object and public interface.
 *
 * The detector watches the line samples for a low-frequency, high-amplitude ring signal and reports when a ring starts and when it ends,
 * together with the ring frequency and the duration of the period that just ended.  It is owned by the RD glue object of core/dp_runtime.c
 * (RD_create / RD_process / RD_delete / RD_ring_details), which is what the host calls; nothing else uses it.
 *
 * Entry points (ring_detector.c): RingDetector_Create(), RingDetector_Delete(), RingDetector_Process(), RingDetector_GetLastRing().
 *
 * Algorithm.  `state` follows the signal: 1 = idle (no ring signal), 2 = in a positive half-wave, 3 = in a negative half-wave.  A threshold
 * crossing counts only after more than `riseConfirm` (upwards) / `fallConfirm` (downwards) consecutive samples beyond the level.  At the start of
 * a burst the two half-wave levels (`upperLevel`, `lowerLevel`) are set from the sign of the first crossing; one full period is closed by the
 * crossing direction whose confirm count is still `confirmSamples`, and each period whose frequency lies in [minFreq, maxFreq] updates the
 * running frequency mean `freqAvg`.  The ring is ON once the counted periods last longer than `minOnMs`, and OFF again once no crossing has been
 * seen for `minOffMs`.  RingDetector_Process() returns 1 on each ON / OFF change; RingDetector_GetLastRing() then reports the ring frequency and
 * the duration of the period (on or off) that just ended.
 *
 * The object is private to the detector, holds no pointers and has the same size and member offsets on 32-bit and 64-bit hosts (asserted below).
 */
#ifndef RINGDETECTOR_H
#define RINGDETECTOR_H

#include <stddef.h>

/** Values of RingDetector::state: where the detector is in the ring waveform. */
enum RingState
{
  RING_STATE_IDLE = 1,     ///< no ring signal: waiting for a confirmed crossing of +threshold or -threshold
  RING_STATE_POSITIVE = 2, ///< in a positive half-wave: waiting for a confirmed fall below lowerLevel
  RING_STATE_NEGATIVE = 3  ///< in a negative half-wave: waiting for a confirmed rise above upperLevel
};

/** Ring detector state.  Create with RingDetector_Create(), release with RingDetector_Delete(). */
typedef struct RingDetector
{
  int minFreq;             ///< lowest accepted ring frequency, Hz (from the configuration, raised to at least 14)
  int maxFreq;             ///< highest accepted ring frequency, Hz (from the configuration, lowered to at most 100)
  int minOnMs;             ///< minimum ring-on duration, ms (from the configuration, raised to at least 40)
  int minOffMs;            ///< minimum ring-off duration, ms (from the configuration, raised to at least 120)
  int sampleRate;          ///< sample rate, Hz
  int phaseSamples;        ///< samples since the last reported ON / OFF change
  int periodSamples;       ///< samples of the signal period being measured (reset when a period is closed)
  int quietSamples;        ///< idle samples without a confirmed crossing (preset when the detector falls back to idle)
  int crossGapSamples;     ///< samples since the last confirmed crossing (compared with the ring-off limit)
  int lastDurationMs;      ///< duration of the ON or OFF period that just ended, ms (reported by RingDetector_GetLastRing())
  int freqAvgCount;        ///< number of periods averaged into freqAvg
  short freqAvg;           ///< running mean of the measured period frequencies, Hz (0 = none yet)
  short lastFreq;          ///< frequency of the last ring, Hz (reported by RingDetector_GetLastRing())
  short confirmSamples;    ///< crossing confirm count: |threshold| / (sampleRate / 80)
  short confirmAfterStart; ///< confirm count of the direction that did NOT start the burst: 2 (threshold >= 0) or 0
  short releaseLevel;      ///< the half-wave level on the near side of zero: 100 (threshold >= 0) or 200
  short threshold;         ///< |configured threshold|
  short maxBetweenSamples; ///< longest stay between the two levels before falling back to idle: 3/4 of a period at the configured minFreq
  short ringOn;            ///< 1 while a ring is detected
  short prevRingOn;        ///< ringOn at the end of the previous RingDetector_Process() call
  short state;             ///< RingState: 1 idle, 2 positive half-wave, 3 negative half-wave
  short aboveCount;        ///< consecutive samples above the upper level
  short belowCount;        ///< consecutive samples below the lower level
  short upperLevel;        ///< upper half-wave level (threshold, or -releaseLevel; set by the starting crossing)
  short lowerLevel;        ///< lower half-wave level (releaseLevel, or -threshold; set by the starting crossing)
  short betweenSamples;    ///< consecutive samples between the two levels
  short cycleCount;        ///< periods counted in the current burst
  short halfWaveSamples;   ///< samples since the last confirmed crossing (compared with one period at minFreq)
  short riseConfirm;       ///< confirm count for an upward crossing (confirmSamples or confirmAfterStart)
  short fallConfirm;       ///< confirm count for a downward crossing (confirmSamples or confirmAfterStart)
} RingDetector;            // sizeof 0x54 (2 bytes of tail padding)

/* The layout is fixed: size and member offsets are asserted, and the detector contains no pointer, so they are the same at 32 and 64 bits. */
_Static_assert(offsetof(RingDetector, minFreq) == 0x00 && offsetof(RingDetector, maxFreq) == 0x04 && offsetof(RingDetector, minOnMs) == 0x08 &&
                   offsetof(RingDetector, minOffMs) == 0x0c && offsetof(RingDetector, sampleRate) == 0x10 && offsetof(RingDetector, phaseSamples) == 0x14 &&
                   offsetof(RingDetector, periodSamples) == 0x18 && offsetof(RingDetector, quietSamples) == 0x1c &&
                   offsetof(RingDetector, crossGapSamples) == 0x20 && offsetof(RingDetector, lastDurationMs) == 0x24 &&
                   offsetof(RingDetector, freqAvgCount) == 0x28,
               "RingDetector int members");
_Static_assert(offsetof(RingDetector, freqAvg) == 0x2c && offsetof(RingDetector, lastFreq) == 0x2e && offsetof(RingDetector, confirmSamples) == 0x30 &&
                   offsetof(RingDetector, confirmAfterStart) == 0x32 && offsetof(RingDetector, releaseLevel) == 0x34 &&
                   offsetof(RingDetector, threshold) == 0x36 && offsetof(RingDetector, maxBetweenSamples) == 0x38 && offsetof(RingDetector, ringOn) == 0x3a &&
                   offsetof(RingDetector, prevRingOn) == 0x3c && offsetof(RingDetector, state) == 0x3e && offsetof(RingDetector, aboveCount) == 0x40 &&
                   offsetof(RingDetector, belowCount) == 0x42 && offsetof(RingDetector, upperLevel) == 0x44 && offsetof(RingDetector, lowerLevel) == 0x46 &&
                   offsetof(RingDetector, betweenSamples) == 0x48 && offsetof(RingDetector, cycleCount) == 0x4a &&
                   offsetof(RingDetector, halfWaveSamples) == 0x4c && offsetof(RingDetector, riseConfirm) == 0x4e &&
                   offsetof(RingDetector, fallConfirm) == 0x50,
               "RingDetector short members");
_Static_assert(sizeof(RingDetector) == 0x54, "RingDetector size (0x54 bytes)");

/** Indices into the int[RD_CFG_COUNT] configuration array passed to RingDetector_Create(). */
enum
{
  RD_CFG_SAMPLE_RATE = 0, ///< sample rate, Hz
  RD_CFG_MIN_FREQ = 1,    ///< lowest accepted ring frequency, Hz
  RD_CFG_MAX_FREQ = 2,    ///< highest accepted ring frequency, Hz
  RD_CFG_MIN_ON_MS = 3,   ///< minimum ring-on duration, ms
  RD_CFG_MIN_OFF_MS = 4,  ///< minimum ring-off duration, ms
  RD_CFG_THRESHOLD = 5,   ///< amplitude threshold; the sign selects the confirm / release presets (see RingDetector_Create())
  RD_CFG_COUNT = 6        ///< number of entries
};

/**
 * @brief Allocate and initialise a ring detector.
 * @param cfg  RD_CFG_COUNT ints indexed by the RD_CFG_* constants
 * @return the new detector (release with RingDetector_Delete())
 */
RingDetector* RingDetector_Create(int* cfg);

/**
 * @brief Release a detector created by RingDetector_Create().
 * @param rd  detector, or NULL (ignored)
 */
void RingDetector_Delete(RingDetector* rd);

/**
 * @brief Run the detector over a block of line samples.
 * @param rd       detector
 * @param samples  16-bit line samples
 * @param count    number of samples
 * @return 1 if the ring state changed (ON or OFF) during this call, else 0
 */
int RingDetector_Process(RingDetector* rd, short* samples, unsigned int count);

/**
 * @brief Report the ring that was last seen to start or end.
 * @param rd          detector
 * @param freq        out: ring frequency, Hz (0 while a ring is detected as ON)
 * @param durationMs  out: duration of the ON or OFF period that just ended, ms
 * @note  Meaningful after RingDetector_Process() returned 1.
 */
void RingDetector_GetLastRing(RingDetector* rd, int* freq, int* durationMs);

#endif /* RINGDETECTOR_H */
