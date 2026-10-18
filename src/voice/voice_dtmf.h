/**
 * @file voice_dtmf.h
 * @brief Types and API of the voice DTMF detector and of the beep / DTMF tone generator.
 *
 * - Dtmf: the detector (create_dtmf / dtmf_progress / dtmf_detect / dtmf_test).  It holds no pointer, so its layout (0x98 bytes, the u16 `busy` at
 *   0x90) is the same on every host; detector_progress() in voice_tone.c reads `busy` through DTMF_W90.
 * - BeepGen: the beep / DTMF generator (beepgen_create / delete / start_beep / start_dtmf / sample).  It keeps the 32-bit image layout at -m32 (the
 *   `_Static_assert`s in voice_dtmf.c pin it) and uses real pointers (the owner's context and the three callbacks), so on x86-64 every member after
 *   the first pointer moves; use the member names and sizeof(), never a raw offset.  The voice engine stores a BeepGen * and calls beepgen_sample()
 *   once per output sample.
 *
 * The arithmetic follows the policy described in voice_dtmf.c (x87 extended intermediates, float rounding at the stores).  The functions are
 * documented at their definitions in voice_dtmf.c.
 */
#ifndef DSPLIBS_PORT_VOICE_DTMF_H
#define DSPLIBS_PORT_VOICE_DTMF_H
#include <stddef.h>
#include <stdint.h>

#include "voice_tone.h" /* VoiceParamFn, voice_getgain, notch, dsplibs_debug_*, sysdep_*, DTMF_W90 */

/** The DTMF detector object: 0x98 bytes, no pointers (the same layout on every host). */
typedef struct Dtmf
{
  float st[8][2];         ///< the eight notch filters' states (notch(x, st[i], coef + 4 * i))
  float e[8];             ///< accumulated output energy of each notch filter (the tone's filter has the minimum)
  float eb;               ///< accumulated input energy
  unsigned short hist[8]; ///< the last eight dtmf_test() results (USHRT_MAX = none), newest first
  unsigned short cnt;     ///< evaluated samples in this block (40, mode 1: 45, make a block)
  unsigned short sub;     ///< sample decimation counter (every second sample is processed)
  float bias[2];          ///< state of the mode 1 pre-filter notch (coefficients biascoef)
  unsigned char w80[16];  ///< never written or read (create_dtmf() does not initialise them)
  unsigned short busy;    ///< a digit is being held (detector_progress() skips the tone detectors; it reads this member through DTMF_W90)
  unsigned short code;    ///< the held digit (USHRT_MAX = none)
  unsigned short easy;    ///< relaxed validation flag (nothing sets it)
  unsigned short pad96;
} Dtmf;

/* the detector API (documented at the definitions in voice_dtmf.c) */
extern Dtmf* create_dtmf(Dtmf* obj);
extern short dtmf_progress(Dtmf* d, const float* x, short count, int mode);
extern int dtmf_detect(float x, Dtmf* d, short mode);
extern int dtmf_test(const float* e, float eb, short mode);
extern const float biascoef[4], us_coef[32], eur_coef[32];

/* ---- beep / DTMF generator ---- */

/** Capacity of the beep queue. */
#define BEEP_QMAX 20
/** A hook callback of the beep generator (called with the owner's context). */
typedef void (*BeepHookFn)(struct modem* m);
/** One queued tone. */
typedef struct BeepEntry
{
  float f1; ///< first (high-group) frequency in Hz; 0 = silent, -1 = hook marker (both gains 0 and the hook callbacks are called)
  float f2; ///< second (low-group) frequency in Hz
  int dur;  ///< duration (the entry lasts dur * 800 samples; dur * 80 for the hook marker)
} BeepEntry;
/** The configuration record beepgen_create() copies from. */
typedef struct BeepCfg
{
  struct modem* ctx;
  BeepHookFn hookOn;     ///< called when an entry with f1 == -1 (the hook marker) starts (VoiceCfg::hook_on)
  BeepHookFn hookOff;    ///< called when an entry whose f1 is -1 or NaN ends
  VoiceParamFn getParam; ///< cb(ctx, 0x18): the duration of a '!' digit
} BeepCfg;
/** The beep / DTMF generator. */
typedef struct BeepGen
{
  struct modem* ctx;      ///< the owner's context; voice_getgain() reads the level parameters from it
  float phase1;           ///< oscillator phase of f1
  float phase2;           ///< oscillator phase of f2
  float f1;               ///< current frequency 1
  float f2;               ///< current frequency 2
  float gain1;            ///< gain of the f1 oscillator
  float gain2;            ///< gain of the f2 oscillator
  int n;                  ///< samples produced in this entry
  int limit;              ///< samples of this entry = dur * 8000 / div
  int count;              ///< queued entries (the one playing is q[cur])
  int cur;                ///< index of the playing entry
  BeepEntry q[BEEP_QMAX]; ///< the queue
  BeepHookFn hookOn;      ///< hook-on callback
  BeepHookFn hookOff;     ///< hook-off callback
  VoiceParamFn getParam;  ///< parameter callback
  int div;                ///< time base: 10 (tone) or 100 (hook marker)
} BeepGen;

/* the generator API (documented at the definitions in voice_dtmf.c) */
extern BeepGen* beepgen_create(BeepGen* obj, const BeepCfg* cfg);
extern void beepgen_delete(BeepGen* b);
extern void beepgen_start_beep(BeepGen* b, int f1, int f2, int dur);
extern void beepgen_start_dtmf(BeepGen* b, int ch, int dur);
extern int beepgen_sample(BeepGen* b, float* out);
#endif
