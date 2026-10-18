/**
 * @file voice_tone.h
 * @brief Types and API of the voice tone / silence / detector layer (voice_tone.c).
 *
 * - ToneObj / ToneCfg: a single-tone energy detector (TONE_create / TONE_detect / TONE_delete) built on a modulated low-pass FIR and a second-order
 *   recursion; MTK_phasor() is the table-driven sine / cosine oscillator it uses, notch() a biquad notch section (the DTMF detector uses it too).
 * - Silence: an 800-sample-window energy detector (silence_*).
 * - Detector: the voice-modem tone detector (detector_*) that owns four ToneObj, a Dtmf object and two cadence detectors; _status() writes its
 *   "DLE <code>" reports.
 * - voice_getgain(): the gains of the beep generator.
 *
 * Layouts.  Every object keeps the 32-bit image layout at -m32 (the `_Static_assert`s in voice_tone.c pin it) and uses real pointers, so on x86-64
 * every member after the first pointer moves: nothing may address a ToneObj / Detector / Silence by a raw byte offset; use the member names, the
 * handles and sizeof() (at -m32: ToneObj 0x1c8, Detector 0x38, Silence 0x14, ToneCfg 0x30).  The voice engine stores a Detector * and a Silence *
 * as opaque handles.
 *
 * Callback type VoiceParamFn: `unsigned (*)(struct modem *, int id)`, the modem context and parameter callback of the voice object.  Ids used here:
 * 0x49 (detector_create: the value times 12.5 is stored), 0x52 (silence_progress: the level 1 .. 3; 0 or above 3 disables the detector), 0x53
 * (silence length in windows), 0x18 (the duration of a '!' beep digit, voice_dtmf.c).
 *
 * Float arithmetic.  The functions compute in `long double` (x87 extended precision) and round to float only where a value is stored; see
 * voice_tone.c.  notch() RETURNS an unrounded extended value (its caller squares it).  The functions are documented at their definitions.
 */
#ifndef DSPLIBS_PORT_VOICE_TONE_H
#define DSPLIBS_PORT_VOICE_TONE_H
#include <stddef.h>
#include <stdint.h>

#include "tone/tone_detect.h" /* Cadence / CadenceParams / cadence_* */
#include "core/dsp_host.h"

struct modem;
struct Dtmf; /* the DTMF object: opaque here (create_dtmf / dtmf_progress); detector_progress reads its u16 member `busy` (offset 0x90) through DTMF_W90 */
typedef unsigned (*VoiceParamFn)(struct modem* m, int id);

/* the DTMF object's u16 `busy` at offset 0x90 (while it is non-zero detector_progress skips the four tone detectors).  The Dtmf layout is the same on
 * every host and keeps this member at 0x90; a file may define DTMF_W90 before including this header to read it another way. */
#ifndef DTMF_W90
#define DTMF_W90(d) (*(const unsigned short*)((const char*)(d) + 0x90))
#endif

/* ---- TONE: config record and object ---- */

/** Configuration of a tone detector. */
typedef struct ToneCfg
{
  float freq;       ///< centre frequency (Hz)
  float f4;         ///< (not used by the detector)
  float f8;         ///< (not used by the detector)
  float fc;         ///< detection factor: tone if B * fc < A
  float f10;        ///< (not used by the detector)
  float f14;        ///< minimum smoothed band energy B
  float f18;        ///< recursion factor (k1 = 2 cos * f18, k2 = -f18^2)
  const float* lpf; ///< low-pass taps (53 floats); a real pointer: the members after it move on x86-64
  short n;          ///< number of taps
  short n_hi;       ///< high half of the 32-bit word that holds `n` (0)
  int w24;          ///< (not used by the detector)
  float f28;        ///< (not used by the detector)
  int w2c;          ///< (not used by the detector)
} ToneCfg;

/** A tone detector. */
typedef struct ToneObj
{
  ToneCfg cfg; ///< copy of the configuration
  float w30;   ///< 0
  float ph;    ///< phase step of the oscillator (cfg.freq * pi / 4000, normalised by MTK_phasor)
  float w38;   ///< 0
  float* coef; ///< n floats: 2 * cos(phase_i) * lpf[i] (the FIR taps)
  float* ring; ///< n floats: the input history ring
  short idx;   ///< ring index of the newest sample
  short pad46;
  float k0, k1, k2; ///< recursion coefficients: -2 cos(ph), -k0 * f18, -f18^2
  float s0, s1;     ///< recursion state
  float accA, accB; ///< smoothed detection energies A (difference) and B (total)
  short w64;        ///< 0
  short pad66;
  int w68, w6c; ///< 0
  int z70[80];  ///< 0
  short w1b0;   ///< 0
  short pad1b2;
  float* b14;     ///< 5 floats (1.0 at [1] and [4], ...); written by TONE_create(), read by nothing
  float* b8;      ///< 2 floats (0, 0); written by TONE_create(), read by nothing
  float* selfk;   ///< &k0
  int w1c0, w1c4; ///< 0
} ToneObj;

/* TONE_create / TONE_detect / TONE_delete: documented at the definitions in voice_tone.c */
extern ToneObj* TONE_create(ToneObj* obj, const ToneCfg* cfg);
extern int TONE_detect(ToneObj* obj, const float* in, short count);
extern void TONE_delete(ToneObj* obj);
extern void MTK_phasor(float* p);                                   /* p = { phase, cos (out), sin (out), step }; see voice_tone.c */
extern long double notch(float x, float* state, const float* coef); /* state[2], coef[4]; returns y, unrounded */
extern const ToneCfg TONE_CFG;
extern const ToneCfg tone_amode_cfg; /* the configuration of the four tone detectors of the detector object (voice_tone_tables.c) */
extern const float MTK_sin_table[257], MTK_cos_table[257], MTK_sin_sign[4], MTK_cos_sign[4];

/* ---- silence detector ---- */

/** The silence detector. */
typedef struct Silence
{
  struct modem* ctx;    ///< first argument of the callback
  VoiceParamFn cb;      ///< parameter callback
  unsigned short count; ///< windows since the last sound (the "silence counter")
  unsigned short n;     ///< samples in the current 800-sample window
  unsigned short heard; ///< 1 once a window above the threshold was seen
  unsigned short pad0e;
  float energy; ///< energy accumulated in the window
} Silence;

/* the silence API: documented at the definitions in voice_tone.c */
extern Silence* silence_create(Silence* obj, struct modem* ctx, VoiceParamFn cb);
extern void silence_delete(Silence* obj);
extern void silence_progress(Silence* obj, const float* x, short count, unsigned char* buf, unsigned short* len);
extern int silence_is_more_then(Silence* obj, float v);

/* ---- detector ---- */

/** The voice-modem tone detector. */
typedef struct Detector
{
  unsigned short enable; ///< bit 0 DTMF, bits 1 .. 4 the four tone detectors, bit 5 the two cadences; 0x3f at create
  unsigned short pad02;
  struct Dtmf* dtmf;     ///< DTMF detector (freed with sysdep_free() by detector_delete())
  Cadence* cad8;         ///< NULL (deleted if non-NULL)
  Cadence* cadBusy;      ///< busy-tone cadence detector
  Cadence* cadDial;      ///< dial-tone cadence detector
  ToneObj* tone[4];      ///< 1296, 1100, 2100 and 2225 Hz ('e', 'c', 'a', 'f')
  unsigned short cnt[4]; ///< consecutive detecting blocks of each tone
  int w2c;               ///< 0
  unsigned w30;          ///< cb(ctx, 0x49) * 12.5 (0 without a callback); not read
  int out_mode;          ///< 1 = write "DLE <char>" into the stream (default), 0 = return the status code
} Detector;

/* the detector API: documented at the definitions in voice_tone.c */
extern Detector* detector_create(Detector* obj, struct modem* ctx, VoiceParamFn cb);
extern void detector_delete(Detector* d);
extern int detector_progress(Detector* d, const float* x, short count, unsigned char* buf, unsigned short* len);
extern void detector_set_enable(Detector* d, unsigned short v);
extern void detector_set_output_status(Detector* d);
extern void detector_set_output_in_stream(Detector* d);
extern void _status(unsigned char* buf, unsigned short* len, int code); /* *len += 2; buf[0] = 0x10; buf[1] = code ("DLE <code>") */

/* the beep generator's gains.  `beep` is any object whose FIRST member is the struct modem *.
 * out_a = F(0.276 * 10^(((6 - highLevel) * 0.05) - beepAtten * 0.05)), out_b = F(10^(-levelDiff * 0.05) * out_a) with highLevel / levelDiff / beepAtten
 * the host parameters GetDTMFHighToneLevel / GetDTMFHighAndLowToneLevelDifference / GetAdditAttenToBeepgenVoice; 10^x is dsplibs_pow(), not libm's pow. */
struct BeepGen;
extern void voice_getgain(struct BeepGen* beep, float* out_b, float* out_a);
extern long double dsplibs_pow(double x, double y); /* the library's own x87 power function (returns an extended value); only voice_getgain() calls it */

/* create_dtmf / dtmf_progress: voice_dtmf.h */
#endif
