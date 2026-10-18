/**
 * @file voice.h
 * @brief Voice mode: the glue object slmodemd drives, the voice engine behind it, and their types.
 *
 * Two layers.  The glue (voice_api.c) is the four entry points slmodemd links: VOICE_create(), VOICE_process(), VOICE_command(), VOICE_delete().
 * The engine (voice_engine.c) is what the glue drives: voice_create() / voice_delete() / voice_command() / voice_modem(), the four state handlers
 * voice_rx() / voice_tx() / voice_online() / voice_duplex(), voice_dle_command() and the mode switches voice_set_*().  The glue also owns the three
 * callbacks the engine hands to its tone, silence and beep generators (vce_get_sreg(), vce_hook_on(), vce_hook_off(); static in voice_api.c).
 *
 * Objects (both hold real pointers; on x86-64 every member behind the first pointer moves, so nothing may address them by a raw byte offset.  The
 * 32-bit layouts are pinned by the `_Static_assert`s at the end of this file):
 * - VoiceGlue: what VOICE_create() returns to slmodemd as an opaque `void *`.  It holds the modem, the Voice engine, slmodemd's `struct voice_info`
 *   (the host parameter MDMPRM_VOICEINFO), the glue state machine (command / receive / transmit / duplex, the last engine message, the number of
 *   tty bytes the engine wants), the two RcFixed converters between the line rate and 8000 Hz (NULL at 8000 Hz), the tty and output byte buffers,
 *   the 160-sample staging buffers and two 384-sample rings.  The block length is srate / 50 samples.
 * - Voice: the engine.  It holds the configuration (modem, parameter callback, hook callbacks), the requested and the last mode, the state handler
 *   `fn` of the current mode, the handles of the beep generator, the tone / DTMF detector, the transmit FIFO, the silence detector and the FDSP
 *   object, a byte / float staging area shared by the transmit and receive handlers, and the small state words of the DLE protocol.
 *
 * Float arithmetic.  voice_rx(), voice_tx(), voice_online(), voice_set_rx() and VOICE_process() compute with x87 semantics: intermediates are
 * `long double`, a value is rounded to float only where it is stored (F() of voice_f32.h), and every float-to-integer conversion truncates and
 * yields the "integer indefinite" for NaN and out-of-range values (fist16() / fist32()).  See voice_engine.c.
 */
#ifndef DSPLIBS_PORT_VOICE_OBJ_H
#define DSPLIBS_PORT_VOICE_OBJ_H
#include <stddef.h>
#include <stdint.h>

#include "core/host_interface.h" /* struct voice_info, enum VOICE_CMD, VOICE_STATUS_* */
#include "voice_fdsp.h"
#include "core/rcfixed_resampler.h"
#include "voice_dtmf.h" /* BeepGen, BeepCfg, BeepHookFn, voice_tone.h (Detector, Silence, VoiceParamFn) */
#include "core/dsp_host.h"

typedef struct Voice Voice;
/**
 * The state handler of a mode (voice_rx / voice_tx / voice_online / voice_duplex), called through Voice::fn by voice_modem().
 * Arguments: the engine, the tty bytes or pcm shorts coming in, the float output, the float input, the byte / short output, the input byte count
 * (in) / byte space still wanted (out), the sample count (in) / output bytes (out).  It returns a VoiceMsg.
 */
typedef int (*VoiceFn)(Voice* v, void* in, float* out_f, float* in_f, void* out_b, unsigned short* inCount, unsigned short* sampleCount);

/**
 * The engine mode (Voice::mode / mode_prev, the argument of the VOICE_ECMD_SET_MODE request), one per state handler.  NONE is the value mode_prev has
 * until the first voice_modem() call and what VOICE_create() sends as the initial request: it matches no mode, so the engine stays online.
 */
enum VoiceMode
{
  VOICE_MODE_RX = 0,
  VOICE_MODE_TX = 1,
  VOICE_MODE_ONLINE = 2,
  VOICE_MODE_DUPLEX = 3,
  VOICE_MODE_NONE = 4
};

/**
 * The engine message: what the state handlers, voice_modem() and voice_command() return and what VOICE_process() acts on (state changes, status
 * for the caller, "DLE <char>" reports to the tty).  The glue prints them as "VOICE_NO_MESSAGE", "VOICE_OK", ... "Underrun" at debug level.
 * VOICE_MSG_PURGE has no producer in the engine.
 */
enum VoiceMsg
{
  VOICE_MSG_NONE = 0,
  VOICE_MSG_OK = 1,
  VOICE_MSG_START_ONLINE = 2,
  VOICE_MSG_START_TX = 3,
  VOICE_MSG_START_RX = 4,
  VOICE_MSG_START_DUPLEX = 5,
  VOICE_MSG_PURGE = 6,
  VOICE_MSG_ERROR = 7,
  VOICE_MSG_START_ONLINE_AFTER_ABORT = 8,
  VOICE_MSG_CANCEL = 9,
  VOICE_MSG_BUSY = 10,
  VOICE_MSG_DIALTONE = 11,
  VOICE_MSG_FAX_TONE = 12,
  VOICE_MSG_UNDERRUN = 13
};

/**
 * The requests of voice_command(v, cmd, args); VOICE_command() maps slmodemd's VOICE_CMD_* to BEEP, DTMF, SET_MODE and ABORT, and VOICE_create() sends
 * VLS and SET_MODE itself.  DETECT_ENABLE (5) packs the three detector enable masks (transmit, receive, online) into the bytes of args[0].
 * TX_LEVEL only logs.  Request 4 and everything above 10 are unknown and answer VOICE_MSG_ERROR.
 */
enum VoiceEngCmd
{
  VOICE_ECMD_BEEP = 0,
  VOICE_ECMD_DTMF = 1,
  VOICE_ECMD_SET_MODE = 2,
  VOICE_ECMD_TX_LEVEL = 3,
  VOICE_ECMD_DETECT_ENABLE = 5,
  VOICE_ECMD_PLAYBACK_VOLUME = 6,
  VOICE_ECMD_TIME_MARK = 7,
  VOICE_ECMD_VLS = 8,
  VOICE_ECMD_ABORT = 9,
  VOICE_ECMD_RESET_DUPLEX = 10
};

/** VoiceGlue::state: what the glue does with the tty and the pcm stream (the same numbers as slmodemd's VOICE_STATE_COMMAND / RX / TX / DUPLEX). */
enum VoiceGlueState
{
  VGLUE_COMMAND = 0,
  VGLUE_RX = 1,
  VGLUE_TX = 2,
  VGLUE_DUPLEX = 3
};

/** The configuration VOICE_create() hands to voice_create(): the modem, the parameter / S-register callback and the two hook callbacks. */
typedef struct VoiceCfg
{
  struct modem* ctx;
  VoiceParamFn get_sreg;
  BeepHookFn hook_on;
  BeepHookFn hook_off;
} VoiceCfg;

/** Size in bytes of Voice::area, the byte / float staging area shared by the transmit and receive handlers. */
#define VOICE_AREA 0x708
/** Byte offset of the float buffer inside Voice::area; the 0xc8 bytes before it are the transmit data buffer. */
#define VOICE_FBUF_OFF 0xc8
/** Number of floats of the float buffer (400; the handlers use 200 of them). */
#define VOICE_NF ((VOICE_AREA - VOICE_FBUF_OFF) / 4)

/**
 * The voice engine.  The members are in the order of the 32-bit image; the `_Static_assert`s at the end of the file pin the offsets.
 * @warning area.b (transmit data bytes) and area.fl.f (float staging buffer) overlap by design: bytes 0xc8 and above of the first are the first floats of
 *          the second.  voice_tx() can read up to 0xde bytes into the transmit buffer (only when `rate` is not 8000, which nothing sets today).
 */
struct Voice
{
  struct modem* ctx;   ///< the modem, first argument of the callbacks
  VoiceParamFn cb;     ///< modem parameter / S-register callback (vce_get_sreg of voice_api.c)
  BeepHookFn hook_on;  ///< beep generator hook-on callback
  BeepHookFn hook_off; ///< beep generator hook-off callback
  int mode;            ///< requested mode, a VoiceMode (rx, tx, online, duplex)
  int mode_prev;       ///< the mode of the last voice_modem() call (VOICE_MODE_NONE until the first one)
  BeepGen* beep;       ///< beep / DTMF tone generator
  Detector* det;       ///< tone, DTMF and cadence detector
  VoiceFn fn;          ///< state handler of the current mode
  FifoObj* fifo;       ///< transmit byte FIFO (default size, 300 bytes)
  Silence* sil;        ///< silence detector of the receive path
  int w2c;             ///< (never accessed)
  int w30;             ///< (never accessed)
  FdspObj* fdsp;       ///< duplex pass-through object
  union
  {
    unsigned char b[VOICE_AREA]; ///< transmit data bytes
    struct
    {
      unsigned char pad[VOICE_FBUF_OFF];
      float f[VOICE_NF]; ///< float staging buffer (200 floats used; voice_tx()'s 0xde-byte read can spill into its start)
    } fl;
  } area;
  int beep_idle;           ///< 1 = no beep / DTMF tone is being generated
  int etx;                 ///< DLE ETX seen while transmitting
  int abort;               ///< DLE CAN / abort requested
  int vls;                 ///< #VLS line select: 1 and 2 = pcm to / from the tty, 3 = pcm from the tty (receive), others = float line audio
  int w750;                ///< (never accessed)
  unsigned short playvol;  ///< playback volume of the last VOICE_ECMD_PLAYBACK_VOLUME request (stored, never read)
  unsigned short rx_on;    ///< receive running
  unsigned short rate;     ///< sample rate of the stream, 8000
  unsigned short bits;     ///< bits per sample of the stream, 8
  unsigned short underrun; ///< 1 = the transmit FIFO has run dry (an underrun was reported or the mode just started)
  unsigned short en_tx;    ///< detector enable mask used while transmitting
  unsigned short en_rx;    ///< detector enable mask used while receiving
  unsigned short en_on;    ///< detector enable mask used while online
  unsigned short tmark;    ///< silence time mark in seconds (0 = off)
  unsigned short tcount;   ///< samples left until the next "DLE T" time mark (tmark * 800 after each mark)
  unsigned char pad768[0x60];
  int first; ///< 1 = the next receive block starts the DC estimate
  float dc;  ///< running mean of the receive input
  /**
   * Receive gains, the callback ids 0x48 / 0x8b / 0x8a divided by 128: voice_rx() scales the pcm of #VLS 1 by rxGainVls1 and of #VLS 3 by rxGainVls3,
   * and the float line input (every other #VLS) by rxGainLine.  (Named from these three uses; the callback ids have no symbolic names.)
   */
  float rxGainVls1, rxGainLine, rxGainVls3;
};

/** Capacity of a glue ring in samples: two blocks of at most 192 samples. */
#define VRING 384
/** A ring of the glue (the same shape as the data-pump wrapper's): the input ring fills at `wr`, the output ring drains at `rd`. */
typedef struct VoiceRing
{
  unsigned count; ///< samples held
  unsigned wr;    ///< write index (input ring)
  unsigned rd;    ///< read index (output ring)
  unsigned half;  ///< start of the block being processed: 0 or the block length
  short data[VRING];
} VoiceRing;

/** The glue object VOICE_create() returns to slmodemd. */
typedef struct VoiceGlue
{
  struct modem* modem;         ///< the modem
  Voice* voice;                ///< the engine
  const struct voice_info* vi; ///< slmodemd's voice parameters (host parameter MDMPRM_VOICEINFO)
  int state;                   ///< a VoiceGlueState
  int last;                    ///< the last engine message acted on
  int cnt;                     ///< tty bytes the engine wants next (transmit) / the last input count
  RcFixed* rcIn;               ///< line rate -> 8000 Hz converter (NULL at 8000 Hz)
  RcFixed* rcOut;              ///< 8000 Hz -> line rate converter (NULL at 8000 Hz)
  unsigned char tty[0x400];    ///< bytes read from the tty
  unsigned char outb[0x400];   ///< bytes the engine produced for the tty
  short sbuf[0xa0];            ///< 8000 Hz 16-bit samples (converter output / input)
  float fin[160];              ///< engine input block
  float fout[160];             ///< engine output block
  unsigned blk;                ///< samples per block at the line rate = srate / 50
  VoiceRing out;               ///< output ring
  VoiceRing in;                ///< input ring
} VoiceGlue;

/**
 * modem_get_param() under another C name (the same prototype as core/dsp_host.h): the glue uses it to fetch the voice_info POINTER (parameter
 * MDMPRM_VOICEINFO) from the `long` result.
 */
extern long voice_get_param(struct modem* m, unsigned param) __asm__("modem_get_param");

/* ---- the entry points slmodemd links (documented at their definitions in voice_api.c) ---- */
extern void* VOICE_create(struct modem* m, unsigned srate);
extern void VOICE_delete(void* obj);
extern int VOICE_process(void* obj, void* in, void* out, int count);
extern int VOICE_command(void* obj, enum VOICE_CMD cmd);

/* ---- the engine (documented at their definitions in voice_engine.c) ---- */
extern Voice* voice_create(const VoiceCfg* cfg);
extern void voice_delete(Voice* v);
extern int voice_command(Voice* v, int cmd, const int* args);
extern int voice_modem(Voice* v, void* inRaw, float* outFloat, float* inFloat, void* outRaw, unsigned short* inCount, unsigned short* sampleCount);
extern int voice_online(Voice* v, void* inRaw, float* outFloat, float* inFloat, void* outRaw, unsigned short* inCount, unsigned short* sampleCount);
extern int voice_tx(Voice* v, void* inRaw, float* outFloat, float* inFloat, void* outRaw, unsigned short* inCount, unsigned short* sampleCount);
extern int voice_rx(Voice* v, void* inRaw, float* outFloat, float* inFloat, void* outRaw, unsigned short* inCount, unsigned short* sampleCount);
extern int voice_duplex(Voice* v, void* inRaw, float* outFloat, float* inFloat, void* outRaw, unsigned short* inCount, unsigned short* sampleCount);
extern int voice_dle_command(Voice* v, int cmd);
extern void voice_set_rx(Voice* v);
extern void voice_set_tx(Voice* v);
extern void voice_set_online(Voice* v);
extern void voice_set_duplex(Voice* v);
extern int voice_linear2ulaw(int pcm); /* the voice engine's own mu-law encoder (the library's separate linear2ulaw lives in src/vpcm) */

#ifdef __i386__
_Static_assert(sizeof(Voice) == 0x7dc, "Voice");
_Static_assert(offsetof(Voice, cb) == 0x04 && offsetof(Voice, hook_on) == 0x08 && offsetof(Voice, hook_off) == 0x0c && offsetof(Voice, mode) == 0x10 &&
                   offsetof(Voice, mode_prev) == 0x14,
               "Voice");
_Static_assert(offsetof(Voice, beep) == 0x18 && offsetof(Voice, det) == 0x1c && offsetof(Voice, fn) == 0x20 && offsetof(Voice, fifo) == 0x24 &&
                   offsetof(Voice, sil) == 0x28 && offsetof(Voice, fdsp) == 0x34,
               "Voice");
_Static_assert(offsetof(Voice, area) == 0x38 && offsetof(Voice, area.fl.f) == 0x100 && offsetof(Voice, beep_idle) == 0x740 && offsetof(Voice, etx) == 0x744 &&
                   offsetof(Voice, abort) == 0x748 && offsetof(Voice, vls) == 0x74c,
               "Voice");
_Static_assert(offsetof(Voice, playvol) == 0x754 && offsetof(Voice, rx_on) == 0x756 && offsetof(Voice, rate) == 0x758 && offsetof(Voice, bits) == 0x75a &&
                   offsetof(Voice, underrun) == 0x75c,
               "Voice");
_Static_assert(offsetof(Voice, en_tx) == 0x75e && offsetof(Voice, en_rx) == 0x760 && offsetof(Voice, en_on) == 0x762 && offsetof(Voice, tmark) == 0x764 &&
                   offsetof(Voice, tcount) == 0x766,
               "Voice");
_Static_assert(offsetof(Voice, first) == 0x7c8 && offsetof(Voice, dc) == 0x7cc && offsetof(Voice, rxGainVls1) == 0x7d0 &&
                   offsetof(Voice, rxGainLine) == 0x7d4 && offsetof(Voice, rxGainVls3) == 0x7d8,
               "Voice");
_Static_assert(sizeof(VoiceGlue) == 0x1484 && sizeof(VoiceRing) == 0x310 && sizeof(VoiceCfg) == 0x10, "VoiceGlue");
_Static_assert(offsetof(VoiceGlue, voice) == 4 && offsetof(VoiceGlue, vi) == 8 && offsetof(VoiceGlue, state) == 0xc && offsetof(VoiceGlue, last) == 0x10 &&
                   offsetof(VoiceGlue, cnt) == 0x14,
               "VoiceGlue");
_Static_assert(offsetof(VoiceGlue, rcIn) == 0x18 && offsetof(VoiceGlue, rcOut) == 0x1c && offsetof(VoiceGlue, tty) == 0x20 &&
                   offsetof(VoiceGlue, outb) == 0x420 && offsetof(VoiceGlue, sbuf) == 0x820,
               "VoiceGlue");
_Static_assert(offsetof(VoiceGlue, fin) == 0x960 && offsetof(VoiceGlue, fout) == 0xbe0 && offsetof(VoiceGlue, blk) == 0xe60 &&
                   offsetof(VoiceGlue, out) == 0xe64 && offsetof(VoiceGlue, in) == 0x1174,
               "VoiceGlue");
#endif
#endif
