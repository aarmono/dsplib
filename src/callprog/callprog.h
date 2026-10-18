/**
 * @file callprog.h
 * @brief Call progress: state objects and prototypes of the call-progress state machine, its helper DSP blocks and the call glue block.
 *
 * Call progress supervises the line around a dial command: it waits for dial tone, dials, and listens for ringback, busy, congestion and answer tones to
 * decide how the call ended.  The module has three layers:
 *   - dp_call.c: the data-pump glue registered with the host (DP_CALLPROG).  It owns a CallGlue block, cuts the host's sample stream into blocks of 48
 *     samples (resampling to 8000 Hz when the line rate is 9600 or 48000 Hz) and maps the result of the state machine to a host status;
 *   - callprog.c: the state machine CALLPROG_Create / _Dial / _Progress / _Delete (CallProg).  It runs the dial-tone and busy cadence detectors (tone/, see
 *     Cadence in tone_detect.h) and the answer-tone detector, counts timeouts, and drives the dialer (dialer.c);
 *   - callprog_dual_tone.c: the helper blocks it uses: the 2100 / 2250 Hz answer-tone detector (DualTone), the optional noise pre-filter (IirFilter) and the
 *     fixed-point cosine and exponential (TONE_read(), FP_Pow()) that the dialer uses to generate tones.
 * callprog_tables.c holds the coefficient tables of these helpers.  The dialer (dialer.h) generates the dial tones / pulses.
 *
 * Layout.  IirFilter and DualTone contain no pointers and have the same layout on every host.  CallProg and CallGlue hold real pointers (cadence and filter
 * handles, the host modem, the resamplers), so on a 64-bit build the members behind the first pointer move; nothing may address them by a raw byte offset
 * (the 32-bit layout is asserted at the end of this file).
 */
#ifndef DSPLIBS_PORT_CALL_OBJ_H
#define DSPLIBS_PORT_CALL_OBJ_H
#include <stddef.h>
#include <stdint.h>

#include "core/host_interface.h" /* host interface: DP_ID, DPSTAT_*, struct dp / dp_operations, MDMPRM_* / Get* ids */
#include "dialer.h"
#include "core/rcfixed_resampler.h"
#include "tone/tone_detect.h"
#include "core/dsp_host.h"
#include "core/dp_registry.h"

/**
 * Four cascaded second-order sections with a state / coefficient block of 105 shorts (callprog_dual_tone.c).  Coefficients are Q13, `scales` are right
 * shifts applied at the input and after each section.
 */
typedef struct IirFilter
{
  short w[105]; ///< delay lines (x [0 .. 11], y [25 .. 32]), coefficients (a [50 .. 61], b [75 .. 86]), shifts [100 .. 104]
  short pad;
  int nB;  ///< create's feed-forward coefficient count
  int nA1; ///< create's feedback coefficient count minus 1
} IirFilter;

/** State of the 2100 / 2250 Hz answer-tone detector (Dual_TONE_*). */
typedef struct DualTone
{
  unsigned e[3];  ///< smoothed energies: e[0] removed by the 2100 Hz notch, e[1] removed by the 1800 + 2250 Hz notch cascade, e[2] input
  short ratio;    ///< detection ratio (Create: 0xe2; the decision level is e[2] * ratio >> 8)
  short thr;      ///< minimum input energy (Create: 1)
  short g[12];    ///< delay lines of the three notch filters (4 shorts each: 2100 Hz at [0], the final 2250 Hz notch at [4], the 1800 Hz notch at [8])
  short st[3][4]; ///< delay lines of the three pre-filter biquads
  short n1;       ///< samples since the last 2100 Hz decision
  short n2;       ///< samples since the last 2250 Hz decision
} DualTone;

/** CallProg::state / nextState and the first index of the decision tables in callprog.c. */
enum CallProgState
{
  CALLPROG_ST_NO_LEGAL_STATE = 0,
  CALLPROG_ST_WAIT_DIAL = 1,
  CALLPROG_ST_DIALING = 2,
  CALLPROG_ST_WAIT_RING = 3,
  CALLPROG_ST_WAIT_TO_ANSWER = 4,
  CALLPROG_ST_ANSWER_STATE = 5,
  CALLPROG_ST_END = 6,
  CALLPROG_ST_END_PARTIALLY_STATE = 7,
  CALLPROG_ST_WFS_STATE = 8,
  CALLPROG_ST_BONGTONE_STATE = 9
};

/**
 * The messages CALLPROG_Progress() returns (and the message_due_* tables hold); the names are those of CALLPROG_Status_string().  16 and 17 have no name
 * there: the 2100 Hz or 2250 Hz detector found its tone.
 */
enum CallProgMessage
{
  CALLPROG_MSG_NO_MESSAGE = 0,
  CALLPROG_MSG_NO_RING = 1,
  CALLPROG_MSG_NO_DIAL_TONE = 2,
  CALLPROG_MSG_DIALING = 3,
  CALLPROG_MSG_END_DIALING = 4,
  CALLPROG_MSG_RINGBACK = 5,
  CALLPROG_MSG_NO_ANSWER = 6,
  CALLPROG_MSG_ANSWER = 7,
  CALLPROG_MSG_MODEM_ANSWER = 8,
  CALLPROG_MSG_VOICE_ANSWER = 9,
  CALLPROG_MSG_BUSY = 10,
  CALLPROG_MSG_CONGESTION = 11,
  CALLPROG_MSG_ERROR = 12,
  CALLPROG_MSG_ANSWER_STATE_TIMEOUT = 13,
  CALLPROG_MSG_END_DIALING_PARTIALLY = 14,
  CALLPROG_MSG_V8BIS_MODEM_ANSWER = 15,
  CALLPROG_MSG_DETECTED_2100 = 0x10, ///< the 2100 Hz detector (Dual_TONE_detect() result 3) found a tone
  CALLPROG_MSG_DETECTED_2250 = 0x11, ///< the 2250 Hz detector (result 5) found a tone
  CALLPROG_MSG_MAX_MESSAGES = 18
};

/**
 * CallProg::event and the second index of next_state_due_cptd / message_due_cptd (callprog.c): the events CALLPROG_Progress() derives from the cadence
 * detectors.  The table columns 3 .. 6 are filled by CALLPROG_Create but nothing produces those events, so they have no name here.
 */
enum CallProgEvent
{
  CALLPROG_EV_NONE = 0,
  CALLPROG_EV_DIAL_TONE = 1, ///< the dial cadence detector found a tone
  CALLPROG_EV_BUSY = 2,      ///< the busy cadence detector found a busy signal
  CALLPROG_EV_NO_ANSWER = 7  ///< the busy cadence detector reported 7 (the timer is set to expire)
};

/** Configuration passed to CALLPROG_Create(). */
typedef struct CallProgCfg
{
  int blind;                              ///< non-zero: blind dialling, no wait for dial tone
  int (*getSReg)(struct modem*, int reg); ///< reads an S-register of the host (call_GetSRegister())
  struct modem* modem;                    ///< the host's modem object
  int w0c;                                ///< copied into CallProg::w28 (not read)
} CallProgCfg;

/** The call-progress state machine (callprog.c). */
typedef struct CallProg
{
  int cfg[7]; ///< timeouts in seconds (default {10, 20, 15, 10, 8, 60, 60}): dial tone, ring, answer, answer state, line clear, silence, bong
  int blind;  ///< CallProgCfg::blind
  int (*getSReg)(struct modem*, int reg); ///< S-register reader; CALLPROG_Dial() calls it with register 0xdd for the calling-tone level
  struct modem* modem;                    ///< the host's modem object
  int w28;                                ///< CallProgCfg::w0c (not read)
  int state;                              ///< CallProgState
  int prevState;                          ///< the state at the end of the previous CALLPROG_Progress() call
  int timer;                              ///< samples (8000 Hz) until the state's timeout; 0 = expires now, negative = already expired
  int nextState;                          ///< the state requested by request_state()
  int change;                             ///< 1 once a state change was requested in this call (the first request wins)
  int lastRet;                            ///< the message returned by the previous call
  int lcCount;                            ///< samples counted for the line-clear timeout
  int lcLimit;                            ///< line-clear timeout in samples (cfg[4] * 8000)
  int lcEnable;                           ///< 1 while the line-clear timeout is armed
  int event;                              ///< CallProgEvent of the current call
  int silence;                            ///< buffers of silence counted in CALLPROG_ST_WFS_STATE
  int dialRes;                            ///< result of DialerCreate() (DIALER_ERROR_MSG: the dial string is unusable)
  int callTone;                           ///< host calling-tone flag (GetCallingToneFlag): handling of '^' in the dial string
  int callToneOn;                         ///< 1 while the calling tone is played in CALLPROG_ST_WAIT_RING
  Cadence* dialCad;                       ///< dial-tone cadence detector
  int w68;                                ///< unused
  Cadence* busyCad;                       ///< busy-tone cadence detector
  int w70;                                ///< cleared by CALLPROG_Delete()
  int w74;                                ///< unused
  IirFilter* iir;                         ///< noise pre-filter (only when applyFilter)
  int applyFilter;                        ///< host flag MustNoiseFilterBeApplied
  unsigned char dialSeen;                 ///< set when the dial cadence detector has found dial tone; stops the pre-filter
  DualTone* dual;                         ///< 2100 / 2250 Hz detector
  CallingTone ct;                         ///< calling-tone generator
  Dialer dialer;                          ///< the dialer
} CallProg;

#define CALLPROG_DIALER_OFF offsetof(CallProg, dialer)

/** A byte ring of the call glue: two halves of 0x60 bytes (48 samples). The two rings use their index members differently. */
typedef struct CallRing
{
  int cnt; ///< bytes in the ring
  int i1;  ///< input ring: write index; output ring: unused
  int i2;  ///< input ring: unused; output ring: read index
  int i3;  ///< input ring: read index; output ring: write index
  unsigned char data[0xc0];
} CallRing;

/** The data-pump block of the call-progress glue (dp_call.c). */
typedef struct CallGlue
{
  struct dp dp;        ///< the host's data-pump object (first: dp_data points back at this block)
  RcFixed* rcIn;       ///< line rate -> 8000 Hz resampler (NULL at 8000 Hz and for unsupported rates)
  RcFixed* rcOut;      ///< 8000 Hz -> line rate resampler
  short stageIn[160];  ///< resampled input block
  short stageOut[160]; ///< output block before the back-resampling
  CallRing outRing;    ///< what the state machine produced, waiting to be returned to the host
  CallRing inRing;     ///< what the host delivered, waiting to be processed
  int ended;           ///< 1 once the state machine reported a final message: only silence is produced from then on
  int status;          ///< the last CALLPROG_Progress() result
  CallProg cp;         ///< the state machine
  CallPulse pulse;     ///< pulse-dial state used by the dialer's pulse helpers (reached through the host's dp pointer)
} CallGlue;

#if UINTPTR_MAX == 0xffffffffu
_Static_assert(sizeof(IirFilter) == 0xdc && offsetof(IirFilter, nB) == 0xd4 && offsetof(IirFilter, nA1) == 0xd8, "IirFilter");
_Static_assert(sizeof(DualTone) == 0x44 && offsetof(DualTone, ratio) == 0xc && offsetof(DualTone, thr) == 0xe && offsetof(DualTone, g) == 0x10 &&
                   offsetof(DualTone, st) == 0x28 && offsetof(DualTone, n1) == 0x40 && offsetof(DualTone, n2) == 0x42,
               "DualTone");
_Static_assert(sizeof(CallProgCfg) == 0x10, "CallProgCfg");
_Static_assert(offsetof(CallProg, blind) == 0x1c && offsetof(CallProg, getSReg) == 0x20 && offsetof(CallProg, modem) == 0x24 &&
                   offsetof(CallProg, state) == 0x2c && offsetof(CallProg, timer) == 0x34 && offsetof(CallProg, change) == 0x3c &&
                   offsetof(CallProg, lcEnable) == 0x4c && offsetof(CallProg, dialRes) == 0x58 && offsetof(CallProg, callToneOn) == 0x60 &&
                   offsetof(CallProg, dialCad) == 0x64 && offsetof(CallProg, busyCad) == 0x6c && offsetof(CallProg, iir) == 0x78 &&
                   offsetof(CallProg, dialSeen) == 0x80 && offsetof(CallProg, dual) == 0x84 && offsetof(CallProg, ct) == 0x88 &&
                   offsetof(CallProg, dialer) == 0x98 && sizeof(CallProg) == 0x170,
               "CallProg");
_Static_assert(offsetof(CallGlue, rcIn) == 0x14 && offsetof(CallGlue, stageIn) == 0x1c && offsetof(CallGlue, stageOut) == 0x15c &&
                   offsetof(CallGlue, outRing) == 0x29c && offsetof(CallGlue, inRing) == 0x36c && offsetof(CallGlue, ended) == 0x43c &&
                   offsetof(CallGlue, status) == 0x440 && offsetof(CallGlue, cp) == 0x444 && offsetof(CallGlue, pulse) == 0x5b4 && sizeof(CallGlue) == 0x5cc &&
                   sizeof(CallRing) == 0xd0,
               "CallGlue");
#endif

/* callprog_dual_tone.c */
int TONE_read(int idx);
int FP_Pow(int x);
IirFilter* _iir_filter_create(IirFilter* obj, int nA, int nB, const short* a, const short* b, const short* scales);
void _iir_filter_progress(IirFilter* f, int n, short* buf);
void _iir_filter_delete(IirFilter* f);
DualTone* Dual_TONE_create(void);
int Dual_TONE_detect(DualTone* d, const short* in, int n);
void Dual_TONE_delete(DualTone* d);
/* callprog.c */
void CALLPROG_Create(CallProg* cp, CallProgCfg* cfg);
void CALLPROG_Dial(CallProg* cp, const char* dialString);
int CALLPROG_Progress(CallProg* cp, const short* in, short* out, int n);
void CALLPROG_Delete(CallProg* cp);
const char* CALLPROG_Status_string(int id);
#endif
