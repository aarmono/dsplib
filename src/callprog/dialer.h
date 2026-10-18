/**
 * @file dialer.h
 * @brief Dialer: dial-string analysis, DTMF / pulse dial generation, flash and pause handling, calling-tone generator and pulse-dial helpers.
 *
 * The dialer turns a dial string ("T5551212,W9;", ...) into line signals.  DialerCreate() analyses the string (AnalyseDialString in dialer.c) and
 * reads the dialing configuration from the host (GetDialerConfig()); DialerProgress() is then called with output buffers and produces DTMF tones, silence
 * between digits, comma pauses and flash, drives pulse dialling through the pulse helpers, and reports the events that need the call-progress state machine
 * (wait for dial tone, silence or bong tone, calling tone, end of string).  The user is the call-progress state machine (callprog.c), which owns the Dialer.
 *
 * Dial-string characters: 0-9 * # A-D digits; T / P switch to tone or pulse dialling (a string must start with one of them); `,` pause, `!` flash, `W`
 * wait for dial tone, `@` wait for silence, `$` wait for bong tone, `^` calling tone, `;` return to command mode (end partially, last character only).
 *
 * Pulse dialling.  The pulse helpers (PulseDialDigit(), IsPulseDialerReady(), LastPulseDigitDialed(), SetPulseBreakTime(), SetPulseMakeTime()) do not keep
 * their state in the Dialer: they find the call glue block through modem_get_param(m, MDMPRM_DP_ADDR) -> struct dp -> dp_data and use its CallPulse member
 * (CallGlue in callprog.h), and they drive the host's hook through modem_set_param().
 *
 * Layout.  Everything but the trailing modem pointer is int / short / char with the 32-bit layout asserted below.  The modem pointer is real, so on a
 * 64-bit build the Dialer is 8 bytes larger: it must be embedded with sizeof / offsetof, never addressed by a raw byte offset.  The dial string buffer
 * holds 100 characters; AnalyseDialString accepts 100 (see DialerCreate()).
 */
#ifndef DSPLIBS_PORT_DIAL_OBJ_H
#define DSPLIBS_PORT_DIAL_OBJ_H
#include <stddef.h>
#include <stdint.h>

#include "core/host_interface.h" /* host interface: DP_ID, DPSTAT_*, struct dp / dp_operations, MDMPRM_* / Get* ids */
#include "core/dsp_host.h"

/** Dialing configuration read from the host by GetDialerConfig(). */
typedef struct DialerCfg
{
  int toneDigitLen;     ///< DTMF digit length (GetDTMFDialSpeed; 8 samples per unit)
  int toneBetween;      ///< silence between DTMF digits (GetDTMFDialSpeed again; 8 samples per unit)
  short gain1;          ///< low-group tone gain, Q14 (derived from GetDTMFHighToneLevel and GetDTMFHighAndLowToneLevelDifference)
  short gain2;          ///< high-group tone gain, Q14 (table in 1 dB steps by GetDTMFHighToneLevel 6 .. 12, else 0x32d6)
  int pulseOnHookTime;  ///< GetPulseDialBreakTime
  int pulseOffHookTime; ///< GetPulseDialMakeTime
  int pulseBetween;     ///< 10 * GetPulseBetweenDigitsInterval (silence between pulse digits, 8 samples per unit)
  int dialPause;        ///< GetDialPauseTime: seconds per comma
  int flashTime;        ///< GetHookFlashTime
  int toneOrPulse;      ///< 1 = tone dialling, 0 = pulse dialling; starts as (GetPulseDialingFlag != 0), changed by T / P in the string
  int modValid;         ///< GetDialModifierValidation: non-zero makes an illegal modifier TOLERABLE instead of INVALID
  int abcd;             ///< GetABCDDialingPermittedFlag: when non-zero the digits A .. D are INVALID in AnalyseDialString and skipped in tone dialling
  int pulseAndTone;     ///< GetPulseAndToneDialInSameDialStringPermittedFlag: when non-zero a T / P that changes the mode is INVALID and ignored
  int callingTone;      ///< GetCallingToneFlag: '^' is INVALID for 2, accepted by the dialer for 1 and 3
  int commaLimit;       ///< GetComaPauseDurationLimit: longest total pause of a run of commas, seconds
  int digitPattern;     ///< GetPulseDialDigitPattern: 1 .. 3 select how a keypad position maps to a pulse count
} DialerCfg;

/**
 * Dialer::state, the states DialerProgress() switches over (GetNext() returns the next one): a digit, silence between digits, a flash, a comma pause, the
 * four wait / event states, and the two end states.
 */
enum DialerState
{
  DIALER_INITIAL_STATE = 0,
  DIALER_DIGIT_STATE = 1,
  DIALER_SILENCE_STATE = 2,
  DIALER_FLASH_STATE = 3,
  DIALER_COMMA_STATE = 4,
  DIALER_WAIT_FOR_DIALTONE_STATE = 5,
  DIALER_WAIT_FOR_SILENCE_STATE = 6,
  DIALER_WAIT_FOR_BONGTONE_STATE = 7,
  DIALER_CALLING_TONE_STATE = 8,
  DIALER_END_PARTIALLY_STATE = 9,
  DIALER_END_STATE = 10
};

/**
 * The value DialerProgress() returns (DialerCreate() returns 0 or DIALER_ERROR_MSG): 0 the buffer is full, dialling continues; 1 .. 4 the wait / event
 * states were reached (the dial state is reset to INITIAL); 5 / 6 the dial string ended (5: partially, by ';'); 7 a bad state.
 */
enum DialerResult
{
  DIALER_DIALING_MSG = 0,
  DIALER_WAIT_DIALTONE_MSG = 1,
  DIALER_WAIT_SILENCE_MSG = 2,
  DIALER_WAIT_BONGTONE_MSG = 3,
  DIALER_CALLING_TONE_MSG = 4,
  DIALER_END_PARTIALLY_MSG = 5,
  DIALER_END_MSG = 6,
  DIALER_ERROR_MSG = 7
};

/** The dialer (dialer.c). */
typedef struct Dialer
{
  char str[0x64];      ///< the dial string (up to 100 characters; the NUL of a 100-character string lands in cfg.toneDigitLen)
  DialerCfg cfg;       ///< the configuration read by GetDialerConfig()
  int lastIdx;         ///< index of the last dialable symbol of str (-2 = none)
  int syntax;          ///< AnalyseDialString verdict: 0 FATAL, 1 INVALID, 2 TOLERABLE, 3 VALID
  int commaCount;      ///< length of the current run of commas
  int cur;             ///< index of the current character in str (-1 before the first)
  int row;             ///< keypad row of the digit being dialled (selects the low-group frequency)
  int col;             ///< keypad column (selects the high-group frequency)
  unsigned state;      ///< DialerState
  int left;            ///< samples left of the current element (digit, silence or pause)
  unsigned short ph1;  ///< phase accumulator of the low-group tone (14 bit: 0x4000 per turn)
  unsigned short ph2;  ///< phase accumulator of the high-group tone
  unsigned short inc1; ///< phase increment of the low-group tone
  unsigned short inc2; ///< phase increment of the high-group tone
  int digitSent;       ///< a pulse digit was handed to PulseDialDigit()
  int pulseActive;     ///< a pulse digit has been dialled (LastPulseDigitDialed() is due)
  int pulseDone;       ///< LastPulseDigitDialed() was called
  struct modem* modem; ///< the host's modem object
} Dialer;

/** State of the calling-tone generator (GenerateCallingTone()): an on / off cadence of a tone. */
typedef struct CallingTone
{
  unsigned short phase; ///< phase accumulator, 14 bit
  int on;               ///< 1 = tone, 0 = silence
  int cnt;              ///< samples left in the current segment
  short gain;           ///< amplitude factor, FP_Pow() of the scaled level
} CallingTone;

/** The pulse-dial state the helpers keep in the call glue block. */
typedef struct CallPulse
{
  unsigned makeTime;  ///< make period (set by SetPulseMakeTime())
  unsigned breakTime; ///< break period (set by SetPulseBreakTime())
  unsigned w5bc;      ///< cleared by call_create, otherwise unused
  unsigned count;     ///< pulses still to dial (the digit, 10 for 0)
  unsigned counter;   ///< time counter (+5 per IsPulseDialerReady() call)
  int hookOn;         ///< 1 during the break period
} CallPulse;

#if UINTPTR_MAX == 0xffffffffu
_Static_assert(sizeof(DialerCfg) == 0x3c, "DialerCfg");
_Static_assert(offsetof(Dialer, cfg) == 0x64 && offsetof(Dialer, lastIdx) == 0xa0 && offsetof(Dialer, syntax) == 0xa4 && offsetof(Dialer, commaCount) == 0xa8,
               "Dialer 1");
_Static_assert(offsetof(Dialer, cur) == 0xac && offsetof(Dialer, row) == 0xb0 && offsetof(Dialer, col) == 0xb4 && offsetof(Dialer, state) == 0xb8, "Dialer 2");
_Static_assert(offsetof(Dialer, left) == 0xbc && offsetof(Dialer, ph1) == 0xc0 && offsetof(Dialer, inc2) == 0xc6 && offsetof(Dialer, digitSent) == 0xc8,
               "Dialer 3");
_Static_assert(offsetof(Dialer, pulseActive) == 0xcc && offsetof(Dialer, pulseDone) == 0xd0 && offsetof(Dialer, modem) == 0xd4 && sizeof(Dialer) == 0xd8,
               "Dialer 4");
_Static_assert(offsetof(DialerCfg, gain1) == 8 && offsetof(DialerCfg, pulseOnHookTime) == 0xc && offsetof(DialerCfg, toneOrPulse) == 0x20 &&
                   offsetof(DialerCfg, digitPattern) == 0x38,
               "cfg");
_Static_assert(sizeof(CallingTone) == 0x10 && offsetof(CallingTone, on) == 4 && offsetof(CallingTone, cnt) == 8 && offsetof(CallingTone, gain) == 0xc,
               "CallingTone");
_Static_assert(sizeof(CallPulse) == 0x18 && offsetof(CallPulse, hookOn) == 0x14, "CallPulse");
_Static_assert(offsetof(struct dp, modem) == 4 && offsetof(struct dp, dp_data) == 0x10, "struct dp");
#endif

int DialerCreate(Dialer* d, const char* dialString, struct modem* m);
int DialerProgress(Dialer* d, short* buf, int* pos, int last);
void DialerAbort(Dialer* d);
void GetDialerConfig(DialerCfg* cfg, struct modem* m);
int IsDialStringInvalid(Dialer* d, const char* s);
int Dialer_IsDialStringInvalid(void* callprog, const char* s);
void ResetCallingTone(CallingTone* ct, int level);
void GenerateCallingTone(CallingTone* ct, short* out, int n);
void PulseDialDigit(struct modem* m, unsigned digit);
int IsPulseDialerReady(struct modem* m);
void LastPulseDigitDialed(struct modem* m);
void SetPulseBreakTime(struct modem* m, unsigned t);
void SetPulseMakeTime(struct modem* m, unsigned t);
#endif
