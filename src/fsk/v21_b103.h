/**
 * @file v21_b103.h
 * @brief Object types of the Bell 103 / V.21 engine (v21_b103.c).
 *
 * The engine keeps its state in three blocks:
 *   - B103Obj    the instance: configuration words, the status word returned to the host, and pointers to the other two blocks;
 *   - B103Hdx    the half-duplex handshake state: mode, sub-state, tx / rx counters, the current tx / rx handler functions, and the two tone objects;
 *   - B103Bundle the signal-processing objects: two AGCs, the transmit and receive matched filters, the FSK demodulator and modulator, the tone-detect
 *                counter and the band-pass correlator that is applied to the received samples.
 * The embedded FPM objects (fpm/fpm.h) are real typed members.  The blocks hold real pointers, so their layout differs between 32- and 64-bit builds; on a
 * 32-bit build the `_Static_assert`s pin the byte offsets the 32-bit test oracles rely on.  Code addresses the members by name; only the status word and a
 * few pointer-slot words use the B103_W / B103_PSTORE / B103_PLOAD macros below.
 */
#ifndef DSPLIBS_PORT_B103_OBJ_H
#define DSPLIBS_PORT_B103_OBJ_H
#include "fpm/fpm.h"

struct B103Obj;
typedef short (*TxHdxFn)(struct B103Obj*, unsigned short*, void*, unsigned short*);
typedef short (*RxHdxFn)(struct B103Obj*, void*, unsigned short*, unsigned short*);

/** Half-duplex state block. */
typedef struct B103Hdx
{
  short mode;      ///< 0 = local loop, 1 = originate, 2 = answer (B103FP_create() stores 1 / 2 for cfg[0] = 0 / 1); index into B103NextState[]
  short waitLimit; ///< blocks to wait for the far-end tone (max(cfg[3] / 20, 700)); also the initial tx countdown of the handshake
  short txCount;   ///< tx-phase countdown, decremented by TxHdxMarksB103() / TxHdxSilenceB103()
  short pad6;
  TxHdxFn txFn;   ///< current transmit handler
  short rxCount;  ///< rx-phase counter: tone-wait blocks (RxDetMarkB103) or lost-carrier blocks (RxHdxStartB103 / RxHdxDataB103)
  RxHdxFn rxFn;   ///< current receive handler
  short subState; ///< handshake sub-state 0 .. 5 (5 = error), advanced by the B103NextState[] functions
  short pad16;
  int w18;           ///< set to 1 in the originate (cfg[4] != 0) and answer branches; nothing in this file reads it
  FpmTone* secTone;  ///< detector of the far end's carrier (created for originate and answer only)
  FpmTone* mainTone; ///< local oscillator of the demodulator (FPM_TONE_generate_demod())
} B103Hdx;

/** Signal-processing block. */
typedef struct B103Bundle
{
  int agcActive;        ///< copied into agc0.active before each AGC run; 1 from create on, never changed
  int agcDetect;        ///< copy of agc0.detect; carrier detect = agcDetect & toneSeen
  int toneSeen;         ///< 1 once the secondary tone was detected or the demodulator is running
  FpmAgc agc0;          ///< AGC of the demodulated signal (its detect flag is the carrier detect)
  FpmAgc agc1;          ///< AGC in front of the far-end tone detector
  FpmMrf mrf0;          ///< transmit filter
  FpmMrf mrf1;          ///< receive filter
  FpmFsd fsd;           ///< FSK demodulator
  FpmFsm fsm;           ///< FSK modulator
  FpmMtd* mtd;          ///< multi-tone detector; created, never run by this file
  short* txBuf;         ///< modulator output / demodulator input, 0x144 bytes (162 samples)
  short* rxBuf;         ///< copy of the received block for the tone detector, 0x144 bytes (162 samples)
  short* bpfHist;       ///< sample history of the band-pass correlator
  const short* bpfCoef; ///< band-pass coefficients (a real table pointer here, not a CFGP handle); NULL in local-loop mode
  short bpfIdx;         ///< write position in the history
  short bpfLen;         ///< correlator length (0x32 answer, 0x28 originate, 0 otherwise)
  short cnt; ///< tone-detect block count: +5 per block with the secondary tone detected, reset to 0 otherwise; >= 0xf = locked, the demodulator runs (0x10)
  short pad;
} B103Bundle;

/** Instance. */
typedef struct B103Obj
{
  int cfg[7]; ///< configuration words, see B103FP_create()
  union
  {
    int w;
    unsigned char b[4];
  } st;               ///< status word returned by B103FP_modem(): b[0] = state, b[1] = flags
  void* fsdOutBuf;    ///< copy of the FSD output buffer address (written, never read here)
  int w24;            ///< cleared by B103FP_create(); not used otherwise
  short* fsdOutCount; ///< &bundle->fsd.outCount
  int w2c;            ///< cleared by B103FP_create(); not used otherwise
  int w30;            ///< cleared by B103FP_create(); not used otherwise
  short s34;          ///< cleared by B103FP_create(); not used otherwise
  short pad36;
  int w38;   ///< cleared by B103FP_create(); not used otherwise
  int w3c;   ///< cleared by B103FP_create(); not used otherwise
  short s40; ///< cleared by B103FP_create(); not used otherwise
  short pad42;
  int w44;   ///< cleared by B103FP_create(); not used otherwise
  int w48;   ///< cleared by B103FP_create(); not used otherwise
  short s4c; ///< cleared by B103FP_create(); not used otherwise
  short pad4e;
  B103Hdx* hdx;       ///< half-duplex state
  B103Bundle* bundle; ///< signal-processing objects
} B103Obj;

/**
 * B103_W(p, i): word i of the instance as an int.  B103_PSTORE / B103_PLOAD store / load a pointer-slot member `m`, whose word index in the 32-bit layout is i.
 * 32-bit build: an int access `*((int *)p + i)` (the pointer-slot macros convert through intptr_t; the int-indexed spelling is the one that keeps the
 * generated code unchanged).  64-bit build: the plain member; B103_W is then valid only for i = 0 .. 7 (no pointer precedes byte 0x20).
 */
#if UINTPTR_MAX == 0xffffffffu
#define B103_W(p, i) (*((int*)(p) + (i)))
#define B103_PSTORE(p, m, i, v) (B103_W(p, i) = (int)(intptr_t)(v))
#define B103_PLOAD(T, p, m, i) ((T)(intptr_t)B103_W(p, i))
#else
#define B103_W(p, i) (*((int*)(p) + (i)))
#define B103_PSTORE(p, m, i, v) ((p)->m = (v))
#define B103_PLOAD(T, p, m, i) ((p)->m)
#endif

#if UINTPTR_MAX == 0xffffffffu
_Static_assert(sizeof(B103Hdx) == 0x24 && offsetof(B103Hdx, txFn) == 8 && offsetof(B103Hdx, rxCount) == 0xc && offsetof(B103Hdx, rxFn) == 0x10 &&
                   offsetof(B103Hdx, subState) == 0x14 && offsetof(B103Hdx, w18) == 0x18 && offsetof(B103Hdx, secTone) == 0x1c &&
                   offsetof(B103Hdx, mainTone) == 0x20,
               "B103Hdx 32-bit layout");
_Static_assert(sizeof(B103Bundle) == 0x100 && offsetof(B103Bundle, agc0) == 0xc && offsetof(B103Bundle, agc1) == 0x38 && offsetof(B103Bundle, mrf0) == 0x64 &&
                   offsetof(B103Bundle, mrf1) == 0x80 && offsetof(B103Bundle, fsd) == 0x9c && offsetof(B103Bundle, fsm) == 0xd4 &&
                   offsetof(B103Bundle, mtd) == 0xe4 && offsetof(B103Bundle, txBuf) == 0xe8 && offsetof(B103Bundle, rxBuf) == 0xec &&
                   offsetof(B103Bundle, bpfHist) == 0xf0 && offsetof(B103Bundle, bpfCoef) == 0xf4 && offsetof(B103Bundle, bpfIdx) == 0xf8 &&
                   offsetof(B103Bundle, cnt) == 0xfc,
               "B103Bundle 32-bit layout");
_Static_assert(sizeof(B103Obj) == 0x58 && offsetof(B103Obj, st) == 0x1c && offsetof(B103Obj, fsdOutBuf) == 0x20 && offsetof(B103Obj, fsdOutCount) == 0x28 &&
                   offsetof(B103Obj, s34) == 0x34 && offsetof(B103Obj, s40) == 0x40 && offsetof(B103Obj, s4c) == 0x4c && offsetof(B103Obj, hdx) == 0x50 &&
                   offsetof(B103Obj, bundle) == 0x54,
               "B103Obj 32-bit layout");
#endif
#endif
