/**
 * @file fax_v21.h
 * @brief Typed objects and entry points of the fax V.21 channel (the 300 bit/s T.30 control channel), receive and transmit halves.
 *
 * Each half is made of three structs: the instance (V21Rx / V21Tx: config words, a state dword, pointers), the half-duplex block ("hdx": the handler
 * function of the current state, the state number and counters) and the bundle of FPM objects (V21RxBundle: AGC, matched filter, FSK detector, tone
 * detector and a sample buffer; V21TxBundle: FSK modulator, matched filter and a sample buffer).  The structs use real pointer members; on 32-bit hosts
 * the _Static_asserts at the end pin the member offsets other code relies on, and on 64-bit hosts nothing may address these objects by a raw byte
 * offset.  The handler slot is a typed function pointer (V21RxHdxFn / V21TxHdxFn).  The transmit FIFO is the FaxFifo of fax_common.h.
 *
 * The VMI layer (fax_vmi.h) reaches the engines through the v21rx_* / v21tx_* wrappers, which take a FaxVmiV21 (the common VMI modem view FaxVmiM) and
 * the plain control and status records FaxV21Ctl / FaxV21Status.  The engine handle in the VMI object is a V21Rx * (receive) or a V21Tx * (transmit).
 * The scrambler wrappers of fax_modem_views.h reach V.17 / V.27ter / V.29 objects only; no V.21 object is accessed by offset.
 *
 * The implementation is fax_v21.c; the tables are fax_v21_tables.c.
 */
#ifndef DSPLIBS_PORT_FAX_V21_OBJ_H
#define DSPLIBS_PORT_FAX_V21_OBJ_H
#include <stddef.h>
#include <stdint.h>
#include "fax_vmi.h"
#include "fpm/fpm.h"
#include "fax_common.h"

/**
 * The config block: V21RX_CFG (6 words) or V21TX_CFG (7 words; the seventh is stored separately as V21Tx::w18).
 * s[0] is the channel (0 = originate side / channel 1, otherwise channel 2), w[1] = 300 baud, w[2] is the carrier loss time (the control function
 * overwrites it), w[5] is the user word copied into the MRF / FSD configs.
 */
typedef union V21Cfg
{
  int w[6];
  short s[12];
  unsigned char b[24];
} V21Cfg;

struct V21Rx;
struct V21Tx;
/**
 * The half-duplex handlers (RxHdx*V21 / TxHdx*V21).  Receive: (engine, input samples, output bits, &count), returns the number of bits produced.
 * Transmit: (engine, input bits, output samples, &count), returns the number of samples produced.  `*count` is in / out (16 bit).
 */
typedef int (*V21RxHdxFn)(struct V21Rx* o, short* in, short* out, unsigned short* n);
typedef int (*V21TxHdxFn)(struct V21Tx* o, short* in, short* out, unsigned short* n);

/** Receive state number (V21RxHdx::state); RxNextStateV21() prints "V21RX_STATE_<name>" for the state it leaves. */
enum V21RxState
{
  V21RX_STATE_START = 0,
  V21RX_STATE_WAIT = 1,
  V21RX_STATE_DATA = 2,
  V21RX_STATE_IDLE = 3,
  V21RX_STATE_ERROR = 4
};

/**
 * Receive status code (byte 0 of V21Rx::st).  The values index the strings of V21RX_MESG ("V.21 Receive Data Mode", "V.21 Rx Start of Protocol",
 * "V.21 Rx Wait for Valid Data Protocol", "ERROR: V.21 Rx Internal Error", "ERROR: Loss Carrier In V.21 Rx Protocol", "V.21 Rx Idle (End of
 * Transmition)", "CONNECT: V.21 Receive 300 bps"); each value is stored by the handler of that state.
 */
enum V21RxStatus
{
  V21RX_MSG_DATA = 0,
  V21RX_MSG_START = 1,
  V21RX_MSG_WAIT = 2,
  V21RX_MSG_ERROR = 3,
  V21RX_MSG_LOSS_CARRIER = 4,
  V21RX_MSG_IDLE = 5,
  V21RX_MSG_CONNECT = 6
};

/** The receive half-duplex block. */
typedef struct V21RxHdx
{
  int hold;      ///< nonzero: the Data state discards its input (control: bit 4 of the flags byte)
  V21RxHdxFn fn; ///< handler of the current state
  short state;   ///< V21RxState: 0 start, 1 wait, 2 data, 3 idle, 4 error
  short wait;    ///< Wait state: valid blocks still to see
  short ones;    ///< run of one bits (Start state flag detector)
  short flags;   ///< number of 0x7e flags (a zero after six ones) seen in the Start state
} V21RxHdx;

/** Transmit state number (V21TxHdx::state); TxNextStateV21() prints "V21TX_STATE_<name>" for the state it leaves. */
enum V21TxState
{
  V21TX_STATE_START = 0,
  V21TX_STATE_DATA = 1,
  V21TX_STATE_IDLE = 2
};

/**
 * Transmit status code (byte 0 of V21Tx::st).  The values index the strings of V21TX_MESG ("V.21 Transmit Data Mode", "V.21 Transmit Idle Mode",
 * "ERROR: V.21 Tx Internal Error", "ERROR: Transmit Input Queue Under-run", "... Over-run", "CONNECT: V.21 Transmit 300 bps").
 */
enum V21TxStatus
{
  V21TX_MSG_DATA = 0,
  V21TX_MSG_IDLE = 1,
  V21TX_MSG_ERROR = 2,
  V21TX_MSG_UNDERRUN = 3,
  V21TX_MSG_OVERRUN = 4,
  V21TX_MSG_CONNECT = 5
};

/** The transmit half-duplex block. */
typedef struct V21TxHdx
{
  FaxFifo* fifo; ///< 6-entry bit queue (FIFO_create with size 6, fill 1)
  int pend;      ///< nonzero: the input queue is not used for the under-run check (control: bit 4)
  V21TxHdxFn fn; ///< handler of the current state
  short state;   ///< V21TxState: 0 start, 1 data, 2 idle
  short s0e;     ///< always 0
} V21TxHdx;

/** The receive bundle: signal-path objects. */
typedef struct V21RxBundle
{
  int w0;        ///< always 1
  int agcDetect; ///< AGC detect flag of the last block
  int noTone;    ///< 1 = no tone seen by the tone detector in the last block
  FpmAgc agc;    ///< automatic gain control
  FpmMrf mrf;    ///< matched filter
  FpmFsd fsd;    ///< FSK detector
  FpmMtd* mtd;   ///< tone detector
  short* buf;    ///< 160-sample scratch: the MRF output, then |FSD output|
} V21RxBundle;

/** The transmit bundle: signal-path objects. */
typedef struct V21TxBundle
{
  FpmFsm fsm; ///< FSK modulator (its output scale c.s[3] is written by the control function and zeroed for one call by TxNoCarrierV21())
  FpmMrf mrf; ///< matched filter
  short* buf; ///< 160-sample scratch: the FSM output
} V21TxBundle;

/** The receive instance. */
typedef struct V21Rx
{
  V21Cfg cfg; ///< config words (see V21Cfg)
  union
  {
    int w;
    unsigned char b[4];
  } st; ///< state dword: b[0] status (V21RxStatus); b[1] flags 0x01 data, 0x02 error, 0x20 carrier, 0x40 / 0x10 set at create, 0x80 SNR low; b[2] bit 0 idle
  short* fsdOut;       ///< copy of bundle->fsd.out
  int w20;             ///< zero
  short* fsdS20;       ///< &bundle->fsd.outCount
  int w28;             ///< zero
  int w2c;             ///< zero
  short s30;           ///< zero
  int w34;             ///< zero
  int w38;             ///< zero
  short s3c;           ///< zero
  int w40;             ///< zero
  int w44;             ///< zero
  short s48;           ///< zero
  V21RxHdx* hdx;       ///< half-duplex block
  V21RxBundle* bundle; ///< signal-path objects
} V21Rx;

/** The transmit instance. */
typedef struct V21Tx
{
  V21Cfg cfg; ///< config words 0 .. 5 (s[0] = channel; bit 2 of b[0x10] is set by the control function)
  int w18;    ///< config word 6 (goes to the MRF config)
  union
  {
    int w;
    unsigned char b[4];
  } st;                ///< state dword: b[0] status code (V21TxStatus); b[1] flags 0x01 / 0x02 error / 0x08 / 0x10 / 0x40; b[2] bit 0
  V21TxHdx* hdx;       ///< half-duplex block
  V21TxBundle* bundle; ///< signal-path objects
} V21Tx;

/**
 * The VMI object view used by the v21rx_* / v21tx_* wrappers.  The VMI view of every modem is one type (FaxVmiM in fax_vmi.h); FaxVmiV21 is its name
 * here.  The engine handle (eng) and the config pointer (u10.cfg) are opaque pointers, cast to the engine / config type where used.
 */
typedef FaxVmiM FaxVmiV21;

/** Control record of V21RX_control() / V21TX_control() (only the fields the functions read). */
typedef struct FaxV21Ctl
{
  int w0;              ///< unused
  int w4;              ///< stored as cfg.w[2] (carrier loss time)
  int txScale;         ///< transmit: low 16 bits become the modulator output scale
  unsigned char b0c;   ///< transmit: bit 2 sets bit 2 of cfg.b[0x10]
  unsigned char flags; ///< bit 4: hdx hold (receive) / pend (transmit); bit 1: re-create the engine
  unsigned char pad0e[2];
} FaxV21Ctl;

/** Status record of V21RX_status() / V21TX_status() (22 bytes; fields the functions do not write are not named). */
typedef struct FaxV21Status
{
  short channel, s2, s4, s6, s8, sa, sc, se, s10, s12;
  unsigned char b14, b15;
} FaxV21Status;

#if UINTPTR_MAX == 0xffffffffu
_Static_assert(sizeof(V21RxHdx) == 0x10 && offsetof(V21RxHdx, fn) == 4 && offsetof(V21RxHdx, state) == 8 && offsetof(V21RxHdx, wait) == 0xa &&
                   offsetof(V21RxHdx, ones) == 0xc && offsetof(V21RxHdx, flags) == 0xe,
               "V21RxHdx 32-bit layout");
_Static_assert(sizeof(V21TxHdx) == 0x10 && offsetof(V21TxHdx, pend) == 4 && offsetof(V21TxHdx, fn) == 8 && offsetof(V21TxHdx, state) == 0xc &&
                   offsetof(V21TxHdx, s0e) == 0xe,
               "V21TxHdx 32-bit layout");
_Static_assert(sizeof(V21RxBundle) == 0x94 && offsetof(V21RxBundle, agc) == 0xc && offsetof(V21RxBundle, mrf) == 0x38 && offsetof(V21RxBundle, fsd) == 0x54 &&
                   offsetof(V21RxBundle, mtd) == 0x8c && offsetof(V21RxBundle, buf) == 0x90,
               "V21RxBundle 32-bit layout");
_Static_assert(sizeof(V21TxBundle) == 0x30 && offsetof(V21TxBundle, mrf) == 0x10 && offsetof(V21TxBundle, buf) == 0x2c, "V21TxBundle 32-bit layout");
_Static_assert(sizeof(V21Rx) == 0x54 && offsetof(V21Rx, st) == 0x18 && offsetof(V21Rx, fsdOut) == 0x1c && offsetof(V21Rx, fsdS20) == 0x24 &&
                   offsetof(V21Rx, s30) == 0x30 && offsetof(V21Rx, s3c) == 0x3c && offsetof(V21Rx, s48) == 0x48 && offsetof(V21Rx, hdx) == 0x4c &&
                   offsetof(V21Rx, bundle) == 0x50,
               "V21Rx 32-bit layout");
_Static_assert(sizeof(V21Tx) == 0x28 && offsetof(V21Tx, w18) == 0x18 && offsetof(V21Tx, st) == 0x1c && offsetof(V21Tx, hdx) == 0x20 &&
                   offsetof(V21Tx, bundle) == 0x24,
               "V21Tx 32-bit layout");
_Static_assert(sizeof(FaxVmiV21) == 0x18 && offsetof(FaxVmiV21, u4) == 4 && offsetof(FaxVmiV21, s0c) == 0xc && offsetof(FaxVmiV21, u10) == 0x10 &&
                   offsetof(FaxVmiV21, eng) == 0x14,
               "FaxVmiV21 32-bit layout");
#endif
_Static_assert(sizeof(FaxV21Ctl) == 0x10 && offsetof(FaxV21Ctl, flags) == 0xd && sizeof(FaxV21Status) == 0x16 && offsetof(FaxV21Status, b14) == 0x14,
               "fax V.21 records");
_Static_assert(sizeof(V21Cfg) == 24 && sizeof(((V21Rx*)0)->cfg) == 24, "V21Cfg");

/* ---- the engine entry points (documented at their definitions in fax_v21.c) ---- */
V21Rx* V21RX_create(V21Rx* o, const int* cfg);
void V21RX_delete(V21Rx* o);
int V21RX_modem(V21Rx* o, short* in, short* out, unsigned short* n);
int V21RX_control(V21Rx* o, const FaxV21Ctl* ctl);
int V21RX_status(V21Rx* o, FaxV21Status* st);
V21Tx* V21TX_create(V21Tx* o, const int* cfg);
void V21TX_delete(V21Tx* o);
int V21TX_modem(V21Tx* o, short* in, short* out, unsigned short* n);
int V21TX_control(V21Tx* o, const FaxV21Ctl* ctl);
int V21TX_status(V21Tx* o, FaxV21Status* st);

/* the state handlers and helpers */
int RxHdxStartV21(V21Rx* o, short* in, short* out, unsigned short* n);
int RxHdxWaitV21(V21Rx* o, short* in, short* out, unsigned short* n);
int RxHdxDataV21(V21Rx* o, short* in, short* out, unsigned short* n);
int RxHdxIdleV21(V21Rx* o, short* in, short* out, unsigned short* n);
int RxHdxErrorV21(V21Rx* o, short* in, short* out, unsigned short* n);
void RxNextStateV21(V21Rx* o);
int TxHdxStartV21(V21Tx* o, short* in, short* out, unsigned short* n);
int TxHdxDataV21(V21Tx* o, short* in, short* out, unsigned short* n);
int TxHdxIdleV21(V21Tx* o, short* in, short* out, unsigned short* n);
void TxNextStateV21(V21Tx* o);
int DemodDataV21(V21Rx* o, short* in, short* out, unsigned short n);
int CarrierDetectV21(V21Rx* o);
int GetSNRV21(V21Rx* o);
int ModDataV21(V21Tx* o, short* in, short* out, unsigned short n);
int TxNoCarrierV21(V21Tx* o, short* in, short* out, unsigned short n);

/* the VMI wrappers (v21rx_* / v21tx_*) */
void* v21rx_create(FaxVmiV21* m, const int* cfg);
void* v21tx_create(FaxVmiV21* m, const int* cfg);
void v21rx_delete(FaxVmiV21* m);
void v21tx_delete(FaxVmiV21* m);
int v21rx_process(FaxVmiV21* m, short* in, unsigned short* outn, unsigned short* n);
int v21tx_process(FaxVmiV21* m, short* out, unsigned short* n, unsigned short* outn);
int v21rx_status(FaxVmiV21* m, FaxV21Status* st);
int v21tx_status(FaxVmiV21* m, FaxV21Status* st);
int v21rx_control(FaxVmiV21* m, const FaxV21Ctl* ctl);
int v21tx_control(FaxVmiV21* m, const FaxV21Ctl* ctl);
void v21rx_message(FaxVmiV21* m, int idx, const char** out);
void v21tx_message(FaxVmiV21* m, int idx, const char** out);
#endif
