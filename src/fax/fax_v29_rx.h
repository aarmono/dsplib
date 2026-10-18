/**
 * @file fax_v29_rx.h
 * @brief Typed objects and entry points of the fax V.29 receiver (7200 / 9600 bit/s, T.30 high-speed data).
 *
 * The receiver is split into three heap objects, all plain structs with real pointer members:
 * - the engine `V29Rx` (six configuration words, the status word, copies of the equalizer's buffer pointers, the pointers to the other two blocks),
 * - the half-duplex block `V29RxHdx` (the tone killer and tone detector of the start state, the handler pointer of the current state and its counters,
 *   the carrier detector with its own AGC),
 * - the bundle `V29RxBundle` (the control gates, the decision-loop state the equalizer callbacks use, and the matched filter, AGC, timing recovery,
 *   equalizer and descrambler sub-objects with their sample buffers).
 * The timing-recovery and equalizer sub-objects are the typed FaxSre / FaxFse of fax_rx_common.h (shared with the V.27ter and V.17 receivers).
 *
 * On a 32-bit build every member sits at the byte offset the _Static_asserts pin; on a 64-bit build the members after a pointer move, so NOTHING may
 * address these objects by a raw byte offset.  The one exception is the descrambler wrapper DescrambleDataV29() (fax_modem_wrappers.c), which takes the
 * engine as an opaque `void *` and reaches `bundle->sdm` through the accessor faxw_v29_rxsdm() of fax_modem_views.h; on a 64-bit build fax_v29_rx.c
 * defines that accessor over the typed members.
 *
 * Differences from the V.27ter receiver (fax_v27_rx.h): one rate pair (7200 / 9600 bit/s), a tone killer in front of the start-state tone detector, a
 * 16 point (8 point at 7200) decision constellation, an LFSR equalizer training sequence, a real SNR estimate (GetSNRV29()), and the receive scrambler
 * at the end of the bundle.
 *
 * The VMI layer (fax_vmi.c) calls the v29rx_* wrappers with a `FaxVmiV29 *`, the view of the VMI object shared with the transmitter (fax_v29_tx.h).  The
 * receive initialiser stores 0x140 / 0xa5 / 0 / 10 into the shorts at byte offsets 8 .. 0xe and the engine handle into the `eng` slot; `eng` is typed
 * `struct V29Tx *` for the transmitter, so the receiver code casts it to `V29Rx *`.
 */
#ifndef DSPLIBS_PORT_FAX_V29RX_OBJ_H
#define DSPLIBS_PORT_FAX_V29RX_OBJ_H
#include <stddef.h>
#include <stdint.h>
#include "fax_common.h"
#include "fax_rx_common.h"
#include "fax_v29_tx.h"
#include "fpm/fpm.h"

/** Configuration block (6 words): `s[2]` is the bit rate (7200 or 9600), `w[2]` (60000) is overwritten by V29RX_control(), `w[5]` is the user word that is
 *  copied into the matched-filter, timing-recovery and equalizer configurations and is otherwise unused. */
typedef union V29RxCfg
{
  int w[6];
  short s[12];
  unsigned char b[24];
} V29RxCfg;

struct V29Rx;
/** Half-duplex handler of one receiver state: consumes the whole input block, returns the number of output symbols; `*n` (16 bit) is the number of input
 *  samples on entry and the number left over on return. */
typedef int (*V29RxHdxFn)(struct V29Rx* o, short* in, short* out, unsigned short* n);

/** Receiver state, V29RxHdx::state; RxNextStateV29() prints the name of the state it leaves (ERROR is never printed). */
enum V29RxState
{
  V29RX_STATE_START = 0,     ///< waiting for carrier
  V29RX_STATE_EPOCH_DET = 1, ///< looking for the phase-reversal epoch of the training sequence
  V29RX_STATE_PROTOCOL = 2,  ///< equalizer training
  V29RX_STATE_DATA = 3,      ///< data
  V29RX_STATE_IDLE = 4,      ///< decision quality lost; back to DATA when the error is small again
  V29RX_STATE_ERROR = 5      ///< carrier lost during training
};

/** Status code in V29Rx::st.b[0]; the value indexes the message table V29RX_MESG (the last two entries are the connect messages). */
enum V29RxStatus
{
  V29RX_MSG_DATA = 0,
  V29RX_MSG_PROTOCOL = 1,
  V29RX_MSG_START = 2,
  V29RX_MSG_ERROR = 3,
  V29RX_MSG_LOSS_CARRIER = 4,
  V29RX_MSG_IDLE = 5,
  V29RX_MSG_CONNECT_9600 = 6,
  V29RX_MSG_CONNECT_7200 = 7
};

/** Half-duplex block: the per-state handler and the detectors that run in the start and data states. */
typedef struct V29RxHdx
{
  FpmMtd* mtd;      ///< tone detector of the start state (coefficients V29_MTD_COEFF)
  FpmTone* tone;    ///< tone killer in front of the tone detector
  int hold;         ///< nonzero: RxHdxDataV29() discards its input (control byte 0xd bit 4)
  short rate;       ///< 0 = 7200, 1 = 9600 (also for any other configured rate)
  short s0e;        ///< unused
  V29RxHdxFn fn;    ///< handler of the current state
  short state;      ///< V29RxState; 0 start, 1 epoch detect, 2 protocol, 3 data, 4 idle, 5 error
  short count;      ///< blocks left in the current state
  short* buf;       ///< 0x140 byte work block: the halved input of the tone killer
  short s1c;        ///< carrier-detector mode, written by the T.30 receiver (cd_flag): 0 = decoder-error test only, else also the V.21 carrier timeout
  short s1e;        ///< unused
  FpmMtd* mtd2;     ///< V.21 carrier detector (V21_CHAN2_MTD_COEFF), used by DataCarrierDetectV29()
  short* buf2;      ///< 0x140 byte work block: copy of the input block for the carrier detector
  short sinceTone;  ///< samples accumulated since the carrier detector `mtd2` last returned nonzero
  short toneSearch; ///< flag: the V.21 carrier search is active
  FpmAgc agc;       ///< AGC of the carrier detector
} V29RxHdx;

/** Decision-loop state shared by the equalizer callbacks V29RX_epoch_det() / V29RX_eq_train() / V29RX_decision() (reached through `fse.cfg.hist`). */
typedef struct V29RxLoop
{
  short avgA, avgB;   ///< epoch detector: smoothed magnitudes of the large (A) / small (B) angle steps
  short symI0, symQ0; ///< the last equalized symbol (I, Q)
  short symI1, symQ1; ///< the symbol before it
  short symI2, symQ2; ///< and the one before that
  int rate;           ///< copy of hdx rate (0 = 7200, 1 = 9600)
  short idx;          ///< previous phase index (decision) / table index (training)
  short reg;          ///< 7 bit LFSR of the equalizer training sequence (0x6a after create)
  short phaseCnt;     ///< calls since the epoch detector / the training was (re)started
  short s1a;          ///< unused
  short prev;         ///< previous angle (epoch detector)
  short cnt;          ///< call counter (wraps from 0x8000 to 0x4000)
} V29RxLoop;

/** Bundle: the control gates, the decision-loop state and the signal-path sub-objects. */
typedef struct V29RxBundle
{
  int w0;         ///< cleared by control byte 0xc bit 3 (status record bit 3 = (w0 == 0))
  int sreGate;    ///< gate of the timing-recovery loop (sre.loopEnable = sreGate & detect)
  int pllGate;    ///< gate of the equalizer carrier PLL (fse.pllEnable = pllGate & detect)
  int wc;         ///< unused
  int w10;        ///< unused
  int w14;        ///< unused
  int w18;        ///< status record bit 1 = w18 & 1
  int w1c;        ///< unused
  int eqGate;     ///< gate of the equalizer update (fse.eqEnable = eqGate & detect); cleared by control byte 0xc bit 5 (status record bit 5 = (eqGate == 0))
  int w24;        ///< unused
  V29RxLoop loop; ///< decision-loop state for the equalizer callbacks
  FpmMrf mrf;     ///< matched / interpolation filter
  FpmAgc agc;     ///< input AGC
  FaxSre sre;     ///< symbol timing recovery
  FaxFse fse;     ///< fractionally spaced equalizer with carrier PLL and decision callback
  FaxSdm sdm;     ///< receive descrambler (used by DescrambleDataV29())
  short* buf1;    ///< 0x140 byte work block: matched-filter output
  short* buf2;    ///< 0x148 byte work block: timing-recovery output
  short decErrSmooth; ///< decoder error smoothed over the first blocks (QualityDetectV29())
  short qualBlocks;   ///< number of blocks averaged so far (0 .. 0x33)
  short errLimit;     ///< error threshold for the 50th block (0x320 at 7200, 0x2bc at 9600)
  short qualityOk;    ///< set when the smoothed error stays below the threshold
  short energyTest;   ///< energy-drop test enabled (1 after create)
  short refLevel;     ///< reference level of the energy-drop test
  short refBlocks;    ///< block counter of the reference update
} V29RxBundle;

/** The receiver engine. */
typedef struct V29Rx
{
  V29RxCfg cfg; ///< configuration words (a copy of the caller's)
  union
  {
    int w;
    unsigned char b[4];
  } st;          ///< status word returned by V29RX_modem(): b[0] status code (V29RxStatus), b[1] flags (0x01 data, 0x02 error, 0x20 carrier, 0x40 / 0x10 set at
                 ///< create, 0x80 low SNR), b[2] bit 0 = idle
  short* fseI;   ///< copy of fse.outI (equalized I coordinates of the last block)
  short* fseQ;   ///< copy of fse.outQ
  short* fseIdx; ///< &bundle->fse.idx
  short* fseTapsI;     ///< copy of fse.tapsI (the equalizer taps)
  short* fseTapsQ;     ///< copy of fse.tapsQ
  short fseNt;         ///< copy of fse.cfg.nt
  int w34;             ///< zero
  int w38;             ///< zero
  short s3c;           ///< zero
  int w40;             ///< zero
  int w44;             ///< zero
  short s48;           ///< zero
  V29RxHdx* hdx;       ///< half-duplex block
  V29RxBundle* bundle; ///< signal-path bundle
} V29Rx;

#if UINTPTR_MAX == 0xffffffffu
_Static_assert(sizeof(V29RxHdx) == 0x58 && offsetof(V29RxHdx, tone) == 4 && offsetof(V29RxHdx, hold) == 8 && offsetof(V29RxHdx, rate) == 0xc &&
                   offsetof(V29RxHdx, s0e) == 0xe && offsetof(V29RxHdx, fn) == 0x10 && offsetof(V29RxHdx, state) == 0x14 && offsetof(V29RxHdx, count) == 0x16 &&
                   offsetof(V29RxHdx, buf) == 0x18 && offsetof(V29RxHdx, s1c) == 0x1c && offsetof(V29RxHdx, s1e) == 0x1e && offsetof(V29RxHdx, mtd2) == 0x20 &&
                   offsetof(V29RxHdx, buf2) == 0x24 && offsetof(V29RxHdx, sinceTone) == 0x28 && offsetof(V29RxHdx, toneSearch) == 0x2a &&
                   offsetof(V29RxHdx, agc) == 0x2c,
               "V29RxHdx 32-bit layout");
_Static_assert(sizeof(V29RxLoop) == 0x20 && offsetof(V29RxLoop, symI0) == 4 && offsetof(V29RxLoop, symI1) == 8 && offsetof(V29RxLoop, symI2) == 0xc &&
                   offsetof(V29RxLoop, rate) == 0x10 && offsetof(V29RxLoop, idx) == 0x14 && offsetof(V29RxLoop, reg) == 0x16 &&
                   offsetof(V29RxLoop, phaseCnt) == 0x18 && offsetof(V29RxLoop, s1a) == 0x1a && offsetof(V29RxLoop, prev) == 0x1c &&
                   offsetof(V29RxLoop, cnt) == 0x1e,
               "V29RxLoop 32-bit layout");
_Static_assert(sizeof(V29RxBundle) == 0x4f6c && offsetof(V29RxBundle, w18) == 0x18 && offsetof(V29RxBundle, eqGate) == 0x20 &&
                   offsetof(V29RxBundle, loop) == 0x28 && offsetof(V29RxBundle, loop.cnt) == 0x46 && offsetof(V29RxBundle, mrf) == 0x48 &&
                   offsetof(V29RxBundle, agc) == 0x64 && offsetof(V29RxBundle, agc.alpha) == 0x70 && offsetof(V29RxBundle, agc.detect) == 0x80 &&
                   offsetof(V29RxBundle, sre) == 0x90 && offsetof(V29RxBundle, sre.pllEnable) == 0xd0 && offsetof(V29RxBundle, sre.loopEnable) == 0xd8 &&
                   offsetof(V29RxBundle, fse) == 0x120 && offsetof(V29RxBundle, fse.cfg.mu) == 0x12e && offsetof(V29RxBundle, fse.eqForce) == 0x160 &&
                   offsetof(V29RxBundle, fse.pllEnable) == 0x164 && offsetof(V29RxBundle, fse.predEnable) == 0x168 &&
                   offsetof(V29RxBundle, fse.eqEnable) == 0x16c && offsetof(V29RxBundle, fse.decErrAvg) == 0x172 && offsetof(V29RxBundle, fse.outI) == 0x174 &&
                   offsetof(V29RxBundle, fse.idx) == 0x17e && offsetof(V29RxBundle, fse.tapsI) == 0x180 && offsetof(V29RxBundle, sdm) == 0x4f3c &&
                   offsetof(V29RxBundle, buf1) == 0x4f54 && offsetof(V29RxBundle, buf2) == 0x4f58 && offsetof(V29RxBundle, decErrSmooth) == 0x4f5c &&
                   offsetof(V29RxBundle, refBlocks) == 0x4f68,
               "V29RxBundle 32-bit layout");
_Static_assert(sizeof(V29Rx) == 0x54 && offsetof(V29Rx, st) == 0x18 && offsetof(V29Rx, fseI) == 0x1c && offsetof(V29Rx, fseNt) == 0x30 &&
                   offsetof(V29Rx, w34) == 0x34 && offsetof(V29Rx, s3c) == 0x3c && offsetof(V29Rx, w40) == 0x40 && offsetof(V29Rx, w44) == 0x44 &&
                   offsetof(V29Rx, s48) == 0x48 && offsetof(V29Rx, hdx) == 0x4c && offsetof(V29Rx, bundle) == 0x50,
               "V29Rx 32-bit layout");
#endif
_Static_assert(sizeof(V29RxCfg) == 24, "V29RxCfg");

/** @name Engine entry points (called by the VMI wrappers below) @{ */
V29Rx* V29RX_create(V29Rx* o, const int* cfg);
void V29RX_delete(V29Rx* o);
int V29RX_modem(V29Rx* o, short* in, short* out, unsigned short* n);
int V29RX_control(V29Rx* o, const FaxV29Ctl* ctl);
int V29RX_status(V29Rx* o, FaxV29Status* st);
/** @} */

/** @name Per-state handlers, state machine and detectors @{ */

int RxHdxStartV29(V29Rx* o, short* in, short* out, unsigned short* n);
int RxHdxEpochDetV29(V29Rx* o, short* in, short* out, unsigned short* n);
int RxHdxPrtcolV29(V29Rx* o, short* in, short* out, unsigned short* n);
int RxHdxDataV29(V29Rx* o, short* in, short* out, unsigned short* n);
int RxHdxIdleV29(V29Rx* o, short* in, short* out, unsigned short* n);
int RxHdxErrorV29(V29Rx* o, short* in, short* out, unsigned short* n);
void RxNextStateV29(V29Rx* o);
int DemodDataV29(V29Rx* o, short* in, short* out, unsigned short n);
int DataCarrierDetectV29(V29Rx* o, short* in, unsigned short n);
int CarrierDetectV29(V29Rx* o);
int EpochDetectV29(V29Rx* o);
int QualityDetectV29(V29Rx* o);
int GetSNRV29(V29Rx* o);
/** @} */

/** Equalizer decision callbacks (FaxFseCb): return the symbol code in the low 16 bits. */
int V29RX_epoch_det(FaxFse* f, short* angle, short* mag);
int V29RX_decision(FaxFse* f, short* angle, short* mag);
int V29RX_eq_train(FaxFse* f, short* angle, short* mag);

/** VMI wrappers (v29rx_*) and the VMI initialiser fax_init_vmi_v29rx(). */
int fax_init_vmi_v29rx(FaxVmiV29* m, unsigned short rate, int unused, void* user);
int v29rx_create(FaxVmiV29* m, const int* cfg); /* returns the bits-per-symbol code 3 (7200) / 4, NOT the engine; the engine goes into m->eng */
void v29rx_delete(FaxVmiV29* m);
int v29rx_process(FaxVmiV29* m, short* in, unsigned short* outn, unsigned short* n);
int v29rx_status(FaxVmiV29* m, FaxV29Status* st);
int v29rx_control(FaxVmiV29* m, const FaxV29Ctl* ctl);
void v29rx_message(FaxVmiV29* m, int idx, const char** out);
#endif
