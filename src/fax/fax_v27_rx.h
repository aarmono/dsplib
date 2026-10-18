/**
 * @file fax_v27_rx.h
 * @brief Typed objects and entry points of the fax V.27ter receiver (2400 / 4800 bit/s, T.30 high-speed data).
 *
 * The receiver is split into three heap objects, all plain structs with real pointer members:
 * - the engine `V27Rx` (seven configuration words, the status word, copies of the equalizer's buffer pointers, the pointers to the other two blocks),
 * - the half-duplex block `V27RxHdx` (the tone detector of the data states, the handler pointer of the current state and its counters, the carrier
 *   detector with its own AGC),
 * - the bundle `V27RxBundle` (the control gates, the decision-loop state the equalizer callbacks use, the receive scrambler, and the matched filter,
 *   AGC, timing recovery and equalizer sub-objects with their sample buffers).
 * The timing-recovery and equalizer sub-objects are the typed FaxSre / FaxFse of fax_rx_common.h (shared with the V.29 and V.17 receivers).
 *
 * On a 32-bit build every member sits at the byte offset the _Static_asserts pin; on a 64-bit build the members after a pointer move, so NOTHING may
 * address these objects by a raw byte offset.  The one exception is the descrambler wrapper DescrambleDataV27() (fax_modem_wrappers.c), which takes the
 * engine as an opaque `void *` and reaches `bundle->sdm` through the accessor faxw_v27_rxsdm() of fax_modem_views.h; on a 64-bit build fax_v27_rx.c
 * defines that accessor over the typed members.
 *
 * The VMI layer (fax_vmi.c) calls the v27rx_* wrappers with a `FaxVmiV27 *`, the view of the VMI object shared with the transmitter (fax_v27_tx.h).  The
 * receive initialiser stores 0x140 / 0xa5 / 0 / 8 into the shorts at byte offsets 8 .. 0xe and the engine handle into the `eng` slot.
 */
#ifndef DSPLIBS_PORT_FAX_V27RX_OBJ_H
#define DSPLIBS_PORT_FAX_V27RX_OBJ_H
#include <stddef.h>
#include <stdint.h>
#include "fax_common.h"
#include "fax_rx_common.h"
#include "fax_v27_tx.h"
#include "fpm/fpm.h"

/** Configuration block (7 words): `s[2]` is the bit rate (2400 or 4800), `w[2]` (60000) is overwritten by V27RX_control(), `w[5] == 0` selects the long
 *  training (V27RxHdx::longTrain), `w[6]` is the user word that is copied into the matched-filter, timing-recovery and equalizer configurations and is
 *  otherwise unused. */
typedef union V27RxCfg
{
  int w[7];
  short s[14];
  unsigned char b[28];
} V27RxCfg;

struct V27Rx;
/** Half-duplex handler of one receiver state: consumes the whole input block, returns the number of output symbols; `*n` (16 bit) is the number of input
 *  samples on entry and the number left over on return. */
typedef int (*V27RxHdxFn)(struct V27Rx* o, short* in, short* out, unsigned short* n);

/** Receiver state, V27RxHdx::state; RxNextStateV27() prints the name of the state it leaves (ERROR is never printed). */
enum V27RxState
{
  V27RX_STATE_START = 0,     ///< waiting for carrier
  V27RX_STATE_EPOCH_DET = 1, ///< looking for the phase-reversal epoch of the training sequence
  V27RX_STATE_PROTOCOL = 2,  ///< equalizer training
  V27RX_STATE_DATA = 3,      ///< data
  V27RX_STATE_IDLE = 4,      ///< decision quality lost; back to DATA when the error is small again
  V27RX_STATE_ERROR = 5      ///< carrier lost during training
};

/** Status code in V27Rx::st.b[0]; the value indexes the message table V27RX_MESG (the last two entries are the connect messages).  The connect code is
 *  computed as `7 - (rate == V27RC_2400)`: 6 at 2400, 7 at 4800. */
enum V27RxStatus
{
  V27RX_MSG_DATA = 0,
  V27RX_MSG_PROTOCOL = 1,
  V27RX_MSG_START = 2,
  V27RX_MSG_ERROR = 3,
  V27RX_MSG_LOSS_CARRIER = 4,
  V27RX_MSG_IDLE = 5,
  V27RX_MSG_CONNECT_2400 = 6,
  V27RX_MSG_CONNECT_4800 = 7
};

/** Half-duplex block: the per-state handler and the detectors that run in the start and data states. */
typedef struct V27RxHdx
{
  FpmMtd* mtd;      ///< tone detector of the start state (V27_MTD_COEFF_2400 / _4800)
  int hold;         ///< nonzero: RxHdxDataV27() discards its input (control byte 0xd bit 4)
  short rate;       ///< 0 = 2400, 1 = 4800 (also for any other configured rate)
  short longTrain;  ///< 1 when cfg.w[5] == 0: long training (0x3e8 calls of the training callback instead of 0x32)
  V27RxHdxFn fn;    ///< handler of the current state
  short state;      ///< V27RxState; 0 start, 1 epoch detect, 2 protocol, 3 data, 4 idle, 5 error
  short count;      ///< blocks left in the current state
  short s14;        ///< carrier-detector mode, written by the T.30 receiver (cd_flag): 0 = decoder-error test only, else also the V.21 carrier timeout
  short s16;        ///< unused
  FpmMtd* mtd2;     ///< V.21 carrier detector (V21_CHAN2_MTD_COEFF), used by DataCarrierDetectV27()
  short* buf;       ///< 0x140 byte work block: copy of the input block for the carrier detector
  short sinceTone;  ///< samples since the V.21 carrier detector last fired
  short toneSearch; ///< flag: the V.21 carrier search is active
  FpmAgc agc;       ///< AGC of the carrier detector
} V27RxHdx;

/** Decision-loop state shared by the equalizer callbacks V27RX_epoch_det() / V27RX_eq_train() / V27RX_decision() (reached through `fse.cfg.hist`). */
typedef struct V27RxLoop
{
  short symI0, symQ0, symI1, symQ1, symI2, symQ2; ///< the last three (I, Q) pairs of equalized symbols (epoch detector), newest first
  int rate4800;                                   ///< 1 at 4800 bit/s
  int shortTrain;                                 ///< 1 when hdx.longTrain == 0: short training, selects the epoch-detector threshold and the training length
  unsigned short mask;                            ///< phase index mask (3 at 2400, 7 at 4800)
  short idx;                                      ///< phase index
  const unsigned short* pmap;                     ///< phase index step -> symbol code (V27RX_DEC_PMAP[rate])
  short phaseCnt;                                 ///< calls since the epoch detector / the training was (re)started
  short avg;                                      ///< smoothed energy (0x3299 after create)
  const short* last;                              ///< phase index -> angle (V27RX_DEC_LAST_PHASE[rate])
  short prev;                                     ///< previous angle
  short cnt;                                      ///< call counter (wraps from 0x8000 to 0x4000)
} V27RxLoop;

/** Bundle: the control gates, the decision-loop state and the signal-path sub-objects. */
typedef struct V27RxBundle
{
  int w0;             ///< cleared by control byte 0xc bit 3
  int sreGate;        ///< gate of the timing-recovery loop (sre.loopEnable = sreGate & detect)
  int pllGate;        ///< gate of the equalizer carrier PLL (fse.pllEnable = pllGate & detect)
  int wc;             ///< unused
  int eqGate;         ///< gate of the equalizer update (fse.eqEnable = eqGate & detect); cleared by control byte 0xc bit 5
  V27RxLoop loop;     ///< decision-loop state for the equalizer callbacks
  FaxSdmV27 sdm;      ///< receive descrambler (used by DescrambleDataV27())
  FpmMrf mrf;         ///< matched / interpolation filter
  FpmAgc agc;         ///< input AGC
  FaxSre sre;         ///< symbol timing recovery
  FaxFse fse;         ///< fractionally spaced equalizer with carrier PLL and decision callback
  short* buf1;        ///< 0x140 byte work block: matched-filter output
  short* buf2;        ///< 0x148 byte work block: timing-recovery output
  short decErrSmooth; ///< decoder error smoothed over the first blocks (QualityDetectV27())
  short qualBlocks;   ///< number of blocks averaged so far (0 .. 0x33)
  short errLimit;     ///< error threshold for the 50th block (0x1b9e at 2400, 0xd27 at 4800)
  short qualityOk;    ///< set when the smoothed error stays below the threshold
  short energyTest;   ///< energy-drop test enabled (1 after create)
  short refLevel;     ///< reference level of the energy-drop test
  short refBlocks;    ///< block counter of the reference update
} V27RxBundle;

/** The receiver engine. */
typedef struct V27Rx
{
  V27RxCfg cfg; ///< configuration words (a copy of the caller's)
  union
  {
    int w;
    unsigned char b[4];
  } st;          ///< status word returned by V27RX_modem(): b[0] status code (V27RxStatus), b[1] flags (0x01 data, 0x02 error, 0x20 carrier, 0x40 / 0x10 set at
                 ///< create, 0x80 low SNR), b[2] bit 0 = idle
  short* fseI;   ///< copy of fse.outI (equalized I coordinates of the last block)
  short* fseQ;   ///< copy of fse.outQ
  short* fseIdx; ///< &bundle->fse.idx
  short* fseTapsI;     ///< copy of fse.tapsI (the equalizer taps)
  short* fseTapsQ;     ///< copy of fse.tapsQ
  short fseNt;         ///< copy of fse.cfg.nt
  int w38;             ///< zero
  int w3c;             ///< zero
  short s40;           ///< zero
  int w44;             ///< zero
  int w48;             ///< zero
  short s4c;           ///< zero
  V27RxHdx* hdx;       ///< half-duplex block
  V27RxBundle* bundle; ///< signal-path bundle
} V27Rx;

#if UINTPTR_MAX == 0xffffffffu
_Static_assert(sizeof(V27RxHdx) == 0x50 && offsetof(V27RxHdx, hold) == 4 && offsetof(V27RxHdx, rate) == 8 && offsetof(V27RxHdx, longTrain) == 0xa &&
                   offsetof(V27RxHdx, fn) == 0xc && offsetof(V27RxHdx, state) == 0x10 && offsetof(V27RxHdx, count) == 0x12 && offsetof(V27RxHdx, s14) == 0x14 &&
                   offsetof(V27RxHdx, mtd2) == 0x18 && offsetof(V27RxHdx, buf) == 0x1c && offsetof(V27RxHdx, sinceTone) == 0x20 &&
                   offsetof(V27RxHdx, toneSearch) == 0x22 && offsetof(V27RxHdx, agc) == 0x24,
               "V27RxHdx 32-bit layout");
_Static_assert(sizeof(V27RxLoop) == 0x28 && offsetof(V27RxLoop, rate4800) == 0xc && offsetof(V27RxLoop, shortTrain) == 0x10 &&
                   offsetof(V27RxLoop, mask) == 0x14 && offsetof(V27RxLoop, pmap) == 0x18 && offsetof(V27RxLoop, phaseCnt) == 0x1c &&
                   offsetof(V27RxLoop, avg) == 0x1e && offsetof(V27RxLoop, last) == 0x20 && offsetof(V27RxLoop, prev) == 0x24 &&
                   offsetof(V27RxLoop, cnt) == 0x26,
               "V27RxLoop 32-bit layout");
_Static_assert(sizeof(V27RxBundle) == 0x4f58 && offsetof(V27RxBundle, eqGate) == 0x10 && offsetof(V27RxBundle, loop) == 0x14 &&
                   offsetof(V27RxBundle, sdm) == 0x3c && offsetof(V27RxBundle, mrf) == 0x4c && offsetof(V27RxBundle, agc) == 0x68 &&
                   offsetof(V27RxBundle, agc.detect) == 0x84 && offsetof(V27RxBundle, sre) == 0x94 && offsetof(V27RxBundle, sre.pllEnable) == 0xd4 &&
                   offsetof(V27RxBundle, sre.loopEnable) == 0xdc && offsetof(V27RxBundle, fse) == 0x124 && offsetof(V27RxBundle, fse.eqForce) == 0x164 &&
                   offsetof(V27RxBundle, fse.decErrAvg) == 0x176 && offsetof(V27RxBundle, fse.outI) == 0x178 && offsetof(V27RxBundle, fse.idx) == 0x182 &&
                   offsetof(V27RxBundle, fse.tapsI) == 0x184 && offsetof(V27RxBundle, buf1) == 0x4f40 && offsetof(V27RxBundle, buf2) == 0x4f44 &&
                   offsetof(V27RxBundle, decErrSmooth) == 0x4f48 && offsetof(V27RxBundle, refBlocks) == 0x4f54,
               "V27RxBundle 32-bit layout");
_Static_assert(sizeof(V27Rx) == 0x58 && offsetof(V27Rx, st) == 0x1c && offsetof(V27Rx, fseI) == 0x20 && offsetof(V27Rx, fseNt) == 0x34 &&
                   offsetof(V27Rx, w38) == 0x38 && offsetof(V27Rx, s40) == 0x40 && offsetof(V27Rx, w44) == 0x44 && offsetof(V27Rx, s4c) == 0x4c &&
                   offsetof(V27Rx, hdx) == 0x50 && offsetof(V27Rx, bundle) == 0x54,
               "V27Rx 32-bit layout");
#endif
_Static_assert(sizeof(V27RxCfg) == 28, "V27RxCfg");

/** @name Engine entry points (called by the VMI wrappers below) @{ */
V27Rx* V27RX_create(V27Rx* o, const int* cfg);
void V27RX_delete(V27Rx* o);
int V27RX_modem(V27Rx* o, short* in, short* out, unsigned short* n);
int V27RX_control(V27Rx* o, const FaxV27Ctl* ctl);
int V27RX_status(V27Rx* o, void* st);
/** @} */

/** @name Per-state handlers, state machine and detectors @{ */

int RxHdxStartV27(V27Rx* o, short* in, short* out, unsigned short* n);
int RxHdxEpochDetV27(V27Rx* o, short* in, short* out, unsigned short* n);
int RxHdxPrtcolV27(V27Rx* o, short* in, short* out, unsigned short* n);
int RxHdxDataV27(V27Rx* o, short* in, short* out, unsigned short* n);
int RxHdxIdleV27(V27Rx* o, short* in, short* out, unsigned short* n);
int RxHdxErrorV27(V27Rx* o, short* in, short* out, unsigned short* n);
void RxNextStateV27(V27Rx* o);
int DemodDataV27(V27Rx* o, short* in, short* out, unsigned short n);
int DataCarrierDetectV27(V27Rx* o, short* in, unsigned short n);
int CarrierDetectV27(V27Rx* o);
int EpochDetectV27(V27Rx* o);
int QualityDetectV27(V27Rx* o);
int GetSNRV27(V27Rx* o);
/** @} */

/** Equalizer decision callbacks (FaxFseCb): return the symbol code in the low 16 bits. */
int V27RX_epoch_det(FaxFse* f, short* angle, short* mag);
int V27RX_decision(FaxFse* f, short* angle, short* mag);
int V27RX_eq_train(FaxFse* f, short* angle, short* mag);

/** VMI wrappers (v27rx_*) and the VMI initialiser fax_init_vmi_v27rx(). */
int fax_init_vmi_v27rx(FaxVmiV27* m, unsigned short rate, int unused, void* user);
void* v27rx_create(FaxVmiV27* m, const int* cfg); /* returns the engine created by V27RX_create() */
void v27rx_delete(FaxVmiV27* m);
int v27rx_process(FaxVmiV27* m, short* in, unsigned short* outn, unsigned short* n);
int v27rx_status(FaxVmiV27* m, void* st);
int v27rx_control(FaxVmiV27* m, const FaxV27Ctl* ctl);
void v27rx_message(FaxVmiV27* m, int idx, const char** out);
#endif
