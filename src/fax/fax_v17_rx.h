/**
 * @file fax_v17_rx.h
 * @brief Typed objects and entry points of the fax V.17 receiver (7200 / 9600 / 12000 / 14400 bit/s, trellis coded).
 *
 * The receiver is three heap objects, all ordinary structs with real pointer members:
 * - V17Rx, the instance the host holds: the configuration (V17RxCfg), the status word, copies of the equalizer's buffer pointers (for the status/debug
 *   readers), and the pointers to the two blocks below;
 * - V17RxHdx, the half-duplex block: the tone killer and tone detector of the start state, the current state's handler (V17RxHdxFn) with its counters, and a
 *   carrier detector with its own AGC;
 * - V17RxBundle, the signal path: gate words, the decision-history object of the FSE callbacks (FaxDec17, with the Viterbi decoder state and the receiver's
 *   SGD), the matched filter (FpmMrf), the AGC, the symbol-timing recovery (FaxSre), the equalizer (FaxFse), the receive descrambler (FaxSdm), two sample
 *   buffers and the quality-detector state.
 *
 * The SRE / FSE sub-objects are the typed FaxSre / FaxFse of fax_rx_common.h, shared with the V.27ter and V.29 receivers.  V.17 has no decision callbacks of
 * its own in this file: the receiver installs FAX_FSE_decision_AB first and the callbacks of fax_v17_decision.h take over (AB, then FSE_decision_eqtrn,
 * FSE_Bridge_det, FSEv17_decision[sel]).
 *
 * Differences from the V.29 receiver (fax_v29_rx.h): four rates (hdx.rate 0 .. 3 = 7200 / 9600 / 12000 / 14400 bit/s); seven handler states (start, epoch
 * detect, protocol (long training: trellis / bridge segments), bridge, scrambler-1 (SCRAM), data, idle; error = 7); a long / short training flag (cfg.w[5],
 * copied to hdx.flag).  With short training the receiver starts from the equalizer taps of a previous call, held in three caller-supplied buffers (cfg.coefI
 * / coefQ / freq), and stores them again after the training (StoreCoefV17() / Restore_rateV17()).  The receiver owns a Viterbi decoder (FaxDec17.vtb) and an
 * SGD (FaxDec17.sgd).
 *
 * On a 32-bit build every member sits at the byte offset that the `_Static_assert`s pin; on a 64-bit build the members after a pointer move, so nothing may
 * address these objects by a raw byte offset.  The descrambler wrapper DescrambleDataV17() (fax_modem_wrappers.c) takes the engine `V17Rx *` as an opaque
 * pointer and reaches `bundle->sdm` through the single accessor of fax_modem_views.h (faxw_v17_rxsdm); fax_v17_rx.c defines it over the typed members on a
 * 64-bit build.
 *
 * The VMI layer (fax_vmi.h) sees the v17rx_*() wrappers, which take a `FaxVmiV17 *` (fax_v17_tx.h: a view of the first 0x18 bytes of the VMI object).  The
 * receive initialiser stores 0x140 / 0xa5 / 0 / 0xc into the shorts at +8 .. +0xe, the engine handle `V17Rx *` into `eng` and the configuration block
 * `V17RxCfg *` into the `u10` slot; FaxVmiV17 types both for the transmitter and the receiver code casts them.
 */
#ifndef DSPLIBS_PORT_FAX_V17RX_OBJ_H
#define DSPLIBS_PORT_FAX_V17RX_OBJ_H
#include <stddef.h>
#include <stdint.h>
#include "fax_common.h"
#include "fax_rx_common.h"
#include "fax_v17_decision.h"
#include "fax_v17_tx.h"
#include "fpm/fpm.h"

/**
 * @brief Receiver configuration block (default V17RX_CFG).
 *
 * `s[2]` is the baud rate (valid: 7200 / 9600 / 12000 / 14400); `w[2]` (60000) is overwritten by V17RX_control(); `w[5]` is the training flag
 * (V17RX_control() copies the control record's `w10` here): 0 = long training, non-zero = short training with the equalizer taps of a previous call.
 *
 * @warning With a non-zero training flag the caller must supply `coefI`, `coefQ` and `freq`; V17RX_create() reads and the receiver writes them unchecked.
 *   The default block has NULL pointers.
 */
typedef struct V17RxCfg
{
  union
  {
    int w[6];
    short s[12];
    unsigned char b[24];
  };
  short* coefI; ///< caller buffer, 0x31 shorts: equalizer taps of the I filter (read by V17RX_create() with short training, written by StoreCoefV17())
  short* coefQ; ///< caller buffer, 0x31 shorts: the Q filter's taps
  short* freq;  ///< caller buffer, one short: the carrier phase integral (read by Restore_rateV17(), written by StoreCoefV17())
  int user;     ///< user word, copied into the MRF / SRE / FSE configurations and otherwise unused
} V17RxCfg;

struct V17Rx;
/** Half-duplex state handler (RxHdx*V17): (engine, input samples, output symbols, &count) -> number of output symbols; `*count` is in / out (16 bit). */
typedef int (*V17RxHdxFn)(struct V17Rx* o, short* in, short* out, unsigned short* n);

/**
 * V17RxHdx::state.  START .. IDLE are the states RxNextStateV17() prints as "V17RX_STATE_<name>" when it leaves them; ERROR (7) is only ever entered on
 * loss of carrier.
 */
enum V17RxState
{
  V17RX_STATE_START = 0,
  V17RX_STATE_EPOCH_DET = 1,
  V17RX_STATE_PROTOCOL = 2,
  V17RX_STATE_BRIDGE = 3,
  V17RX_STATE_SCRAM = 4,
  V17RX_STATE_DATA = 5,
  V17RX_STATE_IDLE = 6,
  V17RX_STATE_ERROR = 7
};

/**
 * V17Rx::st.b[0], the status code: the index into V17RX_MESG (data, protocol, start, internal error, loss of carrier, idle, "CONNECT: V.17 Receive 14400 /
 * 12000 / 9600 / 7200 bps"). The 14400 / 12000 codes are stored as `6 + (rate == V17RC_12000)`.
 */
enum V17RxStatus
{
  V17RX_MSG_DATA = 0,
  V17RX_MSG_PROTOCOL = 1,
  V17RX_MSG_START = 2,
  V17RX_MSG_ERROR = 3,
  V17RX_MSG_LOSS_CARRIER = 4,
  V17RX_MSG_IDLE = 5,
  V17RX_MSG_CONNECT_14400 = 6,
  V17RX_MSG_CONNECT_12000 = 7,
  V17RX_MSG_CONNECT_9600 = 8,
  V17RX_MSG_CONNECT_7200 = 9
};

/** Half-duplex block of the receiver (0x5c bytes on a 32-bit build). */
typedef struct V17RxHdx
{
  FpmMtd* mtd;   ///< tone detector of the start state (V17_MTD_COEFF)
  FpmTone* tone; ///< tone killer in front of the detector
  int hold;      ///< non-zero: RxHdxDataV17() discards its input (control record: bit 4 of byte 0xd)
  short rate;    ///< 0 = 7200, 1 = 9600, 2 = 12000, 3 = 14400 (also set for an invalid configured rate)
  short s0e;     ///< unused
  int flag;      ///< cfg.w[5]: non-zero = short training (Restore_rateV17() / StoreCoefV17(), extra epoch / carrier tests)
  V17RxHdxFn fn; ///< handler of the current state
  short state;   ///< V17RxState: 0 start, 1 epoch detect, 2 protocol, 3 bridge, 4 scrambler (SCRAM), 5 data, 6 idle, 7 error (see RxNextStateV17())
  short count;   ///< blocks left in the current state
  short* buf;    ///< 0x140 bytes: copy of the input block for the tone killer
  short s20;     ///< non-zero selects the data-state carrier test of DataCarrierDetectV17() (the V.21 carrier search); cleared by V17RX_create() and never set
  short s22;     ///< unused
  FpmMtd* mtd2;  ///< carrier detector (V21_CHAN2_MTD_COEFF), used by DataCarrierDetectV17()
  short* buf2;   ///< 0x140 bytes: copy of the input block for the carrier detector
  short sinceTone;  ///< samples since the last tone detection
  short toneSearch; ///< flag: the carrier detector is running
  FpmAgc agc;       ///< AGC of the carrier detector
} V17RxHdx;

/** Signal path of the receiver (0x4fbc bytes on a 32-bit build). */
typedef struct V17RxBundle
{
  int w0;       ///< cleared by control byte 0xc bit 3 (status record bit 3 = (w0 == 0))
  int sreGate;  ///< gate of the SRE loop (sre.loopEnable = sreGate & detect)
  int pllGate;  ///< gate of the FSE carrier PLL (fse.pllEnable = pllGate & detect)
  int wc;       ///< unused
  int eqGate;   ///< gate of the FSE equalizer update (fse.eqEnable = eqGate & detect); control byte 0xc bit 5 clears it (status record bit 5 = (eqGate == 0))
  int w14;      ///< unused
  int w18;      ///< unused
  int w1c;      ///< status record bit 1 = w1c & 1
  int w20;      ///< unused
  short rate24; ///< copy of hdx.rate
  short pad26;  ///< unused
  int w28;      ///< unused
  FaxDec17
      dec; ///< the object the FSE callbacks find through fse.cfg.hist: start-of-training / training history, the Viterbi decoder (dec.vtb), the SGD (dec.sgd)
  FpmMrf mrf;         ///< matched / interpolation filter
  FpmAgc agc;         ///< input AGC
  FaxSre sre;         ///< symbol-timing recovery
  FaxFse fse;         ///< equalizer, carrier PLL and decision loop
  FaxSdm sdm;         ///< receive descrambler (DescrambleDataV17())
  short* buf1;        ///< 0x140 bytes: MRF output
  short* buf2;        ///< 0x148 bytes: SRE output
  short decErrSmooth; ///< smoothed decoder error (QualityDetectV17())
  short qualBlocks;   ///< number of blocks averaged (0 .. 0x33)
  short errLimit;     ///< error threshold of the 50th block (0xa28 / 0x514 / 0x341 / 0x1c2 for 7200 / 9600 / 12000 / 14400)
  short qualityOk;    ///< set when the smoothed error stays below the threshold
  short energyTest;   ///< energy-drop test enabled (1 after create)
  short refLevel;     ///< reference level of the energy-drop test
  short refBlocks;    ///< block counter of the reference update
  short pad4fba;      ///< unused
} V17RxBundle;

/** The receiver instance (0x64 bytes on a 32-bit build). */
typedef struct V17Rx
{
  V17RxCfg cfg; ///< configuration
  union
  {
    int w;
    unsigned char b[4];
  } st; ///< status word.  b[0] = V17RxStatus code; b[1] flags (0x01 data, 0x02 error, 0x20 carrier, 0x40 / 0x10 set at create, 0x80 low SNR); b[2] bit 0 = idle
  short* fseI;         ///< copy of fse.outI (the equalized I coordinates of the last block)
  short* fseQ;         ///< copy of fse.outQ
  short* fseIdx;       ///< &bundle->fse.idx
  short* fseTapsI;     ///< copy of fse.tapsI (the equalizer taps)
  short* fseTapsQ;     ///< copy of fse.tapsQ
  short fseNt;         ///< copy of fse.cfg.nt
  int w44;             ///< cleared at create, unused
  int w48;             ///< cleared at create, unused
  short s4c;           ///< cleared at create, unused
  int w50;             ///< cleared at create, unused
  int w54;             ///< cleared at create, unused
  short s58;           ///< cleared at create, unused
  V17RxHdx* hdx;       ///< half-duplex block
  V17RxBundle* bundle; ///< signal path
} V17Rx;

#if UINTPTR_MAX == 0xffffffffu
_Static_assert(sizeof(V17RxCfg) == 0x28 && offsetof(V17RxCfg, coefI) == 0x18 && offsetof(V17RxCfg, coefQ) == 0x1c && offsetof(V17RxCfg, freq) == 0x20 &&
                   offsetof(V17RxCfg, user) == 0x24,
               "V17RxCfg 32-bit layout");
_Static_assert(sizeof(V17RxHdx) == 0x5c && offsetof(V17RxHdx, tone) == 4 && offsetof(V17RxHdx, hold) == 8 && offsetof(V17RxHdx, rate) == 0xc &&
                   offsetof(V17RxHdx, s0e) == 0xe && offsetof(V17RxHdx, flag) == 0x10 && offsetof(V17RxHdx, fn) == 0x14 && offsetof(V17RxHdx, state) == 0x18 &&
                   offsetof(V17RxHdx, count) == 0x1a && offsetof(V17RxHdx, buf) == 0x1c && offsetof(V17RxHdx, s20) == 0x20 && offsetof(V17RxHdx, s22) == 0x22 &&
                   offsetof(V17RxHdx, mtd2) == 0x24 && offsetof(V17RxHdx, buf2) == 0x28 && offsetof(V17RxHdx, sinceTone) == 0x2c &&
                   offsetof(V17RxHdx, toneSearch) == 0x2e && offsetof(V17RxHdx, agc) == 0x30,
               "V17RxHdx 32-bit layout");
_Static_assert(sizeof(V17RxBundle) == 0x4fbc && offsetof(V17RxBundle, eqGate) == 0x10 && offsetof(V17RxBundle, w1c) == 0x1c &&
                   offsetof(V17RxBundle, rate24) == 0x24 && offsetof(V17RxBundle, w28) == 0x28 && offsetof(V17RxBundle, dec) == 0x2c &&
                   offsetof(V17RxBundle, dec.ctr) == 0x94 && offsetof(V17RxBundle, dec.vtb) == 0x30 && offsetof(V17RxBundle, mrf) == 0x98 &&
                   offsetof(V17RxBundle, agc) == 0xb4 && offsetof(V17RxBundle, agc.alpha) == 0xc0 && offsetof(V17RxBundle, agc.detect) == 0xd0 &&
                   offsetof(V17RxBundle, sre) == 0xe0 && offsetof(V17RxBundle, sre.pllEnable) == 0x120 && offsetof(V17RxBundle, sre.loopEnable) == 0x128 &&
                   offsetof(V17RxBundle, fse) == 0x170 && offsetof(V17RxBundle, fse.cfg.mu) == 0x17e && offsetof(V17RxBundle, fse.cfg.settleCount) == 0x18c &&
                   offsetof(V17RxBundle, fse.eqForce) == 0x1b0 && offsetof(V17RxBundle, fse.pllEnable) == 0x1b4 &&
                   offsetof(V17RxBundle, fse.predEnable) == 0x1b8 && offsetof(V17RxBundle, fse.eqEnable) == 0x1bc &&
                   offsetof(V17RxBundle, fse.decErrAvg) == 0x1c2 && offsetof(V17RxBundle, fse.outI) == 0x1c4 && offsetof(V17RxBundle, fse.idx) == 0x1ce &&
                   offsetof(V17RxBundle, fse.tapsI) == 0x1d0 && offsetof(V17RxBundle, fse.freqAcc) == 0x1ac && offsetof(V17RxBundle, fse.settleCtr) == 0x1f8 &&
                   offsetof(V17RxBundle, sdm) == 0x4f8c && offsetof(V17RxBundle, buf1) == 0x4fa4 && offsetof(V17RxBundle, buf2) == 0x4fa8 &&
                   offsetof(V17RxBundle, decErrSmooth) == 0x4fac && offsetof(V17RxBundle, refBlocks) == 0x4fb8,
               "V17RxBundle 32-bit layout");
_Static_assert(sizeof(V17Rx) == 0x64 && offsetof(V17Rx, st) == 0x28 && offsetof(V17Rx, fseI) == 0x2c && offsetof(V17Rx, fseNt) == 0x40 &&
                   offsetof(V17Rx, w44) == 0x44 && offsetof(V17Rx, w48) == 0x48 && offsetof(V17Rx, s4c) == 0x4c && offsetof(V17Rx, w50) == 0x50 &&
                   offsetof(V17Rx, w54) == 0x54 && offsetof(V17Rx, s58) == 0x58 && offsetof(V17Rx, hdx) == 0x5c && offsetof(V17Rx, bundle) == 0x60,
               "V17Rx 32-bit layout");
#endif

/* ---- engine entry points (see fax_v17_rx.c for the contracts) ---- */
V17Rx* V17RX_create(V17Rx* o, const V17RxCfg* cfg);
void V17RX_delete(V17Rx* o);
int V17RX_modem(V17Rx* o, short* in, short* out, unsigned short* n);
int V17RX_control(V17Rx* o, const FaxV17Ctl* ctl);
int V17RX_status(V17Rx* o, FaxV17Status* st);

/* ---- state handlers and helpers of the receiver ---- */
int RxHdxStartV17(V17Rx* o, short* in, short* out, unsigned short* n);
int RxHdxEpochDetV17(V17Rx* o, short* in, short* out, unsigned short* n);
int RxHdxPrtcolV17(V17Rx* o, short* in, short* out, unsigned short* n);
int RxHdxBridgeV17(V17Rx* o, short* in, short* out, unsigned short* n);
int RxHdxScramV17(V17Rx* o, short* in, short* out, unsigned short* n);
int RxHdxDataV17(V17Rx* o, short* in, short* out, unsigned short* n);
int RxHdxIdleV17(V17Rx* o, short* in, short* out, unsigned short* n);
int RxHdxErrorV17(V17Rx* o, short* in, short* out, unsigned short* n);
void RxNextStateV17(V17Rx* o);
int DemodDataV17(V17Rx* o, short* in, short* out, unsigned short n);
int DataCarrierDetectV17(V17Rx* o, short* in, unsigned short n);
int CarrierDetectV17(V17Rx* o);
int EpochDetectV17(V17Rx* o);
int QualityDetectV17(V17Rx* o);
int GetSNRV17(V17Rx* o);
void Restore_rateV17(V17Rx* o);
void StoreCoefV17(V17Rx* o);

/* ---- VMI wrappers (v17rx_*) and the initialiser (see fax_v17_rx.c) ---- */
int fax_init_vmi_v17rx(FaxVmiV17* m, unsigned short rate, int unused, void* user);
int v17rx_create(FaxVmiV17* m, const V17RxCfg* cfg);
void v17rx_delete(FaxVmiV17* m);
int v17rx_process(FaxVmiV17* m, short* in, unsigned short* outn, unsigned short* n);
int v17rx_status(FaxVmiV17* m, FaxV17Status* st);
int v17rx_control(FaxVmiV17* m, const FaxV17Ctl* ctl);
void v17rx_message(FaxVmiV17* m, int idx, const char** out);
#endif
