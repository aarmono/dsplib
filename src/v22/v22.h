/**
 * @file v22.h
 * @brief V.22 / V.22bis / Bell 212A modem: the engine object, its sub-objects and the function interface of the whole module.
 *
 * The V.22 engine (v22_engine.c) is three heap blocks: the engine proper (V22Engine) with the two pointers `tones` (V22Tones: timer, protocol state,
 * tone generator and detectors) and `core` (V22Core: the signal-processing chain).  The chain, per 160-sample block:
 * - transmit: bits -> scrambler (V22Sdm) -> differential phase encoder (V22Smc) -> circular symbol ring (V22Circ) -> pulse-shaping interpolator
 *   (V22Pps) -> samples;
 * - receive: samples -> sample-rate converter (V22Mrf) -> AGC (FpmAgc) -> symbol timing recovery (V22Sre, v22_timing.c) -> fractionally spaced
 *   equaliser with carrier phase loop and decision (V22Fse, v22_equalizer.c) -> descrambler (V22Sdm) -> bits.
 *
 * The protocol handlers (the call-setup machines v22_originate / v22_answer in v22_call_setup.c, the data state v22_data, the loop-back states in
 * v22_loopback.c and v22_retrain in v22_retrain.c) are selected by `V22Tones::state` and run once per block through V22FP_modem(); a handler's own
 * stage is `V22Tones::sc` (v22_stages.h).  The detectors (v22_detectors.c) and the helper functions around the chain (v22_engine.c) are what the
 * handlers use.  The data-pump glue that the host drives is dp_v22.c.
 *
 * Layouts.  Every object has a fixed 32-bit image layout (the `_Static_assert`s pin the sizes and offsets); on x86-64 the members after a pointer
 * member move, so nothing may address these objects by a raw byte offset.  The FSE and SRE are not initialised from table pointers of a configuration
 * block: the coefficient sources are the engine's own heap buffers (V22FseCfg), so real pointers are used, not CFGP handles.  The configuration
 * blocks (V22PpsCfg, V22MrfCfg, V22SmcCfg) hold real pointers too.
 */
#ifndef DSPLIBS_PORT_V22_OBJ_H
#define DSPLIBS_PORT_V22_OBJ_H
#include <stddef.h>
#include <stdint.h>
#include "core/dsp_access.h"
#include "fpm/fpm.h"
#include "v22_stages.h" /* enums V22OrgStage ... V22RetrainStage: the values of V22Tones.sc */

struct V22Fse;
/**
 * The decision callback of the FSE: FSEv22_decision12 (1200 bit/s) or FSEv22_decision24 (2400 bit/s).  It reads the equalised point of the current
 * symbol, writes the ideal point's angle and magnitude to *angle and *mag, and returns the decoded bits (low 16 bits).
 */
typedef unsigned int (*V22FseDecision)(struct V22Fse* fse, short* angle, short* mag);

/** The FSE configuration block: the two coefficient sources the engine builds; V22_FSE_init() copies the pointers to V22Fse::srcI / srcQ. */
typedef struct V22FseCfg
{
  const short* srcI;
  const short* srcQ;
} V22FseCfg;

/** Receive equaliser + carrier phase loop + per-symbol decision (v22_equalizer.c). */
typedef struct V22Fse
{
  const short* srcI; ///< coefficient source, I part: 0x31 shorts (the taps are srcI >> 2 in reverse order)
  const short* srcQ; ///< coefficient source, Q part
  short mu;          ///< index into v22_fse_mu_tab (adaptation step)
  short gainSel;     ///< index into CRRv22_PLL_K1 / K2 (carrier loop gain set)
  short integ;       ///< carrier PLL integrator
  short pad0e;       ///< (never touched)
  int s10;           ///< set to 0 at init, not read by the engine
  int pllEnable;     ///< carrier phase loop enabled
  int s18;           ///< set to 1 at init, written by DemodDataV22, not read by the engine
  int adaptEnable;   ///< equaliser tap adaptation enabled
  short errAvg;      ///< averaged phase error
  short errPow;      ///< averaged squared error (the signal quality figure)
  short* outI;       ///< allocated for 14 shorts: one equalised I sample per symbol of the last call
  short* outQ;       ///< the same for Q
  short count;       ///< the sample count of the last V22_FSE_receive() call
  short nOut;        ///< symbols produced by the last call
  short* tapsI;      ///< 0x31 taps, I (allocated)
  short* tapsQ;      ///< 0x31 taps, Q (allocated)
  short* hist;       ///< input history, 0x62 samples (allocated)
  short wpos;        ///< write position in hist
  short carrier;     ///< carrier phase
  short phase;       ///< symbol clock phase 0 .. 5
  short s42;         ///< set to 0 at init, not read
  void* buf44;       ///< allocated (0x28 bytes), not used by the engine functions
  void* buf48;       ///< allocated (0x28 bytes), not used by the engine functions
  char pad4c[10];    ///< (never touched)
  short pllCnt;      ///< settling counter: while it is <= 0x30 the PLL integrator is held at 0 (gain set 0, then 1); it counts symbols while the PLL is enabled
  short rem;         ///< samples until the next symbol (init 1)
  short pad5a;
  unsigned short* smc;   ///< the SMC state word the decision callbacks read and write (set by the creator)
  V22FseDecision decide; ///< decision callback (set by the creator, switched by SetRxRate())
} V22Fse;

/** Symbol timing recovery: polyphase interpolator + band-edge IIR filter + timing PLL (v22_timing.c). */
typedef struct V22Sre
{
  short mode;     ///< loop mode 0 / 1 / 2 (settling, coarse, fine; named from the code; index into SRE_ALPHA_AVG ...)
  short integ;    ///< timing PLL integrator
  short avgAngle; ///< averaged timing error angle
  short avgMag;   ///< averaged magnitude of the band-pass filtered (symbol-clock) component: the signal presence measure behind wantTrack
  int wantTrack;  ///< 1 while the signal is tracked (avgMag > 2), 0 when it fell to 1 or below; CarrierDetect() returns it
  int pllFlagC;   ///< 1 at init, 0 once mode 2 has been entered (set to 1 again when the signal is lost); the FSE gates use it
  int pllEnable;  ///< timing PLL enabled
  short len;      ///< taps per phase (0x1b)
  short wpos;     ///< write position in hist
  short* coefs;   ///< 270 shorts, the interpolator's polyphase table (allocated)
  short* hist;    ///< input history, len * 2 shorts (allocated)
  short* state;   ///< the 6-short IIR state (allocated)
  int accX;       ///< band-edge correlation accumulator, cosine part
  int accY;       ///< band-edge correlation accumulator, sine part
  short frac;     ///< fractional part of the interpolator index (0 .. 0x7ff)
  short idx;      ///< interpolator phase (row of coefs, in units of len)
  short blk;      ///< symbols per timing estimate (3 or 0xc)
  short cnt;      ///< settling counter
  short sym;      ///< symbols accumulated
  short clk;      ///< symbol clock 0 .. 5
  short rem;      ///< input samples to take in before the next output symbol (init 1; 0 .. 2 normally, up to 3 at the limit of the timing correction)
} V22Sre;

#if UINTPTR_MAX == 0xffffffffu
_Static_assert(sizeof(V22Fse) == 0x64 && offsetof(V22Fse, srcQ) == 4 && offsetof(V22Fse, mu) == 8 && offsetof(V22Fse, s10) == 0x10 &&
                   offsetof(V22Fse, pllEnable) == 0x14 && offsetof(V22Fse, adaptEnable) == 0x1c && offsetof(V22Fse, errAvg) == 0x20 &&
                   offsetof(V22Fse, outI) == 0x24 && offsetof(V22Fse, outQ) == 0x28 && offsetof(V22Fse, count) == 0x2c && offsetof(V22Fse, tapsI) == 0x30 &&
                   offsetof(V22Fse, tapsQ) == 0x34 && offsetof(V22Fse, hist) == 0x38 && offsetof(V22Fse, wpos) == 0x3c && offsetof(V22Fse, phase) == 0x40 &&
                   offsetof(V22Fse, buf44) == 0x44 && offsetof(V22Fse, buf48) == 0x48 && offsetof(V22Fse, pllCnt) == 0x56 && offsetof(V22Fse, rem) == 0x58 &&
                   offsetof(V22Fse, smc) == 0x5c && offsetof(V22Fse, decide) == 0x60,
               "V22Fse 32-bit layout");
_Static_assert(sizeof(V22Sre) == 0x3c && offsetof(V22Sre, wantTrack) == 8 && offsetof(V22Sre, pllEnable) == 0x10 && offsetof(V22Sre, len) == 0x14 &&
                   offsetof(V22Sre, coefs) == 0x18 && offsetof(V22Sre, hist) == 0x1c && offsetof(V22Sre, state) == 0x20 && offsetof(V22Sre, accX) == 0x24 &&
                   offsetof(V22Sre, frac) == 0x2c && offsetof(V22Sre, blk) == 0x30 && offsetof(V22Sre, clk) == 0x36 && offsetof(V22Sre, rem) == 0x38,
               "V22Sre 32-bit layout");
#endif

/* ======================================== PPS / MRF / IIR / SDM / SMC / detectors ======================================== */
/* The configuration blocks (V22FP_create builds some of them on its stack and patches the pointer words with the engine's heap buffers) are typed structs
 * with REAL pointers: in the 32-bit layout they are word blocks, on x86-64 the pointer members are 8 bytes wide (the table files define them with ordinary
 * pointer initialisers, no CFGP handle). */

/** Configuration of the pulse-shaping interpolator (V22Pps): 16 bytes in the 32-bit layout; the creator replaces coefI / coefQ by the engine's 120-short
 *  buffers. */
typedef struct V22PpsCfg
{
  union
  {
    int w;
    unsigned short s[2];
  } c0; ///< s[0] is added to the phase after every output
  short* coefI;
  short* coefQ;
  int w3;
} V22PpsCfg;
/** Configuration of the receive sample-rate converter (V22Mrf): 8 bytes in the 32-bit layout; the creator replaces `coefs` by the engine's 270-short
 *  coefficient buffer. */
typedef struct V22MrfCfg
{
  short* coefs;
  int w1;
} V22MrfCfg;
/** Configuration of a scrambler / descrambler (V22Sdm), SDMv22_CFG = {4, 14, 17}: s[0] bits per symbol (the creator replaces it by 2 for V.22, 4 for
 *  V.22bis), s[1], s[2] the two feedback taps. */
typedef struct V22SdmCfg
{
  short s[3];
} V22SdmCfg;
/** Configuration of the differential phase encoder (V22Smc), SMCv22_CFG: 44 bytes in the 32-bit layout; FPM_SMC_init() copies it word for word. */
typedef struct V22SmcCfg
{
  int w0;
  int flag;                   ///< != 0: no differential table, `a = x & mask`
  short step, mod;            ///< step added to `b` per symbol; modulus of the output
  unsigned short shift, mask; ///< table index = (x >> shift) & mask
  unsigned short mask2, mask3;
  const short *pmap, *imap, *qmap;
  int w8, w9, w10;
} V22SmcCfg;

/**
 * Transmit pulse-shaping interpolator (v22_pulse_shaping.c).  coefI / coefQ (in `cfg`) are the engine's buffers, reordered in place by the init; imap /
 * qmap are set by the creator after the init (SMCv22_IMAP / QMAP_*); the two histories are allocated (3 shorts each plus room for one shift: 6 shorts).
 */
typedef struct V22Pps
{
  V22PpsCfg cfg;     ///< the configuration block (the init copies it as a whole)
  short flag;        ///< a symbol must be consumed before the next output
  short phase;       ///< interpolation phase
  short fill;        ///< samples in the histories
  short taps;        ///< taps per output (3)
  short* histI;      ///< I history (allocated)
  short* histQ;      ///< Q history (allocated)
  const short* imap; ///< symbol -> I point map (set by the creator)
  const short* qmap; ///< symbol -> Q point map
} V22Pps;

/** The 30-tap polyphase receive sample-rate converter (v22_rx_resampler.c): 20 / 9 input samples per output sample pair. */
typedef struct V22Mrf
{
  V22MrfCfg cfg; ///< the configuration block copied by the init: coefs (the engine's buffer, reordered in place) and w1
  short step;    ///< input samples needed for the next output (init 2)
  short phase;   ///< polyphase row
  short wpos;    ///< write position in buf
  short len;     ///< taps (0x1e)
  short* buf;    ///< input history, 0x3c shorts (allocated)
} V22Mrf;

/** The V.22 IIR section of the receive path (V22IIRFilterInit): numerator, denominator, input history, output history. */
typedef struct V22Iir
{
  short num[4];
  short den[4];
  short x[4];
  short y[4];
} V22Iir;

/** The V.22 scrambler / descrambler (v22_scrambler.c). */
typedef struct V22Sdm
{
  short n;            ///< bits per symbol: shift count of the register
  short tap1;         ///< first feedback tap (cfg[1])
  short tap2;         ///< second feedback tap (cfg[2])
  short pad6;         ///< (never touched)
  unsigned int mask;  ///< (1 << n) - 1
  unsigned int nmask; ///< ~mask
  unsigned int state; ///< shift register
  short sh1;          ///< tap1 - n
  short sh2;          ///< tap2 - n
} V22Sdm;

/** The V.22 differential phase encoder (v22_scrambler.c): a copy of V22SmcCfg plus the two running values. */
typedef struct V22Smc
{
  V22SmcCfg cfg; ///< the configuration block copied by the init
  short a;       ///< previous quadrant value
  short b;       ///< running phase offset (advanced by cfg.step per symbol, modulo cfg.mod)
} V22Smc;

/** The circular symbol ring between the encoder and the pulse shaper: FPM_SMC_encoder() writes `buf[wr]`, V22_PPS_filter() reads symbols from `buf[rd]`
 *  (the low byte of each short). */
typedef struct V22Circ
{
  int w0, w1;          ///< (not used here)
  unsigned short* buf; ///< ring storage (allocated, 12 shorts)
  short wr;            ///< write index
  short rd;            ///< read index
  short lim;           ///< ring length (12)
} V22Circ;

#if UINTPTR_MAX == 0xffffffffu
_Static_assert(sizeof(V22PpsCfg) == 16 && sizeof(V22MrfCfg) == 8 && sizeof(V22SmcCfg) == 0x2c && offsetof(V22SmcCfg, pmap) == 0x14, "cfg layouts");
_Static_assert(sizeof(V22Pps) == 0x28 && offsetof(V22Pps, cfg.coefI) == 4 && offsetof(V22Pps, flag) == 0x10 && offsetof(V22Pps, taps) == 0x16 &&
                   offsetof(V22Pps, histI) == 0x18 && offsetof(V22Pps, imap) == 0x20 && offsetof(V22Pps, qmap) == 0x24,
               "V22Pps 32-bit layout");
_Static_assert(sizeof(V22Mrf) == 0x14 && offsetof(V22Mrf, step) == 8 && offsetof(V22Mrf, len) == 0xe && offsetof(V22Mrf, buf) == 0x10, "V22Mrf 32-bit layout");
_Static_assert(sizeof(V22Iir) == 0x20 && offsetof(V22Iir, y) == 0x18, "V22Iir 32-bit layout");
_Static_assert(sizeof(V22Sdm) == 0x18 && offsetof(V22Sdm, mask) == 8 && offsetof(V22Sdm, state) == 0x10 && offsetof(V22Sdm, sh1) == 0x14 &&
                   offsetof(V22Sdm, sh2) == 0x16,
               "V22Sdm 32-bit layout");
_Static_assert(sizeof(V22Smc) == 0x30 && offsetof(V22Smc, cfg.flag) == 4 && offsetof(V22Smc, cfg.pmap) == 0x14 && offsetof(V22Smc, a) == 0x2c &&
                   offsetof(V22Smc, b) == 0x2e,
               "V22Smc 32-bit layout");
_Static_assert(sizeof(V22Circ) == 0x14 && offsetof(V22Circ, buf) == 8 && offsetof(V22Circ, wr) == 0xc && offsetof(V22Circ, rd) == 0xe &&
                   offsetof(V22Circ, lim) == 0x10,
               "V22Circ 32-bit layout");
#endif

/* ======================================== the V.22 / V.22bis engine (V22FP_*) ======================================== */
/* The engine is THREE heap blocks: the engine proper (0x5c bytes in the 32-bit layout) with two pointers, `tones` and `core`.  Everything the engine
 * functions touch is named; the words only the call-setup / retrain / loop handlers use are plain `int` / `short` members whose names carry their
 * 32-bit image offset ("wNN", "sNN", "bNN").  NOTHING may address these objects by a raw byte offset. */

/**
 * The engine configuration (28 bytes, no pointer: the same on every host).  V22FP_create() copies the default block V22_CFG and patches it from its 7-int
 * argument.  It is the first member of V22Engine.
 */
typedef struct V22EngCfg
{
  short mode;  ///< 0 = originate, 1 = answer, 2 = loop mode (selects the start state and core.txChannel / rxChannel)
  short rate;  ///< 0x4b0 (1200) / 0x960 (2400)
  short rate2; ///< = rate (a second copy: core.rxBis is derived from it)
  short s6;
  int handshakeTimeoutMs; ///< cfg[2] (60000) -> tones.timeout; v22_data's data-mode carrier supervision compares blocks * 20 ms with it (unsigned)
  int gain;               ///< output gain, Q15 (13014): V22FP_modem scales the 160 output samples by it
  unsigned char b10;      ///< bit 0 scrambler on, bit 1 descrambler on, bit 2 -> core.w20 (status flag bit 2: v22_data calls TxClockSync)
  unsigned char
      b11; ///< bit 1 retrain on poor signal quality (cfg[6] / V22FP_control), bit 2 start with the answer tone (cfg[3]), bit 3 data mode directly (cfg[5])
  short s12;
  short loopChannel; ///< the channel (core.txChannel / rxChannel) of the loop mode (1); medium confidence name
  short thr;  ///< energy threshold of the receive path (V22Disconnect table entry 3, 150): DemodDataV22() in the data state treats a lower rms as no signal
  int lossMs; ///< cfg[4] (700): carrier_loss_time in ms (compared as a signed 16-bit value by connect_2400 / connect_1200 / v22_data)
} V22EngCfg;

/** The "tone" block of the engine (0x40 bytes in the 32-bit layout): timer, protocol state word, the V.22 tone generator and detectors, the IIR section. */
typedef struct V22Tones
{
  int timer;     ///< ms counter (ReadGTimer() adds 20 per 160-sample block)
  int timeout;   ///< = cfg.handshakeTimeoutMs (60000): the handshake timeout in ms (the handlers compare ReadGTimer() with it, unsigned)
  short sigAcc;  ///< signal accumulator of the call-setup states (compared as unsigned 16 bit: > 0x3b / > 0x87)
  short onesAcc; ///< ones-detector accumulator (unsigned 16 bit: > 0xe6 = unscrambled ones detected)
  /** The stage within the protocol handler (v22_stages.h: V22OrgStage / V22AnsStage / V22ConnStage / V22LoopStage / V22OrgRmloopStage / V22AnsRmloopStage /
   *  V22RetrainStage); cleared when the engine drops to the data state. */
  short sc;
  short state;   ///< index into V22_PROTOCOL (V22Protocol): 0 data, 1 originate, 2 answer, 3 local loop, 4 / 5 remote loop 2, 6 retrain
  int trained;   ///< RxTrained1200 / 2400 result latched by connect_1200 / connect_2400 (also used as a flag by the remote loop states)
  FpmTone* tone; ///< tone generator (FPM_TONE_create): answer tone, guard tone, and the demodulation oscillator of the originating side
  FpmMtd* mtd1;  ///< detector 1 (MTDv22_CFG: 1980 .. 2040 Hz)
  FpmMtd* mtdS1; ///< detector S1 (MTDs1_CFG)
  FpmMtd* mtd2;  ///< detector 2 (MTDv22_CFG2: 1600 .. 1640 Hz)
  V22Iir* iir;   ///< receive IIR section (allocated, 0x20 bytes)
  short rms;     ///< FPM_rms of the last block (originate stage V22ORG_MEASURE_LEVEL)
  int rmsSum;    ///< sum of the block levels (V22ORG_MEASURE_LEVEL), then their average
  short rmsThr;  ///< level threshold (0x2454): shift is set to 1 when the average exceeds it (unsigned compare)
  short rmsN;    ///< number of levels summed
  short shift;   ///< receive scaling: the 160 input samples are shifted right by it
  short pad36;
  /** Retrain origin: 1 = started locally (connect_2400 / v22_data: carrier back or poor signal quality), 0 = requested by the remote modem (v22_data:
   *  Detect_Retrain); v22_retrain stage V22RETRAIN_SILENCE continues with SCRAMBLED_ONES (1) or ONES_TO_RX_2400 (0). */
  short retrainLocal;
  short pad3a;
  int lossCnt; ///< blocks without carrier (connect_2400 / connect_1200 / v22_data: x 20 ms against cfg.lossMs)
} V22Tones;

/**
 * The "core" block of the engine (0x1f8 bytes in the 32-bit layout): the signal-processing chain.  Transmit: SMC encoder -> circ -> PPS; receive: MRF ->
 * AGC -> SRE -> FSE; `sdm` = scrambler, `sdmRx` = descrambler.
 */
typedef struct V22Core
{
  int w0;         ///< (1)
  int sreGate;    ///< (1) receive path enabled: sre.pllEnable = sreGate & agcIn.active
  int fsePllGate; ///< (1) -> fse.pllEnable
  int fseS18Gate; ///< (1) -> fse.s18 (with the AGC flag; V22_status reports it as `== 0` in flag bit 4)
  int adaptGate;  ///< (1) equaliser adaptation enable (SetAdaptEqV22 sets / clears it; -> fse.adaptEnable with the AGC flag)
  int w14;
  int scrOn;   ///< scrambler enabled
  int descrOn; ///< descrambler enabled
  int w20;     ///< from cfg.b10 bit 2 / V22FP_control: status flag bit 2, makes v22_data run TxClockSync()
  int w24;     ///< (not written by the engine functions)
  short txBis; ///< transmit rate != 1200 (V.22bis: 2400)
  short rxBis; ///< receive rate != 1200
  /** medium confidence name: 1 = originate (low channel: the PPS generator increment is 0x666, half of the 0xccc used otherwise), 2 = answer; loop mode:
   *  cfg.loopChannel */
  short txChannel;
  /** medium confidence name: 2 = originate (receives the high channel: the creator runs V22IIRFilterInit and DemodDataV22 mixes with the demod tone),
   *  1 = answer */
  short rxChannel;
  V22Sdm sdm;            ///< transmit scrambler
  V22Smc smc;            ///< differential phase encoder
  V22Pps pps;            ///< pulse-shaping interpolator
  V22Circ circ;          ///< SMC -> PPS symbol ring
  short* ppsCoefI;       ///< PPS coefficient buffer, I (allocated, 120 shorts)
  short* ppsCoefQ;       ///< PPS coefficient buffer, Q
  V22Mrf mrf;            ///< receive sample-rate converter
  FpmAgc agcIn;          ///< receive AGC (`active` is V22FP_control's bit, `detect` is SignalDetect())
  FpmAgc agc;            ///< detector AGC (Detect_v22)
  V22Sre sre;            ///< symbol timing recovery
  V22Fse fse;            ///< equaliser, carrier loop, decision
  unsigned short fseSmc; ///< the SMC state word of the FSE decision callbacks (fse.smc points here; never initialised by the creator)
  unsigned short pad1ca;
  V22Sdm sdmRx;    ///< descrambler
  short* mrfCoefs; ///< MRF coefficient buffer (allocated, 0x21c bytes), filled by the creator
  short* fseCoefI; ///< FSE coefficient source I (allocated, 0x62 bytes)
  short* fseCoefQ; ///< FSE coefficient source Q
  short rxCount;   ///< samples of the last MRF output
  short* rxBuf;    ///< the MRF output (allocated, 0x154 bytes = 170 shorts)
} V22Core;

/** The engine (0x5c bytes in the 32-bit layout). */
typedef struct V22Engine
{
  V22EngCfg cfg;
  /** the status V22FP_modem returns (V22Msg: 1 = starting, 3 / 4 = connected at 2400 / 1200, ...); a non-zero state zeroes the rx bit count */
  unsigned char state;
  /** status flag bits (byte 1 of the V22FP_modem result): bits 0 .. 2 are cleared at the start of V22FP_modem, 0x40 is set by the creator; the other bits
   *  are set by the handlers */
  unsigned char flags1d;
  unsigned char flags1e; ///< status flag bits (byte 2 of the result): bit 1 is cleared at the start of V22FP_modem, set again by the loop-back states
  unsigned char b1f;     ///< byte 3 of the V22FP_modem result (never set to non-zero by the engine functions)
  short* fseOutI;        ///< = core.fse.outI
  short* fseOutQ;        ///< = core.fse.outQ
  short* fseNOut;        ///< = &core.fse.nOut
  short* fseTapsI;       ///< = core.fse.tapsI
  short* fseTapsQ;       ///< = core.fse.tapsQ
  short s34;             ///< (0x31)
  int w38;
  int w3c;
  short s40;
  int w44;
  int w48;
  short s4c;
  V22Tones* tones;
  V22Core* core;
  int w58; ///< (never written)
} V22Engine;

/* The Detect_v22 view is the real engine (member names tones / core / mtd1 / mtd2 / agc unchanged). */
typedef V22Engine V22DetEngine;
typedef V22Tones V22DetTones;
typedef V22Core V22DetCore;

/**
 * The protocol handlers (v22_data, v22_originate, v22_answer, v22_local_loop, v22_org_rmloop2, v22_ans_rmloop2, v22_retrain): V22FP_modem calls
 * V22_PROTOCOL[tones.state](eng, txBits, out, in, rxBits, &txCount, &rxCount) once per block with the engine's static 16-bit buffers (tx bits in,
 * 160 audio samples out / in, rx bits out).  The return value is ignored.
 */
typedef int (*V22ProtoFn)(V22Engine* eng, short* txBits, short* out, short* in, short* rxBits, short* txCount, short* rxCount);

#if UINTPTR_MAX == 0xffffffffu
_Static_assert(sizeof(V22EngCfg) == 0x1c && offsetof(V22EngCfg, rate2) == 4 && offsetof(V22EngCfg, handshakeTimeoutMs) == 8 &&
                   offsetof(V22EngCfg, gain) == 0xc && offsetof(V22EngCfg, b10) == 0x10 && offsetof(V22EngCfg, b11) == 0x11 &&
                   offsetof(V22EngCfg, loopChannel) == 0x14 && offsetof(V22EngCfg, thr) == 0x16 && offsetof(V22EngCfg, lossMs) == 0x18,
               "V22EngCfg 32-bit layout");
_Static_assert(sizeof(V22Tones) == 0x40 && offsetof(V22Tones, timeout) == 4 && offsetof(V22Tones, sc) == 0xc && offsetof(V22Tones, state) == 0xe &&
                   offsetof(V22Tones, trained) == 0x10 && offsetof(V22Tones, tone) == 0x14 && offsetof(V22Tones, mtd1) == 0x18 &&
                   offsetof(V22Tones, mtdS1) == 0x1c && offsetof(V22Tones, mtd2) == 0x20 && offsetof(V22Tones, iir) == 0x24 &&
                   offsetof(V22Tones, rms) == 0x28 && offsetof(V22Tones, rmsSum) == 0x2c && offsetof(V22Tones, rmsThr) == 0x30 &&
                   offsetof(V22Tones, shift) == 0x34 && offsetof(V22Tones, retrainLocal) == 0x38 && offsetof(V22Tones, lossCnt) == 0x3c,
               "V22Tones 32-bit layout");
_Static_assert(sizeof(V22Core) == 0x1f8 && offsetof(V22Core, sreGate) == 4 && offsetof(V22Core, adaptGate) == 0x10 && offsetof(V22Core, scrOn) == 0x18 &&
                   offsetof(V22Core, descrOn) == 0x1c && offsetof(V22Core, w20) == 0x20 && offsetof(V22Core, txBis) == 0x28 &&
                   offsetof(V22Core, rxChannel) == 0x2e && offsetof(V22Core, sdm) == 0x30 && offsetof(V22Core, smc) == 0x48 && offsetof(V22Core, pps) == 0x78 &&
                   offsetof(V22Core, circ) == 0xa0 && offsetof(V22Core, ppsCoefI) == 0xb4 && offsetof(V22Core, ppsCoefQ) == 0xb8 &&
                   offsetof(V22Core, mrf) == 0xbc && offsetof(V22Core, agcIn) == 0xd0 && offsetof(V22Core, agc) == 0xfc && offsetof(V22Core, sre) == 0x128 &&
                   offsetof(V22Core, fse) == 0x164 && offsetof(V22Core, fseSmc) == 0x1c8 && offsetof(V22Core, sdmRx) == 0x1cc &&
                   offsetof(V22Core, mrfCoefs) == 0x1e4 && offsetof(V22Core, fseCoefI) == 0x1e8 && offsetof(V22Core, fseCoefQ) == 0x1ec &&
                   offsetof(V22Core, rxCount) == 0x1f0 && offsetof(V22Core, rxBuf) == 0x1f4,
               "V22Core 32-bit layout");
_Static_assert(sizeof(V22Engine) == 0x5c && offsetof(V22Engine, state) == 0x1c && offsetof(V22Engine, flags1d) == 0x1d &&
                   offsetof(V22Engine, fseOutI) == 0x20 && offsetof(V22Engine, fseNOut) == 0x28 && offsetof(V22Engine, fseTapsQ) == 0x30 &&
                   offsetof(V22Engine, s34) == 0x34 && offsetof(V22Engine, w38) == 0x38 && offsetof(V22Engine, s40) == 0x40 &&
                   offsetof(V22Engine, w44) == 0x44 && offsetof(V22Engine, s4c) == 0x4c && offsetof(V22Engine, tones) == 0x50 &&
                   offsetof(V22Engine, core) == 0x54 && offsetof(V22Engine, w58) == 0x58,
               "V22Engine 32-bit layout");
#endif

/**
 * The block V22_status() fills: the caller's `short st[12]` (0x18 bytes).  It holds the state word, the two rates, a quality figure, and the flag bytes
 * that v22_data and the call setup read.  Flag byte `flags`: bit 0 scramble, 1 descramble, 2 = core.w20 (TxClockSync), 3 / 4 / 5 = the gate words
 * w0 / fseS18Gate / adaptGate are zero, 6 always set, 7 = cfg.b11 bit 1 (retrain on poor quality); `flags2` bit 0 = cfg.b11 bit 2 (answer tone).  The
 * members that V22_status only clears are offset-named.  Addressed through MEMBER_IDX / MEMBER_AS_OF (core/dsp_access.h) on the `short *` / byte-pointer
 * views of the block.
 */
typedef struct V22Status
{
  short state;          ///< v22_status_map[tone generator state]
  short txRate;         ///< 0x960 (2400) or 0x4b0 (1200)
  short rxRate;         ///< 0x960 or 0x4b0
  short quality;        ///< 0x800 minus the (scaled) equalizer error power
  short w08;            ///< cleared
  short w0a;            ///< cleared
  short unk0c_[2];      ///< not written
  short w10;            ///< cleared
  short w12;            ///< cleared
  unsigned char flags;  ///< see above
  unsigned char flags2; ///< see above
  short unk16_;         ///< not written
} V22Status;
_Static_assert(sizeof(V22Status) == 0x18 && offsetof(V22Status, quality) == 6 && offsetof(V22Status, w08) == 8 && offsetof(V22Status, w10) == 0x10 &&
                   offsetof(V22Status, flags) == 0x14 && offsetof(V22Status, flags2) == 0x15,
               "V22Status layout");

/* ---- the engine functions (src/v22/v22_engine.c; documented at their definitions) ---- */
V22Engine* V22FP_create(V22Engine* eng, const int* cfg); /* eng == NULL: allocate the three blocks; else re-initialise in place */
void V22FP_delete(V22Engine* eng);
unsigned int V22FP_modem(V22Engine* eng, const int* txBits, short* out, const short* in, int* rxBits, const unsigned int* txCount, unsigned int* rxCount);
int V22FP_control(V22Engine* eng, const unsigned char* ctl);
int V22FP_GetDiagnostics(V22Engine* eng, int mode, int* out, int max);
int V22_status(V22Engine* eng, V22Status* st);
void MakeTxData(short* out, const short* count, short pattern);
unsigned int RxTrained1200(const short* data, const unsigned short* count);
unsigned int RxTrained2400(const short* data, const unsigned short* count);
void TxNOP(V22Engine* eng, void* unused, short* out, short* count);
void RxClampV22(V22Engine* eng, void* unused, short* out, short* count);
int ReadGTimer(V22Engine* eng);
void SetTxRate(V22Engine* eng, short rate);
void SetRxRate(V22Engine* eng, short rate);
void ScrambleDataV22(V22Engine* eng, unsigned short* buf, unsigned short n);
void DescrambleDataV22(V22Engine* eng, unsigned short* buf, unsigned short n);
unsigned int ModDataV22(V22Engine* eng, const unsigned short* bits, short* out, unsigned short n);
unsigned int DemodDataV22(V22Engine* eng, short* in, short* out, unsigned short n);
void ResetRx(V22Engine* eng);
void SetAdaptEqV22(V22Engine* eng, unsigned short mode);
void TxClockSync(V22Engine* eng);
int CarrierDetect(V22Engine* eng);
int SignalDetect(V22Engine* eng);
unsigned int GetSignalQuality(V22Engine* eng);
int ScramblerOn(V22Engine* eng);
int DescramblerOn(V22Engine* eng);
/* the handlers V22FP_modem dispatches to: v22_originate / v22_answer are in src/v22/v22_call_setup.c, v22_data in src/v22/v22_data.c,
 * v22_local_loop / v22_org_rmloop2 / v22_ans_rmloop2 in src/v22/v22_loopback.c, v22_retrain in src/v22/v22_retrain.c: see V22ProtoFn */
int v22_data(V22Engine* eng, short* txBits, short* out, short* in, short* rxBits, short* txCount, short* rxCount);
int v22_originate(V22Engine* eng, short* txBits, short* out, short* in, short* rxBits, short* txCount, short* rxCount);
int v22_answer(V22Engine* eng, short* txBits, short* out, short* in, short* rxBits, short* txCount, short* rxCount);
int v22_local_loop(V22Engine* eng, short* txBits, short* out, short* in, short* rxBits, short* txCount, short* rxCount);
int v22_org_rmloop2(V22Engine* eng, short* txBits, short* out, short* in, short* rxBits, short* txCount, short* rxCount);
int v22_ans_rmloop2(V22Engine* eng, short* txBits, short* out, short* in, short* rxBits, short* txCount, short* rxCount);
int v22_retrain(V22Engine* eng, short* txBits, short* out, short* in, short* rxBits, short* txCount, short* rxCount);
/* the tail states of originate and answer (same 7-argument convention; they return nothing) */
void connect_2400(V22Engine* eng, short* txBits, short* out, short* in, short* rxBits, short* txCount, short* rxCount);
void connect_1200(V22Engine* eng, short* txBits, short* out, short* in, short* rxBits, short* txCount, short* rxCount);

/* ---- the chain components (documented at their definitions) ---- */
void V22_PPS_init(V22Pps* pps, const V22PpsCfg* cfg, int allocate);
void V22_PPS_free(V22Pps* pps);
unsigned int V22_PPS_filter(V22Pps* pps, V22Circ* src, short* out, unsigned short n); /* number of output samples (16 bit) */
void V22_MRF_init(V22Mrf* mrf, const V22MrfCfg* cfg, int allocate);
void V22_MRF_free(V22Mrf* mrf);
unsigned int V22_MRF_filter(V22Mrf* mrf, const short* in, short* out, short n); /* number of output samples (16 bit) */
void V22IIRFilterInit(V22Iir* f, const short* num, const short* den);
void V22_iir_filt_demod(short* data, const short* num, const short* den, short* xh, short* yh, const short* gain); /* 160 samples in place */
void FPM_SDM_init(V22Sdm* sdm, const V22SdmCfg* cfg);
void FPM_SDM_scrambler(V22Sdm* sdm, unsigned short* buf, unsigned short n);
void FPM_SDM_descrambler(V22Sdm* sdm, unsigned short* buf, unsigned short n);
void FPM_SMC_init(V22Smc* smc, const V22SmcCfg* cfg);
void FPM_SMC_encoder(V22Smc* smc, V22Circ* out, const unsigned short* in, unsigned short n);
unsigned int Detect_Retrain(const unsigned short* data, const unsigned short* pn);
unsigned int Detect_Rmloop2_ACK(const unsigned short* data, const unsigned short* pn, short mode);
unsigned int Detect_1s(const unsigned short* data, const unsigned short* pn, short mode, short factor);
unsigned int Detect_v22(V22DetEngine* eng, short* buf);
unsigned int Detect_v22_core(FpmAgc* agc, FpmMtd* mtd1, FpmMtd* mtd2, short* buf); /* the body of Detect_v22 on the parts it uses */

void V22_FSE_init(V22Fse* fse, const V22FseCfg* cfg, int allocate);
void V22_FSE_free(V22Fse* fse);
int V22_FSE_getdiag(V22Fse* fse, int mode, int* out, int max);                  /* always 0 */
int V22_FSE_receive(V22Fse* fse, const short* in, short* outBits, short count); /* number of symbols (zero-extended 16 bits) */
unsigned int FSEv22_decision12(V22Fse* fse, short* angle, short* mag);
unsigned int FSEv22_decision24(V22Fse* fse, short* angle, short* mag);

void V22_SRE_init(V22Sre* sre, int allocate);
void V22_SRE_free(V22Sre* sre);
unsigned int V22_SRE_recover(V22Sre* sre, const short* in, short* out, short count); /* number of output samples (zero-extended 16 bits) */

#endif
