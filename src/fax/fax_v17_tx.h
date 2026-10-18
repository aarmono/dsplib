/**
 * @file fax_v17_tx.h
 * @brief Typed objects of the fax V.17 transmit half (fax_v17_tx.c): engine instance, half-duplex block, bundle, state / status / rate / encoder enums, control
 *     and status records.
 *
 * Same design as fax_v27_tx.h / fax_v29_tx.h.  The transmitter is three heap objects: the instance V17Tx (configuration block, state word, pointers to the
 * other two), the half-duplex block V17TxHdx (bit FIFO, pattern generator, state machine) and the bundle V17TxBundle (symbol-code descriptor, scrambler, V.17
 * symbol encoder, pulse shaper, the three encoder entry points and the encoder select). They are ordinary structs with real pointer members:
 * in a 32-bit build the member offsets are pinned by the _Static_asserts below (the raw-offset accessors of fax_modem_views.h and the 32-bit layout tests rely
 * on them), on 64-bit hosts the members after a pointer move, so NOTHING may address these objects by a raw byte offset. The handler slot and the three encoder
 * slots are typed function pointers.
 *
 * What is special to V.17: the symbol encoder is the SMCv17_* family (circ.buf receives symbol CODES, circ.i / circ.q stay NULL: the pulse shaper runs in mode
 * 1 and maps the low byte of the code through SMCv17_IMAP4 / QMAP4 (short training, TEP, AB) or through the VTBv17_* maps that SetTxModeV17 installs for the
 * rate (16T / 32 / 64 / 128 point constellations)); the bundle holds the three encoders {SMCv17_encoder_dif, _abs, _tcm} and the encoder select `encmode` (0 =
 * differential, 1 = absolute, 2 = trellis coded), which SetEncoderV17 writes and ModDataV17 uses as the index of the slot to call; four rates (hdx.rate 0 .. 3
 * = 7200 / 9600 / 12000 / 14400 bit/s); a long / short training flag (hdx.flag = cfg word 6).
 *
 * The scrambler wrappers of fax_modem_wrappers.c (ScrambleDataV17 / SeedScramblerV17 / SetEncoderV17) take the engine `V17Tx *` as their `void *obj` and reach
 * the transmit scrambler `bundle->sdm`, the symbol encoder state `bundle->smc` and the encoder select through three accessors declared in fax_modem_views.h:
 * the 32-bit inline accessors read the raw offsets 0x28 -> 0x1c / 0x34 / 0x8c (asserted below to be exactly these members), on 64-bit hosts fax_v17_tx.c
 * defines faxw_v17_txsdm / faxw_v17_txsmc / faxw_v17_encmode over the typed members.
 *
 * The VMI layer (fax_vmi.h) sees the transmitter through the v17tx_* wrappers, which take a FaxVmiV17 (the common modem view FaxVmiM, with the engine handle in
 * `eng`); the initialiser fax_init_vmi_v17tx() is reached through the table fax_init_vmi_data_tx_modem of fax_vmi.c.
 */
#ifndef DSPLIBS_PORT_FAX_V17_OBJ_H
#define DSPLIBS_PORT_FAX_V17_OBJ_H
#include <stddef.h>
#include <stdint.h>
#include "fax_vmi.h"
#include "fax_common.h"
#include "fax_pulse_shaping.h"

/**
 * Configuration block of the transmitter (default V17TX_CFG, 8 words).  s[1] is the baud rate (7200 / 9600 / 12000 / 14400; anything else makes the create
 * report an error), w[2] is the timeout (60000 by default, overwritten by V17TX_control), w[5] (low 16 bits) scales the FIFO size (times 48 symbols), w[6] is
 * copied to hdx.flag at create (V17TX_control copies its w10 here), w[7] goes to the pulse shaper configuration (fax_init_vmi_v17tx stores the 32-bit user word
 * there); b[0x10] bit 2 is set by V17TX_control.
 */
typedef union V17TxCfg
{
  int w[8];
  short s[16];
  unsigned char b[32];
} V17TxCfg;

struct V17Tx;
/** State handler (TxHdx*V17): (engine, scratch / input buffer, output samples, &count) -> number of output samples produced; *count is in / out (16 bit). */
typedef int (*V17TxHdxFn)(struct V17Tx* o, short* in, short* out, unsigned short* n);
/** Symbol encoder slot: SMCv17_encoder_dif / _abs / _tcm (the tcm one masks the input in place; dif / abs are called through thin adaptors). */
typedef void (*V17EncFn)(FaxSmcV17* s, FaxCirc* o, unsigned short* in, unsigned short n);

/**
 * Rate class (V17TxHdx::rate, V17RxHdx::rate, the argument of SetTxModeV17): the index of the per-rate tables.  V17TX_create / V17RX_create map the configured
 * baud rate 7200 -> 0, 9600 -> 1, 12000 -> 2, 14400 -> 3 and any other value -> 3 plus the error flags.
 */
enum V17RateClass
{
  V17RC_7200 = 0,  ///< 7200 bit/s, 16T-point constellation
  V17RC_9600 = 1,  ///< 9600 bit/s, 32 points
  V17RC_12000 = 2, ///< 12000 bit/s, 64 points
  V17RC_14400 = 3  ///< 14400 bit/s, 128 points
};

/** Encoder select (V17TxBundle::encmode): index of the slot in V17TxBundle::enc that ModDataV17 calls; SetTxModeV17 picks TCM, SetEncoderV17 any. */
enum V17EncMode
{
  V17ENC_DIFF = 0, ///< differential
  V17ENC_ABS = 1,  ///< absolute
  V17ENC_TCM = 2   ///< trellis coded
};

/** Transmit state machine state (V17TxHdx::state); the debug text "V17TX_STATE_<name>" printed by TxNextStateV17 names the state being left. */
enum V17TxState
{
  V17TX_STATE_START = 0,      ///< entry state: moves on immediately
  V17TX_STATE_SILENCE = 1,    ///< silence, one frame
  V17TX_STATE_TEP = 2,        ///< TEP segment: SGD pattern 0, absolute encoder, 0x1e0 symbols
  V17TX_STATE_QUIET = 3,      ///< silence, one frame
  V17TX_STATE_ALT = 4,        ///< segment AB: pattern 0xe, absolute encoder, 0x100 symbols
  V17TX_STATE_EQCOND = 5,     ///< equaliser conditioning: pattern 0xf scrambled, 0xba0 symbols (0x26 with short training)
  V17TX_STATE_BRIDGE = 6,     ///< bridge segment: pattern 0x111, differential encoder, 0x40 symbols (long training only)
  V17TX_STATE_SCR1 = 7,       ///< scrambled ones, trellis encoder, one frame
  V17TX_STATE_DATA = 8,       ///< user data from the FIFO, scrambled
  V17TX_STATE_SCR1_END = 9,   ///< scrambled ones after the data, 0x20 symbols
  V17TX_STATE_QUIET_END = 10, ///< silence, one frame
  V17TX_STATE_IDLE = 11       ///< carrier off until the FIFO is refilled
};

/** Status code (V17Tx::st.b[0]); it indexes V17TX_MESG, which v17tx_message returns. */
enum V17TxStatus
{
  V17TX_MSG_DATA = 0,          ///< data mode
  V17TX_MSG_PROTOCOL = 1,      ///< training segments
  V17TX_MSG_CONNECT_14400 = 2, ///< first data block at 14400 bit/s
  V17TX_MSG_CONNECT_12000 = 3, ///< first data block at 12000 bit/s
  V17TX_MSG_CONNECT_9600 = 4,  ///< first data block at 9600 bit/s
  V17TX_MSG_CONNECT_7200 = 5,  ///< first data block at 7200 bit/s
  V17TX_MSG_IDLE = 6,          ///< idle
  V17TX_MSG_ERROR = 7,         ///< internal error (bad rate or state)
  V17TX_MSG_UNDERRUN = 8,      ///< the FIFO ran dry in data mode
  V17TX_MSG_OVERRUN = 9        ///< the input did not fit into the FIFO
};

/** Half-duplex block: the bit queue, the pattern generator and the state machine. */
typedef struct V17TxHdx
{
  FaxFifo* fifo; ///< bit queue (FIFO_create: size = cfg.w[5] low 16 bits * 48, fill 0)
  FaxSgd* sgd;   ///< training pattern generator (SGD_create with SGD_CFG; 2 bits per symbol at create, V17TX_SYM_SIZE[rate] after SetTxModeV17)
  int pend;      ///< nonzero: the input is not queued (control record bit 4) and the under-run check is skipped; the QUIET_END -> IDLE transition sets it
  int flag;      ///< cfg.w[6]: nonzero = short training (no bridge segment, 0x26 instead of 0xba0 equaliser symbols)
  short rate;    ///< rate class V17RateClass (also 3 for any other configured rate, which sets the error flags)
  short s12;     ///< unused
  V17TxHdxFn fn; ///< handler of the current state
  short state;   ///< V17TxState (see TxNextStateV17)
  short count;   ///< symbols left in the current state
  short s1c;     ///< cleared on the ALT -> EQCOND transition, never read
  short nocarr;  ///< "no carrier" symbol code (TxNoCarrierV17): 4 at create, 0x10 / 0x20 / 0x40 / 0x80 (zero point of the rate's map) after SetTxModeV17
} V17TxHdx;

/** Modulator bundle: symbol-code descriptor, scrambler, encoder state, pulse shaper and the encoder dispatch table. */
typedef struct V17TxBundle
{
  int w0, w4;             ///< unused
  FaxCirc circ;           ///< encoder output / pulse shaper input: i = q = NULL, buf = 50 symbol codes, wr = rd = 0, lim = 50
  FaxSdm sdm;             ///< transmit scrambler (ScrambleDataV17 / SeedScramblerV17)
  FaxSmcV17 smc;          ///< symbol encoder state (SetEncoderV17 writes smc.x)
  FaxPps pps;             ///< pulse shaper (mode 1: symbol codes through imap / qmap)
  V17EncFn enc[3];        ///< SMCv17_encoder_dif, _abs, _tcm (index = encmode)
  unsigned short encmode; ///< encoder select V17EncMode; not initialised by the create, set by SetTxModeV17 / SetEncoderV17
} V17TxBundle;

/** The transmitter instance. */
typedef struct V17Tx
{
  V17TxCfg cfg; ///< configuration block
  /**
   * State word.  b[0] is the status code (V17TxStatus); b[1] holds flags (0x01 data state, 0x02 error, 0x08 / 0x10 / 0x40 set at create); bit 0 of b[2] means
   * idle.  V17TX_modem returns the whole word.
   */
  union
  {
    int w;
    unsigned char b[4];
  } st;
  V17TxHdx* hdx;       ///< half-duplex block
  V17TxBundle* bundle; ///< modulator bundle
} V17Tx;

/** The VMI object view used by the v17tx_* wrappers: the common modem view FaxVmiM (fax_vmi.h); `eng` = engine handle, `u10.cfg` = configuration block. */
typedef FaxVmiM FaxVmiV17;

/** Control record of V17TX_control (only the fields it reads). */
typedef struct FaxV17Ctl
{
  int w0;            ///< unused
  int w4;            ///< copied to cfg.w[2]
  int w8;            ///< multiplied by V17TX_PPS_SCALE[rate] to give the pulse shaper gain
  unsigned char b0c; ///< bit 2 sets cfg.b[0x10] bit 2
  unsigned char b0d; ///< bit 4 sets hdx.pend, bit 1 re-creates the engine
  unsigned char pad0e[2];
  int w10; ///< copied to cfg.w[6] (the short-training flag)
} FaxV17Ctl;

/** Status record filled by V17TX_status (28 bytes; the short se is not written). */
typedef struct FaxV17Status
{
  short s0, s2, s4, s6, s8, sa, sc, se, s10, s12; ///< s0 = cfg.s[0], s2 = s10 = baud rate cfg.s[1], the others 0
  unsigned char b14, b15;                         ///< b14 = cfg.b[0x10] & 4; b15 bit 0 is cleared
  int w18;                                        ///< cfg.w[6]
} FaxV17Status;

#if UINTPTR_MAX == 0xffffffffu
_Static_assert(sizeof(V17TxHdx) == 0x20 && offsetof(V17TxHdx, sgd) == 4 && offsetof(V17TxHdx, pend) == 8 && offsetof(V17TxHdx, flag) == 0xc &&
                   offsetof(V17TxHdx, rate) == 0x10 && offsetof(V17TxHdx, fn) == 0x14 && offsetof(V17TxHdx, state) == 0x18 &&
                   offsetof(V17TxHdx, count) == 0x1a && offsetof(V17TxHdx, s1c) == 0x1c && offsetof(V17TxHdx, nocarr) == 0x1e,
               "V17TxHdx 32-bit layout");
_Static_assert(sizeof(V17TxBundle) == 0x90 && offsetof(V17TxBundle, circ) == 8 && offsetof(V17TxBundle, circ.i) == 8 && offsetof(V17TxBundle, circ.q) == 0xc &&
                   offsetof(V17TxBundle, circ.buf) == 0x10 && offsetof(V17TxBundle, circ.wr) == 0x14 && offsetof(V17TxBundle, circ.rd) == 0x16 &&
                   offsetof(V17TxBundle, circ.lim) == 0x18 && offsetof(V17TxBundle, sdm) == 0x1c && offsetof(V17TxBundle, smc) == 0x34 &&
                   offsetof(V17TxBundle, smc.x) == 0x3a && offsetof(V17TxBundle, pps) == 0x48 && offsetof(V17TxBundle, pps.cfg.imap) == 0x58 &&
                   offsetof(V17TxBundle, pps.cfg.scale) == 0x50 && offsetof(V17TxBundle, enc) == 0x80 && offsetof(V17TxBundle, encmode) == 0x8c,
               "V17TxBundle 32-bit layout");
_Static_assert(sizeof(V17Tx) == 0x2c && offsetof(V17Tx, st) == 0x20 && offsetof(V17Tx, hdx) == 0x24 && offsetof(V17Tx, bundle) == 0x28, "V17Tx 32-bit layout");
_Static_assert(sizeof(FaxVmiV17) == 0x18 && offsetof(FaxVmiV17, u4) == 4 && offsetof(FaxVmiV17, s08) == 8 && offsetof(FaxVmiV17, s0c) == 0xc &&
                   offsetof(FaxVmiV17, u10) == 0x10 && offsetof(FaxVmiV17, eng) == 0x14,
               "FaxVmiV17 32-bit layout");
#endif
_Static_assert(sizeof(FaxV17Ctl) == 0x14 && offsetof(FaxV17Ctl, b0d) == 0xd && offsetof(FaxV17Ctl, w10) == 0x10 && sizeof(FaxV17Status) == 0x1c &&
                   offsetof(FaxV17Status, b14) == 0x14 && offsetof(FaxV17Status, w18) == 0x18,
               "fax V.17 records");
_Static_assert(sizeof(V17TxCfg) == 32, "V17TxCfg");

/* ---- the engine entry points ---- */
V17Tx* V17TX_create(V17Tx* o, const int* cfg);
void V17TX_delete(V17Tx* o);
int V17TX_modem(V17Tx* o, short* in, short* out, unsigned short* n);
int V17TX_control(V17Tx* o, const FaxV17Ctl* ctl);
int V17TX_status(V17Tx* o, FaxV17Status* st);

int TxHdxStartV17(V17Tx* o, short* in, short* out, unsigned short* n);
int TxHdxSilenceV17(V17Tx* o, short* in, short* out, unsigned short* n);
int TxHdxTEP_V17(V17Tx* o, short* in, short* out, unsigned short* n);
int TxHdxABV17(V17Tx* o, short* in, short* out, unsigned short* n);
int TxHdxBridgeV17(V17Tx* o, short* in, short* out, unsigned short* n);
int TxHdxEQCondV17(V17Tx* o, short* in, short* out, unsigned short* n);
int TxHdxSCR1V17(V17Tx* o, short* in, short* out, unsigned short* n);
int TxHdxDataV17(V17Tx* o, short* in, short* out, unsigned short* n);
int TxHdxIdleV17(V17Tx* o, short* in, short* out, unsigned short* n);
void TxNextStateV17(V17Tx* o);
int ModDataV17(V17Tx* o, short* in, short* out, unsigned short n);
int TxNoCarrierV17(V17Tx* o, short* in, short* out, unsigned short n);
void SetTxModeV17(V17Tx* o, short rate);

/* the VMI wrappers (v17tx_*) and the VMI initialiser */
int fax_init_vmi_v17tx(FaxVmiV17* m, unsigned short rate, int unused, void* user);
unsigned int v17tx_create(FaxVmiV17* m, const int* cfg); /* returns the rate word (s[1], zero extended), NOT the engine handle (that is stored in m->eng) */
void v17tx_delete(FaxVmiV17* m);
int v17tx_process(FaxVmiV17* m, short* out, unsigned short* n, unsigned short* outn);
int v17tx_status(FaxVmiV17* m, FaxV17Status* st);
int v17tx_control(FaxVmiV17* m, const FaxV17Ctl* ctl);
void v17tx_message(FaxVmiV17* m, int idx, const char** out);
#endif
