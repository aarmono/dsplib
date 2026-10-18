/**
 * @file fax_v27_tx.h
 * @brief Typed objects of the fax V.27ter transmit half (fax_v27_tx.c): engine instance, half-duplex block, bundle, state and status enums, control / status
 *     records.
 *
 * The transmitter is three heap objects: the instance V27Tx (configuration block, state word, pointers to the other two), the half-duplex block V27TxHdx (bit
 * FIFO, pattern generator, state machine: handler pointer and counters) and the bundle V27TxBundle (symbol-code descriptor, scrambler, symbol encoder and pulse
 * shaper).  They are ordinary structs with real pointer members: in a 32-bit build the member offsets are pinned by the _Static_asserts below (the raw-offset
 * accessors of fax_modem_views.h and the 32-bit layout tests rely on them), on 64-bit hosts the members after a pointer move, so NOTHING may address these
 * objects by a raw byte offset.  The handler slot is a typed function pointer.
 *
 * The scrambler wrappers of fax_modem_wrappers.c (ScrambleDataV27 / SetScramblerV27) take the engine `V27Tx *` as their `void *obj` and reach the transmit
 * scrambler `bundle->sdm` and the rate class `hdx->rate` through two accessors declared in fax_modem_views.h; the 32-bit inline accessors read the raw offsets
 * 0x28 -> 0x1c and 0x24 -> 0xc (asserted below to be exactly these members), on 64-bit hosts fax_v27_tx.c defines faxw_v27_txsdm / faxw_v27_rate over the typed
 * members.
 *
 * The VMI layer (fax_vmi.h) sees the transmitter through the v27tx_* wrappers, which take a FaxVmiV27 (the common modem view FaxVmiM, with the engine handle in
 * `eng`); the initialiser fax_init_vmi_v27tx() is reached through the table fax_init_vmi_data_tx_modem of fax_vmi.c.
 */
#ifndef DSPLIBS_PORT_FAX_V27_OBJ_H
#define DSPLIBS_PORT_FAX_V27_OBJ_H
#include <stddef.h>
#include <stdint.h>
#include "fax_vmi.h"
#include "fax_common.h"
#include "fax_pulse_shaping.h"

/**
 * Configuration block of the transmitter (default V27TX_CFG, 8 words).  s[1] is the baud rate (2400 or 4800; anything else makes the create report an error;
 * the default block carries 9600 and is therefore such a case), w[2] is the timeout (60000 by default, overwritten by V27TX_control), w[3] multiplies the pulse
 * shaper gain, w[5] (low 16 bits) scales the FIFO size (times the frame size), w[6] == 0 selects the long training (hdx.longTrain; V27TX_control copies its w10
 * here), w[7] goes to the pulse shaper configuration (fax_init_vmi_v27tx stores the 32-bit user word there); b[0x10] bit 2 is set by V27TX_control.
 */
typedef union V27Cfg
{
  int w[8];
  short s[16];
  unsigned char b[32];
} V27Cfg;

struct V27Tx;
/** State handler (TxHdx*V27): (engine, scratch / input buffer, output samples, &count) -> number of output samples produced; *count is in / out (16 bit). */
typedef int (*V27TxHdxFn)(struct V27Tx* o, short* in, short* out, unsigned short* n);

/**
 * Rate class (V27TxHdx::rate, V27RxHdx::rate): the index of the per-rate tables.  The create functions map the configured baud rate 2400 -> 0, 4800 -> 1 and
 * any other value -> 1 plus the error flags.
 */
enum V27RateClass
{
  V27RC_2400 = 0, ///< 2400 bit/s
  V27RC_4800 = 1  ///< 4800 bit/s
};

/** Transmit state machine state (V27TxHdx::state); the debug text "V27TX_STATE_<name>" printed by TxNextStateV27 names the state being left. */
enum V27TxState
{
  V27TX_STATE_START = 0,   ///< entry state: moves on immediately
  V27TX_STATE_QUIET = 1,   ///< silence, one frame
  V27TX_STATE_CARR = 2,    ///< carrier: SGD pattern CARR, 10 frames
  V27TX_STATE_NOCARR = 3,  ///< silence, one frame
  V27TX_STATE_ALT = 4,     ///< alternations: SGD pattern ALT, V27TX_ALT_COUNT symbols
  V27TX_STATE_EQCOND = 5,  ///< equaliser conditioning: scrambled ones mapped to the ALT / CARR patterns
  V27TX_STATE_SCR1 = 6,    ///< scrambled ones (SCR1 pattern), 8 symbols
  V27TX_STATE_DATA = 7,    ///< user data from the FIFO, scrambled
  V27TX_STATE_TURNOFF = 8, ///< scrambled ones, one frame
  V27TX_STATE_NOENG = 9,   ///< silence, one frame
  V27TX_STATE_IDLE = 10    ///< carrier off until the FIFO is refilled
};

/**
 * Status code (V27Tx::st.b[0]); it indexes V27TX_MESG, which v27tx_message returns.  The connect codes are named after the rate class that stores them
 * (TxHdxDataV27: 2400 -> CONNECT_2400, 4800 -> CONNECT_4800), but the strings V27TX_MESG[2] / [3] read "4800 bps" / "2400 bps", i.e. the texts are swapped
 * against the codes (slmodemd does not use the texts).
 */
enum V27TxStatus
{
  V27TX_MSG_DATA = 0,         ///< data mode
  V27TX_MSG_PROTOCOL = 1,     ///< training segments
  V27TX_MSG_CONNECT_2400 = 2, ///< first data block at 2400 bit/s
  V27TX_MSG_CONNECT_4800 = 3, ///< first data block at 4800 bit/s
  V27TX_MSG_IDLE = 4,         ///< idle
  V27TX_MSG_ERROR = 5,        ///< internal error (bad rate or state)
  V27TX_MSG_UNDERRUN = 6,     ///< the FIFO ran dry in data mode
  V27TX_MSG_OVERRUN = 7       ///< the input did not fit into the FIFO
};

/** Half-duplex block: the bit queue, the pattern generator and the state machine. */
typedef struct V27TxHdx
{
  FaxFifo* fifo;   ///< bit queue (FIFO_create: size = cfg.w[5] low 16 bits * frame size, fill 0)
  FaxSgd* sgd;     ///< training pattern generator (SGD_create with SGD_CFG, 3 bits per symbol)
  int pend;        ///< nonzero: the input is not queued (control record bit 4) and the under-run check is skipped
  short rate;      ///< rate class V27RateClass (also 1 for any other configured rate, which sets the error flags)
  short longTrain; ///< 1 when cfg.w[6] == 0 (long training), else 0; indexes V27TX_ALT_COUNT / V27TX_EQCOND_COUNT (inferred from those tables)
  V27TxHdxFn fn;   ///< handler of the current state
  short state;     ///< V27TxState (see TxNextStateV27)
  short count;     ///< symbols left in the current state
} V27TxHdx;

/** Modulator bundle: phase-code descriptor, scrambler, encoder and pulse shaper. */
typedef struct V27TxBundle
{
  int w0, w4;    ///< unused
  FaxCirc circ;  ///< encoder output / pulse shaper input: i = q = NULL, buf = phase codes (frame size + 2 entries), wr = rd = 0, lim = frame size + 2
  FaxSdmV27 sdm; ///< transmit scrambler (ScrambleDataV27 / SetScramblerV27)
  FaxSmc smc;    ///< symbol encoder, phase-index output
  FaxPps pps;    ///< pulse shaper
} V27TxBundle;

/** The transmitter instance. */
typedef struct V27Tx
{
  V27Cfg cfg; ///< configuration block
  /**
   * State word.  b[0] is the status code (V27TxStatus); b[1] holds flags (0x01 data state, 0x02 error, 0x08 / 0x10 / 0x40 set at create); bit 0 of b[2] means
   * turned off / idle.  V27TX_modem returns the whole word.
   */
  union
  {
    int w;
    unsigned char b[4];
  } st;
  V27TxHdx* hdx;       ///< half-duplex block
  V27TxBundle* bundle; ///< modulator bundle
} V27Tx;

/** The VMI object view used by the v27tx_* wrappers: the common modem view FaxVmiM (fax_vmi.h); `eng` = engine handle, `u10.cfg` = configuration block. */
typedef FaxVmiM FaxVmiV27;

/** Control record of V27TX_control (only the fields it reads). */
typedef struct FaxV27Ctl
{
  int w0;            ///< unused
  int w4;            ///< copied to cfg.w[2]
  int w8;            ///< multiplied by V27TX_PPS_SCALE[rate] to give the pulse shaper gain
  unsigned char b0c; ///< bit 2 sets cfg.b[0x10] bit 2
  unsigned char b0d; ///< bit 4 sets hdx.pend, bit 1 re-creates the engine
  unsigned char pad0e[2];
  int w10; ///< copied to cfg.w[6] (the long-training selector)
} FaxV27Ctl;

/** Status record filled by V27TX_status (28 bytes; the fields the function does not write are not named). */
typedef struct FaxV27Status
{
  short s0, s2, s4, s6, s8, sa, sc, se, s10, s12; ///< s0 = cfg.s[0], s2 = s10 = baud rate cfg.s[1], the others 0 (se is not written)
  unsigned char b14, b15;                         ///< b14 = 1 | (cfg.b[0x10] & 4); b15 bit 0 is cleared
  int w18;                                        ///< cfg.w[6]
} FaxV27Status;

#if UINTPTR_MAX == 0xffffffffu
_Static_assert(sizeof(V27TxHdx) == 0x18 && offsetof(V27TxHdx, sgd) == 4 && offsetof(V27TxHdx, pend) == 8 && offsetof(V27TxHdx, rate) == 0xc &&
                   offsetof(V27TxHdx, longTrain) == 0xe && offsetof(V27TxHdx, fn) == 0x10 && offsetof(V27TxHdx, state) == 0x14 &&
                   offsetof(V27TxHdx, count) == 0x16,
               "V27TxHdx 32-bit layout");
_Static_assert(sizeof(V27TxBundle) == 0x94 && offsetof(V27TxBundle, circ) == 8 && offsetof(V27TxBundle, circ.buf) == 0x10 &&
                   offsetof(V27TxBundle, circ.wr) == 0x14 && offsetof(V27TxBundle, circ.rd) == 0x16 && offsetof(V27TxBundle, circ.lim) == 0x18 &&
                   offsetof(V27TxBundle, sdm) == 0x1c && offsetof(V27TxBundle, smc) == 0x2c && offsetof(V27TxBundle, pps) == 0x5c,
               "V27TxBundle 32-bit layout");
_Static_assert(sizeof(V27Tx) == 0x2c && offsetof(V27Tx, st) == 0x20 && offsetof(V27Tx, hdx) == 0x24 && offsetof(V27Tx, bundle) == 0x28, "V27Tx 32-bit layout");
_Static_assert(sizeof(FaxVmiV27) == 0x18 && offsetof(FaxVmiV27, u4) == 4 && offsetof(FaxVmiV27, s08) == 8 && offsetof(FaxVmiV27, s0c) == 0xc &&
                   offsetof(FaxVmiV27, u10) == 0x10 && offsetof(FaxVmiV27, eng) == 0x14,
               "FaxVmiV27 32-bit layout");
#endif
_Static_assert(sizeof(FaxV27Ctl) == 0x14 && offsetof(FaxV27Ctl, b0d) == 0xd && offsetof(FaxV27Ctl, w10) == 0x10 && sizeof(FaxV27Status) == 0x1c &&
                   offsetof(FaxV27Status, b14) == 0x14 && offsetof(FaxV27Status, w18) == 0x18,
               "fax V.27 records");
_Static_assert(sizeof(V27Cfg) == 32, "V27Cfg");

/* ---- the engine entry points ---- */
V27Tx* V27TX_create(V27Tx* o, const int* cfg);
void V27TX_delete(V27Tx* o);
int V27TX_modem(V27Tx* o, short* in, short* out, unsigned short* n);
int V27TX_control(V27Tx* o, const FaxV27Ctl* ctl);
int V27TX_status(V27Tx* o, FaxV27Status* st);

int TxHdxStartV27(V27Tx* o, short* in, short* out, unsigned short* n);
int TxHdxQuietV27(V27Tx* o, short* in, short* out, unsigned short* n);
int TxHdxEQCondV27(V27Tx* o, short* in, short* out, unsigned short* n);
int TxHdxSCR1V27(V27Tx* o, short* in, short* out, unsigned short* n);
int TxHdxAltV27(V27Tx* o, short* in, short* out, unsigned short* n);
int TxHdxDataV27(V27Tx* o, short* in, short* out, unsigned short* n);
int TxHdxIdleV27(V27Tx* o, short* in, short* out, unsigned short* n);
void TxNextStateV27(V27Tx* o);
int ModDataV27(V27Tx* o, short* in, short* out, unsigned short n);
int TxNoCarrierV27(V27Tx* o, short* in, short* out, unsigned short n);

/* the VMI wrappers (v27tx_*) and the VMI initialiser */
int fax_init_vmi_v27tx(FaxVmiV27* m, unsigned short rate, int unused, void* user);
int v27tx_create(FaxVmiV27* m, const int* cfg);
void v27tx_delete(FaxVmiV27* m);
int v27tx_process(FaxVmiV27* m, short* out, unsigned short* n, unsigned short* outn);
int v27tx_status(FaxVmiV27* m, FaxV27Status* st);
int v27tx_control(FaxVmiV27* m, const FaxV27Ctl* ctl);
void v27tx_message(FaxVmiV27* m, int idx, const char** out);
#endif
