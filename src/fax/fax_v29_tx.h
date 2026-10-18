/**
 * @file fax_v29_tx.h
 * @brief Typed objects of the fax V.29 transmit half (fax_v29_tx.c): engine instance, half-duplex block, bundle, state and status enums, control / status
 *     records.
 *
 * Same design as fax_v27_tx.h.  The transmitter is three heap objects: the instance V29Tx (configuration block, state word, pointers to the other two), the
 * half-duplex block V29TxHdx (bit FIFO, pattern generator, state machine: handler pointer and counters) and the bundle V29TxBundle (SMC output descriptor,
 * scrambler, symbol encoder and pulse shaper).  They are ordinary structs with real pointer members: in a 32-bit build the member offsets are pinned by the
 * _Static_asserts below (the raw-offset accessors of fax_modem_views.h and the 32-bit layout tests rely on them), on 64-bit hosts the members after a pointer
 * move, so NOTHING may address these objects by a raw byte offset.  The handler slot is a typed function pointer.
 *
 * Differences from the V.27ter transmitter: the symbol encoder produces I / Q SAMPLES (encoder configuration phaseOut == 0: circ.i / circ.q, 50 shorts each,
 * allocated by the create; circ.buf is a third 50-short block that nothing reads), the pulse shaper runs in mode 0 (I / Q taken from circ.i / circ.q), the
 * frame is always 48 symbols, there is ONE filter pair (120 taps, interpolate by 10, decimate by 3) and the rate class (0 = 7200, 1 = 9600) only selects the
 * pulse shaper gain (V29TX_PPS_SCALE), the scrambler bit count (3 / 4), the encoder's second mask and the SCR1 training pattern. V29TxHdx::reg is the 7-bit
 * LFSR of the equaliser-conditioning sequence.
 *
 * The scrambler wrappers of fax_modem_wrappers.c (ScrambleDataV29 / SeedScramblerV29 / SetEncoderV29) take the engine `V29Tx *` as their `void *obj` and reach
 * the transmit scrambler `bundle->sdm` and the encoder select (the encoder configuration word `flag`:
 * 1 = absolute mapping through imap / qmap, 0 = differential table) through two accessors declared in fax_modem_views.h:
 * the 32-bit inline accessors read the raw offsets 0x24 -> 0x1c and 0x24 -> 0x38 (asserted below to be exactly these members), on 64-bit hosts fax_v29_tx.c
 * defines faxw_v29_txsdm / faxw_v29_encsel over the typed members.
 *
 * The VMI layer (fax_vmi.h) sees the transmitter through the v29tx_* wrappers, which take a FaxVmiV29 (the common modem view FaxVmiM, with the engine handle in
 * `eng`); the initialiser fax_init_vmi_v29tx() is reached through the table fax_init_vmi_data_tx_modem of fax_vmi.c.
 */
#ifndef DSPLIBS_PORT_FAX_V29_OBJ_H
#define DSPLIBS_PORT_FAX_V29_OBJ_H
#include <stddef.h>
#include <stdint.h>
#include "fax_vmi.h"
#include "fax_common.h"
#include "fax_pulse_shaping.h"

/**
 * Configuration block of the transmitter (default V29TX_CFG, 7 words).  s[1] is the baud rate (7200 or 9600; anything else makes the create report an error),
 * w[2] is the timeout (60000 by default, overwritten by V29TX_control), w[5] (low 16 bits) scales the FIFO size (times 48 symbols), w[6] is passed to the pulse
 * shaper configuration (fax_init_vmi_v29tx stores the 32-bit user word there); b[0x10] bit 2 is set by V29TX_control.
 */
typedef union V29Cfg
{
  int w[7];
  short s[14];
  unsigned char b[28];
} V29Cfg;

struct V29Tx;
/** State handler (TxHdx*V29): (engine, scratch / input buffer, output samples, &count) -> number of output samples produced; *count is in / out (16 bit). */
typedef int (*V29TxHdxFn)(struct V29Tx* o, short* in, short* out, unsigned short* n);

/**
 * Rate class (V29TxHdx::rate, V29RxHdx::rate): the index of the per-rate tables.  The create functions map the configured baud rate 7200 -> 0, 9600 -> 1 and
 * any other value -> 1 plus the error flags.
 */
enum V29RateClass
{
  V29RC_7200 = 0, ///< 7200 bit/s
  V29RC_9600 = 1  ///< 9600 bit/s
};

/** Transmit state machine state (V29TxHdx::state); the debug text "V29TX_STATE_<name>" printed by TxNextStateV29 names the state being left. */
enum V29TxState
{
  V29TX_STATE_START = 0,  ///< entry state: moves on immediately
  V29TX_STATE_QUIET = 1,  ///< silence, one frame
  V29TX_STATE_ALT = 2,    ///< segment AB: alternating pattern 0x4f, absolute encoding
  V29TX_STATE_EQCOND = 3, ///< equaliser conditioning: 7-bit LFSR sequence
  V29TX_STATE_SCR1 = 4,   ///< scrambled ones (training pattern per rate), differential encoding
  V29TX_STATE_DATA = 5,   ///< user data from the FIFO, scrambled
  V29TX_STATE_IDLE = 6    ///< carrier off until the FIFO is refilled
};

/**
 * Status code (V29Tx::st.b[0]); it indexes V29TX_MESG, which v29tx_message returns.  The text of CONNECT_7200 reads "7600 bps" (a typo in the string; the code
 * is stored for the 7200 rate class).
 */
enum V29TxStatus
{
  V29TX_MSG_DATA = 0,         ///< data mode
  V29TX_MSG_PROTOCOL = 1,     ///< training segments
  V29TX_MSG_CONNECT_9600 = 2, ///< first data block at 9600 bit/s
  V29TX_MSG_CONNECT_7200 = 3, ///< first data block at 7200 bit/s
  V29TX_MSG_IDLE = 4,         ///< idle
  V29TX_MSG_ERROR = 5,        ///< internal error (bad rate or state)
  V29TX_MSG_UNDERRUN = 6,     ///< the FIFO ran dry in data mode
  V29TX_MSG_OVERRUN = 7       ///< the input did not fit into the FIFO
};

/** Half-duplex block: the bit queue, the pattern generator and the state machine. */
typedef struct V29TxHdx
{
  FaxFifo* fifo; ///< bit queue (FIFO_create: size = cfg.w[5] low 16 bits * 48, fill 0)
  FaxSgd* sgd;   ///< training pattern generator (SGD_create with SGD_CFG, 4 bits per symbol)
  int pend;      ///< nonzero: the input is not queued (control record bit 4) and the under-run check is skipped
  short rate;    ///< rate class V29RateClass (also 1 for any other configured rate, which sets the error flags)
  short s0e;     ///< unused
  V29TxHdxFn fn; ///< handler of the current state
  short state;   ///< V29TxState (see TxNextStateV29)
  short count;   ///< symbols / bits left in the current state
  short reg;     ///< LFSR of the equaliser-conditioning sequence (0x2a at create and at the entry of the state)
} V29TxHdx;

/** Modulator bundle: symbol-sample descriptor, scrambler, encoder and pulse shaper. */
typedef struct V29TxBundle
{
  int w0, w4;   ///< unused
  FaxCirc circ; ///< encoder output / pulse shaper input: i, q = 50-short buffers, buf = unused third block (allocated on every create), wr = rd = 0, lim = 50
  FaxSdm sdm;   ///< transmit scrambler (ScrambleDataV29 / SeedScramblerV29)
  FaxSmc smc;   ///< symbol encoder, I / Q output; smc.cfg.flag is the encoder select written by SetEncoderV29
  FaxPps pps;   ///< pulse shaper
} V29TxBundle;

/** The transmitter instance. */
typedef struct V29Tx
{
  V29Cfg cfg; ///< configuration block
  /**
   * State word.  b[0] is the status code (V29TxStatus); b[1] holds flags (0x01 data state, 0x02 error, 0x08 / 0x10 / 0x40 set at create); bit 0 of b[2] means
   * idle.  V29TX_modem returns the whole word.
   */
  union
  {
    int w;
    unsigned char b[4];
  } st;
  V29TxHdx* hdx;       ///< half-duplex block
  V29TxBundle* bundle; ///< modulator bundle
} V29Tx;

/** The VMI object view used by the v29tx_* wrappers: the common modem view FaxVmiM (fax_vmi.h); `eng` = engine handle, `u10.cfg` = configuration block. */
typedef FaxVmiM FaxVmiV29;

/** Control record of V29TX_control (only the fields it reads). */
typedef struct FaxV29Ctl
{
  int w0;            ///< unused
  int w4;            ///< copied to cfg.w[2]
  int w8;            ///< multiplied by V29TX_PPS_SCALE[rate] to give the pulse shaper gain
  unsigned char b0c; ///< bit 2 sets cfg.b[0x10] bit 2
  unsigned char b0d; ///< bit 4 sets hdx.pend, bit 1 re-creates the engine
  unsigned char pad0e[2];
} FaxV29Ctl;

/** Status record filled by V29TX_status (22 bytes are written). */
typedef struct FaxV29Status
{
  short s0, s2, s4, s6, s8, sa, sc, se, s10, s12; ///< s0 = cfg.s[0], s2 = s10 = baud rate cfg.s[1], the others 0 (se is not written)
  unsigned char b14, b15;                         ///< b14 = cfg.b[0x10] & 4; b15 bit 0 is cleared
} FaxV29Status;

#if UINTPTR_MAX == 0xffffffffu
_Static_assert(sizeof(V29TxHdx) == 0x1c && offsetof(V29TxHdx, sgd) == 4 && offsetof(V29TxHdx, pend) == 8 && offsetof(V29TxHdx, rate) == 0xc &&
                   offsetof(V29TxHdx, s0e) == 0xe && offsetof(V29TxHdx, fn) == 0x10 && offsetof(V29TxHdx, state) == 0x14 && offsetof(V29TxHdx, count) == 0x16 &&
                   offsetof(V29TxHdx, reg) == 0x18,
               "V29TxHdx 32-bit layout");
_Static_assert(sizeof(V29TxBundle) == 0x9c && offsetof(V29TxBundle, circ) == 8 && offsetof(V29TxBundle, circ.i) == 8 && offsetof(V29TxBundle, circ.q) == 0xc &&
                   offsetof(V29TxBundle, circ.buf) == 0x10 && offsetof(V29TxBundle, circ.wr) == 0x14 && offsetof(V29TxBundle, circ.rd) == 0x16 &&
                   offsetof(V29TxBundle, circ.lim) == 0x18 && offsetof(V29TxBundle, sdm) == 0x1c && offsetof(V29TxBundle, smc) == 0x34 &&
                   offsetof(V29TxBundle, smc.cfg.flag) == 0x38 && offsetof(V29TxBundle, pps) == 0x64 && offsetof(V29TxBundle, pps.cfg.scale) == 0x6c,
               "V29TxBundle 32-bit layout");
_Static_assert(sizeof(V29Tx) == 0x28 && offsetof(V29Tx, st) == 0x1c && offsetof(V29Tx, hdx) == 0x20 && offsetof(V29Tx, bundle) == 0x24, "V29Tx 32-bit layout");
_Static_assert(sizeof(FaxVmiV29) == 0x18 && offsetof(FaxVmiV29, u4) == 4 && offsetof(FaxVmiV29, s08) == 8 && offsetof(FaxVmiV29, s0c) == 0xc &&
                   offsetof(FaxVmiV29, u10) == 0x10 && offsetof(FaxVmiV29, eng) == 0x14,
               "FaxVmiV29 32-bit layout");
#endif
_Static_assert(sizeof(FaxV29Ctl) == 0x10 && offsetof(FaxV29Ctl, b0d) == 0xd && sizeof(FaxV29Status) == 0x16 && offsetof(FaxV29Status, b14) == 0x14,
               "fax V.29 records");
_Static_assert(sizeof(V29Cfg) == 28, "V29Cfg");

/* ---- the engine entry points ---- */
V29Tx* V29TX_create(V29Tx* o, const int* cfg);
void V29TX_delete(V29Tx* o);
int V29TX_modem(V29Tx* o, short* in, short* out, unsigned short* n);
int V29TX_control(V29Tx* o, const FaxV29Ctl* ctl);
int V29TX_status(V29Tx* o, FaxV29Status* st);

int TxHdxStartV29(V29Tx* o, short* in, short* out, unsigned short* n);
int TxHdxQuietV29(V29Tx* o, short* in, short* out, unsigned short* n);
int TxHdxEQCondV29(V29Tx* o, short* in, short* out, unsigned short* n);
int TxHdxSCR1V29(V29Tx* o, short* in, short* out, unsigned short* n);
int TxHdxABV29(V29Tx* o, short* in, short* out, unsigned short* n);
int TxHdxDataV29(V29Tx* o, short* in, short* out, unsigned short* n);
int TxHdxIdleV29(V29Tx* o, short* in, short* out, unsigned short* n);
void TxNextStateV29(V29Tx* o);
int ModDataV29(V29Tx* o, short* in, short* out, unsigned short n);
int TxNoCarrierV29(V29Tx* o, short* in, short* out, unsigned short n);

/* the VMI wrappers (v29tx_*) and the VMI initialiser */
int fax_init_vmi_v29tx(FaxVmiV29* m, unsigned short rate, int unused, void* user);
V29Tx* v29tx_create(FaxVmiV29* m, const int* cfg); /* returns the engine handle */
void v29tx_delete(FaxVmiV29* m);
int v29tx_process(FaxVmiV29* m, short* out, unsigned short* n, unsigned short* outn);
int v29tx_status(FaxVmiV29* m, FaxV29Status* st);
int v29tx_control(FaxVmiV29* m, const FaxV29Ctl* ctl);
void v29tx_message(FaxVmiV29* m, int idx, const char** out);
#endif
