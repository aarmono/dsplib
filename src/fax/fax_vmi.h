/**
 * @file fax_vmi.h
 * @brief FAXVMI, the framing layer between the T.30 byte / frame buffers and the bit buffers of the fax modems; the VMI object, its framing state and the
 *        ops tables that dispatch to the modems.
 *
 * FAXVMI_create() / FAXVMI_process() / FAXVMI_control() / FAXVMI_status() / FAXVMI_message() / FAXVMI_delete() operate on one VMI object per modem
 * channel.  The object packs the bytes or frames of the T.30 layer into bits for a transmit modem (simple, asynchronous or HDLC framing), unpacks the bits
 * of a receive modem into bytes or frames, and calls the modem engine through the ops tables vxx_*.
 *
 * Objects:
 * - FaxVmiM: the modem sub-object (the view the v21rx_* ... v17rx_* wrappers take; FaxVmiV21 / V27 / V29 / V17 are typedefs of it).  `buf0` is a 200-short
 *   scratch buffer (null_process() copies into it), `u4.bits` a 50-short bit buffer that the modems read (transmit) or write (receive), `s0c` the bit
 *   words per frame, `s0e` the bits per word, `u10` the modem's config block pointer (a short after create), `eng` the engine handle (V21Rx, V21Tx,
 *   V27Tx, V29Rx ...).
 * - FaxVmiSt: the framing state: a ring of 16-bit items (queued bytes, or length-prefixed frames in HDLC mode), the transmit and receive bit state, the
 *   HDLC receive frame buffer and counters.
 * - FaxVmi: the VMI object: a copy of the config (FaxVmiCfg), flags, `st` and `m`.
 * - FaxVmiCfg / FaxVmiCtl / FaxVmiSts: config, control and status records.  The last member of the control / status records points to the modem's own
 *   control / status record.
 *
 * Ops tables: vxx_create / _delete / _process / _status / _control / _message, 13 entries each, indexed by the modem id (0 .. 4 null modem, 5 V.21 tx,
 * 6 V.21 rx, 7 V.27ter tx, 8 rx, 9 V.29 tx, 10 rx, 11 V.17 tx, 12 rx).  The slots are typed function pointers; wrappers whose prototypes differ are
 * reached through small adapters in fax_vmi.c.  fax_init_vmi_data_tx_modem[] / fax_init_vmi_data_rx_modem[] list the initialisers of the three data
 * modems {V.27ter, V.29, V.17}.
 */
#ifndef DSPLIBS_PORT_FAX_VMI_H
#define DSPLIBS_PORT_FAX_VMI_H
#include <stddef.h>
#include <stdint.h>

/* ---------------------------------------------------------------- the modem sub-object (the VMI view of every modem) */
/** The modem sub-object of a VMI object, and the view of it the modem wrappers use. */
typedef struct FaxVmiM
{
  union
  {
    int w0;
    short* buf0; ///< the 200-short scratch buffer (allocated by FAXVMI_create)
  };
  union
  {
    short* bits;     ///< the bit buffer (allocated by FAXVMI_create)
    uintptr_t init1; ///< the init_vmi_* functions store the integer 1 here (data-modem init blocks)
  } u4;
  short s08, s0a; ///< per-modem values set by the init functions; as a FaxVmiCfg they are the ring minimum and the maximum items per call
  short s0c;      ///< bit words per frame
  short s0e;      ///< bits per word (the data-modem init blocks keep the modem id here instead)
  union
  {
    void* cfg; ///< the modem's config block (allocated); the create functions then store a short over its first two bytes
    short s10; ///< the V.21 engines keep their direction / state short here
  } u10;
  void* eng; ///< the engine handle (an opaque pointer: V21Rx / V21Tx / V27Tx / V27Rx / V29Tx / V29Rx / V17Tx / V17Rx)
} FaxVmiM;

/* ---------------------------------------------------------------- config / control / status records */
/** Config of a VMI object (FAXVMI_CFG holds the defaults). */
typedef struct FaxVmiCfg
{
  int mode;                ///< framing mode FaxVmiMode (the low 16 bits are used): 0 simple, 1 asynchronous, 2 HDLC
  int flag;                ///< nonzero: the bytes of the T.30 buffer are bit-reversed before and after the framing
  unsigned short ringMin;  ///< ring size lower bound
  unsigned short maxItems; ///< maximum items per call / ring headroom
  unsigned short frmWords; ///< HDLC receive frame buffer words
  unsigned short id;       ///< modem id 0 .. 12 (FAXVMI_ID_*)
  const void* mcfg;        ///< the modem's config (the modem's create gets this pointer)
  int w14;                 ///< unused
} FaxVmiCfg;

/** Control record of FAXVMI_control(). */
typedef struct FaxVmiCtl
{
  int flushRing;             ///< nonzero: flush the ring
  int breakReq;              ///< stored as st->breakReq (asynchronous transmit: break request)
  unsigned short breakCount; ///< stored as st->breakLeft (idle / break characters to send)
  unsigned short pad0a;
  int rxReset;            ///< nonzero (and newMode <= 2): reset the framing state and take the mode newMode
  unsigned short newMode; ///< new framing mode
  unsigned short pad12;
  const void* mctl; ///< the modem's control record (passed to the modem's control)
} FaxVmiCtl;

/** Status record of FAXVMI_status(). */
typedef struct FaxVmiSts
{
  short ringFree; ///< free room in the ring
  short pad2;
  int ovf;        ///< st->ovf: items the last call could not queue
  int txEmpty;    ///< FaxVmi.txEmpty
  int rxOverflow; ///< FaxVmi.rxOverflow
  int rxBreak;    ///< st->rxBreak
  int hskipDone;  ///< st->hskip <= 1 (the HDLC opening flags have been seen)
  void* msts;     ///< the modem's status record (passed to the modem's status)
} FaxVmiSts;

/* ---------------------------------------------------------------- the framing state (FaxVmi->st) */
/** The framing state of a VMI object. */
typedef struct FaxVmiSt
{
  unsigned short* ring;    ///< allocated ring of `size` items: queued bytes (asynchronous / simple) or length-prefixed frames (HDLC)
  unsigned short size;     ///< ring size in items
  unsigned short rd;       ///< ring read position
  unsigned short wr;       ///< ring write position
  unsigned short cnt;      ///< items stored
  int ovf;                 ///< items the last pack call could not queue
  int tx_mask;             ///< transmit bit mask (0: fetch the next item)
  int tx_cur;              ///< transmit item being shifted out
  int tx_acc;              ///< transmit bit accumulator
  unsigned short tx_nbits; ///< bits in the current output word
  unsigned short pad1e;
  int rx_mask;             ///< receive bit mask (low 16 bits used at load)
  int rx_cur;              ///< receive word being examined
  int rx_acc;              ///< receive bit accumulator
  unsigned short rx_nbits; ///< bits collected
  unsigned short pad2e;
  int rx_state;             ///< asynchronous receive: 0 collecting data bits, else hunting for the start bit (initially 1)
  unsigned short breakLeft; ///< asynchronous transmit: idle (break) characters still to send
  unsigned short pad36;
  int breakReq;            ///< asynchronous transmit: break request
  int rxBreak;             ///< receive: all-zero (break) pattern seen
  unsigned short* frm;     ///< HDLC receive frame buffer (`frm_size` items)
  unsigned short frm_size; ///< frame buffer size in items
  short tx_left;           ///< HDLC transmit: bytes of the current frame still to send
  short flen;              ///< HDLC receive: bytes collected
  unsigned short hskip;    ///< HDLC receive: opening flags still to skip (initially 2)
  int tx_flag;             ///< HDLC transmit: in flag mode (no bit stuffing; initially 1)
  short ones;              ///< HDLC receive: run of one bits
  short pad52;
  int in_frame; ///< HDLC receive: collecting a frame
} FaxVmiSt;

/** The VMI object. */
typedef struct FaxVmi
{
  FaxVmiCfg cfg;          ///< a copy of the config
  int txEmpty;            ///< transmit queue empty flag
  int rxOverflow;         ///< receive overflow flag
  unsigned int stateWord; ///< the state word returned by the last FAXVMI_process() (flag bits, see FaxVmiState)
  FaxVmiSt* st;           ///< framing state
  FaxVmiM* m;             ///< modem sub-object
} FaxVmi;

#if UINTPTR_MAX == 0xffffffffu
_Static_assert(sizeof(FaxVmiM) == 0x18 && offsetof(FaxVmiM, u4) == 4 && offsetof(FaxVmiM, s0c) == 0xc && offsetof(FaxVmiM, s0e) == 0xe &&
                   offsetof(FaxVmiM, u10) == 0x10 && offsetof(FaxVmiM, eng) == 0x14,
               "FaxVmiM 32-bit layout");
_Static_assert(sizeof(FaxVmiCfg) == 0x18 && offsetof(FaxVmiCfg, ringMin) == 8 && offsetof(FaxVmiCfg, id) == 0xe && offsetof(FaxVmiCfg, mcfg) == 0x10 &&
                   offsetof(FaxVmiCfg, w14) == 0x14,
               "FaxVmiCfg 32-bit layout");
_Static_assert(sizeof(FaxVmiCtl) == 0x18 && offsetof(FaxVmiCtl, breakCount) == 8 && offsetof(FaxVmiCtl, rxReset) == 0xc &&
                   offsetof(FaxVmiCtl, newMode) == 0x10 && offsetof(FaxVmiCtl, mctl) == 0x14,
               "FaxVmiCtl 32-bit layout");
_Static_assert(sizeof(FaxVmiSts) == 0x1c && offsetof(FaxVmiSts, ovf) == 4 && offsetof(FaxVmiSts, hskipDone) == 0x14 && offsetof(FaxVmiSts, msts) == 0x18,
               "FaxVmiSts 32-bit layout");
_Static_assert(sizeof(FaxVmiSt) == 0x58 && offsetof(FaxVmiSt, size) == 4 && offsetof(FaxVmiSt, cnt) == 0xa && offsetof(FaxVmiSt, ovf) == 0xc &&
                   offsetof(FaxVmiSt, tx_mask) == 0x10 && offsetof(FaxVmiSt, tx_nbits) == 0x1c && offsetof(FaxVmiSt, rx_mask) == 0x20 &&
                   offsetof(FaxVmiSt, rx_nbits) == 0x2c && offsetof(FaxVmiSt, rx_state) == 0x30 && offsetof(FaxVmiSt, breakLeft) == 0x34 &&
                   offsetof(FaxVmiSt, rxBreak) == 0x3c && offsetof(FaxVmiSt, frm) == 0x40 && offsetof(FaxVmiSt, frm_size) == 0x44 &&
                   offsetof(FaxVmiSt, tx_left) == 0x46 && offsetof(FaxVmiSt, flen) == 0x48 && offsetof(FaxVmiSt, hskip) == 0x4a &&
                   offsetof(FaxVmiSt, tx_flag) == 0x4c && offsetof(FaxVmiSt, ones) == 0x50 && offsetof(FaxVmiSt, in_frame) == 0x54,
               "FaxVmiSt 32-bit layout");
_Static_assert(sizeof(FaxVmi) == 0x2c && offsetof(FaxVmi, txEmpty) == 0x18 && offsetof(FaxVmi, stateWord) == 0x20 && offsetof(FaxVmi, st) == 0x24 &&
                   offsetof(FaxVmi, m) == 0x28,
               "FaxVmi 32-bit layout");
#endif

/** Framing modes (FaxVmiCfg.mode, FaxVmiCtl.newMode). */
enum FaxVmiMode
{
  FAXVMI_MODE_SIMPLE = 0,
  FAXVMI_MODE_ASYNC = 1,
  FAXVMI_MODE_HDLC = 2
};

/**
 * The flag bits of the state word FAXVMI_process() returns.  The low 24 bits and bits 29 .. 31 are the state dword of the modem engine (byte 0 the status
 * code, byte 1 the flags: 0x01 data phase, 0x02 error, 0x20 carrier; see the engines' `st` members); bits 24 .. 27 are added by FAXVMI_process()
 * (bit 28 is cleared when a break pattern was received).
 */
enum FaxVmiState
{
  FAXVMI_ST_DATA = 0x100,
  FAXVMI_ST_ERROR = 0x200,
  FAXVMI_ST_CARRIER = 0x2000,
  FAXVMI_ST_TX_EMPTY = 0x1000000,
  FAXVMI_ST_QUEUE_FULL = 0x2000000,
  FAXVMI_ST_QUEUE_OVF = 0x4000000,
  FAXVMI_ST_RX_OVERFLOW = 0x8000000
};

/* modem ids (FaxVmiCfg.id) */
enum
{
  FAXVMI_ID_NULL = 0,
  FAXVMI_ID_V21TX = 5,
  FAXVMI_ID_V21RX = 6,
  FAXVMI_ID_V27TX = 7,
  FAXVMI_ID_V27RX = 8,
  FAXVMI_ID_V29TX = 9,
  FAXVMI_ID_V29RX = 10,
  FAXVMI_ID_V17TX = 11,
  FAXVMI_ID_V17RX = 12,
  FAXVMI_NIDS = 13
};

/* ---------------------------------------------------------------- the entry points (documented at their definitions in fax_vmi.c) */
FaxVmi* FAXVMI_create(FaxVmi* o, const FaxVmiCfg* cfg);
void FAXVMI_delete(FaxVmi* o);
unsigned int FAXVMI_process(FaxVmi* o, unsigned short* buf, short* samples, unsigned short* n, unsigned short* cnt2);
int FAXVMI_status(FaxVmi* o, FaxVmiSts* st);
int FAXVMI_control(FaxVmi* o, const FaxVmiCtl* ctl);
const char* FAXVMI_message(FaxVmi* o, int idx);
extern const FaxVmiCfg FAXVMI_CFG;
extern const FaxVmiCtl FAXVMI_CTL;
extern const FaxVmiSts FAXVMI_STS;

/* framing functions: (o, buffer pointer by reference, count) for the ring writers, (o, buffer, count) for the pack and unpack functions */
int faxvmi_write_fifo(FaxVmi* o, unsigned short** pp, short n); /* queue n bytes from *pp into the ring (advances *pp); returns the number queued */
int faxvmi_write_frame(FaxVmi* o, unsigned short** pp,
                       short n);                               /* queue n length-prefixed frames ([len][len bytes]) with their CRC; returns the number queued */
int faxvmi_asyc_pack(FaxVmi* o, unsigned short* buf, short n); /* queue n bytes, then fill m->bits with s0c asynchronous characters; returns s0c */
int faxvmi_simp_pack(FaxVmi* o, unsigned short* buf, short n); /* queue n bytes, then fill m->bits with plain bits; returns the number of words produced */
int faxvmi_hdlc_frame(FaxVmi* o, unsigned short* buf, short n); /* queue n frames, then fill m->bits with HDLC bits; returns s0c */
int faxvmi_asyc_unpack(FaxVmi* o, unsigned short* out, short n);
int faxvmi_simp_unpack(FaxVmi* o, unsigned short* out, short n);
int faxvmi_hdlc_unframe(FaxVmi* o, unsigned short* out, short n);
unsigned short faxvmi_gen_fcs16(const unsigned short* p, short n);
void faxvmi_byte_reverse(unsigned short* p, short n);
void faxvmi_frame_reverse(unsigned short* p, short n);

/* null modem (ids 0 .. 4) */
void null_create(FaxVmiM* m, const void* cfg);
void null_delete(FaxVmiM* m);
int null_process(FaxVmiM* m, short* a, unsigned short* b, unsigned short* cnt);
int null_status(FaxVmiM* m, void* st);
int null_control(FaxVmiM* m, const void* ctl);
void null_message(FaxVmiM* m, int idx, const char** out);

/* the ops tables */
typedef void (*FaxVmiCreateFn)(FaxVmiM* m, const void* cfg);
typedef void (*FaxVmiDeleteFn)(FaxVmiM* m);
typedef int (*FaxVmiProcessFn)(FaxVmiM* m, short* samples, unsigned short* a, unsigned short* b);
typedef int (*FaxVmiStatusFn)(FaxVmiM* m, void* st);
typedef int (*FaxVmiControlFn)(FaxVmiM* m, const void* ctl);
typedef void (*FaxVmiMessageFn)(FaxVmiM* m, int idx, const char** out);
extern const FaxVmiCreateFn vxx_create[FAXVMI_NIDS];
extern const FaxVmiDeleteFn vxx_delete[FAXVMI_NIDS];
extern const FaxVmiProcessFn vxx_process[FAXVMI_NIDS];
extern const FaxVmiStatusFn vxx_status[FAXVMI_NIDS];
extern const FaxVmiControlFn vxx_control[FAXVMI_NIDS];
extern const FaxVmiMessageFn vxx_message[FAXVMI_NIDS];

/* the modem initialisers: (m, rate, unused, user) -> a config word the callers ignore */
typedef int (*FaxVmiInitFn)(FaxVmiM* m, unsigned short rate, int unused, void* user);
extern FaxVmiInitFn fax_init_vmi_data_tx_modem[3]; /* {V.27ter, V.29, V.17} */
extern FaxVmiInitFn fax_init_vmi_data_rx_modem[3];
#endif
