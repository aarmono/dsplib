/**
 * @file v23.h
 * @brief Object types of the V.23 modem engine (v23.c).
 *
 * V.23 sends 1200 bit/s FSK on the forward channel (mark 1300 Hz, space 2100 Hz) and 75 bit/s on the backward channel (mark 390 Hz, space 450 Hz).  One
 * engine instance transmits on one channel and receives on the other:
 *   - V23Modem   the top-level instance: which receiver backend is used, the answer-tone sequencer, and pointers to the transmitter and the receiver;
 *   - V23Rx      receiver of the 1200 bit/s forward channel (tone detector, two AGCs, IIR filter, matched filter, FSK demodulator);
 *   - V23BwCh    receiver of the 75 bit/s backward channel (tone detector, AGC, IIR filter, mark / space energy detectors);
 *   - V23Tx      transmitter: a table-driven FSK generator that works for either channel (the samples-per-bit table and the two frequencies select it).
 * The blocks hold real pointers, so their layout differs between 32- and 64-bit builds; on a 32-bit build the `_Static_assert`s pin the byte offsets the
 * 32-bit test oracles rely on.  Code addresses the members by name.
 */
#ifndef DSPLIBS_PORT_V23_OBJ_H
#define DSPLIBS_PORT_V23_OBJ_H
#include "fpm/fpm.h"

/** @brief Engine configuration block that CreateV23Modem() takes (the data pump glue fills it). */
typedef struct V23Cfg
{
  unsigned char tone; ///< non-zero: the engine sends the 2100 Hz answer tone first (the answering side)
  unsigned char pad[3];
  int rate; ///< sample rate in Hz
  int thr;  ///< signal-absence threshold of the receiver (700 from the glue)
} V23Cfg;

/** Receiver of the 1200 bit/s forward channel. */
typedef struct V23Rx
{
  short state; ///< 0 / 5: carrier tone not yet confirmed (+5 per block with the tone); 10: confirmed (the AGC is frozen); 11: running
  short pad2;
  FpmTone* tone;        ///< carrier tone detector
  FpmAgc agc0;          ///< AGC of the demodulated signal; its detect flag is the carrier indication
  FpmAgc agc1;          ///< AGC in front of the tone detector (a.s[0] = 0x4000, a.s[5] = 0x28 after init)
  FpmMrf mrf;           ///< matched filter / sample-rate converter (3 branches, step 4)
  FpmFsd fsd;           ///< FSK demodulator
  const short* iirCoef; ///< IIR cascade coefficients (V23_IIR_FILT)
  short* iirState;      ///< IIR cascade state (16 shorts)
  short iirCount;       ///< number of IIR sections (4)
  short padBE;
  short scratch[0x80 + 0x20]; ///< copy of the received block for the tone detector (160 samples)
  short bits[26];             ///< demodulated bits of the last block
  short nBits;                ///< number of bits in `bits`
  short energyAcc;            ///< signal-absence accumulator: +20 per block without signal, cleared by a block with signal
  short energyThr;            ///< `energyAcc` limit that ends the connection (cfg word 2: 700)
  short carrierAcc;           ///< carrier-search accumulator: +20 per block until the tone is confirmed
  short carrierThr;           ///< `carrierAcc` limit (60000, compared unsigned): the carrier search times out
  short noCarrier;            ///< 1 while the carrier is not yet established; read by V23ModemMain() to force marks on the transmitter
} V23Rx;

/** Transmitter: FSK generator driven by a table of samples per bit. */
typedef struct V23Tx
{
  short idx;          ///< index into `table` (rotates modulo `len`)
  short len;          ///< number of table entries (3)
  const short* table; ///< samples-per-bit table (a real pointer here, not a CFGP handle)
  short countdown;    ///< samples left in the current bit
  short pad0A;
  int carry;     ///< bit value carried over to the next call while `cont` is set
  int cont;      ///< non-zero: the previous call ended inside a bit whose value is `carry`
  int silence;   ///< one-shot: the next call writes one block of silence and clears this flag (set from the answer-tone flag at creation)
  short freq0;   ///< tone frequency for a 0 bit (space)
  short freq1;   ///< tone frequency for a 1 bit (mark)
  FpmTone* tone; ///< tone generator
} V23Tx;

/** Receiver of the 75 bit/s backward channel. */
typedef struct V23BwCh
{
  short st[4];    ///< filter state: mark filter in st[0..1], space filter in st[2..3]
  short mark;     ///< mark-filter energy accumulated over the current block
  short space;    ///< space-filter energy accumulated over the current block
  short sC;       ///< cleared at creation; not used otherwise
  short timeout;  ///< blocks spent waiting for the tone (the wait is abandoned after more than 1000)
  int countdown;  ///< samples left in the current block of `block_size_table`
  int rot;        ///< index into `block_size_table` (modulo 12)
  int debounce;   ///< blocks the bit decision must stay unchanged before a bit is emitted
  int last;       ///< the last block's bit decision
  short noCarAcc; ///< signal-absence accumulator: +20 per call without AGC detect, cleared by a call with detect
  short pad22;
  int noCarThr;  ///< `noCarAcc` limit that ends the connection (cfg word 2: 700)
  short toneCnt; ///< consecutive blocks with the 390 Hz tone; the demodulator starts at 10
  short pad2A;
  FpmTone* tone;        ///< tone detector
  FpmAgc agc;           ///< AGC
  const short* iirCoef; ///< IIR cascade coefficients (BwCh_IIR_FILT)
  short* iirState;      ///< IIR cascade state (12 shorts)
  short iirCount;       ///< number of IIR sections (3)
  short status;         ///< 0 = demodulating, non-zero = still acquiring; read by V23ModemMain() to force marks on the transmitter
} V23BwCh;

/** Top-level instance. */
typedef struct V23Modem
{
  short backend; ///< receiver backend: 0 = forward channel (V23Rx), else backward channel (V23BwCh)
  short pad2;
  union
  {
    unsigned int w;
    short s[2];
  } state;             ///< answer-tone sequencer: 0 = tone, 1 = silence, 2 = steady (receiver and transmitter run)
  unsigned int shadow; ///< copy of `state` for change logging
  union
  {
    int w;
    short s[2];
  } count;    ///< samples elapsed in the current sequencer state
  int thrTx;  ///< length of the answer tone in samples (3 * rate)
  int thrSil; ///< length of the silence after the tone in samples (rate / 20)
  int rate;   ///< sample rate
  union
  {
    FpmTone* p;
    short s[2];
  } tone;    ///< answer-tone generator (NULL / zero when no answer tone is sent)
  V23Tx* tx; ///< transmitter
  void* rx;  ///< V23Rx or V23BwCh, as selected by `backend`
} V23Modem;

#if UINTPTR_MAX == 0xffffffffu
_Static_assert(sizeof(V23Rx) == 0x240 && offsetof(V23Rx, tone) == 4 && offsetof(V23Rx, agc0) == 8 && offsetof(V23Rx, agc1) == 0x34 &&
                   offsetof(V23Rx, mrf) == 0x60 && offsetof(V23Rx, fsd) == 0x7c && offsetof(V23Rx, iirCoef) == 0xb4 && offsetof(V23Rx, iirState) == 0xb8 &&
                   offsetof(V23Rx, iirCount) == 0xbc && offsetof(V23Rx, scratch) == 0xc0 && offsetof(V23Rx, bits) == 0x200 && offsetof(V23Rx, nBits) == 0x234 &&
                   offsetof(V23Rx, energyAcc) == 0x236 && offsetof(V23Rx, noCarrier) == 0x23e,
               "V23Rx 32-bit layout");
_Static_assert(sizeof(V23Tx) == 0x20 && offsetof(V23Tx, table) == 4 && offsetof(V23Tx, countdown) == 8 && offsetof(V23Tx, carry) == 0xc &&
                   offsetof(V23Tx, silence) == 0x14 && offsetof(V23Tx, freq0) == 0x18 && offsetof(V23Tx, tone) == 0x1c,
               "V23Tx 32-bit layout");
_Static_assert(sizeof(V23BwCh) == 0x68 && offsetof(V23BwCh, mark) == 8 && offsetof(V23BwCh, timeout) == 0xe && offsetof(V23BwCh, countdown) == 0x10 &&
                   offsetof(V23BwCh, last) == 0x1c && offsetof(V23BwCh, noCarAcc) == 0x20 && offsetof(V23BwCh, noCarThr) == 0x24 &&
                   offsetof(V23BwCh, toneCnt) == 0x28 && offsetof(V23BwCh, tone) == 0x2c && offsetof(V23BwCh, agc) == 0x30 &&
                   offsetof(V23BwCh, iirCoef) == 0x5c && offsetof(V23BwCh, iirState) == 0x60 && offsetof(V23BwCh, iirCount) == 0x64 &&
                   offsetof(V23BwCh, status) == 0x66,
               "V23BwCh 32-bit layout");
_Static_assert(sizeof(V23Modem) == 0x28 && offsetof(V23Modem, state) == 4 && offsetof(V23Modem, shadow) == 8 && offsetof(V23Modem, count) == 0xc &&
                   offsetof(V23Modem, thrTx) == 0x10 && offsetof(V23Modem, rate) == 0x18 && offsetof(V23Modem, tone) == 0x1c &&
                   offsetof(V23Modem, tx) == 0x20 && offsetof(V23Modem, rx) == 0x24,
               "V23Modem 32-bit layout");
#endif
#endif
