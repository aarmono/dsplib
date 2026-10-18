/**
 * @file v34_records.h
 * @brief The generic bit-queue view of the V.34 module object.
 *
 *  - DftBin: one bin of the single-frequency DFT banks (0x2c bytes).  The object holds 25 + 4 + 3 of them (V34Probe `bin[25]`, `dftB_[4]`, `dftC_[3]`).
 *  - FdStage / TrellisMem: the Viterbi state of the frame decoder (`rx.trellis_`, 0x5a0 bytes; V34TRELLIS).
 *  - V34BitQTail / tag_bitQ: a bit-queue record (0x30 bytes).  The object holds six of them (the sub-object members info0a, info0d, info, info1c, q2, mp at
 *    offsets 0xa94c .. 0xaa6b of the 32-bit layout, each its own generated struct in v34_subobjects.h: the heads differ, the 28-byte queue state `tail_` is
 *    the shared V34BitQTail); the queue pointers bitQ_ / bitQ2_ and the functions that work on whichever queue is current take a tag_bitQ *.
 *
 * Plain C, included by the generated view of the object (v34_object_view.h).  Member names are position-based (`w<offset>_` = 16-bit word, `d<offset>_` =
 * 32-bit word) where no role is known.
 */
#ifndef V34_RECORDS_H
#define V34_RECORDS_H

#include <stdint.h>

/**
 * One bin of a DFT bank: a correlator for one frequency (dftupdate()) and its read-out (dftenergy()).  Layout (0x2c bytes, the same on i386 and x86-64):
 * the doubles are 4-aligned only (`#pragma pack(4)`: on i386 that is the natural alignment, on x86-64 it keeps the record at 0x2c bytes; bin k of an array
 * starts at 0x2c * k, so the doubles of the odd bins sit on a 4-byte boundary).
 */
#pragma pack(push, 4)
struct DftBin
{
  uint16_t phase;     ///< +0x00 14-bit phase of the local oscillator (dftupdate)
  uint16_t step;      ///< +0x02 phase step per sample: the bin frequency (k << 8 for the 25 probing bins, 0x600 .. 0x1100 for the 3 / 4 detector bins)
  int32_t accI;       ///< +0x04 integer I accumulator (wraps)
  int32_t accQ;       ///< +0x08 integer Q accumulator (wraps)
  int16_t energyHi;   ///< +0x0c high half of the integer energy (dftenergy); probeselect() reads it as the bin level
  int16_t normCnt;    ///< +0x0e normalisation count of the integer energy (dftenergy); probeselect() normalises it in place
  double sumI;        ///< +0x10 I sum (dftupdate, extended-precision add)
  double sumQ;        ///< +0x18 Q sum
  double energy;      ///< +0x20 floating energy sumI^2 + sumQ^2 (dftenergy); the 25 values are copied by V34GiveProbeResults()
  int16_t quietLimit; ///< +0x28 retrain-request detector limit of the quiet test (the handshake stores 0x50); not touched by dftupdate / dftenergy
  int16_t toneLimit;  ///< +0x2a limit of the tone test (the handshake stores 0xbb8 = 3000; v34modeminit 3000)
};
#pragma pack(pop)
typedef struct DftBin DftBin;

/**
 * One stage record of the frame decoder's trellis (12 bytes): the best end state found at the stage and the two received (I, Q) points of the stage
 * (V34TRELLIS; src/v34/v34_frame_decoder.c).  The stage number is `fdStage_` (0 .. 31).
 */
struct FdStage
{
  int16_t state;  ///< +0x00 best end state of the stage (0 .. 15), written by the add-compare-select step; start of the trace-back
  int16_t unk02_; ///< +0x02 never accessed (cleared by preinitdigital)
  union
  {
    int16_t pt[2][2]; ///< +0x04 the two received points of the stage: pt[0] = (I, Q) of the even symbol, pt[1] = (I, Q) of the odd symbol
    uint32_t pair[2]; ///< the same four words as two (I | Q << 16) pairs: the stores of the received points
  };
};
typedef struct FdStage FdStage;

/**
 * The whole trellis area of the frame decoder (0x5a0 bytes, the same on i386 and x86-64; `trellis_` of V34Rx at 0xeac).
 * - `metric[s]`: the 16 path metrics (renormalised so that the smallest is 0);
 * - `node[stage * 16 + s]`: the trellis memory.  Little-endian bytes: the high byte is the predecessor state (signed char; 0 .. 15 as written by the
 *   add-compare-select step), the low byte the chosen subset-pair value of the transition into state s at that stage;
 * - `stage[n]`: the 32 stage records.
 */
struct TrellisMem
{
  uint16_t metric[16];
  uint16_t node[32 * 16];
  FdStage stage[32];
};
typedef struct TrellisMem TrellisMem;

/**
 * The state of a bit queue (28 bytes, the same on i386 and x86-64): the part of a bit-queue record that follows the 20-byte word array.  The six
 * records of the object (info0a, info0d, info, info1c, q2, mp; V34Obj sub-objects `tail_`) and the generic view tag_bitQ share it.  The reader
 * (getbit / v04_bitq_getbit) delivers the message bits most significant first: it refills the shift register `acc` from the word array in chunks, feeds
 * the CRC-16 (x^16 + x^12 + x^5 + 1) with the data bits, appends the 16 CRC bits (when `crcOn`) and the 4-bit constant 0xf, and then either ends or
 * restarts from `restartAcc` / `restartBits` (when `loop` is set).
 */
struct V34BitQTail
{
  uint16_t crc;         ///< +0x00 running CRC-16 of the message bits (set to 0xffff at the start of a message)
  uint16_t crcOn;       ///< +0x02 non-zero: the CRC is computed and sent after the data
  uint16_t total;       ///< +0x04 length of the message in bits
  uint16_t pos;         ///< +0x06 bits taken from the word array / CRC / constant so far
  uint16_t chunk;       ///< +0x08 bits loaded from one data word into the shift register
  uint16_t widx;        ///< +0x0a index of the next data word (16-bit words from the start of the record)
  uint16_t loop;        ///< +0x0c non-zero: the message restarts when it ends
  uint16_t pass;        ///< +0x0e restart counter
  uint32_t acc;         ///< +0x10 shift register: the bits not yet delivered
  uint16_t nbits;       ///< +0x14 number of bits left in `acc`
  uint16_t restartBits; ///< +0x16 value of `nbits` at a restart
  uint32_t restartAcc;  ///< +0x18 value of `acc` at a restart
};
typedef struct V34BitQTail V34BitQTail;

/** The generic bit-queue record (0x30 bytes): the word array (INFO / MP words, the data of the message) and the queue state.  The object's current-queue pointers point at one of the six. */
struct tag_bitQ
{
  uint8_t unk00_[20]; ///< +0x00 the data words (addressed by byte offset from the start of the record)
  V34BitQTail tail_;  ///< +0x14 the queue state
};

#endif /* V34_RECORDS_H */
