/**
 * @file v8_obj.h
 * @brief The V.8 engine object (V8Obj, 0xec4 bytes) with its embedded sub-objects (V8Msg, V8Det, V8Dft), the configuration record V8Cfg and the
 *        pointer-slot accessors.
 *
 * V8Obj is one flat block.  Everything in it is a 16- or 32-bit integer except nine pointer slots that hold real addresses: the receive and transmit
 * ring cursors, the receive end pointer (into the caller's sample buffer), the three message-slot pointers (all into the block) and the pointer to the
 * caller's capability block; plus the table-handle words of the sub-objects (the detector's coefficient table and the four V.21 correlator tables).
 * On x86-64 a pointer is 8 bytes, so
 * - every non-pointer member stays at its 32-bit image offset (`_Static_assert`s below pin them at both widths);
 * - the nine pointer slots live in a tail of 8-byte pointers placed after the flat image at V8_TAIL (their 4-byte slots in the image are unused);
 *   V8_PG / V8_PS read / write a slot by its image offset: at -m32 they are a plain access of the 4-byte member, on x86-64 they address the tail;
 * - the table-handle words stay 32-bit handles (core/dsp_table_handles.h: CFGP_SLOT / CFGP_SLOT_PUT / CFGP_WORD); v8_c2100 and the eight V.21
 *   filter tables are registered by v8_tables.c.
 * A harness that builds an object must allocate V8_ALLOC_SIZE bytes (8-aligned, zeroed) and set / read the pointer slots with V8_PS / V8_PG too.
 */
#ifndef DSPLIBS_PORT_V8_OBJ_H
#define DSPLIBS_PORT_V8_OBJ_H
#include <stddef.h>
#include <stdint.h>
#include "core/dsp_access.h"
#include "core/dsp_table_handles.h"

/** Size in bytes of the 32-bit image of the V.8 engine object. */
#define V8_IMG_SIZE 0xec4

/*
 * The object as C structs.  V8Obj is the 0xec4-byte engine block with its members at their 32-bit image offsets (asserted below, at both widths: there
 * is no pointer member; the nine pointer slots are 4-byte `unsigned int` members that hold the real address at -m32 and are unused on x86-64, where
 * V8_PG / V8_PS address the tail).  Member names are offset-derived (`w` = 16-bit, `d` = 32-bit, hex offset) unless the use of the member is
 * established; the comment on each says what it holds, and "medium" marks a reading that rests only on how the code uses the member.  `unkXXXX_`
 * members are bytes no function touches.  Access is plain member access (`p->member`, `p->arr[i]`).  A member has the type the code reads it as;
 * the members that some code reads both as `short` and as `unsigned short`, as another width, or through a different view (the phase pair, the flag
 * bytes of the capability block, the message words walked with a data-dependent index) are reached through MEMBER_AS(T, p, m) / MEMBER_AS_AT
 * (core/dsp_access.h), which names the type and takes the offset from the struct.  Sub-objects embedded in the block: the five message slots V8Msg
 * (stride 0x40: 15 coded words, then the cursor and CRC state), the tone detector V8Det, the first DFT bin V8Dft (v8_dft*); the phase-reversal
 * detector (v8_phase_rev_*) is reached through its own base and stays an opaque byte array here.
 */

/**
 * A message slot: the coded words of one message and the read / write cursor.  The slots of V8Obj::msg are 0 = CM, 1 = CJ, 2 = JM, 3 = the
 * quick-connect (QC) message the caller sends, 4 = the word buffer in which the QCA1 words are received.
 */
typedef struct V8Msg
{
  unsigned short data[15];
  unsigned short crcAcc;
  short crcEnable;
  short totalBits; ///< bits in the message (octets * 10)
  short bitsSent;  ///< bits already handed out
  short wordBits;  ///< bits per coded word (10)
  short wordIdx;   ///< TX slot: next data[] word; RX slot: number of words stored
  short repeatEnable;
  short passCount; ///< how often the message was restarted (repeatEnable)
  unsigned char unk002e_[2];
  unsigned int shiftWord; ///< word being shifted out
  short bitsInWord;       ///< bits left in shiftWord
  unsigned char unk0036_[2];
  unsigned int reloadWord;   ///< shiftWord on a restart (0 in this tree; medium)
  unsigned short reloadBits; ///< bitsInWord on a restart (0 in this tree; medium)
  unsigned char unk003e_[2];
} V8Msg;

_Static_assert(offsetof(V8Msg, data) == 0x0 && offsetof(V8Msg, crcAcc) == 0x1e && offsetof(V8Msg, crcEnable) == 0x20 && offsetof(V8Msg, totalBits) == 0x22,
               "V8Msg members 0 - 3");
_Static_assert(offsetof(V8Msg, bitsSent) == 0x24 && offsetof(V8Msg, wordBits) == 0x26 && offsetof(V8Msg, wordIdx) == 0x28 &&
                   offsetof(V8Msg, repeatEnable) == 0x2a,
               "V8Msg members 4 - 7");
_Static_assert(offsetof(V8Msg, passCount) == 0x2c && offsetof(V8Msg, shiftWord) == 0x30 && offsetof(V8Msg, bitsInWord) == 0x34 &&
                   offsetof(V8Msg, reloadWord) == 0x38,
               "V8Msg members 8 - 11");
_Static_assert(offsetof(V8Msg, reloadBits) == 0x3c, "V8Msg members 12 - 12");
_Static_assert(sizeof(V8Msg) == 0x40, "V8Msg size");

/** One bin of the single-frequency DFT accumulator (v8_dftupdate): phase, phase step, and the cosine / sine sums. */
typedef struct V8Dft
{
  unsigned short phase;
  unsigned short step;
  int re;
  int im;
  unsigned short energy;
  unsigned char unk000e_[2];
} V8Dft;

_Static_assert(offsetof(V8Dft, phase) == 0x0 && offsetof(V8Dft, step) == 0x2 && offsetof(V8Dft, re) == 0x4 && offsetof(V8Dft, im) == 0x8,
               "V8Dft members 0 - 3");
_Static_assert(offsetof(V8Dft, energy) == 0xc, "V8Dft members 4 - 4");
_Static_assert(sizeof(V8Dft) == 0x10, "V8Dft size");

/** The ANSam tone detector state (v8_detectorinit / v8_tone_detect). */
typedef struct V8Det
{
  unsigned char unk0000_[4];
  unsigned short mode; ///< selects the decision rule (v8_tone_detect); 0 at the one call site
  short armed;         ///< level stayed above the floor for 0x33 calls
  short hitCount;      ///< consecutive decisions that passed the threshold (starts at -hitHeadStart)
  short hitTarget;     ///< hitCount above this = tone detected
  unsigned short w00c;
  short hiThreshold; ///< mode != 0: level below this counts a hit (medium)
  short threshold;   ///< level above this counts a hit (mode 0)
  short level;       ///< running average of |filtered sample| >> 4
  short iirA[4];
  short iirB[4];
  unsigned short num[3];
  unsigned short den[3];
  short armCount; ///< consecutive calls with level above the floor (arms at 0x33)
  unsigned char unk0032_[2];
} V8Det;

_Static_assert(offsetof(V8Det, mode) == 0x4 && offsetof(V8Det, armed) == 0x6 && offsetof(V8Det, hitCount) == 0x8 && offsetof(V8Det, hitTarget) == 0xa,
               "V8Det members 0 - 3");
_Static_assert(offsetof(V8Det, w00c) == 0xc && offsetof(V8Det, hiThreshold) == 0xe && offsetof(V8Det, threshold) == 0x10 && offsetof(V8Det, level) == 0x12,
               "V8Det members 4 - 7");
_Static_assert(offsetof(V8Det, iirA) == 0x14 && offsetof(V8Det, iirB) == 0x1c && offsetof(V8Det, num) == 0x24 && offsetof(V8Det, den) == 0x2a,
               "V8Det members 8 - 11");
_Static_assert(offsetof(V8Det, armCount) == 0x30, "V8Det members 12 - 12");
_Static_assert(sizeof(V8Det) == 0x34, "V8Det size");

/** The V.8 engine object. */
typedef struct V8Obj
{
  unsigned char unk0000_[4];
  unsigned int d004;
  unsigned char unk0008_[4];
  unsigned short w00c;
  unsigned char unk000e_[6];
  unsigned short w014;
  unsigned char unk0016_[2];
  unsigned short w018;
  unsigned char unk001a_[12];
  unsigned short flags; ///< bit 0x200: hold AGC adaptation (set by v8_detectorinit, cleared on ANSam); other bits unknown (medium)
  unsigned char unk0028_[4];
  unsigned int rxEndPtr;
  unsigned int blockEnergy; ///< sum of squares of the last 4 AGC output samples
  unsigned char unk0034_[2];
  short rxLevel; ///< smoothed block energy (AGC target 4000)
  unsigned short agcGain;
  short gainErrAcc;    ///< integrated AGC level error (medium)
  short gainAdaptStep; ///< AGC adaptation step: 0x3333 at rxinit, 0x800 once a message is expected (medium)
  unsigned short rxHist[48];
  short rxHistPos;   ///< write position in rxHist[]
  short refAge;      ///< checkSignalStability: age of the gain reference (counts by 4 to 0x3c0)
  short refGain;     ///< checkSignalStability: agcGain snapshot
  short stableCount; ///< checkSignalStability: calls with the gain within 5% of refGain
  short gainStable;  ///< stableCount > 0x3bf
  unsigned char unk00a8_[32];
  short overflowCount; ///< AGC output overflows (10 -> reset agcGain)
  unsigned char unk00ca_[20];
  unsigned short w0de;
  unsigned char unk00e0_[2];
  unsigned short w0e2;
  unsigned short w0e4;
  unsigned char unk00e6_[14];
  unsigned short w0f4;
  unsigned short w0f6;
  unsigned char unk00f8_[24];
  short rxQueueCount; ///< samples in rxRing[] not yet read (v8_rxreadqueue subtracts 4)
  unsigned char unk0112_[2];
  unsigned int rxRingRdPtr;
  unsigned int rxRingWrPtr; ///< RX ring write cursor (V8Process)
  unsigned short rxRing[128];
  unsigned short txQueueCount; ///< samples in txRing[] not yet sent (+4 per v8_txwritequeue); its ADDRESS is also the end of rxRing[]
  unsigned char unk021e_[2];
  unsigned int txRingRdPtr; ///< TX ring read cursor (V8Process)
  unsigned int txRingWrPtr;
  unsigned short txRing[460];
  unsigned short txBuf[4];
  short rxBuf[4];
  unsigned char unk05d0_[428];
  unsigned short unk77c[140];
  unsigned short unk894[160];
  unsigned int phasePair;    ///< two halves: the TX phase (V8TxPhase, low half) and the RX phase (V8RxPhase, high half); read as shorts, once as one int
  unsigned short rxSubState; ///< third dispatch field, valid under rx phase 0x19 and 0x28
  unsigned char unk09da_[98];
  short txBit;                  ///< the FSK bit being sent
  unsigned short txQueueTarget; ///< v8handshak fills the TX ring up to this many samples per call (0x10)
  unsigned short wa40;
  short txGain;            ///< Q14 transmit level (0x4000)
  int side;                ///< V8Cfg.side: 0 = caller (originate), 1 = answerer
  int opMode;              ///< V8Cfg.opMode
  unsigned int sigTimeout; ///< V8Cfg.sigTimeout (seconds)
  unsigned int msgTimeout; ///< V8Cfg.msgTimeout (seconds)
  unsigned int cfgPad;     ///< V8Cfg.pad (9600 from dp_v8.c; never read)
  unsigned int capsPtr;
  unsigned short v21Coef[61];
  unsigned char unk0ad6_[2];
  V8Det det;
  unsigned char unk0b0c_[52];
  unsigned char phaseRev[222];
  unsigned char unk0c1e_[2];
  unsigned short fskTxPhase; ///< FSK transmit carrier phase (v8_fskmodulate)
  short fskSpaceStep;        ///< carrier step for a 0 bit (space)
  short fskMarkStep;         ///< carrier step for a 1 bit (mark)
  short bitSamples;          ///< samples per bit (0x20: 300 baud at 9600)
  short txBitPos;            ///< samples sent in the current bit
  short fskAmp;              ///< FSK transmit amplitude
  unsigned short wc2c;
  unsigned short wc2e;
  unsigned short spaceBit; ///< bit value of a space run (0)
  unsigned short markBit;  ///< bit value of a mark run (1)
  unsigned short wc34;
  short rxFill;              ///< samples collected in fskDelay[] (decision every 12)
  short rxBitCount;          ///< demodulated bits in rxShiftReg
  unsigned short rxShiftReg; ///< demodulated bit shift register
  unsigned char unk0c3c_[2];
  short zeroRun;             ///< consecutive 0 bits
  short oneRun;              ///< consecutive 1 bits
  short lastOneRun;          ///< length of the last run of 1 bits
  short preambleEnds;        ///< times six 0 bits followed a run of more than 9 ones (medium)
  short preambleEndsHandled; ///< preambleEnds already acted on by the resync (medium)
  unsigned int txMsgPtr;     ///< message slot being transmitted
  unsigned int rxMsgPtr;     ///< word buffer the received CM / JM is compared against
  unsigned int qcRxMsgPtr;   ///< word buffer of the QCA1 detector (msg[4])
  V8Msg msg[5];
  V8Dft dft;
  unsigned short amPhase; ///< ANSam 15 Hz amplitude-modulation phase
  short tonePhase;        ///< ANSam 2100 Hz tone phase
  unsigned short amStep;  ///< phase step of the amplitude modulation (0x1a)
  short toneStep;         ///< phase step of the tone (0xe00)
  short ansamAmp;         ///< ANSam amplitude; its sign flips every 0x438 calls (phase reversal)
  short reverseCount;     ///< calls since the last phase reversal
  unsigned char unk0db0_[2];
  short reverseEnable; ///< phase reversals on
  unsigned short wdb4;
  short stepCount;          ///< per-state tick / match counter, reused by the rx sub-states (medium)
  short zeroBits;           ///< zero bits in a row (CJ detection)
  short dcState;            ///< DC-removal filter state of V8Process
  unsigned short rxWordIdx; ///< next word of the message being compared / collected
  short cmEnabled;          ///< CM may be sent (opMode == 0 at init, else V8CTRL_START_CM)
  short qcPreambleCount;    ///< bit periods of the QC preamble sent (tx phase 0x2b ends at 0x3c)
  unsigned char unk0dc2_[2];
  unsigned int qcDetected;   ///< quick connect (QCA1) detected
  int qcLapm;                ///< QCA1 LAPM indication bit
  unsigned int ansPcmLevel;  ///< ANSpcm level index from QCA1d (copied to the capability block)
  unsigned short qca1aSeen;  ///< a good QCA1a was received
  unsigned short toneqPhase; ///< TONEq tone phase
  unsigned short toneqStep;  ///< TONEq phase step (0x688)
  unsigned char unk0dd6_[2];
  int spaceFilt1;        ///< handle of the space in-phase correlator table
  int spaceFilt2;        ///< handle of the space quadrature correlator table
  int markFilt1;         ///< handle of the mark in-phase correlator table
  int markFilt2;         ///< handle of the mark quadrature correlator table
  int decisionPhase;     ///< sample position of the next mark / space decision (+8 each)
  unsigned int markRun;  ///< decisions in a row that chose mark
  unsigned int spaceRun; ///< decisions in a row that chose space
  short fskDelay[52];
  int sigTimeoutTicks; ///< signal-detect timeout in handshake calls (-1 = none); its ADDRESS is also the end of fskDelay[]
  int msgTimeoutTicks; ///< message timeout in handshake calls (-1 = none)
  int timeoutCount;    ///< handshake calls spent waiting
  short agcDelay[40];
  int lastStatus;       ///< last V8Status V8Process returned
  short callFnAccepted; ///< the remote call function was accepted
  short protoAccepted;  ///< the remote protocol was accepted
  short callFnWord;     ///< the accepted call function word
  short protoWord;      ///< the accepted protocol word
} V8Obj;

_Static_assert(offsetof(V8Obj, d004) == 0x4 && offsetof(V8Obj, w00c) == 0xc && offsetof(V8Obj, w014) == 0x14 && offsetof(V8Obj, w018) == 0x18,
               "V8Obj members 0 - 3");
_Static_assert(offsetof(V8Obj, flags) == 0x26 && offsetof(V8Obj, rxEndPtr) == 0x2c && offsetof(V8Obj, blockEnergy) == 0x30 && offsetof(V8Obj, rxLevel) == 0x36,
               "V8Obj members 4 - 7");
_Static_assert(offsetof(V8Obj, agcGain) == 0x38 && offsetof(V8Obj, gainErrAcc) == 0x3a && offsetof(V8Obj, gainAdaptStep) == 0x3c &&
                   offsetof(V8Obj, rxHist) == 0x3e,
               "V8Obj members 8 - 11");
_Static_assert(offsetof(V8Obj, rxHistPos) == 0x9e && offsetof(V8Obj, refAge) == 0xa0 && offsetof(V8Obj, refGain) == 0xa2 &&
                   offsetof(V8Obj, stableCount) == 0xa4,
               "V8Obj members 12 - 15");
_Static_assert(offsetof(V8Obj, gainStable) == 0xa6 && offsetof(V8Obj, overflowCount) == 0xc8 && offsetof(V8Obj, w0de) == 0xde && offsetof(V8Obj, w0e2) == 0xe2,
               "V8Obj members 16 - 19");
_Static_assert(offsetof(V8Obj, w0e4) == 0xe4 && offsetof(V8Obj, w0f4) == 0xf4 && offsetof(V8Obj, w0f6) == 0xf6 && offsetof(V8Obj, rxQueueCount) == 0x110,
               "V8Obj members 20 - 23");
_Static_assert(offsetof(V8Obj, rxRingRdPtr) == 0x114 && offsetof(V8Obj, rxRingWrPtr) == 0x118 && offsetof(V8Obj, rxRing) == 0x11c &&
                   offsetof(V8Obj, txQueueCount) == 0x21c,
               "V8Obj members 24 - 27");
_Static_assert(offsetof(V8Obj, txRingRdPtr) == 0x220 && offsetof(V8Obj, txRingWrPtr) == 0x224 && offsetof(V8Obj, txRing) == 0x228 &&
                   offsetof(V8Obj, txBuf) == 0x5c0,
               "V8Obj members 28 - 31");
_Static_assert(offsetof(V8Obj, rxBuf) == 0x5c8 && offsetof(V8Obj, unk77c) == 0x77c && offsetof(V8Obj, unk894) == 0x894 && offsetof(V8Obj, phasePair) == 0x9d4,
               "V8Obj members 32 - 35");
_Static_assert(offsetof(V8Obj, rxSubState) == 0x9d8 && offsetof(V8Obj, txBit) == 0xa3c && offsetof(V8Obj, txQueueTarget) == 0xa3e &&
                   offsetof(V8Obj, wa40) == 0xa40,
               "V8Obj members 36 - 39");
_Static_assert(offsetof(V8Obj, txGain) == 0xa42 && offsetof(V8Obj, side) == 0xa44 && offsetof(V8Obj, opMode) == 0xa48 && offsetof(V8Obj, sigTimeout) == 0xa4c,
               "V8Obj members 40 - 43");
_Static_assert(offsetof(V8Obj, msgTimeout) == 0xa50 && offsetof(V8Obj, cfgPad) == 0xa54 && offsetof(V8Obj, capsPtr) == 0xa58 &&
                   offsetof(V8Obj, v21Coef) == 0xa5c,
               "V8Obj members 44 - 47");
_Static_assert(offsetof(V8Obj, det) == 0xad8 && offsetof(V8Obj, phaseRev) == 0xb40 && offsetof(V8Obj, fskTxPhase) == 0xc20 &&
                   offsetof(V8Obj, fskSpaceStep) == 0xc22,
               "V8Obj members 48 - 51");
_Static_assert(offsetof(V8Obj, fskMarkStep) == 0xc24 && offsetof(V8Obj, bitSamples) == 0xc26 && offsetof(V8Obj, txBitPos) == 0xc28 &&
                   offsetof(V8Obj, fskAmp) == 0xc2a,
               "V8Obj members 52 - 55");
_Static_assert(offsetof(V8Obj, wc2c) == 0xc2c && offsetof(V8Obj, wc2e) == 0xc2e && offsetof(V8Obj, spaceBit) == 0xc30 && offsetof(V8Obj, markBit) == 0xc32,
               "V8Obj members 56 - 59");
_Static_assert(offsetof(V8Obj, wc34) == 0xc34 && offsetof(V8Obj, rxFill) == 0xc36 && offsetof(V8Obj, rxBitCount) == 0xc38 &&
                   offsetof(V8Obj, rxShiftReg) == 0xc3a,
               "V8Obj members 60 - 63");
_Static_assert(offsetof(V8Obj, zeroRun) == 0xc3e && offsetof(V8Obj, oneRun) == 0xc40 && offsetof(V8Obj, lastOneRun) == 0xc42 &&
                   offsetof(V8Obj, preambleEnds) == 0xc44,
               "V8Obj members 64 - 67");
_Static_assert(offsetof(V8Obj, preambleEndsHandled) == 0xc46 && offsetof(V8Obj, txMsgPtr) == 0xc48 && offsetof(V8Obj, rxMsgPtr) == 0xc4c &&
                   offsetof(V8Obj, qcRxMsgPtr) == 0xc50,
               "V8Obj members 68 - 71");
_Static_assert(offsetof(V8Obj, msg) == 0xc54 && offsetof(V8Obj, dft) == 0xd94 && offsetof(V8Obj, amPhase) == 0xda4 && offsetof(V8Obj, tonePhase) == 0xda6,
               "V8Obj members 72 - 75");
_Static_assert(offsetof(V8Obj, amStep) == 0xda8 && offsetof(V8Obj, toneStep) == 0xdaa && offsetof(V8Obj, ansamAmp) == 0xdac &&
                   offsetof(V8Obj, reverseCount) == 0xdae,
               "V8Obj members 76 - 79");
_Static_assert(offsetof(V8Obj, reverseEnable) == 0xdb2 && offsetof(V8Obj, wdb4) == 0xdb4 && offsetof(V8Obj, stepCount) == 0xdb6 &&
                   offsetof(V8Obj, zeroBits) == 0xdb8,
               "V8Obj members 80 - 83");
_Static_assert(offsetof(V8Obj, dcState) == 0xdba && offsetof(V8Obj, rxWordIdx) == 0xdbc && offsetof(V8Obj, cmEnabled) == 0xdbe &&
                   offsetof(V8Obj, qcPreambleCount) == 0xdc0,
               "V8Obj members 84 - 87");
_Static_assert(offsetof(V8Obj, qcDetected) == 0xdc4 && offsetof(V8Obj, qcLapm) == 0xdc8 && offsetof(V8Obj, ansPcmLevel) == 0xdcc &&
                   offsetof(V8Obj, qca1aSeen) == 0xdd0,
               "V8Obj members 88 - 91");
_Static_assert(offsetof(V8Obj, toneqPhase) == 0xdd2 && offsetof(V8Obj, toneqStep) == 0xdd4 && offsetof(V8Obj, spaceFilt1) == 0xdd8 &&
                   offsetof(V8Obj, spaceFilt2) == 0xddc,
               "V8Obj members 92 - 95");
_Static_assert(offsetof(V8Obj, markFilt1) == 0xde0 && offsetof(V8Obj, markFilt2) == 0xde4 && offsetof(V8Obj, decisionPhase) == 0xde8 &&
                   offsetof(V8Obj, markRun) == 0xdec,
               "V8Obj members 96 - 99");
_Static_assert(offsetof(V8Obj, spaceRun) == 0xdf0 && offsetof(V8Obj, fskDelay) == 0xdf4 && offsetof(V8Obj, sigTimeoutTicks) == 0xe5c &&
                   offsetof(V8Obj, msgTimeoutTicks) == 0xe60,
               "V8Obj members 100 - 103");
_Static_assert(offsetof(V8Obj, timeoutCount) == 0xe64 && offsetof(V8Obj, agcDelay) == 0xe68 && offsetof(V8Obj, lastStatus) == 0xeb8 &&
                   offsetof(V8Obj, callFnAccepted) == 0xebc,
               "V8Obj members 104 - 107");
_Static_assert(offsetof(V8Obj, protoAccepted) == 0xebe && offsetof(V8Obj, callFnWord) == 0xec0 && offsetof(V8Obj, protoWord) == 0xec2,
               "V8Obj members 108 - 110");
_Static_assert(sizeof(V8Obj) == 0xec4, "V8Obj size");

/**
 * V8Create's configuration record: 6 words at -m32 (side, opMode, signal-detect timeout, message-detect timeout, unused, capability block pointer); the
 * last word is a pointer, so it is a real pointer member (x86-64: the record is 0x20 bytes, the pointer at offset 0x18).
 */
typedef struct V8Cfg
{
  unsigned int side, opMode, sigTimeout, msgTimeout, pad;
  const unsigned char* caps;
} V8Cfg;
#if UINTPTR_MAX == 0xffffffffu
_Static_assert(sizeof(V8Cfg) == 0x18 && offsetof(V8Cfg, caps) == 0x14, "V8Cfg 32-bit layout");
#endif

#if UINTPTR_MAX == 0xffffffffu

#define V8_ALLOC_SIZE V8_IMG_SIZE
#define V8_PG(T, p, off) (*(T*)((char*)(p) + (off)))
#define V8_PS(T, p, off, v) (*(T*)((char*)(p) + (off)) = (v))

#else

#define V8_TAIL 0xee0 /* start of the pointer tail: the image size rounded to 0xee0 (>= 0xec4) */
extern int v8_bad_pointer_slot_offset(void); /* never defined: a pointer-slot offset that is not in the list fails to link */
#define V8_PIDX(off)                                                                                                                                           \
  ((off) == 0x2c    ? 0                                                                                                                                        \
   : (off) == 0x114 ? 1                                                                                                                                        \
   : (off) == 0x118 ? 2                                                                                                                                        \
   : (off) == 0x220 ? 3                                                                                                                                        \
   : (off) == 0x224 ? 4                                                                                                                                        \
   : (off) == 0xa58 ? 5                                                                                                                                        \
   : (off) == 0xc48 ? 6                                                                                                                                        \
   : (off) == 0xc4c ? 7                                                                                                                                        \
   : (off) == 0xc50 ? 8                                                                                                                                        \
                    : v8_bad_pointer_slot_offset())
#define V8_NPTR 9
#define V8_ALLOC_SIZE (V8_TAIL + 8 * V8_NPTR)
#define V8_PSLOT(p, off) (*(void**)((char*)(p) + V8_TAIL + 8 * V8_PIDX(off)))
#define V8_PG(T, p, off) ((T)V8_PSLOT(p, off))
#define V8_PS(T, p, off, v) (V8_PSLOT(p, off) = (void*)(v))

#endif
/* slot access by member name: the offset of the 4-byte slot member (S is the struct type of the object `p`) */
#define V8_PGM(T, S, p, m) V8_PG(T, p, offsetof(S, m))
#define V8_PSM(T, S, p, m, v) V8_PS(T, p, offsetof(S, m), v)
/* the capability block the caller passes in V8Cfg (dp_v8.c: the DpRuntime): its flag bytes are named flags0 .. flags3 there */
#define V8_CAPS(c) ((DpRuntime*)(c)) /* not const: the engine clears / sets flag bits of the caller's block (the former MEMBER_AS also cast the const away) */
/* message slot i (a literal 0 .. 4) of the engine object `obj` (void * or V8Obj *) */
#define V8_MSG(obj, i) ((V8Msg*)MEMBER_ADDR((V8Obj*)(obj), msg[i]))
#endif
