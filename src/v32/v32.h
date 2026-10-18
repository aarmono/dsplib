/**
 * @file v32.h
 * @brief The V.32 / V.32bis engine objects: the wrapper, the handshake object and the engine, with the typed structs of their sub-objects.
 *
 * The V.32 modem is built from three blocks.  The wrapper (V32Wrap, 0x6c bytes) holds the configuration and the pointers to the other two; the
 * handshake object (V32Hdx, 0xb0 bytes) holds the handshake state machine, its timers and the tone objects; the engine (V32Eng, 0x50dc bytes) holds all DSP
 * state: the transmit scrambler (`sdm`, +0x30), the symbol encoder (`smc`, +0x48), the pulse shaper (`pps`, +0x60), the circular output descriptor (`circ`,
 * +0xb0), the matched receive filter (MRF, +0xc4), the echo canceller (`ecc`, +0xe0), the symbol timing estimator (`sre`, +0x148), the AGC (+0x1d8), the
 * equaliser (`fse`, +0x204), the decision history object (`hist`, +0x5020, with the Viterbi decoder `hist.vtb` at +0x5038) and the receive descrambler
 * (`rxSdm`, +0x50b0), all embedded as members at fixed offsets.  The files of this module reach the members by name (`p->member`, `p->sub.member`) and
 * pass `&p->sub` to the sub-object functions; the offsets are fixed by the `_Static_assert`s and relied on by the sub-object functions (each sees only
 * its own base, see V32X_PG).
 *
 * Everything in the image is a 16 / 32-bit integer EXCEPT
 *  - about 30 slots that hold real addresses (malloc()ed work buffers of PPS / ECC / SRE / FSE / VTB, the encoder function table, the circular
 *    descriptor's buffer pointers, the FSE hook slots at +0x230 (history object) and +0x234 (decision callback), the two copy buffers at +0x50cc / +0x50d0);
 *  - the table-pointer words of the configuration blocks (CFGP handles, dsp_table_handles.h: they stay 32-bit words);
 *  - the embedded FPM objects MRF (+0xc4) and AGC (+0x1d8), whose size grows on x86-64 (fpm.h).
 *
 * Address slots.  Every non-pointer field has the same offset on 32- and 64-bit hosts, so the image stays a flat block with fixed offsets.  On a 32-bit
 * host a slot is the 4-byte word in the image itself.  On x86-64 the slots live in an 8-byte-per-slot TAIL after the image (V32_TAIL) and the two FPM
 * objects live behind the slots (V32E_MRF / V32E_AGC).  `V32E_PG(T, e, off)` / `V32E_PS(T, e, off, v)` read / write the slot at engine offset `off`; an offset
 * that is not in the slot map fails to LINK (v32_bad_pointer_slot_offset).  A SUB-object function (PPS, SRE, ECC, FSE, VTB) is given only its own base:
 * `V32X_PG(T, sub, subbase, soff)` finds the engine as `sub - subbase`, so a sub-object MUST sit at its fixed offset inside a full engine allocation
 * (V32_ENG_ALLOC bytes, 8-aligned).  The wrapper has a tail of its own (V32W_PG / V32W_PS, V32_WRAP_ALLOC) and so has the handshake object
 * (V32H_PG / V32H_PS).
 * A local that holds the address of the engine or the handshake object is a `V32Eng *` / `V32Hdx *` (the slot macros cast to `char *` themselves); the
 * functions that work on the wrapper or on one embedded object (PPS, ECC, SRE, FSE, SMC, SDM, VTB) take that object's typed pointer.
 */
#ifndef DSPLIBS_PORT_V32_OBJ_H
#define DSPLIBS_PORT_V32_OBJ_H
#include <stddef.h>
#include <stdint.h>
#include "core/dsp_access.h"
#include "core/dsp_table_handles.h"
#include "v32_states.h" /* enums V32State / V32Role / V32RateClass / V32Mode */

typedef void (*v32fn_t)(void); /* a handler / hook function-pointer slot */

#define V32_ENG_IMG 0x50dc /* size of the engine image in bytes */
#define V32_WRAP_IMG 0x6c  /* size of the wrapper in bytes */

/**
 * @brief The objects as C structs.
 *
 * V32Wrap (0x6c), V32Hdx (0xb0) and V32Eng (0x50dc) are the three blocks, with every member at its fixed offset (asserted below at both pointer widths).  No
 * member is a pointer: an address slot is a 4-byte `unsigned int` member that holds the address on a 32-bit host and is unused on x86-64, where the
 * V32?_PG / V32?_PS slot macros address the tail.  Member names are offset-derived (`w` = 16-bit, `d` = 32-bit, `b` = byte, hex offset) unless the code shows
 * what the member is; `unkXXXX_` pads are bytes no function touches.  A member has the signed type (`short` / `int`) when every read names one signed type of
 * its width; the others are read with several types or widths through the cast macros (MEMBER_AS_ADDR(T, S, base, m) and relatives, core/dsp_access.h; the
 * offset comes from the struct).
 *
 * The sub-objects of V32Eng are members of the view struct types below (V32Sdm, V32Smc, V32Pps, V32Circ, V32Ecc, V32Sre, V32Fse, V32Hist); a sub-object
 * function takes a pointer to its own struct.  V32Hist is the object the slot `fse.histObj` points to (engine +0x5020; it embeds the Viterbi object V32Vtb
 * at +0x18).  The embedded FPM objects MRF and AGC stay byte arrays (their size differs between hosts) and the engine's leading option words are the word array `opt`
 * as the word array `opt`.  A 16-bit half of a 32-bit word that the engine stores on its own is an anonymous-union member (V32Sdm::symWidth / tapPos,
 * V32Ecc::w000 / nearTaps, V32Circ::ringLen16).
 *
 * The member descriptions were derived from how the code uses each member.
 */

/** @brief Viterbi (trellis) decoder state of the V.32bis receiver. */
typedef struct V32Vtb
{
  unsigned int histBuf; ///< survivor history buffer (address slot)
  short pathMetric[8];  ///< path metrics of the eight trellis states
  short writeIdx;       ///< write position in the survivor history
  unsigned char unk0016_[2];
  unsigned int iMap;     ///< I coordinates of the constellation (address slot)
  unsigned int qMap;     ///< Q coordinates of the constellation (address slot)
  unsigned int boundTab; ///< candidate-point bound table of the rate family (address slot)
  short family;          ///< rate family: 1 = 7200, 2 = 9600 trellis, 3 = 12000, 4 = 14400 / default
  unsigned char unk0026_[2];
  unsigned int regionTab; ///< region quantisation table of the rate family (address slot)
  short rowSize;          ///< row size of the candidate tables (2 / 4 / 6 / 8)
  short tracebackDepth;   ///< traceback depth in symbols (fixed 16)
  short prevQuadrant;     ///< differential decoder state: previous quadrant, carried between calls
  unsigned short lowMask; ///< mask of the uncoded low bits of a symbol
  short diffShift;        ///< shift applied by the differential decode (equals `family`)
} V32Vtb;

_Static_assert(offsetof(V32Vtb, histBuf) == 0x0 && offsetof(V32Vtb, pathMetric) == 0x4 && offsetof(V32Vtb, writeIdx) == 0x14 && offsetof(V32Vtb, iMap) == 0x18,
               "V32Vtb members 0 - 3");
_Static_assert(offsetof(V32Vtb, qMap) == 0x1c && offsetof(V32Vtb, boundTab) == 0x20 && offsetof(V32Vtb, family) == 0x24 && offsetof(V32Vtb, regionTab) == 0x28,
               "V32Vtb members 4 - 7");
_Static_assert(offsetof(V32Vtb, rowSize) == 0x2c && offsetof(V32Vtb, tracebackDepth) == 0x2e && offsetof(V32Vtb, prevQuadrant) == 0x30 &&
                   offsetof(V32Vtb, lowMask) == 0x32,
               "V32Vtb members 8 - 11");
_Static_assert(offsetof(V32Vtb, diffShift) == 0x34, "V32Vtb members 12 - 12");

/** @brief Decision history shared by the equaliser decision callbacks; the Viterbi object V32Vtb follows it at +0x18. */
typedef struct V32Hist
{
  unsigned short w000; ///< not individually identified
  unsigned char unk0002_[2];
  unsigned short w004; ///< not individually identified
  unsigned short w006; ///< not individually identified
  unsigned short w008; ///< not individually identified
  unsigned char unk000a_[14];
  V32Vtb vtb;                  ///< the Viterbi decoder state (the decision callbacks hand it to VTB_decoder)
  short w050;                  ///< not individually identified
  short symI0;                 ///< most recent decided symbol, I
  short symQ0;                 ///< most recent decided symbol, Q
  short symI1;                 ///< previous decided symbol, I
  short symQ1;                 ///< previous decided symbol, Q
  short symI2;                 ///< decided symbol two intervals back, I
  short symQ2;                 ///< decided symbol two intervals back, Q
  short w05e;                  ///< not individually identified
  short w060;                  ///< not individually identified
  unsigned short evFlags;      ///< event flags raised by the decision callbacks (bit 0 read by RetrainDetectV32, bit 1 by RenegotiateDetectV32)
  unsigned int epochFlag;      ///< set by the callbacks when their detection sequence completes (read by EpochDetectV32)
  unsigned int d068;           ///< not individually identified
  short lfsrShift;             ///< tap position of the pattern register `lfsr`
  unsigned short stateCount;   ///< symbols counted by the detection sequence of the current callback
  unsigned int lfsr;           ///< pattern register the callbacks advance two bits per symbol
  unsigned short retrainCount; ///< number of retrain events reported through evFlags bit 0
} V32Hist;

_Static_assert(offsetof(V32Hist, w008) == 0x8 && offsetof(V32Hist, vtb) == 0x18 && offsetof(V32Hist, w050) == 0x50 && offsetof(V32Hist, symI0) == 0x52,
               "V32Hist members 0 - 3");
_Static_assert(offsetof(V32Hist, symQ0) == 0x54 && offsetof(V32Hist, symI1) == 0x56 && offsetof(V32Hist, symQ1) == 0x58 && offsetof(V32Hist, symI2) == 0x5a,
               "V32Hist members 4 - 7");
_Static_assert(offsetof(V32Hist, symQ2) == 0x5c && offsetof(V32Hist, w05e) == 0x5e && offsetof(V32Hist, w060) == 0x60 && offsetof(V32Hist, evFlags) == 0x62,
               "V32Hist members 8 - 11");
_Static_assert(offsetof(V32Hist, epochFlag) == 0x64 && offsetof(V32Hist, lfsrShift) == 0x6c && offsetof(V32Hist, stateCount) == 0x6e &&
                   offsetof(V32Hist, lfsr) == 0x70,
               "V32Hist members 12 - 15");
_Static_assert(offsetof(V32Hist, retrainCount) == 0x74, "V32Hist members 16 - 16");

/** @brief Echo canceller view of the engine image (FPM_ECC_*). */
typedef struct V32Ecc
{
  union
  {
    unsigned int d000; ///< `w000` and `nearTaps` as one word
    struct
    {
      unsigned short w000; ///< not individually identified
      short nearTaps;      ///< near-end tap count
    };
  };
  union
  {
    unsigned int d004; ///< `w004` and `w006` as one word
    struct
    {
      unsigned short w004; ///< not individually identified
      unsigned short w006; ///< not individually identified
    };
  };
  unsigned int iMapTab;  ///< table of I constellation maps, one per rate class (address slot)
  unsigned int qMapTab;  ///< table of Q constellation maps, one per rate class (address slot)
  unsigned int fillCode; ///< code value the training fill writes into the code history
  unsigned int d014;     ///< not individually identified
  unsigned short w018;   ///< not individually identified
  unsigned short w01a;   ///< not individually identified
  unsigned short w01c;   ///< not individually identified
  unsigned short w01e;   ///< not individually identified
  short inPower;         ///< smoothed input power
  short errPower;        ///< smoothed residual error power
  unsigned short w024;   ///< not individually identified
  unsigned char unk0026_[2];
  int nearAdapt;            ///< near-end adaptation enable
  int farAdapt;             ///< far-end adaptation enable
  short codeRdNear;         ///< read position of the near-end code history
  unsigned short codeRdFar; ///< read position of the far-end code history
  short codeRingLen;        ///< length of the code history ring
  unsigned char unk0036_[2];
  unsigned int codeHist;   ///< transmitted code history (address slot)
  unsigned short nearPos;  ///< write position of the near-end sample history
  unsigned short nearRing; ///< length of the near-end sample history ring
  unsigned int nearIHist;  ///< near-end I history (address slot)
  unsigned int nearQHist;  ///< near-end Q history (address slot)
  unsigned short farPos;   ///< write position of the far-end sample history
  unsigned short farRing;  ///< length of the far-end sample history ring
  unsigned int farIHist;   ///< far-end I history (address slot)
  unsigned int farQHist;   ///< far-end Q history (address slot)
  unsigned int tapSets;    ///< tap sets (address slot)
  unsigned char unk0058_[8];
  unsigned short stepSize; ///< adaptation step size
  unsigned short tapPhase; ///< phase of the every-third-sample tap injection
  unsigned short w064;     ///< not individually identified
  unsigned short w066;     ///< not individually identified
} V32Ecc;

_Static_assert(offsetof(V32Ecc, d000) == 0x0 && offsetof(V32Ecc, d004) == 0x4 && offsetof(V32Ecc, iMapTab) == 0x8 && offsetof(V32Ecc, qMapTab) == 0xc,
               "V32Ecc members 0 - 3");
_Static_assert(offsetof(V32Ecc, fillCode) == 0x10 && offsetof(V32Ecc, d014) == 0x14 && offsetof(V32Ecc, w018) == 0x18 && offsetof(V32Ecc, w01a) == 0x1a,
               "V32Ecc members 4 - 7");
_Static_assert(offsetof(V32Ecc, w01c) == 0x1c && offsetof(V32Ecc, w01e) == 0x1e && offsetof(V32Ecc, inPower) == 0x20 && offsetof(V32Ecc, errPower) == 0x22,
               "V32Ecc members 8 - 11");
_Static_assert(offsetof(V32Ecc, w024) == 0x24 && offsetof(V32Ecc, nearAdapt) == 0x28 && offsetof(V32Ecc, farAdapt) == 0x2c &&
                   offsetof(V32Ecc, codeRdNear) == 0x30,
               "V32Ecc members 12 - 15");
_Static_assert(offsetof(V32Ecc, codeRdFar) == 0x32 && offsetof(V32Ecc, codeRingLen) == 0x34 && offsetof(V32Ecc, codeHist) == 0x38 &&
                   offsetof(V32Ecc, nearPos) == 0x3c,
               "V32Ecc members 16 - 19");
_Static_assert(offsetof(V32Ecc, nearRing) == 0x3e && offsetof(V32Ecc, nearIHist) == 0x40 && offsetof(V32Ecc, nearQHist) == 0x44 &&
                   offsetof(V32Ecc, farPos) == 0x48,
               "V32Ecc members 20 - 23");
_Static_assert(offsetof(V32Ecc, farRing) == 0x4a && offsetof(V32Ecc, farIHist) == 0x4c && offsetof(V32Ecc, farQHist) == 0x50 &&
                   offsetof(V32Ecc, tapSets) == 0x54,
               "V32Ecc members 24 - 27");
_Static_assert(offsetof(V32Ecc, stepSize) == 0x60 && offsetof(V32Ecc, tapPhase) == 0x62 && offsetof(V32Ecc, w064) == 0x64 && offsetof(V32Ecc, w066) == 0x66,
               "V32Ecc members 28 - 31");

/** @brief Fractionally spaced equaliser and carrier loop view of the engine image (FPM_FSE_*). */
typedef struct V32Fse
{
  unsigned char unk0000_[2];
  short w002;          ///< not individually identified
  int iInitTab;        ///< initial I tap table (address)
  int qInitTab;        ///< initial Q tap table (address)
  short nTaps;         ///< number of equaliser taps
  unsigned short w00e; ///< not individually identified
  unsigned short w010; ///< not individually identified
  unsigned char unk0012_[2];
  int carrierTab;    ///< carrier phase table (address)
  short phaseWrap;   ///< carrier phase accumulator modulus
  short phaseInc;    ///< carrier phase increment per sample
  short settleCount; ///< symbols to wait before the loop is considered settled
  short errThreshHi; ///< upper error threshold of the loop
  short errThreshLo; ///< lower error threshold of the loop
  unsigned char unk0022_[2];
  int propTab;             ///< proportional gain table of the carrier loop (address)
  int intTab;              ///< integral gain table of the carrier loop (address)
  unsigned int histObj;    ///< decision history object (V32Hist, address slot)
  unsigned int decisionFn; ///< current decision callback (address slot)
  unsigned char unk0034_[4];
  unsigned short w038;    ///< not individually identified
  short loopState;        ///< state of the carrier loop
  int freqAcc;            ///< frequency accumulator
  int d040;               ///< not individually identified
  unsigned int d044;      ///< not individually identified
  unsigned int d048;      ///< not individually identified
  unsigned int d04c;      ///< not individually identified
  short phaseErrAvg;      ///< smoothed phase error
  short decErrAvg;        ///< smoothed decision error
  unsigned int symIBuf;   ///< symbol I buffer (address slot)
  unsigned int symQBuf;   ///< symbol Q buffer (address slot)
  unsigned short inCount; ///< input samples accumulated for the current symbol
  short outIdx;           ///< output symbol index
  unsigned int iTaps;     ///< I tap vector (address slot)
  unsigned int qTaps;     ///< Q tap vector (address slot)
  unsigned int rxHist;    ///< received sample history (address slot)
  short histPos;          ///< position in the received sample history
  unsigned char unk006e_[2];
  int carrierAcc;           ///< carrier phase accumulator
  short phaseAcc;           ///< phase accumulator
  short w076;               ///< not individually identified
  short unk078[8];          ///< words no function reads or writes
  unsigned short settleCtr; ///< counts the symbols until the loop settles
  short untilSym;           ///< samples until the next symbol is due
  int diagA[960];           ///< diagnostic buffer A (read by FSE_getdiag)
  int diagB[4000];          ///< diagnostic buffer B (read by FSE_getdiag)
  int diagACount;           ///< entries in diagnostic buffer A
  unsigned short w4e10;     ///< not individually identified
  unsigned short w4e12;     ///< not individually identified
  int diagBCount;           ///< entries in diagnostic buffer B
} V32Fse;

_Static_assert(offsetof(V32Fse, w002) == 0x2 && offsetof(V32Fse, iInitTab) == 0x4 && offsetof(V32Fse, qInitTab) == 0x8 && offsetof(V32Fse, nTaps) == 0xc,
               "V32Fse members 0 - 3");
_Static_assert(offsetof(V32Fse, w00e) == 0xe && offsetof(V32Fse, carrierTab) == 0x14 && offsetof(V32Fse, phaseWrap) == 0x18 &&
                   offsetof(V32Fse, phaseInc) == 0x1a,
               "V32Fse members 4 - 7");
_Static_assert(offsetof(V32Fse, settleCount) == 0x1c && offsetof(V32Fse, errThreshHi) == 0x1e && offsetof(V32Fse, errThreshLo) == 0x20 &&
                   offsetof(V32Fse, propTab) == 0x24,
               "V32Fse members 8 - 11");
_Static_assert(offsetof(V32Fse, intTab) == 0x28 && offsetof(V32Fse, histObj) == 0x2c && offsetof(V32Fse, decisionFn) == 0x30 && offsetof(V32Fse, w038) == 0x38,
               "V32Fse members 12 - 15");
_Static_assert(offsetof(V32Fse, loopState) == 0x3a && offsetof(V32Fse, freqAcc) == 0x3c && offsetof(V32Fse, d040) == 0x40 && offsetof(V32Fse, d044) == 0x44,
               "V32Fse members 16 - 19");
_Static_assert(offsetof(V32Fse, d048) == 0x48 && offsetof(V32Fse, d04c) == 0x4c && offsetof(V32Fse, phaseErrAvg) == 0x50 && offsetof(V32Fse, decErrAvg) == 0x52,
               "V32Fse members 20 - 23");
_Static_assert(offsetof(V32Fse, symIBuf) == 0x54 && offsetof(V32Fse, symQBuf) == 0x58 && offsetof(V32Fse, inCount) == 0x5c && offsetof(V32Fse, outIdx) == 0x5e,
               "V32Fse members 24 - 27");
_Static_assert(offsetof(V32Fse, iTaps) == 0x60 && offsetof(V32Fse, qTaps) == 0x64 && offsetof(V32Fse, rxHist) == 0x68 && offsetof(V32Fse, histPos) == 0x6c,
               "V32Fse members 28 - 31");
_Static_assert(offsetof(V32Fse, carrierAcc) == 0x70 && offsetof(V32Fse, phaseAcc) == 0x74 && offsetof(V32Fse, w076) == 0x76 && offsetof(V32Fse, unk078) == 0x78,
               "V32Fse members 32 - 35");
_Static_assert(offsetof(V32Fse, settleCtr) == 0x88 && offsetof(V32Fse, untilSym) == 0x8a && offsetof(V32Fse, diagA) == 0x8c && offsetof(V32Fse, diagB) == 0xf8c,
               "V32Fse members 36 - 39");
_Static_assert(offsetof(V32Fse, diagACount) == 0x4e0c && offsetof(V32Fse, w4e10) == 0x4e10 && offsetof(V32Fse, w4e12) == 0x4e12 &&
                   offsetof(V32Fse, diagBCount) == 0x4e14,
               "V32Fse members 40 - 43");

/** @brief Symbol rate (timing) estimator view of the engine image (FPM_SRE_*). */
typedef struct V32Sre
{
  unsigned char unk0000_[2];
  unsigned short periodsTrack; ///< symbol periods averaged while tracking
  short periodsLock;           ///< symbol periods averaged while locking
  short w006;                  ///< not individually identified
  short w008;                  ///< not individually identified
  short w00a;                  ///< not individually identified
  short coefLen;               ///< length of the filter coefficient table
  unsigned char unk000e_[2];
  int coefTab;           ///< filter coefficient table (address)
  unsigned char tedCoef; ///< timing error detector coefficient
  unsigned char unk0015_[3];
  int corrCosTab;    ///< cosine table of the correlator (address)
  int corrSinTab;    ///< sine table of the correlator (address)
  int propGainTab;   ///< proportional gain table of the timing loop (address)
  int intGainTab;    ///< integral gain table of the timing loop (address)
  short errOn;       ///< timing error level that switches the correction on
  short errOff;      ///< timing error level that switches the correction off
  short w02c;        ///< not individually identified
  short w02e;        ///< not individually identified
  short levelThresh; ///< signal level threshold
  short levelLen;    ///< number of samples in a level measurement
  unsigned char unk0034_[4];
  short loopState;         ///< state of the timing loop
  short loopInteg;         ///< integrator of the timing loop
  short phaseAvg;          ///< smoothed timing phase
  short errMagAvg;         ///< smoothed timing error magnitude
  int pllEnable;           ///< timing PLL enable
  unsigned int d044;       ///< not individually identified
  unsigned int loopEnable; ///< timing loop enable
  short ringLen;           ///< length of the sample ring
  short ringPos;           ///< position in the sample ring
  unsigned int coefBuf;    ///< coefficient work buffer (address slot)
  unsigned int ringBuf;    ///< sample ring (address slot)
  unsigned int tedState;   ///< timing error detector state (address slot)
  int corrA;               ///< correlator accumulator A
  int corrB;               ///< correlator accumulator B
  short phaseFrac;         ///< fractional timing phase
  short phaseRow;          ///< coefficient row selected by the timing phase
  short periodsCur;        ///< symbol periods currently averaged
  short w06a;              ///< not individually identified
  short periodCount;       ///< periods counted so far
  short corrIdx;           ///< correlator table index
  short untilSym;          ///< samples until the next symbol is due
  unsigned char unk0072_[2];
  unsigned int levelBuf;  ///< level measurement buffer (address slot)
  short levelMeasure;     ///< last level measurement
  short levelIdx;         ///< index into the level buffer
  unsigned short w07c;    ///< not individually identified
  short w07e;             ///< not individually identified
  short ppmSum;           ///< sum of the clock offset estimates
  short ppmOffset;        ///< clock offset estimate
  short ppmN;             ///< number of offset estimates summed
  short ppmNet;           ///< net clock offset
  unsigned short ppmStep; ///< clock offset correction step
  short w08a;             ///< not individually identified
  short ppmNMax;          ///< number of estimates after which the offset is applied
  short w08e;             ///< not individually identified
} V32Sre;

_Static_assert(offsetof(V32Sre, periodsTrack) == 0x2 && offsetof(V32Sre, periodsLock) == 0x4 && offsetof(V32Sre, w006) == 0x6 && offsetof(V32Sre, w008) == 0x8,
               "V32Sre members 0 - 3");
_Static_assert(offsetof(V32Sre, w00a) == 0xa && offsetof(V32Sre, coefLen) == 0xc && offsetof(V32Sre, coefTab) == 0x10 && offsetof(V32Sre, tedCoef) == 0x14,
               "V32Sre members 4 - 7");
_Static_assert(offsetof(V32Sre, corrCosTab) == 0x18 && offsetof(V32Sre, corrSinTab) == 0x1c && offsetof(V32Sre, propGainTab) == 0x20 &&
                   offsetof(V32Sre, intGainTab) == 0x24,
               "V32Sre members 8 - 11");
_Static_assert(offsetof(V32Sre, errOn) == 0x28 && offsetof(V32Sre, errOff) == 0x2a && offsetof(V32Sre, w02c) == 0x2c && offsetof(V32Sre, w02e) == 0x2e,
               "V32Sre members 12 - 15");
_Static_assert(offsetof(V32Sre, levelThresh) == 0x30 && offsetof(V32Sre, levelLen) == 0x32 && offsetof(V32Sre, loopState) == 0x38 &&
                   offsetof(V32Sre, loopInteg) == 0x3a,
               "V32Sre members 16 - 19");
_Static_assert(offsetof(V32Sre, phaseAvg) == 0x3c && offsetof(V32Sre, errMagAvg) == 0x3e && offsetof(V32Sre, pllEnable) == 0x40 &&
                   offsetof(V32Sre, d044) == 0x44,
               "V32Sre members 20 - 23");
_Static_assert(offsetof(V32Sre, loopEnable) == 0x48 && offsetof(V32Sre, ringLen) == 0x4c && offsetof(V32Sre, ringPos) == 0x4e &&
                   offsetof(V32Sre, coefBuf) == 0x50,
               "V32Sre members 24 - 27");
_Static_assert(offsetof(V32Sre, ringBuf) == 0x54 && offsetof(V32Sre, tedState) == 0x58 && offsetof(V32Sre, corrA) == 0x5c && offsetof(V32Sre, corrB) == 0x60,
               "V32Sre members 28 - 31");
_Static_assert(offsetof(V32Sre, phaseFrac) == 0x64 && offsetof(V32Sre, phaseRow) == 0x66 && offsetof(V32Sre, periodsCur) == 0x68 &&
                   offsetof(V32Sre, w06a) == 0x6a,
               "V32Sre members 32 - 35");
_Static_assert(offsetof(V32Sre, periodCount) == 0x6c && offsetof(V32Sre, corrIdx) == 0x6e && offsetof(V32Sre, untilSym) == 0x70 &&
                   offsetof(V32Sre, levelBuf) == 0x74,
               "V32Sre members 36 - 39");
_Static_assert(offsetof(V32Sre, levelMeasure) == 0x78 && offsetof(V32Sre, levelIdx) == 0x7a && offsetof(V32Sre, w07c) == 0x7c && offsetof(V32Sre, w07e) == 0x7e,
               "V32Sre members 40 - 43");
_Static_assert(offsetof(V32Sre, ppmSum) == 0x80 && offsetof(V32Sre, ppmOffset) == 0x82 && offsetof(V32Sre, ppmN) == 0x84 && offsetof(V32Sre, ppmNet) == 0x86,
               "V32Sre members 44 - 47");
_Static_assert(offsetof(V32Sre, ppmStep) == 0x88 && offsetof(V32Sre, w08a) == 0x8a && offsetof(V32Sre, ppmNMax) == 0x8c && offsetof(V32Sre, w08e) == 0x8e,
               "V32Sre members 48 - 51");

/** @brief Pulse shaper (phase splitter) view of the engine image (FPM_PPS_*). */
typedef struct V32Pps
{
  unsigned char unk0000_[2];
  short phaseStep;         ///< phase step per output sample
  int mapped;              ///< non-zero: take the samples from the constellation maps, zero: from the circular descriptor
  int gain;                ///< output gain
  unsigned int phaseStep2; ///< phase step, 32-bit copy
  int iMap;                ///< I constellation map (address)
  int qMap;                ///< Q constellation map (address)
  int iCoef;               ///< I shaping filter coefficients (address)
  int qCoef;               ///< Q shaping filter coefficients (address)
  unsigned int firLen;     ///< shaping filter length in taps
  unsigned int d024;       ///< not individually identified
  short needSample;        ///< non-zero when the next output needs a new symbol
  short phase;             ///< phase within the symbol interval
  short writeIdx;          ///< write index in the symbol histories
  short histLen;           ///< length of the symbol histories
  unsigned int iHistBuf;   ///< I symbol history (address slot)
  unsigned int qHistBuf;   ///< Q symbol history (address slot)
} V32Pps;

_Static_assert(offsetof(V32Pps, phaseStep) == 0x2 && offsetof(V32Pps, mapped) == 0x4 && offsetof(V32Pps, gain) == 0x8 && offsetof(V32Pps, phaseStep2) == 0xc,
               "V32Pps members 0 - 3");
_Static_assert(offsetof(V32Pps, iMap) == 0x10 && offsetof(V32Pps, qMap) == 0x14 && offsetof(V32Pps, iCoef) == 0x18 && offsetof(V32Pps, qCoef) == 0x1c,
               "V32Pps members 4 - 7");
_Static_assert(offsetof(V32Pps, firLen) == 0x20 && offsetof(V32Pps, d024) == 0x24 && offsetof(V32Pps, needSample) == 0x28 && offsetof(V32Pps, phase) == 0x2a,
               "V32Pps members 8 - 11");
_Static_assert(offsetof(V32Pps, writeIdx) == 0x2c && offsetof(V32Pps, histLen) == 0x2e && offsetof(V32Pps, iHistBuf) == 0x30 &&
                   offsetof(V32Pps, qHistBuf) == 0x34,
               "V32Pps members 12 - 15");

/** @brief Scrambler / descrambler view of the engine image. */
typedef struct V32Sdm
{
  union
  {
    unsigned int d000; ///< `symWidth` and `tapPos` as one word
    struct
    {
      unsigned short symWidth; ///< bits per symbol of the rate class
      short tapPos;            ///< position of the upper feedback tap of the scrambler polynomial
    };
  };
  short regLen; ///< register length
  unsigned char unk0006_[2];
  unsigned int symMask;  ///< mask of the bits of one symbol
  unsigned int keepMask; ///< mask of the register bits kept when a symbol is shifted in
  unsigned int shiftReg; ///< scrambler register
  short tapShiftA;       ///< first feedback tap position
  short tapShiftB;       ///< second feedback tap position
} V32Sdm;

_Static_assert(offsetof(V32Sdm, symMask) == 0x8 && offsetof(V32Sdm, keepMask) == 0xc && offsetof(V32Sdm, shiftReg) == 0x10 &&
                   offsetof(V32Sdm, tapShiftA) == 0x14,
               "V32Sdm members 0 - 3");
_Static_assert(offsetof(V32Sdm, tapShiftB) == 0x16, "V32Sdm members 4 - 4");
_Static_assert(offsetof(V32Sdm, d000) == 0x0 && offsetof(V32Sdm, symWidth) == 0x0 && offsetof(V32Sdm, tapPos) == 0x2 && offsetof(V32Sdm, regLen) == 0x4,
               "V32Sdm members 5 - 8");
_Static_assert(sizeof(V32Sdm) == 0x18, "V32Sdm size");

/** @brief Symbol mapper / trellis encoder view of the engine image. */
typedef struct V32Smc
{
  unsigned short rateIdx;    ///< constellation / rate index of the transmit mode (0 = 4800 .. 5 = 14400)
  unsigned short w002;       ///< second mode word (0 .. 2)
  unsigned short codedShift; ///< shift that isolates the coded bits
  short phase;               ///< differential encoder phase
  unsigned short w008;       ///< not individually identified
  unsigned short w00a;       ///< not individually identified
  unsigned char unk000c_[2];
  short trellisState;         ///< state of the trellis encoder
  unsigned short transVal;    ///< last trellis transition value
  unsigned short w012;        ///< not individually identified
  unsigned short payloadBits; ///< payload bits per symbol
} V32Smc;

_Static_assert(offsetof(V32Smc, codedShift) == 0x4 && offsetof(V32Smc, phase) == 0x6 && offsetof(V32Smc, w008) == 0x8 && offsetof(V32Smc, trellisState) == 0xe,
               "V32Smc members 0 - 3");
_Static_assert(offsetof(V32Smc, transVal) == 0x10 && offsetof(V32Smc, payloadBits) == 0x14, "V32Smc members 4 - 5");

/** @brief Circular output descriptor between the encoder and the pulse shaper. */
typedef struct V32Circ
{
  unsigned int iBuf;    ///< I sample ring (address slot)
  unsigned int qBuf;    ///< Q sample ring (address slot)
  unsigned int codeBuf; ///< code ring (address slot)
  short writeIdx;       ///< write index
  short readIdx;        ///< read index
  union
  {
    unsigned int ringLen; ///< ring length in entries
    short ringLen16;      ///< low half of `ringLen` (little endian): the engine stores the ring length as a 16-bit value
  };
} V32Circ;

_Static_assert(offsetof(V32Circ, iBuf) == 0x0 && offsetof(V32Circ, qBuf) == 0x4 && offsetof(V32Circ, codeBuf) == 0x8 && offsetof(V32Circ, writeIdx) == 0xc,
               "V32Circ members 0 - 3");
_Static_assert(offsetof(V32Circ, readIdx) == 0xe && offsetof(V32Circ, ringLen) == 0x10, "V32Circ members 4 - 5");

/** @brief The top-level V.32 object ("wrapper"): configuration, role and the pointers to the other two blocks. */
typedef struct V32Wrap
{
  short protoSel; ///< role selector: 0 originate, 1 answer, other = local loop
  short txRate;   ///< transmit rate in bit/s
  short rxRate;   ///< receive rate in bit/s
  unsigned char unk0006_[2];
  int timeoutWord;    ///< handshake timeout (ms)
  int d00c;           ///< not individually identified
  unsigned char b010; ///< not individually identified
  unsigned char b011; ///< not individually identified
  unsigned char unk0012_[2];
  unsigned short lookAhead; ///< physical delay look-ahead handed to the echo canceller
  short w016;               ///< not individually identified
  short w018;               ///< not individually identified
  unsigned char unk001a_[2];
  int highRate;       ///< non-zero when the configured rate is above 7200 bit/s; selects the trellis 9600 class
  short snrDropCount; ///< consecutive status calls with an SNR drop
  unsigned char unk0022_[2];
  int d024;          ///< not individually identified
  short discThresh;  ///< disconnect threshold (V32DiconnectThreshTable entry 3)
  short margin;      ///< configured margin
  short warmupCalls; ///< status calls since creation / the last local retrain (the SNR is held at 40 for the first 200)
  unsigned char unk002e_[2];
  unsigned short w030; ///< not individually identified
  unsigned short w032; ///< not individually identified
  unsigned int d034;   ///< not individually identified
  unsigned int d038;   ///< not individually identified
  unsigned int d03c;   ///< not individually identified
  unsigned int d040;   ///< not individually identified
  unsigned int d044;   ///< not individually identified
  unsigned short w048; ///< not individually identified
  unsigned char unk004a_[2];
  unsigned int d04c;   ///< not individually identified
  unsigned int d050;   ///< not individually identified
  unsigned short w054; ///< not individually identified
  unsigned char unk0056_[2];
  unsigned int d058;   ///< not individually identified
  unsigned int d05c;   ///< not individually identified
  unsigned short w060; ///< not individually identified
  unsigned char unk0062_[2];
  unsigned int hdxPtr; ///< handshake (half-duplex) object (address slot)
  unsigned int engPtr; ///< engine object (address slot)
} V32Wrap;

/** @brief Byte `i` (0 = low, 1 = high; little endian) of the 16-bit member `m` of the V32Wrap `w`: the status and flag words are used as two byte fields. */
#define V32W_BYTE(w, m, i) (((unsigned char*)&(w)->m)[i])

_Static_assert(offsetof(V32Wrap, protoSel) == 0x0 && offsetof(V32Wrap, txRate) == 0x2 && offsetof(V32Wrap, rxRate) == 0x4 &&
                   offsetof(V32Wrap, timeoutWord) == 0x8,
               "V32Wrap members 0 - 3");
_Static_assert(offsetof(V32Wrap, d00c) == 0xc && offsetof(V32Wrap, b010) == 0x10 && offsetof(V32Wrap, b011) == 0x11 && offsetof(V32Wrap, lookAhead) == 0x14,
               "V32Wrap members 4 - 7");
_Static_assert(offsetof(V32Wrap, w016) == 0x16 && offsetof(V32Wrap, w018) == 0x18 && offsetof(V32Wrap, highRate) == 0x1c &&
                   offsetof(V32Wrap, snrDropCount) == 0x20,
               "V32Wrap members 8 - 11");
_Static_assert(offsetof(V32Wrap, d024) == 0x24 && offsetof(V32Wrap, discThresh) == 0x28 && offsetof(V32Wrap, margin) == 0x2a &&
                   offsetof(V32Wrap, warmupCalls) == 0x2c,
               "V32Wrap members 12 - 15");
_Static_assert(offsetof(V32Wrap, w030) == 0x30 && offsetof(V32Wrap, w032) == 0x32 && offsetof(V32Wrap, d034) == 0x34 && offsetof(V32Wrap, d038) == 0x38,
               "V32Wrap members 16 - 19");
_Static_assert(offsetof(V32Wrap, d03c) == 0x3c && offsetof(V32Wrap, d040) == 0x40 && offsetof(V32Wrap, d044) == 0x44 && offsetof(V32Wrap, w048) == 0x48,
               "V32Wrap members 20 - 23");
_Static_assert(offsetof(V32Wrap, d04c) == 0x4c && offsetof(V32Wrap, d050) == 0x50 && offsetof(V32Wrap, w054) == 0x54 && offsetof(V32Wrap, d058) == 0x58,
               "V32Wrap members 24 - 27");
_Static_assert(offsetof(V32Wrap, d05c) == 0x5c && offsetof(V32Wrap, w060) == 0x60 && offsetof(V32Wrap, hdxPtr) == 0x64 && offsetof(V32Wrap, engPtr) == 0x68,
               "V32Wrap members 28 - 31");
_Static_assert(sizeof(V32Wrap) == 0x6c, "V32Wrap size");

/** @brief Handshake (half-duplex) object: state, timers, tone objects and the transmit / receive handlers. */
typedef struct V32Hdx
{
  unsigned char unk0000_[44];
  unsigned int toneA;    ///< first tone generator (address slot)
  unsigned int toneB;    ///< second tone generator (address slot)
  unsigned int toneC;    ///< third tone generator (address slot)
  unsigned int mtd;      ///< tone detector (address slot)
  unsigned short reg[5]; ///< handshake register file (LoadReg / StoreReg)
  short w046;            ///< not individually identified
  unsigned short w048;   ///< not individually identified
  unsigned short w04a;   ///< not individually identified
  unsigned short w04c;   ///< not individually identified
  short w04e;            ///< not individually identified
  unsigned short w050;   ///< not individually identified
  unsigned short w052;   ///< not individually identified
  unsigned short w054;   ///< not individually identified
  unsigned char unk0056_[2];
  unsigned int d058;       ///< not individually identified
  unsigned int d05c;       ///< not individually identified
  unsigned int d060;       ///< not individually identified
  unsigned int d064;       ///< not individually identified
  unsigned int d068;       ///< not individually identified
  unsigned int txHandler;  ///< transmit handler (address slot)
  unsigned int rxHandler;  ///< receive handler (address slot)
  unsigned short subState; ///< handshake state (V32State)
  short role;              ///< protocol role (V32Role)
  unsigned int txBudget;   ///< transmit samples left in the current state
  unsigned int rxElapsed;  ///< receive samples spent in the current state
  unsigned int stateLimit; ///< receive time limit of the current state
  short w084;              ///< not individually identified
  unsigned char unk0086_[2];
  unsigned int d088; ///< not individually identified
  unsigned char unk008c_[4];
  int phsRevLatch; ///< set once the phase reversal has been seen
  short w094;      ///< not individually identified
  short rtdResult; ///< measured round-trip delay
  short w098;      ///< not individually identified
  short w09a;      ///< not individually identified
  short w09c;      ///< not individually identified
  short frameLen;  ///< samples per receive block
  short w0a0;      ///< not individually identified
  unsigned char unk00a2_[2];
  unsigned int rxBitBuf; ///< buffer of demodulated bits (address slot)
  short w0a8;            ///< not individually identified
  short w0aa;            ///< not individually identified
  short toneMisses;      ///< consecutive blocks without the expected tone
  short w0ae;            ///< not individually identified
} V32Hdx;

_Static_assert(offsetof(V32Hdx, toneA) == 0x2c && offsetof(V32Hdx, toneB) == 0x30 && offsetof(V32Hdx, toneC) == 0x34 && offsetof(V32Hdx, mtd) == 0x38,
               "V32Hdx members 0 - 3");
_Static_assert(offsetof(V32Hdx, reg) == 0x3c && offsetof(V32Hdx, w046) == 0x46 && offsetof(V32Hdx, w048) == 0x48 && offsetof(V32Hdx, w04a) == 0x4a,
               "V32Hdx members 4 - 7");
_Static_assert(offsetof(V32Hdx, w04c) == 0x4c && offsetof(V32Hdx, w04e) == 0x4e && offsetof(V32Hdx, w050) == 0x50 && offsetof(V32Hdx, w052) == 0x52,
               "V32Hdx members 8 - 11");
_Static_assert(offsetof(V32Hdx, w054) == 0x54 && offsetof(V32Hdx, d058) == 0x58 && offsetof(V32Hdx, d05c) == 0x5c && offsetof(V32Hdx, d060) == 0x60,
               "V32Hdx members 12 - 15");
_Static_assert(offsetof(V32Hdx, d064) == 0x64 && offsetof(V32Hdx, d068) == 0x68 && offsetof(V32Hdx, txHandler) == 0x6c && offsetof(V32Hdx, rxHandler) == 0x70,
               "V32Hdx members 16 - 19");
_Static_assert(offsetof(V32Hdx, subState) == 0x74 && offsetof(V32Hdx, role) == 0x76 && offsetof(V32Hdx, txBudget) == 0x78 &&
                   offsetof(V32Hdx, rxElapsed) == 0x7c,
               "V32Hdx members 20 - 23");
_Static_assert(offsetof(V32Hdx, stateLimit) == 0x80 && offsetof(V32Hdx, w084) == 0x84 && offsetof(V32Hdx, d088) == 0x88 &&
                   offsetof(V32Hdx, phsRevLatch) == 0x90,
               "V32Hdx members 24 - 27");
_Static_assert(offsetof(V32Hdx, w094) == 0x94 && offsetof(V32Hdx, rtdResult) == 0x96 && offsetof(V32Hdx, w098) == 0x98 && offsetof(V32Hdx, w09a) == 0x9a,
               "V32Hdx members 28 - 31");
_Static_assert(offsetof(V32Hdx, w09c) == 0x9c && offsetof(V32Hdx, frameLen) == 0x9e && offsetof(V32Hdx, w0a0) == 0xa0 && offsetof(V32Hdx, rxBitBuf) == 0xa4,
               "V32Hdx members 32 - 35");
_Static_assert(offsetof(V32Hdx, w0a8) == 0xa8 && offsetof(V32Hdx, w0aa) == 0xaa && offsetof(V32Hdx, toneMisses) == 0xac && offsetof(V32Hdx, w0ae) == 0xae,
               "V32Hdx members 36 - 39");
_Static_assert(sizeof(V32Hdx) == 0xb0, "V32Hdx size");

/** @brief The engine image: all DSP state in one block, with the sub-objects embedded at their fixed offsets. */
typedef struct V32Eng
{
  unsigned int opt[10]; ///< option words: 0 .. 6 enable the stages of the receive chain, 7 .. 9 are the three option bits of V32Wrap::b010
  short txRateClass;    ///< transmit rate class being negotiated (V32RateClass)
  short rxRateClass;    ///< receive rate class being negotiated (V32RateClass)
  short txConnClass;    ///< connected transmit rate class (V32RateClass)
  short rxConnClass;    ///< connected receive rate class (V32RateClass)
  V32Sdm sdm;           ///< transmit scrambler
  V32Smc smc;           ///< symbol mapper / trellis encoder
  unsigned char unk005e_[2];
  V32Pps pps;        ///< pulse shaper
  unsigned int d098; ///< not individually identified
  unsigned int d09c; ///< not individually identified
  unsigned int d0a0; ///< not individually identified
  short smcEncSel;   ///< encoder function selected by the transmit mode (see SetTxModeV32)
  unsigned char unk00a6_[2];
  unsigned char b0a8; ///< not individually identified
  unsigned char unk00a9_[3];
  unsigned char b0ac; ///< not individually identified
  unsigned char unk00ad_[3];
  V32Circ circ;          ///< circular output descriptor between the encoder and the pulse shaper
  unsigned char mrf[28]; ///< embedded matched receive filter (FpmMrf; its size differs between hosts, see V32E_MRF)
  V32Ecc ecc;            ///< echo canceller
  V32Sre sre;            ///< symbol timing estimator
  unsigned char agc[44]; ///< embedded receive AGC (FpmAgc; its size differs between hosts, see V32E_AGC)
  V32Fse fse;            ///< fractionally spaced equaliser and carrier loop
  unsigned char unk501c_[4];
  V32Hist hist;         ///< decision history shared by the equaliser callbacks, with the Viterbi decoder state
  unsigned short w5098; ///< receive mode index (copy of the transmit rate index)
  unsigned short w509a; ///< second receive mode word (copy of the second transmit mode word)
  unsigned short w509c; ///< not individually identified
  unsigned short w509e; ///< not individually identified
  unsigned char b50a0;  ///< not individually identified
  unsigned char unk50a1_[5];
  unsigned short w50a6; ///< not individually identified
  unsigned short w50a8; ///< not individually identified
  unsigned short w50aa; ///< not individually identified
  unsigned char unk50ac_[4];
  V32Sdm rxSdm; ///< receive descrambler
  short w50c8;  ///< not individually identified
  unsigned char unk50ca_[2];
  unsigned int copyBufA; ///< sample copy buffer A (0x154 bytes, address slot)
  unsigned int copyBufB; ///< sample copy buffer B (0x154 bytes, address slot)
  unsigned short w50d4;  ///< not individually identified
  short w50d6;           ///< not individually identified
  short w50d8;           ///< not individually identified
  unsigned char unk50da_[2];
} V32Eng;
_Static_assert(offsetof(V32Eng, opt) == 0x0 && offsetof(V32Eng, txRateClass) == 0x28 && offsetof(V32Eng, rxRateClass) == 0x2a &&
                   offsetof(V32Eng, txConnClass) == 0x2c,
               "V32Eng members 0 - 3");
_Static_assert(offsetof(V32Eng, rxConnClass) == 0x2e && offsetof(V32Eng, sdm) == 0x30 && offsetof(V32Eng, smc) == 0x48 && offsetof(V32Eng, pps) == 0x60,
               "V32Eng members 4 - 7");
_Static_assert(offsetof(V32Eng, d098) == 0x98 && offsetof(V32Eng, d09c) == 0x9c && offsetof(V32Eng, d0a0) == 0xa0 && offsetof(V32Eng, smcEncSel) == 0xa4,
               "V32Eng members 8 - 11");
_Static_assert(offsetof(V32Eng, b0a8) == 0xa8 && offsetof(V32Eng, b0ac) == 0xac && offsetof(V32Eng, circ) == 0xb0 && offsetof(V32Eng, mrf) == 0xc4,
               "V32Eng members 12 - 15");
_Static_assert(offsetof(V32Eng, ecc) == 0xe0 && offsetof(V32Eng, sre) == 0x148 && offsetof(V32Eng, agc) == 0x1d8 && offsetof(V32Eng, fse) == 0x204,
               "V32Eng members 16 - 19");
_Static_assert(offsetof(V32Eng, hist) == 0x5020 && offsetof(V32Eng, w5098) == 0x5098 && offsetof(V32Eng, w509a) == 0x509a && offsetof(V32Eng, w509c) == 0x509c,
               "V32Eng members 20 - 23");
_Static_assert(offsetof(V32Eng, w509e) == 0x509e && offsetof(V32Eng, b50a0) == 0x50a0 && offsetof(V32Eng, w50a6) == 0x50a6 && offsetof(V32Eng, w50a8) == 0x50a8,
               "V32Eng members 24 - 27");
_Static_assert(offsetof(V32Eng, w50aa) == 0x50aa && offsetof(V32Eng, rxSdm) == 0x50b0 && offsetof(V32Eng, w50c8) == 0x50c8 &&
                   offsetof(V32Eng, copyBufA) == 0x50cc,
               "V32Eng members 28 - 31");
_Static_assert(offsetof(V32Eng, copyBufB) == 0x50d0 && offsetof(V32Eng, w50d4) == 0x50d4 && offsetof(V32Eng, w50d6) == 0x50d6 &&
                   offsetof(V32Eng, w50d8) == 0x50d8,
               "V32Eng members 32 - 35");
_Static_assert(sizeof(V32Eng) == 0x50dc, "V32Eng size");

#if UINTPTR_MAX == 0xffffffffu

#define V32_ENG_ALLOC V32_ENG_IMG
#define V32_WRAP_ALLOC V32_WRAP_IMG
#define V32E_PG(T, e, off) (*(T*)((char*)(e) + (off)))
#define V32E_PS(T, e, off, v) (*(T*)((char*)(e) + (off)) = (v))
/* indexed slot: entry `i` of the table of consecutive 4-byte slots that starts at `off` */
#define V32E_PGI(T, e, off, i) (*(T*)((char*)(e) + (off) + (i) * 4))
#define V32X_PG(T, sub, sbase, soff) (*(T*)((char*)(sub) + (soff)))
#define V32X_PGI(T, sub, sbase, soff, i) (*(T*)((char*)(sub) + (soff) + (i) * 4))
#define V32X_PSI(T, sub, sbase, soff, i, v) (*(T*)((char*)(sub) + (soff) + (i) * 4) = (v))
/* the same with the byte offset given as an EXPRESSION (`0x54 + i * 4`, `i * 4 + 0x54`); the 32-bit form evaluates it exactly as written */
#define V32X_PGE(T, sub, sbase, soff, i, expr) (*(T*)((char*)(sub) + (expr)))
#define V32X_PSE(T, sub, sbase, soff, i, expr, v) (*(T*)((char*)(sub) + (expr)) = (v))
#define V32X_PS(T, sub, sbase, soff, v) (*(T*)((char*)(sub) + (soff)) = (v))
#define V32W_PG(T, w, off) (*(T*)((char*)(w) + (off)))
#define V32W_PS(T, w, off, v) (*(T*)((char*)(w) + (off)) = (v))
/* the handshake object (0xb0 bytes, wrapper +0x64): the FPM_AGC at +0, the tone objects +0x2c +0x30 +0x34, the tone detector +0x38,
 * the Tx / Rx handlers +0x6c +0x70, the rx bit buffer +0xa4 */
#define V32_HDX_IMG 0xb0
#define V32_HDX_ALLOC V32_HDX_IMG
#define V32H_PG(T, h, off) (*(T*)((char*)(h) + (off)))
#define V32H_PS(T, h, off, v) (*(T*)((char*)(h) + (off)) = (v))

#else

#define V32_TAIL 0x50e0 /* image size rounded up to 8 */
#define V32_NSLOT 32
/* embedded FPM objects: behind the slots (sizeof(FpmMrf) == 0x28, sizeof(FpmAgc) == 0x38 on x86-64) */
#define V32_MRF_TAIL (V32_TAIL + 8 * V32_NSLOT)
#define V32_AGC_TAIL (V32_MRF_TAIL + 0x28)
#define V32_ENG_ALLOC (V32_AGC_TAIL + 0x38)
#define V32_WRAP_TAIL 0x70
#define V32_WRAP_ALLOC (V32_WRAP_TAIL + 8 * 12)

extern int v32_bad_pointer_slot_offset(void); /* never defined: an offset that is not an address slot fails to link */
/* the slot map: ENGINE offsets of the address slots (a sub-object's slot offsets are listed with the sub-object's base added) */
#define V32E_PIDX(off)                                                                                                                                         \
  ((off) == 0x90   ? 0                                                                                                                                         \
   : (off) == 0x94 ? 1                                                                                                                                         \
                   : /* PPS +0x30 / +0x34: the two history buffers */                                                                                          \
       (off) == 0x98 ? 2                                                                                                                                       \
   : (off) == 0x9c   ? 3                                                                                                                                       \
   : (off) == 0xa0   ? 4                                                                                                                                       \
                     : /* SMC encoder function table (dif / abs / tcm) */                                                                                      \
       (off) == 0xb0 ? 5                                                                                                                                       \
   : (off) == 0xb4   ? 6                                                                                                                                       \
   : (off) == 0xb8   ? 7                                                                                                                                       \
                     : /* circular descriptor: I / Q / code buffer pointers */                                                                                 \
       (off) == 0x118 ? 8                                                                                                                                      \
   : (off) == 0x120   ? 9                                                                                                                                      \
   : (off) == 0x124   ? 10                                                                                                                                     \
   : (off) == 0x12c   ? 11                                                                                                                                     \
   : (off) == 0x130   ? 12                                                                                                                                     \
                      : /* ECC +0x38 +0x40 +0x44 +0x4c +0x50 */                                                                                                \
       (off) == 0x134 ? 13                                                                                                                                     \
   : (off) == 0x138   ? 14                                                                                                                                     \
   : (off) == 0x13c   ? 15                                                                                                                                     \
                      : /* ECC +0x54 +0x58 +0x5c (3 tap buffers) */                                                                                            \
       (off) == 0x198 ? 16                                                                                                                                     \
   : (off) == 0x19c   ? 17                                                                                                                                     \
   : (off) == 0x1a0   ? 18                                                                                                                                     \
   : (off) == 0x1bc   ? 19                                                                                                                                     \
                      : /* SRE +0x50 +0x54 +0x58 +0x74 */                                                                                                      \
       (off) == 0x230 ? 20                                                                                                                                     \
   : (off) == 0x234   ? 21                                                                                                                                     \
                      : /* FSE +0x2c history object, +0x30 decision callback */                                                                                \
       (off) == 0x258 ? 22                                                                                                                                     \
   : (off) == 0x25c   ? 23                                                                                                                                     \
   : (off) == 0x264   ? 24                                                                                                                                     \
   : (off) == 0x268   ? 25                                                                                                                                     \
   : (off) == 0x26c   ? 26                                                                                                                                     \
                      : /* FSE +0x54 +0x58 +0x60 +0x64 +0x68 */                                                                                                \
       (off) == 0x5038 ? 27                                                                                                                                    \
                       : /* VTB scratch (history) buffer */                                                                                                    \
       (off) == 0x50cc ? 28                                                                                                                                    \
   : (off) == 0x50d0   ? 29                                                                                                                                    \
                       : /* the two 0x154-byte sample copy buffers */                                                                                          \
       v32_bad_pointer_slot_offset())
#define V32E_SLOT(e, off) (*(void**)((char*)(e) + V32_TAIL + 8 * V32E_PIDX(off)))
#define V32E_PG(T, e, off) ((T)V32E_SLOT(e, off))
#define V32E_PS(T, e, off, v) (V32E_SLOT(e, off) = (void*)(v))
#define V32E_PGI(T, e, off, i) ((T)(*(void**)((char*)(e) + V32_TAIL + 8 * (V32E_PIDX(off) + (i)))))
#define V32X_PG(T, sub, sbase, soff) V32E_PG(T, (char*)(sub) - (sbase), (sbase) + (soff))
#define V32X_PS(T, sub, sbase, soff, v) V32E_PS(T, (char*)(sub) - (sbase), (sbase) + (soff), v)
#define V32X_PGI(T, sub, sbase, soff, i) V32E_PGI(T, (char*)(sub) - (sbase), (sbase) + (soff), i)
#define V32X_PSI(T, sub, sbase, soff, i, v) (*(void**)((char*)(sub) - (sbase) + V32_TAIL + 8 * (V32E_PIDX((sbase) + (soff)) + (i))) = (void*)(v))
#define V32X_PGE(T, sub, sbase, soff, i, expr) V32X_PGI(T, sub, sbase, soff, i)
#define V32X_PSE(T, sub, sbase, soff, i, expr, v) V32X_PSI(T, sub, sbase, soff, i, v)
#define V32W_PIDX(off)                                                                                                                                         \
  ((off) == 0x64   ? 0                                                                                                                                         \
   : (off) == 0x68 ? 1                                                                                                                                         \
   : (off) == 0x34 ? 2                                                                                                                                         \
   : (off) == 0x38 ? 3                                                                                                                                         \
   : (off) == 0x3c ? 4                                                                                                                                         \
   : (off) == 0x40 ? 5                                                                                                                                         \
   : (off) == 0x44 ? 6                                                                                                                                         \
   : (off) == 0x4c ? 7                                                                                                                                         \
   : (off) == 0x50 ? 8                                                                                                                                         \
   : (off) == 0x58 ? 9                                                                                                                                         \
   : (off) == 0x5c ? 10                                                                                                                                        \
                   : v32_bad_pointer_slot_offset())
#define V32W_SLOT(w, off) (*(void**)((char*)(w) + V32_WRAP_TAIL + 8 * V32W_PIDX(off)))
#define V32W_PG(T, w, off) ((T)V32W_SLOT(w, off))
#define V32W_PS(T, w, off, v) (V32W_SLOT(w, off) = (void*)(v))
#define V32_HDX_TAIL 0xb0
#define V32H_SLOT(h, off)                                                                                                                                      \
  (*(void**)((char*)(h) + V32_HDX_TAIL +                                                                                                                       \
             8 * ((off) == 0x2c   ? 0                                                                                                                          \
                  : (off) == 0x30 ? 1                                                                                                                          \
                  : (off) == 0x34 ? 2                                                                                                                          \
                  : (off) == 0x38 ? 3                                                                                                                          \
                  : (off) == 0x6c ? 4                                                                                                                          \
                  : (off) == 0x70 ? 5                                                                                                                          \
                  : (off) == 0xa4 ? 6                                                                                                                          \
                                  : v32_bad_pointer_slot_offset())))
#define V32H_PG(T, h, off) ((T)V32H_SLOT(h, off))
#define V32H_PS(T, h, off, v) (V32H_SLOT(h, off) = (void*)(v))
#define V32_HDX_AGC_TAIL (V32_HDX_TAIL + 8 * 8)
#define V32_HDX_ALLOC (V32_HDX_AGC_TAIL + 0x38)

#endif

/* offsets of the sub-objects inside the engine image */
#define V32_PPS_OFF 0x60
#define V32_ECC_OFF 0xe0
#define V32_SRE_OFF 0x148
#define V32_FSE_OFF 0x204
#define V32_VTB_OFF 0x5038 /* the Viterbi object (vtbSubObj of VTBv32_init / VTB_decoder) = history object + 0x18 */

/* the embedded sub-objects sit at the offsets the sub-object functions assume, and have the same size at both pointer widths (no member is a pointer) */
_Static_assert(offsetof(V32Eng, pps) == V32_PPS_OFF && offsetof(V32Eng, ecc) == V32_ECC_OFF && offsetof(V32Eng, sre) == V32_SRE_OFF &&
                   offsetof(V32Eng, fse) == V32_FSE_OFF && offsetof(V32Eng, hist.vtb) == V32_VTB_OFF,
               "V32_*_OFF are the offsets of the sub-object members of V32Eng");
_Static_assert(sizeof(V32Smc) == 0x16 && sizeof(V32Pps) == 0x38 && sizeof(V32Circ) == 0x14 && sizeof(V32Ecc) == 0x68 && sizeof(V32Sre) == 0x90 &&
                   sizeof(V32Fse) == 0x4e18 && sizeof(V32Hist) == 0x78 && sizeof(V32Vtb) == 0x38,
               "sizes of the embedded sub-objects");

/* the embedded FPM objects (fpm.h): on a 32-bit host the block inside the image (`e + 0xc4` / `e + 0x1d8`), on x86-64 the typed block in the tail.
 * Include fpm.h first. */
#ifdef DSPLIBS_PORT_FPM_OBJ_H
#if UINTPTR_MAX == 0xffffffffu
#define V32E_MRF(e) ((FpmMrf*)((char*)(e) + (int)offsetof(V32Eng, mrf)))
#define V32E_AGC(e) ((FpmAgc*)((char*)(e) + (int)offsetof(V32Eng, agc)))
/* V32E_AGCF(T, e, rawoff, member): a field of the embedded AGC object: on a 32-bit host the bytes at `rawoff` in the image, on x86-64 the member of the
 * typed block */
#define V32E_AGCF(T, e, rawoff, m) (*(T*)((char*)(e) + (rawoff)))
#define V32H_AGC(h) ((FpmAgc*)(h))
#else
#define V32E_MRF(e) ((FpmMrf*)((char*)(e) + V32_MRF_TAIL))
#define V32E_AGC(e) ((FpmAgc*)((char*)(e) + V32_AGC_TAIL))
#define V32E_AGCF(T, e, rawoff, m) (V32E_AGC(e)->m)
#define V32H_AGC(h) ((FpmAgc*)((char*)(h) + V32_HDX_AGC_TAIL))
_Static_assert(sizeof(FpmMrf) == 0x28 && sizeof(FpmAgc) == 0x38, "v32.h: the FPM objects' x86-64 sizes");
#endif
#endif

/* Functions of src/v32 that other modules call (the fax V.17 modem shares the V.32 trellis decoder and the equaliser tap update).  The other prototypes of
 * the V.32 files are in v32_internal.h. */
extern void VTB_decoder(char* vtbSubObj, short rxI, short rxQ, short* outBits);                    /* v32_datapath.c */
extern void FPM_lmsupd(short* taps, char* historyBase, short histIdx, short tapCount, short gain); /* v32_math.c */

#endif
