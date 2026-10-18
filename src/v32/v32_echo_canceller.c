/**
 * @file v32_echo_canceller.c
 * @brief V.32 / V.32bis echo canceller: FPM_ECC_init(), FPM_ECC_cancel() and FPM_ECC_free().
 *
 * The canceller removes the near and far echo of the transmitted signal from the received samples.  It is an adaptive FIR in the symbol domain: the
 * transmitted constellation codes are kept in a ring (`codeHist`); every third sample (one symbol is three samples) the code at the near and far read positions
 * is mapped to an (I, Q) point through the tables ECCv32_IMAP / ECCv32_QMAP and pushed into the near and far I / Q history rings.  For every sample the echo
 * estimate is the sum of the near and far dot products of the history rings with the current one of three rotating tap sets (the tap set of the sample phase);
 * it is subtracted from the sample in place.  The error sample then drives a sign-less LMS update of the taps while adaptation is enabled.
 * - FPM_ECC_init(): fills the object from a six-word configuration, allocates the buffers (or keeps the existing ones) and clears the adaptive state.
 * - FPM_ECC_cancel(): the per-block processing; also keeps smoothed input and error powers while the measurement flag is 0.
 * - FPM_ECC_free(): releases the eight buffers.
 * The object is the V32Ecc block at offset 0xe0 of the engine (V32Eng) and is passed as a `V32Ecc *` and addressed with member offsets (E16 / E32); its buffer pointers live in the
 * engine's pointer-slot tail on x86-64.
 *
 * @warning FPM_ECC_init() reads V32Ecc::w064 (look-ahead) and V32Ecc::w066 (ring slack) as inputs before it ever writes them: the caller seeds them first (the
 *          engine creator and SetAdaptEcV32() do).  The read positions and ring length it computes (codeRdNear, codeRdFar, codeRingLen) are overwritten by
 *          those callers (0, 0 and 0x30) right after the call.
 * @warning The ECCv32_IMAP / ECCv32_QMAP configuration words are handles of tables of sub-table pointers (src/v32/v32_tables.c); FPM_ECC_cancel() indexes
 * through them
 *          on every symbol tick, so a table whose sub-table pointers are NULL makes the canceller dereference NULL.
 */

#include <stdint.h>
#include "core/dsp_table_handles.h"
#include <stdio.h>
#include <stdlib.h>
#include "v32_internal.h"
#include "core/dsp_host.h"

/* the buffer-address slots of the object (codeHist, the four history rings, the three tap sets) live in the engine's pointer-slot tail on x86-64 (v32.h) */
/* the two buffer addresses FPM_ECC_init() holds while it clears a ring */
#define ECC_LOCALS char *bA, *bB;
#define ECC_PG(T, ecc, off) V32X_PG(T, ecc, V32_ECC_OFF, off)
#define ECC_PS(T, ecc, off, v) V32X_PS(T, ecc, V32_ECC_OFF, off, v)
#define ECC_PGI(T, ecc, off, i) V32X_PGI(T, ecc, V32_ECC_OFF, off, i)
#define ECC_PSI(T, ecc, off, i, v) V32X_PSI(T, ecc, V32_ECC_OFF, off, i, v)
#define ECC_PGE(T, ecc, off, i, expr) V32X_PGE(T, ecc, V32_ECC_OFF, off, i, expr)
#define ECC_PSE(T, ecc, off, i, expr, v) V32X_PSE(T, ecc, V32_ECC_OFF, off, i, expr, v)

/* may_alias: the object is accessed at the same byte offsets as int words (FPM_ECC_init() copies the configuration words with int stores, then reads them
 * back as
 * shorts) and as shorts.  With plain short / int lvalues the optimiser (strict aliasing) may hoist the short loads above the int stores; that sized the tap
 * buffers
 * from stale values and overran the heap.  Applied on x86-64 only; the 32-bit build keeps the plain types, where the same hazard exists in principle at -O2. */
#if UINTPTR_MAX == 0xffffffffu
#define E16(base, o) (*(short*)((char*)(base) + (o)))
#define E32(base, o) (*(int*)((char*)(base) + (o)))
#else
typedef short __attribute__((may_alias)) ecc_short_t;
typedef int __attribute__((may_alias)) ecc_int_t;
#define E16(base, o) (*(ecc_short_t*)((char*)(base) + (o)))
#define E32(base, o) (*(ecc_int_t*)((char*)(base) + (o)))
#endif
/* the object's members by name (the offset comes from V32Ecc, v32.h) */
#define ECC16(m) E16(eccObj, offsetof(V32Ecc, m))
#define ECC32(m) E32(eccObj, offsetof(V32Ecc, m))

/* IMAP / QMAP: the configuration words ECCv32_CFG_raw[2] / [3] hold CFGP(ECCv32_IMAP / QMAP) (a handle on x86-64, src/core/dsp_table_handles.h); the table is a
 * 6-slot array of sub-table pointers (8 bytes each on x86-64).  ECC_MAP_AT() reads entry `off` of sub-table `idx`. */
#if UINTPTR_MAX == 0xffffffffu
#define ECC_MAP_AT(T, h, idx, off) (*(T*)(intptr_t)(E32((intptr_t)(h), (idx) * 4) + (off) * 2)) /* the word is the address of the array of 4-byte slots */
#else
#define ECC_MAP_AT(T, h, idx, off) (((const T*)((const short* const*)cfgp_ptr(h))[idx])[off])
#endif

/**
 * @brief Initialise the echo canceller object from a configuration and clear its adaptive state.
 * @param eccObj    the V32Ecc object (engine offset 0xe0); V32Ecc::w064 and ::w066 must already be seeded (see the file header)
 * @param cfg       six words laid out as ECCv32_CFG_raw: [0] low half = the delay span, high half = near taps; [1] low half = far taps; [2] / [3] the IMAP /
 * QMAP
 *                  table handles; [4] the silence code the code ring is filled with; [5] stored in V32Ecc::d014.  Must not be NULL
 * @param allocFlag non-zero to allocate all eight buffers (two near rings and two far rings of `taps` shorts, the code ring, three tap sets of
 *                  2 * (near + far) shorts); 0 to keep the buffers the object already has and only clear them
 *
 * The code ring has delay span + w066 + w064 entries.  The near and far histories, the tap sets and the power measurements are cleared, adaptation is off
 * (near and far flags 0), the tap step size is 0x29, the tap phase 0 and the code ring is filled with `cfg[4]`.  SetAdaptEcV32() mode 0 re-initialises with
 * `allocFlag == 0`.
 * @warning The allocation results are not checked, and an existing set of buffers is not freed first: call FPM_ECC_free() before allocating again.
 */
void FPM_ECC_init(V32Ecc* eccObj, const int* cfg, int allocFlag)
{
  short fillCode, nearTaps, histBase, codeRingLen, seedA, farTaps;
  int tapPairLen, tmpA, tmpB, ringLen32;
  void* newBuf;
  ECC_LOCALS

  ECC32(d000) = cfg[0];
  ECC32(d004) = cfg[1];
  ECC32(iMapTab) = cfg[2];
  ECC32(qMapTab) = cfg[3];
  ECC32(fillCode) = cfg[4];

  seedA = ECC16(w064); /* an input: seeded by the caller */
  fillCode = ECC16(fillCode);
  farTaps = ECC16(d004);
  nearTaps = E16(eccObj, 2);
  histBase = ECC16(d000);
  ECC32(d014) = cfg[5];

  /* ring sizes: a tap set holds I and Q taps for the near and far echo */
  tapPairLen = (int)(short)(nearTaps + farTaps);
  codeRingLen = (short)(histBase + ECC16(w066) + seedA);
  ringLen32 = (int)codeRingLen;

  if (allocFlag != 0)
  {
    /* the four history rings, the code ring and the three tap sets */
    tmpA = 0;
    newBuf = sysdep_malloc((unsigned int)(unsigned short)(nearTaps * 2));
    ECC_PS(char*, eccObj, (int)offsetof(V32Ecc, nearIHist), (char*)(newBuf));
    newBuf = sysdep_malloc((unsigned int)(unsigned short)(E16(eccObj, 2) * 2));
    ECC_PS(char*, eccObj, (int)offsetof(V32Ecc, nearQHist), (char*)(newBuf));
    newBuf = sysdep_malloc((unsigned int)(unsigned short)(ECC16(d004) * 2));
    ECC_PS(char*, eccObj, (int)offsetof(V32Ecc, farIHist), (char*)(newBuf));
    newBuf = sysdep_malloc((unsigned int)(unsigned short)(ECC16(d004) * 2));
    ECC_PS(char*, eccObj, (int)offsetof(V32Ecc, farQHist), (char*)(newBuf));
    newBuf = sysdep_malloc((unsigned int)(unsigned short)(ringLen32 * 2));
    ECC_PS(char*, eccObj, (int)offsetof(V32Ecc, codeHist), (char*)(newBuf));
    do
    {
      newBuf = sysdep_malloc((unsigned int)(tapPairLen << 2));
      ECC_PSE(char*, eccObj, (int)offsetof(V32Ecc, tapSets), tmpA, 0x54 + tmpA * 4, (char*)(newBuf));
      farTaps = (short)tmpA + 1;
      tmpA = (int)farTaps;
    } while (farTaps < 3);
    nearTaps = E16(eccObj, 2);
    farTaps = ECC16(d004);
    histBase = ECC16(d000);
  }

  /* adaptive state: power measurements, positions, rings and step size */
  ECC16(w018) = 0;
  ECC16(w01a) = 0;
  ECC16(w01c) = 1;
  ECC16(w01e) = 0;
  tmpA = ringLen32 - seedA;
  ECC16(inPower) = 0;
  ECC16(errPower) = 0;
  ECC16(w024) = 0;
  ECC16(nearAdapt) = 0;
  E16(eccObj, 0x2a) = 0;
  ECC16(codeRdFar) = (short)((short)tmpA - histBase);
  ECC16(farAdapt) = 0;
  E16(eccObj, 0x2e) = 0;
  ECC16(codeRingLen) = codeRingLen;
  ECC16(nearPos) = 0;
  ECC16(nearRing) = nearTaps;
  ECC16(farPos) = 0;
  ECC16(farRing) = farTaps;
  ECC16(tapPhase) = 0;
  ECC16(codeRdNear) = (short)(tmpA % ringLen32);
  ECC16(stepSize) = 0x29;

  /* clear the near and far history rings, the tap sets and fill the code ring with the silence code */
  if (0 < nearTaps)
  {
    bA = ECC_PG(char*, eccObj, (int)offsetof(V32Ecc, nearIHist));
    bB = ECC_PG(char*, eccObj, (int)offsetof(V32Ecc, nearQHist));
    tmpB = 0;
    do
    {
      E16(bA, tmpB * 2) = 0;
      E16(bB, tmpB * 2) = 0;
      seedA = (short)tmpB + 1;
      tmpB = (int)seedA;
    } while (seedA < ECC16(nearRing));
    farTaps = ECC16(farRing);
  }
  ringLen32 = 0;
  if (0 < farTaps)
  {
    bA = ECC_PG(char*, eccObj, (int)offsetof(V32Ecc, farIHist));
    bB = ECC_PG(char*, eccObj, (int)offsetof(V32Ecc, farQHist));
    do
    {
      E16(bA, ringLen32 * 2) = 0;
      E16(bB, ringLen32 * 2) = 0;
      seedA = (short)ringLen32 + 1;
      ringLen32 = (int)seedA;
    } while (seedA < ECC16(farRing));
  }
  ringLen32 = 0;
  do
  {
    tmpA = 0;
    if (0 < tapPairLen * 2)
    {
      bA = ECC_PGE(char*, eccObj, (int)offsetof(V32Ecc, tapSets), ringLen32, ringLen32 * 4 + 0x54);
      do
      {
        E16(bA, tmpA * 2) = 0;
        tmpA = tmpA + 1;
      } while (tmpA < tapPairLen * 2);
    }
    seedA = (short)ringLen32 + 1;
    ringLen32 = (int)seedA;
  } while (seedA < 3);
  tapPairLen = 0;
  if (0 < codeRingLen)
  {
    bA = ECC_PG(char*, eccObj, (int)offsetof(V32Ecc, codeHist));
    do
    {
      E16(bA, tapPairLen * 2) = fillCode;
      seedA = (short)tapPairLen + 1;
      tapPairLen = (int)seedA;
    } while (seedA < codeRingLen);
  }
}

/**
 * @brief Free the eight buffers of the echo canceller (three tap sets, code ring, far and near I / Q rings).
 * @param eccObj  the V32Ecc object; the pointers are not cleared afterwards
 */
void FPM_ECC_free(V32Ecc* eccObj)
{
  short i;
  for (i = 2; i >= 0; i--)
  {
    sysdep_free(V32X_PGI(void*, eccObj, V32_ECC_OFF, (int)offsetof(V32Ecc, tapSets), i));
  }
  sysdep_free(ECC_PG(void*, eccObj, (int)offsetof(V32Ecc, codeHist)));
  sysdep_free(ECC_PG(void*, eccObj, (int)offsetof(V32Ecc, farQHist)));
  sysdep_free(ECC_PG(void*, eccObj, (int)offsetof(V32Ecc, farIHist)));
  sysdep_free(ECC_PG(void*, eccObj, (int)offsetof(V32Ecc, nearQHist)));
  sysdep_free(ECC_PG(void*, eccObj, (int)offsetof(V32Ecc, nearIHist)));
}

/**
 * @brief Cancel the echo in a block of received samples, in place, and adapt the taps.
 * @param eccObj       the V32Ecc object (engine offset 0xe0), initialised by FPM_ECC_init()
 * @param samples      received samples, `count` entries; each is replaced by the sample minus the echo estimate
 * @param count        number of samples
 * @return the number of symbol ticks (far-echo history pushes) in the block; the callers ignore it
 *
 * Per sample: (1) the estimate is the near FIR plus the far FIR over the I and Q history rings with the tap set of the current sample phase (products shifted
 * right by 3, the sums by 14); (2) it is subtracted and the result stored; (3) while V32Ecc::w024 is 0 the taps of an enabled echo path are updated by
 * `tap += ((hist * step + 0x10) >> 5) * err + 0x10000 >> 17`; (4) the phase advances, and every third sample the next transmitted code is looked up in the
 * IMAP / QMAP tables and pushed into the I / Q histories (Q negated), the near ring only when there are near taps.
 *
 * While V32Ecc::w018 is 0 the block's mean input power (before) and error power (after) are folded into inPower and errPower with a weight of 0x666 / 2^15
 * on the
 * old value.  The positions, the phase and the step size are stored back at the end.
 * @warning The near taps are skipped by the far update only when the near update runs: the far taps of the tap set are located after the near taps, but
 * `tapSet`
 *          is advanced past the near taps only inside the near-adaptation branch.  With the near flag clear and the far flag set the far update would write
 *          into
 *          the near taps.  SetAdaptEcV32() always sets and clears the two flags together, so this does not happen today.
 */
short FPM_ECC_cancel(V32Ecc* eccObj, short* samples, unsigned short count)
{
  /* nearTaps / farTaps: the lengths of the near and far echo FIRs.  nearIHist / nearQHist and farIHist / farQHist: the I / Q symbol histories (rings of
   * nearRing / farRing entries, newest at nearPos / farPos).  codeHist: the ring (codeRingLen entries) of transmitted constellation codes (IMAP / QMAP row
   * in the
   * high byte, column in the low byte), read at codeRdNear and codeRdFar.  tapSet: the current one of three rotating tap sets (one per sample of a symbol),
   * laid out [near I][near Q][far I][far Q].  accErr / accQ: the I / Q dot products (accErr later holds the error sample of the update); echoEst: the echo
   * estimate.  tmpA / tmpB / tmpC, histPtr, coefPtr and tapPtr are temporaries and walking pointers shared by the estimate loops and the tap update loops. */
  short stepSize, nearTaps, farTaps, nearRing, farRing, codeRingLen, tmpB, tmpC;
  unsigned short codeWord;
  char *nearIHist, *nearQHist, *farIHist, *farQHist, *codeHist;
  short tmpA, echoEst;
  int stepSize32, accErr, nearTaps32;
  short* histPtr;
  int accQ;
  short ticks;
  short farPos16;
  unsigned int farPos;
  short* coefPtr;
  unsigned int tapIdx;
  unsigned int remaining;
  short* tapSet;
  short farLast;
  int errEnergy;
  unsigned int codeRdFar;
  unsigned int codeRdNear;
  unsigned int nearPos;
  unsigned int phase;
  short* tapPtr;

  errEnergy = 0;
  ticks = 0;

  /* load the object state */
  phase = (unsigned int)(unsigned short)ECC16(tapPhase);
  stepSize = ECC16(stepSize);
  stepSize32 = (int)stepSize;
  nearTaps = E16(eccObj, 2);
  nearTaps32 = (int)nearTaps;
  farTaps = ECC16(d004);
  nearIHist = ECC_PG(char*, eccObj, (int)offsetof(V32Ecc, nearIHist));
  nearQHist = ECC_PG(char*, eccObj, (int)offsetof(V32Ecc, nearQHist));
  nearPos = (unsigned int)(unsigned short)ECC16(nearPos);
  nearRing = ECC16(nearRing);
  farIHist = ECC_PG(char*, eccObj, (int)offsetof(V32Ecc, farIHist));
  codeRdNear = (unsigned int)(unsigned short)ECC16(codeRdNear);
  farQHist = ECC_PG(char*, eccObj, (int)offsetof(V32Ecc, farQHist));
  farRing = ECC16(farRing);
  codeHist = ECC_PG(char*, eccObj, (int)offsetof(V32Ecc, codeHist));
  farPos = (unsigned int)(unsigned short)ECC16(farPos);
  codeRdFar = (unsigned int)(unsigned short)ECC16(codeRdFar);
  echoEst = ECC16(w018);
  codeRingLen = ECC16(codeRingLen);

  /* smoothed input power of the block, while the measurement flag is 0 */
  if (echoEst == 0)
  {
    accQ = 0;
    histPtr = samples;
    for (accErr = 0; accErr < (int)(unsigned int)count; accErr = (int)(short)((short)accErr + 1))
    {
      tmpA = *histPtr;
      histPtr = histPtr + 1;
      accQ = (int)(short)((short)accQ + (short)(((int)tmpA * (int)tmpA) >> 0xf));
    }
    ECC16(inPower) = (short)((short)accQ + (short)(((ECC16(inPower) - accQ) * 0x666) >> 0xf));
  }

  /* per sample: echo estimate, subtraction, tap update, history update */
  remaining = (unsigned int)count - 1;
  if ((short)remaining != -1)
  {
    farLast = farRing + -1;
    do
    {
      remaining = remaining & 0xffff;

      /* near echo estimate: I and Q dot products, newest history entry first, wrapping at the ring end */
      echoEst = 0;
      tapSet = ECC_PGI(short*, eccObj, (int)offsetof(V32Ecc, tapSets), phase);
      tapPtr = tapSet;
      if (nearTaps32 != 0)
      {
        accErr = 0;
        histPtr = (short*)(nearIHist + (int)nearPos * 2);
        tapIdx = nearPos;
        coefPtr = tapSet;
        echoEst = (short)nearPos;
        while (-1 < echoEst)
        {
          tmpA = *coefPtr;
          coefPtr = coefPtr + 1;
          farPos16 = *histPtr;
          histPtr = histPtr + -1;
          echoEst = (short)tapIdx + -1;
          tapIdx = (unsigned int)(unsigned short)echoEst;
          accErr = accErr + (((int)farPos16 * (int)tmpA) >> 3);
        }
        histPtr = histPtr + nearRing;
        tmpA = nearRing + -1;
        for (echoEst = tmpA; (short)nearPos < echoEst; echoEst = echoEst + -1)
        {
          farPos16 = *coefPtr;
          coefPtr = coefPtr + 1;
          tmpB = *histPtr;
          histPtr = histPtr + -1;
          accErr = accErr + (((int)tmpB * (int)farPos16) >> 3);
        }
        accQ = 0;
        histPtr = (short*)(nearQHist + (int)nearPos * 2);
        tapIdx = nearPos;
        coefPtr = tapSet + nearTaps32;
        echoEst = (short)nearPos;
        while (-1 < echoEst)
        {
          farPos16 = *coefPtr;
          coefPtr = coefPtr + 1;
          tmpB = *histPtr;
          histPtr = histPtr + -1;
          echoEst = (short)tapIdx + -1;
          tapIdx = (unsigned int)(unsigned short)echoEst;
          accQ = accQ + (((int)tmpB * (int)farPos16) >> 3);
        }
        histPtr = histPtr + nearRing;
        for (; (short)nearPos < tmpA; tmpA = tmpA + -1)
        {
          echoEst = *histPtr;
          histPtr = histPtr + -1;
          farPos16 = *coefPtr;
          coefPtr = coefPtr + 1;
          accQ = accQ + (((int)echoEst * (int)farPos16) >> 3);
        }
        echoEst = (short)(accQ >> 0xe) + (short)(accErr >> 0xe);
        tapPtr = tapSet + nearTaps32 + nearTaps32;
      }

      /* far echo estimate, the same way, with the taps after the near taps */
      accErr = 0;
      farPos16 = (short)farPos;
      histPtr = (short*)(farIHist + (int)farPos * 2);
      tapIdx = farPos;
      coefPtr = tapPtr;
      tmpA = farPos16;
      while (-1 < tmpA)
      {
        tmpA = *histPtr;
        histPtr = histPtr + -1;
        tmpB = *coefPtr;
        coefPtr = coefPtr + 1;
        accErr = accErr + (((int)tmpA * (int)tmpB) >> 3);
        tmpA = (short)tapIdx + -1;
        tapIdx = (unsigned int)(unsigned short)tmpA;
      }
      histPtr = histPtr + farRing;
      for (tmpA = farLast; farPos16 < tmpA; tmpA = tmpA + -1)
      {
        tmpB = *histPtr;
        histPtr = histPtr + -1;
        tmpC = *coefPtr;
        coefPtr = coefPtr + 1;
        accErr = accErr + (((int)tmpB * (int)tmpC) >> 3);
      }
      tapPtr = tapPtr + farTaps;
      accQ = 0;
      histPtr = (short*)(farQHist + (int)farPos * 2);
      tapIdx = farPos;
      tmpA = farPos16;
      while (-1 < tmpA)
      {
        tmpA = *histPtr;
        histPtr = histPtr + -1;
        tmpB = *tapPtr;
        tapPtr = tapPtr + 1;
        accQ = accQ + (((int)tmpA * (int)tmpB) >> 3);
        tmpA = (short)tapIdx + -1;
        tapIdx = (unsigned int)(unsigned short)tmpA;
      }
      histPtr = histPtr + farRing;
      for (tmpA = farLast; farPos16 < tmpA; tmpA = tmpA + -1)
      {
        tmpB = *histPtr;
        histPtr = histPtr + -1;
        tmpC = *tapPtr;
        tapPtr = tapPtr + 1;
        accQ = accQ + (((int)tmpB * (int)tmpC) >> 3);
      }

      /* subtract the estimate; the result is the error sample of the update */
      echoEst = (short)(*samples - ((short)(accQ >> 0xe) + (short)(accErr >> 0xe) + echoEst));
      *samples = echoEst;
      accErr = (int)echoEst;
      if (ECC16(w018) == 0)
      {
        errEnergy = (int)(short)((short)((accErr * accErr) >> 0xf) + (short)errEnergy);
      }

      /* LMS update of the taps of the enabled echo paths (near I, near Q, far I, far Q) */
      if (ECC16(w024) == 0)
      {
        tapIdx = nearPos;
        histPtr = tapSet;
        echoEst = (short)nearPos;
        if (ECC32(nearAdapt) == 1 && nearTaps != 0)
        {
          while (-1 < echoEst)
          {
            echoEst = (short)tapIdx + -1;
            *histPtr = (short)((short)((*(short*)(nearIHist + (int)tapIdx * 2) * stepSize32 + 0x10) >> 5) * accErr + 0x10000 >> 0x11) + *histPtr;
            tapIdx = (unsigned int)(unsigned short)echoEst;
            histPtr = histPtr + 1;
          }
          tmpA = nearRing + -1;
          for (echoEst = tmpA; (short)nearPos < echoEst; echoEst = echoEst + -1)
          {
            *histPtr = (short)((short)((*(short*)(nearIHist + (int)echoEst * 2) * stepSize32 + 0x10) >> 5) * accErr + 0x10000 >> 0x11) + *histPtr;
            histPtr = histPtr + 1;
          }
          tapIdx = nearPos;
          histPtr = tapSet + nearTaps32;
          echoEst = (short)nearPos;
          while (-1 < echoEst)
          {
            echoEst = (short)tapIdx + -1;
            *histPtr = (short)((short)((*(short*)(nearQHist + (int)tapIdx * 2) * stepSize32 + 0x10) >> 5) * accErr + 0x10000 >> 0x11) + *histPtr;
            tapIdx = (unsigned int)(unsigned short)echoEst;
            histPtr = histPtr + 1;
          }
          for (; (short)nearPos < tmpA; tmpA = tmpA + -1)
          {
            *histPtr = (short)((short)((*(short*)(nearQHist + (int)tmpA * 2) * stepSize32 + 0x10) >> 5) * accErr + 0x10000 >> 0x11) + *histPtr;
            histPtr = histPtr + 1;
          }
          tapSet = tapSet + nearTaps32 + nearTaps32;
        }
        tapIdx = farPos;
        histPtr = tapSet;
        echoEst = farPos16;
        if (ECC32(farAdapt) == 1)
        {
          while (tmpA = farLast, -1 < echoEst)
          {
            echoEst = (short)tapIdx + -1;
            *histPtr = (short)((short)((*(short*)(farIHist + (int)tapIdx * 2) * stepSize32 + 0x10) >> 5) * accErr + 0x10000 >> 0x11) + *histPtr;
            tapIdx = (unsigned int)(unsigned short)echoEst;
            histPtr = histPtr + 1;
          }
          for (; farPos16 < tmpA; tmpA = tmpA + -1)
          {
            *histPtr = (short)((short)((*(short*)(farIHist + (int)tmpA * 2) * stepSize32 + 0x10) >> 5) * accErr + 0x10000 >> 0x11) + *histPtr;
            histPtr = histPtr + 1;
          }
          tapIdx = farPos;
          histPtr = tapSet + farTaps;
          echoEst = farPos16;
          while (tmpA = farLast, -1 < echoEst)
          {
            echoEst = (short)tapIdx + -1;
            *histPtr = (short)((short)((*(short*)(farQHist + (int)tapIdx * 2) * stepSize32 + 0x10) >> 5) * accErr + 0x10000 >> 0x11) + *histPtr;
            tapIdx = (unsigned int)(unsigned short)echoEst;
            histPtr = histPtr + 1;
          }
          for (; farPos16 < tmpA; tmpA = tmpA + -1)
          {
            *histPtr = (short)((short)((*(short*)(farQHist + (int)tmpA * 2) * stepSize32 + 0x10) >> 5) * accErr + 0x10000 >> 0x11) + *histPtr;
            histPtr = histPtr + 1;
          }
        }
      }

      /* every third sample: push the next transmitted codes into the near and far histories */
      tapIdx = phase + 1;
      phase = tapIdx & 0xffff;
      if (2 < (unsigned short)tapIdx)
      {
        phase = 0;
        codeWord = *(unsigned short*)(codeHist + (int)codeRdNear * 2);
        accErr = ECC32(iMapTab);
        accQ = ECC32(qMapTab);
        farPos = (unsigned int)(codeWord >> 8);
        tapIdx = codeWord & 0xff;
        echoEst = ECC_MAP_AT(short, accQ, (int)farPos, (int)tapIdx);
        tmpA = (short)codeRdNear + 1;
        codeRdNear = -(unsigned int)(tmpA < codeRingLen) & (unsigned int)(int)tmpA;
        if (nearTaps32 != 0)
        {
          nearPos = -(unsigned int)((short)((short)nearPos + 1) < nearRing) & (unsigned int)(int)(short)((short)nearPos + 1);
          *(unsigned short*)(nearIHist + (int)nearPos * 2) = ECC_MAP_AT(unsigned short, accErr, (int)farPos, (int)tapIdx);
          *(short*)(nearQHist + (int)nearPos * 2) = -echoEst;
        }
        {
          unsigned int uVarQ = (unsigned int)*(unsigned short*)(codeHist + (int)codeRdFar * 2);
          int idxQ = (int)uVarQ >> 8;
          unsigned int offQ = uVarQ & 0xff;
          echoEst = ECC_MAP_AT(short, accQ, idxQ, (int)offQ);
          tmpA = (short)codeRdFar + 1;
          farPos = -(unsigned int)((short)(farPos16 + 1) < farRing) & (unsigned int)(int)(short)(farPos16 + 1);
          *(unsigned short*)(farIHist + (int)farPos * 2) = ECC_MAP_AT(unsigned short, accErr, idxQ, (int)offQ);
          *(short*)(farQHist + (int)farPos * 2) = -echoEst;
          ticks = ticks + 1;
          codeRdFar = -(unsigned int)(tmpA < codeRingLen) & (unsigned int)(int)tmpA;
        }
      }
      remaining = remaining - 1;
      samples = samples + 1;
    } while ((short)remaining != -1);
    echoEst = ECC16(w018);
  }

  /* smoothed error power, then store the state back */
  if (echoEst == 0)
  {
    ECC16(errPower) = (short)((short)(((ECC16(errPower) - errEnergy) * 0x666) >> 0xf) + (short)errEnergy);
  }
  ECC16(farPos) = (short)farPos;
  ECC16(stepSize) = stepSize;
  ECC16(tapPhase) = (short)phase;
  ECC16(codeRdNear) = (short)codeRdNear;
  ECC16(nearPos) = (short)nearPos;
  ECC16(codeRdFar) = (short)codeRdFar;
  return ticks;
}
