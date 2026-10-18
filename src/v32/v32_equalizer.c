/**
 * @file v32_equalizer.c
 * @brief V.32 / V.32bis receive equaliser with carrier recovery, and the per-rate symbol decision callbacks.
 *
 * The equaliser object (V32Fse) is a fractionally spaced complex FIR (I and Q tap vectors over a ring of received samples) followed by a carrier phase and
 * frequency loop.  The decision object (V32Hist, `histObj`) holds the last symbols, the running distance average that detects retrain and rate renegotiation
 * patterns, and the state of the differential decoder; the Viterbi decoder object (V32Vtb) sits inside it at offset 0x18.
 * - FPM_FSE_init() / FPM_FSE_free(): configure the object from a 14-word configuration, allocate or release the five buffers (taps, sample ring, symbol
 * buffers).
 * - FPM_FSE_receive(): the per-block entry point.  It collects input samples, and each time a symbol is complete runs the equaliser, derotates the output by
 * the
 *   carrier phase, calls the current decision callback (`decisionFn`) for the data bits and the ideal point, updates the carrier loop and adapts the taps with
 *   FPM_lmsupd().  The decoded bits go to `outBits`, the equalised symbols to the symbol buffers that the callbacks read.
 * - FSE_decision_*: the callbacks.  Handshake: FSE_decision_AB installs _CD, which installs _trn, which installs _4pt (the 4800 bit/s slicer) by itself.  Data
 *   rates: _16pt (9600 bit/s), _32pt (9600 TCM), _16Tpt (7200), _64pt (12000), _128pt (14400); SetRxModeV32() installs them.  The four trellis-coded ones
 *   hand the symbol to VTB_decoder() (v32_datapath.c).  Each callback returns the decoded bits of the symbol and writes the ideal point's angle (in the
 *   phase unit of
 *   FPM_phasor()) and magnitude to its two pointer arguments.
 * - FSE_getdiag(): drains the diagnostic buffers of the object.
 *
 * @bug FSE_decision_16pt() reports a magnitude of 0 for the ideal point (see there), which FPM_FSE_receive() uses as the decision target of the equaliser
 * error.
 */

#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include "core/dsp_table_handles.h"
#include <stdlib.h>

#include "v32_tables.h"
#include "v32_viterbi_tables.h"
#include "v32_internal.h"
#include "core/dsp_host.h"
#include "fpm/fpm.h"

/** Samples counted since the last "Decoder Error" debug line; the line is printed (debug level above 1) every 0x1c20 samples, then the count restarts. */
unsigned int avg_err_show_0 = 0;

/** A decision callback as FPM_FSE_receive() calls it (the slot `decisionFn`): (equaliser object, angle in/out, magnitude in/out) -> decoded bits. */
typedef short (*fse_cb_t)(V32Fse*, short*, short*);

/* The callbacks are defined with differing result types (unsigned int, unsigned char, short) and called through fse_cb_t; the result is used as a 16-bit
 * value.  Their prototypes are in v32_internal.h. */

/**
 * @brief Configure the equaliser object and (re)allocate its buffers.
 * @param fse        the V32Fse object (the engine's equaliser block)
 * @param cfg        14 words laid out as FSEv32_CFG_raw: they are copied to the start of the object (word 0: low half = the block size in samples, high half
 *                   `w002` = samples per symbol; then the tap table handles, tap count, loop thresholds, gain tables ...).  Must not be NULL
 * @param allocFlag  0 frees the five buffers of a previous initialisation first ("Reallocating FPM_FSE buffers" is printed at debug level above 1); non-zero
 *                   allocates without freeing
 *
 * Allocates the I and Q tap vectors and the sample ring (`nTaps` shorts each) and the two symbol buffers ((block size / samples per symbol) * 2 + 4 bytes
 * each), copies the initial taps from the two tap tables, clears the sample ring, the loop state, the diagnostic counters and the small error history, and
 * sets `untilSym` to 1.
 * @warning The allocation results are not checked.  `w002` is the divisor of the symbol buffer size and must not be 0.
 */
void FPM_FSE_init(V32Fse* fse, const int* cfg, int allocFlag)
{
  char *histBuf, *qTaps, *iTaps;
  cfgp_int_t iInit, qInit; /* ICOFF / QCOFF table bases */
  short nextIdx;
  void* newBuf;
  int idx;
  short* dst;

  MEMBER_IDX(short, V32Fse, fse, d040) = 0;
  MEMBER_IDX_AT(short, V32Fse, fse, d040, 1) = 0;
  MEMBER_IDX(short, V32Fse, fse, d044) = 1;
  MEMBER_IDX_AT(short, V32Fse, fse, d044, 1) = 0;
  MEMBER_IDX(short, V32Fse, fse, d048) = 1;
  MEMBER_IDX_AT(short, V32Fse, fse, d048, 1) = 0;
  MEMBER_IDX(short, V32Fse, fse, d04c) = 1;
  MEMBER_IDX_AT(short, V32Fse, fse, d04c, 1) = 0;

  /* the configuration words become the first fields of the object */
  dst = (short*)fse;
  for (idx = 0xe; idx != 0; idx = idx - 1)
  {
    *(int*)dst = *cfg;
    cfg = cfg + 1;
    dst = dst + 2;
  }

  /* loop and measurement state */
  MEMBER_IDX(short, V32Fse, fse, freqAcc) = 0;
  MEMBER_IDX_AT(short, V32Fse, fse, freqAcc, 1) = 0;
  fse->w038 = 0;
  fse->loopState = 0;
  fse->phaseErrAvg = 0;
  fse->decErrAvg = 0;
  fse->inCount = 0;
  fse->outIdx = 0;
  fse->histPos = 0;
  MEMBER_IDX(short, V32Fse, fse, carrierAcc) = 0;
  MEMBER_IDX_AT(short, V32Fse, fse, carrierAcc, 1) = 0;
  fse->phaseAcc = 0;
  fse->w076 = 0;
  fse->settleCtr = 0;
  fse->untilSym = 1;

  if (allocFlag == 0)
  {
    if (1 < dsplibs_debug_level)
    {
      dsplibs_debug_printf("Reallocating FPM_FSE buffers");
    }
    sysdep_free(V32X_PG(void*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, symQBuf)));
    sysdep_free(V32X_PG(void*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, symIBuf)));
    sysdep_free(V32X_PG(void*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, rxHist)));
    sysdep_free(V32X_PG(void*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, qTaps)));
    sysdep_free(V32X_PG(void*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, iTaps)));
  }

  /* tap vectors and sample ring, then the symbol buffers */
  idx = (int)(short)(fse->nTaps * 2);
  newBuf = sysdep_malloc((unsigned int)idx);
  V32X_PS(void*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, iTaps), newBuf);
  newBuf = sysdep_malloc((unsigned int)idx);
  V32X_PS(void*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, qTaps), newBuf);
  newBuf = sysdep_malloc((unsigned int)idx);
  V32X_PS(void*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, rxHist), newBuf);
  idx = (int)(short)((*(short*)fse / fse->w002) * 2 + 4);
  newBuf = sysdep_malloc((unsigned int)idx);
  V32X_PS(void*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, symIBuf), newBuf);
  newBuf = sysdep_malloc((unsigned int)idx);
  V32X_PS(void*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, symQBuf), newBuf);

  /* initial taps from the tables, empty sample ring */
  idx = 0;
  if (0 < fse->nTaps)
  {
    histBuf = V32X_PG(char*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, rxHist));
    qTaps = V32X_PG(char*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, qTaps));
    iTaps = V32X_PG(char*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, iTaps));
    iInit = CFGP_WORD(fse->iInitTab);
    qInit = CFGP_WORD(fse->qInitTab);
    do
    {
      *(short*)(iTaps + idx * 2) = *(short*)(iInit + idx * 2);
      *(short*)(qTaps + idx * 2) = *(short*)(qInit + idx * 2);
      *(short*)(histBuf + idx * 2) = 0;
      nextIdx = (short)idx + 1;
      idx = (int)nextIdx;
    } while (nextIdx < fse->nTaps);
  }

  idx = 0;
  do
  {
    fse->unk078[idx] = 0;
    fse->unk078[idx + 4] = 0;
    nextIdx = (short)idx + 1;
    idx = (int)nextIdx;
  } while (nextIdx < 4);

  /* the diagnostic buffers (past 0x4e14 bytes into the object) start empty */
  MEMBER_IDX(short, V32Fse, fse, diagACount) = 0;
  MEMBER_IDX_AT(short, V32Fse, fse, diagACount, 1) = 0;
  fse->w4e10 = 0;
  fse->w4e12 = 0;
  MEMBER_IDX(short, V32Fse, fse, diagBCount) = 0;
  MEMBER_IDX_AT(short, V32Fse, fse, diagBCount, 1) = 0;
}

/**
 * @brief Free the five buffers of the equaliser (symbol buffers, sample ring, tap vectors).
 * @param fse     the V32Fse object; the pointers are not cleared afterwards
 */
void FPM_FSE_free(V32Fse* fse)
{
  sysdep_free(V32X_PG(void*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, symQBuf)));
  sysdep_free(V32X_PG(void*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, symIBuf)));
  sysdep_free(V32X_PG(void*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, rxHist)));
  sysdep_free(V32X_PG(void*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, qTaps)));
  sysdep_free(V32X_PG(void*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, iTaps)));
}

/**
 * @brief Drain one of the two diagnostic buffers of the equaliser.
 * @param fse       the V32Fse object
 * @param mode      0 = buffer A (filled by FPM_FSE_receive() with the equalised (I, Q) pairs, at most 0x1e0 pairs); 1 = buffer B (a second buffer that
 * nothing in
 *                  the library fills); other values return 0
 * @param outPairs  receives the (I, Q) pairs as pairs of ints
 * @param maxCount  capacity of `outPairs` in pairs
 * @return the number of pairs copied.  The buffer's count is reset to 0 in every case; if buffer A holds more than 0x1df pairs it is discarded and 0 is
 * returned
 */
int FSE_getdiag(V32Fse* fse, int mode, int* outPairs, int maxCount)
{
  int second;
  int diff;
  int count;
  int idx;

  if (mode == 0)
  {
    count = fse->diagACount;
    if (0x1df < count)
    {
      fse->diagACount = 0;
      return 0;
    }
    if (maxCount < count)
    {
      count = maxCount;
    }
    idx = 0;
    while (idx < count)
    {
      second = fse->diagA[idx * 2 + 1];
      *(int*)((char*)outPairs + idx * 8) = fse->diagA[idx * 2];
      *(int*)((char*)outPairs + 4 + idx * 8) = second;
      idx = idx + 1;
      diff = idx - count;
      (void)diff;
    }
    fse->diagACount = 0;
  }
  else
  {
    if (mode != 1)
    {
      return 0;
    }
    count = fse->diagBCount;
    if (maxCount < fse->diagBCount)
    {
      count = maxCount;
    }
    fse->diagBCount = 0;
    if (0 < count)
    {
      idx = 0;
      while (idx < count)
      {
        second = MEMBER_AS_ADDR_AT(int, V32Fse, (char*)fse, diagB[1], idx * 8);
        *(int*)((char*)outPairs + idx * 8) = MEMBER_AS_ADDR_AT(int, V32Fse, (char*)fse, diagB[0], idx * 8);
        *(int*)((char*)outPairs + 4 + idx * 8) = second;
        idx = idx + 1;
      }
    }
  }
  return count;
}

/**
 * @brief Run the equaliser and carrier loop over a block of input samples and decode the symbols that complete.
 * @param fse        the V32Fse object, initialised by FPM_FSE_init()
 * @param rxSamples  `count` input samples (the output of the symbol timing recovery)
 * @param outBits    receives one decoded value per completed symbol
 * @param count      number of input samples
 * @return the number of symbols decoded (the object's `outIdx`)
 *
 * The samples are pushed into the ring (`nTaps` entries) and counted down in `untilSym`; when a symbol is due:
 * 1. the I and Q tap vectors are applied to the ring (dot products shifted right by 15);
 * 2. the equalised point is converted to angle and magnitude (FPM_atan(), FPM_phasor()), the carrier phase (a table entry for the phase accumulator plus the
 *    loop's `carrierAcc`) is removed, and the derotated point is stored in the symbol buffers (Q13) and, while there is room, in the diagnostic buffer A;
 * 3. the decision callback returns the bits and the ideal point (angle, magnitude); the angle difference, wrapped to +-0x4000, is the phase error and is
 *    smoothed into `phaseErrAvg` (weights 0x799a / 0x666 over 2^15);
 * 4. when the loop is enabled (`d044`), the loop state is 0 until `settleCtr` has counted up to `settleCount` and then 1; afterwards the smoothed error
 *    magnitude sets it to 1 (at or above `errThreshHi`) or 2 (at or below `errThreshLo`).  The error drives the frequency accumulator and the carrier
 *    accumulator (wrapped at +-2^29) through the gain tables of that state; with `d048` a four-entry error history adds a prediction term;
 * 5. the decision error (the ideal point rotated back by the carrier phase, minus the equalised point) is smoothed into `decErrAvg`, and while the error
 *    average is positive and `d04c` is set (or `d040` is) both tap vectors are adapted with FPM_lmsupd();
 * 6. every 0x1c20 input samples "Decoder Error = %d" is printed at debug level above 1.
 *
 * The phase, the ring position and the samples still needed for the next symbol are stored back, also when the block ends before a symbol completes.
 */
short FPM_FSE_receive(V32Fse* fse, short* rxSamples, short* outBits, unsigned short count)
{
  short* qTaps;
  short* iTaps;
  char* rxHist;
  short phaseInc;
  short phaseWrap;
  short nTaps;
  short untilSym;
  int phaseAcc;
  short* symQOut;
  short* symIOut;
  /* {phase, cos, sin, increment} as FPM_phasor() takes them; only cos and sin are read back */
  short phasorBuf[4];
#define phsAngle phasorBuf[0]
#define phsCos phasorBuf[1]
#define phsSin phasorBuf[2]
#define phsInc phasorBuf[3]
  short rawAngle;
  short angleOut;
  short magBuf[3];
  int nTaps32;
  unsigned int histPos;
  short nextPos, phaseErr, mag, eqI;
  short decBits;
  unsigned short errAvg;
  int carrierPhase, tmpInt, acc;
  short *histPtr, *coefPtr;
  unsigned int tapIdx;
  short eqQ;

  qTaps = V32X_PG(short*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, qTaps));
  iTaps = V32X_PG(short*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, iTaps));
  rxHist = V32X_PG(char*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, rxHist));
  phaseAcc = (int)fse->phaseAcc;
  phaseInc = fse->phaseInc;
  phaseWrap = fse->phaseWrap;
  untilSym = fse->untilSym;
  nTaps = fse->nTaps;
  nTaps32 = (int)nTaps;
  histPos = (unsigned int)(unsigned short)fse->histPos;

  avg_err_show_0 = avg_err_show_0 + (unsigned int)count;
  fse->outIdx = 0;
  fse->inCount = count;
  phsInc = 0;
  nextPos = fse->phaseAcc;

  /* per symbol: collect the samples, equalise, derotate, decide */
  if (count != 0)
  {
    symQOut = V32X_PG(short*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, symQBuf));
    symIOut = V32X_PG(short*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, symIBuf));
    eqI = untilSym;
    do
    {
      untilSym = eqI;

      /* fewer samples left than the next symbol needs: keep them in the ring and advance the phase */
      if ((short)count < eqI)
      {
        if (0 < (short)count)
        {
          eqI = eqI - count;
          errAvg = count;
          while (errAvg = errAvg - 1, errAvg != USHRT_MAX)
          {
            untilSym = (short)histPos + 1;
            decBits = *rxSamples;
            rxSamples = rxSamples + 1;
            histPos = -(unsigned int)(untilSym < nTaps) & (unsigned int)(int)untilSym;
            *(short*)(rxHist + histPos * 2) = decBits;
          }
          phaseAcc = (int)(short)((short)phaseAcc + count * phaseInc);
          /* reduce the phase accumulator modulo phaseWrap */
          while ((short)phaseAcc >= phaseWrap)
          {
            phaseAcc = (int)(short)((short)phaseAcc - phaseWrap);
          }
        }
        fse->histPos = (short)histPos;
        fse->phaseAcc = (short)phaseAcc;
        fse->untilSym = eqI;
        return fse->outIdx;
      }

      /* take the samples up to the symbol boundary into the ring */
      while ((short)(untilSym + -1) != -1)
      {
        nextPos = (short)histPos + 1;
        decBits = *rxSamples;
        rxSamples = rxSamples + 1;
        histPos = -(unsigned int)(nextPos < nTaps) & (unsigned int)(int)nextPos;
        *(short*)(rxHist + histPos * 2) = decBits;
        untilSym = untilSym + -1;
      }
      nextPos = eqI * phaseInc + (short)phaseAcc;

      /* phase accumulator at the symbol, reduced modulo phaseWrap; the carrier phase is its table entry plus the loop's accumulator (Q15 rounded) */
      phaseAcc = (int)nextPos;
      while ((short)phaseAcc >= phaseWrap)
      {
        phaseAcc = (int)(short)((short)phaseAcc - phaseWrap);
      }
      untilSym = fse->w002;
      carrierPhase = (int)(short)(*(short*)(CFGP_WORD(fse->carrierTab) + phaseAcc * 2) + (short)((fse->carrierAcc + 0x4000) >> 0xf));
      count = count - eqI;

      /* equalised I and Q: tap vectors against the ring, newest sample first, wrapping at the ring end */
      histPtr = (short*)(rxHist + (int)histPos * 2);
      acc = 0;
      coefPtr = iTaps;
      for (tapIdx = histPos; -1 < (int)tapIdx; tapIdx = tapIdx - 1)
      {
        eqI = *histPtr;
        histPtr = histPtr + -1;
        phaseErr = *coefPtr;
        coefPtr = coefPtr + 1;
        acc = acc + (int)eqI * (int)phaseErr;
      }
      histPtr = histPtr + nTaps32;
      for (tmpInt = nTaps32 + -1; (int)histPos < tmpInt; tmpInt = tmpInt + -1)
      {
        eqI = *histPtr;
        histPtr = histPtr + -1;
        phaseErr = *coefPtr;
        coefPtr = coefPtr + 1;
        acc = acc + (int)eqI * (int)phaseErr;
      }
      eqI = (short)(acc >> 0xf);
      acc = 0;
      histPtr = (short*)(rxHist + (int)histPos * 2);
      coefPtr = qTaps;
      for (tapIdx = histPos; -1 < (int)tapIdx; tapIdx = tapIdx - 1)
      {
        phaseErr = *histPtr;
        histPtr = histPtr + -1;
        mag = *coefPtr;
        coefPtr = coefPtr + 1;
        acc = acc + (int)phaseErr * (int)mag;
      }
      histPtr = histPtr + nTaps32;
      for (tmpInt = nTaps32 + -1; (int)histPos < tmpInt; tmpInt = tmpInt + -1)
      {
        phaseErr = *histPtr;
        histPtr = histPtr + -1;
        mag = *coefPtr;
        coefPtr = coefPtr + 1;
        acc = acc + (int)phaseErr * (int)mag;
      }
      eqQ = (short)(acc >> 0xf);

      /* angle and magnitude of the equalised point */
      acc = (int)(short)(eqQ * 2);
      tmpInt = (int)(short)(eqI * 2);
      FPM_atan((short)acc, (short)tmpInt, (unsigned short*)&rawAngle);
      phsAngle = rawAngle;
      FPM_phasor(phasorBuf);
      mag = (short)(((int)phsCos * tmpInt + (int)phsSin * acc) >> 0xf);

      /* remove the carrier phase (and the predicted error term w076), wrapping the angle into 0 .. 0x3fff */
      phaseErr = (short)(((int)rawAngle - carrierPhase) >> 1) - fse->w076;
      if (phaseErr < 0)
      {
        phaseErr = phaseErr + 0x4000;
      }
      if (0x3fff < phaseErr)
      {
        phaseErr = phaseErr + -0x4000;
      }

      /* the derotated point (Q13) goes to the symbol buffers and the diagnostic buffer */
      phsInc = 0;
      phaseErr = phaseErr * 2;
      phsAngle = phaseErr;
      FPM_phasor(phasorBuf);
      *symIOut = (short)(((int)phsCos * (int)mag) >> 0xd);
      acc = fse->diagACount;
      *symQOut = (short)(((int)phsSin * (int)mag) >> 0xd);
      if (acc < 0x1e0)
      {
        MEMBER_AS_ADDR_AT(int, V32Fse, (char*)fse, diagA[0], acc * 8) = (int)*symIOut >> 1;
        MEMBER_AS_ADDR_AT(int, V32Fse, (char*)fse, diagA[1], fse->diagACount * 8) = (int)*symQOut >> 1;
        fse->diagACount = fse->diagACount + 1;
      }
      else
      {
        fse->diagACount = 0;
      }

      /* decision: the callback returns the bits and replaces angleOut / magBuf[0] with the ideal point's angle and magnitude */
      angleOut = phaseErr;
      magBuf[0] = mag;
      decBits = ((fse_cb_t)V32X_PG(char*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, decisionFn)))(fse, &angleOut, magBuf);
      *outBits = decBits;
      fse->outIdx = fse->outIdx + 1;

      /* phase error against the ideal point, smoothed */
      phaseErr = phaseErr - angleOut;
      if (phaseErr < -0x4000)
      {
        phaseErr = phaseErr + -0x8000;
      }
      if (0x4000 < phaseErr)
      {
        phaseErr = phaseErr + -0x8000;
      }
      acc = (int)phaseErr;
      errAvg = (unsigned short)((short)((fse->phaseErrAvg * 0x799a) >> 0xf) + (short)((acc * 0x666) >> 0xf));
      fse->phaseErrAvg = errAvg;

      /* carrier loop: state from the settle counter and the error thresholds, then the accumulators */
      if (fse->d044 != 0)
      {
        mag = fse->settleCtr;
        if (fse->settleCount < mag)
        {
          mag = (short)((errAvg ^ (unsigned short)((short)errAvg >> 0xf)) - (unsigned short)((short)errAvg >> 0xf));
          if (fse->errThreshHi <= mag)
          {
            fse->loopState = 1;
          }
          if (mag <= fse->errThreshLo)
          {
            fse->loopState = 2;
          }
        }
        else
        {
          fse->settleCtr = mag + 1;
          if (mag < fse->settleCount)
          {
            fse->loopState = 0;
          }
          else
          {
            fse->loopState = 1;
          }
        }
        mag = *(short*)(CFGP_WORD(fse->propTab) + fse->loopState * 2);
        tmpInt = fse->freqAcc;
        fse->freqAcc = *(short*)(CFGP_WORD(fse->intTab) + fse->loopState * 2) * acc + tmpInt;
        tmpInt = mag * acc + fse->carrierAcc + tmpInt;
        if (tmpInt < -0x20000000)
        {
          tmpInt = tmpInt + 0x40000000;
        }
        if (0x20000000 < tmpInt)
        {
          tmpInt = tmpInt + -0x40000000;
        }
        fse->carrierAcc = tmpInt;

        /* error history and the prediction term w076 */
        if (fse->d048 != 0)
        {
          short oldErrSum;
          int j;
          mag = 0;
          do
          {
            mag = mag + 1;
          } while (mag < 4);
          acc = 3;
          do
          {
            mag = (short)acc + -1;
            fse->unk078[acc + 4] = fse->unk078[acc + 3];
            acc = (int)mag;
          } while (0 < mag);
          oldErrSum = fse->w076;
          fse->w076 = 0;
          mag = 0;
          fse->unk078[4] = phaseErr + oldErrSum;
          for (j = 0; j < 4; j++)
          {
            mag = (short)(fse->unk078[j + 4] * fse->unk078[j] + mag);
          }
          fse->w076 = mag;
        }
      }

      /* decision error: the ideal point rotated back by the carrier phase against the equalised point */
      phsAngle = (short)((angleOut + carrierPhase) >> 1);
      if (phsAngle < 0)
      {
        phsAngle = phsAngle + 0x4000;
      }
      if (0x3fff < phsAngle)
      {
        phsAngle = phsAngle + -0x4000;
      }
      phsAngle = phsAngle * 2;
      FPM_phasor(phasorBuf);
      acc = (int)(short)((short)(((int)phsCos * (int)magBuf[0]) >> 0xe) + eqI * -2);
      carrierPhase = (int)(short)((short)(((int)magBuf[0] * (int)phsSin) >> 0xe) + eqQ * -2);
      tapIdx = (unsigned int)(short)(((fse->decErrAvg * 0x799a) >> 0xf) + (((short)((acc * acc + carrierPhase * carrierPhase) >> 0xb) * 0x666) >> 0xf));
      if (tapIdx < 0x8000)
      {
        fse->decErrAvg = (short)tapIdx;
      }
      else
      {
        fse->decErrAvg = 0x7fff;
      }

      /* tap adaptation with the step size w00e[w038] */
      if (((0 < fse->decErrAvg) && (fse->d04c != 0)) || (fse->d040 != 0))
      {
        tmpInt = (int)*(short*)(MEMBER_ADDR_A(V32Fse, (char*)fse, w00e) + *(short*)(MEMBER_ADDR_A(V32Fse, (char*)fse, w038)) * 2);
        FPM_lmsupd(iTaps, rxHist, (short)histPos, (short)nTaps32, (short)(((acc * tmpInt) + 0x800) >> 0xc));
        FPM_lmsupd(qTaps, rxHist, (short)histPos, (short)nTaps32, (short)(((carrierPhase * tmpInt) + 0x800) >> 0xc));
      }

      if (0x1c1f < avg_err_show_0)
      {
        if (1 < dsplibs_debug_level)
        {
          dsplibs_debug_printf("Decoder Error = %d\n", (int)fse->decErrAvg);
        }
        avg_err_show_0 = 0;
      }

      outBits = outBits + 1;
      symQOut = symQOut + 1;
      symIOut = symIOut + 1;
      eqI = untilSym;
    } while (count != 0);
  }

  /* store the state back */
  fse->histPos = (short)histPos;
  fse->phaseAcc = (short)phaseAcc;
  fse->untilSym = untilSym;
  return fse->outIdx;
#undef phsAngle
#undef phsCos
#undef phsSin
#undef phsInc
}

/*
 * The FSE_decision_* callbacks share one signature: (equaliser object, angle in/out, magnitude in/out).  On entry the pointers hold the angle and magnitude of
 * the derotated point (the callbacks of the handshake read the symbol buffers instead); on return they hold the ideal point's angle and magnitude.  The
 * decision history `hist` (V32Hist) keeps the two previous symbols (symI0 / symQ0, symI1 / symQ1), distAvg (w05e) = the running mean of the squared distance
 * between the current symbol and the one before the previous, and stateCount (w06e) = the number of consecutive calls with distAvg below 0x500, the period-2
 * repeat of a retrain or rate-signalling pattern.  Reaching 0x3c sets evFlags = 1 (RetrainDetectV32() tests bit 0); distAvg below dist / 8 with a count of
 * 0x15 .. 0x3b sets evFlags = 2 (RenegotiateDetectV32(), bit 1) and clears the count, as does distAvg >= 0x500.  Local names: rotI / rotQ = the point rotated
 * into one sector by cosRot / sinRot (the quadrant selects the rotation), bestIdx / bestDist / dist / cand / candEnd = the nearest-point search, oldI / oldQ /
 * tmpOld = the previous pair while it is shifted, outBits = the bits VTB_decoder() writes.  A local is reused for several roles; the name is its first or main
 * role.
 */

/**
 * @brief Handshake stage 1: tell the two alternating signal states apart and wait for the pattern to end.
 * @param fse         the V32Fse object
 * @param angleInOut  in: the derotated angle; out: DECv32_ANGL1200 of the decision
 * @param magInOut    out: 0x3299
 * @return the decoded 2-bit value of the decision, as for FSE_decision_4pt()
 *
 * The angle step since the previous call classifies the call: within 0x4000 it is decision 2 and updates the magnitude average magB (w060), otherwise it is
 * decision 1 and updates magA (w05e); the weights are 31/32 and 1/32.  When the squared distances between the current symbol and the older ones exceed 4/3 of
 * the summed squares of the two averages and at least 0x97 calls have passed, the history is cleared, the callback is replaced by FSE_decision_CD(),
 * `epochFlag` is set and the decision is 3.
 */
unsigned int FSE_decision_AB(V32Fse* fse, short* angleInOut, short* magInOut)
{
  short symQ;
  short i2;
  short i1;
  short q1;
  short* hist;
  unsigned int foundIdx;
  short q2;
  unsigned int zero;
  int magB;
  short symI;
  unsigned int mapIdx;
  int magA;
  int decision;

  mapIdx = 0;
  hist = V32X_PG(short*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, histObj));

  /* classify the call by the angle step since the previous call and average the magnitude into the matching state */
  symI = *angleInOut;
  symQ = ((V32Hist*)hist)->w050;
  magA = (int)((V32Hist*)hist)->w05e;
  ((V32Hist*)hist)->w050 = symI;
  symI = symI - symQ;
  magB = (int)((V32Hist*)hist)->w060;
  if (symI < 0)
  {
    symI = symI + -0x8000;
  }
  if (symI < 0x4001)
  {
    magB = (int)(short)((short)((magB * 0x1f) >> 5) + (*magInOut >> 5));
    decision = 2;
  }
  else
  {
    magA = (int)(short)((short)((magA * 0x1f) >> 5) + (*magInOut >> 5));
    decision = 1;
  }

  /* distance test on the last three symbols; promote to the next stage or shift the symbol history */
  symI = *(short*)(V32X_PG(char*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, symIBuf)) + fse->outIdx * 2);
  symQ = *(short*)(V32X_PG(char*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, symQBuf)) + fse->outIdx * 2);
  i2 = ((V32Hist*)hist)->symI1;
  q2 = ((V32Hist*)hist)->symQ1;
  i1 = ((V32Hist*)hist)->symI0;
  q1 = ((V32Hist*)hist)->symQ0;
  if (((int)(short)((magA * magA + magB * magB) >> 0xd) << 2) / 3 <
      (int)(short)((short)(((int)(short)(i2 - symI) * (int)(short)(i2 - symI) + (int)(short)(q2 - symQ) * (int)(short)(q2 - symQ)) >> 0xf) +
                   (short)(((int)(short)(((V32Hist*)hist)->symI2 - i1) * (int)(short)(((V32Hist*)hist)->symI2 - i1) +
                            (int)(short)(((V32Hist*)hist)->symQ2 - q1) * (int)(short)(((V32Hist*)hist)->symQ2 - q1)) >>
                           0xf)))
  {
    if (MEMBER_IDX(short, V32Hist, hist, stateCount) < 0x97)
    {
      ((V32Hist*)hist)->symQ1 = q1;
      ((V32Hist*)hist)->symI1 = i1;
      ((V32Hist*)hist)->symQ0 = symQ;
      ((V32Hist*)hist)->symI0 = symI;
      ((V32Hist*)hist)->symQ2 = q2;
      ((V32Hist*)hist)->symI2 = i2;
    }
    else
    {
      ((V32Hist*)hist)->symQ2 = 0;
      magA = 0;
      magB = 0;
      V32X_PS(fse_cb_t, fse, V32_FSE_OFF, (int)offsetof(V32Fse, decisionFn), FSE_decision_CD);
      ((V32Hist*)hist)->symI2 = 0;
      ((V32Hist*)hist)->symQ1 = 0;
      ((V32Hist*)hist)->symI1 = 0;
      ((V32Hist*)hist)->symQ0 = 0;
      ((V32Hist*)hist)->symI0 = 0;
      ((V32Hist*)hist)->w050 = 0;
      MEMBER_IDX(short, V32Hist, hist, epochFlag) = 1;
      MEMBER_IDX_AT(short, V32Hist, hist, epochFlag, 1) = 0;
      ((V32Hist*)hist)->stateCount = 0;
      decision = 3;
    }
  }
  else
  {
    ((V32Hist*)hist)->symQ1 = q1;
    ((V32Hist*)hist)->symI1 = i1;
    ((V32Hist*)hist)->symQ0 = symQ;
    ((V32Hist*)hist)->symI0 = symI;
    ((V32Hist*)hist)->symQ2 = q2;
    ((V32Hist*)hist)->symI2 = i2;
  }
  ((V32Hist*)hist)->stateCount = MEMBER_IDX(short, V32Hist, hist, stateCount) + 1;

  /* the ideal point of the decision and its differential decode */
  symI = DECv32_IMAP4[decision];
  symQ = DECv32_QMAP4[decision];
  *angleInOut = DECv32_ANGL1200[decision];
  *magInOut = 0x3299;
  i2 = hist[*hist + 4];
  zero = 0;
  {
    unsigned int scanIdx;
    for (scanIdx = 0; scanIdx < 0x10; scanIdx++)
    {
      if (SMCv32_IMAP16[scanIdx] == symI && SMCv32_QMAP16[scanIdx] == symQ)
      {
        foundIdx = scanIdx;
        goto found_ab;
      }
      foundIdx = mapIdx;
    }
  found_ab:;
  }
  mapIdx = foundIdx;
  hist[*hist + 4] = (short)(mapIdx & 0xc);
  symI = *(short*)((char*)&SMCv32_PMAP16 + (((mapIdx & 0xc) - (int)i2 & 0xf) >> 2) * 2);
  ((V32Hist*)hist)->w060 = (short)magB;
  ((V32Hist*)hist)->w05e = (short)magA;
  return (unsigned int)(int)symI >> 2 & 0xffff;
}

/**
 * @brief Handshake stage 2: a countdown of 15 calls between FSE_decision_AB() and FSE_decision_trn().
 * @param fse         the V32Fse object
 * @param angleInOut  unused
 * @param magInOut    out: 0x3299
 * @return 0 or 3, alternating with the count (0 on the last call)
 *
 * While counting, `d04c` is 0 (no tap adaptation).  After the 15th call the count and the descrambler seed (`lfsr`) are cleared, `d04c` is set, the callback
 * becomes FSE_decision_trn() and the stage reports 0.
 */
unsigned char FSE_decision_CD(V32Fse* fse, short* angleInOut, short* magInOut)
{
  unsigned short count;
  char* histAddr;
  short nextCount;

  (void)angleInOut;
  histAddr = V32X_PG(char*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, histObj));
  nextCount = MEMBER_AS_ADDR(short, V32Hist, histAddr, stateCount) + 1;
  if (0xe < nextCount)
  {
    ((V32Hist*)histAddr)->stateCount = 0;
    fse->w038 = 0;
    V32X_PS(fse_cb_t, fse, V32_FSE_OFF, (int)offsetof(V32Fse, decisionFn), FSE_decision_trn);
    fse->d04c = 1;
    count = ((V32Hist*)histAddr)->stateCount;
    ((V32Hist*)histAddr)->lfsr = 0;
    *magInOut = 0x3299;
    return (unsigned char)(~-((count & 1) == 0) & 3);
  }
  ((V32Hist*)histAddr)->stateCount = nextCount;
  fse->d04c = 0;
  count = ((V32Hist*)histAddr)->stateCount;
  *magInOut = 0x3299;
  return (unsigned char)(~-((count & 1) == 0) & 3);
}

/**
 * @brief Handshake stage 3: follow the training sequence, which is generated by a shift register, and switch to FSE_decision_4pt() when it ends.
 * @param fse         the V32Fse object
 * @param angleInOut  out: DECv32_ANGL1200 of the expected point
 * @param magInOut    out: 0x3299
 * @return the expected point (0 .. 3)
 *
 * The 32-bit register `lfsr` advances two bits per call (new bits = register >> lfsrShift ^ register >> 21 ^ 3).  For the first 0x100 calls the expected point
 * is 1 or 3 by the high new bit; up to call 0x500 it is DECv32_MAP_TRN[new bits]; then the count is cleared, `epochFlag` is set, the point is 0 and the
 * callback
 * becomes FSE_decision_4pt().
 */
unsigned int FSE_decision_trn(V32Fse* fse, short* angleInOut, short* magInOut)
{
  short nextCount;
  char* histAddr;
  unsigned int lfsrBits;
  unsigned int lfsr;

  histAddr = V32X_PG(char*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, histObj));
  lfsr = ((V32Hist*)histAddr)->lfsr;
  lfsrBits = (lfsr >> ((unsigned char)((V32Hist*)histAddr)->lfsrShift & 0x1f) ^ 3 ^ (lfsr >> 0x15)) & 3;
  nextCount = MEMBER_AS_ADDR(short, V32Hist, histAddr, stateCount) + 1;
  ((V32Hist*)histAddr)->lfsr = (lfsr << 2) | lfsrBits;
  if (nextCount < 0x100)
  {
    ((V32Hist*)histAddr)->stateCount = nextCount;
    lfsr = (-(unsigned int)((short)(lfsrBits >> 1) == 0) & 0xfffffffe) + 3;
  }
  else if (nextCount < 0x500)
  {
    ((V32Hist*)histAddr)->stateCount = nextCount;
    lfsr = (unsigned int)(unsigned short)DECv32_MAP_TRN[lfsrBits];
  }
  else
  {
    ((V32Hist*)histAddr)->stateCount = 0;
    lfsr = 0;
    V32X_PS(fse_cb_t, fse, V32_FSE_OFF, (int)offsetof(V32Fse, decisionFn), FSE_decision_4pt);
    ((V32Hist*)histAddr)->epochFlag = 1;
  }
  *angleInOut = DECv32_ANGL1200[lfsr];
  *magInOut = 0x3299;
  return lfsr & 0xffff;
}

/**
 * @brief Slicer of the 4-point constellation (V32MODE_4800, and the end state of the handshake chain).
 * @param fse         the V32Fse object
 * @param angleInOut  out: DECv32_ANGL1200 of the nearest point
 * @param magInOut    out: 0x3299
 * @return the decoded 2-bit value: the change of the quadrant against the previous symbol, through SMCv32_PMAP16, shifted right by 2
 *
 * Updates the symbol history and the retrain / renegotiate detector (see the comment above), picks the nearest of the four points (DECv32_IMAP4 / QMAP4) and
 * decodes the quadrant change differentially against the previous quadrant kept in the history (slot hist[hist[0] + 4]).
 */
unsigned int FSE_decision_4pt(V32Fse* fse, short* angleInOut, short* magInOut)
{
  short symQ;
  short tmpOld;
  short symI;
  int dQ;
  unsigned int foundIdx;
  int dI;
  int errI;
  int bestIdx;
  short* hist;
  unsigned int mapIdx;

  mapIdx = 0;
  symI = *(short*)(V32X_PG(char*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, symIBuf)) + fse->outIdx * 2);
  symQ = *(short*)(V32X_PG(char*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, symQBuf)) + fse->outIdx * 2);
  hist = V32X_PG(short*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, histObj));
  dI = (int)(short)(hist[0x2b] - symI);
  dQ = (int)(short)(hist[0x2c] - symQ);
  tmpOld = hist[0x2a];
  hist[0x2a] = symQ;
  symQ = hist[0x29];
  hist[0x2c] = tmpOld;
  hist[0x29] = symI;
  hist[0x2b] = symQ;
  symI = (short)((unsigned int)(dI * dI + dQ * dQ) >> 0x10);
  symQ = (short)((hist[0x2f] * 0xf) >> 4) + (symI >> 4);
  hist[0x2f] = symQ;
  if (symQ < 0x500)
  {
    tmpOld = hist[0x37] + 1;
    if (((symI >> 3 <= symQ) || (0x3b < tmpOld)) || (tmpOld < 0x15))
    {
      hist[0x37] = tmpOld;
      hist = V32X_PG(short*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, histObj));
      if (hist[0x37] == 0x3c)
      {
        hist[0x31] = 1;
      }
      goto decide_4pt;
    }
    hist[0x31] = 2;
  }
  hist[0x37] = 0;
decide_4pt:
  bestIdx = 0;
  dQ = 0;
  symI = 0x7fff;
  do
  {
    errI = (int)(short)(*(short*)(V32X_PG(char*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, symIBuf)) + fse->outIdx * 2) - DECv32_IMAP4[dQ]);
    dI = (int)(short)(*(short*)(V32X_PG(char*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, symQBuf)) + fse->outIdx * 2) - DECv32_QMAP4[dQ]);
    symQ = (short)((errI * errI) >> 0xf) + (short)((unsigned int)(dI * dI) >> 0x10);
    if (symQ < symI)
    {
      symI = symQ;
      bestIdx = dQ;
    }
    symQ = (short)dQ + 1;
    dQ = (int)symQ;
  } while (symQ < 4);
  symI = DECv32_IMAP4[bestIdx];
  symQ = DECv32_QMAP4[bestIdx];
  *angleInOut = DECv32_ANGL1200[bestIdx];
  foundIdx = 0;
  *magInOut = 0x3299;
  tmpOld = hist[*hist + 4];
  {
    unsigned int u;
    for (u = 0; u < 0x10; u++)
    {
      if (SMCv32_IMAP16[u] == symI && SMCv32_QMAP16[u] == symQ)
      {
        foundIdx = u;
        goto found_4pt;
      }
      foundIdx = mapIdx;
    }
  found_4pt:;
  }
  mapIdx = foundIdx;
  hist[*hist + 4] = (short)(mapIdx & 0xc);
  return (unsigned int)(int)(*(short*)((char*)&SMCv32_PMAP16 + (((mapIdx & 0xc) - (int)tmpOld & 0xf) >> 2) * 2)) >> 2 & 0xffff;
}

/**
 * @brief Slicer of the 16-point constellation (V32MODE_9600, no trellis).
 * @param fse         the V32Fse object
 * @param angleInOut  out: DECv32_ANGL9600 of the nearest point
 * @param magInOut    out: always 0 (see the bug note)
 * @return the decoded 4-bit value: the quadrant change through SMCv32_PMAP16 in the upper two bits, the low two bits of the point index below
 *
 * Same history and detector update as FSE_decision_4pt(); the nearest point is searched in the 16-point tables (DECv32_IMAP16 / QMAP16).
 * @bug The magnitude is read at index (|I| + |Q|) >> 1 of a table that is all zero here, so `*magInOut` is always 0.  FPM_FSE_receive() uses it as the
 *      magnitude of the ideal point in the decision error that adapts the taps, so the equaliser error is computed against a zero-magnitude point.  The
 *      index is the shift by 1 where FSE_decision_16Tpt() shifts by 13 (and indexes the 3-entry DECv32_MAG9600) and would be far outside a 3-entry table
 *      (docs/bugs.md; the entry there is labelled 4800 bit/s, but this callback is installed for V32MODE_9600).
 */
unsigned int FSE_decision_16pt(V32Fse* fse, short* angleInOut, short* magInOut)
{
  unsigned short bestI;
  unsigned short bestQ;
  unsigned int foundIdx;
  short symQ;
  short tmpOld;
  int dQ;
  unsigned int zero;
  int dI;
  short symI;
  int bestIdx;
  short* hist;
  unsigned int mapIdx;
  /* stand-in for the table the magnitude is read from: all zero, indexed modulo its size (see @bug) */
  static const short fse16pt_dead_placeholder[64] = {0};

  mapIdx = 0;
  symI = *(short*)(V32X_PG(char*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, symIBuf)) + fse->outIdx * 2);
  symQ = *(short*)(V32X_PG(char*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, symQBuf)) + fse->outIdx * 2);
  hist = V32X_PG(short*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, histObj));
  dI = (int)(short)(hist[0x2b] - symI);
  dQ = (int)(short)(hist[0x2c] - symQ);
  tmpOld = hist[0x2a];
  hist[0x2a] = symQ;
  symQ = hist[0x29];
  hist[0x2c] = tmpOld;
  hist[0x29] = symI;
  hist[0x2b] = symQ;
  symI = (short)((unsigned int)(dI * dI + dQ * dQ) >> 0x10);
  symQ = (short)((hist[0x2f] * 0xf) >> 4) + (symI >> 4);
  hist[0x2f] = symQ;
  if (symQ < 0x500)
  {
    tmpOld = hist[0x37] + 1;
    if (((symI >> 3 <= symQ) || (0x3b < tmpOld)) || (tmpOld < 0x15))
    {
      hist[0x37] = tmpOld;
      hist = V32X_PG(short*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, histObj));
      if (hist[0x37] == 0x3c)
      {
        hist[0x31] = 1;
      }
      goto decide_16pt;
    }
    hist[0x31] = 2;
  }
  hist[0x37] = 0;
decide_16pt:
  bestIdx = 0;
  dQ = 0;
  symI = 0x7fff;
  do
  {
    symQ = *(short*)(V32X_PG(char*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, symIBuf)) + fse->outIdx * 2) - DECv32_IMAP16[dQ];
    tmpOld = *(short*)(V32X_PG(char*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, symQBuf)) + fse->outIdx * 2) - DECv32_QMAP16[dQ];
    symQ = tmpOld * tmpOld + symQ * symQ;
    if (symQ < symI)
    {
      symI = symQ;
      bestIdx = dQ;
    }
    symQ = (short)dQ + 1;
    dQ = (int)symQ;
  } while (symQ < 0x10);
  bestI = DECv32_IMAP16[bestIdx];
  bestQ = DECv32_QMAP16[bestIdx];
  /* magnitude of the ideal point: index (|I| + |Q|) >> 1, see @bug */
  {
    short absSum = (short)((((short)bestI >> 0xf ^ bestI) - ((short)bestI >> 0xf)) + (((short)bestQ >> 0xf ^ bestQ) - ((short)bestQ >> 0xf)));
    int idx = (int)(absSum >> 1);
    *magInOut = fse16pt_dead_placeholder[(unsigned int)idx & 0x3f];
  }
  *angleInOut = *(short*)((char*)DECv32_ANGL9600 + bestIdx * 2);
  symI = hist[*hist + 4];
  {
    unsigned int u;
    zero = 0;
    for (u = 0; u < 0x10; u++)
    {
      if (SMCv32_IMAP16[u] == bestI && SMCv32_QMAP16[u] == bestQ)
      {
        foundIdx = u;
        goto found_16pt;
      }
      foundIdx = mapIdx;
    }
  found_16pt:;
  }
  mapIdx = foundIdx;
  hist[*hist + 4] = (short)(mapIdx & 0xc);
  return (unsigned int)(unsigned short)(*(unsigned short*)((char*)&SMCv32_PMAP16 + (((mapIdx & 0xc) - (int)symI & 0xf) >> 2) * 2)) | (mapIdx & 3);
}

/**
 * @brief Decision for the 32-point trellis-coded constellation (V32MODE_9600_TCM).
 * @param fse         the V32Fse object
 * @param angleInOut  out: DECv32_ANGL9600T of the nearest point
 * @param magInOut    out: DECv32_MAG9600T of the nearest point
 * @return the data bits of the symbol from VTB_decoder()
 *
 * Updates the symbol history and the detector, rotates the point into one sector (the quadrant selects DECv32_COS_ROT_ANGLE / SIN_ROT_ANGLE), finds the
 * nearest ideal point in the rotated sector by the Q distance (a corner case compares two candidate points by their full distance) and looks up its angle and
 * magnitude at index `point + 8 * quadrant`.  The data bits come from the Viterbi decoder, which is given the unrotated point.
 */
short FSE_decision_32pt(V32Fse* fse, short* angleInOut, short* magInOut)
{
  short oldQ;
  short oldI;
  unsigned short rawI;
  unsigned short rawQ;
  short rotQ;
  short bestDist;
  short rotI;
  short dist;
  int symI32;
  unsigned int quadrant;
  int cand;
  int cosRot;
  int sinRot;
  int symQ32;
  char* histAddr;
  int candEnd;
  int bestIdx;
  short outBits;

  rotI = *(short*)(V32X_PG(char*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, symIBuf)) + fse->outIdx * 2);
  rotQ = *(short*)(V32X_PG(char*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, symQBuf)) + fse->outIdx * 2);
  histAddr = V32X_PG(char*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, histObj));
  cand = (int)(short)(((V32Hist*)histAddr)->symI1 - rotI);
  symI32 = (int)(short)(((V32Hist*)histAddr)->symQ1 - rotQ);
  oldQ = ((V32Hist*)histAddr)->symQ0;
  ((V32Hist*)histAddr)->symQ0 = rotQ;
  oldI = ((V32Hist*)histAddr)->symI0;
  ((V32Hist*)histAddr)->symQ1 = oldQ;
  ((V32Hist*)histAddr)->symI0 = rotI;
  ((V32Hist*)histAddr)->symI1 = oldI;
  rotI = (short)((unsigned int)(cand * cand + symI32 * symI32) >> 0x10);
  rotQ = (short)((((V32Hist*)histAddr)->w05e * 0xf) >> 4) + (rotI >> 4);
  ((V32Hist*)histAddr)->w05e = rotQ;
  if (rotQ < 0x500)
  {
    bestDist = MEMBER_AS_ADDR(short, V32Hist, histAddr, stateCount) + 1;
    if (((rotI >> 3 <= rotQ) || (0x3b < bestDist)) || (bestDist < 0x15))
    {
      ((V32Hist*)histAddr)->stateCount = bestDist;
      histAddr = V32X_PG(char*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, histObj));
      if (MEMBER_AS_ADDR(short, V32Hist, histAddr, stateCount) == 0x3c)
      {
        ((V32Hist*)histAddr)->evFlags = 1;
      }
      goto decide_32pt;
    }
    ((V32Hist*)histAddr)->evFlags = 2;
  }
  ((V32Hist*)histAddr)->stateCount = 0;
decide_32pt:
  bestIdx = 0;
  candEnd = 3;
  rawI = *(unsigned short*)(V32X_PG(char*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, symIBuf)) + fse->outIdx * 2);
  symI32 = (int)(short)rawI;
  rawQ = *(unsigned short*)(V32X_PG(char*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, symQBuf)) + fse->outIdx * 2);
  symQ32 = (int)(short)rawQ;
  cand = 0;
  rotI = (short)((rawI ^ (short)rawI >> 0xf) - ((short)rawI >> 0xf));
  rotQ = (short)(((short)rawQ >> 0xf ^ rawQ) - ((short)rawQ >> 0xf));
  if ((short)rawQ < (short)rawI)
  {
    quadrant = (unsigned int)(rotQ < rotI) + 2;
  }
  else
  {
    quadrant = (unsigned int)(rotQ < rotI);
  }
  cosRot = (int)*(short*)((char*)&DECv32_COS_ROT_ANGLE + quadrant * 2);
  sinRot = (int)*(short*)((char*)&DECv32_SIN_ROT_ANGLE + quadrant * 2);
  rotI = (short)((symI32 * cosRot) >> 0xf) - (short)((symQ32 * sinRot) >> 0xf);
  rotQ = (short)((cosRot * symQ32) >> 0xf) + (short)((sinRot * symI32) >> 0xf);
  if (0x16a0 < rotI)
  {
    cand = 3;
    if (rotI < 0x2d42)
    {
      cand = 3;
    }
    else if (rotQ < 0x2d41)
    {
      cand = 6;
      candEnd = 2;
    }
  }
  bestDist = 0x7fff;
  candEnd = cand + candEnd;
  for (; cand < candEnd; cand = (int)(short)((short)cand + 1))
  {
    dist = *(short*)((char*)DECv32_ANA_QMAP + cand * 2);
    if ((int)rotQ - (int)dist < 0)
    {
      dist = dist - rotQ;
    }
    else
    {
      dist = rotQ - dist;
    }
    if (dist < bestDist)
    {
      bestDist = dist;
      bestIdx = cand;
    }
  }
  if ((short)bestIdx == 3 || (short)bestIdx == 6)
  {
    bestIdx = (int)(unsigned short)((short)((short)((int)(short)(*(short*)((char*)DECv32_ANA_QMAP + 12) - rotQ) *
                                                        (int)(short)(*(short*)((char*)DECv32_ANA_QMAP + 12) - rotQ) >>
                                                    0xf) +
                                            (short)((int)(short)(0x3891 - rotI) * (int)(short)(0x3891 - rotI) >> 0xf)) <
                                    (short)((short)((int)(short)(*(short*)((char*)DECv32_ANA_QMAP + 6) - rotQ) *
                                                        (int)(short)(*(short*)((char*)DECv32_ANA_QMAP + 6) - rotQ) >>
                                                    0xf) +
                                            (short)((int)(short)(0x21f1 - rotI) * (int)(short)(0x21f1 - rotI) >> 0xf))) *
                  3 +
              3;
  }
  cand = (int)(short)((short)bestIdx + (short)quadrant * 8);
  *magInOut = *(short*)((char*)DECv32_MAG9600T + cand * 2);
  *angleInOut = *(short*)((char*)DECv32_ANGL9600T + cand * 2);
  VTB_decoder(MEMBER_ADDR_A(V32Hist, histAddr, vtb), symI32, symQ32, &outBits);
  return outBits;
}

/**
 * @brief Decision for the 16-point trellis-coded constellation (V32MODE_7200).
 * @param fse         the V32Fse object
 * @param angleInOut  out: DECv32_ANGL9600 of the nearest point
 * @param magInOut    out: DECv32_MAG9600[(|I| + |Q|) >> 13 - 1] of the nearest point
 * @return the data bits of the symbol from VTB_decoder()
 *
 * Updates the symbol history and the detector, finds the nearest of the 16 points (DECv32_IMAP16 / QMAP16) and gives the received point to the Viterbi decoder.
 * The point sums |I| + |Q| of this constellation are 8192, 16384 or 24576, so the magnitude index is 0 .. 2 of the 3-entry table.
 */
short FSE_decision_16Tpt(V32Fse* fse, short* angleInOut, short* magInOut)
{
  short symI;
  short oldQ;
  short oldI;
  short symQ;
  short bestDist;
  int candIdx;
  short dist;
  int errQ;
  int errI;
  int bestIdx;
  char* histAddr;
  short outBits;

  symI = *(short*)(V32X_PG(char*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, symIBuf)) + fse->outIdx * 2);
  symQ = *(short*)(V32X_PG(char*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, symQBuf)) + fse->outIdx * 2);
  histAddr = V32X_PG(char*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, histObj));
  errQ = (int)(short)(((V32Hist*)histAddr)->symI1 - symI);
  candIdx = (int)(short)(((V32Hist*)histAddr)->symQ1 - symQ);
  oldQ = ((V32Hist*)histAddr)->symQ0;
  ((V32Hist*)histAddr)->symQ0 = symQ;
  oldI = ((V32Hist*)histAddr)->symI0;
  ((V32Hist*)histAddr)->symQ1 = oldQ;
  ((V32Hist*)histAddr)->symI0 = symI;
  ((V32Hist*)histAddr)->symI1 = oldI;
  symI = (short)((unsigned int)(errQ * errQ + candIdx * candIdx) >> 0x10);
  symQ = (short)((((V32Hist*)histAddr)->w05e * 0xf) >> 4) + (symI >> 4);
  ((V32Hist*)histAddr)->w05e = symQ;
  if (symQ < 0x500)
  {
    bestDist = MEMBER_AS_ADDR(short, V32Hist, histAddr, stateCount) + 1;
    if (((symI >> 3 <= symQ) || (0x3b < bestDist)) || (bestDist < 0x15))
    {
      ((V32Hist*)histAddr)->stateCount = bestDist;
      histAddr = V32X_PG(char*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, histObj));
      if (MEMBER_AS_ADDR(short, V32Hist, histAddr, stateCount) == 0x3c)
      {
        ((V32Hist*)histAddr)->evFlags = 1;
      }
      goto decide_16Tpt;
    }
    ((V32Hist*)histAddr)->evFlags = 2;
  }
  ((V32Hist*)histAddr)->stateCount = 0;
decide_16Tpt:
  candIdx = 0;
  symI = *(short*)(V32X_PG(char*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, symIBuf)) + fse->outIdx * 2);
  bestIdx = 0;
  symQ = *(short*)(V32X_PG(char*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, symQBuf)) + fse->outIdx * 2);
  bestDist = 0x7fff;
  do
  {
    errI = (int)(short)(symI - DECv32_IMAP16[candIdx]);
    errQ = (int)(short)(symQ - DECv32_QMAP16[candIdx]);
    dist = (short)((unsigned int)(errI * errI) >> 0x10) + (short)((unsigned int)(errQ * errQ) >> 0x10);
    if (dist < bestDist)
    {
      bestDist = dist;
      bestIdx = candIdx;
    }
    dist = (short)candIdx + 1;
    candIdx = (int)dist;
  } while (dist < 0x10);
  {
    short imapV = DECv32_IMAP16[bestIdx];
    short qmapV = DECv32_QMAP16[bestIdx];
    int idx = (int)((imapV < 0 ? -imapV : imapV) + (qmapV < 0 ? -qmapV : qmapV)) >> 0xd;
    /* idx is 1 .. 3 for the fixed constellation, so the unclamped index idx - 1 stays inside the table; the value is the target magnitude of the equaliser
     * error */
    *magInOut = DECv32_MAG9600[idx - 1];
  }
  *angleInOut = *(short*)((char*)DECv32_ANGL9600 + bestIdx * 2);
  VTB_decoder(MEMBER_ADDR_A(V32Hist, histAddr, vtb), (int)symI, (int)symQ, &outBits);
  return outBits;
}

/**
 * @brief Decision for the 64-point trellis-coded constellation (V32MODE_12000).
 * @param fse         the V32Fse object
 * @param angleInOut  out: DECv32_ANGL12000 of the nearest point
 * @param magInOut    out: DECv32_MAG12000 of the nearest point
 * @return the data bits of the symbol from VTB_decoder()
 *
 * Updates the symbol history and the detector (the renegotiation test is written in the other order than in the smaller callbacks, with the same result),
 * selects the group of four candidate points of the sector the point is in (thresholds at 0 and +-0x2000) and takes the nearest of them.
 */
short FSE_decision_64pt(V32Fse* fse, short* angleInOut, short* magInOut)
{
  short symI;
  short oldQ;
  short oldI;
  short symQ;
  short bestDist;
  short dist;
  int cand;
  int candEnd;
  char* histAddr;
  int bestIdx;
  short outBits;

  symI = *(short*)(V32X_PG(char*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, symIBuf)) + fse->outIdx * 2);
  symQ = *(short*)(V32X_PG(char*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, symQBuf)) + fse->outIdx * 2);
  histAddr = V32X_PG(char*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, histObj));
  candEnd = (int)(short)(((V32Hist*)histAddr)->symI1 - symI);
  cand = (int)(short)(((V32Hist*)histAddr)->symQ1 - symQ);
  oldQ = ((V32Hist*)histAddr)->symQ0;
  ((V32Hist*)histAddr)->symQ0 = symQ;
  oldI = ((V32Hist*)histAddr)->symI0;
  ((V32Hist*)histAddr)->symQ1 = oldQ;
  ((V32Hist*)histAddr)->symI0 = symI;
  ((V32Hist*)histAddr)->symI1 = oldI;
  symI = (short)((unsigned int)(candEnd * candEnd + cand * cand) >> 0x10);
  symQ = (short)((((V32Hist*)histAddr)->w05e * 0xf) >> 4) + (symI >> 4);
  ((V32Hist*)histAddr)->w05e = symQ;
  if (symQ < 0x500)
  {
    bestDist = MEMBER_AS_ADDR(short, V32Hist, histAddr, stateCount) + 1;
    if (((symQ < symI >> 3) && (bestDist < 0x3c)) && (0x14 < bestDist))
    {
      ((V32Hist*)histAddr)->evFlags = 2;
      goto reset_64pt;
    }
    ((V32Hist*)histAddr)->stateCount = bestDist;
    histAddr = V32X_PG(char*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, histObj));
    if (MEMBER_AS_ADDR(short, V32Hist, histAddr, stateCount) == 0x3c)
    {
      ((V32Hist*)histAddr)->evFlags = 1;
      symI = *(short*)(V32X_PG(char*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, symIBuf)) + fse->outIdx * 2);
      symQ = *(short*)(V32X_PG(char*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, symQBuf)) + fse->outIdx * 2);
      goto joined_64pt;
    }
  }
  else
  {
  reset_64pt:
    ((V32Hist*)histAddr)->stateCount = 0;
  }
  symI = *(short*)(V32X_PG(char*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, symIBuf)) + fse->outIdx * 2);
  symQ = *(short*)(V32X_PG(char*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, symQBuf)) + fse->outIdx * 2);
joined_64pt:
  if (symI < 1)
  {
    cand = 0;
    if (symQ < 1)
    {
      cand = 0x10;
      if (symI < -0x1fff)
      {
        if (symQ < -0x2000)
        {
          cand = 0x14;
        }
      }
      else
      {
        cand = 0x18;
        if (symQ < -0x2000)
        {
          cand = 0x1c;
        }
      }
    }
    else if (symI < -0x1fff)
    {
      if (symQ < 0x2000)
      {
        cand = 4;
      }
    }
    else
    {
      cand = 8;
      if (symQ < 0x2000)
      {
        cand = 0xc;
      }
    }
  }
  else
  {
    cand = 0x20;
    if (symQ < 1)
    {
      cand = 0x30;
      if (symI < 0x2001)
      {
        if (symQ < -0x2000)
        {
          cand = 0x34;
        }
      }
      else
      {
        cand = 0x38;
        if (symQ < -0x2000)
        {
          cand = 0x3c;
        }
      }
    }
    else if (symI < 0x2001)
    {
      if (symQ < 0x2000)
      {
        cand = 0x24;
      }
    }
    else
    {
      cand = 0x28;
      if (symQ < 0x2000)
      {
        cand = 0x2c;
      }
    }
  }
  bestIdx = 0;
  candEnd = cand + 4;
  bestDist = 0x7fff;
  do
  {
    dist = (short)(((int)(short)(symQ - DECv32_QMAP64[cand]) * (int)(short)(symQ - DECv32_QMAP64[cand])) >> 0xd) +
           (short)(((int)(short)(symI - DECv32_IMAP64[cand]) * (int)(short)(symI - DECv32_IMAP64[cand])) >> 0xd);
    if (dist < bestDist)
    {
      bestDist = dist;
      bestIdx = cand;
    }
    cand = (int)(short)((short)cand + 1);
  } while (cand < candEnd);
  oldQ = *(short*)((char*)DECv32_ANGL12000 + bestIdx * 2);
  *magInOut = *(short*)((char*)DECv32_MAG12000 + bestIdx * 2);
  *angleInOut = oldQ;
  VTB_decoder(MEMBER_ADDR_A(V32Hist, histAddr, vtb), (int)symI, (int)symQ, &outBits);
  return outBits;
}

/**
 * @brief Decision for the 128-point trellis-coded constellation (V32MODE_14400).
 * @param fse         the V32Fse object
 * @param angleInOut  out: DECv32_ANGL14400 of the nearest point
 * @param magInOut    out: DECv32_MAG14400 of the nearest point
 * @return the data bits of the symbol from VTB_decoder()
 *
 * Updates the symbol history and the detector, rotates the point into one sector (as FSE_decision_32pt()), picks the group of four candidate points of the
 * sector region by thresholds on the rotated point (two corner regions compare two candidates by the full distance) and takes the nearest one; the angle and
 * magnitude are read at index `point + 32 * quadrant`.  The data bits come from the Viterbi decoder, which is given the unrotated point.
 */
short FSE_decision_128pt(V32Fse* fse, short* angleInOut, short* magInOut)
{
  short oldQ;
  short oldI;
  unsigned short rawI;
  unsigned short rawQ;
  int cornerB;
  int cornerA;
  short rotQ;
  short bestDist;
  short rotI;
  short dist;
  int symI32;
  int symQ32;
  int cosRot;
  int sinRot;
  unsigned int quadrant;
  int cand;
  short errQ;
  int bestIdx;
  int regionBase;
  char* histAddr;
  short outBits[7];

  bestIdx = 0;
  rotI = *(short*)(V32X_PG(char*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, symIBuf)) + fse->outIdx * 2);
  rotQ = *(short*)(V32X_PG(char*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, symQBuf)) + fse->outIdx * 2);
  histAddr = V32X_PG(char*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, histObj));
  symQ32 = (int)(short)(((V32Hist*)histAddr)->symI1 - rotI);
  symI32 = (int)(short)(((V32Hist*)histAddr)->symQ1 - rotQ);
  oldQ = ((V32Hist*)histAddr)->symQ0;
  ((V32Hist*)histAddr)->symQ0 = rotQ;
  oldI = ((V32Hist*)histAddr)->symI0;
  ((V32Hist*)histAddr)->symQ1 = oldQ;
  ((V32Hist*)histAddr)->symI0 = rotI;
  ((V32Hist*)histAddr)->symI1 = oldI;
  rotI = (short)((unsigned int)(symQ32 * symQ32 + symI32 * symI32) >> 0x10);
  rotQ = (short)((((V32Hist*)histAddr)->w05e * 0xf) >> 4) + (rotI >> 4);
  ((V32Hist*)histAddr)->w05e = rotQ;
  if (rotQ < 0x500)
  {
    bestDist = MEMBER_AS_ADDR(short, V32Hist, histAddr, stateCount) + 1;
    if (((rotQ < rotI >> 3) && (bestDist < 0x3c)) && (0x14 < bestDist))
    {
      ((V32Hist*)histAddr)->evFlags = 2;
      goto reset_128pt;
    }
    ((V32Hist*)histAddr)->stateCount = bestDist;
    histAddr = V32X_PG(char*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, histObj));
    if (MEMBER_AS_ADDR(short, V32Hist, histAddr, stateCount) == 0x3c)
    {
      ((V32Hist*)histAddr)->evFlags = 1;
    }
  }
  else
  {
  reset_128pt:
    ((V32Hist*)histAddr)->stateCount = 0;
  }
  regionBase = 0;
  cornerA = 0;
  cornerB = 0;
  rawI = *(unsigned short*)(V32X_PG(char*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, symIBuf)) + fse->outIdx * 2);
  symI32 = (int)(short)rawI;
  rawQ = *(unsigned short*)(V32X_PG(char*, fse, V32_FSE_OFF, (int)offsetof(V32Fse, symQBuf)) + fse->outIdx * 2);
  symQ32 = (int)(short)rawQ;
  rotI = (short)((rawI ^ (short)rawI >> 0xf) - ((short)rawI >> 0xf));
  rotQ = (short)(((short)rawQ >> 0xf ^ rawQ) - ((short)rawQ >> 0xf));
  if ((short)rawQ < (short)rawI)
  {
    quadrant = (unsigned int)(rotQ < rotI) + 2;
  }
  else
  {
    quadrant = (unsigned int)(rotQ < rotI);
  }
  cosRot = (int)*(short*)((char*)&DECv32_COS_ROT_ANGLE + quadrant * 2);
  sinRot = (int)*(short*)((char*)&DECv32_SIN_ROT_ANGLE + quadrant * 2);
  rotI = (short)((symI32 * cosRot) >> 0xf) - (short)((symQ32 * sinRot) >> 0xf);
  rotQ = (short)((cosRot * symQ32) >> 0xf) + (short)((sinRot * symI32) >> 0xf);
  if (rotI < 0x16a1)
  {
    if (0x2d40 < rotQ)
      goto pick_128pt;
    cosRot = (unsigned int)(rotQ < 0x16a0) * 4 + 4;
  }
  else
  {
    regionBase = 0xc;
    if (rotI < 0x2d42)
    {
      if ((0x2d40 < rotQ) || (regionBase = 0x10, 0x169f < rotQ))
        goto pick_128pt;
      regionBase = 0x14;
      cosRot = regionBase;
    }
    else if (rotQ < 0x2d42)
    {
      cosRot = (unsigned int)(rotQ < 0x16a1) * 4 + 0x18;
    }
    else if (rotI < 0x3892)
    {
      cosRot = 0xe;
      if (rotQ < 0x3892)
      {
        cornerA = 0;
        cornerB = 1;
        goto pick_128pt;
      }
    }
    else
    {
      cosRot = 0x1a;
      if (0x3890 < rotQ)
      {
        cornerA = 1;
        cornerB = 0;
      pick_128pt:
        cosRot = regionBase;
        if (cornerA)
        {
          bestIdx = (int)(unsigned short)((((short)((short)((int)(short)(*(short*)((char*)DECv32_ANA_QMAP128 + 28) - rotQ) *
                                                                (int)(short)(*(short*)((char*)DECv32_ANA_QMAP128 + 28) - rotQ) >>
                                                            0xd) +
                                                    (short)((int)(short)(0x2799 - rotI) * (int)(short)(0x2799 - rotI) >> 0xd)) <=
                                            (short)((short)((int)(short)(*(short*)((char*)DECv32_ANA_QMAP128 + 52) - rotQ) *
                                                                (int)(short)(*(short*)((char*)DECv32_ANA_QMAP128 + 52) - rotQ) >>
                                                            0xd) +
                                                    (short)((int)(short)(0x3e3a - rotI) * (int)(short)(0x3e3a - rotI) >> 0xd))) -
                                               1 &
                                           0xfff4) +
                                          0x1a);
          goto have_index_128pt;
        }
      }
    }
  }
  regionBase = cosRot;
  if (cornerB)
  {
    bestIdx = (int)(unsigned short)((short)((short)((int)(short)(*(short*)((char*)DECv32_ANA_QMAP128 + 48) - rotQ) *
                                                        (int)(short)(*(short*)((char*)DECv32_ANA_QMAP128 + 48) - rotQ) >>
                                                    0xd) +
                                            (short)((int)(short)(0x32e9 - rotI) * (int)(short)(0x32e9 - rotI) >> 0xd)) <=
                                    (short)((short)((int)(short)(*(short*)((char*)DECv32_ANA_QMAP128 + 30) - rotQ) *
                                                        (int)(short)(*(short*)((char*)DECv32_ANA_QMAP128 + 30) - rotQ) >>
                                                    0xd) +
                                            (short)((int)(short)(0x2799 - rotI) * (int)(short)(0x2799 - rotI) >> 0xd))) *
                  9 +
              0xf;
  }
  else
  {
    bestDist = 0x7fff;
    cand = regionBase;
    do
    {
      errQ = rotQ - *(short*)((char*)DECv32_ANA_QMAP128 + cand * 2);
      dist = (short)(((int)errQ * (int)errQ) >> 0xd) + (short)(((int)(short)(rotI - *(short*)((char*)DECv32_ANA_IMAP128 + cand * 2)) *
                                                                (int)(short)(rotI - *(short*)((char*)DECv32_ANA_IMAP128 + cand * 2))) >>
                                                               0xd);
      if (dist < bestDist)
      {
        bestIdx = cand;
        bestDist = dist;
      }
      cand = (int)(short)((short)cand + 1);
    } while (cand < regionBase + 4);
  }
have_index_128pt:
  cosRot = (int)(short)((short)bestIdx + (short)(quadrant << 5));
  oldQ = *(short*)((char*)DECv32_ANGL14400 + cosRot * 2);
  *magInOut = *(short*)((char*)DECv32_MAG14400 + cosRot * 2);
  *angleInOut = oldQ;
  VTB_decoder(MEMBER_ADDR_A(V32Hist, histAddr, vtb), symI32, symQ32, outBits);
  return outBits[0];
}
