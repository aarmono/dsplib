/**
 * @file v32_pulse_shaping.c
 * @brief V.32 / V.32bis transmit pulse shaper (PPS): an interpolating polyphase FIR over a symbol stream, creation, run and release.
 *
 * The pulse shaper turns the symbol stream of the signal-mapper/coder (SMC, see v32_scrambler.c) into 16-bit line samples.  It owns two circular
 * history buffers (I and Q, `histLen` entries each, malloc'd) and reads two polyphase coefficient tables (ICOFFS / QCOFFS); one output sample is
 * the difference of the I and Q convolutions, scaled by a gain.
 * - FPM_PPS_init(): load the 10-word configuration into the V32Pps object and (re)allocate and clear the history buffers.
 * - FPM_PPS_filter(): produce `count` samples, pulling new symbols from the SMC output ring (V32Circ) whenever the phase accumulator wraps.
 * - FPM_PPS_free(): release the two history buffers.
 *
 * The object is addressed as a `short *` (the first member is the number of phases); the two buffer addresses live in the engine-tail slots of v32.h
 * on x86-64 (accessors PPS_PG / PPS_PS).
 */

#include <stdio.h>
#include "core/dsp_table_handles.h"
#include <stdlib.h>
#include "v32_internal.h"
#include "core/dsp_host.h"

/* the two history-buffer address slots of the PPS object (offsets 0x30 / 0x34 of the 32-bit layout) live in the engine tail on x86-64 (v32.h) */
#define PPS_PG(T, pps, off) V32X_PG(T, pps, V32_PPS_OFF, off)
#define PPS_PS(T, pps, off, v) V32X_PS(T, pps, V32_PPS_OFF, off, v)

/**
 * @brief Configure the pulse shaper and (re)allocate its history buffers.
 * @param pps        PPS object (V32Pps); member 0 receives the number of phases
 * @param cfg        ten ints: [0] number of phases, [1] mapped flag (0: I / Q taken from the ring, else looked up by symbol code), [2] output gain (Q15),
 *                   [3] second phase increment, [4] I symbol map, [5] Q symbol map, [6] I coefficient table, [7] Q coefficient table,
 *                   [8] FIR length (taps), [9] unused word
 * @param allocFlag  1: always allocate fresh history buffers (first creation); 0: keep the existing buffers unless the new length needs more
 * @return history buffer size in bytes when buffers were allocated, 0 when the existing buffers were kept
 *
 * The history length is `firLen / numPhases`; both buffers are zeroed.  The phase accumulator restarts at the first phase increment (already in the
 * object) and `needSample` is cleared, so the first output sample is computed from the zeroed histories before any symbol is fetched.  With
 * `allocFlag == 0` and a larger history length the old buffers are freed and reallocated (debug line "Reallocate FPM_PPS buffer" above level 1).
 * @warning The sysdep_malloc results are not checked.  Callers ignore the return value.
 */
int FPM_PPS_init(V32Pps* pps, const int* cfg, int allocFlag)
{
  short histLen;
  int clearIdx;
  void* newBuf;
  int ret = 0;

  *(int*)pps = cfg[0];
  pps->mapped = cfg[1];
  pps->gain = cfg[2];
  pps->phaseStep2 = cfg[3];
  pps->iMap = cfg[4];
  pps->qMap = cfg[5];
  pps->iCoef = cfg[6];
  pps->qCoef = cfg[7];
  pps->firLen = cfg[8];
  pps->d024 = cfg[9];
  pps->needSample = 0;
  pps->phase = pps->phaseStep;
  pps->writeIdx = 0;
  /* history length per channel: taps divided by the number of phases */
  histLen = MEMBER_IDX(short, V32Pps, pps, firLen) / *(short*)pps;
  if (allocFlag == 0 && pps->histLen < histLen)
  {
    if (1 < dsplibs_debug_level)
    {
      dsplibs_debug_printf("Reallocate FPM_PPS buffer");
    }
    sysdep_free(PPS_PG(void*, pps, (int)offsetof(V32Pps, qHistBuf)));
    sysdep_free(PPS_PG(void*, pps, (int)offsetof(V32Pps, iHistBuf)));
    pps->histLen = histLen;
  }
  else
  {
    pps->histLen = histLen;
    if (allocFlag == 0)
    {
      goto zero_fill;
    }
  }
  /* allocate both buffers */
  newBuf = sysdep_malloc((unsigned int)(histLen * 2));
  PPS_PS(void*, pps, (int)offsetof(V32Pps, iHistBuf), newBuf);
  ret = pps->histLen * 2;
  newBuf = sysdep_malloc((unsigned int)ret);
  PPS_PS(void*, pps, (int)offsetof(V32Pps, qHistBuf), newBuf);
zero_fill:
  /* clear both histories */
  clearIdx = 0;
  if (0 < pps->histLen)
  {
    char* iBuf1 = PPS_PG(char*, pps, (int)offsetof(V32Pps, qHistBuf));
    char* iBuf0 = PPS_PG(char*, pps, (int)offsetof(V32Pps, iHistBuf));
    do
    {
      *(short*)(iBuf0 + clearIdx * 2) = 0;
      *(short*)(iBuf1 + clearIdx * 2) = 0;
      histLen = (short)clearIdx + 1;
      clearIdx = (int)histLen;
    } while (histLen < pps->histLen);
  }
  return ret;
}

/**
 * @brief Produce pulse-shaped output samples.
 * @param pps     PPS object (V32Pps), initialised by FPM_PPS_init()
 * @param circ    the SMC output ring (V32Circ, at engine offset 0xb0): I / Q sample buffers, symbol-code buffer, read index, ring length
 * @param out     receives `count` 16-bit samples
 * @param count   number of output samples to produce
 * @return the number of samples written (`count`)
 *
 * Per output sample: if the phase accumulator wrapped on the previous sample, take the next symbol from the ring (its I / Q values directly when
 * `mapped` is 0, else looked up with the symbol code in the I / Q maps) and push it into the circular histories.  Then convolve each history with
 * its polyphase row (`coef[phase + k * numPhases]`, newest entry first, wrapping), subtract the Q result from the I result (each Q15), scale by the
 * gain and advance the phase by `phaseStep + phaseStep2`, wrapping at `numPhases` (which requests the next symbol).
 * The object (need-sample flag, phase, write index) and the ring read index are updated on return.
 * @warning The ring is not checked for underrun: the caller must have `count` worth of symbols queued.
 */
short FPM_PPS_filter(V32Pps* pps, V32Circ* circ, short* out, unsigned short count)
{
  short numPhases;
  short phaseStep;
  short histLen;
  cfgp_int_t iMap; /* table bases (CFGP_WORD): symbol maps IMAP16 / QMAP16, coefficient tables ICOFFS / QCOFFS */
  int gain;
  cfgp_int_t qMap;
  cfgp_int_t iCoef;
  cfgp_int_t qCoef;
  char* iHist;
  char* qHist;
  char* srcI;
  char* srcQ;
  char* srcCode;
  short phaseShort;
  short produced;
  int histLen32;
  unsigned int tapIdx;
  short scratch16;
  unsigned int needSample;
  int wrapIdx;
  short nextSrcPos;
  short* histPtr;
  unsigned int writeIdx;
  short* coefPtr;
  int accI;
  int accQ;
  short sampleOut;
  int phase;
  unsigned int srcPos;
  unsigned int remaining;
  int bWrap;

  /* load the object and ring state */
  remaining = (unsigned int)count;
  produced = 0;
  numPhases = *(short*)pps;
  phaseStep = pps->phaseStep;
  iMap = CFGP_WORD(pps->iMap);
  gain = pps->gain;
  qMap = CFGP_WORD(pps->qMap);
  iCoef = CFGP_WORD(pps->iCoef);
  qCoef = CFGP_WORD(pps->qCoef);
  iHist = PPS_PG(char*, pps, (int)offsetof(V32Pps, iHistBuf));
  qHist = PPS_PG(char*, pps, (int)offsetof(V32Pps, qHistBuf));
  phase = pps->phase;
  srcI = V32X_PG(char*, circ, 0xb0, (int)offsetof(V32Circ, iBuf));
  histLen = pps->histLen;
  histLen32 = (int)histLen;
  needSample = (unsigned int)pps->needSample;
  writeIdx = (unsigned int)pps->writeIdx;
  srcQ = V32X_PG(char*, circ, 0xb0, (int)offsetof(V32Circ, qBuf));
  srcCode = V32X_PG(char*, circ, 0xb0, (int)offsetof(V32Circ, codeBuf));
  srcPos = (unsigned int)circ->readIdx;
  bWrap = MEMBER_IDX(int, V32Circ, circ, ringLen);
  /* one pass per output sample */
  if (remaining != 0)
  {
    phaseShort = (short)phase;
    do
    {
      phase = (int)phaseShort;
      /* fetch the next symbol into the histories */
      if (needSample != 0)
      {
        unsigned int codeVal;
        unsigned short valI, valQ;
        codeVal = (unsigned int)*(unsigned char*)(srcCode + srcPos * 2);
        if (pps->mapped == 0)
        {
          valI = *(unsigned short*)(srcI + srcPos * 2);
          valQ = *(unsigned short*)(srcQ + srcPos * 2);
        }
        else
        {
          valI = *(unsigned short*)(iMap + codeVal * 2);
          valQ = *(unsigned short*)(qMap + codeVal * 2);
        }
        nextSrcPos = (short)srcPos + 1;
        remaining = remaining - needSample & 0xffff;
        scratch16 = (short)writeIdx + 1;
        srcPos = -(unsigned int)(nextSrcPos < (short)bWrap) & (unsigned int)(int)nextSrcPos;
        writeIdx = -(unsigned int)(scratch16 < histLen) & (unsigned int)(int)scratch16;
        *(unsigned short*)(iHist + writeIdx * 2) = valI;
        *(unsigned short*)(qHist + writeIdx * 2) = valQ;
      }
      /* convolution #1: the I history against the I coefficient row, from the write index backwards with wraparound */
      coefPtr = (short*)(iCoef + phase * 2);
      accI = 0;
      histPtr = (short*)(iHist + writeIdx * 2);
      for (tapIdx = writeIdx; -1 < (int)tapIdx; tapIdx = tapIdx - 1)
      {
        scratch16 = *histPtr;
        histPtr = histPtr + -1;
        accI = accI + (int)scratch16 * (int)*coefPtr;
        coefPtr = coefPtr + numPhases;
      }
      histPtr = histPtr + histLen32;
      for (accQ = histLen32 + -1; (int)writeIdx < accQ; accQ = accQ + -1)
      {
        scratch16 = *histPtr;
        histPtr = histPtr + -1;
        accI = accI + (int)scratch16 * (int)*coefPtr;
        coefPtr = coefPtr + numPhases;
      }
      /* convolution #2: the Q history against the Q coefficient row (`accQ` first serves as the wrap counter of convolution #1) */
      coefPtr = (short*)(qCoef + phase * 2);
      accQ = 0;
      histPtr = (short*)(qHist + writeIdx * 2);
      for (tapIdx = writeIdx; -1 < (int)tapIdx; tapIdx = tapIdx - 1)
      {
        scratch16 = *histPtr;
        histPtr = histPtr + -1;
        accQ = accQ + (int)scratch16 * (int)*coefPtr;
        coefPtr = coefPtr + numPhases;
      }
      histPtr = histPtr + histLen32;
      for (wrapIdx = histLen32 + -1; (int)writeIdx < wrapIdx; wrapIdx = wrapIdx + -1)
      {
        scratch16 = *histPtr;
        histPtr = histPtr + -1;
        accQ = accQ + (int)scratch16 * (int)*coefPtr;
        coefPtr = coefPtr + numPhases;
      }
      /* combine (I - Q), scale by the gain, emit one sample */
      sampleOut = (short)(accI >> 0xf) - (short)(accQ >> 0xf);
      produced = produced + 1;
      *out = (short)((int)sampleOut * gain >> 0xf);
      out = out + 1;
      /* advance the phase accumulator; a wrap requests a new symbol for the next sample */
      phase = phaseShort + phaseStep + MEMBER_IDX(short, V32Pps, pps, phaseStep2);
      {
        int bWrapPhase = numPhases <= (short)phase;
        if (bWrapPhase)
        {
          phase = (short)phase - numPhases;
        }
        needSample = (unsigned int)bWrapPhase;
      }
      phaseShort = (short)phase;
    } while (remaining != 0);
  }
  /* store the state back */
  pps->needSample = (short)needSample;
  pps->writeIdx = (short)writeIdx;
  pps->phase = (short)phase;
  circ->readIdx = (short)srcPos;
  return produced;
}

/**
 * @brief Release the two history buffers of a pulse shaper.
 * @param pps     PPS object (V32Pps) whose buffers were allocated by FPM_PPS_init()
 * @note The object itself belongs to the caller; the buffer pointers are not cleared, so the object must not be run again without FPM_PPS_init().
 */
void FPM_PPS_free(V32Pps* pps)
{
  sysdep_free(PPS_PG(void*, pps, (int)offsetof(V32Pps, qHistBuf)));
  sysdep_free(PPS_PG(void*, pps, (int)offsetof(V32Pps, iHistBuf)));
}
