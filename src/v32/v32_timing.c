/**
 * @file v32_timing.c
 * @brief V.32 / V.32bis receive timing recovery (SRE): creation, run and release of the symbol-rate estimator and resampler.
 *
 * The SRE sits between the echo canceller / receive filter and the equaliser (DemodDataV32() in v32_datapath.c).  It keeps a ring of input samples
 * and, once per output sample, convolves it with one row of a 10-phase interpolation filter (`coefBuf`, 10 rows of `ringLen` taps), so it
 * resamples the line signal at an adjustable fractional delay.  A timing-tone detector then measures the delay error: the output is band-pass
 * filtered (a biquad cascade in `tedState`), the filtered difference is correlated with a cosine / sine pair over the number of samples held in the first word of the object and the
 * correlation phase (FPM_atan) drives a second-order loop (proportional + integral gains selected by the loop state: 0 acquire, 1 track,
 * 2 locked) that moves the interpolation row/fraction.  In the locked state the coefficient rows are re-blended by the fractional part.  When the
 * row leaves 0..9 an input sample is skipped or repeated; the net count is turned into a timing offset in ppm (debug line "TimingVxx").
 * - FPM_SRE_init(): load the 14-word configuration, allocate (or keep) the four work buffers, clear the state.
 * - FPM_SRE_recover(): process one block of input samples, writing the resampled output.
 * - FPM_SRE_free(): release the four work buffers.
 *
 * The object is V32Sre; the configuration is copied into its first 14 words with int stores and read back with short loads, so the accessors are
 * may_alias views.  The four buffer addresses live in the engine-tail slots of v32.h on x86-64.
 */

#include <stdint.h>
#include <stdio.h>
#include "core/dsp_table_handles.h"
#include <stdlib.h>
#include "v32_internal.h"
#include "core/dsp_host.h"
#include "fpm/fpm.h"

/* may_alias: this object is accessed at the same byte offsets as int, short and unsigned short (FPM_SRE_init copies the 14-word cfg with int
 * stores and then reads cfg[3] back as the short at +0xc).  With plain short / int lvalues gcc -O2 (strict aliasing) hoists the short load above
 * the int stores, so the buffer sizes are computed from an uninitialised +0xc (sysdep_malloc(0)) and the later fills overrun the heap.  A test that
 * creates the object over pre-initialised memory does not show it; a real session does. */
typedef short __attribute__((may_alias)) sre_short_t;
typedef unsigned short __attribute__((may_alias)) sre_ushort_t;
typedef int __attribute__((may_alias)) sre_int_t;
#define SRE_S(m) MEMBER_AS_OF(sre_short_t, V32Sre, sre, m)
#define SRE_US(m) MEMBER_AS_OF(sre_ushort_t, V32Sre, sre, m)
#define SRE_I(m) MEMBER_AS_OF(sre_int_t, V32Sre, sre, m)
#define SRE_IAT(bytes) (*(sre_int_t*)((char*)sre + (bytes))) /* the 14 config words by index */
/* the four buffer-address slots (offsets 0x50, 0x54, 0x58, 0x74 of the 32-bit layout) live in the engine tail on x86-64 (v32.h); at 32 bits they are
 * plain may_alias int members */
#if UINTPTR_MAX == 0xffffffffu
#define SRE_PG(T, o) ((T)SRE_I(o))
#define SRE_PS(o, v) (SRE_I(o) = (int)(v))
#else
#define SRE_PG(T, m) V32X_PG(T, sre, V32_SRE_OFF, (int)offsetof(V32Sre, m))
#define SRE_PS(m, v) V32X_PS(void*, sre, V32_SRE_OFF, (int)offsetof(V32Sre, m), v)
#endif

/**
 * @brief Configure the timing recovery and (re)allocate its work buffers.
 * @param sre        SRE object (V32Sre)
 * @param cfg        14 ints copied into the first 14 words of the object: periods (short 0) and correlation counts, loop thresholds, interpolation
 *                   filter length (short at +0xc, in taps) and table, timing-tone cosine / sine tables, proportional / integral gain tables,
 *                   error-magnitude thresholds, level-detector threshold and length, ppm averaging constants
 * @param allocFlag  1: allocate fresh buffers (first creation); 0: keep the existing buffers unless the new filter is longer
 *
 * Allocates the interpolation coefficient buffer (`coefLen` shorts, filled from the table), the sample ring (`coefLen / 10` shorts), the 6-short
 * biquad state and the level-detector buffer (`levelLen` shorts), clears the ring, state and level buffer, and resets the loop to the acquire state
 * (loop state 0, phase row/fraction 0, counters and averages cleared, level measurement on, ppm estimator restarted).  If the old buffers are
 * too short they are freed and reallocated (debug line "Reallocating FPM_SRE buffers" above level 1).
 * @warning The sysdep_malloc results are not checked.  `coefTab` must hold `coefLen + 1` entries (the locked-state blend reads one entry beyond
 *          the filter length).
 */
void FPM_SRE_init(V32Sre* sre, const int* cfg, int allocFlag)
{
  short coefLen;
  int i;

  if (allocFlag == 0 && SRE_S(coefLen) < *(const short*)((const char*)cfg + 0xc))
  {
    if (1 < dsplibs_debug_level)
    {
      dsplibs_debug_printf("Reallocating FPM_SRE buffers");
    }
    allocFlag = 1;
    sysdep_free(V32X_PG(void*, sre, V32_SRE_OFF, (int)offsetof(V32Sre, tedState)));
    sysdep_free(V32X_PG(void*, sre, V32_SRE_OFF, (int)offsetof(V32Sre, ringBuf)));
    sysdep_free(V32X_PG(void*, sre, V32_SRE_OFF, (int)offsetof(V32Sre, coefBuf)));
    sysdep_free(V32X_PG(void*, sre, V32_SRE_OFF, (int)offsetof(V32Sre, levelBuf)));
  }
  SRE_I(pllEnable) = 0;
  for (i = 0; i < 14; i++)
  {
    SRE_IAT(i * 4) = cfg[i];
  }
  SRE_I(d044) = 1;
  SRE_US(loopState) = 0;
  coefLen = SRE_S(coefLen);
  SRE_US(loopInteg) = 0;
  SRE_US(phaseAvg) = 0;
  SRE_US(errMagAvg) = 0;
  SRE_I(loopEnable) = 1;
  SRE_S(ringLen) = coefLen / 10;
  SRE_US(ringPos) = 0;
  SRE_I(corrA) = 0;
  SRE_I(corrB) = 0;
  SRE_US(phaseFrac) = 0;
  SRE_US(phaseRow) = 0;
  SRE_US(periodsCur) = MEMBER_AS_OF(unsigned short, V32Sre, sre, periodsTrack);
  SRE_US(w06a) = 0;
  SRE_US(periodCount) = 0;
  SRE_US(corrIdx) = 0;
  SRE_US(untilSym) = 1;
  SRE_US(levelMeasure) = 1;
  SRE_US(levelIdx) = 0;
  if (allocFlag != 0)
  {
    SRE_PS(coefBuf, sysdep_malloc((unsigned int)(unsigned short)(coefLen * 2)));
    SRE_PS(ringBuf, sysdep_malloc((unsigned int)(unsigned short)(SRE_S(ringLen) * 2)));
    SRE_PS(tedState, sysdep_malloc(0xc));
    SRE_PS(levelBuf, sysdep_malloc((unsigned int)(unsigned short)(SRE_S(levelLen) * 2)));
    coefLen = SRE_S(coefLen);
  }
  if (0 < coefLen)
  {
    char* buf = SRE_PG(char*, coefBuf);
    cfgp_int_t coef = CFGP_WORD(SRE_I(coefTab));
    for (i = 0; i < coefLen; i++)
    {
      *(short*)((char*)buf + i * 2) = *(short*)((char*)coef + i * 2);
    }
  }
  if (0 < SRE_S(ringLen))
  {
    char* buf = SRE_PG(char*, ringBuf);
    for (i = 0; i < SRE_S(ringLen); i++)
    {
      *(short*)((char*)buf + i * 2) = 0;
    }
  }
  {
    char* buf = SRE_PG(char*, tedState);
    for (i = 0; i < 6; i++)
    {
      *(short*)((char*)buf + i * 2) = 0;
    }
  }
  if (0 < SRE_S(levelLen))
  {
    char* buf = SRE_PG(char*, levelBuf);
    for (i = 0; i < SRE_S(levelLen); i++)
    {
      *(short*)((char*)buf + i * 2) = 0;
    }
  }
  SRE_US(w07e) = 0;
  SRE_US(ppmSum) = 0;
  SRE_US(ppmOffset) = 0;
  SRE_US(ppmN) = 1;
  SRE_US(ppmNet) = 0;
  SRE_US(w08e) = 1;
}

#undef SRE_S
#undef SRE_US
#undef SRE_I
#undef SRE_IAT

/**
 * @brief Release the four work buffers of the timing recovery.
 * @param sre     SRE object whose buffers were allocated by FPM_SRE_init()
 */
void FPM_SRE_free(V32Sre* sre)
{
  sysdep_free(V32X_PG(void*, sre, V32_SRE_OFF, (int)offsetof(V32Sre, tedState)));
  sysdep_free(V32X_PG(void*, sre, V32_SRE_OFF, (int)offsetof(V32Sre, ringBuf)));
  sysdep_free(V32X_PG(void*, sre, V32_SRE_OFF, (int)offsetof(V32Sre, coefBuf)));
  sysdep_free(V32X_PG(void*, sre, V32_SRE_OFF, (int)offsetof(V32Sre, levelBuf)));
}

/**
 * @brief Resample a block of line samples and update the timing loop.
 * @param sre         SRE object (V32Sre, addressed as shorts), initialised by FPM_SRE_init()
 * @param rawIn       `count` input samples (read only)
 * @param outSamples  receives the resampled output as 16-bit values, one per output instant (the caller's receive buffer is reused)
 * @param count       number of input samples
 * @return the number of output samples written
 *
 * Input samples are pushed into the ring; every `untilSym` samples (normally 1; 0 or 2 right after a skipped or repeated sample) one output is
 * formed by convolving the ring with the current coefficient row.  Each output also feeds the timing-error detector (band-pass biquads, level gate
 * on the rms of the last `levelLen` inputs, error magnitude average, sine / cosine correlation).  After that many samples (the first word of the object) the correlators are
 * evaluated; after `periodsCur` such periods the loop either gates itself on the error magnitude (hysteresis `errOn` / `errOff`), or updates the
 * phase average, the loop state (acquire / track / locked by the phase deviation and a period counter), the proportional / integral filter and the
 * resampling row/fraction (clamped to about +-1 sample), and in the locked state re-blends the coefficient rows.  The row advances by 10 per output;
 * a wrap in either direction shifts `untilSym` and bumps the net slip count `ppmNet`, which the ppm estimator averages and prints.
 * Ring position and samples remaining until the next output are stored back.
 * @warning `outSamples` must hold up to `count + 1` shorts; there is no bounds check.
 */
unsigned int FPM_SRE_recover(V32Sre* sre, unsigned short* rawIn, char* outSamples, short count)
{
  char* ringBuf;
  short outSample;
  unsigned short phaseAvg;
  short ringLen;
  unsigned short* rawPtr;
  int errMag;
  short untilSym;
  short filtOut;
  short tmpShort;
  short* histPtr;
  unsigned int timingErr;
  unsigned int ringPos;
  short* coefPtr;
  int acc;
  int corr;
#if UINTPTR_MAX == 0xffffffffu
#define iCoffs corr /* 32-bit build: shares `corr` */
#else
  cfgp_int_t iCoffs; /* 64-bit build: the resolved coefficient table base cannot live in an int */
#endif
  char* bufP; /* the coefficient buffer address, a pointer local at both widths */
  /* The names give the dominant role; several locals are reused for unrelated values:
   *   untilSym  = samples until the next symbol (w070); also the sample just read, the FPM_rms result, the table index and the carry of the phase update
   *   outSample = the interpolated output sample; also the next ring index in the two copy loops and the debug-buffer index
   *   errMag    = the running |timing error| average (w03e); also a ring-length counter and a biquad state
   *   acc       = the FIR accumulator; also the second correlator word, the loop-filter output and the coefficient-blend index
   *   corr      = the first correlator word (d05c); at -m32 also the resolved COFFS table base (iCoffs)
   *   filtOut / tmpShort = the biquad output / other 16-bit temporaries (a coefficient, a state, a phase fraction) */
  short loopState;
  unsigned int produced;
  unsigned short phaseAngle;

  /* load the ring state */
  produced = 0;
  ringLen = sre->ringLen;
  ringPos = (unsigned int)sre->ringPos;
  ringBuf = V32X_PG(char*, sre, V32_SRE_OFF, (int)offsetof(V32Sre, ringBuf));
  untilSym = sre->untilSym;
  while (count != 0)
  {
    /* push input samples into the ring (and the level buffer) up to the next output instant; a short block only fills and returns */
    rawPtr = rawIn;
    tmpShort = untilSym;
    if (count < untilSym)
    {
      tmpShort = count + -1;
      if (count != 0)
      {
        do
        {
          outSample = (short)ringPos + 1;
          ringPos = -(unsigned int)(outSample < ringLen) & (unsigned int)(int)outSample;
          *(unsigned short*)(ringBuf + ringPos * 2) = *rawIn;
          if (sre->levelMeasure != 0)
          {
            outSample = sre->levelIdx;
            sre->levelIdx = outSample + 1;
            *(unsigned short*)(V32X_PG(char*, sre, V32_SRE_OFF, (int)offsetof(V32Sre, levelBuf)) + outSample * 2) = *rawIn;
            if (sre->levelLen <= sre->levelIdx)
            {
              sre->levelIdx = 0;
            }
          }
          tmpShort = tmpShort + -1;
          rawIn = rawIn + 1;
        } while (tmpShort != -1);
      }
      sre->ringPos = (short)ringPos;
      sre->untilSym = untilSym - count;
      return produced & 0xffff;
    }
    while (rawIn = rawPtr, tmpShort = tmpShort + -1, tmpShort != -1)
    {
      outSample = (short)ringPos + 1;
      ringPos = -(unsigned int)(outSample < ringLen) & (unsigned int)(int)outSample;
      rawPtr = rawIn + 1;
      *(unsigned short*)(ringBuf + ringPos * 2) = *rawIn;
      if (sre->levelMeasure != 0)
      {
        outSample = sre->levelIdx;
        sre->levelIdx = outSample + 1;
        *(unsigned short*)(V32X_PG(char*, sre, V32_SRE_OFF, (int)offsetof(V32Sre, levelBuf)) + outSample * 2) = *rawIn;
        if (sre->levelLen <= sre->levelIdx)
        {
          sre->levelIdx = 0;
        }
      }
    }
    /* interpolate one output: ring convolved with the current coefficient row (stride 10), newest sample first, wrapping */
    acc = 0;
    count = count - untilSym;
    coefPtr = (short*)(V32X_PG(char*, sre, V32_SRE_OFF, (int)offsetof(V32Sre, coefBuf)) + sre->phaseRow * 2);
    histPtr = (short*)(ringBuf + ringPos * 2);
    for (timingErr = ringPos; -1 < (int)timingErr; timingErr = timingErr - 1)
    {
      untilSym = *histPtr;
      histPtr = histPtr + -1;
      tmpShort = *coefPtr;
      coefPtr = coefPtr + 10;
      acc = acc + (int)untilSym * (int)tmpShort;
    }
    histPtr = histPtr + ringLen;
    errMag = (int)ringLen;
    while (errMag = errMag + -1, (int)ringPos < errMag)
    {
      untilSym = *histPtr;
      histPtr = histPtr + -1;
      tmpShort = *coefPtr;
      coefPtr = coefPtr + 10;
      acc = acc + (int)untilSym * (int)tmpShort;
    }
    /* timing-error detector: biquad band-pass cascade on the output, error = filtered output minus its previous state */
    outSample = (short)(acc >> 0xf) * 2;
    coefPtr = V32X_PG(short*, sre, V32_SRE_OFF, (int)offsetof(V32Sre, tedState));
    histPtr = CFGP_SLOT(short*, MEMBER_ADDR_PIDX(short, V32Sre, sre, tedCoef));
    corr = outSample * 0x200;
    acc = (int)*coefPtr;
    untilSym = (short)(((corr - *histPtr * acc) - (int)histPtr[1] * (int)coefPtr[1]) >> 0xe);
    errMag = (int)coefPtr[2];
    tmpShort = (short)(((corr - histPtr[2] * errMag) - (int)histPtr[3] * (int)coefPtr[3]) >> 0xe);
    filtOut =
        (short)(((int)histPtr[8] *
                     (int)(short)(((int)(short)(tmpShort - (short)(histPtr[6] * errMag >> 0xe)) * (int)(short)(untilSym - (short)(histPtr[4] * acc >> 0xe)) +
                                   (int)(short)(histPtr[7] * errMag >> 0xe) * (int)(short)(histPtr[5] * acc >> 0xe)) >>
                                  0xb) +
                 (int)histPtr[9] * (int)coefPtr[4] - (int)histPtr[10] * (int)coefPtr[5]) >>
                0xe);
    coefPtr[1] = *coefPtr;
    *coefPtr = untilSym;
    untilSym = coefPtr[2];
    coefPtr[2] = tmpShort;
    tmpShort = coefPtr[4];
    coefPtr[4] = filtOut;
    coefPtr[3] = untilSym;
    timingErr = (unsigned int)(short)(filtOut - coefPtr[5]);
    coefPtr[5] = tmpShort;
    /* level gate: no timing error until the input rms exceeds the threshold */
    if (sre->levelMeasure != 0)
    {
      untilSym = FPM_rms((const short*)(V32X_PG(char*, sre, V32_SRE_OFF, (int)offsetof(V32Sre, levelBuf))), sre->levelLen);
      if (sre->levelThresh < untilSym)
      {
        sre->levelMeasure = 0;
      }
      else
      {
        timingErr = 0;
      }
    }
    /* correlate the error with the timing-tone cosine / sine and track its magnitude (15/16 leaky average) */
    untilSym = sre->corrIdx;
    corr = ((int)(timingErr * (unsigned int)(int)(*(short*)(CFGP_WORD(sre->corrCosTab) + untilSym * 2))) >> 3) + sre->corrA;
    sre->corrIdx = untilSym + 1;
    acc = sre->corrB - ((int)((int)*(short*)(CFGP_WORD(sre->corrSinTab) + untilSym * 2) * (int)timingErr) >> 3);
    errMag = (int)(short)((unsigned int)(sre->errMagAvg * 0xf) >> 4) + ((int)(((int)timingErr ^ (int)timingErr >> 0x1f) - ((int)timingErr >> 0x1f)) >> 1);
    if (0x7fff < errMag)
    {
      errMag = 0x7fff;
    }
    sre->errMagAvg = (short)errMag;
    /* end of a correlation period: count it, and after periodsCur periods run the loop */
    if ((short)(untilSym + 1) < *(short*)sre)
    {
      sre->corrB = acc;
      sre->corrA = corr;
    }
    else
    {
      sre->corrIdx = 0;
      untilSym = sre->periodCount + 1;
      if (untilSym < sre->periodsCur)
      {
        sre->periodCount = untilSym;
        sre->corrB = acc;
        sre->corrA = corr;
      }
      else
      {
        /* hysteresis on the error magnitude decides whether the loop runs */
        if (sre->errOn <= (short)errMag)
        {
          MEMBER_IDX(short, V32Sre, sre, pllEnable) = 1;
          MEMBER_IDX_AT(short, V32Sre, sre, pllEnable, 1) = 0;
        }
        if (sre->errMagAvg <= sre->errOff)
        {
          MEMBER_IDX(short, V32Sre, sre, pllEnable) = 0;
          MEMBER_IDX_AT(short, V32Sre, sre, pllEnable, 1) = 0;
        }
        if (sre->pllEnable != 0)
        {
          sre->corrB = acc;
          sre->corrA = corr;
          if ((sre->loopState != 2) || (0x62 < (unsigned short)((short)((unsigned int)corr >> 0x10) + 0x31)))
          {
            sre->periodCount = untilSym;
            /* phase of the correlation, folded to +-half a turn, smoothed */
            FPM_atan((short)(acc >> 0x10), (short)(corr >> 0x10), (unsigned short*)&phaseAngle);
            if (0x4000 < phaseAngle)
            {
              phaseAngle = phaseAngle + -0x8000;
            }
            acc = (int)(short)(((int)*(short*)sre * (int)(short)phaseAngle) >> 3);
            phaseAvg = (short)(sre->phaseAvg * 0xf >> 4) + (short)(acc >> 4);
            sre->phaseAvg = phaseAvg;
            /* loop state machine: 0 acquire, 1 track, 2 locked */
            if (sre->loopEnable != 0)
            {
              untilSym = sre->w06a;
              if (sre->w006 < untilSym)
              {
                untilSym = (short)((phaseAvg ^ ((short)phaseAvg >> 0xf)) - ((short)phaseAvg >> 0xf));
                if (untilSym < sre->w02c >> 3)
                {
                  loopState = sre->loopState;
                }
                else
                {
                  loopState = sre->loopState;
                  if (loopState == 2)
                  {
                    sre->loopState = 1;
                    loopState = 1;
                    sre->periodsCur = MEMBER_IDX(short, V32Sre, sre, periodsTrack);
                    sre->loopInteg = (short)(((int)sre->w008 * (int)sre->loopInteg) >> 0xf);
                  }
                }
                if ((untilSym <= sre->w02e >> 3) && (loopState == 1))
                {
                  untilSym = sre->loopInteg;
                  tmpShort = sre->w00a;
                  filtOut = sre->periodsLock;
                  sre->loopState = 2;
                  MEMBER_IDX(short, V32Sre, sre, d044) = 0;
                  MEMBER_IDX_AT(short, V32Sre, sre, d044, 1) = 0;
                  sre->periodsCur = filtOut;
                  loopState = 2;
                  sre->loopInteg = (short)(((int)tmpShort * (int)untilSym) >> 0xc);
                }
              }
              else
              {
                sre->loopInteg = 0;
                sre->w06a = untilSym + 1;
                if (untilSym < sre->w006)
                {
                  loopState = 0;
                  sre->loopState = 0;
                }
                else
                {
                  sre->loopState = 1;
                  loopState = 1;
                }
              }
              /* proportional + integral filter of the phase, clamped, converted to a row / fraction step (2048 fractions per row) */
              untilSym = *(short*)(CFGP_WORD(sre->propGainTab) + loopState * 2);
              tmpShort = sre->loopInteg;
              sre->loopInteg = (short)((((int)(*(short*)(CFGP_WORD(sre->intGainTab) + loopState * 2)) * acc + 0x800) >> 0xc)) + tmpShort;
              untilSym = (short)((short)(((short)((untilSym * acc) >> 0xc)) + tmpShort) * -10 >> 4);
              acc = (int)untilSym;
              if (untilSym < 0x5001)
              {
                if (untilSym < -0x4800)
                {
                  acc = -0x4800;
                }
              }
              else
              {
                acc = 0x5000;
              }
              untilSym = (short)(acc >> 0xb) - (short)(acc >> 0x1f);
              tmpShort = untilSym + sre->phaseRow;
              untilSym = (short)acc + untilSym * -0x800 + sre->phaseFrac;
              if (untilSym < 0)
              {
                sre->phaseRow = tmpShort + -1;
                sre->phaseFrac = untilSym + 0x800;
              }
              else
              {
                sre->phaseFrac = untilSym;
                sre->phaseRow = tmpShort;
              }
              untilSym = sre->phaseFrac;
              if (0x7ff < untilSym)
              {
                untilSym = untilSym + -0x800;
                sre->phaseFrac = untilSym;
                sre->phaseRow = sre->phaseRow + 1;
              }
              untilSym = (short)((int)untilSym << 4);
              /* locked: re-blend the coefficient rows between two table positions by the fraction */
              if ((loopState == 2) && (acc = 0, 0 < sre->coefLen))
              {
                bufP = V32X_PG(char*, sre, V32_SRE_OFF, (int)offsetof(V32Sre, coefBuf));
                iCoffs = CFGP_WORD(sre->coefTab);
                do
                {
                  tmpShort = (short)(((int)*(short*)(iCoffs + acc * 2) * (int)(short)(0x7fff - untilSym)) >> 0xf);
                  *(short*)(bufP + acc * 2) = tmpShort;
                  filtOut = (short)acc + 1;
                  *(short*)(bufP + acc * 2) = tmpShort + (short)(((int)*(short*)(iCoffs + 2 + acc * 2) * (int)untilSym) >> 0xf);
                  acc = (int)filtOut;
                } while (filtOut < sre->coefLen);
              }
            }
          }
        }
        sre->periodCount = 0;
        MEMBER_IDX(short, V32Sre, sre, corrA) = 0;
        MEMBER_IDX_AT(short, V32Sre, sre, corrA, 1) = 0;
        MEMBER_IDX(short, V32Sre, sre, corrB) = 0;
        MEMBER_IDX_AT(short, V32Sre, sre, corrB, 1) = 0;
      }
    }
    /* advance the row by 10; leaving 0..9 skips (untilSym 2) or repeats (untilSym 0) an input sample and is counted for the ppm estimate */
    untilSym = 0;
    tmpShort = sre->phaseRow + 10;
    sre->phaseRow = tmpShort;
    if (9 < tmpShort)
    {
      untilSym = 0;
      do
      {
        tmpShort = tmpShort - 10;
        untilSym = untilSym + 1;
      } while (9 < tmpShort);
      sre->phaseRow = tmpShort;
    }
    timingErr = (unsigned int)(short)((short)produced + 1);
    *(short*)(outSamples + produced * 2) = outSample;
    produced = timingErr;
    if (untilSym == 2)
    {
      sre->ppmNet = sre->ppmNet + 1;
    }
    else if (untilSym == 0)
    {
      sre->ppmNet = sre->ppmNet + -1;
    }
  }
  /* ppm estimator: close an averaging window every w08a / w07c blocks and print the offset */
  if (sre->w08a != (short)(sre->w07e + sre->w07c))
  {
    sre->w07e = sre->w07e + sre->w07c;
    goto skip_ppm;
  }
  {
    short ppmSum = sre->ppmNet * sre->ppmStep + sre->ppmSum;
    short avgN = sre->ppmN;
    short ppmAvg = ppmSum / avgN;
    sre->ppmOffset = ppmAvg;
    if (avgN < sre->ppmNMax)
    {
      avgN = avgN + 1;
      sre->w07e = 0;
      sre->ppmNet = 0;
      ppmAvg = ppmSum;
      if (sre->w08e != 0)
      {
        goto reset_47;
      }
      goto store_42_40;
    }
    else
    {
      sre->ppmN = 1;
      sre->w07e = 0;
      avgN = 2;
      sre->ppmNet = 0;
      if (sre->w08e == 0)
      {
        goto store_42_40;
      }
    reset_47:
      sre->ppmSum = 0;
      sre->ppmN = 1;
      sre->w08e = 0;
      goto after_42_40;
    }
  store_42_40:
    sre->ppmN = avgN;
    sre->ppmSum = ppmAvg;
  after_42_40:;
  }
  if (1 < dsplibs_debug_level)
  {
    dsplibs_debug_printf("TimingVxx: Timing Offset [ppm] = %d\n", (int)sre->ppmOffset);
  }
skip_ppm:
  sre->ringPos = (short)ringPos;
  sre->untilSym = untilSym;
  return produced & 0xffff;
}
