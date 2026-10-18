/**
 * @file fax_timing.c
 * @brief Symbol timing recovery (clock tracking) of the fax receivers: FAXSRE_init(), FAXSRE_recover(), FAXSRE_free().
 *
 * The V.27ter, V.29 and V.17 receivers each embed a FaxSre (fax_rx_common.h) between the matched filter and the equalizer (FAXFSE_receive(), fax_equalizer.c).
 * The V.32 modem has its own FPM_SRE_* with the same algorithm (src/v32/v32_timing.c); those work on a raw block whose pointer slots live outside the object on
 * 64-bit hosts, so these typed copies exist.
 *
 * Data flow: the input samples are stored in a short history (`histLen` = nt / 10 entries).  Once per output sample a 10-phase polyphase interpolator
 * (`coefs`, taps 10 apart, row `phaseRow`) produces a timing-corrected sample.  A timing-error detector (two biquad sections and a combiner, state `st`,
 * coefficients cfg.tedCoef) turns the output into an error term; a clock correlator (cfg.corrCosTab / corrSinTab, accumulators corrA / corrB over one or more
 * symbol periods) turns it into a timing phase; a second-order loop filter with a three-state machine (idle, tracking, locked: loopState; gain tables
 * propGainTab / intGainTab) moves the fractional phase `phaseFrac` and the polyphase row `phaseRow`.  An energy gate (window `rms`, levelMeasure /
 * levelThresh) holds the loop off until a signal is present.  Once locked, the coefficients are re-interpolated between two rows of the table.  At the end of a
 * block the clock offset in ppm is estimated (ppmElapsed .. ppmFirst) and printed at debug level above 1 ("TimingVxx: Timing Offset [ppm] = %d").
 */
#include <string.h>
#include "fax_rx_common.h"
#include "core/dsp_asm.h"
#include "core/dsp_intmath.h" /* WADD */
#include "core/dsp_host.h"
#include "fpm/fpm.h"

/* The int sums that can leave the int range wrap like 32-bit registers and are formed unsigned (WADD / WSUB). */
#define WSUB(a, b) ((int)((unsigned int)(a) - (unsigned int)(b)))
/* The load / store order of this file is the same with and without -fno-strict-aliasing (compared on the objects built with and without the flag). */

/**
 * @brief Create or re-initialise a timing-recovery block.
 * @param s      block to set up
 * @param cfg    configuration, copied (coefTab is the interpolation filter, tedCoef the detector coefficients, corrCosTab / corrSinTab the correlator tables)
 * @param fresh  0: the block was initialised before and its buffers are kept, unless `cfg->nt` is larger than the current `nt`, in which case they
 *               are freed and reallocated ("Reallocating FPM_SRE buffers" at debug level above 1); non-zero: allocate
 *
 * Allocates the coefficient copy (nt shorts), the input history (nt / 10 shorts), the detector state (6 shorts) and the energy window (levelLen shorts), then
 * resets the loop (idle state), the clock correlator and the clock-offset estimator, and loads the coefficients from the table.
 * @warning The sysdep_malloc() results are not checked.
 */
void FAXSRE_init(FaxSre* s, const FaxSreCfg* cfg, int fresh)
{
  short nt;
  int i;

  if (fresh == 0 && s->cfg.nt < cfg->nt)
  {
    if (1 < dsplibs_debug_level)
      dsplibs_debug_printf("Reallocating FPM_SRE buffers");
    fresh = 1;
    sysdep_free(s->st);
    sysdep_free(s->hist);
    sysdep_free(s->coefs);
    sysdep_free(s->rms);
  }

  /* loop and correlator state */
  s->cfg = *cfg;
  COMPILER_BARRIER();
  s->pllEnable = 0;
  s->w44 = 1;
  s->loopState = FAXSRE_LOOP_IDLE;
  nt = s->cfg.nt;
  s->loopInteg = 0;
  s->phaseAvg = 0;
  s->errMagAvg = 0;
  s->loopEnable = 1;
  s->histLen = (short)(nt / 10);
  s->histPos = 0;
  s->corrA = 0;
  s->corrB = 0;
  s->phaseFrac = 0;
  s->phaseRow = 0;
  s->periodsCur = s->cfg.periodsTrack;
  s->startCtr = 0;
  s->periodCount = 0;
  s->corrIdx = 0;
  s->untilSym = 1;
  s->levelMeasure = 1;
  s->levelIdx = 0;

  /* buffers */
  if (fresh != 0)
  {
    s->coefs = (short*)sysdep_malloc((unsigned int)(unsigned short)(nt * 2));
    s->hist = (short*)sysdep_malloc((unsigned int)(unsigned short)(s->histLen * 2));
    s->st = (short*)sysdep_malloc(0xc);
    s->rms = (short*)sysdep_malloc((unsigned int)(unsigned short)(s->cfg.levelLen * 2));
    nt = s->cfg.nt;
  }

  /* initial coefficients, empty history, detector state and energy window */
  if (0 < nt)
  {
    short* buf = s->coefs;
    const short* coef = (const short*)CFGP_WORD(s->cfg.coefTab);
    for (i = 0; i < nt; i++)
      buf[i] = coef[i];
  }
  if (0 < s->histLen)
  {
    short* buf = s->hist;
    for (i = 0; i < s->histLen; i++)
      buf[i] = 0;
  }
  {
    short* buf = s->st;
    for (i = 0; i < 6; i++)
      buf[i] = 0;
  }
  if (0 < s->cfg.levelLen)
  {
    short* buf = s->rms;
    for (i = 0; i < s->cfg.levelLen; i++)
      buf[i] = 0;
  }

  /* clock-offset estimator */
  s->ppmElapsed = 0;
  s->ppmSum = 0;
  s->ppmOffset = 0;
  s->ppmN = 1;
  s->ppmNet = 0;
  s->ppmFirst = 1;
}

/** @brief Free the four buffers of a timing-recovery block (not the block itself). */
void FAXSRE_free(FaxSre* s)
{
  sysdep_free(s->st);
  sysdep_free(s->hist);
  sysdep_free(s->coefs);
  sysdep_free(s->rms);
}

/**
 * @brief Consume input samples and produce timing-corrected output samples, one per symbol slot.
 * @param s           block, initialised by FAXSRE_init(); updated
 * @param rawIn       input samples (the matched-filter output), `count` entries
 * @param outSamples  receives the output samples
 * @param count       number of input samples
 * @return number of output samples produced (16 bit)
 *
 * Input is stored into the history until `untilSym` samples have arrived for the next output sample.  Each output sample is the polyphase FIR of the history
 * (row `phaseRow`); it also runs the timing-error detector and the clock correlator, and, at the end of each correlation interval (periodsCur symbol periods),
 * the loop update: the angle of (corrB, corrA) is the timing phase error, smoothed in `phaseAvg`; the state machine (idle for startCount updates, then
 * tracking,
 * locked when |phaseAvg| <= lockThresh / 8, back to tracking when it exceeds unlockThresh / 8) selects the loop gains; the loop output moves `phaseFrac` /
 * `phaseRow` (row reduced modulo 10, a wrap of two rows counts +1 and no wrap -1 in `ppmNet`).  After the input is consumed the clock-offset estimate is
 * updated.
 * @warning `outSamples` is not bounds-checked against `count`.  The loop update runs only while the carrier gate `pllEnable` is set (errOn / errOff
 *          hysteresis on errMagAvg) and `loopEnable` is non-zero.
 */
unsigned int FAXSRE_recover(FaxSre* s, const unsigned short* rawIn, short* outSamples, short count)
{
  short* hist;
  short outSample;
  unsigned short phaseAvg;
  short ringLen;
  const unsigned short* rawPtr;
  int errMag;
  short untilSym;
  short filtOut;
  short tmpShort;
  const short* histPtr;
  unsigned int timingErr;
  unsigned int ringPos;
  const short* coefPtr;
  int acc;
  int corr;
  short* tedState;
  short loopState;
  unsigned int produced;
  unsigned short phaseAngle;
  /* Locals are named for their dominant role (the same as in FPM_SRE_recover, src/v32/v32_timing.c) but several are reused for unrelated values:
   *   untilSym  = samples until the next symbol (untilSym member); also the sample just read, the FPM_rms result, a table value and the carry of the
   *               phase update
   *   outSample = the interpolated output sample; also the next ring index in the two copy loops and the level-window index
   *   errMag    = the |timing error| average (errMagAvg); also a ring counter and a biquad state;  acc = the FIR accumulator, later the biquad input and
   *               the coefficient index
   *   corr      = the loop-filter input word, later the second correlator word;  timingErr = the ring index in the FIR loop, then the biquad error
   *   filtOut / tmpShort = the biquad output / other 16-bit temporaries;  ppmAcc / ppmN / ppmAvg = the clock-offset estimate (net count * step + sum, n,
   *               their quotient) */

  produced = 0;
  ringLen = s->histLen;
  ringPos = (unsigned int)s->histPos;
  hist = s->hist;
  untilSym = s->untilSym;
  while (count != 0)
  {
    rawPtr = rawIn;
    tmpShort = untilSym;

    /* Fewer samples left than the next output sample needs: only store them and return. */
    if (count < untilSym)
    {
      tmpShort = count + -1;
      if (count != 0)
      {
        do
        {
          outSample = (short)ringPos + 1;
          ringPos = -(unsigned int)(outSample < ringLen) & (unsigned int)(int)outSample;
          hist[ringPos] = (short)*rawIn;
          if (s->levelMeasure != 0)
          {
            outSample = s->levelIdx;
            s->levelIdx = outSample + 1;
            s->rms[outSample] = (short)*rawIn;
            if (s->cfg.levelLen <= s->levelIdx)
              s->levelIdx = 0;
          }
          tmpShort = tmpShort + -1;
          rawIn = rawIn + 1;
        } while (tmpShort != -1);
      }
      s->histPos = (short)ringPos;
      s->untilSym = untilSym - count;
      return produced & 0xffff;
    }

    /* Store the samples up to the next output sample (history ring, and the energy window while the gate is measuring). */
    while (rawIn = rawPtr, tmpShort = tmpShort + -1, tmpShort != -1)
    {
      outSample = (short)ringPos + 1;
      ringPos = -(unsigned int)(outSample < ringLen) & (unsigned int)(int)outSample;
      rawPtr = rawIn + 1;
      hist[ringPos] = (short)*rawIn;
      if (s->levelMeasure != 0)
      {
        outSample = s->levelIdx;
        s->levelIdx = outSample + 1;
        s->rms[outSample] = (short)*rawIn;
        if (s->cfg.levelLen <= s->levelIdx)
          s->levelIdx = 0;
      }
    }

    /* Polyphase interpolation: FIR over the history, newest first, coefficients 10 apart starting at row phaseRow. */
    acc = 0;
    count = count - untilSym;
    coefPtr = s->coefs + s->phaseRow;
    histPtr = hist + ringPos;
    for (timingErr = ringPos; -1 < (int)timingErr; timingErr = timingErr - 1)
    {
      untilSym = *histPtr;
      histPtr = histPtr + -1;
      tmpShort = *coefPtr;
      coefPtr = coefPtr + 10;
      acc = WADD(acc, (int)untilSym * (int)tmpShort);
    }
    histPtr = histPtr + ringLen;
    errMag = (int)ringLen;
    while (errMag = errMag + -1, (int)ringPos < errMag)
    {
      untilSym = *histPtr;
      histPtr = histPtr + -1;
      tmpShort = *coefPtr;
      coefPtr = coefPtr + 10;
      acc = WADD(acc, (int)untilSym * (int)tmpShort);
    }

    /* Timing-error detector: two biquad sections and a combiner over the output sample; timingErr is the combiner output minus its value two samples
     * earlier. */
    outSample = (short)(acc >> 0xf) * 2;
    tedState = s->st;
    histPtr = (const short*)CFGP_WORD(s->cfg.tedCoef);
    corr = outSample * 0x200;
    acc = (int)tedState[0];
    untilSym = (short)(WSUB(WSUB(corr, histPtr[0] * acc), (int)histPtr[1] * (int)tedState[1]) >> 0xe);
    errMag = (int)tedState[2];
    tmpShort = (short)(WSUB(WSUB(corr, histPtr[2] * errMag), (int)histPtr[3] * (int)tedState[3]) >> 0xe);
    filtOut = (short)(WSUB(WADD((int)histPtr[8] * (int)(short)(WADD((int)(short)(tmpShort - (short)(histPtr[6] * errMag >> 0xe)) *
                                                                        (int)(short)(untilSym - (short)(histPtr[4] * acc >> 0xe)),
                                                                    (int)(short)(histPtr[7] * errMag >> 0xe) * (int)(short)(histPtr[5] * acc >> 0xe)) >>
                                                               0xb),
                                (int)histPtr[9] * (int)tedState[4]),
                           (int)histPtr[10] * (int)tedState[5]) >>
                      0xe);
    tedState[1] = tedState[0];
    tedState[0] = untilSym;
    untilSym = tedState[2];
    tedState[2] = tmpShort;
    tmpShort = tedState[4];
    tedState[4] = filtOut;
    tedState[3] = untilSym;
    timingErr = (unsigned int)(short)(filtOut - tedState[5]);
    tedState[5] = tmpShort;

    /* Energy gate: no timing error until the input level exceeds the threshold. */
    if (s->levelMeasure != 0)
    {
      untilSym = FPM_rms(s->rms, s->cfg.levelLen);
      if (s->cfg.levelThresh < untilSym)
        s->levelMeasure = 0;
      else
        timingErr = 0;
    }

    /* Clock correlator: accumulate the error against the cosine / sine tables; track the smoothed |error|. */
    untilSym = s->corrIdx;
    corr = WADD((int)(timingErr * (unsigned int)(int)(((const short*)CFGP_WORD(s->cfg.corrCosTab))[untilSym])) >> 3, s->corrA);
    s->corrIdx = untilSym + 1;
    acc = WSUB(s->corrB, (int)((int)((const short*)CFGP_WORD(s->cfg.corrSinTab))[untilSym] * (int)timingErr) >> 3);
    errMag = (int)(short)((unsigned int)(s->errMagAvg * 0xf) >> 4) + ((int)(((int)timingErr ^ (int)timingErr >> 0x1f) - ((int)timingErr >> 0x1f)) >> 1);
    if (0x7fff < errMag)
      errMag = 0x7fff;
    s->errMagAvg = (short)errMag;

    /* End of a symbol period: after periodsCur periods run the loop update; otherwise just keep accumulating. */
    if ((short)(untilSym + 1) < s->cfg.sps)
    {
      s->corrB = acc;
      s->corrA = corr;
    }
    else
    {
      s->corrIdx = 0;
      untilSym = s->periodCount + 1;
      if (untilSym < s->periodsCur)
      {
        s->periodCount = untilSym;
        s->corrB = acc;
        s->corrA = corr;
      }
      else
      {
        /* carrier gate with hysteresis */
        if (s->cfg.errOn <= (short)errMag)
          s->pllEnable = 1;
        if (s->errMagAvg <= s->cfg.errOff)
          s->pllEnable = 0;
        if (s->pllEnable != 0)
        {
          s->corrB = acc;
          s->corrA = corr;
          if ((s->loopState != FAXSRE_LOOP_LOCKED) || (0x62 < (unsigned short)((short)((unsigned int)corr >> 0x10) + 0x31)))
          {
            s->periodCount = untilSym;
            FPM_atan((short)(acc >> 0x10), (short)(corr >> 0x10), &phaseAngle);
            if (0x4000 < phaseAngle)
              phaseAngle = phaseAngle + -0x8000;
            acc = (int)(short)(((int)s->cfg.sps * (int)(short)phaseAngle) >> 3);
            phaseAvg = (short)(s->phaseAvg * 0xf >> 4) + (short)(acc >> 4);
            s->phaseAvg = phaseAvg;

            /* loop state machine, gains and phase update */
            if (s->loopEnable != 0)
            {
              untilSym = s->startCtr;
              if (s->cfg.startCount < untilSym)
              {
                untilSym = (short)((phaseAvg ^ ((short)phaseAvg >> 0xf)) - ((short)phaseAvg >> 0xf));
                if (untilSym < s->cfg.unlockThresh >> 3)
                {
                  loopState = s->loopState;
                }
                else
                {
                  loopState = s->loopState;
                  if (loopState == FAXSRE_LOOP_LOCKED)
                  {
                    s->loopState = FAXSRE_LOOP_TRACK;
                    loopState = FAXSRE_LOOP_TRACK;
                    s->periodsCur = s->cfg.periodsTrack;
                    s->loopInteg = (short)(((int)s->cfg.unlockGain * (int)s->loopInteg) >> 0xf);
                  }
                }
                if ((untilSym <= s->cfg.lockThresh >> 3) && (loopState == FAXSRE_LOOP_TRACK))
                {
                  untilSym = s->loopInteg;
                  tmpShort = s->cfg.lockGain;
                  filtOut = s->cfg.periodsLock;
                  s->loopState = FAXSRE_LOOP_LOCKED;
                  s->w44 = 0;
                  s->periodsCur = filtOut;
                  loopState = FAXSRE_LOOP_LOCKED;
                  s->loopInteg = (short)(((int)tmpShort * (int)untilSym) >> 0xc);
                }
              }
              else
              {
                s->loopInteg = 0;
                s->startCtr = untilSym + 1;
                if (untilSym < s->cfg.startCount)
                {
                  loopState = FAXSRE_LOOP_IDLE;
                  s->loopState = FAXSRE_LOOP_IDLE;
                }
                else
                {
                  s->loopState = FAXSRE_LOOP_TRACK;
                  loopState = FAXSRE_LOOP_TRACK;
                }
              }

              /* second-order loop filter; the output is limited to -0x4800 .. 0x5000 and split into whole polyphase rows (0x800 per row) and a fraction */
              untilSym = ((const short*)CFGP_WORD(s->cfg.propGainTab))[loopState];
              tmpShort = s->loopInteg;
              s->loopInteg = (short)((((int)(((const short*)CFGP_WORD(s->cfg.intGainTab))[loopState]) * acc + 0x800) >> 0xc)) + tmpShort;
              untilSym = (short)((short)(((short)((untilSym * acc) >> 0xc)) + tmpShort) * -10 >> 4);
              acc = (int)untilSym;
              if (untilSym < 0x5001)
              {
                if (untilSym < -0x4800)
                  acc = -0x4800;
              }
              else
              {
                acc = 0x5000;
              }
              untilSym = (short)(acc >> 0xb) - (short)(acc >> 0x1f);
              tmpShort = untilSym + s->phaseRow;
              untilSym = (short)acc + untilSym * -0x800 + s->phaseFrac;
              if (untilSym < 0)
              {
                s->phaseRow = tmpShort + -1;
                s->phaseFrac = untilSym + 0x800;
              }
              else
              {
                s->phaseFrac = untilSym;
                s->phaseRow = tmpShort;
              }
              untilSym = s->phaseFrac;
              if (0x7ff < untilSym)
              {
                untilSym = untilSym + -0x800;
                s->phaseFrac = untilSym;
                s->phaseRow = s->phaseRow + 1;
              }
              untilSym = (short)((int)untilSym << 4);

              /* locked: blend the two neighbouring coefficient rows by the fractional phase */
              if ((loopState == FAXSRE_LOOP_LOCKED) && (acc = 0, 0 < s->cfg.nt))
              {
                const short* coffs = (const short*)CFGP_WORD(s->cfg.coefTab);
                short* buf = s->coefs;
                do
                {
                  tmpShort = (short)(((int)coffs[acc] * (int)(short)(0x7fff - untilSym)) >> 0xf);
                  buf[acc] = tmpShort;
                  filtOut = (short)acc + 1;
                  buf[acc] = tmpShort + (short)(((int)coffs[acc + 1] * (int)untilSym) >> 0xf);
                  acc = (int)filtOut;
                } while (filtOut < s->cfg.nt);
              }
            }
          }
        }
        s->periodCount = 0;
        s->corrA = 0;
        s->corrB = 0;
      }
    }

    /* Step to the next row; untilSym becomes the number of input samples to take before the next output sample (the row wrapped 0, 1 or 2 times). */
    untilSym = 0;
    tmpShort = s->phaseRow + 10;
    s->phaseRow = tmpShort;
    if (9 < tmpShort)
    {
      untilSym = 0;
      do
      {
        tmpShort = tmpShort - 10;
        untilSym = untilSym + 1;
      } while (9 < tmpShort);
      s->phaseRow = tmpShort;
    }

    timingErr = (unsigned int)(short)((short)produced + 1);
    outSamples[produced] = outSample;
    produced = timingErr;
    if (untilSym == 2)
      s->ppmNet = s->ppmNet + 1;
    else if (untilSym == 0)
      s->ppmNet = s->ppmNet + -1;
  }

  /* Clock-offset estimate: once per ppmPeriod elapsed units, average the net count into ppmOffset (ppmNMax estimates, then restart). */
  if (s->ppmPeriod != (short)(s->ppmElapsed + s->ppmTick))
  {
    s->ppmElapsed = s->ppmElapsed + s->ppmTick;
    goto ppm_done;
  }
  {
    short ppmAcc = s->ppmNet * s->ppmStep + s->ppmSum;
    short ppmN = s->ppmN;
    short ppmAvg = ppmAcc / ppmN;
    s->ppmOffset = ppmAvg;
    if (ppmN < s->ppmNMax)
    {
      ppmN = ppmN + 1;
      s->ppmElapsed = 0;
      s->ppmNet = 0;
      ppmAvg = ppmAcc;
      if (s->ppmFirst != 0)
        goto ppm_restart;
      goto ppm_store;
    }
    else
    {
      s->ppmN = 1;
      s->ppmElapsed = 0;
      ppmN = 2;
      s->ppmNet = 0;
      if (s->ppmFirst == 0)
        goto ppm_store;
    ppm_restart:
      s->ppmSum = 0;
      s->ppmN = 1;
      s->ppmFirst = 0;
      goto ppm_updated;
    }
  ppm_store:
    s->ppmN = ppmN;
    s->ppmSum = ppmAvg;
  ppm_updated:;
  }
  if (1 < dsplibs_debug_level)
    dsplibs_debug_printf("TimingVxx: Timing Offset [ppm] = %d\n", (int)s->ppmOffset);
ppm_done:
  s->histPos = (short)ringPos;
  s->untilSym = untilSym;
  return produced & 0xffff;
}
