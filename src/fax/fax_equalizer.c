/**
 * @file fax_equalizer.c
 * @brief Carrier-tracking adaptive equalizer and symbol decision loop of the fax receivers: FAXFSE_init(), FAXFSE_receive(), FAXFSE_free().
 *
 * The V.27ter, V.29 and V.17 receivers each embed a FaxFse (fax_rx_common.h) in their state bundle.  The V.32 modem has its own FPM_FSE_* with the same
 * algorithm (src/v32/v32_equalizer.c); those work on a raw block whose pointer slots live outside the object on 64-bit hosts, so these typed copies exist.
 *
 * Data flow: the timing-recovery output (FAXSRE_recover(), fax_timing.c; `sps` samples per symbol) goes into a circular history `hbuf` of `nt` entries.
 * Once a symbol's worth of samples has arrived (`untilSym` counts the samples still needed) two `nt`-tap LMS equalizers (tapsI, tapsQ; the history is read
 * backwards from the write index) produce an I / Q pair.  FPM_atan() / FPM_phasor() derotate it by the carrier phase (a table entry indexed by `phaseAcc`,
 * plus the carrier accumulator `carrierAcc`) and by the predicted phase offset `predOffset`.  The decision callback `cfg.cb` of the owning receiver chooses
 * the symbol (it may rewrite the angle and magnitude) and returns the code, which goes to the output array.  The phase error against the decided angle
 * drives, in order:
 * - the smoothed error `phaseErrAvg`;
 * - a second-order phase-locked loop (`freqAcc`, `carrierAcc`; gain tables propTab / intTab selected by `loopState`) when `pllEnable` is set;
 * - a four-tap phase predictor (`ph` history, `pc` coefficients) when `predEnable` is set;
 * - the smoothed squared decision error `decErrAvg`, which gates the equalizer update with FPM_lmsupd() (when decErrAvg > 0 and `eqEnable`, or `eqForce`).
 *
 * Debug output (level above 1): "Decoder Error = %d" about every 7200 input samples.  The sample counter behind it (avg_err) is a file-static, shared by all
 * fax receivers in the process.
 */
#include <limits.h>
#include <string.h>
#include "fax_rx_common.h"
#include "v32/v32.h" /* FPM_lmsupd (declared in v32.h) takes the history address as a char * */
#include "core/dsp_asm.h"
#include "core/dsp_intmath.h" /* WADD */
#include "core/dsp_host.h"
#include "fpm/fpm.h"

/* The load / store order of this file is the same with and without -fno-strict-aliasing (compared on the objects built with and without the flag).  The int sums that can leave the int range
 * (the FIR accumulators, the PLL accumulators) wrap like 32-bit registers and are formed unsigned (WADD). */

/** Input samples seen since the last "Decoder Error" debug line; shared by all fax receivers. */
static int avg_err;

/**
 * @brief Create or re-initialise a carrier / equalizer block.
 * @param f      block to set up
 * @param cfg    configuration, copied (iCoefTab / qCoefTab are the initial taps, carrierTab the carrier phase table, propTab / intTab the PLL gains)
 * @param fresh  0: the block was initialised before, its five buffers are freed first ("Reallocating FPM_FSE buffers" at debug level above 1); non-zero: new
 *
 * Allocates the I / Q taps and the history (nt shorts each) and the I / Q symbol output arrays (blockLen / sps + 2 shorts), loads the initial taps, zeroes
 * the history and the PLL / predictor state and enables PLL, predictor and equalizer adaptation.
 * @warning The sysdep_malloc() results are not checked; `cfg->sps` must be non-zero.
 */
void FAXFSE_init(FaxFse* f, const FaxFseCfg* cfg, int fresh)
{
  int i;
  short nt;
  short *ti, *tq, *tz;
  const short *ci, *cq;
  int sz;

  /* run-time state */
  f->cfg = *cfg;
  COMPILER_BARRIER();
  f->eqForce = 0;
  f->pllEnable = 1;
  f->predEnable = 1;
  f->eqEnable = 1;
  f->freqAcc = 0;
  f->muIdx = 0;
  f->loopState = FAXFSE_GAIN_SETTLE;
  f->phaseErrAvg = 0;
  f->decErrAvg = 0;
  f->count = 0;
  f->idx = 0;
  f->pos = 0;
  f->carrierAcc = 0;
  f->phaseAcc = 0;
  f->predOffset = 0;
  f->settleCtr = 0;
  f->untilSym = 1;

  /* buffers: release the old ones for a re-initialisation, allocate the new set */
  if (fresh == 0)
  {
    if (1 < dsplibs_debug_level)
      dsplibs_debug_printf("Reallocating FPM_FSE buffers");
    sysdep_free(f->outQ);
    sysdep_free(f->outI);
    sysdep_free(f->hbuf);
    sysdep_free(f->tapsQ);
    sysdep_free(f->tapsI);
  }
  sz = (int)(short)(f->cfg.nt * 2);
  f->tapsI = (short*)sysdep_malloc((unsigned int)sz);
  f->tapsQ = (short*)sysdep_malloc((unsigned int)sz);
  f->hbuf = (short*)sysdep_malloc((unsigned int)sz);
  sz = (int)(short)((f->cfg.blockLen / f->cfg.sps) * 2 + 4);
  f->outI = (short*)sysdep_malloc((unsigned int)sz);
  f->outQ = (short*)sysdep_malloc((unsigned int)sz);
  nt = f->cfg.nt;

  /* initial taps from the configuration tables, empty history */
  if (0 < nt)
  {
    tz = f->hbuf;
    tq = f->tapsQ;
    ti = f->tapsI;
    ci = (const short*)CFGP_WORD(f->cfg.iCoefTab);
    cq = (const short*)CFGP_WORD(f->cfg.qCoefTab);
    i = 0;
    do
    {
      ti[i] = ci[i];
      tq[i] = cq[i];
      tz[i] = 0;
      i = (int)(short)(i + 1);
    } while ((short)i < f->cfg.nt);
  }

  /* phase predictor and diagnostics */
  for (i = 0; (short)i < 4; i = (int)(short)(i + 1))
  {
    f->pc[i] = 0;
    f->ph[i] = 0;
  }
  f->nd0 = 0;
  f->nd1 = 0;
  f->nd2 = 0;
}

/** @brief Free the five buffers of a carrier / equalizer block (not the block itself). */
void FAXFSE_free(FaxFse* f)
{
  sysdep_free(f->outQ);
  sysdep_free(f->outI);
  sysdep_free(f->hbuf);
  sysdep_free(f->tapsQ);
  sysdep_free(f->tapsI);
}

/**
 * @brief Run `count` timing-recovered samples through the equalizer, carrier loop and decision callback.
 * @param f        block, initialised by FAXFSE_init(); updated
 * @param rx       input samples (one per symbol slot, Q15), `count` entries
 * @param outBits  receives one decided symbol code per completed symbol
 * @param count    number of input samples
 * @return number of symbols decided (also f->idx); the equalized, derotated points are left in f->outI / f->outQ and mirrored (halved) into the diag0 ring
 *
 * Per completed symbol: equalize (I and Q FIR, Q15), form the angle and magnitude, subtract the carrier phase (table entry plus loop accumulator) and the
 * predicted offset, call `cfg.cb(f, &angle, &mag)`, then update the smoothed phase error, the PLL (gain set chosen by the settle counter and the thresholds
 * errThreshHi / errThreshLo), the phase predictor and the decision error, and finally adapt the taps with step size cfg.mu[muIdx].  Samples that do not
 * complete a symbol are only stored, and the carrier table position advances by crrAdj per sample modulo crrLen.
 * @warning `outBits` and the f->outI / f->outQ arrays (blockLen / sps + 2 entries) are not bounds-checked against `count`.  The callback is invoked as
 *          `short (*)(FaxFse*, short*, short*)` and may rewrite the angle and magnitude and switch the block's own flags.
 * @bug  The diag0 ring wraps by resetting the index when it reaches FAXFSE_DIAG0 entries, so the entry at that index is never written (a diagnostic only).
 */
short FAXFSE_receive(FaxFse* f, const short* rx, short* outBits, unsigned short count)
{
  short* tq = f->tapsQ;
  short* ti = f->tapsI;
  short* hb = f->hbuf;
  int tablePos = (int)f->phaseAcc;
  const short crrAdj = f->cfg.crrAdj;
  const short crrLen = f->cfg.crrLen;
  short untilSym = f->untilSym;
  const short nt = f->cfg.nt;
  const int ntInt = (int)nt;
  unsigned int histPos = (unsigned int)(unsigned short)f->pos;
  short phasorBuf[4];
#define phsAngle phasorBuf[0]
#define phsCos phasorBuf[1]
#define phsSin phasorBuf[2]
#define phsInc phasorBuf[3]
  short rawAngle, decAngle, magBuf[3];
  short *symQOut, *symIOut;
  short eqI, mag, phaseErr, eqQ, nextPos;
  short decBits;
  unsigned short errAvg;
  int carrierPhase, tmpInt, acc;
  short *histPtr, *coefPtr;
  unsigned int tapIdx;
  unsigned short cnt = count; /* remaining input samples */
  /* Locals are named for their dominant role (the same as in FPM_FSE_receive, src/v32/v32_equalizer.c) but several are reused for unrelated values:
   *   untilSym = samples still needed for the symbol (untilSym member);  eqI / eqQ = the I / Q equalizer outputs (eqI is also the sample counter before the
   *              symbol is complete)
   *   mag      = the rotated magnitude, also the smoothed-error temporary of the PLL;  phaseErr = the phase error after the derotation, also a 16-bit
   *              temporary and the table sum
   *   acc      = the FIR accumulator, then the Q output * 2 and the PLL / LMS error words;  tmpInt = the I output * 2, then the PLL integral and the LMS
   *              step size
   *   nextPos  = the table position before the wrap (not read again);  decAngle / magBuf[0] = the angle / magnitude the decision callback rewrites */

  /* per-call bookkeeping */
  avg_err = avg_err + (int)cnt;
  f->idx = 0;
  f->count = cnt;
  phsInc = 0;
  nextPos = f->phaseAcc;
  if (cnt != 0)
  {
    symQOut = f->outQ;
    symIOut = f->outI;
    eqI = untilSym;
    do
    {
      untilSym = eqI;

      /* Not enough samples for another symbol: store them and wait for the next call. */
      if ((short)cnt < eqI)
      {
        if (0 < (short)cnt)
        {
          eqI = eqI - cnt;
          errAvg = cnt;
          while (errAvg = errAvg - 1, errAvg != USHRT_MAX)
          {
            short s = (short)histPos + 1;
            short v = *rx;
            rx = rx + 1;
            histPos = -(unsigned int)(s < nt) & (unsigned int)(int)s;
            hb[histPos] = v;
          }
          tablePos = (int)(short)((short)tablePos + cnt * crrAdj);
          while ((short)tablePos >= crrLen)
            tablePos = (int)(short)((short)tablePos - crrLen);
        }
        f->pos = (short)histPos;
        f->phaseAcc = (short)tablePos;
        f->untilSym = eqI;
        return f->idx;
      }

      /* Complete the symbol: store the samples still missing and advance the carrier table position. */
      while ((short)(untilSym + -1) != -1)
      {
        short s = (short)histPos + 1;
        short v = *rx;
        rx = rx + 1;
        histPos = -(unsigned int)(s < nt) & (unsigned int)(int)s;
        hb[histPos] = v;
        untilSym = untilSym + -1;
      }
      nextPos = eqI * crrAdj + (short)tablePos;
      tablePos = (int)nextPos;
      while ((short)tablePos >= crrLen)
        tablePos = (int)(short)((short)tablePos - crrLen);
      untilSym = f->cfg.sps;
      carrierPhase = (int)(short)(((const short*)CFGP_WORD(f->cfg.carrierTab))[tablePos] + (short)((f->carrierAcc + 0x4000) >> 0xf));
      cnt = cnt - eqI;

      /* I equalizer: FIR over the history, newest sample first, wrapping to the end of the buffer. */
      histPtr = hb + (int)histPos;
      acc = 0;
      coefPtr = ti;
      for (tapIdx = histPos; -1 < (int)tapIdx; tapIdx = tapIdx - 1)
      {
        eqI = *histPtr;
        histPtr = histPtr + -1;
        phaseErr = *coefPtr;
        coefPtr = coefPtr + 1;
        acc = WADD(acc, (int)eqI * (int)phaseErr);
      }
      histPtr = histPtr + ntInt;
      for (tmpInt = ntInt + -1; (int)histPos < tmpInt; tmpInt = tmpInt + -1)
      {
        eqI = *histPtr;
        histPtr = histPtr + -1;
        phaseErr = *coefPtr;
        coefPtr = coefPtr + 1;
        acc = WADD(acc, (int)eqI * (int)phaseErr);
      }
      eqI = (short)(acc >> 0xf);

      /* Q equalizer. */
      acc = 0;
      histPtr = hb + (int)histPos;
      coefPtr = tq;
      for (tapIdx = histPos; -1 < (int)tapIdx; tapIdx = tapIdx - 1)
      {
        phaseErr = *histPtr;
        histPtr = histPtr + -1;
        mag = *coefPtr;
        coefPtr = coefPtr + 1;
        acc = WADD(acc, (int)phaseErr * (int)mag);
      }
      histPtr = histPtr + ntInt;
      for (tmpInt = ntInt + -1; (int)histPos < tmpInt; tmpInt = tmpInt + -1)
      {
        phaseErr = *histPtr;
        histPtr = histPtr + -1;
        mag = *coefPtr;
        coefPtr = coefPtr + 1;
        acc = WADD(acc, (int)phaseErr * (int)mag);
      }
      eqQ = (short)(acc >> 0xf);

      /* Angle and magnitude of the equalized point, then remove the carrier phase and the predicted offset. */
      acc = (int)(short)(eqQ * 2);
      tmpInt = (int)(short)(eqI * 2);
      FPM_atan((short)acc, (short)tmpInt, (unsigned short*)&rawAngle);
      phsAngle = rawAngle;
      FPM_phasor(phasorBuf);
      mag = (short)(WADD((int)phsCos * tmpInt, (int)phsSin * acc) >> 0xf);
      phaseErr = (short)(((int)rawAngle - carrierPhase) >> 1) - f->predOffset;
      if (phaseErr < 0)
        phaseErr = phaseErr + 0x4000;
      if (0x3fff < phaseErr)
        phaseErr = phaseErr + -0x4000;
      phsInc = 0;
      phaseErr = phaseErr * 2;
      phsAngle = phaseErr;
      FPM_phasor(phasorBuf);

      /* The derotated point is the symbol output (and, halved, the diagnostic ring). */
      {
        const short iv = (short)(((int)phsCos * (int)mag) >> 0xd);
        const short qv = (short)(((int)phsSin * (int)mag) >> 0xd);
        *symIOut = iv;
        acc = f->nd0;
        *symQOut = qv;
        if (acc < 0x1e0)
        {
          f->diag0[acc * 2] = (int)iv >> 1;
          f->diag0[acc * 2 + 1] = (int)qv >> 1;
          f->nd0 = acc + 1;
        }
        else
        {
          f->nd0 = 0;
        }
      }
      symIOut = symIOut + 1;
      symQOut = symQOut + 1;

      /* Decision: the owner's callback picks the symbol code and the ideal angle / magnitude. */
      decAngle = phaseErr;
      magBuf[0] = mag;
      decBits = (short)f->cfg.cb(f, &decAngle, magBuf);
      *outBits = decBits;
      outBits = outBits + 1;
      f->idx = (short)(f->idx + 1);

      /* Phase error against the decided angle, smoothed (0.95 / 0.05). */
      phaseErr = phaseErr - decAngle;
      if (phaseErr < -0x4000)
        phaseErr = phaseErr + -0x8000;
      if (0x4000 < phaseErr)
        phaseErr = phaseErr + -0x8000;
      acc = (int)phaseErr;
      errAvg = (unsigned short)((short)((f->phaseErrAvg * 0x799a) >> 0xf) + (short)((acc * 0x666) >> 0xf));
      f->phaseErrAvg = (short)errAvg;

      /* Carrier loop: choose the gain set (settling, wide or tight), update the frequency and carrier accumulators, then the phase predictor. */
      if (f->pllEnable != 0)
      {
        mag = f->settleCtr;
        if (f->cfg.settleCount < mag)
        {
          mag = (short)((errAvg ^ (unsigned short)((short)errAvg >> 0xf)) - (unsigned short)((short)errAvg >> 0xf));
          if (f->cfg.errThreshHi <= mag)
            f->loopState = FAXFSE_GAIN_WIDE;
          if (mag <= f->cfg.errThreshLo)
            f->loopState = FAXFSE_GAIN_TIGHT;
        }
        else
        {
          f->settleCtr = mag + 1;
          if (mag < f->cfg.settleCount)
            f->loopState = FAXFSE_GAIN_SETTLE;
          else
            f->loopState = FAXFSE_GAIN_WIDE;
        }
        mag = ((const short*)CFGP_WORD(f->cfg.propTab))[f->loopState];
        tmpInt = f->freqAcc;
        f->freqAcc = WADD(((const short*)CFGP_WORD(f->cfg.intTab))[f->loopState] * acc, tmpInt);
        tmpInt = WADD(WADD(mag * acc, f->carrierAcc), tmpInt);
        if (tmpInt < -0x20000000)
          tmpInt = WADD(tmpInt, 0x40000000);
        if (0x20000000 < tmpInt)
          tmpInt = WADD(tmpInt, -0x40000000);
        f->carrierAcc = tmpInt;
        if (f->predEnable != 0)
        {
          short prevPred;
          int j;
          for (acc = 3; 0 < acc; acc = (int)(short)(acc - 1))
            f->ph[acc] = f->ph[acc - 1];
          prevPred = f->predOffset;
          f->predOffset = 0;
          mag = 0;
          f->ph[0] = phaseErr + prevPred;
          for (j = 0; j < 4; j++)
            mag = (short)(f->ph[j] * f->pc[j] + mag);
          f->predOffset = mag;
        }
      }

      /* Decision error: distance between the equalizer output and the ideal point (decided angle and magnitude put back on the carrier). */
      phsAngle = (short)((decAngle + carrierPhase) >> 1);
      if (phsAngle < 0)
        phsAngle = phsAngle + 0x4000;
      if (0x3fff < phsAngle)
        phsAngle = phsAngle + -0x4000;
      phsAngle = phsAngle * 2;
      FPM_phasor(phasorBuf);
      acc = (int)(short)((short)(((int)phsCos * (int)magBuf[0]) >> 0xe) + eqI * -2);
      carrierPhase = (int)(short)((short)(((int)magBuf[0] * (int)phsSin) >> 0xe) + eqQ * -2);
      tapIdx = (unsigned int)(((f->decErrAvg * 0x799a) >> 0xf) +
                              (((short)(WADD(acc * acc, carrierPhase * carrierPhase) >> 0xb) * 0x666) >> 0xf)); /* the 32 bit sum is compared unsigned */
      if (tapIdx < 0x8000)
        f->decErrAvg = (short)tapIdx;
      else
        f->decErrAvg = 0x7fff;

      /* Adapt both equalizers by the error times the step size, unless disabled or the decision error is nil. */
      if (((0 < f->decErrAvg) && (f->eqEnable != 0)) || (f->eqForce != 0))
      {
        tmpInt = (int)f->cfg.mu[f->muIdx];
        FPM_lmsupd(ti, (char*)(hb), (short)histPos, (short)ntInt, (short)(((acc * tmpInt) + 0x800) >> 0xc));
        FPM_lmsupd(tq, (char*)(hb), (short)histPos, (short)ntInt, (short)(((carrierPhase * tmpInt) + 0x800) >> 0xc));
      }

      /* Periodic debug report. */
      if (0x1c1f < avg_err)
      {
        if (1 < dsplibs_debug_level)
          dsplibs_debug_printf("Decoder Error = %d\n", (int)f->decErrAvg);
        avg_err = 0;
      }
      eqI = untilSym;
    } while (cnt != 0);
  }

  f->pos = (short)histPos;
  f->phaseAcc = (short)tablePos;
  f->untilSym = untilSym;
  (void)nextPos;
  return f->idx;
#undef phsAngle
#undef phsCos
#undef phsSin
#undef phsInc
}
