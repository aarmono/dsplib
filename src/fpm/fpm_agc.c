/**
 * @file fpm_agc.c
 * @brief FPM_AGC: block automatic gain control shared by the modem receivers.
 *
 * Owns the FpmAgc state (fpm.h) and the four operations on it: FPM_AGC_init() loads a configuration block, FPM_AGC_agc() normalises a sample
 * buffer in place, FPM_AGC_Freeze() / FPM_AGC_Release() hold and free the gain.  The receivers (V.21 / Bell 103, V.23, V.22, V.32, the fax V.17 / V.27 /
 * V.29 / V.21 modems) run it on the line samples before their demodulators and read FpmAgc::detect as a "signal present" flag.
 *
 * Data flow.  The buffer is cut into blocks of FpmAgc::a.s[5] samples.  For every block FPM_rms() (fpm_math.c) gives the level; a block below the squelch
 * threshold is zeroed, every other block updates a smoothed level (a first-order recursion with the two coefficients `alpha[0]`, `beta[0]` of the
 * configuration tables) and, unless the AGC is frozen, a new gain: the reciprocal of the level from FPM_div() times the target level, applied as a
 * Q15 multiplier plus a left shift and saturated to 16 bits.
 */

#include "core/dsp_table_handles.h"
#include "fpm.h"
#include "core/dsp_host.h"

/**
 * @brief Load an AGC configuration block into `agc`.
 * @param agc    AGC state to initialise
 * @param cfg    six configuration words: 0 .. 2 are copied into FpmAgc::a (shorts: [0] target level, [1] squelch threshold while no gain is applied,
 *               [2] squelch threshold once a gain is applied, [5] block length in samples), 3 and 4 are the CFGP handles of the alpha and beta
 *               coefficient tables, 5 is stored in FpmAgc::w5 (not read in this file)
 * @param reset  non-zero: also mark the AGC active and clear the gain (`rcp`, `shift`)
 * @note  Always clears the frozen flag, the smoothed level (`hist`) and `detect`.
 * @note  At debug level above 1 prints "AGC_Release" (the text FPM_AGC_Release() prints) and then a build stamp line.
 */
void FPM_AGC_init(FpmAgc* agc, const int* cfg, int reset)
{
  agc->detect = 0;
  agc->a.w[0] = cfg[0];
  agc->a.w[1] = cfg[1];
  agc->a.w[2] = cfg[2];
  FPM_TABLE_SET(agc, alpha, const short*, cfg[3]); /* table pointers: handles on x86-64 (src/core/dsp_table_handles.h) */
  FPM_TABLE_SET(agc, beta, const short*, cfg[4]);
  agc->w5 = cfg[5];

  /* a fresh AGC starts active with no gain and no level */
  if (reset != 0)
  {
    agc->active = 1;
    agc->rcp = 0;
    agc->shift = 0;
    agc->hist = 0;
  }

  if (1 < dsplibs_debug_level)
  {
    dsplibs_debug_printf("AGC_Release\n");
  }
  agc->freeze = 0;
  agc->hist = 0;

  if (1 < dsplibs_debug_level)
  {
    dsplibs_debug_printf("AGC %s %s\n", "Sep 22 2005", "15:48:18");
  }
}

/**
 * @brief Apply the AGC in place to a buffer of 16-bit samples.
 * @param agc      AGC state (FPM_AGC_init())
 * @param samples  samples, modified in place
 * @param count    number of samples
 *
 * The buffer is cut into blocks of `a.s[5]` samples.  A remainder shorter than half a block is merged into the last block, a longer one forms a block of
 * its own; a buffer shorter than half a block is left untouched.  A block whose RMS is below the squelch threshold (`a.s[1]` while the stored gain
 * `rcp` is 0, `a.s[2]` once it is not) is set to zero.  Any other block updates the smoothed level `hist` = (alpha[0] * hist + beta[0] * rms) >> 15 and,
 * unless the AGC is frozen or that level is 0, the gain: `shift` = (shift of FPM_div(level)) - 1 and `rcp` = (mantissa * a.s[0]) >> 15.  The block is then
 * multiplied by `rcp` (Q15), shifted left by `shift` and saturated to -32768 .. 32767.  `detect` is set when more than half of the blocks (integer
 * count / 2) were not squelched, else cleared.
 * @note  The reciprocal mantissa returned by FPM_div() is unsigned 16 bit (0x8000 for a level that is a power of two) and must be zero-extended; a sign
 *        extension gives a wrong gain.
 * @warning `a.s[5]` is the divisor of the block count; a configuration with block length 0 divides by zero.
 */
void FPM_AGC_agc(FpmAgc* agc, short* samples, unsigned short count)
{
  short target;
  short squelchIdle;
  short squelchActive;
  short blocksLeft;
  short blockRms;
  short gainShort; /* the stored gain as a short; the update branch reuses the variable for the new smoothed level */
  short gainShift;
  short* p;
  unsigned int blockLen;
  unsigned int curLen;
  int rcpGain;
  unsigned int lastBlockLen;
  unsigned int blockCount;
  int activeBlocks;
  int level;
  short recipShift; /* FPM_div()'s two outputs, kept as separate shorts (they must not share storage) */
  short recipMant;

  /* load the state and cut the buffer into blocks */
  activeBlocks = 0;
  level = (int)agc->hist;
  gainShift = agc->shift;
  target = agc->a.s[0];
  rcpGain = (int)agc->rcp;
  blockLen = (unsigned int)(unsigned short)agc->a.s[5];
  squelchIdle = agc->a.s[1];
  squelchActive = agc->a.s[2];
  blockCount = (unsigned int)((unsigned short)count / (unsigned short)blockLen);
  lastBlockLen = (unsigned int)((unsigned short)count % (unsigned short)blockLen);
  if ((unsigned short)lastBlockLen < (unsigned short)((unsigned short)agc->a.s[5] >> 1))
  {
    lastBlockLen = (lastBlockLen + blockLen) & 0xffff;
  }
  else
  {
    blockCount = (blockCount + 1) & 0xffff;
  }
  blocksLeft = (short)blockCount;

  /* per block: squelch, or update the level and gain and apply it */
  while (1)
  {
    blocksLeft = blocksLeft + -1;
    gainShort = (short)rcpGain;
    if (blocksLeft < 0)
      break;
    curLen = blockLen;
    if (blocksLeft == 0)
    {
      curLen = lastBlockLen;
    }
    blockRms = FPM_rms(samples, (short)curLen);
    p = samples;

    if ((blockRms < squelchIdle && gainShort == 0) || (blockRms < squelchActive && gainShort != 0))
    {
      unsigned int n = curLen;
      while ((short)(n - 1) != -1)
      {
        *p = 0;
        p = p + 1;
        n = (n - 1) & 0xffff;
      }
    }
    else
    {
      unsigned int n = curLen;

      activeBlocks = (int)(short)((short)activeBlocks + 1);
      gainShort = (short)(((int)*agc->alpha * level) >> 0xf) + (short)(((int)blockRms * (int)*agc->beta) >> 0xf);
      level = (int)gainShort;

      if (agc->freeze == 0 && gainShort != 0)
      {
        FPM_div((unsigned short)gainShort, &recipMant, &recipShift);
        gainShift = recipShift + -1;
        /* the mantissa is unsigned (0x8000 for a power-of-two level): zero-extend it */
        rcpGain = (int)(short)((int)((unsigned int)(unsigned short)recipMant * (int)target) >> 0xf);
      }

      /* apply the gain with saturation */
      while ((short)(n - 1) != -1)
      {
        unsigned char shiftAmt = (unsigned char)gainShift;
        int scaled = (int)(short)((*p * rcpGain) >> 0xf) << (shiftAmt & 0x1f);
        if (scaled < 0x8000)
        {
          if (scaled < -0x8000)
          {
            *p = -0x8000;
          }
          else
          {
            *p = (short)scaled;
          }
        }
        else
        {
          *p = 0x7fff;
        }
        p = p + 1;
        n = (n - 1) & 0xffff;
      }
    }
    samples = samples + curLen;
  }

  /* save the state and the detect flag */
  agc->hist = (short)level;
  agc->rcp = gainShort;
  agc->shift = gainShift;
  FPM_M(unsigned int, agc, detect) = (unsigned int)((int)(blockCount >> 1) < activeBlocks);
}

/**
 * @brief Hold the current gain: later FPM_AGC_agc() calls keep applying it (the level still follows the input).
 * @param agc  AGC state
 * @note  At debug level above 1 prints "AGC_Freeze".
 */
void FPM_AGC_Freeze(FpmAgc* agc)
{
  if (dsplibs_debug_level < 2)
  {
    agc->freeze = 1;
    return;
  }
  dsplibs_debug_printf("AGC_Freeze\n");
  agc->freeze = 1;
}

/**
 * @brief Undo FPM_AGC_Freeze() and restart the level tracking (the smoothed level `hist` is cleared; the stored gain is kept until the next update).
 * @param agc  AGC state
 * @note  At debug level above 1 prints "AGC_Release".
 */
void FPM_AGC_Release(FpmAgc* agc)
{
  if (dsplibs_debug_level < 2)
  {
    agc->freeze = 0;
    agc->hist = 0;
    return;
  }
  dsplibs_debug_printf("AGC_Release\n");
  agc->freeze = 0;
  agc->hist = 0;
}
