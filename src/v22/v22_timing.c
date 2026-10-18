/**
 * @file v22_timing.c
 * @brief V.22 / V.22bis SRE: symbol timing recovery (a 10-phase polyphase interpolator, a 4th-order IIR band-pass filter and a symbol-timing PLL).
 *
 * Entry points: V22_SRE_init(), V22_SRE_free(), V22_SRE_recover().  The object is V22Sre (v22.h); every 32-bit product that can overflow is computed in
 * unsigned arithmetic (MUL / ADD / SUB) so that it wraps.
 *
 * V22_SRE_recover() resamples the receive filter output with an interpolator whose phase (`idx`, units of `len` taps, plus the fraction `frac`) is steered
 * by the PLL: per output sample the 27 newest input samples are weighed with the coefficient row `idx`; the output sample is also passed through the IIR
 * band-pass filter, whose output is correlated with the 6-sample symbol clock (xCLOCK / yCLOCK) over `blk` symbols.  From the correlation the timing error
 * angle is derived and drives the proportional + integral PLL, which moves the interpolator phase (and in mode 2 re-derives the coefficient table from the
 * prototype with a fractional-tap shift `frac`).  The loop modes (0 settling, 1 coarse, 2 fine) select the gain sets; `avgMag` (the band-pass magnitude)
 * decides whether a signal is tracked (`wantTrack`, the engine's carrier detect).
 */
#include <stddef.h>
#include "v22.h"
#include "v22_tables.h"
#include "core/dsp_host.h"
#include "fpm/fpm.h"

#define MUL(a, b) ((int)((unsigned int)(int)(a) * (unsigned int)(int)(b)))
#define ADD(a, b) ((int)((unsigned int)(int)(a) + (unsigned int)(int)(b)))
#define SUB(a, b) ((int)((unsigned int)(int)(a) - (unsigned int)(int)(b)))

/**
 * @brief Initialise the timing recovery.
 * @param sre       object
 * @param allocate  non-zero: allocate the coefficient table (0x21c bytes), the history (len * 4 bytes) and the IIR state (0xc bytes); zero: keep the
 *                  existing buffers (a re-initialisation)
 *
 * The loop state is reset (mode 0, rem 1, blk 3, ...).  The coefficient table is SREv22_COFFS[0 .. 0x10d] REORDERED into the polyphase layout: for
 * e = 0x104 .. 0x10d the entries e, e - 10, ... >= 0 follow each other (27 per row, 10 rows, 270 coefficients).  Only the first `len` history samples
 * and the six IIR state words are cleared.
 * @warning The allocation results are not checked.
 */
void V22_SRE_init(V22Sre* sre, int allocate)
{
  short *coefTab, *histBuf, *biquad;
  short phased[0x10e];
  int phaseRow, i, pos, outPos, len;

  sre->wantTrack = 0;
  sre->pllFlagC = 1;
  sre->pllEnable = 1;
  sre->mode = 0;
  sre->integ = 0;
  sre->avgAngle = 0;
  sre->avgMag = 0;
  sre->len = 0x1b;
  sre->wpos = 0;
  sre->accX = 0;
  sre->accY = 0;
  sre->frac = 0;
  sre->idx = 0;
  sre->blk = 3;
  sre->cnt = 0;
  sre->sym = 0;
  sre->clk = 0;
  sre->rem = 1;
  if (allocate != 0)
  {
    sre->coefs = (short*)sysdep_malloc(0x21c);
    sre->hist = (short*)sysdep_malloc((unsigned int)((short)((short)(2 * sre->len)) * 2));
    sre->state = (short*)sysdep_malloc(0xc);
  }
  coefTab = sre->coefs;
  histBuf = sre->hist;
  biquad = sre->state;
  len = sre->len;
  for (i = 0; i <= 0x10d; i++)
    coefTab[i] = SREv22_COFFS[i];
  outPos = 0;
  for (phaseRow = 0x104; (short)phaseRow <= 0x10d; phaseRow++)
  {
    for (pos = phaseRow; (short)pos >= 0; pos = (short)(pos - 10))
      phased[outPos++] = coefTab[pos];
  }
  for (i = 0; i <= 0x10d; i++)
    coefTab[i] = phased[i];
  for (i = 0; i < len; i++)
    histBuf[i] = 0;
  for (i = 0; i <= 5; i++)
    biquad[i] = 0;
}

/** @brief Free the three buffers allocated by V22_SRE_init(). */
void V22_SRE_free(V22Sre* sre)
{
  sysdep_free(sre->state);
  sysdep_free(sre->hist);
  sysdep_free(sre->coefs);
}

/**
 * @brief Resample a block of receive samples at the recovered symbol timing.
 * @param sre    object
 * @param in     `count` samples (the receive filter output)
 * @param out    receives the interpolated samples; may be a buffer unrelated to `in` (the engine passes its input block)
 * @param count  number of input samples
 * @return the number of output samples (zero-extended 16 bits)
 *
 * `rem` is not a constant: after every output sample it is the number of 10s that fit in the new phase index, (idx + 10) / 10, normally 0 .. 2 (up to 3 at
 * the limit of the timing correction), so the interpolator produces an output sample with that many new input samples; this is the timing recovery by
 * resampling.  A call with fewer samples than `rem` only stores them.
 * @bug  Unlike V22_FSE_receive() the partial-call path has no `count > 0` guard: a negative `count` passes a huge length to sysdep_memcpy() (docs/bugs.md;
 *       callers pass block sizes >= 0).
 * @warning `out` is not bounded; every output sample is written to out[nout].
 */
unsigned int V22_SRE_recover(V22Sre* sre, const short* in, short* out, short count)
{
  int cnt = count;
  int nout = 0;
  int len = sre->len;
  int wpos = sre->wpos;
  short* hist = sre->hist;
  short* coefsP = sre->coefs;
  short* biquad = sre->state;
  int rem = sre->rem;

  while (cnt != 0)
  {
    int next, sum, k;
    short filtered, bq1, bq2, bqOut, bpNew, bpPrev1, bpPrev2, mode;
    int bpDiff, accX, accY, scaled;
    short *coefRow, *window;
    int oldX, oldY;
    short clk;

    if ((short)cnt < (short)rem)
    {
      rem = (short)(rem - cnt);
      next = wpos + cnt;
      if (next > 0x36)
      {
        sysdep_memcpy(hist, hist + 0x1b, 0x36);
        wpos = (short)(wpos - 0x1b);
        next = wpos + cnt;
      }
      sysdep_memcpy(hist + wpos, in, (size_t)cnt * 2);
      sre->wpos = (short)next;
      sre->rem = (short)rem;
      return (unsigned short)nout;
    }
    next = wpos + rem;
    if (next > 0x36)
    {
      sysdep_memcpy(hist, hist + 0x1b, 0x36);
      wpos = (short)(wpos - 0x1b);
      next = wpos + rem;
    }
    sysdep_memcpy(hist + wpos, in, (size_t)rem * 2);
    in += rem;
    cnt = (short)(cnt - rem);
    wpos = (short)next;
    oldX = sre->accX; /* the integer accumulators are read before the first 16-bit store (the plain and the -fno-strict-aliasing code are then the same) */
    oldY = sre->accY;
    coefRow = coefsP + MUL(sre->idx, len);
    window = ((short)wpos < (short)len) ? hist : hist + (wpos - len);
    sum = 0;
    for (k = 0; k <= 0x1a; k++)
      sum = ADD(sum, MUL(coefRow[k], window[k]));
    filtered = (short)(sum >> 15);

    /* 4th order IIR (two biquads) on the filtered sample */
    scaled = (int)((unsigned int)(int)filtered << 9);
    bq1 = (short)(SUB(scaled, MUL(biquad[1], 0x3f00)) >> 14);
    bq2 = (short)(SUB(SUB(scaled, MUL(biquad[2], 0x6dfc)), MUL(biquad[3], 0x3f00)) >> 14);
    {
      short x1 = (short)(bq1 * 2);
      short y1 = (short)(MUL(biquad[0], -0x3f80) >> 13);
      short y2 = (short)(MUL(biquad[2], -0x1fbf) >> 13);
      short aa = (short)(SUB(bq2, MUL(biquad[2], -0x36fd) >> 14) * 2);
      bqOut = (short)(ADD(MUL(aa, x1), MUL(y2, y1)) >> 13);
    }
    bpPrev1 = biquad[4];
    bpPrev2 = biquad[5];
    biquad[5] = bpPrev1;
    bpNew = (short)(SUB(ADD(MUL(bqOut, 0x3e0), MUL(bpPrev1, 0x3c01)), MUL(bpPrev2, 0x383f)) >> 14);
    biquad[4] = bpNew;
    bpDiff = (int)(bpNew - bpPrev2) >> 1;
    {
      short o0 = biquad[0], o2 = biquad[2];
      biquad[0] = bq1;
      biquad[2] = bq2;
      biquad[1] = o0;
      biquad[3] = o2;
    }

    clk = sre->clk;
    mode = sre->mode;
    sre->clk = (short)(clk + 1);
    accX = ADD(MUL(bpDiff, SREv22_xCLOCK[clk]) >> 3, oldX);
    accY = SUB(oldY, MUL(SREv22_yCLOCK[clk], bpDiff) >> 3);
    {
      short av = (short)(MUL(sre->avgMag, SRE_ALPHA_AVG[mode]) >> 15);
      int au = (bpDiff ^ (bpDiff >> 31)) - (bpDiff >> 31);
      sre->avgMag = av;
      sre->avgMag = (short)ADD(av, ADD(MUL(au, SRE_BETA_AVG[mode]), 0x4000) >> 15);
    }
    if ((short)(clk + 1) <= 5)
    {
      sre->accY = accY;
      sre->accX = accX;
    }
    else
    {
      short symNext;
      sre->clk = 0;
      symNext = (short)(sre->sym + 1);
      if (sre->blk > symNext)
      {
        sre->sym = symNext;
        sre->accY = accY;
        sre->accX = accX;
      }
      else
      {
        /* a full estimate block: every path of this branch ends with sym = accX = accY = 0, so the stores of accX / accY / sym before it have no effect */
        if (sre->avgMag > 2)
          sre->wantTrack = 1;
        if (sre->avgMag <= 1)
        {
          sre->wantTrack = 0;
          sre->pllFlagC = 1;
          sre->avgAngle = 0;
          sre->blk = 3;
          sre->cnt = 0;
        }
        if (sre->wantTrack != 0)
        {
          int accXHi = accX >> 16, accYHi = accY >> 16;
          if (mode != 2 || (unsigned short)(accXHi + 0x31) > 0x62)
          {
            unsigned short ang;
            short angleErr;
            int modeSel;
            FPM_atan((short)accYHi, (short)accXHi, &ang);
            if ((short)ang > 0x4000)
              ang = (unsigned short)(ang - 0x8000);
            angleErr = (short)(((short)ang * 3) >> 1);
            {
              short av = (short)(MUL(SRE_ALPHA_AVG[sre->mode], sre->avgAngle) >> 15);
              sre->avgAngle = av;
              sre->avgAngle = (short)ADD(av, MUL(SRE_BETA_AVG[sre->mode], angleErr) >> 15);
            }
            modeSel = sre->mode;
            if (sre->pllEnable != 0)
            {
              short settleCnt = sre->cnt;
              if (settleCnt > 0x40)
              {
                int v = sre->avgAngle;
                short av = (short)((v ^ (v >> 31)) - (v >> 31));
                if (av > 0x7ff && modeSel == 2)
                {
                  sre->mode = 1;
                  sre->blk = 3;
                  sre->integ = (short)(sre->integ >> 2);
                  modeSel = 1;
                }
                if (av <= 4 && modeSel == 1)
                {
                  int ig = sre->integ;
                  sre->mode = 2;
                  sre->blk = 0xc;
                  sre->pllFlagC = 0;
                  sre->integ = (short)(ig * 4);
                  modeSel = 2;
                }
              }
              else
              {
                sre->integ = 0;
                sre->cnt = (short)(settleCnt + 1);
                modeSel = (settleCnt <= 0x3f) ? 0 : 1;
                sre->mode = (short)modeSel;
              }
              {
                int ig = (unsigned short)sre->integ;
                short propTerm = (short)ADD(MUL(SREv22_PLL_K1[modeSel], angleErr) >> 12, ig);
                short corr, corrInt;
                int idxNew, fracNew;
                sre->integ = (short)ADD(ADD(MUL(SREv22_PLL_K2[modeSel], angleErr), 0x800) >> 12, ig);
                corr = (short)(SUB(0, MUL(propTerm, 10)) >> 4);
                if (corr > 0x5000)
                  corr = 0x5000;
                else if (corr < -0x4800)
                  corr = -0x4800;
                corrInt = (short)(corr >> 11);
                if (corr < 0)
                  corrInt = (short)(corrInt + 1);
                idxNew = ADD(corrInt, (unsigned short)sre->idx);
                fracNew = ADD(SUB(corr, (int)((unsigned int)(int)corrInt << 11)), (unsigned short)sre->frac);
                if ((short)fracNew < 0)
                {
                  sre->idx = (short)(idxNew - 1);
                  sre->frac = (short)(fracNew + 0x800);
                }
                else
                {
                  sre->frac = (short)fracNew;
                  sre->idx = (short)idxNew;
                }
                if (sre->frac > 0x7ff)
                {
                  sre->frac = (short)(sre->frac - 0x800);
                  sre->idx = (short)(sre->idx + 1);
                }
                if (modeSel == 2)
                {
                  int fr4 = (int)((unsigned int)(int)sre->frac << 4);
                  short wB = (short)fr4;
                  short wA = (short)(0x7fff - fr4);
                  short* coefOut = coefsP;
                  int e, j, outIdx = 0;
                  for (e = 0x104; (short)e <= 0x10d; e++)
                  {
                    for (j = e; (short)j >= 0; j = (short)(j - 10))
                    {
                      coefOut[outIdx] = (short)ADD(MUL(SREv22_COFFS[j], wA) >> 15, MUL(SREv22_COFFS[j + 1], wB) >> 15);
                      outIdx = (short)(outIdx + 1);
                    }
                  }
                }
              }
            }
          }
        }
        sre->sym = 0;
        sre->accX = 0;
        sre->accY = 0;
      }
    }
    {
      short idx = (short)(sre->idx + 10);
      int r = 0;
      sre->idx = idx;
      if (idx > 9)
      {
        do
        {
          idx = (short)(idx - 10);
          r = (short)(r + 1);
        } while (idx > 9);
        sre->idx = idx;
      }
      rem = r;
    }
    out[nout] = filtered;
    nout = (short)(nout + 1);
  }
  sre->rem = (short)rem;
  sre->wpos = (short)wpos;
  return (unsigned short)nout;
}
