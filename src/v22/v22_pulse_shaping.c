/**
 * @file v22_pulse_shaping.c
 * @brief V.22 / V.22bis PPS: the transmit pulse-shaping interpolator.
 *
 * Entry points: V22_PPS_init(), V22_PPS_free(), V22_PPS_filter().  The object is V22Pps (v22.h).  The filter turns the symbols in the circular ring
 * (V22Circ, filled by FPM_SMC_encoder) into audio samples with a polyphase FIR of 40 phases and 3 taps per phase.  The phase advances by 3 per output
 * sample plus the clock adjustment cfg.c0.s[0] (TxClockSync()), so a symbol lasts 40 / 3 = 13.33 samples (600 symbols/s at 8000 Hz) and the 12 symbols
 * of an engine block give 160 samples.  Each output is the difference of the I and Q branch sums scaled by 4.
 */
#include <stddef.h>
#include "v22.h"
#include "core/dsp_host.h"

/**
 * @brief Initialise the pulse shaper.
 * @param p         object
 * @param cfg       configuration block (copied as a whole); coefI / coefQ are the engine's 120-short coefficient buffers
 * @param allocate  non-zero: allocate the two histories (0xc bytes and taps * 4); zero: keep the existing ones
 *
 * Sets flag 0, phase 3, fill 0, taps 3 and clears the first `taps` history entries.  The two coefficient buffers are REORDERED IN PLACE into the
 * polyphase layout: for i = 0x50 .. 0x77 the entries i, i - 0x28, i - 0x50 follow each other (3 per phase, 40 phases, 120 entries).
 * @warning The allocation results are not checked.
 */
void V22_PPS_init(V22Pps* p, const V22PpsCfg* cfg, int allocate)
{
  short t1[0x78], t2[0x78];
  short *c1 = cfg->coefI, *c2 = cfg->coefQ, *hI, *hQ; /* cached: the code does not depend on the aliasing rules */
  int i, j, k, n = 0;

  p->cfg = *cfg;
  p->flag = 0;
  p->phase = 3;
  p->fill = 0;
  p->taps = 3;
  if (allocate != 0)
  {
    hI = (short*)sysdep_malloc(0xc);
    p->histI = hI;
    hQ = (short*)sysdep_malloc((unsigned int)((int)p->taps << 2));
    p->histQ = hQ;
  }
  else
  {
    hI = p->histI;
    hQ = p->histQ;
  }
  for (k = 0; k < 3; k++)
  {
    hI[k] = 0;
    hQ[k] = 0;
  }
  for (i = 0x50; (short)i <= 0x77; i++)
  {
    for (j = i; (short)j >= 0; j = (short)(j - 0x28))
    {
      t1[n] = c1[j];
      t2[n] = c2[j];
      n = (short)(n + 1);
    }
  }
  for (k = 0; k <= 0x77; k++)
  {
    c1[k] = t1[k];
    c2[k] = t2[k];
  }
}

/** @brief Free the two histories allocated by V22_PPS_init(). */
void V22_PPS_free(V22Pps* p)
{
  sysdep_free(p->histQ);
  sysdep_free(p->histI);
}

/**
 * @brief Consume `n` symbols from the ring and produce the matching output samples.
 * @param p    object
 * @param src  symbol ring; the symbol index is the low byte of `buf[rd]`, `rd` advances (wrapping at `lim`)
 * @param out  receives the output samples
 * @param n    number of symbols to consume
 * @return the number of output samples (16 bit)
 *
 * While symbols remain: when `flag` is set a symbol is consumed first (I / Q = imap / qmap[symbol]; the histories shift down by 3 when the 6th entry
 * would be written).  With fill > 2 one output is ((sum histI[fill-2+j] * coefI[phase*taps + j]) >> 15) - ((sum histQ[..] * coefQ[..]) >> 15), scaled by 4;
 * then phase += c0.s[0] + 3 and a phase of 0x28 or more subtracts 0x28 and sets `flag`.
 * @bug With fill <= 2 (only for the first symbols after the init) the sum runs over histI[0..2] with the coefficient row at index `phase` instead of
 *      `phase * taps`, so those first outputs use a different row than the steady state (no memory error: phase < 0x28).
 * @warning The symbol index (low byte of the ring entry) is not range-checked against the 16-entry maps; the encoder writes values below 16.
 */
unsigned int V22_PPS_filter(V22Pps* p, V22Circ* src, short* out, unsigned short n)
{
  int flag = p->flag, ph = p->phase, fill = p->fill, taps = p->taps;
  int rd = src->rd, lim = src->lim;
  const unsigned short* sbuf = src->buf;
  const short *imap = p->imap, *qmap = p->qmap;
  const short *c1 = p->cfg.coefI, *c2 = p->cfg.coefQ;
  short *hI = p->histI, *hQ = p->histQ;
  unsigned int step0 = p->cfg.c0.s[0];
  unsigned int count = 0;
  int nn = n;

  for (;;)
  {
    int j;
    unsigned int a1, a2;
    if (nn == 0)
      break;
    if (flag != 0)
    {
      int sym = (unsigned char)sbuf[rd];
      short vi, vq;
      nn = (unsigned short)(nn - flag);
      vi = imap[sym];
      vq = qmap[sym];
      rd = (short)(rd + 1);
      if (!(rd < lim))
        rd = 0;
      fill = (short)(fill + 1);
      if (fill > 5)
      {
        sysdep_memcpy(hI, hI + 3, 6);
        sysdep_memcpy(hQ, hQ + 3, 6);
        fill = (short)(fill - 3);
      }
      hI[fill] = vi;
      hQ[fill] = vq;
    }
    a1 = 0;
    a2 = 0;
    if (fill > 2)
    {
      const short *r1 = c1 + ph * taps, *r2 = c2 + ph * taps;
      const short *x = hI + (fill - 2), *y = hQ + (fill - 2);
      for (j = 0; j <= 2; j++)
      {
        a1 += (unsigned int)((int)x[j] * (int)r1[j]);
        a2 += (unsigned int)((int)y[j] * (int)r2[j]);
      }
    }
    else
    {
      const short *r1 = c1 + ph, *r2 = c2 + ph;
      for (j = 0; j <= 2; j++)
      {
        a1 += (unsigned int)((int)hI[j] * (int)r1[j]);
        a2 += (unsigned int)((int)hQ[j] * (int)r2[j]);
      }
    }
    {
      int d = ((int)a1 >> 15) - ((int)a2 >> 15);
      *out++ = (short)((unsigned int)(int)(short)d << 2);
    }
    count = (unsigned short)(count + 1);
    ph = (short)(ph + (int)step0 + 3);
    if (ph <= 0x27)
    {
      flag = 0;
    }
    else
    {
      ph = (short)(ph - 0x28);
      flag = 1;
      if (nn == 0)
        break;
    }
  }
  p->flag = (short)flag;
  p->fill = (short)fill;
  p->phase = (short)ph;
  src->rd = (short)rd;
  return count;
}
