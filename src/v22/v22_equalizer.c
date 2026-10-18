/**
 * @file v22_equalizer.c
 * @brief V.22 / V.22bis FSE: fractionally spaced equaliser, carrier phase loop and per-symbol decision.
 *
 * Entry points: V22_FSE_init(), V22_FSE_free(), V22_FSE_getdiag(), V22_FSE_receive(), and the two decision callbacks FSEv22_decision12 (1200 bit/s,
 * four-point constellation) and FSEv22_decision24 (2400 bit/s, sixteen-point).  The object is V22Fse (v22.h); every 32-bit product that can overflow is
 * computed in unsigned arithmetic (the MUL / ADD macros) so that it wraps.
 *
 * V22_FSE_receive() takes the timing recovery's output samples (the SRE output, v22_timing.c), produces one symbol every 6 samples and per symbol:
 * runs the 49-tap complex equaliser over the sample history (taps tapsI / tapsQ, history `hist`), gets the amplitude and angle of the equalised point,
 * rotates it by the carrier phase estimate, hands the point to the decision callback (which returns the decoded bits and the ideal point's angle and
 * magnitude), updates the carrier phase loop (a proportional + integral loop with three gain sets, the first settling symbols with the integrator held
 * at 0) from the angle error, the averaged squared error `errPow`, and, while adaptation is enabled, adapts the taps (LMS) with the error vector.
 * `errPow` is the signal quality figure (GetSignalQuality()).
 */
#include <stddef.h>
#include "v22.h"
#include "v22_tables.h"
#include "core/dsp_host.h"
#include "fpm/fpm.h"

/* unsigned-wrap helpers: 32-bit multiplication and addition that wrap (C signed overflow is undefined) */
#define MUL(a, b) ((int)((unsigned int)(int)(a) * (unsigned int)(int)(b)))
#define ADD(a, b) ((int)((unsigned int)(int)(a) + (unsigned int)(int)(b)))

/**
 * @brief Initialise the equaliser from its configuration.
 * @param f         object
 * @param cfg       the two coefficient sources (49 shorts each); the pointers are stored in f->srcI / srcQ
 * @param allocate  non-zero: allocate the seven buffers (taps 0x62 bytes each, history 0xc4, two unused 0x28, the two output buffers 0x1c); zero: keep the
 *                  existing ones (a re-initialisation)
 *
 * The taps are the coefficient sources >> 2 stored in REVERSE order; the history is cleared; the gates are set (pllEnable, s18, adaptEnable = 1), the
 * adaptation step index and the loop state are cleared, `rem` = 1.  The decision callback and the SMC state word are not set here: the creator does that.
 * @warning The allocation results are not checked.
 */
void V22_FSE_init(V22Fse* f, const V22FseCfg* cfg, int allocate)
{
  short *tI, *tQ, *h;
  const short *srcI = cfg->srcI, *srcQ = cfg->srcQ; /* read before the 16-bit stores: plain and -fno-strict-aliasing code stay the same */
  int k;

  f->s10 = 0;
  f->pllEnable = 1;
  f->s18 = 1;
  f->adaptEnable = 1;
  f->srcI = srcI;
  f->srcQ = srcQ;
  f->mu = 0;
  f->gainSel = 0;
  f->integ = 0;
  f->errAvg = 0;
  f->errPow = 0;
  f->count = 0;
  f->nOut = 0;
  f->wpos = 0;
  f->carrier = 0;
  f->phase = 0;
  f->s42 = 0;
  f->pllCnt = 0;
  f->rem = 1;
  if (allocate != 0)
  {
    f->tapsI = (short*)sysdep_malloc(0x62);
    f->tapsQ = (short*)sysdep_malloc(0x62);
    f->hist = (short*)sysdep_malloc(0xc4);
    f->buf44 = sysdep_malloc(0x28);
    f->buf48 = sysdep_malloc(0x28);
    f->outI = (short*)sysdep_malloc(0x1c);
    f->outQ = (short*)sysdep_malloc(0x1c);
  }
  tI = f->tapsI;
  tQ = f->tapsQ;
  h = f->hist;
  for (k = 0; k <= 0x30; k++)
  {
    tI[0x30 - k] = (short)(srcI[k] >> 2);
    tQ[0x30 - k] = (short)(srcQ[k] >> 2);
    h[k] = 0;
  }
  for (k = 0; k <= 0x61; k++)
    h[k] = 0;
}

/** @brief Free the seven buffers that V22_FSE_init() allocated. */
void V22_FSE_free(V22Fse* f)
{
  sysdep_free(f->outQ);
  sysdep_free(f->outI);
  sysdep_free(f->buf48);
  sysdep_free(f->buf44);
  sysdep_free(f->hist);
  sysdep_free(f->tapsQ);
  sysdep_free(f->tapsI);
}

/** @brief Diagnostics hook of the equaliser; does nothing and returns 0. */
int V22_FSE_getdiag(V22Fse* fse, int mode, int* out, int max)
{
  (void)fse;
  (void)mode;
  (void)out;
  (void)max;
  return 0;
}

/**
 * @brief Equalise and decide the symbols contained in `count` input samples.
 * @param f        object
 * @param in       `count` samples from the timing recovery
 * @param outBits  receives the decoded bits of each symbol (the decision callback's result), one short per symbol
 * @param count    number of samples; a call with fewer samples than are missing for the next symbol only stores them (and stores nothing for count <= 0)
 * @return the number of symbols produced by this call (zero-extended 16 bits); the equalised points of the symbols are in f->outI / outQ
 *
 * One symbol is produced every 6 samples (`rem` counts the samples still missing, `phase` the position within the 6-sample symbol clock).
 * @warning f->outI / outQ hold only 14 symbols, and `outBits` is not bounded: a call must not produce more than 14 symbols, i.e. count <= 84
 *          (docs/bugs.md).  The engine's 160-sample block gives at most 12.
 * @bug     The SMC state word f->smc points at (V22Core::fseSmc) is never initialised by the creator and is not reset by ResetRx(): the decision
 *          callbacks decode the first symbol after V22FP_create() differentially against an indeterminate value (the engine block is not cleared on
 *          allocation), so the first decoded symbol can be wrong (read from the code; not run).
 */
int V22_FSE_receive(V22Fse* f, const short* in, short* outBits, short count)
{
  short* tapsI = f->tapsI;
  short* tapsQ = f->tapsQ;
  short* hist = f->hist;
  int wpos = f->wpos;
  int phase = f->phase;
  short* oI = f->outI;
  short* oQ = f->outQ;
  int rem = f->rem;
  int cnt = count;
  short ph[4];
  unsigned short ang;
  const short* base = hist;

  f->count = count;
  f->nOut = 0;
  ph[3] = 0;

  while (cnt != 0)
  {
    int oldrem, clk, k, sumI, sumQ, errC, amp;
    short pI, pQ, s5, cbAngle, cbMag, s, e;
    unsigned int bits;
    short eI, eQ;

    /* not enough samples for the next symbol: store them and leave */
    if ((short)cnt < (short)rem)
    {
      if ((short)cnt > 0)
      {
        int next;
        rem = (short)(rem - cnt);
        next = wpos + cnt;
        if (next > 0x62)
        {
          sysdep_memcpy(hist, hist + 0x31, 0x62);
          wpos = (short)(wpos - 0x31);
          next = wpos + cnt;
        }
        sysdep_memcpy(hist + wpos, in, (size_t)cnt * 2);
        wpos = (short)next;
        phase = (short)(cnt * 2 + phase);
        while ((short)phase > 5)
          phase = (short)(phase - 6);
      }
      f->phase = (short)phase;
      f->rem = (short)rem;
      f->wpos = (short)wpos;
      return (unsigned short)f->nOut;
    }

    /* take in the samples that complete the symbol (the history moves down when it would overflow) and advance the symbol clock */
    {
      int next = wpos + rem;
      if (next > 0x62)
      {
        sysdep_memcpy(hist, hist + 0x31, 0x62);
        wpos = (short)(wpos - 0x31);
        next = wpos + rem;
      }
      sysdep_memcpy(hist + wpos, in, (size_t)rem * 2);
      in += rem;
      wpos = (short)next;
      phase = (short)(rem * 2 + phase);
      while ((short)phase > 5)
        phase = (short)(phase - 6);
    }
    oldrem = rem;
    rem = 6;
    clk = (short)(CRRv22_CLK[phase] + (unsigned short)f->carrier);
    cnt = (short)(cnt - oldrem);
    base = (wpos > 0x30) ? hist + (wpos - 0x31) : hist;

    /* the equaliser: 49-tap complex filter over the newest samples; the point's angle and amplitude */
    sumI = 0;
    for (k = 0; k <= 0x30; k++)
      sumI = ADD(sumI, MUL(tapsI[k], base[k]));
    pI = (short)(MUL((short)(sumI >> 15), 0x8908) >> 14);
    sumQ = 0;
    for (k = 0; k <= 0x30; k++)
      sumQ = ADD(sumQ, MUL(tapsQ[k], base[k]));
    pQ = (short)(MUL((short)(sumQ >> 15), 0x8908) >> 14);
    FPM_atan(pQ, pI, &ang);
    ph[0] = (short)ang;
    FPM_phasor(ph);
    amp = (short)(ADD(MUL(ph[1], pI), MUL(ph[2], pQ)) >> 15);

    /* remove the carrier phase (the angle is halved to keep it within 16 bits) and form the equalised point */
    s5 = (short)(((int)(short)ang - clk) >> 1);
    if (s5 < 0)
      s5 = (short)(s5 + 0x4000);
    if (s5 > 0x4000)
      s5 = (short)(s5 - 0x4000);
    ph[3] = 0;
    ph[0] = (short)(s5 * 2);
    FPM_phasor(ph);
    *oI++ = (short)(MUL(ph[1], amp) >> 13);
    *oQ++ = (short)(MUL(amp, ph[2]) >> 13);

    /* decision: bits and the ideal point */
    cbAngle = 0;
    cbMag = 0;
    bits = f->decide(f, &cbAngle, &cbMag);
    *outBits++ = (short)bits;
    f->nOut = (short)(f->nOut + 1);

    /* phase error against the ideal point, averaged; carrier loop update (gain set chosen by the settling counter and the error level) */
    s = (short)((short)(s5 * 2) - (unsigned short)cbAngle);
    if (s < -0x4000)
      s = (short)(s - 0x8000);
    if (s > 0x4000)
      s = (short)(s - 0x8000);
    e = s;
    errC = (short)(ADD(MUL(f->errAvg, 0x799a) >> 15, MUL(e, 0x666) >> 15));
    f->errAvg = (short)errC;
    if (f->pllEnable != 0)
    {
      int a, p3e, gs;
      short cn = f->pllCnt;
      if (cn <= 0x30)
      {
        f->integ = 0;
        f->pllCnt = (short)(cn + 1);
        f->gainSel = (short)(cn <= 0x2f ? 0 : 1);
      }
      else
      {
        a = (short)errC;
        a = (short)((a ^ (a >> 31)) - (a >> 31));
        if ((short)a > 0x1999)
          f->gainSel = 1;
        if ((short)a <= 0x666)
          f->gainSel = 2;
      }
      gs = f->gainSel;
      p3e = ADD(ADD(MUL(CRRv22_PLL_K1[gs], e) >> 15, (unsigned short)f->carrier), (unsigned short)f->integ);
      f->carrier = (short)p3e;
      f->integ = (short)ADD(MUL(CRRv22_PLL_K2[gs], e) >> 15, (unsigned short)f->integ);
      {
        int w = (short)p3e;
        if ((short)w < -0x4000)
          w = (short)(w - 0x8000);
        if ((short)w > 0x4000)
          w = (short)(w - 0x8000);
        f->carrier = (short)w;
      }
    }

    /* error vector between the ideal point (rotated back by the carrier) and the equalised point; averaged squared error; tap adaptation */
    {
      short v = (short)(((int)cbAngle + clk) >> 1);
      if (v < 0)
        v = (short)(v + 0x4000);
      if (v > 0x4000)
        v = (short)(v - 0x4000);
      ph[0] = (short)(v * 2);
    }
    FPM_phasor(ph);
    eI = (short)((MUL(ph[1], cbMag) >> 14) - pI);
    eQ = (short)((MUL(cbMag, ph[2]) >> 14) - pQ);
    f->errPow = (short)ADD(MUL((short)(ADD(MUL(eI, eI), MUL(eQ, eQ)) >> 15), 0x666) >> 15, MUL(f->errPow, 0x799a) >> 15);
    if ((short)f->errPow <= 0 || f->adaptEnable == 0)
      continue;
    {
      int mu = v22_fse_mu_tab[f->mu];
      short s1 = (short)(ADD(MUL(eI, mu), 0x800) >> 12);
      short s2 = (short)(ADD(MUL(eQ, mu), 0x800) >> 12);
      for (k = 0; k <= 0x30; k++)
      {
        int d = base[k];
        tapsI[k] = (short)(ADD(MUL(s1, d), 0x40000) >> 19) + tapsI[k];
        tapsQ[k] = (short)(ADD(MUL(s2, d), 0x40000) >> 19) + tapsQ[k];
      }
    }
  }
  f->phase = (short)phase;
  f->rem = (short)rem;
  f->wpos = (short)wpos;
  return (unsigned short)f->nOut;
}

/* Match the decided point to the SMC symbol table (imap / qmap) and decode the bits differentially: idx is the index of the first entry whose (I, Q) / 2 equal
 * the point (0 when none matches).  The quadrant bits (idx & 0xc) are decoded against the previous quadrant kept in *f->smc (which is updated) through
 * SMCv22_PMAP: at 1200 bit/s the result is the dibit PMAP[q >> 2] >> 2; at 2400 bit/s (is24) it is PMAP[q >> 2] | (idx & 3), the quadbit. */
static unsigned int smc_bits(V22Fse* f, const short* imap, const short* qmap, short I, short Q, int is24)
{
  int idx = 0, i;
  unsigned int prev, q;
  for (i = 0; i <= 0xf; i++)
  {
    if ((short)(imap[i] >> 1) == I && (short)(qmap[i] >> 1) == Q)
    {
      idx = i;
      break;
    }
  }
  prev = *f->smc;
  *f->smc = (unsigned short)(idx & 0xc);
  q = (unsigned int)((idx & 0xc) - prev) & 0xf;
  if (is24)
    return (unsigned int)(unsigned short)((unsigned short)SMCv22_PMAP[q >> 2] | (unsigned int)(idx & 3));
  return (unsigned int)(unsigned short)(SMCv22_PMAP[q >> 2] >> 2);
}

/**
 * @brief Decision for 1200 bit/s: pick the nearest of the four constellation points.
 * @param f      object; the equalised point of the current symbol is f->outI / outQ[f->nOut]
 * @param angle  receives the ideal point's angle (DECv22_ANGL12)
 * @param mag    receives the ideal point's magnitude (0x3299)
 * @return the decoded dibit (see smc_bits())
 */
unsigned int FSEv22_decision12(V22Fse* f, short* angle, short* mag)
{
  int i, best = 0;
  short bestd = 0x7fff, I, Q, d;
  int n = f->nOut;
  short rI = f->outI[n], rQ = f->outQ[n];
  for (i = 0; i < 4; i++)
  {
    int dI = (short)(rI - DECv22_IMAP12[i]);
    int dQ = (short)(rQ - DECv22_QMAP12[i]);
    d = (short)(ADD(MUL(dI, dI) >> 16, MUL(dQ, dQ) >> 16));
    if (d < bestd)
    {
      bestd = d;
      best = i;
    }
  }
  I = DECv22_IMAP12[best];
  Q = DECv22_QMAP12[best];
  *angle = DECv22_ANGL12[best];
  *mag = 0x3299;
  return smc_bits(f, SMCv22_IMAP_1200BPS, SMCv22_QMAP_1200BPS, I, Q, 0);
}

/**
 * @brief Decision for 2400 bit/s: pick the nearest of the sixteen constellation points.
 * @param f      object; the equalised point of the current symbol is f->outI / outQ[f->nOut]
 * @param angle  receives the ideal point's angle (DECv22_ANGL24)
 * @param mag    receives the ideal point's magnitude (DECv22_MAG24 by the ring of the point)
 * @return the decoded quadbit (see smc_bits())
 *
 * The signs of the point select a group of four table entries and the position against +-0x2000 narrows it to two, of which the one nearer in Q wins.
 */
unsigned int FSEv22_decision24(V22Fse* f, short* angle, short* mag)
{
  int n = (short)f->nOut;
  short rI = f->outI[n], rQ = f->outQ[n];
  int base = 8, i, best = 0;
  short sign = 1, bestd = 0x2000, I, Q, lim, sum;
  if (rI < 0)
  {
    base = 0;
    sign = -1;
  }
  if (rQ < 0)
    base = (short)(base + 4);
  lim = (short)(2 * sign * 0x1000);
  if (lim <= rI)
    base = (short)(base + 2);
  /* the threshold (+-0x2000) is compared with I as a 16-bit value */
  for (i = base; i < (short)(base + 2); i = (short)(i + 1))
  {
    short t = DECv22_QMAP24[i];
    short dd = (rQ - t < 0) ? (short)(t - rQ) : (short)(rQ - t);
    if (dd < bestd)
    {
      bestd = dd;
      best = i;
    }
  }
  Q = DECv22_QMAP24[best];
  I = DECv22_IMAP24[best];
  {
    int aI = I >> 1, aQ = Q >> 1;
    sum = (short)(((aI ^ (aI >> 31)) - (aI >> 31)) + ((aQ ^ (aQ >> 31)) - (aQ >> 31)));
  }
  *mag = DECv22_MAG24[(sum >> 12) - 1];
  *angle = DECv22_ANGL24[best];
  return smc_bits(f, SMCv22_IMAP_2400BPS, SMCv22_QMAP_2400BPS, I, Q, 1);
}
