/**
 * @file fax_scrambler.c
 * @brief Scramblers, descramblers and symbol-mapping encoders of the fax modems (V.17, V.27ter, V.29).
 *
 * Three families, all integer-only and without callees; the state objects (FaxSdm, FaxSdmV27, FaxSmc, FaxSmcV17, FaxCirc) are declared in fax_common.h:
 * - SDM_*: the self-synchronising scrambler of V.17 / V.29 (the same machine as the V.22 one).  `n` data bits are processed per call element; the state is the
 *   shift register; mask = (1 << n) - 1; the two feedback taps are applied as shifts sh1 = tap1 - n and sh2 = tap2 - n.
 *   Scrambler: d = ((st >> sh1) ^ (st >> sh2) ^ in) & mask; out = d; st = ((st << n) & ~mask) | d.
 *   Descrambler: out = ((st >> sh1) ^ (st >> sh2) ^ in) & mask; st = ((st << n) & ~mask) | in (the UNMASKED input word).
 *   Shift counts use the low 5 bits.
 * - SDMv27_*: the V.27ter (4800 / 2400 bit/s) scrambler over a 16-bit register with feedback taps at bits 6 and 7.  A run counter `cnt` tracks how long
 *   the register bits 8, 9 and 12 have NOT all differed from the scrambled bit; when a symbol could push it to 0x21 (cnt >= 0x21 - n), or a guard flag
 *   f1 / f2 is set, the symbol is processed bit by bit with the guard that breaks a repeating pattern (f1 / f2 feed an extra inversion into the
 *   feedback).  f1 / f2 stay 0 on the normal path.
 * - SMC_*, SMCv17_*: symbol-mapping encoders.  SMC_encoder() adds differential coding and writes either a phase index or a rotated I / Q pair
 *   (carrier phase from cos / sin tables) into a circular descriptor; the V.17 encoders (_abs absolute phase, _dif differential, _tcm trellis coded)
 *   write 2-bit phase codes or trellis-coded symbol codes and use the trellis tables of the V.32 modem (v32/v32_tables.h).
 *
 * The encoders feed FAXPPS_filter() (fax_pulse_shaping.c) through the FaxCirc; the (de)scramblers are called through the per-modem wrappers of
 * fax_modem_wrappers.c and the transmit / receive engines.
 */
#include "fax_common_tables.h"
#include "v32/v32_tables.h"

/* ------------------------------------------------------------------------------------------------ SDM */

/**
 * @brief Set up a V.17 / V.29 scrambler state from its configuration.
 * @param s    state; the shift register is cleared
 * @param cfg  { bits per symbol, tap 1, tap 2 }, e.g. { 4, 5, 23 }
 */
void SDM_init(FaxSdm* s, const FaxSdmCfg* cfg)
{
  unsigned int mask;
  short n = cfg->s[0], t1 = cfg->s[1], t2 = cfg->s[2];

  s->state = 0;
  s->n = n;
  s->tap1 = t1;
  s->tap2 = t2;
  mask = (1u << ((unsigned)(int)n & 31)) - 1u;
  s->sh2 = (short)((unsigned short)t2 - (unsigned short)n);
  s->mask = mask;
  s->nmask = ~mask;
  s->sh1 = (short)((unsigned short)t1 - (unsigned short)n);
}

/**
 * @brief Scramble `n` symbols in place.
 * @param s    scrambler state, updated
 * @param buf  one symbol (n bits, in the low bits) per element, replaced by the scrambled symbol
 * @param n    number of elements
 */
void SDM_scrambler(FaxSdm* s, unsigned short* buf, unsigned short n)
{
  unsigned int st = s->state, mask = s->mask, nmask = s->nmask;
  unsigned int n0 = (unsigned int)(int)s->n & 31, c1 = (unsigned int)(int)s->sh1 & 31, c2 = (unsigned int)(int)s->sh2 & 31;

  while (n != 0)
  {
    unsigned int o = (((st >> c1) ^ (st >> c2)) ^ buf[0]) & mask;

    buf[0] = (unsigned short)o;
    st = ((st << n0) & nmask) | (unsigned short)o;
    buf++;
    n--;
  }
  s->state = st;
}

/**
 * @brief Descramble `n` symbols in place (self-synchronising: the register is fed with the received, unmasked words).
 * @param s    descrambler state, updated
 * @param buf  one scrambled symbol per element, replaced by the descrambled symbol (masked to n bits)
 * @param n    number of elements
 */
void SDM_descrambler(FaxSdm* s, unsigned short* buf, unsigned short n)
{
  unsigned int st = s->state, mask = s->mask, nmask = s->nmask;
  unsigned int n0 = (unsigned int)(int)s->n & 31, c1 = (unsigned int)(int)s->sh1 & 31, c2 = (unsigned int)(int)s->sh2 & 31;

  while (n != 0)
  {
    unsigned int in = buf[0];
    unsigned int x = (unsigned short)((st >> c1) ^ in ^ (st >> c2));

    st = ((st << n0) & nmask) | in;
    buf[0] = (unsigned short)(x & mask);
    buf++;
    n--;
  }
  s->state = st;
}

/* ------------------------------------------------------------------------------------------------ SDMv27 */

/**
 * @brief Set up a V.27ter scrambler state.
 * @param s    state; register 0x3c, run counter and guard flags cleared
 * @param cfg  bits per symbol (2 selects the 2400 bit/s masks, anything else 3); NULL selects the built-in SDMv27_CFG
 */
void SDMv27_init(FaxSdmV27* s, const FaxSdmCfg27* cfg)
{
  unsigned short n;

  if (cfg == NULL)
    cfg = &SDMv27_CFG;
  n = cfg->n;
  s->n = n;
  s->reg = 0x3c;
  s->cnt = 0;
  s->f1 = 0;
  s->f2 = 0;
  if (n == 2)
  {
    s->mask = 3;
    s->nmask = 0xfffc;
  }
  else
  {
    s->mask = 7;
    s->nmask = 0xfff8;
  }
}

/**
 * @brief Update the run counter with the n bits of `x`, from bit n-1 down to bit 0.
 *
 * A set bit continues the run (cnt + 1), a clear bit restarts it at 0.
 */
static unsigned short v27_cnt(unsigned short cnt, unsigned int n, unsigned int x)
{
  short j = (short)(unsigned short)(n - 1u);

  while (j >= 0)
  {
    unsigned int bit = (x >> ((unsigned int)(int)j & 31u)) & 1u;

    cnt = (unsigned short)((unsigned short)(cnt + bit) * bit);
    j = (short)(j - 1);
  }
  return cnt;
}

/**
 * @brief Scramble `count` V.27ter symbols in place.
 * @param s      state, updated
 * @param buf    one symbol (2 or 3 bits) per element, replaced by the scrambled symbol
 * @param count  number of elements
 *
 * Normal path (run counter below 0x21 - n and no guard flag): the symbol is XORed with the register taps and appended to the register.  Otherwise the symbol
 * is processed bit by bit, most significant bit first; a run of 0x21 sets f1 (and clears the counter), and the flag is fed into the next bit's feedback.
 */
void SDMv27_scrambler(FaxSdmV27* s, unsigned short* buf, short count)
{
  unsigned int n = s->n, mask = s->mask, nmask = s->nmask, reg = s->reg;
  unsigned int cnt = s->cnt, f1 = s->f1, f2 = s->f2;
  unsigned int lim = (unsigned short)(0x21u - n);
  unsigned short k = (unsigned short)count;

  while (k != 0)
  {
    if ((unsigned short)cnt < lim && (f1 + f2) == 0)
    {
      unsigned int t = (reg << (n & 31)) & nmask;
      unsigned int d = (*buf ^ ((t >> 6) ^ (t >> 7))) & mask;
      unsigned int x;

      reg = t | d;
      *buf++ = (unsigned short)d;
      x = ~(((reg >> 8) ^ d) & ((reg >> 9) ^ d) & ((reg >> 12) ^ d)) & 0xffffu;
      cnt = v27_cnt((unsigned short)cnt, n, x);
    }
    else
    {
      short bit = (short)n;

      while (bit > 0)
      {
        unsigned int sreg, fb, in, d, m, match, e;

        f2 = f1;
        sreg = (reg * 2u) & 0xffffu;
        fb = (sreg >> 6) ^ (sreg >> 7) ^ f2;
        in = *buf;
        d = ((in >> ((n - 1u) & 31u)) ^ fb) & 1u;
        *buf = (unsigned short)((in << 1) | d);
        reg = sreg | d;
        m = ((d << 16) - d) ^ (sreg | d);
        match = (m & 0x1300u) == 0x1300u;
        e = ~(match | f2) & 1u;
        cnt = (unsigned short)((unsigned short)(cnt + e) * e);
        f1 = (unsigned short)(e * f2);
        if ((unsigned short)cnt == 0x21)
        {
          cnt = 0;
          f1 = 1;
        }
        bit = (short)(bit - 1);
      }
      *buf = (unsigned short)(*buf & mask);
      buf++;
    }
    k--;
  }
  s->reg = (unsigned short)reg;
  s->cnt = (unsigned short)cnt;
  s->f1 = (unsigned short)f1;
  s->f2 = (unsigned short)f2;
}

/**
 * @brief Descramble `count` V.27ter symbols in place; the inverse of SDMv27_scrambler() (the register is fed with the received bits).
 * @param s      state, updated
 * @param buf    one scrambled symbol (2 or 3 bits) per element, replaced by the descrambled symbol
 * @param count  number of elements
 */
void SDMv27_descrambler(FaxSdmV27* s, unsigned short* buf, short count)
{
  unsigned int n = s->n, mask = s->mask, nmask = s->nmask, reg = s->reg;
  unsigned int cnt = s->cnt, f1 = s->f1, f2 = s->f2;
  unsigned int lim = (unsigned short)(0x21u - n);
  unsigned short k = (unsigned short)count;

  while (k != 0)
  {
    if ((unsigned short)cnt < lim && (f1 + f2) == 0)
    {
      unsigned int r16, t, inm, x;

      reg <<= (n & 31);
      r16 = reg & 0xffffu;
      t = r16 & nmask;
      inm = *buf & mask;
      reg = t | inm;
      *buf++ = (unsigned short)((((r16 >> 6) ^ (r16 >> 7)) ^ inm) & mask);
      x = ~(((reg >> 8) ^ inm) & ((reg >> 9) ^ inm) & ((reg >> 12) ^ inm)) & 0xffffu;
      cnt = v27_cnt((unsigned short)cnt, n, x);
    }
    else
    {
      short bit = (short)n;

      while (bit > 0)
      {
        unsigned int sreg, fbp, in2, b, m, match, e, ob;

        sreg = (reg * 2u) & 0xffffu;
        fbp = (sreg >> 6) ^ (sreg >> 7);
        in2 = (unsigned int)*buf * 2u;
        b = (((in2 & 0xfffeu) >> (n & 31u)) & 1u);
        reg = sreg | b;
        m = ((b << 16) - b) ^ reg;
        match = (m & 0x1300u) == 0x1300u;
        e = ~(match | f2) & 1u;
        ob = ((fbp ^ f2) & 1u) ^ b;
        *buf = (unsigned short)(ob | in2);
        cnt = (unsigned short)((unsigned short)(cnt + e) * e);
        f1 = (unsigned short)(e * f1);
        if ((unsigned short)cnt == 0x21)
        {
          cnt = 0;
          f1 = 1;
        }
        f2 = f1;
        bit = (short)(bit - 1);
      }
      *buf = (unsigned short)(*buf & mask);
      buf++;
    }
    k--;
  }
  s->reg = (unsigned short)reg;
  s->cnt = (unsigned short)cnt;
  s->f1 = (unsigned short)f1;
  s->f2 = (unsigned short)f2;
}

/* ------------------------------------------------------------------------------------------------ SMC */

/**
 * @brief Set up a symbol-mapping encoder: copy the configuration and clear the differential value and carrier phase.
 * @param s    encoder state
 * @param cfg  configuration (its table pointers are kept, not copied)
 */
void SMC_init(FaxSmc* s, const FaxSmcCfg* cfg)
{
  s->cfg = *cfg;
  s->a = 0;
  s->b = 0;
}

/**
 * @brief Map `n` scrambled data words to symbols and append them to a circular descriptor.
 * @param s   encoder state (running differential value `a` and carrier phase index `b`), updated
 * @param o   output descriptor; entries are written at `wr`, which wraps at `lim` and is stored back
 * @param in  `n` input words
 * @param n   number of words
 *
 * Per word: the differential part e is the masked input (cfg.flag != 0) or pmap[(in >> shift) & mask] + a, masked with mask3; the symbol index is
 * comb = (in & mask2) | e, and `a` becomes e.  With cfg.phaseOut != 0 the symbol is a phase index, (b + comb) reduced once by `mod`, stored in o->buf.
 * Otherwise imap[comb] / qmap[comb] are rotated by the carrier phase cosv[b] / sinv[b] (Q15) and stored in o->i / o->q.  `b` then advances by `step`
 * modulo `mod`.
 * @warning `imap` / `qmap` / `pmap` are indexed without a range check; the output buffers must hold `lim` entries.
 */
void SMC_encoder(FaxSmc* s, FaxCirc* o, const unsigned short* in, unsigned short n)
{
  const FaxSmcCfg* c = &s->cfg;
  int step = c->step, mod = c->mod;
  unsigned int mask = c->mask, mask2 = c->mask2, mask3 = c->mask3, sh = c->shift & 31u;
  int w0 = c->phaseOut, flag = c->flag;
  const unsigned short* pmap = c->pmap;
  const short *imap = c->imap, *qmap = c->qmap, *cosv = c->cosv, *sinv = c->sinv;
  int a = s->a, b = s->b;
  int wr = o->wr, lim = o->lim;
  unsigned short* buf = o->buf;
  short *oi = o->i, *oq = o->q;
  unsigned short cnt = n;

  while (cnt != 0)
  {
    unsigned int x = *in++, e, comb;

    /* differential coding of the quadrant bits */
    if (flag != 0)
    {
      e = x & mask;
    }
    else
    {
      unsigned int idx = ((unsigned int)(int)(unsigned short)x >> sh) & mask;

      e = ((unsigned int)pmap[idx] + (unsigned int)a) & mask3;
    }
    comb = (x & mask2) | e;
    a = (short)e;

    /* output: phase index, or the symbol rotated by the carrier phase */
    if (w0 != 0)
    {
      int t = (int)(unsigned short)((unsigned int)b + comb);

      if (t >= mod)
        t = (unsigned short)(t - mod);
      buf[wr] = (unsigned short)t;
    }
    else
    {
      int c4 = cosv[b], c5 = sinv[b], im = imap[comb], qm = qmap[comb];

      oi[wr] = (short)((im * c4 >> 15) - (qm * c5 >> 15));
      oq[wr] = (short)((qm * c4 >> 15) + (im * c5 >> 15));
    }

    /* advance the carrier phase and the write index */
    b = (short)(b + step);
    if (b >= mod)
      b = (short)(b - mod);
    {
      int w1 = (short)(wr + 1);

      wr = (w1 < lim) ? w1 : 0;
    }
    cnt--;
  }
  s->b = (short)b;
  s->a = (short)a;
  o->wr = (short)wr;
}

/* ------------------------------------------------------------------------------------------------ SMCv17 */

/**
 * @brief Set up a V.17 symbol-mapping encoder state.
 * @param s    state; phase, differential and trellis state cleared
 * @param cfg  { tcmMask, w2 }; NULL selects the built-in SMCv17_CFG
 */
void SMCv17_init(FaxSmcV17* s, const FaxSmcV17Cfg* cfg)
{
  unsigned short w0, w2;

  if (cfg == NULL)
    cfg = &SMCv17_CFG;
  w0 = cfg->tcmMask;
  w2 = cfg->w2;
  s->tcmMask = w0;
  s->w2 = w2;
  s->y = 0;
  s->x = 0;
  s->st = 0;
  s->y2 = 0;
  s->w10 = 0;
}

/**
 * @brief V.17 encoder, absolute phase: out = (x + ABS4[in & 3]) & 3 where x steps back by one quadrant per symbol.
 * @param s   state (x updated)
 * @param o   output descriptor; the phase codes go to o->buf at `wr` (wrapping at `lim`)
 * @param in  `n` input words (2 bits used)
 * @param n   number of words
 */
void SMCv17_encoder_abs(FaxSmcV17* s, FaxCirc* o, const unsigned short* in, unsigned short n)
{
  unsigned short* buf = o->buf;
  int x = s->x, lim = o->lim, wr = o->wr;
  unsigned short cnt = n;

  while (cnt != 0)
  {
    unsigned int a = *in++ & 3u;

    x = (x + 3) & 3;
    buf[wr] = (unsigned short)((x + (int)SMCv17_ABS4[a]) & 3);
    wr = (short)(wr + 1);
    if (!(wr < lim))
      wr = 0;
    cnt--;
  }
  s->x = (short)x;
  o->wr = (short)wr;
}

/**
 * @brief V.17 encoder, differential phase: y accumulates PMAP4[in & 3] (mod 4) and the output is (y + x + 1) & 3, x stepping back by one quadrant per symbol.
 * @param s   state (x and y updated)
 * @param o   output descriptor; the phase codes go to o->buf
 * @param in  `n` input words (2 bits used)
 * @param n   number of words
 */
void SMCv17_encoder_dif(FaxSmcV17* s, FaxCirc* o, const unsigned short* in, unsigned short n)
{
  unsigned short* buf = o->buf;
  int y = s->y, x = s->x, lim = o->lim, wr = o->wr;
  unsigned short cnt = n;

  while (cnt != 0)
  {
    unsigned int a = *in++ & 3u;

    x = (x + 3) & 3;
    y = (y + (int)SMCv17_PMAP4[a]) & 3;
    buf[wr] = (unsigned short)((y + x + 1) & 3);
    wr = (short)(wr + 1);
    if (!(wr < lim))
      wr = 0;
    cnt--;
  }
  s->y = (short)y;
  s->x = (short)x;
  o->wr = (short)wr;
}

/**
 * @brief V.17 trellis-coded encoder.
 * @param s   state (x, y2 and st updated); `k` is the number of symbol bits below the two trellis bits
 * @param o   output descriptor; the symbol codes go to o->buf
 * @param in  `n` input words (k + 2 bits used); each word is replaced by itself masked to k + 2 bits
 * @param n   number of words
 *
 * Per word: the top two of the k + 2 bits (q) and the encoder state y select the new differential state through TrellisEncodeDifTable; the transition state
 * `st` then selects the next one through TrellisTransitionTable.  The value is the new y over the low k bits, with one extra bit when st > 3; it is mapped
 * through SMCv17_MOD, which is read as a 4-bit entry chosen by the phase x, and ORed with the signed low byte of tcmMask shifted left by 8.
 * @warning `SMCv17_MOD` is indexed by the value without a range check.
 */
void SMCv17_encoder_tcm(FaxSmcV17* s, FaxCirc* o, unsigned short* in, unsigned short n)
{
  unsigned int k = s->k;
  int st = s->st, y = s->y2, x = s->x;
  unsigned int m1 = (unsigned short)((1u << (k & 31)) - 1u);
  unsigned int sz = (unsigned short)(1u << ((k + 2u) & 31));
  unsigned int mk = (unsigned short)(sz - 1u);
  int b0 = (int)(unsigned int)((signed char)(s->tcmMask & 0xffu)) * 256;
  unsigned short* buf = o->buf;
  int lim = o->lim, wr = o->wr;
  unsigned short cnt = n;

  while (cnt != 0)
  {
    unsigned int w = *in & mk, q, val, sh, mv, r, outv;
    int ynew, snew;

    *in++ = (unsigned short)w;
    q = w >> (k & 31);
    ynew = (short)TrellisEncodeDifTable[y + (int)q * 4];
    val = (unsigned short)(((unsigned int)ynew << (k & 31)) + (w & m1));
    if ((short)st > 3)
      val = (unsigned short)(val + sz);
    x = (x + 3) & 3;
    snew = (short)TrellisTransitionTable[ynew + st * 4];
    sh = (unsigned short)(val >> (k & 31));
    mv = SMCv17_MOD[sh];
    r = ((int)mv >> (((unsigned int)x * 4u) & 31u)) & 7;
    outv = ((r << (k & 31)) + (val & m1)) | (unsigned int)b0;
    buf[wr] = (unsigned short)outv;
    wr = (short)(wr + 1);
    if (!(wr < lim))
      wr = 0;
    y = ynew;
    st = snew;
    cnt--;
  }
  s->x = (short)x;
  s->y2 = (short)y;
  s->st = (short)st;
  o->wr = (short)wr;
}
