/**
 * @file v22_scrambler.c
 * @brief V.22 / V.22bis scrambler / descrambler (FPM_SDM_*) and differential phase encoder (FPM_SMC_*).
 *
 * The objects are V22Sdm and V22Smc (v22.h); the table pointers of the SMC configuration are real pointers, not handles.
 * - FPM_SDM_init(): n = cfg[0] (bits per symbol), tap1 = cfg[1], tap2 = cfg[2], mask = (1 << n) - 1, nmask = ~mask, sh1 = tap1 - n, sh2 = tap2 - n,
 *   state = 0.  The shift counts are taken modulo 32 (the x86 shift semantics).
 * - FPM_SDM_scrambler(): for each symbol, out = ((state >> sh1) ^ (state >> sh2) ^ buf[i]) & mask is written back and shifted into the register:
 *   state = ((state << n) & nmask) | out.
 * - FPM_SDM_descrambler(): for each symbol, in = buf[i]; buf[i] = ((state >> sh1) ^ (state >> sh2) ^ in) & mask; state = ((state << n) & nmask) | in
 *   (the UNMASKED input is shifted in).
 * - FPM_SMC_init(): the configuration block is copied, a = b = 0.
 * - FPM_SMC_encoder(): see its function documentation.
 */
#include "v22.h"

/** @brief Initialise a scrambler / descrambler from cfg = { bits per symbol, tap1, tap2 }; the shift register is cleared. */
void FPM_SDM_init(V22Sdm* s, const V22SdmCfg* cfg)
{
  short n = cfg->s[0], t1 = cfg->s[1], t2 = cfg->s[2];
  unsigned int mask = (1u << ((unsigned)(int)n & 31)) - 1u;
  s->state = 0;
  s->n = n;
  s->tap1 = t1;
  s->tap2 = t2;
  s->sh2 = (short)((unsigned short)t2 - (unsigned short)n);
  s->mask = mask;
  s->nmask = ~mask;
  s->sh1 = (short)((unsigned short)t1 - (unsigned short)n);
}

/** @brief Scramble `n` symbols (each `s->n` bits wide, one per short) in place. */
void FPM_SDM_scrambler(V22Sdm* s, unsigned short* buf, unsigned short n)
{
  unsigned int st = s->state, mask = s->mask, nmask = s->nmask;
  unsigned int n0 = (unsigned int)(int)s->n & 31, c1 = (unsigned int)(int)s->sh1 & 31, c2 = (unsigned int)(int)s->sh2 & 31;
  while (n != 0)
  {
    unsigned int bits = ((st >> c1) ^ buf[0]) ^ (st >> c2);
    unsigned int o = bits & mask;
    buf[0] = (unsigned short)o;
    st = ((st << n0) & nmask) | (unsigned short)o;
    buf++;
    n--;
  }
  s->state = st;
}

/** @brief Descramble `n` symbols (each `s->n` bits wide, one per short) in place; the stored value is masked to the symbol width. */
void FPM_SDM_descrambler(V22Sdm* s, unsigned short* buf, unsigned short n)
{
  unsigned int st = s->state, mask = s->mask, nmask = s->nmask;
  unsigned int n0 = (unsigned int)(int)s->n & 31, c1 = (unsigned int)(int)s->sh1 & 31, c2 = (unsigned int)(int)s->sh2 & 31;
  while (n != 0)
  {
    unsigned int in = buf[0];
    unsigned int x1 = st >> c1, x2 = st >> c2;
    buf[0] = (unsigned short)(x1 ^ in ^ x2);
    st = ((st << n0) & nmask) | in;
    buf[0] = (unsigned short)(buf[0] & mask);
    buf++;
    n--;
  }
  s->state = st;
}

/** @brief Initialise the encoder: copy the configuration block and clear the running values a and b. */
void FPM_SMC_init(V22Smc* s, const V22SmcCfg* cfg)
{
  s->cfg = *cfg;
  s->a = 0;
  s->b = 0;
}

/**
 * @brief Encode `n` symbols differentially and append them to the ring.
 * @param s   encoder
 * @param o   ring receiving one value per symbol at `buf[wr]` (wr wraps at `lim`)
 * @param in  `n` input symbols
 * @param n   number of symbols
 *
 * For each x = in[i]: with cfg.flag != 0, e = x & mask (no differential table); otherwise e = (a + pmap[(x >> shift) & mask]) & mask3 (a is signed).
 * Then a = (short)e; t = ((x & mask2) | e) + b (16 bit); t >= mod: t -= mod; the ring value is t; then b += step (short), b >= mod: b -= mod.
 */
void FPM_SMC_encoder(V22Smc* s, V22Circ* o, const unsigned short* in, unsigned short n)
{
  unsigned int mask3 = s->cfg.mask3, mask = s->cfg.mask, mask2 = s->cfg.mask2, sh = s->cfg.shift & 31;
  int mod = s->cfg.mod, step = s->cfg.step, flag = s->cfg.flag;
  int a = s->a, b = s->b;
  const unsigned short* pmap = (const unsigned short*)s->cfg.pmap;
  int idx = o->wr, lim = (short)o->lim;
  unsigned short* buf = o->buf;

  while (n != 0)
  {
    unsigned int x = *in++, e, comb;
    int t;
    if (flag != 0)
    {
      e = mask & x;
      comb = (x & mask2) | e;
    }
    else
    {
      unsigned int tv = pmap[(x >> sh) & mask];
      e = ((unsigned int)a + tv) & mask3;
      comb = (x & mask2) | e;
    }
    a = (short)e;
    t = (int)(unsigned short)((unsigned int)b + comb);
    if (t >= mod)
      t = (unsigned short)(t - mod);
    buf[idx] = (unsigned short)t;
    b = (short)(b + step);
    if (b >= mod)
      b = (short)(b - mod);
    {
      int i1 = (short)(idx + 1);
      idx = (i1 < lim) ? i1 : 0;
    }
    n--;
  }
  s->b = (short)b;
  s->a = (short)a;
  o->wr = (short)idx;
}
