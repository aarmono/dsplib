/**
 * @file fax_fifo_sgd.c
 * @brief 16-bit sample FIFO (FIFO_*) and sequence / symbol generator (SGD_*) of the fax modems.
 *
 * Both objects are declared in fax_common.h and are created by the transmitters and receivers (fax_v21.c, fax_v27_tx.c, fax_v29_tx.c, fax_v17_tx.c,
 * fax_v17_rx.c); the only callees are sysdep_malloc() / sysdep_free().
 *
 * FIFO: count / rd / wr / size are 16-bit unsigned; all index arithmetic wraps at 16 bits and every comparison is a 16-bit unsigned one.  FIFO_write() stores
 * at most size - count words; FIFO_read() hands out at most `count` words and fills the rest of the request with the configured `fill` value.
 *
 * SGD: a generator of training / idle symbol streams.  SGD_sequence_gen() emits a table (tab[0 .. len-1]), either repeatedly (cfg.mode != 0) or once followed
 * by a constant (cfg.tailValue); SGD_symbol_gen() emits `bits`-wide slices of a bit pattern, a symbol index counting down from patSyms - 1 to 0 and wrapping.
 * SGD_create() copies the configuration and (when it allocates the object) the history buffer; SGD_control() replaces the sequence part and / or the pattern
 * part of the configuration.  Create and control share sgd_tail(), which clears the history buffer and the detector state.
 */
#include "fax_common_tables.h"
#include "core/dsp_host.h"

/* ------------------------------------------------------------------------------------------------ FIFO */

/**
 * @brief Create or re-initialise a FIFO.
 * @param f    existing FIFO to re-initialise (its buffer pointer must already be valid), or NULL to allocate the object and a buffer of 2 * size bytes
 * @param cfg  { w0, size in words, fill value }; NULL selects the built-in fax_fifo_cfg
 * @return the FIFO
 *
 * The buffer is zeroed in both cases.
 * @warning The allocation results are not checked.  The zeroing loop uses a signed 16-bit index, so `size` above 32767 does not terminate correctly.
 */
FaxFifo* FIFO_create(FaxFifo* f, const FaxFifoCfg* cfg)
{
  unsigned short w0, size, fill;

  if (cfg == NULL)
    cfg = &fax_fifo_cfg;
  w0 = cfg->w0;
  size = cfg->size;
  fill = cfg->fill;
  if (f == NULL)
  {
    f = (FaxFifo*)sysdep_malloc((unsigned int)sizeof(FaxFifo));
    f->buf = (unsigned short*)sysdep_malloc((unsigned int)size * 2u);
  }
  f->w0 = w0;
  f->size = size;
  f->count = 0;
  f->rd = 0;
  f->fill = fill;
  f->wr = 0;
  if (f->size != 0)
  {
    short i = 0;
    unsigned short* b = f->buf;

    do
    {
      b[i] = 0;
      i = (short)(i + 1);
    } while ((int)i < (int)f->size);
  }
  return f;
}

/**
 * @brief Read `n` words from the FIFO.
 * @param f    FIFO
 * @param dst  receives exactly `n` words: min(n, count) stored words, then the `fill` value
 * @param n    number of words requested
 * @return number of stored words delivered (the rest is padding)
 */
int FIFO_read(FaxFifo* f, unsigned short* dst, unsigned short n)
{
  unsigned short size = f->size, k = f->count, rd = f->rd, rest;
  unsigned short* buf = f->buf;

  if (!(k <= n))
    k = n;
  rest = (unsigned short)(n - k);

  /* the stored words, in order, wrapping at `size` */
  {
    unsigned short c = k;

    while (c != 0)
    {
      unsigned short nx;

      *dst++ = buf[rd];
      nx = (unsigned short)(rd + 1);
      rd = (nx < size) ? nx : 0;
      c--;
    }
  }

  /* padding for what is missing */
  while (rest != 0)
  {
    *dst++ = f->fill;
    rest--;
  }
  f->rd = rd;
  f->count = (unsigned short)(f->count - k);
  return k;
}

/**
 * @brief Append up to `n` words to the FIFO.
 * @param f    FIFO
 * @param src  words to store
 * @param n    number of words offered
 * @return number of words stored: min(n, size - count); the rest is dropped
 */
int FIFO_write(FaxFifo* f, const unsigned short* src, unsigned short n)
{
  unsigned short size = f->size, count = f->count, wr = f->wr;
  unsigned short space = (unsigned short)(size - count), k = (n > space) ? space : n;
  unsigned short* buf = f->buf;
  unsigned short c = k;

  while (c != 0)
  {
    unsigned short nx;

    buf[wr] = *src++;
    nx = (unsigned short)(wr + 1);
    wr = (nx < size) ? nx : 0;
    c--;
  }
  f->wr = wr;
  f->count = (unsigned short)(k + count);
  return k;
}

/**
 * @brief Free the FIFO's buffer AND the FIFO object.
 * @warning The object is freed even when the caller supplied it to FIFO_create(); it must have come from sysdep_malloc().
 */
void FIFO_delete(FaxFifo* f)
{
  sysdep_free(f->buf);
  sysdep_free(f);
}

/* ------------------------------------------------------------------------------------------------ SGD */

/** @brief Low 16 bits of a configuration word, unsigned. */
static unsigned short lo16(int w) { return (unsigned short)w; }

/** @brief High 16 bits of a configuration word, signed. */
static short hi16(int w) { return (short)((unsigned)w >> 16); }

/**
 * @brief Common tail of SGD_create() and the pattern part of SGD_control().
 *
 * With cfg.w24 = (b << 16) | a: the history buffer length is bufn = nbuf + a - 1 shorts, which are cleared; the detector state words are cleared; and
 * `scale` becomes bits * a * (0x4000 - b).
 * @warning The cleared length is not checked against the allocation made by SGD_create() (2 * (nbuf + w4) - 2 bytes); `a` must not exceed `w4`.
 */
static void sgd_tail(FaxSgd* o)
{
  unsigned short a = lo16(o->cfg.w24);
  short b = hi16(o->cfg.w24);
  short n = (short)(unsigned short)((unsigned)(unsigned short)o->cfg.nbuf + a - 1u);

  o->bufn = n;
  if (n > 0)
  {
    unsigned short* buf = o->buf;
    unsigned short i = 0;

    do
    {
      buf[i] = 0;
      i = (unsigned short)(i + 1);
    } while ((int)i < (int)o->bufn);
  }
  o->w58 = 0;
  o->w34 = 0;
  o->w38 = 0;
  o->w3c = 0;
  o->w40 = 0;
  o->w44 = 0;
  o->scale = (unsigned short)((unsigned)o->cfg.bits * (unsigned)a * (unsigned)(0x4000 - (int)b));
}

/**
 * @brief Create or re-initialise a sequence / symbol generator.
 * @param o    existing generator to re-initialise (its history buffer is kept), or NULL to allocate the object and its history buffer
 * @param cfg  configuration, copied; NULL selects the built-in SGD_CFG
 * @return the generator, or NULL when the object allocation failed
 *
 * Resets the table position, the pass counter and the pattern symbol index, then runs sgd_tail().
 * @warning The buffer allocation result is not checked.
 */
FaxSgd* SGD_create(FaxSgd* o, const FaxSgdCfg* cfg)
{
  int alloc = 0;

  if (o == NULL)
  {
    o = (FaxSgd*)sysdep_malloc((unsigned int)sizeof(FaxSgd));
    if (o == NULL)
      return NULL;
    alloc = 1;
  }
  if (cfg == NULL)
    cfg = &SGD_CFG;
  o->cfg = *cfg;
  o->pos = 0;
  o->pass = 0;
  o->symbit = (unsigned short)(lo16(o->cfg.patSyms) - 1u);
  if (alloc)
  {
    int n = (int)o->cfg.nbuf + (int)o->cfg.w4;

    o->buf = (unsigned short*)sysdep_malloc((unsigned int)(unsigned short)(2 * n - 2));
  }
  sgd_tail(o);
  return o;
}

/**
 * @brief Replace parts of a generator's configuration.
 * @param o    generator
 * @param ctl  `seq` (if not NULL): new sequence table, length, mode, tail value, pattern and pattern length; restarts the sequence and the pattern;
 *             `pat` (if not NULL): new second table and the words w24 .. w30; clears the history buffer (sgd_tail())
 *
 * The records are read completely before the first store (the stores must not be assumed to alias the loads).
 */
void SGD_control(FaxSgd* o, const FaxSgdCtl* ctl)
{
  const FaxSgdCtlSeq* s = ctl->seq;
  const FaxSgdCtlPat* p = ctl->pat;
  const unsigned short *tab = NULL, *tab2 = NULL;
  int len = 0, mode = 0, tailValue = 0, pattern = 0, patSyms = 0, w24 = 0, w28 = 0, w2c = 0, w30 = 0;

  /* read both records first */
  if (s != NULL)
  {
    tab = s->tab;
    len = s->len;
    mode = s->mode;
    tailValue = s->tailValue;
    pattern = s->pattern;
    patSyms = s->patSyms;
  }
  if (p != NULL)
  {
    tab2 = p->tab2;
    w24 = p->w24;
    w28 = p->w28;
    w2c = p->w2c;
    w30 = p->w30;
  }

  /* then apply them */
  if (s != NULL)
  {
    o->cfg.tab = tab;
    o->cfg.len = len;
    o->cfg.mode = mode;
    o->cfg.tailValue = tailValue;
    o->cfg.pattern = pattern;
    o->cfg.patSyms = patSyms;
    o->pos = 0;
    o->pass = 0;
    o->symbit = (unsigned short)(lo16(patSyms) - 1u);
  }
  if (p != NULL)
  {
    o->cfg.tab2 = tab2;
    o->cfg.w24 = w24;
    o->cfg.w28 = w28;
    o->cfg.w2c = w2c;
    o->cfg.w30 = w30;
    sgd_tail(o);
  }
}

/**
 * @brief Emit `n` words of the configured sequence.
 * @param o    generator; `pos` and `pass` are updated
 * @param out  receives `n` words
 * @param n    number of words
 * @return words left in the table pass (len - pos)
 *
 * With cfg.mode != 0 the table tab[0 .. len-1] repeats and `pass` counts the completed passes.  With mode 0 the table is emitted once, then every word is the
 * constant cfg.tailValue (`pass` set to 1).
 * @warning `n` is trusted (no limit on `out`); `len` must be positive and `tab` must hold `len` words.
 */
short SGD_sequence_gen(FaxSgd* o, unsigned short* out, short n)
{
  const unsigned short* tab = o->cfg.tab;
  int mode = o->cfg.mode;
  short pos = (short)o->pos;
  int tm = (mode != 0) || (o->pass == 0); /* 1: read the table, 0: the constant */
  unsigned short len = lo16(o->cfg.len), cnt = (unsigned short)n;

  while (cnt != 0)
  {
    unsigned short v;

    if (tm)
    {
      v = tab[pos];
      pos = (short)(pos + 1);
    }
    else
    {
      v = lo16(o->cfg.tailValue);
    }
    *out++ = v;

    /* end of the table: wrap (repeating mode) or switch to the constant */
    if (!((short)len > pos))
    {
      if (mode != 0)
      {
        o->pass = (unsigned short)(o->pass + 1);
        pos = 0;
      }
      else
      {
        o->pass = 1;
        tm = 0;
      }
    }
    cnt--;
  }
  o->pos = (unsigned short)pos;
  return (short)((int)len - (int)pos);
}

/**
 * @brief Emit `n` symbols cut from the configured bit pattern.
 * @param o    generator; `symbit` (the next symbol index, counting down) is updated
 * @param out  receives `n` symbols of cfg.bits bits each
 * @param n    number of symbols
 * @return patSyms - 1 minus the next symbol index
 *
 * Symbol j is (pattern >> (j * bits)) & ((1 << bits) - 1); j runs from patSyms - 1 down to 0 and wraps back to patSyms - 1.
 */
short SGD_symbol_gen(FaxSgd* o, unsigned short* out, short n)
{
  unsigned int pat = lo16(o->cfg.pattern), bits = o->cfg.bits;
  unsigned short last = (unsigned short)(lo16(o->cfg.patSyms) - 1u), cnt = (unsigned short)n;
  unsigned int mask = (unsigned short)((1u << (bits & 31u)) - 1u);
  short j = (short)o->symbit;

  while (cnt != 0)
  {
    short nj = (short)(j - 1);

    *out++ = (unsigned short)((pat >> (((unsigned int)(int)j * bits) & 31u)) & mask);
    j = (nj < 0) ? (short)last : nj;
    cnt--;
  }
  o->symbit = (unsigned short)j;
  return (short)((int)last - (int)j);
}

/**
 * @brief Free the generator's history buffer and the object.
 * @warning The object is freed even when the caller supplied it to SGD_create().
 */
void SGD_delete(FaxSgd* o)
{
  sysdep_free(o->buf);
  sysdep_free(o);
}
