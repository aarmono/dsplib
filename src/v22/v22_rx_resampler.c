/**
 * @file v22_rx_resampler.c
 * @brief V.22 / V.22bis MRF: the receive sample-rate converter, a 30-tap polyphase FIR with 9 phases.
 *
 * Entry points: V22_MRF_init(), V22_MRF_free(), V22_MRF_filter().  The object is V22Mrf (v22.h).  The ratio is 9 / 20: 20 input samples are consumed
 * per 9 output samples (the phase advances by 20 per output and wraps modulo 9; every wrap consumes one more input sample), so the 160-sample engine
 * block gives 72 samples.
 */
#include <stddef.h>
#include "v22.h"
#include "core/dsp_host.h"

/**
 * @brief Initialise the converter.
 * @param m         object
 * @param cfg       configuration (copied); coefs is the engine's 270-short coefficient buffer, filled by the creator
 * @param allocate  non-zero: allocate the history (0x78 bytes); zero: keep the existing one
 *
 * Sets step 2, phase 0, wpos 0, len 0x1e and clears the first `len` history entries.  The coefficient buffer is REORDERED IN PLACE into the polyphase
 * layout: for e = 0x105 .. 0x10d the entries e, e - 9, e - 18, ... >= 0 follow each other (30 per phase, 9 phases).
 * @warning The allocation result is not checked.
 */
void V22_MRF_init(V22Mrf* m, const V22MrfCfg* cfg, int allocate)
{
  short tmp[0x10e];
  short *c = cfg->coefs, *buf; /* cached: the code does not depend on the aliasing rules */
  int e, j, k, n = 0;

  m->cfg = *cfg;
  m->step = 2;
  m->phase = 0;
  m->wpos = 0;
  m->len = 0x1e;
  if (allocate != 0)
  {
    buf = (short*)sysdep_malloc(0x78);
    m->buf = buf;
  }
  else
  {
    buf = m->buf;
  }
  for (k = 0; k < 0x1e; k++)
    buf[k] = 0;
  for (e = 0x105; (short)e <= 0x10d; e++)
  {
    for (j = e; (short)j >= 0; j = (short)(j - 9))
      tmp[n++] = c[j];
  }
  for (k = 0; k <= 0x10d; k++)
    c[k] = tmp[k];
}

/** @brief Free the history allocated by V22_MRF_init(). */
void V22_MRF_free(V22Mrf* m) { sysdep_free(m->buf); }

/**
 * @brief Convert one block of samples.
 * @param m      object
 * @param in     `count` input samples
 * @param out    receives the output samples (at most about 9 / 20 of the input count, plus one)
 * @param count  number of input samples
 * @return the number of output samples (16 bit)
 *
 * While input remains: if fewer samples are left than `step` (the samples needed for the next output) they are only stored (step shrinks); otherwise `step`
 * samples are stored (the 60-sample history moves down by 0x1e shorts when it would overflow) and, once the history holds 30 samples, one output is the dot
 * product of the newest 30 samples with the coefficient row `phase`, >> 15.  Then phase += 20 and `step` becomes the number of 9s subtracted until
 * phase <= 8.
 * @warning `out` is not bounded; the engine passes a 170-short buffer for at most 160 input samples.
 */
unsigned int V22_MRF_filter(V22Mrf* m, const short* in, short* out, short count)
{
  int n = count, w = m->wpos, step = m->step, ph = m->phase, len = m->len;
  const short* coefs = m->cfg.coefs;
  short* buf = m->buf;
  unsigned int cnt = 0;

  while (n != 0)
  {
    int w2;
    if (n < step)
    {
      int nstep = (short)(step - n);
      w2 = w + n;
      if (w2 > 0x3c)
      {
        sysdep_memcpy(buf, buf + 0x1e, 0x3c);
        w = (short)(w - 0x1e);
        w2 = w + n;
      }
      sysdep_memcpy(buf + w, in, (size_t)(n * 2));
      m->step = (short)nstep;
      m->phase = (short)ph;
      m->wpos = (short)w2;
      return cnt;
    }
    w2 = w + step;
    if (w2 > 0x3c)
    {
      sysdep_memcpy(buf, buf + 0x1e, 0x3c);
      w = (short)(w - 0x1e);
      w2 = w + step;
    }
    sysdep_memcpy(buf + w, in, (size_t)(step * 2));
    in += step;
    w = (short)w2;
    n = (short)(n - step);
    {
      const short* cp = coefs + len * ph;
      const short* bp = (w >= len) ? buf + (w - len) : buf;
      unsigned int acc = 0;
      int j;
      for (j = 0; j <= 0x1d; j++)
        acc += (unsigned int)((int)bp[j] * (int)cp[j]);
      out[cnt] = (short)((int)acc >> 15);
      cnt = (unsigned short)(cnt + 1);
    }
    ph = (short)(ph + 20);
    step = 0;
    while (ph > 8)
    {
      ph = (short)(ph - 9);
      step = (short)(step + 1);
    }
  }
  m->phase = (short)ph;
  m->step = (short)step;
  m->wpos = (short)w;
  return cnt;
}
