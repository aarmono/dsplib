/**
 * @file fax_pulse_shaping.c
 * @brief Polyphase pulse-shaping interpolator of the fax transmitters: FAXPPS_init(), FAXPPS_filter(), FAXPPS_free().
 *
 * Data flow: a symbol-mapping encoder (SMC_encoder(), fax_scrambler.c) writes symbol codes, or ready I / Q pairs, into a circular FaxCirc; FAXPPS_filter()
 * reads them one at a time into two short histories (I and Q, `taps` entries, circular) and, for every output sample, evaluates two polyphase FIRs.  The
 * coefficient phase (`acc`) advances by `down + step` per output sample; whenever it reaches `up` the next source symbol is consumed.  The V.27ter, V.29
 * and V.17 transmit code owns the FaxPps objects (fax_v27_tx.c, fax_v29_tx.c, fax_v17_tx.c).  Declarations and the reason for a separate copy of the
 * algorithm are in fax_pulse_shaping.h.
 *
 * Buffer handling: taps = filtlen / up (16-bit division).  Re-initialising an existing object (`fresh == 0`) keeps the history buffers unless `taps` grows,
 * in which case both are freed and reallocated (debug level above 1 prints "Reallocate FPM_PPS buffer"); otherwise they are only zeroed.
 */
#include "fax_pulse_shaping.h"
#include "core/dsp_host.h"

/**
 * @brief Set up a pulse shaper: copy the configuration, reset the phase and zero the histories.
 * @param p      state
 * @param cfg    configuration
 * @param fresh  non-zero for a new object (allocate); 0 to reuse the buffers of a previously initialised one
 * @return 2 * taps when the buffers were allocated here, 0 when they were kept
 * @warning The sysdep_malloc() results are not checked; `cfg->up` must be non-zero.  With `fresh == 0` the object must have been initialised before: `p->taps`
 *          and the history pointers are read.
 */
int FAXPPS_init(FaxPps* p, const FaxPpsCfg* cfg, int fresh)
{
  short taps;
  int ret = 0;
  int i;
  short *hi, *hq;

  p->cfg = *cfg;
  p->need = 0;
  p->acc = p->cfg.down;
  p->pos = 0;
  taps = (short)(p->cfg.filtlen / p->cfg.up);

  /* Re-initialisation reuses the buffers unless the history has to grow; p->taps is garbage in a new object and only read when fresh == 0. */
  if (fresh == 0 && p->taps < taps)
  {
    if (1 < dsplibs_debug_level)
      dsplibs_debug_printf("Reallocate FPM_PPS buffer");
    sysdep_free(p->histQ);
    sysdep_free(p->histI);
    p->taps = taps;
  }
  else
  {
    p->taps = taps;
    if (fresh == 0)
      goto zero_fill;
  }
  p->histI = (short*)sysdep_malloc((unsigned int)(taps * 2));
  ret = p->taps * 2;
  p->histQ = (short*)sysdep_malloc((unsigned int)ret);

zero_fill:
  hi = p->histI;
  hq = p->histQ;
  for (i = 0; i < p->taps; i++)
  {
    hi[i] = 0;
    hq[i] = 0;
  }
  return ret;
}

/**
 * @brief Interpolate a run of symbols into transmit samples.
 * @param p      pulse shaper, initialised by FAXPPS_init()
 * @param src    circular symbol source: `i` / `q` (mode 0) or `buf` (symbol codes, low byte used) indexed by `rd`, wrapping at `lim`; `rd` is advanced
 * @param out    receives the output samples
 * @param count  number of source symbols to consume; the loop produces output samples until that many have been taken
 * @return number of output samples written (16 bit)
 *
 * Per output sample: if the previous phase wrap asked for a symbol (`need`), the next source symbol is stored into both histories at the next write
 * position.  Then each branch computes the sum of history[pos - k] * coef[acc + k * up], walking backwards from the write position with wrap-around; the sums
 * wrap like 32-bit integers.  The output is ((sumI >> 15) - (sumQ >> 15)) * scale >> 15.  Finally the phase advances by `down + step`; reaching `up` subtracts
 * `up` and requests the next symbol.
 * @warning The number of samples written to `out` is not limited by any argument; `src->buf[rd]` indexes `imap` / `qmap` without a range check.
 */
short FAXPPS_filter(FaxPps* p, FaxCirc* src, short* out, unsigned short count)
{
  const short up = p->cfg.up, down = p->cfg.down, step = p->cfg.step;
  const int mode = p->cfg.mode, scale = p->cfg.scale;
  const short *imap = p->cfg.imap, *qmap = p->cfg.qmap, *ifilt = p->cfg.ifilt, *qfilt = p->cfg.qfilt;
  short *histI = p->histI, *histQ = p->histQ;
  const short taps = p->taps;
  const int itaps = (int)taps;
  short acc = p->acc;
  int need = (int)p->need;
  int pos = (int)p->pos;
  const short *srcI = src->i, *srcQ = src->q;
  const unsigned short* srcB = src->buf;
  int rd = (int)src->rd;
  const short lim = src->lim;
  unsigned int left = (unsigned int)count;
  short nout = 0;

  if (left != 0)
  {
    do
    {
      unsigned int accI, accQ; /* the sums wrap like 32-bit registers */
      const short* coefPtr;
      short* histPtr;
      int k, j;

      /* Take the next source symbol into the histories. */
      if (need != 0)
      {
        unsigned int code = (unsigned int)(unsigned char)srcB[rd]; /* the low byte of the entry (little endian) */
        unsigned short valI, valQ;
        short nrd, npos;

        if (mode == 0)
        {
          valI = (unsigned short)srcI[rd];
          valQ = (unsigned short)srcQ[rd];
        }
        else
        {
          valI = (unsigned short)imap[code];
          valQ = (unsigned short)qmap[code];
        }
        nrd = (short)(rd + 1);
        left = (left - (unsigned int)need) & 0xffffu;
        npos = (short)(pos + 1);
        rd = nrd < lim ? (int)nrd : 0;
        pos = npos < taps ? (int)npos : 0;
        histI[pos] = (short)valI;
        histQ[pos] = (short)valQ;
      }

      /* Polyphase FIR on the I history: backwards from the write index, wrapping to the end of the buffer. */
      coefPtr = ifilt + acc;
      histPtr = histI + pos;
      accI = 0;
      for (k = pos; k >= 0; k--)
      {
        accI += (unsigned int)((int)*histPtr * (int)*coefPtr);
        histPtr--;
        coefPtr += up;
      }
      histPtr += itaps;
      for (j = itaps - 1; pos < j; j--)
      {
        accI += (unsigned int)((int)*histPtr * (int)*coefPtr);
        histPtr--;
        coefPtr += up;
      }

      /* The same for the Q history and the Q table. */
      coefPtr = qfilt + acc;
      histPtr = histQ + pos;
      accQ = 0;
      for (k = pos; k >= 0; k--)
      {
        accQ += (unsigned int)((int)*histPtr * (int)*coefPtr);
        histPtr--;
        coefPtr += up;
      }
      histPtr += itaps;
      for (j = itaps - 1; pos < j; j--)
      {
        accQ += (unsigned int)((int)*histPtr * (int)*coefPtr);
        histPtr--;
        coefPtr += up;
      }

      /* Combine the branches and apply the output gain. */
      *out++ = (short)((int)((unsigned int)(int)(short)((short)((int)accI >> 15) - (short)((int)accQ >> 15)) * (unsigned int)scale) >> 15);
      nout++;

      /* Advance the coefficient phase; passing `up` asks for the next symbol. */
      {
        short na = (short)(acc + down + step);

        need = up <= na;
        if (need)
          na = (short)(na - up);
        acc = na;
      }
    } while (left != 0);
  }
  p->need = (short)need;
  p->pos = (short)pos;
  p->acc = acc;
  src->rd = (short)rd;
  return nout;
}

/** @brief Free both history buffers (the FaxPps itself belongs to the caller). */
void FAXPPS_free(FaxPps* p)
{
  sysdep_free(p->histQ);
  sysdep_free(p->histI);
}
