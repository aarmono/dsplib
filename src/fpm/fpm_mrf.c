/**
 * @file fpm_mrf.c
 * @brief FPM_MRF: polyphase FIR sample-rate converter (the receive / transmit filter stage of the FSK and fax modems).
 *
 * Owns the FpmMrf state (fpm.h): a circular history of input samples plus a coefficient table with one interleaved set of taps per polyphase branch.
 * FPM_MRF_init() loads the configuration block and (re)allocates the history, FPM_MRF_filter() consumes input samples and produces output samples at the
 * rate (input rate) * nPhases / step, FPM_MRF_free() releases the history.  The object is embedded in the owner's bundle (the V.21 / Bell 103, V.23,
 * V.22 and fax modems and the caller-ID FSK receiver), which is also where the configuration blocks live.
 */

#include "core/dsp_table_handles.h"
#include "fpm.h"
#include "core/dsp_host.h"

/**
 * @brief Load a configuration block and prepare the history buffer.
 * @param mrf    filter state
 * @param cfg    four configuration words: cfg[0] = FpmMrf::a (shorts: [0] number of polyphase branches `nPhases`, [1] branch step per output sample),
 *               cfg[1] = CFGP handle of the coefficient table, cfg[2] / cfg[3] = FpmMrf::b (short [0] = numerator of the history length)
 * @param alloc  non-zero: the object is new, allocate the history; zero: the object exists, keep its buffer if it is long enough, otherwise free it and
 *               allocate a larger one
 * @return the size in bytes of the history buffer allocated by this call (0 when the old buffer was kept)
 *
 * The history length is b.s[0] / a.s[0] samples.  The history is cleared and the read state is reset (phase 1, fraction 0, write position 0).
 * At debug level above 1 a reallocation prints "Reallocate FPM_MRF buffer".
 * @warning The coefficient table must hold `nPhases` * history-length taps.  `a.s[0]` is a divisor: 0 divides by zero.  The sysdep_malloc() result is not
 *          checked; with `alloc` non-zero any previous `buf` pointer is overwritten, not freed.
 */
int FPM_MRF_init(FpmMrf* mrf, const int* cfg, int alloc)
{
  short needed;
  int bufBytes = 0;

  mrf->a.w = cfg[0];
  FPM_TABLE_SET(mrf, coefs, const short*, cfg[1]); /* table pointer: a handle on x86-64 (src/core/dsp_table_handles.h) */
  mrf->b.w[0] = cfg[2];
  mrf->b.w[1] = cfg[3];
  mrf->phase = 1;
  mrf->frac = 0;
  mrf->wpos = 0;

  /* size the history: reuse the old buffer when it is long enough, otherwise (re)allocate */
  needed = mrf->b.s[0] / mrf->a.s[0];
  if (alloc == 0 && mrf->buflen < needed)
  {
    if (1 < dsplibs_debug_level)
    {
      dsplibs_debug_printf("Reallocate FPM_MRF buffer");
    }
    sysdep_free(FPM_M(void*, mrf, buf));
    mrf->buflen = needed;
    bufBytes = needed * 2;
    FPM_M(void*, mrf, buf) = sysdep_malloc((unsigned int)bufBytes);
  }
  else
  {
    mrf->buflen = needed;
    if (alloc != 0)
    {
      bufBytes = needed * 2;
      FPM_M(void*, mrf, buf) = sysdep_malloc((unsigned int)bufBytes);
    }
  }

  /* clear the history */
  if (0 < mrf->buflen)
  {
    short* buf = mrf->buf;
    int i;

    for (i = 0; i < mrf->buflen; i++)
    {
      buf[i] = 0;
    }
  }

  return bufBytes;
}

/**
 * @brief Resample a block: consume `inLeft` input samples, write the output samples that become due.
 * @param mrf     filter state (FPM_MRF_init())
 * @param in      input samples
 * @param out     output samples; room for about inLeft * nPhases / step + 1 entries is needed
 * @param inLeft  number of input samples
 * @return number of output samples written
 *
 * Input samples are pushed into the circular history (`wpos` is the newest).  `phase` counts the input samples still to be taken before the next output.
 * An output is the Q15 dot product, newest sample first, of the history with the taps of polyphase branch `frac` (taps `frac`, `frac + nPhases`, ...);
 * `frac` then advances by the step `a.s[1]`, and every wrap past `nPhases` adds one input sample to the count before the following output.  When the
 * block ends before the next output is due, `phase`, `wpos` and `frac` carry the state to the next call.  The output is the product sum >> 15, truncated
 * to 16 bits (not saturated).
 */
unsigned int FPM_MRF_filter(FpmMrf* mrf, short* in, short* out, short inLeft)
{
  short stepPhases;
  short histLen;
  const short* coefBase;
  short fracNext;
  short pending;
  short untilNext;
  short* hist;
  short nextIdx;
  short nPhases;
  unsigned int wrIdx;
  unsigned int outCount;
  int fracCur;

  outCount = 0;
  nPhases = mrf->a.s[0];
  stepPhases = mrf->a.s[1];
  coefBase = mrf->coefs;
  fracCur = mrf->frac;
  histLen = mrf->buflen;
  hist = mrf->buf;
  untilNext = mrf->phase;
  wrIdx = (unsigned int)(unsigned short)mrf->wpos;
  fracNext = (short)fracCur;

  if (inLeft != 0)
  {
    do
    {
      fracCur = (int)fracNext;
      pending = untilNext;

      /* the block ends before the next output is due: store the remaining samples and save the state */
      if (inLeft < untilNext)
      {
        untilNext = untilNext - inLeft;
        while (1)
        {
          inLeft = inLeft - 1;
          if (inLeft == -1)
            break;
          nextIdx = (short)wrIdx + 1;
          wrIdx = (unsigned int)(unsigned short)(-(unsigned int)(nextIdx < histLen) & (int)nextIdx);
          hist[wrIdx] = *in;
          in = in + 1;
        }
        mrf->phase = untilNext;
        mrf->wpos = (short)wrIdx;
        mrf->frac = fracNext;
        return outCount & 0xffff;
      }

      /* take the samples that precede the next output */
      while (pending != 0)
      {
        nextIdx = (short)wrIdx + 1;
        wrIdx = (unsigned int)(unsigned short)(-(unsigned int)(nextIdx < histLen) & (int)nextIdx);
        hist[wrIdx] = *in;
        pending = pending - 1;
        in = in + 1;
      }
      inLeft = inLeft - untilNext;

      {
        int acc = 0;
        int k;
        const short* psTap = coefBase + fracCur;
        short* psHist = hist + wrIdx;
        int idx;

        /* dot product of the history (newest to oldest, wrapping at the buffer start) with the taps of branch `frac` */
        for (idx = (int)wrIdx; idx >= 0; idx--)
        {
          untilNext = *psHist;
          psHist = psHist - 1;
          acc = acc + (int)untilNext * (int)*psTap;
          psTap = psTap + nPhases;
        }
        psHist = psHist + histLen;
        k = (int)histLen;
        while (k = k - 1, (int)wrIdx < k)
        {
          untilNext = *psHist;
          psHist = psHist - 1;
          acc = acc + (int)untilNext * (int)*psTap;
          psTap = psTap + nPhases;
        }

        /* advance the branch; each wrap past nPhases is one more input sample before the next output */
        untilNext = 0;
        fracCur = fracNext + stepPhases;
        fracNext = (short)fracCur - nPhases;
        while ((short)fracCur >= nPhases)
        {
          untilNext = untilNext + 1;
          fracCur = (short)fracCur - nPhases;
          fracNext = (short)fracCur - nPhases;
        }

        out[outCount] = (short)(acc >> 0xf);
        outCount = (unsigned int)(unsigned short)((short)outCount + 1);
        fracNext = (short)fracCur;
      }
    } while (inLeft != 0);
  }

  mrf->phase = untilNext;
  mrf->wpos = (short)wrIdx;
  mrf->frac = (short)fracCur;
  return outCount & 0xffff;
}

/**
 * @brief Free the history buffer (the object itself belongs to its owner).
 * @param mrf  filter state
 */
void FPM_MRF_free(FpmMrf* mrf) { sysdep_free(FPM_M(void*, mrf, buf)); }
