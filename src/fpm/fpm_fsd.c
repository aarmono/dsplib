/**
 * @file fpm_fsd.c
 * @brief FPM_FSD: FSK demodulator (delay-and-multiply discriminator, IIR low-pass and bit-timing recovery).
 *
 * Owns the FpmFsd state (fpm.h) and three entry points: FPM_FSD_init() loads a configuration block and allocates the work buffers,
 * FPM_FSD_demodulate() turns a block of line samples into bits, FPM_FSD_free() releases the buffers.  Used by the Bell 103 / V.21 and V.23 receivers
 * and the fax V.21 receiver, behind the polyphase filter of fpm_mrf.c.
 *
 * Per input sample the demodulator (1) pushes the sample into a circular history, (2) runs the history through an FIR (taps `intrp`), (3) multiplies the
 * result by the raw input delayed by `alignOff` samples, (4) low-passes the product with the IIR cascade `coefs` (FPM_iir_filt(), fpm_iir.c) and
 * (5) slices the result against +-threshold into a bit and recovers the bit timing from the run lengths.
 *
 * Configuration words (FpmFsd members, each a 32-bit word of the configuration block holding two shorts, low short first):
 *   cfg[0] intrp table handle;  cfg[1] c1 = {FIR tap count, delay alignOff};  cfg[2] coefs table handle;  cfg[3] c3 = {IIR section count, slicer threshold};
 *   cfg[4] c4 = {bit polarity, samples per bit};  cfg[5] c5 = {per-call bit limit, length of the out[] buffer};  cfg[6] stored in w6 (not read here).
 */

#include <stdint.h>
#include "core/dsp_table_handles.h"
#include "fpm.h"
#include "core/dsp_host.h"

/**
 * @brief Load a configuration block, allocate (optionally) and clear the work buffers.
 * @param obj    demodulator
 * @param cfg    seven configuration words, see the file header
 * @param alloc  non-zero: allocate the history (c1.s[0] shorts), the IIR state (2 * c3.s[0] shorts) and `out` (c5.s[1] shorts); zero: keep the existing
 *               buffers, which must be at least as large for the new configuration
 *
 * Clears the history, the IIR state and `out`, and resets the slicer state (`prevBit`, `bitClock`, `mismatchRun`) and the history position.
 * @warning The sysdep_malloc() results are not checked; with `alloc` non-zero the old buffer pointers are overwritten, not freed.
 */
void FPM_FSD_init(FpmFsd* obj, const int* cfg, int alloc)
{
  FpmFsd* b = obj;
  FpmFsd* o32 = obj;
  short bound2;

  FPM_TABLE_SET(o32, intrp, const short*, cfg[0]); /* table pointers: handles on x86-64 (src/core/dsp_table_handles.h) */
  o32->c1.w = cfg[1];
  FPM_TABLE_SET(o32, coefs, const short*, cfg[2]);
  o32->c3.w = cfg[3];
  o32->c4.w = cfg[4];
  o32->c5.w = cfg[5];
  b->histPos = 0;
  o32->w6 = cfg[6];
  b->prevBit = 0;
  b->bitClock = 0;
  b->mismatchRun = 0;

  /* work buffers */
  if (alloc != 0)
  {
    FPM_M(void*, b, hist) = sysdep_malloc((unsigned int)(b->c1.s[0] * 2));
    FPM_M(void*, b, state) = sysdep_malloc((unsigned int)((int)b->c3.s[0] << 2));
    FPM_M(void*, b, out) = sysdep_malloc((unsigned int)(FPM_M(unsigned short, b, c5.s[1]) * 2));
  }

  /* clear the FIR history */
  if (0 < b->c1.s[0])
  {
    short* buf = b->hist;
    int i = 0;

    do
    {
      buf[i] = 0;
      i = i + 1;
    } while ((short)i < b->c1.s[0]);
  }

  /* clear the IIR state */
  bound2 = b->c3.s[0];
  if ((short)(bound2 * -2) < 0)
  {
    short* buf = b->state;
    int limit = (int)bound2 * 2;
    int i = 0;

    do
    {
      buf[i] = 0;
      i = i + 1;
    } while (i < limit);
  }

  /* clear the output buffer */
  b->outCount = 0;
  b->s22 = (short)(b->c4.s[1] >> 1);
  if (0 < b->c5.s[1])
  {
    short* buf = b->out;
    int i = 0;

    do
    {
      buf[i] = 0;
      i = i + 1;
    } while ((short)i < b->c5.s[1]);
  }
}

/**
 * @brief Demodulate a block of samples into bits.
 * @param obj        demodulator (FPM_FSD_init())
 * @param inSamples  line samples (typically the output of FPM_MRF_filter())
 * @param bitsOut    decided bits, one per 16-bit word (0 or 1); room for c5.s[0] + 2 entries is needed
 * @param count      number of input samples; must not exceed the `out` buffer length c5.s[1]
 * @return number of bits written to `bitsOut`
 *
 * Discriminator: the FIR output (acc >> 15) times the input delayed by `alignOff` samples (>> 13) goes through the IIR cascade; the value is stored in
 * `out[]` (one entry per input sample, left there for the caller).
 *
 * Slicer: a value above the threshold is the bit `c4.s[0]` (the polarity), one below minus the threshold the opposite bit, anything in between keeps the
 * last decided bit (`prevBit`).  `bitClock` counts the samples since the last emitted bit.  When the sliced bit equals `prevBit` and `bitClock` reaches
 * the samples-per-bit value `c4.s[1]` that bit is emitted.  Each sample that disagrees with `prevBit` is counted in `mismatchRun`; when the count
 * reaches half the samples per bit (3 for 5) the new bit is emitted, becomes `prevBit`, and the timing restarts (bit-clock resynchronisation on a
 * transition).  `mismatchRun` is only cleared when a bit is emitted.  The call stops early once more than c5.s[0] + 1 bits have been produced.
 * @bug   The emission test for an unchanged bit is an equality (`bitClock == c4.s[1]`).  If the sample that brings `bitClock` to that value disagrees with
 *        `prevBit`, the test is missed, and while the signal then keeps agreeing with `prevBit` no bit is emitted until `mismatchRun` reaches its limit
 *        (it needs disagreeing samples) or `bitClock` wraps after 65536 samples.
 * @warning More than c5.s[1] input samples overrun `out`.
 */
unsigned int FPM_FSD_demodulate(FpmFsd* obj, unsigned short* inSamples, short* bitsOut, short count)
{
  FpmFsd* b = obj;
  short alignOff;
  short histLen;
  short sections;
  short threshold;
  unsigned short parity;
  unsigned short lastBit;
  unsigned short oversample;
  const short* intrpTaps;
  const short* coefs;
  short* state;
  short scratch16;
  unsigned short sampleCnt;
  unsigned short newBit;
  unsigned short mismatchLimit;
  short left;
  short* outp;
  unsigned int histIdx;
  short* hist;
  unsigned int nBits;

  nBits = 0;
  alignOff = b->c1.s[1];
  left = count - 1;
  intrpTaps = b->intrp; /* table pointer: a handle on x86-64 (src/core/dsp_table_handles.h) */
  histLen = b->c1.s[0];
  sections = b->c3.s[0];
  coefs = b->coefs;
  threshold = b->c3.s[1];
  parity = FPM_M(unsigned short, b, c4.s[0]);
  hist = b->hist;
  state = b->state;
  histIdx = (unsigned int)(unsigned short)b->histPos;
  outp = b->out;

  if (count != 0)
  {
    do
    {
      unsigned int outIdx;
      int acc;
      short* pHist;
      const short* pInterp;
      int idx;
      short sample;

      /* push the sample into the circular history */
      scratch16 = (short)histIdx + 1;
      sample = (short)*inSamples;
      inSamples = inSamples + 1;
      histIdx = (unsigned int)(unsigned short)(-(unsigned int)(scratch16 < histLen) & (int)scratch16);
      hist[(int)histIdx] = sample;

      /* FIR over the history, newest sample first */
      acc = 0;
      pHist = hist + (int)histIdx;
      pInterp = intrpTaps;
      for (idx = (int)histIdx; idx >= 0; idx--)
      {
        scratch16 = *pHist;
        pHist = pHist - 1;
        acc = acc + (int)scratch16 * (int)*pInterp;
        pInterp = pInterp + 1;
      }
      {
        int k = (int)histLen;

        pHist = pHist + histLen;
        while (k = k - 1, (int)histIdx < k)
        {
          scratch16 = *pHist;
          pHist = pHist - 1;
          acc = acc + (int)scratch16 * (int)*pInterp;
          pInterp = pInterp + 1;
        }
      }

      /* multiply by the delayed input and low-pass */
      scratch16 = (short)histIdx - alignOff;
      if (scratch16 < 0)
      {
        scratch16 = scratch16 + histLen;
      }
      scratch16 = FPM_iir_filt((short)((int)(short)(acc >> 0xf) * (int)hist[(int)scratch16] >> 0xd), coefs, state, sections);
      *outp = scratch16;
      outp = outp + 1;

      /* slice */
      lastBit = FPM_M(unsigned short, b, prevBit);
      newBit = lastBit;
      if ((int)scratch16 + (int)threshold < 0)
      {
        newBit = (unsigned short)(1 - parity);
      }
      if (threshold < scratch16)
      {
        newBit = parity;
      }
      outIdx = nBits & 0xffff;
      newBit = newBit & 1;

      /* bit timing */
      oversample = FPM_M(unsigned short, b, c4.s[1]);
      mismatchLimit = oversample >> 1;
      if (oversample == 5)
      {
        mismatchLimit = 3;
      }
      sampleCnt = (unsigned short)(b->bitClock + 1);
      FPM_M(unsigned short, b, bitClock) = sampleCnt;
      if (lastBit == newBit)
      {
        if (oversample == sampleCnt)
        {
          nBits = (unsigned int)(unsigned short)((short)nBits + 1);
          *(unsigned short*)((intptr_t)bitsOut + (int)outIdx * 2) = lastBit;
          b->mismatchRun = 0;
          b->bitClock = 0;
        }
      }
      else
      {
        scratch16 = b->mismatchRun;
        FPM_M(unsigned short, b, mismatchRun) = (unsigned short)(scratch16 + 1);
        if ((unsigned short)(scratch16 + 1) == mismatchLimit)
        {
          nBits = (unsigned int)(unsigned short)((short)nBits + 1);
          FPM_M(unsigned short, b, prevBit) = newBit;
          *(unsigned short*)((intptr_t)bitsOut + (int)outIdx * 2) = newBit;
          b->mismatchRun = 0;
          b->bitClock = 0;
        }
      }

      /* stop when the per-call bit limit is exceeded or the block is done */
      nBits = (unsigned int)(unsigned short)nBits;
      if (!((int)nBits <= b->c5.s[0] + 1))
        break;
      if (left == 0)
        break;
      left = left - 1;
    } while (1);
  }

  b->histPos = (short)histIdx;
  b->outCount = count;
  return nBits & 0xffff;
}

/**
 * @brief Free the work buffers (`out`, IIR state, history); the object itself belongs to its owner.
 * @param fsd  demodulator
 */
void FPM_FSD_free(FpmFsd* fsd)
{
  sysdep_free(FPM_M(void*, fsd, out));
  sysdep_free(FPM_M(void*, fsd, state));
  sysdep_free(FPM_M(void*, fsd, hist));
}
