/**
 * @file v22_detectors.c
 * @brief The V.22 / V.22bis handshake detectors: Detect_Retrain(), Detect_Rmloop2_ACK(), Detect_1s(), Detect_v22() / Detect_v22_core().
 *
 * The first three work on a block of received symbols (the output of DemodDataV22 / the descrambler, one symbol per short) and return a "score": for a
 * positive decision (n * 0x6aab) >> 14 as a 16-bit value, i.e. about 1.67 per symbol (20, the block duration in ms, for the engine's 12-symbol block),
 * else 0.  The handlers accumulate these scores in tones.sigAcc / onesAcc and compare the sums with thresholds (for example 0xe6 = 230 ms).  All the
 * running sums are 16-bit values.  Detect_v22() works on the raw input block and tells whether the answer tone is present.
 */
#include "v22.h"
#include "core/dsp_constants.h" /* RATE_* */
#include "core/dsp_host.h"
#include "fpm/fpm.h"

/* |2 * s2 - s1| as a 32-bit absolute value (the callers compare it as a 16-bit value) */
static unsigned int absdiff16(unsigned int s2, unsigned int s1)
{
  int d = (int)(2u * s2 - s1);
  int a = d < 0 ? -d : d;
  return (unsigned int)a;
}

/**
 * @brief Detect the 2400 bit/s retrain request pattern in a block of received symbols.
 * @param data  received symbols (the dibit in bits 3..2 of each short is used)
 * @param pn    pointer to the number of symbols
 * @return the score (see the file header) when the pattern is present, else 0
 *
 * With v = (data[i] >> 2) & 3: s1 = sum of v^2 over all symbols, s2 = sum of 3 * v over the even-indexed symbols, c9 = 9 per even index, and `run` = the
 * number of consecutive even indices i >= 2 (with i + 1 < n) where data[i] == data[i - 2] and data[i - 1] == data[i + 1] (any miss resets it to 0).
 * The pattern is present when |2 * s2 - s1| >= (c9 * 0x7eb8 >> 15) (compared as 16-bit values) and run > 2.
 */
unsigned int Detect_Retrain(const unsigned short* data, const unsigned short* pn)
{
  unsigned int n = *pn;
  int s1 = 0, s2 = 0, c9 = 0, run = 0, i;

  for (i = 0; i < (int)n; i = (short)(i + 1))
  {
    unsigned int v = (data[i] >> 2) & 3u;
    s1 = (short)((unsigned int)s1 + v * v);
  }
  for (i = 0; i < (int)n;)
  {
    unsigned int b = data[i];
    unsigned int v = (b >> 2) & 3u;
    s2 = (short)((unsigned int)s2 + v * 3u);
    c9 = (short)(c9 + 9);
    if (i != 0 && i + 1 < (int)n)
    {
      if ((unsigned short)b == data[i - 2] && data[i - 1] == data[i + 1])
        run = (short)(run + 1);
      else
        run = 0;
    }
    i = (short)(i + 2);
  }
  {
    int thr = (c9 * 0x7eb8) >> 15;
    unsigned int a = absdiff16((unsigned int)s2, (unsigned int)s1);
    if ((short)a >= (short)thr && (short)run > 2)
      return (unsigned short)(((int)n * 0x6aab) >> 14);
  }
  return 0;
}

/**
 * @brief Detect the remote-digital-loopback acknowledge pattern: a constant symbol value.
 * @param data  received symbols
 * @param pn    pointer to the number of symbols
 * @param mode  line rate: 2400 (reference value K = 10) or anything else (K = 2)
 * @return the score when the symbols are close to the constant K, else 0
 *
 * s1 = sum of data^2, s2 = sum of K * data, s3 = sum of K^2; since 2 * s2 - s1 = s3 - sum of (data - K)^2, the test |2 * s2 - s1| >= (s3 * 0x7eb8 >> 15)
 * (compared as 16-bit values) means that the error energy is below about 4 % of the reference energy (for 4-bit symbols the other solution of the absolute
 * value, an error energy above 196 %, cannot occur).
 */
unsigned int Detect_Rmloop2_ACK(const unsigned short* data, const unsigned short* pn, short mode)
{
  unsigned int n = *pn;
  unsigned int k = (mode == RATE_2400) ? 10u : 2u;
  unsigned int k2 = k * k;
  int s1 = 0, s2 = 0, s3 = 0, i;

  for (i = 0; i < (int)n; i = (short)(i + 1))
    s1 = (short)((unsigned int)s1 + (unsigned int)data[i] * (unsigned int)data[i]);
  for (i = 0; i < (int)n; i = (short)(i + 1))
  {
    s2 = (short)((unsigned int)s2 + (unsigned int)data[i] * k);
    s3 = (short)((unsigned int)s3 + k2);
  }
  {
    unsigned int a = absdiff16((unsigned int)s2, (unsigned int)s1);
    int thr = (s3 * 0x7eb8) >> 15;
    if ((short)a >= (short)thr)
      return (unsigned short)(((int)n * 0x6aab) >> 14);
  }
  return 0;
}

/**
 * @brief Detect "ones": a block of constant symbols (after descrambling, the idle / handshake ones of the protocol).
 * @param data    received symbols
 * @param pn      pointer to the number of symbols
 * @param mode    line rate: 2400 (reference value K = 15) or anything else (K = 3)
 * @param factor  Q15 threshold factor (0x6ccc: error energy below about 15 % of the reference; 0x7eb8: below about 4 %)
 * @return the score when the symbols are close to the constant K, else 0
 *
 * The same statistic as Detect_Rmloop2_ACK() (|2 * s2 - s1| compared as a 16-bit value) against s3 * factor >> 15, with a strict comparison.
 */
unsigned int Detect_1s(const unsigned short* data, const unsigned short* pn, short mode, short factor)
{
  unsigned int n = *pn;
  unsigned int k = (mode == RATE_2400) ? 15u : 3u;
  unsigned int k2 = k * k;
  int s1 = 0, s2 = 0, s3 = 0, i;

  for (i = 0; i < (int)n; i = (short)(i + 1))
  {
    unsigned int v = data[i];
    s1 = (short)((unsigned int)s1 + v * v);
    s2 = (short)((unsigned int)s2 + v * k);
    s3 = (short)((unsigned int)s3 + k2);
  }
  {
    unsigned int a = absdiff16((unsigned int)s2, (unsigned int)s1);
    int thr = (s3 * (int)factor) >> 15;
    if ((short)a > (short)thr)
      return (unsigned short)(((int)n * 0x6aab) >> 14);
  }
  return 0;
}

/**
 * @brief Detect the answer tone in a 160-sample block (the parts of the engine it needs are passed separately).
 * @param agc   detector AGC, applied to the block first
 * @param mtd1  tone detector 1
 * @param mtd2  tone detector 2
 * @param buf   160 samples; MODIFIED: the 40-sample segments in which detector 1 does not report the tone, and the first one in which it does, are zeroed
 * @return 1 if either detector reported the tone (result 0 of FPM_MTD_detect) in at least three consecutive 40-sample segments of the block, else 0
 *
 * Each of the four segments is run through both detectors.  `cA` counts consecutive segments with detector 2 reporting the tone, `cB` the same for
 * detector 1.  With debug level above 1 the line "V22: Detect_V22 OK!!!" is printed when cA reached 3.
 */
unsigned int Detect_v22_core(FpmAgc* agc, FpmMtd* mtd1, FpmMtd* mtd2, short* buf)
{
  int cA = 0, cB = 0, seg;

  FPM_AGC_agc(agc, buf, 0xa0);
  for (seg = 0; seg < 4; seg++)
  {
    short* p = buf + seg * 0x28;
    short r1 = (short)FPM_MTD_detect(mtd1, p, 0x28);
    short r2 = (short)FPM_MTD_detect(mtd2, p, 0x28);
    int zero;
    if (r2 != 0)
    {
      cA = 0;
      if (r1 != 0)
      {
        cB = 0;
        zero = 1;
      }
      else
      {
        cB = (short)(cB + 1);
        zero = cB <= 1;
      }
    }
    else
    {
      cA = (short)(cA + 1);
      if (r1 != 0)
      {
        cB = 0;
        zero = 1;
      }
      else
      {
        cB = (short)(cB + 1);
        zero = cB <= 1;
      }
    }
    if (zero)
    {
      int k;
      for (k = 0; k < 0x28; k++)
        p[k] = 0;
    }
  }
  if (cA > 2 && dsplibs_debug_level > 1)
    dsplibs_debug_printf("V22: Detect_V22 OK!!!\n");
  return (unsigned int)((cB > 2) | (cA > 2));
}

/** @brief Detect_v22_core() on the engine's detector AGC and tone detectors. */
unsigned int Detect_v22(V22DetEngine* eng, short* buf) { return Detect_v22_core(&eng->core->agc, eng->tones->mtd1, eng->tones->mtd2, buf); }
