/**
 * @file cid_detect.c
 * @brief Caller-ID signal detectors: the DTMF digit detector and the FSK demodulator / seizure-tone detector.
 *
 * Integer-only building blocks used by the caller-ID receivers in cid_modem.c (declarations and state objects in cid.h):
 *   - band_pass(): decides whether a block of samples carries a tone in the DTMF band (scales the block, removes DC, band-pass filters it in place with
 *     FPM_iir_filt() and compares the filtered energy with a level-dependent fraction of the input energy and an adaptive floor, CidDtmf::minEn);
 *   - DTMF_MTD_detect(): picks the digit of a block with eight tone filters (two groups of four, coefficient sets MTD1 .. MTD8 for 8000 or 9600 Hz): in
 *     each group the filter with the smallest output energy (below a threshold derived from the input energy) names the row and the column of the
 *     keypad; returns the digit code or -9;
 *   - dtmf_modem(): the digit state machine that applies band_pass() and DTMF_MTD_detect() to successive frames and collects the digits in CidDtmf::digits;
 *   - reset_dtmf(): puts the DTMF detector in its initial state;
 *   - CID_FSD_demodulate(): turns samples into FSK bits: delay-and-multiply discriminator, 17-tap low-pass, adaptive slicer threshold and bit timing;
 *   - CID_MTD_detect(): the 1200 Hz seizure-tone detector, two cascaded biquads and an energy ratio.
 * Digit codes: 0 .. 9 the digits, 10 .. 13 the keys A .. D, 14 '*', 15 '#' (the layout of the table in DTMF_MTD_detect()).
 */
#include <limits.h>
#include "cid.h"
#include "cid_tables.h"
#include "core/dsp_constants.h" /* SAMPLE_RATE_* */
#include "fpm/fpm.h"

#define CID_BLK_MAX 198 /* size of the local copy of the frame in dtmf_modem(): longer frames are truncated (the callers use 160 / 192) */

/**
 * @brief Decide whether a block carries a tone in the DTMF band.
 * @param x  samples, modified in place (scaled, DC-removed, band-pass filtered)
 * @param n  number of samples
 * @param d  DTMF detector (uses fs, mode, minEn, iirBand)
 * @return 1 if the filtered energy exceeds a fraction of the input energy and the input energy exceeds the floor `minEn`, else 0
 *
 * Preparation.  A block with many large positive samples (more than n / 128 above 20000, else more than n / 128 above 10000) is scaled down by 16 or 8.
 * The block mean is subtracted.  Weak blocks are amplified: counting samples whose magnitude exceeds 150, 300 and 600, the block is multiplied by 6 if at
 * most 2 exceed 150, else by 3 if at most 2 exceed 300, else by 2 if at most 2 exceed 600.
 * Decision.  With u = (input energy) / 2^18 and chk = (filtered energy) / 2^15, the fraction v / 2^15 is chosen from u (0x38a4 for a quiet block, down to
 * 0xa9f for a loud one) and reduced by the detector mode (2: 7/8, 3: 3/4, 4: 1/2).  The tone is present when v * u / 2^15 < chk and the floor is below u.  For
 * blocks longer than 100 samples the floor then adapts to u / 8.
 * @note  The energies are unsigned 32-bit sums and wrap for very loud long blocks.
 */
int band_pass(short* x, short n, CidDtmf* d)
{
  static const short coef9600[5] = {(short)0xe071, (short)0xefc8, 0x4bdf, 0, 0x1038};
  static const short coef8000[5] = {(short)0xe57e, (short)0xed41, 0x3f9f, 0, 0x12bf};
  const short* coef = (d->fs == SAMPLE_RATE_9600) ? coef9600 : coef8000;
  unsigned int eIn = 0, eOut = 0, u, t, chk;
  int v = 0x38a4;
  short c20 = 0, c10 = 0, a = 0, b = 0, c = 0, th;
  int i;

  if (n > 0)
  {
    /* scale down a block with many large positive samples */
    for (i = 0; i < n; i++)
    {
      short s = x[i];
      if (s > 20000)
        c20 = (short)(c20 + 1);
      if (s > 10000)
        c10 = (short)(c10 + 1);
    }
    th = (short)(n >> 7);
    if (th < c20)
    {
      for (i = 0; i < n; i++)
        x[i] = (short)((x[i] + 8) >> 4);
    }
    else if (th < c10)
    {
      for (i = 0; i < n; i++)
        x[i] = (short)((x[i] + 4) >> 3);
    }

    /* remove the block mean */
    {
      int sum = 0;
      for (i = 0; i < n; i++)
        sum += x[i];
      for (i = 0; i < n; i++)
        x[i] = (short)(x[i] - sum / (int)n);
    }

    /* amplify a weak block */
    for (i = 0; i < n; i++)
    {
      int m = x[i];
      if (m < 0)
        m = -m;
      if (m > 0x258)
      {
        a = (short)(a + 1);
        b = (short)(b + 1);
        c = (short)(c + 1);
      }
      else if (m > 0x12c)
      {
        a = (short)(a + 1);
        b = (short)(b + 1);
      }
      else if (m > 0x96)
      {
        a = (short)(a + 1);
      }
    }
    if (a <= 2)
    {
      for (i = 0; i < n; i++)
        x[i] = (short)(x[i] * 6);
    }
    else if (b <= 2)
    {
      for (i = 0; i < n; i++)
        x[i] = (short)(x[i] * 3);
    }
    else if (c <= 2)
    {
      for (i = 0; i < n; i++)
        x[i] = (short)(x[i] * 2);
    }

    /* band-pass filter and accumulate the input and output energies */
    for (i = 0; i < n; i++)
    {
      int s = x[i];
      short y;
      eIn += (unsigned int)(s * s);
      y = FPM_iir_filt((short)s, coef, d->iirBand, 1);
      x[i] = y;
      eOut += (unsigned int)(((int)y * (int)y + 4) >> 3);
    }
  }

  /* energy-dependent threshold fraction, scaled by the detector mode */
  u = (eIn + 0x20000u) >> 18;
  chk = (eOut + 0x4000u) >> 15;
  if (u > 0x55f)
    v = 0xa9f;
  else if (u > 0x2af)
    v = 0x1378;
  else if (u > 0x157)
    v = 0x1fdc;
  if (d->mode == 2)
    v = (short)((v * 7 + 4) >> 3);
  else if (d->mode == 3)
    v = (short)((v * 3 + 2) >> 2);
  else if (d->mode == 4)
    v = (short)((v + 1) >> 1);

  /* decide, and adapt the floor */
  t = ((unsigned int)v * u + 0x4000u) >> 15;
  if (t < chk)
  {
    unsigned short m = d->minEn;
    if ((unsigned int)(int)(short)m < u)
    {
      if (n > 100)
      {
        m = (unsigned short)(u >> 3);
        d->minEn = m;
      }
      if (m == 0)
        d->minEn = 1;
      return 1;
    }
  }
  return 0;
}

/**
 * @brief Identify the DTMF digit in a block.
 * @param x  samples (not modified)
 * @param n  number of samples
 * @param d  DTMF detector (uses fs and the filter states iirLow0, iirLow1, iirMtd)
 * @return the digit code (0 .. 15, see the file header), or -9 if either group has no filter below its threshold
 *
 * Every sample (scaled by 0.9) passes a pre-filter and then the four filters of a group; the group's accumulators are the filter output energies plus a
 * bias proportional to the input energy (i times it for filter i of the first group, 7 - i times for the second group).
 * With th = 1.5 * the input energy, the filter with the smallest accumulator below th (below 0.8 * th in the first group) is chosen in each group; the
 * pair indexes the 4 x 4 digit table.
 */
int DTMF_MTD_detect(const short* x, short n, CidDtmf* d)
{
  static const short digit[16] = {1, 2, 3, 10, 4, 5, 6, 11, 7, 8, 9, 12, 14, 0, 15, 13};
  const short* coef[8];
  unsigned int acc[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  unsigned int en = 0, th, cut, best;
  int i, k, lo = 9, hi = 9;

  /* coefficient set for the sampling rate */
  if (d->fs == SAMPLE_RATE_9600)
  {
    coef[0] = MTD1_COEF_9600;
    coef[1] = MTD2_COEF_9600;
    coef[2] = MTD3_COEF_9600;
    coef[3] = MTD4_COEF_9600;
    coef[4] = MTD5_COEF_9600;
    coef[5] = MTD6_COEF_9600;
    coef[6] = MTD7_COEF_9600;
    coef[7] = MTD8_COEF_9600;
  }
  else
  {
    coef[0] = MTD1_COEF_8000;
    coef[1] = MTD2_COEF_8000;
    coef[2] = MTD3_COEF_8000;
    coef[3] = MTD4_COEF_8000;
    coef[4] = MTD5_COEF_8000;
    coef[5] = MTD6_COEF_8000;
    coef[6] = MTD7_COEF_8000;
    coef[7] = MTD8_COEF_8000;
  }

  /* filter the block and accumulate the energies */
  for (k = 0; k < n; k++)
  {
    int s = (short)(((int)x[k] * 0x7332 + 0x4000) >> 15);
    short f;
    unsigned int e12;

    en += (unsigned int)((s * s + 4) >> 3);
    e12 = en >> 12;
    f = FPM_iir_filt((short)s, coef[4], d->iirLow1, 1);
    for (i = 0; i < 4; i++)
    {
      short y = FPM_iir_filt(f, coef[i], d->iirMtd[i], 1);
      acc[i] += e12 * (unsigned int)i + (unsigned int)(((int)y * (int)y + 4) >> 3);
    }
    f = FPM_iir_filt((short)s, coef[3], d->iirLow0, 1);
    for (i = 4; i < 8; i++)
    {
      short y = FPM_iir_filt(f, coef[i], d->iirMtd[i], 1);
      acc[i] += (unsigned int)(7 - i) * e12 + (unsigned int)(((int)y * (int)y + 4) >> 3);
    }
  }

  /* normalise and pick the smallest filter output of each group */
  en = (en + 0x4000u) >> 15;
  for (i = 0; i < 8; i++)
    acc[i] = (acc[i] + 0x4000u) >> 15;
  th = (en * 3u + 1u) >> 1;
  cut = (th * 0x6666u + 0x4000u) >> 15;
  best = th;
  for (i = 0; i < 4; i++)
  {
    if (acc[i] < cut && acc[i] < best)
    {
      best = acc[i];
      lo = i;
    }
  }
  best = th;
  for (i = 0; i < 4; i++)
  {
    if (acc[4 + i] < th && acc[4 + i] < best)
    {
      best = acc[4 + i];
      hi = i;
    }
  }

  if (lo <= 3 && hi <= 3)
    return digit[hi + lo * 4];
  return -9;
}

/**
 * @brief Put a DTMF detector in its initial state: no digits, phase CIDDTMF_LISTEN, all filter states cleared.
 * @param d  DTMF detector (its sampling rate and mode are kept)
 * @note  Only digits[0 .. 15] are cleared.
 */
void reset_dtmf(CidDtmf* d)
{
  int i;

  if (dsplibs_debug_level > 1)
    dsplibs_debug_printf("DTMF CID Reset !\n");

  d->state0 = 0;
  d->samples = 0;
  d->cnt = 0;
  d->lastDig = USHRT_MAX;
  d->minEn = 1;
  d->fails = 0;
  d->nDig = -1;
  d->blkp = d->blk;
  d->s356 = 0;
  d->s354 = 0;
  d->iirBand[1] = 0;
  d->iirBand[0] = 0;
  d->s35c[1] = 0;
  d->s35c[0] = 0;
  d->iirLow0[1] = 0;
  d->iirLow0[0] = 0;
  d->w004 = 0;
  d->s008 = 0;
  for (i = 0; i < 8; i++)
  {
    d->iirMtd[i][1] = 0;
    d->iirMtd[i][0] = 0;
  }
  for (i = 0; i < 16; i++)
    d->digits[i] = 0;
  d->phase = CIDDTMF_LISTEN;
}

/**
 * Store `v` at digits[idx] with the byte offset computed inside the object.
 * @bug `idx` (CidDtmf::nDig) is not bounded by the 20-byte array.  A caller that keeps feeding frames after the result -1 of the 20th digit makes nDig 20 ..
 *      25, and the stores land in the fields behind digits[] (s354, s356, then iirBand).  Here the store is done on the object's bytes, which is defined
 *      behaviour inside the object; a store outside the object is dropped.
 */
static void digit_put(CidDtmf* d, short idx, char v)
{
  long off = (long)offsetof(CidDtmf, digits) + (long)idx;

  if (off >= 0 && off < (long)sizeof(CidDtmf))
    ((char*)d)[off] = v;
}

/* The digit bookkeeping shared by the three exits of dtmf_modem(): 'C' (12) is stored as 0 and ends the number (result 3), 0 .. 9 are stored as ASCII,
 * the other codes (A, B, D, '*', '#', USHRT_MAX = none) are ignored. */
static void dtmf_store(CidDtmf* d, int dig, int* ret)
{
  short idx;

  if (dig == 0xc)
  {
    idx = d->nDig;
    digit_put(d, idx, 0);
    d->nDig = (short)(idx + 1);
    *ret = 3;
    if (dsplibs_debug_level > 1)
      dsplibs_debug_printf("CID DTMF Detected 'C' !\n");
  }
  else if (dig <= 9)
  {
    idx = d->nDig;
    d->nDig = (short)(idx + 1);
    digit_put(d, idx, (char)(dig + '0'));
  }
}

/**
 * @brief Run the DTMF digit state machine over one frame.
 * @param in  frame of samples (not modified)
 * @param n   number of samples; at most CID_BLK_MAX (198) are used
 * @param d   DTMF detector
 * @return 1 = keep going; 2 = more than 2 digits are stored; 3 = the number ended with 'C'; -1 = failed (or the 20th digit was stored)
 *
 * The frame is copied to a local buffer and the time since the last detection (`samples`) is advanced.  Once more than two digits are stored, the result
 * is 2, becomes -1 after 3 s without a new digit, and the phase becomes CIDDTMF_DONE after 300 ms of silence.  By phase (CidDtmfPhase):
 *   - LISTEN: band_pass() is run on the two halves of the frame.  A tone in both halves starts decoding at once; a tone only in the second half starts it
 *     on the next frame (the second half is kept in `stash`, halfFlag = 1).
 *   - DECODE: band_pass() on the whole frame.  If the tone has gone, the last detected digit is stored when `cnt` is non-zero, otherwise the failure count
 *     rises.  If the tone is present, DTMF_MTD_detect() names the digit (on the stash + first half when a half frame was kept); in mode 1 the first thing
 *     taken must be the start tone 'D' (code 13).  A digit is accepted when two consecutive detections agree, after which the phase is WAIT_GAP.
 *     More than 5 failures after the third digit give -1.
 *   - WAIT_GAP: back to LISTEN as soon as one half frame is quiet.
 *   - DONE: nothing more is decoded.
 * @note  Only digits[0 .. 15] are returned by cid_get_strings(); up to 20 are stored.
 * @warning A frame must not be fed after the result -1 of the 20th digit (see digit_put()).
 */
int dtmf_modem(const short* in, unsigned short n, CidDtmf* d)
{
  short buf[CID_BLK_MAX];
  int ret = 1, flag = 1, i;
  int samples;
  short phase;

  if (n > CID_BLK_MAX)
    n = CID_BLK_MAX;
  for (i = 0; i < n; i++)
    buf[i] = in[i];

  /* silence bookkeeping once digits have been collected */
  samples = d->samples + (int)n;
  d->samples = samples;
  if (d->nDig > 2)
  {
    unsigned int fs = d->fs;
    ret = 2;
    if (samples > (int)(fs * 3u))
      ret = -1;
    if (samples > (int)(fs * 300u / 1000u))
      d->phase = CIDDTMF_DONE;
  }
  phase = d->phase;

  /* LISTEN: look for a tone in the half frames */
  if (phase == CIDDTMF_LISTEN)
  {
    unsigned short h = (unsigned short)(n >> 1);
    int r1, r2;

    if (d->samples > (int)((unsigned int)d->fs * 3u) && d->nDig > 0)
      ret = -1;
    r1 = band_pass(buf, (short)h, d);
    r2 = (short)band_pass(buf + h, (short)h, d);
    if (r1 == 0 && r2 == 1)
    {
      flag = 0;
      d->phase = CIDDTMF_DECODE;
      for (i = 0; i < h; i++)
        d->stash[i] = buf[h + i];
      d->halfFlag = 1;
    }
    if (r1 == 1 && r2 == 1)
    {
      d->halfFlag = 0;
      flag = 1;
      d->phase = CIDDTMF_DECODE;
      phase = CIDDTMF_DECODE;
    }
    else
    {
      phase = d->phase;
    }
  }

  /* DECODE: confirm the tone and name the digit */
  if (phase == CIDDTMF_DECODE && flag)
  {
    int r = band_pass(buf, (short)n, d);
    short c, cnt1;

    /* the tone has gone: accept the digit seen so far, or count a failure */
    if (r == 0)
    {
      if (d->cnt > 0)
      {
        dtmf_store(d, (int)d->lastDig, &ret); /* unsigned: USHRT_MAX is > 9 */
      }
      else
      {
        d->minEn = 1;
        d->fails = (short)(d->fails + 1);
      }
      if (d->fails > 5 && d->nDig > 2)
        ret = -1;
      d->phase = CIDDTMF_LISTEN;
      d->cnt = 0;
      d->lastDig = USHRT_MAX;
      return ret;
    }

    /* assemble the frame to decode and identify the digit */
    if (d->halfFlag == 0)
    {
      d->blkp = buf;
    }
    else
    {
      unsigned short m = (unsigned short)(n >> 1);
      for (i = 0; i < m; i++)
      {
        d->blk[i] = d->stash[i];
        d->blk[i + m] = buf[i];
        d->stash[i] = buf[i + m];
      }
      d->blkp = d->blk;
    }
    c = (short)DTMF_MTD_detect(d->blkp, (short)n, d);
    d->samples = 0;
    cnt1 = (short)(d->cnt + 1);

    /* mode 1 waits for the start tone 'D' (13) first */
    if (d->mode == 1)
    {
      int atStart = (d->nDig == -1);
      if (c != 0xd && atStart)
        c = (short)(cnt1 + 0x14);
      if (c == 0xd && atStart)
      {
        d->fails = 0;
        cnt1 = 0;
        d->phase = CIDDTMF_WAIT_GAP;
        d->lastDig = USHRT_MAX;
        d->nDig = 0;
      }
    }
    else if (d->nDig == -1)
    {
      d->nDig = 0;
    }

    /* two equal detections in a row accept the digit */
    if ((short)d->lastDig == c)
    {
      if (c != -9 && c <= 0xd && c != 10 && c != 11)
      {
        d->fails = 0;
        d->cnt = 0;
        d->lastDig = USHRT_MAX;
        d->phase = CIDDTMF_WAIT_GAP;
        dtmf_store(d, (int)c, &ret);
        if (d->nDig != 0x14)
          return ret;
        return -1;
      }
      d->cnt = cnt1;
      d->minEn = 1;
      d->fails = (short)(d->fails + 1);
      if (d->fails <= 5)
        return ret;
      if (d->nDig <= 2)
        return ret;
      return -1;
    }
    d->lastDig = (unsigned short)c;
    d->cnt = (short)(d->nDig > 0);
    return ret;
  }

  /* WAIT_GAP: wait for the tone to go */
  if (phase == CIDDTMF_WAIT_GAP)
  {
    unsigned short h = (unsigned short)(n >> 1);
    short r1 = (short)band_pass(buf, (short)h, d);
    short r2 = (short)band_pass(buf + h, (short)h, d);

    if (r1 == 0 || r2 == 0)
      d->phase = CIDDTMF_LISTEN;
  }
  return ret;
}

/**
 * @brief Demodulate FSK (1200 baud) samples into bits.
 * @param in   samples
 * @param out  decided bits, one per 16-bit word (0 or 1)
 * @param n    number of samples (0 returns 0)
 * @param d    FSK receiver (uses fs and the demodulator state)
 * @return number of bits written to `out`
 *
 * Per sample: the sample enters a 5-sample history; the discriminator multiplies it by a delayed copy (AUTOCOR_COEF_9600: 4 samples, -0.9; otherwise
 * AUTOCOR_COEF_7200: 1 sample); the product goes through the 17-tap low-pass fix_LPF.  The slicer threshold `thr` is adaptive: it is first the midpoint of
 * the mean of the first 128 values above the old threshold and the average of the negative values, and is updated from the negative values after 16 of
 * them and then every 128.  The value, less one eighth of the threshold, is sliced with a dead zone of +-22 into a bit.  The samples per bit are 8 at
 * 9600 Hz and 6 otherwise.  A bit is emitted when the level has stayed the same for that many samples, or earlier when half that many samples of the
 * opposite level have been seen (a resynchronising transition).  A long dead-zone run (three bit times) resets the run counters.
 * @bug   As in FPM_FSD_demodulate(), the same-level test is an equality (`run == samples per bit`) and `runAlt` is only cleared when a bit is emitted: a
 *        disagreeing sample on the sample that completes the bit can make the equality miss, after which no bit is emitted until half a bit time of
 *        opposite-level samples has accumulated (or `run` wraps).
 * @warning `out` receives up to one bit per half bit time of samples, which for noise can exceed the 36 entries of CidFsk::out; the receiver reads them
 *          back from the memory behind it (cid_modem.c).
 */
unsigned short CID_FSD_demodulate(const short* in, unsigned short* out, short n, CidFsk* d)
{
  unsigned short nOut = 0;
  short k;
  int th;

  if (n == 0)
    return 0;
  th = d->thr;
  k = (short)(n - 1);
  do
  {
    unsigned short fs;
    short idx, a, y, cp;
    const short* ac;
    int acc, m, q, dd, kk;
    int j;

    /* push the sample into the 5-sample history and form the delayed copy */
    idx = (short)(d->pos + 1);
    if (idx < 5)
      d->pos = idx;
    else
      d->pos = 0;
    idx = d->pos;
    fs = d->fs;
    d->hist[idx] = *in++;
    ac = (fs == SAMPLE_RATE_9600) ? AUTOCOR_COEF_9600 : AUTOCOR_COEF_7200;
    acc = 0;
    for (j = 0; j <= idx; j++)
      acc += (int)d->hist[idx - j] * (int)ac[j];
    for (j = idx + 1; j <= 4; j++)
      acc += (int)d->hist[idx + 5 - j] * (int)ac[j];
    a = (short)(acc >> 15);

    /* discriminator product, then the 17-tap low-pass */
    m = ((int)d->hist[idx < 0 ? idx + 5 : idx] * (int)a) >> 15;
    cp = (short)(d->lpfPos + 1);
    if (cp <= 16)
      d->lpfPos = cp;
    else
      d->lpfPos = 0;
    j = d->lpfPos;
    d->lpf[j] = (short)m;
    acc = 0;
    {
      int p;
      for (p = 0; p <= j; p++)
        acc += (int)d->lpf[j - p] * (int)fix_LPF[p];
      for (p = j + 1; p <= 16; p++)
        acc += (int)d->lpf[j + 17 - p] * (int)fix_LPF[p];
    }
    y = (short)(acc >> 15);
    q = y;

    /* adapt the slicer threshold */
    if (th < q)
    {
      short c0 = d->nPos;
      d->nPos = (short)(c0 + 1);
      if (c0 <= 0x7f)
      {
        int s = d->posAcc + q;
        d->posAcc = s;
        if ((short)(c0 + 1) == 0x80)
          d->mean = (s + 0x40) >> 7;
      }
    }
    if (d->mean != 0 && q < 0)
    {
      int s = d->negAcc + q;
      unsigned short c1 = (unsigned short)(d->nNeg + 1);
      d->nNeg = (short)c1;
      if (c1 == 0x10)
      {
        th = (d->mean + (s >> 4)) >> 1;
        d->thr = th;
      }
      if ((unsigned char)d->nNeg == 0 || (unsigned char)d->nNeg == 0x80)
      {
        d->negAcc = s >> 7;
        th = (d->mean + (s >> 7)) >> 1;
        d->thr = th;
      }
      else
      {
        d->negAcc = s;
      }
    }

    /* slice with a dead zone and recover the bit timing */
    dd = q - (th >> 3);
    kk = (fs == SAMPLE_RATE_9600) * 2 + 6;
    if ((unsigned int)(dd + 0x16) <= 0x2c)
    {
      short qq = (short)(d->quiet + 1);
      d->quiet = qq;
      if (qq > kk * 3)
      {
        d->runAlt = (short)-kk;
        d->run = (short)-kk;
      }
    }
    else
    {
      unsigned short cur = d->cur;
      unsigned short bit = (dd > 0x16) ? 1u : 0u;
      short r = (short)(d->run + 1);
      d->run = r;
      if ((short)cur == (short)bit)
      {
        if (r == kk)
        {
          out[nOut++] = cur;
          d->runAlt = 0;
          d->run = 0;
        }
      }
      else
      {
        short ra = (short)(d->runAlt + 1);
        d->runAlt = ra;
        if (ra == (kk >> 1))
        {
          d->cur = bit;
          out[nOut++] = bit;
          d->runAlt = 0;
          d->run = 0;
        }
      }
      d->quiet = 0;
    }
  } while (k-- != 0);

  return nOut;
}

/**
 * @brief Detect the 1200 Hz seizure tone in a block.
 * @param x  samples (not modified)
 * @param n  number of samples
 * @param d  FSK receiver (uses fs and the filter states iirMtd1, iirMtd2)
 * @return 0 if the tone is present (the output energy of the two-biquad cascade is below half of the input energy and the input energy exceeds 150),
 *         else 1
 */
int CID_MTD_detect(const short* x, short n, CidFsk* d)
{
  const short *cA, *cB;
  unsigned int e1 = 0, e2 = 0;
  short i;

  if (d->fs == SAMPLE_RATE_9600)
  {
    cA = cid_MTD_COEF_2_9600;
    cB = cid_MTD_COEF_1_9600;
  }
  else
  {
    cA = cid_MTD_COEF_2_8000;
    cB = cid_MTD_COEF_1_8000;
  }

  for (i = 0; i < n; i++)
  {
    int s = x[i];
    short y1, y2;

    e1 += (unsigned int)((s * s + 0x20) >> 6);
    y1 = FPM_iir_filt((short)s, cB, d->iirMtd1, 1);
    y2 = FPM_iir_filt(y1, cA, d->iirMtd2, 1);
    e2 += (unsigned int)(((int)y2 * (int)y2 + 0x20) >> 6);
  }

  return !(((e1 >> 1) > e2) && (e1 > 0x96));
}
