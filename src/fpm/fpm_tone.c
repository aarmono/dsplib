/**
 * @file fpm_tone.c
 * @brief FPM_TONE: fixed-frequency tone generator, band-pass tone detector and phase-reversal finder.
 *
 * Owns the FpmTone object (fpm.h).  FPM_TONE_create() builds it from a nine-word configuration block (or the default block FPM_TONE_CFG_raw: a 2100 Hz tone
 * with a 180 degree phase reversal every 450 * 8 samples), the other entry points work on it:
 *   - generation: FPM_TONE_generate() (sine), FPM_TONE_generate2() (cosine and sine), FPM_TONE_generate_demod() (cosine from FPM_phasor_demod()),
 *     FPM_TONE_set_freq() and FPM_TONE_set_scale() retune the running generator; the phase is kept between calls;
 *   - detection: FPM_TONE_filter() applies the band-pass FIR centred on the tone, FPM_TONE_detect() compares band energy with the energy a notch removes,
 *     FPM_TONE_find_rev() finds phase reversals of the tone, FPM_TONE_kill() removes the tone with the notch;
 *   - FPM_TONE_delete() frees the object and its buffers.
 * Frequencies are in Hz of an 8 kHz sample clock: a phase increment is freq * 0x8312 / 2^13 on the 0x8000-per-turn angle scale of FPM_phasor() (fpm_math.c).
 * Users: the FSK modulator FPM_FSM (fpm_fsm.c) and the V.21 / Bell 103, V.22, V.23, V.32 and fax modems (tone generation, tone detection, notching; the
 * phase-reversal finder is used by the V.32 handshake).
 *
 * Object layout.  FpmTone::r0 .. r3 are runs of 16-bit fields between the pointer members (see fpm.h):
 *   r0.s[0] frequency;  s[1] amplitude;  s[2] phase-reversal interval (units of 8 samples, 0 = none);  s[3] detect ratio (Q15);  s[5] minimum band energy;
 *   s[6] notch pole radius (Q15).
 *   r1.s[0] band-pass tap count;  s[4] / s[5] ratio (Q15) and window length of the reversal finder;  s[6] extra history length;
 *   s[8] generator phase;  s[9] phase increment;  s[10] samples elapsed (units of 8) since the last reversal.
 *   r2: [0] FIR history position;  [1..5] notch coefficients (killCoef);  [6..9] notch state of FPM_TONE_detect();  [10] / [11] removed / band energy;
 *   [12] samples since the last reversal found;  [13] / [14] correlation / energy of the finder;  [95] finder history position;  the 0x50 shorts from
 *   r2[0x0f .. 0x5e] (TONE_HIST) are the finder's sample history.
 *   r3: state of FPM_TONE_kill();  kfilt / kstate: DC-rejecting pre-filter of the finder.
 */
#include <stdint.h>
#include "core/dsp_table_handles.h"
#include "fpm.h"
#include "fpm_tables.h"
#include "core/dsp_host.h"

/* The 0x50-short sample history of FPM_TONE_find_rev() is the part of r2 that starts at r2[0x0f]; on a 32-bit build it is addressed from the object start
 * (short index 0x29 .. 0x78), on a 64-bit build from the r2 run, because the pointer members make the byte offsets differ. */
#if UINTPTR_MAX == 0xffffffffu
#define TONE_HIST(p) ((short*)(p))
#define TONE_HIST_I0 0x29
#else
#define TONE_HIST(p) ((p)->r2)
#define TONE_HIST_I0 (0x29 - 0x1a)
#endif

/**
 * @brief Create a tone object, or re-initialise an existing one.
 * @param toneObj  existing object to re-initialise (its buffers are kept), or NULL to allocate the object and its buffers
 * @param cfg      nine configuration words, or NULL for the default block FPM_TONE_CFG_raw: cfg[0 .. 3] = r0 (shorts, see the file header), cfg[4] = CFGP
 *                 handle of the low-pass prototype (ToneLPF), cfg[5 .. 8] = r1
 * @return the tone object
 *
 * Derives from the configuration: the generator phase increment; the notch (tone-removing) coefficients, a second-order section with zeros on the unit circle
 * at the tone frequency and a pole radius of r0.s[6] / 32768; the band-pass taps `lpfOut`, the low-pass prototype modulated by the cosine of the tone
 * frequency; and the coefficients of the DC-rejecting pre-filter used by FPM_TONE_find_rev().  All states and histories are cleared.
 * @warning A new object (toneObj == NULL) with a band-pass tap count r1.s[0] <= 0 leaves `kfilt` / `kstate` unallocated, and the function then writes through
 *          them.  An existing object must have been created before (its buffers are reused, not checked).  The sysdep_malloc() results are not checked.
 */
FpmTone* FPM_TONE_create(FpmTone* toneObj, const int* cfg)
{
  unsigned short* kfiltCoef;
  unsigned short* kfiltState;
  short lpfLen;
  int idx;
  int bAlloc;
  short ph[4];

  bAlloc = toneObj == (FpmTone*)0;
  if (bAlloc)
  {
    toneObj = (FpmTone*)sysdep_malloc(sizeof(FpmTone));
  }

  /* configuration words: the default block when cfg is NULL */
  if (cfg == (const int*)0)
  {
    toneObj->r0.w[0] = FPM_TONE_CFG_raw[0];
    toneObj->r0.w[1] = FPM_TONE_CFG_raw[1];
    toneObj->r0.w[2] = FPM_TONE_CFG_raw[2];
    toneObj->r0.w[3] = FPM_TONE_CFG_raw[3];
    FPM_TABLE_SET(toneObj, lpf, const short*, CFGP(ToneLPF));
    toneObj->r1.w[0] = FPM_TONE_CFG_raw[5];
    toneObj->r1.w[1] = FPM_TONE_CFG_raw[6];
    toneObj->r1.w[2] = FPM_TONE_CFG_raw[7];
    toneObj->r1.w[3] = FPM_TONE_CFG_raw[8];
  }
  else
  {
    toneObj->r0.w[0] = cfg[0];
    toneObj->r0.w[1] = cfg[1];
    toneObj->r0.w[2] = cfg[2];
    toneObj->r0.w[3] = cfg[3];
    FPM_TABLE_SET(toneObj, lpf, const short*, cfg[4]);
    toneObj->r1.w[0] = cfg[5];
    toneObj->r1.w[1] = cfg[6];
    toneObj->r1.w[2] = cfg[7];
    toneObj->r1.w[3] = cfg[8];
  }

  /* buffers of a new object */
  lpfLen = toneObj->r1.s[0];
  if (bAlloc && 0 < lpfLen)
  {
    FPM_M(void*, toneObj, lpfOut) = sysdep_malloc((unsigned int)lpfLen * 2);
    FPM_M(void*, toneObj, hist) = sysdep_malloc((unsigned int)((int)lpfLen + (int)toneObj->r1.s[6]) * 2);
    FPM_M(void*, toneObj, kfilt) = sysdep_malloc(10);
    FPM_M(void*, toneObj, kstate) = sysdep_malloc(8);
  }

  /* generator phase increment and the notch section; ph[0] = phase, ph[1] / ph[2] = cosine / sine, ph[3] = increment (see FPM_phasor()) */
  ph[0] = (short)(((int)toneObj->r0.s[0] * 0x8312 + 0x1000) >> 0xd);
  ph[3] = 0;
  FPM_phasor(ph);
  toneObj->r1.s[8] = 0;
  idx = (int)toneObj->r0.s[6];
  toneObj->r1.s[10] = 0;
  toneObj->r1.s[9] = ph[0];
  toneObj->r2[1] = 0x4000;
  toneObj->r2[2] = 0x4000;
  toneObj->r2[4] = (short)((unsigned int)(idx * idx) >> 0x10);
  toneObj->r2[3] = (short)(ph[1] * -2);
  toneObj->r2[6] = 0;
  toneObj->r2[7] = 0;
  toneObj->r2[8] = 0;
  toneObj->r2[9] = 0;
  toneObj->r2[0] = 0;
  toneObj->r2[10] = 0;
  /* the product is formed in full int precision and truncated to 16 bits only once, after the shift */
  toneObj->r2[5] = (short)(-(ph[1] * idx) >> 0xe);
  toneObj->r2[11] = 0;

  /* band-pass taps: the low-pass prototype times the cosine of the tone frequency, one phase step per tap */
  ph[3] = ph[0];
  for (idx = 0; (short)idx < lpfLen; idx = (int)(short)((short)idx + 1))
  {
    short* lpf_out;
    const short* lpf_coef;

    FPM_phasor(ph);
    lpf_coef = toneObj->lpf;
    lpf_out = toneObj->lpfOut;
    (toneObj->hist)[idx] = 0;
    lpf_out[idx] = (short)((int)(lpf_coef[idx] * 2 * (int)ph[1]) >> 0xe);
  }

  /* clear the finder state and its sample history */
  toneObj->r2[12] = 0;
  toneObj->r2[13] = 0;
  toneObj->r2[95] = 0;
  toneObj->r2[14] = 0;
  for (idx = 0; idx < 0x50; idx++)
  {
    TONE_HIST(toneObj)[idx + TONE_HIST_I0] = 0;
  }
  toneObj->r3[1] = 0;
  toneObj->r3[0] = 0;
  toneObj->r3[2] = 0;
  toneObj->r3[3] = 0;
  toneObj->killCoef = &toneObj->r2[1];

  /* pre-filter of the finder: a second-order section with a double zero at DC and poles at radius 0.96 */
  {
    short ph0[4] = {0, 0, 0, 0};

    FPM_phasor(ph0);
    kfiltCoef = (unsigned short*)toneObj->kfilt;
    kfiltState = (unsigned short*)toneObj->kstate;
    kfiltCoef[0] = 0x4000;
    kfiltCoef[1] = 0x4000;
    kfiltCoef[3] = 0x3afb;
    kfiltCoef[2] = (unsigned short)(ph0[1] * -2);
    /* as above: the product is truncated to 16 bits once, after the shift */
    kfiltCoef[4] = (unsigned short)((ph0[1] * -0x7ae1) >> 0xe);
    kfiltState[0] = 0;
    kfiltState[1] = 0;
    kfiltState[2] = 0;
    kfiltState[3] = 0;
  }

  return toneObj;
}

/**
 * @brief Free a tone object created by FPM_TONE_create(NULL, ...): its buffers (when the band-pass tap count is positive) and the object.
 * @param toneObj  tone object
 */
void FPM_TONE_delete(FpmTone* toneObj)
{
  FpmTone* base = toneObj;

  if (0 < base->r1.s[0])
  {
    sysdep_free(FPM_M(void*, base, kstate));
    sysdep_free(FPM_M(void*, base, kfilt));
    sysdep_free(FPM_M(void*, base, hist));
    sysdep_free(FPM_M(void*, base, lpfOut));
  }
  sysdep_free(toneObj);
}

/**
 * @brief Tone-present detector over a block of samples (the samples are not modified).
 * @param toneObj  tone object
 * @param samples  input samples
 * @param count    number of samples
 * @return 2 if the smoothed band energy is below r0.s[5] (too little signal); otherwise 1 if the removed energy <= (r0.s[3] * band energy) >> 15, else 0
 *
 * Each sample goes through the band-pass FIR (the taps `lpfOut` over a circular history of the last r1.s[0] input samples) and then the notch (r2[1 .. 5],
 * state r2[6 .. 9]).  The band energy is the smoothed square of the FIR output, the removed energy the smoothed difference between the squares before and
 * after the notch; both are one-pole averages with weights 0.95 / 0.05 (0x799a / 0x666, Q15) and are kept in the object between calls.
 */
int FPM_TONE_detect(FpmTone* toneObj, short* samples, short count)
{
  short lpfLen;
  short tap;
  short* hist;
  short* bpTaps;
  int result;
  short left;
  int m;
  unsigned int k;
  short* pHist;
  unsigned int histIdx;
  unsigned short removedPos;
  short* pTap;
  int acc;
  int more;
  int bandEnergy;
  unsigned int removedEnergy;
  short firOut;

  removedEnergy = (unsigned int)(short)toneObj->r2[10];
  left = count + -1;
  bandEnergy = (int)toneObj->r2[11];
  lpfLen = toneObj->r1.s[0];
  hist = toneObj->hist;
  histIdx = (unsigned int)(unsigned short)toneObj->r2[0];
  bpTaps = toneObj->lpfOut;

  if (count != 0)
  {
    do
    {
      short bandEnergyLo = (short)bandEnergy;
      short scratch16 = (short)histIdx + 1;
      short sample = *samples;

      /* push the sample into the circular history */
      samples = samples + 1;
      histIdx = (unsigned int)(unsigned short)(-(unsigned int)(scratch16 < lpfLen) & (int)scratch16);
      hist[histIdx] = sample;

      /* band-pass FIR, newest sample first */
      acc = 0;
      pHist = hist + histIdx;
      pTap = bpTaps;
      for (k = histIdx; (int)k >= 0; k = k - 1)
      {
        scratch16 = *pHist;
        pHist = pHist - 1;
        tap = *pTap;
        pTap = pTap + 1;
        acc = acc + (int)scratch16 * (int)tap;
      }
      pHist = pHist + lpfLen;
      m = (int)lpfLen;
      while (m = m - 1, (int)histIdx < m)
      {
        scratch16 = *pHist;
        pHist = pHist - 1;
        tap = *pTap;
        pTap = pTap + 1;
        acc = acc + (int)scratch16 * (int)tap;
      }

      /* notch, then update the two energies */
      firOut = (short)(acc >> 0xf);
      acc = (int)firOut;
      FPM_iir_filt_II(&firOut, &toneObj->r2[1], &toneObj->r2[6], 1, 1);
      scratch16 = (short)((acc * acc) >> 0xf);
      removedEnergy =
          (unsigned int)(short)(((int)(removedEnergy * 0x799a) + (int)(short)(scratch16 - (short)(((int)firOut * (int)firOut) >> 0xf)) * 0x666) >> 0xf);
      bandEnergyLo = (short)(((int)bandEnergy * 0x799a + (int)scratch16 * 0x666) >> 0xf);
      bandEnergy = bandEnergyLo;
      more = left != 0;
      left = left + -1;
      if (!more)
        break;
    } while (1);
  }

  /* store the state; a negative removed energy counts as 0 */
  toneObj->r2[0] = (short)histIdx;
  toneObj->r2[11] = (short)bandEnergy;
  removedPos = (unsigned short)(((int)~removedEnergy) >> 0xf) & (unsigned short)removedEnergy;
  toneObj->r2[10] = (short)removedPos;

  /* decide */
  result = 2;
  if (toneObj->r0.s[5] <= (short)bandEnergy)
  {
    result = (int)(short)removedPos <= (int)((int)toneObj->r0.s[3] * (int)(short)bandEnergy >> 0xf);
  }

  return result;
}

/**
 * @brief Apply the band-pass FIR (taps `lpfOut`, centred on the tone) to a block, in place.
 * @param toneObj  tone object
 * @param samples  samples, replaced by the filtered samples (Q15 product sum >> 15)
 * @param count    number of samples
 */
void FPM_TONE_filter(FpmTone* toneObj, short* samples, short count)
{
  short lpfLen;
  short tap;
  short* hist;
  short* bpTaps;
  short scratch16;
  int m;
  unsigned int k;
  short* pHist;
  unsigned int histIdx;
  short* pTap;
  int acc;
  short left;

  lpfLen = toneObj->r1.s[0];
  hist = toneObj->hist;
  left = count + -1;
  bpTaps = toneObj->lpfOut;
  histIdx = (unsigned int)(unsigned short)toneObj->r2[0];

  if (count != 0)
  {
    do
    {
      scratch16 = (short)histIdx + 1;
      acc = 0;
      histIdx = (unsigned int)(unsigned short)(-(unsigned int)(scratch16 < lpfLen) & (int)scratch16);
      hist[histIdx] = *samples;
      pHist = hist + histIdx;
      pTap = bpTaps;
      for (k = histIdx; (int)k >= 0; k = k - 1)
      {
        scratch16 = *pHist;
        pHist = pHist - 1;
        tap = *pTap;
        pTap = pTap + 1;
        acc = acc + (int)scratch16 * (int)tap;
      }
      pHist = pHist + lpfLen;
      m = (int)lpfLen;
      while (m = m - 1, (int)histIdx < m)
      {
        scratch16 = *pHist;
        pHist = pHist - 1;
        tap = *pTap;
        pTap = pTap + 1;
        acc = acc + (int)scratch16 * (int)tap;
      }
      *samples = (short)(acc >> 0xf);
      samples = samples + 1;
      left = left + -1;
    } while (left != -1);
  }

  toneObj->r2[0] = (short)histIdx;
}

/**
 * @brief Find phase reversals of the tone in a block of samples.
 * @param toneObj  tone object
 * @param samples  input samples, replaced by the output of the DC-rejecting pre-filter (kfilt / kstate)
 * @param count    number of samples
 * @return 0, or (samples since the previous reversal) >> 3 when a reversal was found in this block (the last one if several)
 *
 * After the pre-filter a running lag-W correlation and a running energy are kept over a window of W = r1.s[5] samples (the energy over 2 * W), using a
 * sample history of 2 * W entries.  A reversal is reported when 2 * correlation < (r1.s[4] * energy) >> 15 and more than 0xa0 samples have passed
 * since the previous report (r2[12] counts them); the counter then restarts.  Correlation and energy are saturated to 16 bits and kept between calls.
 * @warning The history has room for 0x50 samples, so r1.s[5] must not exceed 40.
 */
int FPM_TONE_find_rev(FpmTone* toneObj, short* samples, short count)
{
  short win;
  short x;
  short xOld;
  int histLen;
  short scratch16;
  unsigned int histIdx;
  int histBase;
  int energy;
  int energyClip;
  int corr;
  int result;
  int corrClip;
  int i;

  result = 0;
  win = toneObj->r1.s[5];
  corr = (int)toneObj->r2[13];
  histIdx = (unsigned int)(unsigned short)toneObj->r2[95];
  histBase = TONE_HIST_I0;
  energy = (int)toneObj->r2[14];
  FPM_iir_filt_II((short*)samples, toneObj->kfilt, toneObj->kstate, 1, count);
  energyClip = energy;
  corrClip = corr;
  i = 0;

  if (0 < count)
  {
    histLen = win * 2;
    do
    {
      /* advance the circular history; `xOld` is the sample 2 * W back, TONE_HIST[scratch16] the one W back */
      histIdx = (unsigned int)(unsigned short)((short)histIdx + 1);
      histIdx = (unsigned int)(unsigned short)(-(unsigned int)((int)histIdx < histLen) & histIdx);
      scratch16 = (short)histIdx - win;
      if (scratch16 < 0)
      {
        scratch16 = (short)(scratch16 + (short)histLen);
      }
      x = samples[i];
      xOld = TONE_HIST(toneObj)[histBase + histIdx];

      /* update the running correlation and energy */
      corrClip = 0x7fff;
      corr = corr + ((((int)x - (int)xOld) * (int)TONE_HIST(toneObj)[histBase + scratch16] + 0x4000) >> 0xf);
      if (corr < 0x8000)
      {
        corrClip = -0x8000;
        if (-0x8001 < corr)
        {
          corrClip = (short)corr;
        }
      }
      energy = (energy + (((int)x * (int)x) >> 0xf)) - (((int)xOld * (int)xOld) >> 0xf);
      energyClip = 0x7fff;
      if (energy < 0x8000)
      {
        energyClip = -0x8000;
        if (-0x8001 < energy)
        {
          energyClip = (short)energy;
        }
      }

      /* reversal test with the minimum spacing */
      if (corrClip * 2 < (int)(((int)toneObj->r1.s[4] * energyClip) >> 0xf))
      {
        scratch16 = toneObj->r2[12];
        if (0xa0 < scratch16)
        {
          toneObj->r2[12] = 0;
          result = (int)scratch16 >> 3;
          scratch16 = 0;
        }
      }
      else
      {
        scratch16 = toneObj->r2[12];
      }
      toneObj->r2[12] = scratch16 + 1;
      scratch16 = (short)i + 1;
      TONE_HIST(toneObj)[histBase + histIdx] = samples[i];
      i = (int)scratch16;
    } while (scratch16 < count);
  }

  toneObj->r2[95] = (short)histIdx;
  toneObj->r2[13] = (short)corrClip;
  toneObj->r2[14] = (short)energyClip;

  return result;
}

/*
 * The generators below pass FPM_phasor() / FPM_phasor_demod() a four-short block `ph` that must be one array: [0] phase (in/out), [1] cosine out,
 * [2] sine out, [3] phase increment (set once before the loop and left unchanged).
 */

/**
 * @brief Generate `count` samples of the tone (sine), amplitude r0.s[1] (Q14 factor).
 * @param toneObj  tone object
 * @param out      output samples
 * @param count    number of samples
 * @return `count`
 *
 * The phase continues from the previous call.  Every call adds count / 8 (integer division) to the elapsed counter r1.s[10]; when it reaches the
 * reversal interval r0.s[2] (> 0) it restarts and the stored phase is advanced by half a turn (0x4000), a 180 degree phase reversal that takes
 * effect from the next call.
 */
int FPM_TONE_generate(FpmTone* toneObj, short* out, short count)
{
  short elapsed;
  int wrapped;
  int total;
  short ph[4];

  total = (int)count;
  ph[0] = toneObj->r1.s[8];
  ph[3] = toneObj->r1.s[9];

  if (count != 0)
  {
    do
    {
      count = count + -1;
      FPM_phasor(ph);
      *out = (short)(((int)toneObj->r0.s[1] * (int)ph[2]) >> 0xe);
      out = out + 1;
    } while (count != 0);
  }

  /* count the elapsed time and apply the periodic phase reversal */
  elapsed = (short)(total >> 3) + toneObj->r1.s[10];
  if (toneObj->r0.s[2] <= elapsed && 0 < toneObj->r0.s[2])
  {
    toneObj->r1.s[10] = 0;
    wrapped = ph[0] + 0x4000;
    if (0x7fff < wrapped)
    {
      wrapped = ph[0] + -0x4000;
    }
    toneObj->r1.s[8] = (short)wrapped;
    return total;
  }
  toneObj->r1.s[10] = elapsed;
  toneObj->r1.s[8] = ph[0];

  return total;
}

/**
 * @brief Generate `count` samples of cosine and sine of the tone, amplitude r0.s[1]; no phase reversal.
 * @param toneObj  tone object
 * @param outCos   cosine samples
 * @param outSin   sine samples
 * @param count    number of samples
 * @return `count`
 */
int FPM_TONE_generate2(FpmTone* toneObj, short* outCos, short* outSin, short count)
{
  short left;
  short ph[4];

  left = count + -1;
  ph[0] = toneObj->r1.s[8];
  ph[3] = toneObj->r1.s[9];

  if (count != 0)
  {
    do
    {
      FPM_phasor(ph);
      *outCos = (short)(((int)toneObj->r0.s[1] * (int)ph[1]) >> 0xe);
      outCos = outCos + 1;
      *outSin = (short)(((int)toneObj->r0.s[1] * (int)ph[2]) >> 0xe);
      left = left + -1;
      outSin = outSin + 1;
    } while (left != -1);
  }

  toneObj->r1.s[8] = ph[0];
  return (int)count;
}

/**
 * @brief Generate `count` cosine samples with FPM_phasor_demod(), amplitude r0.s[1]; no phase reversal.
 * @param toneObj  tone object
 * @param out      output samples
 * @param count    number of samples
 * @return `count`
 */
int FPM_TONE_generate_demod(FpmTone* toneObj, short* out, short count)
{
  int total;
  short ph[4];

  total = (int)count;
  ph[0] = toneObj->r1.s[8];
  ph[3] = toneObj->r1.s[9];

  if (count != 0)
  {
    do
    {
      count = count + -1;
      FPM_phasor_demod(ph);
      *out = (short)(((int)toneObj->r0.s[1] * (int)ph[1]) >> 0xe);
      out = out + 1;
    } while (count != 0);
  }

  toneObj->r1.s[8] = ph[0];
  return total;
}

/**
 * @brief Remove the tone from a block with the notch (coefficients `killCoef`, state r3), in place.
 * @param toneObj  tone object
 * @param samples  samples
 * @param count    number of samples
 */
void FPM_TONE_kill(FpmTone* toneObj, short* samples, short count) { FPM_iir_filt_II(samples, toneObj->killCoef, &toneObj->r3[0], 1, count); }

/**
 * @brief Retune the generator.
 * @param toneObj  tone object
 * @param freq     frequency in Hz (8 kHz sample clock); the phase increment becomes (freq * 0x8312 + 0x1000) >> 13
 * @note  Only the generator changes; the band-pass and notch coefficients keep the frequency given at creation.
 */
void FPM_TONE_set_freq(FpmTone* toneObj, short freq) { toneObj->r1.s[9] = (short)((freq * 0x8312 + 0x1000) >> 0xd); }

/**
 * @brief Set the generator amplitude (r0.s[1]; output = scale * unit sine >> 14).
 * @param toneObj  tone object
 * @param scale    amplitude
 */
void FPM_TONE_set_scale(FpmTone* toneObj, short scale) { toneObj->r0.s[1] = scale; }
