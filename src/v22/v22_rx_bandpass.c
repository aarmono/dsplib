/**
 * @file v22_rx_bandpass.c
 * @brief V.22 / V.22bis receive-side IIR band filter and demodulating gain: V22IIRFilterInit() and V22_iir_filt_demod().
 *
 * Used by the originating side (core.rxChannel == 2), which receives the high channel: DemodDataV22() generates the demodulation tone and passes it as the
 * per-sample gain, so the filter output is also mixed with that tone.
 */
#include "v22.h"

/**
 * @brief Initialise a filter section: copy 4 numerator and 4 denominator coefficients and clear the 4-sample input and output histories.
 * @param f    section
 * @param num  4 numerator coefficients (Q14)
 * @param den  4 denominator coefficients (Q14; den[0] is not used by the filter)
 */
void V22IIRFilterInit(V22Iir* f, const short* num, const short* den)
{
  int k;
  for (k = 0; k <= 3; k++)
    f->num[k] = num[k];
  for (k = 0; k <= 3; k++)
    f->den[k] = den[k];
  for (k = 0; k <= 3; k++)
    f->x[k] = 0;
  for (k = 0; k <= 3; k++)
    f->y[k] = 0;
}

/**
 * @brief Filter 160 samples in place and multiply them by a per-sample gain.
 * @param data  160 samples (in and out)
 * @param num   4 numerator coefficients (Q14)
 * @param den   denominator coefficients; den[1 .. 3] are used (Q14)
 * @param xh    input history, 4 shorts (state)
 * @param yh    output history, 3 shorts used (state)
 * @param gain  160 per-sample gains (the demodulation tone from FPM_TONE_generate_demod())
 *
 * Per sample: acc = sum over k < 4 of (num[k] * xh[k] >> 14) minus the sum over k < 3 of (den[k + 1] * yh[k] >> 14) (every product shifted on its own,
 * 32-bit accumulator); the histories shift, yh[0] = (short)acc and data[i] = ((short)acc * gain[i]) >> 12.
 */
void V22_iir_filt_demod(short* data, const short* num, const short* den, short* xh, short* yh, const short* gain)
{
  int i;
  for (i = 0; i <= 0x9f; i++)
  {
    unsigned int acc = 0;
    int k;
    xh[0] = data[i];
    for (k = 0; k <= 3; k++)
      acc += (unsigned int)(((int)num[k] * (int)xh[k]) >> 14);
    for (k = 0; k <= 2; k++)
      acc -= (unsigned int)(((int)den[k + 1] * (int)yh[k]) >> 14);
    for (k = 3; k > 0; k--)
      xh[k] = xh[k - 1];
    for (k = 2; k > 0; k--)
      yh[k] = yh[k - 1];
    yh[0] = (short)acc;
    data[i] = (short)(((int)(short)acc * (int)gain[i]) >> 12);
  }
}
