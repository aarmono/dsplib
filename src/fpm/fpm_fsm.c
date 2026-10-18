/**
 * @file fpm_fsm.c
 * @brief FPM_FSM: frequency-shift (FSK) modulator built on a tone generator.
 *
 * Owns the FpmFsm state (fpm.h): the two tone increments for bit 0 and bit 1, the samples per bit, the amplitude, and a nested FPM_TONE generator
 * (fpm_tone.c) that does the actual synthesis.  FPM_FSM_init() configures the object, FPM_FSM_modulate() turns a bit array into samples,
 * FPM_FSM_delete() frees the nested generator.  Used by the V.21 / Bell 103 and the fax V.21 transmitters; their MRF filter (fpm_mrf.c) shapes the output.
 */
#include "fpm_tables.h"
#include "fpm.h"

/**
 * @brief Configure a modulator.
 * @param fsm     modulator; `fsm->tone` must be NULL (a fresh generator is allocated) or a generator from an earlier init (re-initialised)
 * @param fsmCfg  two configuration words stored in FpmFsm::c (shorts: [0] / [1] frequency for bit 0 / bit 1, [2] samples per bit, [3] amplitude)
 *
 * The two frequencies are scaled by 0x471c / 2^14 (about 1.111) into the tone increments `f[0]` and `f[1]` handed to FPM_TONE_set_freq().  The nested
 * generator is created from the default tone block FPM_TONE_CFG_raw with the upper half of its word 0 (the amplitude short) replaced by `c.s[3]`.
 */
void FPM_FSM_init(FpmFsm* fsm, const int* fsmCfg)
{
  int cfg[9];
  FpmTone* toneObj;

  fsm->c.w[0] = fsmCfg[0];
  fsm->c.w[1] = fsmCfg[1];
  fsm->f[0] = (short)((fsm->c.s[0] * 0x471c) >> 0xe);
  fsm->f[1] = (short)((fsm->c.s[1] * 0x471c) >> 0xe);

  cfg[0] = (fsm->c.s[3] << 16) | (FPM_TONE_CFG_raw[0] & 0xffff);
  cfg[1] = FPM_TONE_CFG_raw[1];
  cfg[2] = FPM_TONE_CFG_raw[2];
  cfg[3] = FPM_TONE_CFG_raw[3];
  cfg[4] = FPM_TONE_CFG_raw[4];
  cfg[5] = FPM_TONE_CFG_raw[5];
  cfg[6] = FPM_TONE_CFG_raw[6];
  cfg[7] = FPM_TONE_CFG_raw[7];
  cfg[8] = FPM_TONE_CFG_raw[8];

  toneObj = FPM_TONE_create(fsm->tone, cfg);
  fsm->tone = toneObj;
}

/**
 * @brief Modulate a bit array.
 * @param fsm     modulator (FPM_FSM_init())
 * @param bits    one bit per 16-bit word; bit 0 of each word selects the frequency (0 -> f[0], 1 -> f[1])
 * @param out     output samples; room for nBits * c.s[2] entries is needed
 * @param nBits   number of bits
 * @return number of samples written (nBits * samples per bit, as a short)
 *
 * For each bit the generator is retuned and scaled, then produces `c.s[2]` samples one at a time.  The generator keeps its phase across bits, so the
 * output is phase-continuous.
 */
short FPM_FSM_modulate(FpmFsm* fsm, unsigned short* bits, short* out, short nBits)
{
  short scale;
  short samplesPerBit;
  unsigned short bit;
  short k;
  short total;

  total = 0;
  scale = fsm->c.s[3];
  samplesPerBit = fsm->c.s[2];

  for (nBits = nBits - 1; nBits != -1; nBits = nBits - 1)
  {
    bit = *bits;
    bits = bits + 1;
    FPM_TONE_set_freq(fsm->tone, fsm->f[bit & 1]);
    FPM_TONE_set_scale(fsm->tone, scale);
    total = total + samplesPerBit;
    for (k = samplesPerBit; k != 0; k = k - 1)
    {
      FPM_TONE_generate(fsm->tone, out, 1);
      out = out + 1;
    }
  }

  return total;
}

/**
 * @brief Free the nested tone generator.
 * @param fsm  modulator, or NULL (ignored)
 */
void FPM_FSM_delete(FpmFsm* fsm)
{
  if (fsm != (FpmFsm*)0)
  {
    FPM_TONE_delete(fsm->tone);
  }
}
