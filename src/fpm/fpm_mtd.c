/**
 * @file fpm_mtd.c
 * @brief FPM_MTD: energy-ratio tone detector (a DC blocker followed by a configurable IIR cascade).
 *
 * Owns the FpmMtd state (fpm.h).  FPM_MTD_create() builds or re-initialises a detector from a three-word configuration block, FPM_MTD_detect() runs it over a
 * block of samples and returns a three-valued result, FPM_MTD_delete() frees it.  Each input sample is scaled down, DC-blocked with the fixed one-section
 * filter COEF_DC (fpm_tables.c) and then filtered by the configured cascade (FPM_iir_filt(), fpm_iir.c); the detector compares the smoothed energy of
 * the cascade output with that of its input.  Users: the Bell 103 / V.21, V.22, V.32 and fax receivers (carrier and tone detection).
 */

#include "core/dsp_host.h"

#include "core/dsp_table_handles.h"
#include "fpm.h"
#include "fpm_tables.h"

/**
 * @brief Create a detector, or re-initialise an existing one.
 * @param obj  existing object to re-initialise (its filter-state buffer is kept), or NULL to allocate the object and its buffer
 * @param cfg  three configuration words, or NULL for the default block FPM_MTD_CFG_raw: [0] CFGP handle of the cascade coefficients (5 shorts per
 *             section), [1] = c1 (shorts: [0] number of biquad sections, [1] ratio threshold, Q15), [2] = c2 (short [0] minimum input energy)
 * @return the detector
 *
 * Clears the cascade state (2 shorts per section), the DC-blocker state and both energy accumulators.
 * @warning The sysdep_malloc() results are not checked.  With an existing `obj` the buffer must already hold 2 * sections shorts for the new
 *          configuration; FPM_MTD_delete() frees the object, so only objects created with obj == NULL may be deleted.
 */
FpmMtd* FPM_MTD_create(FpmMtd* obj, const int* cfg)
{
  FpmMtd* b;
  FpmMtd* o32;
  int bAlloc = (obj == (FpmMtd*)0);

  if (bAlloc)
  {
    obj = (FpmMtd*)sysdep_malloc(sizeof(FpmMtd));
  }
  b = obj;
  o32 = obj;

  /* configuration: the default block when cfg is NULL */
  if (cfg == (const int*)0)
  {
    FPM_TABLE_SET(o32, coef, const short*, FPM_MTD_CFG_raw[0]); /* table pointer: a handle on x86-64 (src/core/dsp_table_handles.h) */
    o32->c1.w = FPM_MTD_CFG_raw[1];
    o32->c2.w = FPM_MTD_CFG_raw[2];
  }
  else
  {
    FPM_TABLE_SET(o32, coef, const short*, cfg[0]);
    o32->c1.w = cfg[1];
    o32->c2.w = cfg[2];
  }

  if (bAlloc)
  {
    short order = b->c1.s[0];
    int bufBytes = (short)(order * 2) * 2;

    FPM_M(void*, b, buf) = sysdep_malloc((unsigned int)bufBytes);
  }

  /* clear the cascade state and the detector state */
  {
    short order = b->c1.s[0];

    if (0 < order)
    {
      char* buf = (char*)b->buf;
      int i = 0;

      do
      {
        *(short*)(buf + i * 4) = 0;
        *(short*)(buf + 2 + i * 4) = 0;
        i = i + 1;
      } while ((short)i < order);
    }
  }
  b->dc[0] = 0;
  b->dc[1] = 0;
  b->diffEnergy = 0;
  b->inEnergy = 0;

  return obj;
}

/**
 * @brief Run the detector over a block of samples.
 * @param obj      detector (FPM_MTD_create())
 * @param samples  input samples (not modified)
 * @param count    number of samples
 * @return 2 if the smoothed input energy is below the minimum (c2.s[0]: too little signal to decide); otherwise 1 if diffEnergy <= (c1.s[1] * inEnergy) >> 15
 *         (the cascade passes at least (1 - ratio) of the input energy) or if diffEnergy > inEnergy, else 0
 *
 * Per sample: x >> 5, the DC blocker, then the cascade.  `inEnergy` and the cascade-output energy are updated as e = (e * 0x7ccc + (y * y >> 5) * 0x334) >> 15
 * (a one-pole average, weights 0.975 / 0.025).  After the block `diffEnergy` = inEnergy - cascade energy, clamped at 0.
 * @note  The difference is truncated to 16 bits before the clamp, so a difference of 32768 or more (full-scale noise) reads as negative and gives 0.
 */
unsigned int FPM_MTD_detect(FpmMtd* obj, short* samples, short count)
{
  FpmMtd* b = obj;
  short left;
  short x;
  short inEnergy;
  short filtEnergy;
  int dcOut;
  unsigned short diffEnergy;
  unsigned int result;

  inEnergy = b->inEnergy;
  filtEnergy = inEnergy - b->diffEnergy;

  /* filter the block and accumulate the energies */
  if (count != 0)
  {
    left = count - 1;
    do
    {
      x = *samples;
      samples = samples + 1;
      x = FPM_iir_filt((short)(x >> 5), COEF_DC, &b->dc[0], 1);
      dcOut = (int)x;
      x = FPM_iir_filt((short)dcOut, b->coef, b->buf, b->c1.s[0]);
      filtEnergy = (short)((filtEnergy * 0x7ccc + ((int)x * (int)x >> 5) * 0x334) >> 0xf);
      inEnergy = (short)((inEnergy * 0x7ccc + (dcOut * dcOut >> 5) * 0x334) >> 0xf);
      if (left == 0)
        break;
      left = left - 1;
    } while (1);
  }

  /* store the state; the energy difference is truncated to 16 bits before it is clamped at 0 */
  b->inEnergy = inEnergy;
  {
    short d16 = (short)(inEnergy - filtEnergy);

    diffEnergy = (unsigned short)(d16 > 0 ? d16 : 0);
  }
  FPM_M(unsigned short, b, diffEnergy) = diffEnergy;

  /* decide */
  result = 2;
  if (b->c2.s[0] <= inEnergy)
  {
    int condA = (int)(short)diffEnergy <= (((int)b->c1.s[1] * (int)inEnergy) >> 0xf);

    if (condA)
    {
      result = 1;
    }
    else if (inEnergy < (short)diffEnergy)
    {
      result = 1;
    }
    else
    {
      result = 0;
    }
  }

  return result;
}

/**
 * @brief Free a detector created by FPM_MTD_create(NULL, ...): its buffer and the object itself.
 * @param obj  detector
 */
void FPM_MTD_delete(FpmMtd* obj)
{
  sysdep_free(FPM_M(void*, obj, buf));
  sysdep_free(obj);
}
