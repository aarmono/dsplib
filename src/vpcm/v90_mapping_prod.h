/**
 * @file v90_mapping_prod.h
 * @brief prodCnt(), kEps6 and lg2F(): helpers for the constellation-size arithmetic on a V90MappingParams block (C++).
 *
 * - prodCnt() is the product of the six constellation sizes of a V90MappingParams, accumulated in extended precision.
 * - kEps6 is the "+1e-6" guard added to the integer number of bits K before it is divided or logarithmed.
 * - lg2F() is log10(2) rounded to a float.
 *
 * The functions are `static inline` so that every including translation unit gets its own copy; the including files are the constellation designer
 * / noise / power / dmin files and the TRN2 designer.
 */
#ifndef V90_MAPPING_PROD_H
#define V90_MAPPING_PROD_H

#include "v92_cp.h"
#include "core/dsp_floatbits.h"
#include "core/dsp_math.h"

/** @brief Product of the six constellation sizes cnt_[0..5] of @p mp, each zero-extended to 64 bits, multiplied in extended precision (LD). */
static inline LD prodCnt(const V90MappingParams* mp)
{
  LD p = (LD)mp->cnt_[0];
  p = p * (LD)mp->cnt_[1];
  p = p * (LD)mp->cnt_[2];
  p = p * (LD)mp->cnt_[3];
  p = p * (LD)mp->cnt_[4];
  p = p * (LD)mp->cnt_[5];
  return p;
}
/** The +1e-6 guard added to the integer K: the float 9.99999997e-07 (bitsFloat() of dsp_floatbits.h); also used by the constellation power / dmin files
 *  and the TRN2 designer. */
#define kEps6 (bitsFloat(0x358637bdu))
/** @brief log10(2) rounded to a float (the extended-precision value is stored to a float slot and read back, so the result is a float value). */
static inline float lg2F()
{
  float slot;
  return storeF(&slot, dsp_log10(2.0L));
}

#endif /* V90_MAPPING_PROD_H */
