/**
 * @file v92_phase4_diff_out.h
 * @brief diffOut(): the differential output stage of the V92Phase4Modulator.
 *
 * A header-only helper (`static inline`) included by v92_phase4_modulator.cpp and v92_phase4_modulator_symbol.cpp.
 */
#ifndef V92_PHASE4_DIFF_OUT_H
#define V92_PHASE4_DIFF_OUT_H

#include "v92_phase4_modulator.h"
#include "core/dsp_intmath.h"

/**
 * @brief Differential output: store the new phase bit and return the corresponding two-level symbol.
 * @param t  modulator; inv_ is overwritten with v, level_ gives the magnitude
 * @param v  new inversion state (0 or non-zero)
 * @return +level_ when v is zero, -level_ otherwise
 */
static inline short diffOut(V92Phase4Modulator* t, unsigned v)
{
  t->inv_ = v;
  unsigned short l = t->level_;
  return v != 0 ? neg16((short)l) : (short)l;
}

#endif /* V92_PHASE4_DIFF_OUT_H */
