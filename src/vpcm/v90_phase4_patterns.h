/**
 * @file v90_phase4_patterns.h
 * @brief Pattern symbols of the V90Phase4Modulator: the Ri pattern and the stored six / twelve-symbol Rd / Rt and Rf tables.
 *
 * Small `static inline` helpers included by v90_phase4_modulator.cpp and v90_phase4_modulator_symbol.cpp.  Each returns the symbol for the current symbol
 * counter `cnt_` (1-based: the index is `(cnt_ - 1)` modulo the pattern period, computed as unsigned, so cnt_ == 0 selects the last entry).
 */
#ifndef V90_PHASE4_PATTERNS_H
#define V90_PHASE4_PATTERNS_H

#include "v90_phase4_modulator.h"
#include "core/dsp_intmath.h"

/**
 * @brief Ri pattern symbol: +level for the first three symbols of every six, -level for the next three.
 * @param level  linear level of the pattern (level_)
 * @param cnt    1-based symbol counter
 */
static inline short riPattern(unsigned short level, unsigned cnt)
{
  unsigned r = (cnt - 1) % 6;
  return r <= 2 ? (short)level : neg16((short)level);
}

/** @brief Symbol of the six-symbol Rd / Rt pattern (rdrt_) for the current cnt_. */
static inline short rdrtAt(const V90Phase4Modulator* t) { return (short)t->rdrt_[(t->cnt_ - 1) % 6]; }

/** @brief Symbol of the twelve-symbol Rf pattern (rf_) for the current cnt_. */
static inline short rfAt(const V90Phase4Modulator* t) { return (short)t->rf_[(t->cnt_ - 1) % 12]; }

#endif /* V90_PHASE4_PATTERNS_H */
