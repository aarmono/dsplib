/**
 * @file v92_phase4_steps.h
 * @brief stepOnes() / stepZeros(): the constant-bit data steps of the V92Phase4Modulator generators.
 *
 * Header-only helpers (`static inline`) included by v92_phase4_modulator.cpp.  Each step feeds all-ones or all-zeros scrambled bits through the
 * bits-to-symbol queue (V92BitsToSymbol) and returns the symbol it produces.
 */
#ifndef V92_PHASE4_STEPS_H
#define V92_PHASE4_STEPS_H

#include "v92_phase4_modulator.h"

/**
 * @brief One symbol of scrambled ones.
 * @param t  modulator; its queue says how many bits the next symbol needs, and bits_ is used as the bit buffer
 * @return the next symbol of the queue
 */
static inline short stepOnes(V92Phase4Modulator* t)
{
  short out = 0;
  unsigned n = t->b2s_->nofBitsForNextTime();
  if (n != 0)
  {
    t->scr_.processAllOnes(t->bits_, n);
    t->b2s_->process(t->bits_, n);
  }
  t->b2s_->process(n, &out);
  return out;
}

/**
 * @brief One symbol of scrambled zeros.
 * @param t  modulator; see stepOnes()
 * @return the next symbol of the queue
 */
static inline short stepZeros(V92Phase4Modulator* t)
{
  short out = 0;
  unsigned n = t->b2s_->nofBitsForNextTime();
  if (n != 0)
  {
    t->scr_.processAllZeros(t->bits_, n);
    t->b2s_->process(t->bits_, n);
  }
  t->b2s_->process(n, &out);
  return out;
}

#endif /* V92_PHASE4_STEPS_H */
