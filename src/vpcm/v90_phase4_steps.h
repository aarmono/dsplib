/**
 * @file v90_phase4_steps.h
 * @brief Data steps of the V90Phase4Modulator generators: scrambled ones, scrambled zeros, one MP frame, one CP frame.
 *
 * Small `static inline` helpers included by v90_phase4_modulator.cpp and v90_phase4_modulator_symbol.cpp.  Every step has the same shape: ask the
 * bit-to-symbol queue (`b2s_`) how many bits it needs for the next symbol; when that is non-zero, scramble the bits into `bits_` and queue them; then take
 * one symbol out of the queue.
 *
 * @note The step is "one symbol per call": the MP / CP steps queue the whole frame (`mpLen_` / `cpLen_` bits) whenever the queue asks for any bits, not just
 *       the requested count.
 */
#ifndef V90_PHASE4_STEPS_H
#define V90_PHASE4_STEPS_H

#include "v90_phase4_modulator.h"

/** @brief One symbol of a scrambled all-ones stream (TRN2d, B1d, terminated); scrambles and queues the requested bit count first. */
static inline short stepOnes(V90Phase4Modulator* t)
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

/** @brief One symbol of a scrambled all-zeros stream (Ed). */
static inline short stepZeros(V90Phase4Modulator* t)
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

/** @brief One symbol of the scrambled MP frame: when the queue asks for bits, scramble all `mpLen_` bits of `mpBits_` into `bits_` and queue them. */
static inline short stepMp(V90Phase4Modulator* t)
{
  short out = 0;
  unsigned n = t->b2s_->nofBitsForNextTime();
  if (n != 0)
  {
    t->scr_.process(t->mpBits_, t->bits_, t->mpLen_);
    t->b2s_->process(t->bits_, t->mpLen_);
  }
  t->b2s_->process(n, &out);
  return out;
}

/** @brief One symbol of the scrambled CP frame (SUVd / CPd): as stepMp() with `cpBits_` / `cpLen_`. */
static inline short stepCp(V90Phase4Modulator* t)
{
  short out = 0;
  unsigned n = t->b2s_->nofBitsForNextTime();
  if (n != 0)
  {
    t->scr_.process(t->cpBits_, t->bits_, t->cpLen_);
    t->b2s_->process(t->bits_, t->cpLen_);
  }
  t->b2s_->process(n, &out);
  return out;
}

#endif /* V90_PHASE4_STEPS_H */
