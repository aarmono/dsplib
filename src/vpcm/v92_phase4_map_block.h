/**
 * @file v92_phase4_map_block.h
 * @brief mapBlock(): pass one scrambled bit block of the V92Phase4Modulator through the V92Mapper.
 *
 * A header-only helper (`static inline`) included by v92_phase4_modulator.cpp and v92_phase4_modulator_symbol.cpp.
 */
#ifndef V92_PHASE4_MAP_BLOCK_H
#define V92_PHASE4_MAP_BLOCK_H

#include "v92_phase4_modulator.h"

/**
 * @brief Map the block in bits_ to one symbol.
 * @param t  modulator; mult_ bits of bits_ are mapped, inv_ and the last byte of the block are updated
 * @return the symbol from V92Mapper::process()
 * @note  The last bit of the block is XORed with the 32-bit inv_ (the result is also stored back into inv_), so the next differential step starts from it.
 *        mult_ and inv_ are read after the scrambler has run.
 */
static inline short mapBlock(V92Phase4Modulator* t)
{
  unsigned m = t->mult_;
  unsigned char* last = t->bits_ + m - 1;
  unsigned v = (unsigned)*last ^ t->inv_;
  t->inv_ = v;
  *last = (unsigned char)v;
  return t->mapper_->process(t->bits_);
}

#endif /* V92_PHASE4_MAP_BLOCK_H */
