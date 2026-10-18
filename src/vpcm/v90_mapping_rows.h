/**
 * @file v90_mapping_rows.h
 * @brief LA() / LB(): the six lists of a V90MappingParams block as flat byte rows (C++).
 *
 * The lists sit at byte offsets 4 + 0x80 * p + i (list A) and 0x304 + 0x80 * p + i (list B) of the block.  An index up to 0x80 (a list of 128 + 1
 * entries: the overflow test is `count > 0x80`) runs into the next row, which a [6][128] member subscript must not do, hence the byte-offset access.
 * The functions are `static inline` so that every including file has its own copy.
 */
#ifndef V90_MAPPING_ROWS_H
#define V90_MAPPING_ROWS_H

#include <stddef.h>
#include <stdint.h>
#include "v92_cp.h"
#include "core/dsp_cxx.h"

DSP_STATIC_ASSERT(offsetof(V90MappingParams, listA_) == 0x4 && offsetof(V90MappingParams, listB_) == 0x304 && offsetof(V90MappingParams, cnt_) == 0x604 &&
                      offsetof(V90MappingParams, sr_) == 0x620,
                  "the flat list rows of LA / LB are the expected offsets of the mapping block");

/** @brief Entry @p i of list A of constellation @p p, addressed flat from the block start; @p i may reach 0x80 and then lands in the next row. */
static inline uint8_t& LA(V90MappingParams* mp, unsigned p, unsigned i) { return ((uint8_t*)mp)[4u + p * 0x80u + i]; }
/** @brief Entry @p i of list B of constellation @p p, addressed flat from the block start; @p i may reach 0x80 and then lands in the next row. */
static inline uint8_t& LB(V90MappingParams* mp, unsigned p, unsigned i) { return ((uint8_t*)mp)[0x304u + p * 0x80u + i]; }

#endif /* V90_MAPPING_ROWS_H */
