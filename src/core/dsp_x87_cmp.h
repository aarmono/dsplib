/**
 * @file dsp_x87_cmp.h
 * @brief x87-style conditional jumps after a floating-point compare, as C++ predicates on long double.
 *
 * After an x87 compare (`fcom a, b; fnstsw ax; sahf`, a = st(0)) the unsigned-compare jumps read CF = (a < b or unordered) and ZF = (a == b or
 * unordered).  A NaN operand therefore takes jb / jbe and does not take ja / jae; the predicates below are written exactly so.
 */
#ifndef DSP_X87_CMP_H
#define DSP_X87_CMP_H

typedef long double LD;

static inline bool jmpB(LD a, LD b) { return !(a >= b); } /**< jb: taken when a < b or unordered (CF) */
static inline bool jmpAe(LD a, LD b) { return a >= b; }   /**< jae: taken when a >= b (not below, not unordered) */
static inline bool jmpA(LD a, LD b) { return a > b; }     /**< ja: taken when a > b (ordered greater) */
static inline bool jmpBe(LD a, LD b) { return !(a > b); } /**< jbe: taken when a <= b or unordered (CF || ZF, NaN included) */

#endif /* DSP_X87_CMP_H */
