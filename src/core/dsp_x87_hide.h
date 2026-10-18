/**
 * @file dsp_x87_hide.h
 * @brief Float values that must go through a real x87 register (C++): loadLD() and mulF().
 *
 * On x86-64 the compiler would fold the widening of a float, or narrow a float product to an SSE `mulss`, whose NaN rule (first operand) differs from
 * the x87's (larger significand) when both operands are NaNs.  The asm barrier keeps the value in st(0).
 */
#ifndef DSP_X87_HIDE_H
#define DSP_X87_HIDE_H

#include "dsp_asm.h"

typedef long double LD;

/** Load the float `v` as `fld DWORD` does (a signalling NaN is quieted); the barrier forces a real x87 register. */
static inline LD loadLD(float v)
{
  LD r = v;
  OPTIMIZER_HIDE_X87(r);
  return r;
}

/** The product of two floats formed on the x87 stack (`fld a; fmul b; fstp`) and rounded to float. */
static inline float mulF(float a, float b)
{
  LD p = (LD)a * (LD)b;
  OPTIMIZER_HIDE_X87(p);
  return (float)p;
}

#endif /* DSP_X87_HIDE_H */
