/**
 * @file voice_x87.h
 * @brief x87 operations with a float memory operand, for the voice tone and DTMF code.
 *
 * Combining a register with a FLOAT MEMORY operand directly (`fmul dword [m]`, `fadd dword [m]`) differs from loading the operand first when
 * both operands are NaNs: the x87 prefers the quiet NaN over a signalling-NaN memory operand, while `fld` first quiets the signalling NaN and
 * the larger significand wins, so the payloads differ.  The helpers here are the memory-operand forms (the core/dsp_asm.h macros on x86, plain C
 * elsewhere): ldl() is an explicit `fld dword`, so the compiler cannot fold the load into an operation of its own choosing.
 *
 * All helpers are `static inline`; a file includes this header at the point where the helpers are needed (the order of the definitions below is
 * the order the including files rely on).  voice_engine.c keeps its own copies with ld_subrf() added and the definitions in another order.
 */
#ifndef VOICE_X87_H
#define VOICE_X87_H

#include "core/dsp_asm.h"

/** `fld dword [m]`: an explicit load (otherwise the compiler may fold the float into a memory-operand fmul / fadd of its own choosing). */
static inline long double ldl(const float* m)
{
#if defined(__i386__) || defined(__x86_64__)
  long double r;
  X87_LOAD_F32(r, *m);
  return r;
#else
  return (long double)*m;
#endif
}
/** `a * *m` with a float memory operand. */
static inline long double ld_mulf(long double a, const float* m)
{
#if defined(__i386__) || defined(__x86_64__)
  X87_MUL_F32(a, *m);
  return a;
#else
  return a * (long double)*m;
#endif
}
/** `-a` (`fchs`). */
static inline long double ld_chs(long double a)
{
#if defined(__i386__) || defined(__x86_64__)
  X87_CHS(a);
  return a;
#else
  return -a;
#endif
}
/** `a + *m` with a float memory operand. */
static inline long double ld_addf(long double a, const float* m)
{
#if defined(__i386__) || defined(__x86_64__)
  X87_ADD_F32(a, *m);
  return a;
#else
  return a + (long double)*m;
#endif
}

#endif /* VOICE_X87_H */
