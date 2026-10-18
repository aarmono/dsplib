/**
 * @file dsp_math.h
 * @brief The transcendental layer of the library: sin, cos, log10 and sqrt of extended-precision values (C++).
 *
 * The library does not use libm for these.  The V.90 / V.92 / K56Flex code evaluates them with the x87 instructions
 *     fsin   fcos   fyl2x (as `fldlg2; fxch; fyl2x`, i.e. log10)   fsqrt
 * on EXTENDED (64-bit mantissa) operands that are never rounded between the instruction and the next store.  The hardware result of fsin / fcos / fyl2x
 * differs in the last bit(s) of the extended result from every libm (glibc's sinl / cosl / log10l are software routines, musl / newlib / libgcc differ
 * again), and a different 80-bit result can occasionally change a stored float32.  The library therefore calls the INSTRUCTIONS through this header:
 *
 * - dsp_fsin(x), dsp_fcos(x): the x87 instruction on x86 / x86-64 (the hardware result);
 * - dsp_fyl2x(y, x): y * log2(x) by the instruction;
 * - dsp_log10(x): fyl2x(log10(2), x) with the exact `fldlg2` constant;
 * - dsp_sqrt(x): the IEEE correctly rounded square root of an extended value (`fsqrt`; every libm sqrtl is correctly rounded too).
 *
 * When DSP_MATH_PORTABLE is defined (or on a host that is neither i386 nor x86-64) they go through the C library instead: sinl / cosl / log2l
 * (DSP_MATH_PORTABLE=1, the default of -DDSP_MATH_PORTABLE) or the double functions sin / cos / log2 (DSP_MATH_PORTABLE=2).  The portable fallback
 * differs from the instruction in the last bit of the extended result.  Everything that is not one of these instructions (+ - * / sqrt, conversions)
 * is IEEE and identical everywhere, provided `long double` is the x87 80-bit type.
 * @warning On a host where `long double` is binary128 or plain double the float results are not bit-exact against the x87 results.
 *
 * Domain behaviour of the instructions, kept in both implementations: fsin / fcos with |x| >= 2^63 leave the operand UNCHANGED (no reduction).
 * NaN in gives a quiet NaN out; +-inf in gives the default NaN (negative quiet NaN, 0xffc00000 as a float) on the x87; the portable versions return
 * the C library's NaN (sign and payload may differ).
 */
#ifndef DSP_MATH_H
#define DSP_MATH_H

#include "dsp_asm.h"

#if !defined(DSP_MATH_PORTABLE) && (defined(__i386__) || defined(__x86_64__))
#define DSP_MATH_X87 1
#else
#define DSP_MATH_X87 0
#endif

#if DSP_MATH_X87

/** sin(x) by the `fsin` instruction (x unchanged for |x| >= 2^63). */
static inline long double dsp_fsin(long double x)
{
  long double r = x;
  X87_SIN(r);
  return r;
}
/** cos(x) by the `fcos` instruction (x unchanged for |x| >= 2^63). */
static inline long double dsp_fcos(long double x)
{
  long double r = x;
  X87_COS(r);
  return r;
}
/** y * log2(x) by the `fyl2x` instruction (st(0) = x, st(1) = y). */
static inline long double dsp_fyl2x(long double y, long double x)
{
  long double r;
  X87_YL2X(r, x, y);
  return r;
}

#else                      /* ---- portable fallback: the C library (differs from the instruction in the last bit of the extended result) ---- */

extern "C"
{
  long double sinl(long double);
  long double cosl(long double);
  long double log2l(long double);
  double sin(double);
  double cos(double);
  double log2(double);
}
#if DSP_MATH_PORTABLE == 2 /* the DOUBLE libm functions: what a host with a 53-bit libm (or a plain `sin()` call) gets */
static inline long double dsp_fsin(long double x) { return (x >= 9223372036854775808.0L || x <= -9223372036854775808.0L) ? x : (long double)sin((double)x); }
static inline long double dsp_fcos(long double x) { return (x >= 9223372036854775808.0L || x <= -9223372036854775808.0L) ? x : (long double)cos((double)x); }
static inline long double dsp_fyl2x(long double y, long double x) { return y * (long double)log2((double)x); }
#else
static inline long double dsp_fsin(long double x) { return (x >= 9223372036854775808.0L || x <= -9223372036854775808.0L) ? x : sinl(x); }
static inline long double dsp_fcos(long double x) { return (x >= 9223372036854775808.0L || x <= -9223372036854775808.0L) ? x : cosl(x); }
static inline long double dsp_fyl2x(long double y, long double x) { return y * log2l(x); }
#endif

#endif

/** log10(2) rounded to the 64-bit mantissa: the value the `fldlg2` instruction loads. */
#define DSP_LG2 0.30102999566398119521373889472449302677L

/** log10(x) as `fldlg2; fxch st(1); fyl2x` does it: dsp_fyl2x(log10(2), x). */
static inline long double dsp_log10(long double x) { return dsp_fyl2x(DSP_LG2, x); }

/** Square root of an extended value by `fsqrt` (correctly rounded on every IEEE host). */
static inline long double dsp_sqrt(long double x)
{
#if DSP_MATH_X87
  long double r = x;
  X87_SQRT(r);
  return r;
#else
  return __builtin_sqrtl(x);
#endif
}

/**
 * Float -> int32 as `fistp DWORD` does it with the truncating control word: truncation toward zero; NaN and out-of-range values give the "integer
 * indefinite" 0x80000000.  Defined behaviour in C++ on every host.
 */
static inline int dsp_trunc_i32(long double v)
{
  if (!(v > -2147483649.0L && v < 2147483648.0L))
    return (int)0x80000000u;
  return (int)v;
}

/**
 * Float-returning free functions (sum / mean / sqrSum / Var / Std of the dsp classes) leave an UNROUNDED extended value in st(0) on i386: the x87
 * return register is the only place such a float result is "stored".  A plain `return (float)v` would round it (`fstps` / `flds`), so these exports
 * use dsp_ret_ext_float(), whose asm declares the value to be the float result without touching it.  It only works on optimised code (at -O0 the
 * compiler spills the temporary as a float), so the definitions that use it carry the function-level attribute DSP_EXTRET_FN
 * (`optimize("O2")`; a #pragma does not reach template instantiations).  On x86-64 the result is returned in xmm0 and is therefore rounded; callers
 * that need the extended value call the long-double-returning variants (sumLD ...) instead.
 */
#if defined(__i386__) && !defined(DSP_MATH_PORTABLE)
#define DSP_EXTRET_FN __attribute__((optimize("O2")))
static inline __attribute__((always_inline)) float dsp_ret_ext_float(long double v)
{
  float r;
  X87_PASS_AS_RESULT(r, v);
  return r;
}
#else
#define DSP_EXTRET_FN
static inline float dsp_ret_ext_float(long double v) { return (float)v; }
#endif

#endif /* DSP_MATH_H */
