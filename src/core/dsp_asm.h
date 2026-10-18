/**
 * @file dsp_asm.h
 * @brief The inline-asm idioms of the library as named macros (C and C++).
 *
 * Two kinds of asm statement are used in the library: empty statements that stop the optimiser from folding, moving or caching something, and
 * single x87 instructions with a memory operand that the C spelling would compile differently (a float loaded with `flds` keeps a signalling NaN,
 * a `(long double)*m` conversion may not).  Each idiom has one descriptive macro here.  They are macros, not inline functions, on purpose: a macro
 * expands to exactly the asm statement, so the code generated at -O0 is the same, whereas a static inline function would be a call.
 *
 * Statement macros: use them as statements (`OPTIMIZER_HIDE_VAR(w);`).  The argument expressions are evaluated where the asm operand puts them,
 * exactly as in the spelled-out statement.  The x87 family is available on x86 hosts only (i386 and x86-64).
 */
#ifndef DSP_ASM_H
#define DSP_ASM_H

/* ---- optimisation barriers (no instruction is emitted) ---- */

/** `v` is in a general register here and may have changed: the compiler forgets what it knows about its value (no constant folding through it, no
 *  re-materialising it from memory; a float cell copied as a 32-bit word stays an integer move). */
#define OPTIMIZER_HIDE_VAR(v) __asm__("" : "+r"(v))

/** Memory may have been read and written here: no load or store is moved across it and no memory value is cached across it, so the order of the
 *  memory accesses does not depend on the strict-aliasing rules.  Emits no instruction. */
#define COMPILER_BARRIER() __asm__ volatile("" ::: "memory")

/** `p` is read here and memory may have been read and written through it: the stores into `*p` before this point are kept.  Used where the object
 *  dies right after (a destructor body or a delete), because lifetime dead-store elimination would drop the stores.  Emits no instruction. */
#define KEEP_STORES_TO(p) __asm__ volatile("" : : "r"(p) : "memory")

/** An instruction-less volatile statement placed behind a call: the call stays a call (no sibling-call jump) and is followed by a return. */
#define NO_TAIL_CALL() __asm__ volatile("")

/** `v` lives in memory here and may have changed: a long double is forced through its stack slot and a float is rounded to float.  A negation
 *  cannot be folded into a subtraction across it (`-a + b` -> `b - a` differs for NaN operands, because `fchs` flips the sign of a NaN). */
#define OPTIMIZER_HIDE_MEM(v) __asm__("" : "+m"(v))

/* ---- x87 (i386 / x86-64: long double is the x87 register, "t" = st(0), "u" = st(1)) ---- */
#if defined(__i386__) || defined(__x86_64__)
/** `v` is in st(0) here and may have changed: a float widened to long double is a real `fld` and a product a real `fmul` (the compiler does not
 *  narrow it to an SSE operation on x86-64). */
#define OPTIMIZER_HIDE_X87(v) __asm__("" : "+t"(v))

/** `r` (a float return value) is the unrounded st(0) value `v`: no `fstps` / `flds` pair rounds it.  Emits no instruction. */
#define X87_PASS_AS_RESULT(r, v) __asm__("" : "=t"(r) : "0"(v))

/* single x87 instructions on st(0) (the operand register is the macro's first argument) */
#define X87_SIN(v) __asm__("fsin" : "+t"(v))
#define X87_COS(v) __asm__("fcos" : "+t"(v))
#define X87_SQRT(v) __asm__("fsqrt" : "+t"(v))
#define X87_CHS(v) __asm__("fchs" : "+t"(v))
#define X87_F2XM1(v) __asm__("f2xm1" : "+t"(v))
#define X87_RNDINT(v) __asm__("frndint" : "+t"(v))
/** X87_SCALE(v, e): v = v * 2^trunc(e)                      (`fscale`, e in st(1)) */
#define X87_SCALE(v, e) __asm__("fscale" : "+t"(v) : "u"(e))
/** X87_YL2X(r, x, y):   r = y * log2(x)                     (`fyl2x`, x in st(0), y in st(1)) */
#define X87_YL2X(r, x, y) __asm__("fyl2x" : "=t"(r) : "0"(x), "u"(y) : "st(1)")
/** X87_YL2XP1(r, x, y): r = y * log2(x + 1)                 (`fyl2xp1`) */
#define X87_YL2XP1(r, x, y) __asm__("fyl2xp1" : "=t"(r) : "0"(x), "u"(y) : "st(1)")
/** X87_FISTP_I64(dst, v): dst (a 64-bit integer) = v rounded by the current x87 rounding mode (`fistpll`) */
#define X87_FISTP_I64(dst, v) __asm__("fistpll %0" : "=m"(dst) : "t"(v) : "st")
/** X87_FPREM_LOOP(v, m): v = fmod(v, m) to completion       (`fprem` repeated while the C2 flag says the reduction is partial) */
#define X87_FPREM_LOOP(v, m) __asm__("1: fprem\n\tfnstsw %%ax\n\ttestb $4, %%ah\n\tjnz 1b" : "+t"(v) : "u"(m) : "ax", "cc")
/** X87_NEG_SQUARE(r, v): r = v * (-v) as `fld st0; fchs; fmulp` (for a NaN the instruction picks the destination NaN; a C spelling is folded to -(v * v)) */
#define X87_NEG_SQUARE(r, v) __asm__("fld %%st(0)\n\tfchs\n\tfmulp %%st, %%st(1)" : "=t"(r) : "0"(v))

/* x87 arithmetic with a 32-bit float memory operand */
/** X87_LOAD_F32(st0, mem):      st0 = mem                (`flds mem`) */
#define X87_LOAD_F32(st0, mem) __asm__("flds %1" : "=t"(st0) : "m"(mem))
/** X87_ADD_F32(st0, mem):       st0 = st0 + mem          (`fadds mem`) */
#define X87_ADD_F32(st0, mem) __asm__("fadds %1" : "+t"(st0) : "m"(mem))
/** X87_MUL_F32(st0, mem):       st0 = st0 * mem          (`fmuls mem`) */
#define X87_MUL_F32(st0, mem) __asm__("fmuls %1" : "+t"(st0) : "m"(mem))
/** X87_SUBR_F32(st0, mem):      st0 = mem - st0          (`fsubrs mem`, the reversed subtraction) */
#define X87_SUBR_F32(st0, mem) __asm__("fsubrs %1" : "+t"(st0) : "m"(mem))
#endif

#endif /* DSP_ASM_H */
