/**
 * @file dsp_cxx.h
 * @brief Common definitions for the C++ half of the library (the V.90 / V.92 / K56Flex classes, the generic DSP classes, the V.34 handshake pieces).
 *
 * The C++ sources are compiled with `-fno-exceptions -fno-rtti` and have no libstdc++ dependency: no RTTI, no exception tables, no operator new /
 * delete and no libstdc++ symbols.  Only freestanding headers (<stddef.h>, <stdint.h>, <string.h>) may be included from the C++ files and from the
 * headers they include.  <cstdint>, <new>, <array>, ... are not allowed (a 32-bit host has no 32-bit C++ headers; the 32-bit test binaries are
 * linked with `gcc -m32 -no-pie`, never `g++`).  The C++ sources stay within C++98 plus the DSP_STATIC_ASSERT macro below.
 *
 * Memory is allocated with the host's sysdep_malloc() / sysdep_free() (core/dsp_host.h), never with operator new.
 *
 * Provides: DSP_STATIC_ASSERT(), a placement operator new, and DSP_ASSERT_LAYOUT_I386().
 */
#ifndef DSP_CXX_H
#define DSP_CXX_H

#include <stddef.h>
#include <stdint.h>
#include "core/dsp_host.h" /* sysdep_malloc / sysdep_free (and the other host functions) */

#define DSP_CAT2_(a, b) a##b
#define DSP_CAT_(a, b) DSP_CAT2_(a, b)

/** Compile-time assertion: `static_assert` on C++11 compilers, the negative-array-size trick otherwise (`msg` is then unused). */
#if defined(__cplusplus) && __cplusplus >= 201103L
#define DSP_STATIC_ASSERT(cond, msg) static_assert(cond, msg)
#else
#define DSP_STATIC_ASSERT(cond, msg) typedef char DSP_CAT_(dsp_static_assert_, __LINE__)[(cond) ? 1 : -1]
#endif

/**
 * Placement new, without <new> (a freestanding header the library must not include).  A class that constructs a sub-object in a block it allocated
 * with sysdep_malloc() writes `new (p) Class(args)`; the allocation result is not tested, as everywhere in the library.
 * @warning Deliberately NOT declared throw() / noexcept: a non-throwing placement new makes the compiler test the pointer for NULL and skip the
 *          constructor, which would change behaviour for the (unchecked) failed allocation.
 */
inline void* operator new(size_t, void* p) { return p; }

/**
 * DSP_ASSERT_LAYOUT_I386(cond, msg): assertion on the 32-bit image layout of a class (sizeof / offsetof of the objects and of the sub-objects
 * embedded in their owners).  Active only when the translation unit is compiled for 32-bit x86 (`__i386__`, where DSP_LAYOUT_I386 is 1); on any
 * other target it expands to an assertion that always holds and DSP_LAYOUT_I386 is 0.  The numbers are valid for the 32-bit ABI only: pointer
 * members are 4 bytes there and 8 bytes on x86-64, where the native layout differs.
 */
#if defined(__i386__)
#define DSP_LAYOUT_I386 1
#define DSP_ASSERT_LAYOUT_I386(cond, msg) DSP_STATIC_ASSERT(cond, msg)
#else
#define DSP_LAYOUT_I386 0
#define DSP_ASSERT_LAYOUT_I386(cond, msg) DSP_STATIC_ASSERT(1, msg)
#endif

#endif /* DSP_CXX_H */
