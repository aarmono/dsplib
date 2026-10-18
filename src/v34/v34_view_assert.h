/**
 * @file v34_view_assert.h
 * @brief V34VIEW_ASSERT(cond, msg): the compile-time layout assertion used by the generated V.34 view headers.
 *
 * The views are included by C and by C++ (a C++ class is the struct when it is the one definition of the object): static_assert in C++, _Static_assert in C.
 * The seven generated headers (v34_object_view.h, v34_flomodem_view.h, v34_k56flex_view.h, v34_v90params_view.h, v34_v90_phase2_view.h,
 * v34_v92_phase2_view.h, v34_subobjects.h) include this file instead of each carrying the block (tools/tier_rewrite.py, ASSERT_MACRO).  Idempotent
 * through the #ifndef: the block may be reached through several views.
 */
#ifndef V34VIEW_ASSERT
#ifdef __cplusplus
#define V34VIEW_ASSERT(cond, msg) static_assert(cond, msg)
#else
#define V34VIEW_ASSERT(cond, msg) _Static_assert(cond, msg)
#endif
#endif
