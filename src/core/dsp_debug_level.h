/**
 * @file dsp_debug_level.h
 * @brief LEVEL_ON(): the debug-print guard of the plain-C files and the C-style C++ tier files.
 *
 * Diagnostics are printed when the host's level (dsplibs_debug_level) is above 1, an UNSIGNED comparison, and the level is read afresh at every
 * guard: LEVEL_ON() is a macro (a function would be a call at -O0), with no cached value.  The C++ classes use dbgOn() of core/dsp_debug.h (a
 * static inline function: an older spelling of the same test).
 *
 * There is no include guard on purpose: the text is identical at every inclusion (a repeated identical #define or extern declaration is legal), and
 * the test copies that #undef a macro and include a library source a second time need it defined again (see v34_access.h).
 */
#include "core/dsp_host.h" /* the extern declaration of dsplibs_debug_level */

#define LEVEL_ON() (dsplibs_debug_level > 1u)
