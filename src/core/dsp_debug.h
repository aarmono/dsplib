/**
 * @file dsp_debug.h
 * @brief dbgOn(): the debug-print switch of the C++ classes.
 *
 * Diagnostics are printed when the host's debug level (dsplibs_debug_level, a variable of the host) is above 1.  The C files use LEVEL_ON()
 * (core/dsp_debug_level.h) or DBG() (core/dsp_debug_printf.h) for the same test.
 */
#ifndef DSP_DEBUG_H
#define DSP_DEBUG_H

#include "core/dsp_host.h"

/** True when the host's debug level is above 1; the level is read afresh at every call. */
static inline bool dbgOn() { return dsplibs_debug_level > 1; }

#endif /* DSP_DEBUG_H */
