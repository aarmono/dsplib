/**
 * @file fax_debug.h
 * @brief The debug-print guards DBG and DBG3 of the fax files.
 *
 * The fax modems print their diagnostics through the host's dsplibs_debug_printf() when the host debug level (dsplibs_debug_level, core/dsp_host.h)
 * is above 1 (DBG) or above 2 (DBG3).  The compare is written `(1 < level)`; core/dsp_debug_level.h's LEVEL_ON() is the `level > 1u` spelling used by
 * the V.34 / V.PCM files and is a different expression, so it is a different macro.  These are macros, not functions: the level is read afresh at every use.
 *
 * There is no include guard on purpose (identical redefinitions are legal; see core/dsp_debug_level.h).
 */
#include "core/dsp_host.h"

#define DBG (1 < dsplibs_debug_level)
#define DBG3 (2 < dsplibs_debug_level)
