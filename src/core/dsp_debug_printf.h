/**
 * @file dsp_debug_printf.h
 * @brief DBG(...): the guarded debug print of the plain-C callprog / voice / fax API files.
 *
 * `DBG(fmt, ...)` prints through the host's dsplibs_debug_printf() when the host's level is above 1 (unsigned compare, the level read afresh).  It is
 * a macro, so the arguments are evaluated only when the level allows it.  Used by callprog.c, dp_call.c, voice_api.c, voice_engine.c and fax_api.c.
 * Not used by the fax tier files (fax_debug.h defines DBG as an object-like macro there) nor by tone_cadence.c / tone_iir.c (DBG(n) there is a
 * level test with a parameter).
 *
 * There is no include guard on purpose (identical redefinitions are legal; see core/dsp_debug_level.h).
 */
#include "core/dsp_debug_level.h"
#include "core/dsp_host.h" /* dsplibs_debug_printf */

#define DBG(...)                                                                                                                                               \
  do                                                                                                                                                           \
  {                                                                                                                                                            \
    if (dsplibs_debug_level > 1u)                                                                                                                              \
      dsplibs_debug_printf(__VA_ARGS__);                                                                                                                       \
  } while (0)
