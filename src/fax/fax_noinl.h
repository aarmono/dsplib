/**
 * @file fax_noinl.h
 * @brief NOINL, the attribute of the fax modem helper functions that must stay real calls.
 *
 * Helpers marked NOINL are never inlined: the generated code of their callers (and so the order of its loads and stores, which matters for the
 * aliasing-independent code of the fax files) depends on every helper remaining one body reached by a call.  It is an attribute macro, not a function.
 *
 * There is no include guard on purpose (identical redefinitions are legal; see core/dsp_debug_level.h).
 */
#define NOINL __attribute__((noinline))
