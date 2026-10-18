/**
 * @file dsp_host.h
 * @brief The functions and the one variable the library takes from its host (slmodemd): declared once, with the host's real prototypes.
 *
 * The library never defines these; they are resolved at link time against the host (the undefined symbols of the library archive are exactly this list
 * plus libc).  Include this header instead of declaring any of them locally.
 *
 * Freestanding: only <stddef.h> and <stdint.h>.  Usable from the C++ half (compiled -fno-exceptions -fno-rtti, see core/dsp_cxx.h), hence the
 * extern "C" guards.  Guarded against repeated inclusion (the macro headers of the library deliberately are not; this one holds only declarations).
 *
 * The prototypes follow slmodemd:
 * - `sysdep_*`: slmodemd/sysdep_common.c (sysdep_malloc takes `unsigned int`, the string and memory functions `size_t`);
 * - `dsplibs_debug_*`: slmodemd/modem_debug.c (dsplibs_debug_level is the host's variable, read afresh at every use);
 * - `modem_*`: slmodemd/modem.c, modem_param.c, modem_debug.c.  The ids and records they use (MDMPRM_*, enum DP_ID, struct dp) are in
 *   core/host_interface.h.
 *
 * `struct modem` stays an incomplete type here: the library only passes the pointer on.
 *
 * Not declared here:
 * - modem_dp_register / modem_dp_deregister: their prototypes need the host's enum DP_ID and struct dp_operations (core/host_interface.h).  The C files
 *   include core/host_interface.h, which declares them; src/vpcm/dp_vpcm.cpp (C++, freestanding) gets them with `int` / `void *` from the
 *   DSP_HOST_MODEM_OPAQUE branch below, on purpose.
 * - modem_debug_log_data is a macro of the host's modem_debug.h in debug-less builds; the library calls the function.
 * - the libc functions (memcpy, memmove, memset, abort, fprintf ...) come from the system headers of the files that use them; the libm declarations
 *   are in core/dsp_math.h.
 * - voice_get_param (src/voice/voice.h): the same symbol as modem_get_param under another C name (`__asm__("modem_get_param")`), kept there.
 *
 * DSP_HOST_MODEM_OPAQUE: src/vpcm/dp_vpcm.cpp keeps the modem as an opaque `void *` (struct dp::modem_ of its freestanding header); it defines this
 * macro before the include and gets the modem_get_param / modem_set_param / modem_get_bits / modem_put_bits / modem_debug_log_data family (and the
 * registration pair) declared with `void *` instead of the typed declarations (two extern "C" declarations of one name with different parameter types
 * are an error in C++).  The symbols and the calling convention are the same.
 */
#ifndef DSPLIBS_PORT_DSP_HOST_H
#define DSPLIBS_PORT_DSP_HOST_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

  struct modem;

  /* ---- debug output (modem_debug.c) ---- */
  extern unsigned int dsplibs_debug_level;
  extern int dsplibs_debug_printf(const char* fmt, ...);

  /* ---- sysdep_common.c ---- */
  extern void* sysdep_malloc(unsigned int size);
  extern void sysdep_free(void* mem);
  extern void* sysdep_memset(void* dst, int c, size_t len);
  extern void* sysdep_memcpy(void* dst, const void* src, size_t len);
  extern size_t sysdep_strlen(const char* s);
  extern char* sysdep_strcpy(char* d, const char* s);
  extern char* sysdep_strcat(char* d, const char* s);
  extern int sysdep_sprintf(char* buf, const char* fmt, ...);
  /* `ap` is a va_list: __builtin_va_list is what <stdarg.h> typedefs it to (the freestanding rule keeps <stdarg.h> out of this header) */
  extern int sysdep_vsnprintf(char* buf, unsigned int size, const char* fmt, __builtin_va_list ap);

#ifndef DSP_HOST_MODEM_OPAQUE
  /* ---- modem.c, modem_param.c, modem_debug.c ---- */
  extern long modem_get_param(struct modem* m, unsigned param);
  extern long modem_set_param(struct modem* m, unsigned name, int val);
  extern int modem_get_bits(struct modem* m, int nbits, unsigned char* buf, int n);
  extern int modem_put_bits(struct modem* m, int nbits, unsigned char* buf, int n);
  extern int modem_get_sreg(struct modem* m, unsigned int num);
  extern int modem_send_to_tty(struct modem* m, const char* buf, int n);
  extern int modem_recv_from_tty(struct modem* m, char* buf, int n);
  extern int modem_debug_log_data(struct modem* m, unsigned id, const void* data, int count);
#else
/* DSP_HOST_MODEM_OPAQUE (src/vpcm/dp_vpcm.cpp): the same host functions with the modem as an opaque `void *`, and the registration pair with the
   * pump id as `int` and the operations record as `void *` (the host's own types are in core/host_interface.h, C only).  Same symbols, same ABI. */
extern long modem_get_param(void* m, unsigned name);
extern long modem_set_param(void* m, unsigned name, int val);
extern int modem_get_bits(void* m, int nbits, uint8_t* buf, int n);
extern int modem_put_bits(void* m, int nbits, uint8_t* buf, int n);
extern int modem_debug_log_data(void* m, unsigned id, const void* data, int cnt);
extern int modem_dp_register(int id, void* op);
extern void modem_dp_deregister(int id, void* op);
#endif

#ifdef __cplusplus
}
#endif

#endif /* DSPLIBS_PORT_DSP_HOST_H */
