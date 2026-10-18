/**
 * @file v34_api.h
 * @brief The functions of src/v34 that other modules call: V.34 transmit / receive primitives, set-up routines and the debug print.
 *
 * The V.90 / V.92 session code in src/vpcm (vpcm/v34_*.c, vpcm_*.cpp) drives the transmit / receive primitives and set-up routines declared here; edprintf is
 * the library's debug print that the C++ files of src/vpcm and src/dsp use.
 *
 * Each prototype is declared once, here, and follows its definition: the V.34 object is `struct tagV34Object *` (`V34Obj` in the C files, `tagV34Object` in
 * the C++ ones, the same type) wherever the definition takes it typed, and `void *` where the definition still does.  v34handshak is defined in the C++
 * handshake (src/v34/handshake/v34_hs_dispatch.cpp) with the typed object; `struct tagV34Object` is declared here so that the header stays usable from C.
 *
 * The prototypes that only src/v34 itself uses are in v34_internal.h (which includes this header).  Functions of src/vpcm that the V.34 code calls are in
 * vpcm/vpcm_v34_api.h.  Freestanding (<stdint.h> only), C and C++ (extern "C" guards), include-guarded.
 */
#ifndef DSPLIBS_PORT_V34_API_H
#define DSPLIBS_PORT_V34_API_H

#include <stdint.h>

#include "v34_subobjects.h" /* V34Mod and the other sub-object structs */

#ifdef __cplusplus
extern "C"
{
#endif

  struct tagV34Object;

  /* ---- debug print (v34_tx_level.c); v34_ed_index returns the address of the encoding index edprintf keeps, which vpcm/v34_modem_init.c shares ---- */
  extern void edprintf(const char* fmt, ...);
  extern int* v34_ed_index(void);

  /* ---- set-up (v34_init.c, v34_setup.c, v34_agc_demod.c) ---- */
  extern void preinitdigital(struct tagV34Object* obj);
  extern int rxinit(struct tagV34Object* obj);
  extern void txinit(struct tagV34Object* obj);
  extern void txrxdmainit(short* out, const void* in);
  extern int* V34SetupModulator(V34Mod* modobj, short baud_s, short carrier_s, short preemp_s, int v90, int fullReset);
  extern void v34setuptxmit(struct tagV34Object* obj);
  extern void V34InitializeImplementationSpecific(struct tagV34Object* obj);
  extern void V34SetINFO0dBits(struct tagV34Object* obj, struct tag_bitQ* info);

  /* ---- per-sample / per-symbol work (v34_tx_symbol.c, v34_modulator.c, v34_receiver.c, v34_echo_rx_timing.c, v34_handshake_helpers.c,
   *      v34_dsp_primitives.c) ---- */
  extern void txmit(struct tagV34Object* obj);
  extern void txmitdibit(struct tagV34Object* obj, int bits);
  extern void txmitquadbit(struct tagV34Object* obj, int bits);
  extern void modulatevector(struct tagV34Object* obj);
  extern void receiver(struct tagV34Object* obj);
  extern void rxtiminginit(struct tagV34Object* obj);
  extern int adaptecho(struct tagV34Object* obj);
  extern int modem_serrint(struct tagV34Object* obj);
  extern void V34EchoHistoryBackwardClean(struct tagV34Object* obj, uint32_t n);
  extern void initdigital(struct tagV34Object* obj);
  extern unsigned short bitreverse(unsigned short x, short nbits);
  extern short V34scrambler(unsigned int* state, short mode, unsigned short inputBits, short numBits);
  extern void detectorinit(V34ToneDet* detobj, const void* p2, short p3, short p4, short p5, short p6,
                           short p7);                     /* v34_init.c; p2 is the coefficient-table address (c1200_ / c2400_) */
  extern void VPcmV34LogTimingOffset(void* obj, short v); /* v34_init.c */

  /* ---- the handshake driver (v34/handshake/v34_hs_dispatch.cpp, C++ with the typed object) ---- */
  extern void v34handshak(struct tagV34Object* obj);

#ifdef __cplusplus
}
#endif

#endif /* DSPLIBS_PORT_V34_API_H */
