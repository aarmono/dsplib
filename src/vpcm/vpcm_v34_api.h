/**
 * @file vpcm_v34_api.h
 * @brief Declarations of the V.34 session functions that src/vpcm defines and that other files call.
 *
 * The definitions are in v34_api.c, v34_handshake_init.c, v34_modem_init.c, vpcm_create.cpp, vpcm_progress.cpp and vpcm_datapump.cpp; the callers are the
 * V.90 / V.92 pieces of src/vpcm itself (dp_vpcm.cpp, vpcm_flo_modem*.cpp, vpcm_progress.cpp ...) and the V.34 handshake pieces of src/v34/handshake.  This
 * header is the mirror of v34/v34_api.h, which declares what src/v34 defines for vpcm.
 *
 * The prototypes follow the DEFINITIONS: the V.34 object is `struct tagV34Object *` (`V34Obj` in the C files, `tagV34Object` in the C++ ones, the same type)
 * wherever the definition takes it typed; the entry points VPcmFloModem calls with its `void *` ctx_ still take `void *`.  FREESTANDING (<stdint.h> only), C and C++ (extern "C" guards), include-guarded.  Declared elsewhere because they take C++-only types:
 * V34GiveINFO1dBits and indicateJaTransmission (vpcm_session_wrappers.h), V34SetINFO1aBits / VPcmV34InitiateRetrain / v90Phase34 / k56FlexPhase34 /
 * v90RateReneg (v34/v34_object.h).
 */
#ifndef VPCM_V34_API_H
#define VPCM_V34_API_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

  struct tagV34Object;
  struct _tagModemParameters; /* V01_ModemParams (core/dp_params.h) */

  /* ---- v34_modem_init.c, v34_handshake_init.c: the set-up the handshake and the session driver call ---- */
  extern int v34handshakinit(struct tagV34Object* obj, unsigned int mode);
  extern void v34modeminit(struct tagV34Object* obj);
  extern void VPcmV34SetMohMessageBits(struct tagV34Object* obj, void* out);

  /* ---- v34_api.c: the VPcmV34* / V34XF_* API (indications, queries) ---- */
  extern void VPcmV34IndicateLocalRRN(void* obj);
  extern void VPcmV34IndicateRemoteRRN(void* obj);
  extern void VPcmV34SetIndicationOfRemoteRetrain(void* obj);
  extern void VPcmV34SetV90RateReneg(void* obj, short rrn, unsigned char sz);
  extern void VPcmV34InterpretMohMessageBits(struct tagV34Object* obj, const uint16_t* msg);
  extern int VPcmV34GetMaxUpstreamRateIndex(void* obj);
  extern int VPcmV34GetCurrentSessionDP(struct tagV34Object* obj);
  extern int VPcmV34GetCurrentTxBitRate(struct tagV34Object* obj);
  extern void* VPcmV34GetCleanedSamples(struct tagV34Object* obj, int* out);
  extern void V34XF_IndicateJdReceived(void* obj, unsigned char a, unsigned char b);
  extern void V34XF_IndicateDilReceived(void* obj, unsigned char a);
  extern void V34XF_IndicateTrn2dReceived(void* obj);
  extern void* V34XF_GetInfo0BitsPtr(void* obj);
  extern void* V34XF_GetProbeResultsPtr(void* obj);
  extern int V34XF_GetRTD(void* obj);

  /* ---- the C++ session driver (vpcm_create.cpp, vpcm_progress.cpp, vpcm_datapump.cpp) ---- */
  extern int VPcmV34Create(struct tagV34Object* obj, int answerer, int frag, struct _tagModemParameters* pm, int mode);
  extern int VPcmV34Progress(struct tagV34Object* obj, float* inp, float* outp, unsigned nIn, int* bitsOut, int* countOut, int* bitsIn, int* countIn);
  extern void datapumpv34(struct tagV34Object* obj);

#ifdef __cplusplus
}
#endif

#endif /* VPCM_V34_API_H */
