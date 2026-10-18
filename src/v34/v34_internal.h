/**
 * @file v34_internal.h
 * @brief Prototypes of the functions that the files of src/v34 define for each other.
 *
 * The users are the C files v34_*.c and the C++ handshake pieces handshake/v34_hs_*.cpp.  Module-internal: only files of src/v34 include it; what src/vpcm
 * and src/dsp call is in v34_api.h (which this header includes).
 *
 * The prototypes follow the definitions and are grouped by defining file.  The V.34 object is `struct tagV34Object *` (the C view `V34Obj`) and the sub-objects
 * are the typed records of v34_subobjects.h (`V34Rx *`, `V34Mod *`, `V34Tx *`, `V34ToneDet *`) wherever the definition takes them typed; the functions whose callers
 * hand over byte pointers still take `void *` / `unsigned short *`.  The C++ pieces pass their `tagV34Object *` / `tag_bitQ *`.
 * The sub-object state records of v34_dsp_primitives.c are opaque here (their struct definitions stay in that file).
 *
 * Not here, on purpose: getbit and ApplyBulkDelay (the V.34 handshake pieces declare them with a register-argument calling convention at 32 bits and are
 * compiled with -Dgetbit=... -DApplyBulkDelay=..., see dsplib.mk), the v04_* handlers of the handshake (handshake/v34_handshake.h) and the same-file
 * forward declarations of v34_init.c (scrambleGPA / GPC, descrambleGPA / GPC, setInitialPhase, setTimingStateParameters: defined and used only there).
 * Freestanding, C and C++ (extern "C" guards), include-guarded.
 */
#ifndef DSPLIBS_PORT_V34_INTERNAL_H
#define DSPLIBS_PORT_V34_INTERNAL_H

#include <stdint.h>
#include "v34_api.h"
#include "v34_subobjects.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* ---- v34_dsp_primitives.c: echo canceller, equalizer, timing recovery, Hilbert filter ----
 * The callers hand over sub-objects of the V.34 object, so they see the state records opaque (`void *`); the definitions take the typed records
 * EchoAdaptState / PreFilterState / EqualizerState / TimingState, whose structs are private to v34_dsp_primitives.c.  That file defines
 * V34_PRIMITIVES_TYPED before it includes this header and so leaves these prototypes out (a typed and an opaque prototype of one function would conflict). */
#ifndef V34_PRIMITIVES_TYPED
  extern void V34EchoAdapt(void* obj, short err);
  extern void V34EchoCleanUp(void* obj);
  extern int V34EchoEstimateDelayLineEnergy(void* obj);
  extern int V34EchoFilter(void* obj, short val);
  extern void V34EchoPreFilter(short* x, short n, void* obj);
  extern void V34EchoReportCoeff(void* obj);
  extern void V34EchoUpdateDelayLine(void* obj, short sample);
  extern void V34EqualizerAdapt(void* obj, short errReal, short errImag);
  extern void V34EqualizerCenterAdapt(void* obj, short errReal, short errImag);
  extern void V34EqualizerCleanUp(void* obj);
  extern void V34EqualizerClearCenterTaps(void* obj);
  extern void V34EqualizerFilter(void* obj, int* outReal, int* outImag);
  extern void V34EqualizerUpdateDelayLine(void* obj, short i_sample, short q_sample);
  extern void V34HilbertFilter(short* buf, short newSample, int* outReal, int* outImag);
  extern void V34InitHilbertFilter(void* buf);
  extern int V34TimingFilter(void* obj, int packedSample);
  extern void V34TimingFiltersInit(void* obj);
  extern int V34TimingHPFilter(void* obj, short sample);
  extern unsigned int V34TimingPrefilter(void* obj);
#endif
  extern void V34nlencoder(const short in[2], short out[2]);

  /* ---- v34_agc_demod.c: AGC, demodulator, modulator output, INFO bit handling ---- */
  extern void V34agc(V34Rx* obj);
  extern void V34demodulate(V34Rx* rxobj); /* a plain cdecl function */
  extern int V34ModulatorProcess(V34Mod* obj, uint32_t packedIn, short* out);
  extern void V34GiveINFO0dBits(struct tagV34Object* obj, const unsigned short* info);
  extern int V34GiveINFO1aBits(struct tagV34Object* obj, const unsigned short* info);
  extern int V34GiveProbeResults(struct tagV34Object* obj, const DftBin* probe);
  extern void V34SetINFO0aBits(struct tagV34Object* obj, unsigned short* info);

  /* ---- v34_detectors.c ---- */
  extern int tone_detect(V34Rx* b, V34ToneDet* det, const short* p, const short* end);
  extern void dftenergy(DftBin* bins, int nbins, int shift);
  extern void dftupdate(DftBin* bins, int nbins, const short* samples, int nsamples);
  extern void fskdemodulate(struct tagV34Object* obj, const uint16_t* in, void* st);

  /* ---- v34_echo_rx_timing.c ---- */
  extern void rxtiming(struct tagV34Object* obj);
  extern void rxreadqueue(V34Rx* b);
  extern void txwritequeue(V34Tx* q, const short* src);

  /* ---- the remaining tiers ---- */
  extern void TimingV34(struct tagV34Object* obj);                              /* v34_init.c */
  extern void getFrame(struct tagV34Object* obj);                               /* v34_tx_symbol.c */
  extern void decoderv34(struct tagV34Object* obj);                             /* v34_frame_decoder.c */
  extern void probeselect(struct tagV34Object* v34obj);                         /* v34_probe_select.c */
  extern void setfinalrate(struct tagV34Object* obj);                           /* v34_handshake_helpers.c */
  extern void settxlevel(struct tagV34Object* obj, const unsigned short* info); /* v34_tx_level.c */

#ifdef __cplusplus
}
#endif

#endif /* DSPLIBS_PORT_V34_INTERNAL_H */
