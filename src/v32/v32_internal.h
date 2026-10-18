/**
 * @file v32_internal.h
 * @brief Prototypes of the functions that the files of src/v32 define for each other.
 *
 * Module-internal: only files of src/v32 include it.  The two functions the fax modems also call, VTB_decoder and FPM_lmsupd, are declared in v32.h;
 * FPM_atan is in fpm/fpm.h; the engine entry points slmodemd's glue calls (V32FP_*) are in core/dp_glue.h.  The prototypes are grouped by the file that
 * defines them; each is documented at its definition.  A `V32Wrap *` parameter is the connection wrapper, a `char *` the address of an engine sub-object (see v32.h).
 */
#ifndef DSPLIBS_PORT_V32_INTERNAL_H
#define DSPLIBS_PORT_V32_INTERNAL_H

#include "v32.h"

/* ---- v32_datapath.c: the wrapper-level data path the engine's handlers call ---- */
extern unsigned short DemodDataV32(V32Wrap* wrapperObj, short* samples, short* outBits, unsigned short count);
extern void DescrambleDataV32(V32Wrap* wrapperObj, unsigned short* data, unsigned short count);
extern unsigned int EpochDetectV32(V32Wrap* wrapperObj);
extern unsigned int GetRateV32(V32Wrap* wrapperObj);
extern unsigned short ModDataV32(V32Wrap* wrapperObj, void* rawBits, short* out, unsigned short count);
extern int RenegotiateDetectV32(V32Wrap* wrapperObj);
extern int RetrainDetectV32(V32Wrap* wrapperObj);
extern unsigned short RxClampV32(V32Wrap* wrapperObj, void* unusedIn, unsigned short* out);
extern void ScrambleDataV32(V32Wrap* wrapperObj, unsigned short* data, unsigned short count);
extern void SeedScramblerV32(V32Wrap* wrapperObj, unsigned int seed);
extern void SetAdaptEcV32(V32Wrap* wrapperObj, unsigned short mode);
extern void SetAdaptEqV32(V32Wrap* wrapperObj, unsigned short mode);
extern void SetECRndTripDelayV32(V32Wrap* wrapperObj, unsigned short roundTrip);
extern void SetRxLoopsV32(V32Wrap* wrapperObj, unsigned short mode);
extern void SetRxModeV32(V32Wrap* wrapperObj, short mode);
extern void SetTxModeV32(V32Wrap* wrapperObj, short mode);
extern void TxClockSyncV32(void);
extern unsigned short TxNoCarrierV32(V32Wrap* wrapperObj, void* unusedIn, short* out, short count);

/* ---- v32_echo_canceller.c ---- */
extern void FPM_ECC_init(V32Ecc* eccObj, const int* cfg, int allocFlag);
extern void FPM_ECC_free(V32Ecc* eccObj);
extern short FPM_ECC_cancel(V32Ecc* eccObj, short* samples, unsigned short count);

/* ---- v32_equalizer.c ---- */
extern void FPM_FSE_init(V32Fse* fse, const int* cfg, int allocFlag);
extern void FPM_FSE_free(V32Fse* fse);
extern short FPM_FSE_receive(V32Fse* fse, short* rxSamples, short* outBits, unsigned short count);
extern unsigned int FSE_decision_AB(V32Fse* fse, short* angleInOut, short* magInOut);
extern unsigned char FSE_decision_CD(V32Fse* fse, short* angleInOut, short* magInOut);
extern unsigned int FSE_decision_trn(V32Fse* fse, short* angleInOut, short* magInOut);
extern unsigned int FSE_decision_4pt(V32Fse* fse, short* angleInOut, short* magInOut);
extern unsigned int FSE_decision_16pt(V32Fse* fse, short* angleInOut, short* magInOut);
extern short FSE_decision_32pt(V32Fse* fse, short* angleInOut, short* magInOut);
extern short FSE_decision_16Tpt(V32Fse* fse, short* angleInOut, short* magInOut);
extern short FSE_decision_64pt(V32Fse* fse, short* angleInOut, short* magInOut);
extern short FSE_decision_128pt(V32Fse* fse, short* angleInOut, short* magInOut);
extern int FSE_getdiag(V32Fse* fse, int mode, int* outPairs, int maxCount);

/* ---- v32_pulse_shaping.c ---- */
extern int FPM_PPS_init(V32Pps* pps, const int* cfg, int allocFlag);
extern void FPM_PPS_free(V32Pps* pps);
extern short FPM_PPS_filter(V32Pps* pps, V32Circ* circ, short* out, unsigned short count);

/* ---- v32_timing.c ---- */
extern void FPM_SRE_init(V32Sre* sre, const int* cfg, int allocFlag);
extern void FPM_SRE_free(V32Sre* sre);
extern unsigned int FPM_SRE_recover(V32Sre* sre, unsigned short* rawIn, char* outSamples, short count);

/* ---- v32_scrambler.c ---- */
extern void SDMv32_scrambler(V32Sdm* sdm, unsigned short* data, short count);
extern void SDMv32_descrambler(V32Sdm* sdm, unsigned short* data, short count);
extern void SMCv32_encoder_abs(V32Smc* smc, V32Circ* outBuf, unsigned short* rawBits, short count);
extern void SMCv32_encoder_dif(V32Smc* smc, V32Circ* outBuf, unsigned short* rawBits, short count);
extern void SMCv32_encoder_tcm(V32Smc* smc, V32Circ* outBuf, unsigned short* rawBits, short count);

/* ---- v32_math.c ---- */
extern int FPM_log10(unsigned short mantissa, short exponent);

#endif /* DSPLIBS_PORT_V32_INTERNAL_H */
