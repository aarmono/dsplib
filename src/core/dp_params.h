/**
 * @file dp_params.h
 * @brief The data-pump runtime's modem parameter block (modem_get_param(m, 0xa) / dp_param_get()): ONE flat struct, one member per word.
 *
 * The block is 0x88 bytes, created by dp_runtime_create() (core/dp_runtime.c) and read / written by:
 * - the V.8 glue and dp_runtime_create() itself (flags0 .. flags3, qcIndex, codecType ...),
 * - the V.90 / V.92 parameter classes (vpcm/pcm_parameters.h: the rates, power reduction, connection type, probing flags, and the parameter file path),
 * - the V.34 chain and the vpcm glue (v34/, vpcm/: the rate limits, delays, minLevel, entranceFilter ...; the plain-C tiers name the type V34ModParObj).
 *
 * Every word has exactly one member at its fixed offset.  Words that only dp_runtime_create stores (constants) and nobody reads keep offset names (w4, w8,
 * w58); never-touched runs are unkNN arrays.  DpRuntime, V01_ModemParams, V34ModParObj and P07_ModemParams are typedefs of the one struct (the names the
 * users spell).
 *
 * Layout.  32-bit host: sizeof 0x88, every offset below asserted.  x86-64: `path` is a real 8-byte pointer at +0x78 and the unused tail after it is
 * 8 instead of 12 bytes, so sizeof stays 0x88 on both hosts (the allocation size of dp_runtime_create() does not change).  The harness mirrors
 * (layouts/vpcm/pcm_parameters.layout, layouts/v34/v01.layout) describe a PREFIX of this block (0x7c bytes in the 32-bit layout) under their own member names; the layout
 * descriptions map each mirror word to the member here (`mirror=member`, tools/layout_gen.py), and class lines with `prefix` do not assert the
 * class's sizeof against it.
 */
#ifndef DP_PARAMS_H
#define DP_PARAMS_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
#define DPP_ASSERT(cond, msg) static_assert(cond, msg)
#else
#define DPP_ASSERT(cond, msg) _Static_assert(cond, msg)
#endif

typedef struct _tagModemParameters
{
  /* 0x00 flag bits.  Bit 0 selects the default of V90Parameters::anspcmDemodulationLength (0x960 / 0x1e3c); bit 0x10: K56Flex allowed, bit 0x08: V.90 allowed
   * (VPcmV34InitiateRetrain's mode validation); the V.8 glue sets bits 3, 5 and 7 (dp_v8.c).  The V.8 engine reads flags0 .. flags3 as one 32-bit
   * capability word (V8UpdateModemParameters). */
  uint8_t flags0;
  /* 0x01 flag bits; bit 6 is set by the V.8 glue */
  uint8_t flags1;
  /* 0x02 flag bits.  Bit 4 = quick-connect indication (set by the V.8 glue for a V.92 caller; dp_vpcm.cpp may change it), bit 5, bit 6 = qc_lapm,
   * bit 7 (sign bit) = V.92Lite: no PCM upstream, V34GiveINFO1dBits does not retrain to V.34 upstream. */
  uint8_t flags2;
  /* 0x03 flag bits.  Bit 2 = restore the SAS detector (cleared by v90RateRenegSilence and the VPcmFloModem constructor, tested by VPcmFloModem::run) */
  uint8_t flags3;
  /* 0x04 dp_runtime_create stores 0x3c; nobody reads it */
  int w4;
  /* 0x08 dp_runtime_create stores 0x28; nobody reads it */
  int w8;
  /* 0x0c the ANSpcm level index (VPcmV34Create prints it); dp_runtime_create clears it */
  int ansLevel;
  /* 0x10 di->qc_index (0 -> 9; dp_v8 hands it back).  VPcmV34Create reads the same word as the "Uqts index" (prints it, hands it to enterChannelVerification) */
  int qcIndex;
  /* 0x14 the desired TX MD time: VPcmV34Create stores 24 * it / 10 in wabfc_; dp_runtime_create stores 0x2bc */
  uint32_t txMdTime;
  /* 0x18 never touched */
  uint8_t unk18[24];
  /* 0x30 bit/s: the rate limits VPcmV34InitiateRetrain hands to the (K56Flex / V.90) constellation designer and divides by 2400 */
  uint32_t minRate;
  /* 0x34 bit/s */
  uint32_t maxRate;
  /* 0x38 bit/s ("upStream min rate"); vpcm_create stores 4800; the default rate mask uses upstreamMinRate / 2400 */
  uint32_t upstreamMinRate;
  /* 0x3c bit/s ("upStream max rate"); vpcm_create stores 0x8340 (33600) */
  uint32_t upstreamMaxRate;
  /* 0x40 ("tempPR": the digital power reduction is powerReduction / 5 * 0.5); dp_runtime_create clears it */
  uint32_t powerReduction;
  /* 0x44 GetVPcmMinimalTxPowerReduction reads its low 16 bits (signed, clamped to -10 .. 7); dp_runtime_create stores 6 */
  int minTxPowerReduction;
  /* 0x48 di->connection_type ("tempConnectionType": the default of V90Parameters::lineConnectionType); vpcm_delete copies it back */
  uint32_t connectionType;
  /* 0x4c di->clock_deviation (narrowed to 32 bits); the V.90 timing statistics store 1000 * mean here, the V.92 pre-filter set-up prints it * 0.001 */
  int clockDeviation;
  /* 0x50 flag bits.  Bit 0 = trn2d_mean_error_std_evaluation_enable, bit 1 = "tempProbe" (probingMode), bit 2 = tx buffer backward clear (v90Phase34),
   * bit 3 set when the sensitive-ISP flag is seen, bits 4 .. 7 read by the V.34 probe selection; dp_runtime_create stores 1 */
  uint8_t flags50;
  /* 0x51 flag bits; bit 0 is set by VPcmV34Progress with the requestOutputSampleClear */
  uint8_t flags51;
  /* 0x52 never touched */
  uint8_t unk52[2];
  /* 0x54 modem_get_param(m, MDMPRM_CODECTYPE), the hardware codec type: == 0xe selects the entrance filter, == 4 changes echoBeta_ */
  int codecType;
  /* 0x58 dp_runtime_create clears it; nobody reads it */
  int w58;
  /* 0x5c read by the V.90 equalizer set-up (== 1 / == 2 decide the 16-bit mode); dp_runtime_create clears it */
  int w5c;
  /* 0x60 minimum signal level request (VPcmV34Create: index into the minSigLevel table) */
  int minLevel;
  /* 0x64 initial delay (V34 filtdelay) */
  int delay;
  /* 0x68 external delay (V34dmadelay / echo delay) */
  int extDelay;
  /* 0x6c the silence added on PHASE2_COMPLETE (printed) */
  int p2Silence;
  /* 0x70 entrance filter mode of VPcmV34Create: -1 = according to the hardware (codecType == 0xe), 1 = forced enabled, else forced disabled */
  int entranceFilter;
  /* 0x74 never touched */
  uint8_t unk74[4];
  /* 0x78 the parameter file (NULL: no file); the first argument of every Vparser_read_* call (the V01 harness mirror names its low 32 bits `w78_`;
   * vpcm_create clears it) */
  char* path;
  unsigned char tail[0x88 - 0x78 - sizeof(char*)]; /* 0x7c (32-bit host) / 0x80 (x86-64) .. 0x88: never touched */
} _tagModemParameters;

typedef struct _tagModemParameters DpRuntime;
typedef struct _tagModemParameters V01_ModemParams;
typedef struct _tagModemParameters V34ModParObj;
typedef struct _tagModemParameters P07_ModemParams;

/* the same on every host: the size (dp_runtime_create allocates sizeof) and the pointer at +0x78 */
DPP_ASSERT(sizeof(_tagModemParameters) == 0x88 && offsetof(_tagModemParameters, path) == 0x78 && offsetof(_tagModemParameters, tail) == 0x78 + sizeof(char*),
           "_tagModemParameters size / path");
/* every word before +0x78 is a 32-bit int or a byte (array): the same offsets on every host */
DPP_ASSERT(offsetof(_tagModemParameters, flags0) == 0x00 && offsetof(_tagModemParameters, flags1) == 0x01 && offsetof(_tagModemParameters, flags2) == 0x02 &&
               offsetof(_tagModemParameters, flags3) == 0x03 && offsetof(_tagModemParameters, w4) == 0x04 && offsetof(_tagModemParameters, w8) == 0x08 &&
               offsetof(_tagModemParameters, ansLevel) == 0x0c && offsetof(_tagModemParameters, qcIndex) == 0x10 &&
               offsetof(_tagModemParameters, txMdTime) == 0x14 && offsetof(_tagModemParameters, unk18) == 0x18,
           "_tagModemParameters 0x00 .. 0x2f");
DPP_ASSERT(offsetof(_tagModemParameters, minRate) == 0x30 && offsetof(_tagModemParameters, maxRate) == 0x34 &&
               offsetof(_tagModemParameters, upstreamMinRate) == 0x38 && offsetof(_tagModemParameters, upstreamMaxRate) == 0x3c &&
               offsetof(_tagModemParameters, powerReduction) == 0x40 && offsetof(_tagModemParameters, minTxPowerReduction) == 0x44 &&
               offsetof(_tagModemParameters, connectionType) == 0x48 && offsetof(_tagModemParameters, clockDeviation) == 0x4c,
           "_tagModemParameters 0x30 .. 0x4f");
DPP_ASSERT(offsetof(_tagModemParameters, flags50) == 0x50 && offsetof(_tagModemParameters, flags51) == 0x51 && offsetof(_tagModemParameters, unk52) == 0x52 &&
               offsetof(_tagModemParameters, codecType) == 0x54 && offsetof(_tagModemParameters, w58) == 0x58 && offsetof(_tagModemParameters, w5c) == 0x5c &&
               offsetof(_tagModemParameters, minLevel) == 0x60 && offsetof(_tagModemParameters, delay) == 0x64 &&
               offsetof(_tagModemParameters, extDelay) == 0x68 && offsetof(_tagModemParameters, p2Silence) == 0x6c &&
               offsetof(_tagModemParameters, entranceFilter) == 0x70 && offsetof(_tagModemParameters, unk74) == 0x74,
           "_tagModemParameters 0x50 .. 0x77");

#endif /* DP_PARAMS_H */
