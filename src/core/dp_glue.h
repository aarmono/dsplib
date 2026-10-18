/**
 * @file dp_glue.h
 * @brief The slmodemd-side "dp glue" of the library: the objects the data-pump glue allocates, typed for 32- and 64-bit hosts.
 *
 * What the glue is.  slmodemd drives every data pump through a `struct dp` and its `struct dp_operations` (create / delete / process ...).  The glue
 * of each pump (src/fsk/dp_v21_b103.c, dp_v23.c, src/v32/dp_v32.c, src/v8/dp_v8.c, src/v22/dp_v22.c, src/callprog/dp_call.c, src/vpcm/dp_vpcm.cpp)
 * allocates a block that starts with a `struct dp` followed by a private tail (slmodemd frees it through op->delete and treats it as a `struct dp *`).
 * This header declares the blocks of the Bell 103 / V.21, V.23, V.32 and V.8 pumps and the services the glue shares: the DpWrapper rate / block adapter
 * (core/dp_wrapper.c; also used by the V.22 glue), the runtime parameter block,
 * the DC remover and the ring detector wrapper (core/dp_runtime.c), and the entry points of the engines the glue drives.
 *
 * Which structures are the host's own.  `struct dp`, `struct dp_operations`, `struct dsp_info` and the enum DP_ID are the host's definitions
 * (declared in core/host_interface.h, with the host's layout), so on x86-64 the
 * glue reads and writes slmodemd's structures in their native layout (64-bit pointers, `long clock_deviation` 8 bytes).  `struct modem` stays opaque:
 * the glue only passes it to modem_get_param / modem_set_param / modem_get_bits / modem_put_bits.
 *
 * Layouts.  On a 32-bit host the real `struct dp` is 0x14 bytes (id, modem, status, op, dp_data) and the `_Static_assert`s at the end pin every glue
 * object to its 32-bit image layout.  The pointers the glue keeps (the engine instance, the wrapper, the RcFixed handles, the dsp_info / runtime
 * block) are real pointers in the native layout.
 *
 * The glue blocks:
 * - B103Glue / V23Glue (0x348 bytes in the 32-bit layout): struct dp, caller, state ("want"), last status / connected flag, engine, DpWrapper, an int
 *   array of transmit bits and one of receive bits;
 * - V32Glue: the same shape for V.32 / V.32bis;
 * - V8Glue (0x34 bytes): struct dp (dp_data = the block itself), originate flag, DP id, idle timer, dsp_info, DpRuntime, status, V.8 engine;
 * - DpWrapper (0x938 bytes): the generic sample-rate / block-size adapter dp_wrapper_create() builds for the b103, v23, v22 and v32 glue;
 * - DpRuntime (0x88 bytes), DcrObj (0x20 bytes), RdObj (8 / 16 bytes).
 *
 * DpRuntime is the "modem parameter block" (modem_get_param(m, 0xa), dp_param_get()): one definition shared with the V.34 and V.90 code (core/dp_params.h,
 * struct _tagModemParameters; DpRuntime is a typedef of it).  Its 32-bit layout is fixed; on x86-64 the parameter file path at +0x78 is a real pointer
 * and the size stays 0x88.
 *
 * Using DpWrapper from a glue block: the block starts with `struct dp`, `dp_wrapper_create(owner, cb, frag, rate, engineRate)` builds the wrapper, the
 * glue stores itself in `wrapper->priv`, and the process callback finds its glue block as `((DpWrapper *)dp->dp_data)->priv`.  The `create` operation
 * has the signature create(m, id, caller, srate, max_frag, op).
 */
#ifndef DSPLIBS_PORT_DP_GLUE_H
#define DSPLIBS_PORT_DP_GLUE_H
#include <stddef.h>
#include <stdint.h>

#include "core/host_interface.h" /* host interface: DP_ID, DPSTAT_*, struct dp / dp_operations, MDMPRM_* / Get* ids */
#include "rcfixed_resampler.h"
#include "fsk/v21_b103.h"
#include "fsk/v23.h"
#include "v8/v8.h"              /* V8Create / V8Delete / V8Process / V8UpdateModemParameters (and v8_obj.h) */
#include "ring/ring_detector.h" /* RingDetector_* */
#include "v32/v32.h"
#include "core/dp_params.h" /* DpRuntime */
#include "core/dsp_host.h"
#include "core/dp_registry.h"

/* ---- the engine entry points the glue calls ---- */

extern B103Obj* B103FP_create(B103Obj* self, int* cfg);
extern void B103FP_delete(B103Obj* self);
extern unsigned int B103FP_modem(B103Obj* self, int* txBits, short* txOut, short* rxIn, int* rxBits, unsigned short* txCount, unsigned short* rxCount);

extern V23Modem* CreateV23Modem(V23Modem* self, short rxBackend, const V23Cfg* cfg);
extern void DeleteV23Modem(V23Modem* self);
extern int V23ModemMain(V23Modem* self, int* txBitSrc, int* txBitCount, short* txAudioOut, int txSampleCount, short* rxAudioIn, int rxSampleCount,
                        unsigned int* rxBitsOut, int* nRxBitsOut);

/* the V.32 engine entry points (src/v32/v32_engine.c): the engine is the connection wrapper (V32Wrap, v32/v32.h); V32FP_create returns it */
extern struct V32Wrap* V32FP_create(int* cfg6, int param2);
extern void V32FP_delete(struct V32Wrap* obj);
extern unsigned char V32FP_modem(struct V32Wrap* obj, void* txBits, void* txOut, void* rxIn, void* rxBits, unsigned int* txCount, unsigned int* rxCount);
extern int V32FP_status(struct V32Wrap* obj, short* statusOut);
extern char* V32FP_GetCleanedSamples(struct V32Wrap* obj, int* outCount);

/* ---- dp_wrapper ---- */

/** Engine block processor called by dp_wrapper_run(): process `n` samples from `in` into `out`; a non-zero result is reported to the host. */
typedef int (*DpProcessFn)(struct dp* dp, void* in, void* out, int n);

#define DPW_RING 384 /* samples of one ring: two halves of at most 192 */

/** One ring of a DpWrapper: two halves of `blk` samples; the half being processed / filled alternates between 0 and blk. */
typedef struct DpRing
{
  unsigned int count; ///< samples held (the output ring starts with one block of silence)
  unsigned int wr;    ///< write position (input ring only)
  unsigned int rd;    ///< read position (output ring only)
  unsigned int half;  ///< offset of the half being processed / filled: 0 or blk
  short data[DPW_RING];
} DpRing;

/** The rate / block adapter built by dp_wrapper_create() (see core/dp_wrapper.c). */
typedef struct DpWrapper
{
  struct dp* owner;  ///< first dp_wrapper_create() argument; cleared by dp_wrapper_delete()
  DpProcessFn cb;    ///< the engine block processor
  void* priv;        ///< free slot: the b103 / v23 / v32 glue store their own block here
  RcFixed* rcIn;     ///< modem rate -> engine rate (NULL when the rates are equal)
  RcFixed* rcOut;    ///< engine rate -> modem rate
  short inbuf[192];  ///< resampled input block
  short outbuf[192]; ///< the engine's output block before rcOut
  unsigned int blk;  ///< samples per block at the MODEM rate
  DpRing out;        ///< output ring
  DpRing in;         ///< input ring
} DpWrapper;

extern DpWrapper* dp_wrapper_create(struct dp* owner, DpProcessFn cb, int frag, int rate, int engineRate);
extern void dp_wrapper_delete(DpWrapper* w);
extern int dp_wrapper_run(struct dp* dp, void* in, void* out, int cnt);

/* ---- the entry points slmodemd links (prop_dp_init / prop_dp_exit call the dp_*_init / dp_*_exit; modem.c the runtime / dcr / RD functions) ---- */
extern const char* const v8StatusName[19];
extern void* dp_runtime_create(struct modem* m);
extern void dp_runtime_delete(void* runtime);
extern void* dcr_create(void);
extern void dcr_delete(void* dcr);
extern void dcr_reset(void* dcr);
extern int dcr_process(void* dcr, void* buf, int len);
extern void* RD_create(struct modem* m, unsigned rate);
extern void RD_delete(void* obj);
extern int RD_process(void* obj, void* in, int count);
extern void RD_ring_details(void* obj, long* freq, long* duration);

/* ---- the b103 / v23 / v32 glue blocks ---- */

/** Glue block of the Bell 103 / V.21 data pump (src/fsk/dp_v21_b103.c). */
typedef struct B103Glue
{
  struct dp dp;
  int caller;         ///< role passed to create
  int state;          ///< transmit bits wanted by the engine (0 = none, 6 = a block)
  int last;           ///< last engine status (initialised to 1)
  B103Obj* eng;       ///< the Bell 103 / V.21 engine
  DpWrapper* wrapper; ///< rate / block adapter
  int txbits[100];    ///< transmit bits fetched from the host, one per int
  int rxbits[100];    ///< received bits from the engine, one per int
} B103Glue;

/** Glue block of the V.23 data pump (src/fsk/dp_v23.c). */
typedef struct V23Glue
{
  struct dp dp;
  int caller;         ///< role passed to create
  int want;           ///< transmit bits V23ModemMain asked for
  int connected;      ///< link-connected flag
  V23Modem* eng;      ///< the V.23 engine
  DpWrapper* wrapper; ///< rate / block adapter
  int txbits[100];    ///< transmit bits fetched from the host, one per int
  int rxbits[100];    ///< received bits from the engine, one per int
} V23Glue;

/**
 * Glue block of the V.32 / V.32bis data pump (src/v32/dp_v32.c): the same shape as B103Glue.  `bits` is the number of bits per symbol of the connected
 * rate (initially 6), `rate` the negotiated line rate (initially 0x3840 = 14400), `want` the transmit bits requested per block (0 until the first
 * CONNECT, then 12).
 */
typedef struct V32Glue
{
  struct dp dp;
  int bits;
  int rate;
  int want;
  struct V32Wrap* eng; ///< the V.32 engine (V32FP_create())
  DpWrapper* wrapper;  ///< rate / block adapter
  int txbits[100];     ///< transmit bits fetched from the host, one per int
  int rxbits[100];     ///< received bits from the engine, one per int
} V32Glue;

/** Glue block of the V.8 data pump (src/v8/dp_v8.c). */
typedef struct V8Glue
{
  struct dp dp;        ///< dp.dp_data points at the block itself
  int originate;       ///< 1 when create was called with caller == 0
  int id;              ///< the requested DP id
  int w1c;             ///< cleared at create, not read by the glue
  int idle;            ///< idle timer in samples (0 = none, -1 = expired)
  struct dsp_info* di; ///< modem_get_param(m, MDMPRM_DSPINFO)
  DpRuntime* rt;       ///< dp_param_get(m)
  int status;          ///< last V8Process status
  void* eng;           ///< V.8 engine
} V8Glue;

/* ---- runtime, dcr, rd ---- */
/* DpRuntime: core/dp_params.h (the one definition of the parameter block) */

/**
 * State of a DcrObj.  DCR_WAIT counts n1 samples; DCR_ESTIMATE averages n2 samples (when it ends "DCR: initial DC Evaluation done, DC level %d, %sabled" is
 * printed); then DCR_TRACK (flags bit 2 set, "enabled": the running estimate is re-averaged over n3-sample windows) or DCR_HOLD (flags bit 2 clear: the
 * estimate is kept).
 */
enum DcrState
{
  DCR_WAIT = 0,
  DCR_ESTIMATE = 1,
  DCR_TRACK = 2,
  DCR_HOLD = 3
};

/** The DC remover (core/dp_runtime.c).  Holds no pointer: the same layout on every host. */
typedef struct DcrObj
{
  unsigned char flags; ///< bit 0 enabled (otherwise dcr_process() does nothing), bit 1 subtract the DC from the samples, bit 2 track after the first estimate
  unsigned char pad1[3];
  int state;      ///< DcrState
  short dc;       ///< current DC estimate
  short thr;      ///< 3000 by default: |dc| at or above this is reported by dcr_process()
  int sum;        ///< sum of the samples of the running window
  int count;      ///< samples of the running window
  int n1, n2, n3; ///< window lengths: wait 5760, first estimate 9600, tracking window 19200 samples
} DcrObj;

/** The ring detector wrapper (core/dp_runtime.c). */
typedef struct RdObj
{
  struct modem* modem;
  RingDetector* rd;
} RdObj;

#if UINTPTR_MAX == 0xffffffffu
_Static_assert(sizeof(struct dp) == 0x14 && offsetof(struct dp, id) == 0 && offsetof(struct dp, modem) == 4 && offsetof(struct dp, status) == 8 &&
                   offsetof(struct dp, op) == 0xc && offsetof(struct dp, dp_data) == 0x10,
               "struct dp 32-bit layout");
_Static_assert(sizeof(struct dp_operations) == 0x18 && offsetof(struct dp_operations, create) == 8 && offsetof(struct dp_operations, process) == 0x10,
               "dp_operations");
_Static_assert(sizeof(struct dsp_info) == 0x10 && offsetof(struct dsp_info, clock_deviation) == 4 && offsetof(struct dsp_info, qc_lapm) == 8 &&
                   offsetof(struct dsp_info, qc_index) == 0xc,
               "dsp_info 32-bit layout");
_Static_assert(sizeof(DpWrapper) == 0x938 && offsetof(DpWrapper, priv) == 8 && offsetof(DpWrapper, rcIn) == 0xc && offsetof(DpWrapper, rcOut) == 0x10 &&
                   offsetof(DpWrapper, inbuf) == 0x14 && offsetof(DpWrapper, outbuf) == 0x194 && offsetof(DpWrapper, blk) == 0x314 &&
                   offsetof(DpWrapper, out) == 0x318 && offsetof(DpWrapper, in) == 0x628,
               "DpWrapper 32-bit layout");
_Static_assert(sizeof(B103Glue) == 0x348 && offsetof(B103Glue, caller) == 0x14 && offsetof(B103Glue, state) == 0x18 && offsetof(B103Glue, last) == 0x1c &&
                   offsetof(B103Glue, eng) == 0x20 && offsetof(B103Glue, wrapper) == 0x24 && offsetof(B103Glue, txbits) == 0x28 &&
                   offsetof(B103Glue, rxbits) == 0x1b8,
               "B103Glue 32-bit layout");
_Static_assert(sizeof(V23Glue) == 0x348 && offsetof(V23Glue, eng) == 0x20 && offsetof(V23Glue, wrapper) == 0x24 && offsetof(V23Glue, txbits) == 0x28 &&
                   offsetof(V23Glue, rxbits) == 0x1b8,
               "V23Glue 32-bit layout");
_Static_assert(sizeof(V32Glue) == 0x348 && offsetof(V32Glue, bits) == 0x14 && offsetof(V32Glue, rate) == 0x18 && offsetof(V32Glue, want) == 0x1c &&
                   offsetof(V32Glue, eng) == 0x20 && offsetof(V32Glue, wrapper) == 0x24 && offsetof(V32Glue, txbits) == 0x28 &&
                   offsetof(V32Glue, rxbits) == 0x1b8,
               "V32Glue 32-bit layout");
_Static_assert(sizeof(V8Glue) == 0x34 && offsetof(V8Glue, originate) == 0x14 && offsetof(V8Glue, id) == 0x18 && offsetof(V8Glue, idle) == 0x20 &&
                   offsetof(V8Glue, di) == 0x24 && offsetof(V8Glue, rt) == 0x28 && offsetof(V8Glue, status) == 0x2c && offsetof(V8Glue, eng) == 0x30,
               "V8Glue 32-bit layout");
_Static_assert(sizeof(RdObj) == 8, "RdObj 32-bit layout");
#endif
/* DpRuntime: sizeof 0x88 and the named offsets on every host, asserted in core/dp_params.h.  DcrObj holds no pointer: the same layout on every host */
_Static_assert(sizeof(DcrObj) == 0x20 && offsetof(DcrObj, dc) == 8 && offsetof(DcrObj, thr) == 0xa && offsetof(DcrObj, sum) == 0xc &&
                   offsetof(DcrObj, count) == 0x10 && offsetof(DcrObj, n3) == 0x1c,
               "DcrObj layout");

#endif
