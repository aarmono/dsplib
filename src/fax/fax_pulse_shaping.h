/**
 * @file fax_pulse_shaping.h
 * @brief Typed pulse-shaping interpolator (polyphase symbol-to-sample filter) of the fax transmitters.
 *
 * FAXPPS_init() / FAXPPS_filter() / FAXPPS_free() turn a stream of symbols, held in the circular output descriptor of a symbol-mapping encoder (FaxCirc,
 * fax_common.h), into transmit samples: each output sample is a polyphase FIR over the last `taps` symbols, one I and one Q branch, the difference of the
 * two branches scaled by the output gain.  The V.27ter, V.29 and V.17 transmitters embed a FaxPps in their state bundle and call it once per block.
 *
 * The V.32 / V.22 modems have their own FPM_PPS_* with the same arithmetic (src/v32/v32_pulse_shaping.c); those work on a raw block whose pointer slots
 * live outside the object on 64-bit hosts, so they cannot serve these objects.  Here the history pointers are ordinary members.
 *
 * Objects:
 * - FaxPpsCfg: the configuration, copied into the object by FAXPPS_init().
 * - FaxPps: the configuration plus the run-time state (phase accumulator, history write index, the two history buffers).
 */
#ifndef DSPLIBS_PORT_FAX_PPS_H
#define DSPLIBS_PORT_FAX_PPS_H
#include <stddef.h>
#include <stdint.h>
#include "fax_common.h"

/** Pulse-shaper configuration (0x28 bytes on 32-bit hosts; a stack block in the transmitter creators). */
typedef struct FaxPpsCfg
{
  short up, down;             ///< interpolation factor (phases per symbol); initial phase accumulator and base phase advance
  int mode;                   ///< 0: take I / Q from circ->i / circ->q; otherwise map the low byte of circ->buf[rd] through imap / qmap
  int scale;                  ///< output gain, Q15
  short step, s0e;            ///< extra phase advance per output sample (added to `down`); unused
  const short *imap, *qmap;   ///< I / Q value of each symbol code (mode != 0)
  const short *ifilt, *qfilt; ///< I / Q branch polyphase coefficients, filtlen entries each
  short filtlen, s22;         ///< total number of coefficients per branch; unused
  int w24;                    ///< carried, never read
} FaxPpsCfg;

/** Pulse-shaper state (0x38 bytes on 32-bit hosts). */
typedef struct FaxPps
{
  FaxPpsCfg cfg;        ///< configuration copy
  short need;           ///< 1: the next output sample needs a new symbol from the source
  short acc;            ///< phase accumulator (coefficient phase offset, 0 .. up - 1); starts at cfg.down
  short pos;            ///< write index of the histories
  short taps;           ///< history length: cfg.filtlen / cfg.up
  short *histI, *histQ; ///< I / Q symbol histories, `taps` shorts each (sysdep_malloc)
} FaxPps;

#if UINTPTR_MAX == 0xffffffffu
_Static_assert(sizeof(FaxPpsCfg) == 0x28 && offsetof(FaxPpsCfg, step) == 0xc && offsetof(FaxPpsCfg, imap) == 0x10 && offsetof(FaxPpsCfg, qfilt) == 0x1c &&
                   offsetof(FaxPpsCfg, filtlen) == 0x20 && offsetof(FaxPpsCfg, w24) == 0x24,
               "FaxPpsCfg 32-bit layout");
_Static_assert(sizeof(FaxPps) == 0x38 && offsetof(FaxPps, need) == 0x28 && offsetof(FaxPps, taps) == 0x2e && offsetof(FaxPps, histI) == 0x30 &&
                   offsetof(FaxPps, histQ) == 0x34,
               "FaxPps 32-bit layout");
#endif

/**
 * @brief Initialise (or re-initialise) a pulse shaper.
 * @param p      state to set up
 * @param cfg    configuration, copied
 * @param fresh  non-zero: `p` is new, allocate both histories; 0: `p` was initialised before and its buffers are kept unless `taps` grows
 * @return 2 * taps (the byte size of one history) when the buffers were allocated, else 0
 * @warning The allocation results are not checked, `cfg->up` must not be 0 and `cfg->filtlen / cfg->up` must be positive.
 */
int FAXPPS_init(FaxPps* p, const FaxPpsCfg* cfg, int fresh);

/**
 * @brief Produce transmit samples from the symbols in a circular descriptor.
 * @param p      pulse shaper
 * @param src    symbol source; its read index `rd` is advanced (wrapping at `lim`)
 * @param out    output samples
 * @param count  number of SOURCE symbols to consume (not output samples)
 * @return number of output samples written (16 bit)
 * @warning `out` must hold the produced samples; their number depends on the interpolation ratio and is not bounded by the parameters.
 */
short FAXPPS_filter(FaxPps* p, FaxCirc* src, short* out, unsigned short count);

/** @brief Release both history buffers (not the object). */
void FAXPPS_free(FaxPps* p);
#endif
