/**
 * @file dp_v22.h
 * @brief The V.22 / V.22bis / Bell 212A data-pump glue block: the same shape as the V32Glue / B103Glue of core/dp_glue.h.
 *
 * A real slmodemd `struct dp` followed by the private tail: bits per symbol, transmit bits wanted per block, the engine, the DpWrapper and the two bit
 * arrays (0x344 bytes in the 32-bit layout).  Used by dp_v22.c only.
 */
#ifndef DSPLIBS_PORT_DP_V22_H
#define DSPLIBS_PORT_DP_V22_H
#include "core/dp_glue.h"
#include "v22.h"

/** Glue block of the V.22 data pump. */
typedef struct V22Glue
{
  struct dp dp;
  int bits;           ///< bits per symbol of the connected rate (0 until the CONNECT: 2 at 1200 bit/s, 4 at 2400)
  int want;           ///< transmit bits requested per block (0 until the first CONNECT, then 12)
  V22Engine* eng;     ///< the V.22 engine (V22FP_create)
  DpWrapper* wrapper; ///< rate / block adapter (160-sample engine blocks at 8000 Hz)
  int txbits[100];    ///< transmit bits fetched from the host, one per int
  int rxbits[100];    ///< received bits from the engine, one per int
} V22Glue;

#if UINTPTR_MAX == 0xffffffffu
_Static_assert(sizeof(V22Glue) == 0x344 && offsetof(V22Glue, bits) == 0x14 && offsetof(V22Glue, want) == 0x18 && offsetof(V22Glue, eng) == 0x1c &&
                   offsetof(V22Glue, wrapper) == 0x20 && offsetof(V22Glue, txbits) == 0x24 && offsetof(V22Glue, rxbits) == 0x1b4,
               "V22Glue 32-bit layout");
#endif
#endif
