/**
 * @file v34_symbol_tables.h
 * @brief Declarations of the symbol-path tables (defined in v34_symbol_tables.c) and the `costbl` alias of the V.8 cosine table.
 *
 * c1200_, c2400_, bpv22high and bpv22low (also used by the symbol-path functions) are declared in v34_frame_tables.h; rxvect4 and gInvertPat in
 * v34_init_tables.h.
 */
#ifndef DSPLIBS_PORT_V34_TIER5_TABLES_H
#define DSPLIBS_PORT_V34_TIER5_TABLES_H

#include "v8/v8_tables.h" /* v8_costbl */

#ifdef __cplusplus
extern "C"
{
#endif

extern const unsigned short lsbMask[17];
extern const int vect4[4];
extern const int vect16[16]; /* txmitquadbit index is always 0..15 (the feared 16..19 over-read is unreachable) */
extern const short intcoef1[12];
extern const short intcoef2[12];
extern const short intcoef3[12];
extern const short fsklpfcoeff600[80];
extern const short scale2400[28];
extern const short scale2800[28];
extern const short scale3000[28];
extern const short scale3200[28];
extern const short scale3429[28];
extern const short c1600[8];
extern const short c1680[8];
extern const short c1800_[8];
extern const short c1829[8];
extern const short c1867[8];
extern const short c1920[8];
extern const short c1959[8];
extern const short c2000[8];

/** dftupdate()'s cosine table `costbl` (256 entries, Q14) is byte-identical to the V.8 table v8_costbl: it is that table, not a second copy. */
#define costbl v8_costbl

#ifdef __cplusplus
}
#endif

#endif
