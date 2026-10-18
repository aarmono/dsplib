/**
 * @file v34_modulator_tables.h
 * @brief Declarations of the transmit-modulator tables smIndex and quarter (defined in v34_modulator_tables.c).
 *
 * gInvertPat, the third table modulatevector() reads, is declared in v34_init_tables.h.
 */
#ifndef DSPLIBS_PORT_V34_TIER6_TABLES_H
#define DSPLIBS_PORT_V34_TIER6_TABLES_H

/** Shell-ring index permutations (two permutations of 0..7). */
extern const short smIndex[16];
/** Packed constellation points (x << 8) | (y & 0xff) of one quarter of the plane. */
extern const short quarter[416];

#endif
