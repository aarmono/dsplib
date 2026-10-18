/**
 * @file fpm_tables.h
 * @brief Lookup tables and default configuration blocks of the FPM toolkit (defined in fpm_tables.c).
 *
 * The sine / cosine / square-root / reciprocal tables behind FPM_phasor(), FPM_sqrt_dp() and FPM_div() (fpm_math.c, fpm_div32.c), the quadrant sign tables, the
 * filter coefficients and the default configuration blocks of the TONE, MTD, FSM and MRF blocks.
 */
#ifndef DSPLIBS_PORT_FPM_TABLES_H
#define DSPLIBS_PORT_FPM_TABLES_H

extern const short FPM_sin_table[257];
extern const short FPM_cos_table[257];
extern const short FPM_sqrt_table[192];
extern const short FPM_div_table[128];
/* Quadrant sign factors of FPM_phasor() / FPM_phasor_demod(), indexed by (angle >> 13) + 4: entries 4 .. 7 are the factors for quadrants 0 .. 3 (+-16384,
 * Q14 unity); entries 0 .. 3 serve angles with bit 15 set and are not quadrant signs (see FPM_phasor()). */
extern const short FPM_sin_sign_ext[8];
extern const short FPM_cos_sign_ext[8];

extern const short ToneLPF[53];
extern const short COEF_DC[5];
extern const short MTDb103_COEF[10];
extern const short DEF_COEFS[10];

/*
 * Default configuration blocks (the words are documented with the FpmTone / FpmMtd structs in fpm.h and in fpm_tone.c / fpm_mtd.c): FPM_TONE_CFG_raw is
 * what FPM_TONE_create() uses when called with cfg = NULL (FPM_FSM_init() starts from it too), FPM_MTD_CFG_raw the same for FPM_MTD_create(); the fax and
 * Bell 103 setup code also copy words of these blocks into their own configurations.  MTDb103Org_CFG_raw is a block of the Bell 103 receiver, whose
 * word 2 is reused there.
 */
extern const int FPM_TONE_CFG_raw[9];
extern const int MTDb103Org_CFG_raw[3];
extern const int FPM_MTD_CFG_raw[3];

/* Template words for FPM_FSM_init() / FPM_MRF_init() configurations: the fax and caller-ID setup code copies them and patches some halves.  Neither init
 * function substitutes a default itself. */
extern const int FPM_FSM_CFG_raw[2];
extern const int FPM_MRF_CFG_raw[4];

#endif
