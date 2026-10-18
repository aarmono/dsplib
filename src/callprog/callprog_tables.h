/**
 * @file callprog_tables.h
 * @brief Declarations of the coefficient tables of the call-progress helpers (defined in callprog_tables.c).
 *
 * callprog_dual_tone.c reads callprog_fptone, callprog_fact_fp, callprog_iir_scales and callprog_iir_coef; callprog.c reads the rest.
 */
#ifndef DSPLIBS_PORT_CALLPROG_TABLES_H
#define DSPLIBS_PORT_CALLPROG_TABLES_H

extern const short callprog_fptone[513];
extern const short callprog_fact_fp[7];
extern const short callprog_iir_scales[4];
extern const short callprog_iir_coef[15];
extern const short callprog_biquad_scales[5];
extern const short callprog_filter_b[12];
extern const short callprog_filter_a[12];
extern const int callprog_default_cfg[7];

#endif /* DSPLIBS_PORT_CALLPROG_TABLES_H */
