/**
 * @file v8_tables.h
 * @brief Declarations of the V.8 engine's coefficient tables (v8_tables.c).
 *
 * V.8 has its own, self-contained primitive set (AGC, FSK modulator and demodulator, DFT-based tone detector, cosine and square-root tables); it
 * does not share the tables of the FPM toolkit.  The consumers are in v8.c.
 */
#ifndef DSPLIBS_PORT_V8_TABLES_H
#define DSPLIBS_PORT_V8_TABLES_H

#ifdef __cplusplus
extern "C"
{
#endif

/* V8agc's entrance band-pass filters, selected by side (41 coefficients stored, the first 40 are used). */

extern const short CALLER_Entrance_Filter[41];
extern const short ANSWER_Entrance_Filter[41];

/* V8agc's square-root lookup table for the signal level (192 entries). */

extern const short v8_sqrt_table[192];

/* charFlip's nibble-reversal table: v8_wordFlip[n] = the bit reversal of the 4-bit value n */
extern const unsigned char v8_wordFlip[16];

/* The coefficients of the 2100 Hz resonators of the ANSam detector: a flat block of 8 shorts, [0..3] feed-forward (all 0), [4..7] feed-back. */

extern const short v8_c2100[8];

/* The 2-pole IIR in front of the resonators in v8_tone_detect(): numerator b[0..2], denominator a[]; only a[1] and a[2] are read. */

extern const short v8_tone_b[3];
extern const short v8_tone_a[3];

/* v8_cosread's cosine table: 256 points of a period, Q14 (V.8's own, separate from the FPM table) */
extern const short v8_costbl[256];

/* The 61-tap transmit shaping filters of the V.21 low and high channels (loaded into V8Obj::v21Coef by v8_V21_Init). */

extern const short temp_v21_lobnd[61];
extern const short temp_v21_hibnd[61];

/*
 * The correlator bank of v8_fskdemodulate(): per channel, mark and space, in-phase (filter1) and quadrature (filter2) coefficients (41 stored, 40
 * used).  L = low channel, H = high channel, M = mark, S = space.
 */

extern const short LM_filter1_coef[41];
extern const short LM_filter2_coef[41];
extern const short LS_filter1_coef[41];
extern const short LS_filter2_coef[41];
extern const short HM_filter1_coef[41];
extern const short HM_filter2_coef[41];
extern const short HS_filter1_coef[41];
extern const short HS_filter2_coef[41];

#ifdef __cplusplus
}
#endif

#endif
