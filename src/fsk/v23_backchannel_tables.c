/**
 * @file v23_backchannel_tables.c
 * @brief Tables of the V.23 engine: samples-per-bit tables of the transmitter and the coefficients and configuration of the backward-channel receiver (v23.c).
 *
 * Data only (declarations in v23_backchannel_tables.h): the two three-entry samples-per-bit tables that make the average bit length fractional, the
 * block-size table of the 75 bit/s demodulator, its mark / space filter sections, its IIR pre-filter cascade and its AGC configuration.  Table pointers
 * inside the configuration block are table handles (CFGP(), core/dsp_table_handles.h) registered at the end of the file.
 */
#include <stdint.h>
#include "core/dsp_table_handles.h"
#include "v23_backchannel_tables.h"

/* Forward channel (1200 bit/s) samples per bit, rotated through for successive bits: (7 + 7 + 6) / 3 = 6.667 = 8000 / 1200 */
const short fw_ch_samp_per_bit_table[3] = { 7, 7, 6 };

/* Backward channel (75 bit/s) samples per bit: (107 + 107 + 106) / 3 = 106.67 = 8000 / 75 */
const short bw_ch_samp_per_bit_table[3] = { 107, 107, 106 };

/* Backward-channel receiver: samples per decision block, rotated through (index modulo 12); the average, 26.67, is a quarter of a bit at 75 bit/s */
const int block_size_table[12] = {
    26, 27, 26, 27, 27, 27, 26, 27, 27, 27, 26, 27,
};

/* Backward-channel receiver: mark-tone (390 Hz) filter section (FPM_iir_filt, 5 coefficients) */
const short Mark_Coefs[5] = { -14786, 16384, 29861, -31242, 16384 };

/* Backward-channel receiver: space-tone (450 Hz) filter section (FPM_iir_filt, 5 coefficients) */
const short Space_Coefs[5] = { -14786, 16384, 29206, -30743, 16384 };

/* Backward-channel receiver: IIR pre-filter applied to the received block (FPM_iir_filt_II): 3 sections of 5 coefficients */
const short BwCh_IIR_FILT[15] = {
    36, 0, 36, 0, -14009, 16384, 16384, -13845, 13198, -27940,
    16384, 16384, -23027, 15350, -28296,
};

/* AGC alpha / beta weights of the backward-channel receiver (Q15; same values as V23_AGC_DEF_ALPHA / _BETA in fsk_tables.c, kept as separate tables) */
const short V23_BWCH_AGC_DEF_ALPHA[2] = { 16384, 32604 };
const short V23_BWCH_AGC_DEF_BETA[2]  = { 16384, 1638 };

/* AGC configuration of the backward-channel receiver (layout: FpmAgc in fpm.h).  Words 3 and 4 are the handles of V23_BWCH_AGC_DEF_ALPHA / _BETA. */
const int AGCv23_BWCH_CFG_raw[6] = {
    0x504000, 0x3e80050, 0x280000, CFGP(V23_BWCH_AGC_DEF_ALPHA), CFGP(V23_BWCH_AGC_DEF_BETA), 0x0,
};

CFGP_REGISTER(v23_bwch_tables)
