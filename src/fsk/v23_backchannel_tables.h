/**
 * @file v23_backchannel_tables.h
 * @brief Declarations of the V.23 transmitter samples-per-bit tables and the backward-channel (75 bit/s) receiver tables (v23_backchannel_tables.c).
 */
#ifndef DSPLIBS_PORT_V23_BWCH_TABLES_H
#define DSPLIBS_PORT_V23_BWCH_TABLES_H

/**
 * Samples-per-bit tables of the transmitter (CreateV23Modem()): an instance uses the table of the channel it TRANSMITS on, i.e. the forward-channel
 * table when its receiver is the backward-channel one, and the backward-channel table otherwise.
 */
extern const short fw_ch_samp_per_bit_table[3]; ///< 1200 bit/s: about 6.667 samples per bit
extern const short bw_ch_samp_per_bit_table[3]; ///< 75 bit/s: about 106.67 samples per bit

/**
 * Samples per decision block of the backward-channel receiver, indexed modulo 12 by BwChDem_Progress(); entry 0 also seeds the first block in
 * BwChDem_Create().
 */
extern const int block_size_table[12];

/** Backward-channel mark-tone and space-tone filter sections (5 coefficients each). */
extern const short Mark_Coefs[5];
extern const short Space_Coefs[5];

/** Backward-channel IIR pre-filter cascade (3 sections of 5 coefficients). */
extern const short BwCh_IIR_FILT[15];

/** AGC configuration of the backward-channel receiver (words 3 and 4 are the handles of the two weight tables) and the weight tables (Q15). */
extern const int AGCv23_BWCH_CFG_raw[6];
extern const short V23_BWCH_AGC_DEF_ALPHA[2];
extern const short V23_BWCH_AGC_DEF_BETA[2];

#endif
