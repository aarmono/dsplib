/**
 * @file fax_v27_rx_tables.h
 * @brief Declarations of the constant tables of the fax V.27ter receiver (fax_v27_rx_tables.c).
 *
 * Every rate dependent table exists once per rate (suffix _2400 / _4800) and as a two-entry selector indexed by the rate index (0 = 2400 bit/s,
 * 1 = 4800 bit/s).  Table symbols that are stored in a configuration word are CFGP handles (core/dsp_table_handles.h).
 */
#ifndef DSPLIBS_PORT_FAX_V27RX_TABLES_H
#define DSPLIBS_PORT_FAX_V27RX_TABLES_H
#include "core/dsp_table_handles.h"
extern const short V27RX_FSE_IFILT_2400[97];
extern const short V27RX_FSE_IFILT_4800[81];
extern const short V27RX_FSE_QFILT_2400[97];
extern const short V27RX_FSE_QFILT_4800[81];
extern const short V27RX_CRR_TABLE_2400[4];
extern const short V27RX_CRR_TABLE_4800[40];
extern const short V27RX_FSE_PLLK1_2400[3];
extern const short V27RX_FSE_PLLK1_4800[3];
extern const short V27RX_FSE_PLLK2_2400[3];
extern const short V27RX_FSE_PLLK2_4800[3];
extern const short V27RX_MRF_FILT_2400[270];
extern const short V27RX_MRF_FILT_4800[36];
extern const short V27RX_SRE_FILT_2400[271];
extern const short V27RX_SRE_FILT_4800[201];
extern const short V27RX_XB_COFFS_2400[11];
extern const short V27RX_XB_COFFS_4800[11];
extern const short V27RX_XCLOCK_2400[6];
extern const short V27RX_XCLOCK_4800[5];
extern const short V27RX_YCLOCK_2400[6];
extern const short V27RX_YCLOCK_4800[5];
extern const short V27RX_SRE_PLLK1_2400[3];
extern const short V27RX_SRE_PLLK1_4800[3];
extern const short V27RX_SRE_PLLK2_2400[3];
extern const short V27RX_SRE_PLLK2_4800[3];
extern const short V27_MTD_COEFF_2400[10];
extern const short V27_MTD_COEFF_4800[10];
extern const short V27RX_DEC_LAST_PHASE_2400[4];
extern const short V27RX_DEC_LAST_PHASE_4800[8];
extern const unsigned short V27RX_DEC_PMAP_2400[4];
extern const unsigned short V27RX_DEC_PMAP_4800[8];
extern const unsigned short V27RX_DEC_PHS_MASK[2];
extern const unsigned short V27RX_CRR_ADJUST[2];
extern const unsigned short V27RX_CRR_TABLE_LEN[2];
extern const unsigned short V27RX_FSE_FILT_LEN[2];
extern const unsigned short V27RX_FSE_MU_TRACK[2];
extern const unsigned short V27RX_FSE_MU_TRAIN[2];
extern const unsigned short V27RX_MRF_DOWN[2];
extern const unsigned short V27RX_MRF_FILT_LEN[2];
extern const unsigned short V27RX_MRF_UP[2];
extern const unsigned short V27RX_SAMP_PER_BAUD[2];
extern const unsigned short V27RX_SRE_FILT_LEN[2];
extern const short *const V27RX_CRR_TABLE[2];
extern const short *const V27RX_DEC_LAST_PHASE[2];
extern const short *const V27RX_FSE_IFILT[2];
extern const short *const V27RX_FSE_QFILT[2];
extern const short *const V27RX_FSE_PLLK1[2];
extern const short *const V27RX_FSE_PLLK2[2];
extern const short *const V27RX_MRF_FILT[2];
extern const short *const V27RX_SRE_FILT[2];
extern const short *const V27RX_SRE_PLLK1[2];
extern const short *const V27RX_SRE_PLLK2[2];
extern const short *const V27RX_XB_COFFS[2];
extern const short *const V27RX_XCLOCK[2];
extern const short *const V27RX_YCLOCK[2];
extern const unsigned short *const V27RX_DEC_PMAP[2];
extern const short AGC_DEF_ALPHA_v27[2];
extern const short AGC_DEF_BETA_v27[2];
extern const int AGCv27_CFG[6];
extern const int V27RX_CFG[7];
extern const int V27RX_CTL[5];
extern const char *const V27RX_MESG[8];
#endif
