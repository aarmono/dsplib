/**
 * @file fax_v29_rx_tables.h
 * @brief Declarations of the constant tables of the fax V.29 receiver (fax_v29_rx_tables.c).
 *
 * Filter coefficients, timing-recovery and carrier tables, the decision constellation and the configuration / message tables used by fax_v29_rx.c.
 * Table symbols that are stored in a configuration word are CFGP handles (core/dsp_table_handles.h).
 */
#ifndef DSPLIBS_PORT_FAX_V29RX_TABLES_H
#define DSPLIBS_PORT_FAX_V29RX_TABLES_H
#include "core/dsp_table_handles.h"
extern const short V29RX_FSE_IFILT[49];
extern const short V29RX_FSE_QFILT[49];
extern const short V29RX_CRR_TABLE[72];
extern const short V29RX_FSE_PLLK1[3];
extern const short V29RX_FSE_PLLK2[3];
extern const short V29RX_MRF_FILT[270];
extern const short V29RX_SRE_FILT[181];
extern const short V29RX_XB_COFFS[11];
extern const short V29RX_XCLOCK[3];
extern const short V29RX_YCLOCK[3];
extern const short V29RX_SRE_PLLK1[3];
extern const short V29RX_SRE_PLLK2[3];
extern const short V29_MTD_COEFF[10];
extern const short V29RX_DEC_ANGLE[16];
extern const short V29RX_DEC_IMAP[16];
extern const short V29RX_DEC_QMAP[16];
extern const short V29RX_DEC_MAG[16];
extern const unsigned short V29RX_DEC_PMAP[8];
extern const short AGC_DEF_ALPHA_v29[2];
extern const short AGC_DEF_BETA_v29[2];
extern const int AGCv29_CFG[6];
extern const int V29RX_CFG[6];
extern const int V29RX_CTL[4];
extern const char *const V29RX_MESG[8];
#endif
