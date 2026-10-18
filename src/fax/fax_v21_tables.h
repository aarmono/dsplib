/**
 * @file fax_v21_tables.h
 * @brief Declarations of the constant tables of the fax V.21 channel (fax_v21.c); the definitions and their documentation are in fax_v21_tables.c.
 */
#ifndef DSPLIBS_PORT_FAX_V21_TABLES_H
#define DSPLIBS_PORT_FAX_V21_TABLES_H
#include "core/dsp_table_handles.h"
extern const short V21_MRF_FILT[360];
extern const short V21RX_IIR_LPF[15];
extern const short V21RX_CHAN1_INTRP[15];
extern const short V21RX_CHAN2_INTRP[15];
extern const short AGC_DEF_ALPHA_v21[2];
extern const short AGC_DEF_BETA_v21[2];
extern const short V21_CHAN1_MTD_COEFF[10];
extern const short V21_CHAN2_MTD_COEFF[10];
extern const int AGCv21_CFG[6];
extern const int V21RX_CTL[4];
extern const int V21RX_CFG[6];
extern const int V21TX_CTL[5];
extern const int V21TX_CFG[7];
extern const char *const V21RX_MESG[7];
extern const char *const V21TX_MESG[6];
#endif
