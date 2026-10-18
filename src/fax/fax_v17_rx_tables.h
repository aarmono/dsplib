/**
 * @file fax_v17_rx_tables.h
 * @brief Declarations of the V.17 receive tables: filter and clock coefficients, PLL gains, AGC defaults, the default configuration / control words and the
 *   status messages.
 *
 * Table symbols that are stored in a configuration word are table handles (src/core/dsp_table_handles.h).
 */
#ifndef DSPLIBS_PORT_FAX_V17RX_TABLES_H
#define DSPLIBS_PORT_FAX_V17RX_TABLES_H
#include "core/dsp_table_handles.h"
extern const short V17_MTD_COEFF[10];
extern const short MRFv17_COFFS[360];
extern const short SREv17_COFFS[181];
extern const short SREv17_XB_COFFS[11];
extern const short SREv17_xCLOCK[3];
extern const short SREv17_yCLOCK[3];
extern const short SREv17_PLL_K1[3];
extern const short SREv17_PLL_K1_S[3];
extern const short SREv17_PLL_K2[3];
extern const short FSEv17_ICOFF[49];
extern const short FSEv17_QCOFF[49];
extern const short CRRv17_CLK[4];
extern const short CRRv17_PLL_K1[3];
extern const short CRRv17_PLL_K1_S[3];
extern const short CRRv17_PLL_K2[3];
extern const short AGC_DEF_ALPHA_v17[2];
extern const short AGC_DEF_BETA_v17[2];
extern const int AGCv17_CFG[6];
extern const int V17RX_CFG[10];
extern const int V17RX_CTL[5];
extern const char *const V17RX_MESG[10];
#endif
