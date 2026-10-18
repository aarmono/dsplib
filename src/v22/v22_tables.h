/**
 * @file v22_tables.h
 * @brief Declarations of the V.22 / V.22bis constant tables: equaliser and timing recovery tables, pulse shaper / receive converter / IIR coefficients,
 *        the configuration blocks of the chain components, and the engine creation tables (see v22_tables.c).
 */
#ifndef DSPLIBS_PORT_V22_TABLES_H
#define DSPLIBS_PORT_V22_TABLES_H
#include "v22.h"
extern const short FSEv22_CFG[4];
extern const short FSEv22_COFFS[49];
extern const short CRRv22_CLK[6];
extern const short CRRv22_PLL_K1[3];
extern const short CRRv22_PLL_K2[3];
extern const short DECv22_ANGL12[4];
extern const short DECv22_QMAP12[4];
extern const short DECv22_IMAP12[4];
extern const short DECv22_MAG24[3];
extern const short DECv22_ANGL24[16];
extern const short DECv22_IMAP24[16];
extern const short DECv22_QMAP24[16];
extern const short SMCv22_IMAP_1200BPS[16];
extern const short SMCv22_QMAP_1200BPS[16];
extern const short SMCv22_IMAP_2400BPS[16];
extern const short SMCv22_QMAP_2400BPS[16];
extern const short SMCv22_PMAP[4];
extern const short SRE_ALPHA_AVG[3];
extern const short SRE_BETA_AVG[3];
extern const short SREv22_COFFS[271];
extern const short SREv22_PLL_K1[3];
extern const short SREv22_PLL_K2[3];
extern const short SREv22_xCLOCK[6];
extern const short SREv22_yCLOCK[6];
extern const short v22_fse_mu_tab[2]; /* the equaliser adaptation step by V22Fse::mu */
/* pulse shaper, receive converter, IIR and the component configuration blocks */
extern const short PPSv22_COFFS[120];
extern const short MRFv22_COFFS[270];
extern const short IIR_a_coeff[4];
extern const short IIR_b_coeff[4];
extern const V22SdmCfg SDMv22_CFG;
extern const V22PpsCfg PPSv22_CFG;
extern const V22MrfCfg V22_MRF_CFG;
extern const V22SmcCfg SMCv22_CFG;
/* engine creation tables */
extern const V22EngCfg V22_CFG;
extern const short V22_S1_HC_COEF[20];
extern const short MTDv22_COEF[15];
extern const short MTDv22_COEF2[15];
extern const short V22_AGC_DEF_ALPHA[2];
extern const short V22_AGC_DEF_BETA[2];
extern const int MTDs1_CFG_raw[3];  /* word 0 is the table V22_S1_HC_COEF on a 32-bit host and 0 on x86-64 (V22FP_create patches the detector's pointer) */
extern const int MTDv22_CFG_raw[3];
extern const int MTDv22_CFG2_raw[3];
extern const int AGCv22_CFG_raw[6]; /* words 3 / 4: V22_AGC_DEF_ALPHA / _BETA on a 32-bit host, 0 on x86-64 */
extern const int AGCv22_CFG2_raw[6];
#endif
