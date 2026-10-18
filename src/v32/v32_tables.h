/**
 * @file v32_tables.h
 * @brief Declarations of the V.32 / V.32bis configuration blocks, coefficient tables and lookup tables defined in v32_tables.c.
 *
 * The constellation maps, Viterbi decoder tables and decision angle / magnitude tables are declared in v32_viterbi_tables.h.
 */
#ifndef DSPLIBS_PORT_V32_TABLES_H
#define DSPLIBS_PORT_V32_TABLES_H

/* Default configuration blocks (word arrays for the FPM_*_init functions) */
extern const int V32_CFG_raw[12];
extern const int AGCv32_CFG_raw[6];     /* -> V32_AGC_DEF_ALPHA / BETA */
extern const int AGCv32Prc_CFG_raw[6];  /* -> V32_AGC_DEF_ALPHA / BETA */
extern const int MRFv32_CFG_raw[4];     /* -> MRFv32_COFFS */
extern const int ECCv32_CFG_raw[6];     /* -> ECCv32_IMAP / QMAP */
extern const int SREv32_CFG_raw[14];
extern const int FSEv32_CFG_raw[14];
extern const int PPSv32_CFG_raw[10];
extern const short SDMv32_CFG[3];

/* Coefficient / filter tables */
extern const short V32_S_DATA_COEF[15];
extern const short V32_S_COEF[15];
extern const short MRFv32_COFFS[360];
extern const short SREv32_COFFS[181];
extern const short PPSv32_QCOFFS[120];
extern const short PPSv32_ICOFFS[120];
extern const short SREv32_XB_COFFS[11];
/* ECCv32_IMAP / ECCv32_QMAP are not flat coefficient data: each is a 6-slot array of pointers to constellation maps, one per rate class. */
extern const short *const ECCv32_IMAP[6];
extern const short *const ECCv32_QMAP[6];
extern const short FSEv32_ICOFF[103];
extern const short FSEv32_QCOFF[103];

/* Rate-negotiation tables (indexed 0-6) */
extern const short V32_RATE_SEQ[7];
extern const short V32_ESEQ[7];
extern const short V32_FINAL_RATE_SEQ[7];
extern const short V32DiconnectThreshTable[8];

/* Timing / protocol / mode tables */
extern const short V32_TURNAROUND_DLY[2];
extern const short V32_SYMBOL_LEN[2];
extern const short V32_SAMPLE_LEN[2];
extern const short V32_CTL[16];
extern const short V32_CONNECT[7];
extern const short V32_RX_MODE[7];
extern const short V32_TX_MODE[7];

/* Small scalar / state constant tables */
/* Declared with 5 entries: FPM_FSE_receive() reads index 4 when the phase accumulator reaches the modulus (the entry is 0). */
extern const short CRRv32_CLK[5];
extern const short CRRv32_PLL_K1[3];
extern const short CRRv32_PLL_K2[3];
extern const short SREv32_PLL_K1[3];
extern const short SREv32_PLL_K2[3];
extern const short SREv32_xCLOCK[3];
extern const short SREv32_yCLOCK[3];
extern const short RATEv32[6];
extern const short FSEv32_decision[6];

/* AGC default alpha / beta pair of V.32 */
extern const short V32_AGC_DEF_ALPHA[2];
extern const short V32_AGC_DEF_BETA[2];

/* Trellis encoder tables (v32_scrambler.c) and the FPM_atan lookup (v32_math.c) */
extern const short TrellisTransitionTable[32];
extern const short TrellisEncodeDifTable[16];
extern const short SMCv32_MOD[8];
extern const short FPM_atan_table[257];

/* Status, retrain and scrambler tap tables (v32_engine.c) and the FPM_log10 lookup (v32_math.c); FPM_log10_table has 129 entries, the last is reached
 * by a mantissa >= 0x7fc0 */
extern const short PROTOCOL[9];
extern const short SnrToRetrainTable[6];
extern const short SDMv32_GPA[4];
extern const short SDMv32_GPC[4];
extern const short FPM_log10_table[129];

#endif
