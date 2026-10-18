/**
 * @file fax_v27_tx_tables.h
 * @brief Constant tables of the fax V.27ter transmitter (fax_v27_tx.c): per-rate state-machine counts, SGD patterns, constellation and pulse-shaper tables.
 *
 * Declarations only; the data is in fax_v27_tx_tables.c.  The per-rate tables are indexed by the rate class V27TxHdx::rate (0 = 2400 bit/s, 1 = 4800 bit/s).
 * The scrambler bit counts V27TX_SDM_NUM_BITS live in fax_common_tables.h.
 */
#ifndef DSPLIBS_PORT_FAX_V27_TABLES_H
#define DSPLIBS_PORT_FAX_V27_TABLES_H
/* index = V27TxHdx::rate: 0 = 2400 bit/s, 1 = 4800 bit/s.  ALT_COUNT and EQCOND_COUNT are indexed by V27TxHdx::longTrain instead (0 short, 1 long training). */
extern const unsigned short V27TX_ALT_COUNT[2];
extern const unsigned short V27TX_EQCOND_COUNT[2];
extern const unsigned short V27TX_FRMSIZE[2];
extern const unsigned short V27TX_NOCARR_SYMBOL[2];
extern const unsigned short V27TX_PATTERN_ALT[2];
extern const unsigned short V27TX_PATTERN_CARR[2];
extern const unsigned short V27TX_PATTERN_SCR1[2];
extern const unsigned short V27TX_PPS_DOWN_FACT[2];
extern const unsigned short V27TX_PPS_UP_FACT[2];
extern const unsigned short V27TX_PPS_FILT_LEN[2];
extern const unsigned short V27TX_SMC_CRR_ADJ[2];
extern const unsigned short V27TX_SMC_CRR_LEN[2];
extern const unsigned short V27TX_SMC_PHS_MASK[2];
extern const unsigned short V27TX_SMC_PMAP_24[4];
extern const unsigned short V27TX_SMC_PMAP_48[8];
extern const short V27TX_PPS_IMAP_2400[5];
extern const short V27TX_PPS_IMAP_4800[9];
extern const short V27TX_PPS_QMAP_2400[5];
extern const short V27TX_PPS_QMAP_4800[9];
extern const short V27TX_PPS_IFILT_2400[120];
extern const short V27TX_PPS_IFILT_4800[60];
extern const short V27TX_PPS_QFILT_2400[120];
extern const short V27TX_PPS_QFILT_4800[60];
extern const int V27TX_PPS_SCALE[2];
extern const short *const V27TX_PPS_IMAP[2];
extern const short *const V27TX_PPS_QMAP[2];
extern const short *const V27TX_PPS_IFILT[2];
extern const short *const V27TX_PPS_QFILT[2];
extern const unsigned short *const V27TX_SMC_PMAP[2];
extern const int V27TX_CTL[5]; /* default control record words */
extern const int V27TX_CFG[8]; /* default configuration block (V27Cfg::w) */
extern const char *const V27TX_MESG[8]; /* status messages indexed by V27TxStatus */
#endif
