/**
 * @file fax_v29_tx_tables.h
 * @brief Constant tables of the fax V.29 transmitter (fax_v29_tx.c): constellation maps, pulse-shaping filters, per-rate parameters, default configuration.
 *
 * Declarations only; the data is in fax_v29_tx_tables.c.  Per-rate tables are indexed by the rate class of V29TxHdx::rate (0 = 7200 bit/s, 1 = 9600 bit/s).
 */
#ifndef DSPLIBS_PORT_FAX_V29_TABLES_H
#define DSPLIBS_PORT_FAX_V29_TABLES_H
/* per-rate parameters; index = V29TxHdx::rate (0 = 7200 bit/s, 1 = 9600 bit/s, or any other configured rate) */
extern const short V29TX_PATTERN_SCR1[2];
extern const int V29TX_PPS_SCALE[2];
/* the 16-point V.29 constellation encoder (FaxSmcCfg tables): differential phase map, carrier sine / cosine (Q15, 24 phases), absolute I / Q maps */
extern const unsigned short V29TX_SMC_PMAP[8];
extern const short V29TX_SMC_SINE[24];
extern const short V29TX_SMC_COSINE[24];
extern const short V29TX_SMC_IMAP[16];
extern const short V29TX_SMC_QMAP[16];
/* the pulse shaper's polyphase filters (120 taps, interpolate by 10, decimate by 3) */
extern const short V29TX_PPS_IFILT[120];
extern const short V29TX_PPS_QFILT[120];
extern const int V29TX_CTL[4];  /* default control record words */
extern const int V29TX_CFG[7];  /* default configuration block (V29Cfg words) */
extern const char *const V29TX_MESG[8]; /* status messages indexed by V29TxStatus */
#endif
