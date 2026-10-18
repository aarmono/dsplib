/**
 * @file fax_v17_tx_tables.h
 * @brief Constant tables of the fax V.17 transmitter (fax_v17_tx.c): per-rate parameters, pulse-shaping filters and the trellis constellation maps.
 *
 * Declarations only; the data is in fax_v17_tx_tables.c.  The VTBv17_* constellation maps (16T / 32 / 64 / 128 points) are shared with the V.17 receiver
 * (V17RX_create uses these definitions).
 */
#ifndef DSPLIBS_PORT_FAX_V17TX_TABLES_H
#define DSPLIBS_PORT_FAX_V17TX_TABLES_H
/* per-rate tables; index = V17TxHdx::rate: 0 = 7200, 1 = 9600, 2 = 12000, 3 = 14400 bit/s */
extern const short V17TX_PATTERN_SCR1[4];
extern const short V17TX_SYM_SIZE[4]; /* bits per symbol: 3 / 4 / 5 / 6 */
extern const int V17TX_PPS_SCALE[4];
extern const int V17TX_CTL[5]; /* default control record words */
extern const int V17TX_CFG[8]; /* default configuration block (V17TxCfg::w) */
extern const char *const V17TX_MESG[10]; /* status messages indexed by V17TxStatus */
/* the pulse shaper's polyphase filters (120 taps, interpolate by 10, decimate by 3) */
extern const short PPSv17_ICOFFS[120];
extern const short PPSv17_QCOFFS[120];
/* constellation maps: symbol code (low byte of the encoder output) to I / Q amplitude; the extra last entry is the zero (no-carrier) point */
extern const short VTBv17_IMAP16T[17], VTBv17_QMAP16T[17];
extern const short VTBv17_IMAP32[33], VTBv17_QMAP32[33];
extern const short VTBv17_IMAP64[65], VTBv17_QMAP64[65];
extern const short VTBv17_IMAP128[129], VTBv17_QMAP128[129];
#endif
