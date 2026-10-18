/**
 * @file fsk_tables.h
 * @brief Declarations of the Bell 103 / V.21 and V.23 forward-channel coefficient tables and default configuration blocks (fsk_tables.c).
 */
#ifndef DSPLIBS_PORT_B103_V23_TABLES_H
#define DSPLIBS_PORT_B103_V23_TABLES_H

/** Bell 103 transmit (270 = 10 branches x 27 taps) and receive (90 = 3 branches x 30 taps) matched-filter FIR coefficients. */
extern const short B103_MRF_FILT_TX[270];
extern const short B103_MRF_FILT_RX[90];

/** Bell 103 band-pass correlators applied to the received samples: answer role (50 taps) and originate role (40 taps). */
extern const short B103_BPF_ANSWER[50];
extern const short B103_BPF_CALLER[40];

/** Bell 103 / V.21 FSK demodulator input FIR (15 taps) and IIR low-pass cascade (3 sections). */
extern const short B103_CHAN_INTRP[15];
extern const short B103_IIR_LPF[15];

/** V.23 forward-channel receive matched filter (3 branches x 16 taps). */
extern const short V23_MRF_FILT[48];

/** V.23 forward-channel FSK demodulator input FIR (15 taps) and IIR low-pass cascade (3 sections). */
extern const short V23RX_ANSWER_INTRP[15];
extern const short V23RX_IIR_LPF[15];

/** V.23 forward-channel IIR pre-filter cascade (4 sections). */
extern const short V23_IIR_FILT[20];

/**
 * AGC alpha / beta weights (Q15; only the first entry of each is read) of the Bell 103 engine and of the V.23 forward-channel receiver.  The values
 * are the same, the tables are separate.
 */
extern const short B103_AGC_DEF_ALPHA[2];
extern const short B103_AGC_DEF_BETA[2];
extern const short V23_AGC_DEF_ALPHA[2];
extern const short V23_AGC_DEF_BETA[2];

/**
 * Raw configuration blocks copied by B103FP_create() / v23FP_rx_create() / v23FP_tx_create(); the word meanings are those of the object they
 * configure (see the descriptions in fsk_tables.c).
 */
extern const int B103_CFG_raw[7];    ///< default configuration of B103FP_create() (cfg == NULL)
extern const int AGCb103_CFG_raw[6]; ///< Bell 103 AGC; words 3 and 4 are the B103_AGC_DEF_* handles
extern const int FPM_FSD_CFG_raw[7]; ///< default FSK demodulator configuration
extern const int MRFv23_CFG_raw[4];  ///< V.23 receive matched filter
extern const int FSDv23_CFG_raw[7];  ///< V.23 forward-channel FSK demodulator
extern const int TONEv23_CFG_raw[9]; ///< V.23 tone generator / detector
extern const int AGCv23_CFG_raw[6];  ///< V.23 forward-channel AGC; words 3 and 4 are the V23_AGC_DEF_* handles

#endif
