/**
 * @file dsp_constants.h
 * @brief Named constants that several modules share: data rates and sample rates.
 *
 * Plain macros that expand to the decimal literal they replace (an `int` constant, no suffix), so a use compiles exactly like the literal.  A name is
 * used only where the member, parameter or switch is verified to hold that quantity; the same numbers also occur as V.34 baud / carrier codes
 * (src/v34/v34_symbol_rates.h), sample rates, tone frequencies and plain counts: convert by member, never by value.
 *
 * Data rates (bit/s) as the data pumps and the host exchange them: the V.32 / V.32bis wrapper rates, the fax modem config rate words, the T.30 modem
 * rates and the V.22 / V.22bis rates (modem_set_param MDMPRM_TX_RATE / MDMPRM_RX_RATE).
 */
#ifndef DSP_CONSTANTS_H
#define DSP_CONSTANTS_H

#define RATE_14400 14400 /* 0x3840 */
#define RATE_12000 12000 /* 0x2ee0 */
#define RATE_9600 9600   /* 0x2580 */
#define RATE_7200 7200   /* 0x1c20 */
#define RATE_4800 4800   /* 0x12c0 */
#define RATE_2400 2400   /* 0x960 */
#define RATE_1200 1200   /* 0x4b0 */

/* Sample rates (Hz): the line rate of a data pump (dp create's `srate`; 8000 or 9600, 48000 for the resampler pairs), the caller ID / voice sampling
 * rate members (`fs`, Voice.rate: also 7200 and 11025 for voice), and seconds-to-samples conversions at 8000 Hz.  The V.90 / V.92 symbol rate and the
 * V.34 setup constants (0x1f40 as a Q-scaled value) are NOT sample rates and keep their literals. */
#define SAMPLE_RATE_7200 7200   /* 0x1c20 */
#define SAMPLE_RATE_8000 8000   /* 0x1f40 */
#define SAMPLE_RATE_9600 9600   /* 0x2580 */
#define SAMPLE_RATE_11025 11025 /* 0x2b11 */
#define SAMPLE_RATE_48000 48000 /* 0xbb80 */

#endif /* DSP_CONSTANTS_H */
