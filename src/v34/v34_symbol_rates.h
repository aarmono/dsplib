/**
 * @file v34_symbol_rates.h
 * @brief V.34 symbol rates (baud), carrier frequencies (Hz) and the two non-rate selectors of V34SetupModulator().
 *
 * The modem objects carry symbol rates and carrier frequencies as the value itself in Hz (0xaf0 for 2800 baud, 0x725 for a 1829 Hz carrier, ...).  The
 * constants are plain macros that expand to the decimal literal, so a use compiles exactly like the literal it replaces (an `int` constant).  Use them only
 * where the member, parameter or switch is a baud rate or carrier: obj->baud_ / carrier_, the rate block (transmit baud and carrier, receive baud and
 * carrier), V34Rate q (baud at +0x0 / +0x12, carrier at +0x10 / +0x24), and the baud / carrier parameters of V34SetupDemodulator / V34SetupModulator /
 * initV34 / preempindex.  The same numbers also occur as data rates, sample rates and unrelated counts (2400 bit/s, 8000 Hz, 3000 samples): convert by
 * member, never by value.  Each carrier pairs with the tables hsine<Hz> / c<Hz> / ec_prem_coef_B<baud>[High] of the same number.
 */
#ifndef V34_SYMBOL_RATES_H
#define V34_SYMBOL_RATES_H

/* symbol rates (baud); the comment gives the hexadecimal value of the member */
#define V34_BAUD_2400 2400 /* 0x960 */
#define V34_BAUD_2743 2743 /* 0xab7 */
#define V34_BAUD_2800 2800 /* 0xaf0 */
#define V34_BAUD_3000 3000 /* 0xbb8 */
#define V34_BAUD_3200 3200 /* 0xc80 */
#define V34_BAUD_3429 3429 /* 0xd65 */

/* carrier frequencies (Hz) */
#define V34_CARRIER_1200 1200 /* 0x4b0 */
#define V34_CARRIER_1600 1600 /* 0x640 */
#define V34_CARRIER_1680 1680 /* 0x690 */
#define V34_CARRIER_1800 1800 /* 0x708 */
#define V34_CARRIER_1829 1829 /* 0x725 */
#define V34_CARRIER_1867 1867 /* 0x74b */
#define V34_CARRIER_1920 1920 /* 0x780 */
#define V34_CARRIER_1959 1959 /* 0x7a7 */
#define V34_CARRIER_2000 2000 /* 0x7d0 */
#define V34_CARRIER_2400 2400 /* 0x960 */

/**
 * Selectors for the first argument ("baud") of V34SetupModulator() that are not symbol rates.  They choose the pulse-shaping source table of the transmit
 * polyphase bank and are named after what the table is.  Used only as that argument and in its switch; every other value falls into the same arm as
 * V34MOD_DPSK_600 (and prints "invalid baudrate" at debug level above 1).
 *  - V34MOD_ALLPASS_4800: table txAllPass (4 rows x 32 taps, a unity-gain all-pass shaper), N = 0x20, L = 4, no echo pre-filter pointer written, no
 *    pre-emphasis row.  Passed by the SBARSEG transmit state (v04_tx_sbarseg, with carrier 2400) when it moves on to TXMD.
 *  - V34MOD_DPSK_600: table tx600c1 (16 rows x 8 taps, a low-pass interpolator with the centre tap near 0.5), N = 8, L = 0x10, echo pre-filter
 *    ec_prem_coef_B3429.  Passed by the 600-baud DPSK set-ups (v34modeminit with carrier 1200 / 2400, dpskinit with 1200 for a == 0 else 2400, which also
 *    store &bpv22high / &bpv22low).  This arm is not a silent signal (the table is a pulse shaper); it only does not print the invalid-baudrate warning.
 */
enum V34ModulatorSelector
{
  V34MOD_DPSK_600 = 600,
  V34MOD_ALLPASS_4800 = 4800
};

#endif /* V34_SYMBOL_RATES_H */
