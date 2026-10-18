/**
 * @file fax_v27_tx_tables.c
 * @brief Constant tables of the fax V.27ter transmitter (fax_v27_tx.c).
 *
 * - State-machine parameters per rate class (0 = 2400, 1 = 4800 bit/s): frame size in symbols (V27TX_FRMSIZE), symbol code sent for "no carrier"
 *   (V27TX_NOCARR_SYMBOL), SGD patterns of the carrier / alternation / SCR1 states, and the alternation / equaliser-conditioning lengths (V27TX_ALT_COUNT,
 *   V27TX_EQCOND_COUNT, indexed by the long-training flag instead).
 * - Symbol encoder (V27TX_SMC_*): carrier phase step and length, phase mask, differential phase maps for 4 (2400) and 8 (4800) phases.
 * - Pulse shaper (V27TX_PPS_*): interpolation / decimation factors, filter lengths, the I / Q symbol maps and the polyphase filters of each rate, the gain.
 * - V27TX_CFG / V27TX_CTL: default configuration and control words; V27TX_MESG: the status message strings.
 */
#include "fax_v27_tx_tables.h"

/** Symbols of the alternation state, per long-training flag (short, long). */
const unsigned short V27TX_ALT_COUNT[2] = {
    14, 50,
};
/** Symbols of the equaliser-conditioning state, per long-training flag (short, long). */
const unsigned short V27TX_EQCOND_COUNT[2] = {
    58, 1074,
};
/** Symbols per frame, per rate class. */
const unsigned short V27TX_FRMSIZE[2] = {
    24, 32,
};
/** Phase-index code written while the carrier is off, per rate class. */
const unsigned short V27TX_NOCARR_SYMBOL[2] = {
    4, 8,
};
const unsigned short V27TX_PATTERN_ALT[2] = {
    3, 7,
};
const unsigned short V27TX_PATTERN_CARR[2] = {
    0, 1,
};
const unsigned short V27TX_PATTERN_SCR1[2] = {
    3, 7,
};
/** Pulse shaper decimation factor per rate class. */
const unsigned short V27TX_PPS_DOWN_FACT[2] = {
    3, 1,
};
/** Pulse shaper interpolation factor per rate class. */
const unsigned short V27TX_PPS_UP_FACT[2] = {
    20, 5,
};
/** Pulse shaper filter length (taps) per rate class. */
const unsigned short V27TX_PPS_FILT_LEN[2] = {
    120, 60,
};
/** Encoder carrier phase increment per symbol, per rate class. */
const unsigned short V27TX_SMC_CRR_ADJ[2] = {
    2, 1,
};
/** Encoder carrier phase modulus (phases per carrier cycle), per rate class. */
const unsigned short V27TX_SMC_CRR_LEN[2] = {
    4, 8,
};
/** Mask of the phase-index bits taken from the symbol code, per rate class. */
const unsigned short V27TX_SMC_PHS_MASK[2] = {
    3, 7,
};
/** Differential phase map, 2400 bit/s (4 phases). */
const unsigned short V27TX_SMC_PMAP_24[4] = {
    0, 1, 3, 2,
};
/** Differential phase map, 4800 bit/s (8 phases). */
const unsigned short V27TX_SMC_PMAP_48[8] = {
    1, 0, 2, 3, 6, 7, 5, 4,
};
/** Pulse shaper I / Q symbol maps (phase index to amplitude), 2400 bit/s: 4 points + the zero entry. */
const short V27TX_PPS_IMAP_2400[5] = {
    6144, 0, -6144, 0, 0,
};
/** Pulse shaper I / Q symbol maps, 4800 bit/s: 8 points + the zero entry. */
const short V27TX_PPS_IMAP_4800[9] = {
    6144, 4344, 0, -4344, -6144, -4344, 0, 4344, 0,
};
const short V27TX_PPS_QMAP_2400[5] = {
    0, 6144, 0, -6144, 0,
};
const short V27TX_PPS_QMAP_4800[9] = {
    0, 4344, 6144, 4344, 0, -4344, -6144, -4344, 0,
};
/** Polyphase pulse-shaping filters, 2400 bit/s (120 taps). */
const short V27TX_PPS_IFILT_2400[120] = {
    -21, 61, 69, -21, -185, -355, -453, -429, -288, -91, 72, 126, 50, -105, -236,
    -236, -49, 295, 680, 950, 982, 749, 343, -58, -264, -176, 156, 531, 672, 355,
    -470, -1619, -2710, -3299, -3065, -1967, -298, 1401, 2544, 2755, 2055, 898, 14, 103, 1493,
    3900, 6417, 7776, 6805, 2937, -3428, -10872, -17286, -20483, -18936, -12362, -1951, 9870, 20092, 26004,
    26004, 20092, 9870, -1951, -12362, -18936, -20483, -17286, -10872, -3428, 2937, 6805, 7776, 6417, 3900,
    1493, 103, 14, 898, 2055, 2755, 2544, 1401, -298, -1967, -3065, -3299, -2710, -1619, -470,
    355, 672, 531, 156, -176, -264, -58, 343, 749, 982, 950, 680, 295, -49, -236,
    -236, -105, 50, 126, 72, -91, -288, -429, -453, -355, -185, -21, 69, 61, -21,
};
/** Polyphase pulse-shaping filters, 4800 bit/s (60 taps). */
const short V27TX_PPS_IFILT_4800[60] = {
    -62, -12, -37, -141, 6, -56, -36, 109, -23, 158, 167, -14, 235, -93, -246,
    10, -429, -136, -163, -546, 619, 28, 680, 3388, -278, -636, 1288, -12667, -11245, 19992,
    19992, -11245, -12667, 1288, -636, -278, 3388, 680, 28, 619, -546, -163, -136, -429, 10,
    -246, -93, 235, -14, 167, 158, -23, 109, -36, -56, 6, -141, -37, -12, -62,
};
const short V27TX_PPS_QFILT_2400[120] = {
    -5, 52, 166, 273, 302, 217, 36, -178, -338, -381, -299, -147, -21, -8, -145,
    -385, -618, -713, -580, -228, 236, 640, 827, 734, 431, 108, -12, 220, 786, 1479,
    1959, 1896, 1122, -260, -1878, -3210, -3783, -3382, -2173, -662, 493, 767, 33, -1311, -2437,
    -2390, -505, 3221, 7968, 12233, 14277, 12730, 7160, -1612, -11604, -20173, -24786, -23828, -17160, -6243,
    6243, 17160, 23828, 24786, 20173, 11604, 1612, -7160, -12730, -14277, -12233, -7968, -3221, 505, 2390,
    2437, 1311, -33, -767, -493, 662, 2173, 3382, 3783, 3210, 1878, 260, -1122, -1896, -1959,
    -1479, -786, -220, 12, -108, -431, -734, -827, -640, -236, 228, 580, 713, 618, 385,
    145, 8, 21, 147, 299, 381, 338, 178, -36, -217, -302, -273, -166, -52, 5,
};
const short V27TX_PPS_QFILT_4800[60] = {
    73, -7, 90, -34, -80, 4, -150, -45, -37, -135, 143, 23, 97, 387, -19,
    131, 103, -328, 100, -640, -725, 17, -1642, 813, 3536, 50, 5367, 5247, -18350, -17074,
    17074, 18350, -5247, -5367, -50, -3536, -813, 1642, -17, 725, 640, -100, 328, -103, -131,
    19, -387, -97, -23, -143, 135, 37, 45, 150, -4, 80, 34, -90, 7, -73,
};
/** Pulse shaper output gain per rate class (the config's gain word multiplies it). */
const int V27TX_PPS_SCALE[2] = {34767, 34767};
/** Per-rate-class pointers to the maps, filters and phase map above. */
const short *const V27TX_PPS_IMAP[2] = {V27TX_PPS_IMAP_2400, V27TX_PPS_IMAP_4800};
const short *const V27TX_PPS_QMAP[2] = {V27TX_PPS_QMAP_2400, V27TX_PPS_QMAP_4800};
const short *const V27TX_PPS_IFILT[2] = {V27TX_PPS_IFILT_2400, V27TX_PPS_IFILT_4800};
const short *const V27TX_PPS_QFILT[2] = {V27TX_PPS_QFILT_2400, V27TX_PPS_QFILT_4800};
const unsigned short *const V27TX_SMC_PMAP[2] = {V27TX_SMC_PMAP_24, V27TX_SMC_PMAP_48};
/** Default control record words (see FaxV27Ctl). */
const int V27TX_CTL[5] = {0x2580, 0xea60, 0x1, 0x0, 0x0};
/** Default configuration block (V27Cfg::w).  Its rate word is 9600, which V.27ter does not support: a create with the default block takes the error path. */
const int V27TX_CFG[8] = {0x25800000, 0x0, 0xea60, 0x1, 0x0, 0x1, 0x0, 0x0};
/** Status messages indexed by the V27TxStatus code (entries 2 and 3 read 4800 / 2400 bps, swapped against the codes' meaning). */
const char *const V27TX_MESG[8] = {
    "V.27ter Transmit Data Mode",
    "V.27ter Protocal Transmit Mode",
    "CONNECT: V.27ter Transmit 4800 bps",
    "CONNECT: V.27ter Transmit 2400 bps",
    "V.27ter Transmit Idle Mode",
    "ERROR: V.27ter Tx Internal Error",
    "ERROR: Transmit Input Queue Under-run",
    "ERROR: Transmit Input Queue Over-run",
};
