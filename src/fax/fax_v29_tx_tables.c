/**
 * @file fax_v29_tx_tables.c
 * @brief Constant tables of the fax V.29 transmitter (fax_v29_tx.c).
 *
 * - V29TX_SMC_PMAP / _SINE / _COSINE / _IMAP / _QMAP: the 16-point constellation encoder.  PMAP maps the differential symbol code to the phase step, SINE /
 *   COSINE hold 24 carrier phases (Q15), IMAP / QMAP give the I / Q amplitude of each of the 16 absolute points.
 * - V29TX_PPS_IFILT / _QFILT: the 120-tap polyphase pulse-shaping filters (interpolate by 10, decimate by 3); V29TX_PPS_SCALE: output gain per rate class.
 * - V29TX_PATTERN_SCR1: the training pattern fed through the scrambler in the SCR1 state, per rate class.
 * - V29TX_CFG / V29TX_CTL: default configuration and control words; V29TX_MESG: the status message strings.
 */
#include "fax_v29_tx_tables.h"

/** Differential symbol code to phase-step map of the encoder. */
const unsigned short V29TX_SMC_PMAP[8] = {
    1, 0, 2, 3, 6, 7, 5, 4,
};
/** SGD pattern of the SCR1 state per rate class (7200, 9600). */
const short V29TX_PATTERN_SCR1[2] = {
    7, 15,
};
/** Carrier sine, 24 phases, Q15. */
const short V29TX_SMC_SINE[24] = {
    0, 8481, 16384, 23170, 28378, 31651, 32767, 31651, 28378, 23170, 16384, 8481,
    0, -8481, -16384, -23170, -28378, -31651, -32767, -31651, -28378, -23170, -16384, -8481,
};
/** Carrier cosine, 24 phases, Q15. */
const short V29TX_SMC_COSINE[24] = {
    32767, 31651, 28378, 23170, 16384, 8481, 0, -8481, -16384, -23170, -28378, -31651,
    -32767, -31651, -28378, -23170, -16384, -8481, 0, 8481, 16384, 23170, 28378, 31651,
};
/** In-phase amplitude of the 16 absolute constellation points. */
const short V29TX_SMC_IMAP[16] = {
    6144, 2048, 0, -2048, -6144, -2048, 0, 2048, 10240, 6144, 0, -6144,
    -10240, -6144, 0, 6144,
};
/** Quadrature amplitude of the 16 absolute constellation points. */
const short V29TX_SMC_QMAP[16] = {
    0, 2048, 6144, 2048, 0, -2048, -6144, -2048, 0, 6144, 10240, 6144,
    0, -6144, -10240, -6144,
};
/** In-phase polyphase pulse-shaping filter (120 taps). */
const short V29TX_PPS_IFILT[120] = {
    -22, -192, -455, -670, -707, -522, -190, 125, 258, 137, -162, -453, -529, -287, 207,
    738, 1051, 997, 622, 164, -75, 110, 690, 1406, 1887, 1859, 1304, 499, -123, -201,
    333, 1191, 1856, 1858, 1046, -291, -1545, -2104, -1699, -598, 487, 759, -222, -2234, -4436,
    -5760, -5483, -3691, -1359, 51, -675, -3781, -8197, -11812, -12306, -8208, 298, 11207, 21310, 27371,
    27371, 21310, 11207, 298, -8208, -12306, -11812, -8197, -3781, -675, 51, -1359, -3691, -5483, -5760,
    -4436, -2234, -222, 759, 487, -598, -1699, -2104, -1545, -291, 1046, 1858, 1856, 1191, 333,
    -201, -123, 499, 1304, 1859, 1887, 1406, 690, 110, -75, 164, 622, 997, 1051, 738,
    207, -287, -529, -453, -162, 137, 258, 125, -190, -522, -707, -670, -455, -192, -22,
};
/** Quadrature polyphase pulse-shaping filter (120 taps). */
const short V29TX_PPS_QFILT[120] = {
    99, 244, 224, 9, -326, -628, -747, -628, -345, -72, 6, -194, -603, -1017, -1206,
    -1044, -589, -65, 248, 177, -240, -756, -1033, -836, -173, 686, 1339, 1469, 1036, 318,
    -210, -141, 630, 1810, 2836, 3167, 2597, 1406, 246, -186, 451, 1901, 3389, 3989, 3135,
    990, -1546, -3237, -3164, -1305, 1283, 2823, 1630, -3000, -10235, -17804, -22775, -22725, -16800, -6193,
    6193, 16800, 22725, 22775, 17804, 10235, 3000, -1630, -2823, -1283, 1305, 3164, 3237, 1546, -990,
    -3135, -3989, -3389, -1901, -451, 186, -246, -1406, -2597, -3167, -2836, -1810, -630, 141, 210,
    -318, -1036, -1469, -1339, -686, 173, 836, 1033, 756, 240, -177, -248, 65, 589, 1044,
    1206, 1017, 603, 194, -6, 72, 345, 628, 747, 628, 326, -9, -224, -244, -99,
};
/** Pulse-shaper output gain per rate class (7200, 9600); the control record's gain word is multiplied by it. */
const int V29TX_PPS_SCALE[2] = {45016, 25480};
/** Default control record words (see FaxV29Ctl). */
const int V29TX_CTL[4] = {0x2580, 0xea60, 0x1, 0x0};
/** Default configuration block (V29Cfg::w): word 0 carries the rate 9600 (0x2580) in its high half, word 2 the timeout 60000. */
const int V29TX_CFG[7] = {0x25800000, 0x0, 0xea60, 0x1, 0x0, 0x1, 0x0};
/** Status messages indexed by the V29TxStatus code (entry 3 reads "7600 bps" for the 7200 bit/s rate class). */
const char *const V29TX_MESG[8] = {
    "V.29 Transmit Data Mode",
    "V.29 Protocal Transmit Mode",
    "CONNECT: V.29 Transmit 9600 bps",
    "CONNECT: V.29 Transmit 7600 bps",
    "V.29 Transmit Idle Mode",
    "ERROR: V.29 Tx Internal Error",
    "ERROR: Transmit Input Queue Under-run",
    "ERROR: Transmit Input Queue Over-run",
};
