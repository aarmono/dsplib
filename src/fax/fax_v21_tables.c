/**
 * @file fax_v21_tables.c
 * @brief Constant tables of the fax V.21 channel (the 300 bit/s T.30 control channel) used by fax_v21.c.
 *
 * - Filters: V21_MRF_FILT, the 360-tap matched filter shared by the receive and transmit halves (symmetric, Q15); V21RX_IIR_LPF, the receive low-pass
 *   (three biquad sections of five coefficients); V21RX_CHAN1_INTRP / V21RX_CHAN2_INTRP, the 15-tap interpolation filters of the FSK detector for the
 *   two channels.
 * - AGC: AGCv21_CFG (six config words) with its alpha / beta step tables AGC_DEF_ALPHA_v21 / AGC_DEF_BETA_v21.
 * - Tone detector: V21_CHAN1_MTD_COEFF / V21_CHAN2_MTD_COEFF, ten coefficients each (two sections).
 * - Config and control words: V21RX_CFG / V21TX_CFG (channel word 1, 300 baud, 60000 ms carrier loss time, ...) and the control defaults V21RX_CTL /
 *   V21TX_CTL.
 * - Text: V21RX_MESG / V21TX_MESG, the status strings indexed by the status codes V21RxStatus / V21TxStatus.
 *
 * The table pointers inside the config words are table handles (core/dsp_table_handles.h, CFGP()); the tables are registered at the end of the file.
 */
#include "fax_v21_tables.h"

const short V21_MRF_FILT[360] = {
    -149, -29, -25, -16, 0, 22, 49, 82, 117, 154, 191, 226, 255, 277, 291,
    293, 284, 262, 229, 186, 134, 77, 18, -40, -93, -137, -170, -189, -192, -180,
    -152, -111, -59, 0, 61, 120, 172, 213, 238, 246, 235, 205, 157, 95, 23,
    -54, -131, -200, -256, -295, -312, -306, -275, -221, -147, -58, 39, 137, 229, 306,
    363, 394, 395, 365, 304, 218, 110, -11, -136, -255, -360, -440, -490, -502, -476,
    -411, -311, -182, -33, 124, 278, 417, 528, 602, 631, 612, 544, 430, 277, 96,
    -100, -296, -477, -628, -735, -788, -782, -713, -585, -405, -186, 58, 306, 541, 743,
    895, 982, 995, 929, 787, 576, 311, 9, -306, -611, -881, -1093, -1228, -1271, -1215,
    -1060, -814, -491, -114, 290, 690, 1054, 1353, 1558, 1649, 1613, 1447, 1157, 759, 279,
    -249, -786, -1290, -1719, -2036, -2208, -2213, -2041, -1695, -1190, -557, 163, 919, 1654, 2308,
    2825, 3152, 3250, 3093, 2671, 1996, 1097, 25, -1153, -2358, -3498, -4478, -5204, -5590, -5563,
    -5069, -4075, -2575, -593, 1823, 4596, 7627, 10798, 13979, 17035, 19830, 22238, 24150, 25477, 26156,
    26156, 25477, 24150, 22238, 19830, 17035, 13979, 10798, 7627, 4596, 1823, -593, -2575, -4075, -5069,
    -5563, -5590, -5204, -4478, -3498, -2358, -1153, 25, 1097, 1996, 2671, 3093, 3250, 3152, 2825,
    2308, 1654, 919, 163, -557, -1190, -1695, -2041, -2213, -2208, -2036, -1719, -1290, -786, -249,
    279, 759, 1157, 1447, 1613, 1649, 1558, 1353, 1054, 690, 290, -114, -491, -814, -1060,
    -1215, -1271, -1228, -1093, -881, -611, -306, 9, 311, 576, 787, 929, 995, 982, 895,
    743, 541, 306, 58, -186, -405, -585, -713, -782, -788, -735, -628, -477, -296, -100,
    96, 277, 430, 544, 612, 631, 602, 528, 417, 278, 124, -33, -182, -311, -411,
    -476, -502, -490, -440, -360, -255, -136, -11, 110, 218, 304, 365, 395, 394, 363,
    306, 229, 137, 39, -58, -147, -221, -275, -306, -312, -295, -256, -200, -131, -54,
    23, 95, 157, 205, 235, 246, 238, 213, 172, 120, 61, 0, -59, -111, -152,
    -180, -192, -189, -170, -137, -93, -40, 18, 77, 134, 186, 229, 262, 284, 293,
    291, 277, 255, 226, 191, 154, 117, 82, 49, 22, 0, -16, -25, -29, -149,
};
const short V21RX_IIR_LPF[15] = {
    -12333, 445, 28281, -454, 445, -14208, 4528, 29701, -8164, 4528, -15782, 9514,
    30961, -17829, 9514,
};
const short V21RX_CHAN1_INTRP[15] = {
    0, -3, 26, -133, 501, -1604, 6016, 13751, -3008, 1146, -401, 112,
    -23, 3, 0,
};
const short V21RX_CHAN2_INTRP[15] = {
    229, -274, 341, -451, 667, -1282, 16185, 1523, -727, 478, -356, 283,
    -235, 201, -176,
};
const short AGC_DEF_ALPHA_v21[2] = {
    16384, 32604,
};
const short AGC_DEF_BETA_v21[2] = {
    16384, 164,
};
const short V21_CHAN1_MTD_COEFF[10] = {
    -13271, 16384, 21178, -23532, 16384, -13271, 16384, 17707, -19675, 16384,
};
const short V21_CHAN2_MTD_COEFF[10] = {
    -13271, 16384, 8005, -8895, 16384, -13271, 16384, 3466, -3851, 16384,
};
const int V21RX_CTL[4] = {0x12c0000, 0xea60, 0x0, 0x0};
const int V21RX_CFG[6] = {0x1, 0x12c, 0xea60, 0x0, 0x0, 0x0};
const int V21TX_CTL[5] = {0x12c, 0xea60, 0xc80, 0x0, 0x0};
const int V21TX_CFG[7] = {0x12c0001, 0x0, 0xea60, 0xc80, 0x0, 0x0, 0x0};

/** AGC config: {10000, 2}, {32, 1000}, {1, 40}, alpha table, beta table, {158, 6553} (pairs of 16-bit halves). */
const int AGCv21_CFG[6] = {0x00022710, 0x03e80020, 0x00280001, CFGP(AGC_DEF_ALPHA_v21), CFGP(AGC_DEF_BETA_v21), 0x1999009e};

const char *const V21RX_MESG[7] = {
    "V.21 Receive Data Mode",
    "V.21 Rx Start of Protocol",
    "V.21 Rx Wait for Valid Data Protocol",
    "ERROR: V.21 Rx Internal Error",
    "ERROR: Loss Carrier In V.21 Rx Protocol",
    "V.21 Rx Idle (End of Transmition)",
    "CONNECT: V.21 Receive 300 bps",
};
const char *const V21TX_MESG[6] = {
    "V.21 Transmit Data Mode",
    "V.21 Transmit Idle Mode",
    "ERROR: V.21 Tx Internal Error",
    "ERROR: Transmit Input Queue Under-run",
    "ERROR: Transmit Input Queue Over-run",
    "CONNECT: V.21 Transmit 300 bps",
};

CFGP_REGISTER(fax_v21_tables)
