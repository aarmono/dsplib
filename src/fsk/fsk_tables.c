/**
 * @file fsk_tables.c
 * @brief Filter coefficients and default configuration blocks of the Bell 103 / V.21 engine (v21_b103.c) and the V.23 forward-channel receiver (v23.c).
 *
 * Data only (declarations in fsk_tables.h): the matched-filter FIR tables of the two protocols, the Bell 103 band-pass correlators, the channel
 * interpolators and IIR low-pass cascades of the FSK demodulators, the V.23 pre-filter cascade, the AGC alpha / beta pairs, and the raw configuration
 * words handed to the FPM_*_init() / _create() functions.  Table pointers inside the configuration blocks are table handles (CFGP(),
 * core/dsp_table_handles.h) registered at the end of the file.  The V.23 backward-channel tables are in v23_backchannel_tables.c.
 */
#include <stdint.h>
#include "core/dsp_table_handles.h"
#include "fsk_tables.h"

/* Bell 103 transmit matched filter: 10 polyphase branches of 27 taps (FPM_MRF cfg of B103FP_create(); the 7200 Hz modulator output is resampled by 10 / 9) */
const short B103_MRF_FILT_TX[270] = {
    -18, -24, -27, -30, -31, -30, -27, -20, -11, 1,
    15, 31, 49, 66, 83, 98, 111, 119, 123, 121,
    113, 99, 79, 52, 21, -14, -50, -87, -124, -156,
    -182, -200, -210, -209, -198, -174, -140, -96, -44, 14,
    76, 137, 196, 247, 289, 318, 331, 328, 306, 266,
    210, 139, 55, -36, -131, -225, -313, -388, -447, -485,
    -498, -486, -446, -380, -288, -176, -48, 91, 232, 369,
    495, 600, 678, 724, 733, 702, 631, 521, 376, 202,
    6, -202, -410, -609, -787, -931, -1033, -1085, -1080, -1015,
    -890, -709, -476, -201, 102, 419, 736, 1032, 1293, 1500,
    1639, 1696, 1664, 1538, 1315, 1001, 607, 145, -365, -900,
    -1433, -1937, -2380, -2735, -2974, -3070, -3003, -2758, -2323, -1698,
    -883, 108, 1256, 2537, 3920, 5365, 6836, 8288, 9679, 10964,
    12105, 13066, 13814, 14328, 14589, 14589, 14328, 13814, 13066, 12105,
    10964, 9679, 8288, 6836, 5365, 3920, 2537, 1256, 108, -883,
    -1698, -2323, -2758, -3003, -3070, -2974, -2735, -2380, -1937, -1433,
    -900, -365, 145, 607, 1001, 1315, 1538, 1664, 1696, 1639,
    1500, 1293, 1032, 736, 419, 102, -201, -476, -709, -890,
    -1015, -1080, -1085, -1033, -931, -787, -609, -410, -202, 6,
    202, 376, 521, 631, 702, 733, 724, 678, 600, 495,
    369, 232, 91, -48, -176, -288, -380, -446, -486, -498,
    -485, -447, -388, -313, -225, -131, -36, 55, 139, 210,
    266, 306, 328, 331, 318, 289, 247, 196, 137, 76,
    14, -44, -96, -140, -174, -198, -209, -210, -200, -182,
    -156, -124, -87, -50, -14, 21, 52, 79, 99, 113,
    121, 123, 119, 111, 98, 83, 66, 49, 31, 15,
    1, -11, -20, -27, -30, -31, -30, -27, -24, -18,
};

/* Bell 103 receive matched filter: 3 polyphase branches of 30 taps (resamples the received signal by 3 / 10) */
const short B103_MRF_FILT_RX[90] = {
    -948, -349, -370, -359, -307, -211, -67, 123, 355, 619,
    902, 1189, 1462, 1697, 1877, 1979, 1987, 1887, 1669, 1332,
    881, 328, -307, -997, -1709, -2407, -3053, -3607, -4034, -4302,
    -4385, -4269, -3948, -3426, -2720, -1857, -874, 186, 1272, 2331,
    3312, 4165, 4847, 5321, 5565, 5565, 5321, 4847, 4165, 3312,
    2331, 1272, 186, -874, -1857, -2720, -3426, -3948, -4269, -4385,
    -4302, -4034, -3607, -3053, -2407, -1709, -997, -307, 328, 881,
    1332, 1669, 1887, 1987, 1979, 1877, 1697, 1462, 1189, 902,
    619, 355, 123, -67, -211, -307, -359, -370, -349, -948,
};

/* Bell 103 band-pass correlator applied to the received samples in the answer role (50 taps) */
const short B103_BPF_ANSWER[50] = {
    25, 24, -64, -205, -227, 21, 425, 609, 297, -315,
    -673, -460, -15, 35, -371, -460, 508, 2045, 2461, 519,
    -2845, -4809, -3147, 1368, 5235, 5235, 1368, -3147, -4809, -2845,
    519, 2461, 2045, 508, -460, -371, 35, -15, -460, -673,
    -315, 297, 609, 425, 21, -227, -205, -64, 24, 25,
};

/* Bell 103 band-pass correlator applied to the received samples in the originate role (40 taps) */
const short B103_BPF_CALLER[40] = {
    -9, 40, -1, -300, -21, 746, 124, -1133, -263, 1037,
    223, -104, 309, -1647, -1520, 3702, 3234, -5267, -4868, 5696,
    5696, -4868, -5267, 3234, 3702, -1520, -1647, 309, -104, 223,
    1037, -263, -1133, 124, 746, -21, -300, -1, 40, -9,
};

/* Bell 103 / V.21 FSK demodulator input FIR (FPM_FSD cfg word 0): 15 taps */
const short B103_CHAN_INTRP[15] = {
    56, -104, 259, -583, 1225, -2768, 13089, 6543, -2521, 1368,
    -774, 414, -198, 83, -46,
};

/* Bell 103 / V.21 FSK demodulator IIR low-pass (FPM_FSD cfg word 2): 3 sections of 5 coefficients */
const short B103_IIR_LPF[15] = {
    -8989, 233, 23020, 162, 233, -12864, 16384, 22466, -15266, 16384,
    -15520, 16384, 22435, -19789, 16384,
};

/* V.23 forward-channel receive matched filter: 3 polyphase branches of 16 taps (FPM_MRF cfg of v23FP_rx_create()) */
const short V23_MRF_FILT[48] = {
    738, 802, 876, 630, 52, -708, -1366, -1592, -1168, -116,
    1238, 2338, 2592, 1636, -446, -3024, -5056, -5386, -3170, 1772,
    8764, 16400, 22894, 26624, 26624, 22894, 16400, 8764, 1772, -3170,
    -5386, -5056, -3024, -446, 1636, 2592, 2338, 1238, -116, -1168,
    -1592, -1366, -708, 52, 630, 876, 802, 738,
};

/* V.23 forward-channel FSK demodulator input FIR (FPM_FSD cfg word 0): 15 taps */
const short V23RX_ANSWER_INTRP[15] = {
    22, -40, 98, -213, 421, -830, 2045, 16011, -1613, 738,
    -389, 201, -93, 39, -21,
};

/* V.23 forward-channel FSK demodulator IIR low-pass (FPM_FSD cfg word 2): 3 sections of 5 coefficients */
const short V23RX_IIR_LPF[15] = {
    -3965, 510, 13254, 837, 510, -9487, 16384, 10172, 7679, 16384,
    -14380, 16384, 8308, 157, 16384,
};

/* V.23 forward-channel IIR pre-filter applied to the received block (FPM_iir_filt_II): 4 sections of 5 coefficients */
const short V23_IIR_FILT[20] = {
    2440, 2440, 4264, 9925, -1474, 4096, 4096, -7803, 10415, -11355,
    4096, 4096, -6698, 13932, 4857, 8192, 8192, 9272, 14342, -17972,
};

/* AGC alpha / beta weights of the Bell 103 engine (Q15; FPM_AGC reads only the first entry of each) */
const short B103_AGC_DEF_ALPHA[2] = { 16384, 32604 };
const short B103_AGC_DEF_BETA[2]  = { 16384, 1638 };

/* AGC alpha / beta weights of the V.23 forward-channel receiver (same values as the Bell 103 pair, kept as separate tables) */
const short V23_AGC_DEF_ALPHA[2] = { 16384, 32604 };
const short V23_AGC_DEF_BETA[2]  = { 16384, 1638 };

/* Default configuration of B103FP_create() (used when it is called with cfg == NULL): role 2 = local loop, Bell 103, cfg[2] = 0,
 * cfg[3] = 0x36b0 (wait limit 700 blocks), cfg[4] = 1, cfg[5] = 0, cfg[6] = 0xc80 (modulator amplitude). */
const int B103_CFG_raw[7] = {
    0x2, 0x0, 0x0, 0x36b0, 0x1, 0x0, 0xc80,
};

/* AGC configuration of the Bell 103 engine, given to both FPM_AGC_init() calls of B103FP_create() (layout: FpmAgc in fpm.h).  Words 3 and 4 are the
 * handles of B103_AGC_DEF_ALPHA / B103_AGC_DEF_BETA. */
const int AGCb103_CFG_raw[6] = {
    0xa4000, 0x3e80050, 0x240001, CFGP(B103_AGC_DEF_ALPHA), CFGP(B103_AGC_DEF_BETA), 0x9e,
};

/* Default FSK demodulator configuration (layout: FpmFsd in fpm.h; used when FPM_FSD_init() is called with cfg == NULL, and as the base of the
 * Bell 103 / V.21 demodulator configuration built by B103FP_create()).  No pointer words. */
const int FPM_FSD_CFG_raw[7] = {
    0x0, 0x0, 0x0, 0xa0000, 0x80001, 0xa00006, 0x0,
};

/* V.23 receive matched filter configuration (layout: FpmMrf in fpm.h): 3 branches, step 4, word 2 = 48 (16 taps per branch).  Word 1 (the
 * coefficient table) is not taken from here: v23FP_rx_create() supplies V23_MRF_FILT. */
const int MRFv23_CFG_raw[4] = {
    0x40003, 0x0, 0x30, 0x0,
};

/* V.23 forward-channel FSK demodulator configuration (layout: FpmFsd in fpm.h); words 0 to 2 and the low half of word 3 are supplied by
 * v23FP_rx_create() (filter tables and counts); words 4 to 6 (polarity, 5 samples per bit, ...) are used as they are. */
const int FSDv23_CFG_raw[7] = {
    0x0, 0x0, 0x0, 0x0, 0x50001, 0xa00018, 0x0,
};

/* V.23 tone generator / detector configuration (nine words, layout: fpm_tone.c file header), the base of the carrier detector of v23FP_rx_create()
 * and of the transmitter's generator in v23FP_tx_create().  Word 4 (the ToneLPF handle) is set by the callers. */
const int TONEv23_CFG_raw[9] = {
    0xc800672, 0x60000000, 0x30147, 0x7800, 0x0, 0x35, 0x0, 0x4000, 0x0,
};

/* AGC configuration of both AGCs of v23FP_rx_create() (layout: FpmAgc in fpm.h).  Words 3 and 4 are the handles of V23_AGC_DEF_ALPHA / V23_AGC_DEF_BETA. */
const int AGCv23_CFG_raw[6] = {
    0x201999, 0x3e80020, 0x1e0000, CFGP(V23_AGC_DEF_ALPHA), CFGP(V23_AGC_DEF_BETA), 0x0,
};

CFGP_REGISTER(b103_v23_tables)
