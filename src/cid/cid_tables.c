/**
 * @file cid_tables.c
 * @brief Coefficient tables of the caller-ID detectors (cid_detect.c, cid_modem.c).
 *
 * All sets are biquad coefficients in Q14 (five shorts per section, the layout FPM_iir_filt() reads, see fpm_iir.c) unless noted:
 *   - MTD1 .. MTD8_COEF_8000 / _9600: the eight tone filters of DTMF_MTD_detect(), one set per sampling rate; 1 .. 4 form the first group, 5 .. 8 the second;
 *   - AUTOCOR_COEF_9600 / _7200: the five-tap delay coefficients of the FSK discriminator in CID_FSD_demodulate() (a single non-zero tap: a delayed copy of
 *     the input, -0.9 at 4 samples for 9600 Hz, +1.0 at 1 sample for 7200 Hz, the rate after the polyphase filter), and fix_LPF, its 17-tap low-pass (Q15);
 *   - cid_MTD_COEF_1 / _2 _8000 / _9600: the two cascaded biquads of CID_MTD_detect() (1200 Hz seizure tone);
 *   - cid_MRF_FILT: the 90 taps (9 phases of 10) of the polyphase filter reset_cid() sets up to convert 8000 Hz input to 7200 Hz.
 */
#include "cid_tables.h"
const short MTD1_COEF_8000[5] = { -13271, 16384, 25182, -27979, 16384 };
const short MTD2_COEF_8000[5] = { -13271, 16384, 24261, -26957, 16384 };
const short MTD3_COEF_8000[5] = { -13271, 16384, 23131, -25702, 16384 };
const short MTD4_COEF_8000[5] = { -13271, 16384, 21797, -24219, 16384 };
const short MTD5_COEF_8000[5] = { -13271, 16384, 17166, -19073, 16384 };
const short MTD6_COEF_8000[5] = { -13271, 16384, 14692, -16325, 16384 };
const short MTD7_COEF_8000[5] = { -13271, 16384, 11777, -13084, 16384 };
const short MTD8_COEF_8000[5] = { -13271, 16384, 8384, -9314, 16384 };
const short MTD1_COEF_9600[5] = { -13271, 16384, 26475, -29417, 16384 };
const short MTD2_COEF_9600[5] = { -13271, 16384, 25824, -28695, 16384 };
const short MTD3_COEF_9600[5] = { -13271, 16384, 25023, -27804, 16384 };
const short MTD4_COEF_9600[5] = { -13271, 16384, 24073, -26747, 16384 };
const short MTD5_COEF_9600[5] = { -13271, 16384, 20731, -23034, 16384 };
const short MTD6_COEF_9600[5] = { -13271, 16384, 18917, -21019, 16384 };
const short MTD7_COEF_9600[5] = { -13271, 16384, 16751, -21143, 16384 };
const short MTD8_COEF_9600[5] = { -13271, 16384, 14190, -15768, 16384 };
const short AUTOCOR_COEF_9600[5] = { 0, 0, 0, 0, -29491 };
const short AUTOCOR_COEF_7200[5] = { 0, 32767, 0, 0, 0 };
const short cid_MTD_COEF_2_8000[5] = { -13271, 16384, 15409, -17121, 16384 };
const short cid_MTD_COEF_1_8000[5] = { -14786, 15564, 18296, -18296, 15564 };
const short cid_MTD_COEF_2_9600[5] = { -13271, 16384, 19445, -21605, 16384 };
const short cid_MTD_COEF_1_9600[5] = { -13271, 16384, 20853, -23170, 16384 };
const short fix_LPF[17] = { -883, -1023, -713, 159, 1547, 3226, 4842, 6010, 6436, 6010, 4842, 3226, 1547, 159, -713, -1023, -883 };

/* 90 coefficients = 9 polyphase branches x 10 taps, Q15 (see FpmMrf in fpm.h) */
const short cid_MRF_FILT[90] = {
    -633, -811, -938, -1005, -1002, -927, -781, -572, -309, -10,
    308, 622, 911, 1152, 1325, 1414, 1406, 1296, 1083, 774,
    383, -69, -556, -1047, -1510, -1908, -2208, -2379, -2394, -2231,
    -1879, -1332, -597, 311, 1368, 2542, 3792, 5074, 6340, 7540,
    8627, 9556, 10289, 10795, 11054, 11054, 10795, 10289, 9556, 8627,
    7540, 6340, 5074, 3792, 2542, 1368, 311, -597, -1332, -1879,
    -2231, -2394, -2379, -2208, -1908, -1510, -1047, -556, -69, 383,
    774, 1083, 1296, 1406, 1414, 1325, 1152, 911, 622, 308,
    -10, -309, -572, -781, -927, -1002, -1005, -938, -811, -633,
};
