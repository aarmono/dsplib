/**
 * @file voice_dtmf_tables.c
 * @brief Notch filter coefficients of the voice DTMF detector (voice_dtmf.c).
 *
 * - biascoef: the 4 coefficients of the pre-filter notch used in mode 1 (its state is Dtmf::bias);
 * - us_coef: 8 notch filters x 4 floats, used in mode 0 (the eight DTMF frequencies of the North American plan);
 * - eur_coef: the same for mode 1.
 * Each filter is the four coefficients c[0..3] of notch() in voice_tone.c.  Each float is written as a hex float, so the values are bit-exact.
 */
#include "voice_dtmf.h"

const float biascoef[4] = {
    -0x1.fe6bf80000000p+0f, 0x1.b1dbba0000000p+0f, -0x1.71eb860000000p-1f, 0x1.b333340000000p-1f,
};
const float us_coef[32] = {
    -0x1.d52e300000000p-1f, 0x1.cbcc080000000p-1f, -0x1.ebb98c0000000p-1f, 0x1.f5c2900000000p-1f,
    -0x1.69f55a0000000p-1f, 0x1.62b8200000000p-1f, -0x1.ebb98c0000000p-1f, 0x1.f5c2900000000p-1f,
    -0x1.d7d6740000000p-2f, 0x1.ce66880000000p-2f, -0x1.ebb98c0000000p-1f, 0x1.f5c2900000000p-1f,
    -0x1.7b0ff20000000p-3f, 0x1.737b080000000p-3f, -0x1.ebb98c0000000p-1f, 0x1.f5c2900000000p-1f,
    0x1.4a2b5a0000000p-1f, -0x1.4043a20000000p-1f, -0x1.e1bda60000000p-1f, 0x1.f0a3d80000000p-1f,
    0x1.01dae00000000p+0f, -0x1.f43d260000000p-1f, -0x1.e1bda60000000p-1f, 0x1.f0a3d80000000p-1f,
    0x1.5cb9ba0000000p+0f, -0x1.5243780000000p+0f, -0x1.e1bda60000000p-1f, 0x1.f0a3d80000000p-1f,
    0x1.ad40de0000000p+0f, -0x1.9e3ac00000000p+0f, -0x1.dcc9860000000p-1f, 0x1.ee147a0000000p-1f,
};
const float eur_coef[32] = {
    -0x1.d52e300000000p-1f, 0x1.c0114e0000000p-1f, -0x1.d2f4f00000000p-1f, 0x1.e8f5c20000000p-1f,
    -0x1.69f55a0000000p-1f, 0x1.57dc380000000p-1f, -0x1.ce147a0000000p-1f, 0x1.e666660000000p-1f,
    -0x1.d7d6740000000p-2f, 0x1.bde2f00000000p-2f, -0x1.c93a920000000p-1f, 0x1.e3d70a0000000p-1f,
    -0x1.7b0ff20000000p-3f, 0x1.6636b00000000p-3f, -0x1.c93a920000000p-1f, 0x1.e3d70a0000000p-1f,
    0x1.4a2b5a0000000p-1f, -0x1.38027e0000000p-1f, -0x1.c93a920000000p-1f, 0x1.e3d70a0000000p-1f,
    0x1.01dae00000000p+0f, -0x1.df9c420000000p-1f, -0x1.bad42c0000000p-1f, 0x1.dc28f60000000p-1f,
    0x1.5cb9ba0000000p+0f, -0x1.44508c0000000p+0f, -0x1.bad42c0000000p-1f, 0x1.dc28f60000000p-1f,
    0x1.ad40de0000000p+0f, -0x1.8d0f300000000p+0f, -0x1.b6147a0000000p-1f, 0x1.d9999a0000000p-1f,
};
