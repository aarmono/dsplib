/**
 * @file v22_tables.c
 * @brief The constant tables and configuration blocks of the V.22 / V.22bis engine.
 *
 * Equaliser and decision tables (FSEv22_*, CRRv22_*, DECv22_*), differential encoder maps (SMCv22_*), timing recovery tables (SRE_*, SREv22_*), the pulse
 * shaper, receive converter and IIR coefficient tables (PPSv22_COFFS, MRFv22_COFFS, IIR_*), the configuration blocks of the chain components (PPS / MRF /
 * SDM / SMC), and the engine's creation tables (V22_CFG, the detector coefficients and the FPM_MTD / FPM_AGC configuration words).  They are described in
 * v22_tables.h and used by v22_engine.c and the chain components.
 *
 * The tables are 16-bit words and hold no pointer except where noted.  The configuration blocks (V22PpsCfg, V22MrfCfg, V22SmcCfg) are typed structs
 * (v22.h) with real pointer members.  The word arrays MTDs1_CFG_raw / MTDv22_CFG_raw / MTDv22_CFG2_raw / AGCv22_CFG_raw / AGCv22_CFG2_raw are handed to
 * FPM_MTD_create() / FPM_AGC_init() as int words, and their first table word (V22_S1_HC_COEF, MTDv22_COEF ..., V22_AGC_DEF_ALPHA / _BETA) is a table
 * POINTER: on a 32-bit host the address itself, on x86-64 a 0 placeholder (V22TBL) that V22FP_create() replaces by the real pointer right after the
 * creator returns (neither FPM_MTD_create nor FPM_AGC_init dereferences the table); dsp_table_handles.h is not involved.
 */
#include "v22_tables.h"
/* FSEv22_CFG: the equaliser's configuration block (all zero; V22FP_create stores the two coefficient-source pointers in it) */
const short FSEv22_CFG[4] = {
    0, 0, 0, 0,
};
/* FSEv22_COFFS: equaliser coefficient prototype, 49 shorts (the creator multiplies it by a cosine and a sine to get the I and Q coefficient sources) */
const short FSEv22_COFFS[49] = {
    9, 9, -6, -23, -30, -8, 38, 75, 45, -76, -223, -255,
    -64, 280, 477, 144, -844, -2032, -2331, -445, 4405, 11801, 19980, 26405,
    28841, 26405, 19980, 11801, 4405, -445, -2331, -2032, -844, 144, 477, 280,
    -64, -255, -223, -76, 45, 75, 38, -8, -30, -23, -6, 9,
    9,
};
/* CRRv22_CLK: carrier angle offset for each position of the 6-sample symbol clock (0x8000 per turn) */
const short CRRv22_CLK[6] = {
    0, 5461, 10923, 16384, 21845, 27306,
};
/* CRRv22_PLL_K1: proportional gain of the carrier phase loop, by gain set */
const short CRRv22_PLL_K1[3] = {
    2928, 5856, 5856,
};
/* CRRv22_PLL_K2: integral gain of the carrier phase loop, by gain set */
const short CRRv22_PLL_K2[3] = {
    0, 262, 262,
};
/* DECv22_ANGL12: four-point decision (1200 bit/s): angle of each ideal point */
const short DECv22_ANGL12[4] = {
    9869, 18061, 26253, 1678,
};
/* DECv22_QMAP12: four-point decision: Q coordinate of each ideal point */
const short DECv22_QMAP12[4] = {
    12288, -4096, -12288, 4096,
};
/* DECv22_IMAP12: four-point decision: I coordinate of each ideal point */
const short DECv22_IMAP12[4] = {
    -4096, -12288, 4096, 12288,
};
/* DECv22_MAG24: sixteen-point decision (2400 bit/s): magnitude of the three rings */
const short DECv22_MAG24[3] = {
    5792, 12953, 17378,
};
/* DECv22_ANGL24: sixteen-point decision: angle of each ideal point */
const short DECv22_ANGL24[16] = {
    12288, 14706, 9869, 12288, 18061, 20480, 20480, 22898, 6514, 4096, 4096, 1677,
    28672, 26253, 31090, 28672,
};
/* DECv22_IMAP24: sixteen-point decision: I coordinate of each ideal point */
const short DECv22_IMAP24[16] = {
    -12288, -12288, -4096, -4096, -12288, -12288, -4096, -4096, 4096, 4096, 12288, 12288,
    4096, 4096, 12288, 12288,
};
/* DECv22_QMAP24: sixteen-point decision: Q coordinate of each ideal point */
const short DECv22_QMAP24[16] = {
    12288, 4096, 12288, 4096, -4096, -12288, -4096, -12288, 12288, 4096, 12288, 4096,
    -4096, -12288, -4096, -12288,
};
/* SMCv22_IMAP_1200BPS: symbol index -> I coordinate of the transmitted point at 1200 bit/s (also the decoder's reference table) */
const short SMCv22_IMAP_1200BPS[16] = {
    24576, 24576, 24576, 24576, -8192, -8192, -8192, -8192, -24576, -24576, -24576, -24576,
    8192, 8192, 8192, 8192,
};
/* SMCv22_QMAP_1200BPS: symbol index -> Q coordinate at 1200 bit/s */
const short SMCv22_QMAP_1200BPS[16] = {
    8192, 8192, 8192, 8192, 24576, 24576, 24576, 24576, -8192, -8192, -8192, -8192,
    -24576, -24576, -24576, -24576,
};
/* SMCv22_IMAP_2400BPS: symbol index -> I coordinate at 2400 bit/s */
const short SMCv22_IMAP_2400BPS[16] = {
    8192, 24576, 8192, 24576, -8192, -8192, -24576, -24576, -8192, -24576, -8192, -24576,
    8192, 8192, 24576, 24576,
};
/* SMCv22_QMAP_2400BPS: symbol index -> Q coordinate at 2400 bit/s */
const short SMCv22_QMAP_2400BPS[16] = {
    8192, 8192, 24576, 24576, 8192, 24576, 8192, 24576, -8192, -8192, -24576, -24576,
    -8192, -24576, -8192, -24576,
};
/* SMCv22_PMAP: differential quadrant table of the phase encoder and of the decision callbacks */
const short SMCv22_PMAP[4] = {
    4, 0, 8, 12,
};
/* SRE_ALPHA_AVG: timing recovery: weight of the old value in the averages, by loop mode (avg = alpha * avg + beta * x) */
const short SRE_ALPHA_AVG[3] = {
    29720, 29720, 31720,
};
/* SRE_BETA_AVG: timing recovery: weight of the new value in the averages, by loop mode */
const short SRE_BETA_AVG[3] = {
    3048, 3048, 1048,
};
/* SREv22_COFFS: timing recovery interpolator prototype, 271 shorts (V22_SRE_init reorders a copy into 10 polyphase rows of 27 taps) */
const short SREv22_COFFS[271] = {
    102, 90, 69, 39, 3, -36, -77, -115, -146, -166, -174, -167,
    -145, -108, -58, 0, 63, 126, 183, 228, 256, 265, 251, 214,
    155, 79, -8, -101, -193, -273, -336, -373, -381, -356, -299, -212,
    -102, 23, 155, 282, 392, 476, 523, 528, 489, 404, 281, 126,
    -48, -229, -401, -548, -657, -715, -716, -655, -535, -362, -149, 88,
    331, 560, 753, 893, 964, 956, 867, 699, 461, 172, -147, -472,
    -775, -1028, -1207, -1294, -1274, -1145, -911, -586, -193, 238, 674, 1077,
    1412, 1645, 1752, 1715, 1530, 1203, 753, 212, -381, -978, -1530, -1986,
    -2302, -2442, -2381, -2113, -1645, -1002, -227, 624, 1487, 2288, 2956, 3423,
    3634, 3549, 3150, 2440, 1451, 238, -1119, -2526, -3870, -5032, -5896, -6353,
    -6311, -5701, -4484, -2655, -245, 2678, 6016, 9640, 13397, 17121, 20641, 23791,
    26419, 28395, 29621, 30037, 29621, 28395, 26419, 23791, 20641, 17121, 13397, 9640,
    6016, 2678, -245, -2655, -4484, -5701, -6311, -6353, -5896, -5032, -3870, -2526,
    -1119, 238, 1451, 2440, 3150, 3549, 3634, 3423, 2956, 2288, 1487, 624,
    -227, -1002, -1645, -2113, -2381, -2442, -2302, -1986, -1530, -978, -381, 212,
    753, 1203, 1530, 1715, 1752, 1645, 1412, 1077, 674, 238, -193, -586,
    -911, -1145, -1274, -1294, -1207, -1028, -775, -472, -147, 172, 461, 699,
    867, 956, 964, 893, 753, 560, 331, 88, -149, -362, -535, -655,
    -716, -715, -657, -548, -401, -229, -48, 126, 281, 404, 489, 528,
    523, 476, 392, 282, 155, 23, -102, -212, -299, -356, -381, -373,
    -336, -273, -193, -101, -8, 79, 155, 214, 251, 265, 256, 228,
    183, 126, 63, 0, -58, -108, -145, -167, -174, -166, -146, -115,
    -77, -36, 3, 39, 69, 90, 102,
};
/* SREv22_PLL_K1: proportional gain of the timing PLL, by loop mode */
const short SREv22_PLL_K1[3] = {
    758, 762, 762,
};
/* SREv22_PLL_K2: integral gain of the timing PLL, by loop mode */
const short SREv22_PLL_K2[3] = {
    0, 4, 4,
};
/* SREv22_xCLOCK: correlation weight (x) for each position of the 6-sample symbol clock */
const short SREv22_xCLOCK[6] = {
    16384, 8192, -8192, -16384, -8192, 8192,
};
/* SREv22_yCLOCK: correlation weight (y) for each position of the 6-sample symbol clock */
const short SREv22_yCLOCK[6] = {
    0, 14189, 14189, 0, -14189, -14189,
};
/* v22_fse_mu_tab: the equaliser adaptation step, indexed by V22Fse::mu */
const short v22_fse_mu_tab[2] = {2620, 393};

/* ---- the pulse shaper, receive converter and IIR coefficient tables and the configuration blocks of the PPS / MRF / SDM / SMC ----
 * The configuration blocks are typed structs (src/v22/v22.h) with REAL pointer members. */
/* PPSv22_COFFS: pulse-shaping coefficient prototype, 120 shorts (the creator multiplies it by a cosine and a sine; V22_PPS_init reorders it into 40 phases
 * of 3 taps) */
const short PPSv22_COFFS[120] = {
    13, -2, -21, -41, -62, -84, -107, -130, -154, -176, -198, -219,
    -238, -255, -268, -279, -286, -289, -287, -280, -267, -249, -224, -193,
    -155, -111, -59, 0, 65, 138, 217, 304, 396, 495, 599, 709,
    822, 940, 1061, 1184, 1309, 1436, 1562, 1687, 1811, 1932, 2050, 2164,
    2273, 2376, 2472, 2561, 2642, 2715, 2778, 2831, 2874, 2907, 2929, 2940,
    2940, 2929, 2907, 2874, 2831, 2778, 2715, 2642, 2561, 2472, 2376, 2273,
    2164, 2050, 1932, 1811, 1687, 1562, 1436, 1309, 1184, 1061, 940, 822,
    709, 599, 495, 396, 304, 217, 138, 65, 0, -59, -111, -155,
    -193, -224, -249, -267, -280, -287, -289, -286, -279, -268, -255, -238,
    -219, -198, -176, -154, -130, -107, -84, -62, -41, -21, -2, 13,
};
/* MRFv22_COFFS: receive sample-rate converter prototype, 270 shorts (V22_MRF_init reorders it into 9 phases of 30 taps) */
const short MRFv22_COFFS[270] = {
    231, 233, 234, 234, 232, 229, 225, 219, 212, 202, 191, 179,
    164, 148, 129, 109, 87, 63, 37, 9, -21, -53, -87, -122,
    -159, -198, -238, -280, -324, -368, -414, -460, -508, -556, -605, -654,
    -704, -753, -802, -850, -898, -945, -991, -1036, -1079, -1120, -1158, -1195,
    -1229, -1259, -1287, -1311, -1332, -1348, -1360, -1368, -1371, -1369, -1361, -1349,
    -1330, -1306, -1275, -1238, -1195, -1145, -1088, -1024, -953, -875, -790, -697,
    -597, -490, -375, -253, -123, 14, 158, 310, 468, 633, 806, 984,
    1170, 1361, 1558, 1761, 1969, 2183, 2401, 2624, 2850, 3081, 3314, 3551,
    3790, 4032, 4275, 4519, 4763, 5009, 5253, 5498, 5741, 5982, 6221, 6458,
    6691, 6920, 7146, 7367, 7583, 7793, 7997, 8195, 8385, 8569, 8745, 8912,
    9071, 9221, 9362, 9494, 9615, 9726, 9827, 9918, 9997, 10066, 10123, 10169,
    10204, 10227, 10239, 10239, 10227, 10204, 10169, 10123, 10066, 9997, 9918, 9827,
    9726, 9615, 9494, 9362, 9221, 9071, 8912, 8745, 8569, 8385, 8195, 7997,
    7793, 7583, 7367, 7146, 6920, 6691, 6458, 6221, 5982, 5741, 5498, 5253,
    5009, 4763, 4519, 4275, 4032, 3790, 3551, 3314, 3081, 2850, 2624, 2401,
    2183, 1969, 1761, 1558, 1361, 1170, 984, 806, 633, 468, 310, 158,
    14, -123, -253, -375, -490, -597, -697, -790, -875, -953, -1024, -1088,
    -1145, -1195, -1238, -1275, -1306, -1330, -1349, -1361, -1369, -1371, -1368, -1360,
    -1348, -1332, -1311, -1287, -1259, -1229, -1195, -1158, -1120, -1079, -1036, -991,
    -945, -898, -850, -802, -753, -704, -654, -605, -556, -508, -460, -414,
    -368, -324, -280, -238, -198, -159, -122, -87, -53, -21, 9, 37,
    63, 87, 109, 129, 148, 164, 179, 191, 202, 212, 219, 225,
    229, 232, 234, 234, 233, 231,
};
/* IIR_a_coeff: denominator coefficients of the receive IIR section (Q14) */
const short IIR_a_coeff[4] = {
    16384, -2359, 5551, -215,
};
/* IIR_b_coeff: numerator coefficients of the receive IIR section (Q14) */
const short IIR_b_coeff[4] = {
    3064, -9191, 9191, -3064,
};
/* SDMv22_CFG: scrambler configuration { bits per symbol, tap1, tap2 }; the creator replaces the first entry by 2 (V.22) or 4 (V.22bis) */
const V22SdmCfg SDMv22_CFG = {{4, 14, 17}};
/* PPSv22_CFG: pulse shaper configuration (all zero; the creator fills in the coefficient buffer pointers) */
const V22PpsCfg PPSv22_CFG = {{0}, 0, 0, 0};
/* V22_MRF_CFG: receive converter configuration (all zero; the creator fills in the coefficient buffer pointer) */
const V22MrfCfg V22_MRF_CFG = {0, 0};
/* SMCv22_CFG: differential encoder configuration { w0 1, flag 0, step 0, mod 0x10, shift 0, mask 3, mask2 0, mask3 0xf, pmap, imap (1200), qmap (1200), 0, 0,
 * 0 } */
const V22SmcCfg SMCv22_CFG = {1, 0, 0, 0x10, 0, 3, 0, 0xf, SMCv22_PMAP, SMCv22_IMAP_1200BPS, SMCv22_QMAP_1200BPS, 0, 0, 0};

/* ---- the engine's creation tables ----
 * V22_CFG is the default engine configuration (V22EngCfg {1, 2400, 2400, 0, 120000, 13014, 0x5b, 0x06, 0, 1, 103, 0}, patched by V22FP_create's argument).
 * V22_S1_HC_COEF (20 shorts), MTDv22_COEF and MTDv22_COEF2 (15 shorts each) are the coefficient tables of the three tone detectors (S1, detectors 1 and 2).
 * The five word arrays are the FPM_MTD / FPM_AGC configuration blocks V22FP_create hands to FPM_MTD_create() / FPM_AGC_init(): MTDs1_CFG_raw
 * {V22_S1_HC_COEF, 0x4bb80004, 10}, MTDv22_CFG2_raw {MTDv22_COEF2, 0x6d4c0003, 10}, MTDv22_CFG_raw {MTDv22_COEF, 0x747c0003, 10}, and AGCv22_CFG2_raw /
 * AGCv22_CFG_raw (6 words: {0x202ee0 | 0x204268, 0x3e80001, 0x280a3d | 0x240a3d, V22_AGC_DEF_ALPHA, V22_AGC_DEF_BETA, 0x9e}).  V22_AGC_DEF_ALPHA
 * {16384, 32604} and V22_AGC_DEF_BETA {16384, 164} are the AGC's default smoothing tables.  The table words are pointers on a 32-bit host and 0 on
 * x86-64 (see the file header). */
const V22EngCfg V22_CFG = {1, 2400, 2400, 0, 120000, 13014, 0x5b, 0x06, 0, 1, 103, 0};
const short V22_S1_HC_COEF[20] = {
    -13271, 16384, 0, 0, 16384, -13271, 16384, -14746, 16384, 16384,
    -13271, 16384, -25540, 28378, 16384, -13271, 16384, 14746, -16384, 16384,
};
const short MTDv22_COEF[15] = {
    -15099, 15739, -5530, 5533, 15739, -15099, 15739, -6137, 6141, 15739,
    -15099, 15736, -19474, 19484, 15736,
};
const short MTDv22_COEF2[15] = {
    -15099, 15739, 4920, -4924, 15739, -15099, 15739, -6137, 6141, 15739,
    -15099, 15736, -19474, 19484, 15736,
};
const short V22_AGC_DEF_ALPHA[2] = {16384, 32604};
const short V22_AGC_DEF_BETA[2] = {16384, 164};
#if UINTPTR_MAX == 0xffffffffu
#define V22TBL(sym) ((int)(intptr_t)(sym))
#else
#define V22TBL(sym) 0
#endif
const int MTDs1_CFG_raw[3] = {V22TBL(V22_S1_HC_COEF), 0x4bb80004, 0xa};
const int MTDv22_CFG2_raw[3] = {V22TBL(MTDv22_COEF2), 0x6d4c0003, 0xa};
const int MTDv22_CFG_raw[3] = {V22TBL(MTDv22_COEF), 0x747c0003, 0xa};
const int AGCv22_CFG2_raw[6] = {0x202ee0, 0x3e80001, 0x280a3d, V22TBL(V22_AGC_DEF_ALPHA), V22TBL(V22_AGC_DEF_BETA), 0x9e};
const int AGCv22_CFG_raw[6] = {0x204268, 0x3e80001, 0x240a3d, V22TBL(V22_AGC_DEF_ALPHA), V22TBL(V22_AGC_DEF_BETA), 0x9e};
