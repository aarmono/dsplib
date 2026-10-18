/**
 * @file v32_tables.c
 * @brief V.32 / V.32bis default configuration blocks, filter coefficients and the small rate, timing and mode lookup tables.
 *
 * The configuration blocks (`*_CFG_raw`) are the word arrays handed to the FPM_*_init() functions by V32FP_recreate() (v32_engine.c): FPM_PPS_init (pulse
 * shaper), FPM_MRF_init (matched receive filter), FPM_ECC_init (echo canceller), FPM_AGC_init (two AGCs), FPM_SRE_init (symbol timing), FPM_FSE_init
 * (equaliser).  A word that refers to a table is a CFGP handle (core/dsp_table_handles.h: a 32-bit handle on x86-64, the address
 * on a 32-bit host); the creator replaces some words (those marked below) before the block is used.  The coefficient tables are the filter taps those
 * blocks point to.  The small tables are indexed by the rate class (V32RateClass, 0 .. 6), by the role (V32Role) or by a 0 / 1 selector, as noted at each
 * table, and are read by the handshake state machine and by V32FP_status().
 *
 * The constellation maps, the Viterbi decoder tables and the decision angle / magnitude tables are in v32_viterbi_tables.c.
 *
 * Two tables are declared one entry longer than their natural size because the code reads that entry: CRRv32_CLK (5 entries) and FPM_log10_table (129).
 * ECCv32_IMAP / ECCv32_QMAP are not coefficient data but 6-slot arrays of pointers to the 16-point and trellis constellation maps of v32_viterbi_tables.c.
 * V32_AGC_DEF_ALPHA / BETA are V.32's own AGC step-size pair; its second entries (30491 and 2277) differ from the pair used by the V.21 / V.23 modems
 * (32604 and 1638).
 */
#include <stdint.h>
#include "core/dsp_table_handles.h"
#include "v32_tables.h"
#include "v32_viterbi_tables.h" /* for SMCv32_IMAP16 / SMCv32_QMAP16, referenced by PPSv32_CFG_raw below */

/* ===================== Default configuration blocks ===================== */

/**
 * @brief Default contents of the first 48 bytes of the V32Wrap object (the configuration V32FP_create() patches): protoSel 0, txRate and rxRate 14400,
 * timeoutWord 120000, the option bytes at +0x10, discThresh 0x67 (overwritten by V32FP_recreate() with V32DiconnectThreshTable[3]), the other fields 0.
 * Words not named here are 0 or not individually identified.
 */
const int V32_CFG_raw[12] = {
    0x38400000,
    0x3840,
    0x1d4c0,
    0x45df,
    0x68b,
    0x0,
    0x0,
    0x0,
    0x0,
    0x0,
    0x67,
    0x0,
};

/** @brief Configuration of the receive AGC embedded in the engine (FPM_AGC_init); words 3 and 4 point to V32_AGC_DEF_ALPHA / V32_AGC_DEF_BETA. */
const int AGCv32_CFG_raw[6] = {
    0x22365,
    0x3e80050,
    0x240001,
    CFGP(V32_AGC_DEF_ALPHA),
    CFGP(V32_AGC_DEF_BETA),
    0x1999009e,
};

/** @brief Configuration of the second AGC, at the start of the handshake object (FPM_AGC_init); words 3 and 4 point to the same V32_AGC_DEF pair. */
const int AGCv32Prc_CFG_raw[6] = {
    0x22710,
    0x3e80050,
    0x280001,
    CFGP(V32_AGC_DEF_ALPHA),
    CFGP(V32_AGC_DEF_BETA),
    0x1999009e,
};

/**
 * @brief Configuration of the matched receive filter (FPM_MRF_init): word 1 points to MRFv32_COFFS, word 2 is the coefficient count (360); the creator
 * replaces the last word.
 */
const int MRFv32_CFG_raw[4] = {
    0xa0009,
    CFGP(MRFv32_COFFS),
    0x168,
    0x0,
};

/** @brief Configuration of the echo canceller (FPM_ECC_init): words 2 and 3 point to ECCv32_IMAP / ECCv32_QMAP; the creator replaces the last word. */
const int ECCv32_CFG_raw[6] = {
    0x2801e0,
    0x28,
    CFGP(ECCv32_IMAP),
    CFGP(ECCv32_QMAP),
    0x10,
    0x0,
};

/**
 * @brief Configuration of the symbol timing estimator (FPM_SRE_init), 14 words: words 4 .. 9 point to SREv32_COFFS, SREv32_XB_COFFS, SREv32_xCLOCK,
 * SREv32_yCLOCK, SREv32_PLL_K1 and SREv32_PLL_K2.
 */
const int SREv32_CFG_raw[14] = {
    0x30003,
    0x460010,
    0x40002000,
    0xb4,
    CFGP(SREv32_COFFS),
    CFGP(SREv32_XB_COFFS),
    CFGP(SREv32_xCLOCK),
    CFGP(SREv32_yCLOCK),
    CFGP(SREv32_PLL_K1),
    CFGP(SREv32_PLL_K2),
    0x10002,
    0xc82666,
    0x905dc,
    0x0,
};

/**
 * @brief Configuration of the equaliser and carrier loop (FPM_FSE_init), 14 words: words 1 and 2 point to FSEv32_ICOFF / FSEv32_QCOFF, word 5 to CRRv32_CLK,
 * words 9 and 10 to CRRv32_PLL_K1 / CRRv32_PLL_K2.  The creator overwrites words 11 .. 13 (history object, initial decision callback, creator scalar) before
 * the call, so the zeros stored here are never read.
 */
const int FSEv32_CFG_raw[14] = {
    0x30090,
    CFGP(FSEv32_ICOFF),
    CFGP(FSEv32_QCOFF),
    0xcab0067,
    0x361,
    CFGP(CRRv32_CLK),
    0x10004,
    0x198805dc,
    0x666,
    CFGP(CRRv32_PLL_K1),
    CFGP(CRRv32_PLL_K2),
    0x0,
    0x0,
    0x0,
};

/**
 * @brief Configuration of the pulse shaper (FPM_PPS_init): words 4 and 5 point to SMCv32_IMAP16 / SMCv32_QMAP16, words 6 and 7 to PPSv32_ICOFFS /
 * PPSv32_QCOFFS.
 */
const int PPSv32_CFG_raw[10] = {
    0x3000a,
    0x1,
    0x20000,
    0x0,
    CFGP(SMCv32_IMAP16),
    CFGP(SMCv32_QMAP16),
    CFGP(PPSv32_ICOFFS),
    CFGP(PPSv32_QCOFFS),
    0x78,
    0x0,
};

/** @brief Scrambler configuration, three shorts; entry 2 (23) is the register length that V32FP_recreate() uses (as a literal) for both scramblers. */
const short SDMv32_CFG[3] = {
    4, 5, 23,
};

/* ===================== Coefficient and filter tables ===================== */

/**
 * @brief Coefficient set of the tone detector (FPM_MTD) for the connected-data phase.  The handshake code recognises that phase by comparing the detector's
 * current coefficient pointer with this table's address.
 */
const short V32_S_DATA_COEF[15] = {
    -15099, 15735, 27242, -27254, 15735, -15099, 15741, 0, 0, 15741,
    -15099, 15735, -27242, 27254, 15735,
};

/** @brief A second 15-short detector coefficient set, very similar to V32_S_DATA_COEF; not referenced by this module. */
const short V32_S_COEF[15] = {
    -15099, 15735, 28028, -28040, 15735, -15099, 15739, 4920, -4924, 15739,
    -15099, 15736, -22243, 22254, 15736,
};

/** @brief Matched receive filter FIR coefficients (360 taps). */
const short MRFv32_COFFS[360] = {
    84, 85, 83, 76, 66, 54, 40, 26, 13, 4,
    0, 0, 7, 21, 41, 67, 96, 127, 157, 185,
    207, 221, 227, 223, 210, 187, 157, 121, 84, 48,
    17, -4, -16, -15, 0, 27, 68, 118, 175, 233,
    287, 334, 368, 386, 386, 366, 326, 270, 200, 123,
    43, -30, -94, -141, -166, -167, -142, -93, -23, 62,
    156, 251, 337, 407, 452, 467, 448, 394, 308, 194,
    59, -87, -233, -369, -483, -565, -610, -611, -569, -487,
    -371, -231, -80, 68, 200, 300, 358, 364, 313, 206,
    46, -157, -391, -640, -884, -1107, -1289, -1417, -1480, -1472,
    -1393, -1248, -1052, -819, -571, -332, -124, 29, 110, 107,
    11, -175, -445, -783, -1165, -1565, -1953, -2299, -2575, -2758,
    -2831, -2785, -2623, -2355, -2002, -1593, -1163, -751, -395, -132,
    6, 1, -159, -473, -927, -1491, -2129, -2794, -3435, -3999,
    -4438, -4710, -4784, -4645, -4294, -3749, -3046, -2236, -1382, -556,
    169, 722, 1039, 1074, 798, 203, -691, -1840, -3175, -4607,
    -6029, -7325, -8376, -9068, -9300, -8992, -8089, -6571, -4451, -1779,
    1360, 4853, 8555, 12309, 15944, 19291, 22189, 24499, 26105, 26929,
    26929, 26105, 24499, 22189, 19291, 15944, 12309, 8555, 4853, 1360,
    -1779, -4451, -6571, -8089, -8992, -9300, -9068, -8376, -7325, -6029,
    -4607, -3175, -1840, -691, 203, 798, 1074, 1039, 722, 169,
    -556, -1382, -2236, -3046, -3749, -4294, -4645, -4784, -4710, -4438,
    -3999, -3435, -2794, -2129, -1491, -927, -473, -159, 1, 6,
    -132, -395, -751, -1163, -1593, -2002, -2355, -2623, -2785, -2831,
    -2758, -2575, -2299, -1953, -1565, -1165, -783, -445, -175, 11,
    107, 110, 29, -124, -332, -571, -819, -1052, -1248, -1393,
    -1472, -1480, -1417, -1289, -1107, -884, -640, -391, -157, 46,
    206, 313, 364, 358, 300, 200, 68, -80, -231, -371,
    -487, -569, -611, -610, -565, -483, -369, -233, -87, 59,
    194, 308, 394, 448, 467, 452, 407, 337, 251, 156,
    62, -23, -93, -142, -167, -166, -141, -94, -30, 43,
    123, 200, 270, 326, 366, 386, 386, 368, 334, 287,
    233, 175, 118, 68, 27, 0, -15, -16, -4, 17,
    48, 84, 121, 157, 187, 210, 223, 227, 221, 207,
    185, 157, 127, 96, 67, 41, 21, 7, 0, 0,
    4, 13, 26, 40, 54, 66, 76, 8, 85, 84,
};

/** @brief Symbol timing estimator FIR coefficients (181 taps). */
const short SREv32_COFFS[181] = {
    45, 62, 75, 82, 81, 72, 52, 24, -10, -50,
    -89, -126, -153, -169, -169, -152, -116, -64, 0, 72,
    146, 213, 266, 298, 303, 278, 223, 139, 33, -86,
    -210, -325, -419, -480, -499, -470, -392, -267, -105, 82,
    279, 467, 625, 735, 783, 758, 656, 480, 241, -42,
    -347, -645, -907, -1102, -1207, -1202, -1079, -838, -492, -65,
    408, 888, 1327, 1679, 1902, 1959, 1829, 1503, 992, 324,
    -456, -1288, -2099, -2808, -3335, -3604, -3551, -3128, -2310, -1096,
    487, 2388, 4528, 6812, 9126, 11352, 13371, 15074, 16364, 17168,
    17442, 17168, 16364, 15074, 13371, 11352, 9126, 6812, 4528, 2388,
    487, -1096, -2310, -3128, -3551, -3604, -3335, -2808, -2099, -1288,
    -456, 324, 992, 1503, 1829, 1959, 1902, 1679, 1327, 888,
    408, -65, -492, -838, -1079, -1202, -1207, -1102, -907, -645,
    -347, -42, 241, 480, 656, 758, 783, 735, 625, 467,
    279, 82, -105, -267, -392, -470, -499, -480, -419, -325,
    -210, -86, 33, 139, 223, 278, 303, 298, 266, 213,
    146, 72, 0, -64, -116, -152, -169, -169, -153, -126,
    -89, -50, -10, 24, 52, 72, 81, 82, 75, 62,
    45,
};

/** @brief Pulse shaper Q-channel FIR coefficients (120 taps). */
const short PPSv32_QCOFFS[120] = {
    0, -1, -2, -2, -1, 0, 0, -2, -6, -12,
    -14, -12, -5, 0, 1, -7, -20, -31, -29, -12,
    12, 30, 28, 7, -19, -29, -6, 44, 100, 129,
    112, 59, 8, 3, 65, 169, 258, 273, 193, 60,
    -41, -36, 88, 263, 366, 297, 51, -259, -461, -421,
    -151, 168, 260, -94, -911, -1942, -2754, -2923, -2240, -840,
    840, 2240, 2923, 2754, 1942, 911, 94, -260, -168, 151,
    421, 461, 259, -51, -297, -366, -263, -88, 36, 41,
    -60, -193, -273, -258, -169, -65, -3, -8, -59, -112,
    -129, -100, -44, 6, 29, 19, -7, -28, -30, -12,
    12, 29, 31, 20, 7, -1, 0, 5, 12, 14,
    12, 6, 2, 0, 0, 1, 2, 2, 1, 0,
};

/** @brief Pulse shaper I-channel FIR coefficients (120 taps). */
const short PPSv32_ICOFFS[120] = {
    -1, -1, 0, 0, 0, 0, -2, -5, -5, -2,
    3, 10, 13, 9, 1, -4, -1, 12, 34, 50,
    51, 35, 11, 0, 11, 47, 88, 107, 86, 31,
    -27, -50, -20, 48, 106, 104, 20, -113, -226, -251,
    -172, -42, 36, -20, -224, -485, -657, -626, -393, -101,
    36, -143, -629, -1195, -1486, -1190, -216, 1210, 2623, 3499,
    3499, 2623, 1210, -216, -1190, -1486, -1195, -629, -143, 36,
    -101, -393, -626, -657, -485, -224, -20, 36, -42, -172,
    -251, -226, -113, 20, 104, 106, 48, -20, -50, -27,
    31, 86, 107, 88, 47, 11, 0, 11, 35, 51,
    50, 34, 12, -1, -4, 1, 9, 13, 10, 3,
    -2, -5, -5, -2, 0, 0, 0, 0, -1, -1,
};

/** @brief Symbol timing estimator XB coefficients (11 taps). */
const short SREv32_XB_COFFS[11] = {
    -28156, 16128, 28156, 16128, 14078, 8128, -14078, 8128, 992, -15360,
    14399,
};

/**
 * @brief Echo canceller I maps: 6 pointers to constellation maps, one per rate class.  FPM_ECC_cancel() uses the transmitted code to select a sub-table
 * (value >> 8) and an entry in it, on every third sample.  Pointer arrays, not coefficient data: declared as `const short *const` in v32_tables.h.
 */
const short *const ECCv32_IMAP[6] = {
    SMCv32_IMAP16, SMCv32_IMAP16, VTBv32_IMAP32, VTBv32_IMAP16T, VTBv32_IMAP64, VTBv32_IMAP128,
};

/** @brief Echo canceller Q maps: the Q counterparts of ECCv32_IMAP, in the same order. */
const short *const ECCv32_QMAP[6] = {
    SMCv32_QMAP16, SMCv32_QMAP16, VTBv32_QMAP32, VTBv32_QMAP16T, VTBv32_QMAP64, VTBv32_QMAP128,
};

/** @brief Equaliser I coefficients (103 taps). */
const short FSEv32_ICOFF[103] = {
    -5, 0, -2, 0, 2, 0, 6, 0, 10, 0,
    10, 0, 9, 0, 3, 0, -4, 0, -11, 0,
    -19, 0, -20, 0, -18, 0, -7, 0, 9, 0,
    28, 0, 51, 0, 57, 0, 61, 0, 26, 0,
    -43, 0, -142, 0, -415, 0, -582, 0, -2129, 0,
    -16043, 0, 16043, 0, 2129, 0, 582, 0, 415, 0,
    142, 0, 43, 0, -26, 0, -61, 0, -57, 0,
    -51, 0, -28, 0, -9, 0, 7, 0, 18, 0,
    20, 0, 19, 0, 11, 0, 4, 0, -3, 0,
    -9, 0, -10, 0, -10, 0, -6, 0, -2, 0,
    2, 0, 5,
};

/** @brief Equaliser Q coefficients (103 taps). */
const short FSEv32_QCOFF[103] = {
    0, 6, 0, 8, 0, 7, 0, 4, 0, 0,
    0, -6, 0, -11, 0, -14, 0, -14, 0, -8,
    0, -1, 0, 12, 0, 23, 0, 31, 0, 33,
    0, 21, 0, 3, 0, -39, 0, -77, 0, -126,
    0, -167, 0, -112, 0, -86, 0, 863, 0, 3710,
    0, -22796, 0, 3710, 0, 863, 0, -86, 0, -112,
    0, -167, 0, -126, 0, -77, 0, -39, 0, 3,
    0, 21, 0, 33, 0, 31, 0, 23, 0, 12,
    0, -1, 0, -8, 0, -14, 0, -14, 0, -11,
    0, -6, 0, 0, 0, 4, 0, 7, 0, 8,
    0, 6, 0,
};

/* ===================== Rate-negotiation tables, indexed 0 .. 6 by rate class (V32RateClass) ===================== */

/**
 * @brief Rate-sequence code word per rate class (used by the originate, answer and renegotiation state machines and CodeRateSeq .. DecodeRateSeq).  Also
 * indexed by the engine's own rate class to give the locally configured capability word.
 */
const short V32_RATE_SEQ[7] = {
    3345, 2833, 2961, 2513, 2481, 4089, 2455,
};

/** @brief E-sequence code word per rate class. */
const short V32_ESEQ[7] = {
    -751, -1263, -1135, -1583, -1615, -1639, -1641,
};

/** @brief Final negotiated rate-sequence code word per rate class. */
const short V32_FINAL_RATE_SEQ[7] = {
    3345, 2833, 2961, 2513, 2481, 2457, 2455,
};

/** @brief Disconnect threshold table (8 entries); only entry 3 is used (stored into V32Wrap::discThresh).  The identifier is spelled "Diconnect". */
const short V32DiconnectThreshTable[8] = {
    75, 95, 119, 150, 168, 174, 212, 238,
};

/* ===================== Timing, status and mode tables. ===================== */

/** @brief Turn-around delay (samples) for the 0 / 1 rate selector V32Wrap::w018 (not a rate class). */
const short V32_TURNAROUND_DLY[2] = {
    64, 360,
};

/** @brief Symbol length in samples, indexed by the same 0 / 1 selector as V32_TURNAROUND_DLY. */
const short V32_SYMBOL_LEN[2] = {
    12, 48,
};

/** @brief Samples per block, indexed by the same 0 / 1 selector. */
const short V32_SAMPLE_LEN[2] = {
    40, 160,
};

/**
 * @brief Template of the 16-short control block that the retrain request paths hand to V32FP_control(); the callers patch the word at +12 (shorts 6 and 7);
 * the renegotiation path also zeroes the last 8 bytes.
 */
const short V32_CTL[16] = {
    9600, 9600, -11072, 1, 1, 0, 387, 0, 0, 0,
    0, 0, 1, 0, 1, 0,
};

/** @brief Connect status code per rate class (indexed 0 .. 6 like the rate-negotiation tables); entry 6 (no common rate) is the no-carrier status 14. */
const short V32_CONNECT[7] = {
    4, 3, 25, 24, 26, 27, 14,
};

/** @brief Receive mode per rate class: rate class + 1, i.e. {1 .. 7}. */
const short V32_RX_MODE[7] = {
    1, 2, 3, 4, 5, 6, 7,
};

/** @brief Transmit mode per rate class: the same values as V32_RX_MODE, a separate table. */
const short V32_TX_MODE[7] = {
    1, 2, 3, 4, 5, 6, 7,
};

/* ===================== Carrier recovery, symbol timing and rate constants. ===================== */

/**
 * @brief Carrier phase table of the equaliser (0, 0x2000, 0x4000, 0x6000).  A fifth entry (0) is stored because FPM_FSE_receive() reads index 4 when the
 * phase accumulator lands exactly on the modulus; reading it is in bounds and yields 0.
 */
const short CRRv32_CLK[5] = {
    0, 8192, 16384, 24576, 0,
};

/** @brief Carrier loop proportional gains (3 entries). */
const short CRRv32_PLL_K1[3] = {
    602, 3050, 766,
};

/** @brief Carrier loop integral gains (3 entries). */
const short CRRv32_PLL_K2[3] = {
    0, 18, 1,
};

/** @brief Symbol timing loop proportional gains (3 entries). */
const short SREv32_PLL_K1[3] = {
    2336, 3049, 1524,
};

/** @brief Symbol timing loop integral gains (3 entries). */
const short SREv32_PLL_K2[3] = {
    0, 17, 8,
};

/** @brief Cosines (Q14) of the three symbol-clock phases 0, 120 and 240 degrees: the correlator table V32Sre::corrCosTab. */
const short SREv32_xCLOCK[3] = {
    16384, -8192, -8192,
};

/** @brief Sines (Q14) of the same three phases: the correlator table V32Sre::corrSinTab. */
const short SREv32_yCLOCK[3] = {
    0, 14189, -14189,
};

/** @brief Line rate in bit/s per rate class 0 .. 5 (4800, 9600 plain, 9600 trellis, 7200, 12000, 14400); read by V32FP_status(). */
const short RATEv32[6] = {
    4800, 9600, 9600, 7200, 12000, 14400,
};

/** @brief Six zero shorts; not referenced by this module. */
const short FSEv32_decision[6] = {
    0, 0, 0, 0, 0, 0,
};

/* ===================== V.32's own AGC default step-size pair. ===================== */

/** @brief AGC default alpha pair {16384, 30491} (Q15; the second entry differs from the V.21 / V.23 pair). */
const short V32_AGC_DEF_ALPHA[2] = {
    16384, 30491,
};

/** @brief AGC default beta pair {16384, 2277} (Q15). */
const short V32_AGC_DEF_BETA[2] = {
    16384, 2277,
};

/* ===================== Tables of the trellis-coded encoder, SMCv32_encoder_tcm() (v32_scrambler.c). ===================== */

/** @brief Trellis transition values, indexed `trellisState + 4 * previous transition value`. */
const short TrellisTransitionTable[32] = {
    0, 6, 2, 4, 2, 4, 0, 6, 4, 2, 6, 0, 6, 0, 4, 2,
    1, 5, 7, 3, 3, 7, 5, 1, 7, 3, 1, 5, 5, 1, 3, 7,
};

/** @brief Next trellis state, indexed `trellisState + 4 * (raw bits >> payloadBits)`. */
const short TrellisEncodeDifTable[16] = {
    0, 1, 2, 3, 1, 0, 3, 2, 2, 3, 1, 0, 3, 2, 0, 1,
};

/** @brief Rotated output point words of the trellis encoder, indexed by the modulation index. */
const short SMCv32_MOD[8] = {
    24944, 28769, 21314, 16979, 9524, 13349, 5894, 1559,
};

/**
 * @brief Arctangent lookup table of FPM_atan() (v32_math.c), indexed by the high bits (`>> 7`) of the normalised ratio; 257 entries, so that the
 * interpolation can read the entry after the last.
 */
const short FPM_atan_table[257] = {
    0, 128, 256, 384, 512, 640, 768, 896, 1024, 1152, 1279, 1407, 1535, 1663, 1790, 1918,
    2045, 2173, 2300, 2428, 2555, 2682, 2809, 2936, 3063, 3190, 3317, 3443, 3570, 3696, 3823, 3949,
    4075, 4201, 4327, 4452, 4578, 4703, 4829, 4954, 5079, 5204, 5329, 5453, 5578, 5702, 5826, 5950,
    6073, 6197, 6320, 6444, 6567, 6689, 6812, 6935, 7057, 7179, 7301, 7422, 7544, 7665, 7786, 7907,
    8027, 8148, 8268, 8388, 8508, 8627, 8746, 8865, 8984, 9102, 9221, 9339, 9456, 9574, 9691, 9808,
    9925, 10041, 10158, 10274, 10389, 10505, 10620, 10735, 10849, 10964, 11078, 11192, 11305, 11418, 11531, 11644,
    11756, 11868, 11980, 12092, 12203, 12314, 12424, 12535, 12645, 12754, 12864, 12973, 13082, 13190, 13298, 13406,
    13514, 13621, 13728, 13835, 13941, 14047, 14153, 14258, 14363, 14468, 14573, 14677, 14781, 14884, 14987, 15090,
    15193, 15295, 15397, 15499, 15600, 15701, 15801, 15902, 16002, 16101, 16201, 16300, 16398, 16497, 16595, 16693,
    16790, 16887, 16984, 17080, 17176, 17272, 17368, 17463, 17557, 17652, 17746, 17840, 17933, 18027, 18119, 18212,
    18304, 18396, 18488, 18579, 18670, 18760, 18851, 18941, 19030, 19120, 19209, 19297, 19386, 19474, 19561, 19649,
    19736, 19823, 19909, 19995, 20081, 20166, 20252, 20336, 20421, 20505, 20589, 20673, 20756, 20839, 20922, 21004,
    21086, 21168, 21249, 21331, 21411, 21492, 21572, 21652, 21732, 21811, 21890, 21969, 22047, 22126, 22203, 22281,
    22358, 22435, 22512, 22588, 22664, 22740, 22815, 22891, 22966, 23040, 23115, 23189, 23262, 23336, 23409, 23482,
    23555, 23627, 23699, 23771, 23842, 23914, 23985, 24055, 24126, 24196, 24266, 24335, 24405, 24474, 24542, 24611,
    24679, 24747, 24815, 24882, 24950, 25017, 25083, 25150, 25216, 25282, 25347, 25413, 25478, 25543, 25607, 25672,
    25736,
};

/* ===================== Status and recovery tables of V32FP_status() (v32_engine.c). ===================== */

/** @brief Status word reported for each role (V32Role, 0 .. 8): the first word of the V32FP_status() output. */
const short PROTOCOL[9] = {
    0, 1, 2, 9, 6, 6, 3, 7, 8,
};

/** @brief SNR thresholds, indexed by min(connected rate class, 5): below it V32FP_status() counts an SNR drop towards a local retrain. */
const short SnrToRetrainTable[6] = {
    9, 13, 13, 11, 20, 24,
};

/**
 * @brief Descrambler feedback tap position by role (V32Role 0 .. 3): 5 for the originate role, 18 otherwise (V.32 GPA polynomial 1 + x^-5 + x^-23 for the
 * caller).
 */
const short SDMv32_GPA[4] = {
    5, 18, 18, 18,
};

/**
 * @brief Scrambler feedback tap position by role: 18 for the originate role, 5 for the answer role (V.32 GPC polynomial 1 + x^-18 + x^-23 for the caller).
 */
const short SDMv32_GPC[4] = {
    18, 5, 18, 18,
};

/**
 * @brief Logarithm lookup table of FPM_log10() (v32_math.c), indexed `(((mantissa + 0x40) >> 7) - 0x80) & 0xffff`.
 * @note The index reaches 128 for a normalised mantissa >= 0x7fc0, so the table has 129 entries; entry 128 (10) is the value the result must have
 *        there.  A live V.32 session (V32FP_status() SNR estimate) reaches it.
 */
const short FPM_log10_table[129] = {
    -9864, -9753, -9643, -9534, -9426, -9318, -9212, -9106, -9001, -8897, -8793, -8690, -8588, -8487, -8387, -8287,
    -8187, -8089, -7991, -7894, -7798, -7702, -7607, -7512, -7418, -7325, -7232, -7140, -7048, -6957, -6867, -6777,
    -6688, -6599, -6511, -6424, -6337, -6250, -6164, -6079, -5994, -5909, -5825, -5742, -5659, -5576, -5494, -5413,
    -5332, -5251, -5171, -5091, -5012, -4933, -4855, -4777, -4699, -4622, -4545, -4469, -4393, -4318, -4243, -4168,
    -4093, -4020, -3946, -3873, -3800, -3728, -3656, -3584, -3513, -3442, -3371, -3301, -3231, -3161, -3092, -3023,
    -2954, -2886, -2818, -2751, -2683, -2616, -2550, -2483, -2417, -2352, -2286, -2221, -2156, -2092, -2027, -1963,
    -1900, -1836, -1773, -1710, -1648, -1586, -1524, -1462, -1400, -1339, -1278, -1218, -1157, -1097, -1037, -977,
    -918, -859, -800, -741, -683, -625, -567, -509, -451, -394, -337, -280, -224, -167, -111, -55,
    10, /* [128]: reached by a mantissa >= 0x7fc0, see above */
};

CFGP_REGISTER(v32_tables)
