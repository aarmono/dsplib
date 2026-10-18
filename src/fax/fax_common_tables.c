/**
 * @file fax_common_tables.c
 * @brief Constant tables and default configuration blocks of the fax infrastructure (FIFO, sequence / symbol generator, scramblers, symbol-mapping encoders).
 *
 * - fax_fifo_cfg: default FIFO configuration (fill value 0, capacity 100 shorts).
 * - SDM_CFG / SDMv27_CFG: default scrambler configurations (V.17 / V.29 scrambler: 4 bits per symbol, taps 5 and 23; V.27ter: 3 bits per symbol).
 * - SMC_CFG, SMCv17_CFG and the SMCv17_* tables: default symbol-mapping encoder configuration and the V.17 constellation / differential mapping tables.
 * - V27TX_SDM_NUM_BITS: bits per symbol of the V.27ter scrambler for the two transmit rates.
 * - FPM_xor_table: 8-bit population-count table, the table SGD_CFG points to.
 * - SGD_CFG: default sequence / symbol generator configuration.
 *
 * Declared in fax_common_tables.h; consumed by fax_fifo_sgd.c, fax_scrambler.c, fax_modem_wrappers.c and the V.17 / V.27ter / V.29 modem files.
 */
#include "fax_common_tables.h"

/** Default FIFO configuration: w0 = 0, capacity 100 shorts, fill value 0. */
const FaxFifoCfg fax_fifo_cfg = {0, 100, 0};
/** Default scrambler configuration: 4 bits per symbol, feedback taps at 5 and 23. */
const FaxSdmCfg SDM_CFG = {{4, 5, 23}};
/** Default V.27ter scrambler configuration: 3 bits per symbol. */
const FaxSdmCfg27 SDMv27_CFG = {3};
/** Default V.17 encoder configuration: tcmMask 0, w2 1. */
const FaxSmcV17Cfg SMCv17_CFG = {0, 1};
/** V.17 trellis encoder output table: 4-bit entries selected by the phase accumulator (SMCv17_encoder_tcm()). */
const unsigned short SMCv17_MOD[8] = {24944, 28769, 21314, 16979, 9524, 13349, 5894, 1559};
/** Absolute (non-differential) 4-point mapping of the V.17 encoder (SMCv17_encoder_abs()). */
const unsigned short SMCv17_ABS4[4] = {0, 1, 3, 2};
/** Differential phase-increment map of the 4-point V.17 encoder (SMCv17_encoder_dif()). */
const unsigned short SMCv17_PMAP4[4] = {1, 0, 2, 3};
/** In-phase coordinates of the 4 points of the V.17 constellation (Q15); the fifth entry is 0. */
const short SMCv17_IMAP4[5] = {12288, -4096, -12288, 4096, 0};
/** Quadrature coordinates of the 4 points of the V.17 constellation (Q15); the fifth entry is 0. */
const short SMCv17_QMAP4[5] = {4096, 12288, -4096, -12288, 0};
/** V.27ter transmit scrambler bits per symbol, indexed by the rate index of the transmit object (2 at 2400 bit/s, 3 at 4800 bit/s). */
const unsigned short V27TX_SDM_NUM_BITS[2] = {2, 3};
/** Default symbol-mapping encoder configuration: phaseOut = 1, flag = 1, step = 3, mod = 8, shift = 0, mask = 2, mask2 = mask3 = 0, no tables. */
const FaxSmcCfg SMC_CFG = {1, 1, 3, 8, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0};

/** Population count (number of set bits) of every byte value 0 .. 255. */
const unsigned short FPM_xor_table[256] = {
    0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4,
    1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
    1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
    1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
    3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
    4, 5, 5, 6, 5, 6, 6, 7, 5, 6, 6, 7, 6, 7, 7, 8,
};

/** Default sequence / symbol generator configuration: 8 bits per symbol, nbuf 50, w4 1, table = FPM_xor_table, len 1, mode 0, tailValue = pattern = 0,
 * patSyms 1, tab2 = FPM_xor_table, w24 = 0x20000001 (lo16 1, hi16 0x2000). */
const FaxSgdCfg SGD_CFG = {8, 50, 1, 0, FPM_xor_table, 1, 0, 0, 0, 1, FPM_xor_table, 0x20000001, 0, 0, 0};
