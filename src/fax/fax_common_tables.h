/**
 * @file fax_common_tables.h
 * @brief Declarations of the constant tables and default configuration blocks of the fax infrastructure (fax_common.h); defined in fax_common_tables.c.
 */
#ifndef DSPLIBS_PORT_FAX_TABLES_H
#define DSPLIBS_PORT_FAX_TABLES_H
#include "fax_common.h"
extern const FaxFifoCfg fax_fifo_cfg; ///< default FIFO configuration, used by FIFO_create() when it is given none
extern const FaxSgdCfg SGD_CFG;       ///< default sequence / symbol generator configuration
extern const FaxSdmCfg SDM_CFG;       ///< default scrambler configuration (4 bits per symbol, taps 5 and 23)
extern const FaxSmcCfg SMC_CFG;       ///< default symbol-mapping encoder configuration
extern const FaxSmcV17Cfg SMCv17_CFG; ///< default V.17 encoder configuration
extern const FaxSdmCfg27 SDMv27_CFG;  ///< default V.27ter scrambler configuration
extern const unsigned short SMCv17_MOD[8];
extern const unsigned short SMCv17_ABS4[4];
extern const unsigned short SMCv17_PMAP4[4];
extern const short SMCv17_IMAP4[5];
extern const short SMCv17_QMAP4[5];
extern const unsigned short V27TX_SDM_NUM_BITS[2];
extern const unsigned short FPM_xor_table[256];
#endif
