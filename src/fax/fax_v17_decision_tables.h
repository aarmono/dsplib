/**
 * @file fax_v17_decision_tables.h
 * @brief Declarations of the V.17 receive decision tables (DECv17_*) and the trellis decision-function table FSEv17_decision[].
 *
 * The tables are defined in fax_v17_decision_tables.c and used by the decision callbacks of fax_v17_decision.c: constellation coordinate maps (IMAP / QMAP),
 * rotated analysis maps (ANA_*), ideal-point angles (ANGL*) and magnitudes (MAG*), and the rotation constants of the 32 / 128 point decisions.
 */
#ifndef DSPLIBS_PORT_FAX_DEC17_TABLES_H
#define DSPLIBS_PORT_FAX_DEC17_TABLES_H
#include "fax_rx_common.h"
extern const short DECv17_ANA_IMAP128[32];
extern const short DECv17_ANA_QMAP[8];
extern const short DECv17_ANA_QMAP128[32];
extern const short DECv17_ANGL12000[64];
extern const short DECv17_ANGL14400[128];
extern const short DECv17_ANGL4800[4];
extern const short DECv17_ANGL7200[16];
extern const short DECv17_ANGL9600T[32];
extern const short DECv17_COS_ROT_ANGLE[4];
extern const short DECv17_IMAP16[16];
extern const short DECv17_IMAP4[4];
extern const short DECv17_IMAP64[64];
extern const short DECv17_MAG12000[64];
extern const short DECv17_MAG14400[128];
extern const short DECv17_MAG7200[3];
extern const short DECv17_MAG9600T[32];
extern const short DECv17_MAP_BRIDGE[4];
extern const short DECv17_MAP_TRN[4];
extern const short DECv17_QMAP16[16];
extern const short DECv17_QMAP4[4];
extern const short DECv17_QMAP64[64];
extern const short DECv17_SIN_ROT_ANGLE[4];
extern const FaxFseCb FSEv17_decision[4];
#endif
