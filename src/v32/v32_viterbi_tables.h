/**
 * @file v32_viterbi_tables.h
 * @brief Declarations of the V.32 / V.32bis constellation maps, Viterbi decoder tables and decision angle / magnitude tables defined in v32_viterbi_tables.c.
 */
#ifndef DSPLIBS_PORT_V32_TABLES_VTB_H
#define DSPLIBS_PORT_V32_TABLES_VTB_H

/* DEC constellation maps (equaliser decision callbacks) */
extern const short DECv32_QMAP4[4];
extern const short DECv32_IMAP4[4];
extern const short DECv32_QMAP16[16];
extern const short DECv32_IMAP16[16];
extern const short DECv32_QMAP64[64];
extern const short DECv32_IMAP64[64];
extern const short DECv32_ANA_QMAP128[32];
extern const short DECv32_ANA_IMAP128[32];
extern const short DECv32_ANA_QMAP[8];
extern const short DECv32_MAP_TRN[4];

/* VTB trellis decoder constellation maps */
extern const short VTBv32_QMAP16T[16];
extern const short VTBv32_IMAP16T[16];
extern const short VTBv32_QMAP32[32];
extern const short VTBv32_IMAP32[32];
extern const short VTBv32_QMAP64[64];
extern const short VTBv32_IMAP64[64];
extern const short VTBv32_QMAP128[128];
extern const short VTBv32_IMAP128[128];

/* VTB trellis decoder: candidate point indices (VTB_BOUND_*) and region tables (VTB_REGION_*) of each rate family */
extern const short VTB_BOUND_7200[128];
extern const short VTB_BOUND_9600[416];
extern const short VTB_BOUND_12000[960];
extern const short VTB_BOUND_14400[1856];
extern const short VTB_REGION_7200[8];
extern const short VTB_REGION_9600[32];
extern const short VTB_REGION_12000[72];
extern const short VTB_REGION_14400[128];

/* VTB_decoder()'s differential (quadrant) decode table */
extern const short VTB_DIFF_TBL[16];

/* SMC encoder constellation and phase maps */
extern const short SMCv32_QMAP16[17];
extern const short SMCv32_IMAP16[17];
extern const short SMCv32_PMAP_ABS16[4];
extern const short SMCv32_PMAP16[4];

/* DEC angle tables */
extern const short DECv32_ANGL1200[4];
extern const short DECv32_ANGL9600[16];
extern const short DECv32_ANGL9600T[32];
extern const short DECv32_ANGL12000[64];
extern const short DECv32_ANGL14400[128];

/* DEC magnitude tables */
extern const short DECv32_MAG9600[3];
extern const short DECv32_MAG9600T[32];
extern const short DECv32_MAG12000[64];
extern const short DECv32_MAG14400[128];

/* DEC rotation constants (+-0.7071 in Q15) */
extern const short DECv32_SIN_ROT_ANGLE[4];
extern const short DECv32_COS_ROT_ANGLE[4];

#endif
