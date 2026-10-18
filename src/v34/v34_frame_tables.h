/**
 * @file v34_frame_tables.h
 * @brief Declarations of the frame-decoder and handshake-detector tables (defined in v34_frame_tables.c).
 *
 * gInvertPat, rxvect4, hsine1800 and offsetarr, which the same functions also read, are declared in v34_init_tables.h.
 */
#ifndef DSPLIBS_PORT_V34_TIER4_TABLES_H
#define DSPLIBS_PORT_V34_TIER4_TABLES_H

#ifdef __cplusplus
extern "C"
{
#endif

extern const short kLookup[16];
extern const short grid[529];
extern const short kTable[64];
extern const short kkNormal[16];
extern const short kkInvert[16];
extern const short c1200_[8];
extern const short c2400_[8];
extern const short bpv22high[60];
extern const short bpv22low[60];

#ifdef __cplusplus
}
#endif

#endif
