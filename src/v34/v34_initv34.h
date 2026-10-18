/**
 * @file v34_initv34.h
 * @brief Declaration of initV34() (src/v34/v34_setup.c) as v34_handshake_helpers.c calls it.
 *
 * initV34's p6 is an address stored in a pointer slot of the block (offsets 0x2a68 / 0xe84 of the object): `void *`, as the definition has it, at both
 * word sizes.  Kept out of v34_internal.h because a test harness that includes v34_modulator.c (and with it v34_internal.h), declares its
 * own initV34 with a 32-bit spelling.
 */
#ifndef DSPLIBS_PORT_V34_INITV34_H
#define DSPLIBS_PORT_V34_INITV34_H
#include "v34_subobjects.h"                                                                                              /* V34BlkB */
extern int initV34(V34BlkB* blk, unsigned short p2, unsigned short p3, short p4, short p5, void* p6, unsigned short p7); /* v34_setup.c */
#endif
