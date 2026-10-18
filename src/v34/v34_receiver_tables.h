/**
 * @file v34_receiver_tables.h
 * @brief Declaration of the receiver reference-symbol table vectpp (defined in v34_receiver_tables.c).
 *
 * The other tables receive() reads are declared elsewhere: rxvect4 in v34_init_tables.h, costbl (a macro for v8_costbl) in v34_symbol_tables.h and
 * the square-root table in fpm_tables.h.
 */
#ifndef DSPLIBS_PORT_V34_TIER7_TABLES_H
#define DSPLIBS_PORT_V34_TIER7_TABLES_H

#ifdef __cplusplus
extern "C"
{
#endif

/** 48 {I, Q} reference symbol pairs. */
extern const short vectpp[96];

#ifdef __cplusplus
}
#endif

#endif
