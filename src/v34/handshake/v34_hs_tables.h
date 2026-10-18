/**
 * @file v34_hs_tables.h
 * @brief Declaration of the table of the V.34 handshake state machine (defined in v34_hs_tables.c).
 *
 * Read by the transmit state TX_L1 (v34_hs_tx_mid.cpp) and by the PCM modem code (src/vpcm/vpcm_flo_modem.cpp).
 */
#ifndef V34_HS_TABLES_H
#define V34_HS_TABLES_H

#ifdef __cplusplus
extern "C"
{
#endif

/** One period (64 samples, 16-bit signed) of the line-probing signal sent in the transmit state TX_L1. */
extern const short probe[64];

#ifdef __cplusplus
}
#endif

#endif /* V34_HS_TABLES_H */
