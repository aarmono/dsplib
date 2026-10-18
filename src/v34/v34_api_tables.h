/**
 * @file v34_api_tables.h
 * @brief Declaration of the V.34 wrapper table (defined in v34_api_tables.c).
 */
#ifndef DSPLIBS_PORT_V34_TIER10_TABLES_H
#define DSPLIBS_PORT_V34_TIER10_TABLES_H

#ifdef __cplusplus
extern "C"
{
#endif

/** Disconnect energy thresholds, eight 32-bit entries, selected by the configured threshold code. */
extern const int V34DisconnectThreshTable[8];

#ifdef __cplusplus
}
#endif

#endif
