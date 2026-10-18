/**
 * @file v34_init_tables.h
 * @brief Declarations of the set-up tables (defined in v34_init_tables.c).
 *
 * Consumed by v34_init.c (preinitV34 / preinitdigital: Convolve16), v34_setup.c (initV34, V34SetupDemodulator, V34SetupModulator, v34setuptxmit:
 * everything else except offsetarr) and src/vpcm (offsetarr, StateName, rxvect4, hsine1800).
 */
#ifndef DSPLIBS_PORT_V34_TIER3_TABLES_H
#define DSPLIBS_PORT_V34_TIER3_TABLES_H

#include "v34_hs_states.h" /* V34HS_COUNT (the StateName dimension) */

#ifdef __cplusplus
extern "C"
{
#endif

extern const short Convolve64[64];
extern const short Convolve32[64];
extern const short Convolve16[64];
extern const signed char MMinTable[32];
extern const signed char MMaxTable[32];
extern const unsigned short gInvertPat[16];
extern const short hsine1200[16];
extern const short hsine1600[12];
extern const short hsine1680[80];
extern const short hsine1800[32];
extern const short hsine1829[42];
extern const short hsine1867[72];
extern const short hsine1920[10];
extern const short hsine1959[98];
extern const short hsine2000[48];
extern const short hsine2400[16];
extern const short tx600c1[128];
extern const short txAllPass[128];
extern const short tx2400c1[128];
extern const short tx2800c1[768];
extern const short tx3000c1[512];
extern const short tx3200c1_for_v34[96];
extern const short tx3200c1_for_v90[192];
extern const short tx3429c1[448];
extern const short p2400[160];
extern const short p2800[160];
extern const short p3000[160];
extern const short p3200[160];
extern const short p3429[160];
extern const short preemp0[16];
extern const short ec_prem_coef_B2400High[42];
extern const short ec_prem_coef_B2400[42];
extern const short ec_prem_coef_B2800High[42];
extern const short ec_prem_coef_B2800[42];
extern const short ec_prem_coef_B3000High[42];
extern const short ec_prem_coef_B3000[42];
extern const short ec_prem_coef_B3200High[42];
extern const short ec_prem_coef_B3200[42];
extern const short ec_prem_coef_B3429[42];
extern const short V90EchoPrefilterCoeff[42];
extern const int rxvect4[4];
extern const unsigned int xyz[945];
extern const int offsetarr[10];
extern const char *const StateName[V34HS_COUNT];

#ifdef __cplusplus
}
#endif

#endif
