/** @file rcfixed_resampler_tables.h
 *  @brief Coefficient tables of the RcFixed converters (see rcfixed_resampler_tables.c). */
#ifndef DSPLIBS_PORT_RCFIXED_TABLES_H
#define DSPLIBS_PORT_RCFIXED_TABLES_H
extern const short rcfx_rc80to72filter[288];
extern const short rcfx_rc72to80filter[380];
extern const short rcfx_rc320to96filter[204];
extern const short rcfx_rc96to320filter[380];
extern const short rcfx_rc120to80filter[96];
extern const short rcfx_rc80to120filter[126];
extern const short rcfx_rc120to96filter[232];
extern const short rcfx_rc96to120filter[140];
extern const short rcfx_rc384to80filter[340];
extern const short rcfx_rc80to384filter[960];
extern const short rcfx_rc384to96filter[68];
extern const short rcfx_rc96to384filter[184];
extern const short rcfx_rc480to96filter[68];
extern const short rcfx_rc480to80filter[68];
extern const short rcfx_rc96to80filter[160];
extern const short rcfx_rc80to96filter[216];
extern const short rcfx_CI_b2[15];
extern const short rcfx_CI_b1[87];
extern const short rcfx_CI_bDroop[3];
extern const int rcfx_fixedRc_DownFact[21];
extern const int rcfx_fixedRc_UpFact[21];
#endif
