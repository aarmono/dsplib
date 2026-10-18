/**
 * @file v34_timing_hilbert_tables.h
 * @brief Declarations of the timing-filter and Hilbert-filter coefficient tables (defined in v34_timing_hilbert_tables.c).
 *
 * Consumed by V34TimingHPFilter(), V34TimingFilter(), V34TimingFiltersInit(), V34TimingPrefilter(), V34HilbertFilter() and V34InitHilbertFilter() in
 * v34_dsp_primitives.c.
 */
#ifndef DSPLIBS_PORT_V34_TABLES_H
#define DSPLIBS_PORT_V34_TABLES_H

/* timing-filter coefficient tables (V34TimingHPFilter / V34TimingFilter / V34TimingPrefilter) */
extern const short V34TimingHPFilterCoeff[40];
extern const short V34TimingPrefilterCoeff[40];

/* half-baud-rate timing-recovery A / B coefficient triples (V34TimingFilter's stages) */
extern const short posHalfBaud_Acoef_Real[3];
extern const short posHalfBaud_Acoef_Imag[3];
extern const short negHalfBaud_Acoef_Real[3];
extern const short negHalfBaud_Acoef_Imag[3];
extern const short posHalfBaud_Bcoef_Real[3];
extern const short posHalfBaud_Bcoef_Imag[3];
extern const short negHalfBaud_Bcoef_Real[3];
extern const short negHalfBaud_Bcoef_Imag[3];

/* Hilbert-filter coefficient tables (V34HilbertFilter) */
extern const short V34hilbertrealcoef[64];
extern const short V34hilbertimagcoef[64];

#endif
