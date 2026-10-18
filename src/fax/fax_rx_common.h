/**
 * @file fax_rx_common.h
 * @brief The two receive-side DSP blocks the fax modems (V.27ter, V.29, V.17) embed in their receiver bundle: the symbol-rate estimator (SRE, timing recovery)
 *        and the carrier / equalizer block (FSE).
 *
 * - FaxSre / FaxSreCfg with FAXSRE_init(), FAXSRE_recover(), FAXSRE_free() (fax_timing.c): interpolates the received samples to one sample per symbol and
 *   tracks the symbol clock with a correlator-driven loop; also estimates the clock offset in ppm.
 * - FaxFse / FaxFseCfg with FAXFSE_init(), FAXFSE_receive(), FAXFSE_free() (fax_equalizer.c): the complex adaptive equalizer with carrier-phase loop; a
 *   decision callback (FaxFseCb) supplied by the modem turns each equalized symbol into a symbol code.
 *
 * The same algorithms exist for V.32 / V.22 on their own engine objects (src/v32/v32.h, fpm/fpm.h); those keep their pointer slots in an engine tail and so
 * cannot serve other objects. The fax receivers therefore use these typed copies with real pointer members. The default configuration blocks are in
 * fax_rx_common_tables.c.
 *
 * Layout: in 32-bit builds every member sits at the offset the `_Static_assert`s pin, so that the objects embedded in the receiver bundles keep their size;
 * in 64-bit builds the members after a pointer move, so NEVER address these objects by a raw byte offset.
 *
 * Table words: the config words that hold a table address (filters, clocks, PLL constants) stay 32-bit table handles (core/dsp_table_handles.h): `int`,
 * resolved with CFGP_WORD() where used. The FSE config's two address words (history object, decision callback) are real pointers.
 *
 * Member names are descriptive; the ones documented "medium" are named from how the code uses them (and the debug strings "Timing Offset [ppm]" and "Decoder
 * Error"), not from a specification.
 */
#ifndef DSPLIBS_PORT_FAX_RX_OBJ_H
#define DSPLIBS_PORT_FAX_RX_OBJ_H
#include <stddef.h>
#include <stdint.h>
#include "core/dsp_table_handles.h"

/** Symbol-rate estimator configuration (14 words; the names are those of V32Sre in src/v32/v32.h where the member is the same). */
typedef struct FaxSreCfg
{
  short sps;          ///< samples per symbol; also a multiplier of the loop output
  short periodsTrack; ///< symbol periods (of cfg.sps output samples) correlated per loop update while searching / tracking; copied to periodsCur (medium)
  short periodsLock;  ///< the same once locked (state 2) (medium)
  short startCount;   ///< loop updates the loop stays in state 0 (idle) before it starts to track (state 1) (medium)
  short unlockGain;   ///< Q15 factor on loopInteg when the lock is lost (state 2 -> 1) (medium)
  short lockGain;     ///< Q12 factor on loopInteg when the lock is gained (state 1 -> 2) (medium)
  short nt;           ///< number of coefficients of the interpolation filter
  short s0e;          ///< not read
  int coefTab;        ///< table handle: the filter table (nt shorts)
  int tedCoef;        ///< table handle: the timing-error detector coefficient block (11 shorts: two biquads and a combiner) (medium)
  int corrCosTab;     ///< table handle: the clock correlator's x (cosine) table, indexed by corrIdx (medium)
  int corrSinTab;     ///< table handle: the clock correlator's y (sine) table (medium)
  int propGainTab;    ///< table handle: loop proportional gain by loopState (3 shorts)
  int intGainTab;     ///< table handle: loop integral gain by loopState (3 shorts)
  short errOn;        ///< pllEnable is set when errMagAvg reaches this value (medium)
  short errOff;       ///< pllEnable is cleared when errMagAvg falls to this value (medium)
  short unlockThresh; ///< the lock is lost when |phaseAvg| >= unlockThresh >> 3 (medium)
  short lockThresh;   ///< the lock is gained when |phaseAvg| <= lockThresh >> 3 (medium)
  short levelThresh;  ///< level threshold of the energy gate
  short levelLen;     ///< length of the energy buffer
  int w34;            ///< user word (never read)
} FaxSreCfg;

/** FaxSre.loopState, named from the behaviour of FAXSRE_recover() (medium). */
enum FaxSreLoopState
{
  FAXSRE_LOOP_IDLE = 0,  ///< idle / start: the first cfg.startCount loop updates
  FAXSRE_LOOP_TRACK = 1, ///< tracking
  FAXSRE_LOOP_LOCKED = 2 ///< locked
};

/** Symbol-rate estimator state (timing recovery and clock-offset estimator). */
typedef struct FaxSre
{
  FaxSreCfg cfg;      ///< configuration (copied by FAXSRE_init())
  short loopState;    ///< FaxSreLoopState: 0 idle / 1 tracking / 2 locked
  short loopInteg;    ///< integral term of the loop filter
  short phaseAvg;     ///< smoothed timing phase error (15/16 memory)
  short errMagAvg;    ///< smoothed |timing error| (15/16 memory)
  int pllEnable;      ///< enable flag of the loop (errOn / errOff hysteresis of errMagAvg; the receiver's carrier gate)
  int w44;            ///< set at init, cleared at the first lock
  int loopEnable;     ///< enable of the loop update
  short histLen;      ///< cfg.nt / 10
  short histPos;      ///< write position in hist
  short* coefs;       ///< allocated, 2 * nt shorts: the interpolation filter (rewritten by the loop)
  short* hist;        ///< allocated, 2 * histLen shorts: input history
  short* st;          ///< allocated, 12 bytes: timing-error detector state (6 shorts)
  int corrA, corrB;   ///< clock correlator accumulators (x, y); their angle is the timing phase error
  short phaseFrac;    ///< fractional phase (0 .. 0x7ff)
  short phaseRow;     ///< polyphase row, phase index (0 .. 9)
  short periodsCur;   ///< symbol periods per loop update now (periodsTrack or periodsLock)
  short startCtr;     ///< loop updates done in the idle state (counts to cfg.startCount)
  short periodCount;  ///< symbol periods accumulated since the last loop update
  short corrIdx;      ///< position in the correlator tables (0 .. cfg.sps - 1)
  short untilSym;     ///< samples until the next output sample
  short s72;          ///< not used
  short* rms;         ///< allocated, 2 * cfg.levelLen bytes: energy window
  short levelMeasure; ///< energy gate active (1 until the level exceeds cfg.levelThresh)
  short levelIdx;     ///< write index of the energy window
  short ppmTick;      ///< elapsed units per call (nominal samples per block); written by the receiver's create function
  short ppmElapsed;   ///< clock-offset (ppm) estimator (ppmElapsed .. ppmFirst): elapsed units since the last estimate (medium)
  short ppmSum;       ///< running sum of the net counts
  short ppmOffset;    ///< the estimate in ppm ("Timing Offset [ppm]")
  short ppmN;         ///< estimates averaged so far
  short ppmNet;       ///< net count of the output samples: +1 when the phase row wrapped twice in one output sample, -1 when it did not wrap
  short ppmStep;      ///< ppm per net count (1000000 / (sps * ppmPeriod))
  short ppmPeriod;    ///< elapsed units per estimate (medium)
  short ppmNMax;      ///< averaging limit of ppmN
  short ppmFirst;     ///< 1 until the first estimate has been made (medium)
} FaxSre;

struct FaxFse;
/** Decision callback of the equalizer: gets the equalized symbol (angle, magnitude) and returns the symbol code in the low 16 bits. */
typedef int (*FaxFseCb)(struct FaxFse* fse, short* angle, short* mag);

/** Carrier / equalizer configuration (14 words; the names are those of V32Fse in src/v32/v32.h where the member is the same). */
typedef struct FaxFseCfg
{
  short blockLen;       ///< block length (samples) of a call: the output arrays hold blockLen / sps symbols
  short sps;            ///< samples per symbol
  int iCoefTab;         ///< table handle: I filter coefficients (nt shorts)
  int qCoefTab;         ///< table handle: Q filter coefficients
  short nt;             ///< number of equalizer taps
  short mu[3];          ///< step sizes (train, track; indexed by FaxFse.muIdx); the third is never read
  int carrierTab;       ///< table handle: carrier phase table
  short crrLen, crrAdj; ///< length of the carrier table, step
  short settleCount;    ///< symbols the PLL gain set stays at FAXFSE_GAIN_SETTLE (medium)
  short errThreshHi;    ///< smoothed phase error at or above which the wide gain set is used (medium)
  short errThreshLo;    ///< smoothed phase error at or below which the tight gain set is used (medium)
  short s22;            ///< not read
  int propTab;          ///< table handle: PLL proportional gain by loopState (3 shorts)
  int intTab;           ///< table handle: PLL integral gain by loopState (3 shorts)
  void* hist;           ///< the owner's history object (not used by the equalizer itself; the decision callback finds it here)
  FaxFseCb cb;          ///< decision callback
  int w34;              ///< user word (never read)
} FaxFseCfg;

#define FAXFSE_DIAG0 0x1e0 ///< entries (two ints each) of the first diagnostic ring
#define FAXFSE_DIAG1 0x7d0 ///< entries of the second diagnostic ring (never written by the functions of this library)

/** FaxFse.loopState: the PLL gain set (index of cfg.propTab / intTab), named from the behaviour of FAXFSE_receive() (medium). */
enum FaxFseGainSet
{
  FAXFSE_GAIN_SETTLE = 0, ///< settling: the first cfg.settleCount symbols
  FAXFSE_GAIN_WIDE = 1,   ///< smoothed phase error >= cfg.errThreshHi
  FAXFSE_GAIN_TIGHT = 2   ///< smoothed phase error <= cfg.errThreshLo
};

/** Carrier / equalizer state. */
typedef struct FaxFse
{
  FaxFseCfg cfg;               ///< configuration (copied by FAXFSE_init())
  short muIdx;                 ///< index of the step size (0 = train, 1 = track)
  short loopState;             ///< PLL gain set (FaxFseGainSet): index of cfg.propTab / intTab
  int freqAcc;                 ///< integral of the phase error (the frequency term)
  int eqForce;                 ///< equalizer update enable (forced)
  int pllEnable;               ///< PLL update enable
  int predEnable;              ///< phase-offset predictor enable
  int eqEnable;                ///< equalizer update enable (when the error is non zero)
  short phaseErrAvg;           ///< smoothed phase error
  short decErrAvg;             ///< smoothed decoder error ("Decoder Error = %d")
  short *outI, *outQ;          ///< allocated, (blockLen / sps) * 2 + 4 shorts each: the equalized symbol coordinates
  unsigned short count;        ///< block length of the call
  short idx;                   ///< number of symbols decided in the call (index of the current one)
  short *tapsI, *tapsQ;        ///< allocated, 2 * nt shorts each: the equalizer taps
  short* hbuf;                 ///< allocated, 2 * nt shorts: the input history
  short pos;                   ///< write index of hbuf
  short s6e;                   ///< not used
  int carrierAcc;              ///< carrier phase accumulator
  short phaseAcc;              ///< table phase index
  short predOffset;            ///< predicted phase offset
  short pc[4];                 ///< predictor coefficients
  short ph[4];                 ///< phase-error history
  short settleCtr;             ///< symbols counted towards cfg.settleCount
  short untilSym;              ///< samples to the next symbol
  int diag0[FAXFSE_DIAG0 * 2]; ///< ring of (I >> 1, Q >> 1) pairs
  int diag1[FAXFSE_DIAG1 * 2]; ///< second ring (never written here)
  int nd0;                     ///< entries in diag0
  int nd1;                     ///< cleared at init, not used otherwise
  int nd2;                     ///< cleared at init, not used otherwise
  int w4e18;                   ///< not used
} FaxFse;

#if UINTPTR_MAX == 0xffffffffu
_Static_assert(sizeof(FaxSreCfg) == 0x38 && offsetof(FaxSreCfg, nt) == 0xc && offsetof(FaxSreCfg, coefTab) == 0x10 && offsetof(FaxSreCfg, intGainTab) == 0x24 &&
                   offsetof(FaxSreCfg, errOn) == 0x28 && offsetof(FaxSreCfg, levelThresh) == 0x30 && offsetof(FaxSreCfg, levelLen) == 0x32 &&
                   offsetof(FaxSreCfg, w34) == 0x34,
               "FaxSreCfg 32-bit layout");
_Static_assert(sizeof(FaxSre) == 0x90 && offsetof(FaxSre, loopState) == 0x38 && offsetof(FaxSre, pllEnable) == 0x40 && offsetof(FaxSre, loopEnable) == 0x48 &&
                   offsetof(FaxSre, histLen) == 0x4c && offsetof(FaxSre, coefs) == 0x50 && offsetof(FaxSre, hist) == 0x54 && offsetof(FaxSre, st) == 0x58 &&
                   offsetof(FaxSre, corrA) == 0x5c && offsetof(FaxSre, phaseFrac) == 0x64 && offsetof(FaxSre, untilSym) == 0x70 &&
                   offsetof(FaxSre, rms) == 0x74 && offsetof(FaxSre, levelMeasure) == 0x78 && offsetof(FaxSre, ppmTick) == 0x7c &&
                   offsetof(FaxSre, ppmFirst) == 0x8e,
               "FaxSre 32-bit layout");
_Static_assert(sizeof(FaxFseCfg) == 0x38 && offsetof(FaxFseCfg, nt) == 0xc && offsetof(FaxFseCfg, carrierTab) == 0x14 && offsetof(FaxFseCfg, crrLen) == 0x18 &&
                   offsetof(FaxFseCfg, propTab) == 0x24 && offsetof(FaxFseCfg, hist) == 0x2c && offsetof(FaxFseCfg, cb) == 0x30 &&
                   offsetof(FaxFseCfg, w34) == 0x34,
               "FaxFseCfg 32-bit layout");
_Static_assert(sizeof(FaxFse) == 0x4e1c && offsetof(FaxFse, muIdx) == 0x38 && offsetof(FaxFse, freqAcc) == 0x3c && offsetof(FaxFse, eqForce) == 0x40 &&
                   offsetof(FaxFse, eqEnable) == 0x4c && offsetof(FaxFse, phaseErrAvg) == 0x50 && offsetof(FaxFse, outI) == 0x54 &&
                   offsetof(FaxFse, count) == 0x5c && offsetof(FaxFse, idx) == 0x5e && offsetof(FaxFse, tapsI) == 0x60 && offsetof(FaxFse, hbuf) == 0x68 &&
                   offsetof(FaxFse, pos) == 0x6c && offsetof(FaxFse, carrierAcc) == 0x70 && offsetof(FaxFse, pc) == 0x78 && offsetof(FaxFse, ph) == 0x80 &&
                   offsetof(FaxFse, untilSym) == 0x8a && offsetof(FaxFse, diag0) == 0x8c && offsetof(FaxFse, diag1) == 0xf8c &&
                   offsetof(FaxFse, nd0) == 0x4e0c && offsetof(FaxFse, nd2) == 0x4e14,
               "FaxFse 32-bit layout");
#endif

extern const FaxSreCfg fax_sre_cfg_default; ///< default timing-recovery configuration (fax_rx_common_tables.c)
extern const FaxFseCfg fax_fse_cfg_default; ///< default equalizer configuration (fax_rx_common_tables.c)

/**
 * @brief Initialise a symbol-rate estimator from a configuration.
 * @param s     estimator state
 * @param cfg   configuration (copied)
 * @param fresh nonzero: allocate the buffers; zero: re-initialise a live object (the buffers are kept unless the filter grows)
 */
void FAXSRE_init(FaxSre* s, const FaxSreCfg* cfg, int fresh);
/** @brief Release the buffers of a symbol-rate estimator. */
void FAXSRE_free(FaxSre* s);
/**
 * @brief Timing recovery of one block of input samples.
 * @param s     estimator state
 * @param in    `count` input samples
 * @param out   one interpolated sample per symbol is written here
 * @param count number of input samples
 * @return number of output samples written
 */
unsigned int FAXSRE_recover(FaxSre* s, const unsigned short* in, short* out, short count);
/**
 * @brief Initialise an equalizer / carrier loop from a configuration.
 * @param f     equalizer state
 * @param cfg   configuration (copied)
 * @param fresh nonzero: allocate the buffers; zero: re-initialise a live object (the buffers are always freed and allocated again)
 */
void FAXFSE_init(FaxFse* f, const FaxFseCfg* cfg, int fresh);
/** @brief Release the buffers of an equalizer. */
void FAXFSE_free(FaxFse* f);
/**
 * @brief Equalizer and carrier loop over one block of samples (the output of FAXSRE_recover()).
 * @param f       equalizer state
 * @param in      `count` input samples
 * @param outBits one decision (symbol code) per symbol is written here
 * @param count   number of input samples
 * @return number of symbols decided
 */
short FAXFSE_receive(FaxFse* f, const short* in, short* outBits, unsigned short count);
#endif
