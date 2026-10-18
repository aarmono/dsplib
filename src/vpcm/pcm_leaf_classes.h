/**
 * @file pcm_leaf_classes.h
 * @brief Small V.90 / V.92 helper classes, the K56Flex stubs, G.711 companding and the parameter-parser stubs.
 *
 * Declares the leaf classes of the V.90 / V.92 modem that have no dependencies of their own: V90SdDetector, V90RDetector, V90SpectralShapingFilter,
 * V90ConstellationPower, V90Phase2Info, V92Phase2Info, V92Mapper, ModulusEncoder, ModulusDecoder and the K56FlexFloModem stub, plus the free functions
 * listed below (G.711 conversions, K56Flex entry points, the two parameter-file reader stubs, print_echo_coeffs()).  The definitions and the data tables
 * (the G.711 conversion tables, the average-power limit table, the V92 mapper amplitudes) are in pcm_leaf_classes.cpp.
 *
 * The classes are plain: no virtual functions, no vtable, no operator delete.  Member order is the 32-bit layout; the `0x..` offsets in the member
 * comments are the offsets in that layout, which the asserts generated into pcm_leaf_classes_layout.inc (by tools/layout_gen.py from layouts/vpcm/pcm_leaf_classes.layout)
 * check on 32-bit builds.
 *
 * Numerics: float members hold float32 words that the code moves as integer words where a plain copy is meant; intermediate results that the
 * algorithm keeps at extended precision are `long double`, rounded once where a value is stored to a float or double member.  64-bit integer arithmetic
 * is native int64_t arithmetic.  V90MappingParams, V90Parameters and V92Parameters are only forward-declared here (see pcm_parameters.h and v92_cp.h).
 */
#ifndef DSP_CLASSES_P06_H
#define DSP_CLASSES_P06_H

#include "core/dsp_cxx.h"
#include "core/dsp_math.h"
#include "vparser.h" /* Vparser_read_int / _float */

class V90Parameters;
class V92Parameters;
class V90MappingParams; /* plain struct of >= 0x624 bytes read by byte offset (see V90ConstellationPower) */
struct _tagModemParameters;
struct int_complex;

/** Where the transmit power of a V.90 constellation is measured (selects the code-point table of a V90MappingParams block). */
enum V90TxPowerMeasurementPoint
{
  V90_TX_POINT_DIGITAL_MODEM_TERMINAL = 0, ///< at the digital modem terminal
  V90_TX_POINT_CODEC_OUTPUT = 1            ///< at the codec output (printed as "CodecOutput")
};

/** G.711 companding law. */
enum PcmType
{
  PCM_MU_LAW = 0, ///< mu-law
  PCM_A_LAW = 1   ///< A-law
};

/* ----------------------------------------------------------------------------------------------------- free functions */
extern "C"
{
  /* G.711 (ITU-T) companding, plain C names.  The byte arguments use only the low 8 bits of the int argument. */
  int linear2alaw(int pcm);
  int linear2ulaw(int pcm);
  int alaw2linear(int a);
  int ulaw2linear(int u);
  int alaw2ulaw(int a);
  int ulaw2alaw(int u);
  /* the lookup tables of alaw2ulaw() / ulaw2alaw() */
  extern unsigned char _a2u[128];
  extern unsigned char _u2a[128];

  /* K56Flex (the modem is disabled; these are stubs) */
  void* K56FLEX_Create(void);           /* allocates 0x14 bytes with sysdep_malloc() and leaves them uninitialised */
  void K56FLEX_Delete(void* p);         /* frees p with sysdep_free() unless it is NULL */
  int K56FLEX_SessionTermination(void); /* returns 0; takes no arguments here */
}

/** Print a header line and every coefficient as "   = <sign><int>.<6 digits>" through edprintf(). */
void print_echo_coeffs(float* coefs, unsigned n);

/* ----------------------------------------------------------------------------------------------------- K56FlexFloModem */
/** The disabled K56Flex modem: no data members, every method is an empty stub (k56FlexRunDemodulator() returns 5, the getters return 0). */
class K56FlexFloModem
{
public:
  K56FlexFloModem(void*, int, _tagModemParameters*);
  ~K56FlexFloModem();
  void externalReset();
  void internalReset();
  void k56FlexEnterPhase3();
  void enterPhase3FullDuplex();
  void setMinMaxRates(int, int);
  int k56FlexRunDemodulator(float*, unsigned, int*, int*);
  int getConstellation(int_complex*, unsigned long);
  int getLinearEqualizer(int_complex*, unsigned long);
  int getDFE(int_complex*, unsigned long);
  int getDecisionErrors(int_complex*, unsigned long);
  int getResamplerPhase(int_complex*, unsigned long);
  int getResamplerOffset(int_complex*, unsigned long);
  int getK56FlexJaBits(short*);
  int getK56FlexMpBits(short*);
  int getK56MPsReceiver();
};

/* ----------------------------------------------------------------------------------------------------- V92Mapper */
/**
 * Maps the `nbits_` bits of one symbol to a constellation amplitude: constelAmplitudeTable[bits + 8 * flag_] / sqrt(power_) * scale_.
 * @warning The constructor leaves every member uninitialised; call reset() before process().
 */
class V92Mapper
{
public:
  V92Mapper();
  ~V92Mapper();
  void reset(short scale, unsigned char flag); ///< flag != 0: 3 bits, power 21.0f; flag == 0: 2 bits, power 5.0f
  short process(unsigned char* bits);          ///< bits[0 .. nbits_ - 1], most significant bit LAST (bits[nbits_ - 1])
  static int constelAmplitudeTable[16];

public:
  short scale_;             ///< 0x00 amplitude scale
  unsigned char flag_;      ///< 0x02 selects the 3-bit (non-zero) or 2-bit (zero) mapping
  unsigned char unk03_[35]; ///< 0x03 not used
  short nbits_;             ///< 0x26 bits per symbol (2 or 3)
  float power_;             ///< 0x28 average power of the constellation
};

/* ----------------------------------------------------------------------------------------------------- ModulusEncoder / Decoder */
/**
 * Mixed-radix conversion of a bit string to 6 digits (digit k < 5 in radix m_k, digit 5 = the rest).
 * @warning progress() divides by m0_ .. m4_ without a zero check; the default constructor sets every radix to 0.
 */
class ModulusEncoder
{
public:
  ModulusEncoder(); ///< all seven words 0
  ModulusEncoder(unsigned m0, unsigned m1, unsigned m2, unsigned m3, unsigned m4, unsigned m5, unsigned m6);
  void progress(unsigned char* bits, unsigned* digits); ///< bits[0 .. m6_ - 1] -> digits[0 .. 5]
public:
  unsigned m0_, m1_, m2_, m3_, m4_, m5_, m6_; ///< 0x00 .. 0x18: radices m0_ .. m4_ (m5_ is never read), m6_ = number of bits
};

/** Mixed-radix conversion of 6 digits back to a bit string (the inverse of ModulusEncoder). */
class ModulusDecoder
{
public:
  ModulusDecoder();
  ModulusDecoder(unsigned m0, unsigned m1, unsigned m2, unsigned m3, unsigned m4, unsigned m5, unsigned m6);
  void progress(unsigned char* bits, unsigned* digits); ///< digits[0 .. 5] -> bits[0 .. m6_ - 1]
public:
  unsigned m0_, m1_, m2_, m3_, m4_, m5_, m6_; ///< 0x00 .. 0x18: same meaning as in ModulusEncoder
};

/* ----------------------------------------------------------------------------------------------------- V90Phase2Info / V92Phase2Info */
/** The V.90 phase 2 information a modem sends (its INFO fields) with their defaults from V90Parameters. */
class V90Phase2Info
{
public:
  explicit V90Phase2Info(V90Parameters* p);
  void setToDefault();
  void printInfo() const;

public:
  unsigned pcmType_;       ///< 0x00 1 = A-law
  unsigned rtd_;           ///< 0x04 round-trip delay field
  unsigned char uinfo_;    ///< 0x08
  unsigned char maxTx_;    ///< 0x09 maxTxPower = -0.5 * (maxTx_ + 1) dBm0
  unsigned char pad0a_[2]; ///< 0x0a
  unsigned txPoint_;       ///< 0x0c 1 = CodecOutput
  uint32_t* arrA_;         ///< 0x10 -> VPcmFloModem::uinfoA_ (set by VPcmFloModem::setPhaseIIinfo; not touched by this class's functions)
  uint32_t* arrB_;         ///< 0x14 -> uinfoB_
  float* l2_;              ///< 0x18 21 floats, set by the owner (not by this class's functions)
  uint32_t* arrD_;         ///< 0x1c -> uinfoD_
  V90Parameters* params_;  ///< 0x20 source of the defaults
  /* This class is the single definition of the layout; the V.34 chain view V90Phase2InfoView is a typedef of it.  The three array members are real
   * pointers, so on x86-64 l2_ and params_ sit at +0x18 and +0x28. */
};

/** The V.92 phase 2 information; adds the short-phase-2 and V.92-capability flags to the V.90 fields. */
class V92Phase2Info
{
public:
  explicit V92Phase2Info(V92Parameters* p);
  void setToDefault();
  void printInfo() const;

public:
  unsigned pcmType_;                ///< 0x00
  unsigned rtd_;                    ///< 0x04
  unsigned char uinfo_;             ///< 0x08
  unsigned char maxTx_;             ///< 0x09
  unsigned char pad0a_[2];          ///< 0x0a
  unsigned txPoint_;                ///< 0x0c
  unsigned char shortPhase2Local_;  ///< 0x10
  unsigned char v92CapLocal_;       ///< 0x11
  unsigned char shortPhase2Remote_; ///< 0x12
  unsigned char v92CapRemote_;      ///< 0x13
  unsigned char b14_, b15_, b16_;   ///< 0x14 .. 0x16 copies of V92Parameters words 0x5c / 0x60 / 0x64
  unsigned char v90UseHighCarrier_; ///< 0x17
  uint32_t* arrA_;                  ///< 0x18 -> VPcmFloModem::uinfoA_
  uint32_t* arrB_;                  ///< 0x1c -> uinfoB_
  float* l2_;                       ///< 0x20
  uint32_t* arrD_;                  ///< 0x24 -> uinfoD_
  V92Parameters* params_;           ///< 0x28
};

/* ----------------------------------------------------------------------------------------------------- V90ConstellationPower */
/**
 * The average-power bookkeeping of a V.90 constellation set: mixed-radix moduli of the 6 radices of a V90MappingParams block
 * (calcModulusParameters()), and the average transmit power of the code points that block selects (getPower()).
 */
class V90ConstellationPower
{
public:
  V90ConstellationPower();
  ~V90ConstellationPower();
  void calcModulusParameters(V90MappingParams* p);
  void getConstellationInfo(V90MappingParams* p, V90TxPowerMeasurementPoint point, unsigned k); ///< not called by the modem
  float getPower(V90MappingParams* p, V90TxPowerMeasurementPoint point, PcmType pcm);
  unsigned getPowerIndexForPower(float power); ///< the index of the first limit (from the top) that is >= power, 0 if none
  bool isLegalPowerIndex(unsigned index);      ///< index <= 34; not called by the modem
  static unsigned averagePowerLimits[35];

public:
  const unsigned char* tab_; ///< 0x00 table of code points of the current radix (inside *p)
  unsigned n_;               ///< 0x04 its length
  long long M_;              ///< 0x08 1 << (w620 + w0 - 6)
  long long A_[7];           ///< 0x10 A_[0] = M_ - 1, A_[k + 1] = quotient after stage k
  unsigned r_[6];            ///< 0x48 remainders
  double D_[6];              ///< 0x60 1, d0, d0 d1, ... d0 d1 d2 d3 d4
};

/* ----------------------------------------------------------------------------------------------------- V90SdDetector */
/** Detector driven by a 12-sample history: the ratio of a 6-lag correlation to the energy of the newest 6 samples, compared against three thresholds. */
class V90SdDetector
{
public:
  /** a, b, c: the three thresholds (float words, moved as integers); required: the number of consecutive hits */
  V90SdDetector(float a, float b, float c, unsigned required);
  ~V90SdDetector();
  void reset();
  int process(float x); ///< 1 = detected, -1 = negative detection, 0 = nothing
public:
  unsigned count_;        ///< 0x00 consecutive hits so far
  unsigned required_;     ///< 0x04
  float thA_, thB_, thC_; ///< 0x08 .. 0x10 energy, positive-correlation and negative-correlation thresholds
  float* hist_;           ///< 0x14 heap, 12 floats, newest first
  unsigned n_;            ///< 0x18 = 12
};

/* ----------------------------------------------------------------------------------------------------- V90RDetector */
/** Pattern detector over 1-bit decisions: blocks of 6 (detectR(), detectRNot()) or 12 (detectRf(), detectRfNot()) decisions are compared with patterns. */
class V90RDetector
{
public:
  explicit V90RDetector(V90Parameters* p); ///< stores p only: the object is NOT initialised (reset() does it)
  ~V90RDetector();
  void reset(unsigned a, unsigned b);
  int detectR(short x);
  int detectRNot(short x);
  int detectRf(short x);
  int detectRfNot(short x);

public:
  unsigned cnt_;                       ///< 0x00 decisions collected in the current block
  unsigned lim0_, lim1_, lim2_, lim3_; ///< 0x04 .. 0x10 detection lengths (multiples of 6 or 12)
  unsigned c0_, c1_, c2_;              ///< 0x14 .. 0x1c run lengths of the two patterns and of the negative pattern
  unsigned short sr_;                  ///< 0x20 shift register of the last decisions
  unsigned char pad22_[2];
  int dir_;               ///< 0x24 +1 / -1: which pattern was detected
  V90Parameters* params_; ///< 0x28
};

/* ----------------------------------------------------------------------------------------------------- V90SpectralShapingFilter */
/** Four-coefficient recursive filter that accumulates the energy of its output (the spectral-shaping metric). */
class V90SpectralShapingFilter
{
public:
  V90SpectralShapingFilter();
  void reset();
  void setFilterCoeff(float c0, float c1, float c2, float c3); ///< the four floats are stored as words
  /* The metric (the energy accumulator s3 after n * N_ samples) is an unrounded extended-precision value on the 32-bit x87 stack; the float-returning
   * export reproduces that on i386 (dsp_ret_ext_float).  Code that needs the exact value calls getMetricLD(). */
  DSP_EXTRET_FN float getMetric(const short* in, unsigned n) const;
  long double getMetricLD(const short* in, unsigned n) const;
  void progress(const short* in); ///< N_ samples
public:
  float c0_, c1_, c2_, c3_; ///< 0x00 .. 0x0c coefficients
  float s0_, s1_, s2_, s3_; ///< 0x10 .. 0x1c state (s0 = the previous input sample, s3 = the energy accumulator)
  unsigned N_;              ///< 0x20 samples per progress() call (2)
};

typedef V92Mapper P06_V92Mapper;
typedef ModulusEncoder P06_ModulusEncoder;
typedef ModulusDecoder P06_ModulusDecoder;
typedef V90Phase2Info P06_V90Phase2Info;
typedef V92Phase2Info P06_V92Phase2Info;
typedef V90ConstellationPower P06_V90ConstellationPower;
typedef V90SdDetector P06_V90SdDetector;
typedef V90RDetector P06_V90RDetector;
typedef V90SpectralShapingFilter P06_V90SpectralShapingFilter;

#pragma GCC diagnostic ignored "-Winvalid-offsetof"
#include "pcm_leaf_classes_layout.inc"

#endif /* DSP_CLASSES_P06_H */
