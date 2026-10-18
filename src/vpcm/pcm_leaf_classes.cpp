/**
 * @file pcm_leaf_classes.cpp
 * @brief Small V.90 / V.92 helper classes, K56Flex stubs, G.711 companding functions, parameter-parser stubs and print_echo_coeffs().
 *
 * Definitions for the classes declared in pcm_leaf_classes.h:
 * - free functions (C linkage): G.711 conversions linear2alaw() / linear2ulaw() / alaw2linear() / ulaw2linear() / alaw2ulaw() / ulaw2alaw() with their
 *   tables, the K56FLEX_* stubs and the Vparser_read_int() / Vparser_read_float() stubs (no-ops returning 0);
 * - print_echo_coeffs() (C++ linkage);
 * - K56FlexFloModem (empty stubs), V92Mapper (constellation amplitude), ModulusEncoder / ModulusDecoder (mixed-radix bit/digit conversion),
 *   V90Phase2Info / V92Phase2Info (phase 2 INFO fields and their debug dump), V90ConstellationPower (average-power bookkeeping of a constellation set),
 *   V90SdDetector / V90RDetector (pattern detectors) and V90SpectralShapingFilter (energy metric of a 4-coefficient recursive filter).
 *
 * Numerics: float32 / double in memory; intermediate results that are computed at extended precision are `long double` and rounded once where they are
 * stored to a float or double member; float words that are only moved are moved as integer words.  Comparisons against NaN are written so that the
 * unordered case takes the same branch as the 32-bit x87 compare sequence (`ja` / `jae` are not taken, `jb` / `jbe` are).  64-bit divisions are native
 * int64_t `/` and `%`.
 *
 * The parameter blocks V90Parameters and V92Parameters are read through their named members (pcm_parameters.h; their layout differs between 32-bit and
 * 64-bit builds).  The V90MappingParams block holds no pointer, so its layout is the same at both widths and V90ConstellationPower reads it by byte
 * offset (`pw()`).
 */

#include "pcm_leaf_classes.h"
#include "pcm_parameters.h" /* V90Parameters, V92Parameters */
#include "core/dsp_asm.h"
#include "core/dsp_host.h"
#include "v34/v34_api.h"

extern "C"
{
}

typedef long double LD;

/* ------------------------------------------------------------------------------------------------ small helpers */

#include "core/dsp_wordof_value.h"
#include "core/dsp_floatbits.h"
#include "core/dsp_copyword.h"
/* a 32-bit word of a pointer-free block, by byte offset (the offsets are the same at both widths) */
static inline uint32_t pw(const void* p, unsigned off)
{
  uint32_t v;
  __builtin_memcpy(&v, (const unsigned char*)p + off, sizeof v);
  return v;
}
#include "core/dsp_intmath.h"
#include "core/dsp_x87_conv.h"

/* ================================================================================================ G.711 (plain C functions) */
/* (declared extern "C" in the header: the definitions below keep the C linkage) */

/* the segment end points of the G.711 encoders; read by linear2alaw() and linear2ulaw() only */
static short seg_end[8] = {255, 511, 1023, 2047, 4095, 8191, 16383, 32767};

unsigned char _a2u[128] = {0x01, 0x03, 0x05, 0x07, 0x09, 0x0b, 0x0d, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d,
                           0x1e, 0x1f, 0x20, 0x20, 0x21, 0x21, 0x22, 0x22, 0x23, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f,
                           0x30, 0x30, 0x31, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f, 0x40, 0x40, 0x41, 0x42,
                           0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f, 0x4f, 0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57,
                           0x58, 0x59, 0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f, 0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b, 0x6c, 0x6d,
                           0x6e, 0x6f, 0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f};

unsigned char _u2a[128] = {0x01, 0x01, 0x02, 0x02, 0x03, 0x03, 0x04, 0x04, 0x05, 0x05, 0x06, 0x06, 0x07, 0x07, 0x08, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e,
                           0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1b, 0x1d, 0x1f, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28,
                           0x29, 0x2a, 0x2b, 0x2c, 0x2e, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x40, 0x41,
                           0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58,
                           0x59, 0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f, 0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b, 0x6c, 0x6d, 0x6e,
                           0x6f, 0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f, 0x80};

/**
 * @brief Encode a linear sample as A-law.
 * @param pcm  linear sample (16-bit scale)
 * @return the A-law byte (0 .. 255); a magnitude beyond the last segment gives the maximum code
 */
int linear2alaw(int pcm)
{
  unsigned mask = 0xd5;
  int v = pcm;
  if (pcm < 0)
  {
    mask = 0x55;
    v = (int)((0u - (unsigned)pcm) - 8u);
  }
  int seg = 0;
  while (seg < 8 && (int)seg_end[seg] < v)
    seg++; /* signed compare of the 16-bit segment end */
  if (seg >= 8)
    return (int)(mask ^ 0x7f);
  unsigned code = ((unsigned)seg << 4) | ((unsigned)(v >> (seg <= 1 ? 4 : seg + 3)) & 0xf);
  return (int)((code ^ mask) & 0xff);
}

/**
 * @brief Encode a linear sample as mu-law.
 * @param pcm  linear sample (16-bit scale)
 * @return the mu-law byte (0 .. 255); a magnitude beyond the last segment gives the maximum code
 */
int linear2ulaw(int pcm)
{
  unsigned mask = 0xff;
  int v = (int)((unsigned)pcm + 0x84u);
  if (pcm < 0)
  {
    mask = 0x7f;
    v = (int)(0x84u - (unsigned)pcm);
  }
  int seg = 0;
  while (seg < 8 && (int)seg_end[seg] < v)
    seg++;
  if (seg >= 8)
    return (int)(mask ^ 0x7f);
  unsigned code = ((unsigned)seg << 4) | ((unsigned)(v >> (seg + 3)) & 0xf);
  return (int)((code ^ mask) & 0xff);
}

/**
 * @brief Decode an A-law byte to a linear sample.
 * @param a  A-law byte (only the low 8 bits are used)
 * @return the linear sample (16-bit scale)
 */
int alaw2linear(int a)
{
  unsigned b = ((unsigned)a & 0xffu) ^ 0x55u;
  unsigned seg = (b & 0x70u) >> 4;
  int t = (int)((b & 0xfu) << 4);
  int r;
  if (seg == 0)
    r = t + 8;
  else if (seg == 1)
    r = t + 0x108;
  else
    r = (t + 0x108) << (seg - 1);
  return (b & 0x80u) ? r : -r;
}

/**
 * @brief Decode a mu-law byte to a linear sample.
 * @param u  mu-law byte (only the low 8 bits are used)
 * @return the linear sample (16-bit scale)
 */
int ulaw2linear(int u)
{
  unsigned b = ~(unsigned)u & 0xffu;
  unsigned seg = (b & 0x70u) >> 4;
  int t = (int)((((b & 0xfu) << 3) + 0x84u) << seg);
  return (b & 0x80u) ? 0x84 - t : t - 0x84;
}

/**
 * @brief Convert an A-law byte to the nearest mu-law byte through _a2u.
 * @param a  A-law byte (only the low 8 bits are used)
 * @return the mu-law byte
 */
int alaw2ulaw(int a)
{
  unsigned a8 = (unsigned)a & 0xffu;
  if (a8 & 0x80u)
    return (int)(~(unsigned)_a2u[a8 ^ 0xd5u] & 0xffu);
  return (int)((unsigned)_a2u[a8 ^ 0x55u] ^ 0x7fu);
}

/**
 * @brief Convert a mu-law byte to the nearest A-law byte through _u2a.
 * @param u  mu-law byte (only the low 8 bits are used)
 * @return the A-law byte
 */
int ulaw2alaw(int u)
{
  unsigned u8 = (unsigned)u & 0xffu;
  if (u8 & 0x80u)
    return (int)((unsigned)(unsigned char)(_u2a[~u8 & 0xffu] - 1u) ^ 0xd5u);
  return (int)((unsigned)(unsigned char)(_u2a[u8 ^ 0x7fu] - 1u) ^ 0x55u);
}

/*
 * The K56Flex handle has no C++ class: K56FLEX_Create() returns a raw block that v34_agc_demod.c reads through the C view K56HandleObj
 * (src/v34/v34_k56flex_view.h).  The block size is the size of that view, and the view has no pointer member, so the 0x14 bytes and the offsets are the
 * same at -m32 and -m64 (checked here on whichever word size this file is built).
 */
extern "C"
{
#include "v34/v34_k56flex_view.h"
}
DSP_STATIC_ASSERT(sizeof(K56HandleObj) == 0x14, "sizeof(K56HandleObj) != 0x14 (the K56FLEX_Create() allocation)");
DSP_STATIC_ASSERT(offsetof(K56HandleObj, word0_) == 0x0 && offsetof(K56HandleObj, flag8_) == 0x8 && offsetof(K56HandleObj, word_c_) == 0xc,
                  "K56HandleObj members are not at their i386 offsets");

/* ================================================================================================ K56Flex stubs, parser stubs */
/* (declared in the header: all of these do nothing) */
/** @warning The block is returned uninitialised and the sysdep_malloc() result is not checked. */
void* K56FLEX_Create(void) { return sysdep_malloc(sizeof(K56HandleObj)); }
void K56FLEX_Delete(void* p)
{
  if (p)
    sysdep_free(p);
}
int K56FLEX_SessionTermination(void) { return 0; }
int Vparser_read_int(void*, const char*, int*) { return 0; }
int Vparser_read_float(void*, const char*, float*) { return 0; }

K56FlexFloModem::K56FlexFloModem(void*, int, _tagModemParameters*) {}
K56FlexFloModem::~K56FlexFloModem() {}
void K56FlexFloModem::externalReset() {}
void K56FlexFloModem::internalReset() {}
void K56FlexFloModem::k56FlexEnterPhase3() {}
void K56FlexFloModem::enterPhase3FullDuplex() {}
void K56FlexFloModem::setMinMaxRates(int, int) {}
int K56FlexFloModem::k56FlexRunDemodulator(float*, unsigned, int*, int*) { return 5; }
int K56FlexFloModem::getConstellation(int_complex*, unsigned long) { return 0; }
int K56FlexFloModem::getLinearEqualizer(int_complex*, unsigned long) { return 0; }
int K56FlexFloModem::getDFE(int_complex*, unsigned long) { return 0; }
int K56FlexFloModem::getDecisionErrors(int_complex*, unsigned long) { return 0; }
int K56FlexFloModem::getResamplerPhase(int_complex*, unsigned long) { return 0; }
int K56FlexFloModem::getResamplerOffset(int_complex*, unsigned long) { return 0; }
int K56FlexFloModem::getK56FlexJaBits(short*) { return 0; }
int K56FlexFloModem::getK56FlexMpBits(short*) { return 0; }
int K56FlexFloModem::getK56MPsReceiver() { return 0; }

#include "vpcm_split.h"
#include "vpcm_split_value.h"

/**
 * @brief Print a coefficient list through edprintf(), six decimals each.
 * @param coefs  coefficients
 * @param n      number of coefficients
 */
void print_echo_coeffs(float* coefs, unsigned n)
{
  edprintf("**************** NEAR ECHO FILTER *************** \n");
  for (unsigned i = 0; i < n; i++)
  {
    Split s = splitValue((LD)coefs[i], 1000000.0L);
    edprintf("   = %c%d.%06d\r\n", s.sign, s.ip, s.fp);
  }
}

/* ================================================================================================ V92Mapper */
int V92Mapper::constelAmplitudeTable[16] = {1, 3, -1, -3, 0, 0, 0, 0, 1, 3, 5, 7, -1, -3, -5, -7};

/** @brief Leaves every member uninitialised; call reset(). */
V92Mapper::V92Mapper() {}
V92Mapper::~V92Mapper() {}

/**
 * @brief Select the mapping.
 * @param scale  output amplitude scale
 * @param flag   0: 2 bits per symbol, power 5.0; non-zero: 3 bits per symbol, power 21.0
 * @warning A `flag` other than 0 or 1 makes process() index constelAmplitudeTable[] beyond its 16 entries.
 */
void V92Mapper::reset(short scale, unsigned char flag)
{
  flag_ = flag;
  scale_ = scale;
  if (flag == 0)
  {
    nbits_ = 2;
    storeWord(&power_, 0x40a00000u); /* 5.0f */
  }
  else
  {
    nbits_ = 3;
    storeWord(&power_, 0x41a80000u); /* 21.0f */
  }
}

/**
 * @brief Map one symbol's bits to a constellation amplitude.
 * @param bits  nbits_ bytes, most significant bit last; each byte is merged whole (not masked to one bit) into the index
 * @return amplitude = table[index] / sqrt(power_) * scale_, truncated toward zero to 16 bits
 * @note  The index accumulator is kept as a sign-extended 16-bit value.
 */
short V92Mapper::process(unsigned char* bits)
{
  int acc = 0;
  for (int i = (int)nbits_ - 1; i >= 0; i--)
    acc = (short)((unsigned short)(2 * acc) | bits[i]);
  int idx = acc + (int)flag_ * 8;

  LD r = (LD)constelAmplitudeTable[idx] / dsp_sqrt((LD)power_);
  r = r * (LD)scale_;
  return trunc16(r);
}

/* ================================================================================================ ModulusEncoder / ModulusDecoder */
/** @brief All seven words zero. */
ModulusEncoder::ModulusEncoder()
{
  m6_ = 0;
  m5_ = 0;
  m4_ = 0;
  m3_ = 0;
  m2_ = 0;
  m1_ = 0;
  m0_ = 0;
}
ModulusEncoder::ModulusEncoder(unsigned m0, unsigned m1, unsigned m2, unsigned m3, unsigned m4, unsigned m5, unsigned m6)
{
  m0_ = m0;
  m1_ = m1;
  m2_ = m2;
  m3_ = m3;
  m4_ = m4;
  m5_ = m5;
  m6_ = m6;
}

/**
 * @brief Convert the bit string to six mixed-radix digits.
 * @param bits    m6_ bits (one per byte, bit 0 of each byte), most significant bit LAST
 * @param digits  receives digits[0 .. 5]: digits[k] = value mod m_k for k < 5, then value = (value - digit) / m_k; digits[5] is the final quotient
 * @warning The bit counter is signed: m6_ >= 0x80000001 makes the bit loop run for about 2^31 iterations and read far beyond `bits`.
 * @warning A zero radix m0_ .. m4_ (the default-constructed state) divides by zero.
 */
void ModulusEncoder::progress(unsigned char* bits, unsigned* digits)
{
  int64_t acc = 0;
  for (int e = (int)(m6_ - 1u); e >= 0; e--)
    acc = (int64_t)(((uint64_t)acc << 1) | (uint64_t)(bits[e] & 1u));

  digits[0] = (uint32_t)(acc % (int64_t)m0_);
  acc = (int64_t)((uint64_t)acc - (uint64_t)digits[0]) / (int64_t)m0_;
  digits[1] = (uint32_t)(acc % (int64_t)m1_);
  acc = (int64_t)((uint64_t)acc - (uint64_t)digits[1]) / (int64_t)m1_;
  digits[2] = (uint32_t)(acc % (int64_t)m2_);
  acc = (int64_t)((uint64_t)acc - (uint64_t)digits[2]) / (int64_t)m2_;
  digits[3] = (uint32_t)(acc % (int64_t)m3_);
  acc = (int64_t)((uint64_t)acc - (uint64_t)digits[3]) / (int64_t)m3_;
  digits[4] = (uint32_t)(acc % (int64_t)m4_);
  acc = (int64_t)((uint64_t)acc - (uint64_t)digits[4]) / (int64_t)m4_;
  digits[5] = (uint32_t)acc;
}

/** @brief All seven words zero. */
ModulusDecoder::ModulusDecoder()
{
  m6_ = 0;
  m5_ = 0;
  m4_ = 0;
  m3_ = 0;
  m2_ = 0;
  m1_ = 0;
  m0_ = 0;
}
ModulusDecoder::ModulusDecoder(unsigned m0, unsigned m1, unsigned m2, unsigned m3, unsigned m4, unsigned m5, unsigned m6)
{
  m0_ = m0;
  m1_ = m1;
  m2_ = m2;
  m3_ = m3;
  m4_ = m4;
  m5_ = m5;
  m6_ = m6;
}

/**
 * @brief Convert six mixed-radix digits back to a bit string (the inverse of ModulusEncoder::progress()).
 * @param bits    receives m6_ bits, one per byte (0 or 1), least significant bit first
 * @param digits  digits[0 .. 5]
 * @note  The value is built as a 64-bit unsigned number and unpacked with an arithmetic shift, so bits beyond bit 63 repeat its sign.
 */
void ModulusDecoder::progress(unsigned char* bits, unsigned* digits)
{
  uint64_t acc = digits[5];
  acc = acc * m4_ + digits[4];
  acc = acc * m3_ + digits[3];
  acc = acc * m2_ + digits[2];
  acc = acc * m1_ + digits[1];
  acc = acc * m0_ + digits[0];
  if (m6_ != 0)
  {
    int64_t s = (int64_t)acc;
    unsigned i = 0;
    do
    {
      bits[i] = (unsigned char)(s & 1);
      s >>= 1;
      i++;
    } while (m6_ > i);
  }
}

/* ================================================================================================ V90Phase2Info / V92Phase2Info */
static const char* const kPcmNames[2] = {"MU_LAW", "A_LAW"};

/* debug print of a value as sign / integer part / fraction digits (the same split as print_echo_coeffs()); `index` is printed first when `hasIndex` is set */
static inline void dbgValue(const char* fmt, int index, int hasIndex, LD x, LD scale)
{
  Split s = splitValue(x, scale);
  if (hasIndex)
    dsplibs_debug_printf(fmt, index, s.sign, s.ip, s.fp);
  else
    dsplibs_debug_printf(fmt, s.sign, s.ip, s.fp);
}

/* the debug block shared by the two printInfo() functions; the debug level is re-read before every print */
static void phase2DebugBlock(const char* pcmFmt, const char* rtdFmt, const char* maxFmt, const char* txFmt, unsigned pcmType, unsigned rtd, unsigned maxTx,
                             unsigned txPoint)
{
  if (dsplibs_debug_level > 1)
  {
    dsplibs_debug_printf(pcmFmt, pcmType == 1 ? "A_LAW" : "MU_LAW");
    if (dsplibs_debug_level > 1)
    {
      dsplibs_debug_printf(rtdFmt, rtd);
      if (dsplibs_debug_level > 1)
      {
        LD v = (LD)(int)(maxTx + 1u) * (LD)-0.5f; /* exact */
        dbgValue(maxFmt, 0, 0, v, (LD)10.0f);
        if (dsplibs_debug_level > 1)
          dsplibs_debug_printf(txFmt, txPoint == 1 ? "CodecOutput" : "DigitalModemTerminal");
      }
    }
  }
}
/* print the 21 L2 values (3 decimals) when debugging is enabled */
static void phase2L2Loop(const char* l2Fmt, const float* l2)
{
  for (int i = 0; i <= 20; i++)
  {
    if (dsplibs_debug_level > 1)
      dbgValue(l2Fmt, i, 1, (LD)l2[i], (LD)1000.0f);
  }
}

/**
 * @brief Load the phase 2 defaults from the parameter block.
 * @param p  V.90 parameters; the members phase2Info* (PHASE2_INFO_*) are read and `p` is kept
 * @note  arrA_, arrB_, l2_ and arrD_ are left unset; the owner sets them.
 */
V90Phase2Info::V90Phase2Info(V90Parameters* p)
{
  params_ = p;
  pcmType_ = p->phase2InfoAOrMu != 0;
  rtd_ = (unsigned)p->phase2InfoRtd;
  uinfo_ = (unsigned char)p->phase2InfoUinfo;
  maxTx_ = (unsigned char)p->phase2InfoMaxTxPower;
  txPoint_ = p->phase2InfoTxPowerMeasurePoint != 0;
}

/** @brief Reload the phase 2 fields from the parameter block given to the constructor (the same copy the constructor makes). */
void V90Phase2Info::setToDefault()
{
  V90Parameters* p = params_;
  pcmType_ = p->phase2InfoAOrMu != 0;
  rtd_ = (unsigned)p->phase2InfoRtd;
  uinfo_ = (unsigned char)p->phase2InfoUinfo;
  maxTx_ = (unsigned char)p->phase2InfoMaxTxPower;
  txPoint_ = p->phase2InfoTxPowerMeasurePoint != 0;
}

/** @brief Print the fields through the debug print (level > 1) and edprintf(), including the 21 L2 values; l2_ must be set. */
void V90Phase2Info::printInfo() const
{
  phase2DebugBlock("V90Phase2Info: pcmType = %s\r\n", "V90Phase2Info: rtd = %d\r\n", "V90Phase2Info: maxTxPower [dBm0]  = %c%d.%01d\r\n",
                   "V90Phase2Info: txPowerMeasurementPoint = %s\r\n", pcmType_, rtd_, maxTx_, txPoint_);
  edprintf("V90Phase2Info: Uinfo = %d\r\n", (int)uinfo_);
  phase2L2Loop("V90Phase2Info: L2[%d] = %c%d.%03d\r\n", l2_);
}

/**
 * @brief Load the phase 2 defaults from the parameter block.
 * @param p  V.92 parameters; the members v92Phase2Info*, v92NofFilterSections, v92MaxTotalNofCoeffs and v92MaxNofCoeffsInEachSection are read and `p` is kept
 * @note  Local capability flag set, remote flags and short-phase-2 local cleared; arrA_, arrB_, l2_ and arrD_ are left unset.
 */
V92Phase2Info::V92Phase2Info(V92Parameters* p)
{
  params_ = p;
  pcmType_ = p->v92Phase2InfoAOrMu != 0;
  rtd_ = (unsigned)p->v92Phase2InfoRtd;
  uinfo_ = (unsigned char)p->v92Phase2InfoUinfo;
  maxTx_ = (unsigned char)p->v92Phase2InfoMaxTxPower;
  shortPhase2Remote_ = 0;
  v92CapRemote_ = 0;
  shortPhase2Local_ = 0;
  v92CapLocal_ = 1;
  txPoint_ = p->v92Phase2InfoTxPowerMeasurePoint != 0;
  b14_ = (unsigned char)p->v92NofFilterSections;
  b15_ = (unsigned char)p->v92MaxTotalNofCoeffs;
  v90UseHighCarrier_ = 0;
  b16_ = (unsigned char)p->v92MaxNofCoeffsInEachSection;
}

/** @brief Reload the phase 2 fields from the parameter block given to the constructor (the same copy the constructor makes). */
void V92Phase2Info::setToDefault()
{
  V92Parameters* p = params_;
  pcmType_ = p->v92Phase2InfoAOrMu != 0;
  rtd_ = (unsigned)p->v92Phase2InfoRtd;
  uinfo_ = (unsigned char)p->v92Phase2InfoUinfo;
  maxTx_ = (unsigned char)p->v92Phase2InfoMaxTxPower;
  shortPhase2Remote_ = 0;
  v92CapRemote_ = 0;
  shortPhase2Local_ = 0;
  v92CapLocal_ = 1;
  txPoint_ = p->v92Phase2InfoTxPowerMeasurePoint != 0;
  b14_ = (unsigned char)p->v92NofFilterSections;
  b15_ = (unsigned char)p->v92MaxTotalNofCoeffs;
  v90UseHighCarrier_ = 0;
  b16_ = (unsigned char)p->v92MaxNofCoeffsInEachSection;
}

/** @brief Print the fields through the debug print (level > 1) and edprintf(), including the 21 L2 values; l2_ must be set. */
void V92Phase2Info::printInfo() const
{
  phase2DebugBlock("V92Phase2Info: pcmType = %s\r\n", "V92Phase2Info: rtd = %d\r\n", "V92Phase2Info: maxTxPower [dBm0]  = %c%d.%01d\r\n",
                   "V92Phase2Info: txPowerMeasurementPoint = %s\r\n", pcmType_, rtd_, maxTx_, txPoint_);
  edprintf("V92Phase2Info: ShortPhase2: local=%d , remote=%d\r\n", (int)shortPhase2Local_, (int)shortPhase2Remote_);
  edprintf("V92Phase2Info: v92Capabilities: local=%d , remote=%d\r\n", (int)v92CapLocal_, (int)v92CapRemote_);
  edprintf("V92Phase2Info: v90UseHighCarrier = %d\r\n", (int)v90UseHighCarrier_);
  edprintf("V92Phase2Info: Uinfo = %d\r\n", (int)uinfo_);
  phase2L2Loop("V92Phase2Info: L2[%d] = %c%d.%03d\r\n", l2_);
}

/* ================================================================================================ V90ConstellationPower */
unsigned V90ConstellationPower::averagePowerLimits[35] = {
    228735376u, 203804176u, 181710400u, 161900176u, 144288144u, 128595600u, 114661264u, 102171664u, 91087936u, 81144064u, 72318016u, 64448784u,
    57456400u,  51208336u,  45643536u,  40704400u,  36240400u,  32307856u,  28815424u,  25684624u,  22886656u, 20394256u, 18181696u, 16192576u,
    14440000u,  12873744u,  11478544u,  10214416u,  9120400u,   8133904u,   7246864u,   6451600u,   5740816u,  5112121u,  4549689u};

V90ConstellationPower::V90ConstellationPower() {}
V90ConstellationPower::~V90ConstellationPower() {}

/* accessors of a V90MappingParams block (32-bit layout): radix d_k at byte 0x604 + 4 k; code-point table k at 0x4 + 0x80 k
 * (point != CODEC_OUTPUT) or at 0x304 + 0x80 k (CODEC_OUTPUT) */
static inline unsigned mpRadix(const V90MappingParams* p, unsigned k) { return pw(p, 0x604 + 4 * k); }
static inline const unsigned char* mpTable(const V90MappingParams* p, V90TxPowerMeasurementPoint point, unsigned k)
{
  return (const unsigned char*)p + (point == V90_TX_POINT_CODEC_OUTPUT ? 0x304u : 0x4u) + 0x80u * k;
}

/**
 * @brief Compute the mixed-radix decomposition of M = 2^(w620 + w0 - 6) over the six radices of the block.
 * @param p  mapping parameters (words 0, 0x620 and the radices at 0x604 + 4 k)
 * @note  Fills M_, A_[0 .. 6], r_[0 .. 5] and D_[0 .. 5].  A zero radix divides by zero.
 */
void V90ConstellationPower::calcModulusParameters(V90MappingParams* p)
{
  unsigned s = pw(p, 0x620) + pw(p, 0) - 6u;
  M_ = (long long)((unsigned long long)1 << (s & 63u)); /* a 64-bit shift by s mod 64 */
  A_[0] = (long long)((unsigned long long)M_ - 1u);
  for (unsigned k = 0; k < 5; k++)
  {
    int64_t d = (int64_t)(uint64_t)mpRadix(p, k);
    r_[k] = (uint32_t)((int64_t)A_[k] % d);
    A_[k + 1] = (long long)((int64_t)((uint64_t)A_[k] - (uint64_t)r_[k]) / (int64_t)(uint64_t)mpRadix(p, k));
  }
  r_[5] = (uint32_t)A_[5]; /* the last digit takes the remaining quotient: no modulo */
  A_[6] = (long long)((int64_t)((uint64_t)A_[5] - (uint64_t)r_[5]) / (int64_t)(uint64_t)mpRadix(p, 5));

  D_[0] = 1.0;
  LD x = (LD)(uint64_t)mpRadix(p, 0);
  D_[1] = (double)x;
  x = x * (LD)(uint64_t)mpRadix(p, 1);
  D_[2] = (double)x;
  x = x * (LD)(uint64_t)mpRadix(p, 2);
  D_[3] = (double)x;
  x = x * (LD)(uint64_t)mpRadix(p, 3);
  D_[4] = (double)x;
  x = x * (LD)(uint64_t)mpRadix(p, 4);
  D_[5] = (double)x;
}

/** @brief Select code-point table k (0 .. 5) and its length n_ for the given measurement point; other k leave the object unchanged. */
void V90ConstellationPower::getConstellationInfo(V90MappingParams* p, V90TxPowerMeasurementPoint point, unsigned k)
{
  if (k <= 5)
  {
    tab_ = mpTable(p, point, k);
    n_ = mpRadix(p, k);
  }
}

/**
 * @brief Average transmit power of the constellation set selected by the block.
 * @param p      mapping parameters
 * @param point  which code-point tables (digital terminal / codec output) to use
 * @param pcm    companding law used to decode the code points
 * @return the sum over all code points of linear^2 * probability, divided by 6
 * @note  Calls calcModulusParameters() first and leaves tab_ / n_ at the last table.  The probability of a code point depends on whether its index
 *        is below, at or above the remainder r_[k] of its radix.
 */
float V90ConstellationPower::getPower(V90MappingParams* p, V90TxPowerMeasurementPoint point, PcmType pcm)
{
  double total = 0.0;
  calcModulusParameters(p);
  for (unsigned k = 0; k <= 5; k++)
  {
    tab_ = mpTable(p, point, k);
    n_ = mpRadix(p, k);
    for (unsigned j = 0; j < n_; j++)
    {
      double t2;
      unsigned r = r_[k];
      if (r > j)
      {
        double t = (double)((LD)D_[k] / (LD)(int64_t)M_);
        t2 = (double)((LD)(int64_t)((uint64_t)A_[k + 1] + 1u) * (LD)t);
      }
      else if (r == j)
      {
        LD s = ((LD)1 / (LD)(int64_t)M_) * (LD)D_[k];
        LD diff = (LD)(int64_t)((uint64_t)A_[k] - (uint64_t)A_[k + 1]);
        t2 = (double)((LD)1 - s * diff);
      }
      else
      {
        double t = (double)((LD)D_[k] / (LD)(int64_t)M_);
        t2 = (double)((LD)(int64_t)A_[k + 1] * (LD)t);
      }

      unsigned b = tab_[j] & 0x7fu;
      int v1, v2;
      if (pcm != 0)
      {
        v1 = alaw2linear((int)((b ^ 0xd5u) & 0xffu));
        v2 = alaw2linear((int)((b ^ 0xd5u) & 0xffu));
      }
      else
      {
        v1 = ulaw2linear((int)(~b & 0xffu));
        v2 = ulaw2linear((int)(~b & 0xffu));
      }

      LD prod = (LD)v2 * (LD)(double)v1; /* v1 is widened through double (exact) */
      prod = prod * (LD)t2;
      total = (double)(prod + (LD)total);
    }
  }
  return (float)((LD)total * (LD)0.16666666666666666);
}

/** @brief Index of the first limit, searched from the top (34) down, that is >= power; 0 if none. */
unsigned V90ConstellationPower::getPowerIndexForPower(float power)
{
  LD x = power;
  unsigned i = 34;
  if (!((LD)averagePowerLimits[34] >= x))
  {
    do
    {
      i--;
    } while (i != 0 && !((LD)averagePowerLimits[i] >= x)); /* a NaN power runs down to 0 */
  }
  return i;
}

/** @brief True when `index` is a valid index into averagePowerLimits[] (0 .. 34). */
bool V90ConstellationPower::isLegalPowerIndex(unsigned index) { return index <= 34u; }

/* ================================================================================================ V90SdDetector */
/**
 * @brief Create the detector with its 12-float history cleared.
 * @param a, b, c   energy, positive-correlation and negative-correlation thresholds
 * @param required  consecutive positive hits needed for a detection
 * @warning The history block comes from sysdep_malloc() and the result is not checked; a failed allocation is dereferenced.
 * @note  The members are stored in a fixed order, with the allocation in the middle; code that snapshots the object at allocation time sees the thresholds
 *        and `required` already set.
 */
V90SdDetector::V90SdDetector(float a, float b, float c, unsigned required)
{
  storeWord(&thA_, wordOf(a));
  n_ = 12;
  storeWord(&thB_, wordOf(b));
  required_ = required;
  storeWord(&thC_, wordOf(c));
  float* p = (float*)sysdep_malloc(0x30);
  hist_ = p;
  for (unsigned i = 0; n_ > i; i++)
    storeWord(&p[i], 0u); /* n_ is re-read on every pass */
  count_ = 0;
}

/** @brief Free the history block. */
V90SdDetector::~V90SdDetector()
{
  if (hist_)
    sysdep_free(hist_);
}

/** @brief Clear the history and the hit counter. */
void V90SdDetector::reset()
{
  unsigned n = n_;
  if (n > 0)
  {
    float* h = hist_;
    for (unsigned i = 0; n > i; i++)
      storeWord(&h[i], 0u);
  }
  count_ = 0;
}

/**
 * @brief Feed one sample.
 * @param x  input sample
 * @return 1 once the correlation ratio has exceeded thB_ for `required_` consecutive calls; -1 when it is below thC_; otherwise 0
 * @note  A = sum of h[i]^2 and B = sum of h[i] * h[i + 6] over i = 0 .. 5 are accumulated at extended precision.  When A < thA_ (too little energy) the
 *        hit counter is cleared and 0 is returned.  The ratio B / A is an extended-precision division (A == 0 gives inf / NaN, which falls through the
 *        comparisons the way the 32-bit code does).
 */
int V90SdDetector::process(float x)
{
  float* h = hist_;
  unsigned n = n_;
  /* shift the history by one cell with integer moves, oldest first; n is always 12 (for n < 2 nothing is shifted) */
  if (n >= 2)
    for (unsigned k = n - 1; k >= 1; k--)
      copyWord(&h[k], &h[k - 1]);
  storeWord(&h[0], wordOf(x));

  /* energy A and correlation B over 6 cells: two extended accumulators; the product of two floats is exact in extended precision */
  LD A = 0, B = 0;
  for (unsigned i = 0; i < 6; i++)
  {
    LD v = h[i];
    A = v * v + A;
    B = v * (LD)h[i + 6] + B;
  }

  if ((LD)thA_ > A)
  { /* not taken for NaN */
    count_ = 0;
    return 0;
  }

  LD q = B / A;
  if ((LD)thB_ >= q)
  { /* not taken for NaN */
    if (!((LD)thC_ > q))
    { /* taken for NaN */
      count_ = 0;
      return 0;
    }
    return -1;
  }

  unsigned c = count_ + 1;
  count_ = c;
  return c >= required_ ? 1 : 0;
}

/* ================================================================================================ V90RDetector */
/** @brief Keep `p`; every other member is left uninitialised until reset(). */
V90RDetector::V90RDetector(V90Parameters* p) { params_ = p; }
V90RDetector::~V90RDetector() {}

/**
 * @brief Clear the state and set the detection lengths.
 * @param a  length (in decisions) the pattern must persist for detectR() (rounded down to a multiple of 6) and detectRf() (multiple of 12)
 * @param b  the same for the negative-pattern detectors detectRNot() / detectRfNot()
 */
void V90RDetector::reset(unsigned a, unsigned b)
{
  cnt_ = 0;
  c0_ = 0;
  c1_ = 0;
  c2_ = 0;
  dir_ = 1;
  sr_ = 0;
  lim0_ = a / 6u * 6u;
  lim2_ = a / 12u * 12u;
  lim1_ = b / 6u * 6u;
  lim3_ = b / 12u * 12u;
}

/* shift one decision (x > 0 gives a 1 bit) into the 16-bit register */
static inline void srShift(unsigned short* sr, short x)
{
  unsigned v = (unsigned)*sr * 2u;
  if (x > 0)
    v |= 1u;
  *sr = (unsigned short)v;
}

/**
 * @brief Feed one decision; every 6th call evaluates the block of 6 decisions.
 * @param x  decision (positive = 1)
 * @return 1 once the pattern 111000 (dir_ = +1) or 000111 (dir_ = -1) has repeated for lim0_ decisions; otherwise 0
 */
int V90RDetector::detectR(short x)
{
  int ret = 0;
  srShift(&sr_, x);
  unsigned c = cnt_ + 1;
  if (c != 6)
  {
    cnt_ = c;
    return 0;
  }
  unsigned short sr = sr_;
  if (sr == 0x38)
  {
    c1_ = 0;
    c0_ += 6;
    if (c0_ == lim0_)
    {
      dir_ = 1;
      ret = 1;
    }
  }
  else if (sr == 7)
  {
    c0_ = 0;
    c1_ += 6;
    if (c1_ == lim0_)
    {
      dir_ = -1;
      ret = 1;
    }
  }
  else
  {
    c0_ = 0;
    c1_ = 0;
  }
  cnt_ = 0;
  sr_ = 0;
  return ret;
}

/**
 * @brief Feed one decision; every 6th call checks whether the block is the pattern opposite to dir_ (000111 when dir_ > 0, else 111000).
 * @param x  decision (positive = 1)
 * @return -1 once that pattern has repeated for lim1_ decisions; otherwise 0
 */
int V90RDetector::detectRNot(short x)
{
  int ret = 0;
  srShift(&sr_, x);
  unsigned c = cnt_ + 1;
  if (c != 6)
  {
    cnt_ = c;
    return 0;
  }
  unsigned sr = sr_;
  unsigned target = dir_ > 0 ? 7u : 0x38u;
  if (sr == target)
  {
    c2_ += 6;
    if (c2_ == lim1_)
      ret = -1;
  }
  else
  {
    c2_ = 0;
  }
  cnt_ = 0;
  sr_ = 0;
  return ret;
}

/**
 * @brief Feed one decision; every 12th call evaluates the block of 12 decisions.
 * @param x  decision (positive = 1)
 * @return 1 once the pattern 110011001100 (dir_ = +1) or 001100110011 (dir_ = -1) has repeated for lim2_ decisions; otherwise 0
 */
int V90RDetector::detectRf(short x)
{
  int ret = 0;
  srShift(&sr_, x);
  unsigned c = cnt_ + 1;
  if (c != 0xc)
  {
    cnt_ = c;
    return 0;
  }
  unsigned short sr = sr_;
  if (sr == 0xccc)
  {
    c1_ = 0;
    c0_ += 0xc;
    if (c0_ == lim2_)
    {
      dir_ = 1;
      ret = 1;
    }
  }
  else if (sr == 0x333)
  {
    c0_ = 0;
    c1_ += 0xc;
    if (c1_ == lim2_)
    {
      dir_ = -1;
      ret = 1;
    }
  }
  else
  {
    c0_ = 0;
    c1_ = 0;
  }
  cnt_ = 0;
  sr_ = 0;
  return ret;
}

/**
 * @brief Feed one decision; every 12th call checks whether the block is 001100110011 (dir_ > 0) or 110011001100 (dir_ <= 0).
 * @param x  decision (positive = 1)
 * @return -1 once that pattern has repeated for lim3_ decisions; otherwise 0
 */
int V90RDetector::detectRfNot(short x)
{
  int ret = 0;
  srShift(&sr_, x);
  unsigned c = cnt_ + 1;
  if (c != 0xc)
  {
    cnt_ = c;
    return 0;
  }
  unsigned sr = sr_;
  unsigned target = dir_ > 0 ? 0x333u : 0xccc;
  if (sr == target)
  {
    c2_ += 0xc;
    if (c2_ == lim3_)
      ret = -1;
  }
  else
  {
    c2_ = 0;
  }
  cnt_ = 0;
  sr_ = 0;
  return ret;
}

/* ================================================================================================ V90SpectralShapingFilter */
/** @brief Clear the coefficients and the state; N_ = 2. */
V90SpectralShapingFilter::V90SpectralShapingFilter()
{
  storeWord(&c3_, 0u);
  storeWord(&c2_, 0u);
  storeWord(&c1_, 0u);
  storeWord(&c0_, 0u);
  N_ = 2;
  storeWord(&s0_, 0u);
  storeWord(&s1_, 0u);
  storeWord(&s2_, 0u);
  storeWord(&s3_, 0u);
}

/** @brief Clear the filter state (s0 .. s3); the coefficients are kept. */
void V90SpectralShapingFilter::reset()
{
  storeWord(&s0_, 0u);
  storeWord(&s1_, 0u);
  storeWord(&s2_, 0u);
  storeWord(&s3_, 0u);
}

/** @brief Set the four coefficients (stored as float words). */
void V90SpectralShapingFilter::setFilterCoeff(float c0, float c1, float c2, float c3)
{
  storeWord(&c0_, wordOf(c0));
  storeWord(&c1_, wordOf(c1));
  storeWord(&c2_, wordOf(c2));
  storeWord(&c3_, wordOf(c3));
}

/* the filter state between samples, kept at extended precision, and one filter step; progress() and getMetricLD() share the same arithmetic */
struct SsfState
{
  LD s0, s1, s2, s3;
};
static inline void ssfStep(SsfState& st, LD c0, LD c1, LD c2, LD c3, short in)
{
  LD x = (LD)in;
  LD t1 = st.s0 * c2;
  LD t2 = st.s2 * c1;
  LD t3 = c0 * st.s1;
  LD t4 = st.s1 * c3;
  LD t5 = x - t1;
  LD t6 = t3 + t5;
  LD t7 = t6 - t4;
  LD t8 = t2 + t7;
  st.s3 = st.s3 + t8 * t8;
  st.s0 = x;
  st.s1 = t6;
  st.s2 = t8;
}

/**
 * @brief Run N_ input samples through the filter and update the state.
 * @param in  N_ samples
 * @note  Each step adds the square of the filter output to s3 (the energy accumulator).  The state is carried at extended precision across the N_ samples
 *        and rounded to float once at the end.
 */
void V90SpectralShapingFilter::progress(const short* in)
{
  unsigned n = N_;
  if (n == 0)
    return;
  SsfState st = {(LD)s0_, (LD)s1_, (LD)s2_, (LD)s3_};
  LD c0 = c0_, c1 = c1_, c2 = c2_, c3 = c3_;
  do
  {
    ssfStep(st, c0, c1, c2, c3, *in++);
    n--;
  } while (n != 0);
  s0_ = (float)st.s0;
  s1_ = (float)st.s1;
  s3_ = (float)st.s3;
  s2_ = (float)st.s2;
}

/**
 * @brief Energy metric: run n * N_ samples through a copy of the filter state and return the energy accumulator.
 * @param in  n * N_ input samples (the pointer keeps advancing across the blocks)
 * @param n   number of N_-sample blocks
 * @return s3 after the last sample, at extended precision; the object's own state is not modified.  N_ == 0 or n == 0 returns the current s3.
 */
long double V90SpectralShapingFilter::getMetricLD(const short* in, unsigned n) const
{
  SsfState st = {(LD)s0_, (LD)s1_, (LD)s2_, (LD)s3_};
  unsigned N = N_;
  LD c0 = c0_, c1 = c1_, c2 = c2_, c3 = c3_;
  for (unsigned b = 0; b < n && N > 0; b++)
    for (unsigned k = 0; k < N; k++)
      ssfStep(st, c0, c1, c2, c3, *in++);
  return st.s3;
}

/** @brief getMetricLD() returned as float; on a 32-bit build the extended value stays unrounded in st(0) (dsp_ret_ext_float). */
float V90SpectralShapingFilter::getMetric(const short* in, unsigned n) const { return dsp_ret_ext_float(getMetricLD(in, n)); }
