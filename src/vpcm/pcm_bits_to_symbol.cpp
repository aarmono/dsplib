/**
 * @file pcm_bits_to_symbol.cpp
 * @brief Transmit bit-to-symbol front ends of the V.90 and V.92 modulators: V92Transmitter, V90BitsToSymbol and V92BitsToSymbol.
 *
 * Declarations are in pcm_bits_to_symbol.h.  Callees: V90Mapper, V92ModulusEncoder, V92Precoder / V92PreFilter / V92ConvolutionEncoder,
 * sysdep_malloc / sysdep_free and dsplibs_debug_printf.  The file is C++ (no exceptions, no RTTI, freestanding headers only).
 *
 * Behaviour worth knowing:
 * - The three process() overloads of each BitsToSymbol class: (in, n&, out) maps the bits, queues the symbols, hands one block of `blk_` symbols to the
 *   caller, moves the rest to the front of the queue and writes the bit count of the NEXT call into `n`; (in, n) only queues; (n&, out) only drains.
 *   An overflow (more than `cap_` symbols queued) is clamped to `blk_` symbols (the rest is dropped) and reported as 2; a block asked for before `blk_`
 *   symbols are queued is reported as 3 (the queued symbols are handed over and the queue is emptied); `blk_` == 0 reports 1.  `first_` is cleared on
 *   every path.
 * - The bit count for the next call is ceil(d / 6) * w0_ (V.90; d = blk_ - fill_, plus extra_ while `first_` is set) or ceil(d / 12) * w0_ (V.92), but
 *   when d is an exact multiple it is (w0_ * d) / 6 or / 12 with the 32-bit product allowed to wrap.
 * - Every debug print tests the debug level (above 1) afresh.  The reset dump of V92Transmitter is a chain that stops at the first print whose test
 *   fails.
 * - V92Transmitter::reset() prints the gain from the transmitter's own copy (`gain_`).  The sign is '+' unless the value is <= 0 (NaN prints '+'); the
 *   integer part and the 7-digit fraction use truncating conversions (0x80000000 for NaN / out of range).
 * - V92Transmitter::process() converts each filtered sample as trunc16((long double)x * (long double)gain_): 0x8000 for NaN / out of range.
 * - The heap objects are allocated with sizeof(), because the classes holding pointers are larger on x86-64 (V90Mapper 0x728, V92Transmitter 0x80,
 *   V92Precoder 0xa8, V92PreFilter 0x20) and a fixed 32-bit size would overflow the block.
 * - `w0_` == 0 would overflow `buf_` in V92Transmitter::process() (it never reaches the flush threshold); constructors never test a malloc result.
 */
#include "pcm_bits_to_symbol.h"
#include "core/dsp_math.h"
#include "core/dsp_asm.h"
#include "core/dsp_host.h"

typedef long double LD;

/* ------------------------------------------------------------------------------------------------ helpers (static inline) */

#include "core/dsp_copyword.h"

#include "core/dsp_x87_conv.h"

#include "core/dsp_intmath.h"

#include "vpcm_split.h"
#include "vpcm_split_value.h"

#include "core/dsp_debug.h"

/**
 * @brief Number of data bits the next call must bring.
 * @param d    symbols still to queue
 * @param w0   data bits per frame
 * @param div  symbols per frame (6 for V.90, 12 for V.92)
 * @return (w0 * d) / div if d is an exact multiple of the frame (32-bit wrapping product), otherwise (d / div + 1) * w0
 */
static inline unsigned bitsFor(unsigned d, unsigned w0, unsigned div)
{
  unsigned q = d / div;
  if (d == q * div)
    return (w0 * d) / div;
  return (q + 1) * w0;
}

/** @brief Copy the first `n` queue entries to `dst`. */
static inline void copyShorts(short* dst, const short* src, unsigned n)
{
  for (unsigned i = 0; i < n; i++)
    dst[i] = src[i];
}

/** @brief Move the entries from index `blk` on to the front of the queue.  @return the new fill count (0 if `fill` <= `blk`) */
static inline unsigned compact(short* buf, unsigned fill, unsigned blk)
{
  unsigned k = 0;
  if (fill > blk)
  {
    for (unsigned j = blk; j < fill; j++)
      buf[k++] = buf[j];
  }
  return k;
}

/** @brief Debug dump of one float array of V92Transmitter::reset(); the count and the array pointer are re-read for every entry. */
static inline void dumpFloats(const char* fmt, float* const* arr, const unsigned* cnt)
{
  for (unsigned i = 0; i < *cnt; i++)
  {
    if (dbgOn())
    {
      const float* a = *arr;
      Split s = splitValue((LD)a[i], (LD)10000000.0);
      dsplibs_debug_printf(fmt, i, s.sign, s.ip, s.fp);
    }
  }
}

static const char kSep[] = "---------------------------\r\n";

/* ================================================================================================ V92Transmitter */

/**
 * @brief Allocate the 0x50-byte bit buffer, the modulus encoder, the flag byte, the convolutional encoder, the precoder and the pre-filter (both 0x140).
 * @warning No allocation result is tested.
 */
V92Transmitter::V92Transmitter()
{
  buf_ = (unsigned char*)sysdep_malloc(0x50);
  fill_ = 0;
  w0_ = 0;
  V92ModulusEncoder* m = (V92ModulusEncoder*)sysdep_malloc(sizeof(V92ModulusEncoder));
  new (m) V92ModulusEncoder();
  me_ = m;
  unsigned char* fl = (unsigned char*)sysdep_malloc(1);
  *fl = 0;
  flag_ = fl;
  V92ConvolutionEncoder* c = (V92ConvolutionEncoder*)sysdep_malloc(sizeof(V92ConvolutionEncoder));
  new (c) V92ConvolutionEncoder();
  ce_ = c;
  V92Precoder* pr = (V92Precoder*)sysdep_malloc(sizeof(V92Precoder));
  new (pr) V92Precoder(0x140);
  pre_ = pr;
  V92PreFilter* pf = (V92PreFilter*)sysdep_malloc(sizeof(V92PreFilter));
  new (pf) V92PreFilter(0x140);
  pf_ = pf;
}

/** @brief Free the buffers and destroy the encoders; the precoder pointer is cleared after its destruction. */
V92Transmitter::~V92Transmitter()
{
  if (buf_ != 0)
    sysdep_free(buf_);
  if (me_ != 0)
    sysdep_free(me_);
  if (flag_ != 0)
    sysdep_free(flag_);
  if (pre_ != 0)
  {
    pre_->~V92Precoder();
    sysdep_free(pre_);
    pre_ = 0;
    COMPILER_BARRIER(); /* keeps -flifetime-dse from deleting the store above */
  }
  if (pf_ != 0)
  {
    pf_->~V92PreFilter();
    sysdep_free(pf_);
  }
  if (ce_ != 0)
  {
    ce_->~V92ConvolutionEncoder();
    sysdep_free(ce_);
  }
}

/**
 * @brief Load the mapping parameters into the transmitter and its sub-blocks, dump them at debug level 2, and clear the running state.
 * @param p  mapping parameters: gain, bits per group (`w0_`), coefficient arrays and counts for the precoder and pre-filter, trellis type
 */
void V92Transmitter::reset(V92MappingParams* p)
{
  /* parameters into the sub-blocks */
  copyWord(&gain_, &p->gain_);
  w0_ = p->w0_;
  me_->reset(p);
  *flag_ = 0;
  pre_->reset(p);
  pre_->setCoefficients(p->arr_[0], p->arr_[1], p->cnt_[0], p->cnt_[1]);
  pf_->reset();
  pf_->setCoefficients(p->arr_[2], p->arr_[3], p->cnt_[2], p->cnt_[3]);
  ce_->reset(p->trellis_);

  /* debug dump of the general parameters; the first failing level test skips the rest of the chain */
  if (dbgOn())
  {
    do
    {
      dsplibs_debug_printf("On V92 Transmitter reset, here are modulation parameters:\r\n");
      if (!dbgOn())
        break;
      dsplibs_debug_printf("================== General ==================\r\n");
      if (!dbgOn())
        break;
      Split s = splitValue((LD)gain_, (LD)10000000.0);
      dsplibs_debug_printf("Gain = %c%d.%07d\r\n", s.sign, s.ip, s.fp);
      if (!dbgOn())
        break;
      dsplibs_debug_printf("K = %d\r\n", w0_);
      if (!dbgOn())
        break;
      dsplibs_debug_printf("trellisType = %d\r\n", p->trellis_);
      if (!dbgOn())
        break;
      dsplibs_debug_printf("extendEu = %d\r\n", p->extendEu_);
      if (!dbgOn())
        break;
      dsplibs_debug_printf("m0 = %d\r\n", p->m_[0]);
      if (!dbgOn())
        break;
      dsplibs_debug_printf("m1 = %d\r\n", p->m_[1]);
      if (!dbgOn())
        break;
      dsplibs_debug_printf("m2 = %d\r\n", p->m_[2]);
      if (!dbgOn())
        break;
      dsplibs_debug_printf("m3 = %d\r\n", p->m_[3]);
      if (!dbgOn())
        break;
      dsplibs_debug_printf("m4 = %d\r\n", p->m_[4]);
      if (!dbgOn())
        break;
      dsplibs_debug_printf("m5 = %d\r\n", p->m_[5]);
      if (!dbgOn())
        break;
      dsplibs_debug_printf("m6 = %d\r\n", p->m_[6]);
      if (!dbgOn())
        break;
      dsplibs_debug_printf("m7 = %d\r\n", p->m_[7]);
      if (!dbgOn())
        break;
      dsplibs_debug_printf("m8 = %d\r\n", p->m_[8]);
      if (!dbgOn())
        break;
      dsplibs_debug_printf("m9 = %d\r\n", p->m_[9]);
      if (!dbgOn())
        break;
      dsplibs_debug_printf("m10 = %d\r\n", p->m_[10]);
      if (!dbgOn())
        break;
      dsplibs_debug_printf("m11 = %d\r\n", p->m_[11]);
      if (!dbgOn())
        break;
      dsplibs_debug_printf("================== Pre Coder ================\r\n");
      if (!dbgOn())
        break;
      dsplibs_debug_printf("lz1 = %d\r\n", p->cnt_[0]);
      if (!dbgOn())
        break;
      dsplibs_debug_printf("lp1 = %d\r\n", p->cnt_[1]);
    } while (0);
  }

  /* precoder coefficients */
  if (p->cnt_[0] != 0)
  {
    if (dbgOn())
      dsplibs_debug_printf(kSep);
    dumpFloats("z1[%d] = %c%d.%07d\r\n", &p->arr_[0], &p->cnt_[0]);
    if (dbgOn())
      dsplibs_debug_printf(kSep);
  }
  if (p->cnt_[1] != 0)
  {
    if (dbgOn())
      dsplibs_debug_printf(kSep);
    dumpFloats("p1[%d] = %c%d.%07d\r\n", &p->arr_[1], &p->cnt_[1]);
    if (dbgOn())
      dsplibs_debug_printf(kSep);
  }

  /* pre-filter */
  if (dbgOn())
  {
    do
    {
      dsplibs_debug_printf("================== Pre Filter ===============\r\n");
      if (!dbgOn())
        break;
      dsplibs_debug_printf("lz2 = %d\r\n", p->cnt_[2]);
      if (!dbgOn())
        break;
      dsplibs_debug_printf("lp2 = %d\r\n", p->cnt_[3]);
    } while (0);
  }
  if (p->cnt_[2] != 0)
  {
    if (dbgOn())
      dsplibs_debug_printf(kSep);
    dumpFloats("z2[%d] = %c%d.%07d\r\n", &p->arr_[2], &p->cnt_[2]);
    if (dbgOn())
      dsplibs_debug_printf(kSep);
  }
  if (p->cnt_[3] != 0)
  {
    if (dbgOn())
      dsplibs_debug_printf(kSep);
    dumpFloats("p2[%d] = %c%d.%07d\r\n", &p->arr_[3], &p->cnt_[3]);
    if (dbgOn())
      dsplibs_debug_printf(kSep);
  }

  par_ = 0;
  fill_ = 0;
}

/**
 * @brief Encode data bits into symbols.
 * @param bits    `n` data bits, one per byte
 * @param n       number of bits
 * @param out     receives twelve symbols per completed group of `w0_` bits
 * @param outCnt  receives the number of symbols written
 * @warning `out` is not bounds checked; it must hold 12 * ((fill_ + n) / w0_) entries.  `w0_` must be non-zero and at most 0x50 (set by reset()).
 */
void V92Transmitter::process(unsigned char* bits, unsigned n, short* out, unsigned& outCnt)
{
  unsigned pos = 0;
  for (unsigned i = 0; i < n; i++)
  {
    buf_[fill_] = bits[i];
    fill_ = fill_ + 1;
    if (fill_ < w0_)
      continue;

    /* a full group: modulus encoding, three precoder / trellis passes of four symbols, pre-filter, gain */
    int state[4];
    float fb[12];
    float fl[12];
    me_->progress(buf_, digits_);
    for (unsigned j = 0; j <= 2; j++)
    {
      pre_->process(digits_, (int)j, par_, state, fb + 4 * j);
      par_ = ce_->process(state);
    }

    pf_->process(fb, fl);
    LD g = (LD)gain_;
    for (unsigned k = 0; k <= 11; k++)
      out[pos + k] = trunc16((LD)fl[k] * g);
    pos += 12;

    fill_ = fill_ - w0_;
  }
  outCnt = pos;
}

/* ================================================================================================ V90BitsToSymbol */

/**
 * @brief Create the mapper for `p` and the symbol queue.
 * @param cap  queue capacity in symbols
 * @param p    V.90 parameters
 * @note `w0_` and `extra_` are not initialised until reset(); the block size starts at 0.
 */
V90BitsToSymbol::V90BitsToSymbol(unsigned cap, V90Parameters* p)
{
  params_ = p;
  V90Mapper* m = (V90Mapper*)sysdep_malloc(sizeof(V90Mapper));
  new (m) V90Mapper(params_);
  mapper_ = m;
  buf_ = (short*)sysdep_malloc(2 * cap);
  cap_ = cap;
  fill_ = 0;
  blk_ = 0;
  first_ = 1;
}

/** @brief Destroy the mapper and free the queue. */
V90BitsToSymbol::~V90BitsToSymbol()
{
  if (mapper_ != 0)
  {
    mapper_->~V90Mapper();
    sysdep_free(mapper_);
  }
  if (buf_ != 0)
    sysdep_free(buf_);
}

/**
 * @brief Reset the mapper and the queue for a new connection.
 * @param mp   mapping parameters: bits per frame (`kBits_`), spectral shaping rate (`sr_`) and id (`id_`)
 * @param pcm  PCM law
 * @note `extra_` = id * 6 / sr when sr is non-zero, else 0; it is added to the first bit-count computation after the reset.
 */
void V90BitsToSymbol::reset(V90MappingParams* mp, PcmType pcm)
{
  mapper_->reset(mp, pcm);
  w0_ = mp->kBits_;
  unsigned K = mp->sr_;
  extra_ = K != 0 ? (mp->id_ * 6u) / K : 0u;
  fill_ = 0;
  blk_ = 0;
  first_ = 1;
}

/**
 * @brief Reset only the mapper (without spectral shaping) and the bits per frame.
 * @note The queue, block size, `extra_` and `first_` are not touched.
 */
void V90BitsToSymbol::resetNoSpectral(V90MappingParams* mp, PcmType pcm)
{
  mapper_->resetNoSpectral(mp, pcm);
  w0_ = mp->kBits_;
}

/** @brief Number of data bits needed to fill the rest of a block, counting `extra_` while `first_` is set; 0 if the queue already holds a block. */
unsigned V90BitsToSymbol::nofBitsForNextTime()
{
  unsigned r = 0;
  if (blk_ > fill_)
  {
    unsigned d = blk_ - fill_;
    if (first_ != 0)
      d += extra_;
    r = bitsFor(d, w0_, 6);
  }
  return r;
}

/** @brief Set the block size to `n` symbols and return the number of data bits needed to fill it (as nofBitsForNextTime()). */
unsigned V90BitsToSymbol::setSymbolsBlockSize(unsigned n)
{
  unsigned r = 0;
  blk_ = n;
  if (n > fill_)
  {
    unsigned d = n - fill_;
    if (first_ != 0)
      d += extra_;
    r = bitsFor(d, w0_, 6);
  }
  return r;
}

/**
 * @brief Map `n` bits, queue the symbols and deliver one block of `blk_` symbols.
 * @param in   data bits, one per byte
 * @param n    in: number of bits; out: bits the next call must bring
 * @param out  receives `blk_` symbols (fewer on underflow)
 * @return 0 ok, 1 block size not set, 2 queue overflow (queue clamped to one block), 3 underflow (all queued symbols delivered)
 * @bug The mapper writes its symbols to `buf_ + fill_` before the overflow test, so a call that queues more than `cap_ - fill_` symbols writes past the
 *      end of the queue and only then reports 2.  Callers size the bit count with nofBitsForNextTime(), so it does not happen with a correct caller.
 */
unsigned V90BitsToSymbol::process(unsigned char* in, unsigned& n, short* out)
{
  unsigned ret = 0;
  if (blk_ == 0)
  {
    ret = 1;
    if (dbgOn())
      dsplibs_debug_printf("V90BitsToSymbol - error: process called, SIZE_NOT_SET\r\n");
    if (first_ != 0)
      first_ = 0;
    return ret;
  }
  unsigned cnt;
  mapper_->process(in, n, buf_ + fill_, cnt);
  fill_ = fill_ + cnt;
  unsigned ncopy;
  if (fill_ >= blk_)
  {
    if (fill_ > cap_)
    {
      ret = 2;
      if (dbgOn())
        dsplibs_debug_printf("V90BitsToSymbol - error: process called, BUFFER_OVERFLOW\r\n");
      fill_ = blk_;
    }
    ncopy = blk_;
    copyShorts(out, buf_, ncopy);
  }
  else
  {
    ret = 3;
    if (dbgOn())
      dsplibs_debug_printf("V90BitsToSymbol - error: process called, BUFFER_UNDERFLOW\r\n");
    ncopy = fill_;
    copyShorts(out, buf_, ncopy);
    fill_ = 0;
  }
  fill_ = compact(buf_, fill_, blk_);
  unsigned nn = 0;
  if (blk_ > fill_)
  {
    unsigned d = blk_ - fill_;
    if (first_ != 0)
      d += extra_;
    nn = bitsFor(d, w0_, 6);
  }
  n = nn;
  if (first_ != 0)
    first_ = 0;
  return ret;
}

/**
 * @brief Map `n` bits and queue the symbols; nothing is delivered.
 * @return 0 ok, 1 block size not set, 2 queue overflow (the fill count is set to one block, so the queued symbols are dropped)
 * @bug As in the three-argument overload, the symbols are written before the overflow test.
 */
unsigned V90BitsToSymbol::process(unsigned char* in, unsigned n)
{
  unsigned ret = 0;
  if (blk_ == 0)
  {
    ret = 1;
    if (dbgOn())
      dsplibs_debug_printf("V90BitsToSymbol - error: process called, SIZE_NOT_SET\r\n");
    if (first_ != 0)
      first_ = 0;
    return ret;
  }
  unsigned cnt;
  mapper_->process(in, n, buf_ + fill_, cnt);
  fill_ = fill_ + cnt;
  if (fill_ > cap_)
  {
    ret = 2;
    if (dbgOn())
      dsplibs_debug_printf("V90BitsToSymbol - error: process called, BUFFER_OVERFLOW\r\n");
    fill_ = blk_;
  }
  if (first_ != 0)
    first_ = 0;
  return ret;
}

/**
 * @brief Deliver one block from the queue without mapping new bits.
 * @param n    receives the bits the next call must bring
 * @param out  receives `blk_` symbols (the queued ones on underflow)
 * @return 0 ok, 1 block size not set, 3 underflow
 */
unsigned V90BitsToSymbol::process(unsigned& n, short* out)
{
  unsigned ret = 0;
  if (blk_ == 0)
  {
    ret = 1;
    if (dbgOn())
      dsplibs_debug_printf("V90BitsToSymbol - error: process called, SIZE_NOT_SET\r\n");
    if (first_ != 0)
      first_ = 0;
    return ret;
  }
  if (fill_ >= blk_)
  {
    copyShorts(out, buf_, blk_);
  }
  else
  {
    ret = 3;
    if (dbgOn())
      dsplibs_debug_printf("V90BitsToSymbol - error: process called, BUFFER_UNDERFLOW\r\n");
    copyShorts(out, buf_, fill_);
    fill_ = 0;
  }
  fill_ = compact(buf_, fill_, blk_);
  unsigned nn = 0;
  if (blk_ > fill_)
  {
    unsigned d = blk_ - fill_;
    if (first_ != 0)
      d += extra_;
    nn = bitsFor(d, w0_, 6);
  }
  n = nn;
  if (first_ != 0)
    first_ = 0;
  return ret;
}

/* ================================================================================================ V92BitsToSymbol */

/**
 * @brief Create the V92Transmitter and the symbol queue.
 * @param cap  queue capacity in symbols
 * @param p    V.92 parameters
 * @note `w0_` is not initialised until reset(); the block size starts at 0.
 */
V92BitsToSymbol::V92BitsToSymbol(unsigned cap, V92Parameters* p)
{
  params_ = p;
  V92Transmitter* t = (V92Transmitter*)sysdep_malloc(sizeof(V92Transmitter));
  new (t) V92Transmitter();
  tx_ = t;
  buf_ = (short*)sysdep_malloc(2 * cap);
  cap_ = cap;
  fill_ = 0;
  blk_ = 0;
  first_ = 1;
}

/** @brief Destroy the transmitter and free the queue. */
V92BitsToSymbol::~V92BitsToSymbol()
{
  if (tx_ != 0)
  {
    tx_->~V92Transmitter();
    sysdep_free(tx_);
  }
  if (buf_ != 0)
    sysdep_free(buf_);
}

/** @brief Reset the transmitter from `mp`, take `w0_` from it, and empty the queue and the block size. */
void V92BitsToSymbol::reset(V92MappingParams* mp)
{
  tx_->reset(mp);
  unsigned w0 = mp->w0_;
  fill_ = 0;
  blk_ = 0;
  w0_ = w0;
  first_ = 1;
}

/** @brief Number of data bits needed to fill the rest of a block; 0 if the queue already holds a block. */
unsigned V92BitsToSymbol::nofBitsForNextTime()
{
  unsigned r = 0;
  if (blk_ > fill_)
    r = bitsFor(blk_ - fill_, w0_, 12);
  return r;
}

/** @brief Set the block size to `n` symbols and return the number of data bits needed to fill it. */
unsigned V92BitsToSymbol::setSymbolsBlockSize(unsigned n)
{
  unsigned r = 0;
  blk_ = n;
  if (n > fill_)
    r = bitsFor(n - fill_, w0_, 12);
  return r;
}

/**
 * @brief Encode `n` bits, queue the symbols and deliver one block of `blk_` symbols.
 * @param in   data bits, one per byte
 * @param n    in: number of bits; out: bits the next call must bring
 * @param out  receives `blk_` symbols (fewer on underflow)
 * @return 0 ok, 1 block size not set, 2 queue overflow (queue clamped to one block), 3 underflow (all queued symbols delivered)
 * @bug The transmitter writes its symbols to `buf_ + fill_` before the overflow test, so a call that produces more than `cap_ - fill_` symbols writes
 *      past the end of the queue and only then reports 2.  Callers size the bit count with nofBitsForNextTime(), so it does not happen with a correct caller.
 */
unsigned V92BitsToSymbol::process(unsigned char* in, unsigned& n, short* out)
{
  unsigned ret = 0;
  if (blk_ == 0)
  {
    ret = 1;
    if (dbgOn())
      dsplibs_debug_printf("V92BitsToSymbol - error: process called, SIZE_NOT_SET\r\n");
    if (first_ != 0)
      first_ = 0;
    return ret;
  }
  unsigned cnt;
  tx_->process(in, n, buf_ + fill_, cnt);
  fill_ = fill_ + cnt;
  unsigned ncopy;
  if (fill_ >= blk_)
  {
    if (fill_ > cap_)
    {
      ret = 2;
      if (dbgOn())
        dsplibs_debug_printf("V92BitsToSymbol - error: process called, BUFFER_OVERFLOW\r\n");
      fill_ = blk_;
    }
    ncopy = blk_;
    copyShorts(out, buf_, ncopy);
  }
  else
  {
    ret = 3;
    if (dbgOn())
      dsplibs_debug_printf("V92BitsToSymbol - error: process called, BUFFER_UNDERFLOW\r\n");
    ncopy = fill_;
    copyShorts(out, buf_, ncopy);
    fill_ = 0;
  }
  fill_ = compact(buf_, fill_, blk_);
  unsigned nn = 0;
  if (blk_ > fill_)
    nn = bitsFor(blk_ - fill_, w0_, 12);
  n = nn;
  if (first_ != 0)
    first_ = 0;
  return ret;
}

/**
 * @brief Encode `n` bits and queue the symbols; nothing is delivered.
 * @return 0 ok, 1 block size not set, 2 queue overflow (the fill count is set to one block, so the queued symbols are dropped)
 * @bug As in the three-argument overload, the symbols are written before the overflow test.
 */
unsigned V92BitsToSymbol::process(unsigned char* in, unsigned n)
{
  unsigned ret = 0;
  if (blk_ == 0)
  {
    ret = 1;
    if (dbgOn())
      dsplibs_debug_printf("V92BitsToSymbol - error: process called, SIZE_NOT_SET\r\n");
    if (first_ != 0)
      first_ = 0;
    return ret;
  }
  unsigned cnt;
  tx_->process(in, n, buf_ + fill_, cnt);
  fill_ = fill_ + cnt;
  if (fill_ > cap_)
  {
    ret = 2;
    if (dbgOn())
      dsplibs_debug_printf("V92BitsToSymbol - error: process called, BUFFER_OVERFLOW\r\n");
    fill_ = blk_;
  }
  if (first_ != 0)
    first_ = 0;
  return ret;
}

/**
 * @brief Deliver one block from the queue without encoding new bits.
 * @param n    receives the bits the next call must bring
 * @param out  receives `blk_` symbols (the queued ones on underflow)
 * @return 0 ok, 1 block size not set, 3 underflow
 */
unsigned V92BitsToSymbol::process(unsigned& n, short* out)
{
  unsigned ret = 0;
  if (blk_ == 0)
  {
    ret = 1;
    if (dbgOn())
      dsplibs_debug_printf("V92BitsToSymbol - error: process called, SIZE_NOT_SET\r\n");
    if (first_ != 0)
      first_ = 0;
    return ret;
  }
  if (fill_ >= blk_)
  {
    copyShorts(out, buf_, blk_);
  }
  else
  {
    ret = 3;
    if (dbgOn())
      dsplibs_debug_printf("V92BitsToSymbol - error: process called, BUFFER_UNDERFLOW\r\n");
    copyShorts(out, buf_, fill_);
    fill_ = 0;
  }
  fill_ = compact(buf_, fill_, blk_);
  unsigned nn = 0;
  if (blk_ > fill_)
    nn = bitsFor(blk_ - fill_, w0_, 12);
  n = nn;
  if (first_ != 0)
    first_ = 0;
  return ret;
}
