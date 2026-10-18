/**
 * @file v90_mapper.cpp
 * @brief V.90 / V.92 bits <-> symbol mapper pair: V90SignBitsExtractor, V90Mapper (transmit) and V90Demapper (receive).
 *
 * Defines the classes declared in v90_mapper.h.  Callees from other files: the differential encoders / decoders (dsp_int_templates.h), ModulusEncoder
 * and ModulusDecoder::progress() and the G.711 decoders ulaw2linear() / alaw2linear(), V90SpectralShaper (v90_spectral_shaper.cpp), and
 * V90AutoDigitalImpDetector, whose linear-mapping and cumulative arrays are read and written through flat pointers (the code byte of a list can run
 * past a row).  The debug output goes through edprintf() and dsplibs_debug_printf().
 *
 * Behaviour worth knowing before changing anything:
 * - V90SignBitsExtractor::process(): the action state machine is 0 = first group (action_ = in[0] != 0, applied the same), 1 = second group
 *   (action_ = (in[0] == 0), applied pattern 3 when in[0] == 0 else 2); for any other action_ value the stored value itself is applied.
 * - V90Mapper::reset() and V90Demapper::reset() load K_ from the mapping parameters' sr_ and the spectral-shaper delay from id_; the mapper stores
 *   spf_ = 0 for K_ == 0, the demapper leaves spf_ alone.  resetNoSpectral() of both classes computes nbits_ with the OLD skip_ and touches neither
 *   K_, spf_ nor skip_.
 * - The mapper's pointer arithmetic on (spf_ - 1) * k is done in 32 bits (a cast through int).
 * - The level tables are filled with loop bounds re-read from the object (cnt_[i]); they must not exceed 128 (64 in the demapper's alternate-RBS mode,
 *   which doubles the count), otherwise a table row overflows into the next.
 * - V90Demapper::hardDecision(): |x| is formed in an int and narrowed to a short in two places (so -32768 stays -32768); the level search finds the
 *   last index whose level is not below |x| (limit cnt_ - 1, or 2 * cnt_ - 1 in a phase of alternate RBS levels); the lower neighbour is chosen only
 *   when strictly closer; the error histogram counts only when the debug parameter is set and the delay has run out.
 * - V90Demapper::updateConstelation() and the tail of linearMappingStudy() are one sequence (rebuildLevels()): level = trunc16(1.0 / count * sum + 0.5)
 *   with all intermediates in extended precision, truncated to 16 bits (NaN or out of range gives 0x8000).
 * - linearMappingStudy() compares 0.4 * (neighbour spacing) with |d| = ||a| - |b|| in extended precision, accumulates |a| into the detector's cumSum_
 *   through a float and counts.
 * - V90Demapper::process() returns 1 (and compacts the queues) whenever six or more decisions are queued, also when no frame could be decoded.
 */

#include "v90_mapper.h"
#include "core/dsp_host.h"
#include "v34/v34_api.h"

extern "C"
{
}

typedef long double LD;

/* ------------------------------------------------------------------------------------------------ helpers (static inline) */

#include "core/dsp_x87_conv.h"

/*
 * The four float parameters of the V90MappingParams go to V90SpectralShaper::reset() / resetSSFilter() as 32-bit WORDS: a `float` argument would be
 * loaded and stored through the x87 stack in 32-bit code, which quiets a signalling NaN, so the shaper is called through an asm-label declaration of
 * the very same symbol with word arguments.  On 32-bit x86 (cdecl) the four bytes of a float argument are one stack word, so the callee sees exactly
 * the same call.  On x86-64 a float travels in an SSE register; moving the word through memcpy keeps the payload.
 */
#if defined(__x86_64__)
typedef float ShW;
static inline ShW shW(const float* src)
{
  ShW f;
  __builtin_memcpy(&f, src, sizeof f);
  return f;
}
#else
typedef uint32_t ShW;
/* an integer word: no FPU load, nothing to keep out of a register */
static inline ShW shW(const float* src)
{
  uint32_t w;
  __builtin_memcpy(&w, src, sizeof w);
  return w;
}
#endif
extern "C" void shaper_reset_words(V90SpectralShaper*, unsigned, unsigned, ShW, ShW, ShW, ShW) __asm__("_ZN17V90SpectralShaper5resetEjjffff");
extern "C" void shaper_resetSSFilter_words(V90SpectralShaper*, ShW, ShW, ShW, ShW) __asm__("_ZN17V90SpectralShaper13resetSSFilterEffff");

/* The arrays of the V90AutoDigitalImpDetector are indexed unchecked (the code byte of a list can run past a row), by flat pointer. */
/** @brief The detector's linear mapping, [6][128] 16-bit levels. */
static inline const int16_t* lmP(const V90AutoDigitalImpDetector* a)
{
  return (const int16_t*)((const char*)a + offsetof(V90AutoDigitalImpDetector, linearMapping_));
}
/** @brief The detector's alternate-RBS linear mapping, [6][128] 16-bit levels. */
static inline const int16_t* lmAltP(const V90AutoDigitalImpDetector* a)
{
  return (const int16_t*)((const char*)a + offsetof(V90AutoDigitalImpDetector, linearMappingAlt_));
}
/** @brief The detector's cumulative amplitude sums, floats, indexed phase * 128 + level index. */
static inline float* cumSumP(V90AutoDigitalImpDetector* a) { return (float*)((char*)a + offsetof(V90AutoDigitalImpDetector, cumSum_)); }
/** @brief The detector's cumulative counts, indexed like cumSumP(). */
static inline uint32_t* cumCountP(V90AutoDigitalImpDetector* a) { return (uint32_t*)((char*)a + offsetof(V90AutoDigitalImpDetector, cumCount_)); }
/** @brief Element @p i of the detector's alternate-RBS phase flags (nonzero: that frame position has the alternate level set). */
static inline int16_t altRbsOf(const V90AutoDigitalImpDetector* a, int i)
{
  return *((const int16_t*)((const char*)a + offsetof(V90AutoDigitalImpDetector, altRbsPhase_)) + i);
}

/** @brief 16-bit absolute value of an int, narrowed to a short (-32768 stays -32768). */
static inline int absShort(int v) { return (int)(short)(v < 0 ? -v : v); }

/**
 * @brief Apply the sign pattern of one frame position to @p n bits (inlined form of V90SignBitsExtractor::applyFrameAction()).
 * @param action  0 keep, 1 negate (result = bit is zero), 2 negate even-indexed bits, 3 negate odd-indexed bits; any other value stores nothing
 */
static inline void applyFrame(unsigned n, int action, const unsigned char* in, unsigned char* out)
{
  for (unsigned i = 0; i < n; i++)
  {
    switch (action)
    {
    case 0:
      out[i] = in[i];
      break;
    case 1:
      out[i] = (unsigned char)(in[i] == 0);
      break;
    case 2:
      out[i] = (i & 1u) ? in[i] : (unsigned char)(in[i] == 0);
      break;
    case 3:
      out[i] = (i & 1u) ? (unsigned char)(in[i] == 0) : in[i];
      break;
    default:
      break;
    }
  }
}

/**
 * @brief Fill the mapper's level tables from the code points of @p mp (reset / resetNoSpectral).
 *
 * levels_[i * 128 + j] = ulaw2linear(~(code & 0x7f)) (@p pcm == 0) or alaw2linear((code & 0x7f) ^ 0xd5) for the j-th code of list A of constellation i;
 * the rest of the row is zero.  Also copies the six counts into cnt_.
 * @warning cnt_[i] > 128 writes past the row (past the table for i == 5); the mapping parameters keep it <= 128.
 */
static inline void mapperLevels(V90Mapper* t, const V90MappingParams* mp, PcmType pcm)
{
  for (unsigned i = 0; i <= 5; i++)
  {
    t->cnt_[i] = mp->cnt_[i];
    unsigned j = 0;
    while (j < t->cnt_[i])
    {
      unsigned char code = mp->listA_[i][j];
      int lin;
      if (pcm == 0)
        lin = ulaw2linear((unsigned char)~(code & 0x7f));
      else
        lin = alaw2linear((code & 0x7f) ^ 0xd5);
      t->levels_[i * 128 + j] = (unsigned short)lin;
      j++;
    }
    for (j = t->cnt_[i]; j <= 0x7f; j++)
      t->levels_[i * 128 + j] = 0;
  }
}

/** @brief Load the six constellation sizes and the bit count into the mapper's ModulusEncoder. */
static inline void setModulusEncoder(V90Mapper* t)
{
  t->me_.m0_ = t->cnt_[0];
  t->me_.m1_ = t->cnt_[1];
  t->me_.m2_ = t->cnt_[2];
  t->me_.m3_ = t->cnt_[3];
  t->me_.m4_ = t->cnt_[4];
  t->me_.m5_ = t->cnt_[5];
  t->me_.m6_ = t->nbits_;
}
/** @brief Load the six constellation sizes and the bit count into the demapper's ModulusDecoder. */
static inline void setModulusDecoder(V90Demapper* t)
{
  t->md_.m0_ = t->cnt_[0];
  t->md_.m1_ = t->cnt_[1];
  t->md_.m2_ = t->cnt_[2];
  t->md_.m3_ = t->cnt_[3];
  t->md_.m4_ = t->cnt_[4];
  t->md_.m5_ = t->cnt_[5];
  t->md_.m6_ = t->nbits_;
}

/**
 * @brief Fill the demapper's level table from the detector's linear mappings (reset / resetNoSpectral).
 *
 * A plain phase copies mapping[code] for every code of list A.  A phase of alternate RBS levels stores, for every code, the larger then the smaller of
 * its two mappings (normal and alternate), so the phase has twice as many levels, in descending pairs.  Also copies the six counts into cnt_.
 * @warning the alternate phases write 2 * cnt levels into a 128-entry row: cnt must not exceed 64 there.
 */
static inline void demapperLevels(V90Demapper* t, const V90MappingParams* mp)
{
  V90AutoDigitalImpDetector* adid = t->adid_;
  for (unsigned i = 0; i <= 5; i++)
  {
    unsigned cnt = mp->cnt_[i];
    unsigned base = i * 128u;
    t->cnt_[i] = cnt;
    if (altRbsOf(adid, (int)i) == 0)
    {
      for (unsigned j = 0; j < cnt; j++)
        t->lev_[base + j] = lmP(adid)[base + mp->listA_[i][j]];
    }
    else
    {
      short k = 0;
      for (unsigned j = 0; j < cnt; j++)
      {
        unsigned code = mp->listA_[i][j];
        unsigned short a = (unsigned short)lmP(adid)[base + code];
        unsigned short b = (unsigned short)lmAltP(adid)[base + code];
        short k1 = (short)(k + 1);
        if ((short)a > (short)b)
        {
          t->lev_[base + k] = (short)a;
          t->lev_[base + k1] = (short)b;
        }
        else
        {
          t->lev_[base + k] = (short)b;
          t->lev_[base + k1] = (short)a;
        }
        k = (short)(k1 + 1);
      }
    }
  }
}

/**
 * @brief The level estimator (updateConstelation(); also the tail of linearMappingStudy()).
 *
 * lev_[i * 128 + j] = trunc16(1.0 / count * sum + 0.5) for every level whose cumulative count is nonzero; a phase of alternate RBS levels has twice
 * as many levels.  The loop variables are 16-bit.
 */
static inline void rebuildLevels(V90Demapper* t)
{
  V90AutoDigitalImpDetector* adid = t->adid_;
  for (unsigned short i = 0; i <= 5; i = (unsigned short)(i + 1))
  {
    unsigned short n = altRbsOf(adid, i) != 0 ? (unsigned short)(t->cnt_[i] * 2u) : (unsigned short)t->cnt_[i];
    for (unsigned short j = 0; j < n; j = (unsigned short)(j + 1))
    {
      unsigned idx = (unsigned)i * 128u + j;
      unsigned c = cumCountP(adid)[idx];
      if (c != 0)
      {
        LD v = (LD)1.0L / (LD)(uint64_t)c;
        v = v * (LD)cumSumP(adid)[idx];
        v = v + 0.5L;
        t->lev_[idx] = trunc16(v);
      }
    }
  }
}

/** @brief Clear the cumulative sum and count of every (phase, ucode) of the detector (16-bit loop variables). */
static inline void clearAllCumulative(V90AutoDigitalImpDetector* adid)
{
  for (short i = 0; i <= 5; i = (short)(i + 1))
    for (short j = 0; j <= 0x7f; j = (short)(j + 1))
      adid->clearCamulativeVal(i, j);
}

/**
 * @brief Body of V90Demapper's constructor (kept as a helper: the initializer list only builds the sub-objects).
 * @warning the two sysdep_malloc() results are not checked.
 */
static inline void demapperInit(V90Demapper* t, unsigned cap, V90Parameters* p, V90AutoDigitalImpDetector* adid)
{
  t->adid_ = adid;
  t->digits_ = (unsigned*)sysdep_malloc(cap * 4u);
  t->bits_ = (unsigned char*)sysdep_malloc(cap);
  t->cap_ = cap;
  t->wr_ = 0;
  t->rd_ = 0;
  t->phase_ = 0;
  t->spf_ = 0;
  t->K_ = 0;
  t->skip_ = 0;
  t->nbits_ = 0;
  t->w0_ = 0;
  for (unsigned i = 0; i <= 5; i++)
    t->cnt_[i] = 0;
  t->histNo_ = 0;
  t->params_ = p;
}

/* ================================================================================================ V90SignBitsExtractor */

V90SignBitsExtractor::V90SignBitsExtractor() : sde_(), pdd_(6) { action_ = 0; }

V90SignBitsExtractor::~V90SignBitsExtractor() {}

void V90SignBitsExtractor::reset(unsigned period, unsigned action)
{
  period_ = period;
  sde_.state_ = 0;
  n_ = period != 0 ? 6u / period : 0u;
  pdd_.reset(n_, 0);
  action_ = action;
}

void V90SignBitsExtractor::applyFrameAction(ACTIONS action, unsigned char* in, unsigned char* out) { applyFrame(n_, (int)action, in, out); }

void V90SignBitsExtractor::process(unsigned char* in, unsigned char* out)
{
  unsigned a = action_;
  int applied;

  /* choose the sign pattern: the first two groups after reset() derive it from the first bit, afterwards the stored value is used as is */
  if (a == 0)
  {
    applied = in[0] != 0;
    action_ = (unsigned)applied;
  }
  else if (a == 1)
  {
    unsigned z = in[0] == 0;
    applied = z ? 3 : 2;
    action_ = z;
  }
  else
  {
    applied = (int)a;
  }

  /* undo the pattern, the parallel differential coding and (odd positions) the serial coding; drop the first bit */
  applyFrame(n_, applied, in, buf_);
  pdd_.process(buf_, buf_);
  for (unsigned i = 0; i < n_; i++)
    if (i & 1u)
      buf_[i] = sde_.process(buf_[i]);
  for (unsigned j = 1; j < n_; j++)
    out[j - 1] = buf_[j];
}

/* ================================================================================================ V90Mapper */

V90Mapper::V90Mapper(V90Parameters* p)
{
  sde_.state_ = 0;
  buf_ = (unsigned char*)sysdep_malloc(0x50);
  fill_ = 0;
  depth_ = 0;
  spf_ = 0;
  K_ = 0;
  skip_ = 0;
  nbits_ = 0;
  w0_ = 0;
  for (unsigned i = 0; i <= 5; i++)
    cnt_[i] = 0;
  params_ = p;
}

V90Mapper::~V90Mapper()
{
  if (buf_ != 0)
    sysdep_free(buf_);
}

void V90Mapper::reset(V90MappingParams* mp, PcmType pcm)
{
  unsigned w0 = mp->kBits_;
  unsigned K = mp->sr_;

  /* frame geometry: w0 bits per frame, the first 6 - K are sign bits */
  w0_ = w0;
  K_ = K;
  spf_ = K != 0 ? 6u / K : 0u;
  skip_ = 6u - K;
  nbits_ = w0 - (6u - K);

  /* level tables and, with spectral shaping, the shaper (its delay in groups becomes depth_) */
  mapperLevels(this, mp, pcm);
  if (K_ != 0)
  {
    shaper_reset_words(&shaper_, mp->id_, mp->sr_, shW(&mp->shaperA1_), shW(&mp->shaperA2_), shW(&mp->shaperB1_), shW(&mp->shaperB2_));
    depth_ = mp->id_;
  }
  else
  {
    depth_ = 0;
  }
  setModulusEncoder(this);

  z700_ = 0;
  sde_.state_ = 0;
  fill_ = 0;
}

void V90Mapper::resetNoSpectral(V90MappingParams* mp, PcmType pcm)
{
  unsigned skip = skip_;
  unsigned w0 = mp->kBits_;
  w0_ = w0;
  nbits_ = w0 - skip;
  mapperLevels(this, mp, pcm);
  setModulusEncoder(this);
  sde_.state_ = 0;
  shaper_resetSSFilter_words(&shaper_, shW(&mp->shaperA1_), shW(&mp->shaperA2_), shW(&mp->shaperB1_), shW(&mp->shaperB2_));
  z700_ = 0;
}

/** @warning `buf_` holds 0x50 bytes and `fill_` is not checked against it: a frame size w0_ above 0x50 overflows the buffer.  `out` is unchecked. */
void V90Mapper::process(unsigned char* in, unsigned n, short* out, unsigned& outCnt)
{
  unsigned outPos = 0;
  for (unsigned i = 0; i < n; i++)
  {
    buf_[fill_] = in[i];
    fill_ = fill_ + 1;
    if (fill_ < w0_)
      continue;

    /* one frame: bits -> six digits -> six levels */
    me_.progress(buf_ + skip_, idx_);
    for (unsigned j = 0; j < 6; j++)
      lev_[j] = levels_[(j << 7) + idx_[j]];

    /* signs: differential coding of the leading bits, or the spectral shaper over K_ groups */
    if (K_ == 0)
    {
      for (unsigned j = 0; j < 6; j++)
      {
        unsigned char r = sde_.process(buf_[j]);
        unsigned short v = lev_[j];
        sgn_[j] = r;
        if (r == 0)
          v = (unsigned short)(0u - v);
        sym_[j] = v;
      }
    }
    else
    {
      unsigned k = 0;
      do
      {
        shaper_.process((short*)lev_ + spf_ * k, buf_ + (int)((spf_ - 1u) * k), (short*)sym_ + spf_ * k);
        k++;
      } while (K_ > k);
    }

    /* output: all six symbols, or - while the shaper delay depth_ is pending - only the symbols after the delayed groups */
    if (depth_ == 0)
    {
      for (unsigned c = 0; c <= 5; c++)
        out[outPos + c] = (short)sym_[c];
      outPos += 6;
    }
    else if (depth_ < K_)
    {
      unsigned m = spf_ * depth_;
      if (m <= 5)
        for (unsigned t = m; t <= 5; t++)
          out[outPos + (t - m)] = (short)sym_[t];
      outPos = outPos - m + 6;
      depth_ = 0;
    }
    else
    {
      depth_ = depth_ - K_;
    }
    fill_ = fill_ - w0_;
  }
  outCnt = outPos;
}

/* ================================================================================================ V90Demapper */

V90Demapper::V90Demapper(unsigned cap, V90Parameters* p, V90AutoDigitalImpDetector* adid) : md_(), sdd_(), sbe_() { demapperInit(this, cap, p, adid); }

V90Demapper::~V90Demapper()
{
  if (params_->debugDemapperErrorHistogram != 0)
    printErrorHistogramAndReset();
  if (digits_ != 0)
    sysdep_free(digits_);
  if (bits_ != 0)
    sysdep_free(bits_);
}

void V90Demapper::resetLinearMappStudy(unsigned len)
{
  clearAllCumulative(adid_);
  studyCnt_ = 0;
  set_ = 0;
  studyDone_ = 0;
  study2Done_ = 0;
  studyIter_ = 0;
  studyLen_ = len;
}

void V90Demapper::reset(V90MappingParams* mp)
{
  unsigned w0 = mp->kBits_;
  unsigned K = mp->sr_;

  /* frame geometry */
  w0_ = w0;
  K_ = K;
  skip_ = 6u - K;
  nbits_ = w0 - (6u - K);
  if (K != 0)
    spf_ = 6u / K;

  /* levels, decoders, empty queues, frame position 0 */
  demapperLevels(this, mp);
  setModulusDecoder(this);
  sdd_.state_ = 0;
  sbe_.reset(K, 0);
  wr_ = 0;
  rd_ = 0;
  phase_ = 0;

  /* error histogram */
  for (unsigned i = 0; i <= 5; i++)
    for (unsigned j = 0; j <= 0x7f; j++)
    {
      errSum_[i * 128u + j] = 0;
      errCnt_[i * 128u + j] = 0;
    }
  {
    int d = params_->demapperDelayBeforeErrorHistogram;
    delay_ = d < params_->trn2dDdLength ? (unsigned)d : 0u;
  }
  integ_ = 0;

  /* linear-mapping study */
  clearAllCumulative(adid_);
  studyCnt_ = 0;
  set_ = 0;
  studyDone_ = 0;
  study2Done_ = 0;
  studyIter_ = 0;
  studyLen_ = 0;
  linMapStudyOn_ = 0;
}

void V90Demapper::resetNoSpectral(V90MappingParams* mp)
{
  unsigned skip = skip_;
  unsigned w0 = mp->kBits_;
  w0_ = w0;
  nbits_ = w0 - skip;

  /* print and restart the histogram (unlike reset(), the delay is taken without the trn2dDdLength limit) */
  if (params_->debugDemapperErrorHistogram != 0)
  {
    printErrorHistogramAndReset();
    integ_ = 0;
    delay_ = (unsigned)params_->demapperDelayBeforeErrorHistogram;
  }

  demapperLevels(this, mp);
  setModulusDecoder(this);
  sdd_.state_ = 0;
}

void V90Demapper::incrementRBSFramePosition() { phase_ = (phase_ + 1u) % 6u; }

void V90Demapper::printErrorHistogramAndReset()
{
  /* print and clear the histogram only when every constellation has levels; the print counter advances in any case */
  if (cnt_[0] != 0 && cnt_[1] != 0 && cnt_[2] != 0 && cnt_[3] != 0 && cnt_[4] != 0 && cnt_[5] != 0)
  {
    edprintf("===========================================================\r\n");
    edprintf("Error histogram #%d:\r\n", histNo_);
    for (unsigned i = 0; i <= 5; i++)
    {
      unsigned base = i * 128u;
      edprintf("constellation %d :\r\n", i);

      /* one line per level: level, decisions, mean absolute error */
      if (cnt_[i] != 0)
      {
        unsigned j = 0;
        do
        {
          unsigned n = errCnt_[base + j];
          unsigned avg = n != 0 ? errSum_[base + j] / n : 0u;
          edprintf("  %d\t-\t%d\t\t%d\r\n", (int)lev_[base + j], n, avg);
          j++;
        } while (cnt_[i] > j);
      }

      for (unsigned j2 = 0; j2 <= 0x7f; j2++)
      {
        errSum_[base + j2] = 0;
        errCnt_[base + j2] = 0;
      }
    }
    edprintf("===========================================================\r\n");
  }
  histNo_ = histNo_ + 1;
}

short V90Demapper::hardDecision(short x)
{
  int level = x;
  int sign;
  unsigned wr = wr_;
  set_ = (unsigned short)phase_;

  /* queue full: report and decide nothing */
  if (wr >= cap_)
  {
    edprintf("V90Demapper: Hard decision input buffers are full !!!!!!!!!!!!!\n");
    prevIdx_ = 0;
    return 0;
  }

  /* sign bit (1 = non-negative) and magnitude */
  if (x >= 0)
  {
    bits_[wr] = 1;
    sign = 1;
  }
  else
  {
    bits_[wr] = 0;
    sign = -1;
    level = (int)(short)(-level);
  }

  {
    int absx = absShort(level);
    V90AutoDigitalImpDetector* adid = adid_;
    unsigned phase = phase_;
    unsigned base = phase << 7;
    unsigned e = 0;
    int altPhase = altRbsOf(adid, (int)phase) != 0;

    /* level search: e = the last index (below the limit) whose level is not below |x|; the levels descend with the index */
    if (!((short)lev_[base] < (short)absx))
    {
      unsigned lim = altPhase ? cnt_[phase] * 2u - 1u : cnt_[phase] - 1u;
      if (lim != 0)
      {
        do
        {
          e++;
          if ((short)lev_[base + e] < (short)absx)
            break;
        } while (lim > e);
        e = e - 1u;
      }
    }

    {
      /* nearest of the two neighbouring levels; the lower one only when strictly closer */
      int hi = (short)lev_[base + e];
      int lo = (short)lev_[base + e + 1];
      int d1 = absShort(hi - level);
      int d2 = absShort(lo - level);
      unsigned k;
      int derr;
      if ((short)d2 >= (short)d1)
      {
        level = hi;
        derr = d1;
        k = e;
      }
      else
      {
        level = lo;
        derr = d2;
        k = e + 1;
      }

      /* error histogram: skip the first delay_ decisions, print and restart after integ_ decisions */
      if (params_->debugDemapperErrorHistogram != 0)
      {
        if (delay_ == 0)
        {
          unsigned ix = (phase << 7) + k;
          errSum_[ix] += (unsigned)derr;
          errCnt_[ix] += 1;
          if ((int)integ_ >= params_->demapperErrorHistogramIntegrationTime)
          {
            printErrorHistogramAndReset();
            integ_ = 0;
            phase = phase_;
            adid = adid_;
          }
          else
          {
            integ_ = integ_ + 1;
          }
        }
        else
        {
          delay_ = delay_ - 1;
        }
      }

      /* queue the digit (an alternate-RBS phase has two levels per digit), advance the frame position, return the signed level */
      if (altRbsOf(adid, (int)phase) != 0)
        digits_[wr_] = (unsigned)(int)(short)(k >> 1);
      else
        digits_[wr_] = k;
      wr_ = wr_ + 1;
      phase_ = (phase_ + 1u) % 6u;
      prevIdx_ = (unsigned short)k;
      return (short)(level * sign);
    }
  }
}

void V90Demapper::updateConstelation()
{
  rebuildLevels(this);
  if (dsplibs_debug_level > 1)
    dsplibs_debug_printf("V90Demapper: constelation update !!!\n");
}

/**
 * @note Uses the constellation set_ and level index prevIdx_ of the last hardDecision().  `a` and `b` are the two samples of the pair; |a| is added to
 *       the detector's cumulative sum of that level when ||a| - |b|| is below 0.4 times the spacing to the neighbouring level (the pair agrees).
 *       A study round ends after studyLen_ calls.
 * @warning with studyLen_ == 0 the round counter never equals the length and the study never completes.
 */
void V90Demapper::linearMappingStudy(short a, short b)
{
  int ai = a, bi = b;
  int aa = ai < 0 ? -ai : ai;
  int bb = bi < 0 ? -bi : bi;
  int d = aa - bb;
  int idx = (short)prevIdx_;
  int diff;

  /* spacing to the neighbouring level: the upper neighbour when |a| > |b| (the lower one for the first index), otherwise the lower one (the upper one
     for the last index) */
  if ((LD)d > 0.0L)
  {
    if (idx - 1 < 0)
    {
      int k = ((short)set_ << 7) + idx;
      diff = (short)lev_[k] - (short)lev_[k + 1];
    }
    else
    {
      int k = ((short)set_ << 7) + idx;
      diff = (short)lev_[k - 1] - (short)lev_[k];
    }
  }
  else
  {
    int s = (short)set_;
    int n = altRbsOf(adid_, s) != 0 ? (int)(short)(cnt_[s] * 2u) : (int)(short)cnt_[s];
    int k = (s << 7) + idx;
    if (idx + 1 >= n)
      diff = (short)lev_[k - 1] - (short)lev_[k];
    else
      diff = (short)lev_[k] - (short)lev_[k + 1];
  }

  /* accumulate |a| (through a float) when the pair agrees */
  {
    LD ad = (LD)(d < 0 ? -d : d);
    LD lim = (LD)0.4f * (LD)diff;
    if (!(lim <= ad))
    {
      V90AutoDigitalImpDetector* adid = adid_;
      int kk = ((short)set_ << 7) + (short)prevIdx_;
      float* cs = cumSumP(adid) + kk;
      *cs = (float)((LD)aa + (LD)*cs);
      cumCountP(adid)[kk]++;
    }
  }

  /* round not complete yet */
  {
    unsigned c = studyCnt_ + 1u;
    if (c != studyLen_)
    {
      studyCnt_ = c;
      return;
    }
  }

  /* the round is complete: count it (the second round also raises study2Done_), rebuild the levels, start a new round */
  {
    unsigned short it = (unsigned short)(studyIter_ + 1u);
    studyIter_ = it;
    if (it == 2)
      study2Done_ = 1;
  }
  studyCnt_ = 0;
  studyDone_ = 1;
  rebuildLevels(this);
  if (dsplibs_debug_level > 1)
    dsplibs_debug_printf("V90Demapper: constelation update !!!\n");
  clearAllCumulative(adid_);
}

/** @warning `out` is unchecked (w0_ bits per six decisions); the digits are used as unchecked indices by the ModulusDecoder. */
unsigned V90Demapper::process(unsigned char* out, unsigned& outCnt)
{
  unsigned consumed = 0;
  unsigned wr = wr_;
  unsigned rd;

  /* fewer than six decisions queued: nothing to do */
  if (wr <= 5)
  {
    outCnt = 0;
    return 0;
  }

  /* decode every complete group of six decisions: the digits through the modulus decoder (the sign-bit positions are filled first) ... */
  rd = rd_;
  if (!(rd + 6u > wr))
  {
    do
    {
      md_.progress(out + (skip_ + consumed), digits_ + rd);

      /* ... and the sign bits through the serial differential decoder (K_ == 0) or the sign extractor (K_ groups) */
      if (K_ == 0)
      {
        if (skip_ != 0)
        {
          unsigned j = 0;
          do
          {
            out[consumed + j] = sdd_.process(bits_[rd_ + j]);
            j++;
          } while (skip_ > j);
        }
      }
      else
      {
        unsigned k = 0;
        do
        {
          sbe_.process(bits_ + (rd_ + spf_ * k), out + (int)((spf_ - 1u) * k + consumed));
          k++;
        } while (K_ > k);
      }
      consumed += w0_;
      rd = rd_ + 6u;
      rd_ = rd;
      wr = wr_;
    } while (!(rd + 6u > wr));
  }

  /* compact the queues: move the undecoded tail to the front */
  {
    unsigned e = 0;
    while (wr_ > rd)
    {
      digits_[e] = digits_[rd];
      bits_[e] = bits_[rd];
      rd++;
      e++;
    }
    wr_ = e;
  }
  rd_ = 0;
  outCnt = consumed;
  return 1;
}
