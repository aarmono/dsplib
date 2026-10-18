/**
 * @file v92_tx_coding.cpp
 * @brief V.92 transmitter leaf classes: V92ConvolutionEncoder (trellis encoder), V92Precoder (Tomlinson-Harashima style search) and V92PreFilter.
 *
 * Defines the classes declared in v92_tx_coding.h together with the two constant tables of the convolutional encoder (cosetMapping4D,
 * subsetLabelTable).  All three are driven by V92Transmitter::process() (pcm_bits_to_symbol.cpp).  The FIR / IIR stages come from
 * dsp/dsp_float_filters.h.
 *
 * Floating-point rule: V92Precoder::process() keeps the candidate value, the sum and the best squared sum as x87 EXTENDED values (`long double`).
 * The table entry is a zero-extended 32-bit integer (exact), s = (fb1 + X) + fb2 and sq = s * s is NOT rounded to float; only the best s and X are
 * rounded to float (the two values kept across the four symbols) when they are stored.  The comparison treats an unordered result as "below".
 *
 * Behaviour worth knowing:
 *  - V92ConvolutionEncoder::process() with a mode outside 0 .. 2 steps nothing and returns 0; reset() with such a mode builds no table.
 *  - makeStateTtransitionTable() writes every `out_` entry once (the (state, input) -> (next state, input) map is a bijection for the three modes), so
 *    the order of the loop nests is not observable; the `out_` table is indexed by the NEW state.
 *  - V92Precoder::process(): the "best" values of the previous symbol are kept when no candidate of this symbol is below the 999999995904.0f limit
 *    (they start at zero); a symbol with row > 2 reads past m_; a zero radix or an INT_MIN / -1 division traps.
 *  - The constructors never test an allocation result.
 */

#include "v92_tx_coding.h"
#include "core/dsp_asm.h"
#include "core/dsp_host.h"

typedef long double LD;

#include "core/dsp_copyword.h"

/* ------------------------------------------------------------------------------------------------ V92ConvolutionEncoder: tables */

/** @brief Coset label of the 4D mapping, indexed by s1 * 8 + s2 (s1, s2 = 3-bit subset labels of the two pairs). */
int V92ConvolutionEncoder::cosetMapping4D[64] = {0,  0,  1, 1, 8,  8,  9,  9,  3,  2,  2, 3, 11, 10, 10, 11, 5, 5, 4,  4,  13, 13,
                                                 12, 12, 6, 7, 7,  6,  14, 15, 15, 14, 8, 8, 9,  9,  0,  0,  1, 1, 11, 10, 10, 11,
                                                 3,  2,  2, 3, 13, 13, 12, 12, 5,  5,  4, 4, 14, 15, 15, 14, 6, 7, 7,  6};
/** @brief 3-bit subset label of a pair of 2-bit indices (p0 + 4 * p1). */
int V92ConvolutionEncoder::subsetLabelTable[16] = {0, 7, 4, 3, 5, 2, 1, 6, 4, 3, 0, 7, 1, 6, 5, 2};

/* ------------------------------------------------------------------------------------------------ V92ConvolutionEncoder: the trellis tables */

/* Mode 0: 16 states (state = p + 2q + 4r + 8s), 2 input bits (t + 2u), the coded bit r ^ t;
 * next = s | (p ^ s ^ u) << 1 | (q ^ u) << 2 | (r ^ t) << 3 (written with % 2). */
static inline void makeMode0(int* next, int* out)
{
  for (int p = 0; p < 2; p++)
    for (int q = 0; q < 2; q++)
      for (int r = 0; r < 2; r++)
        for (int s = 0; s < 2; s++)
          for (int t = 0; t < 2; t++)
            for (int u = 0; u < 2; u++)
            {
              int state = p + 2 * q + 4 * r + 8 * s;
              int input = t + 2 * u;
              int bit = (r + t) % 2;
              int n = s + 2 * (((p + s + u) % 2) + 2 * (((q + u) % 2) + 2 * bit));
              next[state * 16 + input] = n;
              out[n * 16 + input] = bit;
            }
}

/* Mode 1: 32 states (state = a + 2b + 4c + 8d + 16e), 3 input bits (f + 2g + 4h), the coded bit d ^ g */
static inline void makeMode1(int* next, int* out)
{
  for (int a = 0; a < 2; a++)
    for (int b = 0; b < 2; b++)
      for (int c = 0; c < 2; c++)
        for (int d = 0; d < 2; d++)
          for (int e = 0; e < 2; e++)
            for (int f = 0; f < 2; f++)
              for (int g = 0; g < 2; g++)
                for (int h = 0; h < 2; h++)
                {
                  int state = a + 2 * b + 4 * c + 8 * d + 16 * e;
                  int input = f + 2 * g + 4 * h;
                  int bit = (d + g) % 2;
                  int n = e + 2 * (((a + g) % 2) + 2 * (((b + f) % 2) + 2 * (((c + h) % 2) + 2 * bit)));
                  next[state * 16 + input] = n;
                  out[n * 16 + input] = bit;
                }
}

/* Mode 2: 64 states (state = a + 2b + 4c + 8d + 16e + 32f), 4 input bits (g + 2h + 4i + 8j), the coded bit e ^ c ^ h */
static inline void makeMode2(int* next, int* out)
{
  for (int a = 0; a < 2; a++)
    for (int b = 0; b < 2; b++)
      for (int c = 0; c < 2; c++)
        for (int d = 0; d < 2; d++)
          for (int e = 0; e < 2; e++)
            for (int f = 0; f < 2; f++)
              for (int g = 0; g < 2; g++)
                for (int h = 0; h < 2; h++)
                  for (int i = 0; i < 2; i++)
                    for (int j = 0; j < 2; j++)
                    {
                      int state = a + 2 * b + 4 * c + 8 * d + 16 * e + 32 * f;
                      int input = g + 2 * h + 4 * i + 8 * j;
                      int bit = (e + c + h) % 2;
                      int low = (a + b + (((b + g) % 2) * c) + j) % 2;
                      int n = low + 2 * (((a + b + d + i + c * h) % 2) + 2 * (((b + g + c) % 2) + 2 * (c + 2 * (f + 2 * bit))));
                      next[state * 16 + input] = n;
                      out[n * 16 + input] = bit;
                    }
}

V92ConvolutionEncoder::V92ConvolutionEncoder() {}

V92ConvolutionEncoder::~V92ConvolutionEncoder() {}

/** @brief Fill next_ / out_ for the trellis selected by mode_ (0, 1 or 2); any other mode leaves them unchanged. */
void V92ConvolutionEncoder::makeStateTtransitionTable()
{
  if (mode_ == 1)
    makeMode1(next_, out_);
  else if (mode_ == 0)
    makeMode0(next_, out_);
  else if (mode_ == 2)
    makeMode2(next_, out_);
}

/** @brief Select the trellis, build its tables and restart at state 0. */
void V92ConvolutionEncoder::reset(int mode)
{
  mode_ = mode;
  makeStateTtransitionTable();
  state_ = 0;
}

/* (x % 4 + 4) % 4 of a + 2, with truncating C remainders: the low two bits of a + 2. */
static inline int mod4plus2(int a)
{
  int x = (int)((unsigned)a + 2u);
  return ((x % 4) + 4) % 4;
}

/** @brief Two 2-bit pairs (from a[0..3]) through subsetLabelTable, then cosetMapping4D: the 6-bit label of the four subset indices. */
int V92ConvolutionEncoder::inverseMap(int* a)
{
  int p0 = mod4plus2(a[0]);
  int p1 = mod4plus2(a[1]);
  int p2 = mod4plus2(a[2]);
  int p3 = mod4plus2(a[3]);
  int s1 = subsetLabelTable[p0 + p1 * 4];
  int s2 = subsetLabelTable[p2 + p3 * 4];
  return cosetMapping4D[s1 * 8 + s2];
}

/**
 * @brief Encode four subset indices and step the trellis.
 * @return the coded bit out_[next state][input]; 0 (and no step) when mode_ is outside 0 .. 2
 */
int V92ConvolutionEncoder::process(int* a)
{
  int label = inverseMap(a);
  int input;
  if (mode_ == 1)
    input = (label & 3) | ((label & 8) >> 1);
  else if (mode_ == 2)
    input = label % 16;
  else if (mode_ == 0)
    input = label % 4;
  else
    return 0; /* unknown mode: no step */

  int n = next_[state_ * 16 + input];
  state_ = n;
  return out_[n * 16 + input];
}

/* ------------------------------------------------------------------------------------------------ V92Precoder */

/* A heap FloatFIR(ntaps, NULL, 99): sysdep_malloc + placement constructor; the result of the allocation is not checked. */
static inline P02_FloatFIR* newFir(unsigned ntaps)
{
  P02_FloatFIR* f = (P02_FloatFIR*)sysdep_malloc((unsigned)sizeof(P02_FloatFIR));
  new (f) P02_FloatFIR(ntaps, 0, 99u);
  return f;
}

static inline P02_FloatIIR* newIir(unsigned ntaps)
{
  P02_FloatIIR* f = (P02_FloatIIR*)sysdep_malloc((unsigned)sizeof(P02_FloatIIR));
  new (f) P02_FloatIIR(ntaps, 0, 99u);
  return f;
}

static inline void freeFir(P02_FloatFIR* f)
{
  if (f)
  {
    f->~P02_FloatFIR();
    sysdep_free(f);
  }
}

static inline void freeIir(P02_FloatIIR* f)
{
  if (f)
  {
    f->~P02_FloatIIR();
    sysdep_free(f);
  }
}

/** @brief Create the two feedback FIRs of ntaps taps each. */
V92Precoder::V92Precoder(unsigned ntaps)
{
  fir1_ = newFir(ntaps);
  fir2_ = newFir(ntaps);
}

/** @brief Destroy and free both FIRs. */
V92Precoder::~V92Precoder()
{
  freeFir(fir1_);
  freeFir(fir2_);
}

/** @brief Reset both FIRs (debug print at level > 1).  No caller in the library. */
void V92Precoder::reset()
{
  if (dsplibs_debug_level > 1)
    dsplibs_debug_printf("V92Precoder: reset 2 called\r\n");
  fir1_->reset();
  fir2_->reset();
}

/** @brief Borrow the constellation tables of the mapping parameters, copy radices and sizes, clear the feedback values. */
void V92Precoder::reset(V92MappingParams* p)
{
  if (dsplibs_debug_level > 1)
    dsplibs_debug_printf("V92Precoder: reset 1 (with cfg) called\r\n");
  idxc_ = &p->idxc_[0];
  for (int i = 0; i < 6; i++)
    lcp_[i] = p->lcp_[i];
  for (int i = 0; i < 12; i++)
    m_[i] = (int32_t)p->m_[i];
  for (int i = 0; i < 6; i++)
    lc_[i] = (int32_t)p->lc_[i];
  fb1_ = 0.0f;
  fb2_ = 0.0f;
}

/** @brief Load the coefficients of the two FIRs and remember their tap counts (0 disables a feedback path). */
void V92Precoder::setCoefficients(float* c1, float* c2, unsigned n1, unsigned n2)
{
  if (dsplibs_debug_level > 1)
    dsplibs_debug_printf("V92Precoder: setCoefficients called\r\n");
  fir1_->setCoefficients(c1, n1);
  fir2_->setCoefficients(c2, n2);
  n1_ = n1;
  n2_ = n2;
}

static const float kStartLimit = 999999995904.0f; /* the initial "best squared sum" of every symbol */

/**
 * @brief Precode the four values of row `row`.
 *
 * For each symbol j the candidates are v = u + m * k for the integers k that keep v inside the constellation (the last symbol uses doubled radix and
 * the parity of the earlier choices); the table value of each candidate plus the two feedback values is squared and the smallest square wins.  The
 * winner is stored in state[j], its sum in out[j]; the FIRs are then fed with the winner's value and sum.
 *
 * @param u      data values; u[row * 4 + j]
 * @param row    row index into the radix / constellation tables (row * 4 + j)
 * @param par    parity from the previous group
 * @param state  receives the chosen v per symbol
 * @param out    receives the four chosen sums
 * @warning row > 2 reads past m_; division by a zero radix traps.
 */
void V92Precoder::process(unsigned* u, int row, int par, int* state, float* out)
{
  /* the best values live across the four symbols (zero here; a symbol without a candidate repeats the previous symbol's) */
  float bestS = 0.0f;
  float bestX = 0.0f;
  for (int j = 0; j < 4; j++)
  {

    unsigned idx = (unsigned)j + (unsigned)row * 4u;
    int ci = (int)idxc_[idx % 6u];
    unsigned uv = u[idx];
    int lc2 = (int)((unsigned)lc_[ci] * 2u);
    int m = m_[idx];
    int kLo, kHi;
    if (j < 3)
    {
      kLo = -((int)((unsigned)(lc2 >> 1) + uv) / m);
      kHi = (int)((unsigned)((lc2 - 1) / 2) - uv) / m;
    }
    else
    {
      unsigned par2 = ((unsigned)state[1] + (unsigned)state[0] + (unsigned)state[2] + (unsigned)par) & 1u;
      int m2 = (int)((unsigned)m * 2u);
      kLo = -((int)(uv * 2u + (unsigned)(lc2 >> 1) + par2) / m2);
      kHi = (int)((unsigned)((lc2 - 1) / 2) - uv * 2u) / m2;
    }

    /* search the candidates kLo..kHi for the smallest squared sum */
    if (kLo <= kHi)
    {
      float f1 = fb1_;
      float f2 = fb2_;
      LD limit = (LD)kStartLimit;
      int k = kLo;
      for (;;)
      {
        int v;
        if (j < 3)
          v = (int)(uv + (unsigned)m * (unsigned)k);
        else
        {
          unsigned par2 = ((unsigned)state[1] + (unsigned)state[0] + (unsigned)state[2] + (unsigned)par) & 1u;
          v = (int)(par2 + ((unsigned)m * (unsigned)k + uv) * 2u);
        }
        LD x = (v < 0) ? -(LD)lcp_[ci][-v - 1] : (LD)lcp_[ci][v];
        LD s = ((LD)f1 + x) + (LD)f2;
        LD sq = s * s;
        if (!(sq >= limit))
        { /* fcom + jb: unordered counts as below */
          state[j] = v;
          bestS = (float)s;
          bestX = (float)x;
          limit = sq;
        }
        if (k >= kHi)
          break;
        k++;
      }
    }

    /* update the feedback values and publish the result of this symbol */
    if (n1_ != 0)
      fb1_ = fir1_->process(bestX);
    if (n2_ != 0)
      fb2_ = fir2_->process(bestS);
    out[j] = bestS;
  }
}

/* ------------------------------------------------------------------------------------------------ V92PreFilter */

/** @brief Create the FIR and IIR stages of ntaps taps each. */
V92PreFilter::V92PreFilter(unsigned ntaps)
{
  fir_ = newFir(ntaps);
  iir_ = newIir(ntaps);
}

/** @brief Destroy and free both stages. */
V92PreFilter::~V92PreFilter()
{
  freeFir(fir_);
  freeIir(iir_);
}

/** @brief Reset the state of both stages. */
void V92PreFilter::reset()
{
  fir_->reset();
  iir_->reset();
}

/** @brief Load the FIR and IIR coefficients and remember their tap counts. */
void V92PreFilter::setCoefficients(float* c1, float* c2, unsigned n1, unsigned n2)
{
  fir_->setCoefficients(c1, n1);
  iir_->setCoefficients(c2, n2);
  nFir_ = n1;
  nIir_ = n2;
}

/**
 * @brief Filter 12 samples: FIR then IIR; a stage with a tap count of 0 is skipped (both skipped = copy).
 * @param in   12 input samples
 * @param out  12 output samples
 */
void V92PreFilter::process(float* in, float* out)
{
  if (nFir_ != 0)
  {
    if (nIir_ == 0)
    {
      fir_->process(in, out, 12u);
    }
    else
    {
      float tmp[12];
      fir_->process(in, tmp, 12u);
      iir_->process(tmp, out, 12u);
    }
  }
  else
  {
    if (nIir_ == 0)
    {
      for (int i = 0; i < 12; i++)
        copyWord(&out[i], &in[i]);
    }
    else
    {
      iir_->process(in, out, 12u);
    }
  }
}
