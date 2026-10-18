/**
 * @file v92_echo_canceller.cpp
 * @brief V92EchoCanceller (V.92 near-end echo canceller) and V90PreFilter (V.90 receive pre-filter selection and EIA6 parameter switch).
 *
 * Defines the classes declared in v92_echo_canceller.h and the two ARMA coefficient tables of the echo canceller (v92echoPreFilter_a / _b, file-local;
 * the V90PreFilter tables are in v90_prefilter_tables.cpp).  Callees from other files: FloatARMA / FloatFIR (dsp/dsp_float_filters.h), edprintf() and
 * the sysdep_malloc() / sysdep_free() / dsplibs_debug_printf() host functions.  Callers: VPcmFloModem (the `ec_` member: construction, destruction, the
 * session wrappers that feed the history and run process()), V90Demodulator (the `pf_` member: selectFilter(), setFilter(), getV90Capability(),
 * setParamEia6(), isV90WithEia6()).
 *
 * Numeric conventions: every value that stays on the x87 stack in 32-bit code (the dot-product accumulators, the error, the step times the error, the
 * squared distances of the loop selection) is a `long double` here, with float / double rounding only where a value is stored; float words that are
 * merely moved are copied as 32-bit integers (out[i] = in[i] in the bypass and count-delay states, the EIA6 parameter copies) so that no FPU load can
 * quiet a signalling NaN.  Comparisons are written so that an unordered (NaN) compare takes the same branch as the 32-bit code does.
 *
 * Behaviour worth knowing before changing anything:
 * - V92EchoCanceller::process(): out[0] == 177.0f on entry (the OUTPUT buffer) bypasses everything (out = in, the read position still advances).  The
 *   dot product has two extended accumulators (terms 0 / 2 of every group of four into A, terms 1 / 3 into B, a remainder term into A) and the estimate
 *   is (B + A) + 0.0; out = in - estimate.  The adaptation step is mu = e * beta (extended), coef[k] = (float)(hist[pos + k] * mu + coef[k]) and then
 *   beta = (float)(beta * decay), after every sample.  State 1 only copies and counts, state 0 never counts.
 * - The V92EchoCanceller constructor applies setEchoDelay()'s arithmetic to the uninitialised `delay_` / `histFill_` before reset() overwrites
 *   `histFill_`, and divides by `granularity` (0 traps).
 * - The V90PreFilter constructor re-reads `dsplibs_debug_level` after every print of its error ladder (the cached value decides the final
 *   "HardwareCodecType" message), counts the dataBase records up to the first empty name, replaces an out-of-range codec type by 0 and ends with
 *   the equivalent of reset().
 * - setFilter(idx) and selectFilter() compute the table row of the loaded coefficient type from the RAW or the CLAMPED gain index depending on the path:
 *   the ISDN / PBX gains and the "no reference loop" case are not clamped; the forced and automatic paths clamp to 30 (types 1 / 2) or 50 (type 3, whose
 *   row is index - 20, so a lower index reads before the table); a coefficient type outside 1 .. 3 prints the "BUG" message and uses table 1.
 *   filterIdx_ stores the unclamped gain (selectFilter's type 3 branches and setFilter(type, idx) store idx - 20).
 * - autoSelection() compares in extended precision; a NaN distance wins and the best distance is kept unrounded.
 */

#include "v92_echo_canceller.h"
#include "core/dsp_math.h"
#include "core/dsp_asm.h"
#include "core/dsp_host.h"
#include "v34/v34_api.h"

extern "C"
{
}

typedef long double LD;

/* ---------------------------------------------------------------------------------------------------------------- tables of the echo canceller */

/** @brief AR (denominator) coefficients of the 12th-order ARMA pre-filter of the far-end signal; the asm label keeps the plain symbol name. */
static float v92echoPreFilter_a[12] __asm__("v92echoPreFilter_a") = {1.0f,         0.708999991f, -2.30019999f, -2.67490005f, 1.53729999f,  3.62899995f,
                                                                     0.467999995f, -2.13840008f, -1.00730002f, 0.423099995f, 0.344900012f, 0.0285f};
/** @brief MA (numerator) coefficients of the 12th-order ARMA pre-filter of the far-end signal. */
static float v92echoPreFilter_b[12] __asm__("v92echoPreFilter_b") = {1.49860001f,  0.621999979f, -2.73329997f,  -2.60159993f, 1.71739995f,  3.37879992f,
                                                                     0.135000005f, -1.75409997f, -0.682799995f, 0.350800008f, 0.221100003f, 0.00579999993f};

#include "core/dsp_intmath.h"
#include "vpcm_split.h"
#include "vpcm_split_value.h"

/** @brief Print a float-valued debug line through edprintf() as sign, integer part and six fractional digits (format with "%c%d.%06d"). */
static inline void edFloat(const char* fmt, LD v)
{
  Split s = splitValue(v, 1000000.0L);
  edprintf(fmt, s.sign, s.ip, s.fp);
}

#include "core/dsp_floatbits.h"
#include "core/dsp_copyword.h"

/* ================================================================================================================ V92EchoCanceller */

/**
 * @brief Dot product of the echo coefficients with the history at the read position.
 * @param h  history samples from `pos_`
 * @param c  coefficients
 * @param n  number of terms
 * @return the echo estimate, (B + A) + 0.0, with two extended accumulators: A takes terms 0 and 2 of each group of four (and the remainder),
 *         B takes terms 1 and 3
 */
static inline LD echoDot(const float* h, const float* c, unsigned n)
{
  LD a = 0.0L, b = 0.0L;
  while (n > 3)
  {
    a = a + (LD)h[0] * (LD)c[0];
    b = b + (LD)h[1] * (LD)c[1];
    a = a + (LD)h[2] * (LD)c[2];
    b = b + (LD)h[3] * (LD)c[3];
    h += 4;
    c += 4;
    n -= 4;
  }

  while (n != 0)
  {
    a = a + (LD)h[0] * (LD)c[0];
    h++;
    c++;
    n--;
  }

  LD est = b + a;
  return est + 0.0L;
}

/**
 * @brief Size and allocate the coefficient array and the history from the V92Parameters and create the ARMA pre-filter; ends with reset().
 * @param p            parameter block, borrowed
 * @param granularity  block size of the caller; the history length is rounded up to whole blocks plus two
 * @param extra        extra history samples
 * @warning The sysdep_malloc() results are not checked; `granularity` == 0 divides by zero; `delay_` and `histFill_` are read uninitialised by the
 *          delay update below (the result in `histFill_` is overwritten by reset(), the new `delay_` does not depend on the old value).
 */
V92EchoCanceller::V92EchoCanceller(V92Parameters* p, unsigned granularity, unsigned extra)
{
  if (dsplibs_debug_level > 1)
    dsplibs_debug_printf("V92EchoCanceller: constraction\r\n");
  params_ = p;

  /* the initial echo delay, through the same arithmetic as setEchoDelay() (on the uninitialised members; reset() sets histFill_ below) */
  unsigned d = (unsigned)p->v92EchoInitialDelay;
  unsigned old = delay_;
  delay_ = d;
  histFill_ = histFill_ + (d - old);
  edprintf("V92EchoCanceller: echoDelay updated to: %d\n", d);

  /* the filter length is rounded to a multiple of 4 (towards zero) */
  int n = p->v92EchoFilterLength;
  if (n < 0)
    n += 3;
  n &= ~3;
  nCoefM1_ = (unsigned)(n - 1);
  nCoef_ = (unsigned)n;
  edprintf("V92EchoCanceller: echoFilterLen = %d\r\n", n);

  /* coefficient array and history: the history holds the delay, the filter span and a whole number of caller blocks */
  coef_ = (float*)sysdep_malloc(nCoef_ << 2);
  unsigned a = nCoefM1_ + delay_;
  histLen_ = a + granularity * 2 + (a / granularity) * granularity + extra;
  hist_ = (float*)sysdep_malloc(histLen_ << 2);

  void* mem = sysdep_malloc(sizeof(FloatARMA));
  FloatARMA* arma = new (mem) FloatARMA(12, 12, v92echoPreFilter_a, v92echoPreFilter_b, 99);
  arma_ = arma;
  reset();
}

/** @brief Release the coefficient array, the history and the ARMA pre-filter and clear the three pointers. */
V92EchoCanceller::~V92EchoCanceller()
{
  if (dsplibs_debug_level > 1)
    dsplibs_debug_printf("V92EchoCanceller Destruction\r\n");
  if (coef_)
  {
    sysdep_free(coef_);
    coef_ = NULL;
  }
  if (hist_)
  {
    sysdep_free(hist_);
    hist_ = NULL;
  }
  if (arma_)
  {
    arma_->~FloatARMA();
    sysdep_free(arma_);
    arma_ = NULL;
  }

  /* the pointers are cleared although the object is destroyed after this destructor; the barrier stops the compiler (-flifetime-dse) from
   * deleting the stores */
  KEEP_STORES_TO(this);
}

/**
 * @brief Change the echo delay: moves the history fill point by the difference to the old delay and prints the new value.
 * @param delay new echo delay in samples
 */
void V92EchoCanceller::setEchoDelay(unsigned delay)
{
  unsigned diff = delay - delay_;
  delay_ = delay;
  histFill_ = histFill_ + diff;
  edprintf("V92EchoCanceller: echoDelay updated to: %d\n", delay);
}

/** @brief Set the adaptation step and print it.  @param beta new step  @note No caller in the library; kept for external test harnesses. */
void V92EchoCanceller::setEchoBeta(float beta)
{
  beta_ = beta;
  edFloat("V92EchoCanceller: echoBeta = %c%d.%06d\r\n", (LD)beta);
}

/** @brief Set the per-sample step decay and print it.  @param decay new factor  @note No caller in the library; kept for external test harnesses. */
void V92EchoCanceller::setDecayFactor(float decay)
{
  decay_ = decay;
  edFloat("V92EchoCanceller: echoBetaDecay = %c%d.%06d\r\n", (LD)decay);
}

/** @brief Set beta, decay and the echo delay, printing each.  @note No caller in the library; kept for external test harnesses. */
void V92EchoCanceller::setEchoParams(float beta, float decay, unsigned delay)
{
  beta_ = beta;
  edFloat("V92EchoCanceller: echoBeta = %c%d.%06d\r\n", (LD)beta);
  decay_ = decay;
  edFloat("V92EchoCanceller: echoBetaDecay = %c%d.%06d\r\n", (LD)decay);

  unsigned diff = delay - delay_;
  delay_ = delay;
  histFill_ = histFill_ + diff;
  edprintf("V92EchoCanceller: echoDelay updated to: %d\n", delay);
}

/** @brief Zero the nCoef_ echo coefficients.  @note No caller in the library; kept for external test harnesses. */
void V92EchoCanceller::zeroEchoCoeff()
{
  for (unsigned i = 0; i < nCoef_; i++)
    coef_[i] = 0.0f;
}

/**
 * @brief Restart the history: read position 0 and zeros up to the fill point (nCoef_ / 2 + delay + v92EchoDelayOffset).
 * @note No caller in the library; kept for external test harnesses.
 */
void V92EchoCanceller::resetEchoHistory()
{
  pos_ = 0;
  histFill_ = (nCoef_ >> 1) + delay_ + (unsigned)params_->v92EchoDelayOffset;
  for (unsigned i = 0; i < histFill_; i++)
    hist_[i] = 0.0f;
}

/**
 * @brief Back to the untrained state: zero coefficients, zeroed history up to the fill point, filter-only state, beta = decay = 0, ARMA pre-filter reset.
 * @warning The history is zeroed up to nCoef_ / 2 + delay_ + v92EchoDelayOffset without a check against `histLen_`.
 */
void V92EchoCanceller::reset()
{
  for (unsigned i = 0; i < nCoef_; i++)
    coef_[i] = 0.0f;

  pos_ = 0;
  histFill_ = (nCoef_ >> 1) + delay_ + (unsigned)params_->v92EchoDelayOffset;
  for (unsigned i = 0; i < histFill_; i++)
    hist_[i] = 0.0f;

  state_ = V92_ECHO_FILTER_ONLY;
  beta_ = 0.0f;
  edprintf("V92EchoCanceller: echoBeta = %c%d.%06d\r\n", 0x2d, 0, 0);
  decay_ = 0.0f;
  edprintf("V92EchoCanceller: echoBetaDecay = %c%d.%06d\r\n", 0x2d, 0, 0);
  arma_->reset();
}

/**
 * @brief Enter an echo canceller state, load its step, decay and duration from the parameters, print the change and restart the sample count.
 * @param newState  V92EchoCancellerState: count delay sets the limit to delay + 400; fast / slow training load beta, decay and the update duration of the
 *                  V92Parameters; filter only (from a training state) first prints the echo filter coefficients and then clears beta and decay
 * @note Setting the current state is a no-op (the count is not restarted); an unknown value only logs when the debug level is above 1 and restarts the count.
 */
void V92EchoCanceller::setState(V92EchoCancellerState newState)
{
  int s = (int)newState;
  if (state_ == s)
    return;
  if (s == V92_ECHO_COUNT_DELAY)
  {
    edprintf("V92EchoCanceller: echo state set to count delay before training\r\n");
    state_ = V92_ECHO_COUNT_DELAY;
    limit_ = delay_ + 0x190;
  }
  else if (s == V92_ECHO_FAST_TRAINING)
  {
    edprintf("V92EchoCanceller: echo state set to fast echo training\r\n");
    state_ = V92_ECHO_FAST_TRAINING;
    float b = params_->v92EchoFastBetaFactor;
    beta_ = b;
    edFloat("V92EchoCanceller: echoBeta = %c%d.%06d\r\n", (LD)b);
    float d = params_->v92EchoFastDecayFactor;
    decay_ = d;
    edFloat("V92EchoCanceller: echoBetaDecay = %c%d.%06d\r\n", (LD)d);
    limit_ = (unsigned)params_->v92EchoFastUpdateDuration;
  }
  else if (s == V92_ECHO_SLOW_TRAINING)
  {
    edprintf("V92EchoCanceller: echo state set to slow echo training\r\n");
    state_ = V92_ECHO_SLOW_TRAINING;
    float b = params_->v92EchoSlowBetaFactor;
    beta_ = b;
    edFloat("V92EchoCanceller: echoBeta = %c%d.%06d\r\n", (LD)b);
    float d = params_->v92EchoSlowDecayFactor;
    decay_ = d;
    edFloat("V92EchoCanceller: echoBetaDecay = %c%d.%06d\r\n", (LD)d);
    limit_ = (unsigned)params_->v92EchoSlowUpdateDuration;
  }
  else if (s == V92_ECHO_FILTER_ONLY)
  {
    edprintf("V92EchoCanceller: echo state set to filter only\r\n");
    if ((unsigned)(state_ - 2) <= 1u)
    { /* leaving a training state: print the echo filter */
      float* c = coef_;
      unsigned n = nCoef_;
      edprintf("**************** NEAR ECHO FILTER *************** \n");
      for (unsigned i = 0; i < n; i++)
        edFloat("   = %c%d.%06d\r\n", (LD)c[i]);
    }
    state_ = V92_ECHO_FILTER_ONLY;
    beta_ = 0.0f;
    edprintf("V92EchoCanceller: echoBeta = %c%d.%06d\r\n", 0x2d, 0, 0);
    decay_ = 0.0f;
    edprintf("V92EchoCanceller: echoBetaDecay = %c%d.%06d\r\n", 0x2d, 0, 0);
  }
  else
  {
    if (dsplibs_debug_level > 1)
      dsplibs_debug_printf("V92EchoCanceller: setState ERROR: illegal state\r\n");
  }
  count_ = 0;
}

/**
 * @brief Append far-end samples, ARMA pre-filtered, to the history at the fill point.
 * @param x  far-end samples
 * @param n  sample count
 * @note When the history becomes full, the newest nCoef_ - 1 samples are moved to the front (backwards, element by element) and filling continues behind them.
 */
void V92EchoCanceller::updateEchoHistory(float* x, unsigned n)
{
  unsigned fill = histFill_;
  float* dst = hist_ + fill;
  if (fill + n < histLen_)
  {
    for (unsigned i = 0; i < n; i++)
      *dst++ = arma_->process(x[i]);
    histFill_ = fill + n;
    return;
  }

  while (n != 0)
  {
    float y = arma_->process(*x++);
    *dst++ = y;
    unsigned f1 = histFill_ + 1;
    if (f1 == histLen_)
    {
      /* the history is full: move the newest nCoef_ - 1 samples to the front and continue behind them */
      float* h = hist_;
      unsigned nc = nCoef_;
      float* src = h + (f1 - 1);
      float* to = h + (nc - 2);
      unsigned cnt = nc - 1;
      do
      {
        *to-- = *src--;
        cnt--;
      } while (cnt != 0);
      n--;
      histFill_ = nc - 1;
      dst = h + (nc - 1);
    }
    else
    {
      n--;
      histFill_ = f1;
    }
  }
}

/**
 * @brief Cancel the echo in a block of received samples and, in the training states, adapt the coefficients.
 * @param in   received samples
 * @param out  output samples; out[0] == 177.0f on entry selects the bypass (out = in, only the read position advances)
 * @param n    sample count
 * @note State 0 filters only; state 1 copies the input and counts, switching to fast training when the count reaches `limit_`; states 2 / 3 filter and
 *       adapt per sample, then switch fast -> slow -> filter only when the count reaches `limit_` (the count is per block, so a switch happens at
 *       the end of the block that reaches it).
 * @warning `out` is read before it is written (the bypass marker); a NaN in out[0] also selects the bypass.  The history must hold at least
 *          nCoef_ samples behind `pos_`; the position wraps at histLen_ - nCoefM1_.
 */
void V92EchoCanceller::process(float* in, float* out, unsigned n)
{
  {
    LD marker = (LD)out[0];
    if (!(marker < 177.0L || marker > 177.0L))
    { /* equal to 177.0 or unordered (NaN): bypass */
      unsigned W = histLen_ - nCoefM1_;
      for (unsigned i = 0; i < n; i++)
      {
        copyWord(&out[i], &in[i]);
        unsigned p = pos_ + 1;
        pos_ = (p == W) ? 0u : p;
      }
      return;
    }
  }

  int st = state_;
  if (st == V92_ECHO_FILTER_ONLY)
  {
    for (unsigned i = 0; i < n; i++)
    {
      LD est = echoDot(hist_ + pos_, coef_, nCoef_);
      LD e = (LD)in[i] - est;
      out[i] = (float)e;
      unsigned W = histLen_ - nCoefM1_;
      unsigned p = pos_ + 1;
      pos_ = (p == W) ? 0u : p;
    }
    return;
  }

  if (st == V92_ECHO_COUNT_DELAY)
  {
    for (unsigned i = 0; i < n; i++)
      copyWord(&out[i], &in[i]);

    unsigned W = histLen_ - nCoefM1_;
    unsigned t = n + pos_;
    pos_ = (t >= W) ? t - W : t;
    unsigned c = n + count_;
    count_ = c;
    if (c < limit_)
      return;
    setState(V92_ECHO_FAST_TRAINING);
    return;
  }

  /* training states 2 / 3 (and any other value): filter, then adapt per sample */
  for (unsigned i = 0; i < n; i++)
  {
    LD est = echoDot(hist_ + pos_, coef_, nCoef_);
    LD e = (LD)in[i] - est;
    out[i] = (float)e;
    LD mu = e * (LD)beta_;
    const float* h = hist_ + pos_;
    for (unsigned k = 0; k < nCoef_; k++)
      coef_[k] = (float)((LD)h[k] * mu + (LD)coef_[k]);
    beta_ = (float)((LD)beta_ * (LD)decay_);

    unsigned W = histLen_ - nCoefM1_;
    unsigned p = pos_ + 1;
    pos_ = (p == W) ? 0u : p;
  }

  unsigned c = n + count_;
  count_ = c;
  if (c < limit_)
    return;
  setState(st == V92_ECHO_FAST_TRAINING ? V92_ECHO_SLOW_TRAINING : V92_ECHO_FILTER_ONLY);
}

/* ================================================================================================================ V90PreFilter */

/** @brief Debug message printed when a reference loop names a coefficient array type outside 1 .. 3. */
static const char* kBugMsg = "V90PreFilter: getFilterPointer BUG !!! - unsupported PreFilterCoefType !\r\n";

/** @brief The reference-loop table of the codec the filter was built for. */
static inline V90PreFilterRefLoop* loopsOf(const V90PreFilter* o) { return V90PreFilter::dataBase[o->codecType_].refLoops; }

/*
 * Coefficient rows by raw (unchecked) gain index: 80 bytes per row for the 20-tap arrays, 160 bytes per row for the 40-tap array whose rows start at
 * index 20 (the address is table + 160 * index - 3200, so an index below 20 points before the table).
 */
static inline float* rowType1(unsigned i) { return (float*)((char*)&V90PreFilter::preFilterCoefType1[0][0] + (i * 80u)); }
static inline float* rowType2(unsigned i) { return (float*)((char*)&V90PreFilter::preFilterCoefType2[0][0] + (i * 80u)); }
static inline float* rowType3(unsigned i) { return (float*)((char*)&V90PreFilter::preFilterCoefType3[0][0] + (ptrdiff_t)(int)(i * 160u - 3200u)); }
/** @brief Record count of a reference-loop table: up to the first record with an empty name. */
static inline unsigned countLoops(const V90PreFilterRefLoop* t)
{
  unsigned n = 0;
  while (t[n].name[0] != 0)
    n++;
  return n;
}
/** @brief Number of codec records of V90PreFilter::dataBase: up to the first empty name. */
static inline unsigned countCodecs()
{
  unsigned n = 0;
  while (V90PreFilter::dataBase[n].name[0] != 0)
    n++;
  return n;
}

/**
 * @brief Body of the V90PreFilter constructor (the FloatFIR base, 40 taps, is built by the initialiser list).
 * @param o       object under construction
 * @param codec   codec type used when the parameters do not force one
 * @param info    phase 2 measurements, borrowed
 * @param params  parameter block, borrowed; hwCodecType >= 0 forces the codec type (values 1 .. 15 are accepted, anything else gives 0)
 * @note A codec type beyond the last dataBase record (after the table length check) is replaced by 0 with a framed debug message.
 */
static inline void pfCtorBody(V90PreFilter* o, __tHardwareCodecTypes__ codec, V90Phase2Info* info, V90Parameters* params)
{
  o->info_ = info;
  o->params_ = params;

  /* codec type: from the constructor argument, or forced by the configuration parameter */
  unsigned lvl;
  if (params->hwCodecType < 0)
  {
    o->codecType_ = (int)codec;
    lvl = dsplibs_debug_level;
  }
  else
  {
    lvl = dsplibs_debug_level;
    if (lvl > 1)
    {
      dsplibs_debug_printf("V90PreFilter: HardwareCodecType loaded by configuration parameters\r\n");
      lvl = dsplibs_debug_level;
    }
    unsigned v = (unsigned)o->params_->hwCodecType;
    switch (v)
    {
    case 1:
      o->codecType_ = 1;
      break;
    case 2:
      o->codecType_ = 2;
      break;
    case 3:
      o->codecType_ = 3;
      break;
    case 4:
      o->codecType_ = 4;
      break;
    case 5:
      o->codecType_ = 5;
      break;
    case 6:
      o->codecType_ = 6;
      break;
    case 7:
      o->codecType_ = 7;
      break;
    case 8:
      o->codecType_ = 8;
      break;
    case 9:
      o->codecType_ = 9;
      break;
    case 10:
      o->codecType_ = 10;
      break;
    case 11:
      o->codecType_ = 11;
      break;
    case 12:
      o->codecType_ = 12;
      break;
    case 13:
      o->codecType_ = 13;
      break;
    case 14:
      o->codecType_ = 14;
      break;
    case 15:
      o->codecType_ = 15;
      break;
    default:
      o->codecType_ = 0;
      break;
    }
  }

  /* a codec type beyond the table is replaced by 0 (unknown) */
  int maxIdx = (int)countCodecs() - 1;
  if (o->codecType_ > maxIdx)
  {
    if (lvl > 1)
    {
      dsplibs_debug_printf("*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#**#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*\r\n");
      lvl = dsplibs_debug_level;
      if (lvl > 1)
      {
        dsplibs_debug_printf("V90PreFilter: External Hardware Codec Index exceeds table length (codec inx = %d, table length = %d)\r\n", o->codecType_, maxIdx);
        lvl = dsplibs_debug_level;
        if (lvl > 1)
        {
          dsplibs_debug_printf("*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#**#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*#*\r\n");
          lvl = dsplibs_debug_level;
        }
      }
    }
    o->codecType_ = 0;
  }

  if (lvl > 1)
    dsplibs_debug_printf("V90PreFilter: HardwareCodecType: %s\r\n", V90PreFilter::dataBase[o->codecType_].name);

  /* the reset() sequence: no reference loop, gain 0, coefficient array 1 */
  o->FloatFIR::reset();
  o->refLoop_ = -1;
  o->filterIdx_ = 0;
  o->FloatFIR::setCoefficients(&V90PreFilter::preFilterCoefType1[0][0], 0x14);
}

/** @brief Build the 40-tap FIR base (history extra 99) and pick the codec type and the initial coefficients (see pfCtorBody()). */
V90PreFilter::V90PreFilter(__tHardwareCodecTypes__ codec, V90Phase2Info* info, V90Parameters* params) : FloatFIR(0x28, NULL, 0x63)
{
  pfCtorBody(this, codec, info, params);
}

/** @brief Nothing to do beyond the FloatFIR base destructor. */
V90PreFilter::~V90PreFilter() {}

/** @brief Reset the FIR state, drop the reference loop, set gain index 0 and load coefficient array 1 (20 taps) row 0. */
void V90PreFilter::reset()
{
  FloatFIR::reset();
  refLoop_ = -1;
  filterIdx_ = 0;
  FloatFIR::setCoefficients(&preFilterCoefType1[0][0], 0x14);
}

/**
 * @brief Load row @p idx of the coefficient array @p type (2 = array 2, 3 = array 3 with 40 taps, anything else array 1).
 * @param type  coefficient array
 * @param idx   raw row index, not clamped (array 3 rows start at 20; filterIdx_ is then idx - 20)
 * @note No caller in the library; kept for external test harnesses.
 */
void V90PreFilter::setFilter(PreFilterCoefType type, unsigned idx)
{
  filterIdx_ = (int)idx;
  if (type == 2)
  {
    FloatFIR::setCoefficients(rowType2(idx), 0x14);
  }
  else if (type == 3)
  {
    FloatFIR::setCoefficients(rowType3(idx), 0x28);
    filterIdx_ = (int)(idx - 0x14);
  }
  else
  {
    FloatFIR::setCoefficients(rowType1(idx), 0x14);
  }
}

/**
 * @brief Tap count of the coefficient array of the current reference loop: 40 for array 3, else 20 (also with no loop set).
 * @note A coefficient type outside 1 .. 3 prints the "BUG" message.  No caller in the library; kept for external test harnesses.
 */
unsigned V90PreFilter::getFilterLength(unsigned)
{
  unsigned len = 0x14;
  int r = refLoop_;
  if (r != -1)
  {
    int type = loopsOf(this)[r].coefType;
    if (type == 2)
    {
    }
    else if (type > 2)
    {
      if (type == 3)
        len = 0x28;
      else
        edprintf(kBugMsg);
    }
    else if (type != 1)
    {
      edprintf(kBugMsg);
    }
  }
  return len;
}

/**
 * @brief Pointer to the coefficient row for gain index @p j of the current reference loop's array.
 * @param j  gain index; clamped to 30 (arrays 1 / 2) or 50 (array 3) when a loop is set, used raw with none (array 1)
 * @note A coefficient type outside 1 .. 3 prints the "BUG" message and uses array 1.  No caller in the library; kept for external test harnesses.
 */
float* V90PreFilter::getFilterPointer(unsigned j)
{
  int r = refLoop_;
  if (r == -1)
    return rowType1(j); /* no reference loop: array 1, the index unclamped */

  int type = loopsOf(this)[r].coefType;
  if (type == 2)
  {
    if (j > 0x1e)
      j = 0x1e;
    return rowType2(j);
  }
  if (type == 3)
  {
    if (j > 0x32)
      j = 0x32;
    return rowType3(j);
  }
  if (type != 1)
    edprintf(kBugMsg);
  if (j > 0x1e)
    j = 0x1e;
  return rowType1(j);
}

/**
 * @brief Load the coefficients for gain index @p idx from the array of the current reference loop (array 1 with none); no-op when already current.
 * @param idx  gain index; clamped to 30 (arrays 1 / 2) or 50 (array 3) only when a loop is set
 * @note Called by V90Demodulator for the German ISDN NT1 and PBX gains.  filterIdx_ is set to the unclamped @p idx.
 * @warning With no reference loop the index is not clamped and the row address is computed unchecked (a large index reads past array 1).
 */
void V90PreFilter::setFilter(unsigned idx)
{
  if ((unsigned)filterIdx_ == idx)
    return;

  /* tap count of the loop's array */
  int r = refLoop_;
  unsigned taps = 0x14;
  if (r != -1)
  {
    int type = loopsOf(this)[r].coefType;
    if (type == 2)
    {
    }
    else if (type <= 1)
    {
      if (type - 1 != 0)
      {
        edprintf(kBugMsg);
        r = refLoop_;
      }
    }
    else if (type == 3)
    {
      taps = 0x28;
    }
    else
    {
      edprintf(kBugMsg);
      r = refLoop_;
    }
  }

  /* the row, clamped for a known loop */
  float* ptr;
  if (r == -1)
  {
    ptr = rowType1(idx);
  }
  else
  {
    int type = loopsOf(this)[r].coefType;
    unsigned c = idx;
    if (type == 2)
    {
      if (c > 0x1e)
        c = 0x1e;
      ptr = rowType2(c);
    }
    else if (type <= 1)
    {
      if (type - 1 != 0)
        edprintf(kBugMsg);
      if (c > 0x1e)
        c = 0x1e;
      ptr = rowType1(c);
    }
    else if (type == 3)
    {
      if (c > 0x32)
        c = 0x32;
      ptr = rowType3(c);
    }
    else
    {
      edprintf(kBugMsg);
      if (c > 0x1e)
        c = 0x1e;
      ptr = rowType1(c);
    }
  }

  FloatFIR::setCoefficients(ptr, taps);
  filterIdx_ = (int)idx;
}

/** @brief True when the loop's capability is 2 (V.90 with EIA6) or the loopType parameter is 6. */
static inline bool eia6Of(const V90PreFilter* o)
{
  int a = 0;
  int r = o->refLoop_;
  if (r >= 0)
    a = (loopsOf(o)[r].capability == 2) ? 1 : 0;
  bool eia = (a == 1);
  bool loop6 = (o->params_->loopType == 6);
  return eia || loop6;
}

/** @brief Whether the pre-filter selected for this line is a V.90 EIA6 one (current reference loop capability 2, or loopType parameter 6). */
bool V90PreFilter::isV90WithEia6() const { return eia6Of(this); }

/** @brief Empty. */
void V90PreFilter::displayParamEia6() {}

/** @brief Number of reference loops of the codec's table.  @note No caller in the library; kept for external test harnesses. */
unsigned V90PreFilter::getNofRefLoops() const { return countLoops(loopsOf(this)); }

/**
 * @brief Choose the reference loop of the codec's table nearest to the measured loop and return its gain index.
 *
 * The measured loop is d[i] = l2_[14] - l2_[15 + i] (i = 0 .. 5) of the phase 2 info; the distance to a reference loop is the sum of the squared
 * differences d[k] - ref[k].  Sets refLoop_ and prints the loop name, its coefficient array and tap count, and the codec name.
 * @return the gain index (coefIndex) of the chosen loop
 * @warning If every distance is NaN or not below 1e10, refLoop_ keeps its previous value (-1 after reset()) and is then used as a table index.
 */
int V90PreFilter::autoSelection()
{
  const float* l2 = info_->l2_;
  float d[6];
  for (unsigned i = 0; i <= 5; i++)
    d[i] = (float)((LD)l2[14] - (LD)l2[15 + i]);

  LD best = (LD)bitsFloat(0x501502f9u); /* 1e10f */
  unsigned nrefs = countLoops(loopsOf(this));
  for (unsigned b = 0; b < nrefs; b++)
  {
    const V90PreFilterRefLoop* rec = &loopsOf(this)[b];
    LD acc = 0.0L;
    for (unsigned k = 0; k <= 5; k++)
    {
      LD diff = (LD)d[k] - (LD)rec->ref[k];
      acc = acc + diff * diff;
    }
    if (!(acc >= best))
    { /* a NaN distance wins */
      best = acc;
      refLoop_ = (int)b;
    }
  }

  int r = refLoop_;
  edprintf("V90PreFilter: loop = %s (%d)\r\n", loopsOf(this)[r].name, r);
  int type = loopsOf(this)[refLoop_].coefType;
  edprintf("V90PreFilter: Pre Filter Coeffs Type array %d, (filter length %d)\r\n", type, (type == 3) ? 0x28 : 0x14);
  edprintf("V90PreFilter: HardwareCodecType: %s\r\n", dataBase[codecType_].name);
  return loopsOf(this)[refLoop_].coefIndex;
}

/**
 * @brief Load the coefficients of array @p type for @p gain (selectFilter's connection-type and forced paths).
 * @param gain  gain index, NOT clamped here (the callers clamp it where required); array 3 rows start at index 20 and filterIdx_ becomes gain - 20
 */
static inline void loadByType(V90PreFilter* o, int type, int gain)
{
  if (type == 2)
  {
    o->setCoefficients(rowType2((unsigned)gain), 0x14);
  }
  else if (type == 3)
  {
    o->setCoefficients(rowType3((unsigned)gain), 0x28);
    o->filterIdx_ = gain - 0x14;
  }
  else
  {
    o->setCoefficients(rowType1((unsigned)gain), 0x14);
  }
}

/**
 * @brief Select and load the pre-filter coefficients for the line and print the gain.
 *
 * Sets refLoop_ to 0 and then follows one of three paths.  (1) lineConnectionType 1 / 2 (German ISDN NT1 box / PBX): the box's gain parameter is used
 * unclamped with the coefficient array of the codec's first reference loop.  (2) preFilterGain == -1: autoSelection() picks the loop and the array
 * and gain follow it (the gain is clamped to 30 / 50).  (3) otherwise a forced gain: the array comes from preFilterCoefType (1 .. 3, -1 = the codec's
 * first reference loop, anything else array 1) and the gain is capped at 30 (arrays 1 / 2, no lower bound) or limited to 20 .. 50 (array 3).
 * @warning Paths 1 and 2 index the coefficient tables by the parameter value unchecked in places (see the file notes on clamping).
 */
void V90PreFilter::selectFilter()
{
  V90Parameters* p = params_;
  refLoop_ = 0;
  int lineType = p->lineConnectionType;
  int gain;
  if (lineType == 1 || lineType == 2)
  {
    if (lineType == 1)
      edprintf("V90PreFilter: Connection Type is ISDN NT1 BOX (4.2kHz Null)\r\n");
    else
      edprintf("V90PreFilter: Connection Type is PBX ISDN BOX (4kHz Null)\r\n");
    gain = (lineType == 1) ? params_->germanIsdnNt1BoxFilterGain : params_->germanPbxPreFilterGain;
    int type = dataBase[codecType_].refLoops[0].coefType;
    filterIdx_ = gain;
    loadByType(this, type, gain);
  }
  else if (p->preFilterGain == -1)
  { /* automatic: the nearest reference loop, loaded the way setFilter(idx) does */
    gain = autoSelection();
    if (filterIdx_ != gain)
    {
      int r = refLoop_;
      unsigned taps = 0x14;
      if (r != -1)
      {
        int type = loopsOf(this)[r].coefType;
        if (type == 2)
        {
        }
        else if (type > 2)
        {
          if (type == 3)
            taps = 0x28;
          else
          {
            edprintf(kBugMsg);
            r = refLoop_;
          }
        }
        else if (type - 1 != 0)
        {
          edprintf(kBugMsg);
          r = refLoop_;
        }
      }
      float* ptr;
      if (r == -1)
      {
        ptr = rowType1((unsigned)gain);
      }
      else
      {
        int type = loopsOf(this)[r].coefType;
        unsigned c = (unsigned)gain;
        if (type == 2)
        {
          if (c > 0x1e)
            c = 0x1e;
          ptr = rowType2(c);
        }
        else if (type > 2)
        {
          if (type == 3)
          {
            if (c > 0x32)
              c = 0x32;
            ptr = rowType3(c);
          }
          else
          {
            edprintf(kBugMsg);
            if (c > 0x1e)
              c = 0x1e;
            ptr = rowType1(c);
          }
        }
        else
        {
          if (type - 1 != 0)
            edprintf(kBugMsg);
          if (c > 0x1e)
            c = 0x1e;
          ptr = rowType1(c);
        }
      }
      FloatFIR::setCoefficients(ptr, taps);
      filterIdx_ = gain;
    }
  }
  else
  { /* forced gain, and a forced or codec-given coefficient array */
    int ct = p->preFilterCoefType;
    int type;
    if (ct == 1)
    {
      edprintf("V90PreFilter: Force coeff type array 1\r\n");
      type = 1;
    }
    else if (ct == 2)
    {
      edprintf("V90PreFilter: Force coeff type array 2\r\n");
      type = 2;
    }
    else if (ct == 3)
    {
      edprintf("V90PreFilter: Force coeff type array 3\r\n");
      type = 3;
    }
    else if (ct == -1)
    {
      int t0 = dataBase[codecType_].refLoops[0].coefType;
      edprintf("V90PreFilter: Using coeff type array according to codec type\r\n");
      type = t0;
    }
    else
    {
      type = 1;
    }

    int g = params_->preFilterGain;
    if (type == 3)
    {
      if (g > 0x31)
        gain = 0x32;
      else if (g > 0x13)
        gain = g;
      else
        gain = 0x14;
    }
    else
    {
      gain = (g > 0x1d) ? 0x1e : g;
    }
    filterIdx_ = gain;
    loadByType(this, type, gain);
  }

  edprintf("V90PreFilter: Filter Gain = %d\r\n", gain);
}

/**
 * @brief V.90 capability of the current reference loop; runs autoSelection() first when none is set.
 * @return 1 when the loop has the EIA6 capability or the loopType parameter is 6, else the loop's capability field
 */
int V90PreFilter::getV90Capability()
{
  int r = refLoop_;
  int a = 0;
  if (r < 0)
  {
    autoSelection();
    r = refLoop_;
    a = 0;
    if (r >= 0)
      a = (loopsOf(this)[r].capability == 2) ? 1 : 0;
  }
  else
  {
    a = (loopsOf(this)[r].capability == 2) ? 1 : 0;
  }
  if (a == 1 || params_->loopType == 6)
    return 1;
  return loopsOf(this)[r].capability;
}

/** @brief Copy a float member of the parameter block as a 32-bit word (no FPU load, so a signalling NaN keeps its payload). */
#define CPYW(dst, src) copyWord(&c->dst, &c->src)

/**
 * @brief Copy the EIA6 variants of the parameters over the normal ones.
 *
 * Covers the equalizer / spectral verifier / restricted-Dmin enables, the spectral shaper filter, the linear equalizer and DFE betas, the number of
 * ucodes, the fade-edge settings, the retrain limit, the Phase 3 / 4 PDSNR thresholds and the TRN1d V.34 fallback error.  The BLL timing constants
 * and the initial baud offset are replaced only when the previous clock deviation (the integer `params->clockDeviation` times 0.001) is non-zero.
 * @note Called by V90Demodulator when isV90WithEia6() is true.
 */
void V90PreFilter::setParamEia6()
{
  edprintf("V90PreFilter: setParamEIA6 called\r\n");
  V90Parameters* c = params_;

  /* equalizer, spectral shaper and DFE / linear equalizer settings */
  c->enableEqualizerMmx = c->eia6EnableEqualizerMmx;
  c->spectralVerifierEnable = c->eia6SpectralVerifierEnable;
  c->useRestricedDmin = c->eia6UseRestricedDmin;
  CPYW(spectralShaperA1, eia6SpectralShaperA1);
  CPYW(spectralShaperA2, eia6SpectralShaperA2);
  CPYW(spectralShaperB1, eia6SpectralShaperB1);
  CPYW(spectralShaperB2, eia6SpectralShaperB2);
  c->spectralShaperSr = c->eia6SpectralShaperSr;
  c->spectralShaperId = c->eia6SpectralShaperId;
  CPYW(linearEquTrn1dBeta, eia6LinearEquTrn1dBeta);
  CPYW(linearEquDilBeta, eia6LinearEquDilBeta);
  CPYW(linearEquTrn2dInitialBeta, eia6LinearEquTrn2dInitialBeta);
  CPYW(linearEquTrn2dBeta, eia6LinearEquTrn2dBeta);
  CPYW(linearEquDataBeta, eia6LinearEquDataBeta);
  CPYW(linearEquDilMedUcodeBeta, eia6LinearEquDilMedUcodeBeta);
  CPYW(linearEquDilHighUcodeBeta, eia6LinearEquDilHighUcodeBeta);
  CPYW(dfeDilMedUcodeBeta, eia6DfeDilMedUcodeBeta);
  CPYW(dfeDilHighUcodeBeta, eia6DfeDilHighUcodeBeta);
  c->nofUcodes = c->eia6NofUcodes;
  CPYW(linearEquDilErrorRelaxBeta, eia6LinearEquDilErrorRelaxBeta);
  CPYW(dfeDilErrorRelaxBeta, eia6DfeDilErrorRelaxBeta);
  c->linearEquFadeEdgesCycle = c->eia6LinearEquFadeEdgesCycle;
  CPYW(linearEquFadeLeftEdgeRatio, eia6LinearEquFadeLeftEdgeRatio);
  CPYW(linearEquFadeRightEdgeRatio, eia6LinearEquFadeRightEdgeRatio);
  CPYW(dfeDilBeta, eia6DfeDilBeta);
  CPYW(dfeDataBeta, eia6DfeDataBeta);

  /* the previous clock deviation: the integer in the modem parameters times 0.001 (an extended product), printed with four decimals */
  int32_t raw = c->params->clockDeviation;
  LD cd = (LD)raw * (LD)bitsFloat(0x3a83126fu);
  float cdF = (float)cd;
  Split s = splitValue(cd, 10000.0L);
  edprintf("V90PreFilter: prev params ClockDeviation is = %c%d.%04d\r\n", s.sign, s.ip, s.fp);
  if ((LD)cdF < 0.0L || (LD)cdF > 0.0L)
  { /* non-zero (a NaN counts as zero here): take the EIA6 timing constants */
    edprintf("V90PreFilter: Setting timing parameters (registry)...\r\n");
    c = params_;
    c->initialBaudOffset = cdF;
    CPYW(bllInitialK1, eia6BllInitialK1);
    CPYW(bllInitialK2, eia6BllInitialK2);
    CPYW(bllFastK1, eia6BllFastK1);
    CPYW(bllFastK2, eia6BllFastK2);
    CPYW(bllMediumK1, eia6BllMediumK1);
    CPYW(bllMediumK2, eia6BllMediumK2);
    CPYW(bllSlowK1, eia6BllSlowK1);
    CPYW(bllSlowK2, eia6BllSlowK2);
    CPYW(bllSlow2K1, eia6BllSlow2K1);
    CPYW(bllSlow2K2, eia6BllSlow2K2);
    CPYW(bllDilK1, eia6BllDilK1);
    CPYW(bllDilK2, eia6BllDilK2);
    CPYW(bllTrn2InitialK1, eia6BllTrn2InitialK1);
    CPYW(bllTrn2InitialK2, eia6BllTrn2InitialK2);
    CPYW(bllTrn2K1, eia6BllTrn2K1);
    CPYW(bllTrn2K2, eia6BllTrn2K2);
    CPYW(bllSteadyStateK1, eia6BllSteadyStateK1);
    CPYW(bllSteadyStateK2, eia6BllSteadyStateK2);
  }
  else
  {
    c = params_;
  }

  /* retrain limit and thresholds */
  c->maxNofV90Retrains = c->eia6MaxNofV90Retrains;
  CPYW(pdsnrThresholdInPhase3, eia6PdsnrThresholdInPhase3);
  CPYW(pdsnrThresholdInPhase4, eia6PdsnrThresholdInPhase4);
  CPYW(trn1dErrorForV34Fallback, eia6Trn1dErrorForV34Fallback);
}
