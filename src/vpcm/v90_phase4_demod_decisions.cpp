/**
 * @file v90_phase4_demod_decisions.cpp
 * @brief V90Phase4Demodulator::getV90Decision() and getV92Decision(): the V.90 and V.92 phase 4 receive state machines.
 *
 * Each function runs ONE received symbol (x, a 16-bit PCM level) through the state machine of phase 4 and returns the DECISION symbol: the hard
 * decision of the V90Demapper in the decoding states (3, 5 .. 7, 9, 16), x itself in the detector states, 0 in the terminated states (8 / 17).
 * getDecision() (v90_phase4_demod.cpp) selects the function by `session_`; reset() calls them with a zero symbol to settle the machine.  The class and
 * the state numbers are described in v90_phase4_demod.h; each state is a small static inline function here, and both machines share the states that do
 * not differ (0 Ri, 1 RiNot, 2 / 3 TRN2d known data, 7 B1d, 10 .. 13 the RRN silence measurement).
 *
 * Per state:
 * - 0 wait for Ri (detectR; -> 1); 1 wait for RiNot (detectRNot; -> 2 and the length of the demapper's linear mapping study by quickConnect_);
 * - 2 / 3 TRN2d known data: the demapper's linear mapping study is enabled when cnt_ reaches studyEnd_ and disabled at limit_; the events fire at
 *   limit_ / 2 and at limit_;
 * - 4 (V.92) wait for the CP: every descrambled byte goes through V90CP::bitsToInfo() (kinds CP, CPt, SUV, SUVt, Ed); 5 (V.90) wait for the MP
 *   (V90MP::bitsToInfo(): MP, MPnot); 6 (V.90) wait for Ed; the Ed ends in B1d (7) or, for a rate renegotiation with silence (RRN), in the echo measurement;
 * - 7 B1d: counts the descrambled bits and the zeros after the start-up delay (3 * kBits + 0x17 bits); at 0x120 symbols it prints the bit error ratio
 *   and ends in state 8;
 * - 9 TRN2d DD: wait for RdNot (-> 3 with the RRN length, CP / MP and demapper reset);
 * - 10 .. 13 the RRN silence echo energy measurement (10 wait, 11 accumulate the error energy before the echo cancellation, 12 wait, 13 accumulate after;
 *   the means and the SCR echo energy in dB are printed and keepRate_ is set); 14 wait for Rt, 15 wait for RtNot (-> 5 / 4 and the MP / CP reset),
 *   16 (V.92) wait for RfNot (-> 4).
 *
 * Callees from other files: V90Demapper (hardDecision, process, incrementRBSFramePosition, resetLinearMappStudy, resetNoSpectral, linearMappingStudy),
 * V90RDetector (detectR, detectRNot, detectRfNot, reset), V90MP / V90CP (reset, bitsToInfo, printNofRecievedMpMpNot),
 * Descrambler<unsigned char, int>::process(), edprintf() and dsplibs_debug_printf().
 *
 * Numeric conventions:
 * - the symbol counter is incremented BEFORE the dispatch and event_ is cleared first; the order of the stores to state_ / cnt_ / event_ relative to the
 *   prints and callee calls is part of the behaviour (tests compare the object at every print and every callee call);
 * - the states 4 and 16 (V.90) and 5, 6 (V.92) and every state above 16 other than 17 return 0 without further action;
 * - the echo energy and BER prints are computed in extended precision (`long double`): integer part = truncation of |v|, fraction = truncation of
 *   (v - trunc(v)) * K made positive (INT_MIN stays INT_MIN); the sign is '+' when 0 < v or the value is unordered; the BER's sign comes from
 *   (1 / denominator) * numerator but its digits from numerator / denominator (two different expressions); the accumulators are FLOAT members (rounded at
 *   every store);
 * - `cnt_ % 6 == 0` tests and the unsigned compares of cnt_ with parameter words; the threshold `rrnSilenceScrLength - 3 * rrnSilenceEchoCalcPeriod` is
 *   computed in 32-bit wrapping arithmetic.
 *
 * @warning V90Demapper::process() writes into the 1176-byte `buf_` and reports the count in `nBytes_`; the loops here trust that count.
 */

#include "v90_phase4_demod.h"
#include "core/dsp_math.h"
#include "core/dsp_host.h"
#include "v34/v34_api.h"

extern "C"
{
}

typedef P37_V90Phase4Demodulator D;
typedef long double LD;

#define PFX "V90Phase4Demodulator: "

/* ------------------------------------------------------------------------------------------------ helpers (static inline) */

#include "core/dsp_floatbits.h"
/** @brief Round an extended-precision value to float and back (through a volatile temporary, so the rounding really happens). */
static inline float roundF(LD v)
{
  volatile float slot = (float)v;
  return slot;
}

#include "core/dsp_intmath.h"

#include "vpcm_print_helpers.h"

/** @brief Fraction part of a printed value: |trunc((v - trunc(v)) * k)|, k = 10^(number of printed places). */
static inline int fracPart(LD v, LD k)
{
  int ip = dsp_trunc_i32(v);
  return absI32(dsp_trunc_i32((v - (LD)ip) * k));
}

/** @brief Unsigned 32-bit value as an extended-precision number. */
static inline LD u64ld(unsigned v) { return (LD)(unsigned long long)v; }

/** @brief Print `fmt` with the square root of the float energy e as sign, integer part and four fraction places (the sign is that of e). */
static inline void printSqrtEnergy(const char* fmt, float e)
{
  LD s = dsp_sqrt((LD)e);
  int ip = intPart(s);
  int fr = fracPart(s, (LD)10000.0f);
  edprintf(fmt, signChar((LD)e), ip, fr);
}

/**
 * @brief Hard decision of one symbol, then the demapper's bit extraction for the frame.
 * @param r  receives the result of V90Demapper::process() (non-zero when bytes are in buf_ / nBytes_)
 * @return the hard decision
 */
static inline short decodeStep(D* t, short x, unsigned* r)
{
  short hd = t->dem_->hardDecision(x);
  *r = t->dem_->process(t->buf_, t->nBytes_);
  return hd;
}

/** @brief State 0: wait for Ri; on detection go to state 1 and raise V90DEMOD_STATE_RI_DETECTED.  @return x */
static inline short stRi(D* t, short x)
{
  if (!t->r_.detectR(x))
    return x;
  edprintf(PFX "Ri detected @ %d, polarity = %d\r\n", t->cnt_, t->r_.dir_);
  t->state_ = P4D_WAIT_RI_NOT;
  t->cnt_ = 0;
  t->event_ = V90DEMOD_STATE_RI_DETECTED;
  return x;
}

/**
 * @brief State 1: wait for RiNot; on detection go to state 2 and start the demapper's linear mapping study (quick connect: the shorter 0x960 length, study
 *   end at 0x258). @return x
 */
static inline short stRiNot(D* t, short x)
{
  if (!t->r_.detectRNot(x))
    return x;
  edprintf(PFX "RiNot detected @ %d, enter TRN2dKnownData state\r\n", t->cnt_);
  t->state_ = P4D_TRN2D_ENTRY;
  unsigned w = t->quickConnect_;
  t->cnt_ = 0;
  t->event_ = V90DEMOD_STATE_RINOT_DETECTED;

  /* restart the demapper's linear mapping study: normal or quick-connect length */
  if (w == 0)
  {
    t->dem_->resetLinearMappStudy(0x1c20);
    t->studyEnd_ = 0x7d0;
  }
  else
  {
    t->dem_->resetLinearMappStudy(0x960);
    t->studyEnd_ = 0x258;
  }
  return x;
}

/** @brief States 2 / 3: TRN2d known data; hard decision plus the demapper's linear mapping study (on from studyEnd_ to limit_).  @return the hard decision */
static inline short stKnown(D* t, short x)
{
  V90Demapper* d = t->dem_;
  short hd = d->hardDecision(x);

  /* switch the linear mapping study on at studyEnd_ and feed it while on */
  if (t->cnt_ == t->studyEnd_)
  {
    d = t->dem_;
    d->linMapStudyOn_ = 1;
    if (dsplibs_debug_level > 1)
      dsplibs_debug_printf("V90Phase4Demodulator reset & enable linear mapping study in TRN2.\n");
  }
  d = t->dem_;
  if (d->linMapStudyOn_ != 0)
    d->linearMappingStudy(x, hd);
  t->dem_->process(t->buf_, t->nBytes_);

  /* events at half of the TRN2d data and at its end, where the study is switched off */
  unsigned lim = t->limit_;
  unsigned c = t->cnt_;
  if (c == (lim >> 1))
    t->event_ = V90DEMOD_STATE_MIDDLE_OF_TRN2D;
  if (c == lim)
  {
    t->event_ = V90DEMOD_STATE_TRN2D_RECEIVED;
    t->dem_->linMapStudyOn_ = 0;
    if (dsplibs_debug_level > 1)
      dsplibs_debug_printf(PFX "disable linear mapping study\n");
  }
  return hd;
}

/**
 * @brief State 7: B1d.  The descrambled bits are counted; the zeros after the start-up delay of 3 * mp2_->kBits_ + 0x17 bits count as errors.  At 0x120
 *        symbols the bit error ratio is printed and the machine goes to state 8.
 * @return the hard decision
 * @warning If b1dBits_ is not above the start-up delay when the BER is printed, the divisor is 0 (the print shows inf / NaN) or wraps to a huge value (the
 *          print shows 0).  Debug output only.
 */
static inline short stB1d(D* t, short x)
{
  unsigned r;
  short hd = decodeStep(t, x, &r);

  /* descramble the produced bytes; count the bits and the zeros after the delay */
  if (r != 0 && t->nBytes_ != 0)
  {
    unsigned i = 0;
    do
    {
      int b = t->descr_->process(t->buf_[i]);
      unsigned bits = t->b1dBits_ + 1;
      t->b1dBits_ = bits;
      if (b == 0 && bits > 3u * t->mp2_->kBits_ + 0x17u)
        t->b1dZeros_++;
      i++;
    } while (t->nBytes_ > i);
  }

  /* the end of B1d at 0x120 symbols: print the counts and the BER */
  if (t->cnt_ != 0x120)
    return hd;
  edprintf(PFX "Phase4 Terminated @ %d,  nof B1d bits = %d,  B1d Zeros (after delay) = %d\r\n", 0x120, t->b1dBits_, t->b1dZeros_);
  {
    LD num = u64ld(t->b1dZeros_);
    LD q = num / u64ld(t->b1dBits_ - 3u * t->mp2_->kBits_ - 0x17u);
    int ip = intPart(q);
    int fr = fracPart(q, (LD)100000000.0f);
    LD rr = (1.0L / u64ld(t->b1dBits_ - 3u * t->mp2_->kBits_ - 0x17u)) * num;
    edprintf(PFX "B1d BER = %c%d.%08d\r\n", signChar(rr), ip, fr);
  }

  t->event_ = V90DEMOD_STATE_PHASE4_TERMINATED;
  t->state_ = P4D_TERMINATED;
  t->cnt_ = 0;
  t->rrnPending_ = 0;
  return hd;
}

/**
 * @brief State 10: wait (rrnSilenceWaitBeforeEchoCalc symbols, then a multiple of 6) before the error energy before the echo cancellation is measured.
 *   @return x
 */
static inline short stSilenceWait(D* t, short x)
{
  t->dem_->incrementRBSFramePosition();
  if (t->cnt_ < (unsigned)t->params_->rrnSilenceWaitBeforeEchoCalc)
    return x;
  if (t->cnt_ % 6 != 0)
    return x;
  edprintf(PFX "entering CalcErrorEnergyBeforeEchoCancellation state @ %d\r\n", t->cnt_);
  t->state_ = P4D_ECHO_BEFORE;
  t->cnt_ = 0;
  return x;
}

/**
 * @brief State 11: accumulate the error energy (x * x) before the echo cancellation; at the end of the echo-calculation period (a multiple of 6 symbols) form
 *   the mean and go to state 12. @return x
 */
static inline short stEchoBefore(D* t, short x)
{
  t->dem_->incrementRBSFramePosition();
  int sq = (int)x * (int)x;
  LD acc = (LD)sq + (LD)loadF(&t->echoBefore_);
  unsigned c = t->cnt_;

  /* at the end of the period store and print the mean, else keep accumulating */
  if (c >= (unsigned)t->params_->rrnSilenceEchoCalcPeriod && c % 6 == 0)
  {
    float e = storeF(&t->echoBefore_, acc / u64ld(c));
    edprintf(PFX "entering WaitForEchoCancellation state @ %d\r\n", c);
    printSqrtEnergy(PFX "error energy before echo cancellation  = %c%d.%04d\r\n", e);
    t->state_ = P4D_ECHO_WAIT;
    t->cnt_ = 0;
    return x;
  }
  storeF(&t->echoBefore_, acc);
  return x;
}

/**
 * @brief State 12: wait for the echo canceller (until rrnSilenceScrLength - 3 * rrnSilenceEchoCalcPeriod symbols, then a multiple of 6), then go to state 13.
 *   @return x
 */
static inline short stEchoWait(D* t, short x)
{
  t->dem_->incrementRBSFramePosition();
  V90Parameters* p = t->params_;
  unsigned thr = (unsigned)p->rrnSilenceScrLength - 3u * (unsigned)p->rrnSilenceEchoCalcPeriod;
  if (t->cnt_ < thr)
    return x;
  if (t->cnt_ % 6 != 0)
    return x;
  edprintf(PFX "entering CalcErrorEnergyAfterEchoCancellation state @ %d\r\n", t->cnt_);
  t->state_ = P4D_ECHO_AFTER;
  t->cnt_ = 0;
  t->echoAfter_ = 0.0f;
  return x;
}

/**
 * @brief State 13: accumulate the error energy after the echo cancellation. At the end of the period: the mean, the SCR echo energy [dB] =
 *        10 * log10(before / after), the keep-rate flag (set when that energy exceeds rrnSilenceMinEchoEnergyForKeepRate) and the move to state 14.
 * @return x
 * @note An all-zero "after" energy gives an infinite ratio (printed as is) and sets keepRate_.
 */
static inline short stEchoAfter(D* t, short x)
{
  t->dem_->incrementRBSFramePosition();
  int sq = (int)x * (int)x;
  LD acc = (LD)sq + (LD)loadF(&t->echoAfter_);
  unsigned c = t->cnt_;

  /* at the end of the period store and print the mean and the echo energy gain, else keep accumulating */
  if (c >= (unsigned)t->params_->rrnSilenceEchoCalcPeriod && c % 6 == 0)
  {
    float e = storeF(&t->echoAfter_, acc / u64ld(c));
    edprintf(PFX "entering WaitForRt state @ %d\r\n", c);
    printSqrtEnergy(PFX "error energy after echo cancellation  = %c%d.%04d\r\n", e);

    LD ratio = ((LD)1.0f / (LD)loadF(&t->echoAfter_)) * (LD)loadF(&t->echoBefore_);
    float d = roundF(dsp_log10(ratio) * (LD)10.0f);
    edprintf(PFX "silence SCR echo energy [dB]  = %c%d.%04d\r\n", signChar((LD)d), intPart((LD)d), fracPart((LD)d, (LD)10000.0));

    t->event_ = V90DEMOD_STATE_WAIT_FOR_RT_ENTERED;
    t->state_ = P4D_WAIT_RT;
    t->keepRate_ = !((LD)t->params_->rrnSilenceMinEchoEnergyForKeepRate >= (LD)d);
    t->cnt_ = 0;
    return x;
  }
  storeF(&t->echoAfter_, acc);
  return x;
}

/* ------------------------------------------------------------------------------------------------ the V.90 states */

/**
 * @brief State 9 (V.90): TRN2d DD, wait for RdNot; then back to TRN2d known data with the RRN length (rrnTrn2dDdLength), the MP and the demapper reset.
 *   @return the hard decision
 */
static inline short stV90RdNot(D* t, short x)
{
  unsigned r;
  short hd = decodeStep(t, x, &r);
  if (!t->r_.detectRNot(hd))
    return hd;
  edprintf(PFX "RdNot detected @ %d, enter TRN2d DD state\r\n", t->cnt_);

  /* back to TRN2d known data with the RRN length and a fresh study */
  t->dem_->resetLinearMappStudy(0x1c20);
  t->cnt_ = 0;
  t->studyEnd_ = 0x7d0;
  V90Parameters* p = t->params_;
  t->state_ = P4D_TRN2D_KNOWN;
  unsigned len = (unsigned)p->rrnTrn2dDdLength;
  t->event_ = V90DEMOD_STATE_RDNOT_DETECTED;
  t->limit_ = len;

  /* fresh MP and demapper for the new known data */
  t->mp_->reset();
  t->mp_->trn2dD_ = t->mp1_->kBits_;
  t->dem_->resetNoSpectral(t->mp1_);
  return hd;
}

/** @brief State 14 (V.90): wait for Rt; on detection go to state 15 and clear rtPending_.  @return x */
static inline short stV90Rt(D* t, short x)
{
  t->dem_->incrementRBSFramePosition();
  if (!t->r_.detectR(x))
    return x;
  edprintf(PFX "Rt detected @ %d, polarity = %d\r\n", t->cnt_, t->r_.dir_);
  t->state_ = P4D_WAIT_RT_NOT;
  t->cnt_ = 0;
  t->rtPending_ = 0;
  return x;
}

/** @brief State 15 (V.90): wait for RtNot; then back to the wait for the MP (state 5) with the MP reset (the demapper is not reset).  @return x */
static inline short stV90RtNot(D* t, short x)
{
  t->dem_->incrementRBSFramePosition();
  if (!t->r_.detectRNot(x))
    return x;
  edprintf(PFX "RtNot detected @ %d\r\n", t->cnt_);
  edprintf(PFX "enter WaitForMP state @ %d\r\n", t->cnt_);
  t->state_ = P4D_WAIT_MP;
  t->cnt_ = 0;
  t->event_ = V90DEMOD_STATE_RTNOT_DETECTED;
  t->mp_->reset();
  t->mp_->trn2dD_ = t->mp1_->kBits_;
  edprintf(PFX "No reset to demapper, current Phase - %d\r\n", t->dem_->phase_);
  return x;
}

/**
 * @brief State 5 (V.90): wait for the MP; every descrambled byte goes through V90MP::bitsToInfo() (MP / MPnot raise their events). @return the hard decision
 */
static inline short stV90WaitMP(D* t, short x)
{
  unsigned r;
  short hd = decodeStep(t, x, &r);
  if (r == 0)
    return hd;
  if (t->nBytes_ == 0)
    return hd;
  unsigned i = 0;
  do
  {
    int b = t->descr_->process(t->buf_[i]);
    int k = t->mp_->bitsToInfo(b);
    if (k == MPINFO_MP)
    {
      t->event_ = V90DEMOD_STATE_MP_DETECTED;
      edprintf(PFX "MP detected @ %d\r\n", t->cnt_);
    }
    else if (k == MPINFO_MPNOT)
    {
      t->event_ = V90DEMOD_STATE_MPNOT_DETECTED;
      edprintf(PFX "MPnot detected @ %d\r\n", t->cnt_);
    }
    i++;
  } while (t->nBytes_ > i);
  return hd;
}

/**
 * @brief State 6 (V.90): wait for Ed (MP info kind 3). With the RRN silence requested and the RRN and Rt flags set the echo measurement follows (state 10),
 *        else B1d (state 7).
 * @return the hard decision
 */
static inline short stV90WaitEd(D* t, short x)
{
  unsigned r;
  short hd = decodeStep(t, x, &r);
  if (r == 0)
    return hd;
  if (t->nBytes_ == 0)
    return hd;
  unsigned i = 0;
  do
  {
    int b = t->descr_->process(t->buf_[i]);
    int k = t->mp_->bitsToInfo(b);

    /* an MPnot is only reported; the Ed leaves the state */
    if (k == MPINFO_MPNOT)
    {
      t->event_ = V90DEMOD_STATE_MPNOT_DETECTED;
      if (dsplibs_debug_level > 2)
        dsplibs_debug_printf(PFX "MPnot detected on WaitForEd @ %d\r\n", t->cnt_);
    }
    else if (k == MPINFO_ED)
    {
      if (t->state_ == P4D_WAIT_ED)
      {
        t->event_ = V90DEMOD_STATE_ED_DETECTED;
        t->mp_->printNofRecievedMpMpNot();

        /* RRN with silence: measure the echo energy first (restart the detectors for the later Rt); otherwise start B1d */
        if (t->ce_->rrnSilence_ != 0 && t->rrnPending_ != 0 && t->rtPending_ != 0)
        {
          edprintf(PFX "Ed detected @ %d, enter wait for Rt state\r\n", t->cnt_);
          t->cnt_ = 0;
          t->echoBefore_ = 0.0f;
          t->state_ = P4D_SILENCE_WAIT;
          t->r_.reset((unsigned)t->params_->rrnRDetectionLength, 0x18);
          t->rf_.reset(0xb4, 0xc);
        }
        else
        {
          edprintf(PFX "Ed detected @ %d, enter B1d state\r\n", t->cnt_);
          t->state_ = P4D_B1D;
          t->cnt_ = 0;
          t->dem_->resetNoSpectral(t->mp2_);
          t->b1dZeros_ = 0;
          t->b1dBits_ = 0;
        }
      }
    }
    i++;
  } while (t->nBytes_ > i);
  return hd;
}

/** @brief See the declaration and the file header: the V.90 dispatcher on state_. */
short V90Phase4Demodulator::getV90Decision(short x)
{
  D* t = this;
  unsigned st = t->state_;

  /* count the symbol, clear the event, run the handler of the current state */
  t->cnt_++;
  t->event_ = 0;
  switch (st)
  {
  case P4D_WAIT_RI:
    return stRi(t, x);
  case P4D_WAIT_RI_NOT:
    return stRiNot(t, x);
  case P4D_TRN2D_ENTRY:
    t->state_ = P4D_TRN2D_KNOWN; /* fall through */
  case P4D_TRN2D_KNOWN:
    return stKnown(t, x);
  case P4D_WAIT_MP:
    return stV90WaitMP(t, x);
  case P4D_WAIT_ED:
    return stV90WaitEd(t, x);
  case P4D_B1D:
    return stB1d(t, x);
  case P4D_TRN2D_DD:
    return stV90RdNot(t, x);
  case P4D_SILENCE_WAIT:
    return stSilenceWait(t, x);
  case P4D_ECHO_BEFORE:
    return stEchoBefore(t, x);
  case P4D_ECHO_WAIT:
    return stEchoWait(t, x);
  case P4D_ECHO_AFTER:
    return stEchoAfter(t, x);
  case P4D_WAIT_RT:
    return stV90Rt(t, x);
  case P4D_WAIT_RT_NOT:
    return stV90RtNot(t, x);
  case P4D_TERMINATED:
  case 17:
    return 0;
  default:
    return 0; /* states 4, 16 and > 17: no output */
  }
}

/* ------------------------------------------------------------------------------------------------ the V.92 states */

/**
 * @brief State 9 (V.92): TRN2d DD, wait for RdNot; then back to TRN2d known data with the RRN length, the CP (cptFlag_ cleared) and the demapper reset.
 *   @return the hard decision
 */
static inline short stV92RdNot(D* t, short x)
{
  unsigned r;
  short hd = decodeStep(t, x, &r);
  if (!t->r_.detectRNot(hd))
    return hd;
  edprintf(PFX "RdNot detected @ %d, enter TRN2d DD state\r\n", t->cnt_);

  /* back to TRN2d known data with the RRN length and a fresh study */
  t->dem_->resetLinearMappStudy(0x1c20);
  t->cnt_ = 0;
  t->studyEnd_ = 0x7d0;
  V90Parameters* p = t->params_;
  t->state_ = P4D_TRN2D_KNOWN;
  unsigned len = (unsigned)p->rrnTrn2dDdLength;
  t->event_ = V90DEMOD_STATE_RDNOT_DETECTED;
  t->limit_ = len;

  /* fresh CP and demapper for the new known data */
  t->cp_->reset();
  unsigned d = t->mp1_->kBits_;
  t->cp_->cptFlag_ = 0;
  t->cp_->trn2dD_ = d;
  t->cpSeen_ = 0;
  t->dem_->resetNoSpectral(t->mp1_);
  return hd;
}

/** @brief State 14 (V.92): wait for Rt; on detection go to state 15, raise V90DEMOD_STATE_RT_DETECTED and clear rtPending_.  @return x */
static inline short stV92Rt(D* t, short x)
{
  t->dem_->incrementRBSFramePosition();
  if (!t->r_.detectR(x))
    return x;
  edprintf(PFX "Rt detected @ %d, polarity = %d\r\n", t->cnt_, t->r_.dir_);
  t->event_ = V90DEMOD_STATE_RT_DETECTED;
  t->state_ = P4D_WAIT_RT_NOT;
  t->cnt_ = 0;
  t->rtPending_ = 0;
  return x;
}

/** @brief State 15 (V.92): wait for RtNot; then back to the wait for the CP (state 4) with the CP reset (the demapper is not reset).  @return x */
static inline short stV92RtNot(D* t, short x)
{
  t->dem_->incrementRBSFramePosition();
  if (!t->r_.detectRNot(x))
    return x;
  edprintf(PFX "RtNot detected @ %d\r\n", t->cnt_);
  t->state_ = P4D_WAIT_CP;
  t->cnt_ = 0;
  t->event_ = V90DEMOD_STATE_RTNOT_DETECTED;
  t->cp_->reset();
  t->cp_->trn2dD_ = t->mp1_->kBits_;
  edprintf(PFX "No reset to demapper, current Phase - %d\r\n", t->dem_->phase_);
  return x;
}

/**
 * @brief State 16 (V.92): wait for RfNot; then back to the wait for the CP (state 4) with the CP reset.
 * @return the hard decision
 * @note trn2dD_ is taken from mp2_->kBits_ here, whereas the other CP / MP resets use mp1_->kBits_; possibly an inconsistency (not changed).
 */
static inline short stV92RfNot(D* t, short x)
{
  unsigned r;
  short hd = decodeStep(t, x, &r);
  if (!t->rf_.detectRfNot(hd))
    return hd;
  edprintf(PFX "RfNot detected @ %d, enter WaitForCPu state\r\n", t->cnt_);
  t->state_ = P4D_WAIT_CP;
  t->cnt_ = 0;
  t->cp_->reset();
  unsigned d = t->mp2_->kBits_;
  t->cp_->cptFlag_ = 0;
  t->cp_->trn2dD_ = d;
  t->cpSeen_ = 0;
  return hd;
}

/**
 * @brief The CP short frames (SUV / SUVt) of state 4 once an RRN is pending: copy the CP's short-frame byte (wCa0_) to w4c_ and pick the event.
 * @param ev0  event when neither rrnSilence_ nor that byte is set
 * @param ev1  event when the answer was already sent (answerSent_)
 * @param ev2  event otherwise (answerDue_ is set to 1)
 */
static inline void cpShortFrame(D* t, unsigned ev0, unsigned ev1, unsigned ev2)
{
  unsigned c = t->cp_->wCa0_;
  t->w4c_ = c;
  if (t->rrnSilence_ == 0 && c == 0)
  {
    t->event_ = ev0;
    return;
  }
  if (t->answerSent_ != 0)
  {
    t->event_ = ev1;
    return;
  }
  t->answerDue_ = 1;
  t->event_ = ev2;
}

/**
 * @brief State 4 (V.92): wait for the CP; every descrambled byte goes through V90CP::bitsToInfo().
 *        CP / CPt / SUV / SUVt raise their events (SUV, SUVt with the RRN variants while an RRN is pending); an Ed after an SUV / SUVt ends in B1d, or, for
 *        the first Ed of an RRN with the answer due, starts the silence measurement (state 10, or state 14 when w4c_ is set).
 * @return the hard decision
 */
static inline short stV92WaitCP(D* t, short x)
{
  unsigned r;
  short hd = decodeStep(t, x, &r);
  if (r == 0)
    return hd;
  if (t->nBytes_ == 0)
    return hd;
  unsigned i = 0;
  do
  {
    int b = t->descr_->process(t->buf_[i]);
    unsigned k = (unsigned)t->cp_->bitsToInfo((unsigned char)b);

    /* the CP info kind found in the bits so far, if any */
    switch (k)
    {
    case CPINFO_CP:
      t->event_ = V90DEMOD_STATE_CP_DETECTED;
      edprintf(PFX "CP detected @ %d\r\n", t->cnt_);
      break;
    case CPINFO_CPT:
      t->event_ = V90DEMOD_STATE_CPNOT_DETECTED;
      edprintf(PFX "CPnot detected @ %d\r\n", t->cnt_);
      break;
    case CPINFO_SUV:
      t->cpSeen_ = 1;
      if (t->rrnPending_ == 0)
        t->event_ = V90DEMOD_STATE_SUV_DETECTED;
      else
        cpShortFrame(t, V90DEMOD_STATE_SUV_DETECTED, V90DEMOD_STATE_SILENCE_RRN_SUV_PART2, V90DEMOD_STATE_SILENCE_RRN_SUV_PART1);
      break;
    case CPINFO_SUVT:
      if (t->rrnPending_ == 0)
      {
        t->event_ = V90DEMOD_STATE_SUVT_DETECTED;
        break;
      }
      t->cpSeen_ = 1;
      cpShortFrame(t, V90DEMOD_STATE_SUVT_DETECTED, V90DEMOD_STATE_SILENCE_RRN_SUVT_PART2, V90DEMOD_STATE_SILENCE_RRN_SUVT_PART1);
      break;
    case CPINFO_ED:
      /* an Ed counts only after an SUV / SUVt */
      if (t->cpSeen_ != 0)
      {
        if (t->rrnPending_ == 0 || t->answerDue_ == 0 || t->answerSent_ != 0)
        {
          t->event_ = V90DEMOD_STATE_ED_DETECTED;
          edprintf(PFX "Ed detected @ %d, enter B1d state\r\n", t->cnt_);
          t->state_ = P4D_B1D;
          t->cnt_ = 0;
          t->dem_->resetNoSpectral(t->mp2_);
          t->b1dZeros_ = 0;
          t->b1dBits_ = 0;
        }
        else
        {
          /* the first Ed of an RRN with the answer due: measure the silence (or go to the Rt wait when the CP asked for it) */
          t->event_ = V90DEMOD_STATE_FIRST_RRN_ED_DETECTED;
          edprintf(PFX "First Ed at RRN detected @ %d, Silence state\r\n", t->cnt_);
          t->cnt_ = 0;
          t->echoBefore_ = 0.0f;
          t->r_.reset((unsigned)t->params_->rrnRDetectionLength, 0x18);
          t->rf_.reset(0xb4, 0xc);
          t->answerSent_ = 1;
          unsigned c = t->w4c_;
          t->cpSeen_ = 0;
          t->state_ = (c == 0) ? P4D_SILENCE_WAIT : P4D_WAIT_RT;
        }
      }
      break;
    default:
      break;
    }
    i++;
  } while (t->nBytes_ > i);
  return hd;
}

/** @brief See the declaration and the file header: the V.92 dispatcher on state_. */
short V90Phase4Demodulator::getV92Decision(short x)
{
  D* t = this;
  unsigned st = t->state_;

  /* count the symbol, clear the event, run the handler of the current state */
  t->cnt_++;
  t->event_ = 0;
  switch (st)
  {
  case P4D_WAIT_RI:
    return stRi(t, x);
  case P4D_WAIT_RI_NOT:
    return stRiNot(t, x);
  case P4D_TRN2D_ENTRY:
    t->state_ = P4D_TRN2D_KNOWN; /* fall through */
  case P4D_TRN2D_KNOWN:
    return stKnown(t, x);
  case P4D_WAIT_CP:
    return stV92WaitCP(t, x);
  case P4D_B1D:
    return stB1d(t, x);
  case P4D_TRN2D_DD:
    return stV92RdNot(t, x);
  case P4D_SILENCE_WAIT:
    return stSilenceWait(t, x);
  case P4D_ECHO_BEFORE:
    return stEchoBefore(t, x);
  case P4D_ECHO_WAIT:
    return stEchoWait(t, x);
  case P4D_ECHO_AFTER:
    return stEchoAfter(t, x);
  case P4D_WAIT_RT:
    return stV92Rt(t, x);
  case P4D_WAIT_RT_NOT:
    return stV92RtNot(t, x);
  case P4D_WAIT_RF_NOT:
    return stV92RfNot(t, x);
  case P4D_TERMINATED:
  case 17:
    return 0;
  default:
    return 0; /* states 5, 6 and > 17: no output */
  }
}
