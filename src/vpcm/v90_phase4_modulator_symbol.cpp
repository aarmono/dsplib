/**
 * @file v90_phase4_modulator_symbol.cpp
 * @brief V90Phase4Modulator symbol generators: generateV90Symbol() and generateV92Symbol(), the phase-4 state tables.
 *
 * Each call produces one 16-bit PCM-level symbol and advances the state machine (the states and their order are described in v90_phase4_modulator.h).
 * Both functions are a `switch` over `state_`: a state either returns a pattern symbol (Ri, Rd, Rt, Rf and their inverses), a scrambled data symbol taken from
 * the bit-to-symbol queue (TRN2d, MP, CPd, SUVd, Ed, B1d: via the step helpers of v90_phase4_steps.h), or silence (0), and tests whether its length has
 * elapsed to switch to the next state.  The class and the other methods are in v90_phase4_modulator.h / v90_phase4_modulator.cpp.
 *
 * Callees from other files: Scrambler::reset(), V90BitsToSymbol (nofBitsForNextTime(), process(), resetNoSpectral()), V90MP::getBitVector(),
 * V90CP::getBitVector() / infoToBits(), displaySpectralParams(), edprintf(), dsplibs_debug_printf().
 *
 * Behaviour that a tidier version would change (kept as is):
 * - cnt_ is incremented and event_ cleared BEFORE the state dispatch (also for an illegal state), and the stores to state_ / cnt_ / event_ / the CP words are
 *   ordered relative to each print and each callee exactly as written here;
 * - the symbols are 16-bit values; a negated level is (int16)(-level); the six-symbol pattern index is (cnt_ - 1) % 6 (unsigned), the twelve-symbol one
 *   (cnt_ - 1) % 12;
 * - every data step first asks b2s_->nofBitsForNextTime() and, when it is non-zero, scrambles that many bits (all ones / all zeros) or the whole MP / CP frame
 *   (mpLen_ / cpLen_ bits, re-read after the scrambler call) into bits_ and queues them; the output symbol is what b2s_->process(n, &out) delivers (n becomes
 *   the bit count the next call needs);
 * - the frame lengths mpFrame_ / cpFrame_ are 6 * len / D with a 32-bit product (it wraps) and an unsigned divide;
 * - V.90 state 0xe clears mpNotSeen_ whenever it was set; V.92 state 5 compares cpCnt_ with limit_ + 0x320 as 32-bit unsigned values.
 *
 * @warning The frame lengths and trn2dD_ are used as divisors (`cnt_ % cpFrame_`, `6 * len / D`) without a zero test.  Several states dereference their
 *          pointers without a NULL test: V.92 RtNot (0x1b) cp_, V.92 RfNot (0x1e) cp_ and dataMp_, RdNot (0x16) trn2dMp_ (V.92 also cp_); a NULL there crashes.
 */

#include "v90_phase4_modulator.h"
#include "core/dsp_host.h"
#include "v34/v34_api.h"

extern "C"
{
}

/* ------------------------------------------------------------------------------------------------ helpers (static inline) */

#include "core/dsp_intmath.h"

#include "v90_phase4_patterns.h"

#include "core/dsp_debug.h"

/** @brief Fetch the MP bit vector and set the symbols per MP frame: 6 * len / D (32-bit product, unsigned divide). */
static inline void fetchMp(V90Phase4Modulator* t)
{
  t->mpBits_ = t->mp_->getBitVector(t->mpLen_);
  t->mpFrame_ = (6 * t->mpLen_) / t->mp_->trn2dD_;
}

/** @brief Fetch the CP bit vector and set the symbols per CP frame: 6 * len / D (32-bit product, unsigned divide). */
static inline void fetchCp(V90Phase4Modulator* t)
{
  t->cpBits_ = t->cp_->getBitVector(t->cpLen_);
  t->cpFrame_ = (6 * t->cpLen_) / t->cp_->trn2dD_;
}

/** @brief Build the next CP frame from the CP object's current information and fetch it. */
static inline void startCp(V90Phase4Modulator* t)
{
  t->cp_->infoToBits();
  fetchCp(t);
}

#include "v90_phase4_steps.h"

/** @brief Draining step of states 0x14 / 0x1c: take a symbol out of the queue without asking for bits first; `*n` receives the next bit count. */
static inline short stepDrain(V90Phase4Modulator* t, unsigned* n)
{
  short out = 0;
  t->b2s_->process(*n, &out);
  return out;
}

/** @brief Enter Ed at the end of the MP / CP phase; the Ed length target is the queue's extra_ + 12. */
static inline void enterEd(V90Phase4Modulator* t)
{
  edprintf("V90Phase4Modulator: enter Ed @ %d\r\n", t->cnt_);
  t->cnt_ = 0;
  t->state_ = P4M_ED;
  t->target_ = t->b2s_->extra_ + 12;
}

/** @brief End of Ed with data-phase mapping parameters present: enter B1d, reset the queue to the data mapping and restart the scrambler. */
static inline void enterB1d(V90Phase4Modulator* t, unsigned cnt)
{
  edprintf("V90Phase4Modulator: enter B1d @ %d\r\n", cnt);
  t->state_ = P4M_B1D;
  t->cnt_ = 0;
  t->b2s_->resetNoSpectral(t->dataMp_, (PcmType)t->pcm_);
  t->scr_.reset(0);
}

/** @brief End of Ed without data-phase mapping parameters: go to the error state (no output) and print an error at debug level. */
static inline void nullDataMapping(V90Phase4Modulator* t)
{
  bool d = dbgOn();
  t->state_ = P4M_ERROR;
  t->cnt_ = 0;
  if (d)
    dsplibs_debug_printf("V90Phase4Modulator: ERROR: Null dataPhaseMappingParams @ end of Ed\r\n");
}

/* ------------------------------------------------------------------------------------------------ generateV90Symbol */

short V90Phase4Modulator::generateV90Symbol()
{
  unsigned st = state_;
  short out = 0;
  cnt_++;
  event_ = 0;
  switch (st)
  {
  case P4M_RI: /* Ri: runs until the owner calls exitRi() */
    return riPattern(level_, cnt_);
  case P4M_RI_END:
  { /* Ri, up to the six-symbol period boundary, then RiNot */
    unsigned c = cnt_;
    out = riPattern(level_, c);
    if (c % 6 == 0)
    {
      edprintf("V90Phase4Modulator: enter RiNot @ %d\r\n", c);
      state_ = P4M_RI_NOT;
      cnt_ = 0;
    }
    return out;
  }
  case P4M_RI_NOT:
  { /* RiNot: 0x18 symbols, then TRN2d with a fresh scrambler */
    unsigned c = cnt_;
    out = neg16(riPattern(level_, c));
    if (c == 0x18)
    {
      state_ = P4M_TRN2D;
      cnt_ = 0;
      edprintf("V90Phase4Modulator: enter TRN2d\r\n");
      scr_.reset(0);
    }
    return out;
  }
  case P4M_TRN2D: /* TRN2d: 0x3e7c scrambled ones, then the state chosen by nextState_ (MP or MPNot) with the first MP frame */
    out = stepOnes(this);
    if (cnt_ != 0x3e7c)
      return out;
    if (mp_ == 0)
    {
      bool d = dbgOn();
      state_ = P4M_ERROR;
      cnt_ = 0;
      if (d)
        dsplibs_debug_printf("V90Phase4Modulator: ERROR: Null MP @ end of TRN2d\r\n");
      return out;
    }
    state_ = nextState_;
    edprintf("V90Phase4Modulator: enter %s @ %d\n", state_ == P4M_MP ? "MP" : "DIRECTLY to MPNot", 0x3e7c);
    cnt_ = 0;
    fetchMp(this);
    return out;
  case P4M_MP: /* MP: scrambled MP frames, repeated until exitMP() */
    return stepMp(this);
  case P4M_MP_LAST: /* MP, finishing the current frame; at the frame boundary go to MPNot with the next frame */
    out = stepMp(this);
    {
      unsigned c = cnt_;
      if (c % mpFrame_ == 0)
      {
        edprintf("V90Phase4Modulator: enter MPNot @ %d\r\n", c);
        state_ = P4M_MP_NOT;
        cnt_ = 0;
        fetchMp(this);
      }
    }
    return out;
  case P4M_MP_NOT: /* MPNot: MP frames until the owner sets mpNotSeen_, then Ed at the frame boundary (or state 0xf until it) */
    out = stepMp(this);
    if (mpNotSeen_ == 0)
      return out;
    {
      /* the state cannot change inside this case, so in14 is always true */
      bool in14 = state_ == P4M_MP_NOT;
      mpNotSeen_ = 0;
      if (!in14)
        return out;
    }
    if (cnt_ == 0)
      return out;
    if (cnt_ % mpFrame_ == 0)
      enterEd(this);
    else
      state_ = P4M_WAIT_ED;
    return out;
  case P4M_WAIT_ED: /* MPNot continued to the frame boundary, then Ed */
    out = stepMp(this);
    if (cnt_ % mpFrame_ == 0)
      enterEd(this);
    return out;
  case P4M_ED: /* Ed: scrambled zeros until cnt_ reaches target_, then B1d (or the error state without data mapping parameters) */
    out = stepZeros(this);
    if (cnt_ != target_)
      return out;
    if (dataMp_ != 0)
      enterB1d(this, cnt_);
    else
      nullDataMapping(this);
    return out;
  case P4M_B1D: /* B1d: 0x120 scrambled ones, then phase 4 is terminated (event_ = 7) */
    out = stepOnes(this);
    if (cnt_ != 0x120)
      return out;
    edprintf("V90Phase4Modulator: Phase4 Terminated @ %d\r\n", 0x120);
    state_ = P4M_TERMINATED;
    cnt_ = 0;
    event_ = 7;
    return out;
  case P4M_TERMINATED: /* scrambled ones forever */
    return stepOnes(this);
  case P4M_RD_WAIT:
  { /* wait until the queue wants bits, then start Rd */
    unsigned n;
    out = stepDrain(this, &n);
    if (n != 0)
    {
      edprintf("V90Phase4Modulator: enter Rd @ %d\r\n", cnt_);
      state_ = P4M_RD;
      cnt_ = 0;
    }
    return out;
  }
  case P4M_RD: /* Rd: 0x180 symbols of the rdrt_ pattern, then RdNot */
    out = rdrtAt(this);
    if (cnt_ == 0x180)
    {
      edprintf("V90Phase4Modulator: enter RdNot @ %d\r\n", 0x180);
      state_ = P4M_RD_NOT;
      cnt_ = 0;
    }
    return out;
  case P4M_RD_NOT: /* RdNot: 0x18 inverted symbols, then back to TRN2d with the TRN2d mapping (and a dump of its spectral parameters) */
    out = neg16(rdrtAt(this));
    if (cnt_ == 0x18)
    {
      edprintf("V90Phase4Modulator: enter TRN2d @ %d\r\n", 0x18);
      state_ = P4M_TRN2D;
      cnt_ = 0;
      b2s_->resetNoSpectral(trn2dMp_, (PcmType)pcm_);
      edprintf("V90Phase4Modulator: TRN2d spectral parameters:\r\n");
      displaySpectralParams(trn2dMp_);
      edprintf("V90Phase4Modulator: TRN2d D = %d\r\n", trn2dMp_->kBits_);
    }
    return out;
  case P4M_ERROR:
  case P4M_RT:
  case P4M_RT_NOT:
    return 0;
  default:
    if (dbgOn())
      dsplibs_debug_printf("V90Phase4Modulator: Illegal state\r\n");
    return 0;
  }
}

/* ------------------------------------------------------------------------------------------------ generateV92Symbol */

short V90Phase4Modulator::generateV92Symbol()
{
  unsigned st = state_;
  short out = 0;
  cnt_++;
  event_ = 0;
  switch (st)
  {
  case P4M_RI: /* Ri: runs until the owner moves the state */
    return riPattern(level_, cnt_);
  case P4M_RI_END:
  { /* Ri, up to the six-symbol period boundary, then RiNot */
    unsigned c = cnt_;
    out = riPattern(level_, c);
    if (c % 6 == 0)
    {
      edprintf("V90Phase4Modulator: enter RiNot @ %d\r\n", c);
      state_ = P4M_RI_NOT;
      cnt_ = 0;
    }
    return out;
  }
  case P4M_RI_NOT:
  { /* RiNot: 0x18 symbols, then TRN2d */
    unsigned c = cnt_;
    out = neg16(riPattern(level_, c));
    if (c == 0x18)
    {
      state_ = P4M_TRN2D;
      cnt_ = 0;
      edprintf("V90Phase4Modulator: RiNot Terminated\r\n");
    }
    return out;
  }
  case P4M_TRN2D: /* TRN2d: 0x3e7c scrambled ones, then SUVd with the first CP frame (error state without a CP object); event_ = 4 in both cases */
    out = stepOnes(this);
    if (cnt_ != 0x3e7c)
      return out;
    if (cp_ != 0)
    {
      state_ = P4M_SUVD;
      edprintf("V90Phase4Modulator: enter SUVd @ %d\r\n", 0x3e7c);
      cnt_ = 0;
      cp_->type_ = 1;
      cp_->wCa0_ = w28_;
      startCp(this);
    }
    else
    {
      bool d = dbgOn();
      state_ = P4M_ERROR;
      cnt_ = 0;
      if (d)
        dsplibs_debug_printf("V90Phase4Modulator: ERROR: Null CP @ end of TRN2d\r\n");
    }
    event_ = 4;
    return out;
  case P4M_SUVD: /* SUVd: short CP frames; after limit_ + 0x320 symbols counted since the CP was sent, switch to repeated CPd */
    out = stepCp(this);
    if (cpFlag_ != 0)
      cpCnt_++;
    {
      unsigned c = cnt_;
      if (c % cpFrame_ != 0)
        return out;
      if (c == 0)
        return out;
    }

    /* at a frame boundary: queue the next short frame, and test the repeat limit */
    startCp(this);
    if (cpCnt_ <= limit_ + 0x320)
      return out;
    {
      bool d;
      cpFlag_ = 0;
      d = dbgOn();
      cpCnt_ = 0;
      if (d)
        dsplibs_debug_printf("V90Phase4Modulator: enter repeatedCPd @ %d\r\n", cnt_);
    }
    cp_->type_ = 0;
    state_ = P4M_REPEATED_CPD;
    startCp(this);
    cnt_ = 0;
    return out;
  case P4M_CPD_FIRST: /* SUVd continued to a frame boundary, then CPd with a long frame */
    out = stepCp(this);
    {
      unsigned c = cnt_;
      if (c % cpFrame_ != 0)
        return out;
      edprintf("V90Phase4Modulator: enter CPd @ %d\r\n", c);
    }
    cnt_ = 0;
    state_ = P4M_CPD;
    cp_->type_ = 0;
    startCp(this);
    w2fa0_ = 1;
    return out;
  case P4M_CPD: /* CPd: one long frame, then back to SUVd */
    out = stepCp(this);
    if (cnt_ != cpFrame_)
      return out;
    edprintf("V90Phase4Modulator: CPd Terminated @ %d\r\n", cnt_);
    cpCnt_ = 0;
    cpFlag_ = 1;
    state_ = P4M_SUVD;
    cnt_ = 0;
    cp_->type_ = 1;
    startCp(this);
    return out;
  case P4M_REPEATED_CPD: /* repeated CPd: a new long frame at every frame boundary */
    out = stepCp(this);
    {
      unsigned c = cnt_;
      if (c % cpFrame_ != 0)
        return out;
      if (c == 0)
        return out;
    }
    startCp(this);
    return out;
  case P4M_STATE9:
  case P4M_STATE10: /* CP frame run out after a tag, then Ed */
    out = stepCp(this);
    if (cnt_ % cpFrame_ == 0)
      enterEd(this);
    return out;
  case P4M_STATE11: /* final short frame, then Ed */
    out = stepCp(this);
    if (cnt_ == cpFrame_)
      enterEd(this);
    return out;
  case P4M_FINAL_SUVD: /* SUVd continued to a frame boundary, then the final short frame (state 11) */
    out = stepCp(this);
    {
      unsigned c = cnt_;
      if (c % cpFrame_ != 0)
        return out;
      edprintf("V90Phase4Modulator: enter FinalSUVd @ %d\r\n", c);
    }
    state_ = P4M_STATE11;
    cnt_ = 0;
    cp_->type_ = 1;
    startCp(this);
    return out;
  case P4M_ED: /* Ed: scrambled zeros until cnt_ reaches target_, then silence (RRN section), B1d, or the error state without data mapping parameters */
    out = stepZeros(this);
    if (cnt_ != target_)
      return out;
    if (w24_ != 0 && w2c_ != 0 && w30_ == 0)
    {
      edprintf("V90Phase4Modulator: enter Silence @ %d\r\n", cnt_);
      cnt_ = 0;
      state_ = w34_ == 0 ? P4M_SILENCE_A : P4M_STATE23;
      return out;
    }
    if (dataMp_ != 0)
      enterB1d(this, cnt_);
    else
      nullDataMapping(this);
    return out;
  case P4M_B1D: /* B1d: 0x120 scrambled ones, then phase 4 is terminated (event_ = 7) */
    out = stepOnes(this);
    if (cnt_ != 0x120)
      return out;
    edprintf("V90Phase4Modulator: Phase4 Terminated @ %d\r\n", 0x120);
    state_ = P4M_TERMINATED;
    cnt_ = 0;
    event_ = 7;
    return out;
  case P4M_TERMINATED: /* scrambled ones forever */
    return stepOnes(this);
  case P4M_RD_WAIT:
  { /* wait until the queue wants bits, then start Rd */
    unsigned n;
    out = stepDrain(this, &n);
    if (n != 0)
    {
      edprintf("V90Phase4Modulator: enter Rd @ %d\r\n", cnt_);
      state_ = P4M_RD;
      cnt_ = 0;
    }
    return out;
  }
  case P4M_RD: /* Rd: 0x180 symbols of the rdrt_ pattern, then RdNot */
    out = rdrtAt(this);
    if (cnt_ == 0x180)
    {
      edprintf("V90Phase4Modulator: enter RdNot @ %d\r\n", 0x180);
      state_ = P4M_RD_NOT;
      cnt_ = 0;
    }
    return out;
  case P4M_RD_NOT: /* RdNot: 0x18 inverted symbols, then back to TRN2d with the TRN2d mapping and the CP's frame-padding unit set to its D */
    out = neg16(rdrtAt(this));
    if (cnt_ == 0x18)
    {
      edprintf("V90Phase4Modulator: enter TRN2d @ %d\r\n", 0x18);
      cnt_ = 0;
      state_ = P4M_TRN2D;
      cp_->trn2dD_ = trn2dMp_->kBits_;
      b2s_->resetNoSpectral(trn2dMp_, (PcmType)pcm_);
      edprintf("V90Phase4Modulator: TRN2d spectral parameters:\r\n");
      displaySpectralParams(trn2dMp_);
      edprintf("V90Phase4Modulator: TRN2d D = %d\r\n", trn2dMp_->kBits_);
    }
    return out;
  case P4M_SILENCE_A: /* silence until symbol 0x960, then as state 0x19 */
    if (cnt_ <= 0x95f)
      return 0;
    /* fall through */
  case P4M_SILENCE_B: /* silence up to a six-symbol boundary, then Rt */
    if (cnt_ % 6 != 0)
      return 0;
    edprintf("V90Phase4Modulator: enter Rt @ %d\r\n", cnt_);
    state_ = P4M_RT;
    cnt_ = 0;
    return 0;
  case P4M_RT: /* Rt: 0x180 symbols of the rdrt_ pattern, then RtNot */
    out = rdrtAt(this);
    if (cnt_ == 0x180)
    {
      edprintf("V90Phase4Modulator: enter RtNot @ %d\r\n", 0x180);
      state_ = P4M_RT_NOT;
      cnt_ = 0;
    }
    return out;
  case P4M_RT_NOT: /* RtNot: 0x18 inverted symbols, then SUVd of the second RRN section with a fresh CP frame */
    out = neg16(rdrtAt(this));
    if (cnt_ == 0x18)
    {
      edprintf("V90Phase4Modulator: enter SUVd at RRN @ %d\r\n", 0x18);
      state_ = P4M_SUVD;
      cnt_ = 0;
      w30_ = 1;
      w20_ = 0;
      cpFlag_ = 0;
      cpCnt_ = 0;
      cp_->cptFlag_ = 0;
      w2f9c_ = 0;
      w2fa0_ = 0;
      cp_->type_ = 1;
      startCp(this);
    }
    return out;
  case P4M_RF_WAIT:
  { /* wait until the queue wants bits, then start Rf */
    unsigned n;
    out = stepDrain(this, &n);
    if (n != 0)
    {
      edprintf("V90Phase4Modulator: enter Rf @ %d\r\n", cnt_);
      state_ = P4M_RF;
      cnt_ = 0;
    }
    return out;
  }
  case P4M_RF: /* Rf: 0x180 symbols of the rf_ pattern, then RfNot */
    out = rfAt(this);
    if (cnt_ == 0x180)
    {
      edprintf("V90Phase4Modulator: enter RfNot @ %d\r\n", 0x180);
      state_ = P4M_RF_NOT;
      cnt_ = 0;
    }
    return out;
  case P4M_RF_NOT: /* RfNot: 0x18 inverted symbols, then SUVd with the data mapping's D as the CP frame-padding unit */
    out = neg16(rfAt(this));
    if (cnt_ == 0x18)
    {
      edprintf("V90Phase4Modulator: enter SUVd @ %d\r\n", 0x18);
      cnt_ = 0;
      state_ = P4M_SUVD;
      cp_->type_ = 1;
      cp_->trn2dD_ = dataMp_->kBits_;
      cp_->wCa0_ = w28_;
      startCp(this);
    }
    return out;
  case P4M_ERROR:
  case P4M_STATE23:
    return 0;
  default:
    if (dbgOn())
      dsplibs_debug_printf("V90Phase4Modulator: Illegal state\r\n");
    return 0;
  }
}
