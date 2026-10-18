/**
 * @file v90_phase4_modulator.cpp
 * @brief V90Phase4Modulator: construction, reset, the dispatcher, pattern setup, the owner's event handlers and the small per-state generators.
 *
 * Everything of the class except the two state-table functions generateV90Symbol() / generateV92Symbol() (v90_phase4_modulator_symbol.cpp).  Declarations are
 * in v90_phase4_modulator.h.  This file owns:
 * - construction and destruction: the constructor creates the Scrambler member and, when the caller passes no V90BitsToSymbol, allocates a private one
 *   (capacity 0x140 symbols); the destructor frees only a private one;
 * - reset(): sets the state, the Ri level from a G.711 code and the flag words, resets the scrambler and optionally runs the generator forward;
 * - generateSymbol(): the V.90 / V.92 dispatcher;
 * - setMappingParams() and the six / twelve-symbol Rd / Rt / Rf pattern setters (the G.711 levels of the first ucode of each list);
 * - the owner's event handlers (exitRi / exitMP / exitMPNot / exitSilence and the V.92 recived* family), which move the state machine on at the next frame or
 *   period boundary;
 * - the small per-state generators (Ri, Rd / Rt, Rf, B1d, TRN2d, Ed, MP, CPd, SUVd), built from the helpers of v90_phase4_patterns.h / v90_phase4_steps.h.
 *
 * Callees from other files: Scrambler (dsp/dsp_int_templates.h), V90BitsToSymbol, V90MP / V90CP getBitVector() and infoToBits(), alaw2linear() /
 * ulaw2linear(), edprintf(), dsplibs_debug_printf(), sysdep_malloc() / sysdep_free().
 *
 * Behaviour that a tidier version would change (kept as is):
 * - the constructor stores NOTHING of state_ / cnt_ / event_ / nextState_ / the flags / the counters / level_ / bits_ ...: only the Scrambler (first), params_,
 *   cp_, arg8_, mp_, dataMp_, flag_, w2f9c_, trn2dMp_, w2fa0_ and then b2s_ / notOwned_; every other byte is whatever the memory held until reset();
 * - the destructor frees the V90BitsToSymbol only when notOwned_ == 0 and b2s_ != NULL (destroy, then sysdep_free), then destroys the Scrambler;
 * - reset(): the stores before the scrambler reset (pcm_, limit_, cnt_, state_, event_, level_) are visible to it; nextState_ = flag_ == 0 ? 4 : 5; the symbol
 *   generators run n times (n == 0: none) with the session flag re-read before each call; level_ is the linear value of (code & 0x7f) ^ 0xd5 (A-law) / ^ 0xff
 *   (mu-law), chosen by the pcm ARGUMENT, stored as 16 bits;
 * - setRdRtSymbols() / setRfSymbols() convert one byte of the mapping parameters per symbol, in order, re-reading pcm_ before EACH conversion;
 *   A-law level = alaw2linear((b & 0x7f) ^ 0xd5), mu-law level = ulaw2linear(~(b & 0x7f) & 0xff); negated entries store (short)(-level);
 * - the event handlers compare the symbol counter with the (possibly stale) 32-bit unsigned frame lengths cpFrame_ / mpFrame_; a zero frame length divides by
 *   zero; "enter Ed" sets cnt_ = 0, state_ = 0x10, target_ = b2s_->extra_ + 12 after the print; recivedCPtag() fetches the CP bits WITHOUT infoToBits() and
 *   divides by the NEW trn2dD_ only after having tested the OLD cpFrame_;
 * - recivedSUVtag() / recivedCPtag() / recivedE2u() compare the state as a SIGNED value against 5 (a state >= 2^31 counts as "below 5") and then (state - 7) as
 *   an unsigned value against 1;
 * - generateDataSymbolBeforeFPE() / generateDataSymbolBeforeRRN() pass an uninitialised n and out to process(); the callee writes both.
 */

#include "v90_phase4_modulator.h"
#include "core/dsp_host.h"
#include "v34/v34_api.h"

extern "C"
{
}

/* ------------------------------------------------------------------------------------------------ helpers (static inline) */

#include "core/dsp_intmath.h"
#include "core/dsp_debug.h"

#include "v90_phase4_patterns.h"

/** @brief Linear G.711 level of a mapping-parameter byte `b` (low 7 bits); the codec comes from pcm_, which is read afresh by every conversion. */
static inline int levelOf(const V90Phase4Modulator* t, unsigned char b)
{
  unsigned x = (unsigned)(b & 0x7f);
  if (t->pcm_ != 0)
    return alaw2linear((int)((x ^ 0xd5u) & 0xffu));
  return ulaw2linear((int)(~x & 0xffu));
}

/** @brief Enter Ed: print `fmt` with the current count, restart the count and set the Ed length target (the queue's extra_ + 12). */
static inline void enterEd(V90Phase4Modulator* t, const char* fmt)
{
  edprintf(fmt, t->cnt_);
  t->cnt_ = 0;
  t->state_ = P4M_ED;
  t->target_ = t->b2s_->extra_ + 12;
}
#define ED_FMT "V90Phase4Modulator: enter Ed @ %d\r\n"

/** @brief Enter CPd (state 7) with a new long CP frame at count `c`; w2fa0_ records that a CPd frame has been started. */
static inline void enterCPd(V90Phase4Modulator* t, unsigned c)
{
  edprintf("V90Phase4Modulator: enter CPd @ %d\r\n", c);
  t->cnt_ = 0;
  t->state_ = P4M_CPD;
  t->cp_->type_ = 0;
  t->cp_->infoToBits();
  t->cpBits_ = t->cp_->getBitVector(t->cpLen_);
  unsigned f = (6 * t->cpLen_) / t->cp_->trn2dD_;
  t->w2fa0_ = 1;
  t->cpFrame_ = f;
}

/** @brief End test of state 5 (SUVd): off a CP frame boundary go to state 6, on a boundary enter CPd. */
static inline void suvEnd(V90Phase4Modulator* t)
{
  unsigned c = t->cnt_;
  if (c % t->cpFrame_ != 0)
  {
    t->state_ = P4M_CPD_FIRST;
    return;
  }
  enterCPd(t, c);
}

/** @brief End of the silence states: off a six-symbol boundary go to state 0x19, on a boundary enter Rt. */
static inline void silenceEnd(V90Phase4Modulator* t, unsigned c)
{
  if (c != (c / 6) * 6)
  {
    t->state_ = P4M_SILENCE_B;
    return;
  }
  edprintf("V90Phase4Modulator: enter Rt @ %d\r\n", c);
  t->state_ = P4M_RT;
  t->cnt_ = 0;
}

/**
 * @brief Tail shared by recivedSUVtag() / recivedCPtag() / recivedE2u(): on a CP frame boundary enter Ed (printing `fmt`), else go to state `st`.
 *        Either way the tag-handled word w20_ is set.
 */
static inline void tagEnd(V90Phase4Modulator* t, unsigned c, unsigned st, const char* fmt)
{
  if (c % t->cpFrame_ != 0)
  {
    t->state_ = st;
    t->w20_ = 1;
    return;
  }
  enterEd(t, fmt);
  t->w20_ = 1;
}

/**
 * @brief State to move to after a tag in the V.92 states 5 / 7 / 8: 5 -> 0xa, 7 and 8 -> 9.
 * @param st  current state, compared as a signed value against 5
 * @param to  receives the new state
 * @return true when `*to` was set, false for any other state
 */
static inline bool tagState(unsigned st, unsigned* to)
{
  int s = (int)st;
  if (s == P4M_SUVD)
  {
    *to = P4M_STATE10;
    return true;
  }
  if (s < 5)
    return false;
  if (st - 7 > 1u)
    return false;
  *to = P4M_STATE9;
  return true;
}

#include "v90_phase4_steps.h"

/**
 * @brief Body of the constructor: store the pointers and flags, and take the caller's V90BitsToSymbol or allocate a private one (capacity 0x140).
 * @warning The result of sysdep_malloc() is not checked.
 */
static inline void ctorBody(V90Phase4Modulator* t, V90Parameters* p, unsigned flag, V90BitsToSymbol* b2s, V90MP* mp, V90MappingParams* dataMp,
                            V90MappingParams* trn2dMp, V90CP* cp, unsigned arg8)
{
  t->params_ = p;
  t->cp_ = cp;
  t->arg8_ = arg8;
  t->mp_ = mp;
  t->dataMp_ = dataMp;
  t->flag_ = flag;
  t->w2f9c_ = 0;
  t->trn2dMp_ = trn2dMp;
  t->w2fa0_ = 0;

  if (b2s != 0)
  {
    t->b2s_ = b2s;
    t->notOwned_ = 1;
    return;
  }

  V90BitsToSymbol* b = (V90BitsToSymbol*)sysdep_malloc((unsigned)sizeof(V90BitsToSymbol));
  new (b) V90BitsToSymbol(0x140, p);
  t->b2s_ = b;
  t->notOwned_ = 0;
}

/** @brief Body of the destructor: destroy and free the V90BitsToSymbol when it is private (notOwned_ == 0) and present. */
static inline void dtorBody(V90Phase4Modulator* t)
{
  if (t->notOwned_ == 0)
  {
    V90BitsToSymbol* b = t->b2s_;
    if (b != 0)
    {
      b->~V90BitsToSymbol();
      sysdep_free(b);
    }
  }
}

V90Phase4Modulator::V90Phase4Modulator(V90Parameters* p, unsigned flag, V90BitsToSymbol* b2s, V90MP* mp, V90MappingParams* dataMp, V90MappingParams* trn2dMp,
                                       V90CP* cp, unsigned arg8)
    : scr_(0x12, 0x17, 0x63)
{
  ctorBody(this, p, flag, b2s, mp, dataMp, trn2dMp, cp, arg8);
}

V90Phase4Modulator::~V90Phase4Modulator() { dtorBody(this); }

/* ------------------------------------------------------------------------------------------------ the dispatcher and reset */

short V90Phase4Modulator::generateSymbol()
{
  if (flag_ != 0)
    return generateV92Symbol();
  return generateV90Symbol();
}

void V90Phase4Modulator::reset(PcmType pcm, unsigned char code, Phase4ModulatorState st, unsigned n, unsigned limit)
{
  /* state and counters */
  pcm_ = (unsigned)pcm;
  limit_ = limit;
  cnt_ = 0;
  state_ = (unsigned)st;
  event_ = 0;

  /* Ri level from the G.711 code, in the codec given by the argument */
  unsigned x = (unsigned)(code & 0x7f);
  int lv = (unsigned)pcm != 0 ? alaw2linear((int)(x ^ 0xd5u)) : ulaw2linear((int)(x ^ 0xffu));
  level_ = (unsigned short)lv;

  scr_.reset(0);

  /* flag words and the state entered after TRN2d */
  mpNotSeen_ = 0;
  w24_ = 0;
  w28_ = 0;
  w2c_ = 0;
  nextState_ = flag_ == 0 ? P4M_MP : P4M_SUVD;
  w2f9c_ = 0;
  w2fa0_ = 0;
  w30_ = 0;
  cpCnt_ = 0;
  cpFlag_ = 0;
  w20_ = 0;

  /* run the generator forward, discarding the symbols */
  for (unsigned i = 0; i < n; i++)
  {
    if (flag_ != 0)
      generateV92Symbol();
    else
      generateV90Symbol();
  }
}

/* ------------------------------------------------------------------------------------------------ small setters */

void V90Phase4Modulator::resetBeforRRN()
{
  w2f9c_ = 0;
  w2fa0_ = 0;
  w24_ = 1;
  w28_ = 0;
  w2c_ = 0;
  w30_ = 0;
  w34_ = 0;
  w20_ = 0;
}

void V90Phase4Modulator::setSessionFlag(unsigned f) { flag_ = f; }

void V90Phase4Modulator::setNextStateAfterTRN2d(Phase4ModulatorState st) { nextState_ = (unsigned)st; }

void V90Phase4Modulator::resetRRNSecondSection()
{
  V90CP* cp = cp_;
  w30_ = 1;
  w20_ = 0;
  cpFlag_ = 0;
  cpCnt_ = 0;
  cp->cptFlag_ = 0;
  w2f9c_ = 0;
  w2fa0_ = 0;
}

void V90Phase4Modulator::setMappingParams(V90MappingParams* mp)
{
  if (mp == 0)
  {
    if (dbgOn())
      dsplibs_debug_printf("V90Phase4Modulator: ERROR: Null mappingParams @ setMappingParams\r\n");
    return;
  }

  b2s_->reset(mp, (PcmType)pcm_);
  b2s_->setSymbolsBlockSize(1);
}

/* Rd / Rt pattern: the first ucode of each of the six lists, the last three negated */
void V90Phase4Modulator::setRdRtSymbols(V90MappingParams* mp)
{
  rdrt_[0] = (unsigned short)levelOf(this, mp->listA_[0][0]);
  rdrt_[1] = (unsigned short)levelOf(this, mp->listA_[1][0]);
  rdrt_[2] = (unsigned short)levelOf(this, mp->listA_[2][0]);
  rdrt_[3] = (unsigned short)(-levelOf(this, mp->listA_[3][0]));
  rdrt_[4] = (unsigned short)(-levelOf(this, mp->listA_[4][0]));
  rdrt_[5] = (unsigned short)(-levelOf(this, mp->listA_[5][0]));
}

/* Rf pattern: the six levels twice, with the signs + + - - + + / - - + + - - */
void V90Phase4Modulator::setRfSymbols(V90MappingParams* mp)
{
  rf_[0] = (unsigned short)levelOf(this, mp->listA_[0][0]);
  rf_[1] = (unsigned short)levelOf(this, mp->listA_[1][0]);
  rf_[2] = (unsigned short)(-levelOf(this, mp->listA_[2][0]));
  rf_[3] = (unsigned short)(-levelOf(this, mp->listA_[3][0]));
  rf_[4] = (unsigned short)levelOf(this, mp->listA_[4][0]);
  rf_[5] = (unsigned short)levelOf(this, mp->listA_[5][0]);
  rf_[6] = (unsigned short)(-levelOf(this, mp->listA_[0][0]));
  rf_[7] = (unsigned short)(-levelOf(this, mp->listA_[1][0]));
  rf_[8] = (unsigned short)levelOf(this, mp->listA_[2][0]);
  rf_[9] = (unsigned short)levelOf(this, mp->listA_[3][0]);
  rf_[10] = (unsigned short)(-levelOf(this, mp->listA_[4][0]));
  rf_[11] = (unsigned short)(-levelOf(this, mp->listA_[5][0]));
}

/* ------------------------------------------------------------------------------------------------ the owner's events */

void V90Phase4Modulator::exitRi()
{
  if (state_ != P4M_RI)
    return;
  unsigned c = cnt_;
  if (c == 0)
    return;

  /* off a six-symbol boundary: run Ri on to the boundary; on it: RiNot */
  if (c != (c / 6) * 6)
  {
    state_ = P4M_RI_END;
    return;
  }
  edprintf("V90Phase4Modulator: enter RiNot @ %d\r\n", c);
  state_ = P4M_RI_NOT;
  cnt_ = 0;
}

/**
 * @brief Owner event: leave MP at the next MP frame boundary.
 * @warning Divides by mpFrame_, which is set when TRN2d ends; calling this in state MP with mpFrame_ == 0 would divide by zero.
 */
void V90Phase4Modulator::exitMP()
{
  if (state_ != P4M_MP)
    return;
  unsigned c = cnt_;
  if (c == 0)
    return;

  /* off an MP frame boundary: finish the frame (state 0xd); on it: MPNot with a fresh MP frame */
  if (c % mpFrame_ != 0)
  {
    state_ = P4M_MP_LAST;
    return;
  }
  edprintf("V90Phase4Modulator: enter MPNot @ %d\r\n", c);
  state_ = P4M_MP_NOT;
  cnt_ = 0;
  mpBits_ = mp_->getBitVector(mpLen_);
  mpFrame_ = (6 * mpLen_) / mp_->trn2dD_;
}

/**
 * @brief Owner event: leave MPNot at the next MP frame boundary.
 * @warning Divides by mpFrame_ (see exitMP()).
 */
void V90Phase4Modulator::exitMPNot()
{
  if (state_ != P4M_MP_NOT)
    return;
  unsigned c = cnt_;
  if (c == 0)
    return;

  /* off an MP frame boundary: finish the frame (state 0xf); on it: Ed */
  if (c % mpFrame_ != 0)
  {
    state_ = P4M_WAIT_ED;
    return;
  }
  enterEd(this, ED_FMT);
}

void V90Phase4Modulator::exitSilence()
{
  if (state_ - 0x17 > 1u)
    return;
  unsigned c = cnt_;
  if (c == 0)
    return;
  silenceEnd(this, c);
}

void V90Phase4Modulator::enterRepeatedCPd()
{
  cpFlag_ = 0;
  cpCnt_ = 0;
  if (dbgOn())
    dsplibs_debug_printf("V90Phase4Modulator: enter repeatedCPd @ %d\r\n", cnt_);

  state_ = P4M_REPEATED_CPD;
  cp_->type_ = 0;
  cp_->infoToBits();
  cpBits_ = cp_->getBitVector(cpLen_);
  unsigned f = (6 * cpLen_) / cp_->trn2dD_;
  cnt_ = 0;
  cpFrame_ = f;
}

void V90Phase4Modulator::recivedSUV()
{
  if (w2fa0_ != 0)
    return;
  if (state_ != P4M_SUVD)
    return;
  suvEnd(this);
}

void V90Phase4Modulator::recivedPartOneSilenceRrnSUV() { cp_->cptFlag_ = 1; }

void V90Phase4Modulator::recivedPartTwoSilenceRrnSUV()
{
  unsigned s = state_;
  if (s - 0x17 <= 1u)
    return;
  if (s - 0x19 <= 2u)
    return;
  if (w2fa0_ != 0)
    return;
  if (s != 5)
    return;
  suvEnd(this);
}

void V90Phase4Modulator::recivedCP()
{
  V90CP* cp = cp_;
  w2f9c_ = 1;
  cp->cptFlag_ = 1;
}

void V90Phase4Modulator::recivedSUVtag()
{
  cpFlag_ = 0;
  cpCnt_ = 0;
  if (w2f9c_ == 0)
    return;
  if (w20_ != 0)
    return;

  unsigned to;
  if (!tagState(state_, &to))
    return;
  tagEnd(this, cnt_, to, ED_FMT);
}

void V90Phase4Modulator::recivedPartOneSilenceRrnSUVtag()
{
  cp_->cptFlag_ = 1;
  if (w20_ != 0)
    return;
  tagEnd(this, cnt_, 0xa, ED_FMT);
}

void V90Phase4Modulator::recivedPartTwoSilenceRrnSUVtag() { recivedSUVtag(); }

/**
 * @brief The remote CP tag was received (V.92 event): on the first CP send a short frame and finish the frame; later go to Ed at a frame boundary.
 * @note The CP bits of the new frame are fetched without a preceding infoToBits(): the vector is the one built by the last infoToBits() call.
 * @warning Divides by cpFrame_ (and by the new trn2dD_ when the CP is first received); a zero value divides by zero.
 */
void V90Phase4Modulator::recivedCPtag()
{
  cpFlag_ = 0;
  cpCnt_ = 0;
  if (w20_ == 0)
    return;

  /* first CP received: mark it, send a short frame (type 1) and finish the current frame or start the next one */
  if (w2f9c_ == 0)
  {
    V90CP* cp = cp_;
    w2f9c_ = 1;
    cp->cptFlag_ = 1;
    cp_->type_ = 1;
    if (cnt_ % cpFrame_ != 0)
    {
      state_ = P4M_FINAL_SUVD;
      w20_ = 1;
      return;
    }
    state_ = P4M_STATE11;
    cnt_ = 0;
    w20_ = 1;
    cpBits_ = cp->getBitVector(cpLen_);
    cpFrame_ = (6 * cpLen_) / cp_->trn2dD_;
    return;
  }

  /* CP already received: Ed on a frame boundary, else run the frame out */
  unsigned c = cnt_;
  if (c % cpFrame_ == 0)
  {
    enterEd(this, ED_FMT);
    w20_ = 1;
    return;
  }
  unsigned to;
  if (!tagState(state_, &to))
    return;
  state_ = to;
  w20_ = 1;
}

void V90Phase4Modulator::recivedE2u()
{
  if (w20_ != 0)
    return;
  unsigned c = cnt_;
  if (c % cpFrame_ == 0)
  {
    enterEd(this, ED_FMT);
    w20_ = 1;
    return;
  }
  unsigned to;
  if (!tagState(state_, &to))
    return;
  state_ = to;
  w20_ = 1;
}

void V90Phase4Modulator::recivedFirstRrnE2u()
{
  if (w20_ != 0)
    return;
  tagEnd(this, cnt_, 0xa, "V90Phase4Modulator: enter Ed first at RRN @ %d\r\n");
}

void V90Phase4Modulator::recivedFirstSUVuPartTwoRrn()
{
  unsigned s = state_;
  if (s - 0x17 <= 1u)
  {
    unsigned c = cnt_;
    if (c == 0)
      return;
    silenceEnd(this, c);
    return;
  }
  if (s != 5)
    return;
  if (w2fa0_ != 0)
    return;
  suvEnd(this);
}

/* ------------------------------------------------------------------------------------------------ the per-state generators */

short V90Phase4Modulator::generateRi() { return riPattern(level_, cnt_); }
short V90Phase4Modulator::generateRiNot() { return neg16(riPattern(level_, cnt_)); }
short V90Phase4Modulator::generateRdRt() { return rdrtAt(this); }
short V90Phase4Modulator::generateRdRtNot() { return neg16(rdrtAt(this)); }
short V90Phase4Modulator::generateRf() { return rfAt(this); }
short V90Phase4Modulator::generateRfNot() { return neg16(rfAt(this)); }

short V90Phase4Modulator::generateDataSymbolBeforeFPE()
{
  unsigned n;
  short out;
  b2s_->process(n, &out);
  if (n != 0)
  {
    edprintf("V90Phase4Modulator: enter Rf @ %d\r\n", cnt_);
    state_ = P4M_RF;
    cnt_ = 0;
  }
  return out;
}

short V90Phase4Modulator::generateDataSymbolBeforeRRN()
{
  unsigned n;
  short out;
  b2s_->process(n, &out);
  if (n != 0)
  {
    edprintf("V90Phase4Modulator: enter Rd @ %d\r\n", cnt_);
    state_ = P4M_RD;
    cnt_ = 0;
  }
  return out;
}

short V90Phase4Modulator::generateB1d() { return stepOnes(this); }
short V90Phase4Modulator::generateTRN2d() { return stepOnes(this); }
short V90Phase4Modulator::generateEd() { return stepZeros(this); }
short V90Phase4Modulator::generateMP() { return stepMp(this); }
short V90Phase4Modulator::generateCPd() { return stepCp(this); }
short V90Phase4Modulator::generateSUVd() { return stepCp(this); }
