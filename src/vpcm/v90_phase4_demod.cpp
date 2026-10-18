/**
 * @file v90_phase4_demod.cpp
 * @brief V90Phase4Demodulator: construction, reset(), the state entry points, the Ri / Rd / Rf detector wrappers and the getDecision() dispatcher.
 *
 * The class is declared in v90_phase4_demod.h; the two state machines it dispatches to, getV90Decision() and getV92Decision(), are in
 * v90_phase4_demod_decisions.cpp.  This file owns:
 * - construction and destruction: the constructor builds the embedded V90Phase4Modulator and the two V90RDetector objects, then stores the borrowed
 *   object pointers; the destructor clears nothing;
 * - reset(): sets the starting state, resets the detectors, the CP (V.92) or MP (V.90), the V90Demapper and the modulator, picks the TRN2d data length
 *   (quick connect or normal) and finally runs `settleCount` decisions on a zero symbol;
 * - the state entries enterWaitForCP() / enterWaitForMP() / enterWaitForEd(), the detector wrappers resetRRNDetector(), detectRRN(), detectFPE() and
 *   resetBeforRRN(), and setSessionFlag();
 * - getDecision(): the dispatcher on `session_`;
 * - trn2dKnownDemod(), which has no caller in the library.
 *
 * Callees from other files: V90RDetector (detectR, detectRf, reset), V90MP / V90CP reset, V90Demapper::reset, V90Phase4Modulator (constructor, reset,
 * setMappingParams, setSessionFlag, generateSymbol), linear2alaw() / linear2ulaw(), edprintf() and dsplibs_debug_printf().
 *
 * Details that matter to callers:
 * - the embedded modulator is built with the DATA mapping parameters (mp2) and the TRN2d parameters (mp1) in the opposite order to the constructor's own
 *   arguments;
 * - reset() stores the plain members first, then resets the detectors with the parameter lengths (phase4RDetectionLength, 0x18), and finally runs the
 *   settle decisions, re-reading `session_` before each call;
 * - detectRRN() and detectFPE() print the polarity of `r_` (detectFPE() also for the Rf detector `rf_`, see the @bug there).
 */

#include "v90_phase4_demod.h"
#include "core/dsp_asm.h"
#include "core/dsp_host.h"
#include "v34/v34_api.h"

extern "C"
{
}

typedef P37_V90Phase4Demodulator D;
typedef P18_V90AutoDigitalImpDetector Adid;

#define PFX "V90Phase4Demodulator: "

/* ------------------------------------------------------------------------------------------------ helpers (static inline) */

/** @brief Constructor body: store the borrowed-object pointers and the session flag (everything else is left uninitialised). */
static inline void ctorBody(D* t, V90MappingParams* mp1, V90MappingParams* mp2, V90Demapper* dem, V90CP* cp, V90MP* mp, Descrambler<unsigned char, int>* descr,
                            V90ConnectionEvaluator* ce, V90Parameters* p, V90Phase3Demodulator* p3, V90AutoDigitalImpDetector* adid, unsigned session)
{
  t->mp1_ = mp1;
  t->params_ = p;
  t->session_ = session;
  t->dem_ = dem;
  t->mp2_ = mp2;
  t->cp_ = cp;
  t->mp_ = mp;
  t->descr_ = descr;
  t->ce_ = ce;
  t->p3_ = p3;
  t->adid_ = adid;
}

static inline void dtorBody(D* t)
{
  /* nothing to release: all pointers are borrowed; the members are destroyed by the compiler */
  KEEP_STORES_TO(t);
}

/**
 * @brief G.711 level code of a symbol magnitude, the index inside a 128-entry row of the linear mapping.
 * @param ad   detector whose pcmType_ low word selects the law (non-zero: A-law, zero: mu-law)
 * @param mag  non-negative linear magnitude
 * @note The value can reach 255, i.e. beyond one 128-entry row (see trn2dKnownDemod()).
 */
static inline int levelOfMagnitude(Adid* ad, int mag)
{
  if ((short)ad->pcmType_ != 0)
    return (int)(((unsigned)linear2alaw(mag) ^ 0xd5u) & 0xffu);
  return (int)(short)(0xffu - ((unsigned)linear2ulaw(mag) & 0xffu));
}

/** @brief |v| computed as (v ^ s) - s with s the sign mask; INT_MIN stays INT_MIN. */
static inline int absIdiom(int v)
{
  int s = v >> 31;
  return (int)(((unsigned)v ^ (unsigned)s) - (unsigned)s);
}

/** @brief Part of reset(): reset the V.92 CP or the V.90 MP (by `session_`) and copy word 0 of mp1_ into its trn2dD_ (and, for V.92, into w34fc_). */
static inline void resetOwners(D* t)
{
  if (t->session_ != 0)
  {
    t->cp_->reset();
    unsigned d = t->mp1_->kBits_;
    t->w34fc_ = d;
    t->cp_->trn2dD_ = d;
  }
  else
  {
    t->mp_->reset();
    t->mp_->trn2dD_ = t->mp1_->kBits_;
  }
}

/** @brief Part of reset(): run n decisions on a zero symbol so the machine settles; `session_` is re-read before every call. */
static inline void settle(D* t, unsigned n)
{
  for (; n != 0; n--)
  {
    if (t->session_ != 0)
      t->getV92Decision(0);
    else
      t->getV90Decision(0);
  }
}

/* ------------------------------------------------------------------------------------------------ construction / destruction */

/** @brief See the declaration; the modulator gets (p, session, NULL, NULL, mp2, mp1, NULL, 0xc) and the detectors get p. */
V90Phase4Demodulator::V90Phase4Demodulator(V90MappingParams* mp1, V90MappingParams* mp2, V90Demapper* dem, V90CP* cp, V90MP* mp,
                                           Descrambler<unsigned char, int>* descr, V90ConnectionEvaluator* ce, V90Parameters* p, V90Phase3Demodulator* p3,
                                           V90AutoDigitalImpDetector* adid, unsigned session)
    : mod_(p, session, 0, 0, mp2, mp1, 0, 0xc), r_(p), rf_(p)
{
  ctorBody(this, mp1, mp2, dem, cp, mp, descr, ce, p, p3, adid, session);
}

/** @brief See the declaration. */
V90Phase4Demodulator::~V90Phase4Demodulator() { dtorBody(this); }

/* ------------------------------------------------------------------------------------------------ the methods */

/** @brief See the declaration. */
void V90Phase4Demodulator::setSessionFlag(unsigned f)
{
  session_ = f;
  mod_.setSessionFlag(f);
}

/** @brief Enter the V.92 wait for the CP (state 4), printing the symbol count of the old state. */
void V90Phase4Demodulator::enterWaitForCP()
{
  edprintf(PFX "enter WaitForV90CP state @ %d\r\n", (int)cnt_);
  state_ = P4D_WAIT_CP;
  cnt_ = 0;
}

/** @brief Enter the V.90 wait for the MP (state 5), printing the symbol count of the old state. */
void V90Phase4Demodulator::enterWaitForMP()
{
  edprintf(PFX "enter WaitForMP state @ %d\r\n", (int)cnt_);
  state_ = P4D_WAIT_MP;
  cnt_ = 0;
}

/** @brief Enter the V.90 wait for Ed (state 6), printing the symbol count of the old state. */
void V90Phase4Demodulator::enterWaitForEd()
{
  edprintf(PFX "enter WaitForEd state @ %d\r\n", (int)cnt_);
  state_ = P4D_WAIT_ED;
  cnt_ = 0;
}

/** @brief Reset `r_` with the RRN R-detection length (parameter rrnRDetectionLength) and `rf_` with a 0xb4-symbol window. */
void V90Phase4Demodulator::resetRRNDetector()
{
  r_.reset((unsigned)params_->rrnRDetectionLength, 0x18);
  rf_.reset(0xb4, 0xc);
}

/**
 * @brief Look for Rd on one symbol; on detection print the polarity and enter state 9 (TRN2d DD).
 * @param x  received symbol
 * @return 1 when Rd was detected, else 0
 * @note For V.90 (`session_` == 0) the Rt and RRN pending flags are set on detection.
 */
int V90Phase4Demodulator::detectRRN(short x)
{
  if (r_.detectR(x) == 0)
    return 0;
  edprintf(PFX "Rd detected, polarity = %d\r\n", (int)r_.dir_);

  state_ = P4D_TRN2D_DD;
  unsigned s = session_;
  cnt_ = 0;
  event_ = 0;
  if (s == 0)
  {
    rtPending_ = 1;
    rrnPending_ = 1;
  }
  return 1;
}

/** @brief Mark an RRN exchange as pending (Rt and RRN flags set) and clear the answer flags and `cpSeen_`. */
void V90Phase4Demodulator::resetBeforRRN()
{
  rtPending_ = 1;
  rrnPending_ = 1;
  answerDue_ = 0;
  answerSent_ = 0;
  cpSeen_ = 0;
}

/**
 * @brief Look for Rf (V.92 fast phase-error) on one symbol; on detection enter state 16 (wait for RfNot).
 * @param x  received symbol
 * @return 1 when Rf was detected, else 0
 * @bug The "Rf detected" print shows the polarity of `r_` (the Ri / Rd / Rt detector), not that of `rf_`, the detector that fired.  Debug output only.
 */
int V90Phase4Demodulator::detectFPE(short x)
{
  if (rf_.detectRf(x) == 0)
    return 0;
  edprintf(PFX "Rf detected, polarity = %d\r\n", (int)r_.dir_);
  edprintf(PFX "enter FPE !");
  state_ = P4D_WAIT_RF_NOT;
  cnt_ = 0;
  event_ = 0;
  return 1;
}

/**
 * @brief Known-data decision for TRN2d: the modulator's own next symbol mapped through the linear mapping of the detector.  No caller in the library.
 * @return sign of the modulator symbol times the linear mapping value for phase (cnt_ - 1) % 6 and the G.711 level of the symbol
 * @warning The level index can be up to 255 while a row of linearMapping_ has 128 entries: the index (phase * 128 + level) can leave the 768-entry table
 *          for phase 5.  Not reachable today because nothing calls this function.
 */
short V90Phase4Demodulator::trn2dKnownDemod(short)
{
  /* the modulator's symbol: sign and magnitude (cut to 16 bits) */
  int v = (int)mod_.generateSymbol();
  int sg = v >> 31;
  unsigned ph = (cnt_ - 1u) % 6u;
  int a = absIdiom(v);
  short sa = (short)a;
  short sign = (short)(sg | 1);
  Adid* ad = adid_;

  /* linear mapping value of this phase and level */
  int level = levelOfMagnitude(ad, absIdiom((int)sa));
  unsigned idx = (ph << 7) + (unsigned)level;
  unsigned val = (unsigned short)adid_->linearMapping_[idx];
  return (short)((int)sign * (int)val);
}

/**
 * @brief See the declaration.  Order: plain members, detectors, CP / MP, demapper, modulator, TRN2d data length, settle decisions.
 * @warning mp1_ and adid_ are dereferenced unconditionally (resetOwners(), the modulator reset); the NULL test before the demapper reset is always true.
 */
void V90Phase4Demodulator::reset(unsigned char code, Phase4DemodulatorState st, unsigned settleCount, unsigned quickConnect)
{
  /* starting state */
  event_ = 0;
  keepRate_ = 0;
  code_ = code;
  cnt_ = 0;
  rtPending_ = 1;
  state_ = (unsigned)st;
  rrnPending_ = 0;
  rrnSilence_ = 0;
  answerDue_ = 0;
  answerSent_ = 0;
  cpSeen_ = 0;

  /* detectors, CP / MP, demapper and the modulator for the TRN2d known data */
  r_.reset((unsigned)params_->phase4RDetectionLength, 0x18);
  rf_.reset((unsigned)params_->phase4RDetectionLength, 0x18);
  quickConnect_ = quickConnect;
  resetOwners(this);
  if (mp1_ != 0 && adid_ != 0)
    dem_->reset(mp1_);
  studyEnd_ = 0;
  mod_.reset((PcmType)adid_->pcmType_, code_, P4M_TRN2D, 0, 0);
  mod_.setMappingParams(mp1_);

  /* length of the TRN2d data: the quick-connect length when requested */
  if (dsplibs_debug_level > 1)
    dsplibs_debug_printf(PFX "reset called, quickConnect indication is %d\r\n", (int)quickConnect_);
  if (quickConnect_ != 0)
    limit_ = (unsigned)params_->trn2dQcDdLength;
  else
    limit_ = (unsigned)params_->trn2dDdLength;
  edprintf(PFX "trn2dDDLength = %d symbols\r\n", (int)limit_);

  /* let the machine settle */
  settle(this, settleCount);
}

/** @brief See the declaration: V.92 when `session_` is non-zero, else V.90. */
short V90Phase4Demodulator::getDecision(short x)
{
  if (session_ != 0)
    return getV92Decision(x);
  return getV90Decision(x);
}
