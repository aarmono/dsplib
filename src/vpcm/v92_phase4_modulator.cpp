/**
 * @file v92_phase4_modulator.cpp
 * @brief V92Phase4Modulator: constructor, reset, the owner's event methods and the small symbol generators (generateSymbol() is in
 *        v92_phase4_modulator_symbol.cpp).
 *
 * The class is declared in v92_phase4_modulator.h.  This file holds the constructor / destructor (a Scrambler member, the V92Mapper allocated here),
 * reset() and resetBeforRRN() / resetBeforFPE(), the owner's event methods (exitCPt(), exitTRN2u(), recived*()) that steer the state machine, and
 * the small generators: generateCPu() / generateSUVu() are called by generateSymbol(), the others have no caller in the library.  Helpers shared
 * with generateSymbol() are in v92_phase4_diff_out.h, v92_phase4_map_block.h and v92_phase4_steps.h.
 *
 * Behaviour worth knowing:
 *  - The constructor stores NOTHING of state_ / cnt_ / event_ / w20_ .. w3c_ / level_ / mult_ / limit_ / inv_ / bits_ / cpBits_ ...: only the Scrambler
 *    (first), then the V92Mapper (sysdep_malloc, constructed), w1c0_, mp_, params_, w1c4_, cp_ (and cp_->cptSent_ = 0, no NULL test), b2s_, cpCnt_ and
 *    cpFlag_; every other byte is whatever the memory held.
 *  - The destructor destroys and frees the V92Mapper when mapper_ != NULL, then destroys the Scrambler; nothing is cleared (the pointer stays).
 *  - reset(): the stores before the mapper reset (event_, limit_, level_, b42_, state_, mult_ = b + 2, cnt_) are visible to it, the mapper is reset with
 *    the STORED (sign-extended) level and b, then the scrambler, then inv_ / the counters / the flags; it does NOT clear w08_ / w24_ / w38_ / w1b8_;
 *    cp_->mult_ = 1, cp_->type_ = 0 (cp_ re-read), infoToBits(), the bit vector, extendEu_ = 0, and generateSymbol() is called n times (n == 0: none).
 *  - The owner's event methods compare the symbol counter with the (possibly stale, 32-bit unsigned) frame length cpFrame_; a zero frame length is a
 *    division by zero.  "enter E2u" is state_ = 0xf, cnt_ = 0, w1b8_ = (extendEu_ == 0 ? 0xc : 0xd) (extendEu_ read after the print);
 *    recivedFirstRrnEd() stores w1b8_ = 0xc and THEN complains if extendEu_ != 0.
 *  - recivedSUVtag() / recivedCPtag() / recivedEd() compare the state as a SIGNED value against 5 (a state >= 2^31 is "below 5") and then (state - 0xc)
 *    as an unsigned value against 1.
 *  - recivedPartOneSilenceRrnSUVtag() sets cp_->flag_ = 1 on EVERY path (also when w20_ != 0), after the other stores.
 *  - generateCPu() / generateSUVu() (identical code) walk cpBits_ at w08_ modulo cpLen_ (a zero cpLen_ is a division by zero) and re-read mult_
 *    every iteration.
 */

#include "v92_phase4_modulator.h"
#include "core/dsp_host.h"
#include "v34/v34_api.h"

extern "C"
{
}

/* ------------------------------------------------------------------------------------------------ helpers (static inline) */

#include "core/dsp_intmath.h"
#include "core/dsp_debug.h"

/* The six-symbol pattern of Ru / RuNot: (cnt - 1) % 6 <= 2 -> +level, else -level. */
static inline short ruPattern(unsigned short level, unsigned cnt)
{
  unsigned r = (cnt - 1) % 6;
  return r <= 2 ? (short)level : neg16((short)level);
}

#include "v92_phase4_diff_out.h"

#include "v92_phase4_map_block.h"

#include "v92_phase4_steps.h"

/* One scrambled CP bit: bits_[i] = process(cpBits_[w08_]); w08_ = (w08_ + 1) % cpLen_. */
static inline void cpBitStep(V92Phase4Modulator* t, unsigned i)
{
  unsigned char b = t->cpBits_[t->w08_];
  t->bits_[i] = (unsigned char)t->scr_.process(b);
  t->w08_ = (t->w08_ + 1) % t->cpLen_;
}
/* The CP / SUV bit generator of generateCPu / generateSUVu: direct mapping of mult_ bits, or (FPE mode) the bits the queue asks for. */
static inline short stepCpSuv(V92Phase4Modulator* t)
{
  if (t->w3c_ == 0)
  {
    unsigned i = 0;
    if (t->mult_ != 0)
    {
      do
      {
        cpBitStep(t, i);
        i++;
      } while (i < (unsigned)t->mult_);
    }
    return mapBlock(t);
  }
  short out = 0;
  unsigned n = t->b2s_->nofBitsForNextTime();
  if (n != 0)
  {
    unsigned i = 0;
    if (i < n)
    {
      do
      {
        cpBitStep(t, i);
        i++;
      } while (i < n);
    }
    t->b2s_->process(t->bits_, n);
  }
  t->b2s_->process(n, &out);
  return out;
}

/* A CP restart with the CP's own u8 multiplier: infoToBits, the new bit vector; returns the frame length cpLen_ / cp_->mult_ (cp_ re-read after the calls). */
static inline unsigned cpStartCp(V92Phase4Modulator* t)
{
  t->cp_->infoToBits();
  t->cpBits_ = t->cp_->getBitVector(t->cpLen_);
  return t->cpLen_ / t->cp_->mult_;
}

/* "enter E2u": print (the caller's format), set the state, then w1b8_ from extendEu_ (read after the print, before the cnt_ store). */
static inline void enterE2u(V92Phase4Modulator* t, const char* fmt, unsigned c)
{
  edprintf(fmt, c);
  t->state_ = V92P4_E2U;
  unsigned f = t->extendEu_;
  t->cnt_ = 0;
  t->w1b8_ = f == 0 ? 0xc : 0xd;
}
#define E2U_FMT "V92Phase4Modulator: enter E2u @ %d\r\n"
#define E2U_FIRST_FMT "V92Phase4Modulator: enter E2u First at RRN @ %d\r\n"

/* The SUV -> CPu end of recivedSUV / recivedPartTwoSilenceRrnSUV (identical in both): continue SUVu until the frame boundary, then enter CPu. */
static inline void suvToCpu(V92Phase4Modulator* t)
{
  if (t->w1c4_ != 0)
    return;
  if (t->state_ != V92P4_SUVU)
    return;
  unsigned c = t->cnt_;
  if (c % t->cpFrame_ != 0)
  {
    t->state_ = V92P4_SUVU_TO_CPU;
    return;
  }
  edprintf("V92Phase4Modulator: on recivedSUV enter CPu @ %d\r\n", c);
  t->state_ = V92P4_CPU_END;
  t->cnt_ = 0;
  t->cp_->type_ = 0;
  unsigned f = cpStartCp(t);
  t->w1c4_ = 1;
  t->cpFrame_ = f;
}

/* The state after a tag at a non-boundary: 5 -> 9; signed < 5: nothing; (state - 0xc) as an unsigned value > 1: nothing; 0xc / 0xd -> 0xb.
 * Returns true when w20_ = 1 follows. */
static inline bool tagState(V92Phase4Modulator* t)
{
  int s = (int)t->state_;
  if (s == V92P4_SUVU)
  {
    t->state_ = V92P4_SUVU_9;
    return true;
  }
  if (s < 5)
    return false;
  if ((unsigned)s - 0xcu > 1u)
    return false;
  t->state_ = V92P4_CPU_11;
  return true;
}

/* The bodies of the constructor and the destructor. */
static inline void ctorBody(V92Phase4Modulator* t, V92Parameters* p, V92BitsToSymbol* b2s, V92CP* cp, V92MappingParams* mp)
{
  V92Mapper* m = (V92Mapper*)sysdep_malloc((unsigned)sizeof(V92Mapper));
  new (m) V92Mapper();
  t->mapper_ = m;
  t->w1c0_ = 0;
  t->mp_ = mp;
  t->params_ = p;
  t->w1c4_ = 0;
  t->cp_ = cp;
  cp->cptSent_ = 0;
  t->b2s_ = b2s;
  t->cpCnt_ = 0;
  t->cpFlag_ = 0;
}

static inline void dtorBody(V92Phase4Modulator* t)
{
  V92Mapper* m = t->mapper_;
  if (m != 0)
  {
    m->~V92Mapper();
    sysdep_free(m);
  }
}

/** @brief Build the scrambler and the V92Mapper and store the collaborators; see the file header for what is left uninitialised. */
V92Phase4Modulator::V92Phase4Modulator(V92Parameters* p, V92BitsToSymbol* b2s, V92CP* cp, V92MappingParams* mp) : scr_(5, 0x17, 0x63)
{
  ctorBody(this, p, b2s, cp, mp);
}

/** @brief Destroy and free the V92Mapper. */
V92Phase4Modulator::~V92Phase4Modulator() { dtorBody(this); }

/* ------------------------------------------------------------------------------------------------ reset */

/**
 * @brief Restart the generator in state `st` and run n symbols.
 * @param level  symbol magnitude
 * @param b      mapper parameter (mult_ = b + 2 bits per mapped block)
 * @param st     initial state
 * @param n      number of symbols to generate and discard
 * @param limit  repeat limit (V92Phase2Info::rtd_)
 */
void V92Phase4Modulator::reset(short level, unsigned char b, V92Phase4ModulatorState st, unsigned n, unsigned limit)
{
  /* state visible to the mapper reset */
  event_ = 0;
  limit_ = limit;
  level_ = (unsigned short)level;
  b42_ = b;
  state_ = (unsigned)st;
  mult_ = (unsigned char)(b + 2);
  cnt_ = 0;

  mapper_->reset((short)level_, b);
  scr_.reset(0);
  inv_ = 0;

  /* clear the flags and counters, then start a fresh CP */
  V92CP* cp = cp_;
  w1c0_ = 0;
  cp->cptSent_ = 0;
  w1c4_ = 0;
  w28_ = 0;
  w3c_ = 0;
  rrnSilence_ = 0;
  w30_ = 0;
  w34_ = 0;
  cpCnt_ = 0;
  cpFlag_ = 0;
  w20_ = 0;

  cp->mult_ = 1;
  cp_->type_ = 0;
  cp_->infoToBits();
  cpBits_ = cp_->getBitVector(cpLen_);
  extendEu_ = 0;

  for (unsigned i = 0; i < n; i++)
    generateSymbol();
}

/* ------------------------------------------------------------------------------------------------ small setters */

/** @brief Rate-renegotiation start: clear the counters and flags, w28_ = 1. */
void V92Phase4Modulator::resetBeforRRN()
{
  w1c4_ = 0;
  V92CP* cp = cp_;
  w1c0_ = 0;
  cnt_ = 0;
  cp->cptSent_ = 0;
  w28_ = 1;
  rrnSilence_ = 0;
  w30_ = 0;
  w34_ = 0;
  w38_ = 0;
  w20_ = 0;
}

/** @brief Final-exchange start: cnt_ = 0, w3c_ = 1 (queue mode). */
void V92Phase4Modulator::resetBeforFPE()
{
  cnt_ = 0;
  w3c_ = 1;
}

/** @brief Clear the flags of the second RRN section (w34_ = 1).  No caller. */
void V92Phase4Modulator::resetRRNSecondSection()
{
  w1c4_ = 0;
  w34_ = 1;
  w20_ = 0;
  cpFlag_ = 0;
  cpCnt_ = 0;
  w1c0_ = 0;
  V92CP* cp = cp_;
  cp->cptSent_ = 0;
  cp->flag_ = 0;
}

/** @brief Enter the repeated-CPu state and restart the CP frame.  No caller. */
void V92Phase4Modulator::enterRepeatedCP()
{
  cpFlag_ = 0;
  cpCnt_ = 0;
  if (dbgOn())
    dsplibs_debug_printf("V92Phase4Modulator: enter repeatedCPu @ %d\r\n", cnt_);
  state_ = V92P4_REPEATED_CPU;
  cp_->type_ = 0;
  unsigned f = cpStartCp(this);
  cnt_ = 0;
  cpFrame_ = f;
}

/** @brief Give the bits-to-symbol queue the mapping parameters and block size 1; NULL only prints a debug line.  No caller. */
void V92Phase4Modulator::setMappingParams(V92MappingParams* mp)
{
  if (mp == 0)
  {
    if (dbgOn())
      dsplibs_debug_printf("V92Phase4Modulator: ERROR: Null mappingParams @ setMappingParams\r\n");
    return;
  }
  b2s_->reset(mp);
  b2s_->setSymbolsBlockSize(1);
}

/* ------------------------------------------------------------------------------------------------ the owner's events */

/**
 * @brief The CPt was received: in Ru, at a frame boundary enter E1u, else run Ru to the end of the frame (state 1).
 * @warning Division by cpFrame_ (zero before the first CP restart).
 */
void V92Phase4Modulator::exitCPt()
{
  if (state_ != V92P4_RU)
    return;
  unsigned c = cnt_;
  if (c == 0)
    return;
  if ((c - 0x18) % cpFrame_ != 0)
  {
    state_ = V92P4_RU_END;
    return;
  }
  edprintf("V92Phase4Modulator: enter E1u @ %d\r\n", c);
  state_ = V92P4_E1U;
  cnt_ = 0;
}

/** @brief In TRN2u after at least one symbol: continue to a multiple of 12 symbols (state 4). */
void V92Phase4Modulator::exitTRN2u()
{
  if (state_ != V92P4_TRN2U)
    return;
  if (cnt_ == 0)
    return;
  state_ = V92P4_TRN2U_A;
}

/** @brief A SUV was received: finish the SUV frame and enter CPu (see suvToCpu()). */
void V92Phase4Modulator::recivedSUV() { suvToCpu(this); }

/** @brief First silent part of an RRN SUV: set the CP's flag_. */
void V92Phase4Modulator::recivedPartOneSilenceRrnSUV() { cp_->flag_ = 1; }

/** @brief Second silent part of an RRN SUV: as recivedSUV(). */
void V92Phase4Modulator::recivedPartTwoSilenceRrnSUV() { suvToCpu(this); }

/** @brief A CP was received: w1c0_ = 1, the CP's flag_ = 1, cptSent_ = 0. */
void V92Phase4Modulator::recivedCP()
{
  V92CP* cp = cp_;
  w1c0_ = 1;
  cp->flag_ = 1;
  cp_->cptSent_ = 0;
}

/**
 * @brief A SUV was received after a CP: once the CP has been sent, at the next frame boundary enter E2u, else switch to the state that runs to it.
 * @warning Division by cpFrame_.
 */
void V92Phase4Modulator::recivedSUVtag()
{
  cpFlag_ = 0;
  cpCnt_ = 0;
  if (dbgOn())
    dsplibs_debug_printf("recivedSUVtag called\r\n");
  if (w1c0_ == 0)
    return;
  if (cp_->cptSent_ == 0)
    return;
  if (w20_ != 0)
    return;
  unsigned st = state_;
  bool five = (int)st == V92P4_SUVU;
  if (!five)
  {
    if ((int)st < 5)
      return;
    if (st - 0xcu > 1u)
      return;
  }
  unsigned c = cnt_;
  if (c % cpFrame_ != 0)
  {
    state_ = five ? V92P4_SUVU_9 : V92P4_CPU_11;
  }
  else
  {
    enterE2u(this, E2U_FMT, c);
  }
  w20_ = 1;
}

/**
 * @brief First silent part of a tagged RRN SUV: enter E2u, SUVu -> E2u or FinalSUVu depending on the CP flag and the frame position.
 * @note  The CP's flag_ is set on every path.
 * @warning Division by cpFrame_.
 */
void V92Phase4Modulator::recivedPartOneSilenceRrnSUVtag()
{
  if (w20_ == 0)
  {
    V92CP* cp = cp_;
    if (cp->flag_ == 1)
    {
      unsigned c = cnt_;
      if (c % cpFrame_ == 0)
      {
        enterE2u(this, E2U_FIRST_FMT, c);
      }
      else
      {
        state_ = V92P4_SUVU_9;
      }
      w20_ = 1;
    }
    else
    {
      cp->flag_ = 1;
      if (cnt_ % cpFrame_ == 0)
      {
        state_ = V92P4_SUVU_10;
        w20_ = 1;
        cnt_ = 0;
        cp_->type_ = 1;
        unsigned f = cpStartCp(this);
        cpFrame_ = f;
      }
      else
      {
        state_ = V92P4_FINAL_SUVU_ENTER;
        w20_ = 1;
      }
    }
  }
  cp_->flag_ = 1;
}

/** @brief Second silent part of a tagged RRN SUV: as recivedSUVtag(). */
void V92Phase4Modulator::recivedPartTwoSilenceRrnSUVtag() { recivedSUVtag(); }

/**
 * @brief A tagged CP was received: on the first one start FinalSUVu / SUVu (CP type 1), later ones steer towards E2u like recivedEd().
 * @warning Division by cpFrame_.
 */
void V92Phase4Modulator::recivedCPtag()
{
  cpFlag_ = 0;
  unsigned seen = w1c0_;
  cpCnt_ = 0;
  if (seen == 0)
  {
    if (w20_ != 0)
      return;
    V92CP* cp = cp_;
    w1c0_ = 1;
    cp->flag_ = 1;
    cp_->type_ = 1;
    if (cnt_ % cpFrame_ == 0)
    {
      state_ = V92P4_SUVU_10;
      cnt_ = 0;
      unsigned f = cpStartCp(this);
      cpFrame_ = f;
    }
    else
    {
      state_ = V92P4_FINAL_SUVU_ENTER;
    }
    w20_ = 1;
    return;
  }
  if (w20_ != 0)
    return;
  unsigned c = cnt_;
  if (c % cpFrame_ == 0)
  {
    enterE2u(this, E2U_FMT, c);
    w20_ = 1;
    return;
  }
  if (tagState(this))
    w20_ = 1;
}

/** @brief Ed was received: at a frame boundary enter E2u, else steer towards it (once, guarded by w20_). */
void V92Phase4Modulator::recivedEd()
{
  if (w20_ != 0)
    return;
  unsigned c = cnt_;
  if (c % cpFrame_ == 0)
  {
    enterE2u(this, E2U_FMT, c);
    w20_ = 1;
    return;
  }
  if (tagState(this))
    w20_ = 1;
}

/** @brief The first Ed of an RRN was received: at a frame boundary enter E2u (length 0xc), else continue SUVu towards it. */
void V92Phase4Modulator::recivedFirstRrnEd()
{
  if (w20_ != 0)
    return;
  unsigned c = cnt_;
  if (c % cpFrame_ != 0)
  {
    state_ = V92P4_SUVU_9;
    w20_ = 1;
    return;
  }
  edprintf(E2U_FIRST_FMT, c);
  state_ = V92P4_E2U;
  unsigned f = extendEu_;
  cnt_ = 0;
  w1b8_ = 0xc;
  if (f != 0)
    edprintf("V92Phase4Modulator: ERROR: E2u is extended in RRN !!!\r\n");
  w20_ = 1;
}

/**
 * @brief Rt was received in the second TRN2u part of an RRN (acted on only when w38_ != 0).
 *
 * If more than 0x95f symbols were sent and the count is a multiple of 12, enter SUVu with a new SUV bit vector; otherwise switch to state 0x18 which
 * runs to that point.  A NULL cp_ enters the error state.
 */
void V92Phase4Modulator::recivedRt()
{
  if (state_ != V92P4_TRN2U_SECOND)
    return;
  if (w38_ == 0)
    return;
  unsigned c = cnt_;
  if (c <= 0x95f || c != (c / 12) * 12)
  {
    state_ = V92P4_TRN2U_SECOND_A;
    return;
  }
  if (cp_ == 0)
  {
    bool d = dbgOn();
    state_ = V92P4_ERROR;
    cnt_ = 0;
    if (d)
      dsplibs_debug_printf("V92Phase4Modulator: ERROR: Null CP at RRN @ end of TRN2d\r\n");
    return;
  }
  edprintf("V92Phase4Modulator: on recivedRt enter SUV @ %d\r\n", c);
  unsigned char m = mult_;
  cnt_ = 0;
  V92CP* cp = cp_;
  w08_ = 0;
  state_ = V92P4_SUVU;
  cp->mult_ = m;
  cp_->type_ = 1;
  cp_->suvVal_ = 0;
  cp_->infoToBits();
  unsigned char* bv = cp_->getBitVector(cpLen_);
  unsigned d = mult_;
  cpBits_ = bv;
  cpFrame_ = cpLen_ / d;
}

/* ------------------------------------------------------------------------------------------------ the generators */

/** @brief Ru pattern symbol (no caller). */
short V92Phase4Modulator::generateRu() { return ruPattern(level_, cnt_); }
/** @brief Inverted Ru pattern symbol (no caller). */
short V92Phase4Modulator::generateRuNot() { return neg16(ruPattern(level_, cnt_)); }

/**
 * @brief Data symbol before the FPE; enters Rm when the queue asks for bits (no caller).
 * @warning n and out are uninitialised when passed to V92BitsToSymbol::process(), which writes both.
 */
short V92Phase4Modulator::generateDataSymbolBeforeFPE()
{
  unsigned n;
  short out;
  b2s_->process(n, &out);
  if (n != 0)
  {
    edprintf("V92Phase4Modulator: enter Rm @ %d\r\n", cnt_);
    state_ = V92P4_RM;
    cnt_ = 0;
    b2s_->tx_->me_->mode_ = 1;
  }
  return out;
}

/**
 * @brief Data symbol before the RRN; enters the Ru pattern when the queue asks for bits (no caller).
 * @warning n and out are uninitialised when passed to V92BitsToSymbol::process(), which writes both.
 */
short V92Phase4Modulator::generateDataSymbolBeforeRRN()
{
  unsigned n;
  short out;
  b2s_->process(n, &out);
  if (n != 0)
  {
    edprintf("V92Phase4Modulator: enter Ru @ %d\r\n", cnt_);
    state_ = V92P4_RU_PAT;
    cnt_ = 0;
  }
  return out;
}

/** @brief Rm symbol: scrambled ones through the queue (no caller). */
short V92Phase4Modulator::generateRm() { return stepOnes(this); }
short V92Phase4Modulator::generateB1u() { return stepOnes(this); }

/** @brief TRN2u symbol: scrambled ones through the mapper (no caller). */
short V92Phase4Modulator::generateTRN2u()
{
  scr_.processAllOnes(bits_, mult_);
  return mapBlock(this);
}

/** @brief E2u symbol: scrambled zeros through the mapper, or through the queue in FPE mode (no caller). */
short V92Phase4Modulator::generateE2u()
{
  if (w3c_ == 0)
  {
    scr_.processAllZeros(bits_, mult_);
    return mapBlock(this);
  }
  return stepZeros(this);
}

/** @brief CPu symbol (shares stepCpSuv() with generateSUVu()). */
short V92Phase4Modulator::generateCPu() { return stepCpSuv(this); }
short V92Phase4Modulator::generateSUVu() { return stepCpSuv(this); }

/** @brief E1u symbol: scrambled zeros in the differential stream (no caller). */
short V92Phase4Modulator::generateE1u()
{
  unsigned char r = (unsigned char)scr_.process(0);
  return diffOut(this, (unsigned)r ^ inv_);
}

/** @brief CPt symbol: toggling for the first 0x18 symbols, then the scrambled CP bits (no caller). */
short V92Phase4Modulator::generateCPt()
{
  unsigned c = cnt_;
  if (c <= 0x18)
    return diffOut(this, inv_ ^ 1);
  unsigned char bit = cpBits_[(c - 0x19) % cpLen_];
  unsigned char r = (unsigned char)scr_.process(bit);
  return diffOut(this, (unsigned)r ^ inv_);
}
