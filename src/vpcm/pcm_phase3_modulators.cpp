/**
 * @file pcm_phase3_modulators.cpp
 * @brief V.90 / V.92 phase-3 transmit symbol generators: V90Phase3Modulator and V92Phase3Modulator.
 *
 * Each class is a state machine that returns one 16-bit PCM-level symbol per generateSymbol() call and counts the symbols of the current state in `cnt_`
 * (see pcm_phase3_modulators.h for the state lists).  The bit-valued states take their bits from a Scrambler<unsigned char, int> (`scr_`) and the frame
 * bits supplied by V90Jd / V92Jd; the levels come from G.711 codes (linear2alaw() / alaw2linear() and the mu-law equivalents).  The DIL states replay the
 * descriptor of pcm_dil.h: resetDILGenerator() converts it to linear levels and dilStep() steps through its segments.
 *
 * Behaviour that a tidier rewrite would change:
 * - The symbol counter is incremented before the state dispatch, also for an illegal state.  A state clears the output marker `event_` first (V.90
 *   state 1 only when it does not end).  The order of the stores to `state_` / `cnt_` / `event_` before a diagnostic print is part of the behaviour.
 * - Symbols are 16-bit values; a negated level is `(int16)(-level)`, so -32768 stays -32768.  Sign rules: V.90 states 3 / 7 / 8 and V.92 states 3 / 4 / 5 /
 *   6 give +level when `bit ^ jdBit_` is nonzero and -level when zero; TRN1d (V.90 state 2) gives -level for a zero scrambler output; V.92 TRN1u
 *   (states 3 / 0xc / 0xd) gives +level for a zero output; Ja (V.92 states 4 / 5) gives -level for a nonzero value.
 * - The Jd and V92Jd time-out tests compare `cnt_` with `timeBase_ + 24804` as a 64-bit sum (no wrap at 2^32); the V.92 JdPhase state compares `cnt_` with
 *   24804 alone; the DIL time-out is the 32-bit sum `timeBase_ + 40000`, which wraps.
 * - The DIL step reads `segLen_[seg_]` and `ref_[seg_]` with `seg_` up to 8, one past the arrays: the read lands in the following member (`ref_[0..1]`
 *   and `to_[0]`), which the layout keeps adjacent.  Both are reached through byte offsets of the real members.
 * - The segment search compares the zero-extended 16-bit level with the int limits as SIGNED values and gives 0 .. 7, or 8 when no limit is reached; the row is
 *   `alaw_ * 8` of the 16-word table (callers keep `alaw_` <= 1).
 * - The arguments of linear2alaw() / linear2ulaw() are the zero-extended words of `to_[]`; negative levels arrive as 65536 - |v|.
 * - V92Phase3Modulator::reset() ignores its first argument and stores 0xfa0 in levelA_; levelB_ is trunc((float)(4000 * 1.2247448713915890) + 0.5f) =
 *   4899, computed
 *   through a float slot so the rounding is that of a single-precision store.
 * - The V.92 TRN1u length is 12 * ((fast + slow + 12) / 12) with a SIGNED divide (truncating toward zero), taken only if it exceeds 0x1fdf (else 0x1fe0).
 */

#include "pcm_phase3_modulators.h"
#include "core/dsp_host.h"
#include "v34/v34_api.h"

extern "C"
{
}

typedef long double LD;

/** Segment limits of the DIL level search: 8 per companding law, mu-law in row 0 and A-law in row 1. */
int V90Phase3Modulator::codeSegmentsBoundriesLookupTable[16] = {0x7c,  0x17c, 0x37c, 0x77c, 0xf7c,  0x1f7c, 0x3f7c, 0x7f7c,
                                                                0x100, 0x200, 0x400, 0x800, 0x1000, 0x2000, 0x4000, 0x8000};

/* ------------------------------------------------------------------------------------------------ helpers */

#include "core/dsp_floatbits.h"

#include "core/dsp_x87_conv.h"

/* A 16-bit level negated, or unchanged, as a signed 16-bit symbol (-32768 stays -32768). */
static inline short neg16(unsigned short v) { return (short)(-(int)(short)v); }
static inline short pos16(unsigned short v) { return (short)v; }
/* The level, negated when `x` is zero (negIfZero) or nonzero (negIfNonzero). */
static inline short negIfZero(unsigned x, unsigned short v) { return x == 0 ? neg16(v) : pos16(v); }
static inline short negIfNonzero(unsigned x, unsigned short v) { return x != 0 ? neg16(v) : pos16(v); }

/* G.711 code to linear level: the A-law (xor 0xd5) or mu-law (xor 0xff) decoder applied to the magnitude bits, sign bit dropped. */
static inline unsigned short decodeLevel(unsigned alaw, unsigned char code)
{
  return alaw != 0 ? (unsigned short)alaw2linear((code & 0x7f) ^ 0xd5) : (unsigned short)ulaw2linear((code & 0x7f) ^ 0xff);
}

/* Word k of segLen_ and of ref_.  k may be 8, one past the array: the read then lands in the next member (ref_[0..1] / to_[0]), see the file header. */
static inline unsigned segLenAt(const P27_V90Phase3Modulator* t, unsigned k)
{
  return *(const unsigned*)((const char*)t + offsetof(P27_V90Phase3Modulator, segLen_) + 4u * k);
}
static inline unsigned short refAt(const P27_V90Phase3Modulator* t, unsigned k)
{
  return *(const unsigned short*)((const char*)t + offsetof(P27_V90Phase3Modulator, ref_) + 2u * k);
}

/* sp_ / tp_ and the descriptor's lists are indexed by a byte (up to 255) without a bound check although the arrays have 128 entries; the accessors address
 * them by byte offset so an index past the end runs on into the neighbouring members instead of being an out-of-bounds array access. */
static inline unsigned char& spAt(P27_V90Phase3Modulator* t, unsigned i) { return *((unsigned char*)t + offsetof(P27_V90Phase3Modulator, sp_) + i); }
static inline unsigned char& tpAt(P27_V90Phase3Modulator* t, unsigned i) { return *((unsigned char*)t + offsetof(P27_V90Phase3Modulator, tp_) + i); }
static inline unsigned char dSp(const tagV90DILdescriptor* d, unsigned i) { return *((const unsigned char*)d + offsetof(tagV90DILdescriptor, sp_) + i); }
static inline unsigned char dTp(const tagV90DILdescriptor* d, unsigned i) { return *((const unsigned char*)d + offsetof(tagV90DILdescriptor, tp_) + i); }

/* Code segment of the level to_[toIdx_]: the index of the first limit (row alaw_ * 8) that the zero-extended level does not exceed, 8 when none. */
static inline unsigned char segmentOf(const P27_V90Phase3Modulator* t)
{
  unsigned level = t->to_[t->toIdx_];
  unsigned k = 0;
  do
  {
    if ((int)level <= V90Phase3Modulator::codeSegmentsBoundriesLookupTable[t->alaw_ * 8u + k])
      break;
    k++;
  } while (k <= 7);
  return (unsigned char)k;
}

/* One step of the DIL generator (generateDIL(); states 9 and 0xa of both symbol generators).  Returns the 16-bit symbol: the reference level of the segment
 * or the DIL level, negated when the sign-list entry is zero.  Then updates the G.711 code of the level, and either advances the sign / type positions or,
 * at the end of a segment, restarts them, moves to the next DIL level (wrapping at dilN_) and recomputes the code segment. */
static inline short dilStep(P27_V90Phase3Modulator* t)
{
  unsigned char tp = t->tpIdx_;
  short v;

  if (tpAt(t, tp) == 0)
    v = (short)refAt(t, t->seg_);
  else
    v = (short)t->to_[t->toIdx_];
  t->isRef_ = (unsigned short)(tpAt(t, t->tpIdx_) == 0);

  /* G.711 code of the current level */
  {
    unsigned arg = t->to_[t->toIdx_];
    t->code_ = t->alaw_ != 0 ? (unsigned char)(linear2alaw((int)arg) ^ 0xd5) : (unsigned char)~linear2ulaw((int)arg);
  }

  /* advance the sign / type / level positions */
  {
    unsigned char nextTp = (unsigned char)(t->tpIdx_ + 1);
    unsigned char spi = t->spIdx_;
    unsigned char nextSp;
    unsigned cnt;
    if (nextTp == t->dilLtp_)
      nextTp = 0;

    if (spAt(t, spi) == 0)
      v = neg16((unsigned short)v);

    nextSp = (unsigned char)(spi + 1);
    if (nextSp == t->dilLsp_)
      nextSp = 0;
    cnt = t->segCnt_ + 1;

    if (cnt == segLenAt(t, t->seg_))
    {
      unsigned char al;
      t->segCnt_ = 0;
      t->tpIdx_ = 0;
      t->spIdx_ = 0;
      al = (unsigned char)(t->toIdx_ + 1);
      if (al == t->dilN_)
        t->toIdx_ = 0;
      else
        t->toIdx_ = al;
      t->seg_ = segmentOf(t);
    }
    else
    {
      t->spIdx_ = nextSp;
      t->segCnt_ = cnt;
      t->tpIdx_ = nextTp;
    }
  }
  return v;
}

/* Period-6 pattern of the Sd / SdNot states, `mod` = position in the period: +levelB_ at 0 / 2, levelC_ at 1, -levelB_ at 3 / 5, -levelC_ at 4.
 * `flip` negates every sign (SdNot). */
static inline short sdPattern(const P27_V90Phase3Modulator* t, unsigned mod, bool flip)
{
  short v = 0;
  switch (mod)
  {
  case 0:
  case 2:
    v = flip ? neg16(t->levelB_) : pos16(t->levelB_);
    break;
  case 1:
    v = flip ? neg16(t->levelC_) : pos16(t->levelC_);
    break;
  case 3:
  case 5:
    v = flip ? pos16(t->levelB_) : neg16(t->levelB_);
    break;
  case 4:
    v = flip ? pos16(t->levelC_) : neg16(t->levelC_);
    break;
  default:
    break;
  }
  return v;
}

/* The V.92 Ru / RuNot pattern: +levelA_ for mod 0 .. 2, -levelA_ for mod 3 .. 5 (`flip`: the opposite). */
static inline short ruPattern(const P27_V92Phase3Modulator* t, unsigned mod, bool flip)
{
  short v = 0;
  if (mod <= 2)
    v = flip ? neg16(t->levelA_) : pos16(t->levelA_);
  else if (mod <= 5)
    v = flip ? pos16(t->levelA_) : neg16(t->levelA_);
  return v;
}

/* The V.92 Su / SuNot pattern: +levelB_ at mod 0 / 2, 0 at 1 / 4, -levelB_ at 3 / 5 (`flip`: the signs reversed). */
static inline short suPattern(const P27_V92Phase3Modulator* t, unsigned mod, bool flip)
{
  short v = 0;
  switch (mod)
  {
  case 0:
  case 2:
    v = flip ? neg16(t->levelB_) : pos16(t->levelB_);
    break;
  case 1:
  case 4:
    v = 0;
    break;
  case 3:
  case 5:
    v = flip ? pos16(t->levelB_) : neg16(t->levelB_);
    break;
  default:
    break;
  }
  return v;
}

/* Constructor bodies, kept out of the initializer lists. */
static inline void v90CtorBody(P27_V90Phase3Modulator* t, V90Parameters* p, unsigned flag)
{
  t->params_ = p;
  t->flag_ = flag;
  t->reset(PCM_MU_LAW, 0x40, (Phase3ModulatorState)0, 0, 0, 0, 0, 0);
}
static inline void v92CtorBody(P27_V92Phase3Modulator* t, V92Parameters* p)
{
  t->params_ = p;
  t->reset(0xfa0, (V92Phase3ModulatorState)0, 0, 0, 0, 0);
}

/* ------------------------------------------------------------------------------------------------ V90Phase3Modulator */

/**
 * @brief Construct the generator and restart it in state 0 with mu-law code 0x40.
 * @param p     V.90 parameter block, stored
 * @param flag  0 for a V.90 session, nonzero for a V.92 session
 */
V90Phase3Modulator::V90Phase3Modulator(V90Parameters* p, unsigned flag) : scr_(0x12, 0x17, 0x63) { v90CtorBody(this, p, flag); }

V90Phase3Modulator::~V90Phase3Modulator() {}

/** @brief Select the V.90 (0) or V.92 (nonzero) session behaviour. */
void V90Phase3Modulator::setSessionFlag(unsigned f) { flag_ = f; }

/** @brief Sd pattern symbol for the current `cnt_` (unused: generateV90Symbol() inlines it). */
short V90Phase3Modulator::generateSd() { return sdPattern(this, (cnt_ - 1u) % 6u, false); }

/** @brief SdNot pattern symbol for the current `cnt_` (unused). */
short V90Phase3Modulator::generateSdNot() { return sdPattern(this, (cnt_ - 1u) % 6u, true); }

/**
 * @brief End-of-state decision for the Jd state (unused).
 * @note  Does nothing outside state 3 or with `cnt_` == 0.  On a 72-symbol frame boundary it resets `cnt_` and goes to state 8 (V.90) / 5 (V.92); otherwise
 *        it goes to state 7 (V.90) / 4 (V.92) to finish the frame.
 */
void V90Phase3Modulator::exitJd()
{
  if (state_ != P3M_JD)
    return;
  if (cnt_ == 0)
    return;
  if (cnt_ % 72u == 0)
  {
    cnt_ = 0;
    state_ = (flag_ == 0 ? 3u : 0u) + 5u;
  }
  else
  {
    state_ = (flag_ == 0 ? 3u : 0u) + 4u;
  }
}

/**
 * @brief End-of-state decision for the JdPhase state (unused).
 * @note  In state 5 with `cnt_` != 0: state 8 and `cnt_` = 0 on a frame boundary, else state 6.
 */
void V90Phase3Modulator::exitJdPhase()
{
  if (state_ != P3M_STATE5)
    return;
  if (cnt_ == 0)
    return;
  if (cnt_ % 72u == 0)
  {
    state_ = P3M_JD_NOT_END;
    cnt_ = 0;
  }
  else
  {
    state_ = P3M_STATE6;
  }
}

/**
 * @brief End-of-state decision for the DIL state, called by the owner after a symbol with `event_` != 0.
 * @note  In state 9 with `cnt_` != 0: if the current DIL segment is not complete (`segCnt_` != 0) go to state 0xa to finish it; otherwise print "Phase3
 *        Terminated @ %d", go to state 0xb, reset `cnt_` and raise event 6.
 */
void V90Phase3Modulator::exitDIL()
{
  if (state_ != P3M_DIL)
    return;
  unsigned c = cnt_;
  if (c == 0)
    return;
  unsigned s = segCnt_;
  if (s != 0)
  {
    state_ = P3M_DIL_END;
    return;
  }
  edprintf("V90Phase3Modulator: Phase3 Terminated @ %d\r\n", c);
  state_ = P3M_DONE;
  cnt_ = 0;
  event_ = 6;
}

/** @brief Recompute the code segment `seg_` of the current DIL level (unused). */
void V90Phase3Modulator::updateCodeSegmentPointer() { seg_ = segmentOf(this); }

/**
 * @brief Load a DIL descriptor: copy its lists, convert the level bytes to linear levels and restart the DIL positions.
 * @param d  descriptor, or NULL to clear the DIL (`dilN_` = 0, nothing else changes)
 * @note  `segLen_[k]` = 6 * H[k] + 6 symbols; the reference levels and DIL levels are decoded with the current companding law (`alaw_`).
 * @warning `Lsp` / `Ltp` (bytes) are not limited to the 128-entry `sp_` / `tp_`; see the declaration.
 */
void V90Phase3Modulator::resetDILGenerator(const tagV90DILdescriptor* d)
{
  if (d == 0)
  {
    dilN_ = 0;
    return;
  }
  dilN_ = d->N_;
  dilLsp_ = d->Lsp_;
  dilLtp_ = d->Ltp_;
  for (unsigned i = 0; i < (unsigned)d->Lsp_; i++)
    spAt(this, i) = dSp(d, i);
  for (unsigned i = 0; i < (unsigned)d->Ltp_; i++)
    tpAt(this, i) = dTp(d, i);
  for (unsigned k = 0; k < 8; k++)
  {
    unsigned h = d->hr_[k];
    unsigned char r = d->hr_[8 + k];
    segLen_[k] = h * 6u + 6u;
    ref_[k] = decodeLevel(alaw_, r);
  }
  for (unsigned i = 0; i < (unsigned)dilN_; i++)
    to_[i] = decodeLevel(alaw_, d->to_[i]);
  spIdx_ = 0;
  segCnt_ = 0;
  tpIdx_ = 0;
  toIdx_ = 0;
  seg_ = segmentOf(this);
}

/** @brief One DIL step (unused: dilStep() is inlined in states 9 and 0xa). */
short V90Phase3Modulator::generateDIL() { return dilStep(this); }

/** @brief Scrambled zero bit as a Jd-sign symbol (unused). */
short V90Phase3Modulator::generateJdNot()
{
  unsigned x = (unsigned)scr_.process(0) ^ jdBit_;
  jdBit_ = x;
  return negIfZero(x, levelA_);
}

/** @brief Scrambled Jd frame bit as a symbol (unused). */
short V90Phase3Modulator::generateJd()
{
  unsigned x = (unsigned)scr_.process(jdBits_[(cnt_ - 1u) % 72u]) ^ jdBit_;
  jdBit_ = x;
  return negIfZero(x, levelA_);
}

/** @brief Scrambled all-ones symbol of TRN1d (unused). */
short V90Phase3Modulator::generateTRN1d()
{
  int r = scr_.process(1);
  return r != 0 ? pos16(levelA_) : neg16(levelA_);
}

/** @brief Scrambled JdPhase frame bit as a symbol (unused). */
short V90Phase3Modulator::generateJdPhase()
{
  unsigned x = (unsigned)scr_.process(v92JdPhaseBits_[(cnt_ - 1u) % 72u]) ^ jdBit_;
  jdBit_ = x;
  return negIfZero(x, levelA_);
}

/** @brief Scrambled V92Jd frame bit as a symbol (unused). */
short V90Phase3Modulator::generateV92Jd()
{
  unsigned x = (unsigned)scr_.process(v92JdBits_[(cnt_ - 1u) % 72u]) ^ jdBit_;
  jdBit_ = x;
  return negIfZero(x, levelA_);
}

/**
 * @brief Produce one symbol of a V.90 session.
 * @return the 16-bit symbol of the current state (0 in the idle, error and illegal states)
 * @note  The states 0 .. 2 and 8 .. 0xf are the same as in generateV92Symbol().  Each state handler sets the `event_` marker the owner polls and moves
 *        to the next state when its symbol count is reached.
 * @warning State 3 / 7 read `jdBits_[...]` unchecked; the null test is made only at the end of TRN1d.
 */
short V90Phase3Modulator::generateV90Symbol()
{
  unsigned st = state_;
  short sym = 0;

  cnt_++;

  switch (st)
  {
  /* 0 Sd: 0x180 symbols of the period-6 pattern */
  case P3M_SD:
    event_ = 0;
    sym = sdPattern(this, (cnt_ - 1u) % 6u, false);
    if (cnt_ != 0x180)
      return sym;
    state_ = P3M_SD_NOT;
    cnt_ = 0;
    return sym;

  /* 1 SdNot: 0x30 inverted symbols, then the scrambler restarts and event 1 tells the owner TRN1d begins */
  case P3M_SD_NOT:
    sym = sdPattern(this, (cnt_ - 1u) % 6u, true);
    if (cnt_ == 0x30)
    {
      state_ = P3M_TRN1D;
      event_ = 1;
      cnt_ = 0;
      scr_.reset(0);
    }
    else
    {
      event_ = 0;
    }
    return sym;

  /* 2 TRN1d: 0x3e7c scrambled all-ones symbols; needs a Jd bit vector to continue */
  case P3M_TRN1D:
  {
    event_ = 0;
    int r = scr_.process(1);
    unsigned lvl = dsplibs_debug_level;
    sym = r == 0 ? neg16(levelA_) : pos16(levelA_);
    if (cnt_ != 0x3e7c)
      return sym;
    if (jdBits_ == 0)
    {
      state_ = P3M_ERROR;
      if (lvl > 1)
        dsplibs_debug_printf("V90Phase3Modulator: ERROR: Null JdBits @ end of TRN1d\r\n");
    }
    else
    {
      state_ = P3M_JD;
      event_ = 2;
      jdBit_ = sym > 0 ? 1u : 0u;
    }
    cnt_ = 0;
    return sym;
  }

  /* 3 Jd: scrambled frame bits, 72 per frame, differentially signed; time-out after timeBase_ + 24804 symbols */
  case P3M_JD:
  {
    event_ = 0;
    unsigned x = (unsigned)scr_.process(jdBits_[(cnt_ - 1u) % 72u]) ^ jdBit_;
    jdBit_ = x;
    sym = negIfZero(x, levelA_);
    if ((unsigned long long)cnt_ == (unsigned long long)timeBase_ + 24804ull)
    {
      state_ = P3M_STATE12;
      cnt_ = 0;
      edprintf("V90Phase3Modulator: Jd TimeOut\r\n");
    }
    return sym;
  }

  /* 7 (V.90) JdNot: run on to the end of the frame */
  case P3M_JD_NOT:
  {
    event_ = 0;
    unsigned x = (unsigned)scr_.process(jdBits_[(cnt_ - 1u) % 72u]) ^ jdBit_;
    jdBit_ = x;
    sym = negIfZero(x, levelA_);
    if (cnt_ % 72u != 0)
      return sym;
    state_ = P3M_JD_NOT_END;
    cnt_ = 0;
    return sym;
  }

  /* 8: 12 scrambled zero bits after the last frame, then the DIL (or an error without one) */
  case P3M_JD_NOT_END:
  {
    event_ = 0;
    unsigned x = (unsigned)scr_.process(0) ^ jdBit_;
    jdBit_ = x;
    sym = negIfZero(x, levelA_);
    if (cnt_ != 0xc)
      return sym;
    if (dilN_ == 0)
    {
      state_ = P3M_ERROR;
      edprintf("V90Phase3Modulator: ERROR: Null DIL @ end of JdNOT\r\n");
    }
    else
    {
      state_ = P3M_DIL;
    }
    cnt_ = 0;
    return sym;
  }

  /* 9 DIL: levels of the descriptor until the owner ends the state through exitDIL(); time-out after timeBase_ + 40000 symbols */
  case P3M_DIL:
    event_ = 0;
    sym = dilStep(this);
    if (cnt_ == timeBase_ + 40000u)
    {
      state_ = P3M_STATE14;
      cnt_ = 0;
      edprintf("V90Phase3Modulator: DIL TimeOut\r\n");
    }
    return sym;

  /* 0xa: finish the current DIL segment, then terminate phase 3 */
  case P3M_DIL_END:
    event_ = 0;
    sym = dilStep(this);
    if (segCnt_ == 0)
    {
      edprintf("V90Phase3Modulator: Phase3 Terminated @ %d\r\n", cnt_);
      state_ = P3M_DONE;
      cnt_ = 0;
      event_ = 6;
    }
    return sym;

  /* idle, error and time-out states */
  case P3M_DONE:
  case P3M_STATE12:
  case P3M_STATE14:
  case P3M_ERROR:
    event_ = 0;
    return 0;

  default:
    event_ = 0;
    if (dsplibs_debug_level > 1)
    {
      dsplibs_debug_printf("V90Phase3Modulator: Illegal state\r\n");
      return 0;
    }
    return 0;
  }
}

/**
 * @brief Produce one symbol of a V.92 session.
 * @return the 16-bit symbol of the current state (0 in the idle, error and illegal states)
 * @note  As generateV90Symbol() for states 0 .. 2 and 8 .. 0xf; states 3 .. 6 use the V92Jd / V92JdPhase bit vectors: 3 Jd (time-out -> state 0xc), 4 finishes
 *        the frame (event 3, then 5), 5 JdPhase (time-out at 24804 -> state 0xd), 6 finishes that frame.
 * @warning `v92JdBits_` is tested at the end of TRN1d, but `v92JdPhaseBits_` is never tested before states 5 / 6 read it.
 */
short V90Phase3Modulator::generateV92Symbol()
{
  unsigned st = state_;
  short sym = 0;

  cnt_++;

  switch (st)
  {
  /* 0 Sd: 0x180 symbols of the period-6 pattern */
  case P3M_SD:
    event_ = 0;
    sym = sdPattern(this, (cnt_ - 1u) % 6u, false);
    if (cnt_ != 0x180)
      return sym;
    state_ = P3M_SD_NOT;
    cnt_ = 0;
    return sym;

  /* 1 SdNot: 0x30 inverted symbols, then the scrambler restarts and event 1 tells the owner TRN1d begins */
  case P3M_SD_NOT:
    sym = sdPattern(this, (cnt_ - 1u) % 6u, true);
    if (cnt_ == 0x30)
    {
      state_ = P3M_TRN1D;
      event_ = 1;
      cnt_ = 0;
      scr_.reset(0);
    }
    else
    {
      event_ = 0;
    }
    return sym;

  /* 2 TRN1d: 0x3e7c scrambled all-ones symbols; needs a Jd bit vector to continue */
  case P3M_TRN1D:
  {
    event_ = 0;
    int r = scr_.process(1);
    unsigned lvl = dsplibs_debug_level;
    sym = r == 0 ? neg16(levelA_) : pos16(levelA_);
    if (cnt_ != 0x3e7c)
      return sym;
    if (v92JdBits_ == 0)
    {
      state_ = P3M_ERROR;
      if (lvl > 1)
        dsplibs_debug_printf("V90Phase3Modulator: ERROR: Null v92JdBits @ end of TRN1d\r\n");
    }
    else
    {
      state_ = P3M_JD;
      event_ = 2;
      jdBit_ = sym > 0 ? 1u : 0u;
    }
    cnt_ = 0;
    return sym;
  }

  /* 3 Jd: scrambled frame bits, 72 per frame, differentially signed; time-out after timeBase_ + 24804 symbols */
  case P3M_JD:
  {
    event_ = 0;
    unsigned x = (unsigned)scr_.process(v92JdBits_[(cnt_ - 1u) % 72u]) ^ jdBit_;
    jdBit_ = x;
    sym = negIfZero(x, levelA_);
    if ((unsigned long long)cnt_ == (unsigned long long)timeBase_ + 24804ull)
    {
      state_ = P3M_STATE12;
      cnt_ = 0;
      edprintf("V90Phase3Modulator: V92Jd TimeOut\r\n");
    }
    return sym;
  }
  /* 4 V92Jd continuation: run on to the end of the frame, then JdPhase with event 3 */
  case P3M_STATE4:
  {
    event_ = 0;
    unsigned x = (unsigned)scr_.process(v92JdBits_[(cnt_ - 1u) % 72u]) ^ jdBit_;
    jdBit_ = x;
    sym = negIfZero(x, levelA_);
    if (cnt_ % 72u == 0)
    {
      state_ = P3M_STATE5;
      cnt_ = 0;
      event_ = 3;
    }
    return sym;
  }

  /* 5 V92JdPhase: scrambled JdPhase frame bits; time-out after 24804 symbols */
  case P3M_STATE5:
  {
    event_ = 0;
    unsigned x = (unsigned)scr_.process(v92JdPhaseBits_[(cnt_ - 1u) % 72u]) ^ jdBit_;
    jdBit_ = x;
    sym = negIfZero(x, levelA_);
    if ((unsigned long long)cnt_ == 24804ull)
    {
      state_ = P3M_STATE13;
      cnt_ = 0;
      edprintf("V90Phase3Modulator: V92JdPhase TimeOut\r\n");
    }
    return sym;
  }

  /* 6 JdPhase continuation: run on to the end of the frame */
  case P3M_STATE6:
  {
    event_ = 0;
    unsigned x = (unsigned)scr_.process(v92JdPhaseBits_[(cnt_ - 1u) % 72u]) ^ jdBit_;
    jdBit_ = x;
    sym = negIfZero(x, levelA_);
    if (cnt_ % 72u != 0)
      return sym;
    state_ = P3M_JD_NOT_END;
    cnt_ = 0;
    return sym;
  }

  /* 8: 12 scrambled zero bits after the last frame, then the DIL (or an error without one) */
  case P3M_JD_NOT_END:
  {
    event_ = 0;
    unsigned x = (unsigned)scr_.process(0) ^ jdBit_;
    jdBit_ = x;
    sym = negIfZero(x, levelA_);
    if (cnt_ != 0xc)
      return sym;
    if (dilN_ == 0)
    {
      state_ = P3M_ERROR;
      edprintf("V90Phase3Modulator: ERROR: Null DIL @ end of JdNOT\r\n");
    }
    else
    {
      state_ = P3M_DIL;
    }
    cnt_ = 0;
    return sym;
  }

  /* 9 DIL: levels of the descriptor until the owner ends the state through exitDIL(); time-out after timeBase_ + 40000 symbols */
  case P3M_DIL:
    event_ = 0;
    sym = dilStep(this);
    if (cnt_ == timeBase_ + 40000u)
    {
      state_ = P3M_STATE14;
      cnt_ = 0;
      edprintf("V90Phase3Modulator: DIL TimeOut\r\n");
    }
    return sym;

  /* 0xa: finish the current DIL segment, then terminate phase 3 */
  case P3M_DIL_END:
    event_ = 0;
    sym = dilStep(this);
    if (segCnt_ == 0)
    {
      edprintf("V90Phase3Modulator: Phase3 Terminated @ %d\r\n", cnt_);
      state_ = P3M_DONE;
      cnt_ = 0;
      event_ = 6;
    }
    return sym;

  /* idle, error and time-out states */
  case P3M_DONE:
  case P3M_STATE12:
  case P3M_STATE13:
  case P3M_STATE14:
  case P3M_ERROR:
    event_ = 0;
    return 0;

  default:
    event_ = 0;
    if (dsplibs_debug_level > 1)
    {
      dsplibs_debug_printf("V90Phase3Modulator: Illegal state\r\n");
      return 0;
    }
    return 0;
  }
}

/** @brief Produce one symbol: generateV92Symbol() when `flag_` is nonzero, else generateV90Symbol(). */
short V90Phase3Modulator::generateSymbol()
{
  if (flag_ != 0)
    return generateV92Symbol();
  return generateV90Symbol();
}

/**
 * @brief Restart the generator and run `n` symbols without returning them.
 * @param pcm       companding law for the levels and the DIL codes
 * @param code      G.711 code of levelA_ (levelB_ uses code + 0x10; levelC_ is the smallest-magnitude code)
 * @param st        initial state
 * @param n         number of symbols to generate and discard
 * @param jd        V.90 Jd source, used when `flag_` == 0 (NULL gives a NULL bit vector)
 * @param v92jd     V.92 Jd source, used when `flag_` != 0 (NULL gives NULL bit vectors)
 * @param dil       DIL descriptor, NULL for none
 * @param timeBase  base count for the time-outs
 * @note  The V.92 vectors are left unchanged when `flag_` == 0 and the V.90 vector when `flag_` != 0.
 */
void V90Phase3Modulator::reset(PcmType pcm, unsigned char code, Phase3ModulatorState st, unsigned n, V90Jd* jd, V92Jd* v92jd, const tagV90DILdescriptor* dil,
                               unsigned timeBase)
{
  scr_.reset(0);
  alaw_ = (unsigned)pcm;
  cnt_ = 0;
  event_ = 0;
  timeBase_ = timeBase;
  state_ = (unsigned)st;

  /* levels from the G.711 codes */
  levelA_ = decodeLevel(alaw_, code);
  code = (unsigned char)(code + 0x10);
  levelB_ = decodeLevel(alaw_, code);
  levelC_ = alaw_ != 0 ? (unsigned short)alaw2linear(0xd5) : (unsigned short)ulaw2linear(0xff);
  jdBit_ = 0;

  /* frame bit sources of the session type */
  if (flag_ == 0)
  {
    unsigned char* bits = 0;
    if (jd != 0)
      bits = jd->getBitVector();
    jdBits_ = bits;
  }
  else
  {
    if (v92jd != 0)
    {
      v92JdBits_ = v92jd->getJdBitVector();
      v92JdPhaseBits_ = v92jd->getJdPhaseBitVector();
    }
    else
    {
      v92JdBits_ = 0;
      v92JdPhaseBits_ = 0;
    }
  }
  resetDILGenerator(dil);

  /* run the first n symbols */
  for (unsigned i = n; i != 0; i--)
  {
    if (flag_ != 0)
      generateV92Symbol();
    else
      generateV90Symbol();
  }
}

/* ------------------------------------------------------------------------------------------------ V92Phase3Modulator */

/**
 * @brief Construct the generator and restart it in state 0 (Ru).
 * @param p  V.92 parameter block, stored
 */
V92Phase3Modulator::V92Phase3Modulator(V92Parameters* p) : scr_(5, 0x17, 0x63) { v92CtorBody(this, p); }

V92Phase3Modulator::~V92Phase3Modulator() {}

/**
 * @brief End-of-state decision for the Ja state.
 * @note  In state 4 with `cnt_` != 0: state 6 (silence) and `cnt_` = 0 on a 12-symbol boundary, else state 5 to finish the group.
 */
void V92Phase3Modulator::exitJa()
{
  if (state_ != V92P3_JA)
    return;
  unsigned c = cnt_;
  if (c == 0)
    return;
  if (c % 12u == 0)
  {
    state_ = V92P3_SILENCE;
    cnt_ = 0;
  }
  else
  {
    state_ = V92P3_JA_SECOND;
  }
}

/** @brief End of silence: in state 6 with `cnt_` != 0, go to state 7 (Su) and restart the count. */
void V92Phase3Modulator::exitSilence()
{
  if (state_ != V92P3_SILENCE)
    return;
  if (cnt_ == 0)
    return;
  state_ = V92P3_SU;
  cnt_ = 0;
}

/**
 * @brief End-of-state decision for SuSecond.
 * @note  In state 9 with `cnt_` != 0: state 0xb and `cnt_` = 0 on a 12-symbol boundary, else state 0xa to finish the group.
 */
void V92Phase3Modulator::exitSuSecond()
{
  if (state_ != V92P3_SU_SECOND)
    return;
  unsigned c = cnt_;
  if (c == 0)
    return;
  if (c % 12u == 0)
  {
    state_ = V92P3_STATE11;
    cnt_ = 0;
  }
  else
  {
    state_ = V92P3_STATE10;
  }
}

/** @brief End of the second TRN1u part: in state 0xc with `cnt_` != 0, go to state 0xd. */
void V92Phase3Modulator::exitTRN1u()
{
  if (state_ != V92P3_TRN1U_SECOND)
    return;
  if (cnt_ == 0)
    return;
  state_ = V92P3_STATE13;
}

/** @brief Ru pattern symbol for the current `cnt_` (unused). */
short V92Phase3Modulator::generateRu() { return ruPattern(this, (cnt_ - 1u) % 6u, false); }

/** @brief RuNot pattern symbol (unused). */
short V92Phase3Modulator::generateRuNot() { return ruPattern(this, (cnt_ - 1u) % 6u, true); }

/** @brief Su pattern symbol (unused). */
short V92Phase3Modulator::genereteSu() { return suPattern(this, (cnt_ - 1u) % 6u, false); }

/** @brief SuNot pattern symbol (unused). */
short V92Phase3Modulator::genereteSuNot() { return suPattern(this, (cnt_ - 1u) % 6u, true); }

/**
 * @brief Ja symbol (unused): scrambled 1 for the first 0x18 symbols, then the Ja bits.
 * @warning Divides by `jaLen_`, which is 0 when no Ja block was given.
 */
short V92Phase3Modulator::generateJa()
{
  unsigned char b = 1;
  if (cnt_ > 0x18)
    b = jaBits_[(cnt_ - 0x19u) % jaLen_];
  unsigned x = (unsigned)scr_.process(b) ^ jaBit_;
  jaBit_ = x;
  return negIfNonzero(x, levelA_);
}

/** @brief Scrambled all-ones symbol of TRN1u (unused). */
short V92Phase3Modulator::generateTRN1u()
{
  int r = scr_.process(1);
  return r != 0 ? neg16(levelA_) : pos16(levelA_);
}

/**
 * @brief Produce one symbol of the V.92 phase-3 sequence and advance its state.
 * @return the 16-bit symbol of the current state (0 for silence, idle and illegal states)
 * @note  Ru (0x180 symbols) -> RuNot (0x18, event 2, scrambler reset) -> TRN1u (`trn1uLen_` symbols, event 3) -> Ja (the owner calls exitJa()) -> silence
 *        -> Su (0x90) -> SuNot (0x18, event 5) -> SuSecond (the owner calls exitSuSecond()) -> 0xa / 0xb (0x18 symbols, event 7) -> second TRN1u part (the
 *        owner calls exitTRN1u()) -> state 0xd (until at least 0x7f8 symbols and a multiple of 12, event 8) -> idle.
 * @warning The Ja states divide by `jaLen_` once `cnt_` exceeds 0x18; `jaLen_` is 0 without a Ja block.
 */
short V92Phase3Modulator::generateSymbol()
{
  unsigned st = state_;
  short sym = 0;

  cnt_++;

  switch (st)
  {
  /* 0 Ru: 0x180 symbols of the +-levelA_ pattern */
  case V92P3_RU:
    event_ = 0;
    sym = ruPattern(this, (cnt_ - 1u) % 6u, false);
    if (cnt_ != 0x180)
      return sym;
    state_ = V92P3_RU_NOT;
    cnt_ = 0;
    return sym;

  /* 1 RuNot: 0x18 inverted symbols, then the scrambler restarts and event 2 is raised */
  case V92P3_RU_NOT:
    event_ = 0;
    sym = ruPattern(this, (cnt_ - 1u) % 6u, true);
    if (cnt_ == 0x18)
    {
      state_ = V92P3_TRN1U;
      cnt_ = 0;
      scr_.reset(0);
      event_ = 2;
    }
    return sym;

  /* 3 TRN1u: trn1uLen_ scrambled all-ones symbols, then Ja with event 3 */
  case V92P3_TRN1U:
  {
    event_ = 0;
    int r = scr_.process(1);
    sym = r == 0 ? pos16(levelA_) : neg16(levelA_);
    if (cnt_ != trn1uLen_)
      return sym;
    state_ = V92P3_JA;
    cnt_ = 0;
    event_ = 3;
    jaBit_ = sym <= 0 ? 1u : 0u;
    return sym;
  }

  /* 4 Ja: 0x18 scrambled ones, then the Ja bits cyclically; the owner ends the state through exitJa() */
  case V92P3_JA:
  {
    event_ = 0;
    unsigned char b = 1;
    if (cnt_ > 0x18)
      b = jaBits_[(cnt_ - 0x19u) % jaLen_];
    unsigned x = (unsigned)scr_.process(b) ^ jaBit_;
    jaBit_ = x;
    return negIfNonzero(x, levelA_);
  }

  /* 5 Ja continuation: run on to the next multiple of 12 symbols, then silence */
  case V92P3_JA_SECOND:
  {
    event_ = 0;
    unsigned char b = 1;
    if (cnt_ > 0x18)
      b = jaBits_[(cnt_ - 0x19u) % jaLen_];
    unsigned x = (unsigned)scr_.process(b) ^ jaBit_;
    jaBit_ = x;
    sym = negIfNonzero(x, levelA_);
    if (cnt_ % 12u != 0)
      return sym;
    state_ = V92P3_SILENCE;
    cnt_ = 0;
    return sym;
  }

  /* 6 silence, 14 / 15 idle */
  case V92P3_SILENCE:
  case V92P3_STATE14:
  case V92P3_STATE15:
    event_ = 0;
    return 0;

  /* 7 Su: 0x90 symbols of the levelB_ / 0 pattern */
  case V92P3_SU:
    event_ = 0;
    sym = suPattern(this, (cnt_ - 1u) % 6u, false);
    if (cnt_ != 0x90)
      return sym;
    state_ = V92P3_SU_NOT;
    cnt_ = 0;
    return sym;

  /* 8 SuNot: 0x18 inverted symbols, then SuSecond with event 5 */
  case V92P3_SU_NOT:
    sym = suPattern(this, (cnt_ - 1u) % 6u, true);
    if (cnt_ == 0x18)
    {
      state_ = V92P3_SU_SECOND;
      cnt_ = 0;
      event_ = 5;
    }
    else
    {
      event_ = 0;
    }
    return sym;

  /* 9 SuSecond: Su pattern until the owner calls exitSuSecond() */
  case V92P3_SU_SECOND:
    event_ = 0;
    return suPattern(this, (cnt_ - 1u) % 6u, false);
  /* 0xa: Su pattern to the next multiple of 12 symbols */
  case V92P3_STATE10:
    event_ = 0;
    sym = suPattern(this, (cnt_ - 1u) % 6u, false);
    if (cnt_ % 12u != 0)
      return sym;
    state_ = V92P3_STATE11;
    cnt_ = 0;
    return sym;

  /* 0xb: 0x18 inverted Su symbols, then the second TRN1u part with the scrambler restarted (event 7) */
  case V92P3_STATE11:
    event_ = 0;
    sym = suPattern(this, (cnt_ - 1u) % 6u, true);
    if (cnt_ == 0x18)
    {
      state_ = V92P3_TRN1U_SECOND;
      cnt_ = 0;
      scr_.reset(0);
      event_ = 7;
    }
    return sym;

  /* 0xc: scrambled all-ones until the owner calls exitTRN1u() */
  case V92P3_TRN1U_SECOND:
  {
    event_ = 0;
    int r = scr_.process(1);
    return r == 0 ? pos16(levelA_) : neg16(levelA_);
  }

  /* 0xd: scrambled all-ones, ended (event 8) at the first multiple of 12 symbols from 0x7f8 on */
  case V92P3_STATE13:
  {
    event_ = 0;
    int r = scr_.process(1);
    sym = r == 0 ? pos16(levelA_) : neg16(levelA_);
    if (cnt_ < 0x7f8u)
      return sym;
    if (cnt_ % 12u != 0)
      return sym;
    state_ = V92P3_STATE14;
    cnt_ = 0;
    event_ = 8;
    jaBit_ = sym <= 0 ? 1u : 0u;
    return sym;
  }

  default:
    event_ = 0;
    if (dsplibs_debug_level > 1)
    {
      dsplibs_debug_printf("V92Phase3Modulator: Illegal state\r\n");
      return 0;
    }
    return 0;
  }
}

/**
 * @brief Restart the generator and run `n` symbols without returning them.
 * @param unused  ignored; levelA_ is always 0xfa0
 * @param st      initial state
 * @param n       number of symbols to generate and discard
 * @param ja      Ja bit block, NULL for none
 * @param dil     DIL descriptor; with no Ja block and a DIL a diagnostic is printed (debug level above 1)
 * @param a6      stored in `arg6_`
 * @note  Sets the TRN1u length from the parameter block's echo-canceller update durations: 12 * ((fast + slow + 12) / 12) symbols when that exceeds 0x1fdf,
 *        else 0x1fe0, and prints it.  levelB_ = levelA_ * 1.2247448713915890 rounded through a float slot.
 */
void V92Phase3Modulator::reset(short unused, V92Phase3ModulatorState st, unsigned n, V92Ja* ja, const tagV90DILdescriptor* dil, unsigned a6)
{
  (void)unused;
  scr_.reset(0);
  cnt_ = 0;
  arg6_ = a6;
  state_ = (unsigned)st;

  /* TRN1u length from the echo update durations */
  {
    int sum = (int)((unsigned)params_->v92EchoFastUpdateDuration + (unsigned)params_->v92EchoSlowUpdateDuration);
    int t = (int)((unsigned)sum + 12u);
    int q = t / 12;
    int len = q * 12;
    if ((unsigned)len > 0x1fdfu)
      trn1uLen_ = (unsigned)len;
    else
      trn1uLen_ = 0x1fe0;
    edprintf("V92Phase3Modulator: TRN1u state length set to %d\r\n", (int)trn1uLen_);
  }
  event_ = 0;
  levelA_ = 0xfa0;

  if (ja != 0)
  {
    jaBits_ = ja->bits_;
    jaLen_ = ja->len_;
  }
  else
  {
    if (dil != 0 && dsplibs_debug_level > 1)
      dsplibs_debug_printf("BUG BUG BUG BUG BUG - V92Phase3Modulator: no Ja Object, DIL descriptor available !\r\n");
    jaLen_ = 0;
    jaBits_ = 0;
  }

  /* levelB_ = levelA_ * sqrt(1.5), rounded through a float slot */
  {
    volatile float slot;
    LD prod = (LD)(int16_t)levelA_ * (LD)1.2247448713915890491;
    float f = storeF((float*)&slot, prod);
    LD r = (LD)f + (LD)0.5f;
    levelB_ = (unsigned short)trunc16(r);
  }

  for (unsigned i = n; i != 0; i--)
    generateSymbol();
}
