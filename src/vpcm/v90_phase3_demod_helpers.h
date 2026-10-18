/**
 * @file v90_phase3_demod_helpers.h
 * @brief Inline helpers shared by V90Phase3Demodulator::getV90Decision() and ::getV92Decision().
 *
 * The two decision functions are the same state machine on two Jd frame formats and use the same small helpers: the float to 16-bit truncation of the
 * input, the frame position step, the accessors of the digital impairment detector's level tables, the G.711 code of a level, the bit decision of the
 * TRN1d / Jd states and the time-out checks.  The helpers are `static inline`; each decision file includes this header after its own typedefs.
 *
 * Level tables.  The detector (V90AutoDigitalImpDetector) keeps linear level tables indexed by ucode: linearMapping_ (the learned levels) and
 * linearMappingAlt_ (the alternate set for a phase with robbed-bit signalling), each 6 phases x 128 ucodes of 16 bits, and prevSessionLinearMapping_
 * (128 ucodes, the levels of the previous session, always read at phase 0).  They are addressed here by byte offset from the start of the detector and a
 * 16-bit phase * 128 + ucode index.
 */
#ifndef V90_PHASE3_DEMOD_HELPERS_H
#define V90_PHASE3_DEMOD_HELPERS_H

#include <stddef.h>
#include <stdint.h>
#include "v90_phase3_demod.h"
#include "v34/v34_api.h"

typedef P34_V90Phase3Demodulator D;
typedef P18_V90AutoDigitalImpDetector Adid;

/** @brief Float to 16-bit integer, truncating toward zero; NaN and values outside the 16-bit range give 0x8000. */
static inline short trunc16f(float x)
{
  if (!((long double)x > -32769.0L && (long double)x < 32768.0L))
    return (short)0x8000;
  return (short)(int)x;
}

/** @brief Advance the position in the 6-symbol PCM frame: phase_ = (phase_ + 1 == 6) ? 0 : phase_ + 1. */
static inline void advancePhase(D* t)
{
  unsigned p = t->phase_ + 1;
  t->phase_ = (p == 6) ? 0 : p;
}

#include "core/dsp_floatbits.h" /* copyWordF: copy a float member as its raw word */

/** @brief |m| of a 16-bit symbol as an int (|-32768| = 32768). */
static inline int absSym(short m)
{
  int v = (int)m;
  return v < 0 ? -v : v;
}

/** @brief Sign of a sample: -1 for negative, +1 otherwise (zero gives +1). */
static inline int signOf(short s) { return ((int)s >> 31) | 1; }

/**
 * @brief The G.711 code ("ucode") of a positive linear level.
 * A-law (pcm != 0): the low byte of linear2alaw() ^ 0xd5; mu-law: 0xff minus the low byte of linear2ulaw().
 */
static inline unsigned char ucodeOf(unsigned pcm, int absm)
{
  if (pcm != 0)
    return (unsigned char)((unsigned)linear2alaw(absm) ^ 0xd5u);
  return (unsigned char)(0xffu - ((unsigned)linear2ulaw(absm) & 0xffu));
}

/**
 * @brief Element `idx` of the 16-bit table that starts at byte offset `off` of the detector object (signed / unsigned read).
 * @note  The index may run past the declared array into the neighbouring table; the tables are adjacent, which the callers rely on.
 */
static inline short adidWord(const Adid* a, unsigned off, unsigned idx) { return *(const short*)((const char*)a + off + 2u * idx); }
static inline unsigned short adidUWord(const Adid* a, unsigned off, unsigned idx) { return *(const unsigned short*)((const char*)a + off + 2u * idx); }
#define BASE_OFF (offsetof(Adid, linearMapping_))
#define ALT_OFF (offsetof(Adid, linearMappingAlt_))
#define PREV_OFF (offsetof(Adid, prevSessionLinearMapping_))

/** @brief Level table index (phase << 7) + ucode; the phase is taken as an unsigned 16-bit value. */
static inline unsigned tblIdx(unsigned phase, unsigned ucode) { return (((unsigned)(unsigned short)phase) << 7) + ucode; }

/** @brief The altRbsPhase_ / suspectedPhase_ entry of a frame position (phase_ must be at most 5 to stay inside the arrays). */
static inline short altPhaseAt(const Adid* a, unsigned phase) { return *(const short*)((const char*)a + offsetof(Adid, altRbsPhase_) + 2u * phase); }
static inline unsigned char suspectedAt(const Adid* a, unsigned phase) { return *((const unsigned char*)a + offsetof(Adid, suspectedPhase_) + phase); }

/** @brief 64-bit equality of a symbol count and a (possibly sum) constant: cnt == v with both taken as unsigned 64-bit values. */
static inline bool cntIs(unsigned cnt, unsigned long long v) { return (unsigned long long)cnt == v; }

/**
 * @brief The bit decision of the TRN1d DD, reference study, Jd wait and Jd demodulation states.
 * @param t   the demodulator
 * @param x   input sample
 * @param b2  receives the descrambled bit (the bit x > 0 passed through the differential decoder and the descrambler)
 * @return the decision symbol: the level at (phase_, code_) of the learned table (the alternate table for a robbed-bit phase once the study is done),
 *         negated when x is not positive
 */
static inline short decideBit(D* t, float x, int* b2)
{
  Adid* a = t->adid_;
  unsigned off = BASE_OFF;
  if (a->studyDone_ != 0)
  {
    if ((short)a->isAltRbs((short)t->phase_, (short)t->code_, x) != 0)
      off = ALT_OFF;
  }
  short v = adidWord(a, off, tblIdx(t->phase_, t->code_));
  int bit;
  short r;
  if (x > 0.0f)
  {
    bit = 1;
    r = v;
  }
  else
  {
    bit = 0;
    r = (short)(-(int)v);
  }
  int d = t->sdd_.process(bit);
  *b2 = t->descr_.process(d);
  return r;
}

/** @brief The TRN1d time-out, shared by the TRN1d DD, reference study and Jd wait states: after 36000 symbols P3D_TRN1D_TIMEOUT, event 0x15 (error). */
static inline void trn1dTimeout(D* t)
{
  if (cntIs(t->cnt_, 36000ULL))
  {
    t->state_ = P3D_TRN1D_TIMEOUT;
    t->cnt_ = 0;
    t->event_ = 0x15;
    edprintf("V90Phase3Demodulator: TRN1dDemod TimeOut\r\n");
  }
}

/** @brief Common tail of the DIL states: when the modulator reports the end of its pattern (event_ == 6), terminate phase 3 (P3D_TERMINATED, event 0x14). */
static inline void termCheck(D* t)
{
  if (t->mod_.event_ == 6)
  {
    edprintf("V90Phase3Demodulator: Phase3 Terminated @ %d\r\n", t->cnt_);
    t->state_ = P3D_TERMINATED;
    t->cnt_ = 0;
    t->event_ = 0x14;
  }
}

/** @brief The 40000-symbol time-out of the DIL states: state P3D_DIL_TIMEOUT, event PHASE3_ERROR (0x15), message `msg` (printed as the format). */
static inline void dilTimeout(D* t, const char* msg)
{
  if (cntIs(t->cnt_, 40000ULL))
  {
    t->state_ = P3D_DIL_TIMEOUT;
    t->cnt_ = 0;
    t->event_ = 0x15;
    edprintf(msg);
  }
}

/** @brief sign(s) * table[phase * 128 + ucode] as a 16-bit value, with an unsigned table entry. */
static inline short signedLevel(const Adid* a, unsigned off, short s, unsigned phaseWord, unsigned ucode)
{
  return (short)((int)adidUWord(a, off, tblIdx(phaseWord, ucode)) * signOf(s));
}
/** @brief The same with a signed table entry. */
static inline short signedLevelS(const Adid* a, unsigned off, short s, unsigned phaseWord, unsigned ucode)
{
  return (short)((int)adidWord(a, off, tblIdx(phaseWord, ucode)) * signOf(s));
}

#endif /* V90_PHASE3_DEMOD_HELPERS_H */
