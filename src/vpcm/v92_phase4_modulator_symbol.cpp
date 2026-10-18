/**
 * @file v92_phase4_modulator_symbol.cpp
 * @brief V92Phase4Modulator::generateSymbol(): the symbol generator of the V.92 phase 4 state machine.
 *
 * generateSymbol() is called by V92Modulator::progress() and V92Phase4Modulator::reset().  It dispatches on state_ (0 .. 0x1d, see the state table in
 * v92_phase4_modulator.h) through a switch.  Callees: Scrambler<uchar, uchar> (process, processAllOnes, processAllZeros, reset), V92BitsToSymbol
 * (nofBitsForNextTime, process, reset, setSymbolsBlockSize), V92Mapper::process(), V92CP (setSUV, infoToBits, getBitVector),
 * V92Phase4Modulator::generateSUVu() / generateCPu() (v92_phase4_modulator.cpp), edprintf() and dsplibs_debug_printf().
 *
 * Behaviour worth knowing:
 *  - cnt_ is incremented and event_ cleared BEFORE the state dispatch (also for an illegal state); the order of the stores to state_ / cnt_ relative
 *    to each print and to each callee is significant: callees read the members stored before them.
 *  - The symbols are 16-bit values; a negated level is (int16)(-level); the pattern index is (cnt_ - 1) % 6 (unsigned); inv_ is a 32-bit word that is
 *    XORed in 32 bits with a scrambler output byte or the byte bits_[mult_ - 1] (loaded AFTER the scrambler call), and the byte write-back stores the
 *    low byte of the new inv_.
 *  - The frame length cpFrame_ is cpLen_ / mult_ (this->mult_ re-read after getBitVector() in the TRN2u / FPE / state 13 restarts, the CP's own
 *    8-bit multiplier in the SUVu -> CPu / repeated-CPu / FinalSUVu / CPu-end restarts); a zero divisor (cpLen_, cpFrame_, mult_) is a division by zero.
 *  - The debug level is tested afresh at every print; the NULL tests of cp_ (states 4 / 23 / 24) and mp_ (states 15 / 17 / 27) exist, the other cp_
 *    dereferences are not tested.
 *  - States 18 / 25 call b2s_->process(n&, &out) WITHOUT asking nofBitsForNextTime() first and read `n` as written by it.
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

#include "v92_phase4_diff_out.h"

/* States 0 / 1: the CP bits (after the first 0x18 symbols) scrambled into the differential stream, or just toggled. */
static inline short ruOut(V92Phase4Modulator* t, unsigned cnt)
{
  if (cnt <= 0x18)
    return diffOut(t, t->inv_ ^ 1);
  unsigned char bit = t->cpBits_[(cnt - 0x19) % t->cpLen_];
  unsigned char r = t->scr_.process(bit);
  return diffOut(t, (unsigned)r ^ t->inv_);
}

/* The six-symbol pattern index: unsigned (cnt - 1) % 6. */
static inline unsigned pat6(unsigned cnt) { return (cnt - 1) % 6; }

#include "v92_phase4_map_block.h"
static inline short stepOnesMap(V92Phase4Modulator* t)
{
  t->scr_.processAllOnes(t->bits_, t->mult_);
  return mapBlock(t);
}
static inline short stepZerosMap(V92Phase4Modulator* t)
{
  t->scr_.processAllZeros(t->bits_, t->mult_);
  return mapBlock(t);
}

#include "v92_phase4_steps.h"
/* The draining step of states 0x12 / 0x19 (no bit count asked first): returns the symbol, `n` = the bit count the queue wants next. */
static inline short stepDrain(V92Phase4Modulator* t, unsigned* n)
{
  short out = 0;
  t->b2s_->process(*n, &out);
  return out;
}

/* A CP restart: infoToBits, the new bit vector, the frame length cpLen_ / m (m = this->mult_ re-read, or the CP's own 8-bit multiplier). */
static inline void cpStartThis(V92Phase4Modulator* t)
{
  t->cp_->infoToBits();
  t->cpBits_ = t->cp_->getBitVector(t->cpLen_);
  t->cpFrame_ = t->cpLen_ / t->mult_;
}
static inline void cpStartCp(V92Phase4Modulator* t)
{
  t->cp_->infoToBits();
  t->cpBits_ = t->cp_->getBitVector(t->cpLen_);
  t->cpFrame_ = t->cpLen_ / t->cp_->mult_;
}

/* The "on TRN2uModulationExit / FPE: enter SUVu" tails: the CP becomes a SUV of value rrnSilence_. */
static inline void startSuvFromW2c(V92Phase4Modulator* t)
{
  t->cp_->type_ = 1;
  t->cp_->setSUV(t->rrnSilence_);
  cpStartThis(t);
}

/* The CP-less end of a TRN2u state: error state, optional debug message. */
static inline void nullCp(V92Phase4Modulator* t, const char* msg)
{
  bool d = dbgOn();
  t->state_ = V92P4_ERROR;
  t->cnt_ = 0;
  if (d)
    dsplibs_debug_printf(msg);
}

/* "enter B1u": give the queue the mapping parameters, block size one symbol. */
static inline void setQueueMapping(V92Phase4Modulator* t)
{
  if (t->mp_ == 0)
  {
    if (dbgOn())
      dsplibs_debug_printf("V92Phase4Modulator: ERROR: Null mappingParams @ setMappingParams\r\n");
  }
  else
  {
    t->b2s_->reset(t->mp_);
    t->b2s_->setSymbolsBlockSize(1);
  }
}

/* "enter E2u": the end of a SUVu / CPu frame run. */
static inline void enterE2u(V92Phase4Modulator* t, unsigned cnt)
{
  edprintf("V92Phase4Modulator: enter E2u @ %d\r\n", cnt);
  t->state_ = V92P4_E2U;
  unsigned f = t->extendEu_;
  t->cnt_ = 0;
  t->w1b8_ = f == 0 ? 0xc : 0xd;
}

/* The SUV restart of the two RRN second-part ends (states 23 / 24 differ in what they store first). */
static inline void rrnSuvTail(V92Phase4Modulator* t)
{
  t->state_ = V92P4_SUVU;
  t->cnt_ = 0;
  t->w08_ = 0;
}

/* ------------------------------------------------------------------------------------------------ generateSymbol */

/**
 * @brief Produce the next symbol and advance the state machine.
 * @return the 16-bit symbol; 0 in the error state and in illegal states
 * @note  Sets event_ = 9 when phase 4 terminates (B1u state after 0x240 symbols).
 * @warning cp_ is dereferenced without a NULL test in the states not listed in the file header; a zero cpLen_, cpFrame_ or mult_ divides by zero.
 */
short V92Phase4Modulator::generateSymbol()
{
  unsigned st = state_;
  short out = 0;

  /* count the symbol, then run the state */
  cnt_++;
  event_ = 0;
  switch (st)
  {
  case V92P4_RU: /* Ru */
    return ruOut(this, cnt_);
  case V92P4_RU_END:
  { /* Ru until the end of the frame */
    unsigned c = cnt_;
    out = ruOut(this, c);
    if ((c - 0x18) % cpLen_ == 0)
    {
      edprintf("V92Phase4Modulator: enter E1u @ %d\r\n", c);
      state_ = V92P4_E1U;
      cnt_ = 0;
      w1b8_ = 0xc;
    }
    return out;
  }
  case V92P4_E1U:
  { /* E1u: scrambled zeros */
    unsigned char r = scr_.process(0);
    out = diffOut(this, (unsigned)r ^ inv_);
    if (cnt_ == w1b8_)
    {
      state_ = V92P4_TRN2U;
      cnt_ = 0;
      scr_.reset(0);
    }
    return out;
  }
  case V92P4_TRN2U: /* TRN2uModulation */
    return stepOnesMap(this);
  case V92P4_TRN2U_A:
  { /* TRN2uModulation to a multiple of 12 above 0x3137 */
    out = stepOnesMap(this);
    unsigned c = cnt_;
    if (c <= 0x3137)
      return out;
    if (c % 12 != 0)
      return out;
    if (cp_ != 0)
    {
      edprintf("V92Phase4Modulator: on TRN2uModulationExit enter SUV @ %d\r\n", c);
      unsigned char m = mult_;
      state_ = V92P4_SUVU;
      cnt_ = 0;
      w08_ = 0;
      cp_->mult_ = m;
      startSuvFromW2c(this);
    }
    else
    {
      nullCp(this, "V92Phase4Modulator: ERROR: Null CP @ end of TRN2d\r\n");
    }
    return out;
  }
  case V92P4_SUVU:
  { /* SUVu */
    out = generateSUVu();
    if (cpFlag_ != 0)
      cpCnt_++;

    /* at a frame boundary restart the SUV, and switch to repeated CPu when the count passes the limit */
    unsigned c = cnt_;
    if (c % cpFrame_ != 0)
      return out;
    if (c == 0)
      return out;
    cpStartThis(this);
    if (cpCnt_ <= limit_ + 0x320u)
      return out;

    cpFlag_ = 0;
    bool d = dbgOn();
    cpCnt_ = 0;
    if (d)
      dsplibs_debug_printf("V92Phase4Modulator: enter repeatedCPu @ %d\r\n", cnt_);
    state_ = V92P4_REPEATED_CPU;
    cp_->type_ = 0;
    cpStartCp(this);
    cnt_ = 0;
    return out;
  }
  case V92P4_SUVU_TO_CPU:
  { /* SUVu -> CPu */
    out = generateSUVu();
    unsigned c = cnt_;
    if (c % cpFrame_ != 0)
      return out;
    if (dbgOn())
      dsplibs_debug_printf("V92Phase4Modulator: on SUVuToCPuModulation enter CPu @ %d\r\n", c);
    state_ = V92P4_CPU_END;
    cnt_ = 0;
    cp_->type_ = 0;
    cp_->infoToBits();
    cpBits_ = cp_->getBitVector(cpLen_);
    {
      unsigned m = cp_->mult_;
      w1c4_ = 1;
      cpFrame_ = cpLen_ / m;
    }
    return out;
  }
  case V92P4_FINAL_SUVU_ENTER:
  { /* SUVu -> FinalSUVu */
    out = generateSUVu();
    unsigned c = cnt_;
    if (c % cpFrame_ != 0)
      return out;
    edprintf("V92Phase4Modulator: enter FinalSUVu @ %d\r\n", c);
    state_ = V92P4_SUVU_10;
    cnt_ = 0;
    cp_->type_ = 1;
    cpStartCp(this);
    return out;
  }
  case V92P4_SUVU_9:
  case V92P4_CPU_11:
  { /* SUVu / CPu -> E2u at a frame boundary */
    out = st == 9 ? generateSUVu() : generateCPu();
    unsigned c = cnt_;
    if (c % cpFrame_ != 0)
      return out;
    enterE2u(this, c);
    return out;
  }
  case V92P4_SUVU_10:
  { /* SUVu -> E2u at the frame length */
    out = generateSUVu();
    unsigned c = cnt_;
    if (c != cpFrame_)
      return out;
    enterE2u(this, c);
    return out;
  }
  case V92P4_CPU_END:
  { /* CPu -> terminated -> SUVu */
    out = generateCPu();
    unsigned c = cnt_;
    if (c != cpFrame_)
      return out;
    edprintf("V92Phase4Modulator: CPu Terminated @ %d\r\n", c);
    cpCnt_ = 0;
    cpFlag_ = 1;
    state_ = V92P4_SUVU;
    cnt_ = 0;
    cp_->type_ = 1;
    cpStartCp(this);
    return out;
  }
  case V92P4_REPEATED_CPU:
  { /* repeated CPu */
    out = generateCPu();
    unsigned c = cnt_;
    if (c % cpFrame_ != 0)
      return out;
    if (c == 0)
      return out;
    cpStartThis(this);
    return out;
  }
  case V92P4_E2U:
  { /* E2u */
    if (w3c_ == 0)
      out = stepZerosMap(this);
    else
      out = stepZeros(this);
    unsigned c = cnt_;
    if (c != w1b8_)
      return out;

    /* end of E2u: RRN second TRN2u part, or the data phase */
    if (w28_ != 0 && w30_ != 0 && w34_ == 0)
    {
      edprintf("V92Phase4Modulator: enter TRN2u Second at RRN @ %d\r\n", c);
      cnt_ = 0;
      state_ = V92P4_TRN2U_SECOND;
      w24_ = w38_ == 0 ? 0x1f44 : 0xfa0;
      return out;
    }

    if (mp_ == 0)
    {
      bool d = dbgOn();
      state_ = V92P4_ERROR;
      cnt_ = 0;
      if (d)
        dsplibs_debug_printf("V92Phase4Modulator: ERROR: Null dataPhaseMappingParams @ end of Ed\r\n");
      return out;
    }

    if (w3c_ == 0)
    {
      edprintf("V92Phase4Modulator: enter B1u @ %d\r\n", c);
      state_ = V92P4_B1U;
      setQueueMapping(this);
      cnt_ = 0;
      scr_.reset(0);
    }
    else
    {
      edprintf("V92Phase4Modulator: enter FB1u @ %d\r\n", c);
      state_ = V92P4_FB1U;
      cnt_ = 0;
      scr_.reset(0);
    }
    return out;
  }
  case V92P4_B1U: /* B1u */
    out = stepOnes(this);
    if (cnt_ == 0x240)
    {
      edprintf("V92Phase4Modulator: Phase4 Terminated @ %d\r\n", 0x240);
      state_ = V92P4_TERMINATED;
      cnt_ = 0;
      event_ = 9;
    }
    return out;
  case V92P4_FB1U: /* FB1u */
    out = stepOnes(this);
    if (cnt_ == 0x240)
    {
      edprintf("V92Phase4Modulator: enter B1u @ %d\r\n", 0x240);
      setQueueMapping(this);
      state_ = V92P4_B1U;
      cnt_ = 0;
    }
    return out;
  case V92P4_WAIT_RU:
  { /* waits for the queue to ask for bits */
    unsigned n = 0;
    out = stepDrain(this, &n);
    if (n != 0)
    {
      edprintf("V92Phase4Modulator: enter Ru @ %d\r\n", cnt_);
      state_ = V92P4_RU_PAT;
      cnt_ = 0;
    }
    return out;
  }
  case V92P4_RU_PAT:
  { /* Ru pattern */
    unsigned c = cnt_;
    unsigned r = pat6(c);
    out = r <= 2 ? (short)level_ : neg16((short)level_);
    if (c == 0x180)
    {
      state_ = V92P4_RU_NOT;
      cnt_ = 0;
    }
    return out;
  }
  case V92P4_RU_NOT:
  { /* RuNot */
    unsigned c = cnt_;
    unsigned r = pat6(c);
    out = r <= 2 ? neg16((short)level_) : (short)level_;
    if (c == 0x18)
    {
      if (dbgOn())
        dsplibs_debug_printf("V92Phase4Modulator: RRN: enter TRN2uModulation @ %d\r\n", 0x18);
      state_ = V92P4_TRN2U;
      cnt_ = 0;
      scr_.reset(0);
      inv_ = 0;
    }
    return out;
  }
  case V92P4_TRN2U_SECOND:
  { /* the TRN2u second part at RRN */
    out = stepOnesMap(this);
    unsigned c = cnt_;
    if (c < w24_)
      return out;
    if (c % 12 != 0)
      return out;
    if (cp_ == 0)
    {
      nullCp(this, "V92Phase4Modulator: ERROR: Null CP @ RRN @ end of TRN2u\r\n");
      return out;
    }

    /* end of the second part: clear the CP flags and restart the CP as a SUV */
    edprintf("V92Phase4Modulator: RRN: Terminate TRN2uSecond @ %d\r\n", c);
    w34_ = 1;
    w20_ = 0;
    cpFlag_ = 0;
    cpCnt_ = 0;
    w1c4_ = 0;
    w1c0_ = 0;
    cp_->flag_ = 0;
    cp_->cptSent_ = 0;

    rrnSuvTail(this);
    cp_->type_ = 1;
    cp_->suvVal_ = 0;
    cpStartThis(this);
    return out;
  }
  case V92P4_TRN2U_SECOND_A:
  { /* the same up to 0x95f symbols */
    out = stepOnesMap(this);
    unsigned c = cnt_;
    if (c <= 0x95f)
      return out;
    if (c % 12 != 0)
      return out;
    if (cp_ == 0)
    {
      nullCp(this, "V92Phase4Modulator: ERROR: Null CP @ RRN @ end of TRN2u\r\n");
      return out;
    }

    /* leave the second part: clear the CP flags and restart the CP as a SUV */
    edprintf("V92Phase4Modulator: on TRN2uRrnSecondExit enter SUV @ %d\r\n", c);
    w34_ = 1;
    w1c4_ = 0;
    w1c0_ = 0;
    w20_ = 0;
    cpFlag_ = 0;
    cpCnt_ = 0;
    cp_->flag_ = 0;
    cp_->cptSent_ = 0;

    {
      unsigned char m = mult_;
      rrnSuvTail(this);
      cp_->mult_ = m;
    }
    cp_->type_ = 1;
    cp_->suvVal_ = 0;
    cpStartThis(this);
    return out;
  }

  case V92P4_WAIT_RM:
  { /* waits for the queue to ask for bits */
    unsigned n = 0;
    out = stepDrain(this, &n);
    if (n != 0)
    {
      edprintf("V92Phase4Modulator: enter Rm @ %d\r\n", cnt_);
      state_ = V92P4_RM;
      cnt_ = 0;
      b2s_->tx_->me_->mode_ = 1;
    }
    return out;
  }
  case V92P4_RM: /* Rm */
    out = stepOnes(this);
    if (cnt_ == 0x180)
    {
      state_ = V92P4_FPE;
      cnt_ = 0;
      b2s_->tx_->me_->mode_ = 2;
    }
    return out;
  case V92P4_FPE: /* FPE data */
    out = stepOnes(this);
    if (cnt_ == 0x18)
    {
      if (dbgOn())
        dsplibs_debug_printf("V92Phase4Modulator: FPE: enter SUVu @ %d\r\n", 0x18);
      {
        unsigned char m = mult_;
        state_ = V92P4_SUVU;
        cnt_ = 0;
        cp_->mult_ = m;
      }
      startSuvFromW2c(this);
      setQueueMapping(this);
    }
    return out;
  case V92P4_TERMINATED: /* terminated: data forever */
    return stepOnes(this);
  case V92P4_ERROR:
    return 0;
  default: /* 7, 14, 21, 22, > 0x1d */
    if (dbgOn())
      dsplibs_debug_printf("V92Phase4Modulator: Illegal state\r\n");
    return 0;
  }
}
