/**
 * @file v92_modulator.cpp
 * @brief V92Modulator: the V.92 top-level transmit controller (silence, phase 3, phase 4, data phase, resampling to the output rate).
 *
 * V92Modulator owns the V92Phase3Modulator and V92Phase4Modulator, the V92BitsToSymbol queue of the data phase, a Scrambler, the timing-offset
 * resampler, an optional 36-tap shaping FIR and a float sample queue.  The class is declared in pcm_modulators.h (together with V90Modulator, whose
 * definition is in v90_modulator.cpp).  The owner (VPcmFloModem) calls progress() once per output block and the enter* / exit* / initiate* methods when
 * the received signal changes the protocol phase.
 *
 * Phases (state_): 0 silence (zero symbols), 1 phase 3 (V92Phase3Modulator), 2 phase 4 (V92Phase4Modulator), 3 data (V92BitsToSymbol).
 *
 * Behaviour worth knowing:
 *  - The constructor builds the Scrambler first, stores the arguments, allocates the buffers and sub-objects (sysdep_malloc + placement new, results
 *    unchecked) and ends with the same sequence as reset(): the debug print comes BEFORE the stores, and the queue is primed with
 *    modulatorQueueLength / 2 zeros.  The shaping FIR coefficients are the 36 floats of v92TxPreFilter below.
 *  - The destructor destroys and frees the sub-objects (the resampler through its virtual deleting destructor) and clears nothing.
 *  - Every exit* method tests the sub-modulator's state first; the debug print and `event_ = 0` happen only when it is the expected one.
 *  - progress(): the number of symbols of a block is (int)(0.5 + n * 5/6) in extended precision (truncated through a 64-bit integer, low 32 bits used),
 *    cur_ = half_ - used + that count, EXCEPT in state 0 where cur_ is just that count.  The queue's used count is computed inline (pointer difference
 *    modulo size_).  The symbols are converted to floats, resampled (mkResampledSignal()), optionally filtered in place, written to the queue and the
 *    output block is read from it.  cur_ is then recomputed with blk_ (in state 3 it is also handed to V92BitsToSymbol::setSymbolsBlockSize(), whose
 *    result becomes *nbits), and "Queue is Empty/Full" sets event_ = 1 when rd_ == wr_ or size_ - used - 1 == 0.
 *  - mkResampledSignal(): when state_ == 1 and split_ != 0 the block is resampled in two parts (before / after the symbol where the phase 3
 *    generator raised event 5 / 7).  The resampler's phase is corrected between the parts by getNormalizedPhase() + 0.5 (split_ == 1) or + skew_
 *    (split_ == 2) in extended precision, minus 1 when not below 1.0 (NaN: wrapped), and the second part's outputs are appended (without their first
 *    sample when the phase wrapped).
 */

#include "pcm_modulators.h"
#include "core/dsp_math.h"
#include "core/dsp_asm.h"
#include "core/dsp_host.h"
#include "v34/v34_api.h"

extern "C"
{
}

typedef long double LD;
typedef P39_V90Modulator M9;
typedef P39_V92Modulator M2;

/* ---------------------------------------------------------------------------------------------------------------- v92TxPreFilter */

/** @brief The 36 coefficients of the transmit shaping FIR (a symmetric kernel: centre tap 0.9379 at index 17, small negative neighbours, last entry 0). */
static float v92TxPreFilter[36] __asm__("v92TxPreFilter") = {
    0.000292354584f, -3.63938128e-18f, -0.000459987583f, -0.00129610382f, -0.00272257649f, -0.00493108528f,  -0.00806269515f,  -0.0121834269f,  -0.0172666069f,
    -0.0231844559f,  -0.0297103822f,   -0.0365322642f,   -0.0432757102f,  -0.0495351218f,  -0.0549094528f,   -0.0590389259f,   -0.0616387613f,  0.937895119f,
    -0.0616387613f,  -0.0590389259f,   -0.0549094528f,   -0.0495351218f,  -0.0432757102f,  -0.0365322642f,   -0.0297103822f,   -0.0231844559f,  -0.0172666069f,
    -0.0121834269f,  -0.00806269515f,  -0.00493108528f,  -0.00272257649f, -0.00129610382f, -0.000459987583f, -3.63938128e-18f, 0.000292354584f, 0.0f};

#include "core/dsp_debug.h"

#include "core/dsp_intmath.h"
#include "vpcm_split.h"
#include "vpcm_split_value.h"

#include "core/dsp_copyword.h"

/* The Queue's used count: ((wr + size) - rd) elements (a pointer difference), remainder by size (unsigned). */
static inline unsigned queueUsed(Queue<float>* q)
{
  unsigned d = (unsigned)((q->wr_ + q->size_) - q->rd_);
  return d % q->size_;
}

/* (int)(0.5 + x * 5/6) in extended precision: the unsigned 32-bit input is zero-extended to 64 bits, the result is truncated through int64_t
 * and the low 32 bits are used. */
static inline unsigned scaleFiveSixths(unsigned x)
{
  LD v = (LD)(int64_t)x * (LD)0.833333313f + (LD)0.5f; /* 5/6 as the float constant 0.833333313f */
  return (unsigned)(int64_t)v;
}

/* Reset the phase 4 modulator to state `st` with level 0xfa0 and start code `code`; the repeat limit is the phase-2 info's rtd_. */
static inline void startPhase4V92(M2* t, unsigned st, unsigned char code)
{
  V92Phase2Info* pi = t->info_;
  t->p4_->reset((short)0xfa0, code, (V92Phase4ModulatorState)st, 0, pi->rtd_);
}

/* Set half_ and fill the sample queue with half_ zeros. */
static inline void primeQueue(M2* t)
{
  t->half_ = (unsigned)((int)t->params_->modulatorQueueLength >> 1);
  for (unsigned i = 0; i < t->half_; i++)
    t->q_->write(0.0f);
}

/* Body of the constructor (kept out of the initializer list): store the arguments, allocate the sub-objects, then reset the controller. */
static inline void ctorBody92(M2* t, unsigned blk, V92Phase2Info* info, V92Ja* ja, tagV90DILdescriptor* dil, V92CP* cp, V92MappingParams* mp,
                              V92Parameters* params)
{
  if (dbgOn())
    dsplibs_debug_printf("V92Modulator constraction\r\n");
  unsigned n = scaleFiveSixths(blk);

  t->info_ = info;
  t->blk_ = n;
  t->ja_ = ja;
  t->cp_ = cp;
  t->dil_ = dil;
  t->mp_ = mp;
  t->params_ = params;

  /* buffers sized from the block size */
  t->symBuf_ = (short*)sysdep_malloc(n * 2 + 0x14);
  t->fin_ = (float*)sysdep_malloc((t->blk_ + 10) << 2);
  t->bitBuf_ = (unsigned char*)sysdep_malloc(t->blk_ << 3);
  t->fout_ = (float*)sysdep_malloc(blk * 4 + 0x28);
  t->fcarry_ = (float*)sysdep_malloc(blk * 4 + 0x28);

  /* sub-objects */
  void* b2s = sysdep_malloc(sizeof(V92BitsToSymbol));
  new (b2s) V92BitsToSymbol(t->blk_ * 3, t->params_);
  t->b2s_ = (V92BitsToSymbol*)b2s;
  void* rs = sysdep_malloc(sizeof(ResamplerTimingOffset));
  new (rs) ResamplerTimingOffset(120, 100.0f, 16, 0.980000019f, 0.0f, 0);
  t->rs_ = (ResamplerTimingOffset*)rs;
  void* p3 = sysdep_malloc(sizeof(V92Phase3Modulator));
  new (p3) V92Phase3Modulator(t->params_);
  t->p3_ = (V92Phase3Modulator*)p3;
  void* p4 = sysdep_malloc(sizeof(V92Phase4Modulator));
  new (p4) V92Phase4Modulator(t->params_, t->b2s_, t->cp_, t->mp_);
  t->p4_ = (V92Phase4Modulator*)p4;
  void* q = sysdep_malloc(sizeof(Queue<float>));
  new (q) Queue<float>(t->params_->modulatorQueueLength);
  t->q_ = (Queue<float>*)q;
  void* fir = sysdep_malloc(sizeof(FloatFIR));
  new (fir) FloatFIR(0x24, v92TxPreFilter, 0x63);
  t->fir_ = (FloatFIR*)fir;

  /* the reset sequence, inlined */
  if (dbgOn())
    dsplibs_debug_printf("V92Modulator reset\r\n");
  t->scr_.reset(0);
  t->state_ = 0;
  t->cnt_ = 0;
  t->event_ = 0;
  t->split_ = 0;
  t->cur_ = t->blk_;
  t->q_->reset();
  t->codeA_ = 0;
  t->codeB_ = 0;
  t->ppm_ = 0.0f;
  primeQueue(t);
  t->fir_->reset();
}

/* Body of the destructor: destroy and free every sub-object and buffer that was allocated. */
static inline void dtorBody92(M2* t)
{
  if (t->p3_)
  {
    t->p3_->~V92Phase3Modulator();
    sysdep_free(t->p3_);
  }
  if (t->p4_)
  {
    t->p4_->~V92Phase4Modulator();
    sysdep_free(t->p4_);
  }
  if (t->b2s_)
  {
    t->b2s_->~V92BitsToSymbol();
    sysdep_free(t->b2s_);
  }
  if (t->rs_)
    delete t->rs_; /* the virtual deleting destructor */
  if (t->q_)
  {
    t->q_->~Queue<float>();
    sysdep_free(t->q_);
  }
  if (t->fir_)
  {
    t->fir_->~FloatFIR();
    sysdep_free(t->fir_);
  }
  if (t->symBuf_)
    sysdep_free(t->symBuf_);
  if (t->fin_)
    sysdep_free(t->fin_);
  if (t->bitBuf_)
    sysdep_free(t->bitBuf_);
  if (t->fout_)
    sysdep_free(t->fout_);
  if (t->fcarry_)
    sysdep_free(t->fcarry_);
  KEEP_STORES_TO(t);
}

/* ================================================================================================================ V92Modulator */

/**
 * @brief Create the modulator, its sub-objects and buffers, and reset it to silence.
 * @param blk     output block size in samples
 * @param info    phase-2 info (rtd_ is read when phase 4 starts); not owned
 * @param ja      Ja block for phase 3; not owned
 * @param dil     DIL descriptor for phase 3; not owned
 * @param cp      CP frame for phase 4; not owned
 * @param mp      data-phase mapping parameters; not owned
 * @param params  V.92 parameter block (queue length, shaping filter switch); not owned
 * @warning No allocation result is tested.
 */
V92Modulator::V92Modulator(unsigned blk, V92Phase2Info* info, V92Ja* ja, tagV90DILdescriptor* dil, V92CP* cp, V92MappingParams* mp, V92Parameters* params)
    : scr_(5, 0x17, 0x63)
{
  ctorBody92(this, blk, info, ja, dil, cp, mp, params);
}

/** @brief Destroy and free the sub-objects and buffers. */
V92Modulator::~V92Modulator() { dtorBody92(this); }

/** @brief Delay of the transmit shaping filter in symbols (0x12 when v92ApplyTxShapingFilter is set, else 0).  Not called in the library. */
unsigned V92Modulator::getV92TxFilterDelay() const { return params_->v92ApplyTxShapingFilter != 0 ? 0x12u : 0u; }

/** @brief Return to silence (state 0): clear the counters, the scrambler, the filter and the queue, then refill the queue with half_ zeros. */
void V92Modulator::reset()
{
  if (dbgOn())
    dsplibs_debug_printf("V92Modulator reset\r\n");
  scr_.reset(0);
  state_ = 0;
  unsigned b = blk_;
  cnt_ = 0;
  cur_ = b;
  event_ = 0;
  split_ = 0;
  q_->reset();
  codeA_ = 0;
  codeB_ = 0;
  ppm_ = 0.0f;
  primeQueue(this);
  fir_->reset();
}

/** @brief Start phase 3 (Ru) unless it is already running. */
void V92Modulator::enterPhase3()
{
  if (state_ != 1)
  {
    if (dbgOn())
      dsplibs_debug_printf("V92Modulator enter Phase 3\r\n");
    V92Phase2Info* pi = info_;
    p3_->reset((short)0xfa0, V92P3_RU, 0, ja_, dil_, pi->rtd_);
    state_ = 1;
    cnt_ = 0;
    event_ = 0;
    split_ = 0;
  }
}

/** @brief Start phase 4 at its first state with start code codeA_ unless it is already running.  No caller in the library (progress() does this inline). */
void V92Modulator::enterPhase4()
{
  if (state_ != 2)
  {
    if (dbgOn())
      dsplibs_debug_printf("V92Modulator: enter Phase 4\r\n");
    startPhase4V92(this, 0, codeA_);
    state_ = 2;
    cnt_ = 0;
    event_ = 0;
  }
}

/** @brief Leave the Ja state of phase 3 (only when phase 3 is in it). */
void V92Modulator::exitJa()
{
  if (p3_->state_ == V92P3_JA)
  {
    if (dbgOn())
      dsplibs_debug_printf("V92Modulator: exit Ja\r\n");
    p3_->exitJa();
    event_ = 0;
  }
}

/** @brief Leave the Silence state of phase 3 (only when phase 3 is in it). */
void V92Modulator::exitSilence()
{
  if (p3_->state_ == V92P3_SILENCE)
  {
    if (dbgOn())
      dsplibs_debug_printf("V92Modulator: exit Silence\r\n");
    p3_->exitSilence();
    event_ = 0;
  }
}

/** @brief Leave the SuSecond state of phase 3 (only when phase 3 is in it). */
void V92Modulator::exitSuSecond()
{
  if (p3_->state_ == V92P3_SU_SECOND)
  {
    if (dbgOn())
      dsplibs_debug_printf("V92Modulator: exit SuSecond\r\n");
    p3_->exitSuSecond();
    event_ = 0;
  }
}

/** @brief Leave the TRN1uSecond state of phase 3 (only when phase 3 is in it). */
void V92Modulator::exitTRN1uSecond()
{
  if (p3_->state_ == V92P3_TRN1U_SECOND)
  {
    if (dbgOn())
      dsplibs_debug_printf("V92Modulator: exit TRN1uSecond\r\n");
    p3_->exitTRN1u();
    event_ = 0;
  }
}

/** @brief Tell phase 4 that the CPt was received (only when phase 4 is in its Ru state). */
void V92Modulator::exitCPt()
{
  if (p4_->state_ == V92P4_RU)
  {
    if (dbgOn())
      dsplibs_debug_printf("V92Modulator: exit CPt\r\n");
    p4_->exitCPt();
    event_ = 0;
  }
}

/** @brief Enter the data phase (state 3) and set the symbol block size to blk_.  No caller in the library (progress() does this inline). */
void V92Modulator::enterDataPhase()
{
  if (state_ != 3)
  {
    edprintf("V92Modulator:enter  Data Phase:\r\n");
    state_ = 3;
    unsigned b = blk_;
    cnt_ = 0;
    event_ = 0xa;
    b2s_->setSymbolsBlockSize(b);
  }
}

/**
 * @brief Start a rate renegotiation: move from the data phase to phase 4 at the Ru (or data-to-Ru) state.
 * @return 0 when started, -1 when the modulator is not in the data phase
 */
int V92Modulator::initiateRRN()
{
  if (state_ != 3)
  {
    if (dbgOn())
      dsplibs_debug_printf("V92Modulator: RRN requested but NOT approved\r\n");
    return -1;
  }

  if (dbgOn())
    dsplibs_debug_printf("V92Modulator: RRN requested, enter Phase 4\r\n");
  state_ = 2;
  cnt_ = 0;
  b2s_->setSymbolsBlockSize(1);

  /* the phase 4 start state depends on whether the queue still holds bits */
  unsigned st;
  if (b2s_->nofBitsForNextTime() == 0)
  {
    edprintf("V92Modulator: Phase4Modulator state initialized to DataToRuModulation\r\n");
    st = 0x12;
  }
  else
  {
    edprintf("V92Modulator: Phase4Modulator state initialized to RuModulation\r\n");
    st = 0x13;
  }

  startPhase4V92(this, st, codeB_);
  event_ = 0;
  p4_->resetBeforRRN();
  cp_->flag_ = 0;
  cp_->infoToBits();
  return 0;
}

/**
 * @brief Start the final phase-4 exchange (FPE): move from the data phase to phase 4 at the Rm (or data-to-Rm) state and switch the modulus encoder to mode 1.
 * @return 0 when started, -1 when the modulator is not in the data phase
 */
int V92Modulator::initiateFPE()
{
  if (state_ != 3)
  {
    if (dbgOn())
      dsplibs_debug_printf("V92Modulator: FPE requested but NOT approved\r\n");
    return -1;
  }

  if (dbgOn())
    dsplibs_debug_printf("V92Modulator: FPE requested, enter Phase 4\r\n");
  state_ = 2;
  cnt_ = 0;
  b2s_->setSymbolsBlockSize(1);
  b2s_->tx_->me_->mode_ = 1;

  /* the phase 4 start state depends on whether the queue still holds bits */
  unsigned st;
  if (b2s_->nofBitsForNextTime() == 0)
  {
    edprintf("V92Modulator: Phase4Modulator state initialized to DataToRmModulation\r\n");
    st = 0x19;
  }
  else
  {
    edprintf("V92Modulator: Phase4Modulator state initialized to RmModulation\r\n");
    st = 0x1a;
  }

  startPhase4V92(this, st, codeB_);
  event_ = 0;
  p4_->resetBeforFPE();
  cp_->flag_ = 0;
  cp_->infoToBits();
  return 0;
}

/* The resampler's phase correction between the two parts of a split block: set the phase to s (extended precision), minus 1 when s is not below 1.0;
 * returns the "wrapped" flag. */
static inline bool correctPhase(M2* t, LD s)
{
  bool wrapped;
  if ((LD)1.0L > s)
  {
    wrapped = false;
  }
  else
  {
    wrapped = true;
    s = s - (LD)1.0L;
  }
  t->rs_->setNormalizedPhase((float)s);
  return wrapped;
}

/**
 * @brief Resample the symbols in fin_ (cur_ of them) into fout_.
 * @param outCnt  receives the number of output samples
 * @note  A block that phase 3 marked with split_ is resampled in two parts with a phase correction between them (see the file header);
 *        split_ and splitAt_ are cleared.
 */
void V92Modulator::mkResampledSignal(unsigned& outCnt)
{
  if (state_ == 1 && split_ != 0)
  {
    bool wrapped = false;
    unsigned cnt1, cnt2;
    rs_->resample(fin_, splitAt_, fout_, cnt1);
    unsigned sp = split_;
    if (sp == 2)
    {
      if (dbgOn())
      {
        Split sv = splitValue((LD)skew_, (LD)100000.0f);
        dsplibs_debug_printf("V92Modulator: setPhase = %c%d.%05d\r\n", sv.sign, sv.ip, sv.fp);
      }
      float ph = rs_->getNormalizedPhase();
      wrapped = correctPhase(this, (LD)ph + (LD)skew_);
    }
    else if (sp == 1)
    {
      if (dbgOn())
        dsplibs_debug_printf("V92Modulator: setPhase = 0.5\r\n");
      float ph = rs_->getNormalizedPhase();
      wrapped = correctPhase(this, (LD)ph + (LD)0.5f);
    }
    rs_->resample(fin_ + splitAt_, cur_ - splitAt_, fcarry_, cnt2);
    if (wrapped)
    {
      outCnt = cnt1 + cnt2 - 1;
      unsigned m = cnt2 - 1;
      for (unsigned i = 0; i < m; i++)
        copyWord(&fout_[cnt1 + i], &fcarry_[i + 1]);
    }
    else
    {
      outCnt = cnt1 + cnt2;
      for (unsigned i = 0; i < cnt2; i++)
        copyWord(&fout_[cnt1 + i], &fcarry_[i]);
    }
    split_ = 0;
    splitAt_ = 0;
  }
  else
  {
    rs_->resample(fin_, cur_, fout_, outCnt);
  }
}

/**
 * @brief Produce one output block.
 * @param in     data bits to send (state 3 only)
 * @param nbits  in: number of bits in `in`; out: number of bits wanted for the next call (0 outside the data phase)
 * @param out    receives n output samples
 * @param n      number of output samples
 * @note  Generates the symbols of the current phase (cur_ of them), resamples them, queues them and reads n samples back; event_ reports phase events
 *        (see the file header).
 */
void V92Modulator::progress(int* in, unsigned& nbits, float* out, unsigned n)
{
  unsigned tmp = scaleFiveSixths(n);
  unsigned st;
  unsigned cur;

  /* the number of symbols for this block */
  event_ = 0;
  cnt_ += tmp;
  cur = half_ - queueUsed(q_) + tmp;

  /* generate the symbols of the current phase */
  st = state_;
  if (st == 1)
  { /* phase 3: symbols from the phase 3 generator, switching to phase 4 when it raises event 8 */
    cur_ = cur;
    unsigned i = 0;
    if (cur > 0)
    {
      unsigned s = st;
      for (;;)
      {
        if (s == 1)
        {
          short g = p3_->generateSymbol();
          symBuf_[i] = g;
          unsigned ev = p3_->event_;
          if (ev != 0)
          {
            event_ = ev;
            if (ev == 5)
            {
              split_ = 1;
              splitAt_ = i;
            }
            else if (ev == 7)
            {
              split_ = 2;
              splitAt_ = i;
            }
          }
          if (event_ == 8)
          {
            if (state_ != 2)
            {
              if (dbgOn())
                dsplibs_debug_printf("V92Modulator: enter Phase 4\r\n");
              startPhase4V92(this, 0, codeA_);
              state_ = 2;
              cnt_ = 0;
              event_ = 0;
            }
            if (dbgOn())
              dsplibs_debug_printf("V92Modulator progress: Enter phase4\r\n");
          }
        }
        else
        {
          short g = p4_->generateSymbol();
          symBuf_[i] = g;
          unsigned ev = p4_->event_;
          if (ev != 0)
            event_ = ev;
        }
        i++;
        if (cur_ <= i)
          break;
        s = state_;
      }
    }
    nbits = 0;
  }
  else if (st == 0)
  { /* silence */
    cur_ = tmp;
    for (unsigned i = 0; i < tmp; i++)
      symBuf_[i] = 0;
    nbits = 0;
  }
  else if ((int)st < 0)
  {
    cur_ = cur;
    if (dbgOn())
      dsplibs_debug_printf("V92Modulator progress: Illegal state\r\n");
  }
  else
  {
    cur_ = cur;
    if (st == 2)
    { /* phase 4: symbols from the phase 4 generator; event 9 moves on to the data phase */
      unsigned i = 0;
      if (cur > 0)
      {
        do
        {
          short g = p4_->generateSymbol();
          symBuf_[i] = g;
          unsigned ev = p4_->event_;
          if (ev != 0)
            event_ = ev;
          i++;
        } while (cur_ > i);
      }
      if (event_ == 9)
      {
        if (state_ != 3)
        {
          edprintf("V92Modulator:enter  Data Phase:\r\n");
          state_ = 3;
          unsigned b = blk_;
          cnt_ = 0;
          event_ = 0xa;
          b2s_->setSymbolsBlockSize(b);
        }
        nbits = b2s_->nofBitsForNextTime();
      }
      else
      {
        nbits = 0;
      }
    }
    else if (st == 3)
    { /* data phase: scramble the caller's bits and map them through the queue */
      unsigned nb = nbits;
      scr_.process(in, bitBuf_, nb);
      b2s_->process(bitBuf_, nbits, symBuf_);
    }
    else
    {
      if (dbgOn())
        dsplibs_debug_printf("V92Modulator progress: Illegal state\r\n");
    }
  }

  /* common tail: symbols -> floats -> resampler -> (filter) -> queue -> output */
  {
    unsigned c = cur_;
    short* sb = symBuf_;
    float* fi = fin_;
    for (unsigned i = 0; i < c; i++)
      fi[i] = (float)sb[i];
  }

  rs_->setTimingOffset(ppm_);
  unsigned outCnt;
  mkResampledSignal(outCnt);
  if (params_->v92ApplyTxShapingFilter != 0)
    fir_->process(fout_, fout_, outCnt);
  q_->write(fout_, outCnt);
  q_->read(out, n);

  /* next block size and queue status */
  {
    Queue<float>* q = q_;
    unsigned u = queueUsed(q);
    unsigned c = half_ - u + blk_;
    unsigned s3 = state_;
    cur_ = c;
    if (s3 == 3)
    {
      nbits = b2s_->setSymbolsBlockSize(c);
      q = q_;
    }
    bool full;
    if (q->rd_ == q->wr_)
    {
      full = true;
    }
    else
    {
      unsigned sz = q->size_;
      unsigned used2 = (unsigned)((q->wr_ + sz) - q->rd_) % sz;
      full = (sz - used2 - 1) == 0;
    }
    if (full)
    {
      if (dbgOn())
        dsplibs_debug_printf("V92Modulator: Queue is Empty/Full !!!\r\n");
      event_ = 1;
    }
  }
}
