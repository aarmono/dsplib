
/**
 * @file v90_modulator.cpp
 * @brief V90Modulator: the top-level V.90 transmit controller (symbol generation for silence, phase 3, phase 4 and the data phase).
 *
 * V90Modulator (declared in pcm_modulators.h together with V92Modulator, which is defined in v92_modulator.cpp) owns the heap-allocated
 * sub-modulators and the data path of the V.90 transmitter:
 * - V90Phase3Modulator (p3_) generates the phase-3 symbols (SD, Jd, DIL...), V90Phase4Modulator (p4_) the phase-4 symbols (Ri, MP, MPNot, TRN2d,
 *   RRN / FPE recovery), V90BitsToSymbol (b2s_) maps data bits to symbols in the data phase, and the scrambler scr_ scrambles the input bits first;
 * - state_ is the current phase (0 silence, 1 phase 3, 2 phase 4, 3 data), cnt_ counts the symbols of that phase, event_ is the event raised by the
 *   generator in the last progress() call (6 = phase 3 asks for phase 4, 7 = phase 4 asks for the data phase, 8 = data phase entered,
 *   9 = a debug-requested RRN was started).
 *
 * Entry points: progress() generates one block of n symbols (as floats) per call; the phase changes and the CP / MP acknowledgements are methods of
 * their own (enterPhase3(), exitJd(), exitDIL(), exitRi(), acknowledge*Reception(), initiateRRN(), initiateFPE() ...).  Each exit* / acknowledge*
 * method first tests that the sub-modulator is in the expected state; only then it prints the debug line and sets event_ = 0.  The debug prints
 * are written at debug level > 1.
 *
 * Behaviour worth knowing:
 * - The constructor builds the scrambler first, stores the arguments, then allocates (sysdep_malloc + placement new) the buffers and the
 *   sub-objects.  state_, cnt_ and event_ are NOT initialised by the constructor: call reset() before the first progress().
 * - The destructor destroys and frees the sub-objects and clears nothing.
 * - progress() inlines the entry to the data phase: the rate print tests state_ == 3 again after setSymbolsBlockSize(); in state 1 the loop
 *   re-reads state_ after every symbol (the phase-3 generator may switch the modulator to phase 4 in the middle of a block), and the event of
 *   the phase-3 generator overwrites event_ only when it is non-zero.
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

#include "core/dsp_debug.h"

/** @brief Data rate in bit/s for the debug print: (int)(0.5 + (w0 * 8000) * (1/6 as a float)) in extended precision, w0 = bits per frame of the mapper. */
static inline unsigned dataRate(V90BitsToSymbol* b2s)
{
  unsigned v = b2s->mapper_->w0_ * 8000u;
  LD r = (LD)0.5f + (LD)(int64_t)v * (LD)0.166666672f;
  return (unsigned)(int64_t)r;
}

/**
 * @brief Enter the V.90 data phase (shared by enterDataPhase() and progress()).
 *
 * Prints the spectral parameters of mapping block 2, sets state 3 / event 8, gives the bit-to-symbol mapper its block size and prints the data rate.
 * The rate is printed only when state_ is still 3 after setSymbolsBlockSize() (re-read after the call).
 */
static inline void enterData90(M9* t)
{
  edprintf("V90Modulator: Data Phase spectral parameters:\r\n");
  displaySpectralParams(t->mp2_);
  t->state_ = 3;
  t->cnt_ = 0;
  t->event_ = 8;
  t->b2s_->setSymbolsBlockSize(t->blk_);
  if (dbgOn())
  {
    unsigned rate = 0;
    if (t->state_ == 3)
      rate = dataRate(t->b2s_);
    dsplibs_debug_printf("V90Modulator: enter Data Phase, Rate = %d [bps]\r\n", (int)rate);
  }
}

/** @brief Reset the phase-4 generator into state @p st from the phase-2 info (PCM type, uinfo, round-trip delay); used on every entry to phase 4. */
static inline void startPhase4V90(M9* t, unsigned st)
{
  V90Phase2Info* pi = t->info_;
  t->p4_->reset((PcmType)pi->pcmType_, pi->uinfo_, (Phase4ModulatorState)st, 0, pi->rtd_);
}

/**
 * @brief Body of the V90Modulator constructor: store the arguments, allocate the symbol and bit buffers and construct the sub-modulators.
 *
 * symBuf_ holds @p blk 16-bit symbols and bitBuf_ 8 * @p blk bits; the bits-to-symbol mapper gets a queue of 3 * blk + 5000 entries.
 * @warning the sysdep_malloc() results are not checked.
 */
static inline void ctorBody90(M9* t, unsigned blk, V90Phase2Info* info, V90Jd* jd, V92Jd* v92jd, tagV90DILdescriptor* dil, V90MappingParams* mp1,
                              V90MappingParams* mp2, tagV90AdditionalCPinfo* addCp, V90CP* cp, V90MP* mp, V90Parameters* params, unsigned session)
{
  t->mp1_ = mp1;
  t->info_ = info;
  t->jd_ = jd;
  t->v92jd_ = v92jd;
  t->dil_ = dil;
  t->mp2_ = mp2;
  t->mp_ = mp;
  t->cp_ = cp;
  t->addCp_ = addCp;
  t->session_ = session;
  t->blk_ = blk;
  t->params_ = params;

  t->symBuf_ = (short*)sysdep_malloc(blk * 2);
  t->bitBuf_ = (unsigned char*)sysdep_malloc(blk * 8);

  void* b2s = sysdep_malloc(sizeof(V90BitsToSymbol));
  new (b2s) V90BitsToSymbol(blk * 2 + blk + 5000, t->params_);
  t->b2s_ = (V90BitsToSymbol*)b2s;
  void* p3 = sysdep_malloc(sizeof(V90Phase3Modulator));
  new (p3) V90Phase3Modulator(t->params_, t->session_);
  t->p3_ = (V90Phase3Modulator*)p3;
  void* p4 = sysdep_malloc(sizeof(V90Phase4Modulator));
  new (p4) V90Phase4Modulator(t->params_, t->session_, t->b2s_, t->mp_, t->mp2_, t->mp1_, t->cp_, 0xc);
  t->p4_ = (V90Phase4Modulator*)p4;
}

/** @brief Body of the V90Modulator destructor: destroy and free the sub-modulators and the mapper, free the buffers; no member is cleared. */
static inline void dtorBody90(M9* t)
{
  if (t->p3_)
  {
    t->p3_->~V90Phase3Modulator();
    sysdep_free(t->p3_);
  }
  if (t->p4_)
  {
    t->p4_->~V90Phase4Modulator();
    sysdep_free(t->p4_);
  }
  if (t->b2s_)
  {
    t->b2s_->~V90BitsToSymbol();
    sysdep_free(t->b2s_);
  }
  if (t->symBuf_)
    sysdep_free(t->symBuf_);
  if (t->bitBuf_)
    sysdep_free(t->bitBuf_);
  KEEP_STORES_TO(t);
}

/* ================================================================================================================ V90Modulator */

/**
 * @brief Construct the transmit controller.
 * @param blk      block size in samples (symbols per progress() call at most)
 * @param info     phase-2 info (PCM type, uinfo, round-trip delay); not owned
 * @param jd       V.90 Jd object; not owned
 * @param v92jd    V.92 Jd object; not owned
 * @param dil      DIL descriptor; not owned
 * @param mp1      mapping parameters announced for TRN2d; not owned
 * @param mp2      mapping parameters of the data phase; not owned
 * @param addCp    additional CP info; not owned
 * @param cp       CP frame; not owned
 * @param mp       MP frame; not owned
 * @param params   parameter block; not owned
 * @param session  session flag (nonzero: this side sends the CP frame, zero: the MP frame)
 * @warning state_, cnt_ and event_ are not initialised: call reset() before progress().
 */
V90Modulator::V90Modulator(unsigned blk, V90Phase2Info* info, V90Jd* jd, V92Jd* v92jd, tagV90DILdescriptor* dil, V90MappingParams* mp1, V90MappingParams* mp2,
                           tagV90AdditionalCPinfo* addCp, V90CP* cp, V90MP* mp, V90Parameters* params, unsigned session)
    : scr_(0x12, 0x17, 0x63)
{
  ctorBody90(this, blk, info, jd, v92jd, dil, mp1, mp2, addCp, cp, mp, params, session);
}

V90Modulator::~V90Modulator() { dtorBody90(this); }

/** @brief Set the session flag of the controller and of both sub-modulators. */
void V90Modulator::setSessionFlag(unsigned f)
{
  session_ = f;
  p3_->setSessionFlag(f);
  p4_->setSessionFlag(f);
}

/** @brief Return to silence: reset the scrambler, state_ = 0, cnt_ = 0, event_ = 0. */
void V90Modulator::reset()
{
  if (dbgOn())
    dsplibs_debug_printf("V90Modulator reset\r\n");
  scr_.reset(0);
  state_ = 0;
  cnt_ = 0;
  event_ = 0;
}

/** @brief Enter phase 3 (state 1) unless already there: reset the phase-3 generator into its SD state from the phase-2 info. */
void V90Modulator::enterPhase3()
{
  if (state_ != 1)
  {
    if (dbgOn())
      dsplibs_debug_printf("V90Modulator enter Phase 3\r\n");
    V90Phase2Info* pi = info_;
    p3_->reset((PcmType)pi->pcmType_, pi->uinfo_, P3M_SD, 0, jd_, v92jd_, dil_, pi->rtd_);
    state_ = 1;
    cnt_ = 0;
    event_ = 0;
  }
}

/** @brief Enter phase 4 (state 2) unless already there: reset the phase-4 generator into state 0.  No caller in src/. */
void V90Modulator::enterPhase4()
{
  if (state_ != 2)
  {
    if (dbgOn())
      dsplibs_debug_printf("V90Modulator: enter Phase 4\r\n");
    startPhase4V90(this, 0);
    state_ = 2;
    cnt_ = 0;
    event_ = 0;
  }
}

/** @brief Leave the Jd state of the phase-3 generator (only when it is in it) and clear event_.  No caller in src/. */
void V90Modulator::exitJd()
{
  if (p3_->state_ == P3M_JD)
  {
    if (dbgOn())
      dsplibs_debug_printf("V90Modulator: exit Jd\r\n");
    p3_->exitJd();
    event_ = 0;
  }
}

/** @brief Leave the Jd phase (state 5 of the phase-3 generator, only when it is in it) and clear event_.  No caller in src/. */
void V90Modulator::exitJdPhase()
{
  if (p3_->state_ == P3M_STATE5)
  {
    if (dbgOn())
      dsplibs_debug_printf("V90Modulator: exit JdPhase\r\n");
    p3_->exitJdPhase();
    event_ = 0;
  }
}

/** @brief Leave the DIL state of the phase-3 generator (only when it is in it); when the generator then raises event 6, enter phase 4. */
void V90Modulator::exitDIL()
{
  if (p3_->state_ == P3M_DIL)
  {
    if (dbgOn())
      dsplibs_debug_printf("V90Modulator: exit DIL\r\n");
    p3_->exitDIL();
    if (p3_->event_ == 6 && state_ != 2)
    {
      if (dbgOn())
        dsplibs_debug_printf("V90Modulator: enter Phase 4\r\n");
      startPhase4V90(this, 0);
      state_ = 2;
      cnt_ = 0;
      event_ = 0;
    }
  }
}

/**
 * @brief Leave the Ri state of the phase-4 generator (only when it is in it) and start TRN2d.
 *
 * Gives the generator the mapping parameters mp1_, prints them and the TRN2d D value (mp1_->kBits_) and stores that value as trn2dD_ in the CP frame
 * (session_ != 0) or the MP frame.  No caller in src/.
 */
void V90Modulator::exitRi()
{
  if (p4_->state_ == P4M_RI)
  {
    if (dbgOn())
      dsplibs_debug_printf("V90Modulator: exit Ri\r\n");
    p4_->exitRi();
    event_ = 0;
    p4_->setMappingParams(mp1_);
    edprintf("V90Modulator: TRN2d spectral parameters:\r\n");
    displaySpectralParams(mp1_);
    edprintf("V90Modulator: TRN2d D = %d\r\n", (int)mp1_->kBits_);
    if (session_ != 0)
      cp_->trn2dD_ = mp1_->kBits_;
    else
      mp_->trn2dD_ = mp1_->kBits_;
  }
}

/**
 * @brief The far end's CP frame was received: acknowledge it in the MP frame (cPack_ = 1, re-encode), make MPNot the state after TRN2d, and end the
 *        MP state of the phase-4 generator if it is in it.  No caller in src/.
 */
void V90Modulator::acknowledgeCPReception()
{
  mp_->cPack_ = 1;
  mp_->infoToBits();
  p4_->setNextStateAfterTRN2d(P4M_MP_NOT);
  if (p4_->state_ == P4M_MP)
  {
    if (dbgOn())
      dsplibs_debug_printf("V90Modulator: exit MP due to CP reception\r\n");
    p4_->exitMP();
    event_ = 0;
  }
}

/**
 * @brief The far end's CPNot was received: end MPNot, or acknowledge in the MP frame and end MP, or (in the last MP) mark the MPNot exit as delayed
 *        (mpNotSeen_), depending on the state of the phase-4 generator.  No caller in src/.
 */
void V90Modulator::acknowledgeCPNotReception()
{
  unsigned st = p4_->state_;
  if (st == P4M_MP_NOT)
  {
    if (dbgOn())
      dsplibs_debug_printf("V90Modulator: exit MPNot due to CPNot reception\r\n");
    p4_->exitMPNot();
    event_ = 0;
  }
  else if (st == P4M_MP)
  {
    mp_->cPack_ = 1;
    mp_->infoToBits();
    if (dbgOn())
      dsplibs_debug_printf("V90Modulator: exit MP due to CPNot reception\r\n");
    p4_->exitMP();
    event_ = 0;
  }
  else if (st == P4M_MP_LAST)
  {
    p4_->mpNotSeen_ = 1;
    if (dbgOn())
      dsplibs_debug_printf("V90Modulator: setting delayed MPNot exit due to CPNot reception\r\n");
  }
}

/** @brief The far end's E sequence was received: end MPNot, or (in the last MP) mark the MPNot exit as delayed (mpNotSeen_).  No caller in src/. */
void V90Modulator::acknowledgeEReception()
{
  unsigned st = p4_->state_;
  if (st == P4M_MP_NOT)
  {
    if (dbgOn())
      dsplibs_debug_printf("V90Modulator: exit MPNot due to E reception\r\n");
    p4_->exitMPNot();
    event_ = 0;
  }
  else if (st == P4M_MP_LAST)
  {
    p4_->mpNotSeen_ = 1;
    if (dbgOn())
      dsplibs_debug_printf("V90Modulator: setting delayed MPNot exit due to E reception\r\n");
  }
}

/** @brief Enter the data phase (state 3) unless already there (progress() does the same inline when phase 4 raises event 7).  No caller in src/. */
void V90Modulator::enterDataPhase()
{
  if (state_ != 3)
    enterData90(this);
}

/**
 * @brief Start a rate renegotiation (RRN) from the data phase.
 *
 * Switches to phase 4 with a block size of one symbol; the phase-4 generator starts in the data-to-Rd modulation state when the mapper has no
 * bit pending for the next symbol time, otherwise in the Rd modulation state.  Then it arms the Rd / Rt symbols from mp2_ and re-encodes the CP frame
 * (session_ != 0, cptFlag_ = 0) or the MP frame (cPack_ = 0).
 * @return 0 when started, -1 when refused (not in the data phase)
 */
int V90Modulator::initiateRRN()
{
  if (state_ != 3)
  {
    if (dbgOn())
      dsplibs_debug_printf("V90Modulator: RRN requested but NOT approved\r\n");
    return -1;
  }
  if (dbgOn())
    dsplibs_debug_printf("V90Modulator: RRN requested, enter Phase 4\r\n");
  state_ = 2;
  cnt_ = 0;
  b2s_->setSymbolsBlockSize(1);

  unsigned st;
  if (b2s_->nofBitsForNextTime() == 0)
  {
    edprintf("V90Modulator: Phase4Modulator state initialized to DataToRdModulation\r\n");
    st = 0x14;
  }
  else
  {
    edprintf("V90Modulator: Phase4Modulator state initialized to RdModulation\r\n");
    st = 0x15;
  }
  startPhase4V90(this, st);
  event_ = 0;

  p4_->resetBeforRRN();
  p4_->setRdRtSymbols(mp2_);
  if (session_ != 0)
  {
    cp_->cptFlag_ = 0;
    cp_->infoToBits();
  }
  else
  {
    mp_->cPack_ = 0;
    mp_->infoToBits();
  }
  return 0;
}

/**
 * @brief Start a fast phase-4 entry (FPE) from the data phase; same sequence as initiateRRN() with the Rf states and symbols.
 * @return 0 when started, -1 when refused (not in the data phase)
 * @warning always re-encodes the CP frame (cp_), whatever session_ is; unlike initiateRRN() it has no MP-frame branch, so cp_ must be valid on both
 *          sessions.
 */
int V90Modulator::initiateFPE()
{
  if (state_ != 3)
  {
    if (dbgOn())
      dsplibs_debug_printf("V90Modulator: FPE requested but NOT approved\r\n");
    return -1;
  }
  if (dbgOn())
    dsplibs_debug_printf("V90Modulator: FPE requested, enter Phase 4\r\n");
  state_ = 2;
  cnt_ = 0;
  b2s_->setSymbolsBlockSize(1);

  unsigned st;
  if (b2s_->nofBitsForNextTime() == 0)
  {
    edprintf("V90Modulator: Phase4Modulator state initialized to DataToRfModulation\r\n");
    st = 0x1c;
  }
  else
  {
    edprintf("V90Modulator: Phase4Modulator state initialized to RfModulation\r\n");
    st = 0x1d;
  }
  startPhase4V90(this, st);
  event_ = 0;

  p4_->setRfSymbols(mp2_);
  cp_->cptFlag_ = 0;
  cp_->infoToBits();
  return 0;
}

/**
 * @brief Generate one block of @p n transmit symbols.
 * @param in     data bits to send (data phase only), one bit per int
 * @param nbits  data phase: number of bits in @p in on entry (passed to the scrambler and the mapper, which may update it); other states: set to 0,
 *               except when phase 4 has just raised event 7, when it is the number of bits the mapper wants for the next call
 * @param out    the symbols as floats
 * @param n      number of symbols to generate
 *
 * Per state_: 0 silence (zeros); 1 phase 3 (symbol by symbol from the phase-3 generator, switching to the phase-4 generator after event 6); 2 phase 4
 * (on event 7 the data phase is entered); 3 data (scramble, map bits to symbols; with the debug parameter set, an RRN is started after the debug
 * time and event 9 is raised).  The event of the block is left in event_.
 * @warning @p n must not exceed the block size given to the constructor (symBuf_ is not bounds-checked), and in the data phase @p nbits must not
 *          exceed 8 * that block size (bitBuf_).
 */
void V90Modulator::progress(int* in, unsigned& nbits, float* out, unsigned n)
{
  unsigned st = state_;
  cnt_ += n;
  event_ = 0;

  /* silence */
  if (st == 0)
  {
    for (unsigned i = 0; i < n; i++)
      symBuf_[i] = 0;
    nbits = 0;
  }
  /* phase 3: symbols from the phase-3 generator; when it asks for phase 4 (event 6) the rest of the block comes from the phase-4 generator */
  else if (st == 1)
  {
    unsigned i = 0;
    if (i < n)
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
            event_ = ev;
          if (event_ == 6 && state_ != 2)
          {
            if (dbgOn())
              dsplibs_debug_printf("V90Modulator: enter Phase 4\r\n");
            startPhase4V90(this, 0);
            state_ = 2;
            cnt_ = 0;
            event_ = 0;
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
        if (i >= n)
          break;
        s = state_;
      }
    }
    nbits = 0;
  }
  /* phase 4: on event 7 enter the data phase and report how many bits the next call needs */
  else if (st == 2)
  {
    for (unsigned i = 0; i < n; i++)
    {
      short g = p4_->generateSymbol();
      symBuf_[i] = g;
      unsigned ev = p4_->event_;
      if (ev != 0)
        event_ = ev;
    }
    if (event_ == 7)
    {
      if (state_ != 3)
        enterData90(this);
      nbits = b2s_->nofBitsForNextTime();
    }
    else
    {
      nbits = 0;
    }
  }
  /* data phase: scramble the input bits and map them to symbols */
  else if (st == 3)
  {
    scr_.process(in, bitBuf_, nbits);
    b2s_->process(bitBuf_, nbits, symBuf_);
    if (params_->debugDigitalModemInitiateRrn != 0 && cnt_ > (unsigned)params_->debugDigitalModemInitiateRrnTime)
    {
      initiateRRN();
      event_ = 9;
    }
  }
  else
  {
    if (dbgOn())
      dsplibs_debug_printf("V90Modulator progress: Illegal state\r\n");
  }

  /* convert the symbols to floats */
  for (unsigned i = 0; i < n; i++)
    out[i] = (float)symBuf_[i];
}
