/**
 * @file v90_phase3_demod.cpp
 * @brief V90Phase3Demodulator: construction, destruction, reset(), the owner's event entry points and the getDecision() dispatcher.
 *
 * The class is declared in v90_phase3_demod.h; the two state machines it dispatches to are in v90_phase3_demod_v90_decision.cpp and
 * v90_phase3_demod_v92_decision.cpp.  This file holds everything else:
 * - the constructor builds the members in the order modulator, differential decoder (state 0), descrambler (0x12, 0x17, 0x63), then stores the parameter
 *   block, session flag and detector, heap-allocates the Sd detector (configured from the four SD_DETECTOR_* words of the parameter block, which are passed
 *   as raw words so that NaN payloads survive) and the ANSam tone detector (fixed parameters), and ends with reset(PCM_MU_LAW, 0x40, P3D_WAIT_FOR_SD, ...);
 *   the destructor destroys and frees both detectors without clearing anything;
 * - reset() restarts the phase: clears the counters and flags, resets the descrambler, the Jd objects, the Sd detector, the impairment detector, prints which
 *   initial state was chosen, restarts the modulator in its TRN1d mode, and computes the probing DIL length;
 * - the owner's entry points: clearVerificationStatus(), setSessionFlag(), exitDIL(), setDigitalImairmentsInfo(), enterWaitForANSpcmDrop(), getMaxUcode();
 * - getDecision(), which forwards the sample to the V.92 or V.90 state machine according to session_;
 * - functions with no caller in the library: resetJdNotDetector(), incrementFramePosition(), setAltRbsParams(), JdNotDetector(), twoLevelDemod() (the decision
 *   functions carry the same logic inline).
 *
 * Callers: V90Demodulator (construction, reset, owner entry points) and V90Equalizer::process() (getDecision()).
 */

#include "v90_phase3_demod.h"
#include "core/dsp_asm.h"
#include "core/dsp_host.h"
#include "v34/v34_api.h"

extern "C"
{
}

typedef P34_V90Phase3Demodulator D;
typedef P18_V90AutoDigitalImpDetector Adid;

/* ------------------------------------------------------------------------------------------------ helpers */

/** @brief A float argument as its raw 32-bit word, kept in an integer register so that a signalling NaN is not quieted by a floating-point load and store. */
static inline __attribute__((always_inline)) uint32_t wordOf(float v)
{
  uint32_t w;
  __builtin_memcpy(&w, &v, sizeof w);
  OPTIMIZER_HIDE_VAR(w);
  return w;
}
/** @brief The float with the given raw 32-bit word. */
static inline __attribute__((always_inline)) float floatOfWord(uint32_t w)
{
  float f;
  __builtin_memcpy(&f, &w, sizeof f);
  return f;
}
#include "core/dsp_floatbits.h"
/** @brief The raw word of a float parameter, read in place (a by-value copy through the floating-point unit could quiet a signalling NaN). */
#define PARAM_WORD(v)                                                                                                                                          \
  ({                                                                                                                                                           \
    uint32_t w_;                                                                                                                                               \
    __builtin_memcpy(&w_, &(v), sizeof w_);                                                                                                                    \
    OPTIMIZER_HIDE_VAR(w_);                                                                                                                                    \
    w_;                                                                                                                                                        \
  })

/** @brief A V90Parameters threshold float passed on unchanged (through its raw word) to the V90SdDetector constructor. */
static inline float paramWordF(const float* p) { return floatOfWord(wordOf(*p)); }

/** @brief The linear G.711 level of (code & 0x7f) in the law given by `pcm` (non-zero: A-law, code ^ 0xd5; else mu-law, code ^ 0xff). */
static inline unsigned short levelOf(unsigned pcm, unsigned char code)
{
  unsigned x = (unsigned)(code & 0x7f);
  if (pcm != 0)
    return (unsigned short)alaw2linear((int)(x ^ 0xd5u));
  return (unsigned short)ulaw2linear((int)(x ^ 0xffu));
}

/**
 * @brief Part of reset(): print which initial state was chosen (debug level above 1) and reset the learned level tables.
 * @note  The tables are reset for the TRN1d known-data start, the Sd start and any other value, but not for the quick-connect start (P3D_WAIT_QTS),
 *        which keeps the mapping of the previous session.
 */
static inline void initialStateNote(D* t, unsigned st)
{
  if (st == P3D_TRN1D_KNOWN)
  {
    if (dsplibs_debug_level > 1)
      dsplibs_debug_printf("V90Phase3Demodulator: initial state set to TRN1dKnownData\r\n");
    t->adid_->resetLinearMapping();
  }
  else if (st == P3D_WAIT_QTS)
  {
    if (dsplibs_debug_level > 1)
      dsplibs_debug_printf("V90Phase3Demodulator: initial state set to WaitForQTS\r\n");
  }
  else if (st == P3D_WAIT_FOR_SD)
  {
    if (dsplibs_debug_level > 1)
      dsplibs_debug_printf("V90Phase3Demodulator: initial state set to WaitForSd\r\n");
    t->adid_->resetLinearMapping();
  }
  else
  {
    if (dsplibs_debug_level > 1)
      dsplibs_debug_printf("V90Phase3Demodulator: initial state set to IRREGULAR !!!!! (%d)\r\n", (int)st);
    t->adid_->resetLinearMapping();
  }
}

/**
 * @brief The level decision of twoLevelDemod().
 * @param out  receives the bit (x > 0), before the differential decoder and the descrambler
 * @return the level at (phase_, code_) of the learned table (the alternate table for a robbed-bit phase once the study is done), negated when `x` is
 *         not positive
 */
static inline short pickLevel(D* t, float x, int* out)
{
  Adid* a = t->adid_;
  unsigned off = offsetof(Adid, linearMapping_);
  if (a->studyDone_ != 0)
  {
    if ((short)a->isAltRbs((short)t->phase_, (short)t->code_, x) != 0)
      off = offsetof(Adid, linearMappingAlt_);
  }
  unsigned idx = (((unsigned)(unsigned short)t->phase_) << 7) + t->code_;
  short v = *(const short*)((const char*)a + off + 2u * idx);
  if (x > 0.0f)
  {
    *out = 1;
    return v;
  }
  *out = 0;
  return (short)(-(int)v);
}

/**
 * @brief Constructor body: store the references, allocate and construct the Sd and ANSam detectors, then reset().
 * @warning The sysdep_malloc() results are not checked (placement new on NULL).
 */
static inline void ctorBody(D* t, V90Parameters* p, unsigned session, V90AutoDigitalImpDetector* adid)
{
  t->params_ = p;
  t->session_ = session;
  t->adid_ = adid;

  /* Sd detector: thresholds from the parameter block */
  V90SdDetector* sd = (V90SdDetector*)sysdep_malloc((unsigned)sizeof(V90SdDetector));
  uint32_t w4 = (uint32_t)t->params_->sdDetectorDetectionCounterThreshold;
  float c = paramWordF(&t->params_->sdDetectorNegativeCorrThreshold);
  float b = paramWordF(&t->params_->sdDetectorPositiveCorrThreshold);
  float a = paramWordF(&t->params_->sdDetectorEnergyThreshold);
  new (sd) V90SdDetector(a, b, c, w4);
  t->sd_ = sd;

  /* ANSam tone detector with its fixed parameters, then the initial reset */
  ANSamToneDetector* ans = (ANSamToneDetector*)sysdep_malloc((unsigned)sizeof(ANSamToneDetector));
  new (ans) ANSamToneDetector(0x190, 100, 307200.0f, 0, 0.5f, 8000, 0x32, 0x63);
  t->ans_ = ans;
  t->reset(PCM_MU_LAW, 0x40, P3D_WAIT_FOR_SD, 0, 0, 0, 0, 0, 1, 0.0f, 0);
}

/** @brief Destructor body: destroy and free the Sd and ANSam detectors if present (the pointers are not cleared). */
static inline void dtorBody(D* t)
{
  V90SdDetector* sd = t->sd_;
  if (sd != 0)
  {
    sd->~V90SdDetector();
    sysdep_free(sd);
  }
  ANSamToneDetector* ans = t->ans_;
  if (ans != 0)
  {
    ans->~ANSamToneDetector();
    sysdep_free(ans);
  }
}

/* ------------------------------------------------------------------------------------------------ construction / destruction */

/** See ctorBody(); the modulator, differential decoder and descrambler members are constructed by the initializer list. */
V90Phase3Demodulator::V90Phase3Demodulator(V90Parameters* p, V90SpectralVerifier*, unsigned session, V90AutoDigitalImpDetector* adid)
    : mod_(p, session), sdd_(), descr_(0x12, 0x17, 0x63)
{
  ctorBody(this, p, session, adid);
}

/** See dtorBody(). */
V90Phase3Demodulator::~V90Phase3Demodulator() { dtorBody(this); }

/* ------------------------------------------------------------------------------------------------ the methods */

/**
 * Restart phase 3 in state `st`.  Clears the event, counters and flags, stores the arguments, resets the descrambler, the Jd objects (when non-NULL), the Sd
 * detector and the impairment detector, prints the initial state, restarts the modulator in its TRN1d mode (with `n` as start count only for
 * P3D_TRN1D_KNOWN) and computes dilLen_ from the DIL descriptor.
 * @warning `dil` is passed to calculateDilLength() and `adid_` / `sd_` are used unchecked.  The floating-point `arg10` is stored as its raw word.
 */
void V90Phase3Demodulator::reset(PcmType pcm, unsigned char code, Phase3DemodulatorState st, unsigned n, V90Jd* jd, V92Jd* jd92, tagV90DILdescriptor* dil,
                                 short arg8, short arg9, float arg10, unsigned timeBase)
{
  if (dsplibs_debug_level > 1)
    dsplibs_debug_printf("V90Phase3Demodulator: Reset called, sessionFlag = %d !\r\n", (int)session_);

  /* scalar state; level_ is the linear level of the reference ucode in the law of the argument */
  sdToneSeen_ = 0;
  arg9_ = (unsigned short)arg9;
  event_ = 0;
  pcm_ = (unsigned)pcm;
  timeBase_ = timeBase;
  storeWord(&arg10_, PARAM_WORD(arg10));
  cnt_ = n;
  state_ = (unsigned)st;
  unsigned short lv = levelOf((unsigned)pcm, code);
  level_ = lv;
  sdd_.state_ = 0;
  code_ = code;
  dil_ = dil;

  /* decoders and Jd detectors */
  descr_.reset(0);
  jd_ = jd;
  if (jd != 0)
    jd->unPackReset();
  jd92_ = jd92;
  if (jd92 != 0)
  {
    jd92->unPackJdReset();
    jd92_->unPackJdPhaseReset();
  }

  /* tone detector and impairment detector, flags */
  V90SdDetector* sd = sd_;
  jdNotCnt_ = 0;
  sd->reset();
  adid_->reset(code, (PcmType)pcm_, arg8);
  flag3f9_ = 0;
  phase_ = 0;
  cnt3fc_ = 0;
  flag400_ = 0;

  /* announce the initial state, restart the probing-signal generator, derive the probing DIL length */
  initialStateNote(this, (unsigned)st);
  mod_.reset(pcm, code, P3M_TRN1D, (unsigned)st == P3D_TRN1D_KNOWN ? n : 0u, 0, 0, 0, 0);
  status_ = 0;
  dilLen_ = (unsigned)calculateDilLength(dil_, (PcmType)pcm_);
  lineMode_ = 0;
}

/** Clear the Jd-not zero run counter.  No caller in the library. */
void V90Phase3Demodulator::resetJdNotDetector() { jdNotCnt_ = 0; }

/** verify_ = 0 (the quick-connect verification result is read by the modem after the ANSpcm demodulation). */
void V90Phase3Demodulator::clearVerificationStatus()
{
  if (dsplibs_debug_level > 1)
    dsplibs_debug_printf("V90Phase3Demodulator: clearVerificationStatus called\r\n");
  verify_ = 0;
}

/** Set the session flag (0 V.90, else V.92) here and in the modulator; the choice of state machine follows at the next getDecision(). */
void V90Phase3Demodulator::setSessionFlag(unsigned f)
{
  session_ = f;
  mod_.setSessionFlag(f);
}

/**
 * Called by the owner to cut the DIL short.  In the DIL states (first / second / third study, QC variants, error relaxation) it prints, tells the modulator
 * to exit DIL and, when the modulator then reports the end (event_ 6), terminates phase 3 (event PHASE3_TERMINATED).  In other states nothing happens.
 */
void V90Phase3Demodulator::exitDIL()
{
  unsigned s = state_;
  if (s == P3D_THIRD_STUDY || s == P3D_ERROR_RELAXATION || s == P3D_FIRST_STUDY || s == P3D_SECOND_STUDY || s == P3D_QC_FIRST_STUDY ||
      s == P3D_QC_SECOND_STUDY || s == P3D_QC_THIRD_STUDY)
  {
    edprintf("V90Phase3Demodulator: exit DIL\r\n");
    mod_.exitDIL();
    if (mod_.event_ == 6)
    {
      edprintf("V90Phase3Demodulator: Phase3 Terminated @ %d\r\n", (int)cnt_);
      state_ = P3D_TERMINATED;
      cnt_ = 0;
      event_ = V90DEMOD_STATE_PHASE3_TERMINATED;
    }
  }
}

/** Apply impairInfo_: determine the max ucode per phase, find the pad gain and apply it to the linear mapping (all in the impairment detector). */
void V90Phase3Demodulator::setDigitalImairmentsInfo()
{
  adid_->determineMaxUcode((short)impairInfo_);
  adid_->findPadGain();
  adid_->applyPadGainToLinMapp();
}

/** phase_ = (phase_ + 1) mod 6.  No caller in the library. */
void V90Phase3Demodulator::incrementFramePosition()
{
  unsigned p = phase_ + 1;
  phase_ = (p == 6) ? 0 : p;
}

/** Make the robbed-bit alternate mean-error update-ratio threshold the active one (word copy).  No caller in the library. */
void V90Phase3Demodulator::setAltRbsParams()
{
  copyWordF(&params_->phase4MeanErrorBefToAftUpdateRatioThresh, &params_->phase4MeanErrorBefToAftUpdateRatioThreshAltRbs);
}

/** @return the impairment detector's max-ucode array (not a copy) */
unsigned char* V90Phase3Demodulator::getMaxUcode() { return adid_->maxUcodeArray_; }

/**
 * The JdNot detector: counts consecutive zero bits in jdNotCnt_.
 * @return 1 when more than 11 zeros were seen and cnt_ % 72 == 12, else 0.  No caller in the library (the decision functions inline it).
 */
int V90Phase3Demodulator::JdNotDetector(int bit)
{
  if (bit != 0)
    jdNotCnt_ = 0;
  else
    jdNotCnt_++;
  if (jdNotCnt_ > 11)
  {
    if (cnt_ % 72u == 12u)
      return 1;
  }
  return 0;
}

/** Enter P3D_ANSPCM_WAIT (wait for the ANSpcm energy drop) and restart cnt_, unless already in that state. */
void V90Phase3Demodulator::enterWaitForANSpcmDrop()
{
  if (state_ != P3D_ANSPCM_WAIT)
  {
    edprintf("V90Phase3Demodulator: enter WaitForANSpcmDrop\r\n");
    state_ = P3D_ANSPCM_WAIT;
    cnt_ = 0;
  }
}

/**
 * Level decision plus the differential decoder and descrambler for sample `x`.
 * @param out  receives the descrambled bit
 * @return the decision level.  No caller in the library.
 */
int V90Phase3Demodulator::twoLevelDemod(float x, int& out)
{
  short r = pickLevel(this, x, &out);
  out = sdd_.process(out);
  out = descr_.process(out);
  return r;
}

/**
 * Dispatch one sample to getV92Decision() when session_ is non-zero, else to getV90Decision().  The float is forwarded bit for bit (a signalling NaN
 * stays one).
 * @return the decision symbol of the callee
 */
short V90Phase3Demodulator::getDecision(float x)
{
  if (session_ != 0)
    return getV92Decision(floatOfWord(wordOf(x)));
  return getV90Decision(floatOfWord(wordOf(x)));
}
