/**
 * @file vpcm_flo_modem.cpp
 * @brief VPcmFloModem: construction, CP / Ja bit sources, session set-up and reset, visual diagnostics, quick-connect line verification.
 *
 * Defines every VPcmFloModem member except the two block drivers (vpcm_flo_modem_run.cpp); the class is in vpcm_flo_modem.h.
 * - The constructor builds the sub-objects in member order (V90Modem, V92Modem, V92EchoCanceller, ANSamToneDetector, SineWave, GenericIIR) with fixed
 *   arguments, then clears the bookkeeping members (ctorBody()); the destructor is the compiler's reverse-order teardown.  The GenericIIR runs on two
 *   file-local coefficient tables (a 5-tap notch).
 * - getV90JaBits() / getV90CpBits() feed the V.34 module the Ja / CP / CPnot frames that the block drivers built into cpnotFrame_ / cpFrame_.
 * - setPcmSessionType(), externalReset(), setPhaseIIinfo(), getUinfoValue(), vPcmResetPhase3Modem() and enterPhase3() move the engine through the
 *   V.34 handshake phases; getUinfoValue() also computes the L2 probing levels (10 * log10(probe * 2^-14) + 60 dB, 21 of the 25 probe values).
 * - getConstellation() / getLinearEqualizer() / getDFE() fill caller buffers with diagnostic points; the imaginary parts are truncated extended products.
 * - qcLineVerification() is the quick-connect line verification: receive the ANSpcm, wait for its drop, send TONEq and then silence.
 *
 * Raw accesses worth knowing: the modem parameter block's byte 3 (bit 2 = restore the SAS detector), the V.34 INFO0 words read as `int` at byte offsets
 * 0x68, 0x6c, 0x84 + 4 i, 0x98 and 0x9c, and the phase 2 info records reached through V90Phase2InfoView / V92Phase2InfoView.
 *
 * The debug level is re-read before every print (an unsigned `level > 1` test); the x87 helpers (storeF, dsp_log10, dsp_trunc_i32) keep the
 * extended-precision intermediates of the diagnostics exact.
 */

#include "vpcm_flo_modem.h"
#include "core/dsp_math.h"
#include "core/dsp_host.h"
#include "v34/v34_api.h"
#include "vpcm_v34_api.h"

typedef long double LD;

/* ------------------------------------------------------------------------------------------------ debug and trace strings */
#define S_JA_FLAG "Ja Flag set to %d\r\n"
#define S_CP_FLAG "CP Flag set to %d\r\n"
#define S_CPNOT_FLAG "CPnot Flag set to %d\r\n"
#define S_L2 "L2[%d]    = %c%d.%05d\r\n"
#define S_L2_FINAL "Final L2 Probing signal:\r\n"
#define S_NOF_BITS "SetNofBitsPhase4 - nof bits each call = %d\r\n"
#define S_END_CP "End of CP #%d tx.... (terminateCp=%d, terminateCpNot=%d)\n"
#define S_CP_TERM "GetV90CpBits: Indicating CP termination !!!!\r\n"
#define S_MOVE_CPNOT "OK Time to move to CPNot TX...\r\n"
#define S_SESSION "VPcmFloModem: setting PCM session to V.%d\n"
#define S_EXT_RESET "V90_V34_Main: external reset called.\r\n"
#define S_REINIT "V90_V34_Main: reinitializing parameters.\r\n"
#define S_P2_LAW "P2 REPORT : A_LAW or MU_LAW %d ( 0 = Mu, 1 = A)\r\n"
#define S_P2_POINT "P2 REPORT : Tx power measurment point = %d ( 0 = digital modem terminal, 1 = codec output)\r\n"
#define S_P2_MAXTX "P2 REPORT : Max tx power = %d\r\n"
#define S_P2_SHORT "P2 REPORT : short phase2: local=%d , remote=%d\r\n"
#define S_P2_V92 "P2 REPORT : V92 capabilities: local=%d , remote=%d , selected=%d\r\n"
#define S_JA_LEN "VPcmFloModem: enterPhase3: Ja length = %d\r\n"
#define S_ENTER3 "VPcmFloModem: enterPhase3 called.\r\n"
#define S_QC_VERIFY "VPcmFloModem (QC LineVerify): Channel verification status is: %d\r\n"
#define S_QC_DEMOD_OVER "VPcmFloModem (QC LineVerify): ANSpcm demod over (after %d samples), start TONEq...\r\n"
#define S_QC_DEMOD_STATUS "VPcmFloModem (QC LineVerify): demodulator verification status is %d\r\n"
#define S_QC_DROP "VPcmFloModem (QC LineVerify): got ANSpcm drop detection, with no verification completion status !!!\r\n"
#define S_QC_SILENCE "VPcmFloModem (QC LineVerify): Silence after TONEq over, move to phase2...\r\n"
#define S_QC_TONE_OVER "VPcmFloModem (QC LineVerify): TONEq mod over (after %d samples), tx silence...\r\n"
#define S_QC_TONE_TERM "VPcmFloModem (QC LineVerify): TONEq termination requested, still bellow 50mS (nof samples = %d)...\r\n"

/* ------------------------------------------------------------------------------------------------ helpers (static inline) */

#include "core/dsp_debug.h"

#include "core/dsp_floatbits.h"

/** @brief Byte 3 of the modem parameter block; bit 2 = restore the SAS detector.  @param p parameter block  @return pointer to that byte */
static inline unsigned char* paramByte3(_tagModemParameters* p) { return &p->flags3; }
static inline unsigned char* modemParamByte3(V90Parameters* p) { return paramByte3(p->params); }

/** @brief Store the six V.34 rate flags {1, 0, 1, 1, 1, last}, one byte at a time.  @param m engine  @param last value of the last flag (1 = V.34) */
static inline void setBaudFlags(VPcmFloModem* m, unsigned char last)
{
  m->v34Flags_[0] = 1;
  m->v34Flags_[1] = 0;
  m->v34Flags_[2] = 1;
  m->v34Flags_[3] = 1;
  m->v34Flags_[4] = 1;
  m->v34Flags_[5] = last;
}

/** @brief CP bookkeeping reset shared by the idle sequences (resetBitPointer()): bit pointer, the four flag bytes and the sequence counter. */
static inline void clearCpFlags(VPcmFloModem* m)
{
  m->bitPtr_ = 0;
  m->jaFlag_ = 0;
  m->cpFlag_ = 0;
  m->cpnotFlag_ = 0;
  m->cpnotCopied_ = 0;
  m->cpSeqCnt_ = 0;
}

/**
 * @brief Clear the session state of enterPhase3(), externalReset() and internalReset().
 *
 * Clears the five constellation / reset bytes, sets the V.34 rate flags for V.90, clears the CP flag bytes and lengths, sets nofBitsPhase4_ to 2 and
 * minTxSeq_ to 1.
 */
static inline void clearSessionState(VPcmFloModem* m)
{
  m->trainConstel_ = 0;
  m->rrnConstel_ = 0;
  m->localRrn_ = 0;
  m->dropToV34_ = 0;
  m->clearDown_ = 0;
  setBaudFlags(m, 0);
  m->jaFlag_ = 0;
  m->cpFlag_ = 0;
  m->cpnotFlag_ = 0;
  m->cpnotCopied_ = 0;
  m->nofBitsPhase4_ = 2;
  m->cpnotLen_ = 0;
  m->cpLen_ = 0;
  m->bitPtr_ = 0;
  m->cpSeqCnt_ = 0;
  m->minTxSeq_ = 1;
}

/** @brief Copy the received MP fields (V90MP bytes and halves) into mpB_ / mpRate2_ / mpH_; the rate mask is doubled and stored as 16 bits. */
static inline void copyMpInfo(VPcmFloModem* m)
{
  m->mpB_[0] = m->v90_.mp_.type_;
  m->mpB_[1] = m->v90_.mp_.rate_;
  m->mpB_[2] = m->v90_.mp_.trellis_;
  m->mpB_[3] = m->v90_.mp_.nonLin_;
  m->mpB_[4] = m->v90_.mp_.shaping_;
  m->mpB_[5] = m->v90_.mp_.cPack_;
  m->mpRate2_ = (uint16_t)(2 * (int)(short)m->v90_.mp_.rateMask_);
  m->mpH_[0] = m->v90_.mp_.h1Re_;
  m->mpH_[1] = m->v90_.mp_.h1Im_;
  m->mpH_[2] = m->v90_.mp_.h2Re_;
  m->mpH_[3] = m->v90_.mp_.h2Im_;
  m->mpH_[4] = m->v90_.mp_.h3Re_;
  m->mpH_[5] = m->v90_.mp_.h3Im_;
}

/** @brief The V.90 / V.92 version number a session prints: 90 for session 0, 92 for any other value. */
static inline int sessionVersion(unsigned s) { return s == 0 ? 90 : 92; }

/* The two phase 2 info records through the View types; V90Modem::info_ / V92Modem::info_ are re-read at every use. */
static inline V90Phase2InfoView* info90(VPcmFloModem* m) { return reinterpret_cast<V90Phase2InfoView*>(m->v90_.info_); }
static inline V92Phase2InfoView* info92(VPcmFloModem* m) { return reinterpret_cast<V92Phase2InfoView*>(m->v92_.info_); }

/** @brief Extended-precision product truncated toward zero to 32 bits (0x80000000 for NaN or out of range). */
static inline int chopMul(LD a, LD b) { return dsp_trunc_i32(a * b); }

/** @brief Constructor body (everything after the six sub-object constructors): clear the frame buffers, flags and counters. */
static inline void ctorBody(VPcmFloModem* m, _tagModemParameters* params, V90ModemSide side)
{
  sysdep_memset(m->cpnotFrame_, 0, 0x1518);
  m->cpnotLen_ = 0;
  m->v34Flags_[0] = 1; /* all six rate flags start at 1 */
  m->v34Flags_[1] = 1;
  m->v34Flags_[2] = 1;
  m->v34Flags_[3] = 1;
  m->v34Flags_[4] = 1;
  m->v34Flags_[5] = 1;
  m->constelCtr_ = 0;
  sysdep_memset(m->cpFrame_, 0, 0xe10);
  m->jaFlag_ = 0;
  m->cpFlag_ = 0;
  m->cpnotFlag_ = 0;
  m->cpnotCopied_ = 0;
  m->nofBitsPhase4_ = 0;
  m->flag6119_ = 0;
  m->minTxSeq_ = 0;
  m->cpLen_ = 0;
  m->cpSeqCnt_ = 0;
  m->qcState_ = 0;
  m->qcFlag_ = 0;
  m->qcCount_ = 0;
  m->qcStatus_ = 0;
  *paramByte3(params) = (unsigned char)(*paramByte3(params) & 0xfb);
  sysdep_memset(m->ecOut_, 0, 0x350);
  m->flag6120_ = (side == V90_SIDE_ANALOG) ? 1u : 0u;
  m->v92Session_ = 0;
  m->state_ = 0;
  m->entranceFilter_ = 0;
  m->echoMode_ = 0;
  m->echoCnt_ = 0;
}

/* Coefficients of the entrance filter's 5-tap notch (GenericIIR iir_): numerator and denominator (writable file-local tables the filter is built on). */
static double s_iirNum[5] = {0.962843309849182, -3.851350039011478, 5.777013458394471, -3.851350039011477, 0.9628433098491815};
static double s_iirDen[5] = {1.0, -3.924251435170327, 5.775633188131097, -3.77844829349304, 0.9270672393213297};

/* ------------------------------------------------------------------------------------------------ constructor / destructor */

/** @brief Construct the engine: the six sub-objects in member order with their fixed parameters, then ctorBody().  See the class for the parameters. */
VPcmFloModem::VPcmFloModem(void* ctx, V90ModemSide side, _tagModemParameters* params, unsigned blk, V90ComputationalMode mode90, V92ComputationalMode mode92)
    : ctx_(ctx), v90_(side, params, &dil_, blk, mode90, 1u), v92_((side == V90_SIDE_ANALOG) ? V92_SIDE_ANALOG : V92_SIDE_NONE, params, blk, &dil_, mode92),
      ec_(v92_.params_, blk, 199u), ansam_(0x1770u, 0x1c2u, 250000.0f, 1u, 0.58f, 9600u, 0x32u, 0x63u), sine_(4800.0f, 980.0f, 0.0f, 9600.0f),
      iir_(5u, 5u, s_iirDen, s_iirNum, 99u)
{
  ctorBody(this, params, side);
}

VPcmFloModem::~VPcmFloModem() {}

/* ------------------------------------------------------------------------------------------------ one-line setters (no callers in the library) */

/** @brief Set the Ja-termination flag and trace it.  @param f new value of jaFlag_ */
void VPcmFloModem::setTerminateJaFlag(unsigned char f)
{
  jaFlag_ = f;
  if (dbgOn())
    dsplibs_debug_printf(S_JA_FLAG, (int)f);
}

/** @brief Set the CP flag and trace it.  @param f new value of cpFlag_ */
void VPcmFloModem::setTerminateCpFlag(unsigned char f)
{
  cpFlag_ = f;
  if (dbgOn())
    dsplibs_debug_printf(S_CP_FLAG, (int)f);
}

/** @brief Set the CPnot-termination flag and trace it.  @param f new value of cpnotFlag_ */
void VPcmFloModem::setTerminateCpNotFlag(unsigned char f)
{
  cpnotFlag_ = f;
  if (dbgOn())
    dsplibs_debug_printf(S_CPNOT_FLAG, (int)f);
}

/** @brief Set the minimum number of CP sequences before the CPnot and restart the sequence counter.  @param n new minTxSeq_ */
void VPcmFloModem::setMinNofTransmitSequences(unsigned short n)
{
  cpSeqCnt_ = 0;
  minTxSeq_ = n;
}

/** @brief Choose the CP bits per call from the training constellation (2 for constellation 0, else 4) and trace it.  @param constel training constellation */
void VPcmFloModem::setNofBitsPhase4(unsigned constel)
{
  nofBitsPhase4_ = (unsigned char)(constel == 0 ? 2 : 4);
  if (dbgOn())
    dsplibs_debug_printf(S_NOF_BITS, (int)nofBitsPhase4_);
}

void VPcmFloModem::resetBitPointer() { clearCpFlags(this); }
void VPcmFloModem::setV34BaudForV90() { setBaudFlags(this, 0); }
void VPcmFloModem::setV34BaudForV34() { setBaudFlags(this, 1); }
void VPcmFloModem::internalReset() { clearSessionState(this); }
void VPcmFloModem::copyMpInfoForInterface() { copyMpInfo(this); }

/* ------------------------------------------------------------------------------------------------ the CP / Ja bit sources */

/**
 * @brief Produce the next Ja word: two consecutive bits of the CPnot frame, the second in bit 1.
 * @param out receives the 2-bit word (left unchanged while cpnotLen_ is 0)
 * @return 1 when jaFlag_ is set (the Ja transmission is to end; the bit pointer is rewound), else 0
 * @note  The bit pointer wraps to 0 when it reaches cpnotLen_.
 */
int VPcmFloModem::getV90JaBits(short* out)
{
  int ret = 0;
  if (cpnotLen_ != 0)
  {
    /* two bits per call */
    unsigned idx = bitPtr_;
    bitPtr_ = (uint16_t)(idx + 1);
    *(uint16_t*)out = (uint16_t)cpnotFrame_[idx];
    idx = bitPtr_;
    bitPtr_ = (uint16_t)(idx + 1);
    int v = cpnotFrame_[idx];
    *(uint16_t*)out = (uint16_t)((v + v) | (int)*(uint16_t*)out);
    if ((int)bitPtr_ == (int)(short)cpnotLen_)
      bitPtr_ = 0;
  }

  if (jaFlag_ != 0)
  {
    ret = 1;
    bitPtr_ = 0;
  }
  return ret;
}

/**
 * @brief Produce the next group of CP bits (nofBitsPhase4_ of them, first bit in bit 0) and track the CP sequence.
 *
 * When the last bit of the frame has been sent the sequence counter is incremented and the evaluator's delayed-retrain enable is refreshed; then, if
 * cpnotFlag_ is set, the transmission ends (return 1); otherwise, if the CP has completed and the minimum number of sequences was sent, the CPnot
 * frame replaces the CP frame once (cpnotCopied_).
 * @param out receives the bit group (not written while cpLen_ is 0)
 * @return 1 when the CPnot transmission is to end, else 0
 * @warning  The end of the frame is detected by bitPtr_ == cpLen_ after a whole group.  If cpLen_ is not a multiple of nofBitsPhase4_ the
 *           equality never holds and bitPtr_ walks past the 1800-element cpFrame_ (it wraps only at 16 bits).  getV90JaBits() has the same
 *           hazard with an odd cpnotLen_.  The lengths come from V90CPPacker() / DILdescriptorPacker() and are not checked here.
 */
int VPcmFloModem::getV90CpBits(short* out)
{
  int ret = 0;
  if (cpLen_ == 0)
    return 0;

  /* gather the bit group */
  *out = 0;
  for (unsigned short c = 0; c < (unsigned)nofBitsPhase4_; c++)
  {
    unsigned idx = bitPtr_;
    bitPtr_ = (uint16_t)(idx + 1);
    int v = cpFrame_[idx];
    *(uint16_t*)out = (uint16_t)((int)((unsigned)v << (c & 31u)) | (int)*(uint16_t*)out);
  }
  if ((int)bitPtr_ != (int)(short)cpLen_)
    return 0;

  /* the CP is complete */
  unsigned cnt = (unsigned)cpSeqCnt_ + 1u;
  V90ConnectionEvaluator* ce = v90_.dem_->ce_;
  int saved = ce->delayedRetrainRequest_;
  cpSeqCnt_ = (uint16_t)cnt;
  ce->delayedRetrainEnable_ = saved; /* the pending delayed-retrain request becomes the enable */
  if (dbgOn())
    dsplibs_debug_printf(S_END_CP, (int)cpSeqCnt_, (int)cpFlag_, (int)cpnotFlag_);

  if (cpnotFlag_ != 0)
  {
    ret = 1;
    if (dbgOn())
      dsplibs_debug_printf(S_CP_TERM);
  }
  else if (cpFlag_ != 0 && cpnotCopied_ == 0 && cpSeqCnt_ >= minTxSeq_)
  {
    /* switch the transmission over to the CPnot frame */
    if (dbgOn())
      dsplibs_debug_printf(S_MOVE_CPNOT);
    unsigned short n = cpnotLen_;
    for (unsigned short i = 0; (int)i < (int)(short)n; i++)
      cpFrame_[i] = cpnotFrame_[i];
    cpLen_ = n;
    cpnotCopied_ = 1;
    cpSeqCnt_ = 0;
  }
  bitPtr_ = 0;
  return ret;
}

/* ------------------------------------------------------------------------------------------------ session / reset */

/**
 * @brief Select a V.90 or V.92 session.
 * @param session 0 = V.90, anything else = V.92
 * @note  Stores the session word, the local V.92 capability in the V.92 phase 2 info record, and forwards the flag to the V.90 modem.
 */
void VPcmFloModem::setPcmSessionType(int session)
{
  edprintf(S_SESSION, sessionVersion((unsigned)session));
  V92Phase2InfoView* i = info92(this);
  v92Session_ = (session != 0) ? 1u : 0u;
  i->v92CapLocal_ = (unsigned char)session;
  v90_.setSessionFlag((unsigned)session);
}

/**
 * @brief Reset the engine for a new V.34 session.
 * @note  Re-initialises both parameter blocks and the session state, re-initialises the V.90 demodulator on the analog side, and clears the
 *        quick-connect line verification state.
 */
void VPcmFloModem::externalReset()
{
  /* parameters and flags */
  setBaudFlags(this, 0);
  v90_.params_->initSession();
  v90_.params_->init();
  v92_.params_->init();
  if (dbgOn())
    dsplibs_debug_printf(S_REINIT);
  clearSessionState(this);
  if (flag6120_ != 0)
    v90_.dem_->reInit();

  /* engine state */
  state_ = 0;
  flag6119_ = 0;
  qcState_ = 0;
  qcFlag_ = 0;
  qcCount_ = 0;
  qcStatus_ = 0;
  if (dbgOn())
    dsplibs_debug_printf(S_EXT_RESET);
}

/**
 * @brief Translate the V.34 INFO0 words into the V.90 and V.92 phase 2 information records and publish the uinfo / L2 arrays through them.
 * @param p   INFO0 word block from the V.34 module (words 0x68 / 0x6c: short phase 2 and V.92 capability flags, 0x84 + 4 i: maximum transmit power
 *            bits, 0x98: transmit power measurement point, 0x9c: A-law flag)
 * @param rtd round-trip delay, stored in both records
 * @note  Ends by calling setPcmSessionType() with the current v92Session_.  The two flag words at 0x68 / 0x6c swap roles between the analog and the
 *        digital side.
 */
void VPcmFloModem::setPhaseIIinfo(int* p, int rtd)
{
  /* maximum transmit power: weighted sum of five flag words */
  int w[7];
  w[0] = 1;
  w[1] = 2;
  w[2] = 4;
  w[3] = 8;
  w[4] = 0x10;
  w[5] = 0x20;
  w[6] = 0x40;
  int sum = 0;
  for (int i = 0; i <= 4; i++)
    sum = (int)((unsigned)sum + (unsigned)p[0x84 / 4 + i] * (unsigned)w[i]);
  info90(this)->maxTx_ = (unsigned char)sum;

  /* V.90 record: power measurement point and companding law (analog side only), then mirror both into the V.92 record */
  if (flag6120_ != 0)
  {
    V90Phase2InfoView* a = info90(this);
    a->txPoint_ = (p[0x98 / 4] != 0) ? 1u : 0u;
    a->pcmType_ = (p[0x9c / 4] != 0) ? 1u : 0u;
  }
  info92(this)->maxTx_ = info90(this)->maxTx_;
  {
    V90Phase2InfoView* a = info90(this);
    V92Phase2InfoView* b = info92(this);
    b->txPoint_ = a->txPoint_;
    b->pcmType_ = a->pcmType_;
  }

  /* remote short phase 2 and V.92 capability */
  if (flag6120_ != 0)
  {
    info92(this)->shortPhase2Remote_ = (unsigned char)p[0x68 / 4];
    info92(this)->v92CapRemote_ = (unsigned char)p[0x6c / 4];
  }
  else
  {
    info92(this)->shortPhase2Remote_ = (unsigned char)p[0x6c / 4];
    info92(this)->v92CapRemote_ = (unsigned char)p[0x68 / 4];
  }

  /* report (each print re-tests the debug level) */
  if (dbgOn())
  {
    dsplibs_debug_printf(S_P2_LAW, (int)(info90(this)->pcmType_ == 1u));
    if (dbgOn())
    {
      dsplibs_debug_printf(S_P2_POINT, p[0x98 / 4]);
      if (dbgOn())
      {
        dsplibs_debug_printf(S_P2_MAXTX, (int)info90(this)->maxTx_);
        if (dbgOn())
        {
          dsplibs_debug_printf(S_P2_SHORT, (int)info92(this)->shortPhase2Local_, (int)info92(this)->shortPhase2Remote_);
          if (dbgOn())
            dsplibs_debug_printf(S_P2_V92, (int)info92(this)->v92CapLocal_, (int)info92(this)->v92CapRemote_, sessionVersion(v92Session_));
        }
      }
    }
  }

  /* publish the uinfo / L2 arrays and the round-trip delay through both records */
  {
    V90Phase2InfoView* a = info90(this);
    a->arrB_ = uinfoB_;
    a->arrD_ = uinfoD_;
    V92Phase2InfoView* b = info92(this);
    a->l2_ = l2_;
    a->arrA_ = uinfoA_;
    b->arrA_ = uinfoA_;
    uint32_t* keep = a->arrB_;
    a->rtd_ = (uint32_t)rtd;
    b->rtd_ = (uint32_t)rtd;
    b->arrB_ = keep;
    b->l2_ = a->l2_;
    b->arrD_ = a->arrD_;
  }
  setPcmSessionType((int)v92Session_);
}

/**
 * @brief Compute the L2 probing levels and publish the phase 2 information after the V.34 probing.
 *
 * Clears the four 21-entry arrays.  When x is 0 it turns the 25 probe results of the V.34 module into dB levels
 * (L2 = 10 * log10(probe * 2^-14) + 60, every intermediate rounded to float), keeps the 21 flagged ones in l2_ and, with debugging on, prints the
 * differences l2_[14] - l2_[i] for i = 15 .. 20.  Then it hands the INFO0 words and the round-trip delay to setPhaseIIinfo().
 * @param x non-zero skips the L2 computation
 * @return the uinfo value from the parameter block, or 0 when dropToV34_ is set or the block holds 0 (the V.34 rate flags are then set for V.34)
 */
int VPcmFloModem::getUinfoValue(short x)
{
  /* which of the 25 probe values the L2 table keeps (21 of them) */
  static const unsigned char keep[25] = {1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1};
  double* probe = (double*)V34XF_GetProbeResultsPtr(ctx_);
  for (int i = 0; i <= 20; i++)
  {
    uinfoA_[i] = 0;
    uinfoB_[i] = 0;
    *(volatile float*)&l2_[i] = 0.0f;
    uinfoD_[i] = 0;
  }

  if (x == 0)
  {
    /* L2 levels */
    int k = 0;
    for (int i = 0; i <= 24; i++)
    {
      float slot;
      LD scaled = (LD)probe[i] * (LD)6.103515625e-05f; /* 2^-14 */
      float f = storeF(&slot, scaled);
      float lg = storeF(&slot, dsp_log10((LD)f));
      LD level = (LD)lg * (LD)10.0f + (LD)60.0f;
      if (keep[i] != 0)
      {
        storeF(&l2_[k], level);
        k++;
      }
    }

    /* trace the last six levels relative to l2_[14] as sign, integer and 5-digit fraction */
    if (dbgOn())
      dsplibs_debug_printf(S_L2_FINAL);
    for (int i = 0xf; i <= 0x14; i++)
    {
      if (dbgOn())
      {
        LD v = (LD)l2_[14] - (LD)l2_[i];
        int ipart = dsp_trunc_i32(__builtin_fabsl(v));
        int whole = dsp_trunc_i32(v);
        int frac = dsp_trunc_i32((v - (LD)whole) * (LD)100000.0f);
        unsigned m = (unsigned)(frac >> 31); /* branch-free absolute value (INT_MIN stays) */
        int ufrac = (int)(((unsigned)frac ^ m) - m);
        int sign = (v <= 0.0L) ? '-' : '+';
        dsplibs_debug_printf(S_L2, i, sign, ipart, ufrac);
      }
    }
  }

  /* hand the INFO0 words over, then pick the result */
  int rtd = V34XF_GetRTD(ctx_);
  int* info0 = (int*)V34XF_GetInfo0BitsPtr(ctx_);
  setPhaseIIinfo(info0, rtd);
  if (dropToV34_ == 0)
  {
    int u = (int)(short)v90_.params_->phase2InfoUinfo;
    if (u != 0)
      return u;
  }
  setBaudFlags(this, 1);
  return 0;
}

/**
 * @brief Reset the engine for a new phase 3: parameters, both modems, the echo canceller, the echo-silence state and the TONEq phase.
 * @note  Sets state_ to 1 and clears the restore-SAS bit of the parameter block.
 */
void VPcmFloModem::vPcmResetPhase3Modem()
{
  constelCtr_ = 0;
  v90_.params_->init();
  v90_.setSessionFlag(v92Session_);
  v90_.reset(0);
  v92_.reset();
  ec_.reset();
  echoCnt_ = 0;
  echoMode_ = 0;
  {
    unsigned char* b = modemParamByte3(v90_.params_);
    *b = (unsigned char)(*b & 0xfb);
  }
  *(volatile float*)&sine_.phase_ = 0.0f;
  state_ = 1;
}

/**
 * @brief Enter phase 3 on the V.34 module's Ja indication: clear the session state, restart the demodulator for phase 3 and pack the DIL descriptor
 *        into cpnotFrame_ (its length goes to cpnotLen_).
 */
void VPcmFloModem::enterPhase3()
{
  clearSessionState(this);
  if (dbgOn())
    dsplibs_debug_printf(S_ENTER3);
  v90_.dem_->enterPhase3();
  DILdescriptorPacker(&dil_, cpnotFrame_, (short*)&cpnotLen_);
  edprintf(S_JA_LEN, (int)(short)cpnotLen_);
}

/* ------------------------------------------------------------------------------------------------ visual diagnostics */

/**
 * @brief Constellation points of the last demodulator block.
 *
 * The imaginary part is 1.7 * the equalizer output sample.  The real part is synthesised from the running counter constelCtr_ (in phase 3 a sweep
 * of 750 columns, 35 units apart starting at -14000, advancing every 5th point; in the other phases a 100-column sweep advancing every 15th point
 * and offset by 140 columns per position of the 6-symbol frame).  Only VPcmV34GetVisualDiagnostics() calls it, and nothing in the library calls that.
 * @param out buffer of at least `max` points
 * @param max capacity of `out`
 * @return number of points written (0 on the digital side of a V.92 session)
 */
int VPcmFloModem::getConstellation(int_complex* out, unsigned long max)
{
  if (v92Session_ != 0 && flag6120_ == 0)
    return 0;
  V90Demodulator* d = v90_.dem_;
  unsigned n = d->eqCnt_;
  unsigned fp = d->framePos_;
  float* buf = d->eqBuf_;
  if ((unsigned long)n > max)
    n = (unsigned)max;
  if (d->phase_ == V90D_PHASE_3)
  {
    for (unsigned i = 0; i < n; i++)
    {
      int c = constelCtr_;
      int q = c / 5;
      constelCtr_ = (int)((unsigned)c + 1u);
      int r = q % 750;
      out[i].re_ = 35 * r - 14000;
      out[i].im_ = chopMul((LD)1.7, (LD)buf[i]);
    }
  }
  else
  {
    for (unsigned i = 0; i < n; i++)
    {
      int c = constelCtr_;
      int q = c / 15;
      constelCtr_ = (int)((unsigned)c + 1u);
      int r = q % 100;
      unsigned t = (fp + i) % 6u;
      int e = r + (int)(t * 140u);
      out[i].re_ = 35 * e - 14000;
      out[i].im_ = chopMul((LD)1.7, (LD)buf[i]);
    }
  }
  return (int)n;
}

/**
 * @brief Linear equalizer taps of the V.90 demodulator as diagnostic points: real part 0, imaginary part the tap times 10000 (truncated).
 * @param out buffer of at least `max` points
 * @param max capacity of `out`
 * @return number of points written (0 on the digital side of a V.92 session)
 */
int VPcmFloModem::getLinearEqualizer(int_complex* out, unsigned long max)
{
  if (v92Session_ != 0 && flag6120_ == 0)
    return 0;
  unsigned n = v90_.dem_->eq_->linLen_;
  if ((unsigned long)n > max)
    n = (unsigned)max;
  const float* coef = v90_.dem_->eq_->linCoef_;
  for (unsigned i = 0; i < n; i++)
  {
    out[i].re_ = 0;
    out[i].im_ = chopMul((LD)coef[i], (LD)10000.0f);
  }
  return (int)n;
}

/**
 * @brief Decision-feedback equalizer taps of the V.90 demodulator as diagnostic points, formatted like getLinearEqualizer().
 * @param out buffer of at least `max` points
 * @param max capacity of `out`
 * @return number of points written (0 on the digital side of a V.92 session)
 */
int VPcmFloModem::getDFE(int_complex* out, unsigned long max)
{
  if (v92Session_ != 0 && flag6120_ == 0)
    return 0;
  unsigned n = v90_.dem_->eq_->dfeLen_;
  if ((unsigned long)n > max)
    n = (unsigned)max;
  const float* coef = v90_.dem_->eq_->dfeCoef_;
  for (unsigned i = 0; i < n; i++)
  {
    out[i].re_ = 0;
    out[i].im_ = chopMul((LD)coef[i], (LD)10000.0f);
  }
  return (int)n;
}

/* ------------------------------------------------------------------------------------------------ the quick-connect line verification */

/**
 * @brief One block of the quick-connect line verification.
 *
 * Runs the V.90 receiver on the block and follows the demodulator's state: when the ANSpcm demodulation is over it records the channel verification
 * status and waits for the drop (qcState_ 1); when the drop is detected it starts TONEq (after at least 0x1e0 samples) or notes that the tone must
 * end early; while qcState_ is 1 it writes the 980 Hz TONEq tone, otherwise (state 2) silence.  After the tone has run 0x1e0 samples it restarts
 * the counter at -0x180 and enters state 2, and when that counter reaches 0 the verification is over.
 * @param in     input samples, n floats
 * @param out    receives n output samples (tone or silence)
 * @param n      block length in samples
 * @param bits   receives the demodulated bits
 * @param nBits  set to 0 on return
 * @param bits2  unused
 * @param nBits2 set to 0 on return
 * @return 1 when the silence after TONEq is over (the session moves to phase 2), else 0
 */
int VPcmFloModem::qcLineVerification(float* in, float* out, unsigned n, int* bits, int* nBits, int* bits2, int* nBits2)
{
  int ret = 0;
  v90_.progress(bits, reinterpret_cast<unsigned&>(*nBits), in, n);
  int code = (int)v90_.dem_->state_;
  qcCount_ = (int)((unsigned)qcCount_ + n);
  int count = qcCount_;
  unsigned st;
  bool genTone = false;

  /* follow the demodulator state */
  if (code == V90DEMOD_STATE_ANSPCM_DEMOD_OVER)
  { /* ANSpcm demodulation over: wait for the drop */
    if (dbgOn())
      dsplibs_debug_printf(S_QC_DEMOD_OVER, count);
    qcStatus_ = (unsigned short)v90_.dem_->p3_->verify_;
    if (dbgOn())
      dsplibs_debug_printf(S_QC_VERIFY, (int)qcStatus_);
    qcState_ = 1;
    qcFlag_ = 0;
    qcCount_ = 0;
    v90_.dem_->p3_->enterWaitForANSpcmDrop();
    st = qcState_;
  }
  else if (code == V90DEMOD_STATE_ANSPCM_DROP_DETECTED)
  { /* ANSpcm drop detected */
    st = qcState_;
    if (st == 1)
    {
      if (count > 0x1df)
      {
        if (dbgOn())
          dsplibs_debug_printf(S_QC_TONE_OVER, count);
        qcCount_ = -0x180;
        qcState_ = 2;
        st = 2;
      }
      else
      {
        if (dbgOn())
        {
          dsplibs_debug_printf(S_QC_TONE_TERM, count);
          st = qcState_;
        }
        qcFlag_ = 1;
      }
    }
    else
    {
      if (dbgOn())
        dsplibs_debug_printf(S_QC_DROP);
      qcStatus_ = (unsigned short)v90_.dem_->p3_->verify_;
      if (dbgOn())
        dsplibs_debug_printf(S_QC_DEMOD_STATUS, (int)qcStatus_);
      qcCount_ = 0;
      qcState_ = 1;
      qcFlag_ = 1;
      genTone = true;
    }
  }
  else
  {
    st = qcState_;
  }

  /* produce the output block: TONEq while qcState_ is 1, else silence */
  if (genTone || st == 1)
  {
    sine_.generate(out, n);
    if (qcFlag_ != 0 && qcCount_ > 0x1df)
    {
      if (dbgOn())
        dsplibs_debug_printf(S_QC_TONE_OVER, qcCount_);
      qcCount_ = -0x180;
      qcState_ = 2;
    }
  }
  else
  {
    for (unsigned i = 0; i < n; i++)
      ((int*)out)[i] = 0; /* all-zero bits are 0.0f */
    if (st == 2 && qcCount_ >= 0)
    {
      if (dbgOn())
        dsplibs_debug_printf(S_QC_SILENCE);
      ret = 1;
    }
  }
  *nBits = 0;
  *nBits2 = 0;
  (void)bits2;
  return ret;
}

/*
 * Compile-time tie between the C prefix views of src/v34/v34_engine_views.h (v34_api.c walks ctx->v90_.mod_ / v92_.mod_ / v90_.dem_ through them) and
 * the real engine classes, checked on whichever word size this file is built: every member of every view must sit where the class has it.  These are
 * declarations only (no code, no data).
 */
extern "C"
{
#include "v34/v34_engine_views.h"
}
#define V34ENG_TIE(V, C, vm, cm) DSP_STATIC_ASSERT(offsetof(V, vm) == offsetof(C, cm), #V "." #vm " != " #C "::" #cm)
V34ENG_TIE(V90ModulatorPfx, V90Modulator, info_, info_);
V34ENG_TIE(V90ModulatorPfx, V90Modulator, params_, params_);
V34ENG_TIE(V90ModulatorPfx, V90Modulator, session_, session_);
V34ENG_TIE(V90ModulatorPfx, V90Modulator, state_, state_);
V34ENG_TIE(V90ModulatorPfx, V90Modulator, event_, event_);
V34ENG_TIE(V90ModulatorPfx, V90Modulator, p3_, p3_);
V34ENG_TIE(V90ModulatorPfx, V90Modulator, p4_, p4_);
V34ENG_TIE(V90ModulatorPfx, V90Modulator, b2s_, b2s_);
V34ENG_TIE(V92ModulatorPfx, V92Modulator, codeA_, codeA_);
V34ENG_TIE(V92ModulatorPfx, V92Modulator, info_, info_);
V34ENG_TIE(V92ModulatorPfx, V92Modulator, cp_, cp_);
V34ENG_TIE(V92ModulatorPfx, V92Modulator, skew_, skew_);
V34ENG_TIE(V92ModulatorPfx, V92Modulator, state_, state_);
V34ENG_TIE(V92ModulatorPfx, V92Modulator, splitAt_, splitAt_);
V34ENG_TIE(V92ModulatorPfx, V92Modulator, params_, params_);
V34ENG_TIE(V92ModulatorPfx, V92Modulator, b2s_, b2s_);
V34ENG_TIE(V90BitsToSymbolPfx, V90BitsToSymbol, mapper_, mapper_);
V34ENG_TIE(V92BitsToSymbolPfx, V92BitsToSymbol, tx_, tx_);
V34ENG_TIE(V90MapperPfx, V90Mapper, params_, params_);
V34ENG_TIE(V90MapperPfx, V90Mapper, w0_, w0_);
V34ENG_TIE(V92TransmitterPfx, V92Transmitter, w0_, w0_);
V34ENG_TIE(V90DemodulatorPfx, V90Demodulator, ce_, ce_);
V34ENG_TIE(V90ConnectionEvaluatorPfx, V90ConnectionEvaluator, w04_, nofRetrainsSoFar_);
V34ENG_TIE(V90ConnectionEvaluatorPfx, V90ConnectionEvaluator, externalCommand_, externalCommand_);
DSP_STATIC_ASSERT(sizeof(V90ModulatorPfx) <= sizeof(V90Modulator) && sizeof(V92ModulatorPfx) <= sizeof(V92Modulator) &&
                      sizeof(V90DemodulatorPfx) <= sizeof(V90Demodulator) && sizeof(V90ConnectionEvaluatorPfx) <= sizeof(V90ConnectionEvaluator),
                  "v34_engine_views.h views are prefixes");
#undef V34ENG_TIE

/*
 * The C view PcmFlo47Obj (src/v34/v34_flomodem_view.h) is how the V.34 files reach the engine context tagV34Object::ctx_ (the rate flags v34Flags_,
 * the V.90 / V.92 engine pointers): it has to be the same object as VPcmFloModem on whichever word size this file is built.
 */
extern "C"
{
#include "v34/v34_flomodem_view.h"
}
#define V34CTX_TIE(vm, cm) DSP_STATIC_ASSERT(offsetof(PcmFlo47Obj, vm) == offsetof(VPcmFloModem, cm), "PcmFlo47Obj." #vm " != VPcmFloModem::" #cm)
V34CTX_TIE(ctx_, ctx_);
V34CTX_TIE(dil_, dil_);
V34CTX_TIE(v34Flags_, v34Flags_);
V34CTX_TIE(cpnotFrame_, cpnotFrame_);
V34CTX_TIE(constelCtr_, constelCtr_);
V34CTX_TIE(v90Mod_, v90_.mod_);
V34CTX_TIE(v90Dem_, v90_.dem_);
V34CTX_TIE(v90Params_, v90_.params_);
V34CTX_TIE(flag6120_, flag6120_);
V34CTX_TIE(v92Mod_, v92_.mod_);
V34CTX_TIE(echoMode_, echoMode_);
V34CTX_TIE(echoCnt_, echoCnt_);
DSP_STATIC_ASSERT(sizeof(PcmFlo47Obj) == sizeof(VPcmFloModem), "sizeof(PcmFlo47Obj) != sizeof(VPcmFloModem)");
#undef V34CTX_TIE
