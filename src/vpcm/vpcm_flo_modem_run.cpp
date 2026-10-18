/**
 * @file vpcm_flo_modem_run.cpp
 * @brief VPcmFloModem::v90RunDemodulator and VPcmFloModem::runPcmModem: the per-block drivers of the PCM modem engine.
 *
 * The two drivers are called once per block by VPcmV34Progress() (vpcm_progress.cpp): v90RunDemodulator() for the V.90 digital (receive) side and
 * runPcmModem() for the V.92 analog side.  Both return a small event code that the caller turns into a V.34 session action (see the class in
 * vpcm_flo_modem.h).  Callees are other files' functions: V90Modem / V92Modem progress(), the demodulator and modulator transition methods, the CP
 * packers (V90CPPacker, setV92CPpckFromParamsInfo, V92setParamsInfoFromCPUnPck) and the V.34 module's C entry points (VPcmV34* / V34XF_*), which take
 * the V.34 context ctx_ as first argument.
 *
 * - v90RunDemodulator(): the session state byte state_ (0 .. 4) gives the return value of the block and may advance; V90Modem::progress() runs the
 *   block; then the demodulator's output code (V90DEMOD_STATE_*, 0 .. 0x2b) selects one handler (message to the V.34 module, CP / CPnot frame
 *   building, debug text).
 * - runPcmModem(): the same state_ prologue; the echo canceller and V90Modem::progress() run on the echo-cancelled block; the demodulator's output
 *   code (0 .. 0x35) selects a handler that drives the V92Modulator / V92Phase4Modulator; then V92Modem::progress() generates the transmit block,
 *   which is scaled by 0.4 and fed to the echo history; the V.92 modulator's event (2 / 3 / 10) selects the echo canceller transition.
 *
 * Conventions: every member is re-read after any call (a callee or print may change it); the debug level is tested before every print; the timing
 * offset is logged as (short) trunc(10 * ppm) from an extended-precision product (ResamplerTimingOffset::getTimingOffsetPPM_LD()); runPcmModem()
 * stores the negated ppm in the V92Modulator through the modulator pointer read before the ppm call; V90CPPacker's result is used as a 16-bit value;
 * byte 3 of the modem parameter block is accessed directly (bit 1 = terminate, bit 2 = restore the SAS detector).
 */

#include "vpcm_flo_modem.h"
#include "core/dsp_math.h"
#include "core/dsp_asm.h"
#include "core/dsp_host.h"
#include "v34/v34_api.h"
#include "vpcm_v34_api.h"

typedef long double LD;

/* ------------------------------------------------------------------------------------------------ debug and trace strings */
#define S_PHASE3_ERR "VPcmFloModem (V90): DemodPhase3Error !!! \r\n"
#define S_SD_NOT "VPcmFloModem (V90): DemodSdNotDetected\r\n"
#define S_PH4_TERM "VPcmFloModem (V90): Phase4 terminated.\r\n"
#define S_TRN1D_SAS "VPcmFloModem (V90): ON Start TRN1d restoring SAS detector\n"
#define S_SD_DET "VPcmFloModem (V90): DemodSdDetected\r\n"
#define S_RTNOT_SAS "VPcmFloModem (V90): restoring SAS detector (RtNot)\n"
#define S_RETRAIN "VPcmFloModem (V90): retrain requested !!\r\n"
#define S_ED "VPcmFloModem (V90): Ed Received !!!\r\n"
#define S_BUILD_CPT "VPcmFloModem (V90): Building CPt, CPt length = %d\r\n"
#define S_DATA_PHASE "VPcmFloModem (V90): Enter Data Phase\r\n"
#define S_BUILD_CP "VPcmFloModem (V90): Building CP, CP length = %d (clr=%d)\r\n"
#define S_REBUILD_CP "VPcmFloModem (V90): Rebuilding CP after silence rrn, CP length = %d\r\n"
#define S_ENERGY_DROP "VPcmFloModem (V90): energy drop detected !!\r\n"
#define S_DROP_V34 "VPcmFloModem (V90): drop to V34 requested !! \r\n"
#define S_TRAIN_CONSTEL "VPcmFloModem (V90): Train constellation : %d  RRN constellation : %d (0 = 4 points / 1 = 16 points)\r\n"
#define S_TERMINATE "VPcmFloModem (V90): terminate session requested !!\r\n"
#define S_SENSITIVE_ISP "VPcmFloModem (V90): on sensitive ISP, after one CPnot supposed to move to E...\r\n"
#define S_RATE_REQ "VPcmFloModem (V90): rate renegotiation requested !!\r\n"
#define S_SCR_NOT_SILENCE "VPcmFloModem (V90): phase3 SCR set to NOT silence, due to RTD %d\r\n"
#define S_CLEARDOWN "VPcmFloModem (V90): Indicating Cleardown !\r\n"
#define S_RATE_DET "VPcmFloModem (V90): rate renegotiation detected !!\r\n"
#define S_CPNOT_MP "VPcmFloModem (V90): Building CPnot on MP receive, CPnot length = %d\r\n"
#define S_CPNOT_MPNOT "VPcmFloModem (V90): Building CPnot on MPnot receive, CPnot length = %d\r\n"
#define S_TERM_CPNOT "VPcmFloModem (V90): Going to terminate CPnot...\r\n"
#define S_NOF_BITS "SetNofBitsPhase4 - nof bits each call = %d\r\n"
#define S_V92_FALLBACK "VPcmFloModem (V92): got V90 Fallback request...\r\n"
#define S_V92_EC_TRAIN "VPcmFloModem (V92): Starting echo canceller training...\r\n"
#define S_V92_EC_FREEZE "VPcmFloModem (V92): Freezing echo canceller...\r\n"
#define S_V92_JD "VPcmFloModem (V92): Jd Detected: trainConstel = %d, rrnConstel =%d\r\n"
#define S_JA_FLAG "Ja Flag set to %d\r\n"
#define S_CP_FLAG "CP Flag set to %d\r\n"
#define S_CPNOT_FLAG "CPnot Flag set to %d\r\n"
#define S_ECHO_ON "\n *** Echo Silence ON *** \n"
#define S_ECHO_OFF "\n *** Echo Silence Off *** \n"

/* ------------------------------------------------------------------------------------------------ helpers (static inline) */

#include "core/dsp_debug.h"

#include "core/dsp_floatbits.h"

/** @brief Byte 3 of the modem parameter block: bit 1 = terminate, bit 2 = restore the SAS detector.  @param p V.90 parameters  @return pointer to the byte */
static inline unsigned char* modemParamByte3(V90Parameters* p) { return &p->params->flags3; }

/** @brief Truncate toward zero to a signed 16-bit value; NaN and out-of-range inputs give -32768 (the x87 "integer indefinite").  @param v value */
static inline int chop16(LD v)
{
  if (!(v > -32769.0L && v < 32768.0L))
    return (int)(short)0x8000;
  return (int)(short)(int)v;
}

/** @brief Argument of VPcmV34LogTimingOffset(): (short) trunc(ppm * 10), the product formed in extended precision.  @param ppm timing offset in ppm */
static inline int ppmLogValue(LD ppm) { return chop16(ppm * (LD)10.0f); }

/** @brief CP bits per call for a training constellation: 2 for constellation 0, else 4.  @param constel training constellation */
static inline unsigned char nofBitsFor(unsigned char constel) { return (unsigned char)(constel == 0 ? 2 : 4); }

#include "core/dsp_x87_hide.h"

/** @brief Copy the received MP fields into mpB_ / mpRate2_ / mpH_ (the MP and MPnot states); the rate mask is doubled and stored as 16 bits. */
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

/** @brief Common tail of several v90RunDemodulator() handlers: log the demodulator's timing offset to the V.34 module. */
static inline void logTimingOffsetV90(VPcmFloModem* m)
{
  int v = ppmLogValue(m->v90_.dem_->rs_.getTimingOffsetPPM_LD());
  VPcmV34LogTimingOffset(m->ctx_, v);
}

/** @brief Set the restore-SAS-detector bit (bit 2 of parameter block byte 3). */
static inline void restoreSas(VPcmFloModem* m)
{
  unsigned char* b = modemParamByte3(m->v90_.params_);
  *b = (unsigned char)(*b | 4);
}

/** @brief CP bookkeeping reset shared by every state that has just built a CP / CPt: bit pointer, Ja / CP / CPnot flags and sequence counter to 0. */
static inline void clearCpFlags(VPcmFloModem* m)
{
  m->jaFlag_ = 0;
  m->bitPtr_ = 0;
  m->cpFlag_ = 0;
  m->cpnotFlag_ = 0;
  m->cpnotCopied_ = 0;
  m->cpSeqCnt_ = 0;
}

/**
 * @brief Tell the V.34 module about a rate renegotiation (shared by the local and the remote "rate renegotiation" states).
 * @param m    engine
 * @param rrnC rate-renegotiation constellation size; passed with the evaluator's silence word (16 bits) to VPcmV34SetV90RateReneg()
 */
static inline void rateRenegIndication(VPcmFloModem* m, unsigned char rrnC)
{
  short rs = (short)m->v90_.dem_->ce_->rrnSilence_;
  VPcmV34SetV90RateReneg(m->ctx_, (int)rs, (int)rrnC);
}

/* ------------------------------------------------------------------------------------------------ v90RunDemodulator */

/**
 * @brief Run one block of the digital-side (V.90 receive) engine.
 *
 * The session state selects the default event code (state 3 moves to 4 when the data phase is steady and error-correcting rate renegotiation is
 * enabled, or on the digital side).  After V90Modem::progress() the demodulator's output code selects one handler, which may override the code,
 * build the CP / CPnot frames (cpFrame_ / cpnotFrame_), tell the V.34 module (Jd, DIL, TRN2d, rate renegotiation, retrain), set the termination
 * flags the bit sources getV90CpBits() / getV90JaBits() test, and log the timing offset.
 * @param in    input samples, n floats
 * @param n     block length in samples
 * @param bits  receives the demodulated bits
 * @param nBits receives the number of bits written to `bits`
 * @return event for VPcmV34Progress(): 0 nothing, 1 hand over to the V.34 handshake, 2 data phase, 3 data phase with error-correcting rate
 *         renegotiation, 5 retrain, 7 drop to V.34, 8 end of session
 */
int VPcmFloModem::v90RunDemodulator(float* in, unsigned n, int* bits, int* nBits)
{
  int ret = 0;

  /* default event code from the session state */
  switch (state_)
  {
  case 0:
  case 1:
    ret = 0;
    break;
  case 2:
    ret = 1;
    break;
  case 3:
    if (flag6120_ == 0 || (v90_.dem_->phase_ == V90D_PHASE_DATA_STEADY && v90_.params_->enableErrorCorrectionRrn != 0))
    {
      state_ = 4;
      ret = 3;
    }
    else
    {
      ret = 2;
    }
    break;
  case 4:
    ret = 3;
    break;
  default:
    ret = 0;
    break;
  }

  /* run the receiver, then dispatch on the demodulator's output code */
  v90_.progress(bits, reinterpret_cast<unsigned&>(*nBits), in, n);

  unsigned code = v90_.dem_->state_;
  if (code > 0x2b)
    return ret;
  switch (code)
  {
  case V90DEMOD_STATE_SD_DETECTED: /* Sd detected */
    state_ = 1;
    ret = 0;
    edprintf(S_SD_DET);
    jaFlag_ = 1;
    if (dbgOn())
      dsplibs_debug_printf(S_JA_FLAG, 1);
    return ret;
  case V90DEMOD_STATE_SDNOT_DETECTED: /* Sd not detected */
    edprintf(S_SD_NOT);
    return ret;
  case V90DEMOD_STATE_TRN1D_STARTED: /* start of TRN1d: restore the SAS detector */
    if (flag6119_ == 0)
      return ret;
    edprintf(S_TRN1D_SAS);
    restoreSas(this);
    return ret;
  case V90DEMOD_STATE_JD_DETECTED:
  { /* Jd received */
    v90_.jd_->getConstelationSize(&trainConstel_, &rrnConstel_);
    if (dbgOn())
      dsplibs_debug_printf(S_TRAIN_CONSTEL, (int)trainConstel_, (int)rrnConstel_);
    /* the SCR is switched to "not silence" when the round-trip delay exceeds the configured limit */
    V90Parameters* p = v90_.params_;
    unsigned char scr = (unsigned char)p->silenceScr; /* low byte of the 32-bit member */
    if (scr != 0)
    {
      unsigned rtd = v90_.info_->rtd_;
      if (rtd > (unsigned)p->minimumRtdForNonSilenceScr)
      {
        edprintf(S_SCR_NOT_SILENCE, (int)rtd);
        scr = 0;
      }
    }

    /* CP bits per call follow the training constellation; tell the V.34 module */
    unsigned char tc = trainConstel_;
    nofBitsPhase4_ = nofBitsFor(tc);
    if (dbgOn())
      dsplibs_debug_printf(S_NOF_BITS, (int)nofBitsPhase4_);
    tc = trainConstel_;
    V34XF_IndicateJdReceived(ctx_, (int)tc, (int)scr);
    logTimingOffsetV90(this);
    return ret;
  }
  case V90DEMOD_STATE_JDNOT_DETECTED:
  { /* JdNot detected: clear bit 2 of the parameter byte unless the terminate bit (bit 1) is set */
    unsigned char* b = modemParamByte3(v90_.params_);
    unsigned char v = *b;
    if (v & 2)
      return ret;
    *b = (unsigned char)(v & 0xfb);
    return ret;
  }
  case V90DEMOD_STATE_DIL_RECEIVED:
  { /* DIL received: build the CPt */
    V34XF_IndicateDilReceived(ctx_, (int)trainConstel_);
    short len = (short)V90CPPacker(&v90_.mp1_, &v90_.addCp_, (short*)cpFrame_, 0);
    cpLen_ = (uint16_t)len;
    if (dbgOn())
      dsplibs_debug_printf(S_BUILD_CPT, (int)len);
    clearCpFlags(this);
    logTimingOffsetV90(this);
    return ret;
  }
  case V90DEMOD_STATE_PHASE3_ERROR: /* phase 3 error */
    if (dbgOn())
      dsplibs_debug_printf(S_PHASE3_ERR);
    return 5;
  case V90DEMOD_STATE_RI_DETECTED:
    state_ = 2;
    return 1;
  case V90DEMOD_STATE_RINOT_DETECTED:
    cpnotFlag_ = 1;
    if (dbgOn())
      dsplibs_debug_printf(S_CPNOT_FLAG, 1);
    if (flag6119_ == 0)
      return ret;
    restoreSas(this);
    return ret;
  case V90DEMOD_STATE_TRN2D_RECEIVED:
  { /* TRN2d received: build the CP (clear-down bit) */
    v90_.addCp_.flag_ = 0;
    short len = (short)V90CPPacker(&v90_.mp2_, &v90_.addCp_, (short*)cpFrame_, (int)clearDown_);
    cpLen_ = (uint16_t)len;
    if (dbgOn())
      dsplibs_debug_printf(S_BUILD_CP, (int)(short)cpLen_, (int)clearDown_);
    clearCpFlags(this);
    minTxSeq_ = 1;
    V34XF_IndicateTrn2dReceived(ctx_);
    logTimingOffsetV90(this);
    return ret;
  }
  case V90DEMOD_STATE_MP_DETECTED:
  { /* MP received */
    copyMpInfo(this);
    if (cpFlag_ != 0)
      return ret;
    v90_.addCp_.flag_ = 1;
    short len = (short)V90CPPacker(&v90_.mp2_, &v90_.addCp_, (short*)cpnotFrame_, (int)clearDown_);
    cpnotLen_ = (uint16_t)len;
    if (dbgOn())
      dsplibs_debug_printf(S_CPNOT_MP, (int)len);
    cpFlag_ = 1;
    if (dbgOn())
      dsplibs_debug_printf(S_CP_FLAG, 1);
    return ret;
  }
  case V90DEMOD_STATE_MPNOT_DETECTED:
  { /* MPnot received */
    copyMpInfo(this);
    if (cpFlag_ == 0)
    {
      v90_.addCp_.flag_ = 1;
      short len = (short)V90CPPacker(&v90_.mp2_, &v90_.addCp_, (short*)cpnotFrame_, 0);
      cpnotLen_ = (uint16_t)len;
      if (dbgOn())
        dsplibs_debug_printf(S_CPNOT_MPNOT, (int)len);
      cpFlag_ = 1;
      if (dbgOn())
        dsplibs_debug_printf(S_CP_FLAG, 1);
      if (v90_.params_->sensitiveIspDetected == 0)
        return ret;
      edprintf(S_SENSITIVE_ISP);
    }
    else
    {
      if (cpnotCopied_ == 0)
        return ret;
      if (cpnotFlag_ != 0)
        return ret;
      if (v90_.dem_->ce_->rrnSilence_ != 0)
      {
        V90Phase4Demodulator* p4 = v90_.dem_->p4_;
        if (p4->rrnPending_ != 0)
        {
          if (p4->rtPending_ != 0)
            return ret;
        }
      }
      if (dbgOn())
        dsplibs_debug_printf(S_TERM_CPNOT);
    }
    cpnotFlag_ = 1;
    if (dbgOn())
      dsplibs_debug_printf(S_CPNOT_FLAG, 1);
    cpSeqCnt_ = 0;
    minTxSeq_ = 1;
    return ret;
  }
  case V90DEMOD_STATE_ED_DETECTED: /* Ed received */
    if (dbgOn())
      dsplibs_debug_printf(S_ED);
    cpnotFlag_ = 1;
    if (dbgOn())
      dsplibs_debug_printf(S_CPNOT_FLAG, 1);
    cpSeqCnt_ = 0;
    minTxSeq_ = 1;
    if (clearDown_ != 0)
    {
      if (dbgOn())
        dsplibs_debug_printf(S_CLEARDOWN);
      ret = 8;
    }
    return ret;
  case V90DEMOD_STATE_PHASE4_TERMINATED: /* phase 4 terminated */
    if (dbgOn())
      dsplibs_debug_printf(S_PH4_TERM);
    return ret;
  case V90DEMOD_STATE_DATA_PHASE_ENTERED: /* data phase entered */
    if (dbgOn())
      dsplibs_debug_printf(S_DATA_PHASE);
    state_ = 3;
    ret = 2;
    flag6119_ = 1;
    restoreSas(this);
    logTimingOffsetV90(this);
    return ret;
  case V90DEMOD_STATE_DROP_TO_V34: /* drop to V.34 */
    if (dbgOn())
      dsplibs_debug_printf(S_DROP_V34);
    dropToV34_ = 1;
    ret = 7;
    v34Flags_[0] = 1;
    v34Flags_[1] = 0;
    v34Flags_[2] = 1;
    v34Flags_[3] = 1;
    v34Flags_[4] = 1;
    v34Flags_[5] = 1;
    return ret;
  case V90DEMOD_STATE_RETRAIN_REQUESTED: /* retrain requested */
    if (dbgOn())
      dsplibs_debug_printf(S_RETRAIN);
    return 5;
  case V90DEMOD_STATE_RATE_RENEG_REQUESTED:
  { /* rate renegotiation requested (local) */
    if (dbgOn())
      dsplibs_debug_printf(S_RATE_REQ);
    unsigned char rrnC = rrnConstel_;
    rateRenegIndication(this, rrnC);
    unsigned char t = rrnConstel_;
    nofBitsPhase4_ = nofBitsFor(t);
    if (dbgOn())
      dsplibs_debug_printf(S_NOF_BITS, (int)nofBitsPhase4_);
    localRrn_ = 1;
    VPcmV34IndicateLocalRRN(ctx_);
    state_ = 2;
    return 1;
  }
  case V90DEMOD_STATE_RATE_RENEG_DETECTED: /* rate renegotiation detected (remote) */
    if (localRrn_ != 0)
    {
      localRrn_ = 0;
      return ret;
    }
    else
    {
      if (dbgOn())
        dsplibs_debug_printf(S_RATE_DET);
      unsigned char rrnC = rrnConstel_;
      rateRenegIndication(this, rrnC);
      unsigned char t = rrnConstel_;
      nofBitsPhase4_ = nofBitsFor(t);
      if (dbgOn())
        dsplibs_debug_printf(S_NOF_BITS, (int)nofBitsPhase4_);
      VPcmV34IndicateRemoteRRN(ctx_);
      v90_.dem_->indicateRemoteRateReneg();
      state_ = 2;
      return 1;
    }
  case V90DEMOD_STATE_ENERGY_DROP: /* energy drop */
    if (dbgOn())
      dsplibs_debug_printf(S_ENERGY_DROP);
    ret = 5;
    VPcmV34SetIndicationOfRemoteRetrain(ctx_);
    return ret;
  case V90DEMOD_STATE_RTNOT_DETECTED: /* RtNot */
    edprintf(S_RTNOT_SAS);
    restoreSas(this);
    return ret;
  case V90DEMOD_STATE_WAIT_FOR_RT_ENTERED:
  { /* the demodulator entered its wait-for-Rt state: rebuild the CP after a silence rate renegotiation */
    unsigned char clr = clearDown_;
    v90_.addCp_.flag_ = 0;
    v90_.addCp_.suvVal_ = 0;
    short len = (short)V90CPPacker(&v90_.mp2_, &v90_.addCp_, (short*)cpFrame_, (int)clr);
    cpLen_ = (uint16_t)len;
    if (dbgOn())
      dsplibs_debug_printf(S_REBUILD_CP, (int)len);
    clearCpFlags(this);
    V34XF_IndicateTrn2dReceived(ctx_);
    return ret;
  }
  case V90DEMOD_STATE_TERMINATE_SESSION: /* terminate session requested */
    if (dbgOn())
      dsplibs_debug_printf(S_TERMINATE);
    ret = 8;
    logTimingOffsetV90(this);
    return ret;
  default:
    return ret;
  }
}

/* ------------------------------------------------------------------------------------------------ runPcmModem */

/** @brief The V.92 side's timing-offset log: report the given demodulator's offset to the V.34 module.  @param m engine  @param dem V.90 demodulator */
static inline void logTimingOffsetV92(VPcmFloModem* m, V90Demodulator* dem)
{
  int v = ppmLogValue(dem->rs_.getTimingOffsetPPM_LD());
  VPcmV34LogTimingOffset(m->ctx_, v);
}

/** @brief Start a rate renegotiation in the V.92 modulator, then copy the evaluator's silence word into its phase 4 modulator. */
static inline void v92InitiateRrn(VPcmFloModem* m)
{
  m->v92_.mod_->initiateRRN();
  V90Demodulator* dem = m->v90_.dem_;
  V92Modulator* mod = m->v92_.mod_;
  V90ConnectionEvaluator* ce = dem->ce_;
  V92Phase4Modulator* p4 = mod->p4_;
  p4->rrnSilence_ = (unsigned)ce->rrnSilence_;
}

/**
 * @brief Run one block of the analog-side (V.92) engine.
 *
 * Prologue as for v90RunDemodulator(); the digital side (flag6120_ == 0) returns here.  During echo silence ecOut_[0] carries a ramp.  The block is
 * echo cancelled (ec_) and demodulated by the V.90 half, the demodulator's output code drives the V.92 modulator / phase 4 modulator, the V.92 half
 * generates the transmit block (scaled by 0.4 and fed to the echo history), and the V.92 modulator's event steers the session: event 2 sets session
 * state 1 and starts the echo canceller's delay count, event 3 restricts the echo canceller to filtering, restarts the demodulator in phase 3
 * (returning a V.90 fallback code when that is the demodulator's answer), event 10 sets session state 3.
 * @param in     input samples, n floats
 * @param out    receives the n transmit samples
 * @param n      block length in samples
 * @param bits   receives the demodulated bits
 * @param nBits  receives the number of bits written to `bits`
 * @param bits2  receives the bits consumed by the V.92 transmit half
 * @param nBits2 receives the number of bits written to `bits2`
 * @return event for VPcmV34Progress(): as v90RunDemodulator(), plus 6 for a fallback to V.90
 */
int VPcmFloModem::runPcmModem(float* in, float* out, unsigned n, int* bits, int* nBits, int* bits2, int* nBits2)
{
  int ret = 0;

  /* default event code from the session state */
  switch (state_)
  {
  case 0:
  case 1:
    ret = 0;
    break;
  case 2:
    ret = 1;
    break;
  case 3:
    if (flag6120_ != 0 && v90_.dem_->phase_ == V90D_PHASE_DATA_STEADY && v90_.params_->enableErrorCorrectionRrn != 0)
    {
      state_ = 4;
      ret = 3;
    }
    else
    {
      ret = 2;
    }
    break;
  case 4:
    ret = 3;
    break;
  default:
    ret = 0;
    break;
  }

  if (flag6120_ == 0)
    return ret;

  /* echo silence: ecOut_[0] carries a counter that ramps up to 0xb1; otherwise it is zero */
  if (echoMode_ == 0x21)
  {
    if (echoCnt_ != 0xb1u)
      echoCnt_ = echoCnt_ + 1u;
    ecOut_[0] = (float)(int)echoCnt_;
  }
  else
  {
    setWord(&ecOut_[0], 0u);
  }

  /* echo cancel, then receive */
  ec_.process(in, ecOut_, n);
  v90_.progress(bits, reinterpret_cast<unsigned&>(*nBits), ecOut_, n);

  /* the modulator pointer is read before the ppm call; the negated offset goes to the V.92 modulator */
  V92Modulator* mod = v92_.mod_;
  LD ppm = v90_.dem_->rs_.getTimingOffsetPPM_LD();
  V90Demodulator* dem = v90_.dem_;
  unsigned code = dem->state_;
  mod->ppm_ = (float)(-ppm);

  /* dispatch on the demodulator's output code */
  if (code <= 0x35)
  {
    switch (code)
    {
    case V90DEMOD_STATE_SD_DETECTED: /* echo silence on */
      echoMode_ = 0x21;
      echoCnt_ = 0xaa;
      if (dbgOn())
        dsplibs_debug_printf(S_ECHO_ON);
      state_ = 1;
      ret = 0;
      v92_.mod_->exitJa();
      break;
    case V90DEMOD_STATE_JD_DETECTED: /* echo silence off */
      if (dbgOn())
        dsplibs_debug_printf(S_ECHO_OFF);
      {
        V92Modulator* m = v92_.mod_;
        echoMode_ = 1;
        echoCnt_ = 0xaa;
        m->exitSilence();
      }
      logTimingOffsetV92(this, v90_.dem_);
      break;
    case V90DEMOD_STATE_V92_JD_PHASE_DETECTED:
    { /* Jd detected */
      v90_.jd92_->getConstelationSize(&trainConstel_, &rrnConstel_);
      if (dbgOn())
        dsplibs_debug_printf(S_V92_JD, (int)trainConstel_, (int)rrnConstel_);
      unsigned char b = rrnConstel_;
      unsigned char a = trainConstel_;
      V92Modulator* m = v92_.mod_;
      m->codeB_ = b;
      m->codeA_ = a;
      V92Jd* jd = v90_.jd92_;
      V92Modulator* m2 = v92_.mod_;
      float ph = jd->getJdPhase();
      V92Modulator* m3 = v92_.mod_;
      m2->skew_ = ph;
      m3->exitSuSecond();
      unsigned char* b3 = modemParamByte3(v90_.params_);
      unsigned char v = *b3;
      if ((v & 2) == 0)
        *b3 = (unsigned char)(v & 0xfb);
      break;
    }
    case V90DEMOD_STATE_PHASE3_TERMINATED:
      setV92CPpckFromParamsInfo(&v90_.mp1_, &v90_.addCp_, v92_.cp_);
      v92_.mod_->exitTRN1uSecond();
      logTimingOffsetV92(this, v90_.dem_);
      break;
    case V90DEMOD_STATE_PHASE3_ERROR:
    case V90DEMOD_STATE_RETRAIN_REQUESTED:
      ret = 5;
      break;
    case V90DEMOD_STATE_RI_DETECTED:
      state_ = 2;
      ret = 1;
      v92_.mod_->exitCPt();
      if (flag6119_ != 0)
        restoreSas(this);
      break;
    case V90DEMOD_STATE_TRN2D_RECEIVED:
      setV92CPpckFromParamsInfo(&v90_.mp2_, &v90_.addCp_, v92_.cp_);
      v92_.mod_->p4_->exitTRN2u();
      logTimingOffsetV92(this, v90_.dem_);
      break;
    case V90DEMOD_STATE_ED_DETECTED:
      v92_.mod_->p4_->recivedEd();
      break;
    case V90DEMOD_STATE_DATA_PHASE_ENTERED:
      state_ = 3;
      ret = 2;
      restoreSas(this);
      logTimingOffsetV92(this, v90_.dem_);
      break;
    case V90DEMOD_STATE_DROP_TO_V34:
      dropToV34_ = 1;
      ret = 7;
      v34Flags_[0] = 1;
      v34Flags_[1] = 0;
      v34Flags_[2] = 1;
      v34Flags_[3] = 1;
      v34Flags_[4] = 1;
      v34Flags_[5] = 1;
      break;
    case V90DEMOD_STATE_V90_FALLBACK:
      ret = 6;
      break;
    case V90DEMOD_STATE_RATE_RENEG_REQUESTED:
      state_ = 2;
      ret = 1;
      if (v92_.mod_->state_ == 3)
        v92InitiateRrn(this);
      VPcmV34IndicateLocalRRN(ctx_);
      break;
    case V90DEMOD_STATE_RATE_RENEG_DETECTED:
      state_ = 2;
      ret = 1;
      if (v92_.mod_->state_ == 3)
        v92InitiateRrn(this);
      VPcmV34IndicateRemoteRRN(ctx_);
      break;
    case V90DEMOD_STATE_24:
    case V90DEMOD_STATE_FPE_DETECTED:
      state_ = 2;
      ret = 1;
      if (v92_.mod_->state_ == 3)
        v92_.mod_->initiateFPE();
      break;
    case V90DEMOD_STATE_ENERGY_DROP:
      ret = 5;
      VPcmV34SetIndicationOfRemoteRetrain(ctx_);
      break;
    case V90DEMOD_STATE_RT_DETECTED:
      v92_.mod_->p4_->recivedRt();
      break;
    case V90DEMOD_STATE_TERMINATE_SESSION:
      ret = 8;
      logTimingOffsetV92(this, dem);
      break;
    case V90DEMOD_STATE_CP_DETECTED:
      V92setParamsInfoFromCPUnPck(v92_.mapping_, &v90_.cp_);
      if (v90_.cp_.extendEu_ != 0)
        v92_.mod_->p4_->extendEu_ = 1;
      v92_.mod_->p4_->recivedCP();
      break;
    case V90DEMOD_STATE_CPNOT_DETECTED:
      V92setParamsInfoFromCPUnPck(v92_.mapping_, &v90_.cp_);
      v92_.mod_->p4_->recivedCPtag();
      break;
    case V90DEMOD_STATE_SUV_DETECTED:
      v92_.mod_->p4_->recivedSUV();
      break;
    case V90DEMOD_STATE_SUVT_DETECTED:
      v92_.mod_->p4_->recivedSUVtag();
      break;
    case V90DEMOD_STATE_SILENCE_RRN_SUV_PART1:
    {
      V92Modulator* m = v92_.mod_;
      V90CP* cp = dem->cp_;
      V92Phase4Modulator* p4 = m->p4_;
      p4->w30_ = 1;
      p4->w38_ = cp->wCa0_;
      p4->recivedPartOneSilenceRrnSUV();
      break;
    }
    case V90DEMOD_STATE_SILENCE_RRN_SUV_PART2:
      v92_.mod_->p4_->recivedPartTwoSilenceRrnSUV();
      break;
    case V90DEMOD_STATE_SILENCE_RRN_SUVT_PART1:
    {
      V92Modulator* m = v92_.mod_;
      V90CP* cp = dem->cp_;
      V92Phase4Modulator* p4 = m->p4_;
      p4->w30_ = 1;
      p4->w38_ = cp->wCa0_;
      p4->recivedPartOneSilenceRrnSUVtag();
      break;
    }
    case V90DEMOD_STATE_SILENCE_RRN_SUVT_PART2:
      v92_.mod_->p4_->recivedPartTwoSilenceRrnSUVtag();
      break;
    case V90DEMOD_STATE_FIRST_RRN_ED_DETECTED:
      v92_.mod_->p4_->recivedFirstRrnEd();
      break;
    default:
      break;
    }
  }

  /* the V.92 half generates the transmit block, which is scaled by 0.4 and fed to the echo history */
  v92_.progress(bits2, reinterpret_cast<unsigned&>(*nBits2), out, n);
  for (unsigned i = 0; i < n; i++)
    out[i] = mulF(out[i], 0.4f);
  ec_.updateEchoHistory(out, n);

  /* the modulator's event steers the session and the echo canceller */
  V92Modulator* m = v92_.mod_;
  int ev = (int)m->event_;
  if (ev == 3)
  {
    if (dbgOn())
      dsplibs_debug_printf(S_V92_EC_FREEZE);
    ec_.setState(V92_ECHO_FILTER_ONLY);
    v90_.dem_->enterPhase3();
    if (v90_.dem_->state_ == V90DEMOD_STATE_V90_FALLBACK)
    {
      edprintf(S_V92_FALLBACK);
      ret = 6;
    }
  }
  else if (ev > 3)
  {
    if (ev == 0xa)
      state_ = 3;
  }
  else if (ev == 2)
  {
    state_ = 1;
    if (dbgOn())
      dsplibs_debug_printf(S_V92_EC_TRAIN);
    ec_.setState(V92_ECHO_COUNT_DELAY);
  }
  return ret;
}
