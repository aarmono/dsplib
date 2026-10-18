/**
 * @file v90_demodulator_progress.cpp
 * @brief V90Demodulator::progress(): the per-block receive path of the V.90 / V.92 PCM modem (analog side) and its phase / event state machine.
 *
 * V90Demodulator::progress() is called once per block of input samples by V90Modem::progress().  It owns no state of its own: everything it reads and writes
 * is a member of V90Demodulator (declared in v90_demodulator.h; the constructor, destructor and the phase entry / exit methods are in v90_demodulator.cpp).
 *
 * Signal path of one call:
 * - `symCnt_ += n`; the block is filtered by the pre-filter (V90PreFilter, a FloatFIR), scaled by the Agc, resampled by V90Resampler (timing recovery) and
 *   handed to V90Equalizer::process(), which leaves the phase event in `eq_->event_`; progress() copies it to `state_` (codes: v90_demod_state.h).
 * - A dispatch on `phase_` follows:
 *   - 0 (idle): clears `outCnt` only.
 *   - 1 (phase 3): runs the spectral verifier (V90SpectralVerifier), which selects the pre-filter gain, the German NT1 / PBX parameter sets or the
 *     severe-codec drop; then dispatches on the event (0 .. 0x15: Sd / TRN1d / Jd / DIL handling, the BLL timing-loop state of the resampler, the
 *     Agc freeze and DIL overflow protection, the phase 3 connection evaluator, entry to phase 4).
 *   - 2 (phase 4): counts the phase 4 timeout (`timeCnt_` against `timeLimit_`); dispatches on the V90Phase4Demodulator events (0x19 .. 0x35): the data
 *     constellation is designed from the equalizer mean error (V90ConstellationDesigner::process), the CP / MP / Ed wait states are entered, and the
 *     silence-RRN logic redesigns the constellation; the phase 4 connection evaluator runs at the end.
 *   - 3 / 4 (data): V90Demapper::process() then Descrambler::process() produce the output bits; ConnectionEvaluator::evaluateConnection() results (0 .. 5)
 *     map to rate renegotiation / retrain / fall-back-to-V.34 outcome codes in `state_`.
 * - The common tail runs the remote-retrain energy detector (the Agc power below a threshold for a configured duration), prints the error energy and
 *   timing offset periodically and updates `framePos_` (mod 6) and the outcome counters `cnt23_` / `cnt1f_` / `cnt26_`.
 *
 * Structure: the body is a set of labelled blocks joined by goto, one block per dispatch target or shared tail; each block is introduced by a banner or an
 * end-of-line note naming the event it handles.  Values a block carries across callee calls are locals (`bllState`, `prevRate`, ...); everything a
 * callee may change is re-read from the object.
 *
 * Numeric notes:
 * - Raw float words (the German step sizes, `agcK` -> `agc_.mix_`, `designErr_`, the constellation thresholds, the ratio and statistics handed to the
 *   connection evaluator, the Agc gain handed to adjustHalfBaudBpfGain) are copied bit-for-bit (copyWord / WORDARG) instead of converted.
 * - The "%c%d.%0Nd" prints split a long-double value into sign, integer part and fraction (splitValue, truncation); the sign is '+' when 0 < v or v is
 *   unordered.  The Agc gain divided by the pad gain is printed from the unrounded quotient; the timing-offset print queries the resampler four times.
 */

#include "v90_demodulator.h"
#include "core/dsp_math.h"
#include "core/dsp_asm.h"
#include "core/dsp_host.h"
#include "v34/v34_api.h"

extern "C"
{
}

typedef long double LD;

/* ------------------------------------------------------------------------------------------------ debug / diagnostic message texts */
#define PFX "V90Demodulator: "
#define S_LINMAP_OFF PFX "disable linear mapping study in data\n"
#define S_ERRENERGY PFX "Error Energy = %c%d.%03d\r\n"
#define S_TMOFF PFX "Timing Offset [ppm]  = %c%d.%03d\r\n"
#define S_SVERR PFX "!!! ERROR !!! : Spectral Verifier returned illegal value !!!\r\n"
#define S_SILRRN_NOTFIN PFX "Silence rrn not finished on platform other then USB, masking silence rrn...\r\n"
#define S_SENSITIVE PFX "On phase4 timeout: Assuming Sensitive ISP - use non-silence & 6dB up & maixmal rate 12000 for next time...\r\n"
#define S_P4TIMEOUT PFX "Phase4 TimeOut\r\n"
#define S_PREFILTER PFX "PreFilter adjusted to: %d\r\n"
#define S_PREVRATE PFX "on silence RRN redesign, prevRate = %d\r\n"
#define S_RATEDOWN PFX "one rate down on silence rrn\r\n"
#define S_ABOUTREDESIGN PFX "about to redesign... current Mean Error = %c%d.%04d,    average Mean Error = %c%d.%04d\r\n"
#define S_RTNOT PFX "RtNot detected.\r\n"
#define S_SILRRN_NOTMASKED PFX "Silence RRN is not masked (silence flag = %d)\r\n"
#define S_FREEZING PFX "freezing timing on silence rrn...\r\n"
#define S_JDMAX PFX "Jd maxLookAhead = %d\r\n"
#define S_ENABLE_TRN1D PFX "enabling connectionEvaluator of TRN1d\r\n"
#define S_AGCGAIN PFX "Agc Gain = %c%d.%06d\r\n"
#define S_FALLBACK PFX "Request FallBack to V.34 due to line conditions\r\n"
#define S_BAR61 "-------------------------------------------------------------\r\n"
#define S_REMOTE_RETRAIN PFX "REMOTE RETRAIN - Energy Drop detected\r\n"
#define S_AGCENERGY PFX "Agc Energy = %c%d.%02d\r\n"
#define S_DILMAX PFX "Dil max ucode = %d\n"
#define S_SEVERE_DROP PFX "Severe Codec conditions were detected NOT on EIA6, initiating drop 2 V34...\r\n"
#define S_FORCED_RATEDOWN PFX "FORCED rate down on silence rrn\r\n"
#define S_SEVERE_MASKED PFX "Severe Codec conditions were detected NOT on EIA6, drop is masked, doing nothing...\r\n"
#define S_DILSTUDY_END PFX "Dil study Terminated. Enter error relaxation period.\n"
#define S_DISABLE_P3EVAL PFX "disabling connectionEvaluator of Phase3\r\n"
#define S_AGCACTIVATED PFX "Agc Activated\r\n"
#define S_SEVERE_EIA6 PFX "Severe Codec conditions were detected due to EIA6 loop type...\r\n"
#define S_DILOVF PFX "Agc Gain low > Setting DIL overflow protection !!!\r\n"
#define S_TRN1D_NOTENABLED PFX "connectionEvaluator of TRN1d is NOT ENABLED due to quick connect...\r\n"
#define S_DATASPECTRAL PFX "Data Phase spectral parameters:\r\n"
#define S_CONSTDESIGN_SIL PFX "Constellation design on silence rrn...\r\n"
#define S_FORCED_KEEP PFX "FORCED keep rate on silence rrn\r\n"
#define S_BAR63 "---------------------------------------------------------------\r\n"
#define S_TEARDOWN PFX "tearing down connection, probing mode ended...\r\n"
#define S_KEEPING PFX "keeping rate on silence rrn\r\n"
#define S_EDRECV PFX "Ed received on Silence RRN... current Mean Error = %c%d.%04d,    average Mean Error = %c%d.%04d\r\n"
#define S_DESIGNERR PFX "connection design error, initiating retrain\r\n"
#define S_SETALTRBS PFX "setAltRbsDetectedOnQC was called !!!\r\n"
#define S_DILOVF2 PFX "Agc Gain very low > Setting DIL extreme overflow protection !!!\r\n"
#define S_BAR75 "###########################################################################\r\n"
#define S_PROBLEM_NOTUSB PFX "'Problematic' ISP Modem detected NOT on USB. Masking Silence RRN...\r\n"
#define S_PROBLEM_USB PFX "'Problematic' ISP Modem detected on USB. Masking drop to V34 (MAX_NOF_V90_RETRAINS = %d)...\r\n"
#define S_ENTERP4 PFX "enter Phase 4\r\n"
#define S_FPE_ENTERP4 PFX "FPE detected: enter Phase 4\r\n"
#define S_AGCFROZEN PFX "Agc Frozen\r\n"
#define S_DISABLE_P3 PFX "disabling connectionEvaluator of Phase3\r\n"

/* ------------------------------------------------------------------------------------------------ helpers */

#include "core/dsp_x87_cmp.h"

#include "core/dsp_intmath.h"

#include "vpcm_split.h"
#include "vpcm_split_value.h"
static inline void dbgFix(const char* fmt, LD v, LD scale)
{
  Split s = splitValue(v, scale);
  dsplibs_debug_printf(fmt, s.sign, s.ip, s.fp);
}
/** Print a pair of mean errors ("current" = a, "average" = b) as sign / integer / fraction of a, then of b, with four decimals (scale 10000). */
static inline void edMeans(const char* fmt, LD a, LD b)
{
  Split x = splitValue(a, (LD)10000.0f);
  Split y = splitValue(b, (LD)10000.0f);
  edprintf(fmt, x.sign, x.ip, x.fp, y.sign, y.ip, y.fp);
}

#include "core/dsp_floatbits.h"

#include "core/dsp_copyword_raw.h"

#include "core/dsp_wordarg.h"

#include "core/dsp_x87_conv_int.h"

/** Low 32 bits of the truncated 64-bit integer of `v`; `v` is always in [0.5, 2^32 / 6 + 0.5] here (the bit rate 0.5 + u32 / 6), so it never overflows. */
static inline unsigned trunc64lo(LD v) { return (unsigned)(unsigned long long)(long long)v; }

/** Outcome code for a request the line cannot serve by retrain: drop to V.34 (0x1f) when `retrainOk` is 0, else retrain (0x21). */
static inline unsigned retrainState(unsigned retrainOk) { return (retrainOk == 0) ? V90DEMOD_STATE_DROP_TO_V34 : V90DEMOD_STATE_RETRAIN_REQUESTED; }

/* ------------------------------------------------------------------------------------------------ V90Demodulator::progress */

/**
 * @brief Process one block of received samples: filter, equalize, run the phase state machine and produce data bits.
 * @param out     receives the descrambled output bits of this block (data phases only), one per int
 * @param outCnt  receives the number of bits written to `out`; set to 0 in phases 0, 1 and 2
 * @param in      input samples (float), `n` entries
 * @param n       number of input samples in the block
 * @note  On return `state_` holds the outcome / event code of this block (v90_demod_state.h): 0 means nothing happened; 0x1f, 0x21, 0x22 and 0x26 ask the
 *        caller (VPcmFloModem) to fall back to V.34, retrain, renegotiate the rate or recover from an energy drop.
 * @note  Side effects: may change `phase_` (enterPhase4 / enterDataPhase / exitPhase3 / enterRRN / enterDataSteadyState), the resampler timing-loop state, the
 *        Agc, the pre-filter and several V90Parameters members (German parameter sets, silence-RRN masks, retrain limits).
 * @warning `in` must hold `n` samples and the internal buffers (`inBuf_`, `rsBuf_`, `decBuf_`, `eqBuf_`, `byteBuf_`) were sized from the constructor's `n`;
 *          a block longer than that overruns them.  `out` must be large enough for the bits `dem_` produces for a block of `n` samples.
 */

void V90Demodulator::progress(int* out, unsigned& outCnt, float* in, unsigned n)
{
  unsigned bllState, bllCount; /* resampler timing-loop (BLL) state and its duration counter */
  unsigned retrainFlag;        /* retrainOk_, re-read after each BLL state change */
  unsigned r;                  /* a callee's result */
  unsigned prevRate;           /* bit rate before a silence-RRN redesign */
  float me;                    /* calcMeanErrorStatistics() rounded to float */
  float mw0;                   /* first argument of evaluateMeanErrorStdPhase4 (the second is the raw word of eq_->statMean_) */
  uint32_t mw1;
  float noise; /* noise estimate handed to the constellation designer */
  unsigned rateMask;
  unsigned char maxLookahead;
  unsigned char* maxUcode;
  unsigned spectralCond;
  unsigned tmp;

  symCnt_ += n;

  /* receive chain: pre-filter, Agc (in place), resampler, equalizer; the equalizer's phase event becomes the block's state */
  pf_.process(in, inBuf_, n);
  agc_.process(inBuf_, inBuf_, n);
  rs_.resample(inBuf_, n, rsBuf_, rsCnt_);
  eq_->process(rsBuf_, rsCnt_, decBuf_, eqBuf_, eqCnt_);
  state_ = eq_->event_;

  /* dispatch on the receive phase; anything above 4 skips the phase handlers */
  if (phase_ > 4)
    goto retrainEnergyDetector;
  switch (phase_)
  {
  case V90D_PHASE_IDLE:
    goto phase0;
  case V90D_PHASE_3:
    goto phase1SpectralVerifier;
  case V90D_PHASE_4:
    goto phase4Entry;
  case V90D_PHASE_DATA:
    goto dataPhaseEntry;
  default:
    goto dataProcess;
  }

  /* ---------------------------------------------------------------- data phase: results of the connection evaluator (0 .. 5) */
evalRateUp: /* result 1: the line supports a higher rate */
  state_ = V90DEMOD_STATE_RATE_RENEG_REQUESTED;
  cdes_->state_ = 2;

restoreEqualizer:
  eq_->restoreEqualizerToFloat();

evalDone: /* result 0: nothing to do; on a rate-renegotiation outcome the silence-RRN state is passed to phase 4 first */
  if (session_ == 0)
    goto retrainEnergyDetector;
  if (state_ - 0x22 > 1)
    goto retrainEnergyDetector;
  p4_->resetBeforRRN();
  p4_->rrnSilence_ = ce_->rrnSilence_;
  dem_->linMapStudyOn_ = 0;
  if (dsplibs_debug_level > 1)
    dsplibs_debug_printf(S_LINMAP_OFF);

  /* ---------------------------------------------------------------- remote retrain energy detector */
retrainEnergyDetector:
  if (flag40_ == 0)
    goto periodicPrints;
  if (jmpAe((LD)agc_.power_, (LD)params_->energyDropDetectorThreshold))
  { /* power at or above the threshold: restart the quiet counter */
    quietCnt_ = 0;
    goto periodicPrints;
  }
  {
    unsigned s = n + quietCnt_;
    quietCnt_ = s;
    if (s < (unsigned)params_->noEnergyDurationForRemoteRetrain)
      goto periodicPrints;
  }
  /* energy has been missing long enough: announce a remote retrain */
  if (dsplibs_debug_level > 1)
  {
    dsplibs_debug_printf(S_BAR61);
    if (dsplibs_debug_level > 1)
    {
      dsplibs_debug_printf(S_REMOTE_RETRAIN);
      if (dsplibs_debug_level > 1)
        dsplibs_debug_printf(S_BAR61);
    }
  }
  r = ce_->indicateRemoteRetrain();
  if (r == 5)
    goto remoteRetrainResult;
  state_ = V90DEMOD_STATE_ENERGY_DROP;

  /* ---------------------------------------------------------------- periodic prints (error energy, timing offset) and frame bookkeeping */
periodicPrints:
  tmp = n + errPrintCnt_;
  if (tmp < errPrintPeriod_)
    goto errPrintAccumulate;
  errPrintCnt_ = 0;
  if (!(dsplibs_debug_level > 1))
    goto timingPrintCheck;
  dbgFix(S_ERRENERGY, (LD)eq_->meanErr_, (LD)1000.0f);
  tmp = n + tmPrintCnt_;
  if (tmp < tmPrintPeriod_)
    goto tmPrintAccumulate;

timingOffsetPrint:
  tmPrintCnt_ = 0;
  if (!(dsplibs_debug_level > 1))
    goto frameUpdate;
  {
    /* integer and fractional parts of the timing offset in ppm, from separate (unrounded) queries of the resampler */
    float tq = storeF(rs_.getTimingOffsetPPM_LD());
    int ti = dsp_trunc_i32(rs_.getTimingOffsetPPM_LD());
    float tu = storeF((LD)tq - (LD)ti);
    int tw = absI32(dsp_trunc_i32((LD)1000.0f * (LD)tu));
    int tip = dsp_trunc_i32(__builtin_fabsl(rs_.getTimingOffsetPPM_LD()));
    LD last = rs_.getTimingOffsetPPM_LD();
    dsplibs_debug_printf(S_TMOFF, (last <= 0.0L) ? 0x2d : 0x2b, tip, tw);
  }
  goto frameUpdate;

errPrintAccumulate:
  errPrintCnt_ = tmp;

timingPrintCheck:
  tmp = n + tmPrintCnt_;
  if (tmp >= tmPrintPeriod_)
    goto timingOffsetPrint;

tmPrintAccumulate:
  tmPrintCnt_ = tmp;

frameUpdate:
  {
    unsigned s = eqCnt_ + framePos_;
    framePos_ = s - (s / 6) * 6; /* position within the 6-symbol frame */
  }
  /* count the outcome codes the caller acts on */
  {
    int st = (int)state_;
    if (st == V90DEMOD_STATE_RATE_RENEG_DETECTED)
    {
      cnt23_++;
      return;
    }
    if (st < 0x23)
    {
      if ((unsigned)(st - 0x1f) > 2)
        return;
      cnt1f_++;
      return;
    }
    if (st == V90DEMOD_STATE_ENERGY_DROP)
    {
      cnt26_++;
      return;
    }
    return;
  }

  /* ---------------------------------------------------------------- data phases (3 first data call, 4 steady state) */
dataPhaseEntry:
  if (symCnt_ >= (unsigned)params_->minimumDurationInDataBeforeEcRrn)
    enterDataSteadyState();

dataProcess:
  dem_->process(byteBuf_, outCnt);
  scr_.process(byteBuf_, out, outCnt);
  if (state_ == V90DEMOD_STATE_RATE_RENEG_DETECTED)
    goto enterRrnState;

checkEnterPhase4:
  if (state_ == V90DEMOD_STATE_FPE_DETECTED)
    goto enterPhase4OnEvent;

evaluateDataConnection:
  r = ce_->evaluateConnection();
  if (r > 5)
    goto evalDone;
  switch (r)
  {
  case 0:
    goto evalDone;
  case 1:
    goto evalRateUp;
  case 2:
    goto evalRateDown;
  case 3:
    goto evalNoRestrictionRrn;
  case 4:
    goto evalRetrain;
  default:
    goto evalFallBack;
  }

enterRrnState:
  enterRRN();
  goto checkEnterPhase4;

enterPhase4OnEvent: /* state_ == 0x25 (FPE detected): enter phase 4 */
  if (phase_ == V90D_PHASE_4)
    goto evaluateDataConnection;
  if (dsplibs_debug_level > 1)
    dsplibs_debug_printf(S_FPE_ENTERP4);
  symCnt_ = 0;
  timeCnt_ = 0;
  phase_ = V90D_PHASE_4;
  timeLimit_ = 0x10680 + 2 * info_->rtd_;
  errPrintPeriod_ = params_->errorEnergyPrintPeriodPhase4;
  tmPrintPeriod_ = params_->timingOffsetPrintPeriodPhase4;
  goto evaluateDataConnection;

evalFallBack:
  state_ = V90DEMOD_STATE_DROP_TO_V34;
  goto evalDone;

evalRetrain:
  state_ = V90DEMOD_STATE_RETRAIN_REQUESTED;
  goto evalDone;

evalNoRestrictionRrn:
  cdes_->state_ = 0;
  state_ = V90DEMOD_STATE_RATE_RENEG_REQUESTED;
  goto restoreEqualizer;

evalRateDown:
  if (params_->debugConnectionEvaluatorRateDown == 2)
  {
    cdes_->state_ = 1;
  }
  else
  {
    cdes_->state_ = 3;
  }
  state_ = V90DEMOD_STATE_RATE_RENEG_REQUESTED;
  goto restoreEqualizer;

remoteRetrainResult:
  if (retrainOk_ == 0)
  {
    state_ = V90DEMOD_STATE_DROP_TO_V34;
    goto periodicPrints;
  }
  if ((int)phase_ > 2)
  {
    state_ = V90DEMOD_STATE_DROP_TO_V34;
    goto periodicPrints;
  }
  state_ = V90DEMOD_STATE_RETRAIN_REQUESTED;
  goto periodicPrints;

  /* ---------------------------------------------------------------- phase 0 (idle) */
phase0:
  outCnt = 0;
  goto retrainEnergyDetector;

  /* ---------------------------------------------------------------- phase 1 (modem phase 3): spectral verifier and event handlers */
phase1SpectralVerifier:
  outCnt = 0;
  r = sv_.process(in, n);
  if (r == 0)
    goto phase1StateHandlers;
  {
    int f = (int)sv_.flag_;
    if (f == 1)
      goto spectralNt1Box;
    if (f < 1)
    {
      if (f != 0)
        goto spectralVerifierError;
      adid_->setConnectionType(0);
      params_->params->connectionType = 0;
      goto phase1StateHandlers;
    }
    if (f == 2)
      goto spectralGermanPbx;
    if (f == 3)
      goto spectralSevereCodec;
  }

spectralVerifierError:
  edprintf(S_SVERR);

phase1StateHandlers:
  if (state_ > 0x15)
    goto stateHandlersDone;
  switch (state_)
  { /* phase 3 events 0 .. 0x15 */
  case V90DEMOD_STATE_SD_DETECTED:
    goto sdDetectedCapability;
  case V90DEMOD_STATE_TRN1D_STARTED:
    goto trn1dStartedAccumulate;
  case V90DEMOD_STATE_STUDY_UREF_ENTERED:
    goto studyUrefBll;
  case V90DEMOD_STATE_WAIT_FOR_JD_ENTERED:
    goto trn1dEval;
  case V90DEMOD_STATE_JD_DETECTED:
    goto trn1dEval;
  case V90DEMOD_STATE_JDNOT_DETECTED:
    goto jdNotDil;
  case V90DEMOD_STATE_DIL_STUDY_ENDED:
    goto dilStudyEnded;
  case V90DEMOD_STATE_DIL_RECEIVED:
    goto dilReceivedExitPhase3;
  case V90DEMOD_STATE_PROBING_DIL_ENDED:
    goto probingDilTeardown;
  case V90DEMOD_STATE_PHASE3_TERMINATED:
    goto phase3TerminatedEnterPhase4;
  case V90DEMOD_STATE_PHASE3_ERROR:
    goto phase3ErrorLocalRetrain;
  default:
    goto stateHandlersDone;
  }

spectralNt1Box: /* spectral condition 1: German ISDN NT1 box */
  pf_.setFilter(params_->germanIsdnNt1BoxFilterGain);
  edprintf(S_PREFILTER, pf_.filterIdx_);
  copyWord(&params_->linearEquDataBeta, &params_->germanIsdnNt1LinearEquDataBeta);
  adid_->setConnectionType(1);
  params_->params->connectionType = 1;
  goto phase1StateHandlers;

spectralGermanPbx: /* spectral condition 2: German PBX */
  pf_.setFilter(params_->germanPbxPreFilterGain);
  edprintf(S_PREFILTER, pf_.filterIdx_);
  copyWord(&params_->linearEquDataBeta, &params_->germanPbxLinearEquDataBeta);
  copyWord(&params_->dfeDataBeta, &params_->germanPbxDfeDataBeta);
  copyWord(&params_->linearEquDilBeta, &params_->germanPbxLinearEquDilBeta);
  copyWord(&params_->linearEquDilMedUcodeBeta, &params_->germanPbxLinearEquDilMedUcodeBeta);
  copyWord(&params_->linearEquDilHighUcodeBeta, &params_->germanPbxLinearEquDilHighUcodeBeta);
  adid_->setConnectionType(2);
  params_->params->connectionType = 2;
  goto phase1StateHandlers;

spectralSevereCodec: /* spectral condition 3: severe codec */
  if (pf_.isV90WithEia6())
    goto severeCodecEia6;
  if (params_->enableDrop2V34OnSevereCodec == 0)
    goto severeCodecMasked;
  edprintf(S_SEVERE_DROP);
  state_ = V90DEMOD_STATE_DROP_TO_V34;
  goto phase1StateHandlers;

severeCodecEia6:
  edprintf(S_SEVERE_EIA6);
  goto phase1StateHandlers;

severeCodecMasked:
  edprintf(S_SEVERE_MASKED);
  goto phase1StateHandlers;

sdDetectedCapability: /* state 1 */
  if (pf_.getV90Capability() != 0)
    goto stateHandlersDone;
  state_ = V90DEMOD_STATE_DROP_TO_V34;
  if (dsplibs_debug_level > 1)
    dsplibs_debug_printf(S_FALLBACK);
  goto stateHandlersDone;

trn1dStartedAccumulate: /* state 3 */
  sv_.startAccumulation();
  if (retrainOk_ == 0)
  {
    copyWord(&agc_.mix_, &params_->agcK);
    edprintf(S_AGCACTIVATED);
    goto stateHandlersDone;
  }
  flag40_ = 1;
  goto bllStateMachine;

studyUrefBll: /* state 4 */
  if (retrainOk_ != 0)
    goto bllStateMachine;
  rs_.setBllState(V90BLL_SLOW, 1);
  goto stateHandlersDone;

trn1dEval: /* states 5, 6 */
  if (params_->probingMode != 0)
    goto stateHandlersDone;
  if (retrainOk_ != 0)
  {
    if ((short)adid_->isThereAnyAltRbsPhase() != 0)
    {
      ce_->altRbsDetectedOnQc_ = 1;
      edprintf(S_SETALTRBS);
    }
    edprintf(S_TRN1D_NOTENABLED);
    goto stateHandlersDone;
  }
  ce_->trn1dEvalActive_ = 1;
  ce_->weight_ = 0;
  ce_->avePdsnr_ = 0.0f;
  edprintf(S_ENABLE_TRN1D);
  goto stateHandlersDone;

jdNotDil: /* state 8 */
  if (params_->probingMode == 0)
  {
    ce_->phase3EvalActive_ = 0;
    ce_->trn1dEvalActive_ = 0;
    ce_->weight_ = 0;
    ce_->avePdsnr_ = 0.0f;
    edprintf(S_DISABLE_P3);
  }
  if (session_ == 0)
    maxLookahead = jd_->getMaxLookahead();
  else
    maxLookahead = jd92_->getMaxLookahead();
  edprintf(S_JDMAX, (unsigned)maxLookahead);
  if (params_->probingMode == 0)
  {
    rs_.setBllState(V90BLL_DIL, 1);
    goto stateHandlersDone;
  }
  rs_.setBllState(V90BLL_FROZEN, 1);
  goto stateHandlersDone;

dilStudyEnded: /* state 0x11: DIL study ended */
  if (dsplibs_debug_level > 1)
    dsplibs_debug_printf(S_DILSTUDY_END);
  p3_->setDigitalImairmentsInfo();
  {
    float* g = &agc_.gain_;
    float padGain = adid_->padGain_;
    dilScale_ = padGain;         /* remembered for exitPhase3(), which reports it as the TRN1d RMS */
    LD q = (LD)*g / (LD)padGain; /* the print uses the unrounded quotient */
    *g = (float)q;
    edFix(S_AGCGAIN, q, (LD)1000000.0f);
  }
  p3_->flag3f9_ = 1;
  ce_->phase3EvalActive_ = 0;
  ce_->trn1dEvalActive_ = 0;
  ce_->weight_ = 0;
  ce_->avePdsnr_ = 0.0f;
  goto stateHandlersDone;

dilReceivedExitPhase3: /* state 0x12 */
  exitPhase3();
  goto stateHandlersDone;

probingDilTeardown: /* state 0x13 */
  if (params_->probingMode == 0)
    goto stateHandlersDone;
  if (dsplibs_debug_level > 1)
  {
    dsplibs_debug_printf(S_BAR63);
    if (dsplibs_debug_level > 1)
    {
      dsplibs_debug_printf(S_TEARDOWN);
      if (dsplibs_debug_level > 1)
        dsplibs_debug_printf(S_BAR63);
    }
  }
  state_ = V90DEMOD_STATE_TERMINATE_SESSION;
  goto stateHandlersDone;

phase3TerminatedEnterPhase4: /* state 0x14 */
  if (phase_ == V90D_PHASE_4)
    goto stateHandlersDone;
  if (dsplibs_debug_level > 1)
    dsplibs_debug_printf(S_ENTERP4);
  phase_ = V90D_PHASE_4;
  timeCnt_ += symCnt_;
  symCnt_ = 0;
  timeLimit_ = 0x28230 + 5 * info_->rtd_;
  errPrintPeriod_ = params_->errorEnergyPrintPeriodPhase4;
  tmPrintPeriod_ = params_->timingOffsetPrintPeriodPhase4;
  goto stateHandlersDone;

phase3ErrorLocalRetrain: /* state 0x15 */
  r = ce_->indicateLocalRetrain();
  if (r == 4)
  {
    state_ = V90DEMOD_STATE_RETRAIN_REQUESTED;
    goto stateHandlersDone;
  }
  if (r != 5)
    goto stateHandlersDone;
  state_ = retrainState(retrainOk_);
  goto bllStateMachine;

  /* ---------------------------------------------------------------- resampler timing-loop (BLL) state machine, Agc freeze, phase 3 evaluation */
stateHandlersDone:
bllStateMachine:
  retrainFlag = retrainOk_;
  bllState = rs_.state_;
  bllCount = rs_.stateCount_;
  if (bllState == V90BLL_INITIAL && retrainFlag == 0)
  {
    if ((int)params_->bllTrn1dInitialToFastDuration < (int)bllCount)
    {
      rs_.setBllState(V90BLL_FAST, 1);
      retrainFlag = retrainOk_;
    }
  }
  if (bllState == V90BLL_FAST && retrainFlag == 0)
  {
    if ((int)params_->bllTrn1dFastToSlowDuration < (int)bllCount)
    {
      rs_.setBllState(V90BLL_MEDIUM, 1);
      retrainFlag = retrainOk_;
    }
  }
  if (bllState == V90BLL_SLOW && retrainFlag == 0)
  {
    if ((int)params_->bllSlowToSlow2Duration < (int)bllCount)
    {
      rs_.setBllState(V90BLL_SLOW2, 1);
      retrainFlag = retrainOk_;
    }
  }
  if (retrainFlag != 0 && bllState == V90BLL_TRN1_QC_INITIAL)
  {
    if ((int)params_->bllTrn1QcInitialToFastDuration < (int)bllCount)
    {
      rs_.setBllState(V90BLL_TRN1_QC_FAST, 1);
      retrainFlag = retrainOk_;
    }
  }
  if (retrainFlag != 0 && bllState == V90BLL_TRN1_QC_FAST)
  {
    if ((int)params_->bllTrn1QcFastToMediumDuration < (int)bllCount)
    {
      rs_.setBllState(V90BLL_TRN1_QC_MEDIUM, 1);
      retrainFlag = retrainOk_;
    }
  }
  if (retrainFlag != 0 && bllState == V90BLL_TRN1_QC_MEDIUM)
  {
    if ((int)params_->bllTrn1QcMediumToSlowDuration < (int)bllCount)
      rs_.setBllState(V90BLL_TRN1_QC_SLOW, 1);
  }
  /* during TRN1d known data, once the Agc has adapted long enough and its mix differs from 1.0 (a NaN counts as equal), freeze it and derive the DIL
   * limits from its gain */
  if (p3_->state_ == P3D_TRN1D_KNOWN)
  {
    if ((int)params_->agcAdaptationDuration < (int)p3_->cnt_ && ((LD)agc_.mix_ < 1.0L || (LD)agc_.mix_ > 1.0L))
    {
      agc_.freeze();
      edprintf(S_AGCFROZEN);
      edFix(S_AGCENERGY, (LD)agc_.power_, (LD)100.0f);
      edFix(S_AGCGAIN, (LD)agc_.gain_, (LD)1000000.0f);
      if (retrainOk_ == 0)
      {
        if (!pf_.isV90WithEia6())
          rs_.adjustHalfBaudBpfGain(WORDARG(agc_.gain_));
      }
      flag40_ = 1;
      {
        LD gain = (LD)agc_.gain_;
        LD u6c = (LD)params_->unk6c;
        if (!jmpBe(u6c, gain))
        { /* gain below the first limit (unk6c) */
          LD prod = u6c * (LD)0.85;
          if (jmpBe(prod, gain))
          { /* gain at least 85% of the limit: compute the equalizer divisor word; flag400_ is not set on this path */
            LD one = 1.0L;
            LD d = one / (LD)params_->unk6c;
            LD v = (one - d * (LD)agc_.gain_) * (LD)250.0f;
            unsigned short w = (unsigned short)trunc16(v);
            eq_->w8_ = (unsigned short)(w + 1);
          }
          else
          {
            p3_->flag400_ = 1;
          }
          p3_->impairInfo_ = 0x74;
          if (jmpAe((LD)agc_.gain_, (LD)params_->unk70))
          { /* low gain: DIL overflow protection (message only) */
            if (dsplibs_debug_level > 1)
              dsplibs_debug_printf(S_DILOVF);
          }
          else
          {
            params_->unk300 = params_->unk300 - params_->unk304; /* very low gain: extreme overflow protection lowers the parameter */
            if (dsplibs_debug_level > 1)
              dsplibs_debug_printf(S_DILOVF2);
          }
        }
        else
        {
          p3_->impairInfo_ = 0x74;
        }
      }
      edprintf(S_DILMAX, (unsigned)p3_->impairInfo_);
    }
  }
  r = ce_->evaluatePhase3();
  goto evalResult;

evalResult: /* connection-evaluator result: 4 retrain, 5 retrain or drop to V.34 */
  if (r == 4)
  {
    state_ = V90DEMOD_STATE_RETRAIN_REQUESTED;
    goto retrainEnergyDetector;
  }
  if (r != 5)
    goto retrainEnergyDetector;
  state_ = retrainState(retrainOk_);
  goto retrainEnergyDetector;

  /* ---------------------------------------------------------------- phase 2 (modem phase 4): timeout and event handlers */
phase4Entry:
  outCnt = 0;
  tmp = n + timeCnt_;
  timeCnt_ = tmp;
  if (tmp <= timeLimit_)
    goto phase4EventSwitch;
  if (dsplibs_debug_level > 1)
    dsplibs_debug_printf(S_P4TIMEOUT);
  if (ce_->rrnSilence_ != 0 && p4_->rrnPending_ != 0 && hwCodec_ != 4)
  {
    edprintf(S_SILRRN_NOTFIN);
    params_->rrnSilenceRequested = 0;
  }
  params_->sensitiveIspDetected = 1;
  params_->silenceScr = 0;
  edprintf(S_SENSITIVE);
  r = ce_->indicateLocalRetrain();
  if (r == 4)
  {
    state_ = V90DEMOD_STATE_RETRAIN_REQUESTED;
    goto phase4EventSwitch;
  }
  if (r != 5)
    goto phase4EventSwitch;
  state_ = retrainState(retrainOk_);

phase4EventSwitch:
  if (state_ - 0x19 > 0x1c)
    goto phase4Evaluate;
  switch (state_ - 0x19)
  { /* phase 4 events 0x19 .. 0x35 */
  case 0:
    goto onTrn2dReceived; /* 0x19 TRN2D_RECEIVED (the case labels are relative to 0x19) */
  case 1:
    goto onMpResult; /* 0x1a MP_DETECTED */
  case 2:
    goto onMpResult; /* 0x1b MPNOT_DETECTED */
  case 3:
    goto onEdDetected; /* 0x1c ED_DETECTED */
  case 4:
    goto onPhase4Terminated; /* 0x1d PHASE4_TERMINATED */
  case 15:
    goto onRtNotDetected; /* 0x28 RTNOT_DETECTED */
  case 17:
    goto onWaitForRtEntered; /* 0x2a WAIT_FOR_RT_ENTERED */
  case 24:
    goto onSilenceRrnSuvPart1; /* 0x31 SILENCE_RRN_SUV_PART1 */
  case 26:
    goto onSilenceRrnSuvtPart1; /* 0x33 SILENCE_RRN_SUVT_PART1 */
  case 28:
    goto onFirstRrnEdDetected; /* 0x35 FIRST_RRN_ED_DETECTED */
  default:
    goto phase4Evaluate;
  }

onSilenceRrnSuvtPart1: /* event 0x33 */
  flag270_ = 1;
  goto phase4Evaluate;

onSilenceRrnSuvPart1: /* event 0x31 */
  flag270_ = 1;

phase4Evaluate:
  r = ce_->evaluatePhase4(WORDARG(eq_->ratio_));
  goto evalResult;

onPhase4Terminated: /* event 0x1d */
  enterDataPhase();
  dem_->process(byteBuf_, outCnt);
  scr_.process(byteBuf_, out, outCnt);
  goto phase4Evaluate;

onRtNotDetected: /* event 0x28 */
  edprintf(S_RTNOT);
  rs_.setBllState(V90BLL_TRN2, 1);
  flag40_ = 1;
  goto phase4Evaluate;

onFirstRrnEdDetected: /* event 0x35 */
  edprintf(S_FREEZING);
  rs_.setBllState(V90BLL_FROZEN, 1);
  flag40_ = 0;
  goto phase4Evaluate;

onEdDetected: /* event 0x1c */
  if (ce_->rrnSilence_ != 0 && p4_->rrnPending_ != 0 && p4_->rtPending_ != 0)
  { /* silence RRN with the RRN and Rt pending: freeze the timing loop and print the means */
    edprintf(S_FREEZING);
    rs_.setBllState(V90BLL_FROZEN, 1);
    flag40_ = 0;
    edMeans(S_EDRECV, (LD)eq_->meanErr_, (LD)eq_->statMean_); /* current mean error, then the equalizer's average */
    copyWord(&designErr_, &eq_->statMean_);
  }
  flag270_ = 1;
  goto phase4Evaluate;

onMpResult: /* events 0x1a / 0x1b */
  if (p4_->state_ != P4D_WAIT_MP)
    goto phase4Evaluate;
  if (params_->maskRrnSilenceOnProblematicIsp != 0 && params_->rrnSilenceRequested != 0 && (mp_->trellis_ | mp_->nonLin_) == 0 && mp_->shaping_ == 0 &&
      mp_->type_ == 1 && (mp_->h1Re_ | mp_->h1Im_) == 0 && (mp_->h2Re_ | mp_->h2Im_) == 0 && (mp_->h3Re_ | mp_->h3Im_) == 0)
  { /* problematic ISP modem recognised from the MP: mask the silence RRN, or on USB limit the retrains */
    edprintf(S_BAR75);
    if (hwCodec_ == 4)
    {
      params_->maxNofV90Retrains = 0x96;
      edprintf(S_PROBLEM_USB, 0x96);
      ce_->retrainInsteadOfRateDown_ = 1;
    }
    else
    {
      params_->rrnSilenceRequested = 0;
      edprintf(S_PROBLEM_NOTUSB);
    }
    edprintf(S_BAR75);
    p4_->enterWaitForEd();
    goto phase4Evaluate;
  }
  edprintf(S_SILRRN_NOTMASKED, params_->rrnSilenceRequested);
  p4_->enterWaitForEd();
  goto phase4Evaluate;

onWaitForRtEntered: /* event 0x2a: redesign on a silence RRN */
  {
    /* current bit rate = round(kBits * 8000 / 6) */
    unsigned prod = (unsigned)(mp2_->kBits_ * 0x1f40u);
    prevRate = trunc64lo((LD)0.5f + (LD)(unsigned long long)prod * (LD)0.1666666716337204f);
  }
  edprintf(S_PREVRATE, prevRate);
  if (ce_->noRestrictionRrn_ != 0)
  { /* unrestricted RRN: force a rate reduction */
    cdes_->state_ = 3;
    edprintf(S_FORCED_RATEDOWN);
    goto designCall;
  }
  if (p4_->keepRate_ != 0 && (unsigned)params_->minRateForSilenceRrnKeepRate >= prevRate)
  { /* keep the rate when it is already at or below the configured floor */
    cdes_->state_ = 1;
    edprintf(S_KEEPING);
    goto designCall;
  }
  if (params_->debugConnectionEvaluatorRateDown == 2)
  { /* debug override: keep the rate */
    cdes_->state_ = 1;
    edprintf(S_FORCED_KEEP);
    goto designCall;
  }
  cdes_->state_ = 3;
  edprintf(S_RATEDOWN);

designCall:
  edMeans(S_ABOUTREDESIGN, (LD)eq_->meanErr_, (LD)eq_->statMean_);
  /* design the constellation again; the noise estimate is the mean error remembered in designErr_ */
  tmp = ce_->retrainInsteadOfRateDown_;
  spectralCond = sv_.flag_;
  maxUcode = p3_->getMaxUcode();
  if (session_ == 0)
    rateMask = (unsigned)jd_->getRatesMask();
  else
    rateMask = (unsigned)jd92_->getRatesMask();
  if (session_ == 0)
    maxLookahead = jd_->getMaxLookahead();
  else
    maxLookahead = jd92_->getMaxLookahead();
  r = (unsigned)cdes_->process((unsigned)maxLookahead, adid_, WORDARG(designErr_), (int)rateMask, mp2_, (short (*)[128])adid_->linearMapping_,
                               (short (*)[128])adid_->linearMappingAlt_, (short*)adid_->altRbsPhase_, adid_->suspectedPhase_, maxUcode,
                               (unsigned char)(info_->maxTx_ + 1), hwCodec_, tmp, (V90SpecialSpectralConditions)spectralCond);
  ce_->enableRrnDown_ = params_->enableRrnDown;
  ce_->enableRrnUp_ = params_->enableRrnUp;
  designed_ = 1;
  if (r == 1)
  { /* the design failed */
    if (dsplibs_debug_level > 1)
      dsplibs_debug_printf(S_DESIGNERR);
    r = ce_->indicateLocalRetrain();
    if (r == 5)
    {
      state_ = retrainState(retrainOk_);
    }
    else
    {
      state_ = V90DEMOD_STATE_ENERGY_DROP;
    }
  }
  ce_->updateCurrentConstellationData((short)cdes_->dMin_, WORDARG(cdes_->threshUp_), WORDARG(cdes_->threshDown_), WORDARG(cdes_->threshRetrain_));
  goto phase4Evaluate;

onTrn2dReceived: /* event 0x19: design the data constellation from the mean error */
  me = storeF((LD)eq_->calcMeanErrorStatistics());
  mw0 = me;
  mw1 = wordOf(eq_->statMean_);
  if (ce_->rrnSilence_ != 0 && p4_->rrnPending_ != 0)
  { /* silence RRN: design now, then wait for CP / MP */
    edprintf(S_CONSTDESIGN_SIL);
    cpInfo_->mode_ = 1;
    if (session_ != 0)
      p4_->enterWaitForCP();
    else
      p4_->enterWaitForMP();
    goto phase4Evaluate;
  }
  if (pf_.isV90WithEia6())
    mw0 = 0.0f;
  r = ce_->evaluateMeanErrorStdPhase4(WORDARG(mw0), floatOf(mw1));
  if (r == 4)
    goto toRetrain;
  if (r == 5)
    goto retrainOrFallBack;
  tmp = ce_->retrainInsteadOfRateDown_;
  spectralCond = sv_.flag_;
  maxUcode = p3_->getMaxUcode();
  if (session_ == 0)
    rateMask = (unsigned)jd_->getRatesMask();
  else
    rateMask = (unsigned)jd92_->getRatesMask();
  if (retrainOk_ != 0 && cdes_->w24_ == 0)
  {
    noise = storeF((LD)params_->unk328 * (LD)eq_->statMean_);
  }
  else
  {
    noise = storeF((LD)eq_->statMean_);
  }
  if (session_ == 0)
    maxLookahead = jd_->getMaxLookahead();
  else
    maxLookahead = jd92_->getMaxLookahead();
  r = (unsigned)cdes_->process((unsigned)maxLookahead, adid_, noise, (int)rateMask, mp2_, (short (*)[128])adid_->linearMapping_,
                               (short (*)[128])adid_->linearMappingAlt_, (short*)adid_->altRbsPhase_, adid_->suspectedPhase_, maxUcode,
                               (unsigned char)(info_->maxTx_ + 1), hwCodec_, tmp, (V90SpecialSpectralConditions)spectralCond);
  ce_->enableRrnDown_ = params_->enableRrnDown;
  ce_->enableRrnUp_ = params_->enableRrnUp;
  designed_ = 1;
  if (r != 1)
  { /* the design succeeded */
    edprintf(S_DATASPECTRAL);
    displaySpectralParams(mp2_);
    cpInfo_->mode_ = 1;
    if (session_ != 0)
      p4_->enterWaitForCP();
    else
      p4_->enterWaitForMP();
    ce_->updateCurrentConstellationData((short)cdes_->dMin_, WORDARG(cdes_->threshUp_), WORDARG(cdes_->threshDown_), WORDARG(cdes_->threshRetrain_));
    goto phase4Evaluate;
  }
  if (dsplibs_debug_level > 1)
    dsplibs_debug_printf(S_DESIGNERR);
  r = ce_->indicateLocalRetrain();
  if (r == 5)
    goto retrainOrFallBack;
  state_ = V90DEMOD_STATE_ENERGY_DROP;
  goto phase4Evaluate;

toRetrain:
  state_ = V90DEMOD_STATE_RETRAIN_REQUESTED;
  goto phase4Evaluate;

retrainOrFallBack:
  if (retrainOk_ != 0)
    goto toRetrain;
  state_ = V90DEMOD_STATE_DROP_TO_V34;
  goto phase4Evaluate;
}
