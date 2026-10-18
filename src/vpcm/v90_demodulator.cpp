/**
 * @file v90_demodulator.cpp
 * @brief V90Demodulator: construction, destruction, phase entry / exit methods, reset and accessors (progress() is in v90_demodulator_progress.cpp).
 *
 * Contents:
 * - the constructor, which constructs the embedded Agc, V90PreFilter, V90Resampler(100, 60.0f, 16, 0.98f, params, 0.0f, 0), V90ConstellationPower,
 *   Descrambler(0x12, 0x17, 0x63) and V90SpectralVerifier in its initialiser list and then allocates the receiver chain (ctorBody()): five sample buffers
 *   sized from the block length `n` and the heap objects (AutoDigitalImpDetector, ConnectionEvaluator, Phase3 / Phase4 demodulators, Demapper, Equalizer,
 *   TRN2Designer, ConstellationDesigner), each sysdep_malloc()ed and constructed in place; the destructor (dtorBody()) calls sessionTermination() and
 *   destroys them with an explicit destructor call and sysdep_free();
 * - the phase transitions: enterPhase3() / exitPhase3() (the TRN2d constellation design), enterPhase4() / enterRRN() / enterFPE(), enterDataPhase(),
 *   enterDataSteadyState(), enterChannelVerification(), reInit(), reset(), setSessionFlag();
 * - sessionTermination() and enterDataSteadyState(), which print the resampler's timing-offset history (mean, standard deviation) and, when the deviation is
 *   small enough, store 1000 * mean in the host modem parameter block (clockDeviation) so that the next call starts from it;
 * - the small accessors getBitRate(), getRbsPattern(), indicateRemoteRateReneg() and getAT_UD() (diagnostics record, v90_diagnostics.h).
 *
 * The callees are the member classes of V90Demodulator (declared through v90_demodulator.h) plus edprintf(), dsplibs_debug_printf() and
 * displaySpectralParams().
 *
 * Numeric notes:
 * - The "%c%d.%0Nd" prints take their digits from a long-double value with truncation and the sign from a comparison with zero ('+' when 0 < v or v is
 *   unordered; splitValue(), edFix()).
 * - The timing-history mean and standard deviation come back from the resampler as long double and are rounded to float exactly once (storeF()).
 * - Floats that are only moved (initialBaudOffset -> setTimingOffset, agcNominalEnergy -> agc_.target_) are copied bit-for-bit (copyWord / WORDARG).
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

/* ------------------------------------------------------------------------------------------------ message texts */
#define S_DIAG_HDR "V90 Diagnostics\r\n"
#define S_DIAG_BAR "-------------------------\r\n"
#define S_DIAG_RBS "RBS : %d (%d%d%d%d%d%d)\r\n"
#define S_ST_NOTSAVED "V90Demodulator on sessionTermination: Timing offset NOT saved to registry (isDataState = %d, isEia6 = %d)\r\n"
#define S_ST_MEAN "V90Demodulator on sessionTermination: mean of timing offset History  = %c%d.%04d\r\n"
#define S_ST_STD "V90Demodulator on sessionTermination: std of timing offset History  = %c%d.%04d\r\n"
#define S_ST_STD1000 "V90Demodulator on sessionTermination: 1000* std = %d\r\n"
#define S_ST_SAVED "V90Demodulator on sessionTermination: Timing offset saved in Registry!\r\n"
#define S_ST_DISABLED "V90Demodulator on sessionTermination: Timing offset NOT saved to registry, EVALUATION DISABLED !\n"
#define S_ENTER_P4 "V90Demodulator: enter Phase 4\r\n"
#define S_ENTER_STEADY "V90Demodulator: enter Data steady state\r\n"
#define S_NOLINMAP_STUDY "V90Demodulator: disable linear mapping study.\n"
#define S_MEAN "V90Demodulator: mean of timing offset History  = %c%d.%04d\r\n"
#define S_STD "V90Demodulator: std of timing offset History  = %c%d.%04d\r\n"
#define S_STD1000 "V90Demodulator: 1000* std = %d\r\n"
#define S_SAVED "V90Demodulator: Timing offset saved in Registry!\r\n"
#define S_RRN_P4 "V90Demodulator: RRN detected: enter Phase 4\r\n"
#define S_NOLINMAP "V90Demodulator: disable linear mapping study\n"
#define S_FPE_P4 "V90Demodulator: FPE detected: enter Phase 4\r\n"
#define S_SESSIONFLAG "V90Demodulator: setSessionFlag, flag = %d\r\n"
#define S_ENTER_P3 "V90Demodulator: enter Phase 3\r\n"
#define S_V92LITE "V90Demodulator: we got PCM upstream under V.92Lite, retraining to V.34 upstream...\r\n"
#define S_TRN1D_RMS "V90Demodulator: TRN1d RMS Ratio = %c%d.%08d\r\n"
#define S_TRN2D_SPECTRAL "V90Demodulator: TRN2d spectral parameters:\r\n"
#define S_DELAYED_RETRAIN "V90Demodulator::exitPhase3() delayedRetrainRequest !!!\r\n"
#define S_ENTER_DATA "V90Demodulator: enter Data Phase, Rate = %d [bps]\r\n"
#define S_ENABLE_LINMAP "V90Demodulator: reset and enable linear mapping study in data\n"
#define S_RESET_BAUD "V90Demodulator reset: Baud Offset = %c%d.%03d\r\n"
#define S_RESET_QC "V90Demodulator reset, quick connect flag = %d\r\n"
#define S_ENTER_CHANVER "V90Demodulator: Enter Channel Verification called !\r\n"

/* ------------------------------------------------------------------------------------------------ helpers */

#include "core/dsp_x87_cmp.h"

#include "core/dsp_intmath.h"

#include "vpcm_split.h"
#include "vpcm_split_value.h"

#include "core/dsp_floatbits.h"

#include "core/dsp_x87_hide.h"

#include "core/dsp_copyword_raw.h"

#include "core/dsp_wordarg.h"

/** Low 32 bits of the truncated 64-bit integer of `v`; `v` is always in [0.5, 2^32 / 6 + 0.5] here (the rate 0.5 + u32 / 6), so it never overflows. */
static inline unsigned trunc64lo(LD v) { return (unsigned)(unsigned long long)(long long)v; }

/**
 * @brief Current bit rate in bit/s: 0 until a constellation has been designed, else round(w0 * 8000 / 6) with w0 = mp2->kBits_ (bits per 6-symbol frame).
 * @param designed  V90Demodulator::designed_
 * @param w0        bits per frame of the data constellation
 */
static inline unsigned bitRate(unsigned char designed, unsigned w0)
{
  if (designed == 0)
    return 0;
  unsigned x = w0 * 0x1f40u;
  return trunc64lo((LD)0.5f + (LD)(unsigned long long)x * (LD)0.1666666716337204f);
}

/**
 * Phase 4 entry shared by enterPhase4() and exitPhase3(): phase_ = 4, the symbols counted so far move to timeCnt_, the timeout becomes 5 * rtd + 0x28230
 * samples and the phase 4 print periods are set.
 */
static inline void phase4Common(V90Demodulator* d)
{
  d->phase_ = V90D_PHASE_4;
  unsigned s = d->symCnt_;
  V90Phase2Info* info = d->info_;
  d->timeCnt_ += s;
  V90Parameters* p = d->params_;
  unsigned rtd = info->rtd_;
  d->symCnt_ = 0;
  d->timeLimit_ = rtd * 5u + 0x28230u;
  d->errPrintPeriod_ = (unsigned)p->errorEnergyPrintPeriodPhase4;
  d->tmPrintPeriod_ = (unsigned)p->timingOffsetPrintPeriodPhase4;
}

/**
 * @brief Print the timing-offset history statistics and store the mean for the next session.
 * @param p       parameter block; timingOffesetMinStdForSave is the largest standard deviation for which the mean is saved
 * @param mean    mean timing offset of the resampler's history
 * @param stdv    its standard deviation
 * @param fMean   format for the mean ("%c%d.%04d")
 * @param fStd    format for the standard deviation
 * @param f1000   format for 1000 * stdv
 * @param fSaved  message printed when the value is stored
 * @note  The store writes (int)(1000 * mean) to the word clockDeviation of the host modem parameter block (the host persists it); it is skipped when the
 *        deviation exceeds the limit or either value is NaN.
 */
static inline void timingStats(V90Parameters* p, float mean, float stdv, const char* fMean, const char* fStd, const char* f1000, const char* fSaved)
{
  edFix(fMean, (LD)mean, (LD)10000.0f);
  edFix(fStd, (LD)stdv, (LD)10000.0f);
  edprintf(f1000, dsp_trunc_i32((LD)stdv * (LD)1000.0f));
  if (!jmpB((LD)p->timingOffesetMinStdForSave, (LD)stdv))
  { /* skipped when min < std or unordered */
    edprintf(fSaved);
    int v = dsp_trunc_i32((LD)1000.0f * (LD)mean);
    p->params->clockDeviation = v;
  }
}

/* ------------------------------------------------------------------------------------------------ constructor / destructor bodies */

/**
 * @brief Constructor body: store the borrowed pointers, allocate the sample buffers and the receiver chain, clear the counters.
 * @warning None of the sysdep_malloc() results is checked for NULL.
 * @note  Buffer sizes are fixed by `n`: inBuf_ n floats, rsBuf_ 3 * n floats, decBuf_ n * 2 shorts, eqBuf_ 2 * n floats, byteBuf_ 8 * n bytes.  The objects are
 *        constructed in dependency order (adid, ce, p3, dem, p4, eq, trn2d, cdes) because each takes pointers to the earlier ones.
 */
static inline void ctorBody(V90Demodulator* d, unsigned n, V90Phase2Info* info, V90Jd* jd, V92Jd* jd92, tagV90DILdescriptor* dil, V90MappingParams* mp1,
                            V90MappingParams* mp2, tagV90AdditionalCPinfo* cpInfo, V90CP* cp, V90MP* mp, __tHardwareCodecTypes__ codec, V90Parameters* params,
                            V90ComputationalMode mode, unsigned session)
{
  d->params_ = params;
  d->jd_ = jd;
  d->info_ = info;
  d->mp1_ = mp1;
  d->jd92_ = jd92;
  d->dil_ = dil;
  d->mp2_ = mp2;
  d->cpInfo_ = cpInfo;
  d->mp_ = mp;
  d->cp_ = cp;
  d->hwCodec_ = codec;
  d->session_ = session;

  /* sample buffers */
  d->inBuf_ = (float*)sysdep_malloc(n * 4u);
  d->rsBuf_ = (float*)sysdep_malloc((n + n + n) << 2);
  d->decBuf_ = (short*)sysdep_malloc(n * 4u);
  d->eqBuf_ = (float*)sysdep_malloc(n << 3);
  d->byteBuf_ = (unsigned char*)sysdep_malloc(n << 3);
  d->eqCnt_ = 0;
  d->rsCnt_ = 0;

  /* receiver chain */
  {
    V90AutoDigitalImpDetector* a = (V90AutoDigitalImpDetector*)sysdep_malloc(sizeof(V90AutoDigitalImpDetector));
    new (a) V90AutoDigitalImpDetector(d->params_);
    d->adid_ = a;
  }
  {
    V90ConnectionEvaluator* c = (V90ConnectionEvaluator*)sysdep_malloc(sizeof(V90ConnectionEvaluator));
    new (c) V90ConnectionEvaluator(d->params_);
    d->ce_ = c;
  }
  {
    V90Phase3Demodulator* p3 = (V90Phase3Demodulator*)sysdep_malloc(sizeof(V90Phase3Demodulator));
    new (p3) V90Phase3Demodulator(d->params_, &d->sv_, d->session_, d->adid_);
    d->p3_ = p3;
  }
  {
    V90Demapper* dm = (V90Demapper*)sysdep_malloc(sizeof(V90Demapper));
    new (dm) V90Demapper(n + n, d->params_, d->adid_);
    d->dem_ = dm;
  }
  {
    V90Phase4Demodulator* p4 = (V90Phase4Demodulator*)sysdep_malloc(sizeof(V90Phase4Demodulator));
    new (p4) V90Phase4Demodulator(d->mp1_, d->mp2_, d->dem_, d->cp_, d->mp_, &d->scr_, d->ce_, d->params_, d->p3_, d->adid_, d->session_);
    d->p4_ = p4;
  }
  {
    V90Equalizer* e = (V90Equalizer*)sysdep_malloc(sizeof(V90Equalizer));
    new (e) V90Equalizer((unsigned)d->params_->linearEquLength, (unsigned)d->params_->dfeLength, d->p3_, d->p4_, d->dem_, d->ce_, &d->sv_, d->params_, &d->rs_,
                         &d->pf_, mode);
    d->eq_ = e;
  }
  {
    V90TRN2Designer* t = (V90TRN2Designer*)sysdep_malloc(sizeof(V90TRN2Designer));
    new (t) V90TRN2Designer(d->params_, &d->cpow_);
    d->trn2d_ = t;
  }
  {
    V90ConstellationDesigner* cd = (V90ConstellationDesigner*)sysdep_malloc(sizeof(V90ConstellationDesigner));
    new (cd) V90ConstellationDesigner(d->params_, &d->pf_, &d->cpow_);
    d->cdes_ = cd;
  }

  d->cnt23_ = 0;
  d->cnt26_ = 0;
  d->cnt1f_ = 0;
  d->timingHistoryEval_ = 0;
  d->designed_ = 0;
  d->quietCnt_ = 0;
}

V90Demodulator::V90Demodulator(unsigned n, V90Phase2Info* info, V90Jd* jd, V92Jd* jd92, tagV90DILdescriptor* dil, V90MappingParams* mp1, V90MappingParams* mp2,
                               tagV90AdditionalCPinfo* cpInfo, V90CP* cp, V90MP* mp, __tHardwareCodecTypes__ codec, V90Parameters* params,
                               V90ComputationalMode mode, unsigned session)
    : agc_(), pf_(codec, info, params), rs_(100, 60.0f, 0x10, 0.98f, params, 0.0f, 0), cpow_(), scr_(0x12, 0x17, 0x63), sv_(params)
{
  ctorBody(this, n, info, jd, jd92, dil, mp1, mp2, cpInfo, cp, mp, codec, params, mode, session);
}

/** Destructor body: end-of-session report, then destroy and free the owned objects and buffers (members that are NULL are skipped). */
static inline void dtorBody(V90Demodulator* d)
{
  d->sessionTermination();

  if (d->eq_)
  {
    d->eq_->~V90Equalizer();
    sysdep_free(d->eq_);
  }
  if (d->p3_)
  {
    d->p3_->~V90Phase3Demodulator();
    sysdep_free(d->p3_);
  }
  if (d->p4_)
  {
    d->p4_->~V90Phase4Demodulator();
    sysdep_free(d->p4_);
  }
  if (d->dem_)
  {
    d->dem_->~V90Demapper();
    sysdep_free(d->dem_);
  }
  if (d->trn2d_)
  {
    d->trn2d_->~V90TRN2Designer();
    sysdep_free(d->trn2d_);
  }
  if (d->cdes_)
  {
    d->cdes_->~V90ConstellationDesigner();
    sysdep_free(d->cdes_);
  }
  if (d->adid_)
  {
    d->adid_->~V90AutoDigitalImpDetector();
    sysdep_free(d->adid_);
  }
  if (d->ce_)
  {
    d->ce_->~V90ConnectionEvaluator();
    sysdep_free(d->ce_);
  }

  if (d->inBuf_)
    sysdep_free(d->inBuf_);
  if (d->rsBuf_)
    sysdep_free(d->rsBuf_);
  if (d->decBuf_)
    sysdep_free(d->decBuf_);
  if (d->eqBuf_)
    sysdep_free(d->eqBuf_);
  if (d->byteBuf_)
    sysdep_free(d->byteBuf_);
}

V90Demodulator::~V90Demodulator() { dtorBody(this); }

/* bodies of the const methods */
static inline int getBitRateBody(const V90Demodulator* d) { return (int)bitRate(d->designed_, d->mp2_->kBits_); }

static inline void getRbsPatternBody(const V90Demodulator* d, unsigned* out)
{
  V90AutoDigitalImpDetector* a = d->adid_;
  for (unsigned i = 0; i <= 5; i++)
    out[i] = a->suspectedPhase_[i];
}

static inline void indicateRemoteRateRenegBody(const V90Demodulator* d)
{
  d->ce_->indicateRemoteRateReneg();
  d->cdes_->state_ = 1;
}

/** Body of getAT_UD(): fill the diagnostics record and print its summary (the RBS pattern is packed as a 6-bit number, phase 0 in bit 0). */
static inline void getAT_UDBody(const V90Demodulator* d, TAG_DiagnosticResults* r)
{
  r->cnt23_ = d->cnt23_;
  r->cnt1f_ = d->cnt1f_;
  r->cnt26_ = d->cnt26_;
  r->bitRate_ = bitRate(d->designed_, d->mp2_->kBits_);
  r->zeroBc_ = 0;
  r->symbolRate_ = 8000;
  V90Equalizer* e = d->eq_;
  copyWord(&r->meanErrWord_, &e->meanErr_);
  r->meanErrDb_ = (float)((LD)10.0f * dsp_log10((LD)e->meanErr_));
  r->powerDb_ = (float)((LD)10.0f * dsp_log10((LD)d->agc_.power_));
  unsigned x = d->info_->rtd_ * 10u;
  r->rtdUnits_ = (unsigned)(((unsigned long long)x * 0xaaaaaaabull) >> 32) >> 6; /* 10 * rtd / 96 */
  unsigned rbs[6];
  V90AutoDigitalImpDetector* a = d->adid_;
  for (unsigned i = 0; i <= 5; i++)
    rbs[i] = a->suspectedPhase_[i];
  r->rbs_ = rbs[0] + (rbs[1] + (rbs[2] + (rbs[3] + (rbs[4] + rbs[5] * 2u) * 2u) * 2u) * 2u) * 2u;
  edprintf(S_DIAG_HDR);
  edprintf(S_DIAG_BAR);
  edprintf(S_DIAG_RBS, r->rbs_, rbs[0], rbs[1], rbs[2], rbs[3], rbs[4], rbs[5]);
}

/* ------------------------------------------------------------------------------------------------ the methods (documented in v90_demodulator.h) */

int V90Demodulator::sessionTermination()
{
  bool notSaved = true;
  if (phase_ == V90D_PHASE_DATA)
    notSaved = pf_.isV90WithEia6();

  /* nothing is stored outside the data phase or on an EIA6 loop */
  if (notSaved)
  { /* the EIA6 query is repeated for the report; phase_ is re-read after it */
    int eia6 = pf_.isV90WithEia6();
    edprintf(S_ST_NOTSAVED, (int)(phase_ == V90D_PHASE_DATA), eia6);
  }
  else if (params_->timingHistoryEvaluationEnabled == 0)
  {
    edprintf(S_ST_DISABLED);
  }
  else
  {
    float mean = storeF(rs_.getTimingHistoryMeanLD());
    float stdv = storeF(rs_.getTimingHistoryStdLD());
    timingStats(params_, mean, stdv, S_ST_MEAN, S_ST_STD, S_ST_STD1000, S_ST_SAVED);
  }

  p3_->clearVerificationStatus();
  return 0;
}

void V90Demodulator::enterPhase4()
{
  if (phase_ == V90D_PHASE_4)
    return;

  if (dsplibs_debug_level > 1)
    dsplibs_debug_printf(S_ENTER_P4);
  phase4Common(this);
}

void V90Demodulator::enterDataSteadyState()
{
  if (phase_ == V90D_PHASE_DATA_STEADY)
    return;
  phase_ = V90D_PHASE_DATA_STEADY;
  edprintf(S_ENTER_STEADY);

  int ev = params_->timingHistoryEvaluationEnabled;
  timingHistoryEval_ = (unsigned)ev;
  if (ev != 0)
  {
    float mean = storeF(rs_.getTimingHistoryMeanLD());
    float stdv = storeF(rs_.getTimingHistoryStdLD());
    timingStats(params_, mean, stdv, S_MEAN, S_STD, S_STD1000, S_SAVED);
  }

  /* steady state: the linear-mapping study stops */
  V90Demapper* dm = dem_;
  unsigned lvl = dsplibs_debug_level;
  dm->linMapStudyOn_ = 0;
  if (lvl > 1)
    dsplibs_debug_printf(S_NOLINMAP_STUDY);
}

void V90Demodulator::enterRRN()
{
  if (phase_ == V90D_PHASE_4)
    return;
  if (dsplibs_debug_level > 1)
    dsplibs_debug_printf(S_RRN_P4);
  symCnt_ = 0;
  timeCnt_ = 0;
  phase_ = V90D_PHASE_4;
  timeLimit_ = info_->rtd_ * 2u + 0x10680u;
  errPrintPeriod_ = (unsigned)params_->errorEnergyPrintPeriodPhase4;
  tmPrintPeriod_ = (unsigned)params_->timingOffsetPrintPeriodPhase4;

  /* silence RRN: suvVal_ tells the transmit side whether both an RRN and an Rt are pending */
  V90ConnectionEvaluator* c = ce_;
  unsigned silence = (unsigned)c->rrnSilence_;
  tagV90AdditionalCPinfo* ci = cpInfo_;
  unsigned flag = 0;
  if (silence != 0)
  {
    V90Phase4Demodulator* p4 = p4_;
    if (p4->rrnPending_ != 0)
      flag = (p4->rtPending_ != 0) ? 1u : 0u;
  }
  ci->suvVal_ = flag;

  V90Demapper* dm = dem_;
  designed_ = 0;
  unsigned lvl = dsplibs_debug_level;
  dm->linMapStudyOn_ = 0;
  if (lvl > 1)
    dsplibs_debug_printf(S_NOLINMAP);
}

void V90Demodulator::enterFPE()
{
  if (phase_ == V90D_PHASE_4)
    return;
  if (dsplibs_debug_level > 1)
    dsplibs_debug_printf(S_FPE_P4);
  symCnt_ = 0;
  timeCnt_ = 0;
  phase_ = V90D_PHASE_4;
  timeLimit_ = info_->rtd_ * 2u + 0x10680u;
  errPrintPeriod_ = (unsigned)params_->errorEnergyPrintPeriodPhase4;
  tmPrintPeriod_ = (unsigned)params_->timingOffsetPrintPeriodPhase4;
}

void V90Demodulator::setSessionFlag(unsigned f)
{
  edprintf(S_SESSIONFLAG, f);
  session_ = f;
  p3_->setSessionFlag(f);
  p4_->setSessionFlag(f);
}

void V90Demodulator::enterPhase3()
{
  if (phase_ == V90D_PHASE_3)
    return;
  if (dsplibs_debug_level > 1)
    dsplibs_debug_printf(S_ENTER_P3);
  phase_ = V90D_PHASE_3;
  unsigned s = symCnt_;
  symCnt_ = 0;
  state_ = V90DEMOD_STATE_NONE;
  timeCnt_ = s;
  flag40_ = 0;

  /* pre-filter and timing loop */
  info_->printInfo();
  pf_.selectFilter();
  if (pf_.isV90WithEia6())
  {
    pf_.setParamEia6();
    pf_.displayParamEia6();
    rs_.setTimingOffset(WORDARG(params_->initialBaudOffset));
  }

  /* restart the phase 3 chain: verifier, phase 3 demodulator, equalizer */
  sv_.reset();
  short eia6 = (short)pf_.isV90WithEia6();
  p3_->reset((PcmType)info_->pcmType_, info_->uinfo_, P3D_WAIT_FOR_SD, 0, jd_, jd92_, dil_, eia6, 1, 0.0f, info_->rtd_);
  p3_->lineMode_ = retrainOk_;
  eq_->enterPhase3();

  V90Parameters* p = params_;
  errPrintPeriod_ = (unsigned)p->errorEnergyPrintPeriodPhase3;
  tmPrintPeriod_ = (unsigned)p->timingOffsetPrintPeriodPhase3;

  /* the connection evaluators start disabled */
  V90ConnectionEvaluator* c = ce_;
  c->phase3EvalActive_ = 0;
  c->avePdsnr_ = 0.0f;
  c->trn1dEvalActive_ = 0;
  c->weight_ = 0;
  designed_ = 0;

  /* V.92Lite: a PCM upstream is not allowed unless the host flag (bit 7 of flags2) says so; fall back to V.34 upstream */
  if ((signed char)p->params->flags2 < 0)
    return;
  if (session_ != 0)
  {
    edprintf(S_V92LITE);
    state_ = V90DEMOD_STATE_V90_FALLBACK;
  }
}

int V90Demodulator::getBitRate() const { return getBitRateBody(this); }

void V90Demodulator::getRbsPattern(unsigned* out) const { getRbsPatternBody(this, out); }

void V90Demodulator::indicateRemoteRateReneg() const { indicateRemoteRateRenegBody(this); }

void V90Demodulator::getAT_UD(TAG_DiagnosticResults* r) const { getAT_UDBody(this, r); }

/**
 * Body of V90Demodulator::exitPhase3(); see the declaration.
 * @warning `dilScale_` is only written by the DIL-study-ended handler of progress(); neither the constructor nor reset() initialises it, so a phase 3 that
 *          ends without a DIL study publishes whatever the object memory held (the caller is expected to hand in zeroed memory).
 */
void V90Demodulator::exitPhase3()
{
  if (phase_ != V90D_PHASE_3)
    return;
  /* publish the TRN1d RMS ratio (the pad gain from the DIL study); loading it as a long double quiets a signalling NaN */
  LD g = loadLD(dilScale_);
  tagV90AdditionalCPinfo* ci = cpInfo_;
  ci->trn1dRms_ = (float)g;
  ci->flag_ = 0;
  ci->mode_ = 0;
  edFix(S_TRN1D_RMS, g, (LD)100000000.0f);

  V90AutoDigitalImpDetector* a = adid_;
  ci = cpInfo_;
  V90Parameters* p = params_;
  int codec = a->codecIsAlaw_;
  V90Phase3Demodulator* p3 = p3_;
  ci->suvVal_ = 0;
  ci->wc_ = (unsigned)codec;
  {
    unsigned short m = (unsigned short)p->analogRateMask; /* 16-bit store into w14_ */
    __builtin_memcpy(&ci->w14_, &m, sizeof m);
  }
  p3->exitDIL();

  /* phase 3 terminated by itself: go to phase 4 now */
  if (p3_->event_ == V90DEMOD_STATE_PHASE3_TERMINATED)
  {
    if (phase_ != V90D_PHASE_4)
    {
      if (dsplibs_debug_level > 1)
        dsplibs_debug_printf(S_ENTER_P4);
      phase4Common(this);
    }
    eq_->enterPhase4();
  }

  /* design the TRN2d constellation from the detected impairments */
  short alt = (short)adid_->isThereAnyAltRbsPhase();
  trn2d_->setNofUcodesInTrn2(alt);
  unsigned session = session_;
  unsigned ssc = sv_.flag_; /* spectral condition, read before the lookahead query */
  unsigned la = (session == 0) ? (unsigned)jd_->getMaxLookahead() : (unsigned)jd92_->getMaxLookahead();
  la &= 0xffu;
  unsigned char* cur = p3_->getMaxUcode();
  a = adid_;
  int codecIsAlaw = a->codecIsAlaw_;
  int pcmType = a->pcmType_;
  short unsus = a->unsuspectedPhase_;
  unsigned char maxTx1 = (unsigned char)(info_->maxTx_ + 1);
  a = adid_;
  short r =
      (short)trn2d_->V90TRN2Design(mp1_, (short (*)[128])a->linearMapping_, (short (*)[128])a->linearMappingAlt_, (unsigned char (*)[128])a->ucodeAllowed_,
                                   (short*)a->altRbsPhase_, (PcmType)pcmType, (PcmType)codecIsAlaw, unsus, cur, la, maxTx1, (V90SpecialSpectralConditions)ssc);

  /* design failed: ask the connection evaluator for a delayed retrain */
  if (r == 0)
  {
    unsigned lvl = dsplibs_debug_level;
    ce_->delayedRetrainRequest_ = 1;
    if (lvl > 1)
      dsplibs_debug_printf(S_DELAYED_RETRAIN);
  }

  edprintf(S_TRN2D_SPECTRAL);
  displaySpectralParams(mp1_);

  /* hand over to phase 4 */
  p4_->reset(info_->uinfo_, P4D_WAIT_RI, 0, retrainOk_);
}

void V90Demodulator::enterDataPhase()
{
  if (phase_ == V90D_PHASE_DATA)
    return;
  symCnt_ = 0;
  V90Parameters* p = params_;
  phase_ = V90D_PHASE_DATA;
  state_ = V90DEMOD_STATE_DATA_PHASE_ENTERED;
  errPrintPeriod_ = (unsigned)p->errorEnergyPrintPeriodData;
  tmPrintPeriod_ = (unsigned)p->timingOffsetPrintPeriodData;
  rs_.setBllState(V90BLL_STEADY_STATE, 1);

  if (dsplibs_debug_level > 1)
    dsplibs_debug_printf(S_ENTER_DATA, (int)bitRate(designed_, mp2_->kBits_));

  /* restart the linear-mapping study (shorter window after a quick connect) */
  V90Demapper* dm = dem_;
  dm->resetLinearMappStudy(retrainOk_ != 0 ? 0x3138u : 0x7530u);
  unsigned lvl = dsplibs_debug_level;
  dm = dem_;
  dm->linMapStudyOn_ = 1;
  if (lvl > 1)
    dsplibs_debug_printf(S_ENABLE_LINMAP);
}

void V90Demodulator::reInit()
{
  ce_->reset();
  p3_->clearVerificationStatus();
}

void V90Demodulator::reset(unsigned arg)
{
  if (dsplibs_debug_level > 1)
    dsplibs_debug_printf(S_RESET_QC, arg);

  phase_ = V90D_PHASE_IDLE;
  quietCnt_ = 0;
  flag270_ = 0;
  symCnt_ = 0;
  state_ = V90DEMOD_STATE_NONE;
  timeCnt_ = 0;
  flag40_ = 0;
  scr_.reset(0);

  /* Agc: restart with the configured block length and nominal energy */
  agc_.reset();
  {
    V90Parameters* p = params_;
    unsigned period = (unsigned)p->agcBlockLen;
    float target;
    copyWord(&target, &p->agcNominalEnergy);
    agc_.period_ = period;
    copyWord(&agc_.target_, &target);
  }

  /* pre-filter, resampler (initial baud offset) and equalizer (cursor position) */
  pf_.reset();
  edFix(S_RESET_BAUD, (LD)params_->initialBaudOffset, (LD)1000.0f);
  rs_.reset();
  rs_.setTimingOffset(WORDARG(params_->initialBaudOffset));
  int cursor = params_->linearEquCursorPlace;
  eq_->reset(cursor < 0 ? (eq_->linLen_ >> 1) : (unsigned)cursor);
  cdes_->reset();

  /* print counters and periods back to their idle values (0x4b00 samples = 2.4 s at 8 kHz) */
  tmPrintPeriod_ = 0x4b00;
  errPrintCnt_ = 0;
  errPrintPeriod_ = 0x4b00;
  tmPrintCnt_ = 0;
  rsCnt_ = 0;
  eqCnt_ = 0;
  framePos_ = 0;
  timingHistoryEval_ = 0;
  designed_ = 0;
  retrainOk_ = arg;
  eq_->flag148_ = arg;
}

void V90Demodulator::enterChannelVerification(short a, short b)
{
  (void)a;
  short bv = b;
  if (dsplibs_debug_level > 1)
    dsplibs_debug_printf(S_ENTER_CHANVER);

  reset(1);
  phase_ = V90D_PHASE_CHANNEL_VERIFICATION;
  symCnt_ = 0;
  p3_->clearVerificationStatus();
  p3_->reset((PcmType)info_->pcmType_, info_->uinfo_, P3D_WAIT_QTS, 0, jd_, jd92_, dil_, 0, bv, 0.0f, info_->rtd_);
  eq_->enterChannelVerification();
  flag40_ = 0;
}
