/**
 * @file pcm_parameters.cpp
 * @brief V90Parameters and V92Parameters: defaults, construction and the dp-runtime overrides (loadParams() of V.90 is in pcm_parameters_load.cpp).
 *
 * Defines V92Parameters (constructor, init(), setToDefault(), loadParams() with its 54 reader calls) and V90Parameters (constructor, init(),
 * initSession(), setToDefault(), loadModemParamsData()).  The member lists (pcm_parameters.h), both loadParams() call sequences and the ~370 constant
 * stores of the two setToDefault() functions are generated between the BEGIN / END GENERATED markers by tools/params_gen.py (--write rewrites the
 * regions; without it the tool only reports).  The hand-written parts are the analog rate mask and the non-constant defaults of V90Parameters::setToDefault()
 * and loadModemParamsData().
 *
 * Flow: construction and init() set every member to its default, let the parameter file override members (loadParams(), only when
 * params->path is set), and (V.90 only) let the dp runtime's modem parameters override a few more (loadModemParamsData()).
 *
 * Numerics: float members hold exact constants; the one extended-precision computation (the digital power reduction of loadModemParamsData()) is
 * `long double`, rounded only where it is stored to the member.
 */

#include "pcm_parameters.h"
#include "core/dsp_math.h"
#include "core/dsp_host.h"
#include "v34/v34_api.h"

extern "C"
{
}

typedef long double LD;

/* x / 2400 and x / 5 by multiplication with a magic constant (high word, shift): exact for every 32-bit x (checked by tools/params_gen.py --check-div) */
static inline unsigned div2400(unsigned x) { return (unsigned)(((unsigned long long)x * 0x1b4e81b5u) >> 32) >> 8; }
static inline unsigned div5(unsigned x) { return (unsigned)(((unsigned long long)x * 0xcccccccdu) >> 32) >> 2; }
#include "core/dsp_intmath.h"

/* ================================================================================================ V92Parameters */
/**
 * @brief Construct with defaults and, when params->path is set, the parameter file overrides.
 * @param p  dp runtime modem parameters (kept; `p->path` is the parameter file)
 */
V92Parameters::V92Parameters(_tagModemParameters* p)
{
  params = p;
  setToDefault();
  char* path = params->path;
  if (path)
    loadParams(path);
}

V92Parameters::~V92Parameters() {}

/** @brief Reset to the defaults, then apply the parameter file overrides when params->path is set. */
void V92Parameters::init()
{
  setToDefault();
  char* path = params->path;
  if (path)
    loadParams(path);
}

/** @brief Set every member to its default (generated constant stores). */
void V92Parameters::setToDefault()
{
  /* BEGIN GENERATED: v92 setToDefault (tools/params_gen.py) */
  vpcmSessionType = 1;
  v92Phase2InfoAOrMu = 0;
  v92Phase2InfoRtd = 0;
  v92Phase2InfoUinfo = 78;
  v92Phase2InfoMaxTxPower = 23;
  v92Phase2InfoTxPowerMeasurePoint = 1;
  v92ExtendEu = 0;
  v92DelayBeforSteadyState = 5000;
  v92RrnStartDelay = 4000;
  v92RrnSimulationSwitch = 0;
  v92SilenceRrnRequeste = 0;
  v92RrnTrn2uDdLength = 12000;
  v92MaxSilenceLengthFlag = 0;
  v92SilenceLength = 2400;
  v92FpeSimulationSwitch = 0;
  v92FpeStartDelay = 20000;
  v92aDigitalRateMask = 0xfffffff;
  v92aMaxSpectralShaperLookahead = 3;
  v92aPhase4Constellation = 0;
  v92aRrnConstellation = 0;
  v92Phase4Constellation = 0;
  v92RrnConstellation = 0;
  v92NofFilterSections = 3;
  v92MaxTotalNofCoeffs = 3;
  v92MaxNofCoeffsInEachSection = 3;
  v92ApplyTxShapingFilter = 1;
  v92EchoFilterLength = 180;
  v92EchoInitialDelay = 840;
  v92EchoDelayOffset = -14;
  v92EchoFastBetaFactor = 1.953125e-10f;
  v92EchoFastDecayFactor = 1.0f;
  v92EchoSlowBetaFactor = 1.8554687e-10f;
  v92EchoSlowDecayFactor = 0.9987f;
  v92EchoFastUpdateDuration = 9000;
  v92EchoSlowUpdateDuration = 11000;
  v92ResamplerResulotion = 1600;
  v92LinearEquLength = 128;
  v92LePhase3Beta = 3e-10f;
  v92LeBetaIDuration = 5000;
  v92LePhase3BetaIi = 1e-10f;
  v92DfeLength = 8;
  v92DfePhase3Beta = 6e-10f;
  v92DfeTrn1uFreezeDuration = 1700;
  errorEnergyPrintPeriodPhase3 = 768;
  errorEnergyPrintPeriodPhase4 = 768;
  v92AgcNominalEnergy = 2.4e+07f;
  v92AgcK = 0.6f;
  v92AgcBlockLen = 150;
  v92AgcAdaptationDuration = 1000;
  suDetectorEnergyThreshold = 3e+02f;
  suDetectorPositiveCorrThreshold = 0.8f;
  suDetectorNegativeCorrThreshold = 0.4f;
  suDetectorDetectionCounterThreshold = 80;
  modulatorQueueLength = 1000;
  /* END GENERATED: v92 setToDefault */
}

/**
 * @brief Read the 54 V.92 parameters from the parameter file through the reader functions.
 * @param parser  parameter file handle / path passed through to the readers
 */
void V92Parameters::loadParams(char* parser)
{
  /* BEGIN GENERATED: v92 loadParams (tools/params_gen.py) */
  Vparser_read_int(parser, "VPCM_SESSION_TYPE", &vpcmSessionType);
  Vparser_read_int(parser, "V92_PHASE2_INFO_A_OR_MU", &v92Phase2InfoAOrMu);
  Vparser_read_int(parser, "V92_PHASE2_INFO_RTD", &v92Phase2InfoRtd);
  Vparser_read_int(parser, "V92_PHASE2_INFO_UINFO", &v92Phase2InfoUinfo);
  Vparser_read_int(parser, "V92_PHASE2_INFO_MAX_TX_POWER", &v92Phase2InfoMaxTxPower);
  Vparser_read_int(parser, "V92_PHASE2_INFO_TX_POWER_MEASURE_POINT", &v92Phase2InfoTxPowerMeasurePoint);
  Vparser_read_int(parser, "V92_EXTEND_EU", &v92ExtendEu);
  Vparser_read_int(parser, "V92_DELAY_BEFOR_STEADY_STATE", &v92DelayBeforSteadyState);
  Vparser_read_int(parser, "V92_RRN_SIMULATION_SWITCH", &v92RrnSimulationSwitch);
  Vparser_read_int(parser, "V92_RRN_START_DELAY", &v92RrnStartDelay);
  Vparser_read_int(parser, "V92_SILENCE_RRN_REQUESTE", &v92SilenceRrnRequeste);
  Vparser_read_int(parser, "V92_RRN_TRN2U_DD_LENGTH", &v92RrnTrn2uDdLength);
  Vparser_read_int(parser, "V92_MAX_SILENCE_LENGTH_FLAG", &v92MaxSilenceLengthFlag);
  Vparser_read_int(parser, "V92_SILENCE_LENGTH", &v92SilenceLength);
  Vparser_read_int(parser, "V92_FPE_SIMULATION_SWITCH", &v92FpeSimulationSwitch);
  Vparser_read_int(parser, "V92_FPE_START_DELAY", &v92FpeStartDelay);
  Vparser_read_int(parser, "V92A_DIGITAL_RATE_MASK", &v92aDigitalRateMask);
  Vparser_read_int(parser, "V92A_MAX_SPECTRAL_SHAPER_LOOKAHEAD", &v92aMaxSpectralShaperLookahead);
  Vparser_read_int(parser, "V92A_PHASE4_CONSTELLATION", &v92aPhase4Constellation);
  Vparser_read_int(parser, "V92A_RRN_CONSTELLATION", &v92aRrnConstellation);
  Vparser_read_int(parser, "V92_PHASE4_CONSTELLATION", &v92Phase4Constellation);
  Vparser_read_int(parser, "V92_RRN_CONSTELLATION", &v92RrnConstellation);
  Vparser_read_int(parser, "V92_NOF_FILTER_SECTIONS", &v92NofFilterSections);
  Vparser_read_int(parser, "V92_MAX_TOTAL_NOF_COEFFS", &v92MaxTotalNofCoeffs);
  Vparser_read_int(parser, "V92_MAX_NOF_COEFFS_IN_EACH_SECTION", &v92MaxNofCoeffsInEachSection);
  Vparser_read_int(parser, "V92_APPLY_TX_SHAPING_FILTER", &v92ApplyTxShapingFilter);
  Vparser_read_int(parser, "V92_ECHO_FILTER_LENGTH", &v92EchoFilterLength);
  Vparser_read_int(parser, "V92_ECHO_INITIAL_DELAY", &v92EchoInitialDelay);
  Vparser_read_int(parser, "V92_ECHO_DELAY_OFFSET", &v92EchoDelayOffset);
  Vparser_read_float(parser, "V92_ECHO_FAST_BETA_FACTOR", &v92EchoFastBetaFactor);
  Vparser_read_float(parser, "V92_ECHO_FAST_DECAY_FACTOR", &v92EchoFastDecayFactor);
  Vparser_read_float(parser, "V92_ECHO_SLOW_BETA_FACTOR", &v92EchoSlowBetaFactor);
  Vparser_read_float(parser, "V92_ECHO_SLOW_DECAY_FACTOR", &v92EchoSlowDecayFactor);
  Vparser_read_int(parser, "V92_ECHO_FAST_UPDATE_DURATION", &v92EchoFastUpdateDuration);
  Vparser_read_int(parser, "V92_ECHO_SLOW_UPDATE_DURATION", &v92EchoSlowUpdateDuration);
  Vparser_read_int(parser, "V92_RESAMPLER_RESULOTION", &v92ResamplerResulotion);
  Vparser_read_int(parser, "V92_LINEAR_EQU_LENGTH", &v92LinearEquLength);
  Vparser_read_float(parser, "V92_LE_PHASE_3_BETA", &v92LePhase3Beta);
  Vparser_read_int(parser, "V92_LE_BETA_I_DURATION", &v92LeBetaIDuration);
  Vparser_read_float(parser, "V92_LE_PHASE_3_BETA_II", &v92LePhase3BetaIi);
  Vparser_read_int(parser, "V92_DFE_LENGTH", &v92DfeLength);
  Vparser_read_float(parser, "V92_DFE_PHASE_3_BETA", &v92DfePhase3Beta);
  Vparser_read_int(parser, "V92_DFE_TRN1U_FREEZE_DURATION", &v92DfeTrn1uFreezeDuration);
  Vparser_read_int(parser, "ERROR_ENERGY_PRINT_PERIOD_PHASE3", &errorEnergyPrintPeriodPhase3);
  Vparser_read_int(parser, "ERROR_ENERGY_PRINT_PERIOD_PHASE4", &errorEnergyPrintPeriodPhase4);
  Vparser_read_float(parser, "V92_AGC_NOMINAL_ENERGY", &v92AgcNominalEnergy);
  Vparser_read_float(parser, "V92_AGC_K", &v92AgcK);
  Vparser_read_int(parser, "V92_AGC_BLOCK_LEN", &v92AgcBlockLen);
  Vparser_read_int(parser, "V92_AGC_ADAPTATION_DURATION", &v92AgcAdaptationDuration);
  Vparser_read_float(parser, "SU_DETECTOR_ENERGY_THRESHOLD", &suDetectorEnergyThreshold);
  Vparser_read_float(parser, "SU_DETECTOR_POSITIVE_CORR_THRESHOLD", &suDetectorPositiveCorrThreshold);
  Vparser_read_float(parser, "SU_DETECTOR_NEGATIVE_CORR_THRESHOLD", &suDetectorNegativeCorrThreshold);
  Vparser_read_int(parser, "SU_DETECTOR_DETECTION_COUNTER_THRESHOLD", &suDetectorDetectionCounterThreshold);
  Vparser_read_int(parser, "MODULATOR_QUEUE_LENGTH", &modulatorQueueLength);
  /* END GENERATED: v92 loadParams */
}

/* ================================================================================================ V90Parameters */
/**
 * @brief Construct with defaults, the parameter file overrides and the dp runtime overrides.
 * @param p  dp runtime modem parameters (kept)
 * @note  sensitiveIspDetected and maxTxRateIndexForSensitiveIsp are set before setToDefault(), which reads them.
 */
V90Parameters::V90Parameters(_tagModemParameters* p)
{
  sensitiveIspDetected = 0;
  params = p;
  maxTxRateIndexForSensitiveIsp = 14;
  setToDefault();
  char* path = params->path;
  if (path)
    loadParams(path);
  loadModemParamsData();
}

V90Parameters::~V90Parameters() {}

/**
 * @brief Reset to the defaults, apply the parameter file when params->path is set, then apply the dp runtime overrides.
 * @note  sensitiveIspDetected and maxTxRateIndexForSensitiveIsp are not reset here (see initSession()).
 */
void V90Parameters::init()
{
  setToDefault();
  char* path = params->path;
  if (path)
    loadParams(path);
  loadModemParamsData();
}

/** @brief Start a new session: clear the sensitive-ISP flag and restore the highest allowed transmit rate index (14). */
void V90Parameters::initSession()
{
  sensitiveIspDetected = 0;
  maxTxRateIndexForSensitiveIsp = 14;
}

/**
 * @brief Set every member to its default.
 * @note  Three parts: the constants up to v92JdPhase, the analog (upstream) rate mask from params->upstreamMinRate / upstreamMaxRate, and the remaining
 *        constants with two values that depend on the dp runtime flags.  silenceScr is set only when sensitiveIspDetected is 0; otherwise it keeps its
 *        previous value.
 */
void V90Parameters::setToDefault()
{
  const _tagModemParameters* P = params;

  /* ---- constants that precede the rate computation ---- */
  /* BEGIN GENERATED: v90 setToDefault head (tools/params_gen.py) */
  probingMode = 0;
  hwCodecType = -1;
  lineConnectionType = -1;
  enableEqualizerMmx = 1;
  eia6EnableEqualizerMmx = 0;
  phase2InfoAOrMu = 0;
  phase2InfoRtd = 0;
  phase2InfoUinfo = 78;
  phase2InfoMaxTxPower = 23;
  phase2InfoTxPowerMeasurePoint = 1;
  digitalRateMask = 0xfffffff;
  maxSpectralShaperLookahead = 3;
  v34Phase4Constellation = 0;
  v34RrnConstellation = 0;
  v92DigitalRateMask = 0xfffffff;
  v92MaxSpectralShaperLookahead = 3;
  v92JdPhase = 0.85f;
  /* END GENERATED: v90 setToDefault head */
  analogRateMask = 0;

  /* ---- the analog (upstream) rate mask: bit k - 2 = 2400 * k bit/s is allowed, k = 2 .. 14; the range is clamped to 2 .. 14 and a sensitive
   * ISP lowers the maximum ---- */
  unsigned lo = div2400(P->upstreamMinRate);
  unsigned hi = div2400(P->upstreamMaxRate);
  if (sensitiveIspDetected != 0 && hi > (unsigned)maxTxRateIndexForSensitiveIsp)
    hi = (unsigned)maxTxRateIndexForSensitiveIsp;

  lo = (lo >= 2) ? (lo > 14 ? 14 : lo) : 2;
  hi = (hi >= 2) ? (hi > 14 ? 14 : hi) : 2;
  if (lo > hi)
  {
    if (dsplibs_debug_level > 1)
      dsplibs_debug_printf("V90Parameters: bad upstream rate, using default values\n");
    lo = 2;
    hi = 14;
  }

  unsigned mask = (unsigned)analogRateMask;
  for (unsigned k = 14; k > 1; k--)
  {
    mask = mask * 2u;
    if (k <= hi && k >= lo)
      mask |= 1u;
    analogRateMask = (int)mask;
  }

  if (dsplibs_debug_level > 1)
    dsplibs_debug_printf("V90Parameters: upStream min rate : %d upStream max rate : %d  Rate mask :%x\n", (int)lo * 2400, (int)hi * 2400, mask);

  /* ---- the constants ---- */
  /* BEGIN GENERATED: v90 setToDefault body (tools/params_gen.py) */
  preFilterGain = -1;
  preFilterCoefType = -1;
  germanIsdnNt1BoxFilterGain = 12;
  germanPbxPreFilterGain = 12;
  agcNominalEnergy = 1.4e+07f;
  agcK = 0.6f;
  agcBlockLen = 150;
  agcAdaptationDuration = 1000;
  unk6c = 1.0f;
  unk70 = 0.6f;
  maxUcode = 92;
  nofUcodes = 8;
  eia6NofUcodes = 4;
  nofUcodesInTrn2 = 4;
  initialBaudOffset = 0.0f;
  bllInitialK1 = 0.002f;
  bllInitialK2 = 0.0f;
  bllFastK1 = 0.00113f;
  bllFastK2 = 3e-07f;
  bllMediumK1 = 0.0002f;
  bllMediumK2 = 2.5e-08f;
  bllSlowK1 = 0.00015f;
  bllSlowK2 = 1.875e-08f;
  bllSlow2K1 = 6e-05f;
  bllSlow2K2 = 4e-09f;
  bllDilK1 = 3e-06f;
  bllDilK2 = 0.0f;
  bllTrn2InitialK1 = 3e-05f;
  bllTrn2InitialK2 = 7e-12f;
  bllTrn2K1 = 2e-05f;
  bllTrn2K2 = 5e-12f;
  bllSteadyStateK1 = 3e-06f;
  bllSteadyStateK2 = 1e-13f;
  bllPreAnspcmK1 = 0.0005f;
  bllPreAnspcmK2 = 0.0f;
  bllTrn1QcInitialK1 = 0.0002f;
  bllTrn1QcInitialK2 = 0.0f;
  bllTrn1QcFastK1 = 0.0005f;
  bllTrn1QcFastK2 = 7e-12f;
  bllTrn1QcMediumK1 = 0.0003f;
  bllTrn1QcMediumK2 = 5e-12f;
  bllTrn1QcSlowK1 = 0.0001f;
  bllTrn1QcSlowK2 = 2e-12f;
  bllTrn1dInitialToFastDuration = 2000;
  bllTrn1dFastToSlowDuration = 7200;
  bllSlowToSlow2Duration = 6000;
  bllTrn1QcInitialToFastDuration = 1000;
  bllTrn1QcFastToMediumDuration = 4000;
  bllTrn1QcMediumToSlowDuration = 4000;
  eia6BllInitialK1 = 5e-05f;
  eia6BllInitialK2 = 0.0f;
  eia6BllFastK1 = 3e-05f;
  eia6BllFastK2 = 7e-12f;
  eia6BllMediumK1 = 1.5e-05f;
  eia6BllMediumK2 = 3e-12f;
  eia6BllSlowK1 = 5e-06f;
  eia6BllSlowK2 = 2e-12f;
  eia6BllSlow2K1 = 3e-06f;
  eia6BllSlow2K2 = 1e-12f;
  eia6BllDilK1 = 8e-07f;
  eia6BllDilK2 = 0.0f;
  eia6BllTrn2InitialK1 = 3e-06f;
  eia6BllTrn2InitialK2 = 1e-12f;
  eia6BllTrn2K1 = 1e-06f;
  eia6BllTrn2K2 = 3e-13f;
  eia6BllSteadyStateK1 = 8e-07f;
  eia6BllSteadyStateK2 = 5e-14f;
  eia6BllTrn1dInitialToFastDuration = 1000;
  eia6BllTrn1dFastToSlowDuration = 8000;
  timingHistoryEvaluationEnabled = 1;
  timingHistoryEvaluationBufferLength = 100;
  timingHistoryEvaluationPeriod = 1500;
  timingOffesetMinStdForSave = 0.1f;
  linearEquLength = 300;
  linearEquHistoryLength = 500;
  linearEquFadeEdgesCycle = 20;
  linearEquFadeLeftEdgeRatio = 0.15f;
  linearEquFadeRightEdgeRatio = 0.15f;
  linearEquCursorPlace = -1;
  linearEquTrn1dBeta = 1e-10f;
  linearEquDilBeta = 5e-11f;
  linearEquDilMedUcodeBeta = 3.3e-11f;
  linearEquDilHighUcodeBeta = 1.5e-11f;
  eia6LinearEquDilMedUcodeBeta = 5e-12f;
  eia6LinearEquDilHighUcodeBeta = 2e-12f;
  eia6LinearEquDilErrorRelaxBeta = 8e-11f;
  linearEquAltDilBeta = 1.25e-11f;
  linearEquAltDilMedUcodeBeta = 8.25e-12f;
  linearEquAltDilHighUcodeBeta = 3.75e-12f;
  germanPbxLinearEquDilBeta = 8.5e-11f;
  germanPbxLinearEquDilMedUcodeBeta = 6e-11f;
  germanPbxLinearEquDilHighUcodeBeta = 1.5e-11f;
  linearEquDilErrorRelaxBeta = 7.5e-11f;
  linearEquTrn2dInitialBeta = 1e-10f;
  linearEquTrn2dBeta = 7e-11f;
  linearEquDataBeta = 2e-11f;
  linearEquTrn1dFreezeDuration = 1000;
  linearEquTrn2dInitialDuration = 3000;
  eia6LinearEquTrn1dBeta = 8e-11f;
  eia6LinearEquDilBeta = 1.5e-11f;
  eia6LinearEquTrn2dBeta = 3.5e-11f;
  eia6LinearEquDataBeta = 3e-11f;
  eia6LinearEquTrn2dInitialBeta = 8.5e-11f;
  eia6LinearEquFadeEdgesCycle = 30;
  eia6LinearEquFadeLeftEdgeRatio = 0.12f;
  eia6LinearEquFadeRightEdgeRatio = 0.12f;
  germanPbxLinearEquDataBeta = 7e-11f;
  germanIsdnNt1LinearEquDataBeta = 6e-11f;
  dfeLength = 12;
  dfeTrn1dBeta = 7e-09f;
  dfeDilBeta = 7e-08f;
  dfeTrn2dBeta = 1e-08f;
  dfeDataBeta = 5e-09f;
  dfeDilMedUcodeBeta = 6e-08f;
  dfeDilHighUcodeBeta = 5e-08f;
  eia6DfeDilMedUcodeBeta = 1e-08f;
  eia6DfeDilHighUcodeBeta = 4.52e-09f;
  eia6DfeDilErrorRelaxBeta = 2e-08f;
  dfeDilAltBeta = 1.5e-09f;
  dfeDilAltMedUcodeBeta = 1e-09f;
  dfeDilAltHighUcodeBeta = 4.52e-10f;
  dfeDilErrorRelaxBeta = 7e-08f;
  dfeTrn1dFreezeDuration = 2000;
  germanPbxDfeTrn2dFastBeta = 7e-08f;
  germanPbxDfeTrn2dSlowBeta = 4e-08f;
  germanPbxDfeDataBeta = 1.5e-08f;
  eia6DfeDilBeta = 1.5e-08f;
  eia6DfeTrn1dBeta = 1.5e-08f;
  eia6DfeDataBeta = 8e-09f;
  eia6DfeTrn2dFastBeta = 3e-08f;
  eia6DfeTrn2dSlowBeta = 1.5e-08f;
  eia6DfeTrn2dRrnBeta = 5e-08f;
  errorEnergyMeanBlockLen = 160;
  errorEnergyMeanK = 0.7f;
  errorEnergyPrintPeriodPhase3 = 768;
  errorEnergyPrintPeriodPhase4 = 768;
  errorEnergyPrintPeriodData = 1920;
  nofDdSymbolsBeforeMeanErrorDiagPhase3 = 5000;
  nofDdSymbolsBeforeMeanErrorDiagPhase4 = 2000;
  timingOffsetPrintPeriodPhase3 = 9600;
  timingOffsetPrintPeriodPhase4 = 9600;
  timingOffsetPrintPeriodData = 19200;
  sdDetectorEnergyThreshold = 2.5e+05f;
  sdDetectorPositiveCorrThreshold = 0.8f;
  sdDetectorNegativeCorrThreshold = 0.4f;
  sdDetectorDetectionCounterThreshold = 150;
  phase4RDetectionLength = 180;
  rrnRDetectionLength = 240;
  energyDropDetectorThreshold = 2e+04f;
  noEnergyDurationForRemoteRetrain = 384;
  spectralVerifierEnable = 1;
  eia6SpectralVerifierEnable = 0;
  spectralVerifierSampleFreq = 9.6e+03f;
  spectralVerifierFftLen = 1024;
  spectralVerifierFftWindow = 1;
  spectralVerifierPsdLen = 4096;
  spectralVerifierPsdOverlapLen = 512;
  spectralVerifierPrintSpectrum = 0;
  spectralVerifierIsdnNullFreq = 4.22e+03f;
  spectralVerifierIsdnLeftPeakFreq = 4.12e+03f;
  spectralVerifierIsdnRightPeakFreq = 4.35e+03f;
  spectralVerifierIsdnLeftPeakDelta = 11.0f;
  spectralVerifierIsdnRightPeakDelta = 1.0f;
  spectralVerifierGermanPbxNullFreq = 4e+03f;
  spectralVerifierGermanPbxLeftPeakFreq = 3.95e+03f;
  spectralVerifierGermanPbxRightPeakFreq = 4.1e+03f;
  spectralVerifierGermanPbxLeftPeakDelta = 1e+01f;
  spectralVerifierGermanPbxRightPeakDelta = 1e+01f;
  spectralVerifierSevereCodecRefFreq = 3.6e+03f;
  spectralVerifierSevereCodecTestFreq1 = 4e+03f;
  spectralVerifierSevereCodecTestFreq2 = 4.2e+03f;
  spectralVerifierSevereCodecDelta = 28.0f;
  trn1dDdLength = 12000;
  unk300 = 21841;
  unk304 = 1200;
  unk308 = 24720;
  unk30c = 3840;
  unk310 = 600;
  unk314 = 17041;
  unk318 = 3120;
  unk31c = 240;
  unk320 = 720;
  unk324 = 40000;
  unk328 = 0.96f;
  unk32c = 1920;
  unk330 = 6120;
  unk334 = 3901;
  unk338 = 5161;
  unk33c = 4201;
  unk340 = 4921;
  unk344 = 12000;
  adidStudyLen0 = 600;
  adidStudyLen1 = 1200;
  adidStudyLen2 = 600;
  adidStudyLen3 = 1800;
  adidStudyLen4 = 1200;
  adidStudyLen5 = 3000;
  unk360 = 2;
  minimumRtdForNonSilenceScr = 1000;
  trn2dDdLength = 24000;
  rrnTrn2dDdLength = 12000;
  useRestricedDmin = 0;
  enableRedundancyOptimization = 1;
  enableDigitalPowerReduction = 1;
  digitalPowerReduction = -1.0f;
  upRoundK = 0.745f;
  eia6UseRestricedDmin = 1;
  dminCalcFactor1 = 1.25f;
  dminCalcFactor2 = 12;
  dminEia6Factor = 1.08f;
  forcedDmin = -1;
  unk39c = 0;
  forceRateEnable = 0;
  rateForce = 45333;
  spectralShaperA1 = 1.0f;
  spectralShaperA2 = 0.0f;
  spectralShaperB1 = 0.0f;
  spectralShaperB2 = 0.0f;
  spectralShaperSr = 1;
  spectralShaperId = 3;
  germanPbxSpectralShaperA1 = 1.0f;
  germanPbxSpectralShaperA2 = -1.0f;
  germanPbxSpectralShaperB1 = 0.0f;
  germanPbxSpectralShaperB2 = 0.0f;
  germanPbxSpectralShaperSr = 3;
  germanPbxSpectralShaperId = 1;
  eia6SpectralShaperA1 = 1.0f;
  eia6SpectralShaperA2 = -1.0f;
  eia6SpectralShaperB1 = 0.0f;
  eia6SpectralShaperB2 = 0.0f;
  eia6SpectralShaperSr = 2;
  eia6SpectralShaperId = 1;
  rrnSilenceRequested = 1;
  maskRrnSilenceOnProblematicIsp = 0;
  rrnSilenceScrLength = 2500;
  rrnSilenceWaitBeforeEchoCalc = 132;
  rrnSilenceEchoCalcPeriod = 240;
  rrnSilenceMinEchoEnergyForKeepRate = 2.0f;
  minRateForSilenceRrnKeepRate = 50666;
  pdsnrThresholdInPhase3 = 1.2e+02f;
  pdsnrThresholdInPhase4 = 1e+02f;
  trn1dErrorForV34Fallback = 1.8e+02f;
  trn1dMeanErrorStdEvaluationEnable = 0;
  trn1dMaxMeanErrorStdInPhase3 = 17.0f;
  trn2dMeanErrorStdEvaluationEnable = 0;
  trn2dMaxMeanErrorStdInPhase4 = 1.414f;
  trn2dMaxMeanErrorEnergyInPhase4 = 35.0f;
  phase3ErrorForV34Fallback = 2e+02f;
  phase4ErrorForV34Fallback = 5e+03f;
  pdsnrV34DropThreshPhase4Retrain = 2.5e+02f;
  phase4MeanErrorBefToAftUpdateRatioThresh = 1.5f;
  qcPhase4MeanErrorBefToAftUpdateRatioThresh = 1.75f;
  phase4MeanErrorBefToAftUpdateRatioThreshAltRbs = 1e+01f;
  enableRrnUp = 1;
  enableRrnDown = 1;
  rateUpDetectDuration = 2400;
  rateDownDetectDuration = 2400;
  retrainDetectDuration = 2400;
  nofRemoteRateRenegBeforeRetrain = 10;
  maxNofRetrainsForRatioCheck = 2;
  maxNofV90Retrains = 2;
  maxNofRemoteRetrains = 3;
  retrainCounterFadeCount = 960000;
  remoteRrnCounterFadeCount = 720000;
  minimumDurationInDataBeforeRrnUp = 2400000;
  minimumDurationInDataBeforeRrnDown = 160000;
  minimumDurationInDataBeforeEcRrn = 240000;
  maxNofRatesDiffBeforeRetrain = 4;
  enableErrorCorrectionRrn = 1;
  eia6PdsnrThresholdInPhase4 = 1.5e+02f;
  eia6PdsnrThresholdInPhase3 = 1.6e+02f;
  eia6Trn1dErrorForV34Fallback = 1.8e+02f;
  eia6MaxNofV90Retrains = 1;
  enableDrop2V34OnSevereCodec = 1;
  debugDigitalModemInitiateRrn = 0;
  debugDigitalModemInitiateRrnTime = 5000;
  trn1QcDdLength = 3600;
  unk4a4 = 2400;
  adidStudyLenAlt0 = 300;
  adidStudyLenAlt1 = 600;
  adidStudyLenAlt2 = 600;
  adidStudyLenAlt3 = 600;
  adidStudyLenAlt4 = 300;
  adidStudyLenAlt5 = 600;
  trn2dQcDdLength = 8400;
  linearEquQcTrn1dFreezeDuration = 1000;
  dfeQcTrn1dFreezeDuration = 1500;
  qcLoggingPeriodInitial = 48000;
  qcLoggingPeriodSteadyState = 576000;
  anspcmCorrelationThreshForValidation = 0.975f;
  debugConnectionEvaluatorAlternateDebug = 0;
  debugConnectionEvaluatorFallBack = 0;
  debugConnectionEvaluatorRetrain = 0;
  debugConnectionEvaluatorRateUp = 0;
  debugConnectionEvaluatorRateDown = 0;
  debugConnectionEvaluatorPeriod = 200;
  highLevelTxActive = 0;
  loopType = -1;
  separatePhase3Constellations = 0;
  separatePhase4Constellations = 1;
  separateDataPhaseConstellations = 1;
  writeTimingPhaseAndOffsetToFile = 0;
  writeErrorToFile = 0;
  writeDemodInSamplesToFile = 0;
  writeEquCoefsToFile = 0;
  loadEquCoefsFromFile = 0;
  debugPrintMapperConstellations = 0;
  debugPrintDemapperConstellations = 0;
  debugDemapperErrorHistogram = 0;
  demapperDelayBeforeErrorHistogram = 8000;
  demapperErrorHistogramIntegrationTime = 240000;
  tempIntParameter1 = 0;
  tempIntParameter2 = 0;
  tempIntParameter3 = 0;
  tempIntParameter4 = 0;
  tempFloatParameter1 = 0.0f;
  tempFloatParameter2 = 0.0f;
  tempFloatParameter3 = 0.0f;
  tempFloatParameter4 = 0.0f;
  /* END GENERATED: v90 setToDefault body */

  /* ---- the words that are not plain constants ---- */
  anspcmDemodulationLength = (P->flags0 & 1u) ? 0x960 : 0x1e3c;
  if (sensitiveIspDetected == 0)
    silenceScr = 1; /* the only conditional store: otherwise the member keeps its previous value */
}

/**
 * @brief Let the dp runtime's modem parameters override some defaults.
 * @note  Applies powerReduction (digitalPowerReduction = powerReduction / 5 * 0.5, printed as sign / whole / hundredths), the probe flag (probingMode),
 *        the connection type (lineConnectionType, only while it is still -1) and the trn2d mean-error evaluation flag.  Each step prints a debug line
 *        through edprintf().
 */
void V90Parameters::loadModemParamsData()
{
  unsigned pr = params->powerReduction;
  edprintf("V90Parameters: Debug - tempPR = %d\r\n", pr);
  if (pr != 0)
  {
    /* (pr / 5) * 0.5 at extended precision, rounded only when stored to the member */
    LD v = (LD)(int)div5(pr) * (LD)0.5f;
    digitalPowerReduction = (float)v;
    int whole = dsp_trunc_i32(__builtin_fabsl(v));             /* integer part of |v| */
    LD frac = v - (LD)dsp_trunc_i32(v);                        /* fractional part, sign kept */
    int hundredths = absI32(dsp_trunc_i32(frac * (LD)100.0f)); /* hundredths, absolute value */
    int sign = (digitalPowerReduction <= 0.0f) ? 0x2d : 0x2b;  /* '-' for values <= 0 (so 0 prints as -0.00); a NaN gives '+' */
    edprintf("V90Parameters: setting power reduction to  = %c%d.%02d\r\n", sign, whole, hundredths);
  }

  unsigned probe = (unsigned)(params->flags50 >> 1) & 1u;
  edprintf("V90Parameters: Debug - tempProbe = %d\r\n", probe);
  if (probe != 0)
    probingMode = (int)probe;

  unsigned connection = params->connectionType;
  edprintf("V90Parameters: Debug - tempConnectionType = %d\r\n", connection);
  if (lineConnectionType == -1)
    lineConnectionType = (int)connection;

  trn2dMeanErrorStdEvaluationEnable = (int)(params->flags50 & 1u);
  edprintf("V90Parameters: Debug - trn2d_mean_error_std_evaluation_enable = %d\r\n", trn2dMeanErrorStdEvaluationEnable);
}

/*
 * Compile-time tie between the C view V90ParObj (src/v34/v34_v90params_view.h; the V.34 files read sensitiveIspDetected, highLevelTxActive and
 * maxTxRateIndexForSensitiveIsp through it after casting the void* ctx->v90Params_) and the real class V90Parameters, checked on whichever word size
 * this file is built: every member of the view sits where the class has it.  The only pointer is the first member, so at -m64 every later word moves
 * by the same 4 bytes; the second assert pins each class member to its i386 offset RELATIVE to probingMode (the first word after the pointer), which
 * is the i386 image offset the view documents.  Declarations only (no code, no data).
 */
extern "C"
{
#include "v34/v34_v90params_view.h"
}
#define V90PAR_TIE(vm, cm, off)                                                                                                                                \
  DSP_STATIC_ASSERT(offsetof(V90ParObj, vm) == offsetof(V90Parameters, cm), "V90ParObj." #vm " != V90Parameters::" #cm);                                       \
  DSP_STATIC_ASSERT(offsetof(V90Parameters, cm) - offsetof(V90Parameters, probingMode) == (off) - 4u, "V90Parameters::" #cm " is not at its i386 offset");
#include "v90params_view_ties.inc"
#undef V90PAR_TIE
DSP_STATIC_ASSERT(offsetof(V90ParObj, params) == 0 && offsetof(V90Parameters, params) == 0, "V90Parameters::params is not the first member");
DSP_STATIC_ASSERT(sizeof(V90ParObj) == sizeof(V90Parameters), "sizeof(V90ParObj) != sizeof(V90Parameters)");
#if defined(__i386__)
DSP_STATIC_ASSERT(sizeof(V90Parameters) == 0x558, "sizeof(V90Parameters) != 0x558 at i386");
#endif
