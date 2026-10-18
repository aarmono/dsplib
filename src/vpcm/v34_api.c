/**
 * @file v34_api.c
 * @brief Small V.34 module entry points used by the V.90 / V.92 / K56flex engines: setters, indicators, getters, mode-change wrappers.
 *
 * Plain C functions that work on the V.34 object (tagV34Object, accessed as `V34Obj`) and, for the
 * V.90 / V.92 rate and session queries, on the owning engine context (`PcmFlo47Obj`, v34_flomodem_view.h) and the prefixes of the engine objects
 * (v34_engine_views.h).  The functions fall into five groups:
 * - Setters and indicators: resetRetrainDetector(), SetUpstreamModulationInfo(), VPcmV34SetMinimumSigLevel(), VPcmV34SetMaxBlockLength(),
 *   VPcmV34SetTimeOut(), VPcmV34IndicateLocalRRN() / IndicateRemoteRRN(), VPcmV34SetIndicationOfRemoteRetrain(), VPcmV34Delete().
 * - Accessors: getTimingOffset() / getTimingPhase(), V34XF_GetInfo0BitsPtr(), V34XF_GetProbeResultsPtr(), V34XF_GetRTD(), VPcmV34GetCleanedSamples(),
 *   VPcmV34ReportStartOfEchoAdapt() / ReportMiddleOfEchoAdapt() (diagnostic prints only).
 * - State and mode getters: VPcmV34GetCurrentSessionDP(), VPcmV34GetQuickConnectIndication(), VPcmV34GetCurrentRx/TxBaudRate(), GetCurrentRx/TxCarrier(),
 *   VPcmV34GetSNR(), VPcmV34GetCurrentTxBitRate(), VPcmV34GetMaxUpstreamRateIndex() / V34XF_GetMaxUpstreamRateIndex().
 * - Host notifications: VPcmV34NotifyDP() and VPcmV34RequestDPNotification().
 * - Mode-change wrappers that call v34handshakinit() / preinitdigital() / v34setuptxmit(): VPcmV34InitiateRateRenegotiation(), VPcmV34InitiateHangUp(),
 *   VPcmV34SetV90RateReneg(), V34XF_IndicateJdReceived() / IndicateDilReceived() / IndicateTrn2dReceived(), V34XF_IndicateK56FlexJdReceived() /
 *   IndicateK56FlexRateDetermined(), and the modem-on-hold message decoder VPcmV34InterpretMohMessageBits() together with interpretMohTimeout().
 *
 * Conventions:
 * - Small integer arguments are `short` / `unsigned char` exactly as wide as the callers pass them; the upper argument bits are never read.
 * - Debug prints are guarded by a fresh read of the unsigned debug level (`LEVEL_ON()`); a few functions read the level before a store, and the order of
 *   store, level read and print is observable by a print hook.  VPcmV34InterpretMohMessageBits() re-reads the level after every print of its
 *   illegal-message chain, and its trailing "====" print is guarded by the last of those reads.
 * - The bit-rate arithmetic of VPcmV34GetCurrentTxBitRate() is x87: an unsigned 32-bit product converted to extended precision, multiplied by a float32
 *   constant (1/6 or 1/12), 0.5f added, truncated toward zero, the low 32 bits returned.
 * - The engine pointer chains (context -> V.90 / V.92 modulator -> bits-to-symbol -> mapper / transmitter, context -> demodulator -> connection
 *   evaluator, context -> V.90 parameters) are real pointer members of the typed views, not byte offsets.
 * - The pointers returned by VPcmV34GetCleanedSamples(), V34XF_GetInfo0BitsPtr() and V34XF_GetProbeResultsPtr() are addresses inside the object.
 *
 * Callees: v34handshakinit(), preinitdigital(), v34setuptxmit(), edprintf(), dsplibs_debug_printf(); V34DisconnectThreshTable is defined in
 * src/v34/v34_api_tables.c.
 */

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "v34/v34_api_tables.h"
#include "v34/v34_hs_states.h"    /* V34HS_* */
#include "v34/v34_hs_init_mode.h" /* V34HSINIT_* (the mode argument of v34handshakinit) */
#include "v34/v34_role.h"         /* V34ROLE_* (the values of modeChar_) */
#include "v34/v34_run_mode.h"     /* V34RUN_* (the values of runMode_) */

#include "core/dsp_debug_level.h" /* dsplibs_debug_level, LEVEL_ON() */
#include "core/host_interface.h"  /* enum DP_ID */
#include "core/dsp_host.h"
#include "v34/v34_api.h"
#include "vpcm_v34_api.h"

#include "v34/v34_access.h" /* AS_U16 */

#include "core/dp_params.h"         /* the parameter block (V34ModParObj) */
#include "v34/v34_v90params_view.h" /* V90ParObj */
#include "v34/v34_flomodem_view.h"  /* PcmFlo47Obj: the engine context */
#include "v34/v34_engine_views.h"   /* V90Modem / V92Modem / V90Demod prefixes: the engine objects the context points to */
#include "v34/v34_object_view.h"    /* V34Obj */

/* ============================================================================================================
 * tiny setters / indicators / accessors
 * ============================================================================================================ */

/**
 * @brief Clear a 0x20-byte retrain-request detector record and preset its thresholds for the given role.
 * @param r     detector record (V34Retrain: 16-bit words at 0 .. 6, 32-bit words at 8, 0x14, 0x18, 0x1c, thresholds at 0xc, 0xe, 0x10)
 * @param mode  V34ROLE_CALL (thresholds 0, 0, 0x39c3) or V34ROLE_ANSWER (0x5a82, 0x55fc, 0x39c3); any other value leaves the thresholds unchanged
 */
void resetRetrainDetector(V34Retrain* r, short mode)
{
  r->rd0_ = 0;
  r->rd2_ = 0;
  r->rd4_ = 0;
  r->rd6_ = 0;
  r->rd8_ = 0;
  r->rd14_ = 0;
  r->rd18_ = 0;
  r->rd1c_ = 0;

  if (mode == V34ROLE_CALL)
  {
    r->rdc_ = 0;
    r->rde_ = 0;
    r->rd10_ = 0x39c3;
  }
  else if (mode == V34ROLE_ANSWER)
  {
    r->rdc_ = 0x5a82;
    r->rde_ = 0x55fc;
    r->rd10_ = 0x39c3;
  }
}

/**
 * @brief Modem-on-hold time-out code to a duration.
 * @param code  code received in an MHack message (only the low 16 bits count, compared unsigned)
 * @return 10, 20, 30, 40, 60, 120, 180, 240, 360, 480, 720, 960 for codes 1 .. 12 (seconds), -1 for code 13 (no limit), 0 for any other code
 */
int interpretMohTimeout(short code)
{
  switch ((uint16_t)code)
  {
  case 1:
    return 10;
  case 2:
    return 0x14;
  case 3:
    return 0x1e;
  case 4:
    return 0x28;
  case 5:
    return 0x3c;
  case 6:
    return 0x78;
  case 7:
    return 0xb4;
  case 8:
    return 0xf0;
  case 9:
    return 0x168;
  case 10:
    return 0x1e0;
  case 11:
    return 0x2d0;
  case 12:
    return 0x3c0;
  case 13:
    return -1;
  default:
    return 0;
  }
}

/** @brief Placeholder for the upstream modulation information hook; does nothing. */
void SetUpstreamModulationInfo(void* obj) { (void)obj; }

/**
 * @brief Select the minimum signal level from the disconnect-threshold table.
 * @param obj  V.34 object; its parameter block supplies `minLevel`
 * @note  The table index is `minLevel + 0x30` taken as unsigned; an index above 7 selects entry 3.  The debug level is read before the table load and the
 *        `minSigLevel_` store; the print re-reads `minLevel`; `w234_` is cleared after the print.
 */
void VPcmV34SetMinimumSigLevel(V34Obj* obj)
{
  V34ModParObj* parobj = obj->params_;
  uint32_t idx = (uint32_t)parobj->minLevel + 0x30u;
  int lv_on;
  int thr;

  if (idx > 7u)
    idx = 3;

  lv_on = LEVEL_ON();
  thr = V34DisconnectThreshTable[idx];
  obj->minSigLevel_ = thr;
  if (lv_on)
    dsplibs_debug_printf("VPcmV34Main: minLevel given is %d , minSigLevel set to %d\n", parobj->minLevel, thr);
  obj->w234_ = 0;
}

/** @brief Release hook of the V.34 module; nothing to release.  @return 0 */
int VPcmV34Delete(void) { return 0; }

/** @brief Store the maximum block length (`w8_`); the debug level is read before the store. */
void VPcmV34SetMaxBlockLength(V34Obj* obj, int len)
{
  int lv_on = LEVEL_ON();
  (obj)->w8_ = len;
  if (lv_on)
    dsplibs_debug_printf("VPcmV34Main: Max Block Length modified to %d\r\n", len);
}

/** @brief Arm the time-out: clear the sample clock `sampleTime_` and set `deadline_` = t * 0x2580 (9600 samples per unit; 32-bit product). */
void VPcmV34SetTimeOut(V34Obj* obj, int t)
{
  obj->sampleTime_ = 0;
  obj->deadline_ = (int32_t)((uint32_t)t * 0x2580u);
}

/* Rate-renegotiation indicators: count a local / remote request and flag a remote retrain. */
void VPcmV34IndicateLocalRRN(void* obj) { ((V34Obj*)obj)->dgC0e_ = (uint16_t)(AS_U16(((V34Obj*)obj)->dgC0e_) + 1); }
void VPcmV34IndicateRemoteRRN(void* obj) { ((V34Obj*)obj)->dgC10_ = (uint16_t)(AS_U16(((V34Obj*)obj)->dgC10_) + 1); }
void VPcmV34SetIndicationOfRemoteRetrain(void* obj) { ((V34Obj*)obj)->b_ac17_ = 1; }

/* Receive timing offset and phase (words of the object). */
int getTimingOffset(V34Obj* obj) { return (int32_t)obj->rx.w49c_; }
int getTimingPhase(V34Obj* obj) { return obj->rx.w04a0_; }

/* Addresses of the received INFO0 bits and of the probe results inside the object; round-trip delay estimate (bulk delay + 0x1e0, as a signed 16-bit value). */
void* V34XF_GetInfo0BitsPtr(void* obj) { return ((V34Obj*)obj)->unka8a4_; }
void* V34XF_GetProbeResultsPtr(void* obj) { return ((V34Obj*)obj)->probeEnergy_; }
int V34XF_GetRTD(void* obj) { return (int)(int16_t)(uint16_t)(AS_U16(((V34Obj*)obj)->bulkDelay_) + 0x1e0); }

/**
 * @brief Hand out the cleaned-sample buffer.
 * @param obj  V.34 object
 * @param out  receives the pending sample count (`w2aa6_`, sign-extended); the count is then cleared
 * @return address of the buffer inside the object (1200 bytes after `constPts_`)
 * @warning `out` is written without a NULL check.
 */
void* VPcmV34GetCleanedSamples(V34Obj* obj, int* out)
{
  *out = (int)obj->tx.w2aa6_;
  obj->tx.w2aa6_ = 0;
  return obj->tx.constPts_ + 600;
}

/* Diagnostic prints marking the start / middle of echo adaptation (debug level above 1). */
void VPcmV34ReportStartOfEchoAdapt(void)
{
  if (LEVEL_ON())
    dsplibs_debug_printf("VPcmV34Main: Echo adapt start reported...\r\n");
}
void VPcmV34ReportMiddleOfEchoAdapt(void)
{
  if (LEVEL_ON())
    dsplibs_debug_printf("VPcmV34Main: Echo adapt middle reported...\r\n");
}

/* ============================================================================================================
 * state / mode getters
 * ============================================================================================================ */

/**
 * @brief Session type the V.34 object is running.
 * @return DP_V92 (mode 2, or mode 1 with `wabc6_` and INFO1d pending), DP_V90 (other mode 1), DP_K56 (mode 3), otherwise DP_V34; modes are compared signed
 */
int VPcmV34GetCurrentSessionDP(V34Obj* obj)
{
  int mode = obj->mode_;

  if (mode == 2)
    return DP_V92;
  if (mode == 1)
  {
    if (obj->wabc6_ != 0 && obj->info1dPending_ != 0)
      return DP_V92;
    return DP_V90;
  }
  if (mode == 3)
    return DP_K56;
  return DP_V34;
}

/**
 * @brief Quick-connect indication for the current mode.
 * @return the short phase-2 value for modes 0 .. 2, 5 .. 7; 1 for modes 4, 8, 9; 0 for modes 3, 10 and for any mode above 10 (mode compared unsigned)
 */
int VPcmV34GetQuickConnectIndication(V34Obj* obj)
{
  uint32_t mode = (uint32_t)obj->mode_;
  uint32_t m;

  if (mode > 0xa)
    return 0;
  m = 1u << mode;
  if (m & 0xe7u)
    return (int)(int16_t)obj->shortPhase2_;
  if (m & 0x408u)
    return 0;
  if (m & 0x310u)
    return 1;
  return 0;
}

/* Current receive / transmit baud rate and carrier.  The role (`modeChar_` == V34ROLE_ANSWER) decides which of the two mode numbers is the receive and which
 * the transmit direction; while that direction is in the 8000 Hz PCM phase (mode - base is 0 or 1, unsigned) the baud rate is 8000 and the carrier 0. */
int VPcmV34GetCurrentRxBaudRate(V34Obj* obj)
{
  uint32_t u = (obj->modeChar_ == V34ROLE_ANSWER) ? (uint32_t)obj->mode_ - 1u : (uint32_t)obj->mode_ - 2u;
  return (u <= 1u) ? 8000 : (int)obj->rate.rxBaud_;
}
int VPcmV34GetCurrentTxBaudRate(V34Obj* obj)
{
  uint32_t u = (obj->modeChar_ == V34ROLE_ANSWER) ? (uint32_t)obj->mode_ - 2u : (uint32_t)obj->mode_ - 1u;
  return (u <= 1u) ? 8000 : (int)(int16_t)obj->rate.baud_;
}
int VPcmV34GetCurrentRxCarrier(V34Obj* obj)
{
  uint32_t u = (obj->modeChar_ == V34ROLE_ANSWER) ? (uint32_t)obj->mode_ - 1u : (uint32_t)obj->mode_ - 2u;
  return (u <= 1u) ? 0 : (int)obj->rate.rxCarrier_;
}
int VPcmV34GetCurrentTxCarrier(V34Obj* obj)
{
  if (obj->modeChar_ == V34ROLE_ANSWER)
  {
    if ((uint32_t)obj->mode_ - 2u <= 1u)
      return 0;
  }
  else if (obj->mode_ == 2)
  {
    return 0;
  }
  return (int)(int16_t)obj->rate.carrier_;
}

/**
 * @brief Estimate the receive signal-to-noise ratio in dB.
 * @param obj  V.34 object
 * @return 0 when the equalizer error `equErr_` is not positive, else about 10 * log10(`w4ac_` / `equErr_`)
 * @note  The quotient is divided by about 4 (0x1013 / 2^14) per 6 dB step until it reaches 0, then the last non-zero quotient is scaled by about 0.794
 *        (0x32d6 / 2^14, 1 dB) per step; the products are 32-bit and wrap.
 */
int VPcmV34GetSNR(V34Obj* obj)
{
  int lastQ = 0, snrDb = 0;
  short d = obj->rx.equErr_;

  if (d > 0)
  {
    int q = obj->rx.w4ac_ / (int)d; /* `cltd ; idiv`: d > 0, cannot trap */
    if (q > 0)
    {
      for (;;)
      {
        int p = (int32_t)((uint32_t)q * 0x1013u) >> 14;
        lastQ = q;
        q = p;
        if (q > 0)
          snrDb += 6;
        else
          break;
      }
    }
  }

  if (lastQ > 0)
  {
    int x = lastQ;
    for (;;)
    {
      x = (int32_t)((uint32_t)x * 0x32d6u) >> 14;
      if (x > 0)
        snrDb++;
      else
        break;
    }
  }
  return snrDb;
}

/* Bit pattern to float. */
static float fbits(uint32_t u)
{
  float f;
  memcpy(&f, &u, 4);
  return f;
}
/* trunc(v * c + 0.5f) in extended precision, low 32 bits of the 64-bit integer result. */
static int tx_rate_x87(uint32_t v, float c)
{
  long double x = (long double)v;
  x = x * (long double)c;                     /* fildll ; fmuls */
  x = x + (long double)fbits(0x3f000000u);    /* flds 0.5 ; faddp */
  return (int)(uint32_t)(uint64_t)(int64_t)x; /* fistpll with RC = truncate; low 32 bits */
}
/**
 * @brief Current transmit bit rate in bits per second.
 * @param obj  V.34 object
 * @return for the answering role: in mode 2 the V.92 rate (0 until the V.92 modulator state is 3, then mapper rate * 8000 / 12), 30000 in mode 3; for the
 *         calling role in modes 1 and 2: 0 until the V.90 modulator state is 3, then the mapper rate * 8000 / 6; otherwise `txRateIdx_` * 2400
 * @warning The engine pointers (`v92Mod_`, `v90Mod_`, their `b2s_` and mapper / transmitter members) are dereferenced without NULL checks; callers use it
 *          only while a V.90 / V.92 session is active.
 */
int VPcmV34GetCurrentTxBitRate(V34Obj* obj)
{
  PcmFlo47Obj* ctxobj = (PcmFlo47Obj*)obj->ctx_;

  if (obj->modeChar_ == V34ROLE_ANSWER)
  {
    int mode = obj->mode_;

    if (mode == 2)
    {
      void* e = ctxobj->v92Mod_;
      if (((V92ModulatorPfx*)e)->state_ != 3)
        return 0;
      {
        void* q = ((V92ModulatorPfx*)e)->b2s_;
        void* r = ((V92BitsToSymbolPfx*)q)->tx_;
        uint32_t v = ((V92TransmitterPfx*)r)->w0_ * 8000u;
        return tx_rate_x87(v, fbits(0x3daaaaabu));
      }
    }
    if (mode == 3)
      return 30000;
  }
  else if ((uint32_t)obj->mode_ - 1u <= 1u)
  {
    void* e = ctxobj->v90Mod_;
    if (((V90ModulatorPfx*)e)->state_ != 3)
      return 0;
    {
      void* q = ((V90ModulatorPfx*)e)->b2s_;
      void* r = ((V90BitsToSymbolPfx*)q)->mapper_;
      uint32_t v = ((V90MapperPfx*)r)->w0_ * 8000u;
      return tx_rate_x87(v, fbits(0x3e2aaaabu));
    }
  }
  return (int)obj->rate.txRateIdx_ * 0x960;
}

/* Maximum upstream rate (parameter block word at 0x3c), lowered to maxTxRateIndexForSensitiveIsp * 2400 on a sensitive ISP of a V.90 / V.92 session;
 * shared by the two entry points below. */
static int max_upstream(V34Obj* obj)
{
  PcmFlo47Obj* ctxobj = (PcmFlo47Obj*)obj->ctx_;
  V34ModParObj* P = obj->params_;
  int rate = (int32_t)P->upstreamMaxRate;

  if (ctxobj->flag6120_ != 0 && (obj)->pcmMode_ > 1)
  {
    void* cfg = ctxobj->v90Params_;
    V90ParObj* cfgobj = (V90ParObj*)cfg;
    if (cfgobj->sensitiveIspDetected != 0)
    {
      int lim = (int32_t)((uint32_t)cfgobj->maxTxRateIndexForSensitiveIsp * 0x960u);
      if (rate > lim)
        rate = lim; /* signed minimum */
      edprintf("on get max upstream rate, on sensitive ISP, returning %d\r\n", rate);
      return rate;
    }
  }
  edprintf("on get max upstream rate, on regular ISP, returning %d\r\n", rate);
  return rate;
}
/** @brief Maximum upstream rate (see max_upstream()); prints which ISP class applied. */
int VPcmV34GetMaxUpstreamRateIndex(void* obj) { return max_upstream(obj); }
/** @brief Same as VPcmV34GetMaxUpstreamRateIndex() (unused duplicate entry point). */
int V34XF_GetMaxUpstreamRateIndex(void* obj) { return max_upstream(obj); }

/**
 * @brief Host notification about the line state.
 * @param obj   V.34 object
 * @param code  1 = samples valid (`w262_` = 1), 0 = samples invalid (`w262_` = 0), 2 = CAS detected (mode 5, sample clock cleared), 3 = validate 3-way
 *              call (mode 6, `waa74_` = MOH timer + 0xbb80); other codes are ignored
 */
void VPcmV34NotifyDP(V34Obj* obj, int code)
{
  if (code == 1)
  {
    edprintf("VPcmV34 Notification: Valid in samples...\r\n");
    obj->w262_ = 1;
  }
  else if (code == 0)
  {
    edprintf("VPcmV34 Notification: Invalid in samples...\r\n");
    obj->w262_ = 0;
  }
  else if (code == 2)
  {
    edprintf("VPcmV34 Notification: CAS Detected...\r\n");
    obj->mode_ = 5;
    obj->sampleTime_ = 0;
  }
  else if (code == 3)
  {
    int moh, cnt;
    edprintf("VPcmV34 Notification: Validate 3-Way Call...\r\n");
    obj->mode_ = 6;
    moh = obj->wabdc_;
    cnt = (int32_t)((uint32_t)moh + 0xbb80u);
    obj->waa74_ = cnt;
    edprintf("VPcmV34 debug validate: mohTimer = %d, setting count2 to %d\r\n", moh, cnt);
  }
}

/**
 * @brief Fetch and clear the pending DP notification.
 * @param obj  V.34 object
 * @param a    receives the notification code (`wac40_`)
 * @param b    receives its first argument (`wac44_`)
 * @param c    receives its second argument (`wac48_`)
 * @return 0 if no notification is pending (`wac40_` negative), else 1 after the three words are cleared (code to -1) and bit 0 of parameter byte 0x51
 *         is cleared
 * @warning The three out pointers are written in the order a, b, c before the object is updated, and are not checked for NULL.
 */
int VPcmV34RequestDPNotification(V34Obj* obj, int* a, int* b, int* c)
{
  int d = obj->wac40_;
  V34ModParObj* P;

  if (d < 0)
    return 0;

  *a = d;
  *b = obj->wac44_;
  *c = obj->wac48_;
  obj->wac48_ = 0;
  obj->wac40_ = -1;
  obj->wac44_ = 0;
  P = obj->params_;
  P->flags51 = (uint8_t)(P->flags51 & 0xfe);
  return 1;
}

/* ============================================================================================================
 * mode-change wrappers (call the already-ported v34handshakinit / preinitdigital / v34setuptxmit)
 * ============================================================================================================ */

/**
 * @brief Request a rate renegotiation (unused).
 * @param obj   V.34 object
 * @param code  in a V.90 / V.92 session (modes 1 and 2, unsigned test) it becomes the external command of the connection evaluator; otherwise 2, 0, 5 step
 *              the rate index down, 3 steps it up (within `minRateIdx_` .. `maxRateIdx_`) and any other code requests the lowest rate (-1)
 * @note  Outside a V.90 / V.92 session it restarts the handshake in local rate-renegotiation mode (state 6) and counts the request.
 * @warning In a V.90 / V.92 session the demodulator and its connection evaluator are dereferenced without NULL checks.
 */
void VPcmV34InitiateRateRenegotiation(V34Obj* obj, int code)
{
  if ((uint32_t)obj->mode_ - 1u < 2u)
  { /* modes 1 and 2 (unsigned test) */
    void* a = ((PcmFlo47Obj*)obj->ctx_)->v90Dem_;
    void* b = ((V90DemodulatorPfx*)a)->ce_;
    ((V90ConnectionEvaluatorPfx*)b)->externalCommand_ = code;
    return;
  }

  if (code == 2 || code == 0 || code == 5)
  {
    int v = obj->w228_ - 1;
    if (!(v < obj->minRateIdx_))
      obj->w22c_ = v;
  }
  else if (code == 3)
  {
    int v = obj->w228_ + 1;
    if (!(v > obj->maxRateIdx_))
      obj->w22c_ = v;
  }
  else
  {
    obj->w22c_ = -1;
  }

  v34handshakinit(obj, V34HSINIT_RNEG_LOCAL);
  obj->state_ = 6;
  obj->rx.retrainCnt_ = 0;
  obj->rx.rrnDownCnt_ = 0;
  obj->rx.rrnUpCnt_ = 0;
  obj->mod.runMode_ = V34RUN_RRN_LOCAL;
  obj->dgC0e_ = (uint16_t)(AS_U16(obj->dgC0e_) + 1);
}

/**
 * @brief Request a hang-up (unused): clear the rate limits and either command the V.90 / V.92 connection evaluator to clear down (command 2) or restart the
 *        V.34 handshake in local rate-renegotiation mode (state 6).
 * @param obj  V.34 object
 * @note  The debug level is read before the context pointer.
 * @warning In a V.90 / V.92 session (modes 1 and 2) the demodulator and its connection evaluator are dereferenced without NULL checks.
 */
void VPcmV34InitiateHangUp(V34Obj* obj)
{
  int lv_on = LEVEL_ON();
  PcmFlo47Obj* ctxobj = (PcmFlo47Obj*)obj->ctx_;
  int mode;

  if (lv_on)
    dsplibs_debug_printf("VPcmV34Main: VPcmV34InitiateHangUp called !\r\n");

  obj->w22c_ = 0;
  mode = obj->mode_;
  obj->minRateIdx_ = 0;
  obj->maxRateIdx_ = 0;
  if ((uint32_t)mode - 1u < 2u)
  {
    void* a;
    void* b;
    ctxobj->clearDown_ = 1;
    a = ctxobj->v90Dem_;
    b = ((V90DemodulatorPfx*)a)->ce_;
    ((V90ConnectionEvaluatorPfx*)b)->externalCommand_ = 2;
    return;
  }

  v34handshakinit(obj, V34HSINIT_RNEG_LOCAL);
  obj->state_ = 6;
  obj->mod.runMode_ = V34RUN_RRN_LOCAL;
  obj->rx.retrainCnt_ = 0;
  obj->rx.rrnDownCnt_ = 0;
  obj->rx.rrnUpCnt_ = 0;
}

/**
 * @brief Start a V.90 local rate renegotiation.
 * @param obj  V.34 object
 * @param rrn  renegotiation type: 0 selects PCM mode 0xb, anything else 0xf
 * @param sz   constellation size flag: nonzero selects the receive signal word 0x89b0, zero 0x8990
 * @note  Resets the transmit state, calls preinitdigital(), enters state 6 and schedules the data start (`dataStartTime_` = -0xea600, `deadline_` = 0x69780).
 */
void VPcmV34SetV90RateReneg(void* obj, short rrn, unsigned char sz)
{
  V34Obj* v34obj = (V34Obj*)obj;
  int lv_on = LEVEL_ON();
  uint16_t m;

  if (lv_on)
    dsplibs_debug_printf("setV90RateReneg called, rrn type = %d, constel size = %d\r\n", (int)rrn, (int)sz);
  v34obj->pcmMode_ = (rrn == 0) ? 0xb : 0xf;
  m = (uint16_t)((v34obj->tx.txFlags_ & 0xbfe7u) | 0x2000u);

  v34obj->tx.txPrev_ = 0;
  v34obj->tx.txCnt_ = 0;
  v34obj->tx.scrState_ = 0;
  v34obj->tx.txFlags_ = m;
  preinitdigital(obj);

  v34obj->tx.k56Idx_ = 0;
  v34obj->state_ = 6;
  v34obj->mod.runMode_ = V34RUN_RRN_LOCAL;
  v34obj->rx.rxSig_ = (sz != 0) ? 0x89b0 : 0x8990;
  v34obj->sampleTime_ = 0;
  v34obj->dataStartTime_ = 0xfff15a00u;
  v34obj->deadline_ = 0x69780;
}

/**
 * @brief Record that the Jd sequence was received: PCM mode 3, receive signal word from the constellation size and the silence-scrambler flag.
 * @param obj  V.34 object
 * @param a    constellation size flag (nonzero: 0x89b0, zero: 0x8990)
 * @param b    silence scrambler flag; nonzero sets the receive signal word to 0
 */
void V34XF_IndicateJdReceived(void* obj, unsigned char a, unsigned char b)
{
  V34Obj* v34obj = (V34Obj*)obj;
  if (LEVEL_ON())
    dsplibs_debug_printf("V34Main: IndicateJdReceived - constel size = %d, silence SCR = %d\r\n", (int)a, (int)b);
  v34obj->pcmMode_ = 3;
  if (b != 0)
    v34obj->rx.rxSig_ = 0;
  else
    v34obj->rx.rxSig_ = (a != 0) ? 0x89b0 : 0x8990;
}

/**
 * @brief Record that the DIL was received: PCM mode 6, transmit count cleared, receive signal word from the constellation size flag `a`.
 * @param obj  V.34 object
 * @param a    constellation size flag (nonzero: 0x89b0, zero: 0x8990)
 */
void V34XF_IndicateDilReceived(void* obj, unsigned char a)
{
  V34Obj* v34obj = (V34Obj*)obj;
  if (LEVEL_ON())
    dsplibs_debug_printf("V34Main: IndicateDilReceived - constel size = %d\r\n", (int)a);
  v34obj->pcmMode_ = 6;
  v34obj->tx.txCnt_ = 0;
  v34obj->rx.rxSig_ = (a != 0) ? 0x89b0 : 0x8990;
}

/**
 * @brief Record that TRN2d was received: advance the PCM mode.
 * @param obj  V.34 object
 * @note  The mode (signed) maps: below 10 to 10; 10 .. 14 to 14; 15 .. 17 to 18; above 17 to 20.  The debug level is read before the store; the printed
 *        value is re-read from the object.
 */
void V34XF_IndicateTrn2dReceived(void* obj)
{
  V34Obj* v34obj = (V34Obj*)obj;
  int v = v34obj->pcmMode_;
  int nv;
  int lv_on;

  if (v < 10)
    nv = 10;
  else if (v < 0xf)
    nv = 0xe;
  else
    nv = 0x12 + ((v > 0x11) ? 2 : 0);
  lv_on = LEVEL_ON();
  v34obj->pcmMode_ = nv;
  if (lv_on)
    dsplibs_debug_printf("IndicateTrn2dReceived called, v90Receiver = %d\r\n", v34obj->pcmMode_);
}

/**
 * @brief Record that the K56flex Jd was received (unused): set up the V.34 transmitter and enter the SSEG transmit state (unused).
 * @param obj  V.34 object
 * @param a    constellation size; 0x10 selects the receive signal word 0x89b0, anything else 0x8990
 */
void V34XF_IndicateK56FlexJdReceived(V34Obj* obj, unsigned char a)
{
  int16_t t;

  obj->rx.rxFlags_ = (uint16_t)(obj->rx.rxFlags_ | 8);
  obj->rx.rxSig_ = (a == 0x10) ? 0x89b0 : 0x8990;
  if (LEVEL_ON())
    dsplibs_debug_printf("VPcmV34Main: About to setup v34 txmit, baudrate = %d, carrier = %d, preemp = %d\r\n", (int)(int16_t)obj->rate.baud_,
                         (int)(int16_t)obj->rate.carrier_, (int)(int16_t)obj->rate.preemp_);

  v34setuptxmit(obj);

  if ((int16_t)obj->txState_ != V34HS_SSEG)
    obj->txState_ = V34HS_SSEG;
  obj->tx.txCnt_ = 0;
  obj->tx.txPrev_ = 0;
  obj->tx.scrState_ = 0;
  t = obj->txMdMs_;
  obj->w258_ = 0;
  obj->w256_ = 0;
  obj->w254_ = (uint16_t)((int)t * 0x150 + 0x2710);
}

/** @brief Record that the K56flex rate was determined (unused): K56 mode 5. */
void V34XF_IndicateK56FlexRateDetermined(V34Obj* obj)
{
  int lv_on = LEVEL_ON();
  if (lv_on)
    dsplibs_debug_printf("K56Flex rate determined, starting MP transmission...\r\n");
  (obj)->k56Mode_ = 5;
}

/* ============================================================================================================
 * modem-on-hold message decoder
 * ============================================================================================================ */

/**
 * @brief Decode a received modem-on-hold (MH) message and update the MOH state.
 * @param obj  V.34 object
 * @param msg  message word, read once (low byte significant)
 * @note  0x33 MHreq (`mohF4_` 0); 0x5X MHack with time-out code X (`mohE0_`, `mohF4_` 4); 0x77 MHnack, may init MOH later (5); 0x75 MHnack, will not
 *        (`mohE2_` 3, `mohF4_` 5); 0x9X MHcld with reason X: 5 incoming call, 6 outgoing call, 10 other, anything else reserved and taken as other
 *        (`mohFA_` 0 / 1 / 2, `mohF4_` 2); 0xbb MHcda (3); 0xdd MHfrr (1).  Any other word is reported at debug level above 1 and forced to MH NACK.
 * @warning `msg` is dereferenced without a NULL check.
 */
void VPcmV34InterpretMohMessageBits(V34Obj* obj, const uint16_t* msg)
{
  unsigned int w = *msg; /* movzwl, read once */
  unsigned int hi = w & 0xf0u;

  if (w == 0x33)
  {
    if (LEVEL_ON())
      dsplibs_debug_printf("MHreq message detected !\r\n");
    obj->mohF4_ = 0;
    return;
  }
  if (hi == 0x50)
  {
    unsigned int code = w & 0xfu;
    int lv_on = LEVEL_ON();
    obj->mohE0_ = (uint16_t)code;
    if (lv_on)
      dsplibs_debug_printf("MHack message detected ! , Time out period code = %d\r\n", (int)code);
    obj->mohF4_ = 4;
    return;
  }
  if (w == 0x77)
  {
    if (LEVEL_ON())
      dsplibs_debug_printf("MHnack message detected (modem may init MOH in the future) !\r\n");
    obj->mohF4_ = 5;
    return;
  }
  if (w == 0x75)
  {
    if (LEVEL_ON())
      dsplibs_debug_printf("MHnack message detected (modem may NOT init MOH in the future) !\r\n");
    obj->mohE2_ = 3;
    obj->mohF4_ = 5;
    return;
  }
  if (hi == 0x90)
  {
    unsigned int code = w & 0xfu;
    if (code == 5)
    {
      obj->mohFA_ = 0;
      if (LEVEL_ON())
        dsplibs_debug_printf("MHcld message detected ! , disconnect reason is incoming call\r\n");
    }
    else if (code == 6)
    {
      obj->mohFA_ = 1;
      if (LEVEL_ON())
        dsplibs_debug_printf("MHcld message detected ! , disconnect reason is outgoing call\r\n");
    }
    else if (code == 10)
    {
      obj->mohFA_ = 2;
      if (LEVEL_ON())
        dsplibs_debug_printf("MHcld message detected ! , disconnect reason is other reason\r\n");
    }
    else
    {
      obj->mohFA_ = 2;
      if (LEVEL_ON())
        dsplibs_debug_printf("MHcld message detected ! , reserved disconnect reason (assume other reason)\r\n");
    }
    obj->mohF4_ = 2;
    return;
  }
  if (w == 0xbb)
  {
    if (LEVEL_ON())
      dsplibs_debug_printf("MHcda message detected !\r\n");
    obj->mohF4_ = 3;
    return;
  }
  if (w == 0xdd)
  {
    if (LEVEL_ON())
      dsplibs_debug_printf("MHfrr message detected !\r\n");
    obj->mohF4_ = 1;
    return;
  }
  /* illegal message: a chain of prints, each guarded by the level read right after the previous print; the trailing "====" print is guarded by the last
   * of those reads */
  {
    unsigned int lv = dsplibs_debug_level;
    if (lv > 1u)
    {
      dsplibs_debug_printf("=========================================================================\r\n");
      lv = dsplibs_debug_level;
      if (lv > 1u)
      {
        dsplibs_debug_printf("Illegal MH message detected !!! (rxsq[0] = %X)\r\n", (unsigned int)*msg);
        lv = dsplibs_debug_level;
        if (lv > 1u)
        {
          dsplibs_debug_printf("forcing message type to MH NACK\r\n");
          lv = dsplibs_debug_level;
        }
      }
    }
    obj->mohF4_ = 5;
    if (lv > 1u)
      dsplibs_debug_printf("=========================================================================\r\n");
  }
}
