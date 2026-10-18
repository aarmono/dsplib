/**
 * @file vpcm_session_wrappers.cpp
 * @brief Small wrappers between the V.34 module object and the V.90 / V.92 / K56Flex engines.
 *
 * Defines the eight functions declared in vpcm_session_wrappers.h.  They are C++ because they call methods of VPcmFloModem, K56FlexFloModem and the
 * V.90 / V.92 classes through tagV34Object::ctx_ / k56_.
 * - V34GiveINFO1dBits(): decides from the received INFO1d whether the upstream is PCM (V.92Lite) and may start a retrain to V.34.
 * - indicateJaTransmission(): lets the active engine enter phase 3.
 * - VPcmV34GetCurrentRxBitRate(), VPcmV34SetMinMaxBitRates(), VPcmV34SetDelays(), VPcmV34InitMOH(): rate and delay queries / set-up.
 * - VPcmV34GetVisualDiagnostics(), VPcmV34GetDiagnostics(): diagnostics for the host (no caller in the library).
 *
 * Conventions: the debug level is tested with the unsigned `level > 1` form and re-read where it matters; ctx_ and the K56 handle are read once at
 * entry; the parameter block pointer is re-read after every print; the diagnostics' float values come from small exact integers, the echo-canceller
 * taps are scaled by 10000 in extended precision and truncated (dsp_trunc_i32: NaN or out of range gives 0x80000000), the V.92 transmit rate is
 * trunc(0.5 + u / 12) in extended precision through a 64-bit conversion, and the shifts / multiplications of the error-ratio estimate wrap in 32 bits.
 */

#include "vpcm_session_wrappers.h"
#include "core/dsp_math.h"
#include "core/dsp_access.h"
#include "core/dsp_asm.h"
#include "core/host_interface.h" /* DP_V34 / DP_K56 / DP_V90 / DP_V92 */
#include "core/dsp_host.h"

typedef long double LD;

#include "core/dsp_x87_hide.h"
#include "vpcm_v34_api.h"

/**
 * @brief Reset the retrain request detector (the same sequence as in VPcmV34InitiateRetrain(), v34_api.c).
 *
 * Clears the detector's counters and, depending on the role (modeChar_, read as a signed 16-bit value), preloads the thresholds for the calling or
 * the answering side.
 * @param obj V.34 module object
 */
static inline void resetRetrain(tagV34Object* obj)
{
  V34Retrain* r = &obj->retrain;
  int16_t m = (int16_t)obj->modeChar_;
  r->rd0_ = 0;
  r->rd4_ = 0;
  r->rd2_ = 0;
  r->rd6_ = 0;
  r->rd8_ = 0;
  r->rd14_ = 0;
  r->rd18_ = 0;
  r->rd1c_ = 0;
  if (m == V34ROLE_CALL)
  {
    r->rdc_ = 0;
    r->rde_ = 0;
    r->rd10_ = 0x39c3;
  }
  else if (m == V34ROLE_ANSWER)
  {
    r->rdc_ = 0x5a82;
    r->rde_ = 0x55fc;
    r->rd10_ = 0x39c3;
  }
}

extern "C"
{
}

/**
 * @brief Decide from the received INFO1d bits whether the session continues with PCM upstream.
 *
 * Clears v92Session_ and returns 0 when the V.34 object is not in a PCM mode.  Otherwise v92Session_ becomes 1 when the local V.92 capability is set,
 * info1dPending_ is non-zero and the remote requested PCM upstream (bit 5 of info[7]).  Under V.92Lite (flags2 bit 7 clear) a PCM upstream
 * starts a retrain to V.34 upstream.
 * @param obj  V.34 module object
 * @param info received INFO1d words, at least 10
 * @return 1 when a retrain to V.34 upstream was started, else 0
 */
int V34GiveINFO1dBits(tagV34Object* obj, uint16_t* info)
{
  int ret = 0;
  int pm = obj->pcmMode_;
  VPcmFloModem* ctx = obj->ctx_;
  uint32_t v;
  ctx->v92Session_ = 0;
  if (pm == 0)
    return 0;

  /* trace the received words */
  if (dsplibs_debug_level > 1u)
  {
    dsplibs_debug_printf("VPcmV34Main: giveINFO1dBits\n");
    if (dsplibs_debug_level > 1u)
      dsplibs_debug_printf("VPcmV34Main: rxinfo1d = 0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x\n", (unsigned)info[0], (unsigned)info[1],
                           (unsigned)info[2], (unsigned)info[3], (unsigned)info[4], (unsigned)info[5], (unsigned)info[6], (unsigned)info[7], (unsigned)info[8],
                           (unsigned)info[9]);
  }
  /* PCM upstream needs the local capability, the remote capability and the request in INFO1d */
  v = 0;
  if (ctx->v92_.info_->v92CapLocal_ != 0 && obj->info1dPending_ != 0 && (info[7] & 0x20u) != 0)
    v = 1;
  ctx->v92Session_ = v;
  if (dsplibs_debug_level > 1u)
    dsplibs_debug_printf("VPcmV34Main: upstream selection: local cap - %d, remote cap - %d, requested in info1 - %d, isPCM - %d\r\n",
                         (int)ctx->v92_.info_->v92CapLocal_, (int)obj->info1dPending_, (int)(info[7] & 0x20u), (int)ctx->v92Session_);

  /* V.92Lite: PCM upstream means a retrain to V.34 upstream */
  if (ctx->v92Session_ != 0 && (int8_t)obj->params_->flags2 >= 0)
  {
    if (dsplibs_debug_level > 1u)
      dsplibs_debug_printf("VPcmV34Main: we got PCM upstream under V.92Lite (after Info1d), retraining to V.34 upstream...\r\n");
    VPcmV34InitiateRetrain(obj, DP_V90);
    obj->b_ac00_ = 1;
    ret = 1;
  }
  return ret;
}

/**
 * @brief The V.34 handshake began transmitting Ja: tell the active engine to enter phase 3.
 * @param obj V.34 module object; the V.90 / V.92 engine is used when pcmMode_ > 1, the K56Flex engine when k56Mode_ > 1
 */
void indicateJaTransmission(tagV34Object* obj)
{
  VPcmFloModem* ctx = obj->ctx_;
  K56FlexHandle* k56 = obj->k56_;
  if (obj->pcmMode_ > 1)
    ctx->enterPhase3();
  else if (obj->k56Mode_ > 1)
    ((K56FlexFloModem*)k56)->enterPhase3FullDuplex();
}

/**
 * @brief Current receive bit rate in bit/s.
 * @param obj V.34 module object
 * @return the V.90 demodulator's rate when answering in a V.90 / V.92 mode, the K56Flex handle's rate word in K56Flex mode, else rxRateIdx_ * 2400
 */
int VPcmV34GetCurrentRxBitRate(tagV34Object* obj)
{
  VPcmFloModem* ctx = obj->ctx_;
  K56FlexHandle* k56 = obj->k56_;
  if (obj->modeChar_ == V34ROLE_ANSWER)
  {
    if ((unsigned)(obj->mode_ - 1) <= 1u)
      return ctx->v90_.dem_->getBitRate();
  }
  else if (obj->mode_ == VPCM_MODE_K56FLEX)
  {
    return (int)k56->word0_;
  }
  return (int)obj->rate.rxRateIdx_ * 0x960;
}

/**
 * @brief Restart the V.34 session in the MOH (modem-on-hold) state: set the MOH words, restart the handshake, retrain state and ANSam detector.
 * @param obj V.34 module object
 * @param a1  MOH echo-cancel word; the value 1 selects the special bypass (MOHreq is sent instead of MOHFRR, mohF0_ and the a3 byte are cleared)
 * @param a2  MOH flag byte (also stored non-zero-normalised in mohFA_)
 * @param a3  MOH flag byte
 */
void VPcmV34InitMOH(tagV34Object* obj, unsigned a1, unsigned char a2, unsigned char a3)
{
  VPcmFloModem* ctx = obj->ctx_;
  uint8_t b3 = a3;
  obj->params_->flags3 = (uint8_t)(obj->params_->flags3 & 0xfbu);
  obj->mohEC_ = a1;
  if (a1 == 1)
  {
    if (dsplibs_debug_level > 1u)
      dsplibs_debug_printf("VPcmV34Main: Special bypass - sending MOHreq instead of MOHFRR...\r\n");
    obj->mohF0_ = 0;
    b3 = 0;
  }
  else
  {
    obj->mohF0_ = a1;
  }

  /* MOH words */
  obj->mohF9_ = b3;
  obj->mohF4_ = 5;
  obj->mohFA_ = (uint8_t)(a2 != 0);
  obj->mohE9_ = a2;
  obj->mohE2_ = 0;
  obj->mohE6_ = 0;
  obj->mohE4_ = 0;
  obj->mohE0_ = 0;
  obj->mohD8_ = 0;

  /* restart the handshake in the retrain run mode */
  v34handshakinit(obj, V34HSINIT_MOH);
  obj->state_ = 7;
  obj->rx.retrainCnt_ = 0;
  obj->rx.rrnUpCnt_ = 0;
  obj->rx.rrnDownCnt_ = 0;
  obj->mod.runMode_ = V34RUN_RETRAIN;
  obj->mode_ = VPCM_MODE_V34;
  ctx->ansam_.reset();
  resetRetrain(obj);
}

/**
 * @brief Apply the parameter block's minimum and maximum bit rates.
 *
 * On the analog side of a V.90 / V.92 session they go to the V.90 constellation designer, in a K56Flex session to the K56Flex engine; otherwise
 * they become V.34 rate indices (rate / 2400), the minimum capped at 14 and the maximum raised to at least the minimum and capped at 14
 * (w359a_ is set when the maximum index is 1).
 * @param obj V.34 module object
 */
void VPcmV34SetMinMaxBitRates(tagV34Object* obj)
{
  int pm = obj->pcmMode_;
  VPcmFloModem* ctx = obj->ctx_;
  K56FlexHandle* k56 = obj->k56_;
  if (pm != 0 && ctx->flag6120_ != 0)
  {
    V01_ModemParams* p = obj->params_;
    ctx->v90_.dem_->cdes_->setMinMaxRates(p->minRate, p->maxRate);
    return;
  }
  if (obj->k56Mode_ != 0 && k56->flag8_ != 0)
  {
    V01_ModemParams* p = obj->params_;
    ((K56FlexFloModem*)k56)->setMinMaxRates((int)p->minRate, (int)p->maxRate);
    return;
  }
  {
    V01_ModemParams* p = obj->params_;
    int e;
    obj->minRateIdx_ = (int32_t)(p->minRate / 2400u);
    obj->maxRateIdx_ = (int32_t)(p->maxRate / 2400u);
    if (obj->minRateIdx_ > 14)
      obj->minRateIdx_ = 14;
    e = obj->maxRateIdx_;
    if (e < obj->minRateIdx_)
    {
      obj->maxRateIdx_ = obj->minRateIdx_;
      e = obj->minRateIdx_;
    }
    if (e > 14)
    {
      obj->maxRateIdx_ = 14;
      return;
    }
    if (e == 1)
      obj->w359a_ = 1;
  }
}

/**
 * @brief Derive the V.34 filter delay and the DMA delay from the parameter block's delays and set the echo canceller's echo delay.
 *
 * filtDelay_ = (delay + 2) / 4 + 0x22, dmaDelay_ = 0x610 - extDelay, echo delay = extDelay + 0x68.
 * @param obj V.34 module object
 */
void VPcmV34SetDelays(tagV34Object* obj)
{
  unsigned lvl = dsplibs_debug_level;
  V01_ModemParams* p = obj->params_;
  VPcmFloModem* ctx = obj->ctx_;
  int d = p->delay;
  int x;
  obj->filtDelay_ = (uint16_t)(((int)((unsigned)d + 2u) >> 2) + 0x22);
  if (lvl > 1u)
  {
    dsplibs_debug_printf("V34 filtdelay set to %d (params initial delay = %d)\n", (int)(short)obj->filtDelay_, d);
    p = obj->params_;
    lvl = dsplibs_debug_level;
  }
  x = p->extDelay;
  obj->dmaDelay_ = (uint16_t)(0x610 - x);
  if (lvl > 1u)
  {
    dsplibs_debug_printf("V34FEC, V34dmadelay set to %d, (ext delay=%d)\n", (int)(short)obj->dmaDelay_, x);
    p = obj->params_;
  }
  ctx->ec_.setEchoDelay((unsigned)(p->extDelay + 0x68));
}

/**
 * @brief Constellation diagnostic from the V.34 object's own point buffer (used when no engine supplies one).
 * @param obj V.34 module object; its constCnt_ is reset to 0
 * @param out receives points, the 16-bit coordinates doubled
 * @param n   capacity of `out`
 * @return number of points written
 */
static inline unsigned visConstellation(tagV34Object* obj, int_complex* out, unsigned n)
{
  unsigned cnt = (unsigned)(int)obj->tx.constCnt_, i;
  if (cnt > n)
    cnt = n;
  obj->tx.constCnt_ = 0;
  for (i = 0; i < cnt; i++)
  {
    out[i].re_ = (int)MEMBER_ELEM_FLAT(int16_t, obj, tx.constPts_, 2 * i) * 2;
    out[i].im_ = (int)MEMBER_ELEM_FLAT(int16_t, obj, tx.constPts_, 2 * i + 1) * 2;
  }
  return cnt;
}
/** @brief Equalizer diagnostic from the V.34 object's own tap arrays: up to 0x50 points, the 16-bit coordinates doubled.  @return number written */
static inline unsigned visEqualizer(tagV34Object* obj, int_complex* out, unsigned n)
{
  unsigned cnt = n, i;
  if (cnt > 0x50u)
    cnt = 0x50;
  for (i = 0; i < cnt; i++)
  {
    out[i].re_ = (int)MEMBER_ELEM_FLAT(int16_t, obj, rx.eqRe_, i) * 2;
    out[i].im_ = (int)MEMBER_ELEM_FLAT(int16_t, obj, rx.eqIm_, i) * 2;
  }
  return cnt;
}
/** @brief Sample-buffer diagnostic: `total` 16-bit samples (at most n) as the imaginary parts, real parts 0.  @return number written */
static inline unsigned visSamples(const int16_t* src, unsigned total, int_complex* out, unsigned n)
{
  unsigned cnt = total, i;
  if (cnt > n)
    cnt = n;
  for (i = 0; i < cnt; i++)
  {
    out[i].re_ = 0;
    out[i].im_ = (int)src[i];
  }
  return cnt;
}

/**
 * @brief Visual diagnostics for the host.
 *
 * Selectors 0 .. 2 (constellation, linear equalizer, DFE) ask the V.90 / V.92 engine in a V.92 session, or in a V.90 session on the answering side,
 * and the K56Flex engine in K56Flex mode on the calling side; otherwise the V.34 object's own buffers are used (selector 2 then gives 0).  Selectors 3
 * and 4 return one point holding the resampler phase / offset; 5 the V.92 echo canceller taps (scaled by 10000) or the V.34 echo samples; 6 the PCM
 * samples in V.34 mode; 8 the K56Flex decision errors.
 * @param obj V.34 module object
 * @param sel selector 0 .. 8; other values return 0
 * @param out receives the points
 * @param n   capacity of `out`
 * @return number of int_complex written (or the engine getter's result)
 */
unsigned VPcmV34GetVisualDiagnostics(tagV34Object* obj, unsigned sel, int_complex* out, unsigned n)
{
  VPcmFloModem* ctx = obj->ctx_;
  K56FlexFloModem* k56 = (K56FlexFloModem*)obj->k56_;
  int m;
  if (sel > 8u)
    return 0;
  switch (sel)
  {
  case 0:
    m = obj->mode_;
    if (m == VPCM_MODE_V92)
      return (unsigned)ctx->getConstellation(out, n);
    if (m == VPCM_MODE_V90)
    {
      if (obj->modeChar_ == V34ROLE_ANSWER)
        return (unsigned)ctx->getConstellation(out, n);
    }
    else if (m == VPCM_MODE_K56FLEX)
    {
      if (obj->modeChar_ == V34ROLE_CALL)
        return (unsigned)k56->getConstellation(out, n);
    }
    return visConstellation(obj, out, n);
  case 1:
    m = obj->mode_;
    if (m == VPCM_MODE_V92)
      return (unsigned)ctx->getLinearEqualizer(out, n);
    if (m == VPCM_MODE_V90)
    {
      if (obj->modeChar_ == V34ROLE_ANSWER)
        return (unsigned)ctx->getLinearEqualizer(out, n);
    }
    else if (m == VPCM_MODE_K56FLEX)
    {
      if (obj->modeChar_ == V34ROLE_CALL)
        return (unsigned)k56->getLinearEqualizer(out, n);
    }
    return visEqualizer(obj, out, n);
  case 2:
    m = obj->mode_;
    if (m == VPCM_MODE_V92)
      return (unsigned)ctx->getDFE(out, n);
    if (m == VPCM_MODE_V90)
    {
      if (obj->modeChar_ == V34ROLE_ANSWER)
        return (unsigned)ctx->getDFE(out, n);
    }
    else if (m == VPCM_MODE_K56FLEX)
    {
      if (obj->modeChar_ == V34ROLE_CALL)
        return (unsigned)k56->getDFE(out, n);
    }
    return 0;
  case 3:
    m = obj->mode_;
    if ((unsigned)(m - 1) > 1u)
    {
      if (m == VPCM_MODE_K56FLEX && obj->modeChar_ == V34ROLE_CALL)
        k56->getResamplerPhase(out, n);
      else
        out[0].re_ = obj->rx.resPhase_;
    }
    out[0].im_ = 0;
    return 1;
  case 4:
    m = obj->mode_;
    if ((unsigned)(m - 1) > 1u)
    {
      if (m == VPCM_MODE_K56FLEX && obj->modeChar_ == V34ROLE_CALL)
        k56->getResamplerOffset(out, n);
      else
        out[0].re_ = (int)obj->rx.resOffs_;
    }
    out[0].im_ = 0;
    return 1;
  case 5:
    if (obj->mode_ == VPCM_MODE_V92)
    {
      V92EchoCanceller* ec = &ctx->ec_;
      unsigned cnt = ec->nCoef_, i;
      const float* src;
      if (cnt > n)
        cnt = n;
      src = ec->coef_;
      for (i = 0; i < cnt; i++)
      {
        out[i].re_ = 0;
        out[i].im_ = dsp_trunc_i32(loadLD(src[i]) * (LD)10000.0f);
      }
      return cnt;
    }
    return visSamples(obj->echo0.echoBuf_, obj->echo0.echoCnt_, out, n);
  case 6:
    if (obj->mode_ != VPCM_MODE_V34)
      return 0;
    return visSamples(obj->echo1.echoBuf_, obj->echo1.echoCnt_, out, n);
  case 7:
    return 0;
  default: /* 8 */
    m = obj->mode_;
    if ((unsigned)(m - 1) <= 1u)
      return 0;
    if (m == VPCM_MODE_K56FLEX && obj->modeChar_ == V34ROLE_CALL)
      return (unsigned)k56->getDecisionErrors(out, n);
    return 0;
  }
}

/**
 * @brief Common tail of VPcmV34GetDiagnostics(): copy the V.34 object's diagnostic counters (dgC*) into the record.
 * @param obj V.34 module object
 * @param out record being filled
 * @return the value stored in out->w228_ (dgC0c_, or -1 when it is 0)
 */
static inline int diagTail(tagV34Object* obj, V03_DiagRecord* out)
{
  int a = obj->dgC0e_;
  uint32_t b = obj->dgC04_;
  int c = obj->dgC10_;
  int e = obj->dgC14_;
  uint32_t f = obj->dgC08_;
  int16_t g;
  int h;
  out->wc0_ = (int32_t)b;
  out->wec_ = a + c;
  g = obj->dgC0c_;
  h = obj->dgC12_;
  out->wc4_ = (int32_t)f;
  out->wf4_ = e;
  out->wf0_ = h;
  out->w100_ = h;
  out->we8_ = h;
  if (g == 0)
  {
    out->w228_ = -1;
    return -1;
  }
  out->w228_ = (int)g;
  return (int)g;
}

/**
 * @brief Fill a diagnostics record for the current session type.
 *
 * V.90 / V.92 on the analog side: the V.90 demodulator's getAT_UD() fills the base record, then the symbol rate, SNR and transmit rate are set (V.92
 * mode: 8000 baud, SNR constant -12 dB, transmit rate from the V.92 transmitter's bit count when its state is 3).  The digital side only prints.
 * Every other mode takes the V.34 rates, carriers, SNR and the error-ratio estimate (qcA_ / qcB_: the number of 6 dB steps, then 1 dB steps,
 * until the ratio drops to zero).  All modes finish with the shared tail.
 * @param obj V.34 module object
 * @param out record to fill
 * @return the value stored in out->w228_
 */
int VPcmV34GetDiagnostics(tagV34Object* obj, V03_DiagRecord* out)
{
  unsigned lvl = dsplibs_debug_level;
  VPcmFloModem* ctx = obj->ctx_;
  int mode;
  if (lvl > 1u)
  {
    dsplibs_debug_printf("VPcmV34GetDiagnostics called...\r\n");
    mode = obj->mode_;
    lvl = dsplibs_debug_level;
  }
  else
  {
    mode = obj->mode_;
  }
  /* V.90 session */
  if (mode == VPCM_MODE_V90)
  {
    if (ctx->flag6120_ != 0)
    {
      int d, c, rate;
      if (lvl > 1u)
        dsplibs_debug_printf("...It is diagnostics of V.90 Analog...\r\n");
      ctx->v90_.dem_->getAT_UD((TAG_DiagnosticResults*)out);
      d = obj->rate.txRateIdx_;
      c = obj->rate.baud_;
      rate = (int)(int16_t)c;
      out->baud_ = rate;
      out->snrDb_ = (float)((LD)-12.0f - (LD)(int)(int16_t)obj->tx.w25dc_);
      out->txRate_ = d * 0x960;
      out->carrier_ = (int)(int16_t)obj->rate.carrier_;
    }
    else if (lvl > 1u)
    {
      dsplibs_debug_printf("...It is diagnostics of V.90 Digital...\r\n");
    }
    return diagTail(obj, out);
  }
  /* V.92 session */
  if (mode == VPCM_MODE_V92)
  {
    if (ctx->flag6120_ != 0)
    {
      V92Modulator* mod;
      int r = 0;
      if (lvl > 1u)
        dsplibs_debug_printf("...It is diagnostics of V.92 Analog...\r\n");
      ctx->v90_.dem_->getAT_UD((TAG_DiagnosticResults*)out);
      out->baud_ = 0x1f40;
      out->snrDb_ = (float)-12.0f;
      mod = ctx->v92_.mod_;
      if (mod->state_ == 3)
      {
        uint32_t u = mod->b2s_->tx_->w0_ * 0x1f40u;
        LD v = (LD)0.5f + (LD)u * (LD)0.083333336f;
        r = (int)(uint32_t)(long long)v;
      }
      out->txRate_ = r;
      out->carrier_ = 0;
    }
    else if (lvl > 1u)
    {
      dsplibs_debug_printf("...It is diagnostics of V.92 Digital...\r\n");
    }
    return diagTail(obj, out);
  }
  /* plain V.34 (or phase 2) */
  if (lvl > 1u)
    dsplibs_debug_printf("...It is diagnostics of V.34 (or P2)...\r\n");
  {
    int d = obj->rate.txRateIdx_, c = obj->rate.rxRateIdx_, baud = (int)(int16_t)obj->rate.baud_;
    int16_t divisor;
    int ratio, lastRatio = 0, qcSteps = 0, tailRatio;
    /* rates, carriers and SNR */
    out->baud_ = baud;
    out->txRate_ = d * 0x960;
    out->rxRate_ = c * 0x960;
    out->wbc_ = (int)obj->rate.rxCarrier_;
    out->carrier_ = (int)(int16_t)obj->rate.carrier_;
    out->wb4_ = (int)obj->rate.rxBaud_;
    out->snrDb_ = (float)((LD)-12.0f - (LD)(int)(int16_t)obj->tx.w25dc_);

    /* error-ratio estimate: count the 6 dB steps, then the 1 dB steps, until the ratio reaches zero */
    divisor = obj->rx.equErr_;
    if (divisor > 0)
    {
      ratio = obj->rx.w4ac_ / (int)divisor;
      if (ratio > 0)
      {
        for (;;)
        {
          lastRatio = ratio;
          ratio = (int)((unsigned)ratio * 0x1013u) >> 14;
          if (ratio > 0)
            qcSteps += 6;
          else
            break;
        }
      }
    }
    if (lastRatio > 0)
    {
      tailRatio = lastRatio;
      for (;;)
      {
        tailRatio = (int)((unsigned)tailRatio * 0x32d6u) >> 14;
        if (tailRatio > 0)
          qcSteps++;
        else
          break;
      }
    }

    out->dly84_ = (int)obj->bulkDelay_;
    out->dly80_ = (int)obj->bulkDelay_;
    out->qcA_ = (float)qcSteps;
    out->qcB_ = (float)qcSteps;
  }
  return diagTail(obj, out);
}
