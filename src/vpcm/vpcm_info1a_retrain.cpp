/**
 * @file vpcm_info1a_retrain.cpp
 * @brief V.34 INFO1a / INFO1c / INFO1d bit packing for the PCM and K56Flex modems, and the retrain initiator.
 *
 * Two plain C entry points, written as extern "C" C++ because they call methods of the V.90 classes (VPcmFloModem::getUinfoValue() / setPcmSessionType(),
 * V92EchoCanceller::setEchoDelay(), K56FlexFloModem::setMinMaxRates(), V90ConstellationDesigner::setMinMaxRates()):
 *  - V34SetINFO1aBits(): fills the 16-bit words of the INFO0 / INFO1 frame about to be sent (K56Flex INFO1c / INFO1a bits, the V.90 / V.92 upstream
 *    selection) and sets pcmMode_ to 2; always returns 0;
 *  - VPcmV34InitiateRetrain(): restarts the V.34 handshake in the requested modulation (V.34, K56Flex, V.90 or V.92): sets the rate limits, the minimum
 *    signal level, the filter / DMA / echo delays, the modulation flags, calls v34handshakinit() for a retrain and resets the retrain-request detector.
 *
 * Both work on the session object tagV34Object (v34/v34_object.h).  Every debug print is guarded by the unsigned test dsplibs_debug_level > 1 (LEVEL_ON()),
 * read at the point of the print; the K56Flex handle is a block whose word_c_ / flag8_ members are read as they stand.  bitreverse() results are signed 16-bit
 * values before the shifts; the INFO words are updated with 16-bit read-modify-writes.
 */

#include "v34/v34_object.h"
#include "v34/v34_symbol_rates.h" /* V34_BAUD_*, V34_CARRIER_* */

#include "core/dsp_debug_level.h" /* dsplibs_debug_level, LEVEL_ON() */
#include "core/host_interface.h"  /* DP_V34 / DP_K56 / DP_V90 / DP_V92 */
#include "core/dsp_host.h"
#include "v34/v34_api.h"
#include "v34/v34_api_tables.h" /* V34DisconnectThreshTable */
#include "vpcm_v34_api.h"

/** @brief Sign-extend a 16-bit value to int. */
static inline int s16(unsigned short v) { return (int)(short)v; }

/**
 * @brief Set the PCM / K56Flex bits of the INFO frame that is about to be sent.
 * @param obj   session object
 * @param info  INFO frame words (16-bit); updated in place
 * @return 0 always
 * @note  Sets obj->pcmMode_ to 2 when a PCM modem is selected, or to 0 when getUinfoValue() reports no upstream rate.  Caller-analog K56Flex sends
 *        INFO1c, answer-digital K56Flex INFO1a; the V.92 branch selects PCM upstream, otherwise V.34 upstream with the V.90 receiver announced.
 */
int V34SetINFO1aBits(tagV34Object* obj, uint16_t* info)
{
  int a = obj->pcmMode_;
  K56FlexHandle* k56 = obj->k56_;
  VPcmFloModem* ctx = obj->ctx_;
  int b = obj->k56Mode_;

  /* plain V.34 session: transmit the MD/MS length bits */
  if (a == 0 && b == 0)
  {
    int r = s16(bitreverse((uint16_t)obj->txMdMs_, 7));
    info[0] = (uint16_t)(info[0] | ((r >> 5) & 3));
    info[1] = (uint16_t)(info[1] | (((unsigned)r << 3) & 0xf8u));
    b = obj->k56Mode_;
  }

  /* K56Flex: caller (analog) sends INFO1c, answerer (digital) INFO1a */
  if (b != 0)
  {
    if (obj->modeChar_ == V34ROLE_CALL)
    {
      if (LEVEL_ON())
        dsplibs_debug_printf("V34SetINFO1aBits: Setting INFO1c for K56Flex (Caller-Analog)...\r\n");
      int r = s16(bitreverse((uint16_t)obj->txMdMs_, 7));
      info[0] = (uint16_t)(info[0] | ((r >> 5) & 3));
      info[1] = (uint16_t)(info[1] | (((unsigned)r << 3) & 0xf8u));
      {
        unsigned t = info[8] & ~0xeu;
        info[8] = (uint16_t)(k56->word_c_ == 1 ? (t | 0x13u) : (t | 0x11u));
      }
      info[9] = (uint16_t)(info[9] & 0x3ffu);
    }
    else
    {
      unsigned t;
      if (LEVEL_ON())
        dsplibs_debug_printf("V34SetINFO1aBits: Setting INFO1a for K56Flex (Answer-Digital)...\r\n");
      t = info[3] & ~6u;
      info[3] = (uint16_t)(k56->word_c_ != 0 ? (t | 9u) : (t | 0xbu));
    }
  }

  if (obj->pcmMode_ == 0)
    return 0;

  /* PCM session without the flag word: INFO1d only, V.92 bit chosen by the session type */
  if (ctx->flag6120_ == 0)
  {
    if (LEVEL_ON())
      dsplibs_debug_printf("VPcmV34Main, setINFO1dBits\n");
    if (ctx->v92Session_ == 0)
      info[7] = (uint16_t)(info[7] & 0xdfu);
    else
      info[7] = (uint16_t)(info[7] | 0x20u);
    obj->pcmMode_ = 2;
    return 0;
  }

  /* PCM session: INFO1a with the upstream selection */
  {
    uint16_t sp;
    int u;
    if (LEVEL_ON())
      dsplibs_debug_printf("VPcmV34Main, setINFO1aBits\n");
    sp = obj->shortPhase2_;

    /* short phase 2: fixed 3200 baud, 1829 Hz carrier, no pre-emphasis */
    if (sp != 0)
    {
      if (LEVEL_ON())
      {
        dsplibs_debug_printf("VPcmV34Main: Short phase2 (Hard Coded) !!!\r\n");
        sp = obj->shortPhase2_;
      }
      ctx->v92Session_ = 0;
      obj->rate.baud_ = V34_BAUD_3200;
      obj->rate.carrier_ = V34_CARRIER_1829;
      obj->rate.preemp_ = 0;
      obj->tx.w25dc_ = 0;
    }

    u = (short)ctx->getUinfoValue((short)sp);
    if (u == 0)
    {
      obj->pcmMode_ = 0;
      return 0;
    }

    if (ctx->v92Session_ != 0)
    {
      /* V.92: PCM upstream; INFO word 0 carries bits 0 and 1 of pair0_, pair1_ and pair2_ in bits 7..2 */
      unsigned e = 0;
      int r;
      if (LEVEL_ON())
        dsplibs_debug_printf("VPcmV34Main: PCM Upstream is selected (info1)...\r\n");
      info[0] = 0;
      if (obj->pair0_ & 1u)
        e = 0x80;
      info[0] = (uint16_t)e;
      if (obj->pair0_ & 2u)
        e |= 0x40;
      info[0] = (uint16_t)e;
      if (obj->pair1_ & 1u)
        e |= 0x20;
      info[0] = (uint16_t)e;
      if (obj->pair1_ & 2u)
        e |= 0x10;
      info[0] = (uint16_t)e;
      if (obj->pair2_ & 1u)
        e |= 8;
      info[0] = (uint16_t)e;
      if (obj->pair2_ & 2u)
        e |= 4;
      info[0] = (uint16_t)e;

      info[1] = 0;
      r = s16(bitreverse((uint16_t)u, 7));
      info[1] = (uint16_t)(info[1] | ((r >> 4) & 7));
      info[2] = (uint16_t)(((unsigned)r << 4) & 0xf0u);
      r = s16(bitreverse(6, 3));
      info[2] = (uint16_t)(info[2] | ((r >> 1) & 3));
      info[3] = (uint16_t)(((unsigned)r << 7) & 0x80u);
      r = s16(bitreverse(6, 3));
      info[4] = 0xfc;
      info[3] = (uint16_t)(info[3] | (((unsigned)r << 4) & 0x70u) | 0xfu);
    }
    else
    {
      /* V.90: V.34 upstream; the rate fields come from the MD/MS length, the upstream rate and the previous INFO words */
      int fieldBits = 4;
      int r;
      if (LEVEL_ON())
        dsplibs_debug_printf("VPcmV34Main: V.34 Upstream is selected (info1)...\r\n");
      if (obj->shortPhase2_ == 0)
        fieldBits = s16(bitreverse((uint16_t)(((info[2] & 3u) << 1) | ((info[3] & 0x80u) >> 7)), 3));
      r = s16(bitreverse((uint16_t)obj->txMdMs_, 7));
      info[1] = (uint16_t)(((unsigned)r << 3) & 0xf8u);
      info[0] = (uint16_t)((r >> 5) & 3);
      r = s16(bitreverse((uint16_t)u, 7));
      info[1] = (uint16_t)(info[1] | ((r >> 4) & 7));
      info[2] = (uint16_t)(((unsigned)r << 4) & 0xf0u);
      r = s16(bitreverse((uint16_t)fieldBits, 3));
      info[3] = (uint16_t)(((unsigned)r << 7) & 0x80u);
      info[2] = (uint16_t)(info[2] | ((r >> 1) & 3));
      r = s16(bitreverse(6, 3));
      info[4] = 0;
      info[3] = (uint16_t)(info[3] | (((unsigned)r << 4) & 0x70u));
      if (obj->shortPhase2_ != 0)
        info[4] = 1;
      if (LEVEL_ON())
        dsplibs_debug_printf("VPcmV34Main: It is ===== V90 receiver ==== from Now on\n");
    }
  }

  obj->pcmMode_ = 2;
  return 0;
}

/**
 * @brief Restart the handshake in the requested modulation (retrain).
 * @param obj      session object
 * @param modeArg  requested data pump: 0 keep the current one, else DP_V34 / DP_K56 / DP_V90 / DP_V92; a modulation whose parameter flag is off, or an
 *                 unknown value, is replaced by 0 (keep) with a debug print
 * @note  Programs the rate limits of the active modem (V.90 designer, K56Flex, or the V.34 rate indices clamped to 14), the minimum signal level, the filter /
 *        DMA / echo delays and the sensitive-ISP flag, then runs v34handshakinit() and puts the session in state 7 / mode V.34 with the retrain-request
 *        detector cleared.
 * @bug   The minimum signal level index is minLevel + 0x30 and any result above 7 is replaced by 3, so every non-negative `minLevel` selects table entry 3
 *        (the intended index looks like minLevel itself).
 */
void VPcmV34InitiateRetrain(tagV34Object* obj, unsigned char modeArg)
{
  uint8_t mode = modeArg;
  VPcmFloModem* ctx1 = obj->ctx_;
  VPcmFloModem* ctx2;
  VPcmFloModem* ctx3;
  K56FlexHandle* k56;
  unsigned lvl;
  int a;

  if (LEVEL_ON())
    dsplibs_debug_printf("VPcmV34Main: Initiating retrain, requested DP is %d\r\n", (int)mode);
  a = obj->pcmMode_;
  ctx2 = obj->ctx_;
  k56 = obj->k56_;

  /* rate limits of the active modem */
  if (a != 0 && ctx2->flag6120_ != 0)
  {
    V01_ModemParams* p = obj->params_;
    V90ConstellationDesigner* cd = ctx2->v90_.dem_->cdes_;
    cd->setMinMaxRates(p->minRate, p->maxRate);
  }
  else if (obj->k56Mode_ != 0 && k56->flag8_ != 0)
  {
    V01_ModemParams* p = obj->params_;
    ((K56FlexFloModem*)k56)->setMinMaxRates((int)p->minRate, (int)p->maxRate);
  }
  else
  {
    V01_ModemParams* p = obj->params_;
    int lo = (int)(p->minRate / 2400u);
    int hi = (int)(p->maxRate / 2400u);
    int e;
    obj->minRateIdx_ = lo;
    obj->maxRateIdx_ = hi;
    if (lo > 14)
    {
      obj->minRateIdx_ = 14;
    }
    e = obj->maxRateIdx_;
    if (e < obj->minRateIdx_)
    {
      e = obj->minRateIdx_;
      obj->maxRateIdx_ = e;
    }
    if (e > 14)
    {
      obj->maxRateIdx_ = 14;
    }
    else if (e == 1)
    {
      obj->w359a_ = 1;
    }
  }

  /* minimum signal level, filter / DMA / echo delays */
  {
    V01_ModemParams* p = obj->params_;
    unsigned k = (unsigned)p->minLevel + 0x30u;
    int thr, d, x;
    if (k > 7u)
      k = 3;
    lvl = dsplibs_debug_level;
    thr = V34DisconnectThreshTable[k];
    obj->minSigLevel_ = thr;
    if (lvl > 1u)
      dsplibs_debug_printf("VPcmV34Main: minLevel given is %d , minSigLevel set to %d\n", p->minLevel, thr);
    p = obj->params_;
    obj->w234_ = 0;
    lvl = dsplibs_debug_level;
    d = p->delay;
    ctx3 = obj->ctx_;
    obj->filtDelay_ = (uint16_t)(((int)((unsigned)d + 2u) >> 2) + 0x22);
    if (lvl > 1u)
    {
      dsplibs_debug_printf("V34 filtdelay set to %d (params initial delay = %d)\n", (int)(short)obj->filtDelay_, d);
      lvl = dsplibs_debug_level;
    }
    p = obj->params_;
    x = p->extDelay;
    obj->dmaDelay_ = (uint16_t)(0x610 - x);
    if (lvl > 1u)
      dsplibs_debug_printf("V34FEC, V34dmadelay set to %d, (ext delay=%d)\n", (int)(short)obj->dmaDelay_, x);
    ctx3->ec_.setEchoDelay((unsigned)(obj->params_->extDelay + 0x68));
  }

  if (ctx1->v90_.params_->sensitiveIspDetected != 0)
  {
    edprintf("VPcmV34Main: Notifying Sensitive ISP detected...\r\n");
    obj->params_->flags50 = (uint8_t)(obj->params_->flags50 | 8);
  }

  a = obj->pcmMode_;
  obj->shortPhase2_ = 0;
  obj->wabca_ = 0;
  ctx1->qcStatus_ = 0;

  /* validate the requested modulation against the enabled ones (only while no PCM modem is active) */
  if (a == 0 && mode != 0)
  {
    if (mode == DP_K56)
    {
      if (!(obj->params_->flags0 & 0x10))
      {
        if (LEVEL_ON())
          dsplibs_debug_printf("VPcmV34InitiateRetrain: requested mod K56Flex, but params.flex is OFF, keeping same modulation!!!\r\n");
        mode = 0;
      }
    }
    else if (mode != DP_V34)
    {
      if (mode == DP_V90 || mode == DP_V92)
      {
        if (!(obj->params_->flags0 & 8))
        {
          if (LEVEL_ON())
            dsplibs_debug_printf("VPcmV34InitiateRetrain: requested mod V.%d, but params.v90 is OFF, keeping same modulation!!!\r\n", (int)mode);
          mode = 0;
        }
      }
      else
      {
        if (LEVEL_ON())
          dsplibs_debug_printf("VPcmV34InitiateRetrain: unknown requested mod (%d), keeping same modulation!!!\r\n", (int)mode);
        mode = 0;
      }
    }
  }

  /* select the modulation of the new handshake */
  if (mode == DP_K56)
  {
    obj->k56Mode_ = 1;
    obj->pcmMode_ = 0;
  }
  else if (mode == 0)
  {
    if (obj->pcmMode_ > 0)
      obj->pcmMode_ = 1;
  }
  else if (mode == DP_V90)
  {
    ctx1->setPcmSessionType(0);
    {
      int m = obj->mode_;
      obj->pcmMode_ = 1;
      if (m == VPCM_MODE_V92)
        obj->b_ac00_ = 1;
    }
  }
  else if (mode == DP_V92)
  {
    ctx1->setPcmSessionType(1);
    obj->pcmMode_ = 1;
  }
  else
  {
    if (mode == DP_V34)
      obj->echoOn_ = 1;
    obj->pcmMode_ = 0;
    obj->k56Mode_ = 0;
  }

  v34handshakinit(obj, V34HSINIT_RETRAIN);
  obj->state_ = 7;
  obj->mode_ = VPCM_MODE_V34;
  obj->mod.runMode_ = V34RUN_RETRAIN;
  obj->rx.retrainCnt_ = 0;
  obj->rx.rrnDownCnt_ = 0;
  obj->rx.rrnUpCnt_ = 0;
  obj->w256_ = 0;
  obj->w258_ = 0;
  obj->w254_ = (uint16_t)(10000 + 336 * (int)obj->txMdMs_);

  /* reset the retrain-request detector; its filter coefficients depend on the role */
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
}
