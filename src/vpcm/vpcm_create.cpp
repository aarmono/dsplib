/**
 * @file vpcm_create.cpp
 * @brief VPcmV34Create: constructor of the V.34 module object for a PCM-capable session.
 *
 * Clears and initialises a tagV34Object for a new session and selects its mode from the session type.  It is C++ because it calls methods of the
 * V.90 / V.92 / K56Flex engines: VPcmFloModem::externalReset(), K56FlexFloModem::externalReset() / setMinMaxRates(),
 * V90Demodulator::enterChannelVerification(), V90ConstellationDesigner::setMinMaxRates(), V92EchoCanceller::setEchoDelay() and
 * GenericIIR<float,double>::reset().
 *
 * @code
 * int VPcmV34Create(tagV34Object* obj, int answerer, int frag, V01_ModemParams* pm, int mode);   // always returns 0
 * @endcode
 * - answerer: the ANSWER flag; dp_vpcm passes `caller == 0`.  Its sense depends on mode (below).
 * - frag: the fragment length of the session, stored in obj->w8_.
 * - pm: the dp parameter block; the first reads use it directly, afterwards obj->params_ is used.
 * - mode: the session type (signed compares): 2 = V.92, 1 = V.90, 3 / 4 = K56Flex (4 = A-law), anything else = plain V.34.
 *
 * Steps: print the entry state; save ctx_ and k56_ across a memset of the object; reset the MOH, bulk-delay and V.92 info words; select pcmMode_,
 * k56Mode_, mode_ and the role (modeChar_) by session type; initialise the handshake (v34handshakinit) and the retrain detector; apply the minimum /
 * maximum rates to the active engine or the V.34 rate indices; derive the minimum signal level, the filter and DMA delays and the echo delay; choose
 * the entrance filter setting and reset the IIR.
 *
 * Details worth knowing:
 * - ctx_ and the K56 handle are read at entry, before the memset, and restored after it.  ctx0 (the entry copy) is used for the V.92 info block, the
 *   session word, the entrance filter and the IIR reset; the rate-limit part and the echo delay re-read obj->ctx_ and obj->k56_.
 * - The sense of `answerer`: mode 2: modeChar_ is the answer role iff answerer == 0, and the V.92 session word is set to 1 otherwise; mode 1: the
 *   argument is replaced by (answerer == 0), the answer role iff that is non-zero; every other mode: the answer role iff answerer != 0.
 * - Mode 2 copies four bytes of the V92Phase2Info into pair0_ .. pair2_ / wabd4_, sets its local V.92 capability to 1 and its short-phase-2 flag to the
 *   quick-connect bit; with quick connect on the answering side it starts the channel verification (mode_ = QC line verification, state_ = 10).
 * - Modes 3 / 4 print through edprintf unconditionally and re-read the handle's word_c_.
 * - Rate indices are rate / 2400 (unsigned division), clamped as in VPcmV34InitiateRetrain(); wabfc_ = (uint16)(24 * pm->txMdTime / 10);
 *   w254_ = 10000 + 336 * (short) txMdMs_, read after v34handshakinit.
 * - The debug prints at entry are nested (a failed test skips the rest); the level is re-read at each print.
 */

#include "v34/v34_object.h"

#include "core/dsp_debug_level.h" /* dsplibs_debug_level, LEVEL_ON() */
#include "core/dsp_host.h"
#include "v34/v34_api.h"
#include "v34/v34_api_tables.h" /* V34DisconnectThreshTable: minimum signal level per requested level index */
#include "vpcm_v34_api.h"

/**
 * @brief Create (initialise) a V.34 module object for a new session; see the file comment for the parameters.
 * @param obj      object to initialise; its ctx_ and k56_ members must already point at the engines (they are preserved across the clearing)
 * @param answerer the ANSWER flag (sense depends on mode)
 * @param frag     fragment length, stored in obj->w8_
 * @param pm       dp parameter block, stored in obj->params_
 * @param mode     session type: 2 = V.92, 1 = V.90, 3 / 4 = K56Flex, other = plain V.34
 * @return always 0
 * @warning ctx_ is dereferenced unconditionally (V.92 info record, entrance filter, IIR), so it must be non-NULL in every mode.
 */
extern "C" int VPcmV34Create(tagV34Object* obj, int answerer, int frag, V01_ModemParams* pm, int mode)
{
  VPcmFloModem* ctx0 = obj->ctx_;
  K56FlexHandle* k56 = obj->k56_;
  uint8_t* rx = (uint8_t*)&obj->rx.rxW0_; /* start of the RX front-end sub-object, which is cleared separately below */
  unsigned qc = (pm->flags2 >> 4) & 1u;
  int ansLevel = pm->ansLevel;
  int uqtsIdx = pm->qcIndex;
  bool answerRole; /* modeChar_ = V34ROLE_ANSWER, else V34ROLE_CALL */

  /* entry trace */
  if (LEVEL_ON())
  {
    dsplibs_debug_printf("VPcmV34Create: quick connect indication from phase1 = %d\r\n", (int)qc);
    if (LEVEL_ON())
    {
      dsplibs_debug_printf("VPcmV34Create: Uqts index is %d\r\n", uqtsIdx);
      if (LEVEL_ON())
      {
        dsplibs_debug_printf("VPcmV34Create: ANSpcm level index is %d\r\n", ansLevel);
        if (LEVEL_ON())
        {
          dsplibs_debug_printf("VPcmV34Create, initial Session Type = %d\n", mode);
          if (LEVEL_ON())
            dsplibs_debug_printf("RX at %lX\n", (unsigned long)rx);
        }
      }
    }
  }

  /* clear the object and its RX front end, then restore the engine handles and set the fixed defaults */
  sysdep_memset(obj, 0, sizeof(*obj));
  sysdep_memset(rx, 0, (unsigned)(&obj->rx.unk08b0_[0x150] - rx));
  obj->k56_ = k56;
  obj->waa80_ = 1;
  obj->params_ = pm;
  obj->ctx_ = ctx0;
  V34InitializeImplementationSpecific(obj);

  obj->echoOn_ = 1;
  obj->bulkWr_ = 0;
  obj->bulkRd_ = 0;
  obj->bulkBase_ = obj->unk35b8_;
  obj->bulkLen_ = 0x2580;
  obj->mohD8_ = 0;
  obj->wabdc_ = 0;
  obj->mohE0_ = 0;
  obj->mohE2_ = 0;
  obj->mohE4_ = 0;
  obj->mohE6_ = 0;
  obj->txMdMs_ = 0;
  if (LEVEL_ON())
    dsplibs_debug_printf("On Create: Setting desired TX MD (%d mSec)!\n", 0);

  /* V.92 phase 2 info and handshake words */
  obj->wabfc_ = (uint16_t)((uint32_t)(pm->txMdTime * 24u) / 10u);
  ctx0->v92_.info_->shortPhase2Local_ = 0;
  ctx0->v92_.info_->v92CapLocal_ = 0;
  obj->wabc4_ = 0;
  obj->wabc6_ = 0;
  obj->info1dPending_ = 0;
  obj->wabca_ = 0;
  obj->shortPhase2_ = 0;
  obj->wabd4_ = 0;
  obj->pair2_ = 0;
  obj->pair1_ = 0;
  obj->pair0_ = 0;
  ctx0->v92Session_ = 0;
  obj->mode_ = VPCM_MODE_V34;
  obj->state_ = 0;
  obj->k56Mode_ = 0;

  /* select the session type, the engines and the role */
  if (mode == 2)
  {
    bool zero;
    k56->word_c_ = 0;
    obj->pcmMode_ = 1;
    ctx0->externalReset();
    obj->wabc6_ = 1;
    obj->wabca_ = (uint16_t)qc;
    {
      V92Phase2Info* info = ctx0->v92_.info_;
      obj->wabd4_ = info->v90UseHighCarrier_;
      obj->pair2_ = info->b16_;
      obj->pair1_ = info->b15_;
      obj->pair0_ = info->b14_;
      info->v92CapLocal_ = 1;
    }
    ctx0->v92_.info_->shortPhase2Local_ = (uint8_t)qc;
    zero = (answerer == 0);
    if (qc != 0 && zero)
    {
      obj->mode_ = VPCM_MODE_QC_LINE_VERIFICATION;
      ctx0->v90_.dem_->enterChannelVerification((short)uqtsIdx, (short)ansLevel);
      obj->state_ = 10;
    }
    answerRole = zero;
    if (!zero)
      ctx0->v92Session_ = 1;
  }
  else
  {
    int q = answerer;
    if (mode < 2)
    {
      k56->word_c_ = 0;
      if (mode == 1)
      {
        q = (answerer == 0);
        obj->pcmMode_ = 1;
        ctx0->externalReset();
      }
      else
      {
        obj->pcmMode_ = 0;
      }
    }
    else if (mode == 3 || mode == 4)
    {
      k56->word_c_ = (mode == 4) ? 1 : 0;
      obj->pcmMode_ = 0;
      edprintf("VPcmV34Main: it is K56Flex session type, (AorMu = %d)\r\n", k56->word_c_);
      obj->k56Mode_ = 1;
      ((K56FlexFloModem*)k56)->externalReset();
    }
    else
    {
      k56->word_c_ = 0;
      obj->pcmMode_ = 0;
    }
    answerRole = (q != 0);
  }

  /* handshake and rate defaults */
  obj->modeChar_ = answerRole ? V34ROLE_ANSWER : V34ROLE_CALL;
  obj->w8_ = frag;
  obj->rxWordCnt_ = 0;
  obj->txTail_ = 0;
  obj->maxRateIdx_ = 0xe;
  obj->sampleTime_ = 0;
  obj->txHead_ = 0;
  obj->minRateIdx_ = 0;
  obj->w228_ = 0;
  obj->w22c_ = -1;
  obj->minSigLevel_ = 0;
  obj->w234_ = 0;
  obj->dataStartTime_ = (int32_t)0xfffe8900;
  obj->w359a_ = 0;
  obj->rx.rxGainEst_ = 0x600;
  obj->rx.w49c_ = 0;
  obj->w0c_ = 0;
  obj->w10_ = 0;
  v34handshakinit(obj, V34HSINIT_START);

  /* reset the retrain detector and the diagnostic counters */
  {
    int w35a4 = (int)(int16_t)obj->txMdMs_;
    int mc;
    obj->tx.w2aa6_ = 0;
    obj->retrain.rd2_ = 0;
    obj->retrain.rd4_ = 0;
    obj->retrain.rd6_ = 0;
    obj->retrain.rd8_ = 0;
    obj->retrain.rd14_ = 0;
    obj->retrain.rd18_ = 0;
    obj->retrain.rd1c_ = 0;
    obj->w256_ = 0;
    obj->w258_ = 0;
    obj->w254_ = (uint16_t)(10000 + 336 * w35a4);
    obj->retrain.rd0_ = 0;
    mc = (int)(int16_t)obj->modeChar_;
    if ((int16_t)mc == V34ROLE_CALL)
    {
      obj->retrain.rdc_ = 0;
      obj->retrain.rde_ = 0;
      obj->retrain.rd10_ = 0x39c3;
    }
    else if ((int16_t)mc == V34ROLE_ANSWER)
    {
      obj->retrain.rdc_ = 0x5a82;
      obj->retrain.rde_ = 0x55fc;
      obj->retrain.rd10_ = 0x39c3;
    }
  }
  obj->b_ac16_ = 0;
  obj->dgC0c_ = 0;
  obj->b_ac17_ = 0;
  obj->w262_ = 1;
  obj->dgC04_ = 0;
  obj->dgC08_ = 0;
  obj->dgC0e_ = 0;
  obj->dgC10_ = 0;
  obj->dgC12_ = 0;
  obj->dgC14_ = 0;
  obj->echoThresh_ = 0x7d0;
  obj->wa248_ = 0;
  obj->echoK2_ = 0x7fdf;
  obj->echoBeta_ = 2;

  /* minimum / maximum bit rates for the active engine */
  {
    int pcm = obj->pcmMode_;
    VPcmFloModem* ctx1 = obj->ctx_;
    K56FlexHandle* k56b = obj->k56_;
    if (pcm != 0 && ctx1->flag6120_ != 0)
    {
      V01_ModemParams* p = obj->params_;
      ctx1->v90_.dem_->cdes_->setMinMaxRates(p->minRate, p->maxRate);
    }
    else if (obj->k56Mode_ != 0 && k56b->flag8_ != 0)
    {
      V01_ModemParams* p = obj->params_;
      ((K56FlexFloModem*)k56b)->setMinMaxRates((int)p->minRate, (int)p->maxRate);
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
  }

  /* minimum signal level, filter / DMA delays, echo delay */
  {
    V01_ModemParams* p = obj->params_;
    unsigned k = (unsigned)p->minLevel + 0x30u;
    unsigned lvl;
    int thr, d, x;
    VPcmFloModem* ctx2;
    if (k > 7u)
      k = 3;
    thr = V34DisconnectThreshTable[k];
    obj->minSigLevel_ = thr;
    if (LEVEL_ON())
    {
      dsplibs_debug_printf("VPcmV34Main: minLevel given is %d , minSigLevel set to %d\n", p->minLevel, thr);
    }
    p = obj->params_;
    obj->w234_ = 0;
    lvl = dsplibs_debug_level;
    d = p->delay;
    ctx2 = obj->ctx_;
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
    ctx2->ec_.setEchoDelay((unsigned)(obj->params_->extDelay + 0x68));
  }

  /* entrance filter: -1 follows the hardware code in the parameter block, 1 forces it on, anything else forces it off */
  {
    int ef = obj->params_->entranceFilter;
    if (ef == -1)
    {
      if (LEVEL_ON())
        dsplibs_debug_printf("VPcmFlo: From Stream - Entrance Filter according to HW...\r\n");
      ctx0->entranceFilter_ = (obj->params_->codecType == 0xe) ? 1 : 0;
    }
    else if (ef == 1)
    {
      if (LEVEL_ON())
        dsplibs_debug_printf("VPcmFlo: From Stream - Entrance Filter forced enabled...\r\n");
      ctx0->entranceFilter_ = 1;
    }
    else
    {
      if (LEVEL_ON())
        dsplibs_debug_printf("VPcmFlo: From Stream - Entrance Filter forced disabled...\r\n");
      ctx0->entranceFilter_ = 0;
    }
  }

  ctx0->iir_.reset();
  if (LEVEL_ON())
    dsplibs_debug_printf("VPcmFlo: YES! entrance filter applied = %d\r\n", (int)ctx0->entranceFilter_);
  return 0;
}
