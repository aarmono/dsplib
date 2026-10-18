/**
 * @file v34_hs_tx_silence.cpp
 * @brief V.34 handshake transmit states SILENCE, SILENCEINFO and SILENCERETRAIN.
 *
 * One entry point, v04_tx_silence(), serves the three silent transmit states (5, 0x36, 0x4a) of the per-sample loop of v34handshak()
 * (v34_hs_dispatch.cpp).  Callees: txwritequeue(), V34SetINFO0aBits().  The loop re-evaluates its condition after the case.
 *
 * Every state writes a zero quad to the transmit queue and clears count3_.  SILENCEINFO counts 10 symbols (k56Idx_), then starts TX_DPSK:
 * transmit state 0x18, bitQ_ = &info0a, V34SetINFO0aBits(obj, &info0a), and the bit-queue record is re-initialised through the queue pointer
 * (re-read after the callee) with the role / PCM dependent limit 0x1e or 0x11.  SILENCERETRAIN counts 0xb4 symbols (retrainEnd()), then moves the
 * microstate (RX_PHASE1_CALL / TX_PHASE1_ANS by role, or MOH_TONE when mohE8_ is set), the receive state to RX_DPSK and the transmit state to TONE_AB.
 *
 * @note The debug level and the transmit state are kept in local copies that are refreshed only after a print; the arguments of the transition
 *       prints are partly stale or constant (the transmit name printed by the microstate prints is the constant SILENCERETRAIN), and SILENCEINFO's
 *       print path stores k56Idx_ = 0xa before printing so that the print sees it.
 */

#include <limits.h>
#include "v34_handshake.h"
#include "v34/v34_internal.h"

extern "C"
{
}

/**
 * @brief SILENCERETRAIN step: count symbols and, after 0xb4 of them, restart the tone phase of a retrain.
 * @param obj    handshake object
 * @param txCur  transmit state as read by the caller (refreshed after each print)
 * @note  At the limit: microstate to RX_PHASE1_CALL / TX_PHASE1_ANS (by role) or MOH_TONE (mohE8_ set), receive state to RX_DPSK, transmit state
 *        to TONE_AB, and the tone select, counters and INFO0 receive word are reset.
 */
static void retrainEnd(tagV34Object* obj, unsigned txCur)
{
  unsigned c = (uint16_t)(obj->tx.k56Idx_ + 1u);
  unsigned lv, mstCur, mstNew, rx;
  obj->tx.k56Idx_ = (uint16_t)c;
  if (c != 0xb4)
    return;

  /* microstate for the next phase */
  mstCur = obj->mst_;
  lv = dsplibs_debug_level;
  if (obj->mohE8_ == 0)
  {
    mstNew = obj->modeChar_ == V34ROLE_CALL ? V34HS_RX_PHASE1_CALL : V34HS_TX_PHASE1_ANS;
    if ((uint16_t)mstCur != mstNew)
    {
      if (lv > 1u)
      {
        v04PrMstX(mstCur, (int)mstNew, V34HS_SILENCERETRAIN, obj->rxState_, 0xb4L, v04W78(obj));
        txCur = obj->txState_;
        lv = dsplibs_debug_level;
      }
      obj->mst_ = (uint16_t)mstNew;
    }
  }
  else if ((uint16_t)mstCur != V34HS_MOH_TONE)
  {
    if (lv > 1u)
    {
      v04PrMstX(mstCur, V34HS_MOH_TONE, V34HS_SILENCERETRAIN, obj->rxState_, 0xb4L, v04W78(obj));
      txCur = obj->txState_;
      lv = dsplibs_debug_level;
    }
    obj->mst_ = V34HS_MOH_TONE;
  }

  /* receive state to RX_DPSK, transmit state to TONE_AB */
  rx = obj->rxState_;
  if ((uint16_t)rx != V34HS_RX_DPSK)
  {
    if (lv > 1u)
    {
      dsplibs_debug_printf(V04_FMT_RX, StateName[(int16_t)rx], StateName[V34HS_RX_DPSK], StateName[(int16_t)txCur], StateName[(int16_t)obj->mst_], v04K56(obj),
                           v04W78(obj));
      txCur = obj->txState_;
      lv = dsplibs_debug_level;
    }
    obj->rxState_ = V34HS_RX_DPSK;
  }
  if ((uint16_t)txCur != V34HS_TONE_AB)
  {
    if (lv > 1u)
    {
      v04PrTxX(txCur, V34HS_TONE_AB, obj->rxState_, obj->mst_, v04K56(obj), v04W78(obj));
      lv = dsplibs_debug_level;
    }
    obj->txState_ = V34HS_TONE_AB;
  }

  /* restart the tone phase with an empty INFO0 receive word */
  obj->toneSel_ = 0;
  obj->tx.k56Idx_ = 0;
  obj->hsSymCnt_ = 0;
  obj->v21RxBuf_ = USHRT_MAX;
  obj->v21BitCnt_ = 0;
  if (lv > 1u)
    dsplibs_debug_printf("V34RETRAIN, SILENCERETRAIN finished, rx->rxflgs,=0x%x,rx->gain=0x%x,gainestimate=0x%x\n", (unsigned)obj->rx.rxFlags_,
                         (int)(int16_t)obj->rx.rxGain_, (int)(int16_t)obj->rx.rxGainEst_);
}

/**
 * @brief Transmit one silent quad for the states SILENCE, SILENCEINFO and SILENCERETRAIN and run the state's timer.
 * @param obj  handshake object; writes the transmit queue, count3_, k56Idx_ and the state words
 * @note  SILENCEINFO ends after 10 symbols by starting TX_DPSK with a freshly built INFO0a queue; SILENCERETRAIN is handled by retrainEnd().
 *        SILENCE itself only writes silence.
 */
extern "C" void v04_tx_silence(tagV34Object* obj)
{
  short q[4] = {0, 0, 0, 0};
  unsigned tx;
  unsigned c;
  tag_bitQ* bq;

  /* the silent sample quad, common to all three states */
  obj->count3_ = 0;
  txwritequeue(&obj->tx, q);

  tx = obj->txState_;
  if ((uint16_t)tx == V34HS_SILENCERETRAIN)
  {
    retrainEnd(obj, tx);
    return;
  }
  if ((uint16_t)tx != V34HS_SILENCEINFO)
    return;

  /* SILENCEINFO: after 10 symbols start TX_DPSK and restart the INFO0a bit queue */
  c = (uint16_t)(obj->tx.k56Idx_ + 1u);
  if ((uint16_t)c == 0xa)
  {
    if (dsplibs_debug_level > 1u)
    {
      obj->tx.k56Idx_ = 0xa;
      v04PrTxX(V34HS_SILENCEINFO, V34HS_TX_DPSK, obj->rxState_, obj->mst_, 0xaL, v04W78(obj));
    }
    obj->tx.k56Idx_ = 0;
    obj->txState_ = V34HS_TX_DPSK;
    obj->bitQ_ = (tag_bitQ*)&obj->info0a;
    V34SetINFO0aBits(obj, (unsigned short*)&obj->info0a);
    bq = obj->bitQ_;
    bq->tail_.crc = USHRT_MAX;
    bq->tail_.pos = 0;
    bq->tail_.widx = 0;
    bq->tail_.pass = 0;
    bq->tail_.total = (obj->modeChar_ == V34ROLE_CALL && obj->pcmMode_ != 0) ? 0x1e : 0x11;
    bq->tail_.chunk = 8;
    bq->tail_.crcOn = 1;
    bq->tail_.nbits = 0x10;
    bq->tail_.restartBits = 0x10;
    bq->tail_.loop = 0;
    bq->tail_.acc = 0xff72u;
    bq->tail_.restartAcc = 0xff72u;
    obj->toneSel_ = 0;
  }
  else
  {
    obj->tx.k56Idx_ = (uint16_t)c;
  }
}
