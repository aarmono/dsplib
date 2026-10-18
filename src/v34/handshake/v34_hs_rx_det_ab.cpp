/**
 * @file v34_hs_rx_det_ab.cpp
 * @brief V.34 handshake receive-state handler DET_AB (wait for the end of the L2 signal).
 *
 * One extern "C" handler, v04_rx_det_ab(), called from the receive chain of v34handshak() (v34_hs_dispatch.cpp) when the receive state is DET_AB; the caller
 * performs the transmit-state post processing afterwards.  The handler calls V34agc(), tone_detect(), v34handshakinit(), probeselect() and V34SetINFO1aBits().
 *
 * While the transmitter is in microstate TX_L2 and more than 0x5db symbols have elapsed (k56Idx_), tone_detect() decides whether the far end's tone is
 * still present.  No tone: once k56Idx_ exceeds bulkDelay_ + 0x2418 the handshake is restarted with v34handshakinit(obj, V34HSINIT_RETRAIN).  Tone present
 * (end of L2): the calling role starts sending INFO1a (txstate TX_DPSK, probeselect(), INFO1a queue of 0x4d bits, rxstate RX_DPSK, microstate
 * INFODONE, the INFO1c queue set to 0x26 bits); the answering role goes to txstate TONE_AB, microstate TX_PHASE3_ANS and rxstate RX_DPSK.  Both roles then
 * restore the receive gain from rxGainSaved_ and clear the V.21 receive words and the symbol counters.
 */

#include <limits.h>
#include "v34_handshake.h"
#include "v34/v34_internal.h"
#include "vpcm/vpcm_v34_api.h"

extern "C"
{
}

/**
 * @brief Receive-state handler for DET_AB: detect the end of L2 and move both directions on to the next handshake phase.
 * @param obj  V.34 module object; rxState_ is DET_AB
 * @note  Does nothing beyond the AGC unless mst_ == TX_L2 and k56Idx_ > 0x5db.  The role-specific branch tests modeChar_ == V34ROLE_CALL.
 *        The state-transition debug prints read the other two state words at print time, and the call branch keeps one copy of the debug level that is
 *        re-read after each print.  The "txinfo1c" probe print lists the first ten words of the INFO1a record (not INFO1c).
 */
extern "C" void v04_rx_det_ab(tagV34Object* obj)
{
  V34agc(&obj->rx);
  if (obj->mst_ != V34HS_TX_L2)
    return;
  if ((int16_t)obj->tx.k56Idx_ <= 0x5db)
    return;

  /* L2 has run long enough: is the far end's tone still there? */
  if ((int16_t)tone_detect(&obj->rx, &obj->det, (const short*)obj->rx.rxFskBuf_, obj->rx.rxEnd_) == 0)
  {
    if ((int)(int16_t)obj->tx.k56Idx_ > (int)(int16_t)obj->bulkDelay_ + 0x2418)
    {
      v34handshakinit(obj, V34HSINIT_RETRAIN);
      if (dsplibs_debug_level > 1u)
        dsplibs_debug_printf("V34RETRAIN, retrain is initiated in DET_AB\n");
    }
    return;
  }

  /* end of L2 */
  if (dsplibs_debug_level > 1u)
    dsplibs_debug_printf("V34RETRAIN, End of L2,rx->gain=0x%x,count1=%d\n", (int)(int16_t)obj->rx.rxGain_, (int)(int16_t)obj->tx.k56Idx_);

  if (obj->modeChar_ == V34ROLE_CALL)
  {
    /* calling role: send INFO1a, expect INFO1c */
    unsigned lv;
    unsigned rx;
    tag_bitQ* q;
    v04SetTx(obj, obj->txState_, V34HS_TX_DPSK);
    probeselect(obj);
    obj->bitQ_ = (tag_bitQ*)&obj->info;
    V34SetINFO1aBits(obj, (uint16_t*)&obj->info);
    q = obj->bitQ_;

    /* re-arm the INFO1a bit queue: 0x4d bits */
    q->tail_.crc = USHRT_MAX;
    q->tail_.pos = 0;
    q->tail_.widx = 0;
    q->tail_.pass = 0;
    q->tail_.total = 0x4d;
    q->tail_.chunk = 8;
    q->tail_.crcOn = 1;
    q->tail_.nbits = 0x10;
    q->tail_.restartBits = 0x10;
    q->tail_.loop = 0;
    rx = obj->rxState_;
    q->tail_.acc = 0xff72u;
    q->tail_.restartAcc = 0xff72u;
    obj->toneSel_ = 0;
    lv = dsplibs_debug_level;

    if ((uint16_t)rx != V34HS_RX_DPSK)
    {
      if (lv > 1u)
      {
        dsplibs_debug_printf(V04_FMT_RX, StateName[(int16_t)rx], StateName[V34HS_RX_DPSK], StateName[(int16_t)obj->txState_], StateName[(int16_t)obj->mst_],
                             v04K56(obj), v04W78(obj));
        lv = dsplibs_debug_level;
      }
      obj->rxState_ = V34HS_RX_DPSK;
    }

    {
      unsigned mst = obj->mst_;
      if ((uint16_t)mst != V34HS_INFODONE)
      {
        if (lv > 1u)
        {
          v04PrMstX(mst, V34HS_INFODONE, obj->txState_, obj->rxState_, v04K56(obj), v04W78(obj));
          lv = dsplibs_debug_level;
        }
        obj->mst_ = V34HS_INFODONE;
      }
    }

    obj->bitQ2_ = (tag_bitQ*)&obj->info1c;
    obj->info1c.i1cTail_.total = 0x26;
    if (lv > 1u)
    {
      unsigned w[10];
      int i;
      for (i = 0; i < 10; i++)
      {
        uint16_t v;
        __builtin_memcpy(&v, ((uint8_t*)&obj->info) + 2 * i, 2);
        w[i] = v;
      }
      dsplibs_debug_printf("%s 0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x\n", "V34PROBE, txinfo1c", w[0], w[1], w[2], w[3], w[4], w[5], w[6], w[7], w[8],
                           w[9]);
    }
  }
  else
  {
    /* answering role */
    v04SetTx(obj, obj->txState_, V34HS_TONE_AB);
    v04SetMst(obj, obj->mst_, V34HS_TX_PHASE3_ANS);
    v04SetRx(obj, obj->rxState_, V34HS_RX_DPSK);
  }

  /* both roles: restore the receive gain and clear the counters */
  {
    unsigned fl = obj->rx.rxFlags_;
    unsigned g = obj->rx.rxGainSaved_;
    obj->rx.rxFlags_ = (uint16_t)(fl | 0x200u);
    obj->rx.rxGain_ = (uint16_t)g;
  }
  obj->v21RxBuf_ = 0;
  obj->v21BitCnt_ = 0;
  obj->fskCounter_ = 0;
  obj->fskNextPoint_ = 6;
  obj->tx.k56Idx_ = 0;
  obj->hsSymCnt_ = 0;
}
