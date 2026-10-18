/**
 * @file v34_hs_tx_l1.cpp
 * @brief V.34 handshake, receive-branch microstate TX_L1 (end of the L1 probe tone).
 *
 * Microstate handler called from the RX_DPSK branch of v34handshak() (v34_hs_dispatch.cpp); the caller runs the common status tail afterwards.  The
 * handler counts symbols and acts at symbol 0x2a:
 * - normal case (shortPhase2_ == 0): transmit state TX_L1, microstate TX_L1, the tone detector is re-armed (c1200_, or c2400_ for the calling role) and the
 *   receive state becomes DET_AB;
 * - short phase 2 (shortPhase2_ != 0): INFO1a is sent at once.  Microstate INFODONE, transmit state TX_DPSK, the INFO1a bit queue (info) is built by
 *   V34SetINFO1aBits() and re-armed for 0x26 bits; the scaling table of the previous session's baud rate (baud_, table pointer scalePtr_) and its carrier
 *   are taken over into rxBaud_, rxCarrier_ and scaleTab_.  An unusable 2743 baud becomes 2800 baud, carrier 1829 -> 1867, any other carrier -> 1680.
 *
 * Both cases end by reducing the receive gain (above 0x2000: divided by 4, above 0x1000: divided by 2), clearing w35a0_ and printing the "V34RETRAIN"
 * debug line.
 *
 * Structure note: the microstate word `mst` is the caller's copy taken before the table jump; the first transition print and the "!= NEW" guards use it
 * (refreshed after that print).  The debug level is re-read at fixed points, and the join value `lv` is the last copy read in the branch.
 */

#include <limits.h>
#include "v34_handshake.h"
#include "v34/v34_internal.h"
#include "v34/v34_frame_tables.h"  /* c1200_, c2400_ */
#include "v34/v34_symbol_tables.h" /* scale2400 ... scale3429 */

/**
 * @brief Handle one symbol of the TX_L1 microstate.
 * @param obj  V.34 module object (handshake in progress)
 * @param mst  the microstate word the caller loaded before dispatching (stale copy of mst_ used by the first transition print and guards)
 * @note  Does nothing but count until hsSymCnt_ reaches 0x2a.  In the short phase 2 case it resets hsSymCnt_ and k56Idx_, and rewrites the INFO1a queue.
 * @warning The scaling table is chosen by a switch on baud_; a baud_ outside the six handled values leaves scalePtr_ (and so scaleTab_) unchanged.
 */
extern "C" void v04_t3_tx_l1(tagV34Object* obj, unsigned mst)
{
  unsigned symCnt = (uint16_t)(obj->hsSymCnt_ + 1u);
  unsigned lv;
  obj->hsSymCnt_ = (uint16_t)symCnt;
  if ((uint16_t)symCnt != 0x2a)
    return;

  /* normal case: switch to TX_L1, re-arm the tone detector and wait for tone AB */
  if (obj->shortPhase2_ == 0)
  {
    unsigned tx = obj->txState_;
    unsigned rx;
    if ((uint16_t)tx != V34HS_TX_L1)
    {
      if (dsplibs_debug_level > 1u)
      {
        v04PrTxX(tx, V34HS_TX_L1, obj->rxState_, mst, v04K56(obj), 0x2aL);
        mst = obj->mst_;
      }
      obj->txState_ = V34HS_TX_L1;
    }
    if ((uint16_t)mst != V34HS_TX_L1)
    {
      if (dsplibs_debug_level > 1u)
        v04PrMstX(mst, V34HS_TX_L1, obj->txState_, obj->rxState_, v04K56(obj), v04W78(obj));
      obj->mst_ = V34HS_TX_L1;
    }

    obj->tx.k56Idx_ = 0;
    detectorinit(&obj->det, obj->modeChar_ == V34ROLE_CALL ? c2400_ : c1200_, 0, 0x64, 0x32, 0x800, 0);
    obj->rx.rxFlags_ = (uint16_t)(obj->rx.rxFlags_ | 0x200u);
    rx = obj->rxState_;
    lv = dsplibs_debug_level;
    if ((uint16_t)rx != V34HS_DET_AB)
    {
      if (lv > 1u)
      {
        dsplibs_debug_printf(V04_FMT_RX, StateName[(int16_t)rx], StateName[V34HS_DET_AB], StateName[(int16_t)obj->txState_], StateName[(int16_t)obj->mst_],
                             v04K56(obj), v04W78(obj));
        lv = dsplibs_debug_level;
      }
      obj->rxState_ = V34HS_DET_AB;
    }
  }
  else
  {
    /* short phase 2: send INFO1a at once */
    unsigned tx;
    tag_bitQ* q;
    if ((uint16_t)mst != V34HS_INFODONE)
    {
      if (dsplibs_debug_level > 1u)
        v04PrMstX(mst, V34HS_INFODONE, obj->txState_, obj->rxState_, v04K56(obj), 0x2aL);
      obj->mst_ = V34HS_INFODONE;
    }
    tx = obj->txState_;
    if ((uint16_t)tx != V34HS_TX_DPSK)
    {
      if (dsplibs_debug_level > 1u)
        v04PrTx(obj, tx, V34HS_TX_DPSK);
      obj->txState_ = V34HS_TX_DPSK;
    }

    obj->tx.k56Idx_ = 0;
    obj->hsSymCnt_ = 0;
    obj->rxMdLen_ = 0;
    obj->bitQ_ = (tag_bitQ*)&obj->info;
    V34SetINFO1aBits(obj, (uint16_t*)&obj->info);

    /* pick the scaling table for the previous session's baud rate */
    lv = 0;
    switch ((int16_t)obj->rate.baud_)
    {
    case V34_BAUD_2800:
      obj->rate.scalePtr_ = scale2800;
      lv = dsplibs_debug_level;
      break;
    case V34_BAUD_2400:
      obj->rate.scalePtr_ = scale2400;
      lv = dsplibs_debug_level;
      break;
    case V34_BAUD_2743:
      lv = dsplibs_debug_level;
      if (lv > 1u)
      {
        dsplibs_debug_printf("V34HSHAK: Illegal prev session baud (2743), select 2800 instead...\r\n");
        lv = dsplibs_debug_level;
      }
      obj->rate.baud_ = V34_BAUD_2800;
      obj->rate.scalePtr_ = scale2800;
      obj->rate.carrier_ = (uint16_t)(obj->rate.carrier_ == V34_CARRIER_1829 ? V34_CARRIER_1867 : V34_CARRIER_1680);
      break;
    case V34_BAUD_3200:
      obj->rate.scalePtr_ = scale3200;
      lv = dsplibs_debug_level;
      break;
    case V34_BAUD_3000:
      obj->rate.scalePtr_ = scale3000;
      lv = dsplibs_debug_level;
      break;
    case V34_BAUD_3429:
      obj->rate.scalePtr_ = scale3429;
      lv = dsplibs_debug_level;
      break;
    default:
      lv = dsplibs_debug_level;
      break;
    }

    /* re-arm the INFO1a queue (0x26 bits) */
    q = obj->bitQ_;
    q->tail_.crc = USHRT_MAX;
    q->tail_.pos = 0;
    q->tail_.widx = 0;
    q->tail_.pass = 0;
    q->tail_.total = 0x26;
    q->tail_.chunk = 8;
    q->tail_.crcOn = 1;
    q->tail_.nbits = 0x10;
    q->tail_.restartBits = 0x10;
    q->tail_.loop = 0;
    q->tail_.acc = 0xff72u;
    q->tail_.restartAcc = 0xff72u;

    obj->toneSel_ = 0;
    obj->rate.rxBaud_ = obj->rate.baud_;
    obj->rate.rxCarrier_ = obj->rate.carrier_;
    obj->rate.scaleTab_ = obj->rate.scalePtr_;

    /* probe print: the first ten words of the INFO1a record */
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
      dsplibs_debug_printf("%s 0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x\n", "V34PROBE, txinfo1a (QC)", w[0], w[1], w[2], w[3], w[4], w[5], w[6], w[7],
                           w[8], w[9]);
      lv = dsplibs_debug_level;
    }
  }

  /* common tail: reduce a large receive gain */
  {
    int g = (int16_t)obj->rx.rxGain_;
    if (g > 0x2000)
      obj->rx.rxGain_ = (uint16_t)(g >> 2);
    else if (g > 0x1000)
      obj->rx.rxGain_ = (uint16_t)(g >> 1);
  }

  obj->w35a0_ = 0;
  if (lv > 1u)
    dsplibs_debug_printf("V34RETRAIN, %s, rx->rxflgs = 0x%x,rx->gain=0x%x\n", obj->shortPhase2_ != 0 ? "transmitting info1a" : "starting DET_AB",
                         (unsigned)obj->rx.rxFlags_, (int)(int16_t)obj->rx.rxGain_);
}
