/**
 * @file v34_hs_rx_phase1_call.cpp
 * @brief V.34 handshake, receive microstate RX_PHASE1_CALL (calling modem waiting for the answering modem's phase 1 signal).
 *
 * One of the microstate handlers that the receive branch of v34handshak() (v34_hs_dispatch.cpp) calls while rxState_ is RX_DPSK; after it returns the
 * caller runs the common status tail.  The handler is an extern "C" helper working on the V.34 module object (tagV34Object).
 *
 * While the transmit state is TX_DPSK a finished INFO0 queue (loop set, message counter pass non-zero) is closed (loop = 0).  After 0x60 symbols with
 * the INFO0 word v21RxBuf_ equal to 1 the phase is complete: w358a_ = 2, the symbol count restarts at filtDelay_, the microstate becomes
 * TX_PHASE1_CALL and the receive gain is saved in rxGainSaved_.  After 0x3c0 symbols with w3588_ bit 1 and w358a_ == 2, a repeated INFO0
 * (v21RxBuf_ & 0x3ff == 0x372) runs the INFO0 error recovery (v34handshakinit(obj, V34HSINIT_START) when w3588_ is exactly 2).  After 0x4b1 symbols
 * without any INFO0 (w3588_ == 0, v21RxBuf_ != 0) the same recovery state is entered silently (retrain of the INFO exchange).
 *
 * Structure note: the symbol count and the INFO0 flag word are read into locals at fixed points, and some later tests and debug prints use the stale
 * local value on purpose; the first recovery keeps one debug-level copy that is refreshed after each print.  Do not merge these re-reads without
 * checking the debug output.
 */

#include "v34_handshake.h"
#include "vpcm/vpcm_v34_api.h"

extern "C"
{
}

#include "v34_hs_clear_info_state.h"

#include "v34_hs_rearm_info0a.h"

/**
 * @brief Handle one symbol of the RX_PHASE1_CALL microstate.
 * @param obj  V.34 module object (handshake in progress, calling side)
 * @note  Advances hsSymCnt_; may switch the microstate (TX_PHASE1_CALL or DET_SYNC) and the transmit state (TX_DPSK), and re-arms the INFO0a queue
 *        through rearmInfo0a().  Debug prints are emitted only when dsplibs_debug_level > 1.
 * @warning The recovery paths call v34handshakinit() and rewrite the transmit queue; the object must be a live handshake object.
 */
extern "C" void v04_t3_rx_phase1_call(tagV34Object* obj)
{
  unsigned symCnt;
  unsigned symCntNow;

  /* close a finished INFO0 queue while the INFO0 bits are being sent */
  if (obj->txState_ == V34HS_TX_DPSK)
  {
    tag_bitQ* q = obj->bitQ_;
    if (q->tail_.loop != 0 && q->tail_.pass != 0)
      q->tail_.loop = 0;
  }

  /* count this symbol; after 0x60 of them an INFO0 word of 1 completes phase 1 */
  symCnt = (uint16_t)(obj->hsSymCnt_ + 1u);
  obj->hsSymCnt_ = (uint16_t)symCnt;
  if ((int16_t)symCnt > 0x5f)
  {
    obj->rx.rxFlags_ = (uint16_t)(obj->rx.rxFlags_ | 0x200u);
    if (obj->v21RxBuf_ == 1)
    {
      unsigned mst;
      unsigned fd;
      if (dsplibs_debug_level > 1u)
        dsplibs_debug_printf("V34RETRAIN, RX_PHASE1_CALL received, count2=%d,rx->gain=0x%x,filtdelay=%d\n", (int)(int16_t)obj->hsSymCnt_,
                             (int)(int16_t)obj->rx.rxGain_, (int)(int16_t)obj->filtDelay_);

      obj->w358a_ = 2;
      mst = obj->mst_;
      fd = obj->filtDelay_;
      obj->hsSymCnt_ = (uint16_t)fd;
      if ((uint16_t)mst != V34HS_TX_PHASE1_CALL)
      {
        if (dsplibs_debug_level > 1u)
          v04PrMstX(mst, V34HS_TX_PHASE1_CALL, obj->txState_, obj->rxState_, v04K56(obj), (long)(int16_t)fd);
        obj->mst_ = V34HS_TX_PHASE1_CALL;
      }

      /* remember the receive gain for the following phase */
      {
        unsigned fl = obj->rx.rxFlags_;
        unsigned g = obj->rx.rxGain_;
        obj->rx.rxGainSaved_ = (uint16_t)g;
        obj->rx.rxFlags_ = (uint16_t)(fl | 0x200u);
      }
      return;
    }
  }

  /* a repeated INFO0 after 0x3c0 symbols restarts the INFO0 exchange */
  symCntNow = obj->hsSymCnt_;
  if ((int16_t)symCntNow > 0x3bf)
  {
    unsigned errGate = obj->w3588_;
    if ((errGate & 2u) != 0 && obj->w358a_ == 2 && (obj->v21RxBuf_ & 0x3ffu) == 0x372u)
    { /* the repeated-INFO0 recovery */
      unsigned lv;
      unsigned tx;
      unsigned mst;
      if ((uint16_t)errGate == 2)
      {
        v34handshakinit(obj, V34HSINIT_START);
        errGate = obj->w3588_;
        obj->bitQ_ = (tag_bitQ*)&obj->info0a;
      }

      obj->w358a_ = 1;
      obj->w3588_ = (uint16_t)(errGate | 1u);
      clearInfoState(obj);

      /* back to sending INFO0 (TX_DPSK) and waiting for its detection (DET_SYNC) */
      tx = obj->txState_;
      lv = dsplibs_debug_level;
      if ((uint16_t)tx != V34HS_TX_DPSK)
      {
        if (lv > 1u)
        {
          v04PrTx(obj, tx, V34HS_TX_DPSK);
          lv = dsplibs_debug_level;
        }
        obj->txState_ = V34HS_TX_DPSK;
      }
      mst = obj->mst_;
      if ((uint16_t)mst != V34HS_DET_SYNC)
      {
        if (lv > 1u)
        {
          v04PrMstX(mst, V34HS_DET_SYNC, obj->txState_, obj->rxState_, v04K56(obj), v04W78(obj));
          lv = dsplibs_debug_level;
        }
        obj->mst_ = V34HS_DET_SYNC;
      }

      rearmInfo0a(obj);
      if (lv > 1u)
        dsplibs_debug_printf("errorrecovery for info0 is initialized in RX_PHASE1_CALL\n");
      symCntNow = obj->hsSymCnt_;
    }
  }

  /* no INFO0 at all after 0x4b1 symbols: enter the same recovery state silently */
  if ((int16_t)symCntNow <= 0x4b0)
    return;
  if (obj->w3588_ != 0)
    return;
  if (obj->v21RxBuf_ == 0)
    return;
  {
    unsigned tx;
    unsigned mst;
    obj->w3588_ = 4;
    obj->shortPhase2_ = 0;
    obj->wabca_ = 0;
    obj->w358a_ = 1;
    {
      int i = 0;
      do
      {
        obj->wAbae_[i] = 0;
        i = (int16_t)(i + 1);
      } while (i <= 9);
    }
    obj->wAbc2_ = 0;

    tx = obj->txState_;
    if ((uint16_t)tx != V34HS_TX_DPSK)
    {
      if (dsplibs_debug_level > 1u)
        v04PrTxX(tx, V34HS_TX_DPSK, obj->rxState_, obj->mst_, v04K56(obj), (long)(int16_t)symCntNow);
      obj->txState_ = V34HS_TX_DPSK;
    }
    mst = obj->mst_;
    if ((uint16_t)mst != V34HS_DET_SYNC)
    {
      if (dsplibs_debug_level > 1u)
        v04PrMstX(mst, V34HS_DET_SYNC, obj->txState_, obj->rxState_, v04K56(obj), v04W78(obj));
      obj->mst_ = V34HS_DET_SYNC;
    }
    rearmInfo0a(obj);
  }
}
