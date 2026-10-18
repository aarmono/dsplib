/**
 * @file v34_hs_tx_phase1_call.cpp
 * @brief V.34 handshake, calling side, phase 1: the TX_PHASE1_CALL microstate (waiting for the answering modem's INFO0 after the L1 probe).
 *
 * One entry point, v04_t3_tx_phase1_call(), a case of the microstate jump table of the receive branch of v34handshak() (v34_hs_dispatch.cpp).  The
 * dispatcher runs the transmit-state stubs afterwards.  Callee: v34handshakinit().
 *
 * When the INFO0 receive counter changed since the receive head ran (v21BitCnt_ != preDemodWord) and the low three flags of the INFO0 word are all
 * set ((v21RxBuf_ & 7) == 7), the "repeated INFO0" error recovery runs: w3588_ bit 0 set, transmit state TX_DPSK, microstate DET_SYNC, v21RxBuf_
 * re-armed, the INFO0a bit queue restarted for the INFO0 resend and hsSymCnt_ cleared.  Every 0x60 symbols the tone select toneSel_ flips and the
 * microstate becomes RX_PHASE2_CALL (0x3b).  Finally, in the double-marked state (w3588_ == 2 and w358a_ == 2) with a repeated INFO0
 * ((v21RxBuf_ & 0x3ff) == 0x372), the whole handshake restarts (v34handshakinit(obj, V34HSINIT_START)) with the same error recovery.
 *
 * @note The first recovery keeps one copy of the debug level, refreshed only after its first print, and uses it for the V21RXBUF print; the second
 *       recovery refreshes its copy after each print.  The second recovery also sets bitQ_ = &info0a_.  The double-marked test compares the
 *       32-bit value (w358a_ << 16) | w3588_.
 */

#include "v34_handshake.h"
#include "vpcm/vpcm_v34_api.h"

extern "C"
{
}

#include "v34_hs_clear_info_state.h"

#include "v34_hs_rearm_info0a.h"

/**
 * @brief One pass of microstate TX_PHASE1_CALL (calling side, phase 1).
 * @param obj           handshake object; reads v21RxBuf_, v21BitCnt_, w3588_, w358a_, hsSymCnt_, writes the state words and the INFO0a queue
 * @param preDemodWord  value of v21BitCnt_ captured by the receive head BEFORE fskdemodulate() ran (only the low 16 bits are compared)
 * @note  Called once per symbol block by the receive branch of v34handshak().  v34handshakinit() is called in the double-marked recovery.
 */
extern "C" void v04_t3_tx_phase1_call(tagV34Object* obj, unsigned preDemodWord)
{
  unsigned symCnt;

  /* recovery 1: a changed INFO0 word with the three low flags set means the INFO0 is being repeated */
  if (obj->v21BitCnt_ != (uint16_t)preDemodWord && (obj->v21RxBuf_ & 7u) == 7u)
  {
    unsigned lv = dsplibs_debug_level;
    unsigned tx;
    unsigned mst;
    if (lv > 1u)
    {
      dsplibs_debug_printf("Repeated info0 is detected, errorrecovery is initialized in TX_PHASE1_CALL\n");
      lv = dsplibs_debug_level;
    }
    obj->w3588_ = (uint16_t)(obj->w3588_ | 1u);
    if (lv > 1u)
      dsplibs_debug_printf("V21RXBUF=0x%x\n", (unsigned)obj->v21RxBuf_);
    clearInfoState(obj);
    obj->w358a_ = 1;
    tx = obj->txState_;
    if ((uint16_t)tx != V34HS_TX_DPSK)
    {
      if (dsplibs_debug_level > 1u)
        v04PrTx(obj, tx, V34HS_TX_DPSK);
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
    obj->hsSymCnt_ = 0;
  }

  /* every 0x60 symbols flip the tone select and move on to RX_PHASE2_CALL */
  symCnt = (uint16_t)(obj->hsSymCnt_ + 1u);
  if ((int16_t)symCnt <= 0x5f)
  {
    obj->hsSymCnt_ = (uint16_t)symCnt;
  }
  else
  {
    unsigned toneSel = obj->toneSel_ ^ 1u;
    unsigned mst;
    unsigned lv;
    obj->hsSymCnt_ = 0;
    mst = obj->mst_;
    obj->toneSel_ = (uint16_t)toneSel;
    lv = dsplibs_debug_level;
    if ((uint16_t)mst != V34HS_RX_PHASE2_CALL)
    {
      if (lv > 1u)
      {
        v04PrMstX(mst, V34HS_RX_PHASE2_CALL, obj->txState_, obj->rxState_, v04K56(obj), 0L);
        lv = dsplibs_debug_level;
      }
      obj->mst_ = V34HS_RX_PHASE2_CALL;
    }
    if (lv > 1u)
      dsplibs_debug_printf("V21RXBUF=0x%x\n", (unsigned)obj->v21RxBuf_);
  }

  /* recovery 2: in the double-marked state a repeated INFO0 restarts the whole handshake */
  if ((((uint32_t)obj->w358a_ << 16) | obj->w3588_) == 0x20002u && (obj->v21RxBuf_ & 0x3ffu) == 0x372u)
  {
    unsigned lv;
    unsigned tx;
    unsigned mst;
    v34handshakinit(obj, V34HSINIT_START);
    obj->w358a_ = 1;
    obj->w3588_ = (uint16_t)(obj->w3588_ | 1u);
    obj->bitQ_ = (tag_bitQ*)&obj->info0a;
    clearInfoState(obj);
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
      dsplibs_debug_printf("V34 In Retrain. errorrecovery for info0 is initialized in TX_PHASE1_CALL\n");
  }
}
