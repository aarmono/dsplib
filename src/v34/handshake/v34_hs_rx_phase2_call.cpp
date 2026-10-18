/**
 * @file v34_hs_rx_phase2_call.cpp
 * @brief V.34 handshake, receive microstate RX_PHASE2_CALL (calling modem receiving the answering modem's phase 2 signals).
 *
 * Microstate handler called from the RX_DPSK branch of v34handshak() (v34_hs_dispatch.cpp); the caller runs the common status tail afterwards.  `buf` is
 * the sample buffer (receive front end + 0x10c) that the RX_DPSK head prepared for fskdemodulate() and that tone_detect() reads here.
 *
 * Sequence, in the order the handler tests it:
 * - a complete INFO0 flag set (v21RxBuf_ & 7 == 7) runs the "repeated INFO0" error recovery (as in RX_PHASE1_CALL);
 * - at symbol 0x2a the transmit state goes to SILENCE;
 * - after 0x1260 symbols a retrain request is looked for with tone_detect(): w402_ is set once the tone has been absent, and a tone hit then starts a
 *   retrain (rxFlags_ bit 0x40, v34handshakinit(obj, V34HSINIT_RETRAIN));
 * - once the symbol count passes filtDelay_ + 0x5c with the INFO0 word equal to 8 or 0x18, the bulk delay is derived (4 * (count - filtDelay_ - 0x63),
 *   at least 1) and applied with ApplyBulkDelay(), the count restarts at 0x14 and the receiver is set up for 2400 baud / 1800 Hz as in RX_PHASE2_ANS
 *   (rxState_ RX_L1, receive gain halved, "bulkdelay estimation" print);
 * - finally the doubly marked state (w3588_ == 2 and w358a_ == 2) with a repeated INFO0 (v21RxBuf_ & 0x3ff == 0x372) restarts the whole handshake
 *   (v34handshakinit(obj, V34HSINIT_START)).
 *
 * Structure note: the symbol count is kept in a local that is re-read at fixed points (after the SILENCE print, after the retrain-flag store); the 0x5c
 * gate and the bulk-delay arithmetic use that local.  The first recovery keeps one debug-level copy, the second one a copy refreshed after each print.
 */

#include "v34_handshake.h"
#include "v34/v34_internal.h"
#include "vpcm/vpcm_v34_api.h"
#include "v34/v34_init_tables.h" /* hsine1800 */

#include "v34_hs_clear_info_state.h"

#include "v34_hs_rearm_info0a.h"

/**
 * @brief Handle one symbol of the RX_PHASE2_CALL microstate.
 * @param obj  V.34 module object (handshake in progress, calling side)
 * @param buf  sample buffer handed to tone_detect() (the one the RX_DPSK head set up for fskdemodulate())
 * @note  Advances hsSymCnt_; may change the transmit state, the receive state and the microstate, restart the handshake through v34handshakinit(), and
 *        apply a bulk delay.  Debug prints are emitted only when dsplibs_debug_level > 1.
 * @warning The bulk delay is computed in 16-bit arithmetic from the symbol count and is not range-checked; it is stored in bulkDelay_ and passed to
 *          ApplyBulkDelay() as is.
 */
extern "C" void v04_t3_rx_phase2_call(tagV34Object* obj, const short* buf)
{
  unsigned symCntNow;
  obj->hsSymCnt_ = (uint16_t)(obj->hsSymCnt_ + 1u);

  /* a complete INFO0 flag set: first error recovery, back to sending INFO0 */
  if ((obj->v21RxBuf_ & 7u) == 7u)
  { /* the first error recovery */
    unsigned lv = dsplibs_debug_level;
    unsigned tx;
    unsigned mst;
    if (lv > 1u)
    {
      dsplibs_debug_printf("Repeated info0 is detected, errorrecovery is initialized in RX_PHASE2_CALL\n");
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

  /* symbol 0x2a: stop transmitting (SILENCE) */
  symCntNow = obj->hsSymCnt_;
  if ((uint16_t)symCntNow == 0x2a)
  {
    unsigned tx = obj->txState_;
    if ((uint16_t)tx != V34HS_SILENCE)
    {
      if (dsplibs_debug_level > 1u)
      {
        v04PrTxX(tx, V34HS_SILENCE, obj->rxState_, obj->mst_, v04K56(obj), 0x2aL);
        symCntNow = obj->hsSymCnt_;
      }
      obj->txState_ = V34HS_SILENCE;
      goto after_silence;
    }
    goto gate;
  }

after_silence:
  /* after 0x1260 symbols: watch for the tone that requests a retrain */
  if ((int16_t)symCntNow > 0x125f)
  {
    if (obj->rx.w402_ != 0)
    {
      if ((int16_t)tone_detect(&obj->rx, &obj->det, buf, obj->rx.rxEnd_) != 0)
      {
        if (dsplibs_debug_level > 1u)
          dsplibs_debug_printf("V34RETRAIN, retrain is initiated in RX_PHASE2_CALL\n");
        obj->rx.rxFlags_ = (uint16_t)(obj->rx.rxFlags_ | 0x40u);
        v34handshakinit(obj, V34HSINIT_RETRAIN);
        return;
      }
    }
    obj->rx.w402_ = 1;
    symCntNow = obj->hsSymCnt_;
  }

gate:
  /* INFO0 word 8 / 0x18 past filtDelay_ + 0x5c symbols: derive and apply the bulk delay, then set up the 2400 baud receiver */
  {
    unsigned fd = obj->filtDelay_;
    if ((int)(int16_t)symCntNow > (int)(int16_t)fd + 0x5c)
    {
      unsigned w2 = obj->v21RxBuf_;
      if ((uint16_t)w2 == 8 || (uint16_t)w2 == 0x18)
      {
        int e = (int)(symCntNow - fd - 0x63u);
        unsigned lv;
        e = (int16_t)e * 4;
        if ((int16_t)e <= 0)
        {
          obj->bulkDelay_ = 1;
        }
        else
        {
          obj->bulkDelay_ = (uint16_t)e;
        }

        ApplyBulkDelay(obj, (int16_t)obj->bulkDelay_);
        lv = dsplibs_debug_level;
        obj->hsSymCnt_ = 0x14;
        if (lv > 1u)
          dsplibs_debug_printf("V34SetupDemodulator: baudrate %ld, carrier %ld\n", 0x960L, 0x708L);

        obj->rx.w414_ = 0x3e80;
        obj->rx.w38c_ = 4;
        obj->rx.w410_ = 0x1f40;
        obj->rx.w412_ = 0x3e80;
        obj->rx.w422_ = 0x3e80;
        obj->rx.hsineTab_ = hsine1800;
        obj->rx.hsineLen_ = 0x10;
        rxtiminginit(obj);

        /* receive state RX_L1 */
        {
          unsigned rx = obj->rxState_;
          lv = dsplibs_debug_level;
          if ((uint16_t)rx != V34HS_RX_L1)
          {
            if (lv > 1u)
            {
              dsplibs_debug_printf(V04_FMT_RX, StateName[(int16_t)rx], StateName[V34HS_RX_L1], StateName[(int16_t)obj->txState_], StateName[(int16_t)obj->mst_],
                                   v04K56(obj), v04W78(obj));
              lv = dsplibs_debug_level;
            }
            obj->rxState_ = V34HS_RX_L1;
          }
        }

        /* halve the receive gain and clear rxFlags_ bits 0x200 and 0x800 */
        {
          unsigned fl = obj->rx.rxFlags_;
          int g = (int16_t)obj->rx.rxGain_;
          g >>= 1;
          obj->rx.rxGain_ = (uint16_t)g;
          obj->rx.rxFlags_ = (uint16_t)(fl & ~0xa00u);
          if (lv > 1u)
            dsplibs_debug_printf("V34AGC, rx->gain =0x%x,rx->slowcf=%d, at end of bulkdelay estimation\n", g, (int)(int16_t)obj->rx.w39e_);
        }
      }
    }
  }

  /* second error recovery: marked state 2/2 and a repeated INFO0 restart the whole handshake */
  if ((((uint32_t)obj->w358a_ << 16) | obj->w3588_) == 0x20002u && (obj->v21RxBuf_ & 0x3ffu) == 0x372u)
  { /* the second error recovery */
    unsigned lv;
    unsigned tx;
    unsigned mst;
    v34handshakinit(obj, V34HSINIT_START);
    obj->bitQ_ = (tag_bitQ*)&obj->info0a;
    obj->shortPhase2_ = 0;
    obj->w3588_ = (uint16_t)(obj->w3588_ | 1u);
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
      dsplibs_debug_printf("V34 In Retrain. errorrecovery for info0 is initialized in RX_PHASE2_CALL\n");
  }
}
