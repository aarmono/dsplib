/**
 * @file v34_hs_rx_det_sync.cpp
 * @brief V.34 handshake microstate handler DET_SYNC (detect the INFO0 sync / tone AB while the receiver is in RX_DPSK).
 *
 * One extern "C" handler, v04_t3_det_sync(obj, mst, buf), called from the RX_DPSK microstate dispatch of v34handshak() (v34_hs_dispatch.cpp) after V34agc() and
 * fskdemodulate(); the caller performs the transmit-state post processing afterwards.  `mst` is the microstate value the dispatch switched on (a copy that is
 * used by debug prints and guards) and `buf` the fskdemodulate output buffer, passed on to tone_detect().  Callees: tone_detect() and v34handshakinit().
 *
 * Control flow, selected by the low byte of v21RxBuf_ (the INFO0 receive flags), mohE8_ (music-on-hold active) and w358a_ (the error-recovery sub-state):
 * - low byte 0x72: INFO0 sync received (dsRx72): microstate -> DET_INFO, the receive bit queue is re-armed.
 * - mohE8_ == 0 and w358a_ == 0: once v21BitCnt_ > 0x64 (dsToneStart), tone_detect() or a timeout (v21BitCnt_ > 0x384) arms the INFO0a error recovery
 *   (w358a_ = 1, txstate TX_DPSK, microstate DET_SYNC, INFO0a queue re-armed).
 * - w358a_ == 1 (dsState1): when tone_detect() sees the tone and the transmit queue has its flag bit (0x80 of byte 4) set, recovery ends; the INFO0 words saved
 *   in wAbae_ are copied to the INFO0 receive record and the microstate goes to RX_PHASE1_CALL (calling role) or TX_PHASE1_ANS.
 * - w358a_ == 2 (dsState2): timers (w35a0_ counts INFO1 windows) with three exits: restart of the INFO1a transmission, return to silence / retrain, or a
 *   retrain / fall back to tone AB depending on the role.
 * - everything else ends in dsMohTail(), the music-on-hold FRR-NACK signal-drop watcher.
 */

#include <limits.h>
#include "v34_handshake.h"
#include "v34/v34_internal.h"
#include "vpcm/vpcm_v34_api.h"

extern "C"
{
}

/** @brief Debug print of a receive-state transition; the tx and mst words are read at print time. */
static inline void dsPrRx(tagV34Object* obj, unsigned old, int nw)
{
  dsplibs_debug_printf(V04_FMT_RX, StateName[(int16_t)old], StateName[nw], StateName[(int16_t)obj->txState_], StateName[(int16_t)obj->mst_], v04K56(obj),
                       v04W78(obj));
}
/** @brief Debug print of a microstate transition; the tx and rx words are read at print time. */
static inline void dsPrMst(tagV34Object* obj, unsigned old, int nw) { v04PrMstX(old, nw, obj->txState_, obj->rxState_, v04K56(obj), v04W78(obj)); }

/**
 * @brief Music-on-hold watcher: after a NACK to an FRR request, flag the dropped signal.
 * @param obj        V.34 module object
 * @param mohActive  mohE8_ as read by the caller (only its low byte is tested)
 * @note  Sets mohF8_ = 1 when MOH is active, mohF0_ == 1, more than 0x31 windows have elapsed (w35a0_) and no INFO0 bits are pending (v21RxBuf_ == 0).
 */
static void dsMohTail(tagV34Object* obj, unsigned mohActive)
{
  if ((uint8_t)mohActive == 0)
    return;
  if (obj->mohF0_ != 1)
    return;
  if ((int16_t)obj->w35a0_ <= 0x31)
    return;
  if (obj->v21RxBuf_ != 0)
    return;
  if (obj->mohF8_ != 0)
    return;
  if (dsplibs_debug_level > 1u)
    dsplibs_debug_printf("V34Handshake: Detected signal drop on FRR request (after NACK), time to move to phase1...\r\n");
  obj->mohF8_ = 1;
}

/**
 * @brief Common tail of the "INFO0 received" paths: clear flag 0x200 of rxFlags_ and print the received INFO0 record.
 * @param obj  V.34 module object
 * @param lv   debug level as held by the caller
 */
static void dsInfo0Tail(tagV34Object* obj, unsigned lv)
{
  tag_bitQ* q;
  obj->rx.rxFlags_ = (uint16_t)(obj->rx.rxFlags_ & 0xfffffdffu);
  if (lv > 1u)
  {
    dsplibs_debug_printf("V34INFO, info0 received in DET_SYNC\n");
    lv = dsplibs_debug_level;
  }
  q = obj->bitQ2_;
  if (lv > 1u)
  {
    unsigned w[10];
    int i;
    for (i = 0; i < 10; i++)
    {
      uint16_t v;
      __builtin_memcpy(&v, q->unk00_ + 2 * i, 2);
      w[i] = v;
    }
    dsplibs_debug_printf("%s 0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x\n", "V34INFO, rxinfo0", w[0], w[1], w[2], w[3], w[4], w[5], w[6], w[7], w[8],
                         w[9]);
  }
}

/**
 * @brief The INFO0 sync byte (0x72) was received: go to microstate DET_INFO and re-arm the receive bit queue.
 * @param obj  V.34 module object
 * @param mst  microstate value the dispatch switched on
 */
static void dsRx72(tagV34Object* obj, unsigned mst)
{
  unsigned lv = dsplibs_debug_level;
  tag_bitQ* q;
  if ((uint16_t)mst != V34HS_DET_INFO)
  {
    if (lv > 1u)
    {
      dsPrMst(obj, mst, V34HS_DET_INFO);
      lv = dsplibs_debug_level;
    }
    obj->mst_ = V34HS_DET_INFO;
  }
  q = obj->bitQ2_;
  q->tail_.crc = USHRT_MAX;
  obj->v21BitCnt_ = 0;
  if (obj->w358a_ == 0)
    obj->rx.rxFlags_ = (uint16_t)(obj->rx.rxFlags_ & 0xfffffdffu);
  obj->hsSymCnt_ = 0;
  if (lv > 1u)
    dsplibs_debug_printf("DET_SYNC is received in RX_DPSK,rx->gain=0x%x\n", (int)(int16_t)obj->rx.rxGain_);
}

/**
 * @brief Start of the INFO0a error recovery after the first 0x64 receive bits.
 * @param obj  V.34 module object
 * @param buf  fskdemodulate output buffer for tone_detect()
 * @note  Returns without effect if no tone is detected and v21BitCnt_ <= 0x384.  Otherwise it enters recovery sub-state 1, forces txstate TX_DPSK and
 *        microstate DET_SYNC and re-arms the INFO0a queue (0x1e bits for the calling role in PCM mode, 0x11 bits otherwise).
 */
static void dsToneStart(tagV34Object* obj, const short* buf)
{
  unsigned txCur, lv, mstCur;
  tag_bitQ* q;
  int i;
  obj->rx.rxFlags_ = (uint16_t)(obj->rx.rxFlags_ & 0xfffffdffu);
  if ((int16_t)tone_detect(&obj->rx, &obj->det, buf, obj->rx.rxEnd_) == 0)
  {
    if ((int16_t)obj->v21BitCnt_ <= 0x384)
      return;
  }

  /* enter recovery sub-state 1 */
  obj->shortPhase2_ = 0;
  obj->wabca_ = 0;
  obj->w358a_ = 1;
  obj->w3588_ = (uint16_t)(obj->w3588_ | 1u);
  for (i = 0; i < 10; i++)
    obj->wAbae_[i] = 0;
  txCur = obj->txState_;
  obj->wAbc2_ = 0;
  lv = dsplibs_debug_level;

  if ((uint16_t)txCur != V34HS_TX_DPSK)
  {
    if (lv > 1u)
    {
      v04PrTx(obj, txCur, V34HS_TX_DPSK);
      lv = dsplibs_debug_level;
    }
    obj->txState_ = V34HS_TX_DPSK;
  }
  mstCur = obj->mst_;
  if ((uint16_t)mstCur != V34HS_DET_SYNC)
  {
    if (lv > 1u)
    {
      dsPrMst(obj, mstCur, V34HS_DET_SYNC);
      lv = dsplibs_debug_level;
    }
    obj->mst_ = V34HS_DET_SYNC;
  }

  obj->v21RxBuf_ = USHRT_MAX;
  obj->v21BitCnt_ = 0;

  /* re-arm the INFO0a bit queue */
  q = (tag_bitQ*)&obj->info0a;
  if (obj->modeChar_ == V34ROLE_CALL && obj->pcmMode_ != 0)
  {
    q->tail_.crc = USHRT_MAX;
    q->tail_.pos = 0;
    q->tail_.widx = 0;
    q->tail_.pass = 0;
    q->tail_.total = 0x1e;
  }
  else
  {
    q->tail_.crc = USHRT_MAX;
    q->tail_.pos = 0;
    q->tail_.widx = 0;
    q->tail_.pass = 0;
    q->tail_.total = 0x11;
  }

  q->tail_.chunk = 8;
  q->tail_.crcOn = 1;
  q->tail_.acc = 0xf72u;
  q->tail_.restartAcc = 0xf72u;
  q->tail_.nbits = 0xc;
  q->tail_.restartBits = 0xc;
  q->tail_.loop = 1;
  if (lv > 1u)
    dsplibs_debug_printf("errorrecovery is initialized in DET_SYNC\n");
}

/**
 * @brief Recovery sub-state 1: wait for tone AB with the transmit flag set, then finish the INFO0a recovery.
 * @param obj  V.34 module object
 * @param buf  fskdemodulate output buffer for tone_detect()
 * @note  Without the tone or the flag bit the call only runs the MOH watcher.  On success the saved INFO0 words (wAbae_[0..wAbc2_]) are copied to the
 *        receive bit queue and the microstate becomes RX_PHASE1_CALL (calling role) or TX_PHASE1_ANS.
 */
static void dsState1(tagV34Object* obj, const short* buf)
{
  tag_bitQ* q;
  unsigned lv, mstCur;
  if ((int16_t)tone_detect(&obj->rx, &obj->det, buf, obj->rx.rxEnd_) == 0)
  {
    dsMohTail(obj, obj->mohE8_);
    return;
  }
  q = obj->bitQ_;
  if ((q->unk00_[4] & 0x80u) == 0)
  {
    dsMohTail(obj, obj->mohE8_);
    return;
  }

  /* tone AB seen: recovery is over */
  obj->w358a_ = 0;
  if ((int16_t)obj->wAbc2_ >= 0)
  {
    tag_bitQ* dst = obj->bitQ2_;
    int16_t i = 0;
    do
    {
      uint16_t v = obj->wAbae_[i];
      __builtin_memcpy(dst->unk00_ + 2 * i, &v, 2);
      i = (int16_t)(i + 1);
    } while ((int16_t)obj->wAbc2_ >= i);
  }
  obj->hsSymCnt_ = 0;

  if (obj->modeChar_ == V34ROLE_CALL)
  {
    mstCur = obj->mst_;
    lv = dsplibs_debug_level;
    if ((uint16_t)mstCur != V34HS_RX_PHASE1_CALL)
    {
      if (lv > 1u)
      {
        v04PrMstX(mstCur, V34HS_RX_PHASE1_CALL, obj->txState_, obj->rxState_, v04K56(obj), 0);
        lv = dsplibs_debug_level;
      }
      obj->mst_ = V34HS_RX_PHASE1_CALL;
    }
    if (lv > 1u)
    {
      dsplibs_debug_printf("Tone AB detected ending errorrecovery. Switch to RX_PHASE1_CALL.\n");
      lv = dsplibs_debug_level;
    }
  }
  else
  {
    mstCur = obj->mst_;
    obj->v21RxBuf_ = USHRT_MAX;
    lv = dsplibs_debug_level;
    if ((uint16_t)mstCur != V34HS_TX_PHASE1_ANS)
    {
      if (lv > 1u)
      {
        v04PrMstX(mstCur, V34HS_TX_PHASE1_ANS, obj->txState_, obj->rxState_, v04K56(obj), 0);
        lv = dsplibs_debug_level;
      }
      obj->mst_ = V34HS_TX_PHASE1_ANS;
    }
    if (lv > 1u)
    {
      dsplibs_debug_printf("Tone AB detected ending errorrecovery. Switch to TX_PHASE1_ANS.\n");
      lv = dsplibs_debug_level;
    }
  }

  dsInfo0Tail(obj, lv);
}

/**
 * @brief Recovery sub-state 2: INFO0 window timers and the exits from the recovery.
 * @param obj         V.34 module object
 * @param mst         microstate value the dispatch switched on
 * @param info0Flags  v21RxBuf_ as read by the caller
 * @param mohActive   mohE8_ as read by the caller
 * @note  The window thresholds are relative to bulkDelay_ / 16: below +0x64 nothing happens (MOH watcher only), at +0x190 and +0x1ea the exits are tested.
 *        Exits: (a) w35a0_ > 0x31 with v21RxBuf_ == 0xffff: restart the INFO1a transmission (txstate TX_DPSK, microstate INFODONE);
 *        (b) w35a0_ > 0x31 with v21RxBuf_ == 0: txstate SILENCERETRAIN, rxstate WAIT; (c) at +0x1ea the answering role retrains (v34handshakinit) if
 *        the word at unka24a_ is 1, the calling role falls back to microstate RX_PHASE1_CALL with txstate TONE_AB.
 */
static void dsState2(tagV34Object* obj, unsigned mst, unsigned info0Flags, unsigned mohActive)
{
  unsigned w7e = obj->bulkDelay_;
  unsigned microWord = obj->v21BitCnt_;
  int bulkSixteenth = (int)(int16_t)w7e >> 4;
  int microWordS = (int16_t)microWord;
  unsigned txCur, step, lv;

  if (microWordS <= bulkSixteenth + 0x64)
  {
    dsMohTail(obj, mohActive);
    return;
  }

  /* count the INFO1 windows in which no INFO0 bits were received */
  if (!(microWordS > bulkSixteenth + 0x190))
    obj->w35a0_ = 0;
  step = 0;
  if ((uint16_t)info0Flags == 0 || (uint16_t)info0Flags == 0xffffu)
    step = (unsigned)obj->w35a0_ + 1u;
  txCur = obj->txState_;
  if ((uint16_t)txCur == V34HS_TONE_AB)
    obj->w35a0_ = 0;
  else
    obj->w35a0_ = (uint16_t)step;
  if ((int)(int16_t)microWord < (((int)(int16_t)w7e >> 4) + 0x190))
    return;

  if ((int16_t)obj->w35a0_ > 0x31)
  {
    if ((uint16_t)info0Flags == 0xffffu)
    {
      /* exit (a): restart the INFO1a transmission */
      tag_bitQ* q;
      unsigned mstCur;
      lv = dsplibs_debug_level;
      if ((uint16_t)txCur != V34HS_TX_DPSK)
      {
        if (lv > 1u)
        {
          v04PrTxX(txCur, V34HS_TX_DPSK, obj->rxState_, mst, v04K56(obj), v04W78(obj));
          lv = dsplibs_debug_level;
        }
        obj->txState_ = V34HS_TX_DPSK;
      }

      q = (tag_bitQ*)&obj->info;
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
      mstCur = obj->mst_;
      obj->bitQ_ = q;
      q->tail_.acc = 0xff72u;
      q->tail_.restartAcc = 0xff72u;
      obj->toneSel_ = 0;

      if ((uint16_t)mstCur != V34HS_INFODONE)
      {
        if (lv > 1u)
        {
          dsPrMst(obj, mstCur, V34HS_INFODONE);
          lv = dsplibs_debug_level;
        }
        obj->mst_ = V34HS_INFODONE;
      }
      if (lv > 1u)
        dsplibs_debug_printf("V34PHASE2,detected INFOMARKS, reinitializing Info1c\n");
      return;
    }
    if ((uint16_t)info0Flags == 0)
    {
      /* exit (b): tone seen during INFO1 but no INFO0 sync; go back to silence and wait */
      unsigned rx;
      obj->w3588_ = (uint16_t)(obj->w3588_ | 2u);
      lv = dsplibs_debug_level;

      if ((uint16_t)txCur != V34HS_SILENCERETRAIN)
      {
        if (lv > 1u)
        {
          v04PrTxX(txCur, V34HS_SILENCERETRAIN, obj->rxState_, mst, v04K56(obj), v04W78(obj));
          lv = dsplibs_debug_level;
        }
        obj->txState_ = V34HS_SILENCERETRAIN;
      }
      rx = obj->rxState_;
      if ((uint16_t)rx != V34HS_WAIT)
      {
        if (lv > 1u)
        {
          dsPrRx(obj, rx, V34HS_WAIT);
          lv = dsplibs_debug_level;
        }
        obj->rxState_ = V34HS_WAIT;
      }

      obj->hsSymCnt_ = 0;
      obj->tx.k56Idx_ = 0;
      obj->toneSel_ = 0;
      obj->v21RxBuf_ = USHRT_MAX;
      obj->v21BitCnt_ = 0;
      if (lv > 1u)
        dsplibs_debug_printf("DET_SYNC not detected, but Tone is detected during info1\n");
      return;
    }
  }

  /* exit (c): no sync found within the INFO1 search window */
  if ((int)(int16_t)microWord < (((int)(int16_t)w7e >> 4) + 0x1ea))
    return;
  if (obj->modeChar_ != V34ROLE_CALL)
  {
    uint16_t w;
    __builtin_memcpy(&w, obj->unka24a_, 2);
    if (w != 1)
      return;
    if (dsplibs_debug_level > 1u)
      dsplibs_debug_printf("DET_SYNC not detected, during search for info1c initiating a retrain\n");
    v34handshakinit(obj, V34HSINIT_RETRAIN);
    return;
  }

  /* calling role: give up on INFO1a and resume tone AB */
  obj->w3588_ = (uint16_t)(obj->w3588_ | 2u);
  lv = dsplibs_debug_level;
  if ((uint16_t)mst != V34HS_RX_PHASE1_CALL)
  {
    if (lv > 1u)
    {
      v04PrMstX(mst, V34HS_RX_PHASE1_CALL, txCur, obj->rxState_, v04K56(obj), v04W78(obj));
      txCur = obj->txState_;
      lv = dsplibs_debug_level;
    }
    obj->mst_ = V34HS_RX_PHASE1_CALL;
  }

  {
    unsigned rx = obj->rxState_;
    if ((uint16_t)rx != V34HS_RX_DPSK)
    {
      if (lv > 1u)
      {
        dsplibs_debug_printf(V04_FMT_RX, StateName[(int16_t)rx], StateName[V34HS_RX_DPSK], StateName[(int16_t)txCur], StateName[(int16_t)obj->mst_],
                             v04K56(obj), v04W78(obj));
        txCur = obj->txState_;
        lv = dsplibs_debug_level;
      }
      obj->rxState_ = V34HS_RX_DPSK;
    }
  }

  if ((uint16_t)txCur != V34HS_TONE_AB)
  {
    if (lv > 1u)
    {
      v04PrTx(obj, txCur, V34HS_TONE_AB);
      lv = dsplibs_debug_level;
    }
    obj->txState_ = V34HS_TONE_AB;
  }

  obj->toneSel_ = 0;
  obj->tx.k56Idx_ = 0;
  obj->hsSymCnt_ = 0;
  obj->v21RxBuf_ = USHRT_MAX;
  obj->v21BitCnt_ = 0;
  if (lv > 1u)
    dsplibs_debug_printf("DET_SYNC not detected, during search for info1a\n");
}

/**
 * @brief Microstate handler for DET_SYNC: dispatch on the INFO0 flags, MOH state and error-recovery sub-state.
 * @param obj  V.34 module object
 * @param mst  microstate value the dispatch switched on (passed through to the sub-handlers that compare against it)
 * @param buf  fskdemodulate output buffer for tone_detect()
 * @note  Clears count3_ on every call.
 */
extern "C" void v04_t3_det_sync(tagV34Object* obj, unsigned mst, const short* buf)
{
  unsigned info0Flags = obj->v21RxBuf_;
  unsigned mohActive, recovGate;
  obj->count3_ = 0;

  if ((uint8_t)info0Flags == 0x72)
  {
    dsRx72(obj, mst);
    return;
  }

  mohActive = obj->mohE8_;
  recovGate = obj->w358a_;
  if (mohActive == 0 && recovGate == 0)
  {
    if ((int16_t)obj->v21BitCnt_ > 0x64)
      dsToneStart(obj, buf);
    return;
  }
  if ((uint16_t)recovGate == 2)
  {
    dsState2(obj, mst, info0Flags, mohActive);
    return;
  }
  if ((uint16_t)(recovGate - 1u) == 0)
  {
    dsState1(obj, buf);
    return;
  }
  dsMohTail(obj, mohActive);
}
