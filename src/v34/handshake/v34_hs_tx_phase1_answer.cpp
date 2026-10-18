/**
 * @file v34_hs_tx_phase1_answer.cpp
 * @brief V.34 handshake, answering side, phase 1: the TX_PHASE1_ANS microstate (sending INFO0, waiting for the caller's INFO0).
 *
 * Holds one entry point, v04_t3_tx_phase1_ans(), one case of the microstate jump table of the receive branch of v34handshak() (v34_hs_dispatch.cpp);
 * the dispatcher runs the transmit-state stubs afterwards.
 *
 *   microstate 0x2e TX_PHASE1_ANS: v04_t3_tx_phase1_ans(); callees V34SetINFO0aBits(), V34SetINFO0dBits(), clearInfoState()
 *
 * While the transmit state is TX_DPSK, a finished INFO0 queue (loop set, message counter pass non-zero) restarts the symbol count and is closed.
 * In transmit state TONE_AB the count runs to 0x190 and then either rebuilds the INFO0 words for a retrain (b_ac00_ set: bit queues info0a /
 * info0d, w3588_ |= 4, the error-recovery state) or flips the tone select and moves the microstate to RX_PHASE1_ANS (0x31) with w358a_ = 2.
 * Three recoveries from a repeated INFO0 all end in transmit state TX_DPSK, microstate DET_SYNC, v21RxBuf_ re-armed and the INFO0a queue restarted
 * (recoveryTail()); they differ in their gates and debug messages:
 * - R1: after 0xc8 symbols with (v21RxBuf_ & 0x3ff) == 0x372,
 * - R2: after 0x4b1 symbols with any INFO0 word received,
 * - R3: with w3588_ == 2, after 0xc8 symbols and (v21RxBuf_ & 0xfff) == 0xf72, the 13th time (count3_ counts them).
 *
 * @note The transmit state copy `txCur` and the symbol count copy are loaded once and not reloaded: R1 compares and prints the stale transmit
 *       state, and R1 / R2 print the stale symbol count as the [2] argument of the transmit-state debug line.  The other prints read the object
 *       fresh.  After a recovery the function re-reads w3588_ and re-runs the `== 2` / `== 0` tests.
 */

#include <limits.h>
#include "v34_handshake.h"
#include "v34/v34_internal.h"

extern "C"
{
}

#include "v34_hs_clear_info_state.h"

/**
 * @brief Common tail of the INFO0 recoveries: transmit state TX_DPSK, microstate DET_SYNC, INFO0a queue re-armed.
 * @param obj    handshake object
 * @param txCur  transmit state as the caller last read it (compared and printed stale)
 * @param fresh2 true to print the current symbol count as the [2] debug argument, false to print `arg2`
 * @param arg2   [2] debug argument used when `fresh2` is false
 * @return the debug level read after the last state change (the caller prints its recovery message when it is above 1)
 */
static unsigned recoveryTail(tagV34Object* obj, unsigned txCur, bool fresh2, long arg2)
{
  unsigned lv = dsplibs_debug_level;
  unsigned mst;
  tag_bitQ* q;

  /* force the transmit state to TX_DPSK, then the microstate to DET_SYNC, printing each transition */
  if ((uint16_t)txCur != V34HS_TX_DPSK)
  {
    if (lv > 1u)
    {
      v04PrTxX(txCur, V34HS_TX_DPSK, obj->rxState_, obj->mst_, v04K56(obj), fresh2 ? v04W78(obj) : arg2);
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

  /* re-arm the INFO0 receive shift register and restart the INFO0a transmit queue */
  obj->v21RxBuf_ = USHRT_MAX;
  obj->v21BitCnt_ = 0;
  q = (tag_bitQ*)&obj->info0a;
  q->tail_.crc = USHRT_MAX;
  q->tail_.pos = 0;
  q->tail_.widx = 0;
  q->tail_.pass = 0;
  q->tail_.total = (uint16_t)((obj->modeChar_ == V34ROLE_CALL && obj->pcmMode_ != 0) ? 0x1e : 0x11);
  q->tail_.chunk = 8;
  q->tail_.crcOn = 1;
  q->tail_.acc = 0xf72u;
  q->tail_.restartAcc = 0xf72u;
  q->tail_.nbits = 0xc;
  q->tail_.restartBits = 0xc;
  q->tail_.loop = 1;
  return lv;
}

/**
 * @brief One pass of microstate TX_PHASE1_ANS (answering side, phase 1).
 * @param obj  handshake object; reads txState_, mst_, hsSymCnt_, v21RxBuf_, w3588_, count3_ and the INFO0 queues, writes the recovery state
 * @note  Called once per symbol block by the receive branch of v34handshak(); the transmit-state stubs run after it.
 */
extern "C" void v04_t3_tx_phase1_ans(tagV34Object* obj)
{
  unsigned txCur = obj->txState_;
  unsigned symCntNow = 0;
  unsigned errGate;

  /* a finished INFO0 transmit queue restarts the symbol count */
  if ((uint16_t)txCur == V34HS_TX_DPSK)
  {
    tag_bitQ* q = obj->bitQ_;
    if (q->tail_.loop != 0 && q->tail_.pass != 0)
    {
      obj->hsSymCnt_ = 0;
      q->tail_.loop = 0;
      txCur = obj->txState_;
    }
  }

  /* tone phase: count symbols; at the limit either rebuild INFO0 (retrain) or flip the tone and wait for the caller's INFO0 */
  if ((uint16_t)txCur == V34HS_TONE_AB)
  {
    unsigned symCntNext = (uint16_t)(obj->hsSymCnt_ + 1u);
    if ((int16_t)symCntNext <= 0x18f || obj->v21RxBuf_ != 0)
    {
      obj->hsSymCnt_ = (uint16_t)symCntNext;
    }
    else if (obj->b_ac00_ != 0)
    { /* rebuild the INFO0 words for a retrain */
      unsigned lv;
      obj->hsSymCnt_ = (uint16_t)symCntNext;
      obj->bitQ_ = (tag_bitQ*)&obj->info0a;
      obj->bitQ2_ = (tag_bitQ*)&obj->info0d;
      V34SetINFO0aBits(obj, (unsigned short*)&obj->info0a);
      if (obj->modeChar_ == V34ROLE_ANSWER)
        V34SetINFO0dBits(obj, obj->bitQ2_);
      obj->shortPhase2_ = 0;
      obj->wabca_ = 0;
      obj->w358a_ = 1;
      obj->w3588_ = (uint16_t)(obj->w3588_ | 4u);
      {
        int i = 0;
        do
        {
          obj->wAbae_[i] = 0;
          i = (int16_t)(i + 1);
        } while (i <= 9);
      }
      obj->wAbc2_ = 0;
      lv = recoveryTail(obj, obj->txState_, true, 0);
      if (lv > 1u)
        dsplibs_debug_printf("Repeated info0 is initialized on retrain\n");
      obj->b_ac00_ = 0;
      return;
    }
    else
    {
      unsigned mst;
      obj->hsSymCnt_ = 0;
      obj->toneSel_ = (uint16_t)(obj->toneSel_ ^ 1u);
      mst = obj->mst_;
      if ((uint16_t)mst != V34HS_RX_PHASE1_ANS)
      {
        if (dsplibs_debug_level > 1u)
          v04PrMstX(mst, V34HS_RX_PHASE1_ANS, V34HS_TONE_AB, obj->rxState_, v04K56(obj), 0L);
        obj->mst_ = V34HS_RX_PHASE1_ANS;
      }
      obj->w358a_ = 2;
      return;
    }
  }

  /* error-recovery gate: w3588_ == 0 arms R1 and R2, w3588_ == 2 arms R3 */
  errGate = obj->w3588_;
  if ((uint16_t)errGate == 0)
  {
    symCntNow = obj->hsSymCnt_;
    if ((int16_t)symCntNow > 0xc7 && (obj->v21RxBuf_ & 0x3ffu) == 0x372u)
    { /* R1: repeated INFO0 seen early */
      unsigned lv;
      obj->wabca_ = 0;
      obj->w3588_ = 4;
      clearInfoState(obj);
      obj->w358a_ = 1;
      lv = recoveryTail(obj, txCur, false, (long)(int16_t)symCntNow);
      if (lv > 1u)
        dsplibs_debug_printf("Repeated info0 is detected, errorrecovery is initialized in TX_PHASE1_ANS\n");
      errGate = obj->w3588_;
      goto nonzero;
    }
  count_gate:
    symCntNow = obj->hsSymCnt_;
    if ((int16_t)symCntNow <= 0x4b0)
      return;
    if (obj->v21RxBuf_ == 0)
      return;
    { /* R2: repeated INFO0 seen after 1200 symbols */
      unsigned lv;
      obj->w358a_ = 1;
      obj->w3588_ = 4;
      clearInfoState(obj);
      lv = recoveryTail(obj, obj->txState_, false, (long)(int16_t)symCntNow);
      if (lv > 1u)
        dsplibs_debug_printf("Repeated info0 is detected (after 1200), errorrecovery is initialized in TX_PHASE1_ANS\n");
      return;
    }
  }
nonzero:
  if ((uint16_t)errGate == 2)
  {
    unsigned count3;
    if ((int16_t)obj->hsSymCnt_ <= 0xc7)
      return;
    if ((obj->v21RxBuf_ & 0xfffu) != 0xf72u)
      return;
    if (dsplibs_debug_level > 1u)
      dsplibs_debug_printf("TX_PHASE_ANS: count3 = %d \n", (int)(int16_t)obj->count3_);
    count3 = (uint16_t)(obj->count3_ + 1u);
    if ((int16_t)count3 <= 0xc)
    {
      obj->count3_ = (uint16_t)count3;
    }
    else
    { /* R3: INFO0 detected 13 times */
      unsigned lv;
      obj->count3_ = 0;
      obj->w3588_ = 4;
      obj->bitQ_ = (tag_bitQ*)&obj->info0a;
      clearInfoState(obj);
      obj->w358a_ = 1;
      lv = recoveryTail(obj, obj->txState_, true, 0);
      if (lv > 1u)
        dsplibs_debug_printf("info0 is detected, info0 is initialized in TX_PHASE1_ANS\n");
    }
    errGate = obj->w3588_;
    if ((uint16_t)errGate == 0)
      goto count_gate;
    return;
  }
  if ((uint16_t)errGate == 0)
    goto count_gate;
}
