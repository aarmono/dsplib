/**
 * @file v34_hs_rx_answer.cpp
 * @brief V.34 handshake: the answering modem's receive micro-states RX_PHASE1_ANS and RX_PHASE2_ANS.
 *
 * Both handlers are called from the RX_DPSK receive branch (rxDpsk() in v34_hs_dispatch.cpp) with the micro-state word the dispatcher read; the caller
 * then runs the transmit post-processing (v04_txpost()).
 *
 * | micro-state (value)   | handler                  | helpers called  |
 * |-----------------------|--------------------------|-----------------|
 * | RX_PHASE1_ANS (0x31)  | v04_t3_rx_phase1_ans()   | ApplyBulkDelay  |
 * | RX_PHASE2_ANS (0x32)  | v04_t3_rx_phase2_ans()   | rxtiminginit    |
 *
 * Each call counts one symbol in `hsSymCnt_`.  While the count is below `filtDelay_ + 0x4c` the handlers watch for a repeated INFO0 in the V.21 receive
 * register (`v21RxBuf_ & 0x3ff == 0x372` in phase 1, `> 0x200` in phase 2); errorRecovery() then restarts the INFO exchange (transmit state TX_DPSK,
 * micro-state DET_SYNC, INFO0a bit queue re-armed).  When the count exceeds `filtDelay_ + 0x50` and bit 0 of `v21RxBuf_` is set:
 * - RX_PHASE1_ANS derives the bulk delay from the symbol count (`bulkDelay_ = 4 * (hsSymCnt_ - filtDelay_) - 0x18c`, or the previous session's value
 *   when `shortPhase2_` is set), clamps it to >= 1, applies it with ApplyBulkDelay(), restarts the count at `filtDelay_`, moves the micro-state to
 *   TX_PHASE2_ANS and saves the receive gain in `rxGainSaved_`;
 * - RX_PHASE2_ANS switches the receiver to the 2400 baud / 1800 Hz data-mode setup (hsine1800 table, receive words, rxtiminginit()), sets the receive
 *   state to RX_L1 and halves the receive gain.
 *
 * Counter compares are signed 16-bit.
 */

#include <limits.h>
#include "v34_handshake.h"
#include "v34/v34_internal.h"
#include "v34/v34_init_tables.h" /* hsine1800 */

/**
 * @brief Restart the INFO exchange after a repeated INFO0 was detected.
 * @param obj  V.34 object
 * @param mst  micro-state word the caller read (used for the transition trace, not re-read)
 * @param msg  debug message printed (level > 1) after the reset
 *
 * Clears the INFO bookkeeping (as clearInfoState()), sets the transmit state to TX_DPSK and the micro-state to DET_SYNC, and re-arms the INFO0a queue and
 * V.21 receive words (as rearmInfo0a()).
 */
static void errorRecovery(tagV34Object* obj, unsigned mst, const char* msg)
{
  unsigned txState;
  unsigned lv;
  int i;

  /* clear the INFO bookkeeping */
  obj->shortPhase2_ = 0;
  obj->w3588_ = 4;
  obj->wabca_ = 0;
  obj->w358a_ = 1;
  i = 0;
  do
  {
    obj->wAbae_[i] = 0;
    i = (int16_t)(i + 1);
  } while (i <= 9);
  obj->wAbc2_ = 0;

  /* transmit state TX_DPSK, micro-state DET_SYNC */
  txState = obj->txState_;
  lv = dsplibs_debug_level;
  if ((uint16_t)txState != V34HS_TX_DPSK)
  {
    if (lv > 1u)
    {
      v04PrTxX(txState, V34HS_TX_DPSK, obj->rxState_, mst, v04K56(obj), v04W78(obj));
      mst = obj->mst_;
      lv = dsplibs_debug_level;
    }
    obj->txState_ = V34HS_TX_DPSK;
  }
  if ((uint16_t)mst != V34HS_DET_SYNC)
  {
    if (lv > 1u)
    {
      v04PrMstX(mst, V34HS_DET_SYNC, obj->txState_, obj->rxState_, v04K56(obj), v04W78(obj));
      lv = dsplibs_debug_level;
    }
    obj->mst_ = V34HS_DET_SYNC;
  }

  /* re-arm the INFO0a queue */
  obj->v21RxBuf_ = USHRT_MAX;
  obj->v21BitCnt_ = 0;
  obj->info0a.i0aTail_.crc = USHRT_MAX;
  obj->info0a.i0aTail_.pos = 0;
  obj->info0a.i0aTail_.widx = 0;
  obj->info0a.i0aTail_.pass = 0;
  obj->info0a.i0aTail_.total = (uint16_t)((obj->modeChar_ == V34ROLE_CALL && obj->pcmMode_ != 0) ? 0x1e : 0x11);
  obj->info0a.i0aTail_.chunk = 8;
  obj->info0a.i0aTail_.crcOn = 1;
  obj->info0a.i0aTail_.acc = 0xf72u;
  obj->info0a.i0aTail_.restartAcc = 0xf72u;
  obj->info0a.i0aTail_.nbits = 0xc;
  obj->info0a.i0aTail_.restartBits = 0xc;
  obj->info0a.i0aTail_.loop = 1;

  if (lv > 1u)
    dsplibs_debug_printf("%s", msg);
}

/**
 * @brief Micro-state RX_PHASE1_ANS: wait for the answering modem's INFO0 phase to settle, then set the bulk delay and move to TX_PHASE2_ANS.
 * @param obj    V.34 object
 * @param mstIn  micro-state word read by the dispatcher
 */
extern "C" void v04_t3_rx_phase1_ans(tagV34Object* obj, unsigned mstIn)
{
  unsigned symCnt = (uint16_t)(obj->hsSymCnt_ + 1u);
  unsigned filtDelay;
  obj->hsSymCnt_ = (uint16_t)symCnt;
  filtDelay = obj->filtDelay_;

  /* repeated INFO0 early in the phase: restart the exchange */
  if ((int16_t)symCnt > 0x28 && (int16_t)symCnt < (int16_t)filtDelay + 0x4c && (obj->v21RxBuf_ & 0x3ffu) == 0x372u && obj->w3588_ == 0)
  {
    errorRecovery(obj, mstIn, "Repeated info0 is detected, errorrecovery is initialized in RX_PHASE1_ANS\n");
    filtDelay = obj->filtDelay_;
  }

  /* phase is over only after filtDelay_ + 0x50 symbols and once the INFO flag bit is set */
  if ((int16_t)obj->hsSymCnt_ <= (int16_t)filtDelay + 0x50)
    return;
  obj->rx.rxFlags_ = (uint16_t)(obj->rx.rxFlags_ | 0x200u);
  if ((obj->v21RxBuf_ & 1u) == 0)
    return;

  /* bulk delay: from the previous session after a short phase 2, else from the measured symbol count */
  if (obj->shortPhase2_ != 0)
  {
    obj->bulkDelay_ = obj->prevBulkDelay_;
    if (dsplibs_debug_level > 1u)
      dsplibs_debug_printf("Setting Bulk delay according to prev session - %d\r\n", (int)(int16_t)obj->bulkDelay_);
  }
  else
  {
    obj->bulkDelay_ = (uint16_t)(((int)obj->hsSymCnt_ - (int)obj->filtDelay_) * 4 - 0x18c);
  }
  if (dsplibs_debug_level > 1u)
    dsplibs_debug_printf("On RX_PHASE1_ANS: is short=%d, bulkDelay=%d, filtDelay=%d\r\n", (int)(int16_t)obj->shortPhase2_, (int)(int16_t)obj->bulkDelay_,
                         (int)(int16_t)obj->filtDelay_);

  {
    unsigned w = obj->bulkDelay_;
    if ((int16_t)w <= 0)
    {
      obj->bulkDelay_ = 1;
      w = 1;
    }
    ApplyBulkDelay(obj, (int16_t)w);
  }

  /* restart the symbol count and move on to TX_PHASE2_ANS */
  {
    unsigned mst = obj->mst_;
    unsigned fd = obj->filtDelay_;
    obj->hsSymCnt_ = (uint16_t)fd;
    if ((uint16_t)mst != V34HS_TX_PHASE2_ANS)
    {
      if (dsplibs_debug_level > 1u)
        v04PrMstX(mst, V34HS_TX_PHASE2_ANS, obj->txState_, obj->rxState_, v04K56(obj), (long)(int16_t)fd);
      obj->mst_ = V34HS_TX_PHASE2_ANS;
    }
  }
  obj->rx.rxGainSaved_ = obj->rx.rxGain_;
}

/**
 * @brief Micro-state RX_PHASE2_ANS: wait for the INFO0 phase to settle, then set up the 2400 baud / 1800 Hz receiver and enter receive state RX_L1.
 * @param obj    V.34 object
 * @param mstIn  micro-state word read by the dispatcher
 */
extern "C" void v04_t3_rx_phase2_ans(tagV34Object* obj, unsigned mstIn)
{
  unsigned symCnt = (uint16_t)(obj->hsSymCnt_ + 1u);
  unsigned filtDelay;
  obj->hsSymCnt_ = (uint16_t)symCnt;
  filtDelay = obj->filtDelay_;

  /* repeated INFO0 early in the phase: restart the exchange */
  if ((int16_t)symCnt > 0x32 && (int16_t)symCnt < (int16_t)filtDelay + 0x4c && (obj->v21RxBuf_ & 0x3ffu) > 0x200u && obj->w3588_ == 0)
  {
    errorRecovery(obj, mstIn, "Repeated info0 is detected, errorrecovery is initialized in RX_PHASE2_ANS\n");
    filtDelay = obj->filtDelay_;
  }

  if ((int16_t)obj->hsSymCnt_ <= (int16_t)filtDelay + 0x50)
    return;
  if ((obj->v21RxBuf_ & 1u) == 0)
    return;

  /* data-mode receiver setup for 2400 baud (0x960) / 1800 Hz carrier (0x708) */
  {
    unsigned lv = dsplibs_debug_level;
    obj->hsSymCnt_ = 0x14;
    if (lv > 1u)
      dsplibs_debug_printf("V34SetupDemodulator: baudrate %ld, carrier %ld\n", 0x960L, 0x708L);
  }
  obj->rx.w38c_ = 4;
  obj->rx.hsineLen_ = 0x10;
  obj->rx.w414_ = 0x3e80;
  obj->rx.w412_ = 0x3e80;
  obj->rx.w422_ = 0x3e80;
  obj->rx.w410_ = 0x1f40;
  obj->rx.hsineTab_ = hsine1800;
  rxtiminginit(obj);

  /* receive state RX_L1, halved receive gain */
  v04SetRx(obj, obj->rxState_, V34HS_RX_L1);
  {
    unsigned fl = obj->rx.rxFlags_;
    int g = (int16_t)obj->rx.rxGain_;
    obj->rx.rxGain_ = (uint16_t)(g >> 1);
    obj->rx.rxFlags_ = (uint16_t)(fl & ~0xa00u);
  }
}
