/**
 * @file v34_hs_rx_dpsk_small.cpp
 * @brief V.34 handshake receive microstates RX_PHASE3_CALL, TX_PHASE3_ANS and TX_PHASE2 (answer / call).
 *
 * Three small receive-microstate handlers called by the RX_DPSK branch of v34handshak() (v34_hs_dispatch.cpp) after fskdemodulate; the caller runs
 * the transmit-state tail afterwards.  Each handler takes `mst`, the microstate word the caller loaded before dispatching, which the transition
 * prints and the "already in the new state" guards use (it is refreshed from the object only where a handler reloads it).
 *
 *   microstate                         handler                  callees
 *   RX_PHASE3_CALL                     v04_t3_rx_phase3_call    -
 *   TX_PHASE3_ANS                      v04_t3_tx_phase3_ans     -
 *   TX_PHASE2_ANS / TX_PHASE2_CALL     v04_t3_tx_phase2         -
 *
 * The 16-bit symbol counters (hsSymCnt_) wrap at 16 bits.  TX_PHASE2's threshold is a signed 16-bit compare of the incremented counter.  The
 * INFO0a record (info0a) is re-initialised by the "repeated info0" recovery path of v04_t3_tx_phase2().
 */

#include <limits.h>
#include "v34_handshake.h"

/** Print a microstate transition; the transmit / receive state names are read from the object at print time. */
static void prMstMem(tagV34Object* obj, unsigned oldM, int newM, long a1, long a2) { v04PrMstX(oldM, newM, obj->txState_, obj->rxState_, a1, a2); }

/**
 * @brief Microstate RX_PHASE3_CALL: wait for the receiver's signal, then load the echo start delay.
 * @param obj  V.34 module object
 * @param mst  microstate word loaded by the caller (debug prints and the already-in-state guard)
 * @note  Does nothing until bit 0 of v21RxBuf_ is set.  Then hsSymCnt_ takes filtDelay_, the microstate becomes TX_PHASE2_CALL (the phase 2
 *        listening state), a debug line is printed and rxFlags_ bit 0x200 is set.
 */
extern "C" void v04_t3_rx_phase3_call(tagV34Object* obj, unsigned mst)
{
  unsigned lv;
  unsigned filtDelayWord;

  if ((obj->v21RxBuf_ & 1u) == 0)
    return;

  /* load the delay and enter the next microstate */
  filtDelayWord = obj->filtDelay_;
  obj->hsSymCnt_ = (uint16_t)filtDelayWord;
  lv = dsplibs_debug_level;
  if ((uint16_t)mst != V34HS_TX_PHASE2_CALL)
  {
    if (lv > 1u)
    {
      prMstMem(obj, mst, V34HS_TX_PHASE2_CALL, v04K56(obj), (long)(int16_t)filtDelayWord);
      lv = dsplibs_debug_level;
    }
    obj->mst_ = V34HS_TX_PHASE2_CALL;
  }

  if (lv > 1u)
    dsplibs_debug_printf("V34PHASE2, RX_PHASE3_CALL ,filtdelay = %d, rxflgs= 0x%x,rx->gain=0x%x\n", (int)(int16_t)obj->filtDelay_, (unsigned)obj->rx.rxFlags_,
                         (int)(int16_t)obj->rx.rxGain_);
  obj->rx.rxFlags_ = (uint16_t)(obj->rx.rxFlags_ | 0x200u);
}

/**
 * @brief Microstate TX_PHASE3_ANS: time the answering side's tone sequence.
 * @param obj  V.34 module object
 * @param mst  microstate word loaded by the caller (debug prints and the already-in-state guard)
 * @note  Counts symbols in hsSymCnt_.  At 0x78 it flips the tone select (toneSel_ ^= 1); at 0xa2 it returns the transmitter to SILENCE, moves the
 *        microstate to RX_PHASE2_ANS and restarts the count.  Other counts do nothing.
 */
extern "C" void v04_t3_tx_phase3_ans(tagV34Object* obj, unsigned mst)
{
  uint16_t symCnt = (uint16_t)(obj->hsSymCnt_ + 1u);
  obj->hsSymCnt_ = symCnt;

  if (symCnt == 0x78)
  {
    obj->toneSel_ = (uint16_t)(obj->toneSel_ ^ 1u);
    return;
  }
  if (symCnt != 0xa2)
    return;

  /* end of the sequence: silence, next microstate, restart the count */
  {
    unsigned tx = obj->txState_;
    if ((uint16_t)tx != V34HS_SILENCE)
    {
      if (dsplibs_debug_level > 1u)
      {
        v04PrTxX(tx, V34HS_SILENCE, obj->rxState_, mst, v04K56(obj), 0xa2L);
        mst = obj->mst_;
      }
      obj->txState_ = V34HS_SILENCE;
    }
  }
  if ((uint16_t)mst != V34HS_RX_PHASE2_ANS)
  {
    if (dsplibs_debug_level > 1u)
      v04PrMstX(mst, V34HS_RX_PHASE2_ANS, V34HS_SILENCE, obj->rxState_, v04K56(obj), v04W78(obj));
    obj->mst_ = V34HS_RX_PHASE2_ANS;
  }
  obj->hsSymCnt_ = 0;
}

/**
 * @brief Microstates TX_PHASE2_ANS / TX_PHASE2_CALL: INFO0 repeat recovery and the periodic tone flip.
 * @param obj  V.34 module object
 * @param mst  microstate word loaded by the caller (debug prints and the already-in-state guard)
 * @note  When the INFO0 flags are complete (v21RxBuf_ & 0xf == 0xf) and w3588_ is zero, the "repeated info0" error recovery is started: the
 *        transmitter goes to TX_DPSK, the microstate to DET_SYNC, the receive buffers are reset and the INFO0a record is re-initialised (its
 *        total field is 0x11, or 0x1e for the calling role in PCM mode).  Independently, every 0x60 symbols (signed 16-bit compare of the incremented
 *        count) it flips toneSel_ and moves the microstate to TX_L1.
 */
extern "C" void v04_t3_tx_phase2(tagV34Object* obj, unsigned mst)
{
  if ((obj->v21RxBuf_ & 0xfu) == 0xfu && obj->w3588_ == 0)
  {
    unsigned txCur;
    unsigned lv;
    int i;
    int d18;

    /* repeated INFO0: reset the recovery bookkeeping */
    obj->w3588_ = 4;
    obj->shortPhase2_ = 0;
    obj->wabca_ = 0;
    obj->w358a_ = 1;
    i = 0;
    do
    {
      obj->wAbae_[i] = 0;
      i = (int16_t)(i + 1);
    } while (i <= 9);
    obj->wAbc2_ = 0;

    /* back to the DPSK transmitter and the sync-detect microstate */
    txCur = obj->txState_;
    lv = dsplibs_debug_level;
    if ((uint16_t)txCur != V34HS_TX_DPSK)
    {
      if (lv > 1u)
      {
        v04PrTxX(txCur, V34HS_TX_DPSK, obj->rxState_, mst, v04K56(obj), v04W78(obj));
        mst = obj->mst_;
        lv = dsplibs_debug_level;
      }
    }
    obj->txState_ = V34HS_TX_DPSK;
    if ((uint16_t)mst != V34HS_DET_SYNC)
    {
      if (lv > 1u)
      {
        v04PrMstX(mst, V34HS_DET_SYNC, obj->txState_, obj->rxState_, v04K56(obj), v04W78(obj));
        lv = dsplibs_debug_level;
      }
      obj->mst_ = V34HS_DET_SYNC;
    }

    /* restart INFO0 reception and re-initialise the INFO0a record */
    obj->v21RxBuf_ = USHRT_MAX;
    obj->v21BitCnt_ = 0;
    d18 = 0x11;
    if (obj->modeChar_ == V34ROLE_CALL && obj->pcmMode_ != 0)
      d18 = 0x1e;
    obj->info0a.i0aTail_.crc = USHRT_MAX;
    obj->info0a.i0aTail_.pos = 0;
    obj->info0a.i0aTail_.widx = 0;
    obj->info0a.i0aTail_.pass = 0;
    obj->info0a.i0aTail_.total = (uint16_t)d18;
    obj->info0a.i0aTail_.chunk = 8;
    obj->info0a.i0aTail_.crcOn = 1;
    obj->info0a.i0aTail_.acc = 0xf72u;
    obj->info0a.i0aTail_.restartAcc = 0xf72u;
    obj->info0a.i0aTail_.nbits = 0xc;
    obj->info0a.i0aTail_.restartBits = 0xc;
    obj->info0a.i0aTail_.loop = 1;

    if (lv > 1u)
      dsplibs_debug_printf("Repeated info0 is detected, errorrecovery is initialized in TX_PHASE2_xxx\n");
    obj->hsSymCnt_ = 0;
  }

  /* every 0x60 symbols: flip the tone select and go to TX_L1 */
  {
    unsigned symCnt = (uint16_t)(obj->hsSymCnt_ + 1u);
    unsigned toneSel;
    unsigned mstCur;
    if ((int16_t)symCnt <= 0x5f)
    {
      obj->hsSymCnt_ = (uint16_t)symCnt;
      return;
    }
    toneSel = obj->toneSel_ ^ 1u;
    obj->hsSymCnt_ = 0;
    obj->toneSel_ = (uint16_t)toneSel;
    mstCur = obj->mst_;
    if ((uint16_t)mstCur != V34HS_TX_L1)
    {
      if (dsplibs_debug_level > 1u)
        v04PrMstX(mstCur, V34HS_TX_L1, obj->txState_, obj->rxState_, v04K56(obj), 0L);
      obj->mst_ = V34HS_TX_L1;
    }
  }
}
