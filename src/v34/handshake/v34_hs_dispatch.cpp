/**
 * @file v34_hs_dispatch.cpp
 * @brief V.34 handshake state machine: v34handshak() and its transmit / receive dispatchers.
 *
 * `void v34handshak(tagV34Object *obj)` runs once per symbol block (called from VPcmV34Progress / datapumpv34).  It leaves a status code in
 * `obj->state_` for the caller.  One call performs, in order:
 * 1. Transmit loop: while `(short)txIdx_ < (short)blkLen_`, dispatch on the transmit state `txState_` (v04z_tx_dispatch()).  The jump table covers the 82
 *    states 5 .. 0x56 (V34HS_SILENCE is 5); 19 distinct handlers (v04_tx_*, in the other v34_hs_tx_*.cpp files) exist, every other state does nothing.
 *    A handler produces output samples and advances `txIdx_`.
 * 2. Receive dispatch (v04z_rx_dispatch()): only if `(short)rxW0_ > 5`, on the receive state `rxState_`:
 *    - V34HS_RX_DPSK (0x2b): rxDpsk() runs the AGC, the retrain-detector window (when `dftActive_`), the V.21 bit-counter capture and fskdemodulate(), then
 *      dispatches on the micro-state `mst_` (0x29 .. 0x50, 15 handlers v04_t3_*);
 *    - V34HS_RECEIVE (4): v04_rx_receive(); V34HS_WAIT (0x23): v04_wait(); V34HS_DET_AB (0x35): v04_rx_det_ab(); V34HS_RX_L1 (0x48): v04_rx_l1();
 *    - any other receive state does nothing.
 * 3. Transmit post-processing (v04_txpost() in v34_hs_post_loop.cpp): per-transmit-state status code and the common status tail.
 *
 * @warning A transmit state without a handler never advances `txIdx_`, so step 1 would not terminate; callers never leave such a state active.
 * v04z_tx_dispatch() (ONE loop iteration) and v04z_rx_dispatch() (receive chain without step 3) are separate entry points so the unit tests can drive the
 * halves; v34handshak() is their composition.
 */

#include "v34_handshake.h"
#include "v34/v34_internal.h"

/**
 * @brief Run one transmit handler: the one for the current transmit state.
 * @param o  V.34 object; `txState_` selects the handler (states outside V34HS_SILENCE .. +0x51 (5 .. 0x56) and states without a handler do nothing)
 */
extern "C" void v04z_tx_dispatch(tagV34Object* o)
{
  unsigned i = (unsigned)((int)(int16_t)o->txState_ - 5);
  if (i > 0x51u)
    return;
  switch (i + 5)
  {
  case V34HS_SSEG:
    v04_tx_sseg(o);
    break;
  case V34HS_TONE_AB:
    v04_tx_toneab(o);
    break;
  case V34HS_XMIT0:
    v04_tx_xmit0(o);
    break;
  case V34HS_DATAXMIT:
    v04_tx_dataxmit(o);
    break;
  case V34HS_TXLEVEL:
    v04_tx_txlevel(o);
    break;
  case V34HS_JaTXMIT:
    v04_tx_jatxmit(o);
    break;
  case V34HS_K56JaTXMIT:
    v04_tx_k56jatxmit(o);
    break;
  case V34HS_MOH_SILENCE:
  case V34HS_MOH_ON_HOLD:
  case V34HS_MOH_FRR:
  case V34HS_MOH_CLEARDOWN:
    v04_tx_moh(o);
    break;
  case V34HS_TX_L1:
    v04_tx_txl1(o);
    break;
  case V34HS_SBARSEG:
    v04_tx_sbarseg(o);
    break;
  case V34HS_PPSEG:
    v04_tx_ppseg(o);
    break;
  case V34HS_TXMD:
    v04_tx_txmd(o);
    break;
  case V34HS_EXMIT:
    v04_tx_exmit(o);
    break;
  case V34HS_SILENCE:
  case V34HS_SILENCEINFO:
  case V34HS_SILENCERETRAIN:
    v04_tx_silence(o);
    break;
  case V34HS_JTXMIT:
  case V34HS_J1TXMIT:
    v04_tx_jtxmit(o);
    break;
  case V34HS_TX_DPSK:
    v04_tx_dpsk(o);
    break;
  case V34HS_XMITMP:
    v04_tx_xmitmp(o);
    break;
  case V34HS_TRNSEG4:
    v04_tx_trnseg4(o);
    break;
  case V34HS_TRNSEG4A:
    v04_tx_trnseg4a(o);
    break;
  default:
    break;
  }
}

/**
 * @brief Receive state RX_DPSK: front end for every DPSK-phase micro-state.
 * @param o  V.34 object
 *
 * Runs V34agc(), the retrain-detector window v04_rx_dpsk_dft() when `dftActive_`, captures `v21BitCnt_`, runs fskdemodulate() on `rxFskBuf_`, then
 * calls the handler of the micro-state `mst_` (states 0x29 .. 0x50 only; others do nothing).
 * @note  `v21BitCnt_` is captured BEFORE fskdemodulate() because the V34HS_TX_PHASE1_CALL handler compares that earlier value, not the updated one.
 */
static void rxDpsk(tagV34Object* o)
{
  unsigned mst, i;
  int preDemodWord;
  V34agc(&o->rx);
  if (o->dftActive_)
    v04_rx_dpsk_dft(o);
  preDemodWord = (int)(int16_t)o->v21BitCnt_; /* before fskdemodulate() changes it */
  fskdemodulate(o, (const uint16_t*)o->rx.rxFskBuf_, o->fskState_);
  mst = o->mst_;
  i = (unsigned)((int)(int16_t)mst - 0x29);
  if (i > 0x27u)
    return;
  switch (mst)
  {
  case V34HS_RX_PHASE3_CALL:
    v04_t3_rx_phase3_call(o, mst);
    break;
  case V34HS_TX_PHASE3_ANS:
    v04_t3_tx_phase3_ans(o, mst);
    break;
  case V34HS_TX_PHASE2_ANS:
  case V34HS_TX_PHASE2_CALL:
    v04_t3_tx_phase2(o, mst);
    break;
  case V34HS_INFODONE:
    v04_t3_infodone(o, mst);
    break;
  case V34HS_MOH_TONE_DROP:
    v04_t3_moh_tone_drop(o, mst, (const short*)o->rx.rxFskBuf_);
    break;
  case V34HS_MOH_TONE:
    v04_t3_moh_tone(o, (const short*)o->rx.rxFskBuf_);
    break;
  case V34HS_DET_SYNC:
    v04_t3_det_sync(o, mst, (const short*)o->rx.rxFskBuf_);
    break;
  case V34HS_DET_INFO:
    v04_t3_det_info(o);
    break;
  case V34HS_RX_PHASE1_ANS:
    v04_t3_rx_phase1_ans(o, mst);
    break;
  case V34HS_RX_PHASE2_ANS:
    v04_t3_rx_phase2_ans(o, mst);
    break;
  case V34HS_TX_PHASE1_CALL:
    v04_t3_tx_phase1_call(o, (unsigned)preDemodWord);
    break;
  case V34HS_TX_L1:
    v04_t3_tx_l1(o, mst);
    break;
  case V34HS_RX_PHASE1_CALL:
    v04_t3_rx_phase1_call(o);
    break;
  case V34HS_RX_PHASE2_CALL:
    v04_t3_rx_phase2_call(o, (const short*)o->rx.rxFskBuf_);
    break;
  case V34HS_TX_PHASE1_ANS:
    v04_t3_tx_phase1_ans(o);
    break;
  default:
    break;
  }
}

/**
 * @brief Run the receive handler for the current receive state (step 2 of v34handshak()).
 * @param o  V.34 object; nothing happens unless `(short)rxW0_ > 5`; `rxState_` is compared as a signed 16-bit value
 */
extern "C" void v04z_rx_dispatch(tagV34Object* o)
{
  int rx;
  if (!((int16_t)o->rx.rxW0_ > 5))
    return;
  rx = (int16_t)o->rxState_;
  if (rx == V34HS_RX_DPSK)
    rxDpsk(o);
  else if (rx > V34HS_RX_DPSK)
  {
    if (rx == V34HS_DET_AB)
      v04_rx_det_ab(o);
    else if (rx == V34HS_RX_L1)
      v04_rx_l1(o);
  }
  else if (rx == V34HS_RECEIVE)
    v04_rx_receive(o);
  else if (rx == V34HS_WAIT)
    v04_wait(o);
}

/**
 * @brief The V.34 handshake state machine, one symbol block.
 * @param obj  V.34 object; the transmit loop runs until `txIdx_` reaches `blkLen_`, then the receive dispatch and the status post-processing run once
 * @note  Leaves the status code in `obj->state_` (via v04_txpost()).
 */
extern "C" void v34handshak(tagV34Object* obj)
{
  while ((int16_t)obj->tx.txIdx_ < (int16_t)obj->tx.blkLen_)
    v04z_tx_dispatch(obj);
  v04z_rx_dispatch(obj);
  v04_txpost(obj, obj->txState_);
}
