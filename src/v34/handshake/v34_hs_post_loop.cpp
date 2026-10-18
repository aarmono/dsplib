/**
 * @file v34_hs_post_loop.cpp
 * @brief V.34 handshake: the sequence after the transmit loop - WAIT receive handler, per-state status codes and the common status tail.
 *
 * - v04_wait(): the WAIT receive state; drains the receive queue (rxreadqueue()).
 * - v04_txpost(obj, tx): maps the transmit state `tx` to a status code in `obj->state_` (silenceStatus() for SILENCE), then runs statusTail(), which
 *   overrides the code from the run mode, timeouts and the signal-level counter.
 * - v04_post_loop(): the receive dispatch restricted to WAIT, followed by v04_txpost().  Unreferenced in src/; v34handshak() composes the
 *   full dispatchers of v34_hs_dispatch.cpp instead.
 *
 * Comparison widths matter here: the gate `(short)rxW0_ > 5` and the state compares are signed 16-bit, `runMode_` is the full 32-bit word,
 * `sampleTime_ > deadline_` is unsigned, and `rxLevel_` is sign-extended and compared signed with `minSigLevel_`.
 */

#include "v34_handshake.h"
#include "v34/v34_internal.h"

extern "C"
{
}

#include "v34_hs_rx_obj.h"

/**
 * @brief Receive state WAIT: read the receive queue.
 * @param obj  V.34 object
 */
extern "C" void v04_wait(tagV34Object* obj) { rxreadqueue(rxObj(obj)); }

/**
 * @brief Status code for transmit state SILENCE, from the micro-state, receive state and role.
 * @param obj  V.34 object
 * @return 1 when the INFO exchange has just completed from this side's point of view (micro-state INFODONE for the answering modem; with the receive state
 *         RECEIVE: DET_INFO for the calling modem; with WAIT: INFODONE for the calling modem), otherwise 0
 */
static int silenceStatus(tagV34Object* obj)
{
  unsigned mst = obj->mst_;
  unsigned rx;
  if ((uint16_t)mst == V34HS_INFODONE && obj->modeChar_ == V34ROLE_ANSWER)
    return 1;
  rx = obj->rxState_;
  if ((uint16_t)rx == V34HS_RECEIVE)
    return ((uint16_t)mst == V34HS_DET_INFO && obj->modeChar_ == V34ROLE_CALL) ? 1 : 0;
  if ((uint16_t)rx == V34HS_WAIT)
    return ((uint16_t)mst == V34HS_INFODONE && obj->modeChar_ == V34ROLE_CALL) ? 1 : 0;
  return 0;
}

/**
 * @brief Common status tail of the handshake: refine `state_` from the run mode, timeouts and signal level.
 * @param obj  V.34 object; `state_`, `w436_` and `w234_` (low-signal counter) are written
 * @param tx   the transmit-state word read before the handlers ran (not re-read)
 *
 * In order: data run mode sets state 4 (and `w436_` = 3 * rxBaud_); rate-renegotiation modes (4, 5) set 6; retrain modes (2, 3) in SILENCERETRAIN set 0
 * when `k56Idx_ > 0x3c`, else 7; a passed `deadline_` sets 8; a received level below `minSigLevel_` increments `w234_` (otherwise clears it) and a count above
 * 0x257f sets 9; finally the music-on-hold transmit states set 0xf (FRR), 0xd (ON_HOLD) or 0x10 (CLEARDOWN).  Later steps override earlier ones.
 */
static void statusTail(tagV34Object* obj, unsigned tx)
{
  const int32_t w2218 = obj->mod.runMode_;
  if (w2218 == V34RUN_DATA)
  {
    obj->rx.w436_ = (uint16_t)(3 * (int16_t)obj->rate.rxBaud_);
    obj->state_ = 4;
  }
  if ((uint32_t)(w2218 - 4) <= 1u)
    obj->state_ = 6;
  if ((uint32_t)(w2218 - 2) <= 1u && (uint16_t)tx == V34HS_SILENCERETRAIN)
    obj->state_ = ((int16_t)obj->tx.k56Idx_ > 0x3c) ? 0 : 7;
  if ((uint32_t)obj->sampleTime_ > obj->deadline_)
    obj->state_ = 8;
  if ((int32_t)(int16_t)obj->rx.rxLevel_ >= obj->minSigLevel_)
    obj->w234_ = 0;
  else
    obj->w234_ = obj->w234_ + 1;
  if (obj->w234_ > 0x257f)
    obj->state_ = 9;
  {
    int e = (int16_t)tx;
    if (e == V34HS_MOH_FRR)
      obj->state_ = 0xf;
    else if (e == V34HS_MOH_ON_HOLD)
      obj->state_ = 0xd;
    else if (e == V34HS_MOH_CLEARDOWN)
      obj->state_ = 0x10;
  }
}

/**
 * @brief Set the status code for transmit state `tx`, then run the common status tail.
 * @param obj  V.34 object; `state_` is written
 * @param tx   transmit-state word (the caller passes `txState_`, read before the handlers ran)
 *
 * Codes by state: SILENCE via silenceStatus(); TX_DPSK / TX_L1 / SILENCEINFO / TONE_AB / SILENCERETRAIN 0; DATAXMIT 3 if `wE4c_` is 0, else 4;
 * TRNSEG4A / XMITMP / EXMIT 3; PPSEG / TRNSEG4 / JTXMIT / J1TXMIT 2; SSEG / SBARSEG 2 for the calling modem, otherwise 3 when `rxFlags_` bit 3 is set, else 2.
 * Other states keep the previous code.
 */
extern "C" void v04_txpost(tagV34Object* obj, unsigned tx)
{
  unsigned idx = (unsigned)((int)(int16_t)tx - 5);
  if (idx <= 0x45u)
  {
    switch (idx + 5u)
    {
    case V34HS_SILENCE:
      obj->state_ = silenceStatus(obj);
      break;
    case V34HS_TX_DPSK:
    case V34HS_TX_L1:
    case V34HS_SILENCEINFO:
    case V34HS_TONE_AB:
    case V34HS_SILENCERETRAIN:
      obj->state_ = 0;
      break;
    case V34HS_DATAXMIT:
      obj->state_ = obj->rx.blkb.wE4c_ == 0 ? 3 : 4;
      break;
    case V34HS_TRNSEG4A:
    case V34HS_XMITMP:
    case V34HS_EXMIT:
      obj->state_ = 3;
      break;
    case V34HS_PPSEG:
    case V34HS_TRNSEG4:
    case V34HS_JTXMIT:
    case V34HS_J1TXMIT:
      obj->state_ = 2;
      break;
    case V34HS_SSEG:
    case V34HS_SBARSEG:
      if (obj->modeChar_ == V34ROLE_CALL)
        obj->state_ = 2;
      else
        obj->state_ = ((obj->rx.rxFlags_ >> 3) & 1u) != 0 ? 3 : 2;
      break;
    default:
      break;
    }
  }
  statusTail(obj, tx);
}

/**
 * @brief Post-loop sequence with the receive dispatch limited to WAIT.
 * @param obj  V.34 object
 * @return 1 when handled; 0 when the receive state is RX_DPSK, RECEIVE, DET_AB or RX_L1 (their handlers are not run here, nothing was done)
 * @note  No caller exists in src/ (v34handshak() uses v04z_rx_dispatch()).
 */
extern "C" int v04_post_loop(tagV34Object* obj)
{
  if ((int16_t)obj->rx.rxW0_ > 5)
  {
    int rx = (int16_t)obj->rxState_;
    if (rx == V34HS_RX_DPSK || rx == V34HS_RECEIVE || rx == V34HS_DET_AB || rx == V34HS_RX_L1)
      return 0;
    if (rx == V34HS_WAIT)
      v04_wait(obj);
  }
  v04_txpost(obj, obj->txState_);
  return 1;
}
