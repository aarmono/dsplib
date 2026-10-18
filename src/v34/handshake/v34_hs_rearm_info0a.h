/**
 * @file v34_hs_rearm_info0a.h
 * @brief rearmInfo0a(): re-arm the INFO0a bit queue and the V.21 receive words before a new INFO0a exchange.
 *
 * Included by v34_hs_rx_phase1_call.cpp, v34_hs_rx_phase2_call.cpp and v34_hs_tx_phase1_call.cpp.  Plain `static` (not inline): include it only in a
 * file that calls rearmInfo0a().
 */
#ifndef V34_HS_REARM_INFO0A_H
#define V34_HS_REARM_INFO0A_H

#include <limits.h>
#include "v34_handshake.h"

/**
 * @brief Re-arm the INFO0a bit queue and its parameter words, and reset the V.21 receive shift register and bit counter.
 * @param obj  V.34 object; `info0a` queue, `v21RxBuf_` (set to USHRT_MAX) and `v21BitCnt_` (set to 0) are written
 * @note  Queue word `total` is 0x1e when the object is the calling modem (modeChar_ == 0x65) in a PCM mode (pcmMode_ != 0) and 0x11 otherwise.
 */
static void rearmInfo0a(tagV34Object* obj)
{
  tag_bitQ* q = (tag_bitQ*)&obj->info0a;
  obj->v21RxBuf_ = USHRT_MAX;
  obj->v21BitCnt_ = 0;
  q->tail_.crc = USHRT_MAX;
  q->tail_.pos = 0;
  q->tail_.widx = 0;
  q->tail_.pass = 0;
  q->tail_.total = (uint16_t)((obj->modeChar_ == 0x65 && obj->pcmMode_ != 0) ? 0x1e : 0x11);
  q->tail_.chunk = 8;
  q->tail_.crcOn = 1;
  q->tail_.acc = 0xf72u;
  q->tail_.restartAcc = 0xf72u;
  q->tail_.nbits = 0xc;
  q->tail_.restartBits = 0xc;
  q->tail_.loop = 1;
}

#endif /* V34_HS_REARM_INFO0A_H */
