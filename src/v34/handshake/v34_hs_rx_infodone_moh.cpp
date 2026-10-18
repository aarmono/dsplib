/**
 * @file v34_hs_rx_infodone_moh.cpp
 * @brief V.34 handshake receive microstates INFODONE and MOH_TONE_DROP.
 *
 * Two receive-microstate handlers called by the RX_DPSK branch of v34handshak() (v34_hs_dispatch.cpp).  The caller has already run the RX_DPSK head
 * (AGC, FSK demodulation into the buffer at the receive front end + 0x10c) and runs the transmit-state tail afterwards.
 *
 *   microstate         handler                 callees
 *   INFODONE           v04_t3_infodone         settxlevel, V34SetupModulator, txinit
 *   MOH_TONE_DROP      v04_t3_moh_tone_drop    tone_detect, v34handshakinit
 *
 * INFODONE: in transmit state TONE_AB it counts 15 symbols (hsSymCnt_), then returns the transmitter to SILENCE; the calling role also moves to
 * microstate DET_SYNC, the answering role restarts the symbol count at 0x1e or 0x96 depending on w8_.  In transmit state SILENCE, once the INFO
 * flags are set, it sets up the phase 3 transmitter (settxlevel, V34SetupModulator, receive state WAIT, transmit state SSEG, txinit).
 * MOH_TONE_DROP: counts symbols; a tone-detect hit arms mohF8_ (signal drop seen); when the count passes bulkDelay_/4 + 0x12c0 it either
 * disconnects (transmit state MOH_CLEARDOWN, receive state WAIT, mohE4_ / mohE2_) or, with mohF9_ clear, retrains (v34handshakinit).
 */

#include "v34_handshake.h"
#include "v34/v34_internal.h"
#include "vpcm/vpcm_v34_api.h"

extern "C"
{
}

#include "v34_hs_rx_obj.h"

/**
 * @brief Handle one symbol interval of the INFODONE receive microstate.
 * @param obj  V.34 module object
 * @param mst  the microstate word the caller loaded before dispatching (used only in the debug prints; refreshed from obj->mst_ after a print)
 * @note  Transmit state TONE_AB: counts to 15 symbols, then goes to SILENCE.  Transmit state SILENCE: acts only when the signed byte wAbff_ is
 *        positive and w240_ exceeds 0x240; then it configures the phase 3 transmitter.  Any other transmit state does nothing.
 */
extern "C" void v04_t3_infodone(tagV34Object* obj, unsigned mst)
{
  unsigned tx = obj->txState_;

  /* waiting out the end of the INFO exchange: 15 symbols, then silence */
  if ((uint16_t)tx == V34HS_TONE_AB)
  {
    unsigned symCnt = (uint16_t)(obj->hsSymCnt_ + 1u);
    if ((int16_t)symCnt <= 0xf)
    {
      obj->hsSymCnt_ = (uint16_t)symCnt;
      return;
    }

    if (obj->modeChar_ == V34ROLE_CALL)
    {
      /* calling role: go quiet and wait for the answerer's sync tone */
      unsigned lv = dsplibs_debug_level;
      obj->hsSymCnt_ = (uint16_t)symCnt;
      if (lv > 1u)
      {
        v04PrTxX(V34HS_TONE_AB, V34HS_SILENCE, obj->rxState_, mst, v04K56(obj), (long)(int16_t)symCnt);
        mst = obj->mst_;
      }
      obj->txState_ = V34HS_SILENCE;
      if ((uint16_t)mst != V34HS_DET_SYNC)
      {
        if (dsplibs_debug_level > 1u)
          v04PrMstX(mst, V34HS_DET_SYNC, V34HS_SILENCE, obj->rxState_, v04K56(obj), v04W78(obj));
        obj->mst_ = V34HS_DET_SYNC;
      }
      obj->v21BitCnt_ = 0;
    }
    else
    {
      /* answering role: go quiet and preload the symbol count with the start delay for the next phase */
      if (dsplibs_debug_level > 1u)
      {
        obj->hsSymCnt_ = (uint16_t)symCnt;
        v04PrTxX(V34HS_TONE_AB, V34HS_SILENCE, obj->rxState_, mst, v04K56(obj), (long)(int16_t)symCnt);
      }
      obj->txState_ = V34HS_SILENCE;
      obj->hsSymCnt_ = obj->w8_ == 0x30 ? 0x1e : 0x96;
    }
    return;
  }

  /* in silence, with the INFO flags set: bring up the phase 3 transmitter */
  if ((uint16_t)tx != V34HS_SILENCE)
    return;
  if ((int8_t)obj->wAbff_ <= 0)
    return;
  if (obj->w240_ <= 0x240)
    return;

  if (dsplibs_debug_level > 1u)
    dsplibs_debug_printf("V34Hshak: Setting up transmitter for phase3...\r\n");
  settxlevel(obj, (const unsigned short*)&obj->info1c);

  /* modulator for the negotiated rate; the fifth argument selects the PCM (V.90 / K56flex) pre-emphasis */
  {
    int v90 = 0;
    if (obj->pcmMode_ != 0)
      v90 = 1;
    else if (obj->k56Mode_ != 0)
      v90 = 1;
    V34SetupModulator(&obj->mod, (int16_t)obj->rate.baud_, (int16_t)obj->rate.carrier_, (int16_t)obj->rate.preemp_, v90, 1);
  }
  obj->rx.rxFlags_ = (uint16_t)(obj->rx.rxFlags_ & ~0x800u);

  /* receive state WAIT, transmit state SSEG (with their debug prints) */
  {
    unsigned rx = obj->rxState_;
    if ((uint16_t)rx != V34HS_WAIT)
    {
      if (dsplibs_debug_level > 1u)
        dsplibs_debug_printf(V04_FMT_RX, StateName[(int16_t)rx], StateName[V34HS_WAIT], StateName[(int16_t)obj->txState_], StateName[(int16_t)obj->mst_],
                             v04K56(obj), v04W78(obj));
      obj->rxState_ = V34HS_WAIT;
    }
  }
  {
    unsigned t = obj->txState_;
    if ((uint16_t)t != V34HS_SSEG)
    {
      if (dsplibs_debug_level > 1u)
        v04PrTx(obj, t, V34HS_SSEG);
      obj->txState_ = V34HS_SSEG;
    }
  }

  obj->tx.txCnt_ = 0;
  obj->tx.txFlags_ = (uint16_t)(obj->tx.txFlags_ | 0x200u);
  txinit(obj);
}

/**
 * @brief Handle one symbol interval of the MOH_TONE_DROP receive microstate (music on hold, waiting for the FRR signal to drop).
 * @param obj      V.34 module object
 * @param mst      unused (kept for the common handler signature)
 * @param samples  FSK-demodulator buffer prepared by the RX_DPSK head, handed on to tone_detect()
 * @note  The tone-detect result is tested as a 16-bit value.  When the symbol count (signed 16-bit) reaches bulkDelay_/4 + 0x12c0 the handler
 *        disconnects if mohF9_ is set (mohE4_ / mohE2_ = 1) and otherwise restarts the handshake as a retrain (mohE6_ = 1).
 */
extern "C" void v04_t3_moh_tone_drop(tagV34Object* obj, unsigned mst, const short* samples)
{
  uint16_t w = (uint16_t)(obj->hsSymCnt_ + 1u);
  const short* end = obj->rx.rxEnd_;
  (void)mst;

  /* count the symbol; a tone-detect hit means the signal dropped */
  obj->hsSymCnt_ = w;
  if ((int16_t)tone_detect(rxObj(obj), &obj->det, samples, end) != 0)
  {
    if (obj->mohF8_ == 0)
    {
      if (dsplibs_debug_level > 1u)
        dsplibs_debug_printf("V34Handshake: Detected signal drop on FRR request, time to move to phase1...\r\n");
      obj->mohF8_ = 1;
    }
  }

  /* timeout: no MH sequence arrived */
  if ((int)(int16_t)obj->hsSymCnt_ < ((int)(int16_t)obj->bulkDelay_ >> 2) + 0x12c0)
    return;
  if (obj->mohF9_ != 0)
  {
    unsigned tx;
    unsigned rx;
    if (dsplibs_debug_level > 1u)
      dsplibs_debug_printf("MOH: Timeout waiting for MH sequence under MHfrr, disconnecting...\r\n");
    tx = obj->txState_;
    if ((uint16_t)tx != V34HS_MOH_CLEARDOWN)
    {
      if (dsplibs_debug_level > 1u)
        v04PrTx(obj, tx, V34HS_MOH_CLEARDOWN);
      obj->txState_ = V34HS_MOH_CLEARDOWN;
      tx = V34HS_MOH_CLEARDOWN;
    }
    rx = obj->rxState_;
    if ((uint16_t)rx != V34HS_WAIT)
    {
      if (dsplibs_debug_level > 1u)
        dsplibs_debug_printf(V04_FMT_RX, StateName[(int16_t)rx], StateName[V34HS_WAIT], StateName[(int16_t)tx], StateName[(int16_t)obj->mst_], v04K56(obj),
                             v04W78(obj));
      obj->rxState_ = V34HS_WAIT;
    }
    obj->mohE4_ = 1;
    obj->mohE2_ = 1;
  }
  else
  {
    if (dsplibs_debug_level > 1u)
      dsplibs_debug_printf("MOH: Timeout waiting for MH sequence under MHfrr, initiating retrain\r\n");
    v34handshakinit(obj, V34HSINIT_RETRAIN);
    obj->mohE6_ = 1;
  }
}
