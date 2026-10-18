/**
 * @file v34_hs_rx_moh_tone.cpp
 * @brief V.34 handshake receive microstate MOH_TONE: waiting for the music-on-hold (MH) tone.
 *
 * One of the receive-microstate handlers called by the RX_DPSK branch of v34handshak() (v34_hs_dispatch.cpp).  The caller has already run the
 * RX_DPSK head (AGC, FSK demodulation into the buffer at the receive front end + 0x10c) and passes that buffer on; after the handler returns the
 * caller runs the transmit-state tail.
 *
 *   microstate   handler            callees
 *   MOH_TONE     v04_t3_moh_tone    tone_detect, detectorinit
 *
 * MOH_TONE counts symbols in hsSymCnt_.  A tone-detect hit after wabfc_ symbols ends the wait: the transmit state becomes TX_DPSK and the tone
 * detector is re-armed for 1200 Hz (c1200_) or, for the calling role, 2400 Hz (c2400_) when the MOH request is the retrain one (mohF0_ == 1,
 * then microstate MOH_TONE_DROP), otherwise the microstate becomes DET_SYNC.  Finally the INFO0d record becomes the second bit queue (bitQ2_),
 * bitQ_ is pointed at the INFO0a record, and the symbol counters and the tone select are cleared.
 */

#include <limits.h>
#include "v34_handshake.h"
#include "v34/v34_internal.h"
#include "v34/v34_frame_tables.h" /* c1200_, c2400_ */

/**
 * @brief Handle one symbol interval of the MOH_TONE receive microstate.
 * @param obj  V.34 module object
 * @param buf  FSK-demodulator buffer prepared by the RX_DPSK head (receive front end + 0x10c), handed on to tone_detect()
 * @note  Does nothing until tone_detect() reports a hit and at least wabfc_ symbols have been counted (signed 16-bit compare).  The tone-detect
 *        result is tested as a 16-bit value.  State changes are printed at debug level above 1.
 */
extern "C" void v04_t3_moh_tone(tagV34Object* obj, const short* buf)
{
  const short* end = obj->rx.rxEnd_;

  /* count the symbol; a tone hit only counts once the minimum wait has elapsed */
  obj->hsSymCnt_ = (uint16_t)(obj->hsSymCnt_ + 1u);
  if ((int16_t)tone_detect(&obj->rx, &obj->det, buf, end) == 0)
    return;
  if ((int16_t)obj->hsSymCnt_ < (int16_t)obj->wabfc_)
    return;

  /* tone found: switch the transmitter to DPSK and pick the next receive microstate */
  v04SetTx(obj, obj->txState_, V34HS_TX_DPSK);
  if (obj->mohF0_ == 1)
  {
    /* retrain request: listen for the carrier dropping (2400 Hz for the calling role, else 1200 Hz) */
    detectorinit(&obj->det, obj->modeChar_ == V34ROLE_CALL ? c2400_ : c1200_, 1, 0xf0, 0x32, 0x800, 0x400);
    obj->det.toneDetF6_ = 1;
    {
      unsigned mst = obj->mst_;
      if ((uint16_t)mst != V34HS_MOH_TONE_DROP)
      {
        if (dsplibs_debug_level > 1u)
          v04PrMstX(mst, V34HS_MOH_TONE_DROP, obj->txState_, obj->rxState_, v04K56(obj), v04W78(obj));
        obj->mst_ = V34HS_MOH_TONE_DROP;
      }
    }
  }
  else
  {
    unsigned mst = obj->mst_;
    if ((uint16_t)mst != V34HS_DET_SYNC)
    {
      if (dsplibs_debug_level > 1u)
        v04PrMstX(mst, V34HS_DET_SYNC, obj->txState_, obj->rxState_, v04K56(obj), v04W78(obj));
      obj->mst_ = V34HS_DET_SYNC;
    }
  }

  /* restart the INFO reception: INFO0d is the second bit queue, INFO0a the first */
  obj->fskCounter_ = 0;
  obj->v21BitCnt_ = 0;
  obj->info0d.i0dTail_.crc = USHRT_MAX;
  obj->info0d.i0dTail_.total = 8;
  obj->hsSymCnt_ = 0;
  obj->bitQ2_ = (tag_bitQ*)&obj->info0d;
  obj->bitQ_ = (tag_bitQ*)&obj->info0a;
  obj->toneSel_ = 0;
  obj->tx.k56Idx_ = 0;
}
