/**
 * @file v34_hs_rx_dpsk_dft.cpp
 * @brief V.34 handshake: DFT retrain-request detector run by the RX_DPSK head while INFO1 is being searched for.
 *
 * Entry point v04_rx_dpsk_dft(), called by the RX_DPSK head of v34handshak() (v34_hs_dispatch.cpp) right after V34agc when dftActive_ is set; the
 * head then continues with the v21BitCnt_ capture, fskdemodulate and the microstate dispatch.
 *
 * The receiver keeps three DFT bins (DftBin; C = the members `dftC_`).  Every call feeds them (dftupdate(C, 3, buf, 4));
 * every 32nd call (the counter rtDetCnt_ steps by 4 up to 0x80) evaluates them (dftenergy(C, 3, 5), then the records are cleared) with a two-state
 * detector (state word rtDetState_):
 *  - state 1: the three energies (`energyHi` of the bins) are all below their limits (`quietLimit`): count a quiet interval
 *    (rtDetHits_++).  Otherwise the quiet count is compared with its threshold (rtDetThrA_); reaching it enters state 2.  The count restarts at 0.
 *  - state 2: the energy of the middle bin against its `toneLimit`: not above it resets the count and returns to state 1, above it counts.  Then
 *    checkRetrain() compares the count with its threshold (rtDetThrB_).
 *  - when the count reaches rtDetThrB_ the retrain request is declared (debug text "DET_SYNC : retrain request detected while searching for
 *    info1"): dftActive_ is cleared, the records are re-armed with the limits 0x50 / 0xbb8 and the thresholds 0x600 / 0x800 / 0xa00, the detector
 *    is reset (state 1, thresholds 3 and 9), w3588_ |= 2, the microstate becomes TX_PHASE1_ANS, the receive state RX_DPSK, the transmit state
 *    TONE_AB, a toneSel_ above 1 is cleared, and the counters and INFO0 words are cleared.
 *
 * The detector variables sit in the byte region that begins at the member wa248_ and are reached through ld16 / ld32 / st16 / st32
 * (v34_hs_bytes.h) at the offsets U_(member); only the first word of the region has a name of its own.
 */

#include <limits.h>
#include "v34_handshake.h"
#include "core/dsp_access.h"
#include "v34/v34_internal.h"

/* the byte offset of member m of the object from the word wa248_ that `u` points at (the region is addressed by byte offset) */
#define U_(m) MEMBER_DISTANCE(tagV34Object, wa248_, m)

extern "C"
{
}

#include "v34_hs_bytes.h"

/**
 * @brief Declare a retrain request when the detector's hit count has reached its threshold.
 * @param obj  V.34 module object
 * @param C    the three DFT bins
 * @note  Called after the state-1 / state-2 bookkeeping of v04_rx_dpsk_dft().  Returns at once while rtDetHits_ != rtDetThrB_ (an exact compare, not
 *        >=).  On a match it ends the search for INFO1: it re-arms the DFT records, resets the detector, and moves the handshake to
 *        microstate TX_PHASE1_ANS / receive state RX_DPSK / transmit state TONE_AB (each change is printed at debug level above 1).
 */
static void checkRetrain(tagV34Object* obj, DftBin* C)
{
  uint8_t* const u = (uint8_t*)&obj->wa248_; /* byte region of the detector variables, starting at wa248_ */
  unsigned m, rx, tx;
  int i;

  if (ld16(u, U_(rtDetHits_)) != ld16(u, U_(rtDetThrB_)))
    return;
  if (dsplibs_debug_level > 1u)
    dsplibs_debug_printf("DET_SYNC : retrain request detected while searching for info1\n");

  /* stop the search and re-arm the three records with their initial limits and thresholds */
  obj->dftActive_ = 0;
  for (i = 0; i < 3; i++)
  {
    C[i].quietLimit = 0x50;
    C[i].toneLimit = 0xbb8;
    C[i].phase = 0;
    C[i].accI = 0;
    C[i].accQ = 0;
  }
  C[0].step = 0x600;
  C[1].step = 0x800;
  C[2].step = 0xa00;
  m = obj->mst_;

  /* reset the detector */
  st16(u, U_(rtDetState_), 1);
  st16(u, U_(rtDetHits_), 0);
  st16(u, U_(rtDetThrB_), 9);
  st32(u, U_(rtDetCnt_), 0);
  st16(u, U_(rtDetThrA_), 3);
  obj->w3588_ = (uint16_t)(obj->w3588_ | 2u);

  /* hand over to the answering-side phase 1 transmitter: microstate, receive state and transmit state */
  if ((uint16_t)m != V34HS_TX_PHASE1_ANS)
  {
    if (dsplibs_debug_level > 1u)
      v04PrMstX(m, V34HS_TX_PHASE1_ANS, obj->txState_, obj->rxState_, v04K56(obj), v04W78(obj));
    obj->mst_ = V34HS_TX_PHASE1_ANS;
  }
  rx = obj->rxState_;
  if ((uint16_t)rx != V34HS_RX_DPSK)
  {
    if (dsplibs_debug_level > 1u)
      dsplibs_debug_printf(V04_FMT_RX, StateName[(int16_t)rx], StateName[V34HS_RX_DPSK], StateName[(int16_t)obj->txState_], StateName[(int16_t)obj->mst_],
                           v04K56(obj), v04W78(obj));
    obj->rxState_ = V34HS_RX_DPSK;
  }
  tx = obj->txState_;
  if ((uint16_t)tx != V34HS_TONE_AB)
  {
    if (dsplibs_debug_level > 1u)
      v04PrTx(obj, tx, V34HS_TONE_AB);
    obj->txState_ = V34HS_TONE_AB;
  }

  if (obj->toneSel_ > 1u)
    obj->toneSel_ = 0;
  obj->tx.k56Idx_ = 0;
  obj->hsSymCnt_ = 0;
  obj->v21RxBuf_ = USHRT_MAX;
  obj->v21BitCnt_ = 0;
}

/**
 * @brief Feed the retrain-request detector with one block of demodulated samples and, every 32nd call, evaluate it.
 * @param obj  V.34 module object; reads rxFskBuf_ and the detector region, may clear dftActive_ and change the three state words
 * @note  Call only while dftActive_ != 0.  Between evaluations it only updates the DFT records and the call counter.
 */
extern "C" void v04_rx_dpsk_dft(tagV34Object* obj)
{
  uint8_t* const u = (uint8_t*)&obj->wa248_; /* byte region of the detector variables, starting at wa248_ */
  DftBin* const C = obj->dftC_;
  uint32_t dftCnt;
  int i;

  /* accumulate this block; only every 32nd call (counter 0x80) goes on to evaluate */
  dftupdate(C, 3, (const short*)obj->rx.rxFskBuf_, 4);
  dftCnt = ld32(u, U_(rtDetCnt_)) + 4u;
  if (dftCnt != 0x80u)
  {
    st32(u, U_(rtDetCnt_), dftCnt);
    return;
  }

  /* evaluation: compute the energies, then clear the accumulators for the next interval */
  st32(u, U_(rtDetCnt_), 0);
  dftenergy(C, 3, 5);
  for (i = 0; i < 3; i++)
  {
    C[i].phase = 0;
    C[i].accI = 0;
    C[i].accQ = 0;
  }
  {
    int state = (int16_t)ld16(u, U_(rtDetState_));

    /* state 1: counting quiet intervals (all three energies below their limits) */
    if (state == 1)
    {
      unsigned e0 = (uint16_t)C[0].energyHi;
      int l0 = C[0].quietLimit;
      unsigned e1 = (uint16_t)C[1].energyHi;
      int l1 = C[1].quietLimit;
      unsigned e2 = (uint16_t)C[2].energyHi;
      int l2 = C[2].quietLimit;
      if ((int)e0 < l0 && (int)e1 < l1 && (int)e2 < l2)
      {
        st16(u, U_(rtDetHits_), ld16(u, U_(rtDetHits_)) + 1u);
        return;
      }
      if ((int16_t)ld16(u, U_(rtDetHits_)) >= (int16_t)ld16(u, U_(rtDetThrA_)))
        st16(u, U_(rtDetState_), 2);
      st16(u, U_(rtDetHits_), 0);
      return;
    }

    /* state 2: counting intervals in which the middle energy is above its limit */
    if (state != 2)
      return;
    {
      unsigned energy = (uint16_t)C[1].energyHi;
      int l = C[1].toneLimit;
      if ((int)energy <= l)
      {
        st16(u, U_(rtDetHits_), 0);
        st16(u, U_(rtDetState_), 1);
      }
      else
      {
        st16(u, U_(rtDetHits_), ld16(u, U_(rtDetHits_)) + 1u);
      }
    }

    checkRetrain(obj, C);
  }
}
