/**
 * @file v34_hs_rx_l1.cpp
 * @brief V.34 handshake receive state RX_L1: reception of the L1 probing signal.
 *
 * Entry point v04_rx_l1(), called by the post-loop receive chain of v34handshak() (v34_hs_dispatch.cpp) when the receive state is RX_L1; the caller
 * runs the transmit-state tail afterwards.  The handler needs nothing from the dispatcher.
 *
 * hsSymCnt_ counts the symbols.  While it is <= 0x440 (the probing proper) rlEarly() runs rxtiming() and feeds two DFT arrays
 * (A = the 25 bins `probe.bin_`, B = the 4 bins `dftB_`; DftBin records) with dftupdate() at these symbol counts:
 *  - 0x40: initialise both arrays and the receive gain estimate (rxGainEst_ = 2 * rxGain_, or 0x6000 if the gain is above 0x34ff);
 *  - 0x41 .. 0x17f and 0x1c1 .. 0x2ff: update both arrays (the first range also counts in w388_);
 *  - 0x180: update, evaluate (dftenergy), store the energy sums and their rounded quotient, then re-initialise both arrays;
 *  - 0x300: update, evaluate, store the sums and quotient, re-initialise the 25 records of A with the record index in word 1;
 *  - above 0x300: update A only.
 * Above 0x440 the receiver gain is controlled (V34agc) and the roles differ.  The answering role (not V34ROLE_CALL) sets up the demodulator for
 * 2400 baud, moves to receive state RX_DPSK / transmit state TONE_AB / microstate DET_SYNC, clears the INFO word area, arms INFO1c and sets
 * dftActive_.  The calling role sets up the same demodulator at 0x441, then waits for the end of L2 with fskdemodulate(): INFO0 flags with
 * v21BitCnt_ > 0x14 and (v21RxBuf_ & 0xfff) == 0 move to RX_DPSK / microstate RX_PHASE3_CALL, otherwise the timers of rlTimers() flip toneSel_
 * (tone A wait) or restart L1.
 */

#include <limits.h>
#include "v34_handshake.h"
#include "core/dsp_access.h"
#include "v34/v34_internal.h"

extern "C"
{
}

#include "v34_hs_bytes.h"

/** Print a receive-state transition; the transmit state and microstate names are read from the object at print time. */
static inline void rlPrRx(tagV34Object* obj, unsigned old, int nw)
{
  dsplibs_debug_printf(V04_FMT_RX, StateName[(int16_t)old], StateName[nw], StateName[(int16_t)obj->txState_], StateName[(int16_t)obj->mst_], v04K56(obj),
                       v04W78(obj));
}
/** Print a microstate transition; the transmit and receive state names are read from the object at print time. */
static inline void rlPrMst(tagV34Object* obj, unsigned old, int nw) { v04PrMstX(old, nw, obj->txState_, obj->rxState_, v04K56(obj), v04W78(obj)); }

/* the two DFT arrays of the receiver (A: the 25 bins of the probe block, B: the 4 bins `dftB_`) */
static inline DftBin* dftA(tagV34Object* obj) { return obj->probe.bin_; }
static inline DftBin* dftB(tagV34Object* obj) { return obj->dftB_; }

/** Clear the phase, step and integer accumulators of the first four bins of a DFT array. */
static void clear4(DftBin* base)
{
  int i;
  for (i = 0; i < 4; i++)
  {
    base[i].phase = 0;
    base[i].step = 0;
    base[i].accI = 0;
    base[i].accQ = 0;
  }
}

/**
 * @brief Sum the energies of the first four records of both DFT arrays.
 * @param obj  V.34 module object
 * @param s2   receives the sum of the `energyHi` words of A, each shifted left by 8
 * @return the sum of the `energyHi` words of B
 */
static uint32_t energySums(tagV34Object* obj, uint32_t* s2)
{
  uint32_t s1 = 0, t = 0;
  int i;
  for (i = 0; i < 4; i++)
  {
    s1 += (uint16_t)dftB(obj)[i].energyHi;
    t += (uint32_t)(uint16_t)dftA(obj)[i].energyHi << 8;
  }
  *s2 = t;
  return s1;
}

/**
 * @brief RX_L1 work for symbol counts up to 0x440: timing recovery and the DFT accumulation schedule described in the file header.
 * @param obj  V.34 module object
 * @note  The sample count given to dftupdate() is (rxEnd_ - rxFskBuf_) / 2 as a signed 16-bit value; the pointer difference is taken modulo 2^32.
 */
static void rlEarly(tagV34Object* obj)
{
  unsigned symCnt;
  int cnt;
  DftBin* const A = dftA(obj);
  DftBin* const B = dftB(obj);

  rxtiming(obj);
  symCnt = obj->hsSymCnt_;
  cnt = (int16_t)((int32_t)((uint32_t)(uintptr_t)obj->rx.rxEnd_ - (uint32_t)(uintptr_t)obj->rx.rxFskBuf_) >> 1); /* 32-bit pointer difference */

  /* above 0x300: only array A is kept running */
  if ((int16_t)symCnt > 0x300)
  {
    dftupdate(A, 0x19, (const short*)obj->rx.rxFskBuf_, cnt);
    return;
  }

  /* 0x300: final update, evaluate, store the energy sums and quotient, re-initialise the 25 records of A */
  if ((int16_t)symCnt == 0x300)
  {
    uint32_t s1, s2;
    int k;

    dftupdate(A, 4, (const short*)obj->rx.rxFskBuf_, cnt);
    dftupdate(B, 4, (const short*)obj->rx.rxFskBuf_, cnt);
    dftenergy(B, 4, 6);
    dftenergy(A, 4, 2);

    obj->waabc_ = 0;
    obj->waac0_ = 0;
    s1 = energySums(obj, &s2);
    obj->waac0_ = s2;
    obj->waabc_ = s1;
    obj->waac8_ = s1 == 0 ? 0u : (s2 + (s1 >> 1)) / s1;

    for (k = 1; k <= 0x19; k++)
    {
      DftBin* e = &A[k - 1];
      e->phase = 0;
      e->accI = 0;
      e->step = (uint16_t)((unsigned)k << 8);
      e->sumI = 0.0;
      e->sumQ = 0.0;
      e->energy = 0.0;
      e->accQ = 0;
    }
    return;
  }

  /* 0x1c1 .. 0x2ff: update both arrays */
  if ((uint16_t)(symCnt - 0x1c1u) <= 0x13eu)
  {
    dftupdate(A, 4, (const short*)obj->rx.rxFskBuf_, cnt);
    dftupdate(B, 4, (const short*)obj->rx.rxFskBuf_, cnt);
    return;
  }

  /* 0x180: update, evaluate, store the energy sums and quotient, then re-initialise both arrays with the limits for the second half */
  if ((uint16_t)symCnt == 0x180)
  {
    uint32_t s1, s2;

    dftupdate(A, 4, (const short*)obj->rx.rxFskBuf_, cnt);
    dftupdate(B, 4, (const short*)obj->rx.rxFskBuf_, cnt);
    dftenergy(B, 4, 6);
    dftenergy(A, 4, 2);

    obj->waab4_ = 0;
    obj->waab8_ = 0;
    s1 = energySums(obj, &s2);
    obj->waab8_ = s2;
    obj->waab4_ = s1;
    obj->waac4_ = s1 == 0 ? 0u : (s2 + (s1 >> 1)) / s1;

    clear4(A);
    A[0].step = 0x700;
    A[1].step = 0x900;
    A[2].step = 0xd00;
    A[3].step = 0x1100;
    clear4(B);
    B[0].step = 0x600;
    B[1].step = 0x800;
    B[2].step = 0xc00;
    B[3].step = 0x1000;
    return;
  }

  /* 0x41 .. 0x17f: update both arrays and count the interval in w388_ */
  if ((uint16_t)(symCnt - 0x41u) <= 0x13eu)
  {
    dftupdate(A, 4, (const short*)obj->rx.rxFskBuf_, cnt);
    dftupdate(B, 4, (const short*)obj->rx.rxFskBuf_, cnt);
    obj->rx.w388_ = (uint16_t)(obj->rx.w388_ + 1u);
    return;
  }

  /* 0x40: start of the probing: mark the receiver, set the gain estimate, initialise both arrays */
  if ((uint16_t)symCnt == 0x40)
  {
    unsigned fl = obj->rx.rxFlags_;
    unsigned gain = (uint16_t)obj->rx.rxGain_;
    unsigned level;

    obj->rx.rxFlags_ = (uint16_t)(fl | 0x200u);
    if ((int16_t)gain > 0x34ff)
    {
      level = dsplibs_debug_level;
      if (level > 1u)
      {
        dsplibs_debug_printf("V34AGC, -- ERROR-- gainestimate in RX_L1,0x%x\n", (int)(int16_t)gain);
        level = dsplibs_debug_level;
      }
      obj->rx.rxGainEst_ = 0x6000;
    }
    else
    {
      level = dsplibs_debug_level;
      obj->rx.rxGainEst_ = (int16_t)(gain + gain);
    }
    if (level > 1u)
      dsplibs_debug_printf("V34AGC, rx->gain = 0x%x, at the beginning of RX_L1\n", (int)(int16_t)obj->rx.rxGain_);

    clear4(A);
    A[0].step = 0x700;
    A[3].step = 0x1100;
    A[1].step = 0x900;
    A[2].step = 0xd00;
    clear4(B);
    B[0].step = 0x600;
    B[1].step = 0x800;
    B[2].step = 0xc00;
    B[3].step = 0x1000;
    return;
  }
}

/**
 * @brief Demodulator set-up for 2400 baud, shared by both roles at the end of L1.
 * @param obj  V.34 module object
 * @note  Stores the 2400-baud words (w38c_, w410_, w412_, w414_, w422_), sets receiver flag 0x800 and restores rxGain_ from rxGainSaved_.
 */
static void rlSetup2400(tagV34Object* obj)
{
  unsigned fl = obj->rx.rxFlags_;
  unsigned gain = obj->rx.rxGainSaved_;
  obj->rx.w38c_ = 4;
  obj->rx.w414_ = 0x3e80;
  obj->rx.w412_ = 0x3e80;
  obj->rx.w422_ = 0x3e80;
  obj->rx.w410_ = 0x1f40;
  obj->rx.rxFlags_ = (uint16_t)(fl | 0x800u);
  obj->rx.rxGain_ = (int16_t)gain;
}

/**
 * @brief Calling role, INFO0 flags not complete: the end-of-L2 timers.
 * @param obj  V.34 module object
 * @note  Nothing happens until hsSymCnt_ passes bulkDelay_/4 + 0xf10.  After that v21RxBuf_ is reset each call; at +0xf14 toneSel_ is flipped
 *        (waiting for tone A) and at +0xf2c the count is cleared and the handshake goes back to RX_DPSK / microstate TX_L1 (L1 restarts).
 */
static void rlTimers(tagV34Object* obj)
{
  int bulkQuarter = (int)(int16_t)obj->bulkDelay_ >> 2;
  int symCnt = (int16_t)obj->hsSymCnt_;

  if (symCnt <= bulkQuarter + 0xf10)
    return;
  obj->v21RxBuf_ = USHRT_MAX;
  if (symCnt == bulkQuarter + 0xf14)
  {
    unsigned c = obj->toneSel_ ^ 1u;
    obj->toneSel_ = (uint16_t)c;
    if (dsplibs_debug_level > 1u)
      dsplibs_debug_printf("V34RETRAIN, waiting for tone A at the end of RX_L2\n");
    return;
  }
  if (symCnt == bulkQuarter + 0xf2c)
  {
    unsigned rx, m;
    obj->hsSymCnt_ = 0;
    rx = obj->rxState_;
    if ((uint16_t)rx != V34HS_RX_DPSK)
    {
      if (dsplibs_debug_level > 1u)
        dsplibs_debug_printf(V04_FMT_RX, StateName[(int16_t)rx], StateName[V34HS_RX_DPSK], StateName[(int16_t)obj->txState_], StateName[(int16_t)obj->mst_],
                             v04K56(obj), 0L);
      obj->rxState_ = V34HS_RX_DPSK;
    }
    m = obj->mst_;
    if ((uint16_t)m != V34HS_TX_L1)
    {
      if (dsplibs_debug_level > 1u)
        rlPrMst(obj, m, V34HS_TX_L1);
      obj->mst_ = V34HS_TX_L1;
    }
  }
}

/**
 * @brief Process one symbol interval of the RX_L1 receive state.
 * @param obj  V.34 module object
 * @note  Counts hsSymCnt_ (16-bit, signed compare against 0x440).  See the file header for the per-count schedule and the two roles.  State changes
 *        are printed at debug level above 1.
 */
extern "C" void v04_rx_l1(tagV34Object* obj)
{
  unsigned w = (uint16_t)(obj->hsSymCnt_ + 1u);
  obj->hsSymCnt_ = (uint16_t)w;

  /* probing proper */
  if ((int16_t)w <= 0x440)
  {
    rlEarly(obj);
    return;
  }

  V34agc(&obj->rx);

  if (obj->modeChar_ != V34ROLE_CALL)
  {
    /* answering role: demodulator set-up, RX_DPSK / TONE_AB / DET_SYNC, the INFO area cleared, INFO1c armed */
    unsigned stateCur, mstCur;
    int i;

    obj->waacc_ = obj->rx.w49c_;
    dftenergy(dftA(obj), 0x19, 2);
    if (dsplibs_debug_level > 1u)
      dsplibs_debug_printf("V34SetupDemodulator: baudrate %ld, carrier %ld\n", 0x960L, 0L);
    rlSetup2400(obj);

    stateCur = obj->rxState_;
    if ((uint16_t)stateCur != V34HS_RX_DPSK)
    {
      if (dsplibs_debug_level > 1u)
        rlPrRx(obj, stateCur, V34HS_RX_DPSK);
      obj->rxState_ = V34HS_RX_DPSK;
    }
    stateCur = obj->txState_;
    if ((uint16_t)stateCur != V34HS_TONE_AB)
    {
      if (dsplibs_debug_level > 1u)
        v04PrTx(obj, stateCur, V34HS_TONE_AB);
      obj->txState_ = V34HS_TONE_AB;
    }
    mstCur = obj->mst_;
    if ((uint16_t)mstCur != V34HS_DET_SYNC)
    {
      if (dsplibs_debug_level > 1u)
        rlPrMst(obj, mstCur, V34HS_DET_SYNC);
      obj->mst_ = V34HS_DET_SYNC;
    }

    /* clear the INFO word area and restart the FSK receiver */
    for (i = 0; i <= 0x63; i++)
      st16((uint8_t*)&obj->waae6_, 2 * i, 0);
    obj->fskTapIdx_ = 0x30;
    obj->fskPeriod_ = 0xc;
    obj->fskReload_ = 6;
    obj->fskBias_ = 0;
    obj->fskPatLow_ = 1;
    obj->fskPatHigh_ = 0;
    obj->fskNextPoint_ = 6;
    obj->fskCounter_ = 0;
    obj->waae4_ = 0;
    obj->v21RxBuf_ = USHRT_MAX;
    obj->v21BitCnt_ = 0;

    /* arm INFO1c as the bit queue and switch on the retrain-request detector */
    obj->info1c.i1cTail_.total = 0x4d;
    obj->tx.k56Idx_ = 0;
    obj->dftActive_ = 1;
    obj->bitQ2_ = (tag_bitQ*)&obj->info1c;
    obj->hsSymCnt_ = 0;
    return;
  }

  /* calling role: first symbol past the probing, set up the 2400 baud demodulator */
  if (obj->hsSymCnt_ == 0x441)
  {
    unsigned txCur;

    obj->waacc_ = obj->rx.w49c_;
    dftenergy(dftA(obj), 0x19, 2);
    if (dsplibs_debug_level > 1u)
      dsplibs_debug_printf("V34SetupDemodulator: baudrate %ld, carrier %ld\n", 0x960L, 0L);
    rlSetup2400(obj);

    txCur = obj->txState_;
    if ((uint16_t)txCur != V34HS_TONE_AB)
    {
      if (dsplibs_debug_level > 1u)
        v04PrTx(obj, txCur, V34HS_TONE_AB);
      obj->txState_ = V34HS_TONE_AB;
    }
    obj->v21RxBuf_ = USHRT_MAX;
    obj->v21BitCnt_ = 0;
    return;
  }

  /* calling role: wait for the end of L2 */
  fskdemodulate(obj, (const uint16_t*)obj->rx.rxFskBuf_, obj->fskState_);
  if ((int16_t)obj->v21BitCnt_ <= 0x14 || (obj->v21RxBuf_ & 0xfffu) != 0)
  {
    rlTimers(obj);
    return;
  }

  /* INFO0 flags received: move on to RX_DPSK / microstate RX_PHASE3_CALL */
  {
    unsigned rx = obj->rxState_;
    unsigned mstCur;

    if ((uint16_t)rx != V34HS_RX_DPSK)
    {
      if (dsplibs_debug_level > 1u)
        rlPrRx(obj, rx, V34HS_RX_DPSK);
      obj->rxState_ = V34HS_RX_DPSK;
    }
    mstCur = obj->mst_;
    if ((uint16_t)mstCur == V34HS_RX_PHASE3_CALL)
      return;
    if (dsplibs_debug_level > 1u)
      rlPrMst(obj, mstCur, V34HS_RX_PHASE3_CALL);
    obj->mst_ = V34HS_RX_PHASE3_CALL;
  }
}
