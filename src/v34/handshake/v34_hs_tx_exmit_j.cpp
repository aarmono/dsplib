/**
 * @file v34_hs_tx_exmit_j.cpp
 * @brief V.34 handshake, transmit states EXMIT (E sequence) and JTXMIT / J1TXMIT (J sequence).
 *
 * Handlers called from the per-sample transmit loop of v34handshak() (v34_hs_dispatch.cpp); each call produces one constellation point (or two) with
 * txmit().  EXMIT and the two J states are scrambled bit-pair sequences; the scrambler taps are chosen by txFlags_ bit 0 (set: bit 26, clear: bit 13)
 * in both (TXLEVEL / TXMD choose by role instead).  JTXMIT feeds the scrambler with the 2-bit group (k56Word_ >> (2 * k56Idx_ mod 32)), EXMIT with the
 * constant 3; the shift count is masked to 5 bits like the machine shift.
 *
 * - v04_tx_exmit(): E sequence.  With the 4-point signature (rxSig_ != 0x89b0) one scrambled bit pair per call (k56Idx_ += 2), with the 16-point
 *   signature two (k56Idx_ += 4).  At k56Idx_ == 0x14 the state goes to DATAXMIT.
 * - v04_tx_jtxmit(): J / J1 sequence.  A freeze countdown in hsSymCnt_ freezes the echo cancellers when it reaches zero (the two coefficient reports);
 *   the answering role leaves for XMIT0, the calling role goes JTXMIT -> J1TXMIT -> TRNSEG4A.
 *
 * The debug level is read at fixed points; the echo freeze keeps one copy that is refreshed after each print.
 */

#include "v34_handshake.h"
#include "v34/v34_internal.h"

extern "C"
{
}

/**
 * @brief Run one scrambler step pair and form the differentially encoded dibit.
 * @param obj    V.34 module object; scrState_ is advanced
 * @param tap13  true: feedback tap at bit 13, false: bit 26 (plus bit 8 always)
 * @param seed   input bits for the two steps
 * @return (top three bits of the new register + txPrev_) & 3
 */
static unsigned nextBits(tagV34Object* obj, bool tap13, int seed)
{
  uint32_t e = v04Scramble(obj->tx.scrState_, tap13, seed);
  obj->tx.scrState_ = e;
  return ((e >> 29) + (unsigned)obj->tx.txPrev_) & 3u;
}

/**
 * @brief Transmit state EXMIT (0x45): one call of the E sequence.
 * @param obj  V.34 module object
 * @note  4-point signature (rxSig_ != 0x89b0): one scrambled bit pair per call (k56Idx_ += 2); 16-point signature: two (k56Idx_ += 4).  When k56Idx_
 *        reaches 0x14 it is reset to 8 and the state goes to DATAXMIT.
 */
extern "C" void v04_tx_exmit(tagV34Object* obj)
{
  /* 16-point signature: two dibits, the second indexes vect16 together with the first */
  if (obj->rx.rxSig_ == 0x89b0)
  {
    unsigned fl = obj->tx.txFlags_;
    const bool tap13 = (fl & 1u) == 0;
    unsigned dibit = nextBits(obj, tap13, 0xf); /* seed 0xf: only the low two bits matter for two steps */
    unsigned prevBits;
    uint32_t e2;
    obj->tx.txBits_ = (uint16_t)dibit;
    e2 = v04Scramble(obj->tx.scrState_, tap13, 3);
    obj->tx.scrState_ = e2;
    prevBits = obj->tx.txBits_;
    obj->tx.txPrev_ = (uint16_t)prevBits;
    obj->tx.txPoint_ = (uint32_t)vect16[(e2 >> 29) + (unsigned)((int16_t)prevBits) * 4u];
    txmit(obj);
    obj->tx.k56Idx_ = (uint16_t)(obj->tx.k56Idx_ + 4u);
  }
  else
  {
    /* 4-point signature: one dibit */
    const bool tap13 = (obj->tx.txFlags_ & 1u) == 0;
    unsigned dibit = nextBits(obj, tap13, 3);
    obj->tx.txBits_ = (uint16_t)dibit;
    obj->tx.txPrev_ = (uint16_t)dibit;
    obj->tx.txPoint_ = (uint32_t)vect4[dibit];
    txmit(obj);
    obj->tx.k56Idx_ = (uint16_t)(obj->tx.k56Idx_ + 2u);
  }

  /* end of the E sequence: on to DATAXMIT */
  if (obj->tx.k56Idx_ == 0x14)
  {
    unsigned tx = obj->txState_;
    unsigned lv;
    obj->tx.k56Idx_ = 8;
    lv = dsplibs_debug_level;
    if ((uint16_t)tx != V34HS_DATAXMIT)
    {
      if (lv > 1u)
      {
        v04PrTx(obj, tx, V34HS_DATAXMIT);
        lv = dsplibs_debug_level;
      }
      obj->txState_ = V34HS_DATAXMIT;
    }
    if (lv > 1u)
      dsplibs_debug_printf("V34MP- E transmit completed\n");
  }
}

/**
 * @brief Freeze the echo cancellers: set txFlags_ bit 2 and report the near and far coefficients.
 * @param obj  V.34 module object
 * @param lv   copy of dsplibs_debug_level taken by the caller (gates the first banner only; the second banner re-reads the level)
 */
static void echoReports(tagV34Object* obj, unsigned lv)
{
  obj->tx.txFlags_ = (uint16_t)(obj->tx.txFlags_ | 4u);
  if (lv > 1u)
    dsplibs_debug_printf("==== Near Echo Canceller report ======\n");
  V34EchoReportCoeff(&obj->echo0);
  if (dsplibs_debug_level > 1u)
    dsplibs_debug_printf("==== Far Echo Canceller report ======\n");
  V34EchoReportCoeff(&obj->echo1);
}

/**
 * @brief Transmit states JTXMIT (0x40) / J1TXMIT (0x44): one symbol of the J sequence.
 * @param obj  V.34 module object
 * @note  Sends two bits per symbol of the 16-bit word k56Word_ (eight symbols per word, k56Idx_ counts modulo 8).  A non-zero hsSymCnt_ counts down
 *        once per call and freezes the echo cancellers when it reaches zero.  The answering role leaves for XMIT0 once rxFlags_ bit 0x400 is set or
 *        both rxMdLen_ and w25da_ are non-zero.  The calling role, at the end of a word: J1TXMIT goes to TRNSEG4A (scrambler cleared); JTXMIT with
 *        w25da_ == 2 and symCnt_ above 0x80 loads k56Word_ = 0x899f and goes to J1TXMIT, forcing the echo freeze if a countdown is pending.
 */
extern "C" void v04_tx_jtxmit(tagV34Object* obj)
{
  uint16_t freezeCnt = obj->hsSymCnt_;
  unsigned fl;
  unsigned dibit;
  unsigned idxOld, idxNext;
  int bits;

  /* freeze countdown */
  if (freezeCnt != 0)
  {
    freezeCnt = (uint16_t)(freezeCnt - 1u);
    obj->hsSymCnt_ = freezeCnt;
    if (freezeCnt == 0)
    {
      unsigned lv = dsplibs_debug_level;
      if (lv > 1u)
      {
        dsplibs_debug_printf("V34Hshak: on JTXMIT - time to freeze echo...\r\n");
        lv = dsplibs_debug_level;
        if (lv > 1u)
        {
          dsplibs_debug_printf("V34HSHAK: Freeze EC\n");
          lv = dsplibs_debug_level;
        }
      }
      echoReports(obj, lv);
    }
  }

  /* the next two bits of k56Word_ drive the scrambler */
  obj->tx.symCnt_ = (uint16_t)(obj->tx.symCnt_ + 1u);
  {
    unsigned w = obj->tx.k56Word_;
    int sh = (int16_t)obj->tx.k56Idx_ * 2;
    bits = (int16_t)(w >> (sh & 31));
  }
  fl = obj->tx.txFlags_;
  dibit = nextBits(obj, (fl & 1u) == 0, bits);
  obj->tx.txBits_ = (uint16_t)dibit;
  obj->tx.txPrev_ = (uint16_t)dibit;
  obj->tx.txPoint_ = (uint32_t)vect4[dibit];
  txmit(obj);

  idxOld = obj->tx.k56Idx_;
  idxNext = (idxOld + 1u) & 7u;
  obj->tx.k56Idx_ = (uint16_t)idxNext;

  /* answering role: leave for XMIT0 when the far end is ready */
  if (obj->modeChar_ == V34ROLE_ANSWER)
  {
    if ((obj->rx.rxFlags_ & 0x400u) == 0)
    {
      if (obj->rxMdLen_ == 0)
        return;
      if (obj->tx.w25da_ == 0)
        return;
    }
    v04SetTx(obj, obj->txState_, V34HS_XMIT0);
    obj->tx.k56Idx_ = 0;
    return;
  }

  /* calling role: act only at the end of a word */
  if (idxNext != 0)
    return;
  {
    unsigned tx = obj->txState_;
    if ((uint16_t)tx == V34HS_J1TXMIT)
    {
      if (dsplibs_debug_level > 1u)
        v04PrTx(obj, V34HS_J1TXMIT, V34HS_TRNSEG4A);
      obj->tx.scrState_ = 0;
      obj->txState_ = V34HS_TRNSEG4A;
      obj->tx.txPrev_ = 0;
      obj->tx.txCnt_ = 0;
      return;
    }
  }

  if (obj->tx.w25da_ != 2)
    return;
  if ((int16_t)obj->tx.symCnt_ <= 0x80)
    return;

  /* JTXMIT -> J1TXMIT with the fixed J1 word */
  obj->tx.k56Word_ = 0x899f;
  v04SetTx(obj, obj->txState_, V34HS_J1TXMIT);
  freezeCnt = obj->hsSymCnt_;
  if (freezeCnt == 0)
    return;

  /* a pending countdown: force the echo freeze now */
  {
    unsigned lv = dsplibs_debug_level;
    if (lv > 1u)
    {
      dsplibs_debug_printf("V34Hshak: on J1TXMIT - forced freeze echo (count2 = %d)...\r\n", (int)(int16_t)freezeCnt);
      lv = dsplibs_debug_level;
      if (lv > 1u)
      {
        dsplibs_debug_printf("V34HSHAK: Freeze EC\n");
        lv = dsplibs_debug_level;
      }
    }
    obj->tx.txFlags_ = (uint16_t)(obj->tx.txFlags_ | 4u);
    if (lv > 1u)
      dsplibs_debug_printf("==== Near Echo Canceller report ======\n");
    V34EchoReportCoeff(&obj->echo0);
    if (dsplibs_debug_level > 1u)
      dsplibs_debug_printf("==== Far Echo Canceller report ======\n");
    V34EchoReportCoeff(&obj->echo1);
    obj->hsSymCnt_ = 0; /* cleared after the reports */
  }
}
