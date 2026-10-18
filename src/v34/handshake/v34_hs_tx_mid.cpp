/**
 * @file v34_hs_tx_mid.cpp
 * @brief V.34 handshake, four transmit states of the per-sample loop: TX_L1, SBARSEG, PPSEG and TXMD.
 *
 * Handlers called from v34_hs_dispatch.cpp, one call per transmitted symbol (TX_L1: one call per four samples):
 * - v04_tx_txl1(): the L1 probe tone.  Four samples of the 64-entry `probe` table, scaled by txGain_ (doubled in the TX_L1 microstate), go to the
 *   transmit queue with txwritequeue(); after 0x600 samples in that microstate it moves to TX_L2.
 * - v04_tx_sbarseg(): the Sbar segment.  Two points per call; after 8 calls it chooses the successor: TRNSEG4A (txFlags_ bit 13), PPSEG (txMdMs_ == 0 or
 *   txFlags_ negative) or the set-up of TXMD.
 * - v04_tx_ppseg(): the PP sequence, 48 points of `vectpp` per pass; six passes later it derives the echo start delay (hsSymCnt_) and the symbol
 *   period (period_) from the symbol rate and goes to TRNSEG4.
 * - v04_tx_txmd(): the scrambled MD sequence; hsSymCnt_ symbols into it echo adaptation is enabled, txMdSymbols_ symbols into it the transmitter is set
 *   up again for S / Sbar (state SSEG).
 *
 * The 16-bit counters wrap at 16 bits.  The division by 9600 is a signed C division.  The debug level is read at fixed points (PPSEG keeps one copy that is
 * refreshed only after its transition print).
 */

#include "v34_handshake.h"
#include "v34/v34_internal.h"
#include "v34/v34_receiver_tables.h" /* vectpp */
#include "v34_hs_tables.h"           /* probe */

/**
 * @brief Two steps of the 23-bit transmit scrambler with the constant input bits 3 (shared by TXLEVEL / TXMD).
 * @param reg   scrambler register
 * @param role  true: calling role (feedback tap at bit 13), false: answering role (bit 26); bit 8 is always a tap
 * @return the register after the second step
 */
static uint32_t scrambleTwo(uint32_t reg, bool role)
{
  int inBits = 3;
  int step = 0;
  uint32_t next;
  do
  {
    int feedback = (inBits & 1) != 0 ? 1 : 0;
    inBits >>= 1;
    if (role)
    {
      if ((reg & 0x2000u) != 0)
        feedback = (int16_t)(feedback + 1);
    }
    else
    {
      if ((reg & 0x4000000u) != 0)
        feedback = (int16_t)(feedback + 1);
    }
    if ((reg & 0x100u) != 0)
      feedback = (int16_t)(feedback + 1);
    if ((feedback & 1) != 0)
      reg |= 0x80000000u;
    next = reg >> 1;
    step = (int16_t)(step + 1);
    reg = next;
  } while (step < 2);
  return next;
}

/** @brief Common tail of the SBARSEG transitions: clear the symbol counter k56Idx_ and the pass counter w358e_. */
static void sbarsegTail(tagV34Object* obj)
{
  obj->tx.k56Idx_ = 0;
  obj->w358e_ = 0;
}

/**
 * @brief Transmit state TX_L1 (0x33): send four samples of the L1 probe tone.
 * @param obj  V.34 module object
 * @note  k56Idx_ is the index into the 64-entry `probe` table (wrapped with & 0x3f); each sample is probe * txGain_ >> 14, doubled in the TX_L1 microstate.
 *        After 0x600 samples in that microstate it becomes TX_L2 and k56Idx_ restarts at 0.
 */
extern "C" void v04_tx_txl1(tagV34Object* obj)
{
  short buf[4];
  int i = 0;
  const bool dbl = (obj->mst_ == V34HS_TX_L1);
  do
  {
    unsigned idx = obj->tx.k56Idx_;
    int gain = (int16_t)obj->tx.txGain_;
    int s = probe[idx & 0x3fu];
    if (dbl)
      s = (int16_t)(s + s);
    buf[i] = (short)((int32_t)((uint32_t)s * (uint32_t)gain) >> 14);
    obj->tx.k56Idx_ = (uint16_t)(idx + 1u);
    i = (int16_t)(i + 1);
  } while (i <= 3);
  txwritequeue(&obj->tx, buf);

  if (obj->mst_ == V34HS_TX_L1)
  {
    if (obj->tx.k56Idx_ == 0x600)
    {
      if (dsplibs_debug_level > 1u)
        dsplibs_debug_printf(V04_FMT_MST, StateName[V34HS_TX_L1], StateName[V34HS_TX_L2], StateName[(int16_t)obj->txState_], StateName[(int16_t)obj->rxState_],
                             0x600L, (long)(int16_t)obj->hsSymCnt_);
      obj->mst_ = V34HS_TX_L2;
      obj->tx.k56Idx_ = 0;
    }
  }
}

/**
 * @brief Transmit state SBARSEG (0x13): send the two Sbar points, and after 8 calls choose the successor.
 * @param obj  V.34 module object
 * @note  Successor: TRNSEG4A if txFlags_ bit 13 is set; else PPSEG if txMdMs_ == 0, or if txFlags_ is negative (then txCnt_ is loaded from
 *        txMdSymbols_); else TXMD, with txMdSymbols_ = txMdMs_ * 0x53, the transmit modulator set to the all-pass source at carrier 2400 and the echo
 *        start delay computed into hsSymCnt_ (debug print).
 */
extern "C" void v04_tx_sbarseg(tagV34Object* obj)
{
  uint16_t n;
  obj->tx.txPoint_ = (uint32_t)vect4[2];
  txmit(obj);
  obj->tx.txPoint_ = (uint32_t)vect4[1];
  txmit(obj);

  n = (uint16_t)(obj->tx.txCnt_ + 1u);
  if (n != 8)
  {
    obj->tx.txCnt_ = n;
    return;
  }
  obj->tx.txCnt_ = 0;

  /* choose the successor */
  {
    unsigned fl = obj->tx.txFlags_;
    if ((fl & 0x2000u) != 0)
    {
      v04SetTx(obj, obj->txState_, V34HS_TRNSEG4A);
      sbarsegTail(obj);
      return;
    }
    {
      unsigned w35a4 = (uint16_t)obj->txMdMs_;
      if (w35a4 == 0)
      {
        v04SetTx(obj, obj->txState_, V34HS_PPSEG);
        sbarsegTail(obj);
        return;
      }
      if ((int16_t)fl < 0)
      {
        obj->tx.txCnt_ = obj->txMdSymbols_;
        v04SetTx(obj, obj->txState_, V34HS_PPSEG);
        sbarsegTail(obj);
        return;
      }
      obj->txMdSymbols_ = (uint16_t)(w35a4 * 0x53u);
    }
  }

  /* to TXMD: set up the modulator and the echo start delay */
  v04SetTx(obj, obj->txState_, V34HS_TXMD);
  V34SetupModulator(&obj->mod, V34MOD_ALLPASS_4800, V34_CARRIER_2400, 0, 0, 0); /* not a V.34 symbol rate: the all-pass source (v34_symbol_rates.h) */
  {
    int32_t e = (int32_t)(((uint32_t)(0x5e8 - (int16_t)obj->dmaDelay_)) << 14);
    int32_t q = e / 9600;
    int32_t w = (((int16_t)q * 0x960) >> 14) + 0x96;
    obj->hsSymCnt_ = (uint16_t)w;
    if (dsplibs_debug_level > 1u)
      dsplibs_debug_printf("Moving to TX MD, would take %d symbols , echo start delay is %d...\r\n", (int)(int16_t)obj->txMdSymbols_, (int)w);
  }
  sbarsegTail(obj);
}

/**
 * @brief Transmit state PPSEG (0x14): send one point of the PP sequence.
 * @param obj  V.34 module object
 * @note  k56Idx_ walks the 48 points of `vectpp` (two shorts each); after every pass w358e_ counts, and after the sixth pass k56Idx_ is set to
 *        bulkDelay_ + 0x90, period_ is derived from that and the symbol rate (baud_ 2400, 2800, 3000, 3200 or 3429; other rates leave it unchanged) plus
 *        0x120, hsSymCnt_ becomes the echo start delay (plus bulkDelay_ for the calling role), and the state goes to TRNSEG4.
 * @warning `vectpp` is indexed by the signed k56Idx_ without a bound check; it stays below 0x30 in normal operation (the table has 48 entries).
 */
extern "C" void v04_tx_ppseg(tagV34Object* obj)
{
  int idx = (int16_t)obj->tx.k56Idx_;
  uint32_t pt; /* vectpp is a short table read as dwords: memcpy, not a uint32_t lvalue (strict aliasing) */
  __builtin_memcpy(&pt, vectpp + 2 * (long)idx, sizeof pt);
  obj->tx.k56Idx_ = (uint16_t)(idx + 1);
  obj->tx.txPoint_ = pt;
  txmit(obj);

  /* end of one 48-point pass */
  if (obj->tx.k56Idx_ == 0x30)
  {
    uint16_t m = (uint16_t)(obj->w358e_ + 1u);
    if (m != 6)
    {
      obj->w358e_ = m;
      obj->tx.k56Idx_ = 0;
    }
    else
    {
      unsigned baud = obj->rate.baud_;
      int e = (int)obj->bulkDelay_ + 0x90;
      int a;
      unsigned tx, lv;
      int32_t q, t;

      /* symbol period from the symbol rate */
      obj->tx.k56Idx_ = (uint16_t)e;
      a = (int16_t)e;
      switch (baud)
      {
      case 0x960:
        obj->rate.period_ = (uint16_t)(a >> 2);
        break;
      case V34_BAUD_2800:
        obj->rate.period_ = (uint16_t)((a * 0x12ab) >> 14);
        break;
      case V34_BAUD_3000:
        obj->rate.period_ = (uint16_t)((a * 0x1400) >> 14);
        break;
      case V34_BAUD_3200:
        obj->rate.period_ = (uint16_t)((a * 0x1555) >> 14);
        break;
      case V34_BAUD_3429:
        obj->rate.period_ = (uint16_t)((a * 0x16dc) >> 14);
        break;
      default:
        break;
      }

      obj->w358e_ = 0;
      obj->rate.period_ = (uint16_t)(obj->rate.period_ + 0x120u);

      /* on to TRNSEG4 */
      tx = obj->txState_;
      lv = dsplibs_debug_level;
      if ((uint16_t)tx != V34HS_TRNSEG4)
      {
        if (lv > 1u)
        {
          v04PrTx(obj, tx, V34HS_TRNSEG4);
          baud = obj->rate.baud_;
          lv = dsplibs_debug_level;
        }
        obj->txState_ = V34HS_TRNSEG4;
      }

      /* echo start delay: near-end delay hsSymCnt_, far-end delay in period_ */
      q = (int32_t)((uint32_t)(0x5e8 - (int16_t)obj->dmaDelay_) * (uint32_t)(int32_t)(int16_t)baud) / 9600;
      obj->hsSymCnt_ = (uint16_t)q;
      t = q + (int32_t)((unsigned)obj->tx.txCnt_ + (unsigned)obj->filtDelay_) + 1;
      if (obj->modeChar_ == V34ROLE_CALL)
        obj->hsSymCnt_ = (uint16_t)(t + (int32_t)obj->bulkDelay_);
      else
        obj->hsSymCnt_ = (uint16_t)t;
      {
        unsigned p = (unsigned)obj->rate.period_ + (unsigned)obj->hsSymCnt_;
        obj->rate.period_ = (uint16_t)p;
        if (lv > 1u)
          dsplibs_debug_printf("V34Hshak: echo start wait time would be: NEC %d symbols, FEC %d symbols...\r\n", (int)(int16_t)obj->hsSymCnt_, (int)(int16_t)p);
      }
      return;
    }
  }
  obj->tx.txCnt_ = (uint16_t)(obj->tx.txCnt_ + 1u);
}

/**
 * @brief Transmit state TXMD (0x56): send one point of the scrambled MD sequence.
 * @param obj  V.34 module object
 * @note  The point is vect4[top three bits of the scrambler] (0..3: the top bit of the register is always clear).  When k56Idx_ reaches hsSymCnt_ the
 *        echo adaptation is enabled (txFlags_ bit 2 cleared, adaptation counter and alphas cleared); when it reaches txMdSymbols_ the modulator is set
 *        up for the negotiated baud rate, carrier and pre-emphasis (V.90 flag if PCM or K56 mode), and the state goes to SSEG with the scrambler cleared.
 */
extern "C" void v04_tx_txmd(tagV34Object* obj)
{
  uint32_t next = scrambleTwo(obj->tx.scrState_, obj->modeChar_ == V34ROLE_CALL);
  unsigned top = next >> 29;
  unsigned k;

  obj->tx.scrState_ = next;
  obj->tx.txBits_ = (uint16_t)top;
  obj->tx.txPoint_ = (uint32_t)vect4[top];
  txmit(obj);

  k = (uint16_t)(obj->tx.k56Idx_ + 1u);
  obj->tx.k56Idx_ = (uint16_t)k;
  if (obj->hsSymCnt_ == (uint16_t)k)
  {
    if (dsplibs_debug_level > 1u)
      dsplibs_debug_printf("On MD - enabling echo adaptation...\r\n");
    obj->echoAdaptCnt_ = 0;
    obj->tx.txFlags_ = (uint16_t)(obj->tx.txFlags_ & ~4u);
    obj->echoAlphaNear_ = 0;
    obj->echoAlphaFar_ = 0;
  }

  if (obj->tx.k56Idx_ == obj->txMdSymbols_)
  {
    int v90 = 0;
    unsigned tx;
    if (dsplibs_debug_level > 1u)
      dsplibs_debug_printf("TX: Done with MD, moving to S/Sbar again...\r\n");
    if (obj->pcmMode_ != 0)
      v90 = 1;
    else if (obj->k56Mode_ != 0)
      v90 = 1;

    V34SetupModulator(&obj->mod, (int16_t)obj->rate.baud_, (int16_t)obj->rate.carrier_, (int16_t)obj->rate.preemp_, v90, 0);
    tx = obj->txState_;
    v04SetTx(obj, tx, V34HS_SSEG);
    obj->tx.txCnt_ = 0;
    obj->tx.scrState_ = 0;
    obj->tx.txFlags_ = (uint16_t)(obj->tx.txFlags_ | 0x8004u);
  }

  obj->tx.txCnt_ = (uint16_t)(obj->tx.txCnt_ + 1u);
}
