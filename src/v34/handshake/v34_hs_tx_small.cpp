/**
 * @file v34_hs_tx_small.cpp
 * @brief V.34 handshake: the small transmit-state cases of the per-sample loop of v34handshak().
 *
 * Each case is a separate extern "C" function taking the V.34 module object.  The loop in v34handshak() (v34_hs_dispatch.cpp,
 * `while ((short)txIdx_ < (short)blkLen_)`) re-evaluates its condition after every case; the dispatcher selects a case by the transmit state.
 *
 *   state (txState_)     function               callees
 *   0x12 SSEG            v04_tx_sseg            txmit x2
 *   0x3c TONE_AB         v04_tx_toneab          txmit
 *   0x41 XMIT0           v04_tx_xmit0           txmit
 *   0x46 DATAXMIT        v04_tx_dataxmit        modulatevector
 *   0x47 TXLEVEL         v04_tx_txlevel         txmit
 *   0x4e JaTXMIT         v04_tx_jatxmit         V34EchoReportCoeff x2, v90Phase34
 *   0x55 K56JaTXMIT      v04_tx_k56jatxmit      V34EchoReportCoeff x2, k56FlexPhase34
 *   0x51..0x54 MOH_*     v04_tx_moh             txwritequeue
 *
 * @note Every debug guard is the unsigned test `dsplibs_debug_level > 1`.  JaTXMIT / K56JaTXMIT keep one copy of the level that is refreshed only
 *       after a print.  The transmit-state transition print ("V34HSHAKE: txstate %s=>%s(rx %s, mst %s, [1]%ld, [2]%ld)") indexes the StateName table
 *       with signed 16-bit state words.
 * @warning The state words rxState_ / mst_ are not range checked before they index StateName; an out-of-range value reads outside the table.
 *          The 16-bit counters (txCnt_, k56Idx_, hsSymCnt_) wrap at 16 bits.
 */

#include "v34/v34_object.h"
#include "v34/v34_hs_states.h" /* enum V34HsState: V34HS_* */
#include "core/dsp_host.h"
#include "v34/v34_internal.h"
#include "v34/v34_init_tables.h"   /* StateName */
#include "v34/v34_symbol_tables.h" /* vect4 */

/**
 * @brief Print the transmit-state transition line "V34HSHAKE: txstate old=>new(rx, mst, [1], [2])".
 * @param obj     handshake object (rx / mst names and the [1] = k56Idx_, [2] = hsSymCnt_ values are read from it)
 * @param cur     transmit state word as the caller read it before its guard
 * @param newIdx  new transmit state (V34HS_*)
 */
static void txTransPrint(tagV34Object* obj, unsigned cur, int newIdx)
{
  dsplibs_debug_printf("V34HSHAKE: txstate %s=>%s(rx %s, mst %s, [1]%ld, [2]%ld)\n", StateName[(int16_t)cur], StateName[newIdx],
                       StateName[(int16_t)obj->rxState_], StateName[(int16_t)obj->mst_], (long)(int16_t)obj->tx.k56Idx_, (long)(int16_t)obj->hsSymCnt_);
}

/**
 * @brief Set the transmit state, printing the transition when the debug level is above 1; no-op when it is unchanged.
 * @param obj     handshake object
 * @param cur     transmit state word as the caller read it
 * @param newIdx  new transmit state (V34HS_*)
 */
static void setTx(tagV34Object* obj, unsigned cur, int newIdx)
{
  if ((uint16_t)cur != newIdx)
  {
    if (dsplibs_debug_level > 1u)
      txTransPrint(obj, cur, newIdx);
    obj->txState_ = (uint16_t)newIdx;
  }
}

/**
 * @brief Transmit state SSEG (0x12): send two sync points, and after 64 calls move to SBARSEG (0x13).
 * @param obj  handshake object
 */
extern "C" void v04_tx_sseg(tagV34Object* obj)
{
  uint16_t n;

  obj->tx.txPoint_ = (uint32_t)vect4[0];
  txmit(obj);
  obj->tx.txPoint_ = (uint32_t)vect4[3];
  txmit(obj);
  n = (uint16_t)(obj->tx.txCnt_ + 1u);
  obj->tx.txCnt_ = n;
  if (n == 0x40)
  {
    setTx(obj, obj->txState_, V34HS_SBARSEG);
    obj->tx.txCnt_ = 0;
  }
}

/**
 * @brief Transmit state TONE_AB (0x3c): send the tone point selected by bit 0 of toneSel_ (vect4[0] or vect4[2]).
 * @param obj  handshake object
 */
extern "C" void v04_tx_toneab(tagV34Object* obj)
{
  int sel = (int16_t)obj->toneSel_ & 1;
  obj->tx.txPoint_ = (uint32_t)vect4[sel * 2];
  txmit(obj);
}

/**
 * @brief Transmit state XMIT0 (0x41): send the zero point; once the receiver reports bit 3 of rxFlags_ go to SSEG.
 * @param obj  handshake object
 */
extern "C" void v04_tx_xmit0(tagV34Object* obj)
{
  obj->tx.txPoint_ = 0;
  txmit(obj);
  if ((obj->rx.rxFlags_ & 0x8u) != 0)
  {
    obj->tx.txFlags_ = (uint16_t)(obj->tx.txFlags_ | 0x2000u);
    setTx(obj, obj->txState_, V34HS_SSEG);
    obj->tx.txPrev_ = 0;
    obj->tx.txCnt_ = 0;
    obj->tx.scrState_ = 0;
  }
}

/**
 * @brief Transmit state DATAXMIT (0x46): modulate; when the transmit buffer is flushed (txFlags_ bit 4) latch the rates and enter data mode.
 * @param obj  handshake object; on the switch sets dataStartTime_, w228_ / w22c_ (receive rate index) and runMode_ = V34RUN_DATA
 */
extern "C" void v04_tx_dataxmit(tagV34Object* obj)
{
  modulatevector(obj);

  /* transmit buffer flushed: latch the rates and switch from handshake to data mode */
  if ((obj->tx.txFlags_ & 0x10u) != 0)
  {
    uint32_t w238 = obj->sampleTime_;
    int32_t m2218;
    int32_t rate;
    obj->rx.w484_ = 0;
    obj->rx.w480_ = 0;
    obj->dataStartTime_ = w238;
    m2218 = obj->mod.runMode_;
    rate = (int16_t)obj->rate.rxRateIdx_;
    obj->w228_ = rate;
    obj->w22c_ = rate;
    if (m2218 == V34RUN_HANDSHAKE && dsplibs_debug_level > 1u)
      dsplibs_debug_printf("V34DATA, getting into data mode from Handshake, Tx bit rate - %d, Rx bit Rate - %d\n", (int)((int16_t)obj->rate.txRateIdx_ * 0x960),
                           (int)(rate * 0x960));
    obj->mod.runMode_ = V34RUN_DATA;
  }
}

/**
 * @brief Transmit state TXLEVEL (0x47): run two steps of the transmit scrambler and send the point taken from its top bits.
 * @param obj  handshake object; scrState_ is advanced by two steps, txBits_ and txPoint_ are set
 * @note  The feedback taps are bit 8 and, by role, bit 13 (modeChar_ == V34ROLE_CALL) or bit 26; the input bits are the constant 3.  The new
 *        txBits_ is the top three bits of the register.
 */
extern "C" void v04_tx_txlevel(tagV34Object* obj)
{
  uint32_t reg = obj->tx.scrState_;
  int inBits = 3;
  int step = 0;
  uint32_t next;
  const bool role = (obj->modeChar_ == V34ROLE_CALL);

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

  obj->tx.scrState_ = next;
  obj->tx.txBits_ = (uint16_t)(next >> 29);
  obj->tx.txPoint_ = (uint32_t)vect4[0];
  txmit(obj);
}

/**
 * @brief Echo-freeze countdown shared by JaTXMIT and K56JaTXMIT.
 * @param obj  handshake object
 * @param msg  message printed (debug level above 1) when the countdown reaches zero
 * @note  Counts hsSymCnt_ down while it is non-zero; on reaching zero prints `msg` and "V34HSHAK: Freeze EC", sets txFlags_ bit 2 and reports the
 *        near and far echo-canceller coefficient blocks.
 */
static void jaFreeze(tagV34Object* obj, const char* msg)
{
  uint16_t c = obj->hsSymCnt_;
  if (c != 0)
  {
    c = (uint16_t)(c - 1u);
    obj->hsSymCnt_ = c;
    if (c == 0)
    {
      unsigned lv = dsplibs_debug_level;
      if (lv > 1u)
      {
        dsplibs_debug_printf(msg);
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
    }
  }
}

/**
 * @brief Transmit state JaTXMIT (0x4e): echo-freeze countdown, then the V.90 phase 3 / 4 transmitter.
 * @param obj  handshake object
 */
extern "C" void v04_tx_jatxmit(tagV34Object* obj)
{
  jaFreeze(obj, "V34Hshak: on JaTXMIT - time to freeze echo...\r\n");
  v90Phase34(obj);
}

/**
 * @brief Transmit state K56JaTXMIT (0x55): echo-freeze countdown, then the K56flex phase 3 / 4 transmitter.
 * @param obj  handshake object
 */
extern "C" void v04_tx_k56jatxmit(tagV34Object* obj)
{
  jaFreeze(obj, "V34Hshak: on K56JaTXMIT - time to freeze echo...\r\n");
  k56FlexPhase34(obj);
}

/**
 * @brief Transmit states MOH_SILENCE / MOH_ON_HOLD / MOH_FRR / MOH_CLEARDOWN (0x51 .. 0x54): send silence.
 * @param obj  handshake object
 * @note  Writes a zero quad to the transmit queue.  After 192 symbols in MOH_SILENCE the state becomes MOH_FRR when an MOH request is pending
 *        (mohEC_ or mohF0_ equal to 1), else MOH_ON_HOLD.
 */
extern "C" void v04_tx_moh(tagV34Object* obj)
{
  short q[4] = {0, 0, 0, 0};
  uint16_t n;

  txwritequeue(&obj->tx, q);
  n = (uint16_t)(obj->tx.k56Idx_ + 1u);
  obj->tx.k56Idx_ = n;
  if (n == 0xc0)
  {
    unsigned tx = obj->txState_;
    if ((uint16_t)tx == V34HS_MOH_SILENCE)
    {
      if (dsplibs_debug_level > 1u)
        dsplibs_debug_printf("V34F MOH: After 192 silence, org = %d , act = %d\r\n", (int)obj->mohEC_, (int)obj->mohF0_);
      tx = obj->txState_;
      if (obj->mohEC_ == 1u || obj->mohF0_ == 1u)
        setTx(obj, tx, V34HS_MOH_FRR);
      else
        setTx(obj, tx, V34HS_MOH_ON_HOLD);
      obj->tx.k56Idx_ = 0;
    }
  }
}
