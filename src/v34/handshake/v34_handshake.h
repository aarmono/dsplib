/**
 * @file v34_handshake.h
 * @brief Shared declarations of the V.34 handshake state machine files (src/v34/handshake/v34_hs_*.cpp).
 *
 * v34handshak() (v34_hs_dispatch.cpp) calls one handler per transmit state, receive state and receive micro-state; the handlers live in the other
 * v34_hs_*.cpp files and are declared here.  This header also holds the helpers they all use:
 * - the state-transition tracing (v04SetTx / v04SetRx / v04SetMst and the v04Pr* printers), which print
 *   "V34HSHAKE: <kind>state <old>=><new>(<other two state names>, [1]<k56Idx_>, [2]<hsSymCnt_>)" at debug level > 1.  Names are looked up in StateName[]
 *   with the state word taken as a SIGNED 16-bit index; the two other state words are read from the object when the line is printed;
 * - v04Scramble(), the two-step transmit scrambler used by every transmit state that sends scrambled bits.
 * Header-only (`static inline`), C++ with C linkage for the handlers.
 */
#ifndef V04_COMMON_H
#define V04_COMMON_H

#include "v34/v34_object.h"
#include "v34/v34_hs_states.h"     /* enum V34HsState: V34HS_* (generated from the StateName table) */
#include "v34/v34_symbol_rates.h"  /* V34_BAUD_*, V34_CARRIER_* */
#include "v34/v34_init_tables.h"   /* StateName */
#include "v34/v34_symbol_tables.h" /* vect4, vect16 */
#include "core/dsp_host.h"

extern "C"
{
  /*
   * Helpers the handshake pieces call (v34_handshake_helpers.c; v34_hs_getbit_bulkdelay.cpp in the build that maps the names to its own copies,
   * -DV04_REAL_HELPERS, see dsplib.mk).  At 32 bit the declarations carry regparm(3) unless that mapping is on.  Deliberately not in v34_internal.h.
   */
#if defined(__i386__) && !defined(V04_REAL_HELPERS)
  short getbit(tag_bitQ* q) __attribute__((regparm(3)));
  void ApplyBulkDelay(tagV34Object* obj, int delay) __attribute__((regparm(3)));
#else
  short getbit(tag_bitQ* q);
  void ApplyBulkDelay(struct tagV34Object* obj, int delay);
#endif

  /** Diagnostic prints at the start / middle of the echo adaptation (vpcm/v34_api.c).  The definitions take no argument; the pieces pass the object. */
  void VPcmV34ReportStartOfEchoAdapt(tagV34Object* obj);
  void VPcmV34ReportMiddleOfEchoAdapt(tagV34Object* obj);

  /* handlers of the transmit states, called by v04z_tx_dispatch() (v34_hs_dispatch.cpp); each takes the V.34 object */
  void v04_tx_sseg(tagV34Object*);
  void v04_tx_toneab(tagV34Object*);
  void v04_tx_xmit0(tagV34Object*);
  void v04_tx_dataxmit(tagV34Object*);
  void v04_tx_txlevel(tagV34Object*);
  void v04_tx_jatxmit(tagV34Object*);
  void v04_tx_k56jatxmit(tagV34Object*);
  void v04_tx_moh(tagV34Object*);
  void v04_tx_txl1(tagV34Object*);
  void v04_tx_sbarseg(tagV34Object*);
  void v04_tx_ppseg(tagV34Object*);
  void v04_tx_txmd(tagV34Object*);
  void v04_tx_exmit(tagV34Object*);
  void v04_tx_jtxmit(tagV34Object*);
  void v04_tx_silence(tagV34Object*);
  void v04_tx_dpsk(tagV34Object*);
  void v04_tx_xmitmp(tagV34Object*);
  void v04_tx_trnseg4(tagV34Object*);
  void v04_tx_trnseg4a(tagV34Object*);
  /* handlers of the receive states and receive micro-states, called by v04z_rx_dispatch(); the `unsigned` parameters carry state words the dispatcher has
   already read, `const short*` the demodulator sample buffer */
  void v04_rx_receive(tagV34Object*);
  void v04_rx_det_ab(tagV34Object*);
  void v04_rx_l1(tagV34Object*);
  void v04_wait(tagV34Object*);
  void v04_rx_dpsk_dft(tagV34Object*);
  void v04_t3_rx_phase3_call(tagV34Object*, unsigned);
  void v04_t3_tx_phase3_ans(tagV34Object*, unsigned);
  void v04_t3_tx_phase2(tagV34Object*, unsigned);
  void v04_t3_infodone(tagV34Object*, unsigned);
  void v04_t3_moh_tone_drop(tagV34Object*, unsigned, const short*);
  void v04_t3_moh_tone(tagV34Object*, const short*);
  void v04_t3_det_sync(tagV34Object*, unsigned, const short*);
  void v04_t3_det_info(tagV34Object*);
  void v04_t3_rx_phase1_ans(tagV34Object*, unsigned);
  void v04_t3_rx_phase2_ans(tagV34Object*, unsigned);
  void v04_t3_tx_phase1_call(tagV34Object*, unsigned);
  void v04_t3_tx_l1(tagV34Object*, unsigned);
  void v04_t3_rx_phase1_call(tagV34Object*);
  void v04_t3_rx_phase2_call(tagV34Object*, const short*);
  void v04_t3_tx_phase1_ans(tagV34Object*);
  void v04_txpost(tagV34Object*, unsigned);
  int v04_bitq_getbit(tag_bitQ* q); /* v34_hs_tx_dpsk.cpp; also used by v34_hs_tx_xmitmp.cpp */
}

#define V04_FMT_TX "V34HSHAKE: txstate %s=>%s(rx %s, mst %s, [1]%ld, [2]%ld)\n"
#define V04_FMT_RX "V34HSHAKE: rxstate %s=>%s(tx %s, mst %s, [1]%ld, [2]%ld)\n"
#define V04_FMT_MST "V34HSHAKE: microstate %s=>%s(tx %s, rx %s, [1]%ld, [2]%ld)\n"

/**
 * @brief Print a transmit-state transition line.
 * @param obj     V.34 object; supplies the rx and micro-state names and the [1] (k56Idx_) and [2] (hsSymCnt_) values
 * @param cur     state word the caller loaded before testing for a change (not re-read)
 * @param newIdx  new state, an index into StateName[]
 */
static inline void v04PrTx(tagV34Object* obj, unsigned cur, int newIdx)
{
  dsplibs_debug_printf(V04_FMT_TX, StateName[(int16_t)cur], StateName[newIdx], StateName[(int16_t)obj->rxState_], StateName[(int16_t)obj->mst_],
                       (long)(int16_t)obj->tx.k56Idx_, (long)(int16_t)obj->hsSymCnt_);
}
/**
 * @brief Print a transmit-state transition with every value given by the caller.
 * @param oldTx,newTx  old and new transmit state (StateName indexes, taken as signed 16-bit)
 * @param rx,mst       receive and micro-state to print
 * @param a1,a2        the [1] and [2] values
 * @note  Used where the caller holds values that differ from what the object contains at print time.
 */
static inline void v04PrTxX(unsigned oldTx, int newTx, unsigned rx, unsigned mst, long a1, long a2)
{
  dsplibs_debug_printf(V04_FMT_TX, StateName[(int16_t)oldTx], StateName[newTx], StateName[(int16_t)rx], StateName[(int16_t)mst], a1, a2);
}
/**
 * @brief Print a micro-state transition with every value given by the caller (see v04PrTxX()).
 * @param oldM,newM  old and new micro-state
 * @param tx,rx      transmit and receive state to print
 * @param a1,a2      the [1] and [2] values
 */
static inline void v04PrMstX(unsigned oldM, int newM, unsigned tx, unsigned rx, long a1, long a2)
{
  dsplibs_debug_printf(V04_FMT_MST, StateName[(int16_t)oldM], StateName[newM], StateName[(int16_t)tx], StateName[(int16_t)rx], a1, a2);
}
/** @brief The [1] value of the trace lines: `k56Idx_` as a signed 16-bit number. */
static inline long v04K56(tagV34Object* obj) { return (long)(int16_t)obj->tx.k56Idx_; }
/** @brief The [2] value of the trace lines: `hsSymCnt_` as a signed 16-bit number. */
static inline long v04W78(tagV34Object* obj) { return (long)(int16_t)obj->hsSymCnt_; }

/**
 * @brief Move the transmit state to `newIdx`; trace the change when debug level > 1.  Does nothing if `cur` already equals `newIdx`.
 * @param obj     V.34 object; `txState_` is written
 * @param cur     the transmit-state word the caller read earlier (compared, not re-read)
 * @param newIdx  new state (V34HS_*)
 */
static inline void v04SetTx(tagV34Object* obj, unsigned cur, int newIdx)
{
  if ((uint16_t)cur != newIdx)
  {
    if (dsplibs_debug_level > 1u)
      v04PrTx(obj, cur, newIdx);
    obj->txState_ = (uint16_t)newIdx;
  }
}

/**
 * @brief Move the receive state to `newIdx`; trace the change when debug level > 1.  Does nothing if `cur` already equals `newIdx`.
 * @param obj     V.34 object; `rxState_` is written; the tx and micro-state names in the trace are read from the object
 * @param cur     the receive-state word the caller read earlier (compared, not re-read)
 * @param newIdx  new state (V34HS_*)
 */
static inline void v04SetRx(tagV34Object* obj, unsigned cur, int newIdx)
{
  if ((uint16_t)cur != newIdx)
  {
    if (dsplibs_debug_level > 1u)
      dsplibs_debug_printf(V04_FMT_RX, StateName[(int16_t)cur], StateName[newIdx], StateName[(int16_t)obj->txState_], StateName[(int16_t)obj->mst_],
                           (long)(int16_t)obj->tx.k56Idx_, (long)(int16_t)obj->hsSymCnt_);
    obj->rxState_ = (uint16_t)newIdx;
  }
}

/**
 * @brief Move the receive micro-state to `newIdx`; trace the change when debug level > 1.  Does nothing if `cur` already equals `newIdx`.
 * @param obj     V.34 object; `mst_` is written; the tx and rx names in the trace are read from the object
 * @param cur     the micro-state word the caller read earlier (compared, not re-read)
 * @param newIdx  new state (V34HS_*)
 */
static inline void v04SetMst(tagV34Object* obj, unsigned cur, int newIdx)
{
  if ((uint16_t)cur != newIdx)
  {
    if (dsplibs_debug_level > 1u)
      dsplibs_debug_printf(V04_FMT_MST, StateName[(int16_t)cur], StateName[newIdx], StateName[(int16_t)obj->txState_], StateName[(int16_t)obj->rxState_],
                           (long)(int16_t)obj->tx.k56Idx_, (long)(int16_t)obj->hsSymCnt_);
    obj->mst_ = (uint16_t)newIdx;
  }
}

/**
 * @brief Advance the 23-bit transmit scrambler by two steps.
 * @param reg    scrambler shift register (the object's `scrState_`)
 * @param tap13  true: feedback taps at register bits 13 and 8 (generator polynomial GPC, the calling modem); false: bits 26 and 8 (GPA, the answering
 *               modem), see V.34 clause 6.1
 * @param seed   input bits, consumed least-significant first, one per step (3 for transmit states with a constant input)
 * @return the register after the second step; the caller stores it back into `scrState_`
 * @note  Each step XORs the next input bit with the two tapped register bits, sets bit 31 of the register when the result is 1 and shifts right by one.
 */
static inline uint32_t v04Scramble(uint32_t reg, bool tap13, int seed)
{
  int inBits = seed;
  int step = 0;
  uint32_t next;
  do
  {
    int feedback = (inBits & 1) != 0 ? 1 : 0;
    inBits >>= 1;
    if (tap13)
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

#endif /* V04_COMMON_H */
