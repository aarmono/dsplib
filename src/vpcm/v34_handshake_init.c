/**
 * @file v34_handshake_init.c
 * @brief V.34 handshake (re)initialisation: v34handshakinit() and the modem-on-hold (MOH) message builder VPcmV34SetMohMessageBits().
 *
 * v34handshakinit(obj, mode) resets the handshake state of the V.34 object (tagV34Object, accessed as `V34Obj`) for
 * one of five situations selected by `mode` (V34HSINIT_*, v34_hs_init_mode.h):
 * - V34HSINIT_START (0): originate / answer front end - modem and receive-timing initialisation, silence-INFO transmit state, DPSK receive state,
 *   INFO0d bit queue;
 * - V34HSINIT_RETRAIN (1): retrain - counts the retrain (remote or local, in `dgC14_` / `dgC12_`), silence-retrain transmit state, receive gain re-armed;
 * - V34HSINIT_RNEG_LOCAL / V34HSINIT_RNEG_REMOTE (2, 3): rate renegotiation - SSEG transmit state, INFO / MP word block cleared, digital pre-initialisation;
 * - V34HSINIT_MOH (4): modem on hold - MOH tone state, INFO0a bit queue holding the MOH message, tone detector set up for the 2400 Hz (calling) or 1200 Hz
 *   (answering) tone;
 * - any other value only runs the common head and tail.
 * Every mode first sets the run mode to V34RUN_HANDSHAKE, folds the data start time into the sample clock, clears the MOH working words and arms the 0x69780
 * sample deadline, and ends with `blkLen_` = 0x10.
 *
 * VPcmV34SetMohMessageBits() builds the 16-bit MOH message word for the pending message type; its callers are v34handshakinit() and the main V.34 handshake
 * function.
 *
 * Behaviour that is easy to lose:
 * - Every state change goes through st_tx() / st_rx() / st_mst(): `if ((short)word != K) { if (level > 1) print(...); word = K; }`, a 16-bit compare and
 *   store, with the old and new state names and the other two state names and the two counters in the print.  sname() returns "?" for a state outside the
 *   name table instead of reading past it.
 * - The MOH arm re-reads the reader pointer `bitQ` after calling VPcmV34SetMohMessageBits() and takes the debug-level flag for the "initiating MOH
 *   negotiation" print before the twelve reader stores.
 * - `txFlags_` is only read-modify-written as 16 bits (`& 0x7fff` at entry; `& 0xbfe7 | 0x2000` in the rate-renegotiation arm).
 *
 * Callees: v34modeminit(), rxtiminginit(), preinitdigital(), detectorinit(), V34SetINFO0dBits(), dsplibs_debug_printf().
 */

#include <limits.h>
#include <stddef.h>
#include <stdint.h>

#include "v34/v34_init_tables.h"  /* StateName */
#include "v34/v34_hs_states.h"    /* V34HS_* */
#include "v34/v34_hs_init_mode.h" /* V34HSINIT_* (the mode argument of v34handshakinit) */
#include "v34/v34_role.h"         /* V34ROLE_* (the values of modeChar_) */
#include "v34/v34_run_mode.h"     /* V34RUN_* (the values of runMode_) */
#include "v34/v34_frame_tables.h" /* c1200_, c2400_ */

#include "core/dsp_debug_level.h" /* dsplibs_debug_level, LEVEL_ON() */
#include "core/dsp_host.h"
#include "v34/v34_api.h"
#include "vpcm_v34_api.h"

#include "v34/v34_access.h" /* U16 (the message word), AS_U16 */
#include "v34/v34_object_view.h"

/**
 * @brief Build the modem-on-hold message word for the pending message type `mohF0_`.
 * @param obj  V.34 object
 * @param out  receives the 16-bit message word
 * @note  Message type 0 MHreq 0x33; 1 MHfrr 0xdd; 2 MHclrd 0x95 / 0x96 / 0x9a for disconnect reason `mohFA_` 0 / 1 / other; 3 MHcda 0xbb; 4 MHack
 *        0x50 + time-out code `mohE0_`; 5 MHnack 0x77.  Any other type leaves `*out` untouched.  Each arm prints which message it builds at debug level
 *        above 1.
 * @warning `out` is written without a NULL check.
 */
void VPcmV34SetMohMessageBits(V34Obj* obj, void* out)
{
  switch (obj->mohF0_)
  {
  case 0:
    if (LEVEL_ON())
      dsplibs_debug_printf("VPcmV34SetMohMessageBits: Building MHreq message...\r\n");
    U16(out, 0) = 0x33;
    break;
  case 1:
    if (LEVEL_ON())
      dsplibs_debug_printf("VPcmV34SetMohMessageBits: Building MHfrr message...\r\n");
    U16(out, 0) = 0xdd;
    break;
  case 2:
    if (LEVEL_ON())
      dsplibs_debug_printf("VPcmV34SetMohMessageBits: Building MHclrd message...\r\n");
    U16(out, 0) = 0x90;
    if (obj->mohFA_ == 0)
      U16(out, 0) = 0x95;
    else if (obj->mohFA_ == 1)
      U16(out, 0) = 0x96;
    else
      U16(out, 0) = 0x9a;
    break;
  case 3:
    if (LEVEL_ON())
      dsplibs_debug_printf("VPcmV34SetMohMessageBits: Building MHcda message...\r\n");
    U16(out, 0) = 0xbb;
    break;
  case 4:
    if (LEVEL_ON())
      dsplibs_debug_printf("VPcmV34SetMohMessageBits: Building MHack message...\r\n");
    U16(out, 0) = 0x50;
    U16(out, 0) = (uint16_t)(obj->mohE0_ + 0x50);
    break;
  case 5:
    if (LEVEL_ON())
      dsplibs_debug_printf("VPcmV34SetMohMessageBits: Building MHnack message...\r\n");
    U16(out, 0) = 0x77;
    break;
  default:
    break;
  }
}

/* ==================================================================================================
 * v34handshakinit
 * ================================================================================================== */

/* Name of handshake state i, "?" when i is outside the table. */
static const char* sname(int i) { return (i >= 0 && i < V34HS_COUNT) ? StateName[i] : "?"; }

/* The three state setters: transmit state, receive state and micro-state ("mst").  Each changes its 16-bit state word only when the value differs and then
 * prints the transition (debug level above 1). */
static void st_tx(V34Obj* obj, short k)
{
  if ((int16_t)obj->txState_ != k)
  {
    if (LEVEL_ON())
      dsplibs_debug_printf("V34HSHAKE: txstate %s=>%s(rx %s, mst %s, [1]%ld, [2]%ld)\n", sname((int16_t)obj->txState_), sname(k), sname((int16_t)obj->rxState_),
                           sname((int16_t)obj->mst_), (long)(int16_t)obj->tx.k56Idx_, (long)(int16_t)obj->hsSymCnt_);
    (obj)->txState_ = k;
  }
}

static void st_rx(V34Obj* obj, short k)
{
  if ((int16_t)obj->rxState_ != k)
  {
    if (LEVEL_ON())
      dsplibs_debug_printf("V34HSHAKE: rxstate %s=>%s(tx %s, mst %s, [1]%ld, [2]%ld)\n", sname((int16_t)obj->rxState_), sname(k), sname((int16_t)obj->txState_),
                           sname((int16_t)obj->mst_), (long)(int16_t)obj->tx.k56Idx_, (long)(int16_t)obj->hsSymCnt_);
    (obj)->rxState_ = k;
  }
}

static void st_mst(V34Obj* obj, short k)
{
  if ((int16_t)obj->mst_ != k)
  {
    if (LEVEL_ON())
      dsplibs_debug_printf("V34HSHAKE: microstate %s=>%s(tx %s, rx %s, [1]%ld, [2]%ld)\n", sname((int16_t)obj->mst_), sname(k), sname((int16_t)obj->txState_),
                           sname((int16_t)obj->rxState_), (long)(int16_t)obj->tx.k56Idx_, (long)(int16_t)obj->hsSymCnt_);
    (obj)->mst_ = k;
  }
}

/**
 * @brief Reset the handshake state of the V.34 object for the given situation.
 * @param obj   V.34 object
 * @param mode  V34HSINIT_START, _RETRAIN, _RNEG_LOCAL, _RNEG_REMOTE or _MOH (compared unsigned; any other value runs only the common head and tail)
 * @return always 0
 * @note  Common head: run mode V34RUN_HANDSHAKE; a positive `dataStartTime_` (signed test) is folded into the sample clock: if the elapsed time
 *        `sampleTime_ - dataStartTime_` is at most 0x176ff (unsigned) it becomes the new clock and the start time is cleared, otherwise the clock is
 *        cleared and the start time set to 0xfff15a00.  Then the MOH words and `b_abfe_` / `wAbff_` are cleared, `trainStartTime_` = sample clock,
 *        `deadline_` = clock + 0x69780 and bit 15 of `txFlags_` is cleared.  Common tail: `waa3c_` = 0, `blkLen_` = 0x10.
 */
int v34handshakinit(V34Obj* obj, unsigned int mode)
{

  /* ---- common head ---- */
  obj->mod.runMode_ = V34RUN_HANDSHAKE;
  if (obj->dataStartTime_ > 0)
  { /* signed test */
    uint32_t d = (uint32_t)obj->sampleTime_ - (uint32_t)obj->dataStartTime_;

    if (d <= 0x176ffu)
    { /* unsigned compare */
      obj->sampleTime_ = d;
      obj->dataStartTime_ = 0;
    }
    else
    {
      obj->sampleTime_ = 0;
      obj->dataStartTime_ = 0xfff15a00u;
    }
  }

  obj->mohE6_ = 0;
  obj->trainStartTime_ = (uint32_t)obj->sampleTime_;
  obj->deadline_ = (uint32_t)obj->sampleTime_ + 0x69780u;
  obj->mohE8_ = 0;
  obj->mohF8_ = 0;
  obj->mohE4_ = 0;
  obj->b_abfe_ = 0;
  obj->wAbff_ = 0;
  obj->tx.txFlags_ = (uint16_t)(obj->tx.txFlags_ & 0x7fff);

  switch (mode)
  {
  /* originate / answer front end */
  case V34HSINIT_START:
    v34modeminit(obj);
    rxtiminginit(obj);
    st_tx(obj, V34HS_SILENCEINFO);
    st_rx(obj, V34HS_RX_DPSK);
    st_mst(obj, V34HS_DET_SYNC);
    {
      uint16_t f = (uint16_t)(obj->rx.rxFlags_ | 0x1000); /* read after the state prints, stored after the pointer slot */
      obj->bitQ2_ = (struct tag_bitQ*)&obj->info0d;
      obj->rx.rxFlags_ = f;
    }

    obj->info0d.i0dTail_.total = 0x11;
    obj->tx.w25dc_ = 0;
    if (obj->modeChar_ == V34ROLE_ANSWER)
      V34SetINFO0dBits(obj, (struct tag_bitQ*)&obj->info0d);
    break;

  /* retrain: count the retrain, restart the modem, wait for the far end */
  case V34HSINIT_RETRAIN:
    if ((obj->rx.rxFlags_ & 0x40) != 0 || obj->b_ac17_ != 0)
    {
      obj->b_ac17_ = 0;
      obj->dgC14_ = (uint16_t)(AS_U16(obj->dgC14_) + 1);
    }
    else
    {
      obj->dgC12_ = (uint16_t)(AS_U16(obj->dgC12_) + 1);
    }

    v34modeminit(obj);
    st_tx(obj, V34HS_SILENCERETRAIN);
    st_rx(obj, V34HS_WAIT);
    obj->rx.rxFlags_ = (uint16_t)(obj->rx.rxFlags_ | 0x1000);
    obj->rx.rxGain_ = AS_U16(obj->rx.rxGainEst_);
    obj->w358a_ = 2;
    obj->w3588_ = 2;
    break;

  /* rate renegotiation (local or remote): SSEG transmit, receive state, INFO / MP word block cleared */
  case V34HSINIT_RNEG_LOCAL:
  case V34HSINIT_RNEG_REMOTE:
  {
    st_tx(obj, V34HS_SSEG);
    st_rx(obj, V34HS_RECEIVE);
    st_mst(obj, V34HS_DET_SYNC);

    /* clear the INFO / MP word block */
    obj->q2.q2Tail_.crc = USHRT_MAX;
    obj->q2.q2Tail_.pos = 0;
    obj->q2.q2Tail_.widx = 0;
    obj->q2.q2Tail_.pass = 0;
    obj->q2.q2Tail_.total = 0;
    obj->q2.q2Tail_.chunk = 0;
    obj->q2.q2Tail_.crcOn = 0;
    obj->q2.q2Tail_.nbits = 0;
    obj->q2.q2Tail_.restartBits = 0;
    obj->q2.q2Tail_.loop = 0;
    obj->q2.q2Tail_.acc = 0;
    obj->q2.q2Tail_.restartAcc = 0;

    /* transmit and receive parameters of the renegotiation */
    obj->tx.txCnt_ = 0;
    obj->tx.txPrev_ = 0;
    obj->tx.txFlags_ = (uint16_t)((obj->tx.txFlags_ & 0xbfe7u) | 0x2000u);
    obj->tx.scrState_ = 0;
    obj->rx.w388_ = 0x21e;
    obj->rx.rxSig_ = 0x8990;
    obj->rx.rxFlags_ = (uint16_t)((obj->rx.rxFlags_ & 0xfe27u) | 0x18u);
    obj->tx.k56Idx_ = 0;
    obj->hsSymCnt_ = 0;
    preinitdigital(obj);

    {
      uint16_t rxf;
      obj->rx.w047c_ = 0x400;
      rxf = (uint16_t)(obj->rx.rxFlags_ & 0xdfffu);
      {
        int lv = LEVEL_ON();
        obj->rx.rxFlags_ = rxf;
        if (lv)
          dsplibs_debug_printf("V34RNEG, initialize RNEG, tx->txflags= 0x%x,rx->rxflgs= 0x%x\n", (int)(int16_t)obj->tx.txFlags_, (unsigned int)rxf);
      }
    }
    break;
  }

  /* modem on hold: MOH tone state, the MOH message in the INFO0a queue, tone detector on the MOH tone */
  case V34HSINIT_MOH:
  {
    struct tag_bitQ* rd;
    int lv;

    obj->mohE8_ = 1;
    v34modeminit(obj);
    obj->rx.rxGain_ = AS_U16(obj->rx.rxGainEst_); /* before the state prints, unlike the retrain arm */
    st_tx(obj, V34HS_SILENCERETRAIN);
    st_rx(obj, V34HS_WAIT);
    st_mst(obj, V34HS_MOH_TONE);
    obj->bitQ2_ = (struct tag_bitQ*)&obj->info0d;
    obj->info0d.i0dTail_.total = 8;
    obj->bitQ_ = (struct tag_bitQ*)&obj->info0a;
    VPcmV34SetMohMessageBits(obj, &obj->info0a);

    lv = LEVEL_ON(); /* debug flag for the print below, taken before the reader stores */
    rd = obj->bitQ_; /* re-read after the call */
    rd->tail_.crc = USHRT_MAX;
    rd->tail_.pos = 0;
    rd->tail_.widx = 0;
    rd->tail_.pass = 0;
    rd->tail_.total = 8;
    rd->tail_.chunk = 8;
    rd->tail_.crcOn = 1;
    rd->tail_.nbits = 0xc;
    rd->tail_.restartBits = 0xc;
    rd->tail_.loop = 0;
    rd->tail_.acc = 0xf72;
    rd->tail_.restartAcc = 0xf72;

    obj->toneSel_ = 0;
    if (lv)
      dsplibs_debug_printf("v34handshakinit: initiating MOH negotiation\n");
    detectorinit(&obj->det, obj->modeChar_ == V34ROLE_CALL ? c2400_ : c1200_, 0, 100, 0x32, 0x800, 0);
    obj->det.toneDetF6_ = 1;
    obj->w358a_ = 0;
    break;
  }

  default:
    break;
  }

  /* ---- common tail ---- */
  obj->mp.waa3c_ = 0;
  obj->tx.blkLen_ = 0x10;
  return 0;
}
