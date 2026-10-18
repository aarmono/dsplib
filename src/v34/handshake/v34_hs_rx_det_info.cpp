/**
 * @file v34_hs_rx_det_info.cpp
 * @brief V.34 handshake microstate handler DET_INFO (receive and decode one INFO bit; act on a complete INFO0 / INFO1a / INFO1c / MOH message).
 *
 * One extern "C" handler, v04_t3_det_info(obj), called from the RX_DPSK microstate dispatch of v34handshak() (v34_hs_dispatch.cpp) after V34agc() and
 * fskdemodulate(); the caller performs the transmit-state post processing afterwards.  Callees: V34GiveINFO0dBits(), V34GiveINFO1aBits(), V34GiveINFO1dBits(),
 * V34GiveProbeResults(), V34SetINFO1aBits(), VPcmV34InterpretMohMessageBits(), bitreverse(), detectorinit(), probeselect(), rxinit() and setfinalrate().
 *
 * Each call consumes one received INFO bit (the low bit of v21RxBuf_; v21BitCnt_ counts the symbols left until the next bit).  The CRC register of the active
 * receive record (bitQ2_ -> crc, polynomial 0x1021) is updated per bit and, every eighth bit, the received byte is stored in the record's word array (diTail).
 * When hsSymCnt_ + 1 equals the record's message length total plus the 16 CRC bits, the CRC is compared with v21RxBuf_:
 * - mismatch: re-arm the INFO0 detector (microstate DET_SYNC, diCrcError);
 * - total == 0x4d: INFO1a received by the answering side (diInfo1a): rate bits, probeselect(), INFO1d / INFO1a answer;
 * - total == 0x26: INFO1c received by the calling side (diInfo1c): receiver set-up (baud-rate and carrier dependent constants, tone detectors);
 * - total == 8: music-on-hold message (diMoh);
 * - any other length: INFO0 (diInfo0): copied to the wAbae_ array during error recovery, otherwise handed to V34GiveINFO0dBits() and the next
 *   microstate chosen by role and the short-phase-2 flag.
 */

#include <limits.h>
#include "v34_handshake.h"
#include "core/dsp_access.h"
#include "v34/v34_internal.h"
#include "vpcm/vpcm_v34_api.h"
#include "vpcm/vpcm_session_wrappers.h"
#include "v34/v34_api.h"
#include "v34/v34_frame_tables.h" /* c1200_, c2400_ */
#include "v34/v34_init_tables.h"  /* hsine1600 ... hsine2000 */

/** @brief Debug print of a receive-state transition; the tx and mst words are read at print time. */
static inline void diPrRx(tagV34Object* obj, unsigned old, int nw)
{
  dsplibs_debug_printf(V04_FMT_RX, StateName[(int16_t)old], StateName[nw], StateName[(int16_t)obj->txState_], StateName[(int16_t)obj->mst_], v04K56(obj),
                       v04W78(obj));
}
/** @brief Debug print of a microstate transition; the tx and rx words are read at print time. */
static inline void diPrMst(tagV34Object* obj, unsigned old, int nw) { v04PrMstX(old, nw, obj->txState_, obj->rxState_, v04K56(obj), v04W78(obj)); }
/** @brief Read the unaligned 16-bit word at byte offset `off` of a record. */
static inline uint16_t diW16(const uint8_t* p, unsigned off)
{
  uint16_t v;
  __builtin_memcpy(&v, p + off, 2);
  return v;
}
/** @brief Debug print of the first ten 16-bit words of a record, preceded by `label`. */
static void diPrWords(const char* label, const uint8_t* rec)
{
  unsigned w[10];
  int i;
  for (i = 0; i < 10; i++)
    w[i] = diW16(rec, 2 * i);
  dsplibs_debug_printf("%s 0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x\n", label, w[0], w[1], w[2], w[3], w[4], w[5], w[6], w[7], w[8], w[9]);
}
/**
 * @brief Address of wAbae_[i] without a bound check.
 * @warning  wAbae_ holds 10 words; diInfo0() indexes it with a count derived from hsSymCnt_ and does not check it, so a count above 10 writes into the
 *           members that follow the array.
 */
static inline uint16_t* diWabae(tagV34Object* obj, int i) { return (uint16_t*)((uint8_t*)obj + __builtin_offsetof(tagV34Object, wAbae_)) + i; }

/**
 * @brief Per-bit tail: after every eighth received bit store the byte in v21RxBuf_ in the word array of the receive record.
 * @param obj  V.34 module object
 * @note  Stores word number (hsSymCnt_ / 8 - 1) of bitQ2_ (only for word numbers up to 9).
 */
static void diTail(tagV34Object* obj)
{
  unsigned symCnt = obj->hsSymCnt_;
  int byteIdx;

  if (symCnt & 7u)
    return;
  if ((int16_t)symCnt <= 0)
    return;
  byteIdx = (int16_t)(((int)(int16_t)symCnt >> 3) - 1);
  if ((int16_t)byteIdx > 9)
    return;
  {
    unsigned b = (uint8_t)obj->v21RxBuf_;
    tag_bitQ* q = obj->bitQ2_;
    uint16_t v = (uint16_t)b;
    __builtin_memcpy(q->unk00_ + 2 * byteIdx, &v, 2);
  }
}

/**
 * @brief Arm the INFO0a error recovery: recovery sub-state 1, txstate TX_DPSK, microstate DET_SYNC and the INFO0a bit queue (0x1e bits for the calling role
 *        in PCM mode, 0x11 otherwise).
 * @param obj  V.34 module object
 */
static void diInfo0Rearm(tagV34Object* obj)
{
  unsigned txCur, mstCur;
  tag_bitQ* q = (tag_bitQ*)&obj->info0a;
  int i;

  obj->w358a_ = 1;
  obj->bitQ_ = q;
  obj->w3588_ = (uint16_t)(obj->w3588_ | 4u);
  obj->shortPhase2_ = 0;
  obj->wabca_ = 0;
  for (i = 0; i < 10; i++)
    obj->wAbae_[i] = 0;
  obj->wAbc2_ = 0;

  txCur = obj->txState_;
  if ((uint16_t)txCur != V34HS_TX_DPSK)
  {
    if (dsplibs_debug_level > 1u)
      v04PrTx(obj, txCur, V34HS_TX_DPSK);
    obj->txState_ = V34HS_TX_DPSK;
  }
  mstCur = obj->mst_;
  if ((uint16_t)mstCur != V34HS_DET_SYNC)
  {
    if (dsplibs_debug_level > 1u)
      diPrMst(obj, mstCur, V34HS_DET_SYNC);
    obj->mst_ = V34HS_DET_SYNC;
  }

  obj->v21RxBuf_ = USHRT_MAX;
  obj->v21BitCnt_ = 0;
  q->tail_.crc = USHRT_MAX;
  q->tail_.pos = 0;
  q->tail_.widx = 0;
  q->tail_.pass = 0;
  q->tail_.total = (obj->modeChar_ == V34ROLE_CALL && obj->pcmMode_ != 0) ? 0x1e : 0x11;
  q->tail_.chunk = 8;
  q->tail_.crcOn = 1;
  q->tail_.acc = 0xf72u;
  q->tail_.restartAcc = 0xf72u;
  q->tail_.nbits = 0xc;
  q->tail_.restartBits = 0xc;
  q->tail_.loop = 1;
}

/**
 * @brief Extract the maximum-data-length field from the received rate bits.
 * @param obj  V.34 module object
 * @note  rxMdLen_ = bitreverse(((word0 & 3) << 5) | ((word1 & 0xf8) >> 3), 7), with word0 / word1 the first two words of info1c; printed as "RX MDLENGTH".
 */
static void diMdLength(tagV34Object* obj)
{
  unsigned w0 = diW16(((uint8_t*)&obj->info1c), 0);
  unsigned w1 = diW16(((uint8_t*)&obj->info1c), 2);
  unsigned b = ((w0 & 3u) << 5) | ((w1 & 0xf8u) >> 3);
  unsigned mdLen;
  obj->rxMdLen_ = (uint16_t)b;
  mdLen = bitreverse((unsigned short)b, 7);
  obj->rxMdLen_ = (uint16_t)mdLen;
  if (dsplibs_debug_level > 1u)
    dsplibs_debug_printf("RX MDLENGTH = %d\n", (int)(int16_t)mdLen);
}

/**
 * @brief A complete INFO0 message was received outside error recovery: hand it to V34GiveINFO0dBits() and pick the next microstate.
 * @param obj  V.34 module object
 * @note  Calling role: microstate RX_PHASE1_CALL.  Answering role: TX_PHASE1_ANS, or RX_PHASE1_ANS (with hsSymCnt_ = filtDelay_) when the short phase 2
 *        flag is set.  Clears flag 0x200 of rxFlags_.
 */
static void diInfo0Done(tagV34Object* obj)
{
  unsigned lv, mstCur;
  obj->hsSymCnt_ = 0;
  V34GiveINFO0dBits(obj, (const unsigned short*)&obj->info0d);
  lv = dsplibs_debug_level;
  if (lv > 1u)
  {
    diPrWords("V34INFO, rxinfo0", obj->bitQ2_->unk00_);
    lv = dsplibs_debug_level;
  }

  if (obj->modeChar_ == V34ROLE_CALL)
  {
    mstCur = obj->mst_;
    if ((uint16_t)mstCur != V34HS_RX_PHASE1_CALL)
    {
      if (lv > 1u)
      {
        diPrMst(obj, mstCur, V34HS_RX_PHASE1_CALL);
        lv = dsplibs_debug_level;
      }
      obj->mst_ = V34HS_RX_PHASE1_CALL;
    }
  }
  else
  {
    obj->v21RxBuf_ = USHRT_MAX;
    if (obj->shortPhase2_ == 0)
    {
      mstCur = obj->mst_;
      if ((uint16_t)mstCur != V34HS_TX_PHASE1_ANS)
      {
        if (lv > 1u)
        {
          diPrMst(obj, mstCur, V34HS_TX_PHASE1_ANS);
          lv = dsplibs_debug_level;
        }
        obj->mst_ = V34HS_TX_PHASE1_ANS;
      }
    }
    else
    {
      mstCur = obj->mst_;
      if ((uint16_t)mstCur != V34HS_RX_PHASE1_ANS)
      {
        if (lv > 1u)
        {
          diPrMst(obj, mstCur, V34HS_RX_PHASE1_ANS);
          lv = dsplibs_debug_level;
        }
        obj->mst_ = V34HS_RX_PHASE1_ANS;
      }
      obj->hsSymCnt_ = obj->filtDelay_;
    }
  }

  obj->rx.rxFlags_ = (uint16_t)(obj->rx.rxFlags_ & 0xfffffdffu);
  if (lv > 1u)
    dsplibs_debug_printf("V34INFO,%s info0 received in DET_INFO\n", obj->shortPhase2_ == 0 ? "" : " Short Phase 2");
  diTail(obj);
}

/**
 * @brief CRC mismatch on a received message: return to rxstate RX_DPSK / microstate DET_SYNC and re-arm the INFO0a detector.
 * @param obj  V.34 module object
 * @note  For the answering role the receive record length is reset to 0x1e bits first.
 */
static void diCrcError(tagV34Object* obj)
{
  unsigned rx, mstCur;
  tag_bitQ* q;

  if (dsplibs_debug_level > 1u)
    dsplibs_debug_printf("V34INFO, info0 CRC not received properly in DET_INFO\n");
  rx = obj->rxState_;
  if ((uint16_t)rx != V34HS_RX_DPSK)
  {
    if (dsplibs_debug_level > 1u)
      diPrRx(obj, rx, V34HS_RX_DPSK);
    obj->rxState_ = V34HS_RX_DPSK;
  }
  mstCur = obj->mst_;
  if ((uint16_t)mstCur != V34HS_DET_SYNC)
  {
    if (dsplibs_debug_level > 1u)
      diPrMst(obj, mstCur, V34HS_DET_SYNC);
    obj->mst_ = V34HS_DET_SYNC;
  }

  q = obj->bitQ2_;
  q->tail_.crc = USHRT_MAX;
  obj->v21RxBuf_ = USHRT_MAX;
  obj->v21BitCnt_ = 0;
  if (obj->modeChar_ == V34ROLE_ANSWER)
  {
    q->tail_.total = 0x1e;
    if (dsplibs_debug_level > 1u)
      dsplibs_debug_printf("SetINFO0dBits  \n");
  }
  diInfo0Rearm(obj);
  diTail(obj);
}

/**
 * @brief INFO1c (0x26-bit message) received by the calling side: check it, then set up the demodulator for the negotiated symbol rate and carrier.
 * @param obj  V.34 module object
 * @note  If V34GiveINFO1aBits() rejects the message the handler goes to rxstate WAIT / microstate INFODONE and stops.  Otherwise it extracts the data
 *        length (diMdLength), runs setfinalrate() and rxinit(), loads the baud-rate dependent filter constants (w410_ .. w422_) and the carrier dependent
 *        sine table (hsineTab_ / hsineLen_), restores the receive gain from rxGainEst_ and re-initialises the tone detector twice (windows 0xa and 0x78).
 *        The probe bins passed to V34GiveProbeResults() are `probe.bin_`.  The final debug print is labelled "rxinfo1a" although it prints
 *        the info1c record.
 */
static void diInfo1c(tagV34Object* obj)
{
  unsigned rx, mstCur, lv;
  int baud, carrier;

  V34GiveProbeResults(obj, obj->probe.bin_);
  if ((uint16_t)V34GiveINFO1aBits(obj, (const unsigned short*)obj->bitQ2_) != 0)
  {
    rx = obj->rxState_;
    if ((uint16_t)rx != V34HS_WAIT)
    {
      if (dsplibs_debug_level > 1u)
        diPrRx(obj, rx, V34HS_WAIT);
      obj->rxState_ = V34HS_WAIT;
    }
    mstCur = obj->mst_;
    if ((uint16_t)mstCur != V34HS_INFODONE)
    {
      if (dsplibs_debug_level > 1u)
        diPrMst(obj, mstCur, V34HS_INFODONE);
      obj->mst_ = V34HS_INFODONE;
    }
    diTail(obj);
    return;
  }

  diMdLength(obj);
  setfinalrate(obj);
  rx = obj->rxState_;
  if ((uint16_t)rx != V34HS_RECEIVE)
  {
    if (dsplibs_debug_level > 1u)
      diPrRx(obj, rx, V34HS_RECEIVE);
    obj->rxState_ = V34HS_RECEIVE;
  }
  rxinit(obj);
  lv = dsplibs_debug_level;
  baud = (int16_t)obj->rate.rxBaud_;
  carrier = (int16_t)obj->rate.rxCarrier_;
  if (lv > 1u)
  {
    dsplibs_debug_printf("V34SetupDemodulator: baudrate %ld, carrier %ld\n", (long)baud, (long)carrier);
    lv = dsplibs_debug_level;
  }

  /* baud-rate dependent constants */
  obj->rx.w38c_ = 4;
  switch (baud)
  {
  case V34_BAUD_2800:
    obj->rx.w414_ = 0x3e82;
    obj->rx.w412_ = 0x3594;
    obj->rx.w422_ = 0x3594;
    obj->rx.w410_ = 0x1f41;
    break;
  case V34_BAUD_2400:
    obj->rx.w414_ = 0x3e80;
    obj->rx.w412_ = 0x3e80;
    obj->rx.w422_ = 0x3e80;
    obj->rx.w410_ = 0x1f40;
    break;
  case V34_BAUD_2743:
    obj->rx.w414_ = 0x3e80;
    obj->rx.w412_ = 0x36b0;
    obj->rx.w422_ = 0x36b0;
    obj->rx.w410_ = 0x1f40;
    break;
  case V34_BAUD_3200:
    obj->rx.w414_ = 0x3e80;
    obj->rx.w412_ = 0x2ee0;
    obj->rx.w422_ = 0x2ee0;
    obj->rx.w410_ = 0x1f40;
    break;
  case V34_BAUD_3000:
    obj->rx.w414_ = 0x3e80;
    obj->rx.w412_ = 0x3200;
    obj->rx.w422_ = 0x3200;
    obj->rx.w410_ = 0x1f40;
    break;
  case V34_BAUD_3429:
    obj->rx.w414_ = 0x3e80;
    obj->rx.w412_ = 0x2bc0;
    obj->rx.w422_ = 0x2bc0;
    obj->rx.w410_ = 0x1f40;
    break;
  default:
    break;
  }

  /* carrier dependent sine table */
  switch (carrier)
  {
  case V34_CARRIER_1829:
    obj->rx.hsineTab_ = hsine1829;
    obj->rx.hsineLen_ = 0x15;
    break;
  case V34_CARRIER_1920:
    obj->rx.hsineTab_ = hsine1920;
    obj->rx.hsineLen_ = 5;
    break;
  case V34_CARRIER_1867:
    obj->rx.hsineTab_ = hsine1867;
    obj->rx.hsineLen_ = 0x24;
    break;
  case V34_CARRIER_1959:
    obj->rx.hsineTab_ = hsine1959;
    obj->rx.hsineLen_ = 0x31;
    break;
  case V34_CARRIER_2000:
    obj->rx.hsineTab_ = hsine2000;
    obj->rx.hsineLen_ = 0x18;
    break;
  case V34_CARRIER_1680:
    obj->rx.hsineTab_ = hsine1680;
    obj->rx.hsineLen_ = 0x28;
    break;
  case V34_CARRIER_1800:
    obj->rx.hsineTab_ = hsine1800;
    obj->rx.hsineLen_ = 0x10;
    break;
  case V34_CARRIER_1600:
    obj->rx.hsineTab_ = hsine1600;
    obj->rx.hsineLen_ = 6;
    break;
  default:
    break;
  }

  /* receive gain and tone detectors */
  {
    unsigned gain = obj->rx.rxGainEst_;
    obj->rx.w39e_ = 0x2000;
    obj->rx.rxGain_ = (uint16_t)gain;
    if (lv > 1u)
      dsplibs_debug_printf("V34AGC, setup receiver gain = 0x%x\n", (int)(int16_t)gain);
  }
  obj->rx.rxFlags_ = (uint16_t)(obj->rx.rxFlags_ & ~0xf00u);
  detectorinit(&obj->det, obj->rate.detTab_, 0, 8, 0xa, 0x600, 0);
  obj->rx.rxFlags_ = (uint16_t)(obj->rx.rxFlags_ | 0x200u);
  detectorinit(&obj->det, obj->rate.detTab_, 0, 8, 0x78, 0x600, 0);
  obj->rx.rxFlags_ = (uint16_t)(obj->rx.rxFlags_ | 0x200u);
  obj->hsSymCnt_ = 0;
  obj->waa80_ = 1;
  if (dsplibs_debug_level > 1u)
    diPrWords("V34PROBE, rxinfo1a", ((uint8_t*)&obj->info1c));
  diTail(obj);
}

/**
 * @brief INFO1a (0x4d-bit message) received by the answering side: derive the rates, choose the probe and queue the INFO1c reply.
 * @param obj  V.34 module object
 * @note  Forces txstate TX_DPSK, extracts the data length, runs probeselect() and V34GiveINFO1dBits() (a non-zero result stops the handler), then builds the
 *        INFO1c transmit queue (0x26 bits) in info and goes to microstate INFODONE. 
 *        The probe prints list the first ten words of info1c ("rxinfo1c") and info ("txinfo1a"). */
static void diInfo1a(tagV34Object* obj)
{
  unsigned txCur, mstCur, lv;
  tag_bitQ* q;

  txCur = obj->txState_;
  obj->tx.k56Idx_ = 0;
  obj->hsSymCnt_ = 0;
  if ((uint16_t)txCur != V34HS_TX_DPSK)
  {
    if (dsplibs_debug_level > 1u)
      v04PrTxX(txCur, V34HS_TX_DPSK, obj->rxState_, obj->mst_, 0, 0);
    obj->txState_ = V34HS_TX_DPSK;
  }

  diMdLength(obj);
  probeselect(obj);
  lv = dsplibs_debug_level;
  if (lv > 1u)
  {
    dsplibs_debug_printf("V34PROBESELECT, in ANSWER, txbaudrate = %d,rxbaudrate = %d,\n", (int)(int16_t)obj->rate.baud_, (int)(int16_t)obj->rate.rxBaud_);
    lv = dsplibs_debug_level;
  }
  if (lv > 1u)
    diPrWords("V34PROBE, rxinfo1c", ((uint8_t*)&obj->info1c));

  {
    tag_bitQ* q2 = obj->bitQ2_;
    obj->bitQ_ = (tag_bitQ*)&obj->info;
    if (V34GiveINFO1dBits(obj, (uint16_t*)q2) != 0)
    {
      diTail(obj);
      return;
    }
  }

  V34GiveProbeResults(obj, obj->probe.bin_);
  V34SetINFO1aBits(obj, (uint16_t*)obj->bitQ_);

  /* re-arm the transmit bit queue */
  q = obj->bitQ_;
  q->tail_.crc = USHRT_MAX;
  q->tail_.pos = 0;
  q->tail_.widx = 0;
  q->tail_.pass = 0;
  q->tail_.total = 0x26;
  q->tail_.chunk = 8;
  q->tail_.crcOn = 1;
  q->tail_.nbits = 0x10;
  q->tail_.restartBits = 0x10;
  q->tail_.loop = 0;
  mstCur = obj->mst_;
  q->tail_.acc = 0xff72u;
  q->tail_.restartAcc = 0xff72u;
  obj->toneSel_ = 0;

  if ((uint16_t)mstCur != V34HS_INFODONE)
  {
    lv = dsplibs_debug_level;
    if (lv > 1u)
    {
      diPrMst(obj, mstCur, V34HS_INFODONE);
      lv = dsplibs_debug_level;
    }
    obj->mst_ = V34HS_INFODONE;
  }
  else
  {
    lv = dsplibs_debug_level;
  }
  if (lv > 1u)
    diPrWords("V34PROBE, txinfo1a", ((uint8_t*)&obj->info));
  diTail(obj);
}

/**
 * @brief Music-on-hold message (8 bits) received: interpret it and set the next state.
 * @param obj  V.34 module object
 * @param q    value of bitQ2_ at handler entry (the first debug print reads it; the interpreter gets the member re-read)
 * @param cnt  (short)(hsSymCnt_ + 1), printed
 * @note  With mohF0_ == 1 the tone detector is re-armed (2400 Hz table for the calling role, 1200 Hz for the answering role) and the microstate becomes
 *        MOH_TONE_DROP.  Otherwise mohF8_ is set and the receiver returns to RX_DPSK / DET_SYNC.
 */
static void diMoh(tagV34Object* obj, tag_bitQ* q, int cnt)
{
  unsigned rx, mstCur;

  if (dsplibs_debug_level > 1u)
  {
    dsplibs_debug_printf("Received MOH message ! (count2=%d, data0=%X, data1=%X)\r\n", cnt, (unsigned)diW16(q->unk00_, 0), (unsigned)diW16(q->unk00_, 2));
    q = obj->bitQ2_;
  }
  VPcmV34InterpretMohMessageBits(obj, (const uint16_t*)q);

  if (obj->mohF0_ == 1)
  {
    detectorinit(&obj->det, obj->modeChar_ == V34ROLE_CALL ? c2400_ : c1200_, 1, 0x64, 0x32, 0x800, 0x400);
    obj->det.toneDetF6_ = 1;
    mstCur = obj->mst_;
    if ((uint16_t)mstCur != V34HS_MOH_TONE_DROP)
    {
      if (dsplibs_debug_level > 1u)
        diPrMst(obj, mstCur, V34HS_MOH_TONE_DROP);
      obj->mst_ = V34HS_MOH_TONE_DROP;
    }
    diTail(obj);
    return;
  }

  rx = obj->rxState_;
  obj->mohF8_ = 1;
  if ((uint16_t)rx != V34HS_RX_DPSK)
  {
    if (dsplibs_debug_level > 1u)
      diPrRx(obj, rx, V34HS_RX_DPSK);
    obj->rxState_ = V34HS_RX_DPSK;
  }
  mstCur = obj->mst_;
  if ((uint16_t)mstCur != V34HS_DET_SYNC)
  {
    if (dsplibs_debug_level > 1u)
      diPrMst(obj, mstCur, V34HS_DET_SYNC);
    obj->mst_ = V34HS_DET_SYNC;
  }
  obj->bitQ2_->tail_.crc = USHRT_MAX;
  obj->v21RxBuf_ = USHRT_MAX;
  diTail(obj);
}

/**
 * @brief A good message of any other length: INFO0.
 * @param obj  V.34 module object
 * @param q    value of bitQ2_ at handler entry; the received words are read from it
 * @note  In error-recovery sub-state 1 the received words are saved in wAbae_ (count in wAbc2_) and the transmit queue gets its flag bit 0x80; if the
 *        receive record does not have that flag the receiver returns to RX_DPSK / DET_SYNC and waits for the next sync.  In every other case diInfo0Done()
 *        takes over.
 * @warning  The word count (derived from hsSymCnt_) is not bounded by the 10 words of wAbae_ (see diWabae()).
 */
static void diInfo0(tagV34Object* obj, tag_bitQ* q)
{
  if (obj->w358a_ == 1)
  {
    tag_bitQ* bq = obj->bitQ_;
    unsigned a = diW16(bq->unk00_, 4);
    unsigned symCnt, cnt;
    int n, r;

    if (!(a & 0x80u))
    {
      uint16_t v;
      bq->tail_.pass = 0;
      v = (uint16_t)(a | 0x80u);
      __builtin_memcpy(bq->unk00_ + 4, &v, 2);
    }

    symCnt = obj->hsSymCnt_;
    n = (int16_t)symCnt;
    r = n - (((n < 0) ? n + 7 : n) & ~7);
    cnt = (unsigned)(((n - (int16_t)r) >> 3) + ((symCnt & 7u) != 0 ? 1 : 0));
    obj->wAbc2_ = (uint16_t)cnt;
    if ((int16_t)cnt > 0)
    {
      int16_t i = 0;
      do
      {
        *diWabae(obj, i) = diW16(q->unk00_, 2 * i);
        i = (int16_t)(i + 1);
      } while ((int16_t)cnt > i);
    }

    if (!(q->unk00_[4] & 0x80u))
    {
      unsigned rx, mstCur;
      rx = obj->rxState_;
      if ((uint16_t)rx != V34HS_RX_DPSK)
      {
        if (dsplibs_debug_level > 1u)
          dsplibs_debug_printf(V04_FMT_RX, StateName[(int16_t)rx], StateName[V34HS_RX_DPSK], StateName[(int16_t)obj->txState_], StateName[(int16_t)obj->mst_],
                               v04K56(obj), (long)(int16_t)symCnt);
        obj->rxState_ = V34HS_RX_DPSK;
      }
      mstCur = obj->mst_;
      if ((uint16_t)mstCur != V34HS_DET_SYNC)
      {
        if (dsplibs_debug_level > 1u)
          diPrMst(obj, mstCur, V34HS_DET_SYNC);
        obj->mst_ = V34HS_DET_SYNC;
      }
      obj->bitQ2_->tail_.crc = USHRT_MAX;
      obj->v21RxBuf_ = USHRT_MAX;
      return;
    }
  }
  diInfo0Done(obj);
}

/**
 * @brief Microstate handler for DET_INFO: consume one INFO bit and, at the end of a message, check its CRC and act on it.
 * @param obj  V.34 module object
 * @note  Returns at once when v21BitCnt_ is 0 (no bit due).  Clears count3_ on every call.  The CRC register is updated only while hsSymCnt_ is below the
 *        message length total; the 16 CRC bits themselves are shifted into v21RxBuf_ and compared after the last one.
 */
extern "C" void v04_t3_det_info(tagV34Object* obj)
{
  unsigned cnt, w18;
  tag_bitQ* q;
  int bitCnt;

  obj->count3_ = 0;
  cnt = obj->v21BitCnt_;
  if ((uint16_t)cnt == 0)
    return;

  /* CRC update for this bit */
  q = obj->bitQ2_;
  obj->v21BitCnt_ = (uint16_t)(cnt - 1u);
  cnt = obj->hsSymCnt_;
  if ((int16_t)cnt < (int16_t)q->tail_.total)
  {
    unsigned crc = q->tail_.crc;
    unsigned msb = (int16_t)crc < 0 ? 1u : 0u;
    crc = crc + crc;
    if (obj->v21RxBuf_ & 1u)
      msb ^= 1u;
    if (msb != 0)
      crc ^= 0x1021u;
    q->tail_.crc = (uint16_t)crc;
    cnt = obj->hsSymCnt_;
  }

  cnt = cnt + 1u;
  bitCnt = (int16_t)cnt;
  obj->hsSymCnt_ = (uint16_t)cnt;
  w18 = q->tail_.total;

  /* message complete: check the CRC and dispatch on the message length */
  if (bitCnt == (int)(int16_t)w18 + 0x10)
  {
    if (obj->v21RxBuf_ != q->tail_.crc)
    {
      diCrcError(obj);
      return;
    }
    if ((uint16_t)w18 == 0x4d)
    {
      diInfo1a(obj);
      return;
    }
    if ((uint16_t)w18 == 0x26)
    {
      diInfo1c(obj);
      return;
    }
    if ((uint16_t)w18 == 8)
    {
      diMoh(obj, q, bitCnt);
      return;
    }
    diInfo0(obj, q);
    return;
  }
  diTail(obj);
}
