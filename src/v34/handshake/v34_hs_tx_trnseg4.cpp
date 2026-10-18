/**
 * @file v34_hs_tx_trnseg4.cpp
 * @brief V.34 handshake transmit state TRNSEG4: the 4th training segment and the receiver set-up at its end.
 *
 * One entry point, v04_tx_trnseg4(), the case for transmit state 0x15 of the per-sample loop of v34handshak() (v34_hs_dispatch.cpp).  Callees:
 * txmit(), VPcmV34ReportStartOfEchoAdapt(), VPcmV34ReportMiddleOfEchoAdapt(), V34EchoReportCoeff() x2, rxinit(), detectorinit(),
 * indicateJaTransmission().
 *
 * Every call sends the next scrambled point (two-step scrambler with the constant input 3; txBits_ = the top 3 bits of the scrambler, point
 * vect4[top bits]) and counts txCnt_.  Events, in txCnt_ order:
 * - txCnt_ == hsSymCnt_: the echo adaptation starts (VPcmV34ReportStartOfEchoAdapt(), txFlags_ bit 2 and four echo words cleared).
 * - txCnt_ >= period_: txFlags_ bit 8 is set.
 * - txCnt_ == echoMid_ / 2: the middle of the echo adaptation is reported.
 * - txCnt_ == 1.5 * baud_: txFlags_ gets bit 10 and bit 15 is cleared.
 * - txCnt_ == echoMid_: the segment ends (endOfSegment()): transmit state JTXMIT (0x40); the echo-freeze delay hsSymCnt_ is derived from bulkDelay_
 *   (all of it for V.90 / K56Flex, half for plain V.34; bulkDelay_ <= 2 freezes the echo canceller immediately and reports its coefficients);
 *   startReceiver() starts the receiver in RECEIVE (rxinit() and the demodulator parameters: per receive baud rate the words w410_ .. w422_, per
 *   carrier the sine table hsineTab_ / hsineLen_, the receive gain, the tone detector set-up) and the microstate becomes DET_SYNC (0x29).  V.90
 *   (pcmMode_ > 1) continues in JaTXMIT (0x4e), K56Flex (k56Mode_ > 1) in K56JaTXMIT (0x55), both with receive state WAIT and
 *   indicateJaTransmission().
 *
 * @note The debug level is held in one local copy between the transmit-state print and the receiver set-up (re-read after prints).  The baud and
 *       carrier ladders have no default action: an unknown value leaves the words untouched.  The compares of txCnt_ with hsSymCnt_, period_ and
 *       echoMid_ are 16-bit / signed.
 */

#include <limits.h>
#include "v34_handshake.h"
#include "v34/v34_internal.h"
#include "vpcm/vpcm_session_wrappers.h"

#include "v34/v34_init_tables.h" /* hsine1600 ... hsine2000 */

/**
 * @brief Receiver set-up at the end of TRNSEG4: start the receiver and, for V.90 / K56Flex, continue in the Ja transmit state.
 * @param obj  handshake object; sets the receive state RECEIVE, the demodulator words, hsineTab_, rxGain_, the tone detector, q2 and the microstate
 * @note  The local `L` is the debug-level copy used for the set-up prints.
 */
static void startReceiver(tagV34Object* obj)
{
  unsigned L;
  int baud;
  int carrier;

  /* receive state RECEIVE and the receive front end */
  obj->tx.k56Word_ = 0x8990;
  obj->tx.txPrev_ = obj->tx.txBits_;
  v04SetRx(obj, obj->rxState_, V34HS_RECEIVE);
  rxinit(obj);
  L = dsplibs_debug_level;
  baud = (int16_t)obj->rate.rxBaud_;
  carrier = (int16_t)obj->rate.rxCarrier_;
  if (L > 1u)
  {
    dsplibs_debug_printf("V34SetupDemodulator: baudrate %ld, carrier %ld\n", (long)baud, (long)carrier);
    L = dsplibs_debug_level;
  }

  /* demodulator parameters per receive baud rate */
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

  /* receive sine table per carrier */
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

  /* receive gain and the tone detector for the start of DET_SYNC */
  {
    unsigned gain = obj->rx.rxGainEst_;
    obj->rx.w39e_ = 0x2000;
    obj->rx.rxGain_ = (uint16_t)gain;
    if (L > 1u)
      dsplibs_debug_printf("V34AGC, setup receiver gain = 0x%x\n", (int)(int16_t)gain);
  }
  obj->rx.rxFlags_ = (uint16_t)(obj->rx.rxFlags_ & ~0xf00u);
  detectorinit(&obj->det, obj->rate.detTab_, 0, 8, 0xa, 0x600, 0);
  obj->rx.rxFlags_ = (uint16_t)(obj->rx.rxFlags_ | 0x200u);

  obj->q2.q2Tail_.acc = 0;
  obj->q2.q2Tail_.restartAcc = 0;
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

  /* microstate DET_SYNC; V.90 / K56Flex go on in their Ja transmit state with the receiver waiting */
  {
    unsigned mst = obj->mst_;
    if ((uint16_t)mst != V34HS_DET_SYNC)
    {
      if (dsplibs_debug_level > 1u)
        v04PrMstX(mst, V34HS_DET_SYNC, obj->txState_, obj->rxState_, v04K56(obj), v04W78(obj));
      obj->mst_ = V34HS_DET_SYNC;
    }
  }
  obj->tx.k56Idx_ = 0;
  obj->waa80_ = 1;

  if ((uint32_t)obj->pcmMode_ > 1u)
  {
    v04SetTx(obj, obj->txState_, V34HS_JaTXMIT);
    v04SetRx(obj, obj->rxState_, V34HS_WAIT);
    indicateJaTransmission(obj);
    return;
  }

  if ((uint32_t)obj->k56Mode_ > 1u)
  {
    v04SetTx(obj, obj->txState_, V34HS_K56JaTXMIT);
    v04SetRx(obj, obj->rxState_, V34HS_WAIT);
    indicateJaTransmission(obj);
  }
}

/**
 * @brief End of the segment: transmit state JTXMIT and the echo-freeze delay, then the receiver.
 * @param obj  handshake object; sets txState_, hsSymCnt_ (echo-freeze countdown, 0 when frozen at once) and txFlags_ bit 2 on an immediate freeze
 */
static void endOfSegment(tagV34Object* obj)
{
  unsigned tx = obj->txState_;
  unsigned L = dsplibs_debug_level;
  unsigned w7e;

  if ((uint16_t)tx != V34HS_JTXMIT)
  {
    if (L > 1u)
    {
      v04PrTx(obj, tx, V34HS_JTXMIT);
      L = dsplibs_debug_level;
    }
    obj->txState_ = V34HS_JTXMIT;
  }

  /* echo-freeze delay from the bulk delay; a bulk delay of 2 or less freezes the echo canceller at once */
  w7e = obj->bulkDelay_;
  if ((int16_t)w7e > 2)
  {
    if (obj->pcmMode_ != 0)
      obj->hsSymCnt_ = (uint16_t)w7e;
    else if (obj->k56Mode_ != 0)
      obj->hsSymCnt_ = (uint16_t)w7e;
    else
      obj->hsSymCnt_ = (uint16_t)((int16_t)w7e >> 1);
    if (L > 1u)
      dsplibs_debug_printf("V34Hshak: On J TX start, would freeze EC after bulk delay (%d samples, bulk=%d)\r\n", (int)(int16_t)obj->hsSymCnt_,
                           (int)(int16_t)w7e);
  }
  else
  {
    if (L > 1u)
    {
      dsplibs_debug_printf("V34HSHAK: Freeze EC\n");
      L = dsplibs_debug_level;
    }
    obj->tx.txFlags_ = (uint16_t)(obj->tx.txFlags_ | 4u);
    if (L > 1u)
      dsplibs_debug_printf("==== Near Echo Canceller report ======\n");
    V34EchoReportCoeff(&obj->echo0);
    if (dsplibs_debug_level > 1u)
      dsplibs_debug_printf("==== Far Echo Canceller report ======\n");
    V34EchoReportCoeff(&obj->echo1);
    obj->hsSymCnt_ = 0;
  }

  startReceiver(obj);
}

/**
 * @brief Transmit state TRNSEG4 (0x15): send one scrambled point and run the segment's timeline.
 * @param obj  handshake object; reads scrState_, txCnt_, hsSymCnt_, period_, echoMid_, baud_, writes txBits_, txPoint_, txFlags_ and the echo words
 * @bug   The point is vect4[scrambler >> 29], a 3-bit value (0..7), but vect4 has only 4 entries: for values 4..7 the read goes past the table
 *        (candidate defect, read from the code; not run).
 */
extern "C" void v04_tx_trnseg4(tagV34Object* obj)
{
  const bool tap13 = obj->modeChar_ == V34ROLE_CALL;
  uint32_t e = v04Scramble(obj->tx.scrState_, tap13, 3);
  unsigned cnt;
  unsigned fl;
  int c;
  int32_t mid;

  /* the scrambled point */
  obj->tx.scrState_ = e;
  obj->tx.txBits_ = (uint16_t)(e >> 29);
  obj->tx.txPoint_ = (uint32_t)vect4[e >> 29];
  txmit(obj);

  /* start of the echo adaptation */
  cnt = (uint16_t)(obj->tx.txCnt_ + 1u);
  obj->tx.txCnt_ = (uint16_t)cnt;
  if (obj->hsSymCnt_ == (uint16_t)cnt)
  {
    obj->tx.txFlags_ = (uint16_t)(obj->tx.txFlags_ & ~4u);
    obj->echoAdaptCnt_ = 0;
    obj->echoAlphaNear_ = 0;
    obj->echoAlphaFar_ = 0;
    obj->echoEnergy_ = 0;
    VPcmV34ReportStartOfEchoAdapt(obj);
  }

  fl = obj->tx.txFlags_;
  if ((fl & 0x100u) == 0)
  {
    if ((int16_t)obj->tx.txCnt_ >= (int16_t)obj->rate.period_)
      obj->tx.txFlags_ = (uint16_t)(fl | 0x100u);
  }

  /* timeline events: middle of the echo adaptation, 1.5 baud, segment end */
  c = (int16_t)obj->tx.txCnt_;
  mid = obj->echoMid_;
  if (c == (mid >> 1))
  {
    VPcmV34ReportMiddleOfEchoAdapt(obj);
    return;
  }
  {
    int b = (int16_t)obj->rate.baud_;
    if (c == b + (b >> 1))
    {
      obj->tx.txFlags_ = (uint16_t)((obj->tx.txFlags_ & 0x7fffu) | 0x400u);
      return;
    }
  }
  if (c != mid)
    return;
  endOfSegment(obj);
}
