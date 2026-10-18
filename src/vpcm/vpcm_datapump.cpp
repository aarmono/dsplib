/**
 * @file vpcm_datapump.cpp
 * @brief datapumpv34: the per-block V.34 data pump.
 *
 * Runs the handshake / transmit side (v34handshak or modulatevector) until the transmit sample index has caught up with the block length, then the
 * receive side (receiver) until the receive front end has nothing left to do, and finally decides whether a retrain or a rate renegotiation (RRN) has
 * to be started (v34handshakinit with VPcmV34IndicateLocalRRN / VPcmV34IndicateRemoteRRN).
 *
 * Details worth knowing:
 * - The RX front-end sub-object is part of the V.34 object; all its words are named members of tagV34Object.
 * - `(uint32)(sampleTime_ - dataStartTime_) > 0x46500` sets state_ = 5 once at entry; the same difference is re-read after every receiver() call
 *   against two thresholds, 0x23280 and 0x119400 (unsigned).
 * - runMode_ is compared unsigned (`> 1`): 0 / 1 = the handshake runs through modulatevector, otherwise through v34handshak.
 * - Every 16-bit compare is on the signed low 16 bits (txIdx_ < blkLen_, rxW0_ > 5, w388_ <= 0x752f, equErr_ against the three error thresholds, the
 *   counters against 2 * and 8 * rxBaud_).
 * - retrainCnt_ / rrnDownCnt_ are 16-bit: condition ? old + 1 : 0; rrnUpCnt_ = (error >= rrnUpErrThr_) ? 0 : old + 1; retrainCnt_ is stored before
 *   the sample-time difference is read.
 * - The debug guard is the unsigned `level > 1` test, read after the stores of the same block; rxBaud_ is reloaded after every call and every
 *   reason block.
 */

#include "v34/v34_object.h"

#include "core/dsp_debug_level.h" /* dsplibs_debug_level, LEVEL_ON() */
#include "core/dsp_host.h"
#include "v34/v34_api.h"
#include "vpcm_v34_api.h"

extern "C"
{
}

/** @brief Sign-extend a 16-bit member (the counters and thresholds are compared as signed 16-bit values).  @param v value */
static inline int s16(uint16_t v) { return (int)(int16_t)v; }

/**
 * @brief Run the V.34 data pump for one block.
 *
 * Transmit side first (handshake or modulator until txIdx_ reaches blkLen_), then the receiver while rxW0_ > 5, updating the retrain / rate
 * renegotiation error counters after each receiver() call; finally start a retrain (error run longer than half a baud, or rx flag 0x40), a remote
 * RRN (rx flag 0x20), a downward local RRN (large error run) or an upward local RRN (small error run).
 * @param obj V.34 module object
 */
extern "C" void datapumpv34(tagV34Object* obj)
{
  uint32_t diff0 = (uint32_t)obj->sampleTime_ - (uint32_t)obj->dataStartTime_;
  if (diff0 > 0x46500u)
    obj->state_ = 5;

  if ((uint32_t)obj->mod.runMode_ > 1u)
  {
    /* handshake / full-rate run */
    while (s16(obj->tx.txIdx_) < s16(obj->tx.blkLen_) || s16((uint16_t)obj->rx.rxW0_) > 5)
      v34handshak(obj);
    return;
  }

  /* plain transmit: fill the block, then the receive side */
  while (s16(obj->tx.txIdx_) < s16(obj->tx.blkLen_))
    modulatevector(obj);

  while (s16((uint16_t)obj->rx.rxW0_) > 5)
  {
    if (s16(obj->rx.w388_) <= 0x752f)
      obj->rx.w388_ = (uint16_t)(obj->rx.w388_ + 1);
    receiver(obj);
    {
      int snr = s16(obj->rx.equErr_);
      uint32_t d = (uint32_t)obj->sampleTime_ - (uint32_t)obj->dataStartTime_;
      obj->rx.retrainCnt_ = snr > s16(obj->rx.retrainErrThr_) ? (uint16_t)(obj->rx.retrainCnt_ + 1u) : (uint16_t)0;
      if (d > 0x23280u)
        obj->rx.rrnDownCnt_ = snr > s16(obj->rx.rrnDownErrThr_) ? (uint16_t)(obj->rx.rrnDownCnt_ + 1u) : (uint16_t)0;
      if (d > 0x119400u)
        obj->rx.rrnUpCnt_ = snr >= s16(obj->rx.rrnUpErrThr_) ? (uint16_t)0 : (uint16_t)(obj->rx.rrnUpCnt_ + 1u);
    }
  }

  /* retrain / rate renegotiation decision */
  {
    unsigned flags = obj->rx.rxFlags_;
    bool remote;
    if ((flags & 0x40u) != 0 || s16(obj->rx.retrainCnt_) > (s16(obj->rate.rxBaud_) >> 1))
    {
      /* retrain: the error run exceeded half a baud, or the retrain bit is set */
      v34handshakinit(obj, V34HSINIT_RETRAIN);
      {
        int half = s16(obj->rate.rxBaud_) >> 1;
        obj->mod.runMode_ = (int32_t)((s16(obj->rx.retrainCnt_) <= half ? 1u : 0u) +
                                      2u); /* V34RUN_RETRAIN (2) when the error run passed baud / 2, else V34RUN_RETRAIN_REQ (3) */
      }
      obj->rx.retrainCnt_ = 0;
      obj->rx.rrnDownCnt_ = 0;
      obj->rx.rrnUpCnt_ = 0;
      if (LEVEL_ON())
        dsplibs_debug_printf("V34TIMING,Retrain started\n");
      flags = obj->rx.rxFlags_;
    }
    remote = (flags & 0x20u) != 0;
    if (remote)
    {
      /* remote RRN detected */
      v34handshakinit(obj, V34HSINIT_RNEG_REMOTE);
      obj->rx.rrnUpCnt_ = 0;
      obj->mod.runMode_ = V34RUN_RRN_REMOTE;
      obj->rx.retrainCnt_ = 0;
      obj->rx.rrnDownCnt_ = 0;
      obj->rx.rnegReason_ = 1;
      obj->rx.rnegRateIdx_ = obj->rate.rxRateIdx_;
      if (LEVEL_ON())
        dsplibs_debug_printf("V34RNEG, rate renegotiation detected \n");
      VPcmV34IndicateRemoteRRN(obj);
    }
    {
      unsigned baudNow = obj->rate.rxBaud_;
      if (s16(obj->rx.rrnDownCnt_) > 2 * s16((uint16_t)baudNow))
      {
        /* large error: rate renegotiation DOWN */
        v34handshakinit(obj, V34HSINIT_RNEG_LOCAL);
        obj->mod.runMode_ = V34RUN_RRN_LOCAL;
        obj->rx.retrainCnt_ = 0;
        obj->rx.rrnDownCnt_ = 0;
        obj->rx.rrnUpCnt_ = 0;
        obj->rx.rnegReason_ = 2;
        obj->rx.rnegRateIdx_ = obj->rate.rxRateIdx_;
        if (LEVEL_ON())
          dsplibs_debug_printf("V34RNEG, rate renegotiation DOWN initiated due to large error \n");
        VPcmV34IndicateLocalRRN(obj);
        baudNow = obj->rate.rxBaud_;
      }
      if (s16(obj->rx.rrnUpCnt_) > (s16((uint16_t)baudNow) << 3))
      {
        /* small error: rate renegotiation UP */
        v34handshakinit(obj, V34HSINIT_RNEG_LOCAL);
        obj->rx.rrnUpCnt_ = 0;
        obj->mod.runMode_ = V34RUN_RRN_LOCAL;
        obj->rx.retrainCnt_ = 0;
        obj->rx.rrnDownCnt_ = 0;
        obj->rx.rnegReason_ = 3;
        obj->rx.rnegRateIdx_ = obj->rate.rxRateIdx_;
        if (LEVEL_ON())
          dsplibs_debug_printf("V34RNEG, rate renegotiation UP initiated due to small error \n");
        VPcmV34IndicateLocalRRN(obj);
      }
    }
  }
}
