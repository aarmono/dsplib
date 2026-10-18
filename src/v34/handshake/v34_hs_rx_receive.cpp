/**
 * @file v34_hs_rx_receive.cpp
 * @brief V.34 handshake receive state RECEIVE: the main receiver of the INFO / MP phases.
 *
 * Entry point v04_rx_receive(), called by the post-loop receive chain of v34handshak() (v34_hs_dispatch.cpp) when the receive state is RECEIVE; the
 * caller runs the transmit-state tail afterwards.  The handler needs nothing from the dispatcher.
 *
 * Every call runs the demodulator receiver().  The receive flags rxFlags_ and the symbol counter w388_ (saturating at 0x61a8) then drive the
 * sub-states.  Flag bits used here: 0x40 retrain request, 0x80 retrain veto, 0x08 / 0x10 / 0x90 / 0x98 MP-sequence progress, 0x400 INFO sent,
 * 0x100 / 0x200 tone detector armed.
 *  - retrain: flag 0x40, or a counter beyond 7 * rxBaud_ without flag 0x80, calls v34handshakinit(obj, V34HSINIT_RETRAIN);
 *  - pattern search (flags without 0x10, counter above 0xbb8): a 16-bit shift register of the two bits received per call is compared with the
 *    start patterns 0x8990 / 0x89b0 (then flag 0x08) and 0x899f (then flag 0x10);
 *  - MP bit-serial receiver (flag 0x10 with 0x400): the bits of the bit queue q2 (shift register acc, bit count nbits) are consumed; in
 *    microstate DET_SYNC bit by bit, counting runs of ones in pos (more than 0x13 ones: the E sequence), otherwise as 17-bit words stored in
 *    q2 (index widx).  A complete MP (4 words) or extended MP (10 words) sets the progress flags, copies the six precoder coefficients to the
 *    transmitter filter block precCoef_ and returns to microstate DET_SYNC;
 *  - tone ('S') detection (tone_detect() with the tone detector armed) starts the MD-length wait: the detector words toneCnt_ = 0x1e, toneState_ = 3, toneEnergy_ = 0, then either
 *    rxMdLen_ is scaled or the rate-dependent scaling of the word `waacc_` is applied (sDetected());
 *  - MD-length wait (toneState_ is 3): when the counter exceeds rxMdLen_ the receiver is restarted (restartReceiver());
 *  - flags & 0x98 == 8 for the calling role: end of phase 3 (phase3End()).
 *
 * The control flow of v04_rx_receive() is a set of labelled blocks joined by goto; the labels are described at their definitions.
 */

#include <limits.h>

#include "v34_handshake.h"
#include "core/dsp_access.h"
#include "v34/v34_internal.h"
#include "vpcm/vpcm_v34_api.h"
#include "v34/v34_api.h"
#include "v34/v34_init_tables.h" /* hsine1600 ... hsine2000 */

#include "v34_hs_bytes.h"

/** Print a receive-state transition; the transmit state and microstate names are read from the object at print time. */
static inline void rcPrRx(tagV34Object* obj, unsigned old, int nw)
{
  dsplibs_debug_printf(V04_FMT_RX, StateName[(int16_t)old], StateName[nw], StateName[(int16_t)obj->txState_], StateName[(int16_t)obj->mst_], v04K56(obj),
                       v04W78(obj));
}
/** Print a microstate transition; the transmit and receive state names are read from the object at print time. */
static inline void rcPrMst(tagV34Object* obj, unsigned old, int nw) { v04PrMstX(old, nw, obj->txState_, obj->rxState_, v04K56(obj), v04W78(obj)); }

/**
 * @brief Copy the six coefficient words of the received MP record to the transmitter's filter block.
 * @param obj  V.34 module object
 * @note  Reads the 16-bit words waa10_ .. waa1a_ of q2 (byte offsets 4, 6, 8, 0xa, 0xc, 0xe) and writes precCoef_ (12 words): [0..5] = w4, -w6, w8, -wa, wc, -we and
 *        [6..11] = w6, w4, wa, w8, we, wc (the layout txrxdmainit() builds).  The negation keeps only the low 16 bits.
 */
static void copyCoefs(tagV34Object* obj)
{
  int16_t* p = obj->tx.precCoef_;
  unsigned c4 = obj->q2.waa10_, c6 = obj->q2.waa12_, c8 = obj->q2.waa14_, ca = obj->q2.waa16_, cc = obj->q2.waa18_, ce = obj->q2.waa1a_;
  p[7] = (int16_t)c4;
  p[0] = (int16_t)c4;
  p[1] = (int16_t)(0u - c6);
  p[6] = (int16_t)c6;
  p[9] = (int16_t)c8;
  p[2] = (int16_t)c8;
  p[3] = (int16_t)(0u - ca);
  p[8] = (int16_t)ca;
  p[11] = (int16_t)cc;
  p[4] = (int16_t)cc;
  p[5] = (int16_t)(0u - ce);
  p[10] = (int16_t)ce;
}

/** Debug print of ten consecutive 16-bit words of an MP record with the format `fmt` (ten %x conversions). */
static void printTen(const char* fmt, const uint8_t* q)
{
  unsigned w[10];
  int i;
  for (i = 0; i < 10; i++)
    w[i] = ld16(q, 2 * i);
  dsplibs_debug_printf(fmt, w[0], w[1], w[2], w[3], w[4], w[5], w[6], w[7], w[8], w[9]);
}

/**
 * @brief Select the receive filter words for a symbol rate and the handshake sine table for a carrier.
 * @param obj      V.34 module object
 * @param baud     symbol-rate selector (V34_BAUD_*); an unknown value leaves the filter words unchanged
 * @param carrier  carrier selector (V34_CARRIER_*); an unknown value leaves hsineTab_ / hsineLen_ unchanged
 * @note  Writes w410_, w412_, w414_, w422_ per rate, and hsineTab_ / hsineLen_ (the table and its length) per carrier.
 */
static void ladders(tagV34Object* obj, int baud, int carrier)
{
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
}

/**
 * @brief End of the MD-length wait: re-initialise the receiver for the negotiated rate and re-arm the tone detector.
 * @param obj  V.34 module object
 * @note  Calls rxinit(), applies ladders() for rxBaud_ / rxCarrier_, loads rxGain_ from rxGainEst_, clears flags 0xf00, runs detectorinit() twice
 *        (the second call replaces the first configuration, with waa80_ cleared in between) and clears rxMdLen_ and w25da_.
 */
static void restartReceiver(tagV34Object* obj)
{
  unsigned level;
  int baud, carrier;

  rxinit(obj);
  level = dsplibs_debug_level;
  baud = (int16_t)obj->rate.rxBaud_;
  carrier = (int16_t)obj->rate.rxCarrier_;
  if (level > 1u)
  {
    dsplibs_debug_printf("V34SetupDemodulator: baudrate %ld, carrier %ld\n", (long)baud, (long)carrier);
    level = dsplibs_debug_level;
  }
  obj->rx.w38c_ = 4;
  ladders(obj, baud, carrier);

  /* receiver gain from the estimate */
  {
    unsigned gain = obj->rx.rxGainEst_;
    obj->rx.w39e_ = 0x2000;
    obj->rx.rxGain_ = (uint16_t)gain;
    if (level > 1u)
      dsplibs_debug_printf("V34AGC, setup receiver gain = 0x%x\n", (int)(int16_t)gain);
  }

  /* re-arm the tone detector (two configurations in turn) */
  obj->rx.rxFlags_ = (uint16_t)(obj->rx.rxFlags_ & ~0xf00u);
  detectorinit(&obj->det, obj->rate.detTab_, 0, 8, 0xa, 0x600, 0);
  obj->rx.rxFlags_ = (uint16_t)(obj->rx.rxFlags_ | 0x200u);
  obj->waa80_ = 0;
  detectorinit(&obj->det, obj->rate.detTab_, 0, 0xa, 0x28, 0x600, 0);
  obj->rx.rxFlags_ = (uint16_t)(obj->rx.rxFlags_ | 0x200u);

  obj->rxMdLen_ = 0;
  obj->tx.w25da_ = 0;
}

/**
 * @brief Follow-up of a detected 'S' tone: start the MD-length wait or rescale the timing word.
 * @param obj  V.34 module object
 * @note  Always sets the detector words toneCnt_, toneState_ and toneEnergy_ (0x1e, 3, 0), clears the symbol counter w388_ and counts w25da_.  If rxMdLen_ is non-zero it becomes
 *        (rxBaud_ * 0x23d >> 14) * rxMdLen_ + 0x1e (16 bits).  Otherwise w0424_ is set to 1, flags 0x300 are cleared and, with flag 0x10
 *        clear, the 32-bit word `waacc_` is scaled by a symbol-rate dependent factor (3429: 0xb3/256, 3200: 0xc0/256, 3000: 205/256,
 *        2800: 0xdb/256; other rates unchanged); with flag 0x10 set it takes w49c_ instead.
 */
static void sDetected(tagV34Object* obj)
{
  unsigned md;

  /* enter the MD-length wait */
  obj->det.toneCnt_ = 0x1e;
  obj->det.toneState_ = 3;
  obj->det.toneEnergy_ = 0;
  obj->rx.w388_ = 0;
  obj->tx.w25da_ = (uint16_t)(obj->tx.w25da_ + 1u);

  md = obj->rxMdLen_;
  if ((uint16_t)md != 0)
  {
    /* a previous MD length is known: stretch the wait by the symbol-rate factor */
    unsigned value = (unsigned)((((int)(int16_t)obj->rate.rxBaud_ * 0x23d) >> 14) * (int)md + 0x1e);
    obj->rxMdLen_ = (uint16_t)value;
    if (dsplibs_debug_level > 1u)
      dsplibs_debug_printf("RX MDLENGTH, getting into waiting state for MD = %d\n", (int)(int16_t)value);
    return;
  }

  /* first detection: no MD length yet; reset the timing detector state */
  {
    unsigned value = obj->rx.rxFlags_;
    obj->rx.w0424_ = 1;
    value &= 0xfffffcffu;
    obj->rx.rxFlags_ = (uint16_t)value;
    if (value & 0x10u)
    {
      obj->waacc_ = obj->rx.w49c_;
      return;
    }
  }

  /* rescale the stored word for the symbol rate */
  {
    unsigned value = obj->rate.rxBaud_;
    int32_t x = (int32_t)obj->waacc_;
    if ((uint16_t)value == V34_BAUD_3429)
      obj->waacc_ = ((uint32_t)(int32_t)((int32_t)((uint32_t)x * 0xb3u) >> 8));
    else if ((uint16_t)value == V34_BAUD_3200)
      obj->waacc_ = ((uint32_t)(int32_t)((int32_t)(((uint32_t)x * 3u) << 6) >> 8));
    else if ((uint16_t)value == V34_BAUD_3000)
      obj->waacc_ = ((uint32_t)(int32_t)((int32_t)((uint32_t)x * 205u) >> 8));
    else if ((uint16_t)value == V34_BAUD_2800)
      obj->waacc_ = ((uint32_t)(int32_t)((int32_t)((uint32_t)x * 0xdbu) >> 8));
  }
}

/**
 * @brief End of phase 3 for the calling role: bring up the phase 3 transmitter and set the AGC gain estimate.
 * @param obj  V.34 module object
 * @return the receive flags (rxFlags_) after the update, for the caller's local copy
 * @note  Runs settxlevel(), V34SetupModulator() (PCM pre-emphasis selected when pcmMode_ or k56Mode_ is set), clears flag 0x800, moves the receive
 *        state to WAIT and the transmit state to SSEG, calls txinit(), sets rxGainEst_ = (rxGain_ * 0x6666 + 0x4000) >> 15 and sets flag 0x10.
 */
static unsigned phase3End(tagV34Object* obj)
{
  unsigned tmp, state, flags;
  int v90;

  settxlevel(obj, (const unsigned short*)&obj->info1c);
  v90 = (obj->pcmMode_ != 0 || obj->k56Mode_ != 0) ? 1 : 0;
  V34SetupModulator(&obj->mod, (int16_t)obj->rate.baud_, (int16_t)obj->rate.carrier_, (int16_t)obj->rate.preemp_, v90, 1);
  obj->rx.rxFlags_ = (uint16_t)(obj->rx.rxFlags_ & 0xfffff7ffu);

  /* receive state WAIT, transmit state SSEG */
  state = obj->rxState_;
  if ((uint16_t)state != V34HS_WAIT)
  {
    if (dsplibs_debug_level > 1u)
      rcPrRx(obj, state, V34HS_WAIT);
    obj->rxState_ = V34HS_WAIT;
  }
  state = obj->txState_;
  if ((uint16_t)state != V34HS_SSEG)
  {
    if (dsplibs_debug_level > 1u)
      v04PrTx(obj, state, V34HS_SSEG);
    obj->txState_ = V34HS_SSEG;
  }

  tmp = obj->tx.txFlags_;
  obj->tx.txCnt_ = 0;
  obj->tx.txFlags_ = (uint16_t)(tmp | 0x200u);
  txinit(obj);

  /* AGC gain estimate for the data phase */
  state = dsplibs_debug_level;
  tmp = (unsigned)((((int)(int16_t)obj->rx.rxGain_ * 0x6666) + 0x4000) >> 15);
  obj->rx.rxGainEst_ = (int16_t)tmp;
  if (state > 1u)
  {
    dsplibs_debug_printf("Agc gain estimate at the end of phase 3 is %d\n", (int)(int16_t)tmp);
    state = dsplibs_debug_level;
  }

  obj->rx.w0424_ = 0;
  flags = obj->rx.rxFlags_ | 0x10u;
  obj->rx.rxFlags_ = (uint16_t)flags;
  if (state > 1u)
  {
    uint32_t a = obj->rx.w49c_, b = (uint32_t)obj->rx.w04a0_;
    dsplibs_debug_printf("V34TIMING, detected  J, Phase =  %x,%x, clk =  %x,%x  \n", (unsigned)(b >> 16), (unsigned)(b & 0xffffu), (unsigned)(a >> 16),
                         (unsigned)(a & 0xffffu));
    flags = obj->rx.rxFlags_;
  }
  return flags;
}

/**
 * @brief Process one symbol interval of the RECEIVE state.
 * @param obj  V.34 module object
 * @note  See the file header for the sub-states.  Local `flags` is a copy of rxFlags_ that is refreshed from the object wherever the code has changed
 *        it; `v` is the bit-serial shift register.  The bit-queue word array q2 is indexed by widx without a range check (see storeWord).
 * @warning  q2 word index widx is not bounds-checked; it stays below 10 only because the MP sequence completes at 4 or 10 words.
 */
extern "C" void v04_rx_receive(tagV34Object* obj)
{
  tag_bitQ* const q2 = (tag_bitQ*)&obj->q2;
  unsigned flags, flagsB, tmp, symCnt, v = 0, flagsNow = 0, onesRun = 0;
  uint16_t toneHit;

  receiver(obj);
  flags = obj->rx.rxFlags_;
  symCnt = obj->rx.w388_;

  /* retrain: requested by the receiver, or too long without the veto flag */
  if (!(flags & 0x40u))
  {
    if ((int)(int16_t)symCnt <= 7 * (int)(int16_t)obj->rate.rxBaud_)
      goto afterRetrainCheck;
    if (flags & 0x80u)
      goto afterRetrainCheck;
  }
  v34handshakinit(obj, V34HSINIT_RETRAIN);
  if (dsplibs_debug_level > 1u)
    dsplibs_debug_printf("V34RETRAIN, going into retrain in Handshake\n");
  return;

afterRetrainCheck:
  /* MP progress: decide between the MP bit-serial receiver, the start-pattern search and the common tail */
  if ((flags & 0x98u) == 0x98u)
    goto commonTail;
  if (flags & 0x10u)
  {
    if (!(flags & 0x400u))
      goto commonTail;
    goto mpBitSerial;
  }
  if ((int16_t)symCnt <= 0xbb8)
    goto commonTail;

  /* start-pattern search on the shift register w0380_, two new bits per call */
  flagsB = obj->rx.rxFlags_;
  if (flagsB & 8u)
  {
    /* first pattern seen: wait for 0x899f, which starts the MP reception (flag 0x10) */
    unsigned word = ((unsigned)((int)(int16_t)obj->rx.w038a_ & 3) << 14) | ((unsigned)obj->rx.w0380_ >> 2);
    obj->rx.w0380_ = (uint16_t)word;
    if ((uint16_t)word != 0x899f)
      goto commonTail;
    tmp = flagsB | 0x10u;
    obj->rx.w0384_ = 0;
    obj->rx.rxFlags_ = (uint16_t)tmp;
    flags = tmp;
    goto commonTail;
  }

  /* no pattern yet: compare the shifted register with the reference word rxSig_ for 0x8990 / 0x89b0 (flag 0x08) */
  {
    unsigned prevWord = obj->rx.w0380_, sigWord = obj->rx.rxSig_;
    unsigned sigNew = ((prevWord & 3u) << 14) | (sigWord >> 2);
    unsigned word;
    obj->rx.rxSig_ = (uint16_t)sigNew;
    word = ((unsigned)((int)(int16_t)obj->rx.w038a_ & 3) << 14) | (prevWord >> 2);
    if ((int16_t)word == (int16_t)sigNew)
    {
      unsigned word2 = (uint16_t)word;
      if (word2 == 0x8990 || word2 == 0x89b0)
      {
        unsigned flagsE = flagsB | 8u;
        int offset;
        obj->rx.w0380_ = 0;
        obj->rx.w0384_ = 0;
        obj->rx.rxFlags_ = (uint16_t)flagsE;
        offset = (int16_t)obj->rx.resOffs_;
        VPcmV34LogTimingOffset(obj, (int16_t)(offset * 10));
        flags = obj->rx.rxFlags_;
        goto commonTail;
      }
    }
    obj->rx.w0380_ = (uint16_t)word;
  }
  goto commonTail;

mpBitSerial:
  /* append the two new bits at position nbits of the shift register acc (shift count taken modulo 32) */
  {
    unsigned cnt = q2->tail_.nbits;
    int sh = (int16_t)cnt;
    unsigned w = (unsigned)(int)(int16_t)obj->rx.w038a_;
    unsigned mask = ~(UINT_MAX << (sh & 31));
    unsigned newCnt;
    v = (q2->tail_.acc & mask) | (w << (sh & 31));
    newCnt = (cnt + 2u);
    q2->tail_.nbits = (uint16_t)newCnt;
    tmp = obj->mst_;

    /* microstate DET_SYNC: scan the bits one by one; otherwise collect 17-bit words */
    if ((uint16_t)tmp == V34HS_DET_SYNC)
    {
      if ((uint16_t)newCnt == 0)
        goto storeBitReg;
      goto bitLoop;
    }
    if ((int16_t)newCnt <= 0x10)
      goto storeBitReg;
    if (!(v & 0x10000u))
      goto storeWord;

    /* bit 16 set where a word boundary is expected: lost word sync, go back to DET_SYNC */
    q2->tail_.acc = v;
    if (dsplibs_debug_level > 1u)
      rcPrMst(obj, tmp, V34HS_DET_SYNC);
    obj->mst_ = V34HS_DET_SYNC;
    goto resetWord;
  }

bitLoop:
  /* bit-serial scan of the nbits pending bits, lowest first: count a run of ones in pos; a run longer than 0x13 is the E sequence, a zero ends a
   * run (a run longer than 0x10 ones ends the scan at zerosRunEnd) */
  {
    unsigned remaining = (uint16_t)q2->tail_.nbits;
    unsigned shifted;
    for (;;)
    {
      unsigned bit;
      remaining = remaining - 1u;
      q2->tail_.nbits = (uint16_t)remaining;
      bit = v & 1u;
      shifted = v >> 1;
      v = shifted;
      if (bit)
      {
        onesRun = (uint16_t)(q2->tail_.pos + 1u);
        q2->tail_.pos = (uint16_t)onesRun;
        flagsNow = obj->rx.rxFlags_;
        flagsB = flagsNow & 0x98u;
        if (flagsB == 0x90u || (((uint8_t*)&obj->mp)[0] & 1u))
        {
          if ((int16_t)onesRun > 0x13)
            goto eSequenceDetected;
        }
      }
      else
      {
        if ((int16_t)q2->tail_.pos > 0x10)
          goto zerosRunEnd;
        q2->tail_.pos = 0;
      }
      remaining = (uint16_t)q2->tail_.nbits;
      if (remaining == 0)
        break;
    }
    q2->tail_.acc = shifted;
    goto afterBitLoop;
  }

zerosRunEnd:
  /* a long run of ones ended by a zero: go to microstate DET_INFO and restart the word collection */
  {
    q2->tail_.acc = v;
    tmp = obj->mst_;
    if ((uint16_t)tmp != V34HS_DET_INFO)
    {
      if (dsplibs_debug_level > 1u)
        rcPrMst(obj, tmp, V34HS_DET_INFO);
      obj->mst_ = V34HS_DET_INFO;
    }
    obj->hsSymCnt_ = 0;
    q2->tail_.widx = 0;
    q2->tail_.pos = 0;
    goto afterBitLoop;
  }

eSequenceDetected:
  /* E sequence (a run of more than 0x13 ones): mark the MP reception complete (flags 0x98), log the timing offset, set up the data-phase receiver once */
  {
    unsigned newFlags;
    q2->tail_.acc = v;
    if (flagsB == 0x90u)
    {
      tmp = dsplibs_debug_level;
      newFlags = flagsNow;
    }
    else
    {
      tmp = dsplibs_debug_level;
      if (tmp > 1u)
      {
        dsplibs_debug_printf("V34MP- E received before MP', rxflgs = 0x%x\n", flagsNow);
        tmp = dsplibs_debug_level;
      }
      if (!(q2->unk00_[0] & 1u))
      {
        newFlags = obj->rx.rxFlags_;
      }
      else
      {
        copyCoefs(obj);
        newFlags = obj->rx.rxFlags_;
      }
    }
    newFlags |= 0x98u;
    obj->rx.rxFlags_ = (uint16_t)newFlags;
    obj->rx.w04ca_ = 0;
    obj->hsSymCnt_ = 0;
    if (tmp > 1u)
      dsplibs_debug_printf("V34MP - E sequence detected,rxflgs= 0x%x\n", (unsigned)obj->rx.rxFlags_);
    VPcmV34LogTimingOffset(obj, (int16_t)((int)(int16_t)obj->rx.resOffs_ * 10));
    newFlags = obj->rx.rxFlags_;
    if (newFlags & 0x4000u)
      obj->rx.rxFlags_ = (uint16_t)(newFlags | 0x2000u);
    if (obj->w3598_ == 0)
    {
      initdigital(obj);
      obj->w3598_ = 1;
    }
    obj->rx.w047c_ = 0x4000;
    goto afterESequence;
  }

storeWord:
  /* a complete 17-bit word: store its low 16 bits at index widx (counting a repeat of the previous word in hsSymCnt_), keep the remaining bits */
  {
    unsigned wordIdx = q2->tail_.widx;
    unsigned rawWord = (unsigned)(int)(int16_t)v;
    unsigned rawMasked = rawWord & 0x7fffu;
    unsigned stored = ld16(q2->unk00_, 2 * (unsigned)(int)(int16_t)wordIdx) & 0x7fffu;
    if (stored == rawMasked)
    {
      obj->hsSymCnt_ = (uint16_t)(obj->hsSymCnt_ + 1u);
      wordIdx = q2->tail_.widx;
    }
    st16(q2->unk00_, 2 * (unsigned)(int)(int16_t)wordIdx, rawWord);
    stored = (uint16_t)(wordIdx + 1u);
    q2->tail_.acc = v >> 17;
    q2->tail_.widx = (uint16_t)stored;
    q2->tail_.nbits = (uint16_t)(q2->tail_.nbits - 0x11u);
    if (q2->unk00_[0] & 1u)
    {
      if ((uint16_t)stored == 0xa)
        goto mpSequenceComplete;
      goto afterWordStored;
    }
    if ((uint16_t)stored == 4)
      goto mpSequenceComplete;
    flags = obj->rx.rxFlags_;
    goto commonTail;
  }

mpSequenceComplete:
  /* all words received: a non-negative first word is an MP sequence (flags 0x10), a negative one is MP1 (flags 0x90; extended MP also copies coefficients) */
  if ((int16_t)ld16(q2->unk00_, 0) >= 0)
  {
    unsigned mpFlags = obj->rx.rxFlags_ & 0xffffff67u;
    mpFlags |= 0x10u;
    obj->rx.rxFlags_ = (uint16_t)mpFlags;
    if (dsplibs_debug_level > 1u)
      printTen("V34MP -MP sequence,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x\n", q2->unk00_);
  }
  else
  {
    unsigned mpFlags = obj->rx.rxFlags_ & 0xffffff67u;
    mpFlags |= 0x90u;
    obj->rx.rxFlags_ = (uint16_t)mpFlags;
    if (q2->unk00_[0] & 1u)
      copyCoefs(obj);
    if (dsplibs_debug_level > 1u)
      printTen("V34MP -MP1 sequence,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x\n", q2->unk00_);
  }

  tmp = obj->mst_;
  if ((uint16_t)tmp != V34HS_DET_SYNC)
  {
    if (dsplibs_debug_level > 1u)
      rcPrMst(obj, tmp, V34HS_DET_SYNC);
    obj->mst_ = V34HS_DET_SYNC;
  }

resetWord:
  q2->tail_.widx = 0;
  q2->tail_.pos = 0;
afterESequence:
afterWordStored:
afterBitLoop:
  flags = obj->rx.rxFlags_;
  goto commonTail;

storeBitReg:
  /* not enough bits yet: just keep the shift register */
  q2->tail_.acc = v;
  flags = obj->rx.rxFlags_;

commonTail:
  /* end of phase 3 for the calling role, then the saturating symbol counter */
  if ((flags & 0x98u) == 8u)
  {
    if (obj->modeChar_ == V34ROLE_CALL)
      flags = phase3End(obj);
  }
  {
    unsigned symCntNow = obj->rx.w388_;
    if ((int16_t)symCntNow <= 0x61a7)
      obj->rx.w388_ = (uint16_t)(symCntNow + 1u);
  }

  /* INFO already sent: nothing more to do */
  if (flags & 0x400u)
    return;
  if (obj->det.toneState_ == 3)
    goto mdLengthWait;

  /* arm the tone detector and look for the 'S' tone */
  {
    unsigned flagsWork = flags | 0x100u;
    obj->rx.rxFlags_ = (uint16_t)flagsWork;
    if (obj->waa80_ != 0)
    {
      flagsWork |= 0x200u;
      obj->rx.rxFlags_ = (uint16_t)flagsWork;
    }
  }

  toneHit = (uint16_t)tone_detect(&obj->rx, &obj->det, (const short*)obj->rx.rxFskBuf_, obj->rx.rxEnd_);
  if (toneHit == 0)
    return;
  if (dsplibs_debug_level > 1u)
    dsplibs_debug_printf("S detected,rxflgs= 0x%x,rxgain=0x%x\n", (unsigned)obj->rx.rxFlags_, (int)(int16_t)obj->rx.rxGain_);
  sDetected(obj);
  return;

mdLengthWait:
  /* waiting for the MD sequence: once the counter passes rxMdLen_ restart the receiver */
  {
    unsigned mdLen = obj->rxMdLen_;
    unsigned symCntNow;
    if ((uint16_t)mdLen == 0)
      return;
    symCntNow = obj->rx.w388_;
    if ((int16_t)symCntNow <= (int16_t)mdLen)
      return;
    if (dsplibs_debug_level > 1u)
      dsplibs_debug_printf("RX MDLENGTH over, it was %d bauds,rxflgs = 0x%x,pllcnt = %d,rxgain=0x%x\n", (int)(int16_t)symCntNow, flags & 0xffffu,
                           (int)obj->rx.w0424_, (int)obj->rx.rxGain_);
    restartReceiver(obj);
    return;
  }
}
