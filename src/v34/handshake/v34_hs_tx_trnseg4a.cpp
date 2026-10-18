/**
 * @file v34_hs_tx_trnseg4a.cpp
 * @brief V.34 handshake transmit state TRNSEG4A: send the scrambled training point, then choose the data rate and build the MP bits.
 *
 * One entry point, v04_tx_trnseg4a(), the case for transmit state 0x42 of the per-sample loop of v34handshak() (v34_hs_dispatch.cpp), which
 * re-evaluates its loop condition after the case.  Callees: txmit(), bitreverse(), sysdep_memset(), VPcmV34GetMaxUpstreamRateIndex().
 *
 * Every call sends one scrambled point (two-step scrambler with the constant input 3; one pass and vect4[top 2 bits] for the 4-point constellation
 * (rxSig_ != 0x89b0), two passes and vect16 for the 16-point one) and counts txCnt_.  When txCnt_ reaches 1.5 * baud_ + period_ (or the earlier
 * conditions below) the receiver's results are turned into the MP bits:
 * - Precoder coefficients: with rxFlags_ bit 5 clear, the six coefficient words of the receiver are used (and copied to the precoder filter
 *   precoef_) when the sum of |c >> 2| is at most 0x3fff and the equaliser error test passes, otherwise zeros are sent.
 * - Data-rate search: a rate index r starts at 9 / 0xb / 0xc / 0xd / 0xe by the receive baud rate and is lowered while the SNR estimate (`w04b4_`)
 *   is below the per-rate threshold read from the table scaleTab_ (14 words per carrier row n = waaa6_); the retrain, renegotiate-down and
 *   renegotiate-up thresholds are derived from it.
 * - The result is clamped to minRateIdx_ / maxRateIdx_ and the rate fixed for DATAXMIT (w22c_), bit-reversed into the MP word (the field at bit
 *   10 for the answering role, bit 6 for the calling role), limited by VPcmV34GetMaxUpstreamRateIndex(), and the MP bit-queue record mp is armed.
 * - The transmit state becomes XMITMP (0x43).
 *
 * @note `lo` (lowest rate index) is also the precoder error value until the baud ladder overwrites it.  For a baud rate outside the five of the
 *       ladder, `lo` and `r` keep their initial values (0 and the precoder value); `thr` stays 0 if the search loop is not entered.  Debug level
 *       `lv` is a local copy that is reloaded after every print.  The bit-queue unk00_ words and the six receiver taps are not typed as such and
 *       are read and written through byte offsets with ld16() / st16().
 */

#include <limits.h>
#include "v34_handshake.h"
#include "core/dsp_access.h"
#include "core/dsp_host.h"
#include "v34/v34_internal.h"
#include "vpcm/vpcm_v34_api.h"

extern "C"
{
}

#include "v34_hs_bytes.h"

/**
 * @brief SNR threshold of rate index `r`.
 * @param obj   handshake object (scaleTab_)
 * @param r     rate index
 * @param n     carrier row of scaleTab_ (14 words per row)
 * @param back  entries to step back from the index r + 14 * n
 * @return ((tab[r + 14n - back] >> 5)^2 as int16) * (r >= 13 ? 0x4268 : 0x3a98) >> 14, as int16
 */
static int rateThresh(tagV34Object* obj, int r, int n, int back)
{
  int v = (int)(int16_t)obj->rate.scaleTab_[r + 14 * n - back] >> 5;
  int sq = (int16_t)(v * v);
  int mul = ((int16_t)r >= 0xd) ? 0x4268 : 0x3a98;
  return (int16_t)((sq * mul) >> 14);
}

/**
 * @brief Renegotiation threshold derived from scaleTab_.
 * @param obj   handshake object (scaleTab_)
 * @param r     rate index
 * @param n     carrier row of scaleTab_ (14 words per row)
 * @param back  entries to step back from the index r + 14 * n
 * @return ((tab[r + 14n - back] >> 5)^2 as int16) scaled by 0x3a98 for r <= 12 else 0x4268, >> 14 (not narrowed to int16)
 */
static int rateScale(tagV34Object* obj, int r, int n, int back)
{
  int v = (int)(int16_t)obj->rate.scaleTab_[r + 14 * n - back] >> 5;
  int sq = (int16_t)(v * v);
  int mul = ((int16_t)r <= 0xc) ? 0x3a98 : 0x4268;
  return (sq * mul) >> 14;
}

/**
 * @brief Transmit state TRNSEG4A (0x42): send one scrambled point and, when the receiver has settled, select the data rate and arm the MP bit queue.
 * @param obj  handshake object; reads the receiver results (equErr_, rxBaud_, rxFlags_, scaleTab_, ...), writes txState_, txRateIdx_, rxRateIdx_,
 *             mp, precoef_ and the retrain thresholds
 * @note  Runs once per call until the symbol count test passes; the rate-selection part then runs once and moves the state to XMITMP.
 * @bug   The 4-point point is vect4[scrambler >> 29] and the 16-point one vect16[(scrambler >> 29) + 4 * txBits_] with txBits_ also a 3-bit value;
 *        vect4 has 4 entries and vect16 16, so indices above that read past the tables (candidate defect, read from the code; not run).
 */
extern "C" void v04_tx_trnseg4a(tagV34Object* obj)
{
  tag_bitQ* const q = (tag_bitQ*)&obj->mp;
  unsigned lv;
  int lo = 0, r = 0, thr = 0;

  /* the scrambled point: 4-point (one pass) or 16-point (two passes) constellation */
  {
    const bool tap13 = obj->modeChar_ == V34ROLE_CALL;
    uint32_t e = v04Scramble(obj->tx.scrState_, tap13, 3);
    if (obj->rx.rxSig_ == 0x89b0)
    {
      unsigned bits;
      obj->tx.scrState_ = e;
      obj->tx.txBits_ = (uint16_t)(e >> 29);
      e = v04Scramble(e, tap13, 3);
      obj->tx.scrState_ = e;
      bits = (unsigned)(int)(int16_t)obj->tx.txBits_;
      obj->tx.txPoint_ = (uint32_t)vect16[(e >> 29) + bits * 4u];
    }
    else
    {
      obj->tx.scrState_ = e;
      obj->tx.txBits_ = (uint16_t)(e >> 29);
      obj->tx.txPoint_ = (uint32_t)vect4[e >> 29];
    }
  }
  txmit(obj);

  /* wait until enough symbols have been sent (or the receiver is already settled) */
  {
    unsigned c = (uint16_t)(obj->tx.txCnt_ + 1u);
    int symCnt, baud, per, limit;
    obj->tx.txCnt_ = (uint16_t)c;
    symCnt = (int16_t)c;
    baud = (int16_t)obj->rate.baud_;
    per = (int16_t)obj->rate.period_;
    limit = baud + (baud >> 1) + per;
    if (symCnt != limit)
    {
      if ((uint32_t)obj->mod.runMode_ <= 3u)
        return;
      if (symCnt < limit)
      {
        if (symCnt < baud + per)
          return;
        if ((int)(int16_t)obj->rx.equErr_ > (int)(int16_t)obj->rx.w04b4_ + 10)
          return;
      }
    }
  }

  /* the MP record and the precoder coefficients */
  obj->tx.txPrev_ = obj->tx.txBits_;
  obj->bitQ_ = q;
  if (obj->rx.rxFlags_ & 0x20u)
  {
    st16(q->unk00_, 4, 0);
    st16(q->unk00_, 0, 0);
    obj->rx.w04b4_ = (uint16_t)obj->rx.equErr_;
    lv = dsplibs_debug_level;
  }
  else
  {
    int sum = 0, k, e = 0;
    unsigned cxv, div;
    int d;
    int ok = 0;
    st16(q->unk00_, 0, 0x8000);
    for (k = 0; k < 6; k++)
    {
      int v = (int)(int16_t)ld16((const uint8_t*)obj->rx.tapX2_, 2 * k) >> 2; /* the six taps tapX2_[3] tapX1_[3] */
      int s = v >> 31;
      sum += (v ^ s) - s;
    }
    cxv = (uint16_t)obj->rx.equErr_;
    div = (uint16_t)obj->rx.w0488_;
    d = (int16_t)(cxv - div);
    e = d <= 0 ? (int16_t)div : d;
    e = (int16_t)(e - ((int)(int16_t)cxv >> 2));
    lo = e;
    if (sum > 0x1f40)
    {
      if ((int16_t)cxv > 0xc8 && e > 0)
        sum = 0x5000;
    }
    if (obj->w359a_ == 0 && !((int16_t)cxv > 0x1ff))
    {
      if ((int16_t)div < (int16_t)cxv && sum <= 0x3fff)
        ok = 1;
    }
    if (ok)
    {
      const uint8_t* src = (const uint8_t*)obj->rx.tapX2_; /* tapX2_[3] followed by tapX1_[3] */
      unsigned c4, ca, c2, c8, c0, c6;
      uint16_t* f = obj->rx.precoef_; /* the precoder filter */
      int i;
      if (dsplibs_debug_level > 1u)
        dsplibs_debug_printf("V34DATARATE, precoefs [%d,%d,%d][%d,%d,%d]\n", (int)(int16_t)ld16(src, 0), (int)(int16_t)ld16(src, 2), (int)(int16_t)ld16(src, 4),
                             (int)(int16_t)ld16(src, 6), (int)(int16_t)ld16(src, 8), (int)(int16_t)ld16(src, 0xa));
      c4 = ld16(src, 4);
      ca = ld16(src, 0xa);
      c2 = ld16(src, 2);
      c8 = ld16(src, 8);
      c0 = ld16(src, 0);
      c6 = ld16(src, 6);
      st16(q->unk00_, 4, c4);
      st16(q->unk00_, 6, ca);
      st16(q->unk00_, 8, c2);
      st16(q->unk00_, 0xa, c8);
      st16(q->unk00_, 0xc, c0);
      st16(q->unk00_, 0xe, c6);
      f[7] = (uint16_t)c4;
      f[0] = (uint16_t)c4;
      f[1] = (uint16_t)(0u - ld16(q->unk00_, 6));
      f[6] = (uint16_t)ld16(q->unk00_, 6);
      f[9] = (uint16_t)ld16(q->unk00_, 8);
      f[2] = (uint16_t)ld16(q->unk00_, 8);
      f[3] = (uint16_t)(0u - ld16(q->unk00_, 0xa));
      f[8] = (uint16_t)ld16(q->unk00_, 0xa);
      f[11] = (uint16_t)ld16(q->unk00_, 0xc);
      f[4] = (uint16_t)ld16(q->unk00_, 0xc);
      f[5] = (uint16_t)(0u - ld16(q->unk00_, 0xe));
      f[10] = (uint16_t)ld16(q->unk00_, 0xe);
      for (i = 0; i <= 5; i++)
      {
        uint16_t w = ld16(q->unk00_, 4 + 2 * i);
        st16(q->unk00_, 4 + 2 * i, bitreverse(w, 0x10));
      }
      obj->rx.rxFlags_ = (uint16_t)(obj->rx.rxFlags_ | 0x4000u);
      sysdep_memset(obj->rx.tapY2_, 0, sizeof(obj->rx.tapY2_) + sizeof(obj->rx.tapY1_)); /* tapY1_ follows tapY2_ */
      obj->rx.w04b4_ = (uint16_t)obj->rx.w0488_;
      lv = dsplibs_debug_level;
    }
    else
    {
      lv = dsplibs_debug_level;
      if (lv > 1u)
      {
        dsplibs_debug_printf("V34DATARATE, precoefs=0, 0, 0, 0, 0, 0\n");
        lv = dsplibs_debug_level;
      }
      st16(q->unk00_, 4, 0);
      st16(q->unk00_, 6, 0);
      st16(q->unk00_, 8, 0);
      st16(q->unk00_, 0xa, 0);
      st16(q->unk00_, 0xc, 0);
      st16(q->unk00_, 0xe, 0);
      obj->rx.w04b4_ = (uint16_t)obj->rx.equErr_;
    }
    obj->rx.rxFlags_ = (uint16_t)(obj->rx.rxFlags_ & ~0x1000u);
  }

  /* the rate-index search: start from the per-baud-rate ladder value, then step down while the SNR estimate is below the threshold */
  obj->rate.waaa6_ = 1;
  switch ((uint16_t)obj->rate.rxBaud_)
  {
  case V34_BAUD_2400:
    lo = 1;
    r = 9;
    break;
  case V34_BAUD_2800:
    lo = 2;
    r = 0xb;
    break;
  case V34_BAUD_3000:
    lo = 2;
    r = 0xc;
    break;
  case V34_BAUD_3200:
    lo = 2;
    r = 0xd;
    break;
  case V34_BAUD_3429:
    lo = 2;
    r = 0xe;
    break;
  default:
    break;
  }

  if (obj->rate.waaa6_ != 0)
    st16(q->unk00_, 0, ld16(q->unk00_, 0) | 2u);

  /* the far end does not offer V.34bis: cap the start rate */
  if (!(((uint8_t*)&obj->info0d)[2] & 4u))
  {
    int baudNow = (int16_t)obj->rate.rxBaud_;
    if (baudNow == V34_BAUD_3200 || baudNow == V34_BAUD_3429)
      r = 0xc;
    else if (baudNow == V34_BAUD_3000 || baudNow == V34_BAUD_2800)
      r = (int16_t)(r - 1);
    if (lv > 1u)
    {
      dsplibs_debug_printf("V34INFO, V.34bis is not possible \n");
      lv = dsplibs_debug_level;
    }
  }
  obj->rate.txRateIdx_ = (int16_t)r;

  if ((int16_t)r > (int16_t)lo)
  {
    do
    {
      thr = rateThresh(obj, r, obj->rate.waaa6_, 1);
      if (lv > 1u)
      {
        dsplibs_debug_printf("V34DATARATE,threshold for data rate %d = %d\n", r, thr);
        lv = dsplibs_debug_level;
      }
      if ((int16_t)obj->rx.w04b4_ < (int16_t)thr)
        break;
      r = (int16_t)(r - 1);
    } while ((int16_t)r > (int16_t)lo);
  }

  /* returning from a local rate renegotiation: force the rate below / above the previous one */
  {
    unsigned rrnReason = obj->rx.rnegReason_;
    if ((int16_t)rrnReason > 1)
    {
      unsigned prevRate = obj->rx.rnegRateIdx_;
      bool forcedDown = false;
      if ((uint16_t)rrnReason == 2)
      {
        if (r > (int)(int16_t)prevRate - 1)
        {
          r = (int16_t)(prevRate - 1u);
          if (lv > 1u)
          {
            dsplibs_debug_printf(" TRNSEG4A : returning from local rrn down => forcing rate down\n");
            lv = dsplibs_debug_level;
          }
          forcedDown = true;
        }
      }
      if (!forcedDown)
      {
        if (!(r >= (int)(int16_t)prevRate + 1))
        {
          r = (int16_t)(prevRate + 1u);
          if (lv > 1u)
          {
            dsplibs_debug_printf(" TRNSEG4A : returning from local rrn up => forcing rate up\n");
            lv = dsplibs_debug_level;
          }
        }
      }
      thr = rateThresh(obj, r, obj->rate.waaa6_, 1);
    }
  }

  if (lv > 1u)
  {
    dsplibs_debug_printf("V34DATARATE, ethresh data rate = %d,ethreh=%d,rate2 = 0x%x,data=%d\n", r, (int)(int16_t)obj->rx.w04b4_, (unsigned)obj->rx.rxSig_,
                         thr);
    lv = dsplibs_debug_level;
    if (lv > 1u)
    {
      dsplibs_debug_printf("V34DATARATE, equerr = %d,preerr=%d\n", (int)(int16_t)obj->rx.equErr_, (int)obj->rx.w0488_);
      lv = dsplibs_debug_level;
    }
  }

  /* retrain, renegotiate-down and renegotiate-up thresholds */
  obj->rx.retrainErrThr_ = (uint16_t)(thr + thr);
  if ((int16_t)r <= (int16_t)lo)
  {
    obj->rx.rrnDownErrThr_ = (uint16_t)((int)(int16_t)obj->rx.w04b4_ << 3);
  }
  else
  {
    int n = obj->rate.waaa6_;
    obj->rx.rrnDownErrThr_ = (uint16_t)rateScale(obj, r, n, 2);
    obj->rx.rrnDownErrThr_ = (uint16_t)(((int)(int16_t)obj->rx.rrnDownErrThr_ + thr) >> 1);
  }
  if ((int16_t)obj->rate.txRateIdx_ <= (int16_t)r)
  {
    obj->rx.rrnUpErrThr_ = 0;
  }
  else
  {
    int n = obj->rate.waaa6_;
    obj->rx.rrnUpErrThr_ = (uint16_t)rateScale(obj, r, n, 0);
  }

  obj->rx.rrnDownCnt_ = 0;
  obj->rx.rrnUpCnt_ = 0;
  obj->rx.retrainCnt_ = 0;
  st16(q->unk00_, 2, 0xfffd);
  if ((uint32_t)obj->sampleTime_ < (uint32_t)obj->dataStartTime_ + 0x17700u && (uint32_t)obj->mod.runMode_ <= 3u)
  {
    r = (int16_t)(r - 2);
    if ((int16_t)r < (int16_t)lo)
      r = lo;
    obj->rate.txRateIdx_ = (int16_t)r;
    st16(q->unk00_, 2, ld16(q->unk00_, 2) & 0xfffffffeu);
  }

  obj->rate.rxRateIdx_ = (int16_t)r;
  if (lv > 1u)
    dsplibs_debug_printf("V34DATARATE, automatic: %d, min %d, max %d\n", (int)((unsigned)r * 0x960u), (int)((unsigned)obj->minRateIdx_ * 0x960u),
                         (int)((unsigned)obj->maxRateIdx_ * 0x960u));

  /* clamp to the configured minimum / maximum rate and to the rate fixed for data mode */
  {
    int rateIdx = (uint16_t)obj->rate.rxRateIdx_;
    int32_t bound = obj->minRateIdx_;
    int32_t w22c;
    if ((int)(int16_t)rateIdx < bound)
    {
      obj->rate.rxRateIdx_ = (int16_t)bound;
      rateIdx = bound;
    }
    bound = obj->maxRateIdx_;
    if ((int)(int16_t)rateIdx > bound)
      obj->rate.rxRateIdx_ = (int16_t)bound;
    w22c = obj->w22c_;
    if (w22c >= 0 && w22c != obj->w228_)
      obj->rate.rxRateIdx_ = (int16_t)w22c;
  }

  /* encode the rates into the MP word, limited by the upstream maximum */
  {
    int r4 = (int16_t)bitreverse((uint16_t)obj->rate.rxRateIdx_, 4);
    unsigned w;
    obj->toneSel_ = bitreverse((uint16_t)obj->rate.txRateIdx_, 4);
    if (dsplibs_debug_level > 1u)
      dsplibs_debug_printf("V34DATARATE, Final choice data rate = %d, retrainThresh = %d, renegDownthresh = %d, renegUpthresh = %d\n",
                           (int)obj->rate.rxRateIdx_, (int)(int16_t)obj->rx.retrainErrThr_, (int)(int16_t)obj->rx.rrnDownErrThr_,
                           (int)(int16_t)obj->rx.rrnUpErrThr_);
    w = ld16(q->unk00_, 0);
    if (obj->modeChar_ == V34ROLE_CALL)
    {
      w = ((unsigned)(int)(int16_t)obj->toneSel_ << 10) | w;
      w = w | ((unsigned)r4 << 6);
    }
    else
    {
      w = ((unsigned)r4 << 10) | w;
      w = w | ((unsigned)(int)(int16_t)obj->toneSel_ << 6);
    }
    st16(q->unk00_, 0, w);
    {
      int mx = VPcmV34GetMaxUpstreamRateIndex(obj);
      unsigned wd = ld16(q->unk00_, 0);
      int val, mask;
      if (obj->modeChar_ == V34ROLE_CALL)
      {
        val = (int)((wd >> 13) & 1u) + 2 * (int)((wd >> 12) & 1u) + 4 * (int)((((wd >> 9) & 2u)) + ((wd >> 11) & 1u));
        mask = (int)0xfffffbff;
      }
      else
      {
        val = (int)((wd >> 9) & 1u) + 2 * (int)((wd >> 8) & 1u) + 4 * (int)(((wd >> 5) & 2u) + ((wd >> 7) & 1u));
        mask = (int)0xffffffbf;
      }
      if (val * 0x960 > mx)
      {
        unsigned arg = (unsigned)((int)((unsigned)mx * 7u) >> 14);
        int rateBits = (int16_t)bitreverse((uint16_t)arg, 4);
        int i;
        int maskBits = mask;
        for (i = 0; i <= 3; i++)
        {
          unsigned wv = ld16(q->unk00_, 0);
          unsigned nw;
          if ((rateBits >> i) & 1)
            nw = (unsigned)(~maskBits) | wv;
          else
            nw = wv & (unsigned)maskBits;
          st16(q->unk00_, 0, nw);
          maskBits = (int16_t)((maskBits * 2) | 1);
        }
      }
    }
  }

  /* move to XMITMP and arm the MP bit queue */
  st16(q->unk00_, 0x10, 0);
  {
    unsigned txCur = obj->txState_;
    lv = dsplibs_debug_level;
    if ((uint16_t)txCur != V34HS_XMITMP)
    {
      if (lv > 1u)
      {
        v04PrTx(obj, txCur, V34HS_XMITMP);
        lv = dsplibs_debug_level;
      }
      obj->txState_ = V34HS_XMITMP;
    }
  }
  obj->tx.k56Idx_ = 0;
  obj->w3598_ = 0;
  obj->w359e_ = 0;
  if (lv > 1u)
    dsplibs_debug_printf("V34DATARATE, txmp bits 0x%x,0x%x,0x%x,0x%x,0x%x\n", (unsigned)ld16(q->unk00_, 0), (unsigned)ld16(q->unk00_, 2),
                         (unsigned)ld16(q->unk00_, 4), (unsigned)ld16(q->unk00_, 6), (unsigned)ld16(q->unk00_, 8));

  q->tail_.acc = 0x3fffeu;
  q->tail_.restartAcc = 0x3fffeu;
  q->tail_.crc = USHRT_MAX;
  q->tail_.pos = 0;
  q->tail_.widx = 0;
  q->tail_.pass = 0;
  q->tail_.chunk = 0x10;
  q->tail_.total = (uint16_t)(((obj->rx.rxFlags_ >> 5) & 1u) == 0 ? 0x90 : 0x30);
  q->tail_.crcOn = 1;
  q->tail_.nbits = 0x12;
  q->tail_.restartBits = 0x12;
  q->tail_.loop = 0;
  obj->w3590_ = 0x22;
}
