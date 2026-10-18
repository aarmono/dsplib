/**
 * @file vpcm_progress.cpp
 * @brief VPcmV34Progress(): the per-block driver of a whole V.34 / V.90 / V.92 / K56Flex session.
 *
 * Owns one entry point, @c VPcmV34Progress(), the plain C symbol the session loop (vpcm_run) calls once per block of samples.  It is written as
 * extern "C" C++ because it calls members of VPcmFloModem, K56FlexFloModem, V90Demodulator, V90Parameters, GenericIIR and GenericToneDetector.  The
 * session object is the tagV34Object (v34/v34_object.h); its monitor ring lives inside the constPts_ region and is reached through ringp().
 *
 * Signature: int VPcmV34Progress(tagV34Object *obj, float *in, float *out, unsigned nIn, int *bitsOut, int *countOut, int *bitsIn, int *countIn).
 *  - in / out: n = nIn & ~3 float samples; both pointers advance (or are rewound) over the block,
 *  - bitsOut / countOut: array and count of bits received in this block; bitsIn / countIn: array and count of bits to transmit (on entry the
 *    number provided, on return the number still wanted),
 *  - the return value is obj->state_ (the session state), or 1 / 0 for the two "P2 COMPLETE" waits.
 *
 * Structure.  The function is one large goto-shaped state machine; the labels name the blocks:
 *  - entry gate: w262_ == 0 returns state_; n = nIn & ~3; sampleTime_ = (sampleTime_ + n) & 0x7fffffff;
 *  - txBitPack: the transmit bit packer (V.34, V.90 answerer, K56Flex caller): bitsIn bits -> txWords_ (16 bits per word, bit 0 first);
 *  - entranceFilter: the optional input filter (ctx->entranceFilter_), GenericIIR::process() in place; mode is re-read afterwards;
 *  - the switch on obj->mode_ (0..10): 0 V.34 data pump (dataPump), 1 V.90 phase 3 / 4 (v90Mode, v90RunDemodulator), 2 V.92 / PCM (pcmModemMode,
 *    runPcmModem), 3 K56Flex (k56FlexMode, k56FlexRunDemodulator), 4 QC line verification, 5 wait for the user response, 6 / 7 silence while
 *    the ANSam detector runs (outgoing call / on hold), 8 / 9 / 10 reconnect delay, reconnect request and end of session;
 *  - retrainDetector: for states 3..6 a 3-tap filter over the ring samples, 64-sample energy blocks, and the "Retrain Detected by Tone detector" path.
 *
 * Details worth knowing before changing anything:
 *  - every debug print is guarded by the unsigned test dsplibs_debug_level > 1 read at the point of the print (LEVEL_ON()); the QC-verification block
 *    reads the level once and re-reads it after the first print;
 *  - ring stores truncate a float to 16 bits (NaN or out of range gives 0x8000); the ring index w2aa6_ is a signed 16-bit value, the next index is
 *    (uint16)(idx + 1) > 0x257 ? 0 : idx + 1, stored after the sample (ring of 600 samples);
 *  - `st` caches obj->state_ across the function and is reloaded only after some calls and stores; `st0` is the state at the start of mode 0 and is
 *    tested by the post-processing of states 1 (phase 2 complete) and 2;
 *  - the three echo-canceller loops of modes 1 and 3 are one routine, echoLoop(); the demodulator result switches of modes 1, 2 and 3 map the
 *    demodulator's return code to a session state or a retrain request;
 *  - `mode` is the value read at entry; it is re-read after the entrance filter and after the "ANSam not detected on out-going" print of mode 6.
 *
 * @warning The bit arrays are not range-checked: bitsIn must hold 16 * (*countIn >> 4) entries, bitsOut 16 * rxWordCnt_ entries, and txWords_ must have room
 *          for the words appended at txTail_.
 */

#include "v34/v34_object.h"
#include "pcm_modulators.h"     /* V92Modulator */
#include "pcm_bits_to_symbol.h" /* V92BitsToSymbol / V92Transmitter */
#include "core/dsp_asm.h"

#include "core/dsp_debug_level.h" /* dsplibs_debug_level, LEVEL_ON() */
#include "core/host_interface.h"  /* DP_V34 / DP_K56 / DP_V90 / DP_V92 */
#include "core/dsp_host.h"
#include "v34/v34_api.h"
#include "vpcm_v34_api.h"

extern "C"
{
}

static inline int s16(uint16_t v) { return (int)(int16_t)v; }
#include "core/dsp_floatbits.h"

/**
 * @brief Convert a float to 16 bits by truncation, as the x87 `fistp WORD` does.
 * @param v  value to convert
 * @return the truncated value; 0x8000 for NaN or a value outside (-32769, 32768)
 */
static inline int16_t trunc16(long double v)
{
  if (!(v > -32769.0L && v < 32768.0L))
    return (int16_t)0x8000;
  return (int16_t)v;
}

/**
 * @brief Address of a sample of the monitor ring (600 shorts, stored 1200 bytes behind the start of constPts_).
 * @param obj  session object
 * @param idx  sample index
 * @return pointer to the sample
 * @warning idx is a signed 16-bit ring index and is not range-checked here; a value outside 0..599 addresses memory outside the ring.
 */
static inline int16_t* ringp(tagV34Object* obj, int idx)
{
  char* p = (char*)obj->tx.constPts_ + 0x4b0;
  OPTIMIZER_HIDE_VAR(p);
  return (int16_t*)(p + 2 * idx);
}

/** @brief Store one sample at the ring index w2aa6_ and advance the index, wrapping to 0 after 599. */
static inline void ringPush(tagV34Object* obj, int16_t s)
{
  int idx = (int16_t)obj->tx.w2aa6_;
  unsigned nx;
  *ringp(obj, idx) = s;
  nx = (uint16_t)(idx + 1);
  obj->tx.w2aa6_ = (uint16_t)(nx > 0x257u ? 0u : nx);
}

/** @brief Push the truncated samples src[0 .. n) into the monitor ring (same index rule as ringPush()). */
static inline void ringFill(tagV34Object* obj, const float* src, unsigned n)
{
  int i;
  for (i = 0; i < (int)n; i++)
  {
    int idx = (int16_t)obj->tx.w2aa6_;
    unsigned nx;
    *ringp(obj, idx) = trunc16(src[i]);
    nx = (uint16_t)(idx + 1);
    obj->tx.w2aa6_ = (uint16_t)(nx > 0x257u ? 0u : nx);
  }
}

/**
 * @brief Echo-canceller loop of modes 1 and 3.
 *
 * Per sample the truncated input goes through adaptecho(); a slow DC estimate (updated every 128 samples) is subtracted; the corrected sample
 * replaces the input sample and is pushed into the ring; the echo-cancelled value w25e_ becomes the output sample.
 *
 * @param obj   session object
 * @param inp   in/out: input pointer; advanced by n, the corrected samples are written back
 * @param outp  in/out: output pointer; advanced by n
 * @param n     number of samples
 */
static inline void echoLoop(tagV34Object* obj, float** inp, float** outp, unsigned n)
{
  unsigned left = n;
  for (; left != 0; left--)
  {
    unsigned cnt, sampleIn;
    int16_t s;
    obj->w260_ = (uint16_t)trunc16(**inp);
    adaptecho(obj);
    cnt = obj->w254_;
    sampleIn = obj->w260_;
    if ((uint16_t)cnt != 0)
    {
      uint32_t acc = obj->w258_;
      cnt = (uint16_t)(cnt - 1u);
      obj->w254_ = (uint16_t)cnt;
      acc += (uint32_t)(int32_t)(int16_t)sampleIn;
      if ((cnt & 0x7fu) != 0)
      {
        obj->w258_ = acc;
        sampleIn = obj->w260_;
      }
      else
      {
        int dcEstimate = s16(obj->w256_) >> 1;
        dcEstimate += (int32_t)acc >> 8;
        obj->w256_ = (uint16_t)dcEstimate;
        if (LEVEL_ON())
        {
          obj->w258_ = acc;
          dsplibs_debug_printf("Estimated DC = %d  (acc = %d)\n", (int)(int16_t)dcEstimate, (int)acc);
        }
        obj->w258_ = 0;
        sampleIn = obj->w260_;
      }
    }
    s = (int16_t)((int)sampleIn - s16(obj->w256_));
    ringPush(obj, s);
    **inp = (float)(int)s;
    (*inp)++;
    **outp = (float)s16(obj->w25e_);
    (*outp)++;
  }
}

/** @brief Move the unsent transmit words txWords_[txHead_ .. txTail_) to the front of the queue (shared by mode 0 and the demodulator modes). */
static inline void compactQueue(tagV34Object* obj)
{
  unsigned head = obj->txHead_, tail = obj->txTail_, k = 0;
  while (tail > head)
  {
    obj->txWords_[k] = obj->txWords_[head];
    head++;
    k++;
  }
  obj->txHead_ = 0;
  obj->txTail_ = k;
}

/**
 * @brief Number of transmit bits still wanted, the value stored in *countIn.
 * @return (w10_ - txTail_) * 16 when w10_ is non-zero and larger than txTail_, else 0
 */
static inline int wanted(tagV34Object* obj)
{
  int bits = (int)obj->w10_;
  if (bits != 0)
  {
    bits -= (int)obj->txTail_;
    if (bits <= 0)
      bits = 0;
    else
      bits <<= 4;
  }
  return bits;
}

/** @brief Silence the output block: store n 32-bit zero words. */
static inline void zeroOut(float* outp, unsigned n)
{
  unsigned i;
  for (i = 0; i < n; i++)
    ((int32_t*)outp)[i] = 0;
}

/**
 * @brief Process one block of a V.34 / V.90 / V.92 / K56Flex session (see the file comment for the overall structure).
 * @param obj       session object; a zero w262_ makes the call a no-op returning state_
 * @param inp       input samples (float); the entrance filter and the echo canceller modify them in place
 * @param outp      output samples (float), n = nIn & ~3 of them
 * @param nIn       samples in the block (rounded down to a multiple of 4)
 * @param bitsOut   array receiving the bits demodulated in this block
 * @param countOut  receives the number of bits stored in bitsOut
 * @param bitsIn    transmit bits, one per entry (bit 0 of the entry); packed into 16-bit words
 * @param countIn   in: number of bits provided; out: number of bits wanted for the next block
 * @return the session state (obj->state_); 1 or 0 for the two phase-2-complete waits
 * @warning The bit arrays and txWords_ are not range-checked against the counts.
 */
extern "C" int VPcmV34Progress(tagV34Object* obj, float* inp, float* outp, unsigned nIn, int* bitsOut, int* countOut, int* bitsIn, int* countIn)
{
  VPcmFloModem* const ctx = obj->ctx_;
  K56FlexHandle* const k56 = obj->k56_;
  unsigned n;  /* samples in this block */
  int mode;    /* cached obj->mode_ */
  int st = 0;  /* cached obj->state_ */
  int st0 = 0; /* state_ at the start of mode 0 */
  int r;
  unsigned lvl;

  if (obj->w262_ == 0)
    return obj->state_;

  n = nIn & ~3u;
  obj->sampleTime_ = (int32_t)(((uint32_t)obj->sampleTime_ + n) & 0x7fffffffu);
  mode = obj->mode_;

  /* transmit side: the modes that send data pack the caller's bits into 16-bit words first */
  if (mode == VPCM_MODE_V34)
    goto txBitPack;
  if (mode == VPCM_MODE_V90)
  {
    if (obj->modeChar_ == V34ROLE_ANSWER)
      goto txBitPack;
    goto entranceFilter;
  }
  if (mode == VPCM_MODE_K56FLEX)
  {
    if (obj->modeChar_ == V34ROLE_CALL)
      goto txBitPack;
  }
  goto entranceFilter;

txBitPack:
  /* pack bitsIn (one bit per entry, bit 0 first) into 16-bit words appended to txWords_ */
  {
    int cnt = *countIn >> 4;
    int wordsDone = 0;
    unsigned bitPos = 0;
    if (cnt > 0)
    {
      unsigned tail = obj->txTail_;
      do
      {
        unsigned w = 0;
        int bit;
        for (bit = 0; bit <= 15; bit++)
          w += ((unsigned)bitsIn[bitPos++] & 1u) << bit;
        obj->txWords_[tail] = w;
        tail++;
        obj->txTail_ = tail;
        wordsDone++;
      } while ((*countIn >> 4) > wordsDone);
    }
  }

entranceFilter:
  if (ctx->entranceFilter_ != 0)
  {
    ctx->iir_.process(inp, inp, n);
    mode = obj->mode_;
  }

  /* dispatch on the session mode */
  if ((unsigned)mode > 10u)
    goto illegalState;
  switch (mode)
  {
  case VPCM_MODE_V34:
    goto dataPump;
  case VPCM_MODE_V90:
    goto v90Mode;
  case VPCM_MODE_V92:
    goto pcmModemMode;
  case VPCM_MODE_K56FLEX:
    goto k56FlexMode;
  case VPCM_MODE_QC_LINE_VERIFICATION:
    goto qcLineVerifMode;
  case VPCM_MODE_WAIT_USER_RESPONSE:
    goto waitUserResponse;
  case VPCM_MODE_ANSAM_OUTGOING:
    goto holdWaitAnsam;
  case VPCM_MODE_ON_HOLD:
    goto holdSilence;
  case VPCM_MODE_RECONNECT_WAIT:
    goto reconnectDelay;
  case VPCM_MODE_RECONNECT:
    goto reconnectMode;
  default:
    goto sessionEnd;
  }

illegalState:
  if (LEVEL_ON())
    dsplibs_debug_printf("VPcmV34Main: Illegal Modem State !!!!\r\n");

reloadState:
  st = obj->state_;

  /* retrain detector (states 3..6): a notch-filtered copy of the ring is compared with the unfiltered energy in blocks of 64 samples */
retrainDetector:
  if ((unsigned)(st - 3) > 3u)
    return st;
  {
    int N = s16(obj->tx.w2aa6_);
    int i;
    int c1 = s16(obj->retrain.rdc_), c2 = s16(obj->retrain.rde_), c3 = s16(obj->retrain.rd10_);
    const int16_t* rp = (const int16_t*)ringp(obj, 0);
    int flag = 0;

    for (i = 0; i < N; i++)
    {
      int x = *rp++;
      int w2 = s16(obj->retrain.rd2_) * c3;
      int w0 = s16(obj->retrain.rd0_);
      int t3, t2, t1, y, w4old;
      unsigned sum1, sum2, cnt;

      /* three-tap filter, coefficients Q14 */
      obj->retrain.rd2_ = obj->retrain.rd0_;
      t3 = (int16_t)((w2 + 0x2000) >> 14);
      w4old = obj->retrain.rd4_;
      obj->retrain.rd4_ = (uint16_t)x;
      t2 = (int16_t)((w0 * c2 + 0x2000) >> 14);
      t1 = (int16_t)(((int)(int16_t)w4old * c1 + 0x2000) >> 14);
      y = x + (t2 - t3);
      y -= t1;
      y += obj->retrain.rd6_;
      obj->retrain.rd6_ = (uint16_t)w4old;
      y = (int16_t)y;
      obj->retrain.rd0_ = (uint16_t)y;

      /* accumulate input and filtered energy (>> 6); every 64 samples decide whether the notch removed most of the input energy */
      sum1 = (uint32_t)((x * x + 0x20) >> 6) + obj->retrain.rd14_;
      sum2 = (uint32_t)((y * y + 0x20) >> 6) + obj->retrain.rd18_;
      cnt = obj->retrain.rd1c_ + 1u;
      if (cnt == 0x40u)
      {
        int inRatio = (int)sum1 >> 2;
        if (inRatio > (int)sum2 && (int)sum1 > 0x249f0 && (int)sum2 <= 0x22550f)
        {
          uint32_t k;
          obj->retrain.rd14_ = sum1;
          k = obj->retrain.rd8_;
          obj->retrain.rd18_ = sum2;
          obj->retrain.rd1c_ = 0x40;
          k++;
          obj->retrain.rd8_ = k;
          if (LEVEL_ON())
            dsplibs_debug_printf("********** retrainDetector() notchDetectSigCnt = %d energyInp>>NOTCH_IN_OUT_RATIO_SHIFT = %d energyOut = %d\r\n", (int)k,
                                 inRatio, (int)sum2);
        }
        else
        {
          obj->retrain.rd8_ = 0;
        }
        obj->retrain.rd14_ = 0;
        obj->retrain.rd18_ = 0;
        obj->retrain.rd1c_ = 0;
      }
      else
      {
        obj->retrain.rd1c_ = cnt;
        obj->retrain.rd18_ = sum2;
        obj->retrain.rd14_ = sum1;
      }

      if ((int32_t)obj->retrain.rd8_ > 5)
      {
        flag = 1;
        break;
      }
    }

    if (!flag)
      return obj->state_;

    if (LEVEL_ON())
      dsplibs_debug_printf("VPcmV34Main: Retrain Detected by Tone detector !\r\n");
    VPcmV34InitiateRetrain(obj, 0);
    return obj->state_;
  }

  /* ---- modes 6 / 7: silence output while the ANSam detector watches (outgoing call, or modem on hold) ---- */
holdWaitAnsam:
  {
    int waitMin = obj->wabdc_;
    int elapsed = obj->waa74_;

    if (waitMin < elapsed)
    {
      obj->state_ = 0xd;
      goto holdSilence;
    }
    if ((unsigned)(waitMin - n) < (unsigned)elapsed)
    {
      if (LEVEL_ON())
      {
        dsplibs_debug_printf("VPcmV34Main: ANSam not detected on out-going, assuming 3-way call supported !\r\n");
        mode = obj->mode_;
      }
    }
    obj->state_ = 0xe;
  }

holdSilence:
  zeroOut(outp, n);
  *countOut = 0;
  *countIn = 0;
  if (mode == VPCM_MODE_ON_HOLD)
    obj->state_ = 0xd;

  /* hold timer: end the session when the ISP timeout expires */
  {
    int holdTime = (int)(n + (unsigned)obj->wabdc_);
    int holdTimeout = obj->mohD8_;
    obj->wabdc_ = holdTime;
    if (holdTimeout != -1 && holdTime >= holdTimeout)
    {
      if (LEVEL_ON())
        dsplibs_debug_printf("VPcmV34Main: Modem On Hold Timeout expired , ending session !!!\r\n");
      obj->state_ = 0x10;
      obj->mode_ = VPCM_MODE_END;
    }
  }

  /* ANSam detection */
  r = (int)ctx->ansam_.GenericToneDetector::process(inp, n);
  if (r != 0)
  {
    if (obj->mode_ == VPCM_MODE_ON_HOLD)
    {
      if (obj->mohE9_ != 0 && (int)obj->wabdc_ <= 0xbb7f)
      {
        if (LEVEL_ON())
          dsplibs_debug_printf("VPcmV34Main: ANSam detected on out going call (later case, after %d smp) ! assuming no 3-way call...\r\n", (int)obj->wabdc_);
        obj->mohE2_ = 2;
        goto ansamDetected;
      }
      if (LEVEL_ON())
        dsplibs_debug_printf("VPcmV34Main: ANSam detected on hold ! requesting Reconnect...\r\n");
      goto ansamDetected;
    }
    if (LEVEL_ON())
      dsplibs_debug_printf("VPcmV34Main: ANSam detected on out going call ! assuming no 3-way call...\r\n");
    obj->mohE2_ = 2;
  ansamDetected:
    obj->state_ = 0xf;
    obj->mode_ = VPCM_MODE_RECONNECT;
  }

  ringFill(obj, inp, n);
  goto reloadExit;

  /* ---- mode 5: waiting for the user response (30 s timeout, then a retrain) ---- */
waitUserResponse:
  {
    bool timedOut;
    zeroOut(outp, n);
    timedOut = (uint32_t)obj->sampleTime_ > 0x464ffu;
    *countOut = 0;
    obj->state_ = 0xc;
    *countIn = 0;
    if (timedOut)
    {
      if (LEVEL_ON())
        dsplibs_debug_printf("VPcmV34Main: waiting for user response for 30sec, initiating retrain !\r\n");
      VPcmV34InitiateRetrain(obj, 0);
    }
    ringFill(obj, inp, n);
    goto reloadExit;
  }

  /* ---- mode 4: QC line verification ---- */
qcLineVerifMode:
  {
    V01_ModemParams* pp;
    r = ctx->qcLineVerification(inp, outp, n, bitsOut, countOut, bitsIn, countIn);

    if (r == 0)
    {
      obj->state_ = 0xa;
      goto qcRingFill;
    }

    /* verification finished: choose between the short and the full phase 2 */
    lvl = dsplibs_debug_level;
    if (lvl > 1u)
    {
      dsplibs_debug_printf("VPcmV34Main: Line verification period completed !!!\r\n");
      lvl = dsplibs_debug_level;
    }
    pp = obj->params_;
    if (!(pp->flags2 & 0x20u))
    {
      obj->shortPhase2_ = 0;
      if (lvl > 1u)
        dsplibs_debug_printf("VPcmV34Main: Moving to full phase2 upon DP Manager setting...\r\n");
      goto qcPhase2Select;
    }
    if (s16(obj->wabca_) == (int)ctx->qcStatus_)
    {
      if (lvl > 1u)
        dsplibs_debug_printf("VPcmV34Main: Moving to short phase2 due to same line verification...\r\n");
      obj->shortPhase2_ = 1;
      goto qcPhase2Select;
    }
    obj->shortPhase2_ = 0;
    if (lvl > 1u)
      dsplibs_debug_printf("VPcmV34Main: Moving to full phase2 due to false same line verification...\r\n");
  qcPhase2Select:
    if (obj->shortPhase2_ == 0)
    {
      V92Phase2Info* info = ctx->v92_.info_;
      V90Parameters* vp;
      info->shortPhase2Local_ = 0;
      vp = ctx->v90_.params_;
      obj->wabca_ = 0;
      vp->init();
    }

    obj->mode_ = VPCM_MODE_V34;
    obj->state_ = 0;
  qcRingFill:
    ringFill(obj, inp, n);
    goto reloadExit;
  }

  /* ---- mode 3: K56Flex ---- */
k56FlexMode:
  obj->tx.blkLen_ = (uint16_t)n;

  /* run the transmit side until the block is covered */
  while (s16(obj->tx.txIdx_) < s16(obj->tx.blkLen_))
  {
    if (obj->tx.txFlags_ & 0x10u)
      modulatevector(obj);
    else
      v34handshak(obj);
  }

  if (n != 0)
    echoLoop(obj, &inp, &outp, n);
  inp -= n;
  r = ((K56FlexFloModem*)k56)->k56FlexRunDemodulator(inp, n, bitsOut, countOut);
  if ((unsigned)r > 6u)
    goto k56Failed;
  switch (r)
  {
  case 0:
    goto k56Failed;
  case 1:
    obj->state_ = 3;
    goto k56Done;
  case 2:
    goto k56Connected;
  case 3:
    obj->state_ = 5;
    goto k56Connected;
  case 4:
    VPcmV34InitiateRetrain(obj, DP_K56);
    goto k56Done;
  case 5:
    VPcmV34InitiateRetrain(obj, DP_V34);
    goto k56Done;
  default:
    obj->mode_ = VPCM_MODE_END;
    obj->state_ = 0x10;
    goto k56Done;
  }

k56Connected:
  {
    V01_ModemParams* pp;
    int flag = 1;
    uint8_t b = obj->b_ac16_;
    obj->rx.blkb.wE4c_ = (uint16_t)flag;
    if (b == 0)
    {
      uint32_t w = k56->word0_;
      obj->b_ac16_ = 1;
      obj->dgC08_ = w;
    }
    pp = obj->params_;
    pp->flags3 = (uint8_t)(pp->flags3 | 4u);
    goto k56Done;
  }

k56Failed:
  obj->state_ = 2;

k56Done:
  compactQueue(obj);
  st = obj->state_;
  if (st <= 3)
  {
    *countIn = 0;
    goto retrainDetector;
  }
  *countIn = wanted(obj);
  goto retrainDetector;

  /* ---- mode 2: V.92 / PCM modem ---- */
pcmModemMode:
  r = ctx->runPcmModem(inp, outp, n, bitsOut, countOut, bitsIn, countIn);
  if ((unsigned)r > 8u)
    goto pcmFailed;
  switch (r)
  {
  case 0:
    goto pcmFailed;
  case 1:
    obj->state_ = 3;
    goto pcmRingFill;
  case 2:
    goto pcmConnected;
  case 3:
    obj->wabc4_ = 1;
    obj->state_ = 5;
    obj->wa248_ = 0;
    goto pcmRingFill;
  case 4:
    obj->state_ = 0xb;
    obj->mode_ = VPCM_MODE_WAIT_USER_RESPONSE;
    obj->sampleTime_ = 0;
    goto pcmRingFill;
  case 5:
    VPcmV34InitiateRetrain(obj, DP_V92);
    goto pcmRingFill;
  case 6:
    VPcmV34InitiateRetrain(obj, DP_V90);
    goto pcmRingFill;
  case 7:
    VPcmV34InitiateRetrain(obj, DP_V34);
    goto pcmRingFill;
  default:
    obj->mode_ = VPCM_MODE_END;
    obj->state_ = 0x10;
    goto pcmRingFill;
  }

pcmConnected:
  /* first connect: record the receive and transmit rates (the transmit rate is bits per symbol * 8000 / 12, rounded) */
  if (obj->b_ac16_ == 0)
  {
    int32_t rxBitRate = ctx->v90_.dem_->getBitRate();
    V92Modulator* mod = ctx->v92_.mod_;
    int txBitRate = 0;
    obj->dgC08_ = (uint32_t)rxBitRate;
    if (mod->state_ == 3)
    {
      uint32_t bitsPerSec12 = mod->b2s_->tx_->w0_ * 8000u;
      long double txRateF = 0.5L;
      txRateF += (long double)bitsPerSec12 * (long double)bitsFloat(0x3daaaaabu); /* 1/12 as a float */
      txBitRate = (int)(uint32_t)(uint64_t)(int64_t)txRateF;
    }
    obj->dgC04_ = (uint32_t)txBitRate;
    obj->b_ac16_ = 1;
  }

  obj->wabc4_ = 1;
  obj->state_ = 4;
  obj->wa248_ = 0;
  goto pcmRingFill;

pcmFailed:
  obj->state_ = 2;

pcmRingFill:
  {
    int i;
    for (i = 0; i < (int)n; i++)
    {
      int idx = (int16_t)obj->tx.w2aa6_;
      unsigned nx;
      *ringp(obj, idx) = trunc16(ctx->ecOut_[i]);
      nx = (uint16_t)(idx + 1);
      obj->tx.w2aa6_ = (uint16_t)(nx > 0x257u ? 0u : nx);
    }
  }
  goto reloadExit;

  /* ---- mode 1: V.90 ---- */
v90Mode:
  {
    int pcmMode = obj->pcmMode_;
    if (pcmMode <= 1)
    {
      st = obj->state_;
      goto wrongState;
    }
    st = obj->state_;
    if (st <= 1)
      goto wrongState;

    obj->tx.blkLen_ = (uint16_t)n;

    /* run the handshake / rate-renegotiation transmit side until the block is covered */
    if (s16(obj->tx.txIdx_) < (int)(int16_t)(uint16_t)n)
    {
      for (;;)
      {
        if (pcmMode <= 0xe)
        {
          if (pcmMode > 0xa)
            v90RateReneg(obj);
          else if (obj->tx.txFlags_ & 0x10u)
            modulatevector(obj);
          else
            v34handshak(obj);
        }
        else
        {
          v90RateRenegSilence(obj);
        }
        if (s16(obj->tx.txIdx_) >= s16(obj->tx.blkLen_))
          break;
        pcmMode = obj->pcmMode_;
      }
    }

    if (n != 0)
      echoLoop(obj, &inp, &outp, n);
    inp -= n;
    r = ctx->v90RunDemodulator(inp, n, bitsOut, countOut);
    goto v90Result;

  wrongState:
    if (LEVEL_ON())
    {
      dsplibs_debug_printf("VPcmV34Main: BUG BUG, Wrong state & statuses... (v90receiver = %d, cond = %d ; DPstatus = %d, cond = %d)\r\n", pcmMode, 2, st, 2);
      goto reloadExit;
    }
    goto retrainDetector;
  }

v90Result:
  if ((unsigned)r > 8u)
    goto v90Failed;
  switch (r)
  {
  case 1:
    obj->state_ = 3;
    goto v90Done;
  case 2:
    goto v90Connected;
  case 3:
    obj->rx.blkb.wE4c_ = 1;
    obj->state_ = 5;
    obj->wa248_ = 0;
    goto v90Done;
  case 4:
    obj->state_ = 0xb;
    obj->mode_ = VPCM_MODE_WAIT_USER_RESPONSE;
    obj->sampleTime_ = 0;
    goto v90Done;
  case 5:
    VPcmV34InitiateRetrain(obj, DP_V90);
    goto v90Done;
  case 7:
    VPcmV34InitiateRetrain(obj, DP_V34);
    goto v90Done;
  case 8:
    obj->mode_ = VPCM_MODE_END;
    obj->state_ = 0x10;
    goto v90Done;
  default:
    goto v90Failed;
  }

v90Connected:
  obj->rx.blkb.wE4c_ = 1;
  if (obj->b_ac16_ == 0)
    obj->dgC08_ = (uint32_t)ctx->v90_.dem_->getBitRate();
  if (obj->wabc4_ == 0)
  {
    int flag = 0;
    if (obj->wabc6_ != 0 && obj->info1dPending_ != 0)
      flag = 1;
    obj->wabc4_ = (uint16_t)flag;
  }
  obj->wa248_ = 0;
  goto v90Done;

v90Failed:
  obj->state_ = 2;

v90Done:
  /* honour a pending request to clear the output samples (V.90 only) */
  if (obj->b_abfe_ != 0 && n > 0x30u)
  {
    unsigned clearLen = n + n;
    unsigned bytes;
    if (LEVEL_ON())
      dsplibs_debug_printf("VPcmV34Main: requestOutputSampleClear (asking %d samples clear) !!!\r\n", (int)clearLen);
    obj->b_abfe_ = 0;
    bytes = n << 2;
    outp -= n;
    sysdep_memset(outp, 0, bytes);
    obj->wac44_ = clearLen;
    obj->wac40_ = 1;
    obj->wac48_ = 0;
    obj->params_->flags51 = (uint8_t)(obj->params_->flags51 | 1u);
    V34EchoHistoryBackwardClean(obj, clearLen + n);
  }

  compactQueue(obj);
  st = obj->state_;
  if (st <= 3)
  {
    *countIn = 0;
    goto retrainDetector;
  }
  *countIn = wanted(obj);
  goto retrainDetector;

  /* ---- mode 0: the V.34 data pump ---- */
dataPump:
  st = obj->state_;
  st0 = st;

  /* per sample: echo-cancel / receive, then run the transmit side while the block is not covered */
  if (n != 0)
  {
    unsigned left = n;
    for (;;)
    {
      float x = *inp;
      inp++;
      obj->w260_ = (uint16_t)trunc16(x);
      modem_serrint(obj);
      *outp = (float)s16(obj->w25e_);
      outp++;
      if (s16(obj->tx.txIdx_) < s16(obj->tx.blkLen_) || s16(obj->rx.rxW0_) > 5)
      {
        if (obj->tx.txFlags_ & 0x10u)
          datapumpv34(obj);
        else
          v34handshak(obj);
      }
      if (--left == 0)
        break;
    }
    st = obj->state_;
  }

  /* entering state 4 or 5: raise bit 2 of flags3 */
  if (st != st0 && (unsigned)(st - 4) <= 1u)
  {
    obj->params_->flags3 = (uint8_t)(obj->params_->flags3 | 4u);
    st = obj->state_;
  }

  /* state 0: stop masking CAS detection 4800 samples after the start of training */
  if (st == 0)
  {
    unsigned sinceTrain = (uint32_t)obj->sampleTime_ - obj->trainStartTime_;
    if (sinceTrain > 0x12bfu && sinceTrain < n + 0x12c0u)
    {
      if (LEVEL_ON())
        dsplibs_debug_printf("VPcmV34Main: Masking CAS detection after %d in train...\r\n", (int)sinceTrain);
      obj->params_->flags3 = (uint8_t)(obj->params_->flags3 & 0xfbu);
      st = obj->state_;
    }
  }

  compactQueue(obj);
  if (st <= 3)
    *countIn = 0;
  else
    *countIn = wanted(obj);

  /* unpack the receive words into the output bit array, 16 bits per word, bit 0 first */
  {
    unsigned i = 0, w;
    int* bitPtr = bitsOut;
    for (;;)
    {
      w = obj->rxWordCnt_;
      if (w <= i)
        break;
      {
        uint32_t v = obj->rxWords_[i];
        int bit;
        for (bit = 0; bit <= 15; bit++)
        {
          *bitPtr++ = (int)(v & 1u);
          v >>= 1;
        }
      }
      i++;
    }
    obj->rxWordCnt_ = 0;
    *countOut = (int)(w << 4);
  }

  /* post-processing by state */
  if ((unsigned)st > 0x10u)
    goto retrainDetector;
  switch (st)
  {
  case 1:
    goto phase2Complete;
  case 2:
    goto phase3Entry;
  case 0xd:
    goto holdApproved;
  case 0xf:
    goto reconnectRequest;
  case 0x10:
    goto endTransmission;
  default:
    goto retrainDetector;
  }

endTransmission:
  if (LEVEL_ON())
  {
    dsplibs_debug_printf("VPcmV34Main: End Transmission indicated by V.34 / Handshake...\r\n");
    st = obj->state_;
  }
  obj->mode_ = VPCM_MODE_END;
  goto retrainDetector;

reconnectRequest:
  {
    unsigned mohReq = obj->mohE2_;
    int minWait = (mohReq < 1u) ? -1 : 0;
    obj->mode_ = VPCM_MODE_RECONNECT_WAIT;
    minWait &= 0xbb80;
    minWait += 0x2580;
    obj->waa74_ = 0;
    obj->wabdc_ = minWait;
    st = 0;
    obj->state_ = 0;
    if (LEVEL_ON())
    {
      dsplibs_debug_printf("VPcmV34Main: Reconnect request indicated from phase2 (waiting min time = %d before reconenct request)...\r\n", minWait);
      goto reloadState;
    }
    goto retrainDetector;
  }

phase3Entry:
  /* state 2: switch to the phase 3 modem selected in phase 2 */
  if (obj->pcmMode_ > 1)
  {
    if (ctx->v92Session_ == 0)
    {
      if (LEVEL_ON())
        dsplibs_debug_printf("VPcmV34Main: Moving to Phase3 Modem V90..\r\n");
      obj->mode_ = VPCM_MODE_V90;
    }
    else
    {
      if (LEVEL_ON())
        dsplibs_debug_printf("VPcmV34Main: Moving to Phase3 Modem V92..\r\n");
      obj->mode_ = VPCM_MODE_V92;
    }
  }
  if (obj->k56Mode_ > 1)
  {
    if (LEVEL_ON())
      dsplibs_debug_printf("VPcmV34Main: Moving to Phase3 Modem K56Flex..\r\n");
    obj->mode_ = VPCM_MODE_K56FLEX;
    obj->w254_ = 0;
  }
  st = obj->state_;
  goto retrainDetector;

phase2Complete:
  if (st0 != 0)
    goto phase2CompleteWait;
  {
    V01_ModemParams* pp = obj->params_;
    int echoMid;
    obj->wAbff_ = 0;
    obj->w240_ = 0;
    if ((pp->flags3 & 2u) && obj->wabc4_ != 0 && obj->wa248_ == 0)
    {
      obj->wa248_ = 1;
      echoMid = s16(obj->rate.baud_) * 5 + s16(obj->rate.period_) - 0x24c;
    }
    else
    {
      echoMid = ((s16(obj->rate.baud_) * 15) >> 3) + s16(obj->rate.period_) - 0x24c;
    }
    obj->echoMid_ = echoMid;
    edprintf("VPcmV34Main: phase3halfDuplexLength = %d symbols (baud %d)\r\n", echoMid, s16(obj->rate.baud_));
    if (obj->pcmMode_ > 1)
      ctx->vPcmResetPhase3Modem();
    else if (obj->k56Mode_ > 1)
      ((K56FlexFloModem*)k56)->k56FlexEnterPhase3();
  }

phase2CompleteWait:
  /* report P2 COMPLETE twice, with silence of w240_ samples between the indications and the switch to the phase 3 modem */
  {
    int p2Delay = (int)(n + (unsigned)obj->w240_);
    uint8_t p2Phase;
    obj->w240_ = p2Delay;
    p2Phase = obj->wAbff_;
    if (p2Phase == 0)
    {
      if (p2Delay <= 0x5f)
      {
        if (LEVEL_ON())
          dsplibs_debug_printf("VPcmV34Main: Wait (before P2 COMPLETE)...\r\n");
        return 0;
      }
      obj->wAbff_ = 1;
      if (LEVEL_ON())
        dsplibs_debug_printf("VPcmV34Main: Indicating First P2 COMPLETE... (after %d)\r\n", obj->w240_);
      return 1;
    }
    if (p2Phase == 1)
    {
      V01_ModemParams* pp = obj->params_;
      p2Delay += pp->p2Silence;
      obj->w240_ = p2Delay;
      if (LEVEL_ON())
        dsplibs_debug_printf("On PHASE2_COMPLETE: added Silence = %d, p2DelayCntr = %d\r\n", pp->p2Silence, p2Delay);
    }
    if (obj->w240_ <= 0x240)
    {
      if (LEVEL_ON())
        dsplibs_debug_printf("VPcmV34Main: Wait (after P2 COMPLETE)...\r\n");
      obj->wAbff_ = (uint8_t)(obj->wAbff_ + 1u);
      return 1;
    }
    if (st0 != 1)
      goto reloadExit;
    if (obj->pcmMode_ > 1 && ctx->v92Session_ != 0)
    {
      if (LEVEL_ON())
        dsplibs_debug_printf("VPcmV34Main: Moving to Phase3 Modem V92..\r\n");
      obj->mode_ = VPCM_MODE_V92;
    }
    if (obj->k56Mode_ > 1)
    {
      if (LEVEL_ON())
        dsplibs_debug_printf("VPcmV34Main: Moving to Phase3 Modem K56Flex..\r\n");
      obj->mode_ = VPCM_MODE_K56FLEX;
      obj->w254_ = 0;
    }
    st = obj->state_;
    goto retrainDetector;
  }

holdApproved:
  /* state 13: modem on hold approved; table of ISP timeouts in seconds indexed by mohE0_ */
  {
    static const int hold[14] = {0, 10, 20, 30, 40, 60, 120, 180, 240, 360, 480, 720, 960, -1};
    unsigned holdIdx = (uint16_t)obj->mohE0_;
    int holdSec = holdIdx > 0xdu ? 0 : hold[holdIdx];
    obj->mohD8_ = (uint32_t)holdSec;
    obj->wabdc_ = 0;
    if (LEVEL_ON())
    {
      dsplibs_debug_printf("VPcmV34Main: Modem On Hold approved by phase2 (ISP timeout is %d seconds) !!\r\n", holdSec);
      holdSec = (int)obj->mohD8_;
      st = obj->state_;
    }
    if (holdSec > 0)
      obj->mohD8_ = (uint32_t)(holdSec * 0x2580);
    obj->mode_ = VPCM_MODE_ON_HOLD;
    goto retrainDetector;
  }

  /* ---- modes 8, 9, 10: reconnect delay, reconnect request, end of session ---- */
reconnectDelay:
  {
    unsigned elapsed;
    bool waiting;
    zeroOut(outp, n);
    *countOut = 0;
    *countIn = 0;
    elapsed = n + (unsigned)obj->waa74_;
    waiting = (int)elapsed < (int)obj->wabdc_;
    obj->waa74_ = (int32_t)elapsed;
    if (waiting)
    {
      obj->state_ = 0;
      st = 0;
      goto retrainDetector;
    }
    obj->mode_ = VPCM_MODE_RECONNECT;
    if (LEVEL_ON())
    {
      obj->state_ = 0;
      dsplibs_debug_printf("VPcmV34Main: Delay ended, indicating reconnect request...\r\n");
    }
    st = 0xf;
    obj->state_ = 0xf;
    goto retrainDetector;
  }

reconnectMode:
  zeroOut(outp, n);
  *countOut = 0;
  *countIn = 0;
  st = 0xf;
  obj->state_ = 0xf;
  goto retrainDetector;

sessionEnd:
  zeroOut(outp, n);
  *countOut = 0;
  *countIn = 0;
  obj->state_ = 0x10;
  st = 0x10;
  goto retrainDetector;

reloadExit:
  st = obj->state_;
  goto retrainDetector;
}
