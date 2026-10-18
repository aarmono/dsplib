/**
 * @file dp_vpcm.cpp
 * @brief Data pump glue for V.34 / V.90 / V.92: session create / run / delete and registration with the host.
 *
 * Connects the host's data pump interface to the V.34 module object (Z01_State, dp_vpcm.h):
 * - vpcm_create(), vpcm_delete() and vpcm_run() are the three entry points of the operations record `vpcm_op`; they are file-local (`static`) and reached only
 *   through that record.  dp_vpcm_init() / dp_vpcm_exit() register / deregister the record for the data pump ids DP_V34, DP_V90 and DP_V92.
 * - vpcm_run() converts the host's 16-bit samples to float, hands them to VPcmV34Progress() in blocks of a multiple of four, converts the result back,
 *   moves bits between the host and the protocol and tracks the link state (training, up, error) from the status the protocol returns.
 * - VPCMXF_Create() / VPCMXF_Delete() / VPCMXF_SessionTermination() construct, destroy and terminate the VPcmFloModem (pcm_leaf_classes.h) that the
 *   V.34 object refers to; dp_param_get() returns the modem's data pump parameter block.
 *
 * The file is C++ (no exceptions, no RTTI, freestanding headers only) because VPCMXF_* construct and destroy a class; the symbols are `extern "C"`.
 *
 * Numeric conventions kept on purpose:
 * - Float to integer conversions truncate and map NaN and every out-of-range value to the most negative integer (trunc32(), trunc16()), the behaviour of the
 *   x87 conversion instructions.
 * - VPCMXF_Create() computes the maximum data buffer as trunc32((long double)rate * 9.6 + 0.5f) for the analog side and rate * 8.0f + 0.5f for the digital
 *   side; `rate` is unsigned.
 * - The computational mode argument selects the V.90 / V.92 modes: 2 -> (V.90 0, V.92 1), 3 -> (1, 1), 1 -> (1, 0), anything else (0, 0).
 * - The bit count requested by the protocol is clamped with an UNSIGNED compare against 0x400, so a negative count also becomes 0x400.
 */

/* The modem stays an opaque `void *` in this file (struct dp::modem_): core/dsp_host.h must not declare the typed modem_* family (see there). */
#define DSP_HOST_MODEM_OPAQUE
#include "dp_vpcm.h"
#include "vpcm_session_wrappers.h"

#include "core/dsp_debug_level.h" /* dsplibs_debug_level, LEVEL_ON() */

/* The data pump ids (DP_*) and the host parameter ids (MDMPRM_*, Get*) come from the freestanding core/host_interface.h.  The host functions it calls are
 * declared by core/dsp_host.h with `void *` for the modem (DSP_HOST_MODEM_OPAQUE, defined above). */
#include "core/host_interface.h" /* DP_V34 / DP_K56 / DP_V90 / DP_V92, MDMPRM_* */
#include "core/dsp_constants.h"  /* SAMPLE_RATE_* */
#include "core/dsp_host.h"       /* dsplibs_debug_printf, sysdep_memset, sysdep_memcpy */
#include "vpcm_v34_api.h"
#include "pcm_leaf_classes.h"

/* vpcm_create / vpcm_delete / vpcm_run are file-local; -DZ01_EXPORT_LOCALS gives them external linkage so that test tools can call them directly */
#ifdef Z01_EXPORT_LOCALS
#define Z01_STATIC
#else
#define Z01_STATIC static
#endif

/** @brief Convert to int32_t by truncation; NaN and values outside (-2147483649, 2147483648) give 0x80000000 (the x87 `fistpl` result). */
static inline int32_t trunc32(long double v)
{
  if (!(v > -2147483649.0L && v < 2147483648.0L))
    return (int32_t)0x80000000u;
  return (int32_t)v;
}

/** @brief Convert to int16_t by truncation; NaN and values outside (-32769, 32768) give 0x8000 (the x87 `fistps` result). */
static inline int16_t trunc16(long double v)
{
  if (!(v > -32769.0L && v < 32768.0L))
    return (int16_t)0x8000;
  return (int16_t)v;
}

/* ---------------------------------------------------------------------------------------------------- dp_param_get */

/**
 * @brief Return the data pump parameter block of a modem.
 * @param m  the host modem
 * @return pointer to the V01_ModemParams block (MDMPRM_DPRUNTIME), as a `long` (pointer sized on x86-64)
 */
extern "C" long dp_param_get(void* m) { return modem_get_param(m, MDMPRM_DPRUNTIME); }

/* ---------------------------------------------------------------------------------------------------- VPCMXF_* */

/**
 * @brief Allocate and construct the VPcmFloModem of a session and reset its handshake state.
 * @param side    0 = analog side, otherwise digital side
 * @param ctx     the V.34 module object that owns the modem
 * @param params  data pump parameter block
 * @param rate    sample rate (unsigned); the maximum data buffer is derived from it (about 9.6 or 8 times the rate, plus rounding)
 * @param mode    computational mode: 0 none, 1 V.90 only, 2 V.92 only, 3 both
 * @return the new modem, or 0 if allocation failed
 * @warning The constructor runs before the allocation result is tested (placement new has no null check), so a failed allocation is written through by
 *          the constructor before it is detected.
 */
extern "C" VPcmFloModem* VPCMXF_Create(int side, void* ctx, V01_ModemParams* params, unsigned rate, int mode)
{
  VPcmFloModem* m;
  int32_t n;
  V90ComputationalMode m90 = V90_COMPMODE_0;
  V92ComputationalMode m92 = V92_COMPMODE_0;
  bool analog = (side == 0);

  /* maximum data buffer, from the sample rate */
  if (analog)
    n = trunc32((long double)rate * 9.6 + 0.5f);
  else
    n = trunc32((long double)rate * 8.0f + 0.5f);

  /* computational modes of the V.90 and V.92 modems */
  if (mode == 2)
  {
    m92 = V92_COMPMODE_1;
  }
  else if (mode == 3)
  {
    m90 = V90_COMPMODE_1;
    m92 = V92_COMPMODE_1;
  }
  else if (mode == 1)
  {
    m90 = V90_COMPMODE_1;
  }

  if (LEVEL_ON())
    dsplibs_debug_printf("VPCMXF_Create: side is %s, maxDataBuffer - %d\r\n", side != 0 ? "Digital" : "Analog", (int)n);

  m = (VPcmFloModem*)sysdep_malloc(sizeof(VPcmFloModem));
  new (m) VPcmFloModem(ctx, analog ? V90_SIDE_ANALOG : V90_SIDE_DIGITAL, (_tagModemParameters*)params, (unsigned)n, m90, m92);
  if (m == 0)
  {
    if (LEVEL_ON())
      dsplibs_debug_printf("VPCMXF_Create: new VPcmFloModem() failed.\n");
    return 0;
  }

  /* initial handshake state */
  m->cpnotLen_ = 0;
  m->bitPtr_ = 0;
  m->trainConstel_ = 0;
  m->rrnConstel_ = 0;
  m->localRrn_ = 0;
  m->dropToV34_ = 0;
  m->clearDown_ = 0;
  m->v34Flags_[0] = 1;
  m->v34Flags_[1] = 0;
  m->v34Flags_[2] = 1;
  m->v34Flags_[3] = 1;
  m->v34Flags_[4] = 1;
  m->v34Flags_[5] = 0;
  m->cpLen_ = 0;
  m->jaFlag_ = 0;
  m->cpFlag_ = 0;
  m->cpnotFlag_ = 0;
  m->cpnotCopied_ = 0;
  m->nofBitsPhase4_ = 2;
  m->cpSeqCnt_ = 0;
  m->minTxSeq_ = 1;
  return m;
}

/**
 * @brief Destroy a VPcmFloModem and free it.
 * @param m  the modem, or 0 (then nothing happens)
 * @note The six members that have destructors are destroyed explicitly in reverse member order, then the storage is released.
 */
extern "C" void VPCMXF_Delete(VPcmFloModem* m)
{
  if (m == 0)
    return;
  m->iir_.~GenericIIR<float, double>();
  m->sine_.~SineWave<float, float>();
  m->ansam_.~ANSamToneDetector();
  m->ec_.~V92EchoCanceller();
  m->v92_.~V92Modem();
  m->v90_.~V90Modem();
  sysdep_free(m);
}

/**
 * @brief Run the V.90 demodulator's session termination.
 * @param m  the modem
 * @return the demodulator's result
 * @warning Not null-checked: `m` and its `v90_.dem_` must be valid.
 */
extern "C" int VPCMXF_SessionTermination(VPcmFloModem* m) { return m->v90_.dem_->sessionTermination(); }

/* ---------------------------------------------------------------------------------------------------- vpcm_* */

/**
 * @brief Create a data pump session (operations record `create`).
 * @param modem   the host modem
 * @param id      requested data pump id (DP_V34, DP_V90 or DP_V92)
 * @param caller  host caller flag; VPcmV34Create() receives the test `caller == 0`
 * @param rate    sample rate; only SAMPLE_RATE_9600 is accepted
 * @param frag    fragment size in samples, at most 0x30
 * @param op      the operations record, stored in the session
 * @return the Z01_State session, or 0 on a rejected rate / fragment or any allocation / creation failure (everything allocated so far is released)
 * @note Sets the rate limits (host min / max rate, the maximum clamped to 0xdac0, the upstream range fixed at 4800 .. 0x8340), the V.92 flag bit, the
 *       initial silence and the audio delay budget: the host's I/O delay plus 4, limited to 0xf4, any excess being asked back from the host as a delay
 *       update and remembered in `addDelay_` (at least 0x180).
 */
Z01_STATIC void* vpcm_create(void* modem, int id, int caller, int rate, int frag, void* op)
{
  Z01_State* s;
  V01_ModemParams* dp;
  unsigned lo, hi;
  int t, ext, bit4, session;

  if (rate != SAMPLE_RATE_9600)
    return 0;
  if (LEVEL_ON())
    dsplibs_debug_printf("vpcm: create: dp %d, caller %d, frag %d (size %d).\n", id, caller, frag, 0xd258);
  if (frag > 0x30)
    return 0;

  /* session object: host `struct dp` header, then the protocol state */
  s = (Z01_State*)sysdep_malloc(sizeof(Z01_State));
  if (s == 0)
    return 0;
  sysdep_memset(s, 0, sizeof(Z01_State));
  s->dpData_ = s;
  s->txCnt_ = 4;
  s->modem_ = modem;
  s->id_ = id;
  s->op_ = op;
  s->status_ = 0;
  s->link_ = 0;
  s->trainCnt_ = 0;
  s->dspInfo_ = (struct dsp_info*)modem_get_param(modem, MDMPRM_DSPINFO);
  dp = (V01_ModemParams*)dp_param_get(modem);
  s->dp_ = dp;
  dp->path = 0; /* clears the whole parameter-file pointer */

  /* the VPcmFloModem and the K56flex handle; each failure releases what was created before */
  t = frag * 1000 / rate;
  s->v34_.ctx_ = VPCMXF_Create(0, &s->v34_, s->dp_, (unsigned)t, 0);
  if (s->v34_.ctx_ == 0)
  {
    sysdep_free(s);
    return 0;
  }
  s->v34_.k56_ = (K56FlexHandle*)K56FLEX_Create();
  if (s->v34_.k56_ == 0)
  {
    VPCMXF_Delete(s->v34_.ctx_);
    sysdep_free(s);
    return 0;
  }

  /* rate limits and the V.92 flag */
  lo = (unsigned)modem_get_param(modem, MDMPRM_MIN_RATE);
  hi = (unsigned)modem_get_param(modem, MDMPRM_MAX_RATE);
  if (hi > 0xdac0u)
    hi = 0xdac0u;
  if (LEVEL_ON())
    dsplibs_debug_printf("vpcm: VPCM rate limits: %d-%d\n", (int)lo, (int)hi);
  dp = s->dp_;
  dp->minRate = lo;
  dp->maxRate = hi;
  dp->upstreamMinRate = RATE_4800;
  dp->upstreamMaxRate = 0x8340;
  bit4 = 0;
  if (id == DP_V92)
    bit4 = (dp->flags2 >> 4) & 1;
  dp->flags2 = (uint8_t)((dp->flags2 & 0xef) | (bit4 << 4));
  s->dp_->flags2 &= 0xdf;
  s->silence_ = (((s->dp_->flags2 >> 4) & 1) == 0) ? 0x210 : 0;

  /* audio delay budget */
  t = (int)modem_get_param(modem, MDMPRM_IODELAY) + 4;
  if (0xf4 - t < 0)
  {
    int d = 0xf4 - t;
    int a;
    modem_set_param(modem, MDMPRM_UPDATE_DELAY, d);
    a = -d;
    if (a < 0x180)
      a = 0x180;
    s->addDelay_ = a;
    t = 0xf4;
  }
  dp = s->dp_;
  ext = t - 0x30 + s->addDelay_;
  dp->delay = t;
  dp->extDelay = ext;
  if (LEVEL_ON())
  { /* prints the stored delay_ and the extended delay just computed */
    dsplibs_debug_printf("vpcm: Delays: HW %d, DMA %d\n", s->dp_->delay, ext);
    dp = s->dp_;
  }
  dp->p2Silence = 0;

  /* start the V.34 object: session type 2 = V.92, 1 = V.90, 0 = V.34 */
  session = 2;
  if (id != DP_V92)
    session = (id == DP_V90);
  if (LEVEL_ON())
    dsplibs_debug_printf("vpcm: initial dp V.%d, session type %d.\n", id, session);
  if (VPcmV34Create(&s->v34_, caller == 0, frag, s->dp_, session) != 0)
  {
    VPCMXF_Delete(s->v34_.ctx_);
    K56FLEX_Delete(s->v34_.k56_);
    sysdep_free(s);
    return 0;
  }
  return s;
}

/**
 * @brief Destroy a data pump session (operations record `delete`).
 * @param w  the session (Z01_State)
 * @return 0
 * @note Publishes the register word (as the signed clock deviation) and the connection type to the host's `struct dsp_info`, terminates the V.90
 *       session, then deletes the modem, the K56flex handle and the session object.
 */
Z01_STATIC int vpcm_delete(Z01_State* w)
{
  Z01_State* s = w->dpData_;
  V01_ModemParams* dp;
  struct dsp_info* di;
  if (LEVEL_ON())
    dsplibs_debug_printf("vpcm: delete...\n");
  dp = s->dp_;
  di = s->dspInfo_;
  di->clock_deviation = (long)dp->clockDeviation; /* a signed `long` member: 8 bytes at x86-64, sign-extended */
  di->connection_type = dp->connectionType;
  VPCMXF_SessionTermination(s->v34_.ctx_);
  VPCMXF_Delete(s->v34_.ctx_);
  K56FLEX_Delete(s->v34_.k56_);
  sysdep_free(s);
  return 0;
}

/**
 * @brief Process one block of samples (operations record `process`).
 * @param w   the session
 * @param rx  `n` received 16-bit samples
 * @param tx  receives `n` transmit samples
 * @param n   number of samples in this call
 * @return link event: 1 = link came up, -1 = link error or training timeout, 0 otherwise
 * @note The protocol consumes a multiple of four samples per step, so the remainder is kept in `pendBuf_` and prepended to the next call; the transmit
 *       side is queued in `txBuf_` the same way.  While `silence_` is positive the block is transmitted as silence and the protocol is not run.
 * @warning `n` is trusted: `pendBuf_` and `txBuf_` hold 52 samples, so `n` plus the carry-over must stay within that (the host fragment size is limited
 *          to 0x30 by vpcm_create()).
 */
Z01_STATIC int vpcm_run(Z01_State* w, short* rx, short* tx, int n)
{
  Z01_State* s = w->dpData_;
  int ret = 0; /* link event handed back (1 up, -1 error, else 0) */
  int pend = s->pendCnt_;
  int total = pend + n;
  int e = (total < 0 ? total + 3 : total) & ~3; /* the samples handed to the V.34 object this call */

  /* prepend the carried-over samples */
  if (pend > 0)
  {
    sysdep_memcpy(&s->pendBuf_[s->pendCnt_], rx, (__SIZE_TYPE__)(n * 2));
    rx = (short*)s->pendBuf_;
  }
  if (e > 0)
  {
    uint16_t* txp = &s->txBuf_[s->txCnt_];
    int linkRes = 0;    /* link event detected in this call */
    int res = s->link_; /* link state after this call */
    if (s->silence_ > 0)
    {
      int r;
      sysdep_memset(txp, 0, (__SIZE_TYPE__)(e * 2));
      r = s->silence_ - e;
      if (r <= 0)
        r = 0;
      s->silence_ = r;
    }
    else
    {
      int cnt5 = 0;              /* output bits produced */
      int cnt7 = s->bitsWanted_; /* input bits wanted by the protocol */
      int status, tmp, i;
      uint8_t* bytesIn = (uint8_t*)s->bitsIn_;
      uint8_t* bytesOut = (uint8_t*)s->bitsOut_;
      void* cleaned;

      /* fetch the input bits the protocol asked for, widened to one per word */
      if (cnt7 > 0)
      {
        cnt7 = modem_get_bits(w->modem_, 1, bytesIn, cnt7);
        for (i = cnt7 - 1; i >= 0; i--)
          s->bitsIn_[i] = bytesIn[i];
      }

      /* run the protocol on the samples and log any cleaned samples it exposes */
      for (i = 0; i < e; i++)
        s->inF_[i] = (float)rx[i];
      status = VPcmV34Progress(&s->v34_, s->inF_, s->outF_, (unsigned)e, (int*)s->bitsOut_, &cnt5, (int*)s->bitsIn_, &cnt7);
      tmp = 0;
      cleaned = VPcmV34GetCleanedSamples(&s->v34_, &tmp);
      if (cleaned != 0 && tmp > 0)
        modem_debug_log_data(w->modem_, 3, cleaned, tmp * 2);
      for (i = 0; i < e; i++)
        txp[i] = (uint16_t)trunc16(s->outF_[i]);

      /* bit counts: clamp the request, deliver the produced bits to the host */
      if ((unsigned)cnt7 > 0x400u)
      {
        if (LEVEL_ON())
          dsplibs_debug_printf("vpcm: too many bits requested (%d)\n", cnt7);
        cnt7 = 0x400;
      }
      s->bitsWanted_ = cnt7;
      if (cnt5 > 0)
      {
        for (i = 0; i < cnt5; i++)
          bytesOut[i] = (uint8_t)(s->bitsOut_[i] & 1u);
        modem_put_bits(w->modem_, 1, bytesOut, cnt5);
      }

      /* link state machine, driven by changes of the protocol status */
      if (s->status_ == status)
      {
        if (status == 0)
        {
          s->trainCnt_++;
          if (s->trainCnt_ > 0xbb8)
          {
            if (LEVEL_ON())
              dsplibs_debug_printf("vpcm: train timeout!\n");
            res = -1;
          }
        }
      }
      else
      {
        if (LEVEL_ON())
          dsplibs_debug_printf("vpcm: New status %d\n", status);
        if ((unsigned)status <= 0x10u)
        {
          switch (status)
          {
          case 0:
            if (LEVEL_ON())
              dsplibs_debug_printf("vpcm: Re-starting phase II\n");
            s->trainCnt_ = 0;
            res = 0;
            if (s->dp_->p2Silence > 0 && s->addDelay_ != 0)
            {
              V01_ModemParams* dp;
              modem_set_param(w->modem_, MDMPRM_UPDATE_DELAY, -s->addDelay_);
              s->dp_->p2Silence = 0;
              if (LEVEL_ON())
              {
                dp = s->dp_;
                dsplibs_debug_printf("vpcm: P2 RESTART: decrease delay!! init %d, ext %d, add %d\n", dp->delay, s->dp_->extDelay, 0);
              }
            }
            break;
          case 1:
            if (LEVEL_ON())
              dsplibs_debug_printf("vpcm: Phase II completed !!!\n");
            if (s->addDelay_ != 0 && s->dp_->p2Silence == 0)
            {
              int add = s->addDelay_;
              modem_set_param(w->modem_, MDMPRM_UPDATE_DELAY, add);
              s->dp_->p2Silence = s->addDelay_;
              if (LEVEL_ON())
                dsplibs_debug_printf("vpcm: P2 FINISHED: increase delay!! init %d, ext %d, add %d\n", s->dp_->delay, s->dp_->extDelay, s->addDelay_);
            }
            break;
          case 4:
          case 5:
            res = 1;
            break;
          case 6:
          case 7:
            res = 0;
            break;
          case 8:
          case 9:
          case 16:
            res = -1;
            break;
          case 10:
            if (LEVEL_ON())
              dsplibs_debug_printf("vpcm: Same Line Verification Status\n");
            break;
          default:
            break;
          }
        }
      }

      s->status_ = status;
      if (s->link_ != res)
      {
        if (res == -1)
        {
          if (LEVEL_ON())
            dsplibs_debug_printf("vpcm: Link Error\n");
          linkRes = -1;
        }
        else if (res == 1)
        {
          int sess = VPcmV34GetCurrentSessionDP(&s->v34_);
          int rxr = VPcmV34GetCurrentRxBitRate(&s->v34_);
          int txr = VPcmV34GetCurrentTxBitRate(&s->v34_);
          if (LEVEL_ON())
            dsplibs_debug_printf("vpcm: Link: DP is V.%d, rate: rx %d, tx %d\n", sess, rxr, txr);
          if (sess == DP_V90)
            s->id_ = DP_V90;
          else if (sess == DP_V92)
            s->id_ = DP_V92;
          else
            s->id_ = DP_V34;
          modem_set_param(s->modem_, MDMPRM_TX_RATE, txr);
          modem_set_param(s->modem_, MDMPRM_RX_RATE, rxr);
          linkRes = 1;
        }
      }

      s->link_ = res;
      if (cnt7 == 0 && res == 1)
      {
        if (LEVEL_ON())
          dsplibs_debug_printf("vpcm: No symbols Requested!!!\n");
      }
      if (linkRes != 0)
        ret = linkRes;
    }
  }

  /* keep the unprocessed receive samples, deliver n transmit samples and keep the rest */
  if (s->pendCnt_ + n != e)
    sysdep_memcpy(s->pendBuf_, rx + e, (__SIZE_TYPE__)((s->pendCnt_ + n - e) * 2));
  s->pendCnt_ = s->pendCnt_ + n - e;
  sysdep_memcpy(tx, s->txBuf_, (__SIZE_TYPE__)(n * 2));
  sysdep_memcpy(s->txBuf_, s->txBuf_ + n, (__SIZE_TYPE__)((s->txCnt_ + e - n) * 2));
  s->txCnt_ = s->txCnt_ + e - n;
  return ret;
}

/* ---------------------------------------------------------------------------------------------------- registration */

/** Operations record registered for the three data pump ids: name "VPCM", no use count, create, delete, process, no hangup. */
Z01_STATIC Z01_DpOperations vpcm_op = {"VPCM", 0, (void*)vpcm_create, (void*)vpcm_delete, (void*)vpcm_run, 0};

/** @brief Register the VPCM data pump with the host for DP_V34, DP_V90 and DP_V92.  @return 0 */
extern "C" int dp_vpcm_init(void)
{
  modem_dp_register(DP_V34, &vpcm_op);
  modem_dp_register(DP_V90, &vpcm_op);
  modem_dp_register(DP_V92, &vpcm_op);
  return 0;
}

/** @brief Deregister the VPCM data pump for DP_V34, DP_V90 and DP_V92. */
extern "C" void dp_vpcm_exit(void)
{
  modem_dp_deregister(DP_V34, &vpcm_op);
  modem_dp_deregister(DP_V90, &vpcm_op);
  modem_dp_deregister(DP_V92, &vpcm_op);
}
