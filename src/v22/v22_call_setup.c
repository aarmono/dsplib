/**
 * @file v22_call_setup.c
 * @brief The V.22 / V.22bis / Bell 212A call-setup state machines: v22_originate(), v22_answer() and their shared tail states connect_2400() / connect_1200().
 *
 * - v22_originate(): protocol handler 1, the calling modem (stages V22OrgStage on tones.sc, 14-entry dispatch).
 * - v22_answer(): protocol handler 2, the answering modem (stages V22AnsStage, 15-entry dispatch).
 * - connect_2400() / connect_1200(): the tail states of both (tones.sc 8 .. 11 / 12 .. 13), also used by v22_local_loop; they train the receiver and
 *   report the connection.
 *
 * The handlers are called by V22FP_modem() (v22_engine.c) once per 160-sample block with the engine's static 16-bit buffers: txBits (tx data bits in),
 * out (160 audio samples out), in (160 samples in), rxBits (received bits out, at most 100), txCount / rxCount (in/out counts).  The return value is
 * ignored (always 0).  Each handler stores the engine state byte V22_MSG_HANDSHAKE first and then lets the transitions overwrite it.
 *
 * Originate, by stage (tones.sc): INIT (V22_status, silence, rates 1200, tone retuned to 1200 Hz); WAIT_ANSWER_TONE (Detect_v22) -> MEASURE_LEVEL;
 * MEASURE_LEVEL (level of the line: FPM_rms sum and average -> tones.shift; Detect_1s accumulated on the demodulated bits) -> DESCRAMBLE; DESCRAMBLE
 * (receive scrambled, 0x1a0 ms) -> SEND_S1; SEND_S1 (send S1 at 2400 or scrambled ones at 1200; look for S1 (MTDs1) and for unscrambled ones; 0x218 ms) ->
 * SEND_SCRAMBLED_ONES; SEND_SCRAMBLED_ONES (send scrambled ones; look for the same patterns) -> connect_2400 (V22CONN_2400_A) or connect_1200
 * (V22CONN_1200_A); stages 8 .. 11 connect_2400 (flags1e |= 8), 12 / 13 connect_1200.  Error states (engine state byte): 0x11 (timeout in
 * WAIT_ANSWER_TONE), 0x13 (MEASURE_LEVEL), 0x14 (SEND_SCRAMBLED_ONES).
 *
 * Answer, by stage: INIT (V22_status flag: with the answer-tone flag set go to ANSWER_TONE, else straight to UNSCRAMBLED_ONES with rates 1200 and the tone
 * retuned to 2225 Hz); ANSWER_TONE (FPM_TONE_generate, 0xce4 ms) -> SILENCE_AFTER_2100; SILENCE_AFTER_2100 (four blocks of silence) -> UNSCRAMBLED_ONES;
 * UNSCRAMBLED_ONES (unscrambled ones, a guard tone after 0x5be ms; look for S1 / ones) -> SEND_S1 (V.22bis, "Detected V22bis Carrier") or connect_1200
 * (V.22, "Detected V22 Carrier"); SEND_S1 (pattern 0, 0x61 ms) -> connect_2400; stages 8 .. 11 connect_2400, 12 / 13 connect_1200.
 *
 * The answer-tone stage runs only when the answer-tone flag (cfg.b11 bit 2, V22FP_create cfg[3]) is set; the data-pump glue passes 0, so with the glue
 * the answering modem starts at UNSCRAMBLED_ONES.
 */
#include <stdint.h>

#include "fpm/fpm.h"
#include "fpm/fpm_tables.h"
#include "v22.h"
#include "v22_tables.h"
#include "core/dsp_host.h"

static short iSilenceAfter2100; /* blocks of silence sent in the answer stage V22ANS_SILENCE_AFTER_2100; shared by all engine instances */

#define DBG1 (1 < dsplibs_debug_level)

#include "v22_us.h"
#include "core/dsp_constants.h" /* RATE_* */

/* The transmit side of the connect states: MakeTxData (test pattern), optionally ScrambleDataV22, then ModDataV22 into `out`; *txCount becomes the sample
 * count. */
static void tx_chain(V22Engine* eng, short* txBits, short* out, short* txCount, int pattern, int scramble)
{
  unsigned short n;
  MakeTxData(txBits, txCount, (short)pattern);
  n = (unsigned short)*txCount;
  if (scramble)
    ScrambleDataV22(eng, (unsigned short*)txBits, n);
  n = (unsigned short)*txCount;
  *txCount = (short)ModDataV22(eng, (const unsigned short*)txBits, out, n);
}
/* The receive side of the connect states: DemodDataV22, optionally DescrambleDataV22; *rxCount becomes the number of received symbols. */
static void rx_chain(V22Engine* eng, short* in, short* rxBits, short* rxCount, int descramble)
{
  unsigned short n = (unsigned short)*rxCount;
  unsigned int r = DemodDataV22(eng, in, rxBits, n);
  *rxCount = (short)r;
  if (descramble)
    DescrambleDataV22(eng, (unsigned short*)rxBits, (unsigned short)r);
}

/* Retune the tone generator: new frequency (Hz), the amplitude scaled by 0x5ae1 / 2^15 = 0.71 (about -3 dB per call), then the generator is re-created from
 * its own configuration words.  The words are copied into a nine-word block (the lpf word is the ToneLPF table word the engine's tone was created with), so the
 * same code is valid on 64-bit hosts where the FpmTone layout differs from the 32-bit image. */
static void retune_tone(FpmTone* tone, short freq, int clearS2)
{
  int amp = tone->r0.s[1];
  int tc[9];
  tone->r0.s[0] = freq;
  if (clearS2)
    tone->r0.s[2] = 0;
  tone->r0.s[1] = (short)((amp * 0x5ae1) >> 15);
  tc[0] = tone->r0.w[0];
  tc[1] = tone->r0.w[1];
  tc[2] = tone->r0.w[2];
  tc[3] = tone->r0.w[3];
  tc[4] = FPM_TONE_CFG_raw[4];
  tc[5] = tone->r1.w[0];
  tc[6] = tone->r1.w[1];
  tc[7] = tone->r1.w[2];
  tc[8] = tone->r1.w[3];
  FPM_TONE_create(tone, tc);
}

/* ============================================================================================ connect_2400 */

/**
 * @brief The 2400 bit/s tail of the call setup: train the receiver and supervise the carrier (tones.sc V22CONN_2400_A .. D).
 * @param eng      engine
 * @param txBits   transmit symbols (the handler generates its own pattern; 1 = ones, 2 = the 2400 data pattern)
 * @param out      receives 160 output samples
 * @param in       160 input samples
 * @param rxBits   receives the received symbols
 * @param txCount  transmit symbol count in; sample count out
 * @param rxCount  input sample count in; received symbol count out
 *
 * Stages: A sends scrambled ones, descrambles, and after 0x1c1 ms switches the receive rate to 2400 (-> B); B after 0x258 ms switches the transmit rate to
 * 2400 and sets the equaliser to slow adaptation (-> C); C sends the 2400 pattern, latches RxTrained2400 into tones.trained, and after 0x316 ms reports
 * V22_MSG_CONNECT_2400 if trained, else restarts the timer (-> D); D reports the connection as soon as RxTrained2400 holds, or V22_MSG_ERROR7 once
 * the handshake timeout is exceeded.  A connection sets flags1d |= 0x29.
 *
 * In every stage the carrier is supervised afterwards.  Carrier present with lossCnt != 0: a local retrain is started (protocol state V22PROTO_RETRAIN,
 * stage RESTART, state byte V22_MSG_LOCAL_RETRAIN).  Carrier absent: lossCnt counts blocks; once (short)(lossCnt * 20) exceeds cfg.lossMs (ms) the state
 * byte becomes V22_MSG_NO_CARRIER and the received bits are clamped to ones.
 */
void connect_2400(V22Engine* eng, short* txBits, short* out, short* in, short* rxBits, short* txCount, short* rxCount)
{
  V22Tones* tones = eng->tones;
  int sc;
  eng->state = V22_MSG_HANDSHAKE;
  sc = tones->sc;
  switch (sc)
  {
  case V22CONN_2400_A:
    tx_chain(eng, txBits, out, txCount, 1, 1);
    rx_chain(eng, in, rxBits, rxCount, 1);
    RxClampV22(eng, in, rxBits, rxCount);
    if ((unsigned int)ReadGTimer(eng) > 0x1c1u)
    {
      if (DBG1)
        dsplibs_debug_printf("connect_2400, NODE_2400B\n");
      tones->sc = V22CONN_2400_B;
      SetRxRate(eng, 1);
    }
    break;
  case V22CONN_2400_B:
    tx_chain(eng, txBits, out, txCount, 1, 1);
    rx_chain(eng, in, rxBits, rxCount, 1);
    RxClampV22(eng, in, rxBits, rxCount);
    if ((unsigned int)ReadGTimer(eng) > 0x258u)
    {
      if (DBG1)
        dsplibs_debug_printf("connect_2400, NODE_2400C\n");
      tones->sc = V22CONN_2400_C;
      SetTxRate(eng, 1);
      SetAdaptEqV22(eng, 3);
    }
    break;
  case V22CONN_2400_C:
    tx_chain(eng, txBits, out, txCount, 2, 1);
    rx_chain(eng, in, rxBits, rxCount, 1);
    if (tones->trained == 0)
      tones->trained = (int)RxTrained2400(rxBits, (const unsigned short*)rxCount);
    RxClampV22(eng, in, rxBits, rxCount);
    if ((unsigned int)ReadGTimer(eng) > 0x316u)
    {
      if (tones->trained == 1)
      {
        eng->flags1d |= 0x29;
        eng->state = V22_MSG_CONNECT_2400;
        if (DBG1)
          dsplibs_debug_printf("V22_MSG_CONNECT_2400 In NODE_2400C\n");
      }
      else
      {
        tones->timer = 0;
        tones->sc = V22CONN_2400_D;
      }
    }
    break;
  case V22CONN_2400_D:
    tx_chain(eng, txBits, out, txCount, 2, 1);
    rx_chain(eng, in, rxBits, rxCount, 1);
    if (RxTrained2400(rxBits, (const unsigned short*)rxCount) != 0)
    {
      eng->flags1d |= 0x29;
      eng->state = V22_MSG_CONNECT_2400;
      if (DBG1)
        dsplibs_debug_printf("V22_MSG_CONNECT_2400 In NODE_2400D\n");
    }
    RxClampV22(eng, in, rxBits, rxCount);
    {
      unsigned int t = (unsigned int)ReadGTimer(eng);
      if (t > (unsigned int)tones->timeout)
      {
        eng->flags1d |= 2;
        eng->state = V22_MSG_ERROR7;
        if (DBG1)
          dsplibs_debug_printf("V22_MSG_ERROR7\n");
      }
    }
    break;
  default:
    break;
  }

  /* the carrier supervision shared by every stage */
  if (CarrierDetect(eng) != 0)
  {
    if (tones->lossCnt != 0)
    { /* carrier back inside the loss time: retrain */
      tones->state = V22PROTO_RETRAIN;
      tones->sc = V22RETRAIN_RESTART;
      SetAdaptEqV22(eng, 1);
      tones->retrainLocal = 1;
      eng->state = V22_MSG_LOCAL_RETRAIN;
      tones->lossCnt = 0;
      if (DBG1)
        dsplibs_debug_printf("Carrier back during carrier_loss_time (Connect_2400). Retrain initiated.");
    }
    return;
  }
  {
    int loss = (short)eng->cfg.lossMs; /* read before the counter is stored (the code does not depend on the aliasing of the int store) */
    int lim;
    tones->lossCnt = (int)((unsigned int)tones->lossCnt + 1u);
    lim = (short)((unsigned int)tones->lossCnt * 20u);
    if (loss >= lim)
    {
      if (DBG1)
        dsplibs_debug_printf("V22_MSG_NO_CARRIER won't be reported (carrier_loss_time %d of %d ms)", lim, loss);
      return;
    }
  }

  /* carrier lost for longer than the loss time */
  eng->state = V22_MSG_NO_CARRIER;
  if (DBG1)
    dsplibs_debug_printf("V22_MSG_NO_CARRIER3\n");
  RxClampV22(eng, in, rxBits, rxCount);
}

/* ============================================================================================ connect_1200 */

/**
 * @brief The 1200 bit/s tail of the call setup: train the receiver and supervise the carrier (tones.sc V22CONN_1200_A, B).
 * @param eng      engine
 * @param txBits   transmit symbols (the handler generates its own pattern)
 * @param out      receives 160 output samples
 * @param in       160 input samples
 * @param rxBits   receives the received symbols
 * @param txCount  transmit symbol count in; sample count out
 * @param rxCount  input sample count in; received symbol count out
 *
 * Stage A sends scrambled ones, latches RxTrained1200 into tones.trained, and after 0x2f3 ms reports V22_MSG_CONNECT_1200 if trained (with the slow
 * adaptation) or restarts the timer (-> B).  Stage B reports the connection as soon as RxTrained1200 holds, or V22_MSG_ERROR8 once the handshake
 * timeout is exceeded.  The carrier supervision is as in connect_2400() except that a returning carrier never starts a retrain at 1200 bit/s.
 */
void connect_1200(V22Engine* eng, short* txBits, short* out, short* in, short* rxBits, short* txCount, short* rxCount)
{
  V22Tones* tones = eng->tones;
  int sc;
  eng->state = V22_MSG_HANDSHAKE;
  sc = tones->sc;
  switch (sc)
  {
  case V22CONN_1200_A:
    tx_chain(eng, txBits, out, txCount, 1, 1);
    rx_chain(eng, in, rxBits, rxCount, 1);
    if (tones->trained == 0)
      tones->trained = (int)RxTrained1200(rxBits, (const unsigned short*)rxCount);
    RxClampV22(eng, in, rxBits, rxCount);
    if ((unsigned int)ReadGTimer(eng) > 0x2f3u)
    {
      if (tones->trained == 1)
      {
        SetAdaptEqV22(eng, 3);
        eng->flags1d |= 0x29;
        eng->state = V22_MSG_CONNECT_1200;
      }
      else
      {
        tones->timer = 0;
        tones->sc = V22CONN_1200_B;
      }
    }
    break;
  case V22CONN_1200_B:
    tx_chain(eng, txBits, out, txCount, 1, 1);
    rx_chain(eng, in, rxBits, rxCount, 1);
    if (RxTrained1200(rxBits, (const unsigned short*)rxCount) != 0)
    {
      RxClampV22(eng, in, rxBits, rxCount);
      eng->flags1d |= 0x29;
      eng->state = V22_MSG_CONNECT_1200;
    }
    RxClampV22(eng, in, rxBits, rxCount);
    {
      unsigned int t = (unsigned int)ReadGTimer(eng);
      if (t > (unsigned int)tones->timeout)
      {
        eng->flags1d |= 2;
        eng->state = V22_MSG_ERROR8;
      }
    }
    break;
  default:
    break;
  }

  /* carrier supervision */
  if (CarrierDetect(eng) != 0)
    return; /* no retrain at 1200 */
  {
    int loss = (short)eng->cfg.lossMs; /* read before the counter is stored (the code does not depend on the aliasing of the int store) */
    int lim;
    tones->lossCnt = (int)((unsigned int)tones->lossCnt + 1u);
    lim = (short)((unsigned int)tones->lossCnt * 20u);
    if (loss >= lim)
    {
      if (DBG1)
        dsplibs_debug_printf("V22_MSG_NO_CARRIER won't be reported (carrier_loss_time %d of %d ms)", lim, loss);
      return;
    }
  }

  /* carrier lost for longer than the loss time */
  eng->state = V22_MSG_NO_CARRIER;
  if (DBG1)
    dsplibs_debug_printf("V22_MSG_NO_CARRIER4\n");
  RxClampV22(eng, in, rxBits, rxCount);
}

/* ============================================================================================ v22_originate */

/**
 * @brief Protocol handler 1: the calling modem's call setup (see the file header for the stages).
 * @param eng      engine
 * @param txBits   transmit symbols (generated here, the host's bits are not used during the handshake)
 * @param out      receives 160 output samples
 * @param in       160 input samples
 * @param rxBits   receives the received symbols
 * @param txCount  transmit symbol count in; sample count out
 * @param rxCount  input sample count in; received symbol count out
 * @return 0 (ignored by the caller)
 *
 * Stage-specific timeouts (ms): DESCRAMBLE 0x1a0, SEND_S1 0x218, SEND_SCRAMBLED_ONES 0x786 (V22_MSG_ERROR4); the other stages use the handshake timeout
 * tones.timeout.  The transition into the 2400 states needs sigAcc > 0x3b (the S1 detector fired for several blocks) and the 1200 states
 * onesAcc > 0xe6 (unscrambled ones detected).  A stage value above 13 does nothing.
 * @warning The level measurement of MEASURE_LEVEL uses FPM_rms() over 160 samples, which overflows for a nearly full-scale input (docs/bugs.md, section I).
 */
int v22_originate(V22Engine* eng, short* txBits, short* out, short* in, short* rxBits, short* txCount, short* rxCount)
{
  V22Tones* tones = eng->tones;
  V22Core* core = eng->core;
  unsigned int sw;
  unsigned short var2c = 0; /* 0x14 when the S1 detector reported nothing during the signal-detected block */
  short n;

  eng->state = V22_MSG_HANDSHAKE;
  sw = (unsigned int)(int)tones->sc;
  if (sw > 13u)
    return 0;

  switch (sw)
  {
  case V22ORG_INIT:
  {
    V22Status st = {0};
    if (DBG1)
      dsplibs_debug_printf("V22_originate,NODE_0\n");
    tones->timer = 0;
    tones->sigAcc = 0;
    tones->onesAcc = 0;
    tones->sc = V22ORG_WAIT_ANSWER_TONE;
    V22_status(eng, &st);
    TxNOP(eng, txBits, out, txCount);
    *rxCount = 0;
    RxClampV22(eng, in, rxBits, rxCount);
    SetTxRate(eng, 0);
    SetRxRate(eng, 0);
    retune_tone(tones->tone, 0x4b0, 1);
    return 0;
  }
  case V22ORG_WAIT_ANSWER_TONE:
    if (DBG1)
      dsplibs_debug_printf("V22_originate, NODE_1\n");
    TxNOP(eng, txBits, out, txCount);
    if (Detect_v22(eng, in) != 0)
    {
      unsigned short cnt = (unsigned short)*rxCount;
      (void)DemodDataV22(eng, in, rxBits, cnt); /* run once to start the receive chain; its result is not used (*rxCount is cleared below) */
      tones->timer = 0;
      tones->sc = V22ORG_MEASURE_LEVEL;
      eng->flags1d |= 0x10;
    }
    *rxCount = 0;
    RxClampV22(eng, in, rxBits, rxCount);
    if ((unsigned int)ReadGTimer(eng) > (unsigned int)tones->timeout)
    {
      eng->flags1d |= 2;
      eng->state = V22_MSG_ERROR1;
      if (DBG1)
        dsplibs_debug_printf("V22_MSG_ERROR1\n");
    }
    return 0;
  case 2:
  case 7:
    return 0;
  case V22ORG_MEASURE_LEVEL:
    if (DBG1)
    {
      dsplibs_debug_printf("V22_originate, NODE_3\n");
    }

    /* measure the line level of this block, accumulate it and the ones detector over the received symbols */
    tones->rms = FPM_rms(in, 0xa0);
    TxNOP(eng, txBits, out, txCount);
    *rxCount = (short)DemodDataV22(eng, in, rxBits, (unsigned short)*rxCount);
    if (*rxCount != 0)
    {
      unsigned int r = Detect_1s((const unsigned short*)rxBits, (const unsigned short*)rxCount, RATE_1200, 0x6ccc);
      unsigned int rms, acc, cnt;
      rms = us(tones->rms);
      acc = us(tones->sigAcc);
      cnt = us(tones->rmsN);
      tones->rmsSum = (int)((unsigned int)tones->rmsSum + rms);
      tones->sigAcc = (short)(r + acc);
      tones->rmsN = (short)(cnt + 1u);
    }

    /* enough ones seen: the average level decides the input scaling (tones.shift = 1 above rmsThr), then continue */
    if (us(tones->sigAcc) > 0x87u)
    {
      unsigned int cnt = us(tones->rmsN);
      unsigned int sum = (unsigned int)tones->rmsSum;
      unsigned int q = cnt ? sum / cnt : 0u; /* guarded: rmsN is never 0 here, because it is incremented with every accumulation */
      tones->timer = 0;
      tones->sigAcc = 0;
      tones->sc = V22ORG_DESCRAMBLE;
      tones->rmsSum = (int)q;
      if (q > us(tones->rmsThr))
        tones->shift = 1;
    }
    RxClampV22(eng, in, rxBits, rxCount);
    if ((unsigned int)ReadGTimer(eng) > (unsigned int)tones->timeout)
    {
      eng->flags1d |= 2;
      eng->state = V22_MSG_ERROR3;
      if (DBG1)
        dsplibs_debug_printf("V22_MSG_ERROR3\n");
    }
    return 0;
  case V22ORG_DESCRAMBLE:
    if (DBG1)
      dsplibs_debug_printf("V22_originate, NODE_4\n");
    TxNOP(eng, txBits, out, txCount);
    {
      unsigned int r = DemodDataV22(eng, in, rxBits, (unsigned short)*rxCount);
      *rxCount = (short)r;
      DescrambleDataV22(eng, (unsigned short*)rxBits, (unsigned short)r);
    }
    if ((unsigned int)ReadGTimer(eng) > 0x1a0u)
    {
      tones->sc = V22ORG_SEND_S1;
      SetAdaptEqV22(eng, 1);
    }
    RxClampV22(eng, in, rxBits, rxCount);
    return 0;
  case V22ORG_SEND_S1:
    if (DBG1)
      dsplibs_debug_printf("V22_originate, NODE_5\n");

    /* transmit S1 (pattern 0 at 2400) or scrambled ones (pattern 1 at 1200) */
    MakeTxData(txBits, txCount, (short)(eng->cfg.rate2 == RATE_1200));
    if (eng->cfg.rate2 == RATE_1200)
      ScrambleDataV22(eng, (unsigned short*)txBits, (unsigned short)*txCount);
    *txCount = (short)ModDataV22(eng, (const unsigned short*)txBits, out, (unsigned short)*txCount);

    /* receive: the S1 detector (a result of 0 means the tone is present) raises sigAcc, the ones detector raises onesAcc */
    *rxCount = (short)DemodDataV22(eng, in, rxBits, (unsigned short)*rxCount);
    n = (short)FPM_MTD_detect(tones->mtdS1, core->rxBuf, core->rxCount);
    if (*rxCount != 0 && SignalDetect(eng) == 1)
    {
      if (n == 0)
        tones->sigAcc = (short)(us(tones->sigAcc) + 0x14u);
      else if (us(tones->sigAcc) <= 0x3bu)
        tones->sigAcc = 0;
      if ((unsigned short)Detect_1s((const unsigned short*)rxBits, (const unsigned short*)rxCount, RATE_1200, 0x6ccc) == 0)
      {
        unsigned int r;
        DescrambleDataV22(eng, (unsigned short*)rxBits, (unsigned short)*rxCount);
        r = Detect_1s((const unsigned short*)rxBits, (const unsigned short*)rxCount, RATE_1200, 0x6ccc);
        tones->onesAcc = (short)(r + us(tones->onesAcc));
      }
    }

    if ((unsigned int)ReadGTimer(eng) > 0x218u)
      tones->sc = V22ORG_SEND_SCRAMBLED_ONES;
    RxClampV22(eng, in, rxBits, rxCount);
    return 0;
  case V22ORG_SEND_SCRAMBLED_ONES:
    if (DBG1)
      dsplibs_debug_printf("V22_originate, NODE_6\n");

    /* transmit scrambled ones */
    MakeTxData(txBits, txCount, 1);
    ScrambleDataV22(eng, (unsigned short*)txBits, (unsigned short)*txCount);
    *txCount = (short)ModDataV22(eng, (const unsigned short*)txBits, out, (unsigned short)*txCount);

    /* receive: sigAcc counts the blocks with S1 present (var2c is non-zero in a block that still shows S1), onesAcc the unscrambled ones */
    *rxCount = (short)DemodDataV22(eng, in, rxBits, (unsigned short)*rxCount);
    n = (short)FPM_MTD_detect(tones->mtdS1, core->rxBuf, core->rxCount);
    if (*rxCount != 0 && SignalDetect(eng) == 1)
    {
      var2c = 0;
      if (n != 0)
      {
        if (us(tones->sigAcc) <= 0x3bu)
          tones->sigAcc = 0;
      }
      else
      {
        var2c = 0x14;
        tones->sigAcc = (short)(us(tones->sigAcc) + 0x14u);
      }
      if ((unsigned short)Detect_1s((const unsigned short*)rxBits, (const unsigned short*)rxCount, RATE_1200, 0x6ccc) == 0)
      {
        unsigned int r;
        DescrambleDataV22(eng, (unsigned short*)rxBits, (unsigned short)*rxCount);
        r = Detect_1s((const unsigned short*)rxBits, (const unsigned short*)rxCount, RATE_1200, 0x6ccc);
        tones->onesAcc = (short)(r + us(tones->onesAcc));
      }
    }
    RxClampV22(eng, in, rxBits, rxCount);

    /* decide the rate */
    if (us(tones->sigAcc) > 0x3bu && var2c == 0)
    { /* S1 was present for several blocks and is gone in this one: go to the 2400 states */
      tones->timer = 0;
      tones->sigAcc = 0;
      tones->onesAcc = 0;
      tones->sc = V22CONN_2400_A;
    }
    else if (us(tones->onesAcc) > 0xe6u)
    { /* unscrambled ones: 1200 bit/s */
      if (DBG1)
        dsplibs_debug_printf("V.22 %d modem det true\n", 0x280);
      eng->flags1e |= 8;
      tones->timer = 0;
      tones->sigAcc = 0;
      tones->onesAcc = 0;
      tones->sc = V22CONN_1200_A;
    }
    else
    {
      if ((unsigned int)ReadGTimer(eng) > 0x786u)
      {
        eng->flags1d |= 2;
        eng->state = V22_MSG_ERROR4;
        if (DBG1)
          dsplibs_debug_printf("V22_MSG_ERROR4\n");
      }
      return 0;
    }
    SetAdaptEqV22(eng, 2);
    FPM_AGC_Freeze(&core->agcIn);
    return 0;
  case V22CONN_2400_A:
  case V22CONN_2400_B:
  case V22CONN_2400_C:
  case V22CONN_2400_D:
    if (DBG1)
      dsplibs_debug_printf("V22_originate, NODE %d\n ", (int)tones->sc);
    eng->flags1e |= 8;
    connect_2400(eng, txBits, out, in, rxBits, txCount, rxCount);
    return 0;
  default: /* 12, 13 */
    if (DBG1)
      dsplibs_debug_printf("V22_originate, NODE %d\n ", (int)tones->sc);
    connect_1200(eng, txBits, out, in, rxBits, txCount, rxCount);
    return 0;
  }
}

/* ============================================================================================ v22_answer */

/**
 * @brief Protocol handler 2: the answering modem's call setup (see the file header for the stages).
 * @param eng      engine
 * @param txBits   transmit symbols (generated here)
 * @param out      receives 160 output samples
 * @param in       160 input samples
 * @param rxBits   receives the received symbols
 * @param txCount  transmit symbol count in; sample count out
 * @param rxCount  input sample count in; received symbol count out
 * @return 0 (ignored by the caller)
 *
 * Timeouts (ms): ANSWER_TONE 0xce4, SEND_S1 0x61, guard tone from 0x5be in UNSCRAMBLED_ONES; that stage gives up with V22_MSG_ERROR5 at the handshake
 * timeout.  The V.22bis branch (S1 seen) is taken only at cfg.rate2 == 2400, otherwise unscrambled ones lead to the 1200 states.  A stage value above 14
 * does nothing.
 */
int v22_answer(V22Engine* eng, short* txBits, short* out, short* in, short* rxBits, short* txCount, short* rxCount)
{
  V22Tones* tones = eng->tones;
  V22Core* core = eng->core;
  unsigned int sw;
  unsigned short var2c = 0;
  short n;

  eng->state = V22_MSG_HANDSHAKE;
  sw = (unsigned int)(int)tones->sc;
  if (sw > 14u)
    return 0;
  switch (sw)
  {
  case V22ANS_INIT:
  {
    V22Status st = {0};
    if (DBG1)
    {
      dsplibs_debug_printf("V22_answer,NODE_0\n");
    }
    tones->timer = 0;
    tones->sigAcc = 0;
    tones->onesAcc = 0;
    V22_status(eng, &st);
    *rxCount = 0;
    RxClampV22(eng, in, rxBits, rxCount);
    if (st.flags2 & 1)
    {
      TxNOP(eng, txBits, out, txCount);
      tones->sc = V22ANS_ANSWER_TONE;
      return 0;
    }
    tones->sc = V22ANS_UNSCRAMBLED_ONES;
    SetTxRate(eng, 0);
    SetRxRate(eng, 0);
    MakeTxData(txBits, txCount, 1);
    *txCount = (short)ModDataV22(eng, (const unsigned short*)txBits, out, (unsigned short)*txCount);
    tones->sc = V22ANS_UNSCRAMBLED_ONES;
    retune_tone(tones->tone, 0x8b1, 0);
    return 0;
  }
  case V22ANS_ANSWER_TONE:
    if (DBG1)
    {
      dsplibs_debug_printf("V22_answer,NODE_1\n");
    }
    *txCount = (short)FPM_TONE_generate(tones->tone, out, 0xa0);
    *rxCount = 0;
    RxClampV22(eng, in, rxBits, rxCount);
    if ((unsigned int)ReadGTimer(eng) > 0xce4u)
    {
      tones->sc = V22ANS_SILENCE_AFTER_2100;
      tones->timer = 0;
      iSilenceAfter2100 = 0;
      eng->flags1d |= 0x10;
      SetTxRate(eng, 0);
      SetRxRate(eng, 0);
      retune_tone(tones->tone, 0x8b1, 0);
    }
    return 0;
  case 2:
  case 5:
  case 6:
  case 7:
    return 0;
  case V22ANS_SILENCE_AFTER_2100:
    if (DBG1)
      dsplibs_debug_printf("V22_answer,NODE_SILENCE_AFTER_2100\n");
    iSilenceAfter2100 = (short)(iSilenceAfter2100 + 1);
    if (iSilenceAfter2100 == 4)
      tones->sc = V22ANS_UNSCRAMBLED_ONES;
    TxNOP(eng, txBits, out, txCount);
    return 0;
  case V22ANS_UNSCRAMBLED_ONES:
    if (DBG1)
      dsplibs_debug_printf("V22_answer,NODE_3\n");

    /* transmit unscrambled ones; after 0x5be ms the output is replaced by the tone generator's tone */
    MakeTxData(txBits, txCount, 1);
    {
      unsigned int nm = ModDataV22(eng, (const unsigned short*)txBits, out, (unsigned short)*txCount);
      unsigned int tm = (unsigned int)tones->timer; /* read before the count is stored (the code does not depend on the aliasing of that store) */
      *txCount = (short)nm;
      if (tm > 0x5beu)
        *txCount = (short)FPM_TONE_generate(tones->tone, out, 0xa0);
    }

    /* receive: sigAcc counts the blocks with S1 present, onesAcc the unscrambled ones (reset when not confirmed) */
    *rxCount = (short)DemodDataV22(eng, in, rxBits, (unsigned short)*rxCount);
    n = (short)FPM_MTD_detect(tones->mtdS1, core->rxBuf, core->rxCount);
    if (*rxCount != 0 && SignalDetect(eng) == 1)
    {
      var2c = 0;
      if (n != 0)
      {
        if (us(tones->sigAcc) <= 0x3bu)
          tones->sigAcc = 0;
      }
      else
      {
        var2c = 0x14;
        tones->sigAcc = (short)(us(tones->sigAcc) + 0x14u);
      }
      if ((unsigned short)Detect_1s((const unsigned short*)rxBits, (const unsigned short*)rxCount, RATE_1200, 0x6ccc) == 0)
      {
        int r;
        unsigned int sum, acc;
        DescrambleDataV22(eng, (unsigned short*)rxBits, (unsigned short)*rxCount);
        r = (short)Detect_1s((const unsigned short*)rxBits, (const unsigned short*)rxCount, RATE_1200, 0x6ccc);
        acc = us(tones->onesAcc);
        sum = (unsigned int)r + acc;
        if (r != 0 || (unsigned short)sum > 0x3bu)
          tones->onesAcc = (short)sum;
        else
          tones->onesAcc = 0;
      }
    }
    RxClampV22(eng, in, rxBits, rxCount);

    /* decide the rate */
    if (us(tones->sigAcc) > 0x3bu && var2c == 0 && eng->cfg.rate2 == RATE_2400)
    { /* V.22bis: S1 was present for several blocks and is gone in this one */
      if (DBG1)
        dsplibs_debug_printf("Detected V22bis Carrier\n");
      tones->timer = 0;
      tones->sigAcc = 0;
      tones->onesAcc = 0;
      tones->sc = V22ANS_SEND_S1;
    }
    else if (us(tones->onesAcc) > 0xe6u)
    { /* V.22: unscrambled ones */
      if (DBG1)
      {
        dsplibs_debug_printf("Detected V22 Carrier\n");
      }
      tones->timer = 0;
      tones->sigAcc = 0;
      tones->onesAcc = 0;
      tones->sc = V22CONN_1200_A;
    }
    else
    {
      if ((unsigned int)ReadGTimer(eng) > (unsigned int)tones->timeout)
      {
        eng->flags1d |= 2;
        eng->state = V22_MSG_ERROR5;
        if (DBG1)
          dsplibs_debug_printf("V22_MSG_ERROR5\n");
      }
      return 0;
    }
    SetAdaptEqV22(eng, 2);
    eng->flags1e |= 8;
    FPM_AGC_Freeze(&core->agcIn);
    return 0;
  case V22ANS_SEND_S1:
    if (DBG1)
      dsplibs_debug_printf("V22_answer,NODE_4\n");
    MakeTxData(txBits, txCount, 0);
    *txCount = (short)ModDataV22(eng, (const unsigned short*)txBits, out, (unsigned short)*txCount);
    {
      unsigned int r = DemodDataV22(eng, in, rxBits, (unsigned short)*rxCount);
      *rxCount = (short)r;
      DescrambleDataV22(eng, (unsigned short*)rxBits, (unsigned short)r);
    }
    RxClampV22(eng, in, rxBits, rxCount);
    if ((unsigned int)ReadGTimer(eng) > 0x61u)
      tones->sc = V22CONN_2400_A;
    return 0;
  case V22CONN_2400_A:
  case V22CONN_2400_B:
  case V22CONN_2400_C:
  case V22CONN_2400_D:
    if (DBG1)
      dsplibs_debug_printf("V22_answer, NODE %d\n ", (int)tones->sc);
    connect_2400(eng, txBits, out, in, rxBits, txCount, rxCount);
    return 0;
  default: /* 12, 13 */
    if (DBG1)
      dsplibs_debug_printf("V22_answer, NODE %d\n ", (int)tones->sc);
    connect_1200(eng, txBits, out, in, rxBits, txCount, rxCount);
    return 0;
  }
}
