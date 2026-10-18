/**
 * @file v22_loopback.c
 * @brief The V.22 / V.22bis loop-back protocol states.
 *
 * - v22_local_loop(): protocol handler 3, the analogue loop (tones.state V22PROTO_LOCAL_LOOP; stages V22LoopStage), the call setup against the modem's own
 *   transmitted signal; it ends in connect_2400() / connect_1200().
 * - v22_org_rmloop2(): protocol handler 4, the originating side of the remote digital loopback (stages V22OrgRmloopStage).
 * - v22_ans_rmloop2(): protocol handler 5, the answering side (stages V22AnsRmloopStage).
 *
 * They are called only through V22_PROTOCOL by V22FP_modem() once per 160-sample block (seven arguments as v22_data); the return value is ignored (0).
 * The engine state byte: local loop V22_MSG_HANDSHAKE (the connect_* functions write 3 / 4), org_rmloop2 V22_MSG_HANDSHAKE then 6 / 7, ans_rmloop2
 * V22_MSG_ANS_LOOP_RUNNING then 5 / 7 / 8.  All three set flags1e |= 2.
 *
 * V22_MESG is not a message table: it is 25 single-precision constants (a tapering, symmetric response) that nothing in this tree references.
 */
#include <stdint.h>

#include "fpm/fpm.h"
#include "v22.h"

/* V22_MESG: 25 IEEE-754 single-precision words, unreferenced in this tree (see the file header) */
const unsigned int V22_MESG[25] = {0x3d285bd2u, 0x3d2a9adeu, 0x3d2b5bdcu, 0x3d2a9adeu, 0x3d285bd2u, 0x3d24aa51u, 0x3d1f9955u, 0x3d1942b7u, 0x3d11c67cu,
                                   0x3d0949fbu, 0x3cffedacu, 0x3cebf3dau, 0x3cd70459u, 0x3cc17ec6u, 0x3cabc20bu, 0x3c962a1eu, 0x3c810dccu, 0x3c5979b9u,
                                   0x3c32fc8cu, 0x3c0f1db9u, 0x3bdc80f7u, 0x3ba1608bu, 0x3b5a807fu, 0x3b00aaedu, 0x3a553413u};

#include "v22_us.h"
#include "core/dsp_constants.h" /* RATE_* */

/* ============================================================================================ v22_local_loop */

/**
 * @brief Protocol handler 3: the analogue loop-back call setup, using the tone generator and the S1 / ones detectors of the call setup against the
 *        modem's own signal.
 * @param eng      engine
 * @param txBits   transmit symbols (generated here)
 * @param out      receives 160 output samples
 * @param in       160 input samples
 * @param rxBits   receives the received symbols
 * @param txCount  transmit symbol count in; sample count out
 * @param rxCount  input sample count in; received symbol count out
 * @return 0 (ignored by the caller)
 *
 * Stages (tones.sc): INIT clears the timer and accumulators, runs V22_status and sends silence; with the answer-tone flag (V22_status flags2 bit 0) it goes
 * to ANSWER_TONE, else to DETECT_RATE with flags1d |= 0x10 and rates 1200.  ANSWER_TONE sends the tone (FPM_TONE_generate) for 0xce4 ms, then
 * WAIT_TONE_END (flags1d |= 0x10, rates 1200).  WAIT_TONE_END sends silence until FPM_TONE_detect reports "too little signal" (2).  DETECT_RATE sends
 * scrambled ones at 1200, or S1 (pattern 0) at 2400 and ones (pattern 1, unscrambled) once sigAcc > 0x61, and accumulates the S1 detector in sigAcc (0x14
 * per block with S1 present, cleared at <= 0x3b when it is absent) and the ones detector in onesAcc: sigAcc > 0x61 -> connect_2400 (V22CONN_2400_A), else
 * onesAcc > 0xe6 -> connect_1200 (V22CONN_1200_A), both with equaliser adaptation 2; the handshake timeout gives flags1d |= 2 and V22_MSG_ERROR6.  Stages
 * 4 .. 7 do nothing; 8 .. 11 run connect_2400(), 12 / 13 connect_1200(); a stage above 13 returns.
 */
int v22_local_loop(V22Engine* eng, short* txBits, short* out, short* in, short* rxBits, short* txCount, short* rxCount)
{
  V22Tones* tones = eng->tones;
  V22Core* core = eng->core;
  /* the engine pointers and rate2 are read once: the code must not depend on how the compiler orders these loads against the short stores through
   * rxCount / txCount */
  const short rate2 = eng->cfg.rate2;
  unsigned int sw;

  eng->flags1e |= 2;
  eng->state = V22_MSG_HANDSHAKE;
  sw = (unsigned int)(int)tones->sc;
  if (sw > 13u)
    return 0;
  switch (sw)
  {
  case V22LOOP_INIT:
  {
    V22Status st = {0};
    tones->timer = 0;
    tones->sigAcc = 0;
    tones->onesAcc = 0;
    V22_status(eng, &st);
    TxNOP(eng, txBits, out, txCount);
    *rxCount = 0;
    RxClampV22(eng, in, rxBits, rxCount);
    if (st.flags2 & 1)
    {
      tones->sc = V22LOOP_ANSWER_TONE;
      return 0;
    }
    tones->sc = V22LOOP_DETECT_RATE;
    eng->flags1d |= 0x10;
    SetTxRate(eng, 0);
    SetRxRate(eng, 0);
    return 0;
  }
  case V22LOOP_ANSWER_TONE:
    *txCount = (short)FPM_TONE_generate(tones->tone, out, 0xa0);
    *rxCount = 0;
    RxClampV22(eng, in, rxBits, rxCount);
    if ((unsigned int)ReadGTimer(eng) > 0xce4u)
    {
      tones->timer = 0;
      tones->sc = V22LOOP_WAIT_TONE_END;
      eng->flags1d |= 0x10;
      SetTxRate(eng, 0);
      SetRxRate(eng, 0);
    }
    return 0;
  case V22LOOP_WAIT_TONE_END:
    TxNOP(eng, txBits, out, txCount);
    *rxCount = 0;
    RxClampV22(eng, in, rxBits, rxCount);
    if ((short)FPM_TONE_detect(tones->tone, in, 0xa0) == 2)
    {
      tones->timer = 0;
      tones->sc = V22LOOP_DETECT_RATE;
    }
    return 0;
  case V22LOOP_DETECT_RATE:
  {
    short n;
    int pattern = (rate2 == RATE_1200);

    /* transmit: S1 at 2400 (switching to ones once S1 has been seen), scrambled ones at 1200 */
    if (us(tones->sigAcc) > 0x61u)
      pattern = 1;
    MakeTxData(txBits, txCount, (short)pattern);
    if (rate2 == RATE_1200)
      ScrambleDataV22(eng, (unsigned short*)txBits, (unsigned short)*txCount);
    *txCount = (short)ModDataV22(eng, (const unsigned short*)txBits, out, (unsigned short)*txCount);

    /* receive: the S1 detector (0 = present) raises sigAcc, the ones detector raises onesAcc */
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
    RxClampV22(eng, in, rxBits, rxCount);

    /* decide the rate */
    if (us(tones->sigAcc) > 0x61u)
    { /* S1 seen: 2400 */
      tones->timer = 0;
      tones->sigAcc = 0;
      tones->onesAcc = 0;
      tones->sc = V22CONN_2400_A;
    }
    else if (us(tones->onesAcc) > 0xe6u)
    { /* unscrambled ones: 1200 */
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
        eng->state = V22_MSG_ERROR6;
      }
      return 0;
    }
    SetAdaptEqV22(eng, 2);
    return 0;
  }
  case V22CONN_2400_A:
  case V22CONN_2400_B:
  case V22CONN_2400_C:
  case V22CONN_2400_D:
    connect_2400(eng, txBits, out, in, rxBits, txCount, rxCount);
    return 0;
  case V22CONN_1200_A:
  case V22CONN_1200_B:
    connect_1200(eng, txBits, out, in, rxBits, txCount, rxCount);
    return 0;
  default: /* 4 .. 7 */
    return 0;
  }
}

/* ============================================================================================ v22_org_rmloop2 */

/**
 * @brief Protocol handler 4: the originating side of the remote digital loopback.
 * @param eng      engine
 * @param txBits   transmit symbols (generated here)
 * @param out      receives 160 output samples
 * @param in       160 input samples
 * @param rxBits   receives the received symbols
 * @param txCount  transmit symbol count in; sample count out
 * @param rxCount  input sample count in; received symbol count out
 * @return 0 (ignored by the caller)
 *
 * START: clear, equaliser adaptation off, send pattern 1 / 2 (1200 / 2400) and accumulate Detect_Rmloop2_ACK in sigAcc -> WAIT_ACK.  WAIT_ACK: the same;
 * sigAcc > 0xe7 -> SCRAMBLED (and, when the last block's detector value was 0, trained = 1 and adaptation 3); after 0x514 ms adaptation 3 and back to the
 * data state.  SCRAMBLED: send the scrambled pattern; while trained == 0 wait for the acknowledge (trained = 1, adaptation 3); descramble; Detect_1s is
 * accumulated in sigAcc: > 0xe7 -> state byte V22_MSG_ORG_LOOP_UP (adaptation 3); after 0x514 ms V22_MSG_LOOP_TIMEOUT (adaptation 3).
 */
int v22_org_rmloop2(V22Engine* eng, short* txBits, short* out, short* in, short* rxBits, short* txCount, short* rxCount)
{
  V22Tones* tones = eng->tones;
  const short rate2 = eng->cfg.rate2;
  int sc;
  int ack = 0; /* the Detect_Rmloop2_ACK value of the last block with received symbols (sc 1) */
  int pattern;

  eng->flags1e |= 2;
  eng->state = V22_MSG_HANDSHAKE;
  sc = tones->sc;
  pattern = (rate2 != RATE_1200) + 1;
  switch (sc)
  {
  case V22ORML_START:
    tones->timer = 0;
    tones->sigAcc = 0;
    tones->trained = 0;
    SetAdaptEqV22(eng, 1);
    MakeTxData(txBits, txCount, (short)((rate2 != RATE_1200) + 1));
    *txCount = (short)ModDataV22(eng, (const unsigned short*)txBits, out, (unsigned short)*txCount);
    *rxCount = (short)DemodDataV22(eng, in, rxBits, (unsigned short)*rxCount);
    if (*rxCount != 0)
    {
      unsigned int r = Detect_Rmloop2_ACK((const unsigned short*)rxBits, (const unsigned short*)rxCount, rate2);
      tones->sigAcc = (short)(r + us(tones->sigAcc));
    }
    RxClampV22(eng, in, rxBits, rxCount);
    tones->sc = V22ORML_WAIT_ACK;
    return 0;
  case V22ORML_WAIT_ACK:
    MakeTxData(txBits, txCount, (short)pattern);
    *txCount = (short)ModDataV22(eng, (const unsigned short*)txBits, out, (unsigned short)*txCount);
    *rxCount = (short)DemodDataV22(eng, in, rxBits, (unsigned short)*rxCount);
    if (*rxCount != 0)
    {
      ack = (short)Detect_Rmloop2_ACK((const unsigned short*)rxBits, (const unsigned short*)rxCount, rate2);
      tones->sigAcc = (short)((unsigned int)ack + us(tones->sigAcc));
    }
    RxClampV22(eng, in, rxBits, rxCount);
    if (us(tones->sigAcc) > 0xe7u)
    {
      tones->timer = 0;
      tones->sigAcc = 0;
      tones->sc = V22ORML_SCRAMBLED;
      if (ack == 0)
      {
        tones->trained = 1;
        SetAdaptEqV22(eng, 3);
      }
    }
    if ((unsigned int)ReadGTimer(eng) > 0x514u)
    {
      SetAdaptEqV22(eng, 3);
      tones->state = V22PROTO_DATA;
      tones->sc = 0;
    }
    return 0;
  case V22ORML_SCRAMBLED:
    MakeTxData(txBits, txCount, (short)pattern);
    ScrambleDataV22(eng, (unsigned short*)txBits, (unsigned short)*txCount);
    *txCount = (short)ModDataV22(eng, (const unsigned short*)txBits, out, (unsigned short)*txCount);
    *rxCount = (short)DemodDataV22(eng, in, rxBits, (unsigned short)*rxCount);
    if (*rxCount != 0)
    {
      if (tones->trained == 0 && (unsigned short)Detect_Rmloop2_ACK((const unsigned short*)rxBits, (const unsigned short*)rxCount, rate2) == 0)
      {
        tones->trained = 1;
        SetAdaptEqV22(eng, 3);
      }
      DescrambleDataV22(eng, (unsigned short*)rxBits, (unsigned short)*rxCount);
    }
    {
      unsigned int r = Detect_1s((const unsigned short*)rxBits, (const unsigned short*)rxCount, rate2, 0x6ccc);
      tones->sigAcc = (short)(r + us(tones->sigAcc));
    }
    RxClampV22(eng, in, rxBits, rxCount);
    if (us(tones->sigAcc) > 0xe7u)
    {
      tones->timer = 0;
      tones->sigAcc = 0;
      SetAdaptEqV22(eng, 3);
      eng->state = V22_MSG_ORG_LOOP_UP;
    }
    if ((unsigned int)ReadGTimer(eng) > 0x514u)
    {
      SetAdaptEqV22(eng, 3);
      eng->state = V22_MSG_LOOP_TIMEOUT;
    }
    return 0;
  default:
    return 0;
  }
}

/* ============================================================================================ v22_ans_rmloop2 */

/**
 * @brief Protocol handler 5: the answering side of the remote digital loopback.
 * @param eng      engine
 * @param txBits   transmit symbols (the host's data, scrambled and passed through during the first stages)
 * @param out      receives 160 output samples
 * @param in       160 input samples
 * @param rxBits   receives the received symbols
 * @param txCount  transmit symbol count in; sample count out
 * @param rxCount  input sample count in; received symbol count out
 * @return 0 (ignored by the caller)
 *
 * START: clear, adaptation off, scrambled pass-through of txBits, Detect_1s (factor 0x7eb8) accumulated in sigAcc -> WAIT_ONES.  WAIT_ONES: the same;
 * sigAcc > 0x9a -> PATTERN (with trained = 1 and adaptation 3 when the last detector value was 0); after 0x514 ms V22_MSG_LOOP_TIMEOUT.  PATTERN: send
 * pattern 3 / 4; Detect_1s == 0 -> LOOPBACK with V22_MSG_ANS_LOOP_UP; after 0x1388 ms V22_MSG_LOOP_TIMEOUT.  LOOPBACK: the received bits are scrambled and
 * modulated as the transmit data; a carrier absent for more than 0x27 ms (sigAcc, 0x14 per block) gives V22_MSG_ANS_LOOP_LOST; TxClockSync() when
 * V22_status flag bit 2 is set.
 */
int v22_ans_rmloop2(V22Engine* eng, short* txBits, short* out, short* in, short* rxBits, short* txCount, short* rxCount)
{
  V22Tones* tones = eng->tones;
  const short rate2 = eng->cfg.rate2;
  int sc;
  int det = 0; /* the Detect_1s value of the last block with received symbols (sc 1) */

  eng->flags1e |= 2;
  eng->state = V22_MSG_ANS_LOOP_RUNNING;
  sc = tones->sc;
  switch (sc)
  {
  case V22ARML_START:
    tones->timer = 0;
    tones->sigAcc = 0;
    tones->trained = 0;
    tones->sc = V22ARML_WAIT_ONES;
    SetAdaptEqV22(eng, 1);
    ScrambleDataV22(eng, (unsigned short*)txBits, (unsigned short)*txCount);
    *txCount = (short)ModDataV22(eng, (const unsigned short*)txBits, out, (unsigned short)*txCount);
    *rxCount = (short)DemodDataV22(eng, in, rxBits, (unsigned short)*rxCount);
    if (*rxCount != 0)
    {
      unsigned int r = Detect_1s((const unsigned short*)rxBits, (const unsigned short*)rxCount, rate2, 0x7eb8);
      tones->sigAcc = (short)(r + us(tones->sigAcc));
    }
    RxClampV22(eng, in, rxBits, rxCount);
    return 0;
  case V22ARML_WAIT_ONES:
    ScrambleDataV22(eng, (unsigned short*)txBits, (unsigned short)*txCount);
    *txCount = (short)ModDataV22(eng, (const unsigned short*)txBits, out, (unsigned short)*txCount);
    *rxCount = (short)DemodDataV22(eng, in, rxBits, (unsigned short)*rxCount);
    if (*rxCount != 0)
    {
      det = (short)Detect_1s((const unsigned short*)rxBits, (const unsigned short*)rxCount, rate2, 0x7eb8);
      tones->sigAcc = (short)((unsigned int)det + us(tones->sigAcc));
    }
    RxClampV22(eng, in, rxBits, rxCount);
    if (us(tones->sigAcc) > 0x9au)
    {
      tones->timer = 0;
      tones->sigAcc = 0;
      tones->sc = V22ARML_PATTERN;
      if (det == 0)
      {
        tones->trained = 1;
        SetAdaptEqV22(eng, 3);
      }
    }
    if ((unsigned int)ReadGTimer(eng) > 0x514u)
    {
      SetAdaptEqV22(eng, 3);
      eng->state = V22_MSG_LOOP_TIMEOUT;
    }
    return 0;
  case V22ARML_PATTERN:
    MakeTxData(txBits, txCount, (short)((rate2 != RATE_1200) + 3));
    ScrambleDataV22(eng, (unsigned short*)txBits, (unsigned short)*txCount);
    *txCount = (short)ModDataV22(eng, (const unsigned short*)txBits, out, (unsigned short)*txCount);
    *rxCount = (short)DemodDataV22(eng, in, rxBits, (unsigned short)*rxCount);
    if (*rxCount != 0 && (unsigned short)Detect_1s((const unsigned short*)rxBits, (const unsigned short*)rxCount, rate2, 0x7eb8) == 0)
    {
      tones->trained = 0;
      tones->sigAcc = 0;
      tones->sc = V22ARML_LOOPBACK;
      SetAdaptEqV22(eng, 3);
      RxClampV22(eng, in, rxBits, rxCount);
      eng->state = V22_MSG_ANS_LOOP_UP;
    }
    RxClampV22(eng, in, rxBits, rxCount);
    if ((unsigned int)ReadGTimer(eng) > 0x1388u)
    {
      SetAdaptEqV22(eng, 3);
      eng->state = V22_MSG_LOOP_TIMEOUT;
    }
    return 0;
  case V22ARML_LOOPBACK:
  {
    V22Status st = {0};
    V22_status(eng, &st);
    *rxCount = (short)DemodDataV22(eng, in, rxBits, (unsigned short)*rxCount);
    ScrambleDataV22(eng, (unsigned short*)rxBits, (unsigned short)*rxCount);
    *txCount = (short)ModDataV22(eng, (const unsigned short*)rxBits, out, (unsigned short)*rxCount);
    RxClampV22(eng, in, rxBits, rxCount);
    if (CarrierDetect(eng) == 0)
    {
      tones->sigAcc = (short)(us(tones->sigAcc) + 0x14u);
      if (us(tones->sigAcc) > 0x27u)
      {
        eng->state = V22_MSG_ANS_LOOP_LOST;
        return 0;
      }
    }
    else
    {
      tones->sigAcc = 0;
    }
    if (st.flags & 4)
      TxClockSync(eng);
    return 0;
  }
  default:
    return 0;
  }
}
