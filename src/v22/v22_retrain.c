/**
 * @file v22_retrain.c
 * @brief v22_retrain(): the V.22bis 2400 bit/s retrain handshake (protocol handler 6, tones.state V22PROTO_RETRAIN).
 *
 * Called only through V22_PROTOCOL by V22FP_modem() once per 160-sample block with the engine's static 16-bit buffers (seven arguments as v22_data); the
 * return value is ignored.  It is entered by v22_data (a retrain requested by the remote modem: stage 0, tones.retrainLocal = 0; a local one: stage 1,
 * retrainLocal = 1) and by connect_2400 (carrier back: stage 1, retrainLocal = 1).  The engine state byte is V22_MSG_RETRAIN for the whole handshake; it
 * becomes V22_MSG_CONNECT_2400 with flags1d |= 0x29 at the end (flags1d |= 1 instead when the last stage timed out without training).
 *
 * The S1 detector (the MTDs1 tone detector on the receive filter output) reports the tone with a result of 0; `sigAcc` is raised by 0x14 for each block
 * in which it does.  The stages (V22RetrainStage) dispatch on tones.sc; a value above 7 does nothing.
 */
#include <stdint.h>

#include "fpm/fpm.h"
#include "v22.h"

#include "v22_us.h"
#include "core/dsp_constants.h" /* RATE_* */

/* The transmit side shared by the stages: MakeTxData with the pattern, optionally ScrambleDataV22, then ModDataV22 into `out`; *txCount becomes the sample
 * count. */
static void tx_side(V22Engine* eng, short* txBits, short* out, short* txCount, int pattern, int scramble)
{
  MakeTxData(txBits, txCount, (short)pattern);
  if (scramble)
    ScrambleDataV22(eng, (unsigned short*)txBits, (unsigned short)*txCount);
  *txCount = (short)ModDataV22(eng, (const unsigned short*)txBits, out, (unsigned short)*txCount);
}
/* The receive side shared by the stages: DemodDataV22, optionally DescrambleDataV22; *rxCount becomes the number of received symbols. */
static void rx_side(V22Engine* eng, short* in, short* rxBits, short* rxCount, int descramble)
{
  *rxCount = (short)DemodDataV22(eng, in, rxBits, (unsigned short)*rxCount);
  if (descramble)
    DescrambleDataV22(eng, (unsigned short*)rxBits, (unsigned short)*rxCount);
}
/* Run the S1 detector over the receive filter output of the current block; 0 means that S1 is present. */
static short s1_detect(V22Engine* eng)
{
  V22Core* core = eng->core;
  return (short)FPM_MTD_detect(eng->tones->mtdS1, core->rxBuf, core->rxCount);
}

/**
 * @brief Run one block of the retrain handshake.
 * @param eng      engine
 * @param txBits   transmit symbols (generated here)
 * @param out      receives 160 output samples
 * @param in       160 input samples
 * @param rxBits   receives the received symbols
 * @param txCount  transmit symbol count in; sample count out
 * @param rxCount  input sample count in; received symbol count out
 * @return 0 (ignored by the caller)
 *
 * Stages:
 * - REMOTE_REQ (0): send the scrambled pattern (1 at 1200, 2 at 2400) and watch the S1 detector; once S1 has been seen and is gone, continue at RESTART
 *   (equaliser adaptation 2); after 0x514 ms go to 2400 (both rates, adaptation 3) and back to the data state.
 * - RESTART (1): clear the timer and accumulators, ResetRx, rates 1200, send silence (pattern 0, unscrambled); the S1 detector runs but its result is
 *   not used; -> SILENCE.
 * - SILENCE (2): send silence, accumulate S1 detections; after 0x63 ms continue at SCRAMBLED_ONES (retrainLocal == 1) or ONES_TO_RX_2400.
 * - SCRAMBLED_ONES (3): send scrambled ones, watch S1 as in stage 0; once S1 has been seen and is gone, continue at ONES_TO_RX_2400 (adaptation 2); after
 *   0x514 ms go to 2400 (rates, adaptation 3) and report V22_MSG_CONNECT_2400 with flags1d |= 1 (no training).
 * - ONES_TO_RX_2400 (4): scrambled ones; after 0x1c1 ms the receive rate becomes 2400.  ONES_TO_TX_2400 (5): the same with descrambling; after 0x257 ms
 *   the transmit rate becomes 2400.
 * - DATA_2400 (6): the 2400 data pattern (pattern 2), descrambling; after 0x31f ms the equaliser adaptation is set to 3.  TRAINING_2400 (7): the same
 *   with RxTrained2400(): trained -> V22_MSG_CONNECT_2400 and flags1d |= 0x29; after 0x7cf ms V22_MSG_CONNECT_2400 and flags1d |= 1 regardless.
 */
int v22_retrain(V22Engine* eng, short* txBits, short* out, short* in, short* rxBits, short* txCount, short* rxCount)
{
  V22Tones* tones = eng->tones;
  unsigned int sw;
  short n;
  int seen; /* sc 0 / 3: 0x14 when this block had symbols and no S1 detection (the accumulator was raised), 0 otherwise */

  eng->state = V22_MSG_RETRAIN;
  sw = (unsigned int)(int)tones->sc;
  if (sw > 7u)
    return 0;
  switch (sw)
  {
  case V22RETRAIN_REMOTE_REQ:
  case V22RETRAIN_SCRAMBLED_ONES:
    tx_side(eng, txBits, out, txCount, sw == 0 ? (eng->cfg.rate2 != RATE_1200) + 1 : 1, 1);
    rx_side(eng, in, rxBits, rxCount, 0);

    /* accumulate the blocks with S1 present */
    n = s1_detect(eng);
    seen = 0;
    if (*rxCount != 0 && n == 0)
    {
      seen = 0x14;
      tones = eng->tones;
      tones->sigAcc = (short)(us(tones->sigAcc) + 0x14u);
    }
    tones = eng->tones;
    if (tones->sigAcc != 0 && seen == 0)
    { /* S1 has been seen and is gone in this block: the next stage */
      tones->timer = 0;
      tones->sigAcc = 0;
      tones->sc = (sw == V22RETRAIN_REMOTE_REQ) ? V22RETRAIN_RESTART : V22RETRAIN_ONES_TO_RX_2400;
      SetAdaptEqV22(eng, 2);
    }
    RxClampV22(eng, in, rxBits, rxCount);

    /* timeout of the stage */
    if (sw == 0)
    {
      if ((unsigned int)ReadGTimer(eng) > 0x514u)
      {
        SetTxRate(eng, 1);
        SetRxRate(eng, 1);
        SetAdaptEqV22(eng, 3);
        tones = eng->tones;
        tones->state = V22PROTO_DATA;
        tones->sc = 0;
      }
    }
    else
    {
      if ((unsigned int)ReadGTimer(eng) > 0x514u)
      {
        RxClampV22(eng, in, rxBits, rxCount);
        SetTxRate(eng, 1);
        SetRxRate(eng, 1);
        SetAdaptEqV22(eng, 3);
        eng->flags1d |= 1;
        eng->state = V22_MSG_CONNECT_2400;
      }
    }
    return 0;
  case V22RETRAIN_RESTART:
    tones->timer = 0;
    tones->sigAcc = 0;
    tones->onesAcc = 0;
    tones->sc = V22RETRAIN_SILENCE;
    ResetRx(eng);
    SetTxRate(eng, 0);
    SetRxRate(eng, 0);
    tx_side(eng, txBits, out, txCount, 0, 0);
    rx_side(eng, in, rxBits, rxCount, 0);
    (void)s1_detect(eng);
    RxClampV22(eng, in, rxBits, rxCount);
    (void)ReadGTimer(eng); /* advance the timer by 20 ms */
    return 0;
  case V22RETRAIN_SILENCE:
    tx_side(eng, txBits, out, txCount, 0, 0);
    rx_side(eng, in, rxBits, rxCount, 0);
    n = s1_detect(eng);
    if (*rxCount != 0 && n == 0)
    {
      tones = eng->tones;
      tones->sigAcc = (short)(us(tones->sigAcc) + 0x14u);
    }
    RxClampV22(eng, in, rxBits, rxCount);
    if ((unsigned int)ReadGTimer(eng) > 0x63u)
    {
      tones = eng->tones;
      tones->sc = (short)((tones->retrainLocal != 1) + 3); /* V22RETRAIN_SCRAMBLED_ONES (local) or V22RETRAIN_ONES_TO_RX_2400 (requested by the remote modem) */
    }
    return 0;
  case V22RETRAIN_ONES_TO_RX_2400:
    tx_side(eng, txBits, out, txCount, 1, 1);
    rx_side(eng, in, rxBits, rxCount, 0);
    RxClampV22(eng, in, rxBits, rxCount);
    if ((unsigned int)ReadGTimer(eng) > 0x1c1u)
    {
      SetRxRate(eng, 1);
      eng->tones->sc = V22RETRAIN_ONES_TO_TX_2400;
    }
    return 0;
  case V22RETRAIN_ONES_TO_TX_2400:
    tx_side(eng, txBits, out, txCount, 1, 1);
    rx_side(eng, in, rxBits, rxCount, 1);
    RxClampV22(eng, in, rxBits, rxCount);
    if ((unsigned int)ReadGTimer(eng) > 0x257u)
    {
      SetTxRate(eng, 1);
      eng->tones->sc = V22RETRAIN_DATA_2400;
    }
    return 0;
  case V22RETRAIN_DATA_2400:
    tx_side(eng, txBits, out, txCount, 2, 1);
    rx_side(eng, in, rxBits, rxCount, 1);
    RxClampV22(eng, in, rxBits, rxCount);
    if ((unsigned int)ReadGTimer(eng) > 0x31fu)
    {
      SetAdaptEqV22(eng, 3);
      eng->tones->sc = V22RETRAIN_TRAINING_2400;
    }
    return 0;
  default: /* 7 */
    tx_side(eng, txBits, out, txCount, 2, 1);
    rx_side(eng, in, rxBits, rxCount, 1);
    if (RxTrained2400(rxBits, (const unsigned short*)rxCount) != 0)
    {
      eng->flags1d |= 0x29;
      eng->state = V22_MSG_CONNECT_2400;
      RxClampV22(eng, in, rxBits, rxCount);
    }
    RxClampV22(eng, in, rxBits, rxCount);
    if ((unsigned int)ReadGTimer(eng) > 0x7cfu)
    {
      eng->flags1d |= 1;
      eng->state = V22_MSG_CONNECT_2400;
    }
    return 0;
  }
}
