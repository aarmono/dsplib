/**
 * @file v22_data.c
 * @brief v22_data(): the data state of the V.22 / V.22bis / Bell 212A engine (protocol handler 0, tones.state V22PROTO_DATA).
 *
 * Called only through V22_PROTOCOL by V22FP_modem() once per 160-sample block with the engine's static 16-bit buffers.  It carries the user data in both
 * directions and supervises the link:
 * - transmit: ScrambleDataV22() (V22_status flag bit 0) + ModDataV22() on the caller's bits;
 * - receive: DemodDataV22(), TxClockSync() (V22_status flag bit 2), and at 2400 bit/s (core.rxBis == 1) the detection of a retrain request and of poor
 *   signal quality; the descrambler (flag bit 1);
 * - carrier supervision with the loss time cfg.lossMs.
 * With the engine configuration "data mode directly" (cfg.b11 bit 3) the receive path is not run at all (see v22_data()).
 */
#include <stdint.h>

#include "v22.h"
#include "core/dsp_host.h"

#define DBG1 (1 < dsplibs_debug_level)

/**
 * @brief Run one data-state block.
 * @param eng      engine
 * @param txBits   transmit symbols (the host's data)
 * @param out      receives 160 output samples
 * @param in       160 input samples
 * @param rxBits   receives the received symbols
 * @param txCount  transmit symbol count in; sample count out
 * @param rxCount  input sample count in; received symbol count out
 * @return 0 (ignored by the caller)
 *
 * Data mode directly (cfg.b11 bit 3): no receive path; every block counts as a block without carrier (lossCnt) and is supervised against
 * cfg.handshakeTimeoutMs (lossCnt * 20 ms, unsigned): exceeded -> state byte V22_MSG_NO_CARRIER and the received bits clamped to ones.
 *
 * Otherwise, at 2400 bit/s: a received symbol block is scanned with Detect_Retrain(); a hit means the REMOTE modem requests a retrain (timer and
 * accumulators cleared, retrainLocal = 0, ResetRx, equaliser adaptation off, rates 1200, tones.state = V22PROTO_RETRAIN, state byte V22_MSG_REMOTE_RETRAIN)
 * and the function returns at once.  With flag bit 7 (retrain on poor quality) a signal quality (low 16 bits) of at most 0x7f37 adds 0x14 ms to tones.sigAcc
 * per block (reset to 0 above); more than 0xbb8 ms (3 s) starts a local retrain (tones.state = V22PROTO_RETRAIN, stage RESTART, state byte
 * V22_MSG_LOCAL_RETRAIN, retrainLocal = 1).
 *
 * Carrier supervision: carrier present with lossCnt > 14 at 2400 bit/s starts a local retrain ("Carrier back during carrier_loss_time (v22_data)"),
 * otherwise lossCnt = 0; carrier absent: lossCnt++ and (short)(lossCnt * 20) is compared with (short)cfg.lossMs; while not exceeded only a message is
 * printed, once exceeded the state byte is V22_MSG_NO_CARRIER and the received bits are clamped to ones.
 *
 * The common exit ORs 5 into flags1d and stores the state byte (V22_MSG_RUNNING, V22_MSG_NO_CARRIER or V22_MSG_LOCAL_RETRAIN); the remote retrain request
 * returns before that (flags1d untouched).
 */
int v22_data(V22Engine* eng, short* txBits, short* out, short* in, short* rxBits, short* txCount, short* rxCount)
{
  V22Status st = {0};
  unsigned int state = V22_MSG_RUNNING;
  V22Tones* tones = eng->tones;
  V22Core* core = eng->core; /* read once, so the code does not depend on the order of these loads against the short stores through txCount / rxCount */
  unsigned int flags;

  /* transmit */
  V22_status(eng, &st);
  flags = st.flags;
  if (flags & 1)
    ScrambleDataV22(eng, (unsigned short*)txBits, (unsigned short)*txCount);
  *txCount = (short)ModDataV22(eng, (const unsigned short*)txBits, out, (unsigned short)*txCount);

  /* data mode directly: no receive path */
  if (eng->cfg.b11 & 8)
  {                                                          /* no receive path: only the supervision of the blocks without carrier */
    unsigned int blocks = (unsigned int)tones->lossCnt + 1u; /* read before the short store (the code does not depend on the aliasing of that store) */
    *rxCount = 0;
    tones->lossCnt = (int)blocks;
    if (blocks * 20u > (unsigned int)eng->cfg.handshakeTimeoutMs)
    {
      state = V22_MSG_NO_CARRIER;
      RxClampV22(eng, in, rxBits, rxCount);
    }
    eng->flags1d |= 5;
    eng->state = (unsigned char)state;
    return 0;
  }

  /* receive: demodulate, slave the transmit clock, look for a retrain request, descramble */
  *rxCount = (short)DemodDataV22(eng, in, rxBits, (unsigned short)*rxCount);
  (void)GetSignalQuality(eng);
  if (flags & 4)
    TxClockSync(eng);
  if (*rxCount != 0)
  {
    if (core->rxBis == 1)
    {
      if ((short)Detect_Retrain((const unsigned short*)rxBits, (const unsigned short*)rxCount) > 0)
      { /* the remote modem requests a retrain */
        tones->timer = 0;
        tones->sigAcc = 0;
        tones->onesAcc = 0;
        tones->sc = V22RETRAIN_REMOTE_REQ;
        tones->retrainLocal = 0;
        ResetRx(eng);
        SetAdaptEqV22(eng, 1);
        SetTxRate(eng, 0);
        SetRxRate(eng, 0);
        tones->state = V22PROTO_RETRAIN;
        RxClampV22(eng, in, rxBits, rxCount);
        eng->state = V22_MSG_REMOTE_RETRAIN;
        if (DBG1)
          dsplibs_debug_printf("V.22: Retrain request detected.");
        return 0;
      }
    }
    if (flags & 2)
      DescrambleDataV22(eng, (unsigned short*)rxBits, (unsigned short)*rxCount);
  }

  if (core->rxBis == 1 && (flags & 0x80))
  { /* retrain on a poor signal quality */
    if ((unsigned short)GetSignalQuality(eng) > 0x7f37u)
    {
      tones->sigAcc = 0;
    }
    else
    {
      tones->sigAcc = (short)((unsigned short)tones->sigAcc + 0x14u);
      if ((unsigned short)tones->sigAcc > 0xbb8u)
      {
        tones->state = V22PROTO_RETRAIN;
        tones->sc = V22RETRAIN_RESTART;
        SetAdaptEqV22(eng, 1);
        state = V22_MSG_LOCAL_RETRAIN;
        tones->retrainLocal = 1;
        if (DBG1)
          dsplibs_debug_printf("Signal quality < Retrain level. Retrain initiated.");
      }
    }
  }

  /* carrier supervision */
  if (CarrierDetect(eng) != 0)
  {
    if (tones->lossCnt > 0xe && core->rxBis == 1)
    { /* carrier back after more than 14 blocks: local retrain */
      tones->state = V22PROTO_RETRAIN;
      tones->sc = V22RETRAIN_RESTART;
      SetAdaptEqV22(eng, 1);
      state = V22_MSG_LOCAL_RETRAIN;
      tones->retrainLocal = 1;
      tones->lossCnt = 0;
      if (DBG1)
        dsplibs_debug_printf("Carrier back during carrier_loss_time (v22_data). Retrain initiated.");
    }
    else
    {
      tones->lossCnt = 0;
    }
  }
  else
  {
    int loss = (short)eng->cfg.lossMs; /* read before the counter is stored (the code does not depend on the aliasing of the int store) */
    int lim;
    tones->lossCnt = (int)((unsigned int)tones->lossCnt + 1u);
    lim = (short)((unsigned int)tones->lossCnt * 20u);
    if (loss >= lim)
    {
      if (DBG1)
        dsplibs_debug_printf("V22_MSG_NO_CARRIER won't be reported (carrier_loss_time %d of %d ms)\n", lim, loss);
    }
    else
    {
      state = V22_MSG_NO_CARRIER;
      if (DBG1)
        dsplibs_debug_printf("V22_MSG_NO_CARRIER2\n");
      RxClampV22(eng, in, rxBits, rxCount);
    }
  }

  eng->flags1d |= 5;
  eng->state = (unsigned char)state;
  return 0;
}
