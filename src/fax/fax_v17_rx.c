/**
 * @file fax_v17_rx.c
 * @brief The fax V.17 receiver (7200 / 9600 / 12000 / 14400 bit/s, trellis coded; T.30 high-speed data).
 *
 * Entry points: V17RX_create() / V17RX_delete() / V17RX_modem() / V17RX_control() / V17RX_status(), the half-duplex state handlers
 * RxHdx{Start,EpochDet,Prtcol,Bridge,Scram,Data, Idle,Error}V17() with the state switch RxNextStateV17(), the signal path DemodDataV17(), the carrier /
 * quality / SNR detectors (DataCarrierDetectV17(), CarrierDetectV17(), EpochDetectV17(), QualityDetectV17(), GetSNRV17()), the short-training helpers
 * StoreCoefV17() / Restore_rateV17(), and the VMI wrappers v17rx_*() with the initialiser fax_init_vmi_v17rx().  The objects are in fax_v17_rx.h, the tables
 * in fax_v17_rx_tables.c.  The timing-recovery (SRE) and equalizer (FSE) sub-objects are fax_timing.c / fax_equalizer.c (shared with the V.27ter and V.29
 * receivers); the FSE decision callbacks, the history object FaxDec17 and the Viterbi front end are fax_v17_decision.c / fax_v17_viterbi.c (the receiver only
 * installs FAX_FSE_decision_AB as the first callback and initialises the history object).  The receive descrambler DescrambleDataV17() is in
 * fax_modem_wrappers.c; on a 64-bit build this file defines the accessor faxw_v17_rxsdm() that it uses to reach `bundle->sdm`.
 *
 * Data flow: V17RX_modem(in samples) -> hdx handler loop (each handler consumes the whole block and returns the number of symbols produced) -> DemodDataV17()
 * = AGC (in place) -> [state 0 only: tone killer + tone detector; a detected tone aborts the block] -> matched filter (MRF) -> symbol timing recovery (SRE)
 * -> equalizer, carrier PLL and decision loop (FSE, whose callback decides the symbols) -> descrambler.
 *
 * States (hdx.state, RxNextStateV17()): 0 start (wait for carrier) -> 1 epoch detect -> 2 protocol (long training: the AB / equalizer-training decision
 * callbacks) -> 3 bridge -> 4 SCRAM (scrambler-1 segment, ends with the connect code) -> 5 data -> 6 idle (decision quality lost; back to data when the error
 * is small); 7 error.  With the short-training flag (cfg word 5) the receiver skips the bridge (state 2 goes straight to 4) and starts the equalizer from the
 * taps a previous call stored in the caller's buffers.
 *
 * Behaviour that is easy to miss:
 * - RxNextStateV17() switches on hdx.state 0 .. 6; every other value (including the error state 7) takes the default case (status code V17RX_MSG_ERROR, error
 *   flag). Each case prints its debug line (level > 1) before it acts, with the name of the state it leaves.  The transition 1 -> 2 advances the AGC alpha /
 *   beta table indices by one entry; with the short-training flag it first calls Restore_rateV17() and sets the protocol count to 1 (else 0x3e).  The
 *   transition 2 -> 3 (long training only) stores the equalizer taps (StoreCoefV17()).  Idle -> data restores the connect code 9 (7200) / 8 (9600) / 7
 *   (12000) / 6 (14400); RxHdxScramV17() sets the same code when its count runs out.
 * - The connect codes run opposite to the rate index: rate index 0 (7200) -> 9, 1 -> 8, 2 -> 7, 3 (or any other) -> 6.
 * - V17RX_create(): the buffers and sub-objects are (re)initialised "fresh" (their storage allocated) only when the object is allocated here; the hdx's own
 *   AGC is fresh only when the hdx is allocated here.  The receiver owns the history object of the FSE callbacks: zeroed counters, a FaxVtb set up like the
 *   V.32 decoder for the rate's family (1 .. 4) and an SGD created from SGD_CFG with bits 2, w24 high half 0x2000, w28 0x111, w2c 0xffff.  The matched filter
 *   is up 9 / down 10 with 0x168 taps; the SRE loop constants (0x55 / 0x30) and the FSE filter tables depend on the short-training flag: with it the FSE
 *   starts from the caller's taps (cfg coefI / coefQ) and uses the "_S" PLL tables.
 * - DemodDataV17() returns 0 without running the filters when the state is 0 and the tone detector fires (the tone killer works on a plain copy of the
 *   input).  The AGC output (agc.detect) gates the SRE loop (sre.loopEnable = sreGate & detect) and the FSE (pllEnable = pllGate & detect, eqEnable = eqGate
 *   & detect, predEnable = 0).  An SRE output of more than 164 samples is reported ("ERROR: SRE buffer violation!(%d)", debug level > 1) after the fact.
 * - DataCarrierDetectV17() / CarrierDetectV17() have an extra test for short training: only when the flag is set, the equalizer's training flag (fse.eqForce)
 *   is set and the history object's call counter exceeded 999 may a decoder error above 0x3fff drop the carrier; otherwise the AGC / SRE gate decides.
 * - GetSNRV17() = 13 - fse.decErrAvg.  V17RX_status() fills the 0x1c-byte record (b14 is rebuilt completely from bundle words w1c / w0 / eqGate, b15 keeps
 *   the caller's other bits, w18 is not written).
 * - v17rx_create() returns a rate code, not the engine pointer (the engine only goes into the VMI object); v17rx_message(m, 10) is one entry past V17RX_MESG
 *   and stores NULL.
 */
#include <string.h>
#include "fax_v17_rx.h"
#include "fax_v17_rx_tables.h"
#include "fax_v17_tx_tables.h"
#include "fax_v17_decision_tables.h"
#include "fax_v21_tables.h"
#include "fax_common_tables.h"
#include "fpm/fpm_tables.h"
#include "v32/v32_viterbi_tables.h"
#include "fax_modem_views.h"
#include "core/dsp_asm.h"

#include "fax_debug.h" /* dsplibs_debug_level, DBG, DBG3 */
#include "core/dsp_host.h"
#include "fpm/fpm.h"

/* the functions marked NOINL keep the order of their loads and stores the same with and without -fno-strict-aliasing (compared on the objects built with and without the flag) */
#include "fax_noinl.h"          /* NOINL */
#include "core/dsp_constants.h" /* RATE_* */

#if UINTPTR_MAX == 0xffffffffu
/* the raw-offset accessor of fax_modem_views.h reads exactly these members */
_Static_assert(offsetof(V17Rx, bundle) == 0x60 && offsetof(V17RxBundle, sdm) == 0x4f8c, "faxw_v17_rxsdm offsets");
#else
FaxSdm* faxw_v17_rxsdm(void* o) { return &((V17Rx*)o)->bundle->sdm; }
#endif

/* ============================================================ detectors */
/**
 * @brief Signal-to-noise estimate from the equalizer's decision error.
 * @param o  receiver
 * @return 13 - fse.decErrAvg (16 bit); the handlers flag a low SNR when it is 8 or less
 */
NOINL int GetSNRV17(V17Rx* o) { return (short)(0xd - (unsigned short)o->bundle->fse.decErrAvg); }

/**
 * @brief Carrier detector of the training states.
 * @param o  receiver
 * @return non-zero while the carrier is present: SRE PLL gate and AGC detect; with short training, equalizer training active and more than 999 decisions, a
 *   decoder error above 0x3fff drops the carrier and otherwise only bit 0 of the gate result is kept
 */
NOINL int CarrierDetectV17(V17Rx* o)
{
  V17RxBundle* b = o->bundle;
  int r = b->sre.pllEnable & b->agc.detect;
  V17RxHdx* h = o->hdx;
  if (h->flag != 0 && b->fse.eqForce != 0 && (short)b->dec.ctr > 0x3e7)
  {
    short decErr = b->fse.decErrAvg;
    if (decErr > 0x3fff)
      r = 0;
    else
      r &= 1;
    if (b->fse.decErrAvg > 0x3fff)
    {
      if (DBG)
        dsplibs_debug_printf("V17 Decoder error too big... no carrier\n");
    }
  }
  return r;
}

/**
 * @brief Epoch (start of training) detection.
 * @param o  receiver
 * @return non-zero once the FSE's equalizer-training flag (fse.eqForce) is set, i.e. the decision callback has seen the training start
 */
NOINL int EpochDetectV17(V17Rx* o) { return o->bundle->fse.eqForce != 0; }

/**
 * @brief Quality tracker of the data state: smooths the decoder error over the first 50 blocks and compares it with the rate's limit.
 * @param o  receiver
 * @return the carrier flag (AGC detect & SRE PLL gate), or 2 when there is none ("unreliable data": the caller discards the block)
 * @note Block 0 loads the smoothed error, blocks 1 .. 0x31 average it (weights 0xccd / 0x7333, Q15), block 0x32 sets `qualityOk` when the smoothed error did
 *   not exceed `errLimit`.  `qualityOk` is written here but read nowhere in this receiver.
 */
NOINL int QualityDetectV17(V17Rx* o)
{
  V17RxBundle* b = o->bundle;
  int pllOn = b->sre.pllEnable;
  short det = (short)b->agc.detect;
  short decErr = b->fse.decErrAvg;
  int r = (short)(det & pllOn);
  unsigned short blocks;
  if (r == 0)
  {
    if (DBG)
      dsplibs_debug_printf("V17 Dec error too big... unreliable data\n");
    r = 2;
  }

  /* block 0 loads the smoothed error, blocks 1 .. 0x31 average it, block 0x32 judges it */
  blocks = (unsigned short)b->qualBlocks;
  if (blocks == 0)
  {
    b->decErrSmooth = decErr;
    b->qualBlocks = 1;
    return r;
  }
  if ((short)blocks <= 0x31)
  {
    b->decErrSmooth = (short)((((int)decErr * 0xccd + 0x4000) >> 15) + (((int)b->decErrSmooth * 0x7333 + 0x4000) >> 15));
    b->qualBlocks = (short)(blocks + 1);
    return r;
  }
  if ((short)blocks == 0x32)
  {
    if (!(b->decErrSmooth > (short)b->errLimit))
      b->qualityOk = 1;
    b->qualBlocks = (short)(b->qualBlocks + 1);
  }
  return r;
}

/**
 * @brief Carrier detector of the data state: gate flags, decoder-error test, optional V.21 carrier search and an energy-drop test.
 * @param o   receiver
 * @param in  the input block (not modified)
 * @param n   number of samples in `in`; at most the size of hdx.buf2 (0xa0 samples)
 * @return non-zero while the carrier is present
 * @note With hdx.s20 == 0 (always, as created) the result is the CarrierDetectV17() rule.  Otherwise a V.21 carrier search runs on a copy of the block: when
 *   the decoder error is large or the gate is closed it sets `toneSearch`, and a V.21 tone seen for more than 0x4ff samples ends the carrier.  Independently,
 *   when the energy test is enabled a block whose RMS fell below 0x32fe/0x8000 (about -8 dB) of the reference level drops the carrier; the reference is
 *   refreshed every second block.
 * @warning `n` is not checked against the capacity of hdx.buf2 in the V.21 search branch.
 */
NOINL int DataCarrierDetectV17(V17Rx* o, short* in, unsigned short n)
{
  V17RxBundle* b = o->bundle;
  V17RxHdx* h = o->hdx;
  int ebx = (short)((short)b->agc.detect & b->sre.pllEnable);

  /* carrier decision: the plain gate rule, or the V.21 carrier search */
  if (h->s20 == 0)
  {
    if (h->flag != 0 && b->fse.eqForce != 0 && (short)b->dec.ctr > 0x3e7)
    {
      short decErr = b->fse.decErrAvg;
      if (decErr > 0x3fff)
        ebx = 0;
      else
        ebx &= 1;
      if (b->fse.decErrAvg > 0x3fff)
      {
        if (DBG)
          dsplibs_debug_printf("V17 Decoder error too big... no carrier\n");
      }
    }
  }
  else
  {
    short decErr = b->fse.decErrAvg;
    if (decErr > 0x3fff)
      h->toneSearch = 1;
    else if (!(ebx & 1))
      h->toneSearch = 1;
    ebx = 1;
    if (h->toneSearch != 0)
    {
      short i = 0;
      short* const buf = h->buf2;
      while (i < (int)n)
      {
        buf[i] = in[i];
        i = (short)(i + 1);
      }
      FPM_AGC_agc(&h->agc, buf, n);
      if ((unsigned short)FPM_MTD_detect(h->mtd2, buf, (short)n) != 0)
      {
        h->sinceTone = 0;
      }
      else
      {
        h->sinceTone = (short)((unsigned short)h->sinceTone + n);
      }
      ebx = 1;
      if (h->sinceTone > 0x4ff)
      {
        if (DBG)
          dsplibs_debug_printf("V17: V21 Carrier detected\n");
        ebx = 0;
      }
    }
  }

  /* energy-drop test against the reference level */
  if (b->energyTest != 0)
  {
    short rms = FPM_rms(in, (short)n);
    int thr = ((int)b->refLevel * 0x32fe) >> 15;
    unsigned short c;
    if ((int)rms < thr)
    {
      ebx = 0;
      if (DBG)
        dsplibs_debug_printf("sudden energy drop > 8[dB], no carrier");
    }
    c = (unsigned short)(b->refBlocks + 1);
    if (c == 2)
    {
      b->refLevel = rms;
      b->refBlocks = 0;
    }
    else
    {
      b->refBlocks = (short)c;
    }
  }
  return ebx;
}

/**
 * @brief Short training: restore the carrier phase integral from the caller's buffer and arm the settle counter.
 * @param o  receiver; cfg.freq must be valid
 * @note Called at the epoch-detect -> protocol transition.  Sets fse.freqAcc = *cfg.freq and fse.settleCtr = fse.cfg.settleCount + 5.
 */
NOINL void Restore_rateV17(V17Rx* o)
{
  V17RxBundle* b = o->bundle;
  int savedFreq = *o->cfg.freq;
  unsigned short thr = (unsigned short)b->fse.cfg.settleCount;
  b->fse.freqAcc = savedFreq;
  b->fse.settleCtr = (short)(thr + 5);
}

/**
 * @brief Save the equalizer taps (0x31 per filter) and the carrier phase integral into the caller's buffers.
 * @param o  receiver; cfg.coefI / coefQ must hold 0x31 shorts and cfg.freq one short
 * @note Called at the protocol -> bridge transition of a long training, so a following short training can start from these values.
 */
NOINL void StoreCoefV17(V17Rx* o)
{
  V17RxBundle* b = o->bundle;
  short *const ci = o->cfg.coefI, *const cq = o->cfg.coefQ;
  const short *const ti = b->fse.tapsI, *const tq = b->fse.tapsQ;
  unsigned short i = 0;
  do
  {
    ci[i] = ti[i];
    cq[i] = tq[i];
    i = (unsigned short)(i + 1);
  } while (i <= 0x30);
  *o->cfg.freq = (short)b->fse.freqAcc;
}

/* ============================================================ demodulator */
/**
 * @brief Run one block through the receive chain: AGC, (start state) tone killer + tone detector, matched filter, symbol timing recovery, equalizer /
 *   decision loop.
 * @param o    receiver
 * @param in   input samples (the AGC works in place)
 * @param out  decoded symbols / bits
 * @param n    number of input samples
 * @return number of symbols produced; 0 when the tone detector fired in the start state
 * @note The loop gates are `bundle->sreGate`, `pllGate` and `eqGate`, each ANDed with the AGC's detect flag.
 * @warning An SRE output above 164 samples (the size of bundle->buf2) is only reported after it has been written.
 */
NOINL int DemodDataV17(V17Rx* o, short* in, short* out, unsigned short n)
{
  V17RxBundle* b = o->bundle;
  V17RxHdx* h;
  int detect;
  unsigned int produced;
  unsigned int cnt;

  FPM_AGC_agc(&b->agc, in, n);
  detect = b->agc.detect; /* the AGC's return value, i.e. the detect flag it just stored */
  h = o->hdx;
  if (h->state == V17RX_STATE_START)
  {
    unsigned short i;
    short* const buf = h->buf; /* the sub-objects are cached before the copy loop: same code with and without strict aliasing */
    FpmTone* const tone = h->tone;
    FpmMtd* const mtd = h->mtd;

    /* start state: remove the tone on a copy of the block; a detected tone aborts the block */
    for (i = 0; i < n; i++)
      buf[i] = in[i];
    FPM_TONE_kill(tone, buf, (short)n);
    if ((unsigned short)FPM_MTD_detect(mtd, buf, (short)n) != 0)
      return 0;
  }

  /* matched filter, symbol timing recovery, then the equalizer / decision loop with the gates applied */
  produced = (unsigned short)FPM_MRF_filter(&b->mrf, in, b->buf1, (short)n);
  b->sre.loopEnable = b->sreGate & detect;
  cnt = (unsigned short)FAXSRE_recover(&b->sre, (const unsigned short*)b->buf1, b->buf2, (short)(unsigned short)produced);
  if ((unsigned short)cnt > 0xa4)
  {
    if (DBG)
      dsplibs_debug_printf("ERROR: SRE buffer violation!(%d)", (int)(unsigned short)cnt);
  }

  b->fse.predEnable = 0;
  b->fse.eqEnable = detect & b->eqGate;
  b->fse.pllEnable = detect & b->pllGate;
  return (int)(unsigned short)FAXFSE_receive(&b->fse, b->buf2, out, (unsigned short)cnt);
}

/* ============================================================ state machine */
/**
 * @brief Leave the current state: install the next state's handler and counter and update the status code and flags.
 * @param o  receiver
 * @note See the file header for the transitions.  Status byte flags: b[1] bit 0 = data, bit 5 = carrier, bit 1 = error; b[2] bit 0 = idle.
 */
NOINL void RxNextStateV17(V17Rx* o)
{
  V17RxHdx* h = o->hdx;
  switch (h->state)
  {
  case V17RX_STATE_START:
    if (DBG)
      dsplibs_debug_printf("V17RX_STATE_START\n");
    h->count = 5;
    /* keeps the short stores of count / state apart (the optimiser would merge them into one dword store with strict aliasing: compared on the objects built with and without the flag) */
    COMPILER_BARRIER();
    h->fn = RxHdxEpochDetV17;
    h->state = V17RX_STATE_EPOCH_DET;
    o->st.b[2] &= 0xfe;
    o->st.b[1] &= 0xfe;
    return;
  case V17RX_STATE_EPOCH_DET:
  {
    V17RxBundle* b;
    short cnt = 0x3e;
    if (DBG)
      dsplibs_debug_printf("V17RX_STATE_EPOCH_DET\n");
    h = o->hdx;
    if (h->flag != 0)
    {
      Restore_rateV17(o);
      /* the flag is read again behind the call (its writes are not visible to the optimiser the same way with and without strict aliasing) */
      COMPILER_BARRIER();
      h = o->hdx;
      if (h->flag != 0)
        cnt = 1;
    }
    h->count = cnt;
    COMPILER_BARRIER();
    b = o->bundle;
    h->fn = RxHdxPrtcolV17;
    h->state = V17RX_STATE_PROTOCOL;
    o->st.b[2] &= 0xfe;
    o->st.b[1] &= 0xfe;
    b->agc.alpha = b->agc.alpha + 1;
    b->agc.beta = b->agc.beta + 1;
    return;
  }
  case V17RX_STATE_PROTOCOL:
    if (DBG)
      dsplibs_debug_printf("V17RX_STATE_PROTOCOL\n");
    h = o->hdx;
    if (h->flag == 0)
    {
      h->count = 1;
      COMPILER_BARRIER();
      h->fn = RxHdxBridgeV17;
      h->state = V17RX_STATE_BRIDGE;
      o->st.b[1] &= 0xfe;
      StoreCoefV17(o);
      COMPILER_BARRIER(); /* the byte store behind the call stays behind it (it would be merged with the one in front of it with strict aliasing) */
      o->st.b[2] &= 0xfe;
      return;
    }
    h->count = 1;
    COMPILER_BARRIER();
    h->fn = RxHdxScramV17;
    h->state = V17RX_STATE_SCRAM;
    o->st.b[1] &= 0xfe;
    return;
  case V17RX_STATE_BRIDGE:
    if (DBG)
      dsplibs_debug_printf("V17RX_STATE_BRIDGE\n");
    h = o->hdx;
    h->count = 1;
    COMPILER_BARRIER();
    h->fn = RxHdxScramV17;
    h->state = V17RX_STATE_SCRAM;
    o->st.b[2] &= 0xfe;
    o->st.b[1] &= 0xfe;
    return;
  case V17RX_STATE_SCRAM:
    if (DBG)
      dsplibs_debug_printf("V17RX_STATE_SCRAM\n");
    FPM_AGC_Freeze(&o->bundle->agc);
    h = o->hdx;
    h->count = 0;
    COMPILER_BARRIER();
    h->fn = RxHdxDataV17;
    h->state = V17RX_STATE_DATA;
    o->st.b[2] &= 0xfe;
    o->st.b[1] |= 1;
    return;
  case V17RX_STATE_DATA:
    if (DBG)
      dsplibs_debug_printf("V17RX_STATE_DATA\n");
    h = o->hdx;
    h->fn = RxHdxIdleV17;
    h->state = V17RX_STATE_IDLE;
    h->count = 0;
    h->hold = 0;
    o->st.b[2] |= 1;
    o->st.b[1] &= 0xfe;
    return;
  case V17RX_STATE_IDLE:
  {
    unsigned short rate;
    if (DBG)
      dsplibs_debug_printf("V17RX_STATE_IDLE\n");
    h = o->hdx;
    h->fn = RxHdxDataV17;
    h->state = V17RX_STATE_DATA;
    o->st.b[2] &= 0xfe;
    o->st.b[1] |= 1;
    COMPILER_BARRIER();
    rate = (unsigned short)h->rate;
    if (rate == V17RC_7200)
      o->st.b[0] = V17RX_MSG_CONNECT_7200;
    else if (rate == V17RC_9600)
      o->st.b[0] = V17RX_MSG_CONNECT_9600;
    else
      o->st.b[0] = (unsigned char)(6 + (rate == V17RC_12000));
    return;
  }
  default:
  {
    unsigned char f;
    if (DBG)
      dsplibs_debug_printf("V17RX_DEFAULT: %d\n", (int)h->state);
    f = o->st.b[1];
    o->st.b[2] &= 0xfe;
    o->st.b[0] = V17RX_MSG_ERROR;
    f = (unsigned char)(f | 2);
    f = (unsigned char)(f & 0xde);
    o->st.b[1] = f;
    return;
  }
  }
}

/**
 * @brief Handler of the start state: wait for the carrier.
 * @param o    receiver
 * @param in   input samples
 * @param out  decoded symbols (unused: nothing is produced in this state)
 * @param n    in: number of samples; out: 0 (the block is always consumed)
 * @return 0
 * @note Runs the signal path for the AGC and tone detector, then leaves for epoch detect when CarrierDetectV17() reports a carrier.
 */
NOINL int RxHdxStartV17(V17Rx* o, short* in, short* out, unsigned short* n)
{
  const unsigned short n0 = *n; /* every input is read before the first store (aliasing independent code) */
  o->st.b[1] &= 0xdf;
  o->st.b[0] = V17RX_MSG_START;
  DemodDataV17(o, in, out, n0);
  if (CarrierDetectV17(o) == 0)
  {
    *n = 0;
    return 0;
  }
  o->st.b[1] |= 0x20;
  RxNextStateV17(o);
  *n = 0;
  return 0;
}

/* common exit of the training handlers on loss of carrier: handler = error, state 7, status code "loss of carrier", error flag set and carrier flag cleared */
static void to_error(V17Rx* o)
{
  V17RxHdx* h = o->hdx;
  h->fn = RxHdxErrorV17;
  h->state = V17RX_STATE_ERROR;
  o->st.b[0] = V17RX_MSG_LOSS_CARRIER;
  o->st.b[1] = (unsigned char)((o->st.b[1] | 2) & 0xdf);
}

/**
 * @brief Handler of the epoch-detect state: wait (up to hdx.count blocks) for the equalizer training to start.
 * @param o    receiver
 * @param in   input samples
 * @param out  scratch for the signal path (no data is returned)
 * @param n    in: number of samples; out: 0
 * @return 0
 * @note Loss of carrier goes to the error state; otherwise the state is left when the count expires or EpochDetectV17() fires.
 */
NOINL int RxHdxEpochDetV17(V17Rx* o, short* in, short* out, unsigned short* n)
{
  V17RxHdx* h;
  unsigned short cnt;
  DemodDataV17(o, in, out, *n);
  *n = 0;
  if (CarrierDetectV17(o) != 0)
  {
    o->st.b[1] |= 0x20;
    h = o->hdx;
    o->st.b[0] = V17RX_MSG_PROTOCOL;
    cnt = (unsigned short)(h->count - 1);
    h->count = (short)cnt;
    if ((short)cnt > 0)
    {
      if ((unsigned short)EpochDetectV17(o) == 0)
        return 0;
    }
    RxNextStateV17(o);
    return 0;
  }
  to_error(o);
  return 0;
}

/**
 * @brief Handler of the protocol (long-training) state: demodulate and descramble the training segment.
 * @param o    receiver
 * @param in   input samples
 * @param out  decoded symbols / bits
 * @param n    in: number of samples; out: 0
 * @return number of symbols produced when the state ends in this call, else 0
 * @note Loss of carrier goes to the error state.  When the block count runs out the SNR is checked (low-SNR flag at 8 or less) and RxNextStateV17() runs.
 */
NOINL int RxHdxPrtcolV17(V17Rx* o, short* in, short* out, unsigned short* n)
{
  V17RxHdx* h;
  short snr;
  unsigned int ret = (unsigned short)DemodDataV17(o, in, out, *n);
  DescrambleDataV17(o, (unsigned short*)out, (unsigned short)ret);
  *n = 0;
  if (CarrierDetectV17(o) != 0)
  {
    unsigned short cnt;
    o->st.b[1] |= 0x20;
    h = o->hdx;
    o->st.b[0] = V17RX_MSG_PROTOCOL;
    cnt = (unsigned short)(h->count - 1);
    h->count = (short)cnt;
    if ((short)cnt > 0)
      return 0;
    snr = (short)GetSNRV17(o);
    COMPILER_BARRIER();
    if (snr <= 8)
      o->st.b[1] |= 0x80;
    RxNextStateV17(o);
    return (short)ret;
  }
  to_error(o);
  return 0;
}

/**
 * @brief Handler of the bridge state (long training): same processing as RxHdxPrtcolV17().
 * @param o    receiver
 * @param in   input samples
 * @param out  decoded symbols / bits
 * @param n    in: number of samples; out: 0
 * @return number of symbols produced when the state ends in this call, else 0
 */
NOINL int RxHdxBridgeV17(V17Rx* o, short* in, short* out, unsigned short* n)
{
  V17RxHdx* h;
  short snr;
  unsigned int ret = (unsigned short)DemodDataV17(o, in, out, *n);
  DescrambleDataV17(o, (unsigned short*)out, (unsigned short)ret);
  *n = 0;
  if (CarrierDetectV17(o) != 0)
  {
    unsigned short cnt;
    o->st.b[1] |= 0x20;
    h = o->hdx;
    o->st.b[0] = V17RX_MSG_PROTOCOL;
    cnt = (unsigned short)(h->count - 1);
    h->count = (short)cnt;
    if ((short)cnt > 0)
      return 0;
    snr = (short)GetSNRV17(o);
    COMPILER_BARRIER();
    if (snr <= 8)
      o->st.b[1] |= 0x80;
    RxNextStateV17(o);
    return (short)ret;
  }
  to_error(o);
  return 0;
}

/**
 * @brief Handler of the scrambler-1 (SCRAM) state: demodulate and descramble; at the end of the state set the connect code for the rate.
 * @param o    receiver
 * @param in   input samples
 * @param out  decoded symbols / bits
 * @param n    in: number of samples; out: 0
 * @return number of symbols produced when the state ends in this call, else 0
 */
NOINL int RxHdxScramV17(V17Rx* o, short* in, short* out, unsigned short* n)
{
  V17RxHdx* h;
  short snr;
  unsigned int ret = (unsigned short)DemodDataV17(o, in, out, *n);
  DescrambleDataV17(o, (unsigned short*)out, (unsigned short)ret);
  *n = 0;
  if (CarrierDetectV17(o) != 0)
  {
    unsigned short cnt, rate;
    o->st.b[1] |= 0x20;
    h = o->hdx;
    o->st.b[0] = V17RX_MSG_PROTOCOL;
    cnt = (unsigned short)(h->count - 1);
    h->count = (short)cnt;
    if ((short)cnt > 0)
      return 0;
    rate = (unsigned short)h->rate;
    if (rate == V17RC_7200)
      o->st.b[0] = V17RX_MSG_CONNECT_7200;
    else if (rate == V17RC_9600)
      o->st.b[0] = V17RX_MSG_CONNECT_9600;
    else
      o->st.b[0] = (unsigned char)(6 + (rate == V17RC_12000));
    snr = (short)GetSNRV17(o);
    COMPILER_BARRIER();
    if (snr <= 8)
      o->st.b[1] |= 0x80;
    RxNextStateV17(o);
    return (short)ret;
  }
  to_error(o);
  return 0;
}

/**
 * @brief Handler of the data state: demodulate and descramble one block.
 * @param o    receiver
 * @param in   input samples
 * @param out  decoded symbols / bits
 * @param n    in: number of samples; out: 0
 * @return number of symbols produced; 0 when the carrier is lost, the receiver is on hold or QualityDetectV17() flags the block unreliable
 * @note Sets the low-SNR flag (b[1] bit 7) when GetSNRV17() is 8 or less.
 */
NOINL int RxHdxDataV17(V17Rx* o, short* in, short* out, unsigned short* n)
{
  unsigned int count;
  int ret;
  short snr;
  const unsigned short n0 = *n;
  o->st.b[1] |= 0x20;
  o->st.b[0] = V17RX_MSG_DATA;
  COMPILER_BARRIER();
  if ((short)DataCarrierDetectV17(o, in, n0) == 0 || o->hdx->hold != 0)
  {
    o->st.b[1] &= 0xdf;
    *n = 0;
    return 0;
  }
  count = (unsigned short)DemodDataV17(o, in, out, *n);
  DescrambleDataV17(o, (unsigned short*)out, (unsigned short)count);
  *n = 0;
  ret = ((short)QualityDetectV17(o) != 2) ? (int)count : 0;
  o->st.b[1] &= 0x7f;
  snr = (short)GetSNRV17(o);
  COMPILER_BARRIER();
  if (snr <= 8)
    o->st.b[1] |= 0x80;
  return (short)ret;
}

/**
 * @brief Handler of the idle state (decision quality lost): keep demodulating until the decision error is small again.
 * @param o    receiver
 * @param in   input samples
 * @param out  scratch for the signal path
 * @param n    in: number of samples; out: 0
 * @return 0
 * @note Returns to the data state when the carrier is present and fse.decErrAvg is at most 0x1fff.
 */
NOINL int RxHdxIdleV17(V17Rx* o, short* in, short* out, unsigned short* n)
{
  DemodDataV17(o, in, out, *n);
  *n = 0;
  COMPILER_BARRIER();
  o->st.b[1] &= 0xdf;
  o->st.b[0] = V17RX_MSG_IDLE;
  if (CarrierDetectV17(o) != 0)
    o->st.b[1] |= 0x20;
  if (o->st.b[1] & 0x20)
  {
    if (o->bundle->fse.decErrAvg <= 0x1fff)
    {
      RxNextStateV17(o);
      if (DBG)
        dsplibs_debug_printf("Decision error is small back to DATA mode !!!\n");
    }
  }
  return 0;
}

/**
 * @brief Handler of the error state: demodulate and discard.
 * @param o    receiver
 * @param in   input samples
 * @param out  scratch for the signal path
 * @param n    in: number of samples; out: 0
 * @return 0
 */
NOINL int RxHdxErrorV17(V17Rx* o, short* in, short* out, unsigned short* n)
{
  const unsigned short n0 = *n;
  o->st.b[1] |= 2;
  DemodDataV17(o, in, out, n0);
  *n = 0;
  return 0;
}

/* ============================================================ create / delete / modem / control / status */
/* set up the Viterbi decoder family of a rate (16T / 32 / 64 / 128 point trellis, as VTBv32_init() does; any rate above 2 takes the last)
 * with the V.17 maps of fax_v17_tx_tables.c */
static void vtb_setup(FaxVtb* v, int rate)
{
  v->wr = 0;
  v->prevQuad = 0;
  v->depth = 0x10;
  if (rate == V17RC_9600)
  {
    v->sel = 2;
    v->imap = CFGP(VTBv17_IMAP32);
    v->qmap = CFGP(VTBv17_QMAP32);
    v->bound = CFGP(VTB_BOUND_9600);
    v->region = CFGP(VTB_REGION_9600);
    v->size = 4;
    v->lowMask = 0xf;
  }
  else if (rate == V17RC_7200)
  {
    v->sel = 1;
    v->imap = CFGP(VTBv17_IMAP16T);
    v->qmap = CFGP(VTBv17_QMAP16T);
    v->bound = CFGP(VTB_BOUND_7200);
    v->region = CFGP(VTB_REGION_7200);
    v->size = 2;
    v->lowMask = 7;
  }
  else if (rate == V17RC_12000)
  {
    v->sel = 3;
    v->imap = CFGP(VTBv17_IMAP64);
    v->qmap = CFGP(VTBv17_QMAP64);
    v->bound = CFGP(VTB_BOUND_12000);
    v->region = CFGP(VTB_REGION_12000);
    v->size = 6;
    v->lowMask = 0x1f;
  }
  else
  {
    v->sel = 4;
    v->imap = CFGP(VTBv17_IMAP128);
    v->qmap = CFGP(VTBv17_QMAP128);
    v->bound = CFGP(VTB_BOUND_14400);
    v->region = CFGP(VTB_REGION_14400);
    v->size = 8;
    v->lowMask = 0x3f;
  }
}

/* the default configuration V17RX_CFG as a typed block (the caller buffers are NULL: V17RX_CFG words 6 .. 8 are zero) */
static void cfg_default(V17RxCfg* c)
{
  int i;
  for (i = 0; i < 6; i++)
    c->w[i] = V17RX_CFG[i];
  c->coefI = NULL;
  c->coefQ = NULL;
  c->freq = NULL;
  c->user = V17RX_CFG[9];
}

/**
 * @brief Create or re-initialise a V.17 receiver.
 * @param o    existing receiver to re-initialise in place, or NULL to allocate the instance, hdx, bundle and buffers
 * @param cfg  configuration (copied), or NULL for the defaults (V17RX_CFG); `cfg` may be `&o->cfg`
 * @return the receiver, in the start state with status code V17RX_MSG_START
 * @warning The allocation results are not checked.  With a non-zero training flag `cfg->coefI`, `coefQ` and `freq` must point to 0x31, 0x31 and 1 shorts.
 * @bug A rate other than 7200 / 9600 / 12000 / 14400 selects 14400 and sets the error status, but the status word is cleared again further down, so the
 *   configuration error is never visible to the caller.
 */
V17Rx* V17RX_create(V17Rx* o, const V17RxCfg* cfg)
{
  int objFresh = 0, hdxFresh = 0;
  V17RxHdx* h;
  V17RxBundle* b;
  FaxDec17* dec;
  int i, rate, flag;
  int mtdCfg[3], toneCfg[9], mrfCfg[4];
  FaxSreCfg sre;
  FaxFseCfg fse;
  FaxSdmCfg sdmCfg;
  FaxSgdCfg sgdCfg;
  FaxSgd* oldSgd;
  V17RxCfg cc;
  int user;

  if (DBG)
    dsplibs_debug_printf("V.17 RX Create ");
  if (o == NULL)
  {
    if (DBG)
      dsplibs_debug_printf("New allocation\n");
    o = (V17Rx*)sysdep_malloc((unsigned int)sizeof(V17Rx));
    o->hdx = NULL;
    o->bundle = NULL;
    objFresh = 1;
  }
  if (DBG)
    dsplibs_debug_printf("\n");
  if (cfg == NULL)
    cfg_default(&cc);
  else
    cc = *cfg; /* the configuration is used below from this copy (aliasing independent code: create(o, &o->cfg) and no reload after the stores) */
  o->cfg = cc;

  /* half-duplex block: allocate on first use, then build the tone detector and tone killer of the start state */
  flag = cc.w[5];
  user = cc.user;
  h = o->hdx;
  if (h == NULL)
  {
    h = (V17RxHdx*)sysdep_malloc((unsigned int)sizeof(V17RxHdx));
    o->hdx = h;
    h->mtd = NULL;
    h->tone = NULL;
    h->buf = (short*)sysdep_malloc(0x140);
    h->buf2 = (short*)sysdep_malloc(0x140);
    h->mtd2 = NULL;
    h->s20 = 0;
    hdxFresh = 1;
  }
  /* tone detector of the start state: V17_MTD_COEFF, {2, 0x4ccd}, FPM_MTD_CFG[2] low half 0x64 */
  mtdCfg[0] = CFGP(V17_MTD_COEFF);
  mtdCfg[1] = (int)(0x4ccdu << 16 | 2u);
  mtdCfg[2] = (int)(((unsigned int)FPM_MTD_CFG_raw[2] & 0xffff0000u) | 0x64u);
  h->mtd = FPM_MTD_create(h->mtd, mtdCfg);
  /* tone killer: FPM_TONE_CFG with the low half of word 0 = 0x708 */
  for (i = 0; i < 9; i++)
    toneCfg[i] = FPM_TONE_CFG_raw[i];
  toneCfg[0] = (int)(((unsigned int)toneCfg[0] & 0xffff0000u) | 0x708u);
  h->tone = FPM_TONE_create(h->tone, toneCfg);
  COMPILER_BARRIER(); /* the hdx pointer is re-read behind the store (the load is moved in front of it with strict aliasing only) */
  h = o->hdx;

  /* start state, and the carrier detector (V.21 channel 2 tone detector with its own AGC) */
  h->state = V17RX_STATE_START;
  h->count = 0;
  h->hold = 0;
  h->fn = RxHdxStartV17;
  h->flag = flag;
  /* carrier detector: V21_CHAN2_MTD_COEFF, {2, 0x4ccd}, FPM_MTD_CFG[2] low half 0x12c */
  mtdCfg[0] = CFGP(V21_CHAN2_MTD_COEFF);
  mtdCfg[1] = (int)(0x4ccdu << 16 | 2u);
  mtdCfg[2] = (int)(((unsigned int)FPM_MTD_CFG_raw[2] & 0xffff0000u) | 0x12cu);
  h->mtd2 = FPM_MTD_create(h->mtd2, mtdCfg);
  FPM_AGC_init(&h->agc, AGCv17_CFG, hdxFresh);
  h->sinceTone = 0;
  h->toneSearch = 0;

  /* rate index from the configured baud rate */
  if ((short)cc.s[2] == RATE_9600)
  {
    rate = V17RC_9600;
  }
  else if ((short)cc.s[2] == RATE_7200)
  {
    rate = V17RC_7200;
  }
  else if ((short)cc.s[2] == RATE_12000)
  {
    rate = V17RC_12000;
  }
  else if ((short)cc.s[2] == RATE_14400)
  {
    rate = V17RC_14400;
  }
  else
  {
    rate = V17RC_14400;
    o->st.b[1] |= 2;
    o->st.b[0] = V17RX_MSG_ERROR;
  }
  h->rate = (short)rate;

  /* signal path: bundle and its two sample buffers on first use, then matched filter and AGC */
  b = o->bundle;
  if (b == NULL)
  {
    b = (V17RxBundle*)sysdep_malloc((unsigned int)sizeof(V17RxBundle));
    o->bundle = b;
    b->buf1 = (short*)sysdep_malloc(0x140);
    b->buf2 = (short*)sysdep_malloc(0x148);
  }
  /* matched filter: up 9, down 10, 0x168 taps */
  mrfCfg[0] = (int)(9u | (0xau << 16));
  mrfCfg[1] = CFGP(MRFv17_COFFS);
  mrfCfg[2] = (int)(((unsigned int)FPM_MRF_CFG_raw[2] & 0xffff0000u) | 0x168u);
  mrfCfg[3] = user;
  FPM_MRF_init(&b->mrf, mrfCfg, objFresh);
  FPM_AGC_init(&b->agc, AGCv17_CFG, objFresh);

  /* symbol timing recovery: the start count and the proportional PLL gain table depend on the training flag */
  sre = fax_sre_cfg_default;
  COMPILER_BARRIER();
  sre.sps = 3;
  sre.periodsTrack = 3;
  sre.periodsLock = 0x10;
  sre.intGainTab = CFGP(SREv17_PLL_K2);
  if (flag == 0)
  {
    sre.propGainTab = CFGP(SREv17_PLL_K1);
    sre.startCount = 0x55;
  }
  else
  {
    sre.startCount = 0x30;
    sre.propGainTab = CFGP(SREv17_PLL_K1_S);
  }
  sre.corrSinTab = CFGP(SREv17_yCLOCK);
  sre.nt = 0xb4;
  sre.errOn = 2;
  sre.coefTab = CFGP(SREv17_COFFS);
  sre.tedCoef = CFGP(SREv17_XB_COFFS);
  sre.corrCosTab = CFGP(SREv17_xCLOCK);
  sre.errOff = 1;
  sre.unlockThresh = 0x2666;
  sre.lockThresh = 0xc8;
  sre.unlockGain = 0x2000;
  sre.lockGain = 0x4000;
  sre.levelThresh = (short)((int)b->agc.a.s[0] / 6);
  sre.levelLen = 9;
  sre.w34 = user;
  FAXSRE_init(&b->sre, &sre, objFresh);
  b->sre.ppmTick = 0x30;
  b->sre.ppmPeriod = 0x2580;
  b->sre.ppmStep = (short)(1000000 / ((int)sre.sps * 0x2580));
  b->sre.ppmNMax = 0x68;

  /* equalizer: settle count and carrier PLL gain table depend on the training flag; with short training the start taps are the caller's */
  fse = fax_fse_cfg_default;
  COMPILER_BARRIER();
  fse.blockLen = 0x90;
  fse.sps = 3;
  fse.iCoefTab = CFGP(FSEv17_ICOFF);
  fse.qCoefTab = CFGP(FSEv17_QCOFF);
  if (flag == 0)
  {
    fse.settleCount = 0x5dc;
    fse.propTab = CFGP(CRRv17_PLL_K1);
  }
  else
  {
    fse.settleCount = 0x100;
    fse.propTab = CFGP(CRRv17_PLL_K1_S);
  }
  fse.nt = 0x31;
  fse.mu[0] = 0;
  fse.mu[1] = 0;
  fse.carrierTab = CFGP(CRRv17_CLK);
  fse.crrLen = 4;
  fse.crrAdj = 1;
  fse.errThreshHi = 0x199a;
  fse.errThreshLo = 0x666;
  fse.intTab = CFGP(CRRv17_PLL_K2);
  fse.hist = &b->dec;
  fse.cb = FAX_FSE_decision_AB;
  fse.w34 = user;
  FAXFSE_init(&b->fse, &fse, objFresh);
  if (flag != 0)
  {
    short *const ti = b->fse.tapsI, *const tq = b->fse.tapsQ;
    const short *const ci = cc.coefI, *const cq = cc.coefQ;
    for (i = 0; i < 0x31; i++)
    {
      ti[i] = ci[i];
      tq[i] = cq[i];
    }
    b->fse.cfg.iCoefTab = cfgp_word(ci); /* a table handle on 64-bit builds (0 for a caller pointer that is not a table; the field is not read again) */
    b->fse.cfg.qCoefTab = cfgp_word(cq);
  }

  /* the history object of the FSE callbacks (FaxDec17) and its Viterbi decoder */
  dec = &b->dec;
  dec->ctr = 0;
  dec->s66 = 3;
  dec->sel = (short)rate; /* ============================================================ h->rate / h->flag */
  dec->flag = flag;
  dec->cnt = 0;
  dec->reg = 0;
  dec->prevAngle = 0;
  dec->accA = 0;
  dec->accB = 0;
  dec->w50 = 0;
  dec->s54 = 0;
  dec->s56 = 0;
  dec->abI0 = dec->abQ0 = dec->abI1 = dec->abQ1 = dec->abI2 = dec->abQ2 = 0;
  if (objFresh != 0)
    dec->vtb.hist = (short*)sysdep_malloc(0x200);
  {
    short* const hist = dec->vtb.hist;
    for (i = 0; i < 0x80; i++)
    {
      hist[i * 2 + 1] = 0;
      hist[i * 2] = 0;
    }
  }
  vtb_setup(&dec->vtb, rate);
  dec->vtb.shift = dec->vtb.sel;
  dec->vtb.metric[0] = 0;
  for (i = 1; i < 8; i++)
    dec->vtb.metric[i] = 0;
  oldSgd = objFresh != 0 ? NULL : dec->sgd;
  sgdCfg = SGD_CFG;
  sgdCfg.bits = 2;
  sgdCfg.w24 = (int)(((unsigned int)sgdCfg.w24 & 0xffffu) | (0x2000u << 16));
  sgdCfg.w28 = 0x111;
  sgdCfg.w2c = 0xffff;
  dec->sgd = SGD_create(oldSgd, &sgdCfg);

  /* receive scrambler: bits per symbol = rate index + 3, taps 0x12 / 0x17 */
  sdmCfg.s[0] = (short)(rate + 3);
  sdmCfg.s[1] = 0x12;
  sdmCfg.s[2] = 0x17;
  SDM_init(&b->sdm, &sdmCfg);

  /* clear the sample buffers, reset the quality tracker and the gates */
  {
    short *const buf1 = b->buf1, *const buf2 = b->buf2;
    for (i = 0; i < 0xa0; i++)
    {
      buf1[i] = 0;
      buf2[i] = 0;
    }
  }

  b->decErrSmooth = 0;
  b->qualBlocks = 0;
  b->qualityOk = 0;
  if (rate == V17RC_9600)
    b->errLimit = 0x514;
  else if (rate == V17RC_7200)
    b->errLimit = 0xa28;
  else if (rate == V17RC_12000)
    b->errLimit = 0x341;
  else if (rate == V17RC_14400)
    b->errLimit = 0x1c2;
  b->w0 = 1;
  b->energyTest = 1;
  b->refBlocks = 0;
  b->refLevel = 0;
  b->sreGate = 1;
  b->pllGate = 1;
  b->wc = 0;
  b->rate24 = (short)rate;
  b->w14 = 0;
  b->w18 = 1;
  b->w1c = 1;
  b->w20 = 0;
  b->eqGate = 1;
  b->w28 = 0;

  /* status word and the equalizer buffer pointers published in the instance */
  o->st.w = 0;
  o->st.b[1] |= 0x50;
  o->st.b[0] = V17RX_MSG_START;
  COMPILER_BARRIER();
  o->fseI = b->fse.outI;
  o->fseQ = b->fse.outQ;
  o->fseIdx = &b->fse.idx;
  o->fseNt = b->fse.cfg.nt;
  o->fseTapsI = b->fse.tapsI;
  o->w44 = 0;
  o->fseTapsQ = b->fse.tapsQ;
  o->w48 = 0;
  o->s4c = 0;
  o->w50 = 0;
  o->w54 = 0;
  o->s58 = 0;
  return o;
}

/**
 * @brief Release a receiver and everything it owns (sub-objects, history ring, SGD, buffers, hdx, bundle, instance).
 * @param o  receiver created by V17RX_create()
 */
void V17RX_delete(V17Rx* o)
{
  V17RxBundle* b = o->bundle;
  V17RxHdx* h;
  SGD_delete(b->dec.sgd);
  sysdep_free(o->bundle->dec.vtb.hist);
  FAXFSE_free(&o->bundle->fse);
  FAXSRE_free(&o->bundle->sre);
  FPM_MRF_free(&o->bundle->mrf);
  sysdep_free(o->bundle->buf2);
  sysdep_free(o->bundle->buf1);
  sysdep_free(o->bundle);
  h = o->hdx;
  FPM_MTD_delete(h->mtd);
  FPM_TONE_delete(o->hdx->tone);
  sysdep_free(o->hdx->buf);
  sysdep_free(o->hdx->buf2);
  FPM_MTD_delete(o->hdx->mtd2);
  sysdep_free(o->hdx);
  sysdep_free(o);
}

/**
 * @brief Process one block of samples.
 * @param o    receiver
 * @param in   input samples
 * @param out  receives the decoded symbols / bits (as many as the handlers produce)
 * @param n    in: number of input samples; out: number of output symbols
 * @return the status word `o->st.w` (see V17Rx::st)
 * @note The state handlers run in a loop until they have consumed the block, so the state can change several times in one call; the error flag (b[1] bit 1)
 *   is cleared first.
 */
int V17RX_modem(V17Rx* o, short* in, short* out, unsigned short* n)
{
  short total = 0;
  unsigned short cur = *n; /* read before the first store (aliasing independent code) */
  o->st.b[1] &= 0xfd;
  do
  {
    int before = (short)cur;
    int ret = o->hdx->fn(o, in, out, n);
    cur = *n;
    out = out + (short)ret;
    in = in + (before - (int)cur);
    total = (short)((short)ret + total);
  } while (cur != 0);
  *n = (unsigned short)total;
  return o->st.w;
}

/**
 * @brief Apply a control record.
 * @param o    receiver
 * @param ctl  control record, or NULL
 * @return 0 for NULL, else 1
 * @note `w4` becomes cfg.w[2]; byte 0xd bit 4 sets the hold flag, bit 1 re-creates the receiver with `w10` as the training flag; byte 0xc bit 3 clears
 *   bundle->w0 and bit 5 clears the equalizer gate (eqGate).
 */
int V17RX_control(V17Rx* o, const FaxV17Ctl* ctl)
{
  unsigned char d, c;
  int w4, w10;
  V17RxHdx* h;
  if (ctl == NULL)
    return 0;
  w4 = ctl->w4; /* the control record is read before any store (aliasing independent code) */
  d = ctl->b0d;
  w10 = ctl->w10;
  COMPILER_BARRIER();
  o->cfg.w[2] = w4;
  h = o->hdx;
  h->hold = (d >> 4) & 1;
  if (d & 2)
  {
    o->cfg.w[5] = w10;
    V17RX_create(o, &o->cfg);
  }
  c = ctl->b0c;
  if (c & 8)
    o->bundle->w0 = 0;
  if (c & 0x20)
    o->bundle->eqGate = 0;
  return 1;
}

/**
 * @brief Fill the status record.
 * @param o   receiver
 * @param st  record to fill (only part of it is written, see below), or NULL
 * @return 0 for NULL, else 1
 * @note s0 = cfg.s[0], s4 = s12 = cfg.s[2] (baud rate), s6 = 1 unless the low-SNR flag is set, s8 = GetSNRV17(), s2 / sa / se / s10 = 0, b14 = flags from
 *   bundle words w1c / w0 / eqGate, b15 bit 0 cleared; sc and w18 are not written.
 */
int V17RX_status(V17Rx* o, FaxV17Status* st)
{
  V17RxBundle* b;
  unsigned char b14;
  short cfg0, cfg2, snr, ok;
  int w0, w1c, eqGate;
  if (st == NULL)
    return 0;
  /* every member of the engine is read before the first store into the record (aliasing independent code) */
  cfg0 = o->cfg.s[0];
  cfg2 = o->cfg.s[2];
  ok = (o->st.b[1] & 0x80) == 0;
  snr = (short)GetSNRV17(o);
  b = o->bundle;
  w0 = b->w0;
  w1c = b->w1c;
  eqGate = b->eqGate;
  b14 = (unsigned char)(((w1c & 1) << 1) | ((w0 == 0) << 3) | 0x10 | ((eqGate == 0) << 5) | 0x40); /* every other bit is cleared */
  COMPILER_BARRIER();
  st->s0 = cfg0;
  st->s2 = 0;
  st->s4 = cfg2;
  st->s6 = ok;
  st->s8 = snr;
  st->sa = 0;
  st->se = 0;
  st->s10 = 0;
  st->s12 = cfg2;
  st->b15 &= 0xfe;
  st->b14 = b14;
  return 1;
}

/* ============================================================ VMI wrappers */
/**
 * @brief Initialise a VMI object as a V.17 receive modem.
 * @param m     VMI object to fill (the first 0x18 bytes are written)
 * @param rate  baud rate (7200 / 9600 / 12000 / 14400)
 * @param unused  ignored
 * @param user  stored as the engine slot (v17rx_create() replaces it with the engine) and as the configuration's user word
 * @return 0x80
 * @warning The three equalizer buffers (0x62, 0x62 and 2 bytes) and the configuration block are allocated unchecked and are not freed by v17rx_delete().
 */
int fax_init_vmi_v17rx(FaxVmiV17* m, unsigned short rate, int unused, void* user)
{
  V17RxCfg* c = (V17RxCfg*)sysdep_malloc(sizeof(V17RxCfg));
  int i;
  (void)unused;
  if (2 < dsplibs_debug_level)
    dsplibs_debug_printf("Initializing VMI_V17_RX Modem No ECM (Simple Packing)\n");
  for (i = 0; i < 6; i++)
    c->w[i] = V17RX_CFG[i];
  c->user = (int)(uintptr_t)user; /* 32-bit copy of the user word (only ever copied on into the MRF / SRE / FSE configurations) */
  c->s[2] = (short)rate;
  c->w[5] = 0;
  c->coefI = (short*)sysdep_malloc(0x62); /* the three buffers of the short training (StoreCoefV17() / Restore_rateV17() / the FSE start taps) */
  c->coefQ = (short*)sysdep_malloc(0x62);
  c->freq = (short*)sysdep_malloc(2);

  /* the VMI view: word 0 and the shorts at +8 .. +0xe, the engine slot and the configuration slot */
  m->w0 = 0;
  m->s08 = 0x140;
  m->s0a = 0xa5;
  m->s0c = 0;
  m->s0e = 0xc;
  m->eng = (struct V17Tx*)user; /* FaxVmiV17::eng is typed for the transmitter: the receiver stores its V17Rx * through this cast */
  m->u4.init1 = 1;
  m->u10.cfg = (V17TxCfg*)c; /* likewise: the receiver's V17RxCfg block */
  return 0x80;
}

/**
 * @brief VMI create: build the engine and store it in the VMI object.
 * @param m    VMI object (its `eng` slot holds an existing engine to re-initialise, or the user word from fax_init_vmi_v17rx())
 * @param cfg  configuration, or NULL for the defaults
 * @return a rate code, not the engine: 4 for 9600, 3 for 7200 (and any other rate), and the configuration word 1 itself (the baud rate with its high half)
 *   for 12000 / 14400
 * @warning The return value for 12000 and 14400 bit/s is the configuration word, not a small code like the other rates (the values stored in `m->u10.s10` are
 *   5 and 6); a caller that tests the value against the small codes would misread it.
 */
int v17rx_create(FaxVmiV17* m, const V17RxCfg* cfg)
{
  V17RxCfg w;
  V17Rx* e;
  int r;
  if (cfg == NULL)
    cfg_default(&w);
  else
    w = *cfg;
  e = V17RX_create((V17Rx*)m->eng, &w);
  m->s0c = 0;
  m->eng = (struct V17Tx*)e;
  if ((unsigned short)w.s[2] == RATE_14400)
  {
    m->u10.s10 = 6;
    r = w.w[1];
  }
  else if ((unsigned short)w.s[2] == RATE_12000)
  {
    m->u10.s10 = 5;
    r = w.w[1];
  }
  else
  {
    r = 4 - ((unsigned short)w.s[2] != RATE_9600);
    m->u10.s10 = (short)r;
  }
  m->s0e = 0;
  return r;
}

/** @brief VMI delete: release the engine (V17RX_delete()). */
void v17rx_delete(FaxVmiV17* m) { V17RX_delete((V17Rx*)m->eng); }

/**
 * @brief VMI process: run V17RX_modem() into the VMI object's bit buffer.
 * @param m     VMI object
 * @param in    input samples
 * @param outn  receives the number of output symbols
 * @param n     in: number of input samples; out: 0
 * @return the receiver's status word
 */
int v17rx_process(FaxVmiV17* m, short* in, unsigned short* outn, unsigned short* n)
{
  int r = V17RX_modem((V17Rx*)m->eng, in, m->u4.bits, n);
  *outn = *n;
  *n = 0;
  return r;
}

/** @brief VMI status: V17RX_status() on the engine. */
int v17rx_status(FaxVmiV17* m, FaxV17Status* st) { return V17RX_status((V17Rx*)m->eng, st); }
/** @brief VMI control: V17RX_control() on the engine. */
int v17rx_control(FaxVmiV17* m, const FaxV17Ctl* ctl) { return V17RX_control((V17Rx*)m->eng, ctl); }

/**
 * @brief VMI message lookup.
 * @param m    VMI object (unused)
 * @param idx  status code, 0 .. 9
 * @param out  receives V17RX_MESG[idx], or NULL for idx 10 or more
 */
void v17rx_message(FaxVmiV17* m, int idx, const char** out)
{
  (void)m;
  if ((unsigned int)idx > 10u)
    *out = NULL;
  else if ((unsigned int)idx == 10u)
    *out = NULL; /* index 10 is one past V17RX_MESG: there is no string to return */
  else
    *out = V17RX_MESG[idx & 0xff];
}
