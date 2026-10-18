/**
 * @file fax_v27_rx.c
 * @brief Fax V.27ter receiver (2400 / 4800 bit/s, T.30 high-speed data): engine, half-duplex state machine, detectors, equalizer callbacks, VMI wrappers.
 *
 * Entry points: V27RX_create() / V27RX_delete() / V27RX_modem() / V27RX_control() / V27RX_status(), and the VMI wrappers v27rx_*() with the initialiser
 * fax_init_vmi_v27rx(), through which fax_vmi.c drives the receiver.  Typed objects are in fax_v27_rx.h, tables in fax_v27_rx_tables.c; the timing-recovery
 * and equalizer sub-objects are the FaxSre / FaxFse of fax_timing.c / fax_equalizer.c (shared with the V.29 and V.17 receivers).  The receive
 * descrambler DescrambleDataV27() lives in fax_modem_wrappers.c; on a 64-bit build this file defines the accessor faxw_v27_rxsdm() of fax_modem_views.h
 * that lets it reach `bundle->sdm`.
 *
 * Signal flow of one block: V27RX_modem() runs the handler of the current state (RxHdx*V27), each of which consumes the whole block and returns the number
 * of symbols produced.  The handlers call DemodDataV27(): AGC (in place) -> [state START only: tone detector, a detected tone aborts the block] -> matched
 * filter -> timing recovery -> carrier / equalizer / decision loop (the equalizer calls the decision callback of the current phase to decide each symbol)
 * -> DescrambleDataV27().
 *
 * States (V27RxHdx::state, advanced by RxNextStateV27()):
 * 0 START (wait for carrier) -> 1 EPOCH_DET (the callback V27RX_epoch_det() looks for the phase-reversal epoch, then switches itself to V27RX_eq_train())
 * -> 2 PROTOCOL (equalizer training: V27RX_eq_train(), then V27RX_decision()) -> 3 DATA -> 4 IDLE (decision quality lost; back to DATA when the error is
 * small again); 5 ERROR (carrier lost during training).
 *
 * Behaviour worth knowing (read from the code, not run unless stated at the function):
 * - RxNextStateV27() prints the name of the state it LEAVES (level > 1) before it acts; the transition 1 -> 2 advances the AGC alpha / beta table pointers by
 *   one entry (each table has two).
 * - The AGC output (agc.detect) gates the timing loop (sre.loopEnable = sreGate & detect) and the equalizer (pllEnable = pllGate & detect, eqEnable =
 *   eqGate & detect, predEnable = 0).
 * - The training length is 0x3e8 equalizer calls with the long training (cfg word 5 == 0), 0x32 with the short one.
 * - GetSNRV27() is the constant 10, so RxHdxDataV27() never sets the "SNR low" flag (b[1] bit 7).
 * - v27rx_create() returns the engine pointer (v29rx_create() returns a code); fax_init_vmi_v27rx() returns 0x80; v27rx_message(m, 8) returns NULL.
 */
#include <limits.h>
#include <string.h>
#include "fax_v27_rx.h"
#include "fax_v27_rx_tables.h"
#include "fax_v21_tables.h"
#include "fax_common_tables.h"
#include "fpm/fpm_tables.h"
#include "fax_modem_views.h"
#include "core/dsp_asm.h"

#include "fax_debug.h" /* dsplibs_debug_level, DBG, DBG3 */
#include "core/dsp_host.h"
#include "fpm/fpm.h"

/* NOINL keeps the order of the loads / stores around the detector functions the same with and without -fno-strict-aliasing (compared on the objects built with and without the flag) */
#include "fax_noinl.h"          /* NOINL */
#include "core/dsp_constants.h" /* RATE_* */

#if UINTPTR_MAX == 0xffffffffu
/* the raw-offset accessor of fax_modem_views.h reads exactly these members */
_Static_assert(offsetof(V27Rx, bundle) == 0x54 && offsetof(V27RxBundle, sdm) == 0x3c, "faxw_v27_rxsdm offsets");
#else
/** @brief Receive descrambler of a V.27ter receiver engine (the accessor DescrambleDataV27() uses on a 64-bit build). */
FaxSdmV27* faxw_v27_rxsdm(void* o) { return &((V27Rx*)o)->bundle->sdm; }
#endif

/* ===== FSE decision callbacks */
/**
 * @brief Equalizer decision callback of the EPOCH_DET state: look for the phase-reversal epoch that starts the training sequence.
 * @param f      equalizer; `f->cfg.hist` is the V27RxLoop, `f->outI` / `outQ` / `idx` give the newest equalized symbol
 * @param angle  unused
 * @param mag    unused
 * @return USHRT_MAX (no symbol code is produced while searching)
 *
 * Compares the distance between the last two symbol pairs (new pair and the pair before it) with the smoothed symbol energy.  For the first calls
 * (0x28 with the long training, 0xa with the short one) the energy average just follows the newest symbol; after that it is a one-pole average and a pair
 * distance above four times the average marks the epoch: the callback forces the equalizer update and switches the decision callback to V27RX_eq_train().
 * @note Every member is read before the first store, so the result does not depend on the aliasing rules (compared on the objects built with and without the flag).
 */
int V27RX_epoch_det(FaxFse* f, short* angle, short* mag)
{
  V27RxLoop* h = (V27RxLoop*)f->cfg.hist;
  const short *outI = f->outI, *outQ = f->outQ;
  const short idx = f->idx;
  const int thr = (h->shortTrain == 0) ? 0x28 : 0xa;
  const short cnt = h->cnt, sinceReset = h->phaseCnt, avg = h->avg;
  const short i1 = h->symI1, q1 = h->symQ1, i2 = h->symI2, q2 = h->symQ2, i0 = h->symI0, q0 = h->symQ0;
  const int i = outI[idx], q = outQ[idx];
  int distNew, distOld, distSum, energy;
  short navg = avg;
  int sw = 0;

  (void)angle;
  (void)mag;
  distNew = (int)((unsigned int)((int)(short)(i1 - i) * (int)(short)(i1 - i)) + (unsigned int)((int)(short)(q1 - q) * (int)(short)(q1 - q))) >> 15;
  distOld = (int)((unsigned int)((int)(short)(i2 - i0) * (int)(short)(i2 - i0)) + (unsigned int)((int)(short)(q2 - q0) * (int)(short)(q2 - q0))) >> 15;
  distSum = (short)(distNew + distOld);
  energy = (short)((int)((unsigned int)(i * i) + (unsigned int)(q * q)) >> 15);

  /* update the energy average; after the start-up calls test the pair distance against it */
  if (sinceReset > thr)
  {
    navg = (short)((((avg << 5) - avg) >> 5) + (energy >> 5));
    if (distSum > (int)navg * 4)
      sw = 1;
  }
  else
  {
    navg = (short)energy;
  }

  /* update the history */
  h->cnt = (short)(cnt + 1);
  h->symI2 = i1;
  h->symQ1 = q0;
  h->symQ2 = q1;
  h->symI1 = i0;
  h->symI0 = (short)i;
  h->symQ0 = (short)q;
  h->avg = navg;
  h->phaseCnt = sw ? (short)0 : (short)(sinceReset + 1);
  if (sw)
  {
    f->eqForce = 1;
    f->cfg.cb = V27RX_eq_train;
  }
  return USHRT_MAX;
}

/**
 * @brief Equalizer decision callback of the PROTOCOL state: supply the known training symbols while the equalizer converges.
 * @param f      equalizer; `f->cfg.hist` is the V27RxLoop
 * @param angle  in: angle of the received symbol; out: reference angle of the training symbol
 * @param mag    out: reference magnitude (0x3299)
 * @return USHRT_MAX (no symbol code is produced during training)
 *
 * The training sequence is a fixed-rate phase rotation: whenever the received angle moved more than 0x2000 from the previous reference angle (after
 * wrapping the difference) the phase index advances by 2 (2400 bit/s) / 4 (4800 bit/s), and the reference angle is that phase's entry in
 * V27RX_DEC_LAST_PHASE.  After the training length (0x3e8 calls with the long training, 0x32 with the short one) the callback releases the forced update,
 * selects the tracking step size (muIdx 1) and switches the decision callback to V27RX_decision().
 */
int V27RX_eq_train(FaxFse* f, short* angle, short* mag)
{
  V27RxLoop* h = (V27RxLoop*)f->cfg.hist;
  const unsigned short mask = h->mask;
  const short cnt0 = h->phaseCnt, idx0 = h->idx;
  const short* last = h->last;
  const int rate4800Flag = h->rate4800, shortTrain = h->shortTrain;
  const unsigned short ang = (unsigned short)*angle, prev = (unsigned short)h->prev;
  unsigned short cnt = (unsigned short)(h->cnt + 1);
  int step, d, limit;
  short idx = idx0, nang, cntNew;

  /* step the phase index when the angle moved far enough from the previous reference */
  step = (rate4800Flag == 0) ? 2 : 4;
  d = (short)(ang - prev);
  if ((short)d > 0x4000)
    d = (short)(d + 0x8000);
  if ((short)d < -0x4000)
    d = (short)(d - 0x8000);
  if ((d < 0 ? -d : d) > 0x2000)
    idx = (short)(((unsigned short)idx + step) & mask);
  nang = last[idx];

  /* publish the reference symbol and count the training length */
  cntNew = (short)(cnt0 + 1);
  limit = (shortTrain == 0) ? 0x3e8 : 0x32;
  h->cnt = cnt == 0x8000 ? (short)0x4000 : (short)cnt;
  *mag = 0x3299;
  h->idx = idx;
  *angle = nang;
  h->prev = nang;
  h->phaseCnt = cntNew;
  f->muIdx = 0;

  /* end of the training: switch to decision directed tracking */
  if (cntNew >= limit)
  {
    f->eqForce = 0;
    f->muIdx = 1;
    f->cfg.cb = V27RX_decision;
  }
  return USHRT_MAX;
}

/**
 * @brief Equalizer decision callback of the DATA / IDLE states: decide one symbol.
 * @param f      equalizer; `f->cfg.hist` is the V27RxLoop
 * @param angle  in: angle of the received symbol; out: reference angle of the chosen phase
 * @param mag    out: reference magnitude (0x3299)
 * @return the symbol code `pmap[best]` of the chosen phase step
 *
 * Takes the angle difference to the reference of the previous phase index (wrapped into 0 .. 0x8000) and picks the nearest of 4 (2400) / 8 (4800) phase
 * steps from V27RX_DEC_LAST_PHASE; the new phase index is the previous one plus that step, masked.
 * @note The search starts from the minimum `(short)(0x8000 - d)` (`-0x8000 - d` when `d < 0x8000`), which is -32768 for a difference of 0: nothing beats it, so
 *       index 0 is kept.
 */
int V27RX_decision(FaxFse* f, short* angle, short* mag)
{
  V27RxLoop* h = (V27RxLoop*)f->cfg.hist;
  const int n = (h->rate4800 == 0) ? 4 : 8;
  const short* last = h->last;
  const unsigned short* pmap = h->pmap;
  const unsigned short mask = h->mask;
  const short idx0 = h->idx;
  const unsigned short cnt = (unsigned short)(h->cnt + 1);
  const int ref = last[idx0];
  int d, i, best, init;
  short min, nidx;

  d = (int)*angle - ref;
  if (d < 0)
    d += 0x8000;
  if (d > 0x8000)
    d -= 0x8000;
  init = (d - 0x8000 < 0) ? (-0x8000 - d) : (d + 0x8000);
  min = (short)init;
  best = 0;

  /* nearest phase step */
  for (i = 0; i < n; i++)
  {
    int t = (int)(short)last[i];
    int dist = (d >= t) ? (d - (int)(unsigned short)last[i]) : ((int)(unsigned short)last[i] - d);
    if ((short)dist < min)
    {
      best = i;
      min = (short)dist;
    }
  }

  nidx = (short)(((unsigned short)idx0 + best) & mask);
  h->cnt = cnt == 0x8000 ? (short)0x4000 : (short)cnt;
  h->idx = nidx;
  *mag = 0x3299;
  *angle = (short)last[nidx];
  return pmap[best];
}

/* ===== detectors */
/** @brief Signal-to-noise estimate of the receiver: always 10 (the "SNR low" flag threshold of the V.29 receiver, 8, is never reached). */
int GetSNRV27(V27Rx* o)
{
  (void)o;
  return 10;
}

/** @brief Carrier flag of the training states: the equalizer PLL enable ANDed with the AGC detect flag. */
NOINL int CarrierDetectV27(V27Rx* o)
{
  V27RxBundle* b = o->bundle;
  return b->sre.pllEnable & (int)b->agc.detect;
}

/** @brief Nonzero once the epoch detector has forced the equalizer update (the epoch was found). */
NOINL int EpochDetectV27(V27Rx* o) { return o->bundle->fse.eqForce != 0; }

/**
 * @brief Smooth the equalizer's decision error over the first 50 data blocks and judge the link quality.
 * @return the carrier flag `agc.detect & sre.pllEnable`, or 2 when there is no carrier (the caller drops the block's symbols)
 *
 * The first block seeds `decErrSmooth`; blocks 2 .. 50 form a one-pole average (weights 0xccd / 0x7333 in Q15); after block 50 `qualityOk` is set if the
 * average is not above `errLimit`.
 */
NOINL int QualityDetectV27(V27Rx* o)
{
  V27RxBundle* b = o->bundle;
  int pllOn = b->sre.pllEnable;
  short det = (short)b->agc.detect;
  short decErr = b->fse.decErrAvg;
  int r = (short)(det & pllOn);
  unsigned short blocks;

  if (r == 0)
  {
    if (DBG)
      dsplibs_debug_printf("V27 Dec error too big... unreliable data\n");
    r = 2;
  }

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

  /* the 50th block: decide whether the smoothed error is acceptable */
  if ((short)blocks == 0x32)
  {
    if (!(b->decErrSmooth > (short)b->errLimit))
      b->qualityOk = 1;
    b->qualBlocks = (short)(b->qualBlocks + 1);
  }
  return r;
}

/**
 * @brief Carrier detector of the DATA state.
 * @param o   receiver engine
 * @param in  input block (n samples)
 * @param n   number of input samples
 * @return the carrier flag (0 = no carrier), sign-extended from 16 bit
 * @warning When the V.21 carrier search is active the block is copied into the 0x140 byte `hdx.buf`: `n` above 160 overruns it.
 *
 * Starts from `agc.detect & sre.pllEnable` and then depends on the mode flag `hdx.s14` (set by the T.30 receiver):
 * - 0: after 1500 symbols (`loop.cnt` above 0x5db) the carrier is dropped when the decoder error is above 0x3fff, otherwise only bit 0 of the flag is kept;
 * - nonzero: the carrier search is latched on (`toneSearch`) once the decoder error is above 0x3fff or bit 0 of the flag is clear; it runs on a copy of the
 *   block (own AGC + tone detector `mtd2`): a nonzero detector result resets `sinceTone`, otherwise the block length is added, and more than 0x4ff
 *   accumulated samples (about 160 ms) drop the carrier (debug text "V27: V21 Carrier detected").
 * Finally, if the energy test is enabled, a block RMS more than 8 dB below the reference level (0x32fe / 0x8000) drops the carrier; the reference level is
 * refreshed every second block.
 */
NOINL int DataCarrierDetectV27(V27Rx* o, short* in, unsigned short n)
{
  V27RxBundle* b = o->bundle;
  V27RxHdx* h = o->hdx;
  int ebx = (short)((short)b->agc.detect & b->sre.pllEnable);
  short decErr = b->fse.decErrAvg;

  if (h->s14 == 0)
  {
    if (b->loop.cnt > 0x5db)
    {
      if (decErr > 0x3fff)
        ebx = 0;
      else
        ebx &= 1;
    }
    if (decErr > 0x3fff)
    {
      if (DBG)
        dsplibs_debug_printf("V27 Decoder error too big... no carrier\n");
    }
  }
  else
  {
    if (decErr > 0x3fff)
      h->toneSearch = 1;
    else if (!(ebx & 1))
      h->toneSearch = 1;
    ebx = 1;
    if (h->toneSearch != 0)
    {
      short i = 0;
      short* const buf = h->buf;

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
          dsplibs_debug_printf("V27: V21 Carrier detected\n");
        ebx = 0;
      }
    }
  }

  /* energy-drop test */
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

/* ===== demodulator */
/**
 * @brief Demodulate one block: AGC, matched filter, timing recovery, equalizer / decision loop.
 * @param o    receiver engine
 * @param in   input samples (n entries); the AGC works on them in place
 * @param out  receives the decided symbol codes (one entry per symbol)
 * @param n    number of input samples
 * @return the number of symbols produced; 0 when the state is START and the tone detector fires (the block is dropped)
 * @warning More than 164 timing-recovery output samples are only reported ("ERROR: SRE buffer violation"); the output buffer `bundle->buf2` is 0x148 bytes,
 *          so a larger count has already overrun it (docs/bugs.md, section I).
 *
 * The AGC detect flag gates the timing loop and the equalizer (see the file header).
 */
NOINL int DemodDataV27(V27Rx* o, short* in, short* out, unsigned short n)
{
  V27RxBundle* b = o->bundle;
  V27RxHdx* h = o->hdx;
  int detect;
  unsigned int produced;
  unsigned int cnt;

  FPM_AGC_agc(&b->agc, in, n);
  detect = (int)b->agc.detect;

  /* start state: a tone on the line is not a V.27ter carrier */
  if (h->state == V27RX_STATE_START)
  {
    if ((unsigned short)FPM_MTD_detect(h->mtd, in, (short)n) != 0)
      return 0;
  }

  /* matched filter, then timing recovery (gated by the AGC) */
  produced = (unsigned short)FPM_MRF_filter(&b->mrf, in, b->buf1, (short)n);
  b->sre.loopEnable = b->sreGate & detect;
  cnt = (unsigned short)FAXSRE_recover(&b->sre, (const unsigned short*)b->buf1, b->buf2, (short)(unsigned short)produced);
  if ((unsigned short)cnt > 0xa4)
  {
    if (DBG)
      dsplibs_debug_printf("ERROR: SRE buffer violation(%d)", (int)(unsigned short)cnt);
  }

  /* equalizer and decision loop */
  b->fse.predEnable = 0;
  b->fse.eqEnable = b->eqGate & detect;
  b->fse.pllEnable = detect & b->pllGate;
  return (int)(unsigned short)FAXFSE_receive(&b->fse, b->buf2, out, (unsigned short)cnt);
}

/* ===== state machine */
/**
 * @brief Advance the receiver to the next state and install its handler.
 * @param o  receiver engine
 *
 * START -> EPOCH_DET (2 blocks), EPOCH_DET -> PROTOCOL (0x21 / 0x2d blocks at 4800 / 2400 with the long training, 1 / 2 with the short one; advances the AGC
 * alpha / beta table pointers), PROTOCOL -> DATA (freezes the AGC), DATA -> IDLE, IDLE -> DATA (restores the connect code).  Any other state value records the
 * internal-error status (b[0] = 3, flag 2) and changes nothing else.  Each case prints "V27RX_STATE_<name of the state left>" at debug level > 1 first.
 */
NOINL void RxNextStateV27(V27Rx* o)
{
  V27RxHdx* h = o->hdx;
  switch (h->state)
  {
  case V27RX_STATE_START:
    if (DBG)
      dsplibs_debug_printf("V27RX_STATE_START\n");
    h->count = 2;
    h->fn = RxHdxEpochDetV27;
    h->state = V27RX_STATE_EPOCH_DET;
    o->st.b[2] &= 0xfe;
    o->st.b[1] &= 0xfe;
    return;
  case V27RX_STATE_EPOCH_DET:
  {
    V27RxBundle* b;
    short cnt;
    if (DBG)
      dsplibs_debug_printf("V27RX_STATE_EPOCH_DET\n");
    if (h->longTrain == 0)
      cnt = (short)((h->rate != V27RC_4800) + 1);
    else
      cnt = h->rate == V27RC_4800 ? 0x21 : 0x2d;
    h->count = cnt;
    b = o->bundle;
    h->fn = RxHdxPrtcolV27;
    h->state = V27RX_STATE_PROTOCOL;
    o->st.b[2] &= 0xfe;
    o->st.b[1] &= 0xfe;
    b->agc.alpha = b->agc.alpha + 1;
    b->agc.beta = b->agc.beta + 1;
    return;
  }
  case V27RX_STATE_PROTOCOL:
    if (DBG)
      dsplibs_debug_printf("V27RX_STATE_PROTOCOL\n");
    FPM_AGC_Freeze(&o->bundle->agc);
    h = o->hdx;
    h->count = 0;
    h->fn = RxHdxDataV27;
    h->state = V27RX_STATE_DATA;
    o->st.b[1] |= 1;
    o->st.b[2] &= 0xfe;
    return;
  case V27RX_STATE_DATA:
    if (DBG)
      dsplibs_debug_printf("V27RX_STATE_DATA\n");
    h->fn = RxHdxIdleV27;
    h->state = V27RX_STATE_IDLE;
    h->count = 0;
    h->hold = 0;
    o->st.b[2] |= 1;
    o->st.b[1] &= 0xfe;
    return;
  case V27RX_STATE_IDLE:
    if (DBG)
      dsplibs_debug_printf("V27RX_STATE_IDLE\n");
    h->fn = RxHdxDataV27;
    h->state = V27RX_STATE_DATA;
    o->st.b[1] |= 1;
    o->st.b[2] &= 0xfe;
    o->st.b[0] = (unsigned char)(7 - (h->rate == V27RC_2400));
    return;
  default:
  {
    unsigned char f;
    if (DBG)
      dsplibs_debug_printf("V27RX_DEFAULT, %d\n", (int)h->state);
    f = o->st.b[1];
    o->st.b[2] &= 0xfe;
    o->st.b[0] = V27RX_MSG_ERROR;
    f = (unsigned char)(f | 2);
    f = (unsigned char)(f & 0xde);
    o->st.b[1] = f;
    return;
  }
  }
}

/**
 * @brief Handler of state START: wait for carrier.
 * @return 0 (no symbols); `*n` is set to 0
 *
 * Runs the demodulator for its carrier detection; on carrier it moves to EPOCH_DET.
 */
NOINL int RxHdxStartV27(V27Rx* o, short* in, short* out, unsigned short* n)
{
  const unsigned short n0 = *n; /* every input is read before the first store (the result must not depend on the aliasing rules) */

  o->st.b[1] &= 0xdf;
  o->st.b[0] = V27RX_MSG_START;
  DemodDataV27(o, in, out, n0);
  if (CarrierDetectV27(o) == 0)
  {
    *n = 0;
    return 0;
  }
  RxNextStateV27(o);
  *n = 0;
  return 0;
}

/**
 * @brief Handler of state EPOCH_DET: wait for the phase-reversal epoch (or for the block budget to run out).
 * @return 0 (no symbols); `*n` is set to 0
 *
 * While the carrier is present it reports the protocol status; the state advances when the epoch detector has fired or the block count is exhausted.  Loss of
 * carrier moves to ERROR with the loss-of-carrier status.
 */
NOINL int RxHdxEpochDetV27(V27Rx* o, short* in, short* out, unsigned short* n)
{
  V27RxHdx* h;
  unsigned short cnt;

  DemodDataV27(o, in, out, *n);
  *n = 0;
  if (CarrierDetectV27(o) != 0)
  {
    o->st.b[1] |= 0x20;
    o->st.b[0] = V27RX_MSG_PROTOCOL;
    h = o->hdx;
    cnt = (unsigned short)(h->count - 1);
    h->count = (short)cnt;
    if ((short)cnt > 0)
    {
      if ((unsigned short)EpochDetectV27(o) == 0)
        return 0;
    }
    RxNextStateV27(o);
    return 0;
  }
  h = o->hdx;
  h->fn = RxHdxErrorV27;
  h->state = V27RX_STATE_ERROR;
  o->st.b[0] = V27RX_MSG_LOSS_CARRIER;
  o->st.b[1] = (unsigned char)((o->st.b[1] | 2) & 0xdf);
  return 0;
}

/**
 * @brief Handler of state PROTOCOL: equalizer training; the decided symbols are descrambled and returned.
 * @return the number of symbols produced in the last block of the state, else 0; `*n` is set to 0
 *
 * When the block budget (`hdx.count`) runs out the connect code of the rate is stored and the state advances to DATA.  Loss of carrier moves to ERROR with
 * the loss-of-carrier status.
 */
NOINL int RxHdxPrtcolV27(V27Rx* o, short* in, short* out, unsigned short* n)
{
  V27RxHdx* h;
  unsigned int ret = (unsigned short)DemodDataV27(o, in, out, *n);

  DescrambleDataV27(o, (unsigned short*)out, (short)ret);
  *n = 0;
  if (CarrierDetectV27(o) != 0)
  {
    unsigned short cnt;
    o->st.b[1] |= 0x20;
    o->st.b[0] = V27RX_MSG_PROTOCOL;
    h = o->hdx;
    cnt = (unsigned short)(h->count - 1);
    h->count = (short)cnt;
    if ((short)cnt > 0)
      return 0;
    o->st.b[0] = (unsigned char)(7 - (h->rate == V27RC_2400));
    RxNextStateV27(o);
    return (short)ret;
  }
  h = o->hdx;
  h->fn = RxHdxErrorV27;
  h->state = V27RX_STATE_ERROR;
  o->st.b[0] = V27RX_MSG_LOSS_CARRIER;
  o->st.b[1] = (unsigned char)((o->st.b[1] | 2) & 0xdf);
  return 0;
}

/**
 * @brief Handler of state DATA: demodulate, descramble and return the data symbols.
 * @return the number of symbols produced (0 when the carrier is lost, the input is on hold, or QualityDetectV27() reports no carrier); `*n` is set to 0
 *
 * Sets the carrier flag and the data status, then asks DataCarrierDetectV27(); without carrier (or with `hdx.hold` set) the block is discarded.
 */
NOINL int RxHdxDataV27(V27Rx* o, short* in, short* out, unsigned short* n)
{
  unsigned int count;
  int ret;
  const unsigned short n0 = *n;

  o->st.b[1] |= 0x20;
  o->st.b[0] = V27RX_MSG_DATA;
  COMPILER_BARRIER();
  if ((short)DataCarrierDetectV27(o, in, n0) == 0 || o->hdx->hold != 0)
  {
    o->st.b[1] &= 0xdf;
    *n = 0;
    return 0;
  }

  count = (unsigned short)DemodDataV27(o, in, out, *n);
  DescrambleDataV27(o, (unsigned short*)out, (short)count);
  *n = 0;
  ret = ((short)QualityDetectV27(o) != 2) ? (int)count : 0;

  /* refresh the low-SNR flag */
  o->st.b[1] &= 0x7f;
  if (GetSNRV27(o) <= 8)
    o->st.b[1] |= 0x80;
  return (short)ret;
}

/**
 * @brief Handler of state IDLE: keep demodulating without output until the decision error is small again.
 * @return 0 (no symbols); `*n` is set to 0
 *
 * Returns to DATA (via RxNextStateV27()) when the carrier is present and `fse.decErrAvg` is at most 0x1fff.
 */
NOINL int RxHdxIdleV27(V27Rx* o, short* in, short* out, unsigned short* n)
{
  DemodDataV27(o, in, out, *n);
  *n = 0;
  COMPILER_BARRIER();
  o->st.b[1] &= 0xdf;
  o->st.b[0] = V27RX_MSG_IDLE;
  if (CarrierDetectV27(o) != 0)
    o->st.b[1] |= 0x20;
  if (o->st.b[1] & 0x20)
  {
    if (o->bundle->fse.decErrAvg <= 0x1fff)
    {
      RxNextStateV27(o);
      if (DBG)
        dsplibs_debug_printf("Decision error is small back to DATA mode !!!\n");
    }
  }
  return 0;
}

/**
 * @brief Handler of state ERROR: set the error flag and keep demodulating (the state is left only by re-creating the engine).
 * @return 0 (no symbols); `*n` is set to 0
 */
NOINL int RxHdxErrorV27(V27Rx* o, short* in, short* out, unsigned short* n)
{
  const unsigned short n0 = *n;
  o->st.b[1] |= 2;
  DemodDataV27(o, in, out, n0);
  *n = 0;
  return 0;
}

/* ===== create / delete / modem / control / status */
/**
 * @brief Create or re-initialise a V.27ter receiver.
 * @param o    an existing engine to re-initialise in place (its blocks and sub-objects are reused), or NULL to allocate the engine, the half-duplex block and
 *             the bundle
 * @param cfg  seven configuration words (V27RxCfg), or NULL for the defaults V27RX_CFG; `cfg[1]` low half is the bit rate (2400 or 4800), `cfg[5] == 0`
 *             selects the long training, `cfg[6]` is the user word
 * @return the engine, in state START with status "start of protocol"
 *
 * Builds the V.21 carrier detector with its own AGC, the data-mode tone detector, the matched filter, the AGC, the timing recovery (6 samples per symbol at
 * 2400, 5 at 4800) and the equalizer (with V27RX_epoch_det() as the first decision callback), the receive scrambler and the decision-loop state, and clears
 * the work buffers.  Sub-objects are allocated only when the engine (bundle) or the half-duplex block is new.
 * @warning The allocation results are not checked (a failed allocation dereferences NULL, docs/bugs.md section I).  A rate other than 2400 / 4800 selects the
 *          4800 tables and sets an error status that the common status store a few lines later overwrites: the error is never visible to the caller.
 *          The half-duplex mode flag `hdx.s14` is zeroed only when the half-duplex block is allocated here; the T.30 receiver writes it afterwards.
 */
V27Rx* V27RX_create(V27Rx* o, const int* cfg)
{
  int objFresh = 0, hdxFresh = 0;
  V27RxHdx* h;
  V27RxBundle* b;
  int i, rate;
  int mtdCfg[3], mrfCfg[4];
  FaxSreCfg sre;
  FaxFseCfg fse;
  FaxSdmCfg27 sdmCfg;
  int cc[7];
  int user, sps, longTrain;
  short ppmTick, ppmPeriod;

  if (DBG)
    dsplibs_debug_printf("V.27 RX Create ");
  if (o == NULL)
  {
    if (DBG)
      dsplibs_debug_printf("New allocation\n");
    o = (V27Rx*)sysdep_malloc((unsigned int)sizeof(V27Rx));
    o->hdx = NULL;
    o->bundle = NULL;
    objFresh = 1;
  }
  if (DBG)
    dsplibs_debug_printf("\n");

  /* take a private copy of the configuration: the caller may pass the engine's own `o->cfg` (re-create), and the words must not be re-read after the stores */
  if (cfg == NULL)
    cfg = V27RX_CFG;
  for (i = 0; i < 7; i++)
    cc[i] = cfg[i];
  for (i = 0; i < 7; i++)
    o->cfg.w[i] = cc[i];
  user = cc[6];
  longTrain = cc[5] == 0;

  /* half-duplex block: allocate on first use, then (re)build its state */
  h = o->hdx;
  if (h == NULL)
  {
    h = (V27RxHdx*)sysdep_malloc((unsigned int)sizeof(V27RxHdx));
    o->hdx = h;
    hdxFresh = 1;
    h->mtd = NULL;
    h->buf = (short*)sysdep_malloc(0x140);
    h->mtd2 = NULL;
    h->s14 = 0;
  }
  h->hold = 0;
  h->state = V27RX_STATE_START;
  h->count = 0;
  h->fn = RxHdxStartV27;
  h->longTrain = (short)longTrain;

  /* carrier detector: V21_CHAN2_MTD_COEFF, {2, 0x4ccd}, FPM_MTD_CFG[2] low half 0x12c */
  mtdCfg[0] = CFGP(V21_CHAN2_MTD_COEFF);
  mtdCfg[1] = (int)(0x4ccdu << 16 | 2u);
  mtdCfg[2] = (int)(((unsigned int)FPM_MTD_CFG_raw[2] & 0xffff0000u) | 0x12cu);
  h->mtd2 = FPM_MTD_create(h->mtd2, mtdCfg);
  FPM_AGC_init(&h->agc, AGCv27_CFG, hdxFresh);
  h->sinceTone = 0;
  h->toneSearch = 0;

  /* bit rate */
  if ((short)cc[1] == RATE_2400)
  {
    rate = V27RC_2400;
  }
  else if ((short)cc[1] == RATE_4800)
  {
    rate = V27RC_4800;
  }
  else
  {
    rate = V27RC_4800;
    o->st.b[1] |= 2;
    o->st.b[0] = V27RX_MSG_ERROR;
  }
  h->rate = (short)rate;

  /* data-mode tone detector: V27_MTD_COEFF_4800 at rate 1, else _2400; {2, 0x199a}, low half 0x64 */
  mtdCfg[0] = cfgp_word(rate == V27RC_4800 ? V27_MTD_COEFF_4800 : V27_MTD_COEFF_2400);
  mtdCfg[1] = (int)(0x199au << 16 | 2u);
  mtdCfg[2] = (int)(((unsigned int)FPM_MTD_CFG_raw[2] & 0xffff0000u) | 0x64u);
  h->mtd = FPM_MTD_create(h->mtd, mtdCfg);

  /* bundle: allocate on first use, then (re)build the signal path */
  b = o->bundle;
  if (b == NULL)
  {
    b = (V27RxBundle*)sysdep_malloc((unsigned int)sizeof(V27RxBundle));
    o->bundle = b;
    b->buf1 = (short*)sysdep_malloc(0x140);
    b->buf2 = (short*)sysdep_malloc(0x148);
  }

  /* matched filter */
  mrfCfg[0] = (int)((unsigned int)V27RX_MRF_UP[rate] | ((unsigned int)V27RX_MRF_DOWN[rate] << 16));
  mrfCfg[1] = cfgp_word(V27RX_MRF_FILT[rate]);
  mrfCfg[2] = (int)(((unsigned int)FPM_MRF_CFG_raw[2] & 0xffff0000u) | (unsigned int)V27RX_MRF_FILT_LEN[rate]);
  mrfCfg[3] = user;
  FPM_MRF_init(&b->mrf, mrfCfg, objFresh);
  FPM_AGC_init(&b->agc, AGCv27_CFG, objFresh);
  if (rate == V27RC_4800)
    b->agc.a.s[5] = 0x28;

  /* timing recovery */
  sre = fax_sre_cfg_default;
  COMPILER_BARRIER();
  sps = V27RX_SAMP_PER_BAUD[rate];
  sre.sps = (short)sps;
  sre.periodsTrack = 1;
  sre.periodsLock = 8;
  sre.startCount = 0x40;
  sre.nt = (short)V27RX_SRE_FILT_LEN[rate];
  sre.coefTab = cfgp_word(V27RX_SRE_FILT[rate]);
  sre.tedCoef = cfgp_word(V27RX_XB_COFFS[rate]);
  sre.corrCosTab = cfgp_word(V27RX_XCLOCK[rate]);
  sre.corrSinTab = cfgp_word(V27RX_YCLOCK[rate]);
  sre.propGainTab = cfgp_word(V27RX_SRE_PLLK1[rate]);
  sre.intGainTab = cfgp_word(V27RX_SRE_PLLK2[rate]);
  sre.errOn = 2;
  sre.errOff = 1;
  sre.unlockThresh = 0x3333;
  sre.lockThresh = 0x199a;
  sre.levelThresh = (short)((int)b->agc.a.s[0] / 6);
  sre.levelLen = (short)((int)(short)sps * 3);
  sre.w34 = user;
  FAXSRE_init(&b->sre, &sre, objFresh);
  ppmTick = (short)(rate != V27RC_2400 ? 0x20 : 0x18);
  ppmPeriod = (short)((int)ppmTick * 200);
  b->sre.ppmTick = ppmTick;
  b->sre.ppmPeriod = ppmPeriod;
  b->sre.ppmStep = (short)(1000000 / ((int)(short)sre.sps * (int)ppmPeriod));
  b->sre.ppmNMax = (short)(1000000 / (int)ppmPeriod);

  /* equalizer */
  fse = fax_fse_cfg_default;
  COMPILER_BARRIER();
  fse.blockLen = (short)(rate == V27RC_2400 ? 0x90 : 0xa0);
  fse.sps = (short)sps;
  fse.iCoefTab = cfgp_word(V27RX_FSE_IFILT[rate]);
  fse.qCoefTab = cfgp_word(V27RX_FSE_QFILT[rate]);
  fse.nt = (short)V27RX_FSE_FILT_LEN[rate];
  fse.mu[0] = (short)V27RX_FSE_MU_TRAIN[rate];
  fse.mu[1] = (short)V27RX_FSE_MU_TRACK[rate];
  fse.carrierTab = cfgp_word(V27RX_CRR_TABLE[rate]);
  fse.crrLen = (short)V27RX_CRR_TABLE_LEN[rate];
  fse.crrAdj = (short)V27RX_CRR_ADJUST[rate];
  fse.settleCount = 0x3e8;
  fse.errThreshHi = 0x2666;
  fse.errThreshLo = 0x8f6;
  fse.propTab = cfgp_word(V27RX_FSE_PLLK1[rate]);
  fse.intTab = cfgp_word(V27RX_FSE_PLLK2[rate]);
  fse.hist = &b->loop;
  fse.cb = V27RX_epoch_det;
  fse.w34 = user;
  FAXFSE_init(&b->fse, &fse, objFresh);

  /* work buffers, quality detector and decision-loop state */
  {
    short *const buf1 = b->buf1, *const buf2 = b->buf2;
    for (i = 0; i < 0xa0; i++)
    {
      buf1[i] = 0;
      buf2[i] = 0;
    }
  }

  b->qualityOk = 0;
  b->decErrSmooth = 0;
  b->qualBlocks = 0;
  if (rate == V27RC_2400)
    b->errLimit = 0x1b9e;
  else if (rate == V27RC_4800)
    b->errLimit = 0xd27;
  b->loop.symI0 = 0;
  b->refBlocks = 0;
  b->energyTest = 1;
  b->refLevel = 0;
  b->loop.symQ0 = 0;
  b->loop.symI1 = 0;
  b->loop.symQ1 = 0;
  b->loop.symI2 = 0;
  b->loop.symQ2 = 0;
  b->loop.rate4800 = (rate == V27RC_4800);
  b->loop.mask = V27RX_DEC_PHS_MASK[rate];
  b->loop.idx = 0;
  b->loop.shortTrain = (longTrain == 0);
  b->loop.phaseCnt = 0;
  b->loop.avg = 0x3299;
  b->loop.prev = 0;
  b->loop.cnt = 0;
  b->loop.pmap = V27RX_DEC_PMAP[rate];
  b->loop.last = V27RX_DEC_LAST_PHASE[rate];

  /* receive scrambler, gates and flags */
  sdmCfg.n = (unsigned short)(3 - (rate == V27RC_2400));
  SDMv27_init(&b->sdm, &sdmCfg);
  b->w0 = 1;
  b->sreGate = 1;
  b->pllGate = 1;
  b->wc = 0;
  b->eqGate = 1;

  /* status word and the copies of the equalizer pointers */
  o->st.w = 0;
  o->st.b[1] |= 0x50;
  o->st.b[0] = V27RX_MSG_START;
  COMPILER_BARRIER();
  o->fseI = b->fse.outI;
  o->fseQ = b->fse.outQ;
  o->fseIdx = &b->fse.idx;
  o->fseTapsI = b->fse.tapsI;
  o->fseTapsQ = b->fse.tapsQ;
  o->fseNt = b->fse.cfg.nt;
  o->s40 = 0;
  o->s4c = 0;
  o->w38 = 0;
  o->w3c = 0;
  o->w44 = 0;
  o->w48 = 0;
  return o;
}

/** @brief Release a receiver created by V27RX_create(): equalizer, timing recovery, matched filter, work buffers, detectors and the three blocks. */
void V27RX_delete(V27Rx* o)
{
  V27RxBundle* b = o->bundle;
  V27RxHdx* h = o->hdx;
  FAXFSE_free(&b->fse);
  FAXSRE_free(&b->sre);
  FPM_MRF_free(&b->mrf);
  sysdep_free(b->buf2);
  sysdep_free(b->buf1);
  sysdep_free(b);
  FPM_MTD_delete(h->mtd);
  sysdep_free(h->buf);
  FPM_MTD_delete(h->mtd2);
  sysdep_free(h);
  sysdep_free(o);
}

/**
 * @brief Process one block of received samples.
 * @param o    receiver engine
 * @param in   input samples (`*n` entries)
 * @param out  receives the decided symbol codes
 * @param n    in: number of input samples; out: number of symbols written to `out`
 * @return the status word `st` (b[0] status code, b[1] flags, see V27Rx::st)
 *
 * Calls the handler of the current state repeatedly (a handler may change the state and leave part of the block unconsumed) until the block is used up.
 * @warning `out` must be large enough for the symbols of the whole block.
 */
int V27RX_modem(V27Rx* o, short* in, short* out, unsigned short* n)
{
  short total = 0;
  unsigned short cur = *n; /* read before the first store (the result must not depend on the aliasing rules) */

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
 * @param o    receiver engine
 * @param ctl  control record, or NULL
 * @return 0 when `ctl` is NULL, else 1
 *
 * `ctl->w4` becomes cfg word 2; byte 0xd bit 4 sets `hdx.hold` (discard the data input), bit 1 re-creates the engine in place with the stored configuration;
 * byte 0xc bit 3 clears the bundle flag `w0` and bit 5 closes the equalizer-update gate.
 */
int V27RX_control(V27Rx* o, const FaxV27Ctl* ctl)
{
  unsigned char d, c;
  int w4;
  V27RxHdx* h;

  if (ctl == NULL)
    return 0;
  w4 = ctl->w4; /* the control record is read before any store (the result must not depend on the aliasing rules) */
  d = ctl->b0d;
  c = ctl->b0c;
  COMPILER_BARRIER();
  o->cfg.w[2] = w4;
  h = o->hdx;
  h->hold = (d >> 4) & 1;
  if (d & 2)
    V27RX_create(o, o->cfg.w);
  if (c & 8)
    o->bundle->w0 = 0;
  if (c & 0x20)
    o->bundle->eqGate = 0;
  return 1;
}

/**
 * @brief Status query: writes nothing.
 * @return 1 when `st` is not NULL, else 0
 * @note Unlike V29RX_status() the V.27ter receiver has no status record; the call only reports whether a record was supplied.
 */
int V27RX_status(V27Rx* o, void* st)
{
  (void)o;
  return st != NULL;
}

/* ===== VMI wrappers */
/**
 * @brief Initialise a VMI object as a V.27ter receiver (no ECM, simple packing).
 * @param m       VMI object to fill
 * @param rate    bit rate stored in the configuration (2400 / 4800)
 * @param unused  ignored
 * @param user    user word: stored as the engine handle slot and as cfg word 6 (copied into the filter configurations)
 * @return 0x80
 *
 * Allocates the 7 word configuration block (V27RX_CFG with the rate patched in) and stores the VMI geometry: 0x140, 0xa5, 0, 8.
 * @warning The allocation result is not checked.
 */
int fax_init_vmi_v27rx(FaxVmiV27* m, unsigned short rate, int unused, void* user)
{
  V27RxCfg* c = (V27RxCfg*)sysdep_malloc(sizeof(V27RxCfg));
  int i;
  (void)unused;
  if (2 < dsplibs_debug_level)
    dsplibs_debug_printf("Initializing VMI_V27_RX Modem No ECM (Simple Packing)\n");
  for (i = 0; i < 5; i++)
    c->w[i] = V27RX_CFG[i];
  c->w[5] = V27RX_CFG[5];
  c->w[6] = (int)(uintptr_t)user; /* 32-bit copy of the user word (only ever copied on into the filter configurations) */
  c->s[2] = (short)rate;

  /* VMI object: geometry, engine handle slot, configuration */
  m->w0 = 0;
  m->s08 = 0x140;
  m->s0a = 0xa5;
  m->s0c = 0;
  m->s0e = 8;
  m->eng = user;
  m->u4.init1 = 1;
  m->u10.cfg = c;
  return 0x80;
}

/**
 * @brief VMI create: build the receiver engine and store it in the VMI object.
 * @param m    VMI object; `m->eng` is the engine to re-initialise (NULL: allocate)
 * @param cfg  configuration words, or NULL for V27RX_CFG
 * @return the engine (also stored in `m->eng`)
 */
void* v27rx_create(FaxVmiV27* m, const int* cfg)
{
  int w[7], i;
  V27Rx* e;

  if (cfg == NULL)
    cfg = V27RX_CFG;
  for (i = 0; i < 7; i++)
    w[i] = cfg[i];
  e = V27RX_create((V27Rx*)m->eng, w);
  m->s0c = 0;
  m->eng = e;
  m->s0e = 0;
  m->u10.s10 = (short)(2 + ((short)((V27RxCfg*)w)->s[2] != RATE_2400));
  return e;
}

/** @brief VMI delete: release the engine in `m->eng`. */
void v27rx_delete(FaxVmiV27* m) { V27RX_delete((V27Rx*)m->eng); }

/**
 * @brief VMI process: run V27RX_modem() on `in`, writing the symbols into the VMI object's bit buffer.
 * @param outn  receives the number of symbols produced
 * @param n     in: number of input samples; out: 0
 * @return the status word of V27RX_modem()
 */
int v27rx_process(FaxVmiV27* m, short* in, unsigned short* outn, unsigned short* n)
{
  int r = V27RX_modem((V27Rx*)m->eng, in, m->u4.bits, n);
  *outn = *n;
  *n = 0;
  return r;
}

/** @brief VMI status: V27RX_status() of the engine in `m->eng`. */
int v27rx_status(FaxVmiV27* m, void* st) { return V27RX_status((V27Rx*)m->eng, st); }
/** @brief VMI control: V27RX_control() of the engine in `m->eng`. */
int v27rx_control(FaxVmiV27* m, const FaxV27Ctl* ctl) { return V27RX_control((V27Rx*)m->eng, ctl); }

/** @brief VMI message: `*out` = status message `idx` (0 .. 7, V27RX_MESG); NULL for any other index. */
void v27rx_message(FaxVmiV27* m, int idx, const char** out)
{
  (void)m;
  if ((unsigned int)idx > 8u)
    *out = NULL;
  else if ((unsigned int)idx == 8u)
    *out = NULL; /* index 8 is one past the message table */
  else
    *out = V27RX_MESG[idx & 0xff];
}
