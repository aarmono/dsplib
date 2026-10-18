/**
 * @file fax_v29_rx.c
 * @brief Fax V.29 receiver (7200 / 9600 bit/s, T.30 high-speed data): engine, half-duplex state machine, detectors, equalizer callbacks, VMI wrappers.
 *
 * Entry points: V29RX_create() / V29RX_delete() / V29RX_modem() / V29RX_control() / V29RX_status(), and the VMI wrappers v29rx_*() with the initialiser
 * fax_init_vmi_v29rx(), through which fax_vmi.c drives the receiver.  Typed objects are in fax_v29_rx.h, tables in fax_v29_rx_tables.c; the timing-recovery
 * and equalizer sub-objects are the FaxSre / FaxFse of fax_timing.c / fax_equalizer.c (shared with the V.27ter and V.17 receivers).  The receive
 * descrambler DescrambleDataV29() lives in fax_modem_wrappers.c; on a 64-bit build this file defines the accessor faxw_v29_rxsdm() of fax_modem_views.h
 * that lets it reach `bundle->sdm`.
 *
 * Signal flow of one block: V29RX_modem() runs the handler of the current state (RxHdx*V29), each of which consumes the whole block and returns the number
 * of symbols produced.  The handlers call DemodDataV29(): AGC (in place) -> [state START only: tone killer + tone detector, a detected tone aborts the block]
 * -> matched filter -> timing recovery -> carrier / equalizer / decision loop (the equalizer calls the decision callback of the current phase to decide
 * each symbol) -> DescrambleDataV29().
 *
 * States (V29RxHdx::state, advanced by RxNextStateV29()):
 * 0 START (wait for carrier) -> 1 EPOCH_DET (the callback V29RX_epoch_det() looks for the phase-reversal epoch, then switches itself to V29RX_eq_train())
 * -> 2 PROTOCOL (equalizer training: V29RX_eq_train(), then V29RX_decision()) -> 3 DATA -> 4 IDLE (decision quality lost; back to DATA when the error is
 * small again); 5 ERROR (carrier lost during training).
 *
 * Behaviour worth knowing (read from the code, not run unless stated at the function):
 * - RxNextStateV29() prints the name of the state it LEAVES (level > 1) before it acts; the transition 1 -> 2 advances the AGC alpha / beta table pointers by
 *   one entry (each table has two).  Idle -> data restores the connect code 7 (7200) / 6 (9600).
 * - The connect codes are the other way round than at V.27ter: rate 0 (7200) -> 7, rate 1 (9600) -> 6.  The protocol state sets the code AND the "SNR low"
 *   flag.
 * - The matched filter is the same at both rates (up 9, down 10, 0x10e taps); the rate only selects the scrambler's bit count (3 / 4), the decision
 *   constellation (8 / 16 points) and the quality threshold (0x320 / 0x2bc).
 * - The AGC output (agc.detect) gates the timing loop (sre.loopEnable = sreGate & detect) and the equalizer (pllEnable = pllGate & detect, eqEnable =
 *   eqGate & detect, predEnable = 0).
 * - GetSNRV29() = 14 - fse.decErrAvg, an estimate that the data and protocol handlers compare against 8 (V.27ter returns the constant 10).
 * - v29rx_create() returns the code 3 (7200) / 4 (any other rate), NOT the engine pointer (that only goes into the VMI object); fax_init_vmi_v29rx() returns
 *   0x80; v29rx_message(m, 8) returns NULL.
 */
#include <limits.h>
#include <string.h>
#include "fax_v29_rx.h"
#include "fax_v29_rx_tables.h"
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

/* int arithmetic of the 32-bit target wraps: a sum of two squares that can reach 2^31 is formed unsigned */
#define SQ2(x, y) ((int)((unsigned int)((int)(x) * (int)(x)) + (unsigned int)((int)(y) * (int)(y))))

#if UINTPTR_MAX == 0xffffffffu
/* the raw-offset accessor of fax_modem_views.h reads exactly these members */
_Static_assert(offsetof(V29Rx, bundle) == 0x50 && offsetof(V29RxBundle, sdm) == 0x4f3c, "faxw_v29_rxsdm offsets");
#else
/** @brief Receive descrambler of a V.29 receiver engine (the accessor DescrambleDataV29() uses on a 64-bit build). */
FaxSdm* faxw_v29_rxsdm(void* o) { return &((V29Rx*)o)->bundle->sdm; }
#endif

/* ===== FSE decision callbacks */
/**
 * @brief Equalizer decision callback of the EPOCH_DET state: look for the phase-reversal epoch that starts the training sequence.
 * @param f      equalizer; `f->cfg.hist` is the V29RxLoop, `f->outI` / `outQ` / `idx` give the newest equalized symbol
 * @param angle  in: angle of the received symbol; out: reference angle fed back to the carrier PLL
 * @param mag    in: symbol magnitude; out: reference magnitude
 * @return USHRT_MAX (no symbol code is produced while searching)
 *
 * Tracks the angle steps of consecutive symbols (large steps feed `avgA`, small ones `avgB`) and the distance between the last two symbol pairs.  After more
 * than 0x80 calls a pair distance larger than twice the energy of the two averages (avgA, avgB) marks the epoch: the callback loads the training step
 * sizes (mu[0] = 0x14e6), forces the equalizer update, seeds the training LFSR with 0x55 and switches the decision callback to V29RX_eq_train().
 * @note Every member is read before the first store, so the result does not depend on the aliasing rules (compared on the objects built with and without the flag).
 */
int V29RX_epoch_det(FaxFse* f, short* angle, short* mag)
{
  V29RxLoop* h = (V29RxLoop*)f->cfg.hist;
  const short *outI = f->outI, *outQ = f->outQ;
  const short idx = f->idx;
  const unsigned short cnt = (unsigned short)h->cnt;
  const short avgA0 = h->avgA, avgB0 = h->avgB, i0 = h->symI0, q0 = h->symQ0, i1 = h->symI1, q1 = h->symQ1, i2 = h->symI2, q2 = h->symQ2;
  const unsigned short prev = (unsigned short)h->prev, sinceStart = (unsigned short)h->phaseCnt;
  const unsigned short ang = (unsigned short)*angle;
  const int magIn = *mag;
  const int i = outI[idx], q = outQ[idx];
  int distNew, distOld, distSum, angStep, avgA1 = avgA0, avgB1 = avgB0;
  short ang2;

  distNew = SQ2((short)(i1 - i), (short)(q1 - q)) >> 15;
  distOld = SQ2((short)(i2 - i0), (short)(q2 - q0)) >> 15;
  distSum = (short)(distNew + distOld);
  angStep = (short)(ang - prev);
  /* classify the angle step: a step above half a turn (0x4000 in angle units) feeds the A average, otherwise the B average */
  if ((short)angStep < 0)
    angStep = (short)(angStep - 0x8000);
  if ((short)angStep > 0x4000)
  {
    avgA1 = (short)((avgA0 * 31) >> 5); /* (avgA0 << 5) - avgA0 >> 5 */
    avgA1 = (short)(avgA1 + (magIn >> 5));
    ang2 = V29RX_DEC_ANGLE[4];
  }
  else
  {
    avgB1 = (short)((avgB0 * 31) >> 5);
    avgB1 = (short)(avgB1 + (magIn >> 5));
    ang2 = V29RX_DEC_ANGLE[7];
  }

  /* update the history */
  f->eqEnable = 0;
  h->cnt = (short)(cnt + 1);
  h->symQ0 = (short)q;
  h->symI2 = i1;
  h->symI1 = i0;
  h->symQ2 = q1;
  h->symQ1 = q0;
  h->symI0 = (short)i;
  h->prev = (short)ang;
  h->avgA = (short)avgA1;
  h->avgB = (short)avgB1;
  *angle = ang2;

  /* after the start-up calls, a jump of the pair distance against the smoothed energy marks the epoch */
  if ((short)sinceStart > 0x80)
  {
    int energy = SQ2((short)avgA1, (short)avgB1) >> 15;
    if (distSum > (int)(short)energy * 2)
    {
      f->cfg.mu[0] = 0x14e6;
      f->eqForce = 1;
      f->cfg.cb = V29RX_eq_train;
      h->reg = 0x55;
      h->idx = 0;
      h->phaseCnt = 0;
      return USHRT_MAX;
    }
  }
  h->phaseCnt = (short)(sinceStart + 1);
  return USHRT_MAX;
}

/**
 * @brief Equalizer decision callback of the PROTOCOL state: supply the known training symbols while the equalizer converges.
 * @param f      equalizer; `f->cfg.hist` is the V29RxLoop
 * @param angle  out: reference angle of the training symbol
 * @param mag    out: reference magnitude of the training symbol
 * @return USHRT_MAX (no symbol code is produced during training)
 *
 * The training symbol sequence is a 7 bit LFSR (taps 0 and 6): when bit 0 of the register is set the reference is constellation point 3 (7200) / 0xb (9600),
 * otherwise point 0.  Equalizer updates are on and the error average is held at 0x4000.  After 0x17e calls the callback sets the tracking step size
 * (mu[1] = 0xa73), releases the forced update and switches the decision callback to V29RX_decision().
 */
int V29RX_eq_train(FaxFse* f, short* angle, short* mag)
{
  V29RxLoop* h = (V29RxLoop*)f->cfg.hist;
  const unsigned short cnt = (unsigned short)((unsigned short)h->cnt + 1);
  const int reg = h->reg;
  const int rate = h->rate;
  const unsigned short c18 = (unsigned short)((unsigned short)h->phaseCnt + 1);
  unsigned int t;
  int idx = 0;
  short *const ang = angle, *const mg = mag;

  /* advance the LFSR; a set low bit selects the non-zero training point */
  t = (((unsigned int)reg << 6) & 0x80u) ^ (((unsigned int)reg & 1u) << 7);
  t = ((t | (unsigned int)reg) >> 1) & 0x7fu;
  if (reg & 1)
    idx = (rate == V29RC_7200) ? 3 : 0xb;

  /* keep the equalizer adapting with the training step size and publish the reference symbol */
  h->cnt = cnt == 0x8000 ? (short)0x4000 : (short)cnt;
  f->eqEnable = 1;
  f->muIdx = 0;
  f->decErrAvg = 0x4000;
  *mg = V29RX_DEC_MAG[idx];
  *ang = V29RX_DEC_ANGLE[idx];
  h->reg = (short)t;
  h->phaseCnt = (short)c18;

  /* end of the training sequence: switch to decision directed tracking */
  if (c18 == 0x17e)
  {
    f->cfg.mu[1] = 0xa73;
    f->muIdx = 1;
    f->eqForce = 0;
    f->cfg.cb = V29RX_decision;
  }
  h->idx = (short)(idx & 7);
  return USHRT_MAX;
}

/**
 * @brief Equalizer decision callback of the DATA / IDLE states: decide one symbol.
 * @param f      equalizer; `f->cfg.hist` is the V29RxLoop, `f->outI` / `outQ` / `idx` give the equalized symbol
 * @param angle  out: reference angle of the chosen point
 * @param mag    out: reference magnitude of the chosen point
 * @return the symbol code: the V29RX_DEC_PMAP entry of the difference between the phase index of the chosen point and the previous one, plus bit 3 of the
 *         point at 9600 bit/s
 *
 * Picks the nearest of 8 (7200) / 16 (9600) constellation points by a distance whose Q term is shifted by 16 (the I term by 15).
 */
int V29RX_decision(FaxFse* f, short* angle, short* mag)
{
  V29RxLoop* h = (V29RxLoop*)f->cfg.hist;
  const unsigned short cnt = (unsigned short)((unsigned short)h->cnt + 1);
  const int rate = h->rate;
  const int n = (rate == V29RC_7200) ? 8 : 16;
  const int I = f->outI[f->idx], Q = f->outQ[f->idx];
  int i, best = 0, min = 0x7fff, pidx, ret;

  h->cnt = cnt == 0x8000 ? (short)0x4000 : (short)cnt;

  /* nearest constellation point */
  for (i = 0; i < n; i++)
  {
    int di = (short)(I - V29RX_DEC_IMAP[i]);
    int dq = (short)(Q - V29RX_DEC_QMAP[i]);
    int dist = (short)((di * di >> 15) + (dq * dq >> 16));
    if ((short)dist < (short)min)
    {
      min = dist;
      best = i;
    }
  }

  /* report the reference point and code the phase step to the previous symbol */
  *mag = V29RX_DEC_MAG[best];
  *angle = V29RX_DEC_ANGLE[best];
  pidx = (best & 7) - (unsigned short)h->idx;
  h->idx = (short)(best & 7);
  ret = V29RX_DEC_PMAP[pidx & 7];
  if (rate != V29RC_7200)
    ret |= best & 8;
  return ret;
}

/* ===== detectors */
/**
 * @brief Signal-to-noise estimate of the receiver.
 * @return `14 - fse.decErrAvg` (sign-extended 16 bit); RxHdxDataV29() / RxHdxPrtcolV29() set the "SNR low" flag when it is <= 8
 * @bug `fse.decErrAvg` is the equalizer's smoothed decision error on a scale where 0x3fff means "no carrier" (see DataCarrierDetectV29()), but it is
 *      subtracted from 14 as if it were a dB figure: any average above 5 makes the result <= 8 and sets the low-SNR flag (not run; docs/bugs.md only
 *      records the formula).
 */
NOINL int GetSNRV29(V29Rx* o) { return (short)(0xe - (unsigned short)o->bundle->fse.decErrAvg); }

/** @brief Carrier flag of the training states: the equalizer PLL enable ANDed with the AGC detect flag. */
NOINL int CarrierDetectV29(V29Rx* o)
{
  V29RxBundle* b = o->bundle;
  return b->sre.pllEnable & (int)b->agc.detect;
}

/** @brief Nonzero once the epoch detector has forced the equalizer update (the epoch was found). */
NOINL int EpochDetectV29(V29Rx* o) { return o->bundle->fse.eqForce != 0; }

/**
 * @brief Smooth the equalizer's decision error over the first 50 data blocks and judge the link quality.
 * @return the carrier flag `agc.detect & sre.pllEnable`, or 2 when there is no carrier (the caller drops the block's symbols)
 *
 * The first block seeds `decErrSmooth`; blocks 2 .. 50 form a one-pole average (weights 0xccd / 0x7333 in Q15); after block 50 `qualityOk` is set if the
 * average is not above `errLimit`.
 */
NOINL int QualityDetectV29(V29Rx* o)
{
  V29RxBundle* b = o->bundle;
  int pllOn = b->sre.pllEnable;
  short det = (short)b->agc.detect;
  short decErr = b->fse.decErrAvg;
  int r = (short)(det & pllOn);
  unsigned short blocks;

  if (r == 0)
  {
    if (DBG)
      dsplibs_debug_printf("V29 Dec error too big... unreliable data\n");
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
 * @warning When the V.21 carrier search is active the block is copied into the 0x140 byte `hdx.buf2`: `n` above 160 overruns it.
 *
 * Starts from `agc.detect & sre.pllEnable` and then depends on the mode flag `hdx.s1c` (set by the T.30 receiver):
 * - 0: after 1000 symbols (`loop.cnt` above 0x3e7) the carrier is dropped when the decoder error is above 0x3fff, otherwise only bit 0 of the flag is kept;
 * - nonzero: the carrier search is latched on (`toneSearch`) once the decoder error is above 0x3fff or bit 0 of the flag is clear; it runs on a copy of the
 *   block (own AGC + tone detector `mtd2`): a nonzero detector result resets `sinceTone`, otherwise the block length is added, and more than 0x4ff
 *   accumulated samples (about 160 ms) drop the carrier (debug text "V29: V21 Carrier detected").
 * Finally, if the energy test is enabled, a block RMS more than 8 dB below the reference level (0x32fe / 0x8000) drops the carrier; the reference level is
 * refreshed every second block.
 */
NOINL int DataCarrierDetectV29(V29Rx* o, short* in, unsigned short n)
{
  V29RxBundle* b = o->bundle;
  V29RxHdx* h = o->hdx;
  int ebx = (short)((short)b->agc.detect & b->sre.pllEnable);
  short decErr = b->fse.decErrAvg;

  if (h->s1c == 0)
  {
    if (b->loop.cnt > 0x3e7)
    {
      if (decErr > 0x3fff)
        ebx = 0;
      else
        ebx &= 1;
    }
    if (decErr > 0x3fff)
    {
      if (DBG)
        dsplibs_debug_printf("V29 Decoder error too big... no carrier\n");
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
          dsplibs_debug_printf("V29: V21 Carrier detected\n");
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
 * @warning In state START the input is halved into the 0x140 byte `hdx.buf`: `n` above 160 overruns it.  More than 164 timing-recovery output samples are
 *          only reported ("ERROR: SRE buffer violation"); the output buffer `bundle->buf2` is 0x148 bytes, so a larger count has already overrun it
 *          (docs/bugs.md, section K).
 *
 * The AGC detect flag gates the timing loop and the equalizer (see the file header).  In state START a tone killer and a tone detector run on a halved copy of
 * the block; a detected tone (a V.21 preamble, say) aborts the block.
 */
NOINL int DemodDataV29(V29Rx* o, short* in, short* out, unsigned short n)
{
  V29RxBundle* b = o->bundle;
  V29RxHdx* h;
  int detect;
  unsigned int produced;
  unsigned int cnt;

  FPM_AGC_agc(&b->agc, in, n);
  detect = (int)b->agc.detect;
  h = o->hdx;

  /* start state: a tone on the line is not a V.29 carrier */
  if (h->state == V29RX_STATE_START)
  {
    unsigned short i;
    short* const buf = h->buf;
    for (i = 0; i < n; i++)
      buf[i] = (short)(in[i] >> 1);
    FPM_TONE_kill(h->tone, buf, (short)n);
    if ((unsigned short)FPM_MTD_detect(h->mtd, buf, (short)n) != 0)
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
  b->fse.eqEnable = detect & b->eqGate;
  b->fse.pllEnable = detect & b->pllGate;
  return (int)(unsigned short)FAXFSE_receive(&b->fse, b->buf2, out, (unsigned short)cnt);
}

/* ===== state machine */
/**
 * @brief Advance the receiver to the next state and install its handler.
 * @param o  receiver engine
 *
 * START -> EPOCH_DET (3 blocks), EPOCH_DET -> PROTOCOL (8 blocks; advances the AGC alpha / beta table pointers), PROTOCOL -> DATA (freezes the AGC),
 * DATA -> IDLE, IDLE -> DATA (restores the connect code).  Any other state value records the internal-error status (b[0] = 3, flag 2) and changes nothing
 * else.  Each case prints "V29RX_STATE_<name of the state left>" at debug level > 1 before it acts.
 * @note COMPILER_BARRIER() keeps the short stores of `count` / `state` apart: the optimiser merges them into one dword store only with strict aliasing
 *       (compared on the objects built with and without the flag).
 */
NOINL void RxNextStateV29(V29Rx* o)
{
  V29RxHdx* h = o->hdx;
  switch (h->state)
  {
  case V29RX_STATE_START:
    if (DBG)
      dsplibs_debug_printf("V29RX_STATE_START\n");
    h->count = 3;
    COMPILER_BARRIER();
    h->fn = RxHdxEpochDetV29;
    h->state = V29RX_STATE_EPOCH_DET;
    o->st.b[2] &= 0xfe;
    o->st.b[1] &= 0xfe;
    return;
  case V29RX_STATE_EPOCH_DET:
  {
    V29RxBundle* b;
    if (DBG)
      dsplibs_debug_printf("V29RX_STATE_EPOCH_DET\n");
    h->count = 8;
    COMPILER_BARRIER();
    b = o->bundle;
    h->fn = RxHdxPrtcolV29;
    h->state = V29RX_STATE_PROTOCOL;
    o->st.b[2] &= 0xfe;
    o->st.b[1] &= 0xfe;
    b->agc.alpha = b->agc.alpha + 1;
    b->agc.beta = b->agc.beta + 1;
    return;
  }
  case V29RX_STATE_PROTOCOL:
    if (DBG)
      dsplibs_debug_printf("V29RX_STATE_PROTOCOL\n");
    FPM_AGC_Freeze(&o->bundle->agc);
    h = o->hdx;
    h->count = 0;
    h->fn = RxHdxDataV29;
    h->state = V29RX_STATE_DATA;
    o->st.b[2] &= 0xfe;
    o->st.b[1] |= 1;
    return;
  case V29RX_STATE_DATA:
    if (DBG)
      dsplibs_debug_printf("V29RX_STATE_DATA\n");
    h->fn = RxHdxIdleV29;
    h->state = V29RX_STATE_IDLE;
    h->count = 0;
    h->hold = 0;
    o->st.b[2] |= 1;
    o->st.b[1] &= 0xfe;
    return;
  case V29RX_STATE_IDLE:
    if (DBG)
      dsplibs_debug_printf("V29RX_STATE_IDLE\n");
    h->fn = RxHdxDataV29;
    h->state = V29RX_STATE_DATA;
    o->st.b[2] &= 0xfe;
    o->st.b[1] |= 1;
    o->st.b[0] = (unsigned char)(7 - (h->rate != V29RC_7200));
    return;
  default:
  {
    unsigned char f;
    if (DBG)
      dsplibs_debug_printf("V29RX_DEFAULT, %d\n", (int)h->state);
    f = o->st.b[1];
    o->st.b[2] &= 0xfe;
    o->st.b[0] = V29RX_MSG_ERROR;
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
 * Runs the demodulator for its carrier detection; on carrier it sets the carrier flag and moves to EPOCH_DET.
 */
NOINL int RxHdxStartV29(V29Rx* o, short* in, short* out, unsigned short* n)
{
  const unsigned short n0 = *n; /* every input is read before the first store (the result must not depend on the aliasing rules) */

  o->st.b[1] &= 0xdf;
  o->st.b[0] = V29RX_MSG_START;
  DemodDataV29(o, in, out, n0);
  if (CarrierDetectV29(o) == 0)
  {
    *n = 0;
    return 0;
  }
  o->st.b[1] |= 0x20;
  RxNextStateV29(o);
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
NOINL int RxHdxEpochDetV29(V29Rx* o, short* in, short* out, unsigned short* n)
{
  V29RxHdx* h;
  unsigned short cnt;

  DemodDataV29(o, in, out, *n);
  *n = 0;
  if (CarrierDetectV29(o) != 0)
  {
    o->st.b[1] |= 0x20;
    o->st.b[0] = V29RX_MSG_PROTOCOL;
    h = o->hdx;
    cnt = (unsigned short)(h->count - 1);
    h->count = (short)cnt;
    if ((short)cnt > 0)
    {
      if ((unsigned short)EpochDetectV29(o) == 0)
        return 0;
    }
    RxNextStateV29(o);
    return 0;
  }
  h = o->hdx;
  h->fn = RxHdxErrorV29;
  h->state = V29RX_STATE_ERROR;
  o->st.b[0] = V29RX_MSG_LOSS_CARRIER;
  o->st.b[1] = (unsigned char)((o->st.b[1] | 2) & 0xdf);
  return 0;
}

/**
 * @brief Handler of state PROTOCOL: equalizer training; the decided symbols are descrambled and returned.
 * @return the number of symbols produced in the last block of the state, else 0; `*n` is set to 0
 *
 * When the block budget (`hdx.count`) runs out the connect code of the rate is stored, the "SNR low" flag is set if GetSNRV29() <= 8, and the state advances
 * to DATA.  Loss of carrier moves to ERROR with the loss-of-carrier status.
 */
NOINL int RxHdxPrtcolV29(V29Rx* o, short* in, short* out, unsigned short* n)
{
  V29RxHdx* h;
  short snr;
  unsigned int ret = (unsigned short)DemodDataV29(o, in, out, *n);

  DescrambleDataV29(o, (unsigned short*)out, (unsigned short)ret);
  *n = 0;
  if (CarrierDetectV29(o) != 0)
  {
    unsigned short cnt;
    o->st.b[1] |= 0x20;
    o->st.b[0] = V29RX_MSG_PROTOCOL;
    h = o->hdx;
    cnt = (unsigned short)(h->count - 1);
    h->count = (short)cnt;
    if ((short)cnt > 0)
      return 0;
    o->st.b[0] = (unsigned char)(7 - (h->rate != V29RC_7200));
    snr = (short)GetSNRV29(o);
    COMPILER_BARRIER();
    if (snr <= 8)
      o->st.b[1] |= 0x80;
    RxNextStateV29(o);
    return (short)ret;
  }
  h = o->hdx;
  h->fn = RxHdxErrorV29;
  h->state = V29RX_STATE_ERROR;
  o->st.b[0] = V29RX_MSG_LOSS_CARRIER;
  o->st.b[1] = (unsigned char)((o->st.b[1] | 2) & 0xdf);
  return 0;
}

/**
 * @brief Handler of state DATA: demodulate, descramble and return the data symbols.
 * @return the number of symbols produced (0 when the carrier is lost, the input is on hold, or QualityDetectV29() reports no carrier); `*n` is set to 0
 *
 * Sets the carrier flag and the data status, then asks DataCarrierDetectV29(); without carrier (or with `hdx.hold` set) the block is discarded.  Afterwards
 * the "SNR low" flag (b[1] bit 7) is recomputed from GetSNRV29().
 */
NOINL int RxHdxDataV29(V29Rx* o, short* in, short* out, unsigned short* n)
{
  unsigned int count;
  int ret;
  short snr;
  const unsigned short n0 = *n;

  o->st.b[1] |= 0x20;
  o->st.b[0] = V29RX_MSG_DATA;
  COMPILER_BARRIER();
  if ((short)DataCarrierDetectV29(o, in, n0) == 0 || o->hdx->hold != 0)
  {
    o->st.b[1] &= 0xdf;
    *n = 0;
    return 0;
  }

  count = (unsigned short)DemodDataV29(o, in, out, *n);
  DescrambleDataV29(o, (unsigned short*)out, (unsigned short)count);
  *n = 0;
  ret = ((short)QualityDetectV29(o) != 2) ? (int)count : 0;

  /* refresh the low-SNR flag */
  o->st.b[1] &= 0x7f;
  snr = (short)GetSNRV29(o);
  COMPILER_BARRIER();
  if (snr <= 8)
    o->st.b[1] |= 0x80;
  return (short)ret;
}

/**
 * @brief Handler of state IDLE: keep demodulating without output until the decision error is small again.
 * @return 0 (no symbols); `*n` is set to 0
 *
 * Returns to DATA (via RxNextStateV29()) when the carrier is present and `fse.decErrAvg` is at most 0x1fff.
 */
NOINL int RxHdxIdleV29(V29Rx* o, short* in, short* out, unsigned short* n)
{
  DemodDataV29(o, in, out, *n);
  *n = 0;
  COMPILER_BARRIER();
  o->st.b[1] &= 0xdf;
  o->st.b[0] = V29RX_MSG_IDLE;
  if (CarrierDetectV29(o) != 0)
    o->st.b[1] |= 0x20;
  if (o->st.b[1] & 0x20)
  {
    if (o->bundle->fse.decErrAvg <= 0x1fff)
    {
      RxNextStateV29(o);
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
NOINL int RxHdxErrorV29(V29Rx* o, short* in, short* out, unsigned short* n)
{
  const unsigned short n0 = *n;
  o->st.b[1] |= 2;
  DemodDataV29(o, in, out, n0);
  *n = 0;
  return 0;
}

/* ===== create / delete / modem / control / status */
/**
 * @brief Create or re-initialise a V.29 receiver.
 * @param o    an existing engine to re-initialise in place (its blocks and sub-objects are reused), or NULL to allocate the engine, the half-duplex block and
 *             the bundle
 * @param cfg  six configuration words (V29RxCfg), or NULL for the defaults V29RX_CFG; `cfg[1]` low half is the bit rate (7200 or 9600), `cfg[5]` the user word
 * @return the engine, in state START with status "start of protocol"
 *
 * Builds the start-state tone killer + tone detector, the V.21 carrier detector with its own AGC, the matched filter (up 9, down 10, 0x10e taps), the AGC,
 * the timing recovery and the equalizer (with V29RX_epoch_det() as the first decision callback), the receive scrambler (4 bits per symbol at 9600, 3 at
 * 7200) and the decision-loop state, and clears the work buffers.  Sub-objects are allocated only when the engine (bundle) or the half-duplex block is new.
 * @warning The allocation results are not checked (a failed allocation dereferences NULL, docs/bugs.md section K).  A rate other than 7200 / 9600 selects the
 *          9600 tables and sets an error status that the common status store a few lines later overwrites: the error is never visible to the caller.
 *          The half-duplex mode flag `hdx.s1c` is zeroed only when the half-duplex block is allocated here; the T.30 receiver writes it afterwards.
 */
V29Rx* V29RX_create(V29Rx* o, const int* cfg)
{
  int objFresh = 0, hdxFresh = 0;
  V29RxHdx* h;
  V29RxBundle* b;
  int i, rate;
  int mtdCfg[3], toneCfg[9], mrfCfg[4];
  FaxSreCfg sre;
  FaxFseCfg fse;
  FaxSdmCfg sdmCfg;
  int cc[6];
  int user;

  if (DBG)
    dsplibs_debug_printf("V.29 RX Create ");
  if (o == NULL)
  {
    if (DBG)
      dsplibs_debug_printf("New allocation\n");
    o = (V29Rx*)sysdep_malloc((unsigned int)sizeof(V29Rx));
    o->hdx = NULL;
    o->bundle = NULL;
    objFresh = 1;
  }
  if (DBG)
    dsplibs_debug_printf("\n");

  /* take a private copy of the configuration: the caller may pass the engine's own `o->cfg` (re-create), and the words must not be re-read after the stores */
  if (cfg == NULL)
    cfg = V29RX_CFG;
  for (i = 0; i < 6; i++)
    cc[i] = cfg[i];
  for (i = 0; i < 6; i++)
    o->cfg.w[i] = cc[i];
  user = cc[5];

  /* half-duplex block: allocate on first use, then (re)build its detectors */
  h = o->hdx;
  if (h == NULL)
  {
    h = (V29RxHdx*)sysdep_malloc((unsigned int)sizeof(V29RxHdx));
    o->hdx = h;
    h->mtd = NULL;
    h->tone = NULL;
    h->buf = (short*)sysdep_malloc(0x140);
    h->buf2 = (short*)sysdep_malloc(0x140);
    h->mtd2 = NULL;
    h->s1c = 0;
    hdxFresh = 1;
  }

  /* tone detector of the start state: V29_MTD_COEFF, {2, 0x199a}, FPM_MTD_CFG[2] low half 0x32 */
  mtdCfg[0] = CFGP(V29_MTD_COEFF);
  mtdCfg[1] = (int)(0x199au << 16 | 2u);
  mtdCfg[2] = (int)(((unsigned int)FPM_MTD_CFG_raw[2] & 0xffff0000u) | 0x32u);
  h->mtd = FPM_MTD_create(h->mtd, mtdCfg);
  /* tone killer: FPM_TONE_CFG with the low half of word 0 = 0x6a4 */
  for (i = 0; i < 9; i++)
    toneCfg[i] = FPM_TONE_CFG_raw[i];
  toneCfg[0] = (int)(((unsigned int)toneCfg[0] & 0xffff0000u) | 0x6a4u);
  h->tone = FPM_TONE_create(h->tone, toneCfg);
  h = o->hdx;
  h->state = V29RX_STATE_START;
  h->count = 0;
  h->hold = 0;
  h->fn = RxHdxStartV29;

  /* carrier detector: V21_CHAN2_MTD_COEFF, {2, 0x4ccd}, FPM_MTD_CFG[2] low half 0x12c */
  mtdCfg[0] = CFGP(V21_CHAN2_MTD_COEFF);
  mtdCfg[1] = (int)(0x4ccdu << 16 | 2u);
  mtdCfg[2] = (int)(((unsigned int)FPM_MTD_CFG_raw[2] & 0xffff0000u) | 0x12cu);
  h->mtd2 = FPM_MTD_create(h->mtd2, mtdCfg);
  FPM_AGC_init(&h->agc, AGCv29_CFG, hdxFresh);
  h->sinceTone = 0;
  h->toneSearch = 0;

  /* bit rate */
  if ((short)cc[1] == RATE_7200)
  {
    rate = V29RC_7200;
  }
  else if ((short)cc[1] == RATE_9600)
  {
    rate = V29RC_9600;
  }
  else
  {
    rate = V29RC_9600;
    o->st.b[1] |= 2;
    o->st.b[0] = V29RX_MSG_ERROR;
  }
  h->rate = (short)rate;

  /* bundle: allocate on first use, then (re)build the signal path */
  b = o->bundle;
  if (b == NULL)
  {
    b = (V29RxBundle*)sysdep_malloc((unsigned int)sizeof(V29RxBundle));
    o->bundle = b;
    b->buf1 = (short*)sysdep_malloc(0x140);
    b->buf2 = (short*)sysdep_malloc(0x148);
  }
  /* matched filter: up 9, down 10, 0x10e taps (the same at both rates) */
  mrfCfg[0] = (int)(9u | (0xau << 16));
  mrfCfg[1] = CFGP(V29RX_MRF_FILT);
  mrfCfg[2] = (int)(((unsigned int)FPM_MRF_CFG_raw[2] & 0xffff0000u) | 0x10eu);
  mrfCfg[3] = user;
  FPM_MRF_init(&b->mrf, mrfCfg, objFresh);
  FPM_AGC_init(&b->agc, AGCv29_CFG, objFresh);

  /* timing recovery */
  sre = fax_sre_cfg_default;
  COMPILER_BARRIER();
  sre.sps = 3;
  sre.periodsTrack = 3;
  sre.periodsLock = 0x10;
  sre.startCount = 0x2b;
  sre.unlockGain = 0x2000;
  sre.lockGain = 0x4000;
  sre.nt = 0xb4;
  sre.coefTab = CFGP(V29RX_SRE_FILT);
  sre.tedCoef = CFGP(V29RX_XB_COFFS);
  sre.corrCosTab = CFGP(V29RX_XCLOCK);
  sre.corrSinTab = CFGP(V29RX_YCLOCK);
  sre.propGainTab = CFGP(V29RX_SRE_PLLK1);
  sre.intGainTab = CFGP(V29RX_SRE_PLLK2);
  sre.errOn = 2;
  sre.errOff = 1;
  sre.unlockThresh = 0x2666;
  sre.lockThresh = 0xc8;
  sre.levelThresh = (short)((int)b->agc.a.s[0] / 6);
  sre.levelLen = 9;
  sre.w34 = user;
  FAXSRE_init(&b->sre, &sre, objFresh);
  b->sre.ppmTick = 0x30;
  b->sre.ppmPeriod = 0x2580;
  b->sre.ppmStep = (short)(1000000 / ((int)sre.sps * 0x2580));
  b->sre.ppmNMax = 0x68;

  /* equalizer */
  fse = fax_fse_cfg_default;
  COMPILER_BARRIER();
  fse.sps = 3;
  fse.iCoefTab = CFGP(V29RX_FSE_IFILT);
  fse.qCoefTab = CFGP(V29RX_FSE_QFILT);
  fse.nt = 0x31;
  fse.mu[0] = 0;
  fse.mu[1] = 0;
  fse.carrierTab = CFGP(V29RX_CRR_TABLE);
  fse.crrLen = 0x48;
  fse.crrAdj = 0x11;
  fse.settleCount = 0x1c8;
  fse.errThreshHi = 0x199a;
  fse.errThreshLo = 0xccd;
  fse.propTab = CFGP(V29RX_FSE_PLLK1);
  fse.intTab = CFGP(V29RX_FSE_PLLK2);
  fse.hist = &b->loop;
  fse.cb = V29RX_epoch_det;
  fse.w34 = user;
  FAXFSE_init(&b->fse, &fse, objFresh);

  /* receive scrambler: bits per symbol 4 (9600) / 3 (7200), taps 0x12 / 0x17 */
  sdmCfg.s[0] = (short)(4 - (rate == V29RC_7200));
  sdmCfg.s[1] = 0x12;
  sdmCfg.s[2] = 0x17;
  SDM_init(&b->sdm, &sdmCfg);

  /* work buffers, quality detector and decision-loop state */
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
  if (rate == V29RC_7200)
    b->errLimit = 0x320;
  else if (rate == V29RC_9600)
    b->errLimit = 0x2bc;
  b->loop.avgA = 0;
  b->energyTest = 1;
  b->refBlocks = 0;
  b->refLevel = 0;
  b->loop.avgB = 0;
  b->loop.symI0 = 0;
  b->loop.symQ0 = 0;
  b->loop.symI1 = 0;
  b->loop.symQ1 = 0;
  b->loop.symI2 = 0;
  b->loop.symQ2 = 0;
  b->w0 = 1;
  b->loop.rate = rate;
  b->loop.idx = 0;
  b->loop.reg = 0x6a;
  b->loop.phaseCnt = 0;
  b->loop.s1a = 0;
  b->loop.prev = 0;
  b->loop.cnt = 0;

  /* gates and flags */
  b->sreGate = 1;
  b->pllGate = 1;
  b->wc = 0;
  b->w10 = 0;
  b->w14 = 1;
  b->w18 = 1;
  b->w1c = 0;
  b->eqGate = 1;
  b->w24 = 0;

  /* status word and the copies of the equalizer pointers */
  o->st.w = 0;
  o->st.b[1] |= 0x50;
  o->st.b[0] = V29RX_MSG_START;
  COMPILER_BARRIER();
  o->fseI = b->fse.outI;
  o->fseQ = b->fse.outQ;
  o->fseIdx = &b->fse.idx;
  o->fseTapsI = b->fse.tapsI;
  o->fseTapsQ = b->fse.tapsQ;
  o->fseNt = b->fse.cfg.nt;
  o->s3c = 0;
  o->s48 = 0;
  o->w34 = 0;
  o->w38 = 0;
  o->w40 = 0;
  o->w44 = 0;
  return o;
}

/** @brief Release a receiver created by V29RX_create(): equalizer, timing recovery, matched filter, work buffers, detectors and the three blocks. */
void V29RX_delete(V29Rx* o)
{
  V29RxBundle* b = o->bundle;
  V29RxHdx* h;
  FAXFSE_free(&b->fse);
  FAXSRE_free(&b->sre);
  FPM_MRF_free(&b->mrf);
  sysdep_free(b->buf2);
  sysdep_free(b->buf1);
  sysdep_free(b);
  h = o->hdx;
  FPM_MTD_delete(h->mtd);
  FPM_TONE_delete(h->tone);
  sysdep_free(h->buf);
  sysdep_free(h->buf2);
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
 * @return the status word `st` (b[0] status code, b[1] flags, see V29Rx::st)
 *
 * Calls the handler of the current state repeatedly (a handler may change the state and leave part of the block unconsumed) until the block is used up.
 * @warning `out` must be large enough for the symbols of the whole block.
 */
int V29RX_modem(V29Rx* o, short* in, short* out, unsigned short* n)
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
int V29RX_control(V29Rx* o, const FaxV29Ctl* ctl)
{
  unsigned char d, c;
  int w4;
  V29RxHdx* h;

  if (ctl == NULL)
    return 0;
  w4 = ctl->w4; /* the control record is read before any store (the result must not depend on the aliasing rules) */
  d = ctl->b0d;
  COMPILER_BARRIER();
  o->cfg.w[2] = w4;
  h = o->hdx;
  h->hold = (d >> 4) & 1;
  if (d & 2)
    V29RX_create(o, o->cfg.w);
  c = ctl->b0c;
  if (c & 8)
    o->bundle->w0 = 0;
  if (c & 0x20)
    o->bundle->eqGate = 0;
  return 1;
}

/**
 * @brief Fill a status record.
 * @param o   receiver engine
 * @param st  record to fill, or NULL
 * @return 0 when `st` is NULL, else 1
 *
 * The record holds the configured rate (`s0`), the configured rate word 2 (`s4`, `s12`), the "SNR acceptable" flag (`s6`, from b[1] bit 7), the SNR estimate
 * (`s8`, GetSNRV29()) and the flag byte `b14` rebuilt from the bundle words (bit 1 = w18 & 1, bit 3 = (w0 == 0), bit 5 = equalizer gate closed, bits 4 and 6
 * always set).
 * @warning Only bit 0 of `st->b15` is cleared; its other bits keep the caller's previous contents.
 */
int V29RX_status(V29Rx* o, FaxV29Status* st)
{
  V29RxBundle* b;
  unsigned char b14;
  short cfg0, cfg2, snr, ok;
  int w0, w18, eqGate;
  if (st == NULL)
    return 0;

  /* every member of the engine is read before the first store into the record (the result must not depend on the aliasing rules) */
  cfg0 = o->cfg.s[0];
  cfg2 = o->cfg.s[2];
  ok = (o->st.b[1] & 0x80) == 0;
  snr = (short)GetSNRV29(o);
  b = o->bundle;
  w0 = b->w0;
  w18 = b->w18;
  eqGate = b->eqGate;
  b14 = (unsigned char)(((w18 & 1) << 1) | ((w0 == 0) << 3) | 0x10 | ((eqGate == 0) << 5) | 0x40);
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

/* ===== VMI wrappers */
/**
 * @brief Initialise a VMI object as a V.29 receiver (no ECM, simple packing).
 * @param m       VMI object to fill
 * @param rate    bit rate stored in the configuration (7200 / 9600)
 * @param unused  ignored
 * @param user    user word: stored as the engine handle slot and as cfg word 5 (copied into the filter configurations)
 * @return 0x80
 *
 * Allocates the 6 word configuration block (V29RX_CFG with the rate patched in) and stores the VMI geometry: 0x140, 0xa5, 0, 10.
 * @warning The allocation result is not checked.
 */
int fax_init_vmi_v29rx(FaxVmiV29* m, unsigned short rate, int unused, void* user)
{
  V29RxCfg* c = (V29RxCfg*)sysdep_malloc(sizeof(V29RxCfg));
  int i;
  (void)unused;
  if (2 < dsplibs_debug_level)
    dsplibs_debug_printf("Initializing VMI_V29_RX Modem No ECM (Simple Packing)\n");
  for (i = 0; i < 5; i++)
    c->w[i] = V29RX_CFG[i];
  c->w[5] = (int)(uintptr_t)user; /* 32-bit copy of the user word (only ever copied on into the filter configurations) */
  c->s[2] = (short)rate;

  /* VMI object: geometry, engine handle slot, configuration */
  m->w0 = 0;
  m->s08 = 0x140;
  m->s0a = 0xa5;
  m->s0c = 0;
  m->s0e = 10;
  m->eng = (struct V29Tx*)user; /* FaxVmiV29::eng is typed for the transmitter: the receiver stores its V29Rx * through this cast */
  m->u4.init1 = 1;
  m->u10.cfg = (V29Cfg*)c; /* likewise: the receiver's 6 word V29RxCfg block */
  return 0x80;
}

/**
 * @brief VMI create: build the receiver engine and store it in the VMI object.
 * @param m    VMI object; `m->eng` is the engine to re-initialise (NULL: allocate)
 * @param cfg  configuration words, or NULL for V29RX_CFG
 * @return the bits-per-symbol code 3 (7200) / 4 (any other rate) - NOT the engine (unlike v27rx_create()); the engine is stored in `m->eng`
 */
int v29rx_create(FaxVmiV29* m, const int* cfg)
{
  int w[6], i, r;
  V29Rx* e;

  if (cfg == NULL)
    cfg = V29RX_CFG;
  for (i = 0; i < 6; i++)
    w[i] = cfg[i];
  e = V29RX_create((V29Rx*)m->eng, w);
  m->s0c = 0;
  m->eng = (struct V29Tx*)e;
  m->s0e = 0;
  r = 3 + ((short)w[1] != RATE_7200);
  m->u10.s10 = (short)r;
  return r;
}

/** @brief VMI delete: release the engine in `m->eng`. */
void v29rx_delete(FaxVmiV29* m) { V29RX_delete((V29Rx*)m->eng); }

/**
 * @brief VMI process: run V29RX_modem() on `in`, writing the symbols into the VMI object's bit buffer.
 * @param outn  receives the number of symbols produced
 * @param n     in: number of input samples; out: 0
 * @return the status word of V29RX_modem()
 */
int v29rx_process(FaxVmiV29* m, short* in, unsigned short* outn, unsigned short* n)
{
  int r = V29RX_modem((V29Rx*)m->eng, in, m->u4.bits, n);
  *outn = *n;
  *n = 0;
  return r;
}

/** @brief VMI status: V29RX_status() of the engine in `m->eng`. */
int v29rx_status(FaxVmiV29* m, FaxV29Status* st) { return V29RX_status((V29Rx*)m->eng, st); }
/** @brief VMI control: V29RX_control() of the engine in `m->eng`. */
int v29rx_control(FaxVmiV29* m, const FaxV29Ctl* ctl) { return V29RX_control((V29Rx*)m->eng, ctl); }

/** @brief VMI message: `*out` = status message `idx` (0 .. 7, V29RX_MESG); NULL for any other index. */
void v29rx_message(FaxVmiV29* m, int idx, const char** out)
{
  (void)m;
  if ((unsigned int)idx > 8u)
    *out = NULL;
  else if ((unsigned int)idx == 8u)
    *out = NULL; /* index 8 is one past the message table */
  else
    *out = V29RX_MESG[idx & 0xff];
}
