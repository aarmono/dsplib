/**
 * @file v34_modem_init.c
 * @brief V.34 modem initialisation for the start of a handshake: v34modeminit(), the V.90 MP-word builder getMPrecvdBits(), VPcmV34SetTxScale() and
 *        cEncodeChar().
 *
 * - v34modeminit(obj) puts the V.34 object (tagV34Object, accessed as `V34Obj`; the undeclared record and buffer regions by byte offset, v34_access.h) into the state
 *   needed for the start of the handshake: it clears the three 0x2c-byte records at 0xa81c, the retrain-detector and counter words and two 12-word
 *   buffers, sets the transmit scale, configures the modulator and demodulator for 2400 baud at 1800 Hz (the demodulator arm of V34SetupDemodulator() is
 *   inlined here, with the same print), then selects the role arm: the two roles (modeChar_ == V34ROLE_CALL or not) differ in the flag words, the
 *   1200 Hz (call) or 2400 Hz (answer) modulator carrier and the filter table; both clear the FSK / V.21 receive
 *   state and the 100-word buffer at 0xaae6, set up the 600 baud DPSK modulator and, after preinitdigital() and txinit(), start the tone detector on the
 *   2400 Hz (calling) or 1200 Hz (answering) tone.
 * - getMPrecvdBits(obj) builds the 8-word MP block (`waa0c_` ..) from the V.90 engine context, runs txrxdmainit() on it, selects the maximum upstream rate
 *   and encodes that rate into bits 6 .. 9 of the first INFO word `waa3c_`.
 * - VPcmV34SetTxScale() sets the transmit gain `txGain_` to 0x16a1 and prints it; cEncodeChar() is the character scrambler used by edprintf().
 *
 * Behaviour that is easy to lose:
 * - The transmit flag word `txFlags_` is stored (not OR-ed) early with 4 and again in the role arm with 4 (call) or 5 (answer); `rxFlags_` gets 0 (call)
 *   or 4 (answer) and is then OR-ed with 0xa00 and, at the very end, with 0x200 (16-bit read-modify-writes).  The functions called in between
 *   (preinitdigital(), V34SetupModulator(), rxinit(), txinit(), detectorinit(), txrxdmainit(), edprintf(), VPcmV34SetTxScale()) do not touch `txFlags_`.
 * - getMPrecvdBits(): when `pcmMode_` > 0 the MP word is built and txrxdmainit() runs if its bit 0 is set; step 2 then runs txrxdmainit() again if the
 *   word at 0xaa0c has bit 0 set (it may be stale), so with the flag set and bit 0 set it runs twice.  The V.90 parameter block `v90Params_` is only
 *   dereferenced when the first two tests of the "regular ISP" condition pass.  The rate is clamped to `maxTxRateIndexForSensitiveIsp` * 0x960 (32-bit
 *   product, signed minimum) before the print.
 * - cEncodeChar() uses only the low byte of the offsetarr entry; its index wraps on equality with 9, so an index above 9 keeps counting and reads beyond
 *   the table (see its @warning).
 *
 * Table addresses stored into the object (hsine1800, bpv22high / bpv22low) are those of src/v34/v34_init_tables.c and src/v34/v34_frame_tables.c.
 * getMPrecvdBits() is an ordinary C function so that it can be called from tests.
 */

#include <limits.h>
#include <stddef.h>
#include <stdint.h>

#include "v34/v34_init_tables.h"  /* hsine1800, offsetarr */
#include "v34/v34_symbol_rates.h" /* V34_BAUD_*, V34_CARRIER_* */
#include "v34/v34_frame_tables.h" /* c1200_, c2400_, bpv22high, bpv22low */
#include "core/dsp_host.h"
#include "v34/v34_api.h"
#include "vpcm_v34_api.h"

#include "v34/v34_access.h" /* U16 (byte-offset access to the undeclared regions), AS_U16 */
#include "v34/v34_role.h"   /* V34ROLE_* (the values of modeChar_) */

#include "core/dp_params.h"         /* the parameter block (V34ModParObj) */
#include "v34/v34_v90params_view.h" /* V90ParObj */
#include "v34/v34_flomodem_view.h"  /* PcmFlo47Obj: the engine context */
#include "v34/v34_object_view.h"    /* V34Obj */

/* ====================================================================== */
/* VPcmV34SetTxScale                                                      */
/* ====================================================================== */

/**
 * @brief Set the transmit scale: `txGain_` = 0x16a1 (16-bit store), then print it with edprintf().
 * @param obj  V.34 object
 */
void VPcmV34SetTxScale(V34Obj* obj)
{
  (obj)->tx.txGain_ = 0x16a1;
  edprintf("VPcmV34SetTxScale: tx scale set to %d\r\n", 0x16a1);
}

/* ====================================================================== */
/* cEncodeChar                                                            */
/* ====================================================================== */

/**
 * @brief Encode one character of an edprintf() string: add the current offsetarr entry (low byte) and '0' modulo 256.
 * @param c  character
 * @return the encoded byte, sign-extended
 * @note  The offset index is the one edprintf() uses (src/v34/v34_tx_level.c); it advances after the read and wraps to 0 when it equals 9.  Unused.
 * @warning The index wraps only on equality with 9: an index above 9 keeps counting and reads past `offsetarr`.  edprintf() keeps it within 0 .. 9.
 */
int cEncodeChar(char c)
{
  int* idx = v34_ed_index();
  int e = *idx;
  unsigned char cl = (unsigned char)((unsigned char)c + (unsigned char)offsetarr[e] + '0');
  *idx = (e == 9) ? 0 : e + 1;
  return (int)(signed char)cl;
}

/* ====================================================================== */
/* getMPrecvdBits                                                         */
/* ====================================================================== */

/**
 * @brief Build the MP block and the first INFO word for the V.90 / V.92 handshake.
 * @param obj  V.34 object; its engine context supplies the MP fields and its parameter block the maximum upstream rate
 * @note  1. When `pcmMode_` > 0: word 0 of the MP block (`waa0c_`) = type bit 0 | (drn & 0xf) << 6 | (trellis state & 3) << 11 | nonlinear-encoder 0x2000 |
 *        constellation-shaping 0x4000 | 0x8000 flag, then ORed with (word >> 4) & 0x3c; word 1 = rate mask | 0x8000; words 2 .. 7 copied from the context.
 *        txrxdmainit() runs on the block if bit 0 of the built word is set, followed by a chain of "V90mp" prints, each behind a fresh unsigned
 *        `1 < level` test (the chain stops at the first test that fails).
 *        2. txrxdmainit() runs again if bit 0 of word 0 (0xaa0c) is set.
 *        3. `waa3c_` = 0x9dc3 and the bit-queue pointer is set to the INFO word block.
 *        4. The rate is `params` word 0x3c, lowered to `maxTxRateIndexForSensitiveIsp` * 2400 for a sensitive ISP (V.90 context flag set, `pcmMode_` >= 2,
 *        sensitive ISP detected); the print names the ISP class.
 *        5. For a rate below 0x8340 (33600), bits 6 .. 9 of `waa3c_` receive the bit-reversed 4-bit value (rate * 7) >> 14.
 *        6. `waa3e_` = 0x7ffd and the words `waa40_` .. `waa4c_` are cleared.
 * @warning The V.90 parameter block `v90Params_` is dereferenced without a NULL check once the context flag is set and `pcmMode_` >= 2.
 */
void getMPrecvdBits(V34Obj* obj)
{
  PcmFlo47Obj* ctxobj = (PcmFlo47Obj*)obj->ctx_;
  V34Q2* mpout = &obj->q2;

  /* 1. MP block from the context */
  if (obj->pcmMode_ > 0)
  {
    uint32_t w =
        ((uint32_t)((int)(int8_t)ctxobj->mpB_[2] & 3) << 11) | ((uint32_t)((int)(int8_t)ctxobj->mpB_[1] & 0xf) << 6) | (uint32_t)(ctxobj->mpB_[0] != 0);

    if (ctxobj->mpB_[3] != 0)
      w |= 0x2000;
    if (ctxobj->mpB_[4] != 0)
      w |= 0x4000;
    if (ctxobj->mpB_[5] != 0)
      w |= 0x8000;

    obj->q2.waa0c_ = (uint16_t)w;
    obj->q2.waa0e_ = (uint16_t)(ctxobj->mpRate2_ | 0x8000u);
    obj->q2.waa10_ = ctxobj->mpH_[0];
    obj->q2.waa12_ = ctxobj->mpH_[1];
    obj->q2.waa14_ = ctxobj->mpH_[2];
    obj->q2.waa16_ = ctxobj->mpH_[3];
    obj->q2.waa18_ = ctxobj->mpH_[4];
    obj->q2.waa1a_ = ctxobj->mpH_[5];
    obj->q2.waa0c_ = (uint16_t)(((w >> 4) & 0x3cu) | w);

    if ((w & 1u) != 0)
      txrxdmainit(obj->tx.precCoef_, mpout);

    /* nested debug chain: a fresh unsigned level test before each print */
    if (1 < dsplibs_debug_level)
    {
      dsplibs_debug_printf("V90mp - drn = %d\r\n", (int)(int8_t)ctxobj->mpB_[1]);
      if (1 < dsplibs_debug_level)
      {
        dsplibs_debug_printf("V90mp - type = %d\r\n", (int)(int8_t)ctxobj->mpB_[0]);
        if (1 < dsplibs_debug_level)
        {
          dsplibs_debug_printf("V90mp - trellisState = %d\r\n", (int)(int8_t)ctxobj->mpB_[2]);
          if (1 < dsplibs_debug_level)
          {
            dsplibs_debug_printf("V90mp - nonlinearEncoder = %d\r\n", (int)(int8_t)ctxobj->mpB_[3]);
            if (1 < dsplibs_debug_level)
            {
              dsplibs_debug_printf("V90mp - constellationShaping = %d\r\n", (int)(int8_t)ctxobj->mpB_[4]);
              if (1 < dsplibs_debug_level)
              {
                dsplibs_debug_printf("V90mp - rateMask = 0x%X\r\n", (int)(int16_t)ctxobj->mpRate2_);
                if (1 < dsplibs_debug_level)
                  dsplibs_debug_printf("mp->data[0] = 0x%X , mp->data[1] = 0x%X\r\n", (unsigned)obj->q2.waa0c_, (unsigned)obj->q2.waa0e_);
              }
            }
          }
        }
      }
    }
  }

  /* 2. second conversion on the (possibly stale) block */
  if ((obj->q2.waa0c_ & 1u) != 0)
    txrxdmainit(obj->tx.precCoef_, mpout);

  /* 3. first INFO word and its bit queue */
  obj->mp.waa3c_ = 0x9dc3;
  obj->bitQ_ = (struct tag_bitQ*)&obj->mp;

  /* 4. maximum upstream rate, 5. its code in bits 6 .. 9 of the INFO word */
  {
    V34ModParObj* P = obj->params_;
    int32_t rate = (int32_t)P->upstreamMaxRate;
    PcmFlo47Obj* ctx2 = (PcmFlo47Obj*)obj->ctx_; /* re-read */
    void* cfg;

    if (ctx2->flag6120_ == 0 || obj->pcmMode_ < 2 || ((V90ParObj*)(cfg = ctx2->v90Params_))->sensitiveIspDetected == 0)
    {
      edprintf("on get max upstream rate, on regular ISP, returning %d\r\n", (int)rate);
    }
    else
    {
      int32_t m = (int32_t)((uint32_t)((V90ParObj*)cfg)->maxTxRateIndexForSensitiveIsp * 0x960u); /* 32-bit imul */
      if (rate > m)
        rate = m;
      edprintf("on get max upstream rate, on sensitive ISP, returning %d\r\n", (int)rate);
    }

    if (rate < 0x8340)
    { /* signed compare */
      /* rate * 7 as rate * 8 - rate (32-bit wrap), arithmetic shift right 14, low 16 bits */
      int32_t t = (int32_t)((uint32_t)rate * 8u - (uint32_t)rate);
      int r = (int16_t)bitreverse((unsigned short)((t >> 14) & 0xffff), 4);
      uint16_t mask = 0xffbf;
      int k;
      for (k = 0; k < 4; k++)
      {
        uint16_t v = obj->mp.waa3c_;
        if (((r >> k) & 1) == 0)
          v = (uint16_t)(v & mask);
        else
          v = (uint16_t)(~mask | v);
        obj->mp.waa3c_ = v;
        mask = (uint16_t)(mask * 2u | 1u);
      }
    }
  }

  /* 6. rest of the INFO word block */
  obj->mp.waa3e_ = 0x7ffd;
  obj->mp.waa40_ = 0;
  obj->mp.waa46_ = 0;
  obj->mp.waa4a_ = 0;
  obj->mp.waa4c_ = 0;
  obj->mp.waa42_ = 0;
  obj->mp.waa44_ = 0;
  obj->mp.waa48_ = 0;
}

/* ====================================================================== */
/* v34modeminit                                                           */
/* ====================================================================== */

/**
 * @brief Initialise the V.34 object for the start of a handshake (see the file header for the overall effect).
 * @param obj  V.34 object (its modulator sub-object `mod` is configured through V34SetupModulator())
 * @note  Steps: (1) clear the three 0x2c-byte detector records (0x50 and 3000 in their last two words) and set the three threshold words 0x600, 0x800,
 *        0xa00; (2) reset the scalar state (retrain detector, rate-renegotiation fields, counters, `txFlags_` = 4, block length 6, `k56Word_` = 0x8990);
 *        (3) clear the two 12-word buffers; (4) set the transmit scale, configure the modulator for 2400 baud / 1800 Hz, run rxinit() and set up the
 *        demodulator (inlined arm, printed at debug level above 1); (5) the role arm; (6) preinitdigital(), txinit(), start the tone detector (call role:
 *        2400 Hz tone table, otherwise 1200 Hz; threshold 200, 0x32, 0x600) and OR 0x200 into `rxFlags_`.
 */
void v34modeminit(V34Obj* obj)
{
  V34Mod* m = &obj->mod;
  int i;

  /* 1. detector records and thresholds */
  for (i = 0; i < 3; i++)
  {
    DftBin* rec = &obj->dftC_[i];
    rec->quietLimit = 0x50;
    rec->toneLimit = 3000;
    rec->phase = 0;
    rec->accI = 0;
    rec->accQ = 0;
  }
  obj->dftC_[0].step = 0x600;
  obj->dftC_[1].step = 0x800;
  obj->dftC_[2].step = 0xa00;

  /* 2. scalar state */
  obj->rtDetState_ = 1;
  obj->rtDetHits_ = 0;
  obj->rtDetCnt_ = 0;
  obj->rtDetThrA_ = 3;
  obj->rtDetThrB_ = 9;
  obj->rx.rnegReason_ = 0;
  obj->dftActive_ = 0;
  obj->rx.rnegRateIdx_ = 0;
  obj->shortPhase2_ = 0;
  obj->tx.txFlags_ = 4; /* 16-bit store: overwrites the whole flag word */
  obj->echoAdaptCnt_ = 0;
  obj->w358a_ = 0;
  obj->w3588_ = 0;
  obj->hsSymCnt_ = 0;
  obj->tx.k56Idx_ = 0;
  obj->tx.blkLen_ = 6;
  obj->tx.w25da_ = 0;
  obj->tx.k56Word_ = 0x8990;
  obj->tx.symCnt_ = 0;

  /* 3. two 12-word buffers */
  for (i = 0; i < 12; i++)
  {
    obj->rx.precoef_[i] = 0;
    obj->tx.precCoef_[i] = 0;
  }

  /* 4. transmit scale, 2400 baud / 1800 Hz modulator and demodulator */
  VPcmV34SetTxScale(obj);
  V34SetupModulator(m, V34_BAUD_2400, V34_CARRIER_1800, 0, 0, 0);
  rxinit(obj);

  if (1 < dsplibs_debug_level)
    dsplibs_debug_printf("V34SetupDemodulator: baudrate %ld, carrier %ld\n", (long)V34_BAUD_2400, (long)V34_CARRIER_1800);
  /* inlined V34SetupDemodulator(baud 2400, carrier 1800) arm: the 1800 Hz sine table of 16 points */
  obj->rx.w38c_ = 4;
  obj->rx.w414_ = 16000;
  obj->rx.w422_ = 16000;
  obj->rx.w412_ = 16000;
  obj->rx.w410_ = 8000;
  obj->rx.hsineLen_ = 0x10;
  obj->rx.hsineTab_ = hsine1800;

  /* 5. role arm: calling side (1200 Hz modulator carrier) or answering side (2400 Hz); the arms differ in the flag words, the carrier and the filter table */
  if (obj->modeChar_ == V34ROLE_CALL)
  {
    obj->tx.txFlags_ = 4;
    obj->rx.rxFlags_ = 0;
    for (i = 0; i < 100; i++)
      U16(&obj->waae6_, 2 * i) = 0;
    V34SetupModulator(m, V34MOD_DPSK_600, V34_CARRIER_1200, 0, 0, 1);
    obj->fskBias_ = 0;
    obj->fskTapIdx_ = 0x30;
    obj->rx.p0508_ = (void*)bpv22high;
    obj->fskCounter_ = 0;
    obj->waae4_ = 0;
    obj->v21RxBuf_ = USHRT_MAX;
    obj->v21BitCnt_ = 0;
    obj->fskNextPoint_ = 6;
    obj->fskPatLow_ = 1;
    obj->fskPatHigh_ = 0;
    obj->fskPeriod_ = 0xc;
    obj->fskReload_ = 6;
    {
      uint16_t flags = obj->rx.rxFlags_;
      uint16_t v4c6 = AS_U16(obj->rx.rxGainEst_);
      obj->rx.w39e_ = 0x199a;
      obj->rx.rxFlags_ = (uint16_t)(flags | 0xa00u);
      obj->rx.rxGain_ = v4c6;
    }
    for (i = 0; i < 0x30; i++)
      AS_U16(((V34Obj*)obj)->rx.levelRing_[i]) = 0;
  }
  else
  {
    obj->tx.txFlags_ = 5;
    obj->rx.rxFlags_ = 4;
    for (i = 0; i < 100; i++)
      U16(&obj->waae6_, 2 * i) = 0;
    V34SetupModulator(m, V34MOD_DPSK_600, V34_CARRIER_2400, 0, 0, 1);
    obj->fskBias_ = 0;
    obj->fskTapIdx_ = 0x30;
    obj->rx.p0508_ = (void*)bpv22low;
    obj->fskCounter_ = 0;
    obj->waae4_ = 0;
    obj->v21RxBuf_ = USHRT_MAX;
    obj->v21BitCnt_ = 0;
    obj->fskNextPoint_ = 6;
    obj->fskPatLow_ = 1;
    obj->fskPatHigh_ = 0;
    obj->fskPeriod_ = 0xc;
    obj->fskReload_ = 6;
    {
      uint16_t flags = obj->rx.rxFlags_;
      uint16_t v4c6 = AS_U16(obj->rx.rxGainEst_);
      obj->rx.w39e_ = 0x199a;
      obj->rx.rxFlags_ = (uint16_t)(flags | 0xa00u);
      obj->rx.rxGain_ = v4c6;
    }
    for (i = 0; i < 0x30; i++)
      AS_U16(((V34Obj*)obj)->rx.levelRing_[i]) = 0;
  }

  /* 6. digital and transmit initialisation, tone detector; the role is tested again here */
  obj->rx.w0400_ = 0;
  obj->rx.w402_ = 0;

  preinitdigital(obj);
  txinit(obj);

  detectorinit(&obj->det, obj->modeChar_ == V34ROLE_CALL ? c2400_ : c1200_, 0, 200, 0x32, 0x600, 0);
  obj->rx.rxFlags_ = (uint16_t)(obj->rx.rxFlags_ | 0x200u);
}
