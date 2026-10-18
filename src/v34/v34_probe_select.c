/**
 * @file v34_probe_select.c
 * @brief V.34 phase-2 probing-result selector: probeselect() and chkForceBaudRate().
 *
 * After the line probing signal has been analysed, probeselect() chooses the symbol rate / carrier for the data phase and builds the INFO0 words that
 * announce the choice to the far end.  It is one straight-line function of seven stages, not a state machine:
 *  - S1: clear the ten INFO0 words (sub-object `info`).
 *  - S2: ratio of the two probe SNR measurements (`waac4_`, `waac8_`).
 *  - S3: dB count of that ratio and the requested power reduction from the receive gain.
 *  - S4: power-reduction request: encode it into INFO0 word 0 and scale the AGC gain estimate (`rxGainEst_`) accordingly.
 *  - S5: normalise the 25 probing bins (sub-object `probe`, `bin_[25]`, DftBin records) to their minimum and reduce the band edge entries when the spectrum is
 *    tilted.
 *  - S6: chkForceBaudRate() clamps the per-rate quality entries of the records to the maximum symbol rate index of the host parameters.
 *  - S7: the rate cascade 3429 > 3200 > 3000 > 2800 > 2400: for each rate the records decide whether it is usable.  The answering side (role other
 *    than V34ROLE_CALL) selects the first usable rate and returns after filling the rate block (sub-object `rate`: baud, carrier, scale and detector
 *    tables, pre-emphasis); the calling side records its pre-emphasis index in INFO0 and falls to the next lower rate.
 *
 * The pre-emphasis index is found by pe_search(), a loop that exists in ten copies of one body with different constants (one per rate and role), and is
 * bit-reversed into the INFO0 words by the three tails tail_a() / tail_b() / tail_c().  The body is a single function with labels named after the
 * role of each block (powerReductionRegular, bandEdgeNorm, stage3200, rate3200Return, rate3200Fall ...): a `...Return` label ends the function, a
 * `...Fall` label continues into the next lower rate.
 *
 * Callees: bitreverse() (v34_agc_demod.c), dsplibs_debug_printf().  Tables (v34_symbol_tables.c): scale2400 / 2800 / 3000 / 3200 / 3429 and
 * c1600 / c1680 / c1800_ / c1829 / c1867 / c1920 / c1959 / c2000.  The pointer slots used are `params` (the host parameter block, byte 0x50 holds the
 * maximum symbol rate index in bits 5..7), `ctx` (read by chkForceBaudRate()) and the table pointers stored in the rate block.
 *
 * Hazards:
 *  - H1: dead code: each pre-emphasis loop has a branch for a loop index of 5 ("index is 0"); the index starts at 5 and is incremented before the first
 *    test, so the branch cannot be reached.  It is kept in pe_search().
 *  - H2: chkForceBaudRate() reads and writes the rate flags `v34Flags_` of the engine context `ctx_` when `pcmMode_` is set (a local array otherwise);
 *    `ctx_` is not validated.
 *  - H3: every debug-level guard is the unsigned comparison `1 < level`.  Some guards read the global afresh, others use a value cached earlier
 *    (`lvl`, `lvlCached`); each cache is refreshed right after the only operation that can change the level (a print), so a cached read never differs
 *    observably from a fresh one.
 *  - H4: the minimum of stage S5 starts at 0x20, is compared sign-extended but subtracted zero-extended.
 *  - H5: no division here can trap (all divisors are guarded).
 */

#include <stddef.h>
#include <stdint.h>

#include "v34_symbol_tables.h" /* scale2400..3429, c1600..c2000 */
#include "v34_symbol_rates.h"  /* V34_BAUD_*, V34_CARRIER_* */
#include "v34_internal.h"

#include "core/dsp_debug_level.h" /* dsplibs_debug_level, LEVEL_ON() */
#include "core/dsp_host.h"

#include "v34_access.h"        /* S16 / U16: the probing records and the ten INFO0 words are walked by byte offset */
#include "v34_role.h"          /* V34ROLE_* (the values of modeChar_) */
#include "core/dp_params.h"    /* the parameter block (V34ModParObj) */
#include "v34_flomodem_view.h" /* PcmFlo47Obj: the engine context */
#include "v34_object_view.h"

/**
 * @brief Limit the per-rate quality entries of the probing records to the maximum symbol rate the host parameters allow.
 * @param obj   the V.34 object
 * @param R     the probing sub-object: 25 DFT bins (`bin_`), of which only the `normCnt` entries are written here
 * @note k = (byte)params[0x50] >> 5 is the "max V34 baud rate index" (0..7); it is printed when the debug level allows.  Six flag bytes f[0..5] select
 *       the rates: f[0..4] = 1; f[5] = 1 and the flags live in `ctx_->v34Flags_` when `pcmMode_ != 0`; else f[5] = 1 (`k56Mode_ == 0`) or 0
 *       with a local flag array.  For k != 0 every e[j], j in 1..5 with k <= j, is cleared; a cleared e[5] / e[4] / e[3] / e[2] forces the record
 *       entries of the corresponding rate (3429, 3200, 3000, 2800) to the value 0xb or 7,
 *       above the thresholds the rate cascade tests (> 10 for the first entry, > 6 for the others), so the rate is skipped.
 * @warning writes `ctx_->v34Flags_` when `pcmMode_` is set; `ctx_` is not validated (hazard H2).
 */
void chkForceBaudRate(V34Obj* obj, V34Probe* R)
{
  unsigned char f[6];
  PcmFlo47Obj* ctx;
  V34ModParObj* P;
  unsigned char* e;
  unsigned int k;
  int j;

  ctx = (PcmFlo47Obj*)obj->ctx_;
  for (j = 0; j < 6; j++)
    f[j] = 0;
  P = obj->params_;
  k = (unsigned int)(P->flags50 >> 5);
  if (LEVEL_ON())
    dsplibs_debug_printf("VpcmV34Main: max V34 baud rate index = %d\r\n", (int)k);
  f[0] = f[1] = f[2] = f[3] = f[4] = 1;
  if (obj->pcmMode_ != 0)
  {
    f[5] = 1;
    e = ctx->v34Flags_;
  }
  else if (obj->k56Mode_ == 0)
  {
    f[5] = 1;
    e = f;
  }
  else
  {
    f[5] = 0;
    e = f;
  }
  if (k != 0u)
  {
    if (k <= 1u)
      e[1] = 0;
    if (k <= 2u)
      e[2] = 0;
    if (k <= 3u)
      e[3] = 0;
    if (k <= 4u)
      e[4] = 0;
    if (k <= 5u)
      e[5] = 0;
  }
  if (e[5] == 0)
    R->bin_[24].normCnt = 0xb;
  if (e[4] == 0)
    R->bin_[22].normCnt = 7;
  if (e[3] == 0)
  {
    R->bin_[2].normCnt = 7;
    R->bin_[21].normCnt = 7;
  }
  if (e[2] == 0)
  {
    R->bin_[20].normCnt = 7;
    R->bin_[19].normCnt = 7;
  }
}

/**
 * @brief The pre-emphasis search shared by the ten rate arms (one loop body, ten different constants).
 * @param R     probing sub-object
 * @param rec   index of the bin whose level `energyHi` starts the search (22, 20, 19 or 18 for the rates 3429, 3200, 3000, 2800 / 2400)
 * @param mul   Q14 multiplier applied per step
 * @param baud  symbol rate, for the debug text only
 * @return the first index 6..10 at which the scaled value exceeds the threshold (the level `energyHi` of bin 4); 10 if none does.
 * @note starts with the level of bin `rec` and index 5; each step is v = (short)((v * mul) >> 14), index += 1; prints the result when the debug level
 *       is above 1 (every level guard is a fresh read).  The caller stores (u16)idx and derives the INFO0 bits.
 * @note the `peIdx == 5` branch (return 0) is unreachable (hazard H1).
 */
static int pe_search(const V34Probe* R, int rec, int mul, int baud)
{
  int thr = R->bin_[4].energyHi;
  int v = R->bin_[rec].energyHi;
  int peIdx = 5;
  do
  {
    v = (int16_t)((v * mul) >> 14);
    peIdx = (int16_t)(peIdx + 1);
    if (v > thr)
    {
      if (peIdx == 5)
      { /* H1: unreachable */
        if (LEVEL_ON())
          dsplibs_debug_printf("V34PREEMPHASIS, - index is 0, baudrate= %d\n", baud);
        return 0;
      }
      if (LEVEL_ON())
        dsplibs_debug_printf("V34PREEMPHASIS, - index is %d, baudrate= %d\n", peIdx, baud);
      return peIdx;
    }
  } while (peIdx <= 9);
  if (LEVEL_ON())
    dsplibs_debug_printf("V34PREEMPHASIS, - index is 10, baudrate= %d \n", baud);
  return peIdx;
}

/* The three shared tails: r = bitreverse((u16)idx, 4) zero-extended, merged into INFO0 words 1 and 2. */
static void tail_a(V34Info* infoobj, int idx)
{ /* word 1 |= r >> 2, word 2 |= (r << 6) & 0xff (rates 3429 .. 2800, answering side) */
  unsigned int r = bitreverse((unsigned short)idx, 4);
  infoobj->wa9ae_ = (uint16_t)((r >> 2) | infoobj->wa9ae_);
  infoobj->wa9b0_ = (uint16_t)(((r << 6) & 0xffu) | infoobj->wa9b0_);
}
static void tail_b(V34Info* infoobj, int idx)
{ /* word 1 |= r >> 2, word 2 |= (r << 6) & 0xc0 (2400, answering side) */
  unsigned int r = bitreverse((unsigned short)idx, 4);
  infoobj->wa9ae_ = (uint16_t)((r >> 2) | infoobj->wa9ae_);
  infoobj->wa9b0_ = (uint16_t)(((r << 6) & 0xc0u) | infoobj->wa9b0_);
}
static void tail_c(V34Info* infoobj, int idx)
{ /* word 1 |= (r >> 2) & 3, word 2 |= (r << 6) & 0xc0 (2400, calling side) */
  unsigned int r = bitreverse((unsigned short)idx, 4);
  infoobj->wa9ae_ = (uint16_t)(((r >> 2) & 3u) | infoobj->wa9ae_);
  infoobj->wa9b0_ = (uint16_t)(((r << 6) & 0xc0u) | infoobj->wa9b0_);
}

/**
 * @brief Scale the AGC gain estimate down for a power reduction request.
 * @param b  the receive sub-object
 * @param n  number of steps (a power reduction of n)
 * @return the new gain estimate
 * @note n do-while iterations (at least one) of v = ((short)v * 0x47cf + 0x2000) >> 14 starting from `rxGainEst_`.
 */
static int agc_scale(V34Rx* rxobj, int n)
{
  int32_t ec = AS_U16(rxobj->rxGainEst_), v = 0;
  int cnt = 0;
  do
  {
    v = ((int32_t)(int16_t)ec * 0x47cf + 0x2000) >> 14;
    cnt = (uint16_t)(cnt + 1);
    ec = v;
  } while (cnt < n);
  return (int16_t)v;
}

/**
 * @brief Choose the data-phase symbol rate and carrier from the probing results and build the INFO0 words announcing it.
 * @param v34obj the V.34 object
 * @note see the file header for the stages.  Writes: the ten INFO0 words (`info` sub-object), the rate block (`rate`: baud, carrier, scale / detector
 *       tables, pre-emphasis), `rxGainEst_` (power reduction) and the probing records (band-edge normalisation).  Returns without a rate choice
 *       (rate block unchanged) when the answering side finds no usable rate at 2400 either.
 * @note called from the V.34 handshake once phase-2 probing has finished.
 * @warning relies on the host parameter block, the `ctx` pointer and the sub-objects being valid; none of them is checked.
 */
void probeselect(V34Obj* v34obj)
{
  V34Info* infoobj = &v34obj->info;
  V34Rate* rateobj = &v34obj->rate;
  V34Rx* rxobj = &v34obj->rx;
  V34Probe* R = &v34obj->probe; /* the 25 DFT bins of the probe block */
  uint32_t l1, ratio;
  unsigned int lvl;       /* the debug level as most recently read */
  unsigned int lvlCached; /* the debug level cached after S2, refreshed after prints */
  int dBcnt, powerReduction, i;
  unsigned int mode, rec22;
  uint16_t gainWord;
  unsigned int rev;
  int preempIdx;

  /* ---- S1: clear the ten INFO0 words ---- */
  for (i = 0; i < 10; i++)
    U16(infoobj, 2 * i) = 0;

  /* ---- S2: ratio of the two probe SNR measurements, l1 = waac4_ and v = waac8_ (fixed point, 10 fractional bits) ---- */
  l1 = v34obj->waac4_;
  ratio = 0;
  if (l1 != 0)
  {
    uint32_t v = v34obj->waac8_, mask = 0x80000000u;
    int cnt = 0;
    if ((int32_t)v >= 0)
    {
      do
      {
        cnt = (int16_t)(cnt + 1);
        mask >>= 1;
      } while ((mask & v) == 0 && cnt <= 9);
      if (cnt > 9)
      {
        ratio = ((v << 10) + (l1 >> 1)) / l1; /* cannot trap: l1 != 0 */
        goto S2_done;
      }
    }
    { /* v has its top bits set: normalise by cnt before dividing */
      uint32_t sh = v << (cnt & 31);
      uint32_t a = l1 >> ((0xb - cnt) & 31);
      uint32_t d = l1 >> ((0xa - cnt) & 31);
      ratio = 0;
      if (d != 0)
        ratio = (sh + a) / d;
    }
  }
S2_done:
  lvlCached = dsplibs_debug_level;
  if (lvlCached > 1u)
  {
    dsplibs_debug_printf("V34PROBE, snr_L1=%d , snr_L2=%d , L2toL1ratio=%d  (all not in dB)\n", l1, v34obj->waac8_, ratio);
    lvlCached = dsplibs_debug_level;
  }

  /* ---- S3: the ratio in steps of about 0.4 dB (dBcnt) and the requested power reduction ---- */
  if (ratio > 0x18fffu)
  {
    dBcnt = 1000;
  }
  else
  {
    uint32_t e = 0x47d;
    dBcnt = 0;
    while (e < ratio)
    {
      e = (e * 0x509u + 0x200u) >> 10;
      dBcnt = (int16_t)(dBcnt + 1);
    }
  }
  powerReduction = 0;
  gainWord = AS_U16(rxobj->rxGain_);
  if ((int16_t)gainWord != 0)
  {
    int32_t g = (int16_t)gainWord;
    int32_t num = (g >> 1) + 0x640000;
    uint32_t u = (uint32_t)(num / g); /* guarded by the zero test above */
    if (u > 0x4000u)
      u = 0x4000u; /* unsigned clamp */
    powerReduction = (int16_t)((u * (uint32_t)dBcnt) >> 14);
  }
  if (lvlCached > 1u)
  { /* uses the cached level */
    dsplibs_debug_printf("V34PROBE, dBcnt=%d , powerReductionReq=%d , gain=%d\n", dBcnt, powerReduction, (int)(int16_t)gainWord);
    lvlCached = dsplibs_debug_level;
  }

  /* ---- S4: power reduction request (sensitive-receive mechanism enabled by params bit 4) ---- */
  {
    V34ModParObj* P = v34obj->params_;
    if ((P->flags50 & 0x10) == 0)
    {
      if (lvlCached > 1u)
        dsplibs_debug_printf("Sensitive RX Power Reduction mechanism disabled!\r\n");
      goto powerReductionRegular;
    }
    if (lvlCached > 1u)
      dsplibs_debug_printf("Sensitive RX Power Reduction mechanism enabled!\r\n");
    {
      uint32_t l1v = v34obj->waac4_;
      if (l1v <= 0x1f3u && (v34obj->info0d.wa97e_ & 0x80))
        goto powerReductionBranchA;
      if (l1v <= 0x3e7u && (v34obj->info0d.wa97e_ & 0x80))
        goto powerReductionBranchB;
    }
  }
powerReductionRegular: /* regular path: request the computed reduction, at most 7 */
  if (!(v34obj->info0d.wa97e_ & 0x80))
    goto noPowerReduction;
  if (rxobj->rxGain_ > 0xfff || (int16_t)powerReduction == 0)
    goto noPowerReduction;
  if ((int16_t)powerReduction > 7)
    powerReduction = 7;
  rev = bitreverse((unsigned short)powerReduction, 3);
  lvl = dsplibs_debug_level;
  infoobj->wa9ac_ = (uint16_t)((rev << 5) | infoobj->wa9ac_);
  if (lvl > 1u)
  {
    dsplibs_debug_printf("V34PROBE, asking for a power reduction of %d\n", powerReduction);
    lvl = dsplibs_debug_level;
    if (lvl > 1u)
    {
      dsplibs_debug_printf("V34PROBE, agc gainestimate of L1 signal is %d\n", (int)rxobj->rxGainEst_);
      lvl = dsplibs_debug_level;
    }
  }
  if (powerReduction > 0)
    AS_U16(rxobj->rxGainEst_) = (int16_t)agc_scale(rxobj, powerReduction); /* the step counter is 16 bits, powerReduction <= 7 */
  if (rxobj->rxGainEst_ > 0x1b58)
    AS_U16(rxobj->rxGainEst_) = 0x1b58;
  if (lvl > 1u)
  {
    dsplibs_debug_printf("V34PROBE, agc gainestimate due to power reduction request is %d\n", (int)rxobj->rxGainEst_);
    lvl = dsplibs_debug_level;
  }
  goto bandEdgeNorm;

powerReductionBranchB: /* branch B: 500 <= snr_L1 <= 999 and rxinfo0 bit 0x80 */
  rev = bitreverse(7, 3);
  infoobj->wa9ac_ = (uint16_t)((rev << 5) | infoobj->wa9ac_);
  {
    int v = agc_scale(rxobj, 7);
    lvl = dsplibs_debug_level;
    AS_U16(rxobj->rxGainEst_) = (int16_t)v;
    if (lvl > 1u)
      dsplibs_debug_printf("V34PROBE, asking for a power reduction of %d\n", 7);
  }
  goto noPowerReduction;

powerReductionBranchA: /* branch A: snr_L1 < 500 and rxinfo0 bit 0x80 */
  rev = bitreverse(7, 3);
  infoobj->wa9ac_ = (uint16_t)((rev << 5) | infoobj->wa9ac_);
  rev = bitreverse(2, 3);
  infoobj->wa9ac_ = (uint16_t)((rev << 2) | infoobj->wa9ac_);
  {
    int v = agc_scale(rxobj, 9);
    lvl = dsplibs_debug_level;
    AS_U16(rxobj->rxGainEst_) = (int16_t)v;
    if (lvl > 1u)
      dsplibs_debug_printf("V34PROBE, asking for a power reduction of %d\n", 9);
  }
  /* fall into noPowerReduction */

noPowerReduction:
  lvl = dsplibs_debug_level;
  if (lvl > 1u)
  {
    dsplibs_debug_printf("V34PROBE, not asking for power reduction\n");
    lvl = dsplibs_debug_level;
    if (lvl > 1u)
    {
      dsplibs_debug_printf("V34PROBE, rx->gain=%d ,(obj->rxinfo0.data[1]&0x80)=%d\n", (int)rxobj->rxGain_, (int)(v34obj->info0d.wa97e_ & 0x80));
      lvl = dsplibs_debug_level;
    }
  }

bandEdgeNorm: /* ---- S5: band-edge normalisation of the 25 records ---- */
  {
    int mn0 = 0x20;
    for (i = 0; i < 25; i++)
    {
      uint16_t c = AS_U16(R->bin_[i].normCnt);
      if ((int)(int16_t)c < mn0)
        mn0 = c; /* stored zero-extended (H4) */
    }
    for (i = 0; i < 25; i++)
      AS_U16(R->bin_[i].normCnt) = (uint16_t)(AS_U16(R->bin_[i].normCnt) - mn0);
  }
  if (lvl > 1u)
  { /* uses the level as last read */
#define REC(k) ((int)R->bin_[(k)].normCnt)
    dsplibs_debug_printf("V34PROBE,0=%d,%d,%d,%d,%d,5=%d,%d,%d,%d,%d,10=%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,20=%d,%d,22=%d,%d,24=%d\n", REC(0), REC(1), REC(2),
                         REC(3), REC(4), REC(5), REC(6), REC(7), REC(8), REC(9), REC(10), REC(11), REC(12), REC(13), REC(14), REC(15), REC(16), REC(17),
                         REC(18), REC(19), REC(20), REC(21), REC(22), REC(23), REC(24));
#undef REC
  }
  {
    unsigned int isum = (uint16_t)(AS_U16(R->bin_[2].normCnt) + AS_U16(R->bin_[3].normCnt) + AS_U16(R->bin_[4].normCnt) + AS_U16(R->bin_[6].normCnt));
    unsigned int mn = (uint16_t)(AS_U16(R->bin_[12].normCnt) + AS_U16(R->bin_[13].normCnt) + AS_U16(R->bin_[14].normCnt) + AS_U16(R->bin_[16].normCnt));
    if (isum == 0)
      isum = 1;
    if ((int)mn > (int)(isum * 4u))
    {
      if (LEVEL_ON())
        dsplibs_debug_printf("V34PROBE, min = %d, i= %d, so reducing band edge norm by 2\n", mn, isum);
      for (i = 16; i < 24; i++)
      {
        uint16_t v = AS_U16(R->bin_[i].normCnt);
        if ((int16_t)v > 2)
          AS_U16(R->bin_[i].normCnt) = (uint16_t)(v - 2);
      }
    }
    else if ((int)mn > (int)(isum * 2u))
    {
      if (LEVEL_ON())
        dsplibs_debug_printf("V34PROBE, min = %d, i= %d, so reducing band edge norm by 1\n", mn, isum);
      for (i = 16; i < 24; i++)
      {
        uint16_t v = AS_U16(R->bin_[i].normCnt);
        if ((int16_t)v > 1)
          AS_U16(R->bin_[i].normCnt) = (uint16_t)(v - 1);
      }
    }
  }

  /* ---- S6: clamp the records to the allowed maximum symbol rate ---- */
  chkForceBaudRate(v34obj, R);
  if (v34obj->w359a_ != 0)
    goto forcedBaudMode;

  /* ---- S7: the rate cascade; a rate is skipped when its record entries exceed per-rate thresholds (5 .. 10) ---- */
  if (R->bin_[24].normCnt > 10)
    goto skip3429Rec1068;
  if (R->bin_[23].normCnt > 6)
    goto skip3429Rec1024;
  if (R->bin_[1].normCnt > 5)
    goto skip3429Rec56;
  mode = v34obj->modeChar_;
  if (mode == V34ROLE_CALL)
    goto rate3429Fall;
  if ((v34obj->info1c.wa9ea_ & 1) || (v34obj->info1c.ba9ec_ & 0xe0))
    goto rate3429Return;
  goto stage3200;

skip3429Rec1068:
skip3429Rec1024:
skip3429Rec56:
  mode = v34obj->modeChar_;
  goto stage3200;

rate3429Return: /* 3429, answering side: fill the rate block and return */
  rateobj->baud_ = V34_BAUD_3429;
  rateobj->carrier_ = V34_CARRIER_1959;
  rateobj->scalePtr_ = (void*)scale3429;
  rev = bitreverse((unsigned short)((v34obj->info1c.wa9ea_ >> 1) & 0xf), 4);
  rateobj->preemp_ = (int16_t)rev;
  rateobj->scaleTab_ = (void*)scale3429;
  rateobj->detTab_ = (void*)c1959;
  infoobj->wa9b0_ = (uint16_t)(infoobj->wa9b0_ | 0x2);
  infoobj->wa9b2_ = (uint16_t)(infoobj->wa9b2_ | 0xd0);
  rateobj->rxBaud_ = V34_BAUD_3429;
  rateobj->rxCarrier_ = V34_CARRIER_1959;
  infoobj->wa9b0_ = (uint16_t)(infoobj->wa9b0_ | 0x24);
  preempIdx = pe_search(R, 22, 0x6626, V34_BAUD_3429);
  rateobj->waaa4_ = (int16_t)preempIdx;
  tail_a(infoobj, preempIdx);
  return;

rate3429Fall: /* 3429, calling side: record the pre-emphasis index in INFO0, fall to the 3200 stage */
  infoobj->wa9bc_ = (uint16_t)(infoobj->wa9bc_ | 0xe0);
  preempIdx = pe_search(R, 22, 0x6626, V34_BAUD_3429);
  rateobj->waaa4_ = (int16_t)preempIdx;
  rev = bitreverse((unsigned short)preempIdx, 4);
  infoobj->wa9ba_ = (uint16_t)((rev * 2u) | infoobj->wa9ba_);
  mode = v34obj->modeChar_;
  /* fall into stage3200 */

stage3200:
  rec22 = AS_U16(R->bin_[22].normCnt);
  if ((int16_t)rec22 > 6)
    goto stage3000TestB;
  if (R->bin_[1].normCnt > 6)
    goto stage3000Test;
  if (mode == V34ROLE_CALL)
    goto rate3200Fall;
  if ((v34obj->info1c.wa9e8_ & 3) || (v34obj->info1c.wa9ea_ & 0xc0))
    goto rate3200Return;
stage3000Test:
  if ((int16_t)rec22 > 6)
    goto stage3000TestB;
  if (R->bin_[2].normCnt <= 6)
    goto stage3000;
stage3000TestB:
  if (R->bin_[21].normCnt > 6)
    goto stage2800;
  if (R->bin_[1].normCnt > 6)
    goto stage2800;
stage3000: /* 3000 */
  if (mode == V34ROLE_CALL)
    goto rate3000Fall;
  if ((v34obj->info1c.wa9e6_ & 7) || (v34obj->info1c.wa9e8_ & 0x80))
    goto rate3000Return;
  goto stage2800;

rate3200Return: /* 3200, answering side: fill the rate block and return */
  rateobj->baud_ = V34_BAUD_3200;
  rateobj->scalePtr_ = (void*)scale3200;
  rateobj->carrier_ = (v34obj->info1c.wa9e8_ & 0x40) ? V34_CARRIER_1920 : V34_CARRIER_1829;
  rev = bitreverse((unsigned short)((v34obj->info1c.wa9e8_ >> 2) & 0xf), 4);
  rateobj->preemp_ = (int16_t)rev;
  rateobj->scaleTab_ = (void*)scale3200;
  infoobj->wa9b2_ = (uint16_t)(infoobj->wa9b2_ | 0x90);
  rateobj->rxBaud_ = V34_BAUD_3200;
  if (R->bin_[23].normCnt > 5)
  {
    rateobj->rxCarrier_ = V34_CARRIER_1829;
    rateobj->detTab_ = (void*)c1829;
  }
  else
  {
    infoobj->wa9ae_ = (uint16_t)(infoobj->wa9ae_ | 0x4);
    rateobj->detTab_ = (void*)c1920;
    rateobj->rxCarrier_ = V34_CARRIER_1920;
  }
  infoobj->wa9b0_ = (uint16_t)(infoobj->wa9b0_ | 0x24);
  preempIdx = pe_search(R, 20, 0x639f, V34_BAUD_3200);
  rateobj->waaa2_ = (int16_t)preempIdx;
  tail_a(infoobj, preempIdx);
  return;

rate3200Fall: /* 3200, calling side: record the pre-emphasis index in INFO0, fall to the 3000 test */
  {
    uint16_t r23 = AS_U16(R->bin_[23].normCnt);
    if ((int16_t)r23 <= 5)
    {
      infoobj->wa9b8_ = (uint16_t)(infoobj->wa9b8_ | 0x40);
      r23 = AS_U16(R->bin_[23].normCnt);
    }
    if ((int16_t)r23 <= 7)
    {
      infoobj->wa9b8_ = (uint16_t)(infoobj->wa9b8_ | 0x2);
      infoobj->wa9ba_ = (uint16_t)(infoobj->wa9ba_ | 0xc0);
    }
    else
    {
      infoobj->wa9b8_ = (uint16_t)(infoobj->wa9b8_ | 0x3);
      infoobj->wa9ba_ = (uint16_t)(infoobj->wa9ba_ | 0x40);
    }
  }
  preempIdx = pe_search(R, 20, 0x639f, V34_BAUD_3200);
  rateobj->waaa2_ = (int16_t)preempIdx;
  rev = bitreverse((unsigned short)preempIdx, 4);
  infoobj->wa9b8_ = (uint16_t)((rev << 2) | infoobj->wa9b8_);
  rec22 = AS_U16(R->bin_[22].normCnt);
  mode = v34obj->modeChar_;
  goto stage3000Test;

rate3000Return: /* 3000, answering side: fill the rate block and return */
  rateobj->baud_ = V34_BAUD_3000;
  rateobj->scalePtr_ = (void*)scale3000;
  rateobj->carrier_ = (v34obj->info1c.wa9e6_ & 0x80) ? V34_CARRIER_2000 : V34_CARRIER_1800;
  rev = bitreverse((unsigned short)((v34obj->info1c.wa9e6_ >> 3) & 0xf), 4);
  rateobj->preemp_ = (int16_t)rev;
  rateobj->scaleTab_ = (void*)scale3000;
  infoobj->wa9b0_ = (uint16_t)(infoobj->wa9b0_ | 0x3);
  infoobj->wa9b2_ = (uint16_t)(infoobj->wa9b2_ | 0x60);
  rateobj->rxBaud_ = V34_BAUD_3000;
  if (R->bin_[22].normCnt > 5 || R->bin_[2].normCnt > 5)
  {
    rateobj->rxCarrier_ = V34_CARRIER_1800;
    rateobj->detTab_ = (void*)c1800_;
  }
  else
  {
    rateobj->detTab_ = (void*)c2000;
    infoobj->wa9ae_ = (uint16_t)(infoobj->wa9ae_ | 0x4);
    rateobj->rxCarrier_ = V34_CARRIER_2000;
  }
  infoobj->wa9b0_ = (uint16_t)(infoobj->wa9b0_ | 0x24);
  preempIdx = pe_search(R, 19, 0x656f, V34_BAUD_3000);
  rateobj->waaa0_ = (int16_t)preempIdx;
  tail_a(infoobj, preempIdx);
  return;

rate3000Fall: /* 3000, calling side: record the pre-emphasis index in INFO0, fall to the 2800 stage */
  if ((int16_t)rec22 <= 5)
  {
    if (R->bin_[2].normCnt <= 5)
    {
      infoobj->wa9b6_ = (uint16_t)(infoobj->wa9b6_ | 0x80);
      rec22 = AS_U16(R->bin_[22].normCnt);
    }
  }
  if ((int16_t)rec22 <= 7)
  {
    if ((int16_t)rec22 <= 6)
    {
      infoobj->wa9b6_ = (uint16_t)(infoobj->wa9b6_ | 0x1);
      infoobj->wa9b8_ = (uint16_t)(infoobj->wa9b8_ | 0x80);
    }
    else
    {
      infoobj->wa9b6_ = (uint16_t)(infoobj->wa9b6_ | 0x2);
      infoobj->wa9b8_ = (uint16_t)(infoobj->wa9b8_ | 0x80);
    }
  }
  else
  {
    infoobj->wa9b6_ = (uint16_t)(infoobj->wa9b6_ | 0x4);
    infoobj->wa9b8_ = (uint16_t)(infoobj->wa9b8_ | 0x80);
  }
  preempIdx = pe_search(R, 19, 0x656f, V34_BAUD_3000);
  rateobj->waaa0_ = (int16_t)preempIdx;
  rev = bitreverse((unsigned short)preempIdx, 4);
  infoobj->wa9b6_ = (uint16_t)((rev << 3) | infoobj->wa9b6_);
  mode = v34obj->modeChar_;
  /* fall into stage2800 */

stage2800: /* 2800 */
  {
    unsigned int rec20 = AS_U16(R->bin_[20].normCnt);
    if ((int16_t)rec20 > 6)
      goto stage2800TestB;
    if (R->bin_[2].normCnt <= 6)
      goto stage2800Select;
  stage2800TestB:
    if (R->bin_[19].normCnt > 6)
      goto stage2400;
    if (R->bin_[1].normCnt > 6)
      goto stage2400;
  stage2800Select:
    if (mode == V34ROLE_CALL)
    {
      /* 2800, calling side: record the pre-emphasis index in INFO0, fall to the 2400 stage */
      if ((int16_t)rec20 <= 5)
      {
        if (R->bin_[2].normCnt <= 5)
          infoobj->wa9b2_ = (uint16_t)(infoobj->wa9b2_ | 0x1);
      }
      infoobj->wa9b4_ = (uint16_t)(infoobj->wa9b4_ | 0xd);
      preempIdx = pe_search(R, 18, 0x6789, V34_BAUD_2800);
      rateobj->waa9c_ = (int16_t)preempIdx;
      rev = bitreverse((unsigned short)preempIdx, 4);
      infoobj->wa9b4_ = (uint16_t)(((rev & 0xfu) << 4) | infoobj->wa9b4_);
      mode = v34obj->modeChar_;
      goto stage2400;
    }
    if (!(v34obj->info1c.wa9e4_ & 0xf))
      goto stage2400;
  }
  /* 2800, answering side: fill the rate block and return */
  rateobj->baud_ = V34_BAUD_2800;
  rateobj->scalePtr_ = (void*)scale2800;
  rateobj->carrier_ = (v34obj->info1c.wa9e2_ & 0x1) ? V34_CARRIER_1867 : V34_CARRIER_1680;
  rev = bitreverse((unsigned short)((v34obj->info1c.wa9e4_ >> 4) & 0xf), 4);
  rateobj->preemp_ = (int16_t)rev;
  rateobj->scaleTab_ = (void*)scale2800;
  infoobj->wa9b0_ = (uint16_t)(infoobj->wa9b0_ | 0x1);
  infoobj->wa9b2_ = (uint16_t)(infoobj->wa9b2_ | 0x20);
  rateobj->rxBaud_ = V34_BAUD_2800;
  if (R->bin_[20].normCnt > 5 || R->bin_[2].normCnt > 5)
  {
    rateobj->rxCarrier_ = V34_CARRIER_1680;
    rateobj->detTab_ = (void*)c1680;
  }
  else
  {
    infoobj->wa9ae_ = (uint16_t)(infoobj->wa9ae_ | 0x4);
    rateobj->detTab_ = (void*)c1867;
    rateobj->rxCarrier_ = V34_CARRIER_1867;
  }
  infoobj->wa9b0_ = (uint16_t)(infoobj->wa9b0_ | 0x24);
  preempIdx = pe_search(R, 18, 0x6789, V34_BAUD_2800);
  rateobj->waa9c_ = (int16_t)preempIdx;
  tail_a(infoobj, preempIdx);
  return;

forcedBaudMode:
  mode = v34obj->modeChar_;
stage2400: /* 2400 stage (the last arm): both roles return */
  if (mode == V34ROLE_CALL)
  {
    infoobj->wa9b0_ = (uint16_t)(infoobj->wa9b0_ | 0x24);
    preempIdx = pe_search(R, 18, 0x7da7, V34_BAUD_2400);
    rateobj->waa9a_ = (int16_t)preempIdx;
    tail_c(infoobj, preempIdx);
    return;
  }
  if (!(v34obj->info1c.wa9e0_ & 0x3c))
    return;
  rateobj->baud_ = V34_BAUD_2400;
  rateobj->scalePtr_ = (void*)scale2400;
  rateobj->carrier_ = (v34obj->info1c.wa9de_ & 0x4) ? V34_CARRIER_1800 : V34_CARRIER_1600;
  rev = bitreverse((unsigned short)(((v34obj->info1c.wa9de_ & 3u) << 2) | ((v34obj->info1c.wa9e0_ & 0xc0u) >> 6)), 4);
  rateobj->preemp_ = (int16_t)rev;
  rateobj->rxBaud_ = V34_BAUD_2400;
  rateobj->rxCarrier_ = V34_CARRIER_1600;
  rateobj->scaleTab_ = (void*)scale2400;
  rateobj->detTab_ = (void*)c1600;
  infoobj->wa9b0_ = (uint16_t)(infoobj->wa9b0_ | 0x24);
  preempIdx = pe_search(R, 18, 0x7da7, 0x960);
  rateobj->waa9a_ = (int16_t)preempIdx;
  tail_b(infoobj, preempIdx);
}
