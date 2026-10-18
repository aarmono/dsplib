/**
 * @file v32_datapath.c
 * @brief V.32 / V.32bis data path: the Viterbi trellis decoder and the mid-level helpers the state machines call.
 *
 * Every helper takes the connection wrapper (V32Wrap *): its `engPtr` slot holds the engine (V32Eng: the transmit chain with scrambler,
 * trellis encoder and pulse-shaping filter, the receive chain with matched filter, echo canceller, AGC, symbol-rate recovery and equaliser) and its
 * `hdxPtr` slot the half-duplex handshake object (V32Hdx).  The helpers are thin: they select a mode (tx / rx constellation, equaliser and echo
 * canceller adaptation, loops), run one block through a chain (ModDataV32(), DemodDataV32(), TxNoCarrierV32(), RxClampV32()), or test and clear an event flag
 * (RetrainDetectV32(), RenegotiateDetectV32(), EpochDetectV32()).
 * - VTBv32_init() / VTB_decoder(): the 8-state Viterbi decoder of the trellis-coded rates (7200, 9600 TCM, 12000, 14400 bit/s).  VTBv32_init() picks the
 *   constellation and branch tables for a rate class; VTB_decoder() is called once per received symbol by the FSE_decision_16Tpt / _32pt / _64pt / _128pt
 *   callbacks of v32_equalizer.c.
 * - SetTxModeV32() / SetRxModeV32(): program the scrambler masks, the transmit constellation or the receive decision callback for a bit rate.
 * - ScrambleDataV32() / DescrambleDataV32(): run the scrambler / descrambler of the engine in place.
 */

#include <stdint.h>
#include <stdio.h>
#include "core/dsp_table_handles.h"
#include <stdlib.h>

#include "v32_viterbi_tables.h"
#include "v32_tables.h" /* V32_SYMBOL_LEN: symbol length table of the timing loop, indexed by V32Wrap::w018 */
#include "fpm/fpm.h"
#include "v32.h"
#include "core/dsp_constants.h" /* RATE_* */
#include "core/dsp_host.h"
#include "v32_internal.h"

/** A symbol encoder of the transmit chain (the entries of the engine's encoder table, selected by `smcEncSel`). */
typedef void (*smcenc_t)(V32Smc*, V32Circ*, void*, unsigned short);

/**
 * @brief Select the branch tables and reset the state of the Viterbi decoder for a rate class.
 * @param vtb        the decoder object (V32Vtb, the history object plus 0x18); its table slots are written through the table-handle accessors
 * @param rate       rate class: 2 = 9600 bit/s (32 points), 3 = 7200 (16 points), 4 = 12000 (64 points), 5 or more = 14400 (128 points); 0 and 1 also select
 * the
 *                   128-point tables.  The families are not ordered by rate: 3 selects the 7200 family, 2 the 9600 family
 * @param fresh      0 to use the history buffer already attached to the object; non-zero to allocate a new 0x200-byte buffer first
 *
 * The 0x200-byte circular history buffer (16 slots of 8 states, 4 bytes per state) is cleared on every call, also on a mid-session rate change.  The decoder
 * state is reset: write index 0, previous quadrant 0, traceback depth 16, the eight path metrics 0.  SetRxModeV32() always passes `fresh == 0`, so the
 * allocation branch is not used today.
 */
void VTBv32_init(V32Vtb* vtb, unsigned int rate, int fresh)
{
  short rateS = (short)rate;
  char* scratch;
  int i;

  if (fresh == 0)
  {
    scratch = V32X_PG(char*, vtb, V32_VTB_OFF, (int)offsetof(V32Vtb, histBuf));
  }
  else
  {
    scratch = (char*)(sysdep_malloc(0x200));
    V32X_PS(char*, vtb, V32_VTB_OFF, (int)offsetof(V32Vtb, histBuf), scratch);
  }

  /* clear the history buffer: 16 slots of 8 states, 4 bytes (symbol, predecessor) per state */
  for (i = 0; i < 0x80; i++)
  {
    *(short*)(scratch + 2 + i * 4) = 0;
    *(short*)(scratch + i * 4) = 0;
  }
  vtb->writeIdx = 0;
  vtb->prevQuadrant = 0;
  vtb->tracebackDepth = 0x10; /* one lap of the history buffer */

  if (rateS == 3)
  {
    /* family 1: 16-point trellis-coded constellation, 7200 bit/s */
    vtb->family = 1;
    CFGP_SLOT_SET(const void*, (intptr_t)(&vtb->iMap), VTBv32_IMAP16T);
    CFGP_SLOT_SET(const void*, (intptr_t)(&vtb->qMap), VTBv32_QMAP16T);
    CFGP_SLOT_SET(const void*, (intptr_t)(&vtb->boundTab), VTB_BOUND_7200);
    CFGP_SLOT_SET(const void*, (intptr_t)(&vtb->regionTab), VTB_REGION_7200);
    vtb->rowSize = 2;
    vtb->lowMask = 7;
  }
  else if (rateS < 4)
  {
    if (rateS == 2)
    {
      /* family 2: 32-point constellation, 9600 bit/s */
      vtb->family = 2;
      CFGP_SLOT_SET(const void*, (intptr_t)(&vtb->iMap), VTBv32_IMAP32);
      CFGP_SLOT_SET(const void*, (intptr_t)(&vtb->qMap), VTBv32_QMAP32);
      CFGP_SLOT_SET(const void*, (intptr_t)(&vtb->boundTab), VTB_BOUND_9600);
      CFGP_SLOT_SET(const void*, (intptr_t)(&vtb->regionTab), VTB_REGION_9600);
      vtb->rowSize = 4;
      vtb->lowMask = 0xf;
    }
    else
    {
      /* rates 0 and 1 (SetRxModeV32() does not pass them): the 128-point tables, as for the highest rate below */
      vtb->family = 4;
      CFGP_SLOT_SET(const void*, (intptr_t)(&vtb->iMap), VTBv32_IMAP128);
      CFGP_SLOT_SET(const void*, (intptr_t)(&vtb->qMap), VTBv32_QMAP128);
      CFGP_SLOT_SET(const void*, (intptr_t)(&vtb->boundTab), VTB_BOUND_14400);
      CFGP_SLOT_SET(const void*, (intptr_t)(&vtb->regionTab), VTB_REGION_14400);
      vtb->rowSize = 8;
      vtb->lowMask = 0x3f;
    }
  }
  else if (rateS == 4)
  {
    /* family 3: 64-point constellation, 12000 bit/s */
    vtb->family = 3;
    CFGP_SLOT_SET(const void*, (intptr_t)(&vtb->iMap), VTBv32_IMAP64);
    CFGP_SLOT_SET(const void*, (intptr_t)(&vtb->qMap), VTBv32_QMAP64);
    CFGP_SLOT_SET(const void*, (intptr_t)(&vtb->boundTab), VTB_BOUND_12000);
    CFGP_SLOT_SET(const void*, (intptr_t)(&vtb->regionTab), VTB_REGION_12000);
    vtb->rowSize = 6;
    vtb->lowMask = 0x1f;
  }
  else
  {
    /* family 4: 128-point constellation, 14400 bit/s (rate 5 and any larger value) */
    vtb->family = 4;
    CFGP_SLOT_SET(const void*, (intptr_t)(&vtb->iMap), VTBv32_IMAP128);
    CFGP_SLOT_SET(const void*, (intptr_t)(&vtb->qMap), VTBv32_QMAP128);
    CFGP_SLOT_SET(const void*, (intptr_t)(&vtb->boundTab), VTB_BOUND_14400);
    CFGP_SLOT_SET(const void*, (intptr_t)(&vtb->regionTab), VTB_REGION_14400);
    vtb->rowSize = 8;
    vtb->lowMask = 0x3f;
  }

  vtb->pathMetric[0] = 0;
  vtb->diffShift = vtb->family;
  for (i = 1; i < 8; i++)
  {
    vtb->pathMetric[i] = 0;
  }
}

/**
 * @brief Decode one received symbol with the 8-state Viterbi decoder and return its data bits.
 * @param vtb  decoder object (V32Vtb) initialised by VTBv32_init(); the FSE_decision_*T* callbacks pass the history object plus 0x18
 * @param rxI        equalised in-phase component of the received point
 * @param rxQ        equalised quadrature component of the received point
 * @param outBits    receives the decoded bits: the differentially decoded quadrant bits above `diffShift`, the uncoded bits below
 *
 * One call does, in order: (1) advance the circular write index and rescale the previous path metrics by 0x799a / 2^15 (a leak); (2) for the families 1 and 3 a
 * coarse derotation of the point, used only to pick the region; (3) quantise the magnitudes of the (derotated) point to a column and row of the region table
 * (steps of 2048, clamped to the row size); (4) a quadrant offset (0, 8, 16 or 24) from the signs, which together with the region gives an index into the bound
 * table; it names the candidate constellation points, fetched in two groups of four; (5) the add-compare-select step of the eight states, writing the winning
 * point and predecessor into the new history slot; (6) the state with the smallest path metric; (7) a traceback of 16 slots from that state; (8) the
 * differential
 * decode of the quadrant bits through VTB_DIFF_TBL, joined with the uncoded low bits.
 *
 * Branch metric of a candidate: ((dI^2 + dQ^2) >> 16) * 0x666 >> 12, with dI and dQ the 16-bit differences between the received point (not derotated) and the
 * candidate.  Ties in step 5 and step 6 go to the lowest index.
 *
 * Members of the V32Vtb object used: histBuf (history buffer), pathMetric[8] (by state), writeIdx (0..15), iMap / qMap / boundTab / regionTab (tables), family
 * (1..4), rowSize (2 / 4 / 6 / 8), tracebackDepth (16), prevQuadrant (differential decoder state, kept from call to call), lowMask (mask of the uncoded
 * bits: 7 /
 * 0xf / 0x1f / 0x3f) and diffShift (equal to family).  A history slot holds, per state, two 16-bit words: the winning candidate's symbol and its predecessor
 * state.
 *
 * @note Two asymmetries are deliberate: family 1 derotates with the full 32-bit sums shifted right by 2, family 3 narrows the sums to 16 bits before the shift
 *       right by 1; and step 4 looks at the sign of the low 16 bits of the derotated values only.
 * @note Step 5 is unrolled by hand, one block per state: the states 0, 2, 4 and 6 compete over the previous states 0..3, the states 1, 3, 5 and 7 over 4..7,
 *       each adding the candidate metrics in a permuted order.
 */
void VTB_decoder(char* vtbSubObj, short rxI, short rxQ, short* outBits)
{
  V32Vtb* vtb = (V32Vtb*)vtbSubObj;
  char* hist;                  /* circular history buffer */
  unsigned int wIdx;           /* write index (0..15) after this call's advance */
  char* slot;                  /* hist + wIdx * 0x20: this call's history slot */
  short trellisSize;           /* rowSize (2 / 4 / 6 / 8) */
  short metricBuf[18];         /* [2..9] = branch metrics of the four candidates fetched last, stored twice (at i + 2 and i + 6);
                                    [10..17] = the leak-rescaled previous path metrics, indexed by state */
  short candIdx[10];           /* [2..9] = the candidate point indices, stored twice like metricBuf */
  unsigned short oldSymbol[8]; /* symbol words of the slot about to be overwritten (16 symbols old), saved before the add-compare-select step */
  const short *imap, *qmap, *bound, *region;
  const short* pc;
  int i;
  int rotI, rotQ;
  short famSel;
  short quadCol, quadRow, maxIdx;
  unsigned short quadOff;
  short regionIdx, boundIdx;
  int bestState;

  hist = V32X_PG(char*, vtbSubObj, V32_VTB_OFF, (int)offsetof(V32Vtb, histBuf));

  /* step 1: advance the write index (mod 16), save the symbol words of the slot about to be overwritten, rescale the previous path metrics (leak 0x799a / 2^15)
   * as the baseline of this call's add-compare-select step */
  wIdx = (unsigned int)((unsigned short)(vtb->writeIdx + 1) & 0xf);
  vtb->writeIdx = (short)wIdx;
  slot = hist + (intptr_t)wIdx * 0x20;
  trellisSize = vtb->rowSize;
  for (i = 0; i < 8; i++)
  {
    short m = ((V32Vtb*)vtbSubObj)->pathMetric[i];
    metricBuf[i + 10] = (short)(((int)m * 0x799a) >> 15);
    oldSymbol[i] = *(unsigned short*)(slot + i * 4);
  }

  /* step 2: coarse derotation for the families 1 and 3 (they narrow differently, see @note); the families 2 and 4 use the point as received */
  famSel = vtb->family;
  if (famSel == 1)
  {
    rotI = ((int)rxI + (int)rxQ) >> 2;
    rotQ = ((int)rxQ - (int)rxI) >> 2;
  }
  else
  {
    rotI = rxI;
    rotQ = rxQ;
    if (famSel == 3)
    {
      rotI = (short)(rxI + rxQ) >> 1;
      rotQ = (short)(rxQ - rxI) >> 1;
    }
  }

  /* step 3: quantise the magnitudes of the (derotated) point to a column and a row, clamped to the row size */
  {
    int absI = (rotI ^ (rotI >> 31)) - (rotI >> 31);
    int absQ = (rotQ ^ (rotQ >> 31)) - (rotQ >> 31);
    quadCol = (short)((absI + 0x400) >> 11);
    quadRow = (short)((absQ + 0x400) >> 11);
  }
  maxIdx = (short)(trellisSize - 1);
  if (maxIdx < quadCol)
    quadCol = maxIdx;
  if (maxIdx < quadRow)
    quadRow = maxIdx;

  /* step 4: quadrant offset (0 / 8 / 16 / 24) from the signs of the low 16 bits of the derotated point */
  if ((short)rotI < 1)
  {
    quadOff = (unsigned short)((((short)rotQ < 1) ? 1 : 0) * 8 + 8);
  }
  else
  {
    quadOff = (unsigned short)((((0 < (short)rotQ) ? 1 : 0) - 1) & 0x18);
  }

  region = CFGP_SLOT(const short*, (intptr_t)(&vtb->regionTab));
  {
    short regionRow = (short)(quadRow + ((quadCol < quadRow) ? trellisSize : 0));
    regionIdx = (short)(quadCol + trellisSize * regionRow);
  }
  boundIdx = (short)(region[regionIdx] + (short)quadOff);

  bound = CFGP_SLOT(const short*, (intptr_t)(&vtb->boundTab));
  imap = CFGP_SLOT(const short*, (intptr_t)(&vtb->iMap));
  qmap = CFGP_SLOT(const short*, (intptr_t)(&vtb->qMap));

  /* first group: four candidate point indices from the bound table at boundIdx; their (I, Q) come from the iMap / qMap tables, and the branch metric is the
   * scaled squared distance to the received point */
  pc = &bound[boundIdx];
  for (i = 0; i < 4; i++)
  {
    short cand = *pc++;
    int dI = (int)(short)(rxI - imap[cand]);
    int dQ = (int)(short)(rxQ - qmap[cand]);
    short bm = (short)((((dI * dI + dQ * dQ) >> 16) * 0x666) >> 12);
    candIdx[i + 2] = cand;
    candIdx[i + 6] = cand;
    metricBuf[i + 2] = bm;
    metricBuf[i + 6] = bm;
  }

  /* step 5, first half: add-compare-select for the states 0, 2, 4 and 6 */
  {
    int best;
    short baseline, bestMetric;

    /* state 0 */
    baseline = (short)(metricBuf[10] + metricBuf[2]);
    best = 0;
    bestMetric = baseline;
    for (i = 1; i < 4; i++)
    {
      short cand = (short)(metricBuf[i + 10] + metricBuf[i + 2]);
      if (cand < bestMetric)
      {
        bestMetric = cand;
        best = i;
      }
    }
    vtb->pathMetric[0] = bestMetric;
    *(short*)(slot + 2) = (short)best;
    *(short*)(slot + 0) = candIdx[best + 2];

    /* state 2 */
    baseline = (short)(metricBuf[10] + metricBuf[7]);
    best = 0;
    bestMetric = baseline;
    for (i = 1; i < 4; i++)
    {
      short cand = (short)(metricBuf[i + 10] + metricBuf[7 - i]);
      if (cand < bestMetric)
      {
        bestMetric = cand;
        best = i;
      }
    }
    vtb->pathMetric[2] = bestMetric;
    *(short*)(slot + 10) = (short)best;
    *(short*)(slot + 8) = candIdx[7 - best];

    /* state 4 */
    baseline = (short)(metricBuf[10] + metricBuf[4]);
    best = 0;
    bestMetric = baseline;
    for (i = 1; i < 4; i++)
    {
      short cand = (short)(metricBuf[i + 10] + metricBuf[i + 4]);
      if (cand < bestMetric)
      {
        bestMetric = cand;
        best = i;
      }
    }
    vtb->pathMetric[4] = bestMetric;
    *(short*)(slot + 0x12) = (short)best;
    *(short*)(slot + 0x10) = candIdx[best + 4];

    /* state 6 */
    baseline = (short)(metricBuf[10] + metricBuf[5]);
    best = 0;
    bestMetric = baseline;
    for (i = 1; i < 4; i++)
    {
      short cand = (short)(metricBuf[i + 10] + metricBuf[5 - i]);
      if (cand < bestMetric)
      {
        bestMetric = cand;
        best = i;
      }
    }
    vtb->pathMetric[6] = bestMetric;
    *(short*)(slot + 0x1a) = (short)best;
    *(short*)(slot + 0x18) = candIdx[5 - best];
  }

  /* second group: the next four candidates (boundIdx + 4) for the states 1, 3, 5 and 7; they overwrite the first group, which is used up */
  pc = &bound[boundIdx + 4];
  for (i = 0; i < 4; i++)
  {
    short cand = *pc++;
    int dI = (int)(short)(rxI - imap[cand]);
    int dQ = (int)(short)(rxQ - qmap[cand]);
    short bm = (short)((((dI * dI + dQ * dQ) >> 16) * 0x666) >> 12);
    candIdx[i + 2] = cand;
    candIdx[i + 6] = cand;
    metricBuf[i + 2] = bm;
    metricBuf[i + 6] = bm;
  }

  /* step 5, second half: add-compare-select for the states 1, 3, 5 and 7; the first three start from the rescaled previous metric of state 4 (`r4`) */
  {
    short r4 = metricBuf[14];
    int best;
    short baseline, bestMetric;

    /* state 1 */
    baseline = (short)(r4 + metricBuf[2]);
    best = 4;
    bestMetric = baseline;
    for (i = 5; i < 8; i++)
    {
      short cand = (short)(metricBuf[i + 10] + metricBuf[i + 2]);
      if (cand < bestMetric)
      {
        bestMetric = cand;
        best = i;
      }
    }
    vtb->pathMetric[1] = bestMetric;
    *(short*)(slot + 6) = (short)best;
    *(short*)(slot + 4) = candIdx[best + 2];

    /* state 3 */
    baseline = (short)(r4 + metricBuf[7]);
    best = 4;
    bestMetric = baseline;
    for (i = 5; i < 8; i++)
    {
      short cand = (short)(metricBuf[i + 10] + metricBuf[0xb - i]);
      if (cand < bestMetric)
      {
        bestMetric = cand;
        best = i;
      }
    }
    vtb->pathMetric[3] = bestMetric;
    *(short*)(slot + 0x0e) = (short)best;
    *(short*)(slot + 0x0c) = candIdx[0xb - best];

    /* state 5 */
    baseline = (short)(r4 + metricBuf[5]);
    best = 4;
    bestMetric = baseline;
    for (i = 5; i < 8; i++)
    {
      short cand = (short)(metricBuf[i + 10] + metricBuf[9 - i]);
      if (cand < bestMetric)
      {
        bestMetric = cand;
        best = i;
      }
    }
    vtb->pathMetric[5] = bestMetric;
    *(short*)(slot + 0x16) = (short)best;
    *(short*)(slot + 0x14) = candIdx[9 - best];

    /* state 7 */
    baseline = (short)(r4 + metricBuf[4]);
    best = 4;
    bestMetric = baseline;
    for (i = 5; i < 8; i++)
    {
      short cand = (short)(metricBuf[i + 10] + metricBuf[i]);
      if (cand < bestMetric)
      {
        bestMetric = cand;
        best = i;
      }
    }
    vtb->pathMetric[7] = bestMetric;
    *(short*)(slot + 0x1e) = (short)best;
    *(short*)(slot + 0x1c) = candIdx[best];
  }

  /* step 6: the state with the smallest new path metric (the first wins a tie) */
  {
    int best = 0;
    short bestVal = vtb->pathMetric[0];
    for (i = 1; i < 8; i++)
    {
      short v = ((V32Vtb*)vtbSubObj)->pathMetric[i];
      if (v < bestVal)
      {
        best = i;
        bestVal = v;
      }
    }
    bestState = best;
  }

  /* step 7: traceback of `tracebackDepth` (16) slots, one lap of the history buffer, following the predecessor states back from the best state; it ends in a
   * state of the slot that was overwritten, whose saved symbol is 16 symbols old */
  {
    short depth = vtb->tracebackDepth;
    unsigned int tb = wIdx << 3;
    short count;
    for (count = 0; count < depth; count++)
    {
      bestState = (int)*(short*)(hist + 2 + (intptr_t)(tb + (unsigned int)bestState) * 4);
      tb = (tb - 8) & 0x78;
    }
  }

  /* step 8: differential decode of the quadrant bits through VTB_DIFF_TBL (index: quadrant bits + 4 * previous quadrant), joined with the uncoded low bits;
   * the quadrant bits become the previous quadrant of the next call */
  {
    unsigned short mask = vtb->lowMask;
    unsigned short symBits = (unsigned short)(oldSymbol[bestState] & mask);
    int shift = (int)vtb->diffShift;
    int prevQuadrant = (int)vtb->prevQuadrant;
    int quadBits = (int)(short)symBits >> shift;
    short decodedQuadrant = VTB_DIFF_TBL[(short)(quadBits + prevQuadrant * 4)];
    short lowBitsMask = (short)((1 << shift) - 1);

    *outBits = (short)((decodedQuadrant << shift) + (lowBitsMask & (short)symBits));
    vtb->prevQuadrant = (short)quadBits;
  }
}

/**
 * @brief Scramble `count` words in place with the transmit scrambler of the engine.
 * @param wrapperObj  connection wrapper (V32Wrap)
 * @param data        symbol words, scrambled in place
 * @param count       number of words
 */
void ScrambleDataV32(V32Wrap* wrapperObj, unsigned short* data, unsigned short count)
{
  V32Eng* eng = V32W_PG(V32Eng*, wrapperObj, (int)offsetof(V32Wrap, engPtr));
  SDMv32_scrambler(&eng->sdm, data, (short)count);
}

/**
 * @brief Descramble `count` words in place with the receive scrambler object of the engine.
 * @param wrapperObj  connection wrapper (V32Wrap)
 * @param data        symbol words, descrambled in place
 * @param count       number of words
 */
void DescrambleDataV32(V32Wrap* wrapperObj, unsigned short* data, unsigned short count)
{
  V32Eng* eng = V32W_PG(V32Eng*, wrapperObj, (int)offsetof(V32Wrap, engPtr));
  SDMv32_descrambler(&eng->rxSdm, data, (short)count);
}

/**
 * @brief Transmit one block: encode the bits into constellation codes and run them through the pulse-shaping filter.
 * @param wrapperObj  connection wrapper (V32Wrap)
 * @param rawBits     the bits to send, in the form the selected encoder reads
 * @param out         receives the line samples
 * @param count       number of symbols
 * @return the number of samples FPM_PPS_filter() wrote to `out`
 *
 * The encoder is the entry `smcEncSel` of the engine's encoder table (SetTxModeV32(): 0 differential, 1 absolute phase, 2 trellis-coded); it writes the codes
 * to the circular code buffer that FPM_PPS_filter() reads.
 */
unsigned short ModDataV32(V32Wrap* wrapperObj, void* rawBits, short* out, unsigned short count)
{
  V32Eng* eng = V32W_PG(V32Eng*, wrapperObj, (int)offsetof(V32Wrap, engPtr));
  smcenc_t encoder = V32E_PGI(smcenc_t, eng, (int)offsetof(V32Eng, d098), eng->smcEncSel);
  unsigned short ret;
  encoder(&eng->smc, &eng->circ, rawBits, count);
  ret = (unsigned short)FPM_PPS_filter(&eng->pps, &eng->circ, out, count);
  return ret;
}

/**
 * @brief Receive one block of line samples: matched filter, echo cancellation, AGC, symbol timing recovery and, while a carrier is present, the equaliser.
 * @param wrapperObj  connection wrapper (V32Wrap)
 * @param samples     line samples; the timing recovery and the equaliser read the recovered symbols from this buffer
 * @param outBits     receives the decoded symbols (written by the equaliser's decision callback)
 * @param count       number of input samples
 * @return the number of symbols the equaliser produced.  0 while either echo canceller adaptation flag is set (the block is only filtered and cancelled), in
 * the
 *         data role when the filtered level is below the disconnect threshold, and when the timing loop reports no carrier
 *
 * Side effect on the status byte at wrapper offset 0x31: bit 5 is set while the timing loop has a carrier and cleared otherwise, bit 6 is set when the AGC has
 * no detection.  With debug level 3 or more, "sre no carrier" is printed to stderr when the timing loop has lost the carrier.
 */
unsigned short DemodDataV32(V32Wrap* wrapperObj, short* samples, short* outBits, unsigned short count)
{
  V32Eng* eng;
  unsigned short filtCount;
  unsigned int nFilt;
  int i;
  V32Eng* eng2;
  int bCarrier = 0;

  eng = V32W_PG(V32Eng*, wrapperObj, (int)offsetof(V32Wrap, engPtr));

  /* matched filter into copyBufA; while either echo canceller adaptation flag is set the block ends after the cancellation */
  filtCount = (unsigned short)FPM_MRF_filter(V32E_MRF(eng), samples, V32E_PG(short*, eng, (int)offsetof(V32Eng, copyBufA)), (short)count);
  nFilt = (unsigned int)filtCount;
  if (eng->ecc.nearAdapt != 0 || eng->ecc.farAdapt != 0)
  {
    bCarrier = 1;
  }

  FPM_ECC_cancel(&eng->ecc, V32E_PG(short*, eng, (int)offsetof(V32Eng, copyBufA)), (unsigned short)nFilt);

  /* keep a copy of the cancelled samples in copyBufB and remember the block length */
  if (nFilt == 0)
  {
    eng2 = V32W_PG(V32Eng*, wrapperObj, (int)offsetof(V32Wrap, engPtr));
  }
  else
  {
    char *bufDst, *bufSrc;
    eng2 = V32W_PG(V32Eng*, wrapperObj, (int)offsetof(V32Wrap, engPtr));
    bufDst = V32E_PG(char*, eng2, (int)offsetof(V32Eng, copyBufB));
    bufSrc = V32E_PG(char*, eng2, (int)offsetof(V32Eng, copyBufA));
    for (i = 0; i < (int)nFilt; i++)
    {
      *(short*)((char*)bufDst + i * 2) = *(short*)((char*)bufSrc + i * 2);
    }
  }
  eng2->w50d4 = (unsigned short)nFilt;
  eng2->w50c8 = (unsigned short)nFilt;

  if (!bCarrier)
  {
    /* data role: a level below the disconnect threshold ends the block without carrier */
    if (V32W_PG(V32Hdx*, wrapperObj, (int)offsetof(V32Wrap, hdxPtr))->role == V32ROLE_DATA)
    {
      short rmsLevel = FPM_rms(V32E_PG(short*, eng2, (int)offsetof(V32Eng, copyBufA)), (short)nFilt);
      if (rmsLevel < wrapperObj->discThresh)
      {
        V32W_BYTE(wrapperObj, w030, 1) = V32W_BYTE(wrapperObj, w030, 1) & 0xdf;
        return 0;
      }
      eng2 = V32W_PG(V32Eng*, wrapperObj, (int)offsetof(V32Wrap, engPtr));
    }

    /* AGC, then symbol timing recovery; with the timing loop locked the equaliser runs on the recovered symbols */
    FPM_AGC_agc(V32E_AGC(eng2), V32E_PG(short*, eng2, (int)offsetof(V32Eng, copyBufA)), (unsigned short)nFilt);
    {
      V32Eng* eng3 = V32W_PG(V32Eng*, wrapperObj, (int)offsetof(V32Wrap, engPtr));
      unsigned int agcActive = V32E_AGCF(unsigned int, eng3, (int)(offsetof(V32Eng, agc) + offsetof(FpmAgc, active)), active);
      unsigned short symCount;
      V32W_BYTE(wrapperObj, w030, 1) = (V32W_BYTE(wrapperObj, w030, 1) & 0xbf) |
                                       (unsigned char)((V32E_AGCF(int, eng3, (int)(offsetof(V32Eng, agc) + offsetof(FpmAgc, detect)), detect) == 0) << 6);
      eng3->sre.loopEnable = eng3->opt[1] & agcActive;
      symCount = (unsigned short)FPM_SRE_recover(&eng3->sre, V32E_PG(unsigned short*, eng3, (int)offsetof(V32Eng, copyBufA)), (char*)(samples), (short)nFilt);
      eng3 = V32W_PG(V32Eng*, wrapperObj, (int)offsetof(V32Wrap, engPtr));
      if (eng3->sre.pllEnable != 0)
      {
        short hdxRole;
        V32W_BYTE(wrapperObj, w030, 1) = V32W_BYTE(wrapperObj, w030, 1) | 0x20;
        hdxRole = V32W_PG(V32Hdx*, wrapperObj, (int)offsetof(V32Wrap, hdxPtr))->role;
        eng3->fse.d044 = eng3->opt[2] & agcActive;
        if (hdxRole == V32ROLE_DATA)
        {
          if (eng3->sre.loopState == 0)
          {
            agcActive = 0;
            eng3->fse.d048 = 0;
          }
          else
          {
            eng3->fse.d048 = eng3->opt[3] & agcActive;
            agcActive = agcActive & eng3->opt[4];
          }
          eng3->fse.d04c = agcActive;
        }
        else
        {
          eng3->fse.d048 = 1;
          eng3->fse.d04c = eng3->opt[4];
        }

        return (unsigned short)FPM_FSE_receive(&eng3->fse, samples, outBits, symCount);
      }

      /* the timing loop has no carrier */
      if (dsplibs_debug_level < 3)
      {
        V32W_BYTE(wrapperObj, w030, 1) = V32W_BYTE(wrapperObj, w030, 1) & 0xdf;
      }
      else
      {
        fprintf(stderr, "sre no carrier\n");
        V32W_BYTE(wrapperObj, w030, 1) = V32W_BYTE(wrapperObj, w030, 1) & 0xdf;
      }
    }
  }
  return 0;
}

/**
 * @brief Select the transmit constellation and scrambler for a bit rate.
 * @param wrapperObj  connection wrapper (V32Wrap)
 * @param mode        V32MODE_HANDSHAKE, V32MODE_4800, V32MODE_9600, V32MODE_9600_TCM, V32MODE_7200, V32MODE_12000 or V32MODE_14400
 *
 * Per mode this sets the scrambler register length and the symbol encoder (`smcEncSel`: 0 differential, 1 absolute phase, 2 trellis-coded), the payload and
 * coded bit counts, the constellation tables (I / Q maps), the transmit connection class and the scrambler masks (symbol mask, keep mask and the two tap
 * shifts derived from the number of bits per symbol).  The modes 9600 and 9600 TCM share a tail that fixes the masks for four bits per symbol.
 * @note Any other mode (7, "no common rate") sets the low byte of the wrapper status word to 0x17 and bit 1 of its high byte, then derives the masks from zero
 *       bits per symbol; the constellation and encoder stay as they were.
 */
void SetTxModeV32(V32Wrap* wrapperObj, short mode)
{
  unsigned int symMask;
  short shiftWidth = 0;
  V32Eng* eng = 0;

  switch (mode)
  {
  case V32MODE_HANDSHAKE:
  case V32MODE_4800:
    eng = V32W_PG(V32Eng*, wrapperObj, (int)offsetof(V32Wrap, engPtr));
    if (mode == V32MODE_4800)
    {
      eng->smc.w002 = 0;
    }
    else
    {
      eng->smc.w002 = 1;
    }
    eng->smcEncSel = (unsigned short)(mode != V32MODE_4800);
    shiftWidth = 2;
    eng->sdm.symWidth = 2;
    eng->smc.rateIdx = 0;
    eng->smc.codedShift = 0;
    CFGP_SLOT_SET(const short*, &eng->pps.iMap, SMCv32_IMAP16);
    CFGP_SLOT_SET(const short*, &eng->pps.qMap, SMCv32_QMAP16);
    eng->txConnClass = 0;
    break;
  case V32MODE_9600:
    eng = V32W_PG(V32Eng*, wrapperObj, (int)offsetof(V32Wrap, engPtr));
    eng->sdm.symWidth = 4;
    eng->smc.rateIdx = 1;
    eng->smc.w002 = 0;
    eng->smcEncSel = 0;
    eng->smc.codedShift = 2;
    CFGP_SLOT_SET(const short*, &eng->pps.iMap, SMCv32_IMAP16);
    CFGP_SLOT_SET(const short*, &eng->pps.qMap, SMCv32_QMAP16);
    goto common_tail;
  case V32MODE_9600_TCM:
    eng = V32W_PG(V32Eng*, wrapperObj, (int)offsetof(V32Wrap, engPtr));
    eng->sdm.symWidth = 4;
    eng->smc.payloadBits = 2;
    eng->smc.rateIdx = 2;
    eng->smc.w002 = 2;
    eng->smcEncSel = 2;
    eng->smc.codedShift = 2;
    CFGP_SLOT_SET(const short*, &eng->pps.iMap, VTBv32_IMAP32);
    CFGP_SLOT_SET(const short*, &eng->pps.qMap, VTBv32_QMAP32);
  common_tail:
    eng->txConnClass = 1;
    eng->sdm.symMask = 0xf;
    eng->sdm.keepMask = 0xfffffff0;
    eng->sdm.tapShiftA = eng->sdm.tapPos + -4;
    eng->sdm.tapShiftB = eng->sdm.regLen + -4;
    return;
  case V32MODE_7200:
    eng = V32W_PG(V32Eng*, wrapperObj, (int)offsetof(V32Wrap, engPtr));
    shiftWidth = 3;
    eng->sdm.symWidth = 3;
    eng->smc.payloadBits = 1;
    eng->smc.rateIdx = 3;
    eng->smcEncSel = 2;
    CFGP_SLOT_SET(const short*, &eng->pps.iMap, VTBv32_IMAP16T);
    CFGP_SLOT_SET(const short*, &eng->pps.qMap, VTBv32_QMAP16T);
    eng->txConnClass = 3;
    break;
  case V32MODE_12000:
    eng = V32W_PG(V32Eng*, wrapperObj, (int)offsetof(V32Wrap, engPtr));
    shiftWidth = 5;
    eng->sdm.symWidth = 5;
    eng->smc.payloadBits = 3;
    eng->smc.rateIdx = 4;
    eng->smcEncSel = 2;
    CFGP_SLOT_SET(const short*, &eng->pps.iMap, VTBv32_IMAP64);
    CFGP_SLOT_SET(const short*, &eng->pps.qMap, VTBv32_QMAP64);
    eng->txConnClass = 4;
    break;
  case V32MODE_14400:
    eng = V32W_PG(V32Eng*, wrapperObj, (int)offsetof(V32Wrap, engPtr));
    shiftWidth = 3;
    eng->sdm.symWidth = 6;
    eng->smc.payloadBits = 4;
    eng->smc.rateIdx = 5;
    eng->smc.w002 = 2;
    eng->smcEncSel = 2;
    CFGP_SLOT_SET(const short*, &eng->pps.iMap, VTBv32_IMAP128);
    CFGP_SLOT_SET(const short*, &eng->pps.qMap, VTBv32_QMAP128);
    eng->txConnClass = 5;
    break;
  default:
    V32W_BYTE(wrapperObj, w030, 0) = 0x17;
    V32W_BYTE(wrapperObj, w030, 1) = V32W_BYTE(wrapperObj, w030, 1) | 2;
    eng = V32W_PG(V32Eng*, wrapperObj, (int)offsetof(V32Wrap, engPtr));
  }
  symMask = (1U << (unsigned char)shiftWidth) - 1;
  eng->sdm.symMask = symMask;
  eng->sdm.keepMask = ~symMask;
  eng->sdm.tapShiftA = eng->sdm.tapPos - shiftWidth;
  eng->sdm.tapShiftB = eng->sdm.regLen - shiftWidth;
}

/**
 * @brief Select the receive decision callback and descrambler for a bit rate.
 * @param wrapperObj  connection wrapper (V32Wrap)
 * @param mode        V32MODE_HANDSHAKE .. V32MODE_14400, as for SetTxModeV32()
 *
 * Per mode this stores the descrambler register length, the receive connection class and the equaliser decision callback (FSE_decision_AB / _4pt / _16pt /
 * _32pt / _16Tpt / _64pt / _128pt) and derives the descrambler masks from the bits per symbol.  The trellis-coded modes (9600 TCM, 7200, 12000, 14400) also
 * initialise the Viterbi decoder with VTBv32_init().  Handshake and 4800 reset the equaliser history words; 9600 and 9600 TCM set them for the 16 / 32-point
 * constellations.
 * @note Any other mode sets the same error flags as SetTxModeV32() and leaves the callback and descrambler masks as they were.
 */
void SetRxModeV32(V32Wrap* wrapperObj, short mode)
{
  V32Hist* hist;
  V32Eng* eng = 0;
  void (*decisionFn)(void) = 0;
  unsigned int vtbRate = 0;
  unsigned int symMask;
  short shiftWidth = 0;
  int haveVtb = 0;

  switch ((int)mode)
  {
  case V32MODE_HANDSHAKE:
  case V32MODE_4800:
    eng = V32W_PG(V32Eng*, wrapperObj, (int)offsetof(V32Wrap, engPtr));
    shiftWidth = 2;
    eng->rxSdm.symWidth = 2;
    eng->w509c = 0;
    hist = V32E_PG(V32Hist*, eng, (int)offsetof(V32Eng, fse.histObj));
    eng->w5098 = 0;
    hist->w004 = 0;
    hist->w000 = 0;
    eng->rxConnClass = 0;
    hist->stateCount = 0;
    if (mode == V32MODE_HANDSHAKE)
    {
      hist->epochFlag = 0;
      decisionFn = (void (*)(void))FSE_decision_AB;
    }
    else
    {
      decisionFn = (void (*)(void))FSE_decision_4pt;
    }
    V32E_PS(v32fn_t, eng, (int)offsetof(V32Eng, fse.decisionFn), decisionFn);
    goto common_mask;
  case V32MODE_9600:
    eng = V32W_PG(V32Eng*, wrapperObj, (int)offsetof(V32Wrap, engPtr));
    shiftWidth = 4;
    eng->rxSdm.symWidth = 4;
    hist = V32E_PG(V32Hist*, eng, (int)offsetof(V32Eng, fse.histObj));
    eng->w5098 = 1;
    eng->w509c = 2;
    hist->w000 = 1;
    hist->w004 = 2;
    V32E_PS(v32fn_t, eng, (int)offsetof(V32Eng, fse.decisionFn), (v32fn_t)FSE_decision_16pt);
    eng->rxConnClass = 1;
    goto common_mask;
  case V32MODE_9600_TCM:
    eng = V32W_PG(V32Eng*, wrapperObj, (int)offsetof(V32Wrap, engPtr));
    shiftWidth = 4;
    eng->w5098 = 1;
    hist = V32E_PG(V32Hist*, eng, (int)offsetof(V32Eng, fse.histObj));
    eng->rxSdm.symWidth = 4;
    eng->w509c = 2;
    hist->w000 = 1;
    hist->w004 = 2;
    V32E_PS(v32fn_t, eng, (int)offsetof(V32Eng, fse.decisionFn), (v32fn_t)FSE_decision_32pt);
    eng->rxConnClass = 1;
    vtbRate = 2;
    haveVtb = 1;
    break;
  case V32MODE_7200:
    eng = V32W_PG(V32Eng*, wrapperObj, (int)offsetof(V32Wrap, engPtr));
    shiftWidth = 3;
    eng->rxSdm.symWidth = 3;
    V32E_PS(v32fn_t, eng, (int)offsetof(V32Eng, fse.decisionFn), (v32fn_t)FSE_decision_16Tpt);
    eng->rxConnClass = 3;
    vtbRate = 3;
    haveVtb = 1;
    break;
  case V32MODE_12000:
    eng = V32W_PG(V32Eng*, wrapperObj, (int)offsetof(V32Wrap, engPtr));
    shiftWidth = 5;
    eng->rxSdm.symWidth = 5;
    V32E_PS(v32fn_t, eng, (int)offsetof(V32Eng, fse.decisionFn), (v32fn_t)FSE_decision_64pt);
    vtbRate = 4;
    eng->rxConnClass = 4;
    haveVtb = 1;
    break;
  case V32MODE_14400:
    eng = V32W_PG(V32Eng*, wrapperObj, (int)offsetof(V32Wrap, engPtr));
    eng->rxSdm.symWidth = 6;
    V32E_PS(v32fn_t, eng, (int)offsetof(V32Eng, fse.decisionFn), (v32fn_t)FSE_decision_128pt);
    eng->rxConnClass = 5;
    vtbRate = 5;
    shiftWidth = 3;
    haveVtb = 1;
    break;
  default:
    V32W_BYTE(wrapperObj, w030, 1) = V32W_BYTE(wrapperObj, w030, 1) | 2;
    V32W_BYTE(wrapperObj, w030, 0) = 0x17;
    goto skip_vtb;
  }
  if (haveVtb)
  {
    /* the trellis-coded modes: reset the Viterbi decoder for this rate class */
    VTBv32_init(&V32E_PG(V32Hist*, eng, (int)offsetof(V32Eng, fse.histObj))->vtb, vtbRate, 0);
  }
skip_vtb:
  eng = V32W_PG(V32Eng*, wrapperObj, (int)offsetof(V32Wrap, engPtr));
common_mask:
  symMask = (1U << (unsigned char)shiftWidth) - 1;
  eng->rxSdm.symMask = symMask;
  eng->rxSdm.keepMask = ~symMask;
  eng->rxSdm.tapShiftA = eng->rxSdm.tapPos - shiftWidth;
  eng->rxSdm.tapShiftB = eng->rxSdm.regLen - shiftWidth;
}

/**
 * @brief Load the transmit scrambler register of the engine.
 * @param wrapperObj  connection wrapper (V32Wrap)
 * @param seed        new register value (the word at engine offset 0x40)
 */
void SeedScramblerV32(V32Wrap* wrapperObj, unsigned int seed) { V32W_PG(V32Eng*, wrapperObj, (int)offsetof(V32Wrap, engPtr))->sdm.shiftReg = seed; }

/**
 * @brief Map the negotiated receive rate to the rate class.
 * @param wrapperObj  connection wrapper (V32Wrap)
 * @return V32RC_14400, V32RC_12000, V32RC_9600_TCM (9600 with `highRate` set) or V32RC_9600, V32RC_7200, V32RC_4800, otherwise V32RC_NONE
 */
unsigned int GetRateV32(V32Wrap* wrapperObj)
{
  short rate;
  unsigned int rateClass = V32RC_14400;

  rate = wrapperObj->rxRate;
  if (rate != RATE_14400 && (rateClass = V32RC_12000, rate != RATE_12000))
  {
    if (rate == RATE_9600)
    {
      return 2 - (wrapperObj->highRate == 0);
    }
    rateClass = V32RC_7200;
    if (rate != RATE_7200)
    {
      rateClass = (unsigned int)((rate == RATE_4800) - 1) & 6;
    }
  }
  return rateClass;
}

/**
 * @brief Switch the equaliser adaptation mode.
 * @param wrapperObj  connection wrapper (V32Wrap)
 * @param mode        1 clears the engine word at offset 0x10; 2 sets it and clears the mode word at 0x23c; 3 sets both; other values do nothing
 */
void SetAdaptEqV32(V32Wrap* wrapperObj, unsigned short mode)
{
  V32Eng* eng;
  if (mode == 2)
  {
    eng = V32W_PG(V32Eng*, wrapperObj, (int)offsetof(V32Wrap, engPtr));
    eng->opt[4] = 1;
    eng->fse.w038 = 0;
    return;
  }
  if (mode < 3)
  {
    if (mode == 1)
    {
      V32W_PG(V32Eng*, wrapperObj, (int)offsetof(V32Wrap, engPtr))->opt[4] = 0;
      return;
    }
  }
  else if (mode == 3)
  {
    eng = V32W_PG(V32Eng*, wrapperObj, (int)offsetof(V32Wrap, engPtr));
    eng->opt[4] = 1;
    eng->fse.w038 = 1;
    return;
  }
}

/**
 * @brief Switch the echo canceller adaptation mode.
 * @param wrapperObj  connection wrapper (V32Wrap)
 * @param mode        0 re-initialise the canceller from its current configuration (FPM_ECC_init(ecc, ecc, 0)) and reset the code ring to 0x30 entries with both
 *                    read indices 0; 1 stop adaptation (near and far flags and the two engine words at 0x14 and 0x18 cleared); 2 clear the code history ring
 *                    and start adaptation (the same flags set); 3 divide the word at engine offset 0x140 by 10; other values do nothing
 */
void SetAdaptEcV32(V32Wrap* wrapperObj, unsigned short mode)
{
  V32Eng* eng;
  if (mode == 1)
  {
    eng = V32W_PG(V32Eng*, wrapperObj, (int)offsetof(V32Wrap, engPtr));
    eng->ecc.nearAdapt = 0;
    eng->ecc.farAdapt = 0;
    eng->opt[5] = 0;
    eng->opt[6] = 0;
    return;
  }
  if (mode < 2)
  {
    if (mode == 0)
    {
      eng = V32W_PG(V32Eng*, wrapperObj, (int)offsetof(V32Wrap, engPtr));
      eng->ecc.w064 = wrapperObj->lookAhead;
      eng->ecc.w066 = 0x30;
      FPM_ECC_init(&eng->ecc, (const int*)&eng->ecc, 0);
      eng = V32W_PG(V32Eng*, wrapperObj, (int)offsetof(V32Wrap, engPtr));
      eng->ecc.codeRingLen = 0x30;
      eng->ecc.codeRdNear = 0;
      eng->ecc.codeRdFar = 0;
      return;
    }
  }
  else
  {
    if (mode == 2)
    {
      int i;
      eng = V32W_PG(V32Eng*, wrapperObj, (int)offsetof(V32Wrap, engPtr));
      if (0 < eng->ecc.codeRingLen)
      {
        char* buf = V32E_PG(char*, eng, (int)offsetof(V32Eng, ecc.codeHist));
        for (i = 0; i < eng->ecc.codeRingLen; i++)
        {
          *(short*)((char*)buf + i * 2) = 0;
        }
      }
      eng->opt[5] = 1;
      eng->ecc.w01a = 0;
      eng->ecc.nearAdapt = 1;
      eng->ecc.farAdapt = 1;
      eng->opt[6] = 1;
      return;
    }
    if (mode == 3)
    {
      eng = V32W_PG(V32Eng*, wrapperObj, (int)offsetof(V32Wrap, engPtr));
      eng->ecc.stepSize = (short)eng->ecc.stepSize / 10;
      return;
    }
  }
}

/**
 * @brief Enable or disable the receive loops.
 * @param wrapperObj  connection wrapper (V32Wrap)
 * @param mode        1 clears the four loop-enable words at the start of the engine; 2 and 3 set them; 0 and values above 3 do nothing
 */
void SetRxLoopsV32(V32Wrap* wrapperObj, unsigned short mode)
{
  unsigned int* loopEnable;
  if (mode != 1)
  {
    if (mode != 0 && mode < 4)
    {
      loopEnable = V32W_PG(unsigned int*, wrapperObj, (int)offsetof(V32Wrap, engPtr));
      loopEnable[2] = 1;
      loopEnable[3] = 1;
      *loopEnable = 1;
      loopEnable[1] = 1;
    }
    return;
  }
  loopEnable = V32W_PG(unsigned int*, wrapperObj, (int)offsetof(V32Wrap, engPtr));
  loopEnable[2] = 0;
  loopEnable[3] = 0;
  *loopEnable = 0;
  loopEnable[1] = 0;
}

/**
 * @brief Size the echo canceller code ring and set its read positions for a round-trip delay.
 * @param wrapperObj  connection wrapper (V32Wrap)
 * @param roundTrip   measured round-trip delay; the larger of it and the engine word at 0xe4, limited to 0 .. the engine word at 0xe0, is used as `delay`
 *
 * Ring length = handshake word `w09c` + delay + 2 * symbol length.  The far read index is the symbol length, the near read index is delay + symbol length
 * modulo the ring length, and the circular code buffer's read index is `w09c` + symbol length / 2 + the near index (modulo the ring length); its write index
 * starts at the read index.  The ring is then filled with the code 0x10 (silence).
 */
void SetECRndTripDelayV32(V32Wrap* wrapperObj, unsigned short roundTrip)
{
  short symLen;
  V32Eng* eng;
  int i;
  unsigned short delay;
  short startIdx, delayBase, ringLen;

  delayBase = V32W_PG(V32Hdx*, wrapperObj, (int)offsetof(V32Wrap, hdxPtr))->w09c;
  eng = V32W_PG(V32Eng*, wrapperObj, (int)offsetof(V32Wrap, engPtr));

  /* delay: the larger of the stored and the measured delay, not below 0, not above the first echo canceller word */
  symLen = V32_SYMBOL_LEN[wrapperObj->w018];
  delay = eng->ecc.w004;
  if ((short)eng->ecc.w004 < (short)roundTrip)
  {
    delay = roundTrip;
  }
  delay = (unsigned short)((short)~delay >> 0x1f) & delay;
  if ((short)eng->ecc.w000 < (short)delay)
  {
    delay = eng->ecc.w000;
  }

  /* ring length and the two read positions of the echo canceller */
  eng->ecc.codeRdFar = symLen;
  ringLen = delayBase + delay + symLen * 2;
  eng->ecc.codeRingLen = ringLen;
  startIdx = delay + symLen;
  if (ringLen <= startIdx)
  {
    startIdx = startIdx - ringLen;
  }
  eng->ecc.codeRdNear = startIdx;

  /* the circular code buffer reads from there and writes at its own read position */
  delayBase = delayBase + symLen / 2 + eng->ecc.codeRdNear;
  if (eng->ecc.codeRingLen <= delayBase)
  {
    delayBase = delayBase - eng->ecc.codeRingLen;
  }
  eng->circ.readIdx = delayBase;
  eng->circ.ringLen16 = eng->ecc.codeRingLen;
  eng->circ.writeIdx = eng->circ.readIdx;

  /* fill the code ring with the silence code */
  if (0 < eng->ecc.codeRingLen)
  {
    char* buf = V32E_PG(char*, eng, (int)offsetof(V32Eng, ecc.codeHist));
    for (i = 0; i < eng->ecc.codeRingLen; i++)
    {
      *(short*)((char*)buf + i * 2) = 0x10;
    }
  }
}

/** @brief Transmit clock synchronisation hook; does nothing. */
void TxClockSyncV32(void) {}

/**
 * @brief Read the epoch flag of the engine's history object.
 * @param wrapperObj  connection wrapper (V32Wrap)
 * @return the flag word (V32Hist::epochFlag); it is not cleared
 */
unsigned int EpochDetectV32(V32Wrap* wrapperObj)
{
  return V32E_PG(V32Hist*, V32W_PG(V32Eng*, wrapperObj, (int)offsetof(V32Wrap, engPtr)), (int)offsetof(V32Eng, fse.histObj))->epochFlag;
}

/**
 * @brief Test and clear the retrain event of the engine's history object.
 * @param wrapperObj  connection wrapper (V32Wrap)
 * @return 1 if bit 0 of V32Hist::evFlags was set (it is cleared, the state counter reset and the retrain count incremented), else 0
 */
int RetrainDetectV32(V32Wrap* wrapperObj)
{
  V32Hist* hist = V32E_PG(V32Hist*, V32W_PG(V32Eng*, wrapperObj, (int)offsetof(V32Wrap, engPtr)), (int)offsetof(V32Eng, fse.histObj));
  int hit = (hist->evFlags & 1) != 0;
  if (hit)
  {
    hist->stateCount = 0;
    hist->evFlags = hist->evFlags & 0xfffe;
    hist->retrainCount = hist->retrainCount + 1;
  }
  return hit;
}

/**
 * @brief Test and clear the rate renegotiation event of the engine's history object.
 * @param wrapperObj  connection wrapper (V32Wrap)
 * @return 1 if bit 1 of V32Hist::evFlags was set (it is cleared and the state counter reset), else 0
 */
int RenegotiateDetectV32(V32Wrap* wrapperObj)
{
  V32Hist* hist = V32E_PG(V32Hist*, V32W_PG(V32Eng*, wrapperObj, (int)offsetof(V32Wrap, engPtr)), (int)offsetof(V32Eng, fse.histObj));
  int hit = (hist->evFlags & 2) != 0;
  if (hit)
  {
    hist->stateCount = 0;
    hist->evFlags = hist->evFlags & 0xfffd;
  }
  return hit;
}

/**
 * @brief Transmit `count` symbols of silence: fill the code ring with the silence code and run the pulse-shaping filter.
 * @param wrapperObj  connection wrapper (V32Wrap)
 * @param unusedIn    unused (keeps the signature of the other transmit handlers)
 * @param out         receives the line samples
 * @param count       number of symbols
 * @return the number of samples FPM_PPS_filter() wrote to `out`
 *
 * Each symbol writes the code 0x10 at the circular code buffer's write index (16-bit slots, wrapping at the ring length) and steps the symbol phase back by one
 * (mod 4); the write index and the phase are stored back before the filter runs.
 */
unsigned short TxNoCarrierV32(V32Wrap* wrapperObj, void* unusedIn, short* out, short count)
{
  V32Eng* eng;
  char* codeBuf;
  short ringLen;
  short phase;
  short i;
  unsigned short produced;
  unsigned int writeIdx;

  (void)unusedIn;
  eng = V32W_PG(V32Eng*, wrapperObj, (int)offsetof(V32Wrap, engPtr));
  codeBuf = V32E_PG(char*, eng, (int)offsetof(V32Eng, circ.codeBuf));
  phase = eng->smc.phase;
  writeIdx = (unsigned int)eng->circ.writeIdx;
  ringLen = eng->circ.ringLen16;
  for (i = (short)(count - 1); i != -1; i--)
  {
    /* a 16-bit slot: the same stride as the reads of FPM_PPS_filter() and the writes of the symbol encoders */
    *(unsigned short*)((char*)codeBuf + writeIdx * 2) = 0x10;
    {
      short candidate = (short)(writeIdx + 1);
      writeIdx = (candidate < ringLen) ? (unsigned int)(unsigned short)candidate : 0;
    }
    phase = (short)((phase + 3) & 3);
  }

  eng->smc.phase = (unsigned short)phase;
  eng->circ.writeIdx = (short)writeIdx;
  produced = (unsigned short)FPM_PPS_filter(&eng->pps, &eng->circ, out, (unsigned short)count);
  return produced;
}

/**
 * @brief Fill the receive output with the clamp value 0xff.
 * @param wrapperObj  connection wrapper (V32Wrap)
 * @param unusedIn    unused
 * @param out         receives V32Hdx::frameLen words of 0xff
 * @return V32Hdx::frameLen
 * @warning `out` must hold `frameLen` words; the length is not passed in.
 */
unsigned short RxClampV32(V32Wrap* wrapperObj, void* unusedIn, unsigned short* out)
{
  V32Hdx* hdx;
  short remaining;

  (void)unusedIn;
  hdx = V32W_PG(V32Hdx*, wrapperObj, (int)offsetof(V32Wrap, hdxPtr));
  remaining = hdx->frameLen;
  while (remaining != 0)
  {
    *out = 0xff;
    out = out + 1;
    remaining = remaining + -1;
  }
  return hdx->frameLen;
}
