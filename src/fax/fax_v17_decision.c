/**
 * @file fax_v17_decision.c
 * @brief The V.17 receiver's symbol-decision callbacks: start-of-training detector, equalizer training sequence, bridge, and the 16 / 32 / 64 / 128 point
 *   trellis decisions.
 *
 * The callbacks are the FaxFse decision functions (`fse->cfg.cb`) that FAXFSE_receive() calls once per symbol.  Each reads the equalized symbol
 * (`fse->outI[fse->idx]`, `fse->outQ[fse->idx]`), stores the angle and magnitude of the ideal point in the two slots it is given and returns the decoded
 * code.  State lives in the FaxDec17 history object (`fse->cfg.hist`, fax_v17_decision.h); the tables are in fax_v17_decision_tables.c.
 *
 * Sequence of one reception:
 * - FAX_FSE_decision_AB: the start-of-training detector (installed by V17RX_create).  It smooths the magnitudes of large / small phase steps and watches a
 *   three-pair history for the point where the signal stops moving; then it clears its history, forces an equalizer update and installs FSE_decision_eqtrn.
 * - FSE_decision_eqtrn: the equalizer training sequence.  A two-bits-per-symbol shift register picks one of four points; after `limit` symbols it hands over
 *   to FSEv17_decision[hist->sel] (short training, `hist->flag` set) or to FSE_Bridge_det.
 * - FSE_Bridge_det: nearest of four points for 0x3f symbols, then FSEv17_decision[hist->sel].
 * - FAX_FSE_decision_16pt / _32pt / _64pt / _128pt: nearest constellation point of the rate, then one Viterbi step (FAXVTB_decoder on `hist->vtb`); they
 *   return the decoded bits zero extended.
 *
 * Arithmetic conventions: every 16-bit quantity is truncated to 16 bits where the algorithm does so (sums of two scaled squared distances are cut to 16 bits
 * before they are compared); ties keep the first point (strict `<`); the call counter `hist->ctr` wraps from 0x7fff to 0x4000 (the start-of-training detector
 * does not wrap it).
 *
 * @warning FSE_Bridge_det and FSE_decision_eqtrn index FSEv17_decision[] with `hist->sel` and install NULL as the callback when it is outside 0 .. 3; the
 *   receiver always stores the rate index (0 .. 3) there.
 * @note FAX_FSE_decision_128pt prints "Index fault" and falls back to table index 0 when the table index leaves 0 .. 0x7f.  That cannot happen: the candidate
 *   index is below 0x20 and the sector offset is at most 0x60.
 */
#include <stddef.h>
#include "fax_v17_decision.h"
#include "fax_v17_decision_tables.h"
#include "core/dsp_asm.h"
#include "core/dsp_intmath.h" /* WADD */
#include "core/dsp_host.h"

/* absolute value of a 32-bit quantity */
static inline int abs16(int v) { return v < 0 ? -v : v; }

/* the call counter of the trellis decisions: +1, the value 0x8000 becomes 0x4000 (16 bit); sums that can leave the int range are formed unsigned elsewhere */
static inline void bump_ctr(FaxDec17* h)
{
  unsigned short n = (unsigned short)(h->ctr + 1u);
  h->ctr = (n == 0x8000) ? (unsigned short)0x4000 : n;
}

/* squared distance term ((a - b)^2 >> sh), with a - b taken as a 16-bit value */
static inline int sq16(int a, int b, int sh)
{
  int d = (short)(a - b);
  return (int)((unsigned int)d * (unsigned int)d) >> sh;
}

/**
 * @brief 7200 bit/s decision: nearest of the 16 constellation points, then a Viterbi step.
 * @param f      FSE block (equalized symbol at `outI/outQ[idx]`, history in `cfg.hist`)
 * @param angle  receives the ideal point's phase (DECv17_ANGL7200)
 * @param mag    receives the ideal point's radius (DECv17_MAG7200, selected by (|I| + |Q|) >> 13)
 * @return decoded bits of the Viterbi step, zero extended
 */
int FAX_FSE_decision_16pt(FaxFse* f, short* angle, short* mag)
{
  FaxDec17* h = (FaxDec17*)f->cfg.hist;
  short idx = f->idx;
  int symI = f->outI[idx], symQ = f->outQ[idx];
  int best = 0;
  short bestDist = 0x7fff;
  int i, k;
  short out;

  bump_ctr(h);

  /* nearest point: scaled squared distance over all 16 points */
  for (i = 0; i <= 15; i++)
  {
    short d = (short)(sq16(symI, (unsigned short)DECv17_IMAP16[i], 16) + sq16(symQ, (unsigned short)DECv17_QMAP16[i], 16));
    if (d < bestDist)
    {
      bestDist = d;
      best = i;
    }
  }

  /* ideal point's radius class (1 .. 3) and phase, then the trellis step on the raw symbol */
  k = (abs16(DECv17_IMAP16[best]) + abs16(DECv17_QMAP16[best])) >> 13;
  *mag = (short)DECv17_MAG7200[k - 1];
  *angle = DECv17_ANGL7200[best];
  FAXVTB_decoder(&h->vtb, (short)symI, (short)symQ, &out);
  return (unsigned short)out;
}

/**
 * @brief 9600 bit/s decision: rotate the symbol into one octant, find the nearest of the analysis points there, then a Viterbi step.
 * @param f      FSE block (equalized symbol at `outI/outQ[idx]`, history in `cfg.hist`)
 * @param angle  receives the ideal point's phase (DECv17_ANGL9600T)
 * @param mag    receives the ideal point's radius (DECv17_MAG9600T)
 * @return decoded bits of the Viterbi step, zero extended
 */
int FAX_FSE_decision_32pt(FaxFse* f, short* angle, short* mag)
{
  FaxDec17* h = (FaxDec17*)f->cfg.hist;
  short idx = f->idx;
  int symI = f->outI[idx], symQ = f->outQ[idx];
  int base = 0, nCand = 3, best = 0, sector, i, cosRot, sinRot, rotI, rotQ, absI, absQ;
  short bestDist = 0x7fff, out;

  bump_ctr(h);

  /* rotate by the multiple of 90 degrees that puts the symbol into the first quadrant sector (Q15 cos / sin) */
  absI = (short)abs16(symI);
  absQ = (short)abs16(symQ);
  sector = (symI > symQ) ? (absI > absQ) + 2 : (absI > absQ);
  cosRot = DECv17_COS_ROT_ANGLE[sector];
  sinRot = DECv17_SIN_ROT_ANGLE[sector];
  rotI = (short)(((symI * cosRot) >> 15) - ((symQ * sinRot) >> 15));
  rotQ = (short)(((symQ * cosRot) >> 15) + ((symI * sinRot) >> 15));

  /* the candidate Q levels depend on the rotated I coordinate */
  if (rotI > 0x16a0)
  {
    base = 3;
    if (rotI > 0x2d41 && rotQ <= 0x2d40)
    {
      nCand = 2;
      base = 6;
    }
  }

  /* nearest Q level among the candidates */
  for (i = base; i < base + nCand; i++)
  {
    int q = DECv17_ANA_QMAP[i];
    short dd = (rotQ < q) ? (short)(q - rotQ) : (short)(rotQ - q);
    if (dd < bestDist)
    {
      bestDist = dd;
      best = i;
    }
  }

  /* the two points that share a Q level are told apart by the full squared distance */
  if (best == 3 || best == 6)
  {
    short ea = (short)(sq16((unsigned short)DECv17_ANA_QMAP[3], rotQ, 15) + sq16(0x21f1, rotI, 15));
    short eb = (short)(sq16((unsigned short)DECv17_ANA_QMAP[6], rotQ, 15) + sq16(0x3891, rotI, 15));
    best = 3 + 3 * (eb < ea);
  }

  i = (short)(best + sector * 8);
  *mag = DECv17_MAG9600T[i];
  *angle = DECv17_ANGL9600T[i];
  FAXVTB_decoder(&h->vtb, (short)symI, (short)symQ, &out);
  return (unsigned short)out;
}

/**
 * @brief 12000 bit/s decision: pick the 4-point cell of the symbol, nearest point inside it, then a Viterbi step.
 * @param f      FSE block (equalized symbol at `outI/outQ[idx]`, history in `cfg.hist`)
 * @param angle  receives the ideal point's phase (DECv17_ANGL12000)
 * @param mag    receives the ideal point's radius (DECv17_MAG12000)
 * @return decoded bits of the Viterbi step, zero extended
 */
int FAX_FSE_decision_64pt(FaxFse* f, short* angle, short* mag)
{
  FaxDec17* h = (FaxDec17*)f->cfg.hist;
  short idx = f->idx;
  int symI = f->outI[idx], symQ = f->outQ[idx];
  int base, best = 0, i;
  short bestDist = 0x7fff, out;

  bump_ctr(h);

  /* coarse cell (a group of four table entries) from the signs and the 0x2000 grid lines of I and Q */
  if (symI > 0)
  {
    if (symQ > 0)
    {
      if (symI > 0x2000)
        base = (symQ > 0x1fff) ? 0x28 : 0x2c;
      else
        base = (symQ > 0x1fff) ? 0x20 : 0x24;
    }
    else
    {
      if (symI > 0x2000)
        base = (symQ >= -0x2000) ? 0x38 : 0x3c;
      else
        base = (symQ >= -0x2000) ? 0x30 : 0x34;
    }
  }
  else
  {
    if (symQ > 0)
    {
      if (symI <= -0x2000)
        base = (symQ > 0x1fff) ? 0 : 4;
      else
        base = (symQ > 0x1fff) ? 8 : 0xc;
    }
    else
    {
      if (symI <= -0x2000)
        base = (symQ >= -0x2000) ? 0x10 : 0x14;
      else
        base = (symQ >= -0x2000) ? 0x18 : 0x1c;
    }
  }

  /* nearest of the four points of the cell */
  for (i = base; i < base + 4; i++)
  {
    short d = (short)(sq16(symI, (unsigned short)DECv17_IMAP64[i], 13) + sq16(symQ, (unsigned short)DECv17_QMAP64[i], 13));
    if (d < bestDist)
    {
      bestDist = d;
      best = i;
    }
  }

  *mag = DECv17_MAG12000[best];
  *angle = DECv17_ANGL12000[best];
  FAXVTB_decoder(&h->vtb, (short)symI, (short)symQ, &out);
  return (unsigned short)out;
}

/**
 * @brief 14400 bit/s decision: rotate into one sector, locate the 4-point cell (two cells are resolved by a distance comparison), nearest point, then a
 *   Viterbi step.
 * @param f      FSE block (equalized symbol at `outI/outQ[idx]`, history in `cfg.hist`)
 * @param angle  receives the ideal point's phase (DECv17_ANGL14400)
 * @param mag    receives the ideal point's radius (DECv17_MAG14400)
 * @return decoded bits of the Viterbi step, zero extended
 */
int FAX_FSE_decision_128pt(FaxFse* f, short* angle, short* mag)
{
  FaxDec17* h;
  short idx = f->idx;
  int symI = f->outI[idx], symQ = f->outQ[idx];
  int base = 0, tieCase = 0, bestIdx = 0, sector, i, cosRot, sinRot, rotI, rotQ, absI, absQ, tabIdx;
  short bestDist = 0x7fff, out;

  h = (FaxDec17*)f->cfg.hist;
  absI = (short)abs16(symI);
  absQ = (short)abs16(symQ);
  bump_ctr(h);

  /* rotate by a multiple of 90 degrees into the reference sector */
  sector = (symI > symQ) ? (absI > absQ) + 2 : (absI > absQ);
  cosRot = DECv17_COS_ROT_ANGLE[sector];
  sinRot = DECv17_SIN_ROT_ANGLE[sector];
  rotI = (short)(((symI * cosRot) >> 15) - ((symQ * sinRot) >> 15));
  rotQ = (short)(((symQ * cosRot) >> 15) + ((symI * sinRot) >> 15));

  /* coarse cell (`base`: first of four table entries) from the grid lines of the rotated coordinates;
   * tieCase 1 / 2 mark the two cells that have two candidate points */
  if (rotI > 0x16a1)
  {
    base = 0xc;
    if (rotI > 0x2d41)
    {
      if (rotQ > 0x2d41)
      {
        if (rotI > 0x3892)
        {
          if (rotQ > 0x3891)
            tieCase = 1;
          else
            base = 0x1a;
        }
        else
        {
          if (rotQ > 0x3892)
            base = 0xe;
          else
            tieCase = 2;
        }
      }
      else
      {
        base = (rotQ < 0x16a2) * 4 + 0x18;
      }
    }
    else if (rotQ <= 0x2d40)
    {
      base = (rotQ > 0x16a0) ? 0x10 : 0x14;
    }
  }
  else if (rotQ <= 0x2d40)
  {
    base = (rotQ <= 0x16a0) * 4 + 4;
  }

  /* nearest point: the two special cells compare two fixed candidates, the others scan their four entries */
  if (tieCase == 1)
  {
    short e1 = (short)(sq16(0x2799, rotI, 13) + sq16((unsigned short)DECv17_ANA_QMAP128[14], rotQ, 13));
    short e2 = (short)(sq16(0x3e3a, rotI, 13) + sq16((unsigned short)DECv17_ANA_QMAP128[26], rotQ, 13));
    bestIdx = (e2 >= e1) ? 0x1a : 0xe;
  }
  else if (tieCase == 2)
  {
    short e1 = (short)(sq16(0x2799, rotI, 13) + sq16((unsigned short)DECv17_ANA_QMAP128[15], rotQ, 13));
    short e2 = (short)(sq16(0x32e9, rotI, 13) + sq16((unsigned short)DECv17_ANA_QMAP128[24], rotQ, 13));
    bestIdx = (e2 <= e1) * 9 + 0xf;
  }
  else
  {
    for (i = base; i < base + 4; i++)
    {
      short d = (short)(sq16(rotI, (unsigned short)DECv17_ANA_IMAP128[i], 13) + sq16(rotQ, (unsigned short)DECv17_ANA_QMAP128[i], 13));
      if (d < bestDist)
      {
        bestDist = d;
        bestIdx = i;
      }
    }
  }

  /* table index = point within the sector + sector * 32 */
  tabIdx = (short)(bestIdx + (sector << 5));
  if ((unsigned short)tabIdx > 0x7f)
  {
    if (dsplibs_debug_level > 1)
    {
      dsplibs_debug_printf("******************Index fault %d, changed to 0\n", tabIdx);
      h = (FaxDec17*)f->cfg.hist;
    }
    tabIdx = 0;
  }

  *mag = DECv17_MAG14400[tabIdx];
  *angle = DECv17_ANGL14400[tabIdx];
  FAXVTB_decoder(&h->vtb, (short)symI, (short)symQ, &out);
  return (unsigned short)out;
}

/**
 * @brief Start-of-training detector for the 4800 bit/s training segment.
 * @param f      FSE block; `eqEnable` is cleared on every call, `pllEnable` is set once the short-training count passes 0x96
 * @param angle  in: phase of the received symbol; out: DECv17_ANGL4800[angleIdx] (the point the detector expects)
 * @param mag    in: magnitude of the received symbol; out: 0x3299
 * @return 3 after a large phase step, 2 after a small one, 0 when the detector fires
 * @note Each call smooths the magnitudes of large phase steps (accA) and small ones (accB) with a 1/32 leak and keeps the last three equalized pairs.  When
 *   the summed squared distance of the pair history exceeds 4/3 of the smoothed energy and more than 0xc8 symbols have been seen, the history is cleared, the
 *   equalizer update is forced, `cfg.cb` becomes FSE_decision_eqtrn, `cfg.mu[0]` = 0x147b and the training register is seeded with 0xbb3754.
 */
int FAX_FSE_decision_AB(FaxFse* f, short* angle, short* mag)
{
  FaxDec17* h = (FaxDec17*)f->cfg.hist;
  int accA, accB, retCode, angleIdx, magIn, distThr, distSum, flag;
  short angleStep, symI, symQ, oldI0, oldQ0, oldI1, oldQ1, cnt;
  unsigned short cur, prev;
  short idx;

  /* count the call, track the phase step and smooth the magnitude of its class */
  f->eqEnable = 0;
  COMPILER_BARRIER();
  cnt = (short)(h->cnt + 1);
  h->cnt = cnt;
  h->ctr = (unsigned short)(h->ctr + 1u);
  prev = h->prevAngle;
  accA = h->accA;
  cur = (unsigned short)*angle;
  accB = h->accB;
  angleStep = (short)(cur - prev);
  h->prevAngle = cur;
  if (angleStep < 0)
    angleStep = (short)(angleStep - 0x8000);
  magIn = *mag;
  if (angleStep > 0x4000)
  {
    accA = (short)((((accA * 32 - accA)) >> 5) + (magIn >> 5));
    retCode = 3;
    angleIdx = 1;
  }
  else
  {
    accB = (short)((((accB * 32 - accB)) >> 5) + (magIn >> 5));
    retCode = 2;
    angleIdx = 2;
  }

  /* with short training the carrier PLL is switched on after 0x96 symbols */
  flag = h->flag;
  if (flag != 0 && cnt > 0x96)
    f->pllEnable = 1;
  COMPILER_BARRIER();

  /* movement of the equalized symbol against its history versus the smoothed energy */
  idx = f->idx;
  symI = f->outI[idx];
  symQ = f->outQ[idx];
  oldI0 = h->abI0;
  oldQ0 = h->abQ0;
  oldI1 = h->abI1;
  oldQ1 = h->abQ1;
  distSum = (short)((WADD((int)(short)(oldI1 - symI) * (short)(oldI1 - symI), (int)(short)(oldQ1 - symQ) * (short)(oldQ1 - symQ)) >> 15) +
                    (WADD((int)(short)(h->abI2 - oldI0) * (short)(h->abI2 - oldI0), (int)(short)(h->abQ2 - oldQ0) * (short)(h->abQ2 - oldQ0)) >> 15));
  distThr = (((short)(WADD(accA * accA, accB * accB) >> 15)) * 0x5555) >> 14;

  if (distSum > distThr && cnt > 0xc8)
  {
    /* the signal stopped moving: reset the detector and start the equalizer training */
    h->abQ2 = 0;
    accA = 0;
    h->abI2 = 0;
    accB = 0;
    h->abQ1 = 0;
    h->abI1 = 0;
    h->abQ0 = 0;
    h->abI0 = 0;
    h->prevAngle = 0;
    f->eqEnable = 1;
    f->eqForce = 1;
    h->cnt = 0;
    f->cfg.cb = FSE_decision_eqtrn;
    h->w50 = 1;
    angleIdx = 3;
    retCode = 0;
    f->cfg.mu[0] = 0x147b;
    h->reg = 0xbb3754;
  }
  else
  {
    /* shift the pair history */
    h->abI1 = oldI0;
    h->abQ1 = oldQ0;
    h->abQ0 = symQ;
    h->abI0 = symI;
    h->abQ2 = oldQ1;
    h->abI2 = oldI1;
  }
  COMPILER_BARRIER();

  *angle = DECv17_ANGL4800[angleIdx];
  *mag = 0x3299;
  h->accA = (short)accA;
  h->accB = (short)accB;
  return retCode;
}

/**
 * @brief Bridge segment decision: nearest of the four 4800 bit/s points.
 * @param f      FSE block; `cfg.cb` is replaced by FSEv17_decision[hist->sel] after 0x3f symbols
 * @param angle  receives DECv17_ANGL4800[nearest point]
 * @param mag    receives 0x3299
 * @return 0 (the bridge produces no data bits)
 * @warning The distance scales are not symmetric: the I term is shifted right by 15, the Q term by 16.
 * @warning `hist->sel` is not range checked; a value outside 0 .. 3 installs a NULL callback.
 */
int FSE_Bridge_det(FaxFse* f, short* angle, short* mag)
{
  FaxDec17* h = (FaxDec17*)f->cfg.hist;
  short idx, symI, symQ, cnt = h->cnt;
  int best = 0, i;
  short bestDist = 0x7fff;

  /* after 0x3f symbols hand over to the rate's trellis decision */
  if (cnt > 0x3e)
  {
    short sel = h->sel;
    cnt = 0;
    h->reg = 0;
    f->cfg.cb = ((unsigned short)sel < 4) ? FSEv17_decision[sel] : NULL;
  }
  h->cnt = (short)(cnt + 1);
  bump_ctr(h);

  idx = f->idx;
  symI = f->outI[idx];
  symQ = f->outQ[idx];

  /* nearest of the four points */
  for (i = 0; i <= 3; i++)
  {
    short d = (short)(sq16(symI, (unsigned short)DECv17_IMAP4[i], 15) + sq16(symQ, (unsigned short)DECv17_QMAP4[i], 16));
    if (d < bestDist)
    {
      bestDist = d;
      best = i;
    }
  }

  *angle = DECv17_ANGL4800[best];
  *mag = 0x3299;
  return 0;
}

/**
 * @brief Equalizer training sequence decision: emit the next point of the training sequence.
 * @param f      FSE block; at the end of the sequence `cfg.cb` is replaced, `muIdx` becomes 1 and `cfg.mu[1]` = 0x51f
 * @param angle  receives DECv17_ANGL4800[point]
 * @param mag    receives 0x3299
 * @return the training point (0 .. 3, DECv17_MAP_TRN of the register feedback bit pair)
 * @note The sequence length is 0x25 symbols with short training (`hist->flag` != 0) and 0xb9f without.  The register advances two bits per symbol; the new
 *   bit pair is the xor of register bits 16-17, 21-22 and the constant 3.  At the end the register is cleared, `hist->w50` is reset, and the next callback is
 *   FSEv17_decision[hist->sel] (short training) or FSE_Bridge_det.
 * @warning `hist->sel` is not range checked; a value outside 0 .. 3 installs a NULL callback.
 */
int FSE_decision_eqtrn(FaxFse* f, short* angle, short* mag)
{
  FaxDec17* h = (FaxDec17*)f->cfg.hist;
  int flag = h->flag;
  int limit = 0x25 + (flag == 0 ? 0xb7a : 0);
  int reg = h->reg;
  int lfsrBits = (((reg >> 16) ^ 3) ^ (int)((unsigned int)reg >> 21)) & 3;
  int nextReg = (int)(((unsigned int)reg << 2) | (unsigned int)lfsrBits);
  short cnt = (short)(h->cnt + 1), sel = h->sel;
  int point;

  h->cnt = cnt;
  bump_ctr(h);

  point = DECv17_MAP_TRN[lfsrBits];

  if (cnt >= limit)
  {
    /* end of the sequence: switch the equalizer step size and install the next decision */
    f->muIdx = 1;
    if (flag != 0)
      f->cfg.cb = ((unsigned short)sel < 4) ? FSEv17_decision[sel] : NULL;
    else
      f->cfg.cb = FSE_Bridge_det;
    f->cfg.mu[1] = 0x51f;
    h->cnt = 0;
    h->reg = 0;
    *angle = DECv17_ANGL4800[point];
    h->w50 = 0;
    *mag = 0x3299;
    return (unsigned short)point;
  }

  h->reg = nextReg;
  *angle = DECv17_ANGL4800[point];
  *mag = 0x3299;
  return (unsigned short)point;
}
