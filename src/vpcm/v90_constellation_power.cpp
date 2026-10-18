/**
 * @file v90_constellation_power.cpp
 * @brief V90ConstellationDesigner: trimming the constellation set to the power limit and to the nearest bit count K.
 *
 * Two functions of the designer, both called by V90ConstellationDesigner::process() / constellationDesign() (v90_constellation_dmin.cpp):
 * - adjustConstellationsPower() removes points from the six per-phase ucode lists until the transmit power (V90ConstellationPower::getPower(), measurement
 *   point "codec output") is no longer above the average-power limit for the allowed power index.
 * - adjustConstellationsToNewK() optimises the redundancy of the set: when the real number of bits exceeds the integer K by more than `upRoundK` it adds
 *   points (up to K + 1), otherwise it removes points while K does not fall below its current value, and prints a K / power report.
 *
 * K is the number of bits of the constellation set: trunc(log2(product of cnt_[0..5]) + 1e-6), 0 when the product is 0.  It is stored as
 * `mapParams_->kBits_` = K - `sr_` + 6.
 *
 * Arithmetic: `long double` for the values the FPU keeps on its stack (the count product, log10 of it, the quotient, K as a float sum, the square root of
 * the power, the print fractions); `float` where a value is stored and reloaded (the log10(2) divisor, log10(product) of the "real K" flavours, the power
 * values, the limit); 64-bit and 32-bit truncation through trunc64lo() / dsp_trunc_i32().  The count product is accumulated as
 * ((((c0 * c1) * c2) * c3) * c4) * c5 in extended precision from the zero-extended counts.  K is computed in three flavours (kInt(), realKBefore(),
 * realKAfter()) that differ in where the intermediate roundings happen.
 *
 * Behaviours to know about:
 * - The removal / addition loops re-read `mapParams_` after every debug print (a print may change it); `mp` holds the pointer across the code in between.
 * - adjustConstellationsPower picks the phase with the HIGHEST first ucode (ties: the larger count, then the first phase); the add loop of
 *   adjustConstellationsToNewK the LOWEST (same tie rule); its remove loop the highest.
 * - The dual-phase ucode scan compares A[u] against A[u0] + dMinAlt_ and B[u0] + dMinAlt_ (B only at the START index); the single-phase scan runs on a
 *   SIGNED byte counter (overflow check `(int8_t)u < 0`), the dual one up to 0x71; a table overflow also triggers at a list count above 0x80.
 * - The G.711 code of an added point is computed before the overflow test.
 */

#include "v90_constellation_designer.h"
#include "core/dsp_math.h"
#include "core/dsp_host.h"
#include "v34/v34_api.h"
#include "pcm_leaf_classes.h"

extern "C"
{
}

typedef long double LD;

#include "core/dsp_floatbits.h"

/* kEps6 (the +1e-6 of the integer K) is defined in v90_mapping_prod.h */
#define kEps9 (bitsFloat(0x3089705fu)) /* 9.99999972e-10: the +1e-9 of the real K */

/* ---- helpers ---- */

#include "core/dsp_x87_cmp.h"

#include "core/dsp_intmath.h"

#include "core/dsp_x87_conv.h"

#include "vpcm_split.h"
#include "vpcm_split_value.h"

#include "v90_mapping_rows.h"

#include "v90_mapping_prod.h"

/** Flavour 1: the integer K = trunc(log10(prod) / (float)log10(2) + 1e-6f), 0 for prod == 0 (log10 stays extended). */
static inline uint32_t kInt(const V90MappingParams* mp)
{
  LD p = prodCnt(mp);
  if (p == 0.0L)
    return 0u;
  LD lg = dsp_log10(p);
  float den = lg2F();
  return trunc64lo(lg / (LD)den + (LD)kEps6);
}

/** Flavour 2 ("before"): a float, ((float)log10(prod)) / (float)log10(2) + 1e-9f stored to a float; 0.0f for prod == 0. */
static inline float realKBefore(const V90MappingParams* mp)
{
  float slot;
  storeF(&slot, 0.0L);
  LD p = prodCnt(mp);
  if (p != 0.0L)
  {
    float lgs;
    float lgf = storeF(&lgs, dsp_log10(p));
    float den = lg2F();
    storeF(&slot, (LD)lgf / (LD)den + (LD)kEps9);
  }
  return loadF(&slot);
}

/** Flavour 3 ("after"): the same quotient, but the +1e-9f sum stays extended (never stored); used by the "real K after optimization" print. */
static inline LD realKAfter(const V90MappingParams* mp)
{
  LD p = prodCnt(mp);
  if (p == 0.0L)
    return 0.0L;
  float lgs;
  float lgf = storeF(&lgs, dsp_log10(p));
  float den = lg2F();
  return (LD)lgf / (LD)den + (LD)kEps9;
}

/** The G.711 code of a distance: u-law = ~linear2ulaw(|v|), A-law = linear2alaw(|v|) ^ 0xd5 (A-law when `useAlaw_` is non-zero). */
static inline uint8_t codeOf(const V90ConstellationDesigner* t, int v)
{
  int a = absI32(v);
  if (t->useAlaw_ == 0)
    return (uint8_t)~(uint8_t)linear2ulaw(a);
  return (uint8_t)((uint8_t)linear2alaw(a) ^ 0xd5u);
}

/** The phase the power / remove loops pick: the one with the highest first ucode (listA_[ph][0]); ties: the strictly larger count, then the lower phase. */
static inline unsigned pickMax(V90MappingParams* mp)
{
  unsigned best = 0;
  unsigned bv = LA(mp, 0, 0);
  unsigned bc = mp->cnt_[0];
  for (unsigned ph = 1; ph <= 5; ph++)
  {
    unsigned v = LA(mp, ph, 0);
    if (v > bv)
    {
      bc = mp->cnt_[ph];
      bv = v;
      best = ph;
    }
    else if (v == bv)
    {
      unsigned c = mp->cnt_[ph];
      if (c > bc)
      {
        bc = c;
        best = ph;
      }
    }
  }
  return best;
}

/** The phase the add loop of adjustConstellationsToNewK picks: the lowest first ucode (strictly lower takes); ties: the strictly larger count. */
static inline unsigned pickMin(V90MappingParams* mp)
{
  unsigned best = 0;
  unsigned bv = LA(mp, 0, 0);
  unsigned bc = mp->cnt_[0];
  for (unsigned ph = 1; ph <= 5; ph++)
  {
    unsigned v = LA(mp, ph, 0);
    if (v < bv)
    {
      bc = mp->cnt_[ph];
      bv = v;
      best = ph;
    }
    else if (v == bv)
    {
      unsigned c = mp->cnt_[ph];
      if (c > bc)
      {
        bc = c;
        best = ph;
      }
    }
  }
  return best;
}

/** One power sample: getPower() at the codec output for the current law, rounded to float (the argument of getPowerIndexForPower()). */
static inline float powerOf(V90ConstellationDesigner* t, V90MappingParams* mp)
{
  float slot;
  return storeF(&slot, (LD)t->power_->getPower(mp, V90_TX_POINT_CODEC_OUTPUT, (PcmType)t->useAlaw_));
}

/** Print arguments for "sqrt(power)": sqrt of `P` in extended precision, three decimals, sign from `P`. */
static inline Split sqrtSplit(float P) { return splitValue(dsp_sqrt((LD)P), 1000.0L, (LD)P); }

/** Print arguments for "index %d, power in dBm0 = %c%d.%01d": v = (idx + 1) * -0.5, one decimal. */
static inline Split dbSplit(unsigned idx)
{
  LD v = (LD)(unsigned)(idx + 1u) * (LD)bitsFloat(0xbf000000u);
  return splitValue(v, 10.0L, v);
}

/* ---- adjustConstellationsPower ---- */

/**
 * Removes points from the six ucode lists until the transmit power is within the limit.
 * 1. Computes K and `mapParams_->kBits_`, the power P and its index, and the limit index lim = min(`powerLimit_`, 22) with its average-power limit; done if
 *    P is not above the limit.
 * 2. Otherwise repeats rounds: a round removes n points, each from the phase with the highest first ucode (the first entry of that list is dropped); n is
 *    20 when the index distance lim - idx exceeds 7, 1 when it is below 4, else 10, and always 1 when `flag8_` > 13.  If a list would become empty the
 *    removal is undone and the loop ends.  After each round K is recomputed; the loop continues while K > 20 and the power is still above the limit.
 * 3. If it stopped because K <= 20 and a point was removed, the last removed point is put back at the front of its list.
 * 4. Prints the number of removed points and the final power and index.
 * Reads `powerLimit_`, `flag8_`, `power_` and `mapParams_`; writes the lists, `cnt_[]` and `kBits_`.
 */
void V90ConstellationDesigner::adjustConstellationsPower()
{
  V90MappingParams* mp = mapParams_;
  int cont = 1;
  int removed = 0;
  uint8_t svB = 0, svA = 0;
  unsigned lastP = 0;
  unsigned lim, diff;
  float Plim;

  /* initial K, power and index */
  mp->kBits_ = kInt(mp) - mp->sr_ + 6u;
  float P = powerOf(this, mapParams_);
  unsigned idx = power_->getPowerIndexForPower(P);
  edprintf("--------------------------------------\r\n");
  edprintf("V90ConstellationDesigner:\r\n");
  {
    Split s = sqrtSplit(P);
    edprintf("V90ConstellationDesigner: initial sqrt(power)  = %c%d.%03d\r\n", s.sign, s.ip, s.fp);
    Split d = dbSplit(idx);
    edprintf("V90ConstellationDesigner: initial index %d, power in dBm0  = %c%d.%01d\r\n", (int)idx, d.sign, d.ip, d.fp);
  }

  /* the power limit for the allowed index; nothing to do when the power is not above it */
  lim = (powerLimit_ > 0x15) ? 0x16u : (unsigned)powerLimit_;
  diff = lim - idx;
  {
    float slot;
    Plim = storeF(&slot, (LD)V90ConstellationPower::averagePowerLimits[lim]);
  }
  {
    float pw = powerOf(this, mapParams_);
    if (!jmpA((LD)pw, (LD)Plim))
      goto fin_a; /* fcomp; sahf; jbe: not above (or unordered) */
  }

top:
  /* one round: remove n points */
  {
    unsigned n = (diff > 7) ? 20u : ((diff < 4) ? 1u : 10u);
    if (flag8_ > 13)
      n = 1;
    mp = mapParams_;
    if (n == 0)
      goto recompute;
    do
    {
      unsigned p = pickMax(mp);
      lastP = p;
      unsigned c = --mp->cnt_[p];
      if (c == 0)
      {
        if (dsplibs_debug_level > 1)
        {
          dsplibs_debug_printf("V90ConnectionDesigner: adjustConstellationPower - BUG !!! reached constellation length 0!!!\n");
          mp = mapParams_;
        }
        cont = 0;
        mp->cnt_[p]++;
        goto recompute;
      }
      svB = LB(mp, p, 0);
      svA = LA(mp, p, 0);
      {
        unsigned i = 0;
        do
        {
          LB(mp, p, i) = LB(mp, p, i + 1);
          LA(mp, p, i) = LA(mp, p, i + 1);
          i++;
        } while (mp->cnt_[p] > i);
      }
      removed++;
      n--;
    } while (n != 0);
  }

recompute:
  /* new K; keep going only while K > 20 and the power is still above the limit */
  {
    uint32_t K = kInt(mp) - mp->sr_ + 6u;
    mp->kBits_ = K;
    if (K > 0x14)
    {
      float P2 = powerOf(this, mp);
      unsigned idx2 = power_->getPowerIndexForPower(P2);
      diff = lim - idx2;
      if (jmpA((LD)P2, (LD)Plim) && (cont & 0xffff) != 0)
        goto top;
      goto fin_a;
    }
    if (dsplibs_debug_level > 1)
      dsplibs_debug_printf("V90ConstellationDesigner::adjustConstellationsPower (pParams->d=%d)<V90_MIN_D_IN_DATA\n", (int)K);

    /* K fell to 20 or below: undo the last removal */
    if (removed == 0)
      goto fin_a;
    if (dsplibs_debug_level > 1)
      dsplibs_debug_printf("V90ConstellationDesigner::adjustConstellationsPower restoring last removed point\n");

    /* put the last removed point back at the front of its list */
    mp = mapParams_;
    {
      unsigned c = mp->cnt_[lastP] + 1u;
      mp->cnt_[lastP] = c;
      if (c != 0)
      {
        for (unsigned k = c; k != 0; k--)
        {
          LB(mp, lastP, k) = LB(mp, lastP, k - 1);
          LA(mp, lastP, k) = LA(mp, lastP, k - 1);
        }
      }
    }
    mp = mapParams_;
    LB(mp, lastP, 0) = svB;
    LA(mp, lastP, 0) = svA;
    removed--;
    mp->kBits_ = kInt(mp) - mp->sr_ + 6u;
    goto fin_b;
  }

fin_a:
  mp = mapParams_;
fin_b:
  /* final report */
  {
    float P2 = powerOf(this, mp);
    unsigned idx2 = power_->getPowerIndexForPower(P2);
    Split s = sqrtSplit(P2);
    edprintf("V90ConstellationDesigner: nof points removed %d, new sqrt(power)  = %c%d.%03d\r\n", removed, s.sign, s.ip, s.fp);
    Split d = dbSplit(idx2);
    edprintf("V90ConstellationDesigner: final index %d, final power in dBm0  = %c%d.%01d\r\n", (int)idx2, d.sign, d.ip, d.fp);
    edprintf("--------------------------------------\r\n");
  }
}

/* ---- adjustConstellationsToNewK ---- */

/**
 * Optimises the redundancy of the constellation set against the real (fractional) bit count.
 * 1. K0 = the integer K, `kBits_` is updated, Kreal = the real K as a float; the power index is evaluated but its result is not used.
 * 2. If Kreal - K0 exceeds `params_->upRoundK`, points are ADDED until K reaches K0 + 1 (determineDminForRrn() is called for that target): each step takes
 *    the phase with the lowest first ucode and prepends a new first ucode found by scanning the distance table upward from the old first ucode until the
 *    distance exceeds the old first distance plus dMin_ (dMinAlt_ for a dual phase, which must also exceed the B threshold).  If the scan runs off the table
 *    or a list exceeds 0x80 entries, every addition is undone (the leading entries before each phase's saved first ucode are dropped) and step 3 runs.
 * 3. If nothing was added (b7 is set): K1 = the current K, determineDminForRrn(K1); points are REMOVED from the phase with the highest first ucode while K
 *    stays >= K1, and the last removed point is put back (unless that list has only one entry left).
 * 4. Prints the real K after optimisation, recomputes `kBits_`, and prints the new power and index.
 * @param A      per-phase, per-ucode distances, 6 x 128
 * @param B      second distances used by dual phases, 6 x 128
 * @param dual   per-phase flag: the phase uses both A and B
 * @param valid  per-phase, per-ucode allowed flags (unused)
 * @note The `curK < K1` branch of step 3 cannot be taken: without additions K0 == K1, and after an overflow undo curK >= K1.
 * @warning The undo loop of step 2 searches each list for the saved first ucode with no bound; it terminates because the saved ucodes are the old first
 *          entries and stay in the lists.  The shifts read one entry past the end of a list (stale data, inside the array).
 */
void V90ConstellationDesigner::adjustConstellationsToNewK(short (*A)[128], short (*B)[128], short* dual, unsigned char (*valid)[128])
{
  const int16_t* a = &A[0][0];
  const int16_t* b = &B[0][0];
  (void)valid; /* never read */
  V90MappingParams* mp = mapParams_;
  int removedCnt = 0;
  bool b7 = true;
  uint8_t addc = 0;
  uint8_t saved[6];
  uint8_t svA2 = 0, svB2 = 0;
  unsigned lastP = 0;

  /* K, real K and the starting power */
  uint32_t K0 = kInt(mp);
  uint32_t curK = kInt(mp);
  mp->kBits_ = K0 - mp->sr_ + 6u;
  float Kreal = realKBefore(mp);
  {
    float P = powerOf(this, mp);
    power_->getPowerIndexForPower(P); /* the result is not used */
  }

  edprintf("---------------------------------------------------------------\r\n");
  edprintf("V90ConnectionDesigner: redundant points optimization :\r\n");
  {
    Split s = splitValue((LD)Kreal, 100000.0L, (LD)Kreal);
    edprintf("V90ConnectionDesigner: real K before optimization  = %c%d.%05d\r\n", s.sign, s.ip, s.fp);
  }

  if (jmpA((LD)Kreal - (LD)K0, (LD)params_->upRoundK))
  {
    /* add points until K reaches K0 + 1 */
    uint32_t target = K0 + 1u;
    determineDminForRrn(target);
    b7 = false;
    mp = mapParams_;
    for (unsigned ph = 0; ph < 6; ph++)
      saved[ph] = LA(mp, ph, 0);
    if (!(target <= curK))
    {
      do
      {
        unsigned p = pickMin(mp);
        mp->cnt_[p]++;
        unsigned u0 = LA(mp, p, 0);
        unsigned idx0 = p * 0x80u + u0;
        uint8_t u = (uint8_t)u0;

        /* scan upward for the first ucode far enough from the current first one */
        if (dual[p] == 0)
        {
          /* single phase: scan while the signed byte stays >= 0 */
          if (!((int8_t)u < 0))
          {
            int thr = (int)a[idx0] + (int)(int16_t)dMin_;
            const int16_t* pa = &a[idx0];
            for (;;)
            {
              int cv = *pa;
              pa++;
              if (cv >= thr)
                break;
              u++;
              if ((int8_t)u < 0)
                break;
            }
          }
        }
        else if (u <= 0x71)
        {
          int dA = (int)(int16_t)dMinAlt_;
          int thr1 = (int)a[idx0] + dA;
          const int16_t* pa = &a[idx0];
          for (;;)
          {
            int cv = *pa;
            bool adv = true;
            if (!(cv < thr1))
            {
              int thr2 = (int)b[idx0] + dA;
              if (cv >= thr2)
                adv = false;
            }
            if (!adv)
              break;
            u++;
            pa++;
            if (!(u <= 0x71))
              break;
          }
        }

        uint8_t code = codeOf(this, a[(unsigned)(uint8_t)p * 0x80u + u]);
        if ((int8_t)u < 0 || mapParams_->cnt_[p] > 0x80u)
        {
          /* table overflow: undo every addition (drop the points in front of each phase's saved first ucode) */
          if (dsplibs_debug_level > 1)
            dsplibs_debug_printf("V90ConnectionDesigner::adjustConstellationsToNewK !!! reached Max constellation length!!!\r\n");
          b7 = true;
          mp = mapParams_;
          mp->cnt_[p]--;
          for (unsigned ph = 0; ph < 6; ph++)
          {
            uint8_t dl = 0;
            while (LA(mp, ph, dl) != saved[ph])
              dl++;
            if (dl != 0)
            {
              do
              {
                unsigned c = mp->cnt_[ph];
                uint8_t bl = 0;
                if (bl < c)
                {
                  do
                  {
                    unsigned o = bl;
                    LA(mp, ph, o) = LA(mp, ph, o + 1);
                    bl++;
                    LB(mp, ph, o) = LB(mp, ph, o + 1);
                  } while (bl < c);
                }
                mp->cnt_[ph]--;
                dl--;
              } while (dl != 0);
            }
          }
          goto afterAdd;
        }

        /* prepend the new point to the phase's lists */
        mp = mapParams_;
        {
          unsigned c = mp->cnt_[p];
          for (unsigned k = c; k != 0; k--)
          {
            LB(mp, p, k) = LB(mp, p, k - 1);
            mp = mapParams_;
            LA(mp, p, k) = LA(mp, p, k - 1);
            mp = mapParams_;
          }
        }
        mp = mapParams_;
        LB(mp, p, 0) = code;
        mp = mapParams_;
        LA(mp, p, 0) = u;
        mp = mapParams_;

        /* K after the addition */
        {
          uint32_t K2 = kInt(mp);
          addc++;
          curK = K2;
          mp->kBits_ = K2 - mp->sr_ + 6u;
        }
      } while (target > curK);
    }
    edprintf("V90ConnectionDesigner: Optimizing new K: nof points Add %d \r\n", (int)addc);
  }

afterAdd:
  mp = mapParams_;
  if (b7)
  {
    /* remove points again while K stays >= K1, then put the last one back */
    uint32_t K1 = kInt(mp);
    determineDminForRrn(K1);
    if (curK < K1)
    {
      mp = mapParams_; /* cannot be taken (see the function comment) */
      goto afterLoop;
    }
    mp = mapParams_;

    /* drop the first point of the highest-first-ucode phase until K falls below K1 */
    for (;;)
    {
      unsigned p = pickMax(mp);
      lastP = p;
      unsigned c = --mp->cnt_[p];
      if (c == 0)
      {
        if (dsplibs_debug_level > 1)
        {
          dsplibs_debug_printf("V90ConnectionDesigner::adjustConstellationsToNewK - BUG !!! reached constellation length 0!!!\r\n");
          mp = mapParams_;
        }
        mp->cnt_[p]++;
        break;
      }
      svB2 = LB(mp, p, 0);
      svA2 = LA(mp, p, 0);
      {
        unsigned i = 0;
        do
        {
          LB(mp, p, i) = LB(mp, p, i + 1);
          mp = mapParams_;
          LA(mp, p, i) = LA(mp, p, i + 1);
          mp = mapParams_;
          i++;
        } while (mp->cnt_[p] > i);
      }
      uint32_t K3 = kInt(mp);
      removedCnt++;
      mp->kBits_ = K3 - mp->sr_ + 6u;
      if (!(K3 >= K1))
        break;
    }

  afterLoop:
    /* restore the last removed point so that K stays >= K1 (skipped when the list is down to one entry) */
    {
      unsigned c = mp->cnt_[lastP];
      if (c != 1)
      {
        if (c != 0)
        {
          for (unsigned k = c; k != 0; k--)
          {
            LB(mp, lastP, k) = LB(mp, lastP, k - 1);
            mp = mapParams_;
            LA(mp, lastP, k) = LA(mp, lastP, k - 1);
            mp = mapParams_;
          }
          mp = mapParams_;
        }
        mp->cnt_[lastP]++;
        LB(mp, lastP, 0) = svB2;
        mp = mapParams_;
        LA(mp, lastP, 0) = svA2;
        removedCnt--;
      }
      edprintf("V90ConnectionDesigner: Optimizing new K: nof points removed %d \r\n", removedCnt);
      mp = mapParams_;
    }
  }

  /* final report */
  {
    LD R = realKAfter(mp);
    Split s = splitValue(R, 100000.0L, R);
    edprintf("V90ConnectionDesigner: real K after optimization  = %c%d.%05d\r\n", s.sign, s.ip, s.fp);
  }
  mp = mapParams_;
  mp->kBits_ = kInt(mp) - mp->sr_ + 6u;
  {
    float P3 = powerOf(this, mapParams_);
    unsigned idx3 = power_->getPowerIndexForPower(P3);
    Split s = sqrtSplit(P3);
    edprintf("V90ConnectionDesigner: Optimizing new K: new sqrt(power)  = %c%d.%03d\r\n", s.sign, s.ip, s.fp);
    Split d = dbSplit(idx3);
    edprintf("V90ConstellationDesigner: final index %d, final power in dBm0  = %c%d.%01d\r\n", (int)idx3, d.sign, d.ip, d.fp);
    edprintf("---------------------------------------------------------------\r\n");
  }
}
