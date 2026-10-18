/**
 * @file v90_trn2_designer.cpp
 * @brief V90TRN2Designer: the TRN2 constellation design (V90TRN2Design), its setters and the dummy-constellation fallback.
 *
 * Implements the class declared in v90_trn2_designer.h (what the design computes is described there).  This file owns:
 * - V90TRN2Design(): the whole design, in the order: counts, spectral constants and data rate word; the maximum ucode from the power limit of the TX index;
 *   the per-phase start ucodes; the per-phase dMin or forced design; the second list in the target codec; the power / index prints; the table report;
 * - setNofUcodesInTrn2() and the two helpers without a caller in the library, maxK() and setTrn2DummyConstel();
 * - static helpers: kInt() (the integer K), g711() (G.711 code of an amplitude), spectralBody() (copies the shaper constants into the mapping block),
 *   dummyRows() (the fallback constellation), sqrtSplit() / dbSplit() (print arguments).
 *
 * Layout of the mapping block lists: each phase p (0 .. 5) has N = params->nofUcodes ucodes in mp->listA_[p] (ascending slots counted from the top: the dMin
 * design fills slot j = N - 1 down to 0, the forced design slot 0 first and then up), and the second list listB_ holds the G.711 codes in the target codec.
 * The rows are addressed through the flat LA() / LB() accessors (v90_mapping_rows.h), so an index of 128 and above runs into the next row.  D is
 * kBits_ = K - sr + 6 with K = trunc(log10(product of the six counts) / (float)log10(2) + 1e-6f) (v90_mapping_prod.h).
 *
 * Callees from other files: V90ConstellationPower::getPower() / getPowerIndexForPower() / averagePowerLimits, the G.711 coders linear2alaw() /
 * linear2ulaw(), edprintf(), dsplibs_debug_printf().
 *
 * Numeric conventions.  `long double` (LD) holds every value that is kept in extended precision: the six-count product, log10 of it, the dMin quotient, the
 * forced design's reciprocal and distance update, the square roots of the power limit and of the power, the dBm0 value.  Floats are kept as floats where a
 * value is rounded to a float and read back: the float log10(2), and the power returned by getPower() (through the storeF() helper).  Conversions to
 * integers truncate toward zero (trunc16 / trunc64lo / dsp_trunc_i32; NaN and out-of-range give the "integer indefinite" value).
 *
 * Behaviour that a tidier version would change (kept as is):
 * - this->params_ is re-read exactly where it was re-read before (after each of the first two prints, before the phase loop, after the dMin print, after every
 *   accepted ucode / restart of the forced search, in every pass of the conversion loop, before the header prints of the table dump and after each of its
 *   rows), so a print hook that replaces the parameter block takes effect from the next read;
 * - dsplibs_debug_level is cached in a local in the table dump and re-read after EVERY print there; the comparison `> 1` is unsigned;
 * - the counters have fixed widths: `cl` (a ucode) is a byte (wraps), the dMin design's slot j and the forced design's count / round are 16-bit, the
 *   thresholds are 16-bit compares of table words;
 * - the tables A / B / valid and the lists are flat (an index above 127 runs into the next row, index -1 into the previous one);
 * - the cur[] bytes are copied into a local array first and lowered there (the caller's bytes are never changed);
 * - when pcmA == pcm the second list is the first with every entry that is above its own G.711 code replaced by the code (min(code, ucode)); otherwise it is
 *   the plain code;
 * - averagePowerLimits is read at index powerIdx + 4 (the table has 35 entries, so powerIdx up to 30 stays inside it).
 *
 * @warning V90TRN2Design() does not validate its indices: powerIdx > 30 reads past averagePowerLimits[]; the start-ucode lowering loop does not end when every
 *          code of a phase exceeds the maximum; N > 128 (params->nofUcodes) writes past a list row.
 */

#include "v90_trn2_designer.h"
#include "core/dsp_math.h"
#include "core/dsp_host.h"
#include "v34/v34_api.h"
#include "pcm_leaf_classes.h"

extern "C"
{
}

typedef long double LD;

#include "core/dsp_floatbits.h"

/* float constants of the design (kEps6 is in v90_mapping_prod.h) */
#define kFactor (bitsFloat(0x3fd043fau))  /* 1.62707448: sqrt(power limit) -> the maximum level */
#define kHalfF (bitsFloat(0x3f000000u))   /* 0.5 */
#define kDmin (bitsFloat(0x3f59999au))    /* 0.85: dMin = 0.85 * A[cur] / (N - 0.5) */
#define kThou (bitsFloat(0x447a0000u))    /* 1000: three decimals of the power's square root */
#define kNegHalf (bitsFloat(0xbf000000u)) /* -0.5: dBm0 of an index is (index + 1) * -0.5 */
#define kTen (bitsFloat(0x41200000u))     /* 10: one decimal */

/* ------------------------------------------------------------------------------------------------ helpers (static inline) */

#include "core/dsp_intmath.h"

#include "core/dsp_x87_conv.h"

#include "v90_mapping_rows.h"

#include "v90_mapping_prod.h"

/**
 * @brief The integer K = trunc(log10(prod) / (float)log10(2) + 1e-6f), 0 when the product of the six counts is 0.
 *        log10 of the product stays extended, log10(2) is rounded to a float.  Used by maxK() and by the data rate word of V90TRN2Design().
 */
static inline uint32_t kInt(const V90MappingParams* mp)
{
  LD p = prodCnt(mp);
  if (p == 0.0L)
    return 0u;
  LD lg = dsp_log10(p);
  float slot;
  float den = storeF(&slot, dsp_log10(2.0L));
  return trunc64lo(lg / (LD)den + (LD)kEps6);
}

/** @brief G.711 code of |v| in the codec given by `alaw` (A-law: code ^ 0xd5; mu-law: ~code), zero-extended. */
static inline int g711(bool alaw, int v)
{
  int a = absI32(v);
  if (alaw)
    return (int)(uint8_t)(linear2alaw(a) ^ 0xd5);
  return (int)(uint8_t)~linear2ulaw(a);
}

#include "vpcm_split.h"
#include "vpcm_split_value.h"

/** @brief Print arguments of "sqrt(power)": s = sqrt((LD)P) with three decimals, sign from P. */
static inline Split sqrtSplit(float P) { return splitValue(dsp_sqrt((LD)P), (LD)kThou, (LD)P); }

/** @brief Print arguments of "index %d, power in dBm0 = %c%d.%01d": v = (idx + 1) * -0.5 (the index + 1 zero-extended to 64 bit), one decimal. */
static inline Split dbSplit(unsigned idx)
{
  LD v = (LD)(unsigned)(idx + 1u) * (LD)kNegHalf;
  return splitValue(v, (LD)kTen, v);
}

/**
 * @brief Copy the spectral shaper constants from the parameter block into the mapping block (the float words move as integers).
 * @param P        parameter block
 * @param m        mapping block receiving shaperA1_ / A2_ / B1_ / B2_, sr_ and id_
 * @param limit    upper limit for the shaper identifier (id_ = min(id, limit))
 * @param special  2 selects the German-PBX set of constants, anything else the normal set
 * @return the shaper Sr word (= m->sr_), used for the data rate word
 */
static inline uint32_t spectralBody(const V90Parameters* P, V90MappingParams* m, unsigned limit, int special)
{
  uint32_t id, sr;
  if (special == 2)
  {
    id = (uint32_t)P->germanPbxSpectralShaperId;
    sr = (uint32_t)P->germanPbxSpectralShaperSr;
    __builtin_memcpy(&m->shaperA1_, &P->germanPbxSpectralShaperA1, 4);
    __builtin_memcpy(&m->shaperA2_, &P->germanPbxSpectralShaperA2, 4);
    __builtin_memcpy(&m->shaperB1_, &P->germanPbxSpectralShaperB1, 4);
    __builtin_memcpy(&m->shaperB2_, &P->germanPbxSpectralShaperB2, 4);
  }
  else
  {
    id = (uint32_t)P->spectralShaperId;
    sr = (uint32_t)P->spectralShaperSr;
    __builtin_memcpy(&m->shaperA1_, &P->spectralShaperA1, 4);
    __builtin_memcpy(&m->shaperA2_, &P->spectralShaperA2, 4);
    __builtin_memcpy(&m->shaperB1_, &P->spectralShaperB1, 4);
    __builtin_memcpy(&m->shaperB2_, &P->spectralShaperB2, 4);
  }
  m->sr_ = sr;
  m->id_ = id > limit ? limit : id;
  return sr;
}

/**
 * @brief The dummy constellation (also what setTrn2DummyConstel() does and the failure exit of V90TRN2Design() uses): for each of the six phases the first N
 *        entries of both lists are 0x4e, 0x4d, ... (a byte counting down).
 * @param self  designer, whose params_ is re-read after every pair of stores
 * @param mp    mapping block to fill
 * @param P     parameter block pointer held at entry (N = P->nofUcodes for the first test)
 * @note The entry counter is a 16-bit value; N is re-read after every pair of stores.
 */
static inline void dummyRows(V90TRN2Designer* self, V90MappingParams* mp, V90Parameters* P)
{
  uint8_t* base = (uint8_t*)mp;
  int k = 0;
  do
  {
    int i = 0;
    uint8_t c = 0x4e;
    unsigned o = 0;
    while (i < P->nofUcodes)
    {
      base[4u + o] = c;
      base[0x304u + o] = c;
      i = (int16_t)(i + 1);
      c = (uint8_t)(c - 1);
      P = self->params_;
      o++;
    }
    k = (int16_t)(k + 1);
    base += 0x80;
  } while ((int16_t)k <= 5);
}

/* ------------------------------------------------------------------------------------------------ small functions */

V90TRN2Designer::V90TRN2Designer(V90Parameters* p, V90ConstellationPower* pw)
{
  params_ = p;
  power_ = pw;
}

V90TRN2Designer::~V90TRN2Designer() {}

void V90TRN2Designer::setNofUcodesInTrn2(short n)
{
  if (n != 0)
  {
    V90Parameters* P = params_;
    P->nofUcodes = P->nofUcodesInTrn2;
  }
}

unsigned V90TRN2Designer::maxK(V90MappingParams* mp) { return kInt(mp); }

void V90TRN2Designer::setTrn2DummyConstel(V90MappingParams* mp) { dummyRows(this, mp, params_); }

/* ------------------------------------------------------------------------------------------------ V90TRN2Design */

int V90TRN2Designer::V90TRN2Design(V90MappingParams* mp, short (*A)[128], short (*B)[128], unsigned char (*valid)[128], short* flags, PcmType pcmA, PcmType pcm,
                                   short unused, unsigned char* curIn, unsigned limit, unsigned char powerIdx, V90SpecialSpectralConditions special)
{
  (void)unused;
  const short* Af = (const short*)(const void*)A;
  const short* Bf = (const short*)(const void*)B;
  const uint8_t* Vf = (const uint8_t*)(const void*)valid;
  const bool alaw = (unsigned)pcm != 0u;
  V90Parameters* P;
  uint8_t ab[6];
  uint8_t maxU;
  unsigned p;

  /* counts, spectral constants and the data rate word D = K - sr + 6 */
  P = params_;
  mp->flag_ = 1;
  for (int i = 0; i <= 5; i++)
    mp->cnt_[i] = (uint32_t)P->nofUcodes;
  uint32_t sr = spectralBody(P, mp, limit, (int)special);
  {
    uint32_t K = kInt(mp);
    mp->kBits_ = K - sr + 6u;
  }
  for (int i = 0; i <= 5; i++)
    mp->idx_[i] = 0;

  /* the highest ucode the power limit allows: G.711 code of sqrt(limit) * 1.627 minus 2, but at least params->maxUcode */
  {
    const unsigned* lim = (const unsigned*)(const void*)V90ConstellationPower::averagePowerLimits;
    uint32_t L = lim[(unsigned)powerIdx + 4u];
    uint32_t lv = trunc64lo(dsp_sqrt((LD)L));
    int level = dsp_trunc_i32((LD)lv * (LD)kFactor);
    maxU = (uint8_t)(g711(alaw, level) - 2);
    edprintf("V90TRN2Design: max TX index = %d, givin power limit = %d, implying max level =%d\r\n", (int)powerIdx, (int)lv, level);
  }
  P = params_;
  edprintf("V90TRN2Design: ...hence max ucode (after factor) = %d, by params maxUcode = %d\r\n", (int)maxU, P->maxUcode);
  P = params_;
  {
    uint8_t m1 = (uint8_t)P->maxUcode;
    if (maxU < m1)
      maxU = m1;
  }

  /* the start ucode of each phase: the caller's, lowered while its G.711 code exceeds the maximum (never ends if every code does) */
  for (int i = 0; i <= 5; i++)
  {
    uint8_t c = curIn[i];
    ab[i] = c;
    while (g711(alaw, Af[(unsigned)i * 128u + c]) > (int)maxU)
    {
      c = (uint8_t)(c - 1);
      ab[i] = c;
    }
  }

  /* per phase: choose N ucodes, by the dMin design or the forced design; a phase without a solution goes to the failure exit */
  P = params_;
  for (p = 0; p <= 5; p++)
  {
    const unsigned off = p << 7;
    if (flags[p] == 0)
    {
      /* the dMin design: dMin = trunc(0.85 * A[cur] / (N - 0.5)); N ucodes from the top, each at least dMin above the one before */
      int16_t dMin;
      int half;
      int j;
      uint8_t cl;
      uint8_t cur = ab[p];
      {
        LD den = (LD)P->nofUcodes - (LD)kHalfF;
        LD num = (LD)Af[off + cur] * (LD)kDmin;
        dMin = trunc16(num / den);
      }
      edprintf("Trn2 Design: phase %d, dMin = %d...\r\n", (int)p, (int)dMin);
      P = params_;
      half = (int)dMin / 2;
      cl = (uint8_t)P->unk360;
      j = (int16_t)(P->nofUcodes - 1);
      if (j < 0)
        continue;
      for (;;)
      {
        /* skip ucodes whose table word is below half the distance, but not past `cur` */
        if (Af[off + cl] < (int16_t)half)
        {
          for (;;)
          {
            if (cur < cl)
              break;
            cl = (uint8_t)(cl + 1);
            if (!(Af[off + cl] < (int16_t)half))
              break;
          }
        }

        /* accept it; the next one must be at least dMin above it */
        LA(mp, p, (unsigned)j) = cl;
        half = (int16_t)((int)dMin + (int)(uint16_t)Af[off + cl]);
        j = (int16_t)(j - 1);
        if (cur <= cl)
        {
          /* ran out of ucodes: success only if all N slots are already filled */
          P = params_;
          if ((int16_t)j >= 0)
            goto fail;
          break;
        }
        if (j >= 0)
          continue;
        P = params_;
        break;
      }
    }
    else
    {
      /* the forced design: ucode `cur` first, then ucodes below it whose two table words are at least half the running distance rr and not above the limit lim;
         the distance shrinks round by round */
      uint8_t cur = ab[p];
      int cnt = 0;
      int t = 0;
      int16_t rr;
      {
        LD recip = (LD)1.0L / ((LD)P->nofUcodes - (LD)kHalfF);
        rr = trunc16((LD)Af[off + cur] * recip);
      }
      if (P->nofUcodes > 0)
      {
        for (;;)
        {
          /* one round: restart the list at `cur` and scan downwards for ucodes that fit */
          int lo = (int)Bf[off + cur] - (int)rr;
          int hi = (int)Af[off + cur] - (int)rr;
          int16_t lim = (int16_t)(lo <= hi ? lo : hi);
          int16_t a, b;
          int idx;
          LA(mp, p, 0) = cur;
          cnt = 1;
          P = params_;
          idx = (int)cur - 1;
          if (!(P->unk360 > idx))
          {
            const int16_t halfR = (int16_t)((int)rr / 2);
            for (;;)
            {
              a = (int16_t)Af[(int)off + idx];
              if (a < halfR)
              {
                b = (int16_t)Bf[(int)off + idx];
                break;
              }
              b = (int16_t)Bf[(int)off + idx];
              if (b < halfR || a == 0 || b == 0)
                break;
              if (!(a > lim) && !(b > lim) && Vf[(int)off + idx] != 0)
              {
                /* accept: append the ucode and lower the limit by the new distance */
                LA(mp, p, (unsigned)cnt) = (uint8_t)idx;
                cnt = (int16_t)(cnt + 1);
                {
                  int l2 = (int)(int16_t)Bf[(int)off + idx] - (int)rr;
                  int h2 = (int)(int16_t)Af[(int)off + idx] - (int)rr;
                  lim = (int16_t)(l2 <= h2 ? l2 : h2);
                }
                P = params_;
                if (cnt == P->nofUcodes)
                  break;
              }
              idx--;
              if (P->unk360 <= idx)
                continue;
              a = (int16_t)Af[(int)off + idx];
              b = (int16_t)Bf[(int)off + idx];
              break;
            }
          }
          else
          {
            a = (int16_t)Af[(int)off + idx];
            b = (int16_t)Bf[(int)off + idx];
          }
          t = (int16_t)(t + 1);

          /* shrink the distance by the shortfall of the last candidate, and stop when N were found or after 200 rounds */
          {
            int16_t m = (b <= a) ? b : a;
            int n1 = P->nofUcodes;
            LD t5 = (LD)rr * (LD)kHalfF - (LD)m;
            LD t7 = (LD)1.0L / ((LD)n1 + (LD)kHalfF);
            bool more = (cnt < n1) && ((int16_t)t <= 199);
            rr = trunc16((LD)rr - t5 * t7);
            if (!more)
              break;
          }
        }
      }
      if (cnt != P->nofUcodes)
      {
        if ((int16_t)cnt >= 0)
          goto fail;
      }
    }
  }

  /* the second list: the G.711 codes in the target codec */
  for (int k = 0; k <= 5; k++)
  {
    const unsigned off = (unsigned)k << 7;
    if (P->nofUcodes > 0)
    {
      int i = 0;
      do
      {
        unsigned a = LA(mp, (unsigned)k, (unsigned)i);
        if ((unsigned)pcmA == (unsigned)pcm)
        {
          int code = g711(alaw, Af[off + a]);
          LB(mp, (unsigned)k, (unsigned)i) = (code < (int)a) ? (uint8_t)code : (uint8_t)a;
        }
        else
        {
          LB(mp, (unsigned)k, (unsigned)i) = (uint8_t)g711(alaw, Af[off + a]);
        }
        i++;
        P = params_;
      } while (P->nofUcodes > i);
    }
  }

  /* power and power index of the designed constellation */
  {
    float slot;
    float Pw = storeF(&slot, (LD)power_->getPower(mp, V90_TX_POINT_CODEC_OUTPUT, pcm));
    unsigned idx = power_->getPowerIndexForPower(Pw);
    edprintf("--------------------------------------\r\n");
    edprintf("V90TRN2Design:\r\n");
    Split s = sqrtSplit(Pw);
    edprintf("V90TRN2Design: sqrt(power)  = %c%d.%03d\r\n", s.sign, s.ip, s.fp);
    Split d = dbSplit(idx);
    edprintf("V90TRN2Design: index %d, power in dBm0  = %c%d.%01d\r\n", (int)idx, d.sign, d.ip, d.fp);
  }

  /* the design report (debug level > 1): header, then one line per ucode slot with the six ucodes of each list and the amplitude of each */
  {
    unsigned lv = dsplibs_debug_level;
    if (lv > 1)
    {
      dsplibs_debug_printf("\n--------------------------------------\r\n");
      lv = dsplibs_debug_level;
      if (lv > 1)
      {
        dsplibs_debug_printf("V90TRN2Designer : ADI design report : \r\n");
        lv = dsplibs_debug_level;
        if (lv > 1)
        {
          int n = params_->nofUcodes;
          dsplibs_debug_printf("trn2 size  :  %d\r\n", n);
          lv = dsplibs_debug_level;
          if (lv > 1)
          {
            dsplibs_debug_printf("\n------------------------------------------------------------------------------------------------------\r\n");
            lv = dsplibs_debug_level;
          }
        }
      }
    }
    P = params_;
    if (P->nofUcodes > 0)
    {
      unsigned i = 0;
      for (;;)
      {
        if (lv > 1)
        {
          unsigned a0 = LA(mp, 0, i), a1 = LA(mp, 1, i), a2 = LA(mp, 2, i), a3 = LA(mp, 3, i), a4 = LA(mp, 4, i), a5 = LA(mp, 5, i);
          dsplibs_debug_printf("trn2 constel[%d]  :  %d %d %d %d %d %d  :  %d %d %d %d %d %d  :  %d %d %d %d %d %d\r\n", (int)i, (int)a0, (int)a1, (int)a2,
                               (int)a3, (int)a4, (int)a5, (int)LB(mp, 0, i), (int)LB(mp, 1, i), (int)LB(mp, 2, i), (int)LB(mp, 3, i), (int)LB(mp, 4, i),
                               (int)LB(mp, 5, i), (int)Af[0 * 128u + a0], (int)Af[1 * 128u + a1], (int)Af[2 * 128u + a2], (int)Af[3 * 128u + a3],
                               (int)Af[4 * 128u + a4], (int)Af[5 * 128u + a5]);
          i++;
          P = params_;
          lv = dsplibs_debug_level;
        }
        else
        {
          i++;
        }
        if (!(P->nofUcodes > (int)i))
          break;
      }
    }
    if (lv > 1)
      dsplibs_debug_printf("\n------------------------------------------------------------------------------------------------------\r\n");
  }
  return 1;

fail:
  /* a phase found no constellation: fill the dummy one and report the failure */
  dummyRows(this, mp, P);
  if (dsplibs_debug_level > 1)
    dsplibs_debug_printf("V90TRN2Designer::V90TRN2Design()  constelation design failed !!!\r\n");
  return 0;
}
