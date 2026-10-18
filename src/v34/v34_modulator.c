/**
 * @file v34_modulator.c
 * @brief V.34 transmit vector modulator: modulatevector().
 *
 * modulatevector() is called once per transmitted symbol.  Every 8th call (`k56Idx_ == 8`) it generates a new block of 8 output points (four 2-D
 * symbol groups of two points each) and stores them in `symPts_`; every call then hands one point of the block, selected by `k56Idx_`, to
 * txmit() (optionally through V34nlencoder() when bit 6 of the high byte of `txFlags_` is set).
 *
 * Block generation:
 *  - getFrame() fills the frame state (the 32-bit shell index `shellIdx_`, the mapper bits).
 *  - Shell mapper: a 7-step binary search over the 128-entry table T (`unk2628_ + 512`) finds the ring group; three "peel" loops over the tables A
 *    (`unk2628_ + 256`) and B (`unk2628_`) decompose the remaining index into the eight ring numbers (four (a, b) pairs).
 *  - Constellation mapper: quarter[] gives the points of a quarter of the 1664-point constellation; the point is rotated by the differential phase.
 *  - Precoder: a six-tap complex filter over the history `precHist_` (coefficient set `p2604_`, short[12]), with the "round and carry"
 *    step quant().
 *  - Trellis encoder: state words `trelState0_` .. `trelState5_`, in a 64-state shift-register form (`w25f6_ == 0x40`) or an LFSR form; the
 *    counters `w261c_` / `w261e_` and the pattern table gInvertPat drive the inversion bit.
 *  - The finished points are scaled by `w2622_` (>> 7).
 *
 * Callees: getFrame() and txmit() (v34_tx_symbol.c), V34nlencoder() (v34_dsp_primitives.c).  Tables: quarter and smIndex (v34_modulator_tables.c),
 * gInvertPat (v34_init_tables.c).  The pointer slots `p2604_` (precoder coefficients, short[12]) and `p2608_` (trellis output table, ushort[64]) are
 * real pointers; the object is reached through the typed members of V34Obj (v34_object_view.h); the four interleaved per-group word tables at `w2a34_` ..
 * `w2a3a_` (stride 8 bytes) and the shell tables `unk2628_` are walked by byte offset (v34_access.h).
 *
 * Hazards (unchecked table reads and traps):
 *  - H1: quarter[idx], idx = (ring << (W & 31)) + offset, is not range-checked by the algorithm; valid 0..415 (a quarter of the 1664-point
 *    constellation).  quarter_at() returns 0 outside, so an out-of-range index is memory-safe here.  For every legal V.34 rate / symbol-rate
 *    combination (rate <= 21600 at 2400 Bd, 26400 at 2743 / 2800, 28800 at 3000, 31200 at 3200, 33600 at 3429) the largest index (m << c) - 1, with m
 *    the ring limit and c the offset width, is <= 415; only illegal combinations (a rate above the symbol rate's maximum) overrun.
 *  - H2: gInvertPat[(short)(Fc + 1)] is valid for 0..15 (every state the set-up code produces); ginv_at() returns 0 outside.
 *  - H3: the integer divisions by A[j1] / B[j2] / B[j3] trap on a zero divisor.
 *  - H4: the peel loops are unbounded: for a shell index >= T[last] + g8[last] they read A / B below and above their arrays (object memory) and stop
 *    when the running value turns negative, so the object must be large enough for a 16-bit index.
 *  - H5: k = `k56Idx_` is not range-checked in the tail (index into the 8-point block `symPts_`).
 */

#include <stddef.h>
#include <stdint.h>

#include "v34_init_tables.h"      /* gInvertPat */
#include "v34_modulator_tables.h" /* quarter, smIndex */
#include "v34_internal.h"

#include "v34_access.h" /* U16 / S16 / U32 byte-offset access to the per-group word tables */
#include "v34_object_view.h"

/*
 * Test hooks: a test harness compiles a private copy of this file with PORT_TRACE_INDEX defined (under renamed symbols) as an index
 * oracle; they do not exist in the shipped library.  Each hook reports the value and returns it unchanged where it is in range, otherwise flags a
 * violation.
 */
#ifdef PORT_TRACE_INDEX
extern int tr_quarter(int idx);
extern int tr_ginv(int idx);
extern int tr_div(int d);
extern int tr_peel(int t, int lo); /* returns nonzero = stop the loop (t ran past lo) */
extern void tr_rings(const uint16_t x[8], uint32_t v, int lo, int m);
extern void tr_cov(int id);
/* coverage classes reported by the oracle copy (branch classes of the code under test) */
enum
{
  CV_GEN,
  CV_NOGEN,
  CV_E4C_BIT4,
  CV_E4C_ZERO,
  CV_E4C_LT,
  CV_E4C_GE,
  CV_BS_ARM1,
  CV_BS_ARM2,
  CV_BS_EQ,
  CV_BS_TBIG,
  CV_PEEL,                        /* + phase*3 + {0: no iteration, 1: one, 2: two or more} (9 ids) */
  CV_PAIR_NORM = CV_PEEL + 9,     /* + pair 0..3 */
  CV_PAIR_ALT = CV_PAIR_NORM + 4, /* + pair 0..3 */
  CV_QUANT_NEG = CV_PAIR_ALT + 4,
  CV_QUANT_POS,
  CV_QUANT_ADDT,
  CV_QUANT_NOADD,
  CV_ROT1 /* + rot 0..3 */,
  CV_ROT2 = CV_ROT1 + 4 /* + rot 0..3 */,
  CV_B0 = CV_ROT2 + 4,
  CV_B1,
  CV_CFG64,
  CV_LFSR_ODD,
  CV_LFSR_EVEN,
  CV_LFSR_CFGNEG,
  CV_CNT_INC,
  CV_CNT_FC,
  CV_CNT_ZERO,
  CV_TAIL_NL,
  CV_TAIL_PLAIN,
  CV_W0,
  CV_WPOS,
  CV_MAX
};
#define COV(id) tr_cov(id)
#define QUARTER_IDX(i) tr_quarter(i)
#define GINV_IDX(i) tr_ginv(i)
#define DIVISOR(d) tr_div(d)
#define PEEL_STOP(t, lo) tr_peel((int)(t), (int)(lo))
#define RINGS_HOOK(x, v, lo, m) tr_rings((x), (v), (lo), (m))
#else
#define COV(id) ((void)0)
#define QUARTER_IDX(i) (i)
#define GINV_IDX(i) (i)
#define DIVISOR(d) (d)
#define PEEL_STOP(t, lo) 0
#define RINGS_HOOK(x, v, lo, m) ((void)0)
#endif

/** @brief quarter[idx] where idx is in 0..415, otherwise 0 (hazard H1). */
static int quarter_at(int idx)
{
  idx = QUARTER_IDX(idx);
  if (idx >= 0 && idx < 416)
    return quarter[idx];
  return 0;
}
/** @brief gInvertPat[idx] where idx is in 0..15, otherwise 0 (hazard H2). */
static unsigned int ginv_at(int idx)
{
  idx = GINV_IDX(idx);
  if (idx >= 0 && idx < 16)
    return gInvertPat[idx];
  return 0;
}

/**
 * @brief One "peel" loop of the shell mapper: split a value over a product table.
 * @param tab  table of 16-bit entries
 * @param r    value to decompose
 * @param lo   row length index (16-bit)
 * @param R    receives r minus the first t products (r itself when t == 0)
 * @return the split index t, the number of loop iterations
 * @note cur = r - tab[lo] * tab[0]; while (cur >= 0, signed 32-bit) { t++; R = cur; cur = R - tab[t] * tab[lo - t]; }.  All arithmetic wraps at 32 bits.
 * @warning the table indices are not bounded (hazard H4).
 */
static unsigned int peel(const uint16_t* tab, uint32_t r, unsigned int lo, uint32_t* R)
{
  uint32_t cur = r - (uint32_t)tab[lo] * (uint32_t)tab[0];
  uint32_t Rv = r;
  unsigned int t = 0;
  while ((int32_t)cur >= 0)
  {
    if (PEEL_STOP(t + 1, lo))
      break;
    t++;
    Rv = cur;
    cur = Rv - (uint32_t)tab[t] * (uint32_t)tab[(int)lo - (int)t];
  }
  *R = Rv;
  return t;
}
#define PEEL_COV(phase, t) COV(CV_PEEL + 3 * (phase) + ((t) > 2u ? 2 : (int)(t)))

/**
 * @brief Assemble one (a, b) pair of ring numbers.
 * @param which  pair number 0..3 (coverage id in the test builds only)
 * @param xa     receives ring number a
 * @param xb     receives ring number b
 * @param idx    pair index (16-bit)
 * @param s      pair sum (16-bit)
 * @param L      ring limit (16-bit)
 * @note if s < L (unsigned 16-bit compare): a = idx, b = s - idx; otherwise b = L - idx - 1, a = s - b.  Results are truncated to 16 bits.
 */
static void pair(int which, uint16_t* xa, uint16_t* xb, uint32_t idx, uint32_t s, uint32_t L)
{
  (void)which; /* coverage id only (test builds) */
  if ((uint16_t)s < (uint16_t)L)
  {
    COV(CV_PAIR_NORM + which);
    *xa = (uint16_t)idx;
    *xb = (uint16_t)(s - idx);
  }
  else
  {
    uint32_t b = L - idx - 1u;
    COV(CV_PAIR_ALT + which);
    *xb = (uint16_t)b;
    *xa = (uint16_t)(s - b);
  }
}

/**
 * @brief The "round and carry" step after the six-tap precoder sum.
 * @param acc   precoder accumulator
 * @param t16   (u16)(Rs << 7), the carry threshold
 * @param mask  sext16(-2 * Rs)
 * @param d     receives the low 16 bits of the shifted accumulator
 * @param e     receives the rounded correction term
 * @note a = acc + (acc < 0); full = a >> 14 (logical shift; only bits 14..29 reach the 16-bit d, so logical vs arithmetic is unobservable);
 *       d = (u16)full; v = sext16(d), plus sext16(t16) when ((full & (t16 | 0x80ff)) & 0xffff) != t16; e = (u16)((v >> 7) & mask).
 * @note Only e bits 0..8 are used downstream, and z = e + P is always odd (quarter points have odd coordinates, e is even because mask is even), so the
 *       `- 1` in the smIndex index expressions of modulatevector() is redundant.
 */
static void quant(uint32_t acc, uint16_t t16, uint32_t mask, uint16_t* d, uint16_t* e)
{
  uint32_t a = acc + (((int32_t)acc < 0) ? 1u : 0u);
  uint32_t full = a >> 14;
  int32_t sd = (int16_t)(uint16_t)full;
  int32_t t = (int16_t)t16;
  int carry = (uint16_t)(full & (0xffff0000u | (uint32_t)(t16 | 0x80ffu))) != t16;
  int32_t v = carry ? (int32_t)((uint32_t)sd + (uint32_t)t) : sd;
  COV(((int32_t)acc < 0) ? CV_QUANT_NEG : CV_QUANT_POS);
  COV(carry ? CV_QUANT_ADDT : CV_QUANT_NOADD);
  *d = (uint16_t)full;
  *e = (uint16_t)(((uint32_t)v >> 7) & mask);
}

/** @brief Unpack a quarter[] entry: low half = (u16)(v >> 8) (v sign-extended, arithmetic shift), high half = sext8(v & 0xff) as 16 bits. */
static uint32_t unpack_point(int32_t v)
{
  uint16_t px = (uint16_t)(v >> 8);
  uint32_t py = (uint32_t)(int32_t)(int8_t)(uint8_t)v; /* sign-extend the low byte */
  return (py << 16) | px;
}

/** @brief Rotate a packed point (x low 16 bits, y high 16 bits) by `rot` quarter turns: 0: (x, y)  1: (-y, x)  2: (-x, -y)  3: (y, -x). */
static uint32_t rotate_point(uint32_t P, unsigned int rot)
{
  uint16_t x = (uint16_t)P, y = (uint16_t)(P >> 16);
  switch (rot & 3u)
  {
  case 1:
    return ((uint32_t)x << 16) | (uint16_t)(-(int)y);
  case 2:
    return ((uint32_t)(uint16_t)(-(int)y) << 16) | (uint16_t)(-(int)x);
  case 3:
    return ((uint32_t)(uint16_t)(-(int)x) << 16) | y;
  default:
    return P;
  }
}

/**
 * @brief Produce one transmit symbol: generate a block of 8 points every 8th call, then pass one point to txmit().
 * @param obj  the V.34 object
 * @note reads and writes the shell / mapper / precoder / trellis state members named in the file header; calls getFrame() when generating a block and
 *       txmit() on every call.
 * @warning the hazards H1..H5 of the file header apply: a zero divisor in the shell mapper traps, and `k56Idx_` must stay below 8 outside the
 *          generation call.
 */
void modulatevector(V34Obj* obj)
{
  unsigned int k16 = obj->tx.k56Idx_; /* the 16-bit point counter; the compare with 8 is 16-bit */

  if ((uint16_t)k16 == 8)
  {
    uint16_t L;
    uint32_t v;
    unsigned int Wshift;
    int32_t ph;
    int lo;
    unsigned int mid, hi, cnt;
    uint16_t x[8];
    const uint16_t* A = (const uint16_t*)(const void*)(obj->tx.unk2628_ + 256);
    const uint16_t* B = (const uint16_t*)(const void*)obj->tx.unk2628_;
    const uint32_t* T = (const uint32_t*)(const void*)(obj->tx.unk2628_ + 512);
    uint32_t r, R1, R2, R3;
    unsigned int j1, j2, j3;
    uint32_t d1, d2, d3, q, rem1, qa, ra, qb, rb, m2, s4;
    const short* P;
    const uint16_t* Q;
    int i;
    unsigned int pos;

    /* ---- frame-limit bookkeeping before the frame is fetched: count blocks (waa74_) up to w25e0_, then flag the end (txFlags_ bit 4, w2214_) ---- */
    {
      unsigned int w25c2 = obj->tx.txFlags_;
      if (w25c2 & 0x10u)
      {
        COV(CV_E4C_BIT4);
      }
      else if (obj->rx.blkb.wE4c_ == 0)
      {
        COV(CV_E4C_ZERO);
      }
      else
      {
        int32_t c = obj->waa74_;
        obj->waa74_ = (int32_t)((uint32_t)c + 1u); /* the counter is stored before the compare */
        if (c >= (int32_t)obj->tx.w25e0_)
        { /* signed compare against the zero-extended limit */
          COV(CV_E4C_GE);
          obj->mod.w2214_ = 1;
          obj->tx.txFlags_ = (uint16_t)(w25c2 | 0x10u);
        }
        else
        {
          COV(CV_E4C_LT);
        }
      }
    }

    COV(CV_GEN);
    getFrame(obj);

    L = obj->tx.w25f2_;
    v = obj->tx.shellIdx_;
    Wshift = (unsigned int)(uint8_t)obj->tx.w25f4_ & 31u; /* byte shift count, masked to 5 bits */
    ph = (int32_t)obj->tx.w2618_;                         /* sign-extended */

    /* ---- shell mapper: binary search over T[0..127], exactly 7 steps, unsigned compare ---- */
    lo = 0;
    mid = 0x40;
    hi = 0x80;
    cnt = 0;
    do
    {
      if (T[mid] == v)
        COV(CV_BS_EQ);
      if (T[mid] >= 0x80000000u)
        COV(CV_BS_TBIG);
      if (T[mid] > v)
      { /* T[mid] > v: search the lower half */
        COV(CV_BS_ARM1);
        unsigned int nm = (unsigned int)(int16_t)((lo + mid) / 2u);
        hi = mid;
        mid = nm;
      }
      else
      {
        COV(CV_BS_ARM2);
        unsigned int nm = (unsigned int)(int16_t)((hi + mid) / 2u);
        lo = (int)mid;
        mid = nm;
      }
      cnt++;
    } while ((int16_t)cnt < 7);
    lo = (int)(uint16_t)lo;

    /* ---- peel phase 1: split index j1 over A, then quotient q and remainder rem1 ---- */
    r = v - T[lo];
    j1 = (uint16_t)peel(A, r, (unsigned int)lo, &R1);
    PEEL_COV(0, j1);
    d1 = A[j1];
    q = (uint32_t)((int32_t)R1 / (int32_t)DIVISOR((int)d1));
    rem1 = (uint32_t)((int32_t)R1 % (int32_t)DIVISOR((int)d1));

    /* ---- peel phase 2: rem1 over B gives j2, then qa / ra ---- */
    j2 = (uint16_t)peel(B, rem1, j1, &R2);
    PEEL_COV(1, j2);
    d2 = B[j2];
    qa = (uint16_t)((int32_t)R2 / (int32_t)DIVISOR((int)d2));
    ra = (uint16_t)((int32_t)R2 % (int32_t)DIVISOR((int)d2));

    /* ---- peel phase 3: q over B with row length (u16)(lo - j1) gives j3, then qb / rb ---- */
    m2 = (uint16_t)((unsigned int)lo - j1);
    j3 = (uint16_t)peel(B, q, (unsigned int)m2, &R3);
    PEEL_COV(2, j3);
    d3 = B[j3];
    qb = (uint16_t)((int32_t)R3 / (int32_t)DIVISOR((int)d3));
    rb = (uint16_t)((int32_t)R3 % (int32_t)DIVISOR((int)d3));
    s4 = (uint16_t)((unsigned int)lo - j1 - j3);

    /* ---- the eight ring numbers: four (a, b) pairs ---- */
    pair(0, &x[0], &x[1], ra, j2, L);
    pair(1, &x[2], &x[3], qa, (uint16_t)(j1 - j2), L);
    pair(2, &x[4], &x[5], rb, j3, L);
    pair(3, &x[6], &x[7], qb, s4, L);
    RINGS_HOOK(x, v, lo, (int)L);

    /* ---- per 2-D symbol group i = 0..3: two points each, mapped, precoded, trellis-encoded ---- */
    P = (const short*)obj->tx.p2604_;
    Q = (const uint16_t*)obj->tx.p2608_;
    pos = 0;
    for (i = 0; i < 4; i++)
    {
      const unsigned int grp = 8u * (unsigned int)i;
      uint32_t P1, P2;
      uint16_t d1a, e1a, d2a, e2a, d3a, e3a, d4a, e4a;
      uint16_t z1, z2, z3, z4;
      int32_t Rs;
      uint16_t t16;
      uint32_t mask;
      uint32_t a1, a2, a3, a4;
      int k2;
      uint32_t g, bbit;
      unsigned int rot;
      unsigned int idxA, idxB, qidx;
      unsigned int cfg16;
      unsigned int E, Fc, Dn;
      uint16_t* H = obj->tx.precHist_;

      /* -- first point: quarter[(x[pos] << W) + O1], rotated by the differential phase -- */
      {
        int32_t xv = (int16_t)x[pos];
        int32_t idx = (int32_t)((uint32_t)xv << Wshift) + (int32_t)U16(&obj->tx.w2a38_, grp);
        P1 = unpack_point(quarter_at(idx));
      }

      /* precoder sums for the first point (two accumulators: real and imaginary part), then round-and-carry */
      Rs = (int32_t)obj->tx.w2626_;
      t16 = (uint16_t)((uint32_t)Rs << 7);
      mask = (uint32_t)(int32_t)(int16_t)(uint16_t)(0u - 2u * (uint32_t)Rs);
      a1 = a2 = 0x1fffu;
      for (k2 = 0; k2 < 6; k2++)
      {
        int32_t h = (int32_t)(int16_t)H[k2];
        a1 += (uint32_t)((int32_t)P[k2] * h);
        a2 += (uint32_t)((int32_t)P[6 + k2] * h);
      }

      quant(a1, t16, mask, &d1a, &e1a);
      quant(a2, t16, mask, &d2a, &e2a);
      g = (uint32_t)(int32_t)(int16_t)(uint16_t)(e2a ^ e1a);

      ph = (int32_t)(((uint32_t)ph + (uint32_t)U16(&obj->tx.w2a36_, grp)) & 3u);
      rot = (0u - (uint32_t)ph) & 3u;
      COV(CV_ROT1 + (int)rot);
      P1 = rotate_point(P1, rot);

      z1 = (uint16_t)(e1a + (uint16_t)P1);
      z2 = (uint16_t)((uint16_t)(P1 >> 16) + e2a);
      /* shift the precoder history by one complex tap, then store the new point */
      {
        uint16_t h1 = H[1], h2 = H[2], h3 = H[3];
        uint16_t h0;
        H[3] = h1;
        H[5] = h3;
        H[4] = h2;
        h0 = H[0];
        H[2] = h0;
      }
      {
        uint16_t o1 = (uint16_t)(((uint32_t)(int32_t)(int16_t)z1 << 7) - d1a);
        uint16_t o2 = (uint16_t)(((uint32_t)(int32_t)(int16_t)z2 << 7) - d2a);
        obj->tx.symPts_[4 * i] = o1;
        H[0] = o1;
        obj->tx.symPts_[4 * i + 1] = o2;
        H[1] = o2;
      }

      /* -- second point: quarter[(x[pos + 1] << W) + O2], same steps, rotation also includes the trellis bit -- */
      {
        int32_t xv = (int16_t)x[(int16_t)(pos + 1)];
        int32_t idx = (int32_t)((uint32_t)xv << Wshift) + (int32_t)U16(&obj->tx.w2a3a_, grp);
        P2 = unpack_point(quarter_at(idx));
      }
      a3 = a4 = 0x1fffu;
      for (k2 = 0; k2 < 6; k2++)
      {
        int32_t h = (int32_t)(int16_t)H[k2];
        a3 += (uint32_t)((int32_t)P[k2] * h);
        a4 += (uint32_t)((int32_t)P[6 + k2] * h);
      }

      quant(a3, t16, mask, &d3a, &e3a);
      quant(a4, t16, mask, &d4a, &e4a);

      g ^= (uint32_t)(int32_t)(int16_t)(uint16_t)(e4a ^ e3a);
      g = (uint32_t)((int32_t)g >> 1);
      bbit = (g ^ (uint32_t)obj->tx.trelState0_ ^ (uint32_t)obj->tx.w261a_) & 1u;

      rot = (0u - ((uint32_t)ph + 2u * (uint32_t)(int32_t)S16(&obj->tx.w2a34_, grp) + bbit)) & 3u;
      COV(CV_ROT2 + (int)rot);
      COV(bbit ? CV_B1 : CV_B0);
      P2 = rotate_point(P2, rot);

      z3 = (uint16_t)(e3a + (uint16_t)P2);
      z4 = (uint16_t)((uint16_t)(P2 >> 16) + e4a);
      {
        uint16_t h1 = H[1], h2 = H[2], h3 = H[3];
        uint16_t h0;
        H[3] = h1;
        H[5] = h3;
        H[4] = h2;
        h0 = H[0];
        H[2] = h0;
      }
      {
        uint16_t o3 = (uint16_t)(((uint32_t)(int32_t)(int16_t)z3 << 7) - d3a);
        uint16_t o4 = (uint16_t)(((uint32_t)(int32_t)(int16_t)z4 << 7) - d4a);
        obj->tx.symPts_[4 * i + 2] = o3;
        H[0] = o3;
        obj->tx.symPts_[4 * i + 3] = o4;
        H[1] = o4;
      }

      pos = (unsigned int)(uint16_t)(pos + 2);

      /* -- trellis symbol index: smIndex[second point] + 8 * smIndex[first point], then advance the trellis state -- */

      idxA = (((unsigned int)((int32_t)(int16_t)z3 - 1) & 6u) >> 1) + (((unsigned int)((int32_t)(int16_t)z4 - 1) & 6u) * 2u);
      idxB = (((unsigned int)((int32_t)(int16_t)z1 - 1) & 6u) >> 1) + (((unsigned int)((int32_t)(int16_t)z2 - 1) & 6u) * 2u);
      qidx = (uint16_t)((unsigned int)(uint16_t)smIndex[idxA] + 8u * (unsigned int)(int32_t)smIndex[idxB]);

      cfg16 = obj->tx.w25f6_;
      COV(Wshift == 0 ? CV_W0 : CV_WPOS);
      if ((uint16_t)cfg16 == 0x40)
      {
        COV(CV_CFG64);
        /* 64-state shift-register form on the six words trelState0_ .. trelState5_ (all reads use the old values) */
        int32_t qq = (int32_t)(int16_t)Q[qidx];
        uint32_t F = obj->tx.trelState4_, G = obj->tx.trelState5_;
        uint32_t C0 = obj->tx.trelState0_, C1 = obj->tx.trelState1_, C2 = obj->tx.trelState2_, E3 = obj->tx.trelState3_;
        int32_t Ba = (int32_t)(int16_t)(uint16_t)(((uint32_t)qq & 1u) ^ F);
        int32_t Cc = (int32_t)(int16_t)(uint16_t)(F ^ G);
        obj->tx.trelState1_ = (uint16_t)C0;
        obj->tx.trelState5_ = (uint16_t)(((uint32_t)(qq >> 3) ^ (uint32_t)Cc) ^ ((uint32_t)Ba & E3));
        obj->tx.trelState3_ = (uint16_t)((uint32_t)Ba ^ E3);
        obj->tx.trelState2_ = (uint16_t)E3;
        obj->tx.trelState4_ = (uint16_t)((uint32_t)Cc ^ C2 ^ (uint32_t)(qq >> 2) ^ ((uint32_t)(qq >> 1) & E3));
        obj->tx.trelState0_ = (uint16_t)((uint32_t)(qq >> 1) ^ (C1 ^ E3));
      }
      else
      {
        /* LFSR form: newC0 = (sext16(q ^ C0) ^ ((C0 & 1) * sext16(cfg))) >> 1 (logical 32-bit shift) */
        uint32_t C0 = obj->tx.trelState0_;
        uint32_t qq = Q[qidx];
        COV((C0 & 1u) ? CV_LFSR_ODD : CV_LFSR_EVEN);
        if ((int16_t)cfg16 < 0)
          COV(CV_LFSR_CFGNEG);
        uint32_t t = (uint32_t)(int32_t)(int16_t)(uint16_t)(qq ^ C0);
        t ^= (C0 & 1u) * (uint32_t)(int32_t)(int16_t)cfg16;
        obj->tx.trelState0_ = (uint16_t)(t >> 1);
      }

      /* -- counters: E (period w25e2_), Fc (period w2620_) and the inversion bit D = gInvertPat[Fc] -- */
      E = obj->tx.w261c_;
      if ((int32_t)(int16_t)(uint16_t)(E + 1u) < (int32_t)obj->tx.w25e2_)
      {
        COV(CV_CNT_INC);
        obj->tx.w261c_ = (uint16_t)(E + 1u);
        obj->tx.w261a_ = 0;
      }
      else
      {
        obj->tx.w261c_ = 0;
        Fc = (uint16_t)(obj->tx.w261e_ + 1u);
        if ((int16_t)Fc < obj->tx.w2620_)
        {
          COV(CV_CNT_FC);
          obj->tx.w261e_ = (uint16_t)Fc;
          Dn = ginv_at((int)(int16_t)Fc);
        }
        else
        {
          COV(CV_CNT_ZERO);
          Dn = 0;
          obj->tx.w261e_ = 0;
        }
        obj->tx.w261a_ = (uint16_t)Dn;
      }
    }

    /* store the differential phase and scale the 8 points by w2622_ (>> 7) */
    obj->tx.w2618_ = (uint16_t)ph;
    {
      int32_t s = (int32_t)obj->tx.w2622_;
      int j;
      for (j = 0; j < 8; j++)
      {
        int32_t X = (int32_t)obj->tx.symPts_[2 * j];
        int32_t Y = (int32_t)obj->tx.symPts_[2 * j + 1];
        obj->tx.symPts_[2 * j] = (int16_t)((int32_t)((uint32_t)X * (uint32_t)s) >> 7);
        obj->tx.symPts_[2 * j + 1] = (int16_t)((int32_t)((uint32_t)Y * (uint32_t)s) >> 7);
      }
    }
    obj->tx.k56Idx_ = 0;
    k16 = 0;
  }
  else
  {
    COV(CV_NOGEN);
  }

  /* ---- tail: k = (short)k16 indexes the 8-point block `symPts_` (H5: the bound k < 8 is not established here); hand point k to txmit() and store k + 1 ---- */
  if (obj->tx.txFlags_ & 0x4000u)
  {
    COV(CV_TAIL_NL);
    obj->tx.k56Idx_ = (uint16_t)(k16 + 1u); /* stored before the V34nlencoder() call */
    V34nlencoder(&obj->tx.symPts_[2 * (int)(int16_t)k16], (short*)(void*)&obj->tx.txPoint_);
    txmit(obj);
  }
  else
  {
    COV(CV_TAIL_PLAIN);
    uint32_t pt = U32(&obj->tx.symPts_[2 * (int)(int16_t)k16], 0);
    obj->tx.k56Idx_ = (uint16_t)(k16 + 1u);
    obj->tx.txPoint_ = pt;
    txmit(obj);
  }
}
