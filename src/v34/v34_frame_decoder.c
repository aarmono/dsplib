/**
 * @file v34_frame_decoder.c
 * @brief V.34 receive symbol decision and frame decoder: trellis (Viterbi) decoding, shell demapping and delivery of the demapped bits.
 *
 * Entry point: decoderv34(obj), called once per received symbol by receiver() (v34_receiver.c).  Depending on the receive flags it either
 * - runs the frame decoder: demapFrame() with the received (I, Q) point; or
 * - slices the point to the nearest of the four rxvect4 points, takes the differential 2-bit value against the previous decision and descrambles it
 *   (the plain 4-point mode used by the handshake phases).
 *
 * Frame mode data flow, per received symbol with the running symbol counter `idx` (w04ca_):
 *   decoderv34 -> demapFrame(obj, received point, decided point, idx)
 *     - idx even: the point is stored in the current stage record and decodeDepth() traces the trellis back to the oldest stage and produces the
 *       demapped fields of one four-dimensional symbol (two 2-D points): four 16-bit values (a 1-bit field, a 2-bit field and two `w0a14_`-bit
 *       fields) and two ring numbers (5 bits each).  Returns 0.
 *     - idx odd: the second point of the stage is stored and one add-compare-select step of the 16-state trellis is made (candidate lattice points,
 *       squared distances, best subset pair per transition, new path metrics, the surviving predecessor of each state).  The decided point pair is
 *       returned through `out2`.  Returns 1.  When idx & 7 == 7 (one mapping frame of eight 2-D symbols is complete) it also calls shellDemapper()
 *       to turn the four ring-number pairs into the shell-mapping index and putFrame() to hand the frame bits to the descrambler.
 *
 * The frame decoder state lives in a region of the V.34 module object (V34Obj of v34_object_view.h) (`trellis_` is a TrellisMem record; a few words are still reached by offset from the first member of a group).  Its members:
 * - `w0a00_` (N) modulus of the running bit position; `w0a06_` its increment per frame; `w0a08_` the running position; `w0a0e_`, `w0a10_`
 *   the number of raw (uncoded) bits delivered in a frame depending on whether the position wrapped or not; `w0a02_`, `w0a40_` limits of the
 *   two counters `w0a3c_`, `w0a3e_` that step the inversion pattern selector `w0a3a_`; `w0a04_` a width limit that decides the size of a field
 *   in putFrame(); `w0a12_` the shell mapper size (the demapper uses it minus one); `w0a14_` the number of low bits per coordinate (`shift`);
 *   `w0a38_` the previous k lookup value; `w0a42_` (n) the lattice scale; `w0a44_` the shift applied to squared distances; `w0a46_` (Q) the
 *   quantiser step of the precoder rounding.
 * - `fbHist_` (int16[6]) the six-entry history of the precoder feedback filter and `p0a24_` the pointer to its two rows of six coefficients.
 * - `shellX_` (uint16[128]), `shellY_` (uint16[128]) and `shellZ_` (uint32[128]): the shell-mapper tables.
 * - `p0e48_` the frame sink function (the descrambler, a put_fn_t); `w0e4c_` the "frame decoding is running" flag; `rxShellIdx_` the shell index
 *   (32 bits; putFrame() sends it as its low and high 16-bit halves); `fdFields_` four groups of four 16-bit values (the decodeDepth() output of
 *   the four 4-D symbols of a frame); `ringNo_` four groups of two ring numbers (the shellDemapper() parameters).
 * - `trellis_` (TrellisMem, v34_records.h): `metric[16]` the path metrics, `node[32 * 16]` the trellis memory (stage * 16 + state; high byte =
 *   predecessor state, low byte = chosen subset-pair value), `stage[32]` the stage records (best end state, the two received (I, Q) points of the
 *   stage); then `fdStage_`, the current stage number (0 .. 31).
 *
 * Index hazards that are part of the behaviour and are documented at the functions: decodeDepth() reads grid[] without a range check, demapFrame()
 * reads gInvertPat[] without a lower bound check, shellDemapper() indexes X, Y and Z with values derived from frame parameters, and the trellis
 * walk follows signed predecessor bytes unchecked.  All the arithmetic wraps as 16-bit or 32-bit two's complement.
 *
 * Callers: decoderv34() from v34_receiver.c.  Tables: v34_init_tables.c (gInvertPat, rxvect4) and v34_frame_tables.c (kLookup, grid, kTable, kkNormal,
 * kkInvert).
 */

#include <stddef.h>
#include <stdint.h>

#include "v34_init_tables.h"  /* gInvertPat, rxvect4 */
#include "v34_frame_tables.h" /* kLookup, grid, kTable, kkNormal, kkInvert */
#include "core/dsp_host.h"
#include "v34_internal.h"

/* byte-offset accessors into the V.34 object */
#include "v34_access.h"

/* typed-member accessors: the compiler computes the offset of member V34M_<off> of V34Obj (v34_object_view.h); xxxTI = a byte / word INSIDE a member
 * (offsetof(member) + delta) */
#include "v34_object_view.h"

/*
 * Test-only instrumentation: compiled in ONLY when a test harness includes this file with PORT_TRACE_INDEX defined (the library build never defines
 * it).  The hooks report the three unchecked table indices so that the harness can keep its inputs inside the tables.
 */
#ifdef PORT_TRACE_INDEX
/* the hooks return nonzero when the index is out of range; the test build then reads 0 instead */
extern int port_trace_grid(int idx);
extern int port_trace_ginvert(int idx);
extern int port_trace_walk(int e);
#define GRID_AT(i) (port_trace_grid(i) ? 0 : grid[i])
#define GINV_AT(i) (port_trace_ginvert(i) ? 0 : gInvertPat[i])
#define WALK_IDX(e) (port_trace_walk(e) ? 0 : (e)) /* trellis entry index, must stay in 0..511 */
#else
#define GRID_AT(i) grid[i]
#define GINV_AT(i) gInvertPat[i]
#define WALK_IDX(e) (e)
#endif

/* ====================================================================== */
/* putFrame                                                               */
/* ====================================================================== */

/** The frame sink: fn(obj, bits, nbits) pushes the low `nbits` of `bits` into the descrambler (descrambleGPC / descrambleGPA, or a test stub). */
typedef int (*put_fn_t)(void* obj, unsigned int word, int nbits);

/**
 * @brief Hand one demapped frame to the descrambler.
 * @param obj  V.34 module object
 * @note   The frame is the shell index (`rxShellIdx_`, sent as 16-bit halves) and the four groups of four decodeDepth() fields).  They are pushed
 *         through the function pointer `p0e48_` in a fixed order; the callee may write the object, so every word is read right before its call
 *         (`w0a14_` and the pointer are read once at entry).
 *         The running position is advanced first: s = (u16)(w0a08_ + w0a06_); if the modulus w0a00_ is above s (unsigned 16-bit) then w0a08_ = s and
 *         the raw-bit count is w0a10_, otherwise w0a08_ = s - w0a00_ and it is w0a0e_.  A count above 16 is sent as 16 bits plus the remainder
 *         (n & 15); a count of 1..16 is sent as one word; a count of 0 or less sends no raw bits and instead shortens the second field
 *         (2 bits, or 1 when w0a04_ is 8 or less) and adds the (negative) count to the field of the last group.
 * @warning The sink pointer `p0e48_` is called unchecked; it must have been set (by the set-up code) before the first frame is delivered.
 */
void putFrame(V34Obj* obj)
{
  int l14 = 2; /* bits of the second field of the first three groups */
  int l18 = 2; /* bits of the second field of the last group */
  int v = (int16_t)obj->rx.blkb.w0a14_;
  put_fn_t fn = (put_fn_t)obj->rx.blkb.p0e48_;
  uint32_t s = (uint32_t)obj->rx.blkb.w0a08_ + (uint32_t)obj->rx.blkb.w0a06_;
  uint16_t N = obj->rx.blkb.w0a00_;
  int16_t n;

  /* advance the running bit position and pick the raw-bit count of this frame */
  if ((uint16_t)N > (uint16_t)s)
  {
    obj->rx.blkb.w0a08_ = (uint16_t)s;
    n = ((V34Obj*)obj)->rx.blkb.w0a10_;
  }
  else
  {
    obj->rx.blkb.w0a08_ = (uint16_t)(s - N);
    n = ((V34Obj*)obj)->rx.blkb.w0a0e_;
  }

  /* the raw bits come first */
  if (n > 0x10)
  {
    fn(obj, (uint16_t)obj->rx.rxShellIdx_, 0x10);                 /* the low half of the shell index */
    fn(obj, (uint16_t)(obj->rx.rxShellIdx_ >> 16), (int)n & 0xf); /* the high half */
  }
  else if (n > 0)
  {
    fn(obj, (uint16_t)obj->rx.rxShellIdx_, (int)n);
  }
  else
  {
    int t = 2 - (obj->rx.blkb.w0a04_ <= 8 ? 1 : 0);
    l14 = t;
    l18 = (int16_t)((int)n + t);
  }

  /* then the four groups of four fields: 1 bit, l14 bits, v bits, v bits (the last group uses l18 bits for the second field) */
  fn(obj, obj->rx.fdFields_[0], 1);
  fn(obj, obj->rx.fdFields_[1], l14);
  fn(obj, obj->rx.fdFields_[2], v);
  fn(obj, obj->rx.fdFields_[3], v);
  fn(obj, obj->rx.fdFields_[4], 1);
  fn(obj, obj->rx.fdFields_[5], l14);
  fn(obj, obj->rx.fdFields_[6], v);
  fn(obj, obj->rx.fdFields_[7], v);
  fn(obj, obj->rx.fdFields_[8], 1);
  fn(obj, obj->rx.fdFields_[9], l14);
  fn(obj, obj->rx.fdFields_[10], v);
  fn(obj, obj->rx.fdFields_[11], v);
  fn(obj, obj->rx.fdFields_[12], 1);
  fn(obj, obj->rx.fdFields_[13], l18);
  fn(obj, obj->rx.fdFields_[14], v);
  fn(obj, obj->rx.fdFields_[15], v);
}

/* ====================================================================== */
/* shellDemapper                                                          */
/* ====================================================================== */

/**
 * @brief Compute the shell-mapping index of a frame from its eight ring numbers.
 * @param obj  V.34 module object; the eight parameters w0..w7 are the 16-bit words `ringNo_[0..7]` (w2 and w6 are read signed, the rest unsigned)
 * @return the 32-bit index (0 when the shell mapper size `w0a12_ - 1` is 0)
 * @note   The tables are X = `shellX_`, Y = `shellY_` and Z = `shellZ_` of the block.  The result is
 *         Y[B] * (X[C] * s2 + t2 + acc2) + (X[A] * s1 + t1 + acc1) + acc3 + Z[E], where acc1, acc2 and acc3 are the convolution sums
 *         sum_{i < A} X[i] * X[B - i], sum_{i < C} X[i] * X[D - i] and sum_{i < B} Y[i] * Y[E - i], with A = w0 + w1, B = A + w2 + w3, C = w4 + w5,
 *         D = C + w6 + w7 and E = B + D (16-bit sums); s1, t1, s2, t2 are limited by the size N = w0a12_ - 1.  The products wrap at 32 bits; the
 *         loop counters are signed 16-bit and the loops run while the counter is above 0.
 * @warning X, Y and Z are indexed with values derived from the frame parameters (up to B, E, ...) without a range check; the parameters of a valid
 *          frame keep them below 128.
 */
int shellDemapper(V34Obj* obj)
{
  const uint16_t* X = obj->rx.blkb.shellX_;
  const uint16_t* Y = obj->rx.blkb.shellY_;
  const uint32_t* Z = obj->rx.blkb.shellZ_;
  uint32_t N = (uint16_t)(obj->rx.blkb.w0a12_ - 1u);
  uint32_t w0, w1, w3, w4, w5, w7;
  int32_t w2, w6;
  uint32_t A, B, C, D, E;
  int32_t s1, t1, s2, t2;
  uint32_t acc1, acc2, acc3, part1, part2;
  const uint16_t *lo, *hi;
  int16_t cnt;

  if (N == 0)
    return 0;

  /* the eight frame parameters */
  w0 = obj->rx.ringNo_[0];
  w1 = obj->rx.ringNo_[1];
  w2 = (int16_t)obj->rx.ringNo_[2];
  w3 = obj->rx.ringNo_[3];
  w4 = obj->rx.ringNo_[4];
  w5 = obj->rx.ringNo_[5];
  w6 = (int16_t)obj->rx.ringNo_[6];
  w7 = obj->rx.ringNo_[7];

  /* first half-frame: the sums A and B and the limited remainder terms s1, t1 */
  A = (uint16_t)(w0 + w1);
  B = (uint16_t)(w0 + w1 + (uint32_t)w2 + w3);
  s1 = ((int32_t)(B - A) >= (int32_t)N) ? (int32_t)N - (int16_t)w3 : w2;
  t1 = ((uint16_t)A >= (uint16_t)N) ? (int32_t)N - (int16_t)w1 : (int16_t)w0;

  /* acc1 = sum_{i<(short)A} X[i] * X[B-i] */
  acc1 = 0;
  lo = X;
  hi = X + B;
  for (cnt = (int16_t)A; cnt > 0; cnt = (int16_t)(cnt - 1))
  {
    uint32_t h = *hi;
    hi--;
    uint32_t l = *lo;
    lo++;
    acc1 += l * h;
  }
  part1 = (uint32_t)X[A] * (uint32_t)s1 + (uint32_t)t1; /* X[A]*s1 + t1 */

  /* second half-frame: C and D and the terms s2, t2 */
  C = (uint16_t)(w4 + w5);
  D = (uint16_t)(w4 + w5 + (uint32_t)w6 + w7);
  s2 = ((int32_t)(D - C) >= (int32_t)N) ? (int32_t)N - (int16_t)w7 : w6;
  t2 = ((uint16_t)C < (uint16_t)N) ? (int16_t)w4 : (int32_t)N - (int16_t)w5;

  /* acc2 = sum_{i<(short)C} X[i] * X[D-i]   (low pointer restarts at X[0]) */
  acc2 = 0;
  lo = X;
  hi = X + D;
  for (cnt = (int16_t)C; cnt > 0; cnt = (int16_t)(cnt - 1))
  {
    uint32_t h = *hi;
    hi--;
    uint32_t l = *lo;
    lo++;
    acc2 += l * h;
  }
  part2 = (uint32_t)X[C] * (uint32_t)s2 + (uint32_t)t2; /* X[C]*s2 + t2 */

  /* combine the two halves through the Y and Z tables */
  E = (uint16_t)(B + D);

  /* acc3 = sum_{i<(short)B} Y[i] * Y[E-i] */
  acc3 = 0;
  lo = Y;
  hi = Y + E;
  for (cnt = (int16_t)B; cnt > 0; cnt = (int16_t)(cnt - 1))
  {
    uint32_t h = *hi;
    hi--;
    uint32_t l = *lo;
    lo++;
    acc3 += l * h;
  }

  return (int)((uint32_t)Y[B] * (part2 + acc2) + (acc1 + part1) + acc3 + Z[E]);
}

/* ====================================================================== */
/* decodeDepth                                                            */
/* ====================================================================== */

/* c = ((x / n + (x > 0)) & ~3) + 1 as a signed 16-bit value: the lattice point (spacing 4 n) below / nearest x, from the signed quotient by n */
static int16_t dd_candidate(int16_t x, int n)
{
  int q = (int)x / n;
  if (x > 0)
    q++;
  q &= ~3;
  return (int16_t)(q + 1);
}

/* +/-2 correction of a candidate: += 2 when x > n*c (signed 32-bit compare), else -= 2 */
static int16_t dd_adjust(int16_t c, int16_t x, int n) { return (int16_t)(c + ((int)x > n * (int)c ? 2 : -2)); }

/**
 * One half-step of decodeDepth(): remove the precoder contribution of one 2-D point.
 * The two six-tap dot products of the six-entry history `fbHist_` with the coefficient rows at `p0a24_` (rows 12 bytes apart) are
 * rounded to the multiple of the quantiser step Q (`Q` is w0a46_ sign-extended), giving the corrections *u_a and *u_b (u_a: full 16 bits;
 * u_b: only the low 16 bits and bit 1 are used later).  The history is then shifted by two entries, and the two new entries (fbHist_[0], [1]) are the
 * candidates cf, cs scaled by 128 minus the unrounded sums (r1, r2).
 */
static void dd_half(V34Obj* obj, int Q, int16_t cf, int16_t cs, uint16_t* u_a, uint32_t* u_b)
{
  const int16_t* P = (const int16_t*)obj->rx.blkb.p0a24_;
  const int16_t* h = obj->rx.blkb.fbHist_;
  uint32_t A = 0x1fff, B = 0x1fff;
  int32_t m4 = (int16_t)(-2 * Q);           /* sign-extended (short)(-2*Q) */
  int32_t q7 = (int16_t)((uint32_t)Q << 7); /* sign-extended (short)(Q<<7) */
  uint32_t mask = (uint32_t)q7 | 0xffff80ffu;
  uint32_t e;
  uint16_t r1, r2;
  int i;

  /* the two six-tap dot products (rounding constant 0x1fff) */
  for (i = 0; i < 6; i++)
  {
    int32_t hv = h[i];
    A += (uint32_t)((int32_t)P[i] * hv);
    B += (uint32_t)((int32_t)P[i + 6] * hv);
  }

  /* round each sum to a multiple of the quantiser step */
  if ((int32_t)A < 0)
    A += 1;
  r1 = (uint16_t)(A >> 14);
  e = (uint32_t)(int32_t)(int16_t)r1;
  if ((uint16_t)(r1 & mask) != (uint16_t)q7)
    e += (uint32_t)q7;
  *u_a = (uint16_t)((e >> 7) & (uint32_t)m4);

  if ((int32_t)B < 0)
    B += 1;
  r2 = (uint16_t)(B >> 14);
  e = (uint32_t)(int32_t)(int16_t)r2;
  if ((uint16_t)(r2 & mask) != (uint16_t)q7)
    e += (uint32_t)q7;
  *u_b = (e >> 7) & (uint32_t)m4; /* only the low 16 bits and bit 1 are used later */

  /* history shift, then the two new residuals */
  {
    int16_t* hist = obj->rx.blkb.fbHist_;
    for (i = 5; i >= 2; i--)
      hist[i] = hist[i - 2];
    hist[0] = (int16_t)(((uint32_t)(int32_t)cf << 7) - r1);
    hist[1] = (int16_t)(((uint32_t)(int32_t)cs << 7) - r2);
  }
}

/**
 * Map a corrected (hi, lo) coordinate pair to its constellation rank: the pair is first rotated by the lookup value k (0: 23*hi + lo, 1: 23*lo - hi,
 * 2: -23*hi - lo, 3: -23*lo + hi) into a single 16-bit lattice number v, then grid[(v + 0x408) >> 2] is read.
 * The index is not range checked (see decodeDepth()).
 */
static int16_t dd_grid(int k, uint16_t lo16, uint16_t hi16)
{
  int32_t hi = (int16_t)hi16;
  int32_t lo = (int16_t)lo16;
  int16_t v;
  if (k == 2)
    v = (int16_t)(-23 * hi - lo);
  else if (k == 3)
    v = (int16_t)(-23 * lo + hi);
  else if (k == 1)
    v = (int16_t)(23 * lo - hi);
  else
    v = (int16_t)(23 * hi + lo); /* k == 0 and anything else */
  return (int16_t)GRID_AT(((int32_t)v + 0x408) >> 2);
}

/**
 * @brief Trace the 32-stage trellis back from the newest stage and demap the oldest stage into the fields of one four-dimensional symbol.
 * @param obj   V.34 module object
 * @param out2  receives four 16-bit fields: [0] the parity bit ((k2 - k1) & 2) >> 1, [1] the 2-bit differential (k1 - previous k) & 3,
 *              [2] and [3] the low `w0a14_` bits of the ranks of the first and second 2-D point
 * @param out5  receives two 5-bit ring numbers: [0] and [1] the rank of the first and second 2-D point shifted right by `w0a14_`
 * @note   The trace follows the signed predecessor byte of the trellis memory for 31 steps, ending at the stage `fdStage_ - 1` minus 31 (mod 32).
 *         The subset-pair value of that stage selects which coordinates of its two received points need the +-2 correction; each point is then run
 *         through dd_half() (the second point sees the history left by the first), the k lookups are made through kLookup[], and the ranks come
 *         from dd_grid().  w0a38_ remembers k1 for the next call.
 * @bug    dd_grid() reads grid[529] at (v + 0x408) >> 2 without a range check.  For points on the 23x23 constellation lattice the index is in
 *         range; received points far outside it (about 44 times the lattice scale and beyond) read outside the table (docs/bugs.md).
 * @warning The trace index (stage * 16 + state) is built from signed predecessor bytes and not checked; it must stay in 0 .. 511 (WALK_IDX()).
 */
void decodeDepth(V34Obj* obj, short* out2, unsigned short* out5)
{
  int n = ((V34Obj*)obj)->rx.blkb.w0a42_;
  uint32_t st = ((uint32_t)obj->rx.fdStage_ - 1u) & 0x1f;
  TrellisMem* tr = &obj->rx.trellis_;
  int state = tr->stage[st].state;
  int i;
  unsigned int sym;
  int16_t x0, y0, x1, y1;
  int16_t cf, cs, cf2, cs2;
  uint16_t ua, ub2;
  uint32_t ub;
  int Q, k1, k2, idx;
  uint16_t lo16, hi16;
  int16_t g;
  unsigned int sh;

  if (n == 0)
    n = 1;

  /* trace back through the trellis memory: each word holds the predecessor state in its high byte */
  for (i = 0; i < 31; i++)
  {
    int walkIdx = WALK_IDX((int)(st << 4) + state);
    st = (st - 1u) & 0x1f;
    state = (int8_t)(tr->node[walkIdx] >> 8); /* the high byte: the predecessor state */
  }
  sym = tr->node[WALK_IDX((int)((st << 4) + state))] & 0xff; /* the low byte: the subset-pair value */

  /* first point of the oldest stage: nearest lattice candidates, then the +-2 corrections the subset pair asks for */
  x0 = tr->stage[st].pt[0][0];
  y0 = tr->stage[st].pt[0][1];
  cf = dd_candidate(x0, n);
  cs = dd_candidate(y0, n);
  {
    unsigned int sel = sym >> 2;
    if (sel == 2)
    {
      cf = dd_adjust(cf, x0, n);
      cs = dd_adjust(cs, y0, n);
    }
    else if (sel == 1)
    {
      cs = dd_adjust(cs, y0, n);
    }
    else if (sel == 3)
    {
      cf = dd_adjust(cf, x0, n);
    }
  }

  Q = ((V34Obj*)obj)->rx.blkb.w0a46_;
  dd_half(obj, Q, cf, cs, &ua, &ub);

  /* first lookup: (sym & 0xc) | bit1(ua) | bit1(ub) */
  idx = (int)((sym & 0xc) | (ua & 2u) | ((ub & 2u) >> 1));
  k1 = kLookup[idx];

  /* second point of the oldest stage */
  x1 = tr->stage[st].pt[1][0];
  y1 = tr->stage[st].pt[1][1];
  cf2 = dd_candidate(x1, n);
  cs2 = dd_candidate(y1, n);
  {
    unsigned int sel = sym & 3;
    if (sel == 2)
    {
      cf2 = dd_adjust(cf2, x1, n);
      cs2 = dd_adjust(cs2, y1, n);
    }
    else if (sel == 1)
    {
      cs2 = dd_adjust(cs2, y1, n);
    }
    else if (sel == 3)
    {
      cf2 = dd_adjust(cf2, x1, n);
    }
  }

  /* results of the FIRST half must be captured before the second half runs */
  lo16 = (uint16_t)(cf - ua);
  hi16 = (uint16_t)(cs - ub);
  {
    /* second half (re-reads the already updated history) */
    uint16_t ua2;
    uint32_t ub2w;
    dd_half(obj, Q, cf2, cs2, &ua2, &ub2w);
    ub2 = (uint16_t)ub2w;
    idx = (int)(((sym & 3) << 2) | (ua2 & 2u) | ((ub2w & 2u) >> 1));
    k2 = kLookup[idx];

    /* the parity bit and the differential value */
    out2[0] = (short)((((k2 - k1) & 2) >> 1));
    out2[1] = (short)((k1 - obj->rx.blkb.w0a38_) & 3);
    obj->rx.blkb.w0a38_ = (uint16_t)k1;

    /* ranks of the two points, split into ring number and low bits */
    sh = obj->rx.blkb.w0a14_ & 0x1f;
    g = dd_grid(k1, lo16, hi16);
    out5[0] = (uint16_t)(((int)g >> sh) & 0x1f);
    {
      int16_t mask = (int16_t)((1u << (obj->rx.blkb.w0a14_ & 0x1f)) - 1u);
      out2[2] = (short)(uint16_t)((int)g & (int)mask);

      lo16 = (uint16_t)(cf2 - ua2);
      hi16 = (uint16_t)(cs2 - ub2);
      g = dd_grid(k2, lo16, hi16);
      out5[1] = (uint16_t)(((int)g >> (obj->rx.blkb.w0a14_ & 0x1f)) & 0x1f);
      out2[3] = (short)(uint16_t)((int)g & (int)mask);
    }
  }
}

/* ====================================================================== */
/* demapFrame                                                             */
/* ====================================================================== */

/**
 * @brief Process one received 2-D point in frame mode: store it (even index) or make one trellis step (odd index).
 * @param obj      V.34 module object
 * @param sym      the received point, packed as two 16-bit coordinates in one 32-bit word
 * @param out2     receives the decided point (odd index only): out2[0] and out2[1] are the two coordinates of the second point of the stage
 * @param idx_arg  the symbol counter; only the low 16 bits are used (sign-extended)
 * @return 0 for an even index (decodeDepth() ran and its fields were stored), 1 for an odd index
 * @note   Even index: decodeDepth() writes its four fields at `fdFields_[(idx & 7) / 2 * 4]` and its two ring numbers at
 *         `ringNo_[idx & 7]`; the point is then stored as the first point of the stage record `fdStage_`.
 *         Odd index: the point is stored as the second point of the stage record and one step of the trellis is made.  For both points, the nearest
 *         and the next candidate of each coordinate are found, the squared distances (shifted right by `w0a44_`) are summed per candidate pair, and
 *         for each of the eight subset pairs of the transition table (kkNormal / kkInvert, chosen by `w0a3a_`) the better of the two sums is kept.
 *         The add-compare-select loop then makes the 16 new path metrics from kTable[] (four predecessors per state, the first strictly lowest
 *         wins), stores the surviving predecessor and subset-pair value, renormalises the metrics to the smallest, and picks the best end state
 *         (the last state whose metric is 0).  The output pair is the decided candidate of the second point of that state.
 *         Afterwards `fdStage_` advances (mod 32) and the counters `w0a3c_` / `w0a3e_` step the inversion-pattern selector `w0a3a_`
 *         (gInvertPat[]).  When (idx & 7) == 7, shellDemapper() and putFrame() run: every time once `w0e4c_` is set, otherwise only for idx above
 *         0x40, and `w0e4c_` is set when idx reaches N * 8 + 0x40.
 * @warning `gInvertPat[]` is indexed with the next value of `w0a3e_`, which is only tested against `w0a40_`; a counter that is negative as a
 *          signed 16-bit value, or a `w0a40_` above 16, reads outside the 16-entry table.
 * @warning When all four candidate sums of a state reach 0x7fff (unsigned 16-bit), the best-predecessor index of the previous state is reused
 *          (it is not reset between states).  With normalised metrics this needs saturated distances.
 */
int demapFrame(V34Obj* obj, const uint32_t* sym, int16_t* out2, int16_t idx_arg)
{
  int idx = idx_arg;
  int u9 = idx & 7;
  int n = ((V34Obj*)obj)->rx.blkb.w0a42_;

  if (n == 0)
    n = 1;

  /* even index: demap the oldest stage and keep the new point */
  if ((idx & 1) == 0)
  {
    decodeDepth(obj, (short*)&obj->rx.fdFields_[(u9 >> 1) * 4],
                &obj->rx.ringNo_[u9]); /* u9 = idx & 7 is even here: the groups and ring pairs stay inside the arrays */
    obj->rx.trellis_.stage[(int16_t)obj->rx.fdStage_].pair[0] = *sym;
    return 0;
  }

  {
    int16_t cand[16];     /* [0..7] candidates, [8..15] = the chosen subset-pair value of each of the eight pairs */
    uint16_t pairBest[8]; /* per-pair best metric */
    uint16_t dist[16];    /* distances, then the 16 new metrics */
    int16_t* R4;
    int stage;
    int i, j, k;
    int sh;
    const short* kk;

    /* odd index: store the second point of the stage and fetch both points */
    obj->rx.trellis_.stage[(int16_t)obj->rx.fdStage_].pair[1] = *sym;
    stage = (int16_t)obj->rx.fdStage_;
    R4 = &obj->rx.trellis_.stage[stage].pt[0][0];

    /* candidate lattice points: for each of the 2 points (x, y) 4 shorts */
    for (j = 0; j < 2; j++)
    {
      for (i = 0; i < 2; i++)
      {
        int16_t x = R4[2 * j + i];
        int q = (int)x / n;
        int16_t c;
        if (x > 0)
          q++;
        q &= ~3;
        c = (int16_t)((q + 1) * n);
        cand[4 * j + 2 * i] = c;
        cand[4 * j + 2 * i + 1] = (int16_t)(c + ((int)x > (int)c ? 2 * n : -2 * n));
      }
    }

    /* squared distances (arithmetic shift by w0a44_ & 31 of the int32 low product) */
    sh = (int16_t)obj->rx.blkb.w0a44_ & 31;
#define DSQ(a, b) ((uint16_t)((int32_t)((uint32_t)((int)(a) - (int)(b)) * (uint32_t)((int)(a) - (int)(b))) >> sh))
    for (j = 0; j < 2; j++)
    {
      int rx = R4[2 * j], ry = R4[2 * j + 1];
      const int16_t* L = &cand[4 * j];
      uint16_t d0 = DSQ(rx, L[0]);
      uint16_t d1 = DSQ(ry, L[2]);
      uint16_t d2 = DSQ(ry, L[3]);
      uint16_t d3 = DSQ(rx, L[1]);
      dist[4 * j + 0] = (uint16_t)(d1 + d0);
      dist[4 * j + 1] = (uint16_t)(d2 + d0);
      dist[4 * j + 2] = (uint16_t)(d2 + d3);
      dist[4 * j + 3] = (uint16_t)(d1 + d3);
    }
#undef DSQ

    /* kk pairs: best of two candidate sums per row */
    kk = (((V34Obj*)obj)->rx.blkb.w0a3a_ == 0) ? kkNormal : kkInvert;
    for (i = 0; i < 8; i++)
    {
      int v0 = kk[2 * i];
      int v1 = kk[2 * i + 1];
      uint16_t a = (uint16_t)(dist[v0 >> 2] + dist[4 + (v0 & 3)]);
      int16_t b = (int16_t)(dist[v1 >> 2] + dist[4 + (v1 & 3)]);
      pairBest[i] = a;
      cand[8 + i] = (int16_t)v0;
      if ((int)b < (int)a)
      {
        pairBest[i] = (uint16_t)b;
        cand[8 + i] = (int16_t)v1;
      }
    }

    /* add-compare-select over the 16 states s = 15..0 */
    {
      uint16_t* M = obj->rx.trellis_.metric;
      uint16_t* tw = &obj->rx.trellis_.node[stage * 16];
      uint16_t minv = 0x7fff;
      int best_k = 3; /* NOT reset per iteration (see the @warning above) */
      const short* kt = kTable;
      int s, c_last;
      const uint16_t* lp;
      uint16_t* mw;

      for (s = 15; s >= 0; s--)
      {
        int s4 = s >> 2;
        const uint16_t* mp = M + (3 - s4);
        uint16_t best = 0x7fff;
        int t;
        for (k = 3; k >= 0; k--)
        {
          uint16_t sum = (uint16_t)(*mp + pairBest[*kt]);
          kt++;
          mp += 4;
          if (sum < best)
          {
            best = sum;
            best_k = k;
          }
        }
        dist[15 - s] = best;
        if (best < minv)
          minv = best;
        t = kt[-1 - best_k];
        *tw++ = (uint16_t)(((((-4 * best_k) - s4) + 0xf) << 8) + (uint16_t)cand[8 + t]);
      }

      /* normalise the metrics; the LAST zero (smallest state counter) selects the end state */
      c_last = -1;
      lp = dist;
      mw = M;
      for (s = 15; s >= 0; s--)
      {
        uint16_t v = (uint16_t)(*lp++ - minv);
        *mw++ = v;
        if (v == 0)
          c_last = s;
      }
      {
        int sv = (int16_t)(0xf - c_last);
        int st2 = (int16_t)obj->rx.fdStage_;
        int w, u14;

        obj->rx.trellis_.stage[st2].state = (int16_t)sv;
        w = (int16_t)obj->rx.trellis_.node[(int16_t)obj->rx.fdStage_ * 16 + sv];
        u14 = ((uint32_t)w >> 1) & 1;
        out2[0] = cand[4 + u14];
        w = (int16_t)obj->rx.trellis_.node[(int16_t)obj->rx.fdStage_ * 16 + sv];
        out2[1] = cand[6 + (u14 ^ (w & 1))];
      }
    }

    obj->rx.fdStage_ = (uint16_t)((obj->rx.fdStage_ + 1u) & 0x1f);

    /* selector / counter bookkeeping for the NEXT odd call */
    {
      int16_t c = (int16_t)(obj->rx.blkb.w0a3c_ + 1);
      if ((int)c < (int)obj->rx.blkb.w0a02_)
      {
        obj->rx.blkb.w0a3c_ = (uint16_t)c;
        obj->rx.blkb.w0a3a_ = 0;
      }
      else
      {
        uint16_t d;
        obj->rx.blkb.w0a3c_ = 0;
        d = (uint16_t)(obj->rx.blkb.w0a3e_ + 1);
        if ((int16_t)d < ((V34Obj*)obj)->rx.blkb.w0a40_)
        {
          obj->rx.blkb.w0a3e_ = d;
          obj->rx.blkb.w0a3a_ = GINV_AT((int16_t)d);
        }
        else
        {
          obj->rx.blkb.w0a3e_ = 0;
          obj->rx.blkb.w0a3a_ = 0;
        }
      }
    }

    /* the end of a mapping frame: shell index and delivery of the bits */
    if (u9 == 7)
    {
      if (obj->rx.blkb.wE4c_ == 0)
      {
        if (idx > 0x40)
        {
          obj->rx.rxShellIdx_ = (uint32_t)shellDemapper(obj);
          putFrame(obj);
          if (idx >= (int)obj->rx.blkb.w0a00_ * 8 + 0x40)
            obj->rx.blkb.wE4c_ = 1;
        }
      }
      else
      {
        obj->rx.rxShellIdx_ = (uint32_t)shellDemapper(obj);
        putFrame(obj);
      }
    }
    return 1;
  }
}

/* ====================================================================== */
/* decoderv34                                                             */
/* ====================================================================== */

/* the 16-bit-aliasing view of the packed (I,Q) slicer points */
typedef int16_t sl_t __attribute__((may_alias));

/**
 * @brief Decide one received symbol.
 * @param obj  V.34 module object; reads the received point (`w0474_`, `w0476_`), writes the decided point (`decPt_` or the rxvect4 word)
 *             and, in the 4-point mode, the 2-bit descrambled value `w038a_`
 * @note   Frame mode (both bits of 0x98 set in `rxFlags_`): the symbol counter `w04ca_` is incremented, then demapFrame() runs with that
 *         counter value from before the increment.  After an even index (return 0) the decided point is cleared and flag 0x100 of `rxFlags_` is
 *         set, after an odd index (return 1) the flag is cleared.  If the receive quality word at 0x9fc is below -0x40 the flag is set again,
 *         and between -0x45 and -0x41 a "may be renegotiation" message is printed at debug level 2 and above.  `w047c_` is set to 0x2000.
 *         Otherwise: the received point is sliced to the nearest of the four rxvect4 points (squared distance >> 14), `w040e_` keeps the index of
 *         the decision, the difference to the previous one (mod 4) is descrambled through the 32-bit register `w0408_` (taps delayed by 5 or,
 *         with flag bit 2 of `rxFlags_`, by 18, and by 23) into the two bits stored at `w038a_`, and `w047c_` becomes 0x4000 when the word
 *         `w388_` equals half the receive baud index (rxBaud) and 0x2000 when it equals it (unchanged otherwise).
 */
void decoderv34(V34Obj* obj)
{
  uint16_t flags = obj->rx.rxFlags_;

  if ((flags & 0x98) == 0x98)
  {
    int16_t idx = ((V34Obj*)obj)->rx.w04ca_;
    int16_t r;

    obj->rx.w04ca_ = (int16_t)(idx + 1);
    r = (int16_t)demapFrame(obj, (const uint32_t*)&obj->rx.w0474_, obj->rx.decPt_, idx);
    if (r == 0)
    {
      obj->rx.decPt_[0] = 0;
      obj->rx.decPt_[1] = 0;
      obj->rx.rxFlags_ = (uint16_t)(obj->rx.rxFlags_ | 0x100);
    }
    else
    {
      obj->rx.rxFlags_ = (uint16_t)(obj->rx.rxFlags_ & 0xfeff);
    }
    {
      int16_t v = (int16_t)obj->rx.w09fc_;
      if (v < -0x40)
      {
        obj->rx.rxFlags_ = (uint16_t)(obj->rx.rxFlags_ | 0x100);
        if (v > -0x46 && 1 < dsplibs_debug_level)
          dsplibs_debug_printf("V34RENEG, may be renegotiation,equalizer adaptation disabled\n");
      }
    }
    obj->rx.w047c_ = 0x2000;
    return;
  }

  /* plain 4-point mode: nearest of the four rxvect4 points, differential value, descrambler */
  {
    const sl_t* pts = (const sl_t*)rxvect4;
    uint16_t Iu = obj->rx.w0474_, Qu = obj->rx.w0476_;
    int16_t minv = 0x7fff;
    int best = 0;
    int i;
    uint32_t s;
    uint16_t out = 0, m, prev, d;
    int j;

    for (i = 0; i < 4; i++)
    {
      int a = (int16_t)(Iu - (uint16_t)pts[2 * i]);
      int b = (int16_t)(Qu - (uint16_t)pts[2 * i + 1]);
      int16_t val = (int16_t)(((uint32_t)(a * a) + (uint32_t)(b * b)) >> 14);
      if (val < minv)
      {
        best = i;
        minv = val;
      }
    }

    s = obj->rx.w0408_;
    prev = obj->rx.w040e_;
    obj->rx.decPt_[0] = (int16_t)rxvect4[best]; /* the (I, Q) halves of the point word */
    obj->rx.decPt_[1] = (int16_t)((uint32_t)rxvect4[best] >> 16);
    obj->rx.w040e_ = (uint16_t)best;
    d = (uint16_t)((best - prev) & 3);

    for (j = 0, m = 1; j < 2; j++, m = (uint16_t)(m * 2))
    {
      uint32_t bit = (uint32_t)((m & d) != 0);
      uint32_t tap = (flags & 4) ? ((s >> 0x12) & 1u) : ((s >> 5) & 1u);
      uint32_t t = ((s >> 0x17) & 1u) ^ bit;
      s = (s + bit) * 2u;
      if (tap != t)
        out = (uint16_t)(out | m);
    }
    obj->rx.w0408_ = s;
    obj->rx.w038a_ = out;

    {
      uint16_t a = AS_U16(obj->rate.rxBaud_);
      uint16_t c = obj->rx.w388_;
      if (c == (uint16_t)((int16_t)a >> 1))
        obj->rx.w047c_ = 0x4000;
      else if (c == a)
        obj->rx.w047c_ = 0x2000;
    }
  }
}
