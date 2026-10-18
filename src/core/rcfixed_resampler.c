/**
 * @file rcfixed_resampler.c
 * @brief RcFixed: the fixed-ratio sample-rate converters (polyphase filters and the 4x interpolator / decimator).
 *
 * Entry points (declared with their documentation in rcfixed_resampler.h): RcFixed_Create(), RcFixed_Delete(), RcFixed_Reset(), RcFixed_Resample(),
 * RcFixed_Check_Combination().  The object is the typed RcFixed of rcfixed_resampler.h; the coefficient tables are ordinary const arrays
 * (rcfixed_resampler_tables.c).
 *
 * Type 0 (index 2 .. 19) is a polyphase FIR: the index selects up / down factors and a filter table (rc_filter[]).  Each output is the dot product of
 * `taps` history samples with the coefficient row `phase`, scaled by >> 14; then the phase advances by `down` and wraps modulo `up`, and every wrap
 * means one more input sample must be consumed (`skip`) before the next output.  Type 1 is a three-stage cascade of circular-buffer FIRs (3, 87 and 15
 * taps) that interpolates (mode 0, 4 outputs per input) or decimates (mode 1, 1 output per 4 inputs) by 4.
 *
 * Behaviours callers must know (documented where they apply; see rcfixed_resampler.h): a Create index 20 .. 999 crashes; the type 1 Reset / Create clear
 * only the first 4 bytes of each history block; the type 1 decimator drops its partial-group counters at the end of a call; the type 1 capacity test is
 * `>=`, so a capacity of 0 stops after one group; RcFixed_Check_Combination() divides by gcd(a, b).
 */
#include <stdlib.h>
#include "rcfixed_resampler.h"
#include "rcfixed_resampler_tables.h"
#include "core/dsp_asm.h"
#include "core/dsp_host.h"

/* Create index -> filter table and taps per phase.  Indices 0 / 1 are the type 1 converters (no entry).  Several indices share a filter table and differ
 * in the up / down factors (for example 2 = 8000 -> 9600 Hz and 4 = 8000 -> 48000 Hz both use rcfx_rc80to96filter). */
typedef struct
{
  const short* coef;
  short taps;
} RcFilterSel;
static const RcFilterSel rc_filter[20] = {
    {0, 0},
    {0, 0},
    {rcfx_rc80to96filter, 0x24},
    {rcfx_rc96to80filter, 0x20},
    {rcfx_rc80to96filter, 0x24},
    {rcfx_rc480to80filter, 0x44},
    {rcfx_rc96to80filter, 0x20},
    {rcfx_rc480to96filter, 0x44},
    {rcfx_rc96to384filter, 0x2e},
    {rcfx_rc384to96filter, 0x44},
    {rcfx_rc80to384filter, 0x28},
    {rcfx_rc384to80filter, 0x44},
    {rcfx_rc96to120filter, 0x1c},
    {rcfx_rc120to96filter, 0x3a},
    {rcfx_rc80to120filter, 0x2a},
    {rcfx_rc120to80filter, 0x30},
    {rcfx_rc96to320filter, 0x26},
    {rcfx_rc320to96filter, 0x44},
    {rcfx_rc72to80filter, 0x26},
    {rcfx_rc80to72filter, 0x20},
};

/* table size check: every filter holds up * taps shorts (up = rcfx_fixedRc_UpFact[idx]) */
#define RC_CHK(tab, up, taps) _Static_assert(sizeof(tab) == 2u * (up) * (taps), #tab)
RC_CHK(rcfx_rc80to96filter, 6, 0x24);
RC_CHK(rcfx_rc96to80filter, 5, 0x20);
RC_CHK(rcfx_rc480to80filter, 1, 0x44);
RC_CHK(rcfx_rc480to96filter, 1, 0x44);
RC_CHK(rcfx_rc96to384filter, 4, 0x2e);
RC_CHK(rcfx_rc384to96filter, 1, 0x44);
RC_CHK(rcfx_rc80to384filter, 24, 0x28);
RC_CHK(rcfx_rc384to80filter, 5, 0x44);
RC_CHK(rcfx_rc96to120filter, 5, 0x1c);
RC_CHK(rcfx_rc120to96filter, 4, 0x3a);
RC_CHK(rcfx_rc80to120filter, 3, 0x2a);
RC_CHK(rcfx_rc120to80filter, 2, 0x30);
RC_CHK(rcfx_rc96to320filter, 10, 0x26);
RC_CHK(rcfx_rc320to96filter, 3, 0x44);
RC_CHK(rcfx_rc72to80filter, 10, 0x26);
RC_CHK(rcfx_rc80to72filter, 9, 0x20);

/* ---- reset (shared by Create and Reset) ---- */

/* Type 0 reset: clear the history, restart the write position at `taps`, and set the initial phase down % up and skip (1 when up <= down, i.e. one
 * input sample is consumed before the first output). */
static void rc_reset_poly(RcPoly* p)
{
  short rem;
  sysdep_memset(p->hist, 0, 0x190);
  p->pos = p->taps;
  p->skip = (p->up <= p->down);
  rem = (short)((int)p->down % (int)p->up);
  p->phase = (rem < 0) ? (short)(rem + (unsigned short)p->up) : rem;
}

/* Type 1 reset: only the first 4 bytes of each history block are cleared (the blocks are 6, 0xae and 0x1e bytes); see RcFixed_Reset(). */
static void rc_reset_dec(RcDec* d)
{
  sysdep_memset(d->hist_a, 0, 4);
  sysdep_memset(d->hist_b, 0, 4);
  sysdep_memset(d->hist_c, 0, 4);
  d->idx_a = 0;
  d->idx_b = 0;
  d->idx_c = 0;
}

/**
 * @brief Create a converter (documented in rcfixed_resampler.h).
 *
 * Index 0 / 1: the 4x interpolator / decimator, with the three history blocks allocated here.  Index 2 .. 19: the polyphase converter with the filter and
 * factors of that index.
 */
RcFixed* RcFixed_Create(unsigned int idx)
{
  RcFixed* o;
  if (idx > 999)
    return NULL;
  o = (RcFixed*)sysdep_malloc(sizeof(RcFixed));
  o->state.any = NULL;
  if (idx <= 1)
  {
    RcDec* d;
    o->type = 1;
    d = (RcDec*)sysdep_malloc(sizeof(RcDec));
    o->state.dec = d;
    d->hist_a = (short*)sysdep_malloc(6);
    d->hist_b = (short*)sysdep_malloc(0xae);
    d->hist_c = (short*)sysdep_malloc(0x1e);
    d->mode = (short)idx;
    d->coef_a = rcfx_CI_bDroop;
    d->coef_b = rcfx_CI_b1;
    d->coef_c = rcfx_CI_b2;
    rc_reset_dec(d);
  }
  else if (idx <= 19)
  {
    RcPoly* p;
    o->type = 0;
    p = (RcPoly*)sysdep_malloc(sizeof(RcPoly));
    o->state.poly = p;
    p->up = (short)rcfx_fixedRc_UpFact[idx];
    p->down = (short)rcfx_fixedRc_DownFact[idx];
    p->coef = rc_filter[idx].coef;
    p->taps = rc_filter[idx].taps;
    rc_reset_poly(p);
  }
  else
  {
    /* index 20 .. 999: there is no state; the reset would clear the history of a NULL state, i.e. memset(NULL + 4, ...).  Written as a call so that the
     * fault happens in the memset (a crash by design, see the @bug on RcFixed_Create()). */
    o->type = 0;
    sysdep_memset((char*)0 + 4, 0, 0x190);
  }
  return o;
}

/** @brief Delete a converter (documented in rcfixed_resampler.h): the type 1 history blocks, the state, then the handle. */
void RcFixed_Delete(RcFixed* o)
{
  if (!o)
    return;
  if (o->state.any == NULL)
  {
    sysdep_free(o);
    NO_TAIL_CALL(); /* not a tail call: see the end of this function */
    return;
  }
  if (o->type == 1)
  {
    sysdep_free(o->state.dec->hist_a);
    sysdep_free(o->state.dec->hist_b);
    sysdep_free(o->state.dec->hist_c);
  }
  sysdep_free(o->state.any);
  sysdep_free(o);
  /* deliberately not a tail call: with strict aliasing on, the compiler would store the incoming argument back into its stack slot before a sibling call
   * (the same pattern as dcr_delete in dp_runtime.c), which would make the code of this function depend on the aliasing flags for no aliasing reason */
  NO_TAIL_CALL();
}

/** @brief Reset a converter (documented in rcfixed_resampler.h). */
void RcFixed_Reset(RcFixed* o)
{
  if (o->type == 0)
    rc_reset_poly(o->state.poly);
  else if (o->type == 1)
    rc_reset_dec(o->state.dec);
}

/**
 * @brief Find the converter index of the rate pair (a, b) (documented in rcfixed_resampler.h).
 *
 * Reduce the pair by gcd(a, b) and search the factor tables from index 2 until the table's 0 terminator for the entry with down == a / g and
 * up == b / g.
 */
int RcFixed_Check_Combination(int a, int b)
{
  int x = a, y = b, idx, ra, rb;
  while (y != 0)
  {
    int r = x % y;
    x = y;
    y = r;
  }
  rb = b / x;
  ra = a / x;
  idx = 2;
  while (rcfx_fixedRc_UpFact[idx] != 0)
  {
    if (rcfx_fixedRc_UpFact[idx] == rb && rcfx_fixedRc_DownFact[idx] == ra)
      break;
    idx++;
  }
  return idx;
}

/* ---- resample ---- */

/* circular index advance: idx = (idx + 1 <= last) ? idx + 1 : 0, in 16-bit arithmetic */
static void rc_adv(short* idx, int last)
{
  int v = (int)(unsigned short)*idx + 1;
  *idx = ((short)v <= last) ? (short)v : 0;
}

/* n-tap circular FIR: sum_j coef[j] * buf[(idx - j) mod n] (newest sample at idx, walking back and wrapping once), 32-bit wrap-around */
static unsigned rc_fir(const short* buf, const short* coef, int idx, int n)
{
  unsigned s = 0;
  const short* p = buf + idx;
  int i;
  for (i = idx; i >= 0; i--)
    s += (unsigned)((int)*p-- * (int)*coef++);
  p = buf + (n - 1);
  for (i = n - 1; i > idx; i--)
    s += (unsigned)((int)*p-- * (int)*coef++);
  return s;
}

/* Type 0 conversion.  Per iteration: take the `skip` input samples that the phase wrap asked for into the history (compacting the last `taps` samples to the
 * front when the 200-sample history is full), then, if enough input was available, produce one output from the current phase row and advance the phase.
 * A call that runs out of input in the middle of a skip keeps the remaining skip count for the next call. */
static void rc_resample_poly(RcPoly* S, const short* in, unsigned nIn, short* out, unsigned* pnOut)
{
  /* everything that stays constant over the call, and the output count, is held in locals: the short stores into hist[] / out[] would otherwise force the
   * compiler to reload the int / pointer members and *pnOut when it may not assume strict aliasing, so the code (not the result) would depend on
   * -fno-strict-aliasing */
  const short* const coefTab = S->coef;
  const int up = (int)(unsigned short)S->up;
  const int down = (int)(unsigned short)S->down;
  const short upS = S->up;
  const unsigned maxOut = *pnOut;
  unsigned nout = 0;
  *pnOut = 0;
  while (nIn != 0)
  {
    unsigned skip = (unsigned)S->skip;
    unsigned n = (skip <= nIn) ? skip : nIn;
    int pos = S->pos;
    unsigned i;
    int taps, e;
    unsigned sum;
    const short *c, *h;
    for (i = 0; i < n; i++)
    {
      S->hist[pos++] = *in++;
      if (pos == RC_HIST)
      {
        int t = S->taps, k;
        const short* src = S->hist + (RC_HIST - t);
        short* dst = S->hist;
        for (k = t; k != 0; k--)
          *dst++ = *src++;
        pos = t;
      }
    }
    S->pos = pos;
    if (nIn < skip)
    {
      S->skip = (int)(skip - nIn);
      return;
    }
    nIn -= skip;
    taps = S->taps;
    c = coefTab + (int)S->phase * taps;
    h = S->hist + (S->pos - taps);
    sum = 0;
    if (taps != 0)
    {
      int k;
      for (k = taps; k != 0; k--)
        sum += (unsigned)((int)*c++ * (int)*h++);
    }
    *out++ = (short)((int)sum >> 14);
    nout++;
    *pnOut = nout;
    S->skip = 0;
    e = (int)(unsigned short)S->phase + down;
    S->phase = (short)e;
    if (!(upS > (short)e))
    {
      int cnt = 0;
      do
      {
        cnt++;
        e -= up;
      } while ((short)e >= upS);
      S->phase = (short)e;
      S->skip = cnt;
    }
    if (nout == maxOut)
      return;
  }
}

/* 4x interpolation, three cascaded 2x steps per input sample: the 3-tap droop stage filters the input; the 87-tap stage takes that sample and a zero and
 * produces two samples; the 15-tap stage takes each of those followed by a zero and produces four outputs, each scaled by 0x7cd5 >> 13. */
static void rc_resample_up4(RcDec* S, const short* in, unsigned nIn, short* out, unsigned* pnOut, unsigned maxOut)
{
  /* the three history blocks / coefficient tables are loop invariants (cached: see rc_resample_poly), the output count is a local */
  short *const ha = S->hist_a, *const hb = S->hist_b, *const hc = S->hist_c;
  const short *const ca = S->coef_a, *const cb = S->coef_b, *const cc = S->coef_c;
  unsigned nout = 0;
  if (nIn == 0)
    return;
  for (;;)
  {
    int sA;
    unsigned y1;
    int y2;
    rc_adv(&S->idx_a, 2);
    ha[S->idx_a] = *in++;
    sA = (int)rc_fir(ha, ca, S->idx_a, 3) >> 15;
    rc_adv(&S->idx_b, 0x56);
    hb[S->idx_b] = (short)sA;
    y1 = rc_fir(hb, cb, S->idx_b, 0x57);
    rc_adv(&S->idx_b, 0x56);
    hb[S->idx_b] = 0;
    y2 = (int)rc_fir(hb, cb, S->idx_b, 0x57);

    rc_adv(&S->idx_c, 0xe);
    hc[S->idx_c] = (short)((y1 >> 14) & 0xfffe);
    out[0] = (short)((short)((int)rc_fir(hc, cc, S->idx_c, 0xf) >> 15) * 0x7cd5 >> 13);
    rc_adv(&S->idx_c, 0xe);
    hc[S->idx_c] = 0;
    out[1] = (short)((short)((int)rc_fir(hc, cc, S->idx_c, 0xf) >> 15) * 0x7cd5 >> 13);
    rc_adv(&S->idx_c, 0xe);
    hc[S->idx_c] = (short)((((unsigned)y2 * 4u) >> 16) & 0xfffe);
    out[2] = (short)((short)((int)rc_fir(hc, cc, S->idx_c, 0xf) >> 15) * 0x7cd5 >> 13);
    rc_adv(&S->idx_c, 0xe);
    hc[S->idx_c] = 0;
    out[3] = (short)((short)((int)rc_fir(hc, cc, S->idx_c, 0xf) >> 15) * 0x7cd5 >> 13);
    out += 4;
    nIn--;
    nout += 4;
    *pnOut = nout;
    if (nout >= maxOut)
      return;
    if (nIn == 0)
      return;
  }
}

/* 4x decimation: the 15-tap stage runs at the input rate; every second result goes through the 87-tap stage, and every second result of that through the
 * droop stage, whose output (scaled by 0x7eb8 >> 13) is the output sample.  The two stage counters cA / cB are locals, so they restart at 0 on every call. */
static void rc_resample_down4(RcDec* S, const short* in, unsigned nIn, short* out, unsigned* pnOut, unsigned maxOut)
{
  short *const ha = S->hist_a, *const hb = S->hist_b, *const hc = S->hist_c;
  const short *const ca = S->coef_a, *const cb = S->coef_b, *const cc = S->coef_c;
  unsigned nout = 0;
  int cA = 0, cB = 0;
  if (nIn == 0)
    return;
  for (;;)
  {
    short v;
    rc_adv(&S->idx_c, 0xe);
    hc[S->idx_c] = (short)(*in++ >> 1);
    v = (short)((int)rc_fir(hc, cc, S->idx_c, 0xf) >> 15);
    cA++;
    if ((short)cA == 2)
    {
      cA = 0;
      rc_adv(&S->idx_b, 0x56);
      hb[S->idx_b] = v;
      v = (short)((int)rc_fir(hb, cb, S->idx_b, 0x57) >> 15);
      cB++;
      if ((short)cB == 2)
      {
        cB = 0;
        rc_adv(&S->idx_a, 2);
        ha[S->idx_a] = v;
        v = (short)((int)rc_fir(ha, ca, S->idx_a, 3) >> 15);
        *out++ = (short)((int)v * 0x7eb8 >> 13);
        nout++;
        *pnOut = nout;
        if (nout >= maxOut)
          return;
      }
    }
    if (--nIn == 0)
      return;
  }
}

/**
 * @brief Convert a block of samples (documented in rcfixed_resampler.h): dispatch on the converter type.
 * @warning `o` and `pnOut` are not checked for NULL (a data pump without converters must not call this).  An object of an unknown type produces no output.
 */
void RcFixed_Resample(RcFixed* o, const short* in, unsigned int nIn, short* out, unsigned int* pnOut)
{
  if (o->type == 0)
  {
    rc_resample_poly(o->state.poly, in, nIn, out, pnOut);
  }
  else if (o->type == 1)
  {
    RcDec* S = o->state.dec;
    unsigned maxOut = *pnOut;
    *pnOut = 0;
    if (S->mode == 0)
      rc_resample_up4(S, in, nIn, out, pnOut, maxOut);
    else if (S->mode == 1)
      rc_resample_down4(S, in, nIn, out, pnOut, maxOut);
  }
  else
  {
    *pnOut = 0;
  }
}
