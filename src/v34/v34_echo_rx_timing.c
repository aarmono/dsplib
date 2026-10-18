/**
 * @file v34_echo_rx_timing.c
 * @brief V.34 echo canceller drivers, the receive/transmit sample queues and the receive symbol-timing loop.
 *
 * Entry points:
 * - updateAlpha(): step-size (alpha) update of an echo canceller from the delay-line energy.
 * - adaptecho(): one near-end-only echo-cancellation step (used by the V.PCM progress loop, one sample per call).
 * - modem_serrint(): per-sample receive front end and near / far echo-canceller driver: takes the next transmit-ring sample, cancels the echo,
 *   writes the receive ring in one of three formats and runs the two adaptation loops.  (The name does not mean a serial interrupt.)
 * - V34EchoHistoryBackwardClean(): zeroes the most recent samples of the pre-filter, both echo histories and the transmit history ring after the caller
 *   discarded output samples.
 * - rxreadqueue() / txwritequeue(): move blocks of 4 samples out of the receive ring and into the transmit history ring.
 * - rxtiming() / rxtiminginit(): receive symbol-timing loop: interpolates the demodulated pairs to the timing phase, high-pass filters the energy for
 *   the timing detector and runs a two-pole resonator on the pairs; rxtiminginit() resets the state it uses.
 *
 * All state lives in the V.34 object (V34Obj, v34_object_view.h) and its embedded sub-objects (receive V34Rx, transmit V34Tx, the near and far
 * V34Echo records, V34PreFilter), reached through their members.  The sample rings are:
 * - the receive ring: 4-byte {I, Q} entries between the ring start and `rxFskBuf_`, write pointer `p026c_`, read pointer `p0268_`, count `rxW0_`;
 * - the transmit history ring: 4-byte {sample, 0} entries from the ring base to `txCnt`, read pointer `p2220_`, write pointer `p2224_`, count `txIdx_`.
 *
 * ## Hazards
 * - updateAlpha() hangs for an energy of 0x80000000 and divides by zero for energies in [0xffff8000, UINT_MAX]; the same holds for the energy that
 *   adaptecho() and modem_serrint() feed it (the result of V34EchoEstimateDelayLineEnergy()).
 * - The delay `d` handed to V34EchoFilter() and the ring indices are not range-checked.  modem_serrint() performs the index store, the ring store and
 *   the compare in the order that makes even a negative index, which aliases the index word, behave as it always has.
 * - rxtiming() writes its results through the receive sink pointer, which V34demodulate() advances on every call; the caller runs 4 iterations, so
 *   at most about 8 calls happen.  A larger loop count (word +0x128 of the receive block) would walk the sink over the flag word and then over the
 *   loop count itself.
 */

#include <limits.h>
#include <stddef.h>
#include <stdint.h>
#include "core/dsp_host.h"
#include "v34_internal.h"

#include "v34_access.h" /* U16 / u16_t: the sample rings are byte memory */
#include "v34_object_view.h"

#include "v34_sar32.h"

/* ---- updateAlpha ---- */

/**
 * @brief Update an echo canceller's adaptation step size from the delay-line energy.
 * @param alpha   the step size (16-bit), updated in place
 * @param energy  delay-line energy; 0 leaves the step size unchanged apart from the optional decay
 * @param decay   non-zero to additionally scale the step size by `k2` / 2^15
 * @param k1      gain of the energy-derived step size, Q15
 * @param k2      decay factor, Q15
 * @param tag     "NE" or "FE", only used in the debug print
 *
 * For a non-zero energy x: normalise x by left shifts (n) until bit 30 is set to x', form q = 2^((n + 21) & 31) / ((x' + 0x8000) >> 16) (signed
 * division, truncated to 16 bits, a negative q becomes 0x2000) and store alpha = -((q * k1 + 0x2000) >> 15), i.e. a step size inversely proportional to
 * the energy; the old and new values are printed when the debug level exceeds 1.  Then, when `decay` is set, alpha = (alpha * k2 + 0x4000) >> 15.
 * @warning `energy == 0x80000000` loops forever (the normalisation shifts the single set bit out), and an energy in [0xffff8000, UINT_MAX] makes the
 *          divisor zero (division trap).  The energy comes from V34EchoEstimateDelayLineEnergy().
 */
void updateAlpha(short* alpha, uint32_t energy, int decay, int k1, int k2, const char* tag)
{
  int old = *alpha;

  if (energy != 0)
  {
    int n = 0;
    uint32_t xs = energy;
    int32_t dividend, divisor;
    int q;
    int16_t q16;
    int16_t nv;

    /* normalise */
    while ((xs & 0x40000000u) == 0)
    { /* never terminates for 0x80000000 */
      n = (int16_t)(n + 1);
      xs = xs + xs;
    }

    /* inverse of the energy */
    dividend = (int32_t)(1u << ((unsigned)(n + 0x15) & 31));
    divisor = sar32(xs + 0x8000u, 16);
    q16 = (int16_t)(dividend / divisor);
    q = q16;
    if (q16 < 0)
      q = 0x2000;
    nv = (int16_t)(-sar32((uint32_t)q * (uint32_t)k1 + 0x2000u, 15));
    *alpha = nv;
    if (dsplibs_debug_level > 1)
      dsplibs_debug_printf("updateAlpha%s: updated %d => %d\n", tag, old, (int)nv);
  }

  if (decay != 0)
  {
    *alpha = (int16_t)sar32((uint32_t)(int)*alpha * (uint32_t)k2 + 0x4000u, 15);
  }
}

/* ---- adaptecho ---- */

/**
 * @brief One near-end-only echo-cancellation step (called once per sample by the V.PCM progress loop).
 * @param obj  the V.34 object
 * @return always 0
 *
 * Takes the next sample from the transmit history ring (read pointer `p2220_`, advanced with wrap), counts it off `txIdx_` and forms the delay
 * d = (dmaDelay_ - old txIdx_) as a signed 16-bit value.  The signal f is the sum of the accumulator `w260_` and the carry word `wa23e_` (which is then
 * cleared); the near-echo filter output r is combined into the error e = f + (4 r + 0x8000) >> 16, which becomes the new accumulator.  When the echo is
 * frozen (`txFlags_` bit 4) nothing else happens.
 *
 * Otherwise the adaptation counter `echoAdaptCnt_` advances:
 * - up to 0x8f: the energy of f (f * f >> 6) is accumulated in `echoEnergy_`;
 * - from then on: every 10th count beyond `echoThresh_` sets the decay flag; at count 0x90 the delay-line energy is estimated and the gain `echoBeta_`
 *   is lowered to 4 or 5 or 2 depending on the accumulated energy (below 2500000, above 11000000, above 13000000, never raised); then updateAlpha() runs
 *   on `echoAlphaNear_` with k1 = 0x7999 and k2 = `echoK2_`.
 *
 * Finally the near record's adaptation counter `w80cc_` is incremented and, unless d is negative (an error print instead), V34EchoAdapt() adapts the
 * near filter with (alpha * beta * e + 0x2000) >> 14.
 */
int adaptecho(V34Obj* obj)
{
  V34Echo* ne = &obj->echo0;
  uint32_t cnt = obj->tx.txIdx_;
  uint32_t tgt = obj->dmaDelay_;
  uint32_t ff = obj->wa23e_;
  uint32_t acc = obj->w260_;
  int d = (int16_t)(tgt - cnt); /* sub in 32 bits, then CWDE */
  int f, e, r, n, arg;
  unsigned char *rd, *nx;

  /* next transmit-ring sample */
  obj->tx.txIdx_ = (uint16_t)(cnt - 1);
  f = (int16_t)(acc + ff); /* sum of the accumulator and the carry word, as a signed 16-bit value */
  rd = (unsigned char*)obj->tx.p2220_;
  obj->w25e_ = *(u16_t*)rd;
  nx = rd + 4;
  /* wrap unless the ring end is above the next entry (unsigned) */
  if (!((uintptr_t)&obj->tx.txCnt_ > (uintptr_t)nx))
    nx = obj->tx.unk2228_;
  obj->tx.p2220_ = nx;
  obj->wa23e_ = 0;

  /* echo estimate and error */
  r = V34EchoFilter(ne, (short)d);
  e = (int16_t)((uint32_t)f + (uint32_t)sar32((uint32_t)r * 4u + 0x8000u, 16));
  obj->w260_ = (uint16_t)e;
  if (obj->tx.txFlags_ & 4)
    return 0; /* echo frozen */

  /* adaptation schedule */
  n = (int32_t)((uint32_t)obj->echoAdaptCnt_ + 1u);
  obj->echoAdaptCnt_ = n;
  if (n <= 0x8f)
  { /* signed compare */
    obj->echoEnergy_ += (uint32_t)sar32((uint32_t)((int32_t)f * (int32_t)f), 6);
  }
  else
  {
    uint32_t energy = 0;
    int b3 = 0;

    if (n > obj->echoThresh_ && n % 10 == 0)
      b3 = 1;

    if (n == 0x90)
    {
      int beta;
      uint32_t e2;
      int c13;

      energy = (uint32_t)V34EchoEstimateDelayLineEnergy(ne);
      beta = obj->echoBeta_;
      e2 = obj->echoEnergy_;
      c13 = 0;
      if (beta > 4 && e2 <= 2499999u)
      {
        obj->echoBeta_ = 4;
        c13 = 1;
      }
      else if (beta > 5 && e2 > 11000000u)
      {
        obj->echoBeta_ = 5;
        c13 = 1;
      }
      else if (beta > 2)
      {
        c13 = 1;
      }
      if (c13 && obj->echoEnergy_ > 13000000u)
        obj->echoBeta_ = 2;
      if (dsplibs_debug_level > 1)
        dsplibs_debug_printf("Echo Energy = %d, BETA = %d\n", (int)obj->echoEnergy_, obj->echoBeta_);
    }

    updateAlpha(&obj->echoAlphaNear_, energy, b3, 0x7999, obj->echoK2_, "NE");
  }

  /* adapt the near filter */
  obj->echo0.w80cc_ = (uint16_t)(obj->echo0.w80cc_ + 1);
  arg = (int16_t)sar32((uint32_t)(int)obj->echoAlphaNear_ * (uint32_t)obj->echoBeta_ * (uint32_t)e + 0x2000u, 14);
  if (d < 0)
  {
    if (dsplibs_debug_level > 1)
      dsplibs_debug_printf("V90NEC, --------ERROR--------- occured in adaptecho\n");
  }
  else
  {
    V34EchoAdapt(ne, (short)arg);
  }
  return 0;
}

/* ---- modem_serrint ---- */

/**
 * @brief Per-sample receive front end and near / far echo-canceller driver (the name does not mean a serial interrupt).
 * @param obj  the V.34 object
 * @return always 0
 *
 * Steps:
 * 1. Take the next sample from the transmit history ring (as adaptecho()), pass the signal through a one-pole recursive filter (state `wa23e_`) and,
 *    when `txFlags_` bit 0x200 is set, add the echo estimates of the near filter and (when `echoOn_`) the far filter.  The result x is stored in the
 *    600-entry ring that starts in `constPts_` (index `w2aa6_`).
 * 2. Write x into the receive ring (write pointer `p026c_`, wrapping at `rxFskBuf_`) in the format selected by `rxFlags_`: bit 0x8000 stores {x, 0};
 *    bit 0x800 first replaces x by the output of the 60-tap FIR (coefficients `p0508_`, a 61-short delay line in the memory the far record's `p9144_`
 *    points to) and stores {x, 0}; otherwise the Hilbert filter produces the {I, Q} pair.
 * 3. When the echo is frozen (`txFlags_` bit 4) return.  Otherwise store {x, x} in the 300-entry ring (`constCnt_`), update the smoothed power `wa240_`,
 *    advance `echoAdaptCnt_` (when it reaches 0 print the start message) and run updateAlpha() for the near filter (energy at count 0x90) and, 700
 *    counts later, for the far filter (k1 0x6666 / 0x2b84, k2 0x7f5c); the decay flag is set on every 45th count beyond 2000.
 * 4. Increment the near (and far) records' adaptation counters and, unless the delay is negative (error print), adapt the near filter until the count
 *    reaches 18000 and the far filter (when `echoOn_`) from count 700 on.
 */
int modem_serrint(V34Obj* obj)
{
  V34Rx* rxobj = &obj->rx;
  V34Echo* ne = &obj->echo0;
  V34Echo* fe = &obj->echo1;
  uint32_t cnt = obj->tx.txIdx_;
  uint32_t tgt = obj->dmaDelay_;
  int d = (int16_t)(tgt - cnt);
  int F, s, r, x, i, j, c0, c1, sh, hz;
  int an = 0, af = 0;
  unsigned char *rd, *nx;
  unsigned char* wr;
  uint32_t fl;
  int16_t* ring = obj->tx.constPts_; /* the 600-entry and the 300-entry {x, x} rings start here */

  /* step 1: next transmit-ring sample */
  obj->tx.txIdx_ = (uint16_t)(cnt - 1);
  rd = (unsigned char*)obj->tx.p2220_;
  obj->w25e_ = *(u16_t*)rd;
  nx = rd + 4;
  if (!((uintptr_t)&obj->tx.txCnt_ > (uintptr_t)nx))
    nx = obj->tx.unk2228_;
  F = (int16_t)obj->w260_;
  s = (int16_t)((uint32_t)F + obj->wa23e_);
  obj->tx.p2220_ = nx;
  obj->wa23e_ = (uint16_t)sar32((uint32_t)s * 0xed8u - ((uint32_t)F << 12), 12);

  /* echo estimate */
  r = 0;
  if (obj->tx.txFlags_ & 0x200)
  { /* bit 0x200 of `txFlags_` */
    int nr = V34EchoFilter(ne, (short)d);
    int fr = 0;
    if (obj->echoOn_ != 0)
      fr = V34EchoFilter(fe, (short)d);
    r = sar32((uint32_t)nr * 4u + (uint32_t)fr * 4u + 0x8000u, 16);
  }
  x = (int16_t)((uint32_t)s + (uint32_t)r);

  /* 600-entry ring (index `w2aa6_`): store first, index after */
  i = obj->tx.w2aa6_;
  ring[600 + i] = (int16_t)x; /* i is not range-checked: a negative index reaches the words in front of constPts_ */
  {
    uint16_t nxt = (uint16_t)(i + 1);
    obj->tx.w2aa6_ = (nxt <= 0x257) ? nxt : 0;
  }

  /* step 2: receive ring */
  rxobj->rxW0_ = (uint16_t)(AS_U16(rxobj->rxW0_) + 1);
  fl = rxobj->rxFlags_;
  wr = (unsigned char*)rxobj->p026c_;
  if (fl & 0x8000u)
  {
    U16(wr, 0) = (uint16_t)x;
    U16(wr, 2) = 0;
  }
  else
  {
    if (fl & 0x800u)
    { /* FIR path: x is REPLACED by the FIR output */
      short* p = (short*)obj->echo1.p80c4_;
      const short* c = (const short*)rxobj->p0508_;
      uint32_t acc = 0x8000u;
      int m;

      p[0] = (short)x;
      for (m = 0; m < 0x3c; m++)
      {
        short v = p[59 - m];
        short cm = c[m];
        p[60 - m] = v;
        acc += (uint32_t)((int)cm * (int)v);
      }
      x = sar32(acc, 16);
      fl = rxobj->rxFlags_; /* re-read from memory after the loop */
    }

    if (fl & 0x800u)
    {
      U16(wr, 0) = (uint16_t)x;
      U16(wr, 2) = 0;
    }
    else
    {
      int h1, h2;

      V34HilbertFilter((short*)(obj->unk9158_ + 4192), (short)x, &h1, &h2);
      U16(wr, 0) = (uint16_t)sar32((uint32_t)h1 + 0x2000u, 14);
      U16(wr, 2) = (uint16_t)sar32((uint32_t)h2 + 0x2000u, 14);
    }
  }

  wr += 4;
  if (!((uintptr_t)rxobj->rxFskBuf_ > (uintptr_t)wr))
    wr = rxobj->unk0270_;
  rxobj->p026c_ = wr;
  if (obj->tx.txFlags_ & 4)
    return 0; /* echo frozen */

  /* step 3: 300-entry {x, x} ring (index `constCnt_`): index store, ring stores, then a compare that re-reads the index word from memory */
  j = obj->tx.constCnt_;
  obj->tx.constCnt_ = (uint16_t)(j + 1);
  ring[2 * j] = (int16_t)x; /* j is not range-checked: a negative index reaches the index word in front of constPts_, hence the compare below re-reads it */
  ring[2 * j + 1] = (int16_t)x;
  if (AS_U16(obj->tx.constCnt_) > 0x12b)
    obj->tx.constCnt_ = 0;

  /* smoothed power of x */
  {
    int P = obj->wa240_;
    c0 = obj->echoAdaptCnt_;
    obj->wa240_ = (uint16_t)((uint32_t)sar32((uint32_t)((int32_t)x * (int32_t)x), 10) + (uint32_t)sar32((uint32_t)P * 0x3f48u, 14));
  }

  /* adaptation schedule of the near and far filters */
  c1 = (int32_t)((uint32_t)c0 + 1u);
  obj->echoAdaptCnt_ = c1;
  if (c1 == 0)
  {
    if (dsplibs_debug_level > 1)
      dsplibs_debug_printf("V34NEC, Start NEC Adaptation\n");
  }
  else
  {
    uint32_t en = 0, ef = 0;
    int b13a, b13b, rem0;

    sh = (int16_t)((uint32_t)c0 - 0x2bbu); /* the far filter's count: 700 counts behind the near one */
    rem0 = (c1 % 45) == 0;                 /* the remainder is below 45 in magnitude, so a 16-bit test is exact */
    b13a = rem0 && (c1 > 2000);
    b13b = rem0 && (sh > 2000);

    if (c1 == 0x90)
      en = (uint32_t)V34EchoEstimateDelayLineEnergy(ne);
    if (sh == 0x90 && obj->echoOn_ != 0)
      ef = (uint32_t)V34EchoEstimateDelayLineEnergy(fe);
    updateAlpha(&obj->echoAlphaNear_, en, b13a, 0x6666, 0x7f5c, "NE");
    if (obj->echoOn_ != 0)
      updateAlpha(&obj->echoAlphaFar_, ef, b13b, 0x2b84, 0x7f5c, "FE");
  }

  /* step 4: adapt */
  hz = obj->echoOn_;
  obj->echo0.w80cc_ = (uint16_t)(obj->echo0.w80cc_ + 1);
  an = (int16_t)sar32((uint32_t)(int)obj->echoAlphaNear_ * (uint32_t)x * 2u + 0x2000u, 14);
  if (hz != 0)
  {
    obj->echo1.w80cc_ = (uint16_t)(obj->echo1.w80cc_ + 1);
    af = (int16_t)sar32((uint32_t)(int)obj->echoAlphaFar_ * (uint32_t)x * 2u + 0x2000u, 14);
  }

  if (d < 0)
  {
    if (dsplibs_debug_level > 1)
      dsplibs_debug_printf("V34NEC, --------ERROR--------- occured in modem_serrint\n");
    return 0;
  }

  {
    int c = obj->echoAdaptCnt_;
    if (c <= 17999)
    {
      V34EchoAdapt(ne, (short)an);
      hz = obj->echoOn_;
    }
    else if (c == 18000)
    {
      if (dsplibs_debug_level > 1)
        dsplibs_debug_printf("V34NEC - stop NEC adaptation\n");
      hz = obj->echoOn_;
    }
  }

  if (hz != 0)
  {
    int c2 = obj->echoAdaptCnt_;
    if (c2 > 700)
      V34EchoAdapt(fe, (short)af);
    else if (c2 == 700 && dsplibs_debug_level > 1)
      dsplibs_debug_printf("V34FEC - start FEC adaptation\n");
  }
  return 0;
}

/* ---- V34EchoHistoryBackwardClean ---- */

/**
 * @brief Zero the m most recent entries of an echo history.
 * @param echoobj  the echo record (V34Echo): write pointer `p80b8_`, base `p80bc_`, length (in shorts) `w80d0_`
 * @param m        number of entries to clear, walking backward from write pointer - 1 and wrapping from the base to base + length - 1
 * @warning The write pointer must lie in (base, base + length]; it is not checked.
 */
static void echo_clear_back(V34Echo* echoobj, int32_t m)
{
  unsigned char* wp = (unsigned char*)echoobj->p80b8_;
  unsigned char* base = (unsigned char*)echoobj->p80bc_;
  int32_t len = echoobj->w80d0_;
  unsigned char* p = wp - 2;

  if (wp == base)
    p = base + (uint32_t)len * 2u - 2u;
  if (m > 0)
  {
    do
    {
      *(u16_t*)p = 0;
      if (p == base)
        p = p + (uint32_t)len * 2u - 2u;
      else
        p -= 2;
    } while (--m != 0);
  }
}

/**
 * @brief Clear the history of the most recent samples after the caller discarded output samples.
 * @param obj  the V.34 object
 * @param n    number of samples to clear; the only caller passes three times the number of output samples it cleared
 *
 * - Pre-filter buffer (V34PreFilter, `cap` entries, current index at +0x58 as a signed 16-bit value): when n >= cap (unsigned) the whole buffer is
 *   zeroed; otherwise n entries are zeroed from the index, wrapping at the end.  With cap == 0 the first case is an empty memset for any n.
 * - Echo histories: when n exceeds half the threshold word of the pre-filter record (compared signed), the n - threshold/2 most recent entries of
 *   the near and far echo histories are zeroed (echo_clear_back()).
 * - The modulator's 0x100-byte history and its 16-short output-filter delay line are zeroed.
 * - The three most recently written entries of the transmit history ring ({sample, 0} entries, write pointer `p2224_`) have their sample zeroed;
 *   when the walk falls below the ring base (unsigned) the entry is taken from the top of the ring instead.
 * @warning The pre-filter index and capacity and the echo write pointers are not validated.
 */
void V34EchoHistoryBackwardClean(V34Obj* obj, uint32_t n)
{
  V34PreFilter* pfobj = &obj->mod.pf;  /* PreFilterState */
  unsigned char* pf = pfobj->unk2078_; /* its sample buffer: 2 * pfCap_ bytes, not checked against the 84 bytes of the member */
  uint32_t cap = pfobj->pfCap_;
  uint32_t t;

  /* pre-filter buffer */
  if (n >= cap)
  { /* unsigned compare */
    sysdep_memset(pf, 0, cap * 2u);
  }
  else
  {
    unsigned char* p = pf + 2 * (int)pfobj->pfIdx_; /* signed index */
    if ((int32_t)n > 0)
    {
      unsigned char* end = pf + cap * 2u;
      uint32_t cnt = n;
      do
      {
        *(u16_t*)p = 0;
        p += 2;
        if (p == end)
          p = pf;
      } while (--cnt != 0);
    }
  }

  /* echo histories */
  t = pfobj->pfLen2_ >> 1;
  if ((int32_t)n > (int32_t)t)
  {
    int32_t m = (int32_t)(n - t);
    echo_clear_back(&obj->echo0, m);
    echo_clear_back(&obj->echo1, m);
  }

  /* modulator histories */
  sysdep_memset(obj->mod.modHist_, 0, sizeof(obj->mod.modHist_));
  sysdep_memset(obj->mod.pf.pfDelay_, 0, sizeof(obj->mod.pf.pfDelay_));

  /* the three newest transmit history entries */
  {
    uintptr_t base = (uintptr_t)obj->tx.unk2228_;
    uintptr_t wp = (uintptr_t)obj->tx.p2224_ - 4;
    uintptr_t top = (uintptr_t)&obj->tx.txCnt_ - 4; /* the ring end - 4 */
    int k;

    for (k = 0; k < 3; k++)
    {
      uintptr_t cl = wp;
      if (!(wp >= base))
      { /* below the ring base (unsigned) */
        cl = top;
        top -= 4;
      }
      *(u16_t*)cl = 0;
      wp -= 4;
    }
  }
}

/* ---- rxreadqueue / txwritequeue ---- */

/**
 * @brief Move 4 samples from the receive ring into the 4-short work area `rxFskBuf_`.
 * @param rxobj  the receive sub-object (V34Rx)
 *
 * Decrements the pending count by 4 and copies the first (I) half of 4 consecutive 4-byte ring entries from the read pointer `p0268_`, which wraps
 * from the work-area address back to the ring start.
 */
void rxreadqueue(V34Rx* rxobj)
{
  unsigned char* rd = (unsigned char*)rxobj->p0268_;
  unsigned char* dst = rxobj->rxFskBuf_;
  int k;

  rxobj->rxW0_ = (uint16_t)(AS_U16(rxobj->rxW0_) - 4u);
  for (k = 0; k < 4; k++)
  {
    U16(dst, 0) = *(u16_t*)rd;
    rd += 4;
    dst += 2;
    if (!((uintptr_t)rxobj->rxFskBuf_ > (uintptr_t)rd))
      rd = rxobj->unk0270_;
  }
  rxobj->p0268_ = rd;
}

/**
 * @brief Append 4 samples to the transmit history ring.
 * @param txobj  the transmit sub-object (V34Tx)
 * @param src    4 samples
 *
 * Increments the ring count by 4 and stores the samples as {sample, 0} entries at the write pointer `p2224_`, which wraps from the ring end back to the
 * ring base.
 */
void txwritequeue(V34Tx* txobj, const short* src)
{
  unsigned char* wr = (unsigned char*)txobj->p2224_;
  int k;

  txobj->txIdx_ = (uint16_t)(txobj->txIdx_ + 4u);
  for (k = 0; k < 4; k++)
  {
    uint16_t sv = (uint16_t)src[k];
    U16(wr, 2) = 0;
    U16(wr, 0) = sv;
    wr += 4;
    if (!((uintptr_t)&txobj->txCnt_ > (uintptr_t)wr))
      wr = txobj->unk2228_;
  }
  txobj->p2224_ = wr;
}

/* ---- rxtiming / rxtiminginit ---- */

/**
 * @brief Run the two-pole resonator on the pair V34demodulate() just produced.
 * @param rxobj  the receive sub-object (V34Rx)
 * @param extra  non-zero (first post of the two-wrap path) to also store the two results as the "previous" pair used by the interpolation
 *
 * For the I channel A: n1 = (A * 1024 + P * 0x599b - Q * 0x3eba) >> 14 with P the previous result and Q the one before; Q takes P, and n1 replaces the
 * pair's I value and P.  The Q channel C does the same with its own two history words.
 */
static void rxt_post(V34Rx* rxobj, int extra)
{
  uint16_t P = AS_U16(rxobj->eqPt_[0]);
  int A = rxobj->w04a4_;
  int Q = (int16_t)rxobj->decPt_[0];
  uint16_t n1;
  uint16_t R, n2;
  int C, T;

  /* I channel */
  rxobj->decPt_[0] = P;
  n1 = (uint16_t)sar32((uint32_t)A * 1024u + (uint32_t)((int)(int16_t)P * 0x599b) + (uint32_t)(Q * -0x3eba), 14);
  C = rxobj->w04a6_;
  R = AS_U16(rxobj->eqPt_[1]);
  AS_U16(rxobj->w04a4_) = n1;
  AS_U16(rxobj->eqPt_[0]) = n1;
  if (extra)
    AS_U16(rxobj->w04a8_) = n1;

  /* Q channel */
  T = (int16_t)rxobj->decPt_[1];
  rxobj->decPt_[1] = R;
  n2 = (uint16_t)sar32((uint32_t)C * 1024u + (uint32_t)((int)(int16_t)R * 0x599b) + (uint32_t)(T * -0x3eba), 14);
  AS_U16(rxobj->w04a6_) = n2;
  AS_U16(rxobj->eqPt_[1]) = n2;
  if (extra)
    AS_U16(rxobj->w04aa_) = n2;
}

/**
 * @brief Receive symbol-timing loop: produce the loop-count timing-energy samples.
 * @param obj  the V.34 object
 *
 * Resets the demodulator sink to `rxFskBuf` and, for each iteration of the loop count (word +0x128 of the receive block): interpolates the current
 * and previous demodulated (I, Q) pairs to the
 * timing phase `w410_` (weights phase and `w414_` - phase, Q14), forms the energy x * x + y * y >> 14 and stores its high-pass filtered value in the
 * timing-energy array; then advances the phase by `w412_`.  When the phase reaches the limit `w414_` it wraps and one more input pair is demodulated
 * (V34demodulate()) and passed through rxt_post(); when it wraps twice (the phase step exceeds the limit) two pairs are consumed.
 * @warning The result array and the sink pointer have no bounds check (see the file header); the caller runs 4 iterations.  `smp_[1 + i]` is stored
 *          for i < w38c_ and `smp_` has 8 entries: a count above 7 writes over the members behind it (docs/bugs.md V79).
 */
void rxtiming(V34Obj* obj)
{
  V34Rx* rxobj = &obj->rx;
  int16_t* smp = rxobj->smp_; /* a pointer to the first element: the index is not bounded by the array (see the warning) */
  int i = 0;
  uint32_t pos;
  int d;

  rxobj->rxEnd_ = (const int16_t*)rxobj->rxFskBuf_;
  pos = rxobj->w410_;
  d = (int16_t)((uint32_t)rxobj->w414_ - pos);

  if ((int16_t)rxobj->w38c_ <= 0)
    return;

  do
  {
    int x, y, e;
    uint32_t sum;
    uint32_t s;
    int lims;

    /* interpolate to the timing phase and take the energy */
    x = (int16_t)sar32((uint32_t)((int)rxobj->w04a4_) * pos + (uint32_t)((int)rxobj->w04a8_) * (uint32_t)d + 0x2000u, 14);
    y = (int16_t)sar32((uint32_t)((int)rxobj->w04a6_) * pos + (uint32_t)((int)rxobj->w04aa_) * (uint32_t)d + 0x2000u, 14);
    e = (int16_t)sar32((uint32_t)(x * x) + (uint32_t)(y * y) + 0x2000u, 14);
    smp[1 + i] = (int16_t)V34TimingHPFilter(rxobj->unk050c_, (short)e);

    /* advance the phase; on a wrap consume one or two more input pairs */
    sum = (uint32_t)rxobj->w410_ + (uint32_t)rxobj->w412_;
    s = (uint16_t)sum;
    lims = (int16_t)rxobj->w414_;
    if ((int)s < lims)
    {
      rxobj->w410_ = (uint16_t)s; /* no wrap */
    }
    else
    {
      uint32_t lim = rxobj->w414_;
      uint32_t t = sum - lim;
      if ((int)(uint16_t)t < lims)
      { /* one wrap */
        rxobj->w410_ = (uint16_t)t;
        AS_U16(rxobj->w04aa_) = AS_U16(rxobj->w04a6_);
        AS_U16(rxobj->w04a8_) = AS_U16(rxobj->w04a4_);
      }
      else
      { /* two wraps */
        rxobj->w410_ = (uint16_t)(t - lim);
        V34demodulate(rxobj);
        rxt_post(rxobj, 1);
      }
      V34demodulate(rxobj);
      rxt_post(rxobj, 0);
    }
    i = (int16_t)(i + 1);
    pos = rxobj->w410_;
    d = (int16_t)((uint32_t)rxobj->w414_ - pos);
  } while ((int16_t)rxobj->w38c_ > i);
}

/**
 * @brief Reset the receive timing and resonator state.
 * @param obj  the V.34 object
 *
 * Initialises the timing filters, resets the demodulator sink to `rxFskBuf` and clears or presets the phase, step, resonator and resampler state words
 * (limit `w436_` = 0x960, step word 1, a few words preset to USHRT_MAX or 1).
 */
void rxtiminginit(V34Obj* obj)
{
  V34Rx* rxobj = &obj->rx;

  V34TimingFiltersInit(rxobj->unk050c_);

  rxobj->rxEnd_ = (const int16_t*)rxobj->rxFskBuf_;
  rxobj->w0438_ = 0;
  rxobj->p0448_ = NULL;
  rxobj->w044c_ = 0;
  rxobj->resPhase_ = 0;
  rxobj->w0444_ = 0;
  rxobj->w0454_ = 0;
  rxobj->w0430_ = 0;
  rxobj->w0432_ = 0;
  rxobj->resOffs_ = 0;
  rxobj->w436_ = 0x960;
  AS_U16(rxobj->w0424_) = USHRT_MAX;
  rxobj->w042c_ = 1;
  rxobj->w041c_ = 1;
  rxobj->w0494_ = 0;
  rxobj->w0492_ = 0;
  rxobj->w388_ = 0;
  rxobj->w0420_ = 0;
  rxobj->decPt_[1] = 0;
  rxobj->decPt_[0] = 0;
  AS_U16(rxobj->eqPt_[0]) = 0;
  AS_U16(rxobj->eqPt_[1]) = 0;
  AS_U16(rxobj->w04aa_) = 0;
  AS_U16(rxobj->w04a8_) = 0;
}
