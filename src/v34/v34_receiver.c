/**
 * @file v34_receiver.c
 * @brief V.34 receive symbol processor: receiver().
 *
 * receiver() runs once per received block of `b + 0x128` (4) half-symbol samples.  `b + X` is byte X of the receive sub-object (V34Rx, `obj->rx`).  The steps are:
 *  1. T/2 timing interpolation: for each input sample the timing filter and prefilter produce a half-symbol sample pair; the equalizer delay line is
 *     updated on every second sample, and V34demodulate() is run once or twice depending on how many symbol periods the timing phase wraps.
 *  2. TimingV34() (timing loop) and V34EqualizerFilter() produce the equalized sample (eI, eQ), stored in `eqPt_`.
 *  3. If receive flag 0x400 is set (TRN / training phase): the retrain / renegotiation request detector, which compares the sample with the two
 *     previous ones and raises flag 0x40 (retrain request) or 0x20 (RRN request).
 *  4. If flag 0x2000 is set: a 3-tap complex prefilter update on the equalizer output (rx_taps()).
 *  5. Carrier rotation of the sample into (w0474_, w0476_) and the 300-entry history ring of constellation points (`constPts`, `constCnt_`).
 *  6. Three front-end modes: TRN / sync (flag 0x400: reference symbols from vectpp, from the scrambler + rxvect4, or the frame decoder
 *     decoderv34()), S / S-bar (abcd) detection, and early returns.  The modes compute the reference symbol and the phase error X (rx_phase_error()).
 *  7. rx_tail(): if flag 0x1000, a decision-error 3-tap update; then the carrier PLL (the cos / sin phasor from costbl); unless flag 0x100, the error
 *     accumulators over a 1024-symbol window (reported through `equErr_` and debug text) and the equalizer adaptation (V34EqualizerAdapt() or
 *     V34EqualizerCenterAdapt() depending on flag 0x400).
 *
 * Callees: V34TimingFilter, V34TimingPrefilter, V34Equalizer* and V34scrambler (v34_dsp_primitives.c), TimingV34 (v34_init.c), V34demodulate
 * (v34_agc_demod.c), decoderv34 (v34_frame_decoder.c).  Tables: vectpp (v34_receiver_tables.c), rxvect4 (v34_init_tables.c), costbl
 * (v34_symbol_tables.h), FPM_sqrt_table.  The only pointer slot is `rxEnd_` (the receive sink, written at entry); the scrambler state words are passed
 * as in-object `unsigned int *`.  The caller is the data pump and the handshake.
 *
 * Hazards:
 *  - H1: the store `smp_[1 + i]` (S[] at `b + 0x27a + 2 * i`) (i is the signed 16-bit loop counter) and the sink pointer, which advances one short per V34demodulate()
 *    call, are bounded only by `b + 0x128` (n, not checked); the real n is 4.
 *  - H2: vectpp[(short)b+0x120] is valid for pair indices 0..0x2f; vectpp_at() returns 0 outside.
 *  - H3: rxvect4[V34scrambler(...)]: the scrambler result is masked to 2 bits, so it is always 0..3.
 *  - H4: the history ring index `constCnt_` is a signed 16-bit value used as an element index; the incremented index is stored before the two ring
 *    stores, and the wrap test (> 0x12b) re-reads memory.
 *  - H5: the signal-energy square-root index is clamped to <= 0xbf; the shift count (sh >> 1) is <= 14.
 *  - H6: there is no division in this file.
 */

#include <stddef.h>
#include <stdint.h>

#include "fpm/fpm_tables.h"      /* FPM_sqrt_table */
#include "v34_init_tables.h"     /* rxvect4 */
#include "v34_symbol_tables.h"   /* costbl (= v8_costbl) */
#include "v34_receiver_tables.h" /* vectpp */
#include "core/dsp_host.h"
#include "v34_internal.h"

#include "v34_access.h" /* AS_U16 (unsigned view of an int16_t member) and the byte-offset access of the sample window */
#include "v34_object_view.h"

#include "v34_sar32.h"
/** @brief 32-bit wrapping product of two sign-extended 16-bit / int operands. */
static inline uint32_t mul32(int32_t a, int32_t c) { return (uint32_t)a * (uint32_t)c; }

/* the decided point (I, Q) = the two 16-bit halves of a word of rxvect4[] */
static inline void setDecided(V34Rx* rxobj, int word)
{
  rxobj->decPt_[0] = (int16_t)word;
  rxobj->decPt_[1] = (int16_t)((uint32_t)word >> 16);
}

/** @brief vectpp pair read: element 2 * idx (+1 for Q), idx = (short); 0 outside 0..47 (hazard H2). */
#ifdef PORT_TRACE_INDEX
/* test-only instrumentation, compiled in only by a test harness's private copy of this file (the library never defines
 * PORT_TRACE_INDEX): reports an out-of-range vectpp pair index */
extern int port_trace_vectpp(int idx);
#endif
static int vectpp_at(int idx, int second)
{
#ifdef PORT_TRACE_INDEX
  if (port_trace_vectpp(idx))
    return 0;
#endif
  if (idx < 0 || idx > 47)
    return 0;
  return vectpp[2 * idx + second];
}

/**
 * @brief The 3-tap complex filter shared by the prefilter update (flag 0x2000) and the decision-error update (flag 0x1000).
 * @param rxobj  the receive sub-object
 * @param ee     receives the imaginary correction acc2 >> 14 (32-bit arithmetic shift)
 * @return the real correction dRe (sign-extended short)
 * @note X1 = `tapX1_` and X2 = `tapX2_` (3 shorts each), Y1 = `tapY1_` and Y2 = `tapY2_` (4-entry delay lines, newest entry at index 0; the sums read
 *       indices 2 down to 0).  acc1 = 0x2000 + sum Y1[2 - i] * X1[i]; dRe = (short)((-acc1 + sum Y2[2 - i] * X2[i]) >> 14); acc2 = 0x2000 +
 *       sum Y2[2 - i] * X1[i] + sum Y1[2 - i] * X2[i].  While summing, the two delay lines are shifted up by one slot; each element is read before it is
 *       overwritten.
 */
static int rx_taps(V34Rx* rxobj, int* ee)
{
  uint32_t acc1 = 0x2000u, t, acc2;
  int i;

  for (i = 0; i < 3; i++)
    acc1 += mul32(rxobj->tapY1_[2 - i], rxobj->tapX1_[i]);
  t = 0u - acc1;
  for (i = 0; i < 3; i++)
    t += mul32(rxobj->tapY2_[2 - i], rxobj->tapX2_[i]);
  {
    int dRe = (int16_t)sar32(t, 14);

    acc2 = 0x2000u;
    for (i = 0; i < 3; i++)
    {
      uint16_t v = AS_U16(rxobj->tapY2_[2 - i]);
      AS_U16(rxobj->tapY2_[3 - i]) = v;
      acc2 += mul32((int16_t)v, rxobj->tapX1_[i]);
    }
    for (i = 0; i < 3; i++)
    {
      uint16_t v = AS_U16(rxobj->tapY1_[2 - i]);
      AS_U16(rxobj->tapY1_[3 - i]) = v;
      acc2 += mul32((int16_t)v, rxobj->tapX2_[i]);
    }
    *ee = sar32(acc2, 14);
    return dRe;
  }
}

/** @brief Phase error X = (refI * Y1 - refQ * Y0) << 2 (32-bit wrap); reference at `b + 0x20c / 0x20e`, rotated sample Y at `b + 0x210 / 0x212`. */
static uint32_t rx_phase_error(V34Rx* rxobj)
{
  uint32_t v = mul32((int16_t)rxobj->decPt_[0], (int16_t)rxobj->w0476_) - mul32((int16_t)rxobj->decPt_[1], (int16_t)rxobj->w0474_);
  return v << 2;
}

/**
 * @brief The tail shared by every receiver() path that has computed the phase error: decision-error update, carrier PLL, error accumulators and
 *        equalizer adaptation.
 * @param rxobj  the receive sub-object
 * @param fl     cached receive flags (`rxFlags_`)
 * @param X      the phase error from rx_phase_error()
 */
static void rx_tail(V34Rx* rxobj, uint32_t fl, uint32_t X)
{
  int32_t y0f, y1f;

  /* ---- flags & 0x1000: decision-error 3-tap update ---- */
  if (fl & 0x1000)
  {
    int dRe, ee, E0, E1, i;

    rxobj->w0478_ = (uint16_t)(rxobj->w0474_ - rxobj->decPt_[0]);
    rxobj->w047a_ = (uint16_t)(rxobj->w0476_ - rxobj->decPt_[1]);
    dRe = rx_taps(rxobj, &ee);

    {
      uint16_t p1 = rxobj->w047a_;
      uint16_t p0 = rxobj->w0478_;
      uint16_t n1 = (uint16_t)((int16_t)ee + p1);
      uint16_t n0 = (uint16_t)(p0 + (uint32_t)dRe);

      AS_U16(rxobj->tapY1_[0]) = p1;
      AS_U16(rxobj->tapY2_[0]) = p0;
      rxobj->w0478_ = n0;
      E0 = (int16_t)n0;
      E1 = (int16_t)n1;
      rxobj->w047a_ = n1;
    }
    rxobj->w048c_ = rxobj->w048c_ + mul32(E0, E0) + mul32(E1, E1);
    for (i = 0; i < 3; i++)
    {
      int32_t x2 = rxobj->tapX2_[i];
      int32_t x1 = rxobj->tapX1_[i];
      int32_t yA = rxobj->tapY2_[3 - i];
      int32_t yB = rxobj->tapY1_[3 - i];

      AS_U16(rxobj->tapX2_[i]) = (uint16_t)sar32(((uint32_t)x2 << 16) - mul32(yA, E0) - mul32(yB, E1) + 0x8000u, 16);
      AS_U16(rxobj->tapX1_[i]) = (uint16_t)sar32(((uint32_t)x1 << 16) - mul32(yA, E1) + mul32(yB, E0) + 0x8000u, 16);
    }
    fl = rxobj->rxFlags_;
  }

  /* ---- rotate the reference by the carrier phasor, then the carrier PLL ---- */
  {
    int32_t refI = (int16_t)rxobj->decPt_[0];
    int32_t refQ = (int16_t)rxobj->decPt_[1];
    int32_t c = rxobj->w0456_;
    int32_t s = rxobj->w0458_;

    y0f = sar32(mul32(c, refI) - mul32(s, refQ) + 0x2000u, 14);
    y1f = sar32(mul32(s, refI) + mul32(c, refQ) + 0x2000u, 14);
    rxobj->w0474_ = (uint16_t)y0f;
    rxobj->w0476_ = (uint16_t)y1f;
  }
  {
    int32_t a = (int32_t)X >> ((unsigned)(int32_t)rxobj->w0464_ & 31);
    int32_t a2 = (int32_t)X >> ((unsigned)(int32_t)rxobj->w0466_ & 31);
    uint32_t acc = (uint32_t)rxobj->w045c_ + (uint32_t)a2;
    uint32_t tt;
    int32_t h, c2, s2, f;
    unsigned idx;

    rxobj->w045c_ = acc;
    tt = (uint32_t)a + (uint32_t)sar32(acc, 5) + ((uint32_t)(int32_t)rxobj->w046a_ << 16) + (uint32_t)(int32_t)rxobj->w0468_;
    AS_U16(rxobj->w0468_) = (uint16_t)tt;
    h = sar32(tt, 16);
    AS_U16(rxobj->w046a_) = (uint16_t)(h & 0x1fff);
    idx = ((unsigned)h & 0x1fff) >> 5;
    f = (h & 0x1f) * 50;
    c2 = (int16_t)costbl[idx];
    s2 = (int16_t)(0u - (uint32_t)(uint16_t)costbl[(idx + 0x40) & 0xff]); /* sine = -costbl[idx + 0x40] */
    AS_U16(rxobj->w0456_) = (uint16_t)c2;                                 /* transient store, overwritten below */
    AS_U16(rxobj->w0458_) = (uint16_t)sar32(((uint32_t)s2 << 16) + mul32(f, c2) + 0x8000u, 16);
    AS_U16(rxobj->w0456_) = (uint16_t)sar32(((uint32_t)c2 << 16) - mul32(f, s2) + 0x8000u, 16);
  }

  if (fl & 0x100)
    return; /* flag 0x100 set: no adaptation */

  /* ---- error accumulators, 1024-symbol window, equalizer adaptation ---- */
  {
    int32_t d0 = (int16_t)(AS_U16(rxobj->eqPt_[0]) - (uint32_t)y0f);
    int32_t d1 = (int16_t)(AS_U16(rxobj->eqPt_[1]) - (uint32_t)y1f);
    int32_t gain = (int16_t)rxobj->w047c_;
    int32_t e0 = sar32(mul32(d0, gain), 16);
    int32_t e1 = sar32(mul32(d1, gain), 16);
    uint32_t errEnergy = mul32(d0, d0) + mul32(d1, d1) + rxobj->w484_;
    int32_t ys0 = (int16_t)y0f, ys1 = (int16_t)y1f;
    uint32_t cnt;

    rxobj->w04b0_ = rxobj->w04b0_ + (uint32_t)sar32(mul32(ys0, ys0) + mul32(ys1, ys1), 8);
    cnt = (rxobj->w480_ + 1u) & 0x3ff;
    rxobj->w480_ = (uint16_t)cnt;
    if (cnt == 0)
    {
      AS_U16(rxobj->equErr_) = ((int32_t)errEnergy < 0) ? 0x7fff : (uint16_t)sar32(errEnergy, 16);
      {
        uint32_t w = rxobj->w048c_;
        AS_U16(rxobj->w0488_) = ((int32_t)w < 0) ? 0x7fff : (uint16_t)sar32(w, 16);
      }
      {
        uint32_t en = rxobj->w04b0_;
        rxobj->w484_ = 0;
        rxobj->w048c_ = 0;
        rxobj->w4ac_ = (uint32_t)sar32(en, 8);
        rxobj->w04b0_ = 0;
      }
      if ((int16_t)rxobj->w388_ <= 0x7530 && dsplibs_debug_level > 1)
      {
        dsplibs_debug_printf("V34EQU, equerr = %d, preerr = %d,\n", (int)rxobj->equErr_, (int)rxobj->w0488_);
        fl = rxobj->rxFlags_;
      }
    }
    else
    {
      rxobj->w484_ = errEnergy;
    }
    if (fl & 0x400)
    {
      V34EqualizerAdapt(rxobj->unk0628_ + 8, (short)e0, (short)e1);
    }
    else
    {
      V34EqualizerCenterAdapt(rxobj->unk0628_ + 8, (short)(2 * e0), (short)(2 * e1));
    }
  }
}

/**
 * @brief Process one block of received samples through timing recovery, equalization, carrier tracking and the training-phase detectors.
 * @param obj  the V.34 object
 * @note see the file header for the steps.  Reads `rxFlags_` repeatedly because the debug-print and detector paths may change it; updates the
 *       receive sub-object, the constellation history ring and, when the signal energy is too low in S-S1 detection, sets `mode_` to 10 (disconnect).
 * @warning `b + 0x128` (the sample count `w38c_`) is not range-checked (hazard H1): `smp_[1 + i]` is stored for i < w38c_, and `smp_` has 8 entries, so a
 *          count above 7 writes over the members behind it.  Every writer in the library stores 4; the test harness raises it on purpose (docs/bugs.md V79).
 */
void receiver(V34Obj* obj)
{
  V34Rx* rxobj = &obj->rx;
  int16_t* smp = rxobj->smp_; /* a pointer to the first element: the index is not bounded by the array (see the warning) */
  int i = 0;
  uint32_t fl;
  int oI, oQ;
  int32_t eI, eQ;

  rxobj->rxEnd_ = (const int16_t*)rxobj->rxFskBuf_;

  /* ---- (1) T/2 timing interpolation loop ---- */
  if ((int16_t)rxobj->w38c_ > 0)
  {
    for (;;)
    {
      uint32_t ph = rxobj->w410_;
      uint32_t mz = rxobj->w414_;
      int32_t dd = (int16_t)(mz - ph);
      int32_t lo = sar32(mul32(rxobj->w04a4_, (int32_t)ph) + mul32(rxobj->w04a8_, dd) + 0x2000u, 14);
      int32_t hi = sar32(mul32(rxobj->w04a6_, (int32_t)ph) + mul32(rxobj->w04aa_, dd) + 0x2000u, 14);
      uint32_t sum, s;
      int32_t ms;

      smp[1 + i] = (int16_t)V34TimingFilter(rxobj->unk050c_, (int)(((uint32_t)hi << 16) | (uint16_t)lo));
      if (i & 1)
      {
        uint32_t pf = V34TimingPrefilter(rxobj->unk050c_);
        V34EqualizerUpdateDelayLine(rxobj->unk0628_ + 8, (short)pf, (short)(pf >> 16));
      }
      sum = (uint32_t)rxobj->w410_ + (uint32_t)rxobj->w412_;
      s = (uint16_t)sum;
      mz = rxobj->w414_;
      ms = (int16_t)mz;
      if ((int32_t)s >= ms)
      {
        uint32_t t = sum - mz;
        if ((int32_t)(uint16_t)t < ms)
        {
          rxobj->w410_ = (uint16_t)t; /* one wrap: skip the first demodulate + copy of the 2-wrap path */
        }
        else
        {
          rxobj->w410_ = (uint16_t)(t - mz);
          V34demodulate(rxobj);
        }
        AS_U16(rxobj->w04a8_) = AS_U16(rxobj->w04a4_);
        AS_U16(rxobj->w04aa_) = AS_U16(rxobj->w04a6_);
        V34demodulate(rxobj);
      }
      else
      {
        rxobj->w410_ = (uint16_t)sum;
      }
      i = (int16_t)(i + 1);
      if ((int16_t)rxobj->w38c_ <= i)
        break;
    }
  }

  /* ---- (2) timing loop + equalizer output ---- */
  TimingV34(obj);
  V34EqualizerFilter(rxobj->unk0628_ + 8, &oI, &oQ);
  fl = rxobj->rxFlags_;
  eI = sar32((uint32_t)oI + 0x2000u, 14);
  eQ = sar32((uint32_t)oQ + 0x2000u, 14);
  AS_U16(rxobj->eqPt_[0]) = (uint16_t)eI;
  AS_U16(rxobj->eqPt_[1]) = (uint16_t)eQ;

  /* ---- (3) TRN mode: retrain / RRN request detector ---- */
  if (fl & 0x400)
  {
    uint32_t a0 = rxobj->rxPrevA_[0], a1 = rxobj->rxPrevA_[1], b0 = rxobj->rxPrevB_[0], b1 = rxobj->rxPrevB_[1];
    int32_t d1i = (int16_t)((uint32_t)eI - a0), d1q = (int16_t)((uint32_t)eQ - a1);
    int32_t d2i = (int16_t)((uint32_t)eI - b0), d2q = (int16_t)((uint32_t)eQ - b1);
    int16_t m1 = (int16_t)sar32(mul32(d1i, d1i) + mul32(d1q, d1q), 14);
    int16_t m2 = (int16_t)sar32(mul32(d2i, d2i) + mul32(d2q, d2q), 14);

    rxobj->rxPrevB_[0] = rxobj->rxPrevA_[0]; /* the previous two equalized samples move up by one */
    rxobj->rxPrevB_[1] = rxobj->rxPrevA_[1];
    rxobj->rxPrevA_[0] = AS_U16(rxobj->eqPt_[0]);
    rxobj->rxPrevA_[1] = AS_U16(rxobj->eqPt_[1]);
    if (m1 <= 0x80)
    { /* m1 <= 0x80: close to the previous sample */
      uint32_t c1 = (uint32_t)rxobj->w09fc_ + 1u;
      if ((int16_t)c1 <= 0x8c)
      {
        rxobj->w09fc_ = (uint16_t)c1;
      }
      else
      {
        fl |= 0x40;
        rxobj->w09fc_ = 0;
        rxobj->rxFlags_ = (uint16_t)fl;
        if (dsplibs_debug_level > 1)
        {
          dsplibs_debug_printf("V34RETRAIN, retrain request detected, rtncount = %d \n", 0);
          fl = rxobj->rxFlags_;
        }
      }
    }
    else if (m2 <= 0x80)
    {
      rxobj->w09fc_ = (uint16_t)(rxobj->w09fc_ - 1u);
    }
    else
    {
      int32_t c = (int16_t)rxobj->w09fc_;
      if (c + 0x78 <= 0 && c + 0x84 > 0)
      {
        fl |= 0x20;
        rxobj->rxFlags_ = (uint16_t)fl;
        if (dsplibs_debug_level > 1)
        {
          dsplibs_debug_printf("V34RENEG, RRN request detected,rtncount = %d\n", (int)c);
          fl = rxobj->rxFlags_;
        }
      }
      rxobj->w09fc_ = 0;
    }
  }

  /* ---- (4) flags & 0x2000: prefilter 3-tap update on the equalizer output ---- */
  if (fl & 0x2000)
  {
    int dRe, ee;
    uint16_t i0 = AS_U16(rxobj->eqPt_[0]), q0 = AS_U16(rxobj->eqPt_[1]);

    dRe = rx_taps(rxobj, &ee);
    AS_U16(rxobj->tapY2_[0]) = i0;
    AS_U16(rxobj->tapY1_[0]) = q0;
    AS_U16(rxobj->eqPt_[0]) = (uint16_t)(i0 + (uint32_t)(int32_t)(int16_t)dRe);
    AS_U16(rxobj->eqPt_[1]) = (uint16_t)(q0 + (uint32_t)ee);
  }

  /* ---- (5) carrier rotation + history ring ---- */
  {
    int32_t c = rxobj->w0456_, s = rxobj->w0458_, I = rxobj->eqPt_[0], Q = rxobj->eqPt_[1];
    uint16_t r210 = (uint16_t)sar32(mul32(s, Q) + mul32(c, I) + 0x2000u, 14);
    uint16_t r212 = (uint16_t)sar32(mul32(c, Q) - mul32(s, I) + 0x2000u, 14);
    int idx;
    int16_t* ring = obj->tx.constPts_;

    rxobj->w0474_ = r210;
    rxobj->w0476_ = r212;
    idx = obj->tx.constCnt_;
    obj->tx.constCnt_ = (uint16_t)(idx + 1);
    ring[2 * idx] = r210; /* idx is not range-checked on entry (a negative value reaches the index word in front of constPts_); the compare below re-reads it */
    ring[2 * idx + 1] = r212;
    if (AS_U16(obj->tx.constCnt_) > 0x12b)
      obj->tx.constCnt_ = 0;
  }

  /* ---- (6) front-end modes ---- */
  fl = rxobj->rxFlags_;
  if (fl & 0x400)
  {
    /* --- TRN / sync mode --- */
    int cnt = (int16_t)rxobj->w388_;
    int X;
    uint32_t xerr;

    if (cnt <= 0x11)
    {                                                                                 /* too early: clear the tap block and the pair index */
      sysdep_memset(rxobj->tapX2_, 0, sizeof(rxobj->tapX2_) + sizeof(rxobj->tapX1_)); /* tapX1_ follows tapX2_ */
      rxobj->w0384_ = 0;
      rxobj->w0408_ = 0;
      return;
    }
    if (!(fl & 8))
    {
      X = cnt;
    }
    else
    {
      X = (int16_t)(cnt + 0x120);
    }
    if (!(fl & 8) && cnt <= 0x132)
    {
      /* reference symbol from the vectpp table */
      uint32_t zi = rxobj->w0384_;
      int idx = (int16_t)zi;
      int nx = (int16_t)(zi + 1);

      rxobj->decPt_[0] = (uint16_t)(vectpp_at(idx, 0) >> 1);
      rxobj->decPt_[1] = (uint16_t)(vectpp_at(idx, 1) >> 1);
      if (nx > 0x2f)
        nx = 0;
      rxobj->w0384_ = (uint16_t)nx;
      xerr = rx_phase_error(rxobj);
      rxobj->w0460_ = xerr;
    }
    else if (X > 0x332)
    {
      /* frame mode */
      decoderv34(obj);
      fl = rxobj->rxFlags_;
      xerr = rx_phase_error(rxobj);
      rxobj->w0460_ = xerr;
    }
    else
    {
      /* scrambler-driven reference symbol */
      int sidx = V34scrambler(&rxobj->w0404_, (short)(fl & 4), 3, 2);

      setDecided(rxobj, rxvect4[sidx]);
      fl = rxobj->rxFlags_;
      if (fl & 0x10)
      {
        if ((uint16_t)(X - 0x143) <= 0xf)
        {
          sidx = V34scrambler(&rxobj->w0408_, (short)(fl & 4), 3, 2);
        }
        if ((int16_t)X == 0x153)
        { /* shifted-TRN2 test: compare the energy of two windows of the equalizer taps */
          uint32_t s1 = 0, s2 = 0;
          int k;

          for (k = 0x20; k <= 0x2f; k++)
            s1 += mul32(rxobj->eqRe_[k], rxobj->eqRe_[k]) + mul32(rxobj->eqIm_[k], rxobj->eqIm_[k]);
          for (k = 0x3c; k <= 0x4b; k++)
            s2 += mul32(rxobj->eqRe_[k], rxobj->eqRe_[k]) + mul32(rxobj->eqIm_[k], rxobj->eqIm_[k]);
          if ((int32_t)s2 > (int32_t)s1)
          {
            if (dsplibs_debug_level > 1)
              dsplibs_debug_printf("Detected shifted TRN2... assuming 32 symbols Snot\n");
            rxobj->w0404_ = rxobj->w0408_;
            V34EqualizerCleanUp(rxobj->unk0628_ + 8);
            setDecided(rxobj, rxvect4[sidx]);
          }
          fl = rxobj->rxFlags_;
          rxobj->w0408_ = 0;
        }
        else
        {
          fl = rxobj->rxFlags_;
        }
      }
      xerr = rx_phase_error(rxobj);
      rxobj->w0460_ = xerr;
    }
    rx_tail(rxobj, fl, xerr);
    return;
  }

  /* --- S / S-bar (abcd) detection mode --- */
  {
    int32_t cntw;
    uint32_t cnt;
    uint32_t xerr;
    int16_t m;

    if (rxobj->w0424_ <= 1)
      return;
    cnt = rxobj->w388_;
    if ((uint16_t)cnt == 0)
    { /* classify the first reference: the closer of rxvect4[0] / rxvect4[3] decides the parity */
      int32_t dI, dQ;
      int16_t m3, m0;

      setDecided(rxobj, rxvect4[3]);
      dI = (int16_t)((uint32_t)rxobj->w0474_ - rxobj->decPt_[0]);
      dQ = (int16_t)((uint32_t)rxobj->w0476_ - rxobj->decPt_[1]);
      m3 = (int16_t)sar32(mul32(dI, dI) + mul32(dQ, dQ), 14);
      setDecided(rxobj, rxvect4[0]);
      dI = (int16_t)((uint32_t)rxobj->w0474_ - rxobj->decPt_[0]);
      dQ = (int16_t)((uint32_t)rxobj->w0476_ - rxobj->decPt_[1]);
      m0 = (int16_t)sar32(mul32(dI, dI) + mul32(dQ, dQ), 14);
      cnt = (m0 <= m3) ? 2u : 1u;
      rxobj->w388_ = (uint16_t)cnt;
    }
    setDecided(rxobj, (cnt & 1) ? rxvect4[3] : rxvect4[0]);
    rxobj->w047c_ = 0x7000;
    {
      int32_t dI = (int16_t)((uint32_t)rxobj->w0474_ - rxobj->decPt_[0]);
      int32_t dQ = (int16_t)((uint32_t)rxobj->w0476_ - rxobj->decPt_[1]);
      m = (int16_t)sar32(mul32(dI, dI) + mul32(dQ, dQ), 14);
    }
    cntw = (int16_t)cnt;
    if ((uint16_t)cnt == 0x40)
    {
      if (!(fl & 0x200) && dsplibs_debug_level > 1)
      {
        dsplibs_debug_printf("V34AGC, abcddetect gain = 0x%x, AGC frozen\n", (int)rxobj->rxGain_);
        fl = rxobj->rxFlags_;
      }
    }
    else
    {
      if (cntw > 0x40)
      {
        fl |= 0x200;
        rxobj->rxFlags_ = (uint16_t)fl;
      }
      if (cntw > 0x68)
      {
        fl |= 0x100;
        rxobj->rxFlags_ = (uint16_t)fl;
        if (m > 0x600)
        {
          if (dsplibs_debug_level > 1)
          {
            dsplibs_debug_printf("S-S1 is detected,rxsymcnt= %d,pllcnt= %d,gain= 0x%x\n", (int)(int16_t)cnt, (int)rxobj->w0424_, (int)rxobj->rxGain_);
          }
          rxobj->w388_ = 0;
          rxobj->rxFlags_ = (uint16_t)((rxobj->rxFlags_ & 0xfeffu) | 0x600u);
          V34EqualizerClearCenterTaps(rxobj->unk0628_ + 8);
          fl = rxobj->rxFlags_;
        }
      }
    }
    xerr = rx_phase_error(rxobj);
    rxobj->w0460_ = xerr;
    if (fl & 0x400)
    {
      /* signal-energy check, reachable only after an S-S1 hit set flag 0x400 (0x600) in this call */
      uint32_t sum = 0;
      int k, val;

      for (k = 0; k < 36; k++)
      {
        int32_t sv = rxobj->levelRing_[k];
        sum += (uint32_t)((sv * 0x38e) >> 15) * (uint32_t)sv;
      }
      if (sum == 0)
      {
        val = 0;
      }
      else
      {
        uint32_t e = sum;
        uint16_t sh = 0, mant;

        if (e <= 0x1fffffffu)
        {
          do
          {
            e += e;
            sh = (uint16_t)(sh + 1);
          } while (e <= 0x1fffffffu);
        }
        mant = (uint16_t)(e >> 15);
        if (sh & 1)
          mant = (uint16_t)(mant >> 1);
        {
          int32_t q = ((int32_t)mant + 0x40) >> 7; /* mant + 0x40 >= 0, so the shift equals a division */
          uint16_t ix = (uint16_t)(q - 0x40);

          if (ix > 0xbf)
            ix = 0xbf;
          val = (int16_t)(uint16_t)((unsigned)(uint16_t)FPM_sqrt_table[ix] >> (((unsigned)(sh >> 1)) & 31));
        }
      }
      if (val > obj->minSigLevel_)
      {
        /* above the threshold: nothing */
      }
      else
      {
        if (dsplibs_debug_level > 1)
        {
          dsplibs_debug_printf("Signal Energy below Threshold %d, initiate a disconnection", (int)obj->minSigLevel_);
          fl = rxobj->rxFlags_;
        }
        obj->mode_ = 10;
      }
      return; /* flag 0x400 is set here: no tail */
    }
    rx_tail(rxobj, fl, xerr);
  }
}
