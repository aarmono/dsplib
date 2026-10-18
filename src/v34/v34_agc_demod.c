/**
 * @file v34_agc_demod.c
 * @brief V.34 receive front end and INFO-sequence helpers: AGC, input demodulation, the transmit interpolating modulator and the INFO0 / INFO1a bit
 *        packing.
 *
 * Entry points:
 * - V34agc(): drains 4 input samples from the receive circular buffer, scales them with the AGC gain and updates the level estimate / gain loop.
 * - V34demodulate(): the same for one (I, Q) sample pair, with rounding, followed by a rotation of the pair by the table-driven mixer.
 * - V34ModulatorProcess(): polyphase interpolating modulator of the transmit path, writes the output samples for one symbol.
 * - V34GiveProbeResults(), v34FreezeEcho(): copy the probing results of the receiver into the V.34 object / freeze the echo canceller.
 * - V34SetINFO0dBits(), V34GiveINFO0dBits(), V34GiveINFO1aBits(), V34SetINFO0aBits(): pack and unpack the INFO0 / INFO1a words of the V.PCM / V.90 /
 *   V.92 handshake against the V.90 and V.92 phase-2 info objects.
 * - V34InitializeImplementationSpecific(): sets up the pointers / lengths of the two echo-canceller records and the modulator bank pointer.
 * - bitreverse(): bit reversal of a field, used by the INFO packing here and in src/vpcm.
 *
 * The giant V.34 object (V34Obj, v34_object_view.h) is reached through its members and sub-objects; parts of it are not yet described by members
 * (`unk*_` byte regions), so some addresses are formed inside such a region.  Pointer-valued members are real pointers.
 *
 * The AGC level estimate is shared by V34agc() and V34demodulate(): a 36-entry ring of recent input samples gives a mean-square energy, which is
 * normalised and turned into an amplitude with FPM_sqrt_table; a one-pole filter plus a 2-state gain loop (agc_level_gain_update()) drive rxGain_.
 */

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "fpm/fpm_tables.h"
#include "core/dsp_host.h"
#include "v34_internal.h"

/* the echo-canceller record; defined and used by v34_dsp_primitives.c */
typedef struct EchoAdaptState EchoAdaptState;

#include "v34_k56flex_view.h"    /* K56HandleObj */
#include "v34_flomodem_view.h"   /* PcmFlo47Obj: the engine context */
#include "v34_v90_phase2_view.h" /* P47V90InfoObj */
#include "v34_v92_phase2_view.h" /* P47V92InfoObj */
#include "v34_object_view.h"     /* V34Obj */
/* the member viewed as unsigned 16-bit (the declared type is int16_t; a store of an unsigned value keeps the unsigned conversion) */
#define AS_U16(lv) (*(uint16_t*)&(lv))

/* ---- bit reversal ---- */

/**
 * @brief Reverse the low `nbits` bits of a value.
 * @param x      value; bit 0 becomes the most significant bit of the `nbits`-wide result
 * @param nbits  field width; the loop counter is a signed 16-bit value, so `nbits <= 1` returns `x & 1`
 * @return the reversed field (16-bit accumulator: for `nbits > 16` the top bits are shifted out)
 */
unsigned short bitreverse(unsigned short x, short nbits)
{
  unsigned int xv = x;
  unsigned short r = (unsigned short)(xv & 1);
  short i = 1;

  while (i < nbits)
  {
    xv >>= 1;
    r = (unsigned short)(r * 2);
    if (xv & 1)
      r |= 1;
    i = (short)(i + 1);
  }
  return r;
}

/* ---- AGC level estimate shared by V34agc / V34demodulate ---- */

/**
 * @brief Amplitude estimate of the 36 most recent input samples.
 * @param obj  the receive sub-object (V34Rx); its 36-entry sample ring is read
 * @return the amplitude (16-bit, 0 when the ring is silent)
 *
 * Energy E = sum of x * ((x * 0x38e) >> 15) over the ring (a Q15 1/36 weighting, wrapping 32-bit), normalised by doubling until E >= 0x20000000 (`n`
 * doublings), mantissa E >> 15 (halved for an odd `n`), index into FPM_sqrt_table as ((m + 0x40) >> 7) - 0x40 clamped to 0xbf, result shifted right by
 * n / 2.
 * @note  `n` stays 0 when E is already >= 0x20000000, and then the mantissa E >> 15 (up to 0x1ffff) is truncated to 16 bits; the table index is a
 *        16-bit unsigned wrap clamped to 0xbf.  Both are properties of the arithmetic, not guarded.
 */
static short agc_ring_level(const V34Rx* obj)
{
  unsigned int e = 0;
  int j;

  for (j = 0; j < 36; j++)
  {
    int x = obj->levelRing_[j];
    e += (unsigned int)(((x * 0x38e) >> 15) * x);
  }
  if (e == 0)
    return 0;
  {
    unsigned short n = 0;
    unsigned short m;
    unsigned short idx;

    if (e < 0x20000000u)
    {
      do
      {
        e *= 2;
        n++;
      } while (e < 0x20000000u);
    }

    m = (unsigned short)(e >> 15);
    if (n & 1)
      m = (unsigned short)(m >> 1);
    idx = (unsigned short)((((int)m + 0x40) >> 7) - 0x40);
    if (idx > 0xbf)
      idx = 0xbf;
    return (short)((int)FPM_sqrt_table[idx] >> ((n >> 1) & 31));
  }
}

/**
 * @brief Smooth the signal level and run the AGC gain loop.
 * @param obj  the receive sub-object (V34Rx): rxLevel_, rxGain_, the loop integrator and its gain word are updated
 * @param s    energy sum of the most recent scaled samples (the caller has already stored it in the receive block)
 *
 * Shared tail of V34agc() and V34demodulate().  The level is rxLevel = 0.85 * rxLevel + (s >> 16), saturated to 0x7f00 when it overflows 16 bits.
 * The error d = rxLevel - 4000 is ignored inside the dead band |d| <= 0x4b0.  Otherwise v = (gain-loop weight * d >> 16) + integrator: while |v| <= 500
 * the integrator takes v and the gain is left alone; beyond that the integrator is cleared and the gain is multiplied by 0x390a / 2^14 (about 0.89,
 * level above target) or by 0x47cf / 2^14 (about 1.12, level below target; not applied when the gain is already above 0x6a00).
 */
static void agc_level_gain_update(V34Rx* obj, unsigned int s)
{
  int lv = obj->rxLevel_;
  unsigned int t = (unsigned int)(((lv * 0x6ccd) >> 15) + (int)(s >> 16));
  unsigned int top = t >> 15;
  short d, v;
  int ad, av;

  /* level filter */
  if (top == 0x1ffff || top == 0)
    AS_U16(obj->rxLevel_) = (short)t;
  else
    AS_U16(obj->rxLevel_) = 0x7f00;

  /* error against the target level, dead band */
  d = (short)((int)AS_U16(obj->rxLevel_) - 4000);
  ad = d < 0 ? -(int)d : (int)d;
  if ((short)(ad - 0x4b0) <= 0)
    return;

  /* gain loop */
  v = (short)((((int)(int16_t)obj->w39e_ * (int)d) >> 16) + (int)obj->w039c_);
  av = v < 0 ? -(int)v : (int)v;
  if ((short)(av - 500) <= 0)
  {
    obj->w039c_ = v;
    return;
  }

  obj->w039c_ = 0;
  if (v > 0)
  {
    AS_U16(obj->rxGain_) = (short)(((int)obj->rxGain_ * 0x390a) >> 14);
  }
  else
  {
    if (obj->rxGain_ > 0x6a00)
      return;
    AS_U16(obj->rxGain_) = (short)(((int)obj->rxGain_ * 0x47cf) >> 14);
  }
}

/* ---- V34agc ---- */

/**
 * @brief Automatic gain control step over 4 input samples.
 * @param obj  the receive sub-object (V34Rx) of the V.34 object
 *
 * Drains 4 (I, Q) pairs (taking the first element of each pair) from the receive circular buffer into the 4-short work area `rxFskBuf_`, decrements the
 * pending count, resets the output end pointer, pushes the samples into the 36-entry level ring (index wraps at 36) and scales them by the AGC gain with
 * a truncating >> 10 (V34demodulate() rounds instead).  A product that does not fit 26 bits saturates to 0x7f00 / 0x8100 and is reported through the
 * debug print.  Unless the AGC is frozen (`rxFlags_` bit 0x200) and the ring amplitude exceeds 0x1f, the energy sum of the 4 scaled samples is stored and
 * agc_level_gain_update() runs.
 */
void V34agc(V34Rx* obj)
{
  short* work = (short*)obj->rxFskBuf_;
  short* rd = (short*)obj->p0268_;
  uintptr_t wrap_at = (uintptr_t)obj->rxFskBuf_;
  int i;

  /* drain 4 samples from the circular buffer */
  AS_U16(obj->rxW0_) = (short)(obj->rxW0_ - 4);
  for (i = 0; i < 4; i++)
  {
    work[i] = *rd;
    rd += 2;
    if (!(wrap_at > (uintptr_t)rd))
      rd = (short*)obj->unk0270_;
  }
  obj->p0268_ = rd;
  obj->rxEnd_ = (const int16_t*)obj->unk0378_;

  /* push into the level ring */
  for (i = 0; i < 4; i++)
  {
    short idx = (int16_t)obj->w0400_;
    short nxt = (short)(idx + 1);
    obj->w0400_ = nxt;
    obj->levelRing_[idx] = work[i];
    if (nxt > 0x23)
      obj->w0400_ = 0;
  }

  /* scale by the gain, truncating */
  for (i = 0; i < 4; i++)
  {
    int p = (int)obj->rxGain_ * (int)work[i];
    unsigned int top = (unsigned int)p >> 25;
    if (top == 0 || top == 0x7f)
    {
      work[i] = (short)(p >> 10);
    }
    else
    {
      int hi = p >> 16;
      work[i] = ((short)hi <= 0) ? (short)0x8100 : (short)0x7f00;
      if (dsplibs_debug_level > 1)
        dsplibs_debug_printf("V34AGC, overflow = 0x%x,\n", hi);
    }
  }

  /* level and gain update unless frozen */
  if (obj->rxFlags_ & 0x200)
    return;

  if (agc_ring_level(obj) > 0x1f)
  {
    unsigned int s = 0;
    for (i = 0; i < 4; i++)
      s += (unsigned int)((int)work[i] * (int)work[i]);
    obj->w0390_ = s;
    agc_level_gain_update(obj, s);
  }
}

/* ---- V34demodulate ---- */

/**
 * @brief Scale one sample by the AGC gain, rounding.
 * @param obj  the receive sub-object (V34Rx)
 * @param x    input sample
 * @return (gain * x + 0x200) >> 10; saturated to 0x7f00 / 0x8100 (with a debug print) when the product does not fit 26 bits
 */
static short demod_scale(const V34Rx* obj, short x)
{
  int p = (int)obj->rxGain_ * (int)x;
  unsigned int top = (unsigned int)p >> 25;
  if (top == 0 || top == 0x7f)
  {
    return (short)((int)((unsigned int)p + 0x200) >> 10); /* ROUNDING (agc truncates) */
  }
  else
  {
    int hi = p >> 16;
    if (dsplibs_debug_level > 1)
      dsplibs_debug_printf("V34demodulate, agc overflow = 0x%x,\n", hi);
    return (hi > 0) ? (short)0x7f00 : (short)0x8100;
  }
}

/**
 * @brief Demodulate one (I, Q) input pair: AGC, level estimate and mixer rotation.
 * @param rxobj  the receive sub-object (V34Rx) of the V.34 object
 *
 * Reads the next pair from the circular buffer (wrapping at its end), pushes the I sample into the level ring, scales I (stored at the output end
 * pointer, which advances) and Q with demod_scale(), and accumulates the energy of the scaled I samples; every fourth call (when the AGC is not frozen
 * and the ring amplitude exceeds 0x1f) the sum feeds agc_level_gain_update(), and the counter and sum restart.  Finally the scaled pair is rotated by
 * the mixer (sine table `hsineTab_`, phase index advanced by a step modulo `hsineLen_`) and the rotated pair is stored in the receive block, Q14 with
 * rounding.
 * @note  Called by rxtiming(), one pair per call.
 */
void V34demodulate(V34Rx* rxobj)
{
  short* rd = (short*)rxobj->p0268_;
  short in_i, in_q, oi, oq;
  short *nrd, *sink;
  short cnt;
  unsigned int acc;
  short idx;
  uint16_t stepu;

  /* next input pair */
  AS_U16(rxobj->rxW0_) = (short)(rxobj->rxW0_ - 1);
  in_i = rd[0];
  in_q = rd[1];
  nrd = rd + 2;
  if ((uintptr_t)rxobj->rxFskBuf_ > (uintptr_t)nrd)
    rxobj->p0268_ = nrd;
  else
    rxobj->p0268_ = rxobj->unk0270_;

  /* level ring */
  {
    short ri = (int16_t)rxobj->w0400_;
    short nxt = (short)(ri + 1);
    rxobj->levelRing_[ri] = in_i;
    rxobj->w0400_ = (nxt > 0x23) ? 0 : nxt;
  }

  /* scale */
  sink = (short*)rxobj->rxEnd_;
  oi = demod_scale(rxobj, in_i);
  rxobj->rxEnd_ = sink + 1;
  *sink = oi;
  oq = demod_scale(rxobj, in_q); /* Q' feeds only the rotation below */

  /* energy of every fourth sample group feeds the level / gain update */
  cnt = (short)(AS_U16(rxobj->w038e_) + 1);
  acc = (unsigned int)((int)oi * (int)oi) + rxobj->w0390_;
  if (cnt < 4)
  {
    rxobj->w038e_ = cnt;
    rxobj->w0390_ = acc;
  }
  else
  {
    if (!(rxobj->rxFlags_ & 0x200))
    {
      rxobj->w038e_ = cnt;
      if (agc_ring_level(rxobj) > 0x1f)
      {
        rxobj->w0390_ = acc;
        agc_level_gain_update(rxobj, acc);
      }
    }
    rxobj->w038e_ = 0;
    rxobj->w0390_ = 0;
  }

  /* mixer rotation: the phase index is added unsigned but used as a signed table index */
  {
    const short* tab = (const short*)rxobj->hsineTab_;
    uint16_t idxu = rxobj->w0420_;
    uint16_t modu = rxobj->hsineLen_;
    int c, s;
    int newidx;
    idx = (short)idxu;
    c = tab[idx];
    s = tab[(int)idx + (short)modu];
    rxobj->w04a4_ = (short)(((int)oi * s + (int)oq * c + 0x2000) >> 14);
    rxobj->w04a6_ = (short)((s * (int)oq - c * (int)oi + 0x2000) >> 14);
    stepu = rxobj->w041c_;
    newidx = (int)idxu + (int)stepu;
    if (!((short)modu > (short)newidx))
      newidx -= (int)modu;
    rxobj->w0420_ = (short)newidx;
  }
}

/* ---- V34ModulatorProcess ---- */

/**
 * @brief Polyphase interpolating modulator of the transmit path: produce the output samples of one symbol.
 * @param obj       the transmit modulator sub-object (V34Mod)
 * @param packedIn  the symbol's two baseband components: low 16 bits go into history channel A, high 16 bits into channel B (at slot `idx` of each)
 * @param out       receives the output samples
 * @return the number of samples written to `out`
 *
 * While the filter phase is below its limit: a `taps`-tap polyphase FIR row (selected by the phase) is evaluated on both channels, the phase advances
 * by the step, the two results are combined with the carrier table (entries tab[tp] and tab[tp + len], tp advancing modulo len) into one sample, which
 * runs through a 16-tap output filter (shift-register delay line) and is stored.  The phase wraps by subtracting the limit.  At the end the first
 * `taps` slots of both history channels are shifted up by one.
 * @note  The value shifted into the head of each history channel at the end is the last carrier-table entry read (tab[tp] before the increment),
 *        or 0 when no sample was produced; it is not a sample of the input.  The shift covers the first `taps` slots, not starting at `idx`.
 */
int V34ModulatorProcess(V34Mod* obj, uint32_t packedIn, short* out)
{
  int carry = 0;
  int nout = 0;
  int stride = obj->w2210_;
  int idx = obj->w220c_;
  int limit = obj->modLimit_;
  int phase;
  int taps;
  int i;

  /* store the new symbol components in the two history channels */
  obj->modHist_[idx] = (short)packedIn;
  obj->modHist_[idx + stride] = (short)(packedIn >> 16);
  phase = obj->w145c_;

  /* one output sample per phase step */
  if (phase < limit)
  {
    const short* bank = (const short*)obj->p2074_;
    const short* tab = (const short*)obj->p1460_;
    const short* coef = (const short*)obj->p2100_;
    short* delay = obj->pf.pfDelay_;
    int step = obj->modStep_;
    int len = obj->modLen_;
    int tp = obj->w1468_;
    taps = obj->modTaps_;
    do
    {
      const short* row = (const short*)((const char*)bank + phase * 0x80);
      const short* ha = &obj->modHist_[idx];
      const short* hb = ha + stride;
      unsigned int acc_a = 0x1000, acc_b = 0x1000;
      short y, cur;
      unsigned int fir;
      int t;

      /* polyphase FIR on both channels */
      for (t = 0; t < taps; t++)
      {
        int cf = row[t];
        acc_a += (unsigned int)((int)ha[t] * cf);
        acc_b += (unsigned int)((int)hb[t] * cf);
      }
      phase += step;

      /* combine with the carrier */
      {
        int hiv = tab[tp + len];
        int lov = tab[tp];
        carry = lov;
        y = (short)((int)((unsigned int)(((int)acc_a >> 13) * hiv) - (unsigned int)(((int)acc_b >> 13) * lov) + 0x2000u) >> 14);
      }
      tp++;
      if (!(tp < len))
        tp -= len;

      /* output filter */
      cur = y;
      fir = 0;
      for (i = 0; i < 16; i++)
      {
        short old = delay[i];
        delay[i] = cur;
        fir += (unsigned int)((int)cur * (int)coef[i]);
        cur = old;
      }
      out[nout++] = (short)((int)(fir + 0x2000u) >> 14);
    } while (phase < limit);
    obj->w1468_ = tp;
  }
  else
  {
    taps = obj->modTaps_;
  }

  /* shift both history channels */
  if (taps > 0)
  {
    int c = carry;
    for (i = 0; i < taps; i++)
    {
      short old = obj->modHist_[i];
      obj->modHist_[i] = (short)c;
      c = old;
    }
    for (i = 0; i < taps; i++)
    {
      short old = obj->modHist_[stride + i];
      obj->modHist_[stride + i] = (short)c;
      c = old;
    }
  }

  obj->w145c_ = phase - limit;
  return nout;
}

/* ---- V34GiveProbeResults ---- */

/**
 * @brief Copy the probing results (25 doubles) into the V.34 object when a V.PCM or K56Flex session is running.
 * @param obj    the V.34 object
 * @param probe  the 25 DFT bins of the probe block (`probe.bin_`); the `energy` field of each is copied to `probeEnergy_[i]`
 * @return always 0
 *
 * The copy is done as a floating-point load / store would: a signalling NaN is quieted (mantissa bit 51 set), every other 64-bit pattern is copied
 * unchanged.
 */
int V34GiveProbeResults(V34Obj* obj, const DftBin* probe)
{

  if (obj->pcmMode_ != 0 || obj->k56Mode_ != 0)
  {
    int i;

    for (i = 0; i < 25; i++)
    {
      uint64_t bits;
      memcpy(&bits, &probe[i].energy, 8);
      if ((bits & 0x7ff0000000000000ull) == 0x7ff0000000000000ull && (bits & 0x000fffffffffffffull) != 0 && !(bits & 0x0008000000000000ull))
        bits |= 0x0008000000000000ull;
      memcpy(&obj->probeEnergy_[i], &bits, 8); /* the bit pattern, not a double copy: see above */
    }
  }
  return 0;
}

/* ---- v34FreezeEcho ---- */

/**
 * @brief Freeze the echo canceller and report its coefficients.
 * @param obj  the V.34 object
 *
 * Sets bit 0x04 of the transmit flag word `txFlags_`, then (when the debug level is above 1) reports the near and far echo-canceller coefficient sets
 * through V34EchoReportCoeff().
 * @note  The debug level is sampled once at entry and re-read only after the first print, so the second message uses the entry value.
 */
void v34FreezeEcho(V34Obj* obj)
{
  unsigned int lvl = dsplibs_debug_level;

  if (lvl > 1)
  {
    dsplibs_debug_printf("V34HSHAK: Freeze EC\n");
    lvl = dsplibs_debug_level;
  }

  obj->tx.txFlags_ = (uint16_t)(obj->tx.txFlags_ | 4);

  if (lvl > 1)
    dsplibs_debug_printf("==== Near Echo Canceller report ======\n");
  V34EchoReportCoeff((EchoAdaptState*)&obj->echo0);
  if (dsplibs_debug_level > 1)
    dsplibs_debug_printf("==== Far Echo Canceller report ======\n");
  V34EchoReportCoeff((EchoAdaptState*)&obj->echo1);
}

/* ---- INFO0 / INFO1a helpers ---- */

/**
 * @brief Set the INFO0d record's word at +0x18 to 0x1e (V.PCM sessions only).
 * @param obj   the V.34 object
 * @param info  the INFO0d bit-queue record (the sub-object `info0d`, which bitQ2_ points at)
 */
void V34SetINFO0dBits(V34Obj* obj, struct tag_bitQ* info)
{
  if ((obj)->pcmMode_ != 0)
  {
    if (dsplibs_debug_level > 1)
      dsplibs_debug_printf("V90, setINFO0dBits\n");
    info->tail_.total = 0x1e;
  }
}

/**
 * @brief Unpack a received INFO0 frame and derive the remote V.92 capability and the short-phase-2 decision.
 * @param obj   the V.34 object (nothing is done unless it is in V.PCM mode)
 * @param info  the received INFO0 frame as 16-bit words (at least 4 are read; 10 are printed)
 *
 * Expands the frame into a 41-entry array of 0 / 1 dwords inside the object: entries 0..11 are a constant pattern (0xf72, MSB first), 12..19 / 20..27 /
 * 28..35 are the low 8 bits of info[0] / info[1] / info[2] MSB first, and 36..40 are taken from info[3].  The role flag of the flo-modem context
 * (`flag6120_`) decides which of entries 26 / 27 is the remote V.92 capability and which the remote short-phase-2 request; the former is stored in
 * `info1dPending_`, and `shortPhase2_` is set when the local V.92 flag, the remote V.92 flag, the local short request and the remote short request are all
 * set.
 */
void V34GiveINFO0dBits(V34Obj* obj, const unsigned short* info)
{
  PcmFlo47Obj* ctx = (PcmFlo47Obj*)obj->ctx_;
  int32_t* bits = (int32_t*)obj->unka8a4_; /* 41 dwords of the 168-byte region */
  int i;
  short local_v92, remote_v92, is_short;

  if (obj->pcmMode_ == 0)
    return;

  /* debug dump of the received frame */
  if (dsplibs_debug_level > 1)
  {
    dsplibs_debug_printf("VPcmV34Main, giveINFO0%cBits\n", ctx->flag6120_ == 0 ? 'd' : 'a');
    if (dsplibs_debug_level > 1)
    {
      dsplibs_debug_printf("VPcmV34Main, rxinfo0%c = ", ctx->flag6120_ == 0 ? 'd' : 'a');
      if (dsplibs_debug_level > 1)
      {
        dsplibs_debug_printf("0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x\n", info[0], info[1], info[2], info[3], info[4], info[5], info[6], info[7],
                             info[8], info[9]);
      }
    }
  }

  /* expand the frame to one dword per bit */
  for (i = 0; i < 12; i++)
    bits[i] = (0xf72 >> (11 - i)) & 1;
  for (i = 12; i < 20; i++)
    bits[i] = (info[0] >> (19 - i)) & 1;
  for (i = 20; i < 28; i++)
    bits[i] = (info[1] >> (27 - i)) & 1;
  for (i = 28; i < 36; i++)
    bits[i] = (info[2] >> (35 - i)) & 1;
  for (i = 36; i < 41; i++)
    bits[i] = (info[3] >> (43 - i)) & 1;

  /* remote V.92 capability and short phase 2 */
  local_v92 = (int16_t)obj->wabc6_;
  if (ctx->flag6120_ == 0)
  {
    remote_v92 = (short)bits[26];
    obj->info1dPending_ = remote_v92;
    is_short = (local_v92 != 0 && remote_v92 != 0 && obj->wabca_ != 0 && bits[27] != 0) ? 1 : 0;
  }
  else
  {
    remote_v92 = (short)bits[27];
    obj->info1dPending_ = remote_v92;
    is_short = (local_v92 != 0 && remote_v92 != 0 && obj->wabca_ != 0 && bits[26] != 0) ? 1 : 0;
  }
  obj->shortPhase2_ = is_short;

  /* debug summary */
  if (dsplibs_debug_level > 1)
  {
    int32_t rs = (ctx->flag6120_ == 0) ? bits[27] : bits[26];
    dsplibs_debug_printf("VPcmV34Main: localV92- %d, remoteV92- %d, localShort- %d, remoteShort- %d, isShort- %d\n", (int)local_v92, (int)obj->info1dPending_,
                         (int)obj->wabca_, (int)rs, (int)is_short);
  }
}

/**
 * @brief Interpret a received INFO1a frame.
 * @param obj   the V.34 object
 * @param info  the received INFO1a frame as 16-bit words (at least 4 are read; 10 are printed)
 * @return the V.92 session flag of the flo-modem context (`v92Session_`): 1 when the upstream baud index decoded from INFO1a is 6 (V.PCM mode), else 0
 *
 * V.34 / K56Flex mode: when K56Flex was requested, bit 3 of info[3] keeps it enabled (`k56Mode_` = 2) or disables it.  V.PCM mode: decodes the upstream
 * baud index from info[2] / info[3]; index 6 marks a V.92 session and stores three 2-bit fields of info[0] in `pair0_` .. `pair2_` and in the V.92 info
 * object; the "U-info" byte (7 bits of info[1] / info[2], bit-reversed) is stored in both the V.90 and the V.92 info objects.
 */
int V34GiveINFO1aBits(V34Obj* obj, const unsigned short* info)
{
  int32_t pcm = obj->pcmMode_;
  PcmFlo47Obj* floobj = (PcmFlo47Obj*)obj->ctx_;
  struct K56FlexHandle* k56 = obj->k56_;

  floobj->v92Session_ = 0;

  if (pcm == 0)
  {
    if (obj->k56Mode_ != 0)
    {
      if ((info[3] & 8) == 0)
      {
        if (dsplibs_debug_level > 1)
          dsplibs_debug_printf("VPcmV34Main: K56Flex disabled by remote\r\n");
        obj->k56Mode_ = 0;
      }
      else
      {
        if (dsplibs_debug_level > 1)
          dsplibs_debug_printf("VPcmV34Main: K56Flex enabled by remote, PCM type: local %d, remote %d (A=1, Mu=0)\r\n", ((K56HandleObj*)k56)->word_c_,
                               info[3] & 2);
        obj->k56Mode_ = 2;
      }
    }
  }
  else
  {
    int baud;
    short u;

    if (dsplibs_debug_level > 1)
    {
      dsplibs_debug_printf("VPcmV34Main: giveINFO1aBits\n");
      if (dsplibs_debug_level > 1)
      {
        dsplibs_debug_printf("VPcmV34Main: rxinfo1a = 0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x\n", info[0], info[1], info[2], info[3], info[4],
                             info[5], info[6], info[7], info[8], info[9]);
      }
    }

    /* upstream baud index */
    baud = ((info[2] & 2) >> 1) + (info[2] & 1) * 2 + ((info[3] & 0x80) >> 5);
    if (dsplibs_debug_level > 1)
      dsplibs_debug_printf("VPcmV34Main: upstream baud index = %d\r\n", baud);

    if ((short)baud == 6)
    {
      P47V92InfoObj* v92info;

      floobj->v92Session_ = 1;
      obj->pair0_ = (short)(((info[0] & 0x80) >> 7) + ((info[0] & 0x40) >> 5));
      obj->pair1_ = (short)(((info[0] & 0x20) >> 5) + ((info[0] & 0x10) >> 3));
      obj->pair2_ = (short)(((info[0] & 8) >> 3) + ((info[0] & 4) >> 1));
      v92info = (P47V92InfoObj*)floobj->v92Info_;
      v92info->unk14_[0] = (uint8_t)obj->pair0_;
      v92info->unk14_[1] = (uint8_t)obj->pair1_;
      v92info->unk14_[2] = (uint8_t)obj->pair2_;
    }

    /* U-info byte */
    u = (short)bitreverse((unsigned short)(((info[1] & 7) * 0x10) + ((info[2] & 0xf0) >> 4)), 7);
    if (dsplibs_debug_level > 1)
      dsplibs_debug_printf("VPcmV34Main: Uinfo is %d\r\n", (int)u);
    {
      void* p1760 = floobj->v90Info_;
      ((P47V92InfoObj*)floobj->v92Info_)->uinfo_ = (uint8_t)u;
      ((P47V90InfoObj*)p1760)->uinfo_ = (uint8_t)u;
    }
  }
  return (int)(int16_t)floobj->v92Session_;
}

/**
 * @brief Build the INFO0 words to send.
 * @param obj   the V.34 object
 * @param info  receives the words (info[0] .. info[3] are written; info[0] = 0xff, info[1] = 0x84 plus flags)
 *
 * The role flag of the flo-modem context (`flag6120_`) selects the side: 0 builds the digital-side INFO0d (V.PCM only: 0xff, 0x84, 0x30 plus the
 * bit-reversed maximum transmit power and the V.90 transmit-point flags), non-zero the analogue-side INFO0a.  The two sides assign the flag bits
 * differently: role 0 uses 0x2 for the short-phase-2 request and 0x1 for V.92 capability; role != 0 uses 0x1 for short phase 2 (and sets the previous
 * bulk delay to 0x14) and 0x2 for V.92.  Non-V.PCM sessions send the plain words 0xff, 0x84, 0.
 */
void V34SetINFO0aBits(V34Obj* obj, unsigned short* info)
{
  PcmFlo47Obj* floobj = (PcmFlo47Obj*)obj->ctx_;
  const char* who;

  if (floobj->flag6120_ == 0)
  {
    if (obj->pcmMode_ != 0)
    {
      P47V90InfoObj* p1760;
      short u;
      uint16_t w3;

      if (dsplibs_debug_level > 1)
        dsplibs_debug_printf("setINFO0aBits - setting info0d (Digital) for V.PCM\n");
      info[0] = 0xff;
      info[1] = 0x84;
      if (obj->wabca_ != 0)
      {
        if (dsplibs_debug_level > 1)
          dsplibs_debug_printf("...requesting short phase2...\n");
        info[1] = (unsigned short)(info[1] | 2);
      }
      if (obj->wabc6_ != 0)
      {
        if (dsplibs_debug_level > 1)
          dsplibs_debug_printf("...indicating V.92 capabilities...\n");
        info[1] = (unsigned short)(info[1] | 1);
      }

      info[2] = 0x30;
      u = (short)bitreverse(((P47V90InfoObj*)floobj->v90Info_)->maxTx_, 5);
      info[2] = (unsigned short)(info[2] | (((int)u >> 2) & 7));
      w3 = (uint16_t)(((int)u << 6) & 0xc0);
      p1760 = (P47V90InfoObj*)floobj->v90Info_;
      if (p1760->txPoint_ == 1)
        w3 |= 0x20;
      info[3] = w3;
      if (p1760->pcmType_ == 1)
        info[3] = (unsigned short)(info[3] | 0x10);
      info[3] = (unsigned short)(info[3] | 8);
      return;
    }

    if (dsplibs_debug_level > 1)
    {
      who = (obj->k56Mode_ != 0) ? "K.56Flex (same as V.34)" : "V.34";
      dsplibs_debug_printf("setINFO0aBits - setting info0 (Caller) for %s\n", who);
    }
  }
  else
  {
    if (obj->pcmMode_ != 0)
    {
      if (dsplibs_debug_level > 1)
        dsplibs_debug_printf("setINFO0aBits - setting info0a for V.PCM\n");
      info[0] = 0xff;
      info[1] = 0x84;
      if (obj->wabca_ != 0)
      {
        unsigned int lvl;

        if (dsplibs_debug_level > 1)
          dsplibs_debug_printf("...requesting short phase2...\n");
        lvl = dsplibs_debug_level;
        info[1] = (unsigned short)(info[1] | 1);
        obj->prevBulkDelay_ = 0x14;
        if (lvl > 1)
          dsplibs_debug_printf("Setting prev bulk delay = %d\n", 0x14);
      }
      if (((P47V92InfoObj*)floobj->v92Info_)->v92CapLocal_ != 0)
      {
        if (dsplibs_debug_level > 1)
          dsplibs_debug_printf("...indicating V.92 capabilities...\n");
        info[1] = (unsigned short)(info[1] | 2);
      }
      info[2] = 0;
      return;
    }

    if (dsplibs_debug_level > 1)
    {
      who = (obj->k56Mode_ != 0) ? "K.56Flex (same as V.34)" : "V.34";
      dsplibs_debug_printf("setINFO0aBits - setting info0a for %s\n", who);
    }
  }

  info[0] = 0xff;
  info[1] = 0x84;
  info[2] = 0;
}

/* ---- V34InitializeImplementationSpecific ---- */

/**
 * @brief Initialise the pointers and lengths of the two embedded echo-canceller records and the modulator bank pointer.
 * @param obj  the V.34 object
 *
 * Pure pointer / length initialisation of the near and far EchoAdaptState records (layout in v34_dsp_primitives.c: writePtr +0, base +4, coefHi +8,
 * coefLo +0xc, history +0x10, delay-line length +0x18, N +0x1c) and of the modulator's polyphase bank pointer `p2074_`.  The write pointer of each
 * record is set from the OLD value of its base field before the base is overwritten; field +0x14 of each record is not written.
 */
void V34InitializeImplementationSpecific(V34Obj* obj)
{

  obj->mod.p2074_ = obj->mod.unk146c_;

  /* near echo canceller */
  obj->echo0.w80d0_ = 0x678;
  obj->echo0.p80c8_ = obj->unk80d8_ + 3600;
  obj->echo0.p80b8_ = obj->echo0.p80bc_;
  obj->echo0.p80bc_ = obj->unk80d8_ + 288;
  obj->echo0.echoBuf_ = (int16_t*)(obj->unk80d8_ + 3904);
  obj->echo0.echoCnt_ = 0x90;
  obj->echo0.p80c4_ = obj->unk80d8_;

  /* far echo canceller */
  obj->echo1.w80d0_ = 0x678;
  obj->echo1.p80b8_ = obj->echo1.p80bc_;
  obj->echo1.p80c8_ = obj->unk9158_ + 3600;
  obj->echo1.echoBuf_ = (int16_t*)(obj->unk9158_ + 3904);
  obj->echo1.p80bc_ = obj->unk9158_ + 288;
  obj->echo1.p80c4_ = obj->unk9158_;
  obj->echo1.echoCnt_ = 0x90;
}
