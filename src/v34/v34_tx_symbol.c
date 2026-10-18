/**
 * @file v34_tx_symbol.c
 * @brief V.34 transmit symbol path: bit fetch for the modulator (getFrame) and symbol output (txmit, txmitdibit, txmitquadbit).
 *
 * Entry points:
 *  - getFrame(): reads the next frame's worth of bits from the transmit bit source (a scrambler function reached through the object's `p2a28_` slot,
 *    scrambleGPA or scrambleGPC) and splits them into the shell index, the mapper bit groups and the sign/LSB fields in the object's frame state.  It is the
 *    transmit-side mirror of putFrame() (v34_frame_decoder.c).  Called by the modulator (v34_modulator.c).
 *  - txmit(): runs one transmit symbol through the modulator (V34ModulatorProcess), appends the scaled output to the transmit history ring, feeds the echo
 *    canceller pre-filter and, when enabled, the bulk-delay ring and the near / far echo delay lines.
 *  - txmitdibit() / txmitquadbit(): scramble 2 or 4 data bits, select the constellation point (vect4 / vect16) and call txmit().  They are used by the
 *    V.90 / K56flex phase 3 / 4 and rate-renegotiation state machines in src/vpcm; txmit() is also called from the handshake and from modulatevector().
 *
 * Both entry-point families work on the V.34 object through the typed view V34Obj.  Hazards: see getFrame() (table index not range-checked).
 */

#include <stddef.h>
#include <stdint.h>

#include "v34_symbol_tables.h" /* lsbMask, vect4, vect16 */
#include "v34_internal.h"

/* s16_t / u16_t (the byte-addressed sample ring of txwritequeue) */
#include "v34_access.h"

/* typed view of the V.34 object (V34Obj, member keys V34M_<off>) */
#include "v34_object_view.h"

/* ====================================================================== */
/* getFrame                                                               */
/* ====================================================================== */

/**
 * @brief lsbMask[i] with a bound: the table for 0 .. 16, 0 for any other index.
 * @param i  bit count
 * @return the mask of the low `i` bits, or 0 when `i` is outside the 17-entry table
 */
static unsigned int lsb_at(int i)
{
  if (i >= 0 && i <= 16)
    return lsbMask[i];
  return 0;
}

/** The transmit bit source: scrambleGPA / scrambleGPC, called as fn(obj, count) and returning the new bit count (16 bits significant). */
typedef int (*gf_fn_t)(void* obj, int count);

/**
 * @brief Fetch the bits of one frame from the scrambled bit source and split them into the frame state members.
 * @param obj  the V.34 object (V34Obj); `p2a28_` must hold the bit-source function
 *
 * Inputs, from the object's frame parameter block (w25e0_ ...): `w25e0_` N and `w25e6_` / `w25e8_` (a running position that advances by w25e6_ and wraps
 * at N; the frame's bit count n is `w25f0_` while it stays below N and `w25ee_` on the step that wraps), `w25e4_` and `w25f4_` W (the width of the last two
 * fields of each group).  The bit reader is a 32-bit buffer `w2a60_` and a 16-bit bit count `w2a64_`; the source function is called whenever the count
 * exceeds 15 and refills the buffer.
 *
 * Outputs: the first n bits go to `shellIdx_` (n > 16: a 16-bit word plus a second word of n & 15 bits at +2; n <= 0: zero), then four groups of four
 * fields each, `w2a34_` ... `w2a52_`: per group one bit, a t1-bit field and two W-bit fields (the last group uses t2 = t1 + n instead of t1).  t1 is 2,
 * except in the n <= 0 case, where it is 1 when `w25e4_` is below 9.
 *
 * @note All bit-count arithmetic is 16-bit signed, as is the return value of the source function; shift counts are masked to 5 bits.
 * @note The four groups share one snapshot of the bit buffer which is re-read after every refill.
 * @warning `lsbMask` has 17 entries; indices outside 0 .. 16 yield mask 0 (lsb_at()), which is memory-safe.  The only ways to get there are W (`w25f4_`) > 16
 *          and t2 = t1 + n <= 0 (n <= 0 arm), which the set-up routine does not produce (W in 0 .. ~8, n in -1 .. 30).
 */
void getFrame(V34Obj* obj)
{
  gf_fn_t fn = (gf_fn_t)obj->tx.p2a28_;
  int t1 = 2;
  int t2 = 2;
  int W = (int16_t)obj->tx.w25f4_;
  uint32_t sum = (uint32_t)obj->tx.w25e8_ + (uint32_t)obj->tx.w25e6_;
  uint16_t N = obj->tx.w25e0_;
  int n;
  int c; /* current bit count, always a sign-extended short */
  int sv4, sv5;
  uint32_t snap;
  uint32_t x, y;
  unsigned int m1, mW;

  /* advance the running position (unsigned 16-bit compare of N against the new position) and pick this frame's bit count */
  if (N > (uint16_t)sum)
  {
    obj->tx.w25e8_ = (uint16_t)sum;
    n = obj->tx.w25f0_;
  }
  else
  {
    obj->tx.w25e8_ = (uint16_t)(sum - N);
    n = obj->tx.w25ee_;
  }
  c = (int16_t)obj->tx.w2a64_;

  /* refill while the count exceeds 15 (signed 16-bit) */
  while (c > 0xf)
  {
    c = (int16_t)fn(obj, c);
    obj->tx.w2a64_ = (int16_t)c;
  }

  if (n > 0x10)
  {
    /* n > 16: first a 16-bit word (only the low half of shellIdx_ is written), then a refill and a second word of n & 15 bits */
    obj->tx.shellIdx_ = (obj->tx.shellIdx_ & 0xffff0000u) | (uint16_t)(obj->tx.w2a60_ >> (c & 31));
    fn(obj, 0); /* the return value is ignored and the count is not updated from it */
    {
      int cc = (int16_t)obj->tx.w2a64_;
      uint32_t buf = obj->tx.w2a60_;
      int k = n & 0xf;
      obj->tx.shellIdx_ = (obj->tx.shellIdx_ & 0xffffu) | ((uint32_t)(uint16_t)((buf >> (cc & 31)) & lsb_at(k)) << 16);
      c = (int16_t)(k + obj->tx.w2a64_);
      obj->tx.w2a64_ = (int16_t)c;
    }
    sv4 = (int16_t)(t1 + 2 * W + 1);
    sv5 = (int16_t)(t2 + 2 * W + 1);
  }
  else if (n > 0)
  {
    uint32_t buf = obj->tx.w2a60_;
    int c2 = (int16_t)(n + c);
    obj->tx.w2a64_ = (int16_t)c2;
    obj->tx.shellIdx_ = (buf >> (c & 31)) & lsb_at(n);
    c = c2;
    sv4 = (int16_t)(t1 + 2 * W + 1);
    sv5 = (int16_t)(t2 + 2 * W + 1);
  }
  else
  {
    obj->tx.shellIdx_ = 0;
    t1 = 2 - (obj->tx.w25e4_ < 9 ? 1 : 0);
    t2 = (int16_t)(t1 + n); /* sign-extended: can be <= 0, see the @warning above */
    sv4 = (int16_t)(t1 + 2 * W + 1);
    sv5 = (int16_t)(t2 + 2 * W + 1);
  }

  if (c > 0xf)
  {
    c = (int16_t)fn(obj, c);
    obj->tx.w2a64_ = (int16_t)c;
  }

  snap = obj->tx.w2a60_;
  m1 = lsb_at(t1);
  mW = lsb_at(W);

  /* group 0: a bit, a t1-bit field and two W-bit fields from the snapshot */
  x = snap >> (c & 31);
  obj->tx.w2a34_ = (uint16_t)(x & 1);
  y = x >> 1;
  obj->tx.w2a36_ = (uint16_t)(m1 & y);
  y >>= (t1 & 31);
  obj->tx.w2a3a_ = (uint16_t)((y >> (W & 31)) & mW);
  obj->tx.w2a38_ = (uint16_t)(y & mW);
  c = (int16_t)(sv4 + c);
  obj->tx.w2a64_ = (int16_t)c;
  if (c > 0xf)
  {
    c = (int16_t)fn(obj, c);
    snap = obj->tx.w2a60_;
    obj->tx.w2a64_ = (int16_t)c;
  }

  /* groups 1 and 2 split the next bits the same way; the buffer is refilled when the count passes 15 */

  /* group 1 */
  c = (int16_t)obj->tx.w2a64_;
  x = snap >> (c & 31);
  y = x >> 1;
  obj->tx.w2a3e_ = (uint16_t)(m1 & y);
  obj->tx.w2a3c_ = (uint16_t)(x & 1);
  y >>= (t1 & 31);
  c = (int16_t)(c + sv4);
  obj->tx.w2a64_ = (int16_t)c;
  obj->tx.w2a40_ = (uint16_t)(y & mW);
  obj->tx.w2a42_ = (uint16_t)((y >> (W & 31)) & mW);
  if (c > 0xf)
  {
    c = (int16_t)fn(obj, c);
    snap = obj->tx.w2a60_;
    obj->tx.w2a64_ = (int16_t)c;
  }

  /* group 2 */
  c = (int16_t)obj->tx.w2a64_;
  x = snap >> (c & 31);
  y = x >> 1;
  obj->tx.w2a46_ = (uint16_t)(m1 & y);
  obj->tx.w2a44_ = (uint16_t)(x & 1);
  y >>= (t1 & 31);
  c = (int16_t)(c + sv4);
  obj->tx.w2a64_ = (int16_t)c;
  obj->tx.w2a48_ = (uint16_t)(y & mW);
  obj->tx.w2a4a_ = (uint16_t)((y >> (W & 31)) & mW);
  if (c > 0xf)
  {
    c = (int16_t)fn(obj, c);
    snap = obj->tx.w2a60_;
    obj->tx.w2a64_ = (int16_t)c;
  }

  /* group 3: uses t2 / sv5 for its second field and the advance; no refill afterwards */
  c = (int16_t)obj->tx.w2a64_;
  x = snap >> (c & 31);
  y = x >> 1;
  obj->tx.w2a4c_ = (uint16_t)(x & 1);
  obj->tx.w2a4e_ = (uint16_t)(lsb_at(t2) & y);
  y >>= (t2 & 31);
  obj->tx.w2a50_ = (uint16_t)(y & mW);
  obj->tx.w2a52_ = (uint16_t)((y >> (W & 31)) & mW);
  obj->tx.w2a64_ = (int16_t)(c + sv5);
}

/* ====================================================================== */
/* txmit                                                                  */
/* ====================================================================== */

/**
 * @brief Run one transmit symbol through the modulator and distribute the output samples.
 * @param obj  the V.34 object (V34Obj)
 *
 * Steps:
 *  1. n = V34ModulatorProcess(modulator, `txPoint_`, buf): up to 38 output samples for the constellation point in `txPoint_`.
 *  2. For n > 0: each sample, scaled as (sample * `txGain_` + 0x2000) >> 14, is appended as a {sample, 0} pair to the transmit history ring (write pointer
 *     `p2224_`, ring from member 0x2228 up to `txCnt_`) and `txIdx_` is incremented; the new write pointer is stored once, after the loop.
 *  3. V34EchoPreFilter(buf, n, pre-filter) always runs, also for n <= 0.
 *  4. If bit 2 of the high byte of `txFlags_` is set and n > 0: the bulk-delay ring (`bulkBase_`, `bulkLen_`, read index `bulkRd_`, write index `bulkWr_`,
 *     indices in 16-bit samples) is advanced once per sample, the sample is written at the write index, and the near echo delay line is updated with
 *     sample / 2 and the far one with the sample read from the ring / 2.
 *
 * @note The write pointer `p2224_` is a real pointer; the bulk-delay indices are unsigned 32-bit values scaled by 2.
 * @warning `p2224_` and `bulkBase_` must point to valid storage and `bulkRd_` / `bulkWr_` below `bulkLen_`; none of this is checked here.
 */
void txmit(V34Obj* obj)
{
  short buf[38];
  int n = (short)V34ModulatorProcess(&obj->mod, obj->tx.txPoint_, buf);
  int i;

  if (n > 0)
  {
    unsigned char* w = (unsigned char*)obj->tx.p2224_;
    uintptr_t end = (uintptr_t)&obj->tx.txCnt_;

    /* transmit history ring */
    i = 0;
    do
    {
      int s = buf[i];
      unsigned char* p;

      obj->tx.txIdx_ = (uint16_t)(obj->tx.txIdx_ + 1);
      p = w + 4;
      *(s16_t*)w = (int16_t)((s * (int)obj->tx.txGain_ + 0x2000) >> 14);
      *(s16_t*)(w + 2) = 0;
      if (end > (uintptr_t)p)
        w = p;
      else
        w = obj->tx.unk2228_;
      i = (int16_t)(i + 1);
    } while (i < n);
    obj->tx.p2224_ = w;
  }

  V34EchoPreFilter(buf, (short)n, &obj->mod.pf);

  /* bulk-delay ring and echo delay lines */
  if ((obj->tx.txFlags_ & 0x200) != 0 && n > 0)
  {
    i = 0;
    do
    {
      uint32_t rd = obj->bulkRd_;
      uint32_t len = obj->bulkLen_;
      uintptr_t base = (uintptr_t)obj->bulkBase_;
      int old = *(s16_t*)(base + 2u * (uintptr_t)rd);
      uint32_t wr;

      rd = rd + 1;
      obj->bulkRd_ = (len > rd) ? rd : 0;
      wr = obj->bulkWr_;
      *(s16_t*)(base + 2u * (uintptr_t)wr) = buf[i];
      wr = wr + 1;
      obj->bulkWr_ = (len > wr) ? wr : 0;
      V34EchoUpdateDelayLine(&obj->echo0, (short)(buf[i] >> 1));
      V34EchoUpdateDelayLine(&obj->echo1, (short)(old >> 1));
      i = (int16_t)(i + 1);
    } while (i < n);
  }
}

/* ====================================================================== */
/* txmitdibit / txmitquadbit                                              */
/* ====================================================================== */

/**
 * @brief Two steps of the transmit scrambler, shared by txmitdibit() and txmitquadbit().
 * @param sh    scrambler state (32-bit register)
 * @param bit0  first input bit (only bit 0 is used)
 * @param bit1  second input bit (only bit 0 is used)
 * @param tap   feedback tap mask: 0x4000000 or 0x2000
 * @return the new scrambler state
 *
 * For each input bit: the parity of the bit, (`sh` & `tap`) and (`sh` & 0x100) is computed; when odd, bit 31 is set; then the register is shifted right
 * logically by one.  The new bit therefore ends up in bit 30 and bit 31 is always clear afterwards.
 */
static uint32_t scramble2(uint32_t sh, unsigned int bit0, unsigned int bit1, uint32_t tap)
{
  unsigned int bits[2];
  int j;

  bits[0] = bit0;
  bits[1] = bit1;
  for (j = 0; j < 2; j++)
  {
    unsigned int b = bits[j] & 1u;
    if ((sh & tap) != 0)
      b += 1;
    if ((sh & 0x100u) != 0)
      b += 1;
    if ((b & 1u) != 0)
      sh |= 0x80000000u;
    sh >>= 1;
  }
  return sh;
}

/**
 * @brief Transmit two data bits as one constellation point of vect4.
 * @param obj   the V.34 object (V34Obj)
 * @param bits  the two bits (bit 0 first, then bit 1; higher bits ignored)
 *
 * The scrambler (`scrState_`) is stepped twice; its tap is 0x4000000 when bit 0 of `txFlags_` is set and 0x2000 otherwise.  The point index is
 * k = ((`scrState_` >> 29) + `txPrev_`) & 3, stored in `txBits_` and `txPrev_`; vect4[k] goes to `txPoint_` and txmit() sends it.
 */
void txmitdibit(V34Obj* obj, int bits)
{
  uint32_t tap = (obj->tx.txFlags_ & 1u) ? 0x4000000u : 0x2000u;
  uint32_t sh = obj->tx.scrState_;
  unsigned int k;

  sh = scramble2(sh, (unsigned int)bits, (unsigned int)bits >> 1, tap);
  obj->tx.scrState_ = sh;
  k = ((sh >> 29) + (uint32_t)obj->tx.txPrev_) & 3u;
  obj->tx.txBits_ = (uint16_t)k;
  obj->tx.txPrev_ = (uint16_t)k;
  obj->tx.txPoint_ = (uint32_t)vect4[k];
  txmit(obj);
}

/**
 * @brief Transmit four data bits as one constellation point of vect16.
 * @param obj   the V.34 object (V34Obj)
 * @param bits  the four bits (bit 0 first; higher bits ignored)
 *
 * The first two bits step the scrambler (`scrState_`) as in txmitdibit() and give k = ((`scrState_` >> 29) + `txPrev_`) & 3, stored in `txBits_`.  The
 * other two bits step it again with the same tap (selected once, from `txFlags_`).  Then `txPrev_` = k, `txPoint_` = vect16[(`scrState_` >> 29) + 4 * k]
 * and txmit() sends it.
 *
 * @note The index is always 0 .. 15: bit 31 of the scrambler register is clear after the logical shift, so `scrState_` >> 29 is 0 .. 3, and k is 0 .. 3.
 */
void txmitquadbit(V34Obj* obj, int bits)
{
  uint32_t w = obj->tx.txFlags_;
  uint32_t tap = (w & 1u) ? 0x4000000u : 0x2000u;
  uint32_t sh = obj->tx.scrState_;
  unsigned int k;

  sh = scramble2(sh, (unsigned int)bits, (unsigned int)bits >> 1, tap);
  obj->tx.scrState_ = sh;
  k = ((sh >> 29) + (uint32_t)obj->tx.txPrev_) & 3u;
  obj->tx.txBits_ = (uint16_t)k;
  sh = scramble2(sh, (unsigned int)bits >> 2, (unsigned int)bits >> 3, tap);
  obj->tx.scrState_ = sh;
  obj->tx.txPrev_ = (uint16_t)k;
  /* (sh >> 29) is 0 .. 3 and k is 0 .. 3, so the index is 0 .. 15 */
  obj->tx.txPoint_ = (uint32_t)vect16[(sh >> 29) + 4u * k];
  txmit(obj);
}
