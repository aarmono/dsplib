/**
 * @file v34_handshake_helpers.c
 * @brief V.34 handshake helpers: the data-rate negotiation result (initdigital, setfinalrate), the bulk-delay ring reset and a CRC-protected bit reader.
 *
 * Four functions that the V.34 / V.90 / K56Flex phase routines call once the INFO exchange has fixed the symbol rates:
 * - setfinalrate(obj): decodes the bit-reversed rate fields of the INFO words into the transmit and receive symbol rates, carriers, pre-emphasis index and
 *   the scale / detector table pointers of the rate block (V34Rate, `obj->rate`).
 * - initdigital(obj): combines the negotiated transmit / receive rate indices with the rate bitmap, trims them to a rate both ends support, derives the
 *   number of transmit bits per symbol interval and initialises the transmit and receive bit-allocation blocks through initV34().  It is shared by the
 *   V.34, V.90 and K56Flex phase code.
 * - ApplyBulkDelay(obj, n): resets the bulk-delay ring of the transmitter to a new length and adjusts the far-echo canceller flag and the DMA delay.
 * - getbit(reader): MSB-first bit reader with an on-the-fly CRC-16-CCITT, used to serialise the INFO sequences.
 *
 * The V.34 object is reached through the typed view V34Obj (v34_object_view.h); the reader of getbit() is a bit-queue record (struct tag_bitQ) whose data words
 * are indexed by byte offset (layout in the comment before getbit()).
 *
 * getbit() and ApplyBulkDelay() are NOT declared in v34_internal.h: the handshake pieces that call them declare them with a register-parameter
 * convention on 32-bit builds (see dsplib.mk); here they are ordinary C functions.
 *
 * Scale-table reads.  initdigital() indexes the 28-entry scale tables with `Tn + 14 * hi - 1`.  The index is -1 (Tn == 0, hi == 0) or 28 (Tn == 15,
 * hi == 1) for the extreme rate indices.  scale_at() returns 0 for any index outside 0..27 and never reads outside the table, so a corrupt rate
 * index cannot read outside it.
 */

#include <limits.h>
#include <stddef.h>
#include <stdint.h>

#include "v34_symbol_tables.h" /* scale2400.., c1600.. */
#include "v34_symbol_rates.h"  /* V34_BAUD_*, V34_CARRIER_* */
#include "core/dsp_host.h"
#include "v34_internal.h"
#include "v34_initv34.h" /* initV34 */

#include "v34_access.h" /* U16 (the reader's data words), AS_U16, s16_t */
#include "v34_role.h"   /* V34ROLE_* (the values of modeChar_) */
#include "v34_object_view.h"

/**
 * @brief Reset the transmitter's bulk-delay ring to a new length and re-evaluate the far-echo canceller flag.
 * @param obj   the V.34 module object
 * @param nArg  new bulk delay in samples; only the low 16 bits are used, as a signed value.  A value <= 0 is replaced by 144 (0x90); a value not below
 *              the ring capacity (`bulkLen_`, unsigned compare) is replaced by 0.
 * @note  Sets the write index to `n`, the read index to 0 and clears the first `n` 16-bit entries of the ring storage.  Then, when neither the V.PCM nor
 *        the K56Flex mode is active and the far-echo canceller flag (`echoOn_`) is set: for a delay of at most 29 samples the flag is cleared and the
 *        DMA delay is reduced by min(n + 15, 30); otherwise the flag stays set.  In every other case the flag is cleared.
 *        Debug messages are printed when `dsplibs_debug_level` is above 1; the guard of the "Modifying dma delay" message uses the level read around
 *        the "RTD" message.
 */
void ApplyBulkDelay(V34Obj* obj, int nArg)
{
  int n = (int16_t)nArg;
  uint32_t v;
  unsigned int lv;

  /* replace an unusable estimate by the default of 144 samples, then by 0 when it does not fit the ring */
  if (n <= 0)
  {
    if (1 < dsplibs_debug_level)
      dsplibs_debug_printf("V34 bulk delay first estimation %d\n", n);
    n = 0x90;
  }

  v = (uint32_t)n;
  if (!(v < obj->bulkLen_))
  { /* unsigned compare against the ring capacity */
    if (1 < dsplibs_debug_level)
      dsplibs_debug_printf("V34 bulk delay first estimation %d\n", (int)v);
    v = 0;
  }

  /* restart the ring */
  obj->bulkWr_ = v;
  obj->bulkRd_ = 0;
  sysdep_memset(obj->unk35b8_, 0, 2u * v);

  /* a short round-trip delay masks the far-end echo canceller and gives its delay back to the DMA delay; the flag is only kept (and set) in plain V.34 mode */
  if (obj->pcmMode_ == 0 && obj->k56Mode_ == 0 && (int16_t)obj->echoOn_ != 0)
  {
    if ((int16_t)v <= 0x1d)
    {
      int w;

      lv = dsplibs_debug_level;
      if (1 < lv)
      {
        dsplibs_debug_printf("RTD (%d) lower than min (%d), masking Far EC...\r\n", (int)v, 0x1e);
        lv = dsplibs_debug_level;
      }

      w = (int16_t)(v + 0xf);
      obj->echoOn_ = 0;
      if (w > 0x1e)
        w = 0x1e;

      if (1 < lv)
      {
        dsplibs_debug_printf("...Modifying dma delay from %d to %d\r\n", (int)(int16_t)obj->dmaDelay_, (int)(int16_t)obj->dmaDelay_ - w);
      }
      obj->dmaDelay_ = (uint16_t)(obj->dmaDelay_ - (unsigned int)w);
    }
    else
    {
      obj->echoOn_ = 1;
    }
  }
  else
  {
    obj->echoOn_ = 0;
  }

  if (1 < dsplibs_debug_level)
    dsplibs_debug_printf("V34 bulk delay estimation %d (FAR=%d)\n", (int)v, (int)(int16_t)obj->echoOn_);
}

/**
 * @brief Return the next bit of a bit stream, MSB first, maintaining a CRC-16-CCITT (polynomial 0x1021, initial value USHRT_MAX) over the bits handed out.
 * @param q  reader record (a tag_bitQ; byte offsets):
 *           - +0x00 ushort data[]: the words to read, indexed by the signed 16-bit word index at +0x1e (the index can therefore address the reader's own
 *             fields; stores and loads in the function are ordered accordingly)
 *           - +0x14 ushort crc, +0x16 short crcOn, +0x18 short total (bits in the stream), +0x1a ushort pos (bits consumed), +0x1c short chunk (bits
 *             loaded per word), +0x1e short widx, +0x20 short loop, +0x22 ushort pass (restart count)
 *           - +0x24 u32 acc (bit accumulator), +0x28 short nbits (bits left in acc), +0x2a short / +0x2c u32 values reloaded on a restart
 * @return 0 or 1, the next bit; -1 at the end of a stream whose `loop` flag is 0.
 * @note  When `acc` is empty, d = (short)(total - pos) selects the next step (all counts are 16-bit signed, shift counts are taken modulo 32):
 *        - d > 0: load a chunk of n = chunk bits (widx is incremented) if chunk <= d, else the partial n = d bits (widx unchanged);
 *          acc = (acc << n) | data[widx]; pos += n; when crcOn, the CRC is updated over the n new bits (a loop on a 16-bit counter that starts at n - 1
 *          and stops after 0, so any n terminates);
 *        - d == 0 and crcOn: the 16 CRC bits are delivered (pos += 16);
 *        - d == -16: a 4-bit tail of ones (0xf) is delivered (pos += 4); this is the state after the CRC;
 *        - otherwise: with `loop` == 0 the end is reported (-1); else the reader restarts (crc = USHRT_MAX, pos = 0, ++pass, widx = 0, acc and nbits reloaded
 *          from +0x2c / +0x2a) and getbit() calls itself.
 * @warning A restarted reader that needs a restart again (nbits == 0 and (short)total <= 0 but neither 0 with crcOn nor -16) recurses without end and
 *          overflows the stack.  Valid streams never reach this state.
 * @warning The word index `widx` (signed 16 bits) addresses the data words by byte offset from the start of the record; it is not range-checked, so the
 *          bound of the words read is not established here.
 */
int getbit(struct tag_bitQ* q)
{
  unsigned int nbits = q->tail_.nbits;
  uint32_t acc;

  /* the accumulator is empty: decide what comes next (data chunk, CRC, tail, end or restart) */
  if (nbits == 0)
  {
    uint16_t pos = q->tail_.pos;
    int d = (int16_t)(uint16_t)(q->tail_.total - pos);
    int n; /* 16-bit signed count of the bits just loaded (the CRC loop's start) */

    /* end of the data: CRC, tail or restart */
    if (d <= 0)
    {
      if (d == 0 && (int16_t)q->tail_.crcOn != 0)
      { /* the CRC, 16 bits */
        acc = q->tail_.crc;
        q->tail_.nbits = 0x10;
        q->tail_.pos = (uint16_t)(pos + 0x10);
        q->tail_.acc = acc;
        nbits = 0x10;
        goto emit;
      }
      if (d == -16)
      { /* 4-bit tail 0xf */
        q->tail_.acc = 0xf;
        q->tail_.nbits = 4;
        q->tail_.pos = (uint16_t)(pos + 4);
        acc = 0xf;
        nbits = 4;
        goto emit;
      }
      if ((int16_t)q->tail_.loop == 0)
        return -1;

      /* restart the stream */
      q->tail_.crc = USHRT_MAX;
      q->tail_.pos = 0;
      q->tail_.pass = (uint16_t)(q->tail_.pass + 1);
      q->tail_.widx = 0;
      q->tail_.acc = q->tail_.restartAcc;
      q->tail_.nbits = q->tail_.restartBits;
      return (int16_t)getbit(q);
    }

    /* load the next chunk of data bits into the accumulator */
    {
      int chunk = (int16_t)q->tail_.chunk;
      if (chunk <= d)
      { /* signed 16-bit compare */
        uint32_t a = q->tail_.acc << ((unsigned int)chunk & 31);
        uint16_t widx = q->tail_.widx;
        uint32_t dat;
        q->tail_.nbits = (uint16_t)chunk; /* stored before the data word is read (widx can point here) */
        dat = U16(q, 2 * (int)(int16_t)widx);
        q->tail_.widx = (uint16_t)(widx + 1);
        q->tail_.pos = (uint16_t)(pos + (uint16_t)chunk);
        acc = a | dat;
        q->tail_.acc = acc;
        nbits = (uint16_t)chunk;
        n = chunk;
      }
      else
      { /* partial last chunk: n = d, widx NOT advanced */
        uint32_t a;
        uint32_t dat;
        int16_t widx = (int16_t)q->tail_.widx;
        q->tail_.nbits = (uint16_t)d;
        a = q->tail_.acc << ((unsigned int)d & 31);
        dat = U16(q, 2 * (int)widx);
        q->tail_.pos = (uint16_t)(d + pos);
        acc = a | dat;
        q->tail_.acc = acc;
        nbits = (uint16_t)d;
        n = d;
      }
    }

    /* fold the n new bits into the CRC, most significant first */
    if ((int16_t)q->tail_.crcOn != 0 && n != 0)
    {
      int16_t k = (int16_t)(n - 1);
      for (;;)
      {
        uint32_t crc = q->tail_.crc;
        uint32_t b = (crc >> 15) & 1;
        uint32_t nc = crc * 2;
        if ((acc >> ((unsigned int)k & 31)) & 1)
          b ^= 1;
        if (b)
          nc ^= 0x1021;
        q->tail_.crc = (uint16_t)nc;
        if (k == 0)
          break;
        k = (int16_t)(k - 1);
      }
    }
  }
  else
  {
    acc = q->tail_.acc;
  }

emit:
  /* hand out the top remaining bit of the accumulator */
  {
    int16_t nn = (int16_t)(nbits - 1);
    q->tail_.nbits = (uint16_t)nn;
    return (int)((acc >> ((unsigned int)nn & 31)) & 1);
  }
}

/**
 * @brief Decode the transmit and receive symbol rates from the INFO words into the rate block.
 * @param obj  the V.34 module object
 * @note  Reads the INFO words at +0xa9de / +0xa9e0 / +0xa9e2 and writes the pre-emphasis index (`preemp_`), then for each direction the baud rate, the carrier
 *        frequency and the scale-table pointer (`scalePtr` for transmit, `scaleTab` for receive) of the rate block (V34Rate, `obj->rate`); the receive side
 *        also selects the carrier-detector table (`detTab`).  Each direction is driven by a bit-reversed field: 0 = 2400, 2 = 2800, 3 = 3000, 4 = 3200
 *        and 5 = 3429 baud (the other field values store nothing).  The carrier choice (low or high) reads a different INFO bit per rate: bit 2 of
 *        +0xa9de for all transmit rates, and bit 2 of +0xa9ae, bit 0 of +0xa9b2, bit 7 of +0xa9b6 and bit 6 of +0xa9b8 for the receive rates 2400, 2800,
 *        3000 and 3200 baud.  3429 baud has a single carrier.
 */
void setfinalrate(V34Obj* obj)
{
  int s, b;

  /* pre-emphasis index */
  obj->rate.preemp_ = bitreverse((unsigned short)(((obj->info1c.wa9de_ & 3u) << 2) | ((obj->info1c.wa9e0_ & 0xc0u) >> 6)), 4);

  /* transmit symbol rate, carrier and scale table */
  s = (int16_t)bitreverse((unsigned short)((obj->info1c.wa9e2_ >> 4) & 7u), 3);
  if (s == 0)
  {
    obj->rate.baud_ = V34_BAUD_2400;
    obj->rate.scalePtr_ = scale2400;
    obj->rate.carrier_ = (obj->info1c.wa9de_ & 4) ? V34_CARRIER_1800 : V34_CARRIER_1600;
  }
  else if (s == 2)
  {
    obj->rate.scalePtr_ = scale2800;
    obj->rate.baud_ = V34_BAUD_2800;
    obj->rate.carrier_ = (obj->info1c.wa9de_ & 4) ? V34_CARRIER_1867 : V34_CARRIER_1680;
  }
  else if (s == 3)
  {
    obj->rate.scalePtr_ = scale3000;
    obj->rate.baud_ = V34_BAUD_3000;
    obj->rate.carrier_ = (obj->info1c.wa9de_ & 4) ? V34_CARRIER_2000 : V34_CARRIER_1800;
  }
  else if (s == 4)
  {
    obj->rate.baud_ = V34_BAUD_3200;
    obj->rate.scalePtr_ = scale3200;
    obj->rate.carrier_ = (obj->info1c.wa9de_ & 4) ? V34_CARRIER_1920 : V34_CARRIER_1829;
  }
  else if (s == 5)
  {
    obj->rate.scalePtr_ = scale3429;
    obj->rate.baud_ = V34_BAUD_3429;
    obj->rate.carrier_ = V34_CARRIER_1959;
  }

  /* receive symbol rate, carrier, scale table and carrier-detector table */
  b = (int16_t)bitreverse((unsigned short)(((obj->info1c.wa9e0_ * 2u) & 7u) | ((obj->info1c.wa9e2_ >> 7) & 1u)), 3);
  if (b == 0)
  {
    obj->rate.scaleTab_ = scale2400;
    obj->rate.rxBaud_ = V34_BAUD_2400;
    if (obj->info.wa9ae_ & 4)
    {
      obj->rate.rxCarrier_ = V34_CARRIER_1800;
      obj->rate.detTab_ = c1800_;
    }
    else
    {
      obj->rate.rxCarrier_ = V34_CARRIER_1600;
      obj->rate.detTab_ = c1600;
    }
  }
  else if (b == 2)
  {
    obj->rate.scaleTab_ = scale2800;
    obj->rate.rxBaud_ = V34_BAUD_2800;
    if (obj->info.wa9b2_ & 1)
    {
      obj->rate.rxCarrier_ = V34_CARRIER_1867;
      obj->rate.detTab_ = c1867;
    }
    else
    {
      obj->rate.rxCarrier_ = V34_CARRIER_1680;
      obj->rate.detTab_ = c1680;
    }
  }
  else if (b == 3)
  {
    obj->rate.scaleTab_ = scale3000;
    obj->rate.rxBaud_ = V34_BAUD_3000;
    if (obj->info.wa9b6_ & 0x80)
    {
      obj->rate.rxCarrier_ = V34_CARRIER_2000;
      obj->rate.detTab_ = c2000;
    }
    else
    {
      obj->rate.rxCarrier_ = V34_CARRIER_1800;
      obj->rate.detTab_ = c1800_;
    }
  }
  else if (b == 4)
  {
    obj->rate.scaleTab_ = scale3200;
    obj->rate.rxBaud_ = V34_BAUD_3200;
    if (obj->info.wa9b8_ & 0x40)
    {
      obj->rate.rxCarrier_ = V34_CARRIER_1920;
      obj->rate.detTab_ = c1920;
    }
    else
    {
      obj->rate.rxCarrier_ = V34_CARRIER_1829;
      obj->rate.detTab_ = c1829;
    }
  }
  else if (b == 5)
  {
    obj->rate.scaleTab_ = scale3429;
    obj->rate.rxBaud_ = V34_BAUD_3429;
    obj->rate.rxCarrier_ = V34_CARRIER_1959;
    obj->rate.detTab_ = c1959;
  }

  if (1 < dsplibs_debug_level)
    dsplibs_debug_printf("V34PROBESELECT, setfinalrate, txbaudrate = %d,rxbaudrate = %d,txpreemp= %d\n", (int)(int16_t)obj->rate.baud_, (int)obj->rate.rxBaud_,
                         (int)(int16_t)obj->rate.preemp_);
}

/* Entry i of a 28-short scale table; the neighbouring indices -1 and 28 and anything else outside the table read as 0. */
static int scale_at(const void* p, int i)
{
  if (i < 0 || i > 27)
    return 0;
  return ((const s16_t*)p)[i];
}

/**
 * @brief Fix the final data rates of both directions and initialise the transmit and receive bit-allocation blocks.
 * @param obj  the V.34 module object, after the INFO exchange (setfinalrate() has filled the rate block)
 * @note  Rate block R = `obj->rate`: +0x00 transmit baud, +0x04 Tn (transmit rate index), +0x08 (2-bit field) and +0x0a (flag) taken from the word at
 *        +0xaa0c, +0x0c transmit scale pointer, +0x12 receive baud, +0x14 Rn (receive rate index), +0x22 receive flag (hi), +0x28 receive scale pointer.
 *        The rate bitmap is the word at +0xaa0e; the INFO fields come from the words at +0xaa0c and +0xaa3c, laid out differently for the calling and the
 *        answering role (modeChar_ == V34ROLE_CALL or not).  All counts are 16-bit quantities.  The steps:
 *        - Tn = min(Tn, bit-reversed field of the remote INFO), Rn = min(Rn, field of the local INFO); a "preliminary" bit-rate message is printed;
 *        - unless the bitmap's sign bit and the "asymmetric rates allowed" bit (+0xaa3e bit 0) are both set, Tn = Rn = min(Tn, Rn);
 *        - each index is lowered to the highest rate whose bit is set in the bitmap (bit k - 1 for index k; an out-of-range index counts down through the
 *          16-bit wrap to 0);
 *        - Tn == 0 (no usable transmit rate) with a receive baud other than 2400 is reported as an error and both indices are forced to 2 (4800 bit/s);
 *        - the 2-bit field, the flag and bit 0x4000 of `txFlags_` (from bit 5 of +0xaa0d) are stored; a "finally" message is printed;
 *        - transmit side: the scale entry s1 = scale[Tn + 14 * hi - 1] is read before the Tn == 0 test; the number of transmit bits per interval is
 *          `w10_` = (Tn * PTC) >> 6 + 6 (`w8_` is the PTC value; 0 for Tn == 0); initV34() initialises the transmit block (`obj + 0x25e0`);
 *        - receive side: s2 = scale[Rn + 14 * hiRx - 1] >> 1; a zero s2 prints a division-by-zero warning and becomes 1; initV34() initialises `blkb`;
 *        - when the "rates fixed" byte (`b_ac16_`) is still 0: the transmit data rate Tn * 2400 bit/s is stored in `dgC04_` and, outside V.PCM / K56Flex
 *          mode, the byte is set and the receive rate Rn * 2400 is stored in `dgC08_`.
 *        Data rates are index * 2400 bit/s (0x960).  The debug messages print only above debug level 1; the level read after the "preliminary" message
 *        governs the next two messages, the others read it afresh.
 */
void initdigital(V34Obj* obj)
{
  V34Rate* rateobj = &obj->rate;
  unsigned int x = obj->q2.waa0c_;
  unsigned int y = obj->mp.waa3c_;
  unsigned int b, c, tn0;
  uint16_t Tn, Rn;
  int a;
  unsigned int L;
  uint32_t mask;
  int hi, hiRx, s1, s2;

  /* extract the rate fields of the two INFO words; the role decides which half of each word is local and which remote */
  if ((int16_t)obj->modeChar_ == V34ROLE_CALL)
  {
    tn0 = (x >> 2) & 0xf;
    b = (x >> 6) & 0xf;
    c = (y >> 10) & 0xf;
  }
  else
  {
    tn0 = (x >> 6) & 0xf;
    b = (x >> 2) & 0xf;
    c = (y >> 6) & 0xf;
  }

  /* limit both rate indices by the fields received */
  AS_U16(rateobj->txRateIdx_) = (uint16_t)tn0;
  a = (int16_t)bitreverse((unsigned short)c, 4);
  Tn = AS_U16(rateobj->txRateIdx_);
  if ((int16_t)Tn > (int16_t)a)
  {
    AS_U16(rateobj->txRateIdx_) = (uint16_t)a;
    Tn = (uint16_t)a;
  }
  Rn = AS_U16(rateobj->rxRateIdx_);
  if ((int16_t)Rn > (int16_t)b)
  {
    AS_U16(rateobj->rxRateIdx_) = (uint16_t)b;
    Rn = (uint16_t)b;
  }

  L = dsplibs_debug_level;
  if (1 < L)
  {
    dsplibs_debug_printf("V34DATARATE, preliminary txbitrate %d,rxbitrate %d\n", (int16_t)Tn * 0x960, (int16_t)Rn * 0x960);
    Tn = AS_U16(rateobj->txRateIdx_);
    Rn = AS_U16(rateobj->rxRateIdx_);
    L = dsplibs_debug_level;
  }

  /* symmetric rates unless both ends allow asymmetric ones */
  if ((int16_t)obj->q2.waa0e_ >= 0 || (obj->mp.waa3e_ & 1) == 0)
  {
    uint16_t m = ((int16_t)Tn <= (int16_t)Rn) ? Tn : Rn;
    AS_U16(rateobj->rxRateIdx_) = m;
    AS_U16(rateobj->txRateIdx_) = m;
    Tn = Rn = m;
  }

  /* trim each index down to a rate that the bitmap allows */
  mask = (1u << (((unsigned int)(int)(int16_t)Tn - 1u) & 31u)) & 0xffffu;
  if (Tn != 0 && (obj->q2.waa0e_ & mask) == 0)
  {
    do
    {
      Tn = (uint16_t)(Tn - 1);
      mask >>= 1;
      AS_U16(rateobj->txRateIdx_) = Tn;
      if (Tn == 0)
        break;
    } while ((obj->q2.waa0e_ & mask) == 0);
  }
  mask = (1u << (((unsigned int)(int)(int16_t)Rn - 1u) & 31u)) & 0xffffu;
  if (Rn != 0 && (obj->q2.waa0e_ & mask) == 0)
  {
    do
    {
      Rn = (uint16_t)(Rn - 1);
      mask >>= 1;
      AS_U16(rateobj->rxRateIdx_) = Rn;
      if (Rn == 0)
        break;
    } while ((obj->q2.waa0e_ & mask) == 0);
  }

  /* no transmit rate at a receive baud other than 2400: fall back to index 2 in both directions */
  if (Tn == 0 && rateobj->rxBaud_ != V34_BAUD_2400)
  {
    if (1 < L)
    {
      dsplibs_debug_printf("--ERROR---, 2400bps is not possible at %d baud rate\n", (int)(int16_t)rateobj->rxBaud_);
      L = dsplibs_debug_level;
    }
    AS_U16(rateobj->txRateIdx_) = 2;
    AS_U16(rateobj->rxRateIdx_) = 2;
  }

  rateobj->waa8c_ = (uint16_t)((obj->q2.waa0c_ >> 0xb) & 3);
  rateobj->waa8e_ = (uint16_t)((obj->q2.waa0c_ >> 0xe) & 1);
  if (obj->q2.waa0c_ & 0x2000)
    obj->tx.txFlags_ = (uint16_t)(obj->tx.txFlags_ | 0x4000);
  else
    obj->tx.txFlags_ = (uint16_t)(obj->tx.txFlags_ & 0xbfff);

  if (1 < L)
    dsplibs_debug_printf("V34DATARATE, finally txbitrate %d,rxbitrate %d\n", (int)rateobj->txRateIdx_ * 0x960, (int)rateobj->rxRateIdx_ * 0x960);

  /* transmit side */
  Tn = AS_U16(rateobj->txRateIdx_);
  hi = (int16_t)rateobj->waa8e_;
  s1 = (int16_t)scale_at(obj->rate.scalePtr_, (int)(int16_t)Tn + hi * 14 - 1); /* read before the Tn == 0 test */
  if (Tn == 0)
  {
    obj->w10_ = 0;
  }
  else
  {
    /* 32-bit wrapping product, arithmetic shift */
    obj->w10_ = ((int32_t)((uint32_t)(int32_t)(int16_t)Tn * (uint32_t)obj->w8_) >> 6) + 6;
  }
  if (1 < dsplibs_debug_level)
    dsplibs_debug_printf("V34DATARATE, for tx data rate - %d, PTC - %d, setting nofTxBits to %d\r\n", (int)(int16_t)Tn, (int)obj->w8_, (int)(int32_t)obj->w10_);

  Tn = AS_U16(rateobj->txRateIdx_);
  hi = (int16_t)rateobj->waa8e_;
  initV34((V34BlkB*)&obj->tx.w25e0_, rateobj->baud_, (uint16_t)((int16_t)Tn * 0x960), (int16_t)hi, (int16_t)rateobj->waa8c_, obj->tx.precCoef_,
          (unsigned short)s1);

  /* receive side */
  hiRx = rateobj->waaa6_;
  Rn = AS_U16(rateobj->rxRateIdx_);
  s2 = scale_at(obj->rate.scaleTab_, (int)(int16_t)Rn + 14 * hiRx - 1) >> 1;
  if (s2 == 0)
  {
    if (1 < dsplibs_debug_level)
      dsplibs_debug_printf("FATAL ERROR(initdigital) - ZERODIV expected!");
    s2 = 1;
  }
  Rn = AS_U16(rateobj->rxRateIdx_);
  hiRx = rateobj->waaa6_;
  initV34(&obj->rx.blkb, rateobj->rxBaud_, (uint16_t)((int16_t)Rn * 0x960), (short)hiRx, 0, obj->rx.precoef_, (unsigned short)s2);

  /* record the agreed data rates once */
  if (obj->b_ac16_ == 0)
  {
    obj->dgC04_ = (int)rateobj->txRateIdx_ * 0x960;
    if (obj->pcmMode_ == 0 && obj->k56Mode_ == 0)
    {
      obj->b_ac16_ = 1;
      obj->dgC08_ = (int)rateobj->rxRateIdx_ * 0x960;
    }
  }
}
