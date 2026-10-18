/**
 * @file fax_vmi.c
 * @brief FAXVMI: the layer between the T.30 byte / frame buffers and the bit buffers of the fax modems.
 *
 * Contents: the VMI object functions (FAXVMI_create, _delete, _process, _status, _control, _message); the framing functions that pack bytes or frames
 * into modem bits (faxvmi_simp_pack, faxvmi_asyc_pack, faxvmi_hdlc_frame) and unpack received bits (faxvmi_simp_unpack, faxvmi_asyc_unpack,
 * faxvmi_hdlc_unframe), with the ring writers faxvmi_write_fifo / faxvmi_write_frame, the CRC faxvmi_gen_fcs16 and the bit-reversal helpers; the null
 * modem (null_*, ids 0 .. 4); and the ops tables vxx_* that dispatch to the V.21 / V.27ter / V.29 / V.17 modem wrappers.  Typed objects: fax_vmi.h.
 *
 * The pipeline of FAXVMI_process(o, buf, samples, n, cnt2):
 * -# optional bit reversal of `buf` (cfg.flag);
 * -# `local = pack[mode](o, buf, *n)`: queue the T.30 bytes / frames in the ring, then fill the modem's bit buffer `m->bits` with `s0c` words of `s0e` bits;
 * -# `r = vxx_process[id](m, samples, &local, cnt2)`: the modem consumes (transmit) or produces (receive) the bits;
 * -# `*n = unpack[mode](o, buf, local)`: turn the received bits into bytes / frames;
 * -# optional bit reversal of the result;
 * -# the returned state word is the modem's state with the ring flags added (FaxVmiState).
 *
 * Modes: 0 simple (plain bits, 8 per byte, MSB first), 1 asynchronous (start bit 0, 8 data bits MSB first, stop bit 1: 10 bits per byte), 2 HDLC (flags,
 * zero insertion after five ones, CRC-16: frames are queued as [length][bytes][CRC high][CRC low] with the length counting the data and the two CRC bytes).
 *
 * The transmit and the receive bit state (mask, current item, accumulator, bit count) persist in FaxVmiSt between calls.  A bit is taken as
 * `(cur & mask) != 0` and the mask shifts right.  The mode and the modem id index the ops and framing tables unchecked: modes 0 .. 2 and ids 0 .. 12 only.
 */
#include <limits.h>
#include <string.h>
#include "fax_vmi.h"
#include "fax_t30.h" /* GetT30FrameIDFromBuffer, GetT30FrameNameByID */
#include "fax_v21.h"
#include "fax_v27_tx.h"
#include "fax_v27_rx.h"
#include "fax_v29_tx.h"
#include "fax_v29_rx.h"
#include "fax_v17_tx.h"
#include "fax_v17_rx.h"

#include "fax_debug.h" /* dsplibs_debug_level, DBG, DBG3 */
#include "core/dsp_host.h"

#include "fax_noinl.h" /* NOINL */

/* ===================================================================================================================== CRC / bit reverse */
/**
 * @brief Add one byte (given as a 16-bit word) to the CRC-16 register `bx` in two nibble steps; returns the new register.
 * @note  The register starts at 0xffff and the CRC is the complement of the final value (see faxvmi_gen_fcs16()).
 */
static inline unsigned fcs_step(unsigned bx, unsigned w)
{
  unsigned ecx = w << 8, eax, edx;
  eax = ecx ^ bx;
  ecx <<= 4;
  eax &= 0xf000;
  edx = eax >> 11;
  bx ^= edx;
  bx <<= 4;
  bx ^= eax;
  edx = bx & 0xffff;
  eax >>= 12;
  edx |= eax;
  ecx ^= edx;
  ecx &= 0xf000;
  eax = ecx >> 11;
  edx ^= eax;
  edx <<= 4;
  edx ^= ecx;
  bx = edx & 0xffff;
  ecx >>= 12;
  bx |= ecx;
  return bx;
}

/**
 * @brief CRC-16 (FCS) of `n` bytes held one per 16-bit word.
 * @return the complemented CRC register.  Computed over a frame that ends with its own CRC bytes (high byte first) the result is the constant 0xe2f0,
 *         which faxvmi_hdlc_unframe() uses as the check.
 */
unsigned short faxvmi_gen_fcs16(const unsigned short* p, short n)
{
  unsigned bx = 0xffff;
  short k = (short)(n - 1);
  if ((short)(k + 1) != 0)
  {
    for (;;)
    {
      short kk = k;
      bx = fcs_step(bx, *p++);
      k = (short)(k - 1);
      if (kk == 0)
        break;
    }
  }
  return (unsigned short)~bx;
}

/** @brief The low 8 bits of `w` in reverse bit order. */
static unsigned rev8(unsigned w)
{
  unsigned r = 0, i;
  for (i = 0; i < 8; i++)
  {
    r = ((r + r) & 0xffff) | (w & 1);
    w >>= 1;
  }
  return r;
}

/** @brief Reverse the bit order of each of `n` bytes held one per word (in place). */
void faxvmi_byte_reverse(unsigned short* p, short n)
{
  short k = (short)(n - 1);
  if ((short)(k + 1) == 0)
    return;
  for (;;)
  {
    short kk = k;
    *p = (unsigned short)rev8(*p);
    p++;
    k = (short)(k - 1);
    if (kk == 0)
      break;
  }
}

/** @brief Reverse the bit order of the bytes of `n` consecutive [length][bytes...] frames (in place; the length words stay). */
void faxvmi_frame_reverse(unsigned short* p, short n)
{
  short k = (short)(n - 1);
  if ((short)(k + 1) == 0)
    return;
  for (;;)
  {
    short kk = k;
    unsigned len = *p;
    unsigned short* q = p + 1;
    short j = (short)len;
    if (j != 0)
    {
      short jj = (short)(j - 1);
      for (;;)
      {
        short j0 = jj;
        *q = (unsigned short)rev8(*q);
        q++;
        jj = (short)(jj - 1);
        if (j0 == 0)
          break;
      }
    }
    p = p + 1 + len; /* skip the length word and the data (the length is taken zero extended) */
    k = (short)(k - 1);
    if (kk == 0)
      break;
  }
}

/* ===================================================================================================================== ring writers */
/**
 * @brief Queue up to `n` items from `*pp` in the ring until it is full.
 * @param o   VMI object
 * @param pp  in: source; out: advanced past the consumed items
 * @param n   number of items offered
 * @return the number of items queued; clears txEmpty when at least one was queued
 */
int faxvmi_write_fifo(FaxVmi* o, unsigned short** pp, short n)
{
  int cnt = 0;
  int txEmpty = o->txEmpty;
  unsigned short* src = *pp;
  short k = (short)(n - 1);
  FaxVmiSt* st;
  if ((short)(k + 1) != 0)
  {
    st = o->st;
    for (;;)
    {
      short kk;
      unsigned short w;
      if (st->cnt >= st->size)
      {
        *pp = src;
        return cnt;
      }
      w = st->wr;
      st->wr = (unsigned short)(w + 1);
      st->ring[w] = *src++;
      if (!(st->wr < st->size))
        st->wr = 0;
      cnt = (short)(cnt + 1);
      txEmpty = 0;
      st->cnt = (unsigned short)(st->cnt + 1);
      kk = k;
      k = (short)(k - 1);
      if (kk == 0)
        break;
    }
  }
  *pp = src;
  o->txEmpty = txEmpty;
  return cnt;
}

/**
 * @brief Queue up to `n` frames from `*pp` in the ring, each as [length + 2][data...][CRC high][CRC low].
 * @param o   VMI object
 * @param pp  in: frames as [length][data...]; out: advanced past the frames consumed
 * @param n   number of frames offered
 * @return the number of frames queued; stops before a frame that would not fit (length + 3 more items than the ring has room for)
 *
 * The CRC is computed with faxvmi_gen_fcs16()'s register (init 0xffff, complemented) over the data.
 * @note  The data-copy loop has an overflow exit, but the room check before each frame makes it unreachable.
 */
int faxvmi_write_frame(FaxVmi* o, unsigned short** pp, short n)
{
  int cnt = 0;
  unsigned short* p = *pp;
  short k = (short)(n - 1);
  FaxVmiSt* st;
  unsigned short* ring;
  unsigned short size, wr, qcnt;
  int txEmpty;
  int len;
  if ((short)(k + 1) == 0)
  {
    *pp = p;
    return 0;
  }
  /* the ring state lives in locals (written back once): no load of a state member follows a store of another type, so the code does not depend on
   * the aliasing assumptions */
  st = o->st;
  ring = st->ring;
  size = st->size;
  wr = st->wr;
  qcnt = st->cnt;
  txEmpty = o->txEmpty;
  len = (short)p[0];
  if ((int)qcnt + len + 3 >= (int)size)
  {
    *pp = p;
    return 0;
  }
  for (;;)
  {
    short lm1;
    unsigned crc;
    unsigned short* q;
    unsigned crcf;
    int overflow = 0;
    short kk;
    unsigned short w;
    w = wr;
    wr = (unsigned short)(w + 1);
    ring[w] = (unsigned short)(len + 2);
    if (!(wr < size))
      wr = 0;
    lm1 = (short)(len - 1);
    p++; /* the data */
    cnt = (short)(cnt + 1);
    qcnt = (unsigned short)(qcnt + 3);
    crc = USHRT_MAX;
    q = p;
    if (lm1 != -1)
    {
      short c = lm1;
      for (;;)
      {
        short cc = c;
        crc = fcs_step(crc, *q++);
        c = (short)(c - 1);
        if (cc == 0)
          break;
      }
    }
    crcf = (unsigned short)~crc;
    if (lm1 != -1)
    {
      short c = lm1;
      for (;;)
      {
        short cc;
        if (!(qcnt < size))
        {
          overflow = 1;
          break;
        }
        w = wr;
        wr = (unsigned short)(w + 1);
        ring[w] = *p++;
        if (!(wr < size))
          wr = 0;
        qcnt = (unsigned short)(qcnt + 1);
        txEmpty = 0;
        cc = c;
        c = (short)(c - 1);
        if (cc == 0)
          break;
      }
    }
    (void)overflow;
    w = wr;
    wr = (unsigned short)(w + 1);
    ring[w] = (unsigned short)(crcf >> 8);
    if (!(wr < size))
      wr = 0;
    w = wr;
    wr = (unsigned short)(w + 1);
    ring[w] = (unsigned short)(crcf & 0xff);
    if (!(wr < size))
      wr = 0;
    if (!overflow)
      o->txEmpty = txEmpty; /* stored once the data is copied, before the CRC bytes are written */
    kk = k;
    k = (short)(k - 1);
    if (kk == 0)
      break;
    len = (short)p[0];
    if (!((int)qcnt + len + 3 < (int)size))
      break;
  }
  st->wr = wr;
  st->cnt = qcnt;
  *pp = p;
  return cnt;
}

/* ===================================================================================================================== simple / asynchronous packing */
/**
 * @brief Asynchronous framing, transmit side: queue `n` bytes, then fill the modem bit buffer with `s0c` words of `s0e` bits.
 * @param o    VMI object
 * @param buf  `n` bytes, one per word
 * @param n    number of bytes
 * @return `s0c`, the number of words written to `m->bits`
 *
 * Each byte is sent as a 10-bit character (start bit 0, 8 data bits MSB first, stop bit 1).  With the ring empty the line idles at 1 (and txEmpty is
 * set); a pending break request first sends `breakLeft` zero bits and then one mark bit.  Bytes that did not fit before are queued after the bits were
 * generated (the ring was drained meanwhile); the number still not queued is stored in `st->ovf`.
 */
int faxvmi_asyc_pack(FaxVmi* o, unsigned short* buf, short n)
{
  unsigned short* p = buf;
  int r0 = faxvmi_write_fifo(o, &p, n);
  int left = (short)(n - r0);
  FaxVmiM* m = o->m;
  FaxVmiSt* st = o->st;
  unsigned short* out = (unsigned short*)m->u4.bits;
  unsigned short* ring = st->ring;
  unsigned short size = st->size, rd = st->rd, qcnt = st->cnt, breakLeft = st->breakLeft;
  int breakPend = st->breakReq, txEmpty = o->txEmpty;
  unsigned cur = (unsigned)st->tx_cur;
  unsigned mask = (unsigned short)st->tx_mask;
  unsigned acc = (unsigned)st->tx_acc;
  unsigned nb = st->tx_nbits;
  int cnt = m->s0c;
  unsigned width = (unsigned short)m->s0e;
  unsigned maskw = (1u << ((unsigned char)width & 31)) - 1u;
  int r1;

  /* generate `s0c` output words, a bit at a time */
  while (cnt != 0)
  {
    if (mask == 0)
    {
      if (breakPend != 0)
      {
        unsigned short ax = breakLeft;
        if (ax == 0)
        {
          breakPend = 0;
          cur = 1;
        }
        else
        {
          cur = 0;
          mask = 1;
          breakLeft = (unsigned short)(ax - 1);
        }
      }
      else if (qcnt == 0)
      {
        cur = 1;
        mask = 1;
        txEmpty = 1;
      }
      else
      {
        unsigned short rd0 = rd;
        unsigned c1 = (unsigned short)(rd0 + 1u);
        unsigned w = ring[rd0];
        rd = (unsigned short)c1;
        qcnt = (unsigned short)(qcnt - 1);
        if (!(size > c1))
          rd = 0;
        cur = ((w + w) | 1u) & 0xfffffdffu;
        mask = 0x200;
      }
    }
    acc = acc + acc;
    {
      unsigned bit = (cur & mask) != 0;
      unsigned nb1 = (unsigned short)(nb + 1u);
      acc |= bit;
      mask >>= 1;
      nb = nb1;
      if (nb1 == width)
      {
        *out++ = (unsigned short)(maskw & acc);
        cnt = (short)(cnt - 1);
        nb = 0;
      }
    }
  }
  /* write the state back, then queue what did not fit before */
  st->rd = rd;
  st->cnt = qcnt;
  st->breakLeft = breakLeft;
  st->breakReq = breakPend;
  o->txEmpty = txEmpty;
  st->tx_nbits = (unsigned short)nb;
  st->tx_mask = (int)mask;
  st->tx_acc = (int)acc;
  st->tx_cur = (int)cur;
  r1 = faxvmi_write_fifo(o, &p, (short)left);
  left -= r1;
  st->ovf = (short)left; /* o->st / o->m are not re-read behind the call (they never change): the code must not depend on the aliasing assumptions */
  return m->s0c;
}

/**
 * @brief Simple framing, transmit side: queue `n` bytes, then fill the modem bit buffer with `s0c` words of `s0e` bits, 8 bits per byte MSB first.
 * @param o    VMI object
 * @param buf  `n` bytes, one per word
 * @param n    number of bytes
 * @return the number of output words produced while queued data was still available (zero bits are sent once the ring is empty, and txEmpty is set)
 *
 * Bytes that did not fit before are queued after the bits were generated; the number still not queued is stored in `st->ovf`.
 */
int faxvmi_simp_pack(FaxVmi* o, unsigned short* buf, short n)
{
  unsigned short* p = buf;
  int one = 1, ret = 0;
  int r0 = faxvmi_write_fifo(o, &p, n);
  int left = (short)(n - r0);
  FaxVmiM* m = o->m;
  FaxVmiSt* st = o->st;
  unsigned short* ring = st->ring;
  unsigned short size = st->size, rd = st->rd, qcnt = st->cnt;
  int txEmpty = o->txEmpty;
  unsigned mask = (unsigned short)st->tx_mask;
  unsigned short* out = (unsigned short*)m->u4.bits;
  int cnt = m->s0c;
  unsigned width = (unsigned short)m->s0e;
  unsigned cur = (unsigned)st->tx_cur;
  unsigned acc = (unsigned)st->tx_acc;
  unsigned maskw = ((unsigned)1 << ((unsigned char)width & 31)) - 1u;
  unsigned nb = st->tx_nbits;
  int r1;

  /* generate `s0c` output words, a bit at a time */
  while (cnt != 0)
  {
    if (mask == 0)
    {
      if (qcnt != 0)
      {
        unsigned short rd0 = rd;
        unsigned c1 = (unsigned short)(rd0 + 1u);
        unsigned short c = qcnt;
        rd = (unsigned short)c1;
        cur = ring[rd0];
        qcnt = (unsigned short)(c - 1);
        if (!(size > c1))
          rd = 0;
      }
      else
      {
        cur = 0;
        one = 0;
        txEmpty = 1;
      }
      mask = 0x80;
    }
    acc = acc + acc;
    {
      unsigned bit = (cur & mask) != 0;
      unsigned nb1 = (unsigned short)(nb + 1u);
      acc |= bit;
      mask >>= 1;
      nb = nb1;
      if (nb1 != width)
        continue;
    }
    ret = (short)(one + ret);
    *out++ = (unsigned short)(maskw & acc);
    nb = 0;
    cnt = (short)(cnt - 1);
  }
  st->rd = rd;
  st->cnt = qcnt;
  o->txEmpty = txEmpty;
  st->tx_mask = (int)mask;
  st->tx_acc = (int)acc;
  st->tx_nbits = (unsigned short)nb;
  st->tx_cur = (int)cur;
  r1 = faxvmi_write_fifo(o, &p, (short)left);
  left -= r1;
  st->ovf = (short)left;
  return ret;
}

/* ===================================================================================================================== HDLC transmit */
/**
 * @brief HDLC framing, transmit side: queue `n` frames, then fill the modem bit buffer with `s0c` words of `s0e` bits.
 * @param o    VMI object
 * @param buf  `n` frames as [length][bytes...]
 * @param n    number of frames
 * @return `s0c`, the number of words written to `m->bits`
 *
 * With nothing queued the line sends 0x7e flags (and txEmpty is set).  A queued frame is preceded by three flag bytes; its bytes (data and CRC) go out
 * MSB first with a zero inserted after every five ones.  Frames that did not fit before are queued afterwards; the number still not queued is stored in
 * `st->ovf`.  At debug level above 1 the frame is dumped and named (T.30 frame identification).
 * @note  The loop that walks the ring for the debug dump runs whatever the debug level (it prints only at level above 1).
 */
int faxvmi_hdlc_frame(FaxVmi* o, unsigned short* buf, short n)
{
  unsigned short* p = buf;
  int r0 = faxvmi_write_frame(o, &p, n);
  int left = (short)(n - r0);
  FaxVmiSt* st = o->st;
  FaxVmiM* m = o->m;
  unsigned mask = (unsigned)st->tx_mask;
  unsigned cur = (unsigned)st->tx_cur;
  unsigned acc = (unsigned)st->tx_acc;
  unsigned nb = st->tx_nbits;
  int cnt = m->s0c;
  unsigned short* out = (unsigned short*)m->u4.bits;
  unsigned width = (unsigned short)m->s0e;
  unsigned maskw = (1u << ((unsigned char)width & 31)) - 1u;
  int remain = st->tx_left;
  int flagh = st->tx_flag;
  int r1;

  /* debug dump of the frames just queued */
  if (n != 0)
  {
    unsigned len = st->ring[st->rd];
    unsigned idx;
    if (DBG)
      dsplibs_debug_printf("HDLC Transmitted Frame (Length = %d, iR = %d, iW = %d): \n", len, (unsigned)st->rd, (unsigned)st->wr);
    idx = (unsigned)st->rd + 1u;
    while ((unsigned)st->wr != idx)
    {
      if (DBG)
      {
        dsplibs_debug_printf("%02X,", (unsigned)st->ring[idx]);
      }
      {
        unsigned nx = idx + 1u;
        idx = ((int)st->size > (int)nx) ? nx : 0;
      }
    }
    if (DBG)
      dsplibs_debug_printf("\n");
    if (len > 2)
    {
      if (DBG)
      {
        unsigned char b[3];
        unsigned i2 = (unsigned)st->rd + 1u, j;
        unsigned id;
        for (j = 0; j < 3; j++)
        {
          unsigned nx;
          b[j] = (unsigned char)st->ring[i2];
          nx = i2 + 1u;
          i2 = ((int)st->size > (int)nx) ? nx : 0;
        }
        id = GetT30FrameIDFromBuffer(b[0], b[1], b[2]);
        dsplibs_debug_printf("FCL1: FRAME TRANSMITTED (%s)\n", GetT30FrameNameByID(id & 0xffff7fffu));
      }
    }
  }
  /* generate `s0c` output words, a bit at a time */
  while (cnt != 0)
  {
    if (mask == 0)
    {
      unsigned short c = st->cnt;
      if (c == 0)
      {
        flagh = 1;
        mask = 0x80;
        cur = 0x7e;
        o->txEmpty = 1;
      }
      else if (remain == 0)
      {
        unsigned short rd = st->rd;
        unsigned c1 = (unsigned short)(rd + 1u);
        st->rd = (unsigned short)c1;
        remain = (short)st->ring[rd];
        st->cnt = (unsigned short)(c - 1);
        if (!(st->size > c1))
          st->rd = 0;
        cur = 0x7e7e7e;
        flagh = 1;
        mask = 0x800000;
      }
      else
      {
        unsigned short rd = st->rd;
        unsigned c1 = (unsigned short)(rd + 1u);
        st->rd = (unsigned short)c1;
        cur = st->ring[rd];
        st->cnt = (unsigned short)(c - 1);
        if (!(st->size > c1))
          st->rd = 0;
        mask = 0x80;
        flagh = 0;
        remain = (short)(remain - 1);
      }
    }
    acc = acc + acc;
    {
      unsigned bit = (cur & mask) != 0;
      acc |= bit;
      if ((acc & 0x1f) == 0x1f && flagh == 0)
        cur &= ~mask;
      else
        mask >>= 1;
    }
    nb = (unsigned short)(nb + 1u);
    if (nb == width)
    {
      nb = 0;
      *out++ = (unsigned short)(maskw & acc);
      cnt = (short)(cnt - 1);
    }
  }
  /* write the state back, then queue what did not fit before */
  st->tx_mask = (int)mask;
  st->tx_acc = (int)acc;
  st->tx_cur = (int)cur;
  st->tx_left = (short)remain;
  st->tx_flag = flagh;
  st->tx_nbits = (unsigned short)nb;
  r1 = faxvmi_write_frame(o, &p, (short)left);
  left -= r1;
  st->ovf = (short)left;
  return m->s0c;
}

/* ===================================================================================================================== unpacking */
/**
 * @brief Asynchronous framing, receive side: turn `n` words of received bits into bytes.
 * @param o    VMI object
 * @param out  receives the bytes, one per word
 * @param n    number of input words in `m->bits` (each holds `m->u10.s10` bits, MSB first)
 * @return the number of bytes produced; at most cfg.maxItems per call, further bytes set rxOverflow
 *
 * State 0 collects 8 data bits after a start bit; after the 8th the byte is stored and the machine waits for the next start bit.  23 zero bits in a row
 * restart the search, and a single one after 22 zeros sets st->rxBreak.
 */
int faxvmi_asyc_unpack(FaxVmi* o, unsigned short* out, short n)
{
  FaxVmiSt* st = o->st;
  FaxVmiM* m = o->m;
  int ovfFlag = 0, brkFlag = 0, count = 0;
  unsigned cur = (unsigned)st->rx_cur;
  unsigned nb = st->rx_nbits;
  unsigned mask = (unsigned short)st->rx_mask;
  unsigned acc = (unsigned)st->rx_acc;
  const unsigned short* src = (const unsigned short*)m->u4.bits;
  unsigned topmask = 1u << ((unsigned char)(m->u10.s10 - 1) & 31);
  unsigned state = (unsigned)st->rx_state;
  int left = n;
  while (left != 0)
  {
    unsigned bit;
    if (mask == 0)
    {
      mask = topmask & 0xffff;
      cur = *src++;
      left = (short)(left - 1);
    }
    bit = (cur & mask) != 0;
    acc = acc + acc;
    acc |= bit;
    mask >>= 1;
    if (state == 0)
    {
      nb = (unsigned short)(nb + 1u);
      state = 0;
      if (nb == 8)
      {
        nb = 0;
        state = 1;
        if (count < (int)o->cfg.maxItems)
        {
          *out++ = (unsigned short)(acc & 0xff);
          count = (short)(count + 1);
        }
        else
        {
          ovfFlag = 1;
        }
      }
      if ((acc & 0x7fffff) == 0)
      {
        nb = 0;
        state = 1;
      }
    }
    else
    {
      state = bit;
      if ((acc & 0x7fffff) == 0)
      {
        nb = 0;
        state = 1;
      }
    }
    if ((acc & 0x7fffff) == 1)
      brkFlag = 1;
  }
  st->rx_state = (int)state;
  st->rx_mask = (int)mask;
  st->rx_acc = (int)acc;
  st->rxBreak = brkFlag;
  st->rx_cur = (int)cur;
  st->rx_nbits = (unsigned short)nb;
  o->rxOverflow = ovfFlag;
  return count;
}

/**
 * @brief Simple framing, receive side: turn `n` words of received bits into bytes, 8 bits per byte MSB first.
 * @param o    VMI object
 * @param out  receives the bytes, one per word
 * @param n    number of input words in `m->bits` (each holds `m->u10.s10` bits)
 * @return the number of bytes produced; at most cfg.maxItems per call, further bytes set rxOverflow
 */
int faxvmi_simp_unpack(FaxVmi* o, unsigned short* out, short n)
{
  FaxVmiSt* st = o->st;
  FaxVmiM* m = o->m;
  int ovfFlag = 0, count = 0;
  unsigned mask = (unsigned short)st->rx_mask;
  const unsigned short* src = (const unsigned short*)m->u4.bits;
  unsigned cur = (unsigned)st->rx_cur;
  unsigned nb = st->rx_nbits;
  unsigned acc = (unsigned)st->rx_acc;
  unsigned topmask = 1u << ((unsigned char)(m->u10.s10 - 1) & 31);
  int left = n;
  while (left != 0)
  {
    unsigned bit;
    if (mask == 0)
    {
      mask = topmask & 0xffff;
      cur = *src++;
      left = (short)(left - 1);
    }
    bit = (cur & mask) != 0;
    acc = acc + acc;
    acc |= bit;
    mask >>= 1;
    nb = (unsigned short)(nb + 1u);
    if (nb != 8)
      continue;
    nb = 0;
    if (count < (int)o->cfg.maxItems)
    {
      *out++ = (unsigned short)(acc & 0xff);
      count = (short)(count + 1);
    }
    else
    {
      ovfFlag = 1;
    }
  }
  st->rx_mask = (int)mask;
  st->rx_nbits = (unsigned short)nb;
  st->rx_cur = (int)cur;
  st->rx_acc = (int)acc;
  o->rxOverflow = ovfFlag;
  return count;
}

/**
 * @brief HDLC framing, receive side: turn `n` words of received bits into frames.
 * @param o    VMI object
 * @param out  receives the frames as [length][bytes...] (the length includes the two CRC bytes), one item per word
 * @param n    number of input words in `m->bits` (each holds `m->u10.s10` bits, MSB first)
 * @return the number of frames produced
 *
 * A run of six ones followed by a zero is a flag: after the first `hskip` (initially 2) flags it starts collecting, inside a frame it ends the frame.
 * A zero after five ones is a stuffed bit and is dropped.  A one after six ones abandons the frame in progress (the partial byte is dropped) and the
 * following bits are collected again as data.  A finished frame is checked with the CRC (residue 0xe2f0).  On a bad CRC the code tries to repair
 * the frame: the first byte replaced by 0xff, then the second by 0xc8, then for 1 .. 7 bits it shifts the whole frame left (one byte shorter) and
 * retries both replacements; if nothing matches the frame is delivered as an empty record (length 0).  Debug messages describe the repair at level
 * above 1.
 * @bug  An abandoned frame (a one after six ones) does not reset the collected byte count `flen`, so its bytes stay in the frame buffer and the next
 *       frame is appended to them; that frame then fails the CRC and is delivered as an empty record.
 * @warning The overflow test compares only the frame length with cfg.maxItems (the running total of this call is always 0), so the sum of several
 *          frames of one call can exceed the caller's output buffer.  The bit-shift repair reads `frm[flen + 1]`, one item past the data (past the buffer
 *          when the frame filled it).
 */
int faxvmi_hdlc_unframe(FaxVmi* o, unsigned short* out, short n)
{
  FaxVmiSt* st = o->st;
  FaxVmiM* m = o->m;
  int ovfFlag = 0, count = 0;
  const int total = 0; /* running output total of this call; never updated (see the @warning above) */
  unsigned cur = (unsigned)st->rx_cur;
  unsigned acc = (unsigned)st->rx_acc;
  int flen = st->flen;
  const unsigned short* src = (const unsigned short*)m->u4.bits;
  unsigned topmask = 1u << ((unsigned char)(m->u10.s10 - 1) & 31);
  int ones = st->ones;
  unsigned short* frm =
      st->frm; /* the frame buffer pointer and the flags the loop tests are kept in locals (written back at the end): no aliasing dependence */
  unsigned fsz = st->frm_size;
  unsigned short hskip = st->hskip;
  int in_frame = st->in_frame;
  unsigned s0a = o->cfg.maxItems;
  unsigned mask = (unsigned short)st->rx_mask;
  unsigned nb = st->rx_nbits;
  int left = n;
  unsigned dbgl = dsplibs_debug_level; /* read once: the debug level does not change during a call (the loads must not be re-ordered against the u16 stores) */
  while (left != 0)
  {
    unsigned bit;
    if (mask == 0)
    {
      mask = topmask & 0xffff;
      cur = *src++;
      left = (short)(left - 1);
    }
    bit = (cur & mask) != 0;
    mask >>= 1;
    if (ones == 5 && bit == 0)
    {
      /* a stuffed zero */
    }
    else if (ones == 6)
    {
      if (hskip != 0 && bit == 0)
      {
        hskip = (unsigned short)(hskip - 1);
        in_frame = 0;
        nb = 0;
      }
      else
      {
        if (in_frame != 0 && bit == 0)
        {
          unsigned short fcs = faxvmi_gen_fcs16(frm, (short)flen);
          if (fcs != 0xe2f0)
          {
            if ((1 < dbgl))
              dsplibs_debug_printf("Frame with bad CRC\n");
            {
              short i;
              for (i = 0; i < (short)flen; i = (short)(i + 1))
                if ((1 < dbgl))
                  dsplibs_debug_printf("%2x ", (unsigned)frm[i]);
            }
            if ((1 < dbgl))
            {
              dsplibs_debug_printf("\n");
              if ((1 < dbgl))
                dsplibs_debug_printf("Replacing the first byte...\n");
            }
            {
              unsigned short orig0 = frm[0];
              frm[0] = 0xff;
              fcs = faxvmi_gen_fcs16(frm, (short)flen);
              if (fcs != 0xe2f0)
              {
                unsigned short orig1 = frm[1];
                if ((1 < dbgl))
                  dsplibs_debug_printf("Replacing the second byte...\n");
                frm[1] = 0xc8;
                fcs = faxvmi_gen_fcs16(frm, (short)flen);
                if (fcs != 0xe2f0)
                {
                  int limit;
                  int j;
                  flen = (short)(flen - 1);
                  limit = flen + 1;
                  frm[0] = orig0;
                  frm[1] = orig1;
                  j = 1;
                  for (;;)
                  {
                    if ((1 < dbgl))
                      dsplibs_debug_printf("Shifting data %d bits\n", j);
                    {
                      int kk;
                      unsigned short* q = frm;
                      for (kk = 0; kk < limit; kk++)
                      {
                        unsigned a = q[kk];
                        int b = (short)q[kk + 1];
                        q[kk] = (unsigned short)(((a + a) & 0xfe) | ((b >> 7) & 1));
                      }
                    }
                    {
                      short i;
                      for (i = 0; i < (short)flen; i = (short)(i + 1))
                        if ((1 < dbgl))
                          dsplibs_debug_printf("%2x ", (unsigned)frm[i]);
                    }
                    if ((1 < dbgl))
                      dsplibs_debug_printf("\n");
                    fcs = faxvmi_gen_fcs16(frm, (short)flen);
                    if (fcs == 0xe2f0)
                    {
                      if ((1 < dbgl))
                        dsplibs_debug_printf("CRC is now OK!\n");
                      break;
                    }
                    if ((1 < dbgl))
                    {
                      dsplibs_debug_printf("Bad CRC!\n");
                      if ((1 < dbgl))
                        dsplibs_debug_printf("Replacing first byte ...\n");
                    }
                    {
                      unsigned short o0 = frm[0];
                      unsigned short o1;
                      frm[0] = 0xff;
                      fcs = faxvmi_gen_fcs16(frm, (short)flen);
                      if (fcs == 0xe2f0)
                      {
                        if ((1 < dbgl))
                          dsplibs_debug_printf("CRC is now OK!\n");
                        break;
                      }
                      o1 = frm[1];
                      if ((1 < dbgl))
                        dsplibs_debug_printf("Replacing the second byte ...\n");
                      frm[1] = 0xc8;
                      fcs = faxvmi_gen_fcs16(frm, (short)flen);
                      if (fcs == 0xe2f0)
                      {
                        if ((1 < dbgl))
                          dsplibs_debug_printf("CRC is now OK!\n");
                        break;
                      }
                      frm[0] = o0;
                      frm[1] = o1;
                    }
                    j = (short)(j + 1);
                    if (!(j <= 7))
                      break;
                  }
                  if (fcs != 0xe2f0)
                    flen = 0;
                }
              }
            }
          }
          if (total + flen >= (int)s0a)
          {
            ovfFlag = 1;
          }
          else
          {
            short i;
            *out++ = (unsigned short)flen;
            for (i = 0; i < (short)flen; i = (short)(i + 1))
              *out++ = frm[i];
            flen = 0;
            count = (short)(count + 1);
          }
        }
        in_frame = 0;
        nb = 0;
      }
    }
    else
    {
      acc = acc + acc;
      acc |= bit;
      nb = (unsigned short)(nb + 1u);
      if (nb == 8)
      {
        if (hskip == 0 && flen < (int)fsz)
        {
          in_frame = 1;
          frm[flen] = (unsigned char)acc;
          flen = (short)(flen + 1);
        }
        nb = 0;
      }
    }
    ones = bit ? (short)(ones + 1) : 0;
  }
  st->hskip = hskip;
  st->in_frame = in_frame;
  st->rx_mask = (int)mask;
  st->rx_nbits = (unsigned short)nb;
  st->rx_cur = (int)cur;
  st->flen = (short)flen;
  st->rx_acc = (int)acc;
  st->ones = (short)ones;
  o->rxOverflow = ovfFlag;
  return count;
}

/* ===================================================================================================================== the null modem (ids 0 .. 4) */
/** @brief Null modem create: no engine, 50 words of 8 bits (`s0c` / `s0e` / `u10.s10`). */
void null_create(FaxVmiM* m, const void* cfg)
{
  (void)cfg;
  m->eng = NULL;
  m->s0c = 0x32;
  m->s0e = 8;
  m->u10.s10 = 8;
}
/** @brief Null modem delete: nothing to release. */
void null_delete(FaxVmiM* m) { (void)m; }

/**
 * @brief Null modem process: copy `*cnt` samples from `a` to the scratch buffer `m->buf0`.
 * @return -1
 * @warning `*cnt` is not checked against the 200-short size of `buf0`.
 */
int null_process(FaxVmiM* m, short* a, unsigned short* b, unsigned short* cnt)
{
  short i = 0;
  (void)b;
  if ((short)*cnt > 0)
  {
    short* d = m->buf0;
    do
    {
      d[i] = a[i];
      i = (short)(i + 1);
    } while ((short)*cnt > i);
  }
  return -1;
}
/** @brief Null modem status: nothing to report; returns -1. */
int null_status(FaxVmiM* m, void* st)
{
  (void)m;
  (void)st;
  return -1;
}
/** @brief Null modem control: nothing to do; returns -1. */
int null_control(FaxVmiM* m, const void* ctl)
{
  (void)m;
  (void)ctl;
  return -1;
}
/** @brief Null modem message: no text (`*out` = NULL). */
void null_message(FaxVmiM* m, int idx, const char** out)
{
  (void)m;
  (void)idx;
  *out = NULL;
}

/* ===================================================================================================================== the ops tables */
/* adapters: the modem create functions return different things (the result is unused by FAXVMI_create) and take the config as int / typed pointers;
 * the status / control functions take typed records */
#define ADAPT_CREATE(n, cfgt)                                                                                                                                  \
  static void vc_##n(FaxVmiM* m, const void* cfg) { (void)n##_create(m, (cfgt)cfg); }
#define ADAPT_STATUS(n, stt)                                                                                                                                   \
  static int vs_##n(FaxVmiM* m, void* st) { return n##_status(m, (stt)st); }
#define ADAPT_CONTROL(n, ctt)                                                                                                                                  \
  static int vk_##n(FaxVmiM* m, const void* ctl) { return n##_control(m, (ctt)ctl); }
ADAPT_CREATE(v21tx, const int*)
ADAPT_CREATE(v21rx, const int*)
ADAPT_CREATE(v27tx, const int*)
ADAPT_CREATE(v27rx, const int*)
ADAPT_CREATE(v29tx, const int*)
ADAPT_CREATE(v29rx, const int*)
ADAPT_CREATE(v17tx, const int*)
ADAPT_CREATE(v17rx, const V17RxCfg*)
ADAPT_STATUS(v21tx, FaxV21Status*)
ADAPT_STATUS(v21rx, FaxV21Status*)
ADAPT_STATUS(v27tx, FaxV27Status*)
ADAPT_STATUS(v27rx, void*)
ADAPT_STATUS(v29tx, FaxV29Status*)
ADAPT_STATUS(v29rx, FaxV29Status*)
ADAPT_STATUS(v17tx, FaxV17Status*)
ADAPT_STATUS(v17rx, FaxV17Status*)
ADAPT_CONTROL(v21tx, const FaxV21Ctl*)
ADAPT_CONTROL(v21rx, const FaxV21Ctl*)
ADAPT_CONTROL(v27tx, const FaxV27Ctl*)
ADAPT_CONTROL(v27rx, const FaxV27Ctl*)
ADAPT_CONTROL(v29tx, const FaxV29Ctl*)
ADAPT_CONTROL(v29rx, const FaxV29Ctl*)
ADAPT_CONTROL(v17tx, const FaxV17Ctl*)
ADAPT_CONTROL(v17rx, const FaxV17Ctl*)

const FaxVmiCreateFn vxx_create[FAXVMI_NIDS] = {null_create, null_create, null_create, null_create, null_create, vc_v21tx, vc_v21rx,
                                                vc_v27tx,    vc_v27rx,    vc_v29tx,    vc_v29rx,    vc_v17tx,    vc_v17rx};
const FaxVmiDeleteFn vxx_delete[FAXVMI_NIDS] = {null_delete,  null_delete,  null_delete,  null_delete,  null_delete,  v21tx_delete, v21rx_delete,
                                                v27tx_delete, v27rx_delete, v29tx_delete, v29rx_delete, v17tx_delete, v17rx_delete};
const FaxVmiProcessFn vxx_process[FAXVMI_NIDS] = {null_process,  null_process,  null_process,  null_process,  null_process,  v21tx_process, v21rx_process,
                                                  v27tx_process, v27rx_process, v29tx_process, v29rx_process, v17tx_process, v17rx_process};
const FaxVmiStatusFn vxx_status[FAXVMI_NIDS] = {null_status, null_status, null_status, null_status, null_status, vs_v21tx, vs_v21rx,
                                                vs_v27tx,    vs_v27rx,    vs_v29tx,    vs_v29rx,    vs_v17tx,    vs_v17rx};
const FaxVmiControlFn vxx_control[FAXVMI_NIDS] = {null_control, null_control, null_control, null_control, null_control, vk_v21tx, vk_v21rx,
                                                  vk_v27tx,     vk_v27rx,     vk_v29tx,     vk_v29rx,     vk_v17tx,     vk_v17rx};
const FaxVmiMessageFn vxx_message[FAXVMI_NIDS] = {null_message,  null_message,  null_message,  null_message,  null_message,  v21tx_message, v21rx_message,
                                                  v27tx_message, v27rx_message, v29tx_message, v29rx_message, v17tx_message, v17rx_message};

/** Initialisers of the data modems {V.27ter, V.29, V.17}, transmit and receive; the class 1 layer replaces an entry only in test harnesses (writable data). */
FaxVmiInitFn fax_init_vmi_data_tx_modem[3] = {fax_init_vmi_v27tx, fax_init_vmi_v29tx, fax_init_vmi_v17tx};
FaxVmiInitFn fax_init_vmi_data_rx_modem[3] = {fax_init_vmi_v27rx, fax_init_vmi_v29rx, fax_init_vmi_v17rx};

/* the framing tables: indexed by FaxVmiCfg.mode (0 simple, 1 asynchronous, 2 HDLC) */
typedef int (*PackFn)(FaxVmi* o, unsigned short* buf, short n);
typedef int (*UnpackFn)(FaxVmi* o, unsigned short* out, short n);
typedef void (*RevFn)(unsigned short* p, short n);
static const UnpackFn unpack_tab[3] = {faxvmi_simp_unpack, faxvmi_asyc_unpack, faxvmi_hdlc_unframe};
static const PackFn pack_tab[3] = {faxvmi_simp_pack, faxvmi_asyc_pack, faxvmi_hdlc_frame};
static const RevFn rev_tab[3] = {faxvmi_byte_reverse, faxvmi_byte_reverse, faxvmi_frame_reverse};

/** Default config: simple framing, no bit reversal, ring minimum 0x80, 0x32 items per call, 0x80 HDLC frame words, null modem. */
const FaxVmiCfg FAXVMI_CFG = {0, 0, 0x80, 0x32, 0x80, 0, NULL, 0};
const FaxVmiCtl FAXVMI_CTL = {0, 0, 0, 0, 0, 0, 0, NULL};
const FaxVmiSts FAXVMI_STS = {0, 0, 0, 0, 0, 0, 0, NULL};

/* ===================================================================================================================== the FAXVMI object */
/**
 * @brief Create or reset a VMI object.
 * @param o    NULL to allocate the object, its framing state, the ring (max(ringMin, maxItems + 3) items), the HDLC frame buffer (frmWords items) and the
 *             modem sub-object with its 50-word bit buffer and 200-word scratch buffer; a non-NULL object keeps its buffers and only has their contents reset
 * @param cfg  config (copied into the object), or NULL for FAXVMI_CFG
 * @return the object, in the initial framing state (ring empty, txEmpty set, HDLC opening flags 2), with the modem created through vxx_create[cfg.id]
 * @warning The results of sysdep_malloc() are not checked.  For a non-NULL `o` the ring and frame buffer are not resized to a changed config.
 */
FaxVmi* FAXVMI_create(FaxVmi* o, const FaxVmiCfg* cfg)
{
  int fresh = 0;
  FaxVmiSt* st;
  FaxVmiM* m;
  unsigned short *ring, *frm;
  unsigned short i, size, fsz;
  if (o == NULL)
  {
    o = (FaxVmi*)sysdep_malloc((unsigned int)sizeof(FaxVmi));
    st = (FaxVmiSt*)sysdep_malloc((unsigned int)sizeof(FaxVmiSt));
    o->st = st;
    fresh = 1;
    m = (FaxVmiM*)sysdep_malloc((unsigned int)sizeof(FaxVmiM));
    o->m = m;
  }

  st = o->st; /* the pointers, sizes and fields the code reads are kept in locals: the code does not depend on the aliasing assumptions */
  m = o->m;
  if (cfg == NULL)
    cfg = &FAXVMI_CFG;
  o->cfg = *cfg;

  /* the ring */
  if (fresh)
  {
    unsigned s08 = o->cfg.ringMin, s3 = (unsigned)o->cfg.maxItems + 3u;
    unsigned short sz = (s08 >= s3) ? (unsigned short)s08 : (unsigned short)s3;
    ring = (unsigned short*)sysdep_malloc(2u * sz);
    st->ring = ring;
    st->size = sz;
    size = sz;
  }
  else
  {
    ring = st->ring;
    size = st->size;
  }
  for (i = 0; size != 0 && size > i; i = (unsigned short)(i + 1))
    ring[i] = 0;

  /* the framing state: empty ring, transmit and receive bit states reset */
  st->rd = 0;
  st->wr = 0;
  st->cnt = 0;
  st->tx_nbits = 0;
  st->ovf = 0;
  st->tx_mask = 0;
  st->tx_cur = -1;
  st->tx_acc = -1;
  st->rx_mask = 0;
  st->rx_cur = -1;
  st->rx_acc = -1;
  st->pad2e = st->pad1e; /* copies the unused pad half word (it has no meaning; for a new object it is uninitialised heap) */
  st->rx_nbits = 0;
  st->rx_state = 1;
  st->breakLeft = 0;
  st->breakReq = 0;
  st->rxBreak = 0;

  /* the HDLC receive frame buffer */
  if (fresh)
  {
    unsigned short s0c = o->cfg.frmWords;
    frm = (unsigned short*)sysdep_malloc(2u * s0c);
    st->frm = frm;
    st->frm_size = s0c;
    fsz = s0c;
  }
  else
  {
    frm = st->frm;
    fsz = st->frm_size;
  }
  for (i = 0; fsz != 0 && fsz > i; i = (unsigned short)(i + 1))
    frm[i] = 0;
  st->tx_left = 0;
  st->flen = 0;
  st->hskip = 2;
  st->tx_flag = 1;
  st->ones = 0;
  st->in_frame = 0;

  /* the modem sub-object: bit buffer filled with 0xffff, scratch buffer cleared, then the modem's own create */
  {
    short *bits, *buf0;
    short k;
    if (fresh)
    {
      m->eng = NULL;
      bits = (short*)sysdep_malloc(0x64);
      m->u4.bits = bits;
      buf0 = (short*)sysdep_malloc(0x190);
      m->buf0 = buf0;
    }
    else
    {
      bits = m->u4.bits;
      buf0 = m->buf0;
    }
    for (k = 0; k <= 0x31; k++)
      bits[k] = (short)0xffff;
    for (k = 0; k <= 0xc7; k++)
      buf0[k] = 0;
  }
  vxx_create[o->cfg.id](m, o->cfg.mcfg);
  o->txEmpty = 1;
  o->rxOverflow = 0;
  o->stateWord = 0;
  return o;
}

/**
 * @brief Delete the modem and free everything the object owns (scratch and bit buffers, frame buffer, ring, sub-object, framing state, the object).
 * @param o  the object; freed also when the caller supplied it to FAXVMI_create()
 */
void FAXVMI_delete(FaxVmi* o)
{
  vxx_delete[o->cfg.id](o->m);
  sysdep_free(o->m->buf0);
  sysdep_free(o->m->u4.bits);
  sysdep_free(o->st->frm);
  sysdep_free(o->st->ring);
  sysdep_free(o->m);
  sysdep_free(o->st);
  sysdep_free(o);
}

/**
 * @brief Run one block through the framing layer and the modem (see the pipeline in the file header).
 * @param o        VMI object
 * @param buf      T.30 side: bytes / frames to transmit; on return the received bytes / frames
 * @param samples  modem side: samples for the modem (receive) or the buffer it fills (transmit)
 * @param n        in: items in `buf`; out: items received
 * @param cnt2     in / out: sample count, passed to the modem process function
 * @return the state word: the modem's state with FAXVMI_ST_TX_EMPTY (transmit queue empty), FAXVMI_ST_QUEUE_FULL (ring room below cfg.maxItems),
 *         FAXVMI_ST_QUEUE_OVF (items refused), FAXVMI_ST_RX_OVERFLOW added; bit 28 is cleared when a break pattern was received
 */
unsigned int FAXVMI_process(FaxVmi* o, unsigned short* buf, short* samples, unsigned short* n, unsigned short* cnt2)
{
  unsigned short local;
  unsigned int r, state;
  FaxVmiSt* st;
  int ur;
  if (o->cfg.flag != 0)
    rev_tab[(unsigned short)o->cfg.mode](buf, (short)*n);
  local = (unsigned short)pack_tab[(unsigned short)o->cfg.mode](o, buf, (short)*n);
  r = (unsigned int)vxx_process[o->cfg.id](o->m, samples, &local, cnt2);
  ur = unpack_tab[(unsigned short)o->cfg.mode](o, buf, (short)local);
  *n = (unsigned short)ur;
  if (o->cfg.flag != 0)
    rev_tab[(unsigned short)o->cfg.mode](buf, (short)ur);

  /* the state word */
  state = r & 0xe0ffffffu; /* the engine's bits 0 .. 23 and 29 .. 31; bits 24 .. 28 are set below (24 .. 27 as FAXVMI_ST_*) */
  if (o->txEmpty != 0)
    state |= FAXVMI_ST_TX_EMPTY;
  st = o->st;
  if ((int)st->size - (int)st->cnt < (int)o->cfg.maxItems)
    state |= FAXVMI_ST_QUEUE_FULL;
  if (st->ovf != 0)
    state |= FAXVMI_ST_QUEUE_OVF;
  if (o->rxOverflow != 0)
    state |= FAXVMI_ST_RX_OVERFLOW;
  if (st->rxBreak != 0)
    state &= 0xefffffffu; /* a break pattern clears bit 28 (no FAXVMI_ST_ enumerator for it) */
  o->stateWord = state;
  return state;
}

/**
 * @brief Fill the status record: ring room, overflow counts and flags, whether the opening flags were seen, and the modem's own status.
 * @param st  record to fill, or NULL (returns -1)
 * @return the modem status function's result (0 when the record has no modem status pointer)
 */
int FAXVMI_status(FaxVmi* o, FaxVmiSts* st)
{
  int r = 0;
  FaxVmiSt* s = o->st; /* everything is read into locals before the first store: the code does not depend on the aliasing assumptions */
  unsigned short id = o->cfg.id;
  FaxVmiM* m = o->m;
  void* msts;
  int txEmpty, rxOverflow, ovf, rxBreak, hskip;
  unsigned size, cnt;
  if (st == NULL)
    return -1;
  msts = st->msts;
  if (msts != NULL)
    r = vxx_status[id](m, msts);
  size = s->size;
  cnt = s->cnt;
  ovf = s->ovf;
  txEmpty = o->txEmpty;
  rxOverflow = o->rxOverflow;
  hskip = s->hskip;
  rxBreak = s->rxBreak;
  st->ringFree = (short)(size - cnt);
  st->ovf = ovf;
  st->txEmpty = txEmpty;
  st->rxOverflow = rxOverflow;
  st->hskipDone = hskip <= 1;
  st->rxBreak = rxBreak;
  return r;
}

/**
 * @brief Apply a control record: the modem's control, a framing reset with a new mode, a ring flush, and the break request.
 * @param ctl  control record, or NULL (returns -1)
 * @return the modem control function's result (0 when the record has no modem control pointer)
 *
 * With `rxReset` set and `newMode` at most 2 the framing state is reset (frame buffer cleared, bit states cleared, opening flags back to 2) and the
 * mode is replaced.  `flushRing` empties the ring.  `breakCount` / `breakReq` are always stored.
 */
int FAXVMI_control(FaxVmi* o, const FaxVmiCtl* ctl)
{
  int r = 0;
  FaxVmiSt* s = o->st;
  unsigned short id = o->cfg.id;
  FaxVmiM* m = o->m;
  const void* mctl;
  int flushRing, breakReq, rxReset;
  unsigned short breakCount, newMode;
  unsigned short i;
  if (ctl == NULL)
    return -1;
  mctl = ctl->mctl;
  if (mctl != NULL)
    r = vxx_control[id](m, mctl);
  rxReset = ctl->rxReset;
  flushRing = ctl->flushRing;
  breakReq = ctl->breakReq;
  breakCount = ctl->breakCount;
  newMode = ctl->newMode;
  if (rxReset != 0 && newMode <= 2)
  {
    unsigned short* frm = s->frm;
    unsigned short fsz = s->frm_size;
    for (i = 0; fsz != 0 && fsz > i; i = (unsigned short)(i + 1))
      frm[i] = 0;
    s->tx_left = 0;
    s->flen = 0;
    s->ones = 0;
    s->hskip = 2;
    s->in_frame = 0;
    s->tx_flag = 1;
    s->tx_nbits = 0;
    s->rx_nbits = 0;
    s->pad2e = s->pad1e;
    s->breakLeft = 0;
    s->rx_mask = 0;
    s->rx_cur = -1;
    s->rx_acc = -1;
    s->rx_state = 1;
    s->breakReq = 0;
    s->rxBreak = 0;
    s->tx_mask = 0;
    s->tx_cur = -1;
    s->tx_acc = -1;
    o->cfg.mode = (int)(((unsigned)o->cfg.mode & 0xffff0000u) | newMode); /* a word store: the upper half of the mode dword is kept */
  }
  if (flushRing != 0)
  {
    unsigned short* ring = s->ring;
    unsigned short size = s->size;
    for (i = 0; size != 0 && size > i; i = (unsigned short)(i + 1))
      ring[i] = 0;
    s->rd = 0;
    s->wr = 0;
    s->cnt = 0;
    s->ovf = 0;
  }
  s->breakLeft = breakCount;
  s->breakReq = breakReq;
  return r;
}

/** @brief The modem's status text for status code `idx` (the low byte is used), or NULL. */
const char* FAXVMI_message(FaxVmi* o, int idx)
{
  const char* msg = NULL;
  vxx_message[o->cfg.id](o->m, (unsigned char)idx, &msg);
  return msg;
}
