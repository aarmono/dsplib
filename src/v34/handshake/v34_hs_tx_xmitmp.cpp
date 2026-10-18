/**
 * @file v34_hs_tx_xmitmp.cpp
 * @brief V.34 handshake transmit state XMITMP: send the MP (modulation parameters) message.
 *
 * One entry point, v04_tx_xmitmp(), the case for transmit state 0x43 of the per-sample loop of v34handshak() (v34_hs_dispatch.cpp).  Callees:
 * txmit(), initdigital(), v04_bitq_getbit() (the bit-queue reader of v34_hs_getbit_bulkdelay.cpp), dsplibs_debug_printf().
 *
 * XMITMP sends the MP message as one 4-point (2 bits) or 16-point (4 bits, rxSig_ == 0x89b0) symbol per call: the bits come from the bit queue
 * `bitQ_`, are collected in txBits_ (bit n at position n), scrambled (the same 23-bit scrambler as EXMIT, taps selected by txFlags_ bit 0) and
 * transmitted (sendDibit() / sendQuad()).  While the bits are collected the symbol counter k56Idx_ steps and the receiver flags decide about the
 * MP exchange:
 * - the MP' marker positions 0x55 / 0x58 (rxFlags_ bit 5 set) or 0xbb / 0xbc (clear) stretch the queue (shift left by 3 / 1 bits);
 * - k56Idx_ == w3590_ stretches it by one bit and moves w3590_ on by 0x11;
 * - 0x58 / 0xbc end a pass: w359e_ counts the MP repetitions.  When the far end's MP is seen (word 0 bit 0 of mp) the queue is re-armed
 *   (restartMp(): "Starting txmit MP again", 0x12 bits of 0x3fffe, length 0x30 / 0x90 by rxFlags_ bit 5) or, after four passes with
 *   (rxFlags_ & 0x90) == 0x90, the transmitter goes to EXMIT (initdigital() once, rxFlags_ bit 5 cleared) and the symbol collected so far is sent.
 *
 * @note The symbol-size test reloads rxSig_ after every bit.  The getbit result is shifted by (n & 31) as a sign-extended value (-1 sets the bit
 *       and every bit above it).  The low byte of rxFlags_ is read once per bit (the 0x20 tests) while the 0x90 / 0x18 tests re-read the word.
 */

#include <limits.h>
#include "v34_handshake.h"
#include "v34/v34_internal.h"

/** @brief Read the 16-bit word 0 of the bit-queue record (byte array unk00_). */
static inline unsigned qWord0(const tag_bitQ* q)
{
  uint16_t v;
  __builtin_memcpy(&v, q->unk00_, 2);
  return v;
}
/** @brief Write the 16-bit word 0 of the bit-queue record (byte array unk00_). */
static inline void qSetWord0(tag_bitQ* q, unsigned v)
{
  uint16_t w = (uint16_t)v;
  __builtin_memcpy(q->unk00_, &w, 2);
}

/**
 * @brief Re-arm the bit queue to send the 18-bit MP pattern again.
 * @param obj  handshake object (k56Idx_ and w3590_ are reset)
 * @param q    bit queue to re-arm
 * @param rfl  rxFlags_ value read by the caller (bit 5 selects the queue length 0x30 or 0x90)
 * @param lvl  debug level read by the caller (above 1 prints "Starting txmit MP again")
 */
static void restartMp(tagV34Object* obj, tag_bitQ* q, unsigned rfl, unsigned lvl)
{
  q->tail_.crc = USHRT_MAX;
  q->tail_.pos = 0;
  q->tail_.widx = 0;
  q->tail_.pass = 0;
  q->tail_.chunk = 0x10;
  q->tail_.total = (uint16_t)(((rfl >> 5) & 1u) != 0 ? 0x30u : 0x90u);
  q->tail_.crcOn = 1;
  q->tail_.nbits = 0x12;
  q->tail_.restartBits = 0x12;
  q->tail_.loop = 0;
  q->tail_.acc = 0x3fffeu;
  q->tail_.restartAcc = 0x3fffeu;
  obj->tx.k56Idx_ = 0;
  obj->w3590_ = 0x22;
  if (lvl > 1u)
    dsplibs_debug_printf("V34MP, Starting txmit MP again(%d), rxflgs=0x%x,txflags=0x%x\n", (int)(int16_t)obj->w359e_, (unsigned)obj->rx.rxFlags_,
                         (int)(int16_t)obj->tx.txFlags_);
}

/**
 * @brief Send the 16-point symbol: two scrambler passes, then txBits_ (differential, mod 4) and the point from vect16.
 * @param obj  handshake object; advances scrState_, sets txBits_, txPoint_, txPrev_ and calls txmit()
 */
static void sendQuad(tagV34Object* obj)
{
  unsigned fl = obj->tx.txFlags_;
  const bool tap13 = (fl & 1u) == 0;
  int bits = (int16_t)obj->tx.txBits_;
  uint32_t e = v04Scramble(obj->tx.scrState_, tap13, bits);
  unsigned b;
  obj->tx.scrState_ = e;
  obj->tx.txBits_ = (uint16_t)(((e >> 29) + (unsigned)obj->tx.txPrev_) & 3u);
  e = v04Scramble(obj->tx.scrState_, tap13, bits >> 2);
  obj->tx.scrState_ = e;
  b = obj->tx.txBits_;
  obj->tx.txPoint_ = (uint32_t)vect16[(e >> 29) + (unsigned)((int16_t)b) * 4u];
  obj->tx.txPrev_ = (uint16_t)b;
  txmit(obj);
}

/**
 * @brief Send the 4-point symbol: one scrambler pass, txBits_ (differential, mod 4) and the point from vect4.
 * @param obj  handshake object; advances scrState_, sets txBits_, txPoint_, txPrev_ and calls txmit()
 */
static void sendDibit(tagV34Object* obj)
{
  int bits = (int16_t)obj->tx.txBits_;
  const bool tap13 = (obj->tx.txFlags_ & 1u) == 0;
  uint32_t e = v04Scramble(obj->tx.scrState_, tap13, bits);
  unsigned b;
  obj->tx.scrState_ = e;
  b = ((e >> 29) + (unsigned)obj->tx.txPrev_) & 3u;
  obj->tx.txBits_ = (uint16_t)b;
  obj->tx.txPoint_ = (uint32_t)vect4[b];
  obj->tx.txPrev_ = (uint16_t)b;
  txmit(obj);
}

/**
 * @brief Transmit state XMITMP (0x43): collect the next symbol's bits from the MP bit queue, handle the MP exchange, and transmit the symbol.
 * @param obj  handshake object; reads bitQ_, mp, rxFlags_, rxSig_, writes txBits_, k56Idx_, w359e_, w3590_, the queue words and the scrambler
 * @note  After four MP passes with the far end's MP seen the state changes to EXMIT and the symbol collected so far is still sent.
 */
extern "C" void v04_tx_xmitmp(tagV34Object* obj)
{
  int n = 0;
  bool wide;

  obj->tx.txBits_ = 0;
  wide = obj->rx.rxSig_ == 0x89b0;

  /* collect 2 (or 4) bits, tracking the position k56Idx_ in the MP exchange */
  for (;;)
  {
    unsigned rf;
    int k;
    bool stretch3 = false;
    bool stretch1 = false;

    if (wide ? !(n <= 3) : (n > 1))
      break;

    {
      int bit = v04_bitq_getbit(obj->bitQ_);
      obj->tx.txBits_ = (uint16_t)(((uint32_t)bit << (n & 31)) | obj->tx.txBits_);
    }

    obj->tx.k56Idx_ = (uint16_t)(obj->tx.k56Idx_ + 1u);
    k = (int16_t)obj->tx.k56Idx_;
    rf = obj->rx.rxFlags_;
    if ((rf & 0x20u) != 0)
    {
      if (k == 0x55)
        stretch3 = true;
    }
    else
    {
      if (k == 0xbb)
        stretch1 = true;
    }

    /* MP' marker: stretch the queue by 3 or 1 bits; otherwise check for the end of a pass */
    if (stretch3 || stretch1)
    {
      tag_bitQ* q = obj->bitQ_;
      unsigned w = q->tail_.nbits;
      if ((rf & 0x20u) != 0)
      {
        q->tail_.acc <<= 3;
        q->tail_.nbits = (uint16_t)(w + 3u);
      }
      else
      {
        q->tail_.acc <<= 1;
        q->tail_.nbits = (uint16_t)(w + 1u);
      }
    }
    else
    {
      bool pass_end;
      k = (int16_t)obj->tx.k56Idx_;
      pass_end = (rf & 0x20u) != 0 ? k == 0x58 : k == 0xbc;
      if (pass_end)
      {
        unsigned old = obj->w359e_;
        unsigned rfl;
        unsigned lvl;
        tag_bitQ* q;
        obj->w359e_ = (uint16_t)(old + 1u);
        if ((((uint8_t*)&obj->mp)[0] & 1u) == 0)
        {
          rfl = obj->rx.rxFlags_;
          if ((rfl & 0x18u) != 0x10u)
            goto rearm;
          goto detected;
        }
        rfl = obj->rx.rxFlags_;
        if ((rfl & 0x90u) == 0x90u && (int16_t)(old + 1u) > 3)
        {
          unsigned sig;
          if (obj->w3598_ == 0)
          {
            initdigital(obj);
            obj->w3598_ = 1;
          }
          v04SetTx(obj, obj->txState_, V34HS_EXMIT);
          obj->tx.k56Idx_ = 0;
          rfl = obj->rx.rxFlags_;
          sig = obj->rx.rxSig_;
          obj->rx.rxFlags_ = (uint16_t)(rfl & ~0x20u);
          if ((uint16_t)sig == 0x89b0)
            sendQuad(obj);
          else
            sendDibit(obj);
          return;
        }
        if ((rfl & 0x18u) != 0x10u)
          goto rearm;
      detected:
        if ((int16_t)obj->w359e_ <= 1)
        {
          lvl = dsplibs_debug_level;
          q = obj->bitQ_;
          restartMp(obj, q, rfl, lvl);
        }
        else
        {
          q = obj->bitQ_;
          if ((qWord0(q) & 1u) == 0)
            obj->w359e_ = 0;
          qSetWord0(q, qWord0(q) | 1u);
          lvl = dsplibs_debug_level;
          if (lvl > 1u)
          {
            dsplibs_debug_printf("V34MP, MP detected, starting MP' txmit");
            lvl = dsplibs_debug_level;
          }
          rfl = obj->rx.rxFlags_;
          restartMp(obj, obj->bitQ_, rfl, lvl);
        }
        goto next_bit;
      rearm:
        lvl = dsplibs_debug_level;
        restartMp(obj, obj->bitQ_, rfl, lvl);
        goto next_bit;
      }
      else if ((uint16_t)obj->tx.k56Idx_ == obj->w3590_)
      {
        tag_bitQ* q = obj->bitQ_;
        unsigned w = q->tail_.nbits;
        q->tail_.acc <<= 1;
        q->tail_.nbits = (uint16_t)(w + 1u);
        obj->w3590_ = (uint16_t)(obj->w3590_ + 0x11u);
      }
    }
  next_bit:
    n = (int16_t)(n + 1);
    wide = obj->rx.rxSig_ == 0x89b0;
  }

  /* transmit the collected symbol */
  if (wide)
    sendQuad(obj);
  else
    sendDibit(obj);
}
