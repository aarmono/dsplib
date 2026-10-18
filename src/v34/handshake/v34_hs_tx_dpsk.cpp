/**
 * @file v34_hs_tx_dpsk.cpp
 * @brief V.34 handshake, transmit state TX_DPSK: the INFO0 / MOH message sent bit by bit as a differential phase shift, and the bit-queue reader.
 *
 * Contents:
 * - v04_bitq_getbit(): the reader of the bit queue (`tag_bitQ`); also used by v34_hs_tx_xmitmp.cpp.  The queue holds a 16-bit word array at +0, then the
 *   CRC-16 (x^16 + x^12 + x^5 + 1, shifted in MSB first as the bits are loaded) in crc, the CRC enable crcOn, the message length total, the chunk length
 *   chunk, the read position pos (bits) and word index widx, the "more messages" flag loop, the message counter pass, the shift register acc with its
 *   bit count nbits, and the restart values restartAcc / restartBits.
 * - v04_tx_dpsk(): the transmit-state handler.  Every call takes ONE bit from the queue `bitQ_`, flips the tone select toneSel_ with it and transmits the
 *   point vect4[(toneSel_ & 1) * 2].  When the queue is empty (the bit is -1) the MOH message state machine (mohE8_ / mohF0_ / mohF4_ / mohF8_ /
 *   mohF9_) decides: stay in TONE_AB (mohE8_ clear), or end the message (mohF8_ set: move to MOH_SILENCE / disconnect / retrain / reply with MHcda or
 *   MHfrr), or restart it (mohF8_ clear: read the first bit again and transmit).
 *
 * The reader exists in two copies: v04_bitq_getbit() (the normal one) and restartBit() (the restart one, which differs in what it reloads).  Both branch on
 * the 16-bit remainder `total - pos` (0xfff0 selects the 4-bit constant 0xf, 0 selects the 16 CRC bits) and fall back to the out-of-line getbit()
 * when the next message is armed (loop set).
 */

#include <limits.h>
#include "v34_handshake.h"
#include "v34/v34_internal.h"
#include "vpcm/vpcm_v34_api.h"

/** @brief Word `idx` (taken as a signed 16-bit index) of the queue's data array, which starts at the beginning of the record. */
static inline unsigned qWord(const tag_bitQ* q, int idx)
{
  uint16_t v;
  __builtin_memcpy(&v, (const uint8_t*)q + 2 * (long)idx, 2);
  return v;
}

/**
 * @brief Feed the queue CRC-16 (crc, polynomial 0x1021) with the `count` bits just loaded into the shift register, MSB first.
 * @param q      bit queue
 * @param value  the shift register contents
 * @param count  number of bits to feed (at least 1: the loop runs until the 16-bit counter reaches 0)
 */
static inline void crcBits(tag_bitQ* q, uint32_t value, int count)
{
  int left = count;
  do
  {
    unsigned crc = q->tail_.crc;
    unsigned sign = (crc & 0x8000u) != 0 ? 1u : 0u;
    left = (int16_t)(left - 1);
    crc = crc + crc;
    if (((value >> (left & 31)) & 1u) != 0)
      sign ^= 1u;
    if (sign != 0)
      crc ^= 0x1021u;
    q->tail_.crc = (uint16_t)crc;
  } while (left != 0);
}

/**
 * @brief Shared tail of both readers: take the next bit out of the shift register.
 * @param q         bit queue; nbits is set to bitCount - 1
 * @param bitReg    shift register contents
 * @param bitCount  number of bits the register holds (at least 1)
 * @return the bit (0 or 1), taken from position bitCount - 1
 */
static inline int popBit(tag_bitQ* q, uint32_t bitReg, unsigned bitCount)
{
  unsigned left = bitCount - 1u;
  q->tail_.nbits = (uint16_t)left;
  return (int)((bitReg >> ((int16_t)left & 31)) & 1u);
}

/**
 * @brief Take the next message bit from a bit queue.
 * @param q  bit queue (the INFO0 / INFO1a / MOH record)
 * @return the bit (0 or 1), or -1 at the end of the message when no next message is armed; an armed next message (loop set) is restarted and read
 *         through getbit()
 * @note  Refills the shift register acc from the data words in chunks of chunk bits, feeding the CRC as it goes; after the last chunk it supplies the
 *        4-bit constant 0xf or the 16 CRC bits, depending on total - pos.
 */
extern "C" int v04_bitq_getbit(tag_bitQ* q)
{
  unsigned bitCount = q->tail_.nbits;
  unsigned pos;
  int count;
  uint32_t bitReg;

  /* bits left in the shift register: just pop one */
  if ((uint16_t)bitCount != 0)
    goto have_bits;

  /* register empty: the remainder total - pos says what comes next (data chunk, 4-bit constant, CRC, or end) */
  pos = q->tail_.pos;
  count = (int16_t)(q->tail_.total - pos);
  if ((int16_t)count <= 0)
  {
    if (count != 0)
    {
      if ((int16_t)count != (int16_t)0xfff0)
        goto next_message;

      q->tail_.acc = 0xf;
      bitReg = 0xf;
      q->tail_.nbits = 4;
      q->tail_.pos = (uint16_t)(pos + 4u);
      return popBit(q, bitReg, 4);
    }

    if (q->tail_.crcOn == 0)
      goto next_message;

    count = (int)q->tail_.crc;
    q->tail_.nbits = 0x10;
    q->tail_.pos = (uint16_t)(pos + 0x10u);
    q->tail_.acc = (uint32_t)count;
    return popBit(q, (uint32_t)count, 0x10);
  }

  /* load the next chunk of data bits */
  {
    unsigned chunkLen = q->tail_.chunk;
    if ((int16_t)chunkLen <= (int16_t)count)
    { /* the whole chunk fits */
      uint32_t e;
      unsigned idx;
      q->tail_.nbits = (uint16_t)chunkLen;
      e = q->tail_.acc;
      e <<= ((int16_t)chunkLen & 31);
      idx = q->tail_.widx;
      e |= qWord(q, (int16_t)idx);
      q->tail_.widx = (uint16_t)(idx + 1u);
      q->tail_.acc = e;
      q->tail_.pos = (uint16_t)(pos + chunkLen);
      bitCount = chunkLen;
      count = (int16_t)chunkLen;
    }
    else
    { /* part of the chunk */
      uint32_t e;
      q->tail_.nbits = (uint16_t)count;
      e = q->tail_.acc;
      e = (e << (count & 31)) | qWord(q, (int16_t)q->tail_.widx);
      q->tail_.pos = (uint16_t)(count + pos);
      q->tail_.acc = e;
      bitCount = (unsigned)count;
    }
  }

  if (q->tail_.crcOn != 0 && count != 0)
  {
    bitReg = q->tail_.acc;
    crcBits(q, bitReg, count);
    return popBit(q, bitReg, bitCount);
  }

have_bits:
  return popBit(q, q->tail_.acc, bitCount);

next_message:
  /* end of message: -1, or restart from the stored values when the next message is armed */
  if (q->tail_.loop == 0)
    return -1;

  q->tail_.crc = USHRT_MAX;
  {
    unsigned n = q->tail_.pass;
    uint32_t restartReg = q->tail_.restartAcc;
    unsigned restartBits = q->tail_.restartBits;
    q->tail_.pos = 0;
    q->tail_.pass = (uint16_t)(n + 1u);
    q->tail_.nbits = (uint16_t)restartBits;
    q->tail_.widx = 0;
    q->tail_.acc = restartReg;
  }
  return (int)getbit(q);
}

/**
 * @brief Restart the message from the stored restart values and return its first bit.
 * @param q  bit queue
 * @return the first bit (0 or 1); -1 when the restarted message is empty and no next message is armed
 * @warning The caller (v04_tx_dpsk) does not test for -1: sendBit() with -1 inverts every bit of toneSel_, so the transmitted tone flips.
 */
static int restartBit(tag_bitQ* q)
{
  uint32_t restartReg = q->tail_.restartAcc;
  unsigned restartBits;
  int count;
  uint32_t bitReg;

  q->tail_.pass = (uint16_t)(q->tail_.pass + 1u);
  restartBits = q->tail_.restartBits;
  if ((uint16_t)restartBits != 0)
  {
    q->tail_.crc = USHRT_MAX;
    bitReg = restartReg;
    q->tail_.pos = 0;
    q->tail_.widx = 0;
    q->tail_.acc = restartReg;
    q->tail_.nbits = (uint16_t)restartBits;
    goto pop;
  }

  /* no stored register bits: load the first chunk (or constant / CRC) of the message */
  count = (int16_t)q->tail_.total;
  if ((int16_t)count <= 0)
  {
    if (count != 0)
    {
      if ((int16_t)count != (int16_t)0xfff0)
        goto next_message;
      q->tail_.crc = USHRT_MAX;
      bitReg = 0xf;
      q->tail_.widx = 0;
      q->tail_.acc = 0xf;
      q->tail_.pos = 4;
      q->tail_.nbits = 4;
      goto pop;
    }
    if (q->tail_.crcOn == 0)
      goto next_message;
    q->tail_.crc = USHRT_MAX;
    bitReg = 0xffffu;
    q->tail_.widx = 0;
    q->tail_.acc = 0xffffu;
    q->tail_.pos = 0x10;
    q->tail_.nbits = 0x10;
    goto pop;
  }
  {
    unsigned chunkLen = q->tail_.chunk;
    q->tail_.crc = USHRT_MAX;
    if ((int16_t)chunkLen > (int16_t)count)
    {
      q->tail_.widx = 0;
      restartReg = (restartReg << (count & 31)) | qWord(q, 0);
      q->tail_.pos = (uint16_t)count;
      q->tail_.nbits = (uint16_t)count;
      q->tail_.acc = restartReg;
    }
    else
    {
      count = (int16_t)chunkLen;
      restartReg = (restartReg << (count & 31)) | qWord(q, 0);
      q->tail_.pos = (uint16_t)chunkLen;
      q->tail_.nbits = (uint16_t)chunkLen;
      q->tail_.acc = restartReg;
      q->tail_.widx = 1;
    }
  }

  if (q->tail_.crcOn == 0 || count == 0)
  {
    bitReg = q->tail_.acc;
  }
  else
  {
    bitReg = q->tail_.acc;
    crcBits(q, bitReg, count);
  }

pop:
  {
    unsigned left = (unsigned)(uint16_t)q->tail_.nbits - 1u;
    q->tail_.nbits = (uint16_t)left;
    return (int)((bitReg >> ((int16_t)left & 31)) & 1u);
  }

next_message:
  if (q->tail_.loop == 0)
  {
    q->tail_.crc = USHRT_MAX;
    q->tail_.pos = 0;
    q->tail_.widx = 0;
    q->tail_.acc = restartReg;
    q->tail_.nbits = 0;
    return -1;
  }
  {
    unsigned n = q->tail_.pass;
    q->tail_.crc = USHRT_MAX;
    q->tail_.pos = 0;
    q->tail_.widx = 0;
    q->tail_.pass = (uint16_t)(n + 1u);
    q->tail_.acc = restartReg;
    q->tail_.nbits = 0;
  }
  return (int)getbit(q);
}

/** @brief Flip the tone select toneSel_ with `bit` (XOR) and transmit the point vect4[(toneSel_ & 1) * 2] selects. */
static void sendBit(tagV34Object* obj, int bit)
{
  unsigned t = (unsigned)(uint16_t)obj->toneSel_ ^ (unsigned)bit;
  obj->toneSel_ = (uint16_t)t;
  obj->tx.txPoint_ = (uint32_t)vect4[(t & 1u) * 2u];
  txmit(obj);
}

/** @brief Disconnect tail: transmit state MOH_CLEARDOWN, receive state WAIT, mohE4_ = 1. */
static void disconnectStates(tagV34Object* obj)
{
  v04SetTx(obj, obj->txState_, V34HS_MOH_CLEARDOWN);
  v04SetRx(obj, obj->rxState_, V34HS_WAIT);
  obj->mohE4_ = 1;
}

/**
 * @brief Transmit state TX_DPSK: send one message bit.
 * @param obj  V.34 module object
 * @note  With MOH active (mohE8_) k56Idx_ counts symbols; when it exceeds bulkDelay_ / 16 + 0x4b0 without the awaited message the handler disconnects (mohF9_
 *        set) or retrains (v34handshakinit(obj, V34HSINIT_RETRAIN), mohE6_ = 1); during a cleardown it terminates without acknowledge.
 * @warning The restart path takes its bit from restartBit(), which may return -1 and is not tested (see there).
 */
extern "C" void v04_tx_dpsk(tagV34Object* obj)
{
  int bit;
  tag_bitQ* bq;
  if (obj->mohE8_ != 0)
    obj->tx.k56Idx_ = (uint16_t)(obj->tx.k56Idx_ + 1u);

  /* normal case: a bit is available */
  bit = v04_bitq_getbit(obj->bitQ_);
  if (bit >= 0)
  {
    sendBit(obj, bit);
    return;
  }

  /* message end */
  if (obj->mohE8_ == 0)
  { /* no MOH: keep sending the tone (TONE_AB) */
    v04SetTx(obj, obj->txState_, V34HS_TONE_AB);
    return;
  }

  if (dsplibs_debug_level > 1u)
  {
    tag_bitQ* q = obj->bitQ_;
    dsplibs_debug_printf("End of current MOH msg: isterm=%d, count1(%d), pktcount(%d)...\r\n", (int)(int8_t)obj->mohF8_, (int)(int16_t)obj->tx.k56Idx_,
                         (int)(int16_t)q->tail_.pass);
  }

  /* not the terminating message: send it again */
  if (obj->mohF8_ == 0)
  {
    bit = restartBit(obj->bitQ_);
    sendBit(obj, bit);
    goto timeouts;
  }

  /* terminating message: decide by the MOH state (mohF0_ / mohF4_) */
  {
    unsigned f0 = obj->mohF0_;
    if (f0 == 1)
      goto to_moh_silence;
    if (f0 == 0)
    {
      unsigned f4 = obj->mohF4_;
      if (f4 == 4 || f4 == 0)
        goto to_moh_silence;
      if (f4 == 5)
        goto reply;
      goto illegal;
    }
    if (f0 > 3)
      goto clear_f8;
    disconnectStates(obj);
    goto clear_f8;
  }

to_moh_silence:
  v04SetTx(obj, obj->txState_, V34HS_MOH_SILENCE);
  v04SetRx(obj, obj->rxState_, V34HS_WAIT);
  obj->hsSymCnt_ = 0;
  obj->tx.k56Idx_ = 0;
  goto clear_f8;

illegal:
  if (dsplibs_debug_level > 1u)
    dsplibs_debug_printf("MOH: Illegal MH sequence under MHreq, initiating retrain\r\n");
  v34handshakinit(obj, V34HSINIT_RETRAIN);
  obj->mohE6_ = 1;
  goto clear_f8;

reply:
  /* MHnack received for MHreq: answer with MHcda (mohF9_ set) or MHfrr and re-arm the INFO0a queue for the reply */
  if (obj->mohE2_ != 3)
    obj->mohE2_ = 1;
  if (obj->mohF9_ != 0)
  {
    if (dsplibs_debug_level > 1u)
      dsplibs_debug_printf("MOH: MHnack received for MHreq, sending MHcda\r\n");
    obj->mohF0_ = 3;
  }
  else
  {
    if (dsplibs_debug_level > 1u)
      dsplibs_debug_printf("MOH: MHnack received for MHreq, sending MHfrr\r\n");
    obj->mohF0_ = 1;
  }

  VPcmV34SetMohMessageBits(obj, obj->bitQ_);
  obj->bitQ_ = (tag_bitQ*)&obj->info0a;
  bq = (tag_bitQ*)&obj->info0a;
  bq->tail_.crc = USHRT_MAX;
  bq->tail_.pos = 0;
  bq->tail_.widx = 0;
  bq->tail_.pass = 0;
  bq->tail_.total = 8;
  bq->tail_.chunk = 8;
  bq->tail_.crcOn = 1;
  bq->tail_.acc = 0xf72;
  bq->tail_.restartAcc = 0xf72;
  bq->tail_.nbits = 0xc;
  bq->tail_.restartBits = 0xc;
  bq->tail_.loop = 0;

clear_f8:
  obj->mohF8_ = 0;

timeouts:
  /* timeout waiting for the answering MH sequence */
  if ((int)(int16_t)obj->tx.k56Idx_ < ((int)(int16_t)obj->bulkDelay_ >> 4) + 0x4b0)
    return;
  if (obj->mohF9_ != 0)
  {
    if (dsplibs_debug_level > 1u)
      dsplibs_debug_printf("MOH: Timeout waiting for MH sequence under MHreq, disconnecting...\r\n");
    disconnectStates(obj);
    obj->mohE2_ = 1;
    return;
  }
  if ((uint32_t)(obj->mohF0_ - 2u) > 1u)
  {
    if (dsplibs_debug_level > 1u)
      dsplibs_debug_printf("MOH: Timeout waiting for MH sequence under MHreq, initiating retrain\r\n");
    v34handshakinit(obj, V34HSINIT_RETRAIN);
    obj->mohE6_ = 1;
    return;
  }
  if (dsplibs_debug_level > 1u)
    dsplibs_debug_printf("MOH: Timeout waiting for MH sequence under cleardown, terminating connection without acknowledge\r\n");
  disconnectStates(obj);
}
