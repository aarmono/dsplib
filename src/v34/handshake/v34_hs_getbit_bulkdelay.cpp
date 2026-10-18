/**
 * @file v34_hs_getbit_bulkdelay.cpp
 * @brief V.34 handshake helpers: the bit-queue reader getbit() and the round-trip-delay (bulk delay) setup ApplyBulkDelay().
 *
 * Both are plain `extern "C"` functions named v04_getbit_impl() and v04_apply_bulk_delay_impl().  The handlers of the other v34_hs_*.cpp files call
 * them as getbit() / ApplyBulkDelay(); the library build maps those names with `-Dgetbit=v04_getbit_impl -DApplyBulkDelay=v04_apply_bulk_delay_impl`
 * (V04_DEFS in dsplib.mk), so the unit tests can substitute recording stubs under the plain names.
 *
 * - v04_getbit_impl(q): reads the next bit from a bit queue (tag_bitQ, 0x30 bytes) used by the INFO / MP / MOH transmitters.  The queue holds a message
 *   as 16-bit data words (`unk00_`), a CRC register `crc` (CRC-CCITT, polynomial 0x1021) with enable `crcOn`, the message length `total` in bits, the
 *   position `pos`, the chunk length `chunk` (bits per data word), the word index `widx`, a "next message armed" flag `loop` with counter `pass`, the
 *   bit register `acc` with its bit count `nbits`, and the restart values `restartAcc` / `restartBits`.
 * - v04_apply_bulk_delay_impl(obj, delay): turns the round-trip-delay estimate into the bulk-delay line length `bulkWr_` and clears the delay line.
 *   For a plain V.34 call (no PCM, no K56flex mode) it also masks the far echo canceller when the delay is below 0x1e samples and shortens `dmaDelay_`.
 *
 * All counts are 16-bit values sign-extended before use, and shift counts are taken modulo 32.
 */

#include <limits.h>
#include "v34_handshake.h"
#include "core/dsp_host.h"

#include "v34_hs_bytes.h"

/**
 * @brief Return the next bit of a bit queue, refilling its bit register from the message when it runs empty.
 * @param q  bit queue (tag_bitQ), updated in place
 * @return 0 or 1, the most significant remaining bit of the bit register; -1 when the message and (if not armed) the restart are exhausted
 *
 * When the register is empty the next chunk of min(`chunk`, remaining) message bits is loaded and, if `crcOn` is set, fed into the CRC (MSB first).  After
 * the last message bit the 16 CRC bits follow (when `crcOn` is set), or the constant 0xf (4 bits) when the remaining count is -16; if the queue is armed
 * (`loop`) the message then restarts from `restartAcc` / `restartBits` and `pass` is incremented, else -1 is returned.
 * @warning A restart with `loop` set recurses; if the message length `total` is not positive (and `crcOn` is 0, or the length is negative) every restart
 *          ends in another restart and the recursion never terminates.  Queues are armed with a positive length today.
 * @warning The word index `widx` is not checked against the size of the word buffer `unk00_`.
 */
extern "C" short v04_getbit_impl(tag_bitQ* q)
{
  unsigned bitCount = q->tail_.nbits;
  unsigned bitReg, pos, acc, tmp;
  int count;
  if ((uint16_t)bitCount != 0)
  {
    bitReg = q->tail_.acc;
    goto emitBit;
  }
  pos = q->tail_.pos;
  acc = (unsigned)q->tail_.total - pos;
  count = (int16_t)acc;
  if ((int16_t)count <= 0)
    goto messageExhausted;
  bitReg = q->tail_.chunk;
  if ((int16_t)bitReg <= (int16_t)count)
    goto loadFullChunk;
  /* fewer bits left than a chunk: take the remainder */
  {
    int wordIdx = (int16_t)q->tail_.widx;
    q->tail_.nbits = (uint16_t)count;
    acc = q->tail_.acc;
    tmp = ld16(q->unk00_, 2 * (unsigned)wordIdx);
    acc <<= (count & 31);
    q->tail_.pos = (uint16_t)(count + pos);
    bitCount = (unsigned)count;
    acc |= tmp;
    q->tail_.acc = acc;
  }
crcFeed: /* feed the freshly loaded bits into the CRC, bit count-1 first */
  if (q->tail_.crcOn == 0)
    goto reloadReg;
  if (count == 0)
    goto reloadReg;
  bitReg = q->tail_.acc;
  do
  {
    unsigned crc = q->tail_.crc;
    unsigned feedback;
    count = (int16_t)(count - 1);
    tmp = bitReg >> (count & 31);
    feedback = ((int16_t)crc < 0) ? 1u : 0u;
    crc = crc + crc;
    if (tmp & 1u)
      feedback ^= 1u;
    if (feedback != 0)
      crc ^= 0x1021u;
    q->tail_.crc = (uint16_t)crc;
  } while (count != 0);
  goto emitBit;
reloadReg:
  bitReg = q->tail_.acc;
emitBit: /* shift one bit out of the bit register */
  acc = bitCount - 1u;
  count = (int16_t)acc;
  q->tail_.nbits = (uint16_t)acc;
  return (short)((bitReg >> (count & 31)) & 1u);
loadFullChunk: /* a whole chunk of `chunk` bits */
  {
    int chunk = (int16_t)bitReg;
    unsigned wordIdxRaw;
    int wordIdx;
    q->tail_.nbits = (uint16_t)bitReg;
    acc = q->tail_.acc;
    acc <<= (chunk & 31);
    wordIdxRaw = q->tail_.widx;
    wordIdx = (int16_t)wordIdxRaw;
    tmp = ld16(q->unk00_, 2 * (unsigned)wordIdx);
    q->tail_.widx = (uint16_t)(wordIdxRaw + 1u);
    count = chunk;
    q->tail_.pos = (uint16_t)(pos + bitReg);
    acc |= tmp;
    bitCount = bitReg;
    q->tail_.acc = acc;
  }
  goto crcFeed;
messageExhausted: /* no message bits left: CRC bits, the 0xf tail, or restart */
  if (count != 0)
  {
    if ((uint16_t)count == 0xfff0u)
    {
      q->tail_.acc = 0xf;
      q->tail_.nbits = 4;
      q->tail_.pos = (uint16_t)(pos + 4u);
      bitCount = 4;
      bitReg = 0xf;
      goto emitBit;
    }
    goto restartMessage;
  }
  if (q->tail_.crcOn != 0)
  {
    unsigned crc = q->tail_.crc;
    q->tail_.nbits = 0x10;
    q->tail_.pos = (uint16_t)(pos + 0x10u);
    q->tail_.acc = crc;
    bitCount = 0x10;
    bitReg = crc;
    goto emitBit;
  }
restartMessage: /* armed queue: begin the message again, otherwise report end of data */
  if (q->tail_.loop == 0)
    return (short)-1;
  {
    unsigned restarts = q->tail_.pass;
    unsigned restartBits = q->tail_.restartBits;
    q->tail_.crc = USHRT_MAX;
    q->tail_.pos = 0;
    q->tail_.pass = (uint16_t)(restarts + 1u);
    q->tail_.widx = 0;
    q->tail_.acc = q->tail_.restartAcc;
    q->tail_.nbits = (uint16_t)restartBits;
    return v04_getbit_impl(q);
  }
}

/**
 * @brief Convert a round-trip-delay estimate into the bulk delay line and adjust the far echo canceller and DMA delay for short delays.
 * @param obj    V.34 object; `bulkWr_`, `bulkRd_`, the delay line `unk35b8_` (cleared), `echoOn_` and `dmaDelay_` are written
 * @param delay  delay estimate in samples (16-bit signed); a value <= 0 is replaced by 0x90, and a value >= `bulkLen_` by 0
 *
 * With neither PCM nor K56flex mode and the far echo canceller enabled, a delay of 0x1e or more keeps it on; a smaller one turns it off ("masking Far EC")
 * and reduces `dmaDelay_` by min(delay + 0xf, 0x1e).  In PCM / K56flex mode the far echo canceller is switched off.  Debug prints at level > 1.
 * @warning The cleared length is 2 * delay bytes, bounded by `bulkLen_` and not by the size of the delay line (19200 bytes).
 */
extern "C" void v04_apply_bulk_delay_impl(tagV34Object* obj, int delay)
{
  int clamped = (int16_t)delay;
  unsigned level;
  if (clamped <= 0)
  {
    if (dsplibs_debug_level > 1u)
      dsplibs_debug_printf("V34 bulk delay first estimation %d\n", clamped);
    clamped = 0x90;
  }
  if ((uint32_t)clamped >= obj->bulkLen_)
  {
    if (dsplibs_debug_level > 1u)
      dsplibs_debug_printf("V34 bulk delay first estimation %d\n", clamped);
    clamped = 0;
  }
  obj->bulkWr_ = (uint32_t)clamped;
  obj->bulkRd_ = 0;
  sysdep_memset(obj->unk35b8_, 0, (unsigned)clamped + (unsigned)clamped);
  if (obj->pcmMode_ == 0 && obj->k56Mode_ == 0)
  {
    if (obj->echoOn_ == 0)
      goto farEchoOff;
    if ((int16_t)clamped > 0x1d)
    {
      obj->echoOn_ = 1;
      goto readDebugLevel;
    }
    level = dsplibs_debug_level;
    if (level > 1u)
    {
      dsplibs_debug_printf("RTD (%d) lower than min (%d), masking Far EC...\r\n", clamped, 0x1e);
      level = dsplibs_debug_level;
    }
    {
      int dmaCut = (int16_t)(clamped + 0xf);
      obj->echoOn_ = 0;
      if ((int16_t)dmaCut > 0x1e)
        dmaCut = 0x1e;
      if (level > 1u)
      {
        int cur = (int16_t)obj->dmaDelay_;
        dsplibs_debug_printf("...Modifying dma delay from %d to %d\r\n", cur, cur - dmaCut);
        level = dsplibs_debug_level;
      }
      obj->dmaDelay_ = (uint16_t)((unsigned)obj->dmaDelay_ - (unsigned)dmaCut);
    }
    goto printEstimate;
  }
farEchoOff:
  obj->echoOn_ = 0;
readDebugLevel:
  level = dsplibs_debug_level;
printEstimate:
  if (level > 1u)
    dsplibs_debug_printf("V34 bulk delay estimation %d (FAR=%d)\n", clamped, (int)(int16_t)obj->echoOn_);
}
