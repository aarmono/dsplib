/**
 * @file v8.c
 * @brief The ITU-T V.8 handshake engine: the DSP primitives, the message coder, the handshake state machine and the engine object API.
 *
 * V.8 is the start-up procedure two modems use to agree on a modulation and a protocol.  The answerer sends the tone ANSam (2100 Hz, 15 Hz amplitude
 * modulation, a phase reversal every 450 ms); the caller answers with CM, a call menu sent as V.21 FSK (low channel, 300 bit/s), the answerer replies
 * with JM and the caller ends with CJ (three zero octets).  With quick connect the caller sends a QC preamble and the answerer a QCA1 word instead.
 * The engine runs at 9600 Hz on blocks of 4 samples.
 *
 * Layers of this file, bottom to top:
 * - Primitives (v8_absfn, v8_mpyint, charFlip, v8_copycoeff): fixed-point helpers (Q14 products, 16-bit negation, octet bit reversal).
 * - The message coder: V8SetMessage() / V8GetMessage() and v8_getbit() code a message as 10-bit words (start bit 0, eight data bits LSB first, stop
 *   bit 1) in one of the five V8Msg slots of the engine object.  v8_getbit() serves the transmit side one bit at a time and restarts the message
 *   when it is used up.
 * - The sample rings: v8_rxinit / v8_txinit / v8_rxreadqueue / v8_txwritequeue move blocks of 4 samples between the rings that V8Process() fills and
 *   drains and the block buffers rxBuf / txBuf that the state machine works on.
 * - Receive processing: V8agc() (band-pass filter, level measurement, automatic gain control), v8_tone_detect() (two 2100 Hz resonators and a hit
 *   counter: the ANSam / CI detector), checkSignalStability(), the single-bin DFT (v8_dftupdate / v8_dftenergy: measures the 15 Hz modulation of ANSam)
 *   and the phase-reversal detector (v8_phase_rev_init / v8_phase_rev_detect).
 * - Transmit generators: v8_ansaminit / v8_ansamgenerate (ANSam), v8_TONEq_init / v8_TONEq_generate (a plain 980 Hz tone), the V.21 FSK modulator
 *   (v8_V21_Init, v8_fskmodulate, v8_fsktxfilter) and demodulator (v8_fskdemodulate, a four-correlator bank over mark and space).
 * - The state machine: v8handshakinit() sets the object up for the caller or the answerer, v8handshak() runs one block, initTxSequence() builds the
 *   outgoing CM / JM from the capability block, rebuildJMSequence() (answerer) and evaluateRxJMSequence() (caller) match the received menu against the
 *   local capabilities.
 * - The object API: V8Create(), V8Delete(), V8Process() (the per-sample engine call; maps the state words to a V8Status), V8Control() and
 *   V8UpdateModemParameters() (writes the negotiated result into the shared DpRuntime capability block).  dp_v8.c adapts this API to the modem.
 *
 * The engine state is one flat object, V8Obj (v8_obj.h).  Several state words are read in more than one view (the phase pair, the flag bytes of
 * the capability block); those use the MEMBER_AS accessors of core/dsp_access.h.  The tables are in v8_tables.c.
 *
 * Behaviour callers must know: v8handshak() calls abort() for an impossible phase word (docs/bugs.md); V8Create() does not clear the object and
 * v8_rxinit() leaves the AGC filter delay line untouched, so its first outputs depend on the heap contents (see v8_rxinit()).
 */
#include <limits.h>
#include <stdlib.h>

#include "v8.h"
#include "v8_tables.h"
#include "v8_obj.h"
#include "core/dp_params.h"
#include "core/dsp_host.h"

/* ---- small shared math / bit helpers ---- */

/**
 * @brief Absolute value of a 16-bit sample.
 * @return |value|; v8_absfn(-32768) is -32768 (the negation is done in 16 bits)
 */
int v8_absfn(short value)
{
  if (-1 < value)
  {
    return (int)value;
  }
  /* the negation wraps in 16 bits; C's unary minus on a short promotes to int first, so the explicit (short) is what gives the wrap */
  return (int)(short)-(int)value;
}

/** @brief Q14 product: (factorA * factorB) >> 14, truncated to 16 bits. */
int v8_mpyint(short factorA, short factorB) { return (int)(short)(((int)factorA * (int)factorB) >> 0xe); }

/**
 * @brief Reverse the bit order of an octet (two 4-bit lookups in v8_wordFlip).
 * @note  Converts between the LSB-first order of the V.8 line format and the natural byte order when message octets are packed and unpacked.
 */
unsigned char charFlip(unsigned char octet) { return (unsigned char)((v8_wordFlip[octet & 0xf] << 4) | v8_wordFlip[octet >> 4]); }

/** @brief Copy `count` 16-bit coefficients. */
void v8_copycoeff(short* dst, const short* src, short count)
{
  short i;
  for (i = 0; i < count; i = i + 1)
  {
    dst[i] = src[i];
  }
}

/* ---- message encode / decode ---- */

/**
 * @brief Deliver the next transmit bit of a message slot.
 * @param p  a message slot (V8Msg) filled by V8SetMessage() or initTxSequence()
 * @return the next bit (0 or 1), or UINT_MAX when the message is used up and `repeatEnable` is 0
 * @note  The words of the slot are shifted out MSB first, one 10-bit word (`wordBits`) at a time.  With `crcEnable` set, a CRC-CCITT (polynomial
 *        0x1021) is accumulated over the data bits, appended after the last word as 16 bits and followed by four 1 bits; no slot of the engine
 *        enables it.  When the message has been sent and `repeatEnable` is set, the slot restarts (the CRC accumulator is reset and `passCount` is
 *        incremented) and the function calls itself once to deliver the first bit of the next pass.
 */
unsigned int v8_getbit(V8Msg* p)
{
  short bitPos;
  short bitsLeftInMsg;
  unsigned short word;
  short bitsInWord;
  unsigned int shiftWord;
  short chunkBits;
  unsigned int crcFeedback;

  bitsInWord = p->bitsInWord;
  if (bitsInWord == 0)
  {
    bitPos = p->bitsSent;
    bitsLeftInMsg = p->totalBits - bitPos;
    if (bitsLeftInMsg < 1)
    {
      if (bitsLeftInMsg == 0)
      {
        if (p->crcEnable == 0)
        {
          goto reload;
        }
        shiftWord = (unsigned int)p->crcAcc;
        bitsInWord = 0x10;
        p->bitsInWord = 0x10;
        p->bitsSent = bitPos + 0x10;
        p->shiftWord = shiftWord;
      }
      else
      {
        if (bitsLeftInMsg != -0x10)
        {
          goto reload;
        }
        p->shiftWord = 0xf;
        bitsInWord = 4;
        p->bitsInWord = 4;
        shiftWord = 0xf;
        p->bitsSent = bitPos + 4;
      }
      goto shift_out;
    }
    chunkBits = p->wordBits;
    if (bitsLeftInMsg < chunkBits)
    {
      chunkBits = bitsLeftInMsg;
      if (0 < bitsLeftInMsg)
      {
        p->bitsInWord = bitsLeftInMsg;
        word = MEMBER_AS_AT(unsigned short, p, data[0], p->wordIdx * 2);
        p->bitsSent = bitsLeftInMsg + bitPos;
        p->shiftWord = (MEMBER_AS(int, p, shiftWord) << ((unsigned char)bitsLeftInMsg & 0x1f)) | (unsigned int)word;
        bitsInWord = bitsLeftInMsg;
      }
    }
    else
    {
      p->bitsInWord = chunkBits;
      word = MEMBER_AS_AT(unsigned short, p, data[0], p->wordIdx * 2);
      p->wordIdx = p->wordIdx + 1;
      p->bitsSent = bitPos + chunkBits;
      p->shiftWord = (MEMBER_AS(int, p, shiftWord) << ((unsigned char)chunkBits & 0x1f)) | (unsigned int)word;
      bitsInWord = chunkBits;
    }
    if ((p->crcEnable != 0) && (chunkBits != 0))
    {
      shiftWord = p->shiftWord;
      do
      {
        chunkBits = chunkBits - 1;
        crcFeedback = (unsigned int)((int)MEMBER_AS(short, p, crcAcc) >> 0x1f);
        word = (unsigned short)(MEMBER_AS(short, p, crcAcc) * 2);
        if (((shiftWord >> ((unsigned char)chunkBits & 0x1f)) & 1) != 0)
        {
          crcFeedback = crcFeedback ^ 1;
        }
        if (crcFeedback != 0)
        {
          word = word ^ 0x1021;
        }
        p->crcAcc = word;
      } while (chunkBits != 0);
      goto shift_out;
    }
  }
  shiftWord = p->shiftWord;
  goto shift_out;

reload:
  if (p->repeatEnable == 0)
  {
    return UINT_MAX;
  }
  p->crcAcc = USHRT_MAX;
  p->bitsSent = 0;
  p->passCount = p->passCount + 1;
  p->wordIdx = 0;
  p->shiftWord = p->reloadWord;
  p->bitsInWord = p->reloadBits;
  return v8_getbit(p);

shift_out:
  p->bitsInWord = bitsInWord - 1;
  return (shiftWord >> ((unsigned char)(bitsInWord - 1) & 0x1f)) & 1;
}

/**
 * @brief Store a message in one of the message slots and reset the slot's read cursor.
 * @param p        the engine object
 * @param msgType  0 = CM (slot 0), 1 = JM (slot 2), 2 = CJ (slot 1), 3 = quick connect (slot 3); anything else fails
 * @param octets   message octets
 * @param length   number of octets; at most 15, longer messages are cut to 15 (and 0xf is returned); 0 fails
 * @return 0, 0xf when the message was truncated, or UINT_MAX for a bad type or length (a message is printed at debug level 2)
 * @note  Each octet is coded as the 10-bit word `(charFlip(octet) << 1) | 1`: start bit 0, eight data bits LSB first, stop bit 1.  The slot is
 *        set to repeat the message indefinitely, without CRC.
 */
unsigned int V8SetMessage(V8Obj* p, int msgType, const void* octets, int length)
{
  V8Msg* slot;
  const unsigned char* src = (const unsigned char*)octets;
  int i;
  unsigned short encoded;
  unsigned int truncated;

  if (msgType == 1)
  {
    slot = V8_MSG(p, 2);
  }
  else if (msgType == 0)
  {
    slot = V8_MSG(p, 0);
  }
  else if (msgType == 2)
  {
    slot = V8_MSG(p, 1);
  }
  else
  {
    slot = V8_MSG(p, 3);
    if (msgType != 3)
    {
      if (1 < dsplibs_debug_level)
      {
        dsplibs_debug_printf("V8: Try to set message with illegal V.8 message type\n");
        return UINT_MAX;
      }
      return UINT_MAX;
    }
  }
  if (length == 0)
  {
    if (1 < dsplibs_debug_level)
    {
      dsplibs_debug_printf("V8: Try to set message with 0 length\n");
      return UINT_MAX;
    }
    return UINT_MAX;
  }
  truncated = 0;
  if (0xf < length)
  {
    length = 0xf;
    truncated = 0xf;
  }
  for (i = 0; i < length; i = i + 1)
  {
    encoded = charFlip(src[i]);
    slot->data[i] = (unsigned short)((encoded & 0xff) * 2 | 1);
  }
  slot->crcAcc = USHRT_MAX;
  slot->bitsSent = 0;
  slot->wordIdx = 0;
  slot->passCount = 0;
  slot->totalBits = (short)length * 10;
  slot->wordBits = 10;
  slot->crcEnable = 0;
  slot->shiftWord = 0;
  slot->reloadWord = 0;
  slot->bitsInWord = 0;
  slot->reloadBits = 0;
  slot->repeatEnable = 1;
  return truncated;
}

/**
 * @brief Read back the message received from the remote side.
 * @param p          the engine object
 * @param octetsOut  receives the octets
 * @param lengthIO   in: capacity of `octetsOut`; out: octets copied
 * @return 0 on success, the full message length when it did not fit (`*lengthIO` octets are copied), or UINT_MAX when no message has been received
 * @note  The source is the received JM (caller), the received CM (answerer), or, once quick connect is detected, the QCA1 word buffer.
 */
unsigned int V8GetMessage(V8Obj* p, void* octetsOut, int* lengthIO)
{
  V8Msg* slot;
  unsigned char* dst = (unsigned char*)octetsOut;
  int count;
  int i;
  unsigned int result = UINT_MAX;

  if (MEMBER_AS(int, p, qcDetected) == 0)
  {
    if (p->side == 0)
    {
      slot = V8_MSG(p, 2);
    }
    else
    {
      slot = V8_MSG(p, 0);
    }
  }
  else
  {
    slot = V8_PGM(V8Msg*, V8Obj, p, qcRxMsgPtr);
  }
  if (0 < slot->wordIdx)
  {
    count = (int)slot->wordIdx;
    result = 0;
    if (*lengthIO < count)
    {
      result = count;
      count = *lengthIO;
    }
    for (i = 0; i < count; i = i + 1)
    {
      dst[i] = charFlip((unsigned char)((MEMBER_AS_AT(unsigned short, slot, data[0], i * 2) >> 1) & 0xff));
    }
    *lengthIO = count;
  }
  return result;
}

/* ---- receive / transmit ring plumbing ---- */

/**
 * @brief Reset the receive side: AGC state, level history, receive block pointer.
 * @return 0
 * @warning The AGC filter delay line `agcDelay` (V8agc()) is not cleared here or anywhere else, and V8Create() does not clear the object: the first
 *          outputs of V8agc() mix in whatever the allocator returned (found by reading; not run).
 */
unsigned int v8_rxinit(V8Obj* p)
{
  int i;

  for (i = 0; i < 0xa0; i = i + 1)
  {
    p->unk894[i] = 0;
  }
  p->unk894[40] = 0x1000;

  /* AGC starts at gain 0x200 with a fast adaptation step */
  p->refGain = 0x200;
  p->rxHistPos = 0;
  p->agcGain = 0x200;
  p->gainAdaptStep = 0x3333;
  for (i = 0; i < 0x30; i = i + 1)
  {
    p->rxHist[i] = 0;
  }
  p->gainErrAcc = 0;
  p->refAge = 0;
  p->stableCount = 0;
  p->gainStable = 0;
  p->w0de = 0x50;
  p->w0e4 = 0;
  p->w0e2 = 0;
  p->w0f6 = 0;
  p->w0f4 = 0;
  V8_PSM(char*, V8Obj, p, rxEndPtr, MEMBER_ADDR(p, rxBuf[0]));
  p->rxLevel = 0;
  p->blockEnergy = 0;
  p->overflowCount = 0;
  return 0;
}

/**
 * @brief Reset the transmit side and the rings: both rings empty and zeroed, the transmit ring primed with 32 samples of silence.
 * @return 0
 */
unsigned int v8_txinit(V8Obj* p)
{
  int i;

  p->w014 = 1;
  p->w00c = 0;
  p->w018 = 0;
  p->d004 = 0;
  for (i = 0; i < 0x8c; i = i + 1)
  {
    p->unk77c[i] = 0;
  }
  V8_PSM(char*, V8Obj, p, txRingRdPtr, MEMBER_ADDR(p, txRing[0]));
  for (i = 0; i < 0x1cc; i = i + 1)
  {
    p->txRing[i] = 0;
  }
  p->txQueueCount = 0x20;
  V8_PSM(char*, V8Obj, p, txRingWrPtr, MEMBER_ADDR(p, txRing[64]));
  V8_PSM(char*, V8Obj, p, rxRingRdPtr, MEMBER_ADDR(p, rxRing[0]));
  V8_PSM(char*, V8Obj, p, rxRingWrPtr, MEMBER_ADDR(p, rxRing[0]));
  p->rxQueueCount = 0;
  for (i = 0; i < 0x80; i = i + 1)
  {
    p->rxRing[i] = 0;
  }
  return 0;
}

/**
 * @brief Move 4 samples from the receive ring to the block buffer rxBuf.
 * @return 0
 * @note  The receive ring holds each sample in every second 16-bit word (V8Process() writes the sample and a zero); the read cursor wraps at the end
 *        of the ring.  V8agc() performs the same read inline.
 */
unsigned int v8_rxreadqueue(V8Obj* p)
{
  short i;
  unsigned short* dst = (unsigned short*)(MEMBER_ADDR(p, rxBuf[0]));
  unsigned short* src = V8_PGM(unsigned short*, V8Obj, p, rxRingRdPtr);

  p->rxQueueCount = p->rxQueueCount - 4;
  for (i = 0; i < 4; i = i + 1)
  {
    *dst = *src;
    src = src + 2;
    dst = dst + 1;
    if ((char*)src >= MEMBER_ADDR(p, txQueueCount))
    {
      src = (unsigned short*)(MEMBER_ADDR(p, rxRing[0]));
    }
  }
  V8_PSM(unsigned short*, V8Obj, p, rxRingRdPtr, src);
  return 0;
}

/**
 * @brief Append the 4 samples of the block buffer txBuf to the transmit ring.
 * @return 0
 */
unsigned int v8_txwritequeue(V8Obj* p)
{
  short i;
  unsigned short* dst = V8_PGM(unsigned short*, V8Obj, p, txRingWrPtr);
  unsigned short* src = (unsigned short*)(MEMBER_ADDR(p, txBuf[0]));

  MEMBER_AS(short, p, txQueueCount) = MEMBER_AS(short, p, txQueueCount) + 4;
  for (i = 0; i < 4; i = i + 1)
  {
    *dst = *src;
    src = src + 1;
    dst = dst + 1;
    if ((char*)dst >= MEMBER_ADDR(p, txBuf[0]))
    {
      dst = (unsigned short*)(MEMBER_ADDR(p, txRing[0]));
    }
  }
  V8_PSM(unsigned short*, V8Obj, p, txRingWrPtr, dst);
  return 0;
}

/* ---- ANSam tone detection, gain stability check, AGC ---- */

/**
 * @brief Track whether the AGC gain has settled.
 * @note  Every call (4 samples) compares the gain with a reference snapshot taken every 240 calls: while the relative change stays within 5%,
 *        `stableCount` grows by 4 and `gainStable` is set once it exceeds 959; a larger change clears both.
 */
void checkSignalStability(V8Obj* p)
{
  short relChange;

  relChange = p->refAge + 4;
  if (relChange < 0x3c0)
  {
    p->refAge = relChange;
    relChange = (short)((((int)MEMBER_AS(short, p, agcGain) - (int)p->refGain) * 0x4000) / (int)p->refGain);
  }
  else
  {
    relChange = MEMBER_AS(short, p, agcGain);
    p->refAge = 0;
    p->refGain = relChange;
    relChange = (short)((((int)relChange - (int)relChange) * 0x4000) / (int)relChange);
  }
  if (relChange < 0)
  {
    relChange = -relChange;
  }
  if (0x333 < relChange)
  {
    p->stableCount = 0;
    p->gainStable = 0;
    return;
  }
  relChange = p->stableCount + 4;
  p->stableCount = relChange;
  if (0x3bf < relChange)
  {
    p->gainStable = 1;
  }
}

/**
 * @brief Receive-side conditioning of one block of 4 samples: read, band-pass filter, measure level, apply and adapt the gain.
 * @param p  the engine object
 * @return 0
 * @note  Called by the rx phases of v8handshak().  Steps: (1) read 4 samples from the receive ring into rxBuf; (2) filter them with the 40-tap
 *        entrance band-pass of the side (CALLER_Entrance_Filter or ANSWER_Entrance_Filter); (3) keep the last 36 samples in rxHist and compute their
 *        RMS level through v8_sqrt_table; (4) multiply the block by `agcGain` (>> 10), saturating at +-0x7f00; ten overflows reset the gain to 0x400
 *        and, while an ANSam candidate is being watched, restart the ANSam search; (5) when the level exceeds 0x1f, update the smoothed block
 *        energy `rxLevel` (target 4000) and, unless adaptation is held (flags bit 0x200), integrate the level error and step the gain by about
 *        +12% or -11% when the accumulated error leaves +-1000.
 */
unsigned int V8agc(V8Obj* p)
{
  int i, j;
  short rmsLevel, sample;
  short *block, *coefOrCursor, *taps, *blockPos;
  unsigned short *dst, *src;
  unsigned short normShift, mantissa;
  unsigned int energy;
  unsigned int idxOrHi;
  int acc;

  /* (1) read the block from the receive ring */
  dst = (unsigned short*)(MEMBER_ADDR(p, rxBuf[0]));
  src = V8_PGM(unsigned short*, V8Obj, p, rxRingRdPtr);
  p->rxQueueCount = p->rxQueueCount - 4;
  for (i = 0; i < 4; i = i + 1)
  {
    *dst = *src;
    src = src + 2;
    dst = dst + 1;
    if ((char*)src >= MEMBER_ADDR(p, txQueueCount))
    {
      src = (unsigned short*)(MEMBER_ADDR(p, rxRing[0]));
    }
  }
  V8_PSM(unsigned short*, V8Obj, p, rxRingRdPtr, src);

  /* (2) entrance band-pass filter (40 taps), in place */
  block = (short*)(MEMBER_ADDR(p, rxBuf[0]));
  V8_PSM(char*, V8Obj, p, rxEndPtr, MEMBER_ADDR(p, rxBuf[4]));
  coefOrCursor = (p->side == 0) ? (short*)CALLER_Entrance_Filter : (short*)ANSWER_Entrance_Filter;
  blockPos = block;
  for (i = 0; i < 4; i = i + 1)
  {
    acc = 0;
    p->agcDelay[0] = *blockPos;
    taps = coefOrCursor;
    for (j = 0; j < 0x28; j = j + 1)
    {
      acc = acc + (int)taps[j] * (int)p->agcDelay[j];
    }
    *blockPos = (short)(acc >> 0xe);
    blockPos = blockPos + 1;
    {
      unsigned short* shiftp = (unsigned short*)(MEMBER_ADDR(p, agcDelay[39]));
      for (j = 0; j < 0x27; j = j + 1)
      {
        *shiftp = shiftp[-1];
        shiftp = shiftp - 1;
      }
    }
  }

  /* (3) level: mean square of the last 36 filtered samples, normalised, square root by table */
  coefOrCursor = block;
  for (i = 0; i < 4; i = i + 1)
  {
    short idx = p->rxHistPos;
    p->rxHistPos = idx + 1;
    MEMBER_AS_AT(short, p, rxHist[0], idx * 2) = *coefOrCursor;
    coefOrCursor = coefOrCursor + 1;
    if (0x23 < idx + 1)
    {
      p->rxHistPos = 0;
    }
  }

  energy = 0;
  coefOrCursor = (short*)(MEMBER_ADDR(p, rxHist[0]));
  for (i = 0x23; i >= 0; i = i - 1)
  {
    sample = *coefOrCursor;
    coefOrCursor = coefOrCursor + 1;
    energy = energy + (unsigned int)((sample * 0x38e) >> 0xf) * (int)sample;
  }
  rmsLevel = 0;
  if (energy != 0)
  {
    normShift = 0;
    while (energy < 0x20000000)
    {
      energy = energy * 2;
      normShift = normShift + 1;
    }
    mantissa = (unsigned short)(energy >> 0xf);
    if (normShift != (unsigned short)((normShift >> 1) * 2))
    {
      mantissa = mantissa >> 1;
    }
    idxOrHi = (unsigned int)(((int)(mantissa + 0x40) >> 7) - 0x40);
    if ((unsigned short)idxOrHi > 0xbf)
    {
      idxOrHi = 0xbf;
    }
    rmsLevel = (short)((int)(unsigned int)(unsigned short)v8_sqrt_table[idxOrHi] >> ((unsigned char)(normShift >> 1) & 0x1f));
  }

  /* (4) apply the gain; saturate and count overflows */
  coefOrCursor = block;
  for (i = 0; i < 4; i = i + 1)
  {
    unsigned int scaled = (unsigned int)((int)MEMBER_AS(short, p, agcGain) * (int)*coefOrCursor);
    unsigned short scaledHi = (unsigned short)(scaled >> 0x10);
    if (scaledHi >> 9 == 0x7f || (scaled >> 0x19) == 0)
    {
      *coefOrCursor = (short)((int)scaled >> 10);
    }
    else
    {
      if ((short)scaledHi < 1)
      {
        *coefOrCursor = -0x7f00;
      }
      else
      {
        *coefOrCursor = 0x7f00;
      }
      if (1 < dsplibs_debug_level)
      {
        dsplibs_debug_printf("V8AGC, overflow = 0x%x,\n", (int)scaled >> 0x10);
      }
      {
        short cnt = p->overflowCount + 1;
        p->overflowCount = cnt;
        if (cnt == 10)
        {
          p->agcGain = 0x400;
          if (MEMBER_AS(short, p, rxSubState) == V8SUB_ANSAM_SEEN)
          {
            p->rxSubState = V8SUB_ANSAM_WAIT;
            if (1 < dsplibs_debug_level)
            {
              dsplibs_debug_printf("V8: Due to overflow, looking for ANSam again...\n");
            }
            p->det.hitCount = 0;
            p->det.armCount = 0;
            p->overflowCount = 0;
          }
        }
      }
    }
    coefOrCursor = coefOrCursor + 1;
  }

  /* (5) smoothed output energy and gain adaptation, only for a signal above the noise floor */
  if (rmsLevel > 0x1f)
  {
    energy = 0;
    for (i = 0; i < 4; i = i + 1)
    {
      sample = block[i];
      energy = energy + (unsigned int)((int)sample * (int)sample);
    }
    p->blockEnergy = energy;
    energy = (unsigned int)((p->rxLevel * 0x6ccd) >> 0xf) + (energy >> 0x10);
    idxOrHi = energy >> 0xf;
    if (idxOrHi == 0x1ffff || idxOrHi == 0)
    {
      p->rxLevel = (short)energy;
    }
    else
    {
      p->rxLevel = 0x7f00;
    }
    if ((MEMBER_AS_AT(unsigned char, p, flags, 1) & 2) == 0)
    {
      short levelDev = p->rxLevel - 4000;
      int levelDevSigned = (int)levelDev;
      if (levelDev < 0)
      {
        levelDev = -levelDev;
      }
      if (0 < (short)(levelDev - 2000))
      {
        short gainErr = (short)((unsigned int)(p->gainAdaptStep * levelDevSigned) >> 0x10) + p->gainErrAcc;
        short gainErrAbs = gainErr;
        if (gainErr < 0)
        {
          gainErrAbs = -gainErr;
        }
        if ((short)(gainErrAbs - 1000) < 1)
        {
          p->gainErrAcc = gainErr;
        }
        else
        {
          p->gainErrAcc = 0;
          if (gainErr < 1)
          {
            if (MEMBER_AS(short, p, agcGain) < 0x6a01)
            {
              MEMBER_AS(short, p, agcGain) = (short)((MEMBER_AS(short, p, agcGain) * 0x47cf) >> 0xe);
            }
          }
          else
          {
            MEMBER_AS(short, p, agcGain) = (short)((MEMBER_AS(short, p, agcGain) * 0x390a) >> 0xe);
          }
        }
      }
    }
  }
  return 0;
}

/**
 * @brief Initialise the ANSam tone detector sub-object used by v8_tone_detect().
 * @param obj           the engine object (its flags bit 0x200, "hold AGC adaptation", is set until the detector arms)
 * @param p             the detector (V8Det)
 * @param coefTable     the resonator coefficients: 8 shorts, [0..3] feed-forward (all 0 in v8_c2100) and [4..7] feed-back, two sections of two
 * @param mode          0 = a hit is a level above `threshold`; otherwise a hit is a level below `hiThreshold`
 * @param hitTarget     the tone is reported when the hit counter exceeds this
 * @param hitHeadStart  the hit counter starts at -hitHeadStart
 * @param threshold     level threshold of a hit (mode 0)
 * @param hiThreshold   level threshold of a hit (other modes)
 */
void v8_detectorinit(V8Obj* obj, V8Det* p, const void* coefTable, short mode, short hitTarget, short hitHeadStart, short threshold, short hiThreshold)
{
  int i, j;

  for (i = 0; i < 2; i = i + 1)
  {
    for (j = 0; j < 2; j = j + 1)
    {
      int idx = i * 2 + j;
      p->iirA[idx] = 0;
      p->iirB[idx] = 0;
    }
  }
  for (i = 0; i < 3; i = i + 1)
  {
    p->num[i] = 0;
    p->den[i] = 0;
  }
  MEMBER_AS(short, p, mode) = mode;
  p->hitCount = -hitHeadStart;
  p->w00c = 1;
  CFGP_SLOT_PUT(const void*, p, coefTable);
  p->armed = 0;
  p->threshold = threshold;
  p->hitTarget = hitTarget;
  p->hiThreshold = hiThreshold;
  p->level = 0;
  p->armCount = 0;
  obj->flags = obj->flags | 0x200;
}

/**
 * @brief Run the ANSam tone detector over a block and decide.
 * @param obj      the engine object (its `rxEndPtr` marks the end of the block; flags bit 0x200 is cleared when the detector arms)
 * @param p        the detector initialised by v8_detectorinit()
 * @param samples  the block, filtered in place up to `rxEndPtr`
 * @return 1 when the tone has been detected (the hit counter exceeds its target), else 0
 * @note  Each sample passes a 2-pole IIR (v8_tone_b / v8_tone_a) and two cascaded resonator sections tuned to 2100 Hz (coefficients of
 *        v8_detectorinit()); the absolute output is smoothed into `level` (level = |y| + 0.475 * level).  Mode 0: the detector arms after the level
 *        has stayed at 0x31 or above for 0x33 calls; once armed each call whose level exceeds the threshold adds a hit and any other call clears the
 *        hits.
 */
unsigned int v8_tone_detect(V8Obj* obj, V8Det* p, short* samples)
{
  short filtered, stage1Out;
  int i;
  short* coefs;

  while (samples < V8_PGM(short*, V8Obj, obj, rxEndPtr))
  {
    MEMBER_AS(short, p, num[0]) = *samples;

    filtered = 0;
    for (i = 0; i < 3; i = i + 1)
    {
      filtered = filtered + (short)v8_mpyint(v8_tone_b[i], MEMBER_AS_ATB(short, p, num[0], i * 2));
    }
    for (i = 0; i < 2; i = i + 1)
    {
      filtered = filtered - (short)v8_mpyint(v8_tone_a[1 + i], MEMBER_AS_ATB(short, p, den[0], i * 2));
    }
    /* shift the 3-slot numerator / denominator history windows of the input IIR */
    p->num[2] = p->num[1];
    p->den[2] = p->den[1];
    p->num[1] = p->num[0];
    p->den[1] = p->den[0];

    MEMBER_AS(short, p, den[0]) = filtered;
    *samples = filtered;
    samples = samples + 1;

    coefs = CFGP_SLOT(short*, p);
    {
      /* two resonator sections in direct form I: the feed-forward histories iirA hold the section INPUTS (the sample >> 4, and stage 1's output >> 4),
       * the feed-back histories iirB the section OUTPUTS.  The shipped coefficient table has zero feed-forward coefficients, so only the feed-back
       * terms reach the output. */
      int xin = (int)filtered >> 4;
      int acc1 = xin;
      short newh1c, newh10;
      for (i = 0; i < 2; i = i + 1)
      {
        acc1 = acc1 + (short)v8_mpyint(p->iirA[i], coefs[i]) - (short)v8_mpyint(p->iirB[i], coefs[4 + i]);
      }
      newh10 = p->iirA[0];
      newh1c = p->iirB[0];
      p->iirB[0] = (short)acc1;
      p->iirA[0] = (short)xin;
      p->iirA[1] = newh10;
      p->iirB[1] = newh1c;
      stage1Out = (short)(acc1 >> 4);
    }
    {
      int acc2 = (int)stage1Out;
      short newh18, newh20;
      for (i = 0; i < 2; i = i + 1)
      {
        acc2 = acc2 + (short)v8_mpyint(p->iirA[2 + i], coefs[2 + i]) - (short)v8_mpyint(p->iirB[2 + i], coefs[6 + i]);
      }
      newh18 = p->iirA[2];
      newh20 = p->iirB[2];
      p->iirA[2] = stage1Out; /* the stage-2 INPUT (stage 1's output >> 4), not its output */
      p->iirA[3] = newh18;
      p->iirB[2] = (short)acc2;
      p->iirB[3] = newh20;
      filtered = v8_absfn((int)((short)acc2) >> 4);
    }
    p->level = filtered + (short)v8_mpyint(p->level, 0x3ccd);
  }

  if ((short)p->mode == 0)
  {
    if (p->armed == 0)
    {
      if (p->level < 0x31)
      {
        p->armCount = 0;
        return 0;
      }
      {
        short cnt = p->armCount + 1;
        p->armCount = cnt;
        if (cnt != 0x33)
        {
          return 0;
        }
        p->armed = 1;
        obj->flags = obj->flags & 0xfdff;
        p->hitCount = 0;
        return 0;
      }
    }
    if (p->threshold < p->level)
    {
      p->hitCount = p->hitCount + 1;
      goto check_count;
    }
  }
  else
  {
    if (p->level < p->hiThreshold)
    {
      p->hitCount = p->hitCount + 1;
    }
    if (p->level <= p->threshold)
    {
      goto check_count;
    }
  }
  p->hitCount = 0;
check_count:
  if (p->hitCount <= p->hitTarget)
  {
    return 0;
  }
  return 1;
}

/**
 * @brief Accumulate a single-frequency DFT (cosine / sine correlation) of a block of samples.
 * @param p            the first DFT bin (an array of V8Dft)
 * @param binCount     number of bins
 * @param samples      input samples
 * @param sampleCount  number of samples
 * @note  Per sample and bin the phase advances by `step` (14-bit phase, step 0x1a is 15 Hz at 9600 Hz) and the sample times the cosine and sine of
 *        the phase (v8_costbl) is added to `re` and `im`.
 */
void v8_dftupdate(V8Dft* p, short binCount, short* samples, short sampleCount)
{
  short i, j;
  V8Dft* pw;

  for (i = 0; i < sampleCount; i = i + 1)
  {
    pw = p;
    for (j = 0; j < binCount; j = j + 1)
    {
      unsigned int phaseAcc = ((unsigned int)pw->phase + (unsigned int)pw->step) & 0x3fff;
      short sample = *samples;
      pw->phase = (unsigned short)phaseAcc;
      phaseAcc = phaseAcc >> 6;
      pw->re = pw->re + (((int)v8_costbl[phaseAcc] * (int)sample) >> 6);
      pw->im = pw->im + (((int)v8_costbl[(phaseAcc + 0x40) & 0xff] * (int)sample) >> 6);
      pw = pw + 1;
    }
    samples = samples + 1;
  }
}

/**
 * @brief Convert the accumulated DFT sums to bin energies: energy = ((re << shift) >> 16)^2 + ((im << shift) >> 16)^2, >> 16.
 */
void v8_dftenergy(V8Dft* p, short binCount, unsigned char shift)
{
  short i;
  int reScaled, imScaled;

  for (i = 0; i < binCount; i = i + 1)
  {
    reScaled = (p->re << (shift & 0x1f)) >> 0x10;
    imScaled = (p->im << (shift & 0x1f)) >> 0x10;
    MEMBER_AS(short, p, energy) = (short)((unsigned int)(reScaled * reScaled + imScaled * imScaled) >> 0x10);
    p = p + 1;
  }
}

/**
 * @brief Initialise the ANSam phase-reversal detector (the `phaseRev` area of V8Obj).
 * @note  The detector keeps a 64-sample circular history; its window is 0x20 samples.  Layout of the area: words 0..4 hold the correlation, the
 *        energy, the smoothed energy, the reversal count (low half) and the history position; the half-window is at byte 0x0e, the interval
 *        counter at 0x12, the history at 0x14 and the "reversals detected" flag at 0xdc.
 */
void v8_phase_rev_init(void* pr)
{
  unsigned int* p = (unsigned int*)pr;
  int i;

  *(unsigned short*)((char*)p + 0xdc) = 0; /* the "reversals detected" flag */
  p[0] = 0;
  p[1] = 0;
  p[2] = 0;
  *(unsigned short*)((char*)p + 0x12) = 0;
  p[3] = 0;
  *(unsigned short*)((char*)p + 0xe) = 0x20;
  p[4] = 0;
  for (i = 0; i < 0x40; i = i + 1)
  {
    *(unsigned short*)((char*)p + i * 2 + 0x14) = 0;
  }
}

/**
 * @brief Feed samples to the phase-reversal detector.
 * @param pr     the detector area initialised by v8_phase_rev_init()
 * @param in     input samples
 * @param count  number of samples
 * @note  ANSam reverses its phase every 450 ms.  The detector correlates each sample with the one half a window earlier, keeps a smoothed input
 *        energy, and treats a correlation below about an eighth of that energy as a reversal.  The interval between reversals is converted to
 *        milliseconds (samples / 9.6); a reversal at an interval of 431 .. 468 ms, with two or more counted, sets the flag at byte 0xdc.
 */
void v8_phase_rev_detect(void* pr, const short* in, short count)
{
  char* p = (char*)pr;
  unsigned int* p4 = (unsigned int*)pr;
  short halfWindow, lagPos;
  int window, oldSample, newSample, energy;
  unsigned int pos;
  int energySmooth, corr, delay, n;

  delay = 0;
  energySmooth = (int)p4[2];
  pos = (unsigned int)(unsigned short)(short)p4[4];
  corr = (int)p4[0];
  energy = (int)p4[1];
  halfWindow = *(short*)(p + 0xe);
  n = 0;
  if (0 < count)
  {
    window = halfWindow * 2;
    do
    {
      unsigned int newpos = (unsigned int)(unsigned short)((short)pos + 1);
      short interval;
      newpos = ((int)newpos < window) ? newpos : 0;
      pos = newpos;
      lagPos = (short)pos - halfWindow;
      if (lagPos < 0)
      {
        lagPos = (short)(lagPos + (short)window);
      }
      newSample = (int)in[n];
      oldSample = (int)*(short*)(p + pos * 2 + 0x14);
      energy = (energy + ((newSample * newSample + 0x8000) >> 0x10)) - ((oldSample * oldSample + 0x8000) >> 0x10);
      corr = corr + (((int)(*(short*)(p + lagPos * 2 + 0x14)) * (newSample - oldSample) + 0x8000) >> 0x10);
      {
        int iVar4b = energySmooth * 0x7fe2 + energy * 0x1e;
        energySmooth = iVar4b >> 0xf;
        if (corr * 2 < iVar4b >> 0x11)
        {
          interval = *(short*)(p + 0x12);
          if ((int)interval <= (int)(*(short*)(p + 0xe) * 4))
          {
            *(short*)(p + 0x12) = interval + 1;
          }
          else
          {
            *(unsigned short*)(p + 0x12) = 0;
            delay = (interval * 0xd55) >> 0xf;
            /* word 3 packs the reversal count (low half) and the window size (high half, byte 0x0e); only the low half is written */
            *(short*)(p + 0xc) = (short)(*(short*)(p + 0xc) + 1);
          }
        }
        else
        {
          interval = *(short*)(p + 0x12);
          *(short*)(p + 0x12) = interval + 1;
        }
      }
      if (((unsigned int)(delay - 0x1af) < 0x27) && (1 < (short)p4[3]))
      {
        if (1 < dsplibs_debug_level)
        {
          dsplibs_debug_printf("ANSAM phase reversals detected delay = %d\n", delay, 0);
        }
        *(unsigned short*)((char*)p4 + 0xdc) = 1;
      }
      *(unsigned short*)(p + pos * 2 + 0x14) = (unsigned short)in[n];
      n = n + 1;
    } while (n < count);
  }
  p4[4] = (unsigned int)(unsigned short)(short)pos;
  p4[1] = (unsigned int)energy;
  p4[0] = (unsigned int)corr;
  p4[2] = (unsigned int)energySmooth;
}

/* ---- ANSam and TONEq generators ---- */

/**
 * @brief Initialise the ANSam generator: carrier step 0xe00 (2100 Hz at 9600 Hz, 14-bit phase), modulation step 0x1a (15 Hz), amplitude 16000 * the
 *        transmit gain, phase reversals on.
 */
void v8_ansaminit(V8Obj* p)
{
  p->tonePhase = 0;
  p->amStep = 0x1a;
  p->toneStep = 0xe00;
  p->reverseCount = 0;
  p->amPhase = 0;
  p->ansamAmp = (short)v8_mpyint(16000, p->txGain);
  p->reverseEnable = 1;
}

/**
 * @brief Generate 4 samples of ANSam.
 * @param p    the engine object
 * @param out  4 samples; each is the 2100 Hz carrier times (1 + 0.2 * the 15 Hz cosine) times the amplitude, passed through v8_fsktxfilter()
 * @note  The amplitude changes sign every 0x438 calls (4 samples each), i.e. every 450 ms: the phase reversal.
 */
void v8_ansamgenerate(V8Obj* p, short* out)
{
  int i;
  short sample, carrier;

  for (i = 0; i < 4; i = i + 1)
  {
    unsigned int amPhase = ((unsigned int)p->amPhase + (unsigned int)p->amStep) & 0x3fff;
    p->amPhase = (unsigned short)amPhase;
    p->tonePhase = (p->tonePhase + p->toneStep) & 0x3fff;
    sample = v8_cosread((unsigned char)((amPhase + 0x20) >> 6));
    sample = (short)v8_mpyint(0xccd, sample);
    sample = (short)v8_mpyint((short)(sample + 0x4000), p->ansamAmp);
    carrier = v8_cosread((unsigned char)((p->tonePhase + 0x20) >> 6));
    sample = (short)v8_mpyint(carrier, sample);
    out[i] = v8_fsktxfilter(p, sample);
  }
  if (p->reverseEnable != 0)
  {
    short count = p->reverseCount + 1;
    if (count == 0x438)
    {
      p->reverseCount = 0;
      p->ansamAmp = -p->ansamAmp;
    }
    else
    {
      p->reverseCount = count;
    }
  }
}

/** @brief Initialise the plain tone generator: phase 0, step 0x688 (980 Hz at 9600 Hz). */
void v8_TONEq_init(V8Obj* p)
{
  p->toneqPhase = 0;
  p->toneqStep = 0x688;
}

/** @brief Generate 4 samples of the plain 980 Hz tone (full-scale cosine table values, no filter). */
void v8_TONEq_generate(V8Obj* p, short* out)
{
  int i;

  for (i = 0; i < 4; i = i + 1)
  {
    unsigned int phase = ((unsigned int)p->toneqPhase + (unsigned int)p->toneqStep) & 0x3fff;
    MEMBER_AS(short, p, toneqPhase) = (short)phase;
    out[i] = v8_cosread((unsigned char)((phase + 0x20) >> 6));
  }
}

/** @brief Cosine table lookup: one of 256 points of a period (Q14). */
short v8_cosread(unsigned char index) { return v8_costbl[index]; }

/* ---- V.21 FSK modulator and demodulator (carry the CM / JM / CJ bits) ---- */

/**
 * @brief Select the V.21 channels of the modulator and the demodulator.
 * @param p       the engine object
 * @param txBand  0 = transmit on the low channel (mark 980 Hz, space 1180 Hz), 1 = the high channel (mark 1650 Hz, space 1850 Hz)
 * @param rxBand  0 = receive the low channel, 1 = the high channel; selects the correlator tables of the demodulator
 * @note  Loads the 61-tap transmit filter of the band, the carrier steps (13-bit phase at 9600 Hz) and the amplitude, resets the bit timing (32
 *        samples per bit: 300 bit/s) and the receive bit state, and sets flags bit 0x800.
 */
void v8_V21_Init(V8Obj* p, short txBand, short rxBand)
{
  const short *mark2, *space2, *space1, *mark1;

  if (txBand == 0)
  {
    v8_copycoeff((short*)(MEMBER_ADDR(p, v21Coef[0])), temp_v21_lobnd, 0x3d);
    p->fskSpaceStep = 0x3ef;
    p->fskMarkStep = 0x344;
    p->fskAmp = (short)v8_mpyint(0x3224, p->txGain);
  }
  else
  {
    v8_copycoeff((short*)(MEMBER_ADDR(p, v21Coef[0])), temp_v21_hibnd, 0x3d);
    p->fskSpaceStep = 0x62b;
    p->fskMarkStep = 0x580;
    p->fskAmp = (short)v8_mpyint(0x3224, p->txGain);
  }
  if (rxBand == 0)
  {
    mark1 = LM_filter1_coef;
    p->wc2c = 7;
    mark2 = LM_filter2_coef;
    space1 = LS_filter1_coef;
    p->wc2e = 0;
    space2 = LS_filter2_coef;
  }
  else
  {
    mark2 = HM_filter2_coef;
    p->wc2c = 4;
    space2 = HS_filter2_coef;
    mark1 = HM_filter1_coef;
    p->wc2e = 0xff9c;
    space1 = HS_filter1_coef;
  }
  p->fskTxPhase = 0;
  p->bitSamples = 0x20;
  p->txBitPos = 0;
  p->rxFill = 0;
  p->rxShiftReg = 0;
  p->rxBitCount = 0;
  p->spaceBit = 0;
  p->markBit = 1;
  p->wc34 = 0x18;
  p->flags = p->flags | 0x800;
  V8_setFilters(p, space1, space2, mark1, mark2);
  V8_V21_reset(p);
  p->lastOneRun = 0;
  p->zeroRun = 0;
  p->oneRun = 0;
  p->preambleEnds = 0;
  p->preambleEndsHandled = 0;
}

/** @brief Store the table handles of the four demodulator correlators (space I / Q, mark I / Q). */
void V8_setFilters(V8Obj* p, const void* space1, const void* space2, const void* mark1, const void* mark2)
{
  CFGP_SLOT_PUT(const void*, &p->spaceFilt1, space1);
  CFGP_SLOT_PUT(const void*, &p->spaceFilt2, space2);
  CFGP_SLOT_PUT(const void*, &p->markFilt1, mark1);
  CFGP_SLOT_PUT(const void*, &p->markFilt2, mark2);
}

/** @brief Clear the demodulator window (the older 40 entries of fskDelay) and the mark / space run counters. */
void V8_V21_reset(V8Obj* p)
{
  int i;
  for (i = 0; i < 0x28; i = i + 1)
  {
    p->fskDelay[12 + i] = 0;
  }
  p->decisionPhase = 0;
  p->markRun = 0;
  p->spaceRun = 0;
}

/**
 * @brief Modulate one FSK bit period slice: 4 samples of the mark (bit 1) or space (bit 0) carrier through v8_fsktxfilter(), appended to the
 *        transmit ring by v8_txwritequeue().
 */
void v8_fskmodulate(V8Obj* p, short bit)
{
  short* phase = (short*)(MEMBER_ADDR(p, fskTxPhase));
  unsigned short* dst = (unsigned short*)(MEMBER_ADDR(p, txBuf[0]));
  int i;
  short sample;

  if (bit == 0)
  {
    for (i = 0; i < 4; i = i + 1)
    {
      unsigned int phaseNext = ((int)*phase + (int)p->fskSpaceStep) & 0x1fff;
      *phase = (short)phaseNext;
      sample = v8_cosread((unsigned char)(phaseNext >> 5));
      sample = (short)v8_mpyint(sample, p->fskAmp);
      dst[i] = (unsigned short)v8_fsktxfilter(p, sample);
    }
  }
  else
  {
    for (i = 0; i < 4; i = i + 1)
    {
      unsigned int phaseNext = ((int)*phase + (int)p->fskMarkStep) & 0x1fff;
      *phase = (short)phaseNext;
      sample = v8_cosread((unsigned char)(phaseNext >> 5));
      sample = (short)v8_mpyint(sample, p->fskAmp);
      dst[i] = (unsigned short)v8_fsktxfilter(p, sample);
    }
  }
  v8_txwritequeue(p);
}

/**
 * @brief The transmit shaping filter: a 61-tap FIR (the coefficients v8_V21_Init() loaded into v21Coef) over a sliding history of the generated
 *        samples (also used for ANSam).
 * @return the filtered sample (rounded, >> 16)
 */
short v8_fsktxfilter(V8Obj* p, short sample)
{
  short* ringDown = (short*)(MEMBER_ADDR(p, unk894[60]));
  short* ringUp = (short*)(MEMBER_ADDR(p, unk894[61]));
  short* coefs = (short*)(MEMBER_ADDR(p, v21Coef[0]));
  int acc = 0x8000;
  int i;

  MEMBER_AS(short, p, unk894[0]) = sample;
  for (i = 0; i < 0x3d; i = i + 1)
  {
    short tap = *ringDown;
    ringDown = ringDown - 1;
    *ringUp = tap;
    ringUp = ringUp - 1;
    acc = acc + (int)tap * (int)coefs[i];
  }
  return (short)(acc >> 0x10);
}

/**
 * @brief Demodulate a block of 4 received samples into bits.
 * @param p  the engine object (the block is rxBuf)
 * @note  The samples are collected in fskDelay; every 12 samples the demodulator takes decisions every 8 samples: four correlators (space and mark,
 *        in-phase and quadrature, tables set by V8_setFilters(), 40 taps over a sliding window) give a mark and a space energy; below 50000 for both
 *        the run counters are cleared (no carrier), otherwise the larger energy extends the mark or space run.  A change of symbol turns the
 *        finished run (a run counts 4 decisions per bit) into bits shifted into `rxShiftReg` and counted in `rxBitCount`.  Finally the window is
 *        slid by 12 samples.
 *        The control flow is a network of gotos, one shared exit (`tail`) flushing the runs of more than 4 decisions.
 */
void v8_fskdemodulate(V8Obj* p)
{
  unsigned short* ring = (unsigned short*)(MEMBER_ADDR(p, rxBuf[0]));
  int i;
  short fillPos;
  int nextDecision, runBits;
  unsigned int sampleIdx, lastTap, nextIdx, afterIdx, k;
  int markGeSpace, markWins;

  /* collect the block in the demodulator window */
  for (i = 0; i < 4; i = i + 1)
  {
    fillPos = p->rxFill;
    p->rxFill = fillPos + 1;
    MEMBER_AS_AT(unsigned short, p, fskDelay[0], fillPos * 2) = *ring;
    ring = ring + 1;
  }
  if (p->rxFill != 0xc)
  {
    return;
  }
  p->rxFill = 0;

  /* 12 samples are available: take the decisions that fall into them */
  nextDecision = p->decisionPhase;
  sampleIdx = 0;
  do
  {
    while ((int)sampleIdx < nextDecision)
    {
    advance:
      nextIdx = sampleIdx + 1;
      sampleIdx = nextIdx;
      if (0xb < (int)nextIdx)
      {
        goto tail;
      }
    }
    nextDecision = nextDecision + 8;
    {
      int spaceI = 0x2000, markI = 0x2000, spaceQ = 0x2000, markQ = 0x2000;
      lastTap = sampleIdx;
      if (0x27 < (int)sampleIdx)
      {
        lastTap = 0x27;
      }
      for (k = 0; k <= lastTap; k = k + 1)
      {
        int windowSample = (int)p->fskDelay[(sampleIdx - k)];
        spaceI = spaceI + *(short*)(CFGP_WORD(p->spaceFilt1) + k * 2) * windowSample;
        spaceQ = spaceQ + *(short*)(CFGP_WORD(p->spaceFilt2) + k * 2) * windowSample;
        markI = markI + *(short*)(CFGP_WORD(p->markFilt1) + k * 2) * windowSample;
        markQ = markQ + *(short*)(CFGP_WORD(p->markFilt2) + k * 2) * windowSample;
      }
      afterIdx = sampleIdx + 1;
      for (k = afterIdx; k < 0x28; k = k + 1)
      {
        int prevSample = (int)*(short*)(MEMBER_ADDR(p, sigTimeoutTicks) +
                                        (int)(sampleIdx - k) * 2); /* the index is negative here: an unsigned difference would be +4 GB on a 64-bit pointer */
        spaceI = spaceI + *(short*)(CFGP_WORD(p->spaceFilt1) + k * 2) * prevSample;
        spaceQ = spaceQ + *(short*)(CFGP_WORD(p->spaceFilt2) + k * 2) * prevSample;
        markI = markI + *(short*)(CFGP_WORD(p->markFilt1) + k * 2) * prevSample;
        markQ = markQ + *(short*)(CFGP_WORD(p->markFilt2) + k * 2) * prevSample;
      }
      {
        /* spaceFilt1 / 2 are the space in-phase / quadrature correlators, markFilt1 / 2 the mark ones (V8_setFilters()) */
        int spaceIOut = (int)(short)(spaceI >> 0xe);
        int spaceQOut = (int)(short)(spaceQ >> 0xe);
        int markIOut = (int)(short)(markI >> 0xe);
        int markQOut = (int)(short)(markQ >> 0xe);
        int spaceE = spaceIOut * spaceIOut + spaceQOut * spaceQOut;
        int markE = markIOut * markIOut + markQOut * markQOut;
        markGeSpace = -1 < (markE - spaceE) ? 1 : 0;
        markWins = (markE != spaceE) && markGeSpace;
        if (spaceE < 50000 && markE < 50000)
        {
          p->markRun = 0;
          p->spaceRun = 0;
          goto advance;
        }
        if (markWins)
        {
          k = p->spaceRun;
          if (k != 0)
          {
            unsigned int spaceRunUnits = k >> 2;
            p->spaceRun = k & 3;
            runBits = (spaceRunUnits < 3) ? (int)(spaceRunUnits + 1) : 3;
            runBits = (int)(spaceRunUnits + 1) - (unsigned int)(p->spaceRun < 4U - runBits);
            if (runBits != 0)
            {
              unsigned short shiftReg = p->rxShiftReg;
              short bitCount = p->rxBitCount;
              do
              {
                bitCount = bitCount + 1;
                shiftReg = (unsigned short)((shiftReg * 2) | p->spaceBit);
                runBits = runBits - 1;
              } while (runBits != 0);
              p->rxBitCount = bitCount;
              p->rxShiftReg = shiftReg;
            }
          }
          MEMBER_AS(int, p, markRun) = MEMBER_AS(int, p, markRun) + 1;
          p->spaceRun = 0;
          goto advance;
        }
        if (markE != spaceE && markGeSpace)
        {
          if (!markWins)
          {
            goto advance;
          }
          p->markRun = 0;
          p->spaceRun = 0;
          goto advance;
        }
        k = p->markRun;
        if (k != 0)
        {
          unsigned int markRunUnits = k >> 2;
          runBits = (markRunUnits < 3) ? (int)(markRunUnits + 1) : 3;
          p->markRun = k & 3;
          runBits = (int)(markRunUnits + 1) - (unsigned int)(p->markRun < 4U - runBits);
          if (runBits != 0)
          {
            unsigned short shiftReg = p->rxShiftReg;
            short bitCount = p->rxBitCount;
            do
            {
              bitCount = bitCount + 1;
              shiftReg = (unsigned short)((shiftReg * 2) | p->markBit);
              runBits = runBits - 1;
            } while (runBits != 0);
            p->rxBitCount = bitCount;
            p->rxShiftReg = shiftReg;
          }
        }
        p->markRun = 0;
        MEMBER_AS(int, p, spaceRun) = MEMBER_AS(int, p, spaceRun) + 1;
        sampleIdx = afterIdx;
      }
    }
  } while ((int)afterIdx < 0xc);
tail:
  /* flush runs of more than 4 decisions into bits */
  k = p->spaceRun;
  if (4 < k)
  {
    p->spaceRun = k & 3;
    k = k >> 2;
    if (k != 0)
    {
      unsigned short shiftReg = p->rxShiftReg;
      short bitCount = p->rxBitCount;
      do
      {
        bitCount = bitCount + 1;
        shiftReg = (unsigned short)((shiftReg * 2) | p->spaceBit);
        k = k - 1;
      } while (k != 0);
      p->rxBitCount = bitCount;
      p->rxShiftReg = shiftReg;
    }
  }
  k = p->markRun;
  if (4 < k)
  {
    p->markRun = k & 3;
    k = k >> 2;
    if (k != 0)
    {
      short bitCount = p->rxBitCount;
      unsigned short shiftReg = p->rxShiftReg;
      do
      {
        bitCount = bitCount + 1;
        shiftReg = (unsigned short)((shiftReg * 2) | p->markBit);
        k = k - 1;
      } while (k != 0);
      p->rxBitCount = bitCount;
      p->rxShiftReg = shiftReg;
    }
  }

  /* slide the window by 12 samples */
  p->decisionPhase = nextDecision - 0xc;
  for (i = 0; i < 0xc; i = i + 1)
  {
    unsigned short* src = (unsigned short*)(MEMBER_ADDR(p, fskDelay[13]));
    unsigned short* dstp = (unsigned short*)(MEMBER_ADDR(p, fskDelay[12]));
    int j;
    for (j = 0; j < 0x28; j = j + 1)
    {
      dstp[j] = src[j];
    }
  }
  {
    unsigned short* pw = (unsigned short*)(MEMBER_ADDR(p, fskDelay[51]));
    for (i = 0xb; i >= 0; i = i - 1)
    {
      pw[0] = pw[-0x28];
      pw = pw - 1;
    }
  }
}

/* ---- handshake set-up: initTxSequence and v8handshakinit ---- */

/**
 * @brief Build the outgoing menu message (the CM of the caller, the JM of the answerer) in the message slot `txMsgPtr` points at.
 * @param p  the engine object; the capability block comes from `capsPtr`
 * @note  The message is a list of 10-bit words (see V8SetMessage()): the sync words 0x3ff and 0xf, then the octets
 *        - call function: up to four raw octets of the capability block (caps byte 2 bit 2), or one octet chosen by the capability bits: data
 *          (0x107), call transmitting a fax (0x103), call receiving a fax (0x10b), V.80 (0x109); none selected falls back to data and prints a
 *          "BUG" message;
 *        - modulation modes: three octets built from the V.90 / V.34 / V.34 half duplex bits (byte 0), V.32 / V.22 / V.17 / V.29 / V.27 (byte 0 bit 7
 *          and byte 1) and V.23 / V.21 (byte 1);
 *        - protocol: up to four raw octets (caps byte 2 bit 3), else LAPM (0xa9) unless V.90 is offered;
 *        - PSTN access (0x161) and, when V.90 is offered, the PCM modem availability octets (0x1c9, 0x11).
 *        The slot header (CRC off, repeat on, 10 bits per word) is then initialised as V8SetMessage() does.
 * @warning A raw call function or protocol without octets clears its bit in the caller's capability block (the block is written here).
 */

void initTxSequence(V8Obj* p)
{
  unsigned short* seq = V8_PGM(unsigned short*, V8Obj, p, txMsgPtr);
  V8Msg* hdr = (V8Msg*)seq;
  const unsigned char* capb = V8_PGM(const unsigned char*, V8Obj, p, capsPtr);
  int outLen = 2;
  unsigned int idxOrFlags, count;
  unsigned char capByte;
  unsigned short word;
  int msgLen, pos;

  /* the two sync words */
  seq[0] = 0x3ff;
  seq[1] = 0xf;

  /* call function: raw octets from the capability block ... */
  if ((MEMBER_AS(unsigned int, V8_CAPS(capb), flags0) & 0x40000) != 0)
  {
    capByte = capb[0x18];
    idxOrFlags = 0;
    if (capByte != 0)
    {
      while (1)
      {
        count = idxOrFlags + 1;
        word = charFlip(capByte);
        seq[outLen] = (unsigned short)(((unsigned int)word & 0xff) * 2 | 1);
        outLen = outLen + 1;
        capByte = capb[0x19 + idxOrFlags];
        if (capByte == 0 || 3 < count)
          break;
        idxOrFlags = count;
      }
      if (count != 0)
        goto have_callfn;
    }
    if (1 < dsplibs_debug_level)
    {
      dsplibs_debug_printf("V8: BUG - raw Call Function selected without valid data !!!\r\n");
    }
    V8_CAPS(capb)->flags2 &= 0xfb;
  }

have_callfn:
  /* ... or the one octet chosen by the capability bits */
  idxOrFlags = MEMBER_AS(unsigned int, V8_CAPS(capb), flags0);
  if ((idxOrFlags & 0x40000) == 0)
  {
    if ((idxOrFlags & 0x4000) == 0)
    {
      if ((idxOrFlags & 0x10000) == 0)
      {
        if ((idxOrFlags & 0x8000) == 0)
        {
          if ((idxOrFlags & 0x20000) == 0)
          {
            if (1 < dsplibs_debug_level)
            {
              dsplibs_debug_printf("V8: BUG - no Call Function selected in bit fields - use data as default !!!\r\n");
            }
            V8_CAPS(capb)->flags1 |= 0x40;
            seq[outLen] = 0x107;
            outLen = outLen + 1;
          }
          else
          {
            seq[outLen] = 0x109;
            outLen = outLen + 1;
          }
        }
        else
        {
          seq[outLen] = 0x10b;
          outLen = outLen + 1;
        }
      }
      else
      {
        seq[outLen] = 0x103;
        outLen = outLen + 1;
      }
    }
    else
    {
      seq[outLen] = 0x107;
      outLen = outLen + 1;
    }
  }

  /* the three modulation-mode octets */
  capByte = capb[0];
  word = (unsigned short)((-(unsigned short)((capByte >> 3 & 1) == 0) & 0xfff8) + 0x149);
  if ((capByte & 0x20) != 0)
    word = word | 4;
  if ((capByte & 0x40) != 0)
    word = word | 2;
  seq[outLen] = word;

  idxOrFlags = MEMBER_AS(unsigned int, V8_CAPS(capb), flags0);
  word = (unsigned short)((-(unsigned short)(-1 < (signed char)capb[0]) & 0xff00) + 0x111);
  if ((idxOrFlags & 0x100) != 0)
    word = word | 0x80;
  if ((idxOrFlags & 0x200) != 0)
    word = word | 0x40;
  if ((idxOrFlags & 0x400) != 0)
    word = word | 4;
  if ((idxOrFlags & 0x800) != 0)
    word = word | 2;
  seq[outLen + 1] = word;

  pos = outLen + 3;
  word = (unsigned short)((-(unsigned short)((capb[1] >> 4 & 1) == 0) & 0xffc0) + 0x51);
  if ((capb[1] & 0x20) != 0)
    word = word | 0x13;
  seq[outLen + 2] = word;

  /* protocol: raw octets, or LAPM when V.90 is not offered */
  if ((MEMBER_AS(unsigned int, V8_CAPS(capb), flags0) & 0x80000) != 0)
  {
    unsigned int protoIdx = 0;
    msgLen = pos;
    if (capb[0x1c] != 0)
    {
      while (1)
      {
        pos = msgLen + 1;
        word = charFlip(capb[0x1c + protoIdx]);
        idxOrFlags = protoIdx + 1;
        seq[msgLen] = (unsigned short)(((unsigned int)word & 0xff) * 2 | 1);
        if (capb[0x1d + protoIdx] == 0 || 3 < idxOrFlags)
          break;
        msgLen = pos;
        protoIdx = idxOrFlags;
      }
      if (idxOrFlags != 0)
        goto have_protocol;
    }
    if (1 < dsplibs_debug_level)
    {
      dsplibs_debug_printf("V8: BUG - raw Protocol selected without valid data !!!\r\n");
    }
    V8_CAPS(capb)->flags2 &= 0xf7;
  }

have_protocol:
  if ((MEMBER_AS(unsigned int, V8_CAPS(capb), flags0) & 0x80008) == 0)
  {
    seq[pos] = 0xa9;
    pos = pos + 1;
  }

  /* PSTN access octet, and the PCM availability octets when V.90 is offered */
  seq[pos] = 0x161;
  msgLen = pos + 1;
  if ((MEMBER_AS(unsigned int, V8_CAPS(capb), flags0) & 8) != 0)
  {
    seq[msgLen] = 0x1c9;
    msgLen = pos + 3;
    seq[pos + 2] = 0x11;
  }

  if (1 < dsplibs_debug_level)
  {
    dsplibs_debug_printf("V8: Initial %s message length is %d octets\r\n", (p->side == 1) ? "JM" : "CM", msgLen);
  }

  /* the slot header, as V8SetMessage() initialises it */
  hdr->crcAcc = USHRT_MAX;
  hdr->bitsSent = 0;
  hdr->wordIdx = 0;
  hdr->passCount = 0;
  hdr->totalBits = (short)(msgLen * 10);
  hdr->wordBits = 10;
  hdr->crcEnable = 0;
  hdr->shiftWord = 0;
  hdr->reloadWord = 0;
  hdr->bitsInWord = 0;
  hdr->reloadBits = 0;
  hdr->repeatEnable = 1;
}

/**
 * @brief Initialise the handshake for the side the engine was created with (`side`: 0 = caller, 1 = answerer; any other value only does the common
 *        part).
 * @param p  the engine object (side, opMode, timeouts and the capability block already set by V8Create())
 * @return 0
 * @note  Common: transmit target 16 samples, DFT bin at 15 Hz, receive and transmit rings reset, no quick connect, no accepted menu entries.
 *        Caller: TX silence, RX phase ANSAM with sub-state "waiting for ANSam", the ANSam detector (v8_c2100, 100 hits after a head start of 50,
 *        threshold 1500), the phase-reversal detector, the CM built into msg[0], the received JM expected in msg[2], the CJ (three zero octets) in
 *        msg[1]; CM sending is enabled at once only for opMode 0 (V8Control() releases it otherwise); with quick connect enabled the QC message
 *        (sync words and the QCA1 word built from the capability block) is built into msg[3] and the QCA1 receive buffer is msg[4].
 *        Answerer: TX phase ANSam, RX phase "AGC settle", the ANSam generator, V.21 high channel for transmit and low channel for receive, the JM
 *        built into msg[2], the CM expected in msg[0].  The two timeouts become tick counts of 4 samples (seconds * 9600 / 4); 0 or less means none.
 */

unsigned int v8handshakinit(V8Obj* p)
{
  unsigned int ucodeForQts;
  unsigned short qcBitsC, qcBitsB, qcWord;
  short qtsLow;
  unsigned short txLevel;
  int limitTicks;
  unsigned char* caps;

  /* common state */
  MEMBER_AS(short, p, txQueueTarget) = 0x10;
  MEMBER_AS(short, p, dft.phase) = 0;
  MEMBER_AS(short, p, dft.step) = 0x1a;
  p->dft.im = 0;
  MEMBER_AS(short, p, dft.energy) = 0;
  p->dft.re = 0;
  MEMBER_AS(short, p, wa40) = 0x200;
  v8_rxinit(p);
  v8_txinit(p);
  p->qcDetected = 0;
  MEMBER_AS(short, p, qca1aSeen) = 0;
  p->protoWord = 0;
  p->qcLapm = 0;
  p->ansPcmLevel = 0;
  p->callFnAccepted = 0;
  p->protoAccepted = 0;
  p->callFnWord = 0;

  /* caller */
  if (p->side == 0)
  {
    MEMBER_AS(short, p, phasePair) = V8TX_SILENCE;
    MEMBER_AS(short, p, rxSubState) = V8SUB_ANSAM_WAIT;
    limitTicks = -1;
    MEMBER_AS_AT(short, p, phasePair, 2) = V8RX_ANSAM;
    p->flags = 0x8000;
    if (0 < MEMBER_AS(int, p, sigTimeout))
    {
      limitTicks = (MEMBER_AS(int, p, sigTimeout) * 0x2580) >> 2;
    }
    p->sigTimeoutTicks = limitTicks;
    limitTicks = -1;
    if (0 < MEMBER_AS(int, p, msgTimeout))
    {
      limitTicks = (MEMBER_AS(int, p, msgTimeout) * 0x2580) >> 2;
    }
    p->msgTimeoutTicks = limitTicks;
    p->timeoutCount = 0;

    /* ANSam detectors, the CM to send and the CJ (three zero octets) */
    v8_detectorinit(p, &p->det, v8_c2100, 0, 100, 0x32, 0x5dc, 0);
    v8_phase_rev_init(MEMBER_ADDR(p, phaseRev[0]));
    V8_PSM(char*, V8Obj, p, txMsgPtr, MEMBER_ADDR(p, msg[0]));
    V8_PSM(char*, V8Obj, p, rxMsgPtr, MEMBER_ADDR(p, msg[2]));
    initTxSequence(p);
    MEMBER_AS(short, p, msg[1].data[2]) = 1;
    MEMBER_AS(short, p, msg[1].data[0]) = 1;
    MEMBER_AS(short, p, msg[1].data[1]) = 1;
    p->msg[1].bitsSent = 0;
    p->cmEnabled = (unsigned short)(p->opMode == 0);
    p->msg[1].crcEnable = 0;
    p->msg[1].crcAcc = USHRT_MAX;
    p->msg[1].repeatEnable = 1;
    caps = V8_PGM(unsigned char*, V8Obj, p, capsPtr);
    p->msg[1].wordIdx = 0;
    p->msg[1].passCount = 0;
    p->msg[1].totalBits = 0x1e;
    p->msg[1].wordBits = 10;
    p->msg[1].shiftWord = 0;
    p->msg[1].reloadWord = 0;
    p->msg[1].bitsInWord = 0;
    MEMBER_AS(short, p, msg[1].reloadBits) = 0;

    /* quick connect: the QC preamble message (msg[3]) is sent first; the QCA1 words are received into msg[4] */
    if ((*(unsigned char*)(caps + 2) & 0x10) != 0)
    {
      V8_PSM(char*, V8Obj, p, txMsgPtr, MEMBER_ADDR(p, msg[3]));
      ucodeForQts = *(unsigned int*)(caps + 0x10);
      V8_PSM(char*, V8Obj, p, qcRxMsgPtr, MEMBER_ADDR(p, msg[4]));
      qtsLow = (short)ucodeForQts;
      qcWord = (unsigned short)(qtsLow * 2 & 4);
      MEMBER_AS(short, p, msg[3].data[0]) = 0x3ff;
      qcBitsB = (unsigned short)(qtsLow * 2 & 8);
      MEMBER_AS(short, p, msg[3].data[1]) = 0x155;
      qcBitsC = (unsigned short)(qtsLow * 4 & 0x20);
      if ((*(unsigned char*)(caps + 2) & 0x40) == 0)
      {
        qcWord = qcBitsC | qcBitsB | qcWord | 1;
      }
      else
      {
        qcWord = qcBitsC | qcBitsB | qcWord | 0x41;
      }
      if ((*(unsigned char*)(caps + 0x10) & 1) != 0)
      {
        qcWord = qcWord | 2;
      }
      p->msg[3].data[2] = qcWord;
      p->msg[3].data[5] = qcWord;
      MEMBER_AS(short, p, msg[3].data[3]) = MEMBER_AS(short, p, msg[3].data[0]);
      MEMBER_AS(short, p, msg[3].data[4]) = MEMBER_AS(short, p, msg[3].data[1]);
      p->qcPreambleCount = 0;
      MEMBER_AS(short, p, toneqPhase) = 0;
      MEMBER_AS(short, p, toneqStep) = 0x688;
      p->msg[3].crcAcc = USHRT_MAX;
      p->msg[3].bitsSent = 0;
      p->msg[3].wordIdx = 0;
      p->msg[3].passCount = 0;
      p->msg[3].totalBits = 0x3c;
      p->msg[3].wordBits = 10;
      p->msg[3].crcEnable = 0;
      p->msg[3].shiftWord = 0;
      p->msg[3].reloadWord = 0;
      p->msg[3].bitsInWord = 0;
      MEMBER_AS(short, p, msg[3].reloadBits) = 0;
      p->msg[3].repeatEnable = 1;
    }
    p->stepCount = 0;
    MEMBER_AS(short, p, wdb4) = 0;
    return 0;
  }

  if (p->side != 1)
  {
    return 0;
  }

  /* answerer */
  MEMBER_AS_AT(short, p, phasePair, 2) = V8RX_AGC_SETTLE;
  limitTicks = -1;
  MEMBER_AS(short, p, phasePair) = V8TX_ANSAM;
  if (0 < MEMBER_AS(int, p, sigTimeout))
  {
    limitTicks = (MEMBER_AS(int, p, sigTimeout) * 0x2580) >> 2;
  }
  p->sigTimeoutTicks = limitTicks;
  limitTicks = -1;
  if (0 < MEMBER_AS(int, p, msgTimeout))
  {
    limitTicks = (MEMBER_AS(int, p, msgTimeout) * 0x2580) >> 2;
  }
  p->msgTimeoutTicks = limitTicks;

  /* ANSam generator, V.21 high channel out / low channel in, the JM to send */
  p->tonePhase = 0;
  MEMBER_AS(short, p, amStep) = 0x1a;
  p->toneStep = 0xe00;
  p->reverseCount = 0;
  p->timeoutCount = 0;
  MEMBER_AS(short, p, amPhase) = 0;
  txLevel = (unsigned short)v8_mpyint(16000, (int)p->txGain);
  p->ansamAmp = txLevel;
  p->reverseEnable = 1;
  v8_V21_Init(p, 1, 0);
  V8_PSM(char*, V8Obj, p, txMsgPtr, MEMBER_ADDR(p, msg[2]));
  V8_PSM(char*, V8Obj, p, rxMsgPtr, MEMBER_ADDR(p, msg[0]));
  p->flags = 0x8004;
  initTxSequence(p);
  p->zeroBits = 0;
  p->stepCount = 0;
  MEMBER_AS(short, p, wdb4) = 0;
  return 0;
}

/**
 * @brief Answerer: build the JM reply from the received CM and the local capabilities.
 * @param p  the engine object: the received CM is msg[0], the reply goes to the slot `txMsgPtr` points at, the capabilities are at `capsPtr`
 * @note  Walks the words of the received CM and, for each part, keeps what both sides support:
 *        - call function: the remote octet (tag 0x101) is accepted when the local capability bits (or raw octets) allow it; otherwise the local
 *          choice is sent; `callFnAccepted` / `callFnWord` record the outcome ("NO Call Function Match" zeroes all modulation capabilities);
 *        - modulation modes: the received modulation octets (tag 0x141, up to three) are ANDed into the local ones; whether the remote offers V.90
 *          is remembered;
 *        - PSTN access (tag 0x161) gives the remote digital connection flag and PCM availability (tag 0x1c1) the PCM indication;
 *        - protocol: the remote octet (tag 0xa1) is matched against the raw protocol octets, the protocol range or the default LAPM; `protoAccepted`
 *          / `protoWord` record it.
 *        When V.90 is offered by both sides with a digital connection and PCM indication 1, the V.90 bit is set in the first modulation octet and the
 *        PCM availability octets are appended.  The message ends with the PSTN access octet and the slot header is initialised.
 *        The control flow is a flat network of gotos; its labels name the scan steps (call function, protocol range, V.90 intersection, trailer).
 */

void rebuildJMSequence(V8Obj* p)
{
  unsigned short protoWord;
  unsigned short* txMsg;
  int matched;
  char capOctet;
  unsigned char capByte;
  unsigned short rxWord;
  unsigned short flippedWord;
  short tmpShort;
  unsigned int rxWordU;
  unsigned int rxTag;
  int rxIdx;
  int idxA;
  unsigned char* txAddr;
  int idxB;
  unsigned char* txAddrB;
  int idxC;
  int idxD;
  unsigned char* caps;
  char* msgText;
  short pcmIndication;
  short digitalConn;
  unsigned short remoteV90Mod;
  int outLen;

  pcmIndication = 0;
  rxIdx = 0;
  remoteV90Mod = 0;
  txMsg = V8_PGM(unsigned short*, V8Obj, p, txMsgPtr);
  digitalConn = 0;
  outLen = 2;
  *txMsg = 0x3ff;
  txMsg[1] = 0xf;
  matched = 0;
  if (0 < p->msg[0].wordIdx)
  {
    do
    {
      rxWord = MEMBER_AS_AT(unsigned short, p, msg[0].data[0], rxIdx * 2);
      if ((rxWord & 0xfff1) == 0x101)
      {
        if (1 < dsplibs_debug_level)
        {
          dsplibs_debug_printf("V8: on ANSWER: remote call function is: %X\r\n", rxWord);
        }
        caps = V8_PGM(unsigned char*, V8Obj, p, capsPtr);
        capByte = *(unsigned char*)(caps + 2);
        if ((capByte & 4) != 0)
        {
          capOctet = *(char*)(caps + 0x18);
          idxB = 0;
          if (capOctet != '\0')
            goto rawCfCompare;
          goto rawCfCheck;
        }
        tmpShort = MEMBER_AS_AT(short, p, msg[0].data[0], rxIdx * 2);
        if (tmpShort == 0x107)
        {
          capByte = *(unsigned char*)(caps + 1) & 0x40;
        }
        else
        {
          if (tmpShort == 0x103)
          {
            if ((capByte & 1) == 0)
              goto cfRangeSearch;
            if (1 < dsplibs_debug_level)
            {
              dsplibs_debug_printf("V8: call function FAX TX from caller indication...\r\n");
            }
            goto cfMatched;
          }
          if (tmpShort == 0x10b)
          {
            if ((*(unsigned char*)(caps + 1) & 0x80) == 0)
              goto cfRangeSearch;
            if (1 < dsplibs_debug_level)
            {
              dsplibs_debug_printf("V8: call function FAX RX to caller indication...\r\n");
            }
            goto cfMatched;
          }
          if (tmpShort != 0x109)
            goto cfRangeSearch;
          capByte = capByte & 2;
        }
        if (capByte == 0)
          goto cfRangeSearch;
        if (1 < dsplibs_debug_level)
        {
          dsplibs_debug_printf("V8: call function DATA indication...\r\n");
        }
        goto cfMatched;
      }
      rxIdx = rxIdx + 1;
    } while (rxIdx < p->msg[0].wordIdx);
  }
  tmpShort = p->callFnAccepted;
pickLocalCallFn:
  if (tmpShort != 0)
    goto copyModulation;
  caps = V8_PGM(unsigned char*, V8Obj, p, capsPtr);
  capByte = *(unsigned char*)(caps + 2);
  if ((capByte & 4) == 0)
  {
    if ((*(unsigned char*)(caps + 1) & 0x40) == 0)
    {
      if ((capByte & 1) == 0)
      {
        if ((*(unsigned char*)(caps + 1) & 0x80) == 0)
        {
          if ((capByte & 2) == 0)
          {
            if (1 < dsplibs_debug_level)
            {
              dsplibs_debug_printf("V8: BUG - no Call Function selected in bit fields - use data as default !!!\r\n");
              caps = V8_PGM(unsigned char*, V8Obj, p, capsPtr);
            }
            *(unsigned char*)(caps + 1) = *(unsigned char*)(caps + 1) | 0x40;
            *(unsigned short*)(V8_PGM(unsigned char*, V8Obj, p, txMsgPtr) + outLen * 2) = 0x107;
            outLen = outLen + 1;
          }
          else
          {
            *(unsigned short*)(V8_PGM(unsigned char*, V8Obj, p, txMsgPtr) + outLen * 2) = 0x109;
            outLen = outLen + 1;
          }
        }
        else
        {
          *(unsigned short*)(V8_PGM(unsigned char*, V8Obj, p, txMsgPtr) + outLen * 2) = 0x10b;
          outLen = outLen + 1;
        }
      }
      else
      {
        *(unsigned short*)(V8_PGM(unsigned char*, V8Obj, p, txMsgPtr) + outLen * 2) = 0x103;
        outLen = outLen + 1;
      }
    }
    else
    {
      *(unsigned short*)(V8_PGM(unsigned char*, V8Obj, p, txMsgPtr) + outLen * 2) = 0x107;
      outLen = outLen + 1;
    }
  callFnDone:
    if (p->callFnAccepted != 0)
      goto copyModulation;
  }
  else
  {
    capOctet = *(char*)(caps + 0x18);
    rxIdx = 0;
    if (capOctet != '\0')
    {
      do
      {
        txAddrB = V8_PGM(unsigned char*, V8Obj, p, txMsgPtr);
        idxD = rxIdx + 1;
        idxA = outLen + 1;
        rxWord = charFlip((unsigned char)capOctet);
        *(unsigned short*)(txAddrB + outLen * 2) = (unsigned short)((rxWord & 0xff) * 2 | 1);
        capOctet = *(char*)(V8_PGM(unsigned char*, V8Obj, p, capsPtr) + (rxIdx + 0x19));
        rxIdx = idxD;
        outLen = idxA;
      } while (capOctet != '\0' && idxD < 4);
      goto callFnDone;
    }
  }
  if (1 < dsplibs_debug_level)
  {
    dsplibs_debug_printf("V8: NO Call Function Match !!! zeroing all modulation capabilities...\r\n");
  }
  txAddr = V8_PGM(unsigned char*, V8Obj, p, txMsgPtr);
  caps = V8_PGM(unsigned char*, V8Obj, p, capsPtr);
  *(unsigned short*)(txAddr + outLen * 2) = 0x141;
  *(unsigned short*)(txAddr + 2 + outLen * 2) = 0x11;
  *(unsigned short*)(txAddr + 4 + outLen * 2) = 0x11;
  outLen = outLen + 3;
checkV90Rebuild:
  if ((*caps & 8) != 0 && remoteV90Mod != 0)
  {
  v90Intersect:
    if (digitalConn != 0 && pcmIndication == 1)
    {
      if ((caps[2] & 8) != 0)
      {
        tmpShort = p->msg[0].wordIdx;
        rxIdx = 0;
        matched = 0;
        if (0 < tmpShort)
        {
          do
          {
            while ((MEMBER_AS_AT(unsigned short, p, msg[0].data[0], rxIdx * 2) & 0xfff1) != 0xa1)
            {
              rxIdx = rxIdx + 1;
              if (tmpShort <= rxIdx)
                goto jmTailPrep;
            }
            capByte = caps[0x1c];
            idxA = 0;
            if (capByte != 0)
            {
              do
              {
                while (1)
                {
                  flippedWord = charFlip(capByte);
                  rxWord = MEMBER_AS_AT(unsigned short, p, msg[0].data[0], rxIdx * 2);
                  if (rxWord != (unsigned short)((flippedWord & 0xff) * 2 | 1))
                    break;
                  idxB = idxA + 1;
                  matched = 1;
                  rxIdx = rxIdx + 1;
                  *(unsigned short*)(V8_PGM(unsigned char*, V8Obj, p, txMsgPtr) + outLen * 2) = rxWord;
                  outLen = outLen + 1;
                  caps = V8_PGM(unsigned char*, V8Obj, p, capsPtr);
                  capByte = caps[idxA + 0x1d];
                  idxA = idxB;
                  if (capByte == 0 || 3 < idxB)
                    goto v90ProtoCheck;
                }
                if (matched)
                {
                  caps = V8_PGM(unsigned char*, V8Obj, p, capsPtr);
                  break;
                }
                caps = V8_PGM(unsigned char*, V8Obj, p, capsPtr);
                idxB = idxA + 1;
                capByte = caps[idxA + 0x1d];
                idxA = idxB;
              } while (capByte != 0 && idxB < 4);
            }
          v90ProtoCheck:
            if ((caps[idxA + 0x1c] == 0 || idxA == 4) && matched)
              goto protoMatchedFlag;
            tmpShort = p->msg[0].wordIdx;
            rxIdx = rxIdx + 1;
          } while (rxIdx < tmpShort);
        }
      }
      goto jmTailPrep;
    }
  }
  goto protoSearch;
cfRangeAdvance:
  idxC = idxB + 1;
  idxD = idxB + 0x21;
  idxB = idxC;
  if (*(char*)(caps + idxD) == '\0' || 7 < idxC)
    goto cfResolved;
  goto cfRangeCompare;
rawProtoCompare:
  do
  {
    flippedWord = charFlip(caps[idxA + 0x1c]);
    rxWord = MEMBER_AS_AT(unsigned short, p, msg[0].data[0], rxIdx * 2);
    if (rxWord == (unsigned short)((flippedWord & 0xff) * 2 | 1))
    {
      rxIdx = rxIdx + 1;
      matched = 1;
      *(unsigned short*)(V8_PGM(unsigned char*, V8Obj, p, txMsgPtr) + outLen * 2) = rxWord;
      outLen = outLen + 1;
      p->protoWord = rxWord;
    }
    else if (matched)
    {
      caps = V8_PGM(unsigned char*, V8Obj, p, capsPtr);
      break;
    }
    caps = V8_PGM(unsigned char*, V8Obj, p, capsPtr);
    idxD = idxA + 1;
    idxB = idxA + 0x1d;
    idxA = idxD;
  } while (caps[idxB] != 0 && idxD < 4);
rawProtoCheck:
  if ((caps[idxA + 0x1c] == 0 || idxA == 4) && matched)
  {
  protoMatchedFlag:
    p->protoAccepted = 1;
  jmTailPrep:
    txAddr = V8_PGM(unsigned char*, V8Obj, p, txMsgPtr);
    goto appendTrailer;
  }
  p->protoWord = 0;
protoRangeSearch:
  if (!matched)
  {
    capByte = charFlip((unsigned char)(MEMBER_AS_AT(unsigned short, p, msg[0].data[0], rxIdx * 2) >> 1 & 0xff));
    caps = V8_PGM(unsigned char*, V8Obj, p, capsPtr);
    idxA = 0;
    if (caps[0x28] != 0)
    {
    protoRangeCompare:
      if (caps[idxA + 0x28] != capByte)
        goto protoRangeAdvance;
      if (1 < dsplibs_debug_level)
      {
        msgText = "V8: Got Protocol Match (in protocol range) !!!\r\n";
      printProtoMsg:
        dsplibs_debug_printf(msgText);
        caps = V8_PGM(unsigned char*, V8Obj, p, capsPtr);
      }
    protoMatched:
      matched = 1;
    }
  }
protoResolved:
  tmpShort = p->protoAccepted;
  if (tmpShort == 0 && matched)
  {
    if (((*caps & 8) == 0 || remoteV90Mod == 0) || (digitalConn == 0 || pcmIndication != 1))
    {
      txAddr = V8_PGM(unsigned char*, V8Obj, p, txMsgPtr);
      *(unsigned short*)(txAddr + outLen * 2) = MEMBER_AS_AT(unsigned short, p, msg[0].data[0], rxIdx * 2);
    protoAppended:
      outLen = outLen + 1;
      protoWord = MEMBER_AS_AT(unsigned short, p, msg[0].data[0], rxIdx * 2);
    }
    else
    {
      tmpShort = MEMBER_AS_AT(short, p, msg[0].data[0], rxIdx * 2);
      if (tmpShort != 0xa9)
      {
        txAddr = V8_PGM(unsigned char*, V8Obj, p, txMsgPtr);
        *(short*)(txAddr + outLen * 2) = tmpShort;
        goto protoAppended;
      }
      txAddr = V8_PGM(unsigned char*, V8Obj, p, txMsgPtr);
      protoWord = 0xa9;
    }
    p->protoWord = protoWord;
    p->protoAccepted = 1;
    goto appendTrailer;
  }
  txAddr = V8_PGM(unsigned char*, V8Obj, p, txMsgPtr);
  goto protoFallback;
protoRangeAdvance:
  idxD = idxA + 1;
  idxB = idxA + 0x29;
  idxA = idxD;
  if (caps[idxB] == 0 || 7 < idxD)
    goto protoResolved;
  goto protoRangeCompare;
rawCfCompare:
  do
  {
    flippedWord = charFlip((unsigned char)capOctet);
    rxWord = MEMBER_AS_AT(unsigned short, p, msg[0].data[0], rxIdx * 2);
    if (rxWord == (unsigned short)((flippedWord & 0xff) * 2 | 1))
    {
      rxIdx = rxIdx + 1;
      matched = 1;
      *(unsigned short*)(V8_PGM(unsigned char*, V8Obj, p, txMsgPtr) + outLen * 2) = rxWord;
      outLen = outLen + 1;
      p->callFnWord = rxWord;
    }
    else if (matched)
    {
      caps = V8_PGM(unsigned char*, V8Obj, p, capsPtr);
      break;
    }
    caps = V8_PGM(unsigned char*, V8Obj, p, capsPtr);
    idxD = idxB + 1;
    capOctet = *(char*)(idxB + 0x19 + caps);
    idxB = idxD;
  } while (capOctet != '\0' && idxD < 4);
rawCfCheck:
  if (*(char*)(idxB + 0x18 + caps) == '\0' || idxB == 4)
  {
    if (!matched)
    {
    cfRangeSearch:
      capOctet = (char)charFlip((unsigned char)(MEMBER_AS_AT(unsigned short, p, msg[0].data[0], rxIdx * 2) >> 1 & 0xff));
      caps = V8_PGM(unsigned char*, V8Obj, p, capsPtr);
      idxB = 0;
      if (*(char*)(caps + 0x20) != '\0')
      {
      cfRangeCompare:
        if (*(char*)(idxB + 0x20 + caps) != capOctet)
          goto cfRangeAdvance;
        if (1 < dsplibs_debug_level)
        {
          dsplibs_debug_printf("V8: Got Call Function Match (in call function range) !!!\r\n");
        }
      cfMatched:
        matched = 1;
      }
      goto cfResolved;
    }
  }
  else
  {
    if (!matched)
      goto cfRangeSearch;
  cfResolved:
    tmpShort = p->callFnAccepted;
    if (tmpShort != 0 || !matched)
      goto pickLocalCallFn;
    *(unsigned short*)(V8_PGM(unsigned char*, V8Obj, p, txMsgPtr) + outLen * 2) = MEMBER_AS_AT(unsigned short, p, msg[0].data[0], rxIdx * 2);
    outLen = outLen + 1;
    p->callFnWord = MEMBER_AS_AT(unsigned short, p, msg[0].data[0], rxIdx * 2);
  }
  p->callFnAccepted = 1;
copyModulation:
  rxIdx = outLen;
  tmpShort = p->msg[0].wordIdx;
  idxA = 0;
  if (0 < tmpShort)
  {
    do
    {
      while (1)
      {
        rxWord = MEMBER_AS_AT(unsigned short, p, msg[0].data[0], idxA * 2);
        rxWordU = (unsigned int)rxWord;
        if ((rxWordU & 0xfff1) != 0x141)
          break;
        txAddrB = V8_PGM(unsigned char*, V8Obj, p, txMsgPtr);
        *(unsigned short*)(txAddrB + outLen * 2) = *(unsigned short*)(txAddrB + outLen * 2) & rxWord;
        idxD = idxA + 1;
        remoteV90Mod = MEMBER_AS_AT(unsigned short, p, msg[0].data[0], idxA * 2) >> 3 & 1;
        rxWord = MEMBER_AS_AT(unsigned short, p, msg[0].data[0], idxD * 2);
        rxWordU = (unsigned int)rxWord;
        if ((rxWordU & 0x39) == 0x11)
        {
          idxD = idxA + 2;
          *(unsigned short*)(txAddrB + (outLen + 1) * 2) = *(unsigned short*)(txAddrB + (outLen + 1) * 2) & rxWord;
          rxWord = MEMBER_AS_AT(unsigned short, p, msg[0].data[0], idxD * 2);
          rxWordU = (unsigned int)rxWord;
          if ((rxWordU & 0x39) == 0x11)
          {
            idxD = idxA + 3;
            *(unsigned short*)(txAddrB + (outLen + 2) * 2) = *(unsigned short*)(txAddrB + (outLen + 2) * 2) & rxWord;
            rxWordU = (unsigned int)MEMBER_AS_AT(unsigned short, p, msg[0].data[0], idxD * 2);
          }
        }
        outLen = outLen + 3;
        rxTag = rxWordU & 0xfff1;
        idxA = idxD;
        if (rxTag != 0x161)
          goto tag1c1Check;
      tag161Seen:
        idxA = idxD + 1;
        digitalConn = (short)(rxWordU >> 1 & 1);
        if (tmpShort <= idxA)
          goto v90Report;
      }
      rxTag = rxWordU & 0xfff1;
      idxD = idxA;
      if (rxTag == 0x161)
        goto tag161Seen;
    tag1c1Check:
      if (rxTag == 0x1c1)
      {
        pcmIndication = (short)(rxWordU >> 2 & 3);
      }
      idxA = idxA + 1;
    } while (idxA < tmpShort);
  }
v90Report:
  if (1 < dsplibs_debug_level)
  {
    dsplibs_debug_printf("V8: on ANSWER: remote V90: mod - %d, digital connection - %d, pcmIndication - %d , local - %d\n", remoteV90Mod, digitalConn,
                         pcmIndication, *V8_PGM(unsigned char*, V8Obj, p, capsPtr) >> 3 & 1);
  }
  caps = V8_PGM(unsigned char*, V8Obj, p, capsPtr);
  if (((*caps & 8) == 0 || remoteV90Mod == 0) || (digitalConn == 0 || pcmIndication != 1))
    goto checkV90Rebuild;
  if (1 < dsplibs_debug_level)
  {
    dsplibs_debug_printf("V8: on ANSWER: rebuilding JM with V90 capabilities...\r\n");
    caps = V8_PGM(unsigned char*, V8Obj, p, capsPtr);
  }
  *(unsigned short*)(V8_PGM(unsigned char*, V8Obj, p, txMsgPtr) + rxIdx * 2) = *(unsigned short*)(V8_PGM(unsigned char*, V8Obj, p, txMsgPtr) + rxIdx * 2) | 8;
  if ((*caps & 8) != 0 && remoteV90Mod != 0)
    goto v90Intersect;
protoSearch:
  matched = 0;
  if (0 < p->msg[0].wordIdx)
  {
    rxIdx = 0;
    do
    {
      rxWord = MEMBER_AS_AT(unsigned short, p, msg[0].data[0], rxIdx * 2);
      if ((rxWord & 0xfff1) == 0xa1)
      {
        if ((caps[2] & 8) != 0)
        {
          idxA = 0;
          if (caps[0x1c] == 0)
            goto rawProtoCheck;
          goto rawProtoCompare;
        }
        if (caps[0x28] != 0)
          goto protoRangeSearch;
        if (rxWord != 0xa9)
          goto protoResolved;
        if (dsplibs_debug_level < 2)
          goto protoMatched;
        msgText = "V8: Got Default Protocol Match (LAPM) !!!\r\n";
        goto printProtoMsg;
      }
      rxIdx = rxIdx + 1;
    } while (rxIdx < p->msg[0].wordIdx);
  }
  tmpShort = p->protoAccepted;
  txAddr = V8_PGM(unsigned char*, V8Obj, p, txMsgPtr);
protoFallback:
  if (tmpShort == 0)
  {
    if ((caps[2] & 8) == 0)
    {
      if (((*caps & 8) == 0 || remoteV90Mod == 0) || (digitalConn == 0 || pcmIndication != 1))
      {
        *(unsigned short*)(txAddr + outLen * 2) = 0xa9;
        outLen = outLen + 1;
        p->protoWord = 0xa9;
      }
    }
    else
    {
      capByte = caps[0x1c];
      if (capByte != 0)
      {
        rxIdx = 0;
        do
        {
          idxD = rxIdx + 1;
          rxWord = charFlip(capByte);
          *(unsigned short*)(txAddr + outLen * 2) = (unsigned short)((rxWord & 0xff) * 2 | 1);
          txAddr = V8_PGM(unsigned char*, V8Obj, p, txMsgPtr);
          idxB = outLen * 2;
          outLen = outLen + 1;
          caps = V8_PGM(unsigned char*, V8Obj, p, capsPtr);
          p->protoWord = *(unsigned short*)(txAddr + idxB);
          capByte = caps[rxIdx + 0x1d];
          rxIdx = idxD;
        } while (capByte != 0 && idxD < 4);
      }
    }
  }
appendTrailer:
  *(unsigned short*)(txAddr + outLen * 2) = 0x161;
  rxIdx = outLen + 1;
  if (((*caps & 8) != 0 && remoteV90Mod != 0) && (digitalConn != 0 && pcmIndication == 1))
  {
    *(unsigned short*)(txAddr + rxIdx * 2) = 0x1c9;
    *(unsigned short*)(txAddr + 4 + outLen * 2) = 0x11;
    rxIdx = outLen + 3;
  }
  if (1 < dsplibs_debug_level)
  {
    dsplibs_debug_printf("V8: Final JM message length is %d octets\r\n", (int)rxIdx);
    txAddr = V8_PGM(unsigned char*, V8Obj, p, txMsgPtr);
  }
  *(unsigned short*)(txAddr + 0x1e) = USHRT_MAX;
  *(unsigned short*)(txAddr + 0x24) = 0;
  *(unsigned short*)(txAddr + 0x28) = 0;
  *(unsigned short*)(txAddr + 0x2c) = 0;
  *(short*)(txAddr + 0x22) = (short)(rxIdx * 10);
  *(unsigned short*)(txAddr + 0x26) = 10;
  *(unsigned short*)(txAddr + 0x20) = 0;
  *(unsigned int*)(txAddr + 0x30) = 0;
  *(unsigned int*)(txAddr + 0x38) = 0;
  *(unsigned short*)(txAddr + 0x34) = 0;
  *(unsigned short*)(txAddr + 0x3c) = 0;
  *(unsigned short*)(txAddr + 0x2a) = 1;
}

/**
 * @brief Caller: find out which call function and protocol the received JM accepted.
 * @param p  the engine object: the received JM is msg[2], the capabilities are at `capsPtr`
 * @note  Scans the JM for the call function octet (tag 0x101) and accepts it when it is one the local capability bits (or raw octets) offer; then
 *        for the protocol octet (tag 0xa1).  The results are `callFnAccepted` / `callFnWord` and `protoAccepted` / `protoWord`; with debug level 2
 *        the outcome is printed.  V8UpdateModemParameters() turns them into capability bits.
 *        The control flow is a flat network of gotos around the two scans.
 */

void evaluateRxJMSequence(V8Obj* p)
{
  unsigned char capByte;
  unsigned short rxWord;
  int matched;
  unsigned short flippedWord;
  char capOctet;
  short rxCountOrWord;
  char* suffixText;
  char* gotText;
  unsigned char* caps;
  int rxIdx;
  int capIdx;
  int capIdxDone;

  rxIdx = 0;
  rxCountOrWord = p->msg[2].wordIdx;
  p->callFnAccepted = 0;
  if (0 < rxCountOrWord)
  {
    do
    {
      while (rxWord = MEMBER_AS_AT(unsigned short, p, msg[2].data[0], rxIdx * 2), (rxWord & 0xfff1) != 0x101)
      {
        rxIdx = rxIdx + 1;
        if (rxCountOrWord <= rxIdx)
          goto scanProtocols;
      }
      if (1 < dsplibs_debug_level)
      {
        dsplibs_debug_printf("V8: on CALLER: remote call function is: %X\r\n", rxWord);
      }
      caps = V8_PGM(unsigned char*, V8Obj, p, capsPtr);
      capByte = *(unsigned char*)(caps + 2);
      if ((capByte & 4) == 0)
      {
        rxCountOrWord = MEMBER_AS_AT(short, p, msg[2].data[0], rxIdx * 2);
        if (rxCountOrWord == 0x107)
        {
          capByte = *(unsigned char*)(caps + 1) & 0x40;
        cfTestBits:
          if (capByte != 0)
          {
            if (dsplibs_debug_level < 2)
            {
            cfMatchedQuiet:
              p->callFnAccepted = 1;
            }
            else
            {
              dsplibs_debug_printf("V8: call function DATA indication...\r\n");
            cfMatchedLogged:
              p->callFnAccepted = 1;
            }
          }
        }
        else if (rxCountOrWord == 0x103)
        {
          if ((capByte & 1) != 0)
          {
            if (dsplibs_debug_level < 2)
              goto cfMatchedQuiet;
            dsplibs_debug_printf("V8: call function FAX TX from caller indication...\r\n");
            goto cfMatchedLogged;
          }
        }
        else if (rxCountOrWord == 0x10b)
        {
          if ((*(unsigned char*)(caps + 1) & 0x80) != 0)
          {
            if (1 < dsplibs_debug_level)
            {
              dsplibs_debug_printf("V8: call function FAX RX to caller indication...\r\n");
              goto cfMatchedLogged;
            }
            goto cfMatchedQuiet;
          }
        }
        else if (rxCountOrWord == 0x109)
        {
          capByte = capByte & 2;
          goto cfTestBits;
        }
      }
      else
      {
        capOctet = *(char*)(caps + 0x18);
        capIdx = 0;
        capIdxDone = capIdx;
        if (capOctet != '\0')
        {
          do
          {
            while (1)
            {
              flippedWord = charFlip((unsigned char)capOctet);
              rxWord = MEMBER_AS_AT(unsigned short, p, msg[2].data[0], rxIdx * 2);
              if (rxWord != (unsigned short)((flippedWord & 0xff) * 2 | 1))
                break;
              p->callFnWord = rxWord;
              caps = V8_PGM(unsigned char*, V8Obj, p, capsPtr);
              p->callFnAccepted = 1;
              capIdxDone = capIdx + 1;
              rxIdx = rxIdx + 1;
              capOctet = *(char*)(caps + capIdx + 0x19);
              capIdx = capIdxDone;
              if (capOctet == '\0' || 3 < capIdxDone)
                goto rawCfCheck;
            }
            if (p->callFnAccepted != 0)
            {
              caps = V8_PGM(unsigned char*, V8Obj, p, capsPtr);
              capIdxDone = capIdx;
              break;
            }
            caps = V8_PGM(unsigned char*, V8Obj, p, capsPtr);
            capIdxDone = capIdx + 1;
            capOctet = *(char*)(caps + capIdx + 0x19);
            capIdx = capIdxDone;
          } while (capOctet != '\0' && capIdxDone < 4);
        }
      rawCfCheck:
        if ((*(char*)(caps + capIdxDone + 0x18) == '\0' || capIdxDone == 4) && (p->callFnAccepted != 0))
        {
          rxCountOrWord = p->msg[2].wordIdx;
          break;
        }
        p->callFnWord = 0;
        p->callFnAccepted = 0;
      }
      if (p->callFnAccepted != 0)
      {
        p->callFnWord = MEMBER_AS_AT(unsigned short, p, msg[2].data[0], rxIdx * 2);
      }
      rxCountOrWord = p->msg[2].wordIdx;
      rxIdx = rxIdx + 1;
    } while (rxIdx < rxCountOrWord);
  }
scanProtocols:
  rxIdx = 0;
  matched = 0;
  if (0 < rxCountOrWord)
  {
    do
    {
      while (rxWord = MEMBER_AS_AT(unsigned short, p, msg[2].data[0], rxIdx * 2), (rxWord & 0xfff1) != 0xa1)
      {
      nextProtoWord:
        rxIdx = rxIdx + 1;
        if (rxCountOrWord <= rxIdx)
          goto reportMatch;
      }
      caps = V8_PGM(unsigned char*, V8Obj, p, capsPtr);
      if ((*(unsigned char*)(caps + 2) & 8) == 0)
      {
        p->protoWord = rxWord;
        p->protoAccepted = 1;
        goto nextProtoWord;
      }
      capOctet = *(char*)(caps + 0x1c);
      capIdx = 0;
      capIdxDone = capIdx;
      if (capOctet != '\0')
      {
        do
        {
          while (1)
          {
            flippedWord = charFlip((unsigned char)capOctet);
            rxWord = MEMBER_AS_AT(unsigned short, p, msg[2].data[0], rxIdx * 2);
            if (rxWord != (unsigned short)((flippedWord & 0xff) * 2 | 1))
              break;
            p->protoWord = rxWord;
            capIdxDone = capIdx + 1;
            matched = 1;
            caps = V8_PGM(unsigned char*, V8Obj, p, capsPtr);
            rxIdx = rxIdx + 1;
            capOctet = *(char*)(caps + capIdx + 0x1d);
            capIdx = capIdxDone;
            if (capOctet == '\0' || 3 < capIdxDone)
              goto rawProtoCheck;
          }
          if (matched)
          {
            caps = V8_PGM(unsigned char*, V8Obj, p, capsPtr);
            capIdxDone = capIdx;
            break;
          }
          caps = V8_PGM(unsigned char*, V8Obj, p, capsPtr);
          capIdxDone = capIdx + 1;
          capOctet = *(char*)(caps + capIdx + 0x1d);
          capIdx = capIdxDone;
        } while (capOctet != '\0' && capIdxDone < 4);
      }
    rawProtoCheck:
      if ((*(char*)(caps + capIdxDone + 0x1c) == '\0' || capIdxDone == 4) && matched)
      {
        p->protoAccepted = 1;
        break;
      }
      rxCountOrWord = p->msg[2].wordIdx;
      rxIdx = rxIdx + 1;
      p->protoWord = 0;
    } while (rxIdx < rxCountOrWord);
  }
reportMatch:
  if (1 < dsplibs_debug_level)
  {
    suffixText = "";
    if (p->callFnAccepted == 0)
    {
      suffixText = " (not indicating modulation capabilities)!!";
    }
    gotText = "Got";
    if (p->callFnAccepted == 0)
    {
      gotText = "Didn't get";
    }
    dsplibs_debug_printf("V8: %s Call Function Match%s!\n", gotText, suffixText);
  }
}

/**
 * @brief Run the handshake for one block of 4 samples: top up the transmit ring, then process one received block.
 * @param p  the engine object
 * @return 0 normally, 1 when a timeout fired (the rx phase was set to the matching timeout phase), 2 when the handshake is finished (rx phase DONE)
 * @note  Called by V8Process() on every sample for which a block can be exchanged.  Two parts:
 *        Transmit: while the transmit ring holds fewer than `txQueueTarget` (16) samples, one group of 4 samples is generated by the TX phase
 *        (V8TxPhase): silence; ANSam (answerer, with the signal-detect timeout against `timeoutCount`); the V.21 FSK of the message at `txMsgPtr`
 *        (one bit per 32 samples, bits from v8_getbit(); with the message-detect timeout); the 60-bit quick-connect preamble; or the plain 980 Hz tone.
 *        Receive: only when at least 6 samples are queued.  The RX phase (V8RxPhase) selects: the answerer's AGC settling (601 blocks); the
 *        caller's ANSam search and analysis (sub-states V8SUB_ANSAM_WAIT / V8SUB_ANSAM_SEEN: tone detection, then 2400 blocks of gain stability, 15 Hz
 *        modulation energy and phase-reversal observation before CM is started); the V.21 message reception (V8RX_MESSAGE, sub-states of V8RxSub:
 *        sync word search, collecting the words of CM / JM until two successive passes agree, QCA1 reception, CJ sending and waiting); the idle and
 *        terminal phases.
 * @warning A TX or RX phase word outside the values listed in v8_phase.h calls abort() (docs/bugs.md); the engine only stores valid values.
 */

unsigned int v8handshak(V8Obj* p)
{
  int i;

  for (;;)
  {
    if (p->txQueueCount >= p->txQueueTarget)
    {
      break;
    }
    switch (MEMBER_AS(unsigned short, p, phasePair))
    {
    case V8TX_SILENCE:
    {
      short* txBuf = (short*)(MEMBER_ADDR(p, txBuf[0]));
      for (i = 0; i < 4; i++)
        txBuf[i] = 0;
      v8_txwritequeue(p);
      break;
    }
    case V8TX_ANSAM:
    {
      /* Answerer: ANSam while waiting for the caller's CM.  On timeout (the signal-detect budget against `timeoutCount`) the rx phase becomes
       * V8RX_TIMEOUT_CM and the call returns 1.  Otherwise 4 samples: the 2100 Hz carrier times (1 + 0.2 * 15 Hz cosine) times the amplitude, through
       * v8_fsktxfilter(); the amplitude changes sign every 0x438 calls (the 450 ms phase reversal). */
      int limit = p->sigTimeoutTicks;
      if (limit != -1)
      {
        int cnt = p->timeoutCount;
        if (limit <= cnt)
        {
          if (cnt == limit)
          {
            if (1 < dsplibs_debug_level)
            {
              dsplibs_debug_printf("V8: Time Out Waiting For CM...\r\n");
              cnt = p->timeoutCount;
            }
            p->timeoutCount = cnt + 1;
          }
          MEMBER_AS_AT(unsigned short, p, phasePair, 2) = V8RX_TIMEOUT_CM;
          return 1;
        }
      }
      {
        short* txBuf = (short*)(MEMBER_ADDR(p, txBuf[0]));
        int k;
        for (k = 0; k < 4; k++)
        {
          unsigned int acc1 = ((unsigned int)p->amPhase + (unsigned int)p->amStep) & 0x3fff;
          int acc2 = (p->tonePhase + p->toneStep) & 0x3fff;
          short tone, mod;
          p->amPhase = (unsigned short)acc1;
          p->tonePhase = (unsigned short)acc2;
          tone = v8_cosread((unsigned char)((acc1 + 0x20) >> 6));
          tone = (short)v8_mpyint(0xccd, tone);
          tone = (short)v8_mpyint((short)(tone + 0x4000), p->ansamAmp);
          mod = v8_cosread((unsigned char)((acc2 + 0x20) >> 6));
          tone = (short)v8_mpyint(mod, tone);
          txBuf[k] = v8_fsktxfilter(p, tone);
        }
        if (p->reverseEnable != 0)
        {
          short cnt2 = p->reverseCount + 1;
          if (cnt2 == 0x438)
          {
            p->reverseCount = 0;
            p->ansamAmp = -p->ansamAmp;
          }
          else
          {
            p->reverseCount = cnt2;
          }
        }
        v8_txwritequeue(p);
        p->timeoutCount += 1;
      }
      break;
    }
    case V8TX_MESSAGE:
    {
      /* FSK transmission of the message at `txMsgPtr`.  The message-detect budget (`msgTimeoutTicks` against `timeoutCount`) bounds the wait for the
       * answer: on expiry the rx phase becomes TIMEOUT_CJ (answerer) or TIMEOUT_JM (caller) and the call returns 1; the tx phase stays, so the next call
       * takes the same branch.  Otherwise one 4-sample slice of the current bit is modulated, and after 32 samples the next bit is fetched
       * with v8_getbit(). */
      int limit = p->msgTimeoutTicks;
      if (limit != -1)
      {
        int cnt = p->timeoutCount;
        if (limit <= cnt)
        {
          /* the rx phase is set at once; both branches then return 1 without reaching the rx dispatch of this call */
          MEMBER_AS_AT(unsigned short, p, phasePair, 2) = (p->side == 1) ? V8RX_TIMEOUT_CJ : V8RX_TIMEOUT_JM;
          if (cnt != limit)
          {
            return 1;
          }
          if (1 < dsplibs_debug_level)
          {
            /* the answerer (side 1) waits for the caller's CJ, the caller (side 0) for the answerer's JM */
            const char* what = (p->side == 1) ? "CJ" : "JM";
            dsplibs_debug_printf("V8: Timeout waiting for %s message...\r\n", what);
            cnt = p->timeoutCount;
          }
          p->timeoutCount = cnt + 1;
          return 1;
        }
      }
      v8_fskmodulate(p, p->txBit);
      {
        short cnt2 = (short)(p->txBitPos + 4);
        p->txBitPos = cnt2;
        if (p->bitSamples == cnt2)
        {
          unsigned int bit = v8_getbit(V8_PGM(void*, V8Obj, p, txMsgPtr));
          p->txBit = (unsigned short)bit;
          p->txBitPos = 0;
        }
      }
      p->timeoutCount += 1;
      break;
    }
    case V8TX_QC_PREAMBLE:
    {
      /* The same FSK tick for a fixed 60 bit periods (the quick-connect preamble); then the message phase starts with msg[0] as the message. */
      v8_fskmodulate(p, p->txBit);
      {
        short cnt2 = (short)(p->txBitPos + 4);
        p->txBitPos = cnt2;
        if (p->bitSamples == cnt2)
        {
          unsigned int bit = v8_getbit(V8_PGM(void*, V8Obj, p, txMsgPtr));
          short cnt3 = (short)(p->qcPreambleCount + 1);
          p->txBit = (unsigned short)bit;
          p->qcPreambleCount = cnt3;
          if (cnt3 == 0x3c)
          {
            MEMBER_AS(unsigned short, p, phasePair) = V8TX_MESSAGE;
            V8_PSM(void*, V8Obj, p, txMsgPtr, MEMBER_ADDR(p, msg[0]));
          }
          p->txBitPos = 0;
        }
      }
      break;
    }
    case V8TX_TONEQ:
    {
      /* The plain 980 Hz tone (no amplitude envelope, no filter), sent by the caller after a good QCA1a. */
      short* txBuf = (short*)(MEMBER_ADDR(p, txBuf[0]));
      int k;
      for (k = 0; k < 4; k++)
      {
        unsigned int acc = ((unsigned int)p->toneqPhase + (unsigned int)p->toneqStep) & 0x3fff;
        p->toneqPhase = (unsigned short)acc;
        txBuf[k] = v8_cosread((unsigned char)((acc + 0x20) >> 6));
      }
      v8_txwritequeue(p);
      break;
    }
    default:
      abort(); /* a TX phase that is not one of the values above: the engine never stores one */
    }
  }

  if (p->rxQueueCount < 6)
  {
    return 0;
  }

  switch (MEMBER_AS_AT(unsigned short, p, phasePair, 2))
  {
  case V8RX_AGC_SETTLE:
  {
    /* Answerer: let the AGC settle for 601 blocks, then start listening for the caller's CM (RX message phase, sub-state "searching sync"). */
    short cnt;
    V8agc(p);
    cnt = p->stepCount + 1;
    p->stepCount = cnt;
    if (cnt <= 0x258)
    {
      return 0;
    }
    MEMBER_AS_AT(unsigned short, p, phasePair, 2) = V8RX_MESSAGE;
    p->rxSubState = V8SUB_SYNC;
    p->gainAdaptStep = 0x800;
    p->stepCount = 0;
    return 0;
  }
  case V8RX_ANSAM:
  {
    V8agc(p);
    if (p->rxSubState == V8SUB_ANSAM_WAIT)
    {
      int limit, cnt;

      limit = p->sigTimeoutTicks;
      if (limit == -1)
      {
        cnt = p->timeoutCount;
      }
      else
      {
        cnt = p->timeoutCount;
        if (limit <= cnt)
        {
          if (cnt == limit)
          {
            if (1 < dsplibs_debug_level)
            {
              dsplibs_debug_printf("V8: Time Out Waiting For ANSam...\r\n");
              cnt = p->timeoutCount;
            }
            p->timeoutCount = cnt + 1;
          }
          MEMBER_AS_AT(unsigned short, p, phasePair, 2) = V8RX_TIMEOUT_ANSAM;
          return 1;
        }
      }
      p->timeoutCount = cnt + 1;
      if (v8_tone_detect(p, &p->det, (short*)(MEMBER_ADDR(p, rxBuf[0]))) != 0)
      {
        p->rxSubState = V8SUB_ANSAM_SEEN;
        p->stepCount = 0;
        p->timeoutCount = 0;
        p->flags &= 0xfdff;
      }
      return 0;
    }

    /* Sub-state V8SUB_ANSAM_SEEN: a candidate ANSam has been detected.  For 2400 blocks gain stability, the 15 Hz modulation of the rectified
     * signal (single-bin DFT) and the phase reversals are observed.  Then, if neither the modulation energy (< 400) nor a phase reversal was
     * found, it was not an ANSam: the call returns 1 (and with a quick-connect QCA1a already seen, the handshake ends or the tone stops).
     * Otherwise the caller starts its CM (V.21 low channel out, high channel in; with quick connect the QC preamble first) when CM sending is
     * enabled, or sends the plain tone when a QCA1a was seen and quick connect is enabled. */
    {
      short absSamp[4];
      int k;
      short cnt;
      const unsigned char* caps;
      unsigned int ansamCond;

      for (k = 0; k < 4; k++)
      {
        absSamp[k] = (short)v8_absfn(p->rxBuf[k]);
      }
      checkSignalStability(p);
      if (p->gainStable != 0)
      {
        v8_dftupdate(&p->dft, 1, absSamp, 4);
      }
      v8_phase_rev_detect(MEMBER_ADDR(p, phaseRev[0]), (const short*)MEMBER_ADDR(p, rxBuf[0]), 4);
      cnt = (short)(p->stepCount + 1);
      p->stepCount = cnt;
      if (cnt <= 0x960)
      {
        return 0;
      }

      p->flags |= 0x200;
      if (p->gainStable != 0)
      {
        v8_dftenergy(&p->dft, 1, 1);
      }

      caps = V8_PGM(const unsigned char*, V8Obj, p, capsPtr);
      ansamCond = (unsigned int)(p->qca1aSeen & ((caps[2] >> 4) & 1));

      if (p->dft.energy < 400 && MEMBER_AS(short, p, phaseRev[220]) == 0)
      {
        if (ansamCond != 0)
        {
          if (MEMBER_AS(short, p, phasePair) == V8TX_TONEQ)
          {
            /* the terminal transition (also taken by the lost-sync timeout and by the CJ wait) */
            MEMBER_AS(unsigned short, p, phasePair) = V8TX_SILENCE;
            MEMBER_AS_AT(unsigned short, p, phasePair, 2) = V8RX_DONE;
            return 2;
          }
          MEMBER_AS(unsigned short, p, phasePair) = V8TX_SILENCE;
        }
        return 1;
      }

      if (ansamCond == 0)
      {
        if (p->cmEnabled != 0)
        {
          if (1 < dsplibs_debug_level)
          {
            dsplibs_debug_printf("V8 ANSAM Detected (CM ready)\n");
          }
          v8_V21_Init(p, 0, 1);
          p->gainAdaptStep = 0x800;
          MEMBER_AS(unsigned short, p, phasePair) = V8TX_MESSAGE;
          p->txBit = 1;
          p->stepCount = 0;
          p->wdb4 = 0;
          MEMBER_AS_AT(unsigned short, p, phasePair, 2) = V8RX_MESSAGE;
          p->rxSubState = V8SUB_SYNC;
          if ((caps[2] & 0x10) == 0)
          {
            MEMBER_AS(unsigned short, p, phasePair) = V8TX_MESSAGE;
          }
          else
          {
            MEMBER_AS(unsigned short, p, phasePair) = V8TX_QC_PREAMBLE;
          }
        }
      }
      else
      {
        MEMBER_AS(unsigned short, p, phasePair) = V8TX_TONEQ;
      }
      return 0;
    }
  }
  case V8RX_TIMEOUT_ANSAM:
    return 0;
  case V8RX_DRAIN: /* only drains the receive ring (V8SUB_SEND_CJ is a different field that shares the value) */
    v8_rxreadqueue(p);
    return 0;
  case V8RX_DONE: /* the terminal "handshake finished" phase */
    return 2;
  case V8RX_MESSAGE:
  {
    /* One block of the V.21 message reception.  V8agc() and v8_fskdemodulate() run first; the newly demodulated bits update the run counters
     * zeroRun / oneRun / lastOneRun (consecutive 0 / 1 bits and the last run of 1 bits) and `preambleEnds` (six 0 bits after a run of more than nine
     * 1 bits, the end of a preamble); from the second preamble end on, the bit counter and shift register are re-aligned (bit count 6) unless the
     * answerer waits for CJ.  Then the sub-state (V8RxSub) selects:
     * - V8SUB_SYNC: look for the 12-bit sync word 0xc0f (start of a CM / JM; the word buffer at `rxMsgPtr` is seeded and the sub-state becomes
     *   V8SUB_COLLECT) or 0xd55 (a QCA1 word, only with quick connect enabled; sub-state V8SUB_QCA1, buffer `qcRxMsgPtr`);
     * - V8SUB_SEND_CJ: when the CM is finished (the slot's bitsInWord is 0) switch the transmitter to the CJ slot msg[1]; the handshake ends
     *   0x104 calls later;
     * - after each complete 10-bit word: V8SUB_COLLECT compares the words of the repeated CM / JM; V8SUB_QCA1 checks the last six words for a good
     *   QCA1; the remaining sub-states (V8SUB_WAIT_CJ) count runs of nine 0 bits.
     * V8SUB_JM_DETECTED and V8SUB_CM_DETECTED do nothing here (the host acts through V8Control()). */
    short cOld, cNew, subState;

    V8agc(p);
    cOld = p->rxBitCount;
    v8_fskdemodulate(p);
    cNew = p->rxBitCount;

    if (cNew > cOld)
    {
      unsigned short bits = p->rxShiftReg;
      int count = cNew - cOld;
      int k;
      for (k = 0; k < count; k++)
      {
        int bitPos = count - k - 1;
        int bit = (bits >> bitPos) & 1;
        if (bit)
        {
          short c40 = (short)(p->oneRun + 1);
          p->zeroRun = 0;
          p->oneRun = c40;
          p->lastOneRun = c40;
        }
        else
        {
          short c3e = (short)(p->zeroRun + 1);
          p->zeroRun = c3e;
          if (c3e == 6 && p->lastOneRun > 9)
          {
            short c44 = p->preambleEnds;
            p->preambleEnds = (short)(c44 + 1);
            p->preambleEndsHandled = c44;
          }
          p->oneRun = 0;
        }
      }
    }

    {
      short c46 = p->preambleEndsHandled;
      short c44 = p->preambleEnds;
      if (c46 != c44 && c44 >= 2)
      {
        subState = MEMBER_AS(short, p, rxSubState);
        if (subState != V8SUB_WAIT_CJ)
        {
          p->rxBitCount = 6;
          p->rxShiftReg = 0;
          p->preambleEndsHandled = (short)(c46 + 1);
        }
      }
      else
      {
        subState = MEMBER_AS(short, p, rxSubState);
      }
    }

    if (subState == V8SUB_SYNC)
    {
      unsigned short word12 = p->rxShiftReg & 0xfff;
      if (word12 == 0xc0f)
      {
        unsigned short* w = V8_PGM(unsigned short*, V8Obj, p, rxMsgPtr);
        int j;
        w[0] = 0xf;
        for (j = 1; j < 0xf; j++)
          w[j] = 0x3ff;
        p->rxSubState = V8SUB_COLLECT;
        p->stepCount = 1;
        MEMBER_AS(short, p, rxWordIdx) = 1;
        w[0x14] = USHRT_MAX;
        p->rxBitCount = 0;
        p->rxShiftReg = 0;
      }
      else if (word12 == 0xd55 && (*(const unsigned char*)(V8_PGM(char*, V8Obj, p, capsPtr) + 2) & 0x10) != 0)
      {
        unsigned short* w = V8_PGM(unsigned short*, V8Obj, p, qcRxMsgPtr);
        int j;
        w[0] = 0x155;
        for (j = 1; j <= 5; j++)
          w[j] = 0x3ff;
        p->rxSubState = V8SUB_QCA1;
        p->stepCount = 1;
        MEMBER_AS(short, p, rxWordIdx) = 1;
        w[0x14] = USHRT_MAX;
        p->rxBitCount = 0;
        p->rxShiftReg = 0;
      }
      return 0;
    }

    if (subState == V8SUB_SEND_CJ)
    {
      if (p->stepCount == 0)
      {
        char* slot = V8_PGM(char*, V8Obj, p, txMsgPtr);
        if (*(short*)(slot + 0x34) == 0)
        {
          V8_PSM(char*, V8Obj, p, txMsgPtr, MEMBER_ADDR(p, msg[1]));
          p->stepCount = 1;
        }
      }
      else
      {
        short cnt = (short)(p->stepCount + 1);
        p->stepCount = cnt;
        if (cnt == 0x104)
        {
          MEMBER_AS(unsigned short, p, phasePair) = V8TX_SILENCE;
          MEMBER_AS_AT(unsigned short, p, phasePair, 2) = V8RX_DONE;
          return 2;
        }
      }
      return 0;
    }

    if (p->rxBitCount == 10)
    {
      p->rxBitCount = 0;
      /* the complete 10-bit word, for whichever sub-state follows */
      unsigned int wordVal = (unsigned int)MEMBER_AS(short, p, rxShiftReg) & 0x3ff;
      short word10 = (short)wordVal;

      if (subState == V8SUB_COLLECT)
      {
        /* Collect the words of the repeated CM / JM.  The word buffer (`rxMsgPtr`) holds the candidate message of the previous pass, ended
         * by the marker word 0xf (a sync word) and its word count in element 0x14.  A decoded 0xf closes a pass: if the number of words that
         * matched the previous pass equals the stored count, the message is accepted (answerer: build the JM, caller: evaluate the JM; the
         * sub-state moves on according to opMode); otherwise the pass becomes the new candidate.  Any other word is compared with the stored
         * one; a mismatch stores it (up to 15 words) and restarts the match count. */
        unsigned short* wbuf = V8_PGM(unsigned short*, V8Obj, p, rxMsgPtr);
        if (word10 == 0xf)
        {
          if (wbuf[0x14] == p->stepCount)
          {
            if (p->side == 1)
            {
              if (p->opMode == 1)
              {
                p->rxSubState = V8SUB_CM_DETECTED;
              }
              else
              {
                p->rxSubState = V8SUB_WAIT_CJ;
                MEMBER_AS(unsigned short, p, phasePair) = V8TX_MESSAGE;
                p->timeoutCount = 0;
              }
              rebuildJMSequence(p);
              p->wdb4 = 0;
              {
                unsigned char* seq = V8_PGM(unsigned char*, V8Obj, p, txMsgPtr);
                p->rxWordIdx = 0;
                p->txBit = 1;
                *(unsigned int*)(seq + 0x30) = 0;
                *(unsigned short*)(seq + 0x34) = 0;
              }
            }
            else
            {
              evaluateRxJMSequence(p);
              if (p->opMode == 1)
              {
                p->rxSubState = V8SUB_JM_DETECTED;
              }
              else
              {
                p->rxSubState = V8SUB_SEND_CJ;
              }
            }
            p->flags = p->flags | 0x200;
            p->wa40 = p->agcGain;
            p->stepCount = 0;
          }
          else
          {
            wbuf[0] = 0xf;
            wbuf[0x14] = p->rxWordIdx;
            p->stepCount = 1;
            p->rxWordIdx = 1;
          }
        }
        else
        {
          short dbc = MEMBER_AS(short, p, rxWordIdx);
          if (*(unsigned short*)((char*)wbuf + dbc * 2) == wordVal)
          {
            MEMBER_AS(short, p, rxWordIdx) = dbc + 1;
            p->stepCount = p->stepCount + 1;
          }
          else
          {
            if (dbc < 0xf)
            {
              *(short*)((char*)wbuf + dbc * 2) = word10;
              MEMBER_AS(short, p, rxWordIdx) = MEMBER_AS(short, p, rxWordIdx) + 1;
            }
            p->stepCount = 0;
          }
        }
      }
      else if (subState == V8SUB_QCA1)
      {
        /* Quick-connect word detector: `qcRxMsgPtr` collects the six words of a QCA1 message.  With six words, the message is good when the
         * last word has the tag bits 0x3f0 set, the second word is a QCA1a or QCA1d pattern, the third and fourth words are 0x3ff and 0x155 and
         * the fifth equals the second.  A good QCA1d (digital connection) ends the handshake with the ANSpcm level index; a good QCA1a
         * (analogue) returns to the ANSam search with `qca1aSeen` set; anything else restarts the sync search. */
        unsigned short* wbuf = V8_PGM(unsigned short*, V8Obj, p, qcRxMsgPtr);
        short dbc = MEMBER_AS(short, p, rxWordIdx);
        MEMBER_AS(short, p, rxWordIdx) = dbc + 1;
        wbuf[dbc] = word10;
        if (MEMBER_AS(short, p, rxWordIdx) == 6)
        {
          unsigned short w1 = wbuf[1];
          if ((*(unsigned short*)((char*)wbuf + 0xa) & 0x3f0) == 0x3f0 && ((w1 & 0x3b9) == 0x181 || (w1 & 0x391) == 0x81) &&
              *(unsigned int*)((char*)wbuf + 4) == 0x15503ff && wbuf[4] == w1)
          {
            MEMBER_AS(int, p, qcDetected) = 1;
            p->qcLapm = (unsigned int)(w1 >> 6) & 1;
            if ((w1 & 0x3b9) == 0x181)
            {
              /* QCA1d: digital connection indicated: terminal */
              if (1 < dsplibs_debug_level)
              {
                dsplibs_debug_printf("V8:  QCA1d: Got Good QCA1d !!!!\r\n");
                wbuf = V8_PGM(unsigned short*, V8Obj, p, qcRxMsgPtr);
              }
              {
                unsigned short w4s = (unsigned short)(wbuf[4] >> 1);
                unsigned short w1s = (unsigned short)(wbuf[1] >> 1);
                p->ansPcmLevel = (unsigned int)((w1s & 2) + (w1s & 1));
                if (1 < dsplibs_debug_level)
                {
                  dsplibs_debug_printf("V8:  QCA1d: ANSpcm level index: bits27-28 = %d, bits57-58 = %d\r\n", (w1s & 2) + (w1s & 1), (w4s & 2) + (w4s & 1));
                }
              }
              MEMBER_AS(unsigned short, p, phasePair) = V8TX_SILENCE;
              MEMBER_AS_AT(unsigned short, p, phasePair, 2) = V8RX_DONE;
              return 2;
            }
            /* QCA1a: analogue connection indicated: back to the ANSam search */
            if (1 < dsplibs_debug_level)
            {
              dsplibs_debug_printf("V8:  QCA1a: Got Good QCA1a !!!!\r\n");
              if (1 < dsplibs_debug_level)
              {
                unsigned short* wbuf2 = V8_PGM(unsigned short*, V8Obj, p, qcRxMsgPtr);
                dsplibs_debug_printf("V8:  QCA1a: U_QTS: bits24,26-28 = %d%d%d%d, bits54,56-58 = %d%d%d%d\r\n", (wbuf2[1] >> 5) & 1, (wbuf2[1] >> 3) & 1,
                                     (wbuf2[1] >> 2) & 1, (wbuf2[1] >> 1) & 1, (wbuf2[4] >> 5) & 1, (wbuf2[4] >> 3) & 1, (wbuf2[4] >> 2) & 1,
                                     (wbuf2[4] >> 1) & 1);
              }
            }
            MEMBER_AS(unsigned short, p, phasePair) = V8TX_SILENCE;
            p->rxSubState = V8SUB_ANSAM_WAIT;
            MEMBER_AS_AT(unsigned short, p, phasePair, 2) = V8RX_ANSAM;
            p->qca1aSeen = 1;
            if (1 < dsplibs_debug_level)
            {
              unsigned short* wbuf3 = V8_PGM(unsigned short*, V8Obj, p, qcRxMsgPtr);
              dsplibs_debug_printf("V8:  QCA1d: LAPM Indication: bit23 = %d, bit53 = %d\r\n", (wbuf3[1] >> 6) & 1, (wbuf3[4] >> 6) & 1);
            }
          }
          else
          {
            if (1 < dsplibs_debug_level)
            {
              dsplibs_debug_printf("V8: reseting QCA1 detector...\r\n");
            }
            p->rxSubState = V8SUB_SYNC;
          }
        }
      }
      else if ((unsigned short)(subState - 0x32) > 1)
      {
        /* Waiting for CJ (answerer): scan the 10 bits MSB first for runs of exactly nine 0 bits followed by a 1 bit (a zero octet); the second
         * such run ends the handshake. */
        unsigned int mask = 0x200;
        int bitIdx;
        for (bitIdx = 0; bitIdx < 10; bitIdx++, mask >>= 1)
        {
          if ((mask & (unsigned int)(unsigned short)word10) == 0)
          {
            p->zeroBits += 1;
          }
          else
          {
            if (p->zeroBits == 9)
            {
              short cnt = (short)(p->stepCount + 1);
              p->stepCount = cnt;
              if (cnt == 2)
              {
                MEMBER_AS(unsigned short, p, phasePair) = V8TX_SILENCE;
                MEMBER_AS_AT(unsigned short, p, phasePair, 2) = V8RX_DONE;
                return 2;
              }
            }
            p->zeroBits = 0;
          }
        }
      }
    }
    return 0;
  }
  case V8RX_TIMEOUT_CM:
    /* set by the ANSam transmit branch on a timeout; nothing more to do */
    return 0;
  default:
    abort(); /* an RX phase that is not one of the values above: the engine never stores one */
  }
}

/* ---- the engine object API ---- */

/* the names printed by the debug output of V8Process() ("State changed from %s to %s") and V8Control(); indexed by V8Status / V8CtrlType */

static const char* const v8StatusName[19] = {
    "V8_INIT",
    "V8_ANS_SEND_ANSAM",
    "V8_ANS_CM_DETECTED",
    "V8_ANS_SEND_JM",
    "V8_ANS_TIME_OUT_WAITING_FOR_CM",
    "V8_ANS_TIME_OUT_WAITING_FOR_CJ",
    "V8_ORG_WAITING_FOR_ANSAM",
    "V8_ORG_ANSAM_DETECTED_WAITING_TE",
    "V8_ORG_SEND_CM",
    "V8_ORG_JM_DETECTED",
    "V8_ORG_SEND_CJ",
    "V8_ORG_TIME_OUT_WAITING_FOR_ANSAM",
    "V8_ORG_TIME_OUT_WAITING_FOR_JM",
    "V8_OK",
    "V8_ORG_SEND_QC",
    "V8_ORG_WAITING_FOR_QCA1d",
    "V8_ORG_BAD_QCA1d_MESSAGE",
    "V8_ORG_TIME_OUT_WAITING_FOR_QCA1d",
    "V8_LAST_ENUM",
};
static const char* const v8ControlName[11] = {
    "V8CTRL_START_CM", "V8CTRL_START_CJ", "V8CTRL_START_JM", "V8CTRL_CTRL3", "V8CTRL_CTRL4",  "V8CTRL_CTRL5",
    "V8CTRL_CTRL6",    "V8CTRL_CTRL7",    "V8CTRL_CTRL8",    "V8CTRL_CTRL9", "V8CTRL_CTRL10",
};

/**
 * @brief Create a V.8 engine object and start the handshake.
 * @param cfg  the configuration: side (0 = caller, 1 = answerer), opMode (0 = the CM / JM is sent as soon as the tone is seen, 1 = the host
 *             releases it through V8Control()), the signal-detect and message-detect timeouts in seconds, and the capability block
 * @return the engine (state V8_INIT), or NULL when the allocation failed
 * @note  With debug level 2 the whole configuration is printed ("V8: local configuration").  The object is released with V8Delete().
 * @warning `cfg` and `cfg->caps` must be valid; the capability block is written by the engine and must outlive it.  The object is not cleared
 *          before v8handshakinit() runs (see v8_rxinit()).
 */

void* V8Create(const V8Cfg* cfg)
{
  V8Obj* p = (V8Obj*)sysdep_malloc(V8_ALLOC_SIZE);
  const unsigned char* cap;
  if (p == NULL)
  {
    return NULL;
  }

  /* the configuration */
  p->side = cfg->side;
  p->opMode = cfg->opMode;
  p->sigTimeout = cfg->sigTimeout;
  p->msgTimeout = cfg->msgTimeout;
  p->cfgPad = cfg->pad;
  V8_PSM(const unsigned char*, V8Obj, p, capsPtr, cfg->caps);
  cap = V8_PGM(const unsigned char*, V8Obj, p, capsPtr);
  if (1 < dsplibs_debug_level)
  {
    dsplibs_debug_printf("V8: Create called, V8 version 23/09/03 .\r\n");
    dsplibs_debug_printf("############################################################\r\n");
    dsplibs_debug_printf("V8: local configuration : \r\n");
    dsplibs_debug_printf("\tSide = %s\r\n", p->side != 0 ? "Answer" : "Caller");
    dsplibs_debug_printf("\tOperation Mode = %d\r\n", p->opMode);
    dsplibs_debug_printf("\tModulations - V90=%d, V34=%d, V34HD=%d, V32=%d, V22=%d, V17=%d, V29=%d, V27=%d, V23=%d, V21=%d\r\n", (cap[0] >> 3) & 1,
                         (cap[0] >> 5) & 1, (cap[0] >> 6) & 1, cap[0] >> 7, cap[1] & 1, (cap[1] >> 1) & 1, (cap[1] >> 2) & 1, (cap[1] >> 3) & 1,
                         (cap[1] >> 4) & 1, (cap[1] >> 5) & 1);
    if ((cap[2] & 4) == 0)
    {
      dsplibs_debug_printf("\tCall Functions - Data=%d, CallRxFax=%d, CallTxFax=%d, V.80=%d\r\n", (cap[1] >> 6) & 1, cap[1] >> 7, cap[2] & 1,
                           (cap[2] >> 1) & 1);
    }
    else
    {
      dsplibs_debug_printf("\tCall Functions - raw CF specified: cf[0]=%d , cf[1]=%d , cf[2]=%d , cf[3]=%d\r\n", cap[0x18], cap[0x19], cap[0x1a], cap[0x1b]);
    }
    if ((cap[2] & 8) != 0)
    {
      dsplibs_debug_printf("\tProtocol - raw Protocol specified: prot[0]=%d , prot[1]=%d , prot[2]=%d , prot[3]=%d\r\n", cap[0x1c], cap[0x1d], cap[0x1e],
                           cap[0x1f]);
    }
    else
    {
      dsplibs_debug_printf("\tProtocol - LAPM V.42\r\n");
    }
    dsplibs_debug_printf("\tv8bisIndication - %d\r\n", (cap[0] >> 1) & 1);
    dsplibs_debug_printf("\ttimeouts - signal detect %d sec, message detect %d sec\r\n", p->sigTimeout, p->msgTimeout);
    dsplibs_debug_printf("\tquickConnectEnabled - %d\r\n", (cap[2] >> 4) & 1);
    dsplibs_debug_printf("\tlapmIndication - %d\r\n", (cap[2] >> 6) & 1);
    dsplibs_debug_printf("\tucodeForQts - %d\r\n", *(const unsigned int*)(cap + 0x10));
    dsplibs_debug_printf("\tansPcmLevel - %d\r\n", *(const unsigned int*)(cap + 0xc));
    dsplibs_debug_printf("############################################################\r\n");
  }

  p->txGain = 0x4000;
  v8handshakinit(p);
  p->dcState = 0;
  p->lastStatus = V8_INIT;
  return p;
}

/** @brief Free a V.8 engine object (NULL is ignored). */

void V8Delete(void* v8)
{
  if (v8 != NULL)
  {
    sysdep_free(v8);
  }
}

/**
 * @brief Host control of the handshake.
 * @param p         the engine object
 * @param ctrlType  V8CTRL_START_CM: release the CM the caller holds back when opMode != 0 (caller, ANSam search phase only); V8CTRL_START_CJ:
 *                  continue from "JM detected" into sending CJ; V8CTRL_START_JM: continue from "CM detected" into sending the JM.  Other types
 *                  are not supported.
 * @return 0 when the transition was made, UINT_MAX when the type is unsupported or the engine is in the wrong state
 * @note  A line "V8: V8Control called - control type is ..." is printed at debug level 2.
 */

unsigned int V8Control(V8Obj* p, int ctrlType)
{
  unsigned int ret = UINT_MAX;
  int printIt = 0;

  if (ctrlType == V8CTRL_START_CJ)
  {
    if (MEMBER_AS(short, p, rxSubState) == V8SUB_JM_DETECTED)
    {
      p->rxSubState = V8SUB_SEND_CJ;
      ret = 0;
    }
  }
  else if (ctrlType == V8CTRL_START_CM)
  {
    if (p->side == 0 && MEMBER_AS_AT(short, p, phasePair, 2) == V8RX_ANSAM && p->cmEnabled == 0)
    {
      p->cmEnabled = 1;
      ret = 0;
    }
  }
  else if (ctrlType == V8CTRL_START_JM)
  {
    if (MEMBER_AS(short, p, rxSubState) == V8SUB_CM_DETECTED)
    {
      p->rxSubState = V8SUB_WAIT_CJ;
      p->timeoutCount = 0;
      MEMBER_AS(unsigned short, p, phasePair) = V8TX_MESSAGE;
      ret = 0;
    }
  }
  else
  {
    if (1 < dsplibs_debug_level)
    {
      dsplibs_debug_printf("V8: V8Control called with currently not supported control type (type=%d)\r\n", ctrlType);
    }
    return UINT_MAX;
  }
  printIt = 1 < dsplibs_debug_level;
  if (printIt)
  {
    dsplibs_debug_printf("V8: V8Control called - control type is %s\n", v8ControlName[ctrlType]);
  }
  return ret;
}

/**
 * @brief Process samples through the V.8 engine: output the transmit signal, take in the received signal, run the handshake.
 * @param p             the engine object
 * @param samplesIn     received samples (9600 Hz)
 * @param samplesOut    transmit samples, one per input sample
 * @param sampleCount   number of samples
 * @return the V8Status that describes the state of the handshake after the call (V8_OK once the handshake finished during this call)
 * @note  Per sample: one sample leaves the transmit ring; the input sample passes a DC-removal filter and enters the receive ring; whenever the
 *        transmit ring is nearly empty or 5 samples are queued, v8handshak() runs once.  If it reports the end of the handshake during the call,
 *        the result is V8_OK.  The status is derived from the TX phase, RX phase and sub-state (for the caller the packed phase pair
 *        0x190005 means "listening for ANSam with TX silence"); a change is printed at debug level 2 as "State changed from ... to ...".
 */

unsigned int V8Process(V8Obj* p, const short* samplesIn, short* samplesOut, int sampleCount)
{
  int completed = 0; /* set when v8handshak() reports the end of the handshake: the result is then V8_OK whatever the phase words say */
  int status = 0;

  if (0 < sampleCount)
  {
    for (;;)
    {
      for (;;)
      { /* one sample per iteration */
        short* txPtr;
        short* rxPtr;
        short sample;
        short dcOut;
        short ringCount;

        /* transmit: take one sample from the transmit ring */
        MEMBER_AS(short, p, txQueueCount) -= 1;

        txPtr = V8_PGM(short*, V8Obj, p, txRingRdPtr);
        *samplesOut = *txPtr;
        samplesOut = samplesOut + 1;
        txPtr = txPtr + 1;
        if ((short*)(MEMBER_ADDR(p, txBuf[0])) <= txPtr)
        {
          txPtr = (short*)(MEMBER_ADDR(p, txRing[0]));
        }
        V8_PSM(short*, V8Obj, p, txRingRdPtr, txPtr);

        /* receive: DC-removal filter, then into the receive ring (every second word; the other is 0) */
        sample = *samplesIn;
        samplesIn = samplesIn + 1;
        dcOut = (short)(sample + p->dcState);
        p->dcState = (short)((((int)dcOut * 0xf85) - ((int)sample << 0xc)) >> 0xc);

        rxPtr = V8_PGM(short*, V8Obj, p, rxRingWrPtr);
        rxPtr[0] = dcOut;
        rxPtr[1] = 0;
        rxPtr = rxPtr + 2;
        if ((short*)(MEMBER_ADDR(p, txQueueCount)) <= rxPtr)
        {
          rxPtr = (short*)(MEMBER_ADDR(p, rxRing[0]));
        }
        V8_PSM(short*, V8Obj, p, rxRingWrPtr, rxPtr);

        /* run the handshake when the transmit ring is nearly empty or a block of received samples is queued */
        ringCount = (short)(p->rxQueueCount + 1);
        p->rxQueueCount = ringCount;

        if (MEMBER_AS(short, p, txQueueCount) < 5 || 4 < ringCount)
        {
          status = (int)(short)v8handshak(p);
          if (status == 2)
          {
            break; /* the handshake is finished: count this sample, then continue with the rest */
          }
        }
        sampleCount = sampleCount - 1;
        if (sampleCount == 0)
        {
          goto after_loop;
        }
      }
      sampleCount = sampleCount - 1;
      completed = 1;
      if (sampleCount == 0)
      {
        break;
      }
    }
  }

after_loop:
  /* map the phase words to the status */
  status = V8_INIT;
  if (p->side == 0)
  {
    int combined = MEMBER_AS(int, p, phasePair); /* packed tx-phase(low16)/rx-phase(high16) */
    short subState;
    if (combined == 0x190005)
    {
      if (MEMBER_AS(short, p, rxSubState) == V8SUB_ANSAM_WAIT)
      {
        status = V8_ORG_WAITING_FOR_ANSAM;
        goto haveStatus;
      }
      subState = MEMBER_AS(short, p, rxSubState);
    }
    else
    {
      subState = MEMBER_AS(short, p, rxSubState);
    }
    if (subState == V8SUB_ANSAM_SEEN)
    {
      status = V8_ORG_ANSAM_DETECTED_WAITING_TE;
    }
    else if (MEMBER_AS(short, p, phasePair) == V8TX_MESSAGE)
    {
      status = V8_ORG_SEND_CJ;
      if (V8_PGM(void*, V8Obj, p, txMsgPtr) != (void*)(MEMBER_ADDR(p, msg[1])))
      {
        status = (subState == V8SUB_JM_DETECTED) + V8_ORG_SEND_CM; /* SEND_CM, or JM_DETECTED (the next status) */
      }
    }
    else if (MEMBER_AS(short, p, phasePair) == V8TX_QC_PREAMBLE)
    {
      status = V8_ORG_SEND_QC;
    }
    else if (MEMBER_AS_AT(short, p, phasePair, 2) == V8RX_TIMEOUT_ANSAM)
    {
      status = V8_ORG_TIME_OUT_WAITING_FOR_ANSAM;
    }
    else if (MEMBER_AS_AT(short, p, phasePair, 2) == V8RX_TIMEOUT_JM)
    {
      status = V8_ORG_TIME_OUT_WAITING_FOR_JM;
    }
  }
  else
  {
    short rxPhase;
    if (MEMBER_AS(short, p, phasePair) == V8TX_ANSAM)
    {
      status = (MEMBER_AS(short, p, rxSubState) == V8SUB_CM_DETECTED) + V8_ANS_SEND_ANSAM; /* SEND_ANSAM, or CM_DETECTED (the next status) */
    }
    else if (MEMBER_AS(short, p, phasePair) == V8TX_MESSAGE)
    {
      status = V8_ANS_SEND_JM;
    }
    rxPhase = MEMBER_AS_AT(short, p, phasePair, 2);
    if (rxPhase == V8RX_TIMEOUT_CM)
    {
      status = V8_ANS_TIME_OUT_WAITING_FOR_CM;
    }
    else if (rxPhase == V8RX_TIMEOUT_CJ)
    {
      status = V8_ANS_TIME_OUT_WAITING_FOR_CJ;
    }
  }

haveStatus:
  if (completed)
  {
    status = V8_OK;
  }

  if (p->lastStatus != status)
  {
    if (1 < dsplibs_debug_level)
    {
      int prevIdx = p->lastStatus;
      dsplibs_debug_printf("V8: State changed from %s to %s\r\n", v8StatusName[prevIdx], v8StatusName[status]);
    }
    p->lastStatus = status;
  }
  return (unsigned int)status;
}

/**
 * @brief Write the negotiated result into the shared capability block (the DpRuntime of the data-pump glue).
 * @param p     the engine object
 * @param caps  the capability block (also the one given in V8Cfg)
 * @return 0, or UINT_MAX when no message has been received
 * @note  With quick connect: sets the quick-connect and LAPM bits and the ANSpcm level, and the V.90 bit.  Otherwise: clears the call function and
 *        raw octet bits, sets the one negotiated by `callFnWord`, walks the received menu (the JM for the caller, the CM for the answerer): the
 *        modulation octets (tag 0x14 in the upper bits of the word) clear the modulation bits the remote does not offer, the PSTN access octet
 *        (tag 0x16) gives the digital connection flag and the PCM availability octet (tag 0x1c) the PCM indication; the V.90 bit stays set only when
 *        the remote offers V.90 with a digital connection and PCM indication 1.  A negotiated protocol other than LAPM becomes the raw protocol
 *        octet.  Bit 0 of the first byte is set at the end.
 */

unsigned int V8UpdateModemParameters(V8Obj* p, unsigned char* caps)
{
  unsigned short rxWord;
  unsigned short rxTag;
  short curIdx;
  char flag;
  unsigned char capByteA;
  int rxIdx;
  unsigned char capByteB;
  unsigned short pcmIndication;
  unsigned short digitalConn;
  unsigned int modBits;
  short rxCount;
  char* rxBase;
  unsigned int remoteV90Mod;

  remoteV90Mod = 0;
  digitalConn = 0;
  pcmIndication = 0;

  /* the received menu: the JM for the caller, the CM for the answerer */
  if (p->side == 0)
  {
    rxBase = MEMBER_ADDR(p, msg[2]);
  }
  else
  {
    rxBase = MEMBER_ADDR(p, msg[0]);
  }

  /* quick-connect result and the LAPM indication */
  capByteA = caps[2];
  capByteB = (unsigned char)((MEMBER_AS(unsigned char, p, qcDetected) & 1) << 4);
  caps[2] = (unsigned char)((capByteA & 0xef) | capByteB);
  *(unsigned int*)(caps + 0xc) = p->ansPcmLevel;
  flag = '\0';
  if ((p->qcLapm != 0) && ((capByteA & 0x40) != 0))
  {
    flag = '\x01';
  }
  caps[2] = (unsigned char)((capByteA & 0xaf) | capByteB | (flag << 6));
  if (MEMBER_AS(int, p, qcDetected) != 0)
  {
    if (1 < dsplibs_debug_level)
    {
      dsplibs_debug_printf("V8Report: Finished with Quick Connect\n");
    }
    *caps = *caps | 9;
    return 0;
  }

  if (*(short*)(rxBase + 0x28) < 1)
  {
    return UINT_MAX;
  }

  /* the negotiated call function */
  capByteB = (unsigned char)((capByteA & 0xa8) | capByteB | (flag << 6));
  caps[2] = capByteB;
  caps[0x18] = 0;
  capByteA = caps[1] & 0x3f;
  caps[1] = capByteA;
  if (p->callFnAccepted == 0)
  {
    if (1 < dsplibs_debug_level)
    {
      dsplibs_debug_printf("V8Report: since no call function match, not indicating any modulation capability...\r\n");
    }
    goto LAB_no_call_function_match;
  }
  rxCount = p->callFnWord;
  if (rxCount == 0x107)
  {
    capByteA = capByteA | 0x40;
    caps[1] = capByteA;
  }
  else if (rxCount < 0x108)
  {
    if (rxCount == 0)
    {
      if (1 < dsplibs_debug_level)
      {
        dsplibs_debug_printf("V8Report: Didn't get matching call function\r\n");
      }
    }
    else if (rxCount == 0x103)
    {
      caps[2] = capByteB | 1;
    }
    else
    {
      caps[2] = capByteB | 4;
      capByteA = charFlip((unsigned char)(((int)p->callFnWord >> 1) & 0xff));
      caps[0x18] = capByteA;
    }
  }
  else if (rxCount != 0x109)
  {
    if (rxCount != 0x10b)
    {
      caps[2] = capByteB | 4;
      capByteA = charFlip((unsigned char)(((int)p->callFnWord >> 1) & 0xff));
      caps[0x18] = capByteA;
      goto LAB_after_cf_dispatch;
    }
    capByteA = capByteA | 0x80;
    caps[1] = capByteA;
  }
  else
  {
    caps[2] = capByteB | 2;
  }
LAB_after_cf_dispatch:
  /* walk the received octets: modulation modes, PSTN access, PCM availability */
  rxCount = *(short*)(rxBase + 0x28);
  rxIdx = 0;
  if (0 < rxCount)
  {
    do
    {
      unsigned char lastWord;

      rxWord = *(unsigned short*)(rxBase + rxIdx * 2);
      rxTag = (unsigned short)(rxWord >> 4);
      if (rxTag == 0x14)
      {
        remoteV90Mod = (unsigned int)(rxWord & 0xf);
        modBits = remoteV90Mod >> 1;
        if ((modBits & 2) == 0)
        {
          *caps = *caps & 0xdf;
        }
        if ((modBits & 1) == 0)
        {
          *caps = *caps & 0xbf;
        }
        remoteV90Mod = remoteV90Mod >> 3;
        if ((*(unsigned char*)(rxBase + 2 + rxIdx * 2) & 0x10) != 0)
        {
          rxCount = (short)rxIdx;
          rxIdx = (int)(short)(rxCount + 1);
          rxWord = (unsigned short)(*(unsigned short*)(rxBase + rxIdx * 2) >> 1);
          if ((rxWord & 1) == 0)
          {
            caps[1] = caps[1] & 0xf7;
          }
          if ((rxWord & 2) == 0)
          {
            caps[1] = caps[1] & 0xfb;
          }
          if ((rxWord & 0x20) == 0)
          {
            caps[1] = caps[1] & 0xfd;
          }
          if ((rxWord & 0x40) == 0)
          {
            caps[1] = caps[1] & 0xfe;
          }
          if ((rxWord & 0x80) == 0)
          {
            *caps = *caps & 0x7f;
          }
          if ((*(unsigned char*)(rxBase + 2 + rxIdx * 2) & 0x10) != 0)
          {
            rxIdx = (int)(short)(rxCount + 2);
            rxWord = *(unsigned short*)(rxBase + rxIdx * 2);
            if ((rxWord & 0x40) == 0)
            {
              caps[1] = caps[1] & 0xef;
              rxWord = *(unsigned short*)(rxBase + rxIdx * 2);
            }
            if ((rxWord & 3) == 0)
            {
              caps[1] = caps[1] & 0xdf;
            }
          }
        }
        rxCount = *(short*)(rxBase + 0x28);
      }
      if (rxTag == 0x16)
      {
        digitalConn = (unsigned short)((*(unsigned short*)(rxBase + rxIdx * 2) >> 1) & 1);
      }
      curIdx = (short)rxIdx;
      if (rxTag == 0x1c)
      {
        pcmIndication = (unsigned short)((*(unsigned short*)(rxBase + rxIdx * 2) >> 2) & 3);
        lastWord = rxCount == (short)(curIdx + 1);
      }
      else
      {
        lastWord = rxCount == (short)(curIdx + 1);
      }
      rxIdx = (int)(short)(curIdx + 1);
      if (lastWord)
        break;
    } while (rxCount > (short)(curIdx + 1));
  }

  if (1 < dsplibs_debug_level)
  {
    dsplibs_debug_printf("V8Report: remote V90: mod - %d, digital connection - %d, pcmIndication - %d\n", remoteV90Mod, digitalConn, pcmIndication);
  }
  flag = '\0';
  if (((*caps & 8) != 0 && (short)remoteV90Mod != 0) && (digitalConn != 0 && pcmIndication == 1))
  {
    flag = '\x01';
  }
  *caps = (unsigned char)((*caps & 0xf7) | (flag << 3));
LAB_no_call_function_match:
  /* a negotiated protocol other than LAPM */
  if (p->protoWord != 0 && p->protoWord != 0xa9)
  {
    caps[2] = caps[2] | 8;
    capByteA = charFlip((unsigned char)(((int)p->protoWord >> 1) & 0xff));
    caps[0x1c] = capByteA;
  }
  capByteA = *caps;
  *caps = capByteA | 1;
  if (dsplibs_debug_level < 2)
  {
    return 0;
  }
  capByteB = caps[1];
  dsplibs_debug_printf("V8Report: v90:%d, v34:%d, v34hd:%d, V32:%d, V22:%d, V17:%d, V29:%d, V27:%d, V23:%d, V21:%d\n", (capByteA >> 3) & 1, (capByteA >> 5) & 1,
                       (capByteA >> 6) & 1, capByteA >> 7, capByteB & 1, (capByteB >> 1) & 1, (capByteB >> 2) & 1, (capByteB >> 3) & 1, (capByteB >> 4) & 1,
                       (capByteB >> 5) & 1);
  return 0;
}
