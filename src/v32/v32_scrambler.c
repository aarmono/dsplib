/**
 * @file v32_scrambler.c
 * @brief V.32 / V.32bis self-synchronising scrambler and descrambler, and the three symbol-code encoders (absolute, differential, trellis).
 *
 * Transmit side: ScrambleDataV32() (v32_datapath.c) calls SDMv32_scrambler() on the data words; ModDataV32() then calls one of the SMC encoders, which
 * turn the scrambled symbol words into constellation codes and write them into the SMC output ring (V32Circ) that FPM_PPS_filter() reads.
 * Receive side: DescrambleDataV32() calls SDMv32_descrambler() on the decoded words.
 * - SDMv32_scrambler() / SDMv32_descrambler(): 23-bit shift register with two taps (V32Sdm); a symbol carries 2 to 6 bits, a 6-bit symbol is handled
 *   as two 3-bit halves.
 * - SMCv32_encoder_abs() / _dif() / _tcm(): absolute-phase, differential and trellis-coded symbol codes (V32Smc), selected by the engine's mode.
 *
 * The objects are addressed through typed views (V32Sdm, V32Smc, V32Circ); the functions are independent of each other and keep all state in those
 * objects.
 */

#include "v32_tables.h"
#include "v32_viterbi_tables.h"
#include "v32_internal.h"

/**
 * @brief Scramble data words in place.
 * @param sdm     scrambler object (V32Sdm): short 0 is the symbol width in bits (2 to 6), short 1 the polynomial tap, short 2 the register length (23);
 *                `symMask` = (1 << shiftWidth) - 1, `keepMask` = ~symMask, `shiftReg` the register, `tapShiftA` / `tapShiftB` = tap and register length
 *                minus the shift width
 * @param data    words to scramble; each holds one symbol (2 to 5 bits), or for width 6 the two 3-bit halves (bits 5..3 and 2..0)
 * @param count   number of words; 0 leaves the data alone, a negative count is not handled
 *
 * Per symbol: the scrambled bits are the data bits XOR the two register taps; they are shifted into the register (the scrambler feeds back its
 * OUTPUT).  A width-6 symbol does this twice with 3 bits each (high half first) and the scrambled halves are recombined into one word.  The register
 * is written back when the call ends.
 */
void SDMv32_scrambler(V32Sdm* sdm, unsigned short* data, short count)
{
  unsigned int keepMask;
  unsigned int symMask;
  short shiftWidth;
  unsigned int scrOut;
  unsigned int word;
  unsigned int shiftReg;
  short symWidth;
  unsigned int payload;
  unsigned char tapShiftB;
  unsigned char shiftCnt;

  keepMask = sdm->keepMask;
  symMask = sdm->symMask;
  shiftReg = sdm->shiftReg;
  symWidth = *(short*)sdm;
  shiftWidth = symWidth;
  if (symWidth == 6)
  {
    shiftWidth = 3;
  }
  count = count + -1;
  if (count != -1)
  {
    while (1)
    {
      if (symWidth == 6)
      {
        word = (unsigned int)*data;
        payload = (word & 0x38) >> 3;
      }
      else
      {
        word = (unsigned int)*data;
        payload = word;
      }
      tapShiftB = (unsigned char)sdm->tapShiftB;
      scrOut = (shiftReg >> (tapShiftB & 0x1f) ^ payload ^ shiftReg >> ((unsigned char)sdm->tapShiftA & 0x1f)) & 0xffff & symMask;
      shiftCnt = (unsigned char)shiftWidth;
      shiftReg = (shiftReg << (shiftCnt & 0x1f) & keepMask) | scrOut;
      if (symWidth == 6)
      {
        word = (shiftReg >> (tapShiftB & 0x1f) ^ (word & 7) ^ shiftReg >> ((unsigned char)sdm->tapShiftA & 0x1f)) & 0xffff & symMask;
        shiftReg = (shiftReg << (shiftCnt & 0x1f) & keepMask) | word;
        *data = (unsigned short)(scrOut << 3) | (unsigned short)word;
      }
      else
      {
        *data = (unsigned short)scrOut;
      }
      if (count == 0)
        break;
      count = count + -1;
      data = data + 1;
      symWidth = *(short*)sdm;
    }
  }
  sdm->shiftReg = shiftReg;
}

/**
 * @brief Descramble data words in place.
 * @param sdm     descrambler object (V32Sdm), same layout as for SDMv32_scrambler()
 * @param data    scrambled words; replaced by the data bits
 * @param count   number of words
 *
 * Same taps and symbol handling as SDMv32_scrambler(), but the register is fed with the received (still scrambled) bits, which makes the pair
 * self-synchronising: after a register length of symbols the descrambler state is independent of its start value.
 */
void SDMv32_descrambler(V32Sdm* sdm, unsigned short* data, short count)
{
  unsigned int keepMask;
  unsigned int symMask;
  short shiftWidth;
  unsigned int descrOut;
  unsigned int word;
  unsigned int tapA;
  unsigned int tapB2;
  unsigned int shiftReg;
  short symWidth;
  unsigned int payload;
  unsigned char tapShiftB;
  unsigned char shiftCnt;

  keepMask = sdm->keepMask;
  symMask = sdm->symMask;
  shiftReg = sdm->shiftReg;
  symWidth = *(short*)sdm;
  shiftWidth = symWidth;
  if (symWidth == 6)
  {
    shiftWidth = 3;
  }
  count = count + -1;
  if (count != -1)
  {
    while (1)
    {
      if (symWidth == 6)
      {
        word = (unsigned int)*data;
        payload = (word & 0x38) >> 3;
      }
      else
      {
        word = (unsigned int)*data;
        payload = word;
      }
      tapA = shiftReg >> ((unsigned char)sdm->tapShiftA & 0x1f);
      tapShiftB = (unsigned char)sdm->tapShiftB;
      descrOut = shiftReg >> (tapShiftB & 0x1f);
      shiftCnt = (unsigned char)shiftWidth;
      shiftReg = (shiftReg << (shiftCnt & 0x1f) & keepMask) | payload;
      descrOut = (descrOut ^ payload ^ tapA) & 0xffff & symMask;
      if (symWidth == 6)
      {
        tapA = shiftReg >> ((unsigned char)sdm->tapShiftA & 0x1f);
        tapB2 = shiftReg >> (tapShiftB & 0x1f);
        shiftReg = (shiftReg << (shiftCnt & 0x1f) & keepMask) | (word & 7);
        *data = (unsigned short)(descrOut << 3) |
                (unsigned short)(((unsigned short)tapB2 ^ (unsigned short)(word & 7) ^ (unsigned short)tapA) & (unsigned short)symMask);
      }
      else
      {
        *data = (unsigned short)descrOut;
      }
      if (count == 0)
        break;
      count = count + -1;
      data = data + 1;
      symWidth = *(short*)sdm;
    }
  }
  sdm->shiftReg = shiftReg;
}

/**
 * @brief Symbol-code encoder without differential or trellis state (absolute phase).
 * @param smc      SMC object (V32Smc); byte 0 is the mode tag copied into bits 15..8 of every code
 * @param circ     SMC output ring (V32Circ): code buffer, write index, ring length
 * @param rawBits  `count` words; only bits 1..0 are used
 * @param count    number of symbols
 *
 * Each symbol steps the 2-bit phase counter down by one quadrant and writes `(SMCv32_PMAP_ABS16[bits] + phase * 4) & 0xf` with the mode tag to the
 * ring, wrapping the write index at the ring length.  Phase and write index are stored back.
 */
void SMCv32_encoder_abs(V32Smc* smc, V32Circ* circ, unsigned short* rawBits, short count)
{
  char modeTag;
  short ringLen;
  unsigned short rawWord;
  char* outBase;
  unsigned int writeIdx;
  short nextIdx;
  unsigned short phase;

  phase = smc->phase;
  ringLen = MEMBER_AS_ADDR(short, V32Circ, (char*)circ, ringLen);
  outBase = V32X_PG(char*, circ, 0xb0, (int)offsetof(V32Circ, codeBuf));
  modeTag = *(char*)smc;
  writeIdx = (unsigned int)circ->writeIdx;
  while (count != 0)
  {
    rawWord = *rawBits;
    phase = phase + 3 & 3;
    rawBits = rawBits + 1;
    *(unsigned short*)(outBase + writeIdx * 2) = (unsigned short)((SMCv32_PMAP_ABS16[rawWord & 3] + phase * 4 & 0xf) | (unsigned short)((int)modeTag << 8));
    nextIdx = (short)writeIdx + 1;
    writeIdx = -(unsigned int)(nextIdx < ringLen) & (unsigned int)(int)nextIdx;
    count = count + -1;
  }
  smc->phase = phase;
  circ->writeIdx = (short)writeIdx;
}

/**
 * @brief Differential symbol-code encoder.
 * @param smc      SMC object (V32Smc, addressed as shorts): short 0 is the mode, shorts 4 and 5 hold the differential state of mode 0 and of the
 *                 other mode
 * @param circ     SMC output ring (V32Circ)
 * @param rawBits  `count` words; the 2 bits at bit position `codedShift` select the differential step, bits 1..0 are used again in mode != 0
 * @param count    number of symbols
 *
 * Per symbol: the quadrant counter steps down by one; the 4-bit differential state advances by SMCv32_PMAP16[step].  The code is
 * `(state + 1 + phase * 4) & 0xf` in mode 0, and `((bits1..0) + state + phase * 4) & 0xf` otherwise, with the mode in bits 15..8.  Phase, the
 * differential state of the mode and the ring write index are stored back.
 * @warning `count` of 0 produces nothing, but the stored state is rewritten from the loaded values; negative counts are not handled.
 */
void SMCv32_encoder_dif(V32Smc* smc, V32Circ* circ, unsigned short* rawBits, short count)
{
  short ringLen;
  unsigned short rawWord;
  char* outBase;
  unsigned short modeTag;
  short nextIdx;
  unsigned short diffState;
  short remaining;
  unsigned short phase;
  int mode;
  int more;
  unsigned int writeIdx;
  unsigned char codedShift;

  mode = (int)*(short*)smc;
  diffState = (unsigned short)((short*)smc)[mode + 4];
  phase = (unsigned short)smc->phase;
  outBase = V32X_PG(char*, circ, 0xb0, (int)offsetof(V32Circ, codeBuf));
  writeIdx = (unsigned int)circ->writeIdx;
  ringLen = MEMBER_AS_ADDR(short, V32Circ, (char*)circ, ringLen);
  remaining = count + -1;
  modeTag = (unsigned short)(mode << 8);
  if (count != 0)
  {
    codedShift = (unsigned char)smc->codedShift;
    if (mode == 0)
    {
      do
      {
        rawWord = *rawBits;
        phase = phase + 3 & 3;
        rawBits = rawBits + 1;
        diffState = (unsigned short)((diffState + SMCv32_PMAP16[(short)rawWord >> (codedShift & 0x1f) & 3]) & 0xf);
        *(unsigned short*)(outBase + writeIdx * 2) = (unsigned short)((diffState + 1 + phase * 4 & 0xf) | modeTag);
        nextIdx = (short)writeIdx + 1;
        writeIdx = -(unsigned int)(nextIdx < ringLen) & (unsigned int)(int)nextIdx;
        more = remaining != 0;
        remaining = remaining + -1;
      } while (more);
    }
    else
    {
      do
      {
        rawWord = *rawBits;
        phase = phase + 3 & 3;
        rawBits = rawBits + 1;
        diffState = (unsigned short)((diffState + SMCv32_PMAP16[(short)rawWord >> (codedShift & 0x1f) & 3]) & 0xf);
        *(unsigned short*)(outBase + writeIdx * 2) = (unsigned short)(((rawWord & 3) + diffState + phase * 4 & 0xf) | modeTag);
        nextIdx = (short)writeIdx + 1;
        writeIdx = -(unsigned int)(nextIdx < ringLen) & (unsigned int)(int)nextIdx;
        more = remaining != 0;
        remaining = remaining + -1;
      } while (more);
    }
  }
  smc->phase = (short)phase;
  ((short*)smc)[mode + 4] = (short)diffState;
  circ->writeIdx = (short)writeIdx;
}

/**
 * @brief Trellis-coded symbol-code encoder.
 * @param smc      SMC object (V32Smc): mode byte, phase counter, trellis state, previous transition value, number of uncoded payload bits
 * @param circ     SMC output ring (V32Circ)
 * @param rawBits  `count` words, masked to the payload width plus the two coded bits and written back masked
 * @param count    number of symbols
 *
 * Per symbol: the trellis state advances through TrellisEncodeDifTable (old state, the two high coded bits); the new state is combined with the low
 * uncoded bits into the code word, offset by the word limit when the previous transition value was above 3.  The quadrant counter steps down by one,
 * the next transition value comes from TrellisTransitionTable, and the ring entry is `(SMCv32_MOD[code >> payloadBits] >> (phase * 4) & 7)`
 * shifted up by the payload width plus the uncoded bits, tagged with the mode in bits 15..8.  Phase, trellis state, transition value and write
 * index are stored back.
 * @note The input words are modified: each is replaced by its masked value.
 */
void SMCv32_encoder_tcm(V32Smc* smc, V32Circ* circ, unsigned short* rawBits, short count)
{
  char modeTag;
  short transVal;
  short ringLen;
  char* outBase;
  unsigned int codeWord;
  unsigned char payloadBits;
  short nextIdx;
  unsigned int payloadMask;
  unsigned int wordLimit;
  int trellisState;
  unsigned int writeIdx;
  unsigned short phase;
  int prevTransVal;
  unsigned int rawWord;
  int modIdx;
  unsigned short modWord;
  unsigned short outNibble;

  transVal = smc->transVal;
  phase = smc->phase;
  trellisState = (int)smc->trellisState;
  payloadBits = (unsigned char)smc->payloadBits;
  payloadMask = (1U << (payloadBits & 0x1f)) - 1;
  wordLimit = (1U << (payloadBits + 2 & 0x1f)) & 0xffff;
  outBase = V32X_PG(char*, circ, 0xb0, (int)offsetof(V32Circ, codeBuf));
  ringLen = MEMBER_AS_ADDR(short, V32Circ, (char*)circ, ringLen);
  writeIdx = (unsigned int)circ->writeIdx;
  modeTag = *(char*)smc;
  for (count = count + -1; prevTransVal = (int)transVal, count != -1; count = count + -1)
  {
    /* mask the source word to the payload plus coded bits and write it back */
    rawWord = (unsigned int)*rawBits & (wordLimit - 1) & 0xffff;
    *rawBits = (unsigned short)rawWord;
    /* advance the trellis state, keyed by (old state, the coded bits) */
    trellisState = (int)TrellisEncodeDifTable[trellisState + ((int)rawWord >> (payloadBits & 0x1f)) * 4];
    /* combine the new state with the uncoded payload bits; add the word limit if the previous transition value was above 3 */
    codeWord = (unsigned int)(((trellisState << (payloadBits & 0x1f)) + (rawWord & payloadMask & 0xffff)) & 0xffff);
    if (3 < transVal)
    {
      codeWord = (codeWord + wordLimit) & 0xffff;
    }
    /* step the quadrant counter; look up the transition value for the next symbol */
    phase = phase + 3 & 3;
    transVal = TrellisTransitionTable[trellisState + prevTransVal * 4];
    nextIdx = (short)writeIdx + 1;
    /* ring entry: the rotated point from SMCv32_MOD above the uncoded bits, tagged with the mode */
    modIdx = (int)codeWord >> (payloadBits & 0x1f) & 0xffff;
    modWord = (unsigned short)SMCv32_MOD[modIdx];
    outNibble = (unsigned short)((((int)modWord >> (unsigned char)(phase * 4)) & 7) << (payloadBits & 0x1f));
    *(unsigned short*)(outBase + writeIdx * 2) =
        (unsigned short)((outNibble + ((unsigned short)codeWord & (unsigned short)payloadMask)) | (unsigned short)((int)modeTag << 8));
    writeIdx = -(unsigned int)(nextIdx < ringLen) & (unsigned int)(int)nextIdx;
    rawBits = rawBits + 1;
  }
  smc->phase = phase;
  smc->trellisState = (short)trellisState;
  smc->transVal = transVal;
  circ->writeIdx = (short)writeIdx;
}
