/**
 * @file v32_engine.c
 * @brief V.32 / V.32bis engine: object lifecycle, the handshake state machines, the per-block entry point, and the shared sequence helpers.
 *
 * One connection is three heap blocks, all reached from the connection wrapper `obj` (a V32Wrap *, the first parameter of the handlers and helpers here):
 * - the wrapper itself (0x6c bytes): the first 48 bytes are a copy of the creator's configuration (V32_CFG-shaped), followed by the status word `w030`
 *   (low byte: status or error code, high byte: flag bits) and the two slots `hdxPtr` and `engPtr`;
 * - the handshake block, V32Hdx (`hdxPtr`, 0xb0 bytes): the role and sub-state of the handshake machine, the current transmit and receive handler
 *   function slots (`txHandler`, `rxHandler`), the budgets and counters that time a phase, the GenSequence / DetSequence bit-serialiser fields, three
 *   FPM_TONE objects and one FPM_MTD object used as tone detectors, and the handshake AGC in its first 0x2c bytes;
 * - the engine block, V32Eng (`engPtr`, 0x50dc bytes): the transmit chain (scrambler, trellis / differential encoder, pulse-shaping filter) and the receive
 *   chain (matched filter, echo canceller, AGC, symbol-rate recovery, fractionally spaced equaliser, Viterbi decoder) as sub-objects, plus the copy
 *   buffers that carry the samples and bits of the current block.
 *
 * Entry points (called by the dp glue in dp_v32.c and by the fax code): V32FP_create(), V32FP_recreate() (in-place re-initialisation, also used by
 * V32FP_control()), V32FP_delete(), V32FP_status(), V32FP_control() (retrain and rate renegotiation requests), V32FP_GetDiagnostics(),
 * V32FP_GetCleanedSamples() and V32FP_modem(), the per-block call.
 *
 * Data flow of V32FP_modem(): the input samples and bits are copied into the engine, then the protocol function selected by the role
 * (V32_PROTOCOL_PORT[role]) runs.  Before the connection is up that is v32_handshake(): it receives through the active receive handler
 * (V32RxHdxModem()) and transmits through the active transmit handler (V32TxHdxModem()).  The handlers (TxHdx*, RxHdx*) generate or detect the
 * handshake signals of the current sub-state and watch the phase budget; when a phase ends or a detector fires they call V32NextState[role], one of the
 * five role state machines (V32OrgNextState, V32AnsNextState, V32LocLoopNextState, V32RngInitNextState, V32RngRespNextState), which advances
 * V32Hdx.subState, reprograms the engine (tone detectors, constellation, equaliser and echo-canceller adaptation, scrambler seed) and installs the handlers
 * of the next sub-state.  When the machine reaches the connected state the role becomes V32ROLE_DATA and v32_data() pumps the data directly.  The
 * helpers GenSequence / DetSequence / InitGenSequence / InitDetSequence / CodeRateSeq .. DecodeRateSeq code and decode the fixed bit patterns and the rate
 * sequences of the handshake.
 *
 * The signal-processing helpers the handlers call (ModDataV32, DemodDataV32, SetTxModeV32 ...) are in v32_datapath.c; the subsystem objects are in
 * v32_pulse_shaping.c, v32_echo_canceller.c, v32_timing.c, v32_equalizer.c and the fpm/ toolkit; the constant tables are in v32_tables.c and
 * v32_viterbi_tables.c.
 */

#include "core/dsp_table_handles.h"
#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "v32_tables.h"
#include "v32_viterbi_tables.h"
#include "fpm/fpm_tables.h"
#include "fpm/fpm.h"
#include "v32.h"
#include "core/dsp_constants.h" /* RATE_* */
#include "core/dsp_host.h"
#include "v32_internal.h"

/* The MTD object's table slot: on a 32-bit build the table handle word, on x86-64 the real pointer member.  MTD_RECREATE re-creates the detector from its
 * own configuration (the object doubles as its cfg block, so the call only re-points the table and resets the state); on x86-64 the three cfg words are built
 * from the members, the table pointer as its handle. */
#if UINTPTR_MAX == 0xffffffffu
#define MTD_COEF_IS(m, sym) CFGP_SLOT_IS(const short*, m, sym)
#define MTD_COEF_SET(m, sym) CFGP_SLOT_SET(const short*, m, sym)
#define MTD_RECREATE(m) FPM_MTD_create(m, (const int*)(m))
#else
#define MTD_COEF_IS(m, sym) ((m)->coef == (sym))
#define MTD_COEF_SET(m, sym) ((m)->coef = (sym))
#define MTD_RECREATE(m)                                                                                                                                        \
  do                                                                                                                                                           \
  {                                                                                                                                                            \
    int mtdc_[3];                                                                                                                                              \
    mtdc_[0] = cfgp_word((m)->coef);                                                                                                                           \
    mtdc_[1] = (m)->c1.w;                                                                                                                                      \
    mtdc_[2] = (m)->c2.w;                                                                                                                                      \
    FPM_MTD_create((m), mtdc_);                                                                                                                                \
  } while (0)
#endif

/* Handlers and state machines that are defined further down but installed or called as values earlier (V32NextState[], the construction-time handler
 * defaults of V32FP_recreate(), the error transitions of the receive handlers). */
short TxHdxTone(V32Wrap* obj, void* a2, short* out, short* a4);
short TxHdxNull(V32Wrap* obj, void* a2, short* out, short* a4);

short RxHdxTone(V32Wrap* obj, void* inSamples, void* outBits, short* count);

short TxHdxNoCarrier(V32Wrap* obj, void* a2, void* a3, unsigned short* a4);

short TxHdxTRN(V32Wrap* obj, void* a2, void* a3, unsigned short* a4);

/**
 * @brief Terminal receive handler after a failed phase.
 * @param obj  connection wrapper (V32Wrap)
 * @param a2   receive samples
 * @param a3   output bit buffer
 * @param a4   in: number of samples in the block; out: set to 0
 *
 * Installed by the error transition of every other receive handler.  It does no timing of its own: it flags the failure again in the status word
 * (`w030` bit 1 of the high byte), runs the demodulator once to drain the block so the receive chain stays consistent, discards the result and reports no
 * output bits.
 */
void RxHdxError(V32Wrap* obj, void* a2, void* a3, unsigned short* a4)
{
  V32W_BYTE(obj, w030, 1) |= 2;
  DemodDataV32(obj, a2, a3, *a4);
  *a4 = 0;
}

void RxHdxNoSignal(V32Wrap* obj, void* inSamples, void* outBits, short* count);

/* The role state machines; V32NextState[] (below) lists them. */
void V32OrgNextState(V32Wrap* obj);
void V32AnsNextState(V32Wrap* obj);
void V32RngInitNextState(V32Wrap* obj);
void V32RngRespNextState(V32Wrap* obj);

short TxHdxCarrierState(V32Wrap* obj, void* rawBits, void* out, unsigned short* count);
short TxHdxScrSequence(V32Wrap* obj, void* rawBits, void* out, unsigned short* count);
void RxHdxPhsReversal(V32Wrap* obj, void* inSamples, void* outBits, short* count);
void RxHdxSequence(V32Wrap* obj, void* inSamples, void* outBits, unsigned short* count);
void RxHdxData(V32Wrap* obj, void* inSamples, void* outBits, unsigned short* count);

/**
 * @brief Generate `count` entries of the repeating handshake bit pattern.
 * @param obj    connection wrapper (V32Wrap)
 * @param out    receives `count` entries, each the next `shiftDiv`-bit group of the pattern (most significant group first)
 * @param count  number of entries to produce
 *
 * The pattern, the group width, the period and the current position are the V32Hdx serialiser fields seeded by InitGenSequence(); the position counts
 * down and wraps with the mask `period / width - 1`, so a call continues where the previous one stopped.
 */
void GenSequence(V32Wrap* obj, unsigned short* out, short count)
{
  V32Hdx* hdx = V32W_PG(V32Hdx*, obj, (int)offsetof(V32Wrap, hdxPtr));
  short shiftDiv = hdx->w04e;
  unsigned short pattern = hdx->w052;
  unsigned short pos = hdx->w04a;
  unsigned short wrapMask = hdx->w04c;
  unsigned short outMask = hdx->w050;
  short n;

  for (n = (short)(count - 1); n != -1; n--)
  {
    int shiftAmt = ((int)(signed char)pos * (int)(signed char)shiftDiv) & 0x1f;
    *out = (unsigned short)(((int)(unsigned int)pattern) >> shiftAmt) & outMask;
    out++;
    pos = (unsigned short)((pos - 1) & wrapMask);
  }
  hdx->w04a = pos;
}

/**
 * @brief Correlate demodulated symbols against the expected handshake pattern.
 * @param obj    connection wrapper (V32Wrap)
 * @param in     `count` demodulated symbols, each holding `maskWidth` bits (see InitDetSequence())
 * @param count  number of symbols
 * @return 1-based index of the first symbol after which the register matched, or -1 when no symbol of the block matched
 *
 * The bits of each symbol are shifted most significant first into a 32-bit register kept in V32Hdx (`d064`); after every bit the register is tested
 * against the expected value under the AND mask, and the value UINT_MAX never counts as a match.  On a match the register ANDed with the expose mask is
 * stored where GetSequence() reads it.
 * @note  The scan of the matching symbol continues to its last bit, but the register saved for the next block is the value at the moment of the
 *        (last) match, not the value after the symbol's remaining bits.
 */
int DetSequence(V32Wrap* obj, unsigned short* in, short count)
{
  V32Hdx* hdx = V32W_PG(V32Hdx*, obj, (int)offsetof(V32Wrap, hdxPtr));
  unsigned short maskWidth = hdx->w054;
  unsigned int reg = hdx->d064;
  short remaining = (short)(count - 1);
  short pos = 0;

  for (;;)
  {
    int matched;
    unsigned short bits;

    if (remaining == -1)
    {
      hdx->d064 = reg;
      return -1;
    }
    pos = (short)(pos + 1);
    bits = *in;
    matched = 0;
    if (maskWidth != 0)
    {
      int i;
      unsigned short bitIdx = maskWidth;
      for (i = 0; i < (int)maskWidth; i++)
      {
        bitIdx = (unsigned short)(bitIdx - 1);
        reg = (reg << 1) | ((bits >> (bitIdx & 0x1f)) & 1);
        if ((hdx->d060 & reg) == hdx->d05c && reg != UINT_MAX)
        {
          hdx->d068 = reg & hdx->d058;
          hdx->d064 = reg;
          matched = 1;
        }
      }
      if (matched)
      {
        return pos;
      }
    }
    remaining = (short)(remaining - 1);
    in++;
  }
}

/**
 * @brief Store a value in one of the five small handshake registers of V32Hdx (`reg[0..4]`).
 * @param obj    connection wrapper (V32Wrap)
 * @param value  value to store
 * @param idx    register number; a value of 5 or more is ignored
 */
void StoreReg(V32Wrap* obj, short value, unsigned short idx)
{
  if (idx < 5)
  {
    V32Hdx* hdx = V32W_PG(V32Hdx*, obj, (int)offsetof(V32Wrap, hdxPtr));
    hdx->reg[idx] = value;
  }
}

/**
 * @brief Read one of the five small handshake registers of V32Hdx (`reg[0..4]`), sign-extended.
 * @param obj  connection wrapper (V32Wrap)
 * @param idx  register number
 * @return the register, or 0 when `idx` is 5 or more
 */
int LoadReg(V32Wrap* obj, unsigned short idx)
{
  int result = 0;
  if (idx < 5)
  {
    V32Hdx* hdx = V32W_PG(V32Hdx*, obj, (int)offsetof(V32Wrap, hdxPtr));
    result = (short)hdx->reg[idx];
  }
  return result;
}

/**
 * @brief Intersect the far end's rate capabilities with the local ones and return the rate-sequence word of the best common rate.
 * @param obj      connection wrapper (V32Wrap)
 * @param peerSeq  rate capability word received from the far end
 * @return the V32_RATE_SEQ word of the best rate both sides offer, or 0x111 when they have none in common
 *
 * The local word is V32_RATE_SEQ[V32Eng.rxRateClass].  The capability bits are tested from the highest rate down: 0x8 = 14400, 0x20 = 12000,
 * 0x200 = 9600 (with 0x80 on both sides the trellis-coded variant), 0x40 = 7200, 0x400 = 4800.  CodeESeq() and CodeFinalRateSeq() run the same
 * decision with another result table; DecodeRateSeq() returns the class itself.
 */
unsigned short CodeRateSeq(V32Wrap* obj, unsigned short peerSeq)
{
  V32Eng* eng = V32W_PG(V32Eng*, obj, (int)offsetof(V32Wrap, engPtr));
  unsigned short ownSeq = (unsigned short)V32_RATE_SEQ[eng->rxRateClass];
  unsigned short cls = V32RC_NONE;

  if ((peerSeq & 8) == 0 || (ownSeq & 8) == 0)
  {
    if ((peerSeq & 0x20) == 0 || (ownSeq & 0x20) == 0)
    {
      if ((peerSeq & 0x200) == 0 || (ownSeq & 0x200) == 0)
      {
        if ((peerSeq & 0x40) == 0 || (ownSeq & 0x40) == 0)
        {
          if ((peerSeq & 0x400) != 0)
          {
            cls = ((ownSeq & 0x400) == 0) ? V32RC_NONE : V32RC_4800;
          }
        }
        else
        {
          cls = V32RC_7200;
        }
      }
      else if ((peerSeq & 0x80) == 0 || (ownSeq & 0x80) == 0)
      {
        cls = V32RC_9600;
      }
      else
      {
        cls = V32RC_9600_TCM;
      }
    }
    else
    {
      cls = V32RC_12000;
    }
  }
  else
  {
    cls = V32RC_14400;
  }

  return (cls != V32RC_NONE) ? (unsigned short)V32_RATE_SEQ[cls] : 0x111;
}

/**
 * @brief As CodeRateSeq(), but the result is looked up in V32_ESEQ (the E-sequence acknowledgment of the far end's rate request).
 * @param obj      connection wrapper (V32Wrap)
 * @param peerSeq  rate capability word received from the far end
 * @return the V32_ESEQ word of the best common rate, or 0xf111 when there is none
 */
unsigned short CodeESeq(V32Wrap* obj, unsigned short peerSeq)
{
  V32Eng* eng = V32W_PG(V32Eng*, obj, (int)offsetof(V32Wrap, engPtr));
  unsigned short ownSeq = (unsigned short)V32_RATE_SEQ[eng->rxRateClass];
  unsigned short cls = V32RC_NONE;

  if ((peerSeq & 8) == 0 || (ownSeq & 8) == 0)
  {
    if ((peerSeq & 0x20) == 0 || (ownSeq & 0x20) == 0)
    {
      if ((peerSeq & 0x200) == 0 || (ownSeq & 0x200) == 0)
      {
        if ((peerSeq & 0x40) == 0 || (ownSeq & 0x40) == 0)
        {
          if ((peerSeq & 0x400) != 0)
          {
            cls = ((ownSeq & 0x400) == 0) ? V32RC_NONE : V32RC_4800;
          }
        }
        else
        {
          cls = V32RC_7200;
        }
      }
      else if ((peerSeq & 0x80) == 0 || (ownSeq & 0x80) == 0)
      {
        cls = V32RC_9600;
      }
      else
      {
        cls = V32RC_9600_TCM;
      }
    }
    else
    {
      cls = V32RC_12000;
    }
  }
  else
  {
    cls = V32RC_14400;
  }

  return (cls != V32RC_NONE) ? (unsigned short)V32_ESEQ[cls] : 0xf111;
}

/**
 * @brief As CodeRateSeq(), but the result is looked up in V32_FINAL_RATE_SEQ (the final rate-sequence acknowledgment of the answering side).
 * @param obj      connection wrapper (V32Wrap)
 * @param peerSeq  rate capability word received from the far end
 * @return the V32_FINAL_RATE_SEQ word of the best common rate, or 0x111 when there is none
 */
unsigned short CodeFinalRateSeq(V32Wrap* obj, unsigned short peerSeq)
{
  V32Eng* eng = V32W_PG(V32Eng*, obj, (int)offsetof(V32Wrap, engPtr));
  unsigned short ownSeq = (unsigned short)V32_RATE_SEQ[eng->rxRateClass];
  unsigned short cls = V32RC_NONE;

  if ((peerSeq & 8) == 0 || (ownSeq & 8) == 0)
  {
    if ((peerSeq & 0x20) == 0 || (ownSeq & 0x20) == 0)
    {
      if ((peerSeq & 0x200) == 0 || (ownSeq & 0x200) == 0)
      {
        if ((peerSeq & 0x40) == 0 || (ownSeq & 0x40) == 0)
        {
          if ((peerSeq & 0x400) != 0)
          {
            cls = ((ownSeq & 0x400) == 0) ? V32RC_NONE : V32RC_4800;
          }
        }
        else
        {
          cls = V32RC_7200;
        }
      }
      else if ((peerSeq & 0x80) == 0 || (ownSeq & 0x80) == 0)
      {
        cls = V32RC_9600;
      }
      else
      {
        cls = V32RC_9600_TCM;
      }
    }
    else
    {
      cls = V32RC_12000;
    }
  }
  else
  {
    cls = V32RC_14400;
  }

  return (cls != V32RC_NONE) ? (unsigned short)V32_FINAL_RATE_SEQ[cls] : 0x111;
}

/**
 * @brief Decode a received rate capability word to the best common rate class.
 * @param obj      connection wrapper (V32Wrap)
 * @param peerSeq  rate capability word received from the far end
 * @return a V32RateClass: the best rate both sides offer, V32RC_NONE when they have none in common
 *
 * Makes the same decision as CodeRateSeq() but returns the class instead of a table word.
 */
unsigned char DecodeRateSeq(V32Wrap* obj, unsigned short peerSeq)
{
  V32Eng* eng = V32W_PG(V32Eng*, obj, (int)offsetof(V32Wrap, engPtr));
  unsigned short ownSeq = (unsigned short)V32_RATE_SEQ[eng->rxRateClass];
  unsigned char cls = V32RC_NONE;

  if ((peerSeq & 8) == 0 || (ownSeq & 8) == 0)
  {
    if ((peerSeq & 0x20) != 0 && (ownSeq & 0x20) != 0)
    {
      return V32RC_12000;
    }
    if ((peerSeq & 0x200) == 0 || (ownSeq & 0x200) == 0)
    {
      if ((peerSeq & 0x40) != 0 && (ownSeq & 0x40) != 0)
      {
        return V32RC_7200;
      }
      if ((peerSeq & 0x400) != 0)
      {
        return ((ownSeq & 0x400) == 0) ? V32RC_NONE : V32RC_4800;
      }
    }
    else if ((peerSeq & 0x80) == 0 || (cls = V32RC_9600_TCM, (ownSeq & 0x80) == 0))
    {
      return V32RC_9600;
    }
  }
  else
  {
    cls = V32RC_14400;
  }
  return cls;
}

/**
 * @brief Rate-sequence word of a rate class.
 * @param obj        unused
 * @param rateClass  V32RateClass, 0 .. 6 (the table has seven entries)
 * @return V32_RATE_SEQ[rateClass]
 * @warning No range check on `rateClass`.
 */
unsigned short RateToSeq(V32Wrap* obj, short rateClass)
{
  (void)obj;
  return (unsigned short)V32_RATE_SEQ[rateClass];
}

/**
 * @brief Value exposed by the last DetSequence() match.
 * @param obj  connection wrapper (V32Wrap)
 * @return the matched register ANDed with the expose mask
 */
unsigned int GetSequence(V32Wrap* obj)
{
  V32Hdx* hdx = V32W_PG(V32Hdx*, obj, (int)offsetof(V32Wrap, hdxPtr));
  return hdx->d068;
}

/**
 * @brief Seed the GenSequence() serialiser.
 * @param obj         connection wrapper (V32Wrap)
 * @param pattern     the bit pattern, `periodBits` bits long
 * @param periodBits  pattern length in bits
 * @param shiftDiv    bits per generated entry (2: dibits, 8: bytes)
 * @warning `shiftDiv` must be non-zero (division), and `periodBits / shiftDiv` a power of two, because the wrap of the position is a mask.  The callers
 *          pass (4, 2) and (16, 8).
 *
 * The position starts at the top group, so the first GenSequence() call emits the most significant group of the pattern.
 */
void InitGenSequence(V32Wrap* obj, unsigned short pattern, unsigned short periodBits, unsigned short shiftDiv)
{
  V32Hdx* hdx = V32W_PG(V32Hdx*, obj, (int)offsetof(V32Wrap, hdxPtr));
  short wrap = (short)(periodBits / shiftDiv - 1);

  hdx->w04e = shiftDiv;
  hdx->w052 = (short)pattern;
  hdx->w04c = wrap;
  hdx->w04a = wrap;
  hdx->w050 = (short)((1 << (shiftDiv & 0x1f)) - 1);
}

/**
 * @brief Seed the DetSequence() correlator.
 * @param obj            connection wrapper (V32Wrap)
 * @param expectedValue  register value that counts as a match (under `andMask`)
 * @param andMask        mask applied to the register before the comparison
 * @param exposeMask     mask applied to the matching register to form the value GetSequence() returns
 * @param maskWidth      bits per input symbol
 *
 * Clears the shift register and the exposed value.
 */
void InitDetSequence(V32Wrap* obj, unsigned int expectedValue, unsigned int andMask, unsigned int exposeMask, unsigned short maskWidth)
{
  V32Hdx* hdx = V32W_PG(V32Hdx*, obj, (int)offsetof(V32Wrap, hdxPtr));

  hdx->w054 = maskWidth;
  hdx->d058 = exposeMask;
  hdx->d05c = expectedValue;
  hdx->d064 = 0;
  hdx->d068 = 0;
  hdx->d060 = andMask;
}

/**
 * @brief Retune the first handshake tone detector to another frequency.
 * @param obj   connection wrapper (V32Wrap)
 * @param freq  new detection frequency parameter (low 16 bits of the first configuration word of the FPM_TONE object)
 *
 * The detector's own nine-word configuration is read back, word 0 is patched and the object is re-created in place (the return value of
 * FPM_TONE_create() is not used, the object keeps its address).
 */
void SetToneDetect(V32Wrap* obj, unsigned short freq)
{
  V32Hdx* hdx = V32W_PG(V32Hdx*, obj, (int)offsetof(V32Wrap, hdxPtr));
#if UINTPTR_MAX == 0xffffffffu
  unsigned int* toneObj = *(unsigned int**)&hdx->toneA;
  unsigned int cfg[9];

  cfg[0] = (toneObj[0] & 0xffff0000u) | freq;
  cfg[1] = toneObj[1];
  cfg[2] = toneObj[2];
  cfg[3] = toneObj[3];
  cfg[4] = toneObj[4];
  cfg[5] = toneObj[5];
  cfg[6] = toneObj[6];
  cfg[7] = toneObj[7];
  cfg[8] = toneObj[8];

  FPM_TONE_create(V32H_PG(FpmTone*, hdx, (int)offsetof(V32Hdx, toneA)), (const int*)cfg);
#else
  /* x86-64: the tone object's 9 configuration words are r0.w[0..3], the table pointer (as a table handle) and r1.w[0..3] (fpm.h) */
  FpmTone* toneObj = V32H_PG(FpmTone*, hdx, (int)offsetof(V32Hdx, toneA));
  int cfg[9];

  cfg[0] = (int)(((unsigned int)toneObj->r0.w[0] & 0xffff0000u) | freq);
  cfg[1] = toneObj->r0.w[1];
  cfg[2] = toneObj->r0.w[2];
  cfg[3] = toneObj->r0.w[3];
  cfg[4] = cfgp_word(toneObj->lpf);
  cfg[5] = toneObj->r1.w[0];
  cfg[6] = toneObj->r1.w[1];
  cfg[7] = toneObj->r1.w[2];
  cfg[8] = toneObj->r1.w[3];

  FPM_TONE_create(toneObj, cfg);
#endif
}

/**
 * @brief Role state machine of the local loop (V32ROLE_LOCLOOP): advance the handshake by one sub-state.
 * @param obj  connection wrapper (V32Wrap)
 *
 * Called by the active handler when its phase budget runs out or its detector fires.  The sub-states install the handlers for the carrier, the phase
 * reversal hunt, the scrambled training sequence and the data phase, set the transmit and receive modes and the equaliser adaptation, and end in the
 * connected status (`w030`).  Sub-state V32ST_ERROR only refreshes the budget.
 * @note  The local-loop role is selected only by a configuration whose protocol selector is neither 0 nor 1; the engine's own creator never produces it.
 */
void V32LocLoopNextState(V32Wrap* obj)
{
  V32Hdx* hdx = V32W_PG(V32Hdx*, obj, (int)offsetof(V32Wrap, hdxPtr));

  switch (hdx->subState)
  {
  case V32ST_A:
    if ((int)hdx->txBudget > 0)
    {
      return;
    }
    hdx->subState = V32ST_B;
    hdx->txBudget = 0xb4;
    SetToneDetect(obj, 0);
    break;
  case V32ST_B:
    hdx->subState = V32ST_C;
    hdx->rxElapsed = 0;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, txHandler), (v32fn_t)TxHdxCarrierState);
    hdx->txBudget = (int)hdx->stateLimit;
    InitGenSequence(obj, 3, 4, 2);
    SetToneDetect(obj, 600);
    break;
  case V32ST_C:
    hdx->subState = V32ST_D;
    hdx->txBudget = 0x200;
    hdx->rxElapsed = 0;
    hdx->phsRevLatch = 1;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, rxHandler), (v32fn_t)RxHdxPhsReversal);
    break;
  case V32ST_D:
    hdx->subState = V32ST_E;
    hdx->txBudget = (int)hdx->stateLimit;
    InitGenSequence(obj, 0xc, 4, 2);
    break;
  case V32ST_E:
    hdx->subState = V32ST_F;
    hdx->txBudget = 0x100;
    hdx->rxElapsed = 0;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, rxHandler), (v32fn_t)RxHdxData);
    StoreReg(obj, hdx->rtdResult, 0);
    SetRxLoopsV32(obj, 2);
    SetTxModeV32(obj, V32MODE_HANDSHAKE);
    InitGenSequence(obj, 1, 4, 2);
    break;
  case V32ST_F:
    hdx->subState = V32ST_G;
    hdx->rxElapsed = 0;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, txHandler), (v32fn_t)TxHdxScrSequence);
    hdx->txBudget = (int)hdx->stateLimit;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, rxHandler), (v32fn_t)RxHdxSequence);
    InitGenSequence(obj, 0xf, 4, 2);
    SetTxModeV32(obj, V32MODE_4800);
    SeedScramblerV32(obj, 0);
    InitDetSequence(obj, 0xffff, 0xffff, 0xffff, 2);
    break;
  case V32ST_G:
    hdx->subState = V32ST_H;
    hdx->txBudget = 0x2000;
    hdx->rxElapsed = 0;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, rxHandler), (v32fn_t)RxHdxData);
    SetAdaptEqV32(obj, 2);
    V32W_BYTE(obj, w030, 1) = (unsigned char)((V32W_BYTE(obj, w030, 1) & 0xbf) | 0x20);
    break;
  case V32ST_H:
  {
    short rate;
    hdx->subState = V32ST_I;
    hdx->txBudget = 0x960;
    hdx->rxElapsed = 0;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, rxHandler), (v32fn_t)RxHdxData);
    SetAdaptEqV32(obj, 1);
    rate = (short)GetRateV32(obj);
    SetTxModeV32(obj, V32_TX_MODE[rate]);
    SetRxModeV32(obj, V32_RX_MODE[rate]);
    InitGenSequence(obj, 0xffff, 0x10, 8);
    break;
  }
  case V32ST_I:
  {
    short rate;
    hdx->rxElapsed = 0;
    hdx->txBudget = (int)hdx->stateLimit;
    SetAdaptEqV32(obj, 3);
    rate = (short)GetRateV32(obj);
    V32W_BYTE(obj, w030, 1) = (unsigned char)(V32W_BYTE(obj, w030, 1) | 0x19);
    V32W_BYTE(obj, w030, 0) = (char)V32_CONNECT[rate];
    break;
  }
  case V32ST_ERROR:
    hdx->txBudget = (int)hdx->stateLimit;
    break;
  default:
    break;
  }
}

/** Role state machines, indexed by V32Hdx.role (slot 3 duplicates the local loop; the connected role has no entry because v32_data() does not use one). */
typedef void (*v32_nextstate_fn)(V32Wrap*);
static const v32_nextstate_fn V32NextState[6] = {
    V32OrgNextState, V32AnsNextState, V32LocLoopNextState, V32LocLoopNextState, V32RngInitNextState, V32RngRespNextState,
};

/**
 * @brief Transmit handler: modulate a generated training pattern until the phase budget is used up.
 * @param obj      connection wrapper (V32Wrap)
 * @param rawBits  work buffer for the generated bits
 * @param out      output samples
 * @param count    in: number of samples left in the block; out: number not yet produced
 * @return number of output samples written
 * @warning The budget (`txBudget`) is decreased by the full requested count but the number generated is clamped to the remaining budget; if the budget is
 *          already negative on entry the clamp yields `budget & 0xffff`, a large count.  The state machines refresh the budget whenever it reaches zero,
 *          so this does not arise today.
 *
 * Generates the pattern with GenSequence(), modulates it with ModDataV32() and, once the budget is exhausted, calls the role's state machine.
 */
short TxHdxCarrierState(V32Wrap* obj, void* rawBits, void* out, unsigned short* count)
{
  V32Hdx* hdx = V32W_PG(V32Hdx*, obj, (int)offsetof(V32Wrap, hdxPtr));
  unsigned int budget = hdx->txBudget;
  unsigned int produced = (unsigned int)*count;
  short modOut;

  if ((int)budget <= (int)produced)
  {
    produced = budget & 0xffffu;
  }

  hdx->txBudget = hdx->txBudget - (unsigned int)*count;

  GenSequence(obj, (unsigned short*)rawBits, (short)produced);
  modOut = (short)ModDataV32(obj, rawBits, out, (unsigned short)produced);

  if (0 < (int)hdx->txBudget)
  {
    *count = (unsigned short)(*count - (short)produced);
    return modOut;
  }
  V32NextState[hdx->role](obj);
  *count = (unsigned short)(*count - (short)produced);
  return modOut;
}

/**
 * @brief Transmit handler: as TxHdxCarrierState(), but the generated pattern is scrambled before it is modulated.
 * @param obj      connection wrapper (V32Wrap)
 * @param rawBits  work buffer for the generated bits
 * @param out      output samples
 * @param count    in: number of samples left in the block; out: number not yet produced
 * @return number of output samples written
 */
short TxHdxScrSequence(V32Wrap* obj, void* rawBits, void* out, unsigned short* count)
{
  V32Hdx* hdx = V32W_PG(V32Hdx*, obj, (int)offsetof(V32Wrap, hdxPtr));
  unsigned int budget = hdx->txBudget;
  unsigned int produced = (unsigned int)*count;
  short modOut;

  if ((int)budget <= (int)produced)
  {
    produced = budget & 0xffffu;
  }

  hdx->txBudget = hdx->txBudget - (unsigned int)*count;

  GenSequence(obj, (unsigned short*)rawBits, (short)produced);
  ScrambleDataV32(obj, rawBits, (unsigned short)produced);
  modOut = (short)ModDataV32(obj, rawBits, out, (unsigned short)produced);

  if (0 < (int)hdx->txBudget)
  {
    *count = (unsigned short)(*count - (short)produced);
    return modOut;
  }
  V32NextState[hdx->role](obj);
  *count = (unsigned short)(*count - (short)produced);
  return modOut;
}

/**
 * @brief Transmit handler of the training (TRN) segment: scrambled pattern with every dibit remapped before modulation.
 * @param obj      connection wrapper (V32Wrap)
 * @param rawBits  work buffer for the generated bits
 * @param out      output samples
 * @param count    in: number of samples left in the block; out: number not yet produced
 * @return number of output samples written
 *
 * As TxHdxScrSequence(), with the dibits mapped 0 -> 0, 1 -> 0, 2 -> 3, 3 -> 3 after scrambling (alternating phase pair).
 */
short TxHdxTRN(V32Wrap* obj, void* rawBits, void* out, unsigned short* count)
{
  static const short DIBIT_REMAP[4] = {0, 0, 3, 3};
  V32Hdx* hdx = V32W_PG(V32Hdx*, obj, (int)offsetof(V32Wrap, hdxPtr));
  unsigned int budget = hdx->txBudget;
  unsigned int produced = (unsigned int)*count;
  short modOut;
  int i;
  unsigned short* bits = (unsigned short*)rawBits;

  if ((int)budget <= (int)produced)
  {
    produced = budget & 0xffffu;
  }

  hdx->txBudget = hdx->txBudget - (unsigned int)*count;

  GenSequence(obj, (unsigned short*)rawBits, (short)produced);
  ScrambleDataV32(obj, rawBits, (unsigned short)produced);

  for (i = 0; i < (int)produced; i++)
  {
    bits[i] = (unsigned short)DIBIT_REMAP[bits[i] & 3];
  }

  modOut = (short)ModDataV32(obj, rawBits, out, (unsigned short)produced);

  if (0 < (int)hdx->txBudget)
  {
    *count = (unsigned short)(*count - (short)produced);
    return modOut;
  }
  V32NextState[hdx->role](obj);
  *count = (unsigned short)(*count - (short)produced);
  return modOut;
}

/**
 * @brief Transmit handler that scrambles and modulates the caller's own bits (no generated pattern).
 * @param obj      connection wrapper (V32Wrap)
 * @param rawBits  the bits to send (scrambled in place)
 * @param out      output samples
 * @param count    in: number of samples left in the block; out: number not yet produced
 * @return number of output samples written
 *
 * The transmit handler of the responder of a rate renegotiation: the data keeps flowing while the new rate is negotiated.
 */
short TxHdxData(V32Wrap* obj, void* rawBits, void* out, unsigned short* count)
{
  V32Hdx* hdx = V32W_PG(V32Hdx*, obj, (int)offsetof(V32Wrap, hdxPtr));
  unsigned int budget = hdx->txBudget;
  unsigned int produced = (unsigned int)*count;
  short modOut;

  if ((int)budget <= (int)produced)
  {
    produced = budget & 0xffffu;
  }

  hdx->txBudget = hdx->txBudget - (unsigned int)*count;

  ScrambleDataV32(obj, rawBits, (unsigned short)produced);
  modOut = (short)ModDataV32(obj, rawBits, out, (unsigned short)produced);

  if (0 < (int)hdx->txBudget)
  {
    *count = (unsigned short)(*count - (short)produced);
    return modOut;
  }
  V32NextState[hdx->role](obj);
  *count = (unsigned short)(*count - (short)produced);
  return modOut;
}

/**
 * @brief Receive handler: hunt for the phase reversal that follows the carrier.
 * @param obj       connection wrapper (V32Wrap)
 * @param inSamples received samples
 * @param outBits   output bit buffer (filled with the clamp value by RxClampV32())
 * @param count     in: number of samples; out: number of output bits
 *
 * Every call runs the AGC and removes the two helper tones.  Until the detector has missed the carrier tone on more than three successive calls the
 * handler only counts misses; then it looks for the phase reversal with FPM_TONE_find_rev().  A hit computes the turn-around delay and the round-trip
 * delay estimate (`rtdResult`, two different formulas for the originating side and the answering side, both clamped at zero), and calls the role's state
 * machine.  A phase that outlasts `stateLimit` ends in the error state (status 0x12).
 */
void RxHdxPhsReversal(V32Wrap* obj, void* inSamples, void* outBits, short* count)
{
  V32Hdx* hdx = V32W_PG(V32Hdx*, obj, (int)offsetof(V32Wrap, hdxPtr));
  short c = *count;
  short toneHit;

  FPM_AGC_agc(V32H_AGC(hdx), (short*)inSamples, (unsigned short)c);
  FPM_TONE_kill(V32H_PG(FpmTone*, hdx, (int)offsetof(V32Hdx, toneB)), (short*)inSamples, c);
  FPM_TONE_kill(V32H_PG(FpmTone*, hdx, (int)offsetof(V32Hdx, toneC)), (short*)inSamples, c);

  hdx->rxElapsed = (int)hdx->rxElapsed + (int)hdx->frameLen;
  hdx->w0a8 = (short)(hdx->frameLen + hdx->w0a8);

  if (hdx->phsRevLatch != 0)
  {
    /* the carrier has been lost: look for the phase reversal */
    toneHit = FPM_TONE_find_rev(V32H_PG(FpmTone*, hdx, (int)offsetof(V32Hdx, toneA)), (short*)inSamples, c);
    if (0 < toneHit)
    {
      unsigned short delayRaw = (unsigned short)(hdx->w094 - (short)(hdx->w09c + hdx->w098 + hdx->w09a));
      /* clamp the delay at zero */
      int turnDelay = ((short)delayRaw < 0) ? 0 : (int)(short)delayRaw;
      short shiftedFindRev = (short)((((int)toneHit * 0x4ccc) + 0x1000) >> 0xd);
      short rtd;

      hdx->rxElapsed = turnDelay;
      if (dsplibs_debug_level > 1)
      {
        dsplibs_debug_printf("v32 Turn Around Delay = %d\n", turnDelay);
      }

      if (hdx->role == V32ROLE_ORG)
      {
        rtd = (short)(shiftedFindRev + hdx->w094 * -2);
        rtd = (rtd < 0) ? 0 : rtd;
      }
      else
      {
        rtd = (short)(shiftedFindRev + hdx->frameLen - hdx->w094 - hdx->w09c - hdx->w098 - hdx->w09a);
        if (dsplibs_debug_level > 1)
        {
          dsplibs_debug_printf("v32 RTD = %d\n", (int)rtd);
        }
        rtd = (rtd < 0) ? 0 : rtd;
      }
      hdx->rtdResult = rtd;

      V32NextState[hdx->role](obj);
    }
  }
  else
  {
    toneHit = FPM_TONE_detect(V32H_PG(FpmTone*, hdx, (int)offsetof(V32Hdx, toneA)), (short*)inSamples, c);
    {
      short misses = (toneHit == 0) ? (short)(hdx->toneMisses + 1) : 0;
      hdx->toneMisses = misses;
      if (misses > 3)
      {
        hdx->phsRevLatch = 1;
        hdx->toneMisses = 0;
      }
    }
  }

  if (hdx->stateLimit <= hdx->rxElapsed)
  {
    V32W_BYTE(obj, w030, 1) = (unsigned char)(V32W_BYTE(obj, w030, 1) | 2);
    V32W_BYTE(obj, w030, 0) = 0x12;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, txHandler), (v32fn_t)TxHdxNoCarrier);
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, rxHandler), (v32fn_t)RxHdxError);
    hdx->subState = V32ST_ERROR;
  }

  *count = (short)RxClampV32(obj, inSamples, (unsigned short*)outBits);
}

/**
 * @brief Receive handler: demodulate, descramble and correlate against the expected sequence.
 * @param obj       connection wrapper (V32Wrap)
 * @param inSamples received samples
 * @param outBits   descrambled output bits
 * @param count     in: number of samples; out: number of output bits
 *
 * A match of DetSequence() calls the role's state machine.  A phase that outlasts `stateLimit` ends in the error state (status 0x13).
 */
void RxHdxSequence(V32Wrap* obj, void* inSamples, void* outBits, unsigned short* count)
{
  V32Hdx* hdx = V32W_PG(V32Hdx*, obj, (int)offsetof(V32Wrap, hdxPtr));
  unsigned short c = *count;
  int matchPos;

  hdx->rxElapsed = (int)hdx->rxElapsed + (int)hdx->frameLen;

  c = DemodDataV32(obj, inSamples, outBits, c);
  *count = c;
  DescrambleDataV32(obj, outBits, c);
  matchPos = DetSequence(obj, (unsigned short*)outBits, (short)*count);
  if (matchPos >= 0)
  {
    V32NextState[hdx->role](obj);
  }

  if (hdx->stateLimit <= hdx->rxElapsed)
  {
    V32W_BYTE(obj, w030, 1) = (unsigned char)(V32W_BYTE(obj, w030, 1) | 2);
    V32W_BYTE(obj, w030, 0) = 0x13;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, txHandler), (v32fn_t)TxHdxNoCarrier);
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, rxHandler), (v32fn_t)RxHdxError);
    hdx->subState = V32ST_ERROR;
  }

  *count = RxClampV32(obj, inSamples, (unsigned short*)outBits);
}

/**
 * @brief Receive handler of the data phases: demodulate and descramble.
 * @param obj       connection wrapper (V32Wrap)
 * @param inSamples received samples
 * @param outBits   descrambled output bits
 * @param count     in: number of samples; out: number of output bits
 *
 * No state machine call and no timeout: the phase is ended by the transmit side.
 */
void RxHdxData(V32Wrap* obj, void* inSamples, void* outBits, unsigned short* count)
{
  V32Hdx* hdx = V32W_PG(V32Hdx*, obj, (int)offsetof(V32Wrap, hdxPtr));
  unsigned short c = *count;

  hdx->rxElapsed = (int)hdx->rxElapsed + (int)hdx->frameLen;

  c = DemodDataV32(obj, inSamples, outBits, c);
  *count = c;
  DescrambleDataV32(obj, outBits, c);
  *count = RxClampV32(obj, inSamples, (unsigned short*)outBits);
}

/**
 * @brief Transmit handler of the silent phases: send no carrier for the rest of the phase budget.
 * @param obj      connection wrapper (V32Wrap)
 * @param rawBits  work buffer (scrambled but not used)
 * @param out      output samples
 * @param count    in: number of samples left in the block; out: number not yet produced
 * @return number of output samples written
 *
 * Like TxHdxCarrierState(), but nothing is generated and the sample count is forced to zero once the budget is already used up; the samples come from
 * TxNoCarrierV32() (silence through the pulse-shaping filter).
 */
short TxHdxNoCarrier(V32Wrap* obj, void* rawBits, void* out, unsigned short* count)
{
  V32Hdx* hdx = V32W_PG(V32Hdx*, obj, (int)offsetof(V32Wrap, hdxPtr));
  unsigned int budget = hdx->txBudget;
  unsigned int produced = (unsigned int)*count;
  short modOut;

  if ((int)budget <= (int)produced)
  {
    produced = budget & 0xffffu;
  }

  produced = ((int)budget > 0) ? produced : 0;

  hdx->txBudget = hdx->txBudget - (unsigned int)*count;
  ScrambleDataV32(obj, rawBits, (unsigned short)produced);
  modOut = (short)TxNoCarrierV32(obj, rawBits, out, (short)produced);

  if (0 < (int)hdx->txBudget)
  {
    *count = (unsigned short)(*count - (short)produced);
    return modOut;
  }
  V32NextState[hdx->role](obj);
  *count = (unsigned short)(*count - (short)produced);
  return modOut;
}

/**
 * @brief Transmit handler of the last frame of a local retrain (installed by the answering role's retrain loop-back).
 * @param obj      connection wrapper (V32Wrap)
 * @param rawBits  work buffer (scrambled but not used)
 * @param out      output samples
 * @param count    in: number of samples left in the block; out: always set to 0
 * @return number of output samples written
 *
 * Unlike the other transmit handlers it does not clamp the count to the remaining budget: it scrambles and sends silence for the full `*count`, subtracts
 * that from the budget, calls the role's state machine once the budget is used up, and always reports the whole count as consumed.  The installing state
 * gives it a budget of one frame, so the count is small.
 */
short TxHdxFinishFrame(V32Wrap* obj, void* rawBits, void* out, unsigned short* count)
{
  V32Hdx* hdx = V32W_PG(V32Hdx*, obj, (int)offsetof(V32Wrap, hdxPtr));
  unsigned short produced = *count;
  short modOut;

  hdx->txBudget = (int)hdx->txBudget - (int)produced;
  ScrambleDataV32(obj, rawBits, produced);

  modOut = (short)TxNoCarrierV32(obj, rawBits, out, *count);
  hdx = V32W_PG(V32Hdx*, obj, (int)offsetof(V32Wrap, hdxPtr));

  if (0 < (int)hdx->txBudget)
  {
    *count = 0;
    return modOut;
  }
  V32NextState[hdx->role](obj);
  *count = 0;
  return modOut;
}

/**
 * @brief Receive handler of the initial tone phases: wait for the far end's tone to appear or drop out.
 * @param obj       connection wrapper (V32Wrap)
 * @param inSamples received samples
 * @param outBits   output bit buffer (filled with the clamp value by RxClampV32())
 * @param count     in: number of samples; out: number of output bits
 * @return 0
 *
 * The construction-time receive handler.  The AGC and the tone detector run when the side originates, when `b011` bit 2 of the wrapper is set, or
 * while the transmit budget exceeds 0xb4; a detector result of 0 (tone gone) calls the role's state machine.  This handler alone drives an originating
 * object through its first two sub-states.  A phase that outlasts `stateLimit` ends in the error state (status 0x10).
 */
short RxHdxTone(V32Wrap* obj, void* inSamples, void* outBits, short* count)
{
  V32Hdx* hdx = V32W_PG(V32Hdx*, obj, (int)offsetof(V32Wrap, hdxPtr));
  short c = *count;
  short detected;

  hdx->rxElapsed = (int)hdx->rxElapsed + (int)hdx->frameLen;

  if ((obj->protoSel != 1) || ((obj->b011 & 4) != 0) || (0xb4 < (int)hdx->txBudget))
  {
    FPM_AGC_agc(V32H_AGC(hdx), (short*)inSamples, (unsigned short)c);
    detected = (short)FPM_TONE_detect(V32H_PG(FpmTone*, hdx, (int)offsetof(V32Hdx, toneA)), (short*)inSamples, c);
    if (detected == 0)
    {
      V32NextState[hdx->role](obj);
    }
  }

  if (hdx->stateLimit <= hdx->rxElapsed)
  {
    V32W_BYTE(obj, w030, 1) = (unsigned char)(V32W_BYTE(obj, w030, 1) | 2);
    V32W_BYTE(obj, w030, 0) = 0x10;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, txHandler), (v32fn_t)TxHdxNoCarrier);
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, rxHandler), (v32fn_t)RxHdxError);
    hdx->subState = V32ST_ERROR;
  }

  *count = (short)RxClampV32(obj, inSamples, (unsigned short*)outBits);
  return 0;
}

/**
 * @brief Receive handler: wait for the S tone with the MTD detector.
 * @param obj       connection wrapper (V32Wrap)
 * @param inSamples received samples
 * @param outBits   output bit buffer
 * @param count     in: number of samples; out: number of output bits
 *
 * When the detector holds the V32_S_DATA_COEF table the block is demodulated first and the detector reads the engine's copy buffer; otherwise it reads
 * the raw samples.  A detector result of 0 calls the role's state machine.  A phase that outlasts `stateLimit` ends in the error state (status 0x14).
 * @note  The detector is only ever created with V32_S_DATA_COEF in this file, so the table test is always true and the raw-sample branch is not reached.
 */
void RxHdxSTone(V32Wrap* obj, void* inSamples, void* outBits, short* count)
{
  V32Hdx* hdx = V32W_PG(V32Hdx*, obj, (int)offsetof(V32Wrap, hdxPtr));
  V32Eng* eng = V32W_PG(V32Eng*, obj, (int)offsetof(V32Wrap, engPtr));
  FpmMtd* mtdObj = V32H_PG(FpmMtd*, hdx, (int)offsetof(V32Hdx, mtd));
  short detCount;
  short* detSamples;
  short r;

  hdx->rxElapsed = (int)hdx->rxElapsed + (int)hdx->frameLen;

  if (MTD_COEF_IS(mtdObj, V32_S_DATA_COEF))
  {
    *count = (short)DemodDataV32(obj, inSamples, outBits, (unsigned short)*count);
    detCount = eng->w50c8;
    detSamples = V32E_PG(short*, eng, (int)offsetof(V32Eng, copyBufA));
  }
  else
  {
    detCount = *count;
    detSamples = (short*)inSamples;
  }

  r = (short)FPM_MTD_detect(mtdObj, detSamples, detCount);
  if (r == 0)
  {
    V32NextState[hdx->role](obj);
  }

  if (hdx->stateLimit <= hdx->rxElapsed)
  {
    V32W_BYTE(obj, w030, 1) = (unsigned char)(V32W_BYTE(obj, w030, 1) | 2);
    V32W_BYTE(obj, w030, 0) = 0x14;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, txHandler), (v32fn_t)TxHdxNoCarrier);
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, rxHandler), (v32fn_t)RxHdxError);
    hdx->subState = V32ST_ERROR;
  }

  *count = (short)RxClampV32(obj, inSamples, (unsigned short*)outBits);
}

/**
 * @brief Receive handler: demodulate (no descrambling) until EpochDetectV32() reports symbol-timing lock.
 * @param obj       connection wrapper (V32Wrap)
 * @param inSamples received samples
 * @param outBits   output bit buffer
 * @param count     in: number of samples; out: number of output bits
 *
 * Lock calls the role's state machine.  A phase that outlasts `stateLimit` ends in the error state (status 0x15).
 */
void RxHdxEpoch(V32Wrap* obj, void* inSamples, void* outBits, unsigned short* count)
{
  V32Hdx* hdx = V32W_PG(V32Hdx*, obj, (int)offsetof(V32Wrap, hdxPtr));
  unsigned short c = *count;

  hdx->rxElapsed = (int)hdx->rxElapsed + (int)hdx->frameLen;

  c = DemodDataV32(obj, inSamples, outBits, c);
  *count = c;

  if (EpochDetectV32(obj) != 0)
  {
    V32NextState[hdx->role](obj);
  }

  if (hdx->stateLimit <= hdx->rxElapsed)
  {
    V32W_BYTE(obj, w030, 1) = (unsigned char)(V32W_BYTE(obj, w030, 1) | 2);
    V32W_BYTE(obj, w030, 0) = 0x15;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, txHandler), (v32fn_t)TxHdxNoCarrier);
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, rxHandler), (v32fn_t)RxHdxError);
    hdx->subState = V32ST_ERROR;
  }

  *count = RxClampV32(obj, inSamples, (unsigned short*)outBits);
}

/**
 * @brief Receive handler for the rate sequence: demodulate, descramble and correlate.
 * @param obj       connection wrapper (V32Wrap)
 * @param inSamples received samples
 * @param outBits   descrambled output bits
 * @param count     in: number of samples; out: number of output bits
 *
 * A DetSequence() match is accepted when the exposed value repeats: its high 16 bits equal its low 16 bits.  Only then is the role's state machine called
 * (the value is read through GetSequence() twice).  A phase that outlasts `stateLimit` ends in the error state (status 0x13).
 */
void RxHdxRateSequence(V32Wrap* obj, void* inSamples, void* outBits, unsigned short* count)
{
  V32Hdx* hdx = V32W_PG(V32Hdx*, obj, (int)offsetof(V32Wrap, hdxPtr));
  unsigned short c = *count;
  int matchPos;

  hdx->rxElapsed = (int)hdx->rxElapsed + (int)hdx->frameLen;

  c = DemodDataV32(obj, inSamples, outBits, c);
  *count = c;
  DescrambleDataV32(obj, outBits, c);

  matchPos = DetSequence(obj, (unsigned short*)outBits, (short)*count);
  if (matchPos >= 0)
  {
    unsigned int seqA = GetSequence(obj);
    unsigned int seqB = GetSequence(obj);
    if ((seqA >> 16) == (seqB & 0xffffu))
    {
      V32NextState[hdx->role](obj);
    }
  }

  if (hdx->stateLimit <= hdx->rxElapsed)
  {
    V32W_BYTE(obj, w030, 1) = (unsigned char)(V32W_BYTE(obj, w030, 1) | 2);
    V32W_BYTE(obj, w030, 0) = 0x13;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, txHandler), (v32fn_t)TxHdxNoCarrier);
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, rxHandler), (v32fn_t)RxHdxError);
    hdx->subState = V32ST_ERROR;
  }

  *count = RxClampV32(obj, inSamples, (unsigned short*)outBits);
}

/**
 * @brief Receive handler that ignores the line for one short phase.
 * @param obj       connection wrapper (V32Wrap)
 * @param inSamples unused
 * @param outBits   output bit buffer (filled with the clamp value by RxClampV32())
 * @param count     in: number of samples; out: number of output bits
 *
 * Only advances the phase timer; when it reaches `stateLimit` the handler goes to the error state (status 0x10).  No demodulation is done.
 */
void RxHdxNull(V32Wrap* obj, void* inSamples, void* outBits, unsigned short* count)
{
  V32Hdx* hdx = V32W_PG(V32Hdx*, obj, (int)offsetof(V32Wrap, hdxPtr));
  unsigned int acc = (unsigned int)((int)hdx->frameLen + (int)hdx->rxElapsed);

  hdx->rxElapsed = acc;

  if (hdx->stateLimit <= acc)
  {
    V32W_BYTE(obj, w030, 1) = (unsigned char)(V32W_BYTE(obj, w030, 1) | 2);
    V32W_BYTE(obj, w030, 0) = 0x10;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, txHandler), (v32fn_t)TxHdxNoCarrier);
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, rxHandler), (v32fn_t)RxHdxError);
    hdx->subState = V32ST_ERROR;
  }

  *count = RxClampV32(obj, inSamples, (unsigned short*)outBits);
}

/**
 * @brief Receive handler for the rate sequence of a rate renegotiation.
 * @param obj       connection wrapper (V32Wrap)
 * @param inSamples received samples
 * @param outBits   descrambled output bits
 * @param count     in: number of samples; out: number of output bits
 *
 * Two modes, selected by V32Hdx.w048 (the hold-off counter, zero when the handler is installed).  While it is zero the block is correlated with
 * DetSequence(); on a match the received sequence is stored in register 4, the handler replaces itself with RxHdxData(), the receive mode is set from the
 * decoded rate (DecodeRateSeq(), V32_RX_MODE) and the counter starts at 1; the role's state machine is not called.  Afterwards the counter accumulates the
 * frame length, and once it exceeds 0x17 the status flag 4 is raised.  A phase that outlasts `stateLimit` ends in the error state (status 0x13).
 */
void RxHdxSequenceE(V32Wrap* obj, void* inSamples, void* outBits, unsigned short* count)
{
  V32Hdx* hdx = V32W_PG(V32Hdx*, obj, (int)offsetof(V32Wrap, hdxPtr));
  unsigned short c = *count;
  int matchPos;

  hdx->rxElapsed = (int)hdx->rxElapsed + (int)hdx->frameLen;
  c = DemodDataV32(obj, inSamples, outBits, c);
  *count = c;
  DescrambleDataV32(obj, outBits, c);

  if (hdx->w048 == 0)
  {
    matchPos = DetSequence(obj, (unsigned short*)outBits, (short)*count);
    if (matchPos >= 0)
    {
      unsigned short seq = (unsigned short)GetSequence(obj);
      unsigned char rateClass;
      hdx->reg[4] = seq;
      V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, rxHandler), (v32fn_t)RxHdxData);
      rateClass = DecodeRateSeq(obj, hdx->reg[4]);
      SetRxModeV32(obj, V32_RX_MODE[rateClass]);
      hdx->w048 = (short)((short)hdx->w048 + 1);
    }
  }
  else
  {
    unsigned short settle = (unsigned short)(hdx->w048 + hdx->frameLen);
    hdx->w048 = settle;
    if (0x17 < settle)
    {
      V32W_BYTE(obj, w030, 1) = (unsigned char)(V32W_BYTE(obj, w030, 1) | 4);
    }
  }

  if (hdx->stateLimit <= hdx->rxElapsed)
  {
    V32W_BYTE(obj, w030, 1) = (unsigned char)(V32W_BYTE(obj, w030, 1) | 2);
    V32W_BYTE(obj, w030, 0) = 0x13;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, txHandler), (v32fn_t)TxHdxNoCarrier);
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, rxHandler), (v32fn_t)RxHdxError);
    hdx->subState = V32ST_ERROR;
  }
  *count = RxClampV32(obj, inSamples, (unsigned short*)outBits);
}

/**
 * @brief Receive handler for the start of a rate renegotiation: watch the tone and feed data at the same time.
 * @param obj       connection wrapper (V32Wrap)
 * @param inSamples received samples
 * @param outBits   descrambled output bits
 * @param count     in: number of samples; out: number of output bits
 *
 * Every call runs the tone detector (a result of 0 calls the role's state machine) and checks the phase timeout (status 0x14), and then, whichever of
 * the two happened, demodulates and descrambles the block.
 */
void RxHdxToneData(V32Wrap* obj, void* inSamples, void* outBits, short* count)
{
  V32Hdx* hdx = V32W_PG(V32Hdx*, obj, (int)offsetof(V32Wrap, hdxPtr));
  short c = *count;
  short detected;

  hdx->rxElapsed = (int)hdx->rxElapsed + (int)hdx->frameLen;

  detected = (short)FPM_TONE_detect(V32H_PG(FpmTone*, hdx, (int)offsetof(V32Hdx, toneA)), (short*)inSamples, c);
  if (detected == 0)
  {
    V32NextState[hdx->role](obj);
  }

  if (hdx->stateLimit <= hdx->rxElapsed)
  {
    V32W_BYTE(obj, w030, 1) = (unsigned char)(V32W_BYTE(obj, w030, 1) | 2);
    V32W_BYTE(obj, w030, 0) = 0x14;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, txHandler), (v32fn_t)TxHdxNoCarrier);
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, rxHandler), (v32fn_t)RxHdxError);
    hdx->subState = V32ST_ERROR;
  }

  *count = (short)DemodDataV32(obj, inSamples, outBits, (unsigned short)*count);
  DescrambleDataV32(obj, outBits, (unsigned short)*count);
  *count = (short)RxClampV32(obj, inSamples, (unsigned short*)outBits);
}

/**
 * @brief Role state machine of the originating side: advance the handshake by one sub-state.
 * @param obj  connection wrapper (V32Wrap)
 *
 * Called by the active handler when its phase budget runs out or its detector fires.  Covers the originate sequence from the first tone wait through the
 * carrier and phase reversal phases, the silent and S-tone phases, symbol-timing acquisition, the training segment and the rate-sequence exchange up to the
 * connected status; the universal V32ST_ERROR sub-state only refreshes the budget.  Each sub-state sets the next sub-state, the budgets and the handlers
 * of the following phase.
 */
void V32OrgNextState(V32Wrap* obj)
{
  V32Hdx* hdx = V32W_PG(V32Hdx*, obj, (int)offsetof(V32Wrap, hdxPtr));

  switch (hdx->subState)
  {
  case V32ST_A:
    /* V32ST_A -> V32ST_B: restart the timers. `w084` = 0, so the transmit handler's budget does not count down yet. */
    hdx->subState = V32ST_B;
    hdx->rxElapsed = 0;
    hdx->txBudget = (int)hdx->stateLimit;
    hdx->w084 = 0;
    break;

  case V32ST_B:
    /* V32ST_B -> V32ST_B2, once the receive accumulator exceeds 2399 samples: re-tune tone generator A and the tone detector for the phase-reversal hunt,
     * send carrier, receive with RxHdxPhsReversal, restart the timers. */
    if (0x95f < hdx->rxElapsed)
    {
      FpmTone* toneObj;
      hdx->subState = V32ST_B2;
      InitGenSequence(obj, 0, 4, 2);
      toneObj = V32H_PG(FpmTone*, hdx, (int)offsetof(V32Hdx, toneA));
      toneObj->r0.s[4] = 0x4000;
      toneObj->r0.s[3] = 0x6666;
      toneObj->r0.s[5] = (short)(toneObj->r0.s[5] >> 1);
      SetToneDetect(obj, 600);
      hdx->phsRevLatch = 0;
      V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, txHandler), (v32fn_t)TxHdxCarrierState);
      V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, rxHandler), (v32fn_t)RxHdxPhsReversal);
      hdx->txBudget = (int)hdx->stateLimit;
      hdx->rxElapsed = 0;
    }
    break;

  case V32ST_B2:
    /* V32ST_B2 -> V32ST_C, unconditional: the budget takes the accumulator value and `w084` the frame length for this one pass; V32ST_C replaces both. */
    hdx->subState = V32ST_C;
    hdx->txBudget = (int)hdx->rxElapsed;
    hdx->w084 = hdx->frameLen;
    break;

  case V32ST_C:
    /* V32ST_C -> V32ST_D: new transmit pattern, restart the timers, clear RxHdxPhsReversal's reversal counter (`w0a8`). */
    InitGenSequence(obj, 0xf, 4, 2);
    hdx->subState = V32ST_D;
    hdx->rxElapsed = 0;
    hdx->txBudget = (int)hdx->stateLimit;
    hdx->w084 = 0;
    hdx->w0a8 = 0;
    break;

  case V32ST_D:
    /* V32ST_D -> V32ST_D2, once RxHdxPhsReversal has counted more than 72: silence the transmitter and wait for the far end's signal to disappear
     * (RxHdxNoSignal). */
    if (0x48 < hdx->w0a8)
    {
      hdx->subState = V32ST_D2;
      V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, txHandler), (v32fn_t)TxHdxNoCarrier);
      V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, rxHandler), (v32fn_t)RxHdxNoSignal);
      hdx->txBudget = (int)hdx->stateLimit;
    }
    break;

  case V32ST_D2:
    /* V32ST_D2 -> V32ST_E, unconditional: post the "local retrain requested" status pattern, hand the measured round-trip delay to register 0, receive with
     * RxHdxSTone, and halve word 5 of tone generator A's state (as in V32ST_B). */
    V32W_BYTE(obj, w032, 0) = (unsigned char)(V32W_BYTE(obj, w032, 0) | 8);
    V32W_BYTE(obj, w030, 0) = 0xf;
    V32W_BYTE(obj, w030, 1) = (unsigned char)((V32W_BYTE(obj, w030, 1) & 0xbf) | 0x20);
    StoreReg(obj, hdx->rtdResult, 0);
    hdx->rxElapsed = 0;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, txHandler), (v32fn_t)TxHdxNoCarrier);
    {
      FpmTone* toneObj = V32H_PG(FpmTone*, hdx, (int)offsetof(V32Hdx, toneA));
      short halved = (short)(toneObj->r0.s[5] >> 1);
      hdx->txBudget = (int)hdx->stateLimit;
      V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, rxHandler), (v32fn_t)RxHdxSTone);
      toneObj->r0.s[5] = halved;
    }
    hdx->subState = V32ST_E;
    break;

  case V32ST_E:
    /* V32ST_E -> V32ST_F: begin symbol-timing (epoch) recovery in the handshake receive mode. */
    hdx->subState = V32ST_F;
    hdx->rxElapsed = 0;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, txHandler), (v32fn_t)TxHdxNoCarrier);
    hdx->txBudget = (int)hdx->stateLimit;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, rxHandler), (v32fn_t)RxHdxEpoch);
    SetRxModeV32(obj, V32MODE_HANDSHAKE);
    SetRxLoopsV32(obj, 2);
    InitDetSequence(obj, 0xcc0c, 0xfc0f, 0xffff, 2);
    break;

  case V32ST_F:
    /* V32ST_F -> V32ST_G: transmit budget 0x500, receive data with the equaliser in adaptation mode 2, and select the second entry of the default
     * gain-control coefficient tables. Shares its receive set-up with V32ST_P. */
    hdx->subState = V32ST_G;
    hdx->txBudget = 0x500;
    hdx->rxElapsed = 0;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, txHandler), (v32fn_t)TxHdxNoCarrier);
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, rxHandler), (v32fn_t)RxHdxData);
    SetAdaptEqV32(obj, 2);
    {
      V32Eng* eng = V32W_PG(V32Eng*, obj, (int)offsetof(V32Wrap, engPtr));
      V32E_AGCF(const short*, eng, (int)(offsetof(V32Eng, agc) + offsetof(FpmAgc, alpha)), alpha) = &V32_AGC_DEF_ALPHA[1];
      V32E_AGCF(const short*, eng, (int)(offsetof(V32Eng, agc) + offsetof(FpmAgc, beta)), beta) = &V32_AGC_DEF_BETA[1];
    }
    break;

  case V32ST_G:
    /* V32ST_G -> V32ST_H: the far end's rate sequence starts here. Receive with RxHdxRateSequence in the 4800 mode, freeze the gain control. */
    hdx->subState = V32ST_H;
    hdx->rxElapsed = 0;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, txHandler), (v32fn_t)TxHdxNoCarrier);
    hdx->txBudget = (int)hdx->stateLimit;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, rxHandler), (v32fn_t)RxHdxRateSequence);
    SetRxModeV32(obj, V32MODE_4800);
    SetAdaptEqV32(obj, 3);
    InitDetSequence(obj, 0x1110111, 0xf111f111, UINT_MAX, 2);
    FPM_AGC_Freeze(V32E_AGC(V32W_PG(V32Eng*, obj, (int)offsetof(V32Wrap, engPtr))));
    break;

  case V32ST_H:
    /* V32ST_H -> V32ST_I: a one-frame tick; the receiver is parked in RxHdxNull. */
    hdx->subState = V32ST_I;
    hdx->rxElapsed = 0;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, txHandler), (v32fn_t)TxHdxNoCarrier);
    hdx->txBudget = (int)hdx->frameLen;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, rxHandler), (v32fn_t)RxHdxNull);
    break;

  case V32ST_I:
  {
    /* V32ST_I -> V32ST_J: the transmit budget is the turn-around delay (`w094`) + 0x100 + register 0 (the measured round-trip delay), rounded up to an even
     * value. Carrier on, receive data, equaliser mode 1, receive loops 1; the bit sequence detected so far (16 bits) goes to register 1; begin sending the
     * first training pattern in the handshake mode. */
    short tad = hdx->w094;
    int reg0 = LoadReg(obj, 0);
    short seq;

    hdx->subState = V32ST_J;
    hdx->txBudget = tad + 0x100 + reg0;
    if ((hdx->txBudget & 1) != 0)
    {
      hdx->txBudget = hdx->txBudget + 1;
    }
    hdx->rxElapsed = 0;
    (void)LoadReg(obj, 0); /* second read of register 0; the result is not used */
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, txHandler), (v32fn_t)TxHdxCarrierState);
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, rxHandler), (v32fn_t)RxHdxData);
    SetAdaptEqV32(obj, 1);
    SetRxLoopsV32(obj, 1);
    seq = (short)GetSequence(obj);
    StoreReg(obj, seq, 1);
    SetTxModeV32(obj, V32MODE_HANDSHAKE);
    InitGenSequence(obj, 1, 4, 2);
    break;
  }

  case V32ST_J:
    /* V32ST_J -> V32ST_K: short (16 sample) tick with a new transmit pattern; the echo canceller's round-trip delay is set from register 0. */
    hdx->subState = V32ST_K;
    hdx->txBudget = 0x10;
    hdx->rxElapsed = 0;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, txHandler), (v32fn_t)TxHdxCarrierState);
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, rxHandler), (v32fn_t)RxHdxData);
    InitGenSequence(obj, 0xe, 4, 2);
    SetECRndTripDelayV32(obj, (unsigned short)LoadReg(obj, 0));
    break;

  case V32ST_K:
    /* V32ST_K -> V32ST_L: start the training segment (TxHdxTRN, 256 samples) from a freshly seeded scrambler. */
    hdx->subState = V32ST_L;
    hdx->txBudget = 0x100;
    hdx->rxElapsed = 0;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, txHandler), (v32fn_t)TxHdxTRN);
    InitGenSequence(obj, 0xf, 4, 2);
    SeedScramblerV32(obj, 0);
    break;

  case V32ST_L:
    /* V32ST_L -> V32ST_M: scrambled-sequence transmission for 1024 samples, echo canceller adaptation mode 2. */
    hdx->subState = V32ST_M;
    hdx->txBudget = 0x400;
    hdx->rxElapsed = 0;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, txHandler), (v32fn_t)TxHdxScrSequence);
    SetAdaptEcV32(obj, 2);
    break;

  case V32ST_M:
    /* V32ST_M -> V32ST_N: the long training segment (0x1b00 samples), echo canceller adaptation mode 3. */
    hdx->subState = V32ST_N;
    hdx->txBudget = 0x1b00;
    hdx->rxElapsed = 0;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, txHandler), (v32fn_t)TxHdxScrSequence);
    SetAdaptEcV32(obj, 3);
    break;

  case V32ST_N:
  {
    /* V32ST_N -> V32ST_O: code the local rate capability (register 1) into the rate sequence, keep it in register 2 and start sending it.  Receive with
     * RxHdxSTone and listen for the far end's pattern (gain control released, default coefficient tables entry 0).
     *
     * The tone detector's coefficient table is set to V32_S_DATA_COEF and the detector is then re-created from itself (MTD_RECREATE): the call re-points
     * the table word and clears the detector's state, the other configuration words keep their values.
     *
     * InitGenSequence() takes the coded rate zero-extended, StoreReg() the sign-extended value of the same 16 bits. */
    short codedRate;
    FpmMtd* mtdObj;
    V32Eng* eng;

    hdx->subState = V32ST_O;
    hdx->rxElapsed = 0;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, txHandler), (v32fn_t)TxHdxScrSequence);
    hdx->txBudget = (int)hdx->stateLimit;
    SetAdaptEcV32(obj, 1);
    codedRate = (short)CodeRateSeq(obj, (unsigned short)LoadReg(obj, 1));
    StoreReg(obj, codedRate, 2);

    mtdObj = V32H_PG(FpmMtd*, hdx, (int)offsetof(V32Hdx, mtd));
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, rxHandler), (v32fn_t)RxHdxSTone);
    MTD_COEF_SET(mtdObj, V32_S_DATA_COEF);
    MTD_RECREATE(mtdObj);

    InitGenSequence(obj, (unsigned short)codedRate, 0x10, 2);
    InitDetSequence(obj, 0x1110111, 0xf111f111, 0xffff, 2);
    SetTxModeV32(obj, V32MODE_4800);

    eng = V32W_PG(V32Eng*, obj, (int)offsetof(V32Wrap, engPtr));
    FPM_AGC_Release(V32E_AGC(eng));
    eng = V32W_PG(V32Eng*, obj, (int)offsetof(V32Wrap, engPtr));
    V32E_AGCF(const short*, eng, (int)(offsetof(V32Eng, agc) + offsetof(FpmAgc, alpha)), alpha) = &V32_AGC_DEF_ALPHA[0];
    V32E_AGCF(const short*, eng, (int)(offsetof(V32Eng, agc) + offsetof(FpmAgc, beta)), beta) = &V32_AGC_DEF_BETA[0];
    break;
  }

  case V32ST_O:
  {
    /* V32ST_O -> V32ST_P: second symbol-timing recovery stage; the symbol-rate estimator is re-initialised in place (no new allocation) and its flag word
     * cleared. The transmit budget is left as V32ST_N set it. */
    V32Eng* eng;

    hdx->subState = V32ST_P;
    hdx->rxElapsed = 0;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, txHandler), (v32fn_t)TxHdxScrSequence);
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, rxHandler), (v32fn_t)RxHdxEpoch);
    SetRxModeV32(obj, V32MODE_HANDSHAKE);
    SetRxLoopsV32(obj, 2);
    eng = V32W_PG(V32Eng*, obj, (int)offsetof(V32Wrap, engPtr));
    FPM_SRE_init(&eng->sre, SREv32_CFG_raw, 0);
    eng->fse.settleCtr = 0;
    break;
  }

  case V32ST_P:
  {
    /* V32ST_P -> V32ST_Q: the old transmit budget is remembered as `w046` = stateLimit - budget (a timing offset V32ST_R reads back); new budget 0x4d0.
     * Receive data with equaliser mode 2 and the second entry of the default gain-control coefficient tables, the same receive set-up as V32ST_F. */
    int oldBudget = (int)hdx->txBudget;
    V32Eng* eng;

    hdx->subState = V32ST_Q;
    hdx->txBudget = 0x4d0;
    hdx->rxElapsed = 0;
    hdx->w046 = (short)((int)hdx->stateLimit) - (short)oldBudget;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, txHandler), (v32fn_t)TxHdxScrSequence);
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, rxHandler), (v32fn_t)RxHdxData);
    SetAdaptEqV32(obj, 2);
    eng = V32W_PG(V32Eng*, obj, (int)offsetof(V32Wrap, engPtr));
    V32E_AGCF(const short*, eng, (int)(offsetof(V32Eng, agc) + offsetof(FpmAgc, alpha)), alpha) = &V32_AGC_DEF_ALPHA[1];
    V32E_AGCF(const short*, eng, (int)(offsetof(V32Eng, agc) + offsetof(FpmAgc, beta)), beta) = &V32_AGC_DEF_BETA[1];
    break;
  }

  case V32ST_Q:
    /* V32ST_Q -> V32ST_R: listen for the far end's coded rate sequence (RxHdxSequence, equaliser mode 3, detector pattern 0x1110111 / 0xf111f111), gain
     * control frozen. */
    hdx->subState = V32ST_R;
    hdx->rxElapsed = 0;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, txHandler), (v32fn_t)TxHdxScrSequence);
    hdx->txBudget = (int)hdx->stateLimit;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, rxHandler), (v32fn_t)RxHdxSequence);
    SetAdaptEqV32(obj, 3);
    InitDetSequence(obj, 0x1110111, 0xf111f111, 0xffff, 2);
    FPM_AGC_Freeze(V32E_AGC(V32W_PG(V32Eng*, obj, (int)offsetof(V32Wrap, engPtr))));
    break;

  case V32ST_R:
  {
    /* V32ST_R -> V32ST_S: align the transmit budget to the 8-sample grid, 8 - (((stateLimit - budget) + 0x4d0 + w046) & 7); the detected rate-sequence bits
     * (16 bits) go to register 3. */
    int phaseInc = (int)hdx->stateLimit;
    int curBudget = (int)hdx->txBudget;
    short delta46 = hdx->w046;
    short seq;

    hdx->subState = V32ST_S;
    hdx->txBudget = (unsigned int)(8 - (((phaseInc - curBudget) + 0x4d0 + (int)delta46) & 7));
    seq = (short)GetSequence(obj);
    StoreReg(obj, seq, 3);
    break;
  }

  case V32ST_S:
  {
    /* V32ST_S -> V32ST_T: decode the far end's rate sequence (register 3) against the local capabilities. If no rate is in common (V32RC_NONE) the failure
     * status (flag 2, code 0x17) is posted, but the handshake goes on. Then answer with the coded acknowledgement sequence and listen for the next pattern. */
    unsigned short reg3;
    unsigned char cls;
    unsigned short eseq;

    hdx->subState = V32ST_T;
    hdx->txBudget = 8;
    hdx->rxElapsed = 0;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, txHandler), (v32fn_t)TxHdxScrSequence);
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, rxHandler), (v32fn_t)RxHdxSequence);
    reg3 = (unsigned short)LoadReg(obj, 3);
    cls = DecodeRateSeq(obj, reg3);
    if (cls == V32RC_NONE)
    {
      V32W_BYTE(obj, w030, 1) = (unsigned char)(V32W_BYTE(obj, w030, 1) | 2);
      V32W_BYTE(obj, w030, 0) = 0x17;
    }
    SetAdaptEcV32(obj, 1);
    reg3 = (unsigned short)LoadReg(obj, 3);
    eseq = CodeESeq(obj, reg3);
    InitGenSequence(obj, eseq, 0x10, 2);
    InitDetSequence(obj, 0x111f111, 0xf111f111, UINT_MAX, 2);
    break;
  }

  case V32ST_T:
  {
    /* V32ST_T -> V32ST_U: switch the transmitter to the agreed mode (V32_TX_MODE[class]; class V32RC_NONE gives the unrecognised mode 7), send the idle
     * pattern, and reset the TCM state word for classes above V32RC_9600_TCM. */
    unsigned char cls;
    V32Eng* eng;

    hdx->subState = V32ST_U;
    hdx->rxElapsed = 0;
    hdx->txBudget = (int)hdx->stateLimit;
    cls = DecodeRateSeq(obj, (unsigned short)LoadReg(obj, 3));
    SetTxModeV32(obj, V32_TX_MODE[cls]);
    InitGenSequence(obj, 0xffff, 0x10, 8);
    if (cls > 2)
    {
      eng = V32W_PG(V32Eng*, obj, (int)offsetof(V32Wrap, engPtr));
      eng->smc.transVal = 0;
    }
    break;
  }

  case V32ST_U:
  {
    /* V32ST_U -> V32ST_V: keep the far end's acknowledgement sequence in register 4 and switch the receiver to the agreed mode (V32_RX_MODE[class]). */
    short seqVal;
    unsigned char cls;

    hdx->subState = V32ST_V;
    hdx->txBudget = 0x80;
    hdx->rxElapsed = 0;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, rxHandler), (v32fn_t)RxHdxData);
    seqVal = (short)GetSequence(obj);
    StoreReg(obj, seqVal, 4);
    cls = DecodeRateSeq(obj, (unsigned short)seqVal);
    SetRxModeV32(obj, V32_RX_MODE[cls]);
    break;
  }

  case V32ST_V:
  {
    /* V32ST_V, terminal: the state stays V32ST_V. Each call posts the "connected" flags, the connect code V32_CONNECT[class] for the agreed rate (the code
     * the host reports as the CONNECT result), clears the event flags of the equaliser history and sets the equaliser and the echo canceller to their
     * steady adaptation modes. */
    unsigned short reg3;
    unsigned char cls;
    V32Eng* eng;
    V32Hist* hist;

    hdx->subState = V32ST_V;
    hdx->rxElapsed = 0;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, rxHandler), (v32fn_t)RxHdxData);
    hdx->txBudget = (int)hdx->stateLimit;
    V32W_BYTE(obj, w030, 1) = (unsigned char)(V32W_BYTE(obj, w030, 1) | 0x19);
    reg3 = (unsigned short)LoadReg(obj, 3);
    cls = DecodeRateSeq(obj, reg3);
    V32W_BYTE(obj, w030, 0) = (char)V32_CONNECT[cls];
    eng = V32W_PG(V32Eng*, obj, (int)offsetof(V32Wrap, engPtr));
    hist = V32E_PG(V32Hist*, eng, (int)offsetof(V32Eng, fse.histObj));
    hist->evFlags = 0;
    SetAdaptEqV32(obj, 3);
    SetAdaptEcV32(obj, 1);
    break;
  }

  case V32ST_ERROR:
    /* any state may land here after an error: only the transmit budget is refreshed */
    hdx->txBudget = (int)hdx->stateLimit;
    break;

  default:
    break;
  }
}

/**
 * @brief Transmit handler of the answering side's start-up: a steady tone while the state machine waits for its budget.
 * @param obj  V.32 wrapper object (V32Wrap)
 * @param a2   unused
 * @param out  receives `w0a0` samples of tone generator A
 * @param a4   receives 0 (no input bits consumed)
 * @return number of samples written, `w0a0`
 *
 * Installed by the engine's creation code as the transmit handler of the answer role. Subtracts `w084` from the transmit budget on every call and calls
 * V32NextState[role] when the budget is used up.
 */
short TxHdxTone(V32Wrap* obj, void* a2, short* out, short* a4)
{
  V32Hdx* hdx = V32W_PG(V32Hdx*, obj, (int)offsetof(V32Wrap, hdxPtr));
  int budget;

  FPM_TONE_generate(V32H_PG(FpmTone*, hdx, (int)offsetof(V32Hdx, toneA)), out, hdx->w0a0);
  *a4 = 0;
  budget = (int)hdx->txBudget - (int)hdx->w084;
  hdx->txBudget = budget;
  if (0 < budget)
  {
    return hdx->w0a0;
  }
  V32NextState[hdx->role](obj);
  return ((V32Hdx*)V32W_PG(char*, obj, (int)offsetof(V32Wrap, hdxPtr)))->w0a0;
}

/**
 * @brief Transmit handler of the originating side's start-up: silence until the state machine moves on.
 * @param obj  V.32 wrapper object (V32Wrap)
 * @param a2   unused
 * @param out  receives `w0a0` zero samples
 * @param a4   receives 0 (no input bits consumed)
 * @return number of samples written, `w0a0`
 *
 * Counterpart of TxHdxTone() for the originate role: the same budget handling (`w084` per call, V32NextState[role] when it is used up), but nothing is sent.
 */
short TxHdxNull(V32Wrap* obj, void* a2, short* out, short* a4)
{
  V32Hdx* hdx = V32W_PG(V32Hdx*, obj, (int)offsetof(V32Wrap, hdxPtr));
  int n = (int)hdx->w0a0;
  int budget;

  (void)a2;
  while (n-- > 0)
    *out++ = 0;

  budget = (int)hdx->txBudget - (int)hdx->w084;
  hdx->txBudget = budget;
  if (0 < budget)
  {
    *a4 = 0;
    return hdx->w0a0;
  }
  V32NextState[hdx->role](obj);
  hdx = V32W_PG(V32Hdx*, obj, (int)offsetof(V32Wrap, hdxPtr));
  *a4 = 0;
  return hdx->w0a0;
}

/**
 * @brief Receive handler that waits for the far end's signal to return after a phase reversal.
 * @param obj        V.32 wrapper object (V32Wrap)
 * @param inSamples  input samples of this block
 * @param outBits    receives the decoded bits (through RxClampV32())
 * @param count      in: number of input samples; out: number of output entries
 *
 * Installed by both start-up machines when V32ST_D is left (the originate machine replaces it in V32ST_D2); in the answer machine it stays the receive
 * handler through V32ST_E and V32ST_F. Every call adds one frame length (`frameLen`) to `rxElapsed` and to the private counter `w0aa` (the answer machine's
 * V32ST_F guard uses `w0aa`), runs the gain control and the tone detector on the block, and calls V32NextState[role] when the detector reports a tone and
 * more than 60 samples have elapsed. It fails the call (status 0x11, flag 2, V32ST_ERROR, silent transmitter, RxHdxError) when `rxElapsed` reaches
 * `stateLimit`.
 * @note The detector result tested here is "signal present"; the other receive handlers act on the loss of the tone.
 */
void RxHdxNoSignal(V32Wrap* obj, void* inSamples, void* outBits, short* count)
{
  V32Hdx* hdx = V32W_PG(V32Hdx*, obj, (int)offsetof(V32Wrap, hdxPtr));
  short c = *count;
  short detected;

  hdx->rxElapsed = (int)hdx->rxElapsed + (int)hdx->frameLen;
  hdx->w0aa = (short)(hdx->w0aa + hdx->frameLen);

  FPM_AGC_agc(V32H_AGC(hdx), (short*)inSamples, (unsigned short)c);
  detected = (short)FPM_TONE_detect(V32H_PG(FpmTone*, hdx, (int)offsetof(V32Hdx, toneA)), (short*)inSamples, c);
  if (detected != 0)
  {
    if (0x3c < hdx->rxElapsed)
    {
      V32NextState[hdx->role](obj);
    }
  }
  if (hdx->stateLimit <= hdx->rxElapsed)
  {
    V32W_BYTE(obj, w030, 1) = (unsigned char)(V32W_BYTE(obj, w030, 1) | 2);
    V32W_BYTE(obj, w030, 0) = 0x11;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, txHandler), (v32fn_t)TxHdxNoCarrier);
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, rxHandler), (v32fn_t)RxHdxError);
    hdx->subState = V32ST_ERROR;
  }
  *count = (short)RxClampV32(obj, inSamples, (unsigned short*)outBits);
}

/**
 * @brief Answering-side handshake state machine: leave the current state of V32Hdx::subState and enter the next one.
 * @param obj  V.32 wrapper object (V32Wrap)
 *
 * The counterpart of V32OrgNextState() for the answer role (V32NextState[V32ROLE_ANS]); the timers `txBudget`, `rxElapsed`, `stateLimit` and the guard
 * convention are described there. Sequence, in short: V32ST_A and V32ST_B send the answer tone and arm the detector (the transmitter runs TxHdxTone());
 * V32ST_B2 sends the carrier and waits for the far end; V32ST_C .. V32ST_E hunt for the phase reversal and wait for the far end's signal to disappear and
 * return (RxHdxPhsReversal, RxHdxNoSignal); V32ST_F posts the "local retrain requested" status; V32ST_G .. V32ST_N send the training segments; V32ST_L codes
 * the local rate capability; V32ST_O .. V32ST_Q bring up the receiver; V32ST_T .. V32ST_Z exchange the rate and acknowledgement sequences and switch both
 * directions to the agreed mode; V32ST_END loops on itself and reports the connection. The states V32ST_D2 (5), 0x13 and 0x14 have no case, they do nothing.
 * The order of the states in the second half is V32ST_T, U, V, W, X, X2 (0x1e), Y, Z, END, not numeric.
 *
 * Notes on single states:
 * - V32ST_B2 is guarded by `txBudget < 1`.  The transmit handler only calls this function when the budget has just run out, so a call from there always takes
 *   the give-up branch (status code 0x16, flag 2, silent transmitter, RxHdxError, V32ST_ERROR); the other branch (go on to V32ST_C) can only be taken when
 *   another handler calls this function while the budget is still positive.
 * - V32ST_D rounds the old accumulator up to an even number for the new transmit budget: `(old + 1) - ((old & 1) == 0)`.
 * - V32ST_F reads `V32Hdx::rtdResult`, which only RxHdxPhsReversal writes (see the warning at that case).
 * - V32ST_F2 (0x1d) is entered from V32ST_F and goes back to V32ST_G, so the sequence of training segments is run through a second time.
 * - Unlike the originate machine, the answer machine does not change the gain-control coefficient tables, and it uses registers 2 and 4 for the far end's
 *   sequences (the originate machine registers 3 and 4).
 */
void V32AnsNextState(V32Wrap* obj)
{
  V32Hdx* hdx = V32W_PG(V32Hdx*, obj, (int)offsetof(V32Wrap, hdxPtr));

  switch (hdx->subState)
  {
  case V32ST_A:
    /* V32ST_A -> V32ST_B, once the tone budget has run out: new transmit budget of 180 samples, tone detector parameter 0, tone word 3 = 0x747a. */
    if ((int)hdx->txBudget <= 0)
    {
      FpmTone* toneObj;
      hdx->subState = V32ST_B;
      hdx->txBudget = 0xb4;
      SetToneDetect(obj, 0);
      toneObj = V32H_PG(FpmTone*, hdx, (int)offsetof(V32Hdx, toneA));
      toneObj->r0.s[3] = 0x747a;
    }
    break;

  case V32ST_B:
    /* V32ST_B -> V32ST_B2, unconditional: the carrier budget is 0x100 samples, or 0xa60 when bit 1 of the caller's configuration byte (V32Wrap::b011) is
     * set. Carrier on, detector threshold 0x708, tone word 3 = 0x747a. */
    {
      FpmTone* toneObj;
      hdx->subState = V32ST_B2;
      hdx->txBudget = ((obj->b011 & 2) != 0) ? 0xa60 : 0x100;
      hdx->rxElapsed = 0;
      V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, txHandler), (v32fn_t)TxHdxCarrierState);
      InitGenSequence(obj, 3, 4, 2);
      SetToneDetect(obj, 0x708);
      toneObj = V32H_PG(FpmTone*, hdx, (int)offsetof(V32Hdx, toneA));
      toneObj->r0.s[3] = 0x747a;
    }
    break;

  case V32ST_B2:
    /* V32ST_B2: see the notes in the function header. Give up (status 0x16, flag 2) when the budget is gone; otherwise go on to V32ST_C and set the
     * phase-reversal latch. */
    if ((int)hdx->txBudget < 1)
    {
      V32W_BYTE(obj, w030, 1) = (unsigned char)(V32W_BYTE(obj, w030, 1) | 2);
      V32W_BYTE(obj, w030, 0) = 0x16;
      V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, txHandler), (v32fn_t)TxHdxNoCarrier);
      V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, rxHandler), (v32fn_t)RxHdxError);
      hdx->subState = V32ST_ERROR;
    }
    else
    {
      hdx->subState = V32ST_C;
      hdx->phsRevLatch = 1;
      hdx->rxElapsed = 0;
    }
    break;

  case V32ST_C:
    /* V32ST_C -> V32ST_D, once the accumulator exceeds 139 samples: hunt for the phase reversal (RxHdxPhsReversal, new transmit pattern) and clear its
     * counter. */
    if (0x8b < hdx->rxElapsed)
    {
      hdx->subState = V32ST_D;
      hdx->phsRevLatch = 1;
      hdx->rxElapsed = 0;
      hdx->txBudget = (int)hdx->stateLimit;
      V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, rxHandler), (v32fn_t)RxHdxPhsReversal);
      InitGenSequence(obj, 0xc, 4, 2);
      hdx->w0a8 = 0;
    }
    break;

  case V32ST_D:
    /* V32ST_D -> V32ST_E (state 5 is skipped), once RxHdxPhsReversal has counted more than 72: listen for the far end's signal with RxHdxNoSignal while the
     * transmitter keeps sending its carrier. The new transmit budget is the old accumulator rounded up to an even number. Detector threshold 0x708, tone
     * word 3 = 0x4a38. */
    if (0x48 < hdx->w0a8)
    {
      unsigned int oldAccum = hdx->rxElapsed;
      FpmTone* toneObj;
      hdx->subState = V32ST_E;
      hdx->rxElapsed = 0;
      V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, rxHandler), (v32fn_t)RxHdxNoSignal);
      hdx->txBudget = (oldAccum + 1) - (unsigned int)((oldAccum & 1) == 0);
      SetToneDetect(obj, 0x708);
      toneObj = V32H_PG(FpmTone*, hdx, (int)offsetof(V32Hdx, toneA));
      toneObj->r0.s[3] = 0x4a38;
    }
    break;

  case V32ST_E:
    /* V32ST_E -> V32ST_F, once the carrier budget has run out: new transmit pattern, restart RxHdxNoSignal's private counter (`w0aa`). */
    if ((int)hdx->txBudget < 1)
    {
      hdx->subState = V32ST_F;
      hdx->txBudget = (int)hdx->stateLimit;
      InitGenSequence(obj, 3, 4, 2);
      hdx->w0aa = 0;
    }
    break;

  case V32ST_F:
    /* V32ST_F -> V32ST_F2, once RxHdxNoSignal has counted more than 95: post the "local retrain requested" status pattern, store the round-trip delay in
     * register 0,
     * silence the transmitter and receive data.
     * @warning `rtdResult` is only written by RxHdxPhsReversal, which the answer machine does not run; the value stored here is whatever the V32Hdx block
     *          held after allocation (zero only if the host's sysdep_malloc returns zeroed memory). */
    if (0x5f < hdx->w0aa)
    {
      unsigned char status31 = V32W_BYTE(obj, w030, 1);
      short reg96 = hdx->rtdResult;
      V32W_BYTE(obj, w032, 0) = (unsigned char)(V32W_BYTE(obj, w032, 0) | 8);
      V32W_BYTE(obj, w030, 0) = 0xf;
      V32W_BYTE(obj, w030, 1) = (unsigned char)((status31 & 0xbf) | 0x20);
      StoreReg(obj, reg96, 0);
      hdx = V32W_PG(V32Hdx*, obj, (int)offsetof(V32Wrap, hdxPtr));
      hdx->subState = V32ST_F2;
      hdx->txBudget = 0x10;
      hdx->rxElapsed = 0;
      V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, txHandler), (v32fn_t)TxHdxNoCarrier);
      V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, rxHandler), (v32fn_t)RxHdxData);
    }
    break;

  case V32ST_G:
  {
    /* V32ST_G -> V32ST_H: carrier with a fresh 0x100 sample budget; the echo canceller's round-trip delay comes from register 0. */
    short reg0;

    hdx->subState = V32ST_H;
    hdx->txBudget = 0x100;
    hdx->rxElapsed = 0;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, txHandler), (v32fn_t)TxHdxCarrierState);
    reg0 = (short)LoadReg(obj, 0);
    SetECRndTripDelayV32(obj, (unsigned short)reg0);
    SetTxModeV32(obj, V32MODE_HANDSHAKE);
    InitGenSequence(obj, 1, 4, 2);
    break;
  }

  case V32ST_H:
    /* V32ST_H -> V32ST_I: 16 sample tick with a new transmit pattern. */
    hdx->subState = V32ST_I;
    hdx->txBudget = 0x10;
    hdx->rxElapsed = 0;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, txHandler), (v32fn_t)TxHdxCarrierState);
    InitGenSequence(obj, 0xe, 4, 2);
    break;

  case V32ST_I:
    /* V32ST_I -> V32ST_J: start the training segment (TxHdxTRN, 256 samples) from a freshly seeded scrambler. */
    hdx->subState = V32ST_J;
    hdx->txBudget = 0x100;
    hdx->rxElapsed = 0;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, txHandler), (v32fn_t)TxHdxTRN);
    InitGenSequence(obj, 0xf, 4, 2);
    SeedScramblerV32(obj, 0);
    break;

  case V32ST_J:
    /* V32ST_J -> V32ST_K: scrambled-sequence transmission for 1024 samples, echo canceller adaptation mode 2. */
    hdx->subState = V32ST_K;
    hdx->txBudget = 0x400;
    hdx->rxElapsed = 0;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, txHandler), (v32fn_t)TxHdxScrSequence);
    SetAdaptEcV32(obj, 2);
    break;

  case V32ST_K:
    /* V32ST_K -> V32ST_L: the long training segment (0x1a6c samples), echo canceller adaptation mode 3. */
    hdx->subState = V32ST_L;
    hdx->txBudget = 0x1a6c;
    hdx->rxElapsed = 0;
    SetAdaptEcV32(obj, 3);
    break;

  case V32ST_L:
  {
    /* V32ST_L -> V32ST_M, unconditional: code the local rate (GetRateV32 -> RateToSeq -> CodeRateSeq) into register 1 and start sending it. Receive with
     * RxHdxSTone; the tone detector's table is set to V32_S_DATA_COEF and the detector re-created from itself (see V32ST_N of V32OrgNextState()). The coded
     * rate is passed zero-extended to InitGenSequence() and sign-extended to StoreReg(). */
    unsigned short rate;
    unsigned short seq;
    short codedRate;
    FpmMtd* mtdObj;

    hdx->subState = V32ST_M;
    hdx->rxElapsed = 0;
    hdx->txBudget = (int)hdx->stateLimit;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, rxHandler), (v32fn_t)RxHdxSTone);
    mtdObj = V32H_PG(FpmMtd*, hdx, (int)offsetof(V32Hdx, mtd));
    MTD_COEF_SET(mtdObj, V32_S_DATA_COEF);
    MTD_RECREATE(mtdObj);
    rate = (unsigned short)GetRateV32(obj);
    seq = RateToSeq(obj, (short)rate);
    codedRate = (short)CodeRateSeq(obj, seq);
    StoreReg(obj, codedRate, 1);
    InitGenSequence(obj, (unsigned short)codedRate, 0x10, 2);
    SetTxModeV32(obj, V32MODE_4800);
    SetAdaptEcV32(obj, 1);
    break;
  }

  case V32ST_M:
  {
    /* V32ST_M -> V32ST_N: the transmit budget is register 0 + the turn-around delay (`w094`); the transmitter is silent, the receiver takes data. */
    short reg0 = (short)LoadReg(obj, 0);
    short tad;

    hdx->subState = V32ST_N;
    tad = hdx->w094;
    hdx->rxElapsed = 0;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, txHandler), (v32fn_t)TxHdxNoCarrier);
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, rxHandler), (v32fn_t)RxHdxData);
    hdx->txBudget = reg0 + tad;
    break;
  }

  case V32ST_N:
  {
    /* V32ST_N -> V32ST_O: receive with RxHdxSTone, detector threshold 0x708, tone word 3 = 0x747a. */
    FpmTone* toneObj;

    hdx->subState = V32ST_O;
    hdx->txBudget = (int)hdx->stateLimit;
    SetToneDetect(obj, 0x708);
    hdx = V32W_PG(V32Hdx*, obj, (int)offsetof(V32Wrap, hdxPtr));
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, rxHandler), (v32fn_t)RxHdxSTone);
    toneObj = V32H_PG(FpmTone*, hdx, (int)offsetof(V32Hdx, toneA));
    toneObj->r0.s[3] = 0x747a;
    break;
  }

  case V32ST_O:
    /* V32ST_O -> V32ST_P: symbol-timing (epoch) recovery in the handshake receive mode. */
    hdx->subState = V32ST_P;
    hdx->rxElapsed = 0;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, rxHandler), (v32fn_t)RxHdxEpoch);
    hdx->txBudget = (int)hdx->stateLimit;
    SetRxModeV32(obj, V32MODE_HANDSHAKE);
    SetRxLoopsV32(obj, 2);
    break;

  case V32ST_P:
    /* V32ST_P -> V32ST_Q: transmit budget 0x610, silent transmitter, receive data with equaliser mode 2. */
    hdx->subState = V32ST_Q;
    hdx->txBudget = 0x610;
    hdx->rxElapsed = 0;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, txHandler), (v32fn_t)TxHdxNoCarrier);
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, rxHandler), (v32fn_t)RxHdxData);
    SetAdaptEqV32(obj, 2);
    break;

  case V32ST_Q:
    /* V32ST_Q -> V32ST_T: listen for the far end's rate sequence (RxHdxRateSequence, 4800 mode, equaliser mode 3). */
    hdx->subState = V32ST_T;
    hdx->rxElapsed = 0;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, rxHandler), (v32fn_t)RxHdxRateSequence);
    hdx->txBudget = (int)hdx->stateLimit;
    SetRxModeV32(obj, V32MODE_4800);
    SetAdaptEqV32(obj, 3);
    InitDetSequence(obj, 0x1110111, 0xf111f111, UINT_MAX, 2);
    break;

  case V32ST_T:
  {
    /* V32ST_T -> V32ST_U: carrier, 0x100 sample budget; the bit sequence detected so far (16 bits) is kept in register 2 and decoded against the local rate
     * capabilities. If no rate is in common (V32RC_NONE) the failure status (flag 2, code 0x17) is posted, but the handshake goes on. */
    unsigned short reg2;
    unsigned char cls;

    hdx->subState = V32ST_U;
    hdx->txBudget = 0x100;
    hdx->rxElapsed = 0;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, txHandler), (v32fn_t)TxHdxCarrierState);
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, rxHandler), (v32fn_t)RxHdxData);
    SetTxModeV32(obj, V32MODE_HANDSHAKE);
    InitGenSequence(obj, 1, 4, 2);
    reg2 = (unsigned short)GetSequence(obj);
    StoreReg(obj, (short)reg2, 2);
    cls = DecodeRateSeq(obj, (unsigned short)LoadReg(obj, 2));
    if (cls == V32RC_NONE)
    {
      V32W_BYTE(obj, w030, 1) = (unsigned char)(V32W_BYTE(obj, w030, 1) | 2);
      V32W_BYTE(obj, w030, 0) = 0x17;
    }
    break;
  }

  case V32ST_U:
    /* V32ST_U -> V32ST_V: 16 sample tick with a new transmit pattern. */
    hdx->subState = V32ST_V;
    hdx->txBudget = 0x10;
    hdx->rxElapsed = 0;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, txHandler), (v32fn_t)TxHdxCarrierState);
    InitGenSequence(obj, 0xe, 4, 2);
    break;

  case V32ST_V:
    /* V32ST_V -> V32ST_W: start the acknowledgement training segment (TxHdxTRN, 256 samples) from a freshly seeded scrambler, echo canceller adaptation
     * mode 1. */
    hdx->subState = V32ST_W;
    hdx->txBudget = 0x100;
    hdx->rxElapsed = 0;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, txHandler), (v32fn_t)TxHdxTRN);
    InitGenSequence(obj, 0xf, 4, 2);
    SeedScramblerV32(obj, 0);
    SetAdaptEcV32(obj, 1);
    break;

  case V32ST_W:
    /* V32ST_W -> V32ST_X: scrambled-sequence transmission for 0x1bbc samples. */
    hdx->subState = V32ST_X;
    hdx->txBudget = 0x1bbc;
    hdx->rxElapsed = 0;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, txHandler), (v32fn_t)TxHdxScrSequence);
    break;

  case V32ST_X:
  {
    /* V32ST_X -> V32ST_X2: listen for the far end's sequence (RxHdxSequence, equaliser mode 1) and send the coded final rate (register 2,
     * CodeFinalRateSeq()). */
    unsigned short reg2;
    short codedFinalRate;

    hdx->subState = V32ST_X2;
    hdx->rxElapsed = 0;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, txHandler), (v32fn_t)TxHdxScrSequence);
    hdx->txBudget = (int)hdx->stateLimit;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, rxHandler), (v32fn_t)RxHdxSequence);
    reg2 = (unsigned short)LoadReg(obj, 2);
    codedFinalRate = (short)CodeFinalRateSeq(obj, reg2);
    InitGenSequence(obj, (unsigned short)codedFinalRate, 0x10, 2);
    SetTxModeV32(obj, V32MODE_4800);
    InitDetSequence(obj, 0x111f111, 0xf111f111, UINT_MAX, 2);
    SetAdaptEqV32(obj, 1);
    break;
  }

  case V32ST_Y:
  {
    /* V32ST_Y -> V32ST_Z: 8 sample budget (`rxElapsed` is not cleared here). The receiver switches to the mode of the rate class in register 4, the coded
     * acknowledgement sequence is sent, and the gain control is frozen. */
    unsigned short reg4;
    unsigned char cls;
    unsigned short eseq;

    hdx->subState = V32ST_Z;
    hdx->txBudget = 8;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, txHandler), (v32fn_t)TxHdxScrSequence);
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, rxHandler), (v32fn_t)RxHdxData);
    reg4 = (unsigned short)LoadReg(obj, 4);
    cls = DecodeRateSeq(obj, reg4);
    SetRxModeV32(obj, V32_RX_MODE[cls]);
    reg4 = (unsigned short)LoadReg(obj, 4);
    eseq = CodeESeq(obj, reg4);
    InitGenSequence(obj, eseq, 0x10, 2);
    FPM_AGC_Freeze(V32E_AGC(V32W_PG(V32Eng*, obj, (int)offsetof(V32Wrap, engPtr))));
    break;
  }

  case V32ST_Z:
  {
    /* V32ST_Z -> V32ST_END: the idle pattern is started before the state is written (the V32Hdx pointer is re-read after the call). Transmit budget 0x80;
     * the transmitter switches to the mode of the rate class in register 4. */
    unsigned short reg4;
    unsigned char cls;

    InitGenSequence(obj, 0xffff, 0x10, 8);
    hdx = V32W_PG(V32Hdx*, obj, (int)offsetof(V32Wrap, hdxPtr));
    hdx->subState = V32ST_END;
    hdx->txBudget = 0x80;
    hdx->rxElapsed = 0;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, txHandler), (v32fn_t)TxHdxScrSequence);
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, rxHandler), (v32fn_t)RxHdxData);
    reg4 = (unsigned short)LoadReg(obj, 4);
    cls = DecodeRateSeq(obj, reg4);
    SetTxModeV32(obj, V32_TX_MODE[cls]);
    break;
  }

  case V32ST_END:
  {
    /* V32ST_END, terminal: the state stays V32ST_END. Each call posts the "connected" flags and the connect code V32_CONNECT[class] of register 4's rate
     * class, sets the equaliser and the receive loops to mode 3 and clears the event flags of the equaliser history. */
    unsigned short reg4;
    unsigned char cls;
    V32Eng* eng;
    V32Hist* hist;

    hdx->subState = V32ST_END;
    hdx->rxElapsed = 0;
    hdx->txBudget = (int)hdx->stateLimit;
    V32W_BYTE(obj, w030, 1) = (unsigned char)(V32W_BYTE(obj, w030, 1) | 0x19);
    reg4 = (unsigned short)LoadReg(obj, 4);
    cls = DecodeRateSeq(obj, reg4);
    V32W_BYTE(obj, w030, 0) = (char)V32_CONNECT[cls];
    SetAdaptEqV32(obj, 3);
    SetRxLoopsV32(obj, 3);
    eng = V32W_PG(V32Eng*, obj, (int)offsetof(V32Wrap, engPtr));
    hist = V32E_PG(V32Hist*, eng, (int)offsetof(V32Eng, fse.histObj));
    hist->evFlags = 0;
    break;
  }

  case V32ST_F2:
  {
    /* V32ST_F2 -> V32ST_G: no guard; entered after V32ST_F. Loops back to V32ST_G, running the training segments again, with a one-frame budget and
     * TxHdxFinishFrame as the transmitter. */
    short symLen = hdx->frameLen;

    hdx->subState = V32ST_G;
    hdx->rxElapsed = 0;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, txHandler), (v32fn_t)TxHdxFinishFrame);
    hdx->txBudget = (int)symLen;
    break;
  }

  case V32ST_X2:
  {
    /* V32ST_X2 -> V32ST_Y: align the transmit budget to the 8-sample grid, 8 - ((stateLimit - budget) & 7); the detected bits (16) go to register 4. */
    int phaseInc = (int)hdx->stateLimit;
    int curBudget = (int)hdx->txBudget;
    short seq;

    hdx->subState = V32ST_Y;
    hdx->txBudget = (unsigned int)(8 - ((phaseInc - curBudget) & 7));
    seq = (short)GetSequence(obj);
    StoreReg(obj, seq, 4);
    break;
  }

  case V32ST_ERROR:
    /* any state may land here after an error: only the transmit budget is refreshed */
    hdx->txBudget = (int)hdx->stateLimit;
    break;

  default:
    /* no case for the other values of the state; nothing to do */
    break;
  }
}

/*
 * Mid-call rate renegotiation: V32RngInitNextState() (initiator, V32ROLE_RNG_INIT) and V32RngRespNextState() (responder, V32ROLE_RNG_RESP).
 *
 * The link is already trained when a renegotiation starts, so both machines skip the carrier and tone phases of the start-up machines and go straight to the
 * exchange of the rate sequences (12 and 11 cases against 25 and 29). The two work as a pair: each case of one side is answered by a handler of the other.
 * Their handlers are TxHdxData (responder, V32ST_A only), RxHdxSequenceE (initiator V32ST_D, responder V32ST_B) and RxHdxToneData (initiator, V32ST_A / B /
 * C). The "still waiting" branches of initiator V32ST_B and V32ST_C both end by re-installing RxHdxToneData; the two statements are repeated in each case.
 *
 * The first case of each machine sets the first word of the engine block (`*(int *)bigbuf = 1`), the carrier-present word V32FP_status() tests: nothing in
 * the renegotiation path sets it otherwise.
 *
 * The modes passed to SetTxModeV32() / SetRxModeV32() are fixed adaptation constants in the first cases and V32_TX_MODE[class] / V32_RX_MODE[class] in the
 * last ones. Mode 1 (rate class 0) does not touch the Viterbi decoder; any higher agreed class reaches VTBv32_init() through SetRxModeV32().
 */

/**
 * @brief Initiating side of a mid-call rate renegotiation: leave the current state of V32Hdx::subState and enter the next one.
 * @param obj  V.32 wrapper object (V32Wrap)
 *
 * V32NextState[V32ROLE_RNG_INIT]. Timers and guards as in V32OrgNextState(). The tone parameters depend on the role in V32Wrap::protoSel (0 = originate):
 * V32ST_A starts the carrier and the tone detector (600 or 1800), V32ST_B / V32ST_C wait for the budget while RxHdxToneData watches the line, V32ST_D picks
 * up the far end's sequence, V32ST_D2 sends the acknowledgement, V32ST_E / V32ST_F switch the transmitter to the agreed mode, V32ST_G listens, V32ST_DONE
 * decides the outcome.
 * @bug V32ST_DONE: when either decoded rate sequence (register 4 or register 1) is V32RC_NONE it posts failure code 0xe with flag 2 and enters
 *      V32ST_CLEARDOWN, but the statements after the `if` run for both outcomes: they set the "connected" flags, replace the transmit budget and overwrite the
 *      code with V32_CONNECT[class of register 4].  The code 0xe survives only when register 4 itself decodes to V32RC_NONE (V32_CONNECT[V32RC_NONE] is 0xe);
 *      when only register 1 does, the call reports the connect code of register 4's class although the state is V32ST_CLEARDOWN.
 */
void V32RngInitNextState(V32Wrap* obj)
{
  V32Hdx* hdx = V32W_PG(V32Hdx*, obj, (int)offsetof(V32Wrap, hdxPtr));
  V32Eng* eng = V32W_PG(V32Eng*, obj, (int)offsetof(V32Wrap, engPtr));
  short role = obj->protoSel;

  switch (hdx->subState)
  {
  case V32ST_A:
    SetAdaptEqV32(obj, 1);
    SetAdaptEcV32(obj, 1);
    SetRxLoopsV32(obj, 1);
    hdx->subState = V32ST_B;
    eng->opt[0] = 1;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, txHandler), (v32fn_t)TxHdxCarrierState);

    hdx->txBudget = 0x38;
    SetTxModeV32(obj, V32MODE_HANDSHAKE);
    InitGenSequence(obj, (unsigned short)(role == V32ROLE_ORG ? 0 : 3), 4, 2);
    V32W_BYTE(obj, w030, 1) = (unsigned char)((V32W_BYTE(obj, w030, 1) & 0xf7) | 4);

    SetToneDetect(obj, (unsigned short)(role == V32ROLE_ORG ? 600 : 1800));
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, rxHandler), (v32fn_t)RxHdxToneData);
    hdx->rxElapsed = 0;
    break;
  case V32ST_B:
    if ((int)hdx->txBudget > 0)
    {
      /* the tone budget has not run out yet: keep waiting */
      V32W_BYTE(obj, w030, 1) &= 0xfb;
    }
    else
    {
      hdx->subState = V32ST_C;
      hdx->txBudget = 8;
      InitGenSequence(obj, (unsigned short)((role == V32ROLE_ORG ? 3 : 0) + 0xc), 4, 2);
    }
    /* both paths (waiting and moving on) keep the tone-data receive handler */
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, rxHandler), (v32fn_t)RxHdxToneData);
    break;
  case V32ST_C:
    if ((int)hdx->txBudget < 1)
    {
      short r;
      hdx->subState = V32ST_D;
      V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, txHandler), (v32fn_t)TxHdxScrSequence);
      hdx->txBudget = (int)hdx->stateLimit;
      if ((V32W_BYTE(obj, w030, 1) & 4) == 0)
      {
        V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, rxHandler), (v32fn_t)RxHdxSequence);
        SetRxModeV32(obj, V32MODE_4800);
        InitDetSequence(obj, 0x1110111, 0xf111f111, UINT_MAX, 2);
      }

      r = (short)GetRateV32(obj);
      r = (short)RateToSeq(obj, r);
      r = (short)CodeRateSeq(obj, (unsigned short)r);
      StoreReg(obj, r, 0);
      InitGenSequence(obj, (unsigned short)r, 0x10, 2);

      SetTxModeV32(obj, V32MODE_4800);
      SeedScramblerV32(obj, 0);
      eng->smc.w008 = (unsigned short)(role == V32ROLE_ORG ? 0xc : 0);
      break;
    }
    /* budget not yet exhausted: same wait as in state B */
    V32W_BYTE(obj, w030, 1) &= 0xfb;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, rxHandler), (v32fn_t)RxHdxToneData);
    break;
  case V32ST_D:
    if ((V32W_BYTE(obj, w030, 1) & 4) == 0)
    {
      short seq = (short)GetSequence(obj);
      unsigned int budget, adjust;
      StoreReg(obj, seq, 1);
      hdx->subState = V32ST_D2;
      hdx->rxElapsed = 0;
      hdx->reg[4] = 0;
      V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, rxHandler), (v32fn_t)RxHdxSequenceE);

      InitDetSequence(obj, 0x111f111, 0xf111f111, UINT_MAX, 2);

      adjust = hdx->stateLimit - hdx->txBudget;
      if ((int)adjust > 0x3f)
      {
        budget = 8 - (adjust & 7);
      }
      else
      {
        budget = 0x40 - adjust;
      }
      hdx->txBudget = budget;
    }
    else
    {
      V32W_BYTE(obj, w030, 1) &= 0xfb;
      V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, rxHandler), (v32fn_t)RxHdxSequence);
      SetRxModeV32(obj, V32MODE_4800);
      InitDetSequence(obj, 0x1110111, 0xf111f111, UINT_MAX, 2);
    }
    break;
  case V32ST_D2:
    SetAdaptEqV32(obj, 3);
    SetRxLoopsV32(obj, 3);
    hdx->subState = V32ST_E;
    hdx->txBudget = 8;
    {
      short r = (short)LoadReg(obj, 1);
      r = (short)CodeESeq(obj, (unsigned short)r);
      InitGenSequence(obj, (unsigned short)r, 0x10, 2);
    }
    break;
  case V32ST_E:
  {
    short r = (short)LoadReg(obj, 1);
    unsigned char cls = DecodeRateSeq(obj, (unsigned short)r);
    hdx->subState = V32ST_F;
    hdx->txBudget = 0x18;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, txHandler), (v32fn_t)TxHdxScrSequence);
    SetTxModeV32(obj, V32_TX_MODE[cls]);
    InitGenSequence(obj, 0xffff, 0x10, 8);
    if (cls > 2)
    {
      eng->smc.transVal = 0;
    }
    break;
  }
  case V32ST_F:
  {
    short r = (short)LoadReg(obj, 1);
    unsigned char cls = DecodeRateSeq(obj, (unsigned short)r);
    V32W_BYTE(obj, w030, 1) |= 8;
    SetTxModeV32(obj, V32_TX_MODE[cls]);
    InitGenSequence(obj, 0xffff, 0x10, 8);
    if (hdx->reg[4] == 0)
    {
      hdx->rxElapsed = 0;
      hdx->subState = V32ST_G;
      V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, rxHandler), (v32fn_t)RxHdxSequence);
      hdx->txBudget = (int)hdx->stateLimit;
      SetRxModeV32(obj, V32MODE_4800);
    }
    else
    {
      hdx->rxElapsed = 0;
      hdx->subState = V32ST_DONE;
      hdx->txBudget = 0x18 - (short)hdx->w048;
    }
    break;
  }
  case V32ST_G:
  {
    unsigned short seq = (unsigned short)GetSequence(obj);
    unsigned char cls;
    hdx->subState = V32ST_DONE;
    hdx->txBudget = 0x24;
    hdx->rxElapsed = 0;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, rxHandler), (v32fn_t)RxHdxData);
    hdx->reg[4] = seq;
    cls = DecodeRateSeq(obj, hdx->reg[4]);
    SetRxModeV32(obj, V32_RX_MODE[cls]);
    SetAdaptEqV32(obj, 3);
    SetRxLoopsV32(obj, 3);
    break;
  }
  case V32ST_DONE:
  {
    unsigned char farLocal = DecodeRateSeq(obj, hdx->reg[4]);
    unsigned char farRemote = DecodeRateSeq(obj, hdx->reg[1]);
    if (farRemote == 6 || farLocal == 6)
    {
      hdx->txBudget = 0x40;
      V32W_BYTE(obj, w030, 1) |= 2;
      V32W_BYTE(obj, w030, 0) = 0xe;
      hdx->subState = V32ST_CLEARDOWN;
    }
    hdx->rxElapsed = 0;
    hdx->txBudget = (int)hdx->stateLimit;
    V32W_BYTE(obj, w030, 1) |= 5;
    V32W_BYTE(obj, w030, 0) = (char)V32_CONNECT[farLocal];
    V32E_PG(V32Hist*, eng, (int)offsetof(V32Eng, fse.histObj))->evFlags = 0;
    break;
  }
  case V32ST_ERROR:
    hdx->txBudget = (int)hdx->stateLimit;
    break;
  default:
    break;
  }
}

/**
 * @brief One step of the handshake of the RESPONDER of a mid-call rate renegotiation (role V32ROLE_RNG_RESP).
 * @param obj  V.32 wrapper (V32Wrap) created by V32FP_create()
 *
 * Counterpart of V32RngInitNextState(); entered the same way (V32FP_control() resets V32Hdx::subState to V32ST_A, or v32_data() switches the role to
 * V32ROLE_RNG_RESP when RenegotiateDetectV32() reports a request).  The responder keeps transmitting data bits until the far end's rate sequence arrives:
 * A sends data bits, arms the sequence detector and stores the own coded rate sequence in register 0; B takes the far end's sequence (register 1) and starts
 * the tone pattern (0 when originating, 3 when answering); C and D wait out the budgets and send the scrambled own sequence; E sends the E sequence;
 * F and G decode the far end's rate class and select the transmit mode; H takes the final sequence (register 4) and selects the receive mode; DONE
 * publishes the connect status.
 * @note State F latches the failure flag, status 0x17 and V32ST_CLEARDOWN when the rate class is V32RC_NONE.  State DONE has a single class
 *       decode (register 4) and no incompatibility branch, unlike the initiator.
 */
void V32RngRespNextState(V32Wrap* obj)
{
  V32Hdx* hdx = V32W_PG(V32Hdx*, obj, (int)offsetof(V32Wrap, hdxPtr));
  V32Eng* eng = V32W_PG(V32Eng*, obj, (int)offsetof(V32Wrap, engPtr));
  short role = obj->protoSel;

  switch (hdx->subState)
  {
  case V32ST_A:
  {
    short r;
    SetAdaptEqV32(obj, 1);
    SetRxLoopsV32(obj, 1);
    SetAdaptEcV32(obj, 1);
    hdx->subState = V32ST_B;
    eng->opt[0] = 1;

    hdx->txBudget = (int)hdx->stateLimit;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, txHandler), (v32fn_t)TxHdxData);
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, rxHandler), (v32fn_t)RxHdxSequence);
    hdx->rxElapsed = 0;

    InitDetSequence(obj, 0x1110111, 0xf111f111, 0xffff, 2);
    SetRxModeV32(obj, V32MODE_4800);
    V32W_BYTE(obj, w030, 1) &= 0xfb;

    r = (short)GetRateV32(obj);
    r = (short)RateToSeq(obj, r);
    r = (short)CodeRateSeq(obj, (unsigned short)r);
    StoreReg(obj, r, 0);
    break;
  }
  case V32ST_B:
  {
    short seq;
    SetAdaptEqV32(obj, 3);
    SetRxLoopsV32(obj, 3);
    hdx->subState = V32ST_C;
    hdx->reg[4] = 0;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, txHandler), (v32fn_t)TxHdxCarrierState);
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, rxHandler), (v32fn_t)RxHdxSequenceE);
    hdx->txBudget = 0x38;
    hdx->rxElapsed = 0;

    InitGenSequence(obj, (unsigned short)(role == V32ROLE_ORG ? 0 : 3), 4, 2);
    SetTxModeV32(obj, V32MODE_HANDSHAKE);

    seq = (short)GetSequence(obj);
    StoreReg(obj, seq, 1);
    InitDetSequence(obj, 0x111f111, 0xf111f111, UINT_MAX, 2);

    V32W_BYTE(obj, w030, 1) &= 0xf7;
    break;
  }
  case V32ST_C:
  {
    /* wait 8 units, then the tone pattern 0xf (originating) or 0xc (answering) */
    unsigned short pattern = (unsigned short)(role == V32ROLE_ORG ? 0xf : 0xc);
    hdx->subState = V32ST_D;
    hdx->w084 = 0;
    hdx->txBudget = 8;
    hdx->rxElapsed = 0;
    InitGenSequence(obj, pattern, 4, 2);
    break;
  }
  case V32ST_D:
  {
    short r;
    unsigned short v;
    hdx->subState = V32ST_E;
    hdx->txBudget = 0x40;
    hdx->rxElapsed = 0;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, txHandler), (v32fn_t)TxHdxScrSequence);

    SetTxModeV32(obj, V32MODE_4800);

    r = (short)LoadReg(obj, 0);
    r = (short)CodeRateSeq(obj, (unsigned short)r);
    InitGenSequence(obj, (unsigned short)r, 0x10, 2);
    SeedScramblerV32(obj, 0);

    v = (unsigned short)(role != V32ROLE_ORG ? 0 : 0xc);
    eng->smc.w008 = v;
    break;
  }
  case V32ST_E:
  {
    short r = (short)LoadReg(obj, 1);
    r = (short)CodeESeq(obj, (unsigned short)r);
    hdx->subState = V32ST_F;
    hdx->txBudget = 8;
    hdx->rxElapsed = 0;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, txHandler), (v32fn_t)TxHdxScrSequence);
    InitGenSequence(obj, (unsigned short)r, 0x10, 2);
    break;
  }
  case V32ST_F:
  {
    short r = (short)LoadReg(obj, 1);
    unsigned char cls = DecodeRateSeq(obj, (unsigned short)r);
    hdx->subState = V32ST_G;
    SetTxModeV32(obj, V32_TX_MODE[cls]);
    if (cls == V32RC_NONE)
    {
      /* no rate in common: latch the failure flag and status 0x17, then clear down */
      hdx->txBudget = 0x28;
      V32W_BYTE(obj, w030, 1) |= 2;
      V32W_BYTE(obj, w030, 0) = 0x17;
      hdx->subState = V32ST_CLEARDOWN;
    }
    else
    {
      InitGenSequence(obj, 0xffff, 0x10, 8);
      hdx->txBudget = 0x18;
      V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, txHandler), (v32fn_t)TxHdxScrSequence);
      if (cls > 2)
      {
        eng->smc.transVal = 0;
      }
    }
    break;
  }
  case V32ST_G:
  {
    short r = (short)LoadReg(obj, 1);
    unsigned char cls = DecodeRateSeq(obj, (unsigned short)r);
    V32W_BYTE(obj, w030, 1) |= 8;
    SetTxModeV32(obj, V32_TX_MODE[cls]);
    if (hdx->reg[4] == 0)
    {
      V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, rxHandler), (v32fn_t)RxHdxSequence);
      hdx->subState = V32ST_H;
      hdx->txBudget = (int)hdx->stateLimit;
    }
    else
    {
      hdx->rxElapsed = 0;
      hdx->subState = V32ST_DONE;
      hdx->txBudget = 0x18 - (short)hdx->w048;
    }
    break;
  }
  case V32ST_H:
  {
    unsigned short seq = (unsigned short)GetSequence(obj);
    unsigned char cls;
    hdx->subState = V32ST_DONE;
    hdx->txBudget = 0x24;
    hdx->rxElapsed = 0;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, rxHandler), (v32fn_t)RxHdxData);
    hdx->reg[4] = seq;
    cls = DecodeRateSeq(obj, hdx->reg[4]);
    SetRxModeV32(obj, V32_RX_MODE[cls]);
    break;
  }
  case V32ST_DONE:
  {
    unsigned char cls = DecodeRateSeq(obj, hdx->reg[4]);
    hdx->rxElapsed = 0;
    hdx->txBudget = (int)hdx->stateLimit;
    V32W_BYTE(obj, w030, 1) |= 5;
    V32W_BYTE(obj, w030, 0) = (char)V32_CONNECT[cls];
    V32E_PG(V32Hist*, eng, (int)offsetof(V32Eng, fse.histObj))->evFlags = 0;
    break;
  }
  case V32ST_ERROR:
    hdx->txBudget = (int)hdx->stateLimit;
    break;
  default:
    break;
  }
}

/**
 * @brief Create a V.32 / V.32bis engine from the six-word configuration of the host glue.
 * @param args      six ints: [0] nonzero = answering side (wrapper protocol selector 1; 0 = originate), [1] look-ahead (stored as V32Wrap::lookAhead),
 *                  [2] handshake timeout in ms (V32Wrap::timeoutWord), [3] margin (V32Wrap::margin), [4] bit 0 -> option bit 2 of V32Wrap::b011 (when set the
 *                  creator leaves the handshake in state V32ST_A), [5] bit rate in the low 16 bits, used for both directions
 * @param reserved  passed through to V32FP_recreate() (ignored there)
 * @return the engine (the V32Wrap block), to be released with V32FP_delete()
 *
 * Builds the 12-word wrapper configuration (the default V32_CFG_raw with the fields above patched in; V32Wrap::highRate = 1 when the rate is above 7199 bit/s)
 * and creates the engine with V32FP_recreate().
 */
V32Wrap* V32FP_recreate(V32Wrap* obj, short* cfgIn, int unusedFlag);

V32Wrap* V32FP_create(int* args, int reserved)
{
  int cfg[12];
  unsigned char* cb = (unsigned char*)cfg;
  unsigned short txRate;
  unsigned char enableByte;

  memcpy(cfg, V32_CFG_raw, sizeof(cfg));

  txRate = *(unsigned short*)(args + 5);
  enableByte = *(unsigned char*)((char*)args + 16);

  MEMBER_AS_OF(short, V32Wrap, cb, protoSel) = (short)(args[0] != 0); /* originate / answer */
  MEMBER_AS_OF(short, V32Wrap, cb, txRate) = (short)txRate;           /* transmit rate */
  MEMBER_AS_OF(short, V32Wrap, cb, rxRate) = (short)txRate;           /* receive rate (same value) */
  MEMBER_AS_OF(int, V32Wrap, cb, timeoutWord) = args[2];              /* handshake timeout */
  MEMBER_AS_OF(short, V32Wrap, cb, lookAhead) = (short)args[1];       /* look-ahead */
  MEMBER_AS_OF(int, V32Wrap, cb, highRate) = (int)(0x1c1f < txRate);
  MEMBER_AS_OF(short, V32Wrap, cb, margin) = (short)args[3]; /* margin */
  cb[17] = (unsigned char)((cb[17] & 0xfb) | ((enableByte & 1) << 2));

  return V32FP_recreate(0, (short*)cfg, reserved);
}

/**
 * @brief Create or re-initialise a V.32 engine: allocate the three blocks, configure every sub-object and select the initial handshake state.
 * @param obj         an existing wrapper to re-initialise in place (its blocks and buffers are reused), or NULL to allocate the wrapper, the half-duplex
 *                    object, the engine block and their work buffers
 * @param cfgIn       the 12-word wrapper configuration (48 bytes, copied to the start of the wrapper), or NULL for the default V32_CFG_raw
 * @param unusedFlag  ignored
 * @return the wrapper
 *
 * The steps: copy the configuration; derive the transmit / receive rate classes (V32RateClass) and the handshake role from the protocol selector (0 originate,
 * 1 answer, otherwise local loop); program the transmit scrambler and the receive descrambler from the role-indexed tap tables; initialise the PPS, MRF,
 * ECC, AGC, SRE and FSE sub-objects with their configuration blocks; set up the Viterbi sub-object for the receive rate class; wire the SMC encoder
 * table and the circular output descriptor; create the three handshake tone generators and the multi-tone detector; initialise the half-duplex handler
 * (receive handler RxHdxTone, transmit handler TxHdxNull when originating and TxHdxTone when answering); fill the wrapper's diagnostic descriptor; and finally
 * pre-position the handshake (originate: state B2, answer: state B) unless option bit 2 of V32Wrap::b011 is set.
 * @warning The allocation results are not checked and the new blocks are not cleared, so every field the code does not store stays indeterminate.
 *          When `obj == cfgIn` (V32FP_control() re-initialises in place) the 48-byte copy has identical source and destination, which memcpy() does not
 *          define.
 * @bug V32Hdx::stateLimit (the handshake timeout in symbol intervals) is computed as `(int)(short)((timeoutWord * 0x4ccc) >> 13)`, which wraps for timeouts
 *      above 13653 ms: the 60000 ms of the host glue gives 12922 intervals (about 5.4 s at 2400 symbols/s) instead of 143994.
 */
V32Wrap* V32FP_recreate(V32Wrap* obj, short* cfgIn, int unusedFlag)
{
  int fresh = (obj == 0);
  V32Hdx* hdx;
  V32Eng* eng;
  short txRate, rxRate;
  int txRateClass, rxRateClass;
  int eccDelay, eccNearTaps;
  unsigned int symMask;
  void* rxBitBuf;
  int cfgLastWord;

  (void)unusedFlag;

  if (fresh)
  {
    /* the blocks include the address-slot tails of v32.h (V32_WRAP_ALLOC / V32_ENG_ALLOC / V32_HDX_ALLOC on 64-bit hosts); the slots and the embedded FPM
     * objects are set up by the FPM_*_init calls below */
    obj = (V32Wrap*)(sysdep_malloc(V32_WRAP_ALLOC));
    eng = (V32Eng*)(sysdep_malloc(V32_ENG_ALLOC));
    V32W_PS(V32Eng*, obj, (int)offsetof(V32Wrap, engPtr), eng);
    hdx = (V32Hdx*)(sysdep_malloc(V32_HDX_ALLOC));
    V32W_PS(V32Hdx*, obj, (int)offsetof(V32Wrap, hdxPtr), hdx);
    V32H_PS(char*, hdx, (int)offsetof(V32Hdx, toneA), 0);
    V32H_PS(char*, hdx, (int)offsetof(V32Hdx, mtd), 0);
    rxBitBuf = sysdep_malloc(100);
    V32H_PS(char*, hdx, (int)offsetof(V32Hdx, rxBitBuf), (char*)(rxBitBuf));
    V32E_PS(char*, eng, (int)offsetof(V32Eng, copyBufA), (char*)(sysdep_malloc(0x154)));
    V32E_PS(char*, eng, (int)offsetof(V32Eng, copyBufB), (char*)(sysdep_malloc(0x154)));
  }
  else
  {
    hdx = V32W_PG(V32Hdx*, obj, (int)offsetof(V32Wrap, hdxPtr));
    eng = V32W_PG(V32Eng*, obj, (int)offsetof(V32Wrap, engPtr));
    rxBitBuf = V32H_PG(void*, hdx, (int)offsetof(V32Hdx, rxBitBuf));
  }

  /* the wrapper starts with the 12-word configuration */
  {
    const short* src = cfgIn ? cfgIn : (const short*)(const void*)V32_CFG_raw;
    memcpy(obj, src, 48);
  }
  /* the fresh run-time fields: the warm-up counter restarts, the disconnect threshold is entry 3 of the table, and the configuration word d024 (0 in
   * the default configuration) is the last word of the MRF, ECC, FSE and tone configuration blocks below */
  obj->warmupCalls = 0;
  cfgLastWord = obj->d024;
  obj->discThresh = V32DiconnectThreshTable[3];

  {
    /* the engine's leading option words: 0 .. 6 enabled, 7 .. 9 the three option bits of V32Wrap::b010 */
    unsigned char optByte = obj->b010;
    eng->opt[7] = optByte & 1;
    eng->opt[8] = (optByte >> 1) & 1;
    eng->opt[9] = (optByte >> 2) & 1;
    eng->opt[0] = 1;
    eng->opt[1] = 1;
    eng->opt[2] = 1;
    eng->opt[3] = 1;
    eng->opt[4] = 1;
    eng->opt[5] = 1;
    eng->opt[6] = 1;
  }

  /* the rate classes from the configured bit rates; 9600 is the 16-point class unless V32Wrap::highRate selects the trellis-coded one */
  txRate = obj->txRate;
  if (txRate == RATE_14400)
    txRateClass = V32RC_14400;
  else if (txRate == RATE_12000)
    txRateClass = V32RC_12000;
  else if (txRate == RATE_9600)
    txRateClass = (obj->highRate == 0) ? V32RC_9600 : V32RC_9600_TCM;
  else if (txRate == RATE_7200)
    txRateClass = V32RC_7200;
  else
    txRateClass = V32RC_4800;
  eng->txRateClass = (short)txRateClass;

  rxRate = obj->rxRate;
  if (rxRate == RATE_14400)
    rxRateClass = V32RC_14400;
  else if (rxRate == RATE_12000)
    rxRateClass = V32RC_12000;
  else if (rxRate == RATE_9600)
    rxRateClass = (obj->highRate == 0) ? V32RC_9600 : V32RC_9600_TCM;
  else if (rxRate == RATE_7200)
    rxRateClass = V32RC_7200;
  else
    rxRateClass = V32RC_4800;
  eng->rxRateClass = (short)rxRateClass;

  /* the handshake role from the protocol selector: 0 originate, 1 answer, otherwise local loop */
  {
    short protoSel = *(short*)obj;
    short curRole = (protoSel == 0) ? V32ROLE_ORG : (protoSel == 1) ? V32ROLE_ANS : V32ROLE_LOCLOOP;
    hdx->role = curRole;
  }

  /* the transmit scrambler (SDM): symbol width 2 bits (rate class 0) or 4, the role-indexed tap from SDMv32_GPC, the 23-bit register length, the derived tap
   * shifts and the symbol / keep masks; sdmTapShiftA and d030 are read back below */
  {
    unsigned short symWidth = 2;
    unsigned short gpcTap;
    short role76 = hdx->role;
    short regLen = 23;
    short shiftWidth;

    if (eng->txRateClass != 0)
      symWidth = 4;
    gpcTap = (unsigned short)SDMv32_GPC[role76];
    eng->sdm.d000 = ((unsigned int)gpcTap << 16) | symWidth;
    eng->sdm.shiftReg = 0;

    eng->sdm.regLen = regLen;
    shiftWidth = (short)eng->sdm.symWidth;
    eng->sdm.tapShiftA = (short)(eng->sdm.tapPos - shiftWidth);
    symMask = (1u << ((unsigned char)shiftWidth & 0x1f)) - 1;
    eng->sdm.tapShiftB = (short)(regLen - shiftWidth);
    eng->sdm.symMask = symMask;
    eng->sdm.keepMask = ~symMask;
    eng->smc.rateIdx = (unsigned short)(eng->txRateClass != 0);
    eng->smc.w002 = 0;
    /* the SMC encoder state words must start at zero: a fresh allocation is not cleared, and the trellis encoder (SMCv32_encoder_tcm) indexes
     * TrellisTransitionTable with its state */
    eng->smc.phase = 0;
    eng->smc.w008 = 0;
    eng->smc.w00a = 0;
    eng->smc.trellisState = 0;
    eng->smc.transVal = 0;
    eng->smc.w012 = 0;
  }

  /* pulse-shaping filter */
  FPM_PPS_init(&eng->pps, PPSv32_CFG_raw, fresh);

  /* matched receive filter */
  {
    int mrfCfg[4];
    memcpy(mrfCfg, MRFv32_CFG_raw, sizeof(mrfCfg));
    mrfCfg[3] = cfgLastWord;
    FPM_MRF_init(V32E_MRF(eng), mrfCfg, fresh);
  }

  /* echo canceller: the delay in symbol units is derived from the look-ahead (0.3 per unit, Q15), clamped at 0 */
  {
    int eccCfg[6];
    V32Ecc* ecc = &eng->ecc;
    eccDelay = (int)(((short)obj->lookAhead)) * 0x2666 + 0x8000;
    ecc->w066 = 0x30;
    /* the sum is at least 0x8000 for a non-negative look-ahead, so the clamp only matters for a negative one */
    ecc->w064 = (eccDelay < 0) ? 0 : (unsigned short)(eccDelay >> 0xf);
    memcpy(eccCfg, ECCv32_CFG_raw, sizeof(eccCfg));
    eccCfg[5] = cfgLastWord;
    FPM_ECC_init(ecc, eccCfg, fresh);
    /* the three fields of the echo canceller's every-third-sample tap injection (circular read indexes and their shared modulus) are forced to
     * (0, 0, 0x30) after FPM_ECC_init, which computes other values for them from the configuration */
    ecc->codeRingLen = 0x30;
    ecc->codeRdNear = 0;
    ecc->codeRdFar = 0;
  }

  /* receive AGC (embedded in the engine); the first three words are its initial ring state */
  eng->fse.w010 = 0x30;
  eng->fse.nTaps = 0;
  eng->fse.w00e = 0;
  FPM_AGC_init(V32E_AGC(eng), AGCv32_CFG_raw, fresh);

  /* symbol-rate estimator (timing recovery) and its fixed loop constants */
  FPM_SRE_init(&eng->sre, SREv32_CFG_raw, fresh);
  eng->sre.w07c = 0xc;
  eng->sre.w08a = 0x2580;
  eng->sre.ppmNMax = 0x68;
  eng->sre.ppmStep =
      /* low 16 bits of configuration word 0 (signed) times 0x2580 as the divisor; the result is stored but no code reads w1d0 back */
      (short)(1000000 / (long long)((long long)(short)SREv32_CFG_raw[0] * 0x2580));

  /* fractionally-spaced equalizer: the last three configuration words are the decision-history object, the initial decision callback (FSE_decision_AB) and
   * the common last word */
  {
    int fseCfg[14];
    memcpy(fseCfg, FSEv32_CFG_raw, sizeof(fseCfg));
#if UINTPTR_MAX == 0xffffffffu
    fseCfg[11] = (int)(intptr_t)&eng->hist;
    fseCfg[12] = (int)(intptr_t)(void*)FSE_decision_AB;
    fseCfg[13] = cfgLastWord;
    FPM_FSE_init(&eng->fse, fseCfg, fresh);
#else
    /* 64-bit hosts: the two address words of the configuration (history object, first decision callback) are the engine-tail slots fseHistObj / fseDecision
     * (v32.h); FPM_FSE_init copies the 14 words into the image only, so the slots are stored after the init */
    fseCfg[11] = 0;
    fseCfg[12] = 0;
    fseCfg[13] = cfgLastWord;
    FPM_FSE_init(&eng->fse, fseCfg, fresh);
    V32E_PS(V32Hist*, eng, (int)offsetof(V32Eng, fse.histObj), &eng->hist);
    V32E_PS(v32fn_t, eng, (int)offsetof(V32Eng, fse.decisionFn), (v32fn_t)FSE_decision_AB);
#endif
  }

  /* clear the FSE decision history and the Viterbi bookkeeping words that follow it */
  eng->hist.lfsr = 0;
  eng->hist.stateCount = 0;
  eng->hist.w050 = 0;
  eng->hist.w05e = 0;
  eng->hist.w060 = 0;
  eng->hist.epochFlag = 0;
  eng->hist.evFlags = 0;
  eng->hist.d068 = 0;
  eng->hist.retrainCount = 0;
  memset(&eng->hist.symI0, 0, 6 * 2);
  memset(&eng->hist.w008, 0, 2 * 2);
  eng->hist.w000 = 0;
  eng->hist.w004 = 0;
  eng->hist.w006 = 0;

  /* the Viterbi sub-object (V32Vtb): a 0x200-byte history buffer (allocated on creation, zeroed every time), the traceback depth 0x10, and cleared write
   * index and previous quadrant; the rate-dependent table slots follow */
  if (fresh)
  {
    void* vtbScratch = sysdep_malloc(0x200);
    V32E_PS(void*, eng, (int)offsetof(V32Eng, hist.vtb.histBuf), vtbScratch);
  }
  {
    char* vtbScratch = V32E_PG(char*, eng, (int)offsetof(V32Eng, hist.vtb.histBuf));
    int i;
    for (i = 0; i < 0x80; i++)
    {
      *(short*)(vtbScratch + 2 + i * 4) = 0;
      *(short*)(vtbScratch + i * 4) = 0;
    }
  }
  eng->hist.vtb.writeIdx = 0;
  eng->hist.vtb.prevQuadrant = 0;
  eng->hist.vtb.tracebackDepth = 0x10;

  /* the Viterbi tables for the receive rate class: classes 3 (7200), 2 (9600 trellis), 4 (12000) and 5 / 6 select the 2 / 4 / 6 / 8-row sets
   * (rowSize 2, 4, 6, 8; low mask 7, 0xf, 0x1f, 0x3f); classes 0 and 1, which do not use the trellis decoder, get the largest set */
  {
    short rc = rxRateClass;
    short trellisSize;
    if (rc == 3)
    {
      eng->hist.vtb.family = 1;
      CFGP_SLOT_SET(const void*, &eng->hist.vtb.iMap, VTBv32_IMAP16T);
      CFGP_SLOT_SET(const void*, &eng->hist.vtb.qMap, VTBv32_QMAP16T);
      CFGP_SLOT_SET(const void*, &eng->hist.vtb.boundTab, VTB_BOUND_7200);
      CFGP_SLOT_SET(const void*, &eng->hist.vtb.regionTab, VTB_REGION_7200);
      trellisSize = 2;
      eng->hist.vtb.lowMask = 7;
    }
    else if (rc < 4)
    {
      if (rc == 2)
      {
        eng->hist.vtb.family = 2;
        CFGP_SLOT_SET(const void*, &eng->hist.vtb.iMap, VTBv32_IMAP32);
        CFGP_SLOT_SET(const void*, &eng->hist.vtb.qMap, VTBv32_QMAP32);
        CFGP_SLOT_SET(const void*, &eng->hist.vtb.boundTab, VTB_BOUND_9600);
        CFGP_SLOT_SET(const void*, &eng->hist.vtb.regionTab, VTB_REGION_9600);
        trellisSize = 4;
        eng->hist.vtb.lowMask = 0xf;
      }
      else
      {
        eng->hist.vtb.family = 4;
        CFGP_SLOT_SET(const void*, &eng->hist.vtb.iMap, VTBv32_IMAP128);
        CFGP_SLOT_SET(const void*, &eng->hist.vtb.qMap, VTBv32_QMAP128);
        CFGP_SLOT_SET(const void*, &eng->hist.vtb.boundTab, VTB_BOUND_14400);
        CFGP_SLOT_SET(const void*, &eng->hist.vtb.regionTab, VTB_REGION_14400);
        trellisSize = 8;
        eng->hist.vtb.lowMask = 0x3f;
      }
    }
    else if (rc == 4)
    {
      eng->hist.vtb.family = 3;
      CFGP_SLOT_SET(const void*, &eng->hist.vtb.iMap, VTBv32_IMAP64);
      CFGP_SLOT_SET(const void*, &eng->hist.vtb.qMap, VTBv32_QMAP64);
      CFGP_SLOT_SET(const void*, &eng->hist.vtb.boundTab, VTB_BOUND_12000);
      CFGP_SLOT_SET(const void*, &eng->hist.vtb.regionTab, VTB_REGION_12000);
      trellisSize = 6;
      eng->hist.vtb.lowMask = 0x1f;
    }
    else
    {
      eng->hist.vtb.family = 4;
      CFGP_SLOT_SET(const void*, &eng->hist.vtb.iMap, VTBv32_IMAP128);
      CFGP_SLOT_SET(const void*, &eng->hist.vtb.qMap, VTBv32_QMAP128);
      CFGP_SLOT_SET(const void*, &eng->hist.vtb.boundTab, VTB_BOUND_14400);
      CFGP_SLOT_SET(const void*, &eng->hist.vtb.regionTab, VTB_REGION_14400);
      trellisSize = 8;
      eng->hist.vtb.lowMask = 0x3f;
    }
    eng->hist.vtb.rowSize = trellisSize;
    eng->hist.vtb.pathMetric[0] = 0;
    memset(&eng->hist.vtb.pathMetric[1], 0, 7 * 2);
    eng->hist.vtb.diffShift = eng->hist.vtb.family;
  }

  /* the receive descrambler (SDM): same shape as the transmit scrambler above, with the receive tap table SDMv32_GPA (role-indexed) and the shared symbol
   * width; the per-tap shift amounts are derived from the tap and the mode field */
  {
    short role76 = hdx->role;
    unsigned short symWidth = eng->sdm.symWidth;
    unsigned short gpaHi = (unsigned short)SDMv32_GPA[role76];
    short regLenRx = eng->sdm.regLen;
    short modeField;

    eng->w5098 = eng->smc.rateIdx;
    eng->w509a = eng->smc.w002;
    memset(&eng->b50a0, 0, 2 * 2);
    eng->w509e = 0;
    eng->w50a8 = 0;
    eng->w50a6 = 0;
    eng->w509c = (unsigned short)(((short)eng->w5098 != 0) ? 2 : 0);
    eng->w50aa = 0;
    eng->rxSdm.shiftReg = 0;
    eng->rxSdm.d000 = ((unsigned int)gpaHi << 16) | symWidth;
    eng->rxSdm.regLen = regLenRx;
    modeField = eng->rxSdm.symWidth;
    eng->rxSdm.tapShiftA = (short)(gpaHi - modeField);
    symMask = (1u << ((unsigned char)modeField & 0x1f)) - 1;
    eng->rxSdm.tapShiftB = (short)(regLenRx - modeField);
    eng->circ.readIdx = 0;
    eng->circ.writeIdx = 0;
    eng->rxSdm.symMask = symMask;
    eng->rxSdm.keepMask = ~symMask;
  }

  /* the SMC encoder table (differential / absolute-phase / trellis-coded), the 16-point constellation maps and the circular output descriptor, whose
   * ring length is the symbol length in samples of the wrapper's rate index (V32Wrap::w018, 0 or 1) */
  V32E_PS(char*, eng, (int)offsetof(V32Eng, circ.codeBuf), V32E_PG(char*, eng, (int)offsetof(V32Eng, ecc.codeHist)));
  {
    short symLen = V32_SYMBOL_LEN[obj->w018];
    V32E_PS(v32fn_t, eng, (int)offsetof(V32Eng, d098), (v32fn_t)SMCv32_encoder_dif);
    V32E_PS(v32fn_t, eng, (int)offsetof(V32Eng, d09c), (v32fn_t)SMCv32_encoder_abs);
    V32E_PS(v32fn_t, eng, (int)offsetof(V32Eng, d0a0), (v32fn_t)SMCv32_encoder_tcm);
    CFGP_SLOT_SET(const short*, &eng->b0a8, SMCv32_IMAP16);
    CFGP_SLOT_SET(const short*, &eng->b0ac, SMCv32_QMAP16);
    eng->circ.ringLen16 = symLen;
    eng->smcEncSel = 0;
    eng->ecc.codeRingLen = symLen;
  }

  /* the three handshake tone generators share one configuration; only the frequency (low 16 bits of word 0) differs: A 2100, B 1800 when originating
   * and 600 when answering, C 3000 */
  {
    int toneCfg[9];
    toneCfg[1] = 0x747a0000;
    toneCfg[2] = 0x127ae;
    toneCfg[3] = (FPM_TONE_CFG_raw[3] & (int)0xffff0000u) | 0x7c00;
    toneCfg[4] = FPM_TONE_CFG_raw[4]; /* ToneLPF pointer */
    toneCfg[5] = FPM_TONE_CFG_raw[5];
    toneCfg[6] = cfgLastWord;
    toneCfg[7] = 0x280000;
    toneCfg[8] = FPM_TONE_CFG_raw[8];

    if (fresh)
      V32H_PS(char*, hdx, (int)offsetof(V32Hdx, toneA), 0);
    toneCfg[0] = 0x16a10834;
    V32H_PS(FpmTone*, hdx, (int)offsetof(V32Hdx, toneA), FPM_TONE_create(V32H_PG(FpmTone*, hdx, (int)offsetof(V32Hdx, toneA)), toneCfg));

    if (fresh)
      V32H_PS(char*, hdx, (int)offsetof(V32Hdx, toneB), 0);
    toneCfg[0] = (hdx->role == V32ROLE_ORG) ? 0x708 : 600;
    V32H_PS(FpmTone*, hdx, (int)offsetof(V32Hdx, toneB), FPM_TONE_create(V32H_PG(FpmTone*, hdx, (int)offsetof(V32Hdx, toneB)), toneCfg));

    if (fresh)
      V32H_PS(char*, hdx, (int)offsetof(V32Hdx, toneC), 0);
    toneCfg[0] = 3000;
    V32H_PS(FpmTone*, hdx, (int)offsetof(V32Hdx, toneC), FPM_TONE_create(V32H_PG(FpmTone*, hdx, (int)offsetof(V32Hdx, toneC)), toneCfg));
  }

  /* the half-duplex handler: the timing constants come from the rate index (V32Wrap::w016, 0 or 1), the handshake timeout in ms becomes a symbol count
   * (2.4 symbols per ms) and the handlers are the initial ones */
  {
    short rateIdx01 = obj->w016; /* 0/1-valued, NOT the 0-6 rate class */
    int optWord = obj->timeoutWord;
    int phaseInc = (int)(short)((optWord * 0x4ccc) >> 0xd);
    short symLen = V32_SYMBOL_LEN[rateIdx01];

    /* the state starts at A with no elapsed time (a fresh allocation is not cleared) */
    hdx->subState = V32ST_A;
    hdx->rxElapsed = 0;
    hdx->stateLimit = phaseInc;
    V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, rxHandler), (v32fn_t)RxHdxTone);
    hdx->w084 = symLen;
    hdx->frameLen = symLen;
    hdx->w0a0 = V32_SAMPLE_LEN[rateIdx01];
    hdx->w094 = V32_TURNAROUND_DLY[rateIdx01];
    hdx->w09a = 0;
    hdx->w098 = (short)(symLen + 5);
    hdx->w09a = (short)(((int)(short)40 * 0x2666 + 0x4000) >> 0xf);
    hdx->w09c = (short)(((int)((short)obj->lookAhead) * 0x2666 + 0x4000) >> 0xf);

    if (hdx->role == V32ROLE_ORG)
    {
      V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, txHandler), (v32fn_t)TxHdxNull);
      hdx->txBudget = phaseInc;
    }
    else
    {
      V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, txHandler), (v32fn_t)TxHdxTone);
      hdx->txBudget = 0x1ef0;
    }
    hdx->w0ae = 0;
    hdx->toneMisses = 0;
  }

  SetAdaptEcV32(obj, 1);
  SetAdaptEqV32(obj, 1);
  SetRxLoopsV32(obj, 1);
  SetTxModeV32(obj, V32MODE_HANDSHAKE);
  SetRxModeV32(obj, V32MODE_4800);

  /* the history object that FPM_FSE_init stored in the engine slot gets the receive descrambler tap shift */
  V32E_PG(V32Hist*, eng, (int)offsetof(V32Eng, fse.histObj))->lfsrShift = eng->rxSdm.tapShiftA;

  memset((char*)hdx + offsetof(V32Hdx, reg), 0, 7 * 2); /* the five registers and the two words behind them */
  hdx->d088 = 0;

  /* multi-tone detector over the V32_S_DATA_COEF table */
  {
    int mtdCfg[3];
    mtdCfg[0] = CFGP(V32_S_DATA_COEF);
    mtdCfg[1] = (0x747a << 16) | 3;
    mtdCfg[2] = 1;
    V32H_PS(FpmMtd*, hdx, (int)offsetof(V32Hdx, mtd), FPM_MTD_create(V32H_PG(FpmMtd*, hdx, (int)offsetof(V32Hdx, mtd)), mtdCfg));
  }

  /* handshake AGC (at the start of the half-duplex object) and the retrain bookkeeping words */
  FPM_AGC_init(V32H_AGC(hdx), AGCv32Prc_CFG_raw, fresh);

  eng->w50d4 = 0;
  eng->w50d6 = 0;
  eng->w50d8 = 0;
  obj->w030 = 0;
  obj->w032 = 0;
  V32W_BYTE(obj, w030, 1) = (unsigned char)(V32W_BYTE(obj, w030, 1) | 0x40);
  V32W_BYTE(obj, w030, 0) = 1;

  /* status word: code 1, flag bit 6 set; then the wrapper's descriptor of the FSE symbol / tap buffers and the echo canceller taps (address slots and
   * lengths; nothing in src/v32 reads them back) */
  V32W_PS(char*, obj, (int)offsetof(V32Wrap, d034), V32E_PG(char*, eng, (int)offsetof(V32Eng, fse.symIBuf)));
  V32W_PS(char*, obj, (int)offsetof(V32Wrap, d038), V32E_PG(char*, eng, (int)offsetof(V32Eng, fse.symQBuf)));
  V32W_PS(char*, obj, (int)offsetof(V32Wrap, d040), V32E_PG(char*, eng, (int)offsetof(V32Eng, fse.iTaps)));
  eccNearTaps = eng->ecc.nearTaps;
  V32W_PS(char*, obj, (int)offsetof(V32Wrap, d044), V32E_PG(char*, eng, (int)offsetof(V32Eng, fse.qTaps)));
  V32W_PS(char*, obj, (int)offsetof(V32Wrap, d04c), V32E_PG(char*, eng, (int)offsetof(V32Eng, ecc.tapSets)));
  V32W_PS(char*, obj, (int)offsetof(V32Wrap, d050), V32E_PG(char*, eng, (int)offsetof(V32Eng, ecc.tapSets)) + eccNearTaps * 2);
  obj->w048 = 0x31;
  V32W_PS(char*, obj, (int)offsetof(V32Wrap, d058), V32E_PG(char*, eng, (int)offsetof(V32Eng, ecc.tapSets)) + eccNearTaps * 4);
  V32W_PS(char*, obj, (int)offsetof(V32Wrap, d03c), (char*)&eng->fse.outIdx);
  obj->w054 = (short)eccNearTaps;
  V32W_PS(char*, obj, (int)offsetof(V32Wrap, d05c), V32E_PG(char*, eng, (int)offsetof(V32Eng, ecc.tapSets)) + (eccNearTaps * 2 + (short)eng->ecc.w004) * 2);
  obj->w060 = (short)eng->ecc.w004;

  /* pre-position the handshake unless option bit 2 is set: the originator starts in B2 with the 600 detector and the phase-reversal receive handler, the
   * answerer in B */
  {
    short protoSel = *(short*)obj;
    if (protoSel == 0)
    {
      if ((obj->b011 & 4) == 0)
      {
        hdx->subState = V32ST_B2;
        InitGenSequence(obj, 0, 4, 2);
        (V32H_PG(FpmTone*, hdx, (int)offsetof(V32Hdx, toneA)))->r0.s[4] = 0x4000;
        SetToneDetect(obj, 600);
        hdx->phsRevLatch = 0;
        V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, txHandler), (v32fn_t)TxHdxCarrierState);
        V32H_PS(v32fn_t, hdx, (int)offsetof(V32Hdx, rxHandler), (v32fn_t)RxHdxPhsReversal);
        hdx->txBudget = (int)hdx->stateLimit;
        hdx->rxElapsed = 0;
        protoSel = *(short*)obj;
      }
    }
    if (protoSel == 1 && (obj->b011 & 4) == 0)
    {
      hdx->subState = V32ST_B;
      hdx->txBudget = ((obj->b011 >> 2) & 1) ? 0xb4u : 0x8cu;
      SetToneDetect(obj, 0);
    }
  }

  if (1 < dsplibs_debug_level)
  {
    dsplibs_debug_printf("V32FP version: %s %s\n", "Sep 22 2005", "15:48:07");
  }

  return obj;
}

/**
 * @brief Release an engine: the tone generators, the detector, the equalizer, estimator, canceller, filter sub-objects, the work buffers and the three blocks.
 * @param obj  the wrapper returned by V32FP_create(); it is freed too
 * @warning `obj` must not be used afterwards.
 */
void V32FP_delete(V32Wrap* obj)
{
  V32Hdx* hdx = V32W_PG(V32Hdx*, obj, (int)offsetof(V32Wrap, hdxPtr));
  V32Eng* eng = V32W_PG(V32Eng*, obj, (int)offsetof(V32Wrap, engPtr));

  FPM_MTD_delete(V32H_PG(FpmMtd*, hdx, (int)offsetof(V32Hdx, mtd)));
  FPM_TONE_delete(V32H_PG(FpmTone*, hdx, (int)offsetof(V32Hdx, toneC)));
  FPM_TONE_delete(V32H_PG(FpmTone*, hdx, (int)offsetof(V32Hdx, toneB)));
  FPM_TONE_delete(V32H_PG(FpmTone*, hdx, (int)offsetof(V32Hdx, toneA)));
  sysdep_free(V32E_PG(void*, eng, (int)offsetof(V32Eng, hist.vtb.histBuf)));
  FPM_FSE_free(&eng->fse);
  FPM_SRE_free(&eng->sre);
  FPM_ECC_free(&eng->ecc);
  FPM_MRF_free(V32E_MRF(eng));
  FPM_PPS_free(&eng->pps);
  sysdep_free(V32E_PG(void*, eng, (int)offsetof(V32Eng, copyBufA)));
  sysdep_free(V32E_PG(void*, eng, (int)offsetof(V32Eng, copyBufB)));
  sysdep_free(V32H_PG(void*, hdx, (int)offsetof(V32Hdx, rxBitBuf)));
  sysdep_free((void*)hdx);
  sysdep_free((void*)eng);
  sysdep_free(obj);
}

/**
 * @brief Read diagnostic data from the equalizer (FSE_getdiag() on the engine's FSE sub-object).
 * @param obj       the wrapper
 * @param mode      selector, passed through
 * @param outBuf    result buffer, passed through
 * @param maxCount  capacity of `outBuf`, passed through
 */
void V32FP_GetDiagnostics(V32Wrap* obj, int mode, int* outBuf, int maxCount)
{
  V32Eng* eng = V32W_PG(V32Eng*, obj, (int)offsetof(V32Wrap, engPtr));
  FSE_getdiag(&eng->fse, mode, outBuf, maxCount);
}

/**
 * @brief Return the samples of the last V32FP_modem() block after echo cancellation.
 * @param obj       the wrapper
 * @param outCount  receives the number of 16-bit samples in the block, or 0 when the block was longer than 160
 * @return the engine's copy buffer (valid until the next V32FP_modem() call; owned by the engine)
 */
char* V32FP_GetCleanedSamples(V32Wrap* obj, int* outCount)
{
  V32Eng* eng = V32W_PG(V32Eng*, obj, (int)offsetof(V32Wrap, engPtr));
  unsigned short count = eng->w50d4;

  if (count < 0xa1)
  {
    *outCount = (short)count;
  }
  else
  {
    *outCount = 0;
  }
  return V32E_PG(char*, eng, (int)offsetof(V32Eng, copyBufB));
}

/**
 * @brief Compute the signal-to-noise estimate and fill the caller's status snapshot; start a retrain when the SNR stays below the rate's threshold.
 * @param obj     the wrapper
 * @param status  22 shorts (44 bytes): [0] protocol status code (PROTOCOL[role]), [1] / [2] connected transmit / receive bit rate, [3] decision-error
 *                indicator, [4] SNR estimate, [5] echo-path estimate, [6] AGC history, [7] 0, [8] / [9] timing-loop values, [10] option flags (low byte:
 *                bits 0 .. 2 the engine options, 3 .. 5 the inverted enables, 7 the retrain-detect option; high byte bit 0: option bit 2 of V32Wrap::b011),
 *                [12 .. 13] the symbol-rate estimator's PLL enable word, [14 .. 15] 1 when the AGC state word differs from 1, [16 .. 19] cleared
 * @return always 1
 *
 * The decision-error average is smoothed and converted to a fixed-point SNR estimate through FPM_div() and FPM_log10(); the first 200 calls
 * after creation report the fixed value 40 (V32Wrap::warmupCalls).  When the SNR is below SnrToRetrainTable[connected receive class] the call counts a
 * drop in V32Wrap::snrDropCount; the tenth consecutive drop (the counter has reached 9) adds the full-reconnect option 0x400, resets the counters and sets
 * V32Eng::w50d8.  Every drop passes the block (a copy of V32_CTL) to V32FP_control().
 * @note The thresholds are at most 24, so the warm-up value 40 can never trigger a retrain.
 */
int V32FP_control(V32Wrap* obj, short* ctl);

int V32FP_status(V32Wrap* obj, short* status)
{
  V32Eng* eng = V32W_PG(V32Eng*, obj, (int)offsetof(V32Wrap, engPtr));
  short connectRate = eng->rxConnClass;
  unsigned int idx = (unsigned int)connectRate;
  short snr = 0x28;
  short decErrIdx = 0;
  short otherEst = 0;
  short decErr;

  if (idx > 5)
    idx = 5;

  decErr = (short)(((int)(eng->w50d6) * 0x7333 + 0x4000) >> 0xf) + (short)((((int)(unsigned short)(eng->fse.decErrAvg) >> 2) * 0xccd + 0x4000) >> 0xf);
  eng->w50d6 = decErr;

  if (decErr != 0)
  {
    short divOut1, divOut2;
    int errProd;
    /* the reciprocal of the smoothed error: the mantissa (divOut1, unsigned) scales the reference, the shift (divOut2) is the exponent for the logarithm */
    FPM_div((unsigned short)decErr, &divOut1, &divOut2);
    errProd = (int)(short)(((-(unsigned short)((unsigned short)(connectRate - 4) < 2) & 0x4000) + 0x1400) - decErr) * (int)(unsigned short)divOut1;
    if ((short)((unsigned int)errProd >> 16) != 0 && -1 < errProd)
    {
      short logOut = (short)FPM_log10((unsigned short)((errProd >> 16) & 0xffff), (short)~divOut2);
      decErrIdx = (short)(((logOut >> 3) * 10) + 0x100) >> 9;
    }
    else
    {
      decErrIdx = -0x3c;
    }

    if (eng->ecc.errPower == 0 || eng->ecc.inPower == 0)
    {
      otherEst = 0;
    }
    else
    {
      short divOut1b, divOut2b;
      short logOut2;
      FPM_div((unsigned short)eng->ecc.errPower, &divOut1b, &divOut2b);
      logOut2 = (short)FPM_log10((unsigned short)(((int)eng->ecc.inPower * (int)(unsigned short)divOut1b) >> 0xf), (short)-divOut2b);
      otherEst = (short)(((int)logOut2 >> 3) * 0xa0000 >> 0x19);
    }
    snr = decErrIdx;
  }

  if (obj->warmupCalls < 200)
  {
    obj->warmupCalls = (short)(obj->warmupCalls + 1);
    snr = 0x28;
  }

  /* the snapshot: protocol code of the role, the connected rates (transmit and receive connect classes of the engine), the error indicator and the SNRs */
  status[0] = PROTOCOL[((V32Hdx*)V32W_PG(char*, obj, (int)offsetof(V32Wrap, hdxPtr)))->role];
  status[1] = RATEv32[eng->txConnClass];
  status[2] = RATEv32[connectRate];
  {
    short statusWord = eng->fse.decErrAvg;
    status[3] = (short)(1 - (statusWord >> 1));
  }
  status[5] = otherEst;
  status[4] = snr;
  {
    unsigned char optBit0 = (unsigned char)(eng->opt[7] & 1);
    unsigned short agcHist = V32E_AGCF(unsigned short, eng, (int)(offsetof(V32Eng, agc) + offsetof(FpmAgc, hist)), hist);
    unsigned char* opt = (unsigned char*)(status + 10);
    unsigned char oldOpts = *opt;
    unsigned char optBit1, optBit2, optBit3, optBit4, optBit5;

    status[7] = 0;
    status[6] = (short)agcHist;
    status[8] = (unsigned short)eng->pps.phaseStep2;
    status[9] = eng->sre.loopInteg;

    *opt = (unsigned char)((oldOpts & 0xfe) | optBit0);
    optBit1 = (unsigned char)((eng->opt[8] & 1) * 2);
    *opt = (unsigned char)((oldOpts & 0xfc) | optBit0 | optBit1);
    /* option bit 2 is the engine option word 9 (written by V32FP_control() from bit 2 of its flag byte) */
    optBit2 = (unsigned char)((eng->opt[9] & 1) << 2);
    *opt = (unsigned char)((oldOpts & 0xf8) | optBit0 | optBit1 | optBit2);
    optBit3 = (unsigned char)(((int)eng->opt[0] == 0) << 3);
    *opt = (unsigned char)((oldOpts & 0xf0) | optBit0 | optBit1 | optBit2 | optBit3);
    optBit4 = (unsigned char)(((int)eng->opt[3] == 0) << 4);
    *opt = (unsigned char)((oldOpts & 0xe0) | optBit0 | optBit1 | optBit2 | optBit3 | optBit4);
    optBit5 = (unsigned char)(((int)eng->opt[4] == 0) << 5);
    *opt = (unsigned char)((oldOpts & 0x80) | optBit0 | optBit1 | optBit2 | optBit3 | optBit4 | optBit5);
    *opt = (unsigned char)(optBit0 | optBit1 | optBit2 | optBit3 | optBit4 | optBit5 | ((obj->b011 >> 1) << 7));
  }
  {
    unsigned char wrapFlags = obj->b011;
    unsigned char* b15 = (unsigned char*)status + 0x15; /* byte 21: the high byte of status[10] */
    memset(status + 0x10, 0, 4);
    memset(status + 0x12, 0, 4);
    *b15 = (unsigned char)((*b15 & 0xfe) | ((wrapFlags >> 2) & 1));
  }
  memcpy(status + 0xc, (void*)(&eng->sre.pllEnable), 4);

  {
    short thresh = SnrToRetrainTable[idx];
    *(int*)(status + 0xe) = (V32E_AGCF(int, eng, (int)(offsetof(V32Eng, agc) + offsetof(FpmAgc, detect)), detect) != 1) ? 1 : 0;
    if (snr < thresh)
    {
      /* SNR-drop retrain path: the control block is a copy of V32_CTL (16 shorts) whose word at shorts 6 / 7 (flag byte and option byte) gets the option bit
       * 0x400 (full reconnect) on the tenth consecutive drop, then the low three flag bits of the status options byte in its low byte, and bit 7 */
      short ctlBlk[16];
      unsigned int w12;
      unsigned char optb;
      memcpy(ctlBlk, V32_CTL, sizeof(ctlBlk));
      memcpy(&w12, &ctlBlk[6], 4);
      if (1 < dsplibs_debug_level)
      {
        dsplibs_debug_printf("V32STC - SNR drop observed, SNR = %d < threshold = %d\n", (int)snr, (int)thresh);
        if (1 < dsplibs_debug_level)
          dsplibs_debug_printf("Dec error = %d (*64)\n", (int)eng->w50d6);
      }
      if (obj->snrDropCount < 9)
      {
        obj->snrDropCount = (short)(obj->snrDropCount + 1);
      }
      else
      {
        if (1 < dsplibs_debug_level)
          dsplibs_debug_printf("%d coseq SNR drops detected local retrain is initiated\n", 8);
        w12 |= 0x400;
        obj->snrDropCount = 0;
        obj->warmupCalls = 0;
        eng->w50d8 = 1;
      }
      optb = *((unsigned char*)status + 0x14);
      w12 = (w12 & ~0xffu) | (unsigned int)(((w12 & 0xf8u) | (optb & 1) | (optb & 2) | (optb & 4)) & 0xff);
      w12 |= 0x80;
      memcpy(&ctlBlk[6], &w12, 4);
      V32FP_control(obj, ctlBlk);
      return 1;
    }
    obj->snrDropCount = 0;
  }
  return 1;
}

/**
 * @brief Apply a control block: update the engine options and, on request, start a rate renegotiation or a full reconnect.
 * @param obj  the wrapper
 * @param ctl  16 shorts, a copy of V32_CTL with changes: [0] requested bit rate; byte 12 flags (bits 0 .. 2 the option words 7 .. 9, bits 3 / 4 / 5 the
 *             inverted enables 0 / 3 / 4, bit 7 option bit 1 of V32Wrap::b011); byte 13 options (bit 2 full reconnect, bit 3 rate renegotiation, handled
 *             bits are cleared); bytes 16 .. 19 the V32Wrap::highRate word (9600 with trellis); words at bytes 24 and 28 stored as engine words 5 and 6;
 *             a nonzero word at byte 20 re-derives the half-duplex timing constants from the rate indexes V32Wrap::w018 / w016 and resets the status code to 1
 * @return always 1
 *
 * With bit 2 of the options clear and bit 3 set, the half-duplex state is reset to V32ST_A and V32RngRespNextState() (when the role is
 * V32ROLE_RNG_RESP) or V32RngInitNextState() (the role is forced to V32ROLE_RNG_INIT and the requested rate becomes the rate class) takes the first step.
 * With bit 2 set the engine is re-initialised in place with V32FP_recreate() and the handshake is restarted at state B through V32OrgNextState() or
 * V32AnsNextState() (the originator with
 * the elapsed counter preset to 0x960); the status becomes 0xb with flag bit 5.  When V32Eng::w50d8 (local retrain) is set the
 * configured rates are first lowered according to the connected receive class (classes 1 / 2 -> 7200, 3 -> 4800, 4 -> 9600, 5 -> 12000).
 * @note A status code of 9 (retrain requested) forces the full-reconnect option.
 */
int V32FP_control(V32Wrap* obj, short* ctl)
{
  V32Eng* eng = V32W_PG(V32Eng*, obj, (int)offsetof(V32Wrap, engPtr));
  unsigned char ctlOptByte = *(unsigned char*)((char*)ctl + 0xd);
  unsigned char ctl6 = *(unsigned char*)(ctl + 6);

  if ((char)V32W_BYTE(obj, w030, 0) == 9)
  {
    ctlOptByte |= 4;
    *(unsigned char*)((char*)ctl + 0xd) = ctlOptByte;
  }

  eng->opt[7] = ctl6 & 1;
  eng->opt[8] = (ctl6 >> 1) & 1;
  eng->opt[9] = (ctl6 >> 2) & 1;
  eng->opt[0] = (ctl6 & 8) == 0;
  eng->opt[3] = (ctl6 & 0x10) == 0;
  eng->opt[4] = (ctl6 & 0x20) == 0;
  eng->opt[5] = *(unsigned int*)(ctl + 0xc);
  eng->opt[6] = *(unsigned int*)(ctl + 0xe);
  obj->b011 = (unsigned char)((obj->b011 & 0xfd) | (unsigned char)(((char)ctl6 >> 7) * -2));

  if (*(int*)(ctl + 10) != 0)
  {
    V32Hdx* hdx = V32W_PG(V32Hdx*, obj, (int)offsetof(V32Wrap, hdxPtr));
    if ((char)V32W_BYTE(obj, w030, 0) != 0xf)
    {
      V32W_BYTE(obj, w030, 1) = (unsigned char)(V32W_BYTE(obj, w030, 1) | 2);
      V32W_BYTE(obj, w030, 0) = 0xe;
    }

    hdx->w09c = (short)((V32_SAMPLE_LEN[obj->w018] / V32_SAMPLE_LEN[obj->w016]) * hdx->w09c);
    hdx->w084 = V32_SYMBOL_LEN[obj->w018];
    hdx->frameLen = V32_SYMBOL_LEN[obj->w018];
    hdx->w0a0 = V32_SAMPLE_LEN[obj->w018];
    V32W_BYTE(obj, w030, 0) = 1;
  }

  if ((ctlOptByte & 4) == 0)
  {
    if ((ctlOptByte & 8) != 0)
    {
      /* mid-call rate renegotiation: restart the handshake state machine at state A for the role's side */
      V32Hdx* hdx = V32W_PG(V32Hdx*, obj, (int)offsetof(V32Wrap, hdxPtr));
      hdx->subState = V32ST_A;
      V32W_BYTE(obj, w030, 1) = (unsigned char)(V32W_BYTE(obj, w030, 1) & 0xfe);
      if (hdx->role == V32ROLE_RNG_RESP)
      {
        /* responder: clear the final-sequence register before state A runs */
        hdx->reg[4] = 0;
        V32RngRespNextState(obj);
        *(unsigned char*)((char*)ctl + 0xd) &= 0xf7;
      }
      else
      {
        /* initiator: take the requested rate and its trellis word (ctl bytes 16 .. 19, stored in V32Wrap::highRate) and derive the rate class from the
         * bit rate; the 9600 class is 1 or 2 depending on the trellis word */
        int rateWord;
        short rate;
        hdx->role = V32ROLE_RNG_INIT;
        rateWord = *(int*)((char*)ctl + 0x10);
        rate = *(short*)ctl;
        obj->highRate = rateWord;
        obj->txRate = rate;
        obj->rxRate = rate;
        if (rate == RATE_14400)
        {
          eng->txRateClass = 5;
        }
        else if (rate == RATE_12000)
        {
          eng->txRateClass = 4;
        }
        else if (rate == RATE_9600)
        {
          eng->txRateClass = (short)(2 - (rateWord == 0));
        }
        else if (rate == RATE_7200)
        {
          eng->txRateClass = 3;
        }
        else
        {
          eng->txRateClass = 0;
        }
        eng->rxRateClass = eng->txRateClass;
        V32RngInitNextState(obj);
        *(unsigned char*)((char*)ctl + 0xd) &= 0xf7;
      }
      return 1;
    }
  }
  else
  {
    /* full reconnect.  With a local retrain pending the rates are first stepped down from the connected receive class (overriding the rates the control
     * block set) */
    if (eng->w50d8 != 0)
    {
      short connectRate = eng->rxConnClass;
      switch (connectRate)
      {
      case 1:
      case 2:
        obj->rxRate = RATE_7200;
        obj->txRate = RATE_7200;
        break;
      case 3:
        obj->rxRate = RATE_4800;
        obj->txRate = RATE_4800;
        break;
      case 4:
        obj->rxRate = RATE_9600;
        obj->txRate = RATE_9600;
        break;
      case 5:
        obj->rxRate = RATE_12000;
        obj->txRate = RATE_12000;
        break;
      default:
        break;
      }
    }

    /* re-initialise in place and restart the handshake at state B.  The two tests are sequential on purpose: the selector is re-read after the first body,
     * so a selector changed by V32OrgNextState() would also run the second body */
    {
      short protoSel = *(short*)obj;
      if (protoSel == 0)
      {
        V32Hdx* hdx;
        V32FP_recreate(obj, (short*)obj, 0);
        hdx = V32W_PG(V32Hdx*, obj, (int)offsetof(V32Wrap, hdxPtr));
        hdx->subState = V32ST_B;
        hdx->rxElapsed = 0x960;
        V32OrgNextState(obj);
        protoSel = *(short*)obj;
      }
      if (protoSel == 1)
      {
        V32FP_recreate(obj, (short*)obj, 0);
        ((V32Hdx*)V32W_PG(char*, obj, (int)offsetof(V32Wrap, hdxPtr)))->subState = V32ST_B;
        V32AnsNextState(obj);
      }
    }

    V32W_BYTE(obj, w030, 1) = (unsigned char)(V32W_BYTE(obj, w030, 1) | 0x20);
    V32W_BYTE(obj, w030, 0) = 0xb;
    *(unsigned char*)((char*)ctl + 0xd) &= 0xfb;
  }
  return 1;
}

/* the transmit and receive handler slots of the half-duplex object (V32Hdx::txHandler / rxHandler) */
typedef short (*hdx_tx_t)(V32Wrap*, void*, short*, short*);
typedef short (*hdx_rx_t)(V32Wrap*, void*, void*, short*);

/** @brief Protocol slot that does nothing (roles 3, 7 and 8, which no state selects). */
void v32_null_protocol(void) {}

/**
 * @brief Run the transmit handler of the half-duplex object until it has produced a block.
 * @param obj       the wrapper
 * @param txBits    transmit data bits (the bit staging buffer V32Hdx::rxBitBuf)
 * @param out       receives the transmit samples
 * @param outCount  receives the number of samples produced
 *
 * The handler (V32Hdx::txHandler) is called with the frame length V32Hdx::frameLen as the sample budget; it lowers the budget by what it consumed and
 * the loop repeats until the budget is 0.  The handler slot is read again on every iteration because a handler may install its successor.
 */
void V32TxHdxModem(V32Wrap* obj, void* txBits, short* out, short* outCount)
{
  V32Hdx* hdx = V32W_PG(V32Hdx*, obj, (int)offsetof(V32Wrap, hdxPtr));
  short total = 0;
  short remaining = hdx->frameLen;

  for (;;)
  {
    hdx_tx_t handler = V32H_PG(hdx_tx_t, hdx, (int)offsetof(V32Hdx, txHandler));
    short produced = handler(obj, txBits, out, &remaining);
    total = (short)(total + produced);
    out += produced;
    if (remaining == 0)
      break;
    hdx = V32W_PG(V32Hdx*, obj, (int)offsetof(V32Wrap, hdxPtr));
  }
  *outCount = total;
}

/**
 * @brief Run the receive handler of the half-duplex object (V32Hdx::rxHandler) on one block.
 * @param obj        the wrapper
 * @param rxSamples  received line samples
 * @param outBits    receives the demodulated data
 * @param outCount   receives the output count
 * @return the handler's result
 */
short V32RxHdxModem(V32Wrap* obj, void* rxSamples, void* outBits, short* outCount)
{
  V32Hdx* hdx = V32W_PG(V32Hdx*, obj, (int)offsetof(V32Wrap, hdxPtr));
  hdx_rx_t handler = V32H_PG(hdx_rx_t, hdx, (int)offsetof(V32Hdx, rxHandler));
  return handler(obj, rxSamples, outBits, outCount);
}

/**
 * @brief Protocol slot of the pre-connect roles (originate, answer, local loop and the two renegotiation roles): one block through the half-duplex handlers.
 * @param obj        the wrapper
 * @param rxIn       the transmit data bits of this block (16-bit); the argument name follows the shared slot signature
 * @param txOut      receives the transmit samples
 * @param rxSamples  received line samples
 * @param rxOutBuf   receives the demodulated data
 * @param txBits     out: transmit sample count (the input value is not used)
 * @param outCount   out: receive output count
 *
 * Clears flag bit 0 of the status word (bits 2 and 3 as well for the renegotiation initiator), stages `frameLen` input bits in V32Hdx::rxBitBuf, runs the
 * receive handler and then the transmit handler on the staged bits.
 */
void v32_handshake(V32Wrap* obj, short* rxIn, void* txOut, void* rxSamples, void* rxOutBuf, void* txBits, short* outCount)
{
  V32Hdx* hdx = V32W_PG(V32Hdx*, obj, (int)offsetof(V32Wrap, hdxPtr));
  unsigned char flags = V32W_BYTE(obj, w030, 1);
  int i;

  if (hdx->role == V32ROLE_RNG_INIT)
    flags &= 0xf3;
  V32W_BYTE(obj, w030, 1) = (unsigned char)(flags & 0xfe);

  if (hdx->frameLen > 0)
  {
    short* dst = V32H_PG(short*, hdx, (int)offsetof(V32Hdx, rxBitBuf));
    for (i = 0; i < hdx->frameLen; i++)
      dst[i] = rxIn[i];
  }

  V32RxHdxModem(obj, rxSamples, rxOutBuf, (short*)outCount);
  V32TxHdxModem(obj, V32H_PG(void*, hdx, (int)offsetof(V32Hdx, rxBitBuf)), (short*)txOut, (short*)txBits);
}

/**
 * @brief Protocol slot of the connected role: scramble, modulate, demodulate and descramble one block, and watch for retrain and renegotiation requests.
 * @param obj        the wrapper
 * @param rxIn       the transmit data bits of this block (16-bit); the argument name follows the shared slot signature
 * @param txOut      receives the transmit samples
 * @param rxSamples  received line samples
 * @param rxOutBuf   receives the demodulated data bits
 * @param txCount    in: number of transmit bits; out: number of transmit samples
 * @param rxCount    in: number of received samples; out: number of demodulated bits
 *
 * The call first refreshes the status snapshot with V32FP_status() and takes the option flags from it (bit 0 scramble the transmit bits, bit 1
 * descramble the received bits, bit 2 transmit clock sync, bit 7 look for a retrain request).  A retrain request (RetrainDetectV32()) builds a control block
 * in the static `v32_ctl_stash`, reports status 9 and sets Control_Flag; the NEXT call applies that block with V32FP_control() instead of processing audio.
 * A renegotiation request (RenegotiateDetectV32()) makes the engine the responder (role V32ROLE_RNG_RESP), applies a control block at once and reports
 * status 0x1c.  Without flag bit 5 of the status word a call counter runs and status 0xc is reported once 5 times the count exceeds V32Wrap::margin.
 * @bug On the call after a retrain request only V32FP_control() runs: the counts are unchanged, no samples are produced and `txOut` / `rxOutBuf` are not
 *      written, yet V32FP_modem() then scales `*txCount` entries of `txOut` and copies `*rxCount` entries of its receive work buffer to the host.
 *      Stale buffer contents go out for that block.
 * @warning `Control_Flag` and `v32_ctl_stash` are static: all engines in a process share them, so two engines retraining at the same time would mix their
 *          control blocks.
 */
/** Set by v32_data() when a retrain request is pending; the next call applies the stashed control block. */
int Control_Flag = 0;
static short v32_ctl_stash[16]; /* the control block of a pending retrain request (see v32_data) */

void v32_data(V32Wrap* obj, short* rxIn, void* txOut, void* rxSamples, void* rxOutBuf, short* txCount, short* rxCount)
{
  short statusBuf[0x16];
  V32Hdx* hdx = V32W_PG(V32Hdx*, obj, (int)offsetof(V32Wrap, hdxPtr));
  unsigned char flagsByte;
  unsigned char newStatus = 0;

  /* refresh the status snapshot; this may itself call V32FP_control() */
  V32FP_status(obj, statusBuf);

  /* the option flags are the low byte of status word 10 */
  flagsByte = ((unsigned char*)&statusBuf[10])[0];

  if (Control_Flag != 1)
  {
    int i;
    if (hdx->frameLen > 0)
    {
      short* dst = V32H_PG(short*, hdx, (int)offsetof(V32Hdx, rxBitBuf));
      for (i = 0; i < hdx->frameLen; i++)
        dst[i] = rxIn[i];
    }
    if (flagsByte & 1)
    {
      ScrambleDataV32(obj, V32H_PG(void*, hdx, (int)offsetof(V32Hdx, rxBitBuf)), (unsigned short)*txCount);
    }
    *rxCount = (short)DemodDataV32(obj, rxSamples, rxOutBuf, (unsigned short)*rxCount);
    *txCount = (short)ModDataV32(obj, V32H_PG(void*, hdx, (int)offsetof(V32Hdx, rxBitBuf)), txOut, (unsigned short)*txCount);
    if (flagsByte & 2)
    {
      DescrambleDataV32(obj, rxOutBuf, (unsigned short)*rxCount);
    }
    if (flagsByte & 4)
    {
      TxClockSyncV32();
    }
    if (flagsByte & 0x80)
    {
      if (RetrainDetectV32(obj))
      {
        /* the retrain request: V32_CTL with the word at shorts 6 / 7 = (low byte & 0xf8 | the option flag bits 0 .. 2) | 0x480, applied by the next call */
        unsigned int w12;
        V32W_BYTE(obj, w030, 1) = (unsigned char)(V32W_BYTE(obj, w030, 1) & 0xfe);
        memcpy(v32_ctl_stash, V32_CTL, sizeof(v32_ctl_stash));
        memcpy(&w12, &v32_ctl_stash[6], 4);
        w12 = (w12 & ~0xffu) | (((w12 & 0xf8u) | (flagsByte & 1) | (flagsByte & 2) | (flagsByte & 4)) & 0xffu);
        w12 |= 0x480;
        memcpy(&v32_ctl_stash[6], &w12, 4);
        newStatus = 9;
        Control_Flag = 1;
      }
    }
    if (RenegotiateDetectV32(obj))
    {
      /* the renegotiation request: V32_CTL with the last 8 bytes cleared and the word at shorts 6 / 7 as above | 0x800; the role switches to responder first */
      short blk[16];
      unsigned int w12;
      memcpy(blk, V32_CTL, sizeof(blk));
      memcpy(&w12, &blk[6], 4);
      w12 = (w12 & ~0xffu) | (((w12 & 0xf8u) | (flagsByte & 1) | (flagsByte & 2) | (flagsByte & 4)) & 0xffu);
      w12 |= 0x800;
      memcpy(&blk[6], &w12, 4);
      memset(&blk[12], 0, 8);
      hdx->role = V32ROLE_RNG_RESP;
      V32FP_control(obj, blk);
      newStatus = 0x1c;
    }
    if ((V32W_BYTE(obj, w030, 1) & 0x20) == 0)
    {
      short elapsed = (short)(hdx->w0ae + 1);
      hdx->w0ae = elapsed;
      if (obj->margin < (short)(elapsed * 5))
      {
        newStatus = 0xc;
      }
    }
    else
    {
      hdx->w0ae = 0;
    }
    V32W_BYTE(obj, w030, 1) = (unsigned char)(V32W_BYTE(obj, w030, 1) | 0xc);
    V32W_BYTE(obj, w030, 0) = newStatus;
  }
  else
  {
    V32FP_control(obj, v32_ctl_stash); /* the block built by the retrain branch of the previous call */
    Control_Flag = 0;
  }
}

/* protocol slots indexed by the role (V32Hdx::role, enum V32Role) */
typedef void (*v32_protocol_fn)(V32Wrap*, short*, void*, void*, void*, void*, short*);

static const v32_protocol_fn V32_PROTOCOL_PORT[9] = {
    (v32_protocol_fn)v32_handshake,     /* 0: originate */
    (v32_protocol_fn)v32_handshake,     /* 1: Answer */
    (v32_protocol_fn)v32_handshake,     /* 2: local loop */
    (v32_protocol_fn)v32_null_protocol, /* 3: unused role */
    (v32_protocol_fn)v32_handshake,     /* 4: renegotiation initiator */
    (v32_protocol_fn)v32_handshake,     /* 5: renegotiation responder */
    (v32_protocol_fn)v32_data,          /* 6: connected data */
    (v32_protocol_fn)v32_null_protocol, (v32_protocol_fn)v32_null_protocol,
};

/* 16-bit work buffers of V32FP_modem, shared by all engines (the engine is not reentrant) */
static short tx_in_internal[100];
static short rx_out_internal[100];

/**
 * @brief Process one block: run the protocol slot of the current role and apply the transmit gain.
 * @param obj        the wrapper
 * @param rxBits     transmit data bits, one int per bit (the low 16 bits are used)
 * @param txOut      receives the transmit samples (16-bit)
 * @param txBitsIn   received line samples (16-bit); the argument name follows an earlier role of the buffer
 * @param rxOut      receives the demodulated data, one int per entry
 * @param txCount    in: number of transmit bits; out: number of transmit samples
 * @param rxCount    in: number of received samples; out: number of demodulated entries
 * @return the engine status code (low byte of V32Wrap::w030): the host glue treats 3, 4, 13 and 24 .. 27 as connected; 0xc, 0xe, 0x17 and 0x1c are timeout,
 *         failure and request codes
 *
 * Copies the received samples into the engine's copy buffer (the source for V32FP_GetCleanedSamples() when no later stage overwrites it) and the transmit
 * bits into a work buffer, clears flag bit 1, calls the protocol slot of V32Hdx::role, widens the demodulated entries into `rxOut`, switches to the connected
 * role (V32ROLE_DATA, state V32ST_DONT_CARE) when the handshake raised flag bit 0, and scales the transmit samples by V32Wrap::d00c (Q15).
 * @warning The static work buffers hold 100 entries: `txCount` and the demodulated count must stay at or below 100, and `rxCount` above 170 overflows
 *          the engine's 340-byte copy buffer.  The host glue clamps the demodulated count only after this function has returned.
 */
unsigned char V32FP_modem(V32Wrap* obj, void* rxBits, void* txOut, void* txBitsIn, void* rxOut, unsigned int* txCount, unsigned int* rxCount)
{
  V32Hdx* hdx;
  unsigned int rxLen = *rxCount;
  unsigned int txLen = *txCount;
  V32Eng* eng = V32W_PG(V32Eng*, obj, (int)offsetof(V32Wrap, engPtr));
  unsigned int i;

  if ((int)rxLen >= 1)
  {
    /* keep a copy of the received samples (the 4th argument; the 2nd argument carries the transmit bits) */
    short* dst = V32E_PG(short*, eng, (int)offsetof(V32Eng, copyBufB));
    for (i = 0; i < rxLen; i++)
      dst[i] = ((short*)txBitsIn)[i];
  }
  eng->w50d4 = (unsigned short)rxLen;

  for (i = 0; i < txLen; i++)
    tx_in_internal[i] = (short)((int*)rxBits)[i];

  V32W_BYTE(obj, w030, 1) = (unsigned char)(V32W_BYTE(obj, w030, 1) & 0xfd);

  {
    short tc = (short)txLen;
    short rc = (short)rxLen;
    hdx = V32W_PG(V32Hdx*, obj, (int)offsetof(V32Wrap, hdxPtr));
    V32_PROTOCOL_PORT[hdx->role](obj, tx_in_internal, txOut, txBitsIn, rx_out_internal, &tc, &rc);
    *txCount = tc;
    *rxCount = rc;
  }

  for (i = 0; i < *rxCount; i++)
    ((unsigned int*)rxOut)[i] = (unsigned int)(unsigned short)rx_out_internal[i];

  if (V32W_BYTE(obj, w030, 1) & 1)
  {
    hdx = V32W_PG(V32Hdx*, obj, (int)offsetof(V32Wrap, hdxPtr));
    hdx->role = V32ROLE_DATA;
    hdx->subState = V32ST_DONT_CARE;
  }

  {
    int gain = obj->d00c;
    for (i = 0; i < *txCount; i++)
    {
      short* s = (short*)txOut + i;
      *s = (short)(((int)*s * gain) >> 0xf);
    }
  }

  return V32W_BYTE(obj, w030, 0);
}
