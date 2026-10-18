/**
 * @file v34_init.c
 * @brief V.34 object initialisation, scramblers / descramblers and the receive timing-recovery loop.
 *
 * Two groups of functions that work on the V.34 module object (tagV34Object, v34_object_view.h):
 * - Initialisation and scrambling: txrxdmainit() (DMA coefficient block), detectorinit() (tone-detector record), preinitV34() / preinitdigital() (the
 *   transmit and receive bit-allocation blocks and the scrambler / descrambler selection), txinit() / rxinit() (transmit and receive front ends) and the
 *   two scrambler / descrambler pairs scrambleGPC / scrambleGPA / descrambleGPC / descrambleGPA, whose addresses preinit*() stores in the object.
 * - Receive timing recovery: TimingV34() runs once per symbol on the receive front end and drives a first-order timing loop; setInitialPhase() picks the
 *   initial sampling phase from the first samples and setTimingStateParameters() loads the loop gains for the current timing mode;
 *   VPcmV34LogTimingOffset() publishes the measured timing offset.
 *
 * In the notes below "b+X" means obj + 0x264 + X, a member of the receive front end `obj->rx` (V34Rx).  Members are reached as `obj->member`
 * (v34_object_view.h, v34_subobjects.h); the undeclared sample window and the six words of the bit-allocation block are walked with the byte-offset
 * accessors of v34_access.h.
 *
 * Hazards:
 * - TimingV34() divides by two run-time values without a zero check (see its @warning); the division is a plain C `/`, so a zero divisor traps.
 * - The descramblers shift by a 16-bit count taken modulo 32 (`<< (count & 31)`).
 * - The scrambler / descrambler function addresses are stored as pointers in the object (`p2a28_`, `p0e48_`, `p0a28_`).
 */

#include <limits.h>
#include <stddef.h>
#include <stdint.h>

#include "v34_init_tables.h" /* Convolve16 */
#include "core/dsp_host.h"
#include "v34_internal.h"

#include "v34_access.h" /* S16 (byte-offset walks), may_alias u16_t */
#include "v34_role.h"   /* V34ROLE_* (the values of modeChar_) */
#include "v34_object_view.h"

/* forward declarations: preinit*() store the addresses of the scramblers */
int scrambleGPC(struct tagV34Object* obj, short count);
int scrambleGPA(struct tagV34Object* obj, short count);
int descrambleGPC(void* obj, unsigned short v, short n);
int descrambleGPA(void* obj, unsigned short v, short n);
void VPcmV34LogTimingOffset(void* obj, short v);
void setInitialPhase(V34Obj* obj);
void setTimingStateParameters(V34Obj* obj);

/**
 * @brief Build the 12-short DMA coefficient block from six words of a configuration record.
 * @param out  12 shorts: out[0..5] = w4, -w6, w8, -wa, wc, -we and out[6..11] = w6, w4, wa, w8, we, wc, where wX is the 16-bit word of `in` at byte
 *             offset X
 * @param in   the record; its 16-bit words at byte offsets 4, 6, 8, 0xa, 0xc and 0xe are used
 * @note  Only the low 16 bits are stored, so a negated -32768 stays -32768.  The stores are in a fixed order, which matters only if `in` and `out` overlap.
 */
void txrxdmainit(short* out, const void* in)
{
  const unsigned char* p = (const unsigned char*)in;
#define IN16(off) ((uint16_t)*(const u16_t*)(p + (off)))
  out[7] = (short)IN16(4);
  out[0] = (short)IN16(4);
  out[1] = (short)(0u - IN16(6));
  out[6] = (short)IN16(6);
  out[9] = (short)IN16(8);
  out[2] = (short)IN16(8);
  out[3] = (short)(0u - IN16(0xa));
  out[8] = (short)IN16(0xa);
  out[11] = (short)IN16(0xc);
  out[4] = (short)IN16(0xc);
  out[5] = (short)(0u - IN16(0xe));
  out[10] = (short)IN16(0xe);
#undef IN16
}

/**
 * @brief Initialise a tone-detector record (V34ToneDet).
 * @param detobj  the record: +0 coefficient-table address, +4 p3, +6 0, +8 -p5, +0xa p4, +0xc 1, +0xe p7, +0x10 p6, +0x12 0;
 *                the shorts at +0x14..+0x23 are cleared
 * @param p2      address of the coefficient table (stored as a pointer)
 * @param p3      16-bit parameter stored at +4
 * @param p4      16-bit parameter stored at +0xa
 * @param p5      16-bit parameter; its negation (16-bit, so -32768 stays -32768) is stored at +8
 * @param p6      16-bit parameter stored at +0x10
 * @param p7      16-bit parameter stored at +0xe
 */
void detectorinit(V34ToneDet* detobj, const void* p2, short p3, short p4, short p5, short p6, short p7)
{
  int i, j;

  /* clear the two accumulator groups */
  for (i = 0; i < 2; i++)
    for (j = 0; j < 2; j++)
    {
      int k = i * 2 + j;
      detobj->toneInHist_[k] = 0;
      detobj->toneOutHist_[k] = 0;
    }

  detobj->toneMode_ = p3;
  detobj->toneCnt_ = (short)(0 - (int)p5);
  detobj->toneState_ = 1;
  detobj->toneDetF6_ = 0;
  detobj->toneReq_ = p4;
  detobj->p3564_ = (void*)p2; /* the coefficient table address, stored as a pointer */
  detobj->toneHigh_ = p6;
  detobj->toneLow_ = p7;
  detobj->toneEnergy_ = 0;
}

/* ===================== scramblers (scrambleGPC / scrambleGPA) ===================== */

/*
 * Shared prologue of the scramblers: the next 16-bit override value from the capture-in list, or 0xffff.  The list (`txWords_[idx]`, one value per dword
 * slot) is enabled by the word `w2214_`; `txHead_` is the read index and `txTail_` the limit (unsigned compare).  The index only advances when a value
 * was consumed.
 * @warning `txHead_` and `txTail_` are not checked against the 64 entries of `txWords_` here; a tail above 64 reads past the array.  The queue code
 *          (vpcm_progress.cpp) is expected to keep the tail within the 64 words.
 */
static uint32_t scr_next_input(V34Obj* obj)
{
  uint32_t v = 0xffffu;

  if (obj->mod.w2214_ != 0)
  {
    uint32_t idx = obj->txHead_;
    if (idx < obj->txTail_)
    {
      v = (uint16_t)obj->txWords_[idx]; /* the low half of the word */
      obj->txHead_ = idx + 1u;
    }
  }
  return v;
}

/**
 * @brief Advance the GPC scrambler state by one 16-bit step.
 * @param obj    the V.34 module object; the scrambler state is the four dwords at +0x2a54..+0x2a60
 * @param count  bit count of the caller; the return value is count - 16
 * @return (short)(count - 0x10), sign-extended
 * @note  The new state dwords are w0 = (input << 16) | old half, w1 = (old top << 2) | old half and w2 = (old top << 7) | old half; the new top dword
 *        is `w2a60_ = w0 ^ w1 ^ w2`.  All state reads (the 16-bit halves at
 *        +0x2a56, +0x2a5a, +0x2a5e and the dword at +0x2a60) happen before the overlapping dword stores, so they are all taken first.
 */
int scrambleGPC(V34Obj* obj, short count)
{
  uint32_t v = scr_next_input(obj);
  uint32_t r56 = (uint16_t)(obj->tx.w2a54_ >> 16);
  uint32_t r5a = (uint16_t)(obj->tx.w2a58_ >> 16);
  uint32_t r5e = (uint16_t)(obj->tx.w2a5c_ >> 16);
  uint32_t s60 = obj->tx.w2a60_;
  uint32_t w0 = (v << 16) | r56;
  uint32_t w1 = (s60 << 2) | r5a;
  uint32_t w2 = (s60 << 7) | r5e;

  obj->tx.w2a54_ = w0;
  obj->tx.w2a58_ = w1;
  obj->tx.w2a5c_ = w2;
  obj->tx.w2a60_ = w0 ^ w1 ^ w2;
  return (int)(short)(count - 0x10);
}

/**
 * @brief Advance the GPA scrambler state by one 16-bit step.
 * @param obj    the V.34 module object (same state as scrambleGPC())
 * @param count  bit count of the caller
 * @return (short)(count - 0x10), sign-extended
 * @note  Same input prologue as scrambleGPC(); the feedback is four XOR rounds of `((r << 5) | s) ^ x`.
 */
int scrambleGPA(V34Obj* obj, short count)
{
  uint32_t v = scr_next_input(obj);
  uint32_t r56 = (uint16_t)(obj->tx.w2a54_ >> 16);
  uint32_t s = (uint16_t)(obj->tx.w2a58_ >> 16);
  uint32_t r5e = (uint16_t)(obj->tx.w2a5c_ >> 16);
  uint32_t s60 = obj->tx.w2a60_;
  uint32_t w0 = (v << 16) | r56;
  uint32_t w2 = (s60 << 7) | r5e;
  uint32_t x = w0 ^ w2;
  uint32_t r = ((s60 >> 11) | s) ^ x;

  r = ((r << 5) | s) ^ x;
  r = ((r << 5) | s) ^ x;
  r = ((r << 5) | s) ^ x;

  obj->tx.w2a54_ = w0;
  obj->tx.w2a5c_ = w2;
  obj->tx.w2a60_ = r;
  obj->tx.w2a58_ = s | (r << 5);
  return (int)(short)(count - 0x10);
}

/* ===================== descramblers (descrambleGPC / descrambleGPA) ===================== */

/*
 * Common body of the two descramblers.  State: acc = dword `w0e74_`, w1 = dword `w0e78_`, w2 = dword `w0e7c_`, bit count = word `w0e80_`.  `v` (the new
 * input bits) and `n` (their count) are used in 16-bit adds and 5-bit shift counts.  The descrambler produces nothing until 32 bits have accumulated;
 * then one 16-bit result `(acc ^ w1' ^ w2) & 0xffff` is formed, where w1' is `w1` (GPC) or `(acc << 5) | w1` (GPA, `gpa` != 0).  When the capture
 * list is enabled (`w2214_` != 0) and its index `rxWordCnt_` is at most 0x3f the result is appended to `rxWords_[]`.  Always returns 0.
 */
static int descramble_common(V34Obj* obj, uint16_t v, uint16_t n, int gpa)
{
  uint32_t t = ((uint32_t)v << ((uint32_t)(int32_t)(int16_t)obj->rx.w0e80_ & 31u)) | obj->rx.w0e74_;
  int16_t cnt;
  uint32_t w1, w2, w1e, out;

  /* merge the new bits into the accumulator; wait until 32 are present */
  obj->rx.w0e74_ = t;
  cnt = (int16_t)(uint16_t)(obj->rx.w0e80_ + n);
  obj->rx.w0e80_ = cnt;
  if (cnt <= 0x1f)
    return 0;

  /* form the output word */
  w1 = obj->rx.w0e78_;
  w2 = obj->rx.w0e7c_;
  w1e = gpa ? ((t << 5) | w1) : w1;
  out = (t ^ w1e ^ w2) & 0xffffu;

  if (obj->mod.w2214_ != 0)
  {
    uint32_t idx = obj->rxWordCnt_;
    if (idx <= 0x3fu)
    {
      obj->rxWords_[idx] = out; /* idx <= 0x3f: inside the 64 words */
      obj->rxWordCnt_ = idx + 1u;
    }
  }

  /* shift the registers and keep the unconsumed bits */
  obj->rx.w0e78_ = gpa ? (w1e >> 16) : ((t << 2) | (w1 >> 16));
  obj->rx.w0e7c_ = (t << 7) | (w2 >> 16);
  {
    uint16_t nc = (uint16_t)((uint16_t)cnt - 0x10);
    obj->rx.w0e80_ = (int16_t)nc;
    obj->rx.w0e74_ = ((uint32_t)v << (((uint32_t)(int32_t)(int16_t)nc - (uint32_t)n) & 31u)) | (t >> 16);
  }
  return 0;
}

/**
 * @brief Descramble `n` new bits with the GPC polynomial.
 * @param obj  the V.34 module object
 * @param v    the new bits
 * @param n    number of new bits
 * @return 0
 * @see descramble_common() for the state and the output capture
 */
int descrambleGPC(void* obj, unsigned short v, short n) { return descramble_common(obj, v, (uint16_t)n, 0); }
/** @brief Descramble `n` new bits with the GPA polynomial (as descrambleGPC(), with the shifted accumulator fed into the XOR). */
int descrambleGPA(void* obj, unsigned short v, short n) { return descramble_common(obj, v, (uint16_t)n, 1); }

/* ===================== preinit (PreInitBlock) ===================== */

/*
 * Initialise the 0x450-byte bit-allocation block shared by preinitV34() and both halves of preinitdigital(): +0x48 s16[128] = 0; +0x148 s16[128] = 0;
 * +0x248 u32[128] = UINT_MAX; +0x2c s16[6] = 0; +0x18 s16[6] = 0; +0x16 = 0x18; +0x28 = Convolve16; +0x3c = 0; +0x38 = 0; +0x44c = 0; +0x8 = 0;
 * +0x448 = scrambleGPC.  All other bytes of the block are left untouched.
 */
static void preinit_block(V34BlkB* blkbobj)
{
  int i;

  for (i = 0; i < 0x80; i++)
  {
    blkbobj->shellX_[i] = 0;
    blkbobj->shellY_[i] = 0;
    blkbobj->shellZ_[i] = UINT_MAX;
  }
  for (i = 0; i < 6; i++)
  {
    blkbobj->w0a2c_[i] = 0;
    blkbobj->fbHist_[i] = 0;
  }

  blkbobj->w0a16_ = 0x18;
  blkbobj->p0a28_ = (void*)Convolve16;
  blkbobj->w0a3c_ = 0;
  blkbobj->w0a38_ = 0;
  blkbobj->wE4c_ = 0;
  blkbobj->w0a08_ = 0;
  blkbobj->p0e48_ = (void*)scrambleGPC;
}

/**
 * @brief Initialise one bit-allocation block (see preinit_block()).
 * @param blk  the block
 */
void preinitV34(void* blk) { preinit_block((V34BlkB*)blk); }

/**
 * @brief Initialise the transmit and receive bit-allocation blocks, the scrambler / descrambler state and select the scrambler pair for the role.
 * @param obj  the V.34 module object
 * @note  Block A (scrambler side) is at obj + 0x25e0 (its +0x448 slot is `p2a28_`), block B (descrambler side, `blkb`) at obj + 0xa00 (+0x448 is
 *        `p0e48_`); then three buffers after them are cleared, the scrambler / descrambler registers reset and the function slots set from
 *        `modeChar_` (16-bit compare): not V34ROLE_CALL: scrambler = scrambleGPA, descrambler = descrambleGPC; V34ROLE_CALL: scrambler =
 *        scrambleGPC, descrambler = descrambleGPA.  The two slots were set to scrambleGPC by the block initialisation and are overwritten here.
 */
void preinitdigital(V34Obj* obj)
{

  preinit_block((V34BlkB*)&obj->tx.w25e0_); /* the transmit block has the layout of V34BlkB */
  preinit_block(&obj->rx.blkb);
  sysdep_memset(&obj->rx.trellis_, 0, sizeof(obj->rx.trellis_)); /* path metrics, trellis nodes, stage records */

  /* reset the scrambler and descrambler registers */
  obj->tx.w2a54_ = 0;
  obj->rx.fdStage_ = 0;
  obj->tx.w2a64_ = 0x20;
  obj->rx.w0e80_ = 0;
  obj->tx.w2a60_ = 0;
  obj->tx.w2a5c_ = 0;
  obj->tx.w2a58_ = 0;
  obj->rx.w0e74_ = 0;
  obj->rx.w0e7c_ = 0;
  obj->rx.w0e78_ = 0;
  obj->mod.w2214_ = 0;
  obj->waa74_ = 0;

  /* the calling modem scrambles with GPC and descrambles with GPA; the answering modem the other way round */
  if (obj->modeChar_ != V34ROLE_CALL)
  {
    obj->tx.p2a28_ = (void*)scrambleGPA;
    obj->rx.blkb.p0e48_ = (void*)descrambleGPC;
  }
  else
  {
    obj->tx.p2a28_ = (void*)scrambleGPC;
    obj->rx.blkb.p0e48_ = (void*)descrambleGPA;
  }
}

/**
 * @brief Reset the transmit front end.
 * @param obj  the V.34 module object
 * @note  Clears the echo-canceller step sizes (`echoAlphaNear_` / `echoAlphaFar_`), the scrambler state (`scrState_`), the previous symbol and the counter at
 *        +0x25c0 / +0x25c6 / +0x25cc (not the flag word +0x25c2); cleans up the near (`echo0`) then far (`echo1`) echo canceller;
 *        sets the transmit index to 0x20
 *        and the two ring pointers at +0x2220 / +0x2224 to obj + 0x2228 / obj + 0x22a8; clears the 0x398-byte ring at obj + 0x2228, the receive ring at
 *        obj + 0x270 (0x100 bytes; its read / write pointers are set to it) and the 0x54-byte pre-filter delay line.
 */
void txinit(V34Obj* obj)
{

  obj->echoAlphaNear_ = 0;
  obj->echoAlphaFar_ = 0;
  obj->tx.scrState_ = 0;
  obj->tx.txPrev_ = 0;
  obj->tx.txCnt_ = 0;

  V34EchoCleanUp(&obj->echo0);
  V34EchoCleanUp(&obj->echo1);

  obj->tx.txIdx_ = 0x20;
  obj->tx.p2220_ = obj->tx.unk2228_;
  obj->tx.p2224_ = obj->tx.unk2228_ + 128;
  sysdep_memset(obj->tx.unk2228_, 0, 0x398);

  obj->rx.rxW0_ = 0;
  obj->rx.p0268_ = obj->rx.unk0270_;
  obj->rx.p026c_ = obj->rx.unk0270_;
  sysdep_memset(obj->rx.unk0270_, 0, 0x100);
  sysdep_memset(&obj->mod.pf, 0, 0x54);
}

/**
 * @brief Reset the receive front end `b`.
 * @param obj  the V.34 module object
 * @return 0
 * @note  Sets the receive gain to 0x200, the constant 0x3333 at b+0x13a and the flag b+0x1b8 to 1; cleans up the equalizer (obj + 0x630), clears two small
 *        histories (obj + 0x4ec, 0xc bytes; obj + 0x4f8, 0x10 bytes) and initialises the Hilbert filter (obj + 0xa1b8); then clears the loop, level, error and
 *        counter members and points the sink pointer `rxEnd_` at the FSK buffer.  The test `b+0x122 & 0x08` selects the timing-loop start values: bit clear
 *        sets the dword b+0x1f8 to 0 and b+0x202 to 8, bit set leaves b+0x1f8 alone and sets b+0x202 to 10.  b+0x200 is 2 in both cases.
 */
int rxinit(V34Obj* obj)
{
  V34Rx* rxobj = &obj->rx;

  /* gains and equalizer / filter state */
  rxobj->rxGain_ = 0x200;
  rxobj->w39e_ = 0x3333;
  rxobj->w041c_ = 1;
  V34EqualizerCleanUp(rxobj->unk0628_ + 8);
  sysdep_memset(rxobj->tapX2_, 0, sizeof(rxobj->tapX2_) + sizeof(rxobj->tapX1_)); /* tapX1_ follows tapX2_ (V34VIEW_ASSERT) */
  sysdep_memset(rxobj->tapY2_, 0, sizeof(rxobj->tapY2_) + sizeof(rxobj->tapY1_)); /* tapY1_ follows tapY2_ */
  V34InitHilbertFilter(obj->unk9158_ + 4192);

  /* timing loop and level state */
  rxobj->w047c_ = 0x4000;
  rxobj->w0456_ = 0x4000;
  rxobj->w039c_ = 0;
  rxobj->rxLevel_ = 0;
  rxobj->w0458_ = 0;

  if ((rxobj->rxFlags_ & 0x08) == 0)
  {
    rxobj->w045c_ = 0;
    rxobj->w0464_ = 2;
    rxobj->w0466_ = 8;
  }
  else
  {
    rxobj->w0464_ = 2;
    rxobj->w0466_ = 10;
  }

  /* counters and error state */
  rxobj->w09fc_ = 0; /* the first word after the equalizer state */
  rxobj->w480_ = 0;
  rxobj->w046a_ = 0;
  rxobj->equErr_ = 0;
  rxobj->w0468_ = 0;
  rxobj->w0488_ = 0;
  rxobj->w484_ = 0;
  rxobj->w048c_ = 0;
  rxobj->w0404_ = 0;
  rxobj->w04aa_ = 0;
  rxobj->w388_ = 0;
  rxobj->w0408_ = 0;
  rxobj->w04a8_ = 0;
  rxobj->w0420_ = 0;
  rxobj->w0390_ = 0;
  rxobj->w0384_ = 0;
  rxobj->w038e_ = 0;
  (obj)->tx.constCnt_ = 0;

  rxobj->rxEnd_ = (const int16_t*)rxobj->rxFskBuf_; /* sample sink pointer: start of the FSK buffer */
  rxobj->w4ac_ = 0;
  rxobj->w04b0_ = 0;
  return 0;
}

/**
 * @brief Store the measured receive timing offset for the V.PCM reports.
 * @param obj  the V.34 module object
 * @param v    timing offset in ppm (16 bits; stored in `dgC0c_`)
 */
void VPcmV34LogTimingOffset(void* obj, short v) { ((V34Obj*)obj)->dgC0c_ = v; }

/* ===================== setInitialPhase ===================== */

/* sample S[k] of the receive front end (`smp_`, 8 entries).  The indexes used here stay inside it: setInitialPhase() reads S[1] .. S[5]; TimingV34() reads
 * S[ia], S[ia + 2], S[ib], S[ib + 2] with ia, ib = `w0450_`, `w0452_`, which are 0 (cleared) or the pair (2, 1) / (1, 2) that setInitialPhase() stores (C: the only
 * writers in the library). */
#define SMP(rx, k) ((rx)->smp_[k])

/* ===================== setInitialPhase ===================== */

/**
 * @brief Choose the initial timing phase (b+0x1ac) from the first few samples S[] at b+0x278.
 * @param obj  the V.34 module object
 * @note  Steps (16-bit samples, 32-bit arithmetic):
 *        1. i = 1; if S1 * S2 < 0, step i while i <= 4 and S[i] * S[i+1] < 0; reaching i == 5 prints "(i==5)" (debug level > 1) and resets i = 1.
 *        2. a = S[i], c = S[i+1]; if a < 0 both are negated (16-bit).
 *        3. b+0x1ec / b+0x1ee = (2, 1) when i == 2, else (1, 2): the pair of samples the timing loop later compares.
 *        4. q0 = ((sum >> 1) + ((c - a) << 13)) / sum with sum = a + c (0 and a message when sum == 0).
 *        5. For k = 0..19: P2 = (short)poly(k + 20), P1 = (short)poly(k) with poly(x) = -21 x^2 + 0x345 x - 0x162; s = P2 + P1; q = ((s >> 1) +
 *           ((P2 - P1) << 13)) / s (0 and a message when s == 0); score = (short)((d*d + 0x1000) >> 13) with d = q0 - q; the best (strictly smaller,
 *           signed 16-bit) k wins, starting from a score of 32000.
 *        6. v = (ushort)((short)((10 - bestk) * 0x230) + b+0x1ac); b+0x1ac = v if v < (short)b+0x1b0, else b+0x1b0 - 1.
 *        The divisions are guarded by the zero tests above.  An exhaustive search over the 32-bit q0 values found at least one k scoring
 *        below 32000 each time, so `bestk` is always set by the loop.
 */
void setInitialPhase(V34Obj* obj)
{
  V34Rx* rxobj = &obj->rx;
  int i = 1;
  int a, c, sum, q0, k, acc, bestk;
  short best;
  uint32_t v;

  /* find a sign-change-free pair of adjacent samples */
  if ((int)SMP(rxobj, 1) * (int)SMP(rxobj, 2) < 0)
  {
    do
    {
      i = (short)(i + 1);
    } while (i <= 4 && (int)SMP(rxobj, i) * (int)SMP(rxobj, i + 1) < 0);
    if (i == 5)
    {
      if (dsplibs_debug_level > 1)
        dsplibs_debug_printf("setInitialPhase: Error (i==5)  !!!!!!!\n");
      i = 1;
    }
  }

  /* normalise the pair to a positive first sample and record which samples the timing loop compares */
  a = SMP(rxobj, i);
  c = SMP(rxobj, i + 1);
  if (a < 0)
  {
    a = (short)(-a);
    c = (short)(-c);
  }
  if (i == 2)
  {
    rxobj->w0450_ = 2;
    rxobj->w0452_ = 1;
  }
  else
  {
    rxobj->w0450_ = 1;
    rxobj->w0452_ = 2;
  }

  /* measured ratio of the two samples */
  sum = a + c;
  if (sum == 0)
  {
    q0 = 0;
    if (dsplibs_debug_level > 1)
      dsplibs_debug_printf("setInitialPhase() : Error - deviding by 0 (samp2+samp1=0)\n");
  }
  else
  {
    q0 = ((sum >> 1) + (c - a) * 0x2000) / sum;
  }

  /* find the table position (k) whose predicted ratio is closest to the measured one */
  best = 32000;
  bestk = 0; /* always overwritten by the loop (see the function comment) */
  acc = 0;
  for (k = 0; k < 20; k++)
  {
    int t = k + 20;
    int p2 = (short)(t * t * -0x15 + -0x162 + t * 0x345);
    int p1 = (short)(k * k * -0x15 + -0x162 + acc);
    int s = p2 + p1;
    int q;
    uint32_t d;
    short score;

    if (s == 0)
    {
      q = 0;
      if (dsplibs_debug_level > 1)
        dsplibs_debug_printf("setInitialPhase() : Error - deviding by 0 (polyValue(k2)+polyValue(k)=0)\n");
    }
    else
    {
      q = ((s >> 1) + (p2 - p1) * 0x2000) / s;
    }
    d = (uint32_t)(q0 - q);
    score = (short)((d * d + 0x1000u) >> 13);
    if (score < best)
    {
      best = score;
      bestk = k;
    }
    acc += 0x345;
  }

  /* move the sampling phase by the offset of the best position, limited to the symbol period */
  v = (uint16_t)((short)((10 - bestk) * 0x230) + (int)rxobj->w410_);
  if ((int)v < (int)(int16_t)rxobj->w414_)
    rxobj->w410_ = (uint16_t)v;
  else
    rxobj->w410_ = (uint16_t)(rxobj->w414_ - 1);
}

/* ===================== setTimingStateParameters ===================== */

/**
 * @brief Load the timing-loop gains for the current timing mode b+0x1c0.
 * @param obj  the V.34 module object
 * @note  Mode m = (short)b+0x1c0.  The selector is an unsigned compare of the mode against 8, so negative modes and m > 8 do nothing; modes 0 and 1 do
 *        nothing either.  alt = (modeChar_ == V34ROLE_CALL) selects the second column where two values are listed.  Gains written (Kp at b+0x234, Ki at
 *        b+0x236, limit at b+0x232):
 *
 *            m   Kp               Ki            limit
 *            2   14000            0             400
 *            3   12000            0xd2          1000
 *            4   6000             0x5a          1000
 *            5   0xdac            0x1e          alt ? 0xffff : 1000      (+ LogTimingOffset)
 *            6   alt ? 0xdac : 2000   alt ? 3 : 10   2000
 *            7   alt ? 1000 : 0x5dc   2             alt ? 2000 : 4000
 *            8   500              1             0xffff                   (+ LogTimingOffset)
 *
 *        In modes 5 and 8 VPcmV34LogTimingOffset(obj, 10 * b+0x1d0) is called after the three stores and the mode is reloaded.  In every path (also the
 *        no-op ones) the function ends with: if ((ushort)b+0x1c0 == 2), b+0x1d2 = (short)rxBaud_ >> 3.
 */
void setTimingStateParameters(V34Obj* obj)
{
  V34Rx* rxobj = &obj->rx;
  int alt = ((int16_t)obj->modeChar_ == V34ROLE_CALL);
  int mode = rxobj->w0424_;
  int dolog = 0;

  /* select the gain set */
  if ((unsigned int)mode <= 8u)
  {
    switch (mode)
    {
    case 2:
      rxobj->w0498_ = 14000;
      rxobj->w049a_ = 0;
      rxobj->w0496_ = 400;
      break;
    case 3:
      rxobj->w0498_ = 12000;
      rxobj->w049a_ = 0xd2;
      rxobj->w0496_ = 1000;
      break;
    case 4:
      rxobj->w0498_ = 6000;
      rxobj->w049a_ = 0x5a;
      rxobj->w0496_ = 1000;
      break;
    case 5:
      rxobj->w0498_ = 0xdac;
      rxobj->w049a_ = 0x1e;
      rxobj->w0496_ = alt ? (short)0xffff : 1000;
      dolog = 1;
      break;
    case 6:
      rxobj->w0498_ = alt ? 0xdac : 2000;
      rxobj->w049a_ = alt ? 3 : 10;
      rxobj->w0496_ = 2000;
      break;
    case 7:
      rxobj->w0498_ = alt ? 1000 : 0x5dc;
      rxobj->w049a_ = 2;
      rxobj->w0496_ = alt ? 2000 : 4000;
      break;
    case 8:
      rxobj->w0498_ = 500;
      rxobj->w049a_ = 1;
      rxobj->w0496_ = (short)0xffff;
      dolog = 1;
      break;
    default: /* 0, 1: nothing */
      break;
    }
  }

  if (dolog)
  {
    VPcmV34LogTimingOffset(obj, (short)(10 * (int)rxobj->resOffs_));
    mode = rxobj->w0424_;
  }

  if ((short)mode == 2)
    rxobj->w436_ = (short)((obj)->rate.rxBaud_ >> 3);
}

/* ===================== TimingV34 ===================== */

/**
 * @brief Run one symbol interval of the receive timing-recovery loop.
 * @param obj  the V.34 module object; the state is the receive front end `b`
 * @warning Two divisions are not guarded: by (short)n, the number of ticks accumulated (step 6), and by T = (short)b+0x1be (step 6).  A zero divisor
 *          traps.  Callers keep T non-zero; n is non-zero whenever the limit b+0x1d2 is positive.  INT_MIN / -1 is not excluded either.
 * @note  m = (ushort)b+0x1c0 is the timing mode.  Steps:
 *        0. m == USHRT_MAX: return.
 *        1. m == 1: clear the tick counter b+0x230, run setInitialPhase(); the mode becomes 2 (and b+0x1c8 is cleared) if b+0x1c8 == 1, else 6;
 *           run setTimingStateParameters(); reload m.
 *        2. lim = (ushort)b+0x232; unless lim == USHRT_MAX or m == 0: cnt = b+0x230 + 1; when cnt == (short)lim the counter is cleared, the mode is
 *           incremented and the gains reloaded, else the counter is stored.
 *        3. Energies E_A = S[iA]^2 + S[iA+2]^2 and E_B = S[iB]^2 + S[iB+2]^2 (iA, iB = b+0x1ec, b+0x1ee; 32-bit sums that can wrap to negative).
 *        4. m == 0: v = the integrator b+0x1e0.  Otherwise the energies are normalised (sh = leading-bit position; skipped when either is negative, then
 *           sh = 0), A = (E_A >> (16 - sh)) & 0xffff, B likewise, q = (((B - A) << 15) + ((A + B) >> 1)) / (A + B) (0 when A | B == 0); the integrator becomes
 *           (Ki * q + 0x4000) >> 15 plus its old value and v = integrator + ((Kp * q + 0x200) >> 11).
 *        5. F2 = v + b+0x1d8 (the phase accumulator); r = (F2 + 0x4000) >> 15 is the whole-sample correction; the accumulator keeps F2 - (r << 15);
 *           b+0x1ae = r + T with T = b+0x1be; acc = r + b+0x1cc (sum of corrections); n = b+0x1ce + 1.
 *        6. While (short)n < b+0x1d2, store n and acc and return.  Otherwise q1 = (acc * 10000 + n/2) / n, the offset ppm = (100 * q1 + T/2) / T is stored
 *           in b+0x1d0 (printed at debug level > 1) and n and acc are cleared.
 */
void TimingV34(V34Obj* obj)
{
  V34Rx* rxobj = &obj->rx;
  uint16_t m = AS_U16(rxobj->w0424_);
  uint32_t ea, eb;
  int ia, ib;
  uint32_t v;
  uint32_t f2;
  short r, acc, ppm;
  uint16_t lim, t, n;
  int nn, q1;

  if (m == USHRT_MAX)
    return;

  /* initial acquisition: choose the starting phase and the gains */
  if (m == 1)
  {
    rxobj->w0494_ = 0;
    setInitialPhase(obj);
    if (rxobj->w042c_ == 1)
    {
      AS_U16(rxobj->w0424_) = 2;
      rxobj->w042c_ = 0;
    }
    else
    {
      AS_U16(rxobj->w0424_) = 6;
    }
    setTimingStateParameters(obj);
    m = AS_U16(rxobj->w0424_);
  }

  /* advance to the next timing mode when the current one has run for its limit */
  lim = AS_U16(rxobj->w0496_);
  if (lim != USHRT_MAX && m != 0)
  {
    uint16_t cnt = (uint16_t)(rxobj->w0494_ + 1);
    if ((int)cnt == (int)(short)lim)
    {
      rxobj->w0494_ = 0;
      AS_U16(rxobj->w0424_) = (uint16_t)(m + 1);
      setTimingStateParameters(obj);
      m = AS_U16(rxobj->w0424_);
    }
    else
    {
      rxobj->w0494_ = cnt;
    }
  }

  /* energies of the two sample pairs */
  ia = rxobj->w0450_;
  ib = rxobj->w0452_;
  {
    int a0 = SMP(rxobj, ia), a1 = SMP(rxobj, ia + 2);
    int b0 = SMP(rxobj, ib), b1 = SMP(rxobj, ib + 2);
    ea = (uint32_t)(a0 * a0) + (uint32_t)(a1 * a1);
    eb = (uint32_t)(b0 * b0) + (uint32_t)(b1 * b1);
  }

  /* loop filter: proportional plus integral term of the normalised energy difference */
  if (m == 0)
  {
    v = rxobj->w0444_;
  }
  else
  {
    short sh = 0;
    uint32_t mask = 0x80000000u;
    uint32_t A, B;
    int q = 0;
    uint32_t i2;

    if ((int32_t)ea >= 0 && (int32_t)eb >= 0)
    {
      do
      {
        mask >>= 1;
        sh = (short)(sh + 1);
        if (ea & mask)
          break;
      } while ((eb & mask) == 0 && sh <= 15);
    }
    A = (uint32_t)((int32_t)ea >> (16 - sh)) & 0xffffu;
    B = (uint32_t)((int32_t)eb >> (16 - sh)) & 0xffffu;
    if ((A | B) != 0)
      q = (int)(((B - A) << 15) + ((A + B) >> 1)) / (int)(A + B);
    i2 = (uint32_t)((rxobj->w049a_ * q + 0x4000) >> 15) + rxobj->w0444_;
    v = i2 + (uint32_t)((rxobj->w0498_ * q + 0x200) >> 11);
    rxobj->w0444_ = i2;
  }

  /* split the correction into whole samples and the remaining phase */
  f2 = v + (uint32_t)rxobj->resPhase_;
  r = (short)((int32_t)(f2 + 0x4000u) >> 15);
  t = rxobj->w422_;
  rxobj->resPhase_ = f2 - ((uint32_t)(int32_t)r << 15);
  rxobj->w412_ = (uint16_t)(r + t);
  acc = (short)(uint16_t)(r + rxobj->w0430_);
  n = (uint16_t)(rxobj->w0432_ + 1);

  /* accumulate until enough ticks are available, then convert the mean correction to ppm */
  if ((short)n < (int16_t)rxobj->w436_)
  {
    rxobj->w0432_ = n;
    rxobj->w0430_ = (uint16_t)acc;
    return;
  }

  nn = (short)n;
  q1 = ((int)acc * 10000 + (nn >> 1)) / nn;                                                   /* unguarded idiv by n */
  ppm = (short)((int)((uint32_t)q1 * 100u + (uint32_t)((int)(short)t >> 1)) / (int)(short)t); /* unguarded idiv by T */
  rxobj->resOffs_ = ppm;

  if (dsplibs_debug_level > 1)
  {
    rxobj->w0430_ = (uint16_t)acc;
    rxobj->w0432_ = n;
    dsplibs_debug_printf("TimingV34: Timing Offset [ppm] = %d\n", (int)ppm);
  }
  rxobj->w0432_ = 0;
  rxobj->w0430_ = 0;
}
