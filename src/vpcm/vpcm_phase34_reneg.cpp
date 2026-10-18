/**
 * @file vpcm_phase34_reneg.cpp
 * @brief Per-symbol transmit state machines of the V.90 / K56Flex phase 3 / 4 handshake and of the V.90 rate renegotiation.
 *
 * Four plain C entry points, written as extern "C" C++ because they call methods of VPcmFloModem (getV90CpBits / getV90JaBits) and K56FlexFloModem
 * (getK56FlexJaBits / getK56FlexMpBits):
 *  - v90Phase34(): the V.90 phase 3 / 4 transmitter, states 3 .. 10 of obj->pcmMode_ (2 = done);
 *  - k56FlexPhase34(): the K56Flex transmitter, states 3 .. 5 of obj->k56Mode_ (2 = done);
 *  - v90RateReneg(): the rate renegotiation, states 0xb .. 0xe of pcmMode_ (the same machines as states 3 / 4 / 9 / 10 of v90Phase34());
 *  - v90RateRenegSilence(): the rate renegotiation on silence, states 0xf .. 0x14.
 *
 * Each call sends one symbol (or a pair) of constellation points through txmit() / txmitdibit() / txmitquadbit() and counts symbols in txCnt_.  The
 * constellation size follows rxSig_ (0x89b0 = 16-point, 0x8990 = 4-point); the points come from the vect4 / vect16 tables, the scrambled ones through
 * V34scrambler().  The Cp / Ja / Mp bits are fetched from the modem object into txBits_.  Every function returns 0.
 *
 * Details worth knowing:
 *  - the counter compares are 16-bit: `(short)(count + 1) <= 0x7f` is signed on the low 16 bits, and the counter is stored back as 16 bits;
 *  - for the 16-point constellation V34scrambler() is called twice per point (the first result is stored to txBits_ and re-read after the second
 *    call, index = second + 4 * first), once for the 4-point one; vect4 / vect16 are packed I/Q words and the index is not masked;
 *  - getMPrecvdBits() is declared here with the same regparm(1) attribute it has at 32 bits (defined in vpcm_getmp_alias.cpp).
 */

#include "v34/v34_object.h"
#include "v34/v34_hs_states.h" /* V34HS_* */

#include "core/dsp_debug_level.h" /* dsplibs_debug_level, LEVEL_ON() */
#include "core/dsp_host.h"
#include "v34/v34_api.h"
#include "v34/v34_symbol_tables.h" /* vect4, vect16 */

/* getMPrecvdBits(): regparm(1), C++ linkage, defined in vpcm_getmp_alias.cpp (it forwards to the C function of v34_modem_init.c) */
#ifdef __i386__
void getMPrecvdBits(tagV34Object* obj) __attribute__((regparm(1)));
#else
void getMPrecvdBits(tagV34Object* obj);
#endif

#define SIG_16 0x89b0u /* 16-point constellation */
#define SIG_4 0x8990u  /* 4-point constellation */

/** @brief Send txBits_ as a quad-bit (16-point constellation) or a di-bit (4-point) symbol, chosen by rxSig_. */
static inline void sendBits(tagV34Object* obj)
{
  if (obj->rx.rxSig_ == SIG_16)
    txmitquadbit(obj, (short)obj->tx.txBits_);
  else
    txmitdibit(obj, (short)obj->tx.txBits_);
}

/** @brief Send the all-ones symbol: quad-bit 0xf (16-point) or di-bit 3 (4-point). */
static inline void sendFixed(tagV34Object* obj)
{
  if (obj->rx.rxSig_ == SIG_16)
    txmitquadbit(obj, 0xf);
  else
    txmitdibit(obj, 3);
}

/** @brief Common tail of every Cp / Mp bits state: record the received MP bits, start the digital stage and enter the transmit state V34HS_EXMIT. */
static inline void bitsDone(tagV34Object* obj)
{
  getMPrecvdBits(obj);
  initdigital(obj);
  if (obj->txState_ != V34HS_EXMIT)
    obj->txState_ = V34HS_EXMIT;
}

/** @brief Send the points vect4[0], vect4[3]; @return the counter txCnt_ + 2 as 16 bits (txCnt_ itself is advanced once). */
static inline uint16_t pairA(tagV34Object* obj)
{
  obj->tx.txPoint_ = (uint32_t)vect4[0];
  txmit(obj);
  obj->tx.txCnt_ = (uint16_t)(obj->tx.txCnt_ + 1);
  obj->tx.txPoint_ = (uint32_t)vect4[3];
  txmit(obj);
  return (uint16_t)(obj->tx.txCnt_ + 1);
}

/** @brief Send the points vect4[2], vect4[1]; @return the counter txCnt_ + 2 as 16 bits (txCnt_ itself is advanced once). */
static inline uint16_t pairB(tagV34Object* obj)
{
  obj->tx.txPoint_ = (uint32_t)vect4[2];
  txmit(obj);
  obj->tx.txCnt_ = (uint16_t)(obj->tx.txCnt_ + 1);
  obj->tx.txPoint_ = (uint32_t)vect4[1];
  txmit(obj);
  return (uint16_t)(obj->tx.txCnt_ + 1);
}

/** @brief End of a pairB run: clear the previous-bits word, the symbol counter and the scrambler state. */
static inline void pairBdone(tagV34Object* obj)
{
  obj->tx.txPrev_ = 0;
  obj->tx.txCnt_ = 0;
  obj->tx.scrState_ = 0;
}

/**
 * @brief Draw one scrambled constellation point.
 * @param obj   session object
 * @param mode  mode argument of V34scrambler()
 * @param wide  true for the 16-point constellation (two scrambler results), false for the 4-point one
 * @return the packed I/Q word of the point
 */
static inline uint32_t scrambledPoint(tagV34Object* obj, short mode, bool wide)
{
  if (wide)
  {
    short r2;
    obj->tx.txBits_ = (uint16_t)V34scrambler(&obj->tx.scrState_, mode, 3, 2);
    r2 = V34scrambler(&obj->tx.scrState_, mode, 3, 2);
    return (uint32_t)vect16[(int)r2 + 4 * (int)(short)obj->tx.txBits_];
  }
  else
  {
    short r = V34scrambler(&obj->tx.scrState_, mode, 3, 2);
    obj->tx.txBits_ = (uint16_t)r;
    return (uint32_t)vect4[(int)r];
  }
}

/**
 * @brief One step of the V.90 phase 3 / 4 transmitter.
 *
 * First the Ja bits go out as di-bits until the modem reports them complete (this also arms the transmit-buffer backward clear when flags50 bit 2 is
 * set), then silence until pcmMode_ passes 2, then the state machine: 3 and 6 send the point pair A for 128 symbols, 4 and 7 the pair B for 16, 5 the
 * scrambled points, 8 and 10 the Cp bits (10 finishes the handshake: pcmMode_ = 2), 9 the all-ones symbol.
 *
 * @param obj  session object
 * @return 0 always
 */
int v90Phase34(tagV34Object* obj)
{
  VPcmFloModem* ctx = obj->ctx_;
  uint16_t f = obj->rx.rxFlags_;

  if (!(f & 0x400u))
  {
    /* Ja bits not yet sent: the Ja bits of the V.90 modem go out as di-bits until the modem reports them complete */
    int r = (short)ctx->getV90JaBits((short*)&obj->tx.txBits_);
    txmitdibit(obj, (short)obj->tx.txBits_);
    if (r != 0)
    {
      obj->rx.rxFlags_ = (uint16_t)(obj->rx.rxFlags_ | 0x400u);
      if (obj->params_->flags50 & 4u)
      {
        if (LEVEL_ON())
          dsplibs_debug_printf("VPcmV34Main: tx buffer backward clear is enabled...\r\n");
        obj->b_abfe_ = 1;
      }
    }
    return 0;
  }

  if (!(f & 0x10u))
  {
    /* gate not yet open: silence until the state word passes 2 */
    obj->tx.txPoint_ = 0;
    txmit(obj);
    if (obj->pcmMode_ > 2)
    {
      obj->rx.rxFlags_ = (uint16_t)(obj->rx.rxFlags_ | 0x10u);
      obj->tx.txCnt_ = 0;
    }
    return 0;
  }

  switch (obj->pcmMode_)
  {
  case 3:
  {
    uint16_t c = pairA(obj);
    if ((short)c <= 0x7f)
    {
      obj->tx.txCnt_ = c;
      return 0;
    }
    if (LEVEL_ON())
    {
      obj->tx.txCnt_ = c;
      dsplibs_debug_printf("Entered v34m->v90Receiver == V90RCV_P3_THIRD_S with tx->symcnt = %d period = %d\n", (int)(short)c, (int)(short)obj->rate.period_);
    }
    obj->pcmMode_ = 4;
    obj->tx.txCnt_ = 0;
    return 0;
  }
  case 4:
  {
    uint16_t c = pairB(obj);
    if (c != 0x10)
    {
      obj->tx.txCnt_ = c;
      return 0;
    }
    obj->pcmMode_ = 5;
    pairBdone(obj);
    return 0;
  }
  case 5:
  {
    uint16_t sig = obj->rx.rxSig_;
    if (sig == SIG_16)
    {
      obj->tx.txPoint_ = scrambledPoint(obj, 0, true);
    }
    else if (sig == SIG_4)
    {
      obj->tx.txPoint_ = scrambledPoint(obj, 0, false);
    }
    else
    {
      obj->tx.txPoint_ = 0;
    }
    txmit(obj);
    obj->tx.txCnt_ = (uint16_t)(obj->tx.txCnt_ + 1);
    return 0;
  }
  case 6:
  {
    uint16_t c = pairA(obj);
    if ((short)c <= 0x7f)
    {
      obj->tx.txCnt_ = c;
      return 0;
    }
    obj->pcmMode_ = 7;
    obj->tx.txCnt_ = 0;
    return 0;
  }
  case 7:
  {
    uint16_t c = pairB(obj);
    if (c != 0x10)
    {
      obj->tx.txCnt_ = c;
      return 0;
    }
    obj->pcmMode_ = 8;
    pairBdone(obj);
    return 0;
  }
  case 8:
  {
    int r = (short)ctx->getV90CpBits((short*)&obj->tx.txBits_);
    sendBits(obj);
    if (r != 0)
      obj->pcmMode_ = 9;
    return 0;
  }
  case 9:
    sendFixed(obj);
    obj->tx.txCnt_ = (uint16_t)(obj->tx.txCnt_ + 1);
    return 0;
  case 10:
  {
    int r = (short)ctx->getV90CpBits((short*)&obj->tx.txBits_);
    sendBits(obj);
    if (r != 0)
    {
      bitsDone(obj);
      obj->pcmMode_ = 2;
    }
    return 0;
  }
  default:
    return 0;
  }
}

/**
 * @brief One step of the K56Flex phase 3 / 4 transmitter.
 *
 * The Ja bits go out as di-bits until the modem reports them complete; then state 3 sends the Ja word 0x899f as eight di-bits (low pair first), state 4 the
 * scrambled points, state 5 the Mp bits (the end sets k56Mode_ = 2).
 *
 * @param obj  session object
 * @return 0 always
 */
int k56FlexPhase34(tagV34Object* obj)
{
  K56FlexFloModem* k56 = (K56FlexFloModem*)obj->k56_;

  if (!(obj->rx.rxFlags_ & 0x400u))
  {
    /* the K56Flex JA bits go out as di-bits until the modem reports them complete; then the JA word 0x899f is queued for the dibit state */
    int r = (short)k56->getK56FlexJaBits((short*)&obj->tx.txBits_);
    txmitdibit(obj, (short)obj->tx.txBits_);
    if (r != 0)
    {
      obj->rx.rxFlags_ = (uint16_t)(obj->rx.rxFlags_ | 0x400u);
      obj->tx.k56Word_ = 0x899f;
      obj->tx.k56Idx_ = 0;
      obj->k56Mode_ = 3;
    }
    return 0;
  }

  switch (obj->k56Mode_)
  {
  case 3:
  {
    /* the eight di-bits (low pair first) of the 16-bit JA word */
    int sh = 2 * (int)(short)obj->tx.k56Idx_;
    uint16_t n;
    txmitdibit(obj, (short)((unsigned)obj->tx.k56Word_ >> (sh & 31)));
    n = (uint16_t)((obj->tx.k56Idx_ + 1) & 7u);
    obj->tx.k56Idx_ = n;
    if (n != 0)
      return 0;
    obj->tx.txPrev_ = 0;
    obj->tx.txCnt_ = 0;
    obj->tx.scrState_ = 0;
    obj->k56Mode_ = 4;
    return 0;
  }
  case 4:
    if (obj->rx.rxSig_ == SIG_16)
      obj->tx.txPoint_ = scrambledPoint(obj, 1, true);
    else
      obj->tx.txPoint_ = scrambledPoint(obj, 1, false);
    txmit(obj);
    obj->tx.txCnt_ = (uint16_t)(obj->tx.txCnt_ + 1);
    return 0;
  case 5:
  {
    int r = (short)k56->getK56FlexMpBits((short*)&obj->tx.txBits_);
    sendBits(obj);
    if (r != 0)
    {
      bitsDone(obj);
      obj->k56Mode_ = 2;
    }
    return 0;
  }
  default:
    return 0;
  }
}

/**
 * @brief One step of the V.90 rate renegotiation transmitter, states 0xb .. 0xe (the machines of v90Phase34() states 3 / 4 / 9 / 10).
 * @param obj  session object
 * @return 0 always
 */
int v90RateReneg(tagV34Object* obj)
{
  VPcmFloModem* ctx = obj->ctx_;
  switch (obj->pcmMode_)
  {
  case 0xb:
  {
    uint16_t c = pairA(obj);
    if ((short)c <= 0x7f)
    {
      obj->tx.txCnt_ = c;
      return 0;
    }
    obj->pcmMode_ = 0xc;
    obj->tx.txCnt_ = 0;
    return 0;
  }
  case 0xc:
  {
    uint16_t c = pairB(obj);
    if (c != 0x10)
    {
      obj->tx.txCnt_ = c;
      return 0;
    }
    obj->pcmMode_ = 0xd;
    pairBdone(obj);
    return 0;
  }
  case 0xd:
    sendFixed(obj);
    obj->tx.txCnt_ = (uint16_t)(obj->tx.txCnt_ + 1);
    return 0;
  case 0xe:
  {
    int r = (short)ctx->getV90CpBits((short*)&obj->tx.txBits_);
    sendBits(obj);
    if (r != 0)
    {
      bitsDone(obj);
      obj->pcmMode_ = 2;
    }
    return 0;
  }
  default:
    return 0;
  }
}

/**
 * @brief One step of the rate renegotiation transmitter on silence, states 0xf .. 0x14.
 *
 * State 0x12 sends the Cp bits and on completion switches off the SAS detector; 0x13 sends scrambled points; 0x14 sends the Cp bits again with the
 * transmit flag restored and finishes with pcmMode_ = 2.
 *
 * @param obj  session object
 * @return 0 always
 */
int v90RateRenegSilence(tagV34Object* obj)
{
  VPcmFloModem* ctx = obj->ctx_;
  switch (obj->pcmMode_)
  {
  case 0xf:
  {
    uint16_t c = pairA(obj);
    if ((short)c <= 0x7f)
    {
      obj->tx.txCnt_ = c;
      return 0;
    }
    obj->pcmMode_ = 0x10;
    obj->tx.txCnt_ = 0;
    return 0;
  }
  case 0x10:
  {
    uint16_t c = pairB(obj);
    if (c != 0x10)
    {
      obj->tx.txCnt_ = c;
      return 0;
    }
    obj->pcmMode_ = 0x11;
    pairBdone(obj);
    return 0;
  }
  case 0x11:
    sendFixed(obj);
    obj->tx.txCnt_ = (uint16_t)(obj->tx.txCnt_ + 1);
    return 0;
  case 0x12:
  {
    int r = (short)ctx->getV90CpBits((short*)&obj->tx.txBits_);
    sendBits(obj);
    if (r != 0)
    {
      if (LEVEL_ON())
        dsplibs_debug_printf("move to SCR on silence rrn (disabling SAS detector on silence)\r\n");
      obj->pcmMode_ = 0x13;
      obj->tx.txFlags_ = (uint16_t)(obj->tx.txFlags_ & ~4u);
      obj->params_->flags3 = (uint8_t)(obj->params_->flags3 & ~4u);
    }
    return 0;
  }
  case 0x13:
  {
    obj->tx.txPoint_ = scrambledPoint(obj, 0, obj->rx.rxSig_ == SIG_16);
    txmit(obj);
    obj->tx.txCnt_ = (uint16_t)(obj->tx.txCnt_ + 1);
    obj->tx.txPrev_ = obj->tx.txBits_;
    return 0;
  }
  case 0x14:
  {
    int r;
    obj->tx.txFlags_ = (uint16_t)(obj->tx.txFlags_ | 4u);
    r = (short)ctx->getV90CpBits((short*)&obj->tx.txBits_);
    sendBits(obj);
    if (r != 0)
    {
      bitsDone(obj);
      obj->pcmMode_ = 2;
    }
    return 0;
  }
  default:
    return 0;
  }
}
