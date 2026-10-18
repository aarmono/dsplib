/**
 * @file fax_v29_tx.c
 * @brief The fax V.29 transmitter (T.30 high-speed data at 7200 / 9600 bit/s): engine entry points, half-duplex state machine, modulator and VMI wrappers.
 *
 * Entry points of the engine: V29TX_create / V29TX_delete / V29TX_modem / V29TX_control / V29TX_status.  Internals: the state handlers TxHdx{Start,Quiet,AB,
 * EQCond,SCR1,Data,Idle}V29 with the dispatcher TxNextStateV29, ModDataV29 (symbol encoder + pulse shaper) and TxNoCarrierV29.  The VMI layer (fax_vmi.c)
 * reaches the engine through the v29tx_* wrappers (create, delete, process, status, control, message) and the initialiser fax_init_vmi_v29tx(). The typed
 * objects are in fax_v29_tx.h, the tables in fax_v29_tx_tables.c, the pulse shaper in fax_pulse_shaping.c; FIFO / SGD / SDM / SMC and the wrappers
 * ScrambleDataV29 / SeedScramblerV29 / SetEncoderV29 come from fax_common.h / fax_modem_wrappers.c. On 64-bit hosts this file also defines the two accessors of
 * fax_modem_views.h that reach the V.29 transmit half (faxw_v29_txsdm, faxw_v29_encsel).
 *
 * Data flow: V29TX_modem(in bits) -> FIFO (unless hdx.pend) -> loop over the state handlers; each handler produces up to one frame of 48 symbols (the remaining
 * frame count travels through the handlers as a 16-bit in / out counter and the loop repeats while it stays positive) -> ModDataV29 = SMC_encoder (symbols -> I
 * / Q samples in bundle.circ.i / q) -> pulse shaper (FAXPPS_filter, mode 0) -> output samples.
 *
 * States (TxNextStateV29, hdx.state; the case of state N is the transition OUT of N):
 * 0 start -> 1 quiet (silence, 48 symbols via TxNoCarrierV29) -> 2 AB (SGD pattern 0x4f x 2, absolute encoder, 0x80 symbols) -> 3 EQ conditioning (7-bit LFSR
 * sequence of 0 / 0xb symbols, 0x180 symbols) -> 4 SCR1 (SGD pattern V29TX_PATTERN_SCR1[rate], scrambler seed 0, differential encoder, 48 symbols) -> 5 data
 * (FIFO bits, scrambled) -> 6 idle (carrier off, until the FIFO is refilled) -> 0. Every transition is made by the handler that runs out of count; the data
 * state counts nothing and is left only by the short-block path of TxHdxDataV29 when hdx.pend is set.
 *
 * Notes on behaviour:
 *  - TxNextStateV29 switches on the state; negative or larger values take the default case (status ERROR, error flag, debug text "V29TX_DEFAULT, %d").  At
 *    debug level > 1 each case prints the name of the state it LEAVES. The DATA case enters IDLE (count 0, b[2] bit 0 set), not START.
 *  - SGD control: the machine sends a sequence part with only the pattern word and the symbol count set (the other words are zero, SGD_symbol_gen does not read
 *    them); the pattern part is NULL.
 *  - TxHdxEQCondV29 produces the symbols itself: reg = hdx.reg (7-bit LFSR, initial 0x2a), per symbol bit = reg & 1, reg = (reg | ((reg << 6) & 0x80 ^ bit <<
 *    7)) >> 1 & 0x7f, symbol = bit ? 0xb :
 *    0.
 *  - TxHdxDataV29's first call (hdx.count != 0) reports the rate in the status byte: rate 0 -> CONNECT_7200, 1 -> CONNECT_9600, any other -> ERROR.
 *  - V29TX_create: s[1] == 7200 -> rate class 0, 9600 -> 1, ANY other value -> 1 plus the error flags (b[1] |= 2, status ERROR).  The FIFO is created with size
 *    cfg.w[5] (low 16 bits) * 48 and fill 0, the SGD with 4 bits per symbol; the circular I / Q buffers hold 50 entries (cleared); circ.buf (never used) is
 *    allocated on EVERY create.
 *  - V29TX_modem returns the whole state word; an over-run (*n differs from the number of bits queued) sets b[1] |= 2 and status OVERRUN.  Under-run in
 *    TxHdxDataV29 (FIFO gave fewer bits than asked, no pend) sets b[1] |= 2, status UNDERRUN and scrambles / modulates the fill (0) bits for the rest; with
 *    pend the machine moves on (TxNextStateV29) after the short block.
 *  - V29TX_control stores w8 * V29TX_PPS_SCALE[rate] as the pulse shaper gain and re-creates the engine (V29TX_create(o, o->cfg.w)) when control byte b0d has
 *    bit 1.
 *  - v29tx_message(m, 8) has no string to return (index 8 is one past V29TX_MESG) and stores NULL.
 */
#include <string.h>
#include "fax_v29_tx.h"
#include "fax_v29_tx_tables.h"
#include "fax_common_tables.h"
#include "fax_modem_views.h"

#include "fax_debug.h" /* dsplibs_debug_level, DBG, DBG3 */
#include "core/dsp_host.h"

#include "fax_noinl.h"          /* NOINL */
#include "core/dsp_constants.h" /* RATE_* */
#define FRMSIZE 48              /* symbols per frame */
#define CIRC_N 50               /* entries of the circular I / Q buffers */

#if UINTPTR_MAX == 0xffffffffu
/* the raw-offset accessors of fax_modem_views.h read exactly these members */
_Static_assert(offsetof(V29Tx, bundle) == 0x24 && offsetof(V29TxBundle, sdm) == 0x1c && offsetof(V29TxBundle, smc.cfg.flag) == 0x38,
               "faxw_v29_txsdm / faxw_v29_encsel offsets");
#else
FaxSdm* faxw_v29_txsdm(void* o) { return &((V29Tx*)o)->bundle->sdm; }
int* faxw_v29_encsel(void* o) { return &((V29Tx*)o)->bundle->smc.cfg.flag; }
#endif

/* ===================================================================================================================== create / delete */
/**
 * @brief Create a V.29 transmitter, or re-initialise one in place.
 * @param o    existing engine to re-initialise (its FIFO, SGD and pulse shaper buffers are reused), or NULL to allocate engine, half-duplex block and bundle
 * @param cfg  7-word configuration block (see V29Cfg), or NULL for V29TX_CFG; copied into the engine
 * @return the engine, in the START state with status PROTOCOL; with a rate other than 7200 / 9600 the rate class is 9600 and the status is ERROR with the error
 *     flag set
 * @note  circ.buf is allocated on every call, so a re-create of a live engine leaks the previous block (docs/bugs.md, section J).
 * @warning Allocation results are not checked.  Re-creating with a larger FIFO scale (cfg.w[5]) than before overruns the FIFO buffer, which keeps its old
 *          allocation (docs/bugs.md, section J).
 */
V29Tx* V29TX_create(V29Tx* o, const int* cfg)
{
  int fresh = 0;
  V29TxHdx* h;
  V29TxBundle* b;
  int i, rate;
  V29Cfg cc;
  short *ibuf, *qbuf;
  FaxFifo* oldFifo;
  FaxSgd* oldSgd;
  FaxSgdCfg sgdCfg;
  FaxFifoCfg fifoCfg;
  FaxSmcCfg smcCfg;
  FaxPpsCfg ppsCfg;
  FaxSdmCfg sdmCfg;

  if (DBG)
    dsplibs_debug_printf("V.29 TX Create ");
  if (o == NULL)
  {
    if (DBG)
      dsplibs_debug_printf("New allocation\n");
    o = (V29Tx*)sysdep_malloc((unsigned int)sizeof(V29Tx));
    o->hdx = NULL;
    o->bundle = NULL;
    fresh = 1;
  }
  if (DBG)
    dsplibs_debug_printf("\n");

  /* copy the configuration; the words are used below from the local copy so that no load of o->cfg follows the stores through h / b */
  if (cfg == NULL)
    cfg = V29TX_CFG;
  for (i = 0; i < 7; i++)
    cc.w[i] = cfg[i];
  o->cfg = cc;
  h = o->hdx; /* read after the configuration copy, before the state stores */
  b = o->bundle;

  /* half-duplex block: FIFO, pattern generator and the state machine reset to START */
  if (h == NULL)
  {
    h = (V29TxHdx*)sysdep_malloc((unsigned int)sizeof(V29TxHdx));
    o->hdx = h;
    h->fifo = NULL;
    h->sgd = NULL;
  }
  oldFifo = h->fifo; /* the FIFO / SGD pointers are read before the stores into the engine */
  oldSgd = h->sgd;
  o->st.w = 0x5801; /* status PROTOCOL, flags 0x58 (0x08 / 0x10 / 0x40) */
  fifoCfg = fax_fifo_cfg;
  fifoCfg.size = (unsigned short)((unsigned int)(unsigned short)cc.w[5] * 48u);
  fifoCfg.fill = 0;
  h->fifo = FIFO_create(oldFifo, &fifoCfg);
  sgdCfg = SGD_CFG;
  sgdCfg.bits = 4;
  h->sgd = SGD_create(oldSgd, &sgdCfg);
  h->state = V29TX_STATE_START;
  h->count = 0;
  h->reg = 0x2a;
  h->pend = 0;
  h->fn = TxHdxStartV29;

  /* rate class from the configured baud rate; an unsupported rate is reported but the engine is still built (at 9600) */
  if (cc.s[1] == RATE_7200)
  {
    rate = V29RC_7200;
  }
  else if (cc.s[1] == RATE_9600)
  {
    rate = V29RC_9600;
  }
  else
  {
    rate = V29RC_9600;
    o->st.b[1] |= 2;
    o->st.b[0] = V29TX_MSG_ERROR;
  }
  h->rate = (short)rate;

  /* modulator bundle: the I / Q sample buffers (cleared) that the encoder fills and the pulse shaper reads */
  if (b == NULL)
  {
    b = (V29TxBundle*)sysdep_malloc((unsigned int)sizeof(V29TxBundle));
    o->bundle = b;
    b->circ.i = (short*)sysdep_malloc(CIRC_N * 2);
    b->circ.q = (short*)sysdep_malloc(CIRC_N * 2);
  }
  b->circ.rd = 0;
  b->circ.wr = 0;
  b->circ.buf = (unsigned short*)sysdep_malloc(CIRC_N * 2); /* every create (a re-create leaks the old block) */
  b->circ.lim = CIRC_N;
  ibuf = b->circ.i;
  qbuf = b->circ.q;
  {
    short k = 0;
    do
    {
      ibuf[k] = 0;
      qbuf[k] = 0;
      k++;
    } while (k < CIRC_N);
  }

  /* symbol encoder: 16-point constellation with I / Q sample output, absolute mapping until SetEncoderV29 selects the differential table */
  smcCfg = SMC_CFG;
  smcCfg.phaseOut = 0;
  smcCfg.flag = 1;
  smcCfg.step = 17;
  smcCfg.mod = 24;
  smcCfg.shift = 0;
  smcCfg.mask = 7;
  smcCfg.mask2 = (unsigned short)((rate != V29RC_7200) << 3);
  smcCfg.mask3 = 7;
  smcCfg.pmap = V29TX_SMC_PMAP;
  smcCfg.imap = V29TX_SMC_IMAP;
  smcCfg.qmap = V29TX_SMC_QMAP;
  smcCfg.cosv = V29TX_SMC_COSINE;
  smcCfg.sinv = V29TX_SMC_SINE;

  /* scrambler: 3 bits per symbol at 7200 bit/s, 4 at 9600 */
  sdmCfg.s[0] = (short)(3 + (rate != V29RC_7200));
  sdmCfg.s[1] = 18;
  sdmCfg.s[2] = 23;
  SDM_init(&b->sdm, &sdmCfg);
  SMC_init(&b->smc, &smcCfg);

  /* pulse shaper: 120-tap filter pair, interpolate by 10, decimate by 3, mode 0 (I / Q samples from circ.i / circ.q), gain per rate class */
  ppsCfg.up = 10;
  ppsCfg.down = 3;
  ppsCfg.mode = 0;
  ppsCfg.scale = V29TX_PPS_SCALE[rate];
  ppsCfg.step = 0;
  ppsCfg.s0e = 0;
  ppsCfg.imap = NULL;
  ppsCfg.qmap = NULL;
  ppsCfg.ifilt = V29TX_PPS_IFILT;
  ppsCfg.qfilt = V29TX_PPS_QFILT;
  ppsCfg.filtlen = 120;
  ppsCfg.s22 = 0;
  ppsCfg.w24 = cc.w[6];
  FAXPPS_init(&b->pps, &ppsCfg, fresh);
  return o;
}

/**
 * @brief Release a transmitter made by V29TX_create(): pulse shaper, sample buffers, FIFO, pattern generator and the three blocks.
 * @param o  engine; invalid after the call
 * @note  Only the descriptor's `buf` block of the latest create is freed; blocks leaked by earlier re-creates stay leaked.
 */
void V29TX_delete(V29Tx* o)
{
  V29TxBundle* b = o->bundle;
  V29TxHdx* h;

  FAXPPS_free(&b->pps);
  sysdep_free(b->circ.buf);
  b = o->bundle;
  sysdep_free(b->circ.q);
  b = o->bundle;
  sysdep_free(b->circ.i);
  sysdep_free(o->bundle);
  h = o->hdx;
  SGD_delete(h->sgd);
  h = o->hdx;
  FIFO_delete(h->fifo);
  sysdep_free(o->hdx);
  sysdep_free(o);
}

/* ===================================================================================================================== state machine */
/** Send a pattern-generator control record: a sequence part with only the pattern word and the symbol count set (the rest zero), no pattern part. */
static void sgd_pattern(V29TxHdx* h, int pattern, int count)
{
  FaxSgdCtlSeq seq;
  FaxSgdCtl ctl;
  memset(&seq, 0, sizeof seq);
  seq.pattern = pattern;
  seq.patSyms = count;
  ctl.seq = &seq;
  ctl.pat = NULL;
  SGD_control(h->sgd, &ctl);
}

/**
 * @brief Leave the current state: set up the next one (handler, count, pattern generator, encoder, scrambler) and update the status flags.
 * @param o  engine
 *
 * The switch is on the state being left; see the file header for the sequence.  The flag bytes of the state word are read once, up front, and written back as
 * whole bytes (bit 0 of b[1] = data state, bit 0 of b[2] = idle).  An unknown state selects the default case (status ERROR, error flag).
 */
NOINL void TxNextStateV29(V29Tx* o)
{
  V29TxHdx* h = o->hdx;
  int state = (int)h->state;
  unsigned char b1 = o->st.b[1]; /* read once, up front (no reload after the stores through h) */
  const unsigned char b2 = o->st.b[2];

  switch ((unsigned int)state > 6u ? 7 : state)
  {
  case V29TX_STATE_START: /* start -> quiet */
    if (DBG)
      dsplibs_debug_printf("V29TX_STATE_START\n");
    h->count = FRMSIZE;
    h->fn = TxHdxQuietV29;
    h->state = V29TX_STATE_QUIET;
    o->st.b[2] = (unsigned char)(b2 & 0xfe);
    o->st.b[1] = (unsigned char)(b1 & 0xfe);
    break;
  case V29TX_STATE_QUIET: /* quiet -> AB: SGD pattern 0x4f (2 symbols), absolute encoder, 0x80 symbols */
    if (DBG)
      dsplibs_debug_printf("V29TX_STATE_QUIET\n");
    sgd_pattern(h, 0x4f, 2);
    SetEncoderV29(o, 1);
    h = o->hdx;
    h->count = 0x80;
    h->fn = TxHdxABV29;
    h->state = V29TX_STATE_ALT;
    o->st.b[2] = (unsigned char)(b2 & 0xfe);
    o->st.b[1] = (unsigned char)(b1 & 0xfe);
    break;
  case V29TX_STATE_ALT: /* AB -> EQ conditioning */
    if (DBG)
      dsplibs_debug_printf("V29TX_STATE_ALT\n");
    h->count = 0x180;
    h->fn = TxHdxEQCondV29;
    h->state = V29TX_STATE_EQCOND;
    h->reg = 0x2a;
    o->st.b[2] = (unsigned char)(b2 & 0xfe);
    o->st.b[1] = (unsigned char)(b1 & 0xfe);
    break;
  case V29TX_STATE_EQCOND: /* EQ conditioning -> scrambled ones: SGD pattern SCR1[rate] (1 symbol), scrambler seed 0, differential encoder */
    if (DBG)
      dsplibs_debug_printf("V29TX_STATE_EQCOND\n");
    sgd_pattern(h, (int)V29TX_PATTERN_SCR1[h->rate], 1);
    SeedScramblerV29(o, 0);
    SetEncoderV29(o, 0);
    h = o->hdx;
    h->count = FRMSIZE;
    h->fn = TxHdxSCR1V29;
    h->state = V29TX_STATE_SCR1;
    o->st.b[2] = (unsigned char)(b2 & 0xfe);
    o->st.b[1] = (unsigned char)(b1 & 0xfe);
    break;
  case V29TX_STATE_SCR1: /* SCR1 -> data */
    if (DBG)
      dsplibs_debug_printf("V29TX_STATE_SCR1\n");
    h->count = 1;
    h->fn = TxHdxDataV29;
    h->state = V29TX_STATE_DATA;
    o->st.b[2] = (unsigned char)(b2 & 0xfe);
    o->st.b[1] = (unsigned char)(b1 | 1);
    break;
  case V29TX_STATE_DATA: /* data -> idle (reached through the pend path of TxHdxDataV29) */
    if (DBG)
      dsplibs_debug_printf("V29TX_STATE_DATA\n");
    h->count = 0;
    h->fn = TxHdxIdleV29;
    h->state = V29TX_STATE_IDLE;
    o->st.b[2] = (unsigned char)(b2 | 1);
    o->st.b[1] = (unsigned char)(b1 & 0xfe);
    break;
  case V29TX_STATE_IDLE: /* idle -> start */
    if (DBG)
      dsplibs_debug_printf("V29TX_STATE_IDLE\n");
    h->count = 0;
    h->fn = TxHdxStartV29;
    h->state = V29TX_STATE_START;
    o->st.b[2] = (unsigned char)(b2 & 0xfe);
    o->st.b[1] = (unsigned char)(b1 & 0xfe);
    break;
  default:
    if (DBG)
      dsplibs_debug_printf("V29TX_DEFAULT, %d\n", state);
    o->st.b[2] = (unsigned char)(b2 & 0xfe);
    o->st.b[0] = V29TX_MSG_ERROR;
    o->st.b[1] = (unsigned char)((b1 | 2) & 0xfe);
    break;
  }
}

/* ===================================================================================================================== modulator */
/**
 * @brief Encode `n` symbols into I / Q samples and pulse-shape them.
 * @param o    engine
 * @param in   symbol codes (n entries)
 * @param out  output samples
 * @param n    number of symbols (at most 50, the size of the sample buffers)
 * @return number of samples written to `out` (16 bit)
 */
NOINL int ModDataV29(V29Tx* o, short* in, short* out, unsigned short n)
{
  V29TxBundle* b = o->bundle;

  SMC_encoder(&b->smc, &b->circ, (const unsigned short*)in, n);
  b = o->bundle;
  return (unsigned short)FAXPPS_filter(&b->pps, &b->circ, out, n);
}

/**
 * @brief Produce the output for `n` symbol periods with the carrier off (zero I / Q samples through the pulse shaper).
 * @param o    engine
 * @param in   unused
 * @param out  output samples
 * @param n    number of symbol periods (at most 50)
 * @return number of samples written to `out` (16 bit)
 */
NOINL int TxNoCarrierV29(V29Tx* o, short* in, short* out, unsigned short n)
{
  V29TxBundle* b = o->bundle;
  short* ibuf = b->circ.i;
  short* qbuf = b->circ.q;
  int lim = (int)b->circ.lim;
  int wr = (int)b->circ.wr;
  unsigned short i;
  int r;

  (void)in;
  for (i = 0; i < n; i++)
  {
    short nw = (short)(wr + 1);
    ibuf[wr] = 0;
    qbuf[wr] = 0;
    wr = nw < (short)lim ? (int)nw : 0;
  }
  r = (unsigned short)FAXPPS_filter(&b->pps, &b->circ, out, n);
  o->bundle->circ.wr = (short)wr;
  return r;
}

/* ===================================================================================================================== handlers */
/** START state: nothing to send, move on to QUIET. */
int TxHdxStartV29(V29Tx* o, short* in, short* out, unsigned short* n)
{
  (void)in;
  (void)out;
  (void)n;
  TxNextStateV29(o);
  return 0;
}

/** How much of this call a counting state takes: min(count, n). */
static unsigned int take(V29TxHdx* h, unsigned int n)
{
  unsigned int cnt = (unsigned int)(unsigned short)h->count;
  return (int)(short)cnt > (int)n ? n : cnt;
}

/**
 * @brief IDLE state: carrier off; leaves the state as soon as the FIFO holds data.
 * @param o    engine
 * @param in   scratch buffer
 * @param out  output samples
 * @param n    in: symbols still wanted this frame; out: symbols left (0 when the whole request was sent as silence)
 * @return number of samples written (0 when the state was left)
 */
int TxHdxIdleV29(V29Tx* o, short* in, short* out, unsigned short* n)
{
  V29TxHdx* h = o->hdx;
  unsigned int m;
  int r;

  o->st.b[0] = V29TX_MSG_IDLE;
  if (h->fifo->count != 0)
  {
    TxNextStateV29(o);
    return 0;
  }
  m = *n;
  r = TxNoCarrierV29(o, in, out, (unsigned short)m);
  *n = (unsigned short)(*n - m);
  return (short)r;
}

/**
 * @brief QUIET state: silence for `count` symbols.
 * @param o    engine
 * @param in   scratch buffer
 * @param out  output samples
 * @param n    in: symbols still wanted this frame; out: symbols left after this call
 * @return number of samples written (0 when the state was left)
 */
int TxHdxQuietV29(V29Tx* o, short* in, short* out, unsigned short* n)
{
  V29TxHdx* h = o->hdx;
  unsigned int m;
  int r;

  o->st.b[0] = V29TX_MSG_PROTOCOL;
  if ((short)h->count <= 0)
  {
    TxNextStateV29(o);
    return 0;
  }
  m = take(h, *n);
  h->count = (short)(h->count - m);
  r = TxNoCarrierV29(o, in, out, (unsigned short)m);
  *n = (unsigned short)(*n - m);
  return (short)r;
}

/**
 * @brief EQ-conditioning state: the 7-bit LFSR sequence of constellation points 0 / 0xb, unscrambled.
 * @param o    engine
 * @param in   scratch buffer for the symbols
 * @param out  output samples
 * @param n    in: symbols still wanted this frame; out: symbols left after this call
 * @return number of samples written (0 when the state was left)
 */
int TxHdxEQCondV29(V29Tx* o, short* in, short* out, unsigned short* n)
{
  V29TxHdx* h = o->hdx;
  unsigned int m;
  int r, reg;
  unsigned short k;
  short* p;

  o->st.b[0] = V29TX_MSG_PROTOCOL;
  if ((short)h->count <= 0)
  {
    TxNextStateV29(o);
    return 0;
  }
  m = take(h, *n);
  h->count = (short)(h->count - m);

  /* advance the LFSR once per symbol: a set low bit selects point 0xb, a clear one point 0 */
  reg = (int)h->reg;
  p = in;
  for (k = (unsigned short)m; k != 0; k--)
  {
    int bit = reg & 1;
    unsigned int x = (unsigned int)reg | ((((unsigned int)reg << 6) & 0x80u) ^ ((unsigned int)bit << 7));
    reg = (int)((x >> 1) & 0x7fu);
    *p++ = (short)(bit ? 0xb : 0);
  }
  h->reg = (short)reg;
  r = ModDataV29(o, in, out, (unsigned short)m);
  *n = (unsigned short)(*n - m);
  return (short)r;
}

/**
 * @brief SCR1 state: the SCR1 pattern through the scrambler (scrambled ones).
 * @param o    engine
 * @param in   scratch buffer for the symbols
 * @param out  output samples
 * @param n    in: symbols still wanted this frame; out: symbols left after this call
 * @return number of samples written (0 when the state was left)
 */
int TxHdxSCR1V29(V29Tx* o, short* in, short* out, unsigned short* n)
{
  V29TxHdx* h = o->hdx;
  unsigned int m;
  int r;

  o->st.b[0] = V29TX_MSG_PROTOCOL;
  if ((short)h->count <= 0)
  {
    TxNextStateV29(o);
    return 0;
  }
  m = take(h, *n);
  h->count = (short)(h->count - m);
  SGD_symbol_gen(h->sgd, (unsigned short*)in, (short)m);
  ScrambleDataV29(o, (unsigned short*)in, (unsigned short)m);
  r = ModDataV29(o, in, out, (unsigned short)m);
  *n = (unsigned short)(*n - m);
  return (short)r;
}

/**
 * @brief AB state: the alternating pattern from the pattern generator, unscrambled, absolute encoding.
 * @param o    engine
 * @param in   scratch buffer for the symbols
 * @param out  output samples
 * @param n    in: symbols still wanted this frame; out: symbols left after this call
 * @return number of samples written (0 when the state was left)
 */
int TxHdxABV29(V29Tx* o, short* in, short* out, unsigned short* n)
{
  V29TxHdx* h = o->hdx;
  unsigned int m;
  int r;

  o->st.b[0] = V29TX_MSG_PROTOCOL;
  if ((short)h->count <= 0)
  {
    TxNextStateV29(o);
    return 0;
  }
  m = take(h, *n);
  h->count = (short)(h->count - m);
  SGD_symbol_gen(h->sgd, (unsigned short*)in, (short)m);
  r = ModDataV29(o, in, out, (unsigned short)m);
  *n = (unsigned short)(*n - m);
  return (short)r;
}

/**
 * @brief DATA state: read the next bits from the FIFO, scramble, encode and shape them.
 * @param o    engine
 * @param in   buffer receiving the FIFO bits (one symbol per entry)
 * @param out  output samples
 * @param n    in: symbols still wanted this frame; out: symbols left after this call
 * @return number of samples written
 *
 * The first call (hdx.count != 0) reports the connect code of the rate in the status byte.  If the FIFO gives fewer bits than asked: with hdx.pend set the
 * short block is sent and the machine moves on (TxNextStateV29); otherwise the status becomes UNDERRUN, the error flag is set and the missing bits are the FIFO
 * fill value (0), which are scrambled and sent too.
 */
int TxHdxDataV29(V29Tx* o, short* in, short* out, unsigned short* n)
{
  V29TxHdx* h = o->hdx;
  unsigned int got, want, left;
  int r;

  o->st.b[0] = V29TX_MSG_DATA;
  if (h->count != 0)
  { /* the one-symbol start state: report the rate in the status byte */
    short rate = h->rate;
    h->count = 0;
    if (rate == V29RC_7200)
      o->st.b[0] = V29TX_MSG_CONNECT_7200;
    else if (rate == V29RC_9600)
      o->st.b[0] = V29TX_MSG_CONNECT_9600;
    else
      o->st.b[0] = V29TX_MSG_ERROR;
  }

  want = *n;
  got = (unsigned short)FIFO_read(h->fifo, (unsigned short*)in, (unsigned short)want);
  left = *n;
  if ((unsigned short)left > (unsigned short)got)
  { /* under-run: the FIFO gave less than asked (the rest is the fill value) */
    if (o->hdx->pend != 0)
    {
      *n = (unsigned short)(left - got);
      ScrambleDataV29(o, (unsigned short*)in, (unsigned short)got);
      r = ModDataV29(o, in, out, (unsigned short)got);
      TxNextStateV29(o);
      return (short)r;
    }
    o->st.b[1] |= 2;
    got = (unsigned short)left;
    o->st.b[0] = V29TX_MSG_UNDERRUN;
  }
  ScrambleDataV29(o, (unsigned short*)in, (unsigned short)got);
  r = ModDataV29(o, in, out, (unsigned short)got);
  *n = (unsigned short)(*n - got);
  return (short)r;
}

/* ===================================================================================================================== modem / control / status */
/**
 * @brief Process one block of transmit bits: queue them and run the state machine until a whole frame of 48 symbols has been produced.
 * @param o    engine
 * @param in   input bits (one per entry, as FIFO_write takes them); also the scratch buffer of the handlers, so it must hold at least 48 entries
 * @param out  output samples
 * @param n    in: number of input bits; out: number of output samples
 * @return the state word (status code in the low byte, flags above it)
 *
 * With hdx.pend set the input is not queued (the machine produces its own signal).  If not all bits fit into the FIFO, the status becomes OVERRUN and the error
 * flag is set.
 * @warning `out` receives the samples of the whole frame; the caller must size it for the interpolated frame length.
 */
int V29TX_modem(V29Tx* o, short* in, short* out, unsigned short* n)
{
  V29TxHdx* h = o->hdx;
  const unsigned int n0 = *n; /* read before the flag store */
  unsigned int queued;
  unsigned short frm;
  int total = 0;
  short* op = out;
  int r;

  o->st.b[1] &= 0xfd;
  if (h->pend != 0)
    queued = n0;
  else
    queued = (unsigned short)FIFO_write(h->fifo, (const unsigned short*)in, (unsigned short)n0);

  /* run the handlers until the 48 symbols of the frame are used up (each handler decrements the count) */
  frm = FRMSIZE;
  do
  {
    r = (short)h->fn(o, in, op, &frm);
    total = (short)(total + r);
    op += r;
    h = o->hdx;
  } while ((short)frm > 0);

  if ((unsigned short)n0 != (unsigned short)queued)
  {
    o->st.b[1] |= 2;
    o->st.b[0] = V29TX_MSG_OVERRUN;
  }
  r = o->st.w; /* read before the store into *n */
  *n = (unsigned short)total;
  return r;
}

/**
 * @brief Apply a control record: pulse shaper gain, timeout, flags, and optionally re-create the engine.
 * @param o    engine
 * @param ctl  control record, or NULL
 * @return 0 when `ctl` is NULL, else 1
 *
 * The gain is w8 * V29TX_PPS_SCALE[rate]; w4 is stored as the timeout cfg.w[2]; b0c bit 2 sets cfg.b[0x10] bit 2; b0d bit 4 sets / clears hdx.pend; b0d bit 1
 * re-creates the engine from its own configuration (the state machine restarts, a block is leaked, see V29TX_create()).
 */
int V29TX_control(V29Tx* o, const FaxV29Ctl* ctl)
{
  V29TxHdx* h;
  V29TxBundle* b;
  int w8, w4;
  unsigned char c, d;

  if (ctl == NULL)
    return 0;
  h = o->hdx;
  b = o->bundle;
  w8 = ctl->w8;
  w4 = ctl->w4;
  c = ctl->b0c;
  d = ctl->b0d;
  b->pps.cfg.scale = (int)((unsigned int)w8 * (unsigned int)V29TX_PPS_SCALE[h->rate]);
  o->cfg.w[2] = w4;
  if (c & 4)
    o->cfg.b[0x10] |= 4;
  h->pend = (d & 0x10) != 0;
  if (d & 2)
    V29TX_create(o, o->cfg.w);
  return 1;
}

/**
 * @brief Fill a status record from the configuration.
 * @param o   engine
 * @param st  record to fill, or NULL
 * @return 0 when `st` is NULL, else 1
 *
 * s0 = cfg.s[0], s2 = s10 = the baud rate, the other shorts are 0 (se is not written), b14 = cfg.b[0x10] & 4, bit 0 of b15 is cleared (the rest of b15 is the
 * caller's).
 */
int V29TX_status(V29Tx* o, FaxV29Status* st)
{
  short s0, s1;
  unsigned char c10;

  if (st == NULL)
    return 0;
  s0 = o->cfg.s[0];
  s1 = o->cfg.s[1];
  c10 = o->cfg.b[0x10]; /* every field of the engine is read before the first store into the record */
  st->s0 = s0;
  st->s4 = 0;
  st->s6 = 0;
  st->s8 = 0;
  st->s2 = s1;
  st->sa = 0;
  st->sc = 0;
  st->s12 = 0;
  st->s10 = s1;
  st->b15 &= 0xfe;
  st->b14 = (unsigned char)(c10 & 4);
  return 1;
}

/* ===================================================================================================================== VMI wrappers */
/**
 * @brief VMI initialiser of the V.29 transmit modem: allocate its configuration block and fill the modem view.
 * @param m       VMI modem view to initialise
 * @param rate    baud rate stored in the configuration (7200 / 9600)
 * @param unused  ignored
 * @param user    engine handle (may be NULL; v29tx_create passes it to V29TX_create); also stored, truncated to 32 bits, in cfg.w[6]
 * @return the rate (sign-extended 16 bit)
 *
 * The view gets the frame geometry of this modem (shorts 0x60 / 0x35 / 0 / 9), bit buffer marker 1 and the configuration block in `u10.cfg`.
 * The configuration block is allocated here and belongs to the init block `m` (`u10.cfg`); the caller frees it (`_init_transmitter` / `_delete_data_tx_modem`, fax_t30_tx.c).
 * @warning v29tx_create stores `u10.s10 = 0` into the 16-bit view of the same union, which would truncate `u10.cfg` and leak the block if the init block itself were passed to it;
 *          the production path gives it the VMI's own sub-object instead (bugs.md V47), and no code passes the init block.  The truncated user word in cfg.w[6] only reaches
 *          the pulse shaper configuration, where it is never read.
 */
int fax_init_vmi_v29tx(FaxVmiV29* m, unsigned short rate, int unused, void* user)
{
  V29Cfg* c = (V29Cfg*)sysdep_malloc(sizeof(V29Cfg));
  int i;

  (void)unused;
  if (2 < dsplibs_debug_level)
    dsplibs_debug_printf("Initializing VMI_V29_TX Modem No ECM (Simple Packing)\n");
  for (i = 0; i < 6; i++)
    c->w[i] = V29TX_CFG[i];
  c->s[1] = (short)rate;
  c->w[6] = (int)(uintptr_t)user;
  c->s[10] = 2;
  m->w0 = 0;
  m->s08 = 0x60;
  m->s0a = 0x35;
  m->s0c = 0;
  m->s0e = 9;
  m->eng = (V29Tx*)user;
  m->u4.init1 = 1;
  m->u10.cfg = c;
  return (short)rate;
}

/**
 * @brief VMI create: (re)create the engine from `cfg` and set the frame geometry of the modem view.
 * @param m    VMI modem view; m->eng is passed to V29TX_create (NULL allocates)
 * @param cfg  7-word configuration block, or NULL for V29TX_CFG
 * @return the engine handle (also stored in m->eng)
 *
 * Sets s0c = 0x30 (symbols per frame), s0e = 3 (7200) or 4 (otherwise) and clears `u10.s10`.
 * @note `u10.s10` (the bits per word read by the FAXVMI pack functions) is cleared to 0 for this modem.  `u10` is a union with the configuration pointer `u10.cfg` set by
 *       fax_init_vmi_v29tx(), so passing that init block here would truncate the pointer (bugs.md V47); the VMI passes its own sub-object, so nothing is clobbered.
 */
V29Tx* v29tx_create(FaxVmiV29* m, const int* cfg)
{
  int w[7], i;
  V29Tx* e;

  if (cfg == NULL)
    cfg = V29TX_CFG;
  for (i = 0; i < 7; i++)
    w[i] = cfg[i];
  e = V29TX_create(m->eng, w);
  m->s0c = 0x30;
  m->eng = e;
  m->u10.s10 = 0;
  m->s0e = (short)(3 + (((V29Cfg*)w)->s[1] != RATE_7200));
  return e;
}

/** VMI delete: release the engine. */
void v29tx_delete(FaxVmiV29* m) { V29TX_delete(m->eng); }

/**
 * @brief VMI process: run the engine on the view's bit buffer.
 * @param m     VMI modem view (u4.bits holds the input bits)
 * @param out   output samples
 * @param n     in: number of input bits; out: set to 0
 * @param outn  receives the number of output samples
 * @return the engine state word
 */
int v29tx_process(FaxVmiV29* m, short* out, unsigned short* n, unsigned short* outn)
{
  int r = V29TX_modem(m->eng, m->u4.bits, out, n);

  *outn = *n;
  *n = 0;
  return r;
}

/** VMI status: forwards to V29TX_status(). */
int v29tx_status(FaxVmiV29* m, FaxV29Status* st) { return V29TX_status(m->eng, st); }

/** VMI control: forwards to V29TX_control(). */
int v29tx_control(FaxVmiV29* m, const FaxV29Ctl* ctl) { return V29TX_control(m->eng, ctl); }

/**
 * @brief VMI message: look up the status message text for a status code.
 * @param m    unused
 * @param idx  status code (V29TxStatus), 0 .. 7
 * @param out  receives the string, or NULL for an index of 8 or more
 */
void v29tx_message(FaxVmiV29* m, int idx, const char** out)
{
  (void)m;
  if ((unsigned int)idx > 8u)
    *out = NULL;
  else if ((unsigned int)idx == 8u)
    *out = NULL; /* one past the table: there is no string to return */
  else
    *out = V29TX_MESG[idx & 0xff];
}
