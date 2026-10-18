/**
 * @file fax_v27_tx.c
 * @brief The fax V.27ter transmitter (T.30 high-speed data at 2400 / 4800 bit/s): engine entry points, half-duplex state machine, modulator and VMI wrappers.
 *
 * Entry points of the engine: V27TX_create / V27TX_delete / V27TX_modem / V27TX_control / V27TX_status.  Internals: the state handlers
 * TxHdx{Start,Quiet,EQCond,SCR1,Alt,Data,Idle}V27 with the dispatcher TxNextStateV27, ModDataV27 (symbol encoder + pulse shaper) and TxNoCarrierV27. The VMI
 * layer (fax_vmi.c) reaches the engine through the v27tx_* wrappers (create, delete, process, status, control, message) and the initialiser
 * fax_init_vmi_v27tx().  The typed objects are in fax_v27_tx.h, the tables in fax_v27_tx_tables.c, the pulse shaper in fax_pulse_shaping.c; FIFO / SGD / SDMv27
 * / SMC and the wrappers ScrambleDataV27 / SetScramblerV27 come from fax_common.h / fax_modem_wrappers.c. On 64-bit hosts this file also defines the two
 * accessors of fax_modem_views.h that reach the V.27ter transmit half (faxw_v27_txsdm, faxw_v27_rate).
 *
 * Data flow: V27TX_modem(in bits) -> FIFO (unless hdx.pend) -> loop over the state handlers; each handler produces up to one frame of symbols
 * (V27TX_FRMSIZE[rate] = 24 / 32 symbols; the remaining frame count travels through the handlers as a 16-bit in / out counter and the loop repeats while it
 * stays positive) -> ModDataV27 = SMC_encoder (symbol phase indexes into bundle.circ.buf) -> pulse shaper (FAXPPS_filter) -> output samples.
 *
 * States (TxNextStateV27, hdx.state; the case of state N is the transition OUT of N):
 * 0 start -> 1 quiet (silence via TxNoCarrierV27) -> 2 carrier (SGD pattern CARR through TxHdxAltV27, 10 frames) -> 3 quiet -> 4 alt (pattern ALT,
 * V27TX_ALT_COUNT symbols) -> 5 EQ conditioning (all-ones scrambled, V27TX_EQCOND_COUNT symbols) -> 6 SCR1 (pattern SCR1, 8 symbols, SetScramblerV27) -> 7 data
 * (FIFO bits, scrambled) -> 8 SCR1 (one frame) -> 9 quiet -> 10 idle (carrier off, until the FIFO is refilled) -> 0. Every transition is made by the handler
 * that runs out of count.
 *
 * Notes on behaviour:
 *  - TxNextStateV27 switches on the state; values above 10 take the default case (status ERROR, error flag, debug text "V27TX_DEFAULT, %d").  At debug level >
 *    1 each case prints the name of the state it LEAVES.
 *  - TxHdxEQCondV27 replaces symbol i by V27TX_PATTERN_ALT / CARR according to bit 2 of the scrambled symbol i + 1 (not i); for the last symbol of a call that
 *    entry lies one past the scrambled block (see the @bug at TxHdxEQCondV27).
 *  - SGD control: the pattern part of every control record is NULL; the sequence part has only the pattern word and the symbol count (1) set, the other words
 *    are zero (SGD_symbol_gen does not read them).
 *  - V27TX_create: s[1] == 2400 -> rate class 0, 4800 -> 1, ANY other value (the default configuration carries 9600) -> 1 plus the error flags (b[1] |= 2,
 *    status ERROR). The FIFO is created with size cfg.w[5] (low 16 bits) * frame size and fill 0, the SGD with 3 bits per symbol (any rate); the phase-code
 *    buffer holds frame size + 2 entries.
 *  - V27TX_modem returns the whole state word; an over-run (*n differs from the number of bits queued) sets b[1] |= 2 and status OVERRUN.  Under-run in
 *    TxHdxDataV27 (FIFO gave fewer bits than asked, no pend) sets b[1] |= 2, status UNDERRUN and scrambles / modulates the fill (0) bits for the rest; with
 *    pend the machine moves on (TxNextStateV27) after the short block.
 *  - V27TX_control stores w8 * V27TX_PPS_SCALE[rate] as the pulse shaper gain and re-creates the engine (V27TX_create(o, o->cfg.w)) when control byte b0d has
 *    bit 1.
 *  - v27tx_message(m, 8) has no string to return (index 8 is one past V27TX_MESG) and stores NULL.
 */
#include <string.h>
#include "fax_v27_tx.h"
#include "fax_v27_tx_tables.h"
#include "fax_common_tables.h"
#include "fax_modem_views.h"

#include "fax_debug.h" /* dsplibs_debug_level, DBG, DBG3 */
#include "core/dsp_host.h"

#include "fax_noinl.h"          /* NOINL */
#include "core/dsp_constants.h" /* RATE_* */

#if UINTPTR_MAX == 0xffffffffu
/* the raw-offset accessors of fax_modem_views.h read exactly these members */
_Static_assert(offsetof(V27Tx, bundle) == 0x28 && offsetof(V27TxBundle, sdm) == 0x1c && offsetof(V27Tx, hdx) == 0x24 && offsetof(V27TxHdx, rate) == 0xc,
               "faxw_v27_txsdm / faxw_v27_rate offsets");
#else
FaxSdmV27* faxw_v27_txsdm(void* o) { return &((V27Tx*)o)->bundle->sdm; }
short* faxw_v27_rate(void* o) { return &((V27Tx*)o)->hdx->rate; }
#endif

/* ===================================================================================================================== create / delete */
/**
 * @brief Create a V.27ter transmitter, or re-initialise one in place.
 * @param o    existing engine to re-initialise (its FIFO, SGD and pulse shaper buffers are reused), or NULL to allocate engine, half-duplex block and bundle
 * @param cfg  8-word configuration block (see V27Cfg), or NULL for V27TX_CFG; copied into the engine
 * @return the engine, in the START state with status PROTOCOL; with a rate other than 2400 / 4800 the rate class is 4800 and the status is ERROR with the error
 *     flag set
 * @warning Allocation results are not checked.  Re-creating with a larger FIFO scale (cfg.w[5]) than before overruns the FIFO buffer, which keeps its old
 *          allocation; re-creating after the rate class changed from 2400 to 4800 overruns the phase-code buffer (allocated for the old frame size).
 * @note  V27TX_CFG, the default block, carries the rate 9600 and so always takes the error path.
 */
V27Tx* V27TX_create(V27Tx* o, const int* cfg)
{
  int fresh = 0;
  V27TxHdx* h;
  V27TxBundle* b;
  int i, rate, size;
  V27Cfg cc;
  unsigned short* cbuf;
  FaxFifo* oldFifo;
  FaxSgd* oldSgd;
  FaxSgdCfg sgdCfg;
  FaxFifoCfg fifoCfg;
  FaxSmcCfg smcCfg;
  FaxPpsCfg ppsCfg;
  FaxSdmCfg27 sdmCfg;

  if (DBG)
    dsplibs_debug_printf("V.27 TX Create ");
  if (o == NULL)
  {
    if (DBG)
      dsplibs_debug_printf("New allocation\n");
    o = (V27Tx*)sysdep_malloc((unsigned int)sizeof(V27Tx));
    o->hdx = NULL;
    o->bundle = NULL;
    fresh = 1;
  }
  if (DBG)
    dsplibs_debug_printf("\n");

  /* copy the configuration; the words are used below from the local copy so that no load of o->cfg follows the stores through h / b */
  if (cfg == NULL)
    cfg = V27TX_CFG;
  for (i = 0; i < 8; i++)
    cc.w[i] = cfg[i];
  o->cfg = cc;
  h = o->hdx; /* read after the configuration copy, before the state stores */
  b = o->bundle;

  /* half-duplex block: pattern generator, FIFO and the state machine reset to START */
  if (h == NULL)
  {
    h = (V27TxHdx*)sysdep_malloc((unsigned int)sizeof(V27TxHdx));
    o->hdx = h;
    h->fifo = NULL;
    h->sgd = NULL;
  }
  oldFifo = h->fifo; /* the FIFO / SGD pointers are read before the stores into the engine */
  oldSgd = h->sgd;
  o->st.w = 0x5801; /* status PROTOCOL, flags 0x58 (0x08 / 0x10 / 0x40) */
  sgdCfg = SGD_CFG;
  sgdCfg.bits = 3;
  h->sgd = SGD_create(oldSgd, &sgdCfg);
  h->pend = 0;
  h->state = V27TX_STATE_START;
  h->count = 0;
  h->fn = TxHdxStartV27;
  h->longTrain = (short)(cc.w[6] == 0);

  /* rate class from the configured baud rate; an unsupported rate is reported but the engine is still built (at 4800) */
  if (cc.s[1] == RATE_2400)
  {
    rate = V27RC_2400;
  }
  else if (cc.s[1] == RATE_4800)
  {
    rate = V27RC_4800;
  }
  else
  {
    rate = V27RC_4800;
    o->st.b[1] |= 2;
    o->st.b[0] = V27TX_MSG_ERROR;
  }
  h->rate = (short)rate;
  fifoCfg = fax_fifo_cfg;
  fifoCfg.size = (unsigned short)((unsigned int)(unsigned short)cc.w[5] * (unsigned int)V27TX_FRMSIZE[rate]);
  fifoCfg.fill = 0;
  h->fifo = FIFO_create(oldFifo, &fifoCfg);

  /* modulator bundle: the phase-code buffer (cleared) that the encoder fills and the pulse shaper reads */
  size = (short)(V27TX_FRMSIZE[rate] + 2);
  if (b == NULL)
  {
    b = (V27TxBundle*)sysdep_malloc((unsigned int)sizeof(V27TxBundle));
    o->bundle = b;
    b->circ.buf = (unsigned short*)sysdep_malloc((unsigned int)(size * 2));
  }
  cbuf = b->circ.buf; /* read before the stores into the descriptor */
  b->circ.rd = 0;
  b->circ.wr = 0;
  b->circ.i = NULL;
  b->circ.q = NULL;
  b->circ.lim = (short)size;
  if (size > 0)
  {
    short k = 0;
    do
    {
      cbuf[k] = 0;
      k++;
    } while (k < (short)size);
  }

  /* symbol encoder: phase-index output, differential phase map of the rate (4 or 8 phases) */
  smcCfg = SMC_CFG;
  smcCfg.phaseOut = 1;
  smcCfg.flag = 0;
  smcCfg.step = (short)V27TX_SMC_CRR_ADJ[rate];
  smcCfg.mod = (short)V27TX_SMC_CRR_LEN[rate];
  smcCfg.shift = 0;
  smcCfg.mask = V27TX_SMC_PHS_MASK[rate];
  smcCfg.mask2 = 0;
  smcCfg.mask3 = V27TX_SMC_PHS_MASK[rate];
  smcCfg.pmap = V27TX_SMC_PMAP[rate];
  SMC_init(&b->smc, &smcCfg);

  /* pulse shaper: the rate's interpolation / decimation factors, filters and I / Q maps, mode 1 (phase codes through the maps), gain table * cfg.w[3] */
  ppsCfg.up = (short)V27TX_PPS_UP_FACT[rate];
  ppsCfg.down = (short)V27TX_PPS_DOWN_FACT[rate];
  ppsCfg.mode = 1;
  ppsCfg.scale = (int)((unsigned int)V27TX_PPS_SCALE[rate] * (unsigned int)cc.w[3]);
  ppsCfg.step = 0;
  ppsCfg.s0e = 0;
  ppsCfg.imap = V27TX_PPS_IMAP[rate];
  ppsCfg.qmap = V27TX_PPS_QMAP[rate];
  ppsCfg.ifilt = V27TX_PPS_IFILT[rate];
  ppsCfg.qfilt = V27TX_PPS_QFILT[rate];
  ppsCfg.filtlen = (short)V27TX_PPS_FILT_LEN[rate];
  ppsCfg.s22 = 0;
  ppsCfg.w24 = cc.w[7];
  FAXPPS_init(&b->pps, &ppsCfg, fresh);

  /* scrambler */
  sdmCfg.n = 3;
  SDMv27_init(&b->sdm, &sdmCfg);
  return o;
}

/**
 * @brief Release a transmitter made by V27TX_create(): pulse shaper, phase-code buffer, FIFO, pattern generator and the three blocks.
 * @param o  engine; invalid after the call
 */
void V27TX_delete(V27Tx* o)
{
  V27TxBundle* b = o->bundle;
  V27TxHdx* h = o->hdx;

  FAXPPS_free(&b->pps);
  sysdep_free(b->circ.buf);
  sysdep_free(b);
  FIFO_delete(h->fifo);
  SGD_delete(h->sgd);
  sysdep_free(h);
  sysdep_free(o);
}

/* ===================================================================================================================== state machine */
/** Send a pattern-generator control record: a sequence part with only the pattern word set and the symbol count 1 (the rest zero), no pattern part. */
static void sgd_pattern(V27TxHdx* h, int pattern)
{
  FaxSgdCtlSeq seq;
  FaxSgdCtl ctl;
  memset(&seq, 0, sizeof seq);
  seq.pattern = pattern;
  seq.patSyms = 1;
  ctl.seq = &seq;
  ctl.pat = NULL;
  SGD_control(h->sgd, &ctl);
}

/**
 * @brief Leave the current state: set up the next one (handler, count, pattern generator, scrambler) and update the status flags.
 * @param o  engine
 *
 * The switch is on the state being left; see the file header for the sequence.  Bit 0 of b[1] marks the data state, bit 0 of b[2] idle.  An unknown state
 * selects the default case (status ERROR, error flag).
 */
NOINL void TxNextStateV27(V27Tx* o)
{
  V27TxHdx* h = o->hdx;
  int state = (int)h->state;
  int rate;
  unsigned char b1;

  switch ((unsigned int)state > 10u ? 11 : state)
  {
  case V27TX_STATE_START: /* start -> quiet */
    if (DBG)
      dsplibs_debug_printf("V27TX_STATE_START\n");
    rate = h->rate;
    h->fn = TxHdxQuietV27;
    h->state = V27TX_STATE_QUIET;
    h->count = (short)V27TX_FRMSIZE[rate];
    o->st.b[2] &= 0xfe;
    o->st.b[1] &= 0xfe;
    break;
  case V27TX_STATE_QUIET: /* quiet -> carrier: SGD pattern CARR, 10 frames of TxHdxAltV27 */
    if (DBG)
      dsplibs_debug_printf("V27TX_STATE_QUIET\n");
    sgd_pattern(h, (short)V27TX_PATTERN_CARR[h->rate]);
    h = o->hdx;
    rate = h->rate;
    h->fn = TxHdxAltV27;
    h->state = V27TX_STATE_CARR;
    h->count = (short)(V27TX_FRMSIZE[rate] * 10);
    o->st.b[2] &= 0xfe;
    o->st.b[1] &= 0xfe;
    break;
  case V27TX_STATE_CARR: /* carrier -> quiet */
    if (DBG)
      dsplibs_debug_printf("V27TX_STATE_CARR\n");
    rate = h->rate;
    h->fn = TxHdxQuietV27;
    h->state = V27TX_STATE_NOCARR;
    h->count = (short)V27TX_FRMSIZE[rate];
    o->st.b[2] &= 0xfe;
    o->st.b[1] &= 0xfe;
    break;
  case V27TX_STATE_NOCARR: /* quiet -> alternations: SGD pattern ALT, V27TX_ALT_COUNT[long-training flag] symbols */
    if (DBG)
      dsplibs_debug_printf("V27TX_STATE_NOCARR\n");
    sgd_pattern(h, (short)V27TX_PATTERN_ALT[h->rate]);
    h = o->hdx;
    rate = h->longTrain; /* here an index into the training-length tables, not a rate class */
    h->fn = TxHdxAltV27;
    h->state = V27TX_STATE_ALT;
    h->count = (short)V27TX_ALT_COUNT[rate];
    o->st.b[2] &= 0xfe;
    o->st.b[1] &= 0xfe;
    break;
  case V27TX_STATE_ALT: /* alternations -> EQ conditioning: V27TX_EQCOND_COUNT[long-training flag] symbols */
    if (DBG)
      dsplibs_debug_printf("V27TX_STATE_ALT\n");
    rate = h->longTrain;
    h->fn = TxHdxEQCondV27;
    h->state = V27TX_STATE_EQCOND;
    h->count = (short)V27TX_EQCOND_COUNT[rate];
    o->st.b[2] &= 0xfe;
    o->st.b[1] &= 0xfe;
    break;
  case V27TX_STATE_EQCOND: /* EQ conditioning -> scrambled ones: SGD pattern SCR1, 8 symbols, scrambler reset */
    if (DBG)
      dsplibs_debug_printf("V27TX_STATE_EQCOND\n");
    sgd_pattern(h, (short)V27TX_PATTERN_SCR1[h->rate]);
    h = o->hdx;
    h->count = 8;
    h->fn = TxHdxSCR1V27;
    h->state = V27TX_STATE_SCR1;
    o->st.b[2] &= 0xfe;
    o->st.b[1] &= 0xfe;
    SetScramblerV27(o);
    break;
  case V27TX_STATE_SCR1: /* SCR1 -> data */
    if (DBG)
      dsplibs_debug_printf("V27TX_STATE_SCR1\n");
    h->count = 1;
    h->fn = TxHdxDataV27;
    h->state = V27TX_STATE_DATA;
    o->st.b[2] &= 0xfe;
    b1 = o->st.b[1];
    o->st.b[1] = (unsigned char)(b1 | 1);
    break;
  case V27TX_STATE_DATA: /* data -> scrambled ones (SCR1, one frame) */
    if (DBG)
      dsplibs_debug_printf("V27TX_STATE_DATA\n");
    sgd_pattern(h, (short)V27TX_PATTERN_SCR1[h->rate]);
    h = o->hdx;
    rate = h->rate;
    h->fn = TxHdxSCR1V27;
    h->state = V27TX_STATE_TURNOFF;
    h->count = (short)V27TX_FRMSIZE[rate];
    o->st.b[2] &= 0xfe;
    o->st.b[1] &= 0xfe;
    break;
  case V27TX_STATE_TURNOFF: /* SCR1 -> quiet */
    if (DBG)
      dsplibs_debug_printf("V27TX_STATE_TURNOFF\n");
    rate = h->rate;
    h->fn = TxHdxQuietV27;
    h->state = V27TX_STATE_NOENG;
    h->count = (short)V27TX_FRMSIZE[rate];
    o->st.b[2] &= 0xfe;
    o->st.b[1] &= 0xfe;
    break;
  case V27TX_STATE_NOENG: /* quiet -> idle */
    if (DBG)
      dsplibs_debug_printf("V27TX_STATE_NOENG\n");
    h->count = 0;
    h->fn = TxHdxIdleV27;
    h->state = V27TX_STATE_IDLE;
    o->st.b[2] |= 1;
    o->st.b[1] &= 0xfe;
    break;
  case V27TX_STATE_IDLE: /* idle -> start */
    if (DBG)
      dsplibs_debug_printf("V27TX_STATE_IDLE\n");
    h->count = 0;
    h->fn = TxHdxStartV27;
    h->state = V27TX_STATE_START;
    o->st.b[2] &= 0xfe;
    o->st.b[1] &= 0xfe;
    break;
  default:
    if (DBG)
      dsplibs_debug_printf("V27TX_DEFAULT, %d\n", state);
    o->st.b[2] &= 0xfe;
    o->st.b[0] = V27TX_MSG_ERROR;
    b1 = o->st.b[1];
    o->st.b[1] = (unsigned char)((b1 | 2) & 0xfe);
    break;
  }
}

/* ===================================================================================================================== modulator */
/**
 * @brief Encode `n` symbols into phase codes and pulse-shape them.
 * @param o    engine
 * @param in   symbol codes (n entries)
 * @param out  output samples
 * @param n    number of symbols (at most the frame size)
 * @return number of samples written to `out` (16 bit)
 */
NOINL int ModDataV27(V27Tx* o, short* in, short* out, unsigned short n)
{
  V27TxBundle* b = o->bundle;

  SMC_encoder(&b->smc, &b->circ, (const unsigned short*)in, n);
  return (unsigned short)FAXPPS_filter(&b->pps, &b->circ, out, n);
}

/**
 * @brief Produce the output for `n` symbol periods with the carrier off (the zero-point phase code through the pulse shaper).
 * @param o    engine
 * @param in   unused
 * @param out  output samples
 * @param n    number of symbol periods (at most the frame size)
 * @return number of samples written to `out` (16 bit)
 */
NOINL int TxNoCarrierV27(V27Tx* o, short* in, short* out, unsigned short n)
{
  V27TxBundle* b = o->bundle;
  unsigned short* buf = b->circ.buf;
  int lim = (int)b->circ.lim;
  int wr = (int)b->circ.wr;
  unsigned short i;
  int r;
  const unsigned short sym = V27TX_NOCARR_SYMBOL[o->hdx->rate];

  (void)in;
  for (i = 0; i < n; i++)
  {
    short nw;
    buf[wr] = sym;
    nw = (short)(wr + 1);
    wr = nw < (short)lim ? (int)nw : 0;
  }
  r = (unsigned short)FAXPPS_filter(&b->pps, &b->circ, out, n);
  o->bundle->circ.wr = (short)wr;
  return r;
}

/* ===================================================================================================================== handlers */
/** START state: nothing to send, move on to QUIET. */
int TxHdxStartV27(V27Tx* o, short* in, short* out, unsigned short* n)
{
  (void)in;
  (void)out;
  (void)n;
  TxNextStateV27(o);
  return 0;
}

/** How much of this call a counting state takes: min(count, n). */
static unsigned int take(V27TxHdx* h, unsigned int n)
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
int TxHdxIdleV27(V27Tx* o, short* in, short* out, unsigned short* n)
{
  V27TxHdx* h = o->hdx;
  unsigned int m;
  int r;

  o->st.b[0] = V27TX_MSG_IDLE;
  if (h->fifo->count != 0)
  {
    TxNextStateV27(o);
    return 0;
  }
  m = *n;
  r = TxNoCarrierV27(o, in, out, (unsigned short)m);
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
int TxHdxQuietV27(V27Tx* o, short* in, short* out, unsigned short* n)
{
  V27TxHdx* h = o->hdx;
  unsigned int m;
  int r;

  o->st.b[0] = V27TX_MSG_PROTOCOL;
  if ((short)h->count <= 0)
  {
    TxNextStateV27(o);
    return 0;
  }
  m = take(h, *n);
  h->count = (short)(h->count - m);
  r = TxNoCarrierV27(o, in, out, (unsigned short)m);
  *n = (unsigned short)(*n - m);
  return (short)r;
}

/**
 * @brief EQ-conditioning state: scrambled ones, each symbol replaced by the ALT or CARR pattern according to bit 2 of the following scrambled symbol.
 * @param o    engine
 * @param in   scratch buffer for the symbols; it must hold at least m + 1 entries, m being the symbols taken in this call
 * @param out  output samples
 * @param n    in: symbols still wanted this frame; out: symbols left after this call
 * @return number of samples written (0 when the state was left)
 * @bug The loop that picks the pattern tests bit 2 of in[j + 1] for symbol j, so for the last symbol of a call it reads in[m], one entry past the scrambled
 *      block.  That entry still holds whatever the caller's buffer contained (input bits of the current block), so the last symbol of each call depends on
 *      stale data and the pattern sequence is not deterministic.
 */
int TxHdxEQCondV27(V27Tx* o, short* in, short* out, unsigned short* n)
{
  V27TxHdx* h = o->hdx;
  unsigned int m;
  int i, r;
  unsigned int k;

  o->st.b[0] = V27TX_MSG_PROTOCOL;
  if ((short)h->count <= 0)
  {
    TxNextStateV27(o);
    return 0;
  }
  m = take(h, *n);
  h->count = (short)(h->count - m);

  /* all-ones symbols through the scrambler */
  for (i = 0; i < (int)m; i++)
    in[i] = 7;
  ScrambleDataV27(o, (unsigned short*)in, (short)m);
  h = o->hdx;

  /* map each symbol to the ALT / CARR pattern by bit 2 of the NEXT scrambled symbol (in[j + 1]) */
  for (k = m; k != 0; k--)
  {
    int j = (int)(m - k);
    unsigned short nxt = (unsigned short)in[j + 1];

    in[j] = (short)((nxt & 4) ? V27TX_PATTERN_ALT[h->rate] : V27TX_PATTERN_CARR[h->rate]);
  }
  r = ModDataV27(o, in, out, (unsigned short)m);
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
int TxHdxSCR1V27(V27Tx* o, short* in, short* out, unsigned short* n)
{
  V27TxHdx* h = o->hdx;
  unsigned int m;
  int r;

  o->st.b[0] = V27TX_MSG_PROTOCOL;
  if ((short)h->count <= 0)
  {
    TxNextStateV27(o);
    return 0;
  }
  m = take(h, *n);
  h->count = (short)(h->count - m);
  SGD_symbol_gen(h->sgd, (unsigned short*)in, (short)m);
  ScrambleDataV27(o, (unsigned short*)in, (short)m);
  r = ModDataV27(o, in, out, (unsigned short)m);
  *n = (unsigned short)(*n - m);
  return (short)r;
}

/**
 * @brief Alternation / carrier state: the pattern from the pattern generator, unscrambled.
 * @param o    engine
 * @param in   scratch buffer for the symbols
 * @param out  output samples
 * @param n    in: symbols still wanted this frame; out: symbols left after this call
 * @return number of samples written (0 when the state was left)
 */
int TxHdxAltV27(V27Tx* o, short* in, short* out, unsigned short* n)
{
  V27TxHdx* h = o->hdx;
  unsigned int m;
  int r;

  o->st.b[0] = V27TX_MSG_PROTOCOL;
  if ((short)h->count <= 0)
  {
    TxNextStateV27(o);
    return 0;
  }
  m = take(h, *n);
  h->count = (short)(h->count - m);
  SGD_symbol_gen(h->sgd, (unsigned short*)in, (short)m);
  r = ModDataV27(o, in, out, (unsigned short)m);
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
 * short block is sent and the machine moves on (TxNextStateV27); otherwise the status becomes UNDERRUN, the error flag is set and the missing bits are the FIFO
 * fill value (0), which are scrambled and sent too.
 */
int TxHdxDataV27(V27Tx* o, short* in, short* out, unsigned short* n)
{
  V27TxHdx* h = o->hdx;
  unsigned int got, want, left;
  int r;

  o->st.b[0] = V27TX_MSG_DATA;
  if (h->count != 0)
  { /* the one-symbol start state: report the rate in the status byte */
    short rate = h->rate;
    h->count = 0;
    if (rate == V27RC_2400)
      o->st.b[0] = V27TX_MSG_CONNECT_2400;
    else if (rate == V27RC_4800)
      o->st.b[0] = V27TX_MSG_CONNECT_4800;
    else
      o->st.b[0] = V27TX_MSG_ERROR;
  }

  want = *n;
  got = (unsigned short)FIFO_read(h->fifo, (unsigned short*)in, (unsigned short)want);
  left = *n;
  if ((unsigned short)left > (unsigned short)got)
  { /* under-run: the FIFO gave less than asked (the rest is the fill value) */
    if (o->hdx->pend != 0)
    {
      *n = (unsigned short)(left - got);
      ScrambleDataV27(o, (unsigned short*)in, (short)got);
      r = ModDataV27(o, in, out, (unsigned short)got);
      TxNextStateV27(o);
      return (short)r;
    }
    o->st.b[1] |= 2;
    got = (unsigned short)left;
    o->st.b[0] = V27TX_MSG_UNDERRUN;
  }
  ScrambleDataV27(o, (unsigned short*)in, (short)got);
  r = ModDataV27(o, in, out, (unsigned short)got);
  *n = (unsigned short)(*n - got);
  return (short)r;
}

/* ===================================================================================================================== modem / control / status */
/**
 * @brief Process one block of transmit bits: queue them and run the state machine until a whole frame of symbols has been produced.
 * @param o    engine
 * @param in   input bits (one per entry, as FIFO_write takes them); also the scratch buffer of the handlers, so it must hold at least frame size + 1 entries
 * @param out  output samples
 * @param n    in: number of input bits; out: number of output samples
 * @return the state word (status code in the low byte, flags above it)
 *
 * With hdx.pend set the input is not queued (the machine produces its own signal).  If not all bits fit into the FIFO, the status becomes OVERRUN and the error
 * flag is set.
 * @warning `out` receives the samples of the whole frame; the caller must size it for the interpolated frame length.
 */
int V27TX_modem(V27Tx* o, short* in, short* out, unsigned short* n)
{
  V27TxHdx* h = o->hdx;
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

  /* run the handlers until the symbols of the frame are used up (each handler decrements the count) */
  frm = V27TX_FRMSIZE[h->rate];
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
    o->st.b[0] = V27TX_MSG_OVERRUN;
  }
  r = o->st.w; /* read before the store into *n */
  *n = (unsigned short)total;
  return r;
}

/**
 * @brief Apply a control record: pulse shaper gain, timeout, training selector, flags, and optionally re-create the engine.
 * @param o    engine
 * @param ctl  control record, or NULL
 * @return 0 when `ctl` is NULL, else 1
 *
 * The gain is w8 * V27TX_PPS_SCALE[rate]; w4 is stored as the timeout cfg.w[2], w10 as cfg.w[6] (the long-training selector, which takes effect at the next
 * create); b0c bit 2 sets cfg.b[0x10] bit 2; b0d bit 4 sets / clears hdx.pend; b0d bit 1 re-creates the engine from its own configuration (the state machine
 * restarts).
 */
int V27TX_control(V27Tx* o, const FaxV27Ctl* ctl)
{
  V27TxHdx* h;
  V27TxBundle* b;
  int w8, w10, w4;
  unsigned char c, d;

  if (ctl == NULL)
    return 0;
  h = o->hdx;
  b = o->bundle;
  w8 = ctl->w8;
  w4 = ctl->w4;
  w10 = ctl->w10;
  c = ctl->b0c;
  d = ctl->b0d;
  b->pps.cfg.scale = (int)((unsigned int)w8 * (unsigned int)V27TX_PPS_SCALE[h->rate]);
  o->cfg.w[6] = w10;
  o->cfg.w[2] = w4;
  if (c & 4)
    o->cfg.b[0x10] |= 4;
  h->pend = 0;
  if (d & 0x10)
    h->pend = 1;
  if (d & 2)
    V27TX_create(o, o->cfg.w);
  return 1;
}

/**
 * @brief Fill a status record from the configuration.
 * @param o   engine
 * @param st  record to fill, or NULL
 * @return 0 when `st` is NULL, else 1
 *
 * s0 = cfg.s[0], s2 = s10 = the baud rate, the other shorts are 0 (se is not written), b14 = 1 | (cfg.b[0x10] & 4), bit 0 of b15 is cleared (the rest of b15 is
 * the caller's), w18 = cfg.w[6].
 */
int V27TX_status(V27Tx* o, FaxV27Status* st)
{
  short s0, s1;
  unsigned char c10;
  int w6;

  if (st == NULL)
    return 0;
  s0 = o->cfg.s[0];
  s1 = o->cfg.s[1];
  c10 = o->cfg.b[0x10];
  w6 = o->cfg.w[6]; /* every field of the engine is read before the first store into the record */
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
  st->b14 = (unsigned char)(1 | (c10 & 4));
  st->w18 = w6;
  return 1;
}

/* ===================================================================================================================== VMI wrappers */
/**
 * @brief VMI initialiser of the V.27ter transmit modem: allocate its configuration block and fill the modem view.
 * @param m       VMI modem view to initialise
 * @param rate    baud rate stored in the configuration (2400 / 4800)
 * @param unused  ignored
 * @param user    engine handle (may be NULL; v27tx_create passes it to V27TX_create); also stored, truncated to 32 bits, in cfg.w[7]
 * @return the rate (sign-extended 16 bit)
 *
 * The view gets the frame geometry of this modem (shorts 0x40 / 0x25 / 0 / 7), bit buffer marker 1 and the configuration block in `u10.cfg`.
 * The configuration block is allocated here and belongs to the init block `m` (`u10.cfg`); the caller frees it (`_init_transmitter` / `_delete_data_tx_modem`, fax_t30_tx.c).
 * @warning v27tx_create stores `u10.s10 = 0` into the 16-bit view of the same union, which would truncate `u10.cfg` and leak the block if the init block itself were passed to it;
 *          the production path gives it the VMI's own sub-object instead (bugs.md V47), and no code passes the init block.  The truncated user word in cfg.w[7] only reaches
 *          the pulse shaper configuration, where it is never read.
 */
int fax_init_vmi_v27tx(FaxVmiV27* m, unsigned short rate, int unused, void* user)
{
  V27Cfg* c = (V27Cfg*)sysdep_malloc(sizeof(V27Cfg));
  int i;

  (void)unused;
  if (2 < dsplibs_debug_level)
    dsplibs_debug_printf("Initializing VMI_V27_TX Modem No ECM (Simple Packing)\n");
  for (i = 0; i < 6; i++)
    c->w[i] = V27TX_CFG[i];
  c->s[1] = (short)rate;
  c->s[10] = 2;
  c->w[6] = V27TX_CFG[6];
  c->w[7] = (int)(uintptr_t)user;
  m->w0 = 0;
  m->s08 = 0x40;
  m->s0a = 0x25;
  m->s0c = 0;
  m->s0e = 7;
  m->eng = user;
  m->u4.init1 = 1;
  m->u10.cfg = c;
  return (short)rate;
}

/**
 * @brief VMI create: (re)create the engine from `cfg` and set the frame geometry of the modem view.
 * @param m    VMI modem view; m->eng is passed to V27TX_create (NULL allocates) and receives the engine
 * @param cfg  8-word configuration block, or NULL for V27TX_CFG
 * @return the configured baud rate (not the engine handle; that is in m->eng)
 *
 * Sets s0c = 0x18 (2400) or 0x20 (otherwise), s0e = 2 (2400) or 3 (otherwise) and clears `u10.s10`.
 * @note `u10.s10` (the bits per word read by the FAXVMI pack functions) is cleared to 0 for this modem.  `u10` is a union with the configuration pointer `u10.cfg` set by
 *       fax_init_vmi_v27tx(), so passing that init block here would truncate the pointer (bugs.md V47); the VMI passes its own sub-object, so nothing is clobbered.
 */
int v27tx_create(FaxVmiV27* m, const int* cfg)
{
  int w[8], i;
  unsigned short rate;

  if (cfg == NULL)
    cfg = V27TX_CFG;
  for (i = 0; i < 8; i++)
    w[i] = cfg[i];
  m->eng = V27TX_create((V27Tx*)m->eng, w);
  rate = (unsigned short)(((V27Cfg*)w)->s[1]);
  m->s0c = rate == RATE_2400 ? 0x18 : 0x20;
  m->u10.s10 = 0;
  m->s0e = (short)(2 + (rate != RATE_2400));
  return rate;
}

/** VMI delete: release the engine. */
void v27tx_delete(FaxVmiV27* m) { V27TX_delete((V27Tx*)m->eng); }

/**
 * @brief VMI process: run the engine on the view's bit buffer.
 * @param m     VMI modem view (u4.bits holds the input bits)
 * @param out   output samples
 * @param n     in: number of input bits; out: set to 0
 * @param outn  receives the number of output samples
 * @return the engine state word
 */
int v27tx_process(FaxVmiV27* m, short* out, unsigned short* n, unsigned short* outn)
{
  int r = V27TX_modem((V27Tx*)m->eng, m->u4.bits, out, n);

  *outn = *n;
  *n = 0;
  return r;
}

/** VMI status: forwards to V27TX_status(). */
int v27tx_status(FaxVmiV27* m, FaxV27Status* st) { return V27TX_status((V27Tx*)m->eng, st); }

/** VMI control: forwards to V27TX_control(). */
int v27tx_control(FaxVmiV27* m, const FaxV27Ctl* ctl) { return V27TX_control((V27Tx*)m->eng, ctl); }

/**
 * @brief VMI message: look up the status message text for a status code.
 * @param m    unused
 * @param idx  status code (V27TxStatus), 0 .. 7
 * @param out  receives the string, or NULL for an index of 8 or more
 */
void v27tx_message(FaxVmiV27* m, int idx, const char** out)
{
  (void)m;
  if ((unsigned int)idx > 8u)
    *out = NULL;
  else if ((unsigned int)idx == 8u)
    *out = NULL; /* one past the table: there is no string to return */
  else
    *out = V27TX_MESG[idx & 0xff];
}
