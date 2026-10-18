/**
 * @file fax_v17_tx.c
 * @brief The fax V.17 transmitter (T.30 high-speed data at 7200 / 9600 / 12000 / 14400 bit/s): engine entry points, half-duplex state machine, constellation
 *        setup, modulator and VMI wrappers.
 *
 * Entry points of the engine: V17TX_create / V17TX_delete / V17TX_modem / V17TX_control / V17TX_status.  Internals: the state handlers
 * TxHdx{Start,Silence,TEP_,AB,Bridge,EQCond,SCR1,Data,Idle}V17 with the dispatcher TxNextStateV17, ModDataV17 (symbol encoder + pulse shaper), TxNoCarrierV17
 * and SetTxModeV17 (the rate dependent constellation setup). The VMI layer (fax_vmi.c) reaches the engine through the v17tx_* wrappers (create, delete,
 * process, status, control, message) and the initialiser fax_init_vmi_v17tx(). The typed objects are in fax_v17_tx.h, the tables in fax_v17_tx_tables.c, the
 * pulse shaper in fax_pulse_shaping.c; FIFO / SGD / SDM / SMCv17 and the wrappers ScrambleDataV17 / SeedScramblerV17 / SetEncoderV17 come from fax_common.h /
 * fax_modem_wrappers.c.  On 64-bit hosts this file also defines the three accessors of fax_modem_views.h that reach the V.17 transmit half (faxw_v17_txsdm /
 * _txsmc / _encmode).
 *
 * Data flow: V17TX_modem(in bits) -> FIFO (unless hdx.pend) -> loop over the state handlers; each handler produces up to one frame of 48 symbols (the remaining
 * frame count travels through the handlers as a 16-bit in / out counter and the loop repeats while it stays positive) -> ModDataV17 = the encoder selected by
 * bundle.encmode (SMCv17_encoder_dif / _abs / _tcm: symbols -> symbol codes in bundle.circ.buf) -> pulse shaper (FAXPPS_filter, mode 1: codes through the imap
 * / qmap tables) -> output samples.
 *
 * States (TxNextStateV17, hdx.state; the case of state N is the transition OUT of N):
 *   0 start -> 1 silence (0x30 symbols) -> 2 TEP (SGD pattern 0, 2 symbols; absolute encoder; 0x1e0 symbols) -> 3 silence (0x30) -> 4 AB (pattern 0xe, 2;
 *   absolute; 0x100) -> 5 EQ conditioning (pattern 0xf, 2; scrambled; 0xba0 symbols, or 0x26 when hdx.flag != 0; scrambler seed 0x2ecdd5) -> 6 bridge (pattern
 *   0x111, 8; differential encoder with x = 3; 0x40; only when hdx.flag == 0) -> 7 SCR1 (SetTxModeV17(rate), pattern V17TX_PATTERN_SCR1[rate], 1; trellis
 *   encoder with x = 0 after the bridge, 3 with short training; 0x30) -> 8 data (FIFO bits, scrambled) -> 9 SCR1 (0x20) -> 10 silence (0x30) -> 11 idle
 *   (carrier off, pend = 1, until the FIFO is refilled) -> 0. With hdx.flag != 0 (short training) the transition out of state 5 goes straight to 7 and performs
 *   the SCR1 setup itself.
 *
 * Notes on behaviour:
 *  - TxNextStateV17 switches on the state; negative or larger values take the default case (status ERROR, error flag, debug text "V17TX_DEFAULT, %d").  At
 *    debug level > 1 each case prints the name of the state it LEAVES.
 *  - SGD control: the machine sends a sequence part with only the pattern word and the symbol count set (the other words are zero, SGD_symbol_gen does not read
 *    them); the pattern part is NULL.
 *  - V17TX_create: s[1] == 7200 -> rate class 0, 9600 -> 1, 12000 -> 2, 14400 -> 3, ANY other value -> 3 plus the error flags (b[1] |= 2, status ERROR;
 *    hdx.nocarr is 4 in every case until SetTxModeV17 runs). The FIFO is created with size cfg.w[5] (low 16 bits) * 48 and fill 0, the SGD with 2 bits per
 *    symbol (SetTxModeV17 re-creates it with V17TX_SYM_SIZE[rate]); circ.i / circ.q stay NULL, circ.buf (50 entries, cleared) is allocated only together with
 *    a new bundle. The encoder select is NOT initialised by the create:
 *    the state machine always runs SetEncoderV17 / SetTxModeV17 before the first modulation.
 *  - SetTxModeV17 re-creates the SGD (bits = V17TX_SYM_SIZE[rate]), re-initialises the scrambler (n = bits, taps 0x12 / 0x17) but KEEPS its shift register,
 *    resets the encoder state (smc.tcmMask per rate 3 / 2 / 4 / 5, smc.w2 = 2, x = y = y2 = st = w10 = 0; k and w4 are untouched), selects the trellis encoder
 *    (encmode = 2), installs the constellation maps (VTBv17_IMAP16T / 32 / 64 / 128 and the Q maps) in the pulse shaper (mode stays 1) and sets hdx.nocarr =
 *    0x10 / 0x20 / 0x40 / 0x80 and smc.k = 1 / 2 / 3 / 4 per rate; any other rate sets the error flags only (status ERROR, b[1] = (b[1] | 2) & ~1) after the
 *    common part. V17TX_SYM_SIZE has four entries; a rate outside 0 .. 3 reads 0 there.
 *  - TxHdxDataV17's first call (hdx.count != 0) reports the rate in the status byte: rate 0 -> CONNECT_7200, 1 -> 9600, 2 -> 12000, 3 -> 14400, any other ->
 *    ERROR.
 *  - V17TX_modem returns the whole state word; an over-run (*n differs from the number of bits queued) sets b[1] |= 2 and status OVERRUN.  Under-run in
 *    TxHdxDataV17 (FIFO gave fewer bits than asked, no pend) sets b[1] |= 2, status UNDERRUN and scrambles / modulates the fill (0) bits for the rest; with
 *    pend the machine moves on (TxNextStateV17) after the short block.
 *  - V17TX_control stores w8 * V17TX_PPS_SCALE[rate] as the pulse shaper gain, copies its w10 to cfg.w[6] (the short-training flag; the V.27ter control does
 *    too, the V.29 control does not) and re-creates the engine (V17TX_create(o, o->cfg.w)) when control byte b0d has bit 1. V17TX_status:
 *    b14 = cfg.b[0x10] & 4, b15 &= 0xfe, w18 = cfg.w[6].
 *  - v17tx_create returns the rate word (cfg s[1], zero extended), not the engine handle; the engine goes to m->eng.
 *  - v17tx_message(m, 10) has no string to return (index 10 is one past V17TX_MESG) and stores NULL.
 *  - The engine uses the typed pulse shaper FAXPPS_* (fax_pulse_shaping.c), the same algorithm as the FPM_PPS_* of the V.32 / V.22 modules.
 */
#include <string.h>
#include "fax_v17_tx.h"
#include "fax_v17_tx_tables.h"
#include "fax_common_tables.h"
#include "fax_modem_views.h"

#include "fax_debug.h" /* dsplibs_debug_level, DBG, DBG3 */
#include "core/dsp_host.h"

#include "fax_noinl.h"          /* NOINL */
#include "core/dsp_constants.h" /* RATE_* */
#define FRMSIZE 48              /* symbols per frame */
#define CIRC_N 50               /* entries of the circular symbol code buffer */

#if UINTPTR_MAX == 0xffffffffu
/* the raw-offset accessors of fax_modem_views.h read exactly these members */
_Static_assert(offsetof(V17Tx, bundle) == 0x28 && offsetof(V17TxBundle, sdm) == 0x1c && offsetof(V17TxBundle, smc) == 0x34 &&
                   offsetof(V17TxBundle, encmode) == 0x8c,
               "faxw_v17_txsdm / _txsmc / _encmode offsets");
#else
FaxSdm* faxw_v17_txsdm(void* o) { return &((V17Tx*)o)->bundle->sdm; }
FaxSmcV17* faxw_v17_txsmc(void* o) { return &((V17Tx*)o)->bundle->smc; }
unsigned short* faxw_v17_encmode(void* o) { return &((V17Tx*)o)->bundle->encmode; }
#endif

/* The encoder slots: SMCv17_encoder_dif / _abs take a const input, the slot type is the tcm encoder's (it masks the input in place): two one-line adaptors. */
static void enc_dif(FaxSmcV17* s, FaxCirc* o, unsigned short* in, unsigned short n) { SMCv17_encoder_dif(s, o, in, n); }
static void enc_abs(FaxSmcV17* s, FaxCirc* o, unsigned short* in, unsigned short n) { SMCv17_encoder_abs(s, o, in, n); }

/* ===================================================================================================================== create / delete */
/**
 * @brief Create a V.17 transmitter, or re-initialise one in place.
 * @param o    existing engine to re-initialise (its FIFO, SGD and pulse shaper buffers are reused), or NULL to allocate engine, half-duplex block and bundle
 * @param cfg  8-word configuration block (see V17TxCfg), or NULL for V17TX_CFG; copied into the engine
 * @return the engine, in the START state with status PROTOCOL; with an unsupported rate the rate class is 14400 and the status is ERROR with the error flag set
 * @warning Allocation results are not checked.  Re-creating with a larger FIFO scale (cfg.w[5]) than before overruns the FIFO buffer, which keeps its old
 *          allocation (docs/bugs.md, section M).  A re-create after SetTxModeV17 resets the pulse shaper to the 5-entry SMCv17_IMAP4 / QMAP4 maps and the
 *          no-carrier code to 4 but keeps the trellis encoder selected, so the pulse shaper reads past those maps until the next SetTxModeV17; the state
 *          machine only re-creates before SCR1.
 * @note  encmode is not initialised here (see the file header).
 */
V17Tx* V17TX_create(V17Tx* o, const int* cfg)
{
  int fresh = 0;
  V17TxHdx* h;
  V17TxBundle* b;
  int i, rate;
  V17TxCfg cc;
  FaxFifo* oldFifo;
  FaxSgd* oldSgd;
  FaxSgdCfg sgdCfg;
  FaxFifoCfg fifoCfg;
  FaxSmcV17Cfg smcCfg;
  FaxPpsCfg ppsCfg;
  FaxSdmCfg sdmCfg;

  if (DBG)
    dsplibs_debug_printf("V.17 TX Create ");
  if (o == NULL)
  {
    if (DBG)
      dsplibs_debug_printf("New allocation\n");
    o = (V17Tx*)sysdep_malloc((unsigned int)sizeof(V17Tx));
    o->hdx = NULL;
    o->bundle = NULL;
    fresh = 1;
  }
  if (DBG)
    dsplibs_debug_printf("\n");

  /* copy the configuration; the words are used below from the local copy so that no load of o->cfg follows the stores through h / b */
  if (cfg == NULL)
    cfg = V17TX_CFG;
  for (i = 0; i < 8; i++)
    cc.w[i] = cfg[i];
  o->cfg = cc;
  h = o->hdx; /* read after the configuration copy, before the state stores */
  b = o->bundle;
  o->st.w = 0x5801; /* status PROTOCOL, flags 0x58 (0x08 / 0x10 / 0x40) */

  /* half-duplex block: FIFO, pattern generator and the state machine reset to START */
  if (h == NULL)
  {
    h = (V17TxHdx*)sysdep_malloc((unsigned int)sizeof(V17TxHdx));
    o->hdx = h;
    h->fifo = NULL;
    h->sgd = NULL;
  }
  oldFifo = h->fifo;
  fifoCfg = fax_fifo_cfg;
  fifoCfg.size = (unsigned short)((unsigned int)(unsigned short)cc.w[5] * 48u);
  fifoCfg.fill = 0;
  h->fifo = FIFO_create(oldFifo, &fifoCfg);
  sgdCfg = SGD_CFG;
  sgdCfg.bits = 2;
  oldSgd = h->sgd;
  h->sgd = SGD_create(oldSgd, &sgdCfg);
  h->state = V17TX_STATE_START;
  h->count = 0;
  h->nocarr = 4;
  h->pend = 0;
  h->flag = cc.w[6];
  h->fn = TxHdxStartV17;

  /* rate class from the configured baud rate; an unsupported rate is reported but the engine is still built (at 14400) */
  if (cc.s[1] == RATE_9600)
  {
    rate = V17RC_9600;
  }
  else if (cc.s[1] == RATE_7200)
  {
    rate = V17RC_7200;
  }
  else if (cc.s[1] == RATE_12000)
  {
    rate = V17RC_12000;
  }
  else if (cc.s[1] == RATE_14400)
  {
    rate = V17RC_14400;
  }
  else
  {
    rate = V17RC_14400;
    o->st.b[1] |= 2;
    o->st.b[0] = V17TX_MSG_ERROR;
  }
  h->rate = (short)rate;

  /* modulator bundle: the symbol-code buffer (cleared) that the encoder fills and the pulse shaper reads */
  if (b == NULL)
  {
    b = (V17TxBundle*)sysdep_malloc((unsigned int)sizeof(V17TxBundle));
    o->bundle = b;
    b->circ.buf = (unsigned short*)sysdep_malloc(CIRC_N * 2);
  }
  b->circ.rd = 0;
  b->circ.wr = 0;
  b->circ.i = NULL;
  b->circ.q = NULL;
  b->circ.lim = CIRC_N;
  {
    unsigned short* buf = b->circ.buf; /* read after the stores into the descriptor */
    short k = 0;
    do
    {
      buf[k] = 0;
      k++;
    } while (k < CIRC_N);
  }

  /* scrambler (2 bits per symbol until SetTxModeV17 widens it) and encoder state */
  sdmCfg.s[0] = 2;
  sdmCfg.s[1] = 18;
  sdmCfg.s[2] = 23;
  SDM_init(&b->sdm, &sdmCfg);
  smcCfg = SMCv17_CFG;
  SMCv17_init(&b->smc, &smcCfg);

  /* pulse shaper: 120-tap filter pair, interpolate by 10, decimate by 3, mode 1 (symbol codes through the small start-up maps), gain per rate class */
  ppsCfg.up = 10;
  ppsCfg.down = 3;
  ppsCfg.mode = 1;
  ppsCfg.scale = V17TX_PPS_SCALE[rate];
  ppsCfg.step = 0;
  ppsCfg.s0e = 0;
  ppsCfg.imap = SMCv17_IMAP4;
  ppsCfg.qmap = SMCv17_QMAP4;
  ppsCfg.ifilt = PPSv17_ICOFFS;
  ppsCfg.qfilt = PPSv17_QCOFFS;
  ppsCfg.filtlen = 120;
  ppsCfg.s22 = 0;
  ppsCfg.w24 = cc.w[7];
  FAXPPS_init(&b->pps, &ppsCfg, fresh);

  /* encoder dispatch table, indexed by the encoder select */
  b->enc[0] = enc_dif;
  b->enc[1] = enc_abs;
  b->enc[2] = SMCv17_encoder_tcm;
  return o;
}

/**
 * @brief Release a transmitter made by V17TX_create(): pulse shaper, symbol-code buffer, pattern generator, FIFO and the three blocks.
 * @param o  engine; invalid after the call
 */
void V17TX_delete(V17Tx* o)
{
  V17TxBundle* b = o->bundle;
  V17TxHdx* h;

  FAXPPS_free(&b->pps);
  sysdep_free(b->circ.buf);
  sysdep_free(o->bundle);
  h = o->hdx;
  SGD_delete(h->sgd);
  h = o->hdx;
  FIFO_delete(h->fifo);
  sysdep_free(o->hdx);
  sysdep_free(o);
}

/* ===================================================================================================================== rate dependent constellation */
/**
 * @brief Configure the engine for the data constellation of a rate class.
 * @param o     engine
 * @param rate  rate class V17RateClass (0 .. 3)
 *
 * Re-creates the pattern generator with the rate's bits per symbol, re-initialises the scrambler (same taps, new width) keeping its register, resets the
 * encoder state, selects the trellis encoder, installs the rate's constellation maps in the pulse shaper and sets the no-carrier code (0x10 / 0x20 / 0x40 /
 * 0x80, the zero point of the map) and the trellis parameter smc.k. For a rate outside 0 .. 3 only the common part runs and then the status becomes ERROR with
 * the error flag set (the call reads 0 as bits per symbol).
 */
void SetTxModeV17(V17Tx* o, short rate)
{
  FaxSgdCfg sgdCfg;
  FaxSdmCfg sdmCfg;
  V17TxHdx* h;
  V17TxBundle* b;
  unsigned int keep;
  short sym = (unsigned int)(int)rate < 4u ? V17TX_SYM_SIZE[rate] : (short)0; /* guard: V17TX_SYM_SIZE has four entries */

  sgdCfg = SGD_CFG;
  sgdCfg.bits = (unsigned short)sym;
  h = o->hdx;
  h->sgd = SGD_create(h->sgd, &sgdCfg);
  sdmCfg.s[0] = sym;
  sdmCfg.s[1] = 0x12;
  sdmCfg.s[2] = 0x17;
  b = o->bundle;
  keep = b->sdm.state; /* the scrambler register survives the re-initialisation */
  SDM_init(&b->sdm, &sdmCfg);

  /* reset the trellis encoder state and select it */
  b = o->bundle;
  b->smc.tcmMask = SMCv17_CFG.tcmMask;
  b->smc.w2 = 2;
  b->smc.y = 0;
  b->smc.x = 0;
  b->smc.st = 0;
  b->smc.y2 = 0;
  b->smc.w10 = 0;
  b->sdm.state = keep;
  b->encmode = V17ENC_TCM;

  /* per-rate trellis parameter, constellation maps and no-carrier code */
  switch (rate)
  {
  case V17RC_9600:
    b->smc.k = 2;
    b->smc.tcmMask = 2;
    b->pps.cfg.imap = VTBv17_IMAP32;
    b->pps.cfg.qmap = VTBv17_QMAP32;
    o->hdx->nocarr = 0x20;
    break;
  case V17RC_12000:
    b->smc.k = 3;
    b->smc.tcmMask = 4;
    b->pps.cfg.imap = VTBv17_IMAP64;
    b->pps.cfg.qmap = VTBv17_QMAP64;
    o->hdx->nocarr = 0x40;
    break;
  case V17RC_14400:
    b->smc.k = 4;
    b->smc.tcmMask = 5;
    b->pps.cfg.imap = VTBv17_IMAP128;
    b->pps.cfg.qmap = VTBv17_QMAP128;
    o->hdx->nocarr = 0x80;
    break;
  case V17RC_7200:
    b->smc.k = 1;
    b->smc.tcmMask = 3;
    b->pps.cfg.imap = VTBv17_IMAP16T;
    b->pps.cfg.qmap = VTBv17_QMAP16T;
    o->hdx->nocarr = 0x10;
    break;
  default:
  {
    unsigned char b1 = o->st.b[1];
    o->st.b[0] = V17TX_MSG_ERROR;
    o->st.b[1] = (unsigned char)((b1 | 2) & 0xfe);
    break;
  }
  }
}

/* ===================================================================================================================== state machine */
/** Send a pattern-generator control record: a sequence part with only the pattern word and the symbol count set (the rest zero), no pattern part. */
static void sgd_pattern(V17TxHdx* h, int pattern, int count)
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
 * whole bytes (bit 0 of b[1] = data state, bit 0 of b[2] = idle).  An unknown state selects the default case (status ERROR, error flag).  The transition into
 * IDLE sets hdx.pend, which stays set (input is no longer queued) until V17TX_control or V17TX_create clears it.
 */
NOINL void TxNextStateV17(V17Tx* o)
{
  V17TxHdx* h = o->hdx;
  int state = (int)h->state;
  unsigned char b1 = o->st.b[1]; /* read once, up front (no reload after the stores through h) */
  const unsigned char b2 = o->st.b[2];

  switch ((unsigned int)state > 11u ? 12 : state)
  {
  case V17TX_STATE_START: /* start -> silence */
    if (DBG)
      dsplibs_debug_printf("V17TX_STATE_START\n");
    h->count = FRMSIZE;
    h->fn = TxHdxSilenceV17;
    h->state = V17TX_STATE_SILENCE;
    o->st.b[2] = (unsigned char)(b2 & 0xfe);
    o->st.b[1] = (unsigned char)(b1 & 0xfe);
    break;
  case V17TX_STATE_SILENCE: /* silence -> TEP: pattern 0 (2 symbols), absolute encoder, 0x1e0 symbols */
    if (DBG)
      dsplibs_debug_printf("V17TX_STATE_SILENCE\n");
    sgd_pattern(h, 0, 2);
    SetEncoderV17(o, V17ENC_ABS, 0);
    h = o->hdx;
    h->count = 0x1e0;
    h->fn = TxHdxTEP_V17;
    h->state = V17TX_STATE_TEP;
    o->st.b[2] = (unsigned char)(b2 & 0xfe);
    o->st.b[1] = (unsigned char)(b1 & 0xfe);
    break;
  case V17TX_STATE_TEP: /* TEP -> silence */
    if (DBG)
      dsplibs_debug_printf("V17TX_STATE_TEP\n");
    h->count = FRMSIZE;
    h->fn = TxHdxSilenceV17;
    h->state = V17TX_STATE_QUIET;
    o->st.b[2] = (unsigned char)(b2 & 0xfe);
    o->st.b[1] = (unsigned char)(b1 & 0xfe);
    break;
  case V17TX_STATE_QUIET: /* silence -> AB: pattern 0xe (2 symbols), absolute encoder, 0x100 symbols */
    if (DBG)
      dsplibs_debug_printf("V17TX_STATE_QUIET\n");
    sgd_pattern(h, 0xe, 2);
    SetEncoderV17(o, V17ENC_ABS, 0);
    h = o->hdx;
    h->count = 0x100;
    h->fn = TxHdxABV17;
    h->state = V17TX_STATE_ALT;
    o->st.b[2] = (unsigned char)(b2 & 0xfe);
    o->st.b[1] = (unsigned char)(b1 & 0xfe);
    break;
  case V17TX_STATE_ALT:
  { /* AB -> EQ conditioning: pattern 0xf (2 symbols), 0xba0 symbols (0x26 with the short training flag), scrambler seed */
    int cnt;
    if (DBG)
      dsplibs_debug_printf("V17TX_STATE_ALT\n");
    sgd_pattern(h, 0xf, 2);
    h = o->hdx;
    cnt = h->flag != 0 ? 0x26 : 0xba0;
    h->count = (short)cnt;
    h->fn = TxHdxEQCondV17;
    h->state = V17TX_STATE_EQCOND;
    h->s1c = 0;
    SeedScramblerV17(o, 0x2ecdd5);
    o->st.b[2] = (unsigned char)(b2 & 0xfe);
    o->st.b[1] = (unsigned char)(b1 & 0xfe);
    break;
  }
  case V17TX_STATE_EQCOND: /* EQ conditioning -> bridge (long training), or straight to SCR1 (short training) */
    if (DBG)
      dsplibs_debug_printf("V17TX_STATE_EQCOND\n");
    if (h->flag != 0)
    {
      SetTxModeV17(o, h->rate);
      h = o->hdx;
      sgd_pattern(h, (int)V17TX_PATTERN_SCR1[h->rate], 1);
      SetEncoderV17(o, V17ENC_TCM, 3);
      h = o->hdx;
      h->count = FRMSIZE;
      h->fn = TxHdxSCR1V17;
      h->state = V17TX_STATE_SCR1;
    }
    else
    {
      sgd_pattern(h, 0x111, 8);
      SetEncoderV17(o, V17ENC_DIFF, 3);
      h = o->hdx;
      h->count = 0x40;
      h->fn = TxHdxBridgeV17;
      h->state = V17TX_STATE_BRIDGE;
    }
    o->st.b[2] = (unsigned char)(b2 & 0xfe);
    o->st.b[1] = (unsigned char)(b1 & 0xfe);
    break;
  case V17TX_STATE_BRIDGE: /* bridge -> SCR1: constellation for the rate, pattern SCR1[rate] (1 symbol), trellis encoder with x = 0 */
    if (DBG)
      dsplibs_debug_printf("V17TX_STATE_BRIDGE\n");
    SetTxModeV17(o, h->rate);
    h = o->hdx;
    sgd_pattern(h, (int)V17TX_PATTERN_SCR1[h->rate], 1);
    SetEncoderV17(o, V17ENC_TCM, 0);
    h = o->hdx;
    h->count = FRMSIZE;
    h->fn = TxHdxSCR1V17;
    h->state = V17TX_STATE_SCR1;
    o->st.b[2] = (unsigned char)(b2 & 0xfe);
    o->st.b[1] = (unsigned char)(b1 & 0xfe);
    break;
  case V17TX_STATE_SCR1: /* SCR1 -> data */
    if (DBG)
      dsplibs_debug_printf("V17TX_STATE_SCR1\n");
    h->count = 1;
    h->fn = TxHdxDataV17;
    h->state = V17TX_STATE_DATA;
    o->st.b[2] = (unsigned char)(b2 & 0xfe);
    o->st.b[1] = (unsigned char)(b1 | 1);
    break;
  case V17TX_STATE_DATA: /* data -> SCR1 (end): pattern SCR1[rate] (1 symbol), 0x20 symbols */
    if (DBG)
      dsplibs_debug_printf("V17TX_STATE_DATA\n");
    sgd_pattern(h, (int)V17TX_PATTERN_SCR1[h->rate], 1);
    h = o->hdx;
    h->count = 0x20;
    h->fn = TxHdxSCR1V17;
    h->state = V17TX_STATE_SCR1_END;
    o->st.b[2] = (unsigned char)(b2 & 0xfe);
    o->st.b[1] = (unsigned char)(b1 | 1);
    break;
  case V17TX_STATE_SCR1_END: /* SCR1 (end) -> silence */
    if (DBG)
      dsplibs_debug_printf("V17TX_STATE_SCR1_END\n");
    h->count = FRMSIZE;
    h->fn = TxHdxSilenceV17;
    h->state = V17TX_STATE_QUIET_END;
    o->st.b[2] = (unsigned char)(b2 & 0xfe);
    o->st.b[1] = (unsigned char)(b1 | 1);
    break;
  case V17TX_STATE_QUIET_END: /* silence -> idle (pend = 1) */
    if (DBG)
      dsplibs_debug_printf("V17TX_STATE_QUIET_END\n");
    h->count = 0;
    h->fn = TxHdxIdleV17;
    h->state = V17TX_STATE_IDLE;
    h->pend = 1;
    o->st.b[2] = (unsigned char)(b2 | 1);
    o->st.b[1] = (unsigned char)(b1 & 0xfe);
    break;
  case V17TX_STATE_IDLE: /* idle -> start */
    if (DBG)
      dsplibs_debug_printf("V17TX_STATE_IDLE\n");
    h->count = 0;
    h->fn = TxHdxStartV17;
    h->state = V17TX_STATE_START;
    o->st.b[2] = (unsigned char)(b2 & 0xfe);
    o->st.b[1] = (unsigned char)(b1 & 0xfe);
    break;
  default:
    if (DBG)
      dsplibs_debug_printf("V17TX_DEFAULT, %d\n", state);
    o->st.b[2] = (unsigned char)(b2 & 0xfe);
    o->st.b[0] = V17TX_MSG_ERROR;
    o->st.b[1] = (unsigned char)((b1 | 2) & 0xfe);
    break;
  }
}

/* ===================================================================================================================== modulator */
/**
 * @brief Encode `n` symbols with the selected encoder and pulse-shape them.
 * @param o    engine
 * @param in   symbol values (n entries; the trellis encoder masks them in place)
 * @param out  output samples
 * @param n    number of symbols (at most 50, the size of the symbol-code buffer)
 * @return number of samples written to `out` (16 bit)
 * @warning bundle.encmode must have been set by SetEncoderV17 / SetTxModeV17 (the create leaves it uninitialised, docs/bugs.md section M); the state machine
 *          always does so before the first modulation.
 */
NOINL int ModDataV17(V17Tx* o, short* in, short* out, unsigned short n)
{
  V17TxBundle* b = o->bundle;

  b->enc[(short)b->encmode](&b->smc, &b->circ, (unsigned short*)in, n);
  b = o->bundle;
  return (unsigned short)FAXPPS_filter(&b->pps, &b->circ, out, n);
}

/**
 * @brief Produce the output for `n` symbol periods with the carrier off (the rate's zero-point symbol code through the pulse shaper).
 * @param o    engine
 * @param in   unused
 * @param out  output samples
 * @param n    number of symbol periods (at most 50)
 * @return number of samples written to `out` (16 bit)
 */
NOINL int TxNoCarrierV17(V17Tx* o, short* in, short* out, unsigned short n)
{
  V17TxBundle* b = o->bundle;
  unsigned short* buf = b->circ.buf;
  int lim = (int)b->circ.lim;
  int wr = (int)b->circ.wr;
  const unsigned short code = (unsigned short)o->hdx->nocarr; /* read once: the buffer stores cannot change it */
  unsigned short i;
  int r;

  (void)in;
  for (i = 0; i < n; i++)
  {
    short nw = (short)(wr + 1);
    buf[wr] = code;
    wr = nw < (short)lim ? (int)nw : 0;
  }
  r = (unsigned short)FAXPPS_filter(&b->pps, &b->circ, out, n);
  o->bundle->circ.wr = (short)wr;
  return r;
}

/* ===================================================================================================================== handlers */
/** START state: nothing to send, move on to SILENCE. */
int TxHdxStartV17(V17Tx* o, short* in, short* out, unsigned short* n)
{
  (void)in;
  (void)out;
  (void)n;
  TxNextStateV17(o);
  return 0;
}

/** How much of this call a counting state takes: min(count, n). */
static unsigned int take(V17TxHdx* h, unsigned int n)
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
 * @note  hdx.pend is set when this state is entered, so V17TX_modem no longer queues input and the FIFO stays empty: the engine remains idle until
 *        V17TX_control clears pend or V17TX_create resets the engine.
 */
int TxHdxIdleV17(V17Tx* o, short* in, short* out, unsigned short* n)
{
  V17TxHdx* h = o->hdx;
  unsigned int m;
  int r;

  o->st.b[0] = V17TX_MSG_IDLE;
  if (h->fifo->count != 0)
  {
    TxNextStateV17(o);
    return 0;
  }
  m = *n;
  r = TxNoCarrierV17(o, in, out, (unsigned short)m);
  *n = (unsigned short)(*n - m);
  return (short)r;
}

/**
 * @brief SILENCE state: carrier off for `count` symbols (the status byte is left as it is).
 * @param o    engine
 * @param in   scratch buffer
 * @param out  output samples
 * @param n    in: symbols still wanted this frame; out: symbols left after this call
 * @return number of samples written (0 when the state was left)
 */
int TxHdxSilenceV17(V17Tx* o, short* in, short* out, unsigned short* n)
{
  V17TxHdx* h = o->hdx;
  unsigned int m;
  int r;

  if ((short)h->count <= 0)
  {
    TxNextStateV17(o);
    return 0;
  }
  m = take(h, *n);
  h->count = (short)(h->count - m);
  r = TxNoCarrierV17(o, in, out, (unsigned short)m);
  *n = (unsigned short)(*n - m);
  return (short)r;
}

/**
 * @brief TEP state: the pattern from the pattern generator, unscrambled (the status byte is left as it is).
 * @param o    engine
 * @param in   scratch buffer for the symbols
 * @param out  output samples
 * @param n    in: symbols still wanted this frame; out: symbols left after this call
 * @return number of samples written (0 when the state was left)
 */
int TxHdxTEP_V17(V17Tx* o, short* in, short* out, unsigned short* n)
{
  V17TxHdx* h = o->hdx;
  unsigned int m;
  int r;

  if ((short)h->count <= 0)
  {
    TxNextStateV17(o);
    return 0;
  }
  m = take(h, *n);
  h->count = (short)(h->count - m);
  SGD_symbol_gen(h->sgd, (unsigned short*)in, (short)m);
  r = ModDataV17(o, in, out, (unsigned short)m);
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
int TxHdxABV17(V17Tx* o, short* in, short* out, unsigned short* n)
{
  V17TxHdx* h = o->hdx;
  unsigned int m;
  int r;

  o->st.b[0] = V17TX_MSG_PROTOCOL;
  if ((short)h->count <= 0)
  {
    TxNextStateV17(o);
    return 0;
  }
  m = take(h, *n);
  h->count = (short)(h->count - m);
  SGD_symbol_gen(h->sgd, (unsigned short*)in, (short)m);
  r = ModDataV17(o, in, out, (unsigned short)m);
  *n = (unsigned short)(*n - m);
  return (short)r;
}

/** Common body of the Bridge / EQCond / SCR1 handlers: pattern symbols, scrambled, modulated. */
static int scrambled_pattern(V17Tx* o, short* in, short* out, unsigned short* n)
{
  V17TxHdx* h = o->hdx;
  unsigned int m;
  int r;

  o->st.b[0] = V17TX_MSG_PROTOCOL;
  if ((short)h->count <= 0)
  {
    TxNextStateV17(o);
    return 0;
  }
  m = take(h, *n);
  h->count = (short)(h->count - m);
  SGD_symbol_gen(h->sgd, (unsigned short*)in, (short)m);
  ScrambleDataV17(o, (unsigned short*)in, (unsigned short)m);
  r = ModDataV17(o, in, out, (unsigned short)m);
  *n = (unsigned short)(*n - m);
  return (short)r;
}

/** BRIDGE state: pattern symbols, scrambled; see scrambled_pattern().  Parameters and result as for the other state handlers. */
int TxHdxBridgeV17(V17Tx* o, short* in, short* out, unsigned short* n) { return scrambled_pattern(o, in, out, n); }

/** EQ-conditioning state: pattern symbols, scrambled; see scrambled_pattern().  Parameters and result as for the other state handlers. */
int TxHdxEQCondV17(V17Tx* o, short* in, short* out, unsigned short* n) { return scrambled_pattern(o, in, out, n); }

/** SCR1 state: pattern symbols, scrambled; see scrambled_pattern().  Parameters and result as for the other state handlers. */
int TxHdxSCR1V17(V17Tx* o, short* in, short* out, unsigned short* n) { return scrambled_pattern(o, in, out, n); }

/**
 * @brief DATA state: read the next bits from the FIFO, scramble, encode and shape them.
 * @param o    engine
 * @param in   buffer receiving the FIFO bits (one symbol per entry)
 * @param out  output samples
 * @param n    in: symbols still wanted this frame; out: symbols left after this call
 * @return number of samples written
 *
 * The first call (hdx.count != 0) reports the connect code of the rate in the status byte.  If the FIFO gives fewer bits than asked: with hdx.pend set the
 * short block is sent and the machine moves on (TxNextStateV17); otherwise the status becomes UNDERRUN, the error flag is set and the missing bits are the FIFO
 * fill value (0), which are scrambled and sent too.
 */
int TxHdxDataV17(V17Tx* o, short* in, short* out, unsigned short* n)
{
  V17TxHdx* h = o->hdx;
  unsigned int got, want, left;
  int r;

  o->st.b[0] = V17TX_MSG_DATA;
  if (h->count != 0)
  { /* the one-symbol start state: report the rate in the status byte */
    short rate = h->rate;
    h->count = 0;
    if (rate == V17RC_9600)
      o->st.b[0] = V17TX_MSG_CONNECT_9600;
    else if (rate == V17RC_12000)
      o->st.b[0] = V17TX_MSG_CONNECT_12000;
    else if (rate == V17RC_14400)
      o->st.b[0] = V17TX_MSG_CONNECT_14400;
    else if (rate == V17RC_7200)
      o->st.b[0] = V17TX_MSG_CONNECT_7200;
    else
      o->st.b[0] = V17TX_MSG_ERROR;
  }

  want = *n;
  got = (unsigned short)FIFO_read(h->fifo, (unsigned short*)in, (unsigned short)want);
  left = *n;
  if ((unsigned short)left > (unsigned short)got)
  { /* under-run: the FIFO gave less than asked (the rest is the fill value) */
    if (o->hdx->pend != 0)
    {
      *n = (unsigned short)(left - got);
      ScrambleDataV17(o, (unsigned short*)in, (unsigned short)got);
      r = ModDataV17(o, in, out, (unsigned short)got);
      TxNextStateV17(o);
      return (short)r;
    }
    o->st.b[1] |= 2;
    got = (unsigned short)left;
    o->st.b[0] = V17TX_MSG_UNDERRUN;
  }
  ScrambleDataV17(o, (unsigned short*)in, (unsigned short)got);
  r = ModDataV17(o, in, out, (unsigned short)got);
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
int V17TX_modem(V17Tx* o, short* in, short* out, unsigned short* n)
{
  V17TxHdx* h = o->hdx;
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
    o->st.b[0] = V17TX_MSG_OVERRUN;
  }
  r = o->st.w; /* read before the store into *n */
  *n = (unsigned short)total;
  return r;
}

/**
 * @brief Apply a control record: pulse shaper gain, timeout, training flag, flags, and optionally re-create the engine.
 * @param o    engine
 * @param ctl  control record, or NULL
 * @return 0 when `ctl` is NULL, else 1
 *
 * The gain is w8 * V17TX_PPS_SCALE[rate]; w4 is stored as the timeout cfg.w[2], w10 as cfg.w[6] (the short-training flag, which takes effect at the next
 * create); b0c bit 2 sets cfg.b[0x10] bit 2; b0d bit 4 sets / clears hdx.pend; b0d bit 1 re-creates the engine from its own configuration (the state machine
 * restarts; see the
 * @warning of V17TX_create()).
 */
int V17TX_control(V17Tx* o, const FaxV17Ctl* ctl)
{
  V17TxHdx* h;
  V17TxBundle* b;
  int w8, w4, w10;
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
  b->pps.cfg.scale = (int)((unsigned int)w8 * (unsigned int)V17TX_PPS_SCALE[h->rate]);
  o->cfg.w[6] = w10;
  o->cfg.w[2] = w4;
  if (c & 4)
    o->cfg.b[0x10] |= 4;
  h->pend = (d & 0x10) != 0;
  if (d & 2)
    V17TX_create(o, o->cfg.w);
  return 1;
}

/**
 * @brief Fill a status record from the configuration.
 * @param o   engine
 * @param st  record to fill, or NULL
 * @return 0 when `st` is NULL, else 1
 *
 * s0 = cfg.s[0], s2 = s10 = the baud rate, the other shorts are 0 (se is not written), b14 = cfg.b[0x10] & 4, bit 0 of b15 is cleared (the rest of b15 is the
 * caller's), w18 = cfg.w[6].
 */
int V17TX_status(V17Tx* o, FaxV17Status* st)
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
  st->b14 = (unsigned char)(c10 & 4);
  st->w18 = w6;
  return 1;
}

/* ===================================================================================================================== VMI wrappers */
/**
 * @brief VMI initialiser of the V.17 transmit modem: allocate its configuration block and fill the modem view.
 * @param m       VMI modem view to initialise
 * @param rate    baud rate stored in the configuration (7200 / 9600 / 12000 / 14400)
 * @param unused  ignored
 * @param user    engine handle (may be NULL; v17tx_create passes it to V17TX_create); also stored, truncated to 32 bits, in cfg.w[7]
 * @return the rate (sign-extended 16 bit)
 *
 * The configuration selects long training (cfg.w[6] = 0).  The view gets the frame geometry of this modem (shorts 0x60 / 0x30 / 0 / 0xb), bit buffer marker 1
 * and the configuration block in `u10.cfg`.
 * The configuration block is allocated here and belongs to the init block `m` (`u10.cfg`); the caller frees it (`_init_transmitter` / `_delete_data_tx_modem`, fax_t30_tx.c).
 * @warning v17tx_create stores `u10.s10 = 0` into the 16-bit view of the same union, which would truncate `u10.cfg` and leak the block if the init block itself were passed to it;
 *          the production path gives it the VMI's own sub-object instead (bugs.md V47), and no code passes the init block.  The truncated user word in cfg.w[7] only reaches
 *          the pulse shaper configuration, where it is never read.
 */
int fax_init_vmi_v17tx(FaxVmiV17* m, unsigned short rate, int unused, void* user)
{
  V17TxCfg* c = (V17TxCfg*)sysdep_malloc(sizeof(V17TxCfg));
  int i;

  (void)unused;
  if (2 < dsplibs_debug_level)
    dsplibs_debug_printf("Initializing VMI_V17_TX Modem No ECM (Simple Packing)\n");
  for (i = 0; i < 6; i++)
    c->w[i] = V17TX_CFG[i];
  c->s[1] = (short)rate;
  c->w[7] = (int)(uintptr_t)user;
  c->s[10] = 1;
  c->w[6] = 0;
  m->w0 = 0;
  m->s08 = 0x60;
  m->s0a = 0x30;
  m->s0c = 0;
  m->s0e = 0xb;
  m->eng = (V17Tx*)user;
  m->u4.init1 = 1;
  m->u10.cfg = c;
  return (short)rate;
}

/**
 * @brief VMI create: (re)create the engine from `cfg` and set the frame geometry of the modem view.
 * @param m    VMI modem view; m->eng is passed to V17TX_create (NULL allocates) and receives the engine
 * @param cfg  8-word configuration block, or NULL for V17TX_CFG
 * @return the configured baud rate (zero extended), NOT the engine handle (that is in m->eng)
 *
 * Sets s0c = 0x30 (symbols per frame), s0e = 6 (14400), 5 (12000), 4 (9600) or 3 (otherwise) and clears `u10.s10`.
 * @note `u10.s10` (the bits per word read by the FAXVMI pack functions) is cleared to 0 for this modem.  `u10` is a union with the configuration pointer `u10.cfg` set by
 *       fax_init_vmi_v17tx(), so passing that init block here would truncate the pointer (bugs.md V47); the VMI passes its own sub-object, so nothing is clobbered.
 */
unsigned int v17tx_create(FaxVmiV17* m, const int* cfg)
{
  int w[8], i;
  unsigned short rate;
  V17Tx* e;

  if (cfg == NULL)
    cfg = V17TX_CFG;
  for (i = 0; i < 8; i++)
    w[i] = cfg[i];
  e = V17TX_create(m->eng, w);
  m->s0c = 0x30;
  m->eng = e;
  rate = (unsigned short)(((V17TxCfg*)w)->s[1]);
  if (rate == RATE_14400)
  {
    m->s0e = 6;
  }
  else if (rate == RATE_12000)
  {
    m->s0e = 5;
  }
  else
  {
    m->u10.s10 = 0;
    m->s0e = (short)(4 - (rate != RATE_9600));
    return rate;
  }
  m->u10.s10 = 0;
  return rate;
}

/** VMI delete: release the engine. */
void v17tx_delete(FaxVmiV17* m) { V17TX_delete(m->eng); }

/**
 * @brief VMI process: run the engine on the view's bit buffer.
 * @param m     VMI modem view (u4.bits holds the input bits)
 * @param out   output samples
 * @param n     in: number of input bits; out: set to 0
 * @param outn  receives the number of output samples
 * @return the engine state word
 */
int v17tx_process(FaxVmiV17* m, short* out, unsigned short* n, unsigned short* outn)
{
  int r = V17TX_modem(m->eng, m->u4.bits, out, n);

  *outn = *n;
  *n = 0;
  return r;
}

/** VMI status: forwards to V17TX_status(). */
int v17tx_status(FaxVmiV17* m, FaxV17Status* st) { return V17TX_status(m->eng, st); }

/** VMI control: forwards to V17TX_control(). */
int v17tx_control(FaxVmiV17* m, const FaxV17Ctl* ctl) { return V17TX_control(m->eng, ctl); }

/**
 * @brief VMI message: look up the status message text for a status code.
 * @param m    unused
 * @param idx  status code (V17TxStatus), 0 .. 9
 * @param out  receives the string, or NULL for an index of 10 or more
 */
void v17tx_message(FaxVmiV17* m, int idx, const char** out)
{
  (void)m;
  if ((unsigned int)idx > 10u)
    *out = NULL;
  else if ((unsigned int)idx == 10u)
    *out = NULL; /* one past the table: there is no string to return */
  else
    *out = V17TX_MESG[idx & 0xff];
}
