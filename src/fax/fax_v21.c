/**
 * @file fax_v21.c
 * @brief The V.21 channel of the fax class 1 layer (the 300 bit/s T.30 control channel): receiver, transmitter and the VMI wrappers around them.
 *
 * This is the HDLC channel engine of fax_class1.c, not the Bell 103 / V.21 engine of the null-modem path (src/fsk/v21_b103.c).  Typed objects are in
 * fax_v21.h, the tables in fax_v21_tables.c.
 *
 * Both halves are built from three parts: an instance (config words and a state dword whose byte 0 is the status code and bytes 1 / 2 are flag bytes),
 * a half-duplex block ("hdx": the handler of the current state, the state number and counters) and a bundle of FPM objects.
 * - Receiver: V21RX_create() / V21RX_delete() / V21RX_modem() / V21RX_control() / V21RX_status().  The signal path is AGC -> tone detector (MTD) ->
 *   receive filter (MRF) -> FSK detector (FSD), implemented by DemodDataV21().  The states are Start (count 0x7e HDLC flags: a zero after six ones;
 *   RxHdxStartV21), Wait (wait for `wait` blocks of carrier; RxHdxWaitV21), Data (pass the demodulated bits; RxHdxDataV21), Idle (watch the carrier;
 *   RxHdxIdleV21) and Error (RxHdxErrorV21); RxNextStateV21() advances Start -> Wait -> Data -> Idle.
 * - Transmitter: V21TX_create() / V21TX_delete() / V21TX_modem() / V21TX_control() / V21TX_status().  The signal path is a 6-entry bit FIFO -> FSK
 *   modulator (FSM) -> MRF, implemented by ModDataV21() (and TxNoCarrierV21(), which sends with the output scale forced to zero).  The states are Start
 *   (TxHdxStartV21), Data (TxHdxDataV21) and Idle (TxHdxIdleV21); TxNextStateV21() cycles through them.
 * - The v21rx_* / v21tx_* functions adapt the engines to the VMI object of fax_vmi.h (create, delete, process, status, control, message).
 *
 * Each handler consumes a whole block (or what the FIFO provides) and returns the number of produced items; V21RX_modem() / V21TX_modem() loop over the
 * handlers until the block is used up.  The state transitions print their names when dsplibs_debug_level is above 1.
 */
#include "fax_v21.h"
#include "fax_v21_tables.h"
#include "fax_common_tables.h"
#include "fpm/fpm_tables.h"
#include "fsk/fsk_tables.h"

#include "fax_debug.h" /* dsplibs_debug_level, DBG, DBG3 */
#include "core/dsp_host.h"
#include "fpm/fpm.h"

#include "fax_noinl.h" /* NOINL */

/* ===================================================================================================================== create / delete */
/**
 * @brief Create or re-initialise a V.21 receiver.
 * @param o    an existing receiver to re-initialise in place (its hdx and bundle are reused), or NULL to allocate a new one
 * @param cfg  six config words (V21RX_CFG layout: channel in the first short, 300 baud, carrier loss time, user word w[5] copied to the FSD / MRF
 *             configs), or NULL for V21RX_CFG
 * @return the receiver, in state Start with status "start of protocol"
 *
 * Builds the matched filter, AGC, FSK detector and tone detector for the channel (0 = channel 1, otherwise channel 2) and clears the status and the
 * instance words.  The FPM init functions get `fresh = 1` only when the receiver itself was allocated here.
 * @warning The results of sysdep_malloc() are not checked.
 */
V21Rx* V21RX_create(V21Rx* o, const int* cfg)
{
  int fresh = 0;
  V21RxBundle* b;
  V21RxHdx* h;
  int i, chan;
  int mrfCfg[4], fsdCfg[7], mtdCfg[3];

  if (DBG)
    dsplibs_debug_printf("V.21 RX Create ");
  if (o == NULL)
  {
    if (DBG)
      dsplibs_debug_printf("New allocation\n");
    o = (V21Rx*)sysdep_malloc((unsigned int)sizeof(V21Rx));
    o->hdx = NULL;
    o->bundle = NULL;
    fresh = 1;
  }
  if (DBG)
    dsplibs_debug_printf("\n");

  /* the config words: `cfg` may alias the object (the control function re-creates the engine from its own first words), so hdx and bundle are read first */
  h = o->hdx;
  b = o->bundle;
  if (cfg == NULL)
    cfg = V21RX_CFG;
  for (i = 0; i < 6; i++)
    o->cfg.w[i] = cfg[i];

  /* the half-duplex block, in the Start state */
  chan = o->cfg.s[0];
  if (h == NULL)
  {
    h = (V21RxHdx*)sysdep_malloc((unsigned int)sizeof(V21RxHdx));
    o->hdx = h;
  }
  h->hold = 0;
  h->fn = RxHdxStartV21;
  h->state = V21RX_STATE_START;
  h->wait = 0;
  h->ones = 0;
  h->flags = 0;

  /* the instance words: status code "start" (1) and the flag byte 0x50 (bits 0x40 / 0x10 are set from the create on) */
  o->st.w = 0x5001;
  o->s30 = 0;
  o->w20 = 0;
  o->w28 = 0;
  o->w2c = 0;
  o->w34 = 0;
  o->w38 = 0;
  o->s3c = 0;
  o->w40 = 0;
  o->w44 = 0;
  o->s48 = 0;

  /* the bundle, with its 160-sample scratch buffer (the MRF output) */
  if (b == NULL)
  {
    short* buf;
    b = (V21RxBundle*)sysdep_malloc((unsigned int)sizeof(V21RxBundle));
    o->bundle = b;
    sysdep_memset(b, 0, sizeof(V21RxBundle));
    buf = (short*)sysdep_malloc(0x140);
    b->buf = buf;
    sysdep_memset(buf, 0, 0x140);
    b->mtd = NULL;
  }

  /* matched filter: 0x168 taps, interpolation 9 / decimation 10 (V21_MRF_FILT is shared with the transmitter) */
  mrfCfg[0] = (int)(0xa0000u | 9u);
  mrfCfg[1] = CFGP(V21_MRF_FILT);
  mrfCfg[2] = (int)(((unsigned int)FPM_MRF_CFG_raw[2] & 0xffff0000u) | 0x168u);
  mrfCfg[3] = o->cfg.w[5];
  FPM_MRF_init(&b->mrf, mrfCfg, fresh);
  FPM_AGC_init(&b->agc, AGCv21_CFG, fresh);
  b->w0 = 1;
  b->agcDetect = 0;
  b->noTone = 0;

  /* FSK detector: the per-channel interpolation filter and the low-pass shared by both channels */
  fsdCfg[0] = chan == 0 ? CFGP(V21RX_CHAN1_INTRP) : CFGP(V21RX_CHAN2_INTRP);
  fsdCfg[1] = (int)(((chan == 0 ? 5u : 3u) << 16) | 0xfu);
  fsdCfg[2] = CFGP(V21RX_IIR_LPF);
  fsdCfg[3] = (int)(((unsigned int)FPM_FSD_CFG_raw[3] & 0xffff0000u) | 3u);
  fsdCfg[4] = (int)(0x18u << 16);
  fsdCfg[5] = FPM_FSD_CFG_raw[5];
  fsdCfg[6] = o->cfg.w[5];
  FPM_FSD_init(&b->fsd, fsdCfg, fresh);

  /* tone detector with the per-channel coefficients */
  mtdCfg[0] = chan == 0 ? CFGP(V21_CHAN1_MTD_COEFF) : CFGP(V21_CHAN2_MTD_COEFF);
  mtdCfg[1] = (int)(0x4ccdu << 16 | 2u);
  mtdCfg[2] = (int)(((unsigned int)FPM_MTD_CFG_raw[2] & 0xffff0000u) | 0x12cu);
  b->mtd = FPM_MTD_create(b->mtd, mtdCfg);

  o->fsdOut = b->fsd.out;
  o->fsdS20 = &b->fsd.outCount;
  return o;
}

/**
 * @brief Release a V.21 receiver and everything it owns (tone detector, FSK detector, matched filter, buffer, bundle, hdx, the object itself).
 * @param o  the receiver; must not be NULL
 */
void V21RX_delete(V21Rx* o)
{
  FPM_MTD_delete(o->bundle->mtd);
  FPM_FSD_free(&o->bundle->fsd);
  FPM_MRF_free(&o->bundle->mrf);
  sysdep_free(o->bundle->buf);
  sysdep_free(o->bundle);
  sysdep_free(o->hdx);
  sysdep_free(o);
}

/**
 * @brief Create or re-initialise a V.21 transmitter.
 * @param o    an existing transmitter to re-initialise in place, or NULL to allocate a new one
 * @param cfg  seven config words (V21TX_CFG layout: channel in the first short, 300 baud, carrier loss time, ...; word 6 goes to the MRF config), or NULL
 *             for V21TX_CFG
 * @return the transmitter, in state Start with status "data mode"
 *
 * Creates the 6-entry bit FIFO, the FSK modulator and the matched filter.  A channel other than 0 or 1 sets the error flag and status.
 * @bug  The channel-specific FSK frequency pair ({1180, 980} Hz for channel 0, {1850, 1650} Hz for channel 1) is never used: the modulator is always
 *       configured from FPM_FSM_CFG_raw[0] and the fixed word 0x19000018, so every channel transmits the same default pair.
 * @warning The results of sysdep_malloc() are not checked.
 */
V21Tx* V21TX_create(V21Tx* o, const int* cfg)
{
  int fresh = 0;
  int i, w18, chan;
  unsigned short fifoW0;
  FaxFifo* fifoOld;
  V21TxBundle* b;
  V21TxHdx* h;
  FaxFifoCfg fifoCfg;
  int fsmCfg[2], mrfCfg[4];

  if (DBG)
    dsplibs_debug_printf("V.21 TX Create ");
  if (o == NULL)
  {
    if (DBG)
      dsplibs_debug_printf("New allocation\n");
    o = (V21Tx*)sysdep_malloc((unsigned int)sizeof(V21Tx));
    o->hdx = NULL;
    o->bundle = NULL;
    fresh = 1;
  }
  if (DBG)
    dsplibs_debug_printf("\n");

  /* the config words: `cfg` may alias the object (see V21RX_create), so hdx, bundle and the FIFO template are read first */
  h = o->hdx;
  b = o->bundle;
  fifoW0 = fax_fifo_cfg.w0;
  fifoOld = h ? h->fifo : NULL;
  if (cfg == NULL)
    cfg = V21TX_CFG;
  for (i = 0; i < 6; i++)
    o->cfg.w[i] = cfg[i];
  w18 = cfg[6];
  o->st.w = 0x5800; /* status code "data" (0), flag byte 0x58 */
  o->w18 = w18;
  chan = o->cfg.s[0];

  /* the half-duplex block with its bit FIFO, in the Start state */
  if (h == NULL)
  {
    h = (V21TxHdx*)sysdep_malloc((unsigned int)sizeof(V21TxHdx));
    o->hdx = h;
    fifoOld = NULL;
  }
  fifoCfg.w0 = fifoW0;
  fifoCfg.size = 6;
  fifoCfg.fill = 1;
  h->fifo = FIFO_create(fifoOld, &fifoCfg);
  h->pend = 0;
  h->fn = TxHdxStartV21;
  h->state = V21TX_STATE_START;
  h->s0e = 0;
  w18 = o->w18;

  /* the bundle, with its 160-sample scratch buffer (the FSM output) */
  if (b == NULL)
  {
    short* buf;
    b = (V21TxBundle*)sysdep_malloc((unsigned int)sizeof(V21TxBundle));
    o->bundle = b;
    sysdep_memset(b, 0, sizeof(V21TxBundle));
    buf = (short*)sysdep_malloc(0x140);
    b->buf = buf;
    sysdep_memset(buf, 0, 0x140);
  }
  if (chan != 0 && chan != 1)
  {
    o->st.b[1] |= 2;
    o->st.b[0] = V21TX_MSG_ERROR;
  }

  /* FSK modulator with the default frequency pair (see the @bug note above), then the matched filter: interpolation 10 / decimation 9 */
  fsmCfg[0] = FPM_FSM_CFG_raw[0];
  fsmCfg[1] = (int)0x19000018u;
  FPM_FSM_init(&b->fsm, fsmCfg);
  mrfCfg[0] = (int)(0x90000u | 0xau);
  mrfCfg[1] = CFGP(V21_MRF_FILT);
  mrfCfg[2] = (int)(((unsigned int)FPM_MRF_CFG_raw[2] & 0xffff0000u) | 0x168u);
  mrfCfg[3] = w18;
  FPM_MRF_init(&b->mrf, mrfCfg, fresh);
  return o;
}

/**
 * @brief Release a V.21 transmitter and everything it owns (modulator, matched filter, buffer, bundle, FIFO, hdx, the object itself).
 * @param o  the transmitter; must not be NULL
 */
void V21TX_delete(V21Tx* o)
{
  FPM_FSM_delete(&o->bundle->fsm);
  FPM_MRF_free(&o->bundle->mrf);
  sysdep_free(o->bundle->buf);
  sysdep_free(o->bundle);
  FIFO_delete(o->hdx->fifo);
  sysdep_free(o->hdx);
  sysdep_free(o);
}

/* ===================================================================================================================== receive side */
/** @brief Carrier present in the last demodulated block: the AGC detected a signal (agcDetect) and the tone detector saw no tone (noTone). */
NOINL int CarrierDetectV21(V21Rx* o) { return o->bundle->noTone & o->bundle->agcDetect; }

/**
 * @brief Signal-quality estimate for the status record: stores |FSD output| of the last block in the bundle buffer.
 * @return always 0
 * @bug  The function returns 0 whatever the signal quality, so the status SNR is always 0 and RxHdxDataV21() always sets the "SNR low" flag (bit 7 of
 *       the flag byte).
 */
NOINL int GetSNRV21(V21Rx* o)
{
  V21RxBundle* b = o->bundle;
  short n = b->fsd.outCount;
  short* src = b->fsd.out;
  short* dst = b->buf;
  short i;
  if (n > 0)
  {
    i = 0;
    do
    {
      int v = src[i];
      dst[i] = (short)(v < 0 ? -v : v);
      i = (short)(i + 1);
    } while (i < n);
  }
  return 0;
}

/**
 * @brief Demodulate one block of samples to bits.
 * @param o    receiver
 * @param in   `n` input samples; the AGC works in place, and the block is zeroed when a tone is detected outside the Data state
 * @param out  receives the demodulated bits
 * @param n    number of input samples
 * @return number of bits produced
 *
 * Runs the AGC (its detect flag goes to `agcDetect`), the tone detector (`noTone` = 0 when a tone is present), the matched filter and the FSK detector.
 */
NOINL int DemodDataV21(V21Rx* o, short* in, short* out, unsigned short n)
{
  V21RxBundle* b = o->bundle;
  V21RxHdx* h = o->hdx;
  short* buf = b->buf; /* the pointers are read once, before any store through `in` (aliasing independent code) */
  FpmMtd* mtd = b->mtd;
  unsigned int produced;

  FPM_AGC_agc(&b->agc, in, n);
  b->agcDetect = b->agc.detect;
  b->noTone = 1;
  if ((unsigned short)FPM_MTD_detect(mtd, in, (short)n) != 0)
  {
    unsigned short i;
    b->noTone = 0;
    if (h->fn != RxHdxDataV21)
    {
      i = 0;
      while (i < n)
      {
        in[i] = 0;
        i = (unsigned short)(i + 1);
      }
    }
  }
  produced = FPM_MRF_filter(&b->mrf, in, buf, (short)n);
  return (int)(unsigned short)FPM_FSD_demodulate(&b->fsd, (unsigned short*)buf, out, (short)(unsigned short)produced);
}

/**
 * @brief Advance the receive state machine: Start -> Wait -> Data -> Idle.
 *
 * Installs the handler of the next state and updates the flag bytes (entering Data sets the data flag; leaving Data sets the idle bit and clears the data
 * flag).  The state names are printed at debug level above 1.  Any other state (Idle, Error) takes the default case: the status becomes "internal error"
 * and the error flag is set.
 */
NOINL void RxNextStateV21(V21Rx* o)
{
  V21RxHdx* h = o->hdx;
  switch (h->state)
  {
  case V21RX_STATE_START:
    if (DBG)
      dsplibs_debug_printf("V21RX_STATE_START\n");
    h->wait = 1;
    h->fn = RxHdxWaitV21;
    h->state = V21RX_STATE_WAIT;
    break;
  case V21RX_STATE_WAIT:
    if (DBG)
      dsplibs_debug_printf("V21RX_STATE_WAIT\n");
    h->wait = 0;
    h->fn = RxHdxDataV21;
    h->state = V21RX_STATE_DATA;
    o->st.b[1] |= 1;
    break;
  case V21RX_STATE_DATA:
    if (DBG)
      dsplibs_debug_printf("V21RX_STATE_DATA\n");
    h->fn = RxHdxIdleV21;
    h->state = V21RX_STATE_IDLE;
    h->wait = 0;
    h->hold = 0;
    o->st.b[2] |= 1;
    o->st.b[1] &= 0xfe;
    break;
  default:
  {
    unsigned char f;
    if (DBG)
      dsplibs_debug_printf("V21RX_DEFAULT, %d\n", (int)h->state);
    f = o->st.b[1];
    o->st.b[2] &= 0xfe;
    o->st.b[0] = V21RX_MSG_ERROR;
    f |= 2;
    f &= 0xde;
    o->st.b[1] = f;
    break;
  }
  }
}

/** @brief Error state: set the error flag, demodulate and discard the block (`*n` = 0), produce nothing. */
int RxHdxErrorV21(V21Rx* o, short* in, short* out, unsigned short* n)
{
  unsigned short cnt = *n; /* read before the flag store (aliasing independent code) */
  o->st.b[1] |= 2;
  DemodDataV21(o, in, out, cnt);
  *n = 0;
  return 0;
}

/** @brief Idle state (after the data): demodulate and discard, report status "idle" and track the carrier flag; produces nothing. */
int RxHdxIdleV21(V21Rx* o, short* in, short* out, unsigned short* n)
{
  int cd;
  DemodDataV21(o, in, out, *n);
  cd = CarrierDetectV21(o);
  *n = 0;
  o->st.b[1] &= 0xdf;
  o->st.b[0] = V21RX_MSG_IDLE;
  if (cd)
    o->st.b[1] |= 0x20;
  return 0;
}

/**
 * @brief Start state: demodulate the block and count HDLC flags (a zero bit after six one bits) across blocks.
 *
 * With carrier present and more than 4 flags counted, the carrier flag is set and the machine advances to Wait.  Produces no output bits (`*n` = 0).
 */
int RxHdxStartV21(V21Rx* o, short* in, short* out, unsigned short* n)
{
  unsigned short cnt, idx, cin = *n;
  short ones, flags;
  int cd;
  V21RxHdx* h = o->hdx;

  o->st.b[0] = V21RX_MSG_START;
  cnt = (unsigned short)DemodDataV21(o, in, out, cin);
  *n = 0;

  /* flag detector over the demodulated bits */
  idx = 0;
  ones = h->ones;
  flags = h->flags;
  while (cnt != 0)
  {
    if (ones == 6 && out[idx] == 0)
      flags = (short)(flags + 1);
    ones = out[idx] != 0 ? (short)(ones + 1) : 0;
    cnt = (unsigned short)(cnt - 1);
    idx = (unsigned short)(idx + 1);
  }
  h->ones = ones;
  h->flags = flags;

  cd = CarrierDetectV21(o); /* read before the flag store (aliasing independent code) */
  o->st.b[1] &= 0xdf;
  if (cd && flags > 4)
  {
    o->st.b[1] |= 0x20;
    RxNextStateV21(o);
  }
  return 0;
}

/**
 * @brief Wait state: with carrier, count down `wait` blocks, then report "CONNECT" and advance to Data; without carrier go to the Error state.
 * @return the bits produced by the block that ended the wait, otherwise 0
 */
int RxHdxWaitV21(V21Rx* o, short* in, short* out, unsigned short* n)
{
  unsigned short d;
  int cd;
  V21RxHdx* h;
  d = (unsigned short)DemodDataV21(o, in, out, *n);
  cd = CarrierDetectV21(o); /* read before the *n store (aliasing independent code) */
  *n = 0;
  if (cd)
  {
    o->st.b[1] |= 0x20;
    h = o->hdx;
    o->st.b[0] = V21RX_MSG_WAIT;
    h->wait = (short)(h->wait - 1);
    if (h->wait > 0)
      return 0;
    o->st.b[0] = V21RX_MSG_CONNECT;
    RxNextStateV21(o);
    return (short)d;
  }
  h = o->hdx;
  h->fn = RxHdxErrorV21;
  h->state = V21RX_STATE_ERROR;
  o->st.b[0] = V21RX_MSG_LOSS_CARRIER;
  o->st.b[1] = (unsigned char)((o->st.b[1] | 2) & 0xdf);
  return 0;
}

/**
 * @brief Data state: with carrier (and `hold` clear) demodulate the block and return its bits; otherwise leave the state.
 * @return the number of bits produced
 *
 * Sets the "SNR low" flag (bit 7 of the flag byte) when GetSNRV21() is 5 or below, which is always (see GetSNRV21()).  Loss of carrier or `hold` ends the
 * data state through RxNextStateV21().
 */
int RxHdxDataV21(V21Rx* o, short* in, short* out, unsigned short* n)
{
  unsigned short d;
  int cd = CarrierDetectV21(o);
  int hold = o->hdx->hold;
  o->st.b[1] |= 0x20;
  o->st.b[0] = V21RX_MSG_DATA;
  if (cd && hold == 0)
  {
    unsigned char f;
    d = (unsigned short)DemodDataV21(o, in, out, *n);
    f = (unsigned char)(o->st.b[1] & 0x7f);
    *n = 0;
    if ((short)GetSNRV21(o) <= 5)
      f |= 0x80;
    o->st.b[1] = f;
    return (short)d;
  }
  o->st.b[1] &= 0xdf;
  RxNextStateV21(o);
  return 0;
}

/**
 * @brief Process one block of received samples through the state machine.
 * @param o    receiver
 * @param in   input samples
 * @param out  receives the output bits
 * @param n    in: number of samples; out: number of bits produced
 * @return the state dword (status code in the low byte, flag bytes above it)
 *
 * Clears the error flag, then calls the current state's handler repeatedly until it has consumed all input.
 */
int V21RX_modem(V21Rx* o, short* in, short* out, unsigned short* n)
{
  int total = 0;
  unsigned short cur;
  o->st.b[1] &= 0xfd;
  cur = *n;
  do
  {
    int old = (short)cur, ret, consumed;
    ret = (short)o->hdx->fn(o, in, out, n);
    cur = *n;
    consumed = old - (int)(unsigned short)cur;
    out += ret;
    in += consumed;
    total = (short)(ret + total);
  } while (cur != 0);
  *n = (unsigned short)total;
  return o->st.w;
}

/**
 * @brief Apply a control record to the receiver.
 * @param o    receiver
 * @param ctl  control record, or NULL (returns 0, nothing done)
 * @return 1 when applied
 *
 * Stores `ctl->w4` as the carrier loss time, sets the hdx `hold` flag from bit 4 of `ctl->flags` (the Data state then discards its input) and re-creates
 * the engine in place from its own config when bit 1 is set.
 */
int V21RX_control(V21Rx* o, const FaxV21Ctl* ctl)
{
  unsigned char f;
  if (ctl == NULL)
    return 0;
  o->cfg.w[2] = ctl->w4;
  f = ctl->flags;
  o->hdx->hold = (f & 0x10) != 0;
  if (f & 2)
    V21RX_create(o, (const int*)o);
  return 1;
}

/**
 * @brief Fill the status record of the receiver.
 * @param o   receiver
 * @param st  record to fill, or NULL (returns 0)
 * @return 1 when filled
 * @warning Divides by the FSD's second configuration half word (0x18 after V21RX_create()); a zero there is a division fault.
 */
int V21RX_status(V21Rx* o, FaxV21Status* st)
{
  V21RxBundle* b;
  int q, snr, s22, c4, chan, fl;
  if (st == NULL)
    return 0;
  b = o->bundle; /* everything is read before the first store through `st` (aliasing independent code) */
  chan = o->cfg.s[0];
  fl = o->st.b[1];
  s22 = b->fsd.s22;
  c4 = b->fsd.c4.s[1];
  snr = GetSNRV21(o);
  q = (2 * s22) / c4;
  st->s2 = 0;
  st->s4 = 0x12c;
  st->channel = (short)chan;
  st->s6 = (fl & 0x80) == 0;
  st->s8 = (short)snr;
  st->sa = 0;
  st->se = 0;
  st->s10 = 0;
  st->b15 &= 0xfe;
  st->b14 = 0;
  st->s12 = (short)((2 - q) * 300);
  return 1;
}

/* ===================================================================================================================== transmit side */
/** @brief FIFO_read() of the transmit queue (kept out of line so that the evaluation order of the callers does not depend on alias analysis). */
static NOINL int v21_fifo_read(V21TxHdx* h, short* dst, unsigned short n) { return FIFO_read(h->fifo, (unsigned short*)dst, n); }
/**
 * @brief Modulate `n` bits from `in` to samples: FSK modulator, then the matched filter.
 * @return number of samples written to `out`
 */
NOINL int ModDataV21(V21Tx* o, short* in, short* out, unsigned short n)
{
  V21TxBundle* b = o->bundle;
  unsigned int produced = (unsigned short)FPM_FSM_modulate(&b->fsm, (unsigned short*)in, b->buf, (short)n);
  b = o->bundle;
  return (int)(unsigned short)FPM_MRF_filter(&b->mrf, b->buf, out, (short)produced);
}

/** @brief Like ModDataV21() with the modulator's output scale forced to zero for this call (silence on the line); the scale is restored afterwards. */
NOINL int TxNoCarrierV21(V21Tx* o, short* in, short* out, unsigned short n)
{
  V21TxBundle* b = o->bundle;
  short scale = b->fsm.c.s[3];
  unsigned int produced;
  b->fsm.c.s[3] = 0;
  produced = (unsigned short)FPM_FSM_modulate(&b->fsm, (unsigned short*)in, b->buf, (short)n);
  b = o->bundle;
  b->fsm.c.s[3] = scale;
  return (int)(unsigned short)FPM_MRF_filter(&b->mrf, b->buf, out, (short)produced);
}

/**
 * @brief Advance the transmit state machine: Start -> Data -> Idle -> Start.
 *
 * Installs the next handler and rewrites the flag bytes of the state dword: entering Start or Data sets bit 0 of byte 1 and clears bit 0 of byte 2;
 * entering Idle does the opposite.  Any other state sets the error status (2) and the error flag.
 */
NOINL void TxNextStateV21(V21Tx* o)
{
  V21TxHdx* h = o->hdx;
  unsigned int w = (unsigned int)o->st.w; /* read once, before the stores (aliasing independent code) */
  switch (h->state)
  {
  case V21TX_STATE_START:
    if (DBG)
      dsplibs_debug_printf("V21TX_STATE_START\n");
    h->s0e = 0;
    h->fn = TxHdxDataV21;
    h->state = V21TX_STATE_DATA;
    /* byte 2 bit 0 cleared, byte 1 bit 0 set */
    o->st.w = (int)((w & ~0x10000u) | 0x100u);
    break;
  case V21TX_STATE_DATA:
    if (DBG)
      dsplibs_debug_printf("V21TX_STATE_DATA\n");
    h->fn = TxHdxIdleV21;
    h->state = V21TX_STATE_IDLE;
    /* byte 2 bit 0 set, byte 1 bit 0 cleared */
    o->st.w = (int)((w | 0x10000u) & ~0x100u);
    break;
  case V21TX_STATE_IDLE:
    if (DBG)
      dsplibs_debug_printf("V21TX_STATE_IDLE\n");
    h->s0e = 0;
    h->fn = TxHdxStartV21;
    h->state = V21TX_STATE_START;
    /* byte 2 bit 0 cleared, byte 1 bit 0 set */
    o->st.w = (int)((w & ~0x10000u) | 0x100u);
    break;
  default:
    if (DBG)
      dsplibs_debug_printf("V21TX_DEFAULT, %d\n", (int)h->state);
    /* status code 2 (error), byte 1 = (byte 1 | 2) & 0xfe, byte 2 bit 0 cleared */
    o->st.w = (int)((w & 0xff000000u) | (w & 0x00fe0000u) | ((((w >> 8) | 2u) & 0xfeu) << 8) | 2u);
    break;
  }
}

/** @brief Start state: send whatever the FIFO holds (up to the input count), advance to Data and report "CONNECT". @return samples produced */
int TxHdxStartV21(V21Tx* o, short* in, short* out, unsigned short* n)
{
  unsigned short c, cin = *n;
  unsigned short produced;
  c = (unsigned short)v21_fifo_read(o->hdx, in, cin);
  *n = (unsigned short)(cin - c);
  produced = (unsigned short)ModDataV21(o, in, out, c);
  TxNextStateV21(o);
  o->st.b[0] = V21TX_MSG_CONNECT;
  return (short)produced;
}

/** @brief Idle state: while the FIFO is empty send silence (TxNoCarrierV21) and report "idle"; when data arrives advance to Start. @return samples produced */
int TxHdxIdleV21(V21Tx* o, short* in, short* out, unsigned short* n)
{
  if (o->hdx->fifo->count == 0)
  {
    unsigned short cnt = *n;
    int r = (short)TxNoCarrierV21(o, in, out, cnt);
    *n = (unsigned short)(*n - cnt);
    o->st.b[0] = V21TX_MSG_IDLE;
    return r;
  }
  TxNextStateV21(o);
  return 0;
}

/**
 * @brief Data state: modulate the bits the FIFO provides.
 * @return samples produced
 *
 * When the FIFO holds fewer bits than requested: with `pend` set the machine advances to the next state after sending them; otherwise the block is
 * padded to the full count (the modulator is run for all `*n` bits), the error flag is set and the status becomes "under-run".
 */
int TxHdxDataV21(V21Tx* o, short* in, short* out, unsigned short* n)
{
  unsigned short c, dx;
  unsigned short produced;
  c = (unsigned short)v21_fifo_read(o->hdx, in, *n);
  dx = *n;
  if (dx > c)
  {
    if (o->hdx->pend != 0)
    {
      *n = (unsigned short)(dx - c);
      produced = (unsigned short)ModDataV21(o, in, out, c);
      TxNextStateV21(o);
      return (short)produced;
    }
    o->st.b[1] |= 2;
    c = dx;
    o->st.b[0] = V21TX_MSG_UNDERRUN;
  }
  {
    int r = (short)ModDataV21(o, in, out, c);
    *n = (unsigned short)(*n - c);
    o->st.b[0] = V21TX_MSG_DATA;
    return r;
  }
}

/**
 * @brief Process one block of transmit bits through the state machine.
 * @param o    transmitter
 * @param in   input bits
 * @param out  receives the output samples
 * @param n    in: number of bits; out: number of samples produced
 * @return the state dword (status code in the low byte, flag bytes above it)
 *
 * Unless `pend` is set the bits are first written to the 6-entry FIFO; bits that do not fit set the error flag and the status "over-run".  The handlers
 * then run in 6-bit steps until the FIFO is used up.
 */
int V21TX_modem(V21Tx* o, short* in, short* out, unsigned short* n)
{
  unsigned short cin = *n, consumed,
                 local = 6; /* *n, the hdx pointer, its pend flag and the FIFO are read before the first store (aliasing independent code) */
  V21TxHdx* h = o->hdx;
  int pend = h->pend;
  FaxFifo* fifo = h->fifo;
  int total = 0;
  o->st.b[1] &= 0xfd;
  if (pend == 0)
  {
    consumed = (unsigned short)FIFO_write(fifo, (const unsigned short*)in, cin);
  }
  else
  {
    consumed = cin;
  }
  for (;;)
  {
    int ret = (short)o->hdx->fn(o, in, out, &local);
    total = (short)(total + ret);
    out += ret;
    if (!((short)local > 0))
      break;
  }
  if (cin != consumed)
  {
    o->st.b[1] |= 2;
    o->st.b[0] = V21TX_MSG_OVERRUN;
  }
  *n = (unsigned short)total;
  return o->st.w;
}

/**
 * @brief Apply a control record to the transmitter.
 * @param o    transmitter
 * @param ctl  control record, or NULL (returns 0)
 * @return 1 when applied
 *
 * Sets the modulator output scale from the low 16 bits of `ctl->txScale`, the carrier loss time from `ctl->w4`, bit 2 of config byte 0x10 from bit 2 of
 * `ctl->b0c`, the hdx `pend` flag from bit 4 of `ctl->flags`, and re-creates the engine in place from its own config when bit 1 is set.
 */
int V21TX_control(V21Tx* o, const FaxV21Ctl* ctl)
{
  unsigned char f;
  if (ctl == NULL)
    return 0;
  o->bundle->fsm.c.s[3] = (short)ctl->txScale;
  o->cfg.w[2] = ctl->w4;
  if (ctl->b0c & 4)
    o->cfg.b[0x10] |= 4;
  f = ctl->flags;
  o->hdx->pend = (f & 0x10) != 0;
  if (f & 2)
    V21TX_create(o, (const int*)o);
  return 1;
}

/** @brief Fill the status record of the transmitter (channel, 300 baud, config bit 2 in `b14`). @return 1, or 0 when `st` is NULL */
int V21TX_status(V21Tx* o, FaxV21Status* st)
{
  unsigned char b;
  short chan;
  if (st == NULL)
    return 0;
  b = (unsigned char)(o->cfg.b[0x10] & 4); /* read before the stores through `st` (aliasing independent code) */
  chan = o->cfg.s[0];
  st->s2 = 0x12c;
  st->s4 = 0;
  st->s6 = 0;
  st->channel = chan;
  st->s8 = 0;
  st->sa = 0;
  st->sc = 0;
  st->s10 = 0;
  st->s12 = 0;
  st->b15 &= 0xfe;
  st->b14 = b;
  return 1;
}

/* ===================================================================================================================== VMI wrappers */
/**
 * @brief VMI wrapper: create or re-initialise the transmitter held in `m->eng` from a copy of `cfg` (or V21TX_CFG when NULL).
 * @return the engine handle; also sets the VMI object's size / mode shorts for the transmit direction
 */
void* v21tx_create(FaxVmiV21* m, const int* cfg)
{
  int w[7];
  int i;
  if (cfg != NULL)
  {
    for (i = 0; i < 7; i++)
      w[i] = cfg[i];
  }
  else
  {
    for (i = 0; i < 7; i++)
      w[i] = V21TX_CFG[i];
  }
  m->eng = V21TX_create((V21Tx*)m->eng, w);
  m->s0c = 6;
  m->s0e = 1;
  m->u10.s10 = 0;
  return m->eng;
}

/** @brief VMI wrapper: create or re-initialise the receiver held in `m->eng` from a copy of `cfg` (or V21RX_CFG when NULL). @return the engine handle */
void* v21rx_create(FaxVmiV21* m, const int* cfg)
{
  int w[6];
  int i;
  if (cfg != NULL)
  {
    for (i = 0; i < 6; i++)
      w[i] = cfg[i];
  }
  else
  {
    for (i = 0; i < 6; i++)
      w[i] = V21RX_CFG[i];
  }
  m->eng = V21RX_create((V21Rx*)m->eng, w);
  m->s0c = 0;
  m->u10.s10 = 1;
  m->s0e = 0;
  return m->eng;
}

/** @brief VMI wrappers: delete the engine in `m->eng`. */
void v21tx_delete(FaxVmiV21* m) { V21TX_delete((V21Tx*)m->eng); }
void v21rx_delete(FaxVmiV21* m) { V21RX_delete((V21Rx*)m->eng); }

/** @brief VMI wrapper: modulate the bits of the VMI bit buffer into `out`; `*n` in = bit count, `*outn` = samples produced, `*n` is cleared. */
int v21tx_process(FaxVmiV21* m, short* out, unsigned short* n, unsigned short* outn)
{
  int r = V21TX_modem((V21Tx*)m->eng, m->u4.bits, out, n);
  *outn = *n;
  *n = 0;
  return r;
}

/** @brief VMI wrapper: demodulate `in` (`*n` samples) into the VMI bit buffer; `*outn` = bits produced, `*n` is cleared. */
int v21rx_process(FaxVmiV21* m, short* in, unsigned short* outn, unsigned short* n)
{
  int r = V21RX_modem((V21Rx*)m->eng, in, m->u4.bits, n);
  *outn = *n;
  *n = 0;
  return r;
}

/** @brief VMI wrappers: status and control of the engine in `m->eng`. */
int v21tx_status(FaxVmiV21* m, FaxV21Status* st) { return V21TX_status((V21Tx*)m->eng, st); }
int v21rx_status(FaxVmiV21* m, FaxV21Status* st) { return V21RX_status((V21Rx*)m->eng, st); }
int v21tx_control(FaxVmiV21* m, const FaxV21Ctl* ctl) { return V21TX_control((V21Tx*)m->eng, ctl); }
int v21rx_control(FaxVmiV21* m, const FaxV21Ctl* ctl) { return V21RX_control((V21Rx*)m->eng, ctl); }

/** @brief VMI wrapper: the status text for status code `idx` (V21TxStatus, 0 .. 5) in `*out`; NULL for any other index. */
void v21tx_message(FaxVmiV21* m, int idx, const char** out)
{
  (void)m;
  *out = (unsigned int)idx <= 5u ? V21TX_MESG[(unsigned char)idx] : NULL;
}

/** @brief VMI wrapper: the status text for status code `idx` (V21RxStatus, 0 .. 6) in `*out`; NULL for any other index. */
void v21rx_message(FaxVmiV21* m, int idx, const char** out)
{
  (void)m;
  *out = (unsigned int)idx <= 6u ? V21RX_MESG[(unsigned char)idx] : NULL;
}
