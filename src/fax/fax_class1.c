/**
 * @file fax_class1.c
 * @brief The fax class 1 layer: creation, command, per-block progress and deletion of the T.30 object (FaxT30).
 *
 * fax_class1_create() builds the object with its two V.21 HDLC channels (FaxVmi objects for the T.30 control frames) and starts it in the answer or
 * originate role; fax_class1_command() applies the class 1 commands (TH, TM, RM, RH, TS, RS) that fax_api.c derives from AT+F*; fax_class1_progress() is
 * called once per 160-sample block and runs the current state function; fax_class1_delete() releases everything.  The state functions themselves
 * (transmit and receive of HDLC frames and of the high-speed data modems) are in fax_t30_tx.c and fax_t30_rx.c; the object is declared in fax_t30.h,
 * the glue object and entry points are in fax_api.c.
 *
 * - fax_class1_create(NULL, cfg) allocates the FaxT30 without clearing it (only the members it sets are written) and builds the HDLC channels: a
 *   FaxVmiCfg block, a copy of the V.21 channel config (V21TX_CFG / V21RX_CFG) and FAXVMI_create().  Transmit channel: HDLC mode, bit-reversal flag 1,
 *   ring minimum 0x60, 0x34 items per call, id FAXVMI_ID_V21TX; receive channel: ring 0, 0x60 items, 0x60 frame words, id FAXVMI_ID_V21RX.  Answer role
 *   (cfg.mode == FAXC1_ROLE_ANSWER): state "answer tone" with a 2100 Hz tone generator; originate role: state "HDLC receive look for carrier"
 *   (through _cHDLCrx_init_from_idle) and a 1100 Hz calling tone (sent when cfg.cngOff is 0).  With a non-NULL first argument only the common part is
 *   done (no channels are created, no members are cleared).
 * - fax_class1_state_functions[] is one global table filled by every fax_class1_create(); the state number in FaxT30.state indexes it.
 * - fax_class1_progress(): optional input activity watch (rx_act), the clock (2 ms per call), the optional input high-pass filter, the call of the
 *   state function (with a copy of the input count), the state-change message, the delayed status, the accept-rate check and the status-name message.
 *   The returned value is the class 1 status (FAX_CLASS1_*).
 * - fax_class1_command(): `extra` (FAX_class1_command passes 0x50 for TM) becomes the silence time; every command except RH clears sf_fill; an unknown
 *   command is ignored.
 * - fax_class1_delete() frees the two HDLC config blocks (the V.21 config copies and the FaxVmiCfg blocks), the channels, the data modem (receiver or
 *   transmitter according to `chan`), the FIFO and the tone generator, then the object.
 *
 * The debug messages (state changes, status names, commands) print at dsplibs_debug_level above 1.
 */
#include <string.h>
#include "fax_class1.h"
#include "fax_common_tables.h"
#include "fax_v21_tables.h"
#include "fax_v27_rx.h"
#include "fax_v29_rx.h"
#include "fax_v17_rx.h"
#include "fpm/fpm_tables.h"
#include "core/dsp_asm.h"

#include "fax_debug.h" /* dsplibs_debug_level, DBG, DBG3 */
#include "core/dsp_host.h"
#include "fpm/fpm.h"

/* the order of the loads / stores around it is the same with and without -fno-strict-aliasing (compared on the objects built with and without the flag) */

/** The state functions indexed by FaxT30.state (see fax_t30.h); one table shared by all class 1 objects, filled by fax_class1_create(). */
FaxT30StateFn fax_class1_state_functions[19];

/* the accept-rate check in fax_class1_progress() reaches the receivers' qualityOk flags through typed members; these asserts pin the 32-bit layout */
#if UINTPTR_MAX == 0xffffffffu
_Static_assert(offsetof(V27Rx, bundle) == 0x54 && offsetof(V29Rx, bundle) == 0x50 && offsetof(V17Rx, bundle) == 0x60, "the receivers' bundle pointers");
_Static_assert(offsetof(V27RxBundle, qualityOk) == 0x4f4e && offsetof(V29RxBundle, qualityOk) == 0x4f62 && offsetof(V17RxBundle, qualityOk) == 0x4fb2,
               "the receivers' qualityOk flags");
#endif

/** @brief The name of `key` in a name table of `n` entries; the LAST entry with that key wins; `dflt` when there is none. */
static const char* name_of(const FaxClass1Name* tab, int n, int key, const char* dflt)
{
  const char* r = dflt;
  int i;
  for (i = 0; i < n; i++)
    if (tab[i].key == key)
      r = tab[i].name;
  return r;
}

/* ===================================================================================================================== fax_class1_create */
/**
 * @brief Fill the FaxVmiCfg of a V.21 HDLC channel: the FAXVMI_CFG defaults with HDLC mode, bit-reversal flag 1 and the given ring / item / frame sizes.
 * @param c     block to fill
 * @param s08   ring minimum size
 * @param s0a   maximum items per call
 * @param s0c   HDLC frame buffer words
 * @param id    modem id (FAXVMI_ID_V21TX / FAXVMI_ID_V21RX)
 * @param mcfg  the V.21 config copy passed to the modem's create
 */
static void hdlc_cfg(FaxVmiCfg* c, unsigned short s08, unsigned short s0a, unsigned short s0c, unsigned short id, const void* mcfg)
{
  FaxVmiCfg base = FAXVMI_CFG;
  *c = base;
  c->mode = (int)(((unsigned int)base.mode & 0xffff0000u) | FAXVMI_MODE_HDLC);
  c->flag = 1;
  c->ringMin = s08;
  c->maxItems = s0a;
  c->frmWords = s0c;
  c->id = id;
  c->mcfg = mcfg;
}

/**
 * @brief Create the tone generator `o->tone` from the default tone config with word 0 = 0x19000000 | `freq`.
 * @param freq  0x834 for the 2100 Hz answer tone, 0x44c for the 1100 Hz calling tone
 */
static void create_tone(FaxT30* o, unsigned int freq)
{
  int cfg[9];
  int i;
  for (i = 0; i < 9; i++)
    cfg[i] = FPM_TONE_CFG_raw[i];
  cfg[0] = (int)(0x19000000u | freq);
  cfg[1] = (int)((unsigned int)cfg[1] & 0xffff0000u);
  o->tone = FPM_TONE_create(NULL, cfg);
}

/**
 * @brief Create the class 1 object and start it in the answer or originate role.
 * @param o       NULL to allocate a new object with its HDLC channels; a non-NULL object gets only the common initialisation
 * @param cfg_in  configuration (FaxClass1Cfg); the words are read once, up front
 * @return the object, or NULL when an allocation failed (everything allocated so far is released)
 *
 * The common part sets the high-pass filter state, S7, the answer-tone length (cfg.answerMs / 20 frames, default 150), the clock and flags, and fills
 * fax_class1_state_functions[].  Answer role: state "answer tone" with a 2100 Hz tone generator.  Originate role: state "HDLC receive look for
 * carrier" through _cHDLCrx_init_from_idle(o, 3), with a 1100 Hz calling tone generator (cng_on = cfg.cngOff == 0).
 * @warning A new object is not cleared: members this function does not set keep whatever the heap held.
 * @warning The results of the two FAXVMI_create() calls are not checked; a NULL channel makes the first FAXVMI_process() on it fault (fax_class1_delete()
 *          skips a NULL channel).
 */
FaxT30* fax_class1_create(FaxT30* o, const FaxClass1Cfg* cfg_in)
{
  const FaxClass1Cfg c = *cfg_in; /* (the config words are read once, up front: cached locals keep the result independent of -fno-strict-aliasing) */
  const FaxClass1Cfg* cfg = &c;
  if (DBG)
    dsplibs_debug_printf(" Fax Class1 Create - Fixed-Point Version %s %s\n", "Sep 22 2005", "15:48:16");
  if (o == NULL)
  {
    FaxVmiCfg *tc, *rc;
    int *vtx, *vrx;
    o = (FaxT30*)sysdep_malloc((unsigned int)sizeof(FaxT30));
    if (o == NULL)
      return NULL;
    o->hdlc_tx = NULL;
    o->hdlc_tx_cfg = NULL;
    o->hdlc_rx_cfg = NULL;
    o->data_cfg = NULL;
    o->data_vmi = NULL;
    o->hdlc_rx = NULL;
    o->tone = NULL;
    o->fifo = NULL;
    /* the transmit HDLC channel */
    tc = (FaxVmiCfg*)sysdep_malloc((unsigned int)sizeof(FaxVmiCfg));
    o->hdlc_tx_cfg = tc;
    vtx = (int*)sysdep_malloc(sizeof(V21TX_CFG));
    if (tc == NULL || vtx == NULL)
    {
      /* allocation failure: free what exists and fail */
      if (vtx != NULL)
        sysdep_free(vtx);
      if (tc != NULL)
        sysdep_free(tc);
      sysdep_free(o);
      return NULL;
    }
    memcpy(vtx, V21TX_CFG, sizeof(V21TX_CFG));
    hdlc_cfg(tc, 0x60, 0x34, 0, FAXVMI_ID_V21TX, vtx);
    o->hdlc_tx = FAXVMI_create(NULL, tc);

    /* the receive HDLC channel */
    rc = (FaxVmiCfg*)sysdep_malloc((unsigned int)sizeof(FaxVmiCfg));
    o->hdlc_rx_cfg = rc;
    vrx = (int*)sysdep_malloc(sizeof(V21RX_CFG));
    if (rc == NULL || vrx == NULL)
    {
      if (vrx != NULL)
        sysdep_free(vrx);
      if (rc != NULL)
        sysdep_free(rc);
      o->hdlc_rx_cfg = NULL;
      fax_class1_delete(o);
      return NULL;
    }
    memcpy(vrx, V21RX_CFG, sizeof(V21RX_CFG));
    hdlc_cfg(rc, 0, 0x60, 0x60, FAXVMI_ID_V21RX, vrx);
    o->hdlc_rx = FAXVMI_create(NULL, rc);
    o->emu_idx = 0;
    o->sf_fill = 0;
  }

  /* common initialisation: input filter, S7, answer tone length */
  o->hpCoef = fax_hp_coeff;
  o->s7 = cfg->s7;
  o->gain_att = 0;
  o->cd_flag = cfg->cdFlag;
  sysdep_memset(o->hpState, 0, 16);
  o->ans_len = 0x96;
  o->hp_on = (cfg->hpFilter != 0);
  if (cfg->answerMs != 0)
  {
    o->ans_len = cfg->answerMs / 20; /* ms -> 20 ms frames */
    if (DBG)
      dsplibs_debug_printf("Answer tone length %d ms", cfg->answerMs);
  }

  /* the state table, indexed by FaxT30.state */
  fax_class1_state_functions[0] = _t30_silence_before_tx_state;
  fax_class1_state_functions[1] = _t30_preabmle_state;
  fax_class1_state_functions[2] = _send_hdlc_buffer_state;
  fax_class1_state_functions[3] = _send_hdlc_between_buffer_state;
  fax_class1_state_functions[4] = _hdlc_receive_look_carrier_state;
  fax_class1_state_functions[5] = _hdlc_receive_state;
  fax_class1_state_functions[6] = _hdlc_receive_between_buffers_state;
  fax_class1_state_functions[7] = _hdlc_emulate_receive_state;
  fax_class1_state_functions[8] = fax_idle_state;
  fax_class1_state_functions[9] = _tx_scrambled_ones_state;
  fax_class1_state_functions[10] = _tx_data_state;
  fax_class1_state_functions[11] = _tx_nulls_state;
  fax_class1_state_functions[12] = _rx_look_carrier_state;
  fax_class1_state_functions[13] = _rx_data_state;
  fax_class1_state_functions[14] = fax_answer_tone_state;
  fax_class1_state_functions[15] = fax_send_silence_state;
  fax_class1_state_functions[16] = fax_recieve_silence_state;
  fax_class1_state_functions[17] = cHDLCtx_off;
  fax_class1_state_functions[18] = _tx_silence_before_scrm_ones;

  /* clock and flags */
  o->ms = 0;
  o->sec = 0;
  o->rx_act = 1;
  o->dlePending = 0;
  o->cng_on = 0;
  o->modem_sel = 3;
  o->rx_kind = 0;

  /* answer role: start with the answer tone */
  if (cfg->mode == FAXC1_ROLE_ANSWER)
  {
    o->state = FAXC1_ST_ANSWER_TONE_STATE;
    o->state_prev = FAXC1_ST_ANSWER_TONE_STATE;
    create_tone(o, 0x834);
    o->tmr = 0;
    o->ans_org = FAXC1_ROLE_ANSWER;
    return o;
  }

  /* originate role: look for the carrier of the answering side, sending the calling tone meanwhile */
  o->state = FAXC1_ST_HDLC_RECEIVE_LOOK_CARRIER_STATE;
  o->state_prev = FAXC1_ST_HDLC_RECEIVE_LOOK_CARRIER_STATE;
  o->cng_on = (cfg->cngOff == 0);
  if (o->cng_on == 0 && DBG)
    dsplibs_debug_printf("CNG generation disabled");
  create_tone(o, 0x44c);
  o->tmr = 0;
  o->cng_tone = 1;
  o->cng_ctr = 0;
  _cHDLCrx_init_from_idle(o, 3);
  o->ans_org = FAXC1_ROLE_ORIGINATE;
  return o;
}

/* ===================================================================================================================== fax_class1_command */
/**
 * @brief Apply a class 1 command.
 * @param o      class 1 object
 * @param cmd    FAXC1_TH (send HDLC), FAXC1_TM (send data modem), FAXC1_RM (receive data modem), FAXC1_RH (receive HDLC), FAXC1_TS (transmit silence),
 *               FAXC1_RS (receive silence); other values are ignored
 * @param param  modulation selection (FAXMOD_*) for TH / TM / RM / RH, silence time in 10 ms units (stored as `param / 2` frames, at least 1) for TS / RS
 * @param extra  stored as the silence time for TM
 * @return always 1
 *
 * Every command except RH clears sf_fill.  TM and RM delete a data modem of the opposite direction first.  RH continues the current HDLC reception when
 * it is already between buffers on the same modulation, restarts it from idle otherwise, or (when sf_fill is set) goes to the HDLC emulation state with a
 * delay of 2.
 */
int fax_class1_command(FaxT30* o, int cmd, int param, int extra)
{
  const char* name = "";
  if (DBG)
    dsplibs_debug_printf("Fax Class1 Command %X\n", (unsigned int)cmd);
  if (cmd <= 5)
  {
    name = name_of(fax_class1_cmd_names, 6, cmd, "");
    if (DBG)
      dsplibs_debug_printf("At %2d.%02d[sec] %s %d\n", o->sec, o->ms, name, param);
  }
  if (cmd != 3)
    o->sf_fill = 0;

  switch ((unsigned int)cmd)
  {
  case FAXC1_TH:
    o->modem_sel = param;
    cHDLCtx_preamble_state_init(o);
    break;
  case FAXC1_TM:
    if (o->data_cfg != NULL && o->data_vmi != NULL && o->chan == FAXC1_CHAN_RX)
      _delete_data_rx_modem(o);
    o->modem_sel = param;
    o->silenceTime = extra;
    _tx_scrambled_ones_init(o, param);
    o->chan = FAXC1_CHAN_TX;
    break;
  case FAXC1_RM:
    if (o->data_cfg != NULL && o->data_vmi != NULL && o->chan == FAXC1_CHAN_TX)
      _delete_data_tx_modem(o);
    o->modem_sel = param;
    _rx_look_carrier_init(o, param);
    _cHDLCrx_init_from_idle(o, param);
    o->chan = FAXC1_CHAN_RX;
    break;
  case FAXC1_RH:
    if (o->state == FAXC1_ST_HDLC_RECEIVE_BETWEEN_BUFFERS_STATE && o->modem_sel == param)
    {
      _hdlc_receive_state_init(o);
      o->state = FAXC1_ST_HDLC_RECEIVE_STATE;
      o->modem_sel = param;
      break;
    }
    if (o->sf_fill == 0)
    {
      _cHDLCrx_init_from_idle(o, param);
      o->modem_sel = param;
    }
    else
    {
      o->state = FAXC1_ST_HDLC_EMULATE_RECEIVE_STATE;
      o->emu_delay = 2;
      o->modem_sel = param;
    }
    break;
  case FAXC1_TS:
  case FAXC1_RS:
  {
    int t;
    if (cmd == FAXC1_TS)
    {
      o->state = FAXC1_ST_SEND_SILENCE_STATE;
    }
    else
    {
      o->state = FAXC1_ST_RECIEVE_SILENCE_STATE;
      o->silenceTime = 0;
      o->silence_rms &= (int)0xffff0000u;
    }
    t = param / 2;
    if (t == 0)
      t = 1;
    o->tmr = t;
    break;
  }
  default:
    break;
  }
  return 1;
}

/* ===================================================================================================================== fax_class1_progress */
/**
 * @brief Run the class 1 object for one 160-sample block.
 * @param o        class 1 object
 * @param rx       received samples (the input high-pass filter works in place)
 * @param tx       samples to transmit (written by the transmit states)
 * @param tty_out  bytes for the DTE (written by the receive states)
 * @param tty_in   bytes from the DTE
 * @param n_in     number of input samples (passed to the state function by value; not modified)
 * @param n_out    receives the number of output samples (160 expected)
 * @param p7       receives the number of bytes written to `tty_out`
 * @param p8       in / out: the DTE byte count (the number of bytes available in `tty_in`, updated by the state)
 * @return the class 1 status (FAX_CLASS1_*): the status of the current state, the delayed status once its countdown ends, or ACCEPT_RATE when the data
 *         receiver signals that the rate is acceptable
 *
 * Advances the clock (`ms` += 2 per call, carried into `sec` at 100; used by the debug messages), filters the input, calls the state function and
 * prints state changes and status names (debug level above 1).
 * @note  A state number outside 0 .. 18 skips the state function (no-op).
 */
int fax_class1_progress(FaxT30* o, short* rx, short* tx, unsigned char* tty_out, const unsigned char* tty_in, int* n_in, int* n_out, int* p7, int* p8)
{
  int n = *n_in;
  int st, prev, status;

  /* input activity watch: prints the first n / 8 samples and counts the non-zero ones; the watch ends once more than n / 16 were non-zero */
  if (o->rx_act != 0)
  {
    int i, lim = (n < 0 ? n + 7 : n) >> 3, lim2;
    if (DBG)
      dsplibs_debug_printf("At %2d.%02d[sec] Input Buffer: ", o->sec, o->ms);
    for (i = 0; i < lim; i++)
    {
      if (DBG)
        dsplibs_debug_printf("%d,", (int)rx[i]);
      if (rx[i] != 0)
        o->rx_act++;
    }
    if (DBG)
      dsplibs_debug_printf("\n");
    lim2 = (n < 0 ? n + 15 : n) >> 4;
    if (o->rx_act > lim2)
      o->rx_act = 0;
  }

  /* the clock */
  if (o->ms + 2 > 0x63)
  {
    o->sec++;
    o->ms = 0;
  }
  else
  {
    o->ms = o->ms + 2;
  }
  *p7 = 0;
  o->status = FAX_CLASS1_NO_MESSAGE;

  /* input high-pass filter, then the state function */
  if (o->hp_on != 0)
    FPM_iir_filt_II(rx, (short*)o->hpCoef, (short*)o->hpState, 2, (short)n);
  {
    int nloc = n;
    FaxT30StateFn fn = o->state >= 0 && o->state < 19 ? fax_class1_state_functions[o->state] : NULL;
    COMPILER_BARRIER();
    if (fn != NULL)
      fn(o, rx, tx, tty_out, tty_in, &nloc, n_out, p7, p8);
  }
  /* state-change message */
  st = o->state;
  prev = o->state_prev;
  if (st != prev)
  {
    if (DBG)
    {
      const char* newn = name_of(fax_class1_state_names, 20, st, "");
      const char* prevn = name_of(fax_class1_state_names, 20, prev, newn);
      dsplibs_debug_printf("At %2d.%02d[sec], FAX_CLASS1_STATE: %s ===> %s\n", o->sec, o->ms, prevn, newn);
      st = o->state;
    }
  }
  o->state_prev = st;
  if (*n_out != 0xa0 && DBG)
    dsplibs_debug_printf("ERROR: TxSmpCnt != 160 !!!\n");

  /* delayed status: rx_kind counts blocks down, then rx_event becomes the status */
  if (o->rx_kind != 0)
  {
    int d = o->rx_kind - 1;
    o->rx_kind = d;
    if (d == 0)
    {
      int dbg = DBG;
      o->status = o->rx_event;
      if (dbg)
        dsplibs_debug_printf("STATUS = DELAYED_STATUS\n");
    }
  }
  status = o->status;

  /* no status yet: while receiving data, turn the receiver's quality-OK flag into ACCEPT_RATE (and clear the flag) */
  if (status == FAX_CLASS1_NO_MESSAGE)
  {
    int sel = o->modem_sel;
    short* flag = NULL;
    if (o->chan == FAXC1_CHAN_RX)
    {
      FaxVmiM* vm = o->data_vmi != NULL ? o->data_vmi->m : NULL;
      void* eng = vm != NULL ? vm->eng : NULL;
      if (eng != NULL)
      {
        if (sel == FAXMOD_V17_14400 || sel == FAXMOD_V17_12000 || sel == FAXMOD_V17_9600 || sel == FAXMOD_V17_7200)
          flag = &((V17Rx*)eng)->bundle->qualityOk;
        else if (sel == FAXMOD_V29_9600 || sel == FAXMOD_V29_7200)
          flag = &((V29Rx*)eng)->bundle->qualityOk;
        else if (sel == FAXMOD_V27_4800 || sel == FAXMOD_V27_2400)
          flag = &((V27Rx*)eng)->bundle->qualityOk;
      }
    }
    if (flag == NULL || *flag == 0)
      return 0;
    *flag = 0;
    o->status = FAX_CLASS1_ACCEPT_RATE;
    status = FAX_CLASS1_ACCEPT_RATE;
  }
  /* status-name message */
  {
    const char* name = name_of(fax_class1_status_names, 11, status, "");
    if (DBG)
    {
      dsplibs_debug_printf("At %2d.%02d[sec] New STATUS message: %s \n", o->sec, o->ms, name);
      return o->status;
    }
  }
  return status;
}

/* ===================================================================================================================== fax_class1_delete */
/**
 * @brief Release a class 1 object and everything it owns: the HDLC config blocks and channels, the data modem, the FIFO, the tone generator.
 * @param o  the object; must not be NULL
 * @return always 1
 */
int fax_class1_delete(FaxT30* o)
{
  if (o->hdlc_tx_cfg != NULL)
  {
    sysdep_free((void*)(uintptr_t)o->hdlc_tx_cfg->mcfg);
    sysdep_free(o->hdlc_tx_cfg);
  }
  if (o->hdlc_tx != NULL)
    FAXVMI_delete(o->hdlc_tx);
  if (o->hdlc_rx_cfg != NULL)
  {
    sysdep_free((void*)(uintptr_t)o->hdlc_rx_cfg->mcfg);
    sysdep_free(o->hdlc_rx_cfg);
  }
  if (o->hdlc_rx != NULL)
    FAXVMI_delete(o->hdlc_rx);
  if (o->data_cfg != NULL && o->data_vmi != NULL)
  {
    if (o->chan == FAXC1_CHAN_RX)
      _delete_data_rx_modem(o);
    else
      _delete_data_tx_modem(o);
  }
  if (o->fifo != NULL)
    FIFO_delete(o->fifo);
  if (o->tone != NULL)
    FPM_TONE_delete(o->tone);
  sysdep_free(o);
  return 1;
}
