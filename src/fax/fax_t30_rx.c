/**
 * @file fax_t30_rx.c
 * @brief The T.30 / HDLC receive side of the fax class 1 object: the receive state machines and their initialisers, the data modem receiver setup, and the DTE
 *        input converters.
 *
 * - Receive states (called by fax_class1_progress() through fax_class1_state_functions[]): _hdlc_receive_look_carrier_state (carrier search on the V.21
 *   channel, with calling tone), _hdlc_receive_state (frame reception), _hdlc_receive_between_buffers_state (command-mode buffering into the superframe),
 *   _hdlc_emulate_receive_state (hand the buffered frames to the DTE one by one), _rx_look_carrier_state and _rx_data_state (data modem), and
 *   fax_recieve_silence_state (wait for silence).
 * - Initialisers: _hdlc_receive_state_init, _cHDLCrx_init_from_idle, _rx_look_carrier_init, _init_receiver (create or restart the V.27ter / V.29 / V.17 receive
 *   channel for a modem selection), _delete_data_rx_modem.  _recieve_silence_state_init and _handle_hdlc_input_close have no caller in this library.
 * - DTE input converters used by the transmit states of fax_t30_tx.c: _handle_data_input (DLE un-stuffing into words, DLE ETX ends the stream),
 *   _handle_hdlc_input (collects one HDLC frame, DLE ETX ends it), _handle_hdlc_input_open.
 * The object type, the state-function signature and the calling convention are in fax_t30.h; every state function returns 0 and reports through o->status.
 * The receive states pass `o->buf` (the object itself) as the buffer of FAXVMI_process. The HDLC channel (o->hdlc_rx, FAXVMI mode 2) delivers one frame per
 * call as [len][bytes] words in buf[0 ..]: buf[0] = 0 means a frame with a bad CRC. The data channel (o->data_vmi) delivers up to *p7 words that
 * _handle_data_output turns into DLE-stuffed bytes for the DTE (tty_out). `sec` / `ms` of the object are the clock the debug lines print; every debug line
 * needs dsplibs_debug_level > 1 (the one in _handle_data_input needs > 2).
 *
 * Notes on behaviour:
 * - The superframe (command mode: the DTE did not collect the frame yet) lives in buf[0x800 .. 0x8ff] as [len][bytes] records, sf_fill words used.
 *   _hdlc_emulate_receive_state keeps the frame lengths in a local array of 256 entries, which the 0xff-word superframe cannot overflow.
 *   _hdlc_receive_between_buffers_state copies with a 16-bit index; the "full" test keeps the length small enough for that.
 * - The receive timeout of _rx_look_carrier_state / _hdlc_receive_look_carrier_state is S7 seconds in blocks: s7 * 8000 / *n_in (n_in == 0: s7 * 50); the
 *   division traps for *n_in == -1 with a product of 0x80000000.
 * - _hdlc_receive_look_carrier_state sends the calling tone while cng_on: 0.5 s tone (0xfa0 samples) / 3 s silence (0x5dc0), starting with silence; when a
 *   carrier is found it asks for a gain attenuation of 12 - 3 * k dB for the first k with (short)agc.hist < fax_gain_threshold_table[k] (gain_att), and
 *   enters state 5.
 * - _hdlc_receive_state copies the V.21 AGC (rcp, shift) of a good frame into agc_rcp / agc_shift; _init_receiver hands them to the AGC of the data receiver it
 *   creates (V.27ter, V.29 or V.17 bundle->agc) and stores cd_flag in the data receiver's carrier flag (hdx s14 / s1c / s20).
 * - _init_receiver restarts an existing receiver of the same class by a control record chosen by the VMI config's modem id (0xc V.17, 0xa V.29, else V.27ter;
 *   V.17 gets the short-train flag in word 4, the rate in the HIGH half of word 0 for all three, word 3 |= 0x200); a changed class deletes the old one first.
 *   A modem value outside the twelve known ones leaves the class and the rate at 0. A data_cfg without a data_vmi, a NULL data_vmi after FAXVMI_create, or a
 *   NULL config block (allocation failure) makes the function return without selecting a state.
 * - fax_recieve_silence_state: the energy (FPM_rms of the first *n_in words, a short) is stored in the low half of silence_rms; a pending DTE byte (*p8 != 0)
 *   aborts with status = FAX_CLASS1_OK_NO_CARRIER; energy > 100 restarts the silence time (silenceTime = 0, status = FAX_CLASS1_NO_MESSAGE, *p8 = 5),
 *   otherwise silenceTime is incremented until it reaches tmr (unsigned).
 * - _handle_data_input keeps a write-only counter of the bytes it saw (fax_handle_data_input_count).
 */
#include <string.h>
#include "fax_t30.h"
#include "fax_common_tables.h"
#include "fax_v21.h"
#include "fax_v21_tables.h"
#include "fax_v27_rx.h"
#include "fax_v27_rx_tables.h"
#include "fax_v29_rx.h"
#include "fax_v29_rx_tables.h"
#include "fax_v17_rx.h"
#include "fax_v17_rx_tables.h"
#include "core/dsp_asm.h"

#include "fax_debug.h"          /* dsplibs_debug_level, DBG, DBG3 */
#include "core/dsp_constants.h" /* RATE_* */
#include "core/dsp_host.h"
#include "fpm/fpm.h"

/* The order of the loads / stores in the functions below is the same with and without -fno-strict-aliasing (the objects built with and without the flag were compared). */

int fax_handle_data_input_count;

/**
 * @brief Build the FaxVmiCfg for FAXVMI_create() from the VMI block an init_vmi_data_* function filled (same as in fax_t30_tx.c).
 * @note  `w14` receives the engine pointer truncated to int; FAXVMI_create() does not read that field.
 */
static FaxVmiCfg vmi_cfg_of(const FaxVmiM* m)
{
  FaxVmiCfg c;
  c.mode = m->w0;
  c.flag = (int)m->u4.init1;
  c.ringMin = (unsigned short)m->s08;
  c.maxItems = (unsigned short)m->s0a;
  c.frmWords = (unsigned short)m->s0c;
  c.id = (unsigned short)m->s0e;
  c.mcfg = m->u10.cfg;
  c.w14 = (int)(uintptr_t)m->eng;
  return c;
}

/* ===================================================================================================================== DTE input side */
/**
 * @brief Convert DLE-stuffed DTE data bytes into words.
 * @param o    the object (dlePending, dataEnd and lastByte are updated)
 * @param in   `*n` bytes from the DTE; DLE DLE is a data byte 0x10, DLE ETX ends the stream
 * @param out  one word per data byte
 * @param n    in: number of input bytes; out: number of words written
 * @return 0
 *
 * After DLE ETX dataEnd is set and 20 zero words are appended (they flush the modem); further input is ignored (*n = 0).  lastByte is the last byte of the
 * block.
 * @warning `out` is not bounds-checked for ordinary data: it must have room for *n words (the callers pass o->buf and at most the free room of the FIFO).
 */
int _handle_data_input(FaxT30* o, const unsigned char* in, unsigned short* out, int* n)
{
  int outPos = 0, inPos;

  if (o->dataEnd != 0)
  {
    *n = 0;
    return 0;
  }

  if (*n > 0)
    o->lastByte = in[*n - 1];

  for (inPos = 0; *n > inPos; inPos++)
  {
    unsigned char ch;
    fax_handle_data_input_count++;
    if (o->dlePending != 0)
    {
      if (DBG3)
        dsplibs_debug_printf("CLASS1: DLE %1X in data\n", (unsigned)in[inPos]);
      o->dlePending = 0;
      ch = in[inPos];
      if (ch == 0x10)
      {
        out[outPos++] = 0x10;
      }
      else if (ch == 3)
      {
        int k;
        o->dataEnd = 1;
        for (k = 0x13; k >= 0; k--)
          if (outPos <= 0x7ff)
            out[outPos++] = 0;
        *n = outPos;
        return 0;
      }
    }
    else
    {
      ch = in[inPos];
      if (ch == 0x10)
        o->dlePending = 1;
      else
        out[outPos++] = ch;
    }
  }
  *n = outPos;
  return 0;
}

/** @brief Start collecting a frame: the write index hdlcWr becomes 1 (buf[0] is reserved for the length). @return 0 */
int _handle_hdlc_input_open(FaxT30* o)
{
  o->hdlcWr = 1;
  return 0;
}

/** @brief Finish the frame being collected: buf[0] = hdlcWr - 1 (the length); lastFrame is set when the control byte buf[2] has bit 4 (final frame). @return
 * 0 */
int _handle_hdlc_input_close(FaxT30* o)
{
  int idx = o->hdlcWr - 1;
  int last = (o->buf[2] & 0x10u) != 0;
  o->buf[0] = (unsigned short)idx;
  if (last)
    o->lastFrame = 1;
  return 0;
}

/**
 * @brief Collect DLE-stuffed frame bytes from the DTE into a frame buffer.
 * @param o    the object (hdlcWr is the write index into `out`, dlePending the DLE state; lastFrame is set for a final frame)
 * @param in   `*n` bytes from the DTE; DLE DLE is a data byte 0x10, DLE ETX ends the frame
 * @param out  the frame buffer (o->buf): one word per byte from index hdlcWr
 * @param n    in: number of input bytes; out: the index after the last word when the frame is complete (buf[0] = that - 1 is the frame length), else 0
 * @return 1 when the frame is complete (DLE ETX seen), else 0
 * @warning The write index is not range-checked: a frame of more than 0x8ff bytes without DLE ETX, accumulated over several calls, writes past o->buf.
 */
int _handle_hdlc_input(FaxT30* o, const unsigned char* in, unsigned short* out, int* n)
{
  int idx = o->hdlcWr, i = 0, done = 0;
  for (;;)
  {
    COMPILER_BARRIER();
    if (!(*n > 0))
      break;
    if (o->dlePending != 0)
    {
      unsigned char ch;
      o->dlePending = 0;
      ch = in[i];
      if (ch == 0x10)
      {
        out[idx++] = 0x10;
      }
      else if (ch == 3)
      {
        int last;
        o->hdlcWr = idx;
        o->buf[0] = (unsigned short)(idx - 1);
        last = (o->buf[2] & 0x10u) != 0;
        if (last)
          o->lastFrame = 1;
        *n = idx;
        done = 1;
        break;
      }
    }
    else
    {
      unsigned char ch = in[i];
      if (ch == 0x10)
        o->dlePending = 1;
      else
        out[idx++] = ch;
    }
    i++;
    COMPILER_BARRIER();
    if (!(*n > i))
      break;
  }
  o->hdlcWr = idx;
  if (!done)
  {
    *n = 0;
    return 0;
  }
  return 1;
}

/* ===================================================================================================================== receive silence state */
/** @brief Start the receive-silence state: timer n / 2 blocks (at least 1), silenceTime = 0, energy cleared. @return the timer */
int _recieve_silence_state_init(FaxT30* o, int n)
{
  int h = n / 2;
  o->state = FAXC1_ST_RECIEVE_SILENCE_STATE;
  o->silenceTime = 0;
  o->silence_rms = (int)((unsigned int)o->silence_rms & 0xffff0000u);
  if (h == 0)
    h = 1;
  o->tmr = h;
  return h;
}

/**
 * @brief State 16: wait until the line has been quiet (energy of the received block <= 100) for `tmr` blocks.
 *
 * Sends silence.  A pending DTE byte aborts with FAX_CLASS1_OK_NO_CARRIER; noise restarts the quiet count; after tmr quiet blocks the command ends with
 * FAX_CLASS1_OK_NO_CARRIER.
 */
int fax_recieve_silence_state(FaxT30* o, short* rx, short* tx, unsigned char* tty_out, const unsigned char* tty_in, int* n_in, int* n_out, int* p7, int* p8)
{
  int pending = *p8, i;
  short e;
  (void)tty_out;
  (void)tty_in;
  (void)p7;

  for (i = 0; i < 0xa0; i++)
    tx[i] = 0;
  *n_out = 0xa0;

  e = FPM_rms(rx, (short)(unsigned short)*n_in);
  o->silence_rms = (int)(((unsigned int)o->silence_rms & 0xffff0000u) | (unsigned short)e);
  if (pending != 0)
  {
    if (DBG)
      dsplibs_debug_printf("Abort waiting for silence!");
    o->status = FAX_CLASS1_OK_NO_CARRIER;
    o->state = FAXC1_ST_IDLE_STATE;
    return 0;
  }
  if (e > 0x64)
  {
    if (DBG)
      dsplibs_debug_printf("Energy %d > silence treshold\n", (int)e);
    o->silenceTime = 0;
    o->status = FAX_CLASS1_NO_MESSAGE;
    *p8 = 5;
    return 0;
  }
  if (DBG)
    dsplibs_debug_printf("Energy %d < silence treshold...\n", (int)e);
  {
    unsigned int t = (unsigned int)o->silenceTime + 1u;
    o->silenceTime = (int)t;
    if (t >= (unsigned int)o->tmr)
    {
      o->status = FAX_CLASS1_OK_NO_CARRIER;
      o->state = FAXC1_ST_IDLE_STATE;
    }
  }
  return 0;
}

/* ===================================================================================================================== HDLC receive */
/** @brief Clear the state timer on entering the HDLC receive state. @return 0 */
int _hdlc_receive_state_init(FaxT30* o)
{
  if (DBG)
    dsplibs_debug_printf("At %2d.%02d[sec] hdlc_receive_state_init\n", o->sec, o->ms);
  o->tmr = 0;
  return 0;
}

/**
 * @brief Restart the V.21 receive channel (FAXVMI_control(): flush, reset the framing, HDLC mode, V21RX_CTL with the re-create flag, bit 9 of word 3).
 * @param mode  3: also select state 4 (_hdlc_receive_look_carrier_state)
 *
 * The state timer and rx_kind are cleared.
 */
void _cHDLCrx_init_from_idle(FaxT30* o, int mode)
{
  int v[4];
  FaxVmiCtl c;
  memcpy(v, V21RX_CTL, sizeof v);
  v[3] = (int)((unsigned int)v[3] | 0x200u);
  c = FAXVMI_CTL;
  c.flushRing = 1;
  c.rxReset = 1;
  c.newMode = FAXVMI_MODE_HDLC;
  c.mctl = v;
  FAXVMI_control(o->hdlc_rx, &c);
  if (mode == 3)
    o->state = FAXC1_ST_HDLC_RECEIVE_LOOK_CARRIER_STATE;
  o->tmr = 0;
  o->rx_kind = 0;
  if (DBG)
    dsplibs_debug_printf("At %2d.%02d[sec]  HDLCrx_init_from_idle\n", o->sec, o->ms);
}

/**
 * @brief Common abort of the receive states when the DTE sends a byte: go idle (state 8), hand the DTE a bare DLE ETX and report FAX_CLASS1_OK.
 * @note  The callers print their own diagnostics and set *p8 = 5.
 */
static void rx_abort_to_command(FaxT30* o, unsigned char* tty_out, int* p7)
{
  o->state = FAXC1_ST_IDLE_STATE;
  _idle_state_init(o);
  *p7 = cTOOLS_handle_hdlc_output(o, o->buf + 1, tty_out, 0, 1);
  o->status = FAX_CLASS1_OK;
}

/**
 * @brief State 5: receive HDLC frames from the V.21 channel.
 *
 * A good frame is converted to DLE-stuffed bytes for the DTE (tty_out, *p7 bytes), rx_event = FAX_CLASS1_OK, the V.21 AGC gain is saved, and the object goes
 * to state 6. A frame with a bad CRC sets rx_event = FAX_CLASS1_ERROR and also goes to state 6. Loss of carrier ends in the idle state with
 * FAX_CLASS1_NO_CARRIER. A pending DTE byte aborts to command mode. Sends silence, *p8 = 5.
 */
int _hdlc_receive_state(FaxT30* o, short* rx, short* tx, unsigned char* tty_out, const unsigned char* tty_in, int* n_in, int* n_out, int* p7, int* p8)
{
  unsigned short nFrames = 0, nSamples = (unsigned short)*n_in;
  unsigned int timer;
  (void)tty_in;

  if (o->state_prev == FAXC1_ST_HDLC_RECEIVE_BETWEEN_BUFFERS_STATE)
    o->status = FAX_CLASS1_CONNECT;

  timer = (unsigned int)o->tmr + 1u;
  o->tmr = (int)timer;
  if (timer != 0u)
  {
    unsigned int vmiState;
    o->buf[0] = 0;
    vmiState = FAXVMI_process(o->hdlc_rx, o->buf, rx, &nFrames, &nSamples);
    if ((vmiState & FAXVMI_ST_CARRIER) == 0)
    {
      if (DBG)
        dsplibs_debug_printf("At %2d.%02d[sec] No carrier in HDLC receive state\n", o->sec, o->ms);
      o->state = FAXC1_ST_IDLE_STATE;
      *p7 = cTOOLS_handle_hdlc_output(o, o->buf + 1, tty_out, 0, 1);
      o->rx_kind = 2;
      o->rx_event = FAX_CLASS1_NO_CARRIER;
    }

    /* a frame was received: hand it to the DTE, or note the error */
    if (nFrames != 0)
    {
      unsigned short len = o->buf[0];
      if (len == 0)
      {
        if (DBG)
          dsplibs_debug_printf("%2d.%02d[sec] Receive buffer with error in _hdlc_receive_state\n", o->sec, o->ms);
        o->rx_kind = 2;
        o->rx_event = FAX_CLASS1_ERROR;
        o->state = FAXC1_ST_HDLC_RECEIVE_BETWEEN_BUFFERS_STATE;
      }
      else
      {
        const V21RxBundle* bd;
        if (DBG)
        {
          dsplibs_debug_printf("%2d.%02d[sec] Receive buffer OK in _hdlc_receive_state\n", o->sec, o->ms);
          len = o->buf[0];
        }
        *p7 = cTOOLS_handle_hdlc_output(o, o->buf + 1, tty_out, (int)len, 1);
        o->rx_kind = 2;
        o->rx_event = FAX_CLASS1_OK;
        bd = ((const V21Rx*)o->hdlc_rx->m->eng)->bundle;
        {
          short rcp = bd->agc.rcp, shift = bd->agc.shift;
          COMPILER_BARRIER();
          o->agc_rcp = rcp;
          o->agc_shift = shift;
        }
        o->state = FAXC1_ST_HDLC_RECEIVE_BETWEEN_BUFFERS_STATE;
      }
    }
  }

  /* the DTE sent a byte: abort to command mode */
  if (*p8 != 0)
  {
    if (DBG)
      dsplibs_debug_printf("%2d.%02d[sec] TxDatCnt>0 in _hdlc_receive_state... abort command mode\n", o->sec, o->ms);
    rx_abort_to_command(o, tty_out, p7);
  }
  *p8 = 5;
  _put_silence(tx, 0xa0);
  *n_out = 0xa0;
  return 0;
}

/**
 * @brief State 4: look for the V.21 carrier, sending the calling tone when cng_on.
 *
 * On carrier detect the state becomes 5 (_hdlc_receive_state), the timer is cleared, the calling tone is switched off, and the gain attenuation to request
 * (gain_att) is derived from the AGC history. Without a carrier for S7 seconds (and no calling tone) the command ends with FAX_CLASS1_NO_CARRIER. A pending
 * DTE byte aborts to command mode. *p8 = 5.
 */
int _hdlc_receive_look_carrier_state(FaxT30* o, short* rx, short* tx, unsigned char* tty_out, const unsigned char* tty_in, int* n_in, int* n_out, int* p7,
                                     int* p8)
{
  unsigned short nFrames = 0, nSamples = (unsigned short)*n_in;
  int nIn = *n_in, timeout;
  unsigned int vmiState;
  (void)tty_in;

  /* carrier search timeout: S7 seconds in blocks */
  if (nIn != 0)
    timeout = (int)((unsigned int)o->s7 * SAMPLE_RATE_8000) / nIn;
  else
    timeout = (int)((unsigned int)o->s7 * 50u);
  o->tmr = (int)((unsigned int)o->tmr + 1u);

  vmiState = FAXVMI_process(o->hdlc_rx, o->buf, rx, &nFrames, &nSamples);
  if ((vmiState & FAXVMI_ST_CARRIER) != 0)
  {
    const V21RxBundle* bd;
    int k = 0, att = 12;
    if (DBG)
      dsplibs_debug_printf("At %2d.%02d[sec] Carrier Detected in _hdlc_receive_look_carrier_state\n", o->sec, o->ms);
    o->status = FAX_CLASS1_CONNECT;
    if (DBG)
      dsplibs_debug_printf("At %2d.%02d[sec] hdlc_receive_state_init\n", o->sec, o->ms);
    o->tmr = 0;
    o->state = FAXC1_ST_HDLC_RECEIVE_STATE;
    o->cng_on = 0;
    bd = ((const V21Rx*)o->hdlc_rx->m->eng)->bundle;
    for (;;)
    {
      short h = (short)bd->agc.hist;
      short thr = fax_gain_threshold_table[k];
      COMPILER_BARRIER();
      if (h < thr)
      {
        o->gain_att = att;
        if (DBG)
          dsplibs_debug_printf("Gain Attenuation Reuqest: +%d[dB], avg_rms = %d", att, (int)h);
        break;
      }
      k = (short)(k + 1);
      att -= 3;
      if (k > 3)
        break;
    }
  }
  else if (timeout != 0 && (unsigned int)o->tmr > (unsigned int)timeout && o->cng_on == 0)
  {
    if (DBG)
      dsplibs_debug_printf("At %2d.%02d[sec], curent_timeout = %d, No carrier in _hdlc_receive_look_carrier_state, S7 = %d[sec]\n", o->sec, o->ms, timeout,
                           o->s7);
    o->state = FAXC1_ST_IDLE_STATE;
    o->status = FAX_CLASS1_NO_CARRIER;
    o->cng_on = 0;
  }

  /* output: silence, or the calling tone cadence (0.5 s tone, 3 s silence) */
  if (o->cng_on == 0)
  {
    _put_silence(tx, 0xa0);
    *n_out = 0xa0;
  }
  else if (o->cng_tone == 0)
  {
    int c;
    _put_silence(tx, 0xa0);
    *n_out = 0xa0;
    c = o->cng_ctr + 0xa0;
    if (c <= 0x5dc0)
    {
      o->cng_ctr = c;
    }
    else
    {
      o->cng_ctr = 0;
      o->cng_tone = 1;
    }
  }
  else
  {
    int c;
    FPM_TONE_generate(o->tone, tx, 0xa0);
    *n_out = 0xa0;
    c = o->cng_ctr + 0xa0;
    if (c <= 0xfa0)
    {
      o->cng_ctr = c;
    }
    else
    {
      o->cng_ctr = 0;
      o->cng_tone = 0;
    }
  }

  if (*p8 != 0)
  {
    if (DBG)
      dsplibs_debug_printf("At %2d.%02d[sec] TxDatCnt>0 in _hdlc_receive_look_carrier_state abort command mode.\n", o->sec, o->ms);
    rx_abort_to_command(o, tty_out, p7);
  }
  *p8 = 5;
  return 0;
}

/**
 * @brief State 6: command mode after a frame; frames that arrive before the DTE collects them are appended to the superframe.
 *
 * A frame that does not fit in the superframe (fill + len + 1 > 0xff words) is skipped.  Loss of carrier ends in the idle state with
 * FAX_CLASS1_NO_CARRIER_NO_MESSAGE.  A pending DTE byte aborts to command mode.  Sends silence, *p8 = 5.
 */
int _hdlc_receive_between_buffers_state(FaxT30* o, short* rx, short* tx, unsigned char* tty_out, const unsigned char* tty_in, int* n_in, int* n_out, int* p7,
                                        int* p8)
{
  unsigned short nFrames = 0, nSamples = (unsigned short)*n_in;
  unsigned int vmiState;
  (void)tty_in;
  o->buf[0] = 0;
  vmiState = FAXVMI_process(o->hdlc_rx, o->buf, rx, &nFrames, &nSamples);

  /* append a frame that arrived while the DTE was busy to the superframe */
  if (nFrames != 0)
  {
    int len, fill;
    len = (int)(unsigned short)o->buf[0];
    fill = o->sf_fill;
    COMPILER_BARRIER();
    if (DBG)
      dsplibs_debug_printf("%2d.%02d[sec] Missing HDLC frame of %d during command mode(%d already in)!\n", o->sec, o->ms, len, fill);
    len = (int)(unsigned short)o->buf[0];
    fill = o->sf_fill;
    if (fill + len + 1 > 0xff)
    {
      if (DBG)
        dsplibs_debug_printf("SuperFrame full, skipping HDLC frame!\n");
    }
    else
    {
      int idx;
      short k;
      o->buf[0x800 + fill] = (unsigned short)len;
      idx = fill + 1;
      for (k = 0; k < len; k = (short)(k + 1))
      {
        o->buf[0x800 + idx] = o->buf[1 + k];
        idx++;
      }
      o->sf_fill = idx;
    }
  }

  if ((vmiState & FAXVMI_ST_CARRIER) == 0)
  {
    if (DBG)
      dsplibs_debug_printf("%2d.%02d[sec] No carrier during command mode, NO MESSAGE****\n", o->sec, o->ms);
    o->status = FAX_CLASS1_NO_CARRIER_NO_MESSAGE;
    o->state = FAXC1_ST_IDLE_STATE;
  }

  if (*p8 != 0)
  {
    if (DBG)
      dsplibs_debug_printf("%2d.%02d[sec] TxDatCnt>0 in _hdlc_receive_between_buffers_state abort command mode.\n", o->sec, o->ms);
    rx_abort_to_command(o, tty_out, p7);
  }
  *p8 = 5;
  _put_silence(tx, 0xa0);
  *n_out = 0xa0;
  return 0;
}

/**
 * @brief State 7: the frames collected in the superframe are handed to the DTE one per call after a delay (emu_delay); then the superframe is cleared.
 *
 * When every frame has been delivered the command ends with FAX_CLASS1_NO_CARRIER_NO_MESSAGE.  Sends silence.
 */
int _hdlc_emulate_receive_state(FaxT30* o, short* rx, short* tx, unsigned char* tty_out, const unsigned char* tty_in, int* n_in, int* n_out, int* p7, int* p8)
{
  int fr[256];
  int total = o->sf_fill, pos = 0, nfr = 0, cur, delay;
  (void)rx;
  (void)tty_in;
  (void)n_in;
  (void)p8;

  /* the lengths of the buffered frames */
  while (total > pos && nfr < 256)
  {
    int len = (int)(unsigned short)o->buf[0x800 + pos];
    nfr++;
    pos = len + pos + 1;
    fr[nfr - 1] = len;
  }

  /* the delay counts down once per call; the first call of the state reports the connect */
  if (o->state_prev == FAXC1_ST_HDLC_EMULATE_RECEIVE_STATE)
  {
    delay = o->emu_delay;
    cur = o->emu_idx;
    o->emu_delay = delay - 1;
  }
  else
  {
    cur = o->emu_idx;
    if (cur < nfr)
      o->status = FAX_CLASS1_CONNECT;
    delay = o->emu_delay;
    o->emu_delay = delay - 1;
  }

  /* time to deliver the next frame (or to finish) */
  if (delay <= 0)
  {
    if (cur >= nfr)
    {
      o->status = FAX_CLASS1_NO_CARRIER_NO_MESSAGE;
      o->state = FAXC1_ST_IDLE_STATE;
    }
    else
    {
      int off = 0, j;
      if (DBG)
      {
        dsplibs_debug_printf("%2d.%02d[sec] Receive buffer OK in _hdlc_emulate_receive_state\n", o->sec, o->ms);
        cur = o->emu_idx;
      }
      for (j = 0; j < cur; j++)
        off += fr[j] + 1;
      {
        int flen = (int)(unsigned short)o->buf[0x800 + off];
        COMPILER_BARRIER();
        *p7 = cTOOLS_handle_hdlc_output(o, &o->buf[0x800 + off + 1], tty_out, flen, 1);
      }
      o->rx_event = FAX_CLASS1_OK;
      o->rx_kind = 2;
      cur = o->emu_idx + 1;
      o->emu_idx = cur;
      o->state = FAXC1_ST_IDLE_STATE;
    }
  }

  /* all frames delivered: clear the superframe */
  if (cur == nfr)
  {
    o->sf_fill = 0;
    o->emu_delay = 2;
    o->emu_idx = 0;
  }

  _put_silence(tx, 0xa0);
  *n_out = 0xa0;
  return 0;
}

/* ===================================================================================================================== data modem receiver */
/** @brief Create or restart the data receiver for `modem` (_init_receiver()), reset the bit alignment and clear the timer. @return 0 */
int _rx_look_carrier_init(FaxT30* o, int modem)
{
  _init_receiver(o, modem);
  cTOOLS_handle_data_output_reset(o);
  o->tmr = 0;
  return 0;
}

/**
 * @brief State 12: look for the data modem carrier (and meanwhile for an HDLC carrier).
 *
 * A V.21 carrier while the data carrier is absent selects the HDLC receive state with FAX_CLASS1_OTHER_CARRIER; a data connect selects state 13 with
 * FAX_CLASS1_CONNECT; S7 seconds without a carrier end the command with FAX_CLASS1_NO_CARRIER.  A pending DTE byte aborts to command mode.  Sends silence,
 * *p8 = 5.
 */
int _rx_look_carrier_state(FaxT30* o, short* rx, short* tx, unsigned char* tty_out, const unsigned char* tty_in, int* n_in, int* n_out, int* p7, int* p8)
{
  unsigned short nData = 0, nFrames = 0, nDataSamples = (unsigned short)*n_in, nFrameSamples = nDataSamples;
  int nIn = *n_in, timeout;
  unsigned int vmiState;
  (void)tty_in;

  /* carrier search timeout: S7 seconds in blocks */
  if (nIn != 0)
    timeout = (int)((unsigned int)o->s7 * SAMPLE_RATE_8000) / nIn;
  else
    timeout = (int)((unsigned int)o->s7 * 50u);
  o->tmr = (int)((unsigned int)o->tmr + 1u);

  vmiState = FAXVMI_process(o->data_vmi, o->buf, rx, &nData, &nDataSamples);
  if ((vmiState & FAXVMI_ST_CARRIER) == 0)
  {
    unsigned int hdlcState = FAXVMI_process(o->hdlc_rx, o->buf, rx, &nFrames, &nFrameSamples);
    if ((hdlcState & FAXVMI_ST_CARRIER) != 0)
    {
      if (DBG)
        dsplibs_debug_printf("At %2d.%02d[sec] HDLC frame detected during look for DATA carrier !!!\n", o->sec, o->ms);
      _hdlc_receive_state_init(o);
      o->state = FAXC1_ST_HDLC_RECEIVE_STATE;
      o->status = FAX_CLASS1_OTHER_CARRIER;
    }
  }

  if (((vmiState >> 8) & 3u) == 1u)
  {
    if (DBG)
      dsplibs_debug_printf("At %2d.%02d[sec] Data RX connect in _rx_look_carrier_state\n", o->sec, o->ms);
    o->status = FAX_CLASS1_CONNECT;
    o->state = FAXC1_ST_RX_DATA_STATE;
  }

  if (timeout != 0 && (unsigned int)o->tmr > (unsigned int)timeout && (vmiState & FAXVMI_ST_CARRIER) == 0)
  {
    if (DBG)
      dsplibs_debug_printf("S7 time elapsed in look carrier\n");
    o->status = FAX_CLASS1_NO_CARRIER;
    o->state = FAXC1_ST_IDLE_STATE;
  }

  if (*p8 != 0)
  {
    if (DBG)
      dsplibs_debug_printf("TxDatCnt !=0 in _rx_look_carrier_state... abort to command mode\n");
    rx_abort_to_command(o, tty_out, p7);
  }
  *p8 = 5;
  _put_silence(tx, *n_in);
  *n_out = *n_in;
  return 0;
}

/**
 * @brief State 13: receive data.  The data channel's words (*p7 of them) are converted to DLE-stuffed bytes in tty_out; *p7 is the byte count on return.
 *
 * Loss of carrier appends DLE ETX, reports FAX_CLASS1_NO_CARRIER through rx_event and goes idle.  A pending DTE byte aborts to command mode.  Sends silence,
 * *p8 = 5.
 */
int _rx_data_state(FaxT30* o, short* rx, short* tx, unsigned char* tty_out, const unsigned char* tty_in, int* n_in, int* n_out, int* p7, int* p8)
{
  unsigned short nSamples = (unsigned short)*n_in, nItems = (unsigned short)*p7;
  unsigned int vmiState;
  (void)tty_in;

  o->tmr = (int)((unsigned int)o->tmr + 1u);
  vmiState = FAXVMI_process(o->data_vmi, o->buf, rx, &nItems, &nSamples);

  /* convert the received words for the DTE; loss of carrier ends the data with DLE ETX */
  if ((vmiState & FAXVMI_ST_CARRIER) == 0)
  {
    if (DBG)
      dsplibs_debug_printf("At %2d.%02d[sec] No carrier in _rx_data_state, move to idle\n", o->sec, o->ms);
    o->state = FAXC1_ST_IDLE_STATE;
    _idle_state_init(o);
    nItems = (unsigned short)_handle_data_output(o, o->buf, tty_out, (short)nItems, 1);
    o->rx_kind = 2;
    o->rx_event = FAX_CLASS1_NO_CARRIER;
  }
  else
  {
    nItems = (unsigned short)_handle_data_output(o, o->buf, tty_out, (short)nItems, 0);
  }

  if (*p8 != 0)
  {
    if (DBG)
      dsplibs_debug_printf("TxDatCnt !=0 in _rx_data_state... abort to command mode\n");
    o->state = FAXC1_ST_IDLE_STATE;
    _idle_state_init(o);
    nItems = (unsigned short)cTOOLS_handle_hdlc_output(o, o->buf + 1, tty_out, 0, 1);
    o->status = FAX_CLASS1_OK;
  }

  *p8 = 5;
  *p7 = (short)nItems;
  _put_silence(tx, *n_in);
  *n_out = *n_in;
  return 0;
}

/**
 * @brief Release the data modem receive channel: the V.17 receiver's tables (if it is a V.17 receiver), the config blocks and the VMI channel.
 * @warning o->data_cfg and o->data_vmi must not be NULL.
 */
void _delete_data_rx_modem(FaxT30* o)
{
  if ((unsigned short)o->data_cfg->s0e == FAXVMI_ID_V17RX)
  {
    sysdep_free(((V17RxCfg*)o->data_cfg->u10.cfg)->freq);
    sysdep_free(((V17RxCfg*)o->data_cfg->u10.cfg)->coefQ);
    sysdep_free(((V17RxCfg*)o->data_cfg->u10.cfg)->coefI);
  }
  sysdep_free(o->data_cfg->u10.cfg);
  sysdep_free(o->data_cfg);
  o->data_cfg = NULL;
  FAXVMI_delete(o->data_vmi);
  o->data_vmi = NULL;
}

/** @brief The VMI's bits-per-symbol code of a data rate (stored in m->u10.s10 of the VMI object; the unpack function reads that many bits); 0 for other
 * rates. */
static short rate_code(int rate)
{
  switch (rate)
  {
  case RATE_7200:
    return 3;
  case RATE_2400:
    return 2;
  case RATE_4800:
    return 3;
  case RATE_12000:
    return 5;
  case RATE_9600:
    return 4;
  case RATE_14400:
    return 6;
  default:
    return 0;
  }
}

/**
 * @brief Create or restart the data modem receive channel for a modem selection and select the carrier-search state (12).
 * @param o      the object; a receiver of another modem class is deleted first
 * @param modem  a FaxModemSel value; any other value leaves class and rate at 0
 *
 * A new channel gets the saved V.21 AGC gain (agc_rcp / agc_shift); an existing receiver of the same class is restarted with a control record carrying the
 * new rate (and the short-train flag for V.17). The data receiver's carrier flag is set from cd_flag in both cases.
 * @warning If an allocation fails, or the object has a data_cfg but no data_vmi, the function returns without selecting a state.
 */
void _init_receiver(FaxT30* o, int modem)
{
  int short_train = 0, cls = 0, rate = 0;

  if (DBG)
    dsplibs_debug_printf("%2d.%02d[sec] Initializing RX modem receiver, MODEM_IDX=%d\n", o->sec, o->ms, o->modemClass);
  if (modem == FAXMOD_V17_7200_SHORT || modem == FAXMOD_V17_9600_SHORT || modem == FAXMOD_V17_12000_SHORT || modem == FAXMOD_V17_14400_SHORT)
  {
    short_train = 1;
    if (DBG)
      dsplibs_debug_printf("Short train in V.17 mode is selected\n");
  }

  /* class and rate per modem selection */
  switch (modem)
  {
  case FAXMOD_V17_14400:
  case FAXMOD_V17_14400_SHORT:
    cls = FAXMC_V17;
    rate = RATE_14400;
    break;
  case FAXMOD_V17_12000:
  case FAXMOD_V17_12000_SHORT:
    cls = FAXMC_V17;
    rate = RATE_12000;
    break;
  case FAXMOD_V17_9600:
  case FAXMOD_V17_9600_SHORT:
    cls = FAXMC_V17;
    rate = RATE_9600;
    break;
  case FAXMOD_V17_7200:
  case FAXMOD_V17_7200_SHORT:
    cls = FAXMC_V17;
    rate = RATE_7200;
    break;
  case FAXMOD_V29_9600:
    rate = RATE_9600;
    cls = FAXMC_V29;
    break;
  case FAXMOD_V29_7200:
    rate = RATE_7200;
    cls = FAXMC_V29;
    break;
  case FAXMOD_V27_4800:
    rate = RATE_4800;
    cls = FAXMC_V27;
    break;
  case FAXMOD_V27_2400:
    rate = RATE_2400;
    cls = FAXMC_V27;
    break;
  default:
    break;
  }

  /* a receiver of another class is deleted first */
  if (o->modemClass != cls && o->data_cfg != NULL && o->data_vmi != NULL)
  {
    if (DBG)
      dsplibs_debug_printf("%2d.%02d[sec] New RX Modem... Deleting previous existing one\n", o->sec, o->ms);
    _delete_data_rx_modem(o);
  }

  /* create a new receiver, or restart the existing one */
  if (o->data_cfg == NULL)
  {
    FaxVmiCfg cfg;
    FaxVmiM* m = (FaxVmiM*)sysdep_malloc((unsigned int)sizeof(FaxVmiM)); /* 0x18 bytes in 32-bit builds */
    o->data_cfg = m;
    if (m == NULL)
      return; /* allocation failed: nothing is selected */
    fax_init_vmi_data_rx_modem[cls](m, (unsigned short)rate, 0, NULL);
    cfg = vmi_cfg_of(m);
    o->data_vmi = FAXVMI_create(NULL, &cfg);
    if (o->data_vmi == NULL)
      return; /* the channel could not be created */

    /* give the new receiver's AGC the gain measured on the V.21 channel */
    {
      void* eng = o->data_vmi->m->eng;
      short rcp = (short)o->agc_rcp, shift = (short)o->agc_shift;
      COMPILER_BARRIER();
      if (cls == FAXMC_V17)
      {
        V17RxBundle* bd = ((V17Rx*)eng)->bundle;
        bd->agc.rcp = rcp;
        bd->agc.shift = shift;
      }
      else if (cls == FAXMC_V29)
      {
        V29RxBundle* bd = ((V29Rx*)eng)->bundle;
        bd->agc.rcp = rcp;
        bd->agc.shift = shift;
      }
      else if (cls == FAXMC_V27)
      {
        V27RxBundle* bd = ((V27Rx*)eng)->bundle;
        bd->agc.rcp = rcp;
        bd->agc.shift = shift;
      }
    }
  }
  else
  {
    /* restart the existing receiver */
    FaxVmiCtl c;
    int v[5];
    FaxVmiM* vm;
    unsigned short id = (unsigned short)o->data_cfg->s0e;
    void* eng;
    if (o->data_vmi == NULL)
      return; /* a config block without a channel: nothing to restart */
    if (DBG)
      dsplibs_debug_printf("Restarting existing RX modem\n");
    if (id == FAXVMI_ID_V17RX)
    {
      v[0] = (int)((unsigned int)V17RX_CTL[0] & 0xffffu) | (int)((unsigned int)(unsigned short)rate << 16);
      v[1] = V17RX_CTL[1];
      v[2] = V17RX_CTL[2];
      v[3] = (int)((unsigned int)V17RX_CTL[3] | 0x200u);
      v[4] = short_train;
    }
    else if (id == FAXVMI_ID_V29RX)
    {
      v[0] = (int)((unsigned int)V29RX_CTL[0] & 0xffffu) | (int)((unsigned int)(unsigned short)rate << 16);
      v[1] = V29RX_CTL[1];
      v[2] = V29RX_CTL[2];
      v[3] = (int)((unsigned int)V29RX_CTL[3] | 0x200u);
      v[4] = 0; /* not part of the record */
    }
    else
    {
      v[0] = (int)((unsigned int)V27RX_CTL[0] & 0xffffu) | (int)((unsigned int)(unsigned short)rate << 16);
      v[1] = V27RX_CTL[1];
      v[2] = V27RX_CTL[2];
      v[3] = (int)((unsigned int)V27RX_CTL[3] | 0x200u);
      v[4] = V27RX_CTL[4];
    }
    c = FAXVMI_CTL;
    c.mctl = v;

    vm = o->data_vmi->m;
    eng = vm->eng;
    COMPILER_BARRIER();
    /* the baud rate in the engine's config block (the three config blocks agree on the position; typed by the id of the existing receiver) */
    if (id == FAXVMI_ID_V17RX)
      ((V17Rx*)eng)->cfg.s[2] = (short)rate;
    else if (id == FAXVMI_ID_V29RX)
      ((V29Rx*)eng)->cfg.s[2] = (short)rate;
    else
      ((V27Rx*)eng)->cfg.s[2] = (short)rate;
    COMPILER_BARRIER();
    vm->u10.s10 = rate_code(rate);
    COMPILER_BARRIER();
    FAXVMI_control(o->data_vmi, &c);
  }

  /* the carrier flag of the data receiver's half-duplex block */
  {
    void* eng = o->data_vmi->m->eng;
    short cd = (short)o->cd_flag;
    COMPILER_BARRIER();
    if (cls == FAXMC_V17)
      ((V17Rx*)eng)->hdx->s20 = cd;
    else if (cls == FAXMC_V29)
      ((V29Rx*)eng)->hdx->s1c = cd;
    else if (cls == FAXMC_V27)
      ((V27Rx*)eng)->hdx->s14 = cd;
  }

  o->state = FAXC1_ST_RX_LOOK_CARRIER;
  o->modemClass = cls;
}
