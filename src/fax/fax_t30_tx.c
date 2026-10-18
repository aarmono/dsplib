/**
 * @file fax_t30_tx.c
 * @brief The T.30 / HDLC transmit side of the fax class 1 object: the transmit state machines and their initialisers, the DTE-output helpers, the shared
 *        idle / silence / answer-tone states and the T.30 frame-name lookups.
 *
 * - Transmit states (called by fax_class1_progress() through fax_class1_state_functions[]): _t30_silence_before_tx_state, _t30_preabmle_state,
 *   _send_hdlc_buffer_state, _send_hdlc_between_buffer_state (the V.21 HDLC phases), _tx_silence_before_scrm_ones, _tx_scrambled_ones_state, _tx_data_state,
 *   _tx_nulls_state (the data modem phases) and cHDLCtx_off (V.21 receive channel kept running while the transmitter is off).
 * - Shared states of the class 1 layer: fax_idle_state, fax_send_silence_state, fax_answer_tone_state.
 * - Initialisers: cHDLCtx_preamble_state_init, _tx_scrambled_ones_init, _init_transmitter (creates the V.27ter / V.29 / V.17 transmit channel for a modem
 *   selection), _delete_data_tx_modem. _init_tx_nulls_state, _send_silence_state_init, _send_hdlc_between_buffer_state_init and cHDLCtx_off_init have no
 *   caller in this library.
 * - Output helpers (formerly the tools of the class 1 layer): _put_silence, _handle_data_output (receive words -> DLE-stuffed bytes for the DTE, with bit
 *   alignment), cTOOLS_handle_hdlc_output (HDLC frame words -> DLE-stuffed bytes, with diagnostics), cTOOLS_handle_data_output_reset.
 * - GetT30FrameIDFromBuffer / GetT30FrameNameByID: frame identification and naming for the diagnostics (table in fax_t30_tables.c).
 * The input side that converts DTE bytes into words (_handle_data_input / _handle_hdlc_input) is in fax_t30_rx.c.
 *
 * The object type, the state-function signature and the calling convention are in fax_t30.h; every state function returns 0 and reports through o->status.
 * `sec` / `ms` of the object are the clock the debug lines print; every debug line needs dsplibs_debug_level > 1 (two need > 2).
 *
 * Notes on behaviour:
 * - The object IS its work buffer: every FIFO_read / FIFO_write / FAXVMI_process / _handle_*_input buffer argument is `o` (o->buf at offset 0).
 * - _init_transmitter: a modem value other than the twelve known ones leaves the rate and the modem class at 0. FAXVMI_control is called only for the
 *   short-train modes (0x4a / 0x62 / 0x7a / 0x92): the V.27ter / V.29 record branches are reachable only when the init table slot of the V.17 class
 *   (fax_init_vmi_data_tx_modem[2]) is replaced (the table is writable data; a test can do so). The V.27ter record carries the rate in the HIGH half of word
 *   0, the V.29 / V.17 records in the low half. The debug line "Initializing TX modem" prints the OLD modem class (modemClass).
 * - _tx_nulls_state / _tx_scrambled_ones_state / _tx_data_state write their FIFO "free room - 1" to *p8 after FAXVMI_process; with no new data (*p8 <= 0) the
 *   nulls state passes the 16-bit truncation of *p8 as the item count (0 for the usual 0).
 * - _t30_preabmle_state and the HDLC states pass the object as the frame buffer and the number of FRAMES as the count to FAXVMI_process (mode 2).
 * - cTOOLS_handle_hdlc_output and GetT30FrameIDFromBuffer index aReversedCharsArray with the low byte of the word / the byte.
 * - fax_cDATAtx_counter counts the calls of _tx_scrambled_ones_state / _tx_data_state (printed at debug level 3; the scrambled-ones state sets status = 6 on
 *   its first call after a reset).
 */
#include <limits.h>
#include <string.h>
#include "fax_t30.h"
#include "fax_common_tables.h"
#include "fax_v21_tables.h"
#include "fax_v27_tx_tables.h"
#include "fax_v29_tx_tables.h"
#include "fax_v17_tx_tables.h"

#include "fax_debug.h"          /* dsplibs_debug_level, DBG, DBG3 */
#include "core/dsp_constants.h" /* RATE_* */
#include "core/dsp_host.h"
#include "fpm/fpm.h"

int fax_cDATAtx_counter;

/**
 * @brief Build the FaxVmiCfg for FAXVMI_create() from the VMI block an init_vmi_data_* function filled (mode, flag, ring sizes, modem id, modem config, w14).
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

/* ===================================================================================================================== output helpers */
/** @brief Write `n` zero samples to `buf`. */
void _put_silence(short* buf, int n)
{
  int i;
  for (i = 0; i < n; i++)
    buf[i] = 0;
}

/** @brief Restart the bit alignment of the receive data stream: the next byte is aligned again (alignDone = 0, history all ones). */
void cTOOLS_handle_data_output_reset(FaxT30* o)
{
  o->alignDone = 0;
  o->alignHist = UINT_MAX;
}

/**
 * @brief Convert received data words into DLE-stuffed bytes for the DTE, aligning the bit stream on the first byte.
 * @param o     the object (alignment state: alignDone, alignHist, alignShift, alignMask)
 * @param in    `n` words; the low byte of each is a data byte
 * @param out   output bytes, 0x10 (DLE) doubled; room for 2 * n + 2 bytes is needed
 * @param n     number of words
 * @param flag  nonzero: append DLE ETX
 * @return number of bytes written
 *
 * While alignDone is 0 the first byte of a stream is aligned first: the first zero bit among bits 16 .. 23 of (byte << 24 | alignHist) selects the extraction
 * window (alignMask / alignShift: mask and shift); if there is no zero bit in the eight positions the byte stays 0xff and alignDone stays 0.
 * @warning `out` is not bounds-checked.
 */
int _handle_data_output(FaxT30* o, const unsigned short* in, unsigned char* out, int n, int flag)
{
  int inPos = 0, outLen = 0;

  if (outLen < n)
  {
    do
    {
      unsigned char outByte = 0xff;
      unsigned int window = ((unsigned int)(in[inPos] & 0xffu) << 24) | o->alignHist;
      int skip = 0;

      /* find the alignment of the first byte of the stream */
      if (o->alignDone == 0)
      {
        unsigned int bit = 0x10000u;
        int cnt = 0;
        o->alignShift = 0x10;
        o->alignMask = 0xff0000u;
        if ((bit & window) != 0)
        {
          for (;;)
          {
            o->alignMask <<= 1;
            cnt++;
            bit += bit;
            o->alignShift++;
            if (cnt > 7)
            {
              skip = 1; /* no zero bit in the eight positions: the byte stays 0xff, alignDone stays 0 */
              break;
            }
            if ((bit & window) == 0)
              break;
          }
        }
        if (!skip)
          o->alignDone = 1;
      }

      /* extract the aligned byte, keep the history, double a DLE */
      if (!skip)
        outByte = (unsigned char)((window & o->alignMask) >> (o->alignShift & 0x1f));
      o->alignHist = window >> 8;
      out[outLen++] = outByte;
      if (outByte == 0x10)
        out[outLen++] = 0x10;
      inPos++;
    } while (inPos < n);
  }

  if (flag != 0)
  {
    out[outLen] = 0x10;
    out[outLen + 1] = 3;
    outLen += 2;
  }
  return outLen;
}

/**
 * @brief Convert one received HDLC frame (words) into DLE-stuffed bytes for the DTE and print it in the diagnostics.
 * @param o     the object (unused)
 * @param in    the frame, `n` words; the low byte of each is a frame byte
 * @param out   output bytes, DLE (0x10) doubled, DLE ETX appended when `flag`; room for 2 * n + 2 bytes is needed
 * @param n     number of words (0 = no frame: only the DLE ETX is written when `flag`)
 * @param flag  nonzero: append DLE ETX
 * @return number of bytes written
 *
 * With debug level above 1 the frame name (n > 2) and the bytes (bit reversed through aReversedCharsArray) are printed.
 * @warning `out` is not bounds-checked.  The DLE test compares the whole word, so a word 0x110 would not be doubled although its low byte is 0x10 (the frame
 *          words are bytes).
 */
int cTOOLS_handle_hdlc_output(FaxT30* o, const unsigned short* in, unsigned char* out, int n, int flag)
{
  int k = 0, i;
  (void)o;

  if (n != 0)
  {
    /* diagnostics: the frame name from the address / control / FCF bytes */
    if (n > 2)
    {
      if (DBG)
      {
        unsigned int id = GetT30FrameIDFromBuffer((unsigned char)in[0], (unsigned char)in[1], (unsigned char)in[2]);
        dsplibs_debug_printf("FCL1: FRAME RECEIVED (%s)\n", GetT30FrameNameByID(id & 0xffff7fffu));
      }
    }
    if (DBG)
      dsplibs_debug_printf("HDLC Recieved Frame of %d: \n", n);

    /* copy the frame bytes, stuffing DLE */
    for (i = 0; i < n; i++)
    {
      if (DBG)
        dsplibs_debug_printf("%02X,", (unsigned)aReversedCharsArray[in[i] & 0xffu]);
      out[k++] = (unsigned char)in[i];
      if (in[i] == 0x10)
      {
        out[k++] = 0x10;
        if (DBG)
          dsplibs_debug_printf("%02X,", (unsigned)aReversedCharsArray[0x10]);
      }
    }

    if (flag != 0)
    {
      if (DBG)
      {
        dsplibs_debug_printf("%02X,", (unsigned)aReversedCharsArray[0x10]);
        dsplibs_debug_printf("%02X\n", (unsigned)aReversedCharsArray[3]);
      }
    }
  }

  if (flag != 0)
  {
    out[k] = 0x10;
    out[k + 1] = 3;
    k += 2;
  }
  return k;
}

/* ===================================================================================================================== T.30 frame identification */
/**
 * @brief Identify a T.30 frame from the first three bytes of an HDLC frame (address, control, facsimile control field).
 * @param b0  address byte (0xff for T.30 frames)
 * @param b1  control byte (0x03 / 0x13: the FCF is bit reversed in the buffer)
 * @param b2  facsimile control field
 * @return 0xff if the address is not 0xff; otherwise the FCF with the X bit (0x80) cleared when it is above 0x84; bit 15 (0x8000) is set when the control
 *   byte is neither 0x03 nor 0x13 (the id is then taken as is)
 */
unsigned int GetT30FrameIDFromBuffer(unsigned char b0, unsigned char b1, unsigned char b2)
{
  unsigned int v, hi;
  if (b0 != 0xff)
    return 0xff;
  if (b1 == 3 || b1 == 0x13)
  {
    v = aReversedCharsArray[b2];
    hi = 0;
  }
  else
  {
    v = b2;
    hi = 0x8000;
  }
  if (v > 0x84)
    v &= 0x7f;
  return v | hi;
}

/**
 * @brief Name of a T.30 frame id.
 * @param id  a frame id as returned by GetT30FrameIDFromBuffer() (without bit 15)
 * @return the name from fax_t30_frame_names, or "Unknown frame (ID=0x..)" in a static buffer
 * @warning The unknown-frame text lives in one static 64-byte buffer: the result is overwritten by the next call and the function is not reentrant.
 */
const char* GetT30FrameNameByID(unsigned int id)
{
  static char buffer[64]; /* text of the unknown-frame result */
  unsigned int i;
  for (i = 0; i <= 0x23; i++)
    if (fax_t30_frame_names[i].id == id)
      return fax_t30_frame_names[i].name;
  sysdep_sprintf(buffer, "Unknown frame (ID=0x%02x)", id);
  return buffer;
}

/* ===================================================================================================================== shared states */
/** @brief Initialiser of the idle state; nothing to do. */
int _idle_state_init(FaxT30* o)
{
  (void)o;
  return 0;
}

/**
 * @brief Idle state: send silence (n_in samples, or 0xa0 if none were received).
 * @note  Signature and parameters: FaxT30StateFn in fax_t30.h.  Writes the samples to `tx`, which must hold 0xa0 or *n_in samples.
 */
int fax_idle_state(FaxT30* o, short* rx, short* tx, unsigned char* tty_out, const unsigned char* tty_in, int* n_in, int* n_out, int* p7, int* p8)
{
  int n = *n_in;
  (void)o;
  (void)rx;
  (void)tty_out;
  (void)tty_in;
  (void)p7;
  (void)p8;
  if (n > 0)
  {
    int i;
    for (i = 0; i < n; i++)
      tx[i] = 0;
    *n_out = n;
    return 0;
  }
  else
  {
    int i;
    for (i = 0; i < 0xa0; i++)
      tx[i] = 0;
    *n_out = 0xa0;
    return 0;
  }
}

/** @brief Start the send-silence state with a timer of n / 2 blocks (at least 1). @return the timer */
int _send_silence_state_init(FaxT30* o, int n)
{
  int h = n / 2;
  o->state = FAXC1_ST_SEND_SILENCE_STATE;
  if (h == 0)
    h = 1;
  o->tmr = h;
  return h;
}

/** @brief Send-silence state: one block of silence per call; when the timer runs out the command ends with FAX_CLASS1_OK_NO_CARRIER. */
int fax_send_silence_state(FaxT30* o, short* rx, short* tx, unsigned char* tty_out, const unsigned char* tty_in, int* n_in, int* n_out, int* p7, int* p8)
{
  int i;
  (void)rx;
  (void)tty_out;
  (void)tty_in;
  (void)n_in;
  (void)p7;
  (void)p8;
  o->tmr = (int)((unsigned int)o->tmr - 1u);
  for (i = 0; i < 0xa0; i++)
    tx[i] = 0;
  *n_out = 0xa0;
  if (o->tmr == 0)
    o->status = FAX_CLASS1_OK_NO_CARRIER;
  return 0;
}

/** @brief Answer-tone state: one block of the tone per call; after ans_len blocks the HDLC preamble state is started. */
int fax_answer_tone_state(FaxT30* o, short* rx, short* tx, unsigned char* tty_out, const unsigned char* tty_in, int* n_in, int* n_out, int* p7, int* p8)
{
  (void)rx;
  (void)tty_out;
  (void)tty_in;
  (void)n_in;
  (void)p7;
  (void)p8;
  o->tmr = (int)((unsigned int)o->tmr + 1u);
  *n_out = 0xa0;
  if (o->tmr > o->ans_len)
  {
    cHDLCtx_preamble_state_init(o);
    return 0;
  }
  FPM_TONE_generate(o->tone, tx, 0xa0);
  return 0;
}

/* ===================================================================================================================== HDLC transmit */
/**
 * @brief Start the HDLC transmit sequence: re-create the V.21 transmit channel and select the silence-before-preamble state.
 *
 * FAXVMI_control() gets the default FAXVMI_CTL record with the V.21 transmit control record (V21TX_CTL with the re-create flag, bit 9 of word 3, set). The
 * frame input is reopened and the frame counters are cleared.
 */
int cHDLCtx_preamble_state_init(FaxT30* o)
{
  int v[5];
  FaxVmiCtl c;
  memcpy(v, V21TX_CTL, sizeof v);
  v[3] = (int)((unsigned int)v[3] | 0x200u);
  c = FAXVMI_CTL;
  c.mctl = v;
  FAXVMI_control(o->hdlc_tx, &c);
  _handle_hdlc_input_open(o);
  o->tmr = 0;
  o->state = FAXC1_ST_T30_SILENCE_BEFORE_PREAMBLE_STATE;
  o->frameDone = 0;
  o->frameCount = 0;
  o->lastFrame = 0;
  return 0;
}

/**
 * @brief Reopen the HDLC frame input and restart the state timer.
 * @note  The timer store stays after the call: `o->tmr = 0; return f(o);` is the one spot of this file where the generated code depends on strict aliasing.
 */
int _send_hdlc_between_buffer_state_init(FaxT30* o)
{
  int r;
  r = _handle_hdlc_input_open(o);
  o->tmr = 0;
  return r;
}

/**
 * @brief State 2: transmit the queued HDLC frames through the V.21 channel; when the channel reports an empty transmit queue go to state 3.
 * @note  *p8 is set to 0x200 (room for the next frame).
 */
int _send_hdlc_buffer_state(FaxT30* o, short* rx, short* tx, unsigned char* tty_out, const unsigned char* tty_in, int* n_in, int* n_out, int* p7, int* p8)
{
  unsigned short nItems = 0, nSamples;
  FaxVmiSts sts;
  (void)rx;
  (void)tty_out;
  (void)tty_in;
  (void)n_in;
  (void)p7;
  nSamples = (unsigned short)*n_out;
  FAXVMI_process(o->hdlc_tx, o->buf, tx, &nItems, &nSamples);
  sts = FAXVMI_STS;
  FAXVMI_status(o->hdlc_tx, &sts);
  if (sts.txEmpty == 1)
  {
    if (DBG)
      dsplibs_debug_printf("%2d.%02d[sec] End of HDLC buffer transmission\n", o->sec, o->ms);
    o->state = FAXC1_ST_SEND_HDLC_BETWEEN_BUFFER_STATE;
    o->tmr = 0;
    _handle_hdlc_input_open(o);
  }
  *p8 = 0x200;
  return 0;
}

/**
 * @brief State 0: send silence for about 50 ms (timer in samples, above 0x190), then go to the preamble state and report FAX_CLASS1_CONNECT.
 */
int _t30_silence_before_tx_state(FaxT30* o, short* rx, short* tx, unsigned char* tty_out, const unsigned char* tty_in, int* n_in, int* n_out, int* p7, int* p8)
{
  unsigned int t;
  (void)rx;
  (void)tty_out;
  (void)tty_in;
  (void)n_in;
  (void)p7;
  (void)p8;
  _put_silence(tx, 0xa0);
  *n_out = 0xa0;
  t = (unsigned int)o->tmr;
  if (t > 0x190u)
  {
    if (DBG)
      dsplibs_debug_printf("%2d.%02d[sec], Elapsed 50MS second, send preamble\n", o->sec, o->ms);
    o->state = FAXC1_ST_T30_PREAMBLE_STATE;
    o->tmr = 0;
    o->status = FAX_CLASS1_CONNECT;
    t = 0;
  }
  o->tmr = (int)(t + (unsigned int)*n_out);
  return 0;
}

/**
 * @brief State 1: send the HDLC preamble while collecting the first frames from the DTE.
 *
 * DTE bytes (*p8 > 0) go through _handle_hdlc_input(); a completed frame increments frameCount.  After 1 s (timer above 0x1f40 samples) with a complete frame
 * the queued frames are handed to the channel and state 2 is selected; after 5 s (0x9c40) without that the command fails with FAX_CLASS1_ERROR_NO_CARRIER and
 * the object goes idle.  *p8 is set to 0x200.
 */
int _t30_preabmle_state(FaxT30* o, short* rx, short* tx, unsigned char* tty_out, const unsigned char* tty_in, int* n_in, int* n_out, int* p7, int* p8)
{
  unsigned short nItems = 0, nSamples = 0;
  unsigned int elapsed;
  (void)rx;
  (void)tty_out;
  (void)n_in;
  (void)p7;
  *n_out = 0xa0;

  /* collect the DTE bytes into HDLC frames */
  {
    int avail = *p8;
    if (avail > 0)
    {
      int r;
      if (DBG3)
        dsplibs_debug_printf("%2d.%02d[sec] Collect data with %d bytes.\n", o->sec, o->ms, avail);
      r = _handle_hdlc_input(o, tty_in, o->buf, p8);
      o->frameDone = r;
      if (r != 0)
        o->frameCount++;
    }
  }

  /* after one second with a complete frame start sending; give up after five seconds */
  elapsed = (unsigned int)o->tmr;
  if (elapsed > 0x1f40u)
  {
    if (o->frameDone != 0)
    {
      if (DBG)
        dsplibs_debug_printf("At %2d.%02d[sec] Elapsed 1 second, send %d buffers\n", o->sec, o->ms, o->frameCount);
      o->state = FAXC1_ST_SEND_HDLC_BUFFER_STATE;
      nItems = (unsigned short)o->frameCount;
    }
  }
  if (elapsed > 0x9c40u)
  {
    if (DBG)
      dsplibs_debug_printf("At %2d.%02d[sec] Elapsed 5 second in _t30_preabmle_state\n", o->sec, o->ms);
    o->status = FAX_CLASS1_ERROR_NO_CARRIER;
    o->state = FAXC1_ST_IDLE_STATE;
  }

  FAXVMI_process(o->hdlc_tx, o->buf, tx, &nItems, &nSamples);
  o->tmr = (int)((unsigned int)o->tmr + (unsigned int)*n_out);
  *p8 = 0x200;
  return 0;
}

/**
 * @brief State 3: between HDLC frame buffers; wait for the DTE to supply the next frame.
 *
 * When the timer reaches 2 the previous command ends: the object goes idle after the last frame (lastFrame), otherwise FAX_CLASS1_CONNECT is reported. A new
 * frame from the DTE (once the timer is above 1) selects state 2 again. After 250 calls (5 s) the command fails with FAX_CLASS1_ERROR_ON_HOOK. *p8 is set to
 * 0x200.
 */
int _send_hdlc_between_buffer_state(FaxT30* o, short* rx, short* tx, unsigned char* tty_out, const unsigned char* tty_in, int* n_in, int* n_out, int* p7,
                                    int* p8)
{
  unsigned short nItems = 0, nSamples = 0;
  unsigned int timer;
  (void)rx;
  (void)tty_out;
  (void)n_in;
  (void)p7;
  *n_out = 0xa0;

  /* the previous command ends when the timer reaches 2 */
  if ((unsigned int)o->tmr == 2u)
  {
    if (o->lastFrame == 1)
    {
      if (DBG)
        dsplibs_debug_printf("%2d.%02d[sec] Idle state\n", o->sec, o->ms);
      o->status = FAX_CLASS1_OK_NO_CARRIER;
      o->state = FAXC1_ST_IDLE_STATE;
      _idle_state_init(o);
    }
    else
    {
      o->status = FAX_CLASS1_CONNECT;
    }
  }

  /* a new frame from the DTE */
  if (*p8 > 0)
  {
    if ((unsigned int)o->tmr > 1u)
    {
      if (_handle_hdlc_input(o, tty_in, o->buf, p8) != 0)
      {
        o->state = FAXC1_ST_SEND_HDLC_BUFFER_STATE;
        nItems = (unsigned short)(nItems + 1u);
      }
    }
  }

  FAXVMI_process(o->hdlc_tx, o->buf, tx, &nItems, &nSamples);
  timer = (unsigned int)o->tmr + 1u;
  o->tmr = (int)timer;
  if (timer > 0xfau)
  {
    if (DBG)
      dsplibs_debug_printf("At %2d.%02d[sec], CURRENT_STATE_TIMER > 5[sec]\n", o->sec, o->ms);
    o->status = FAX_CLASS1_ERROR_ON_HOOK;
    o->state = FAXC1_ST_IDLE_STATE;
    _idle_state_init(o);
  }
  *p8 = 0x200;
  return 0;
}

/**
 * @brief Restart the V.21 receive channel: flush, reset the framing and take the HDLC mode (FAXVMI_control() with V21RX_CTL, re-create flag set); clear the
 *   timer.
 */
int cHDLCtx_off_init(FaxT30* o)
{
  int v[4], r;
  FaxVmiCtl c;
  memcpy(v, V21RX_CTL, sizeof v);
  v[3] = (int)((unsigned int)v[3] | 0x200u);
  c = FAXVMI_CTL;
  c.flushRing = 1;
  c.rxReset = 1;
  c.newMode = FAXVMI_MODE_HDLC;
  c.mctl = v;
  r = FAXVMI_control(o->hdlc_rx, &c);
  o->tmr = 0;
  return r;
}

/**
 * @brief State 17: transmitter off.  Keep the V.21 receive channel running on the received samples and send silence.
 *
 * The state ends (FAX_CLASS1_OK_NO_CARRIER, idle) when the timer exceeds 15 calls while a carrier is present, or 2 calls without one.
 */
int cHDLCtx_off(FaxT30* o, short* rx, short* tx, unsigned char* tty_out, const unsigned char* tty_in, int* n_in, int* n_out, int* p7, int* p8)
{
  unsigned short nItems = 0, nSamples;
  unsigned int vmiState, limit, timer;
  (void)tty_out;
  (void)tty_in;
  (void)p7;
  (void)p8;
  nSamples = (unsigned short)*n_in;
  o->tmr = (int)((unsigned int)o->tmr + 1u);
  vmiState = FAXVMI_process(o->hdlc_rx, o->buf, rx, &nItems, &nSamples);
  timer = (unsigned int)o->tmr;
  limit = (vmiState & FAXVMI_ST_CARRIER) ? 0xfu : 2u;
  if (timer > limit)
  {
    if (DBG)
      dsplibs_debug_printf("%2d.%02d[sec] End of off state, COUNTER %d\n", o->sec, o->ms, (int)timer);
    o->status = FAX_CLASS1_OK_NO_CARRIER;
    o->state = FAXC1_ST_IDLE_STATE;
    _idle_state_init(o);
  }
  _put_silence(tx, 0xa0);
  *n_out = 0xa0;
  return 0;
}

/* ===================================================================================================================== data modem transmitter */
/** @brief Clear the state timer of the nulls state. */
int _init_tx_nulls_state(FaxT30* o)
{
  o->tmr = 0;
  return 0;
}

/**
 * @brief Release the data modem transmit channel: its config blocks, the VMI channel and the FIFO.
 * @warning o->data_cfg must not be NULL.
 */
void _delete_data_tx_modem(FaxT30* o)
{
  sysdep_free(o->data_cfg->u10.cfg);
  sysdep_free(o->data_cfg);
  o->data_cfg = NULL;
  FAXVMI_delete(o->data_vmi);
  o->data_vmi = NULL;
  if (o->fifo != NULL)
    FIFO_delete(o->fifo);
  o->fifo = NULL;
}

/**
 * @brief Create the data modem transmit channel for a modem selection and choose the first transmit state.
 * @param o    the object; an existing transmit channel is deleted first
 * @param mod  a FaxModemSel value; any other value leaves rate and class at 0
 *
 * The rate and class follow from `mod`; the init_vmi_data_tx_modem function of the class fills the VMI block, FAXVMI_create() builds the channel.  For the
 * short-train V.17 modes a control record with the short-train flag is passed to FAXVMI_control().  The state becomes the scrambled-ones state, or the silence
 * before it when silenceTime is nonzero.
 * @warning If the allocation of the VMI config block fails the function returns without selecting a state.
 */
void _init_transmitter(FaxT30* o, int mod)
{
  int short_train = 0, l2c = 0, rate = 0;

  if (DBG)
    dsplibs_debug_printf("%2d.%02d[sec] Initializing TX modem, MODEM_IDX = %d, silence %d ms\n", o->sec, o->ms, o->modemClass, o->silenceTime);
  if (mod == FAXMOD_V17_7200_SHORT || mod == FAXMOD_V17_9600_SHORT || mod == FAXMOD_V17_12000_SHORT || mod == FAXMOD_V17_14400_SHORT)
  {
    if (DBG)
      dsplibs_debug_printf("Short train in V.17 mode is selected\n");
    short_train = 1;
  }

  /* rate, modem class and the (unused) parameter words per modem selection */
  switch (mod)
  {
  case FAXMOD_V17_14400:
  case FAXMOD_V17_14400_SHORT:
    o->w1230 = 0x30;
    o->w1234 = 0x8000;
    l2c = FAXMC_V17;
    rate = RATE_14400;
    break;
  case FAXMOD_V17_12000:
  case FAXMOD_V17_12000_SHORT:
    o->w1230 = 0x30;
    o->w1234 = 0x8000;
    l2c = FAXMC_V17;
    rate = RATE_12000;
    break;
  case FAXMOD_V17_9600:
  case FAXMOD_V17_9600_SHORT:
    o->w1230 = 0x18;
    o->w1234 = 0x8000;
    l2c = FAXMC_V17;
    rate = RATE_9600;
    break;
  case FAXMOD_V17_7200:
  case FAXMOD_V17_7200_SHORT:
    o->w1230 = 0x18;
    o->w1234 = 0x8000;
    l2c = FAXMC_V17;
    rate = RATE_7200;
    break;
  case FAXMOD_V29_9600:
    o->w1234 = 0x8000;
    o->w1230 = 0x18;
    rate = RATE_9600;
    l2c = FAXMC_V29;
    break;
  case FAXMOD_V29_7200:
    o->w1234 = 0x75a2;
    o->w1230 = 0x18;
    rate = RATE_7200;
    l2c = FAXMC_V29;
    break;
  case FAXMOD_V27_4800:
    o->w1234 = 0x4000;
    o->w1230 = 0xc;
    rate = RATE_4800;
    l2c = FAXMC_V27;
    break;
  case FAXMOD_V27_2400:
    o->w1234 = 0x4000;
    o->w1230 = 6;
    rate = RATE_2400;
    l2c = FAXMC_V27;
    break;
  default:
    break;
  }

  /* delete an existing channel, then create the new one */
  if (o->data_cfg != NULL && o->data_vmi != NULL)
  {
    if (DBG)
      dsplibs_debug_printf("%2d.%02d[sec] New TX Modem... Deleting previous existing one\n", o->sec, o->ms);
    sysdep_free(o->data_cfg->u10.cfg);
    sysdep_free(o->data_cfg);
    o->data_cfg = NULL;
    FAXVMI_delete(o->data_vmi);
    o->data_vmi = NULL;
    if (o->fifo != NULL)
      FIFO_delete(o->fifo);
    o->fifo = NULL;
  }

  if (o->data_vmi == NULL)
  {
    FaxVmiCfg cfg;
    if (o->data_cfg == NULL)
    {
      o->data_cfg = (FaxVmiM*)sysdep_malloc((unsigned int)sizeof(FaxVmiM)); /* 0x18 bytes in 32-bit builds */
      if (o->data_cfg == NULL)
        return; /* allocation failed: nothing is selected */
    }
    fax_init_vmi_data_tx_modem[l2c](o->data_cfg, (unsigned short)rate, 0, NULL);
    cfg = vmi_cfg_of(o->data_cfg);
    o->data_vmi = FAXVMI_create(NULL, &cfg);
  }

  /* short train: pass the modem control record with the short-train flag */
  if (short_train)
  {
    FaxVmiCtl c;
    int v[5];
    unsigned short id = (unsigned short)o->data_cfg->s0e;
    if (id == FAXVMI_ID_V17TX)
    {
      v[0] = (int)(V17TX_CTL[0] & 0xffff0000u) | (int)(unsigned short)rate;
      v[1] = V17TX_CTL[1];
      v[2] = V17TX_CTL[2];
      v[3] = (int)((unsigned int)V17TX_CTL[3] | 0x200u);
      v[4] = short_train;
    }
    else if (id == FAXVMI_ID_V29TX)
    {
      v[0] = (int)((unsigned int)V29TX_CTL[0] & 0xffff0000u) | (int)(unsigned short)rate;
      v[1] = V29TX_CTL[1];
      v[2] = V29TX_CTL[2];
      v[3] = (int)((unsigned int)V29TX_CTL[3] | 0x200u);
      v[4] = 0; /* not part of the record (V29TX_control does not read word 4) */
    }
    else
    {
      v[0] = (int)((unsigned int)V27TX_CTL[0] & 0xffffu) | (int)((unsigned int)(unsigned short)rate << 16);
      v[1] = V27TX_CTL[1];
      v[2] = V27TX_CTL[2];
      v[3] = (int)((unsigned int)V27TX_CTL[3] | 0x200u);
      v[4] = V27TX_CTL[4];
    }
    c = FAXVMI_CTL;
    c.mctl = v;
    FAXVMI_control(o->data_vmi, &c);
  }

  o->dataRate = rate;
  o->modemClass = l2c;
  o->state = (o->silenceTime == 0) ? FAXC1_ST_TX_SCRAMBLED_ONES_STATE : FAXC1_ST_TX_SILENCE_BEFORE_SCRM_ONES;
  o->tmr = 0;
}

/**
 * @brief Start a data transmission: create the transmit channel (_init_transmitter()), create the 0x800-word FIFO and clear the hand-over flags.
 * @return 0
 */
int _tx_scrambled_ones_init(FaxT30* o, int modem)
{
  FaxFifoCfg cfg;
  if (DBG)
    dsplibs_debug_printf("_tx_scrambled_ones_init\n");
  _init_transmitter(o, modem);
  cfg.fill = 0;
  cfg.w0 = fax_fifo_cfg.w0;
  cfg.size = 0x800;
  o->enableDelay = 0;
  o->fifo = FIFO_create(o->fifo, &cfg);
  o->txConnected = 0;
  o->txEnable = 0;
  o->fifoReady = 0;
  o->frameWords = o->dataRate / 400;
  o->dataEnd = 0;
  fax_cDATAtx_counter = 0;
  return 0;
}

/**
 * @brief State 18: send silence (the timer counts 20 ms per call) until silenceTime ms have passed, then select the scrambled-ones state.  *p8 is set to 0.
 */
int _tx_silence_before_scrm_ones(FaxT30* o, short* rx, short* tx, unsigned char* tty_out, const unsigned char* tty_in, int* n_in, int* n_out, int* p7, int* p8)
{
  (void)rx;
  (void)tty_out;
  (void)tty_in;
  (void)n_in;
  (void)p7;
  o->tmr = (int)((unsigned int)o->tmr + 0x14u);
  if (DBG)
    dsplibs_debug_printf("Tx silence before scrambled ones ...\n");
  _put_silence(tx, 0xa0);
  *n_out = 0xa0;
  if ((unsigned int)o->tmr >= (unsigned int)o->silenceTime)
    o->state = FAXC1_ST_TX_SCRAMBLED_ONES_STATE;
  *p8 = 0;
  return 0;
}

/**
 * @brief State 11: the DTE queue ran empty; keep the modem fed with nulls until the DTE supplies data again.
 *
 * New DTE data (*p8 > 0) goes through the FIFO and selects state 10 (_tx_data_state).  After 250 calls (5 s) without data the command fails with
 * FAX_CLASS1_ERROR_ON_HOOK and the object goes idle.  *p8 is set to the free room of the FIFO - 1.
 */
int _tx_nulls_state(FaxT30* o, short* rx, short* tx, unsigned char* tty_out, const unsigned char* tty_in, int* n_in, int* n_out, int* p7, int* p8)
{
  unsigned short nItems, nSamples;
  int nWords;
  (void)rx;
  (void)tty_out;
  (void)n_in;
  (void)p7;
  o->tmr = (int)((unsigned int)o->tmr + 1u);
  if ((unsigned int)o->tmr > 0xfau)
  {
    if (DBG)
      dsplibs_debug_printf("CURRENT_STATE_TIMER > FIVE_SECONDS in _tx_nulls_state\n");
    o->state = FAXC1_ST_IDLE_STATE;
    o->status = FAX_CLASS1_ERROR_ON_HOOK;
  }

  /* new DTE data: queue it and go back to the data state */
  nWords = *p8;
  if (nWords > 0)
  {
    int w;
    if (DBG)
      dsplibs_debug_printf("At %2d.%02d[sec] Back to TX_DATA_STATE in _tx_nulls_state\n", o->sec, o->ms);
    o->state = FAXC1_ST_TX_DATA_STATE;
    _handle_data_input(o, tty_in, o->buf, p8);
    w = FIFO_write(o->fifo, o->buf, (unsigned short)*p8);
    if (*p8 > (int)(unsigned short)w)
    {
      if (DBG)
        dsplibs_debug_printf("Fifo is full in _tx_nulls_state\n");
    }
    nWords = (int)(unsigned short)FIFO_read(o->fifo, o->buf, (unsigned short)o->frameWords);
    *p8 = nWords;
    if (nWords < o->frameWords)
    {
      if (DBG)
        dsplibs_debug_printf("class1 object fifo under run in _tx_nulls_state !!!\n");
      nWords = *p8;
    }
  }

  nItems = (unsigned short)nWords;
  nSamples = (unsigned short)*n_out;
  FAXVMI_process(o->data_vmi, o->buf, tx, &nItems, &nSamples);
  *p8 = (int)o->fifo->size - (int)o->fifo->count - 1;
  return 0;
}

/**
 * @brief State 9: send scrambled ones while the modem trains; switch to data when transmission is enabled and a frame is queued.
 *
 * The first call after a reset reports FAX_CLASS1_CONNECT.  DTE bytes are queued in the FIFO; once the channel reports the data state (connect) enableDelay is
 * set to 2, counted down to txEnable.  When txEnable and fifoReady are both set a frame is read from the FIFO and state 10 is selected.  *p8 is set to the free
 * room of the FIFO - 1.
 */
int _tx_scrambled_ones_state(FaxT30* o, short* rx, short* tx, unsigned char* tty_out, const unsigned char* tty_in, int* n_in, int* n_out, int* p7, int* p8)
{
  unsigned short nItems, nSamples;
  unsigned int vmiState;
  int nWords, frameLen, i, prevCount;
  (void)rx;
  (void)tty_out;
  (void)n_in;
  (void)p7;
  if (DBG3)
    dsplibs_debug_printf("cDATAtx_counter %d\n", fax_cDATAtx_counter);
  prevCount = fax_cDATAtx_counter;
  fax_cDATAtx_counter = prevCount + 1;
  if (prevCount == 0)
    o->status = FAX_CLASS1_CONNECT;

  /* count down to the transmit enable after the connect */
  nWords = o->enableDelay;
  if (nWords > 0)
  {
    nWords -= 1;
    o->enableDelay = nWords;
    if (nWords == 0)
    {
      if (DBG)
        dsplibs_debug_printf("At %2d.%02d[sec] ENABLE_TRANSMIT in _tx_scrambled_ones_state\n", o->sec, o->ms);
      o->txEnable = 1;
    }
  }

  /* queue the DTE bytes */
  if (*p8 > 0)
  {
    int w;
    _handle_data_input(o, tty_in, o->buf, p8);
    w = FIFO_write(o->fifo, o->buf, (unsigned short)*p8);
    if (*p8 > (int)(unsigned short)w)
    {
      if (DBG)
        dsplibs_debug_printf("At %2d.%02d[sec] Fifo is full in _tx_scrambled_ones_state\n", o->sec, o->ms);
    }
    o->fifoReady = ((int)o->fifo->count >= o->frameWords);
    frameLen = o->frameWords;
  }
  else
  {
    frameLen = o->frameWords;
  }

  /* a frame of ones, or the first queued frame once the transmission is enabled */
  for (i = 0; i < frameLen; i++)
    o->buf[i] = 0xff;
  *p8 = frameLen;
  nWords = frameLen;
  if (o->txEnable != 0 && o->fifoReady != 0)
  {
    o->state = FAXC1_ST_TX_DATA_STATE;
    nWords = (int)(unsigned short)FIFO_read(o->fifo, o->buf, (unsigned short)o->frameWords);
    *p8 = nWords;
    if (nWords < o->frameWords)
    {
      if (DBG)
        dsplibs_debug_printf("class1 object fifo under run in _tx_scrambled_ones_state !!!\n");
      nWords = *p8;
    }
  }

  nItems = (unsigned short)nWords;
  nSamples = (unsigned short)*n_out;
  vmiState = FAXVMI_process(o->data_vmi, o->buf, tx, &nItems, &nSamples);
  if (o->txConnected == 0 && (vmiState & FAXVMI_ST_DATA) != 0)
  {
    if (DBG)
      dsplibs_debug_printf("At %2d.%02d[sec] Tx connect\n", o->sec, o->ms);
    o->enableDelay = 2;
    o->txConnected = 1;
  }

  *p8 = (int)o->fifo->size - (int)o->fifo->count - 1;
  return 0;
}

/**
 * @brief State 10: transmit data.  Queue the DTE bytes, send one frame of frameWords words from the FIFO through the data modem channel.
 *
 * When the channel reports an empty transmit queue: if the DTE's last byte was nonzero (DLE ETX seen) the transmission ends with FAX_CLASS1_OK_NO_CARRIER,
 * else the object continues with nulls (state 11, FAX_CLASS1_CONNECT). A FIFO under-run is only reported in the diagnostics. *p8 is set to the free room of
 * the FIFO - 1.
 */
int _tx_data_state(FaxT30* o, short* rx, short* tx, unsigned char* tty_out, const unsigned char* tty_in, int* n_in, int* n_out, int* p7, int* p8)
{
  unsigned short nSamples;
  short nItems;
  unsigned int vmiState;
  int avail, want;
  (void)rx;
  (void)tty_out;
  (void)n_in;
  (void)p7;
  nSamples = (unsigned short)*n_out;
  if (DBG3)
    dsplibs_debug_printf("cDATAtx_counter %d\n", fax_cDATAtx_counter);
  fax_cDATAtx_counter++;

  /* queue the DTE bytes */
  avail = *p8;
  if (avail > 0)
  {
    short w;
    _handle_data_input(o, tty_in, o->buf, p8);
    w = (short)FIFO_write(o->fifo, o->buf, (unsigned short)*p8);
    want = *p8;
    if ((int)w < want)
    {
      if (DBG)
        dsplibs_debug_printf("At %2d.%02d[sec] Fifo is full in _tx_data_state(%d=>%d>%d)\n", o->sec, o->ms, avail, want, (int)w);
    }
  }

  /* one frame to the modem */
  nItems = (short)FIFO_read(o->fifo, o->buf, (unsigned short)o->frameWords);
  want = o->frameWords;
  if ((int)nItems < want)
  {
    if (DBG)
      dsplibs_debug_printf("At %2d.%02d[sec] fifo underrun in _tx_data_state, count %d < %d\n", o->sec, o->ms, (int)nItems, want);
  }
  vmiState = FAXVMI_process(o->data_vmi, o->buf, tx, (unsigned short*)&nItems, &nSamples);

  /* the modem ran out of data: end the transmission or continue with nulls */
  if (vmiState & FAXVMI_ST_TX_EMPTY)
  {
    if (o->lastByte != 0)
    {
      if (DBG)
        dsplibs_debug_printf("At %2d.%02d[sec] Queue underrun, Stop TX\n", o->sec, o->ms);
      o->state = FAXC1_ST_IDLE_STATE;
      o->status = FAX_CLASS1_OK_NO_CARRIER;
    }
    else
    {
      if (DBG)
        dsplibs_debug_printf("At %2d.%02d[sec] Queue underrun, Continue NULLS\n", o->sec, o->ms);
      o->state = FAXC1_ST_TX_NULLS_STATE;
      o->tmr = 0;
      o->status = FAX_CLASS1_CONNECT;
    }
  }

  if (vmiState & FAXVMI_ST_QUEUE_FULL)
  {
    if (DBG)
      dsplibs_debug_printf("At %2d.%02d[sec] Queue is full in _tx_data_state\n", o->sec, o->ms);
  }

  *p8 = (int)o->fifo->size - (int)o->fifo->count - 1;
  return 0;
}
