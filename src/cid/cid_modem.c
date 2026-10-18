/**
 * @file cid_modem.c
 * @brief Caller-ID FSK receiver, result formatting, the cid_* object and the entry points used by the host.
 *
 * Contents.
 *   - cid_modem(): one frame of the FSK receiver: DC removal, 1200 Hz seizure-tone gate (CID_MTD_detect()), resampling of 8000 Hz input to 7200 Hz with the
 *     polyphase filter of fpm/fpm_mrf.c, automatic gain, CID_FSD_demodulate() (cid_detect.c), a byte framer (seizure of 16 mark bits, then start bit, 8 data
 *     bits least significant first, stop bit) that collects the message in CidFsk::msg, and the message checksum;
 *   - create_cid(), create_cid_dtmf(), reset_cid(): creation and reset of the two detector objects;
 *   - data_formatted_output(), data_unformatted_output(), data_raw(): the result strings ("DATE = ..", "TIME = ..", "NMBR = ..", "NAME = ..", "MESG = .."
 *     or the message as hexadecimal text);
 *   - cid_create(), cid_delete(), cid_reset(), cid_freq_sampl(), cid_threshold(), cid_value(), cid_progress(), cid_get_strings(): the cid_* object (CidObj),
 *     which holds one or both detectors according to its mode and collects the input into frames of 160 (8000 Hz) or 192 (9600 Hz) samples;
 *   - CID_create(), CID_delete(), CID_process(): the entry points slmodemd calls.  CID_create() always asks for the FSK-only mode (CID_MODE_FSK).
 *
 * Data flow: CID_process() -> cid_progress() -> cid_modem() / dtmf_modem() per frame -> on success cid_get_strings() -> modem_send_to_tty().
 * The objects are in cid.h, the coefficient tables in cid_tables.c.
 *
 * Behaviour worth knowing (see also the notes at the functions): data_formatted_output() writes the header of a second or later "MESG = " string over the
 * previous hex string; create_cid_dtmf() leaves some members uninitialised; a message length above 0x75 makes the checksum read behind the message buffer
 * (the bytes are read from the object itself and 0 behind it); cid_modem() with n == 0 returns 1 and clamps frames longer than 206 samples; the object
 * creators return NULL when an allocation fails; the parameter searches of data_formatted_output() stop after 4096 steps.
 */
#include <limits.h>
#include <stddef.h>

#include "cid.h"
#include "core/dsp_access.h"
#include "cid_tables.h"
#include "fpm/fpm.h"
#include "fpm/fpm_tables.h"
#include "core/dsp_constants.h" /* SAMPLE_RATE_* */
#include "core/dsp_host.h"

#define CID_MODEM_BUF 206 /* size of the local copy of the frame in cid_modem(): longer frames are truncated (the callers use 160 / 192) */
#define CID_SEARCH_MAX 4096

/*
 * Byte image of the message area of CidFsk: msg[i] for -0x4a <= i < 0x88 covers `tc`, out[], msg[] and the framer state (they are contiguous on every host,
 * asserted below); everything outside the object reads as 0.
 */
_Static_assert(offsetof(CidFsk, s15e) + sizeof(short) - offsetof(CidFsk, tc) == 0x160 - 0x8e, "CidFsk tail is contiguous");
static int mb(const CidFsk* f, int i)
{
  const unsigned char* base = (const unsigned char*)&f->tc;
  int j = i + (int)MEMBER_DISTANCE(CidFsk, tc, msg);
  if (j < 0 || j >= (int)(0x160 - 0x8e))
    return 0;
  return base[j];
}
static int mbs(const CidFsk* f, int i) { return (signed char)mb(f, i); }

/* ------------------------------------------------------------------------------------------------ cid_modem */
/**
 * @brief Run one frame through the FSK receiver.
 * @param in  frame of samples (not modified)
 * @param n   number of samples; frames longer than 206 are truncated, 0 returns 1
 * @param f   FSK receiver state
 * @return 1 = still waiting for the seizure tone; 2 = receiving; 3 = message complete and checksum good; -1 = failure (checksum bad, or a message length
 *         of 0 or less)
 *
 * The frame is copied, and the block mean is removed: dc = 0.8 * (mean of this frame) + 0.2 * (previous dc), which is subtracted from the samples.
 * Seizure tone.  With tdiv = 0xa000 / n (0xc000 / n at 9600 Hz) the tone-counter limits are 18 * tdiv / 256 (upper) and 12 * tdiv / 256 (lower); they are
 * 18 and 12 for the 160 / 192-sample frames.  While the tone counter `tc` is below the upper limit CID_MTD_detect() is run: a frame with the 1200 Hz tone
 * adds `toneInc` (9) to `tc`, a frame without it clears `tc`.  The function returns 1 until `tc` has reached the upper limit.
 * Demodulation.  Once `tc` is above the lower limit the frame is resampled from 8000 to 7200 Hz (not at 9600 Hz) and
 * multiplied by the gain `agc`; the gain is re-estimated from the mean magnitude only while `tc` is between the two limits.  After the tone is confirmed,
 * `tc` is incremented per frame, and when the byte count reaches the message length (type, length byte, data, checksum; 0x73 at most) the checksum
 * (the two's complement of the sum of all bytes before it) is checked.  Otherwise CID_FSD_demodulate() produces bits, which the framer turns into
 * bytes.
 * @bug   `tc` keeps counting once the tone is confirmed and is tested as a signed short.  After 32768 - limit frames (about 11 minutes of 160-sample
 *        frames) it reads as negative, which restarts the seizure-tone search, and a message that has not completed by then is lost.
 * @bug   More than 36 bits from one CID_FSD_demodulate() call run past CidFsk::out into msg[] (see there); the bits are read back from that memory, but the
 *        first bytes of a message already collected are overwritten.
 * @warning A message length byte above 0x75 makes the checksum read behind the msg buffer (the bytes of the object itself, 0 behind it).
 */
int cid_modem(const short* in, unsigned short n, CidFsk* f)
{
  short buf[CID_MODEM_BUF];
  int nn = n > CID_MODEM_BUF ? CID_MODEM_BUF : (int)n;
  int i, sum, tdiv, thrHi, thrLo, n2;
  short dc, tcs, bits;
  unsigned short fs = f->fs;

  if (nn == 0)
    return 1;

  /* copy the frame and remove the DC estimate */
  for (i = 0; i < nn; i++)
    buf[i] = in[i];
  sum = 0;
  for (i = 0; i < nn; i++)
    sum += buf[i];
  dc = (short)(((sum / nn) * 0xcce >> 12) + ((int)f->dc * 0xcce >> 14));
  f->dc = dc;
  for (i = nn - 1; i >= 0; i--)
    buf[i] = (short)(buf[i] - dc);

  /* the tone-counter limits scale with the frame length */
  tdiv = (int)((fs == SAMPLE_RATE_9600 ? 0xc000u : 0xa000u) / (unsigned)nn);
  if (tdiv <= 0)
    tdiv = 1;
  thrHi = tdiv * 18 >> 8;
  thrLo = tdiv * 12 >> 8;

  /* seizure-tone gate */
  tcs = (short)f->tc;
  if (tcs < thrHi)
  {
    if (CID_MTD_detect(buf, (short)nn, f) == 0)
    {
      f->tc = (unsigned short)(f->tc + (unsigned short)f->toneInc);
      if (dsplibs_debug_level > 1)
        dsplibs_debug_printf("Tone 1200 Detected\n");
      tcs = (short)f->tc;
    }
    else
    {
      f->tc = 0;
      tcs = 0;
    }
  }

  /* tone present: resample and apply the gain */
  n2 = nn;
  if (tcs > thrLo)
  {
    int j, g;
    n2 = (short)nn;
    if (f->fs != SAMPLE_RATE_9600)
    {
      n2 = (short)FPM_MRF_filter(&f->mrf, buf, buf, (short)nn);
      tcs = (short)f->tc;
    }
    if (tcs <= thrHi && thrLo < tcs)
    { /* the gain follows the signal level only while the tone counter is inside the window */
      int mag = 0;
      short m, s;
      unsigned v;
      if ((short)n2 > 0)
      {
        for (j = 0; (short)j < (short)n2; j = (short)(j + 1))
          mag += buf[j] < 0 ? -buf[j] : buf[j];
        if (mag == 0)
          mag = 1;
      }
      else
      {
        mag = 1;
      }
      FPM_div_32((unsigned)mag, &m, &s);
      v = ((unsigned)(unsigned short)m * 25u) >> ((31u - (unsigned)(unsigned short)s) & 31u);
      g = (int)(v + (unsigned)f->agc) >> 1;
      f->agc = g != 0 ? g : 1;
    }
    g = f->agc;
    for (j = 0; (short)j < (short)n2; j = (short)(j + 1))
      buf[j] = (short)(buf[j] * g);
  }
  if (tcs < thrHi)
    return 1;

  /* the tone is confirmed: count the frame and test for a complete message */
  f->tc = (unsigned short)(tcs + 1);
  {
    int limit = 0x73;
    short cnt = f->count;
    if (cnt > 2)
    {
      limit = (short)(f->msg[1] + 3);
      if (limit <= 0x73)
      {
        if (limit <= 3)
          return -1;
      }
      else
      {
        limit = 0x73;
      }
    }
    if (cnt >= limit)
    { /* the message is complete: checksum */
      int len = f->msg[1] + 2, k;
      unsigned char s = 0;
      for (k = 0; k < len; k++)
        s = (unsigned char)(s + mb(f, k));
      return mb(f, f->msg[1] + 2) == (unsigned char)-s ? 3 : -1;
    }
  }

  /* demodulate.  The demodulator writes one 16-bit value per bit to out[]; more than 36 bits (a damaged state, or noise) run on into msg[], and the bits are
   * read back from there, so they are read through a pointer, not as out[i] (out[] and msg[] are contiguous on every host). */
  bits = (short)CID_FSD_demodulate(buf, f->out, (short)n2, f);

  /* byte framer, one bit per step */
  for (i = 0; i < bits; i++)
  {
    int bit = (short)((const unsigned short*)&f->out)[i] & 1;
    unsigned thr = f->cfgThr;
    switch ((unsigned short)f->state)
    {
    case CIDFSK_SEIZURE: /* mark bits count up, space bits down; 16 mark bits in a row end the seizure */
      f->ones = (short)(bit == 1 ? f->ones + 1 : f->ones - 1);
      if (f->ones > 0xf)
      {
        f->state = CIDFSK_FIRST_START;
        f->acc = 0;
        f->bitIdx = 0;
      }
      break;
    case CIDFSK_FIRST_START: /* wait for the first start bit: a run of thr + 1 space bits */
      f->bitIdx = (short)(bit != 0 ? 0 : f->bitIdx + 1);
      if ((int)f->bitIdx == (int)(thr + 1u))
      {
        f->state = CIDFSK_DATA;
        f->bitIdx = (short)thr;
      }
      break;
    case CIDFSK_STOP: /* stop bit expected: the start bit of the next byte is a space */
      if (bit == 0)
      {
        f->state = CIDFSK_DATA;
        f->acc = 0;
        f->bitIdx = 0;
      }
      break;
    default:
    { /* data bits, least significant first */
      int idx = f->bitIdx;
      unsigned a = (unsigned short)f->acc | ((unsigned)bit << (idx & 31));
      f->bitIdx = (short)(idx + 1);
      f->acc = (short)a;
      if ((short)(idx + 1) == 8)
      {
        int c = f->count;
        f->count = (short)(c + 1);
        if (c >= 0 && c < (int)sizeof f->msg)
          f->msg[c] = (unsigned char)a;
        f->state = CIDFSK_STOP;
      }
      break;
    }
    }
  }

  return 2;
}

/* ------------------------------------------------------------------------------------------------ create / reset */
/**
 * @brief Reset the FSK receiver to its start state (no tone, no message, empty filters) and (re)configure its polyphase filter.
 * @param f  FSK receiver; `fs`, `cfgThr` and `toneInc` are kept
 *
 * The filter is set to 9 phases with step 10 (8000 Hz in, 7200 Hz out) with the coefficients cid_MRF_FILT; its history is allocated when `mrf.buf` is NULL.
 * Only out[0] and out[1] are cleared, not the rest of the bit buffer.
 */
void reset_cid(CidFsk* f)
{
  int cfg[4], i;

  if (dsplibs_debug_level > 1)
    dsplibs_debug_printf("FSK CID Reset !\n");

  f->s04 = 0;
  f->s08 = 0;
  f->dc = 0;
  f->agc = 0;

  /* polyphase filter: 9 phases, step 10; the table is set directly because it is not in the table-handle registry */
  cfg[0] = (10 << 16) | 9;
  cfg[1] = 0;
  cfg[2] = (FPM_MRF_CFG_raw[2] & ~0xffff) | 0x5a;
  cfg[3] = FPM_MRF_CFG_raw[3];
  FPM_MRF_init(&f->mrf, cfg, f->mrf.buf == NULL ? 1 : 0);
  f->mrf.coefs = cid_MRF_FILT;

  /* demodulator and framer state */
  f->pos = 0;
  f->cur = 0;
  f->lpfPos = 0;
  for (i = 0; i < 17; i++)
    f->lpf[i] = 0;
  for (i = 0; i < 5; i++)
    f->hist[i] = 0;
  f->nPos = 0;
  f->nNeg = 0;
  f->iirMtd1[0] = f->iirMtd1[1] = 0;
  f->iirMtd2[0] = f->iirMtd2[1] = 0;
  for (i = 0; i < 4; i++)
    f->z86[i] = 0;
  f->tc = 0;
  f->out[0] = 0;
  f->out[1] = 0;
  f->s150 = 0;
  f->s152 = 0;
  f->ones = 0;
  f->state = CIDFSK_SEIZURE;
  f->count = 0;
  f->thr = 0;
  f->mean = 0;
  f->negAcc = 0;
  f->posAcc = 0;
  f->pad_0074[0] = f->pad_0074[1] = f->pad_0074[2] = f->pad_0074[3] = 0;
  f->run = 0;
  f->runAlt = 0;
  f->quiet = 0;
  f->acc = 0;
  f->bitIdx = 0;
  for (i = 0; i < (int)sizeof f->msg; i++)
    f->msg[i] = 0;
}

/**
 * @brief Create an FSK receiver (8000 Hz, framer threshold 2, tone increment 9) and reset it.
 * @param f  object to initialise, or NULL to allocate one
 * @return the receiver, or NULL if the allocation failed
 */
CidFsk* create_cid(CidFsk* f)
{
  if (f == NULL)
  {
    f = (CidFsk*)sysdep_malloc(sizeof(CidFsk));
    if (f == NULL)
      return NULL;
  }
  f->mrf.buf = NULL;
  if (dsplibs_debug_level > 1)
    dsplibs_debug_printf("FSK CID Creating \n");
  reset_cid(f);
  f->fs = SAMPLE_RATE_8000;
  f->cfgThr = 2;
  f->toneInc = 9;
  if (dsplibs_debug_level > 1)
    dsplibs_debug_printf("FSK CID  Setting  Fs = %d   Threshold = %d !\n", (int)f->fs, 2);
  return f;
}

/**
 * @brief Create a DTMF detector (8000 Hz, mode 0) in its reset state.
 * @param d  object to initialise, or NULL to allocate one
 * @return the detector, or NULL if the allocation failed
 * @bug   `iirLow1` is not initialised (reset_dtmf() does not clear it either), so the first DTMF_MTD_detect() starts its first-group pre-filter from heap
 *        garbage in a freshly allocated object.  `halfFlag`, `blk`, `stash` and `digits[16 .. 19]` are not initialised either, but dtmf_modem() writes the
 *        first three before it reads them and cid_get_strings() returns only digits[0 .. 15].
 */
CidDtmf* create_cid_dtmf(CidDtmf* d)
{
  int i;

  if (d == NULL)
  {
    d = (CidDtmf*)sysdep_malloc(sizeof(CidDtmf));
    if (d == NULL)
      return NULL;
  }
  d->fs = SAMPLE_RATE_8000;
  d->mode = 0;
  if (dsplibs_debug_level > 1)
    dsplibs_debug_printf("DTMF Cid Creating   Fs = %d   Threshold = %d !\n", (int)d->fs, 0);

  /* the reset state of reset_dtmf() */
  if (dsplibs_debug_level > 1)
    dsplibs_debug_printf("DTMF CID Reset !\n");
  d->state0 = 0;
  d->samples = 0;
  d->cnt = 0;
  d->lastDig = USHRT_MAX;
  d->minEn = 1;
  d->fails = 0;
  d->nDig = -1;
  d->blkp = d->blk;
  d->s356 = 0;
  d->s354 = 0;
  d->iirBand[1] = 0;
  d->iirBand[0] = 0;
  d->s35c[1] = 0;
  d->iirLow0[1] = 0;
  d->w004 = 0;
  d->s008 = 0;
  d->s35c[0] = 0;
  d->iirLow0[0] = 0;
  for (i = 0; i < 8; i++)
  {
    d->iirMtd[i][1] = 0;
    d->iirMtd[i][0] = 0;
  }
  for (i = 0; i < 16; i++)
    d->digits[i] = 0;
  d->phase = CIDDTMF_LISTEN;
  return d;
}

/* ------------------------------------------------------------------------------------------------ result strings */
static const char hexd[] = "0123456789abcdef";

/* Write the hexadecimal text of the message part that starts at byte `base` (type, length, data: the length byte + 2 bytes, at most 0xf5) to out. */
static void hex_out(const CidFsk* f, int base, char* out)
{
  int n = mbs(f, base + 1) + 2, k;
  if (n > 0xf5)
    n = 0xf5;
  if (n < 0)
    n = 0;
  for (k = 0; k < n; k++)
  {
    int b = mb(f, base + k);
    out[2 * k] = hexd[b >> 4];
    out[2 * k + 1] = hexd[b & 15];
  }
  out[2 * k] = 0;
}

/**
 * @brief The received message as lowercase hexadecimal text.
 * @param f    FSK receiver holding the message
 * @param out  output string (room for 2 * 0xf5 + 1 characters)
 */
void data_unformatted_output(const CidFsk* f, char* out) { hex_out(f, 0, out); }

/**
 * @brief The same hexadecimal text for a message given by its address.
 * @param msg  message bytes (type, length, data ...)
 * @param out  output string (room for 2 * 0xf5 + 1 characters)
 */
void data_raw(const unsigned char* msg, char* out)
{
  int n = (signed char)msg[1] + 2, k;
  if (n > 0xf5)
    n = 0xf5;
  if (n < 0)
    n = 0;
  for (k = 0; k < n; k++)
  {
    out[2 * k] = hexd[msg[k] >> 4];
    out[2 * k + 1] = hexd[msg[k] & 15];
  }
  out[2 * k] = 0;
}

/* The position of the first parameter of type `type` in a multiple-data-message-format (MDMF) message, or -1.  The walk stops after CID_SEARCH_MAX steps. */
static short mdmf_find(const CidFsk* f, int type)
{
  short cx = (short)mbs(f, 1), p = 2;
  int steps = 0;
  while (p < cx)
  {
    if (mb(f, p) == type)
      return p;
    p = (short)(mbs(f, p + 1) + p + 2);
    if (++steps > CID_SEARCH_MAX)
      break;
  }
  return -1;
}

static void put7(char* o, const char* s) /* the 7 character header "XXXX = " */
{
  int i;
  for (i = 0; i < 7; i++)
    o[i] = s[i];
}

/* Append the data bytes of the parameter at p (type, length, data) at dst[idx ..] and return the new index. */
static int put_param(const CidFsk* f, short p, char* dst, int idx)
{
  int len = mb(f, p + 1) + 2, k;
  if (len > 0xff)
    len = 0xff;
  for (k = 2; k < len; k++)
    dst[idx++] = (char)mb(f, p + k);
  return idx;
}

/**
 * @brief Format the received message as a list of NUL-separated strings.
 * @param f    FSK receiver holding the message (nothing is written when no byte was received)
 * @param out  output buffer (CidObj::str, 600 bytes); the strings follow each other, each terminated by NUL
 *
 * A single-data message (type other than 0x80) gives "DATE = " + 4 digits, "TIME = " + 4 digits and "NMBR = " + the rest.  A multiple-data message (type
 * 0x80) is searched for the parameters of type 1 (date and time, 8 digits: the first four are written as DATE and the next four as TIME), 2 (number,
 * "NMBR = ") and 7 (name, "NAME = "); every other parameter with more than one byte of data becomes a "MESG = " string with the type, length and data in
 * hexadecimal text.
 * @bug   A multiple-data message without a date and time parameter (type 1) takes DATE and TIME from the bytes at message index 1 .. 8 (the position -1
 *        is not checked).
 * @bug   The parameter walks add the SIGNED length byte plus 2 to the index: a parameter length of -2 keeps the walk in place and a lower one moves it
 *        back, so a checksum-valid message could make it endless; the walks stop after 4096 steps and the function returns (or the search reports none).
 * @bug   The second and later "MESG = " strings are written with the index not advanced, so each header lands over the hexadecimal text of the previous
 *        string.  CID_process() sends only the first four strings.
 */
void data_formatted_output(const CidFsk* f, char* out)
{
  if (f->count == 0)
    return;
  if (mb(f, 0) != 0x80)
  { /* SDMF: date + time + number */
    int len, k, idx = 7;
    put7(out, "DATE = ");
    for (k = 0; k < 4; k++)
      out[7 + k] = (char)mb(f, 2 + k);
    out[11] = 0;
    put7(out + 12, "TIME = ");
    for (k = 0; k < 4; k++)
      out[19 + k] = (char)mb(f, 6 + k);
    out[23] = 0;
    put7(out + 24, "NMBR = ");
    len = mb(f, 1) + 2;
    if (len > 0xff)
      len = 0xff;
    for (k = 10; k < len; k++)
      out[24 + idx++] = (char)mb(f, k);
    out[24 + idx] = 0;
    return;
  }
  else
  { /* MDMF: the parameters by type */
    short pDate = mdmf_find(f, 1), pName = mdmf_find(f, 7), pNum = mdmf_find(f, 2);
    char* base;
    int idx, k, steps = 0;

    put7(out, "DATE = ");
    for (k = 0; k < 4; k++)
      out[7 + k] = (char)mb(f, pDate + 2 + k);
    out[11] = 0;
    put7(out + 12, "TIME = ");
    for (k = 0; k < 4; k++)
      out[19 + k] = (char)mb(f, pDate + 6 + k);
    out[23] = 0;

    /* number and name */
    base = out + 24;
    put7(base, "NMBR = ");
    idx = 7;
    if (pNum > 0)
      idx = put_param(f, pNum, base, idx);
    base[idx] = 0;
    if (pName > 0)
    {
      base = base + idx + 1;
      put7(base, "NAME = ");
      idx = put_param(f, pName, base, 7);
      base[idx] = 0;
    }

    /* any other parameter (a type other than 1, 7, 2): "MESG = " and the bytes of type, length and data in hex.  The index stays 7, so the header of the next
     * one is written over the hex string of the previous one. */
    {
      short si = (short)mbs(f, 1), eb = 2, pos = -1;
      while (eb < si)
      {
        int cl = mb(f, eb);
        if (cl != 1 && cl != 7 && cl != 2)
        {
          if (eb != 0)
          {
            pos = eb;
            break;
          }
          eb = (short)(mbs(f, 1) + 2);
        }
        else
        {
          eb = (short)(mbs(f, eb + 1) + eb + 2);
        }
        if (++steps > CID_SEARCH_MAX)
          return;
      }
      while (pos > 0)
      {
        short next = -1;
        if ((unsigned char)mb(f, pos + 1) > 1)
        {
          base = base + idx + 1;
          put7(base, "MESG = ");
          idx = 7;
          hex_out(f, pos, base + 7);
        }
        si = (short)mbs(f, 1);
        eb = pos;
        if (eb < si)
        {
          for (;;)
          {
            int cl = mb(f, eb);
            if (cl != 1 && cl != 7 && cl != 2)
            {
              if (eb != pos)
              {
                next = eb;
                break;
              }
              eb = (short)(mbs(f, pos + 1) + pos + 2);
            }
            else
            {
              eb = (short)(mbs(f, eb + 1) + eb + 2);
            }
            if (!(eb < si))
              break;
            if (++steps > CID_SEARCH_MAX)
              return;
          }
        }
        pos = next;
      }
    }
  }
}

/* ------------------------------------------------------------------------------------------------ the CID object */
/**
 * @brief Create a CID object (or re-create the detectors of an existing one).
 * @param c     existing object, or NULL to allocate one
 * @param fmt   result format: 2 = hexadecimal text, anything else = formatted strings
 * @param mode  0 = FSK only, 1 = DTMF only, above 1 = automatic (both detectors, CID_MODE_AUTO)
 * @return the object, or NULL if an allocation failed (what was allocated is freed)
 */
CidObj* cid_create(CidObj* c, int fmt, int mode)
{
  if (c == NULL)
  {
    c = (CidObj*)sysdep_malloc(sizeof(CidObj));
    if (c == NULL)
      return NULL;
    c->mode = mode;
    c->fsk = NULL;
    c->dtmf = NULL;
    c->fill = 0;
    if (mode > 1)
    {
      if (dsplibs_debug_level > 1)
        dsplibs_debug_printf("\n AUTOMATIC MODULATION MODE \n");
      c->mode = CID_MODE_AUTO;
    }
  }

  /* the detectors the mode needs */
  if (c->mode != CID_MODE_FSK)
  {
    CidDtmf* d = create_cid_dtmf(c->dtmf);
    c->dtmf = d;
    if (d == NULL)
    {
      sysdep_free(c);
      return NULL;
    }
  }
  if (c->mode != CID_MODE_DTMF)
  {
    CidFsk* k = create_cid(c->fsk);
    c->fsk = k;
    if (k == NULL)
    {
      if (c->dtmf != NULL)
        sysdep_free(c->dtmf);
      sysdep_free(c);
      return NULL;
    }
  }
  if (c->mode == CID_MODE_AUTO)
    c->fsk->toneInc = 9;
  c->fmt = fmt;
  return c;
}

/**
 * @brief Free a CID object with its detectors and the filter buffer.
 * @param c  object
 */
void cid_delete(CidObj* c)
{
  if (c->mode != CID_MODE_FSK)
    sysdep_free(c->dtmf);
  if (c->mode != CID_MODE_DTMF)
    FPM_MRF_free(&c->fsk->mrf);
  sysdep_free(c->fsk);
  sysdep_free(c);
}

/**
 * @brief Reset the detectors of the object and discard the collected frame; any mode above 1 becomes the automatic mode again.
 * @param c  object
 */
void cid_reset(CidObj* c)
{
  if (c->mode > 1)
    c->mode = CID_MODE_AUTO;
  if (c->mode != CID_MODE_FSK)
    reset_dtmf(c->dtmf);
  if (c->mode != CID_MODE_DTMF)
    reset_cid(c->fsk);
  c->fill = 0;
  if (c->mode == CID_MODE_AUTO)
    c->fsk->toneInc = 9;
}

/**
 * @brief Set the sampling rate of the detectors that the mode uses.
 * @param c   object
 * @param fs  sampling rate, 8000 or 9600 (no other value is checked); the tone increment is 9 for either
 */
void cid_freq_sampl(CidObj* c, int fs)
{
  int mode = c->mode;
  CidDtmf* d = c->dtmf; /* the sub-object pointers are read once: a 16-bit store cannot alias them, but the optimiser must not be able to move the loads */
  CidFsk* k = c->fsk;
  if (mode != CID_MODE_FSK)
    d->fs = (unsigned short)fs;
  if (mode != CID_MODE_DTMF)
    k->fs = (unsigned short)fs;
  if (fs == SAMPLE_RATE_9600 && mode == CID_MODE_FSK)
    k->toneInc = 9;
  if (fs == SAMPLE_RATE_8000 && mode == CID_MODE_FSK)
    k->toneInc = 9;
  if (mode > 1)
    k->toneInc = 9;
}

/**
 * @brief Set the detection threshold of the detectors that the mode uses: the sensitivity mode of the DTMF detector, the byte framer threshold of the
 *        FSK receiver.
 * @param c  object
 * @param v  value
 */
void cid_threshold(CidObj* c, int v)
{
  int mode = c->mode;
  CidDtmf* d = c->dtmf;
  CidFsk* k = c->fsk;
  if (mode != CID_MODE_FSK)
    d->mode = (short)v;
  if (mode != CID_MODE_DTMF)
    k->cfgThr = (unsigned short)v;
}

/**
 * @brief Select the result format.
 * @param c  object
 * @param v  2 = hexadecimal text, anything else = formatted strings
 */
void cid_value(CidObj* c, int v) { c->fmt = v; }

/**
 * @brief Build the result strings.
 * @param c  object
 * @return the string buffer `c->str`: NUL-separated strings, ended by an empty one.  With the DTMF modes (all but the FSK modes 0 and 2) a single string
 *         with the first 16 digits; with the FSK modes the message in the selected format.
 */
char* cid_get_strings(CidObj* c)
{
  char* s = c->str;
  sysdep_memset(s, 0, sizeof c->str);
  if (c->mode != CID_MODE_FSK && c->mode != CID_MODE_FSK_DONE)
  {
    int i;
    for (i = 0; i < 16; i++)
      s[i] = c->dtmf->digits[i];
    return s;
  }
  if (c->fmt != 2)
    data_formatted_output(c->fsk, s);
  else
    data_unformatted_output(c->fsk, s);
  return s;
}

/*
 * Collect the samples into frames and run the detectors on each full frame.  Returns 0 (need more input), 1 (success), 2 (failure) or 3 (the DTMF
 * detector is receiving digits).  The three result variables (the DTMF result of the automatic mode, the FSK result, the result of the single detector)
 * keep their value across the rounds of the loop.
 */
static int cid_progress_body(CidObj* c, const short* in, int n)
{
  /* c->mode, c->fill and the sub-object pointers are mirrored in locals (stores go to both): the 16-bit sample stores into buf[] cannot alias them, but the
   * optimiser must not be able to decide whether to re-read them (the code has to be the same with and without -fstrict-aliasing) */
  int cm = 1, dm = 1, r1 = 1, ret = 0, frame = 0, mode = c->mode, i = 0, f = c->fill;
  CidDtmf* dtmf = c->dtmf;
  CidFsk* fsk = c->fsk;
  short* buf = c->buf;

  /* frame length from the sampling rate: 160 at 8000 Hz, else 192 */
  if (mode != CID_MODE_FSK)
    frame = dtmf->fs != SAMPLE_RATE_8000 ? 192 : 160;
  if (mode != CID_MODE_DTMF)
    frame = fsk->fs != SAMPLE_RATE_8000 ? 192 : 160;

  /* fill the frame buffer */
  if ((short)0 < (short)n && f < frame)
  {
    do
    {
      buf[f] = in[i];
      f++;
      i = (short)(i + 1);
      if ((short)i >= (short)n)
        break;
    } while (f < frame);
    c->fill = f;
  }
  if (f != frame)
    return ret;

  /* one iteration per full frame */
  for (;;)
  {
    if (mode > 1)
    {
      if (mode == CID_MODE_AUTO)
      {
        dm = (short)dtmf_modem(buf, (unsigned short)frame, dtmf);
        cm = (short)cid_modem(buf, (unsigned short)frame, fsk);
      }
      else
      {
        ret = 3;
        if (mode == CID_MODE_DTMF_MESSAGE)
          r1 = (short)dtmf_modem(buf, (unsigned short)frame, dtmf);
      }
      if ((short)dm == 2)
      {
        mode = CID_MODE_DTMF_MESSAGE;
        c->mode = CID_MODE_DTMF_MESSAGE;
      }
      if ((short)dm == 3 || (short)dm == -1)
      {
        ret = 2;
        mode = CID_MODE_DTMF_MESSAGE;
        c->mode = CID_MODE_DTMF_MESSAGE;
        if (dsplibs_debug_level > 1)
          dsplibs_debug_printf("\n DTMF Failed , not enough numbers detected \n");
      }
      if ((short)cm == 3)
      {
        mode = CID_MODE_FSK_DONE;
        c->mode = CID_MODE_FSK_DONE;
        ret = 1;
      }
      if ((short)cm == -1)
      {
        if (dsplibs_debug_level > 1)
          dsplibs_debug_printf("\n FSK demodulator Failed \n");
        reset_cid(fsk);
      }
    }
    else
    {
      if (mode == CID_MODE_DTMF)
        r1 = (short)dtmf_modem(buf, (unsigned short)frame, dtmf);
      else
        r1 = (short)cid_modem(buf, (unsigned short)frame, fsk);
      if ((short)r1 == 2 && mode == CID_MODE_DTMF)
        ret = 3;
    }
    if ((short)r1 == 3)
      ret = 1;
    if ((short)r1 == -1)
    {
      ret = 2;
      if (mode == CID_MODE_FSK && dsplibs_debug_level > 1)
        dsplibs_debug_printf("\n FSK demodulator Failed \n");
      if (mode == CID_MODE_DTMF && dsplibs_debug_level > 1)
        dsplibs_debug_printf("\n DTMF demodulator Failed  \n");
      if (mode == CID_MODE_DTMF_MESSAGE && dsplibs_debug_level > 1)
        dsplibs_debug_printf("\n DTMF demodulator Failed after CID_MESSAGE state \n");
    }

    /* start the next frame from the remaining input */
    f = 0;
    if ((short)i < (short)n && frame > 0)
    {
      do
      {
        buf[f] = in[i];
        i = (short)(i + 1);
        f++;
        if ((short)i >= (short)n)
          break;
      } while (f < frame);
    }
    c->fill = f;
    if (f != frame)
      return ret;
  }
}

/**
 * @brief Feed samples to the CID object.
 * @param c       object
 * @param in      samples
 * @param unused  not used
 * @param pn      in: number of samples in `in`; out: 0 (all samples are consumed or buffered)
 * @return 0 = need more input; 1 = a message is complete (success); 2 = failure; 3 = the DTMF detector is receiving digits (also returned in mode 2, in
 *         which no detector runs)
 *
 * The samples are collected in frames of 160 (8000 Hz) or 192 (9600 Hz) and each full frame is run through the detector(s) of the current mode.  In the
 * FSK mode a result 3 of cid_modem() is success and -1 failure; in the automatic mode the DTMF and FSK results are combined and the mode changes
 * (CidMode).
 * @note  After a failure in the FSK-only mode the receiver is not reset: the next frame meets the same state again (an incomplete or bad message stays a
 *        failure).
 */
int cid_progress(CidObj* c, const short* in, int unused, short* pn)
{
  int n = *pn, r;
  (void)unused;
  r = cid_progress_body(c, in, n);
  *pn = 0;
  return r;
}

/* ------------------------------------------------------------------------------------------------ entry points */
/**
 * @brief Create the caller-ID receiver for a modem.
 * @param modem  the host's modem object, passed back to modem_send_to_tty()
 * @param fs     sampling rate, 8000 or 9600 (any other value gives NULL)
 * @param fmt    result format (see cid_value())
 * @return the handle, or NULL on failure; release with CID_delete().  The FSK-only mode is used.
 */
CidHandle* CID_create(void* modem, int fs, int fmt)
{
  CidHandle* h;
  CidObj* c;

  if (dsplibs_debug_level > 1)
    dsplibs_debug_printf("cid: create...\n");
  if (fs != SAMPLE_RATE_8000 && fs != SAMPLE_RATE_9600)
    return NULL;

  h = (CidHandle*)sysdep_malloc(sizeof(CidHandle));
  if (h == NULL)
    return NULL;
  sysdep_memset(h, 0, sizeof *h);
  h->modem = modem;
  c = cid_create(NULL, fmt, CID_MODE_FSK);
  h->cid = c;
  if (c == NULL)
  {
    sysdep_free(h);
    return NULL;
  }
  if (fs == SAMPLE_RATE_9600)
    cid_freq_sampl(h->cid, SAMPLE_RATE_9600);
  return h;
}

/**
 * @brief Delete a handle from CID_create().
 * @param h  handle
 */
void CID_delete(CidHandle* h)
{
  if (dsplibs_debug_level > 1)
    dsplibs_debug_printf("cid: delete...\n");
  cid_delete(h->cid);
  sysdep_free(h);
}

/**
 * @brief Process a block of line samples.
 * @param h  handle
 * @param in samples
 * @param n  number of samples (nothing is done for n < 1)
 *
 * The samples are passed to cid_progress() in chunks of at most 192.  On success (the message is complete) the strings of cid_get_strings() are written to
 * the host's tty, at most four, each followed by CR LF, and the function returns; on failure it logs (debug level above 1) and returns; the remaining
 * samples of the block are dropped in both cases.
 * @return 1 when a complete message was found and sent, -1 when the detector failed, 0 when more samples are needed (the host stops the detector for
 *         any non-zero value).
 */
int CID_process(CidHandle* h, const short* in, int n)
{
  int result = 0;

  for (;;)
  {
    int chunk, r;
    short pn;
    if (n < 1)
      return result;
    chunk = n > 0xc0 ? 0xc0 : n;
    result = 0;
    pn = (short)chunk;
    r = (short)cid_progress(h->cid, in, 0, &pn);
    if (r != 0)
    {
      if (r != 1)
      {
        result = -1;
        if (dsplibs_debug_level > 1)
          dsplibs_debug_printf("cid: CID Failure\n");
      }
      else
      {
        char* s;
        int k;
        if (dsplibs_debug_level > 1)
          dsplibs_debug_printf("cid: CID Success\n");
        result = 1;
        s = cid_get_strings(h->cid);
        for (k = 0; k < 4; k++)
        {
          if (s == NULL || *s == 0)
            break;
          if (dsplibs_debug_level > 1)
            dsplibs_debug_printf("cid: STR: '%s'\n", s);
          modem_send_to_tty(h->modem, s, (int)sysdep_strlen(s));
          modem_send_to_tty(h->modem, "\r\n", 2);
          s += sysdep_strlen(s) + 1;
        }
        return result;
      }
    }
    if (result != 0)
      return result;
    n -= chunk;
    in += chunk;
  }
}
