/**
 * @file v34_tx_level.c
 * @brief V.34 transmit level: the encoded debug print, the V.PCM minimum power reduction, and the transmit gain set from the remote modem's INFO.
 *
 * Entry points:
 *  - edprintf(): the library's debug print.  It formats the message, scrambles it into a printable encoded string and passes that to the host's
 *    dsplibs_debug_printf() when the debug level is above 1.  v34_ed_index() exposes its rotating encoding index to src/vpcm/v34_modem_init.c, which encodes
 *    single characters with the same index.
 *  - GetVPcmMinimalTxPowerReduction(): derives the minimum transmit power reduction (dB) of a V.PCM session from the dp parameter block and the session
 *    state, and, as a side effect, programs the echo-canceller start values of the V.34 object and the "high level transmit" flag of the V.90 parameters.
 *    Only settxlevel() calls it on the V.34 path.
 *  - settxlevel(): converts the power reduction the remote modem asked for (two 3-bit fields of its INFO word), combined with the minimum above in V.PCM
 *    sessions, into the transmit gain `txGain_` of the V.34 object.
 *
 * The V.34 object is reached through the typed view V34Obj (v34_object_view.h); the dp parameter block (V34ModParObj), the V.90 parameter object
 * (V90ParObj) and the V.PCM modem context (PcmFlo47Obj) through their typed views.  edprintf() keeps its state in one file-local struct and is
 * not reentrant.
 */

#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>

#include "v34_init_tables.h" /* offsetarr */
#include "core/dsp_host.h"
#include "v34_internal.h"

/* byte-offset accessors */
#include "v34_access.h"

/* typed views: V34ModParObj (member V34P_<off>, the dp parameter block), V90ParObj (V34CFG_<off>), PcmFlo47Obj (VPFM_<off>), V34Obj (V34M_<off>) */
#include "core/dp_params.h" /* the one definition of the parameter block (V34ModParObj, V34P_0xNN) */
#include "v34_v90params_view.h"
#include "v34_flomodem_view.h"
#include "v34_object_view.h"

/* ===================== edprintf ===================== */

/**
 * State of edprintf(), kept in one struct so that its three parts have a fixed relative layout:
 * `temp` the formatted message, `enc` the encoded output, `iEncodeOffset` the rotating index (0 .. 9) into `offsetarr`.
 */
static struct
{
  char temp[0x100];  ///< the formatted message (sysdep_vsnprintf is told 0x100 bytes)
  char enc[0x110];   ///< the encoded output: at most 4 + 2 * 131 + 4 + 1 = 271 bytes including the terminator
  int iEncodeOffset; ///< position in `offsetarr`, advanced after every encoded character
} g_ed;

/**
 * @brief Address of edprintf()'s rotating encoding index.
 * @return pointer to the index (0 .. 9); src/vpcm/v34_modem_init.c uses it for its single-character encoder, so both encoders advance one shared index
 */
int* v34_ed_index(void) { return &g_ed.iEncodeOffset; }

/**
 * @brief Format a message and, at debug level 2 or higher, print it in the library's encoded form.
 * @param fmt  printf-style format; the remaining arguments are the format's
 *
 * The message is formatted into a 256-byte buffer.  If it is at most 131 characters long, the printed string is "$!$ " followed by two characters per
 * message byte (the high nibble, then the low nibble, each offset by one entry of the 10-entry `offsetarr` table and by '0') and "????".  A longer message
 * is replaced by "too long print string".  The result goes to dsplibs_debug_printf("%s\n", ...) only when dsplibs_debug_level is above 1 (unsigned).
 *
 * @note The high nibble is an arithmetic shift of the SIGNED byte, so bytes of 0x80 and above give a negative nibble; all character arithmetic wraps
 *       modulo 256.  Only the low byte of each `offsetarr` entry is used.
 * @note The encoding index is reset to 0 at the start of every message that is not too long; the too-long path leaves it alone.
 * @warning Not reentrant: the buffers and the index are shared by all callers.
 */
void edprintf(const char* fmt, ...)
{
  va_list ap;
  size_t w, i;

  va_start(ap, fmt);
  sysdep_vsnprintf(g_ed.temp, 0x100, fmt, ap);
  va_end(ap);

  /* the encoded form takes 2 * len + 8 bytes; the test is an unsigned 32-bit compare against 0x10e (len > 131 is too long) */
  if ((uint32_t)sysdep_strlen(g_ed.temp) * 2u + 8u <= 0x10eu)
  {
    g_ed.iEncodeOffset = 0;
    sysdep_strcpy(g_ed.enc, "$!$ ");
    w = sysdep_strlen(g_ed.enc);
    i = 0;

    /* two output characters per input byte; the length is re-evaluated on every pass, and the index is re-read from memory before each character */
    while (sysdep_strlen(g_ed.temp) > i)
    {
      unsigned int c = (unsigned char)g_ed.temp[i];
      int e = g_ed.iEncodeOffset;
      int e2 = (e == 9) ? 0 : e + 1;

      /* high nibble: signed arithmetic shift of the byte, plus the low byte of the table entry, plus '0', wrapped to 8 bits */
      g_ed.enc[w] = (char)(unsigned char)(((int)(int8_t)(unsigned char)c >> 4) + (unsigned char)offsetarr[e] + '0');
      g_ed.iEncodeOffset = e2;
      i++;

      /* low nibble, with the next table entry */
      e2 = g_ed.iEncodeOffset;
      g_ed.enc[w + 1] = (char)(unsigned char)((int)(c & 0xf) + (unsigned char)offsetarr[e2] + '0');
      g_ed.iEncodeOffset = (e2 == 9) ? 0 : e2 + 1;
      w += 2;
    }

    g_ed.enc[w] = 0;
    sysdep_strcat(g_ed.enc, "????");
  }
  else
  {
    /* the index is deliberately not reset on this path */
    sysdep_strcpy(g_ed.enc, "too long print string");
  }

  /* the level is read once, after the last host call; the compare is unsigned */
  if (dsplibs_debug_level > 1)
    dsplibs_debug_printf("%s\n", g_ed.enc);
}

/* ===================== GetVPcmMinimalTxPowerReduction ===================== */

/**
 * @brief Minimum transmit power reduction of a V.PCM session, and the echo-canceller start values that go with it.
 * @param obj  the V.34 object (V34Obj); its `params_` (dp parameter block) and `ctx_` (V.PCM modem context, whose `v90Params_` is the V.90 parameter
 *             object) pointers must be valid
 * @return the reduction in dB, clamped to -10 .. 7
 *
 * Steps:
 *  - r = the signed 16-bit word at offset 0x44 of the parameter block, clamped to -10 .. 7.
 *  - If the context's flag at 0x6120 is zero, or `pcmMode_` is zero, or the V.90 parameters' `sensitiveIspDetected` is non-zero, then r becomes 0 unless
 *    `k56Mode_` is set; otherwise r is kept.
 *  - r > 0 ("high level" off): V.90 parameters `highLevelTxActive` = 0, `echoThresh_` = 2000, `echoK2_` = 0x7fdf, `echoBeta_` = 2.
 *    r <= 0: `highLevelTxActive` = 1, `echoThresh_` = 2000, `echoK2_` = 0x7fcb, `echoBeta_` = 4, or 6 when the parameter block's word at 0x54 is not 4.
 *  - Two debug prints report the result.
 *
 * @note Writes into the V.90 parameter object and the V.34 object, not only reads.
 * @warning No NULL checks on `params_`, `ctx_` or `ctx_->v90Params_`.
 */
int GetVPcmMinimalTxPowerReduction(V34Obj* obj)
{
  V34ModParObj* P = obj->params_;
  PcmFlo47Obj* floobj = (PcmFlo47Obj*)obj->ctx_;
  void* cfg;
  int r = (int16_t)P->minTxPowerReduction; /* the low 16 bits of the word (little-endian), as the former S16 view at +0x44 */

  if (r < -10)
    r = -10;
  else if (r > 7)
    r = 7;

  cfg = floobj->v90Params_;
  if (floobj->flag6120_ == 0 || obj->pcmMode_ == 0 || ((V90ParObj*)cfg)->sensitiveIspDetected != 0)
  {
    /* `||` short-circuits: the sensitive-ISP flag is only read when the first two are non-zero.  Without the K56flex flag the reduction is dropped */
    if (obj->k56Mode_ == 0)
      r = 0;
  }

  if ((short)r > 0)
  {
    /* a positive reduction: high-level transmit off, steeper echo decay */
    ((V90ParObj*)cfg)->highLevelTxActive = 0;
    obj->echoThresh_ = 2000;
    obj->echoK2_ = 0x7fdf;
    obj->echoBeta_ = 2;
  }
  else
  {
    /* no reduction: high-level transmit on; beta depends on the parameter block's word at 0x54 */
    ((V90ParObj*)cfg)->highLevelTxActive = 1;
    obj->echoThresh_ = 2000;
    obj->echoK2_ = 0x7fcb;
    obj->echoBeta_ = (P->codecType != 4) * 2 + 4;
  }

  edprintf("VPcmV34Main: Due to final MinTXPR = %d, setting echo: decay start = %d, decay fact = %d, beta = %d\r\n", r, obj->echoThresh_, obj->echoK2_,
           obj->echoBeta_);
  edprintf("VPcmV34Main: Get Minimal power reduction - returning %d (cfg flag set to %d)\r\n", r, ((V90ParObj*)(floobj->v90Params_))->highLevelTxActive);
  return r;
}

/* ===================== settxlevel ===================== */

/**
 * @brief Set the transmit gain `txGain_` from the power reduction the remote modem requested.
 * @param obj   the V.34 object (V34Obj)
 * @param info  the remote INFO record (first 16-bit word read): bits 5 .. 7 and bits 2 .. 4 are two 3-bit fields, each bit-reversed with bitreverse()
 *
 * Steps:
 *  - min = GetVPcmMinimalTxPowerReduction(obj) (truncated to short).
 *  - `w25dc_` (the requested reduction in dB) = bitreverse(info bits 5 .. 7) + min(bitreverse(info bits 2 .. 4), 3).  It is printed when the debug level
 *    is above 1.
 *  - Without `pcmMode_` the reduction red is `w25dc_`.  In a V.PCM session a negative min is added to it (and the sum stored back in `w25dc_`); a
 *    non-negative min acts as a floor (`w25dc_` is raised to min when below).
 *  - The gain is scaled once per dB: red < 0 multiplies by 0x47cf / 2^14 (about +1 dB), truncating each step to 16 bits; red > 0 multiplies by 0x390a / 2^14
 *    (about -1 dB) with no intermediate truncation (32-bit arithmetic that may wrap).
 *  - Finally txGain_ = (scale * 0x4b4b + 0x2000) >> 14 (32-bit arithmetic, truncated to 16 bits), and the before / after values are printed.
 *
 * @note The debug level is sampled before the first print and again after it (when it printed), so a level change by the host inside the print is seen
 *       by the second print's guard only in that case.
 */
void settxlevel(V34Obj* obj, const unsigned short* info)
{
  int min = (short)GetVPcmMinimalTxPowerReduction(obj);
  int scale = obj->tx.txGain_;
  short t;
  short red;
  unsigned int level;

  /* the requested reduction: one reversed field plus the other, capped at 3 */
  obj->tx.w25dc_ = bitreverse((unsigned short)((*info >> 5) & 7), 3);
  t = (short)bitreverse((unsigned short)((*info >> 2) & 7), 3);
  if (t > 3)
    t = 3;
  obj->tx.w25dc_ = (uint16_t)(t + obj->tx.w25dc_);
  level = dsplibs_debug_level;
  if (level > 1)
  {
    dsplibs_debug_printf("V34TXSCALE, power reduction requested by remote modem is %d dB\n", (int)(short)obj->tx.w25dc_);
    level = dsplibs_debug_level;
  }

  /* combine with the V.PCM minimum */
  if (obj->pcmMode_ == 0)
  {
    red = (short)obj->tx.w25dc_;
  }
  else if ((short)min < 0)
  {
    red = (short)(min + obj->tx.w25dc_);
    obj->tx.w25dc_ = (uint16_t)red;
  }
  else
  {
    red = (short)obj->tx.w25dc_;
    if (red < (short)min)
    {
      obj->tx.w25dc_ = (uint16_t)min;
      red = (short)min;
    }
  }

  /* one scaling step per dB */
  if (red < 0)
  {
    short n;
    for (n = red; n < 0; n = (short)(n + 1))
      scale = (int)(int16_t)((int32_t)((uint32_t)scale * 0x47cfu) >> 14);
  }
  else
  {
    short n;
    for (n = 0; red > n; n = (short)(n + 1))
      scale = (int32_t)((uint32_t)scale * 0x390au) >> 14;
  }

  if (level > 1)
    dsplibs_debug_printf("V34TXSCALE, txscale before is %d, reduced txscale is %d dB,final txscale is %d\n", (int)obj->tx.txGain_, (int)red, scale);
  obj->tx.txGain_ = (int16_t)((int32_t)((uint32_t)scale * 0x4b4bu + 0x2000u) >> 14);
}
