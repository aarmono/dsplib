/**
 * @file v34_setup.c
 * @brief V.34 bit-allocation block set-up, receive and transmit front-end set-up and the transmit restart.
 *
 * - initV34(blk, ...): builds the bit-allocation ("shell mapper") block for one direction from the symbol rate, the data rate and the scale entry:
 *   the block geometry (rows, taps, exponents), the triangle 1..m..1 and its autoconvolution, and a slice of the `xyz` table.  initdigital()
 *   (v34_handshake_helpers.c) calls it for the transmit and the receive block.
 * - V34SetupDemodulator(obj, baud, carrier): sets the receive front end's timing constants and carrier rotation table for a symbol rate and carrier.
 *   Nothing in the library calls it (v34modeminit inlines its 2400 baud / 1800 Hz arm).
 * - V34SetupModulator(m, baud, carrier, preemp, v90, fullReset): loads the transmit modulator with its polyphase bank, the echo pre-filter coefficients,
 *   the pre-emphasis row and the carrier rotation table.
 * - v34setuptxmit(obj): restarts the transmitter: sets the transmit level (settxlevel), calls V34SetupModulator, moves the handshake state machines to
 *   their initial states and calls txinit().
 *
 * Tables are those of v34_init_tables.c; the pointers stored in the object are the addresses of those tables, so the object holds the table identity, not
 * a copy.  Objects are reached through the typed views (v34_object_view.h, v34_subobjects.h).
 *
 * Hazards:
 * - V34SetupModulator() stores a pre-emphasis row pointer computed from the caller's `preemp` without a range check; a value outside 1..10 points outside
 *   the pre-emphasis table.
 * - V34SetupModulator() returns a pointer-typed value that is the quotient `[m+0x18] / [m+0x14]` when fullReset == 0 (see its @warning).
 * - initV34() never terminates if the minimum / maximum tables gave m == 0; the tables contain only 1..15 and 1..18, so this cannot happen for the
 *   exponent range e = 0..31 that the geometry computation always yields.
 */

#include <stddef.h>
#include <stdint.h>

#include "v34_init_tables.h"
#include "v34_symbol_rates.h" /* V34_BAUD_*, V34_CARRIER_* */
#include "core/dsp_host.h"
#include "v34_internal.h"

#include "v34_access.h" /* AS_U16 */
#include "v34_object_view.h"

/* ===================== initV34 ===================== */

/**
 * @brief Build the bit-allocation block of one direction.
 * @param blkbobj  the block (V34BlkB layout, 0x450 bytes) to fill
 * @param p2       symbol rate in baud (only the low 16 bits are compared)
 * @param p3       data rate in bit/s (low 16 bits)
 * @param p4       selects the table of the minimum (0) or maximum (non-zero) exponent
 * @param p5       non-zero: store p5 << 5 at +0x16 and the convolution routine (Convolve32 for p5 == 1, else Convolve64) at +0x28
 * @param p6       address stored at +0x24 (initdigital() passes an address inside the object: obj + 0x2a68 or obj + 0xe84)
 * @param p7       scale value stored at +0x42; its ceiling log2 (at least 7) plus 1 is stored at +0x44
 * @return 0
 * @note  Fields written (byte offsets in the block): +0x00 N (symbols per row: 12 below 2800 baud, 14 at 2800, 15 above except 16 at 3200), +0x02 2N,
 *        +0x24 p6, +0x3a gInvertPat[2K-2], +0x3e 2K-2, +0x40 2K (K = 7, or 8 at 2743 and 3429 baud), +0x04 rows, +0x46 2 - (rows <= 0x37),
 *        +0x0a 15-K, +0x06 u - (rows-1) N, +0x14 c, +0x0e e, +0x10 e-1, +0x12 m, the triangle 1..m..1 at +0x48, its autoconvolution at +0x148, the
 *        slice of `xyz` at +0x248, +0x42 p7 and +0x44.  Here u = ((p3 / 25) * 7) / K bits and rows = ceil(u / N).  All stores are 16 bits wide;
 *        N, rows and e are compared as unsigned 16-bit values, the minimum / maximum table entries are read as signed bytes.
 */
int initV34(V34BlkB* blkbobj, unsigned short p2, unsigned short p3, short p4, short p5, void* p6, unsigned short p7)
{
  unsigned int K = 7u + ((p2 == V34_BAUD_2743 || p2 == V34_BAUD_3429) ? 1u : 0u); /* 7 or 8 */
  unsigned int N;
  unsigned int u;
  int rowsFull;
  uint16_t rows;
  int e;
  int8_t m8;
  uint16_t m16;
  uint32_t m, base, cnt, i, k;
  const unsigned short* A;

  /* symbols per row N (unsigned 16-bit compares of the baud rate) */
  if (p2 > V34_BAUD_2800)
    N = 0x10u - (p2 != V34_BAUD_3200 ? 1u : 0u);
  else if (p2 == V34_BAUD_2800)
    N = 0xe;
  else
    N = 0xc;
  blkbobj->w0a00_ = (uint16_t)N;
  blkbobj->w0a02_ = (uint16_t)(2 * N);
  blkbobj->p0a24_ = p6; /* an address (initdigital passes obj+0x2a68 / obj+0xe84), a real pointer slot on x86-64 */

  if (p5 != 0)
  {
    blkbobj->w0a16_ = (uint16_t)((uint32_t)(int32_t)p5 << 5);
    blkbobj->p0a28_ = (p5 == 1) ? (void*)Convolve32 : (void*)Convolve64;
  }

  AS_U16(blkbobj->w0a40_) = (uint16_t)(2 * K);
  blkbobj->w0a3e_ = (uint16_t)(2 * K - 2);
  AS_U16(blkbobj->w0a3a_) = gInvertPat[(int16_t)(2 * K - 2)];

  /* block geometry: u = ((p3 / 25) * 7) / K bits per frame (fits 16 bits), cut into rows of N symbols */
  u = (uint16_t)((((unsigned int)p3 / 25u) * 7u) / K);
  rowsFull = ((int)u + (int)N - 1) / (int)N; /* signed division */
  rows = (uint16_t)rowsFull;
  blkbobj->w0a04_ = rows;
  blkbobj->w0a46_ = (uint16_t)(2 - (rows <= 0x37 ? 1 : 0));
  blkbobj->w0a0a_ = (uint16_t)(15 - K);
  blkbobj->w0a06_ = (uint16_t)((int)u - ((int)rows - 1) * (int)N);
  blkbobj->w0a14_ = 0;

  /* exponent e: rows above 12 are reduced by steps of 8 until e fits 0..31 (c counts the steps) */
  e = 0;
  if (rows > 0xc)
  { /* unsigned 16-bit compare */
    e = rowsFull - 0xc;
    if ((uint16_t)e > 0x1f)
    {
      unsigned int c = 0;
      do
      {
        c++;
        e = rowsFull - (int)((c & 0xffffu) << 3) - 0xc;
      } while ((uint16_t)e > 0x1f);
      blkbobj->w0a14_ = (uint16_t)c;
    }
  }

  /* e is always in 0..31 here: rowsFull <= 331 and each loop step (8) is smaller than the 32-wide acceptance window */
  AS_U16(blkbobj->w0a0e_) = (uint16_t)e;
  AS_U16(blkbobj->w0a10_) = (uint16_t)(e - 1);

  /* triangle half-width m from the minimum / maximum table (signed byte entries) */
  m8 = (p4 == 0) ? MMinTable[(uint16_t)e] : MMaxTable[(uint16_t)e];
  m16 = (uint16_t)(int16_t)m8;
  blkbobj->w0a12_ = m16;
  m = m16;

  base = xyz[m];

  /* triangle 1, 2, ..., m, ..., 2, 1 at +0x48 (the mirrored half is stored first, then the direct one) */
  if (m != 0)
  {
    i = 0;
    do
    {
      uint32_t v = i + 1;
      blkbobj->shellX_[2 * (m - 1) - i] = (uint16_t)v;
      blkbobj->shellX_[i] = (uint16_t)v;
      i = v;
    } while (m > i);
  }

  /* autoconvolution of the triangle, low 16 bits, stored mirrored at 4(m-1)-i and at i (+0x148); the inner loop count is the signed 16-bit value
   * (short)(i+1); the outer loop runs while i+1 <= 2(m-1) (unsigned) */
  A = blkbobj->shellX_;
  i = 0;
  do
  {
    uint32_t ip1 = i + 1;
    int16_t count = (int16_t)ip1;
    const unsigned short* hi = A + (uint16_t)i;
    const unsigned short* lo = A;
    uint32_t acc = 0;
    while (count > 0)
    {
      acc += (uint32_t)(*lo) * (uint32_t)(*hi);
      lo++;
      hi--;
      count = (int16_t)(count - 1);
    }
    blkbobj->shellY_[4 * (m - 1) - i] = (uint16_t)acc;
    blkbobj->shellY_[i] = (uint16_t)acc;
    i = ip1;
  } while (i <= 2 * (m - 1));

  /* copy xyz[xyz[m] .. xyz[m+1]) to +0x248 (unsigned count compare) */
  cnt = xyz[m + 1] - base;
  for (k = 0; k < cnt; k++)
    blkbobj->shellZ_[k] = xyz[base + k];

  /* scale value and its ceiling log2 (at least 7), plus 1 */
  AS_U16(blkbobj->w0a42_) = p7;
  blkbobj->w0a44_ = 7;
  if ((unsigned int)p7 > 0x80)
  { /* on the zero-extended value */
    int c = 7;
    do
    {
      c++;
    } while ((int)(1u << (c & 31)) < (int)(unsigned int)p7); /* the shift count is taken modulo 32 */
    blkbobj->w0a44_ = (uint16_t)c;
  }
  blkbobj->w0a44_ = (uint16_t)(blkbobj->w0a44_ + 1);
  return 0;
}

/* ===================== V34SetupDemodulator ===================== */

/**
 * @brief Set the receive front end's timing constants and carrier rotation table for a symbol rate and carrier.
 * @param obj        the V.34 module object; the receive front end is `obj->rx` (b = obj + 0x264)
 * @param baud_s     symbol rate in baud (V34_BAUD_*); compared as a signed 32-bit value
 * @param carrier_s  carrier in Hz (V34_CARRIER_*)
 * @note  Prints a debug message (level above 1), sets b+0x128 to 4, then per baud rate the timing constants b+0x1ac / 0x1ae / 0x1b0 / 0x1be and per carrier
 *        the rotation table pointer (`hsineTab_`, b+0x1b4) and its half length (`hsineLen_`, b+0x1ba).  An unlisted baud rate or carrier writes nothing and
 *        prints nothing.  No code in the library calls this function.
 */
void V34SetupDemodulator(V34Obj* obj, short baud_s, short carrier_s)
{
  V34Rx* rxobj = &obj->rx;
  int baud = baud_s, carrier = carrier_s;

  if (1 < dsplibs_debug_level)
    dsplibs_debug_printf("V34SetupDemodulator: baudrate %ld, carrier %ld\n", (long)baud, (long)carrier);
  rxobj->w38c_ = 4;

  /* timing constants by symbol rate */
  switch (baud)
  {
  case V34_BAUD_2800:
    rxobj->w414_ = 0x3e82;
    rxobj->w412_ = 0x3594;
    rxobj->w422_ = 0x3594;
    rxobj->w410_ = 0x1f41;
    break;
  case V34_BAUD_2400:
    rxobj->w414_ = 0x3e80;
    rxobj->w412_ = 0x3e80;
    rxobj->w422_ = 0x3e80;
    rxobj->w410_ = 0x1f40;
    break;
  case V34_BAUD_2743:
    rxobj->w412_ = 0x36b0;
    rxobj->w414_ = 0x3e80;
    rxobj->w422_ = 0x36b0;
    rxobj->w410_ = 0x1f40;
    break;
  case V34_BAUD_3000:
    rxobj->w414_ = 0x3e80;
    rxobj->w412_ = 0x3200;
    rxobj->w422_ = 0x3200;
    rxobj->w410_ = 0x1f40;
    break;
  case V34_BAUD_3200:
    rxobj->w412_ = 0x2ee0;
    rxobj->w414_ = 0x3e80;
    rxobj->w422_ = 0x2ee0;
    rxobj->w410_ = 0x1f40;
    break;
  case V34_BAUD_3429:
    rxobj->w414_ = 0x3e80;
    rxobj->w412_ = 0x2bc0;
    rxobj->w422_ = 0x2bc0;
    rxobj->w410_ = 0x1f40;
    break;
  default:
    break;
  }

  /* carrier rotation table by carrier frequency */
  switch (carrier)
  {
  case V34_CARRIER_1600:
    rxobj->hsineTab_ = (void*)hsine1600;
    rxobj->hsineLen_ = 6;
    break;
  case V34_CARRIER_1680:
    rxobj->hsineTab_ = (void*)hsine1680;
    rxobj->hsineLen_ = 0x28;
    break;
  case V34_CARRIER_1800:
    rxobj->hsineTab_ = (void*)hsine1800;
    rxobj->hsineLen_ = 0x10;
    break;
  case V34_CARRIER_1829:
    rxobj->hsineTab_ = (void*)hsine1829;
    rxobj->hsineLen_ = 0x15;
    break;
  case V34_CARRIER_1867:
    rxobj->hsineTab_ = (void*)hsine1867;
    rxobj->hsineLen_ = 0x24;
    break;
  case V34_CARRIER_1920:
    rxobj->hsineTab_ = (void*)hsine1920;
    rxobj->hsineLen_ = 5;
    break;
  case V34_CARRIER_1959:
    rxobj->hsineTab_ = (void*)hsine1959;
    rxobj->hsineLen_ = 0x31;
    break;
  case V34_CARRIER_2000:
    rxobj->hsineTab_ = (void*)hsine2000;
    rxobj->hsineLen_ = 0x18;
    break;
  default:
    break;
  }
}

/* ===================== V34SetupModulator ===================== */

/* Row `preemp` (1-based) of a 10-row, 32-byte-row pre-emphasis table: (start - 0x20) + 32 * preemp.  There is no bounds check; the computation is done in
 * integers so that an out-of-range index does not invoke pointer-arithmetic undefined behaviour. */
static const short* pre_row_base(const short* tbl, int preemp) { return (const short*)(uintptr_t)((intptr_t)(uintptr_t)tbl - 0x20 + (intptr_t)preemp * 32); }

/**
 * @brief Load the transmit modulator for a symbol rate, carrier and pre-emphasis.
 * @param modobj     the modulator sub-object (`obj->mod`, obj + 0x1450); v34setuptxmit() passes it
 * @param baud_s     symbol rate in baud: V34_BAUD_2400 / 2800 / 3000 / 3200 / 3429, V34MOD_ALLPASS_4800 or V34MOD_DPSK_600; any other value prints an error
 *                   (level above 1) and selects the 600 baud shaper
 * @param carrier_s  carrier in Hz (V34_CARRIER_*); an unlisted carrier prints an error (level above 1) and selects the 1200 Hz table
 * @param preemp_s   pre-emphasis index; 0 keeps the default row (`preemp0`), 1..10 select a row of the rate's pre-emphasis table
 * @param v90        non-zero for the V.90 / K56Flex variant (only the 3200 baud arm reads it: other source table, prefilter and exponents)
 * @param fullReset  non-zero: clear the phase accumulator, the history (0x100 bytes) and the 16-tap FIR state (0x20 bytes) and finish with the index 0x40 / 0;
 *                   zero: reduce the phase accumulator and the sine index modulo the new row count and half length
 * @return `modobj` when fullReset != 0; with fullReset == 0 the quotient `[m+0x18] / [m+0x14]` converted to a pointer
 * @warning The pre-emphasis row pointer is computed from `preemp` without a range check; a value outside 1..10 points outside the table.
 * @warning With fullReset == 0 the return value is an integer quotient disguised as a pointer, and it is truncated or sign-extended on 64-bit hosts.
 *          Callers use it only as an opaque value or ignore it.
 * @note  m fields used: +0x00 N (taps per row), +0x04 step, +0x08 L (rows), +0x0c phase accumulator, +0x10 rotation-table pointer, +0x14 its half length,
 *        +0x18 sine index, +0xc24 polyphase bank pointer (read, not written), +0xc7c echo-prefilter coefficient pointer, +0xc8c prefilter shift,
 *        +0xc90 16-tap FIR state, +0xcb0 transmit pre-emphasis pointer, +0xcbc history, +0xdbc index, +0xdc0 stride.  The default pre-emphasis pointer is
 *        stored first; a non-zero `preemp` then overwrites it.  The divisions at the end have non-zero divisors ([m+8] in {3, 4, 0x10, 0x18, 0xe},
 *        [m+0x14] >= 5).  Arguments baud, carrier and preemp are compared as signed 32-bit values.
 */
int* V34SetupModulator(V34Mod* modobj, short baud_s, short carrier_s, short preemp_s, int v90, int fullReset)
{
  int baud = baud_s, carrier = carrier_s, preemp = preemp_s;
  const short* src;
  int Nn, Ll;
  short* dst;
  int r;

  if (1 < dsplibs_debug_level)
    dsplibs_debug_printf("V34SetupModulator: baudrate %ld, carrier %ld, preemp %ld, V90=%ld. fullReset=%1d\n", (long)baud, (long)carrier, (long)preemp,
                         (long)v90, fullReset);

  /* the default pre-emphasis row; a non-zero `preemp` below overwrites it */
  modobj->p2100_ = (void*)preemp0;

  if (fullReset != 0)
  {
    modobj->w145c_ = 0;
    modobj->w1468_ = 0;
    sysdep_memset(modobj->modHist_, 0, sizeof(modobj->modHist_));
    sysdep_memset(modobj->pf.pfDelay_, 0, sizeof(modobj->pf.pfDelay_));
  }

  /* per symbol rate: taps N, step, rows L, source table, echo-prefilter coefficients (+0xc7c), shift (+0xc8c) and the pre-emphasis row (+0xcb0) */
  switch (baud)
  {
  case V34_BAUD_3000:
    modobj->pf.pfShift_ = 0xf;
    modobj->modTaps_ = 0x20;
    modobj->modStep_ = 5;
    modobj->modLimit_ = 0x10;
    src = tx3000c1;
    modobj->pf.p20cc_ = (void*)((carrier == V34_CARRIER_1800) ? ec_prem_coef_B3000 : ec_prem_coef_B3000High);
    if (preemp != 0)
      modobj->p2100_ = (void*)pre_row_base(p3000, preemp);
    break;
  case V34_BAUD_2400:
    modobj->pf.pfShift_ = 0xf;
    modobj->modTaps_ = 0x20;
    modobj->modStep_ = 1;
    modobj->modLimit_ = 4;
    src = tx2400c1;
    modobj->pf.p20cc_ = (void*)((carrier == V34_CARRIER_1600) ? ec_prem_coef_B2400 : ec_prem_coef_B2400High);
    if (preemp != 0)
      modobj->p2100_ = (void*)pre_row_base(p2400, preemp);
    break;
  case V34_BAUD_2800:
    modobj->pf.pfShift_ = 0xf;
    modobj->modTaps_ = 0x20;
    modobj->modStep_ = 7;
    modobj->modLimit_ = 0x18;
    src = tx2800c1;
    modobj->pf.p20cc_ = (void*)((carrier == V34_CARRIER_1680) ? ec_prem_coef_B2800 : ec_prem_coef_B2800High);
    if (preemp != 0)
      modobj->p2100_ = (void*)pre_row_base(p2800, preemp);
    break;
  case V34_BAUD_3429:
    modobj->pf.pfShift_ = 0xf;
    modobj->pf.p20cc_ = (void*)ec_prem_coef_B3429;
    src = tx3429c1;
    modobj->modTaps_ = 0x20;
    modobj->modStep_ = 5;
    modobj->modLimit_ = 0xe;
    if (preemp != 0)
      modobj->p2100_ = (void*)pre_row_base(p3429, preemp);
    break;
  case V34_BAUD_3200:
    modobj->modStep_ = 1;
    modobj->modLimit_ = 3;
    if (v90 == 0)
    {
      modobj->pf.pfShift_ = 0xf;
      modobj->modTaps_ = 0x20;
      src = tx3200c1_for_v34;
      modobj->pf.p20cc_ = (void*)((carrier == V34_CARRIER_1829) ? ec_prem_coef_B3200 : ec_prem_coef_B3200High);
    }
    else
    {
      modobj->modTaps_ = 0x40;
      modobj->pf.p20cc_ = (void*)V90EchoPrefilterCoeff;
      modobj->pf.pfShift_ = 0xe;
      src = tx3200c1_for_v90;
    }
    if (preemp != 0)
      modobj->p2100_ = (void*)pre_row_base(p3200, preemp);
    break;
  case V34MOD_ALLPASS_4800:
    modobj->pf.pfShift_ = 0xf;
    modobj->modTaps_ = 0x20;
    modobj->modStep_ = 1;
    modobj->modLimit_ = 4;
    src = txAllPass; /* no prefilter coefficients and no pre-emphasis row for this rate */
    break;
  default:
    /* V34MOD_DPSK_600 (the 600-baud DPSK shaper) and every other baud (print only if level > 1) */
    modobj->pf.pfShift_ = 0xf;
    if (baud != V34MOD_DPSK_600 && 1 < dsplibs_debug_level)
      dsplibs_debug_printf("V34SetupModulator: invalid baudrate %ld\n", (long)baud);
    modobj->modTaps_ = 8;
    modobj->modStep_ = 1;
    modobj->modLimit_ = 0x10;
    modobj->pf.p20cc_ = (void*)ec_prem_coef_B3429;
    src = tx600c1;
    break;
  }

  /* polyphase bank: L rows of 64 shorts at the pointer stored in the object; row r is source row r (N shorts) reversed and zero-padded to 64; N and L are
   * re-read from the object (they were just stored) */
  Ll = modobj->modLimit_;
  dst = (short*)modobj->p2074_;
  if (Ll > 0)
  {
    Nn = modobj->modTaps_;
    for (r = Ll; r != 0; r--)
    {
      int j = 0;
      if (Nn > 0)
      {
        for (; j < Nn; j++)
          dst[j] = (short)(uint16_t)src[Nn - 1 - j];
        if (j > 0x3f)
          goto row_done; /* the row is full */
      }
      for (; j <= 0x3f; j++)
        dst[j] = 0;
    row_done:
      dst += 64;
      src += Nn;
    }
  }

  /* per carrier: rotation table and its half length (dwords) */
  switch (carrier)
  {
  case V34_CARRIER_1829:
    modobj->p1460_ = (void*)hsine1829;
    modobj->modLen_ = 0x15;
    break;
  case V34_CARRIER_1959:
    modobj->p1460_ = (void*)hsine1959;
    modobj->modLen_ = 0x31;
    break;
  case V34_CARRIER_1867:
    modobj->p1460_ = (void*)hsine1867;
    modobj->modLen_ = 0x24;
    break;
  case V34_CARRIER_1920:
    modobj->p1460_ = (void*)hsine1920;
    modobj->modLen_ = 5;
    break;
  case V34_CARRIER_2000:
    modobj->p1460_ = (void*)hsine2000;
    modobj->modLen_ = 0x18;
    break;
  case V34_CARRIER_2400:
    modobj->p1460_ = (void*)hsine2400;
    modobj->modLen_ = 8;
    break;
  case V34_CARRIER_1600:
    modobj->p1460_ = (void*)hsine1600;
    modobj->modLen_ = 6;
    break;
  case V34_CARRIER_1680:
    modobj->p1460_ = (void*)hsine1680;
    modobj->modLen_ = 0x28;
    break;
  case V34_CARRIER_1800:
    modobj->p1460_ = (void*)hsine1800;
    modobj->modLen_ = 0x10;
    break;
  case V34_CARRIER_1200:
    modobj->p1460_ = (void*)hsine1200;
    modobj->modLen_ = 8;
    break;
  default:
    if (1 < dsplibs_debug_level)
      dsplibs_debug_printf("V34SetupModulator: invalid carrier %ld\n", (long)carrier);
    modobj->p1460_ = (void*)hsine1200;
    modobj->modLen_ = 8;
    break;
  }

  if (fullReset != 0)
  {
    modobj->w2210_ = 0x40;
    modobj->w220c_ = 0;
    return (int*)modobj;
  }

  {
    /* partial reset: keep the phase accumulator and the sine index inside the new row count and half length (signed division, the remainders keep the
     * dividend's sign) */
    int q;
    modobj->w145c_ = modobj->w145c_ % modobj->modLimit_;
    q = modobj->w1468_ / modobj->modLen_;
    modobj->w1468_ = modobj->w1468_ % modobj->modLen_;
    return (int*)(intptr_t)q; /* the quotient, as the (opaque) return value */
  }
}

/* ===================== v34setuptxmit ===================== */

/**
 * @brief Restart the transmitter after the rates are fixed.
 * @param obj  the V.34 module object
 * @note  Sets the transmit level from the remote INFO (settxlevel with `info1c`), loads the modulator (V34SetupModulator with the final transmit baud,
 *        carrier and pre-emphasis; v90 is set for V.PCM or K56Flex mode; fullReset = 1), clears bit 0x0800 of `rxFlags_`, moves the receive state to
 *        V34HS_WAIT and the transmit state to V34HS_SSEG (a debug message is printed per change at level above 1; the second message shows the already
 *        updated receive state), clears the transmit counter, sets bit 0x200 of `txFlags_` and calls txinit().  The state names are looked up by the
 *        16-bit states without a bounds check.
 */
void v34setuptxmit(V34Obj* obj)
{
  int v90;

  /* level and modulator for the final rates */
  settxlevel(obj, (const unsigned short*)&obj->info1c);

  v90 = (obj->pcmMode_ != 0 || obj->k56Mode_ != 0) ? 1 : 0;
  V34SetupModulator(&obj->mod, (int16_t)obj->rate.baud_, (int16_t)obj->rate.carrier_, (int16_t)obj->rate.preemp_, v90, 1);

  obj->rx.rxFlags_ = (uint16_t)(obj->rx.rxFlags_ & 0xf7ffu);

  /* handshake state machines: receiver waits, transmitter sends the initial segment */
  if (obj->rxState_ != V34HS_WAIT)
  {
    if (1 < dsplibs_debug_level)
    {
      /* old rx state, new rx state, tx state, mst, k56Idx, hsSymCnt */
      dsplibs_debug_printf("V34HSHAKE: rxstate %s=>%s(tx %s, mst %s, [1]%ld, [2]%ld)\n", StateName[(int16_t)obj->rxState_], StateName[V34HS_WAIT],
                           StateName[(int16_t)obj->txState_], StateName[(int16_t)obj->mst_], (long)(int16_t)obj->tx.k56Idx_, (long)(int16_t)obj->hsSymCnt_);
    }
    obj->rxState_ = V34HS_WAIT;
  }

  if (obj->txState_ != V34HS_SSEG)
  {
    if (1 < dsplibs_debug_level)
    {
      dsplibs_debug_printf("V34HSHAKE: txstate %s=>%s(rx %s, mst %s, [1]%ld, [2]%ld)\n", StateName[(int16_t)obj->txState_], StateName[V34HS_SSEG],
                           StateName[(int16_t)obj->rxState_], StateName[(int16_t)obj->mst_], (long)(int16_t)obj->tx.k56Idx_, (long)(int16_t)obj->hsSymCnt_);
    }
    obj->txState_ = V34HS_SSEG;
  }

  obj->tx.txCnt_ = 0;
  obj->tx.txFlags_ = (uint16_t)(obj->tx.txFlags_ | 0x200u);
  txinit(obj);
}
