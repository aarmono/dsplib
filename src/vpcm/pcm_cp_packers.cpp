/**
 * @file pcm_cp_packers.cpp
 * @brief V.90 CP frame packer and the helpers around the V90MappingParams block.
 *
 * Free functions that work on the mapping-parameter block (V90MappingParams) and the additional CP info (tagV90AdditionalCPinfo), both declared in v92_cp.h:
 * - V90CPPacker() builds the V.90 CP frame, one 16-bit value (0 / 1) per bit; displaySpectralParams() prints the spectral-shaping parameters.
 * - float2Bits() (two overloads) converts a float to a fixed-point bit string against the tables fltTable2 / fltTable1.
 * - getDataBitRate() / setDataBitRate(), getConstellationsIndex(), and the constellation-mask accessors convert between the six code-point lists of the
 *   block and the 8-word bit masks used in the frame.
 *
 * Numeric conventions: the greedy bit conversion keeps the remainder in `long double` (the difference is formed in extended precision); the comparison
 * `table > value` is false for unordered operands, so NaN yields all ones, while the range checks and the sign bit treat unordered as out of range / negative.
 * The debug print of displaySpectralParams() uses truncating integer conversions and an extended-precision `fraction * 1e6`.
 *
 * Properties kept on purpose:
 * - The constellation-mask accessors take a signed group number and use `g < 6 ? g : 0`, so a negative group indexes in front of the arrays.
 * - The word index of a list element is not bounded: an element >= 128 writes past the eight mask words.
 * - The packer's CRC covers the bits from index 0x12 up to the end of the arrays, without the start bits, as 16-bit groups whose parity counts.
 * - The debug header prints "t" when `in->mode_` == 0 (sic) and the seven rows printed start at the start bits.
 */

#include "v92_cp.h"
#include "core/dsp_math.h"
#include "core/dsp_host.h"
#include "v34/v34_api.h"

extern "C"
{
}

typedef long double LD;

/* ---------------------------------------------------------------------------- data tables (copies of fltTable_1 / fltTable_2 of v92_cp.cpp) */

/** Bit weights for float2Bits(): fltTable2 for the Q3.13 value (16 entries), fltTable1 for the Q1.6 magnitude (7 entries).
 * @bug fltTable2 skips 2^-8 (0.00390625) and lists 2^-9 (0.001953125) twice, so the Q3.13 conversion has no weight for the 2^-8 bit.  The V.92 copy
 *      (fltTable_2 in v92_cp.cpp) is the same table. */
extern "C"
{
  float fltTable2[16] = {4.0f,      2.0f,       1.0f,         0.5f,         0.25f,         0.125f,         0.0625f,         0.03125f,
                         0.015625f, 0.0078125f, 0.001953125f, 0.001953125f, 0.0009765625f, 0.00048828125f, 0.000244140625f, 0.0001220703125f};
  float fltTable1[7] = {1.0f, 0.5f, 0.25f, 0.125f, 0.0625f, 0.03125f, 0.015625f};
}

/* ------------------------------------------------------------------------------------------------ helpers (static inline) */

#include "core/dsp_intmath.h"

#include "vpcm_split.h"
#include "vpcm_split_value.h"

/**
 * @brief Greedy conversion of a magnitude against a table of descending weights.
 * @param dst  bit array (16-bit entries); entry i of the table sets dst[top - i]
 * @param top  index of the most significant bit
 * @param v    magnitude
 * @param tab  weights, descending
 * @param n    number of weights
 * @note For each weight t: the bit is 0 if t > v (false for unordered, so NaN gives all ones), otherwise 1 and v = v - t in extended precision.
 */
static inline void greedyBits(short* dst, int top, LD v, const float* tab, int n)
{
  int i;
  for (i = 0; i < n; i++)
  {
    LD t = (LD)tab[i];
    if (t > v)
    {
      dst[top - i] = 0;
    }
    else
    {
      dst[top - i] = 1;
      v = v - t;
    }
  }
}

/** @brief greedyBits() writing bytes instead of 16-bit entries. */
static inline void greedyBitsU8(unsigned char* dst, int top, LD v, const float* tab, int n)
{
  int i;
  for (i = 0; i < n; i++)
  {
    LD t = (LD)tab[i];
    if (t > v)
    {
      dst[top - i] = 0;
    }
    else
    {
      dst[top - i] = 1;
      v = v - t;
    }
  }
}

/**
 * @brief Turn a list of code-point numbers into an 8-word bit mask.
 * @param out   eight mask words, cleared first; element `pos` sets bit (pos & 15) of out[pos >> 4]
 * @param list  `cnt` code-point numbers
 * @warning The word index is not bounded: an element >= 128 writes past the eight words.
 */
static inline void listToMask(unsigned short* out, const unsigned char* list, unsigned cnt)
{
  unsigned e;
  for (e = 0; e < 8; e++)
    out[e] = 0;
  for (e = 0; e < cnt; e++)
  {
    unsigned pos = list[e];
    unsigned hi = pos >> 4;
    out[hi] = (unsigned short)(out[hi] | (1u << (pos & 15)));
  }
}

/**
 * @brief Turn an 8-word bit mask into a list of code-point numbers.
 * @param list  receives the numbers; the walk is j = 7 .. 0 and bit 15 .. 0 of each word (high to low), writing 16 * j + bit for each set bit
 * @param in    eight mask words
 * @return number of elements written
 */
static inline unsigned maskToList(unsigned char* list, const short* in)
{
  unsigned cnt = 0;
  int j;
  for (j = 7; j >= 0; j--)
  {
    int w = in[j];
    int b;
    for (b = 15; b >= 0; b--)
    {
      if (w & 1)
      {
        list[cnt] = (unsigned char)(16 * j + b);
        cnt++;
      }
      w >>= 1;
    }
  }
  return cnt;
}

/** @brief Debug print of one row of 17 frame bits. */
static inline void printRow(const short* r)
{
  edprintf("%1d%1d%1d%1d%1d%1d%1d%1d%1d%1d%1d%1d%1d%1d%1d%1d%1d\r\n", r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8], r[9], r[10], r[11], r[12], r[13],
           r[14], r[15], r[16]);
}

/* ================================================================================================ data rate */

/**
 * @brief Data bit rate code of a mapping block.
 * @param mp    mapping parameters
 * @param flag  non-zero: subtract 0x14 from the stored value, else 8
 * @return `kBits_` minus the offset
 */
int getDataBitRate(V90MappingParams* mp, int flag)
{
  if (flag != 0)
    return (int)mp->kBits_ - 0x14;
  return (int)mp->kBits_ - 8;
}

/** @brief Inverse of getDataBitRate(): store rate + 0x14 (flag non-zero) or rate + 8 in `kBits_`. */
void setDataBitRate(V90MappingParams* mp, int flag, int rate)
{
  if (flag != 0)
    mp->kBits_ = (uint32_t)(rate + 0x14);
  else
    mp->kBits_ = (uint32_t)(rate + 8);
}

/* ================================================================================================ the six code-point lists */

/**
 * @brief Find the distinct code-point lists among the six groups.
 * @param mp   mapping parameters: counts `cnt_`, lists `listA_` / `listB_`; `idx_[n]` receives the first group having the n-th distinct list
 * @param out  six entries; out[g] receives the number of the distinct list that group g uses
 * @return number of distinct lists (1 .. 6)
 * @note Two groups are equal when their counts and both lists match.
 */
int getConstellationsIndex(V90MappingParams* mp, unsigned* out)
{
  unsigned n = 1;
  unsigned g;
  mp->idx_[0] = 0;
  out[0] = 0;
  for (g = 1; g <= 5; g++)
  {
    unsigned b;
    unsigned cnt = mp->cnt_[g];
    for (b = 0; b < n; b++)
    {
      unsigned d = mp->idx_[b];
      unsigned i;
      if (cnt != mp->cnt_[d])
        continue;
      for (i = 0; i < cnt; i++)
      {
        if (mp->listA_[d][i] != mp->listA_[g][i])
          break;
        if (mp->listB_[d][i] != mp->listB_[g][i])
          break;
      }
      if (i == cnt)
      {
        out[g] = out[d];
        break;
      }
    }
    if (b == n)
    {
      mp->idx_[n] = g;
      out[g] = n;
      n++;
    }
  }
  return (int)n;
}

/**
 * @brief Mask of the code points of constellation group `g` (list A).
 * @param mp   mapping parameters
 * @param g    group number; values >= 6 are treated as 0
 * @param out  eight mask words
 * @warning A negative `g` indexes in front of `idx_`.
 */
void getConstellationMask(V90MappingParams* mp, int g, unsigned short* out)
{
  int gg = (g < 6) ? g : 0;
  unsigned idx = *(mp->idx_ + gg);
  unsigned cnt = mp->cnt_[idx];
  listToMask(out, &mp->listA_[0][0] + 128 * idx, cnt);
}

/** @brief As getConstellationMask(), for the codec list (list B). */
void getCodecConstellationMask(V90MappingParams* mp, int g, unsigned short* out)
{
  int gg = (g < 6) ? g : 0;
  unsigned idx = *(mp->idx_ + gg);
  unsigned cnt = mp->cnt_[idx];
  listToMask(out, &mp->listB_[0][0] + 128 * idx, cnt);
}

/**
 * @brief Replace list A of group `g` by the code points set in an 8-word mask and set its count.
 * @param in  eight mask words
 * @warning A negative `g` indexes in front of the arrays.
 */
void setConstellationMask(V90MappingParams* mp, int g, const short* in)
{
  int gg = (g < 6) ? g : 0;
  unsigned* cnt = mp->cnt_ + gg;
  *cnt = 0;
  *cnt = maskToList(&mp->listA_[0][0] + 128 * gg, in);
}

/** @brief As setConstellationMask(), for list B. */
void setCodecConstellationMask(V90MappingParams* mp, int g, const short* in)
{
  int gg = (g < 6) ? g : 0;
  unsigned* cnt = mp->cnt_ + gg;
  *cnt = 0;
  *cnt = maskToList(&mp->listB_[0][0] + 128 * gg, in);
}

/* ================================================================================================ Q-format conversion (overloads) */

/**
 * @brief Convert a float to fixed-point bits, most significant bit at the highest index.
 * @param f     value
 * @param dst   bit array (16-bit entries)
 * @param mode  0: |f| as Q3.13 into dst[15 .. 0] against fltTable2; 1: sign into dst[7] (1 if negative or NaN) and |f| as Q1.6 into dst[6 .. 0] against
 *              fltTable1; any other mode writes nothing
 * @note Out-of-range values (Q3.13: above 8 or negative or NaN; Q1.6: outside [-1, 1] or NaN) print a "format violation" message at debug level 2 and are
 *       converted anyway.
 */
void float2Bits(float f, short* dst, int mode)
{
  if (mode == 0)
  {
    LD x = (LD)f;
    if (x > 8.0L || !(x >= 0.0L))
    {
      if (dsplibs_debug_level > 1)
        dsplibs_debug_printf("Q3.13 format violation!!\r\n");
    }
    greedyBits(dst, 15, (LD)__builtin_fabsl((LD)f), fltTable2, 16);
  }
  else if (mode == 1)
  {
    LD x = (LD)f;
    if (x > 1.0L || !(x >= -1.0L))
    {
      if (dsplibs_debug_level > 1)
        dsplibs_debug_printf("Q1.6 format violation!!\r\n");
    }
    dst[7] = (short)!((LD)f >= 0.0L); /* setb: also for unordered */
    greedyBits(dst, 6, (LD)__builtin_fabsl((LD)f), fltTable1, 7);
  }
}

/** @brief float2Bits() writing bytes, against fltTable_2 / fltTable_1 (v92_cp.h), without the range checks.  Not called by the library. */
void float2Bits(float f, unsigned char* dst, int mode)
{
  if (mode == 0)
  {
    greedyBitsU8(dst, 15, (LD)__builtin_fabsl((LD)f), fltTable_2, 16);
  }
  else if (mode == 1)
  {
    dst[7] = (unsigned char)!((LD)f >= 0.0L);
    greedyBitsU8(dst, 6, (LD)__builtin_fabsl((LD)f), fltTable_1, 7);
  }
}

/* ================================================================================================ the V.90 CP frame packer */

/**
 * @brief Pack the mapping parameters and additional CP info into a V.90 CP frame, one 16-bit value per bit.
 * @param mp         mapping parameters (rate, spectral shaping, constellation groups)
 * @param in         additional CP info (mode, SUV, flags, TRN1d RMS, ...)
 * @param out        receives the frame, sized for the frame length (at most 0x88 * 13 + 0x14 = 1788 entries)
 * @param cleardown  non-zero: send a zero data rate and print a notice at debug level 2
 * @return frame length in 16-bit entries
 * @note Frame layout (indices in hex; every index that is a multiple of 17 is a 0 start bit after the leading ones): 0-0x10 ones, 0x11-0x12 zeros,
 *       0x13 mode, 0x14-0x18 data rate (5 bits, least significant first), 0x19-0x1d zeros, 0x1e SUV, 0x1f-0x20 `sr_`, 0x21 flag, 0x23 `wc_`,
 *       0x24-0x30 `w14_` (13 bits), 0x31-0x32 `id_`, 0x34-0x43 TRN1d RMS (Q3.13), 0x45-0x65 the four spectral-shaping coefficients A1, A2, B1, B2
 *       (sign + Q1.6, 8 bits each), 0x67-0x7f the distinct-list number of each of the six groups (4 bits each), 0x80 flag that codec masks follow,
 *       0x81-0x87 zeros, then one 8-row mask (17 slots per row) per distinct list, the codec masks if flagged, a final zero, 16 CRC bits and three zeros.
 * @note The CRC register is 16 ints, each holding 0 / 1; it runs over the 16 payload bits of each 17-slot row from index 0x12, rows 0 .. P / 17 - 2, with
 *       taps at stages 3 and 10 (parity counts).
 * @warning `out` is not bounds checked.  Prints the packed frame through edprintf().
 */
int V90CPPacker(V90MappingParams* mp, tagV90AdditionalCPinfo* in, short* out, int cleardown)
{
  unsigned idxmap[6];
  int nG = getConstellationsIndex(mp, idxmap);
  int i, g, e, b, rate, P;
  int crc[16];

  /* leading ones, the mode and the data rate */
  for (i = 0; i <= 16; i++)
    out[i] = 1;
  out[0x11] = 0;
  out[0x12] = 0;
  out[0x13] = (short)in->mode_;
  if (cleardown == 0)
  {
    rate = getDataBitRate(mp, (int)in->mode_);
  }
  else
  {
    if (dsplibs_debug_level > 1)
      dsplibs_debug_printf("V90CPPacker: CLEARDOWN indicated !\r\n");
    rate = 0;
  }

  {
    int r = rate;
    for (i = 0; i < 5; i++)
    {
      out[0x14 + i] = (short)(r & 1);
      r >>= 1;
    }
  }

  /* flags, spectral-shaping rate, id and TRN1d RMS */
  for (i = 0x19; i <= 0x1d; i++)
    out[i] = 0;
  out[0x22] = 0;
  out[0x1e] = (short)in->suvVal_;
  {
    uint32_t w = mp->sr_;
    out[0x1f] = (short)(w & 1);
    out[0x20] = (short)((w >> 1) & 1);
  }
  out[0x21] = (short)in->flag_;
  out[0x23] = (short)in->wc_;
  {
    int v = (int16_t)in->w14_;
    for (i = 0; i <= 12; i++)
    {
      out[0x24 + i] = (short)(v & 1);
      v >>= 1;
    }
  }
  out[0x33] = 0;
  {
    uint32_t w = mp->id_;
    out[0x31] = (short)(w & 1);
    out[0x32] = (short)((w >> 1) & 1);
  }

  float2Bits(in->trn1dRms_, out + 0x34, 0);
  out[0x44] = 0;

  /* spectral-shaping coefficients */
  float2Bits(mp->shaperA1_, out + 0x45, 1);
  float2Bits(mp->shaperA2_, out + 0x4d, 1);
  out[0x55] = 0;
  float2Bits(mp->shaperB1_, out + 0x56, 1);
  float2Bits(mp->shaperB2_, out + 0x5e, 1);
  out[0x66] = 0;

  /* which distinct constellation list each group uses */
  for (g = 0; g < 4; g++)
  {
    int v = (int)idxmap[g];
    for (b = 0; b < 4; b++)
    {
      out[0x67 + 4 * g + b] = (short)(v & 1);
      v >>= 1;
    }
  }
  out[0x77] = 0;
  for (g = 0; g < 2; g++)
  {
    int v = (int)idxmap[4 + g];
    for (b = 0; b < 4; b++)
    {
      out[0x78 + 4 * g + b] = (short)(v & 1);
      v >>= 1;
    }
  }

  out[0x80] = (short)(mp->flag_ == 1);
  for (i = 0x81; i <= 0x87; i++)
    out[i] = 0;

  /* the constellation masks, one 8-row block per distinct list */
  for (g = 0; g < nG; g++)
  {
    unsigned short tmp[8];
    getConstellationMask(mp, g, tmp);
    for (e = 0; e < 8; e++)
    {
      unsigned v = tmp[e];
      out[0x88 + 0x88 * g + 0x11 * e] = 0;
      for (b = 0; b < 16; b++)
      {
        out[0x89 + 0x88 * g + 0x11 * e + b] = (short)(v & 1);
        v >>= 1;
      }
    }
  }
  P = 0x88 + 0x88 * nG;

  /* the codec masks, when flagged */
  if (out[0x80] != 0)
  {
    for (g = 0; g < nG; g++)
    {
      unsigned short tmp[8];
      getCodecConstellationMask(mp, g, tmp);
      for (e = 0; e < 8; e++)
      {
        unsigned v = tmp[e];
        out[P + 0x88 * g + 0x11 * e] = 0;
        for (b = 0; b < 16; b++)
        {
          out[P + 0x88 * g + 0x11 * e + 1 + b] = (short)(v & 1);
          v >>= 1;
        }
      }
    }
    P = P + 0x88 * nG;
  }
  out[P] = 0;

  /* the CRC: 16 bits per 17-slot row of the payload, rows 0 .. P / 17 - 2, stages 3 and 10 tapped (one int per stage, parity counts) */
  for (i = 0; i < 16; i++)
    crc[i] = 1;
  {
    int q = P / 17;
    int row;
    for (row = 0; row < q - 1; row++)
    {
      for (i = 0; i < 16; i++)
      {
        int fb = (int)out[0x12 + 17 * row + i] + crc[0];
        for (g = 0; g < 15; g++)
          crc[g] = crc[g + 1];
        crc[3] = (crc[3] + fb) & 1;
        crc[10] = (crc[10] + fb) & 1;
        crc[15] = fb & 1;
      }
    }
  }

  for (i = 0; i < 16; i++)
    out[P + 1 + i] = (short)crc[i];
  for (i = 0; i < 3; i++)
    out[P + 0x11 + i] = 0;

  /* debug dump */
  edprintf("V90CP packed: CP%s%s%s bits:\n", in->mode_ != 0 ? "" : "t", in->suvVal_ != 0 ? "s" : "", in->flag_ != 0 ? "'" : "");
  for (i = 0; i < 7; i++)
    printRow(out + 0x11 * (i + 1));
  return P + 0x14;
}

/* ================================================================================================ the debug dump of the spectral parameters */

/** @brief Print `sr_`, `id_` and the shaper coefficients A1, A2, B1, B2 (six decimals) through edprintf(). */
void displaySpectralParams(V90MappingParams* mp)
{
  Split s;
  edprintf("V90MappingParams: Sr = %d\r\n", mp->sr_);
  edprintf("V90MappingParams: Id = %d\r\n", mp->id_);
  s = splitValue((LD)mp->shaperA1_, (LD)1000000.0);
  edprintf("V90MappingParams: A1  = %c%d.%06d\r\n", s.sign, s.ip, s.fp);
  s = splitValue((LD)mp->shaperA2_, (LD)1000000.0);
  edprintf("V90MappingParams: A2  = %c%d.%06d\r\n", s.sign, s.ip, s.fp);
  s = splitValue((LD)mp->shaperB1_, (LD)1000000.0);
  edprintf("V90MappingParams: B1  = %c%d.%06d\r\n", s.sign, s.ip, s.fp);
  s = splitValue((LD)mp->shaperB2_, (LD)1000000.0);
  edprintf("V90MappingParams: B2  = %c%d.%06d\r\n", s.sign, s.ip, s.fp);
}
