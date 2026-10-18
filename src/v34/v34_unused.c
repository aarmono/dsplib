/**
 * @file v34_unused.c
 * @brief Sixteen V.34 helper functions that nothing in the library calls.
 *
 * None of these functions is referenced from any other file of the library or of slmodemd (checked by name over src/); they are linked in and exercised
 * only by external unit tests.  They are documented here as what each one computes, and are left unchanged.
 *
 *   setScramble, scaleVector, initG248, decision, agcadapt        scrambler hook, vector scaling, a symmetric-table builder, nearest-point decision, AGC
 *   dpskDetectInfo1Init, dpskinit                                 INFO-sequence DPSK / FSK detector state initialisation (dpskinit also sets up the
 *                                                                 600 baud transmit modulator)
 *   dftfreqinit, dftnlinitSignalBins, dftnlinitNoiseBins,
 *   dftRetrainDetInit, detectRetrainReq                           DFT bin set-up for the tone detectors and the retrain-request detector
 *   setupreceiver                                                 receive front-end set-up for a given baud rate and carrier
 *   polyValue, cosread, preempindex                               a small polynomial, a cosine-table read, the transmit pre-emphasis index search
 *
 * Callees: rxinit(), detectorinit() and V34SetupModulator() (v34_init.c / v34_setup.c), dftupdate() and dftenergy() (v34_detectors.c),
 * dsplibs_debug_printf().  The objects are reached through offset accessors (v34_access.h); a few slots are pointers written through PTRT / PTRT_rx.
 *
 * Hazards, per function (all read from the code, none run): initG248() has no bounds check on its block count; preempindex() has an unreachable
 * branch and depends on the baud being one of five values; see the functions.
 */

#include <stddef.h>
#include <stdint.h>

#include "v34_init_tables.h"   /* xyz, hsine* */
#include "v34_symbol_rates.h"  /* V34_BAUD_*, V34_CARRIER_* */
#include "v34_frame_tables.h"  /* bpv22high / bpv22low */
#include "v34_symbol_tables.h" /* costbl (== v8_costbl) */

#include "core/dsp_debug_level.h" /* dsplibs_debug_level, LEVEL_ON() */
#include "core/dsp_host.h"
#include "v34_internal.h"

#include "v34_access.h"
#include "v34_access_sub.h"

/* typed view of the V.34 object (V34Obj, member keys V34M_<off>) */
#include "v34_object_view.h"
/* 16-bit member of the frame-decoder block B (V34BlkB, key V34BLKB_<off>) */
#define U16T_blkb(o, off) (*(u16_t*)&((V34BlkB*)(o))->V34BLKB_##off)

/**
 * @brief Install the scrambler / descrambler function in the object's function-pointer slot (offset 0x448).
 * @param obj  the V.34 object
 * @param fn   the function to store
 * @note Unreferenced.  Writes a pointer through the 32-bit-slot accessor PTRT.
 */
void setScramble(void* obj, void* fn) { PTRT(obj, 0x448) = fn; }

/**
 * @brief Scale 16 shorts in place: v[i] = (short)((v[i] * g) >> 7), i = 0 .. 15.
 * @param v  vector of 16 shorts
 * @param g  gain (Q7)
 * @note Unreferenced.
 */
void scaleVector(short* v, short g)
{
  int i;
  for (i = 0; i < 8; i++)
  {
    v[2 * i] = (short)(((int)v[2 * i] * (int)g) >> 7);
    v[2 * i + 1] = (short)(((int)v[2 * i + 1] * (int)g) >> 7);
  }
}

/**
 * @brief Build the three tables of frame-decoder block B at offsets 0x48, 0x148 and 0x248 for a block count n.
 * @param blk  the block (V34BlkB); the 16-bit count n is read from offset 0x12
 *
 * With n the count and p, q the 16-bit tables at +0x48 and +0x148:
 *  - p[k] = p[2(n-1) - k] = k + 1 for k = 0 .. n-1, a triangle 1, 2, ..., n, ..., 2, 1.
 *  - q[j] = q[4(n-1) - j] = the low 16 bits of the self-convolution of p, sum over t of p[j - t] * p[t] (unsigned), for j = 0 .. 2(n-1).
 *  - The 32-bit words at +0x248 are a copy of the segment of table `xyz` that starts at xyz[n] and ends at xyz[n + 1].
 *
 * @note Unreferenced.
 * @warning No bounds check on n.  Valid values are 1 .. 18 (`xyz[n + 1]` must still be an index: xyz[19] = 945 ends the index part).  n == 0 makes the
 *          second loop run about 2^32 iterations and write far outside the block (the bound 2 * (n - 1) wraps); n == 19 would read xyz[20] = 0 and copy about
 *          2^32 words.
 */
void initG248(void* blk)
{
  uint32_t n = (uint32_t)U16T_blkb(blk, 0x12);
  uint32_t x0 = xyz[n];
  uint32_t nm1 = n - 1u;
  uint32_t k, j, i, cnt;

  /* the triangle p */
  if (n != 0)
  {
    k = 0;
    do
    {
      uint32_t v = k + 1u;
      ((V34BlkB*)blk)->shellX_[2 * nm1 - k] = (int16_t)v;
      ((V34BlkB*)blk)->shellX_[k] = (int16_t)v;
      k = v;
    } while (n > k);
  }

  /* its self-convolution q */
  j = 0;
  do
  {
    uint32_t acc = 0;
    int cntr = (int)(int16_t)(uint16_t)(j + 1u); /* 16-bit term counter */
    uint32_t hi = (uint32_t)(uint16_t)j;         /* the descending index starts at (u16)j */
    uint32_t lo = 0;
    while ((int16_t)cntr > 0)
    {
      uint32_t a = (uint32_t)((V34BlkB*)blk)->shellX_[hi];
      uint32_t b = (uint32_t)((V34BlkB*)blk)->shellX_[lo];
      acc += b * a;
      hi--;
      lo++;
      cntr = (int)(int16_t)(uint16_t)(cntr - 1);
    }
    ((V34BlkB*)blk)->shellY_[4 * nm1 - j] = (int16_t)acc;
    ((V34BlkB*)blk)->shellY_[j] = (int16_t)acc;
    j++;
  } while (j <= 2 * nm1);

  /* the table segment */
  cnt = xyz[n + 1] - x0;
  for (i = 0; i < cnt; i++)
    ((V34BlkB*)blk)->shellZ_[i] = xyz[x0 + i];
}

/**
 * @brief Nearest-point decision: find the constellation point closest to the received point.
 * @param b    the receive sub-object (V34Rx); the received point is the pair of 16-bit words at +0x210 (I) and +0x212 (Q)
 * @param pts  n points as {I, Q} pairs of shorts
 * @param n    number of points
 * @return the index of the nearest point (sign-extended 16 bits); also stored in the 16-bit word at +0x126, with the point's 32-bit word at +0x20c
 *
 * The metric is (short)((dI * dI + dQ * dQ) >> 14) with d the 16-bit differences; only a strictly smaller metric replaces the best, so the first minimum
 * wins.  An initial best of 0x7fff means a point whose metric is 0x7fff or more is never chosen (index 0 is returned then); for n <= 0 the first point's
 * word is returned as the decision.
 *
 * @note Unreferenced.
 */
int decision(void* b, short* pts, short n)
{
  short best = 0x7fff;
  short cnt = 0;
  short* bestPt = pts;
  short* p = pts;
  uint32_t ci = (uint32_t)U16T_rx(b, 0x210), cq = (uint32_t)U16T_rx(b, 0x212);
  if (n > 0)
  {
    do
    {
      short di = (short)(ci - (uint32_t)(uint16_t)p[0]);
      short dq = (short)(cq - (uint32_t)(uint16_t)p[1]);
      short m = (short)(((uint32_t)((int)di * (int)di) + (uint32_t)((int)dq * (int)dq)) >> 14);
      if (m < best)
      {
        bestPt = p;
        best = m;
      }
      cnt = (short)(cnt + 1);
      p += 2;
    } while (cnt < n);
  }
  {
    uint32_t w;
    short idx = (short)(((intptr_t)bestPt - (intptr_t)pts) >> 2);
    w = *(uint32_t*)(void*)bestPt;
    S16T_rx(b, 0x126) = idx;
    U32T_rx(b, 0x20c) = w;
    return (int)idx;
  }
}

/**
 * @brief Receive AGC adaptation step.
 * @param b  the receive sub-object (V34Rx)
 * @return always 0
 *
 * Steps (all 16-bit state in `b`):
 *  - rxLevel = rxLevel * 0x6ccd / 2^15 + the unsigned word at +0x12e (a leaky update); if the sum does not fit 16 bits signed, rxLevel is set to 0x7f00.
 *  - Unless bit 1 of the high byte of rxFlags is set: d = rxLevel - 0xfa0.  When |d| is above 0x4b0, an error integrator at +0x138 is updated:
 *    s = (word at +0x13a * d >> 16) + the integrator.  If |s| is at most 0x1f4 the integrator is stored; otherwise it is cleared and rxGain is changed one
 *    step: s > 0 multiplies it by 0x390a / 2^14 (lower gain); s <= 0 multiplies it by 0x47cf / 2^14 (higher gain) unless it is already above 0x6a00.
 *
 * @note Unreferenced.
 */
int agcadapt(void* b)
{
  int gainAcc = (int)S16T_rx(b, rxLevel);
  uint32_t gainInc = (uint32_t)U16(b, (int)offsetof(V34Rx, w0390_) + 2);
  uint32_t sum = (uint32_t)((int32_t)((uint32_t)gainAcc * 0x6ccdu) >> 15) + gainInc;
  uint32_t hi = sum >> 15;

  if (hi == 0 || hi == 0x1ffffu)
    S16T_rx(b, rxLevel) = (int16_t)sum;
  else
    S16T_rx(b, rxLevel) = 0x7f00;

  /* outside the dead zone around the target level: integrate the error and step the gain */
  if ((U8(b, ((int)offsetof(V34Rx, rxFlags_) + (1))) & 2) == 0)
  {
    int d = (int)(int16_t)(uint16_t)((uint32_t)U16T_rx(b, rxLevel) - 0xfa0u);
    int t = (d < 0) ? -d - 0x4b0 : d - 0x4b0;
    if ((int16_t)t > 0)
    {
      int prod = (int)S16T_rx(b, 0x13a) * d;
      uint32_t s = (uint32_t)(prod >> 16) + (uint32_t)U16T_rx(b, 0x138);
      int sd = (int)(int16_t)s;
      int u = (sd < 0) ? -sd - 0x1f4 : sd - 0x1f4;
      if ((int16_t)u <= 0)
      {
        S16T_rx(b, 0x138) = (int16_t)s;
        return 0;
      }
      S16T_rx(b, 0x138) = 0;
      if (sd > 0)
      {
        S16T_rx(b, rxGain) = (int16_t)(((int)S16T_rx(b, rxGain) * 0x390a) >> 14);
      }
      else
      {
        int g = (int)(int16_t)U16T_rx(b, rxGain);
        if (g > 0x6a00)
          return 0;
        S16T_rx(b, rxGain) = (int16_t)((g * 0x47cf) >> 14);
      }
    }
  }
  return 0;
}

/**
 * @brief Reset the INFO-sequence FSK / DPSK detector state of the object.
 * @param obj  the V.34 object
 *
 * Clears the 100-word buffer at +0xaae6 and sets the detector members (tap index 0x30, receive word 0xffff, bit pattern 1, period 0xc, reload and next point
 * 6, the rest 0).
 *
 * @note Unreferenced.  dpskinit() repeats the same initialisation.
 */
void dpskDetectInfo1Init(void* obj)
{
  int i;
  for (i = 0; i < 100; i++)
    U16(obj, V34OFF(0xaae6) + 2 * i) = 0;
  U16T(obj, fskTapIdx) = 0x30;
  U16T(obj, fskBias) = 0;
  U16T(obj, fskCounter) = 0;
  U16T(obj, 0xaae4) = 0;
  U16T(obj, v21RxBuf) = 0xffff;
  U16T(obj, v21BitCnt) = 0;
  U16T(obj, fskPatLow) = 1;
  U16T(obj, fskPatHigh) = 0;
  U16T(obj, fskNextPoint) = 6;
  U16T(obj, fskPeriod) = 0xc;
  U16T(obj, fskReload) = 6;
}

/**
 * @brief Initialise 25 DFT bins of 0x2c bytes each for frequency selectors 1 .. 25.
 * @param bins  array of 25 bins; bin k-1 gets frequency word (k << 8), all accumulators 0
 * @note Unreferenced.
 */
void dftfreqinit(void* bins)
{
  int k;
  for (k = 1; k <= 25; k++)
  {
    unsigned char* e = OBJ_BYTES(bins, 0x2c * (k - 1));
    U16(e, 0) = 0;
    U16(e, 2) = (uint16_t)(k << 8);
    U32(e, 4) = 0;
    U32(e, 0x10) = 0;
    U32(e, 0x14) = 0;
    U32(e, 0x18) = 0;
    U32(e, 0x1c) = 0;
    U32(e, 0x20) = 0;
    U32(e, 0x24) = 0;
    U32(e, 8) = 0;
  }
}

/**
 * @brief Initialise four DFT bins (0x2c bytes each) for the signal-tone set: frequency words 0x700, 0x900, 0xd00, 0x1100; accumulators 0.
 * @param bins  array of four bins
 * @note Unreferenced.
 */
void dftnlinitSignalBins(void* bins)
{
  int i;
  for (i = 0; i < 4; i++)
  {
    unsigned char* e = OBJ_BYTES(bins, 0x2c * i);
    U16(e, 0) = 0;
    U16(e, 2) = 0;
    U32(e, 4) = 0;
    U32(e, 8) = 0;
  }
  U16(bins, 2) = 0x700;
  U16(bins, 0x2e) = 0x900;
  U16(bins, 0x5a) = 0xd00;
  U16(bins, 0x86) = 0x1100;
}

/**
 * @brief Initialise four DFT bins (0x2c bytes each) for the noise-measurement set: frequency words 0x600, 0x800, 0xc00, 0x1000; accumulators 0.
 * @param bins  array of four bins
 * @note Unreferenced.
 */
void dftnlinitNoiseBins(void* bins)
{
  int i;
  for (i = 0; i < 4; i++)
  {
    unsigned char* e = OBJ_BYTES(bins, 0x2c * i);
    U16(e, 0) = 0;
    U16(e, 2) = 0;
    U32(e, 4) = 0;
    U32(e, 8) = 0;
  }
  U16(bins, 2) = 0x600;
  U16(bins, 0x2e) = 0x800;
  U16(bins, 0x5a) = 0xc00;
  U16(bins, 0x86) = 0x1000;
}

/**
 * @brief Initialise the retrain-request detector: three DFT bins and the detector state.
 * @param obj  the V.34 object; the bins are the three DftBin records `dftC_`
 *
 * Each bin gets the thresholds 0x50 (+0x28) and 0xbb8 (+0x2a); the frequency words are 0x600, 0x800, 0xa00.  The detector state is 1, its counter and hit
 * count 0, and the hit thresholds 3 (`rtDetThrA`) and 9 (`rtDetThrB`).
 *
 * @note Unreferenced.
 */
void dftRetrainDetInit(void* obj)
{
  unsigned char* bins = OBJ_BYTES(obj, V34OFF(dftC));
  int i;
  for (i = 0; i < 3; i++)
  {
    unsigned char* e = OBJ_BYTES(bins, 0x2c * i);
    U16(e, 0x28) = 0x50;
    U16(e, 0x2a) = 0xbb8;
    U16(e, 0) = 0;
    U32(e, 4) = 0;
    U32(e, 8) = 0;
  }
  U16(bins, 2) = 0x600;
  U16(bins, 0x2e) = 0x800;
  U16(bins, 0x5a) = 0xa00;
  U16T(obj, rtDetState) = 1;
  U32T(obj, rtDetCnt) = 0;
  U16T(obj, rtDetHits) = 0;
  U16T(obj, rtDetThrA) = 3;
  U16T(obj, rtDetThrB) = 9;
}

/**
 * @brief Feed samples to the retrain-request detector.
 * @param obj       the V.34 object (detector state initialised by dftRetrainDetInit())
 * @param nBins     number of DFT bins in use (16 bits)
 * @param samples   input samples
 * @param nSamples  number of samples (16 bits)
 * @return 1 when a retrain request is detected, otherwise 0
 *
 * Every call accumulates the samples into the bins (dftupdate()) and adds 4 to `rtDetCnt`.  When the counter equals 0x80 it is cleared, the bin energies
 * are computed (dftenergy(), shift 5), the accumulators are cleared and the two-state detector runs once:
 *  - State 1: if all three bin energies are below their thresholds (+0x28, +0x54, +0x80), the hit count is incremented; otherwise, if the hit count has
 *    reached `rtDetThrA`, the state becomes 2; the hit count is cleared.
 *  - State 2: if the energy of the middle bin (+0x38) is at most its threshold (+0x56), the hit count is cleared and the state returns to 1; otherwise the
 *    hit count is incremented, and the result is 1 when it equals `rtDetThrB`.
 *
 * @note Unreferenced.  The counter test is an equality, so it only fires if the counter hits 0x80 exactly (it starts at 0 and steps by 4).
 */
int detectRetrainReq(void* obj, short nBins, const short* samples, short nSamples)
{
  unsigned char* bins = OBJ_BYTES(obj, V34OFF(dftC));
  int cnt;

  dftupdate((DftBin*)bins, (int)nBins, samples, (int)nSamples);
  cnt = S32T(obj, rtDetCnt) + 4;
  if (cnt != 0x80)
  {
    S32T(obj, rtDetCnt) = cnt;
    return 0;
  }

  /* end of the window: energies, then clear the accumulators for the next window */
  S32T(obj, rtDetCnt) = 0;
  dftenergy((DftBin*)bins, (int)nBins, 5);
  {
    short i = 0;
    while (i < nBins)
    { /* signed 16-bit counter against the 16-bit bin count */
      unsigned char* e = OBJ_BYTES(bins, 0x2c * (int)i);
      U16(e, 0) = 0;
      U32(e, 4) = 0;
      U32(e, 8) = 0;
      i = (short)(i + 1);
    }
  }

  /* state 1: wait for quiet windows, then for the tone */
  if (S16T(obj, rtDetState) == 1)
  {
    if ((int)U16(bins, 0xc) < (int)S16(bins, 0x28) && (int)U16(bins, 0x38) < (int)S16(bins, 0x54) && (int)U16(bins, 0x64) < (int)S16(bins, 0x80))
    {
      U16T(obj, rtDetHits) = (uint16_t)(U16T(obj, rtDetHits) + 1);
      return 0;
    }
    if (!(S16T(obj, rtDetHits) < (int16_t)U16T(obj, rtDetThrA)))
      U16T(obj, rtDetState) = 2;
    U16T(obj, rtDetHits) = 0;
    return 0;
  }

  /* state 2: count consecutive windows with energy in the middle bin */
  if (S16T(obj, rtDetState) == 2)
  {
    if ((int)U16(bins, 0x38) <= (int)S16(bins, 0x56))
    {
      U16T(obj, rtDetHits) = 0;
      U16T(obj, rtDetState) = 1;
    }
    else
    {
      U16T(obj, rtDetHits) = (uint16_t)(U16T(obj, rtDetHits) + 1);
    }
    return (U16T(obj, rtDetHits) == U16T(obj, rtDetThrB)) ? 1 : 0;
  }
  return 0;
}

/**
 * @brief Set up the receive front end for the object's receive baud rate and carrier.
 * @param obj  the V.34 object
 *
 * Steps: rxinit(); read the receive baud and carrier from the object and print them (debug level above 1); store the per-baud constants in the receive
 * sub-object (words +0x1ac, +0x1ae, +0x1b0, +0x1be) and the symbol-rate dependent constant at +0x128 (always 4); select the carrier's half-sine table
 * (`hsineTab`) and its length (`hsineLen`); set the receive gain from its estimate (`rxGainEst`) and the word at +0x13a to 0x2000; clear flag bits 0x0f00;
 * initialise the tone detector with detectorinit() (coefficient table `detTab`); set flag bit 0x200.
 *
 * @note Unreferenced.  The debug level is read once after rxinit() and re-read right after the first print; the re-read value guards the second print.
 * @note A baud rate or carrier outside the handled sets leaves the corresponding constants and table pointer as rxinit() left them.
 */
void setupreceiver(void* obj)
{
  unsigned char* b = V34SUBP(obj, rx);
  int baud, car;
  unsigned int lv;

  rxinit(obj);
  baud = (int)S16T(obj, rxBaud);
  lv = dsplibs_debug_level;
  car = (int)S16T(obj, rxCarrier);
  if (lv > 1u)
  {
    dsplibs_debug_printf("V34SetupDemodulator: baudrate %ld, carrier %ld\n", (long)baud, (long)car);
    lv = dsplibs_debug_level;
  }
  S16T_rx(b, 0x128) = 4;

  /* per-baud constants */
  switch (baud)
  {
  case V34_BAUD_2800:
    S16T_rx(b, 0x1b0) = 0x3e82;
    S16T_rx(b, 0x1ae) = 0x3594;
    S16T_rx(b, 0x1be) = 0x3594;
    S16T_rx(b, 0x1ac) = 0x1f41;
    break;
  case V34_BAUD_2400:
    S16T_rx(b, 0x1b0) = 0x3e80;
    S16T_rx(b, 0x1ae) = 0x3e80;
    S16T_rx(b, 0x1be) = 0x3e80;
    S16T_rx(b, 0x1ac) = 0x1f40;
    break;
  case V34_BAUD_2743:
    S16T_rx(b, 0x1b0) = 0x3e80;
    S16T_rx(b, 0x1ae) = 0x36b0;
    S16T_rx(b, 0x1be) = 0x36b0;
    S16T_rx(b, 0x1ac) = 0x1f40;
    break;
  case V34_BAUD_3000:
    S16T_rx(b, 0x1b0) = 0x3e80;
    S16T_rx(b, 0x1ae) = 0x3200;
    S16T_rx(b, 0x1be) = 0x3200;
    S16T_rx(b, 0x1ac) = 0x1f40;
    break;
  case V34_BAUD_3200:
    S16T_rx(b, 0x1ae) = 0x2ee0;
    S16T_rx(b, 0x1b0) = 0x3e80;
    S16T_rx(b, 0x1be) = 0x2ee0;
    S16T_rx(b, 0x1ac) = 0x1f40;
    break;
  case V34_BAUD_3429:
    S16T_rx(b, 0x1b0) = 0x3e80;
    S16T_rx(b, 0x1ae) = 0x2bc0;
    S16T_rx(b, 0x1be) = 0x2bc0;
    S16T_rx(b, 0x1ac) = 0x1f40;
    break;
  default:
    break;
  }

  /* per-carrier half-sine table */
  switch (car)
  {
  case V34_CARRIER_1600:
    PTRT_rx(b, hsineTab) = (void*)hsine1600;
    S16T_rx(b, hsineLen) = 6;
    break;
  case V34_CARRIER_1680:
    PTRT_rx(b, hsineTab) = (void*)hsine1680;
    S16T_rx(b, hsineLen) = 0x28;
    break;
  case V34_CARRIER_1800:
    PTRT_rx(b, hsineTab) = (void*)hsine1800;
    S16T_rx(b, hsineLen) = 0x10;
    break;
  case V34_CARRIER_1829:
    PTRT_rx(b, hsineTab) = (void*)hsine1829;
    S16T_rx(b, hsineLen) = 0x15;
    break;
  case V34_CARRIER_1867:
    PTRT_rx(b, hsineTab) = (void*)hsine1867;
    S16T_rx(b, hsineLen) = 0x24;
    break;
  case V34_CARRIER_1920:
    PTRT_rx(b, hsineTab) = (void*)hsine1920;
    S16T_rx(b, hsineLen) = 5;
    break;
  case V34_CARRIER_1959:
    PTRT_rx(b, hsineTab) = (void*)hsine1959;
    S16T_rx(b, hsineLen) = 0x31;
    break;
  case V34_CARRIER_2000:
    PTRT_rx(b, hsineTab) = (void*)hsine2000;
    S16T_rx(b, hsineLen) = 0x18;
    break;
  default:
    break;
  }

  /* gain, flags, tone detector */
  {
    uint16_t g = U16T_rx(b, rxGainEst);
    S16T_rx(b, 0x13a) = 0x2000;
    U16T_rx(b, rxGain) = g;
    if (lv > 1u)
      dsplibs_debug_printf("V34AGC, setup receiver gain = 0x%x\n", (int)(int16_t)g);
  }
  U16T_rx(b, rxFlags) = (uint16_t)(U16T_rx(b, rxFlags) & 0xf0ffu);
  detectorinit((V34ToneDet*)V34SUBP(obj, det), PTRT(obj, detTab), 0, 8, 10, 0x600, 0);
  U16T_rx(b, rxFlags) = (uint16_t)(U16T_rx(b, rxFlags) | 0x200u);
}

/**
 * @brief Set up the DPSK INFO-sequence transmitter and detector.
 * @param obj        the V.34 object
 * @param answer     0: carrier 1200 Hz, otherwise 2400 Hz
 * @param highTaps   non-zero: shaping taps bpv22high, otherwise bpv22low (stored in the pointer slot at +0x508)
 *
 * Steps: clear the 100-word buffer at +0xaae6; V34SetupModulator() at 600 baud and the chosen carrier; store the taps pointer; initialise the detector state
 * as dpskDetectInfo1Init() does; set the receive word at +0x13a to 0x199a, rxGain from rxGainEst, flag bits 0xa00; clear the 48-word buffer `levelRing_`
 * and the words at +0x19c / +0x19e.
 *
 * @note Unreferenced.  Writes a table address into a pointer slot through PTRT (a 32-bit-only slot).
 */
void dpskinit(void* obj, short answer, short highTaps)
{
  unsigned char* b = V34SUBP(obj, rx);
  int i;

  for (i = 0; i < 100; i++)
    U16(obj, V34OFF(0xaae6) + 2 * i) = 0;
  V34SetupModulator((V34Mod*)V34SUBP(obj, mod), V34MOD_DPSK_600, (short)((answer == 0) ? V34_CARRIER_1200 : V34_CARRIER_2400), 0, 0, 1);
  PTRT(obj, 0x508) = (void*)((highTaps != 0) ? bpv22high : bpv22low);

  /* detector state, as in dpskDetectInfo1Init() */
  U16T(obj, fskTapIdx) = 0x30;
  U16T(obj, fskBias) = 0;
  U16T(obj, fskCounter) = 0;
  U16T(obj, 0xaae4) = 0;
  U16T(obj, v21RxBuf) = 0xffff;
  U16T(obj, v21BitCnt) = 0;
  U16T(obj, fskPatLow) = 1;
  U16T(obj, fskPeriod) = 0xc;
  U16T(obj, fskPatHigh) = 0;
  U16T(obj, fskNextPoint) = 6;
  U16T(obj, fskReload) = 6;

  /* receive gain and flags */
  {
    uint16_t fl = U16T_rx(b, rxFlags);
    uint16_t g = U16T_rx(b, rxGainEst);
    S16T_rx(b, 0x13a) = 0x199a;
    U16T_rx(b, rxFlags) = (uint16_t)(fl | 0xa00u);
    U16T_rx(b, rxGain) = g;
  }

  for (i = 0; i < 0x30; i++)
    AS_U16(((V34Rx*)b)->levelRing_[i]) = 0;
  U16T_rx(b, 0x19c) = 0;
  U16T_rx(b, 0x19e) = 0;
}

/**
 * @brief Evaluate the quadratic -21 x^2 + 837 x - 354, truncated to 16 bits.
 * @param x  argument
 * @return the polynomial value (sign-extended 16 bits)
 * @note Unreferenced as a function (a debug message in setInitialPhase() names it).
 */
int polyValue(short x)
{
  int v = (int)x;
  return (int)(int16_t)(v * v * -0x15 + -0x162 + v * 0x345);
}

/**
 * @brief Read entry i of the cosine table `costbl`.
 * @param i  index 0 .. 255
 * @return the table entry
 * @note Unreferenced.
 */
int cosread(unsigned char i) { return (int)costbl[i]; }

/**
 * @brief Find the transmit pre-emphasis index for a symbol rate.
 * @param obj   the V.34 object
 * @param baud  symbol rate: 3429, 3200, 3000, 2800 or 2400
 * @return the index 6 .. 10
 *
 * A per-baud 16-bit value x is read from the object (offset 0x3d4 for 3429, 0x37c for 3200, 0x350 for 3000, 0x324 for 2800 and 2400) and multiplied
 * repeatedly by a per-baud Q14 factor (0x6626, 0x639f, 0x656f, 0x6789, 0x7da7).  The index starts at 5 and is incremented before each step; the search stops
 * at the first index whose product exceeds the threshold (the 16-bit word at offset 0xbc), or at index 10 after printing "index is 10".  The chosen index is
 * printed when the debug level is above 1.
 *
 * @note Unreferenced.
 * @warning Any other baud leaves x and the factor at 0, so the product stays 0 and the search ends at index 10 unless the threshold is negative.
 * @warning The final "index is 0" print and `return 0` are unreachable: the index is incremented before the first test, so it is at least 6 at that point.
 */
int preempindex(void* obj, short baud)
{
  int x = 0, m = 0;
  int peIdx = 5;
  short thr = S16(obj, 0xbc);
  int baud32 = (int)baud;
  switch (baud)
  {
  case V34_BAUD_3429:
    x = (int)S16(obj, 0x3d4);
    m = 0x6626;
    break;
  case V34_BAUD_3200:
    x = (int)S16(obj, 0x37c);
    m = 0x639f;
    break;
  case V34_BAUD_3000:
    x = (int)S16(obj, 0x350);
    m = 0x656f;
    break;
  case V34_BAUD_2800:
    x = (int)S16(obj, 0x324);
    m = 0x6789;
    break;
  case V34_BAUD_2400:
    x = (int)S16(obj, 0x324);
    m = 0x7da7;
    break;
  default:
    break; /* x and m stay 0 */
  }

  /* multiply until the product exceeds the threshold, up to index 10 */
  for (;;)
  {
    peIdx = (int)(int16_t)(uint16_t)(peIdx + 1);
    x = (int)(int16_t)((x * m) >> 14);
    if ((short)x > thr)
      break;
    if (peIdx <= 9)
      continue;
    if (LEVEL_ON())
      dsplibs_debug_printf("V34PREEMPHASIS, - index is 10, baudrate= %d \n", baud32);
    return peIdx;
  }

  if (peIdx != 5)
  {
    if (LEVEL_ON())
      dsplibs_debug_printf("V34PREEMPHASIS, - index is %d, baudrate= %d\n", peIdx, baud32);
    return peIdx;
  }
  if (LEVEL_ON())
    dsplibs_debug_printf("V34PREEMPHASIS, - index is 0, baudrate= %d\n", baud32); /* unreachable: the index is at least 6 here */
  return 0;
}
