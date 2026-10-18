/**
 * @file rcfixed_resampler.h
 * @brief RcFixed: the fixed-ratio sample-rate converter object (RcFixed_Create / _Reset / _Resample / _Delete / _Check_Combination).
 *
 * Used by the data-pump glue to convert between the host's line rate and the 8000 Hz rate of the engines (core/dp_wrapper.c, callprog/dp_call.c,
 * voice/voice_api.c).  `RcFixed *` is what RcFixed_Create() returns (NULL if idx > 999) and what Delete / Reset / Resample take.  It is a typed heap
 * object `{ int type; state pointer }` (8 bytes in the 32-bit layout, 16 bytes on x86-64), not an integer address: a glue that holds it must hold an
 * `RcFixed *`.
 *
 * Two kinds of converter, selected by `type`:
 * - type 0: a polyphase filter (create index 2 .. 19): state RcPoly = coefficient table + 200-sample history + phase accumulator.  The index selects
 *   the ratio up / down (the rcfx_fixedRc_UpFact / _DownFact tables) and the filter table; for example index 2 converts 8000 -> 9600 Hz (up 6, down 5)
 *   and index 3 converts 9600 -> 8000 Hz.
 * - type 1: a fixed 4x interpolator (create index 0) or 4x decimator (index 1): state RcDec = three small FIR stages with 3, 87 and 15 taps whose
 *   history buffers are separate heap blocks (6, 0xae and 0x1e bytes).
 *
 * Layout.  The `_Static_assert`s pin the 32-bit image layout of every member (active on 32-bit hosts only); on x86-64 the pointer members widen and the
 * members behind them move.  Nothing outside rcfixed_resampler.c and the test harness addresses these objects by offset.
 */
#ifndef DSPLIBS_PORT_RCFIXED_OBJ_H
#define DSPLIBS_PORT_RCFIXED_OBJ_H
#include <stddef.h>
#include <stdint.h>

#define RC_HIST 200 /* history length (shorts) of the type 0 state */

/** State of a type 0 (polyphase) converter. */
typedef struct RcPoly
{
  const short* coef;   ///< filter table (up * taps shorts, one of the rcfx_rcNNtoMMfilter tables)
  short hist[RC_HIST]; ///< input history
  short phase;         ///< current polyphase (0 .. up-1)
  short down;          ///< phase step per output sample (rcfx_fixedRc_DownFact[idx])
  short up;            ///< number of phases (rcfx_fixedRc_UpFact[idx])
  short taps;          ///< taps per phase
  int pos;             ///< write position in hist (taps .. 200)
  int skip;            ///< input samples to consume before the next output
} RcPoly;

/** State of a type 1 (fixed 4x) converter: three cascaded FIR stages with circular histories. */
typedef struct RcDec
{
  short mode;          ///< 0 = 4x interpolation, 1 = 4x decimation (the Create index)
  short* hist_a;       ///< 6-byte history block: the 3-tap droop stage
  const short* coef_a; ///< rcfx_CI_bDroop
  short idx_a;         ///< circular position in hist_a
  short* hist_b;       ///< 0xae-byte history block: the 87-tap stage
  const short* coef_b; ///< rcfx_CI_b1
  short idx_b;         ///< circular position in hist_b
  short* hist_c;       ///< 0x1e-byte history block: the 15-tap stage
  const short* coef_c; ///< rcfx_CI_b2
  short idx_c;         ///< circular position in hist_c
} RcDec;

/** The converter handle. */
typedef struct RcFixed
{
  int type; ///< 0 = RcPoly, 1 = RcDec
  union
  {
    RcPoly* poly;
    RcDec* dec;
    void* any;
  } state; ///< the type-specific state (NULL right after a failed RcFixed_Create())
} RcFixed;

#if UINTPTR_MAX == 0xffffffffu
_Static_assert(sizeof(RcPoly) == 0x1a4 && offsetof(RcPoly, hist) == 4 && offsetof(RcPoly, phase) == 0x194 && offsetof(RcPoly, down) == 0x196 &&
                   offsetof(RcPoly, up) == 0x198 && offsetof(RcPoly, taps) == 0x19a && offsetof(RcPoly, pos) == 0x19c && offsetof(RcPoly, skip) == 0x1a0,
               "RcPoly 32-bit layout");
_Static_assert(sizeof(RcDec) == 0x28 && offsetof(RcDec, hist_a) == 4 && offsetof(RcDec, coef_a) == 8 && offsetof(RcDec, idx_a) == 0xc &&
                   offsetof(RcDec, hist_b) == 0x10 && offsetof(RcDec, coef_b) == 0x14 && offsetof(RcDec, idx_b) == 0x18 && offsetof(RcDec, hist_c) == 0x1c &&
                   offsetof(RcDec, coef_c) == 0x20 && offsetof(RcDec, idx_c) == 0x24,
               "RcDec 32-bit layout");
_Static_assert(sizeof(RcFixed) == 8 && offsetof(RcFixed, state) == 4, "RcFixed 32-bit layout");
#endif

/**
 * @brief Create a converter.
 * @param idx  converter index: the value returned by RcFixed_Check_Combination() (2 .. 19), or 0 / 1 for the fixed 4x interpolator / decimator
 * @return the converter (release with RcFixed_Delete()); NULL when idx > 999
 * @bug    idx 20 .. 999 crashes: the common reset writes through a NULL state pointer (a memset at address 4, a SIGSEGV).  RcFixed_Check_Combination()
 *         returns 20 for "no such combination", so passing its result on unchecked crashes.
 * @warning The allocation results are not checked.
 */
RcFixed* RcFixed_Create(unsigned int idx);
/** @brief Delete a converter and its state; NULL is accepted. */
void RcFixed_Delete(RcFixed* obj);
/**
 * @brief Return a converter to its initial state.
 * @bug   For a type 1 converter only the first 4 bytes of each of the three history blocks are cleared (RcFixed_Create() has the same limit): the rest
 *        of the blocks (6, 0xae and 0x1e bytes) keeps stale data, which the first Resample calls read.  A freshly created type 1 converter therefore
 *        starts with uninitialised heap contents in its filter histories (run, with an allocator that fills new blocks with a pattern: a zero input to a new
 *        4x interpolator gave non-zero output).
 */
void RcFixed_Reset(RcFixed* obj);
/**
 * @brief Convert nIn input samples.
 * @param obj    converter, not NULL
 * @param in     input samples
 * @param nIn    number of input samples
 * @param out    output buffer
 * @param pnOut  on entry the output capacity (the conversion stops after producing that many samples; 0 = unbounded for a type 0 converter), on return
 *               the number of samples written
 * @bug   For a type 1 converter a capacity of 0 does not mean "unbounded": the capacity test is `produced >= capacity`, so the conversion stops after the
 *        first input sample group (4 outputs for the interpolator, 1 for the decimator; run).  A capacity below 4 is also exceeded by the interpolator,
 *        which writes 4 samples per input sample.
 * @bug   The type 1 decimator counts its decimation phase in locals: a call whose input length is not a multiple of 4 forgets the partial group's phase
 *        while keeping the filter state, so a block size that is not a multiple of 4 shifts the output samples (run: 8 samples in one call give 2
 *        outputs, in calls of 3 + 5 samples give 1).
 */
void RcFixed_Resample(RcFixed* obj, const short* in, unsigned int nIn, short* out, unsigned int* pnOut);
/**
 * @brief Find the converter index for the rate pair (a, b).
 * @param a  input rate (any common unit)
 * @param b  output rate (same unit)
 * @return the index (2 .. 19) of the combination with rcfx_fixedRc_DownFact[i] == a / g and rcfx_fixedRc_UpFact[i] == b / g, g = gcd(a, b); 20 if there is none
 * @warning a = b = 0 divides by zero (SIGFPE), and so does INT_MIN with -1.
 */
int RcFixed_Check_Combination(int a, int b);
#endif
