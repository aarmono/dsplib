/**
 * @file fpm.h
 * @brief FPM toolkit: state objects and prototypes of the shared fixed-point DSP building blocks.
 *
 * The FPM_* functions (src/fpm/fpm_*.c) are the common signal-processing blocks of the classic modems (V.21 / Bell 103, V.22, V.23, V.32, the fax modems
 * and the caller-ID receiver): AGC (fpm_agc.c), polyphase resampling filter MRF (fpm_mrf.c), tone detector MTD (fpm_mtd.c), FSK demodulator FSD
 * (fpm_fsd.c), FSK modulator FSM (fpm_fsm.c), tone generator / detector TONE (fpm_tone.c), biquad IIR filters (fpm_iir.c), and the math helpers
 * (fpm_math.c, fpm_div32.c).  The tables and default configuration blocks they use are in fpm_tables.c (declared in fpm_tables.h).
 *
 * Each block has a state struct declared here; the owning modem embeds it (or points to it) and passes it to the functions.  A configuration is a short
 * array of 32-bit words copied into the struct: a word either holds two 16-bit parameters (the anonymous unions `w` / `s[]`, little endian, `s[0]` the
 * low half) or a table handle (dsp_table_handles.h; FPM_*_init() resolves it once with cfgp_ptr()).  Buffers (histories, filter state) are allocated
 * with sysdep_malloc() by the init / create functions.
 *
 * Layout.  On a 32-bit build every struct has the byte layout the _Static_asserts below pin; on a 64-bit build the pointer members make later members
 * move, so no code may address these objects by a raw byte offset, and containers must embed the struct rather than reserve bytes for it.
 */
#ifndef DSPLIBS_PORT_FPM_OBJ_H
#define DSPLIBS_PORT_FPM_OBJ_H
#include <stddef.h>
#include <stdint.h>
#include "core/dsp_table_handles.h"

/** FPM_MRF state: polyphase resampling FIR (fpm_mrf.c). */
typedef struct FpmMrf
{
  union
  {
    int w;
    short s[2];
  } a;                ///< cfg[0]: s[0] = number of polyphase branches, s[1] = branch step per output sample
  const short* coefs; ///< cfg[1]: coefficient table (taps of a branch are `nPhases` apart)
  union
  {
    int w[2];
    short s[4];
  } b;          ///< cfg[2], cfg[3]: s[0] = numerator of the history length (history = s[0] / nPhases samples)
  short phase;  ///< input samples still to be taken before the next output
  short frac;   ///< current polyphase branch
  short wpos;   ///< write index of the newest sample in the circular history
  short buflen; ///< history length in samples
  short* buf;   ///< circular history (sysdep_malloc()ed)
} FpmMrf;

/** FPM_AGC state: block automatic gain control (fpm_agc.c). */
typedef struct FpmAgc
{
  union
  {
    int w[3];
    short s[6];
  } a;                ///< cfg[0..2]: s[0] target level, s[1] / s[2] squelch thresholds (no gain yet / gain applied), s[5] block length
  const short* alpha; ///< cfg[3]: table whose first entry is the weight of the previous level (Q15)
  const short* beta;  ///< cfg[4]: table whose first entry is the weight of the block RMS (Q15)
  int w5;             ///< cfg[5] (stored, not read by fpm_agc.c)
  int active;         ///< set by FPM_AGC_init() with `reset`
  int detect;         ///< 1 when more than half of the blocks of the last FPM_AGC_agc() call were above the squelch
  short hist;         ///< smoothed signal level
  short s22;          ///< not used
  short rcp;          ///< gain multiplier (Q15 mantissa of target / level); 0 = no gain applied yet
  short shift;        ///< left shift applied with `rcp`
  int freeze;         ///< non-zero: the gain is held (FPM_AGC_Freeze())
} FpmAgc;

/** FPM_TONE state: tone generator, band-pass detector, notch and phase-reversal finder (fpm_tone.c; field meanings in its file header). */
typedef struct FpmTone
{
  union
  {
    int w[4];
    short s[8];
  } r0;             ///< cfg[0..3]: frequency, amplitude, reversal interval, detect ratio, minimum band energy, notch pole radius
  const short* lpf; ///< cfg[4]: low-pass prototype table (ToneLPF) from which the band-pass taps are derived
  union
  {
    int w[6];
    short s[12];
  } r1;            ///< cfg[5..8] (s[0] band-pass tap count, s[4] / s[5] finder ratio / window, s[6] extra history) and the generator state (s[8] .. s[10])
  short* lpfOut;   ///< band-pass taps (sysdep_malloc()ed, r1.s[0] shorts)
  short* hist;     ///< circular input history of the band-pass FIR (sysdep_malloc()ed, r1.s[0] + r1.s[6] shorts)
  short r2[0x60];  ///< detector, notch and finder state (indices in the file header of fpm_tone.c)
  short* kfilt;    ///< coefficients of the finder's DC-rejecting pre-filter (sysdep_malloc()ed, 5 shorts)
  short* kstate;   ///< state of that pre-filter (sysdep_malloc()ed, 4 shorts)
  short* killCoef; ///< notch coefficients used by FPM_TONE_kill(): points at r2[1]
  short r3[4];     ///< notch state used by FPM_TONE_kill()
} FpmTone;

/** FPM_FSM state: FSK modulator (fpm_fsm.c). */
typedef struct FpmFsm
{
  union
  {
    int w[2];
    short s[4];
  } c;           ///< cfg[0], cfg[1]: s[0] / s[1] frequency for bit 0 / 1, s[2] samples per bit, s[3] amplitude
  short f[2];    ///< tone increments for bit 0 / bit 1, derived from c.s[0] / c.s[1]
  FpmTone* tone; ///< the nested tone generator
} FpmFsm;

/** FPM_FSD state: FSK demodulator (fpm_fsd.c). */
typedef struct FpmFsd
{
  const short* intrp; ///< cfg[0]: FIR taps applied to the input history
  union
  {
    int w;
    short s[2];
  } c1;               ///< cfg[1]: s[0] FIR tap count (history length), s[1] delay of the multiplied copy in samples
  const short* coefs; ///< cfg[2]: IIR cascade coefficients
  union
  {
    int w;
    short s[2];
  } c3; ///< cfg[3]: s[0] IIR section count, s[1] slicer threshold
  union
  {
    int w;
    short s[2];
  } c4; ///< cfg[4]: s[0] polarity of the sliced bit, s[1] samples per bit
  union
  {
    int w;
    short s[2];
  } c5;              ///< cfg[5]: s[0] per-call bit limit, s[1] length of out[]
  int w6;            ///< cfg[6] (stored, not read by fpm_fsd.c)
  short* out;        ///< filtered discriminator value per input sample of the last call (sysdep_malloc()ed, c5.s[1] shorts)
  short outCount;    ///< number of valid entries of out[] (the input count of the last FPM_FSD_demodulate() call)
  short s22;         ///< c4.s[1] >> 1 (not read)
  short* hist;       ///< circular input history (sysdep_malloc()ed, c1.s[0] shorts)
  short histPos;     ///< write index of the newest sample in the history
  short* state;      ///< IIR cascade state (sysdep_malloc()ed, 2 * c3.s[0] shorts)
  short prevBit;     ///< the last bit decided
  short bitClock;    ///< samples counted since the last emitted bit
  short mismatchRun; ///< samples that disagreed with prevBit since the last emitted bit
} FpmFsd;

/** FPM_MTD state: energy-ratio tone detector (fpm_mtd.c). */
typedef struct FpmMtd
{
  const short* coef; ///< cfg[0]: cascade coefficients (5 shorts per section)
  union
  {
    int w;
    short s[2];
  } c1; ///< cfg[1]: s[0] number of biquad sections, s[1] ratio threshold (Q15)
  union
  {
    int w;
    short s[2];
  } c2;             ///< cfg[2]: s[0] minimum input energy
  short* buf;       ///< cascade state (sysdep_malloc()ed, 2 * c1.s[0] shorts)
  short dc[2];      ///< state of the DC-blocking section (COEF_DC)
  short diffEnergy; ///< inEnergy minus the cascade-output energy, clamped at 0 (FPM_MTD_detect())
  short inEnergy;   ///< smoothed energy of the DC-blocked input (FPM_MTD_detect())
} FpmMtd;

#if UINTPTR_MAX == 0xffffffffu
_Static_assert(sizeof(FpmMrf) == 0x1c && offsetof(FpmMrf, coefs) == 4 && offsetof(FpmMrf, b) == 8 && offsetof(FpmMrf, phase) == 0x10 &&
                   offsetof(FpmMrf, buf) == 0x18,
               "FpmMrf 32-bit layout");
_Static_assert(sizeof(FpmAgc) == 0x2c && offsetof(FpmAgc, alpha) == 0xc && offsetof(FpmAgc, beta) == 0x10 && offsetof(FpmAgc, w5) == 0x14 &&
                   offsetof(FpmAgc, active) == 0x18 && offsetof(FpmAgc, detect) == 0x1c && offsetof(FpmAgc, hist) == 0x20 && offsetof(FpmAgc, rcp) == 0x24 &&
                   offsetof(FpmAgc, shift) == 0x26 && offsetof(FpmAgc, freeze) == 0x28,
               "FpmAgc 32-bit layout");
_Static_assert(sizeof(FpmTone) == 0x108 && offsetof(FpmTone, lpf) == 0x10 && offsetof(FpmTone, r1) == 0x14 && offsetof(FpmTone, lpfOut) == 0x2c &&
                   offsetof(FpmTone, hist) == 0x30 && offsetof(FpmTone, r2) == 0x34 && offsetof(FpmTone, kfilt) == 0xf4 && offsetof(FpmTone, kstate) == 0xf8 &&
                   offsetof(FpmTone, killCoef) == 0xfc && offsetof(FpmTone, r3) == 0x100,
               "FpmTone 32-bit layout");
_Static_assert(sizeof(FpmFsm) == 0x10 && offsetof(FpmFsm, f) == 8 && offsetof(FpmFsm, tone) == 0xc, "FpmFsm 32-bit layout");
_Static_assert(offsetof(FpmFsd, c1) == 4 && offsetof(FpmFsd, coefs) == 8 && offsetof(FpmFsd, c3) == 0xc && offsetof(FpmFsd, c4) == 0x10 &&
                   offsetof(FpmFsd, c5) == 0x14 && offsetof(FpmFsd, w6) == 0x18 && offsetof(FpmFsd, out) == 0x1c && offsetof(FpmFsd, outCount) == 0x20 &&
                   offsetof(FpmFsd, hist) == 0x24 && offsetof(FpmFsd, histPos) == 0x28 && offsetof(FpmFsd, state) == 0x2c &&
                   offsetof(FpmFsd, prevBit) == 0x30 && offsetof(FpmFsd, mismatchRun) == 0x34 && sizeof(FpmFsd) == 0x38,
               "FpmFsd 32-bit layout");
_Static_assert(sizeof(FpmMtd) == 0x18 && offsetof(FpmMtd, c1) == 4 && offsetof(FpmMtd, c2) == 8 && offsetof(FpmMtd, buf) == 0xc &&
                   offsetof(FpmMtd, dc) == 0x10 && offsetof(FpmMtd, inEnergy) == 0x16,
               "FpmMtd 32-bit layout");
#endif

/**
 * FPM_M(T, p, member): the member read or written as a T that is not its declared type: a `void *` view of an object-pointer member (for the allocator
 * calls), or a signed / unsigned / int-versus-short view of one member.  On a 32-bit build it is a typed access at the member's byte offset, on a 64-bit
 * build the plain member access with T ignored (the unions give the legal overlay of a word and its shorts).  Code that does not need a different view
 * writes `p->member`.
 */
#if UINTPTR_MAX == 0xffffffffu
#define FPM_M(T, p, m) (*(T*)((char*)(p) + offsetof(__typeof__(*(p)), m)))
#else
#define FPM_M(T, p, m) ((p)->m)
#endif

/**
 * FPM_TABLE_SET(p, member, T, word): store a configuration word that is a table handle into the pointer member.  On a 32-bit build it is an int store
 * (a pointer-typed store changes the order in which the operands are evaluated at -O0); on a 64-bit build the handle is resolved to the real pointer.
 */
#if UINTPTR_MAX == 0xffffffffu
#define FPM_TABLE_SET(p, m, T, w) (*(int*)((char*)(p) + offsetof(__typeof__(*(p)), m)) = (w))
#else
#define FPM_TABLE_SET(p, m, T, w) ((p)->m = (T)cfgp_ptr(w))
#endif

/*
 * Prototypes of the FPM_* toolkit, in one place.  Each follows its definition in src/fpm/fpm_*.c; FPM_atan is the one function of the toolkit that is
 * defined elsewhere (src/v32/v32_math.c); V.22, the fax modems and V.32 all call it.
 */
extern void FPM_AGC_init(FpmAgc* agc, const int* cfg, int reset);
extern void FPM_AGC_agc(FpmAgc* agc, short* samples, unsigned short count);
extern void FPM_AGC_Freeze(FpmAgc* agc);
extern void FPM_AGC_Release(FpmAgc* agc);

extern int FPM_div_32(unsigned int x, short* mant, short* shift);
extern int FPM_div(unsigned short x, short* mant, short* shift);
extern void FPM_phasor(short* ph);
extern void FPM_phasor_demod(short* ph);
extern short FPM_rms(const short* data, short count);
extern unsigned int FPM_sqrt_dp(unsigned int x);
extern void FPM_atan(short i, short q, unsigned short* angle_out); /* src/v32/v32_math.c */

extern short FPM_iir_filt(short seed, const short* coefs, short* state, short count);
extern void FPM_iir_filt_II(short* data, short* coefs, short* state, short sections, short count);
extern void FPM_iir_filt_block(short* unusedData, short* samples, short* coefs, short* state, short sections, short count);

extern void FPM_FSD_init(FpmFsd* obj, const int* cfg, int alloc);
extern unsigned int FPM_FSD_demodulate(FpmFsd* obj, unsigned short* inSamples, short* bitsOut, short count);
extern void FPM_FSD_free(FpmFsd* fsd);

extern void FPM_FSM_init(FpmFsm* fsm, const int* fsmCfg);
extern short FPM_FSM_modulate(FpmFsm* fsm, unsigned short* bits, short* out, short nBits);
extern void FPM_FSM_delete(FpmFsm* fsm);

extern int FPM_MRF_init(FpmMrf* mrf, const int* cfg, int alloc);
extern unsigned int FPM_MRF_filter(FpmMrf* mrf, short* in, short* out, short inLeft);
extern void FPM_MRF_free(FpmMrf* mrf);

extern FpmMtd* FPM_MTD_create(FpmMtd* obj, const int* cfg);
extern unsigned int FPM_MTD_detect(FpmMtd* obj, short* samples, short count);
extern void FPM_MTD_delete(FpmMtd* obj);

extern FpmTone* FPM_TONE_create(FpmTone* toneObj, const int* cfg);
extern void FPM_TONE_delete(FpmTone* toneObj);
extern int FPM_TONE_detect(FpmTone* toneObj, short* samples, short count);
extern void FPM_TONE_filter(FpmTone* toneObj, short* samples, short count);
extern int FPM_TONE_find_rev(FpmTone* toneObj, short* samples, short count);
extern int FPM_TONE_generate(FpmTone* toneObj, short* out, short count);
extern int FPM_TONE_generate2(FpmTone* toneObj, short* outCos, short* outSin, short count);
extern int FPM_TONE_generate_demod(FpmTone* toneObj, short* out, short count);
extern void FPM_TONE_kill(FpmTone* toneObj, short* samples, short count);
extern void FPM_TONE_set_freq(FpmTone* toneObj, short freq);
extern void FPM_TONE_set_scale(FpmTone* toneObj, short scale);

#endif
