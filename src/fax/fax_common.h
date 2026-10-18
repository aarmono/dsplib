/**
 * @file fax_common.h
 * @brief Typed objects and entry points of the fax infrastructure shared by the V.17, V.27ter and V.29 modems and the class 1 layer.
 *
 * - FaxFifo: a FIFO of 16-bit words (FIFO_create / FIFO_write / FIFO_read / FIFO_delete, fax_fifo_sgd.c), used by the class 1 object to queue DTE data.
 * - FaxSgd: sequence / symbol generator (SGD_*, fax_fifo_sgd.c): emits a table-driven bit sequence or fixed-pattern symbols (training and idle sequences).
 * - FaxSdm / FaxSdmV27: self-synchronising scramblers (SDM_* / SDMv27_*, fax_scrambler.c).
 * - FaxSmc / FaxSmcV17 / FaxCirc: symbol-mapping encoders (SMC_* / SMCv17_*, fax_scrambler.c) that turn scrambled bits into constellation points or phase
 *   indices, writing to a circular output descriptor.
 * - The wrappers ScrambleData*, DescrambleData*, SeedScrambler*, SetEncoder*, SetScramblerV27 (fax_modem_wrappers.c) reach the scrambler / encoder embedded
 *   in a modem object through the accessors of fax_modem_views.h.
 * Default configurations are in fax_common_tables.c.
 *
 * Layout: the objects are ordinary C structs with real pointers.  In 32-bit builds every member sits at the offset pinned by the `_Static_assert`s below;
 * in 64-bit builds the members after a pointer move, so NEVER address these objects by a raw byte offset: use the member names and sizeof().
 * The *Cfg structs hold real pointers (nothing reads them as words).
 *
 * Members documented "medium" are named from the behaviour of the code that uses them: tcmMask is the per-rate byte SetEncoderV17 installs, which the trellis
 * encoder shifts left by 8 and ORs into the output.  The members that are neither read nor of known purpose keep their offset names (w*).
 */
#ifndef DSPLIBS_PORT_FAX_OBJ_H
#define DSPLIBS_PORT_FAX_OBJ_H
#include <stddef.h>
#include <stdint.h>

/** FIFO configuration: 6 bytes. */
typedef struct FaxFifoCfg
{
  unsigned short w0, size, fill; ///< w0 (copied to FaxFifo.w0), capacity in shorts, value FIFO_read() hands out for missing data
} FaxFifoCfg;

/** FIFO of 16-bit words; all index arithmetic wraps at 16 bits. */
typedef struct FaxFifo
{
  unsigned short w0;    ///< cfg.w0
  unsigned short size;  ///< capacity in shorts
  unsigned short fill;  ///< value FIFO_read() hands out for missing data
  unsigned short pad6;  ///< padding
  unsigned short* buf;  ///< allocated 2 * size bytes, unless the caller supplied the object (then the caller's buffer)
  unsigned short count; ///< shorts stored
  unsigned short rd;    ///< read index
  unsigned short wr;    ///< write index
  unsigned short pad12; ///< padding
} FaxFifo;

/** Sequence / symbol generator configuration (13 words at 32 bits); SGD_create() copies it, SGD_control() overwrites the sequence part and the pattern part. */
typedef struct FaxSgdCfg
{
  unsigned short bits;        ///< bits per symbol (SGD_symbol_gen)
  short nbuf;                 ///< sequence length / buffer shorts (SGD_create: buffer = 2 * (nbuf + w4) - 2 bytes)
  short w4;                   ///< buffer sizing term (see nbuf)
  unsigned short w6;          ///< not used
  const unsigned short* tab;  ///< sequence table of SGD_sequence_gen
  int len;                    ///< sequence length (low 16 bits used)
  int mode;                   ///< nonzero: repeat the table forever; 0: one pass, then the constant tailValue
  int tailValue;              ///< constant of the second phase (low 16 bits)
  int pattern;                ///< bit pattern SGD_symbol_gen slices (low 16 bits)
  int patSyms;                ///< symbols per pattern (low 16 bits)
  const unsigned short* tab2; ///< second table (pattern detector; not used by the functions of this library)
  int w24;                    ///< lo16 = unsigned a (>= 0), hi16 = signed b: sets the scale, see sgd_tail() in fax_fifo_sgd.c
  int w28, w2c, w30;          ///< not used
} FaxSgdCfg;

/** Sequence / symbol generator state. */
typedef struct FaxSgd
{
  FaxSgdCfg cfg;             ///< configuration
  int w34, w38;              ///< cleared by create / control; pattern detector state, not used here
  unsigned short w3c, pad3e; ///< not used
  int w40, w44;              ///< not used
  unsigned short pass;       ///< SGD_sequence_gen: finished passes (mode != 0) / done flag (mode == 0)
  unsigned short pad4a;      ///< padding
  unsigned short pos;        ///< position in tab
  unsigned short symbit;     ///< SGD_symbol_gen: symbol index inside the pattern
  unsigned short* buf;       ///< history buffer (allocated by SGD_create() when it allocates the object, else the caller's)
  short bufn;                ///< nbuf + lo16(w24) - 1: shorts cleared in the buffer
  unsigned short scale;      ///< bits * lo16(w24) * (0x4000 - hi16(w24))
  int w58;                   ///< not used
} FaxSgd;

/** The sequence part of an SGD_control() record: copied to FaxSgdCfg.tab .. patSyms. */
typedef struct FaxSgdCtlSeq
{
  const unsigned short* tab;
  int len, mode, tailValue, pattern, patSyms;
} FaxSgdCtlSeq;

/** The pattern part of an SGD_control() record: copied to FaxSgdCfg.tab2 .. w30. */
typedef struct FaxSgdCtlPat
{
  const unsigned short* tab2;
  int w24, w28, w2c, w30;
} FaxSgdCtlPat;

/** SGD_control() record: two pointers, either may be NULL (leave that part alone). */
typedef struct FaxSgdCtl
{
  const FaxSgdCtlSeq* seq;
  const FaxSgdCtlPat* pat;
} FaxSgdCtl;

/** Scrambler configuration: bits per symbol and the two feedback taps (default SDM_CFG {4, 5, 23}). */
typedef struct FaxSdmCfg
{
  short s[3];
} FaxSdmCfg;

/** V.17 / V.29 scrambler state: 0x18 bytes on every host (same layout as the V.22 scrambler). */
typedef struct FaxSdm
{
  short n;                  ///< bits per symbol
  short tap1, tap2;         ///< feedback taps
  short pad6;               ///< not used
  unsigned int mask, nmask; ///< (1 << n) - 1, and its complement
  unsigned int state;       ///< shift register (SeedScramblerV17 / V29 write it)
  short sh1, sh2;           ///< tap1 - n, tap2 - n
} FaxSdm;

/** V.27ter scrambler configuration (default SDMv27_CFG {3}); only the first short is read. */
typedef struct FaxSdmCfg27
{
  unsigned short n;
} FaxSdmCfg27;

/** V.27ter scrambler state: 0xe bytes on every host; includes the guard against all-ones / repeating patterns. */
typedef struct FaxSdmV27
{
  unsigned short n;      ///< bits per symbol (2 or 3)
  unsigned short mask;   ///< 3 (n == 2) or 7
  unsigned short nmask;  ///< ~mask (0xfffc / 0xfff8)
  unsigned short reg;    ///< shift register (initial 0x3c)
  unsigned short cnt;    ///< length of the current run of non-repeating bits (0x21 = reset trigger)
  unsigned short f1, f2; ///< reset-guard flags (0 in the normal path)
} FaxSdmV27;

/** Symbol-mapping encoder configuration (0x2c bytes at 32 bits); SMC_init() copies it. */
typedef struct FaxSmcCfg
{
  int phaseOut;                ///< 0: complex I / Q output; else the phase-index output
  int flag;                    ///< nonzero: e = in & mask (no differential table)
  short step, mod;             ///< phase increment per symbol, modulus
  unsigned short shift, mask;  ///<
  unsigned short mask2, mask3; ///<
  const unsigned short* pmap;  ///< differential table
  const short *imap, *qmap;    ///< symbol I / Q table
  const short *cosv, *sinv;    ///< carrier phase tables (Q15)
  int w28;                     ///< not used
} FaxSmcCfg;

/** Symbol-mapping encoder state. */
typedef struct FaxSmc
{
  FaxSmcCfg cfg; ///< configuration (copied by SMC_init())
  short a;       ///< running differential value
  short b;       ///< running carrier phase
} FaxSmc;

/** Circular output descriptor of the SMC encoders (0x14 bytes at 32 bits). */
typedef struct FaxCirc
{
  short *i, *q;        ///< I / Q outputs (SMC_encoder with cfg.phaseOut == 0)
  unsigned short* buf; ///< phase-index / symbol output
  short wr, rd, lim;   ///< write index (updated), read index, limit
} FaxCirc;

/** V.17 encoder configuration (default SMCv17_CFG {0, 1}): one 32-bit word. */
typedef struct FaxSmcV17Cfg
{
  unsigned short tcmMask, w2;
} FaxSmcV17Cfg;

/** V.17 symbol-mapping encoder state: 0x14 bytes on every host (the V.17 transmit state embeds it). */
typedef struct FaxSmcV17
{
  /** cfg word, low half; the trellis encoder uses its low BYTE (signed) << 8 as an OR mask on the output (SetEncoderV17 sets 3 / 2 / 4 / 5 per rate; medium) */
  unsigned short tcmMask;
  unsigned short w2;  ///< cfg word, high half
  unsigned short w4;  ///< not used
  short x;            ///< phase accumulator (SetEncoderV17 writes it)
  short y;            ///< differential state (SMCv17_encoder_dif)
  unsigned short w0a; ///< not used
  short y2;           ///< trellis encoder state
  short st;           ///< trellis transition state
  unsigned short w10; ///< not used
  unsigned short k;   ///< bits of the symbol below the 2 trellis bits (set by the creator)
} FaxSmcV17;

#if UINTPTR_MAX == 0xffffffffu
_Static_assert(sizeof(FaxFifoCfg) == 6 && sizeof(FaxFifo) == 0x14 && offsetof(FaxFifo, buf) == 8 && offsetof(FaxFifo, count) == 0xc &&
                   offsetof(FaxFifo, rd) == 0xe && offsetof(FaxFifo, wr) == 0x10,
               "FaxFifo 32-bit layout");
_Static_assert(sizeof(FaxSgdCfg) == 0x34 && offsetof(FaxSgdCfg, tab) == 8 && offsetof(FaxSgdCfg, len) == 0xc && offsetof(FaxSgdCfg, mode) == 0x10 &&
                   offsetof(FaxSgdCfg, tailValue) == 0x14 && offsetof(FaxSgdCfg, pattern) == 0x18 && offsetof(FaxSgdCfg, patSyms) == 0x1c &&
                   offsetof(FaxSgdCfg, tab2) == 0x20 && offsetof(FaxSgdCfg, w24) == 0x24 && offsetof(FaxSgdCfg, w30) == 0x30,
               "FaxSgdCfg 32-bit layout");
_Static_assert(sizeof(FaxSgd) == 0x5c && offsetof(FaxSgd, w34) == 0x34 && offsetof(FaxSgd, w3c) == 0x3c && offsetof(FaxSgd, w40) == 0x40 &&
                   offsetof(FaxSgd, pass) == 0x48 && offsetof(FaxSgd, pos) == 0x4c && offsetof(FaxSgd, symbit) == 0x4e && offsetof(FaxSgd, buf) == 0x50 &&
                   offsetof(FaxSgd, bufn) == 0x54 && offsetof(FaxSgd, scale) == 0x56 && offsetof(FaxSgd, w58) == 0x58,
               "FaxSgd 32-bit layout");
_Static_assert(sizeof(FaxSgdCtlSeq) == 0x18 && sizeof(FaxSgdCtlPat) == 0x14 && sizeof(FaxSgdCtl) == 8, "FaxSgdCtl 32-bit layout");
_Static_assert(sizeof(FaxSdm) == 0x18 && offsetof(FaxSdm, mask) == 8 && offsetof(FaxSdm, state) == 0x10 && offsetof(FaxSdm, sh1) == 0x14 &&
                   offsetof(FaxSdm, sh2) == 0x16,
               "FaxSdm 32-bit layout");
_Static_assert(sizeof(FaxSdmV27) == 0xe && offsetof(FaxSdmV27, reg) == 6 && offsetof(FaxSdmV27, f2) == 0xc, "FaxSdmV27 32-bit layout");
_Static_assert(sizeof(FaxSmcCfg) == 0x2c && offsetof(FaxSmcCfg, pmap) == 0x14 && offsetof(FaxSmcCfg, sinv) == 0x24 && sizeof(FaxSmc) == 0x30 &&
                   offsetof(FaxSmc, a) == 0x2c && offsetof(FaxSmc, b) == 0x2e,
               "FaxSmc 32-bit layout");
_Static_assert(sizeof(FaxCirc) == 0x14 && offsetof(FaxCirc, buf) == 8 && offsetof(FaxCirc, wr) == 0xc && offsetof(FaxCirc, lim) == 0x10,
               "FaxCirc 32-bit layout");
_Static_assert(sizeof(FaxSmcV17) == 0x14 && offsetof(FaxSmcV17, x) == 6 && offsetof(FaxSmcV17, y) == 8 && offsetof(FaxSmcV17, y2) == 0xc &&
                   offsetof(FaxSmcV17, st) == 0xe && offsetof(FaxSmcV17, k) == 0x12,
               "FaxSmcV17 32-bit layout");
#endif

/**
 * @brief Create (or initialise) a FIFO.
 * @param f   NULL: allocate the object and its buffer; otherwise the caller's object, which must already carry its own buffer pointer (`buf`)
 * @param cfg configuration, or NULL for fax_fifo_cfg {0, 100, 0}
 * @return the FIFO
 * @warning The allocation results are not checked.
 */
FaxFifo* FIFO_create(FaxFifo* f, const FaxFifoCfg* cfg);
/**
 * @brief Append words to the FIFO.
 * @return number of words stored (at most the free space)
 */
int FIFO_write(FaxFifo* f, const unsigned short* src, unsigned short n);
/**
 * @brief Take up to n words from the FIFO; the rest of the n words are filled with the configured fill value.
 * @return number of words really read
 */
int FIFO_read(FaxFifo* f, unsigned short* dst, unsigned short n);
/** @brief Free the buffer AND the object (also an object the caller supplied). */
void FIFO_delete(FaxFifo* f);

/**
 * @brief Create a sequence / symbol generator.
 * @param o   NULL: allocate the object and its buffer; otherwise the caller's object
 * @param cfg configuration, or NULL for SGD_CFG
 */
FaxSgd* SGD_create(FaxSgd* o, const FaxSgdCfg* cfg);
/** @brief Replace the sequence part and / or the pattern part of the configuration and restart the generator. */
void SGD_control(FaxSgd* o, const FaxSgdCtl* ctl);
/** @brief Emit n words of the configured sequence; @return words left in the table pass (len - pos). */
short SGD_sequence_gen(FaxSgd* o, unsigned short* out, short n);
/** @brief Emit n symbols sliced from the configured bit pattern; @return patSyms - 1 minus the next symbol index. */
short SGD_symbol_gen(FaxSgd* o, unsigned short* out, short n);
/** @brief Free the generator. */
void SGD_delete(FaxSgd* o);

/** @brief Initialise a V.17 / V.29 scrambler (SDM_CFG layout configuration). */
void SDM_init(FaxSdm* s, const FaxSdmCfg* cfg);
/** @brief Scramble n symbols in place. */
void SDM_scrambler(FaxSdm* s, unsigned short* buf, unsigned short n);
/** @brief Descramble n symbols in place. */
void SDM_descrambler(FaxSdm* s, unsigned short* buf, unsigned short n);
/** @brief Initialise a V.27ter scrambler. */
void SDMv27_init(FaxSdmV27* s, const FaxSdmCfg27* cfg);
/** @brief Scramble n symbols in place (V.27ter, with the repeating-pattern guard). */
void SDMv27_scrambler(FaxSdmV27* s, unsigned short* buf, short n);
/** @brief Descramble n symbols in place (V.27ter). */
void SDMv27_descrambler(FaxSdmV27* s, unsigned short* buf, short n);

/** @brief Initialise a symbol-mapping encoder; the configuration (and so its table pointers) is copied. */
void SMC_init(FaxSmc* s, const FaxSmcCfg* cfg);
/**
 * @brief Encode n symbols.
 * @param o  output: with cfg.phaseOut != 0 the phase index goes to o->buf; with cfg.phaseOut == 0 the complex I / Q output goes to o->i / o->q, rotated by the
 *           carrier phase b
 */
void SMC_encoder(FaxSmc* s, FaxCirc* o, const unsigned short* in, unsigned short n);
/** @brief Initialise a V.17 encoder. */
void SMCv17_init(FaxSmcV17* s, const FaxSmcV17Cfg* cfg);
/** @brief V.17 encoder, absolute (non-differential) mapping. */
void SMCv17_encoder_abs(FaxSmcV17* s, FaxCirc* o, const unsigned short* in, unsigned short n);
/** @brief V.17 encoder, differential mapping. */
void SMCv17_encoder_dif(FaxSmcV17* s, FaxCirc* o, const unsigned short* in, unsigned short n);
/** @brief V.17 trellis-coded encoder; masks the input buffer in place. */
void SMCv17_encoder_tcm(FaxSmcV17* s, FaxCirc* o, unsigned short* in, unsigned short n);

/* The modem-object wrappers (fax_modem_wrappers.c): obj is the V.17 / V.27ter / V.29 modem object; the sub-object is found through fax_modem_views.h. */
void ScrambleDataV17(void* obj, unsigned short* buf, unsigned short n);
void ScrambleDataV27(void* obj, unsigned short* buf, short n);
void ScrambleDataV29(void* obj, unsigned short* buf, unsigned short n);
void DescrambleDataV17(void* obj, unsigned short* buf, unsigned short n);
void DescrambleDataV27(void* obj, unsigned short* buf, short n);
void DescrambleDataV29(void* obj, unsigned short* buf, unsigned short n);
void SeedScramblerV17(void* obj, unsigned int seed);
void SeedScramblerV29(void* obj, unsigned int seed);
void SetEncoderV17(void* obj, short which, short val);
void SetEncoderV29(void* obj, short which);
void SetScramblerV27(void* obj);

#endif
