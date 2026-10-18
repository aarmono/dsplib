/**
 * @file pcm_phase3_modulators.h
 * @brief Declarations of the V.90 / V.92 phase-3 transmit symbol generators V90Phase3Modulator and V92Phase3Modulator.
 *
 * Both classes are plain (no virtual functions).  Each is a state machine (`state_`, a symbol counter `cnt_` and an output marker `event_`) that
 * produces one 16-bit PCM-level symbol per generateSymbol() call.  The bit-valued states run a Scrambler<unsigned char, int> embedded by value
 * (`scr_`, which owns a history block) and a few linear levels derived from G.711 codes.  The owning modulators call generateSymbol() once per
 * symbol interval and, after a symbol with `event_` != 0, the exit*() functions that decide how a state ends.  Definitions are in
 * pcm_phase3_modulators.cpp; the member order is the 32-bit layout checked by pcm_phase3_modulators_layout.inc.
 *
 * V90Phase3Modulator serves V.90 and V.92 sessions (`flag_` != 0 selects generateV92Symbol()).  States 0 .. 0xf; an illegal state prints
 * "Illegal state" at debug level above 1 and returns 0:
 * - 0 Sd (period-6 pattern of levelB_ / levelC_, 0x180 symbols), 1 SdNot (0x30 symbols, then the scrambler is reset and event 1 is raised),
 * - 2 TRN1d (scrambled all-ones, 0x3e7c symbols; without a Jd bit vector it goes to state 0xf with an error print),
 * - 3 Jd (scrambled Jd frame bits, 72 per frame, until cnt_ == timeBase_ + 24804, then "Jd TimeOut" and state 0xc),
 * - V.90: 7 JdNot (finishes the current frame) and 8 (12 zero-bit symbols, then the DIL state 9, or an error print and state 0xf without a DIL),
 * - V.92: 4 V92Jd (finishes the frame, event 3), 5 V92JdPhase (time-out at cnt_ == 24804 -> state 0xd), 6 (finishes the frame), then 8,
 * - 9 DIL (the levels of the DIL descriptor, time-out at timeBase_ + 40000 -> state 0xe) and 0xa (the DIL to the end of its segment, then
 *   "Phase3 Terminated @ %d", state 0xb and event 6); states 0xb .. 0xf output 0.
 * The DIL generator is resetDILGenerator() (copies the descriptor and converts its G.711 codes to linear levels) plus one stepping function that
 * is inlined in states 9 and 0xa of both symbol generators (also exposed as generateDIL()).
 *
 * V92Phase3Modulator: the V.92 phase-3 states 0 .. 0xf (Ru 0, RuNot 1, TRN1u 3, Ja 4 / 5, silence 6, Su 7, SuNot 8, SuSecond 9, 0xa / 0xb,
 * TRN1u second part 0xc / 0xd), levelA_ = 4000 and levelB_ = 4899.
 *
 * Functions marked "unused" below are defined but no symbol generator or owner calls them.
 */
#ifndef DSP_CLASSES_P27_H
#define DSP_CLASSES_P27_H

#include "core/dsp_cxx.h"
#include "dsp/dsp_int_templates.h" /* Scrambler<unsigned char, int> */
#include "pcm_leaf_classes.h"      /* PcmType */
#include "pcm_parameters.h"        /* V90Parameters, V92Parameters */
#include "pcm_dil.h"               /* tagV90DILdescriptor */
#include "pcm_jd.h"                /* V90Jd, V92Jd */

/** States of V90Phase3Modulator::state_.  States 4 .. 6 are the V.92 Jd continuation states; 12 .. 14 are the time-out / idle states. */
enum Phase3ModulatorState
{
  P3M_SD = 0,
  P3M_SD_NOT = 1,
  P3M_TRN1D = 2,
  P3M_JD = 3,
  P3M_STATE4 = 4,
  P3M_STATE5 = 5,
  P3M_STATE6 = 6,
  P3M_JD_NOT = 7,
  P3M_JD_NOT_END = 8,
  P3M_DIL = 9,
  P3M_DIL_END = 10,
  P3M_DONE = 11,
  P3M_STATE12 = 12,
  P3M_STATE13 = 13,
  P3M_STATE14 = 14,
  P3M_ERROR = 15
};
/** States of V92Phase3Modulator::state_ (state 2 is not used). */
enum V92Phase3ModulatorState
{
  V92P3_RU = 0,
  V92P3_RU_NOT = 1,
  V92P3_STATE2 = 2,
  V92P3_TRN1U = 3,
  V92P3_JA = 4,
  V92P3_JA_SECOND = 5,
  V92P3_SILENCE = 6,
  V92P3_SU = 7,
  V92P3_SU_NOT = 8,
  V92P3_SU_SECOND = 9,
  V92P3_STATE10 = 10,
  V92P3_STATE11 = 11,
  V92P3_TRN1U_SECOND = 12,
  V92P3_STATE13 = 13,
  V92P3_STATE14 = 14,
  V92P3_STATE15 = 15
};

/** The V.92 "Ja" block handed to V92Phase3Modulator::reset(): a length word, then the bit bytes (one bit per byte, read modulo `len_`). */
struct V92Ja
{
  uint32_t len_;
  uint8_t bits_[1];
};

/**
 * @brief V.90 phase-3 transmit symbol generator (also drives V.92 sessions when `flag_` is nonzero).
 */
class V90Phase3Modulator
{
public:
  /**
   * @brief Construct the generator: scrambler (0x12, 0x17, 0x63), parameter block `p`, session flag `flag`, then reset() to mu-law, code 0x40, state 0.
   * @param p     V.90 parameter block; stored, not owned
   * @param flag  0 for a V.90 session, nonzero for V.92 (selects generateV92Symbol())
   */
  V90Phase3Modulator(V90Parameters* p, unsigned flag);
  ~V90Phase3Modulator();
  /** @brief Select the V.90 (0) or V.92 (nonzero) symbol generator. */
  void setSessionFlag(unsigned f);
  short generateSd();    ///< unused: Sd pattern symbol of the current count (the symbol generators inline it)
  short generateSdNot(); ///< unused: SdNot pattern symbol
  /**
   * @brief End-of-state decision after a Jd symbol (state 3): next state 8 (V.90) / 5 (V.92) on a 72-symbol frame boundary, else 7 / 4 to finish the
   *        frame.  Unused.
   */
  void exitJd();
  /** @brief End-of-state decision after a JdPhase symbol (state 5): next state 8 on a frame boundary, else state 6.  Unused. */
  void exitJdPhase();
  /** @brief End-of-state decision in the DIL state: go to state 0xa (finish the segment) or end the phase ("Phase3 Terminated", state 0xb, event 6). */
  void exitDIL();
  void updateCodeSegmentPointer(); ///< unused: recompute `seg_` from the current DIL level
  /**
   * @brief Load a DIL descriptor into the generator and convert its levels.
   * @param d  descriptor, or NULL to clear the DIL (`dilN_` = 0)
   * @warning The speaker / tap lists are copied `Lsp` / `Ltp` bytes (each up to 255) into the 128-byte `sp_` / `tp_` arrays; a descriptor with a list
   *          longer than 128 overruns into the neighbouring members.  The descriptor packers never produce one.
   */
  void resetDILGenerator(const tagV90DILdescriptor* d);
  short generateDIL();   ///< unused: one DIL step (inlined in states 9 and 0xa of the symbol generators)
  short generateJdNot(); ///< unused: JdNot symbol
  short generateJd();    ///< unused: Jd symbol
  short generateTRN1d(); ///< unused: TRN1d symbol
  /** @brief One symbol of a V.90 session: advance `cnt_`, dispatch on `state_`, return the 16-bit symbol. */
  short generateV90Symbol();
  short generateJdPhase(); ///< unused: JdPhase symbol
  short generateV92Jd();   ///< unused: V92Jd symbol
  /** @brief One symbol of a V.92 session (as generateV90Symbol() plus the V92Jd / V92JdPhase states 4 .. 6 and 0xd). */
  short generateV92Symbol();
  /** @brief One symbol: generateV92Symbol() when `flag_` is nonzero, else generateV90Symbol(). */
  short generateSymbol();
  /**
   * @brief Restart the generator in state `st` and run `n` symbols silently.
   * @param pcm       companding law (mu-law 0 / A-law 1) used for the levels and the DIL codes
   * @param code      G.711 code of the base level (levelA_); levelB_ uses `code + 0x10`
   * @param st        initial state
   * @param n         number of symbols to generate and discard after the restart
   * @param jd        V.90 Jd frame source (used when `flag_` == 0), may be NULL
   * @param v92jd     V.92 Jd frame source (used when `flag_` != 0), may be NULL
   * @param dil       DIL descriptor, may be NULL
   * @param timeBase  base count of the Jd / DIL time-outs
   */
  void reset(PcmType pcm, unsigned char code, Phase3ModulatorState st, unsigned n, V90Jd* jd, V92Jd* v92jd, const tagV90DILdescriptor* dil, unsigned timeBase);

  static int codeSegmentsBoundriesLookupTable[16]; ///< 8 segment limits per companding law: mu-law row 0, A-law row 1

public:
  unsigned flag_;                     ///< 0 = V.90 session, nonzero = V.92 session
  unsigned alaw_;                     ///< companding law (0 mu-law, 1 A-law)
  unsigned timeBase_;                 ///< base count of the Jd / DIL time-outs
  unsigned short levelA_;             ///< base level (from the code given to reset())
  unsigned short levelB_;             ///< second level (code + 0x10)
  unsigned char pad10_[2];            ///< padding, never touched
  unsigned short levelC_;             ///< third level (the smallest-magnitude code, 0xff mu-law / 0xd5 A-law)
  unsigned state_;                    ///< current state
  unsigned cnt_;                      ///< symbols produced in the current state
  unsigned event_;                    ///< marker set for the owner by the symbol that ends a state (0 = none)
  Scrambler<unsigned char, int> scr_; ///< scrambler of the Jd / TRN1d states
  unsigned jdBit_;                    ///< previous scrambled bit (differential sign)
  unsigned char* jdBits_;             ///< V.90 Jd frame bits (72 per frame), not owned
  unsigned char* v92JdBits_;          ///< V.92 Jd frame bits, not owned
  unsigned char* v92JdPhaseBits_;     ///< V.92 JdPhase frame bits, not owned
  V90Parameters* params_;             ///< parameter block, not owned
  unsigned char dilN_;                ///< number of entries in `to_` (0 = no DIL)
  unsigned char dilLsp_;              ///< length of the sign list `sp_`
  unsigned char dilLtp_;              ///< length of the type list `tp_`
  unsigned char sp_[128];             ///< sign list of the DIL
  unsigned char tp_[128];             ///< type list of the DIL (0 = reference level)
  unsigned char pad157_[1];           ///< padding, never touched
  unsigned segLen_[8];                ///< symbols per DIL segment
  unsigned short ref_[8];             ///< reference levels
  unsigned short to_[256];            ///< DIL levels (linear)
  unsigned char spIdx_;               ///< position in `sp_`
  unsigned char tpIdx_;               ///< position in `tp_`
  unsigned char toIdx_;               ///< position in `to_`
  unsigned char pad38b_[1];           ///< padding, never touched
  unsigned segCnt_;                   ///< symbols produced in the current DIL segment
  unsigned char seg_;                 ///< current code segment (0 .. 8)
  unsigned char pad391_[1];           ///< padding, never touched
  unsigned short isRef_;              ///< 1 when the last DIL symbol used a reference level
  unsigned char code_;                ///< G.711 code of the last DIL level
  unsigned char pad395_[3];           ///< padding, never touched
};

/**
 * @brief V.92 phase-3 transmit symbol generator (Ru, TRN1u, Ja, Su states).
 */
class V92Phase3Modulator
{
public:
  /**
   * @brief Construct the generator: scrambler (5, 0x17, 0x63), parameter block `p`, then reset(0xfa0, 0, 0, 0, 0, 0).
   * @param p  V.92 parameter block; stored, not owned
   */
  explicit V92Phase3Modulator(V92Parameters* p);
  ~V92Phase3Modulator();
  /** @brief End-of-state decision after a Ja symbol: silence (state 6) on a 12-symbol boundary, else the Ja continuation (state 5). */
  void exitJa();
  /** @brief End of the silence state: go to Su (state 7) once at least one symbol was produced. */
  void exitSilence();
  /** @brief End-of-state decision in SuSecond (state 9): state 0xb on a 12-symbol boundary, else state 0xa. */
  void exitSuSecond();
  /** @brief End of the second TRN1u part (state 0xc): go to state 0xd. */
  void exitTRN1u();
  short generateRu();    ///< unused: Ru pattern symbol
  short generateRuNot(); ///< unused: RuNot pattern symbol
  short genereteSu();    ///< unused: Su pattern symbol (the name is spelled this way)
  short genereteSuNot(); ///< unused: SuNot pattern symbol (the name is spelled this way)
  short generateJa();    ///< unused: Ja symbol
  short generateTRN1u(); ///< unused: TRN1u symbol
  /** @brief One symbol: advance `cnt_`, dispatch on `state_`, return the 16-bit symbol. */
  short generateSymbol();
  /**
   * @brief Restart the generator in state `st` and run `n` symbols silently.
   * @param unused  ignored (levelA_ is always 0xfa0)
   * @param st      initial state
   * @param n       number of symbols to generate and discard
   * @param ja      Ja bit block, may be NULL
   * @param dil     DIL descriptor; only tested for a diagnostic when `ja` is NULL
   * @param arg6    value stored in `arg6_`
   * @warning Without a Ja block `jaLen_` is 0; reaching the Ja states with `cnt_` above 0x18 then divides by zero.
   */
  void reset(short unused, V92Phase3ModulatorState st, unsigned n, V92Ja* ja, const tagV90DILdescriptor* dil, unsigned arg6);

public:
  unsigned arg6_;                     ///< sixth argument of reset(), kept
  unsigned short levelA_;             ///< base level (4000)
  unsigned short levelB_;             ///< second level (4899 = 4000 * 1.2247...)
  unsigned state_;                    ///< current state
  unsigned cnt_;                      ///< symbols produced in the current state
  unsigned trn1uLen_;                 ///< length of the first TRN1u state in symbols
  unsigned event_;                    ///< marker set for the owner by the symbol that ends a state (0 = none)
  Scrambler<unsigned char, int> scr_; ///< scrambler of the TRN1u / Ja states
  unsigned jaBit_;                    ///< previous scrambled bit (differential sign)
  unsigned char* jaBits_;             ///< Ja bits, not owned
  unsigned jaLen_;                    ///< number of Ja bits
  unsigned char pad44_[8];            ///< padding, never touched
  V92Parameters* params_;             ///< parameter block, not owned
};

typedef V90Phase3Modulator P27_V90Phase3Modulator;
typedef V92Phase3Modulator P27_V92Phase3Modulator;

#pragma GCC diagnostic ignored "-Winvalid-offsetof"
#include "pcm_phase3_modulators_layout.inc"

#endif /* DSP_CLASSES_P27_H */
