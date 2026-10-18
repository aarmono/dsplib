/**
 * @file v90_phase4_modulator.h
 * @brief V90Phase4Modulator: the V.90 / V.92 phase-4 (transmit) symbol generator, one PCM-level symbol per call.
 *
 * A plain class (no virtual functions): a state machine (state_, symbol counter cnt_, output marker event_) that produces one 16-bit PCM-level symbol per
 * generateSymbol() call.  Data symbols come from a V90BitsToSymbol (b2s_) fed with bits scrambled by the embedded Scrambler<uchar,uchar> (scr_); a session
 * with flag_ == 0 runs generateV90Symbol(), any other session generateV92Symbol().  The class is owned by V90Modulator, which creates it with the
 * mapping parameters of the TRN2d and data phases and the MP / CP frame objects, and drives it through reset(), the exit* / setMappingParams() calls and
 * generateSymbol().
 *
 * Source files: v90_phase4_modulator_symbol.cpp defines the two symbol generators generateV90Symbol() and generateV92Symbol() (the state tables); all other
 * methods are in v90_phase4_modulator.cpp.  The patterns and data steps shared by both files are in v90_phase4_patterns.h and v90_phase4_steps.h.
 * Member order is the 32-bit layout of the whole class; the generated v90_phase4_modulator_layout.inc asserts it.
 *
 * States (the debug string of each entry is the "enter ..." print):
 * - V.90: 0 Ri (until exitRi()), 1 Ri up to a symbol-period boundary, 2 RiNot (0x18 symbols), 3 TRN2d (0x3e7c symbols; resets the scrambler on entry, scrambled
 *   all-ones), then nextState_ (4 "MP": the MP bits scrambled until exitMP(); or "DIRECTLY to MPNot"), 0xd / 0xe MPNot (until mpNotSeen_ is set by the owner),
 *   0xf, 0x10 Ed (scrambled zeros, until target_ = b2s_->extra_ + 12), 0x11 B1d (scrambled ones, 0x120 symbols), 0x12 "Phase4 Terminated" (event_ = 7,
 *   data forever); 0x13 error (no output); 0x14 / 0x15 Rd (0x180 symbols of the rdrt_ pattern) and 0x16 RdNot (0x18 symbols), back to 3 TRN2d with
 *   resetNoSpectral() and a dump of the spectral parameters.
 * - V.92: 3 TRN2d, 5 SUVd (the CP bits scrambled, CP type 1) alternating with 6 / 7 CPd / 8 repeated CPd and the tag states 9 .. 0xc, then 0x10 Ed,
 *   0x11 B1d, 0x12, 0x14 .. 0x16 Rd / RdNot, 0x17 .. 0x19 silence, 0x1a Rt / 0x1b RtNot (rdrt_ pattern), 0x1c .. 0x1e Rf / RfNot (rf_ pattern).
 * - An illegal state prints "Illegal state" at debug level > 1 and yields 0.
 *
 * @note Only exitRi(), exitMP(), exitMPNot(), setMappingParams(), setNextStateAfterTRN2d(), setRdRtSymbols(), setRfSymbols(), setSessionFlag(),
 *       resetBeforRRN(), reset() and generateSymbol() are called from the library (by V90Modulator; checked with grep).  The remaining event handlers
 *       (recived*), the per-state generate*() functions and enterRepeatedCPd() have no caller on this class (the V.92 owner uses V92Phase4Modulator).
 * @warning The constructor stores only some members (see v90_phase4_modulator.cpp); state_, cnt_, event_ and the flag words are valid only after reset().
 */
#ifndef V90_PHASE4_MODULATOR_H
#define V90_PHASE4_MODULATOR_H

#include "core/dsp_cxx.h"
#include "dsp/dsp_int_templates.h" /* Scrambler<unsigned char, unsigned char> */
#include "pcm_leaf_classes.h"      /* PcmType */
#include "pcm_parameters.h"        /* V90Parameters */
#include "v90_mp.h"                /* V90MP */
#include "v90_cp.h"                /* V90CP */
#include "v92_cp.h"                /* V90MappingParams, displaySpectralParams */
#include "pcm_bits_to_symbol.h"    /* V90BitsToSymbol */

/** @brief State of the V90Phase4Modulator state machine; the values are stored in state_ / nextState_ and appear in the state tables of the file header. */
enum Phase4ModulatorState
{
  P4M_RI = 0,           ///< Ri pattern, until the owner calls exitRi()
  P4M_RI_END = 1,       ///< Ri pattern continued up to the next symbol-period boundary
  P4M_RI_NOT = 2,       ///< inverted Ri pattern, 0x18 symbols
  P4M_TRN2D = 3,        ///< TRN2d: scrambled all-ones, 0x3e7c symbols
  P4M_MP = 4,           ///< V.90: MP frame, repeated until exitMP()
  P4M_SUVD = 5,         ///< V.92: SUVd (short CP frame)
  P4M_CPD_FIRST = 6,    ///< V.92: SUVd continued up to a CP frame boundary
  P4M_CPD = 7,          ///< V.92: CPd (long CP frame)
  P4M_REPEATED_CPD = 8, ///< V.92: CPd repeated
  P4M_STATE9 = 9,       ///< V.92: CP frame running to a boundary, after a tag in CPd / repeated CPd
  P4M_STATE10 = 10,     ///< V.92: CP frame running to a boundary, after a tag in SUVd
  P4M_STATE11 = 11,     ///< V.92: final SUV frame, then Ed
  P4M_FINAL_SUVD = 12,  ///< V.92: SUVd continued to a frame boundary before the final SUV frame
  P4M_MP_LAST = 13,     ///< V.90: MP continued to an MP frame boundary
  P4M_MP_NOT = 14,      ///< V.90: MPNot, until the owner has seen the MPNot indication
  P4M_WAIT_ED = 15,     ///< V.90: MPNot continued to a frame boundary before Ed
  P4M_ED = 16,          ///< Ed: scrambled zeros until target_ symbols
  P4M_B1D = 17,         ///< B1d: scrambled ones, 0x120 symbols
  P4M_TERMINATED = 18,  ///< phase 4 finished: scrambled ones forever, event_ = 7 once
  P4M_ERROR = 19,       ///< missing MP / CP / mapping parameters: no output
  P4M_RD_WAIT = 20,     ///< waiting for the symbol queue to want bits, then Rd
  P4M_RD = 21,          ///< Rd: rdrt_ pattern, 0x180 symbols
  P4M_RD_NOT = 22,      ///< RdNot: inverted rdrt_ pattern, 0x18 symbols, then TRN2d
  P4M_STATE23 = 23,     ///< V.92: silence (RRN variant), no output
  P4M_SILENCE_A = 24,   ///< V.92: silence until symbol 0x960, then to a six-symbol boundary
  P4M_SILENCE_B = 25,   ///< V.92: silence up to a six-symbol boundary, then Rt
  P4M_RT = 26,          ///< V.92: Rt: rdrt_ pattern, 0x180 symbols
  P4M_RT_NOT = 27,      ///< V.92: RtNot: inverted rdrt_ pattern, 0x18 symbols, then SUVd
  P4M_RF_WAIT = 28,     ///< V.92: waiting for the symbol queue to want bits, then Rf
  P4M_RF = 29,          ///< V.92: Rf: rf_ pattern, 0x180 symbols
  P4M_RF_NOT = 30       ///< V.92: RfNot: inverted rf_ pattern, 0x18 symbols, then SUVd
};

/**
 * @brief The phase-4 symbol generator state machine (see the file header for the state tables).
 *
 * The methods are grouped as: construction / reset, the owner's event calls (exit* / recived*), pattern setup (setRdRtSymbols / setRfSymbols /
 * setMappingParams), the per-state generators and the two session generators.
 */
class V90Phase4Modulator
{
public:
  /**
   * @brief Create the generator.  Only the scrambler, the pointers, flag_, arg8_, w2f9c_ and w2fa0_ are initialised; call reset() before generating.
   * @param p        V.90 parameters, stored in params_ and given to a locally created V90BitsToSymbol
   * @param flag     session flag: 0 = V.90, else V.92
   * @param b2s      bit-to-symbol queue owned by the caller, or NULL to allocate a private one (capacity 0x140 symbols)
   * @param mp       MP frame object (V.90 only)
   * @param dataMp   mapping parameters of the data phase (used at the end of Ed)
   * @param trn2dMp  mapping parameters of the TRN2d phase (used after Rd / RdNot)
   * @param cp       CP frame object (V.92 only)
   * @param arg8     stored unchanged in arg8_; not read by the library
   * @warning The allocation of the private V90BitsToSymbol is not checked for NULL.
   */
  V90Phase4Modulator(V90Parameters* p, unsigned flag, V90BitsToSymbol* b2s, V90MP* mp, V90MappingParams* dataMp, V90MappingParams* trn2dMp, V90CP* cp,
                     unsigned arg8);
  /** @brief Destroy the generator; a V90BitsToSymbol it allocated itself is destroyed and freed, a caller-supplied one is left alone. */
  ~V90Phase4Modulator();
  /** @brief Start the RRN section: set the RRN flag w24_ and clear the CP bookkeeping words w2f9c_, w2fa0_, w28_, w2c_, w30_, w34_ and w20_. */
  void resetBeforRRN();
  /** @brief Switch between V.90 (0) and V.92 (non-zero) generation. */
  void setSessionFlag(unsigned f);
  /** @brief Ri pattern symbol for the current count.  No caller in the library. */
  short generateRi();
  /** @brief Inverted Ri pattern symbol.  No caller in the library. */
  short generateRiNot();
  /** @brief Enter the repeated-CPd state with a fresh CP frame.  No caller in the library. */
  void enterRepeatedCPd();
  /** @brief Rd / Rt pattern symbol.  No caller in the library. */
  short generateRdRt();
  /** @brief Inverted Rd / Rt pattern symbol.  No caller in the library. */
  short generateRdRtNot();
  /** @brief Restart the SUVd frame sequence after the second RRN section.  No caller in the library. */
  void resetRRNSecondSection();
  /** @brief Rf pattern symbol.  No caller in the library. */
  short generateRf();
  /** @brief Inverted Rf pattern symbol.  No caller in the library. */
  short generateRfNot();
  /** @brief End the MPNot state at the next MP frame boundary (V.90).  Called by V90Modulator. */
  void exitMPNot();
  /** @brief The remote SUV was received (V.92 event).  No caller in the library. */
  void recivedSUV();
  /** @brief Silence-RRN SUV, part one: mark the CP as transmitted (cptFlag_ = 1).  No caller in the library. */
  void recivedPartOneSilenceRrnSUV();
  /** @brief Silence-RRN SUV, part two: leave SUVd at a frame boundary.  No caller in the library. */
  void recivedPartTwoSilenceRrnSUV();
  /** @brief The remote CP was received (V.92 event): set w2f9c_ and mark the CP as transmitted.  No caller in the library. */
  void recivedCP();
  /** @brief The remote SUV tag was received (V.92 event).  No caller in the library. */
  void recivedSUVtag();
  /** @brief Silence-RRN SUV tag, part one.  No caller in the library. */
  void recivedPartOneSilenceRrnSUVtag();
  /** @brief Silence-RRN SUV tag, part two; same as recivedSUVtag().  No caller in the library. */
  void recivedPartTwoSilenceRrnSUVtag();
  /** @brief The remote CP tag was received (V.92 event).  No caller in the library. */
  void recivedCPtag();
  /** @brief The remote E2u was received (V.92 event): go to Ed at a frame boundary.  No caller in the library. */
  void recivedE2u();
  /** @brief The first E2u after RRN was received.  No caller in the library. */
  void recivedFirstRrnE2u();
  /** @brief Leave the silence states at a six-symbol boundary and enter Rt.  No caller in the library. */
  void exitSilence();
  /** @brief The first SUV after the second RRN part was received.  No caller in the library. */
  void recivedFirstSUVuPartTwoRrn();
  /** @brief Leave Ri at the next six-symbol boundary and enter RiNot (V.90).  Called by V90Modulator. */
  void exitRi();
  /** @brief Set the state entered after TRN2d (nextState_).  Called by V90Modulator with P4M_MP_NOT. */
  void setNextStateAfterTRN2d(Phase4ModulatorState st);
  /** @brief Leave MP at the next MP frame boundary and enter MPNot (V.90), fetching the MP frame for the next round.  Called by V90Modulator. */
  void exitMP();
  /** @brief Reset the bit-to-symbol queue with the mapping parameters `mp` (NULL: print an error at debug level, do nothing); block size becomes one symbol. */
  void setMappingParams(V90MappingParams* mp);
  /** @brief Fill the six-symbol Rd / Rt pattern rdrt_ from the first ucode of each of the six lists of `mp` (the last three negated). */
  void setRdRtSymbols(V90MappingParams* mp);
  /** @brief Fill the twelve-symbol Rf pattern rf_ from the first ucode of each of the six lists of `mp`. */
  void setRfSymbols(V90MappingParams* mp);
  /** @brief Data symbol that switches to Rf when the queue wants bits.  No caller in the library. */
  short generateDataSymbolBeforeFPE();
  /** @brief Data symbol that switches to Rd when the queue wants bits.  No caller in the library. */
  short generateDataSymbolBeforeRRN();
  /** @brief One scrambled-ones step (B1d).  No caller in the library. */
  short generateB1d();
  /** @brief One scrambled-ones step (TRN2d).  No caller in the library. */
  short generateTRN2d();
  /** @brief One scrambled-zeros step (Ed).  No caller in the library. */
  short generateEd();
  /** @brief One MP-frame step.  No caller in the library. */
  short generateMP();
  /** @brief One CP-frame step (CPd).  No caller in the library. */
  short generateCPd();
  /** @brief One CP-frame step (SUVd).  No caller in the library. */
  short generateSUVd();
  /** @brief Produce the next symbol: generateV92Symbol() when flag_ is non-zero, else generateV90Symbol(). */
  short generateSymbol();
  /**
   * @brief Restart the generator in state `st` and optionally run it forward.
   * @param pcm    codec of the stream (non-zero = A-law, 0 = mu-law)
   * @param code   G.711 code of the Ri / RiNot level
   * @param st     initial state
   * @param n      number of symbols to generate and discard after the reset (0 = none)
   * @param limit  stored in limit_ (SUVd repeat limit)
   */
  void reset(PcmType pcm, unsigned char code, Phase4ModulatorState st, unsigned n, unsigned limit);
  /** @brief Produce the next symbol of a V.90 session (one state-table step; increments cnt_, clears event_). */
  short generateV90Symbol();
  /** @brief Produce the next symbol of a V.92 session (one state-table step; increments cnt_, clears event_). */
  short generateV92Symbol();

public:
  unsigned flag_;             ///< session: 0 = V.90, else V.92
  unsigned state_;            ///< current Phase4ModulatorState
  unsigned cnt_;              ///< symbols generated in the current state
  unsigned event_;            ///< marker for the owner: 4 = TRN2d finished in a V.92 session, 7 = phase 4 terminated, else 0
  unsigned nextState_;        ///< state entered when TRN2d ends (V.90); reset() sets MP (V.90) or SUVd (V.92)
  unsigned char mpNotSeen_;   ///< set by the owner when the MPNot indication arrived; consumed in MPNot
  unsigned char pad15_[3];    ///< padding, never touched
  unsigned cpCnt_;            ///< symbols counted since the CP was marked transmitted (SUVd repeat counter)
  unsigned char cpFlag_;      ///< non-zero while cpCnt_ is counting
  unsigned char pad1d_[3];    ///< padding, never touched
  unsigned w20_;              ///< a tag / E2u event has been handled (set by the recived* handlers, cleared by reset())
  unsigned w24_;              ///< RRN section flag: set by resetBeforRRN(), cleared by reset(); with w2c_ non-zero and w30_ == 0 it sends Ed to silence
  unsigned w28_;              ///< value copied to cp_->wCa0_ when SUVd starts; cleared by reset() and set by nothing in the library
  unsigned w2c_;              ///< silence-after-Ed condition; cleared by reset() and set by nothing in the library
  unsigned w30_;              ///< second RRN section flag (set by resetRRNSecondSection() and by the owner)
  unsigned w34_;              ///< selects silence state 0x18 (0) or 0x17 (non-zero) after Ed; cleared by reset() / resetBeforRRN(), never set
  unsigned pcm_;              ///< codec: non-zero = A-law, 0 = mu-law
  unsigned short level_;      ///< linear level of the Ri pattern
  unsigned char pad3e_[2];    ///< padding, never touched
  unsigned limit_;            ///< SUVd repeat limit: cpCnt_ may reach limit_ + 0x320 before the repeated CPd state is entered
  V90BitsToSymbol* b2s_;      ///< bit-to-symbol queue (owned only when notOwned_ == 0)
  V90MP* mp_;                 ///< MP frame object (V.90)
  V90MappingParams* dataMp_;  ///< mapping parameters of the data phase
  V90MappingParams* trn2dMp_; ///< mapping parameters of the TRN2d phase
  V90CP* cp_;                 ///< CP frame object (V.92)
  Scrambler<unsigned char, unsigned char> scr_; ///< scrambler for all scrambled states
  unsigned char bits_[12000];                   ///< scrambled bits of the current step
  unsigned char* mpBits_;                       ///< bits of the current MP frame
  unsigned mpLen_;                              ///< length of mpBits_ in bits
  unsigned mpFrame_;                            ///< symbols per MP frame: 6 * mpLen_ / mp_->trn2dD_
  unsigned target_;                             ///< Ed ends when cnt_ reaches this (b2s_->extra_ + 12)
  unsigned short rdrt_[6];                      ///< Rd / Rt pattern symbols
  unsigned short rf_[12];                       ///< Rf pattern symbols
  unsigned char* cpBits_;                       ///< bits of the current CP frame
  unsigned cpLen_;                              ///< length of cpBits_ in bits
  unsigned cpFrame_;                            ///< symbols per CP frame: 6 * cpLen_ / cp_->trn2dD_
  unsigned arg8_;                               ///< constructor argument 8, stored only
  unsigned w2f9c_;                              ///< a CP was received (set by recivedCP() / recivedCPtag())
  unsigned w2fa0_;                              ///< a new CPd frame was started (set when CPd is entered)
  unsigned notOwned_;                           ///< non-zero when b2s_ was supplied by the caller
  V90Parameters* params_;                       ///< V.90 parameters
};

typedef V90Phase4Modulator P30_V90Phase4Modulator;

#pragma GCC diagnostic ignored "-Winvalid-offsetof"
#include "v90_phase4_modulator_layout.inc"

#endif /* V90_PHASE4_MODULATOR_H */
