/**
 * @file v90_phase3_demod.h
 * @brief V90Phase3Demodulator: the phase-3 (line probing) receive state machine of V.90 / V.92 (class declaration and state enum).
 *
 * The object is driven one PCM sample per call by V90Equalizer::process() through getDecision().  getDecision() dispatches on `session_` to
 * getV90Decision() (V.90, v90_phase3_demod_v90_decision.cpp) or getV92Decision() (V.92, v90_phase3_demod_v92_decision.cpp).  Both run the same state
 * machine, with different Jd frame handling (V90Jd versus V92Jd / V92JdPhase), over the states of Phase3DemodulatorState: detection of the Sd tone and its
 * inversion (SdNot), TRN1d with known data and with data decoded from the line (DD), the study of the reference ucode, the Jd frame, the three stages of the
 * digital impairment learning (DIL) with error relaxation, probing DIL, and, for the V.92 quick connect, the QTS / QTSNot and ANSpcm states.  The return
 * value is the DECISION symbol: the reference level (16-bit linear PCM) that the equalizer compares the soft value with.  The side effects are `state_`,
 * `cnt_` (symbols spent in the state) and `event_` (the V90DemodState code that V90Demodulator::progress() and the equalizer read).
 *
 * Owned state: a V90Phase3Modulator `mod_` (the probing signal the far end sends, whose generateSymbol() provides the reference symbol), a
 * SerialDifferentialDecoder and a Descrambler (the Jd bit decoding), a heap V90SdDetector and a heap ANSamToneDetector.  The class refers to, and does not
 * own, the parameter block, the digital impairment detector (adid_), the Jd objects and the DIL descriptor.  The class is plain (no virtual functions) and
 * V90Demodulator embeds it.  The 32-bit layout of the whole class is pinned by the generated v90_phase3_demod_layout.inc.
 *
 * Source files: v90_phase3_demod.cpp (construction, reset, owner events, getDecision), the two decision files, and v90_phase3_demod_helpers.h (the small
 * inline helpers the decision files share).
 */
#ifndef V90_PHASE3_DEMOD_H
#define V90_PHASE3_DEMOD_H

#include "core/dsp_cxx.h"
#include "v90_demod_state.h"        /* V90DemodState: the event / outcome codes of V90Demodulator::state_ and the event_ words of the phase objects */
#include "dsp/dsp_int_templates.h"  /* SerialDifferentialDecoder<int>, Descrambler<int, int> */
#include "dsp/dsp_tone_detectors.h" /* ANSamToneDetector / GenericToneDetector */
#include "pcm_leaf_classes.h"       /* PcmType, V90SdDetector */
#include "pcm_parameters.h"         /* V90Parameters */
#include "pcm_dil.h"                /* tagV90DILdescriptor */
#include "pcm_jd.h"                 /* V90Jd, V92Jd */
#include "v90_adid.h"               /* V90AutoDigitalImpDetector */
#include "v90_spectral_shaper.h"    /* V90SpectralVerifier */
#include "pcm_phase3_modulators.h"  /* V90Phase3Modulator */

/**
 * States of V90Phase3Demodulator::state_.  The names follow the debug strings the state machine prints.  Both decision functions use the same numbers;
 * P3D_STATE6 is the V.90 wait for the Jd frame, P3D_WAIT_FOR_V92JD / P3D_V92JD_PHASE are its V.92 counterparts (the V.90 function returns 0 for those).
 */
enum Phase3DemodulatorState
{
  P3D_WAIT_FOR_SD = 0,       ///< wait for the Sd detector to fire
  P3D_SD_DEMOD = 1,          ///< Sd detected, wait for its inversion (SdNot)
  P3D_SD_NOT = 2,            ///< SdNot detected, wait 0x30 symbols, then start TRN1d
  P3D_TRN1D_KNOWN = 3,       ///< TRN1d with known data: the decision is the modulator's own symbol
  P3D_TRN1D_DD = 4,          ///< TRN1d with data decoded from the line (DD)
  P3D_V92_STUDY_UREF = 5,    ///< study of the reference ucode during TRN1d (both V.90 and V.92)
  P3D_STATE6 = 6,            ///< V.90: wait for the V90Jd frame
  P3D_WAIT_FOR_V92JD = 7,    ///< V.92: wait for the V92Jd frame
  P3D_V92JD_PHASE = 8,       ///< V.92: V92JdPhase frame
  P3D_JD_DEMOD = 9,          ///< Jd demodulation, with the JdNot detector
  P3D_FIRST_STUDY = 10,      ///< DIL first study
  P3D_SECOND_STUDY = 11,     ///< DIL second study
  P3D_THIRD_STUDY = 12,      ///< DIL third study
  P3D_QC_FIRST_STUDY = 13,   ///< quick connect: DIL first study
  P3D_QC_SECOND_STUDY = 14,  ///< quick connect: DIL second study
  P3D_QC_THIRD_STUDY = 15,   ///< quick connect: DIL third study
  P3D_ERROR_RELAXATION = 16, ///< DIL error relaxation
  P3D_PROBING_DIL = 17,      ///< probing DIL (decisions are the received samples)
  P3D_STATE18 = 18,          ///< not used by the state machines (no output)
  P3D_TERMINATED = 19,       ///< phase 3 over (no output)
  P3D_WAIT_SD_TIMEOUT = 20,  ///< error: Sd not found in time (no output)
  P3D_SD_TIMEOUT = 21,       ///< error: SdNot not found in time (no output)
  P3D_TRN1D_TIMEOUT = 22,    ///< error: TRN1d too long (no output)
  P3D_JD_TIMEOUT = 23,       ///< error: Jd phase too long (no output)
  P3D_DIL_TIMEOUT = 24,      ///< error: DIL too long (no output)
  P3D_NULL_JD = 25,          ///< error: no Jd object (no output)
  P3D_WAIT_QTS = 26,         ///< V.92 quick connect: wait for QTS (the Sd detector) while the ANSpcm detector runs
  P3D_WAIT_QTS_NOT = 27,     ///< wait for QTSNot
  P3D_QTS_NOT_DONE = 28,     ///< QTSNot seen, wait 0x30 symbols, then demodulate ANSpcm
  P3D_ANSPCM_DEMOD = 29,     ///< ANSpcm demodulation (fixed length)
  P3D_ANSPCM_WAIT = 30,      ///< wait for the ANSpcm energy drop (entered from outside by enterWaitForANSpcmDrop())
  P3D_STATE31 = 31,          ///< idle after the ANSpcm drop
  P3D_ANSPCM_DROP = 32,      ///< QTS timed out: wait for ANSpcm
  P3D_ANSPCM_RECOVERY = 33   ///< QTSNot timed out: ANSpcm recovery
};

/** The phase-3 receive state machine.  See the file comment. */
class V90Phase3Demodulator
{
public:
  /**
   * @brief Construct the object and reset() it to PCM_MU_LAW, code 0x40, P3D_WAIT_FOR_SD.
   * @param p        parameter block (borrowed)
   * @param sv       unused
   * @param session  0 = V.90, non-zero = V.92
   * @param adid     digital impairment detector (borrowed)
   * @warning The sysdep_malloc() results for the Sd and ANSam detectors are not checked.
   */
  V90Phase3Demodulator(V90Parameters* p, V90SpectralVerifier* sv, unsigned session, V90AutoDigitalImpDetector* adid);
  /** @brief Destroy and free the Sd and ANSam detectors (nothing is cleared). */
  ~V90Phase3Demodulator();
  /**
   * @brief Restart the phase: clear counters and detectors and enter state `st`.
   * @param pcm       G.711 law of the line (PcmType; the law decides the ucode to level conversions)
   * @param code      reference ucode of the DIL signal
   * @param st        initial state (P3D_WAIT_FOR_SD, P3D_TRN1D_KNOWN or P3D_WAIT_QTS; anything else prints "IRREGULAR")
   * @param n         initial symbol count (used for P3D_TRN1D_KNOWN, else the modulator starts at 0)
   * @param jd        V.90 Jd object, reset if non-NULL
   * @param jd92      V.92 Jd object, reset if non-NULL
   * @param dil       DIL descriptor, used for the length of the probing DIL
   * @param arg8      passed to V90AutoDigitalImpDetector::reset()
   * @param arg9      stored in arg9_
   * @param arg10     stored in arg10_
   * @param timeBase  added to the Sd and Jd time-outs
   */
  void reset(PcmType pcm, unsigned char code, Phase3DemodulatorState st, unsigned n, V90Jd* jd, V92Jd* jd92, tagV90DILdescriptor* dil, short arg8, short arg9,
             float arg10, unsigned timeBase);
  /** @brief Clear the Jd-not run counter.  Not called. */
  void resetJdNotDetector();
  /** @brief Clear verify_ (the quick-connect line verification result). */
  void clearVerificationStatus();
  /** @brief Set the session flag (0 V.90, else V.92) of this object and of the modulator. */
  void setSessionFlag(unsigned f);
  /** @brief Leave the DIL states: print, tell the modulator to exit DIL and, when it reports the end (event 6), terminate phase 3 (other states: no effect). */
  void exitDIL();
  /** @brief Apply the digital impairment info: max ucode from impairInfo_, then pad gain search and application in the detector. */
  void setDigitalImairmentsInfo();
  /** @brief Advance the frame position phase_ by one (0 .. 5).  Not called. */
  void incrementFramePosition();
  /** @brief Copy the phase-4 mean-error update-ratio threshold of robbed-bit signalling into the active one.  Not called. */
  void setAltRbsParams();
  /** @brief Return the detector's per-phase max ucode array. */
  unsigned char* getMaxUcode();
  /** @brief The JdNot detector as a function (the decision functions inline it): 1 when more than 11 zero bits were seen and cnt_ % 72 == 12.  Not called. */
  int JdNotDetector(int bit);
  /** @brief Enter P3D_ANSPCM_WAIT (waiting for the ANSpcm energy drop) unless already there; restarts cnt_. */
  void enterWaitForANSpcmDrop();
  /** @brief Bit decision as a function (decideBit() in the helpers does the same inline).  Not called. */
  int twoLevelDemod(float x, int& out);
  /** @brief Run one sample through the V.90 state machine and return the decision symbol. */
  short getV90Decision(float x);
  /** @brief Run one sample through the V.92 state machine and return the decision symbol. */
  short getV92Decision(float x);
  /** @brief Dispatch one sample to getV92Decision() (session_ != 0) or getV90Decision(); the float is forwarded bit for bit. */
  short getDecision(float x);

public:
  V90AutoDigitalImpDetector* adid_;    ///< digital impairment detector, source of the level tables (borrowed)
  unsigned phase_;                     ///< position in the 6-symbol PCM frame, 0 .. 5
  unsigned session_;                   ///< 0 = V.90, non-zero = V.92
  V90Parameters* params_;              ///< parameter block (borrowed)
  unsigned pcm_;                       ///< G.711 law, a PcmType (non-zero: A-law)
  unsigned timeBase_;                  ///< added to the Sd and Jd time-outs
  unsigned char code_;                 ///< reference ucode of the DIL signal
  unsigned char pad19_[1];             ///< padding, never accessed
  unsigned short level_;               ///< linear G.711 level of code_ & 0x7f (read as signed)
  tagV90DILdescriptor* dil_;           ///< DIL descriptor (borrowed)
  V90Jd* jd_;                          ///< V.90 Jd frame detector (borrowed, may be NULL)
  V92Jd* jd92_;                        ///< V.92 Jd frame detector (borrowed, may be NULL)
  unsigned state_;                     ///< current state, a Phase3DemodulatorState
  unsigned cnt_;                       ///< symbols counted in the current state
  unsigned event_;                     ///< event code of the last sample (V90DemodState), 0 = none
  V90Phase3Modulator mod_;             ///< the probing signal generator; provides the reference symbol of the TRN1d / DIL states
  SerialDifferentialDecoder<int> sdd_; ///< differential decoder of the Jd bit path
  Descrambler<int, int> descr_;        ///< descrambler of the Jd bit path (taps 0x12, 0x17, polynomial 0x63)
  V90SdDetector* sd_;                  ///< Sd / QTS tone detector (owned, heap)
  unsigned w3f4_;                      ///< set with trn1Len_ from a parameter word; never read in the library
  unsigned char impairInfo_;           ///< digital impairment info code (set by V90Demodulator::progress(), consumed by setDigitalImairmentsInfo())
  unsigned char flag3f9_;              ///< set by the owner once the DIL gain is applied: enables the error-relaxation count cnt3fc_
  unsigned char pad3fa_[2];            ///< padding, never accessed
  unsigned cnt3fc_;                    ///< symbols counted in error relaxation after flag3f9_ was set
  unsigned short flag400_;             ///< set by the owner when the AGC gain is well below its limit: the second study ends in freeze
  unsigned char pad402_[2];            ///< padding, never accessed
  unsigned jdNotCnt_;                  ///< run of zero bits in the Jd demodulation (the JdNot detector)
  unsigned status_;                    ///< equalizer adaptation enable read by V90Equalizer::process() (non-zero = adapt); set per state
  unsigned dilLen_;                    ///< length of the probing DIL in symbols (calculateDilLength())
  unsigned lineMode_;                  ///< 0 = normal training; non-zero = quick connect (short TRN1, QC study states); set from V90Demodulator::retrainOk_
  unsigned short arg9_;                ///< reset() argument, stored
  unsigned char pad416_[2];            ///< padding, never accessed
  float arg10_;                        ///< reset() argument, stored as a raw word
  unsigned verify_;                    ///< quick-connect line verification result: 1 when ANSpcm demodulation was entered, 0 on a time-out ("not same line")
  unsigned trn1Len_;                   ///< length of TRN1d DD in symbols (a parameter)
  unsigned char sdToneSeen_;           ///< set when the Sd detector hit counter reaches 10, cleared when it drops to 0 (the Sd tone ended)
  unsigned char pad425_[3];            ///< padding, never accessed
  ANSamToneDetector* ans_;             ///< ANSpcm tone detector (owned, heap)
};

typedef V90Phase3Demodulator P34_V90Phase3Demodulator;

#pragma GCC diagnostic ignored "-Winvalid-offsetof"
#include "v90_phase3_demod_layout.inc"

#endif /* V90_PHASE3_DEMOD_H */
