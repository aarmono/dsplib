/**
 * @file v90_adid.h
 * @brief V90AutoDigitalImpDetector: the automatic digital-impairment detector of the V.90 / V.92 PCM demodulator.
 *
 * During TRN1 the detector studies the received samples of the six PCM phases (a V.90 frame carries six symbol phases; every phase can use up to 128
 * ucodes).  From them it builds, per phase, a LINEAR MAPPING: the linear PCM value that the far-end codec really produced for every ucode
 * (`linearMapping_`, and `linearMappingAlt_` for the alternate values of robbed-bit-signalling phases).  It decides which phases are "alternate RBS"
 * phases (`altRbsPhase_`: the codec level of that phase differs from the others), which are only suspected of it (`suspectedPhase_`), identifies the
 * pad gain and the codec law (findPadGain()) and the highest usable ucode (determineMaxUcode()).
 *
 * The object is 0xa9b0 bytes in the 32-bit layout.  V90Demodulator allocates it on the heap and shares it by pointer with V90Demapper,
 * V90Phase3Demodulator and V90ConstellationDesigner.  The constructor stores only `params_`; reset() initialises everything else, so no other method may
 * run before reset().  Two-dimensional tables are flattened [phase 0..5][ucode 0..127] with index = (phase << 7) + ucode (some members use a
 * different row stride, see their comments).
 *
 * Definitions: v90_adid.cpp (accumulation, mean / variance, phase uniting, small accessors), v90_adid_pad_gain.cpp (getAltVarThresh(),
 * updateAltRbsPhaseInDil(), setQcLinearMapping(), determineMaxUcode(), findPadGain()), v90_adid_study.cpp (the TRN1 study state machine:
 * resetStudyUrefHandler(), studyUrefHandler(), porcessFirstStudy(), porcessSecondStudy()).  The 32-bit layout is checked by v90_adid_layout.inc
 * (generated).  The detector reads V90Parameters (adidStudyLen0..5, adidStudyLenAlt0..5, lineConnectionType).
 */
#ifndef DSP_CLASSES_P18_H
#define DSP_CLASSES_P18_H

#include "core/dsp_cxx.h"
#include "pcm_leaf_classes.h" /* PcmType */
#include "pcm_parameters.h"   /* V90Parameters */

/* The codec law of the far end (the second argument of reset()) is the enum PcmType { PCM_MU_LAW = 0, PCM_A_LAW = 1 } of pcm_leaf_classes.h: 0 means mu-law,
 * anything else A-law.  The enum is defined once, there, because several headers are included together in one translation unit. */

/**
 * Study of the received PCM samples of the TRN1 sequence: per-phase linear mapping, alternate-RBS phase detection, pad gain, codec law and maximum ucode.
 *
 * Methods marked "not called" have no caller in the library; they are complete and kept for the day the study logic uses them.
 */
class V90AutoDigitalImpDetector
{
public:
  /** Stores the parameter block only; call reset() before any other method. */
  explicit V90AutoDigitalImpDetector(V90Parameters* p);
  ~V90AutoDigitalImpDetector();

  /* ---- accumulation, mean / variance, uniting of phases (v90_adid.cpp) ---- */

  /** Clears the sum, sum of squares and count of the bin (phase, ucode). */
  void clearCamulativeVal(short phase, short ucode);
  /** Not called: clears the alternate sum and count of `phase`; the second argument is ignored. */
  void clearCamulativeAltVal(short phase, short unused);
  /** Selects the thresholds of the connection type (2 = the second line type, anything else the default); see lineType2Flag_. */
  void setConnectionType(short type);
  /** Starts a study session: reference ucode, codec law, empty statistics, default thresholds. */
  void reset(unsigned char ucode, PcmType pcmType, short arg3);
  /** Zeroes both linear mappings and sets the reference ucode of every phase to the reference linear value. */
  void resetLinearMapping();
  /** @return 1 when at least one phase is flagged as alternate RBS, else 0 */
  int isThereAnyAltRbsPhase();
  /** @return 1 when `phase` is an alternate-RBS phase and `value` is farther than altRbsThresh_ from the phase's linear value of `ucode`, else 0 */
  int isAltRbs(short phase, short ucode, float value);
  /** Not called: adds |value| to the alternate sum / count of `phase`. */
  void calculateLinearMeanAndVarAlt(short value, unsigned phase);
  /** Merges the statistics of the phases whose linear value of `ucode` agrees (within sqrDiffThreshUref_) and writes the shared mean / variance back. */
  void unitePhasesInfoOfUref(short ucode);
  /** Turns the accumulated sums of the bin (phase, ucode) into variance and linear value. */
  void updateLinMappMeanAndVar(short phase, short ucode);
  /** Not called: mean / variance of the reference ucode in all six phases, then unitePhasesInfoOfUref() and clearing of the sums. */
  void updateUref();
  /** Not called: alternate linear value of the bin (phase, ucode) from the alternate sum / count. */
  void updateLinMappMeanAndVarAlt(short phase, short ucode);
  /** Not called: alternate linear value of the reference ucode of every alternate phase; clears the alternate sums. */
  void updateUrefAlt();
  /** Unites the phases that are not suspected of RBS for `ucode` into the largest group of agreeing phases and clears their sums. */
  void uniteLinMappInfoOfUnsuspectedPhases(unsigned char ucode);
  /** Not called: the linear value of `phase` (ucodes 5..116) nearest to |value|. */
  int unSuspectedPhaseNearestLinMapp(short value, short phase);
  /** Stores one received sample of (phase, ucode) for the later alternate-phase analysis. */
  void addReceivedSampleToStorage(short phase, unsigned char ucode, float value);
  /** Adds one sample to the sum, sum of squares and count of the bin of its linear value's ucode. */
  void calculateLinearMeanAndVar(short sample, short linearValue, unsigned phase);
  /** Divides every entry of both linear mappings by padGain_ (rounded). */
  void applyPadGainToLinMapp();
  /** Not called: copies six per-phase maximum ucodes. */
  void setMaxUcodeArray(unsigned char* a);
  /** Not called: copies the 128-entry linear mapping of a previous session. */
  void setPrevSessionLinearMapping(short* m);
  /** Rotates the six linear values of the reference ucode so that they start at phase `offset`. */
  void adjustUinfoToPhaseOffset(short offset);

  /* ---- pad gain, maximum ucode, alternate phases (v90_adid_pad_gain.cpp) ---- */

  /** @return the variance threshold for alternate-phase detection: `cap` times the mean of the variances below the mean of all six, at least
   *         altVarThreshMax_ */
  float getAltVarThresh(float* var, float cap);
  /** Rebuilds the mappings of the alternate-RBS phases from the stored samples. */
  void updateAltRbsPhaseInDil();
  /** Chooses the codec law and pad gain that best explain the linear mapping. */
  void findPadGain();
  /** Determines the highest usable ucode (overall and per phase) and the allowed-ucode flags from the variance table, up to ucode `arg`. */
  void determineMaxUcode(short arg);
  /** Quick-connect: takes the mappings from the previous session (unsuspected phases) or from the accumulated sums, then updateAltRbsPhaseInDil(). */
  void setQcLinearMapping();

  /* ---- TRN1 study state machine (v90_adid_study.cpp) ---- */

  /** Loads the study lengths and thresholds for the line mode (0 = defaults from the parameters, else derived from studyParam_) and restarts the study. */
  void resetStudyUrefHandler(unsigned lineMode);
  /** End of the first study: marks the phases suspected of RBS, clears the statistics below and at the reference ucode. */
  void porcessFirstStudy();
  /** End of the second study: snaps the suspected phases onto the unsuspected phase, then updateAltRbsPhaseInDil(). */
  void porcessSecondStudy();
  /** Feeds one received sample of `phase` to the study. @return 1 carry on, 0 the sample went into the alternate statistics, 2 the study is finished */
  int studyUrefHandler(float sample, unsigned phase);

public:
  int16_t linearMapping_[768];            ///< +0x0000 [phase][ucode] linear value of each ucode
  int16_t linearMappingAlt_[768];         ///< +0x0600 [phase][ucode] alternate linear value (alternate-RBS phases)
  int16_t prevSessionLinearMapping_[128]; ///< +0x0c00 [ucode] mapping of a previous session (quick connect)
  uint8_t ucodeAllowed_[768];             ///< +0x0d00 [phase][ucode] 1 when the ucode may be used (set by determineMaxUcode())
  float cumSum_[768];                     ///< +0x1000 [phase][ucode] sum of |sample|
  uint32_t cumCount_[768];                ///< +0x1c00 [phase][ucode] number of accumulated samples
  int16_t altRbsPhase_[6];                ///< +0x2800 1 when the phase is an alternate-RBS phase
  uint8_t suspectedPhase_[6];             ///< +0x280c 1 when the phase is suspected of RBS
  uint8_t unk2812_[2];                    ///< +0x2812 padding, never accessed
  V90Parameters* params_;                 ///< +0x2814 parameter block (borrowed)
  int16_t storage_[12660];                ///< +0x2818 [phase][2110] stored |samples| of the alternate phases (row stride 2110)
  uint16_t storedCount_[768];             ///< +0x8b00 [phase][ucode] number of stored samples per bin
  uint32_t storageFill_[6];               ///< +0x9100 number of samples stored per phase
  float cumSumSq_[768];                   ///< +0x9118 [phase][ucode] sum of sample^2
  float altSum_[6];                       ///< +0x9d18 per phase: sum of |alternate samples|
  uint32_t altCount_[6];                  ///< +0x9d30 per phase: number of alternate samples
  float variance_[768];                   ///< +0x9d48 [phase][ucode] variance of the bin
  int16_t studyDone_;                     ///< +0xa948 1 once the second study update has run
  uint8_t unka94a_[2];                    ///< +0xa94a padding, never accessed
  float padGain_;                         ///< +0xa94c identified pad gain (1.0 = none)
  float studyParam_;                      ///< +0xa950 scale of the thresholds for resetStudyUrefHandler(!= 0); never assigned in the library
  uint8_t maxUcode_;                      ///< +0xa954 highest usable ucode
  uint8_t unka955_[1];                    ///< +0xa955 padding, never accessed
  uint8_t maxUcodeArray_[6];              ///< +0xa956 per-phase highest usable ucode
  int32_t pcmType_;                       ///< +0xa95c codec law given to reset() (0 = mu-law)
  int32_t codecIsAlaw_;                   ///< +0xa960 result of findPadGain(): 1 = A-law
  float trn1Sigma_;                       ///< +0xa964 mean variance of the non-alternate phases at the reference ucode
  int16_t unsuspectedPhase_;              ///< +0xa968 first phase not suspected of RBS
  uint8_t nofSuspectedPhases_;            ///< +0xa96a number of suspected phases
  uint8_t refUcode_;                      ///< +0xa96b reference ucode of the study
  int16_t refLinear_;                     ///< +0xa96c linear value of the reference ucode
  int16_t resetArg3_;                     ///< +0xa96e third argument of reset() (selects altVarFactor_)
  float altVarFactor_;                    ///< +0xa970 factor of the alternate-phase variance threshold
  float altVarFactor974_;                 ///< +0xa974 second factor, set by reset(), not read
  int16_t lineType2Flag_;                 ///< +0xa978 1 for connection type 2, not read
  int16_t minMaxUcode_;                   ///< +0xa97a lower limit of maxUcode_
  float minUcodeScale_;                   ///< +0xa97c scale of the lowest ucode considered by findPadGain()
  float varThreshFactor_;                 ///< +0xa980 factor of the variance threshold of determineMaxUcode()
  uint32_t studyState_;                   ///< +0xa984 state of studyUrefHandler() (0..6)
  uint32_t studyCounter_;                 ///< +0xa988 symbols seen in the current state
  uint32_t studyLen0_;                    ///< +0xa98c length of state 0
  uint32_t studyLen1_;                    ///< +0xa990 length of state 1
  uint32_t studyLen2_;                    ///< +0xa994 length of state 2
  uint32_t studyLen3_;                    ///< +0xa998 length of states 3 and 5
  uint32_t studyLen4_;                    ///< +0xa99c length of the waiting state 4
  uint32_t studyLen5_;                    ///< +0xa9a0 loaded, not used by the state machine
  int16_t sqrDiffThreshUref_;             ///< +0xa9a4 distance below which phases are united at the reference ucode
  int16_t altRbsThresh_;                  ///< +0xa9a6 distance above which a sample counts as alternate
  float altVarThreshMax_;                 ///< +0xa9a8 floor of the alternate-variance threshold (despite the name, getAltVarThresh() never returns less)
  int16_t sqrDiffThreshMin_;              ///< +0xa9ac lower limit of the squared-difference threshold of porcessFirstStudy()
  int16_t sqrDiffThreshMax_;              ///< +0xa9ae upper limit of the squared-difference threshold of porcessFirstStudy()
};

typedef V90AutoDigitalImpDetector P18_V90AutoDigitalImpDetector;

#include "v90_adid_layout.inc"

#endif /* DSP_CLASSES_P18_H */
