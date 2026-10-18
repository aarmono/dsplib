/**
 * @file v90_demod_state.h
 * @brief V90DemodState: the event / outcome codes of the V.90 / V.92 receive chain.
 *
 * One code space, three producers and two consumers:
 * - V90Phase3Demodulator::event_ (1 .. 0x15 and 0x37 .. 0x3b: the Sd / TRN1d / Jd / DIL / ANSpcm decisions), V90Phase4Demodulator::event_ (0x16 .. 0x35)
 *   and V90Equalizer::event_ (the same values, copied from the phase object by V90Equalizer::process(); it adds RRN 0x23 and FPE 0x25) leave one code
 *   per symbol (0 = nothing);
 * - V90Demodulator::progress() copies the code to V90Demodulator::state_ (and stores 0x1e, 0x1f, 0x21, 0x22, 0x26 and 0x2b itself, enterDataPhase() 0x1e
 *   and enterPhase3() 0x20), then reacts per phase;
 * - VPcmFloModem::v90RunDemodulator() / runPcmModem() / qcLineVerification() (vpcm_flo_modem_run.cpp, vpcm_flo_modem.cpp) read V90Demodulator::state_
 *   and switch on it.
 *
 * Naming: where a debug string exists for a code, the name follows its wording ("Sd detected", "SdNot detected", "Jd detected", "Ri detected", "MP detected",
 * "Ed detected", "Rt detected", "CP detected", "QTS detected", the "DemodDil..." prints of V90Equalizer::process(), "Phase3 Terminated", "Probing DIL
 * ended", "enter study reference Ucode state", "enter Wait For Jd state", "entering WaitForRt state", "ANSpcm energy drop detected", and the
 * "VPcmFloModem (V90): ... requested / detected !!" prints of the consumer).  The V.92 CP / SUV family (0x2d .. 0x35) is named after the V92Phase4Modulator
 * method the consumer calls for it (recivedCP, recivedCPtag, recivedSUV, recivedSUVtag, recivedPartOne / PartTwoSilenceRrnSUV(tag), recivedFirstRrnEd).
 * Less certain names (the code is stored or switched on, but no string or method names it; the neutral number stays in the name where nothing better
 * is known): V90DEMOD_STATE_24, _29 and _36 (never stored by the library; 0x24 is switched on, together with 0x25, by the V.92 consumer),
 * V90DEMOD_STATE_FPE_DETECTED (0x25: stored by the equalizer when detectFPE fires; the consumer calls initiateFPE), V90DEMOD_STATE_TRN1D_STARTED (3: the
 * consumer's "start of TRN1d" and the phase's "enter TRN1d Known Data / DD state"), V90DEMOD_STATE_STUDY_UREF_ENTERED / _WAIT_FOR_JD_ENTERED (4 / 5: named
 * after the phase's prints; the equalizer and progress handlers are the only users of the events), V90DEMOD_STATE_SD_TONE_DROPPED (9: the equalizer reacts
 * with resetSdHalfBaudDft), V90DEMOD_STATE_DIL_RECEIVED (0x12: the consumer calls V34XF_IndicateDilReceived; produced at the end of the error-relaxation
 * stage) and V90DEMOD_STATE_WAIT_FOR_RT_ENTERED (0x2a: the producer's print; the consumer rebuilds the CP after a silence RRN).
 *
 * The guards (`state_ > 0x15`, `code > 0x2b`, `code <= 0x35`, `state_ - 0x19 > 0x1c`, `st - 0x1f > 2`, `phaseEvent - 3 > 0x11`, ...) and the case labels of the
 * switches on `state_ - 0x19` stay literal.  Only the values of V90Demodulator::state_ and of the three event_ members are named; the event_ words of
 * V90Modulator / V92Modulator / V90Phase3Modulator / V92Phase4Modulator are other code spaces.
 */
#ifndef V90_DEMOD_STATE_H
#define V90_DEMOD_STATE_H

/** Event / outcome code of the receive chain (see the file header); values are stored in 32-bit event_ / state_ members. */
enum V90DemodState
{
  V90DEMOD_STATE_NONE = 0,
  /* phase 3: produced by V90Phase3Demodulator */
  V90DEMOD_STATE_SD_DETECTED = 0x01,
  V90DEMOD_STATE_SDNOT_DETECTED = 0x02,
  V90DEMOD_STATE_TRN1D_STARTED = 0x03,
  V90DEMOD_STATE_STUDY_UREF_ENTERED = 0x04,
  V90DEMOD_STATE_WAIT_FOR_JD_ENTERED = 0x05,
  V90DEMOD_STATE_JD_DETECTED = 0x06,
  V90DEMOD_STATE_V92_JD_PHASE_DETECTED = 0x07,
  V90DEMOD_STATE_JDNOT_DETECTED = 0x08,
  V90DEMOD_STATE_SD_TONE_DROPPED = 0x09,
  V90DEMOD_STATE_DIL_HIGH_UCODES_STAGE = 0x0a,
  V90DEMOD_STATE_DIL_HIGH_UCODES_STAGE_END = 0x0b,
  V90DEMOD_STATE_DIL_MED_UCODES_STAGE = 0x0c,
  V90DEMOD_STATE_DIL_MED_UCODES_STAGE_END = 0x0d,
  V90DEMOD_STATE_DIL_INITIAL_ERROR_RELAXATION = 0x0e,
  V90DEMOD_STATE_DIL_INITIAL_ERROR_RELAX_END = 0x0f,
  V90DEMOD_STATE_DIL_IN_FREEZE = 0x10,
  V90DEMOD_STATE_DIL_STUDY_ENDED = 0x11,
  V90DEMOD_STATE_DIL_RECEIVED = 0x12,
  V90DEMOD_STATE_PROBING_DIL_ENDED = 0x13,
  V90DEMOD_STATE_PHASE3_TERMINATED = 0x14,
  V90DEMOD_STATE_PHASE3_ERROR = 0x15,
  /* phase 4: produced by V90Phase4Demodulator */
  V90DEMOD_STATE_RI_DETECTED = 0x16,
  V90DEMOD_STATE_RINOT_DETECTED = 0x17,
  V90DEMOD_STATE_MIDDLE_OF_TRN2D = 0x18,
  V90DEMOD_STATE_TRN2D_RECEIVED = 0x19,
  V90DEMOD_STATE_MP_DETECTED = 0x1a,
  V90DEMOD_STATE_MPNOT_DETECTED = 0x1b,
  V90DEMOD_STATE_ED_DETECTED = 0x1c,
  V90DEMOD_STATE_PHASE4_TERMINATED = 0x1d,
  /* stored by V90Demodulator itself (0x1e, 0x1f, 0x20 .. 0x22, 0x26), by the equalizer (0x23, 0x25), or never (0x24) */
  V90DEMOD_STATE_DATA_PHASE_ENTERED = 0x1e,
  V90DEMOD_STATE_DROP_TO_V34 = 0x1f,
  V90DEMOD_STATE_V90_FALLBACK = 0x20,
  V90DEMOD_STATE_RETRAIN_REQUESTED = 0x21,
  V90DEMOD_STATE_RATE_RENEG_REQUESTED = 0x22,
  V90DEMOD_STATE_RATE_RENEG_DETECTED = 0x23,
  V90DEMOD_STATE_24 = 0x24,
  V90DEMOD_STATE_FPE_DETECTED = 0x25,
  V90DEMOD_STATE_ENERGY_DROP = 0x26,
  /* phase 4 again (0x2b is stored by V90Demodulator itself) */
  V90DEMOD_STATE_RT_DETECTED = 0x27,
  V90DEMOD_STATE_RTNOT_DETECTED = 0x28,
  V90DEMOD_STATE_29 = 0x29,
  V90DEMOD_STATE_WAIT_FOR_RT_ENTERED = 0x2a,
  V90DEMOD_STATE_TERMINATE_SESSION = 0x2b,
  V90DEMOD_STATE_RDNOT_DETECTED = 0x2c,
  V90DEMOD_STATE_CP_DETECTED = 0x2d,
  V90DEMOD_STATE_CPNOT_DETECTED = 0x2e,
  V90DEMOD_STATE_SUV_DETECTED = 0x2f,
  V90DEMOD_STATE_SUVT_DETECTED = 0x30,
  V90DEMOD_STATE_SILENCE_RRN_SUV_PART1 = 0x31,
  V90DEMOD_STATE_SILENCE_RRN_SUV_PART2 = 0x32,
  V90DEMOD_STATE_SILENCE_RRN_SUVT_PART1 = 0x33,
  V90DEMOD_STATE_SILENCE_RRN_SUVT_PART2 = 0x34,
  V90DEMOD_STATE_FIRST_RRN_ED_DETECTED = 0x35,
  V90DEMOD_STATE_36 = 0x36,
  /* quick-connect line verification: V90Phase3Demodulator, ANSpcm */
  V90DEMOD_STATE_QTS_DETECTED = 0x37,
  V90DEMOD_STATE_QTSNOT_DETECTED = 0x38,
  V90DEMOD_STATE_ANSPCM_DEMOD_ENTERED = 0x39,
  V90DEMOD_STATE_ANSPCM_DEMOD_OVER = 0x3a,
  V90DEMOD_STATE_ANSPCM_DROP_DETECTED = 0x3b
};

#endif /* V90_DEMOD_STATE_H */
