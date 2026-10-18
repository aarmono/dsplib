/**
 * @file v90_phase3_demod_v92_decision.cpp
 * @brief V90Phase3Demodulator::getV92Decision(): the V.92 phase 3 (line probing, quick connect) receive state machine.
 *
 * getV92Decision() runs ONE input sample (a float PCM level) through the state machine and returns the DECISION symbol: the reference level the receive side
 * compares with, or the modulator's own symbol in the known-data states.  getDecision() (v90_phase3_demod.cpp) calls it when `session_` is set.  The class,
 * its members and the state enum Phase3DemodulatorState are declared in v90_phase3_demod.h; the small helpers shared with the V.90 machine (float to 16-bit
 * truncation, phase step, table accessors of the V90AutoDigitalImpDetector, the bit decision, the time-outs) are in v90_phase3_demod_helpers.h.
 *
 * States (the names follow the debug prints):
 * - detection: 0 WaitForSd (Sd detector), 1 SdDemod, 2 SdNot;
 * - TRN1d: 3 known data (the modulator's own TRN1d symbol; the V92Jd is reset at the end), 4 TRN1d DD, 5 study of the reference Ucodes (studyUrefHandler);
 * - Jd: 7 wait for the V92Jd frame (V92Jd::unPackJdData), 8 V92Jd phase frame, 9 JdDemod (the JdNot detector is inlined);
 * - digital impairment learning (DIL): 10 / 13 first study (normal / quick connect), 11 second study, 12 third study, 14 quick-connect second study,
 *   15 quick-connect third study, 16 error relaxation, 17 probing DIL;
 * - 19 .. 25 terminated or time-out states (no output);
 * - V.92 quick connect: 26 / 27 WaitFor QTS / QTSNot, 28 .. 33 the ANSpcm states;
 * - 6 and 18 are unused; they and every state above 33 return 0.
 *
 * Callees from other files: V90SdDetector::process(), ANSamToneDetector::process() / reset(), V92Jd::unPackJd* (jd92_), V90AutoDigitalImpDetector (study and
 * mapping methods), V90Phase3Modulator::generateSymbol() / reset(), linear2alaw() / linear2ulaw(), edprintf() and dsplibs_debug_printf().
 *
 * Conventions:
 * - the symbol counter `cnt_` is incremented BEFORE the dispatch and `event_` is cleared first; the order of the stores to state_ / cnt_ / event_ relative
 *   to the prints (edprintf) is part of the behaviour (tests compare the object at every print and every callee call);
 * - the input x is truncated toward zero to 16 bits (NaN and out-of-range give 0x8000); float comparisons use the plain C conditions (x > 0.0f is false for
 *   NaN);
 * - the callees' results are used at the width the machine tests them: isAltRbs / isThereAnyAltRbsPhase the low 16 bits, V92Jd::unPackJd*Data the low byte;
 * - the decision tables are indexed phase * 128 + ucode with the phase read as 16 bits in most places and as 32 bits for the altRbsPhase_ / suspectedPhase_
 *   indexes (phase_ stays at or below 5);
 * - `cnt_ == constant` tests (36000, 24804, timeBase_ + 12000 / 38760) are exact integer compares;
 * - the float parameter words phase4MeanErrorBefToAftUpdateRatioThresh* are copied as 32-bit words, never loaded as floats (keeps NaN payloads intact).
 *
 * @warning The state handlers use `sd_`, `adid_`, `ans_`, `mod_` and (from state 3 on) `jd92_` without a NULL test; only the time-out in state 3 checks
 *          `jd92_`.  V90Phase3Demodulator::reset() sets them up and the owner passes a V92Jd for V.92 sessions.
 */

#include "v90_phase3_demod.h"
#include "core/dsp_host.h"
#include "v34/v34_api.h"

extern "C"
{
}

typedef P34_V90Phase3Demodulator D;
typedef P18_V90AutoDigitalImpDetector Adid;

/* ------------------------------------------------------------------------------------------------ helpers (static inline) */

#include "v90_phase3_demod_helpers.h"

/**
 * @brief Run one input sample through the V.92 phase 3 state machine.
 * @param x  input sample, linear PCM level as float
 * @return the decision symbol (the 16-bit truncation of x, the modulator's symbol, or a table level, depending on the state); 0 in the terminated states
 * @note Raises the progress events in `event_` and moves `state_`; see the file header for the states.
 * @warning Mutates the parameter block in some states: states 11 and 14 set unk31c = unk320 when flag400_ is set, states 27 and 28 set anspcmDemodulationLength
 *          to 0x320 when bit 0 of the session parameter flags0 is set, and states 5 / 10 / 13 copy a mean-error threshold word over another.
 */
short V90Phase3Demodulator::getV92Decision(float x)
{
  D* t = this;
  Adid* a;
  V90Parameters* p;
  short s = trunc16f(x);

  /* count the sample and clear the event, then run the handler of the current state */
  t->cnt_++;
  t->event_ = 0;
  switch (t->state_)
  {
  case P3D_WAIT_FOR_SD:
  { /* state 0: wait for the Sd tone; time out after timeBase_ + 12000 samples */
    int res = t->sd_->process(x);
    if (res > 0)
    {
      edprintf("V90Phase3Demodulator: Sd detected @ %d\r\n", t->cnt_);
      t->state_ = P3D_SD_DEMOD;
      t->cnt_ = 0;
      t->event_ = V90DEMOD_STATE_SD_DETECTED;
    }
    else if (cntIs(t->cnt_, (unsigned long long)t->timeBase_ + 12000ULL))
    {
      t->state_ = P3D_WAIT_SD_TIMEOUT;
      t->cnt_ = 0;
      t->event_ = V90DEMOD_STATE_PHASE3_ERROR;
      edprintf("V90Phase3Demodulator: WaitForSd TimeOut\r\n");
    }
    return s;
  }
  case P3D_SD_DEMOD:
  { /* state 1: Sd present; wait for its end (SdNot), time out after 0x180 samples */
    int res = t->sd_->process(x);
    if (res < 0)
    {
      edprintf("V90Phase3Demodulator: SdNot detected @ %d\r\n", t->cnt_);
      t->state_ = P3D_SD_NOT;
      t->cnt_ = 0;
      t->event_ = V90DEMOD_STATE_SDNOT_DETECTED;
    }
    else if (t->cnt_ == 0x180)
    {
      t->state_ = P3D_SD_TIMEOUT;
      t->cnt_ = 0;
      t->event_ = V90DEMOD_STATE_PHASE3_ERROR;
      edprintf("V90Phase3Demodulator: SdDemod TimeOut\r\n");
    }
    return s;
  }
  case P3D_SD_NOT:
  { /* state 2: SdNot; 0x30 samples after it start TRN1d (known data, or DD directly for the quick connect) */
    if (t->cnt_ == 0x30)
    {
      if (t->lineMode_ == 0)
      {
        t->state_ = P3D_TRN1D_KNOWN;
        edprintf("V90Phase3Demodulator: enter TRN1d Known Data state\r\n");
      }
      else
      {
        p = t->params_;
        t->state_ = P3D_TRN1D_DD;
        t->trn1Len_ = (unsigned)p->trn1QcDdLength;
        t->w3f4_ = (unsigned)p->unk4a4;
        t->jd92_->unPackJdReset();
        edprintf("V90Phase3Demodulator: enter TRN1d DD state\r\n");
      }
      t->adid_->resetStudyUrefHandler(t->lineMode_);
      t->cnt_ = 0;
      t->event_ = V90DEMOD_STATE_TRN1D_STARTED;
    }
    return s;
  }
  case P3D_TRN1D_KNOWN:
  { /* state 3: TRN1d known data; the modulator's own symbol is the result; move to TRN1d DD at 0x7f8 samples */
    short m = t->mod_.generateSymbol();

    if (t->cnt_ == 0x7f8)
    {
      if (t->jd92_ == 0)
      {
        t->state_ = P3D_NULL_JD;
        t->event_ = V90DEMOD_STATE_PHASE3_ERROR;
        if (dsplibs_debug_level > 1)
          dsplibs_debug_printf("V90Phase3Demodulator: ERROR: Null V92JdDetector\r\n");
        t->cnt_ = 0;
      }
      else
      {
        t->state_ = P3D_TRN1D_DD;
        p = t->params_;
        if (t->lineMode_ == 0)
        {
          t->trn1Len_ = (unsigned)p->trn1dDdLength;
          t->w3f4_ = (unsigned)p->unk344;
        }
        else
        {
          if (dsplibs_debug_level > 1)
            dsplibs_debug_printf("V90Phase3Demodulator: V92 setting params for short TRN1\n");
          p = t->params_;
          t->trn1Len_ = (unsigned)p->trn1QcDdLength;
          t->w3f4_ = (unsigned)p->unk4a4;
        }
        t->adid_->resetStudyUrefHandler(t->lineMode_);
        t->jd92_->unPackJdReset();
        edprintf("V90Phase3Demodulator: enter TRN1d DD state\r\n");
        t->cnt_ = 0;
      }
    }
    return m;
  }
  case P3D_TRN1D_DD:
  { /* state 4: TRN1d DD; bit decisions until trn1Len_ samples */
    int b2;
    short r = decideBit(t, x, &b2);
    if (t->cnt_ == t->trn1Len_)
    {
      edprintf("V90Phase3Demodulator: enter V92 study reference Ucode state @ %d\r\n", t->cnt_);
      t->state_ = P3D_V92_STUDY_UREF;
      t->cnt_ = 0;
    }
    else
    {
      trn1dTimeout(t);
    }
    return r;
  }
  case P3D_V92_STUDY_UREF:
  { /* state 5: study of the reference Ucodes; when done (status 2) wait for a 6-sample frame boundary, then wait for the V92Jd */
    int b2;
    short r = decideBit(t, x, &b2);
    t->status_ = (unsigned)t->adid_->studyUrefHandler(x, t->phase_);
    advancePhase(t);

    if (t->status_ == 2)
    {
      if (t->cnt_ % 6 == 0)
      {
        if ((short)t->adid_->isThereAnyAltRbsPhase() != 0)
        {
          p = t->params_;
          copyWordF(&p->phase4MeanErrorBefToAftUpdateRatioThresh, &p->phase4MeanErrorBefToAftUpdateRatioThreshAltRbs);
        }
        edprintf("V90Phase3Demodulator: enter Wait For V92Jd state @ %d\r\n", t->cnt_);
        t->state_ = P3D_WAIT_FOR_V92JD;
        t->cnt_ = 0;
        t->event_ = V90DEMOD_STATE_WAIT_FOR_JD_ENTERED;
      }
      t->status_ = 1;
    }
    else
    {
      trn1dTimeout(t);
    }
    return r;
  }
  case P3D_WAIT_FOR_V92JD:
  { /* state 7: wait for the V92Jd frame (accepted at a 6-sample boundary unless quick connect); time out after timeBase_ + 38760 samples */
    int b2;
    short r = decideBit(t, x, &b2);
    advancePhase(t);

    /* a complete Jd frame: at a 6-sample boundary (any, for quick connect) leave to the phase frame, else start over */
    if ((unsigned char)t->jd92_->unPackJdData(b2) != 0)
    {
      if (t->cnt_ % 6 != 0 && t->lineMode_ == 0)
      {
        t->jd92_->unPackJdReset();
      }
      else
      {
        if (dsplibs_debug_level > 1)
          dsplibs_debug_printf("V90Phase3Demodulator: waitForJd framePosition = %d\n", t->phase_);
        if (t->phase_ != 0)
        {
          if (dsplibs_debug_level > 1)
            dsplibs_debug_printf("V90Phase3Demodulator: adjustUinfoToPhaseOffset\n");
          t->adid_->adjustUinfoToPhaseOffset((short)t->phase_);
          t->phase_ = 0;
        }
        edprintf("V90Phase3Demodulator: V92Jd detected @ %d\r\n", t->cnt_);
        t->event_ = V90DEMOD_STATE_JD_DETECTED;
        t->state_ = P3D_V92JD_PHASE;
        t->jd92_->unPackJdPhaseReset();
        t->cnt_ = 0;
      }
    }

    if (cntIs(t->cnt_, (unsigned long long)t->timeBase_ + 38760ULL))
    {
      t->state_ = P3D_JD_TIMEOUT;
      t->cnt_ = 0;
      t->event_ = V90DEMOD_STATE_PHASE3_ERROR;
      edprintf("V90Phase3Demodulator: V92JdDemod TimeOut\r\n");
    }
    return r;
  }
  case P3D_V92JD_PHASE:
  { /* state 8: the V92Jd phase frame, accepted at a 6-sample boundary; time out after 24804 samples */
    int b2;
    short r = decideBit(t, x, &b2);
    advancePhase(t);

    if ((unsigned char)t->jd92_->unPackJdPhaseData(b2) != 0)
    {
      if (t->cnt_ % 6 != 0)
      {
        t->jd92_->unPackJdPhaseReset();
      }
      else
      {
        edprintf("V90Phase3Demodulator: V92JdPhase detected @ %d\r\n", t->cnt_);
        t->event_ = V90DEMOD_STATE_V92_JD_PHASE_DETECTED;
        t->state_ = P3D_JD_DEMOD;
        t->cnt_ = 0;
        t->jdNotCnt_ = 0;
      }
    }

    if (cntIs(t->cnt_, 24804ULL))
    {
      t->state_ = P3D_JD_TIMEOUT;
      t->cnt_ = 0;
      t->event_ = V90DEMOD_STATE_PHASE3_ERROR;
      edprintf("V90Phase3Demodulator: JdPhaseDemod TimeOut\r\n");
    }
    return r;
  }
  case P3D_JD_DEMOD:
  { /* state 9: JdDemod, with the JdNot detector inlined: more than 11 zero bits ending at cnt_ % 72 == 12 mean JdNot */
    int b2;
    short r = decideBit(t, x, &b2);
    advancePhase(t);

    /* count consecutive zero bits */
    if (b2 != 0)
      t->jdNotCnt_ = 0;
    else
      t->jdNotCnt_++;

    if (t->jdNotCnt_ > 11 && t->cnt_ % 72 == 12)
    {
      edprintf("V90Phase3Demodulator: JdNot detected @ %d\r\n", t->cnt_);
      t->event_ = V90DEMOD_STATE_JDNOT_DETECTED;
      if (t->lineMode_ == 0)
      {
        t->state_ = (t->params_->probingMode == 0) ? P3D_FIRST_STUDY : P3D_PROBING_DIL;
      }
      else
      {
        t->state_ = P3D_QC_FIRST_STUDY;
        if (dsplibs_debug_level > 1)
          dsplibs_debug_printf("V90Phase3Demodulator: changing state to DILDemodQCfirstStudy\n");
      }
      t->cnt_ = 0;
      t->mod_.reset((PcmType)t->pcm_, t->code_, P3M_DIL, 0, 0, 0, t->dil_, 0);
    }
    else if (cntIs(t->cnt_, (unsigned long long)t->timeBase_ + 38760ULL))
    {
      t->state_ = P3D_JD_TIMEOUT;
      t->cnt_ = 0;
      t->event_ = V90DEMOD_STATE_PHASE3_ERROR;
      edprintf("V90Phase3Demodulator: JdDemod TimeOut\r\n");
    }
    return r;
  }
  case P3D_FIRST_STUDY: /* state 10: DIL first study */
  case P3D_QC_FIRST_STUDY:
  { /* state 13: quick-connect DIL first study (shares the code of state 10) */
    short m = t->mod_.generateSymbol();
    short r = s;
    a = t->adid_;

    /* a reference symbol: look the level up in the (alternate, for an RBS phase) mapping table */
    if (t->mod_.isRef_ != 0)
    {
      unsigned off = BASE_OFF;
      if ((short)a->isAltRbs((short)t->phase_, (short)t->code_, x) != 0)
        off = ALT_OFF;
      unsigned uc = ucodeOf(t->pcm_, absSym(m));
      r = signedLevelS(a, off, s, t->phase_, uc);
    }

    t->status_ = 1;
    a->calculateLinearMeanAndVar(s, m, t->phase_);
    advancePhase(t);

    /* end of the first study: evaluate it and go on to the second one; otherwise check the time-out */
    p = t->params_;
    if (t->cnt_ == (unsigned)p->unk30c)
    {
      a->porcessFirstStudy();
      if ((short)a->isThereAnyAltRbsPhase() != 0)
      {
        p = t->params_;
        copyWordF(&p->phase4MeanErrorBefToAftUpdateRatioThresh, &p->phase4MeanErrorBefToAftUpdateRatioThreshAltRbs);
      }
      t->cnt_ = 0;
      t->state_ = (t->state_ == P3D_FIRST_STUDY) ? P3D_SECOND_STUDY : P3D_QC_SECOND_STUDY;
    }
    else if (t->state_ == P3D_FIRST_STUDY)
    {
      dilTimeout(t, "V90Phase3Demodulator: DILDemodFirstStudy TimeOut\r\n");
    }
    else
    {
      dilTimeout(t, "V90Phase3Demodulator: DILDemodQCfirstStudy TimeOut\r\n");
    }

    termCheck(t);
    return r;
  }
  case P3D_SECOND_STUDY:
  { /* state 11: DIL second study */
    short m = t->mod_.generateSymbol();
    short r;
    a = t->adid_;
    a->calculateLinearMeanAndVar(s, m, t->phase_);
    {
      unsigned ph = t->phase_;
      int am = absSym(m);
      if (altPhaseAt(a, ph) == 0)
      {
        if (am != (int)(short)t->level_)
          a->updateLinMappMeanAndVar((short)t->phase_, (short)ucodeOf(t->pcm_, am));
        r = signedLevelS(a, BASE_OFF, s, t->phase_, ucodeOf(t->pcm_, am));
      }
      else if (am == (int)(short)t->level_)
      {
        unsigned off = BASE_OFF;
        if ((short)a->isAltRbs((short)t->phase_, (short)t->code_, x) != 0)
          off = ALT_OFF;
        r = signedLevelS(a, off, s, t->phase_, ucodeOf(t->pcm_, am));
      }
      else
      {
        a->addReceivedSampleToStorage((short)t->phase_, ucodeOf(t->pcm_, am), x);
        r = s;
      }
    }
    t->status_ = 1;
    if (t->mod_.segCnt_ == 0 && t->mod_.code_ != t->code_)
      a->uniteLinMappInfoOfUnsuspectedPhases(t->mod_.code_);
    p = t->params_;
    if (t->cnt_ == (unsigned)p->unk314)
      t->event_ = V90DEMOD_STATE_DIL_MED_UCODES_STAGE;
    if (t->cnt_ == (unsigned)p->unk300)
    {
      if ((short)t->flag400_ != 0)
      {
        t->event_ = V90DEMOD_STATE_DIL_IN_FREEZE;
        p->unk31c = p->unk320;
      }
      else
      {
        t->event_ = V90DEMOD_STATE_DIL_HIGH_UCODES_STAGE;
      }
    }
    advancePhase(t);
    if (t->cnt_ == (unsigned)p->unk308)
    {
      if (p->unk310 == 0)
      {
        a->porcessSecondStudy();
        t->event_ = V90DEMOD_STATE_DIL_STUDY_ENDED;
        t->state_ = P3D_ERROR_RELAXATION;
        t->cnt_ = 0;
      }
      else
      {
        t->event_ = V90DEMOD_STATE_DIL_IN_FREEZE;
        t->state_ = P3D_THIRD_STUDY;
        t->cnt_ = 0;
      }
    }
    else
    {
      dilTimeout(t, "V90Phase3Demodulator: DILDemodSecondStudy TimeOut\r\n");
    }

    termCheck(t);
    return r;
  }
  case P3D_THIRD_STUDY:
  { /* state 12: DIL third study */
    short m = t->mod_.generateSymbol();
    short r;
    a = t->adid_;
    {
      unsigned ph = t->phase_;
      int am = absSym(m);
      if (altPhaseAt(a, ph) == 0)
      {
        if (t->mod_.isRef_ == 0)
        {
          a->calculateLinearMeanAndVar(s, m, t->phase_);
          a->updateLinMappMeanAndVar((short)t->phase_, (short)ucodeOf(t->pcm_, am));
        }
        r = signedLevelS(a, BASE_OFF, s, t->phase_, ucodeOf(t->pcm_, am));
      }
      else
      {
        r = s;
        if (t->mod_.isRef_ == 0)
          a->addReceivedSampleToStorage((short)t->phase_, ucodeOf(t->pcm_, am), x);
      }
    }
    advancePhase(t);
    t->status_ = 0;
    if (t->mod_.segCnt_ == 0 && t->mod_.code_ != t->code_)
      a->uniteLinMappInfoOfUnsuspectedPhases(t->mod_.code_);
    p = t->params_;
    if (t->cnt_ == (unsigned)p->unk310)
    {
      a->porcessSecondStudy();
      t->cnt_ = 0;
      t->event_ = V90DEMOD_STATE_DIL_STUDY_ENDED;
      t->state_ = P3D_ERROR_RELAXATION;
    }
    else
    {
      dilTimeout(t, "V90Phase3Demodulator: DILDemodThirdStudyStage TimeOut\r\n");
    }

    termCheck(t);
    return r;
  }
  case P3D_QC_SECOND_STUDY:
  { /* state 14: quick-connect DIL second study */
    short m = t->mod_.generateSymbol();
    short r;
    a = t->adid_;
    a->calculateLinearMeanAndVar(s, m, t->phase_);
    {
      unsigned ph = t->phase_;
      int am = absSym(m);
      if (altPhaseAt(a, ph) == 0)
      {
        if (am == (int)(short)t->level_)
        {
          r = signedLevelS(a, BASE_OFF, s, t->phase_, ucodeOf(t->pcm_, am));
        }
        else
        {
          r = s;
          if (suspectedAt(a, ph) == 0)
            r = signedLevel(a, PREV_OFF, s, 0, ucodeOf(t->pcm_, am));
        }
      }
      else if (am == (int)(short)t->level_)
      {
        unsigned off = BASE_OFF;
        if ((short)a->isAltRbs((short)t->phase_, (short)t->code_, x) != 0)
          off = ALT_OFF;
        r = signedLevelS(a, off, s, t->phase_, ucodeOf(t->pcm_, am));
      }
      else
      {
        r = s;
        a->addReceivedSampleToStorage((short)t->phase_, ucodeOf(t->pcm_, am), x);
      }
    }
    p = t->params_;
    t->status_ = 1;
    if (t->cnt_ == (unsigned)p->unk314)
      t->event_ = V90DEMOD_STATE_DIL_MED_UCODES_STAGE;
    if (t->cnt_ == (unsigned)p->unk300)
    {
      if ((short)t->flag400_ != 0)
      {
        t->event_ = V90DEMOD_STATE_DIL_IN_FREEZE;
        p->unk31c = p->unk320;
      }
      else
      {
        t->event_ = V90DEMOD_STATE_DIL_HIGH_UCODES_STAGE;
      }
    }
    advancePhase(t);
    if (t->cnt_ == (unsigned)p->unk308)
    {
      t->cnt_ = 0;
      t->event_ = V90DEMOD_STATE_DIL_IN_FREEZE;
      t->state_ = P3D_QC_THIRD_STUDY;
    }
    else
    {
      dilTimeout(t, "V90Phase3Demodulator: DILDemodQCsecondStudy TimeOut\r\n");
    }

    termCheck(t);
    return r;
  }
  case P3D_QC_THIRD_STUDY:
  { /* state 15: quick-connect DIL third study */
    short m = t->mod_.generateSymbol();
    short r;
    a = t->adid_;
    if (t->mod_.isRef_ == 0)
      a->calculateLinearMeanAndVar(s, m, t->phase_);
    {
      unsigned ph = t->phase_;
      int am = absSym(m);
      if (altPhaseAt(a, ph) == 0)
      {
        r = s;
        if (suspectedAt(a, ph) == 0)
          r = signedLevel(a, PREV_OFF, s, 0, ucodeOf(t->pcm_, am));
      }
      else
      {
        r = s;
        if (t->mod_.isRef_ == 0)
          a->addReceivedSampleToStorage((short)t->phase_, ucodeOf(t->pcm_, am), x);
      }
    }
    advancePhase(t);
    t->status_ = 0;
    p = t->params_;
    if (t->cnt_ == (unsigned)p->unk310)
    {
      a->setQcLinearMapping();
      t->cnt_ = 0;
      t->event_ = V90DEMOD_STATE_DIL_STUDY_ENDED;
      t->state_ = P3D_ERROR_RELAXATION;
    }
    else
    {
      dilTimeout(t, "V90Phase3Demodulator: DILDemodQCthirdStudy TimeOut\r\n");
    }

    termCheck(t);
    return r;
  }
  case P3D_ERROR_RELAXATION:
  { /* state 16: DIL error relaxation */
    short m = t->mod_.generateSymbol();
    short r = s;
    a = t->adid_;
    {
      unsigned ph = t->phase_;
      int am = absSym(m);
      if (altPhaseAt(a, ph) == 0)
      {
        r = signedLevelS(a, BASE_OFF, s, t->phase_, ucodeOf(t->pcm_, am));
      }
      else if (t->mod_.isRef_ != 0)
      {
        unsigned off = BASE_OFF;
        if ((short)a->isAltRbs((short)ph, (short)t->code_, x) != 0)
          off = ALT_OFF;
        r = signedLevelS(a, off, s, t->phase_, ucodeOf(t->pcm_, am));
      }
    }
    advancePhase(t);
    p = t->params_;
    if (t->flag3f9_ != 0)
    {
      t->cnt3fc_ = t->cnt3fc_ + 1;
      if (t->cnt3fc_ == (unsigned)p->unk31c)
      {
        t->event_ = V90DEMOD_STATE_DIL_INITIAL_ERROR_RELAX_END;
        t->status_ = 1;
      }
    }
    if (t->cnt_ == (unsigned)p->unk318)
    {
      t->event_ = V90DEMOD_STATE_DIL_RECEIVED;
    }
    else
    {
      dilTimeout(t, "V90Phase3Demodulator: DILDemodErrorRelaxation TimeOut\r\n");
    }

    termCheck(t);
    return r;
  }
  case P3D_PROBING_DIL:
  { /* state 17: probing DIL; time out after 40000 samples */
    short m = t->mod_.generateSymbol();
    p = t->params_;
    t->status_ = (absSym(m) == 0xffc && t->mod_.segCnt_ > (unsigned)p->dfeLength) ? 1u : 0u;
    if (t->cnt_ == t->dilLen_)
    {
      edprintf("V90Phase3Demodulator: Probing DIL ended\r\n");
      t->event_ = V90DEMOD_STATE_PROBING_DIL_ENDED;
    }
    else if (t->cnt_ == 40000)
    {
      t->state_ = P3D_DIL_TIMEOUT;
      t->cnt_ = 0;
      t->event_ = V90DEMOD_STATE_PHASE3_ERROR;
      edprintf("V90Phase3Demodulator: ProbingDILDemod TimeOut\r\n");
    }
    return s;
  }
  case P3D_TERMINATED:
  case P3D_WAIT_SD_TIMEOUT:
  case P3D_SD_TIMEOUT:
  case P3D_TRN1D_TIMEOUT:
  case P3D_JD_TIMEOUT:
  case P3D_DIL_TIMEOUT:
  case P3D_NULL_JD:
    return 0;
  case P3D_WAIT_QTS:
  { /* state 26: wait for QTS (quick connect); on time-out assume it is not the same line */
    t->ans_->process(x);
    int res = t->sd_->process(x);
    if (res > 0)
    {
      edprintf("V90Phase3Demodulator: QTS detected @ %d\r\n", t->cnt_);
      t->state_ = P3D_WAIT_QTS_NOT;
      t->cnt_ = 0;
      t->event_ = V90DEMOD_STATE_QTS_DETECTED;
    }
    else
    {
      p = t->params_;
      if (t->cnt_ == (unsigned)p->anspcmDemodulationLength)
      {
        t->state_ = P3D_ANSPCM_DROP;
        t->cnt_ = 0;
        t->verify_ = 0;
        t->event_ = V90DEMOD_STATE_ANSPCM_DEMOD_OVER;
        edprintf("V90Phase3Demodulator: WaitFor Qts TimeOut, decision is set to not same line\r\n");
      }
    }
    return s;
  }
  case P3D_WAIT_QTS_NOT:
  { /* state 27: wait for the end of QTS */
    t->ans_->process(x);
    int res = t->sd_->process(x);
    if (res < 0)
    {
      edprintf("V90Phase3Demodulator: QTSNot detected @ %d\r\n", t->cnt_);
      t->state_ = P3D_QTS_NOT_DONE;
      t->cnt_ = 0;
      t->event_ = V90DEMOD_STATE_QTSNOT_DETECTED;
    }
    else if (t->cnt_ == 0x300)
    {
      t->state_ = P3D_ANSPCM_RECOVERY;
      t->cnt_ = 0;
      t->verify_ = 0;
      edprintf("V90Phase3Demodulator: WaitFor QtsNot TimeOut, decision is set to not same line\r\n");
      p = t->params_;
      if ((p->params->flags0 & 1) != 0)
        p->anspcmDemodulationLength = 0x320;
    }
    return s;
  }
  case P3D_QTS_NOT_DONE: /* state 28: 0x30 samples after QtsNot start the ANSpcm demodulation */
  {
    if (t->cnt_ == 0x30)
    {
      t->state_ = P3D_ANSPCM_DEMOD;
      p = t->params_;
      t->cnt_ = 0;
      if ((p->params->flags0 & 1) != 0)
        p->anspcmDemodulationLength = 0x320;
      t->event_ = V90DEMOD_STATE_ANSPCM_DEMOD_ENTERED;
      t->verify_ = 1;
      t->ans_->reset();
      edprintf("V90Phase3Demodulator: enter ANSpcm demod state\r\n");
    }
    return s;
  }
  case P3D_ANSPCM_DEMOD: /* state 29: ANSpcm demodulation until anspcmDemodulationLength samples */
  {
    t->ans_->process(x);
    p = t->params_;
    if (t->cnt_ == (unsigned)p->anspcmDemodulationLength)
      t->event_ = V90DEMOD_STATE_ANSPCM_DEMOD_OVER;
    return s;
  }
  case P3D_ANSPCM_WAIT: /* state 30: wait for the ANSpcm energy to drop */
  {
    if (t->ans_->process(x) == 0)
    {
      edprintf("V90Phase3Demodulator: ANSpcm energy drop detected...\r\n");
      t->event_ = V90DEMOD_STATE_ANSPCM_DROP_DETECTED;
      t->state_ = P3D_STATE31;
    }
    return s;
  }
  case P3D_STATE31: /* state 31: ANSpcm energy drop reported, nothing more to do */
    return s;
  case P3D_ANSPCM_DROP: /* state 32: wait for the ANSpcm energy drop after a QTS time-out; fake it after 0x7d0 samples */
  {
    if (t->ans_->process(x) != 0)
    {
      edprintf("V90Phase3Demodulator: ANSpcm detected on recovery, move to wait for drop...\r\n");
      t->event_ = V90DEMOD_STATE_ANSPCM_DEMOD_OVER;
    }
    if (t->cnt_ > 0x7cf)
    {
      edprintf("V90Phase3Demodulator: faking ANSpcm energy drop detected (QTs timeout)...\r\n");
      t->event_ = V90DEMOD_STATE_ANSPCM_DROP_DETECTED;
      t->state_ = P3D_STATE31;
    }
    return s;
  }
  case P3D_ANSPCM_RECOVERY: /* state 33: ANSpcm seen again after a QtsNot time-out */
  {
    if (t->ans_->process(x) != 0)
    {
      p = t->params_;
      if (t->cnt_ >= (unsigned)p->anspcmDemodulationLength)
      {
        edprintf("V90Phase3Demodulator: ANSpcm detected on recovery, move to wait for drop...\r\n");
        t->event_ = V90DEMOD_STATE_ANSPCM_DEMOD_OVER;
      }
    }
    return s;
  }
  default: /* states 6, 18 and above 33 */
    break;
  }
  return 0;
}
