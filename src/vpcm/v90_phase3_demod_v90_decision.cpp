/**
 * @file v90_phase3_demod_v90_decision.cpp
 * @brief V90Phase3Demodulator::getV90Decision(): the V.90 phase-3 state machine, one PCM sample per call.
 *
 * getDecision() calls this when session_ is 0.  The function counts the sample (cnt_), clears event_, then runs the state in state_ and returns the DECISION
 * symbol, the 16-bit reference level that the equalizer compares with.  It is the V.92 machine (v90_phase3_demod_v92_decision.cpp) with a V90Jd frame detector
 * in place of the V92Jd frames.  The states (Phase3DemodulatorState), in the order of a normal start-up:
 * - 0 WaitForSd: the Sd detector looks for the Sd tone (a hit counter that reached 10 and falls to 0 raises SD_TONE_DROPPED); 1 SdDemod: wait for SdNot;
 *   2 SdNot: after 0x30 symbols start TRN1d (known data on a normal start, TRN1d DD in quick connect);
 * - 3 TRN1d known data: the decision is the modulator's own symbol; after 0x7f8 symbols switch to TRN1d DD; 4 TRN1d DD: decisions from the learned level
 *   tables (decideBit()); after trn1Len_ symbols go to 5; 5 study of the reference ucode: the detector's studyUrefHandler() classifies the phases; 6 wait for
 *   the V90Jd frame (V90Jd::unPackData() on the descrambled bits), then 9 JdDemod: runs until the JdNot pattern (more than 11 zeros, cnt_ % 72 == 12);
 * - 10 / 13 DIL first study (13 in quick connect), 11 / 14 second study, 12 / 15 third study, 16 error relaxation: the DIL signal comes from the
 *   modulator (generateSymbol()), the detector learns the level tables (mean and variance per ucode and phase, robbed-bit-signalling phases kept apart),
 *   and the decision is the learned level with the sign of the input;  17 probing DIL (instead of the studies when probingMode is set);
 * - 19 .. 25 terminated or error states, no output (the symbol result is 0);
 * - 26 .. 33 quick connect (V.92 line verification): WaitFor QTS and QTSNot with the Sd detector, ANSpcm demodulation with the ANSam detector, then waiting
 *   for the ANSpcm energy drop.
 * States 7, 8, 18 and every value above 33 do nothing and return 0.
 *
 * The thresholds of the DIL stages (symbol counts of the study stages, the relaxation and freeze points) are words of the parameter block that have no
 * parameter name (the unk3xx members of V90Parameters).  Events are the V90DemodState codes; the owner reads them from event_.
 *
 * Numeric behaviour: the input x is truncated to 16 bits by trunc16f() (NaN and out-of-range values give 0x8000); `x > 0.0f` is false for NaN; the symbol-count
 * comparisons against 12000 + timeBase_ and 38760 + timeBase_ are exact 64-bit comparisons; the update-ratio threshold words of the parameter block are
 * copied as raw 32-bit words.  The order of the state_ / cnt_ / event_ stores relative to the prints differs from case to case and is part of the
 * observable behaviour.
 *
 * @warning The state machine reads and writes the shared parameter block: it copies phase4MeanErrorBefToAftUpdateRatioThreshAltRbs over
 *          phase4MeanErrorBefToAftUpdateRatioThresh when robbed-bit phases are found, writes unk31c, and in quick-connect states writes
 *          anspcmDemodulationLength.  jd_ is dereferenced without a NULL check in the Jd states and in P3D_SD_NOT in quick connect (only the TRN1d known
 *          data state checks it).
 */

#include "v90_phase3_demod.h"
#include "core/dsp_host.h"
#include "v34/v34_api.h"

extern "C"
{
}

typedef P34_V90Phase3Demodulator D;
typedef P18_V90AutoDigitalImpDetector Adid;

/* ------------------------------------------------------------------------------------------------ helpers */

#include "v90_phase3_demod_helpers.h"

/**
 * @brief Run one sample through the V.90 phase-3 state machine.
 * @param x  the equalized PCM-level sample (float); passed bit for bit from getDecision()
 * @return the decision symbol for this sample (16-bit linear level), or 0 in the terminated, error and unused states
 * @note  Side effects: cnt_ is incremented first, event_ is cleared and then set when a transition happens, state_ changes on transitions.
 */
short V90Phase3Demodulator::getV90Decision(float x)
{
  D* t = this;
  Adid* a;
  V90Parameters* p;
  short s = trunc16f(x); /* the input as a 16-bit symbol: the result of the pass-through states */

  t->cnt_++;
  t->event_ = 0;

  switch (t->state_)
  {
  case P3D_WAIT_FOR_SD:
  { /* WaitForSd: feed the Sd detector, latch the end of the tone, time out after 12000 + timeBase_ symbols */
    V90SdDetector* sd = t->sd_;
    int res;
    if (sd->count_ == 10)
      t->sdToneSeen_ = 1;
    if (t->sdToneSeen_ != 0 && sd->count_ == 0)
    { /* the detector's hit counter ran 10 -> 0: the Sd tone dropped */
      t->sdToneSeen_ = 0;
      t->event_ = V90DEMOD_STATE_SD_TONE_DROPPED;
    }
    res = sd->process(x);
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
  { /* SdDemod: wait for the detector to report the inverted tone (SdNot), time out after 0x180 symbols */
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
  { /* SdNot: 0x30 symbols after SdNot start TRN1d (known data, or DD in quick connect) and restart the study of the reference ucode */
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
        t->jd_->unPackReset();
        edprintf("V90Phase3Demodulator: enter TRN1d DD state\r\n");
      }
      t->adid_->resetStudyUrefHandler(t->lineMode_);
      t->cnt_ = 0;
      t->event_ = V90DEMOD_STATE_TRN1D_STARTED;
    }

    return s;
  }
  case P3D_TRN1D_KNOWN:
  { /* TRN1d known data: the modulator's own symbol is the result; after 0x7f8 symbols go on to TRN1d DD */
    short m = t->mod_.generateSymbol();
    if (t->cnt_ == 0x7f8)
    {
      if (t->jd_ == 0)
      {
        t->state_ = P3D_NULL_JD;
        t->event_ = V90DEMOD_STATE_PHASE3_ERROR;
        if (dsplibs_debug_level > 1)
          dsplibs_debug_printf("V90Phase3Demodulator: ERROR: Null JdDetector\r\n");
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
            dsplibs_debug_printf("V90Phase3Demodulator: setting params for short TRN1\n");
          p = t->params_;
          t->trn1Len_ = (unsigned)p->trn1QcDdLength;
          t->w3f4_ = (unsigned)p->unk4a4;
        }
        t->adid_->resetStudyUrefHandler(t->lineMode_);
        t->jd_->unPackReset();
        edprintf("V90Phase3Demodulator: enter TRN1d DD state\r\n");
        t->cnt_ = 0;
      }
    }

    return m;
  }
  case P3D_TRN1D_DD:
  { /* TRN1d DD: decide from the learned tables until trn1Len_ symbols are in */
    int b2;
    short r = decideBit(t, x, &b2);
    if (t->cnt_ == t->trn1Len_)
    {
      edprintf("V90Phase3Demodulator: enter study reference Ucode state @ %d\r\n", t->cnt_);
      t->state_ = P3D_V92_STUDY_UREF;
      t->cnt_ = 0;
      t->event_ = V90DEMOD_STATE_STUDY_UREF_ENTERED;
    }
    else
    {
      trn1dTimeout(t);
    }

    return r;
  }
  case P3D_V92_STUDY_UREF:
  { /* study of the reference ucode: the detector classifies the phases; when done (and aligned to the 6-symbol frame) wait for Jd */
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
        edprintf("V90Phase3Demodulator: enter Wait For Jd state @ %d\r\n", t->cnt_);
        t->state_ = P3D_STATE6;
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
  case P3D_STATE6:
  { /* wait for the V90Jd frame: feed the descrambled bits to the detector (the TRN1d time-out applies) */
    int b2;
    short r = decideBit(t, x, &b2);

    advancePhase(t);
    if ((unsigned char)t->jd_->unPackData(b2) != 0)
    {
      if (t->cnt_ % 6 != 0 && t->lineMode_ == 0)
      {
        t->jd_->unPackReset();
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
        edprintf("V90Phase3Demodulator: Jd detected @ %d\r\n", t->cnt_);
        t->event_ = V90DEMOD_STATE_JD_DETECTED;
        t->state_ = P3D_JD_DEMOD;
        t->cnt_ = 0;
        t->jdNotCnt_ = 0;
      }
    }
    trn1dTimeout(t);

    return r;
  }
  case P3D_JD_DEMOD:
  { /* JdDemod: decide bits until JdNot (more than 11 zero bits, at cnt_ % 72 == 12) starts the DIL; time out after 38760 + timeBase_ symbols */
    int b2;
    short r = decideBit(t, x, &b2);

    advancePhase(t);
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
  case P3D_FIRST_STUDY: /* DIL first study */
  case P3D_QC_FIRST_STUDY:
  { /* DIL first study (also the quick-connect one): learn mean and variance; at the end of the stage evaluate it and go to the second study */
    short m = t->mod_.generateSymbol();
    short r = s;
    a = t->adid_;
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
  { /* DIL second study: refine the levels; raise the stage events; end with the third study (unk310 != 0) or error relaxation */
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
  { /* DIL third study: collect samples for the phases not yet resolved; at the end evaluate the second study and go to error relaxation */
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
  { /* quick-connect DIL second study: levels from the previous session for unsuspected phases; ends in the quick-connect third study */
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
  { /* quick-connect DIL third study: at the end setQcLinearMapping() builds the mapping and error relaxation starts */
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
  { /* DIL error relaxation: decisions from the learned tables; raises DIL_INITIAL_ERROR_RELAX_END and DIL_RECEIVED at their counts */
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
  { /* probing DIL: the decision is the received sample; status_ marks the full-scale part of the signal after the DFE length */
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
  /* terminated and error states: no output */
  case P3D_TERMINATED:
  case P3D_WAIT_SD_TIMEOUT:
  case P3D_SD_TIMEOUT:
  case P3D_TRN1D_TIMEOUT:
  case P3D_JD_TIMEOUT:
  case P3D_DIL_TIMEOUT:
  case P3D_NULL_JD:
    return 0;
  case P3D_WAIT_QTS:
  { /* WaitFor QTS (quick connect): the ANSam detector runs, the Sd detector looks for QTS; on a time-out assume another line */
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
  { /* WaitFor QTSNot: wait for the inverted tone; on a time-out assume another line and go to ANSpcm recovery */
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
  case P3D_QTS_NOT_DONE:
  { /* 0x30 symbols after QTSNot: start the ANSpcm demodulation and mark the line verified (verify_ = 1) */
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
  case P3D_ANSPCM_DEMOD:
  { /* ANSpcm demodulation: only the length is checked; the owner reacts to the event */
    t->ans_->process(x);

    p = t->params_;
    if (t->cnt_ == (unsigned)p->anspcmDemodulationLength)
      t->event_ = V90DEMOD_STATE_ANSPCM_DEMOD_OVER;

    return s;
  }
  case P3D_ANSPCM_WAIT:
  { /* wait for the ANSpcm energy drop */
    if (t->ans_->process(x) == 0)
    {
      if (dsplibs_debug_level > 1)
        dsplibs_debug_printf("V90Phase3Demodulator: ANSpcm energy drop detected...\r\n");
      t->event_ = V90DEMOD_STATE_ANSPCM_DROP_DETECTED;
      t->state_ = P3D_STATE31;
    }

    return s;
  }
  case P3D_STATE31: /* after the ANSpcm drop: nothing more to do */
    return s;
  case P3D_ANSPCM_DROP:
  { /* QTS timed out: wait for ANSpcm; fake the energy drop after 0x7d0 symbols */
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
  case P3D_ANSPCM_RECOVERY:
  { /* QTSNot timed out: ANSpcm seen after the demodulation length is reported as demodulation over */
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
  default: /* states 7, 8, 18 and above 33 */
    break;
  }
  return 0;
}
