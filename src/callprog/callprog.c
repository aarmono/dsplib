/**
 * @file callprog.c
 * @brief CALLPROG: the call-progress state machine that supervises dialling and the call set-up phase.
 *
 * Entry points: CALLPROG_Create() builds the machine (cadence detectors for dial and busy tone, the answer-tone detector, the optional noise pre-filter, the
 * decision tables), CALLPROG_Dial() starts a call for a dial string (creates the dialer, sets the timeouts), CALLPROG_Progress() processes one buffer of
 * line samples and returns a message, CALLPROG_Delete() releases everything, CALLPROG_Status_string() names a message.  The object is CallProg
 * (callprog.h); the caller is the call glue in dp_call.c.
 *
 * States (CallProgState): WAIT_DIAL (wait for dial tone, or for the blind-dial pause), DIALING (the dialer runs), WAIT_RING (the calling tone is played),
 * WAIT_TO_ANSWER, ANSWER_STATE, END, END_PARTIALLY (the dial string ended with ';'), WFS (wait for 5 s of silence, the dial-string character '@'), BONGTONE
 * ('$').  Events (CallProgEvent): 1 dial tone seen by the dial cadence, 2 busy seen by the busy cadence, 7 no answer.
 *
 * One CALLPROG_Progress() call handles a buffer of at most 160 samples (8000 Hz).  The (optionally noise-filtered) copy goes sample by sample through the
 * dial and busy cadences, as the state selects; the timeout and line-clear counters tick per sample; the 2100 / 2250 Hz detector looks at the whole
 * unfiltered buffer in the automatic states; WFS looks for silence; WAIT_RING plays the calling tone; DIALING runs the dialer.  The result is a message
 * (CallProgMessage).  A requested state change takes effect at the end of the call.
 *
 * Decision tables.  Which state follows which event or timeout, and the message that goes with it, are tables indexed by [state] or [state][event].  They are
 * file-static arrays rebuilt by CALLPROG_Create() and (partly) CALLPROG_Dial(), so every CallProg object shares them.
 *
 * Behaviour worth knowing: the event `ev` is not cleared inside a call, so once set every following sample re-arms `event` and `lcCount` and re-applies the
 * table message; in state END the message is replaced by the previous result; '^' in a dial string loops back into the dialer; the line-clear timeout is
 * never armed (see CALLPROG_Progress()).
 */
#include "callprog.h"
#include "callprog_tables.h"
#include "core/dsp_asm.h"
#include "core/dsp_debug_printf.h" /* dsplibs_debug_level, dsplibs_debug_printf, DBG(...) */
#include "core/dsp_constants.h"    /* SAMPLE_RATE_* */

/* The decision tables (see the file header and CALLPROG_Create()). */
static unsigned char next_state_due_cptd[10][8];
static unsigned char message_due_cptd[10][8];
static unsigned char next_state_due_line_clear_timeout[10];
static unsigned char message_due_line_clear_timeout[10];
static unsigned char next_state_due_timeout[10];
static unsigned char message_due_timeout[10];
static int timeout_table[10];
static int enable_line_clear_timeout[10];
static unsigned char automode_table[10];
static unsigned char toneiir_dialtone_table[10];
static unsigned char toneiir_busy_table[10];

static const char* const state_names[10] = {"CALLPROG_NO_LEGAL_STATE", "CALLPROG_WAIT_DIAL",     "CALLPROG_DIALING", "CALLPROG_WAIT_RING",
                                            "CALLPROG_WAIT_TO_ANSWER", "CALLPROG_ANSWER_STATE",  "CALLPROG_END",     "CALLPROG_END_PARTIALLY_STATE",
                                            "CALLPROG_WFS_STATE",      "CALLPROG_BONGTONE_STATE"};
static const struct
{
  int id;
  const char* name;
} message_names[17] = {{0, "CALLPROG_NO_MESSAGE"},
                       {1, "CALLPROG_NO_RING"},
                       {2, "CALLPROG_NO_DIAL_TONE"},
                       {3, "CALLPROG_DIALING"},
                       {4, "CALLPROG_END_DIALING"},
                       {5, "CALLPROG_RINGBACK"},
                       {6, "CALLPROG_NO_ANSWER"},
                       {7, "CALLPROG_ANSWER"},
                       {8, "CALLPROG_MODEM_ANSWER"},
                       {9, "CALLPROG_VOICE_ANSWER"},
                       {10, "CALLPROG_BUSY"},
                       {11, "CALLPROG_CONGESTION"},
                       {12, "CALLPROG_ERROR"},
                       {13, "CALLPROG_ANSWER_STATE_TIMEOUT"},
                       {14, "CALLPROG_END_DIALING_PARTIALLY"},
                       {15, "CALLPROG_V8BIS_MODEM_ANSWER"},
                       {18, "CALLPROG_MAX_MESSAGES"}};

/**
 * @brief Name of a message.
 * @param id  CallProgMessage
 * @return the name ("CALLPROG_BUSY", ...), or "" for an unknown id (also for the unnamed 16 and 17)
 */
const char* CALLPROG_Status_string(int id)
{
  const char* r = "";
  int i;
  for (i = 0; i <= 16; i++)
    if (message_names[i].id == id)
      r = message_names[i].name;
  return r;
}

/*
 * Request a state change.  Honoured only once per call (the first request wins), not towards the state the previous call ended in, and not towards state 0.
 * The change itself is applied at the end of CALLPROG_Progress().
 */
static void request_state(CallProg* cp, unsigned target)
{
  if (cp->change != 1 && (unsigned)cp->prevState != target && target != 0)
  {
    if (dsplibs_debug_level > 1u)
    {
      const char *from = state_names[cp->state], *to = state_names[target];
      COMPILER_BARRIER(); /* the loads happen before the call set-up whatever the aliasing rules */
      dsplibs_debug_printf("STATE:  %s --> %s\n", from, to);
    }
    cp->nextState = (int)target;
    cp->change = 1;
  }
}

/* (Re)build the timeout tables from cp->cfg: the next state and message of each state's timeout, and the timeout in seconds. Used by Create and Dial. */
static void init_timeout_tables(CallProg* cp)
{
  int i;
  for (i = 0; i < 10; i++)
    next_state_due_timeout[i] = CALLPROG_ST_NO_LEGAL_STATE;
  for (i = 0; i < 10; i++)
    timeout_table[i] = 0;
  for (i = 0; i < 10; i++)
    message_due_timeout[i] = CALLPROG_MSG_NO_MESSAGE;
  next_state_due_timeout[CALLPROG_ST_WAIT_DIAL] = CALLPROG_ST_END;
  message_due_timeout[CALLPROG_ST_WAIT_DIAL] = CALLPROG_MSG_NO_DIAL_TONE;
  next_state_due_timeout[CALLPROG_ST_WAIT_RING] = CALLPROG_ST_END;
  message_due_timeout[CALLPROG_ST_WAIT_RING] = CALLPROG_MSG_NO_RING;
  next_state_due_timeout[CALLPROG_ST_WAIT_TO_ANSWER] = CALLPROG_ST_END;
  message_due_timeout[CALLPROG_ST_WAIT_TO_ANSWER] = CALLPROG_MSG_NO_ANSWER;
  next_state_due_timeout[CALLPROG_ST_ANSWER_STATE] = CALLPROG_ST_END;
  message_due_timeout[CALLPROG_ST_ANSWER_STATE] = CALLPROG_MSG_ANSWER_STATE_TIMEOUT;
  next_state_due_timeout[CALLPROG_ST_WFS_STATE] = CALLPROG_ST_END;
  message_due_timeout[CALLPROG_ST_WFS_STATE] = CALLPROG_MSG_BUSY;
  next_state_due_timeout[CALLPROG_ST_BONGTONE_STATE] = CALLPROG_ST_END;
  timeout_table[CALLPROG_ST_WAIT_DIAL] = cp->cfg[0];
  timeout_table[CALLPROG_ST_WAIT_RING] = cp->cfg[1];
  timeout_table[CALLPROG_ST_WAIT_TO_ANSWER] = cp->cfg[2];
  timeout_table[CALLPROG_ST_ANSWER_STATE] = cp->cfg[3];
  timeout_table[CALLPROG_ST_WFS_STATE] = cp->cfg[5];
  timeout_table[CALLPROG_ST_BONGTONE_STATE] = cp->cfg[6];
}

/**
 * @brief Build the call-progress machine.
 * @param cp   object to initialise (embedded in the call glue)
 * @param cfg  blind flag, S-register reader and modem (read once)
 *
 * Creates the busy and dial cadence detectors (tone/tone_cadence.c) and the 2100 / 2250 Hz detector, the noise pre-filter when the host asks for it
 * (MustNoiseFilterBeApplied), sets the default timeouts (CallProg::cfg), fills the decision tables, and starts in WAIT_DIAL with its timeout.
 * @warning The allocation results of cadence_create(), Dual_TONE_create() and _iir_filter_create() are not checked; a failure leaves a NULL pointer that
 *          CALLPROG_Progress() dereferences.  The decision tables are shared by all objects.
 */
void CALLPROG_Create(CallProg* cp, CallProgCfg* cfg)
{
  CadenceParams p;
  int i, j;
  p.w0 = 0x32;
  p.w4 = 0x32;
  p.w8 = 3;
  p.callerParamA = 0;
  p.type = CADENCE_BUSY;
  p.w14 = 0;
  p.callerParamB = 0;
  DBG("CallProgFP_Create >>\n");

  /* take over the configuration */
  {
    int cblind = cfg->blind, cw0c = cfg->w0c; /* read once: the stores below must not move the loads */
    int (*cfn)(struct modem*, int) = cfg->getSReg;
    struct modem* cm = cfg->modem;
    COMPILER_BARRIER(); /* the loads of cfg stay before the stores into cp */
    cp->blind = cblind;
    cp->getSReg = cfn;
    cp->modem = cm;
    cp->w28 = cw0c;
  }

  /* noise pre-filter, cadence detectors, answer-tone detector */
  cp->iir = NULL;
  cp->applyFilter = (int)modem_get_param(cp->modem, MustNoiseFilterBeApplied);
  DBG("APPLY_FILTER = %d\n", cp->applyFilter);
  if (cp->applyFilter != 0)
    cp->iir = _iir_filter_create(cp->iir, 12, 12, callprog_filter_a, callprog_filter_b, callprog_biquad_scales);
  cp->dialSeen = 0;
  p.callerParamB = 1;
  p.type = CADENCE_BUSY;
  cp->busyCad = cadence_create(NULL, &p, 0, cp->modem);
  p.type = CADENCE_DIAL;
  cp->dialCad = cadence_create(NULL, &p, 0, cp->modem);
  cp->dual = Dual_TONE_create();

  /* decision tables: cadence events (every state: busy -> END / BUSY, column 3 -> END / CONGESTION, column 6 -> END / ERROR), then the per-state entries */
  for (i = 0; i < 7; i++)
    cp->cfg[i] = callprog_default_cfg[i];
  for (i = 0; i < 10; i++)
    for (j = 0; j < 8; j++)
      next_state_due_cptd[i][j] = CALLPROG_ST_NO_LEGAL_STATE;
  for (i = 0; i < 10; i++)
    for (j = 0; j < 8; j++)
      message_due_cptd[i][j] = CALLPROG_MSG_NO_MESSAGE;
  for (i = 0; i < 10; i++)
    next_state_due_line_clear_timeout[i] = CALLPROG_ST_NO_LEGAL_STATE;
  for (i = 0; i < 10; i++)
    message_due_line_clear_timeout[i] = CALLPROG_MSG_NO_MESSAGE;
  for (i = 0; i < 10; i++)
  {
    next_state_due_cptd[i][CALLPROG_EV_BUSY] = CALLPROG_ST_END;
    message_due_cptd[i][CALLPROG_EV_BUSY] = CALLPROG_MSG_BUSY;
    next_state_due_cptd[i][3] = CALLPROG_ST_END;
    message_due_cptd[i][3] = CALLPROG_MSG_CONGESTION;
    next_state_due_cptd[i][6] = CALLPROG_ST_END;
    message_due_cptd[i][6] = CALLPROG_MSG_ERROR;
  }
  next_state_due_cptd[CALLPROG_ST_WAIT_DIAL][CALLPROG_EV_DIAL_TONE] = CALLPROG_ST_DIALING;
  message_due_cptd[CALLPROG_ST_WAIT_DIAL][CALLPROG_EV_DIAL_TONE] = CALLPROG_MSG_DIALING;
  next_state_due_cptd[CALLPROG_ST_WAIT_DIAL][4] = CALLPROG_ST_END;
  message_due_cptd[CALLPROG_ST_WAIT_DIAL][4] = CALLPROG_MSG_ERROR;
  next_state_due_cptd[CALLPROG_ST_WAIT_DIAL][5] = CALLPROG_ST_END;
  message_due_cptd[CALLPROG_ST_WAIT_DIAL][5] = CALLPROG_MSG_ERROR;
  next_state_due_cptd[CALLPROG_ST_WAIT_RING][CALLPROG_EV_DIAL_TONE] = CALLPROG_ST_END;
  message_due_cptd[CALLPROG_ST_WAIT_RING][CALLPROG_EV_DIAL_TONE] = CALLPROG_MSG_ERROR;
  next_state_due_cptd[CALLPROG_ST_WAIT_RING][4] = CALLPROG_ST_WAIT_TO_ANSWER;
  message_due_cptd[CALLPROG_ST_WAIT_RING][4] = CALLPROG_MSG_RINGBACK;
  next_state_due_cptd[CALLPROG_ST_WAIT_RING][5] = CALLPROG_ST_WAIT_TO_ANSWER;
  message_due_cptd[CALLPROG_ST_WAIT_RING][5] = CALLPROG_MSG_RINGBACK;
  next_state_due_cptd[CALLPROG_ST_WAIT_TO_ANSWER][CALLPROG_EV_DIAL_TONE] = CALLPROG_ST_END;
  message_due_cptd[CALLPROG_ST_WAIT_TO_ANSWER][CALLPROG_EV_DIAL_TONE] = CALLPROG_MSG_ERROR;
  next_state_due_cptd[CALLPROG_ST_WAIT_TO_ANSWER][4] = CALLPROG_ST_WAIT_TO_ANSWER;
  message_due_cptd[CALLPROG_ST_WAIT_TO_ANSWER][4] = CALLPROG_MSG_NO_MESSAGE;
  next_state_due_cptd[CALLPROG_ST_WAIT_TO_ANSWER][5] = CALLPROG_ST_WAIT_TO_ANSWER;
  message_due_cptd[CALLPROG_ST_WAIT_TO_ANSWER][5] = CALLPROG_MSG_NO_MESSAGE;
  next_state_due_cptd[CALLPROG_ST_ANSWER_STATE][CALLPROG_EV_DIAL_TONE] = CALLPROG_ST_END;
  message_due_cptd[CALLPROG_ST_ANSWER_STATE][CALLPROG_EV_DIAL_TONE] = CALLPROG_MSG_ERROR;
  next_state_due_cptd[CALLPROG_ST_ANSWER_STATE][4] = CALLPROG_ST_WAIT_TO_ANSWER;
  message_due_cptd[CALLPROG_ST_ANSWER_STATE][4] = CALLPROG_MSG_NO_MESSAGE;
  next_state_due_cptd[CALLPROG_ST_ANSWER_STATE][5] = CALLPROG_ST_WAIT_TO_ANSWER;
  message_due_cptd[CALLPROG_ST_ANSWER_STATE][5] = CALLPROG_MSG_NO_MESSAGE;
  next_state_due_line_clear_timeout[CALLPROG_ST_WAIT_TO_ANSWER] = CALLPROG_ST_ANSWER_STATE;
  message_due_line_clear_timeout[CALLPROG_ST_WAIT_TO_ANSWER] = CALLPROG_MSG_ANSWER;

  /* timeouts, which detectors listen in which state, line clear */
  init_timeout_tables(cp);
  for (i = 0; i < 10; i++)
    toneiir_dialtone_table[i] = 0;
  toneiir_dialtone_table[CALLPROG_ST_WAIT_DIAL] = 1;
  toneiir_dialtone_table[CALLPROG_ST_DIALING] = 1;
  for (i = 0; i < 10; i++)
    toneiir_busy_table[i] = 1;
  for (i = 0; i < 10; i++)
    automode_table[i] = 0;
  automode_table[CALLPROG_ST_WAIT_RING] = 1;
  automode_table[CALLPROG_ST_WAIT_TO_ANSWER] = 1;
  automode_table[CALLPROG_ST_ANSWER_STATE] = 1;
  for (i = 0; i < 10; i++)
    enable_line_clear_timeout[i] = 0;

  /* start in WAIT_DIAL */
  cp->state = CALLPROG_ST_WAIT_DIAL;
  cp->timer = (int)((unsigned)cp->cfg[0] * SAMPLE_RATE_8000); /* wraps */
  if (next_state_due_line_clear_timeout[CALLPROG_ST_WAIT_DIAL] != 0)
    cp->lcEnable = enable_line_clear_timeout[CALLPROG_ST_WAIT_DIAL];
  if (cp->state == CALLPROG_ST_WFS_STATE)
    cp->silence = 0;
  cp->lcLimit = (int)((unsigned)cp->cfg[4] * SAMPLE_RATE_8000);
  DBG("CALLPROG Create <<\n");
}

/**
 * @brief Release the machine: abort the dialer, free the cadence detectors, the pre-filter and the answer-tone detector.
 * @param cp  object (the object itself belongs to the caller)
 * @note  `busyCad` and `dual` are freed but not set to NULL.
 */
void CALLPROG_Delete(CallProg* cp)
{
  DialerAbort(&cp->dialer);
  DBG("CALLPROG_Delete is entered\n");
  if (cp->dialCad)
  {
    DBG("cadence_delete with CADENCE_DIAL_OBJ is invoked\n");
    cadence_delete(cp->dialCad);
  }
  if (cp->busyCad)
  {
    DBG("cadence_delete with CADENCE_OBJ is invoked\n");
    cadence_delete(cp->busyCad);
  }
  if (cp->applyFilter != 0)
    _iir_filter_delete(cp->iir);
  cp->w70 = 0;
  cp->dialCad = NULL;
  if (cp->dual)
    Dual_TONE_delete(cp->dual);
  DBG("CALLPROG_Delete is exited\n");
}

/**
 * @brief Start a call.
 * @param cp          machine (CALLPROG_Create())
 * @param dialString  dial string (see dialer.h)
 *
 * Reads the calling-tone flag and level, creates the dialer for the string (CallProg::dialRes keeps its verdict), sets the timeouts [1] .. [5] from
 * GetNoAnswerTimeOut and rebuilds the timeout tables.  In non-blind mode the dial-tone wait is GetDialToneWaitTime + max(2, ceil(GetDialToneValidationTime
 * / 10)) seconds and both cadence detectors listen in WAIT_DIAL and DIALING.  In blind mode the detectors are off and the machine waits GetBlindDialPause
 * seconds before it dials.  If the previous string ended partially (';') the machine continues in DIALING, else it starts in WAIT_DIAL.
 * @note  Nothing happens (apart from a debug message) when the S-register reader is NULL.
 */
void CALLPROG_Dial(CallProg* cp, const char* dialString)
{
  int s, i;
  DBG("CALLPROG Dialing %s\n", dialString);
  if (cp->getSReg == NULL)
  {
    DBG("sreg function is not defined!\n");
    return;
  }
  cp->callTone = (int)modem_get_param(cp->modem, GetCallingToneFlag);
  cp->callToneOn = cp->callTone == 0 ? 0 : cp->callTone == 1 ? 1 : (cp->callTone == 2);
  ResetCallingTone(&cp->ct, (signed char)cp->getSReg(cp->modem, 0xdd));
  cp->dialRes = DialerCreate(&cp->dialer, dialString, cp->modem);

  /* timeouts */
  if (dsplibs_debug_level > 1u)
    dsplibs_debug_printf("GetNoAnswerTimeOut. %d\n", (int)modem_get_param(cp->modem, GetNoAnswerTimeOut));
  for (i = 1; i <= 5; i++)
    cp->cfg[i] = (int)modem_get_param(cp->modem, GetNoAnswerTimeOut);
  init_timeout_tables(cp);

  /* dial-tone wait, or the blind-dial pause */
  if (cp->blind == 0)
  {
    int a = (int)modem_get_param(cp->modem, GetDialToneWaitTime);
    int b = (int)modem_get_param(cp->modem, GetDialToneValidationTime);
    int t = (int)((unsigned)a + 2u);
    if ((int)((unsigned)b + 9u) / 10 > 2)
      t = (int)((unsigned)a + (unsigned)((int)((unsigned)(int)modem_get_param(cp->modem, GetDialToneValidationTime) + 9u) / 10)); /* wraps */
    next_state_due_timeout[CALLPROG_ST_WAIT_DIAL] = CALLPROG_ST_END;
    message_due_timeout[CALLPROG_ST_WAIT_DIAL] = CALLPROG_MSG_NO_DIAL_TONE;
    toneiir_dialtone_table[CALLPROG_ST_WAIT_DIAL] = 1;
    toneiir_busy_table[CALLPROG_ST_WAIT_DIAL] = 1;
    toneiir_dialtone_table[CALLPROG_ST_DIALING] = 1;
    toneiir_busy_table[CALLPROG_ST_DIALING] = 1;
    timeout_table[CALLPROG_ST_WAIT_DIAL] = t;
    DBG("WAIT DIAL TIMEOUT = %d\n", t);
  }
  else
  {
    if (dsplibs_debug_level > 1u)
      dsplibs_debug_printf("BlindCall: GetBlindDialPause = %d .\n", (int)modem_get_param(cp->modem, GetBlindDialPause));
    timeout_table[CALLPROG_ST_WAIT_DIAL] = (int)modem_get_param(cp->modem, GetBlindDialPause);
    next_state_due_timeout[CALLPROG_ST_WAIT_DIAL] = CALLPROG_ST_DIALING;
    message_due_timeout[CALLPROG_ST_WAIT_DIAL] = CALLPROG_MSG_DIALING;
    toneiir_dialtone_table[CALLPROG_ST_WAIT_DIAL] = 0;
    toneiir_busy_table[CALLPROG_ST_WAIT_DIAL] = 0;
    toneiir_dialtone_table[CALLPROG_ST_DIALING] = 0;
    toneiir_busy_table[CALLPROG_ST_DIALING] = 0;
  }

  /* enter the first state */
  if (cp->state == CALLPROG_ST_END_PARTIALLY_STATE)
  {
    DBG("Set state to CALLPROG_DIALING_STATE\n");
    cp->state = CALLPROG_ST_DIALING;
  }
  else
  {
    cp->state = CALLPROG_ST_WAIT_DIAL;
  }
  s = cp->state;
  cp->timer = (int)((unsigned)timeout_table[s] * SAMPLE_RATE_8000);
  if (next_state_due_line_clear_timeout[s] != 0)
    cp->lcEnable = enable_line_clear_timeout[s];
  if (s == CALLPROG_ST_WFS_STATE)
    cp->silence = 0;
  DBG("CALLPROG_Dial was exited.\n");
}

/**
 * @brief Process one buffer of line samples.
 * @param cp   machine
 * @param in   received samples (8000 Hz)
 * @param out  transmit samples to produce (dial tones, calling tone, or zeros)
 * @param n    number of samples; at most 160 (a longer buffer returns CALLPROG_MSG_ERROR without processing)
 * @return a CallProgMessage; CALLPROG_MSG_NO_MESSAGE when nothing happened
 *
 * Per call: copy the input (noise-filtered while the dial tone has not been seen); on a dialer error return CALLPROG_MSG_ERROR; for each sample run the
 * dial and busy cadences if the state's table says so, apply a cadence event (message and state change of the tables), count down the state timeout (a
 * timeout applies its message and state change; none fires in DIALING and END_PARTIALLY) and the line-clear counter.  In the automatic states
 * (WAIT_RING, WAIT_TO_ANSWER, ANSWER_STATE) the 2100 / 2250 Hz detector looks at the unfiltered buffer (messages 0x10 / 0x11).  WFS counts buffers of
 * silence (an envelope of |x| below 100) and starts DIALING after 5 s.  WAIT_RING plays the calling tone.  DIALING runs the dialer and maps its result
 * to a state request and message.  A requested state change is applied at the end of the call, together with the new state's timeout.
 * @bug   The line-clear timeout can never fire: every entry of enable_line_clear_timeout is 0, so CallProg::lcEnable is never set and the WAIT_TO_ANSWER ->
 *        ANSWER transition by line-clear timeout is unreachable.
 * @warning When DialerCreate() failed (dialRes is DIALER_ERROR_MSG) every call requests the change to END but returns before applying it, so the machine
 *          stays in its state and returns CALLPROG_MSG_ERROR on each call.  A negative `n` passes the length check and gives the copy a huge size.
 */
int CALLPROG_Progress(CallProg* cp, const short* in, short* out, int n)
{
  short tmp[0xa0];
  int ev = CALLPROG_EV_NONE, ret = 0, i, state;
  int dres, j;
  if (n > 0xa0)
  {
    DBG("Invalid buffer length\n");
    return CALLPROG_MSG_ERROR;
  }

  /* copy (and filter) the input; a dialer error ends the call set-up */
  sysdep_memcpy(tmp, in, (size_t)(unsigned)n * 2u);
  cp->change = 0;
  cp->event = CALLPROG_EV_NONE;
  if (cp->applyFilter != 0 && cp->dialSeen == 0)
    _iir_filter_progress(cp->iir, n, tmp);
  if (cp->dialRes == DIALER_ERROR_MSG)
  {
    DBG("DIALER_ERROR_MSG encountered in Callprog_progress.\n");
    if (cp->change == 1)
      return CALLPROG_MSG_ERROR;
    if (cp->prevState == CALLPROG_ST_END)
      return CALLPROG_MSG_ERROR;
    request_state(cp, CALLPROG_ST_END);
    return CALLPROG_MSG_ERROR;
  }

  /* per sample: cadence events, timeouts, line clear */
  state = cp->state;
  for (i = 0; i < n; i++)
  {
    if (toneiir_dialtone_table[state] == 1)
    {
      if (cadence_progress(cp->dialCad, tmp[i]) == 1 && ev == CALLPROG_EV_NONE)
      {
        DBG("tone detected by cadence dial\n");
        cp->dialSeen = 1;
        ev = CALLPROG_EV_DIAL_TONE;
      }
    }
    if (toneiir_busy_table[state] == 1)
    {
      int r = cadence_progress(cp->busyCad, tmp[i]);
      if (r == 1)
      {
        DBG("busy detected by cadence\n");
        ev = CALLPROG_EV_BUSY;
      }
      else if (r == 7)
      {
        DBG("no answer detected by cadence\n");
        cp->timer = 0;
        ev = CALLPROG_EV_NO_ANSWER;
      }
    }

    if (ev != CALLPROG_EV_NONE)
    {
      cp->event = ev;
      cp->lcCount = 0;
      ret = message_due_cptd[state][ev];
      request_state(cp, next_state_due_cptd[state][ev]);
    }

    if (cp->timer > 0)
    {
      cp->timer--;
    }
    else if (cp->timer == 0 && state != CALLPROG_ST_DIALING && state != CALLPROG_ST_END_PARTIALLY_STATE)
    {
      cp->timer = -10;
      DBG("CALLPROG: Time out\n");
      ret = message_due_timeout[state];
      request_state(cp, next_state_due_timeout[state]);
    }

    if (state == CALLPROG_ST_END)
      ret = cp->lastRet;
    if (cp->lcEnable == 1)
    {
      cp->lcCount = (int)((unsigned)cp->lcCount + 1u);
      if (cp->lcCount > cp->lcLimit)
      {
        DBG("CALLPROG: LINE CLEAR TIMEOUT\n");
        request_state(cp, next_state_due_line_clear_timeout[state]);
        ret = message_due_line_clear_timeout[state];
        cp->lcEnable = 0;
      }
    }
  }

  /* answer tones in the automatic states */
  if (automode_table[state] == 1)
  {
    int r = Dual_TONE_detect(cp->dual, in, n);
    if (r == 3)
    {
      DBG("Found 2100\n");
      ret = CALLPROG_MSG_DETECTED_2100;
    }
    else if (r == 5)
    {
      DBG("Found 2250\n");
      ret = CALLPROG_MSG_DETECTED_2250;
    }
  }

  /* WFS: wait for silence; WAIT_RING: calling tone */
  if (state == CALLPROG_ST_WFS_STATE)
  {
    if (n != 0)
    {
      int thr = 40000 / n; /* buffers in 5 seconds */
      int acc = 0, k;
      for (k = 0; k < n; k++)
      {
        int v = (int)tmp[k] * 0xa4;
        int sa = v >> 31;
        int a = ((v >> 14) ^ sa) - sa;
        int w = acc * 0x3f5c;
        int sb = w >> 31;
        int b = ((w >> 14) ^ sb) - sb;
        acc = (short)(a + b);
      }
      if (n > 0 && acc > 99)
      {
        cp->silence = 0;
      }
      else
      {
        cp->silence = (int)((unsigned)cp->silence + 1u);
        if (cp->silence == thr)
        {
          DBG("CALLPROG: 5 sec. silence was detected.\n");
          request_state(cp, CALLPROG_ST_DIALING);
        }
      }
    }
  }
  else if (state == CALLPROG_ST_WAIT_RING)
  {
    if (cp->callToneOn != 0)
      GenerateCallingTone(&cp->ct, out, n);
    else
      sysdep_memset(out, 0, (size_t)(unsigned)n * 2u);
  }

  /* DIALING: run the dialer */
  j = 0;
  dres = CALLPROG_MSG_NO_MESSAGE;
  if (state == CALLPROG_ST_DIALING)
  {
    for (;;)
    {
      int r = DialerProgress(&cp->dialer, out, &j, n - 1);
      if (r == DIALER_DIALING_MSG)
      {
        dres = CALLPROG_MSG_DIALING;
      }
      else if (r == DIALER_WAIT_DIALTONE_MSG)
      {
        request_state(cp, CALLPROG_ST_WAIT_DIAL);
        DBG("CALLPROG: Wait dial tone, reset the cadence\n");
        cadence_reset(cp->dialCad);
        dres = CALLPROG_MSG_NO_MESSAGE;
      }
      else if (r == DIALER_WAIT_SILENCE_MSG)
      {
        request_state(cp, CALLPROG_ST_WFS_STATE);
        DBG("CALLPROG: @ encountered.\n");
        dres = CALLPROG_MSG_NO_MESSAGE;
      }
      else if (r == DIALER_WAIT_BONGTONE_MSG)
      {
        request_state(cp, CALLPROG_ST_BONGTONE_STATE);
        DBG("CALLPROG: $ encountered.\n");
        dres = CALLPROG_MSG_NO_MESSAGE;
      }
      else if (r == DIALER_CALLING_TONE_MSG)
      {
        if (cp->callTone == 0)
        {
          cp->callToneOn = 0;
          DBG("CALLPROG: ^ encountered.\n");
        }
        else if (cp->callTone == 1)
        {
          cp->callToneOn = 0;
          DBG("CALLPROG: ^ encountered. Disabling Calling-Tone.\n");
        }
        else if (cp->callTone == 2)
        {
          cp->callToneOn = 1;
          DBG("CALLPROG: ^ encountered.\n");
        }
        else if (cp->callTone == 3)
        {
          DBG("CALLPROG: ^ encountered. Enabling Calling-Tone.\n");
          cp->callToneOn = 1;
        }
        continue;
      }
      else if (r == DIALER_END_PARTIALLY_MSG)
      {
        request_state(cp, CALLPROG_ST_END_PARTIALLY_STATE);
        DBG("CALLPROG: Dialing string partially ended.\n");
        dres = CALLPROG_MSG_END_DIALING_PARTIALLY;
      }
      else if (r == DIALER_END_MSG)
      {
        request_state(cp, CALLPROG_ST_WAIT_RING);
        dres = CALLPROG_MSG_END_DIALING;
      }
      else if (r == DIALER_ERROR_MSG)
      {
        request_state(cp, CALLPROG_ST_END);
        DBG("CALLPROG: Dialing string error.\n");
        dres = CALLPROG_MSG_ERROR;
      }
      else
      {
        dres = CALLPROG_MSG_NO_MESSAGE;
      }
      break;
    }
  }

  /* zero the rest of the output; the barrier keeps cp->change from being cached across the short stores */
  if (state != CALLPROG_ST_WAIT_RING)
  {
    for (; j < n; j++)
      out[j] = 0;
    COMPILER_BARRIER();
  }
  if (dres == CALLPROG_MSG_END_DIALING)
    ret = CALLPROG_MSG_END_DIALING;
  else if (dres == CALLPROG_MSG_ERROR)
    ret = CALLPROG_MSG_ERROR;
  else if (dres == CALLPROG_MSG_END_DIALING_PARTIALLY)
    ret = CALLPROG_MSG_END_DIALING_PARTIALLY;

  /* apply the requested state change */
  if (cp->change != 0)
  {
    state = cp->nextState;
    cp->state = state;
    cp->timer = (int)((unsigned)timeout_table[state] * SAMPLE_RATE_8000);
    if (next_state_due_line_clear_timeout[state] != 0)
      cp->lcEnable = enable_line_clear_timeout[state];
    if (state == CALLPROG_ST_WFS_STATE)
      cp->silence = 0;
  }

  cp->prevState = state;
  cp->lastRet = ret;
  return ret;
}
