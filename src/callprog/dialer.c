/**
 * @file dialer.c
 * @brief Dialer: dial-string analysis, DTMF and pulse dialling, flash / pause / wait handling, the calling-tone generator and the pulse-dial helpers.
 *
 * Entry points (types and the dial-string syntax are in dialer.h): DialerCreate() analyses the string and reads the configuration, DialerProgress()
 * produces the dialling signal into the caller's buffer, DialerAbort() ends pulse dialling, GetDialerConfig() reads the host parameters,
 * ResetCallingTone() / GenerateCallingTone() produce the calling tone, and the pulse helpers PulseDialDigit() / IsPulseDialerReady() /
 * LastPulseDigitDialed() / SetPulseBreakTime() / SetPulseMakeTime() carry out pulse dialling through the host's hook control.  IsDialStringInvalid() and
 * Dialer_IsDialStringInvalid() expose the string analysis and have no caller in this library.
 *
 * Internals: AnalyseDialString() classifies a string (FATAL / INVALID / TOLERABLE / VALID) and records the last dialable symbol; GetNext() steps through
 * the string and returns the next DialerState; DialerProgress() is a state machine over those states that fills the buffer with DTMF tones (two cosines
 * from TONE_read(), callprog_dual_tone.c), silence or zeros while pulses are dialled.  The caller is callprog.c.
 *
 * Behaviour worth knowing: the between-digit sample counts are `x << 3` (8 kHz) without overflow checks; a tone digit with a digit length of 0 never
 * completes; a digit-pattern value outside 1 .. 3 hands -1 to PulseDialDigit() (see the notes at the functions).
 */
#include "callprog.h"
#include "core/dsp_asm.h"
#include "core/dsp_constants.h" /* SAMPLE_RATE_* */

/* The zero-fill loops must stay loops: gcc -O2 turns them into memset calls and the library imports no libc symbol. */
#define NOIDIOM __attribute__((optimize("no-tree-loop-distribute-patterns")))
/* The debug level is read with compiler barriers around the load, so that the order of the surrounding stores and the check is the source order whatever the
 * aliasing rules (the level may be changed by any callee, e.g. a debug print). */
static inline unsigned dbg_level(void)
{
  unsigned l;
  COMPILER_BARRIER();
  l = dsplibs_debug_level;
  COMPILER_BARRIER();
  return l;
}
#define DBGP(...)                                                                                                                                              \
  do                                                                                                                                                           \
  {                                                                                                                                                            \
    if (dbg_level() > 1)                                                                                                                                       \
      dsplibs_debug_printf(__VA_ARGS__);                                                                                                                       \
  } while (0)

/* ---------------------------------------------------------------------------------------------------------------- tables */
/* DTMF frequencies in Hz: the column (high group) frequencies at [0 .. 3], the row (low group) frequencies at [5 .. 8]; entries 4 and 9 are placeholders. */
static const short dial_freqs[10] = {1209, 1336, 1477, 1633, 0, 697, 770, 852, 941, 0};
/* High-group gain (Q14) by GetDTMFHighToneLevel - 6: 1 dB steps from 1.0 down */
static const unsigned short High_Tone_Gain[7] = {16384, 14602, 13014, 11599, 10338, 9213, 8211};
/* Low-group gain relative to the high group (Q15) by GetDTMFHighAndLowToneLevelDifference - 1: 1 dB steps */
static const short Lower_Tone_Diff[5] = {29205, 26029, 23198, 20675, 18427};
static const char* const syntaxStatName[4] = {"FATAL", "INVALID", "TOLERABLE", "VALID"};

/* ---------------------------------------------------------------------------------------------------------------- pulse-dial helpers */
/* The pulse state lives in the call glue block that dp_data points at. */
static CallPulse* pulse_of(struct dp* dp) { return &((CallGlue*)dp->dp_data)->pulse; }

/**
 * @brief Start dialling one pulse digit.
 * @param m      host modem object; the call glue is found through MDMPRM_DP_ADDR (nothing happens if there is none)
 * @param digit  digit 0 .. 9 (0 is sent as 10 pulses); other values are taken as the pulse count
 *
 * Sets the pulse count and clears the pulse timer, and tells the host (MDMPRM_PULSE_DIAL) how many pulses follow.
 * @warning The count is unsigned: a digit of (unsigned)-1 starts about four billion pulses (see the digit-pattern note in DialerProgress()).
 */
void PulseDialDigit(struct modem* m, unsigned digit)
{
  struct dp* dp = (struct dp*)modem_get_param(m, MDMPRM_DP_ADDR);
  CallPulse* p;
  unsigned n;
  if (!dp)
    return;
  DBGP("call: PulseDialDigit %lu...\n", (unsigned long)digit);
  p = pulse_of(dp);
  n = digit ? digit : 10;
  p->count = n;
  p->counter = 0;
  modem_set_param(m, MDMPRM_PULSE_DIAL, (int)n);
}

/**
 * @brief Advance the pulse generator by one tick and report whether it is idle.
 * @param m  host modem object (the call glue is found through MDMPRM_DP_ADDR; with none the dialer is ready)
 * @return 1 when no pulse is left to dial, else 0
 *
 * Each call adds 5 to the pulse timer.  During the break period (timer below breakTime) the hook is on, then off for the make period
 * (breakTime .. breakTime + makeTime), after which one pulse is counted and the timer restarts.  The host's hook is set with MDMPRM_HOOK_ON on the dp's
 * modem.
 */
int IsPulseDialerReady(struct modem* m)
{
  struct dp* dp = (struct dp*)modem_get_param(m, MDMPRM_DP_ADDR);
  CallPulse* p;
  unsigned count, counter, brk;
  if (!dp)
    return 1;
  p = pulse_of(dp);
  DBGP("call: IsPulseDialerReady !(%u) (count %d)\n", p->count, (int)p->counter);
  count = p->count;
  if (count == 0)
    return 1;
  counter = p->counter;
  brk = p->breakTime;
  if (counter >= brk)
  {
    if (p->hookOn == 0)
      goto timing;

    /* the break (hook on) period is over: hook off again */
    p->hookOn = 0;
    modem_set_param(dp->modem, MDMPRM_HOOK_ON, 0);
    DBGP("call: %d: hook off...\n", (int)p->count);
  }
  else if (p->hookOn != 0)
  {
    goto timing;
  }
  else
  { /* the break period starts: hook on */
    p->hookOn = 1;
    DBGP("call: %d: hook on...\n", (int)count);
    modem_set_param(dp->modem, MDMPRM_HOOK_ON, 1);
  }

  p->counter = p->counter + 5;
  return p->count == 0;

timing: /* hook off: when the make period is over, count the pulse and restart the timer */
  if (counter < brk + p->makeTime)
  {
    p->counter = counter + 5;
    return count == 0;
  }
  p->counter = 0;
  count--;
  p->count = count;
  p->counter = 5;
  return count == 0;
}

/**
 * @brief Tell the host that the last pulse digit is complete (MDMPRM_PULSE_DIAL = 0).
 * @param m  host modem object
 */
void LastPulseDigitDialed(struct modem* m)
{
  DBGP("call: LastPulseDigitDialed...\n");
  modem_set_param(m, MDMPRM_PULSE_DIAL, 0);
}

/**
 * @brief Set the pulse break (hook on) period.
 * @param m  host modem object (nothing happens if the call glue is not found)
 * @param t  period, in IsPulseDialerReady() timer units
 */
void SetPulseBreakTime(struct modem* m, unsigned t)
{
  struct dp* dp = (struct dp*)modem_get_param(m, MDMPRM_DP_ADDR);
  if (!dp)
    return;
  DBGP("call: SetPulseBreakTime %lu\n", (unsigned long)t);
  pulse_of(dp)->breakTime = t;
}

/**
 * @brief Set the pulse make (hook off) period.
 * @param m  host modem object (nothing happens if the call glue is not found)
 * @param t  period, in IsPulseDialerReady() timer units
 */
void SetPulseMakeTime(struct modem* m, unsigned t)
{
  struct dp* dp = (struct dp*)modem_get_param(m, MDMPRM_DP_ADDR);
  if (!dp)
    return;
  DBGP("call: SetPulseMakeTime %lu\n", (unsigned long)t);
  pulse_of(dp)->makeTime = t;
}

/* ---------------------------------------------------------------------------------------------------------------- GetDialerConfig */
/**
 * @brief Read the dialing parameters from the host into `cfg`.
 * @param cfg  configuration to fill (see DialerCfg)
 * @param m    host modem object
 *
 * The tone gains follow from GetDTMFHighToneLevel (6 .. 12, tables of 1 dB steps; 0x32d6 otherwise) and the high / low level difference (1 .. 5; 0x65ad
 * / 2^15 of the high gain otherwise).  With debug level above 1 every value is printed.
 */
void GetDialerConfig(DialerCfg* cfg, struct modem* m)
{
  long v;
  unsigned w;
  unsigned vv;
  cfg->toneDigitLen = (int)modem_get_param(m, GetDTMFDialSpeed);
  DBGP("Configuration->tone_DigitLength %d\n", cfg->toneDigitLen);
  cfg->pulseOffHookTime = (int)modem_get_param(m, GetPulseDialMakeTime);
  DBGP("Configuration->pulse_OffHookTime %d\n", cfg->pulseOffHookTime);
  cfg->pulseOnHookTime = (int)modem_get_param(m, GetPulseDialBreakTime);
  DBGP("Configuration->pulse_OnHookTime %d\n", cfg->pulseOnHookTime);
  cfg->dialPause = (int)modem_get_param(m, GetDialPauseTime);
  DBGP("Configuration->dialPauseTime %d\n", cfg->dialPause);
  cfg->flashTime = (int)modem_get_param(m, GetHookFlashTime);
  DBGP("Configuration->flashTime %d\n", cfg->flashTime);
  cfg->toneOrPulse = (int)modem_get_param(m, GetPulseDialingFlag) != 0;
  DBGP("Configuration->toneOrPulseFlag %d\n", cfg->toneOrPulse);
  cfg->modValid = (int)modem_get_param(m, GetDialModifierValidation);
  DBGP("Configuration->dialModifierValidationFlag %d\n", cfg->modValid);
  cfg->abcd = (int)modem_get_param(m, GetABCDDialingPermittedFlag);
  DBGP("Configuration->ABCD_PermittedFlag %d\n", cfg->abcd);
  cfg->pulseAndTone = (int)modem_get_param(m, GetPulseAndToneDialInSameDialStringPermittedFlag);
  DBGP("Configuration->pulseAndToneInSameStringPermittedFlag %d\n", cfg->pulseAndTone);
  cfg->callingTone = (int)modem_get_param(m, GetCallingToneFlag);
  DBGP("Configuration->callingToneFlag %d\n", cfg->callingTone);
  cfg->commaLimit = (int)modem_get_param(m, GetComaPauseDurationLimit);
  DBGP("Configuration->commaPauseDurLimit %d\n", cfg->commaLimit);
  cfg->digitPattern = (int)modem_get_param(m, GetPulseDialDigitPattern);
  DBGP("Configuration->digitPattern %d\n", cfg->digitPattern);
  cfg->toneBetween = (int)modem_get_param(m, GetDTMFDialSpeed);
  DBGP("Configuration->tone_BetweenDigitsInterval %d\n", cfg->toneBetween);
  cfg->pulseBetween = (int)((unsigned)(int)modem_get_param(m, GetPulseBetweenDigitsInterval) * 10u);
  DBGP("Configuration->pulse_BetweenDigitsInterval %d\n", cfg->pulseBetween);

  /* tone gains from the level and the high / low difference */
  v = modem_get_param(m, GetDTMFHighToneLevel);
  w = (unsigned)(int)modem_get_param(m, GetDTMFHighAndLowToneLevelDifference);
  vv = (unsigned)(int)v;
  if (vv - 6u <= 6u)
    cfg->gain2 = (short)High_Tone_Gain[vv - 6u];
  else
    cfg->gain2 = 0x32d6;
  if (w - 1u <= 4u)
    cfg->gain1 = (short)(((int)Lower_Tone_Diff[w - 1u] * (int)cfg->gain2) >> 15);
  else
    cfg->gain1 = (short)(((int)cfg->gain2 * 0x65ad) >> 15);
  DBGP("DTMF_Gain1 = %d\n", (int)cfg->gain1);
  DBGP("DTMF_Gain2 = %d\n", (int)cfg->gain2);
}

/* ---------------------------------------------------------------------------------------------------------------- AnalyseDialString  */
/*
 * Classify a dial string.  Returns 0 FATAL (more than 100 characters), 1 INVALID, 2 TOLERABLE, 3 VALID.  NULL and the empty string are VALID; a string whose
 * first character is not T / t / P / p is INVALID.  An illegal character makes the string INVALID when cfg.modValid is 0, else TOLERABLE.  Illegal are:
 * characters outside the dial alphabet, ';' anywhere but last, A .. D when cfg.abcd is non-zero, '^' when cfg.callingTone is 2, and a T / P that changes the
 * mode when cfg.pulseAndTone is non-zero.  With `record` non-zero the index of the last dialable symbol is stored in d->lastIdx (-2 = none).
 */
static int AnalyseDialString(Dialer* d, const char* s, int record)
{
  int mode, i, lastIdx, res;
  unsigned char c;
  d->lastIdx = -2;
  if (!s)
    return 3;
  if (sysdep_strlen(s) > 0x64)
    return 0;
  DBGP("AnalyzeDialString: Updated 17 May 1999 00:50\n");
  c = (unsigned char)s[0];
  if (c == 't' || c == 'T')
    mode = 1;
  else if (c == 'p' || c == 'P')
    mode = 0;
  else
    return c == 0 ? 3 : 1;

  /* scan the characters after the prefix */
  res = 3;
  lastIdx = -2;
  for (i = 1; (c = (unsigned char)s[i]) != 0; i++)
  {
    int bad = 0;
    int ch = (signed char)c - 0x20;
    if ((unsigned)ch > 0x57u)
    {
      bad = 1;
    }
    else
    {
      switch (ch + 0x20)
      {
      case ' ':
      case '$':
      case '(':
      case ')':
      case ',':
      case '-':
      case '@':
      case 'W':
      case 'w':
        break;
      case ';':
        if (s[i + 1] != 0)
          bad = 1;
        break;
      case '!':
      case '#':
      case '*':
      case '0':
      case '1':
      case '2':
      case '3':
      case '4':
      case '5':
      case '6':
      case '7':
      case '8':
      case '9':
        lastIdx = i;
        break;
      case 'A':
      case 'B':
      case 'C':
      case 'D':
      case 'a':
      case 'b':
      case 'c':
      case 'd':
        if (d->cfg.abcd != 0)
          bad = 1;
        else
          lastIdx = i;
        break;
      case '^':
        if (d->cfg.callingTone == 2)
          bad = 1;
        break;
      case 'T':
      case 't':
        if (mode == 0 && d->cfg.pulseAndTone != 0)
          bad = 1;
        break;
      case 'P':
      case 'p':
        if (mode == 1 && d->cfg.pulseAndTone != 0)
          bad = 1;
        break;
      default:
        bad = 1;
        break;
      }
    }
    if (bad)
    {
      if (d->cfg.modValid == 0)
      {
        if ((unsigned)res > 1u)
          res = 1;
      }
      else
      {
        if ((unsigned)res > 2u)
          res = 2;
      }
    }
  }

  if (record)
  {
    d->lastIdx = lastIdx;
    DBGP("AnalyzeDialString: LAST_DIALABLE_SYMBOL is %d\n", lastIdx);
  }
  return res;
}

/**
 * @brief Tell whether a dial string is unusable (FATAL or INVALID).  Nothing in the library calls it.
 * @param d  dialer (supplies the modem; its configuration is re-read)
 * @param s  dial string
 * @return 1 if the verdict is FATAL or INVALID, else 0
 */
int IsDialStringInvalid(Dialer* d, const char* s)
{
  int r;
  GetDialerConfig(&d->cfg, d->modem);
  DBGP("DIALER.C: IsDialStringInvalid(char * dialString )  got: %s\n", s);
  r = AnalyseDialString(d, s, 0);
  DBGP("Dial String Syntax is %s\n", (unsigned)r > 3u ? "ILLEGAL!" : syntaxStatName[r]);
  return (unsigned)r <= 1u;
}

/** @brief IsDialStringInvalid() on the Dialer embedded in a CallProg object.  Nothing in the library calls it. */
int Dialer_IsDialStringInvalid(void* callprog, const char* s) { return IsDialStringInvalid((Dialer*)((char*)callprog + CALLPROG_DIALER_OFF), s); }

/* ---------------------------------------------------------------------------------------------------------------- DialerCreate / DialerAbort */
/**
 * @brief Prepare a dialer for a dial string.
 * @param d           dialer to initialise (the caller owns it)
 * @param dialString  dial string, or NULL for none
 * @param m           host modem object
 * @return 0 on success, DIALER_ERROR_MSG if the string is FATAL or INVALID (nothing is copied; d->str keeps its old contents)
 *
 * Reads the configuration (GetDialerConfig()), clears the dial state, sets the pulse make and break times from it and copies the string.
 * @bug   AnalyseDialString accepts 100 characters but `str` holds 100 bytes including the NUL: a 100-character string writes its terminator over the low
 *        byte of cfg.toneDigitLen.  A digit length below 256 then becomes 0, and with a digit length of 0 a tone digit never completes (see DialerProgress()).
 * @note  With a NULL string `d->syntax` is not set.
 */
int DialerCreate(Dialer* d, const char* dialString, struct modem* m)
{
  const char* name;
  d->modem = m;
  GetDialerConfig(&d->cfg, m);
  d->state = DIALER_INITIAL_STATE;
  d->col = 0;
  d->cur = -1;
  d->row = 0;
  d->left = 0;
  d->digitSent = 0;

  SetPulseMakeTime(d->modem, (unsigned)d->cfg.pulseOffHookTime);
  SetPulseBreakTime(d->modem, (unsigned)d->cfg.pulseOnHookTime);
  d->pulseActive = 0;
  d->pulseDone = 0;

  /* analyse and copy the string */
  if (!dialString)
  {
    d->str[0] = 0;
    return 0;
  }
  d->syntax = AnalyseDialString(d, dialString, 1);
  name = (unsigned)d->syntax > 3u ? "ILLEGAL!" : syntaxStatName[d->syntax];
  DBGP("Dial String Syntax is %s\n", name);
  if ((unsigned)d->syntax <= 1u)
    return DIALER_ERROR_MSG;
  sysdep_strcpy(d->str, dialString); /* a 100 character string writes its NUL over the low byte of cfg.toneDigitLen (str is followed by cfg) */
  return 0;
}

/**
 * @brief Stop dialling: if a pulse digit was dialled and not yet closed, tell the host the pulse train is over.
 * @param d  dialer
 */
void DialerAbort(Dialer* d)
{
  if (d->state > 10u)
  {
    DBGP("Dialer was aborted - error. \n");
    return;
  }
  if (d->pulseDone == 0 && d->pulseActive != 0)
  {
    LastPulseDigitDialed(d->modem);
    DBGP(" **** Dialer.C: LastPulseDigitDialed was called\n");
    d->pulseDone = 1;
  }
  DBGP("Dialer was aborted.\n");
}

/* ---------------------------------------------------------------------------------------------------------------- GetNext  */
/*
 * Advance d->cur to the next interesting character and return the next DialerProgress state: DIGIT (d->row / d->col set; '#', '*' and A .. D only in tone
 * mode), FLASH '!', COMMA ',' (d->commaCount = the length of the run), WAIT_FOR_DIALTONE 'W' / 'w', WAIT_FOR_SILENCE '@', WAIT_FOR_BONGTONE '$',
 * CALLING_TONE '^' (only when cfg.callingTone is 1 or 3), END_PARTIALLY ';' (last character only), END at the end of the string.  Other characters are
 * skipped.  'T' / 'P' switch cfg.toneOrPulse (always at the first character; later only when the string may mix them).  It never returns SILENCE.
 */
static int GetNext(Dialer* d)
{
  d->commaCount = 1;
  for (;;)
  {
    int c, ch;
    d->cur = d->cur + 1;
    c = d->str[d->cur]; /* movzx; the signed value printed below is movsx */
    if (c == 0)
      return DIALER_END_STATE;
    DBGP("Digit is %c\n", (int)(signed char)c);
    ch = (int)(signed char)c - 0x20;
    if ((unsigned)ch > 0x57u)
      continue;
    switch (ch + 0x20)
    {
    case '!':
      return DIALER_FLASH_STATE;
    case '#':
      if (d->cfg.toneOrPulse == 1)
      {
        d->row = 3;
        d->col = 2;
        return DIALER_DIGIT_STATE;
      }
      continue;
    case '$':
      return DIALER_WAIT_FOR_BONGTONE_STATE;
    case '*':
      if (d->cfg.toneOrPulse == 1)
      {
        d->row = 3;
        d->col = 0;
        return DIALER_DIGIT_STATE;
      }
      continue;
    case ',':
      if (d->str[d->cur + 1] != ',')
        return DIALER_COMMA_STATE;
      d->commaCount = d->commaCount + 1;
      continue;
    case '0':
      d->row = 3;
      d->col = 1;
      return DIALER_DIGIT_STATE;
    case '1':
      d->row = 0;
      d->col = 0;
      return DIALER_DIGIT_STATE;
    case '2':
      d->row = 0;
      d->col = 1;
      return DIALER_DIGIT_STATE;
    case '3':
      d->row = 0;
      d->col = 2;
      return DIALER_DIGIT_STATE;
    case '4':
      d->row = 1;
      d->col = 0;
      return DIALER_DIGIT_STATE;
    case '5':
      d->row = 1;
      d->col = 1;
      return DIALER_DIGIT_STATE;
    case '6':
      d->row = 1;
      d->col = 2;
      return DIALER_DIGIT_STATE;
    case '7':
      d->row = 2;
      d->col = 0;
      return DIALER_DIGIT_STATE;
    case '8':
      d->row = 2;
      d->col = 1;
      return DIALER_DIGIT_STATE;
    case '9':
      d->row = 2;
      d->col = 2;
      return DIALER_DIGIT_STATE;
    case ';':
      if (d->str[d->cur + 1] != 0)
        continue;
      return DIALER_END_PARTIALLY_STATE;
    case '@':
      return DIALER_WAIT_FOR_SILENCE_STATE;
    case '^':
      if (d->cfg.callingTone == 1 || d->cfg.callingTone == 3)
        return DIALER_CALLING_TONE_STATE;
      continue;
    case 'W':
    case 'w':
      return DIALER_WAIT_FOR_DIALTONE_STATE;
    case 'A':
    case 'a':
      if (d->cfg.abcd != 0 || d->cfg.toneOrPulse != 1)
        continue;
      d->row = 0;
      d->col = 3;
      return DIALER_DIGIT_STATE;
    case 'B':
    case 'b':
      if (d->cfg.abcd != 0 || d->cfg.toneOrPulse != 1)
        continue;
      d->row = 1;
      d->col = 3;
      return DIALER_DIGIT_STATE;
    case 'C':
    case 'c':
      if (d->cfg.abcd != 0 || d->cfg.toneOrPulse != 1)
        continue;
      d->row = 2;
      d->col = 3;
      return DIALER_DIGIT_STATE;
    case 'D':
    case 'd':
      if (d->cfg.abcd != 0 || d->cfg.toneOrPulse != 1)
        continue;
      d->row = 3;
      d->col = 3;
      return DIALER_DIGIT_STATE;
    case 'P':
    case 'p':
      if (d->cur == 0)
      {
        d->cfg.toneOrPulse = 0;
        DBGP("Dialer.c: GetNextDigit... TONE_OR_PULSE_FLAG became PULSE_DIALING\n");
        continue;
      }
      if (d->cfg.toneOrPulse != 1)
        continue;
      if (d->cfg.pulseAndTone != 0)
      {
        DBGP("Dialer.c: GetNextDigit... Not permitted to switch to Pulse\n");
        continue;
      }
      d->cfg.toneOrPulse = 0;
      DBGP("Dialer.c: GetNextDigit... Switching to Pulse\n");
      continue;
    case 'T':
    case 't':
      if (d->cur == 0)
      {
        d->cfg.toneOrPulse = 1;
        DBGP("Dialer.c: GetNextDigit... TONE_OR_PULSE_FLAG became TONE_DIALING\n");
        continue;
      }
      if (d->cfg.toneOrPulse != 0)
        continue;
      if (d->cfg.pulseAndTone != 0)
      {
        DBGP("Dialer.c: GetNextDigit... Not permitted to switch to Tone\n");
        continue;
      }
      d->cfg.toneOrPulse = 1;
      DBGP("Dialer.c: GetNextDigit... Switching to Tone\n");
      continue;
    default:
      continue;
    }
  }
}

/* ---------------------------------------------------------------------------------------------------------------- DialerProgress */
NOIDIOM __attribute__((noinline)) static void fill_zero(short* buf, int* pos, int last)
{
  int i;
  for (i = *pos; i <= last; i++)
    buf[i] = 0;
  if (*pos <= last)
    *pos = i;
}

#include "core/dsp_mulw.h"

/* A DTMF digit starts: samples left, phases and phase increments (increment = frequency * 2.048 for the 14-bit phase at 8000 Hz). */
static void tone_setup(Dialer* d)
{
  int s;
  int lo, hi;
  s = (int)((unsigned)d->cfg.toneDigitLen << 3);
  d->left = s;
  DBGP("Samples left = %d\n", s);
  d->ph2 = 0x2000;
  lo = dial_freqs[5 + d->row];
  d->ph1 = 0;
  hi = dial_freqs[d->col];
  d->inc1 = (unsigned short)(mulw(lo, 0x4189) >> 13);
  d->inc2 = (unsigned short)(mulw(hi, 0x4189) >> 13);
}

/* A comma pause starts: the run of commas lasts dialPause seconds per comma, at most commaLimit seconds, 8000 samples per second. */
static void comma_setup(Dialer* d)
{
  int total = mulw(d->cfg.dialPause, d->commaCount);
  int lim = d->cfg.commaLimit;
  d->left = mulw(total <= lim ? total : lim, SAMPLE_RATE_8000);
  DBGP("Consequitive-Commas Dialing; Number of samples is %d.\n", d->left);
}

static int between_samples(Dialer* d)
{
  int x = d->cfg.toneOrPulse == 1 ? d->cfg.toneBetween : d->cfg.pulseBetween;
  return (int)((unsigned)x << 3);
}

/* Enter the state `r` that GetNext() returned: set up the sample count or the phases it needs. */
static void enter_state(Dialer* d, int r)
{
  if (r == DIALER_SILENCE_STATE)
  {
    d->left = between_samples(d);
  }
  else if (r > 2)
  {
    if (r == DIALER_FLASH_STATE)
      d->digitSent = 0;
    else if (r == DIALER_COMMA_STATE)
      comma_setup(d);
  }
  else if (r == DIALER_DIGIT_STATE)
  {
    if (d->cfg.toneOrPulse == 1)
      tone_setup(d);
    else
      d->digitSent = 0;
  }
  d->state = (unsigned)r;
}

/**
 * @brief Produce dialling signal.
 * @param d    dialer (DialerCreate())
 * @param buf  output samples (8000 Hz)
 * @param pos  in / out: index of the next free entry of `buf`
 * @param last index of the last entry to fill (inclusive)
 * @return a DialerResult: DIALER_DIALING_MSG when the buffer is full and dialling continues; DIALER_WAIT_DIALTONE_MSG, DIALER_WAIT_SILENCE_MSG,
 *         DIALER_WAIT_BONGTONE_MSG or DIALER_CALLING_TONE_MSG when the string asks for that event (the state returns to INITIAL, so the next call goes on
 *         with the following character); DIALER_END_PARTIALLY_MSG or DIALER_END_MSG at the end of the string (returned again by every later call);
 *         DIALER_ERROR_MSG for an invalid state
 *
 * Digits.  A tone digit is the sum of two cosines (rows: gain1, columns: gain2) for `toneDigitLen * 8` samples.  A pulse digit hands its pulse count
 * (from the keypad position and cfg.digitPattern 1 .. 3) to PulseDialDigit() and fills the buffer with zeros until IsPulseDialerReady() reports the train
 * complete.  After a digit comes silence of the between-digit time unless the digit was the last dialable symbol.  A comma is a pause of dialPause seconds
 * per comma (at most commaLimit).  A flash is a pulse of flashTime * 10 on the hook with make time 0.
 * @bug   A tone digit with cfg.toneDigitLen == 0 fills every buffer with tone and never completes, so dialling never ends.
 * @bug   A cfg.digitPattern outside 1 .. 3 leaves the pulse digit at -1, which PulseDialDigit() takes as about four billion pulses.
 * @warning `*pos` must not exceed `last + 1` on entry.  A cfg.toneOrPulse other than 0 or 1 (it is only set to those) would loop forever in the digit
 *          state.
 */
NOIDIOM int DialerProgress(Dialer* d, short* buf, int* pos, int last)
{
  int startPos, r;
top:
  startPos = *pos;
  if (startPos > last)
    return DIALER_DIALING_MSG;
  switch (d->state)
  {
  case DIALER_INITIAL_STATE:
    DBGP("DIALER_INITIAL_STATE\n");
    r = GetNext(d);
    enter_state(d, r);
    goto top;

  case DIALER_DIGIT_STATE:
  { /* one digit */
    int cur = startPos, end = last, flag = -1, pulseDigit = -1;
    if (d->cfg.toneOrPulse != 0)
    {
      if (d->cfg.toneOrPulse != 1)
        goto s1_end; /* never leaves (flag -1) */
      /* DTMF digit */
      int avail, left;
      if (d->pulseDone == 0 && d->pulseActive != 0)
      {
        LastPulseDigitDialed(d->modem);
        DBGP(" **** Dialer.C: LastPulseDigitDialed was called\n");
        d->pulseActive = 0;
        d->pulseDone = 1;
      }

      avail = last - startPos + 1;
      flag = 0;
      if (d->cfg.toneDigitLen != 0)
      {
        left = d->left;
        if (left > avail)
        {
          d->left = left - avail;
          flag = 0;
        }
        else
        {
          end = left + startPos - 1;
          d->left = 0;
          flag = 1;
        }
      }

      /* two cosines */
      while (cur <= end)
      {
        unsigned p1, p2;
        int g1, g2, t1, t2, v;
        p1 = ((unsigned)d->ph1 + (unsigned)d->inc1) & 0x3fffu;
        p2 = ((unsigned)d->ph2 + (unsigned)d->inc2) & 0x3fffu;
        d->ph1 = (unsigned short)p1;
        d->ph2 = (unsigned short)p2;
        g1 = d->cfg.gain1;
        t1 = TONE_read((int)((p1 + 4u) >> 3));
        v = mulw(g1, t1);
        g2 = d->cfg.gain2;
        t2 = TONE_read((int)(((int)(short)d->ph2 + 4) >> 3));
        buf[cur] = (short)((v >> 14) + (mulw(g2, t2) >> 14));
        cur++;
      }
      *pos = cur;
      goto s1_end;
    }

    /* pulse digit */
    if (d->digitSent == 0)
    {
      switch (d->cfg.digitPattern)
      {
      case 1:
        pulseDigit = d->row * 3 + d->col + 1;
        if (pulseDigit == 11)
          pulseDigit = 10;
        break;
      case 2:
      {
        int t = d->row * 3 + d->col;
        pulseDigit = t == 10 ? 1 : t + 2;
        break;
      }
      case 3:
        pulseDigit = d->row * 3 + d->col + 1;
        DBGP("Original digitToPulseDial %d", pulseDigit);
        if (pulseDigit == 11)
          pulseDigit = 10;
        else if (pulseDigit <= 9)
          pulseDigit = 10 - pulseDigit;
        break;
      default:
        break;
      }
      if (!IsPulseDialerReady(d->modem))
      {
        DBGP("=========> Waiting to PULSE_IS_DIALER_READY_PROC to become true.\n");
        fill_zero(buf, pos, last);
        goto s1_end;
      }
      DBGP("=========> Calling PULSE_DIAL_DIGIT_PROC with %d.\n", pulseDigit);
      PulseDialDigit(d->modem, (unsigned)pulseDigit);
      d->digitSent = 1;
    }
    if (!IsPulseDialerReady(d->modem))
    {
      fill_zero(buf, pos, last);
      flag = 0;
      goto s1_end;
    }
    DBGP("=========>END OF DIAL DIGIT DETECTED.\n");
    d->digitSent = 0;
    d->pulseDone = 0;
    d->pulseActive = 1;
    flag = 1;
  s1_end:
    if (flag != 1)
      goto top;

    /* the digit is complete */
    DBGP("Done Generating digit\n");
    if (d->cur == d->lastIdx)
    {
      r = GetNext(d);
      enter_state(d, r);
    }
    else
    {
      d->left = between_samples(d);
      d->state = DIALER_SILENCE_STATE;
    }
    goto top;
  }

  case DIALER_SILENCE_STATE: /* silence between digits */
  case DIALER_COMMA_STATE:
  { /* comma pause */
    int avail = last - startPos + 1;
    int left = d->left;
    int endi, i, partial;
    if (left > avail)
    {
      d->left = left - avail;
      endi = last;
      partial = 1;
    }
    else
    {
      endi = left + startPos - 1;
      d->left = 0;
      partial = 0;
    }
    for (i = startPos; i <= endi; i++)
      buf[i] = 0;
    *pos = i;
    if (partial)
      goto top;
    if (d->state == DIALER_SILENCE_STATE)
      DBGP("Done Generating silence between digits\n");
    else
      DBGP("Done Generating Dial-Pause\n");
    goto next_element;
  }

  case DIALER_FLASH_STATE: /* hook flash */
    if (d->digitSent == 0)
    {
      SetPulseMakeTime(d->modem, 0);
      SetPulseBreakTime(d->modem, (unsigned)mulw(d->cfg.flashTime, 10));
      if (!IsPulseDialerReady(d->modem))
      {
        DBGP("=========> Waiting to PULSE_IS_DIALER_READY_PROC to become true.\n");
        fill_zero(buf, pos, last);
        goto top;
      }
      DBGP("=========> Begin Dialing FLASH.\n");
      PulseDialDigit(d->modem, 1);
      d->digitSent = 1;
    }
    if (!IsPulseDialerReady(d->modem))
    {
      fill_zero(buf, pos, last);
      goto top;
    }
    DBGP("=========> End dialing Flash.\n");
    d->digitSent = 0;
    SetPulseMakeTime(d->modem, (unsigned)d->cfg.pulseOffHookTime);
    SetPulseBreakTime(d->modem, (unsigned)d->cfg.pulseOnHookTime);
    d->pulseDone = 0;
    d->pulseActive = 1;
    DBGP("Done Generating flash\n");
    d->left = (int)((unsigned)d->cfg.pulseBetween << 3);
    d->state = DIALER_SILENCE_STATE;
    goto top;

  case DIALER_WAIT_FOR_DIALTONE_STATE:
    DBGP("DIALER_WAIT_FOR_DIALTONE_STATE\n");
    d->state = DIALER_INITIAL_STATE;
    return DIALER_WAIT_DIALTONE_MSG;
  case DIALER_WAIT_FOR_SILENCE_STATE:
    DBGP("DIALER_WAIT_FOR_SILENCE_STATE\n");
    d->state = DIALER_INITIAL_STATE;
    return DIALER_WAIT_SILENCE_MSG;
  case DIALER_WAIT_FOR_BONGTONE_STATE:
    DBGP("DIALER_WAIT_FOR_BONGTONE_STATE\n");
    d->state = DIALER_INITIAL_STATE;
    return DIALER_WAIT_BONGTONE_MSG;
  case DIALER_CALLING_TONE_STATE:
    DBGP("DIALER_CALLING_TONE_STATE\n");
    d->state = DIALER_INITIAL_STATE;
    return DIALER_CALLING_TONE_MSG;
  case DIALER_END_PARTIALLY_STATE:
  case DIALER_END_STATE:
    if (d->pulseDone == 0 && d->pulseActive != 0)
    {
      LastPulseDigitDialed(d->modem);
      DBGP(" **** Dialer.C: LastPulseDigitDialed was called\n");
      d->pulseDone = 1;
    }
    if (d->state == DIALER_END_PARTIALLY_STATE)
    {
      DBGP("DIALER_END_PARTIALLY_STATE\n");
      return DIALER_END_PARTIALLY_MSG;
    }
    DBGP("DIALER_END_STATE\n");
    return DIALER_END_MSG;
  default:
    DBGP("Dialer switch default case\n");
    return DIALER_ERROR_MSG;
  }

next_element:
  r = GetNext(d);
  enter_state(d, r);
  goto top;
}

/* ---------------------------------------------------------------------------------------------------------------- calling tone */
/**
 * @brief Start the calling tone: tone on, 5760 samples (0.72 s) long, gain from `level`.
 * @param ct     generator state
 * @param level  level; only the low byte is used, as a signed char (S-register 0xdd)
 *
 * The gain is FP_Pow((level * 0x25ca7) >> 14), a Q14 factor around 1.0.
 */
void ResetCallingTone(CallingTone* ct, int level)
{
  int x = (int)((unsigned)(int)(signed char)level * 0x25ca7u);
  ct->phase = 0;
  ct->on = 1;
  ct->cnt = 0x1680;
  ct->gain = FP_Pow(x >> 14);
}

/**
 * @brief Generate `n` samples of the calling tone: 5760 samples of tone, 16800 samples of silence, repeating.
 * @param ct   generator state (ResetCallingTone())
 * @param out  output samples
 * @param n    number of samples; see the warning
 * @bug   The phase is converted to the 11-bit argument of TONE_read() with a shift of 6 instead of 3 (the DTMF path shifts by 3): the waveform covers only
 *        the first 45 degrees of the cosine, a sawtooth between about 0.7 and 1.0 of full scale at 1083 Hz with a large DC component, not a sine.
 * @bug   With a gain of 16384 or more (a level of 0 or above) the peak product reaches 32768 and wraps to -32768.
 * @warning Segment lengths are compared with the absolute buffer index, not the remaining count: from a fresh generator an n above 16800 never
 *          terminates (the tone segment ends at 5760, the silence segment at 16800, and the next tone segment is then too short to advance the index).
 */
NOIDIOM void GenerateCallingTone(CallingTone* ct, short* out, int n)
{
  int i = 0, on = ct->on, cnt = ct->cnt, gain = ct->gain;
  unsigned phase = ct->phase;
  while (i < n)
  {
    int m = cnt <= n ? cnt : n;
    int rest;
    if (on != 0)
    {
      for (; i < m; i++)
      {
        int t = TONE_read(((int)(short)phase + 0x20) >> 6);
        out[i] = (short)(mulw(gain, (short)t) >> 13);
        phase = (phase + 0x8abu) & 0x3fffu;
      }
    }
    else
    {
      for (; i < m; i++)
        out[i] = 0;
    }
    rest = (int)((unsigned)cnt - (unsigned)m);
    if (rest != 0)
    {
      cnt = rest;
      continue;
    }
    if (on == 0)
    {
      on = 1;
      cnt = 0x1680;
      continue;
    }
    on = 0;
    cnt = 0x41a0;
  }
  ct->phase = (unsigned short)phase;
  ct->on = on;
  ct->cnt = cnt;
}
