/**
 * @file v8_phase.h
 * @brief Names for the three dispatch words of the V.8 handshake: the TX phase and RX phase (V8Obj::phasePair, two halves) and the RX sub-state
 *        (V8Obj::rxSubState).
 *
 * v8handshak() dispatches on the TX phase (the low half of the phase pair), the RX phase (the high half) and, under the RX phases V8RX_ANSAM and
 * V8RX_MESSAGE, the RX sub-state.  v8handshakinit(), v8handshak() and V8Control() set them; V8Process() maps them to the V8Status the glue sees.  The
 * names come from the status names of v8_status.h where V8Process() maps a value to a status (the comment on the enumerator says which), otherwise
 * from what the code does; "medium" marks a name that rests on the behaviour alone.
 *
 * The three fields are different quantities that share numbers: 0x23 is both an RX phase (V8RX_DRAIN) and an RX sub-state (V8SUB_SEND_CJ), 5 is both
 * the TX phase V8TX_SILENCE and the RX phase V8RX_TIMEOUT_CJ, 0x19 is both an RX phase and an RX sub-state.  The enums are therefore separate on
 * purpose; a site may only use the enum of the field it reads or writes.  Range guards on these fields (`(unsigned short)(subState - 0x32) > 1`)
 * and the packed compare of the two halves as one int (`0x190005` = RX phase 0x19 << 16 | TX phase 5, V8Process) stay literal.
 */
#ifndef V8_PHASE_H
#define V8_PHASE_H

/* TX phase: what v8handshak() sends (4 samples per round, rounds until the transmit ring holds 16 samples). */
enum V8TxPhase
{
  V8TX_SILENCE = 5,        /* zeros.  Where the caller starts and where every finished handshake leaves it */
  V8TX_ANSAM = 6,          /* answerer: ANSam (2100 Hz, 15 Hz AM, phase reversal every 450 ms) while it waits for CM: V8_ANS_SEND_ANSAM / V8_ANS_CM_DETECTED */
  V8TX_MESSAGE = 0x17,     /* V.21 FSK of the message at V8Obj::txMsgPtr: V8_ANS_SEND_JM, V8_ORG_SEND_CM, V8_ORG_SEND_CJ */
  V8TX_QC_PREAMBLE = 0x2b, /* 60 bit periods of the quick-connect preamble, then V8TX_MESSAGE: V8_ORG_SEND_QC */
  V8TX_TONEQ = 0x2d        /* plain 980 Hz tone (v8_TONEq_generate), after a good QCA1a with quick connect enabled (medium: no status maps it) */
};

/* RX phase: what v8handshak() does with the received samples. */
enum V8RxPhase
{
  V8RX_TIMEOUT_CM = 4, /* answerer gave up waiting for CM: V8_ANS_TIME_OUT_WAITING_FOR_CM; dispatches to nothing */
  V8RX_TIMEOUT_CJ =
      5, /* answerer gave up waiting for CJ: V8_ANS_TIME_OUT_WAITING_FOR_CJ; never reaches the RX dispatch (tx phase V8TX_MESSAGE returns first) */
  V8RX_TIMEOUT_ANSAM = 0xb, /* caller gave up waiting for ANSam: V8_ORG_TIME_OUT_WAITING_FOR_ANSAM; dispatches to nothing */
  V8RX_TIMEOUT_JM = 0xc,    /* caller gave up waiting for JM: V8_ORG_TIME_OUT_WAITING_FOR_JM; never reaches the RX dispatch */
  V8RX_ANSAM = 0x19,      /* caller: AGC, ANSam detection (V8SUB_ANSAM_WAIT), then phase-reversal / CI analysis (V8SUB_ANSAM_SEEN); entered again after QCA1a */
  V8RX_AGC_SETTLE = 0x20, /* answerer: 601 AGC calls, then V8RX_MESSAGE with V8SUB_SYNC */
  V8RX_DRAIN = 0x23,      /* only reads the RX ring (v8_rxreadqueue); nothing in this tree sets it (medium) */
  V8RX_MESSAGE = 0x28,    /* V.21 FSK demodulation of CM / JM / CJ / QCA1: the sub-state says which step */
  V8RX_DONE = 99          /* handshake finished: v8handshak returns 2, V8Process reports V8_OK */
};

/* RX sub-state (V8Obj::rxSubState): valid under V8RX_ANSAM (the first two) and V8RX_MESSAGE (the rest). */
enum V8RxSub
{
  V8SUB_ANSAM_WAIT = 0x19, /* caller: listening for ANSam: V8_ORG_WAITING_FOR_ANSAM (with TX silence, RX phase V8RX_ANSAM) */
  V8SUB_SEND_CJ =
      0x23, /* caller: the CM is finished, CJ is being sent (V8Obj.txMsgPtr = msg[1]); handshake done 0x104 calls later.  V8CTRL_START_CJ enters it */
  V8SUB_ANSAM_SEEN = 0x24, /* caller: ANSam detected, watching it for ~2400 calls before deciding CM / CI: V8_ORG_ANSAM_DETECTED_WAITING_TE */
  V8SUB_COLLECT =
      0x28, /* a sync word was found: collecting the 10-bit words of CM (answerer) or JM (caller) until two successive passes of the repeated message agree */
  V8SUB_SYNC = 0x29,        /* searching the bit stream for the sync word 0xc0f (CM / JM) or 0xd55 (QCA1) */
  V8SUB_WAIT_CJ = 0x2a,     /* answerer: JM is being sent, waiting for CJ (two runs of nine 0 bits end the handshake) (medium) */
  V8SUB_QCA1 = 0x2c,        /* quick connect: collecting and checking the QCA1 words */
  V8SUB_JM_DETECTED = 0x32, /* caller with opMode 1: JM received, waits for V8CTRL_START_CJ: V8_ORG_JM_DETECTED */
  V8SUB_CM_DETECTED = 0x33  /* answerer with opMode 1: CM received, waits for V8CTRL_START_JM: V8_ANS_CM_DETECTED */
};

#endif /* V8_PHASE_H */
