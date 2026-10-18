/**
 * @file v22_stages.h
 * @brief Names for the stages of the V.22 / V.22bis protocol handlers (the value of V22Tones::sc), and the protocol and status code enums.
 *
 * V22FP_modem() calls V22_PROTOCOL[tones.state](...) once per 160-sample block (state: 0 data, 1 originate, 2 answer, 3 local loop, 4 / 5 remote loop 2,
 * 6 retrain); the handler switches on tones.sc, its stage.  The numbers overlap between the handlers (stage 3 of the originate machine and stage 3 of
 * the answer machine have different jobs), so every handler has its own enum; stages 8 .. 13 are the tail the call-setup machines and the local loop
 * share (connect_2400 / connect_1200).  The value 0 means "the handler's first stage"; where an engine function stores sc = 0 only to say "no stage"
 * (the drop to the data state), the literal 0 stays.  The jump-table range guards (`sw > 13u`, `sw > 14u`, `sw > 7u`) stay literal, as do the unused
 * slots of the tables.
 *
 * The call-setup stages print their number at debug level 2 as "V22_originate,NODE_<n>", "V22_answer,NODE_<n>" and "connect_2400, NODE_2400B / 2400C /
 * 2400D" (the 14th answer stage as NODE_SILENCE_AFTER_2100); those tags are noted below.  The descriptive names come from what the code of the stage
 * does (medium confidence where flagged); the loop-back and retrain stages and the stages 8 (the first connect_2400 stage; "A" follows from B .. D),
 * 12 and 13 (connect_1200) print no tag.
 */
#ifndef DSPLIBS_PORT_V22_STAGES_H
#define DSPLIBS_PORT_V22_STAGES_H

/** Stages of v22_originate (V22_PROTOCOL[1]); 2 and 7 are unused table slots. */
enum V22OrgStage
{
  V22ORG_INIT = 0,               ///< NODE_0: clear the timers, silence, rates 1200, tone retuned to 1200 Hz
  V22ORG_WAIT_ANSWER_TONE = 1,   ///< NODE_1: Detect_v22, error 0x11 on the timeout
  V22ORG_MEASURE_LEVEL = 3,      ///< NODE_3: line level (FPM_rms average -> tones.shift) and Detect_1s accumulation
  V22ORG_DESCRAMBLE = 4,         ///< NODE_4: receive scrambled, 0x1a0 ms
  V22ORG_SEND_S1 = 5,            ///< NODE_5: send S1 (2400) or scrambled ones (1200), look for S1 / unscrambled ones; 0x218 ms
  V22ORG_SEND_SCRAMBLED_ONES = 6 ///< NODE_6: send scrambled ones, decide 2400 (-> V22CONN_2400_A) or 1200 (-> V22CONN_1200_A)
};

/** Stages of v22_answer (V22_PROTOCOL[2]); 2 and 5 .. 7 are unused table slots. */
enum V22AnsStage
{
  V22ANS_INIT = 0,               ///< NODE_0
  V22ANS_ANSWER_TONE = 1,        ///< NODE_1: the 2100 Hz answer tone (FPM_TONE_generate), 0xce4 ms
  V22ANS_UNSCRAMBLED_ONES = 3,   ///< NODE_3: send unscrambled ones (guard tone after 0x5be ms), look for S1 / ones
  V22ANS_SEND_S1 = 4,            ///< NODE_4: send S1 (pattern 0), 0x61 ms
  V22ANS_SILENCE_AFTER_2100 = 14 ///< NODE_SILENCE_AFTER_2100: four blocks of silence after the answer tone
};

/** Stages of connect_2400 / connect_1200, entered from v22_originate, v22_answer and v22_local_loop. */
enum V22ConnStage
{
  V22CONN_2400_A = 8,  ///< no tag (B .. D follow from it); medium confidence
  V22CONN_2400_B = 9,  ///< "NODE_2400B"
  V22CONN_2400_C = 10, ///< "NODE_2400C"
  V22CONN_2400_D = 11, ///< "NODE_2400D" (the two retry messages name C and D)
  V22CONN_1200_A = 12, ///< no tag; medium confidence
  V22CONN_1200_B = 13
};

/** Stages of v22_local_loop (V22_PROTOCOL[3]); 4 .. 7 are unused table slots. */
enum V22LoopStage
{
  V22LOOP_INIT = 0,
  V22LOOP_ANSWER_TONE = 1,   ///< 0xce4 ms of the answer tone
  V22LOOP_WAIT_TONE_END = 2, ///< silence until FPM_TONE_detect reports 2
  V22LOOP_DETECT_RATE = 3    ///< the S1 / ones detectors choose V22CONN_2400_A or V22CONN_1200_A
};

/** Stages of v22_org_rmloop2 (V22_PROTOCOL[4]): the originating side of the remote digital loopback. */
enum V22OrgRmloopStage
{
  V22ORML_START = 0,
  V22ORML_WAIT_ACK = 1, ///< send the pattern, accumulate Detect_Rmloop2_ACK
  V22ORML_SCRAMBLED = 2 ///< scrambled pattern, accumulate Detect_1s
};

/** Stages of v22_ans_rmloop2 (V22_PROTOCOL[5]): the answering side of the remote digital loopback. */
enum V22AnsRmloopStage
{
  V22ARML_START = 0,
  V22ARML_WAIT_ONES = 1, ///< scrambled pass-through, accumulate Detect_1s
  V22ARML_PATTERN = 2,   ///< pattern 3 / 4 until Detect_1s reports 0
  V22ARML_LOOPBACK = 3   ///< the received bits are modulated as the transmit data
};

/** Stages of v22_retrain (V22_PROTOCOL[6]); v22_data enters at V22RETRAIN_REMOTE_REQ (the remote modem asked) or V22RETRAIN_RESTART (a local retrain). */
enum V22RetrainStage
{
  V22RETRAIN_REMOTE_REQ = 0,      ///< send the data pattern, wait until S1 has been seen and is gone
  V22RETRAIN_RESTART = 1,         ///< clear, ResetRx, rates 1200, silence
  V22RETRAIN_SILENCE = 2,         ///< silence for 0x63 ms (S1 detections are accumulated)
  V22RETRAIN_SCRAMBLED_ONES = 3,  ///< scrambled ones, wait until S1 has been seen and is gone
  V22RETRAIN_ONES_TO_RX_2400 = 4, ///< scrambled ones, 0x1c1 ms, then the receive rate is switched
  V22RETRAIN_ONES_TO_TX_2400 = 5, ///< the same with descrambling, 0x257 ms, then the transmit rate is switched
  V22RETRAIN_DATA_2400 = 6,       ///< 2400 data pattern, 0x31f ms
  V22RETRAIN_TRAINING_2400 = 7    ///< 2400 data pattern, RxTrained2400
};

/** V22Tones::state: the index into V22_PROTOCOL (src/v22/v22_engine.c) that V22FP_modem() calls, and into v22_status_map. */
enum V22Protocol
{
  V22PROTO_DATA = 0,        ///< v22_data
  V22PROTO_ORIGINATE = 1,   ///< v22_originate
  V22PROTO_ANSWER = 2,      ///< v22_answer
  V22PROTO_LOCAL_LOOP = 3,  ///< v22_local_loop
  V22PROTO_ORG_RMLOOP2 = 4, ///< v22_org_rmloop2
  V22PROTO_ANS_RMLOOP2 = 5, ///< v22_ans_rmloop2
  V22PROTO_RETRAIN = 6      ///< v22_retrain
};

/**
 * V22Engine::state, the status byte V22FP_modem() returns (and dp_v22.c switches on: 0, 1, 3 and 4 are understood, every other value is a DPSTAT_ERROR
 * there).  Names in the V22_MSG_ spelling are those of the debug strings the engine prints where it stores the value: "V22_MSG_ERROR1 / 3 / 4 / 5 / 7"
 * where it stores 0x11 / 0x13 / 0x14 / 0x15 / 0x17, "V22_MSG_CONNECT_2400" where it stores 3, "V22_MSG_NO_CARRIER*" (suffix 2, 3, 4 = the call site)
 * where it stores 0x10.  V22_MSG_CONNECT_1200 (4) is confirmed by dp_v22.c (case 4 sets the 1200 bit/s rates); V22_MSG_ERROR6 (0x16, local-loop timeout)
 * and V22_MSG_ERROR8 (0x18, connect_1200 timeout) follow the numbering; the loop-back and retrain codes 2 and 5 .. 0xb print no such string: medium
 * confidence, named from the code that stores them.
 */
enum V22Msg
{
  V22_MSG_RUNNING = 0,   ///< data state, nothing to report
  V22_MSG_HANDSHAKE = 1, ///< a handshake state is running (V22FP_create also stores it)
  V22_MSG_RETRAIN = 2,   ///< v22_retrain is running
  V22_MSG_CONNECT_2400 = 3,
  V22_MSG_CONNECT_1200 = 4,
  V22_MSG_ANS_LOOP_UP = 5,        ///< v22_ans_rmloop2: the loop-back pattern was seen, looping the received bits back (medium confidence)
  V22_MSG_ORG_LOOP_UP = 6,        ///< v22_org_rmloop2: the remote loop-back is established (medium confidence)
  V22_MSG_LOOP_TIMEOUT = 7,       ///< the loop-back handshake timed out (medium confidence)
  V22_MSG_ANS_LOOP_LOST = 8,      ///< v22_ans_rmloop2: carrier lost while looping back (medium confidence)
  V22_MSG_REMOTE_RETRAIN = 9,     ///< v22_data: the remote modem requested a retrain (medium confidence)
  V22_MSG_ANS_LOOP_RUNNING = 0xa, ///< v22_ans_rmloop2 is running (medium confidence)
  V22_MSG_LOCAL_RETRAIN = 0xb,    ///< a local retrain was initiated (poor signal quality / carrier back) (medium confidence)
  V22_MSG_NO_CARRIER = 0x10,
  V22_MSG_ERROR1 = 0x11,
  V22_MSG_ERROR3 = 0x13,
  V22_MSG_ERROR4 = 0x14,
  V22_MSG_ERROR5 = 0x15,
  V22_MSG_ERROR6 = 0x16,
  V22_MSG_ERROR7 = 0x17,
  V22_MSG_ERROR8 = 0x18
};

#endif
