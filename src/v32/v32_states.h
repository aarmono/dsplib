/**
 * @file v32_states.h
 * @brief Names for the small enumerated quantities of the V.32 / V.32bis engine: handshake state, handshake role, rate class and constellation mode.
 *
 * - V32State: the handshake state (V32Hdx::subState).  The letters are the state labels of the handshake of ITU-T V.32 (A, B, C ...); the numeric values are
 *   the position in the 35-entry order A, B, B2, C, D, D2, E .. Z, END, F2, X2, CLEARDOWN, DONE, ERROR, DONT_CARE.
 *   The V32?NextState functions (V32OrgNextState, V32AnsNextState, V32RngInitNextState, V32RngRespNextState) switch on the value.  Where the code gives a state
 *   a clear job the comment on the enumerator says so (inferred from the code).
 * - V32Role: the handshake role (V32Hdx::role), the index into the dispatch tables of src/v32/v32_tables.c and v32_engine.c: V32NextState[0] = V32OrgNextState,
 *   [1] = V32AnsNextState, [2] = [3] = V32LocLoopNextState (slot 3 is a duplicate that nothing selects), [4] = V32RngInitNextState, [5] = V32RngRespNextState;
 *   V32_PROTOCOL[6] is v32_data, [3], [7], [8] are v32_null_protocol, the rest v32_handshake.  V32FP_recreate sets 0 / 1 / 2 from the role selector
 *   (V32Wrap::protoSel: 0 originate, 1 answer, anything else local loop); the engine sets 4 (initiator of a rate renegotiation, V32FP_control), 5 (responder)
 *   and 6 (connected data) later.  Roles 7 and 8 exist in the tables only.
 * - V32RateClass: GetRateV32 maps the configured symbol rate to 0 .. 5 (14400 -> 5, 12000 -> 4, 9600 -> 2 or 1, 7200 -> 3, 4800 -> 0); the rate-sequence
 *   helpers (CodeRateSeq .. DecodeRateSeq) intersect the far end's capability word with the local V32_RATE_SEQ word and return the best class, or 6
 *   for "none in common".  V32_TX_MODE / V32_RX_MODE (v32_tables.c) turn a class into a mode as class + 1 (mode 7 for class 6).  The two 9600 classes are
 *   told apart by V32Wrap::highRate (set by V32FP_create); the enumerator comments name them by the constellation SetTxModeV32 / SetRxModeV32 install
 *   for the matching mode.
 *   Range guards on these fields (`idx > 5`) stay literal in the code.
 * - V32Mode: the constellation mode, the argument of SetTxModeV32 / SetRxModeV32.
 */
#ifndef DSPLIBS_PORT_V32_STATES_H
#define DSPLIBS_PORT_V32_STATES_H

/** Handshake state, V32Hdx::subState; the value is the position in the order A, B, B2, C, D, D2, E .. Z, END, F2, X2, CLEARDOWN, DONE, ERROR, DONT_CARE. */
enum V32State
{
  V32ST_A = 0,
  V32ST_B = 1,
  V32ST_B2 = 2,
  V32ST_C = 3,
  V32ST_D = 4,
  V32ST_D2 = 5,
  V32ST_E = 6,
  V32ST_F = 7,
  V32ST_G = 8,
  V32ST_H = 9,
  V32ST_I = 10,
  V32ST_J = 11,
  V32ST_K = 12,
  V32ST_L = 13,
  V32ST_M = 14,
  V32ST_N = 15,
  V32ST_O = 16,
  V32ST_P = 17,
  V32ST_Q = 18,
  V32ST_R = 19,
  V32ST_S = 20,
  V32ST_T = 21,
  V32ST_U = 22,
  V32ST_V = 23,
  V32ST_W = 24,
  V32ST_X = 25,
  V32ST_Y = 26,
  V32ST_Z = 27,
  V32ST_END = 0x1c,
  V32ST_F2 = 0x1d, ///< the "local retrain" state: the originate state machine's loop-back from it goes to state 8 (G)
  V32ST_X2 = 0x1e,
  V32ST_CLEARDOWN = 0x1f,
  V32ST_DONE = 0x20,
  V32ST_ERROR = 0x21,    ///< every receive handler's budget-exceeded transition sets it (with status 0x12 / 0x13)
  V32ST_DONT_CARE = 0x22 ///< set when the engine enters the connected data phase (role 6)
};

/** Handshake role, V32Hdx::role. */
enum V32Role
{
  V32ROLE_ORG = 0,         ///< originate: V32OrgNextState
  V32ROLE_ANS = 1,         ///< answer: V32AnsNextState
  V32ROLE_LOCLOOP = 2,     ///< local loop: V32LocLoopNextState
  V32ROLE_LOCLOOP_DUP = 3, ///< duplicate table slot of the local loop; nothing selects it
  V32ROLE_RNG_INIT = 4,    ///< rate renegotiation, initiator side: V32RngInitNextState
  V32ROLE_RNG_RESP = 5,    ///< rate renegotiation, responder side: V32RngRespNextState
  V32ROLE_DATA = 6         ///< connected: V32_PROTOCOL[6] = v32_data, no NextState
};

/** Rate class: V32Eng::txRateClass / rxRateClass / txConnClass / rxConnClass; returned by GetRateV32 and CodeRateSeq .. DecodeRateSeq. */
enum V32RateClass
{
  V32RC_4800 = 0,
  V32RC_9600 = 1,     ///< 9600 bit/s with V32Wrap::highRate == 0: mode 2, the 16-point constellation without trellis
  V32RC_9600_TCM = 2, ///< 9600 bit/s with highRate != 0: mode 3, the 32-point trellis-coded constellation
  V32RC_7200 = 3,
  V32RC_12000 = 4,
  V32RC_14400 = 5,
  V32RC_NONE = 6 ///< no rate in common ("incompatible")
};

/** Constellation mode, the argument of SetTxModeV32 / SetRxModeV32; for 1 .. 6 it is the rate class + 1 (V32_TX_MODE / V32_RX_MODE). */
enum V32Mode
{
  V32MODE_HANDSHAKE = 0, ///< transmit: the absolute-phase encoder (SMCv32_encoder_abs); receive: the AB -> CD -> trn -> 4pt acquisition chain
  V32MODE_4800 = 1,      ///< transmit: the differential encoder; receive: FSE_decision_4pt
  V32MODE_9600 = 2,
  V32MODE_9600_TCM = 3,
  V32MODE_7200 = 4,
  V32MODE_12000 = 5,
  V32MODE_14400 = 6
  /* 7 (class 6, none) takes the default branch and only flags an error */
};

#endif /* DSPLIBS_PORT_V32_STATES_H */
