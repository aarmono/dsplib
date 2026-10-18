/**
 * @file v34_run_mode.h
 * @brief V34RunMode: the values of tagV34Object::runMode_ (the 32-bit word at offset 0x2218 of the V.34 object).  A plain enum, includable from C and C++.
 *
 * How the V.34 module runs the current block:
 *   - V34RUN_HANDSHAKE (0): the initial handshake; v34handshakinit stores 0 in its common head, VPcmV34Create clears the object.
 *   - V34RUN_DATA (1): data mode; v04_tx_dataxmit latches the rates ("V34DATA, getting into data mode from Handshake") and stores 1.
 *   - V34RUN_RETRAIN (2): a retrain this side started; VPcmV34InitiateRetrain and VPcmV34InitMOH store 2, and so does datapumpv34 when the error run
 *     retrainCnt_ passed baud / 2.
 *   - V34RUN_RETRAIN_REQ (3): a retrain the far side asked for; datapumpv34 stores 3 when rxFlags_ bit 0x40 (retrain request) is set and retrainCnt_ is
 *     not past baud / 2 (it computes `(retrainCnt_ <= baud / 2) + 2`).
 *   - V34RUN_RRN_REMOTE (4): rate renegotiation detected from the far side ("V34RNEG, rate renegotiation detected"; datapumpv34).
 *   - V34RUN_RRN_LOCAL (5): rate renegotiation started by this side (datapumpv34 DOWN / UP, VPcmV34InitiateRateRenegotiation, VPcmV34InitiateHangUp).
 *
 * Consumers: datapumpv34 runs the handshake path (v34handshak) when the value is above 1 and modulatevector otherwise; the status tail of v34handshak maps
 * 1 -> status 4, 4 / 5 -> 6 and 2 / 3 -> 7 (or 0 once k56Idx_ > 0x3c) in SILENCERETRAIN; TRNSEG4A treats values up to 3 as "not a renegotiation".  The
 * guards (`> 1`, `<= 3`, `w - 4 <= 1`, `w - 2 <= 1`) stay literal in the code.  The values are read from the stores and the consumers; the names are
 * descriptive choices (the distinction between 2 and 3 especially).
 */
#ifndef V34_RUN_MODE_H
#define V34_RUN_MODE_H

/** Run mode of the V.34 module. */
enum V34RunMode
{
  V34RUN_HANDSHAKE = 0,   ///< initial handshake
  V34RUN_DATA = 1,        ///< data mode
  V34RUN_RETRAIN = 2,     ///< retrain started by this side
  V34RUN_RETRAIN_REQ = 3, ///< retrain requested by the far side
  V34RUN_RRN_REMOTE = 4,  ///< rate renegotiation detected from the far side
  V34RUN_RRN_LOCAL = 5    ///< rate renegotiation started by this side
};

#endif /* V34_RUN_MODE_H */
