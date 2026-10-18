/**
 * @file v34_hs_init_mode.h
 * @brief V34HsInitMode: the `mode` argument of v34handshakinit(obj, mode) (src/vpcm/v34_handshake_init.c).  A plain enum, includable from C and C++.
 *
 * v34handshakinit resets the handshake state for one of five situations; modes 2 and 3 share one code path, any other value takes the `default:` arm
 * (the comparison is unsigned).  What each mode stores:
 *   - V34HSINIT_START: (re)start of the whole handshake: tx state SILENCEINFO, rx state RX_DPSK, microstate DET_SYNC.  Used by VPcmV34Create and by the
 *     "repeated INFO0 is detected" error recoveries of RX_PHASE1_CALL / RX_PHASE2_CALL / TX_PHASE1_CALL, which restart the whole handshake.
 *   - V34HSINIT_RETRAIN: retrain: tx state SILENCERETRAIN, rx state WAIT.  Used by the retrain triggers of the receive states (rxflags & 0x40, the error
 *     counters), by the callers of V34SetINFO1aBits and by the retrain detector of datapumpv34 ("V34TIMING,Retrain started").
 *   - V34HSINIT_RNEG_LOCAL: rate renegotiation started by this side: tx state SSEG, rx state RECEIVE.  Used by the InitiateRateRenegotiation /
 *     InitiateHangUp wrappers and by the large / small error branches of datapumpv34 ("V34RNEG, rate renegotiation DOWN / UP initiated").
 *   - V34HSINIT_RNEG_REMOTE: rate renegotiation detected from the far side ("V34RNEG, rate renegotiation detected"): the same path as mode 2; only
 *     datapumpv34 passes it.
 *   - V34HSINIT_MOH: modem on hold: tx state SILENCERETRAIN, rx state WAIT, microstate MOH_TONE (VPcmV34InitMOH).
 *
 * The values and the states each one stores are read from the code and from the prints at the callers; the names START / RNEG_LOCAL / RNEG_REMOTE are
 * descriptive choices, not names from a specification.
 */
#ifndef V34_HS_INIT_MODE_H
#define V34_HS_INIT_MODE_H

/** Reset situations of v34handshakinit(). */
enum V34HsInitMode
{
  V34HSINIT_START = 0,       ///< (re)start of the whole handshake
  V34HSINIT_RETRAIN = 1,     ///< retrain
  V34HSINIT_RNEG_LOCAL = 2,  ///< rate renegotiation started by this side
  V34HSINIT_RNEG_REMOTE = 3, ///< rate renegotiation detected from the far side (same path as 2)
  V34HSINIT_MOH = 4          ///< modem on hold
};

#endif /* V34_HS_INIT_MODE_H */
