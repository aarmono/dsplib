/**
 * @file v34_role.h
 * @brief V34Role: the values of tagV34Object::modeChar_ (the 16-bit word at offset 0x359c of the V.34 object).  A plain enum, includable from C and C++.
 *
 * The word is stored once, by VPcmV34Create (`modeChar_ = answer ? 0x66 : 0x65`), and read by every part of the handshake that depends on which side of the
 * call this V.34 module plays:
 *   - V34ROLE_CALL (0x65): the calling modem's role.  Its transmit scrambler is scrambleGPC (generator polynomial GPC, taps at bits 13 and 8 of the
 *     register: v04Scramble's `tap13`) and its descrambler descrambleGPA (v34_init.c: "== 0x65: scrambler = scrambleGPC, descrambler = descrambleGPA");
 *     v04_tx_silence moves to the microstate RX_PHASE1_CALL for it (TX_PHASE1_ANS for the other); it listens for the 2400 Hz answer tone
 *     (detectorinit(c2400_)) and transmits the INFO sequences as DPSK on 1200 Hz (v34modeminit stores bpv22high, carrier 1200).
 *   - V34ROLE_ANSWER (0x66): the answering modem's role (scrambleGPA / descrambleGPC, c1200_, bpv22low, carrier 2400).
 *
 * The second argument of VPcmV34Create (dp_vpcm.cpp passes `caller == 0`) is therefore the ANSWER flag.  For a V.90 session (VPcmV34Create mode 1) the flag
 * is inverted before the word is stored, so a V.90 module acts with the opposite V.34 role to the call direction (VPcmV34Progress: mode V90 transmits the
 * packed bits when modeChar_ is ANSWER, mode K56Flex when it is CALL); the names describe the handshake role, not the telephone call.  The constants are
 * the ASCII letters 'e' (0x65) and 'f' (0x66); the enum and constant names are descriptive choices.
 */
#ifndef V34_ROLE_H
#define V34_ROLE_H

/** Handshake role of the V.34 module. */
enum V34Role
{
  V34ROLE_CALL = 0x65,  ///< calling side
  V34ROLE_ANSWER = 0x66 ///< answering side
};

#endif /* V34_ROLE_H */
