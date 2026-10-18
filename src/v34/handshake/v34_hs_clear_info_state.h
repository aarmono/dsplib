/**
 * @file v34_hs_clear_info_state.h
 * @brief clearInfoState(): reset of the INFO-exchange state words shared by the phase 1 / phase 2 handshake files.
 *
 * Included by v34_hs_rx_phase1_call.cpp, v34_hs_rx_phase2_call.cpp, v34_hs_tx_phase1_answer.cpp and v34_hs_tx_phase1_call.cpp.  Plain `static` (not
 * inline) so that an including file that does not call it would get an unused-function warning: include it only where it is called.
 */
#ifndef V34_HS_CLEAR_INFO_STATE_H
#define V34_HS_CLEAR_INFO_STATE_H

#include "v34_handshake.h"

/**
 * @brief Clear the INFO-exchange bookkeeping of the handshake object.
 * @param obj  V.34 object; `shortPhase2_`, `wabca_`, `wAbae_[0..9]` and `wAbc2_` are zeroed
 */
static void clearInfoState(tagV34Object* obj)
{
  int i;
  obj->shortPhase2_ = 0;
  obj->wabca_ = 0;
  i = 0;
  do
  {
    obj->wAbae_[i] = 0;
    i = (int16_t)(i + 1);
  } while (i <= 9);
  obj->wAbc2_ = 0;
}

#endif /* V34_HS_CLEAR_INFO_STATE_H */
