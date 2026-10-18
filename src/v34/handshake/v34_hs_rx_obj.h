/**
 * @file v34_hs_rx_obj.h
 * @brief rxObj(): the receiver sub-object of the V.34 object, for the receive-side DSP helpers.
 *
 * Included by v34_hs_post_loop.cpp and v34_hs_rx_infodone_moh.cpp.  Plain `static` (not inline): include it only in a file that calls rxObj().
 */
#ifndef V34_HS_RX_OBJ_H
#define V34_HS_RX_OBJ_H

#include "v34_handshake.h"

/**
 * @brief Return the receiver sub-object of `obj`.
 * @param obj  V.34 object
 * @return the receive sub-object `obj->rx`, passed to rxreadqueue() and tone_detect()
 */
static V34Rx* rxObj(tagV34Object* obj) { return &obj->rx; }

#endif /* V34_HS_RX_OBJ_H */
