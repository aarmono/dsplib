/**
 * @file fax_class1_tables.c
 * @brief Constant tables of the fax class 1 layer (fax_class1.c): debug-name tables and the input high-pass filter.
 *
 * The name tables map a status, state or command number to the text fax_class1.c prints at debug level above 1; the lookup (name_of()) scans the whole
 * table and the last matching entry wins.  fax_hp_coeff is the coefficient set of the high-pass filter on the class 1 input.
 */
#include "fax_class1.h"

/** Names of the class 1 status codes fax_class1_progress() prints: 11 entries. */
const FaxClass1Name fax_class1_status_names[11] = {
    {0, "FAX_CLASS1_NO_MESSAGE"},
    {1, "FAX_CLASS1_OK"},
    {2, "FAX_CLASS1_ERROR"},
    {3, "FAX_CLASS1_OK_NO_CARRIER"},
    {4, "FAX_CLASS1_ERROR_NO_CARRIER"},
    {5, "FAX_CLASS1_ERROR_ON_HOOK"},
    {6, "FAX_CLASS1_CONNECT"},
    {7, "FAX_CLASS1_NO_CARRIER"},
    {8, "FAX_CLASS1_NO_CARRIER_NO_MESSAGE"},
    {9, "FAX_CLASS1_OTHER_CARRIER"},
    {10, "FAX_CLASS1_ACCEPT_RATE"},
};

/** Names of the class 1 states: 20 entries (states 0 .. 18, then MAX_STATES). */
const FaxClass1Name fax_class1_state_names[20] = {
    {0, "T30_SILENCE_BEFORE_PREAMBLE_STATE"},
    {1, "T30_PREAMBLE_STATE"},
    {2, "SEND_HDLC_BUFFER_STATE"},
    {3, "SEND_HDLC_BETWEEN_BUFFER_STATE"},
    {4, "HDLC_RECEIVE_LOOK_CARRIER_STATE"},
    {5, "HDLC_RECEIVE_STATE"},
    {6, "HDLC_RECEIVE_BETWEEN_BUFFERS_STATE"},
    {7, "HDLC_EMULATE_RECEIVE_STATE"},
    {8, "IDLE_STATE"},
    {9, "TX_SCRAMBLED_ONES_STATE"},
    {10, "TX_DATA_STATE"},
    {11, "TX_NULLS_STATE"},
    {12, "RX_LOOK_CARRIER"},
    {13, "RX_DATA_STATE"},
    {14, "ANSWER_TONE_STATE"},
    {15, "SEND_SILENCE_STATE"},
    {16, "RECIEVE_SILENCE_STATE"},
    {17, "CHDLCTX_OFF_STATE"},
    {18, "TX_SILENCE_BEFORE_SCRM_ONES"},
    {19, "MAX_STATES"},
};

/** Names of the class 1 commands of fax_class1_command(): 6 entries (0 TH, 1 TM, 2 RM, 3 RH, 4 TS, 5 RS). */
const FaxClass1Name fax_class1_cmd_names[6] = {
    {0, "FAX_CLASS1_TH_COMMAND"},
    {1, "FAX_CLASS1_TM_COMMAND"},
    {2, "FAX_CLASS1_RM_COMMAND"},
    {3, "FAX_CLASS1_RH_COMMAND"},
    {4, "FAX_CLASS1_TS_COMMAND"},
    {5, "FAX_CLASS1_RS_COMMAND"},
};

/** High-pass filter of the class 1 input: two biquad sections of five coefficients for FPM_iir_filt_II(). */
const short fax_hp_coeff[10] = {14784, 14784, -29569, 12246, -28278, 14784, 14784, -29569, 14533, -30537};
