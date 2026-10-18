/**
 * @file v34_hs_tables.c
 * @brief Constant table of the V.34 handshake state machine: the L1 probe signal.
 *
 * `probe` is one period (64 samples, 16-bit signed, peak magnitude 13317) of the line-probing signal that the transmit state TX_L1 sends:
 * v04_tx_txl1 (v34_hs_tx_mid.cpp) reads `probe[k56Idx_ & 0x3f]` once per output sample.  The second half mirrors the first:
 * probe[32] == -probe[0] and probe[32 + k] == probe[32 - k] for k = 1..31.  The table is also read by the PCM modem code (src/vpcm/vpcm_flo_modem.cpp).
 */
#include <stdint.h>
#include "v34_hs_tables.h"

const short probe[64] = {
  13027, 13097, 7026, -8421, -2867, 3724, -12568, 251,
  4448, -9900, 7173, -752, -11389, 9409, 9436, 10147,
  5211, -9736, 2748, 11799, 9862, -547, -12548, 1980,
  763, -13317, 3205, 10584, -6029, -11058, -4471, -7261,
  -13027, -7261, -4471, -11058, -6029, 10584, 3205, -13317,
  763, 1980, -12548, -547, 9862, 11799, 2748, -9736,
  5211, 10147, 9436, 9409, -11389, -752, 7173, -9900,
  4448, 251, -12568, 3724, -2867, -8421, 7026, 13097,
};
