/**
 * @file fax_rx_common_tables.c
 * @brief Default configuration blocks of the fax receivers' symbol-rate estimator (FaxSreCfg) and equalizer (FaxFseCfg).
 *
 * The V.17, V.27ter and V.29 receivers copy these blocks into a local configuration and then fill in the coefficient-table handles and the per-rate fields
 * (fax_timing.c and fax_equalizer.c consume the result).  The table handles are left 0 here.
 */
#include "fax_rx_common.h"

/** Default timing-recovery configuration (as 32-bit words {0x10004, 0x300018, 0x40000555, 0x3c, 0 x 6, 0xa40852, 0xccd2000, 0x908cc, 0}); tables unset. */
const FaxSreCfg fax_sre_cfg_default = {
    .sps = 4, .periodsTrack = 1, .periodsLock = 0x18, .startCount = 0x30, .unlockGain = 0x555, .lockGain = 0x4000, .nt = 0x3c, .s0e = 0,
    .coefTab = 0, .tedCoef = 0, .corrCosTab = 0, .corrSinTab = 0, .propGainTab = 0, .intGainTab = 0,
    .errOn = 0x852, .errOff = 0xa4, .unlockThresh = 0x2000, .lockThresh = 0xccd, .levelThresh = 0x8cc, .levelLen = 9, .w34 = 0,
};
/** Default equalizer configuration (as 32-bit words {0x30090, 0 x 5, 0x10004, 0x199a0060, 0x666, 0 x 5}); tables, history and callback unset. */
const FaxFseCfg fax_fse_cfg_default = {
    .blockLen = 0x90, .sps = 3, .iCoefTab = 0, .qCoefTab = 0, .nt = 0, .mu = {0, 0, 0}, .carrierTab = 0, .crrLen = 4, .crrAdj = 1,
    .settleCount = 0x60, .errThreshHi = 0x199a, .errThreshLo = 0x666, .s22 = 0, .propTab = 0, .intTab = 0, .hist = NULL, .cb = NULL, .w34 = 0,
};
