/**
 * @file fax_modem_wrappers.c
 * @brief Entry points that run the V.17 / V.27ter / V.29 scramblers and encoder-mode selectors on a modem object.
 *
 * The functions take the fax modem object as an opaque `void *`, find the sub-object they work on through the accessors of fax_modem_views.h, and call
 * or poke it.  The scramble / descramble entry points are thin forwards to SDM_scrambler() / SDM_descrambler() (V.17, V.29) and SDMv27_scrambler() /
 * SDMv27_descrambler() (V.27ter); the transmit and receive modem files (fax_v17_tx.c, fax_v27_rx.c, ...) call them for every block of bits.
 *
 * - ScrambleDataV17 / V27 / V29(obj, buf, n) and DescrambleDataV17 / V27 / V29(obj, buf, n): scramble / descramble `n` bit groups in `buf` with the
 *   modem's transmit / receive scrambler.
 * - SeedScramblerV17 / V29(obj, seed): set the transmit scrambler register (`sdm.state`) to `seed`.
 * - SetEncoderV17(obj, which, val): select the V.17 transmit encoder (`which` is one of V17ENC_ABS, V17ENC_TCM, V17ENC_DIFF; any other value does
 *   nothing).  TCM and DIFF also set the symbol-mapper state `smc.x` to `val`.
 * - SetEncoderV29(obj, which): `which` 0 or 1 sets the V.29 encoder select to that value; anything else does nothing.
 * - SetScramblerV27(obj): re-initialise the V.27ter transmit scrambler for the current rate (bit count V27TX_SDM_NUM_BITS[rate]) while keeping its
 *   register contents.
 */
#include "fax_common_tables.h"
#include "fax_modem_views.h"
#include "fax_v17_tx.h" /* V17ENC_* */

void ScrambleDataV17(void* obj, unsigned short* buf, unsigned short n) { SDM_scrambler(faxw_v17_txsdm(obj), buf, n); }
void ScrambleDataV27(void* obj, unsigned short* buf, short n) { SDMv27_scrambler(faxw_v27_txsdm(obj), buf, n); }
void ScrambleDataV29(void* obj, unsigned short* buf, unsigned short n) { SDM_scrambler(faxw_v29_txsdm(obj), buf, n); }
void DescrambleDataV17(void* obj, unsigned short* buf, unsigned short n) { SDM_descrambler(faxw_v17_rxsdm(obj), buf, n); }
void DescrambleDataV27(void* obj, unsigned short* buf, short n) { SDMv27_descrambler(faxw_v27_rxsdm(obj), buf, n); }
void DescrambleDataV29(void* obj, unsigned short* buf, unsigned short n) { SDM_descrambler(faxw_v29_rxsdm(obj), buf, n); }
void SeedScramblerV17(void* obj, unsigned int seed) { faxw_v17_txsdm(obj)->state = seed; }
void SeedScramblerV29(void* obj, unsigned int seed) { faxw_v29_txsdm(obj)->state = seed; }

/**
 * @brief Select the V.17 transmit encoder.
 * @param obj    the V.17 transmit modem object
 * @param which  V17ENC_ABS (absolute), V17ENC_TCM (trellis) or V17ENC_DIFF (differential); other values change nothing
 * @param val    symbol-mapper state `smc.x` for the TCM and DIFF encoders (ignored for V17ENC_ABS)
 */
void SetEncoderV17(void* obj, short which, short val)
{
  if (which == V17ENC_ABS)
  {
    unsigned short* mode = faxw_v17_encmode(obj);
    *mode = V17ENC_ABS;
  }
  else if (which == V17ENC_TCM)
  {
    unsigned short* mode = faxw_v17_encmode(obj);
    FaxSmcV17* smc = faxw_v17_txsmc(obj);
    *mode = V17ENC_TCM;
    smc->x = val;
  }
  else if (which == V17ENC_DIFF)
  {
    unsigned short* mode = faxw_v17_encmode(obj);
    FaxSmcV17* smc = faxw_v17_txsmc(obj);
    *mode = V17ENC_DIFF;
    smc->x = val;
  }
}

/**
 * @brief Select the V.29 transmit encoder variant.
 * @param obj    the V.29 transmit modem object
 * @param which  0 or 1, stored as the encoder select; other values change nothing
 */
void SetEncoderV29(void* obj, short which)
{
  if (which == 0)
  {
    int* sel = faxw_v29_encsel(obj);
    *sel = 0;
  }
  else if (which == 1)
  {
    int* sel = faxw_v29_encsel(obj);
    *sel = 1;
  }
}

/**
 * @brief Re-initialise the V.27ter transmit scrambler for the modem's current rate, preserving its shift register.
 * @param obj  the V.27ter transmit modem object (its rate index selects the bit count in V27TX_SDM_NUM_BITS)
 */
void SetScramblerV27(void* obj)
{
  FaxSdmCfg27 cfg;
  FaxSdmV27* s = faxw_v27_txsdm(obj);
  short rate = *faxw_v27_rate(obj);
  unsigned short keep;
  cfg.n = V27TX_SDM_NUM_BITS[rate];
  keep = s->reg;
  SDMv27_init(s, &cfg);
  s->reg = keep;
}
