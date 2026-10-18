/**
 * @file dp_v32.c
 * @brief Data-pump glue between slmodemd and the V.32 / V.32bis engine: create, process and delete callbacks, and the registration for DP_V32 / DP_V32BIS.
 *
 * slmodemd drives the pump through a `struct dp` and the operations record `v32_ops` ({ "v32", 0, v32_create, v32_delete, dp_wrapper_run, NULL }).
 * dp_v32_init() / dp_v32_exit() register / deregister that one record for DP_V32 (32) and DP_V32BIS (132).  The engine itself (V32FP_*) is in
 * src/v32/v32_engine.c; the sample-rate / block adapter (DpWrapper) is in core/dp_wrapper.c.
 *
 * Flow.  op->create(m, id, caller, srate, frag, op) builds a V32Glue (a `struct dp` followed by the private tail) that owns a DpWrapper (40-sample engine
 * blocks at 8000 Hz) and the V.32 engine.  op->process is the generic dp_wrapper_run(), which calls v32_process() once per engine block; v32_process()
 * finds its glue through `dp->dp_data->priv` (the wrapper's free slot).  Per block, unless dp->status is 4 (then the output block is zeroed and 4 is returned
 * at once):
 *  - `want` transmit bits (0 until the first CONNECT, then 12) are fetched with modem_get_bits() and widened in place to ints, then V32FP_modem() runs;
 *  - the engine's cleaned samples are logged (modem_debug_log_data, id 3) when it has any;
 *  - when `want` is 0 the demodulated bits are dropped, otherwise at most 100 of them (the message "rx_len is too big" is printed above that) are masked to
 *    `bits` bits, packed to bytes in place and handed to modem_put_bits() (which is called also for 0 bits);
 *  - the engine status byte is mapped to the result: 3, 4, 13 and 24 .. 27 = connected (V32FP_status gives the connect rates, a table of five line rates
 *    updates `rate`, `want` becomes 12, `bits` = rate / 2400, both modem rates are set; result DPSTAT_CONNECT); 12 and 14 = no carrier, 16 .. 23 = error and
 *    anything above 28 = unknown give DPSTAT_ERROR; all others give DPSTAT_OK.  The result is also stored in dp->status.
 *
 * Note.  modem_set_param() is given the glue's own modem pointer (g->dp.modem) while modem_get_bits() / modem_put_bits() / modem_debug_log_data() are given
 * dp->modem; slmodemd passes the same object for both.
 *
 * The engine configuration built by v32_create() is { caller == 0, (physDelay * 5) / 6, 60000, 700, 1, txRate | txRate << 16 } with
 * txRate = min(modem parameter MDMPRM_MAX_RATE, 14400) and physDelay = MDMPRM_IODELAY + 0x30 clipped to 0xd8 (the excess is written back to
 * MDMPRM_UPDATE_DELAY as a negative delay).
 */
#include "core/dp_glue.h"
#include "core/dsp_constants.h" /* RATE_* */

/** @brief The V32Glue that owns `dp` (stored in the wrapper's private slot by v32_create()). */
static V32Glue* glue_of(struct dp* dp) { return (V32Glue*)((DpWrapper*)dp->dp_data)->priv; }

static int v32_process(struct dp* dp, void* in, void* out, int cnt);
static int v32_delete(struct dp* dp);

/**
 * @brief Create a V.32 / V.32bis data pump (operations record `create` callback).
 * @param m      the modem (read for the maximum rate and I/O delay parameters)
 * @param id     DP_V32 or DP_V32BIS
 * @param caller non-zero when the modem is the calling side
 * @param srate  line sample rate of the host
 * @param frag   unused
 * @param op     the operations record to store in the new `struct dp`
 * @return the new `struct dp` (the head of a V32Glue), or NULL when an allocation or the engine creation fails
 *
 * Starts in the 14400 bit/s, 6 bits-per-symbol configuration with no transmit bits requested (`want` = 0) until the first CONNECT.
 */
static struct dp* v32_create(struct modem* m, enum DP_ID id, int caller, int srate, int frag, struct dp_operations* op)
{
  V32Glue* g;
  DpWrapper* w;
  int cfg[6];
  int physDelay;
  unsigned txRate;

  (void)frag;
  if (1 < dsplibs_debug_level)
    dsplibs_debug_printf("v32: create...\n");

  g = (V32Glue*)sysdep_malloc(sizeof(V32Glue));
  if (!g)
    return 0;
  sysdep_memset(g, 0, sizeof(V32Glue));
  g->bits = 6;
  g->rate = RATE_14400;
  g->want = 0;
  g->dp.id = id;
  g->dp.op = op;
  g->dp.modem = m;

  /* the wrapper converts between the host's block size / sample rate and the engine's 40-sample blocks at 8000 Hz */
  w = dp_wrapper_create(&g->dp, v32_process, 0x28, srate, SAMPLE_RATE_8000);
  g->wrapper = w;
  if (!w)
  {
    sysdep_free(g);
    return 0;
  }
  w->priv = g;
  g->dp.dp_data = w;

  /* engine configuration from the host parameters */
  txRate = (unsigned)modem_get_param(m, MDMPRM_MAX_RATE);
  if (txRate > RATE_14400)
    txRate = RATE_14400;
  physDelay = (int)modem_get_param(m, MDMPRM_IODELAY) + 0x30;
  if (physDelay > 0xd8)
  {
    modem_set_param(m, MDMPRM_UPDATE_DELAY, 0xd8 - physDelay);
    physDelay = 0xd8;
  }
  if (1 < dsplibs_debug_level)
    dsplibs_debug_printf("v32: phys. delay is %d\n", physDelay);
  cfg[0] = (caller == 0);
  cfg[1] = (physDelay * 5) / 6;
  cfg[2] = 60000;
  cfg[3] = 700;
  cfg[4] = 1;
  cfg[5] = (int)(txRate | (txRate << 16));
  if (1 < dsplibs_debug_level)
    dsplibs_debug_printf("v32: V32 config S%d,R%d,T%d,A%d,L%d\n", cfg[0], (int)txRate, 60000, 1, 700);

  g->eng = V32FP_create(cfg, 0);
  if (!g->eng)
  {
    dp_wrapper_delete(g->wrapper);
    sysdep_free(g);
    return 0;
  }
  return &g->dp;
}

/**
 * @brief Destroy a V.32 data pump (operations record `delete` callback): release the engine, the wrapper and the glue block.
 * @param dp  the `struct dp` returned by v32_create()
 * @return 0
 */
static int v32_delete(struct dp* dp)
{
  V32Glue* g = glue_of(dp);
  if (1 < dsplibs_debug_level)
    dsplibs_debug_printf("v32: delete...\n");
  V32FP_delete(g->eng);
  dp_wrapper_delete(g->wrapper);
  dp->dp_data = 0;
  sysdep_free(g);
  return 0;
}

/**
 * @brief Process one engine block: transmit bits in, modulated samples out; line samples in, demodulated bits out (called by dp_wrapper_run()).
 * @param dp   the data pump
 * @param in   received line samples (16-bit)
 * @param out  receives the transmit samples (16-bit); `cnt` samples
 * @param cnt  samples in this block
 * @return DPSTAT_OK, DPSTAT_CONNECT (the engine reports a connected state), or DPSTAT_ERROR (no carrier, error or an unknown engine status); 4 is returned
 *         at once, with `out` zeroed, when a previous call already stored the error status in dp->status
 * @note   The receive bits are taken from the engine as one bit group per int and are clipped to 100 per block (the size of V32Glue::rxbits).
 */
static int v32_process(struct dp* dp, void* in, void* out, int cnt)
{
  V32Glue* g = glue_of(dp);
  unsigned int txCount, rxCount;
  unsigned char st;
  int ret, i, n;
  int cleaned;
  char* samples;
  unsigned char* rb;

  if (dp->status == 4)
  {
    sysdep_memset(out, 0, (size_t)(unsigned)cnt * 2u);
    return 4;
  }

  /* fetch the transmit bits as bytes, then widen them in place to ints (last first) */
  n = modem_get_bits(dp->modem, g->bits, (unsigned char*)g->txbits, g->want);
  txCount = (unsigned int)n;
  for (i = n - 1; i >= 0; i--)
    g->txbits[i] = ((unsigned char*)g->txbits)[i];
  rxCount = (unsigned int)cnt;

  st = V32FP_modem(g->eng, g->txbits, out, in, g->rxbits, &txCount, &rxCount);

  /* log the equalised samples when the engine has them */
  cleaned = 0;
  samples = V32FP_GetCleanedSamples(g->eng, &cleaned);
  if (samples && cleaned > 0)
    modem_debug_log_data(dp->modem, 3, (const void*)samples, cleaned * 2);

  /* limit the received bits and pass them to the host as bytes */
  if (g->want == 0)
  {
    rxCount = 0;
  }
  else if (rxCount > 99)
  {
    if (1 < dsplibs_debug_level)
      dsplibs_debug_printf("v32: FATAL: rx_len is too big (%d).\n", (int)rxCount);
    rxCount = 100;
  }
  rb = (unsigned char*)g->rxbits;
  for (i = 0; i < (int)rxCount; i++)
    rb[i] = (unsigned char)(((1 << g->bits) - 1) & (unsigned)g->rxbits[i]);
  modem_put_bits(dp->modem, g->bits, rb, (int)rxCount);

  /* map the engine status to the result for the host */
  switch (st)
  {
  case 3:
  case 4:
  case 13:
  case 24:
  case 25:
  case 26:
  case 27:
  {
    short stat[22] = {0};
    short txr, rxr;
    if (1 < dsplibs_debug_level)
      dsplibs_debug_printf("v32: process: connect %x.\n", (unsigned)st);
    /* the connect rates come from the engine status; stat[1] / stat[2] are the transmit / receive rates */
    V32FP_status(g->eng, stat);
    txr = stat[1];
    rxr = stat[2];
    switch (txr)
    {
    case RATE_9600:
      g->rate = RATE_9600;
      break;
    case RATE_12000:
      g->rate = RATE_12000;
      break;
    case RATE_14400:
      g->rate = RATE_14400;
      break;
    case RATE_4800:
      g->rate = RATE_4800;
      break;
    case RATE_7200:
      g->rate = RATE_7200;
      break;
    default:
      break;
    }

    /* from now on 12 transmit bit groups are requested per block, bits per symbol = rate / 2400 */
    g->want = 12;
    g->bits = (int)(((unsigned long long)(unsigned)g->rate * 0x1b4e81b5ull) >> 40);
    if (1 < dsplibs_debug_level)
      dsplibs_debug_printf("v32: v32_update connect: tx_rate %d, rx_rate %d\n", (int)txr, (int)rxr);
    modem_set_param(g->dp.modem, MDMPRM_TX_RATE, txr);
    modem_set_param(g->dp.modem, MDMPRM_RX_RATE, rxr);
    ret = DPSTAT_CONNECT;
    break;
  }
  case 12:
  case 14:
    if (1 < dsplibs_debug_level)
      dsplibs_debug_printf("v32: process: nocarrier.\n");
    ret = DPSTAT_ERROR;
    break;
  case 16:
  case 17:
  case 18:
  case 19:
  case 20:
  case 21:
  case 22:
  case 23:
    if (1 < dsplibs_debug_level)
      dsplibs_debug_printf("v32: process: error.\n");
    ret = DPSTAT_ERROR;
    break;
  case 0:
  case 1:
  case 2:
  case 5:
  case 6:
  case 7:
  case 8:
  case 9:
  case 10:
  case 11:
  case 15:
  case 28:
    ret = DPSTAT_OK;
    break;
  default:
    if (1 < dsplibs_debug_level)
      dsplibs_debug_printf("v32: process: unknown V.32 status: %d.\n", (int)st);
    ret = DPSTAT_ERROR;
    break;
  }
  dp->status = (unsigned)ret;
  return ret;
}

/** The operations record registered for DP_V32 and DP_V32BIS. */
static struct dp_operations v32_ops = {"v32", 0, v32_create, v32_delete, dp_wrapper_run, 0};

/** @brief Register the V.32 and V.32bis data pumps with the host.  @return 0 */
int dp_v32_init(void)
{
  modem_dp_register(DP_V32, &v32_ops);
  modem_dp_register(DP_V32BIS, &v32_ops);
  return 0;
}

/** @brief Deregister the V.32 and V.32bis data pumps. */
void dp_v32_exit(void)
{
  modem_dp_deregister(DP_V32, &v32_ops);
  modem_dp_deregister(DP_V32BIS, &v32_ops);
}
