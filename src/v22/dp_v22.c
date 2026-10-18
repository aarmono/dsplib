/**
 * @file dp_v22.c
 * @brief The V.22 / V.22bis / Bell 212A data-pump glue: registers the pump with the host and runs the V.22 engine through a DpWrapper.
 *
 * The pump operations are `{ "v22", 0, v22_create, v22_delete, dp_wrapper_run, NULL }`, registered by dp_v22_init() for DP_V22BIS, DP_V22 and DP_B212
 * (in that order) and deregistered by dp_v22_exit().  The engine (V22FP_*, v22_engine.c) and dp_wrapper are separate modules.
 *
 * Flow.  op->create(m, id, caller, srate, frag, op) builds a V22Glue (a `struct dp`, dp_v22.h) that owns a DpWrapper (160-sample engine blocks at 8000 Hz)
 * and the engine, V22FP_create(NULL, cfg) with cfg = { caller == 0 (1 = the local modem answers), protocol (DP_B212: 2, DP_V22: 1, DP_V22BIS: 0), 60000
 * (handshake timeout ms), 0, 700 (carrier loss time ms), 0, 1 (retrain on poor quality allowed) }.  op->process is the generic dp_wrapper_run(), which
 * calls v22_process() once per block (the glue is found as dp->dp_data->priv).  Per block:
 * - transmit: while no CONNECT has been reported (want == 0) the engine is told that 12 tx bits are available and the tx array is not refreshed; else
 *   modem_get_bits(m, bits, txbits, want) fetches the bits and widens them in place to ints;
 * - V22FP_modem(eng, txbits, out, in, rxbits, &txCount, &rxCount = cnt); then, while want == 0, the received bits are dropped (count 0), else an
 *   UNSIGNED count above 100 is clamped to 100 (message "v22: FATAL: rx_len is huge (%d)." at debug level > 1); the rx bits are masked with
 *   (1 << bits) - 1, packed to bytes in place and handed to modem_put_bits(m, bits, ...) (also called for 0 bits);
 * - the engine status byte (V22Msg): 0 -> DPSTAT_OK; 1 (handshake) -> want = 0 and DPSTAT_OK; 3 -> DPSTAT_CONNECT at 2400 bit/s (want = 12, bits = 4,
 *   both modem rates 2400); 4 -> DPSTAT_CONNECT at 1200 (bits = 2, rates 1200); every other value (2, 5 and above) -> DPSTAT_ERROR.  The result is
 *   stored in dp->status; when it differs from the old status the message "v22: V22STAT: --> %d" (the engine status) is printed at debug level > 1.
 */
#include "dp_v22.h"
#include "core/dsp_constants.h" /* RATE_* */

/* The glue block of a pump instance, found through the wrapper that dp->dp_data points at. */
static V22Glue* glue_of(struct dp* dp) { return (V22Glue*)((DpWrapper*)dp->dp_data)->priv; }

static int v22_process(struct dp* dp, void* in, void* out, int cnt);
static int v22_delete(struct dp* dp);

/**
 * @brief The `create` operation: build the glue block, the DpWrapper and the engine.
 * @param m      host modem object
 * @param id     DP_V22BIS, DP_V22 or DP_B212 (selects the engine protocol)
 * @param caller 0 when the local modem answers (the engine runs the answering machine), non-zero when it originates
 * @param srate  line sample rate, Hz; must be a rate dp_wrapper_create() can convert to 8000 Hz
 * @param frag   host fragment size (unused)
 * @param op     the operations record (stored in the block)
 * @return the `struct dp` of the new glue block, or NULL if an allocation or the wrapper creation failed
 * @warning The V22FP_create() result is checked, but it does not check its own allocations (see there).
 */
static struct dp* v22_create(struct modem* m, enum DP_ID id, int caller, int srate, int frag, struct dp_operations* op)
{
  V22Glue* g;
  DpWrapper* w;
  int cfg[7];

  (void)frag;
  if (1 < dsplibs_debug_level)
    dsplibs_debug_printf("v22: create...\n");
  g = (V22Glue*)sysdep_malloc(sizeof(V22Glue));
  if (!g)
    return 0;
  sysdep_memset(g, 0, sizeof(V22Glue));
  g->dp.id = id;
  g->dp.modem = m;
  g->dp.op = op;
  w = dp_wrapper_create(&g->dp, v22_process, 0xa0, srate, SAMPLE_RATE_8000);
  g->wrapper = w;
  if (!w)
  {
    sysdep_free(g);
    return 0;
  }
  g->dp.dp_data = w;
  w->priv = g;
  cfg[0] = (caller == 0);
  cfg[1] = (id == DP_B212) ? 2 : (id == DP_V22);
  cfg[2] = 60000;
  cfg[3] = 0;
  cfg[4] = 700;
  cfg[5] = 0;
  cfg[6] = 1;
  g->eng = V22FP_create(NULL, cfg);
  if (!g->eng)
  {
    dp_wrapper_delete(g->wrapper);
    sysdep_free(g);
    return 0;
  }
  return &g->dp;
}

/** @brief The `delete` operation: delete the engine and the wrapper and free the glue block.  @return 0 */
static int v22_delete(struct dp* dp)
{
  V22Glue* g = glue_of(dp);
  V22FP_delete(g->eng);
  dp_wrapper_delete(g->wrapper);
  dp->dp_data = 0;
  sysdep_free(g);
  return 0;
}

/**
 * @brief The block processor called by dp_wrapper_run(): one engine block.
 * @param dp   the data pump
 * @param in   `cnt` input samples (16-bit, 8000 Hz) of the engine block
 * @param out  receives `cnt` output samples
 * @param cnt  samples in the block (160)
 * @return the DPSTAT_* code derived from the engine status (see the file header)
 * @warning `cnt` must not exceed 160: the engine copies the input into a 160-sample static buffer.
 * @bug     Only the engine states 0, 1, 3 and 4 are understood; every other state byte is reported as DPSTAT_ERROR.  That includes the retrain states
 *          (2 retrain running, 9 retrain requested by the remote modem, 0xb local retrain) and 0x10 (carrier lost for longer than the loss time).
 *          slmodemd maps DPSTAT_ERROR to STATUS_DP_ERROR and ends the call with NO CARRIER (modem.c, dpstat2status() / modem_update_status()), so a
 *          retrain request or a carrier loss hangs the modem up instead of retraining (read from the code; not run).
 */
static int v22_process(struct dp* dp, void* in, void* out, int cnt)
{
  V22Glue* g = glue_of(dp);
  unsigned int txCount, rxCount;
  unsigned char st;
  int ret, i;
  unsigned char* rb;
  unsigned char* tb;

  /* transmit bits for the engine */
  if (g->want == 0)
  {
    txCount = 12;
  }
  else
  {
    int n;
    tb = (unsigned char*)g->txbits;
    n = modem_get_bits(dp->modem, g->bits, tb, g->want);
    txCount = (unsigned int)n;
    for (i = n - 1; i >= 0; i--)
      g->txbits[i] = tb[i];
  }
  rxCount = (unsigned int)cnt;
  st = (unsigned char)V22FP_modem(g->eng, g->txbits, (short*)out, (const short*)in, g->rxbits, &txCount, &rxCount);

  /* received bits to the host */
  if (g->want == 0)
  {
    rxCount = 0;
  }
  else if (rxCount > 100)
  {
    if (1 < dsplibs_debug_level)
      dsplibs_debug_printf("v22: FATAL: rx_len is huge (%d).\n", (int)rxCount);
    rxCount = 100;
  }
  rb = (unsigned char*)g->rxbits;
  for (i = 0; i < (int)rxCount; i++)
    rb[i] = (unsigned char)(((1u << (g->bits & 31)) - 1u) & (unsigned)g->rxbits[i]);
  modem_put_bits(dp->modem, g->bits, rb, (int)rxCount);

  /* engine status -> data-pump status */
  switch (st)
  {
  case V22_MSG_RUNNING:
    ret = DPSTAT_OK;
    break;
  case V22_MSG_HANDSHAKE:
    g->want = 0;
    ret = DPSTAT_OK;
    break;
  case V22_MSG_CONNECT_2400:
    g->want = 12;
    g->bits = 4;
    modem_set_param(g->dp.modem, MDMPRM_TX_RATE, RATE_2400);
    modem_set_param(g->dp.modem, MDMPRM_RX_RATE, RATE_2400);
    ret = DPSTAT_CONNECT;
    break;
  case V22_MSG_CONNECT_1200:
    g->want = 12;
    g->bits = 2;
    modem_set_param(g->dp.modem, MDMPRM_TX_RATE, RATE_1200);
    modem_set_param(g->dp.modem, MDMPRM_RX_RATE, RATE_1200);
    ret = DPSTAT_CONNECT;
    break;
  default:
    ret = DPSTAT_ERROR;
    break;
  }
  if (ret != (int)dp->status && 1 < dsplibs_debug_level)
    dsplibs_debug_printf("v22: V22STAT: --> %d\n", (int)st);
  dp->status = (unsigned)ret;
  return ret;
}

static struct dp_operations v22_ops = {"v22", 0, v22_create, v22_delete, dp_wrapper_run, 0};

/** @brief Register the V.22 pump with the host for DP_V22BIS, DP_V22 and DP_B212.  @return 0 */
int dp_v22_init(void)
{
  modem_dp_register(DP_V22BIS, &v22_ops);
  modem_dp_register(DP_V22, &v22_ops);
  modem_dp_register(DP_B212, &v22_ops);
  return 0;
}

/** @brief Deregister the V.22 pump from the host. */
void dp_v22_exit(void)
{
  modem_dp_deregister(DP_V22BIS, &v22_ops);
  modem_dp_deregister(DP_V22, &v22_ops);
  modem_dp_deregister(DP_B212, &v22_ops);
}
