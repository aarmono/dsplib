/**
 * @file dp_v23.c
 * @brief Host glue of the V.23 modem: the "v23" dp_operations record and its create / process / delete callbacks.
 *
 * Same shape as the Bell 103 / V.21 glue (dp_v21_b103.c): one operations record ("v23") registered for DP_V23 by dp_v23_init() / dp_v23_exit(); `create`
 * allocates a V23Glue (core/dp_glue.h: a `struct dp` followed by the private state) that owns a DpWrapper (engine blocks of 160 samples at 8000 Hz) and a
 * V23Modem engine (CreateV23Modem(), v23.c); `process` is the generic dp_wrapper_run(), which calls v23_process() once per block, and v23_process() finds
 * its glue block through `dp->dp_data->priv`.
 *
 * The side of the call decides the engine's role: when `caller` is 0 (the local modem answers) the engine sends the answer tone, receives the 75 bit/s
 * backward channel and transmits the 1200 bit/s forward channel; otherwise it receives the 1200 bit/s channel and transmits at 75 bit/s.  The engine
 * configuration block is {answer-tone flag = (caller == 0), rate 8000, threshold 700}.
 *
 * One block in v23_process(): while the link is connected (`connected`) and the engine asked for `want` transmit bits, fetch them with modem_get_bits() and
 * reduce each to its low bit; run V23ModemMain() with the block as both transmit and receive length; while connected, pack the low bit of each received int
 * into a byte and hand the bits to modem_put_bits(); remember the engine's new request in `want`.  The engine result maps to the data-pump status: 0 ->
 * DPSTAT_OK, and on the first such result since `connected` was cleared: connected = 1, both modem rates set to 75 (caller == 0) or 1200, DPSTAT_CONNECT;
 * 1 -> DPSTAT_OK with `connected` cleared; anything else -> DPSTAT_ERROR with `connected` cleared.  A change of the status is logged
 * as "v23: V23STAT: --> r".
 */
#include "core/dp_glue.h"
#include "core/dsp_constants.h" /* SAMPLE_RATE_* */

/** @brief The glue block of a data pump, found through the wrapper's private slot. */
static V23Glue* glue_of(struct dp* dp) { return (V23Glue*)((DpWrapper*)dp->dp_data)->priv; }

static int v23_process(struct dp* dp, void* in, void* out, int cnt);
static int v23_delete(struct dp* dp);

/**
 * @brief `create` callback: build the glue block, the sample-rate wrapper and the engine.
 * @param m       host modem
 * @param id      data-pump id (DP_V23)
 * @param caller  0 when the local side answers (answer tone, backward-channel receiver), non-zero when it originates
 * @param srate   host sample rate in Hz
 * @param frag    host fragment size (unused: the wrapper is built for blocks of 0xa0 = 160 samples)
 * @param op      the operations record, stored in the `struct dp`
 * @return the new data pump, or NULL when an allocation failed
 */
static struct dp* v23_create(struct modem* m, enum DP_ID id, int caller, int srate, int frag, struct dp_operations* op)
{
  V23Glue* g;
  DpWrapper* w;
  V23Cfg cfg;
  int originate;

  (void)frag;
  if (1 < dsplibs_debug_level)
    dsplibs_debug_printf("v23: create...\n");

  g = (V23Glue*)sysdep_malloc(sizeof(V23Glue));
  if (!g)
    return 0;
  sysdep_memset(g, 0, sizeof(V23Glue));
  g->caller = caller;
  g->dp.id = id;
  g->dp.modem = m;
  g->dp.op = op;

  w = dp_wrapper_create(&g->dp, v23_process, 0xa0, srate, SAMPLE_RATE_8000);
  g->wrapper = w;
  if (!w)
  {
    sysdep_free(g);
    return 0;
  }
  g->dp.dp_data = w;
  w->priv = g;

  /* engine: `originate` is (caller == 0), which is the answering side: it sends the answer tone and receives the backward channel */
  originate = (caller == 0);
  cfg.tone = (unsigned char)originate;
  cfg.pad[0] = cfg.pad[1] = cfg.pad[2] = 0;
  cfg.rate = SAMPLE_RATE_8000;
  cfg.thr = 700;
  g->eng = CreateV23Modem(0, (short)originate, &cfg);
  if (!g->eng)
  {
    dp_wrapper_delete(g->wrapper);
    sysdep_free(g);
    return 0;
  }
  return &g->dp;
}

/** @brief `delete` callback: free the engine, the wrapper and the glue block.  Always returns 0. */
static int v23_delete(struct dp* dp)
{
  V23Glue* g = glue_of(dp);

  DeleteV23Modem(g->eng);
  dp_wrapper_delete(g->wrapper);
  dp->dp_data = 0;
  sysdep_free(g);
  return 0;
}

/**
 * @brief Process one engine block of `cnt` samples (160 at 8000 Hz): transmit the host's bits, deliver the received bits.
 * @param dp   the data pump
 * @param in   received samples (filtered in place by the engine)
 * @param out  transmit samples
 * @param cnt  number of samples in `in` and `out`
 * @return DPSTAT_OK, DPSTAT_CONNECT (first time the engine reports its receiver running) or DPSTAT_ERROR; also stored in `dp->status`
 * @note Both modem rates are set to the same value (75 when answering, 1200 otherwise) although the engine transmits and receives on channels of
 *       different rates; the host's use of the two parameters was not checked.
 */
static int v23_process(struct dp* dp, void* in, void* out, int cnt)
{
  V23Glue* g = glue_of(dp);
  int want = g->want;
  int nRx = 0;
  int r, st, i;

  /* transmit bits: only while connected, and only as many as the engine asked for (low bit of each byte, widened in place) */
  if (g->connected != 0 && want > 0)
  {
    int n = (int)modem_get_bits(dp->modem, 1, (unsigned char*)g->txbits, want);

    want = n;
    for (i = n - 1; i >= 0; i--)
      g->txbits[i] = ((unsigned char*)g->txbits)[i] & 1;
  }

  /* the engine */
  r = V23ModemMain(g->eng, g->txbits, &want, (short*)out, cnt, (short*)in, cnt, (unsigned int*)g->rxbits, &nRx);

  /* received bits, packed to bytes */
  if (g->connected != 0 && nRx > 0)
  {
    unsigned char* rb = (unsigned char*)g->rxbits;

    for (i = 0; i < nRx; i++)
      rb[i] = (unsigned char)(g->rxbits[i] & 1);
    modem_put_bits(dp->modem, 1, rb, nRx);
  }
  g->want = want;

  /* engine result -> data-pump status */
  st = DPSTAT_OK;
  if (r == 0)
  {
    if (g->connected == 0)
    {
      int rate = (g->caller == 0) ? 75 : 1200;

      g->connected = 1;
      modem_set_param(g->dp.modem, MDMPRM_TX_RATE, rate);
      modem_set_param(g->dp.modem, MDMPRM_RX_RATE, rate);
      st = DPSTAT_CONNECT;
    }
  }
  else if (r == 1)
  {
    g->connected = 0;
  }
  else
  {
    g->connected = 0;
    st = DPSTAT_ERROR;
  }
  if (st != (int)dp->status && 1 < dsplibs_debug_level)
    dsplibs_debug_printf("v23: V23STAT: --> %d\n", r);
  dp->status = (unsigned)st;
  return st;
}

static struct dp_operations v23_ops = {"v23", 0, v23_create, v23_delete, dp_wrapper_run, 0};

/** @brief Register the "v23" operations record for DP_V23.  Returns 0. */
int dp_v23_init(void)
{
  modem_dp_register(DP_V23, &v23_ops);
  return 0;
}

/** @brief Deregister the "v23" operations record from DP_V23. */
void dp_v23_exit(void) { modem_dp_deregister(DP_V23, &v23_ops); }
