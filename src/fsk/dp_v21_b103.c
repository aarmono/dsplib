/**
 * @file dp_v21_b103.c
 * @brief Host glue of the Bell 103 / V.21 modem: the "b103" dp_operations record and its create / process / delete callbacks.
 *
 * One operations record ("b103") is registered for the two data-pump ids DP_B103 and DP_V21 (dp_b103_init() / dp_b103_exit()).  The record's `create`
 * allocates a B103Glue (core/dp_glue.h: a `struct dp` followed by the private state) that owns a DpWrapper, which adapts the host's sample rate and
 * fragment size to engine blocks of 160 samples at 8000 Hz, and the Bell 103 / V.21 engine object (B103FP_create(), v21_b103.c).  The record's
 * `process` is the generic dp_wrapper_run(), which calls b103_process() once per engine block with the wrapper's own buffers; b103_process() finds its
 * glue block through `dp->dp_data->priv`.
 *
 * One block in b103_process():
 *   - when `state` (the number of transmit bits the engine asked for) is non-zero, fetch that many bits from the host with modem_get_bits() and widen them
 *     from bytes to ints;
 *   - run B103FP_modem();
 *   - when `state` is non-zero and bits were received, pack the low bit of each received int into a byte and hand the bits to modem_put_bits();
 *   - map the engine status (low byte of the return value) to the next `state` and to the data-pump status: 0 -> state 6 (the engine wants 6 bits per block),
 *     1 .. 4 and values above 7 -> state 0, 5 and 6 -> state 0 and DPSTAT_ERROR, 7 -> DPSTAT_CONNECT (state unchanged).
 * The first block that reports DPSTAT_CONNECT (the status held in `dp` differs) forces state 6 and sets both modem rates to 300.
 *
 * The engine is configured with {caller == 0, the id is DP_V21, 0, 60000, 1, 700, 6200}; B103FP_create() describes the words.
 */
#include "core/dp_glue.h"
#include "core/dsp_constants.h" /* SAMPLE_RATE_* */

/** @brief The glue block of a data pump, found through the wrapper's private slot. */
static B103Glue* glue_of(struct dp* dp) { return (B103Glue*)((DpWrapper*)dp->dp_data)->priv; }

static int b103_process(struct dp* dp, void* in, void* out, int cnt);
static int b103_delete(struct dp* dp);

/**
 * @brief `create` callback: build the glue block, the sample-rate wrapper and the engine.
 * @param m       host modem
 * @param id      DP_B103 or DP_V21; selects the Bell 103 or V.21 frequency plan
 * @param caller  non-zero when the local side originates the call (the engine's role word is `caller == 0`, 0 meaning originate)
 * @param srate   host sample rate in Hz
 * @param frag    host fragment size (unused here: the wrapper is built for blocks of 0xa0 = 160 samples)
 * @param op      the operations record, stored in the `struct dp`
 * @return the new data pump, or NULL when an allocation failed
 */
static struct dp* b103_create(struct modem* m, enum DP_ID id, int caller, int srate, int frag, struct dp_operations* op)
{
  B103Glue* g;
  DpWrapper* w;
  int cfg[7];
  int isV21;

  (void)frag;
  if (1 < dsplibs_debug_level)
    dsplibs_debug_printf("b103: create...\n");

  g = (B103Glue*)sysdep_malloc(sizeof(B103Glue));
  if (!g)
    return 0;
  sysdep_memset(g, 0, sizeof(B103Glue));
  g->caller = caller;
  g->dp.id = id;
  g->dp.modem = m;
  g->dp.op = op;

  w = dp_wrapper_create(&g->dp, b103_process, 0xa0, srate, SAMPLE_RATE_8000);
  g->wrapper = w;
  if (!w)
  {
    sysdep_free(g);
    return 0;
  }
  g->dp.dp_data = w;
  w->priv = g;

  /* engine configuration; see B103FP_create() */
  isV21 = (id == DP_B103) ? 0 : (id == DP_V21);
  cfg[0] = (caller == 0);
  cfg[1] = isV21;
  cfg[2] = 0;
  cfg[3] = 60000;
  cfg[4] = 1;
  cfg[5] = 700;
  cfg[6] = 6200;
  if (1 < dsplibs_debug_level)
    dsplibs_debug_printf("b103: %s config %d,%d,%d,%d,%d %d", isV21 ? "V.21" : "Bell103", cfg[0], cfg[1], 60000, 1, 700, 6200);

  g->last = 1;
  g->eng = B103FP_create(0, cfg);
  if (!g->eng)
  {
    dp_wrapper_delete(g->wrapper);
    sysdep_free(g);
    return 0;
  }
  return &g->dp;
}

/** @brief `delete` callback: free the engine, the wrapper and the glue block.  Always returns 0. */
static int b103_delete(struct dp* dp)
{
  B103Glue* g = glue_of(dp);

  B103FP_delete(g->eng);
  dp_wrapper_delete(g->wrapper);
  dp->dp_data = 0;
  sysdep_free(g);
  return 0;
}

/**
 * @brief Process one engine block (160 samples at 8000 Hz): modulate the host's bits into `out`, demodulate `in` into bits for the host.
 * @param dp   the data pump
 * @param in   received samples of this block (the engine filters them in place)
 * @param out  transmit samples of this block
 * @param cnt  unused
 * @return DPSTAT_OK, DPSTAT_ERROR or DPSTAT_CONNECT (also stored in `dp->status`)
 * @note The transmit side asks for 6 bits per call when `state` is 6; with `state` 0 the engine is called with 6 stale bits, but it forces marks while
 *       its own state byte is non-zero (the link is not up yet).
 */
static int b103_process(struct dp* dp, void* in, void* out, int cnt)
{
  B103Glue* g = glue_of(dp);
  unsigned short txCount, rxCount;
  unsigned int ret;
  int st;

  (void)cnt;

  /* transmit bits: fetch as many as the engine asked for and widen them from bytes to ints (back to front, in place) */
  if (g->state != 0)
  {
    int n, i;

    txCount = (unsigned short)g->state;
    n = (int)modem_get_bits(dp->modem, 1, (unsigned char*)g->txbits, (int)txCount);
    txCount = (unsigned short)n;
    for (i = (int)txCount - 1; i >= 0; i--)
      g->txbits[i] = ((unsigned char*)g->txbits)[i];
  }
  else
  {
    txCount = 6;
  }

  /* the engine: txCount / rxCount are in / out counts (see B103FP_modem()) */
  rxCount = 0xa0;
  ret = B103FP_modem(g->eng, g->txbits, out, in, g->rxbits, &txCount, &rxCount);

  /* received bits: the low bit of each int, packed to bytes */
  if (g->state != 0 && rxCount != 0)
  {
    int i;
    unsigned char* rb = (unsigned char*)g->rxbits;

    for (i = 0; i < (int)rxCount; i++)
      rb[i] = (unsigned char)(g->rxbits[i] & 1);
    modem_put_bits(dp->modem, 1, rb, (int)rxCount);
  }

  if (ret != (unsigned int)g->last)
  {
    if (1 < dsplibs_debug_level)
      dsplibs_debug_printf("b103: B103 state -> %x (msg: %x)\n", ret, ret & 0xff);
    g->last = (int)ret;
  }

  /* engine state byte -> number of bits wanted next and data-pump status */
  st = DPSTAT_OK;
  switch ((int)(ret & 0xff))
  {
  case 0:
    g->state = 6;
    break;
  case 1:
  case 2:
  case 3:
  case 4:
    g->state = 0;
    break;
  case 5:
  case 6:
    g->state = 0;
    st = DPSTAT_ERROR;
    break;
  case 7:
    st = DPSTAT_CONNECT;
    if (1 < dsplibs_debug_level)
      dsplibs_debug_printf("b103: ReturnStatus = BELL_103_LINKED\n");
    break;
  default:
    g->state = 0;
    break;
  }

  /* the first CONNECT: start sending data at 300 bit/s */
  if (st != (int)dp->status && st == DPSTAT_CONNECT)
  {
    g->state = 6;
    modem_set_param(g->dp.modem, MDMPRM_TX_RATE, 300);
    modem_set_param(g->dp.modem, MDMPRM_RX_RATE, 300);
    dp->status = DPSTAT_CONNECT;
    return DPSTAT_CONNECT;
  }
  dp->status = (unsigned)st;
  return st;
}

static struct dp_operations b103_ops = {"b103", 0, b103_create, b103_delete, dp_wrapper_run, 0};

/** @brief Register the "b103" operations record for DP_B103 and DP_V21.  Returns 0. */
int dp_b103_init(void)
{
  modem_dp_register(DP_B103, &b103_ops);
  modem_dp_register(DP_V21, &b103_ops);
  return 0;
}

/** @brief Deregister the "b103" operations record from DP_B103 and DP_V21. */
void dp_b103_exit(void)
{
  modem_dp_deregister(DP_B103, &b103_ops);
  modem_dp_deregister(DP_V21, &b103_ops);
}
