/**
 * @file dp_v8.c
 * @brief The V.8 data-pump glue: registers the "V8" data pump with the modem and adapts the V.8 engine (v8.c) to the dp interface.
 *
 * Entry points: dp_v8_init() / dp_v8_exit() register and deregister the data pump (DP_V8); the dp operations v8_create() / v8_delete() / v8_process()
 * are static and reached through the registered table `v8_op` (V.8 does not go through the data-pump wrapper: v8_process() is the process slot
 * itself).  The table v8StatusName (19 names of the V8Status values) is exported and used for the debug output.
 *
 * v8_create(m, id, caller, srate, frag, op): only 9600 Hz.  It allocates a V8Glue (a `struct dp` whose dp_data points at the glue itself) that
 * holds the V.8 engine (V8Create() with the configuration { originate = (caller == 0), operation mode 0, signal-detect timeout 12 s, message
 * timeout 7 s, 9600, the DpRuntime block }), the modem's dsp_info (host parameter MDMPRM_DSPINFO) and the DpRuntime block (dp_param_get()).  It sets
 * the capability bits of the runtime block: flag byte 0 bit 1 cleared; bit 3 (V.90) when this side originates (`caller` non-zero) for DP id V.90 or
 * V.92; bits 5 and 7 (V.34, V.32) set; byte 1 bit 6 (data call function) set; byte 2 bit 4 (V.92 quick connect) when originating for V.92.
 * `automode` (MDMPRM_AUTOMODE) is only printed.
 *
 * v8_process(dp, in, out, cnt): runs V8Process() and maps its V8Status to the glue status: the timeout statuses (V8_ANS_TIME_OUT_WAITING_FOR_CM / _CJ,
 * V8_ORG_TIME_OUT_WAITING_FOR_ANSAM / _JM / _QCA1d) give DPSTAT_ERROR; V8_OK, after V8UpdateModemParameters(), selects the data pump to switch to
 * (the id the caller asked for when the runtime block has quick connect set, otherwise V.90, V.34 or V.32bis from the capability bits; the quick
 * connect LAPM bit and index go to dsp_info), requests it with MDMPRM_DP_REQUESTED, arms an idle timer (MDMPRM_IODELAY + 0x2a0 samples) and returns
 * DPSTAT_CHANGEDP; V8_ORG_BAD_QCA1d_MESSAGE requests DP_V92 (only when the glue was created for V.90 / V.92).  While the idle timer runs it counts
 * `cnt` down and, when it expires, requests data pump 0 and returns DPSTAT_CHANGEDP.
 */
#include "core/dp_glue.h"
#include "v8/v8_status.h"       /* enum V8Status (generated from v8StatusName below) */
#include "core/dsp_constants.h" /* SAMPLE_RATE_* */

const char* const v8StatusName[19] = {
    "V8_INIT",
    "V8_ANS_SEND_ANSAM",
    "V8_ANS_CM_DETECTED",
    "V8_ANS_SEND_JM",
    "V8_ANS_TIME_OUT_WAITING_FOR_CM",
    "V8_ANS_TIME_OUT_WAITING_FOR_CJ",
    "V8_ORG_WAITING_FOR_ANSAM",
    "V8_ORG_ANSAM_DETECTED_WAITING_TE",
    "V8_ORG_SEND_CM",
    "V8_ORG_JM_DETECTED",
    "V8_ORG_SEND_CJ",
    "V8_ORG_TIME_OUT_WAITING_FOR_ANSAM",
    "V8_ORG_TIME_OUT_WAITING_FOR_JM",
    "V8_OK",
    "V8_ORG_SEND_QC",
    "V8_ORG_WAITING_FOR_QCA1d",
    "V8_ORG_BAD_QCA1d_MESSAGE",
    "V8_ORG_TIME_OUT_WAITING_FOR_QCA1d",
    "V8_LAST_ENUM",
};

static int v8_delete(struct dp* dp);
static int v8_process(struct dp* dp, void* in, void* out, int cnt);

/**
 * @brief dp create operation: build the V.8 glue for a call.
 * @param m       the modem
 * @param id      the data pump id the call is for (V.90 / V.92 / V.34 ...); selects the capability bits and the quick-connect handling
 * @param caller  non-zero when this side originates the call (the engine side is then 0, the caller); 0 when it answers (engine side 1)
 * @param srate   sample rate; anything but 9600 Hz fails
 * @param frag    unused
 * @param op      the registered operations table, stored in the glue
 * @return the dp object (inside the V8Glue), or NULL when the rate is not 9600 or an allocation failed
 */
static struct dp* v8_create(struct modem* m, enum DP_ID id, int caller, int srate, int frag, struct dp_operations* op)
{
  V8Glue* g;
  DpRuntime* rt;
  V8Cfg cfg;
  int automode;
  int answerV90;

  (void)frag;
  automode = (int)modem_get_param(m, MDMPRM_AUTOMODE);
  if (srate != SAMPLE_RATE_9600)
    return 0;

  if (1 < dsplibs_debug_level)
    dsplibs_debug_printf("v8: create: caller %d, automode %d, dp id %d.\n", caller, automode, (int)id);
  g = (V8Glue*)sysdep_malloc(sizeof(V8Glue));
  if (!g)
    return 0;
  sysdep_memset(g, 0, sizeof(V8Glue));
  g->dp.modem = m;
  g->dp.id = DP_V8;
  g->dp.dp_data = g;
  g->dp.op = op;
  g->originate = (caller == 0);
  g->id = (int)id;
  g->idle = 0;
  g->di = (struct dsp_info*)modem_get_param(m, MDMPRM_DSPINFO);
  rt = (DpRuntime*)dp_param_get(m);
  g->rt = rt;

  /* the capability bits of the runtime block */
  rt->flags0 &= 0xfd;
  rt->flags1 |= 0x40;
  answerV90 = 0;
  if (caller != 0 && (id == DP_V90 || id == DP_V92))
    answerV90 = 1;
  rt->flags0 = (unsigned char)((rt->flags0 & 0xf7) | ((answerV90 & 1) << 3));
  rt->flags0 |= 0x20;
  rt->flags0 |= 0x80;
  rt->flags2 = (unsigned char)((rt->flags2 & 0xef) | ((caller != 0 && id == DP_V92) << 4));

  /* the engine */
  sysdep_memset(&cfg, 0, sizeof(cfg));
  cfg.side = (caller == 0);
  cfg.opMode = 0;
  cfg.sigTimeout = 0xc;
  cfg.msgTimeout = 7;
  cfg.pad = 0x2580;
  cfg.caps = (const unsigned char*)rt;
  g->eng = V8Create(&cfg);
  if (!g->eng)
  {
    sysdep_free(g);
    return 0;
  }
  g->status = V8_INIT;
  return &g->dp;
}

/** @brief dp delete operation: delete the engine and the glue.  Returns 0. */
static int v8_delete(struct dp* dp)
{
  V8Glue* g = (V8Glue*)dp->dp_data;
  if (1 < dsplibs_debug_level)
    dsplibs_debug_printf("v8: delete...\n");
  V8Delete(g->eng);
  sysdep_free(g);
  return 0;
}

/**
 * @brief dp process operation: run the V.8 engine over `cnt` samples (`in` received, `out` transmitted) and act on its status.
 * @return DPSTAT_OK, DPSTAT_ERROR on a handshake timeout or an unusable result, or DPSTAT_CHANGEDP when the next data pump has been requested
 */
static int v8_process(struct dp* dp, void* in, void* out, int cnt)
{
  V8Glue* g = (V8Glue*)dp->dp_data;
  DpRuntime* rt = g->rt; /* read once: the glue block, the runtime block and the modem's dsp_info are three distinct objects */
  struct dsp_info* di = g->di;
  unsigned int st = V8Process(g->eng, (const short*)in, (short*)out, cnt);
  int ret = DPSTAT_OK;
  int req = -1; /* DP id to request from the modem (modem_set_param(m, 9, req)), -1 = none */
  int idle_expiry = 0;

  /* react to the new status */
  switch (st)
  {
  case V8_ANS_TIME_OUT_WAITING_FOR_CM:
  case V8_ANS_TIME_OUT_WAITING_FOR_CJ:
  case V8_ORG_TIME_OUT_WAITING_FOR_ANSAM:
  case V8_ORG_TIME_OUT_WAITING_FOR_JM:
  case V8_ORG_TIME_OUT_WAITING_FOR_QCA1d:
    if (1 < dsplibs_debug_level)
      dsplibs_debug_printf("v8: process: timeout.\n");
    ret = DPSTAT_ERROR;
    break;
  case V8_OK:
    if (1 < dsplibs_debug_level)
      dsplibs_debug_printf("v8: process: OK.\n");
    if (g->idle != 0)
      break;
    V8UpdateModemParameters(g->eng, (unsigned char*)rt);
    if (rt->flags2 & 0x10)
    {
      if (1 < dsplibs_debug_level)
        dsplibs_debug_printf("v8: process: QC.\n");
      req = g->id;
    }
    else
    {
      unsigned char b0 = rt->flags0;
      if (b0 & 0x08)
        req = DP_V90;
      else if (b0 & 0x20)
        req = DP_V34;
      else if (b0 & 0x80)
        req = DP_V32;
      else
      {
        ret = DPSTAT_ERROR;
        break;
      }
    }
    di->qc_lapm = (rt->flags2 >> 6) & 1;
    di->qc_index = (unsigned)rt->qcIndex;
    break;
  case V8_ORG_BAD_QCA1d_MESSAGE:
    if (g->id != DP_V92 && g->id != DP_V90)
    {
      ret = DPSTAT_ERROR;
      break;
    }
    if (g->idle != 0)
      break;
    di->qc_lapm &= 1;
    req = DP_V92;
    break;
  default:
    break;
  }

  /* a data pump was selected: request it from the modem and start the idle timer */
  if (req >= 0)
  {
    ret = DPSTAT_CHANGEDP;
    modem_set_param(dp->modem, MDMPRM_DP_REQUESTED, req);
    g->idle = (int)modem_get_param(dp->modem, MDMPRM_IODELAY) + 0x2a0;
    if (1 < dsplibs_debug_level)
      dsplibs_debug_printf("v8: Link established. Idle timer %d.\n", g->idle);
  }

  if (g->status != (int)st)
  {
    if (1 < dsplibs_debug_level)
      dsplibs_debug_printf("v8: status (%d) %s\n", (int)st, v8StatusName[st]);
    g->status = (int)st;
  }

  /* idle timer */
  if (g->idle > 0)
  {
    int left = g->idle - cnt;
    if (left > 0)
    {
      g->idle = left;
    }
    else
    {
      idle_expiry = 1;
    }
  }
  if (idle_expiry)
  {
    g->idle = -1;
    g->w1c = 0;
    modem_set_param(dp->modem, MDMPRM_DP_REQUESTED, 0);
    return DPSTAT_CHANGEDP;
  }
  return ret;
}

static struct dp_operations v8_op = {"V8", 0, v8_create, v8_delete, v8_process, 0};

/** @brief Register the V.8 data pump (DP_V8) with the modem.  Returns 0. */
int dp_v8_init(void)
{
  modem_dp_register(DP_V8, &v8_op);
  return 0;
}

/** @brief Deregister the V.8 data pump. */
void dp_v8_exit(void) { modem_dp_deregister(DP_V8, &v8_op); }
