/**
 * @file v22_engine.c
 * @brief The V.22 / V.22bis / Bell 212A engine: creation, the per-block entry point V22FP_modem(), and the helper functions the protocol handlers use.
 *
 * The engine objects are the typed structs V22Engine / V22Tones / V22Core of v22.h; the protocol handlers V22FP_modem() dispatches to are a table of
 * function pointers, V22_PROTOCOL.
 * - V22FP_create(eng, cfg[7]): `eng == NULL` allocates the three blocks (and the work buffers), otherwise the engine is re-initialised in place (the
 *   call-setup and loop code re-create it).  cfg = { mode (0 originate / 1 answer / 2 loop), protocol (0 = V.22bis 2400, 1 / 2 = 1200), the handshake
 *   timeout in ms, bit 0 -> b11 bit 2 (start with the answer tone), the carrier loss time in ms, bit 0 -> b11 bit 3 (start in the data state), bit 0 ->
 *   b11 bit 1 (retrain on poor signal quality) }.  It builds the transmit chain (SMC -> PPS, with the pulse-shaping coefficients computed through a
 *   temporary tone generator), the receive chain (MRF, SRE, two AGCs, FSE, descrambler), the detectors and the IIR section.
 * - V22FP_modem(): one 160-sample block.
 * - Everything else: small helpers over the typed members (SetTxRate / SetRxRate re-programme the scrambler / SMC / PPS tables for 1200 or 2400 bit/s,
 *   ModDataV22 / DemodDataV22 run the transmit and receive chains, the status and detector accessors ...).
 *
 * The engine keeps three static 16-bit work buffers (tx_in_internal, rx_in_internal, rx_out_internal) that V22FP_modem() fills on every call; the
 * engine is therefore not reentrant and all engine instances share them.
 */
#include <stdint.h>

#include "fpm/fpm.h"
#include "fpm/fpm_tables.h"
#include "v22.h"
#include "v22_tables.h"
#include "core/dsp_constants.h" /* RATE_* */
#include "core/dsp_host.h"

/* the table words of the FPM_* configuration blocks are 0 on x86-64 (src/v22/v22_tables.c, V22TBL): the creator's real pointer is stored right after the
 * FPM creator returns */
#if UINTPTR_MAX == 0xffffffffu
#define V22_FIXTBL(dst, src) ((void)0)
#else
#define V22_FIXTBL(dst, src) ((dst) = (src))
#endif

/* ---- tables ---- */
/* TONEv22_CFG / TONEv22INIT_CFG: the nine-word FPM_TONE configuration blocks of the engine's tone generator and of the temporary generator that computes the
 * coefficient buffers; word 4 is replaced by the ToneLPF table word of FPM_TONE_CFG_raw by the creator */
static const int TONEv22_CFG[9] = {0x2d430834, 0x0ba50000, 0x00010148, 0x00007ae1, 0, 0x35, 0, 0x28, 0};
static const int TONEv22INIT_CFG[9] = {0x2d430834, 0x0ba50000, 0x00010148, 0x00007ae1, 0, 0x35, 0, 0x28, 0};
/* V22DiconnectThreshTable: candidate energy thresholds of the receive path; only entry 3 (150) is used, by the creator (V22EngCfg::thr) */
static const short V22DiconnectThreshTable[8] = {75, 95, 119, 150, 168, 174, 212, 238};
/* v22_status_map: protocol state (V22Protocol) -> status word of V22_status() */
static const short v22_status_map[7] = {3, 0, 1, 2, 7, 8, 5};
/* V22_PROTOCOL: the protocol handlers, indexed by V22Tones::state */
static const V22ProtoFn V22_PROTOCOL[7] = {v22_data, v22_originate, v22_answer, v22_local_loop, v22_org_rmloop2, v22_ans_rmloop2, v22_retrain};
/* the 16-bit working copies of V22FP_modem; shared by all engine instances.  rx_out_internal has 160 entries because V22FP_modem copies as many entries as
 * the rx count the handler left, which is the input count (160) when a handler does not set it (answer stage 14 / originate stages 2, 7 / answer stages 2,
 * 5, 6, 7); the entries above 100 are never used (the state byte is non-zero then and the count is zeroed) */
static short tx_in_internal[100];
static short rx_in_internal[160];
static short rx_out_internal[160];

/**
 * @brief Create or re-initialise a V.22 / V.22bis / Bell 212A engine.
 * @param eng  an existing engine to re-initialise in place (its buffers and objects are reused), or NULL to allocate the three blocks and the work buffers
 * @param cfg  seven ints: [0] mode (0 originate, 1 answer, 2 loop; other values keep the default 1), [1] protocol (0 = V.22bis 2400, 1 or 2 = 1200; other
 *             values keep the default 2400), [2] handshake timeout in ms, [3] bit 0 -> b11 bit 2 (start with the answer tone), [4] carrier loss time in ms
 *             (stored as 16 bits), [5] bit 0 -> b11 bit 3 (start in the data state), [6] bit 0 -> b11 bit 1 (retrain on poor signal quality)
 * @return the engine (state V22_MSG_HANDSHAKE); with `eng == NULL` a new engine to be released with V22FP_delete()
 *
 * The engine starts in the protocol state for the mode (originate / answer / local loop; the data state when cfg[5] is set) with the default
 * configuration V22_CFG patched by `cfg`.  The pulse-shaping, receive and equaliser coefficient buffers are computed with a temporary tone generator;
 * with debug level above 1 the line "V22FP create, built ..." is printed.
 * @warning The allocation results are not checked, and `cfg` must point to seven ints.
 */
V22Engine* V22FP_create(V22Engine* eng, const int* cfg)
{
  int cfgMode, cfgRate, cfgTimeout, cfgB11Bit2, cfgLossMs, cfgB11Bit3, cfgB11Bit1;
  short mode, rate, s14;
  unsigned b11, b10;
  int allocate = 0;
  int i;
  V22Core* core;
  V22Tones* tones;
  FpmTone* tmpTone;
  FpmTone* oldTone;
  FpmMtd *oldS1, *oldM1, *oldM2;
  V22Iir* iir;
  short *ppsI, *ppsQ, *mrfC, *fseI, *fseQ, *outI, *outQ, *tapsI, *tapsQ;
  V22SdmCfg sdmCfg;
  V22PpsCfg ppsCfg;
  V22MrfCfg mrfCfg;
  V22FseCfg fseCfg;
  int toneCfg[9];
  int mtdCfg[3];
  short cosSample, sinSample;

  if (1 < dsplibs_debug_level)
    dsplibs_debug_printf("V22FP create, built %s %s\n", "Sep 22 2005", "15:48:09");

  /* the engine configuration: V22_CFG patched by the 7-int argument (read into scalars first, so the code does not depend on the aliasing rules) */
  cfgMode = cfg[0];
  cfgRate = cfg[1];
  cfgTimeout = cfg[2];
  cfgB11Bit2 = cfg[3];
  cfgLossMs = cfg[4];
  cfgB11Bit3 = cfg[5];
  cfgB11Bit1 = cfg[6];
  mode = V22_CFG.mode;
  rate = V22_CFG.rate;
  s14 = V22_CFG.loopChannel;
  b11 = V22_CFG.b11;
  b10 = V22_CFG.b10;
  if (cfgMode == 0)
    mode = 0;
  else if (cfgMode == 1)
    mode = 1;
  else if (cfgMode == 2)
  {
    mode = 2;
    s14 = 1;
  }
  if (cfgRate == 0)
    rate = RATE_2400;
  else if (cfgRate == 1 || cfgRate == 2)
    rate = RATE_1200;
  b11 = ((b11 & 0xf3u) | ((unsigned)(cfgB11Bit2 & 1) << 2) | ((unsigned)(cfgB11Bit3 & 1) << 3)) & 0xffu;
  b11 = ((b11 & 0xfdu) | ((unsigned)(cfgB11Bit1 & 1) << 1)) & 0xffu;

  if (eng == NULL)
  { /* allocate the blocks (the results are not checked) */
    eng = (V22Engine*)sysdep_malloc(sizeof(V22Engine));
    core = (V22Core*)sysdep_malloc(sizeof(V22Core));
    eng->core = core;
    tones = (V22Tones*)sysdep_malloc(sizeof(V22Tones));
    eng->tones = tones;
    tones->tone = NULL;
    tones->mtd1 = NULL;
    tones->mtd2 = NULL;
    tones->mtdS1 = NULL;
    tones->iir = (V22Iir*)sysdep_malloc(0x20);
    core->rxBuf = (short*)sysdep_malloc(0x154);
    core->circ.buf = (unsigned short*)sysdep_malloc(0x18);
    core->ppsCoefI = (short*)sysdep_malloc(0xf0);
    core->ppsCoefQ = (short*)sysdep_malloc(0xf0);
    core->mrfCoefs = (short*)sysdep_malloc(0x21c);
    core->fseCoefI = (short*)sysdep_malloc(0x62);
    core->fseCoefQ = (short*)sysdep_malloc(0x62);
    allocate = 1;
  }

  core = eng->core;
  tones = eng->tones;

  /* the old tone generator / detector objects (NULL after the allocation above) and the engine's work buffers are read into locals first: the code must
   * not depend on how the compiler orders these pointer loads against the short / int stores around them */
  oldTone = tones->tone;
  oldS1 = tones->mtdS1;
  oldM1 = tones->mtd1;
  oldM2 = tones->mtd2;
  iir = tones->iir;
  ppsI = core->ppsCoefI;
  ppsQ = core->ppsCoefQ;
  mrfC = core->mrfCoefs;
  fseI = core->fseCoefI;
  fseQ = core->fseCoefQ;

  /* configuration, start state and the gates of the chain */
  eng->cfg.mode = mode;
  eng->cfg.rate = rate;
  eng->cfg.rate2 = rate;
  eng->cfg.s6 = V22_CFG.s6;
  eng->cfg.handshakeTimeoutMs = cfgTimeout;
  eng->cfg.gain = V22_CFG.gain;
  eng->cfg.b10 = (unsigned char)b10;
  eng->cfg.b11 = (unsigned char)b11;
  eng->cfg.s12 = V22_CFG.s12;
  eng->cfg.loopChannel = s14;
  eng->cfg.thr = V22DiconnectThreshTable[3];
  eng->cfg.lossMs = (unsigned short)cfgLossMs;

  core->scrOn = b10 & 1;
  core->descrOn = (b10 >> 1) & 1;
  core->w0 = 1;
  core->sreGate = 1;
  core->fsePllGate = 1;
  core->w20 = (b10 >> 2) & 1;
  core->fseS18Gate = 1;
  core->adaptGate = 1;
  core->txBis = (short)(rate != RATE_1200);
  core->w14 = 0;
  core->rxBis = (short)(rate != RATE_1200);

  /* protocol state and channel assignment follow the mode */
  if (mode == 0)
  {
    tones->state = V22PROTO_ORIGINATE;
    core->txChannel = 1;
    core->rxChannel = 2;
  }
  else if (mode == 1)
  {
    tones->state = V22PROTO_ANSWER;
    core->txChannel = 2;
    core->rxChannel = 1;
  }
  else
  {
    tones->state = V22PROTO_LOCAL_LOOP;
    core->txChannel = s14;
    core->rxChannel = s14;
  }
  if (b11 & 8)
    tones->state = V22PROTO_DATA;

  /* handshake timers and accumulators */
  tones->timer = 0;
  tones->sigAcc = 0;
  tones->timeout = cfgTimeout;
  tones->onesAcc = 0;
  tones->sc = 0;
  tones->rms = 0;
  tones->rmsThr = 0x2454;
  tones->rmsN = 0;
  tones->shift = 0;
  tones->trained = 0;
  tones->rmsSum = 0;
  tones->lossCnt = 0;

  /* the tone generator and the detectors */
  for (i = 0; i < 9; i++)
    toneCfg[i] = TONEv22_CFG[i];
  toneCfg[4] = FPM_TONE_CFG_raw[4];
  tones->tone = FPM_TONE_create(oldTone, toneCfg);
  mtdCfg[0] = MTDs1_CFG_raw[0];
  mtdCfg[1] = MTDs1_CFG_raw[1];
  mtdCfg[2] = MTDs1_CFG_raw[2];
  tones->mtdS1 = FPM_MTD_create(oldS1, mtdCfg);
  V22_FIXTBL(tones->mtdS1->coef, V22_S1_HC_COEF);
  tones->mtd1 = FPM_MTD_create(oldM1, MTDv22_CFG_raw);
  V22_FIXTBL(tones->mtd1->coef, MTDv22_COEF);
  tones->mtd2 = FPM_MTD_create(oldM2, MTDv22_CFG2_raw);
  V22_FIXTBL(tones->mtd2->coef, MTDv22_COEF2);

  /* the transmit chain: a temporary tone generator (phase increment 0x666 for transmit channel 1, else 0xccc) multiplies the pulse-shaping (PPS)
   * prototype by its cosine and sine, which gives the engine's I and Q coefficient buffers */
  for (i = 0; i < 9; i++)
    toneCfg[i] = TONEv22INIT_CFG[i];
  toneCfg[4] = FPM_TONE_CFG_raw[4];
  tmpTone = FPM_TONE_create(NULL, toneCfg);

  sdmCfg = SDMv22_CFG;
  sdmCfg.s[0] = core->txBis ? 4 : 2;
  FPM_SDM_init(&core->sdm, &sdmCfg);
  FPM_SMC_init(&core->smc, &SMCv22_CFG);

  tmpTone->r1.s[9] = (core->txChannel == 1) ? 0x666 : 0xccc;
  tmpTone->r0.s[1] = 0x4000;
  for (i = 0; i < 120; i++)
  {
    FPM_TONE_generate2(tmpTone, &cosSample, &sinSample, 1);
    ppsI[i] = (short)(((int)cosSample * (int)PPSv22_COFFS[i]) >> 14);
    ppsQ[i] = (short)(((int)PPSv22_COFFS[i] * (int)sinSample) >> 14);
  }

  ppsCfg = PPSv22_CFG;
  ppsCfg.coefI = ppsI;
  ppsCfg.coefQ = ppsQ;
  core->pps.imap = SMCv22_IMAP_1200BPS;
  core->pps.qmap = SMCv22_QMAP_1200BPS;
  V22_PPS_init(&core->pps, &ppsCfg, allocate);
  core->circ.wr = 0;
  core->circ.rd = 0;
  core->circ.lim = 0xc;
  if (core->rxChannel == 2)
    V22IIRFilterInit(iir, IIR_b_coeff, IIR_a_coeff);

  /* the receive chain: the receive filter (MRF) prototype is multiplied by the cosine of the generator (phase increment 0x222); then the MRF, the SRE and the
   * two AGCs are initialised */
  tmpTone->r1.s[9] = 0x222;
  tmpTone->r1.s[8] = 0;
  for (i = 0; i < 270; i++)
  {
    FPM_TONE_generate2(tmpTone, &cosSample, &sinSample, 1);
    mrfC[i] = (short)(((int)MRFv22_COFFS[i] * (int)cosSample) >> 14);
  }
  mrfCfg = V22_MRF_CFG;
  mrfCfg.coefs = mrfC;
  V22_MRF_init(&core->mrf, &mrfCfg, allocate);
  V22_SRE_init(&core->sre, allocate);
  FPM_AGC_init(&core->agcIn, AGCv22_CFG_raw, allocate);
  V22_FIXTBL(core->agcIn.alpha, V22_AGC_DEF_ALPHA);
  V22_FIXTBL(core->agcIn.beta, V22_AGC_DEF_BETA);
  FPM_AGC_init(&core->agc, AGCv22_CFG2_raw, allocate);
  V22_FIXTBL(core->agc.alpha, V22_AGC_DEF_ALPHA);
  V22_FIXTBL(core->agc.beta, V22_AGC_DEF_BETA);

  /* the equaliser: its prototype is multiplied by the cosine and sine of the generator (phase increment 0x2aaa); the SMC state word and the decision
   * callback are set, the descrambler is initialised, and the temporary generator is released */
  tmpTone->r1.s[9] = 0x2aaa;
  tmpTone->r1.s[8] = 0;
  fseCfg.srcI = fseI;
  fseCfg.srcQ = fseQ;
  for (i = 0; i < 49; i++)
  {
    FPM_TONE_generate2(tmpTone, &cosSample, &sinSample, 1);
    fseI[i] = (short)(((int)cosSample * (int)FSEv22_COFFS[i]) >> 14);
    fseQ[i] = (short)(((int)FSEv22_COFFS[i] * (int)sinSample) >> 14);
  }
  core->fse.smc = &core->fseSmc;
  core->fse.decide = FSEv22_decision12;
  V22_FSE_init(&core->fse, &fseCfg, allocate);
  FPM_SDM_init(&core->sdmRx, &sdmCfg);
  FPM_TONE_delete(tmpTone);

  /* the engine's status fields and the pointers to the equaliser's output and tap buffers */
  outI = core->fse.outI;
  outQ = core->fse.outQ;
  tapsI = core->fse.tapsI;
  tapsQ = core->fse.tapsQ;
  eng->state = V22_MSG_HANDSHAKE;
  eng->flags1d = 0x40;
  eng->flags1e = 0;
  eng->b1f = 0;
  eng->fseOutI = outI;
  eng->s34 = 0x31;
  eng->s40 = 0;
  eng->fseOutQ = outQ;
  eng->fseNOut = &core->fse.nOut;
  eng->s4c = 0;
  eng->fseTapsI = tapsI;
  eng->fseTapsQ = tapsQ;
  eng->w38 = 0;
  eng->w3c = 0;
  eng->w44 = 0;
  eng->w48 = 0;
  return eng;
}

/** @brief Release an engine from V22FP_create(): the chain components, the tone generator and detectors, all buffers, then the three blocks. */
void V22FP_delete(V22Engine* eng)
{
  V22Core* core = eng->core;
  V22Tones* tones;

  V22_PPS_free(&core->pps);
  V22_MRF_free(&eng->core->mrf);
  V22_SRE_free(&eng->core->sre);
  V22_FSE_free(&eng->core->fse);
  tones = eng->tones;
  FPM_TONE_delete(tones->tone);
  FPM_MTD_delete(eng->tones->mtd1);
  FPM_MTD_delete(eng->tones->mtdS1);
  FPM_MTD_delete(eng->tones->mtd2);
  sysdep_free(eng->tones->iir);
  sysdep_free(eng->core->circ.buf);
  sysdep_free(eng->core->ppsCoefI);
  sysdep_free(eng->core->ppsCoefQ);
  sysdep_free(eng->core->mrfCoefs);
  sysdep_free(eng->core->fseCoefI);
  sysdep_free(eng->core->fseCoefQ);
  sysdep_free(eng->core->rxBuf);
  sysdep_free(eng->tones);
  sysdep_free(eng->core);
  sysdep_free(eng);
}

/** @brief Diagnostics hook; forwards to V22_FSE_getdiag() and therefore always returns 0.  Not called by the data-pump glue. */
int V22FP_GetDiagnostics(V22Engine* eng, int mode, int* out, int max) { return V22_FSE_getdiag(&eng->core->fse, mode, out, max); }

/**
 * @brief Process one block: the engine's main entry point.
 * @param eng      engine
 * @param txBits   transmit bits, one per int (low bits), `*txCount` of them (at most 100)
 * @param out      receives 160 output samples (the handler output scaled by cfg.gain, Q15)
 * @param in       `*rxCount` input samples (at most 160)
 * @param rxBits   receives the received bits, one per int
 * @param txCount  number of transmit bits available (the handler's own transmit count is not returned)
 * @param rxCount  on entry the input sample count, on return the number of received bits (0 whenever the status byte is non-zero)
 * @return the status dword: byte 0 = V22Engine::state (V22Msg), byte 1 = flags1d, byte 2 = flags1e, byte 3 = b1f; dp_v22.c uses the low byte only
 *
 * The inputs are narrowed into three static 16-bit buffers (the input is shifted right by tones.shift), the handler V22_PROTOCOL[tones.state] runs, and
 * its received bits are copied out.  When the handler reports a connection (states 3, 4) or one of the loop-back results (6, 7, 8) the engine drops to
 * the data state (tones.state = V22PROTO_DATA, sc = 0).  All 160 output samples are scaled by the gain, whatever the handler wrote.
 * @warning `*txCount` must be at most 100 and `*rxCount` at most 160: the static work buffers are not bounds-checked.  The buffers are shared by all
 *          engine instances, so two threads must not call this function at the same time.
 */
unsigned int V22FP_modem(V22Engine* eng, const int* txBits, short* out, const short* in, int* rxBits, const unsigned int* txCount, unsigned int* rxCount)
{
  V22Tones* tones = eng->tones;
  int ntx = (int)*txCount; /* the counts and the engine pointers are cached, so the code does not depend on the aliasing rules */
  int nrx = (int)*rxCount;
  short rx16 = (short)*rxCount;
  short tx16 = (short)*txCount;
  int i;
  int gain;
  int shift = (unsigned short)tones->shift & 31;
  V22ProtoFn fn;
  unsigned st, f1d, f1e, b1f;

  /* clear the per-block status flags, narrow the inputs to the 16-bit work buffers */
  eng->flags1d &= 0xf8;
  eng->flags1e &= 0xfd;
  for (i = 0; i < ntx; i++)
    tx_in_internal[i] = (short)txBits[i];
  for (i = 0; i < nrx; i++)
    rx_in_internal[i] = (short)((int)in[i] >> shift);

  /* run the protocol handler of the current state */
  fn = V22_PROTOCOL[tones->state];
  fn(eng, tx_in_internal, out, rx_in_internal, rx_out_internal, &tx16, &rx16);
  st = eng->state; /* the status bytes are read once after the handler: the short stores below cannot change them */
  f1d = eng->flags1d;
  f1e = eng->flags1e;
  b1f = eng->b1f;
  nrx = (unsigned short)rx16;
  *rxCount = (unsigned int)nrx;

  /* connected (3, 4) or a loop-back result (6, 7, 8): continue in the data state */
  if ((unsigned char)(st - 3) <= 1)
  {
    tones->sc = 0;
    tones->state = V22PROTO_DATA;
  }
  if ((unsigned char)(st - 6) <= 2)
  {
    tones->sc = 0;
    tones->state = V22PROTO_DATA;
  }

  /* copy the received bits out and scale the output */
  for (i = 0; i < nrx; i++)
    rxBits[i] = (unsigned short)rx_out_internal[i];
  gain = eng->cfg.gain;
  for (i = 0; i < 160; i++)
    out[i] = (short)((int)((unsigned int)(int)out[i] * (unsigned int)gain) >> 15); /* a 32-bit wrapping product */
  if (st != 0)
    *rxCount = 0;
  return (unsigned int)st | ((unsigned int)f1d << 8) | ((unsigned int)f1e << 16) | ((unsigned int)b1f << 24);
}

/**
 * @brief Apply a control block from the host: scrambler / descrambler switches, retrain and remote-loop requests.  Not called by the data-pump glue.
 * @param eng  engine
 * @param ctl  control block; only bytes 0xc and 0xd are read.  Byte 0xc: bit 0 scrambler on, bit 1 descrambler on, bit 2 -> core.w20 (TxClockSync),
 *             bit 3 set = receive AGC inactive, bit 7 -> cfg.b11 bit 1 (retrain on poor signal quality).  Byte 0xd: bit 2 = start a local retrain
 *             (protocol state V22PROTO_RETRAIN, stage V22RETRAIN_RESTART), bits 7..6 == 2 = request a remote digital loopback (state V22PROTO_ORG_RMLOOP2)
 * @return 1
 * @warning `ctl` must point to at least 14 bytes.
 */
int V22FP_control(V22Engine* eng, const unsigned char* ctl)
{
  V22Core* core = eng->core;
  V22Tones* tones = eng->tones;
  unsigned ctlByteC = ctl[0xc];
  unsigned ctlByteD;
  unsigned b11 = eng->cfg.b11;

  core->descrOn = (int)((ctlByteC >> 1) & 1);
  core->scrOn = (int)(ctlByteC & 1);
  ctlByteD = ctl[0xd];
  if (ctlByteD & 4)
  {
    tones->state = V22PROTO_RETRAIN;
    tones->sc = V22RETRAIN_RESTART;
    ctlByteD = ctl[0xd];
  }
  if ((ctlByteD >> 6) == 2)
  {
    tones->state = V22PROTO_ORG_RMLOOP2;
    tones->sc = V22ORML_START;
  }
  ctlByteC = ctl[0xc];
  core->w20 = (int)((ctlByteC >> 2) & 1);
  core->agcIn.active = (ctlByteC & 8) == 0;
  eng->cfg.b11 = (unsigned char)((b11 & 0xfdu) | (((ctlByteC >> 7) & 1u) << 1));
  return 1;
}

/**
 * @brief Fill the status block (V22Status) for the host or the handlers.
 * @param eng  engine
 * @param st   receives the status block (V22Status, 0x18 bytes); the words V22_status() only clears are zeroed, the unwritten ones are left alone
 * @return 1
 *
 * state = v22_status_map[protocol state]; transmit and receive rate 2400 or 1200; quality = 0x800 minus the equaliser error power (at 1200 bit/s the
 * error power is first scaled by 0x143c / 2^14); the flag bytes as described at V22Status.  v22_originate / v22_answer / v22_local_loop /
 * v22_data read the flag bytes.
 */
int V22_status(V22Engine* eng, V22Status* st)
{
  V22Core* core = eng->core;
  int errScaled;
  unsigned char flagsByte;
  /* every engine word is read before the first store into the caller's buffer (the code must not depend on the aliasing of `st`) */
  int scrOn = core->scrOn, descrOn = core->descrOn, w20 = core->w20, w0 = core->w0, wc = core->fseS18Gate, w10 = core->adaptGate;
  short txBis = core->txBis, rxBis = core->rxBis, errPow = core->fse.errPow, tstate = eng->tones->state;
  unsigned b11 = eng->cfg.b11;

  /* state, rates and quality */
  st->state = v22_status_map[tstate];
  st->txRate = txBis ? RATE_2400 : RATE_1200;
  st->rxRate = rxBis ? RATE_2400 : RATE_1200;
  /* at the receive rate 2400 the error power is used as it is, at 1200 it is scaled by 0x143c / 2^14 */
  errScaled = rxBis ? (int)errPow : ((int)errPow * 0x143c) >> 14;
  st->w08 = 0;
  st->quality = (short)(0x800 - errScaled);
  st->w0a = 0;
  st->w10 = 0;
  st->w12 = 0;

  /* the flag bytes: bits 0 .. 2 and 7 from the switches and cfg.b11, bits 3 .. 5 set when the gate words are zero, bit 6 always set */
  flagsByte = st->flags;
  flagsByte = (unsigned char)((flagsByte & 0xfeu) | (unsigned)(scrOn & 1));
  flagsByte = (unsigned char)((flagsByte & 0xfdu) | ((unsigned)(descrOn & 1) << 1));
  flagsByte = (unsigned char)((flagsByte & 0xfbu) | ((unsigned)(w20 & 1) << 2));
  flagsByte = (unsigned char)((flagsByte & 0xf7u) | ((unsigned)(w0 == 0) << 3));
  flagsByte = (unsigned char)((flagsByte & 0xefu) | ((unsigned)(wc == 0) << 4));
  flagsByte = (unsigned char)((flagsByte & 0xdfu) | ((unsigned)(w10 == 0) << 5));
  flagsByte |= 0x40;
  flagsByte = (unsigned char)((flagsByte & 0x7fu) | (((b11 >> 1) & 1u) << 7));
  st->flags = flagsByte;
  st->flags2 = (unsigned char)((st->flags2 & 0xfeu) | ((b11 >> 2) & 1u));
  return 1;
}

/**
 * @brief Fill `*count` transmit symbols (one per short) with a fixed test pattern.
 * @param out      receives the symbols
 * @param count    number of symbols to produce
 * @param pattern  0: alternating 0, 3 (the S1 pattern); 1: all 3 (ones); 2: all 0xf; 3: all 2; 4: all 0xa; any other value writes nothing
 * @warning Pattern 0 writes pairs and counts down by 2 in 16 bits: an odd count never reaches 0, so the loop writes past the end of `out` until the process
 *          faults (read from the code; not run).  The callers pass the even counts of the engine (12 per block).
 */
void MakeTxData(short* out, const short* count, short pattern)
{
  short remaining = *count;
  switch (pattern)
  {
  case 0:
    if (remaining == 0)
      return;
    do
    {
      out[0] = 0;
      out[1] = 3;
      out += 2;
      remaining = (short)(remaining - 2);
    } while (remaining != 0);
    return;
  case 1:
  case 2:
  case 3:
  case 4:
  {
    short fillBits = pattern == 1 ? 3 : pattern == 2 ? 0xf : pattern == 3 ? 2 : 0xa;
    remaining = (short)(remaining - 1);
    while (remaining != -1)
    {
      *out++ = fillBits;
      remaining = (short)(remaining - 1);
    }
    return;
  }
  default:
    return;
  }
}

/**
 * @brief Training check at 1200 bit/s.
 * @param data   received symbols
 * @param count  pointer to the number of symbols
 * @return 1 when every received symbol is 3 (all ones), else 0; 1 for an empty block
 */
unsigned int RxTrained1200(const short* data, const unsigned short* count)
{
  unsigned int nBits = *count;
  int run = 0;
  if (run < (int)nBits && data[0] == 3)
  {
    do
    {
      run = (short)(run + 1);
      if (run >= (int)nBits)
        break;
    } while (data[run] == 3);
  }
  return run == (int)nBits;
}

/**
 * @brief Training check at 2400 bit/s.
 * @param data   received symbols
 * @param count  pointer to the number of symbols
 * @return 1 when the block ends with a run of more than 7 symbols equal to 0xf (counted back from the last symbol), else 0
 */
unsigned int RxTrained2400(const short* data, const unsigned short* count)
{
  int nBits = *count;
  int run = 0;
  int pos;
  if (!(0 < nBits))
    return 0;
  pos = (short)(nBits - 1);
  if (data[pos] != 0xf)
    return 0;
  pos = (short)(pos - 1);
  for (;;)
  {
    int curPos;
    run = (short)(run + 1);
    if (run >= nBits)
      break;
    curPos = pos;
    pos = (short)(pos - 1);
    if (data[curPos] != 0xf)
      break;
  }
  return (short)run > 7;
}

/** @brief Transmit silence: 160 zero samples in `out`, `*count` = 0xa0.  `eng` and the second argument are unused. */
void TxNOP(V22Engine* eng, void* unused, short* out, short* count)
{
  int i;
  (void)eng;
  (void)unused;
  for (i = 0; i < 160; i++)
    out[i] = 0;
  *count = 0xa0;
}

/** @brief Replace the received bits by 12 symbols of 0xf (all ones) and set `*count` = 12.  Used while no valid data is being received.  `eng` and the
 *         second argument are unused. */
void RxClampV22(V22Engine* eng, void* unused, short* out, short* count)
{
  int i;
  (void)eng;
  (void)unused;
  for (i = 0; i < 12; i++)
    out[i] = 0xf;
  *count = 0xc;
}

/** @brief Advance the handshake timer by one block (20 ms) and return it, in ms.  Called once per block by the handlers. */
int ReadGTimer(V22Engine* eng)
{
  V22Tones* tones = eng->tones;
  tones->timer = (int)((unsigned int)tones->timer + 0x14u);
  return tones->timer;
}

/**
 * @brief Switch the transmit chain to 1200 (rate 0) or 2400 bit/s (rate 1); any other value does nothing.
 *
 * Reprograms the scrambler (2 or 4 bits per symbol, shift register cleared), the encoder's mask and shift, and the PPS symbol maps, and sets txBis.
 */
void SetTxRate(V22Engine* eng, short rate)
{
  V22Core* core;
  unsigned t1, t2;
  if (rate == 0)
  {
    core = eng->core;
    t1 = core->sdm.tap1 & 0xffffu;
    t2 = core->sdm.tap2 & 0xffffu;
    core->sdm.n = 2;
    core->sdm.mask = 3;
    core->sdm.sh1 = (short)(t1 - 2);
    core->sdm.sh2 = (short)(t2 - 2);
    core->sdm.nmask = (unsigned int)-4;
    core->sdm.state = 0;
    core->smc.cfg.mask2 = 0;
    core->smc.cfg.shift = 0;
    core->pps.imap = SMCv22_IMAP_1200BPS;
    core->pps.qmap = SMCv22_QMAP_1200BPS;
    core->txBis = 0;
  }
  else if (rate == 1)
  {
    core = eng->core;
    t1 = core->sdm.tap1 & 0xffffu;
    t2 = core->sdm.tap2 & 0xffffu;
    core->sdm.n = 4;
    core->sdm.mask = 0xf;
    core->sdm.sh1 = (short)(t1 - 4);
    core->sdm.sh2 = (short)(t2 - 4);
    core->sdm.nmask = (unsigned int)-16;
    core->sdm.state = 0;
    core->smc.cfg.mask2 = 3;
    core->smc.cfg.shift = 2;
    core->pps.imap = SMCv22_IMAP_2400BPS;
    core->pps.qmap = SMCv22_QMAP_2400BPS;
    core->txBis = 1;
  }
}

/**
 * @brief Switch the receive chain to 1200 (rate 0) or 2400 bit/s (rate 1); any other value does nothing.
 *
 * Reprograms the descrambler (2 or 4 bits per symbol, shift register cleared), sets rxBis, and selects the four-point (FSEv22_decision12) or sixteen-point
 * (FSEv22_decision24) decision callback of the equaliser.
 */
void SetRxRate(V22Engine* eng, short rate)
{
  V22Core* core;
  unsigned t1, t2;
  if (rate == 0)
  {
    core = eng->core;
    core->sdmRx.n = 2;
    core->sdmRx.mask = 3;
    core->sdmRx.nmask = (unsigned int)-4;
    core->sdmRx.state = 0;
    t1 = core->sdmRx.tap1 & 0xffffu;
    t2 = core->sdmRx.tap2 & 0xffffu;
    core->rxBis = 0;
    core->sdmRx.sh1 = (short)(t1 - 2);
    core->sdmRx.sh2 = (short)(t2 - 2);
    core->fse.decide = FSEv22_decision12;
  }
  else if (rate == 1)
  {
    core = eng->core;
    core->sdmRx.n = 4;
    core->sdmRx.mask = 0xf;
    core->sdmRx.nmask = (unsigned int)-16;
    core->sdmRx.state = 0;
    t1 = core->sdmRx.tap1 & 0xffffu;
    t2 = core->sdmRx.tap2 & 0xffffu;
    core->rxBis = 1;
    core->sdmRx.sh1 = (short)(t1 - 4);
    core->sdmRx.sh2 = (short)(t2 - 4);
    core->fse.decide = FSEv22_decision24;
  }
}

/** @brief Scramble `n` transmit symbols in place with the transmit scrambler. */
void ScrambleDataV22(V22Engine* eng, unsigned short* buf, unsigned short n) { FPM_SDM_scrambler(&eng->core->sdm, buf, n); }
/** @brief Descramble `n` received symbols in place with the receive descrambler. */
void DescrambleDataV22(V22Engine* eng, unsigned short* buf, unsigned short n) { FPM_SDM_descrambler(&eng->core->sdmRx, buf, n); }

/**
 * @brief Run the transmit chain: encode `n` symbols differentially and pulse-shape them into audio.
 * @param eng   engine
 * @param bits  `n` symbols (already scrambled where the protocol requires it)
 * @param out   receives the output samples
 * @param n     number of symbols
 * @return the number of output samples (16 bit)
 */
unsigned int ModDataV22(V22Engine* eng, const unsigned short* bits, short* out, unsigned short n)
{
  V22Core* core = eng->core;
  unsigned int r;
  FPM_SMC_encoder(&core->smc, &core->circ, bits, n);
  core = eng->core;
  r = V22_PPS_filter(&core->pps, &core->circ, out, n);
  return r & 0xffffu;
}

/**
 * @brief Run the receive chain on one block: (originating side: demodulate), resample, AGC, timing recovery, equalise and decide.
 * @param eng  engine
 * @param in   `n` input samples (modified: used as the timing recovery's output buffer, and filtered in place on the originating side)
 * @param out  receives one decoded symbol (the 2- or 4-bit group) per received symbol
 * @param n    number of input samples (at most 160)
 * @return the number of symbols produced (0 in the data state when the receive energy is below cfg.thr)
 *
 * On the originating side (core.rxChannel == 2) the block is first mixed with the demodulation tone and filtered by the IIR section.  The MRF output is
 * kept in core.rxBuf (the call-setup code runs the S1 detector on it).
 * @warning `n` must not exceed 160 (the demodulation tone is generated into a 160-sample stack buffer).
 */
unsigned int DemodDataV22(V22Engine* eng, short* in, short* out, unsigned short n)
{
  /* the engine pointers never change; they are read once, so the code does not depend on the aliasing of the short stores below */
  V22Core* core = eng->core;
  V22Tones* tones = eng->tones;
  short* rxBuf = core->rxBuf;
  short tstate = tones->state;
  unsigned int nSym, nIn;
  int agcGate;

  /* originating side: mix down with the demodulation tone and filter */
  if (core->rxChannel == 2)
  {
    short demodTone[160];
    FPM_TONE_generate_demod(tones->tone, demodTone, (short)n);
    V22_iir_filt_demod(in, tones->iir->num, tones->iir->den, tones->iir->x, tones->iir->y, demodTone);
  }

  /* sample-rate conversion; in the data state a weak signal means no carrier */
  nIn = V22_MRF_filter(&core->mrf, in, rxBuf, (short)n) & 0xffffu;
  core->rxCount = (short)nIn;
  if (tstate == V22PROTO_DATA)
  {
    short rms = FPM_rms(rxBuf, (short)nIn);
    if ((short)eng->cfg.thr > rms)
    {
      core->sre.wantTrack = 0;
      return 0;
    }
  }

  /* AGC and symbol timing recovery */
  FPM_AGC_agc(&core->agcIn, rxBuf, (unsigned short)nIn);
  core->sre.pllEnable = core->sreGate & core->agcIn.active;
  nSym = V22_SRE_recover(&core->sre, rxBuf, in, (short)nIn) & 0xffffu;

  /* equaliser gates: the carrier loop follows its gate; the s18 flag and the adaptation are enabled only after the timing loop has left its settling mode
   * and has reached fine tracking (pllFlagC == 0), and while the receive AGC is active */
  agcGate = 0;
  if (core->sre.pllFlagC == 0)
    agcGate = core->agcIn.active & 1;
  core->fse.pllEnable = core->fsePllGate;
  if (core->sre.mode != 0)
  {
    core->fse.s18 = core->fseS18Gate & agcGate;
    core->fse.adaptEnable = core->adaptGate & agcGate;
  }
  else
  {
    core->fse.s18 = 0;
    core->fse.adaptEnable = 0;
  }
  V22_FSE_receive(&core->fse, in, out, (short)nSym);
  return (unsigned short)core->fse.nOut;
}

/** @brief Reset the receive side for a retrain: re-initialise the timing recovery and the equaliser (their buffers are kept; the equaliser taps are reloaded
 *         from its coefficient sources). */
void ResetRx(V22Engine* eng)
{
  V22Core* core = eng->core;
  V22FseCfg fc;
  V22_SRE_init(&core->sre, 0);
  core = eng->core;
  fc.srcI = core->fse.srcI;
  fc.srcQ = core->fse.srcQ;
  V22_FSE_init(&core->fse, &fc, 0);
}

/**
 * @brief Select the equaliser adaptation mode.
 * @param mode  1: adaptation off; 2: adaptation allowed, step index 0 (fast); 3: adaptation on, step index 1 (slow); any other value does nothing
 */
void SetAdaptEqV22(V22Engine* eng, unsigned short mode)
{
  V22Core* core;
  if (mode == 2)
  {
    core = eng->core;
    core->adaptGate = 1;
    core->fse.mu = 0;
  }
  else if (mode == 1)
  {
    core = eng->core;
    core->adaptGate = 0;
  }
  else if (mode == 3)
  {
    core = eng->core;
    core->adaptGate = 1;
    core->fse.mu = 1;
    core->fse.adaptEnable = 1;
  }
}

/** @brief Slave the transmit symbol clock to the received timing: the pulse shaper's phase step per output becomes 3 times the timing recovery's integrator. */
void TxClockSync(V22Engine* eng)
{
  V22Core* core = eng->core;
  core->pps.cfg.c0.s[0] = (unsigned short)(core->sre.integ * 3);
}

/** @brief Carrier detect: non-zero while the timing recovery tracks a signal (sre.wantTrack). */
int CarrierDetect(V22Engine* eng) { return eng->core->sre.wantTrack; }
/** @brief Signal detect: the receive AGC's detect flag. */
int SignalDetect(V22Engine* eng) { return eng->core->agcIn.detect; }
/** @brief Signal quality: 0x8000 minus the averaged equaliser error power, as an unsigned 16-bit value (higher is better). */
unsigned int GetSignalQuality(V22Engine* eng) { return (unsigned short)(0xffff8000u - (unsigned short)eng->core->fse.errPow); }
/** @brief Whether the transmit scrambler is enabled. */
int ScramblerOn(V22Engine* eng) { return eng->core->scrOn; }
/** @brief Whether the receive descrambler is enabled. */
int DescramblerOn(V22Engine* eng) { return eng->core->descrOn; }
