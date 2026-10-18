/**
 * @file v21_b103.c
 * @brief Bell 103 / V.21 300 bit/s FSK modem engine: modulator, demodulator and the half-duplex handshake state machine.
 *
 * The engine object (B103Obj, v21_b103.h) is created by B103FP_create(), run once per block by B103FP_modem() and freed by B103FP_delete(); the host glue
 * (dp_v21_b103.c) is the only caller.  One B103FP_modem() call modulates exactly 6 transmit bits (padded with marks when fewer are supplied) and demodulates
 * the block of received samples it is given.  With the configuration the glue passes, 6 bits are 144 modulator samples (24 per bit, a 7200 Hz sample
 * clock), which the transmit filter converts by 10 / 9 to 160 samples at 8000 Hz; the receive filter converts 160 samples by 3 / 10 to 48 samples, 8 per bit.
 *
 * Signal path.
 *   - Transmit: the FSK modulator FPM_FSM writes into `txBuf`, the matched filter FPM_MRF `mrf0` resamples it into the caller's output.  The frequency
 *     pairs (bit 0 / bit 1, in Hz) are 1070 / 1270 (Bell 103 originate), 2025 / 2225 (Bell 103 answer) and 1180 / 980 (V.21 originate); the answer role
 *     uses the 2025 / 2225 pair for V.21 as well (see the @bug note at B103FP_create()).
 *   - Receive: the samples are first filtered in place by a band-pass correlator (`bpfCoef` / `bpfHist`, answer or caller coefficient set), then
 *     mixed with a local oscillator (`mainTone`), filtered by FPM_MRF `mrf1` and fed through the AGC `agc0` and the FSK demodulator FPM_FSD.  A parallel branch
 *     (`rxBuf` -> AGC `agc1` -> tone detector `secTone`) looks for the far end's carrier; five counts are added to `bundle->cnt` per block with the
 *     carrier and the demodulator is released at 15.
 *
 * Handshake.  The half-duplex block (B103Hdx) holds a mode (local loop, originate, answer) and a sub-state, and two function pointers for the current
 * transmit and receive handler.  The handlers (TxHdxStartB103 ... RxDetMarkB103) do the work of the current phase and call the mode's NextState function
 * (B103LocLoopNextState / B103OriginateNextState / B103AnswerNextState, indexed by `hdx->mode`) when a counter runs out or a carrier appears; it installs
 * the handlers of the next phase and sets the state byte of the status word.  The state byte, as seen by the host glue: 0 data transfer, 1 created,
 * 2 waiting for the far-end tone (sub-state 1), 3 sub-state 2, 4 originate: sending marks before data, 5 error, 6 carrier lost, 7 link established (data
 * handlers installed).  While the state byte is non-zero B103FP_modem() sends marks instead of the host's bits; it returns the state byte to 0 on the call
 * after the flags byte had bit 0 set (the transition to state 7 sets it).
 *
 * Not reentrant: the two 100-entry scratch buffers of B103FP_modem() are file-static and shared by all engine objects, so concurrent calls from several
 * threads would corrupt each other.  TxHdxStartB103 is installed by B103FP_create(); RxHdxStartB103 is not installed by any transition in this file.
 */
#include <stdint.h>
#include "core/dsp_table_handles.h"
#include "fpm/fpm_tables.h"
#include "fsk_tables.h"
#include "v21_b103.h"
#include "core/dsp_host.h"
#include "fpm/fpm.h"

/* mainTone[idx] as a short index into the FpmTone image: only 0x12 (generator phase) and 0x13 (phase increment) are used here; they live in the r1 run. */
#define TONE_S(t, idx) ((t)->r1.s[(idx) - 0xa])

/* Scratch buffers of B103FP_modem(): transmit bits (one per short) and received bits.  Shared by all engine objects. */
static short tx_in_internal[100];
static short rx_out_internal[100];

/*
 * The handlers of the state machine.  They are stored in the two handler slots of B103Hdx, interchangeably by the NextState functions, so they all use the
 * common TxHdxFn / RxHdxFn signature.
 */
short TxHdxStartB103(B103Obj* b103, unsigned short* bitsIn, void* samplesOut, unsigned short* bitsLeft);
short TxHdxMarksB103(B103Obj* b103, unsigned short* bitsIn, void* samplesOut, unsigned short* bitsLeft);
short TxHdxDataB103(B103Obj* b103, unsigned short* bitsIn, void* samplesOut, unsigned short* bitsLeft);
short TxHdxSilenceB103(B103Obj* b103, unsigned short* bitsIn, void* samplesOut, unsigned short* bitsLeft);
short RxHdxStartB103(B103Obj* b103, void* samplesIn, unsigned short* bitsOut, unsigned short* samplesLeft);
short RxHdxDataB103(B103Obj* b103, void* samplesIn, unsigned short* bitsOut, unsigned short* samplesLeft);
short RxDetMarkB103(B103Obj* b103, void* samplesIn, unsigned short* bitsOut, unsigned short* samplesLeft);
short ModDataB103(B103Obj* b103, unsigned short* bitsIn, void* samplesOut, unsigned short bitCount);
short DemodDataB103(B103Obj* b103, short* samplesIn, short* bitsOut, unsigned short sampleCount);
short TxNoCarrierB103(B103Obj* b103, unsigned short* bitsIn, void* samplesOut, unsigned short bitCount);
unsigned int CarrierDetectB103(B103Obj* b103);
void B103AnswerNextState(B103Obj* b103);
void B103OriginateNextState(B103Obj* b103);
void B103LocLoopNextState(B103Obj* b103);

/** Per-mode transition functions, indexed by B103Hdx::mode (0 local loop, 1 originate, 2 answer). */
static void (*const B103NextState[3])(B103Obj*) = {
    B103LocLoopNextState,
    B103OriginateNextState,
    B103AnswerNextState,
};

/**
 * @brief Build the nine-word FPM_TONE_create() configuration of the far-end carrier detector (`secTone`).
 * @param cfg        nine configuration words to fill
 * @param freqLow16  frequency word of the detector (low half of word 0)
 *
 * Word 0 takes `freqLow16`; words 1, 2 and 3 get the same half-word overrides for both roles; words 4 .. 8 are the default tone block.
 */
static void buildSecondaryToneCfg(int* cfg, int freqLow16)
{
  cfg[0] = (FPM_TONE_CFG_raw[0] & 0xffff0000) | (freqLow16 & 0xffff);
  cfg[1] = (0x55c3 << 16) | (FPM_TONE_CFG_raw[1] & 0xffff);
  cfg[2] = (6 << 16) | (FPM_TONE_CFG_raw[2] & 0xffff);
  cfg[3] = (FPM_TONE_CFG_raw[3] & 0xffff0000) | 0x7c00;
  cfg[4] = FPM_TONE_CFG_raw[4];
  cfg[5] = FPM_TONE_CFG_raw[5];
  cfg[6] = FPM_TONE_CFG_raw[6];
  cfg[7] = FPM_TONE_CFG_raw[7];
  cfg[8] = FPM_TONE_CFG_raw[8];
}

/**
 * @brief Build the seven-word FPM_FSD_init() configuration of the demodulator.
 * @param cfg       seven configuration words to fill
 * @param mode      the engine's role word, cfg[0] of the object: 0 = originate, 1 = answer, anything else = local loop
 * @param cfgWord1  the object's cfg[1] (non-zero = V.21)
 *
 * The filter tables and counts are fixed; only the polarity half of word 4 depends on the role: in the originate role it is 1 for Bell 103 and 0 for V.21,
 * in every other role it keeps the default.
 */
static void buildFsdCfg(int* cfg, int mode, int cfgWord1)
{
  cfg[0] = CFGP(B103_CHAN_INTRP);
  cfg[1] = 0x4000f;
  cfg[2] = CFGP(B103_IIR_LPF);
  cfg[3] = (FPM_FSD_CFG_raw[3] & 0xffff0000) | 3;
  if (mode == 0)
  {
    cfg[4] = (FPM_FSD_CFG_raw[4] & 0xffff0000) | (cfgWord1 == 0 ? 1 : 0);
  }
  else
  {
    cfg[4] = FPM_FSD_CFG_raw[4];
  }
  cfg[5] = FPM_FSD_CFG_raw[5];
  cfg[6] = FPM_FSD_CFG_raw[6];
}

/**
 * @brief Create a Bell 103 / V.21 engine, or re-initialise an existing one.
 * @param b103   existing object to re-initialise (its blocks are reused), or NULL to allocate the object and its blocks
 * @param cfgIn  seven configuration words, or NULL for the default block B103_CFG_raw
 * @return the engine object
 *
 * Configuration words (cfg[i]):
 *   - [0] role: 0 = originate, 1 = answer, anything else = local loop (the default block selects local loop);
 *   - [1] non-zero = V.21 frequency plan, zero = Bell 103;
 *   - [2] used only by the local-loop role: zero selects the originate frequency pair, non-zero the answer pair;
 *   - [3] sets the far-end tone wait: `waitLimit` = max(cfg[3] / 20, 700) blocks;
 *   - [4] originate role: zero skips the handshake and starts sending marks and data at once (sub-state 3), non-zero runs the handshake;
 *   - [5] stored, not read;
 *   - [6] amplitude of the modulator (low 16 bits).
 *
 * Builds the half-duplex block, the local-oscillator tone object, the far-end carrier detector (originate and answer only), and the signal bundle
 * (matched filters, AGCs, FSK modulator and demodulator, multi-tone detector, band-pass correlator).  Finally the status word is set to state 1, flags 0x40.
 * @warning A new object takes its blocks from sysdep_malloc() without checking the result.  The local-loop role (or an invalid role) leaves
 *          `bpfCoef` NULL with `bpfLen` 0, so B103FP_modem() then dereferences a NULL coefficient pointer as soon as it is given received samples; the
 *          `rxFn` handler is also not installed before the first call in that role.  The originate role with cfg[4] == 0 leaves the local-oscillator
 *          increment at its default also for V.21.  The glue always selects originate or answer with cfg[4] = 1.
 * @bug   The answer role selects the modulator pair 2025 / 2225 Hz (Bell 103 answer) whether cfg[1] selects Bell 103 or V.21; only the receive side
 *        (detector, oscillator) follows cfg[1].  A V.21 answerer therefore transmits on Bell 103 frequencies (read from the code, not run).
 */
B103Obj* B103FP_create(B103Obj* b103, int* cfgIn)
{
  B103Hdx* hdx;
  B103Bundle* bundle;
  FpmTone* mainTone;
  int mode, cfgWord1, cfgWord4;
  int freshBundle;
  int fsmCfg[2];
  int fsdCfg[7];
  int mrfCfg[4];

  if (b103 == 0)
  {
    b103 = (B103Obj*)sysdep_malloc(sizeof(B103Obj));
    B103_PSTORE(b103, hdx, 0x14, 0);
    B103_PSTORE(b103, bundle, 0x15, 0);
  }

  /* configuration words */
  if (cfgIn == 0)
  {
    b103->cfg[0] = B103_CFG_raw[0];
    b103->cfg[1] = B103_CFG_raw[1];
    b103->cfg[2] = B103_CFG_raw[2];
    b103->cfg[3] = B103_CFG_raw[3];
    b103->cfg[4] = B103_CFG_raw[4];
    b103->cfg[5] = B103_CFG_raw[5];
    b103->cfg[6] = B103_CFG_raw[6];
  }
  else
  {
    b103->cfg[0] = cfgIn[0];
    b103->cfg[1] = cfgIn[1];
    b103->cfg[2] = cfgIn[2];
    b103->cfg[3] = cfgIn[3];
    b103->cfg[4] = cfgIn[4];
    b103->cfg[5] = cfgIn[5];
    b103->cfg[6] = cfgIn[6];
  }

  /* half-duplex block: counters and handlers cleared, wait limit from cfg[3] */
  if (B103_PLOAD(B103Hdx*, b103, hdx, 0x14) == 0)
  {
    B103_PSTORE(b103, hdx, 0x14, sysdep_malloc(sizeof(B103Hdx)));
  }
  hdx = B103_PLOAD(B103Hdx*, b103, hdx, 0x14);

  {
    int threshold = b103->cfg[3] / 20;

    if (threshold < 700)
      threshold = 700;
    hdx->waitLimit = (short)threshold;
  }
  hdx->txCount = 0;
  hdx->rxCount = 0;
  hdx->subState = 0;
  hdx->txFn = TxHdxStartB103;
  hdx->w18 = 0;
  hdx->secTone = 0;
  hdx->mainTone = 0;

  /* local oscillator of the demodulator: a default tone object, phase 0, phase increment set per role below */
  {
    int mainCfg[9];
    int i;

    for (i = 0; i < 9; i++)
      mainCfg[i] = FPM_TONE_CFG_raw[i];
    mainTone = FPM_TONE_create(0, mainCfg);
  }
  hdx->mainTone = mainTone;
  TONE_S(mainTone, 0x13) = 0x159a;
  TONE_S(mainTone, 0x12) = 0;

  /* role: handshake mode, far-end carrier detector, oscillator increment */
  cfgWord1 = b103->cfg[1];
  mode = b103->cfg[0];
  if (mode == 0)
  {
    /* originate */
    int secCfg[9];

    hdx->mode = 1;
    buildSecondaryToneCfg(secCfg, cfgWord1 == 0 ? 0x8b1 : 0x672);
    hdx->secTone = FPM_TONE_create(0, secCfg);

    cfgWord4 = b103->cfg[4];
    if (cfgWord4 != 0)
    {
      hdx->w18 = 1;
      if (cfgWord1 != 0)
      {
        TONE_S(mainTone, 0x13) = 0xf9a;
      }
    }
    else
    {
      /* no handshake: start in the "sending marks" phase with the data handlers installed */
      hdx->txCount = 0x28;
      hdx->txFn = (TxHdxFn)TxHdxMarksB103;
      hdx->rxFn = (RxHdxFn)RxHdxDataB103;
      hdx->subState = 3;
      b103->st.b[0] = 4;
      b103->st.b[1] = (b103->st.b[1] & 0xbf) | 0x34;
    }
  }
  else if (mode == 1)
  {
    /* answer */
    int secCfg[9];

    hdx->mode = 2;
    hdx->w18 = 1;
    buildSecondaryToneCfg(secCfg, cfgWord1 == 0 ? 0x4f6 : 0x49c);
    TONE_S(mainTone, 0x13) = cfgWord1 == 0 ? 0x652 : 0x4e2;
    hdx->secTone = FPM_TONE_create(0, secCfg);
  }
  else
  {
    /* local loop (the default block) or any other value */
    hdx->mode = 0;
  }

  /* signal bundle: a new one is cleared and gets its buffers; an existing one is re-initialised */
  freshBundle = 0;
  if (B103_PLOAD(B103Bundle*, b103, bundle, 0x15) == 0)
  {
    bundle = (B103Bundle*)sysdep_malloc(sizeof(B103Bundle));
    B103_PSTORE(b103, bundle, 0x15, bundle);
    sysdep_memset(bundle, 0, sizeof(B103Bundle));
    bundle->txBuf = (short*)sysdep_malloc(0x144);
    bundle->rxBuf = (short*)sysdep_malloc(0x144);
    bundle->bpfHist = (short*)sysdep_malloc(mode == 1 ? 0x68 : 0x54);
    sysdep_memset(bundle->txBuf, 0, 0x144);
    bundle->mtd = 0;
    freshBundle = 1;
  }
  else
  {
    bundle = B103_PLOAD(B103Bundle*, b103, bundle, 0x15);
  }

  /* matched filters: transmit 10 phases, step 9, 27 taps per phase; receive 3 phases, step 10, 30 taps per phase */
  mrfCfg[0] = 0x9000a;
  mrfCfg[1] = CFGP(B103_MRF_FILT_TX);
  mrfCfg[2] = 0x10e;
  mrfCfg[3] = 0;
  FPM_MRF_init(&bundle->mrf0, mrfCfg, freshBundle);

  mrfCfg[0] = 0xa0003;
  mrfCfg[1] = CFGP(B103_MRF_FILT_RX);
  mrfCfg[2] = 0x5a;
  /* mrfCfg[3] is deliberately left at 0 */
  FPM_MRF_init(&bundle->mrf1, mrfCfg, freshBundle);

  /* AGCs */
  FPM_AGC_init(&bundle->agc0, AGCb103_CFG_raw, freshBundle);
  FPM_AGC_init(&bundle->agc1, AGCb103_CFG_raw, freshBundle);

  bundle->agc1.a.s[5] = 0x28;
  bundle->agcActive = 1;
  bundle->agcDetect = 0;
  bundle->toneSeen = 0;

  /* modulator frequencies and band-pass correlator of the role */
  if (mode == 1)
  {
    fsmCfg[0] = 0x8b107e9;
    bundle->bpfCoef = B103_BPF_ANSWER;
    bundle->bpfIdx = 0;
    bundle->bpfLen = 0x32;
    sysdep_memset(bundle->bpfHist, 0, 100);
  }
  else if (mode == 0)
  {
    fsmCfg[0] = cfgWord1 == 0 ? 0x4f6042e : 0x3d4049c;
    bundle->bpfCoef = B103_BPF_CALLER;
    bundle->bpfIdx = 0;
    bundle->bpfLen = 0x28;
    sysdep_memset(bundle->bpfHist, 0, 0x50);
  }
  else
  {
    /* local loop / invalid role: no correlator set up (the bundle was cleared, so bpfCoef is NULL and bpfLen 0) */
    fsmCfg[0] = b103->cfg[2] == 0 ? 0x4f6042e : 0x8b107e9;
    if (mode != 2)
    {
      b103->st.b[1] |= 2;
      b103->st.b[0] = 5;
    }
  }
  fsmCfg[1] = ((b103->cfg[6] & 0xffff) << 16) | 0x18;
  FPM_FSM_init(&bundle->fsm, fsmCfg);

  /* demodulator and multi-tone detector */
  buildFsdCfg(fsdCfg, mode, cfgWord1);
  FPM_FSD_init(&bundle->fsd, fsdCfg, freshBundle);

  {
    int mtdCfg[3];

    mtdCfg[0] = CFGP(MTDb103_COEF);
    mtdCfg[1] = (int)((0x3666u << 16) | 2u);
    mtdCfg[2] = (int)(((unsigned int)MTDb103Org_CFG_raw[2] & 0xffff0000u) | 2u);
    bundle->mtd = FPM_MTD_create(bundle->mtd, mtdCfg);
  }

  /*
   * The status word is cleared as a whole and then set to state 1, flags 0x40: whatever state and flags the role branches above stored are overwritten.
   */
  bundle->cnt = 0;
  B103_W(b103, 7) = 0;
  b103->st.w = 0;
  b103->st.b[1] = 0x40;
  b103->st.b[0] = 1;
  b103->s34 = 0;

  B103_PSTORE(b103, fsdOutBuf, 8, (void*)bundle->fsd.out);
  b103->w24 = 0;
  B103_PSTORE(b103, fsdOutCount, 0xa, &bundle->fsd.outCount);
  b103->w2c = 0;
  b103->w30 = 0;
  b103->w38 = 0;
  b103->w3c = 0;
  b103->s40 = 0;
  b103->w44 = 0;
  b103->w48 = 0;
  b103->s4c = 0;

  if (1 < dsplibs_debug_level)
  {
    dsplibs_debug_printf("B103FP version: %s %s\n", "Sep 22 2005", "15:48:11");
  }
  return b103;
}

/**
 * @brief Free an engine object with all its blocks and sub-objects.
 * @param b103  engine created by B103FP_create()
 */
void B103FP_delete(B103Obj* b103)
{
  B103Bundle* bundle = b103->bundle;
  B103Hdx* hdx = b103->hdx;
  FpmTone* secTone;

  /* the signal bundle */
  FPM_FSM_delete(&bundle->fsm);
  FPM_MTD_delete(bundle->mtd);
  FPM_FSD_free(&bundle->fsd);
  FPM_MRF_free(&bundle->mrf1);
  FPM_MRF_free(&bundle->mrf0);
  sysdep_free(FPM_M(void*, bundle, txBuf));
  sysdep_free(FPM_M(void*, bundle, rxBuf));
  sysdep_free(FPM_M(void*, bundle, bpfHist));
  sysdep_free(bundle);

  /* the half-duplex block and its tone objects */
  secTone = hdx->secTone;
  if (secTone != 0)
  {
    FPM_TONE_delete(secTone);
  }
  {
    FpmTone* mainTone = hdx->mainTone;

    if (mainTone != 0)
    {
      FPM_TONE_delete(mainTone);
      sysdep_free(hdx);
      sysdep_free(b103);
      return;
    }
  }
  sysdep_free(hdx);
  sysdep_free(b103);
}

/**
 * @brief Run the modem for one block: modulate 6 transmit bits, demodulate the received samples.
 * @param b103        engine
 * @param txBits      transmit bits, one per int (bit 0)
 * @param txOut       transmit samples are written here
 * @param rxIn        received samples; they are FILTERED IN PLACE by the band-pass correlator and then multiplied in place by the demodulator's oscillator
 * @param rxBits      received bits are written here, one per int
 * @param txCountIO   in: number of transmit bits supplied; out: number of transmit samples produced
 * @param rxCountIO   in: number of received samples; out: number of received bits produced
 * @return the status word: the low byte is the state (see the file header), the second byte the flags; the host glue uses only the state byte
 * @note  When the state byte is non-zero (the link is not up) the supplied transmit bits are replaced by marks.  Fewer than 6 bits are padded with marks, and
 *        exactly 6 bits are modulated per call whatever the count.
 * @warning `txCountIO` must not exceed 100 (the scratch array) and `rxCountIO` must not exceed 162: the demodulator buffers (`txBuf`, `rxBuf`) hold 162
 *          samples and `toneBuf` 166.  The bits produced must fit `rx_out_internal` (100) and the caller's `rxBits`.  A local-loop engine must not be given
 *          received samples (NULL `bpfCoef`, see B103FP_create()).  The glue passes 160 samples and at most 6 bits.
 */
unsigned int B103FP_modem(B103Obj* b103, int* txBits, short* txOut, short* rxIn, int* rxBits, unsigned short* txCountIO, unsigned short* rxCountIO)
{
  int i;
  unsigned short txCount = *txCountIO;
  B103Bundle* bundlePtr;
  unsigned short rxSampleCount;
  unsigned short txBitsLeft;
  unsigned short txBitsConsumed;
  short* rxOut;

  /* transmit bits; marks while the link is not up */
  for (i = 0; i < (int)txCount; i++)
  {
    tx_in_internal[i] = (short)txBits[i];
  }
  if (b103->st.b[0] != 0)
  {
    for (i = 0; i < (int)*txCountIO; i++)
    {
      tx_in_internal[i] = 1;
    }
  }

  /* band-pass correlator over the received block, written back into the block (history in bpfHist, coefficients in bpfCoef) */
  rxSampleCount = *rxCountIO;
  if (rxSampleCount != 0)
  {
    bundlePtr = b103->bundle;
    int idx = 0;

    do
    {
      short newIdx = bundlePtr->bpfIdx + 1;

      if (newIdx < bundlePtr->bpfLen)
      {
        bundlePtr->bpfIdx = newIdx;
      }
      else
      {
        bundlePtr->bpfIdx = 0;
      }
      {
        short* buf3 = bundlePtr->bpfHist;
        const short* coefs = bundlePtr->bpfCoef;
        short curIdx = bundlePtr->bpfIdx;
        short len = bundlePtr->bpfLen;
        int acc = 0;
        short* pHist;
        const short* pCoef = coefs;
        int j;

        buf3[(int)curIdx] = rxIn[idx] >> 2;
        pHist = buf3 + curIdx;
        for (j = (int)curIdx; j >= 0; j--)
        {
          acc += (int)*pHist * (int)*pCoef;
          pHist--;
          pCoef++;
        }
        pHist += len;
        {
          int k = (int)len;

          while (--k, (int)curIdx < k)
          {
            acc += (int)*pHist * (int)*pCoef;
            pHist--;
            pCoef++;
          }
        }
        rxIn[idx] = (short)((acc >> 0xf) << 2);
      }
      idx++;
    } while (idx < (int)*rxCountIO);
  }

  /* flags: clear bit 1 (error) from the previous call; bit 0 (link up) returns the state byte to 0 */
  {
    unsigned char flags = b103->st.b[1];

    b103->st.b[1] = flags & 0xfd;
    if (flags & 1)
    {
      b103->st.b[0] = 0;
    }
  }

  /* pad the transmit bits to 6 with marks */
  for (i = *txCountIO; i < 6; i++)
  {
    tx_in_internal[i] = 1;
  }

  /* transmit: the current handler runs until it has consumed the 6 bits (a handler may change after a transition) */
  txBitsLeft = 6;
  txBitsConsumed = 0;
  do
  {
    B103Hdx* txHandlerObj = b103->hdx;
    TxHdxFn txHandler = txHandlerObj->txFn;
    short produced = txHandler(b103, (unsigned short*)tx_in_internal, txOut, &txBitsLeft);

    txOut = txOut + produced;
    txBitsConsumed = txBitsConsumed + produced;
  } while (0 < (short)txBitsLeft);
  *txCountIO = txBitsConsumed;

  /* receive: the current handler runs until the block is consumed; the bits are collected in rx_out_internal and widened into rxBits */
  {
    unsigned short rxBitsProduced = 0;
    rxOut = rx_out_internal;
    unsigned short remaining = *rxCountIO;

    do
    {
      B103Hdx* rxHandlerObj = b103->hdx;
      RxHdxFn rxHandler = rxHandlerObj->rxFn;
      short before = (short)remaining;
      short produced = rxHandler(b103, rxIn, (unsigned short*)rxOut, rxCountIO);

      remaining = *rxCountIO;
      rxIn = rxIn + (before - remaining);
      rxBitsProduced = rxBitsProduced + produced;
      rxOut = rxOut + produced;
    } while (remaining != 0);
    *rxCountIO = rxBitsProduced;

    for (i = 0; i < (int)rxBitsProduced; i++)
    {
      rxBits[i] = (unsigned short)rx_out_internal[i];
    }
  }

  /* diagnostics for the states that should not occur */
  switch (b103->st.b[0])
  {
  case 5:
    if (1 < dsplibs_debug_level)
    {
      dsplibs_debug_printf("Bell103 internal error detected!");
    }
    break;
  case 0:
  case 1:
  case 2:
  case 3:
  case 4:
  case 6:
  case 7:
    break;
  default:
    if (1 < dsplibs_debug_level)
    {
      dsplibs_debug_printf("Bell103 unknown internal state!");
    }
  }

  /* the whole status word: state byte, flags byte and two zero bytes */
  return (unsigned int)b103->st.w;
}

/* ---- half-duplex state machine ---- */

/**
 * @brief Transmit handler of sub-state 0: run the mode's first transition.
 *
 * Called through the common TxHdxFn slot with four arguments but uses only the engine; it installs the next handlers and consumes no bits (`*bitsLeft` is
 * unchanged, so B103FP_modem() calls the new handler for the same bits).
 * @return 0 (the number of samples produced).  The caller adds the return value to its output pointer, so the function must return a value.
 */
short TxHdxStartB103(B103Obj* b103, unsigned short* bitsIn, void* samplesOut, unsigned short* bitsLeft)
{
  (void)bitsIn;
  (void)samplesOut;
  (void)bitsLeft;
  B103Hdx* hdxPtr = b103->hdx;
  unsigned short idx = FPM_M(unsigned short, hdxPtr, mode);

  B103NextState[idx](b103);
  return 0;
}

/**
 * @brief Transmit handler: send marks (the supplied bits are overwritten with 1) and count down `txCount`.
 * @return the number of samples produced.  When the countdown ends (Bell 103, cfg[1] == 0: and only once `cnt` has reached 0xf, i.e. the far-end tone is
 *         locked) the mode's transition runs.
 */
short TxHdxMarksB103(B103Obj* b103, unsigned short* bitsIn, void* samplesOut, unsigned short* bitsLeft)
{
  unsigned int i = 0;
  unsigned short avail = *bitsLeft;
  short produced;
  B103Hdx* hdxPtr;
  unsigned short newCount;

  while (avail != (unsigned short)i)
  {
    if (avail < (unsigned short)i)
      break;
    bitsIn[i] = 1;
    i = (i + 1) & 0xffff;
    avail = *bitsLeft;
  }

  produced = ModDataB103(b103, bitsIn, samplesOut, avail);
  *bitsLeft = 0;

  hdxPtr = b103->hdx;
  newCount = FPM_M(unsigned short, hdxPtr, txCount) - 1;
  FPM_M(unsigned short, hdxPtr, txCount) = newCount;

  if (b103->cfg[1] == 0)
  {
    if (0 < (short)newCount || b103->bundle->cnt < 0xf)
    {
      return produced;
    }
  }
  else if (0 < (short)newCount)
  {
    return produced;
  }
  B103NextState[FPM_M(unsigned short, hdxPtr, mode)](b103);
  return produced;
}

/** @brief Transmit handler of the data phase: modulate the supplied bits.  Returns the number of samples produced. */
short TxHdxDataB103(B103Obj* b103, unsigned short* bitsIn, void* samplesOut, unsigned short* bitsLeft)
{
  short produced = ModDataB103(b103, bitsIn, samplesOut, *bitsLeft);

  *bitsLeft = 0;
  return produced;
}

/**
 * @brief Transmit handler: send silence (no carrier) and count down `txCount` (the transmit counter at hdx offset 4, not the receive counter).
 * @return the number of samples produced; the mode's transition runs when the countdown ends
 */
short TxHdxSilenceB103(B103Obj* b103, unsigned short* bitsIn, void* samplesOut, unsigned short* bitsLeft)
{
  B103Hdx* hdxPtr;
  unsigned short newCount;
  short produced = TxNoCarrierB103(b103, bitsIn, samplesOut, *bitsLeft);

  *bitsLeft = 0;

  hdxPtr = b103->hdx;
  newCount = FPM_M(unsigned short, hdxPtr, txCount) - 1;
  FPM_M(unsigned short, hdxPtr, txCount) = newCount;
  if (0 < (short)newCount)
  {
    return produced;
  }
  B103NextState[FPM_M(unsigned short, hdxPtr, mode)](b103);
  return produced;
}

/**
 * @brief Receive handler of the pre-data phase: demodulate and watch the carrier.
 *
 * With the carrier present the mode's transition runs; without it `rxCount` is counted down, and when it drops below 1 the sub-state becomes 5 (error), the
 * transition runs and the status word gets state 5 with flag bit 1.  Returns 0 (no bits are passed on).
 */
short RxHdxStartB103(B103Obj* b103, void* samplesIn, unsigned short* bitsOut, unsigned short* samplesLeft)
{
  B103Hdx* hdxPtr;
  unsigned short newCount;

  DemodDataB103(b103, samplesIn, (short*)bitsOut, *samplesLeft);
  *samplesLeft = 0;
  if (CarrierDetectB103(b103) == 0)
  {
    hdxPtr = b103->hdx;
    newCount = FPM_M(unsigned short, hdxPtr, rxCount) - 1;
    FPM_M(unsigned short, hdxPtr, rxCount) = newCount;
    if ((short)newCount < 1)
    {
      FPM_M(unsigned short, hdxPtr, subState) = 5;
      B103NextState[FPM_M(unsigned short, hdxPtr, mode)](b103);
      b103->st.b[1] |= 2;
      b103->st.b[0] = 5;
    }
    return 0;
  }
  hdxPtr = b103->hdx;
  B103NextState[FPM_M(unsigned short, hdxPtr, mode)](b103);
  return 0;
}

/**
 * @brief Receive handler of the data phase: demodulate, return the bits, and detect a lost carrier.
 * @return the number of bits produced.  With the carrier present flag bit 5 is set and `rxCount` cleared; after more than 7 consecutive blocks without it
 *         the mode's transition runs, the status word gets state 6 and 0 is returned.
 */
short RxHdxDataB103(B103Obj* b103, void* samplesIn, unsigned short* bitsOut, unsigned short* samplesLeft)
{
  B103Hdx* hdxPtr;
  unsigned short newCount;
  short produced = DemodDataB103(b103, samplesIn, (short*)bitsOut, *samplesLeft);

  *samplesLeft = 0;
  b103->st.b[1] &= 0xdf;

  if (CarrierDetectB103(b103) == 0)
  {
    hdxPtr = b103->hdx;
    newCount = FPM_M(unsigned short, hdxPtr, rxCount) + 1;
    FPM_M(unsigned short, hdxPtr, rxCount) = newCount;
    if (7 < (short)newCount)
    {
      B103NextState[FPM_M(unsigned short, hdxPtr, mode)](b103);
      b103->st.b[1] &= 0x7f;
      b103->st.b[0] = 6;
      return 0;
    }
  }
  else
  {
    hdxPtr = b103->hdx;
    b103->st.b[1] |= 0x20;
    FPM_M(unsigned short, hdxPtr, rxCount) = 0;
  }
  b103->st.b[1] &= 0x7f;
  return produced;
}

/**
 * @brief Receive handler while waiting for the far end's carrier: run the tone detector, no bits are produced.
 *
 * Counts `rxCount` up and runs the tone detector on the block.  A block with the tone adds 5 to `cnt` and, in sub-state 1, runs the mode's transition; a block
 * without it ends the wait once `rxCount` has reached `waitLimit`: sub-state 5, the transition, and state 5 with flag bit 1.  Returns 0.
 */
short RxDetMarkB103(B103Obj* b103, void* samplesIn, unsigned short* bitsOut, unsigned short* samplesLeft)
{
  B103Hdx* hdxPtr = b103->hdx;
  B103Bundle* bundle = b103->bundle;

  (void)bitsOut;

  hdxPtr->rxCount = hdxPtr->rxCount + 1;
  FPM_AGC_agc(&bundle->agc1, (short*)samplesIn, *samplesLeft);

  if (FPM_TONE_detect(hdxPtr->secTone, (short*)samplesIn, *samplesLeft) == 0)
  {
    bundle->cnt = bundle->cnt + 5;
    if (FPM_M(unsigned short, hdxPtr, subState) == 1)
    {
      B103NextState[FPM_M(unsigned short, hdxPtr, mode)](b103);
    }
  }
  else
  {
    if ((short)FPM_M(unsigned short, hdxPtr, waitLimit) <= (short)FPM_M(unsigned short, hdxPtr, rxCount))
    {
      FPM_M(unsigned short, hdxPtr, subState) = 5;
      B103NextState[FPM_M(unsigned short, hdxPtr, mode)](b103);
      b103->st.b[1] |= 2;
      b103->st.b[0] = 5;
      *samplesLeft = 0;
      return 0;
    }
  }
  *samplesLeft = 0;
  return 0;
}

/**
 * @brief Modulate `bitCount` bits and run the result through the transmit matched filter.
 * @return the number of samples written to `samplesOut`
 */
short ModDataB103(B103Obj* b103, unsigned short* bitsIn, void* samplesOut, unsigned short bitCount)
{
  B103Bundle* bundle = b103->bundle;
  short produced = FPM_FSM_modulate(&bundle->fsm, bitsIn, bundle->txBuf, (short)bitCount);

  produced = (short)FPM_MRF_filter(&bundle->mrf0, bundle->txBuf, samplesOut, produced);
  return produced;
}

/**
 * @brief Demodulate a block of received samples.
 * @param b103         engine
 * @param samplesIn    received samples (MODIFIED: multiplied in place by the local oscillator)
 * @param bitsOut      decided bits, one per short
 * @param sampleCount  number of samples
 * @return the number of bits produced; 0 until the far-end tone is locked (`cnt` >= 0xf)
 *
 * While `cnt` < 0xf a copy of the block goes through AGC `agc1` and the carrier detector `secTone` (each block with the tone adds 5 to `cnt`, any block
 * without it clears `cnt`).  The block is then mixed with the oscillator, filtered by `mrf1` and gain-controlled by `agc0` whose detect flag is kept in
 * `agcDetect`.  When `cnt` reaches 0xf the gain is frozen; from then on `cnt` stays 0x10, `toneSeen` is 1 and the demodulator FPM_FSD produces the bits.
 * @warning `sampleCount` must not exceed 162 (the size of `rxBuf` and `txBuf`) and 166 (`toneBuf`).
 */
short DemodDataB103(B103Obj* b103, short* samplesIn, short* bitsOut, unsigned short sampleCount)
{
  B103Bundle* bundle = b103->bundle;
  short toneBuf[166];
  int i;
  unsigned int produced;

  /* far-end tone lock */
  if (bundle->cnt < 0xf)
  {
    void* rxBuf = FPM_M(void*, bundle, rxBuf);

    for (i = 0; i < (int)sampleCount; i++)
    {
      ((short*)rxBuf)[i] = samplesIn[i];
    }
    FPM_AGC_agc(&bundle->agc1, (short*)rxBuf, (unsigned short)sampleCount);
    bundle = b103->bundle;
    bundle->toneSeen = 0;
    if (FPM_TONE_detect(b103->hdx->secTone, (short*)rxBuf, (short)sampleCount) == 0)
    {
      bundle->toneSeen = 1;
      bundle->cnt = bundle->cnt + 5;
    }
    else
    {
      bundle->cnt = 0;
    }
  }

  /* mix with the local oscillator (Q14), filter, level */
  FPM_TONE_generate_demod(b103->hdx->mainTone, toneBuf, (short)sampleCount);
  for (i = 0; i < (int)sampleCount; i++)
  {
    samplesIn[i] = (short)(((int)samplesIn[i] * (int)toneBuf[i]) >> 0xe);
  }

  produced = FPM_MRF_filter(&bundle->mrf1, samplesIn, bundle->txBuf, (short)sampleCount);
  bundle->agc0.active = bundle->agcActive;
  FPM_AGC_agc(&bundle->agc0, bundle->txBuf, (unsigned short)produced);
  bundle = b103->bundle;
  bundle->agcDetect = bundle->agc0.detect;

  /* freeze the gain when the lock is reached; bits only once locked */
  if (bundle->cnt == 0xf)
  {
    FPM_AGC_Freeze(&bundle->agc0);
    bundle = b103->bundle;
  }
  if (bundle->cnt < 0xf)
  {
    return 0;
  }
  bundle->toneSeen = 1;
  bundle->cnt = 0x10;
  return (short)FPM_FSD_demodulate(&bundle->fsd, (unsigned short*)FPM_M(void*, bundle, txBuf), bitsOut, (short)produced);
}

/**
 * @brief Run the modulator with its amplitude temporarily set to 0 (silence), through the transmit matched filter.
 * @return the number of samples written to `samplesOut`
 */
short TxNoCarrierB103(B103Obj* b103, unsigned short* bitsIn, void* samplesOut, unsigned short bitCount)
{
  B103Bundle* bundle = b103->bundle;
  short saved = bundle->fsm.c.s[3];
  short produced;

  bundle->fsm.c.s[3] = 0;
  produced = FPM_FSM_modulate(&bundle->fsm, bitsIn, bundle->txBuf, (short)bitCount);
  bundle = b103->bundle;
  bundle->fsm.c.s[3] = saved;
  return (short)FPM_MRF_filter(&bundle->mrf0, bundle->txBuf, samplesOut, produced);
}

/** @brief Carrier detect: non-zero when the far-end tone was seen and the AGC reports signal. */
unsigned int CarrierDetectB103(B103Obj* b103)
{
  B103Bundle* bundle = b103->bundle;

  return FPM_M(unsigned int, bundle, toneSeen) & FPM_M(unsigned int, bundle, agcDetect);
}

/**
 * @brief Handshake transition of the answer role, selected by `hdx->subState`.
 *
 * 0 -> 1: send marks, receive in the tone-wait handler, state 2.  1 -> 2: `txCount` and `rxCount` set to 8, state 3.  2 -> 4: data handlers installed,
 * state 7 (link established).
 */
void B103AnswerNextState(B103Obj* b103)
{
  B103Hdx* hdxPtr = b103->hdx;
  short subState = hdxPtr->subState;

  if (subState == 1)
  {
    hdxPtr->txCount = 8;
    hdxPtr->rxCount = 8;
    hdxPtr->subState = 2;
    b103->st.b[1] &= 0xbf;
    b103->st.b[0] = 3;
  }
  else if (subState < 2)
  {
    if (subState == 0)
    {
      hdxPtr->txFn = (TxHdxFn)TxHdxMarksB103;
      hdxPtr->rxFn = (RxHdxFn)RxDetMarkB103;
      hdxPtr->rxCount = 0;
      hdxPtr->txCount = hdxPtr->waitLimit;
      hdxPtr->subState = 1;
      b103->st.b[1] |= 0x10;
      b103->st.b[0] = 2;
    }
  }
  else if (subState == 2)
  {
    hdxPtr->txFn = (TxHdxFn)TxHdxDataB103;
    hdxPtr->rxFn = (RxHdxFn)RxHdxDataB103;
    hdxPtr->subState = 4;
    b103->st.b[1] |= 0x2d;
    b103->st.b[0] = 7;
  }
}

/**
 * @brief Handshake transition of the originate role, selected by `hdx->subState`.
 *
 * 0 -> 1: send silence, receive in the tone-wait handler, state 2.  1 -> 2: `txCount` 8, state 3.  2 -> 3: send marks for 0x28 blocks, state 4.
 * 3 -> 4: data handlers installed, state 7 (link established).
 */
void B103OriginateNextState(B103Obj* b103)
{
  B103Hdx* hdxPtr = b103->hdx;
  short subState = hdxPtr->subState;

  if (subState == 1)
  {
    hdxPtr->txCount = 8;
    hdxPtr->rxCount = 0;
    hdxPtr->subState = 2;
    b103->st.b[1] &= 0xbf;
    b103->st.b[0] = 3;
  }
  else if (subState < 2)
  {
    if (subState == 0)
    {
      hdxPtr->txFn = (TxHdxFn)TxHdxSilenceB103;
      hdxPtr->rxCount = 1;
      hdxPtr->rxFn = (RxHdxFn)RxDetMarkB103;
      hdxPtr->txCount = hdxPtr->waitLimit;
      hdxPtr->subState = 1;
      b103->st.b[1] |= 0x10;
      b103->st.b[0] = 2;
    }
  }
  else if (subState == 2)
  {
    hdxPtr->txCount = 0x28;
    hdxPtr->rxCount = 0;
    hdxPtr->txFn = (TxHdxFn)TxHdxMarksB103;
    hdxPtr->subState = 3;
    b103->st.b[1] |= 0x24;
    b103->st.b[0] = 4;
  }
  else if (subState == 3)
  {
    hdxPtr->txFn = (TxHdxFn)TxHdxDataB103;
    hdxPtr->rxFn = (RxHdxFn)RxHdxDataB103;
    hdxPtr->subState = 4;
    b103->st.b[1] |= 9;
    b103->st.b[0] = 7;
  }
}

/**
 * @brief Handshake transition of the local-loop role, selected by `hdx->subState`.
 *
 * 0 -> 1: send marks, `txCount` = 2 * `waitLimit`, state 2.  1 -> 2: data receive handler, `txCount` 8, state 3.  2 -> 4: state 7 (the transmit handler
 * stays the marks handler).
 */
void B103LocLoopNextState(B103Obj* b103)
{
  B103Hdx* hdxPtr = b103->hdx;
  short subState = hdxPtr->subState;

  if (subState == 1)
  {
    hdxPtr->txCount = 8;
    hdxPtr->rxFn = (RxHdxFn)RxHdxDataB103;
    hdxPtr->rxCount = 0;
    hdxPtr->subState = 2;
    b103->st.b[1] &= 0xbf;
    b103->st.b[0] = 3;
  }
  else if (subState < 2)
  {
    if (subState == 0)
    {
      hdxPtr->txFn = (TxHdxFn)TxHdxMarksB103;
      hdxPtr->subState = 1;
      hdxPtr->txCount = hdxPtr->waitLimit * 2;
      hdxPtr->rxCount = hdxPtr->waitLimit;
      b103->st.b[1] |= 0x10;
      b103->st.b[0] = 2;
    }
  }
  else if (subState == 2)
  {
    hdxPtr->txFn = (TxHdxFn)TxHdxMarksB103;
    hdxPtr->subState = 4;
    b103->st.b[1] |= 0x2d;
    b103->st.b[0] = 7;
  }
}
