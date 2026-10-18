/**
 * @file v23.c
 * @brief V.23 modem engine: 1200 bit/s forward-channel receiver, 75 bit/s backward-channel receiver, table-driven FSK transmitter and the top-level sequencer.
 *
 * V.23 uses two FSK channels: 1200 bit/s forward (mark 1300 Hz, space 2100 Hz) and 75 bit/s backward (mark 390 Hz, space 450 Hz).  An engine instance
 * (V23Modem, v23.h) transmits on one and receives on the other, selected by the `rxBackend` argument of CreateV23Modem():
 *   - rxBackend == 0: receives the forward channel with v23FP_rx_* (V23Rx) and transmits the backward channel (390 / 450 Hz, about 106.67 samples per bit);
 *   - rxBackend != 0: receives the backward channel with BwChDem_* (V23BwCh) and transmits the forward channel (1300 / 2100 Hz, about 6.67 samples per bit).
 * The transmitter v23FP_tx_* (V23Tx) is the same code for both; it turns bits into a tone of one of two frequencies and counts samples per bit from a
 * three-entry table (bw_ch_samp_per_bit_table or fw_ch_samp_per_bit_table, v23_backchannel_tables.c) so that the average is not an integer.
 *
 * Entry points (called by the host glue dp_v23.c): CreateV23Modem(), V23ModemMain() once per block, DeleteV23Modem().  V23ModemMain() first runs a
 * sequencer: an instance created with the answer-tone flag sends a 2100 Hz tone for 3 seconds (state 0), then 50 ms of silence (state 1), and only then
 * (state 2) runs transmitter and receiver; an instance without the flag starts in state 2.  In state 2 the transmitter sends marks until the receiver has
 * found its carrier.  The result code of V23ModemMain() is 0 = the receiver is running, 1 = still setting up (tone, silence or carrier search),
 * 2 = failure (carrier search time-out, loss of signal).
 *
 * The receivers filter the received block in place (the caller's input buffer is overwritten).  All objects are created from the configuration blocks of
 * fsk_tables.c / v23_backchannel_tables.c and fpm/fpm_tables.c, with the per-object overrides of the build...Cfg() helpers below.
 */
#include <stdint.h>
#include "core/dsp_table_handles.h"
#include "fpm/fpm_tables.h"
#include "fsk_tables.h"
#include "v23_backchannel_tables.h"
#include "v23.h"
#include "core/dsp_host.h"
#include "fpm/fpm.h"

/* The answer-tone slot is cleared as two shorts on a 32-bit build, as one pointer on a 64-bit build. */
#if UINTPTR_MAX == 0xffffffffu
#define V23_CLR_TONE(o) ((o)->tone.s[0] = 0, (o)->tone.s[1] = 0)
#else
#define V23_CLR_TONE(o) ((o)->tone.p = 0)
#endif

V23BwCh* BwChDem_Create(V23BwCh* existing, const V23Cfg* cfg);
void BwChDem_Delete(V23BwCh* bw);
int BwChDem_Progress(V23BwCh* self, short* rxIn, unsigned int rxCount, unsigned int* bitsOut, int* nBitsOut);

/**
 * @brief Build the nine-word FPM_TONE_create() configuration of the forward-channel receiver's carrier detector.
 * @param cfg         nine configuration words to fill
 * @param freqLow16   frequency word, replaces the low 16 bits of word 0 (the high 16 bits stay)
 * @param word1Hi16   replaces the high 16 bits of word 1 (the low 16 bits stay)
 *
 * Everything else is TONEv23_CFG_raw; word 4 is the low-pass prototype table ToneLPF.
 */
static void buildV23RxToneCfg(int* cfg, int freqLow16, int word1Hi16)
{
  int i;

  for (i = 0; i < 9; i++)
    cfg[i] = TONEv23_CFG_raw[i];
  cfg[0] = (TONEv23_CFG_raw[0] & 0xffff0000) | (freqLow16 & 0xffff);
  cfg[1] = (word1Hi16 << 16) | (TONEv23_CFG_raw[1] & 0xffff);
  cfg[4] = CFGP(ToneLPF);
}

/**
 * @brief Build the nine-word FPM_TONE_create() configuration of the transmitter's tone generator.
 * @param cfg  nine configuration words to fill
 *
 * TONEv23_CFG_raw with the high 16 bits of word 0 (the amplitude) replaced by 0xee4; the frequency (low half) is set per bit by v23FP_tx_progress().
 * Word 4 is the low-pass prototype table ToneLPF.
 */
static void buildV23TxToneCfg(int* cfg)
{
  int i;

  for (i = 0; i < 9; i++)
    cfg[i] = TONEv23_CFG_raw[i];
  cfg[0] = (0xee4 << 16) | (TONEv23_CFG_raw[0] & 0xffff);
  cfg[4] = CFGP(ToneLPF);
}

/**
 * @brief Build the nine-word FPM_TONE_create() configuration of the answer-tone generator of CreateV23Modem().
 * @param cfg                 nine configuration words to fill
 * @param backwardChannelRx   non-zero for an instance that receives the backward channel (the amplitude half of word 0 is then 0xee4, otherwise 0)
 *
 * FPM_TONE_CFG_raw with word 0 replaced by (amplitude << 16) | 0x834 (the 2100 Hz tone) and the low half of word 1 cleared.  Word 4 is ToneLPF.
 */
static void buildAnswerToneCfg(int* cfg, int backwardChannelRx)
{
  int i;

  for (i = 0; i < 9; i++)
    cfg[i] = FPM_TONE_CFG_raw[i];
  cfg[0] = ((backwardChannelRx ? 0xee4 : 0) << 16) | 0x834;
  cfg[1] = FPM_TONE_CFG_raw[1] & 0xffff0000;
  cfg[4] = CFGP(ToneLPF);
}

/**
 * @brief Build the nine-word FPM_TONE_create() configuration of the backward-channel receiver's tone detector (390 Hz).
 * @param cfg  nine configuration words to fill
 *
 * FPM_TONE_CFG_raw with word 0 replaced by (its high half | 0x186), the high half of word 1 by 29000 and the high half of word 2 by 3.  Word 4 is ToneLPF.
 */
static void buildBwChTxToneCfg(int* cfg)
{
  int i;

  for (i = 0; i < 9; i++)
    cfg[i] = FPM_TONE_CFG_raw[i];
  cfg[0] = (((unsigned int)FPM_TONE_CFG_raw[0] >> 16) << 16) | 0x186;
  cfg[1] = (29000 << 16) | (FPM_TONE_CFG_raw[1] & 0xffff);
  cfg[2] = (3 << 16) | (FPM_TONE_CFG_raw[2] & 0xffff);
  cfg[4] = CFGP(ToneLPF);
}

/* ==================== v23FP_rx_*: 1200 bit/s forward-channel receiver ==================== */

/**
 * @brief Create a forward-channel receiver, or re-initialise an existing one.
 * @param existing  existing object (its filter objects are re-initialised as new ones), or NULL to allocate
 * @param cfg       engine configuration block (V23Cfg); its `thr` (signal-absence threshold, 700 from the glue) becomes `energyThr`
 * @return the receiver
 *
 * Creates the matched filter (3 branches, step 4, 16 taps per branch), the FSK demodulator (5 samples per bit), the carrier tone detector (1300 Hz),
 * the 4-section IIR pre-filter and two AGCs, and sets the carrier-search limit to 60000 (3000 blocks).
 * @warning The allocations are not checked.  Passing an existing object allocates new filter objects without freeing the old ones.
 */
V23Rx* v23FP_rx_create(V23Rx* existing, const V23Cfg* cfg)
{
  V23Rx* obj = existing;
  FpmTone* tone;
  int mrfCfg[4];
  int fsdCfg[7];
  int toneCfg[9];

  if (obj == 0)
  {
    obj = (V23Rx*)sysdep_malloc(sizeof(V23Rx));
  }

  /* matched filter and demodulator */
  mrfCfg[0] = MRFv23_CFG_raw[0];
  mrfCfg[1] = CFGP(V23_MRF_FILT);
  mrfCfg[2] = MRFv23_CFG_raw[2];
  mrfCfg[3] = MRFv23_CFG_raw[3];
  FPM_MRF_init(&obj->mrf, mrfCfg, 1);

  fsdCfg[0] = CFGP(V23RX_ANSWER_INTRP);
  fsdCfg[1] = 0x6000f;
  fsdCfg[2] = CFGP(V23RX_IIR_LPF);
  fsdCfg[3] = (FSDv23_CFG_raw[3] & 0xffff0000) | 3;
  fsdCfg[4] = FSDv23_CFG_raw[4];
  fsdCfg[5] = FSDv23_CFG_raw[5];
  fsdCfg[6] = FSDv23_CFG_raw[6];
  FPM_FSD_init(&obj->fsd, fsdCfg, 1);

  /* carrier detector, IIR pre-filter */
  buildV23RxToneCfg(toneCfg, 0x514, 0x7148);
  tone = FPM_TONE_create(0, toneCfg);
  obj->tone = tone;

  obj->iirCoef = V23_IIR_FILT;
  obj->state = 0;
  obj->iirCount = 4;
  {
    short* iirState = (short*)sysdep_malloc(0x20);
    int i;

    obj->iirState = iirState;
    for (i = 0; i < 16; i++)
      iirState[i] = 0;
  }

  /* AGCs and the time-outs */
  FPM_AGC_init(&obj->agc0, AGCv23_CFG_raw, 1);
  FPM_AGC_init(&obj->agc1, AGCv23_CFG_raw, 1);
  obj->agc1.a.s[5] = 0x28;
  obj->energyAcc = 0;
  obj->agc1.a.s[0] = 0x4000;
  obj->energyThr = (short)cfg->thr;
  obj->carrierAcc = 0;
  obj->carrierThr = 60000;
  obj->noCarrier = 1;

  if (1 < dsplibs_debug_level)
  {
    dsplibs_debug_printf("V23FP Rx Created, version 10-December-02.\r\n");
  }
  return obj;
}

/** @brief Free a forward-channel receiver and its sub-objects; NULL is accepted. */
void v23FP_rx_delete(V23Rx* rx)
{
  V23Rx* obj = rx;

  if (obj != 0)
  {
    FPM_TONE_delete(obj->tone);
    FPM_MRF_free(&obj->mrf);
    FPM_FSD_free(&obj->fsd);
    sysdep_free(obj->iirState);
    sysdep_free(obj);
  }
}

/**
 * @brief Process one block of the forward channel.
 * @param self       receiver
 * @param rxIn       received samples; FILTERED IN PLACE (IIR pre-filter, then the matched filter writes its output over the input)
 * @param rxCount    number of samples; at most 160 (the size of `scratch`), and the demodulated bits must fit `bits` (26)
 * @param bitsOut    receives the demodulated bits, one per word
 * @param nBitsOut   receives the number of bits; written only on the return-0 path
 * @return 0 = carrier established and demodulating; 1 = carrier not yet established; 2 = failure (carrier search time-out or loss of signal)
 *
 * Phase 1 (`state` < 10): each call adds 20 to `carrierAcc`; while it stays below the limit the block is copied to `scratch`, levelled by `agc1` and
 * tested for the carrier tone, a block with the tone adds 5 to `state` (two blocks confirm).  When the limit is reached the call fails.
 * Phase 2: the block is filtered and demodulated; at `state` 10 the gain of `agc0` is frozen and the call still returns 1; afterwards `state` is 11.  Blocks
 * without signal (AGC detect clear) add 20 to `energyAcc` and fail the call when it reaches `energyThr` (35 blocks for 700); a block with signal clears it.
 * While `energyAcc` is non-zero but below the limit the bits are demodulated and dropped.
 * @note Once a time-out has been returned the counters keep running: `carrierAcc` / `energyAcc` are 16-bit and wrap, so a caller that keeps calling sees
 *       2 until the wrap and then a new search (read from the code, not run).
 */
int v23FP_rx_progress(V23Rx* self, short* rxIn, unsigned int rxCount, unsigned int* bitsOut, int* nBitsOut)
{
  V23Rx* obj = self;
  short state = obj->state;
  int i;

  /* carrier search: count the blocks, look for the carrier tone */
  if (state < 10)
  {
    short acc = obj->carrierAcc + 0x14;

    obj->carrierAcc = acc;
    if ((unsigned short)acc < (unsigned short)obj->carrierThr)
    {
      short* scratch = obj->scratch;

      for (i = 0; i < (int)rxCount; i++)
        scratch[i] = rxIn[i];
      FPM_AGC_agc(&obj->agc1, scratch, (unsigned short)rxCount);
      if (FPM_TONE_detect(obj->tone, scratch, (short)rxCount) == 0)
      {
        obj->state = state + 5;
        if (1 < dsplibs_debug_level)
        {
          dsplibs_debug_printf("v23 tone detected, counter = %d,threshold = 2\n", (int)(short)((short)(state + 5) / 5));
        }
      }
      goto steady;
    }
    if (1 < dsplibs_debug_level)
    {
      dsplibs_debug_printf("Carrier Detection Time Out ");
    }
    return 2;
  }

steady:
  {
    unsigned int mrfCount;
    short agcRet;

    /* pre-filter, matched filter, gain control; agcRet is the AGC's signal-present flag */
    FPM_iir_filt_II(rxIn, (short*)obj->iirCoef, obj->iirState, obj->iirCount, (short)rxCount);
    mrfCount = FPM_MRF_filter(&obj->mrf, rxIn, rxIn, (short)rxCount);
    FPM_AGC_agc(&obj->agc0, rxIn, (unsigned short)mrfCount);
    agcRet = FPM_M(short, obj, agc0.detect);

    /* carrier confirmed: freeze the gain; the bits are returned from the next call on */
    state = obj->state;
    if (state == 10)
    {
      FPM_AGC_Freeze(&obj->agc0);
      state = obj->state;
    }
    if (state < 10)
    {
      obj->noCarrier = 1;
      return (int)obj->noCarrier;
    }

    /* signal-absence watch */
    obj->state = 0xb;
    if (agcRet == 0)
    {
      short acc = obj->energyAcc + 0x14;

      obj->energyAcc = acc;
      if ((unsigned short)obj->energyThr <= (unsigned short)acc)
      {
        if (1 < dsplibs_debug_level)
        {
          dsplibs_debug_printf("Energy drop detected......\n");
        }
        return 2;
      }
    }
    else
    {
      obj->energyAcc = 0;
    }

    /* demodulate; the bits are passed on only while the signal is present */
    {
      unsigned short demodBits = FPM_FSD_demodulate(&obj->fsd, (unsigned short*)rxIn, obj->bits, (short)mrfCount);

      obj->nBits = (short)demodBits;
      if (obj->energyAcc == 0)
      {
        *nBitsOut = (int)(short)demodBits;
        for (i = 0; (short)i < (short)demodBits; i++)
        {
          bitsOut[i] = (unsigned int)(unsigned short)*(short*)((char*)obj->bits + i * 2);
        }
        obj->noCarrier = 0;
      }
      else
      {
        obj->noCarrier = 0;
        *nBitsOut = 0;
      }
    }
    return (int)obj->noCarrier;
  }
}

/* ==================== v23FP_tx_*: table-driven FSK generator ==================== */

/**
 * @brief Create a transmitter, or re-initialise an existing one.
 * @param existing          existing object, or NULL to allocate
 * @param freqMark          tone frequency of a 1 bit
 * @param freqSpace         tone frequency of a 0 bit
 * @param tableLen          number of entries in the samples-per-bit table (3 in this engine)
 * @param sampPerBitTable   samples per bit, rotated through for successive bits (must stay valid while the transmitter lives)
 * @param silenceFlag       non-zero: the first v23FP_tx_progress() call writes one block of silence
 * @return the transmitter
 * @warning The allocation is not checked; passing an existing object allocates a new tone generator without freeing the old one.
 */
V23Tx* v23FP_tx_create(V23Tx* existing, short freqMark, short freqSpace, short tableLen, const short* sampPerBitTable, int silenceFlag)
{
  V23Tx* obj = existing;
  FpmTone* tone;
  int toneCfg[9];

  if (obj == 0)
  {
    obj = (V23Tx*)sysdep_malloc(sizeof(V23Tx));
  }
  obj->len = tableLen;
  obj->freq0 = freqSpace;
  obj->table = sampPerBitTable;
  obj->freq1 = freqMark;
  obj->idx = 0;
  obj->cont = 0;
  obj->countdown = sampPerBitTable[0];
  obj->silence = silenceFlag;

  buildV23TxToneCfg(toneCfg);
  tone = FPM_TONE_create(0, toneCfg);
  obj->tone = tone;
  return obj;
}

/** @brief Free a transmitter and its tone generator; NULL is accepted. */
void v23FP_tx_delete(V23Tx* tx)
{
  V23Tx* obj = tx;

  if (obj != 0)
  {
    FPM_TONE_delete(obj->tone);
    sysdep_free(obj);
  }
}

/**
 * @brief Generate `count` samples of FSK from a bit array.
 * @param self      transmitter
 * @param out       output samples
 * @param count     number of samples to write
 * @param bitSrc    bits, one per int (non-zero = mark); advanced as bits are started
 * @param nBitsOut  receives the number of bits taken from `bitSrc`
 * @return in the silence case the (non-zero) flag value; otherwise 0 when the block ends inside a bit, else 1 when the bit that ended was the last table
 *         entry of a revolution and 0 if not.  The only caller, V23ModemMain(), ignores it.
 *
 * Every bit lasts `table[idx]` samples, `idx` rotating through the table, so the average samples per bit is not an integer.  For each sample the tone
 * generator is retuned to the bit's frequency and one sample of its cosine output is produced.  A bit that does not end inside the block is remembered
 * (`cont`, `carry`) and finished by the next call without reading `bitSrc` again.  With the one-shot silence flag set, one block of zeros is written
 * instead and the flag is cleared; no bit is consumed but `*nBitsOut` is set to `count`.
 * @warning A call with `count` == 0 takes the "ends inside a bit" exit with `carryBit` still 0: it marks the current bit as in progress with value 0 (or
 *          overwrites the remembered value with 0), so the next call finishes that bit as a space whatever bit the caller supplies.  V23ModemMain() never
 *          passes 0 samples with the glue's block size (read from the code, not run).
 */
int v23FP_tx_progress(V23Tx* self, short* out, int count, int* bitSrc, int* nBitsOut)
{
  V23Tx* obj = self;
  int carryBit = 0;
  int bitsConsumed = 0;
  int silenceFlag = obj->silence;
  short countdown;

  /* one-shot silence block */
  if (silenceFlag != 0)
  {
    int i;

    for (i = 0; i < count; i++)
      out[i] = 0;
    obj->silence = 0;
    *nBitsOut = count;
    return silenceFlag;
  }

  /* samples of the current and following bits */
  while (count != 0)
  {
    while (countdown = obj->countdown, obj->countdown = countdown - 1, countdown != 0)
    {
      short freq;

      if (obj->cont == 0)
      {
        carryBit = *bitSrc;
        freq = (carryBit != 0) ? obj->freq1 : obj->freq0;
      }
      else
      {
        carryBit = obj->carry;
        freq = (carryBit != 0) ? obj->freq1 : obj->freq0;
      }
      count--;
      FPM_TONE_set_freq(obj->tone, freq);
      FPM_TONE_generate_demod(obj->tone, out, 1);
      out++;
      if (count == 0)
        goto tailCheck;
    }

    /* the bit is complete: next bit, next table entry */
    if (obj->cont == 0)
    {
      bitSrc++;
    }
    else
    {
      obj->cont = 0;
    }
    bitsConsumed++;
    {
      short idx = (short)((obj->idx + 1) % obj->len);
      const short* table = obj->table;

      obj->idx = idx;
      obj->countdown = table[idx];
    }
  }

tailCheck:
  /* the block ends inside a bit: remember the bit for the next call */
  if (obj->countdown != 0)
  {
    obj->cont = 1;
    obj->carry = carryBit;
    *nBitsOut = bitsConsumed;
    return 0;
  }

  /* the block ends exactly at the end of a bit */
  {
    short idx0 = obj->idx;
    short len = obj->len;
    short idx1 = (short)((idx0 + 1) % len);
    const short* table = obj->table;

    obj->idx = idx1;
    obj->countdown = table[idx1];
    *nBitsOut = bitsConsumed + 1;
    return (int)(short)((idx0 + 1) / len);
  }
}

/* ==================== CreateV23Modem / DeleteV23Modem / V23ModemMain ==================== */

/**
 * @brief Create a V.23 engine, or re-initialise the sequencer of an existing one.
 * @param existing    existing engine (receiver and transmitter are kept), or NULL to allocate everything
 * @param rxBackend   0: receive the forward channel (V23Rx), transmit the backward channel; non-zero: receive the backward channel (V23BwCh), transmit
 *                    the forward channel
 * @param cfg         configuration block (V23Cfg): `tone` = answer-tone flag, `rate` = sample rate, `thr` = signal-absence
 *                    threshold given to the receiver
 * @return the engine
 *
 * The sequencer limits are 3 * rate samples for the answer tone and rate / 20 for the silence.  With the answer-tone flag the sequencer starts in state 0
 * with a 2100 Hz generator; the flag is also given to the transmitter as its one-shot silence flag.  Without it the sequencer starts in state 2.
 * @warning The allocations are not checked.  On an existing engine the transmitter keeps its flag, and an answer-tone generator created earlier is
 *          replaced (flag set) or dropped (flag clear) without being freed.
 */
V23Modem* CreateV23Modem(V23Modem* existing, short rxBackend, const V23Cfg* cfg)
{
  V23Modem* obj = existing;
  int rate;

  /* a new engine: the receiver of the selected backend and the transmitter of the other channel */
  if (obj == 0)
  {
    const short* sampPerBitTable;
    short freqMark, freqSpace;
    void* rxObj;
    V23Tx* txObj;
    char answerTone = (char)cfg->tone;

    obj = (V23Modem*)sysdep_malloc(sizeof(V23Modem));
    obj->backend = rxBackend;
    if (rxBackend == 0)
    {
      rxObj = v23FP_rx_create(0, cfg);
      sampPerBitTable = bw_ch_samp_per_bit_table;
      freqMark = 0x186;
      freqSpace = 0x1c2;
    }
    else
    {
      rxObj = BwChDem_Create(0, cfg);
      sampPerBitTable = fw_ch_samp_per_bit_table;
      freqMark = 0x514;
      freqSpace = 0x834;
    }
    obj->rx = rxObj;
    txObj = v23FP_tx_create(0, freqMark, freqSpace, 3, sampPerBitTable, answerTone);
    obj->tx = txObj;
  }

  if (1 < dsplibs_debug_level)
  {
    dsplibs_debug_printf("V23FP version %s %s\n", "Sep 22 2005", "15:48:09");
  }

  /* sequencer limits */
  obj->count.s[0] = 0;
  obj->count.s[1] = 0;
  rate = cfg->rate;
  obj->rate = rate;
  obj->thrTx = rate * 3;
  obj->thrSil = rate / 20;

  /* initial state: answer tone, or straight to the steady state */
  if (cfg->tone != 0)
  {
    int toneCfg[9];

    if (1 < dsplibs_debug_level)
    {
      dsplibs_debug_printf("Generating answer tone.\n");
    }
    buildAnswerToneCfg(toneCfg, obj->backend != 0);
    obj->state.s[0] = 0;
    obj->state.s[1] = 0;
    obj->tone.p = FPM_TONE_create(0, toneCfg);
    FPM_M(int, obj, shadow) = FPM_M(int, obj, state.w);
    return obj;
  }
  obj->state.s[0] = 2;
  obj->state.s[1] = 0;
  V23_CLR_TONE(obj);
  FPM_M(int, obj, shadow) = FPM_M(int, obj, state.w);
  return obj;
}

/** @brief Free a V.23 engine with its answer-tone generator, transmitter and receiver; NULL is accepted. */
void DeleteV23Modem(V23Modem* modem)
{
  V23Modem* obj = modem;

  if (obj == 0)
    return;

  if (obj->tone.p != 0)
  {
    FPM_TONE_delete(obj->tone.p);
  }
  v23FP_tx_delete(obj->tx);
  if (obj->backend != 0)
  {
    BwChDem_Delete(FPM_M(V23BwCh*, obj, rx));
  }
  else
  {
    v23FP_rx_delete(FPM_M(V23Rx*, obj, rx));
  }
  sysdep_free(obj);
}

/**
 * @brief Run the engine for one block.
 * @param self            engine
 * @param txBitSrc        transmit bits, one per int; overwritten with marks while the receiver has no carrier
 * @param txBitCount      in: number of bits available; out: number of bits the transmitter took from `txBitSrc`
 * @param txAudioOut      transmit samples
 * @param txSampleCount   number of transmit samples to write
 * @param rxAudioIn       received samples (FILTERED IN PLACE by the receiver)
 * @param rxSampleCount   number of received samples
 * @param rxBitsOut       demodulated bits
 * @param nRxBitsOut      number of demodulated bits (set by the receiver on the paths that produce bits)
 * @return 0 = receiver running; 1 = answer tone, silence or carrier search still in progress; 2 = failure (see the file header)
 *
 * States 0 and 1 (answer tone, silence) only write `rxSampleCount` samples of tone or silence into `txAudioOut`, count samples, and return 1; the received
 * samples are not looked at and no bits are consumed.  In state 2 the transmitter runs with the receiver's "no carrier" flag forcing marks, and the
 * receiver's result is returned.
 * @note The tone and silence lengths of states 0 and 1 use `rxSampleCount`, not `txSampleCount`; the glue passes the same value for both.
 */
int V23ModemMain(V23Modem* self, int* txBitSrc, int* txBitCount, short* txAudioOut, int txSampleCount, short* rxAudioIn, int rxSampleCount,
                 unsigned int* rxBitsOut, int* nRxBitsOut)
{
  V23Modem* obj = self;
  unsigned int state = obj->state.w;

  /* state-change logging */
  if (state != obj->shadow)
  {
    if (1 < dsplibs_debug_level)
    {
      dsplibs_debug_printf("V23ModemMain: modem state moved from %d to %d\n", obj->shadow, state);
      state = obj->state.w;
    }
    obj->shadow = state;
  }

  /* steady state: transmit (marks until the receiver has a carrier) and receive */
  if (1 < state)
  {
    int i;
    void* rx = obj->rx;

    if (obj->backend == 0)
    {
      if (((V23Rx*)rx)->noCarrier != 0)
      {
        for (i = 0; i < *txBitCount; i++)
          txBitSrc[i] = 1;
      }
    }
    else
    {
      if (((V23BwCh*)rx)->status != 0)
      {
        for (i = 0; i < *txBitCount; i++)
          txBitSrc[i] = 1;
      }
    }
    v23FP_tx_progress(obj->tx, txAudioOut, txSampleCount, txBitSrc, txBitCount);
    if (obj->backend == 0)
    {
      return v23FP_rx_progress(FPM_M(V23Rx*, obj, rx), rxAudioIn, rxSampleCount, rxBitsOut, nRxBitsOut);
    }
    return BwChDem_Progress(FPM_M(V23BwCh*, obj, rx), rxAudioIn, rxSampleCount, rxBitsOut, nRxBitsOut);
  }

  /* state 0: answer tone until 3 seconds of samples have been sent */
  if (state == 0)
  {
    FPM_TONE_generate(obj->tone.p, txAudioOut, (short)rxSampleCount);
    rxSampleCount = obj->count.w + rxSampleCount;
    if (obj->thrTx <= rxSampleCount)
    {
      obj->count.s[0] = 0;
      obj->count.s[1] = 0;
      obj->state.s[0] = 1;
      obj->state.s[1] = 0;
      return 1;
    }
  }
  else
  {
    /* state 1: silence for rate / 20 samples */
    int i;

    if (state != 1)
      return 1;
    for (i = 0; i < rxSampleCount; i++)
      txAudioOut[i] = 0;
    rxSampleCount = obj->count.w + rxSampleCount;
    if (obj->thrSil < rxSampleCount)
    {
      obj->count.s[0] = 0;
      obj->count.s[1] = 0;
      obj->state.s[0] = 2;
      obj->state.s[1] = 0;
      return 1;
    }
  }
  obj->count.w = rxSampleCount;
  return 1;
}

/* ==================== BwChDem_*: 75 bit/s backward-channel receiver ==================== */

/**
 * @brief Create a backward-channel receiver, or re-initialise an existing one.
 * @param existing  existing object, or NULL to allocate
 * @param cfg       engine configuration block (V23Cfg); the `thr` (signal-absence threshold, 700 from the glue) becomes `noCarThr`
 * @return the receiver
 *
 * Creates the 390 Hz tone detector, the AGC and the 3-section IIR pre-filter, and starts in the "acquiring" state (`status` 1, `toneCnt` 0, bit block 0 of
 * `block_size_table`, debounce 2, last decision 1).
 * @warning The allocations are not checked; passing an existing object allocates a new tone detector and IIR state without freeing the old ones.
 */
V23BwCh* BwChDem_Create(V23BwCh* existing, const V23Cfg* cfg)
{
  V23BwCh* obj = existing;
  int i;
  int toneCfg[9];
  short* iirState;

  if (obj == 0)
  {
    obj = (V23BwCh*)sysdep_malloc(sizeof(V23BwCh));
  }

  /* bit decision state */
  obj->st[0] = 0;
  obj->st[2] = 0;
  obj->countdown = block_size_table[0];
  obj->st[1] = 0;
  obj->st[3] = 0;
  obj->mark = 0;
  obj->space = 0;
  obj->sC = 0;
  obj->timeout = 0;
  obj->rot = 0;
  obj->debounce = 2;
  obj->last = 1;

  /* tone detector, AGC, IIR pre-filter */
  buildBwChTxToneCfg(toneCfg);
  obj->tone = FPM_TONE_create(0, toneCfg);
  obj->toneCnt = 0;

  FPM_AGC_init(&obj->agc, AGCv23_BWCH_CFG_raw, 1);
  obj->iirCoef = BwCh_IIR_FILT;
  obj->iirCount = 3;
  iirState = (short*)sysdep_malloc(0x18);
  obj->iirState = iirState;
  for (i = 0; i < 12; i++)
    iirState[i] = 0;

  /* signal-absence watch */
  obj->noCarAcc = 0;
  obj->status = 1;
  obj->noCarThr = cfg->thr;
  return obj;
}

/** @brief Free a backward-channel receiver and its tone detector and IIR state; NULL is accepted. */
void BwChDem_Delete(V23BwCh* bw)
{
  V23BwCh* obj = bw;

  if (obj != 0)
  {
    FPM_TONE_delete(obj->tone);
    sysdep_free(obj->iirState);
    sysdep_free(obj);
  }
}

/**
 * @brief Process one block of the backward channel.
 * @param self       receiver
 * @param rxIn       received samples; FILTERED IN PLACE by the IIR pre-filter
 * @param rxCount    number of samples
 * @param bitsOut    receives the demodulated bits, one per word (room for rxCount / 26 + 1 is needed)
 * @param nBitsOut   receives the number of bits (cleared at entry)
 * @return 0 = demodulating (`status` cleared); 1 = still waiting for the carrier tone (`status` set); 2 = failure (time-out or loss of signal)
 *
 * Waiting phase: until `toneCnt` reaches 10 each call looks for the 390 Hz tone (consecutive blocks with it count up, a block without clears the count) and
 * counts `timeout`; more than 1000 calls without success fail.  Afterwards, a call whose AGC reports no signal adds 20 to `noCarAcc` and fails when it reaches
 * `noCarThr` (35 calls for 700); a call with signal clears it.
 * Demodulation: each sample (>> 4) goes through a mark and a space filter whose squared outputs are accumulated.  The samples are divided into blocks
 * of 26 or 27 samples (`block_size_table`, a quarter of a bit at 75 bit/s); at the end of a block the two accumulators decide the bit: when both exceed 1
 * or neither does, the bit is 1 if mark <= space, otherwise it is 1 if space exceeds 1.  A changed decision sets `debounce` to 2; a bit is emitted when
 * `debounce` has counted down to 0 (the third block of an unchanged decision), after which it is set to 3, i.e. one bit per four blocks.
 */
int BwChDem_Progress(V23BwCh* self, short* rxIn, unsigned int rxCount, unsigned int* bitsOut, int* nBitsOut)
{
  V23BwCh* obj = self;
  short agcRet;
  int idx;

  *nBitsOut = 0;

  /* pre-filter and gain control; agcRet is the AGC's signal-present flag */
  FPM_iir_filt_II(rxIn, (short*)obj->iirCoef, obj->iirState, obj->iirCount, (short)rxCount);
  FPM_AGC_agc(&obj->agc, rxIn, (unsigned short)rxCount);
  agcRet = FPM_M(short, obj, agc.detect);

  /* waiting for the 390 Hz tone */
  if (obj->toneCnt < 10)
  {
    if (FPM_TONE_detect(obj->tone, rxIn, (short)rxCount) == 0)
    {
      obj->toneCnt = obj->toneCnt + 1;
      if (1 < dsplibs_debug_level)
      {
        dsplibs_debug_printf("V23 backward channel: Tone detected 390[Hz]\n");
      }
    }
    else
    {
      obj->toneCnt = 0;
    }
    {
      short timeout = obj->timeout;

      obj->timeout = timeout + 1;
      if (1000 < timeout)
      {
        if (1 < dsplibs_debug_level)
        {
          dsplibs_debug_printf("V23Debug: Timeout waiting for V23 signal...\r\n");
        }
        return 2;
      }
    }
    obj->status = 1;
    if (obj->toneCnt < 10)
      return 1;
  }

  /* signal-absence watch */
  if (agcRet == 0)
  {
    unsigned short acc = obj->noCarAcc + 0x14;

    FPM_M(unsigned short, obj, noCarAcc) = acc;
    if (obj->noCarThr <= (int)(unsigned int)acc)
    {
      if (1 < dsplibs_debug_level)
      {
        dsplibs_debug_printf("V23 no carrier detected...\r\n");
      }
      return 2;
    }
  }
  else
  {
    obj->noCarAcc = 0;
  }

  /* demodulation: accumulate mark / space energy per block, decide a bit at each block end */
  idx = 0;
  while (idx < (int)rxCount)
  {
    short markAcc, spaceAcc;

    for (;;)
    {
      int sample = (int)rxIn[idx] >> 4;
      short markOut = FPM_iir_filt((short)sample, Mark_Coefs, &obj->st[0], 1);

      /* the previous accumulator value is ZERO-extended before the sum: the 16-bit sum can wrap over a block, and a sign-extended read would differ */
      markAcc = (short)((((unsigned int)(markOut * markOut)) >> 8) + (unsigned int)(unsigned short)obj->mark);
      obj->mark = markAcc;
      {
        short spaceOut = FPM_iir_filt((short)sample, Space_Coefs, &obj->st[2], 1);

        spaceAcc = (short)((((unsigned int)(spaceOut * spaceOut)) >> 8) + (unsigned int)(unsigned short)obj->space);
      }
      {
        int countdown = obj->countdown - 1;

        if (countdown == 0)
          break;
        obj->countdown = countdown;
        idx++;
        obj->space = spaceAcc;
        if ((int)rxCount <= idx)
          goto done;
      }
    }

    /* end of a block: decide, debounce, emit */
    {
      int spaceGT1 = spaceAcc > 1;
      int markGT1 = markAcc > 1;
      unsigned int bitVal = (spaceGT1 == markGT1) ? (unsigned int)(markAcc <= spaceAcc) : (unsigned int)spaceGT1;

      if (bitVal != (unsigned int)obj->last)
      {
        obj->debounce = 2;
      }
      obj->last = (int)bitVal;
      if (obj->debounce == 0)
      {
        obj->countdown = 0;
        obj->debounce = 3;
        bitsOut[*nBitsOut] = bitVal;
        *nBitsOut = *nBitsOut + 1;
      }
      else
      {
        obj->debounce = obj->debounce - 1;
      }
      obj->space = 0;
      obj->mark = 0;
      {
        int rot = (obj->rot + 1) % 12;
        int seed = block_size_table[rot];

        obj->rot = rot;
        idx++;
        obj->countdown = seed;
      }
    }
  }

done:
  obj->status = 0;
  return (int)obj->status;
}
