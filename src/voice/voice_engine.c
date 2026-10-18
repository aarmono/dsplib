/**
 * @file voice_engine.c
 * @brief The voice engine: mode state machine, the four mode handlers (receive, transmit, online, duplex), commands and the mu-law encoder.
 *
 * The engine (type Voice, voice.h) is driven by the glue of voice_api.c once per 160-sample, 8000 Hz block through voice_modem().  voice_modem()
 * runs the tone / DTMF detector (voice_tone.c, voice_dtmf.c) over the float input, calls the state handler of the current mode and returns an engine
 * message (VoiceMsg).  Entry points:
 * - voice_create() / voice_delete(): the engine with its beep generator, detector, transmit FIFO, silence detector and FDSP object;
 * - voice_command(): the requests of VoiceEngCmd (beep, DTMF, set mode, detector enable masks, #VLS, abort ...); voice_set_rx() / _tx() / _online() /
 *   _duplex() switch the mode, voice_dle_command() handles the DLE codes of the transmit stream;
 * - voice_rx(): receive mode.  Converts the block to 8-bit mu-law bytes for the tty (DLE-doubling a data byte 0x10, "DLE T" time marks, silence
 *   reports from the silence detector), after removing the running DC offset and applying the receive gain;
 * - voice_tx(): transmit mode.  Collects the tty bytes (DLE codes: DLE DLE data, DLE ETX end, DLE CAN abort) into the FIFO and, when a block is
 *   available, converts the 8-bit samples to float (or 16-bit pcm for #VLS 1 / 2), reporting an underrun when the FIFO runs dry;
 * - voice_online(): command / online mode.  Silence, or the beep / DTMF tone while one is generated (float, or pcm for #VLS 1 / 2);
 * - voice_duplex(): duplex mode.  The FDSP pass-through between 16-bit pcm and float, plus the beep tone;
 * - voice_linear2ulaw(): the engine's own mu-law encoder.
 *
 * Float policy (shared with voice_tone.c and voice_dtmf.c): every intermediate is a `long double`, which is the x87 format on i386 and x86-64
 * whatever -O or -mfpmath; a value is rounded to float only where the code stores into a float (F(), a volatile float store); the memory-operand
 * forms of the x87 arithmetic are inline asm; a float-to-integer conversion goes through fist16() / fist32(), which truncate and return the
 * "integer indefinite" 0x8000 / 0x80000000 for NaN and out-of-range values.
 *
 * Behaviour callers must know (docs/bugs.md, section H): (1) VOICE_create() accepts only 8000 / 9600 / 48000 Hz; at 8000 Hz both converters of the
 * glue are NULL and VOICE_process() crashes in RcFixed_Resample(); 48000 Hz (960-sample blocks) overflows the 384-sample rings.  (2) The outer loop of
 * VOICE_process() advances by the block length in bytes, so a call with more than one block reads and writes the wrong place; slmodemd passes one
 * block.  (3) voice_tx() reads 0xde bytes into the 0xc8-byte transmit buffer when the stream rate is not 8000 (never today).  (4) voice_rx() ignores
 * the result of silence_is_more_then() and treats its `inRaw` as 16-bit pcm from the tty in the #VLS 1 / 3 modes.  (5) The first sample of the first
 * receive block after a mode change uses the unrounded mean for the DC subtraction, the others the rounded float.
 */
#include <string.h>

#include "voice.h"
#include "core/dsp_asm.h"

/* COMPILER_BARRIER() (dsp_asm.h) pins the order of the memory accesses (and the re-reads of a count word the handlers take after calls), so the generated
 * code does not depend on -fstrict-aliasing */
#include "core/dsp_debug_printf.h" /* dsplibs_debug_level, dsplibs_debug_printf, DBG(...) */

#include "voice_f32.h"
/* explicit x87 memory-operand forms (see voice_x87.h for the NaN rule): `fld dword [m]`, `fadd`, `fmul`, and the reversed subtraction `m - a` */
static inline long double ldl(const float* m)
{
#if defined(__i386__) || defined(__x86_64__)
  long double r;
  X87_LOAD_F32(r, *m);
  return r;
#else
  return (long double)*m;
#endif
}
static inline long double ld_addf(long double a, const float* m)
{
#if defined(__i386__) || defined(__x86_64__)
  X87_ADD_F32(a, *m);
  return a;
#else
  return a + (long double)*m;
#endif
}
static inline long double ld_mulf(long double a, const float* m)
{
#if defined(__i386__) || defined(__x86_64__)
  X87_MUL_F32(a, *m);
  return a;
#else
  return a * (long double)*m;
#endif
}
static inline long double ld_subrf(long double a, const float* m)
{
#if defined(__i386__) || defined(__x86_64__)
  X87_SUBR_F32(a, *m); /* st0 = m - st0 */
  return a;
#else
  return (long double)*m - a;
#endif
}
#include "voice_fist16.h"
#include "core/dsp_constants.h" /* SAMPLE_RATE_* */
static inline int fist32(long double v)
{
  if (!(v > -2147483649.0L && v < 2147483648.0L))
    return (int)0x80000000u;
  return (int)v;
}

/* a 16-bit store to a buffer that need not be 2-byte aligned (voice_modem() hands the handlers `buf + n` with an arbitrary byte count n) */
static inline void put16(void* base, unsigned idx, short v) { memcpy((char*)base + 2 * (size_t)idx, &v, 2); }

/* ---- constants ---- */
static const double k_099 = 0x1.fae147ae147aep-1;    /* 0.99: weight of the old DC estimate */
static const double k_001 = 0x1.47ae147ae147bp-7;    /* 0.01: weight of the new block mean */
static const float k_30000 = 0x1.d4c0000000000p+14f; /* 30000.0: float -> pcm scale (beep tone, transmit at 16 bits) */
static const float k_128 = 0x1.0000000000000p-7f;    /* 1/128: receive gain parameter -> gain */
static const float k_32767 = 0x1.fffc000000000p+14f; /* 32767.0: float -> 16-bit sample in the receive encoder */
static const float k_1_64 = 0x1.0000000000000p-6f;   /* 1/64: transmit byte -> float */
static const float k_64 = 0x1.0000000000000p+6f;     /* 64.0: the same scale for the pcm path */
static const float k_260 = 0x1.0400000000000p+8f;    /* 260.0: float -> pcm scale of the 8-bit transmit path */
static const float k_08 = 0x1.99999a0000000p-1f;     /* 0.8: argument of the (ignored) silence query in voice_rx() */
static const uint32_t k_rxbits = 0x38000100u;        /* the float 1/32768 * (1 + 2^-15): pcm -> float scale of voice_rx() */

/* ---- mu-law encoder ---- */
/* the segment end values of the mu-law encoder */
static const short seg_end[8] = {0xff, 0x1ff, 0x3ff, 0x7ff, 0xfff, 0x1fff, 0x3fff, 0x7fff};

/**
 * @brief Encode a linear 14-bit sample as an 8-bit mu-law byte.
 * @param pcm  linear sample (the engine passes a 16-bit sample shifted right by 2)
 * @return the mu-law byte (bit-inverted code word); 0x7f / 0xff-xor for values beyond the last segment
 * @note  The library has a second encoder of the same algorithm for the V.90 code (src/vpcm, `linear2ulaw`); this one has its own name so that the
 *        voice code links without it.
 */
int voice_linear2ulaw(int pcm)
{
  unsigned xorMask = 0xff;
  int biased = (int)((unsigned)pcm + 0x84u);
  int seg;

  if (pcm < 0)
  {
    biased = (int)(0x84u - (unsigned)pcm);
    xorMask = 0x7f;
  }
  for (seg = 0; seg < 8; seg++)
    if ((int)seg_end[seg] >= biased)
      return (unsigned char)((((unsigned)(biased >> (seg + 3)) & 0xfu) | ((unsigned)seg << 4)) ^ xorMask);
  return (int)(xorMask ^ 0x7fu);
}

/* ---- the voice engine ---- */

/**
 * @brief Create the voice engine and its sub-objects.
 * @param cfg  modem, parameter callback and the two hook callbacks (copied)
 * @return the engine in the online mode (detector enable masks 0x3f, rate 8000, 8 bits, mode_prev = VOICE_MODE_NONE), or NULL when `cfg` is NULL or an
 *         allocation failed (everything allocated is released again)
 * @note  The sub-objects are the beep generator, the detector, the transmit FIFO, the silence detector and the FDSP object (created first, with the
 *        delays of STRM_VCE_GetFDSPEnvironmentalParams()).  The FDSP result is not checked; it is deleted only if non-NULL.
 */
Voice* voice_create(const VoiceCfg* cfg)
{
  Voice* v;
  BeepCfg bc;
  short rx_delay, tx_delay;
  DBG("voice_create 11.8.98 12:41\n");
  if (!cfg)
    return NULL;
  bc.ctx = cfg->ctx;
  bc.hookOn = cfg->hook_on;
  bc.hookOff = cfg->hook_off;
  bc.getParam = cfg->get_sreg;
  v = (Voice*)sysdep_malloc((unsigned)sizeof(Voice));
  if (!v)
    return NULL;
  sysdep_memset(v, 0, sizeof(Voice));
  v->ctx = cfg->ctx;
  v->cb = cfg->get_sreg;
  v->hook_on = cfg->hook_on;
  v->hook_off = cfg->hook_off;

  /* the sub-objects; the first failure releases the ones before it */
  STRM_VCE_GetFDSPEnvironmentalParams(&rx_delay, &tx_delay);
  v->fdsp = FDSP_DP_Create(NULL, rx_delay, tx_delay);
  v->beep = beepgen_create(NULL, &bc);
  if (v->beep)
  {
    v->det = detector_create(NULL, v->ctx, v->cb);
    if (v->det)
    {
      v->fifo = FIFO8_create(v->fifo, NULL);
      if (v->fifo)
      {
        v->sil = silence_create(v->sil, v->ctx, v->cb);
        if (v->sil)
        {
          /* the initial state: online mode, all detectors enabled */
          v->mode = VOICE_MODE_ONLINE;
          v->etx = 0;
          v->en_tx = 0x3f;
          v->en_rx = 0x3f;
          v->en_on = 0x3f;
          v->abort = 0;
          v->tmark = 0;
          v->rate = SAMPLE_RATE_8000;
          v->mode_prev = VOICE_MODE_NONE;
          v->fn = voice_online;
          v->vls = 0;
          v->bits = 8;
          v->beep_idle = 1;
          return v;
        }
      }
    }
  }

  /* a sub-object could not be created */
  DBG("voice delete %lX\n", (unsigned long)(uintptr_t)v);
  if (v->beep)
    beepgen_delete(v->beep);
  if (v->det)
    detector_delete(v->det);
  if (v->fifo)
    FIFO8_delete(v->fifo);
  if (v->sil)
    silence_delete(v->sil);
  if (v->fdsp)
    FDSP_DP_Delete(v->fdsp);
  sysdep_free(v);
  return NULL;
}

/**
 * @brief Delete the engine and its sub-objects.
 * @param v  the engine
 * @warning `v` is not checked for NULL.
 */
void voice_delete(Voice* v)
{
  DBG("voice delete %lX\n", (unsigned long)(uintptr_t)v);
  if (v->beep)
    beepgen_delete(v->beep);
  if (v->det)
    detector_delete(v->det);
  if (v->fifo)
    FIFO8_delete(v->fifo);
  if (v->sil)
    silence_delete(v->sil);
  if (v->fdsp)
    FDSP_DP_Delete(v->fdsp);
  sysdep_free(v);
}

/**
 * @brief Handle a DLE code of the transmit stream.
 * @param v    the engine
 * @param cmd  the byte that followed a DLE (taken as signed char): 3 (ETX) ends the transmission, 0x18 (CAN) aborts it
 * @return VOICE_MSG_CANCEL for CAN, 0 otherwise (an unknown code is logged and ignored)
 */
int voice_dle_command(Voice* v, int cmd)
{
  int c = (signed char)cmd;
  if (c == 3)
  {
    DBG("voice dle command: ETX\n");
    v->etx = 1;
    return 0;
  }
  if (c == 0x18)
  {
    DBG("voice <CAN> command\n");
    v->abort = 1;
    return VOICE_MSG_CANCEL;
  }
  DBG("Unknown command - %2x\n", c);
  return 0;
}

/** @brief Switch to the online mode: handler voice_online(), detector enable mask `en_on`. */
void voice_set_online(Voice* v)
{
  unsigned short en = v->en_on;
  v->mode = VOICE_MODE_ONLINE;
  v->fn = voice_online;
  detector_set_enable(v->det, en);
}

/** @brief Switch to the duplex mode: handler voice_duplex(), detector enable mask 0x24. */
void voice_set_duplex(Voice* v)
{
  v->mode = VOICE_MODE_DUPLEX;
  v->fn = voice_duplex;
  detector_set_enable(v->det, 0x24);
}

/** @brief Switch to the transmit mode: handler voice_tx(), mask `en_tx`, 8-bit samples at 8000 Hz, and flag an underrun until data arrives. */
void voice_set_tx(Voice* v)
{
  unsigned short en = v->en_tx;
  v->mode = VOICE_MODE_TX;
  v->fn = voice_tx;
  detector_set_enable(v->det, en);
  DBG("PCM 8 bit.\n");
  v->bits = 8;
  v->rate = SAMPLE_RATE_8000;
  v->underrun = 1;
}

/**
 * @brief Switch to the receive mode: handler voice_rx(), mask `en_rx`.
 * @note  Re-initialises the silence detector, restarts the DC estimate and the time-mark counter (tmark * 800 samples) and reads the three receive
 *        gains from the parameter callback (ids 0x48, 0x8a, 0x8b, divided by 128).
 */
void voice_set_rx(Voice* v)
{
  unsigned short en = v->en_rx;
  unsigned t;
  v->rx_on = 1;
  v->mode = VOICE_MODE_RX;
  v->fn = voice_rx;
  detector_set_enable(v->det, en);
  silence_create(v->sil, v->ctx, v->cb);

  v->tcount = (unsigned short)((unsigned)v->tmark * 800u);
  DBG("Sample rate 8000\n");
  v->rate = SAMPLE_RATE_8000;
  v->bits = 8;
  v->dc = 0.0f;
  v->first = 1;

  t = v->cb(v->ctx, 0x48);
  v->rxGainVls1 = F(LD((unsigned long long)t) * LD(k_128));
  t = v->cb(v->ctx, 0x8a);
  v->rxGainVls3 = F(LD((unsigned long long)t) * LD(k_128));
  t = v->cb(v->ctx, 0x8b);
  v->rxGainLine = F(LD((unsigned long long)t) * LD(k_128));
}

/**
 * @brief Execute an engine request.
 * @param v     the engine
 * @param cmd   a VoiceEngCmd
 * @param args  the request's arguments: BEEP {f1, f2, duration}, DTMF {symbol, duration}, SET_MODE {VoiceMode}, DETECT_ENABLE {three packed masks:
 *              byte 0 transmit, byte 1 receive, byte 2 online}, TX_LEVEL / PLAYBACK_VOLUME / TIME_MARK / VLS {value}; ignored by ABORT and RESET_DUPLEX
 * @return 0, or VOICE_MSG_ERROR for an unknown request or a beep / DTMF outside the online and duplex modes
 * @note  SET_MODE with a mode that matches no handler (VOICE_MODE_NONE) leaves the mode unchanged.  Every SET_MODE marks the beep generator idle.
 *        PLAYBACK_VOLUME is stored and never used; TX_LEVEL only logs.
 * @warning `args` must point to as many ints as the request needs.
 */
int voice_command(Voice* v, int cmd, const int* args)
{
  DBG("voice_command # %d\n", cmd);
  switch ((unsigned)cmd)
  {
  case VOICE_ECMD_BEEP: /* two-tone beep (online and duplex modes only) */
    if (((unsigned)v->mode - 2u) > 1u)
      goto not_online;
    DBG("call beep gen with: %d %d %d\n", args[0], args[1], args[2]);
    FDSP_Kernel_SetInternalBeepInProgress(1);
    beepgen_start_beep(v->beep, args[0], args[1], args[2]);
    v->beep_idle = 0;
    return 0;
  case VOICE_ECMD_DTMF: /* DTMF digit (online and duplex modes only) */
    if (((unsigned)v->mode - 2u) > 1u)
      goto not_online;
    FDSP_Kernel_SetInternalBeepInProgress(1);
    beepgen_start_dtmf(v->beep, args[0], args[1]);
    v->beep_idle = 0;
    return 0;
  case VOICE_ECMD_SET_MODE: /* switch the mode (an unknown mode value changes nothing) */
    switch (args[0])
    {
    case VOICE_MODE_RX:
      voice_set_rx(v);
      break;
    case VOICE_MODE_TX:
      voice_set_tx(v);
      break;
    case VOICE_MODE_ONLINE:
    {
      unsigned short en = v->en_on;
      v->mode = VOICE_MODE_ONLINE;
      v->fn = voice_online;
      detector_set_enable(v->det, en);
      break;
    }
    case VOICE_MODE_DUPLEX:
      v->mode = VOICE_MODE_DUPLEX;
      v->fn = voice_duplex;
      detector_set_enable(v->det, 0x24);
      break;
    default:
      break;
    }
    v->beep_idle = 1;
    return 0;
  case VOICE_ECMD_TX_LEVEL:
    DBG("VOICE_OUTPUT_TRANSMIT_LEVEL_COMMAND %d\n", args[0]);
    return 0;
  case VOICE_ECMD_DETECT_ENABLE:
  {
    int w = args[0];
    v->en_tx = (unsigned short)(unsigned char)w;
    v->en_rx = (unsigned short)(unsigned char)(w >> 8);
    v->en_on = (unsigned short)((w >> 16) & 0xff);
    return 0;
  }
  case VOICE_ECMD_PLAYBACK_VOLUME:
    DBG("VOICE_PLAYBACK_VOLUME_COMMAND %d\n", args[0]);
    v->playvol = (unsigned short)args[0];
    return 0;
  case VOICE_ECMD_TIME_MARK:
    DBG("VOICE_TIME_MARK_COMMAND %d\n", args[0]);
    v->tmark = (unsigned short)args[0];
    return 0;
  case VOICE_ECMD_VLS:
    DBG("VOICE_VLS_COMMAND %d\n", args[0]);
    v->vls = args[0];
    return 0;
  case VOICE_ECMD_ABORT:
    DBG("VOICE_ABORT_COMMAND\n");
    v->abort = 1;
    return 0;
  case VOICE_ECMD_RESET_DUPLEX:
  {
    short rx_delay, tx_delay;
    STRM_VCE_GetFDSPEnvironmentalParams(&rx_delay, &tx_delay);
    v->fdsp = FDSP_DP_Create(v->fdsp, rx_delay, tx_delay);
    DBG("VOICE_RESET_DUPLEX_COMMAND\n");
    return 0;
  }
  default:
    break; /* request 4 and everything above 10: unknown */
  }
  return VOICE_MSG_ERROR;
not_online:
  DBG("modem not in online or duplex\n");
  return VOICE_MSG_ERROR;
}

/* ---- the four state handlers: (v, inRaw = tty bytes / pcm shorts, outFloat = float out, inFloat = float in, outRaw = byte / short out,
 *      *inCount = input bytes in / space left out, *sampleCount = samples in / output bytes out) ---- */

/**
 * @brief Online mode: silence, or the beep / DTMF tone while one is being generated.
 * @param v            the engine
 * @param inRaw        unused
 * @param outFloat     float output block, *sampleCount samples (used for #VLS other than 1 and 2)
 * @param inFloat      unused
 * @param outRaw       16-bit pcm output (#VLS 1 and 2)
 * @param inCount      set to 0
 * @param sampleCount  samples in the block; set to 0 (no output bytes)
 * @return VOICE_MSG_OK when a tone has just ended, VOICE_MSG_START_ONLINE when the mode was entered from another mode than receive, otherwise 0
 * @note  While a tone is generated the samples come from the beep generator (scaled by 30000 for the pcm output); the rest of the block is zero.
 */
int voice_online(Voice* v, void* inRaw, float* outFloat, float* inFloat, void* outRaw, unsigned short* inCount, unsigned short* sampleCount)
{
  int ret = 0;
  int st = 0;
  unsigned outPos = 0, limit;
  short* o = (short*)outRaw;
  (void)inRaw;
  (void)inFloat;

  if (v->beep_idle == 0)
  {
    /* a tone is being generated: tone samples until the generator reports its end, then zeros */
    float tmp;
    if (((unsigned)v->vls - 1u) <= 1u)
    {
      COMPILER_BARRIER();
      limit = *sampleCount;
      if (limit != 0)
      {
        do
        {
          st = beepgen_sample(v->beep, &tmp);
          put16(o, outPos, fist16(ld_mulf(ldl(&tmp), &k_30000)));
          outPos = (unsigned short)(outPos + 1);
          COMPILER_BARRIER();
          limit = *sampleCount;
        } while (limit > outPos && st != 1);
      }
      if (limit > outPos)
      {
        do
        {
          put16(o, outPos, 0);
          outPos = (unsigned short)(outPos + 1);
          COMPILER_BARRIER();
          limit = *sampleCount;
        } while (limit > outPos);
      }
    }
    else
    {
      COMPILER_BARRIER();
      limit = *sampleCount;
      if (limit != 0)
      {
        do
        {
          st = beepgen_sample(v->beep, &outFloat[outPos]);
          outPos = (unsigned short)(outPos + 1);
          COMPILER_BARRIER();
          limit = *sampleCount;
        } while (limit > outPos && st != 1);
      }
      while (limit > outPos)
      {
        outFloat[outPos] = 0.0f;
        outPos = (unsigned short)(outPos + 1);
      }
    }
    if (st == 1)
    {
      DBG("beepgend end, send ok\n");
      v->beep_idle = 1;
      FDSP_Kernel_SetInternalBeepInProgress(0);
      COMPILER_BARRIER();
      limit = *sampleCount;
      ret = VOICE_MSG_OK;
    }
    *inCount = (unsigned short)limit;
  }
  else
  {
    /* idle: a block of silence */
    unsigned i = 0;
    if (((unsigned)v->vls - 1u) <= 1u)
    {
      COMPILER_BARRIER();
      limit = *sampleCount;
      while (limit > i)
      {
        put16(o, i, 0);
        i = (unsigned short)(i + 1);
        COMPILER_BARRIER();
        limit = *sampleCount;
      }
    }
    else
    {
      COMPILER_BARRIER();
      limit = *sampleCount;
      while (limit > i)
      {
        outFloat[i] = 0.0f;
        i = (unsigned short)(i + 1);
      }
    }
    *inCount = (unsigned short)limit;
  }

  *sampleCount = 0;
  COMPILER_BARRIER();
  {
    int mp = v->mode_prev;
    *inCount = 0;
    COMPILER_BARRIER();
    if (mp != v->mode && mp != VOICE_MODE_RX)
      ret = VOICE_MSG_START_ONLINE;
  }
  return ret;
}

/**
 * @brief Duplex mode: the FDSP pass-through between pcm and float, plus the beep tone.
 * @param v            the engine
 * @param inRaw        16-bit pcm input, *sampleCount samples
 * @param outFloat     float output (pcm input / 32000; the beep generator overwrites it while a tone is generated)
 * @param inFloat      float input
 * @param outRaw       16-bit pcm output (float input * 32000)
 * @param inCount      passed to FDSP_DP_Run() (ignored)
 * @param sampleCount  samples in the block
 * @return VOICE_MSG_START_DUPLEX when the mode was just entered, else VOICE_MSG_OK when a tone has just ended, else 0
 * @warning The glue passes its tty byte buffer as `inRaw` and does not fill it in the duplex state, so the float output is derived from whatever
 *          that buffer last held.
 */
int voice_duplex(Voice* v, void* inRaw, float* outFloat, float* inFloat, void* outRaw, unsigned short* inCount, unsigned short* sampleCount)
{
  int ret = 0;

  FDSP_DP_Run(v->fdsp, (const short*)inRaw, outFloat, inFloat, (short*)outRaw, inCount, sampleCount);

  /* a tone replaces the pass-through output until the generator ends */
  if (v->beep_idle == 0)
  {
    unsigned outPos = 0;
    int st = 0;
    if (*sampleCount != 0)
    {
      do
      {
        st = beepgen_sample(v->beep, &outFloat[outPos]);
        outPos = (unsigned short)(outPos + 1);
      } while (*sampleCount > outPos && st != 1);
      if (st == 1)
      {
        DBG("beepgend end, send ok\n");
        v->beep_idle = 1;
        ret = VOICE_MSG_OK;
        FDSP_Kernel_SetInternalBeepInProgress(0);
      }
    }
  }

  if (v->mode_prev != v->mode)
    ret = VOICE_MSG_START_DUPLEX;
  return ret;
}

/**
 * @brief Transmit mode: tty bytes -> FIFO -> one block of samples.
 * @param v            the engine
 * @param inRaw        tty bytes, *inCount of them (DLE DLE = data byte 0x10, DLE ETX = end of data, DLE CAN = abort)
 * @param outFloat     float output block (#VLS other than 1 and 2)
 * @param inFloat      unused
 * @param outRaw       16-bit pcm output (#VLS 1 and 2), *sampleCount samples
 * @param inCount      in: bytes in `inRaw`; out: how many more bytes the tty may send (0xaa minus the FIFO fill, at least 0)
 * @param sampleCount  in: samples wanted; out: 0 (no output bytes)
 * @return VOICE_MSG_UNDERRUN once when the FIFO runs dry (not after DLE ETX), VOICE_MSG_CANCEL after DLE CAN, VOICE_MSG_START_TX when the mode was
 *         just entered (this overrides the others), else 0
 * @note  The block is produced only when the FIFO holds a full block for the stream format (8-bit samples at 8000 Hz: 160 bytes); the block mean is
 *        removed.  Otherwise the block is zeros.
 * @bug   A DLE as the last byte of the chunk makes the code read `inRaw[*inCount]`, one byte past the data, as the DLE code; the byte is whatever
 *        the tty buffer held before (the glue's buffer is 0x400 bytes, so nothing is read out of bounds).  A DLE split across two tty reads can
 *        therefore cancel the transmission or end it by chance (found by reading; docs/bugs.md does not list it).
 * @warning With a stream rate other than 8000 the 0xde-byte read overlaps the float buffer behind the transmit buffer (nothing sets another rate).
 */
int voice_tx(Voice* v, void* inRaw, float* outFloat, float* inFloat, void* outRaw, unsigned short* inCount, unsigned short* sampleCount)
{
  const unsigned char* in = (const unsigned char*)inRaw;
  short* o = (short*)outRaw;
  unsigned char* txb = v->area.b;
  int ret = 0;
  unsigned want = 0xa0;
  float* dest = outFloat;
  unsigned i = 0, n = 0;
  int enough;
  unsigned fifoCount;
  (void)inFloat;

  /* where the block is built, and how many bytes it takes */
  if (v->rate != SAMPLE_RATE_8000)
  {
    dest = v->area.fl.f;
    want = (v->rate == SAMPLE_RATE_7200) ? 0x90u : 0xdeu;
  }
  if (((unsigned)v->vls - 1u) <= 1u)
    dest = v->area.fl.f;

  /* split the tty bytes into data and DLE codes */
  if (*inCount != 0)
  {
    do
    {
      unsigned c = in[i];
      if (c != 0x10)
      {
        txb[n] = (unsigned char)c;
        i = (unsigned short)(i + 1);
        n = (unsigned short)(n + 1);
      }
      else
      {
        i = (unsigned short)(i + 1);
        c = in[i];
        if (c == 0x10)
        {
          DBG("TX: double dle\n");
          c = in[i];
          txb[n] = (unsigned char)c;
          i = (unsigned short)(i + 1);
          n = (unsigned short)(n + 1);
        }
        else
        {
          DBG("TX: shel comando dle\n");
          c = in[i];
          ret = voice_dle_command(v, (signed char)c);
          i = (unsigned short)(i + 1);
        }
      }
    } while (*inCount > i);
  }

  /* queue the data and see whether a block is complete (more than one block of bytes for the format: bits << 16 | rate) */
  FIFO8_write(v->fifo, txb, (unsigned short)n);
  fifoCount = v->fifo->count;
  switch (((unsigned)v->bits << 16) | (unsigned)v->rate)
  {
  case 0x81f40:
    enough = fifoCount > 0x9f;
    break;
  case 0x81c20:
    enough = fifoCount > 0x8f;
    break;
  case 0x82b11:
    enough = fifoCount > 0xdb;
    break;
  case 0x41f40:
    enough = fifoCount > 0x4f;
    break;
  case 0x41c20:
    enough = fifoCount > 0x47;
    break;
  case 0x42b11:
    enough = fifoCount > 0x6d;
    break;
  default:
    enough = 0;
    break;
  }

  if (enough)
  {
    /* one block: bytes -> floats (1/64 per step) minus the block mean, then pcm for #VLS 1 / 2 */
    unsigned cnt, k, vls1;
    long double mean, sum = 0.0L;
    v->underrun = 0;
    cnt = (unsigned short)FIFO8_read(v->fifo, txb, (unsigned short)want);
    for (k = 0; k < cnt; k++)
    {
      long double t = LD((unsigned short)txb[k]) * LD(k_1_64);
      sum = LD(F(sum + t));
    }
    mean = sum / LD(cnt);
    vls1 = ((unsigned)v->vls - 1u);
    if (vls1 > 1u)
    {
      for (k = 0; k < cnt; k++)
      {
        long double t = LD((unsigned short)txb[k]) * LD(k_1_64);
        dest[k] = F(t - mean);
      }
    }
    else
    {
      long double m64 = ld_mulf(mean, &k_64);
      for (k = 0; k < cnt; k++)
        dest[k] = F(LD((unsigned short)txb[k]) - m64);
    }
    if (vls1 <= 1u)
    {
      long double scale = (v->bits == 8) ? LD(k_260) : LD(k_30000);
      for (k = 0; k < *sampleCount; k++)
        put16(o, k, (short)fist32(ldl(&dest[k]) * scale));
    }
  }
  else
  {
    /* not enough data: report the underrun once and send silence */
    unsigned k;
    DBG("\n ** NO enouch data  ** \n");
    if (v->underrun == 0 && v->etx == 0)
    {
      DBG("VOICE TX: not enough data to tx\n");
      ret = VOICE_MSG_UNDERRUN;
      v->underrun = 1;
    }
    if (((unsigned)v->vls - 1u) > 1u)
    {
      for (k = 0; k < 0xa0; k++)
        outFloat[k] = 0.0f;
    }
    else
    {
      for (k = 0; k < *sampleCount; k++)
        put16(o, k, 0);
    }
  }

  /* how much the tty may send next */
  {
    short s = (short)(0xaa - (int)v->fifo->count);
    *inCount = (unsigned short)(s < 0 ? 0 : s);
    *sampleCount = 0;
    if (v->mode_prev != v->mode)
      ret = VOICE_MSG_START_TX;
  }
  return ret;
}

/**
 * @brief Receive mode: one block of line audio -> mu-law bytes for the tty.
 * @param v            the engine
 * @param inRaw        16-bit pcm from the tty (#VLS 1 and 3 only)
 * @param outFloat     unused
 * @param inFloat      float input block (#VLS other than 1 and 3); the DC offset is removed in place
 * @param outRaw       receives the tty bytes: mu-law samples with DLE doubling, "DLE T" marks, the silence detector's reports, "DLE ETX" after an abort
 * @param inCount      set to 0
 * @param sampleCount  in: samples in the block (at most 200); out: number of bytes written to `outRaw`
 * @return VOICE_MSG_START_RX when the mode was just entered (overrides the others), VOICE_MSG_START_ONLINE_AFTER_ABORT after an abort,
 *         VOICE_MSG_ERROR when the pcm block exceeds 200 samples, otherwise 0
 * @note  The sample is the received value minus a running mean (0.01 * block mean + 0.99 * previous mean; the first block uses its own mean),
 *        times the receive gain of the #VLS setting, as a 14-bit linear value through voice_linear2ulaw().  After an abort the bytes stop until the
 *        mode is left ("RX WAIT ABORT").
 * @warning The pcm path reads `*sampleCount` shorts from `inRaw` whatever the tty delivered (stale bytes of the glue's buffer).  The result of
 *          silence_is_more_then() is ignored.  `outRaw` must have room for one byte per sample plus the "DLE T" mark, the silence report and "DLE ETX".
 */
int voice_rx(Voice* v, void* inRaw, float* outFloat, float* inFloat, void* outRaw, unsigned short* inCount, unsigned short* sampleCount)
{
  unsigned char* buf = (unsigned char*)outRaw;
  const short* pcm = (const short*)inRaw;
  float* src = v->area.fl.f;
  int ret = 0;
  int sn = (short)*sampleCount;
  unsigned un = (unsigned short)*sampleCount;
  unsigned blen = 0, i;
  (void)outFloat;

  /* remove the DC offset: block mean, running estimate, subtract */
  if (un != 0)
  {
    long double sum = 0.0L, mean, first_val;
    for (i = 0; i < un; i++)
      sum = ld_addf(sum, &inFloat[i]);
    mean = sum / LD(un);
    if (v->first == 0)
    {
      float t = F(LD(k_001) * mean + ld_mulf(LD(k_099), &v->dc)); /* 0.01 * mean + 0.99 * previous estimate */
      first_val = LD(t);
    }
    else
    {
      v->first = 0;
      first_val = mean;
    }
    v->dc = F(first_val);
    for (i = 0; i < un; i++)
    {
      long double base = (i == 0) ? first_val : ldl(&v->dc);
      inFloat[i] = F(ld_subrf(base, &inFloat[i]));
    }
  }

  silence_is_more_then(v->sil, k_08);
  if (v->rx_on == 1)
  {
    /* apply the receive gain of the #VLS setting; src is the float block the encoder reads */
    unsigned vl = (unsigned)v->vls;
    if (vl == 3 || vl == 1)
    {
      uint32_t bits = k_rxbits;
      float c;
      long double gain;
      memcpy(&c, &bits, 4);
      COMPILER_BARRIER();
      gain = (vl == 1) ? ld_mulf(LD(c), &v->rxGainVls1) : ld_mulf(LD(c), &v->rxGainVls3);
      unsigned m = (unsigned short)*sampleCount;
      if (m > 0xc8)
      {
        DBG("rx buffer greater than internal\n");
        return VOICE_MSG_ERROR;
      }
      for (i = 0; i < m; i++)
        v->area.fl.f[i] = F(LD(pcm[i]) * gain);
    }
    else
    {
      long double g = ldl(&v->rxGainLine);
      unsigned m = (unsigned short)*sampleCount;
      src = inFloat;
      for (i = 0; i < m; i++)
        inFloat[i] = F(ldl(&inFloat[i]) * g);
    }

    /* encode the block as mu-law bytes, with a "DLE T" mark every tmark seconds */
    for (i = 0; i < un; i++)
    {
      int x = (int)fist16(ld_mulf(ldl(&src[i]), &k_32767)) >> 2;
      unsigned char b = (unsigned char)voice_linear2ulaw(x);
      buf[blen] = b;
      if (b == 0x10)
      {
        blen = (unsigned short)(blen + 1);
        buf[blen] = 0x10;
      }
      blen = (unsigned short)(blen + 1);
      if (v->tmark != 0)
      {
        unsigned short t = (unsigned short)(v->tcount - 1);
        v->tcount = t;
        if (t == 0)
        {
          buf[blen] = 0x10;
          buf[blen + 1] = 0x54;
          blen = (unsigned short)(blen + 2);
          v->tcount = (unsigned short)((unsigned)v->tmark * 800u);
        }
      }
    }

    /* silence reports, then the end of the data after an abort */
    *sampleCount = (unsigned short)blen;
    silence_progress(v->sil, src, (short)sn, buf + blen, sampleCount);
    if (v->abort != 0)
    {
      DBG("****** Send DLE ETX\n");
      _status(buf + *sampleCount, sampleCount, 3);
      v->rx_on = 0;
      detector_set_enable(v->det, 0);
      ret = VOICE_MSG_START_ONLINE_AFTER_ABORT;
    }
  }
  else
  {
    /* receiving has stopped; wait for the mode to change */
    DBG("RX WAIT ABORT\n");
    *sampleCount = 0;
  }
  *inCount = 0;
  COMPILER_BARRIER();
  if (v->mode_prev != v->mode)
    ret = VOICE_MSG_START_RX;
  return ret;
}

/**
 * @brief Run the engine for one block: detector, then the handler of the current mode.
 * @param v            the engine
 * @param inRaw        tty bytes / pcm shorts for the handler
 * @param outFloat     float output block
 * @param inFloat      float input block (also scanned by the detector)
 * @param outRaw       byte output: the detector's "DLE <char>" reports first, then the handler's bytes
 * @param inCount      handler: input bytes in / bytes still wanted out
 * @param sampleCount  in: samples in the block; out: number of bytes written to `outRaw` (detector plus handler)
 * @return VOICE_MSG_BUSY, _DIALTONE or _FAX_TONE when the detector reports a tone (these replace the handler's message), else the handler's message
 * @note  Records the mode as `mode_prev` after the handler. After a DLE ETX or an abort the engine returns to the online mode.  In the duplex mode
 *        the detector reports through status codes, in the others through the output stream.
 */
int voice_modem(Voice* v, void* inRaw, float* outFloat, float* inFloat, void* outRaw, unsigned short* inCount, unsigned short* sampleCount)
{
  unsigned short detLen = 0, engineLen = *sampleCount;
  short cnt;
  int st, ret;

  /* the detector reports into the stream, except in the duplex mode */
  if (v->mode == VOICE_MODE_DUPLEX)
    detector_set_output_status(v->det);
  else
    detector_set_output_in_stream(v->det);
  COMPILER_BARRIER();
  cnt = (short)*sampleCount;
  COMPILER_BARRIER();
  st = detector_progress(v->det, inFloat, cnt, (unsigned char*)outRaw, &detLen);

  /* the handler of the current mode, writing behind the detector's bytes */
  ret = v->fn(v, inRaw, outFloat, inFloat, (unsigned char*)outRaw + detLen, inCount, &engineLen);
  *sampleCount = (unsigned short)(detLen + engineLen);
  COMPILER_BARRIER();
  v->mode_prev = v->mode;
  COMPILER_BARRIER();

  /* the end of a transmission or an abort returns the engine to the online mode */
  if (v->etx != 0 || v->abort != 0)
  {
    unsigned short en = v->en_on;
    v->mode = VOICE_MODE_ONLINE;
    v->fn = voice_online;
    detector_set_enable(v->det, en);
    v->etx = 0;
    v->abort = 0;
  }

  if (st == 1)
    return VOICE_MSG_BUSY;
  if (st == 2)
    return VOICE_MSG_DIALTONE;
  if (st == 4)
    return VOICE_MSG_FAX_TONE;
  return ret;
}
