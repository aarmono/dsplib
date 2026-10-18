/**
 * @file voice_fdsp.c
 * @brief The voice engine's FDSP block: duplex sample conversion and the fixed delay parameters.
 *
 * Entry points (types in voice_fdsp.h): FDSP_DP_Create() / FDSP_DP_Delete() manage the FdspObj and its heap blocks; FDSP_Kernel_InitObj() clears
 * the history and channel blocks; FDSP_DP_Run() is the per-block call of the duplex mode (voice_duplex() in voice_engine.c);
 * FDSP_Kernel_SetInternalBeepInProgress() records whether the engine is generating a beep; STRM_VCE_GetFDSPEnvironmentalParams() returns the two delays.
 *
 * FDSP_DP_Run() is the only arithmetic.  Loop 1 converts the 16-bit input to float, in1[i] * (1/32000): a 16-bit integer times a 24-bit mantissa
 * is exact in 40 bits, so one rounding at the float store gives the same result with x87 or SSE code.  Loop 2 converts the float input to 16 bits,
 * in2[i] * 32000 truncated toward zero: a 24 x 24 bit mantissa product is exact in a double, so it is formed in double and truncated.  NaN and
 * out-of-range products give -32768 explicitly, as the x87 `fistp` does, so the result does not depend on the FPU control word or on the
 * code the compiler generates.
 *
 * FDSP_DP_Create() checks every allocation: on a failure it frees what it has allocated and returns NULL.  The history, channel and tail blocks
 * are written only by FDSP_Kernel_InitObj() and read by nothing else in the library (the echo-cancelling kernel that used them is not present).
 */
#include "voice_fdsp.h"

/* file-scope state that is written but never read: the last object created, a correlation report counter and the internal-beep flag */
static FdspObj* pGlobalFDSPObj;
static unsigned int uCorrelationReportsNo;
static int bInternalBeepInProgress;

/* the sample scales of FDSP_DP_Run(): 1/32000 as a float, and 32000 */
static const float fdsp_c1 = 0x1.0624dep-15f;
static const float fdsp_c2 = 32000.0f;
/* initial gain of the receive channel, 0.032 as a float */
#define FDSP_GAIN_RX 0x1.0624dcp-5f

/**
 * @brief Reset an FDSP object: clear the history, both channels and their tail blocks and set the default parameters.
 * @param o  object with its three blocks allocated (hist, ch14, ch18 and the two tails)
 * @note  Sets mode 2, w8 = 0x50, wc = 0x28, the receive channel gain to 0.032 and the transmit channel gain to 0.
 * @warning The blocks are not checked for NULL.
 */
void FDSP_Kernel_InitObj(FdspObj* o)
{
  FdspHist* h = o->hist;
  FdspChan* c = o->ch14;
  FdspChan* b = o->ch18;
  int32_t* bt = b->tail; /* read once: the stores below must not be assumed to alias the pointer member (the code must not depend on the aliasing rules) */
  int32_t* ct = c->tail;
  unsigned i;

  /* history rings */
  for (i = 0; i <= 0x7cf; i++)
  {
    h->x[i] = 0;
    h->y[i] = 0;
  }

  /* per-channel scalar state and the object's parameters */
  b->v16a4 = 0;
  for (i = 0; i < 4; i++)
    b->v1694[i] = 0;
  c->v16a4 = 0;
  for (i = 0; i < 4; i++)
    c->v1694[i] = 0;
  o->w4 = 0;
  h->z[0] = 0;
  h->z[1] = 0;
  h->z[2] = 0;
  c->gain = FDSP_GAIN_RX;
  o->mode = 2;
  b->gain = 0.0f;
  o->w8 = 0x50;
  o->wc = 0x28;
  c->s1690 = 0;
  b->s1690 = 0;

  /* channel word arrays and tail blocks */
  for (i = 0; i < 0x500; i++)
    b->w[i] = 0;
  for (i = 0; i < 0xf0; i++)
    bt[i] = 0;
  for (i = 0; i < 0x500; i++)
    c->w[i] = 0;
  for (i = 0; i < 0xf0; i++)
    ct[i] = 0;
}

/**
 * @brief Create an FDSP object, or re-initialise an existing one.
 * @param obj       NULL to allocate the object and its five blocks, or an existing object to re-initialise in place
 * @param rx_delay  receive-side delay in samples (stored in ch14 on creation; a negative value gives mode 0)
 * @param tx_delay  transmit-side delay in samples (stored in ch18 on creation)
 * @return the object, or NULL when an allocation failed (everything allocated is freed again)
 * @note  The delays are stored only when the object is allocated; a re-initialisation keeps the old ones (only the sign of `rx_delay` is used again).
 */
FdspObj* FDSP_DP_Create(FdspObj* obj, short rx_delay, short tx_delay)
{
  FdspObj* o = obj;

  if (dsplibs_debug_level > 1)
    dsplibs_debug_printf("ver 120 sRxSamplesDelay %d ,sTxSamplesDelay %d \n", (int)rx_delay, (int)tx_delay);
  if (o != NULL)
  {
    if (dsplibs_debug_level > 1)
      dsplibs_debug_printf("Reinitialization requested: \n");
  }
  else
  {
    /* the allocation chain works on locals and stores the members once at the end (the code must not depend on the aliasing rules) */
    FdspHist* h = NULL;
    FdspChan *b = NULL, *c = NULL;
    int32_t *bt = NULL, *ct = NULL;

    o = (FdspObj*)sysdep_malloc((unsigned)sizeof(FdspObj));
    if (o != NULL)
    {
      h = (FdspHist*)sysdep_malloc((unsigned)sizeof(FdspHist));
      if (h != NULL)
        b = (FdspChan*)sysdep_malloc((unsigned)sizeof(FdspChan));
      if (b != NULL)
        c = (FdspChan*)sysdep_malloc((unsigned)sizeof(FdspChan));
      if (c != NULL)
        bt = (int32_t*)sysdep_malloc(FDSP_TAIL_BLOB_SIZE);
      if (bt != NULL)
        ct = (int32_t*)sysdep_malloc(FDSP_TAIL_BLOB_SIZE);
    }
    if (ct == NULL)
    { /* some allocation failed: free what exists (history, receive channel, transmit tail, transmit channel, object) */
      if (h != NULL)
        sysdep_free(h);
      if (c != NULL)
        sysdep_free(c); /* (ch14->tail is NULL here) */
      if (b != NULL)
      {
        if (bt != NULL)
          sysdep_free(bt);
        sysdep_free(b);
      }
      if (o != NULL)
        sysdep_free(o);
      pGlobalFDSPObj = NULL;
      uCorrelationReportsNo = 0;
      return NULL;
    }
    o->hist = h;
    o->ch18 = b;
    o->ch14 = c;
    b->tail = bt;
    c->tail = ct;
    c->delay = rx_delay;
    b->delay = tx_delay;
  }
  FDSP_Kernel_InitObj(o);
  o->mode = rx_delay >= 0 ? 2 : 0;
  pGlobalFDSPObj = o;
  uCorrelationReportsNo = 0;
  return o;
}

/**
 * @brief Free an FDSP object and its blocks.
 * @param o  the object; NULL is ignored
 */
void FDSP_DP_Delete(FdspObj* o)
{
  FdspChan *c, *b;

  if (o == NULL)
    return;
  if (o->hist != NULL)
    sysdep_free(o->hist);
  c = o->ch14;
  if (c != NULL)
  {
    if (c->tail != NULL)
      sysdep_free(c->tail);
    sysdep_free(c);
  }
  b = o->ch18;
  if (b != NULL)
  {
    if (b->tail != NULL)
      sysdep_free(b->tail);
    sysdep_free(b);
  }
  sysdep_free(o);
  pGlobalFDSPObj = NULL;
}

/**
 * @brief Convert one block between 16-bit and float samples (the duplex pass-through).
 * @param o       the object (its mode is set to 2)
 * @param in1     16-bit input, *n samples
 * @param out1    float output, out1[i] = in1[i] / 32000
 * @param in2     float input, *n samples
 * @param out2    16-bit output, out2[i] = in2[i] * 32000 truncated toward zero (NaN and out-of-range give -32768)
 * @param unused  ignored
 * @param n       sample count
 * @return 1
 */
int FDSP_DP_Run(FdspObj* o, const short* in1, float* out1, const float* in2, short* out2, void* unused, const unsigned short* n)
{
  unsigned count = *n;
  unsigned i;
  (void)unused;

  for (i = 0; i < count; i++)
    out1[i] = (float)in1[i] * fdsp_c1;
  for (i = 0; i < count; i++)
  {
    double p = (double)in2[i] * (double)fdsp_c2;                             /* exact: 24 x 24 bit mantissas */
    out2[i] = (p > -32768.0 && p < 32768.0) ? (short)(int)p : (short)-32768; /* truncate toward zero; overflow and NaN give 0x8000 */
  }
  o->mode = 2;
  return 1;
}

/**
 * @brief Record that the engine starts (1) or stops (0) generating an internal beep; prints "Internal Beep ON / OFF" at debug level 2.
 * @note  The flag is not read by anything.
 */
void FDSP_Kernel_SetInternalBeepInProgress(int on)
{
  bInternalBeepInProgress = on;
  if (dsplibs_debug_level > 1)
    dsplibs_debug_printf("Internal Beep %s \n", on ? "ON" : "OFF");
}

/**
 * @brief Return the FDSP delay parameters: *rx_delay = 0x33 and *tx_delay = 0x171 samples.
 * @note  At debug level 2 the previous contents of the two outputs are printed first; callers pass uninitialised variables, so the "old" numbers are
 *        indeterminate.
 */
void STRM_VCE_GetFDSPEnvironmentalParams(short* rx_delay, short* tx_delay)
{
  unsigned lvl = dsplibs_debug_level;

  if (lvl > 1)
  {
    unsigned lvl2;
    dsplibs_debug_printf("voice: StrmVCE old: *psFarEchoDelay %d ,*psNearEchoDelay %d \n", (int)*rx_delay, (int)*tx_delay);
    lvl2 = dsplibs_debug_level; /* read before the two stores, so that the code does not depend on type based alias analysis */
    *rx_delay = 0x33;
    *tx_delay = 0x171;
    if (lvl2 > 1)
      dsplibs_debug_printf("voice: StrmVCE new: *psFarEchoDelay %d ,*psNearEchoDelay %d \n", (int)*rx_delay, (int)*tx_delay);
    return;
  }
  *rx_delay = 0x33;
  *tx_delay = 0x171;
}
