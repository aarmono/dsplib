/**
 * @file dp_wrapper.c
 * @brief DpWrapper: the generic adapter between slmodemd's block size and sample rate and a data pump that wants fixed blocks at its own rate.
 *
 * Used by the Bell 103 / V.21, V.23, V.22 and V.32 glue (core/dp_glue.h has the DpWrapper layout).  The wrapper owns two rings of 384 shorts (two halves
 * of up to 192 samples each) and, when the modem rate differs from the engine rate, one RcFixed resampler per direction (core/rcfixed_resampler.h).
 *
 * - dp_wrapper_create(owner, cb, frag, rate, engineRate): `frag` is the number of samples per engine block (0 < frag <= 0xc0), `rate` the modem's
 *   sample rate and `engineRate` the engine's.  The block length at the modem rate is blk = frag * rate / engineRate (32-bit signed product and
 *   division).  Equal rates: no converters.  Otherwise one RcFixed per direction is chosen by the rate pair (8000, 9600 and 48000 Hz only; see
 *   rc_index()); an unsupported pair or a failed RcFixed_Create() gives NULL.  The output ring starts holding one block of silence (a latency of
 *   one block).
 * - dp_wrapper_run(dp, in, out, cnt): the process callback of the pump.  It copies the input into a two-half ring and, whenever a half is complete
 *   (blk samples), runs the engine callback on it (through rcIn / rcOut when present), then copies `cnt` samples out of the output ring.
 * - dp_wrapper_delete(w): clears the owner, deletes the converters and frees the wrapper.
 *
 * The wrapper has a fixed size, so blk <= 192 and an engine block (after rcIn) <= 192 are the caller's duty (see the @warning on dp_wrapper_create()).
 * Every handle in the wrapper is a real pointer.
 */
#include "dp_glue.h"
#include "dsp_constants.h" /* SAMPLE_RATE_* */

/* The rate pairs with a converter; the result is the RcFixed_Create() index of the conversion `from` -> `to`, or 0 when there is none. */
static unsigned rc_index(int from, int to)
{
  if (from == SAMPLE_RATE_8000 && to == SAMPLE_RATE_9600)
    return 2;
  if (from == SAMPLE_RATE_9600 && to == SAMPLE_RATE_8000)
    return 3;
  if (from == SAMPLE_RATE_8000 && to == SAMPLE_RATE_48000)
    return 4;
  if (from == SAMPLE_RATE_48000 && to == SAMPLE_RATE_8000)
    return 5;
  if (from == SAMPLE_RATE_9600 && to == SAMPLE_RATE_48000)
    return 6;
  if (from == SAMPLE_RATE_48000 && to == SAMPLE_RATE_9600)
    return 7;
  return 0;
}

/**
 * @brief Create a rate / block adapter around an engine block processor.
 * @param owner       the data pump (`struct dp`) that owns the wrapper; stored, cleared again by dp_wrapper_delete()
 * @param cb          engine block processor, called with `frag` (or the resampled count) samples per block
 * @param frag        samples per engine block, 1 .. 0xc0
 * @param rate        the modem's sample rate, Hz
 * @param engineRate  the engine's sample rate, Hz
 * @return the wrapper, or NULL if a parameter is 0, `frag` is above 0xc0, the allocation fails, or the rate pair has no converter
 *         (supported: 8000 <-> 9600, 8000 <-> 48000, 9600 <-> 48000)
 * @warning The wrapper's rings and staging buffers are fixed (2 x 384 and 2 x 192 shorts).  Only `frag` is checked, not the block length at the modem
 *          rate: blk = frag * rate / engineRate must be 1 .. 192 and the resampled engine block must be <= 192.  A larger blk overruns the rings; a
 *          blk of 0 (a tiny `frag` with `rate` much lower than `engineRate`) makes dp_wrapper_run() divide by zero.  The glue callers pass values
 *          that satisfy this (for example 160, 9600, 8000 gives blk 192).
 */
DpWrapper* dp_wrapper_create(struct dp* owner, DpProcessFn cb, int frag, int rate, int engineRate)
{
  DpWrapper* w;
  int blk;
  unsigned idx;
  RcFixed* rc;

  if (frag == 0 || rate == 0 || engineRate == 0 || frag > 0xc0)
    return 0;
  w = (DpWrapper*)sysdep_malloc(sizeof(DpWrapper));
  if (!w)
    return 0;
  sysdep_memset(w, 0, sizeof(DpWrapper));
  w->owner = owner;
  w->cb = cb;
  blk = (int)((unsigned)frag * (unsigned)rate) / engineRate;
  w->blk = (unsigned)blk;
  w->out.count = (unsigned)blk;
  w->out.half = (unsigned)blk;
  if (rate == engineRate)
    return w;

  /* different rates: one converter per direction; any failure releases what was built */
  idx = rc_index(rate, engineRate);
  rc = 0;
  if (idx != 0)
    rc = RcFixed_Create(idx);
  w->rcIn = rc;
  if (rc)
  {
    idx = rc_index(engineRate, rate);
    rc = 0;
    if (idx != 0)
      rc = RcFixed_Create(idx);
    w->rcOut = rc;
    if (rc)
      return w;
    rc = w->rcIn;
  }
  w->owner = 0;
  if (rc)
    RcFixed_Delete(rc);
  if (w->rcOut)
    RcFixed_Delete(w->rcOut);
  sysdep_free(w);
  return 0;
}

/** @brief Delete a wrapper from dp_wrapper_create(): clear the owner, delete the converters, free the block. */
void dp_wrapper_delete(DpWrapper* w)
{
  w->owner = 0;
  if (w->rcIn)
    RcFixed_Delete(w->rcIn);
  if (w->rcOut)
    RcFixed_Delete(w->rcOut);
  sysdep_free(w);
}

/**
 * @brief Process `cnt` samples through the wrapper: the `process` operation of the data pumps that use a DpWrapper.
 * @param dp   the data pump; `dp->dp_data` is the DpWrapper
 * @param in_  `cnt` input samples (16-bit) at the modem rate
 * @param out_ receives `cnt` output samples at the modem rate
 * @param cnt  number of samples; 0 or negative does nothing
 * @return the last non-zero result of the engine callback during this call, or DPSTAT_OK (0) if none was non-zero
 *
 * The input is copied into the input ring in pieces that never cross a ring or half boundary.  Each time a full block of `blk` samples is
 * available, the engine callback runs on it: directly on the ring half when the rates are equal, otherwise on the block resampled by rcIn into
 * `inbuf` with the output going to `outbuf` and back through rcOut into the output ring half.  The output ring is read at the same rate as the input
 * is written, so the output lags the input by one block (the initial silence).
 */
int dp_wrapper_run(struct dp* dp, void* in_, void* out_, int cnt)
{
  DpWrapper* w = (DpWrapper*)dp->dp_data;
  DpRing* A = &w->in;
  DpRing* B = &w->out;
  const short* in = (const short*)in_;
  short* out = (short*)out_;
  int ret = DPSTAT_OK;

  while (cnt > 0)
  {
    /* chunk length: what is left, but not past the end of either ring */
    unsigned blk = w->blk;
    unsigned n = (blk <= (unsigned)cnt) ? blk : (unsigned)cnt;
    unsigned room = 2 * blk - A->wr;
    if (n > room)
      n = room;
    room = 2 * blk - B->rd;
    if (n > room)
      n = room;

    /* store the input chunk */
    sysdep_memcpy(&A->data[A->wr], in, 2 * (size_t)n);
    in += n;
    A->count += n;
    A->wr = (A->wr + n) % (2 * w->blk);

    /* a full block is available: run the engine on it, through the converters when the rates differ */
    if (A->count >= w->blk)
    {
      unsigned cb_n; /* the engine block length: blk, or what rcIn produced */
      int r;
      short* eng_out;
      const short* eng_in;

      if (w->rcIn)
      {
        unsigned cap = 0xc0;
        RcFixed_Resample(w->rcIn, &A->data[A->half], w->blk, w->inbuf, &cap);
        cb_n = cap;
        eng_in = w->inbuf;
        eng_out = w->outbuf;
      }
      else
      {
        cb_n = w->blk;
        eng_in = &A->data[A->half];
        eng_out = &B->data[B->half];
      }
      r = w->cb(dp, (void*)eng_in, eng_out, (int)cb_n);
      if (r)
        ret = r;
      if (w->rcOut)
      {
        unsigned cap = w->blk;
        RcFixed_Resample(w->rcOut, w->outbuf, cb_n, &B->data[B->half], &cap);
      }
      A->count -= w->blk;
      A->half = (A->half == 0) ? w->blk : 0;
      B->count += w->blk;
      B->half = (B->half == 0) ? w->blk : 0;
    }

    /* return the output chunk */
    sysdep_memcpy(out, &B->data[B->rd], 2 * (size_t)n);
    B->count -= n;
    out += n;
    B->rd = (B->rd + n) % (2 * w->blk);
    cnt -= (int)n;
  }
  return ret;
}
