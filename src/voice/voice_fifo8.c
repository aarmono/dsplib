/**
 * @file voice_fifo8.c
 * @brief The 8-bit ring buffer of the voice transmit path: FIFO8_create / FIFO8_write / FIFO8_read / FIFO8_delete.
 *
 * The voice engine (voice_tx() in voice_engine.c) writes the tty bytes of a block into the FIFO and reads a fixed number of bytes out per block.
 * Types: FifoObj / FifoCfg in voice_fdsp.h.  All counters are 16 bits and wrap.
 * - FIFO8_create(f, cfg): `cfg` NULL selects the default record { 0, 300, 0 } (a 300-byte buffer, fill byte 0).  `f` NULL allocates the header and then
 *   the buffer; an existing `f` keeps its buffer (cfg->size must not exceed it).  The buffer is cleared.  NULL is returned when an allocation fails.
 * - FIFO8_write(f, src, n): stores min(n, free space) bytes at the write index (wrapping at `size`) and returns the number stored.
 * - FIFO8_read(f, dst, n): reads min(n, count) bytes from the read index (wrapping) and fills the remaining bytes of dst with the low byte of
 *   `f->fill` (the underrun byte); returns the number of real bytes read.
 * - FIFO8_delete(f): frees the buffer and the header; NULL is ignored.
 */
#include "voice_fdsp.h"

/* the default creation record: no tag word, 300 bytes, fill byte 0 */
static const FifoCfg fifo8_cfg = {0, 300, 0};

/**
 * @brief Create a FIFO, or re-initialise an existing one.
 * @param f    NULL to allocate the header and the buffer, or an existing FIFO whose buffer is reused
 * @param cfg  creation record, or NULL for the default (300 bytes, fill 0)
 * @return the FIFO, or NULL when an allocation failed (a header allocated before the failure is freed)
 * @warning With an existing `f`, cfg->size must not exceed the size of its buffer.  A size of 0x8000 or more is cleared completely here.
 */
FifoObj* FIFO8_create(FifoObj* f, const FifoCfg* cfg)
{
  uint16_t w0, size, fill;
  uint32_t i;

  if (cfg == NULL)
    cfg = &fifo8_cfg;
  w0 = cfg->w0;
  size = cfg->size;
  fill = cfg->fill;
  if (f == NULL)
  {
    f = (FifoObj*)sysdep_malloc((unsigned)sizeof(FifoObj));
    if (f == NULL)
      return NULL;
    f->buf = (uint8_t*)sysdep_malloc(size);
    if (f->buf == NULL)
    {
      sysdep_free(f);
      return NULL;
    }
  }
  f->w0 = w0;
  f->size = size;
  f->count = 0;
  f->rd = 0;
  f->fill = fill;
  f->wr = 0;
  for (i = 0; i < size; i++)
    f->buf[i] = 0;
  return f;
}

/**
 * @brief Append bytes to the FIFO.
 * @param f    the FIFO
 * @param src  bytes to store
 * @param n    number of bytes offered
 * @return the number of bytes stored, min(n, free space); the rest is dropped
 */
int FIFO8_write(FifoObj* f, const unsigned char* src, unsigned short n)
{
  uint16_t size = f->size;
  uint16_t count = f->count;
  uint16_t space = (uint16_t)(size - count);
  uint16_t w = n > space ? space : n;
  uint8_t* buf = f->buf;
  uint16_t wr = f->wr;
  uint16_t k;

  for (k = w; k != 0; k--)
  {
    uint16_t nx;
    buf[wr] = *src++;
    nx = (uint16_t)(wr + 1);
    wr = nx < size ? nx : 0;
  }
  f->wr = wr;
  f->count = (uint16_t)(count + w);
  return w;
}

/**
 * @brief Remove up to n bytes from the FIFO.
 * @param f    the FIFO
 * @param dst  receives exactly n bytes: the bytes held, then the underrun fill byte (low byte of f->fill) for the missing ones
 * @param n    number of bytes wanted
 * @return the number of real bytes read, min(n, count)
 */
int FIFO8_read(FifoObj* f, unsigned char* dst, unsigned short n)
{
  uint16_t size = f->size;
  uint16_t count = f->count;
  uint16_t m = count < n ? count : n;
  uint16_t rest = (uint16_t)(n - m);
  uint16_t rd = f->rd;
  const uint8_t* buf = f->buf;
  uint8_t fb = (uint8_t)f->fill;
  uint16_t k;

  for (k = m; k != 0; k--)
  {
    uint16_t nx;
    *dst++ = buf[rd];
    nx = (uint16_t)(rd + 1);
    rd = nx < size ? nx : 0;
  }
  for (k = rest; k != 0; k--)
    *dst++ = fb;
  f->rd = rd;
  f->count = (uint16_t)(count - m);
  return m;
}

/**
 * @brief Free a FIFO and its buffer.
 * @param f  the FIFO; NULL is ignored
 */
void FIFO8_delete(FifoObj* f)
{
  if (f == NULL)
    return;
  sysdep_free(f->buf);
  sysdep_free(f);
}
