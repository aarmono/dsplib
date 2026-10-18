/**
 * @file voice_fdsp.h
 * @brief Types and API of the voice support layer: the FDSP object and the 8-bit FIFO.
 *
 * Two small objects used only by the voice engine (voice.h, voice_engine.c), both held through opaque handles:
 * - FdspObj, the duplex stream block.  It owns three heap blocks: a history block (FdspHist: two 2000-sample rings and three words) and two channel
 *   blocks (FdspChan, receive side `ch14` with the delay sRxSamplesDelay, transmit side `ch18` with sTxSamplesDelay), each with a 240-word tail block.
 *   In the duplex mode FDSP_DP_Run() only converts between 16-bit and float samples (the echo-cancelling kernel is not present); the history and
 *   channel blocks are cleared by FDSP_Kernel_InitObj() and otherwise unused.
 * - FifoObj, the byte ring buffer the transmit path fills with the tty bytes (FIFO8_*): 16-bit counters that wrap, an underrun fill byte.
 *
 * Layouts.  The 16- and 32-bit members keep their 32-bit image offsets on every host (the `_Static_assert`s pin them); the pointers are real
 * pointers.  A pointer that sat inside the image of the structure lives in a tail slot after the image instead (`FdspChan::tail`, `FifoObj::buf`),
 * so sizeof(FdspObj) = 0x28, sizeof(FdspChan) = 0x16b0 and sizeof(FifoObj) = 0x20 on x86-64 (0x1c, 0x16ac and 0x18 at -m32).  Use the member names and
 * sizeof(), never a raw offset.
 *
 * API summary:
 * - FDSP_DP_Create(obj, rxDelay, txDelay): obj NULL allocates six blocks (object, history, two channels, two tails; NULL on any failure, everything
 *   freed), otherwise the object is re-initialised in place ("Reinitialization requested").  FDSP_DP_Delete(obj) ignores NULL.
 * - FDSP_DP_Run(): out1[i] = in1[i] / 32000, out2[i] = (short)trunc(in2[i] * 32000) for i < *n; sets obj->mode = 2; returns 1.
 * - STRM_VCE_GetFDSPEnvironmentalParams(): the two delays, always 0x33 (receive) and 0x171 (transmit).
 * - FIFO8_create(f, cfg): f NULL allocates the header and the buffer (cfg->size bytes); cfg NULL is the built-in default of 300 bytes.
 *   FIFO8_write() returns the bytes stored (the smaller of n and the free space); FIFO8_read() returns the bytes read and fills the rest of n
 *   with the low byte of `fill`.
 */
#ifndef DSPLIBS_PORT_FDSP_OBJ_H
#define DSPLIBS_PORT_FDSP_OBJ_H
#include <stddef.h>
#include <stdint.h>
#include "core/dsp_host.h"

/** The FDSP history block: two 2000-sample rings and three words, cleared by FDSP_Kernel_InitObj() and not used afterwards. */
typedef struct FdspHist
{
  int16_t x[2000]; ///< first history ring
  int16_t y[2000]; ///< second history ring
  uint8_t unk_1f40[0x7d0];
  int32_t z[3]; ///< state words
} FdspHist;

/**
 * One channel of the FDSP object (receive side `ch14`, transmit side `ch18`).  Offset 0x1680 holds a 32-bit pointer slot that is reserved and
 * unused; the pointer to the 240-word tail block (`tail`) lives after the 32-bit image.
 */
typedef struct FdspChan
{
  int32_t w[0x500]; ///< 0x500 words of channel state, cleared at init
  uint8_t unk_1400[0x280];
  uint32_t slot_1680; ///< reserved pointer slot (unused)
  uint32_t unk_1684;
  int16_t delay; ///< delay in samples (sRxSamplesDelay / sTxSamplesDelay)
  int16_t unk_168a;
  float gain;    ///< 0.032 on the receive channel (FDSP_GAIN_RX), 0.0 on the transmit channel
  int16_t s1690; ///< cleared at init
  int16_t unk_1692;
  int32_t v1694[4]; ///< cleared at init
  int32_t v16a4;    ///< cleared at init
  int32_t* tail;    ///< the 240-word block (allocated separately; cleared at init)
} FdspChan;

/** The FDSP object: mode word, parameters set by FDSP_Kernel_InitObj(), and the three blocks. */
typedef struct FdspObj
{
  int32_t mode;   ///< 2 = duplex pass-through (0 when created with a negative receive delay)
  int32_t w4;     ///< cleared at init
  int32_t w8;     ///< 0x50 at init
  int32_t wc;     ///< 0x28 at init
  FdspHist* hist; ///< history block
  FdspChan* ch14; ///< receive-side channel
  FdspChan* ch18; ///< transmit-side channel
} FdspObj;

/** FIFO creation record (FIFO8_create's second argument). */
typedef struct FifoCfg
{
  uint16_t w0;   ///< stored in FifoObj::w0
  uint16_t size; ///< buffer size in bytes
  uint16_t fill; ///< the low byte is the underrun fill of FIFO8_read()
} FifoCfg;

/** The 8-bit ring buffer.  All counters are 16 bits and wrap at `size`. */
typedef struct FifoObj
{
  uint16_t w0;   ///< copy of FifoCfg::w0
  uint16_t size; ///< buffer size in bytes
  uint16_t fill; ///< underrun fill (low byte)
  uint16_t unk_6;
  uint32_t slot_8; ///< reserved 32-bit slot (unused)
  uint16_t count;  ///< bytes held
  uint16_t rd;     ///< read index
  uint16_t wr;     ///< write index
  uint16_t unk_12;
  uint8_t* buf; ///< the buffer (allocated by FIFO8_create(), size bytes)
} FifoObj;

#ifdef __i386__
_Static_assert(sizeof(FdspObj) == 0x1c, "FdspObj");
_Static_assert(offsetof(FdspObj, hist) == 0x10 && offsetof(FdspObj, ch14) == 0x14 && offsetof(FdspObj, ch18) == 0x18, "FdspObj");
_Static_assert(sizeof(FdspChan) == 0x16ac, "FdspChan");
_Static_assert(offsetof(FifoObj, buf) == 0x14, "FifoObj");
#endif
_Static_assert(sizeof(FdspHist) == 0x271c, "FdspHist");
_Static_assert(offsetof(FdspHist, y) == 0xfa0 && offsetof(FdspHist, z) == 0x2710, "FdspHist");
_Static_assert(offsetof(FdspChan, slot_1680) == 0x1680 && offsetof(FdspChan, delay) == 0x1688 && offsetof(FdspChan, gain) == 0x168c &&
                   offsetof(FdspChan, s1690) == 0x1690,
               "FdspChan");
_Static_assert(offsetof(FdspChan, v1694) == 0x1694 && offsetof(FdspChan, v16a4) == 0x16a4 && offsetof(FdspChan, tail) >= 0x16a8, "FdspChan");
_Static_assert(offsetof(FdspObj, mode) == 0 && offsetof(FdspObj, wc) == 0xc, "FdspObj");
_Static_assert(sizeof(FifoCfg) == 6 && offsetof(FifoObj, size) == 2 && offsetof(FifoObj, fill) == 4 && offsetof(FifoObj, count) == 0xc, "FifoObj");
_Static_assert(offsetof(FifoObj, rd) == 0xe && offsetof(FifoObj, wr) == 0x10, "FifoObj");

/* the 32-bit image sizes of the blocks (sizeof differs from them on x86-64) */
#define FDSP_OBJ_BLOB_SIZE 0x1cu
#define FDSP_HIST_BLOB_SIZE 0x271cu
#define FDSP_CHAN_BLOB_SIZE 0x16a8u
#define FDSP_TAIL_BLOB_SIZE 0x3c0u
#define FIFO8_OBJ_BLOB_SIZE 0x14u

extern FdspObj* FDSP_DP_Create(FdspObj* obj, short rx_delay, short tx_delay);
extern void FDSP_DP_Delete(FdspObj* obj);
extern int FDSP_DP_Run(FdspObj* obj, const short* in1, float* out1, const float* in2, short* out2, void* unused, const unsigned short* n);
extern void FDSP_Kernel_InitObj(FdspObj* obj);
extern void FDSP_Kernel_SetInternalBeepInProgress(int on);
extern void STRM_VCE_GetFDSPEnvironmentalParams(short* rx_delay, short* tx_delay);
extern FifoObj* FIFO8_create(FifoObj* f, const FifoCfg* cfg);
extern void FIFO8_delete(FifoObj* f);
extern int FIFO8_write(FifoObj* f, const unsigned char* src, unsigned short n);
extern int FIFO8_read(FifoObj* f, unsigned char* dst, unsigned short n);
#endif
