/**
 * @file fax_v17_viterbi.c
 * @brief FAXVTB_decoder(): typed front end of the V.32 Viterbi decoder for the V.17 trellis decisions.
 *
 * The decoder itself is VTB_decoder() of src/v32/v32_datapath.c, which works on the V.32 engine object: it takes the address of its state sub-object and, on
 * a 64-bit build, finds the history ring pointer in the engine's tail slot (src/v32/v32.h), so the sub-object must sit at V32_VTB_OFF inside a full engine
 * allocation.  The V.17 receiver keeps its decoder state as a typed FaxVtb inside the decision-history object (FaxDec17).  On a 32-bit build FaxVtb has
 * exactly the V.32 sub-object layout and is passed directly; on a 64-bit build this file builds a scratch image of the V.32 sub-object, runs VTB_decoder() on
 * it (nothing is re-implemented) and copies the changed state back.
 */
#include <stdint.h>
#include <string.h>
#include "fax_v17_decision.h"
#include "v32/v32.h"

#if UINTPTR_MAX == 0xffffffffu
/**
 * @brief Run one Viterbi decoder step (32-bit build: direct call).
 * @param v    decoder state, same layout as the V.32 decoder object
 * @param i    equalized in-phase coordinate
 * @param q    equalized quadrature coordinate
 * @param out  receives the decoded bits
 */
void FAXVTB_decoder(FaxVtb* v, short i, short q, short* out) { VTB_decoder((char*)(v), i, q, out); }
#else
#define SUB(off, T) (*(T*)((char*)sub + (off)))
/**
 * @brief Run one Viterbi decoder step (64-bit build: through a scratch image of the V.32 sub-object).
 * @param v    decoder state; metrics, write index and previous quadrant are updated, the rest is read only
 * @param i    equalized in-phase coordinate
 * @param q    equalized quadrature coordinate
 * @param out  receives the decoded bits
 * @note The table handles and parameters are copied into the image on every call; only `metric`, `wr` and `prevQuad` are copied back.
 */
void FAXVTB_decoder(FaxVtb* v, short i, short q, short* out)
{
  /* a scratch V.32 engine image: only the sub-object bytes and its tail slot are ever read */
  char eng[V32_ENG_ALLOC] __attribute__((aligned(8)));
  char* sub = eng + V32_VTB_OFF;
  int k;

  /* load the typed state into the image (the history pointer goes into the engine tail slot) */
  V32X_PS(char*, sub, V32_VTB_OFF, 0x00, (char*)v->hist);
  for (k = 0; k < 8; k++)
    SUB(0x04 + 2 * k, short) = v->metric[k];
  SUB(0x14, short) = v->wr;
  SUB(0x18, int) = v->imap;
  SUB(0x1c, int) = v->qmap;
  SUB(0x20, int) = v->bound;
  SUB(0x24, short) = v->sel;
  SUB(0x28, int) = v->region;
  SUB(0x2c, short) = v->size;
  SUB(0x2e, short) = v->depth;
  SUB(0x30, short) = v->prevQuad;
  SUB(0x32, unsigned short) = v->lowMask;
  SUB(0x34, short) = v->shift;

  VTB_decoder((char*)sub, i, q, out);

  /* copy back the state the decoder changes */
  for (k = 0; k < 8; k++)
    v->metric[k] = SUB(0x04 + 2 * k, short);
  v->wr = SUB(0x14, short);
  v->prevQuad = SUB(0x30, short);
}
#endif
