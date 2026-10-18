/**
 * @file fax_v17_decision.h
 * @brief The V.17 receiver's symbol-decision callbacks and the two objects they work on (the Viterbi decoder state FaxVtb and the decision-history object
 *   FaxDec17).
 *
 * The decision callbacks are FaxFse callbacks (FaxFseCb of fax_rx_common.h: `cfg.cb`, called by FAXFSE_receive() once per symbol with the FSE block, an angle
 * slot and a magnitude slot).  They read the equalized symbol (`fse->outI[fse->idx]` / `fse->outQ[fse->idx]`) and keep their state in the decision-history
 * object that the FSE configuration points at (`fse->cfg.hist`, a FaxDec17 here).  FaxFse is the same block the V.27ter and V.29 receivers use, so the V.17
 * receiver embeds the FaxFse of fax_rx_common.h and only supplies its own callbacks (fax_v17_decision.c) and history object.
 *
 * FaxDec17 is owned by the V.17 receiver (fax_v17_rx.h embeds it in the receive bundle, and V17RX_create() initialises it); it sits first in the bundle's
 * decision data and must keep its members in this order.  The Viterbi decoder is the V.32 one (VTB_decoder() of src/v32/v32_datapath.c); FAXVTB_decoder()
 * (fax_v17_viterbi.c) runs it on the typed FaxVtb.
 *
 * On a 32-bit build the members sit at the byte offsets that the `_Static_assert`s pin and that the V.32 decoder reads raw; on a 64-bit build the members
 * after a pointer move, so nothing may address these objects by a raw byte offset.
 */
#ifndef DSPLIBS_PORT_FAX_DEC17_H
#define DSPLIBS_PORT_FAX_DEC17_H
#include <stddef.h>
#include <stdint.h>
#include "fax_rx_common.h"
#include "fax_common.h"

/**
 * @brief Viterbi decoder state of the V.17 trellis (the same layout as the V.32 decoder object VTBv32_init() builds).
 *
 * The members `imap`, `qmap`, `bound` and `region` are table handles (CFGP), not pointers.
 */
typedef struct FaxVtb
{
  short* hist;            ///< traceback ring: 16 slots of 8 states x 2 shorts (0x200 bytes), allocated by the receiver
  short metric[8];        ///< per-state path metrics
  short wr;               ///< ring write index, 0 .. 15
  short s16;              ///< unused
  int imap, qmap;         ///< handles of the I / Q coordinate maps of the trellis points
  int bound;              ///< handle of the candidate-point table
  short sel;              ///< rate family, 1 .. 4 (7200 / 9600 / 12000 / 14400)
  short s26;              ///< unused
  int region;             ///< handle of the region table
  short size;             ///< row size of the tables: 2 / 4 / 6 / 8
  short depth;            ///< traceback depth (16)
  short prevQuad;         ///< previous quadrant (state of the differential decoder)
  unsigned short lowMask; ///< mask of the uncoded bits
  short shift;            ///< differential decoder shift (equal to `sel`)
  short s36;              ///< unused
} FaxVtb;

/**
 * @brief Decision-history object shared by the V.17 decision callbacks and the receiver (reached through `fse->cfg.hist`).
 *
 * Holds the training-detector history (previous angle, the last three equalized I / Q pairs, the smoothed magnitudes), the training sequence register, the
 * counters that sequence the callbacks, and the Viterbi decoder state.  The object is at least 0x6c bytes on a 32-bit build.
 */
typedef struct FaxDec17
{
  FaxSgd* sgd;              ///< the receiver's scrambled-sequence generator (created and deleted by the receiver; not touched by the decisions)
  FaxVtb vtb;               ///< Viterbi decoder state used by the trellis decisions
  unsigned short prevAngle; ///< start-of-training detector: previous angle
  short abI0, abQ0;         ///< start-of-training detector: newest equalized (I, Q) pair
  short abI1, abQ1;         ///< start-of-training detector: the pair before
  short abI2, abQ2;         ///< start-of-training detector: the oldest pair
  short accA, accB;         ///< start-of-training detector: smoothed magnitudes of the large / small phase steps
  short s4e;                ///< unused
  int w50;                  ///< set to 1 when the start-of-training detector fires, cleared at the end of the equalizer training sequence
  short s54, s56, s58;      ///< unused (cleared at creation)
  short cnt;                ///< symbol counter of the current phase
  int reg;                  ///< shift register of the equalizer training sequence (two bits per symbol)
  int flag;                 ///< non-zero: short training, the trellis-coded decision follows the training directly (zero: the 4-point bridge comes first)
  short sel;                ///< index into FSEv17_decision[] (0 .. 3 = 16 / 32 / 64 / 128 point)
  short s66;                ///< unused (set to 3 at creation)
  unsigned short ctr;       ///< call counter of the trellis decisions (wraps 0x7fff to 0x4000); the carrier detectors test it against 999
  short s6a;                ///< unused
} FaxDec17;

#if UINTPTR_MAX == 0xffffffffu
_Static_assert(sizeof(FaxVtb) == 0x38 && offsetof(FaxVtb, metric) == 0x4 && offsetof(FaxVtb, wr) == 0x14 && offsetof(FaxVtb, imap) == 0x18 &&
                   offsetof(FaxVtb, qmap) == 0x1c && offsetof(FaxVtb, bound) == 0x20 && offsetof(FaxVtb, sel) == 0x24 && offsetof(FaxVtb, region) == 0x28 &&
                   offsetof(FaxVtb, size) == 0x2c && offsetof(FaxVtb, depth) == 0x2e && offsetof(FaxVtb, prevQuad) == 0x30 &&
                   offsetof(FaxVtb, lowMask) == 0x32 && offsetof(FaxVtb, shift) == 0x34,
               "FaxVtb 32-bit layout");
_Static_assert(sizeof(FaxDec17) == 0x6c && offsetof(FaxDec17, vtb) == 0x4 && offsetof(FaxDec17, prevAngle) == 0x3c && offsetof(FaxDec17, abI0) == 0x3e &&
                   offsetof(FaxDec17, abQ2) == 0x48 && offsetof(FaxDec17, accA) == 0x4a && offsetof(FaxDec17, accB) == 0x4c &&
                   offsetof(FaxDec17, w50) == 0x50 && offsetof(FaxDec17, cnt) == 0x5a && offsetof(FaxDec17, reg) == 0x5c && offsetof(FaxDec17, flag) == 0x60 &&
                   offsetof(FaxDec17, sel) == 0x64 && offsetof(FaxDec17, ctr) == 0x68,
               "FaxDec17 32-bit layout");
#endif

/**
 * @brief Run one Viterbi decoder step on the V.17 trellis state.
 * @param v    decoder state (FaxDec17::vtb)
 * @param i    equalized in-phase coordinate, Q15
 * @param q    equalized quadrature coordinate, Q15
 * @param out  receives the decoded bits (zero extended into a short)
 * @note On a 64-bit build the call goes through a scratch copy of the state, see fax_v17_viterbi.c.
 */
void FAXVTB_decoder(FaxVtb* v, short i, short q, short* out);

/**
 * @name Decision callbacks (FaxFseCb) Each reads the equalized symbol of `f`, stores the ideal point's angle in `*angle` and magnitude in `*mag`, and returns
 *   the decoded code in the low 16 bits (zero extended). @{
 */
int FAX_FSE_decision_16pt(FaxFse* f, short* angle, short* mag);  ///< 7200 bit/s, 16 points, trellis decoded
int FAX_FSE_decision_32pt(FaxFse* f, short* angle, short* mag);  ///< 9600 bit/s, 32 points, trellis decoded
int FAX_FSE_decision_64pt(FaxFse* f, short* angle, short* mag);  ///< 12000 bit/s, 64 points, trellis decoded
int FAX_FSE_decision_128pt(FaxFse* f, short* angle, short* mag); ///< 14400 bit/s, 128 points, trellis decoded
int FAX_FSE_decision_AB(FaxFse* f, short* angle, short* mag);    ///< start-of-training detector (4800 bit/s segment); installs FSE_decision_eqtrn when it fires
int FSE_Bridge_det(FaxFse* f, short* angle, short* mag);         ///< bridge segment: nearest of 4 points; then installs FSEv17_decision[hist->sel]
int FSE_decision_eqtrn(FaxFse* f, short* angle,
                       short* mag); ///< equalizer training sequence; then FSEv17_decision[hist->sel] (short training) or FSE_Bridge_det
/** @} */
#endif
