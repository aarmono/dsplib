/**
 * @file v34_object.h
 * @brief The C++ view of the V.34 module object, `tagV34Object` (0xac4c bytes in the 32-bit layout), and the V.34 entry points implemented in C++.
 *
 * The object is shared by the V.34 code of the vpcm and v34/handshake modules.  It has one definition, the generated C struct in v34_object_view.h
 * (tools/tier_rewrite.py header --cxx layouts/v34/v34obj_cxx.txt --tag tagV34Object; run tools/gen_views.sh).  The plain-C files (src/v34/v34_*.c) include it as
 * V34Obj, this header includes it as tagV34Object.  It has real pointer members, so the compiler computes the 64-bit layout; the 32-bit offsets are
 * asserted in the generated header and, by member name, in v34_object_layout.inc (tools/layout_gen.py from layouts/v34/v01.layout); the x86-64 sizes, sub-object bases and pointer-member offsets are asserted in v34_object_layout64.inc (tools/layout64.py).  What the C++ files see on top
 * of the C files' view is listed in layouts/v34/v34obj_cxx.txt: typed pointers (ctx_ = the VPcmFloModem of vpcm_flo_modem.h, params_ = the dp parameter block, k56_ =
 * the K56Flex handle), signed words, and the sub-object structs (rx, mod, tx, echo0, echo1, det, rate, probe, the bit-queue records info0a ... mp and retrain), whose members the
 * C files and the C++ files both name by path.
 *
 * Also here: the enum VpcmSessionMode (the values of `mode_`), and the declarations of V34SetINFO1aBits and VPcmV34InitiateRetrain
 * (src/vpcm/vpcm_info1a_retrain.cpp) and of the phase 3 / 4 transmit state machines (src/vpcm/vpcm_phase34_reneg.cpp), all `extern "C"`.  The enums of the
 * object's state words are in v34_hs_init_mode.h, v34_role.h and v34_run_mode.h, which this header includes.
 */
#ifndef DSP_CLASSES_V01_H
#define DSP_CLASSES_V01_H

#include "core/dsp_cxx.h"
#include "vpcm/pcm_leaf_classes.h"           /* K56FlexFloModem */
#include "vpcm/vpcm_flo_modem.h"             /* VPcmFloModem and everything it embeds (V90Modem, V90Demodulator, V90Parameters, V92EchoCanceller ...) */
#include "vpcm/v90_constellation_designer.h" /* V90ConstellationDesigner */

/* the dp parameter block (the V.34 code reads a subset of its words): `V01_ModemParams`, defined once in core/dp_params.h (the plain-C files include the same
 * struct as V34ModParObj); the meaning of each member is documented there. */
#include "core/dp_params.h"

/* the K56Flex handle (K56FLEX_Create allocates it with sysdep_malloc(0x14) and does not initialise it): `struct K56FlexHandle`, defined once in the generated
 * C view v34/v34_k56flex_view.h (the plain-C files include the same struct as K56HandleObj).  Members: word0_ (+0x00) is returned by
 * VPcmV34GetCurrentRxBitRate in mode 3, flag8_ (+0x08), word_c_ (+0x0c); unk04_ / unk09_ / unk10_ are never accessed. */
#include "v34/v34_k56flex_view.h"

/* tagV34Object: the one definition of the V.34 module object is the generated C view v34_object_view.h (the plain-C files include the same struct as V34Obj).
 * The members named by the C++ files (typed pointers, signed words, the C++-only names of regions inside a sub-object) are listed in layouts/v34/v34obj_cxx.txt.  The sub-objects rx (with blkb), mod (with pf), tx, echo0 / echo1, det, rate, probe, info0a / info0d / info / info1c / q2 / mp (the bit-queue records) and retrain are named members: their
 * struct (v34_subobjects.h) is the only definition of their members (`obj->rx.rxFlags_`, `obj->echo1.echoBuf_`).  The view has real pointer members, so the compiler computes both layouts, and the 32-bit
 * offsets are asserted in the header and in v34_object_layout.inc. */
#include "v34_object_view.h"
#include "v34_hs_init_mode.h" /* V34HsInitMode: the mode argument of v34handshakinit */
#include "v34_role.h"         /* V34Role: the values of modeChar_ */
#include "v34_run_mode.h"     /* V34RunMode: the values of runMode_ */

/* tagV34Object::mode_, the mode of the session driver VPcmV34Progress.  The names come from the debug strings of VPcmV34Progress / VPcmV34GetDiagnostics:
 * "diagnostics of V.90 / V.92", "Moving to Phase3 Modem V90 / V92 / K56Flex", "Line verification period completed", "waiting for user response",
 * "Modem On Hold", "Reconnect request", "End Transmission".  The name of 6 is the least certain (named after the ANSam
 * wait of its handler, "ANSam not detected on out-going"); no vpcm source stores it, the V.34 handshake may.
 * The guards `mode > 10` and `(mode - 1) <= 1` stay literal in the code. */
enum VpcmSessionMode
{
  VPCM_MODE_V34 = 0,                  ///< plain V.34 session
  VPCM_MODE_V90 = 1,                  ///< V.90
  VPCM_MODE_V92 = 2,                  ///< V.92
  VPCM_MODE_K56FLEX = 3,              ///< K56flex
  VPCM_MODE_QC_LINE_VERIFICATION = 4, ///< quick-connect line verification period
  VPCM_MODE_WAIT_USER_RESPONSE = 5,   ///< waiting for the user's response
  VPCM_MODE_ANSAM_OUTGOING = 6,       ///< waiting for ANSam on an outgoing call
  VPCM_MODE_ON_HOLD = 7,              ///< modem on hold
  VPCM_MODE_RECONNECT_WAIT = 8,       ///< waiting for a reconnect request
  VPCM_MODE_RECONNECT = 9,            ///< reconnect
  VPCM_MODE_END = 10                  ///< end of transmission
};

typedef tagV34Object V01_tagV34Object;
typedef K56FlexHandle V01_K56Handle;

extern "C"
{
  /** @brief Store the INFO1a / INFO1d bits.  @param info  the INFO1a / INFO1d record (16-bit words 0 .. 9 are used)  @return always 0 */
  int V34SetINFO1aBits(tagV34Object* obj, uint16_t* info);
  /** @brief Request a retrain.  @param mode  the retrain mode, one byte */
  void VPcmV34InitiateRetrain(tagV34Object* obj, unsigned char mode);
  /* src/vpcm/vpcm_phase34_reneg.cpp: the phase 3 / 4 transmit state machines of the V.90 / K56Flex / rate-renegotiation modes; all return 0 */
  int v90Phase34(tagV34Object* obj);
  int k56FlexPhase34(tagV34Object* obj);
  int v90RateReneg(tagV34Object* obj);
  int v90RateRenegSilence(tagV34Object* obj);
}

#pragma GCC diagnostic ignored "-Winvalid-offsetof"
#include "v34_object_layout.inc"

#endif /* DSP_CLASSES_V01_H */
