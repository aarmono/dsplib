/**
 * @file pcm_dil.h
 * @brief The DIL descriptor (tagV90DILdescriptor) and the free functions around it.
 *
 * Declares setDilDescriptor(), getSegmentPointer(), calculateDilLength() (two overloads), DILdescriptorPacker() (V.90 frame, one `short` per bit) and
 * V92DILdescriptorPacker() (V.92 frame, one byte per bit).  Definitions and the eight constant tables: pcm_dil.cpp.
 *
 * tagV90DILdescriptor is a plain byte structure embedded in VPcmFloModem at +4 (the owner's V.90 frame buffer starts at +0x21e).  It is filled by
 * setDilDescriptor() (V90Modem::reset) from the tables and consumed by calculateDilLength() (V90Phase3Demodulator::reset) and the two packers.  The
 * member names follow the debug strings of the packer (N / Lsp / Ltp) and the table names TO, H, N, REF, TP, SP, Ltp, Lsp.  `hr_` is H[8] followed by
 * REF[8], so calculateDilLength() reads REF[0] when a segment index reaches 8.  Layout asserts: pcm_dil_layout.inc (generated).
 */
#ifndef DSP_CLASSES_P13_H
#define DSP_CLASSES_P13_H

#include "core/dsp_cxx.h"
#include "pcm_leaf_classes.h" /* PcmType */

/** DIL option; the names are the ones setDilDescriptor() prints ("ADI", "ADI_QC"). */
enum DilType
{
  DIL_ADI = 0,
  DIL_ADI_QC = 1
};

/** DIL descriptor: the three counts and the SP, TP, H / REF and TO tables (0x213 bytes). */
struct tagV90DILdescriptor
{
  uint8_t N_;       ///< +0x000 number of TO entries (0x90 for both options)
  uint8_t Lsp_;     ///< +0x001 number of SP entries (0x78 for ADI, 0x3c for ADI_QC)
  uint8_t Ltp_;     ///< +0x002 number of TP entries (0x78 for ADI, 0x3c for ADI_QC)
  uint8_t sp_[128]; ///< +0x003 SP symbols (0 / 1)
  uint8_t tp_[128]; ///< +0x083 TP symbols (0 / 1)
  uint8_t hr_[16];  ///< +0x103 H[8] then REF[8]
  uint8_t to_[256]; ///< +0x113 TO values
};
typedef tagV90DILdescriptor P13_DIL;

extern "C"
{
  /* C linkage (unmangled symbol); called by VPcmFloModem with the V.90 frame buffer */
  void DILdescriptorPacker(tagV90DILdescriptor* d, short* out, short* len);
}
void setDilDescriptor(tagV90DILdescriptor* d, DilType t);
int getSegmentPointer(PcmType p, int v);
int calculateDilLength(DilType t, PcmType p);
int calculateDilLength(tagV90DILdescriptor* d, PcmType p);
void V92DILdescriptorPacker(tagV90DILdescriptor* d, unsigned char* out, int* len);

#include "pcm_dil_layout.inc"

#endif /* DSP_CLASSES_P13_H */
