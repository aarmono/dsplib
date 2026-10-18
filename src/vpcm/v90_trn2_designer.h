/**
 * @file v90_trn2_designer.h
 * @brief V90TRN2Designer: designs the TRN2 training constellation for the V.90 / V.92 demodulator.
 *
 * A small class of two borrowed pointers (the V.90 parameter block and the constellation-power object; no virtual functions, trivial constructor and
 * destructor; 8 bytes in the 32-bit layout, asserted by the generated v90_trn2_designer_layout.inc).  The work is V90TRN2Design(), implemented in
 * v90_trn2_designer.cpp; V90Demodulator calls it, and setNofUcodesInTrn2() beforehand.
 *
 * What V90TRN2Design() does: it fills the mapping block (V90MappingParams) with the constellation of the TRN2 training sequence.  The six per-phase counts
 * are params->nofUcodes; the spectral shaper constants come from the parameter block (the German-PBX set when the special condition is 2); the data rate
 * word is D = K - sr + 6 with K the integer log2 of the product of the counts.  From the power limit of the given TX index it computes the highest allowed
 * ucode (the G.711 code of sqrt(limit) * 1.627, minus 2, but at least params->maxUcode) and lowers the per-phase start ucode until its code fits.  Per phase it
 * then runs either the dMin design (flags[p] == 0: dMin = 0.85 * A[cur] / (N - 0.5), N ucodes spaced at least dMin apart, from params->unk360 up) or the
 * forced design (flags[p] != 0: up to 200 rounds of an N-point search with a shrinking minimum distance).  The second list holds the G.711 codes in the
 * target codec.  Finally it prints the resulting power, power index and the table.  It returns 1, or 0 (after filling in a dummy constellation) when a phase
 * finds no constellation.
 */
#ifndef V90_TRN2_DESIGNER_H
#define V90_TRN2_DESIGNER_H

#include "core/dsp_cxx.h"
#include "pcm_parameters.h"             /* V90Parameters */
#include "v92_cp.h"                     /* V90MappingParams */
#include "pcm_leaf_classes.h"           /* V90ConstellationPower, PcmType */
#include "v90_constellation_designer.h" /* V90SpecialSpectralConditions */

/** @brief TRN2 constellation designer; holds the parameter block and the power calculator it was created with (both borrowed, not owned). */
class V90TRN2Designer
{
public:
  /**
   * @brief Create the designer.
   * @param p   V.90 parameter block (nofUcodes, maxUcode, unk360, spectral shaper constants are read from it); must outlive the designer
   * @param pw  power calculator used for the power and power index of the designed constellation; must outlive the designer
   */
  V90TRN2Designer(V90Parameters* p, V90ConstellationPower* pw);
  /** @brief Destroy the designer (nothing is owned). */
  ~V90TRN2Designer();
  /** @brief If `n` is non-zero, use the TRN2 constellation size: params->nofUcodes = params->nofUcodesInTrn2. */
  void setNofUcodesInTrn2(short n);
  /**
   * @brief Design the TRN2 constellation into `mp`.
   * @param mp        mapping block to fill (counts, lists, spectral constants, data rate word)
   * @param A         per-phase table of ucode amplitudes, 6 x 128 entries
   * @param B         per-phase second amplitude table, 6 x 128 entries
   * @param valid     per-phase validity flags of the ucodes, 6 x 128 entries
   * @param flags     per-phase design selector: 0 = dMin design, non-zero = forced design (6 entries)
   * @param pcmA      codec of the tables A / B
   * @param pcm       target codec (non-zero = A-law)
   * @param unused    ignored
   * @param cur       starting ucode of each of the six phases (6 bytes; copied, not modified)
   * @param limit     upper limit for the spectral shaper identifier
   * @param powerIdx  TX power index selecting the power limit
   * @param special   special spectral condition (2 = German PBX shaper set)
   * @return 1 on success, 0 when a phase has no solution (the mapping block then holds a dummy constellation)
   * @warning The start-ucode lowering loop does not terminate when every code of a phase exceeds the maximum, and ucode indices 128 and above run into the
   *          next table row.
   */
  int V90TRN2Design(V90MappingParams* mp, short (*A)[128], short (*B)[128], unsigned char (*valid)[128], short* flags, PcmType pcmA, PcmType pcm, short unused,
                    unsigned char* cur, unsigned limit, unsigned char powerIdx, V90SpecialSpectralConditions special);
  /** @brief The integer K = trunc(log2(product of the six counts of `mp`)); no caller in the library. */
  unsigned maxK(V90MappingParams* mp);
  /** @brief Fill the first N entries of both lists of every phase with a descending dummy pattern (0x4e, 0x4d, ...); no caller in the library. */
  void setTrn2DummyConstel(V90MappingParams* mp);

public:
  V90Parameters* params_;        ///< borrowed V.90 parameter block
  V90ConstellationPower* power_; ///< borrowed power calculator
};

typedef V90TRN2Designer P26_V90TRN2Designer;

#include "v90_trn2_designer_layout.inc"

#endif /* V90_TRN2_DESIGNER_H */
