/**
 * @file v34_engine_views.h
 * @brief C prefix views of the V.90 / V.92 engine objects that the V.34 wrapper functions of src/vpcm/v34_api.c reach through the VPcmFloModem context.
 *
 * The wrappers walk, from the context `ctx` (the V.34 object's `ctx_` pointer):
 *   - [ctx + 0x1758] = v90_.mod_ -> V90Modulator, .state_ / .b2s_ -> V90BitsToSymbol .mapper_ -> V90Mapper .w0_  (VPcmV34GetCurrentTxBitRate, modes 1 / 2)
 *   - [ctx + 0x6124] = v92_.mod_ -> V92Modulator, .state_ / .b2s_ -> V92BitsToSymbol .tx_ -> V92Transmitter .w0_  (the same, the V.92 branch)
 *   - [ctx + 0x175c] = v90_.dem_ -> V90Demodulator .ce_ -> V90ConnectionEvaluator .externalCommand_  (InitiateRateRenegotiation / InitiateHangUp)
 * (offsets of the 32-bit layout).
 *
 * Each view is a prefix of the real C++ class (src/vpcm/pcm_modulators.h and the classes behind V90Mapper, V92Transmitter, V90Demodulator and
 * V90ConnectionEvaluator), named after it, with the class's own member names: where the prefix is simple it repeats the class's member types (natural layout
 * at both word sizes); V90Demodulator's prefix holds embedded sub-objects, so its leading bytes are one gap whose two sizes (32-bit / 64-bit layout) are
 * the class's offsetof(ce_).  The C++ side checks every member offset of these views against the real classes at compile time, at whichever word size it
 * is built (src/vpcm/vpcm_flo_modem.cpp, end of file), so a class change that moves a member breaks the build instead of the V.34 chain.
 */
#ifndef V34ENG_H
#define V34ENG_H

#include <stddef.h>
#include <stdint.h>

/* a byte gap whose size differs by word size: both numbers are offsets of the real C++ class (checked in src/vpcm/vpcm_flo_modem.cpp) */
#define V34ENG_GAP(w32, x64) (sizeof(void*) == 8 ? (x64) : (w32))

/** V90Modulator (pcm_modulators.h): members up to b2s_; the offsets in the comments are those of the 32-bit layout. */
typedef struct
{
  void *info_, *jd_, *v92jd_, *dil_, *mp1_, *mp2_, *addCp_, *mp_, *cp_, *params_; ///< 0x00 .. 0x24
  uint32_t session_;                                                              ///< 0x28
  int32_t state_;                                                                 ///< 0x2c 3 = running
  uint32_t cnt_;                                                                  ///< 0x30
  uint32_t event_;                                                                ///< 0x34
  void *p3_, *p4_;                                                                ///< 0x38 0x3c
  void* b2s_;                                                                     ///< 0x40 -> V90BitsToSymbol
} V90ModulatorPfx;
/** V92Modulator: members up to b2s_ (32-bit layout offsets in the comments). */
typedef struct
{
  uint32_t blk_, half_, cur_;              ///< 0x00 .. 0x08
  uint8_t codeA_, codeB_, pad0e_[2];       ///< 0x0c .. 0x0f
  void *info_, *ja_, *dil_, *mp_, *cp_;    ///< 0x10 .. 0x20
  float skew_, ppm_;                       ///< 0x24 0x28
  int32_t state_;                          ///< 0x2c 3 = running
  uint32_t cnt_, event_, split_, splitAt_; ///< 0x30 .. 0x3c
  void *params_, *p3_, *p4_;               ///< 0x40 .. 0x48
  void* b2s_;                              ///< 0x4c -> V92BitsToSymbol
} V92ModulatorPfx;
/** V90BitsToSymbol / V92BitsToSymbol: the first member. */
typedef struct
{
  void* mapper_; ///< 0x00 -> V90Mapper
} V90BitsToSymbolPfx;
typedef struct
{
  void* tx_; ///< 0x00 -> V92Transmitter
} V92BitsToSymbolPfx;
/** V90Mapper / V92Transmitter: the current transmit rate word w0_ (scaled by 8000 / 6 or 8000 / 12 in VPcmV34GetCurrentTxBitRate). */
typedef struct
{
  void* params_; ///< 0x00
  uint32_t w0_;  ///< 0x04
} V90MapperPfx;
typedef struct
{
  uint8_t unk0_[4]; ///< 0x00
  uint32_t w0_;     ///< 0x04
} V92TransmitterPfx;
/** V90Demodulator: the connection-evaluator pointer ce_. */
typedef struct
{
  uint8_t unk00_[V34ENG_GAP(0x20c, 0x2a8)]; ///< hwCodec_ .. cdes_ (pointers and embedded sub-objects)
  void* ce_;                                ///< 32-bit layout 0x20c, 64-bit layout 0x2a8 -> V90ConnectionEvaluator
} V90DemodulatorPfx;
/** V90ConnectionEvaluator: externalCommand_, the rate-renegotiation / hang-up command word. */
typedef struct
{
  void* params_;            ///< 0x00
  int32_t w04_[34];         ///< 0x04 .. 0x88: nofRetrainsSoFar_ .. trn1dEvalActive_ (all 32-bit scalars)
  int32_t externalCommand_; ///< 0x8c
} V90ConnectionEvaluatorPfx;

#if !defined(__cplusplus) && UINTPTR_MAX == 0xffffffffu
_Static_assert(offsetof(V90ModulatorPfx, state_) == 0x2c && offsetof(V90ModulatorPfx, b2s_) == 0x40, "V90Modulator 32-bit layout");
_Static_assert(offsetof(V92ModulatorPfx, state_) == 0x2c && offsetof(V92ModulatorPfx, b2s_) == 0x4c, "V92Modulator 32-bit layout");
_Static_assert(offsetof(V90MapperPfx, w0_) == 0x4 && offsetof(V92TransmitterPfx, w0_) == 0x4 && offsetof(V90DemodulatorPfx, ce_) == 0x20c &&
                   offsetof(V90ConnectionEvaluatorPfx, externalCommand_) == 0x8c,
               "V90 engine 32-bit layout");
#endif

#endif /* V34ENG_H */
