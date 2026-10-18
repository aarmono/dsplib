/**
 * @file v34_access_sub.h
 * @brief Typed accessors of the sub-object structs embedded in the V.34 object and of the V.90 / V.PCM parameter objects, `&member` spelling.
 *
 * Companion of v34_access.h (include it after that one).  Each macro reads member <PREFIX><off> of a typed struct, the compiler computing the offset:
 *   <T>T_rx   V34Rx (V34RX_<rel>, v34_subobjects.h)           PTRT_det S16T_det: V34ToneDet (V34DET_<rel>)
 *   <T>T_P    V34ModParObj (V34P_<off>, the dp parameter block) <T>T_cfg: V90ParObj (V34CFG_<off>)       <T>T_ctx: PcmFlo47Obj (VPFM_<off>)
 * (T = S16 U16 S32 U32 U8 PTR; only the combinations the files use exist).  Needs the matching view header (v34_subobjects.h / v34_object_view.h /
 * v34_v90params_view.h / v34_flomodem_view.h) before the first expansion; this header does not include them.
 *
 * Only v34_unused.c (the dead functions) includes it now; the live V.34 files use plain members.
 */
/* no include guard: see v34_access.h (the macros are repeated verbatim when a harness includes a library source a second time) */

#define PTRT_rx(o, off) (*(ptr_t*)&((V34Rx*)(o))->V34RX_##off)
#define S16T_rx(o, off) (*(s16_t*)&((V34Rx*)(o))->V34RX_##off)
#define U16T_rx(o, off) (*(u16_t*)&((V34Rx*)(o))->V34RX_##off)
#define U32T_rx(o, off) (*(u32_t*)&((V34Rx*)(o))->V34RX_##off)
#define PTRT_det(o, off) (*(ptr_t*)&((V34ToneDet*)(o))->V34DET_##off)
#define S16T_det(o, off) (*(s16_t*)&((V34ToneDet*)(o))->V34DET_##off)
#define S32T_cfg(o, off) (*(s32_t*)&((V90ParObj*)(o))->V34CFG_##off)
#define PTRT_ctx(o, off) (*(ptr_t*)&((PcmFlo47Obj*)(o))->VPFM_##off)
#define S32T_ctx(o, off) (*(s32_t*)&((PcmFlo47Obj*)(o))->VPFM_##off)
#define U8T_ctx(o, off) (*(u8_t*)&((PcmFlo47Obj*)(o))->VPFM_##off)
