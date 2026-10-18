/**
 * @file v34_access.h
 * @brief Byte-offset and typed-member accessor macros for the V.34 module object (V34Obj), shared by the V.34 and V.PCM files.
 *
 * The V.34 object (v34_object_view.h) is a large flat struct.  Most code reaches it with plain `obj->member`; where a member is read through another
 * type than it is declared with (sign views, pointer views, a byte or half inside a word), at an offset that falls in an undeclared gap, or in a walk over
 * an array, it goes through the macros below.  All of them are unaligned- and alias-safe (`may_alias` typedefs).
 *
 *   OBJ_BYTES(o, off)       the byte at offset `off` of object `o`;  U8 S16 U16 S32 U32 PTR: an unaligned, alias-safe access to it
 *                           (PTR: a 32-bit-only slot, wrong on a 64-bit host)
 *   V34OFF(off) / V34OFFI   the host offset of the member that starts at 32-bit-layout offset `off` (V34M_<off> of v34_object_view.h) / of member
 *                           `mem` (+ d): OBJ_BYTES(obj, V34OFF(0xa48) + i) reaches byte i of the array that starts at 0xa48
 *   V34SUBP(o, mem)         the address of an embedded sub-object of the typed object
 *   S16T U16T S32T U32T U8T PTRT (o, off)
 *                           typed access to member V34M_<off> of V34Obj (the compiler computes the offset); <off> is the 32-bit-layout offset (0x3594)
 *                           or, for a member that has a descriptive name, that name without its trailing underscore (rxState): S16T(obj, rxState)
 *                           is S16T(obj, 0x3594).  The keys are generated into v34_object_view.h, and into v34_subobjects.h for the sub-object
 *                           families S16T_rx(b, rxFlags) ...
 *   U8TI U16TI (o, off, mem, d)
 *                           a byte / short INSIDE member `mem` (offsetof(member) + delta)
 *
 * Users.  v34_init.c, v34_setup.c, v34_handshake_helpers.c, v34_tx_symbol.c, v34_tx_level.c, v34_modulator.c, v34_receiver.c, v34_echo_rx_timing.c,
 * v34_agc_demod.c, v34_probe_select.c, v34_detectors.c, v34_frame_decoder.c and, in src/vpcm, v34_api.c, v34_modem_init.c and v34_handshake_init.c use plain
 * `obj->member` (a cast or AS_U16 for a sign view, a typed sub-object pointer such as `&obj->rx` for a callee) and keep only the raw accessors for the walks
 * that no member describes: untyped records (the DFT bins, the probing records), the 16-bit halves of a 32-bit member
 * (`shellIdx_`), 32-bit accesses over two 16-bit members and the walks over adjacent single words.  v34_unused.c (the dead functions) still uses the
 * typed accessors throughout.
 *
 * Needs v34_object_view.h (V34Obj, the V34M_<off> names) in the including file before the first expansion; this header does not include it, so that a
 * file that also includes the other view headers keeps its own include order.  Included bare from src/v34/, as "v34/v34_access.h" from src/vpcm/.
 *
 * Not here (per-file variants, see v34_access_sub.h and the files):
 *   - macros that exist in one file only (I32, I32T, S8, S8T, U16TI_ctx, ... of v34_unused.c).
 */
#include <stddef.h>
#include <stdint.h>

/* The typedefs are guarded, the macros are NOT: a test harness #undefs the accessors and includes a library source a second time (a second copy of the
 * code under test), which must define them again.  Every definition below is textually the same each time, so repeating
 * them is no redefinition. */
#ifndef V34_ACCESS_TYPES
#define V34_ACCESS_TYPES
typedef int8_t s8_t __attribute__((may_alias));
typedef uint8_t u8_t __attribute__((may_alias));
typedef int16_t s16_t __attribute__((may_alias));
typedef uint16_t u16_t __attribute__((may_alias));
typedef int32_t s32_t __attribute__((may_alias));
typedef uint32_t u32_t __attribute__((may_alias));
typedef void* ptr_t __attribute__((may_alias));
#endif /* V34_ACCESS_TYPES */

#define OBJ_BYTES(o, off) ((unsigned char*)(o) + (off))
#define AS_U16(lv) (*(uint16_t*)&(lv)) /* a signed 16-bit member viewed as unsigned (compatible types: no aliasing issue) */
#define U8(o, off) (*(u8_t*)OBJ_BYTES(o, off))
#define S16(o, off) (*(s16_t*)OBJ_BYTES(o, off))
#define U16(o, off) (*(u16_t*)OBJ_BYTES(o, off))
#define S32(o, off) (*(s32_t*)OBJ_BYTES(o, off))
#define U32(o, off) (*(u32_t*)OBJ_BYTES(o, off))
#define PTR(o, off) (*(ptr_t*)OBJ_BYTES(o, off)) /* 32-bit-only slot: 4 bytes of a pointer on a 32-bit host, half of one on a 64-bit host */

/* host offsets of members of the typed object */
#define V34OFF(off) ((int)offsetof(V34Obj, V34M_##off))
#define V34OFFI(mem, d) ((int)offsetof(V34Obj, mem) + (d))
/* the address of an embedded sub-object of the typed object */
#define V34SUBP(o, mem) ((unsigned char*)&((V34Obj*)(o))->mem)

/* typed-member accessors: the compiler computes the offset of member V34M_<off> of V34Obj (src/v34/v34_object_view.h) */
#define S16T(o, off) (*(s16_t*)&((V34Obj*)(o))->V34M_##off)
#define S32T(o, off) (*(s32_t*)&((V34Obj*)(o))->V34M_##off)
#define U16T(o, off) (*(u16_t*)&((V34Obj*)(o))->V34M_##off)
#define U32T(o, off) (*(u32_t*)&((V34Obj*)(o))->V34M_##off)
#define U8T(o, off) (*(u8_t*)&((V34Obj*)(o))->V34M_##off)
#define PTRT(o, off) (*(ptr_t*)&((V34Obj*)(o))->V34M_##off)
/* a byte / short INSIDE a member (offsetof(member) + delta) */
#define U8TI(o, off, mem, d) (*(u8_t*)((unsigned char*)(o) + offsetof(V34Obj, mem) + (d)))
#define U16TI(o, off, mem, d) (*(u16_t*)((unsigned char*)(o) + offsetof(V34Obj, mem) + (d)))
