/**
 * @file dsp_access.h
 * @brief Typed views of library objects: member access by name through a different type, address or element index (C and C++).
 *
 * Most code reads and writes object members as plain `p->member`.  The macros below cover the cases where that is not enough:
 * - a member that is declared with one type but is read both as `short` and as `unsigned short` (or as a byte inside a wider word),
 * - a struct laid over a byte or element address (the sub-objects of a flat object, the flat V.8 / V.32 state blocks),
 * - the byte distance between two members.
 *
 * Layouts.  Many library objects exist in two layouts: the 32-bit image layout, in which every pointer is a 4-byte word and the members sit at
 * fixed offsets (the layout that the `_Static_assert` blocks and `*_layout.inc` fragments next to the structs pin), and the native layout of the
 * build, in which pointer members are as wide as the host pointer.  Every macro here takes the offset from the struct definition (`offsetof`),
 * never from a literal, so it is right in both layouts.  The `p` and `a` arguments are evaluated once.
 *
 * Families (T = the type the bytes are read or written as, S = the struct type, E = the element type of an element-indexed base, `m` = member):
 * - MEMBER_DISTANCE, MEMBER_ADDR_VIA: byte distances and addresses relative to a byte-array member;
 * - MEMBER_AS, MEMBER_AS_AT, MEMBER_AS_ATB, MEMBER_ADDR: a view of member `m` of `*p` as a T (the struct type is deduced from `p`);
 * - MEMBER_AS_OF*, MEMBER_AS_ADDR*, MEMBER_ADDR_A, MEMBER_ADDR_OF: the same with the struct S named explicitly, for a base that is not a pointer
 *   to S (the V.32 objects);
 * - MEMBER_IDX*, MEMBER_AS_PIDX, MEMBER_ADDR_PIDX: element-indexed bases (`short *`, `int *`), where the index counts elements, not bytes;
 * - MEMBER_ELEM_FLAT (C++ only): element access of an array member.
 *
 * Every use of the cast-based families is a place where the declared type of the member and the accessed type differ; they type-pun through
 * `char *`.  The `_AT` / `_ATB` suffix selects the order of the two additions (`(base + member) + bytes` or `(base + bytes) + member`); the order is
 * kept because it changes the code the compiler generates at -O0.
 */
#ifndef DSP_ACCESS_H
#define DSP_ACCESS_H

#include <stddef.h>
#include <stdint.h>

/** Byte distance from member `from` to member `to` of struct T, as a size_t.  `to` must not lie before `from`; cast the result for a signed value. */
#define MEMBER_DISTANCE(T, from, to) (offsetof(T, to) - offsetof(T, from))

/**
 * Address of member `m`, reached from the byte-array member `base` of the same object: `p->base + MEMBER_DISTANCE(base, m)`, typed like `p->base`
 * (a byte pointer).  For code that addresses a later member through an earlier opaque byte array.  The address is formed in two steps
 * (object + base, then + distance) rather than as `(uint8_t *)p->m`, which changes the code generated at -O0.
 * @warning `base` must be a byte array (uint8_t[] / char[]): the distance is added in units of its element.  The distance comes from the struct
 *          layout, so it is right in both layouts even where width-dependent padding separates the two members.
 */
#define MEMBER_ADDR_VIA(p, base, m) ((p)->base + MEMBER_DISTANCE(__typeof__(*(p)), base, m))

/**
 * The bytes at member `m` of `*p`, read or written as a T instead of the member's own type: `*(T *)((char *)p + offsetof(m))`.  For objects whose
 * members are declared with one fixed type while the code accesses them as `short` and as `unsigned short` (the V.8 engine block, v8_obj.h).  `m`
 * may name an array element (`rxHist[3]`) or a member of an embedded struct (`msg[2].crcAcc`).  In src/v8/v8.c it covers the sign / width views of
 * members that are read both ways, the phase pair, the DpRuntime flag word and message words walked with a data-dependent index.
 */
#define MEMBER_AS(T, p, m) (*(T*)((char*)(p) + offsetof(__typeof__(*(p)), m)))

/**
 * MEMBER_AS_AT(T, p, m, bytes): the T at `bytes` past member `m`, the address formed as `(p + offsetof(m)) + bytes` (an indexed element of an array
 * member, or a byte inside a wider member).  MEMBER_AS_ATB: `(p + bytes) + offsetof(m)`, for an offset expression that is added first.
 */
#define MEMBER_AS_AT(T, p, m, bytes) (*(T*)((char*)(p) + offsetof(__typeof__(*(p)), m) + (bytes)))
#define MEMBER_AS_ATB(T, p, m, bytes) (*(T*)((char*)(p) + (bytes) + offsetof(__typeof__(*(p)), m)))

/** The byte address of member `m` of `*p`: `(char *)p + offsetof(m)`.  Used to pass a sub-object's base address on or to store it. */
#define MEMBER_ADDR(p, m) ((char*)(p) + offsetof(__typeof__(*(p)), m))

/**
 * The same accessors for objects addressed by a base that is not a pointer to the struct, with the struct type S named explicitly (the V.32 objects,
 * v32.h).  _OF: the base `p` is a pointer of any type, cast to `char *`: `(char *)p + offsetof(S, m)`.  _ADDR: the base `a` is a `char *`:
 * `*(T *)((a) + offsetof(S, m))`.  Suffixes _AT / _ATB as for MEMBER_AS.  MEMBER_ADDR_A(S, a, m) is the address `(a) + offsetof(S, m)` (a `char *`),
 * MEMBER_ADDR_OF(S, p, m) the byte address `(char *)p + offsetof(S, m)`.  The _ADDR forms add the offset as a SIGNED integer (`(intptr_t)offsetof`):
 * an unsigned offset changes how the compiler reassociates `a + K + i * 2` at -O0.
 *
 * In src/v32 these remain where a view is not the member's own type: a 16-bit half of a 32-bit member, a byte inside a word, sub-object base addresses
 * passed to callees, and array walks whose bound is not established.
 */
#define MEMBER_AS_OF(T, S, p, m) (*(T*)((char*)(p) + offsetof(S, m)))
#define MEMBER_AS_OF_AT(T, S, p, m, bytes) (*(T*)((char*)(p) + offsetof(S, m) + (bytes)))
#define MEMBER_AS_OF_ATB(T, S, p, m, bytes) (*(T*)((char*)(p) + (bytes) + offsetof(S, m)))
#define MEMBER_AS_ADDR(T, S, a, m) (*(T*)((a) + (intptr_t)offsetof(S, m)))
#define MEMBER_AS_ADDR_AT(T, S, a, m, bytes) (*(T*)((a) + (intptr_t)offsetof(S, m) + (bytes)))
#define MEMBER_AS_ADDR_ATB(T, S, a, m, bytes) (*(T*)((a) + (bytes) + (intptr_t)offsetof(S, m)))
#define MEMBER_ADDR_A(S, a, m) ((a) + (intptr_t)offsetof(S, m))
#define MEMBER_ADDR_OF(S, p, m) ((char*)(p) + offsetof(S, m))

/**
 * Element-indexed bases (`short *p`, `int *p`), where the index counts elements, not bytes.  E is the element type of the base.
 * MEMBER_IDX(E, S, p, m) is `((E *)p)[offsetof(S, m) / sizeof(E)]`; _AT / _ATB add an index expression after / before it.  MEMBER_AS_PIDX(T, E, S, p, m)
 * is the T at element offset `(E *)p + offsetof(S, m) / sizeof(E)`, MEMBER_ADDR_PIDX(E, S, p, m) that element's address.  The member offset enters
 * as a signed int.
 */
#define MEMBER_IDX(E, S, p, m) (((E*)(p))[(int)(offsetof(S, m) / sizeof(E))])
#define MEMBER_IDX_AT(E, S, p, m, i) (((E*)(p))[(int)(offsetof(S, m) / sizeof(E)) + (i)])
#define MEMBER_IDX_ATB(E, S, p, m, i) (((E*)(p))[(i) + (int)(offsetof(S, m) / sizeof(E))])
#define MEMBER_AS_PIDX(T, E, S, p, m) (*(T*)((E*)(p) + (int)(offsetof(S, m) / sizeof(E))))
#define MEMBER_ADDR_PIDX(E, S, p, m) ((E*)(p) + (int)(offsetof(S, m) / sizeof(E)))

#ifdef __cplusplus
/**
 * MEMBER_ELEM_FLAT(T, p, m, i): element `i` of the array member `m` (element type T, which must be given exactly) of `*p`.  C++ only.
 * Semantically `p->m[i]`.  When the member lives inside the anonymous unions that embed a sub-object in a generated view (the V.34 object,
 * v34_object_view.h), the compiler splits the constant offset differently between index register and displacement than for a member of a flat
 * struct, which changes the code of the function.  On a 32-bit host this macro therefore addresses the element through a helper struct whose
 * member `a` sits at offsetof(m); on a 64-bit host it is the plain member.
 */
#if __SIZEOF_POINTER__ == 4
template <unsigned DSP_OFF, class DSP_T> struct DspAtOffset
{
  char pad[DSP_OFF];
  DSP_T a[0];
};
#define MEMBER_ELEM_FLAT(T, p, m, i) (((DspAtOffset<offsetof(__typeof__(*(p)), m), T>*)(p))->a[(i)])
#else
#define MEMBER_ELEM_FLAT(T, p, m, i) ((p)->m[(i)])
#endif
#endif

#endif /* DSP_ACCESS_H */
