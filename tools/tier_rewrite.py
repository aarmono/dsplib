#!/usr/bin/env python3
"""tier_rewrite.py - T01: mechanical byte-offset -> typed-member rewrite of the V.34 tier files (and harness files).

The tier files reach the shared V.34 object through macros with a LITERAL offset:
    #define S32(o, off) (*(s32_t *)OBJ_BYTES(o, off))        S32(obj, 0x230)
On x86-64 the pointer slots of the object are 8 bytes, so literal offsets are wrong.  This tool rewrites every such call whose base token is
`obj` (option --base, repeatable) and whose offset is a literal that is the START OF A MEMBER of the V34Obj mirror to the typed form
    S32T(obj, 0x230)  with  #define S32T(o, off) (*(s32_t *)&((V34Obj *)(o))->V34M_##off)      (V34M_0x230 -> minSigLevel_)
so the compiler computes the offset (identical on i386, moved on x86-64).  The accessor type is taken from the file's own macro definition
(tier2: int16_t without may_alias, the others s16_t).  Offsets that are NOT a member start are left as they are (the file stays compilable) and
listed: that list is the worklist of members T02 has to name.  Computed offsets (0x2aa8 + 4*i), other bases (b, m, det, ctx, ...) and
OBJ_BYTES(obj, 0x264) sub-object bases are never touched.
CAVEAT: the tool trusts that the base token `obj` IS the V34 object in every function of the file.  Functions where it is not (v34_agc_demod.c: V34agc / V34demodulate /
agc_level_gain_update / demod_scale / agc_ring_level take the RX front end b, V34ModulatorProcess the modulator; v34_unused.c: preempindex) are listed in
layouts/v34/v34_base_exceptions.txt (`<file> <func> ...`, option --deny-file) and their sites are neither rewritten nor in the worklist.  Identity is only a weak guard (a wrong
base with a same-width member compiles identically): the exceptions file is the review record.
T02: the layout the tool binds to is layouts/v34/v34obj.layout (hand-maintained since V34SUB / V34CLEAN / V34REL; new members come from tools/split_region.py), the unknown regions
that the tier files address by byte offset split into typed members.  An offset INSIDE a scalar member (a byte of a u16, the high half of a dword) is rewritten as
`S16TI(obj, 0x232, minSigLevel_, 2)` = member + delta; an offset inside an unknown-region stays raw (listed by `worklist`).  `--struct KEY` rewrites the sites of ANOTHER base
(b -> rx, m -> mod, q -> rate, p -> info, det -> det, R -> probe, blk -> blkb, ctx, P, k56) through the sub-object structs of src/v34/v34_subobjects.h (`slice`, spec layouts/v34/v34sub.txt)
or the external-class headers (v34_flomodem_view.h = VPcmFloModem of layouts/vpcm/vpcm_flo_modem.layout, v34_modem_params_view.h, v34_k56flex_view.h); `--hazards` lists the x86-64 hazards of the resolved sites (an integer
accessor on a pointer slot, a pointer accessor on a non-pointer member, an access that spans members with padding between them on x86-64).

Subcommands
  header  --layout layouts/v34/v01.layout [--class tagV34Object] [-o src/v34/v34_object_view.h] [--check]
          generate (or, with --check, verify that the committed file equals the freshly generated one) the standalone header: the mirror
          struct with REAL pointer members + i386 _Static_asserts + one `#define V34M_0x230 minSigLevel_` per member (members in
          unknown-region count as members: their start offset is the region start; `--sites` reports how many sites land on one).
          RS22: plus `#define V34M_minSigLevel minSigLevel_` for every member whose name is not just an offset (the same for the sub-object
          structs of `slice`: V34RX_rxFlags ...): the accessors take either spelling, S16T_rx(b, rxFlags) == S16T_rx(b, 0x122).
  rewrite FILE... [-o OUT | --in-place] [--header HDR] [--layout ...] [--base obj] [--mode partial|all]
          rewrite.  partial (default): rewrite only resolvable sites, leave the rest; all: rewrite every literal-obj site (unresolved ones then
          fail to compile with "no member named V34M_0x..." - the compiler-made worklist).  Prints per file: sites, rewritten, worklist.
  worklist FILE... [--json]   no output file; the non-member offsets per file (count of sites per offset).
  check   NEW --orig OLD [-I dir]... [--opt -O0 -O2 ...] [--cflags ...]
          the 32-bit identity test: gcc -m32 -O<n> -fno-pie -fcf-protection=none -fno-stack-protector -c of OLD and NEW, then
          `objdump -dr --no-show-raw-insn` of both must be IDENTICAL (default -O0, -O2 and the PINNED set -O2 -fno-strict-aliasing = PIN_FLAGS of flags.mk; --opt takes quoted flag strings).  Exit 0 only if every level is identical.
          With --orig omitted the OLD text is `git show HEAD:<NEW>`.
  apply   FILE... [--layout ...]   rewrite --in-place, then check against the HEAD version; restores the file if the identity fails.  Tries the plain member spelling
          `*(T *)&((V34Obj *)(o))->V34M_##off` first and, when only -O0 differs (an exact-type accessor, e.g. int32_t on an int32_t member, is folded by gcc -O0 into one
          addressing mode), the offsetof spelling `*(T *)((unsigned char *)(o) + offsetof(V34Obj, V34M_##off))` (tier2).  Harness files: `--base obj --base g_obj --header ../src/v34/v34_object_view.h`.
  mapheader -o FILE [--check]   (T04) the i386 -> native offset map of the V34Obj mirror (one entry per member: i386 offset / native offset / sizes / is-pointer) for the
          marshalling layer (the harness image layer) of the x86-64 harnesses
  shims   HARNESS.c [--only f...] [--list]   (T04) (re)generate the block between `/* V34SHIMS */` (or BEGIN/END) markers: for every `extern` prototype of the file with a pointer parameter /
          result a `static V34SH_<f>(...)` wrapper that marshals the harness' IMAGE objects around the call (v34i_enter / v34i_in / v34i_leave), `V34REAL_<f>` (the real address) and
          `#define <f> V34SH_<f>`; the whole block is x86-64 only (-m32 object code unchanged: tools/ident32.sh)
  bytes   FILE... [--accessors] [--macros] [--base TOK ...] [--sub KEY [--funcs F ...]] [--apply]   (T04) rewrite the LITERAL of OBJ_BYTES(obj, LIT [+ rest]) (and with --accessors NAME(obj, LIT [+ rest]))
          to V34OFF(0x..) / V34OFFI(member, delta) (native offset constants; the expression keeps its shape, so the i386 code is identical); with --sub KEY the base token addresses the
          sub-object KEY of layouts/v34/v34sub.txt and the literal becomes V34REL(base, abs) / V34RELI / V34RELJ (native DISTANCE from the sub-object base member inside the flat object); prints the
          contiguous run to the next pointer member for each site; --apply checks 32-bit identity (-O0, -O2, pinned) and restores the file on failure
  paths   [-o layouts/v34/v34obj_paths.txt] [--check]   (V34SUB) `<member> <path>` of every member / C++ marker that lives in a SOLE sub-object struct (`sole` in layouts/v34/v34sub.txt), for layout_gen.py --paths
  slice   [-o src/v34/v34_subobjects.h] [--check]   the sub-object structs of layouts/v34/v34sub.txt (slices of the V34Obj mirror with real pointer members + a report of how many members have the
          same relative x86-64 offset as in the flat V34Obj)
"""
import argparse
import difflib
import json
import os
import re
import subprocess
import sys
import tempfile
import textwrap

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import port_objs  # noqa: E402  (the library include root, sources.mk DSP_INCDIR)
INCROOT = os.path.join(ROOT, port_objs.incdir())
sys.path.insert(0, HERE)
import layout_gen as lg  # noqa: E402

DEFAULT_LAYOUT = os.path.join(ROOT, 'layouts', 'v34', 'v34obj.layout')
DEFAULT_SUB = os.path.join(ROOT, 'layouts', 'v34', 'v34sub.txt')
EXTERNAL = {   # key: (layout, class, prefix, header file)
    'ctx': ('layouts/vpcm/vpcm_flo_modem.layout', 'VPcmFloModem', 'VPFM_', 'v34_flomodem_view.h'),
    'P': ('layouts/v34/v01.layout', '_tagModemParameters', 'V34P_', 'v34_modem_params_view.h'),
    'v92i': ('layouts/vpcm/vpcm_flo_modem.layout', 'V92Phase2InfoView', 'V34V92I_', 'v34_v92_phase2_view.h'),   # T05: *(ctx + 0x612c) = the V92Modem info record
    'v90i': ('layouts/vpcm/vpcm_flo_modem.layout', 'V90Phase2InfoView', 'V34V90I_', 'v34_v90_phase2_view.h'),   # T05: *(ctx + 0x1760) = the V90Modem info record
    'cfg': ('layouts/vpcm/pcm_parameters.layout', 'V90Parameters', 'V34CFG_', 'v34_v90params_view.h'),   # T05: the V90Parameters block *(ctx + 0x610c) of the tier files
    'k56': ('layouts/v34/v01.layout', 'K56FlexHandle', 'V34K56_', 'v34_k56flex_view.h')}
DEFAULT_DENY = os.path.join(ROOT, 'layouts', 'v34', 'v34_base_exceptions.txt')
DEFAULT_HEADER = os.path.join(ROOT, 'src/v34/v34_object_view.h')
PREFIX = 'V34M_'
OFFSET_STYLE_NAME = re.compile(r'^(?:unk|aln|alna|ba|p|w|wA|wa|wE|wAb|wAa|b_|d|i|q|mp|rd|dg|moh)[0-9a-fA-F]{1,5}_?$')   # RS22: names that only say an offset get no descriptive key
TGT_OBJ = dict(prefix='V34M_', ctype='V34Obj', t='T', ti='TI', header='src/v34/v34_object_view.h')


# --------------------------------------------------------------------------------------------- header generator
def load_class(layout, cname):
    classes = lg.parse(open(layout).read(), layout)
    for c in classes:
        lg.check_i386(c, layout)
    for c in classes:
        if c.name == cname:
            return c
    sys.exit('%s: no class %s' % (layout, cname))


def members(c):
    """offset -> (member name, kind) for every member that starts at that offset (zero-size align8 markers excluded)"""
    m = {}
    for f in c.fields:
        if f.kind == 'align8':
            continue
        m.setdefault(f.off, (f.name, f.kind))
    return m


def interior(c, off):
    """(member name, delta) when `off` lies strictly inside a scalar / scalar-array member that is not a pointer or an unknown-region, else None"""
    for f in c.fields:
        if f.kind in ('align8', 'unknown-region') or 'ptr' in f.kind or f.kind == 'vptr':
            continue
        sz = lg.KINDS[f.kind][2] * (f.count or 1)
        if f.off < off < f.off + sz:
            return f.name, off - f.off
    return None


def layout64(c):
    """x86-64 offsets of the mirror members (natural alignment; pointers and doubles 8): {name: (off64, size64)}"""
    pos, out = 0, {}
    for f in c.fields:
        if f.kind == 'align8':
            pos = (pos + 7) // 8 * 8
            continue
        _, _, sz, al = lg.KINDS[f.kind]
        if 'ptr' in f.kind or f.kind in ('vptr', 'double', 'u32-ptrslot'):
            sz = al = 8
        pos = (pos + al - 1) // al * al
        out[f.name] = (pos, sz * max(f.count, 1))
        pos += sz * max(f.count, 1)
    return out


ACC_W = {1: ('u8', 's8', 'uint8_t', 'int8_t', 'u8_t', 's8_t', 'char'), 2: ('s16_t', 'u16_t', 'int16_t', 'uint16_t', 'short'),
         4: ('s32_t', 'u32_t', 'int32_t', 'uint32_t', 'int', 'unsigned', 'unsigned int')}


def acc_width(ty):
    """(width on i386, is pointer accessor) of an accessor C type text, or (None, False)"""
    ty = ty.strip()
    if '*' in ty or ty.startswith('ptr'):
        return 4, True
    for w, names in ACC_W.items():
        if ty in names:
            return w, False
    return None, False


def embed_list(c, spec):
    """T05: [(member name, key, struct, base, size)] of the sub-object instances that V34Obj EMBEDS (layouts/v34/v34sub.txt): every instance whose shape equals that of the first one
    (the other V34Info records differ in shape and stay flat).  One instance: member = key; several: key + index (echo0, echo1)."""
    out = []
    for key, name, bases, size, _ in spec:
        sc = make_slice(c, name, bases[0], size)
        ok = [b for b in bases if slice_shapes_equal(sc, make_slice(c, name, b, size)) and any(f.off == b and f.kind != 'align8' for f in c.fields)]
        for i, b in enumerate(ok):
            out.append((key if len(ok) == 1 else '%s%d' % (key, i), key, name, b, size))
    return out


class CxxSpec:
    """RS5: the C++-facing additions to the V34Obj view, from layouts/v34/v34obj_cxx.txt.  The view is the ONE definition of the V.34 module object: the C++ ports
    include it too (src/v34/v34_object.h), so it must (a) type the members the way the C++ code uses them (pointers to classes, signed 16-bit words), (b) keep the
    names the C++ code uses for regions the tier split into several members (zero-size markers `uint8_t name[0]` at the offset: same address, no storage),
    (c) overlay the 0x20 / 0x30-byte records the C++ code names as structs (anonymous union over the flat members).
        include <header>                        a header with the record structs (plain C)
        fwd struct NAME                         forward declaration for the typed pointers
        type <member> <ctype>[<n>]              the member is declared with this type (replaces the layout's)
        name <member> <ctype> <offset>          a zero-size marker array `<ctype> <member>[0]` at that i386 offset (declared before the member that starts there)
        pathname <member> <path>                a layout member of the C++ class that is a sub-object (or a member of it) of the V34Obj struct: only the `paths` file gets the line
        record <member> <ctype> <offset> <size> the record struct <ctype> overlays the flat members of [offset, offset + size); members of the layout with a
                                                designator `<member>.<m>` are asserted to sit at offsetof(<ctype>, <m>)"""

    def __init__(self, path):
        self.includes, self.fwd, self.types, self.names, self.records, self.pathnames = [], [], {}, [], [], {}
        for ln in open(path):
            ln = ln.split('#')[0].strip()
            if not ln:
                continue
            w = ln.split()
            if w[0] == 'include':
                self.includes.append(w[1])
            elif w[0] == 'fwd':
                self.fwd.append(' '.join(w[1:]))
            elif w[0] == 'type':
                self.types[w[1]] = ' '.join(w[2:])
            elif w[0] == 'pathname':
                self.pathnames[w[1]] = w[2]
            elif w[0] == 'name':
                self.names.append((w[1], w[2], int(w[3], 0)))
            elif w[0] == 'record':
                self.records.append((w[1], ' '.join(w[2:-2]), int(w[-2], 0), int(w[-1], 0)))
            else:
                sys.exit('%s: bad line %r' % (path, ln))


CXX = None      # the CxxSpec in force while the V34Obj header is generated
CXX_PATH_FOR_CMD = 'layouts/v34/v34obj_cxx.txt'


def _field_lines(f):
    _, ct, _, _ = lg.KINDS[f.kind]
    if f.kind == 'align8':
        return '%s[0] __attribute__((aligned(sizeof(void *)))); /* 0x%02x align8 */' % ('char ' + f.name, f.off)
    arr = '[%d]' % f.count if f.count else ''
    if CXX and f.name in CXX.types:
        t = CXX.types[f.name]
        m = re.match(r'^(.*?)\s*(\[\d+\])?$', t)
        ct, arr = m.group(1), m.group(2) or ''
        CXX_USED.add(f.name)
        return '%s%s%s; /* 0x%02x %s, C++ type */' % (ct + ('' if ct.endswith('*') else ' '), f.name, arr, f.off, f.kind)
    return '%s%s%s; /* 0x%02x %s */' % (ct + ('' if ct.endswith('*') else ' '), f.name, arr, f.off, f.kind)


CXX_USED = set()
MARK_DONE = set()


def sole_list(embeds):
    """the SOLE instances of `embeds` (embed_list tuples (member, key, struct, base, size))"""
    return [e for e in (embeds or []) if e[1] in SOLE]


def top_sole(embeds):
    """the sole instances that lie in no other sole instance (a nested one is a member of its outer struct)"""
    s = sole_list(embeds)
    return [e for e in s if not any(o is not e and o[3] <= e[3] and e[3] + e[4] <= o[3] + o[4] for o in s)]


def field_paths(fields, embeds):
    """{member name: member path} of the members in `fields` that lie inside a SOLE embed: `det.p3564_`, `rx.blkb.w0a00_`, `echo1.echoBuf_`.  The members of a second
    instance take the names of the first (one struct, one set of names)."""
    sole = sole_list(embeds)
    paths = {}
    flat = [f for f in fields if f.kind != 'align8']
    for f in flat:
        chain = sorted([e for e in sole if e[3] <= f.off < e[3] + e[4]], key=lambda e: -e[4])
        if not chain:
            continue
        inner = chain[-1]
        b0 = next(e[3] for e in sole if e[1] == inner[1])
        g = next(x for x in flat if x.off == f.off - inner[3] + b0)
        paths[f.name] = '.'.join([e[0] for e in chain] + [g.name])
    return paths


def _marker_line(mn, mt, mo, ind):
    return ind + '%s %s[0]; /* 0x%02x C++ name of a region the tier split: same address, no storage */' % (mt, mn, mo)


def _emit_fields(fields, embeds, ind, records=(), markers_done=None, delta=None):
    """lines for `fields`; an embedded sub-object range becomes  union { struct { <flat members> }; Struct name; };  (recursively: V34BlkB lies inside V34Rx, V34PreFilter inside V34Mod),
    a SOLE one (SOLE) the member  Struct name;  alone.
    RS5: a record (CxxSpec.records) is such a range too; ranges with the same base and size share ONE union; the CxxSpec markers are emitted before the first member at their offset.
    `delta` is None for the V34Obj itself, else the i386 offset of the sub-object whose struct is being emitted (its fields have relative offsets): the markers are then matched at
    delta + offset, and the one at the base of the struct is left to the object that holds it."""
    out, i = [], 0
    markers_done = MARK_DONE if markers_done is None else markers_done
    d = delta or 0
    items = [(e[0], e[2], e[3], e[4]) for e in embeds] + [(r[0], r[1], r[2], r[3]) for r in records]
    sole_names = {e[0] for e in sole_list(embeds)}
    while i < len(fields):
        f = fields[i]
        if CXX and f.kind != 'align8':
            for (mn, mt, mo) in CXX.names:
                if mo == f.off + d and mn not in markers_done and not (delta is not None and mo == d):
                    markers_done.add(mn)
                    out.append(_marker_line(mn, mt, mo, ind))
        iv = [x for x in items if x[2] == f.off]
        if not iv:
            out.append(ind + _field_lines(f))
            i += 1
            continue
        e = max(iv, key=lambda x: x[3])
        end = e[2] + e[3]
        same = [x for x in iv if x[3] == e[3]]
        j = i
        while j < len(fields) and fields[j].off < end:
            fe = fields[j].off + (lg.KINDS[fields[j].kind][2] * (fields[j].count or 1) if fields[j].kind != 'align8' else 0)
            if fe > end:
                sys.exit('embed %s [0x%x, 0x%x): member %s crosses the end' % (e[0], e[2], end, fields[j].name))
            j += 1
        if e[0] in sole_names:
            if len(same) != 1:
                sys.exit('embed %s: a sole sub-object cannot share its range with %s' % (e[0], ', '.join(x[0] for x in same if x is not e)))
            base_marks = [(mn, mt, mo) for (mn, mt, mo) in (CXX.names if CXX else ()) if mo == e[2] + d and mn not in markers_done]
            if fields[i].kind == 'align8' and (base_marks or not any('ptr' in x.kind for x in fields[i:j])):
                out.append(ind + _field_lines(fields[i]))   # the pointer alignment of the struct (and of the C++ marker names that stand at its address)
            for (mn, mt, mo) in base_marks:   # the marker at the base of the struct stays beside it
                markers_done.add(mn)
                out.append(_marker_line(mn, mt, mo, ind))
            emb = next(x for x in embeds if x[0] == e[0])
            first = next(x[3] for x in sole_list(embeds) if x[1] == emb[1])
            out.append(ind + '%s %s;' % (e[1], e[0]))
            i = j
            continue
        inner = [x for x in items if x not in same and x[2] >= e[2] and x[2] + x[3] <= end]
        inner_e = [x for x in embeds if (x[0], x[2], x[3], x[4]) in inner]
        inner_r = [x for x in records if (x[0], x[1], x[2], x[3]) in inner]
        out.append(ind + 'union { /* %s: %s at 0x%x */' % (e[0], e[1], e[2]))
        out.append(ind + '  struct {')
        out += _emit_fields(fields[i:j], inner_e, ind + '    ', inner_r, markers_done, delta)
        out.append(ind + '  };')
        for x in same:
            out.append(ind + '  %s %s;' % (x[1], x[0]))
        out.append(ind + '};')
        i = j
    return out


# RS10: the V34VIEW_ASSERT block is the hand-written src/v34/v34_view_assert.h (it was emitted into each of the 7 generated headers); the generated header
# includes it (bare name: resolved next to the including header, so "v34/v34_object_view.h" from src/vpcm works too).
ASSERT_MACRO = ['#include "v34_view_assert.h"']


def gen_block(c, prefix, tname=None, note='', embeds=None, tag=None, keys=False, delta=None):
    """typedef struct (real pointer members) + i386 asserts + one `#define <prefix>0x<off> <member>` per member start.  T05: with `embeds` (embed_list) the V34Obj mirror embeds the
    sub-object structs of src/v34/v34_subobjects.h as anonymous unions over the flat members (the flat names keep working) + _Static_asserts that every sub-object member sits at the same
    relative offset in the sub-object struct and in the flat object (32-bit x86 AND x86-64).  A SOLE embed (SOLE) is a named member only; its flat members do not exist, the keys name
    the member path (`det.p3564_`).  `delta` (slice of a sole sub-object): the i386 offset of the struct in the V34Obj; `embeds` are then the sole sub-objects nested in it, with RELATIVE bases."""
    tname = tname or c.mirror
    for e in sole_list(embeds):
        for o2 in (embeds or []):
            if o2[1] not in SOLE and o2[3] <= e[3] and e[3] + e[4] <= o2[3] + o2[4] and o2 is not e:
                sys.exit('layouts/v34/v34sub.txt: the sole sub-object %s lies in %s, which is not sole' % (e[0], o2[0]))
    paths = field_paths(c.fields, embeds)
    pth = lambda f: paths.get(f.name, f.name)
    head = '%s (32-bit sizeof 0x%x)%s' % (c.name, c.size, note)
    if len(head) + 6 <= 150:
        o = ['/* ' + head + ' */']
    else:
        o = ['/*'] + [' * ' + w for w in textwrap.wrap(head, 150)] + [' */']
    o.append('typedef struct %s{' % (tag + ' ' if tag else ''))
    if embeds or (CXX and (CXX.records or CXX.names)):
        MARK_DONE.clear()
        CXX_USED.clear()
        o += _emit_fields(c.fields, embeds or [], '  ', CXX.records if (CXX and delta is None) else (), None, delta)
        if CXX and delta is None:
            inside = lambda off: any(e[3] < off < e[3] + e[4] for e in sole_list(embeds))   # a marker inside a sole struct is emitted by its slice
            lost = [n for (n, _, mo) in CXX.names if n not in MARK_DONE and not inside(mo)] + ['type ' + n for n in CXX.types if n not in CXX_USED and n not in paths]
            if lost:
                sys.exit('layouts/v34/v34obj_cxx.txt: not applied (no member / no field starts at that offset): %s' % ', '.join(lost))
    else:
        for f in c.fields:
            o.append('  ' + _field_lines(f))
    o.append('} %s;' % tname)
    o.append('#if defined(__i386__)')
    conds = ['sizeof(%s) == 0x%x' % (tname, c.size)] + ['offsetof(%s, %s) == 0x%x' % (tname, pth(f), f.off) for f in c.fields if f.kind != 'align8']
    o.append('V34VIEW_ASSERT(%s,' % ' &&\n                   '.join(conds))
    o.append('               "%s mirror layout (32-bit)");' % c.name)
    o.append('#endif')
    seen = set()
    for f in c.fields:
        if f.kind == 'align8' or f.off in seen:
            continue
        seen.add(f.off)
        o.append('#define %s0x%x %s' % (prefix, f.off, pth(f)))
    if keys:
        # RS22: a descriptive key for every member whose name says something: S16T(obj, rxState) is S16T(obj, 0x3594).  The key is the member name without its trailing
        # underscore; members named by their offset (w3598_, unk0266_, p0268_ ...) get none, the offset key stays the only spelling of those.
        o.append('/* descriptive keys: %s<member name without the trailing underscore> is the same member as %s0x<offset> */' % (prefix, prefix))
        seen = set()
        keys = set()
        for f in c.fields:
            if f.kind == 'align8' or f.off in seen:
                continue
            seen.add(f.off)
            if OFFSET_STYLE_NAME.match(f.name):
                continue
            key = f.name.rstrip('_')
            assert key not in keys, 'duplicate descriptive key ' + key
            keys.add(key)
            o.append('#define %s%s %s' % (prefix, key, pth(f)))
    if embeds and delta is None and top_sole(embeds):
        o.append('#if defined(__i386__)')
        o.append('V34VIEW_ASSERT(%s,' % ' &&\n                   '.join('offsetof(%s, %s) == 0x%x && sizeof(%s) == 0x%x' % (tname, e[0], e[3], e[2], e[4]) for e in top_sole(embeds)))
        o.append('               "sole sub-objects: base and size (32-bit)");')
        o.append('#endif')
    if embeds and delta is not None:
        o.append('#if defined(__i386__)')
        o.append('V34VIEW_ASSERT(%s,' % ' &&\n                   '.join('offsetof(%s, %s) == 0x%x && sizeof(%s) == 0x%x' % (tname, e[0], e[3], e[2], e[4]) for e in sole_list(embeds)))
        o.append('               "nested sub-objects: base and size (32-bit)");')
        o.append('#endif')
    if embeds and delta is None and [e for e in embeds if e[1] not in SOLE]:
        flat = [f for f in c.fields if f.kind != 'align8']
        o.append('/* the embedded sub-objects (32-bit x86 and x86-64): offset of the sub-object and, member by member, the same relative offset as in the flat object */')
        for (name, key, st, base, size) in embeds:
            if key in SOLE:
                continue
            o.append('V34VIEW_ASSERT(offsetof(%s, %s) == offsetof(%s, V34M_0x%x), "%s: base");' % (tname, name, tname, base, name))
            sl = [f for f in flat if base <= f.off < base + size]
            b0 = next(b for (n2, k2, s2, b, z2) in embeds if k2 == key)   # the first instance names the struct members
            s0 = [f for f in flat if b0 <= f.off < b0 + size]
            assert len(s0) == len(sl)
            for f, g in zip(sl, s0):
                o.append('V34VIEW_ASSERT(offsetof(%s, %s) - offsetof(%s, %s) == offsetof(%s, %s), "%s.%s");' % (tname, f.name, tname, name, st, g.name, name, g.name))
    if CXX and CXX.records and delta is None:
        flat = [f for f in c.fields if f.kind != 'align8']
        o.append('/* the records the C++ ports name as structs overlay the flat members (same bytes): size of the record, base, and member by member the same relative offset */')
        for (rn, rt, ro, rs) in CXX.records:
            o.append('V34VIEW_ASSERT(sizeof(%s) == 0x%x, "%s: size");' % (rt, rs, rn))
            o.append('V34VIEW_ASSERT(offsetof(%s, %s) == offsetof(%s, V34M_0x%x), "%s: base");' % (tname, rn, tname, ro, rn))
            for f in flat:
                if f.cxxpath.startswith(rn + '.'):
                    o.append('V34VIEW_ASSERT(offsetof(%s, %s) - offsetof(%s, %s) == offsetof(%s, %s), "%s");' % (tname, f.name, tname, rn, rt, f.cxxpath[len(rn) + 1:], f.cxxpath))
    o.append('')
    return o


def gen_header(c, layout_rel, prefix=PREFIX, guard='V34OBJ_H', out='src/v34/v34_object_view.h', cmd=None, embeds=None, tag=None):
    cmd = cmd or 'python3 tools/tier_rewrite.py header --layout %s%s%s%s%s -o %s' % (
        layout_rel, ' --class ' + c.name if c.name != 'tagV34Object' else '', ' --prefix ' + prefix if prefix != PREFIX else '',
        ' --guard ' + guard if guard != 'V34OBJ_H' else '', (' --cxx ' + CXX_PATH_FOR_CMD if CXX else '') + (' --tag ' + tag if tag else ''), out)
    fname = os.path.basename(out)
    o = ['/**',
         ' * @file %s' % fname,
         ' * @brief %s' % ('The one C / C++ definition of the V.34 module object %s (typed members at fixed 32-bit offsets) and the %s accessor keys.' % (c.name, prefix)
                           if tag == 'tagV34Object' else 'C mirror of %s (typed members, real pointer members) and the %s accessor keys.' % (c.name, prefix)),
         ' *',
         ' * GENERATED by tools/tier_rewrite.py header from %s (class %s): do not edit.  Regenerate with' % (layout_rel, c.name),
         ] + [(' *   ' if i == 0 else ' *       ') + w for i, w in enumerate(textwrap.wrap(cmd, 140, break_on_hyphens=False, break_long_words=False))] + [
         ' *',
         ' * The %s mirror holds the members of the layout, without the test-harness parts.  Pointer members are real pointers (8 bytes on x86-64),' % c.mirror,
         ' * so the compiler computes the member offsets on the host; the 32-bit offsets are pinned by the V34VIEW_ASSERT block active when compiled for 32-bit x86.',
         ' * Every member start has a key `#define %s0x230 <member>` used by the typed accessors of the V.34 files:' % prefix,
         ' *   #define S32T(o, off) (*(s32_t *)&((%s *)(o))->%s##off)' % (c.mirror, prefix),
         ' * S32T(obj, 0x230) names the member that starts at offset 0x230 of the 32-bit object, wherever that member lands on the host.',
         ' * An offset that is not a member start has no %s macro: using it fails to compile, which is the point.' % prefix,
         ' * Every member with a descriptive name also has a descriptive key, `#define %s<name> <member>` (the name without a trailing underscore):' % prefix,
         ' * the accessors take S32T(obj, minSigLevel) as well as S32T(obj, 0x230).']
    if embeds:
        sole = [e[0] for e in embeds if e[1] in SOLE and e is not None and not any(o2 is not e and o2[1] in SOLE and o2[3] <= e[3] and e[3] + e[4] <= o2[3] + o2[4] for o2 in embeds)]
        uni = [e[0] for e in embeds if e[1] not in SOLE]
        o += [' *']
        if sole:
            o += [' * ' + w for w in textwrap.wrap(
                'The sub-objects of src/v34/v34_subobjects.h are members of the object and their struct is the ONLY definition of their members: %s (V34Rx contains '
                'V34BlkB blkb, V34Mod contains V34PreFilter pf).  The key macros name the member path (`#define V34M_0x3564 det.p3564_`), so `obj->det.toneDetF6_` is the '
                'spelling; a second instance of a struct (echo1) uses the member names of the first (echo1.echoBuf_).' % ', '.join(sole), 145)]
        if uni:
            o += [' * ' + w for w in textwrap.wrap(
                'The records %s are anonymous unions over the flat members of the object (and of their struct), so `obj->info.x` and the flat member are the same lvalue; '
                'the _Static_asserts after the struct prove, for 32-bit x86 and for x86-64, that the struct and the flat members agree member by member.' % ', '.join(uni), 145)]
    o.append(' */')
    o += ['#ifndef ' + guard, '#define ' + guard, '', '#include <stddef.h>', '#include <stdint.h>']
    if embeds:
        o.append('#include "v34_subobjects.h"')
    o += ASSERT_MACRO
    if CXX:
        o += ['#include "%s"' % h for h in CXX.includes] + ['%s;' % f for f in CXX.fwd]
    o.append('')
    o += gen_block(c, prefix, embeds=embeds, tag=tag, keys=True)
    if tag == 'tagV34Object':
        o += ['', '#include "v34_object_layout64.inc" /* x86-64 layout asserts (tools/layout64.py) */']
    o.append('#endif /* %s */' % guard)
    return '\n'.join(o) + '\n'


def gen_map_header(c, layout_rel, out='test/v34/v34objmap.h', name='V34OBJ', inc='../src/v34/v34_object_view.h', cmd=None, paths=None):
    """T04: the i386 -> native offset map of the mirror, for the harness marshalling layer (the harness image layer): one entry per member:
    {i386 offset, native offset, i386 size, native size, is-pointer}.  Pointer kinds are 4 bytes in the image and sizeof(void *) natively.
    T05: also for other classes (ctx / cfg): `name` prefixes the NAME_MAP / NAME_SIZE32 / NAME_NMAP / NAME_NATSZ symbols."""
    guard = os.path.basename(out).upper().replace('.', '_')
    o = ['/* GENERATED by tools/tier_rewrite.py mapheader from %s (class %s): do not edit.  Regenerate with' % (layout_rel, c.name),
         ' *   ' + (cmd or 'python3 tools/tier_rewrite.py mapheader -o %s' % out),
         ' * The 32-bit IMAGE -> native %s offset map used by the x86-64 harnesses (the harness image layer): the harness keeps its model of the object in the' % c.mirror,
         ' * i386 layout (the layout of the blob and of the recorded goldens) and marshals it member by member into/out of a native %s around every call of a' % c.mirror,
         ' * function under test.  Not for production code. */',
         '#ifndef ' + guard, '#define ' + guard, '', '#include "%s"' % inc, '',
         '#ifndef V34MAPENT_DEFINED', '#define V34MAPENT_DEFINED', 'typedef struct { uint32_t o32, o64, s32, s64; uint8_t isptr; } V34MapEnt;', '#endif',
         '#define %s_SIZE32 0x%x' % (name, c.size),
         '#define %s_NATSZ sizeof(%s)' % (name, c.mirror),
         'static const V34MapEnt %s_MAP[] = {' % name]
    for f in c.fields:
        if f.kind == 'align8':
            continue
        _, _, sz, _ = lg.KINDS[f.kind]
        isp = 1 if ('ptr' in f.kind and f.kind != 'u32-ptrslot') or f.kind == 'vptr' else 0
        if f.kind == 'u32-ptrslot':
            isp = 1
        pf = (paths or {}).get(f.name, f.name)   # a member of a sole sub-object is reached through its path (det.p3564_)
        o.append('  {0x%x, offsetof(%s, %s), %d, sizeof(((%s *)0)->%s), %d},' % (f.off, c.mirror, pf, sz * max(f.count, 1), c.mirror, pf, isp))
    o += ['};', '#define %s_NMAP (sizeof %s_MAP / sizeof %s_MAP[0])' % (name, name, name), '', '#endif /* %s */' % guard]
    return '\n'.join(o) + '\n'


# --------------------------------------------------------------------------------------------- T04: harness call shims
SHIM_SKIP = {'printf', 'fprintf', 'sprintf', 'snprintf', 'memcpy', 'memset', 'memcmp', 'strcpy', 'strcmp', 'strlen', 'malloc', 'calloc', 'free', 'exit', 'abort',
             'dsplibs_debug_printf', 'edprintf', 'fopen', 'fclose', 'fwrite', 'fread', 'puts', 'qsort', 'getenv', 'atoi', 'sysdep_malloc', 'sysdep_free'}
TYPE_WORDS = {'void', 'int', 'short', 'char', 'long', 'unsigned', 'signed', 'float', 'double', 'const', 'volatile', 'struct', 'size_t', 'uintptr_t', 'intptr_t',
              'uint8_t', 'uint16_t', 'uint32_t', 'uint64_t', 'int8_t', 'int16_t', 'int32_t', 'int64_t'}
SHIM_BEGIN = '/* V34SHIMS BEGIN (generated by tools/tier_rewrite.py shims; do not edit) */'
SHIM_END = '/* V34SHIMS END */'
SHIM_MARK = '/* V34SHIMS */'


def split_params(txt):
    out, depth, cur = [], 0, ''
    for ch in txt:
        if ch in '([':
            depth += 1
        elif ch in ')]':
            depth -= 1
        if ch == ',' and depth == 0:
            out.append(cur.strip())
            cur = ''
        else:
            cur += ch
    if cur.strip():
        out.append(cur.strip())
    return out


def param_type_name(p, i):
    """(type text, name) of a parameter declaration; an unnamed parameter gets a<i>"""
    m = re.match(r'^(.*?)([A-Za-z_]\w*)\s*(\[\s*\w*\s*\])?$', p, re.S)
    if m and m.group(2) not in TYPE_WORDS and (m.group(1).strip() or '*' in m.group(1)) and not re.match(r'^\s*(const\s+)?(struct\s+)?\w+\s*$', p):
        ty = m.group(1).strip() + (' *' if m.group(3) else '')
        return ty, m.group(2)
    return p, 'a%d' % i


def shim_block(src_before, only=None):
    """the generated shim block for every extern function prototype in src_before that has a pointer parameter or a pointer result"""
    out, names, skipped = [], [], []
    for m in re.finditer(r'^extern\s+([^;()]*?)\b([A-Za-z_]\w*)\s*\(((?:[^;()]|\([^()]*\))*)\)\s*(?:[A-Z_a-z_]\w*\([^;]*\))?\s*;', src_before, re.M | re.S):
        ret, name, ptxt = m.group(1).strip(), m.group(2), m.group(3)
        if name in SHIM_SKIP or (only and name not in only):
            continue
        params = [] if ptxt.strip() in ('', 'void') else split_params(ptxt)
        if any('...' in p or '(*' in p for p in params):
            skipped.append(name)
            continue
        pt = [param_type_name(p, i) for i, p in enumerate(params)]
        retptr = '*' in ret
        if not retptr and not any('*' in t or t.strip().endswith(']') for t, _ in pt):
            continue
        ret_t = ret if ret else 'int'
        decl = ', '.join('%s %s' % (t, n) if not t.rstrip().endswith('*') else '%s%s' % (t, n) for t, n in pt) or 'void'
        args = ', '.join(('(%s)v34i_in((void *)%s)' % (t, n)) if '*' in t else n for t, n in pt)
        isvoid = ret_t.strip() == 'void'
        rv = 'v34r_' if any(n == 'r' for _, n in pt) else 'r'   # T06: a parameter named r (getbit) must not clash with the result variable
        body = 'v34i_enter(); %s%s(%s); v34i_leave(); ' % ('' if isvoid else rv + ' = ', name, args)
        if not isvoid:
            body += 'return %s;' % (('(%s)v34i_outp((void *)%s)' % (ret_t, rv)) if retptr else rv)
        out.append('static %s V34SH_%s(%s) { %s%s}' % (ret_t, name, decl, '' if isvoid else '%s %s; ' % (ret_t, rv), body))
        names.append(name)
    blk = [SHIM_BEGIN, '#if defined(__x86_64__)'] + out + ['static void *const V34REAL_%s = (void *)%s; /* the real address (the name is a macro for the shim below) */' % (n, n) for n in names] + ['#define %s V34SH_%s' % (n, n) for n in names] + ['#endif', SHIM_END]
    return '\n'.join(blk), names, skipped


# --------------------------------------------------------------------------------------------- T04: OBJ_BYTES(obj, <literal> [+ rest]) derivations
def find_bytes_sites(t, mask, bases, accs=None, in_macros=False):
    """[(start, end, base, litvalue, littext, rest or None)] for OBJ_BYTES(base, LIT [+ rest]) and, with `accs`, NAME(base, LIT [+ rest]) of the file's accessor macros (a LIT that is already
    a typed-member site is gone; what is left is a non-member / computed offset) outside comments / #define lines; other argument shapes are returned with lit None"""
    out = []
    names = ['OBJ_BYTES'] + sorted(accs or [], key=len, reverse=True)
    for m in re.finditer(r'(?<![\w.>])(?:%s)\(\s*(%s)\s*,' % ('|'.join(map(re.escape, names)), '|'.join(map(re.escape, bases))), mask):
        ls = mask.rfind('\n', 0, m.start()) + 1
        if not in_macros and re.match(r'\s*#\s*define', mask[ls:m.start()]):
            continue
        depth, i = 1, m.end()
        while i < len(mask) and depth:
            depth += {'(': 1, ')': -1}.get(mask[i], 0)
            i += 1
        arg = t[m.end():i - 1]
        am = re.match(r'^\s*(0[xX][0-9a-fA-F]+|[0-9]+)[uUlL]*\s*(?:[+-]\s*(.*?))?\s*$', arg, re.S)
        if am:
            out.append((m.start(), i, m.group(1), int(am.group(1), 0), am.group(1), am.group(2)))
        else:
            out.append((m.start(), i, m.group(1), None, arg.strip(), None))
    return out


def bytes_binding(cls, mem, lit):
    """-> ('start', member, run) | ('in', member, delta, run) | None; run = bytes up to the next pointer member (the contiguous extent that is the same on x86-64)"""
    nxt = [f.off for f in cls.fields if f.kind != 'align8' and f.off > lit and ('ptr' in f.kind or f.kind in ('vptr', 'u32-ptrslot'))]
    run = (min(nxt) if nxt else cls.size) - lit
    if lit in mem:
        return ('start', mem[lit][0], run)   # a pointer member start is allowed (a PTR accessor); an integer accessor on it is a hazard (report below)
    for f in cls.fields:
        if f.kind == 'align8' or 'ptr' in f.kind or f.kind == 'vptr':
            continue
        n = lg.KINDS[f.kind][2] * (f.count or 1)
        if f.off < lit < f.off + n:
            return ('in', f.name, lit - f.off, run)
    return None


def bytes_rewrite(t, mask, sites, cls, mem, header_inc, deny=None, funcs=None, B=None, only=None, Bj=None):
    """replace the LITERAL of OBJ_BYTES(obj, LIT [+ rest]) by the native offset constant (V34OFF(0x..) = offsetof of the member, V34OFFI(member, delta) inside a region):
    the expression keeps its shape, so the i386 code is identical, and on x86-64 the compiler computes the offset"""
    out, pos, n, rep, left = [], 0, 0, [], []
    for (s, e, base, val, txt, rest) in sites:
        if deny and funcs is not None and func_at(funcs, s) in deny:
            continue
        if only is not None and funcs is not None and func_at(funcs, s) not in only:
            continue
        b = bytes_binding(cls, mem, val + (B or 0)) if val is not None else None
        if b is None:
            left.append((txt, rest))
            continue
        mm = re.compile(r'\(\s*%s\s*,\s*' % re.escape(base)).search(t, s)
        i = mm.end()   # the literal inside this NAME(base, LIT ...)
        assert t.startswith(txt, i), (t[i:i + 20], txt)
        out.append(t[pos:i])
        if B is None:
            out.append('V34OFF(0x%x)' % val if b[0] == 'start' else 'V34OFFI(%s, %d)' % (b[1], b[2]))
        else:   # offset relative to the sub-object base member (flat object, native distances)
            if Bj is None:
                out.append('V34REL(0x%x, 0x%x)' % (B, val + B) if b[0] == 'start' else 'V34RELI(0x%x, %s, %d)' % (B, b[1], b[2]))
            else:   # the base itself lies inside a member (region Bj[0] + Bj[1])
                out.append('V34RELJ(%s, %d, 0x%x)' % (Bj[0], Bj[1], val + B) if b[0] == 'start' else 'V34RELIJ(%s, %d, %s, %d)' % (Bj[0], Bj[1], b[1], b[2]))
        pos = i + len(txt)
        rep.append((val, rest, b[-1]))
        n += 1
    out.append(t[pos:])
    t2 = ''.join(out)
    if not n:
        return t2, 0, rep, left
    m2 = mask_text(t2)
    inc = '#include "%s"\n' % header_inc
    has_inc = ('#include "%s"' % header_inc) in t2
    if B is not None and Bj is not None and '#define V34RELJ(' not in t2:
        defs = ('/* T04: offsets relative to a sub-object base that lies INSIDE a member of the flat typed object (tools/tier_rewrite.py bytes --sub): native distances */\n'
                '#define V34RELJ(bmem, bd, off) ((int)(offsetof(V34Obj, V34M_##off) - offsetof(V34Obj, bmem)) - (bd))\n'
                '#define V34RELIJ(bmem, bd, mem, d) ((int)(offsetof(V34Obj, mem) - offsetof(V34Obj, bmem)) - (bd) + (d))\n')
        m = re.search(r'^#define OBJ_BYTES\(.*$', m2, re.M)
        ins = m.end() + 1
        if has_inc:
            mi = re.search(r'^#include "%s".*$' % re.escape(header_inc), t2, re.M)
            ins = max(ins, mi.end() + 1)
            t2 = t2[:ins] + defs + t2[ins:]
        else:
            t2 = t2[:ins] + inc + defs + t2[ins:]
        has_inc = True
    if B is not None and Bj is None and '#define V34REL(' not in t2:
        defs = ('/* T04: offsets relative to the sub-object base member 0x%x of the FLAT typed object (tools/tier_rewrite.py bytes --sub): the native distance, = the i386 offset at -m32 */\n'
                '#define V34REL(base, off) ((int)(offsetof(V34Obj, V34M_##off) - offsetof(V34Obj, V34M_##base)))\n'
                '#define V34RELI(base, mem, d) ((int)(offsetof(V34Obj, mem) - offsetof(V34Obj, V34M_##base)) + (d))\n' % B)
        m = re.search(r'^#define OBJ_BYTES\(.*$', m2, re.M)
        ins = m.end() + 1
        if has_inc:
            mi = re.search(r'^#include "%s".*$' % re.escape(header_inc), t2, re.M)
            ins = max(ins, mi.end() + 1)
            t2 = t2[:ins] + defs + t2[ins:]
        else:
            t2 = t2[:ins] + inc + defs + t2[ins:]
        has_inc = True
    if B is None and '#define V34OFF(' not in t2:
        defs = ('/* T04: native offsets of the typed object (tools/tier_rewrite.py bytes): OBJ_BYTES(obj, V34OFF(0xa48) + i) keeps the expression of the original OBJ_BYTES(obj, 0xa48 + i) */\n'
                '#define V34OFF(off) ((int)offsetof(V34Obj, V34M_##off))\n'
                '#define V34OFFI(mem, d) ((int)offsetof(V34Obj, mem) + (d))\n')
        m = re.search(r'^#define OBJ_BYTES\(.*$', m2, re.M)
        ins = m.end() + 1
        if has_inc:
            mi = re.search(r'^#include "%s".*$' % re.escape(header_inc), t2, re.M)
            ins = max(ins, mi.end() + 1)
            t2 = t2[:ins] + defs + t2[ins:]
        else:
            t2 = t2[:ins] + inc + defs + t2[ins:]
    return t2, n, rep, left


SOLE = set()   # keys of layouts/v34/v34sub.txt whose sub-object struct is the ONLY definition of its members in V34Obj (directive `sole <key> ...`); filled by read_sub_spec


def read_sub_spec(path):
    """[(key, struct name, [bases], size, comment)] from layouts/v34/v34sub.txt:  <key> <Struct> <size> <base> [<base2> ...]   # comment
    A line `sole <key> [<key> ...]` makes those sub-objects SOLE: V34Obj holds a named member `V34X key;` (key + index for several instances) instead of the
    flat members (an anonymous union with the struct), the keys V34M_0x.. name the member path (`det.p3564_`) and the struct takes the C++ member types of
    layouts/v34/v34obj_cxx.txt.  A sole sub-object nested in another (blkb in rx, pf in mod) needs the outer one to be sole too."""
    out = []
    SOLE.clear()
    for ln, raw in enumerate(open(path).read().split('\n'), 1):
        body, _, cmt = raw.partition('#')
        t = body.split()
        if not t:
            continue
        if t[0] == 'sole':
            SOLE.update(t[1:])
            continue
        if len(t) < 4:
            sys.exit('%s:%d: expected <key> <Struct> <size> <base> [<base> ...]' % (path, ln))
        out.append((t[0], t[1], [int(x, 0) for x in t[3:]], int(t[2], 0), cmt.strip()))
    return out


def make_slice(c, name, base, size):
    """the sub-object class: the members of class c inside [base, base+size), offsets relative; an unknown-region that crosses the interval is trimmed"""
    sc = lg.Cls(name, size, name, name, 0)
    for f in c.fields:
        if f.kind == 'align8':
            if base < f.off < base + size:   # T05: interior pointer-alignment markers (a nested sub-object's tail padding) belong to the slice
                sc.fields.append(lg.Fld(f.off - base, 'align8', f.name, 0, 0))
            continue
        n = lg.KINDS[f.kind][2] * (f.count or 1)
        lo, hi = max(f.off, base), min(f.off + n, base + size)
        if hi <= lo:
            continue
        if (lo, hi) != (f.off, f.off + n):
            if f.kind != 'unknown-region':
                sys.exit('slice %s [0x%x, 0x%x): member %s (0x%x..0x%x) crosses the boundary' % (name, base, base + size, f.name, f.off, f.off + n))
            sc.fields.append(lg.Fld(lo - base, 'unknown-region', f.name, hi - lo, 0))
        else:
            sc.fields.append(lg.Fld(f.off - base, f.kind, f.name, f.count, 0))
    return sc


def slice_shapes_equal(a, b):
    return [(f.off, f.kind, f.count) for f in a.fields] == [(f.off, f.kind, f.count) for f in b.fields]


def slices_from(c, spec):
    """-> [(key, struct, slice class, [other instances' classes], comment)]"""
    out = []
    for key, name, bases, size, cmt in spec:
        sc = make_slice(c, name, bases[0], size)
        others = [(b, slice_shapes_equal(sc, make_slice(c, name, b, size))) for b in bases[1:]]
        out.append((key, name, bases, sc, others, cmt))
    return out


def gen_paths(c, spec, layout_rel, spec_rel):
    """layouts/v34/v34obj_paths.txt: `<member> <path>` for every member (and C++ marker) that lives inside a sole sub-object struct, for tools/layout_gen.py --paths"""
    embeds = embed_list(c, spec)
    sole = sole_list(embeds)
    P = field_paths(c.fields, embeds)
    cxx = CxxSpec(os.path.join(ROOT, CXX_PATH_FOR_CMD))
    for (mn, mt, mo) in cxx.names:
        chain = sorted([e for e in sole if e[3] < mo < e[3] + e[4]], key=lambda e: -e[4])
        if chain and all(e[3] == next(x[3] for x in sole if x[1] == e[1]) for e in chain):
            P[mn] = '.'.join([e[0] for e in chain] + [mn])
    P.update(cxx.pathnames)
    o = ['# GENERATED by `python3 tools/tier_rewrite.py paths` from %s + %s: do not edit.' % (layout_rel, spec_rel),
         '# The members of the V.34 module object that live inside a SOLE sub-object struct (`sole` in the spec): name, member path.  Read by tools/layout_gen.py --paths',
         '# (the offsetof asserts and the probe of the C++ class name the member by its path).']
    order = {f.name: f.off for f in c.fields}
    o += ['%s %s' % (k, v) for k, v in sorted(P.items(), key=lambda kv: (order.get(kv[0], 1 << 30), kv[0]))]
    return '\n'.join(o) + '\n'


def gen_sub_header(c, spec, layout_rel, spec_rel):
    global CXX
    sl = slices_from(c, spec)
    embeds = embed_list(c, spec)
    sole = sole_list(embeds)
    cxx = CxxSpec(os.path.join(ROOT, CXX_PATH_FOR_CMD)) if SOLE else None
    nested_sole = lambda s_: any(e[1] != s_[0] and e[3] <= s_[2][0] and s_[2][0] + s_[3].size <= e[3] + e[4] for e in sole)
    sl.sort(key=lambda s_: 0 if (s_[0] in SOLE and nested_sole(s_)) else 1)   # a sole struct nested in another is defined first
    l64 = layout64(c)
    flat = {f.name: f for f in c.fields if f.kind != 'align8'}
    o = ['/**',
         ' * @file v34_subobjects.h',
         ' * @brief The sub-objects of the V.34 module object (receive front end, modulator, transmit block, echo, tone detector, rate, INFO records) as C structs.',
         ' *',
         ' * GENERATED by tools/tier_rewrite.py slice from %s + %s: do not edit.  Regenerate with' % (layout_rel, spec_rel),
         ' *   python3 tools/tier_rewrite.py slice -o src/v34/v34_subobjects.h',
         ' *',
         ' * The V.34 functions reach these parts of the module object through a derived base pointer (rx = obj + 0x264, mod = obj + 0x1450, ...).',
         ' * Each struct is the slice [base, base + size) of the V34Obj mirror (src/v34/v34_object_view.h) with REAL pointer members, offsets relative to the base;',
         ' * the sizes are the distance to the next known sub-object (an upper bound where the real class size is unknown).  There is one',
         ' * `#define V34<KEY>_0x130 <member>` per member start for the typed accessors, and one descriptive key per named member.',
         ' *',
         ' * V34Obj EMBEDS these structs.  A sole one (directive `sole` of the spec) is the only definition of its members, typed as the C++ ports use them',
         ' * (layouts/v34/v34obj_cxx.txt); the others are anonymous unions over the flat members of V34Obj, whose _Static_asserts check the agreement member by member',
         ' * (the structs agree with the flat object on 32-bit x86, and on x86-64 when no pointer member before the member shifts the alignment differently). */',
         '#ifndef V34SUB_H', '#define V34SUB_H', '', '#include <stddef.h>', '#include <stdint.h>'] + ASSERT_MACRO + ['#include "%s"' % h for h in (cxx.includes if cxx else [])] + ['']
    rep = []
    for key, name, bases, sc, others, cmt in sl:
        # x86-64 agreement of the slice with the flat object
        sl64 = layout64(sc)
        b0 = next((f for f in c.fields if f.off == bases[0] and f.kind != 'align8'), None)
        agree = bad = 0
        if b0 is not None:
            for f in sc.fields:
                if f.name in flat and f.name in sl64:
                    if sl64[f.name][0] == l64[f.name][0] - l64[b0.name][0]:
                        agree += 1
                    else:
                        bad += 1
        note = ' - %s; instances at %s%s' % (cmt, ', '.join('0x%x' % b for b in bases), ''.join('; the instance at 0x%x has %s shape' % (b, 'the same' if eq else 'a DIFFERENT') for b, eq in others))
        CXX = cxx if key in SOLE else None
        nested = [(e[0], e[1], e[2], e[3] - bases[0], e[4]) for e in sole if e[1] != key and bases[0] <= e[3] and e[3] + e[4] <= bases[0] + sc.size]
        if CXX:
            MARK_DONE.clear()
        o += gen_block(sc, 'V34%s_' % key.upper(), name, note, embeds=nested or None, keys=True, delta=bases[0] if key in SOLE else None)
        if CXX:   # a marker name inside the struct that its sub-object could not take (the second instance of a struct: the name exists once) is reported
            for (mn, mt, mo) in cxx.names:
                if bases[0] < mo < bases[0] + sc.size and mn not in MARK_DONE and not any(e[3] < mo < e[3] + e[4] and e[1] != key for e in sole):
                    sys.exit('slice %s: the C++ marker %s (0x%x) was not emitted' % (name, mn, mo))
        CXX = None
        rep.append('%s (key %s, base 0x%x, size 0x%x): %d members, x86-64 agreement with the flat V34Obj: %d same relative offset / %d shifted%s' % (
            name, key, bases[0], sc.size, len([f for f in sc.fields]), agree, bad, '' if all(eq for _, eq in others) else ' ; other instances differ in shape'))
    o.append('#endif /* V34SUB_H */')
    return '\n'.join(o) + '\n', rep


# --------------------------------------------------------------------------------------------- source analysis
ACC_DEF = re.compile(r'^[ \t]*#[ \t]*define[ \t]+([A-Za-z_]\w*)\(\s*(\w+)\s*,\s*(\w+)\s*\)[ \t]+'
                     r'\(\s*\*\s*\(\s*([^()]*?)\s*\*\s*\)\s*(?:[A-Za-z_]\w*\(\s*(\w+)\s*,\s*(\w+)\s*\)|'
                     r'\(\s*\(\s*(?:unsigned\s+char|uint8_t|char)\s*\*\s*\)\s*\(?\s*(\w+)\s*\)?\s*\+\s*\(?\s*(\w+)\s*\)?\s*\))\s*\)',
                     re.M)


def mask_text(t):
    """same length as t, comments and string / char literals replaced by spaces (newlines kept): matches inside them are ignored"""
    out, i, n = [], 0, len(t)
    while i < n:
        c = t[i]
        if t.startswith('/*', i):
            j = t.find('*/', i + 2)
            j = n if j < 0 else j + 2
            out.append(''.join(ch if ch == '\n' else ' ' for ch in t[i:j]))
            i = j
        elif t.startswith('//', i):
            j = t.find('\n', i)
            j = n if j < 0 else j
            out.append(' ' * (j - i))
            i = j
        elif c in '"\'':
            j = i + 1
            while j < n and t[j] != c:
                j += 2 if t[j] == '\\' else 1
            j = min(j + 1, n)
            out.append(''.join(ch if ch == '\n' else ' ' for ch in t[i:j]))
            i = j
        else:
            out.append(c)
            i += 1
    return ''.join(out)


def accessors(t, mask):
    """{macro name: C type} from the file's own definitions of the form  #define NAME(o, off) (*(TYPE *)OBJ_BYTES(o, off))"""
    acc = {}
    for m in ACC_DEF.finditer(mask):
        name, p1, p2, ty = m.group(1), m.group(2), m.group(3), m.group(4)
        if m.group(5):
            if (m.group(5), m.group(6)) != (p1, p2):
                continue
        elif (m.group(7), m.group(8)) != (p1, p2):
            continue
        # take the type text from the real source (the mask only blanks comments, the type has none)
        acc[name] = ' '.join(t[m.start(4):m.end(4)].split())
    return acc


FUNC_DEF = re.compile(r'^[A-Za-z_][^;{}()#\n]*?\b(\w+)\s*\([^;{}]*\)\s*\{', re.M)


def func_starts(mask):
    """[(position, name)] of the top-level function definitions of the (masked) source"""
    return [(m.start(), m.group(1)) for m in FUNC_DEF.finditer(mask)]


def func_at(funcs, pos):
    name = '?'
    for (a, n) in funcs:
        if a <= pos:
            name = n
    return name


def load_deny(path, fname):
    """{function names} the file `fname` lists in the exceptions file (lines: `<file> <func> [<func> ...]  # reason`)"""
    deny = set()
    if path and os.path.exists(path):
        for ln in open(path):
            t = ln.split('#', 1)[0].split()
            if len(t) >= 2 and t[0] == fname:
                deny.update(t[1:])
    return deny


def find_sites(t, mask, accs, bases):
    """[(start, end, macro, base, offtext, offvalue or None)]  for every NAME(base, <literal>) outside comments / strings"""
    names = '|'.join(sorted(map(re.escape, accs), key=len, reverse=True))
    bs = '|'.join(map(re.escape, bases))
    rx = re.compile(r'(?<![\w.>])(%s)\(\s*(%s)\s*,\s*(0[xX][0-9a-fA-F]+|[0-9]+)[uUlL]*\s*\)' % (names, bs))
    sites = []
    for m in rx.finditer(mask):
        # skip the macro's own #define line
        ls = mask.rfind('\n', 0, m.start()) + 1
        if re.match(r'\s*#\s*define', mask[ls:m.start()]):
            continue
        txt = m.group(3)
        sites.append((m.start(), m.end(), m.group(1), m.group(2), txt, int(txt, 0)))
    return sites


def analyse(path, mem, bases, deny_file=None):
    """-> text, mask, accessors, sites (sites inside a function listed for this file in the exceptions file are dropped)"""
    t = open(path).read()
    mask = mask_text(t)
    accs = accessors(t, mask)
    sites = find_sites(t, mask, accs, bases) if accs else []
    deny = load_deny(deny_file, os.path.basename(path))
    analyse.denied = 0
    if deny and sites:
        fs = func_starts(mask)
        keep = []
        for st in sites:
            if func_at(fs, st[0]) in deny:
                analyse.denied += 1
            else:
                keep.append(st)
        sites = keep
    return t, mask, accs, sites


def resolve(cls, mem, mac, accs, val):
    """how a site's offset binds: ('start', None) | ('interior', (member, delta)) | None (non-member: opaque region interior / outside)"""
    if val in mem:
        return 'start', None
    it = interior(cls, val) if cls is not None else None
    if it:
        f = next(x for x in cls.fields if x.name == it[0])
        w, isptr = acc_width(accs[mac])
        ext = lg.KINDS[f.kind][2] * (f.count or 1) - it[1]
        if w is not None and not isptr and w <= ext:
            return 'interior', it
    return None


def rewrite_text(t, mask, accs, sites, mem, mode, header_inc, cls=None, form='member', tgt=None):
    tgt = tgt or TGT_OBJ
    out, pos, used, usedi, nrew = [], 0, set(), set(), 0
    for (s, e, mac, base, txt, val, *_) in sites:
        r = resolve(cls, mem, mac, accs, val)
        if r is None and mode == 'partial':
            continue
        out.append(t[pos:s])
        if r is not None and r[0] == 'interior':
            out.append('%s%s(%s, 0x%x, %s, %d)' % (mac, tgt['ti'], base, val, r[1][0], r[1][1]))
            usedi.add(mac)
        else:
            out.append('%s%s(%s, 0x%x)' % (mac, tgt['t'], base, val))
            used.add(mac)
        pos = e
        nrew += 1
    out.append(t[pos:])
    t2 = ''.join(out)
    if not used and not usedi:
        return t2, 0
    # inject the include + the typed macros after the last accessor #define (the macro lines themselves are never rewritten)
    mask2 = mask_text(t2)
    last = 0
    for m in ACC_DEF.finditer(mask2):
        if m.group(1) in accs:
            last = max(last, m.end())
    eol = mask2.find('\n', last)
    eol = len(t2) - 1 if eol < 0 else eol
    inc = '' if ('#include "%s"' % header_inc) in t2 else '#include "%s"\n' % header_inc
    T, TI, ct, pf = tgt['t'], tgt['ti'], tgt['ctype'], tgt['prefix']
    if form == 'member':
        fmt = '#define %s%s(o, off) (*(%s *)&((' + ct + ' *)(o))->' + pf + '##off)\n'
    else:   # an exact-type accessor (plain int32_t on an int32_t member) is folded by gcc -O0 into one addressing mode; this spelling keeps the original instruction selection
        fmt = '#define %s%s(o, off) (*(%s *)((unsigned char *)(o) + offsetof(' + ct + ', ' + pf + '##off)))\n'
    defs = ''.join(fmt % (m, T, accs[m]) for m in sorted(used) if ('#define %s%s(' % (m, T)) not in t2)
    defs += ''.join('#define %s%s(o, off, mem, d) (*(%s *)((unsigned char *)(o) + offsetof(' % (m, TI, accs[m]) + ct + ', mem) + (d)))\n'
                    for m in sorted(usedi) if ('#define %s%s(' % (m, TI)) not in t2)
    block = ('/* T01/T02: typed-member accessors (tools/tier_rewrite.py): the compiler computes the offset of member %s<off> of %s (%s); %s = a byte / word INSIDE a member (offsetof(member) + delta) */\n%s%s' % (
        pf, ct, tgt['header'], TI, inc, defs))
    t2 = t2[:eol + 1] + block + t2[eol + 1:]
    return t2, nrew


def worklist(sites, mem, accs=None, cls=None):
    w = {}
    for (_, _, mac, base, txt, val, *_) in sites:
        if resolve(cls, mem, mac, accs, val) is None:
            w.setdefault(val, 0)
            w[val] += 1
    return dict(sorted(w.items()))


def hazards(sites, accs, cls, mem):
    """x86-64 hazards of the RESOLVED sites: a pointer accessor on a non-pointer member, an integer access to a pointer slot, an access that spans
    members which are not contiguous on x86-64 (padding between them).  -> [(offset, accessor, text)]"""
    l64 = layout64(cls)
    fl = [f for f in cls.fields if f.kind != 'align8']
    out = []
    for (_, _, mac, base, txt, val, *_) in sites:
        r = resolve(cls, mem, mac, accs, val)
        if r is None:
            continue
        w, isptr = acc_width(accs[mac])
        if w is None:
            continue
        cov = [f for f in fl if f.off < val + w and val < f.off + lg.KINDS[f.kind][2] * (f.count or 1)]
        if not cov:
            continue
        f0 = cov[0]
        slot = 'ptr' in f0.kind and not f0.count
        if isptr and not (slot and f0.off == val):
            out.append((val, mac, 'pointer accessor on %s %s' % (f0.kind, f0.name)))
        elif slot and not isptr:
            out.append((val, mac, 'integer accessor on pointer slot %s (8 bytes on x86-64)' % f0.name))
        elif not isptr:
            for a, b in zip(cov, cov[1:]):
                oa, sa = l64[a.name]
                if oa + sa != l64[b.name][0]:
                    out.append((val, mac, 'access %d B at %s spans %s / %s with padding on x86-64' % (w, f0.name, a.name, b.name)))
                    break
    return out


# --------------------------------------------------------------------------------------------- identity check
def compile_dump(src, workdir, name, opt, incs, extra):
    obj = os.path.join(workdir, name + re.sub(r'\W', '_', opt) + '.o')
    cmd = ['gcc', '-m32'] + opt.split() + ['-fno-pie', '-fcf-protection=none', '-fno-stack-protector', '-c', src, '-o', obj] + \
          ['-I' + i for i in incs] + extra
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode:
        return None, r.stderr
    d = subprocess.run(['objdump', '-dr', '--no-show-raw-insn', obj], capture_output=True, text=True)
    # the first line names the object file: drop it
    return '\n'.join(d.stdout.split('\n')[2:]), ''


def check_identity(new, orig_text, incs, opts, extra, quiet=False):
    ok = True
    with tempfile.TemporaryDirectory(prefix='tierrw_') as td:
        base = os.path.basename(new)
        od = os.path.join(td, 'orig')
        nd = os.path.join(td, 'new')
        os.mkdir(od)
        os.mkdir(nd)
        op, np_ = os.path.join(od, base), os.path.join(nd, base)
        open(op, 'w').write(orig_text)
        open(np_, 'w').write(open(new).read())
        # relative includes of the file (#include "x.h" next to the source) resolve through -I, same for both
        src_dir = os.path.dirname(os.path.abspath(new))
        inc = list(incs) + [src_dir]
        for opt in opts:
            a, ea = compile_dump(op, od, 'o', opt, inc, extra)
            b, eb = compile_dump(np_, nd, 'n', opt, inc, extra)
            if a is None or b is None:
                ok = False
                print('%s %s: COMPILE FAILED (%s)' % (base, opt, 'orig' if a is None else 'new'))
                print((ea or eb)[:1500])
            elif a != b:
                ok = False
                print('%s %s: DIFFERENT (%d / %d lines)' % (base, opt, a.count('\n'), b.count('\n')))
                for ln in list(difflib.unified_diff(a.split('\n'), b.split('\n'), 'orig', 'new', lineterm='', n=1))[:30]:
                    print('   ' + ln)
            else:
                if not quiet:
                    print('%s %s: IDENTICAL (%d lines of objdump -dr)' % (base, opt, a.count('\n')))
    return ok


def head_text(path):
    rel = os.path.relpath(os.path.abspath(path), ROOT)
    r = subprocess.run(['git', '-C', ROOT, 'show', 'HEAD:./' + rel], capture_output=True, text=True)
    if r.returncode:
        sys.exit('git show HEAD:%s failed: %s' % (rel, r.stderr.strip()))
    return r.stdout


# --------------------------------------------------------------------------------------------- main
# --------------------------------------------------------------------------------------------- embed (T05): V34REL sites -> typed member access of the embedded sub-object structs
REL_CALL = re.compile(r'(?<![\w.>])(V34REL|V34RELI|V34RELJ|V34RELIJ)\(')


def split_args(txt):
    out, depth, cur = [], 0, ''
    for ch in txt:
        if ch in '([':
            depth += 1
        elif ch in ')]':
            depth -= 1
        if ch == ',' and depth == 0:
            out.append(cur.strip())
            cur = ''
        else:
            cur += ch
    out.append(cur.strip())
    return out


def embed_rewrite(t, mask, flat, accs, spec, form='member', only_funcs=None):
    """-> (new text, stats).  Every V34REL(B, ABS) / V34RELI(B, mem, d) / V34RELJ / V34RELIJ call (native distance inside the FLAT object) whose base B is an embedded sub-object instance
    (embed_list) becomes
      ACC(base, V34REL(B, ABS))   ->  ACC<T>_key(base, 0xREL)       typed member access  (*(T *)&((V34Rx *)(base))->V34RX_##REL)
      V34REL(B, ABS)  (other)     ->  (int)offsetof(V34Rx, member)                      (a computed offset: OBJ_BYTES(blk, ... + 2 * i))
      V34RELI(B, mem, d)          ->  ((int)offsetof(V34Rx, mem') + (d))
    REL = ABS - B; the member name is that of the FIRST instance of the sub-object (the other instances have the same shape).  Calls whose base is not an embedded instance
    (the other V34Info records, the probe record inside an unknown region) are left alone and counted."""
    embeds = flat['embeds']
    by_base = {}
    for (name, key, st, b, size) in embeds:
        by_base[b] = (key, st, size)
    first = {}
    for (name, key, st, b, size) in embeds:
        first.setdefault(key, b)
    mem_off = flat['mem_off']          # flat member name -> i386 offset
    off_mem = flat['off_mem']          # i386 offset -> flat member name (member starts)
    slice_by_rel = {}                  # key -> {rel offset: member name}
    for (name, key, st, b, size) in embeds:
        if first[key] == b:
            slice_by_rel[key] = {o - b: off_mem[o] for o in off_mem if b <= o < b + size}
    fs = func_starts(mask)
    edits, used, stats = [], set(), dict(typed=0, offsetof=0, left=0, nested=0)
    pos = 0
    for m in REL_CALL.finditer(mask):
        ls = mask.rfind('\n', 0, m.start()) + 1
        if re.match(r'\s*#\s*define\s+$', mask[ls:m.start()]):   # the helper macros' own definitions; other #define lines (SMP(b, k), REC(k)) are rewritten
            continue
        if only_funcs is not None and func_at(fs, m.start()) not in only_funcs and not re.match(r'\s*#\s*define', mask[ls:m.start()]):
            continue
        i, depth = m.end(), 1
        while i < len(mask) and depth:
            depth += {'(': 1, ')': -1}.get(mask[i], 0)
            i += 1
        a = [x.split('.')[-1] if re.fullmatch(r'[A-Za-z_]\w*(\.[A-Za-z_]\w*)+', x) else x for x in split_args(t[m.end():i - 1])]   # V34SUB: a member path (rx.unk04cc_) names its last member
        kind = m.group(1)
        try:
            if kind == 'V34REL':
                B, absoff, mem, d = int(a[0], 0), int(a[1], 0), None, '0'
            elif kind == 'V34RELI':
                B, absoff, mem, d = int(a[0], 0), None, a[1], a[2]
            elif kind == 'V34RELJ':
                B, absoff, mem, d = mem_off[a[0]] + int(a[1], 0), int(a[2], 0), None, '0'
            else:
                B, absoff, mem, d = mem_off[a[0]] + int(a[1], 0), None, a[2], a[3]
        except (ValueError, KeyError):
            stats['left'] += 1
            continue
        if B not in by_base:
            stats['left'] += 1
            continue
        key, st, size = by_base[B]
        tbl = slice_by_rel[key]
        cover = None   # (member, delta) when the target lies inside a slice member (an unknown-region): offsetof(member) + delta
        if absoff is not None:
            rel = absoff - B
        else:
            if mem not in mem_off:
                stats['left'] += 1
                continue
            rel = mem_off[mem] - B
            if re.fullmatch(r'-?(0[xX][0-9a-fA-F]+|\d+)', d):   # a literal displacement: resolve it here
                rel += int(d, 0)
                absoff, mem, d = B + rel, None, '0'
        if rel not in tbl and (absoff is not None or mem is None):
            hit = [(o, n, z) for (o, n, z) in flat['fields'] if o <= B + rel < o + z and B <= o < B + size]
            if not hit:
                stats['left'] += 1
                continue
            o0, n0, z0 = hit[0]
            cover = (tbl[o0 - B], B + rel - o0)
            kind = 'V34RELJ'   # not a plain member start: no typed accessor
        elif rel not in tbl:
            stats['left'] += 1
            continue
        memname = tbl[rel] if rel in tbl else None
        # an enclosing  ACC(base, <this>)  with the file's own accessor: typed member access
        if cover is not None:
            edits.append((m.start(), i, '((int)offsetof(%s, %s) + %d)' % (st, cover[0], cover[1]) if cover[1] else '(int)offsetof(%s, %s)' % (st, cover[0])))
            stats['offsetof'] += 1
            continue
        pre = re.search(r'(?<![\w.>])([A-Za-z_]\w*)\(\s*([A-Za-z_]\w*)\s*,\s*$', mask[:m.start()])
        if kind == 'V34REL' and pre and pre.group(1) in accs and mask[i:i + 1] == ')' and form in ('member', 'offsetof'):
            edits.append((pre.start(), i + 1, '%sT_%s(%s, 0x%x)' % (pre.group(1), key, pre.group(2), rel)))
            used.add((pre.group(1), key))
            stats['typed'] += 1
        elif kind in ('V34REL', 'V34RELJ'):
            edits.append((m.start(), i, '(int)offsetof(%s, %s)' % (st, memname)))
            stats['offsetof'] += 1
        else:
            edits.append((m.start(), i, '((int)offsetof(%s, %s) + (%s))' % (st, memname, d)))
            stats['offsetof'] += 1
    # apply (edits are in order and do not overlap: an enclosing ACC( ... ) never contains another REL call)
    out, pos = [], 0
    for (s0, e0, new) in edits:
        if s0 < pos:
            sys.exit('overlapping edits at %d' % s0)
        out.append(t[pos:s0])
        out.append(new)
        pos = e0
    out.append(t[pos:])
    t2 = ''.join(out)
    if not edits:
        return t2, stats
    mask2 = mask_text(t2)
    # macros after the last V34REL / accessor #define
    defs = ''
    for (acc, key) in sorted(used):
        stn = next(st for (_, k, st, _, _) in embeds if k == key)
        if form == 'member':
            defs += '#define %sT_%s(o, off) (*(%s *)&((%s *)(o))->V34%s_##off)\n' % (acc, key, accs[acc], stn, key.upper())
        else:   # an exact-type accessor is folded by gcc -O0 into one addressing mode; this spelling keeps the original instruction selection
            defs += '#define %sT_%s(o, off) (*(%s *)((unsigned char *)(o) + offsetof(%s, V34%s_##off)))\n' % (acc, key, accs[acc], stn, key.upper())
    anchor = 0
    for mm in re.finditer(r'^#define (?:V34REL\w*|V34OFF\w*|OBJ_BYTES)\(.*$', mask2, re.M):
        anchor = max(anchor, mm.end())
    if '#include "v34_subobjects.h"' not in t2 and '#include "v34_object_view.h"' not in t2:
        defs = '#include "v34_object_view.h"\n' + defs
    block = '/* T05: typed member access of the sub-object structs EMBEDDED in V34Obj (src/v34/v34_subobjects.h; tools/tier_rewrite.py embed): the compiler computes the offset of member V34<KEY>_<rel> */\n' + defs if defs else ''
    t2 = t2[:anchor + 1] + block + t2[anchor + 1:]
    # drop the V34REL helper macros that no use is left
    for mac in ('V34RELIJ', 'V34RELJ', 'V34RELI', 'V34REL'):
        body = re.sub(r'^(?:/\*[^\n]*\*/\n)?#define %s\(.*$\n' % mac, '', t2, flags=re.M)
        if not re.search(r'(?<![\w.>])%s\(' % mac, mask_text(body)):
            t2 = body
    return t2, stats


def embed_bases(t, mask, flat):
    """T05: OBJ_BYTES(X, V34OFF(0xB)) with 0xB the base of an embedded sub-object  ->  V34SUBP(X, member)  =  (unsigned char *)&((V34Obj *)(X))->member (the sub-object is a member now)"""
    by_base = {b: name for (name, key, st, b, size) in flat['embeds']}
    n = [0]

    def rep(m):
        b = int(m.group(2), 16)
        if b not in by_base:
            return m.group(0)
        n[0] += 1
        return 'V34SUBP(%s, %s)' % (m.group(1), by_base[b])
    out = []
    pos = 0
    for m in re.finditer(r'OBJ_BYTES\(\s*(\w+)\s*,\s*V34OFF\((0x[0-9a-fA-F]+)\)\s*\)', mask):
        r = rep(m)
        out.append(t[pos:m.start()])
        out.append(r)
        pos = m.end()
    out.append(t[pos:])
    t2 = ''.join(out)
    if n[0] and '#define V34SUBP(' not in t2:
        m2 = mask_text(t2)
        a = max(mm.end() for mm in re.finditer(r'^#define V34OFF\w*\(.*$', m2, re.M))
        t2 = t2[:a + 1] + '/* T05: the address of an embedded sub-object of the typed object */\n#define V34SUBP(o, mem) ((unsigned char *)&((V34Obj *)(o))->mem)\n' + t2[a + 1:]
    return t2, n[0]


def embed_context(args, tgtlayout):
    c = load_class(tgtlayout, 'tagV34Object')
    emb = embed_list(c, read_sub_spec(DEFAULT_SUB))
    flat = dict(embeds=emb, mem_off={}, off_mem={})
    for f in c.fields:
        if f.kind == 'align8':
            continue
        flat['mem_off'][f.name] = f.off
        flat['off_mem'].setdefault(f.off, f.name)
    flat['fields'] = [(f.off, f.name, lg.KINDS[f.kind][2] * (f.count or 1)) for f in c.fields if f.kind != 'align8']
    return flat



def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)

    def common(p):
        p.add_argument('--layout', default=DEFAULT_LAYOUT)
        p.add_argument('--class', dest='cls', default='tagV34Object')
        p.add_argument('--base', action='append', help='base token of the sites to rewrite (default: obj)')
        p.add_argument('--header', default='v34_object_view.h', help='name used in #include (default v34_object_view.h)')
        p.add_argument('--deny-file', default=DEFAULT_DENY, help='per-file function exceptions (functions whose `obj` is not the V34 object); default layouts/v34/v34_base_exceptions.txt')
        p.add_argument('--struct', default='obj', help='rewrite the sites of another base through a sub-object struct: obj (default, V34Obj), a key of layouts/v34/v34sub.txt (rx mod tx echo det rate info probe blkb), ctx (VPcmFloModem), P (dp parameter block), k56')
        p.add_argument('--hazards', action='store_true', help='list the x86-64 hazards of the resolved sites (pointer / integer slot mix-ups, accesses spanning padding)')

    h = sub.add_parser('header')
    common(h)
    h.add_argument('-o', default=None)
    h.add_argument('--check', action='store_true')
    h.add_argument('--prefix', default=PREFIX)
    h.add_argument('--guard', default='V34OBJ_H')
    h.add_argument('--cxx', default=None, help='(RS5) layouts/v34/v34obj_cxx.txt: the C++-facing additions (member types, marker names, record overlays) of the one definition of the object')
    h.add_argument('--tag', default=None, help='(RS5) the struct tag of the typedef: `typedef struct TAG {...} Mirror;` (the C++ class name when this struct is the one definition of the object)')
    h.add_argument('--no-embed', action='store_true', help='(T05) do not embed the sub-object structs of src/v34/v34_subobjects.h in the V34Obj mirror')
    r = sub.add_parser('rewrite')
    common(r)
    r.add_argument('files', nargs='+')
    r.add_argument('-o')
    r.add_argument('--in-place', action='store_true')
    r.add_argument('--mode', choices=['partial', 'all'], default='partial')
    w = sub.add_parser('worklist')
    common(w)
    w.add_argument('files', nargs='+')
    w.add_argument('--json', action='store_true')
    c = sub.add_parser('check')
    c.add_argument('new')
    c.add_argument('--orig')
    c.add_argument('-I', dest='incs', action='append', default=[])
    c.add_argument('--opt', nargs='+', default=['-O0', '-O2', '-O2 -fno-strict-aliasing'])
    c.add_argument('--cflags', nargs=argparse.REMAINDER, default=[])
    sl = sub.add_parser('slice')
    sl.add_argument('--layout', default=DEFAULT_LAYOUT)
    sl.add_argument('--spec', default=DEFAULT_SUB)
    sl.add_argument('-o', default=os.path.join(ROOT, 'src/v34/v34_subobjects.h'))
    sl.add_argument('--check', action='store_true')
    sh = sub.add_parser('shims')
    sh.add_argument('file')
    sh.add_argument('--only', nargs='*', help='shim only these functions')
    sh.add_argument('--list', action='store_true')
    by = sub.add_parser('bytes')
    common(by)
    by.add_argument('files', nargs='+')
    by.add_argument('--sub', help='flat sub-object mode: the base token(s) address the sub-object KEY of layouts/v34/v34sub.txt; offsets become native distances inside the flat V34Obj (V34REL)')
    by.add_argument('--funcs', nargs='*', help='with --sub: only the sites inside these functions (a function whose `obj` parameter is the sub-object)')
    by.add_argument('--macros', action='store_true', help='also the sites inside #define lines whose base token is the requested one (a macro parameter named like the base)')
    by.add_argument('--accessors', action='store_true', help='also the NAME(obj, LIT [+ rest]) sites of the file own accessor macros that the typed rewrite left (non-member / computed offsets)')
    by.add_argument('--apply', action='store_true', help='rewrite in place and check 32-bit identity against HEAD (restore on failure)')
    by.add_argument('-I', dest='incs', action='append', default=[])
    by.add_argument('--opt', nargs='+', default=['-O0', '-O2', '-O2 -fno-strict-aliasing'])
    em = sub.add_parser('embed')
    em.add_argument('files', nargs='+')
    em.add_argument('--layout', default=DEFAULT_LAYOUT)
    em.add_argument('--funcs', nargs='*', help='only the calls inside these functions')
    em.add_argument('--bases', action='store_true', help='instead: OBJ_BYTES(obj, V34OFF(0xB)) of an embedded sub-object base -> V34SUBP(obj, member)')
    em.add_argument('--apply', action='store_true', help='rewrite in place and check 32-bit identity against the pre-edit text (restore on failure)')
    em.add_argument('-I', dest='incs', action='append', default=[])
    em.add_argument('--opt', nargs='+', default=['-O0', '-O2', '-O2 -fno-strict-aliasing'])
    mh = sub.add_parser('mapheader')
    mh.add_argument('--layout', default=DEFAULT_LAYOUT)
    mh.add_argument('-o', required=True)
    mh.add_argument('--check', action='store_true')
    mh.add_argument('--class', dest='cls', default='tagV34Object')
    mh.add_argument('--name', default='V34OBJ', help='symbol prefix of the map (V34OBJ_MAP ...)')
    mh.add_argument('--inc', default='../src/v34/v34_object_view.h')
    pa = sub.add_parser('paths')
    pa.add_argument('--layout', default=DEFAULT_LAYOUT)
    pa.add_argument('--spec', default=DEFAULT_SUB)
    pa.add_argument('-o', default=os.path.join(ROOT, 'layouts', 'v34', 'v34obj_paths.txt'))
    pa.add_argument('--check', action='store_true')
    a = sub.add_parser('apply')
    common(a)
    a.add_argument('files', nargs='+')
    a.add_argument('-I', dest='incs', action='append', default=[])
    a.add_argument('--opt', nargs='+', default=['-O0', '-O2', '-O2 -fno-strict-aliasing'])
    args = ap.parse_args()

    if args.cmd == 'check':
        incs = args.incs or [INCROOT]
        if args.orig:   # RS4: the original's own directory (src/<module>/) resolves its bare same-directory includes for the temporary copies
            incs = incs + [os.path.dirname(os.path.abspath(args.orig))]
        orig = open(args.orig).read() if args.orig else head_text(args.new)
        sys.exit(0 if check_identity(args.new, orig, incs, args.opt, args.cflags) else 1)

    if args.cmd == 'shims':
        text = open(args.file).read()
        if SHIM_BEGIN in text:
            pre = text[:text.index(SHIM_BEGIN)]
            post = text[text.index(SHIM_END) + len(SHIM_END):]
        elif SHIM_MARK in text:
            pre, _, post = text.partition(SHIM_MARK)
        else:
            sys.exit('%s: no %s marker line' % (args.file, SHIM_MARK))
        blk, names, skipped = shim_block(pre, set(args.only) if args.only else None)
        if args.list:
            print(' '.join(names))
            return
        open(args.file, 'w').write(pre + blk + post)
        print('%s: %d shims%s' % (args.file, len(names), ('; skipped (variadic / function pointer params): ' + ' '.join(skipped)) if skipped else ''))
        return
    if args.cmd == 'mapheader':
        c = load_class(args.layout, args.cls)
        orel = os.path.relpath(os.path.abspath(args.o), ROOT)
        cmd = 'python3 tools/tier_rewrite.py mapheader' + ('' if args.cls == 'tagV34Object' else ' --layout %s --class %s --name %s --inc %s' % (os.path.relpath(os.path.abspath(args.layout), ROOT), args.cls, args.name, args.inc)) + ' -o ' + orel
        paths = field_paths(c.fields, embed_list(c, read_sub_spec(DEFAULT_SUB))) if args.cls == 'tagV34Object' else None
        text = gen_map_header(c, os.path.relpath(os.path.abspath(args.layout), ROOT), orel, args.name, args.inc, cmd, paths)
        if args.check:
            if open(args.o).read() != text:
                sys.exit('%s differs from the freshly generated map' % args.o)
            print('map up to date')
        else:
            open(args.o, 'w').write(text)
            print('%s written (%d entries)' % (args.o, text.count('\n  {')))
        return
    if args.cmd == 'slice':
        c = load_class(args.layout, 'tagV34Object')
        text, rep = gen_sub_header(c, read_sub_spec(args.spec), os.path.relpath(os.path.abspath(args.layout), ROOT), os.path.relpath(os.path.abspath(args.spec), ROOT))
        if args.check:
            if open(args.o).read() != text:
                sys.exit('%s differs from the freshly generated header' % args.o)
            print('sub-object header up to date')
        else:
            open(args.o, 'w').write(text)
            print('%s written' % args.o)
        for r in rep:
            print('  ' + r)
        return
    if args.cmd == 'paths':
        c = load_class(args.layout, 'tagV34Object')
        text = gen_paths(c, read_sub_spec(args.spec), os.path.relpath(os.path.abspath(args.layout), ROOT), os.path.relpath(os.path.abspath(args.spec), ROOT))
        if args.check:
            if open(args.o).read() != text:
                sys.exit('%s differs from the freshly generated list' % args.o)
            print('paths up to date')
        else:
            open(args.o, 'w').write(text)
            print('%s written (%d members)' % (args.o, text.count('\n') - 3))
        return
    if args.cmd == 'embed':
        flat = embed_context(args, args.layout)
        status = 0
        for f in args.files:
            t = open(f).read()
            mask = mask_text(t)
            accs = accessors(t, mask)
            res = None
            if args.bases:
                t2, nb = embed_bases(t, mask, flat)
                open(f, 'w').write(t2)
                ok = check_identity(f, t, args.incs or [INCROOT], args.opt, [], quiet=True)
                print('%s: %d sub-object bases -> V34SUBP%s' % (os.path.basename(f), nb, '' if ok else ' [IDENTITY FAILED, file restored]'))
                if not ok:
                    open(f, 'w').write(t)
                    status = 1
                continue
            for form in ('member', 'offsetof', 'plain'):
                t2, st = embed_rewrite(t, mask, flat, accs, None, form, set(args.funcs) if args.funcs else None)
                if not args.apply:
                    res = (t2, st, form)
                    break
                open(f, 'w').write(t2)
                incs = args.incs or [INCROOT]
                if check_identity(f, t, incs, args.opt, [], quiet=True):
                    res = (t2, st, form)
                    break
            print('%s: %s' % (os.path.basename(f), ', '.join('%s %d' % kv for kv in st.items())) + (' [form %s]' % res[2] if res else ' [IDENTITY FAILED, file restored]'))
            if args.apply and res is None:
                open(f, 'w').write(t)
                status = 1
            elif not args.apply:
                sys.stdout.write(res[0]) if False else None
        sys.exit(status)
    tgt = TGT_OBJ
    if args.cmd != 'header' and args.struct != 'obj':
        if args.struct in EXTERNAL:
            lay, cn, pf, hf = EXTERNAL[args.struct]
            cls = load_class(os.path.join(ROOT, lay), cn)
            tgt = dict(prefix=pf, ctype=cls.mirror, t='T_' + args.struct, ti='TI_' + args.struct, header='src/v34/' + hf)
            args.header = hf
        else:
            spec = {k: (n, bs, sz) for k, n, bs, sz, _ in read_sub_spec(DEFAULT_SUB)}
            if args.struct not in spec:
                sys.exit('--struct %s: not obj / %s / a key of layouts/v34/v34sub.txt (%s)' % (args.struct, '/'.join(EXTERNAL), ' '.join(spec)))
            n, bs, sz = spec[args.struct]
            cls = make_slice(load_class(args.layout, args.cls), n, bs[0], sz)
            tgt = dict(prefix='V34%s_' % args.struct.upper(), ctype=n, t='T_' + args.struct, ti='TI_' + args.struct, header='src/v34/v34_subobjects.h')
            args.header = 'v34_subobjects.h'
    else:
        cls = load_class(args.layout, args.cls)
    mem = members(cls)
    bases = args.base or ['obj']
    layout_rel = os.path.relpath(os.path.abspath(args.layout), ROOT)

    if args.cmd == 'header':
        global CXX
        CXX = CxxSpec(args.cxx) if args.cxx else None
        emb = embed_list(cls, read_sub_spec(DEFAULT_SUB)) if (args.cls == 'tagV34Object' and args.prefix == PREFIX and not args.no_embed) else None
        text = gen_header(cls, layout_rel, args.prefix, args.guard, args.o and os.path.relpath(os.path.abspath(args.o), ROOT) or 'src/v34/v34_object_view.h', embeds=emb, tag=args.tag)
        if args.check:
            cur = open(args.o or DEFAULT_HEADER).read()
            if cur != text:
                sys.exit('%s differs from the freshly generated header' % (args.o or DEFAULT_HEADER))
            print('header up to date (%d members, %d distinct offsets)' % (len(cls.fields), len(mem)))
        elif args.o:
            open(args.o, 'w').write(text)
            print('%s: %d members, %d distinct member offsets' % (args.o, len(cls.fields), len(mem)))
        else:
            sys.stdout.write(text)
        return

    if args.cmd == 'bytes':
        status = 0
        for f in args.files:
            t = open(f).read()
            mask = mask_text(t)
            sites = find_bytes_sites(t, mask, bases, accessors(t, mask) if args.accessors else None, args.macros)
            funcs = func_starts(mask)
            B = None
            Bj = None
            only = None
            if args.sub:
                spec = {k: (n, bs, sz) for k, n, bs, sz, _ in read_sub_spec(DEFAULT_SUB)}
                B = spec[args.sub][1][0]
                Bj = None
                if B not in mem:
                    jb = bytes_binding(cls, mem, B)
                    if not jb or jb[0] != 'in':
                        sys.exit('sub-object %s: base 0x%x is not inside a member of the layout' % (args.sub, B))
                    Bj = (jb[1], jb[2])
                only = set(args.funcs) if args.funcs else None
            t2, n, rep, left = bytes_rewrite(t, mask, sites, cls, mem, args.header, None if args.sub else load_deny(args.deny_file, os.path.basename(f)), funcs, B, only, Bj)
            print('%s: %d OBJ_BYTES sites, %d rewritten, %d left' % (os.path.basename(f), len(sites), n, len(left)))
            for (val, rest, run) in rep:
                print('   0x%x%s   contiguous run (to the next pointer member) 0x%x' % (val, (' + ' + rest) if rest else '', run))
            for (txt, rest) in left:
                print('   LEFT: %s' % txt)
            if args.apply and n:
                open(f, 'w').write(t2)
                incs = args.incs or [INCROOT]
                if not check_identity(f, t, incs, args.opt, [], quiet=False):
                    open(f, 'w').write(t)
                    print('%s: NOT identical at 32 bit: file restored' % f)
                    status = 1
        sys.exit(status)

    allw = {}
    status = 0
    for f in args.files:
        t, mask, accs, sites = analyse(f, mem, bases, args.deny_file)
        ndeny = analyse.denied
        wl = worklist(sites, mem, accs, cls)
        onunk = sum(1 for s in sites if s[5] in mem and mem[s[5]][1] == 'unknown-region')
        if args.hazards:
            for (o, mac, msg) in hazards(sites, accs, cls, mem):
                print('HAZARD %s 0x%x %s: %s' % (os.path.basename(f), o, mac, msg))
        if args.cmd == 'worklist':
            allw[os.path.basename(f)] = {'accessors': accs, 'sites': len(sites), 'denied': ndeny, 'resolved': len(sites) - sum(wl.values()),
                                         'on_unknown_region': onunk, 'worklist': {'0x%x' % k: v for k, v in wl.items()}}
            continue
        if args.cmd in ('rewrite', 'apply'):
            inplace = args.cmd == 'apply' or args.in_place
            mode = getattr(args, 'mode', 'partial')
            t2, n = rewrite_text(t, mask, accs, sites, mem, mode, args.header, cls, 'member', tgt)
            if args.cmd == 'apply':
                incs = args.incs or [INCROOT]
                ok = False
                for form in ('member', 'offsetof'):
                    t2, n = rewrite_text(t, mask, accs, sites, mem, mode, args.header, cls, form, tgt)
                    open(f, 'w').write(t2)
                    ok = check_identity(f, t, incs, args.opt, [], quiet=(form == 'member'))
                    if ok:
                        print('%s: accessor form: %s' % (os.path.basename(f), form))
                        break
                if not ok:
                    open(f, 'w').write(t)
                    print('%s: identity FAILED, file restored' % f)
                    status = 1
            elif inplace:
                open(f, 'w').write(t2)
            elif args.o:
                open(args.o, 'w').write(t2)
            else:
                sys.stdout.write(t2)
            print('%s: %d literal-%s sites (%d in excepted functions, not touched), %d rewritten, %d unresolved (%d distinct offsets), %d on an opaque unknown-region start, accessors=%s' % (
                os.path.basename(f), len(sites), '/'.join(bases), ndeny, n, sum(wl.values()), len(wl), onunk, ','.join(sorted(accs))), file=sys.stderr)
    if args.cmd == 'worklist':
        if args.json:
            json.dump(allw, sys.stdout, indent=1)
        else:
            for k, v in allw.items():
                print('%s: sites=%d (excepted functions: %d) resolved=%d (on unknown-region start: %d) unresolved=%d distinct=%d' % (
                    k, v['sites'], v['denied'], v['resolved'], v['on_unknown_region'], sum(v['worklist'].values()), len(v['worklist'])))
                print('   ' + ' '.join('%s*%d' % kv for kv in v['worklist'].items()))
    sys.exit(status)


if __name__ == '__main__':
    main()
