#!/usr/bin/env python3
"""layout_gen.py - per-class layout description -> harness mirror structs + field tables + i386 asserts + layout probe.

ONE description per chunk (layouts/<chunk>.layout, plain text), up to three generated files (all committed; regenerate with the same
command; the output is deterministic; each is produced only when its option is given):

  --out-test   <dir>/<chunk>_layout.h     C mirror structs (for a test harness; the library does not use it) (REAL pointer members, so they widen on x86-64 like the port's
                                          classes), HFld field tables for h_dump_obj (of the harness), i386
                                          _Static_asserts of the mirrors, <chunk>_layout_check() which compares every mirror
                                          against the real port class through the probe
  --out-port   src/<module>/<name>_layout.inc  DSP_ASSERT_LAYOUT_I386(...) text, #include it at the END of the port header
                                          (guarded by __i386__ through DSP_ASSERT_LAYOUT_I386, src/core/dsp_cxx.h)
  --out-probe  <dir>/<chunk>_layout_probe.cpp  exports sizeof / offsetof of the real classes (C++, no code instantiated)

Description syntax (blank lines and `#` comments ignored; numbers decimal or 0x..):

  class <Name> size <bytes> cxx <C++ type> mirror <C mirror struct name>
      <offset> <kind> <member> [<count>]
      ...

  A class line may end with `prefix <n>` (added with RS6): the description (and so the mirror) covers only the first <size> bytes of a real class
           whose i386 sizeof is <n> (the dp parameter block: the harness mirrors are the 0x7c-byte prefix the chunk reads, the port class is the whole 0x88 B block).
           The generated port assert is then `sizeof(cxx) == <n>` and the probe's first element is the END OF THE LAST MEMBER (what the mirror's sizeof is).

  <Name>   free text for messages / dump tags; <C++ type> must be a plain identifier (a typedef declared in the port header,
           because the asserts are macro arguments and template commas would break them); member names are the C++ member
           names (they are also the mirror's member names and the names printed by the dump).
  kind     u8 u16 u32 i32 float32 double | ptr-to-heap (an OWNING pointer: the block it points to is dumped with the object)
           ptr-into-heap (pointer that is only normalised, e.g. a read position inside the owner's block) | ptr-to-embedded
           (pointer into the object itself / an embedded sub-object: printed as E+<offset>) | vptr | unknown-region (count =
           byte length; printed as hex) | u32-ptrslot (added with P27: a 32-bit value that is the LAST member of an embedded sub-object
           that also holds pointers (Scrambler<uchar,int>: 7 pointers + tail_): the sub-object is padded to pointer alignment on
           x86-64 and the next member of the owner starts after the padding, so the mirror member is a pointer-sized integer
           (uintptr_t; 4 bytes on i386); printed as a u32 = the low word)
           `vptr` is the IMPLICIT vptr of a polymorphic class (not a declared member: the generated port asserts skip it and the
           probe reports offset 0, the ABI's place of the primary vptr; the harness mirror has a real `void *vptr_` member).
  count    optional array length of a scalar kind (`float32 coef 4`); required for unknown-region.
  member   `name` or `name=a.b`: for a member of an EMBEDDED sub-object (a real class member `a` of class type with a member `b`) or of a BASE
           class (`name=b`: the member `b` the real class inherits) the mirror / dump name is `name` and the generated offsetof asserts / probe use
           the designator of the real class (flattened sub-objects: the mirror has no embedded struct; added with P21 for V90SpectralShaper's PDE /
           filter, P22 for V90PreFilter's FloatFIR base).

The description is SELF-CHECKING: the generator lays the members out with the i386 System V rules (every scalar incl. double
aligned to <= 4, pointers 4 bytes) and refuses a description whose offsets or size differ (an unlisted gap must be written as an
unknown-region; the size must be the i386 sizeof incl. tail padding).  Measured offsets therefore cannot drift silently, and the
same file yields the x86-64 mirror (pointers 8 bytes, doubles 8-aligned) simply by being compiled there.

The kind `align8` (added with P28) is a ZERO-SIZE marker that exists only in the C mirror: `char name[0] __attribute__((aligned(sizeof(void *))))`, i.e. the next member starts pointer-aligned (8 on x86-64)
and nothing changes on i386.  Put it before and after a flattened embedded sub-object that holds a pointer (such a sub-object is 8-aligned and its size is padded to 8 on x86-64, which a flat
mirror does not do by itself).  It is not a member of the real class: no HFld entry, no offsetof assert, no probe entry, no mirror offset.

The kind `pad64` (added with X3) is a marker that exists in BOTH the C mirror and the real class: `<offset> pad64 <name> <bytes>` = `uint8_t name[sizeof(void *) == 8 ? bytes : 0]`
placed immediately before the member at <offset>: nothing on i386, `bytes` on x86-64.  It carries the widening of the pointer slots that the description does not
list (the V.34 module object has 41 pointer slots, most of them inside unknown-regions): the class (until RS5: src/v34/v34_object.h, V34_PAD64) and the mirror shift by the same
amounts, which are those of the tier files' V34Obj (src/v34/v34_object_view.h; the two are checked against each other by the harness).  Like align8 it has no HFld entry, no assert, no probe entry.

Usage: tools/layout_gen.py layouts/p02.layout --chunk p02 --out-test OUT/p02_layout.h --out-port src/dsp/dsp_float_filters_layout.inc \
                           --out-probe OUT/p02_layout_probe.cpp --port-header dsp/dsp_float_filters.h [--harness-include PATH  how the --out-test header includes the harness header]
"""
import argparse
import re
import sys

KINDS = {
    # name: (code, C type, i386 size, i386 align)
    'u8': ('b', 'uint8_t', 1, 1),
    'u16': ('h', 'uint16_t', 2, 2),
    'u32': ('u', 'uint32_t', 4, 4),
    'i32': ('i', 'int32_t', 4, 4),
    'float32': ('f', 'float', 4, 4),
    'double': ('d', 'double', 8, 4),
    'u32-ptrslot': ('u', 'uintptr_t', 4, 4),
    'ptr-to-heap': ('o', 'void *', 4, 4),
    'ptr-into-heap': ('p', 'void *', 4, 4),
    'ptr-to-embedded': ('e', 'void *', 4, 4),
    'vptr': ('v', 'void *', 4, 4),
    'unknown-region': ('x', 'uint8_t', 1, 1),
    'align8': ('a', 'char', 0, 1),
    'pad64': ('z', 'uint8_t', 0, 1),
}


class Fld:
    def __init__(self, off, kind, name, count, line, cxxpath=None):
        self.off, self.kind, self.name, self.count, self.line = off, kind, name, count, line
        self.cxxpath = cxxpath or name   # the member designator of the REAL class (offsetof(cls, <cxxpath>)); differs from `name` for a member of an embedded sub-object


class Cls:
    def __init__(self, name, size, cxx, mirror, line, prefix=None):
        self.name, self.size, self.cxx, self.mirror, self.line = name, size, cxx, mirror, line
        self.prefix = prefix   # RS6: the i386 sizeof of the REAL class when the description (and so the mirror) covers only a PREFIX of it
        self.fields = []


def num(s):
    return int(s, 0)


def parse(text, path):
    classes, cur = [], None
    for ln, raw in enumerate(text.split('\n'), 1):
        line = raw.split('#', 1)[0].strip()
        if not line:
            continue
        t = line.split()
        if t[0] == 'class':
            kv = dict(zip(t[2::2], t[3::2]))
            if len(t) < 2 or len(t) % 2 != 0 or not ({'size', 'cxx', 'mirror'} <= set(kv) <= {'size', 'cxx', 'mirror', 'prefix'}):
                sys.exit('%s:%d: expected: class <Name> size <n> cxx <T> mirror <M> [prefix <i386 sizeof of the real class>]' % (path, ln))
            if not re.fullmatch(r'[A-Za-z_]\w*', kv['cxx']) or not re.fullmatch(r'[A-Za-z_]\w*', kv['mirror']):
                sys.exit('%s:%d: cxx / mirror must be plain identifiers' % (path, ln))
            cur = Cls(t[1], num(kv['size']), kv['cxx'], kv['mirror'], ln, num(kv['prefix']) if 'prefix' in kv else None)
            classes.append(cur)
            continue
        if cur is None:
            sys.exit('%s:%d: member before any class' % (path, ln))
        if len(t) not in (3, 4) or t[1] not in KINDS:
            sys.exit('%s:%d: expected: <offset> <kind> <member> [<count>]; kinds: %s' % (path, ln, ' '.join(KINDS)))
        count = num(t[3]) if len(t) == 4 else 0
        if t[1] in ('unknown-region', 'pad64') and count < 1:
            sys.exit('%s:%d: unknown-region needs a byte count' % (path, ln))
        cxxpath = None
        if '=' in t[2]:
            t[2], cxxpath = t[2].split('=', 1)
            if not re.fullmatch(r'[A-Za-z_]\w*(\.[A-Za-z_]\w*)*', cxxpath):
                sys.exit('%s:%d: bad C++ member path %r (expected a or a.b[.c])' % (path, ln, cxxpath))
        if not re.fullmatch(r'[A-Za-z_]\w*', t[2]):
            sys.exit('%s:%d: bad member name' % (path, ln))
        cur.fields.append(Fld(num(t[0]), t[1], t[2], count, ln, cxxpath))
    return classes


def check_i386(c, path):
    """lay the members out with the i386 rules and compare with the stated offsets / size"""
    pos, maxal, seen = 0, 1, set()
    for f in c.fields:
        _, _, sz, al = KINDS[f.kind]
        n = max(f.count, 1)
        pos = (pos + al - 1) // al * al
        if f.name in seen:
            sys.exit('%s:%d: duplicate member %s in %s' % (path, f.line, f.name, c.name))
        seen.add(f.name)
        if pos != f.off:
            sys.exit('%s:%d: %s::%s is at 0x%x in the description but the i386 layout of the preceding members puts it at 0x%x '
                     '(add an unknown-region for the gap, or fix the offset)' % (path, f.line, c.name, f.name, f.off, pos))
        pos += sz * n
        maxal = max(maxal, al)
    total = (pos + maxal - 1) // maxal * maxal
    if total != c.size:
        sys.exit('%s:%d: %s: size is 0x%x in the description, the i386 layout gives 0x%x (last member ends at 0x%x)' %
                 (path, c.line, c.name, c.size, total, pos))


def gen_test(classes, chunk, src, harness_inc='harness/harness.h'):
    o = ['/* GENERATED by tools/layout_gen.py from %s: do not edit.  Harness mirrors (C structs with real pointer members), field' % src,
         ' * tables for h_dump_obj and the mirror-vs-port-class layout check of chunk %s. */' % chunk,
         '#ifndef %s_LAYOUT_H' % chunk.upper(), '#define %s_LAYOUT_H' % chunk.upper(), '',
         '#include <stddef.h>', '#include <stdint.h>', '#include "%s"' % harness_inc, '']
    for c in classes:
        o.append('/* %s (i386 sizeof 0x%x) */' % (c.name, c.size))
        o.append('typedef struct {')
        for f in c.fields:
            _, ct, _, _ = KINDS[f.kind]
            if f.kind == 'align8':
                o.append('  char %s[0] __attribute__((aligned(sizeof(void *)))); /* 0x%02x align8 (pointer alignment: 4 on i386, 8 on x86-64) */' % (f.name, f.off))
                continue
            if f.kind == 'pad64':
                o.append('  uint8_t %s[sizeof(void *) == 8 ? %d : 0]; /* 0x%02x pad64 (x86-64 only: widening of unlisted pointer slots) */' % (f.name, f.count, f.off))
                continue
            arr = '[%d]' % f.count if f.count else ''
            o.append('  %s%s%s%s; /* 0x%02x %s */' % (ct, '' if ct.endswith('*') or ct.endswith('*') else ' ', f.name, arr, f.off, f.kind))
        o.append('} %s;' % c.mirror)
        o.append('#if defined(__i386__)')
        conds = ['sizeof(%s) == 0x%x' % (c.mirror, c.size)] + ['offsetof(%s, %s) == 0x%x' % (c.mirror, f.name, f.off) for f in c.fields if f.kind not in ('align8', 'pad64')]
        o.append('_Static_assert(%s,' % ' &&\n                   '.join(conds))
        o.append('               "%s mirror layout (measured from the blob)");' % c.name)
        o.append('#endif')
        o.append('static const HFld f_%s[] = {' % c.mirror)
        for f in c.fields:
            code = KINDS[f.kind][0]
            if f.kind in ('align8', 'pad64'):
                continue
            if f.count:
                o.append("  HFLDN(%s, %s, '%s', %d)," % (c.mirror, f.name, code, f.count))
            else:
                o.append("  HFLD(%s, %s, '%s')," % (c.mirror, f.name, code))
        o.append('};')
        o.append('#define NF_%s ((int)(sizeof f_%s / sizeof f_%s[0]))' % (c.mirror, c.mirror, c.mirror))
        o.append('static const unsigned mirror_%s[] = { sizeof(%s)%s };' % (
            c.mirror, c.mirror, ''.join(', offsetof(%s, %s)' % (c.mirror, f.name) for f in c.fields if f.kind not in ('align8', 'pad64'))))
        o.append('extern const unsigned %s_probe_%s[];' % (chunk, c.mirror))
        o.append('')
    o.append('/* the mirrors must equal the REAL port classes (the probe is C++ compiled from the port header) on every architecture */')
    o.append('static void %s_layout_check(void) {' % chunk)
    for c in classes:
        o.append('  h_layout_compare("%s", mirror_%s, %s_probe_%s, %d);' % (c.name, c.mirror, chunk, c.mirror, 1 + len([f for f in c.fields if f.kind not in ('align8', 'pad64')])))
    o.append('}')
    o.append('')
    o.append('#endif /* %s_LAYOUT_H */' % chunk.upper())
    return '\n'.join(o) + '\n'


def gen_port(classes, chunk, src, banner='header'):
    # banner 'header' (default): the generic wording; 'chunk': the wording of the 30 src/vpcm .inc files, whose banners were edited by hand in 03806a5 (TESTREORG6)
    what = 'i386 layout asserts of chunk %s' % chunk if banner == 'chunk' else '32-bit layout asserts of the classes of the header that includes this file'
    o = ['/* GENERATED by tools/layout_gen.py from %s: do not edit.  %s;' % (src, what),
         ' * no include guard: include ONCE, at the end of the class header, after the typedefs it names.  The asserts are active only',
         ' * on i386 (DSP_ASSERT_LAYOUT_I386 in dsp_cxx.h); the 64-bit build derives its own layout. */']
    for c in classes:
        o.append('DSP_ASSERT_LAYOUT_I386(sizeof(%s) == 0x%x, "sizeof %s");' % (c.cxx, c.prefix or c.size, c.name))
        grp = []
        for f in c.fields:
            if f.kind in ('vptr', 'align8', 'pad64'):
                continue  # the implicit vptr of a polymorphic class is not a member (offset 0 by the ABI): nothing to offsetof
            grp.append('offsetof(%s, %s) == 0x%02x' % (c.cxx, f.cxxpath, f.off))
        for i in range(0, len(grp), 2):
            o.append('DSP_ASSERT_LAYOUT_I386(%s, "offsets %s");' % (' && '.join(grp[i:i + 2]), c.name))
    return '\n'.join(o) + '\n'


def gen_probe(classes, chunk, header, src):
    o = ['/* GENERATED by tools/layout_gen.py from %s: do not edit.  sizeof / offsetof of the REAL port classes, for the harness\' mirror check. */' % src,
         '#include <stddef.h>', '#include "%s"' % header, '', 'extern "C" {']
    for c in classes:
        o.append('extern const unsigned %s_probe_%s[];' % (chunk, c.mirror))
        last = [f for f in c.fields if f.kind not in ('align8', 'pad64')][-1]
        if c.prefix and last.kind in ('u8', 'u16', 'u32', 'i32', 'float32', 'double'):   # EOS: a plain scalar mirror member: its end is offset + the MIRROR's size, whatever the real member's type is (w78_ = the low 32 bits of the pointer `path`)
            size0 = 'offsetof(%s, %s) + sizeof(%s)%s' % (c.cxx, last.cxxpath, KINDS[last.kind][1], (' * %d' % last.count) if last.count else '')
        else:
            size0 = ('offsetof(%s, %s) + sizeof(%s::%s)' % (c.cxx, last.cxxpath, c.cxx, last.cxxpath)) if c.prefix else 'sizeof(%s)' % c.cxx   # a prefix class: the end of its last member (what the mirror's sizeof is)
        o.append('const unsigned %s_probe_%s[] = { %s%s };' % (
            chunk, c.mirror, size0, ''.join((', 0u' if f.kind == 'vptr' else ', offsetof(%s, %s)' % (c.cxx, f.cxxpath)) for f in c.fields if f.kind not in ('align8', 'pad64'))))
    o.append('}')
    return '\n'.join(o) + '\n'


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('desc')
    ap.add_argument('--chunk', required=True)
    ap.add_argument('--out-test')
    ap.add_argument('--out-port')
    ap.add_argument('--out-probe')
    ap.add_argument('--port-header', help='port header the probe includes, relative to the include root (src/): <module>/<name>.h; required with --out-probe')
    ap.add_argument('--harness-include', default='harness/harness.h', help='how the generated --out-test header includes the harness header (a quote include, relative to the directory of the '
                    'header)')
    ap.add_argument('--paths', help='file of `<member> <designator>` lines (tier_rewrite.py paths): members of the class tagV34Object that live inside a sole sub-object struct '
                    '(`rxFlags_ rx.rxFlags_`); the designator replaces the member name in the generated offsetof asserts and in the probe')
    ap.add_argument('--src-label', help='the description name printed in the "GENERATED ... from <name>" banners instead of the path of the description (layouts/p29.layout was committed as p29.layout)')
    ap.add_argument('--inc-banner', choices=('header', 'chunk'), default='header', help='wording of the first line of the --out-port file (default header: generic; chunk: "i386 layout asserts of chunk <chunk>", the 30 src/vpcm files)')
    a = ap.parse_args()
    label = a.src_label or a.desc
    text = open(a.desc).read()
    classes = parse(text, a.desc)
    if not classes:
        sys.exit('no class in %s' % a.desc)
    if a.paths:
        pm = dict(ln.split() for ln in open(a.paths).read().split('\n') if ln.strip() and not ln.startswith('#'))
        for c in classes:
            if c.name == 'tagV34Object':
                for f in c.fields:
                    f.cxxpath = pm.get(f.name, pm.get(f.cxxpath, f.cxxpath))
    for c in classes:
        check_i386(c, a.desc)
    # the probe #includes the port header as spelled here, compiled with -I<include root>: since RS4 that is the root-relative path (vpcm/x.h);
    # the old default dsp_classes_<chunk>.h names no file any more
    if a.out_probe and not a.port_header:
        sys.exit('layout_gen: --port-header <module>/<name>.h is required with --out-probe (the header as the probe includes it, relative to src/)')
    hdr = a.port_header
    for path, gen in ((a.out_test, lambda: gen_test(classes, a.chunk, label, a.harness_include)), (a.out_port, lambda: gen_port(classes, a.chunk, label, a.inc_banner)),
                      (a.out_probe, lambda: gen_probe(classes, a.chunk, hdr, label))):
        if path:
            open(path, 'w').write(gen())
    print('layout_gen: %d classes, %d members' % (len(classes), sum(len(c.fields) for c in classes)))


if __name__ == '__main__':
    main()
