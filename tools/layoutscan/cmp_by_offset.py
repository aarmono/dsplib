#!/usr/bin/env python3
"""cmp_by_offset.py VIEW REAL [--base N] [--quiet]
VIEW / REAL = <json-prefix>:<TU>:<Type>   (json prefix 'types' = production objects, 'hdrtypes' = header probes; files <prefix>32.json / <prefix>64.json)
Flattens both record types recursively into leaf members (embedded records expanded, arrays kept as one leaf) with their i386 and x86-64 offsets, then
pairs every VIEW leaf with the REAL leaf that covers the same i386 byte (VIEW i386 offset + base).  A pair DISAGREES at -m64 when
   x64(VIEW leaf) != x64(REAL leaf) + (i386 delta inside the REAL leaf)
(delta is only meaningful for byte/array leaves; for a scalar landing inside another scalar the pair is reported as 'partial').
Prints counts and the disagreeing pairs.  Exit 0."""
import json, sys
_cache = {}


def load(prefix, bits):
    k = (prefix, bits)
    if k not in _cache:
        _cache[k] = json.load(open('%s%s.json' % (prefix, bits)))
    return _cache[k]


def flat(T, tu, name, base, path, out, depth=0):
    t = T[tu].get(name)
    if t is None:
        # nested record not emitted in this TU: look in any TU
        for k in T:
            if name in T[k]:
                t = T[k][name]
                break
    if t is None or depth > 12:
        out.append((base, path or name, 'rec?', None))
        return
    for m, (off, kind, sz) in sorted(t['members'].items(), key=lambda x: x[1][0]):
        p = (path + '.' if path else '') + m
        if kind.startswith('rec:') and not kind.endswith('?'):
            sub = kind[4:]
            flat(T, tu, sub, base + off, p, out, depth + 1)
        else:
            out.append((base + off, p, kind, sz))


def leaves(spec, bits):
    prefix, tu, name = spec.split(':', 2)
    T = load(prefix, bits)
    if name not in T.get(tu, {}):
        tu = next(k for k in sorted(T) if name in T[k])
    out = []
    flat(T, tu, name, 0, '', out)
    sz = T[tu][name]['size']
    return out, sz


def main():
    a = sys.argv[1:]
    base = int(a[a.index('--base') + 1], 0) if '--base' in a else 0
    quiet = '--quiet' in a
    view, real = a[0], a[1]
    v32, vs32 = leaves(view, '32'); v64, vs64 = leaves(view, '64')
    r32, rs32 = leaves(real, '32'); r64, rs64 = leaves(real, '64')
    v64m = dict((p, o) for o, p, k, s in v64)
    r64m = dict((p, o) for o, p, k, s in r64)
    rl = sorted(r32)
    def cover(off):
        best = None
        for o, p, k, s in rl:
            if s == 0:
                continue
            if o <= off and (off < o + (s if s is not None else 1)):
                best = (o, p, k, s)
            if o > off:
                break
        return best
    c0 = cover(base)
    base64 = (r64m[c0[1]] + base - c0[0]) if c0 else base
    n = agree = 0
    bad = []; partial = []; gap = []; opq = []
    for o, p, k, s in v32:
        if p.startswith('pad_') or k.startswith('arr:') and (s == 0):
            continue
        c = cover(o + base)
        if c is None:
            gap.append(p); continue
        ro, rp, rk, rs = c
        d = o + base - ro
        got = v64m.get(p)
        if got is None:
            continue
        n += 1
        exp = r64m[rp] + d - base64
        if d and not (rk.startswith('arr') or rk in ('char', 'unsigned char', 'signed char')):
            partial.append((p, rp, d))
        if got != exp:
            if rk in ('arr:unsigned char', 'arr:char', 'arr:signed char') and d:
                opq.append((p, o, got, rp + ('+%d' % d if d else ''), exp))
            else:
                bad.append((p, o, got, rp + ('+%d' % d if d else ''), exp))
        else:
            agree += 1
    print('VIEW %s  size i386 %d x64 %d | REAL %s size i386 %d x64 %d | base i386 0x%x (x64 0x%x)' % (view, vs32, vs64, real, rs32, rs64, base, base64))
    print('  view leaves paired: %d, agree at -m64: %d, DISAGREE (real leaf is a named member / starts there): %d, differ INSIDE a real opaque byte region: %d, partial: %d, unpaired: %d' % (n, agree, len(bad), len(opq), len(partial), len(gap)))
    if '--opaque' in sys.argv:
        for b in opq[:60]:
            print('    OPQ %-28s i386 0x%05x x64 view 0x%05x | real %-40s naive x64 0x%05x' % b)
    if not quiet:
        for b in bad[:40]:
            print('    DIS %-28s i386 0x%05x x64 view 0x%05x | real %-40s expected x64 0x%05x' % b)
        if len(bad) > 40:
            print('    ... %d more' % (len(bad) - 40))
        for p in partial[:10]:
            print('    partial', p)


main()
