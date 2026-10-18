#!/usr/bin/env python3
"""member_map_source.py - Mode B of tools/member_map.py: raw byte-offset struct accesses in the PORTED sources (the conversion checklist
for the later 64-bit conversion) cross-checked against the layout descriptions layouts/pNN.layout.

  tools/member_map.py source src/dsp/dsp_resampler.cpp src/vpcm/pcm_leaf_classes.cpp ... [--layout layouts/pNN.layout ...] [--all] [--markdown] [--json]

No file under src/ is changed; the report is the only output.  What is recognised (comments and string literals are blanked first):
  * `*(T *)(EXPR)` / `*(T *)((char *)base + off)` / `*(T *)(intptr_t)(base + off)` / `*(T *)OBJ_BYTES(base, off)` where EXPR reduces, after the
    casts are dropped, to `NAME + <number>` (the offset is a literal; `NAME + <variable>` is reported as dynamic, not as an offset);
  * word-index forms `((T *)base)[<number>]` (offset = index * sizeof(T));
  * the accessor MACROS and tiny inline FUNCTIONS defined in the scanned files that wrap such a deref, found by parsing the definitions
    (`#define S16(o, off) (*(short *)OBJ_BYTES(o, off))`, `E32(base, o)`, `SRE_S(o)` with the base fixed, `static inline uint32_t paramWord(const V90Parameters *p,
    unsigned off)`); a call with a literal offset is one access of the macro's width.
The base variable is bound to a struct when a declaration `TYPE *name` is found in the enclosing function (parameters or locals) and TYPE is a layout class
(the layout's name or its `cxx` typedef), or `this` inside `Class::method`; `--bind NAME=Class` adds a binding by hand.  An access is then looked up in the
layout: member and delta, flags `GAP` (unnamed gap) / `UNK` (falls in an unknown-region) / `STRADDLE` / `WIDTH` (width differs from the member kind), and the
offset the member would have on x86-64 (pointers 8 bytes / 8-aligned, doubles 8-aligned, the implicit vptr 8 bytes): `x64 WRONG` when a raw offset no longer
points at the same member.  Accesses whose base cannot be bound are counted per file and base name (all listed with --all).
"""
import os
import re
import sys
from collections import defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

TYPE_W = {'char': 1, 'signed char': 1, 'unsigned char': 1, 'uint8_t': 1, 'int8_t': 1, 'u8_t': 1, 's8_t': 1, 'short': 2, 'unsigned short': 2,
          'uint16_t': 2, 'int16_t': 2, 'u16_t': 2, 's16_t': 2, 'int': 4, 'unsigned': 4, 'unsigned int': 4, 'uint32_t': 4, 'int32_t': 4,
          'u32_t': 4, 's32_t': 4, 'float': 4, 'long': 4, 'unsigned long': 4, 'double': 8, 'int64_t': 8, 'uint64_t': 8, 'long long': 8,
          'unsigned long long': 8, 'void': 0, 'long double': 10}
NUM = r'(?:0[xX][0-9a-fA-F]+|\d+)'


def blank(text):
    """comments and string / char literals -> spaces (newlines kept so line numbers survive)"""
    out, i, n = [], 0, len(text)
    while i < n:
        c = text[i]
        if text.startswith('/*', i):
            j = text.find('*/', i + 2)
            j = n if j < 0 else j + 2
            out.append(''.join('\n' if ch == '\n' else ' ' for ch in text[i:j]))
            i = j
        elif text.startswith('//', i):
            j = text.find('\n', i)
            j = n if j < 0 else j
            out.append(' ' * (j - i))
            i = j
        elif c in '"\'':
            j = i + 1
            while j < n and text[j] != c:
                j += 2 if text[j] == '\\' else 1
            out.append(c + ' ' * (j - i - 1) + c)
            i = j + 1
        else:
            out.append(c)
            i += 1
    return ''.join(out)


def balanced(text, i, open_='(', close=')'):
    """text[i] == open_: index just after the matching close"""
    d = 0
    for j in range(i, len(text)):
        if text[j] == open_:
            d += 1
        elif text[j] == close:
            d -= 1
            if d == 0:
                return j + 1
    return len(text)


def split_args(s):
    out, d, cur = [], 0, ''
    for ch in s:
        if ch in '([':
            d += 1
        elif ch in ')]':
            d -= 1
        if ch == ',' and d == 0:
            out.append(cur.strip())
            cur = ''
        else:
            cur += ch
    if cur.strip():
        out.append(cur.strip())
    return out


def type_width(t):
    t = re.sub(r'\b(const|volatile|struct)\b', '', t)
    t = re.sub(r'\s+', ' ', t.replace('*', ' * ')).strip()
    if '*' in t:
        return 4, 'ptr'
    return TYPE_W.get(t, TYPE_W.get(t.replace('signed ', ''), None)), t


def evalnum(s):
    s = s.strip()
    if re.fullmatch(r'[0-9a-fA-FxX+\-*() ]+', s) and re.search(NUM, s):
        try:
            return eval(s, {'__builtins__': {}})
        except Exception:
            return None
    return None


CASTS = re.compile(r'\(\s*(?:const\s+|volatile\s+|struct\s+)*(?:unsigned\s+|signed\s+)?[A-Za-z_]\w*(?:\s+(?:long|int|char|short))*\s*\*+\s*\)|\(\s*(?:intptr_t|uintptr_t|unsigned|int|size_t|ptrdiff_t|long)\s*\)')


def reduce_expr(e):
    """`((char *)(base) + (0x2c))` -> ('base', 0x2c);  `base` -> ('base', 0);  None when it is not NAME + literal"""
    e = CASTS.sub('', e)
    e = re.sub(r'\s+', '', e)
    for _ in range(6):
        m = re.fullmatch(r'\((.*)\)', e)
        if m and balanced(e, 0) == len(e):
            e = m.group(1)
        else:
            break
    e = re.sub(r'\((' + NUM + r')\)', r'\1', e)
    e = re.sub(r'\((\w+(?:->\w+|\.\w+)*)\)', r'\1', e)
    m = re.fullmatch(r'(\w+(?:->\w+|\.\w+)*)\+(' + NUM + r')', e)
    if m:
        return m.group(1), int(m.group(2), 0)
    m = re.fullmatch(r'(\w+(?:->\w+|\.\w+)*)\+\((' + NUM + r')\)', e)
    if m:
        return m.group(1), int(m.group(2), 0)
    m = re.fullmatch(r'(\w+(?:->\w+|\.\w+)*)', e)
    if m:
        return m.group(1), 0
    m = re.fullmatch(r'(\w+(?:->\w+|\.\w+)*)\+(.+)', e)
    if m and evalnum(m.group(2)) is not None:
        return m.group(1), evalnum(m.group(2))
    return None


# --------------------------------------------------------------------------------------------------------------- definitions
def norm_expr(e):
    """whitespace, pointer / integer casts and redundant parentheses around identifiers / numbers removed"""
    e = re.sub(r'\s+', '', CASTS.sub('', e))
    for _ in range(4):
        e = re.sub(r'\((\w+)\)', r'\1', e)
    return e


def find_accessors(text):
    """accessor macros / tiny inline functions: name -> dict(width, ty, base = argument index or a fixed variable name, off = argument index,
    kind = 'deref' (reads / writes memory) or 'addr' (returns base + off))"""
    acc = {}
    defs = []
    for m in re.finditer(r'^[ \t]*#[ \t]*define[ \t]+(\w+)\(([^)]*)\)[ \t]*((?:[^\n]|\\\n)*)', text, re.M):
        defs.append((m.group(1), [x.strip() for x in m.group(2).split(',') if x.strip()], m.group(3).replace('\\\n', ' ')))
    for rnd in range(2):                                 # twice: a macro may use a macro defined after it
        for name, args, body in defs:
            if name in acc:
                continue
            d = DEREF.search(body)
            if d:
                w, ty = type_width(d.group(1))
                rest = body[d.end():].strip()
                m2 = re.match(r'\(?\s*(\w+)\s*\(([^()]*)\)', rest)
                if m2 and m2.group(1) in acc and acc[m2.group(1)]['kind'] == 'addr':
                    inner = [x.strip() for x in m2.group(2).split(',')]
                    A = acc[m2.group(1)]
                    try:
                        acc[name] = dict(width=w, ty=ty, base=args.index(inner[A['base']]), off=args.index(inner[A['off']]), kind='deref')
                    except (ValueError, IndexError):
                        pass
                    continue
                nb = norm_expr(rest)
                pm = re.search(r'(\w+)\+(\w+)', nb)
                if pm and pm.group(2) in args:
                    b_ = pm.group(1)
                    acc[name] = dict(width=w, ty=ty, base=args.index(b_) if b_ in args else b_, off=args.index(pm.group(2)), kind='deref')
            elif re.search(r'\*\s*\)', body):
                nb = norm_expr(body)
                pm = re.fullmatch(r'\(*(\w+)\+(\w+)\)*', nb)
                if pm and pm.group(1) in args and pm.group(2) in args:
                    acc[name] = dict(width=None, ty='', base=args.index(pm.group(1)), off=args.index(pm.group(2)), kind='addr')
    # tiny inline functions: `static inline T name(const X *p, unsigned off) { ... (const unsigned char *)p + off ... }`
    for m in re.finditer(r'(?:static\s+)?inline\s+([\w\s\*]+?)\s+(\w+)\s*\(([^()]*)\)\s*\{([^{}]*)\}', text):
        ret, name, params, body = m.group(1).strip(), m.group(2), split_args(m.group(3)), m.group(4)
        pn = [re.findall(r'\w+', p_)[-1] for p_ in params if re.findall(r'\w+', p_)]
        nb = norm_expr(body)
        for i, b_ in enumerate(pn):
            for j, o_ in enumerate(pn):
                if i != j and '*' in params[i] and '*' not in params[j] and re.search(r'\b%s\+%s\b' % (re.escape(b_), re.escape(o_)), nb) and re.search(r'\b(int|unsigned|size_t)\b', params[j]):
                    w, ty = type_width(ret)
                    if not w:
                        vt = re.search(r'\b(uint8_t|uint16_t|uint32_t|int8_t|int16_t|int32_t|float|double)\s+(\w+)\s*;', body)
                        w, ty = type_width(vt.group(1)) if vt else (None, '')
                    acc[name] = dict(width=w, ty=ty, base=i, off=j, kind='deref')
    return acc


def functions(text):
    """top-level function bodies: [(start, end, name, signature_text)]"""
    out, depth, i, n, last = [], 0, 0, len(text), 0
    while i < n:
        c = text[i]
        if c == '{':
            if depth == 0:
                sig = re.sub(r'__attribute__\s*\(\(.*?\)\)', ' ', text[last:i], flags=re.S)
                m = re.search(r'([\w:~<>]+)\s*\(([^;{}]*)\)\s*(?:const)?\s*$', sig, re.S)
                j = balanced(text, i, '{', '}')
                if m and not re.search(r'\b(struct|class|enum|union|namespace)\b\s*\w*\s*$', sig):
                    out.append((i, j, m.group(1), m.group(2)))
                i = j
                last = j
                continue
            depth += 1
        elif c == '}':
            depth = max(0, depth - 1)
        elif c == ';' and depth == 0:
            last = i + 1
        i += 1
    return out


# --------------------------------------------------------------------------------------------------------------- layouts
def layout64(c, kinds, widths):
    """x86-64 offsets of the members of a layout class: [(field, off32, size32, off64, size64)], size64"""
    pos, maxal, out = 0, 1, []
    PTR = ('ptr-to-heap', 'ptr-into-heap', 'ptr-to-embedded', 'vptr', 'u32-ptrslot')
    for f in c.fields:
        _, _, sz, al = kinds[f.kind]
        n = max(f.count, 1)
        if f.kind in PTR:
            sz, al = 8, 8
        elif f.kind == 'double' or f.kind == 'align8':
            al = 8
        elif f.kind == 'pad64':
            sz = 1            # x86-64 only: `count` bytes (0 in the i386 layout), layout_gen `uint8_t name[sizeof(void *) == 8 ? count : 0]`
        pos = (pos + al - 1) // al * al
        out.append((f, f.off, kinds[f.kind][2] * n, pos, sz * n))
        pos += sz * n
        maxal = max(maxal, al)
    return out, (pos + maxal - 1) // maxal * maxal


class Layouts:
    def __init__(self, paths):
        import layout_gen as lg
        self.lg = lg
        self.by_name, self.classes = {}, []
        for p in paths:
            with open(p) as fh:
                for c in lg.parse(fh.read(), p):
                    c.src = os.path.basename(p)
                    c.m64, c.size64 = layout64(c, lg.KINDS, None)
                    c.members = [dict(name=f.name, off=f.off, kind=f.kind, w=lg.KINDS[f.kind][2], n=max(f.count, 1), off64=o64)
                                 for (f, o32, s32, o64, s64) in c.m64]
                    for m in c.members:
                        m['end'] = m['off'] + m['w'] * m['n']
                    self.classes.append(c)
                    for k in (c.name, c.cxx, c.mirror, re.sub(r'<.*', '', c.name)):
                        self.by_name.setdefault(re.sub(r'\s+', '', k), c)

    def find(self, t):
        t = re.sub(r'\b(const|struct|volatile)\b', '', t)
        t = re.sub(r'[*&\s]+', '', t)
        return self.by_name.get(t)


def member_at(c, off):
    for m in c.members:
        if m['off'] <= off < m['end']:
            return m
    return None


# --------------------------------------------------------------------------------------------------------------- scanning
DEREF = re.compile(r'\*\s*\(\s*((?:const\s+|volatile\s+)*(?:unsigned\s+|signed\s+)?[A-Za-z_]\w*(?:\s+(?:long|int|char|short))*)\s*\*\s*\)')
INDEX = re.compile(r'\(\s*\(\s*((?:const\s+|volatile\s+)*(?:unsigned\s+|signed\s+)?[A-Za-z_]\w*(?:\s+(?:long|int|char|short))*)\s*\*\s*\)\s*(\w+(?:->\w+)?)\s*\)\s*\[\s*(' + NUM + r')\s*\]')


def line_of(starts, pos):
    import bisect
    return bisect.bisect_right(starts, pos)


def scan_file(path, lay, binds):
    with open(path, errors='replace') as fh:
        raw = fh.read()
    text = blank(raw)
    starts = [0] + [m.end() for m in re.finditer('\n', text)]
    accs = find_accessors(text)
    fns = functions(text)
    rows = []
    defspans = [(m.start(), m.end()) for m in re.finditer(r'^[ \t]*#[ \t]*define\b(?:[^\n]|\\\n)*', text, re.M)]

    def in_define(pos):
        return any(a_ <= pos < b_ for a_, b_ in defspans)

    def ctx(pos):
        for (s, e, name, sig) in fns:
            if s <= pos < e:
                return name, sig, text[s:e]
        return None, '', ''

    def bind(var, name, sig, body):
        v = var.split('->')[0].split('.')[0]
        if var in binds:
            return binds[var], 'bind'
        if v in binds:
            return binds[v], 'bind'
        if v == 'this' and name and '::' in name:
            return re.sub(r'<.*', '', name.rsplit('::', 1)[0]), 'this'
        for src in (sig, body):
            m = re.search(r'\b((?:const\s+)?[A-Za-z_]\w*)\s*\*+\s*(?:const\s+)?%s\b' % re.escape(v), src)
            if m:
                return m.group(1).replace('const', '').strip(), 'decl'
        return None, ''

    def add(pos, base, off, width, ty, how, extra=''):
        name, sig, body = ctx(pos)
        tname, src = bind(base, name, sig, body)
        c = lay.find(tname) if tname else None
        rows.append(dict(file=os.path.basename(path), line=line_of(starts, pos), func=name or '-', base=base, off=off, width=width, ty=ty,
                         how=how, bound=tname or '', bsrc=src, layout=c, note=extra))

    # direct dereferences
    for m in DEREF.finditer(text):
        if in_define(m.start()):
            continue
        w, ty = type_width(m.group(1))
        j = m.end()
        while j < len(text) and text[j] in ' \t\n':
            j += 1
        if j >= len(text):
            continue
        if text[j] == '(':
            k = balanced(text, j)
            ex = text[j:k]
        else:
            mm = re.match(r'(\w+)\s*\(', text[j:])
            if mm and mm.group(1) in accs:                       # *(T *)OBJ_BYTES(o, off)
                k = balanced(text, j + mm.end() - 1)
                args = split_args(text[j + mm.end():k - 1])
                a = accs[mm.group(1)]
                if a['kind'] == 'addr' and len(args) > max(a['base'], a['off']):
                    off = evalnum(args[a['off']])
                    bs = re.sub(r'\s+', '', CASTS.sub('', args[a['base']])).strip('()')
                    if off is not None:
                        add(m.start(), bs, off, w, ty, 'deref+' + mm.group(1))
                    else:
                        add(m.start(), bs, None, w, ty, 'dynamic')
                continue
            mm = re.match(r'\w+', text[j:])
            if not mm:
                continue
            ex = mm.group(0)
            k = j + mm.end()
        # `*(T *)(expr)[...]` / member access after the group: not an offset form
        r = reduce_expr(ex)
        if r is None:
            if '*' in re.sub(r'\*\s*\w+\s*\(', '', ex).replace('*(', '') and not re.search(r'\w\*\w', ex):
                pass
            if re.search(r'\+', CASTS.sub('', ex)) and re.search(r'[A-Za-z_]', CASTS.sub('', ex)):
                add(m.start(), re.sub(r'\s+', '', CASTS.sub('', ex))[:40], None, w, ty, 'dynamic')
            continue
        base, off = r
        if off == 0 and base in ('this',):
            continue
        if off == 0 and '+' not in ex:
            continue                                           # a plain cast of a pointer: not a raw offset access
        add(m.start(), base, off, w, ty, 'deref')
    # word-index forms
    for m in INDEX.finditer(text):
        if in_define(m.start()):
            continue
        w, ty = type_width(m.group(1))
        if w:
            add(m.start(), m.group(2), int(m.group(3), 0) * w, w, ty, 'index')
    # accessor macro / function calls
    for name, a in accs.items():
        if a['kind'] != 'deref':
            continue
        for m in re.finditer(r'\b%s\s*\(' % re.escape(name), text):
            if in_define(m.start()):
                continue
            k = balanced(text, m.end() - 1)
            args = split_args(text[m.end():k - 1])
            if len(args) <= a['off']:
                continue
            off = evalnum(args[a['off']])
            if off is None and re.fullmatch(r'(?:const\s+)?\w+\s+\w+', args[a['off']].strip()):
                continue                                         # the definition of the accessor function itself (`unsigned off`)
            if isinstance(a['base'], int):
                if len(args) <= a['base']:
                    continue
                bs = re.sub(r'\s+', '', CASTS.sub('', args[a['base']])).strip('()')
            else:
                bs = a['base']
            if off is None:
                add(m.start(), bs, None, a['width'], a['ty'], 'dynamic')
            else:
                add(m.start(), bs, off, a['width'], a['ty'], 'macro:' + name)
    # drop the accesses that sit inside the definition of an accessor itself
    rows = [r for r in rows if not (r['func'] == '-' and r['how'] != 'macro:' and False)]
    # de-duplicate (a deref of an accessor call is found twice)
    seen, out = set(), []
    for r in sorted(rows, key=lambda r: (r['line'], str(r['off']))):
        k = (r['line'], r['base'], r['off'], r['width'])
        if k in seen:
            continue
        seen.add(k)
        out.append(r)
    return out, accs


def classify(r):
    """layout lookup of one access -> (member string, flags, x64 offset)"""
    c = r['layout']
    if c is None or r['off'] is None:
        return '', '', ''
    off, w = r['off'], r['width'] or 0
    if off >= c.size:
        return '-', 'OUTSIDE', ''
    m = member_at(c, off)
    if m is None:
        return '-', 'GAP', ''
    delta = off - m['off']
    fl = []
    if m['kind'] == 'unknown-region':
        fl.append('UNK')
    if off + w > m['end']:
        fl.append('STRADDLE')
    elif w and m['kind'] not in ('unknown-region', 'vptr') and w != m['w'] and not (w < m['w'] and (delta % m['w']) % w == 0):
        fl.append('WIDTH')
    x64 = m['off64'] + delta
    # an element of an array member keeps its element stride on x86-64 (pointer arrays double it)
    if m['kind'].startswith('ptr') or m['kind'] == 'vptr':
        x64 = m['off64'] + (delta // 4) * 8 if m['n'] > 1 else m['off64']
    if x64 != off:
        fl.append('x64 WRONG')
    return '%s%s' % (m['name'], '' if not delta else '+%#x' % delta), ' '.join(fl), x64


def mode_source(a):
    files = a.files
    if not files:
        import glob
        # the C++ class files of the P chunks (P01 - P47, the former pNN_*.cpp files): src/dsp/*.cpp + src/vpcm/*.cpp without the V.34 / PCM
        # session layer (V01 - V07, Z01, the getMPrecvdBits alias), which RS4 also put into vpcm/
        session = {'vpcm_info1a_retrain.cpp', 'vpcm_phase34_reneg.cpp', 'vpcm_session_wrappers.cpp', 'vpcm_datapump.cpp', 'vpcm_create.cpp',
                   'vpcm_progress.cpp', 'vpcm_getmp_alias.cpp', 'dp_vpcm.cpp'}
        files = sorted(f for d in ('dsp', 'vpcm') for f in glob.glob(os.path.join(ROOT, 'src', d, '*.cpp')) if os.path.basename(f) not in session)
    layouts = a.layout or sorted(l for l in __import__('glob').glob(os.path.join(ROOT, 'layouts', 'p*.layout')) + __import__('glob').glob(os.path.join(ROOT, 'layouts', '*', '*.layout'))
                                 if os.path.basename(l) not in ('v01.layout', 'v34obj.layout'))   # TESTREORG: descriptions live in layouts/<module>/ (the V.34 object descriptions layouts/v34/v01.layout, v34obj.layout were never in the default table)
    lay = Layouts(layouts)
    binds = {}
    for b in (getattr(a, 'bind', None) or []):
        k, _, v = b.partition('=')
        binds[k] = v
    allrows, accessors = [], {}
    for f in files:
        rows, accs = scan_file(f, lay, binds)
        allrows += rows
        for k, v in accs.items():
            accessors.setdefault(k, (os.path.basename(f), v))
    bound = [r for r in allrows if r['layout'] is not None and r['off'] is not None]
    other = [r for r in allrows if r not in bound]
    hdr = ['file:line', 'function', 'base', 'offset', 'type', 'struct', 'member', 'x86-64 off', 'flags']
    rows_out = []
    for r in bound:
        mem, fl, x64 = classify(r)
        r['member'], r['flags'], r['x64'] = mem, fl, x64
        rows_out.append(['%s:%d' % (r['file'], r['line']), r['func'], r['base'], '%#x' % r['off'], '%s/%s' % (r['ty'], r['width']),
                         r['layout'].name, mem, '%#x' % x64 if x64 != '' else '', fl])
    if a.json:
        import json
        print(json.dumps([{k: v for k, v in r.items() if k != 'layout'} | {'struct': r['layout'].name if r['layout'] else None} for r in allrows],
                         indent=1, default=str))
        return
    import member_map as mm
    print('Mode B: %d files, %d raw-offset accesses found (%d bound to a layout struct, %d unbound or dynamic); accessors found: %s' % (
        len(files), len(allrows), len(bound), len(other), ', '.join('%s(%s)' % (k, v[0]) for k, v in sorted(accessors.items())) or 'none'))
    print()
    print('== accesses bound to a layout struct (the conversion checklist) ==')
    print(mm.table(hdr, rows_out, a.markdown) if rows_out else '(none)')
    print()
    # per file / per struct counts
    pf = defaultdict(lambda: [0, 0, 0])
    for r in allrows:
        pf[r['file']][0] += 1
        if r['layout'] is not None and r['off'] is not None:
            pf[r['file']][1] += 1
            if 'x64 WRONG' in r.get('flags', ''):
                pf[r['file']][2] += 1
    print('== per file: raw accesses / bound to a layout / of those, wrong on x86-64 ==')
    print(mm.table(['file', 'accesses', 'bound', 'x64 WRONG'], [[f] + v for f, v in sorted(pf.items())], a.markdown))
    print()
    ps = defaultdict(lambda: [0, 0, 0, 0, set()])
    for r in bound:
        d = ps[r['layout'].name]
        d[0] += 1
        d[1] += 'x64 WRONG' in r['flags']
        d[2] += 'GAP' in r['flags'] or 'UNK' in r['flags']
        d[3] += 'STRADDLE' in r['flags'] or 'WIDTH' in r['flags']
        d[4].add(r['file'])
    print('== per layout struct: accesses / x64 WRONG / in a gap or unknown-region / straddle or width ==')
    print(mm.table(['struct', 'accesses', 'x64 WRONG', 'gap/unk', 'straddle/width', 'files'],
                   [[k, v[0], v[1], v[2], v[3], ','.join(sorted(v[4]))] for k, v in sorted(ps.items())], a.markdown))
    print()
    print('== x86-64 layout of every layout struct: size and members whose offset moves ==')
    rows = []
    for c in lay.classes:
        moved = [(m['name'], m['off'], m['off64']) for m in c.members if m['off'] != m['off64']]
        has_emb = any(m['kind'] == 'unknown-region' for m in c.members)
        rows.append([c.src, c.name, '%#x' % c.size, '%#x' % c.size64, 'SIZE CHANGES' if c.size != c.size64 else '', len(moved),
                     ('first: %s %#x -> %#x' % moved[0]) if moved else ''])
    print(mm.table(['layout', 'struct', 'i386 size', 'x86-64 size', '', 'members that move', 'first moved member'], rows, a.markdown))
    print('(unknown-region members are assumed size-invariant: an embedded object that holds pointers makes the real x86-64 offsets larger; ptr-to-embedded counts as an 8-byte pointer)')
    print()
    byvar = defaultdict(lambda: defaultdict(int))
    for r in other:
        byvar[r['file']][r['base'] if r['off'] is not None else '(dynamic) ' + r['base']] += 1
    print('== unbound / dynamic accesses (no layout struct for the base variable): per file, top base names; --all lists them ==')
    for f, d in sorted(byvar.items()):
        print('  %-28s %5d   %s' % (f, sum(d.values()), ', '.join('%s x%d' % kv for kv in sorted(d.items(), key=lambda x: -x[1])[:8])))
    if a.all:
        print()
        for r in other:
            print('%s:%d  %s  %s  %s  %s/%s  (%s)' % (r['file'], r['line'], r['func'], r['base'], '%#x' % r['off'] if r['off'] is not None else 'dynamic',
                                                      r['ty'], r['width'], r['how']))
