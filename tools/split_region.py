#!/usr/bin/env python3
"""split_region.py SPEC - split unknown regions of the V.34 sub-objects into typed members and rewrite the plain-C sites that access them.

    python3 tools/split_region.py SPEC.py       (from the repository root, on a clean tree; then `sh tools/gen_views.sh`)

SPEC.py defines SPECS = [dict(struct='V34Rx', reg='unk0424_', abs=0x424, size=12, members=[(rel, 'name', kind, count), ...]), ...]:
  struct/reg/abs/size  the region as layouts/v34/v34obj.layout has it (`  0x0424 unknown-region unk0424_ 12`)
  members              byte offset inside the region, member name, kind i16 u16 i32 u32, element count (> 1: array)
  forms                optional: other spellings of the region in `offsetof(S, path)`: [(struct, path, cast struct, member prefix[, byte bias])]
                       (V34Mod: `pf.unk20d0_`; a bias moves the sites that name a neighbouring region, e.g. `unk0488_ + 14`)
What it does: (1) layouts/v34/v34obj.layout: the region line becomes members + the remaining gaps (unkXXXX_ regions; 32-bit offsets unchanged); (2) layouts/v34/v34obj_cxx.txt:
`type` lines for the int16_t kinds and a zero-size `name` marker when layouts/v34/v01.layout still has the region name (the C++ class asserts it); (3) the accessor
sites S16/U16/S32/U32/I32(base, (int)offsetof(S, region) + N [+/- K * idx]) and the key sites S16T_rx(b, 0x1c0) ... in src/**/*.c, *.cpp become
`((S*)base)->member[idx]` (`AS_U16(...)` for an unsigned access of an int16_t member; a wide or misaligned access keeps its accessor at offsetof(member) + delta;
`((S*)b)->` becomes the local `S* x = (S*)b;` when the function has one).  Sites it cannot place are printed as REPORT lines (walks with another stride, indexes
into scalars): fix them by hand.  Not rewritten: OBJ_BYTES / V34OFFI uses (memset of a region, pointers into it), C++ ld16 / st16 uses and comments.
It does not run gen_views.sh and is not idempotent for the layout step.
"""
import re, sys, glob, os

KIND = {'i16': ('u16', 'int16_t', 2), 'u16': ('u16', 'uint16_t', 2), 'i32': ('i32', 'int32_t', 4), 'u32': ('u32', 'uint32_t', 4)}
ACC = {'S16': ('i16', 2), 'U16': ('u16', 2), 'S32': ('i32', 4), 'I32': ('i32', 4), 'U32': ('u32', 4)}

LAYOUT = 'layouts/v34/v34obj.layout'
CXX = 'layouts/v34/v34obj_cxx.txt'


def gaps(spec):
    """list of (rel, size) gaps and members sorted"""
    mem = sorted(spec['members'])
    pos = 0
    out = []
    for rel, name, kind, cnt in mem:
        es = KIND[kind][2]
        if rel < pos:
            sys.exit('overlap at %s' % name)
        if rel > pos:
            out.append(('gap', pos, rel - pos))
        out.append(('mem', rel, (name, kind, cnt)))
        pos = rel + es * cnt
    if pos > spec['size']:
        sys.exit('members exceed region %s' % spec['reg'])
    if pos < spec['size']:
        out.append(('gap', pos, spec['size'] - pos))
    return out


def edit_layout(specs):
    txt = open(LAYOUT).read()
    cxx = open(CXX).read()
    v01 = open('layouts/v34/v01.layout').read()
    alladd = ''
    for s in specs:
        old = '  0x%04x unknown-region %s %d\n' % (s['abs'], s['reg'], s['size'])
        if old not in txt:
            sys.exit('layout line not found: ' + old)
        new = ''
        first_is_gap = False
        for kind, rel, v in gaps(s):
            a = s['abs'] + rel
            if kind == 'gap':
                new += '  0x%04x unknown-region unk%04x_ %d\n' % (a, a, v)
                if rel == 0:
                    first_is_gap = True
            else:
                name, k, cnt = v
                new += '  0x%04x %s %s%s\n' % (a, KIND[k][0], name, (' %d' % cnt) if cnt > 1 else '')
        txt = txt.replace(old, new)
        # cxx types
        add = ''
        for rel, name, k, cnt in sorted(s['members']):
            if k == 'i16':
                add += 'type %s int16_t%s\n' % (name, ('[%d]' % cnt) if cnt > 1 else '')
        # marker for a region name that v01.layout still has and that no longer exists
        keeps = first_is_gap
        if not keeps and re.search(r'unknown-region %s ' % re.escape(s['reg']), v01):
            if ('name %s ' % s['reg']) not in cxx:
                add += 'name %s uint8_t 0x%x\n' % (s['reg'], s['abs'])
        alladd += add
    if alladd:
        key = '# --- records (anonymous union over the flat members)'
        hdr = '# --- members split out of unknown regions (tools/split_region.py): signed 16-bit words, and the markers that keep the names layouts/v34/v01.layout asserts\n'
        if hdr in cxx:
            cxx = cxx.replace(key, alladd + '\n' + key, 1)
        else:
            cxx = cxx.replace(key, hdr + alladd + '\n' + key, 1)
    open(LAYOUT, 'w').write(txt)
    open(CXX, 'w').write(cxx)


# ---------------------------------------------------------------- site rewriting
def match_paren(t, i):
    """t[i] == '(' -> index of the matching ')'"""
    d = 0
    for j in range(i, len(t)):
        if t[j] == '(':
            d += 1
        elif t[j] == ')':
            d -= 1
            if d == 0:
                return j
    return -1


def split_args(a):
    out, d, cur = [], 0, ''
    for ch in a:
        if ch == '(':
            d += 1
        elif ch == ')':
            d -= 1
        if ch == ',' and d == 0:
            out.append(cur)
            cur = ''
        else:
            cur += ch
    out.append(cur)
    return out


class Region:
    def __init__(self, s):
        self.s = s
        self.struct = s['struct']
        self.reg = s['reg']
        self.items = gaps(s)
        self.forms = s.get('forms', [(self.struct, self.reg, self.struct, '')])

    def find(self, n):
        for kind, rel, v in self.items:
            if kind == 'mem':
                name, k, cnt = v
                es = KIND[k][2]
                if rel <= n < rel + es * cnt:
                    return ('mem', rel, name, k, cnt, es)
            else:
                if rel <= n < rel + v:
                    return ('gap', rel, 'unk%04x_' % (self.s['abs'] + rel), None, 1, 1)
        return None


def parse_rest(rest, es):
    """rest after the offset: '' | ' + K * expr' | ' - K * expr' | ' + expr * K' -> (sign, expr) in element units or None"""
    rest = rest.strip()
    if rest == '':
        return (1, None)
    m = re.match(r'^([+-])\s*(\d+|0x[0-9a-fA-F]+)\s*\*\s*(.+)$', rest)
    if m and int(m.group(2), 0) == es:
        return (1 if m.group(1) == '+' else -1, m.group(3).strip())
    m = re.match(r'^([+-])\s*(.+?)\s*\*\s*(\d+)$', rest)
    if m and int(m.group(3), 0) == es:
        return (1 if m.group(1) == '+' else -1, m.group(2).strip())
    return 'bad'


def paren(e):
    e = e.strip()
    if re.match(r'^[A-Za-z_]\w*$', e) or re.match(r'^\d+$', e):
        return e
    if e.startswith('(') and match_paren(e, 0) == len(e) - 1:
        return e
    return '(' + e + ')'


def inner(e):
    e = e.strip()
    if e.startswith('(') and match_paren(e, 0) == len(e) - 1:
        return e[1:-1].strip()
    return e


def rewrite_text(text, regions, fname, report):
    out = ''
    i = 0
    pat = re.compile(r'\b(S16|U16|S32|U32|I32)\(')
    while True:
        m = pat.search(text, i)
        if not m:
            out += text[i:]
            break
        op = m.end() - 1
        cl = match_paren(text, op)
        if cl < 0:
            out += text[i:m.end()]
            i = m.end()
            continue
        args = split_args(text[op + 1:cl])
        if len(args) != 2:
            out += text[i:m.end()]
            i = m.end()
            continue
        base, off = args[0].strip(), args[1].strip()
        hit = None
        for r in regions:
            for form in r.forms:
                fs, fr = form[0], form[1]
                bias = form[4] if len(form) > 4 else 0
                mm = re.match(r'^(?:\(\(int\)offsetof\(%s,\s*%s\)\s*\+\s*\((\d+)\)\)|\(int\)offsetof\(%s,\s*%s\))(.*)$' % (fs, re.escape(fr), fs, re.escape(fr)), off, re.S)
                if mm:
                    n0 = (int(mm.group(1)) if mm.group(1) else 0) + bias
                    rest0 = mm.group(2)
                    mc0 = re.match(r'^\s*\+\s*(\d+)\b(?!\s*\*)(.*)$', rest0, re.S)
                    if mc0:
                        n0 += int(mc0.group(1))
                        rest0 = mc0.group(2)
                    if not (0 <= n0 < r.s['size']):
                        continue
                    hit = (r, n0, rest0, form)
                    break
            if hit:
                break
        if not hit:
            out += text[i:m.end()]
            i = m.end()
            continue
        r, n, rest, form = hit
        fs, fr, fcast, fpre = form[:4]
        frp = fr.rsplit('.', 1)[0] + '.' if '.' in fr else ''
        f = r.find(n)
        acc = m.group(1)
        if f is None:
            report.append('%s: offset %d outside region: %s' % (fname, n, text[m.start():cl + 1]))
            out += text[i:m.end()]
            i = m.end()
            continue
        kind, rel, name, k, cnt, es = f
        if kind == 'gap':
            # rebase onto the remaining gap region
            nn = n - rel
            rst = rest
            newoff = '(int)offsetof(%s, %s%s)' % (fs, frp, name) + ((' + %d' % nn) if nn else '')
            if rst.strip():
                newoff = '(' + newoff + ')' + rst if nn else newoff + rst
            repl = '%s(%s, %s)' % (acc, base, newoff)
            if nn == 0 and not rst.strip():
                repl = '%s(%s, (int)offsetof(%s, %s%s))' % (acc, base, fs, frp, name)
            out += text[i:m.start()] + repl
            i = cl + 1
            continue
        if ACC[acc][1] != es or (n - rel) % es:
            if rest.strip() or (cnt > 1 and False):
                report.append('%s: wide/misaligned walk: %s' % (fname, text[m.start():cl + 1]))
                out += text[i:m.end()]
                i = m.end()
                continue
            dl = n - rel
            offs = '(int)offsetof(%s, %s%s)' % (fs, fpre, name) + ((' + %d' % dl) if dl else '')
            out += text[i:m.start()] + '%s(%s, %s)' % (acc, base, offs)
            i = cl + 1
            continue
        pr = parse_rest(rest, es)
        if pr == 'bad':
            report.append('%s: unhandled walk: %s' % (fname, text[m.start():cl + 1]))
            out += text[i:m.end()]
            i = m.end()
            continue
        sign, expr = pr
        c0 = (n - rel) // es
        if (n - rel) % es:
            report.append('%s: misaligned access into %s: %s' % (fname, name, text[m.start():cl + 1]))
            out += text[i:m.end()]
            i = m.end()
            continue
        if cnt == 1 and expr is None and c0 == 0:
            idx = ''
        elif expr is None:
            idx = '[%d]' % c0
        else:
            e = inner(expr)
            if sign > 0:
                idx = '[%s]' % (e if c0 == 0 else '%s + %d' % (e, c0))
            else:
                idx = '[%d - %s]' % (c0, paren(e)) if re.match(r'^[\w]+$', e) else '[%d - (%s)]' % (c0, e)
        if cnt == 1 and idx:
            report.append('%s: index into scalar %s: %s' % (fname, name, text[m.start():cl + 1]))
            out += text[i:m.end()]
            i = m.end()
            continue
        lv = '\x01((%s*)%s)->%s%s%s' % (fcast, base, fpre, name, idx)
        after = text[cl + 1:cl + 8]
        islv = re.match(r'^\s*(=[^=]|\+=|-=|\|=|&=|\^=|<<=|>>=)', after) is not None
        macc, _ = ACC[acc]
        if macc == k:
            repl = lv
        elif acc == 'U16' and k == 'i16':
            repl = 'AS_U16(%s)' % lv
        elif acc == 'S16' and k == 'u16':
            repl = lv if islv else '(int16_t)%s' % lv
        elif k in ('i32', 'u32') and acc in ('S32', 'I32', 'U32'):
            if islv:
                repl = lv
            else:
                repl = '(%s)%s' % ('uint32_t' if acc == 'U32' else 'int32_t', lv)
        else:
            repl = lv
        out += text[i:m.start()] + repl
        i = cl + 1
    return out


BASES = {'V34Obj': ('obj', 0), 'V34Info': ('info', 0xa9ac), 'V34Tx': ('tx', 0x221c), 'V34Rx': ('rx', 0x264), 'V34ToneDet': ('det', 0x3564), 'V34Rate': ('rate', 0xaa84), 'V34BlkB': ('blkb', 0xa00), 'V34Mod': ('mod', 0x1450),
         'V34PreFilter': ('pf', 0x2078), 'V34Probe': ('probe', 0xa320)}


def rewrite_keys(text, regions, fname, report):
    """S16T_rx(b, 0x1c0) style key sites whose key is the start of a new member (or an element of a new array) -> plain member access"""
    keymap = {}
    for r in regions:
        prefix, kb = BASES[r.struct]
        for kind, rel, v in r.items:
            if kind != 'mem':
                continue
            name, k, cnt = v
            es = KIND[k][2]
            for e in range(cnt):
                key = r.s['abs'] + rel + e * es - kb
                keymap[(prefix, key)] = (r.struct, name, k, cnt, e)
            keymap[(prefix, name[:-1])] = (r.struct, name, k, cnt, 0)
    pat = re.compile(r'\b(S16|U16|S32|U32|I32)T_(rx|det|mod|rate|blkb|pf|probe|info)\(\s*(\w+),\s*(\w+)\s*\)')

    def sub(m):
        acc, pre, base, key = m.groups()
        kv = int(key, 16) if key.startswith('0x') else key
        hit = keymap.get((pre, kv))
        if not hit:
            return m.group(0)
        st, name, k, cnt, e = hit
        if ACC[acc][1] != KIND[k][2]:
            return m.group(0)
        idx = '' if cnt == 1 else '[%d]' % e
        lv = '\x01((%s*)%s)->%s%s' % (st, base, name, idx)
        macc, _ = ACC[acc]
        islv = re.match(r'^\s*(=[^=]|\+=|-=)', text[m.end():m.end() + 8]) is not None
        if macc == k:
            return lv
        if acc == 'U16' and k == 'i16':
            return 'AS_U16(%s)' % lv
        if acc == 'S16' and k == 'u16':
            return lv if islv else '(int16_t)%s' % lv
        if islv:
            return lv
        return '(%s)%s' % ('uint32_t' if acc == 'U32' else 'int32_t', lv)
    return pat.sub(sub, text)


def localize(text, structs):
    """((S*)X)->m -> NAME->m when the function declares `S* NAME = (S*)X;` before the site"""
    out = ''
    pos = 0
    for st in structs:
        pat = re.compile(r'\x01\(\(%s\*\)(\w+)\)->' % st)
        res = ''
        last = 0
        for m in pat.finditer(text):
            x = m.group(1)
            decl = re.compile(r'\b%s\s*\*\s*(\w+)\s*=\s*\(%s\s*\*\)\s*%s\s*;' % (st, st, x))
            # nearest function start: last "\n}" before the site
            fs = text.rfind('\n}', 0, m.start())
            seg = text[fs + 1 if fs >= 0 else 0:m.start()]
            ds = list(decl.finditer(seg))
            if ds:
                res += text[last:m.start()] + ds[-1].group(1) + '->'
                last = m.end()
        text = res + text[last:] if res else text
    return text.replace('\x01', '')


def rewrite_files(regions):
    files = []
    for pat in ('src/**/*.c', 'src/**/*.cpp'):
        files += glob.glob(pat, recursive=True)
    report = []
    changed = []
    for f in sorted(files):
        t = open(f, errors='surrogateescape').read()
        if not any(r.reg in t for r in regions) and 'T_' not in t:
            continue
        n = rewrite_text(t, regions, os.path.basename(f), report)
        n = rewrite_keys(n, regions, os.path.basename(f), report)
        n = localize(n, sorted(set(fm[2] for r in regions for fm in r.forms)))
        if n != t:
            open(f, 'w', errors='surrogateescape').write(n)
            changed.append(f)
    return changed, report


def main():
    specfile = sys.argv[1]
    ns = {}
    exec(open(specfile).read(), ns)
    specs = ns['SPECS']
    edit_layout(specs)
    regions = [Region(s) for s in specs]
    changed, report = rewrite_files(regions)
    print('changed files:', ' '.join(os.path.basename(c) for c in changed))
    for r in report:
        print('REPORT', r)


if __name__ == '__main__':
    main()
