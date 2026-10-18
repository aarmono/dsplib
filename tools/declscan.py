#!/usr/bin/env python3
"""tools/declscan.py SRCDIR - declaration audit (AUD): list every function prototype / definition at file scope in SRCDIR/**/*.{c,cpp,h,hpp,inc}, group by name and
print the names whose normalised signatures (return type, parameter types, attributes) differ between declarations.  The count goes to stderr.
A regex scan, not a parser: macros such as NOINL are dropped, extern "C" wrappers are looked through, block-scope declarations are not seen."""
import re, sys, os, collections
root = sys.argv[1]
def strip(s):
    s = re.sub(r'/\*.*?\*/', ' ', s, flags=re.S)
    s = re.sub(r'//[^\n]*', ' ', s)
    s = re.sub(r'^\s*#[^\n]*(\\\n[^\n]*)*', ' ', s, flags=re.M)
    s = re.sub(r'"(\\.|[^"\\])*"', '""', s)
    return s
KW = {'if','while','for','switch','return','sizeof','defined'}
def split_top(s):
    depth = 0; cur = []; out = []; stack = []
    i = 0; n = len(s)
    while i < n:
        c = s[i]
        if c == '{':
            head = ''.join(cur)
            if re.search(r'extern\s*""\s*$', head) or (re.search(r'\bnamespace\b[^=]*$', head) and '(' not in head):
                stack.append('transparent'); cur = []
            elif depth == 0:
                out.append((head, '{')); cur = []; depth = 1; stack.append('body')
            else:
                depth += 1; stack.append('body')
        elif c == '}':
            k = stack.pop() if stack else None
            if k == 'body':
                depth -= 1
                if depth == 0: cur = []
            elif k == 'transparent':
                cur = []
        elif depth == 0:
            if c == ';':
                out.append((''.join(cur), ';')); cur = []
            else:
                cur.append(c)
        i += 1
    return out
proto = re.compile(r'^\s*(?P<pre>(?:extern\s*""\s*)?(?:(?:extern|static|inline|const|unsigned|signed|struct|enum|volatile|__attribute__\s*\(\(.*?\)\))\s+)*[A-Za-z_][\w\s\*]*?[\s\*])(?P<name>[A-Za-z_]\w*)\s*\((?P<args>.*)\)\s*(?P<post>(?:__attribute__\s*\(\(.*?\)\)|__asm__\s*\(.*?\))?\s*)$', re.S)
def norm_args(a):
    a = re.sub(r'\s+', ' ', a.strip())
    if a in ('', 'void'): return 'void'
    parts = []; d = 0; cur = ''
    for ch in a:
        if ch in '([': d += 1
        if ch in ')]': d -= 1
        if ch == ',' and d == 0: parts.append(cur); cur = ''
        else: cur += ch
    parts.append(cur)
    res = []
    for p in parts:
        p = p.strip()
        p = re.sub(r'\s*\*\s*', '* ', p).strip()
        m = re.match(r'^(.*?[\s\*])([A-Za-z_]\w*)(\s*\[.*\])?$', p)
        if m and m.group(1).strip() and not re.match(r'^(unsigned|signed|struct|enum|const|int|short|long|char|float|double|void)$', m.group(2)):
            ty = m.group(1).strip() + ('[]' if m.group(3) else '')
        else: ty = p
        res.append(re.sub(r'\s+', ' ', ty).replace('* *','**'))
    return ', '.join(res)
def norm_ret(pre):
    pre = re.sub(r'\b(extern|static|inline)\b', '', pre)
    pre = re.sub(r'extern\s*""', '', pre)
    pre = re.sub(r'""', '', pre)
    pre = re.sub(r'\b[A-Z][A-Z0-9_]*\b', lambda m: m.group(0) if m.group(0) in ('FILE',) else '', pre)
    attrs = re.findall(r'__attribute__\s*\(\(.*?\)\)', pre)
    pre = re.sub(r'__attribute__\s*\(\(.*?\)\)', '', pre)
    pre = re.sub(r'\s*\*\s*', '* ', pre); pre = re.sub(r'\s+', ' ', pre).strip()
    return pre, attrs
db = collections.defaultdict(list)
for dp, dn, fn in os.walk(root):
    for f in fn:
        if not f.endswith(('.c','.cpp','.h','.hpp','.inc')): continue
        p = os.path.join(dp, f)
        s = strip(open(p, errors='replace').read())
        for text, kind in split_top(s):
            t = text.strip()
            if not t or '=' in t.split('(')[0] or t.startswith('typedef') or t.startswith('template'): continue
            tail = re.findall(r'(__attribute__\s*\(\(.*?\)\)|__asm__\s*\(.*?\))\s*$', t)
            t2 = re.sub(r'(__attribute__\s*\(\(.*?\)\)|__asm__\s*\(.*?\))\s*$', '', t) if tail else t
            m = proto.match(t2)
            if not m: continue
            name = m.group('name')
            if name in KW: continue
            ret, attrs = norm_ret(m.group('pre'))
            post = ' '.join(tail)
            attrs += re.findall(r'__attribute__\s*\(\(.*?\)\)', post)
            asm = re.findall(r'__asm__\s*\(.*?\)', post)
            static = bool(re.search(r'\bstatic\b', m.group('pre')))
            db[name].append((ret, norm_args(m.group('args')), tuple(attrs), asm, p[len(root)+1:], 'def' if kind == '{' else 'decl', static))
bad = 0
for name, lst in sorted(db.items()):
    lst = [x for x in lst if not x[6]]
    sigs = set((x[0], x[1], x[2]) for x in lst)
    if len(sigs) > 1:
        bad += 1
        print('==', name)
        for x in sorted(lst, key=lambda x: x[4]): print('   ', x[5], x[4], '|', x[0], '(', x[1], ')', x[2] or '')
print('names with differing signatures:', bad, 'of', len(db), file=sys.stderr)
