#!/usr/bin/env python3
"""The DSP library's source list: sources.mk (per-source attributes), read and checked.

  python3 tools/port_objs.py check      every listed source exists and is listed once, the objects are unique, DSP_V04_SRCS is a subset of
                                        DSP_SRCS, and every .c / .cpp below the include root (recursively) is listed.  Exit 0 / 1.
  python3 tools/port_objs.py list       one line per source in LINK ORDER: <path below the repository root> <kind>   (kind = c | cpp | v04)
  python3 tools/port_objs.py incdir     the include root (-I) below the repository root (DSP_INCDIR)

Paths are relative to the repository root.  The list is the single source of truth for what the library builds and with which extra flags
(dsplib.mk includes it); scripts that compile "every library source" use `list` instead of a glob, so a moved file cannot drop out of them silently.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
DSPROOT = os.path.dirname(HERE)                                  # repository root
MKFILE = os.path.join(DSPROOT, 'sources.mk')                     # sources.mk


def parse(path=MKFILE):
    """{variable: [words]} of the simple `NAME := word word \\` assignments of sources.mk."""
    vars_, cur = {}, None
    for raw in open(path):
        line = raw.rstrip('\n')
        if cur is None:
            s = line.strip()
            if not s or s.startswith('#'):
                continue
            if ':=' not in s:
                raise SystemExit('%s: unexpected line: %s' % (path, line))
            name, val = s.split(':=', 1)
            cur = name.strip()
            vars_[cur] = []
            line = val
        cont = line.rstrip().endswith('\\')
        vars_[cur] += line.rstrip().rstrip('\\').split()
        if not cont:
            cur = None
    return vars_


def sources(path=MKFILE):
    """[(path, kind)] in link order."""
    v = parse(path)
    v04 = set(v.get('DSP_V04_SRCS', []))
    out = []
    for s in v.get('DSP_SRCS', []):
        out.append((s, 'v04' if s in v04 else ('cpp' if s.endswith('.cpp') else 'c')))
    return out


def incdir(path=MKFILE):
    return parse(path)['DSP_INCDIR'][0]


def check(path=MKFILE, root=DSPROOT):
    v = parse(path)
    srcs, v04 = v.get('DSP_SRCS', []), v.get('DSP_V04_SRCS', [])
    bad = []
    seen, objs = set(), {}
    for s in srcs:
        if s in seen:
            bad.append('listed twice: ' + s)
        seen.add(s)
        if not s.endswith(('.c', '.cpp')):
            bad.append('not a .c / .cpp source: ' + s)
        if not os.path.isfile(os.path.join(root, s)):
            bad.append('listed but missing: ' + s)
        o = os.path.splitext(s)[0] + '.o'
        if o in objs:
            bad.append('two sources, one object %s: %s %s' % (o, objs[o], s))
        objs[o] = s
    for s in v04:
        if s not in seen:
            bad.append('DSP_V04_SRCS entry not in DSP_SRCS: ' + s)
    inc = os.path.join(root, v['DSP_INCDIR'][0])
    for d, _, fs in os.walk(inc):
        for f in fs:
            if f.endswith(('.c', '.cpp')):
                rel = os.path.relpath(os.path.join(d, f), root)
                if rel not in seen:
                    bad.append('source below %s not listed: %s' % (v['DSP_INCDIR'][0], rel))
    return bad


def lib_files(exts=None, path=MKFILE, root=DSPROOT):
    """every file below the include root (recursively; RS4: src/<module>/...), paths relative to the repository root, sorted bytewise; exts = a tuple of
    extensions to keep ('.c', '.h') or None for all.  For tools that scan "every library source / header"."""
    inc = os.path.join(root, incdir(path))
    out = []
    for d, _, fs in os.walk(inc):
        for f in fs:
            if exts is None or f.endswith(exts):
                out.append(os.path.relpath(os.path.join(d, f), root))
    return sorted(out)


def main(argv):
    if len(argv) < 2 or argv[1] not in ('check', 'list', 'incdir'):
        sys.stderr.write(__doc__)
        return 2
    if argv[1] == 'list':
        for s, k in sources():
            print(s, k)
        return 0
    if argv[1] == 'incdir':
        print(incdir())
        return 0
    bad = check()
    for b in bad:
        print('port_objs: ' + b)
    n = len(sources())
    print('port_objs: %s (%d sources)' % ('OK' if not bad else '%d problem(s)' % len(bad), n))
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
