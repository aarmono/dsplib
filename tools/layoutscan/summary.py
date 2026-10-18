#!/usr/bin/env python3
"""summary.py OUTDIR/out : one line per view pair of pairs.txt (x86-64 DISAGREE count = a named member of the real type at a different x86-64 position;
'opaque' = view leaves inside an opaque byte region of the real type, informational), the same-name conflicts of same_name.txt and the V.34 raw audit total.
Diff two summaries (before / after a fix) to see which object changed."""
import os, re, sys
d = sys.argv[1]
rows = []
cur = None
for line in open(os.path.join(d, 'pairs.txt')):
    m = re.match(r'VIEW (\S+)\s+size i386 (\d+) x64 (\d+) \| REAL (\S+) size i386 (\d+) x64 (\d+) \| base i386 (0x[0-9a-f]+)', line)
    if m:
        cur = m.groups()
        continue
    m = re.match(r'\s+view leaves paired: (\d+), agree at -m64: (\d+), DISAGREE[^:]*: (\d+), differ INSIDE[^:]*: (\d+)', line)
    if m and cur:
        rows.append((cur, m.groups()))
        cur = None
    elif line.startswith('Traceback') or 'Error' in line:
        rows.append((('ERROR', '', '', line.strip()[:80], '', '', ''), ('-', '-', '-', '-')))
print('%-8s %-6s %-6s %-6s  %s' % ('DISAGREE', 'paired', 'agree', 'opaque', 'view -> real (base)  [i386 / x64 sizes]'))
tot = 0
for (v, vs32, vs64, r, rs32, rs64, base), (n, agree, bad, opq) in rows:
    try:
        tot += int(bad)
    except ValueError:
        pass
    print('%-8s %-6s %-6s %-6s  %s -> %s (%s)  [view %s/%s, real %s/%s]' % (bad, n, agree, opq, v, r, base, vs32, vs64, rs32, rs64))
print('total DISAGREE over all pairs: %d' % tot)
for line in open(os.path.join(d, 'same_name.txt')):
    if line.startswith('=='):
        print('same-name ' + line.strip())
    elif line.startswith('  ') and not line.startswith('     '):
        print('   ' + line.strip())
for line in open(os.path.join(d, 'v34_rawaudit.txt')):
    if line.startswith('total'):
        print('v34 raw audit: ' + line.strip())
    elif 'CROSSING' in line or 'NOT COVERED' in line:
        print('   ' + ' '.join(line.split()))
