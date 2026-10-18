#!/usr/bin/env python3
"""find_views.py types32.json types64.json : candidate VIEW pairs = two differently named record types that share >= MIN member names at the SAME i386 offset
(mirrors / prefixes / re-typed copies of one blob object).  For each pair: i386 sizes, the TUs that emit each, and at -m64 the count of common members whose
offsets disagree + the size delta."""
import json, sys, collections, itertools
t32 = json.load(open(sys.argv[1])); t64 = json.load(open(sys.argv[2]))
MIN = int(sys.argv[3]) if len(sys.argv) > 3 else 3
def merge(T):
    d = {}; tus = collections.defaultdict(set)
    for tu, types in T.items():
        for n, t in types.items():
            n0 = n.split('#')[0]
            if n0 == 'VPcmFloModem' and tu == 'vpcm_flo_modem_run': n0 = 'VPcmFloModem[p46]'
            d.setdefault(n0, t); tus[n0].add(tu)
    return d, tus
d32, tus = merge(t32); d64, _ = merge(t64)
# index member (name, off) -> types, only member names that are not generic
idx = collections.defaultdict(set)
for n, t in d32.items():
    for m, v in t['members'].items():
        if len(m) >= 3:
            idx[(m, v[0])].add(n)
cnt = collections.Counter()
for k, ns in idx.items():
    if len(ns) > 40: continue  # generic member (e.g. 'state' at 0)
    for a, b in itertools.combinations(sorted(ns), 2):
        cnt[(a, b)] += 1
rows = []
for (a, b), c in cnt.items():
    if c < MIN: continue
    A, B = d32[a], d32[b]
    common = [m for m in A['members'] if m in B['members'] and A['members'][m][0] == B['members'][m][0]]
    frac = c / max(1, min(len(A['members']), len(B['members'])))
    if a not in d64 or b not in d64: continue
    A6, B6 = d64[a], d64[b]
    dis = [m for m in common if m in A6['members'] and m in B6['members'] and A6['members'][m][0] != B6['members'][m][0]]
    rows.append((a, b, c, frac, A['size'], B['size'], A6['size'], B6['size'], len(dis), dis[:6]))
rows.sort(key=lambda r: (-r[3], -r[2]))
for r in rows:
    if r[3] < 0.5 and r[2] < 10: continue
    print('%-28s %-28s common=%-4d frac=%.2f i386 %d/%d  x64 %d/%d  x64-disagree=%d %s' % r)
