#!/usr/bin/env python3
"""same_name.py types64.json types32.json : type names DEFINED in >1 TU whose layout (size + member offsets) differs between TUs (an ODR-style conflict)."""
import json, sys, collections
t64 = json.load(open(sys.argv[1])); t32 = json.load(open(sys.argv[2]))
def sig(t): return (t['size'], tuple(sorted((k, v[0]) for k, v in t['members'].items())))
for label, T in (('-m64', t64), ('-m32', t32)):
    by = collections.defaultdict(dict)
    for tu, types in T.items():
        for n, t in types.items():
            if 'alias_of' in t:     # a typedef alias (RS1): file-local names such as `typedef V90Phase3Demodulator D;` are not record definitions
                continue
            by[n.split('#')[0]].setdefault(sig(t), []).append(tu)
    print('==', label, 'names with >1 distinct layout:', sum(1 for n in by if len(by[n]) > 1))
    for n in sorted(by):
        if len(by[n]) > 1:
            print(' ', n)
            for s, tus in by[n].items():
                print('     size %d, %d members: %s' % (s[0], len(s[1]), ' '.join(sorted(tus))[:300]))
