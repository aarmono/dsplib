#!/usr/bin/env python3
"""v34_rawaudit.py : every raw byte access `obj->member + K` / ld*/st*(obj->member, K) / constPts_ + K in the V.34 C++ files (the session layer src/vpcm/vpcm_*.cpp of V01 - V07, the handshake pieces src/v34/handshake/*.cpp, src/vpcm/dp_vpcm.cpp)
on the shared V.34 object.  i386 target = offsetof(tagV34Object, member) + K; the C tiers (V34Obj, the canonical x86-64 layout of the object) keep that byte
at x64(V34Obj leaf covering the target) + delta.  The C++ access lands at x64(tagV34Object member) + K.  Equal -> consistent; else CROSSING."""
import json, re, os, sys
# usage (cwd = the scan's OUTDIR, which holds hdrtypes32/64.json): v34_rawaudit.py INCROOT   (the library's include root, src/)
P = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'src')
SESSION = ('vpcm_info1a_retrain.cpp', 'vpcm_phase34_reneg.cpp', 'vpcm_session_wrappers.cpp', 'vpcm_datapump.cpp', 'vpcm_create.cpp', 'vpcm_progress.cpp',
           'dp_vpcm.cpp')
FILES = sorted([os.path.join('vpcm', f) for f in SESSION] +
               [os.path.join('v34', 'handshake', f) for f in os.listdir(os.path.join(P, 'v34', 'handshake')) if f.endswith('.cpp')])
h32 = json.load(open('hdrtypes32.json')); h64 = json.load(open('hdrtypes64.json'))
c32 = h32['h_v34_object']['tagV34Object']['members']; c64 = h64['h_v34_object']['tagV34Object']['members']
sys.argv = ['x']; exec(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'cmp_by_offset.py')).read().split('def main():')[0])
v32, _ = leaves('hdrtypes:h_v34_object_view:V34Obj', '32'); v64, _ = leaves('hdrtypes:h_v34_object_view:V34Obj', '64')
v64m = dict((p, o) for o, p, k, s in v64)
rl = sorted(x for x in v32 if x[3] != 0)
def cover(off):
    best = None
    for o, p, k, s in rl:
        if o <= off < o + (s if s else 1): best = (o, p, k, s)
        if o > off: break
    return best
pat = [re.compile(r'\bobj->(\w+_)\s*\+\s*(0x[0-9a-fA-F]+|\d+)\b'), re.compile(r'\b(?:ld|st)\w*\(\s*obj->(\w+_)\s*,\s*(0x[0-9a-fA-F]+|\d+)\b')]
tot = ok = 0
for fn in FILES:
    for ln, line in enumerate(open(os.path.join(P, fn)), 1):
        code = re.sub(r'/\*.*?\*/', '', line.split('//')[0])
        if code.strip().startswith('*') or code.strip().startswith('/*'): continue
        sites = []
        for pt in pat:
            for m in pt.finditer(code):
                mem, k = m.group(1), int(m.group(2), 0)
                sites.append((mem, k, k))
        # RS1: `obj->member + (offsetof(tagV34Object, X) - offsetof(tagV34Object, member))` (the M4 spelling): K is the distance to X on each host
        for m in re.finditer(r'\bobj->(\w+_)\s*\+\s*\(\s*offsetof\(\s*tagV34Object\s*,\s*(\w+)\s*\)\s*-\s*offsetof\(\s*tagV34Object\s*,\s*(\w+)\s*\)\s*\)', code):
            mem, x, y = m.group(1), m.group(2), m.group(3)
            if x in c32 and y in c32:
                sites.append((mem, c32[x][0] - c32[y][0], c64[x][0] - c64[y][0]))
        for mem, k, k64 in sites:
                if mem not in c32 or not c32[mem][1].startswith('arr'): continue
                t = c32[mem][0] + k
                c = cover(t)
                if c is None:
                    print('%-26s %4d %s+0x%x i386 0x%05x NOT COVERED' % (fn, ln, mem, k, t)); tot += 1; continue
                cxx64 = c64[mem][0] + k64
                exp = v64m[c[1]] + t - c[0]
                tot += 1
                st = 'ok' if exp == cxx64 else 'CROSSING'
                if exp == cxx64: ok += 1
                print('%-26s %4d %-28s i386 0x%05x  C-tier leaf %-14s x64 0x%05x | C++ x64 0x%05x  %s' % (fn, ln, '%s+0x%x' % (mem, k), t, c[1], exp, cxx64, st))
print('total %d, consistent %d, crossing %d' % (tot, ok, tot - ok))
