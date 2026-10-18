#!/usr/bin/env python3
"""layout_check.py - drift check between the committed layouts/**/*.layout descriptions and the files tools/layout_gen.py generates from them.

For every tracked `*.layout` the `# regenerate: tools/layout_gen.py ...` line of its header (the convention of every description; one physical line) is
run with its --out-port redirected into a temporary directory and each output is compared byte for byte with the committed
file it names.  Exit 0 = no drift; 1 = a list of the files that differ (fix the .layout / the generator / the regenerate line, never the generated file by hand).
Called by `tools/gen_views.sh --check`.  Run from anywhere; ~2 s.  

Skipped on purpose:
  layouts/v34/v01.layout     its outputs need --paths layouts/v34/v34obj_paths.txt (the regenerate line is the plain one): gen_views.sh generates and checks them itself
  layouts/v34/v34obj.layout  not an input of layout_gen.py (its `regenerate` comment names layouts/v34/v01.layout)
ALLOW lists outputs whose drift is known and left alone, one comment each (empty since TESTREORG6).
"""
import os
import re
import subprocess
import sys
import tempfile

SKIP = {'layouts/v34/v01.layout', 'layouts/v34/v34obj.layout'}
ALLOW = {}  # committed output path -> why its drift is tolerated


def main():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    os.chdir(root)
    lays = subprocess.check_output(['git', 'ls-files', '*.layout'], text=True).split()
    bad = []
    n = 0
    with tempfile.TemporaryDirectory(prefix='layout_check.') as tmp:
        for l in lays:
            if l in SKIP:
                continue
            m = re.search(r'^#\s*regenerate[^:\n]*:\s*(tools/layout_gen\.py[^\n]*)$', open(l).read(), re.M)
            if not m:
                bad.append('%s: no `# regenerate: tools/layout_gen.py ...` line' % l)
                continue
            args = m.group(1).split()[1:]
            outs = []
            for k, a in enumerate(args):
                if a == '--out-port':
                    outs.append(args[k + 1])
                    args[k + 1] = os.path.join(tmp, args[k + 1].replace('/', '__'))
            r = subprocess.run([sys.executable, 'tools/layout_gen.py'] + args, capture_output=True, text=True)
            if r.returncode:
                bad.append('%s: layout_gen failed: %s' % (l, r.stderr.strip()[-200:]))
                continue
            for o in outs:
                n += 1
                g = os.path.join(tmp, o.replace('/', '__'))
                if not os.path.exists(o) or open(o, 'rb').read() != open(g, 'rb').read():
                    if o not in ALLOW:
                        bad.append('%s: %s differs from the output of its regenerate line' % (l, o))
    if bad:
        print('layout_check: drift between .layout files and their generated outputs:', file=sys.stderr)
        for b in bad:
            print('  ' + b, file=sys.stderr)
        sys.exit(1)
    print('layout_check: %d generated layout files match their .layout descriptions' % n)


if __name__ == '__main__':
    main()
