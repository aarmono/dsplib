#!/bin/sh
# tools/check_generated.sh - one command for "everything generated is up to date and the tree is consistent": exit 0 = all pass, 1 = the first failure is shown.
#   source list (sources.mk), generated V.34 views + layout asserts (gen_views.sh --check), enum headers, clang-format, declarations.
# hostiface_check.sh is not run here: it needs a host's headers (HOST_INCDIR).
cd "$(dirname "$0")/.." || exit 2
rc=0
step() { printf '%-34s' "$1"; shift; if out=$("$@" 2>&1); then echo ok; else echo FAIL; echo "$out" | tail -8; rc=1; fi; }
step "sources.mk"              python3 tools/port_objs.py check
step "gen_views.sh --check"    sh tools/gen_views.sh --check
step "v34_hs_states_gen.py"    python3 tools/v34_hs_states_gen.py --check
step "v8_status_gen.py"        python3 tools/v8_status_gen.py --check
step "format.sh --check src"   sh tools/format.sh --check src
step "declscan.py src"         python3 tools/declscan.py src
exit $rc
