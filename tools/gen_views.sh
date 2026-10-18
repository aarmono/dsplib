#!/bin/sh
# gen_views.sh - regenerate (or, with --check, verify) the generated C views of the V.34 chain: src/v34/v34_*_view.h and v34_subobjects.h, from the layout
# descriptions layouts/v34/v34obj.layout (the V.34 object), layouts/v34/v01.layout, layouts/vpcm/pcm_parameters.layout, layouts/vpcm/vpcm_flo_modem.layout.
# Run from anywhere; a few seconds.  Structs generated with --tag ARE the C++ type of the same name: src/v34/v34_object.h includes them instead of
# declaring the object a second time.
# layouts/v34/v34obj.layout is maintained by hand (new members: tools/split_region.py).  With --check every other layouts/**/*.layout is verified, too:
# its generated src/ file must be what its `# regenerate:` line produces (tools/layout_check.py; no write mode: edit the .layout / run the line).
cd "$(dirname "$0")/.." || exit 1
CHECK=; [ "$1" = "--check" ] && CHECK=--check
TR="python3 tools/tier_rewrite.py"
set -e
$TR header --cxx layouts/v34/v34obj_cxx.txt --tag tagV34Object -o src/v34/v34_object_view.h $CHECK
$TR slice $CHECK
$TR paths $CHECK
# the x86-64 layout asserts of the object (compiled probe, gcc -m64): src/v34/v34_object_layout64.inc, included at the end of v34_object_view.h
python3 tools/layout64.py $CHECK
# the C++ class's offsetof asserts name the members of a sole sub-object by their path (layouts/v34/v34obj_paths.txt)
LG="python3 tools/layout_gen.py layouts/v34/v01.layout --chunk v01 --paths layouts/v34/v34obj_paths.txt --port-header v34/v34_object.h"
if [ -n "$CHECK" ]; then
  LGT=$(mktemp -d "${TMPDIR:-/tmp}/gen_views.XXXXXX")
  $LG --out-port "$LGT/p.inc" >/dev/null
  if cmp -s "$LGT/p.inc" src/v34/v34_object_layout.inc; then echo "layout_gen output up to date"; else rm -r "$LGT"; echo "layout_gen output src/v34/v34_object_layout.inc differs from the freshly generated one" >&2; exit 1; fi
  rm -r "$LGT"
else
  $LG --out-port src/v34/v34_object_layout.inc
fi
[ -n "$CHECK" ] && python3 tools/layout_check.py
$TR header --layout layouts/vpcm/vpcm_flo_modem.layout --class VPcmFloModem --prefix VPFM_ --guard V34CTX_H -o src/v34/v34_flomodem_view.h $CHECK
$TR header --layout layouts/v34/v01.layout --class K56FlexHandle --prefix V34K56_ --guard V34K56_H --tag K56FlexHandle -o src/v34/v34_k56flex_view.h $CHECK
$TR header --layout layouts/vpcm/pcm_parameters.layout --class V90Parameters --prefix V34CFG_ --guard V34CFG_H -o src/v34/v34_v90params_view.h $CHECK
$TR header --layout layouts/vpcm/vpcm_flo_modem.layout --class V90Phase2InfoView --prefix V34V90I_ --guard V34V90I_H -o src/v34/v34_v90_phase2_view.h $CHECK
$TR header --layout layouts/vpcm/vpcm_flo_modem.layout --class V92Phase2InfoView --prefix V34V92I_ --guard V34V92I_H -o src/v34/v34_v92_phase2_view.h $CHECK
