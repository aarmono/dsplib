#!/bin/sh
# tools/layoutscan/scan_layouts.sh [OUTDIR] - the layout scan of src/ (adapted from a read-only scan,
# "layout-scan.md"): finds objects that are described by more than one type and checks, member by member, that the descriptions agree on x86-64.
# Writes ONLY under OUTDIR (default ${TMPDIR:-/tmp}/layoutscan-$USER); reads the port sources of the checkout this script lives in.  Steps:
#   1. every port TU with the production recipe (-O2 -fno-strict-aliasing -w -fno-stack-protector -fno-pie -Isrc, C++ -fno-exceptions -fno-rtti,
#      the DSP_V04_SRCS of sources.mk + V04_DEFS) plus -gdwarf-4 (debug info does not change layout), -m32 and -m64                                    -> OUTDIR/obj32g obj64g
#   2. header probes: one TU per view header with -fno-eliminate-unused-debug-types (types used only through offsetof leave no DWARF otherwise) -> probes/hdr32 hdr64
#   3. dwarf_types.py: every struct / class / union (+ typedef aliases of records) with size and member offsets                -> types*.json hdrtypes*.json
#   4. same_name.py: same type name, different layout between TUs (an ODR-style conflict)                                       -> out/same_name.txt
#   5. find_views.py: differently named types sharing >= 3 member names at the same i386 offset (candidate views; heuristic)     -> out/views_auto.txt
#   6. cmp_by_offset.py for the enumerated view pairs (leaf by leaf, paired by i386 offset, x86-64 offsets compared)            -> out/pairs.txt
#      (the v34_engine_views.h prefix views of v34_api.c are among the pairs; V34ENG_PAIRS="view:tu:Real ..." overrides that list)
#   8. v34_rawaudit.py: raw `obj->member + K` accesses of the V.34 C++ files vs the C tiers' x86-64 layout                      -> out/v34_rawaudit.txt
#   9. summary.py: one line per pair (DISAGREE counts) + same-name conflicts                                                      -> out/summary.txt (also printed)
# SKIP_BUILD=1 reuses the objects.  Not part of any test job; read-only on the repository.
set -e
S=$(cd "$(dirname "$0")" && pwd)
OUT=${1:-${TMPDIR:-/tmp}/layoutscan-${USER:-u}}
# RS4: the library is src/<module>/ (sources.mk lists the TUs); objects are named by the file's stem (unique), header probes h_<stem>.
D=$(cd "$S/../.." && pwd)                          # the repository root of this checkout
P=$(cd "${PORT:-$D/$(python3 "$S/../port_objs.py" incdir)}" && pwd)   # PORT=<dir>: the include root of another tree with the RS4 layout
mkdir -p "$OUT/out" "$OUT/probes/hdr"
PF="-O2 -fno-strict-aliasing -w -fno-stack-protector -fno-pie -I$P -gdwarf-4"
V04="-DV04_REAL_HELPERS -Dgetbit=v04_getbit_impl -DApplyBulkDelay=v04_apply_bulk_delay_impl"
SRCLIST=$(cd "$D" && python3 tools/port_objs.py list)   # "path kind" (kind c | cpp | v04), paths below the repository root, in link order
cd "$OUT"
if [ -z "$SKIP_BUILD" ]; then
  for B in 32 64; do
    rm -rf obj${B}g; mkdir -p obj${B}g
    echo "$SRCLIST" | while read f k; do
      f=$(dirname "$P")/$f; b=$(basename "$f"); o=obj${B}g/${b%.*}.o
      case $k in
        v04) echo "g++ -m$B $PF -fno-exceptions -fno-rtti $V04 -c $f -o $o";;
        cpp) echo "g++ -m$B $PF -fno-exceptions -fno-rtti -c $f -o $o";;
        c) echo "gcc -m$B $PF -c $f -o $o";;
      esac
    done | xargs -P "${JOBS:-8}" -I{} sh -c '{} || echo FAIL {}'
  done
fi
rm -f probes/hdr/*
# view headers (C) and class headers (C++), relative to the include root; probe TU = h_<stem>
for h in v34/v34_engine_views v34/v34_flomodem_view v34/v34_v90params_view v34/v34_modem_params_view v34/v34_k56flex_view v34/v34_v90_phase2_view \
         v34/v34_v92_phase2_view v34/v34_object_view; do
  [ -f "$P/$h.h" ] && printf '#include <stddef.h>\n#include <stdint.h>\n#include "%s.h"\nint main(void){return 0;}\n' $h > probes/hdr/h_${h##*/}.c; done
for h in vpcm/vpcm_flo_modem v34/v34_object vpcm/dp_vpcm vpcm/vpcm_session_wrappers vpcm/pcm_leaf_classes vpcm/pcm_parameters; do
  [ -f "$P/$h.h" ] && printf '#include <stddef.h>\n#include <stdint.h>\n#include "%s.h"\nint main(void){return 0;}\n' $h > probes/hdr/h_${h##*/}.cpp; done
for b in 32 64; do
  rm -rf probes/hdr$b; mkdir -p probes/hdr$b
  for f in probes/hdr/*.c; do gcc -m$b -w -gdwarf-4 -fno-eliminate-unused-debug-types -I"$P" -c $f -o probes/hdr$b/$(basename ${f%.c}).o; done
  for f in probes/hdr/*.cpp; do g++ -m$b -w -gdwarf-4 -fno-eliminate-unused-debug-types -fno-exceptions -fno-rtti -I"$P" -c $f -o probes/hdr$b/$(basename ${f%.cpp}).o; done
  python3 "$S/dwarf_types.py" obj${b}g types$b.json
  python3 "$S/dwarf_types.py" probes/hdr$b hdrtypes$b.json
done
python3 "$S/same_name.py" types64.json types32.json > out/same_name.txt
python3 "$S/find_views.py" types32.json types64.json 3 > out/views_auto.txt
C="python3 $S/cmp_by_offset.py"
{
echo "## V.34 module object"
$C hdrtypes:h_v34_object_view:V34Obj types:vpcm_create:tagV34Object
for s in V34Rx:0x264 V34BlkB:0xa00 V34Mod:0x1450 V34PreFilter:0x2078 V34Tx:0x221c V34ToneDet:0x3564 V34Echo:0x80b8 V34Echo:0x9138 V34Probe:0xa320 \
         V34Info0a:0xa94c V34Info0d:0xa97c V34Info:0xa9ac V34Info1c:0xa9dc V34Q2:0xaa0c V34MpQ:0xaa3c V34Rate:0xaa84; do
  $C hdrtypes:h_v34_object_view:${s%%:*} hdrtypes:h_v34_object_view:V34Obj --base ${s#*:} --quiet; done
echo "## v34_dsp_primitives.c overlays"
$C types:v34_dsp_primitives:EchoAdaptState hdrtypes:h_v34_object_view:V34Obj --base 0x80b8 --quiet
$C types:v34_dsp_primitives:PreFilterState hdrtypes:h_v34_object_view:V34Obj --base 0x2078 --quiet
$C types:v34_dsp_primitives:TimingState hdrtypes:h_v34_object_view:V34Obj --base 0x50c --quiet
echo "## VPcmFloModem"
$C hdrtypes:h_v34_flomodem_view:PcmFlo47Obj types:vpcm_flo_modem:VPcmFloModem --quiet
$C types:vpcm_flo_modem_run:VPcmFloModem types:vpcm_flo_modem:VPcmFloModem
echo "## V90Parameters"
$C hdrtypes:h_v34_v90params_view:V90ParObj types:pcm_parameters:V90Parameters --quiet
echo "## modem parameter block: RS6 - ONE definition (src/core/dp_params.h: DpRuntime / V01_ModemParams / V34ModParObj / _tagModemParameters are one struct), nothing to pair"
echo "## K56Flex handle"
$C hdrtypes:h_v34_k56flex_view:K56HandleObj hdrtypes:h_v34_object:K56FlexHandle --quiet
echo "## phase 2 info records"
$C types:vpcm_flo_modem:V90Phase2InfoView types:pcm_leaf_classes:V90Phase2Info
$C types:vpcm_flo_modem:V92Phase2InfoView types:pcm_leaf_classes:V92Phase2Info
$C hdrtypes:h_v34_v90_phase2_view:P47V90InfoObj types:pcm_leaf_classes:V90Phase2Info
$C hdrtypes:h_v34_v92_phase2_view:P47V92InfoObj types:pcm_leaf_classes:V92Phase2Info
echo "## v34_engine_views.h prefix views (v34_api.c) vs the C++ objects they are applied to"
# RS1 / M3 renamed the views (V90Modem -> V90ModulatorPfx, V92Modem -> V92ModulatorPfx, V90RateSet -> V9xBitsToSymbolPfx, V90RateCur -> V90MapperPfx /
# V92TransmitterPfx, V90Demod -> V90DemodulatorPfx, V90Mgr -> V90ConnectionEvaluatorPfx).  To scan a tree from before RS1 / M3 pass the old list:
#   V34ENG_PAIRS="V90Modem:v90_modulator:V90Modulator V92Modem:v92_modulator:V92Modulator V90RateSet:pcm_bits_to_symbol:V90BitsToSymbol
#     V90RateSet:pcm_bits_to_symbol:V92BitsToSymbol V90RateCur:v90_mapper:V90Mapper V90RateCur:pcm_bits_to_symbol:V92Transmitter
#     V90Demod:v90_demodulator:V90Demodulator V90Mgr:v90_connection_evaluator:V90ConnectionEvaluator"
for pr in ${V34ENG_PAIRS:-V90ModulatorPfx:v90_modulator:V90Modulator V92ModulatorPfx:v92_modulator:V92Modulator \
          V90BitsToSymbolPfx:pcm_bits_to_symbol:V90BitsToSymbol V92BitsToSymbolPfx:pcm_bits_to_symbol:V92BitsToSymbol \
          V90MapperPfx:v90_mapper:V90Mapper V92TransmitterPfx:pcm_bits_to_symbol:V92Transmitter \
          V90DemodulatorPfx:v90_demodulator:V90Demodulator V90ConnectionEvaluatorPfx:v90_connection_evaluator:V90ConnectionEvaluator}; do
  v=${pr%%:*}; r=${pr#*:}
  $C hdrtypes:h_v34_engine_views:$v types:$r; done
echo "## Z01"
$C types:dp_vpcm:Z01_DpOperations types:dp_vpcm:dp_operations --quiet
} > out/pairs.txt 2>&1
python3 "$S/v34_rawaudit.py" "$P" > out/v34_rawaudit.txt
python3 "$S/summary.py" out > out/summary.txt
cat out/summary.txt
echo "outputs: $OUT/out/{summary,pairs,same_name,views_auto,v34_rawaudit}.txt"
