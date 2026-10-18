#!/bin/sh
# tools/hostids_check.sh - src/core/host_interface.h (the port-owned host interface: DP_ID, DPSTAT_*, MDMPRM_* ids, struct dp ...) must be FREESTANDING,
# because the C++ half of the library (compiled -fno-exceptions -fno-rtti, only freestanding headers) includes it for the data pump and parameter ids:
#   1. the header alone, compiled with -ffreestanding -nostdinc as C and as C++ (-m32 and -m64): it must not include anything;
#   2. the ids the C++ files use (DP_V34 / DP_K56 / DP_V90 / DP_V92, DPSTAT_*, the voice parameter ids) have their documented values, in C and C++;
#   3. negative tests: a header that includes a system header, and a wrong id value, must each make the check fail.
# The values are compared with the slmodemd host headers by the optional tools/hostiface_check.sh.
# Compile only (-fsyntax-only).  Usage, from the repository root: sh tools/hostids_check.sh
cd "$(dirname "$0")/.." || exit 2
rc=0
H=src/core/host_interface.h
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
cat > "$tmp/ids.c" <<'EOT'
#include "core/host_interface.h"
#ifdef __cplusplus
#define CHECK(c) static_assert(c, #c)
#else
#define CHECK(c) _Static_assert(c, #c)
#endif
CHECK(DP_V34 == 34 && DP_K56 == 56 && DP_V90 == 90 && DP_V92 == 92);
CHECK(DPSTAT_OK == 0 && DPSTAT_CONNECT == 1 && DPSTAT_ERROR == 4 && DPSTAT_NODIALTONE == 6 && DPSTAT_BUSY == 7 && DPSTAT_NOANSWER == 8 && DPSTAT_CHANGEDP == 10);
CHECK(MDMPRM_VOICEINFO == 12 && GetDTMFHighToneLevel == 20 && GetDTMFHighAndLowToneLevelDifference == 36 && GetAdditAttenToBeepgenVoice == 60);
EOT
for m in -m32 -m64; do
  gcc $m -fsyntax-only -ffreestanding -nostdinc -x c "$H" || rc=1
  g++ $m -fsyntax-only -ffreestanding -nostdinc -x c++ "$H" || rc=1
  gcc $m -fsyntax-only -ffreestanding -nostdinc -std=gnu11 -Isrc -x c "$tmp/ids.c" || rc=1
  g++ $m -fsyntax-only -ffreestanding -nostdinc -std=c++11 -Isrc -x c++ "$tmp/ids.c" || rc=1
done
mkdir -p "$tmp/core"
{ echo '#include <stddef.h>'; cat "$H"; } > "$tmp/core/host_interface.h"
if gcc -fsyntax-only -ffreestanding -nostdinc -x c "$tmp/core/host_interface.h" 2> /dev/null; then echo "hostids_check: negative test (system include) did NOT fail"; rc=1; fi
sed 's/DP_V90 = 90,/DP_V90 = 91,/' "$H" > "$tmp/core/host_interface.h"
if gcc -fsyntax-only -ffreestanding -nostdinc -std=gnu11 -I"$tmp" -x c "$tmp/ids.c" 2> /dev/null; then echo "hostids_check: negative test (wrong value) did NOT fail"; rc=1; fi
[ $rc = 0 ] && echo "hostids_check: OK"
exit $rc
