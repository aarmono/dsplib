#!/bin/sh
# tools/hostiface_check.sh - OPTIONAL conformance check of src/core/host_interface.h against a host's headers (slmodemd's).
#
# The library does not need the host headers to build; this script exists to prove that the library-owned header still agrees with the host
# interface it mirrors.  Usage, from anywhere:
#   [HOST_INCDIR=<dir with modem_defs.h, modem_dp.h, modem_param.h>] sh tools/hostiface_check.sh
# There is no default for HOST_INCDIR: when it is not set, or does not hold the three headers, the check is SKIPPED (a message and exit 0).  Exit 1 = a difference was found, 2 = the check itself could not run.
#
# The two interfaces cannot share a translation unit (both declare enum DP_ID, struct dp ...), so the same program is compiled twice, once against
# the host headers and once against the port header, for -m32 and -m64; each program prints
#   - the value of every enumerator and of every DPSTAT_* / FAX_STATUS_* / VOICE_STATUS_* / VOICE_STATE_* macro of the port header,
#   - sizeof of the four enums and four structures, offsetof of every structure member,
# and the two outputs are diffed.  The compile also initialises function pointers with modem_get_param, modem_set_param, modem_get_bits, modem_put_bits,
# modem_get_sreg and modem_dp_register / modem_dp_deregister under -Werror=incompatible-pointer-types, so the prototypes of core/dsp_host.h and
# core/host_interface.h have to match the host's.  Finally every enumerator of the host's four enums must exist in the port header.
cd "$(dirname "$0")/.." || exit 2
if [ -z "$HOST_INCDIR" ]; then
  echo "hostiface_check: SKIPPED (HOST_INCDIR not set; set it to a directory with the host headers modem_defs.h, modem_dp.h, modem_param.h)"
  exit 0
fi
for h in modem_defs.h modem_dp.h modem_param.h; do
  if [ ! -f "$HOST_INCDIR/$h" ]; then
    echo "hostiface_check: SKIPPED ($HOST_INCDIR/$h not found; set HOST_INCDIR to a directory with the host headers)"
    exit 0
  fi
done
PORT_H=src/core/host_interface.h
T=$(mktemp -d "${TMPDIR:-/tmp}/hostiface.XXXXXX") || exit 2
trap 'rm -rf "$T"' EXIT

# enumerators of the four enums of a header (comments removed): "ENUM NAME" lines
enumerators() {
  sed -e 's://.*$::' -e 's:/\*.*\*/::' "$1" | awk '
    /^[ \t]*enum[ \t]+(DP_ID|FAX_CMD|VOICE_CMD|MODEM_PARAMETER_NAMES)[ \t]*(\{|$)/ { e = $2; next }
    e != "" && /^[ \t]*\{/ { next }
    e != "" && /^[ \t]*\}/ { e = ""; next }
    e != "" && match($0, /[A-Za-z_][A-Za-z_0-9]*/) { print e, substr($0, RSTART, RLENGTH) }'
}
# integer macros of the families the port header carries
macros() {
  sed -e 's://.*$::' "$1" | sed -n 's/^[ \t]*#[ \t]*define[ \t][ \t]*\(\(DPSTAT\|FAX_STATUS\|VOICE_STATUS\|VOICE_STATE\)_[A-Z0-9_]*\)[ \t].*$/\1/p'
}
enumerators "$PORT_H" | sort -u > "$T/port.enum"
macros "$PORT_H" | sort -u > "$T/port.macro"
[ -s "$T/port.enum" ] && [ -s "$T/port.macro" ] || { echo "hostiface_check: cannot read the enumerators of $PORT_H"; exit 2; }

# every enumerator / macro of the host must be in the port header
rc=0
for h in modem_defs.h modem_dp.h modem_param.h; do enumerators "$HOST_INCDIR/$h"; done | sort -u > "$T/host.enum"
for h in modem_defs.h; do macros "$HOST_INCDIR/$h"; done | sort -u > "$T/host.macro"
missing=$(comm -13 "$T/port.enum" "$T/host.enum")
[ -z "$missing" ] || { echo "hostiface_check: enumerators of the host that the port header lacks:"; echo "$missing"; rc=1; }
missing=$(comm -13 "$T/port.macro" "$T/host.macro")
[ -z "$missing" ] || { echo "hostiface_check: macros of the host that the port header lacks:"; echo "$missing"; rc=1; }

# the program
{
  printf '%s\n' '#include <stdio.h>'
  printf '%s\n' '#include <stddef.h>'
  printf '%s\n' '#ifdef AGAINST_HOST'
  printf '%s\n' '#include <modem_defs.h>'
  printf '%s\n' '#include <modem_dp.h>'
  printf '%s\n' '#include <modem_param.h>'
  printf '%s\n' '#else'
  printf '%s\n' '#include "core/dsp_host.h"'
  printf '%s\n' '#include "core/host_interface.h"'
  printf '%s\n' '#endif'
  printf '%s\n' '#define V(n) printf("%s = %lld\n", #n, (long long)(n))'
  printf '%s\n' '#define SZ(t) printf("sizeof(%s) = %zu\n", #t, sizeof(t))'
  printf '%s\n' '#define OFF(t, m) printf("offsetof(%s, %s) = %zu\n", #t, #m, offsetof(t, m))'
  printf '%s\n' '/* prototype agreement: the initialisers must not warn (-Werror=incompatible-pointer-types) */'
  printf '%s\n' 'static long (*const p_get_param)(struct modem*, unsigned) = modem_get_param;'
  printf '%s\n' 'static long (*const p_set_param)(struct modem*, unsigned, int) = modem_set_param;'
  printf '%s\n' 'static int (*const p_get_bits)(struct modem*, int, unsigned char*, int) = modem_get_bits;'
  printf '%s\n' 'static int (*const p_put_bits)(struct modem*, int, unsigned char*, int) = modem_put_bits;'
  printf '%s\n' 'static int (*const p_get_sreg)(struct modem*, unsigned int) = modem_get_sreg;'
  printf '%s\n' 'static int (*const p_dp_register)(enum DP_ID, struct dp_operations*) = modem_dp_register;'
  printf '%s\n' 'static void (*const p_dp_deregister)(enum DP_ID, struct dp_operations*) = modem_dp_deregister;'
  printf '%s\n' 'int main(void)'
  printf '%s\n' '{'
  printf '%s\n' '  (void)p_get_param; (void)p_set_param; (void)p_get_bits; (void)p_put_bits; (void)p_get_sreg; (void)p_dp_register; (void)p_dp_deregister;'
  awk '{ print "  V(" $2 ");" }' "$T/port.enum"
  awk '{ print "  V(" $1 ");" }' "$T/port.macro"
  for t in "enum DP_ID" "enum FAX_CMD" "enum VOICE_CMD" "enum MODEM_PARAMETER_NAMES" "struct dsp_info" "struct voice_info" "struct dp_operations" "struct dp"; do
    echo "  SZ($t);"
  done
  for m in connection_type clock_deviation qc_lapm qc_index; do echo "  OFF(struct dsp_info, $m);"; done
  for m in comp_method sample_rate rx_gain tx_gain dtmf_symbol tone1_freq tone2_freq tone_duration inactivity_timer silence_detect_sensitivity \
    silence_detect_period; do echo "  OFF(struct voice_info, $m);"; done
  for m in name use_count create delete process hangup; do echo "  OFF(struct dp_operations, $m);"; done
  for m in id modem status op dp_data; do echo "  OFF(struct dp, $m);"; done
  printf '%s\n' '  return 0;'
  printf '%s\n' '}'
} > "$T/table.c"

for m in -m32 -m64; do
  for side in host port; do
    if [ $side = host ]; then DEF=-DAGAINST_HOST; else DEF=; fi
    if ! gcc $m -std=gnu11 -Wall -Werror=incompatible-pointer-types -Wl,--unresolved-symbols=ignore-all $DEF -I"$HOST_INCDIR" -Isrc "$T/table.c" -o "$T/$side$m" 2> "$T/$side$m.err"; then
      echo "hostiface_check: $side $m: compile failed"; cat "$T/$side$m.err"; exit 2
    fi
    if ! "$T/$side$m" > "$T/$side$m.txt" 2>/dev/null; then echo "hostiface_check: cannot run the $m program (is a 32-bit runtime installed?)"; exit 2; fi
  done
  if diff "$T/host$m.txt" "$T/port$m.txt" > "$T/diff$m"; then
    echo "hostiface_check: $m OK ($(wc -l < "$T/port$m.txt") values, sizes and offsets identical)"
  else
    echo "hostiface_check: $m DIFFERENT (< host, > port):"; cat "$T/diff$m"; rc=1
  fi
done
[ $rc = 0 ] && echo "hostiface_check: OK"
exit $rc
