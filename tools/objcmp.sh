#!/bin/sh
# tools/objcmp.sh REFA [REFB] - whole-object comparison of two states of the library (RS8): the complement of tools/o0check.py, which is blind to the ORDER
# of the functions inside an object (it compares function by function).  An object's function order is part of the identity of a restructuring (an unused plain
# `static` function is emitted at -O0, and the order in which used `static inline` helpers are defined is the order they are emitted in), so the check that
# a header move or a dedup changed nothing is this one.
#
#   tools/objcmp.sh REFA            REFA (any git ref) against the working tree
#   tools/objcmp.sh REFA REFB       REFA against REFB
#   tools/objcmp.sh --o2 REFA [REFB]  gate on the -O2 configurations only: all four configurations are built and compared and the differing objects are listed
#                                   per configuration (lines `DIFF <config> <object>`, config = 32-O0 | 32-O2 | 64-O0 | 64-O2), but only -m32 -O2 and -m64 -O2
#                                   decide the exit status (0 = both identical).
#
# Both states are built with the production recipe (the library's own Makefile: `make BITS=32|64 PORT_OPT=-O0|-O2 obj$BITS/libdsplib.a`) in mkdtemp scratch trees
# under $TMPDIR; per object file the sha256 of `objdump -dr` + `objdump -s -r -h` (in section / function order, file name line removed) is compared, for -m32 and
# -m64 at -O0 and -O2: one hash per object and configuration.  The identifier renames of tools/o0check_renames.tsv (if any) are applied to the dumps of BOTH sides first.
# Exit 0 = every object identical, 1 = list of differing objects.  About 25 s per state.
HERE=$(cd "$(dirname "$0")" && pwd)
LIB=$(cd "$HERE/.." && pwd)
TOP=$(git -C "$LIB" rev-parse --show-toplevel) || exit 2
O2GATE=0
if [ "$1" = "--o2" ]; then O2GATE=1; shift; fi
[ -n "$1" ] || { sed -n 2,19p "$0"; exit 2; }
T=$(mktemp -d "${TMPDIR:-/tmp}/objcmp.XXXXXX") || exit 2
trap 'rm -rf "$T"' EXIT
SEDF=$T/renames.sed
grep -v '^#' "$HERE/o0check_renames.tsv" | awk -F'\t' 'NF>=2{printf "s/\\b%s\\b/%s/g\n",$1,$2}' > "$SEDF"

# state NAME REF -> $T/NAME (the repository tree at REF, or a copy of the working tree)
state() {
  mkdir -p "$T/$1"
  if [ "$2" = WORKTREE ]; then tar -C "$TOP" --exclude=.git --exclude='obj32' --exclude='obj64' --exclude='objnative' -c . | tar -x -C "$T/$1"
  else git -C "$TOP" archive "$2" | tar -x -C "$T/$1"; fi
}
hashes() {   # NAME -> $T/NAME.txt
  d=$T/$1
  : > "$T/$1.txt"
  for b in 32 64; do
    for o in -O0 -O2; do
      rm -rf "$d/obj$b"
      make -s -j"$(nproc)" -C "$d" BITS=$b PORT_OPT=$o "obj$b/libdsplib.a" >"$T/$1.make.$b$o.log" 2>&1 || { echo "objcmp: build failed ($1 $b $o), see $T/$1.make.$b$o.log" >&2; cat "$T/$1.make.$b$o.log" | tail -5 >&2; exit 2; }
      for f in $(cd "$d/obj$b" && find . -name '*.o' | sort); do
        h=$( (objdump -dr "$d/obj$b/$f"; objdump -s -r -h "$d/obj$b/$f") | grep -v 'file format' | sed -f "$SEDF" | sha256sum | cut -c1-16 )
        echo "$b$o $f $h" >> "$T/$1.txt"
      done
    done
  done
}
state a "$1" || exit 2
state b "${2:-WORKTREE}" || exit 2
hashes a || exit 2
hashes b || exit 2
n=$(wc -l < "$T/a.txt")
if [ "$O2GATE" = 1 ]; then
  diff "$T/a.txt" "$T/b.txt" | grep '^>' | awk '{print "DIFF", $2, $3}' > "$T/diffs.txt"
  for c in 32-O0 32-O2 64-O0 64-O2; do echo "objcmp: $c: $(grep -c "^DIFF $c " "$T/diffs.txt") of $(grep -c "^$c " "$T/a.txt") objects differ"; done
  cat "$T/diffs.txt"
  if grep -q '^DIFF [0-9]*-O2 ' "$T/diffs.txt"; then echo "objcmp: -O2 DIFFERENT"; exit 1; fi
  echo "objcmp: -O2 IDENTICAL (-m32 and -m64)"; exit 0
fi
if cmp -s "$T/a.txt" "$T/b.txt"; then echo "objcmp: IDENTICAL ($n objects: -m32 / -m64 x -O0 / -O2, function order included)"; exit 0; fi
echo "objcmp: DIFFERENT"; diff "$T/a.txt" "$T/b.txt" | grep '^>' | awk '{print $2, $3}'
exit 1
