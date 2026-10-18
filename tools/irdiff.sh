#!/bin/bash
# tools/irdiff.sh - compare the LLVM IR of the library at two commits (clang, -m32, optimised, value names stripped).
#   bash tools/irdiff.sh [-O1|-O2] [--bits 32|64] <old-rev> <new-rev>     |     bash tools/irdiff.sh --selftest
# ADVISORY: static evidence that a change kept the meaning of the code; it does NOT replace the o0check / objcmp gate.  Blind spots: x87 precision / evaluation order, inline asm (passed through as text), clang-vs-gcc front-end differences.
# Files whose IR differs are listed; inline-asm srcloc cookies (!N = !{i64 ...}) are ignored (they shift when a header's line numbers change).
# After each differing file the functions whose text changed are listed ("    function: name"; "<module>" = globals / declarations / types).
# Needs clang, clang++, opt (LLVM 14 tested).
set -e
OPT=-O1; BITS=32
while [ $# -gt 0 ]; do case $1 in -O1|-O2) OPT=$1; shift;; --bits) BITS=$2; shift 2;; *) break;; esac; done
HERE=$(cd "$(dirname "$0")/.." && pwd)
W=${IRDIFF_TMP:-$(mktemp -d)}
FILTER='^source_filename\|^; ModuleID\|^!.* = !{i64 '
emit() { # <srcdir> <outdir>
  ( cd "$1"; srcs=$(make -s -f sources.mk -f - print <<'M'
print:
	@echo $(DSP_SRCS)
M
)
    for s in $srcs; do
      o=$2/$s.ll; mkdir -p "$(dirname "$o")"
      case $s in *.cpp) d=clang++; x="-fno-exceptions -fno-rtti -x c++";; *) d=clang; x=;; esac
      $d -m$BITS $OPT -S -emit-llvm -w -fno-stack-protector -fno-pie -Isrc $x -Xclang -disable-llvm-passes -c "$s" -o "$o" 2>"$o.err" || echo "irdiff: compile failed: $s" >&2
      opt -O1 -S "$o" -o "$o.opt" 2>/dev/null && opt -strip -S "$o.opt" -o "$o.n" 2>/dev/null || true
    done )
}
tree() { # <rev> -> W/<rev>
  mkdir -p "$W/$1"; git -C "$HERE" archive "$1" . | tar -x -C "$W/$1"
}
if [ "$1" = --selftest ]; then
  tree HEAD; emit "$W/HEAD" "$W/HEAD/ir"
  f=src/fpm/fpm_math.c; cp "$W/HEAD/$f" "$W/orig.c"
  n=$(grep -n '0x38e' "$W/HEAD/$f" | head -1 | cut -d: -f1)
  [ -n "$n" ] || { echo "selftest: probe constant gone; pick another"; exit 2; }
  sed -i "${n}s/0x38e/0x38f/" "$W/HEAD/$f"
  ( cd "$W/HEAD"; clang -m$BITS $OPT -S -emit-llvm -w -fno-pie -Isrc -Xclang -disable-llvm-passes -c $f -o "$W/p.ll"; opt -O1 -S "$W/p.ll" -o "$W/p.opt"; opt -strip -S "$W/p.opt" -o "$W/p.n" )
  if diff -q <(grep -v "$FILTER" "$W/p.n") <(grep -v "$FILTER" "$W/HEAD/ir/$f.ll.n") >/dev/null; then echo "selftest FAILED: a changed constant was not detected"; exit 1; fi
  echo "selftest OK: a changed constant is detected"; exit 0
fi
# fnlist <old.n> <new.n>: names of the functions (and "<module>" for globals / declarations / types) whose text differs; per-function text is compared,
# so the hunks of a plain diff (shifted by value renumbering) need not be mapped to functions by hand.
fnsplit() { grep -v "$FILTER" "$1" | awk '/^define /{n=$0; sub(/^[^@]*@/,"",n); sub(/\(.*/,"",n); fn=n; body=""} fn!=""{body=body "\001" $0; if($0=="}"){print fn "\t" body; fn=""}; next} /^(@|declare |%|\$)/{g=g "\001" $0} END{print "<module>\t" g}' | sort; }
fnlist() { diff <(fnsplit "$1") <(fnsplit "$2") | grep '^[<>]' | cut -f1 | sed 's/^[<>] //' | sort -u | sed 's/^/    function: /'; }
OLD=$1; NEW=$2; [ -n "$NEW" ] || { echo "usage: irdiff.sh [-O1|-O2] [--bits N] <old> <new> | --selftest"; exit 2; }
tree "$OLD"; tree "$NEW"; emit "$W/$OLD" "$W/$OLD/ir"; emit "$W/$NEW" "$W/$NEW/ir"
same=0; diffn=0
for f in $(cd "$W/$OLD/ir" && find . -name '*.n' | sort); do
  if [ -f "$W/$NEW/ir/$f" ] && diff -q <(grep -v "$FILTER" "$W/$OLD/ir/$f") <(grep -v "$FILTER" "$W/$NEW/ir/$f") >/dev/null; then same=$((same+1)); else diffn=$((diffn+1)); echo "IR differs: ${f%.ll.n}"; fnlist "$W/$OLD/ir/$f" "$W/$NEW/ir/$f"; fi
done
echo "irdiff $OPT -m$BITS $OLD..$NEW: $same files identical, $diffn differ"
[ -n "$IRDIFF_TMP" ] || rm -rf "$W"
[ $diffn -eq 0 ]
