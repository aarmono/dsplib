#!/bin/sh
# ident32.sh [-r REF] [--allow-new] file.c ...   (run from the repository root)
# 32-bit code AND data identity of the working tree against git REF (default HEAD): each file is compiled with the library's own recipe
# (`make -n -B BITS=32 PORT_OPT=<level> PIN_FLAGS=<pin or empty>` of the Makefile prints the command) at -O0, -O2 and the pinned set (-O2 -fno-strict-aliasing)
# from a `git archive REF` copy and from the working tree; the two objects must give identical
#   objdump -dr --no-show-raw-insn   (code + code relocations)
#   objdump -s -r on every non-code section (.data* .rodata* .bss ... contents, and the relocations that fill them)
# Prints IDENTICAL / DIFFERENT per file and level, exit 0 only if everything is identical.  A file that REF does not have is an ERROR (exit 2)
# unless --allow-new (then it is reported as NEW and not compared); a file the Makefile does not build is an ERROR, too.
# Not claimed: -O1/-Os/-O3, other compilers.  For whole-tree checks use tools/objcmp.sh (-m32 and -m64) or tools/o0check.py (per function, layouts).
cd "$(dirname "$0")/.." || exit 2
REF=HEAD; NEWOK=
while [ $# -gt 0 ]; do case "$1" in -r) REF=$2; shift 2;; --allow-new) NEWOK=1; shift;; *) break;; esac; done
[ $# -gt 0 ] || { sed -n 2,11p "$0"; exit 2; }
T=$(mktemp -d "${TMPDIR:-/tmp}/ident32.XXXXXX"); trap 'rm -rf "$T"' EXIT
mkdir "$T/ref"
git archive "$REF" | tar -x -C "$T/ref" || exit 2
RC=0
for f in "$@"; do
  f=${f#./}
  [ -f "$f" ] || { echo "$f: no such file in the working tree"; RC=2; continue; }
  if [ ! -f "$T/ref/$f" ]; then
    if [ -n "$NEWOK" ]; then echo "$f: NEW (not in $REF; not compared)"; else echo "$f: NOT IN $REF: ERROR (--allow-new to accept new files)"; RC=2; fi
    continue
  fi
  for lvl in "-O0" "-O2" "-O2 -fno-strict-aliasing"; do
    case "$lvl" in "-O2 -fno-strict-aliasing") MOPT=-O2; MPIN="-fno-strict-aliasing";; *) MOPT=$lvl; MPIN=;; esac
    for side in ref work; do
      if [ $side = ref ]; then D="$T/ref"; else D=.; fi
      OD="$T/$side.objs"
      CMD=$(make -s -n -B -C "$D" BITS=32 PORT_OPT="$MOPT" PIN_FLAGS="$MPIN" OBJDIR="$OD" "$OD/${f%.*}.o" 2>/dev/null | grep -- ' -c ' | sed "s# -o $OD/[^ ]*\$# -o $T/$side.o#")
      if [ -z "$CMD" ]; then echo "$f [$lvl]: not built by the Makefile ($side): ERROR"; RC=2; continue 2; fi
      ( cd "$D" && sh -c "$CMD" ) || { echo "$f [$lvl]: COMPILE FAIL ($side)"; [ $RC = 2 ] || RC=1; continue 2; }
      objdump -dr --no-show-raw-insn "$T/$side.o" | grep -v "file format" > "$T/$side.code"
      : > "$T/$side.data"
      for sec in $(objdump -h "$T/$side.o" | awk '$2 ~ /^\.(data|rodata|bss|sdata|tdata)/ {print $2}'); do
        echo "== $sec" >> "$T/$side.data"
        objdump -s -r -j "$sec" "$T/$side.o" | grep -v "file format" >> "$T/$side.data"
      done
    done
    c1=IDENTICAL; cmp -s "$T/ref.code" "$T/work.code" || c1=DIFFERENT
    d=IDENTICAL; cmp -s "$T/ref.data" "$T/work.data" || d=DIFFERENT
    [ $c1 = IDENTICAL ] && [ $d = IDENTICAL ] || { [ $RC = 2 ] || RC=1; }
    echo "$f [$lvl]: code $c1 ($(wc -l < "$T/work.code") lines), data $d ($(wc -l < "$T/work.data") lines)"
  done
done
exit $RC
