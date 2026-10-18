#!/bin/sh
# tools/stripcmp.sh PREV-REF - proof that a comment-only pass changed no code.
#
# For every C / C++ source under src/ (.c .cpp .h .hpp .inc) that differs between PREV-REF and the working tree, both versions are run through
# `gcc -fpreprocessed -dD -E -P` (comments removed; #define / #include / #if text kept; string and character literals respected), whitespace is normalised
# (runs of blanks and line breaks collapse, blanks next to punctuation disappear; preprocessor directives stay on lines of their own) and the results are compared.
# Exit 0 = every differing file is token-identical apart from comments; 1 = list of files whose code differs (or that were added / removed).
#   tools/stripcmp.sh REF        REF (any git ref) against the working tree
# tools/stripcmp.allow (optional): the code-token rewordings a comment pass may make, lines `path<TAB>old<TAB>new` (path relative to the repository
# root, e.g. src/cid/cid.h; old / new are literal text; blank and `#` lines ignored).  Each `old` is replaced by `new` in the REF text of that file first.
HERE=$(cd "$(dirname "$0")" && pwd)
LIB=$(cd "$HERE/.." && pwd)
[ -n "$1" ] || { sed -n 2,10p "$0"; exit 2; }
REF=$1
PREFIX=$(git -C "$LIB" rev-parse --show-prefix)
ALLOW=$HERE/stripcmp.allow
T=$(mktemp -d "${TMPDIR:-/tmp}/stripcmp.XXXXXX") || exit 2
trap 'rm -rf "$T"' EXIT
: > "$T/bad"
: > "$T/n"
norm() {   # FILE LANG -> comment-free, whitespace-normalised text on stdout
  gcc -x "$2" -std=gnu99 -fpreprocessed -dD -E -P "$1" 2>/dev/null | awk '
    function flush() { if (buf != "") { print buf; buf = "" } }
    /^[ \t]*#/ { flush(); sub(/^[ \t]+/, ""); gsub(/[ \t]+/, " "); print; next }
    { buf = buf " " $0 }
    END { flush() }' | sed -e 's/[ \t][ \t]*/ /g' -e 's/ *\([][(){};,.<>=+*\/%&|^!~?:-]\) */\1/g' -e 's/^ //'
}
git -C "$LIB" diff --name-status --no-renames "$REF" -- src > "$T/list"
while IFS="$(printf '\t')" read -r st f; do
  case $f in *.c|*.cpp|*.h|*.hpp|*.inc) ;; *) continue ;; esac
  rel=${f#"$PREFIX"}
  case $f in *.c) lang=c ;; *) lang=c++ ;; esac
  if [ "$st" != M ]; then echo "STATUS $st: $rel"; echo x >> "$T/bad"; continue; fi
  git -C "$LIB" show "$REF:$f" > "$T/old.src" 2>/dev/null
  cp "$LIB/$rel" "$T/new.src"
  if [ -f "$ALLOW" ]; then   # apply this file's allowlist entries to the old side
    awk -F '\t' -v f="$rel" '$0 !~ /^#/ && NF >= 3 && $1 == f { print $2 "\t" $3 }' "$ALLOW" > "$T/subs"
    if [ -s "$T/subs" ]; then
      awk -F '\t' 'NR == FNR { o[++n] = $1; w[n] = $2; next }
        { s = $0
          for (i = 1; i <= n; i++) {
            out = ""
            while ((k = index(s, o[i])) > 0) { out = out substr(s, 1, k - 1) w[i]; s = substr(s, k + length(o[i])) }
            s = out s
          }
          print s }' "$T/subs" "$T/old.src" > "$T/old2.src" && mv "$T/old2.src" "$T/old.src"
    fi
  fi
  norm "$T/old.src" "$lang" > "$T/old.txt"
  norm "$T/new.src" "$lang" > "$T/new.txt"
  echo x >> "$T/n"
  if [ ! -s "$T/new.txt" ] || ! cmp -s "$T/old.txt" "$T/new.txt"; then echo "CODE DIFFERS: $rel"; echo x >> "$T/bad"; fi
done < "$T/list"
checked=$(wc -l < "$T/n")
if [ -s "$T/bad" ]; then echo "stripcmp: FAIL"; exit 1; fi
echo "stripcmp: OK, $checked changed file(s), code identical with comments removed"
