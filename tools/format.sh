#!/bin/sh
# format.sh - run clang-format (config: .clang-format, skip list: .clang-format-ignore) over the C/C++ sources of the given directories (default: src).
#   tools/format.sh [dir ...]          reformat in place
#   tools/format.sh --check [dir ...]  list the files that would change and exit 1 if there are any
cd "$(dirname "$0")/.." || exit 1
CHECK=; [ "$1" = "--check" ] && { CHECK=1; shift; }
[ $# -eq 0 ] && set -- src
FILES=$(find "$@" -type f \( -name '*.c' -o -name '*.cpp' -o -name '*.h' -o -name '*.inc' \) | sort)
if [ -n "$CHECK" ]; then
    # --dry-run honours .clang-format-ignore; each file that would change yields an "error:" line (-Werror)
    BAD=$(echo "$FILES" | xargs clang-format --dry-run -Werror 2>&1 | grep -E '^[^ ]+:[0-9]+:[0-9]+: error:' | cut -d: -f1 | sort -u)
    [ -z "$BAD" ] && exit 0
    echo "$BAD"
    exit 1
else
    echo "$FILES" | xargs clang-format -i
fi
