#!/bin/sh
# install-hooks.sh - install tools/hooks/pre-commit into the repository's hooks directory (shared by all worktrees) as a symlink.
cd "$(dirname "$0")/.." || exit 1
H=$(git rev-parse --git-common-dir)/hooks
mkdir -p "$H"
ln -sf "$(pwd)/tools/hooks/pre-commit" "$H/pre-commit" && echo "installed $H/pre-commit -> $(pwd)/tools/hooks/pre-commit"
