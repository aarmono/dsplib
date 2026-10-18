# CLAUDE.md - dsplib

A portable, 64-bit-clean C / C++ DSP library (soft-modem signal processing: V.8, V.21/V.23, V.22, V.32, V.34, V.90/V.92 PCM, fax, voice, tones,
call progress, caller ID).  It builds `libdsplib.a` with a C/C++ compiler and make only; the host application (a modem daemon) supplies the
`sysdep_*`, `modem_*` and `dsplibs_debug_printf` functions the library calls, see `docs/host-interface.md`.  Read `docs/architecture.md` first.

## Build

* `make [BITS=32|64] [PORT_OPT=-O2]` -> `obj$(BITS)/libdsplib.a`; `make install [PREFIX=...]` installs the archive and `host_interface.h`.
* `sources.mk` lists the sources (`python3 tools/port_objs.py check` verifies it), `dsplib.mk` has the recipe, `flags.mk` the pinned flags.
* The library owns its host interface (`src/core/host_interface.h`, `src/core/dsp_host.h`) and includes no header of the host.
  `tools/hostiface_check.sh` compares it with a host's headers; it runs only when `HOST_INCDIR` is set and is skipped otherwise.

## Formatting

`src/` is clang-formatted (`.clang-format`, skip list `.clang-format-ignore`; `tools/format.sh [--check] [dir]`).  Install the pre-commit hook once
per clone with `tools/install-hooks.sh`; it refuses a commit whose staged files under `src/` are unformatted.  Format before committing:
`tools/format.sh src && git add -u`.

## Checks

`tools/check_generated.sh` runs the cheap checks in one go (source list, generated headers and layout asserts, enum headers, clang-format, declarations);
run it before committing.  `tools/hostiface_check.sh` is separate and needs `HOST_INCDIR`.

## Layout descriptions and generated headers

* `layouts/**/*.layout` describe the data structures of the library (member offsets, sizes, types) and are the SOURCE of the generated headers
  `src/**/*_layout.inc` and `src/v34/v34_*_view.h`, `v34_subobjects.h`, `v34_object_layout*.inc`.  Never edit a generated file by hand: edit the
  description (new members of the V.34 object: `tools/split_region.py`) and run the generator.
* `tools/gen_views.sh [--check]` regenerates / verifies the V.34 views; `tools/layout_gen.py` produces one `.inc` per description (the
  `# regenerate:` line in each description's header is the command); `tools/layout_check.py` verifies that all of them are up to date.
  `tools/v34_hs_states_gen.py` and `tools/v8_status_gen.py` generate the state / status enums from the tables in the sources.

## Proving that a change did not alter behaviour

From the weakest to the strongest claim about a change; run the one that matches what the change is meant to be:

* `tools/stripcmp.sh <ref>` - comment-only / whitespace-only: the code tokens are identical once comments are removed.
* `tools/ident32.sh [-r ref] files...` - the named files give identical -m32 code and data at -O0, -O2 and the pinned set.
* `tools/objcmp.sh [--o2] <ref> [<ref2>]` - whole-library object comparison, -m32 / -m64 at -O0 and -O2 (function order included).
  `--o2` lists the differing objects per configuration and gates on the -O2 ones.
* `tools/o0check.py [-r ref] [--o2] [--o2-only]` - per-function assembly and struct layouts, with the accepted differences in
  `tools/o0check_expected.tsv`, `o0check_renames.tsv` and `o0check_unified.tsv` (each entry needs a reason).  `--self-test` checks the checker.
* `tools/irdiff.sh [-O1|-O2] <old> <new>` - LLVM IR text diff of two commits; advisory (it shows where a change is NOT neutral).

Identical -O2 objects mean the shipped code is unchanged.  A change that is not neutral needs a behavioural test by whoever consumes the
library; say plainly what was and was not verified.

## Rules

* Plain member access (`p->member`) and typed parameters; no new offset / cast accessor macros.  Addresses are pointers, not integers.
* Keep opaque `void *` only at the entry points the host calls.
* A module's internal prototypes go in its `<module>_internal.h`; what another module calls goes in the defining module's public header.
  Never `extern` a library function inside a .c / .cpp and never include another module's `*_internal.h`.
* The host's functions are declared once, in `src/core/dsp_host.h`.
* Scratch files: one directory under `/tmp/<branch-name>/`, deleted at the end.  Stop if `/` has under 5 GB free.
* Commit identity: Use the user name and email configured within git.  One commit per file or small group; the message says
  what changed and the evidence.  Commit messages and pull request descriptions contain no mention of Claude, of any AI model, or of
  tooling that generated the change: no `Co-Authored-By` trailer and no "generated with" line.
