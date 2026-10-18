# Comment style (src/)

Comments in `src/` describe what the code IS and DOES, in doxygen form.  They do not describe how it was produced: no mention of the
original binary ("blob", "dsplibs.o", "i386 image", "decompile", "Ghidra", "phase-1", "RSn/PMn" pass names, `porting/` paths, "faithful to",
"vendor", "original"), and no archaeology (offsets that exist only to match the original image, "was X before").  The comments carry no history; the known defects live in `docs/bugs.md`.

## File header (top of every .c / .cpp / .h / .hpp / .inc that has a comment at all)

Structural and functional.  Which subsystem the file is, what it owns (state objects, tables), the entry points other files use, how the
data flows through it, how it relates to its neighbours.  Not a changelog, not a derivation.

    /**
     * @file v32_scrambler.c
     * @brief V.32 / V.32bis scrambler and descrambler.
     *
     * Implements the self-synchronising 23-bit scrambler of V.22bis/V.32 ... entry points ... callers ...
     */

## Function comments (before every function that has a comment worth having; all non-trivial and all non-static functions)

Functional: what the function computes or does, when it is called, what it reads and writes, units and fixed-point formats, preconditions,
ownership of buffers, return value meaning.

    /**
     * @brief Run one symbol interval of the receive equalizer.
     * @param fse   equalizer state, created by FPM_FSE_init()
     * @param in    complex input samples, Q15, 2 * span entries
     * @return number of output symbols produced (0 or 1)
     * @note  ...
     */

Use `@brief`, `@param`, `@return`, `@note`, `@warning`, `@see`.  Struct and enum members: `///< ...` after the member (the one standard form; `/** ... */` blocks for everything else).

## Inside the code

Higher-level intent: what this block / phase / state is doing and why, not a restatement of the line.  Delete comments that only repeat the
code, that cite offsets/addresses of the original image, or that record a debugging story.  Keep (reworded functionally) any comment that
protects a real hazard: aliasing rules, 32-bit-slot layouts that other code relies on, ordering requirements, table units.

## Known defects

Where the code has a candidate defect recorded in `docs/bugs.md`, say so at the place it
occurs, in doxygen form, and say what actually happens:

    /** @bug  `idx` is not range-checked here; a value >= 36 writes past `ring`.  Callers keep it below 36 today (docs/bugs.md, section V). */

`@bug` marks a defect that can misbehave under realistic input or state.  A hazard that only bites when the caller breaks a precondition (unchecked
allocation result, unvalidated host parameter, missing NULL check) is a `@warning`.

Do not fix the defect in a comment-only pass, and do not remove a defect note without removing the defect.

## Mechanics

* Comment-only changes: the code tokens must not change.  Verify with `tools/stripcmp.sh <prev>` (code with comments removed identical) and
  `tools/objcmp.sh <prev>` (all objects identical).
* Lines at most 160 columns; run `tools/format.sh src` (the pre-commit hook enforces it).
* Preserve `#include`/macro text exactly; do not reflow code.
* String literals and identifiers that name the original (a `_Static_assert` message, a typedef called `..._at_m32`) are reworded too.  These are
  the only code-token changes a comment pass may make; list each as a line `path<TAB>old<TAB>new` in `tools/stripcmp.allow` (stripcmp applies the
  substitution to the old side before comparing) and `tools/objcmp.sh` must still say IDENTICAL.

## Paragraphs

Function bodies are broken into paragraphs by single empty lines, following the flow of the function: setup / validation, each processing phase, the state
update, the return.  Put a blank line before each new logical step and before a block comment that introduces one.  Do not split tightly coupled statements
(a load and its use, the lines of one loop, an `if` and its `else`), do not put blank lines inside short functions (about 8 statements or fewer) unless they have
distinct phases, and never use more than one consecutive blank line.  This is whitespace only: `tools/stripcmp.sh` normalises it, `tools/objcmp.sh` stays IDENTICAL
and clang-format keeps single blank lines.
