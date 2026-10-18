# tools/ - the library's own tools

| tool | what it does |
|---|---|
| `format.sh`, `hooks/pre-commit`, `install-hooks.sh` | clang-format the sources; the pre-commit hook refuses unformatted staged files under `src/` |
| `port_objs.py` | reads and checks `sources.mk` (every source listed once; link-order list) |
| `declscan.py` | do all declarations of one function (header, local `extern`, definition) agree? |
| `hostiface_check.sh`, `hostids_check.sh` | optional: compare `src/core/host_interface.h` with a host's headers (`HOST_INCDIR`); skipped when not set |
| `gen_views.sh`, `tier_rewrite.py`, `layout_gen.py`, `layout_check.py`, `layout64.py`, `split_region.py`, `member_map_source.py` | generators and checks for the files derived from `layouts/*.layout` |
| `v34_hs_states_gen.py`, `v8_status_gen.py` | enum headers from tables in the sources |
| `stripcmp.sh`, `ident32.sh`, `objcmp.sh`, `o0check.py`, `irdiff.sh` | prove that a change did not alter the generated code (see `CLAUDE.md`) |
| `layoutscan/` | object-layout scan of the sources |

The sections below are the original notes of these tools; they were written while the tools lived in a larger tree, so some wording (chunk and pass
names, `test/` references) is historical.

---------------------------------------------------------------------------------------------------------------

## tier_rewrite.py - T01 / T02: byte-offset accessors -> typed V34Obj members (x86-64 conversion of the V.34 tiers)

`python3 tools/tier_rewrite.py header|slice|rewrite|worklist|check|apply ...` (the docstring has the details).
* `header -o src/v34/v34_object_view.h` regenerates the V34Obj mirror (REAL pointer members) + `V34M_0x<off>` macros from **test/v34obj.layout** (`header --check` verifies; other classes: `--layout test/vpcm/vpcm_flo_modem.layout --class VPcmFloModem --prefix VPFM_ --guard V34CTX_H -o src/v34/v34_flomodem_view.h`, also v34_k56flex_view.h; the former v34_modem_params_view.h is gone since RS6: src/core/dp_params.h is the hand-written one definition of the parameter block).
* **test/v34obj.layout** (T02) = test/v01.layout with the unknown regions that the plain-C tiers address by byte offset split into typed members; HAND-MAINTAINED since V34SUB / V34CLEAN / V34REL (the former `split` / `refine-check` commands and their spec test/v34obj_members.txt were removed in HOUSE; new members: `tools/split_region.py`).  test/v01.layout is NOT edited (the C++ V04 ports and test/v04_diff_main.c name its unknown regions); `member_map.py validate test/v34obj.layout` gives the same DISAGREE as v01.layout, and `member_map.py blob --func '^(<tier functions>)$' --layout test/v34obj.layout --layout-class tagV34Object` is the width check of the new members (0 width disagreements, 3 straddle records = the dword stores over two u16 members at 0x470 / 0xe50; unknown-region accesses 890 -> 359).
* `apply src/v34/<tier file>.c` rewrites `S32(obj, 0x230)` to `S32T(obj, 0x230)` for every offset that is a member start (or `S16TI(obj, 0x232, minSigLevel_, 2)` for an offset inside a scalar member) and proves the result compiles to identical `objdump -dr` at -m32 -O0, -O2 and -O2 -fno-strict-aliasing (restores the file otherwise); `--base obj --base g_obj --header ../src/v34/v34_object_view.h` for harness files.  `worklist` lists the unresolved offsets (opaque unknown-region interiors); `--hazards` lists the x86-64 hazards of the resolved sites.
* **test/v34_base_exceptions.txt**: functions whose `obj` is not the V34 object (tier2 V34agc ... / V34ModulatorProcess, tier10b preempindex): their sites are never touched.  The identity check cannot catch a wrong base with a same-width member: this file is the review record.
* `slice` generates **src/v34/v34_subobjects.h** from test/v34sub.txt: the sub-object structs (V34Rx b=obj+0x264, V34Mod m, V34Tx, V34Echo near/far, V34ToneDet det, V34Rate q, V34Info p, V34Probe R, V34BlkB blk) as slices of the V34Obj mirror with real pointer members; `rewrite --struct rx --base b FILE` (keys rx mod tx echo det rate info probe blkb ctx P k56) rewrites the sites of those bases (macros `S16T_rx` ...).  Only a flat V34Obj and a slice agree on i386; the `slice` report says how many members also have the same relative x86-64 offset (rx: 2 of 142 - the future V34Obj must EMBED the structs).
Caveat of every identity claim: it is object-code identity of gcc 13 at the three flag sets, nothing else.
Self-test: `sh test/test_tier_rewrite.sh` (~20 s, run by hand).


### T04 additions to tier_rewrite.py: the x86-64 harness marshalling layer (test/v34_img.h) and the flat sub-object offsets
* **Idea.** The v34 tier harnesses model the V.34 object as an i386 IMAGE (random templates, raw offsets, word-index diffs hashed into the recorded goldens).  On x86-64 the port works on a real `V34Obj`.  `test/v34_img.h` keeps the model as it is and, around every call of a function under test, copies the image member by member into a native SHADOW (offset map `test/v34objmap.h`, generated by `tier_rewrite.py mapheader`), calls the port with the shadow pointer, and copies back; pointer members are 32-bit words in the image (a low-memory address, zero-extended; an address inside a registered image is translated to the shadow and back) and real pointers in the shadow.  The call-site translation is done by generated shims (`tier_rewrite.py shims HARNESS.c`, block between `/* V34SHIMS */` markers, x86-64 only).  Standalone sub-object buffers of a harness (block B, tone detector, INFO0 record, RX sub-object `b`, modulator `m`) are registered as WINDOWS (`V34I_REGW(buf, size, base32)`); heap objects of exactly 0xb400 bytes are registered by the calloc/malloc macros; callbacks that run while the port works on a shadow call `V34I_SYNC_IMG()` / `V34I_SYNC_NAT()` / `V34I_IMGP(p)`.  All harness additions are `#if defined(__x86_64__)` or macros that vanish at -m32: `sh tools/ident32.sh test/v34_tier*_diff_main.c` must stay IDENTICAL.  Every run re-executes itself once with ASLR off (deterministic addresses).
* **Port side.** `tier_rewrite.py bytes` (see the docstring): `OBJ_BYTES(obj, 0xa48 + i)` -> `OBJ_BYTES(obj, V34OFF(0xa48) + i)`; the sub-object bases (`b`, `m`, `blk`, `q`, `p`, `R`, `det`, `es`, `pf`, `q`) are rewritten with `--sub KEY` to `V34REL(base, abs)` = the native distance inside the FLAT object, so no embedding is needed as long as the base is derived from the object (`b = OBJ_BYTES(obj, V34OFF(0x264))`).  This requires every sub-object base to be 8-aligned natively (all instances of a multi-instance sub-object must have the same shape): `test/v34obj_members.txt` now carries zero-size `align8` markers before every sub-object base (`split` supports `<off> align8 <name>`), after which `tier_rewrite.py slice` reports full agreement of every V34* slice struct with the flat object (V34Rx 146/146 ...).
* **RS9 note.** The accessor macros of the V.34 tier files (OBJ_BYTES, U8 .. PTR, V34OFF, V34OFFI, V34SUBP, the `xxxT` accessors) are no longer in each file: they are in `src/v34/v34_access.h` (and the `&` spelled sub-object accessors in `src/v34/v34_access_sub.h`); `tier_rewrite.py` was a one-shot that inserted macros after the file's own `#define OBJ_BYTES` line and finds none now.  V34ACCESS: the live V.34 / vpcm files no longer use the `xxxT` accessors, `V34OFF*`, `V34SUBP` or `OBJ_BYTES` except for the untyped records and walks listed in the journal section V34ACCESS; `v34_access_sub.h` is used by the dead functions only and the per-file twin `v34_access_sub_off.h` was deleted.
* `tier_golden64.sh` diagnostic recipe used throughout T04: `TG64_KEEP=1 TPREFIX=test/tgx_X sh test/tier_golden64.sh X` keeps the binary; `TG64_M=-m32` builds the same harness as 32-bit (self-test PASS); dump the image after one call in both builds and compare the words (only pointer words may differ).

### T05 additions to tier_rewrite.py: V34Obj embeds the sub-object structs; `embed`; ctx / cfg images
* **V34SUB (supersedes the union described next for every sub-object except the INFO / bit-queue records)**: the `sole <key> ...` line of test/v34sub.txt makes a sub-object a named member of V34Obj (`V34Rx rx;`, `V34Echo echo0, echo1;`; blkb nested in V34Rx, pf nested in V34Mod) and its struct the ONLY definition of the members: no flat member list, no agreement asserts (one i386 base+size assert per sub-object instead); the `V34M_0x..` / `V34M_<name>` keys expand to the member path (`rx.rxW0_`, `echo1.echoBuf_`: the second instance uses the names of the first), the struct takes the C++ member types of test/v34obj_cxx.txt (`type`, `name` markers; the former `alias` directive is gone: V34CLEAN removed the twelve unsigned aliases, the plain-C sites that need the unsigned view of an int16_t member use `AS_U16(lv)`) and `paths` writes test/v34obj_paths.txt (`<member> <path>`), which `layout_gen.py --paths` (run by gen_views.sh) uses so that v34_object_layout.inc / the probe say `offsetof(V01_tagV34Object, det.toneDetF6_)`.  `mapheader` uses the paths too.  V34CLEAN: the INFO / bit-queue records (info0a info0d info info1c q2 mp) and `retrain` are sole too, there is no union left in the view; `pathname <member> <path>` of the cxx file names a layout member of the C++ class that is a whole sub-object or lies in a later instance (toneDet_ = det, echoNear_ = echo0, echoFar_ = echo1, unk9144_ = echo1.unk80c4_) in the paths file without a marker in the struct.
* **`header`** (v34_object_view.h) used to EMBED the sub-object structs of src/v34/v34_subobjects.h (`#include "v34_subobjects.h"`): every instance of test/v34sub.txt whose shape equals the first one and whose base is a member start becomes an anonymous `union { struct { <flat members> }; V34Rx rx; };` (nested: `blkb` inside `rx`, `pf` inside `mod`), so `obj->rx.rxEnd_` and `obj->rxEnd_` are the same lvalue.  Members: rx, mod, tx, echo0, echo1, det, rate, info (INFO1a), info0a, info0d, info1c, q2, mp (the five other bit-queue records got their own keys/structs in test/v34sub.txt: they have the same pointer-free 0x30-byte shape but other member names), probe (needs members at 0xa320 / 0xa76c: carved in test/v34obj_members.txt), blkb, pf.  One `_Static_assert` per sub-object member proves, on i386 AND x86-64, that the sub-object struct and the flat members sit at the same relative offset (`--no-embed` generates the old flat header).  A trailing `align8` marker (0xe50) makes the tail padding of V34BlkB identical in the flat object and the slice; `slice` keeps interior markers.
* **`embed FILE... [--apply] [--funcs F...]`** rewrites the T04 `V34REL(B, ABS)` / `V34RELI(B, mem, d)` / `V34RELJ` / `V34RELIJ` sites (native distance inside the flat object) by the sub-object struct selected from the base B: `ACC(base, V34REL(B, ABS))` -> `ACCT_key(base, 0xREL)` (typed member accessor of the sub-object, `member` spelling, `offsetof` spelling when only -O0 differs), any other use -> `(int)offsetof(V34Rx, member)` (+ delta inside an unknown-region member); the unused V34REL helper macros are dropped; `--apply` checks 32-bit identity (-O0/-O2/pinned) against the pre-edit text and tries the three spellings in order.  **`embed --bases`**: `OBJ_BYTES(obj, V34OFF(0xB))` of an embedded sub-object -> `V34SUBP(obj, rx)`.
* **`rewrite/apply --struct KEY`** knows the extra external classes `cfg` (V90Parameters of test/vpcm/pcm_parameters.layout, src/v34/v34_v90params_view.h, the block behind `*(ctx + 0x610c)`), `v90i` / `v92i` (src/v34/v34_v90_phase2_view.h / v34_v92_phase2_view.h, the info records behind ctx+0x1760 / 0x612c).
* **`mapheader --class C --name N --inc H -o OUT`** also generates the offset map of other classes (test/v34ctxmap.h = VPcmFloModem, test/v34cfgmap.h = V90Parameters); test/v34_img.h marshals ctx / cfg IMAGES like the V34 object (`V34I_REGC(p, n, ctx|cfg)`, the harness keeps 32-bit pointer slots in them: `PTRX_SET`).

### T06 additions: harness images for the remaining v34 tiers, engine objects, OOB clamp experiments
* **`shims`** names the result variable `v34r_` when a parameter is called `r` (tier 5d's `getbit(void *r)`); otherwise unchanged (the other 12 blocks are byte-identical).  A `V34P6T`-style type macro in a prototype (a pointer on x86-64, `uint32_t` at -m32) hides the parameter from the `*` test, so such a parameter is passed through unchanged (3c initV34 `p6`, 3ab/4b/9a detectorinit `p2`, 5d `P6T`).
* **test/v34_img.h**: (1) shadows are mapped at INCREASING addresses (0x100000000000 upwards) in registration order - the harnesses compare pointers into different image buffers (7a: the RX ring pointer of the template `o` against a member of the called `w`), so the order of the shadows must follow the order of the images (mmap's own top-down placement reversed it); (2) a pointer to ANY registered image (also an obj-class one) marshals that image on demand (7a: ring pointer of `o` stored in `w`); (3) six more classes for the V90 / V92 engine chain nodes of src/v34/v34_engine_views.h (`e90 e92 ers erc edm emg`, maps in the hand-written test/v34engmap.h; `V34I_REGC(p, n, e90)` ...).
* **Raw (unmarshalled) calls** (`V34REAL_<f>` cast to the prototype): functions that work on a record / vector at an arbitrary offset of the image and address no V34Obj member run on the image directly (10a `resetRetrainDetector`, 10b `scaleVector`, `preempindex`); block-relative `initG248` runs on a copy of the 0x450 bytes registered as the block window (10b `initG248_blk`).
* **OOB clamp experiments** (not committed; how the designed xfails of NOTES.md T06 were proven): a scratch copy of the harness with an env-guarded `getenv("TnX_CLAMP")` restriction of the garbage indices, built twice with /tmp-style scripts (`-m32` and `-m64`, same port files, no blob), then `diff` of the two outputs: 0 lines in every case (list in NOTES.md).

---------------------------------------------------------------------------------------------------------------

## o0check.py - the O0 identity checker (RS1, 2026-10-06)

**The criterion it implements (user decision, RS1).**  A restructuring of the port is *identical* when (1) the i386 struct layouts are
unchanged and (2) the `-m32 -O0` assembly of every function is unchanged, with symbol names / relocation targets normalised.  It replaces
the per-file `ident32.sh` for anything that touches many files (ident32 compares one file at a time, with its own flags, and treats a
rename as a difference; see the restructure analysis).

```
tools/o0check.py                         # REF = tag dsplibs-port-switchover vs the working tree: -m32 -O0 (fatal) + -m64 -O0 (fatal unless expected)
tools/o0check.py -r HEAD~1               # just the last commit's effect
tools/o0check.py -r A --work-ref B       # two commits, no working tree involved
tools/o0check.py --o2                    # + the production i386 recipe at -O2 (pinned), ADVISORY (never fatal)
tools/o0check.py -r HEAD~1 --o2-only    # ONLY the production -O2 recipes (-m32, -m64), FATAL; ~22 s; the gate for passes meant to be neutral at -O2
tools/o0check.py --no-m64 | --m64-advisory | --no-layout
tools/o0check.py --map renames.tsv --file-map moves.tsv      # identifier renames (old<TAB>new) / source moves (old<TAB>new)
tools/o0check.py --self-test             # see below
```

* **Builds.**  Every production TU of the library, exactly as `dsplib.mk` compiles it (RS5: through the library's own `Makefile`; `--via slmodemd` selects slmodemd/Makefile, `auto` (default) uses the library's when the tree has it - refs from before RS4 step C do not; TU labels are spelled `src/...` in both cases; byte-identical objects, see the porting journal RS5): the commands come from
  `make -n -B BITS=32|64 PORT_OPT=-O0 OBJDIR=<scratch> <port objects>` of the tree being checked (PIN_FLAGS from `test/flags.sh`,
  `-w -fno-stack-protector -fno-pie`, C++ `-fno-exceptions -fno-rtti`, `v04*` + `V04_DEFS`; only PORT_OPT and OBJDIR are overridden).  The
  REF tree is `git archive REF slmodemd/Makefile slmodemd/*.h the repository root/test/flags.sh` + the library (the source list and the directories it names: since RS4 `sources.mk` + `dsplib.mk`, RS3 `slmodemd/port_objs.mk`, before that the flat library directory) in the
  scratch directory (default `$TMPDIR/o0check-$USER`; `--scratch`), and its results are cached per (commit, configuration, compiler
  versions, checker version).  The layout build is a second object set with `-gdwarf-4 -fno-eliminate-unused-debug-types` appended to
  PORT_OPT (types used only through `offsetof` leave no DWARF otherwise); the code compare never sees debug flags.
* **Functions.**  `objdump -d` per FUNC symbol (aliases at one address = one function `a=b`); addresses function-relative; branch / call
  targets as `<.+off>` (itself) or `<function+off>`; relocations from the ELF, symbol names through `--map`, section-relative targets resolved
  to the function / object / string they point at (x86-64 RIP bias and the i386 in-place addend handled, the i386 REL addend printed in an
  operand is replaced by the resolved target); gcc's numbered statics (`x.3`) unnumbered.  Header line = bind, visibility, size, section
  kind.  Reported: added / removed / changed, a unified diff, and a classification of each change: `constants only: 0x7e42->0x7e46 x1, ...`
  (same instructions, only displacements / immediates differ) or `structural change`.
* **Data.**  Every allocated non-code section except `.eh_frame` (size, flags, bytes with relocated words masked, resolved relocations, the
  OBJECT symbols), plus the set of undefined symbols.
* **Layouts.**  DWARF of every TU: every named struct / class / union (typedef names of anonymous ones, nested names qualified, typedef aliases
  of records), sizeof + member offset / size / kind, compared per TU.  UNCHANGED (reported as NEUTRAL if the member list differs) = same sizeof
  and every member keeps offset and size, except renames (same offset and size) and opaque byte arrays (`uint8_t[]`, zero-size markers)
  refined into members that cover exactly the same bytes, and (RS9) ALIAS collapses: a vanished member whose offset and size equal those of a member that is present in the
  work tree, or whose bytes are all named by members unchanged on both sides (`union { int w1218; int ans_org; }` -> `int ans_org;`; `union { int w00; FaxSgd *sgd; }` -> `FaxSgd *sgd;`, 4 B in a 4 / 8 B slot; `union { struct { short s08, s0a; }; int w8; }` -> `short s08, s0a;`) adds and removes no bytes (reported as `alias removed`).  Everything else is CHANGED.  A type that exists in only one tree is INFO.
* **Renames of static helpers** (`--map`, RS8 default `tools/o0check_renames.tsv`, old TAB new, plain and C++-mangled spelling each): applied to the REF side, so a
  duplicate helper that was unified under another name (`absWrap` -> `absI32`, `fbits` -> `bitsFloat` ...) is not a difference; a rename is neutral only if every definition
  of the old name in the library is renamed too (the map is global).
* **Member renames of ONE type** (RS21: `member` rows of `tools/o0check_unified.tsv`, `member TAB type TAB old TAB new TAB reason`): the member `old` of the REF type `type` is
  renamed to `new` before the layout comparison.  Needed where several name sets overlay the same bytes (the dp parameter block: `DpRuntime`, `P07`, `V01` names): the generic
  rename pairing (one REF-only and one WORK-only member at the same offset and size) cannot choose between the several WORK-only names there, so a rename inside such a type
  was reported as a changed layout (RS20).  Offset and size are still compared: a member that moves or is resized under the new name is reported (negative control in the RS21 journal).
  `--map` renames a name in every type; these rows are type-qualified.  A row is harmless against a REF that already has the new name.
* **Renames of the V.34 module object's members** (RS22, `tagV34Object` / `V34Obj`): no `member` row was needed - the object has one name set, so the generic pairing of the
  checker finds every rename by offset and size.  The names live in `test/v34obj.layout` (the name column; the generated views `src/v34/v34_object_view.h`,
  `v34_subobjects.h` and `test/v34objmap.h` come from it: `sh tools/gen_views.sh`, `--check` verifies), in `test/v01.layout` (written `old=new`, so the harness mirror
  `test/v01_layout.h` and the recorded dumps keep the old name; then `tools/layout_gen.py test/v01.layout --chunk v01 ...` with the command of its header regenerates
  `src/v34/v34_object_layout.inc` and `test/v01_layout_probe.cpp`) and in `test/v34obj_cxx.txt`, plus the identifier in `src/`.  The generator also writes a DESCRIPTIVE KEY next
  to every `#define V34M_0x3594 rxState_` of a member whose name is not just an offset (`tools/tier_rewrite.py` `OFFSET_STYLE_NAME`): `#define V34M_rxState rxState_`, and the same
  for the sub-object families (`V34RX_rxFlags`, `V34TX_txIdx` ...) and the external views (`VPFM_v92Mod`, `V34CFG_sensitiveIspDetected`).  The accessors paste the key, so
  `S16T(obj, rxState)` and `S16T(obj, 0x3594)` are the same expression; a member renamed in the layout gets its key at the next `gen_views.sh`.
* **Expected differences** (`tools/o0check_expected.tsv`, `width kind tu name reason`): allowed only for `m64` (and the advisory `m32o2`),
  never for `-m32 -O0`.  Each entry is a deliberate x86-64 fix with its reason; entries not observed are listed (not an error).
* **Exit status:** 0 identical (within the expected list), 1 unexpected difference, 2 build / usage error.
* **Self-test** (`--self-test`, 7 cases, all must PASS): (1) REF vs REF with every width and `--o2`: clean; (2) a one-line change
  (`c->maxCycles = 100` -> `101` in tone_cadence.c): detected, and exactly `cadence_create` is reported; (3) a comment change + a local-variable
  rename: clean; (4) a static function rename without `--map`: detected (`absd` removed, `abs_diff` added, the caller changed); (5) the same
  rename with `--map absd<TAB>abs_diff`: clean; (6) a struct member rename (the Cadence member at +0x26c, `flag26c` up to RS2 and `reportTone` since: RS8 looks the name up in the REF tree, so
  the case works with any REF, `-r HEAD` included; in tone_detect.h + tone_cadence.c): clean, layout NEUTRAL in the 15 TUs that see Cadence; (7) that member
  `int` -> `short` + `short pad`: detected as a layout change (and the code changes).  The self-test passes `--map ''`, so the default rename map does not apply to it.
* **Cost** (8 cores, gcc 11.4, 212 TUs): ~25-35 s wall for -m32 + -m64 code and layouts with a cached REF (~60 s with a cold REF or `--o2`);
  the layout DWARF extraction (pyelftools) is about two thirds of it.
* **Limits.**  Per-function identity, not whole-object identity: function ORDER inside a TU and `.eh_frame` are not compared (function order
  matters for the final link and for -O2 IPA, see the restructure analysis E2); `-m32` section-relative code references with a zero addend
  stay `section+0`; one compiler (the one on PATH; the cache key includes its version); DWARF covers what the TU declares (with
  `-fno-eliminate-unused-debug-types`: everything its headers declare).

## layoutscan/ - object-layout scan (RS1)

`sh tools/layoutscan/scan_layouts.sh [OUTDIR]` (default `$TMPDIR/layoutscan-$USER`; `PORT=<dir>` scans another tree's include root with the RS4 layout (src/); the
TU list is sources.mk, objects and probes are named by the file stem; `SKIP_BUILD=1` reuses the objects).  RS4: the pair list names the new stems.  Read-only on the repository.  Builds every TU with the production recipe at
-O2 plus `-gdwarf-4` (-m32 and -m64) and one probe TU per view header, lists every record type from DWARF (`dwarf_types.py`), then:
`same_name.py` (one type name with two layouts in one program), `find_views.py` (heuristic: differently named types sharing members at the same
i386 offsets), `cmp_by_offset.py VIEW REAL [--base N]` (every leaf of a view paired with the leaf of the real type at the same i386 byte; DISAGREE
= a different x86-64 position) for the enumerated view pairs, `v34_rawaudit.py` (raw `obj->member + K` sites of the V.34 C++ files vs the C
tiers' layout) and `summary.py` (one line per pair; diff two summaries to see what a change did).  The pair list is in scan_layouts.sh.
Origin: the read-only layout scan done before RS1 (four x86-64 mismatches M1 - M4, NOTES.md RS1).  ~2 minutes.

## port_objs.py - the library's source list (RS3; RS4: sources.mk + the build fragment dsplib.mk)

`sources.mk` (RS3: `slmodemd/port_objs.mk`; included by the library's build fragment `dsplib.mk`, which slmodemd/Makefile includes) lists every library source in LINK ORDER (`DSP_SRCS`, paths below dsplibs-port) and the
per-source attributes (`DSP_V04_SRCS`: compiled with `V04_DEFS`); `DSP_INCDIR` is the include root.  It replaced the former `$(wildcard <dir>/*.c) $(wildcard
<dir>/*.cpp)` over the flat directory, whose order depended on the file names and on the caller's locale (GNU make 4.3 sorts the wildcard with LC_COLLATE: en_US.UTF-8 and C
order 13 of the 212 objects differently), and the `v04%` filename rule.  The list was generated in the en_US.UTF-8 order of the former wildcard, so the
binaries did not change (NOTES.md RS3).  `python3 tools/port_objs.py check` (every listed file exists once, objects unique, V04 list a subset, every
.c / .cpp below the include root listed), `list` (`path kind` per source, kind c / cpp / v04, link order; for scripts that build every library
source), `incdir`; `lib_files()` (python) = every file below the include root.  Renaming a source = editing its line in place (the position is the link position); a new source = a new line (any new object changes the
binary; where it goes in the link order is a decision).

## gen_views.sh, test/v34obj_cxx.txt and the one definition of the V.34 object (RS5)

`sh tools/gen_views.sh [--check]` regenerates (or verifies) the generated C views of the V.34 chain: `src/v34/v34_object_view.h` (`tier_rewrite.py header --cxx
test/v34obj_cxx.txt --tag tagV34Object`), `v34_subobjects.h` (`slice`), `test/v34objmap.h` (`mapheader`) and the external-class views (`v34_flomodem_view.h`,
`v34_k56flex_view.h` (RS6: `v34_modem_params_view.h` was replaced by the hand-written `src/core/dp_params.h`) (`--tag K56FlexHandle`), `v34_v90params_view.h`, `v34_v90_phase2_view.h`, `v34_v92_phase2_view.h`).  test/v34obj.layout is maintained by hand (the `split` / `refine-check` commands were removed in HOUSE).

V34REL: `gen_views.sh` also runs `python3 tools/layout64.py [--check]`, which writes `src/v34/v34_object_layout64.inc` (included at the end of `v34_object_view.h`): the x86-64 layout
asserts of the V.34 object - `sizeof(V34Obj)`, base and size of every embedded sub-object and the offset of every pointer member, numbers probed with `gcc -m64`
(`-DV34L64_PROBE` keeps the probe from reading the file being generated).  The i386 offsets stay the `V34VIEW_ASSERT` blocks of the generated headers.
`python3 tools/split_region.py SPEC.py` (SPEC.py: the region as `test/v34obj.layout` has it and the members to cut out of it: offset, name, kind i16 / u16 / i32 / u32, count) splits an
unknown region of a sub-object into members: the layout lines, the `type` / `name` lines of `test/v34obj_cxx.txt` and the plain-C sites (accessors `S16(b, (int)offsetof(S, region) + N + K * i)`,
key sites `S16T_rx(b, 0x1c0)`) in one go; sites it cannot place are printed (`REPORT`), OBJ_BYTES / V34OFFI uses and C++ `ld16` / `st16` / `st32` uses are done by hand.  Run `sh tools/gen_views.sh` afterwards.  `BASES` (top of the site rewriter) maps each struct to its sub-object prefix and 32-bit base (`V34Obj`, `V34Rx`, `V34Tx`, `V34Mod`, `V34Info`, `V34Rate`, `V34BlkB`, `V34ToneDet`, `V34PreFilter`, `V34Probe`); a new struct needs an entry.  The spec line of a region that carries a trailing `# comment` in `test/v34obj.layout` must be stripped first (the tool matches the whole line); the tool also rewrites `src/v34/v34_unused.c` (on hold: `git checkout` it afterwards).

The struct of `v34_object_view.h` is the plain-C `V34Obj` of the tiers AND the C++ class `tagV34Object` (`src/v34/v34_object.h` includes it).  `--cxx FILE` adds what the C++
ports need on top of the tiers' layout (syntax in `tier_rewrite.py`, class `CxxSpec`): `type <member> <ctype>` (typed pointers, signed 16-bit words), `name <member> <ctype>
<offset>` (a zero-size marker array `uint8_t name[0]` at that offset: the C++ name of a region the tiers split into members; same address, no storage),
`pathname <member> <path>` (V34CLEAN: the layout name of a sub-object base, for the paths file only), `record <member> <struct ctype> <offset> <size>` (an anonymous union over the flat members with a struct from `src/v34/v34_records.h`; no record is declared any more, the bit-queue records and retrain are sole sub-objects; the flat members with a designator `<member>.<m>` in the layout are asserted to sit at `offsetof(<struct>, <m>)`), `include` / `fwd` (headers and forward
declarations for the typed pointers).  Every member / marker that cannot be applied is an error.  The generated asserts use `V34VIEW_ASSERT` (`static_assert` in C++, `_Static_assert` in C), defined once in the hand-written `src/v34/v34_view_assert.h` that every generated view includes (RS10; the generator emits the include, `ASSERT_MACRO`).

Consequences for the checker: `o0check.py` compares the type `tagV34Object` of the tag (hand struct with `V34_PAD64` pads) with the generated one.  The layout verdict
therefore also accepts (a) WORK-only members whose bytes belong to members that exist unchanged on both sides (overlay by an anonymous union, zero-size markers, new
record members), (b) bytes of vanished REF members named `pad*` that are compiler padding on the WORK side (x86-64 only).  The advisory `--o2` adds `m64o2`, the
production x86-64 recipe at -O2: a difference there (found with `MEMBER_ELEM_FLAT`, see src/core/dsp_access.h) is invisible to the -m64 -O0 gate.  The strongest check
of a restructuring step is `o0check.py -r HEAD --o2` (the previous commit as the reference): all four configurations (-m32 / -m64 x -O0 / -O2) must report 0 changed
functions and 0 differing data sections.

## objcmp.sh - whole-object comparison, the order-sensitive complement of o0check.py (RS8)

`sh tools/objcmp.sh REFA [REFB]` builds the library of two states (git refs; REFB defaults to the working tree, uncommitted changes included) with the production
recipe at `-m32` / `-m64` x `-O0` / `-O2` (the library's own Makefile, `obj$BITS/libdsplib.a`, mkdtemp scratch trees) and compares, object by object, the sha256 of
`objdump -dr` + `objdump -s -r -h` (the identifier renames of `tools/o0check_renames.tsv` applied to both dumps first).  `o0check.py` compares function by function and
cannot see the ORDER of the functions in an object; this can.  Measured (RS8): swapping two adjacent `static inline` definitions of voice_engine.c leaves o0check
`-r HEAD` IDENTICAL and objcmp reports voice_engine.o DIFFERENT at -O0 (-m32 and -m64); an unused plain `static` helper pulled in through a shared header shows as a
function ADDED in o0check and in objcmp.  Use it for every step that moves a definition (headers, dedup).  212 objects x 4 configurations, about 25 s per state.

## v34_hs_states_gen.py, v8_status_gen.py, hostids_check.sh (RS12)

* `python3 tools/v34_hs_states_gen.py [--check]` writes `src/v34/v34_hs_states.h` (`enum V34HsState`, `V34HS_<name>` = the 87 strings of the blob's
  `StateName[87]` table in `src/v34/v34_init_tables.c`, `V34HS_COUNT`).  `--check` exits 1 if the committed header differs from the table text and compiles the table
  with the header to verify `StateName[V34HS_x]` is the string `x` for every x.  The header is generated: change the table, run the tool.
* `python3 tools/v8_status_gen.py [--check]` writes `src/v8/v8_status.h` (`enum V8Status`, the 19 strings of `v8StatusName[19]`, the blob's table, as
  they are in `src/v8/dp_v8.c`; `--check` also requires the static copy in `src/v8/v8.c` to be the same list).  RS15: the same header also holds `enum V8CtrlType`
  (`V8CTRL_START_CM` ... `V8CTRL_CTRL10`, the 11 strings of the static `v8ControlName[11]` in `src/v8/v8.c`, the control types of `V8Control`).
* `sh tools/hostids_check.sh` compile-checks `src/core/host_interface.h` (the port-owned host interface the freestanding C++ vpcm files and the voice files use): it must
  compile alone with `-ffreestanding -nostdinc` as C and C++ (-m32 and -m64) and the ids the C++ files use must have their values; a system include or a wrong value
  must make the check fail.  Compile only, a few seconds.  (Until the host interface moved into the library this script compared a copy of the ids with slmodemd's headers; that
  comparison is now `tools/hostiface_check.sh`.)
* `[HOST_INCDIR=dir] sh tools/hostiface_check.sh` (OPTIONAL, needs a copy of slmodemd's `modem_defs.h`, `modem_dp.h`, `modem_param.h`; default `..`, skips with exit 0 when they are
  missing): builds one program against the host headers and one against `src/core/host_interface.h`, for -m32 and -m64, each printing every enumerator and `DPSTAT_*` /
  `FAX_STATUS_*` / `VOICE_*` value, `sizeof` of the enums and structures and `offsetof` of every member, and diffs the outputs; the program also takes the address of
  `modem_get_param` ... `modem_dp_deregister` into typed function pointers (`-Werror=incompatible-pointer-types`), and every enumerator of the host's four enums must exist in the port header.
  Exit 1 = a difference, 2 = the check could not run.  About 3 seconds.

## irdiff.sh - LLVM IR comparison (pilot, advisory)

`bash tools/irdiff.sh [-O1|-O2] [--bits 32|64] <old-rev> <new-rev>` builds every library source at both commits with clang (`-emit-llvm`, `opt -O1`, value names stripped) and lists the files whose IR differs, each followed by the functions whose IR text changed (`    function: name`; `<module>` = globals / declarations / types); `--selftest` checks that a changed constant is detected.  Inline-asm source-location cookies (`!N = !{i64 ...}`) are ignored.  Needs clang/opt (LLVM 14 tested).  `llvm-diff` is unreliable on loops; the comparison is a text diff of the stripped IR.  Trial: bc6a0ce..93f998d, a77fb9e..93f998d: 212 files identical.

It does NOT replace o0check / objcmp or the golden runs.  Intended use: the relaxed phase (assembly identity no longer holds there), as a guide to where behaviour-relevant differences may appear.  Blind spots: x87 precision and evaluation order, inline asm (passed through as text), clang-vs-gcc front-end differences.

`o0check.py` bounds its scratch directory (the REF cache is ~0.5 GB per distinct commit): `--cache-keep N` (default 4) keeps the N most recently used `ref-*` directories, never one used in the last 2 hours, and removes `work-*` / `selftest-*` leftovers older than a day; `--cache-keep -1` disables it.

## stripcmp.sh - proof that a comment pass changed no code (CM1)

`sh tools/stripcmp.sh REF` compares, for every C / C++ source under `src/` (`.c .cpp .h .hpp .inc`) that differs between the git ref REF and the working tree, the two versions with the comments removed.
Each version goes through `gcc -fpreprocessed -dD -E -P` (comments dropped, string and character literals respected, `#define` / `#include` / `#if` text kept), then whitespace is normalised (runs of
blanks and line breaks collapse, blanks next to punctuation disappear, preprocessor directives stay on lines of their own).  Exit 0 = every differing file is token-identical apart from comments; exit 1 lists
the files whose code differs and any file that was added or removed.  Needs only gcc and git, no build: about a second.  Use it with `tools/objcmp.sh REF` for every comment-only change (the style is in
`docs/comment-style.md`).  Limits: a change inside a string literal or in preprocessor text is a code change and is reported; a change that only moves a comment across a line break is not.

`tools/stripcmp.allow` (optional) is the allowlist for the rewordings a comment pass may make in code tokens (a string literal or identifier that names the original, see `docs/comment-style.md`).  One entry per line,
`path<TAB>old<TAB>new`, with `path` relative to the library directory (`src/cid/cid.h`) and `old` / `new` literal text (no patterns); blank lines and lines starting with `#` are ignored.  Before the comparison every `old`
is replaced by `new` in the REF text of that file (all occurrences, entries in file order), so exactly the listed rewording passes and any other token change still fails.  `tools/objcmp.sh` must still say IDENTICAL
(a reworded literal that lands in an object would show there).  Tested: a changed string literal fails without its entry and passes with it.
