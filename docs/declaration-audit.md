# Declaration audit (AUD): library against host, and the library against itself

Question: is every function or variable that one side declares and the other side defines declared with the same type on both sides?  A mismatch is
silent in C (separate translation units, no link-time type check), and it has already cost one real defect: `CID_process` was `void` in the library and
`int` in slmodemd, which read an undefined return register (`CID_process`, fixed).  Evidence labels: **R** = run (a tool or a build output
was read), **C** = read from the code.  Host = slmodemd's `*.c` and `*.h`; the audit was made against a checkout of slmodemd and changed no host file.

Method (all of it re-runnable):

1. **Symbols that cross the boundary (R).**  Build slmodemd (`make BITS=64 slmodemd`), then `nm -u` the host objects against `nm --defined-only` the library
   objects, and the reverse.  Host undefined and library defined: 22 symbols (table 1).  Library undefined and host defined: 21 symbols (table 2); the
   library's other undefined symbols are libc only (`abort fwrite memmove memset stderr`).  No variable of the library is referenced by the host; the
   only host variable the library reads is `dsplibs_debug_level`.
2. **Link-time type check (R).**  Whole program built with `-flto` (library at `-O1 -flto`, host `-O1 -flto`) and linked with `-Wlto-type-mismatch -Wall
   -flto-partition=none`, `-m32` and `-m64`.  Result on the final tree, both widths: 2 `-Wlto-type-mismatch` (`VPcmV34ReportStartOfEchoAdapt` /
   `...MiddleOfEchoAdapt`, parameter 1, deliberate, table 5) and 2 `-Wodr` (`shaper_reset_words`, `shaper_resetSSFilter_words`, the asm-label calls of
   `v90_mapper.cpp`, deliberate).  **The check has blind spots:** it did not report `dcr_process` / `prop_dp_exit` (return `void` against `int`) or
   `CID_create` (`unsigned` against `int`), which the reading below finds.  It is a net, not a proof.
3. **Declaration scan of the library (R).**  `python3 tools/declscan.py src` (new, a regex scan over `src/**/*.{c,cpp,h,inc}` that groups file-scope
   prototypes and definitions by name and prints the names whose normalised signatures differ) and `grep -rnE '^\s*extern' src --include=*.c --include=*.cpp`.
   Before the pass 149 names differed with the macros kept (`NOINL` ...), 37 after normalising; after the pass the same 37 remain, all classified in
   tables 4 and 5.
4. **Data pump records (R + C).**  `dp_*.c` compiled with `-Werror=incompatible-pointer-types` at `-m32` and `-m64` (the production flags use `-w`, which
   hides that warning), plus the C++ record of `dp_vpcm.cpp` read by hand.
5. `tools/hostiface_check.sh` (R): the shared records and the function-pointer initialisation of the host prototypes, both widths, OK.

## 1. Host declares, library defines (direction 1)

| name | host declaration (file) | library definition | verdict | action |
|---|---|---|---|---|
| `dp_runtime_create` / `dp_runtime_delete` | `void *(struct modem *)`, `void (void *)` (modem.c:75-76) | `core/dp_runtime.c`, same | same (C) | none |
| `dcr_create` | `void *dcr_create()` (modem.c:77, unprototyped) | `void *dcr_create(void)` | compatible in C (no argument either way) | none; host could write `(void)` |
| `dcr_delete` | `void (void *)` (modem.c:78) | same | same (C) | none |
| `dcr_process` | `void (void *, void *, int)` (modem.c:79) | `int dcr_process(void *, void *, int)` | **return type differs**; the host drops the value, so harmless on x86 / x86-64 (C, R: call site `modem.c:677` is a statement) | library unchanged (changing it alters the body); host-side candidate, section 6 |
| `RD_create` / `RD_delete` / `RD_process` / `RD_ring_details` | `void *(struct modem *, unsigned)`, `void (void *)`, `int (void *, void *, int)`, `void (void *, long *, long *)` (modem.c:81-84) | `core/dp_runtime.c`, same | same (C); `long *` on both sides | none |
| `CID_create` | `void *(struct modem *, unsigned rate, unsigned cid_val)` (modem.c:87) | `CidHandle *CID_create(void *modem, int fs, int fmt)` | pointer / integer ABI identical; **signedness differs** for two parameters; values are 8000 / 9600 and a small S-register (C) | none; changing the library to `unsigned` would change comparisons in the body |
| `CID_delete` | `void (void *)` | `void (CidHandle *)` | same ABI | none |
| `CID_process` | `int (void *, void *, int)` | `int (CidHandle *, const short *, int)` | same (fixed earlier in `c895058`: the library returned `void`) | none |
| `VOICE_create` / `_delete` / `_process` / `_command` | modem.c:92-95 | `voice/voice_api.c`, same | same (C) | none |
| `FAX_create` / `_process` / `_delete` | `void *(struct modem *, unsigned, unsigned)`, `int (void *, void *, void *, int)`, `void (void *)` (modem.c:98-100) | `fax/fax_api.c`, same | same (C) | none |
| `FAX_class1_command` | `int (void *, unsigned, unsigned)` (modem_at.c:824) | same | same (C) | none |
| `prop_dp_init` | `int (void)` (modem_main.c:102, modem_test.c:73) | `int prop_dp_init(void)` | same (C) | none |
| `prop_dp_exit` | `void (void)` (modem_main.c:103, modem_test.c:74) | `int prop_dp_exit(void)` | **return type differs**, the host ignores the value (C; already noted in the function's `@warning`) | library unchanged; host-side candidate, section 6 |

## 2. Library calls, host defines (direction 2)

All 21 are declared once in `core/dsp_host.h` (the registration pair in `core/host_interface.h`); every row compared with the definition by reading it (C) and
by the LTO link (R: no report).

| name | library declaration | host definition | verdict |
|---|---|---|---|
| `dsplibs_debug_level` | `unsigned int` | modem_debug.c:63 `unsigned int` | same |
| `dsplibs_debug_printf` | `int (const char *, ...)` | modem_debug.c:117 | same |
| `modem_debug_log_data` | `int (struct modem *, unsigned, const void *, int)` | modem_debug.c:71 | same |
| `modem_dp_register` / `_deregister` | `int / void (enum DP_ID, struct dp_operations *)` | modem.c:211, 225 | same (hostiface_check R) |
| `modem_get_bits` / `modem_put_bits` | `int (struct modem *, int, unsigned char *, int)` | modem.c:594, 612 (`u8 *`) | same |
| `modem_get_param` / `modem_set_param` | `long (struct modem *, unsigned)`, `long (struct modem *, unsigned, int)` | modem_param.c:57, 180 | same (hostiface_check R) |
| `modem_get_sreg` | `int (struct modem *, unsigned int)` | modem.c:1655 | same |
| `modem_send_to_tty` / `modem_recv_from_tty` | `int (struct modem *, const char *, int)`, `int (struct modem *, char *, int)` | modem.c:1558, 1563 | same |
| `sysdep_malloc` `_free` `_memset` `_memcpy` `_strlen` `_strcpy` `_strcat` `_sprintf` `_vsnprintf` | `dsp_host.h` | sysdep_common.c:54-103 | same (`unsigned` size for malloc / vsnprintf, `size_t` for the rest, as defined) |

Two library-side restatements of the same symbols exist on purpose: `voice_get_param` (`voice/voice.h`, an `__asm__("modem_get_param")` alias with the
same prototype) and the opaque-modem set of `dp_vpcm.cpp` (`void *` for `struct modem *`, `uint8_t *` for the bit buffers; same symbols, same ABI).  The latter
used to be a local extern block of `dp_vpcm.cpp`; it is now the `DSP_HOST_MODEM_OPAQUE` branch of `dsp_host.h` (commit `a1ffd8c`).

## 3. Data pump records (direction 3)

`struct dp_operations` of the host: `create(struct modem *, enum DP_ID, int caller, int srate, int max_frag, struct dp_operations *) -> struct dp *`,
`delete(struct dp *) -> int`, `process(struct dp *, void *in, void *out, int cnt) -> int`, `hangup(struct dp *) -> int`.  The host calls `create` at
modem.c:1051 and 1601 with `(m, id, m->caller, m->srate, m->frag, op)`, `delete` unconditionally (1065, 1194), `process` at 646 (result switched on
`DPSTAT_*`), and `hangup` only after a non-NULL test (431, 446, 463).

| pump | create / delete / process / hangup | verdict |
|---|---|---|
| b103 (`fsk/dp_v21_b103.c`), v23 (`fsk/dp_v23.c`), v22 (`v22/dp_v22.c`), v32 (`v32/dp_v32.c`) | `*_create(struct modem *, enum DP_ID, int, int, int, struct dp_operations *)`, `*_delete`, `dp_wrapper_run`, `0` | typed initialisers; `-Werror=incompatible-pointer-types` clean at -m32 and -m64 (R); `hangup` NULL is guarded by the host (C) |
| call progress (`callprog/dp_call.c`) | `call_create`, `call_delete`, `call_run`, `0` | same (R) |
| V.8 (`v8/dp_v8.c`) | `v8_create`, `v8_delete`, `v8_process`, `0` | same (R) |
| VPCM for V.34 / V.90 / V.92 (`vpcm/dp_vpcm.cpp`) | `vpcm_create(void *, int, int, int, int, void *) -> void *`, `vpcm_delete(Z01_State *) -> int`, `vpcm_run(Z01_State *, short *, short *, int) -> int`, `0` (stored as `void *` in the C++ copy of the record) | signatures read against the host's (C); no compiler check is possible for the `void *` slots.  `vpcm_run` returns 1 (link up, = `DPSTAT_CONNECT`), -1 on error (not a `DPSTAT_*` code; the host's default branch maps it to `STATUS_ERROR`, which the host treats like `STATUS_DP_ERROR`, modem.c:530-531) or 0 |

The records themselves (`struct dp`, `struct dp_operations`, `struct dsp_info`, the ids) are compared member by member and by `sizeof` / `offsetof` by
`tools/hostiface_check.sh` (R, both widths).

## 4. Local declarations inside the library

`extern` / prototype declarations in `.c` / `.cpp` files whose definition is in another file, before and after.  "Header" is where the one declaration
now lives.

| name(s) | was declared in | header now | commit |
|---|---|---|---|
| `V34DisconnectThreshTable` | vpcm_create.cpp, vpcm_info1a_retrain.cpp | v34/v34_api_tables.h | 1179979 |
| `vect4`, `vect16` | vpcm_phase34_reneg.cpp, v34_hs_tx_small.cpp, v34_hs_tx_xmitmp.cpp, v34_handshake.h | v34/v34_symbol_tables.h | 1179979, 3e68309 |
| `StateName` | v34_hs_tx_small.cpp, v34_handshake.h | v34/v34_init_tables.h | 3e68309 |
| `hsine1600 ... hsine2000` (8 tables, unsized there, sized in the header) | v34_hs_tx_trnseg4, _rx_receive, _rx_det_info, _rx_answer, _rx_phase2_call | v34/v34_init_tables.h | 3e68309 |
| `c1200_`, `c2400_` | v34_hs_tx_l1, _rx_moh_tone, _rx_det_info | v34/v34_frame_tables.h | 3e68309 |
| `scale2400 ... scale3429` | v34_hs_tx_l1 | v34/v34_symbol_tables.h | 3e68309 |
| `vectpp` | v34_hs_tx_mid | v34/v34_receiver_tables.h | 3e68309 |
| `probe` | v34_hs_tx_mid | new v34/handshake/v34_hs_tables.h (the defining file includes it) | 3e68309 |
| `getbit`, `ApplyBulkDelay` (regparm(3) at -m32 unless `V04_REAL_HELPERS`) | v34_hs_tx_dpsk, _rx_answer, _rx_phase2_call | v34/handshake/v34_handshake.h | 72f8568 |
| `VPcmV34ReportStartOfEchoAdapt`, `...MiddleOfEchoAdapt` | v34_hs_tx_trnseg4.cpp | v34/handshake/v34_handshake.h (still `(tagV34Object *)`, definitions `(void)`, deliberate) | 72f8568 |
| `callprog_fptone` ... `callprog_default_cfg` (8) | callprog_dual_tone.c, callprog.c | new callprog/callprog_tables.h | 311e269 |
| `tone_amode_cfg` | voice_tone.c | voice/voice_tone.h (next to `TONE_CFG`) | 311e269 |
| `V32_SYMBOL_LEN` | v32_datapath.c | v32/v32_tables.h | 311e269 |
| `Vparser_read_int`, `Vparser_read_float` | pcm_leaf_classes.h and pcm_parameters.h | new vpcm/vparser.h | 9c88cf1 |
| `modem_get_param` ... `modem_dp_deregister` with `void *` | dp_vpcm.cpp | core/dsp_host.h, DSP_HOST_MODEM_OPAQUE branch | a1ffd8c |
| the five v34 `*_tables.h` and `v8_tables.h` | had no `extern "C"` guard, so a .cpp could not include them | guard added | 1179979 |

Left where they are, on purpose (nothing to merge or the use is a test hook):

* `getMPrecvdBits`: three views of one routine.  `vpcm/v34_modem_init.c` defines the C function `void getMPrecvdBits(void *)`; `vpcm_getmp_alias.cpp`
  defines a C++-linkage forwarder `getMPrecvdBits(tagV34Object *)` carrying `regparm(1)` at -m32; `vpcm_phase34_reneg.cpp` declares that forwarder with the
  same attribute.  The attribute is kept.
* `shaper_reset_words`, `shaper_resetSSFilter_words` (`v90_mapper.cpp`): an asm-label declaration of the C++ members `V90SpectralShaper::reset` /
  `resetSSFilter` that takes the four float coefficients as raw words (`-m32`) or through `memcpy`-ed floats (`-m64`).  `-Wodr` names them; deliberate.
* `tr_*` (`v34_modulator.c`), `port_trace_*` (`v34_frame_decoder.c`, `v34_receiver.c`): test hooks under `#ifdef PORT_TRACE_INDEX`, defined only by a
  test harness.
* `fltTable_1` / `fltTable_2` (`v92_cp.cpp`) and `fltTable1` / `fltTable2` (`pcm_cp_packers.cpp`): all four are declared in `v92_cp.h`; there is no type
  mismatch, but the two pairs are separate copies of the same values.  A test harness byte-compares `fltTable1` / `fltTable2` with the original's, so the pairs stay separate.

## 5. Mismatches that remain inside the library (final scan, 37 names)

| names | the two declarations | verdict | action |
|---|---|---|---|
| `V34EchoAdapt` ... `V34TimingPrefilter` (19, `v34/v34_dsp_primitives.c`) | `v34_internal.h`: `void *` / `short *` parameters; the definitions: `EchoAdaptState *`, `EqualizerState *`, `PreFilterState *`, `TimingState *` | the header is not included by the defining file, so the compiler never sees both; ABI identical (pointers) | none; typed prototypes are the direction (typed parameters instead of `void *`) |
| `VPcmV34ReportStartOfEchoAdapt`, `...MiddleOfEchoAdapt` | callers `(tagV34Object *)`; definitions in `vpcm/v34_api.c` `(void)` | a surplus argument in cdecl; a test harness records the object argument, so the callers must keep passing it | none, deliberate |
| `getbit`, `ApplyBulkDelay` | handshake pieces: `short (tag_bitQ *)` / `void (tagV34Object *, int)` with `regparm(3)` for i386 builds without `V04_REAL_HELPERS`; C definitions `int getbit(void *)` / `void ApplyBulkDelay(void *, int)` in `v34_handshake_helpers.c` (cdecl) | in the library build the pieces are compiled with `-Dgetbit=v04_getbit_impl -DApplyBulkDelay=v04_apply_bulk_delay_impl` and call the C++ copies of `v34_hs_getbit_bulkdelay.cpp`, which match; the regparm branch is only for the blob-comparison builds | none, deliberate |
| `getMPrecvdBits` | see section 4 | deliberate | none |
| `VPcmV34Create`, `VPcmV34Progress`, `datapumpv34`, `v34handshak` | `struct tagV34Object *` in the header, `tagV34Object *` in the definition; `struct _tagModemParameters *` against `V01_ModemParams *` | the same types, spelled differently | none |
| `modem_*` (7) | `dsp_host.h` typed branch against the opaque branch | one translation unit sees only one of them | none |
| `calculateDilLength`, `float2Bits` | C++ overloads | not a mismatch | none |

## 6. Candidate bugs (host side, none touched)

All C unless marked R.  None misbehaves today.

1. `modem.c:79`: `extern void dcr_process(void *, void *, int)`; the library returns `int` (0 or a state value).  The value is dropped, so no register is read.
   Declare it `int` to match.
2. `modem_main.c:103`, `modem_test.c:74`: `extern void prop_dp_exit(void)`; the library returns `int` 0.  Same remark.
3. `modem.c:87`: `CID_create(struct modem *, unsigned rate, unsigned cid_val)`; the library takes `int` for both.  Values are in range, so no effect.
4. `modem.c:77`: `extern void *dcr_create();` is an old-style declaration; it means "unspecified parameters" before C23 and `(void)` from C23 on.  Write `(void)`.
5. `vpcm_run` returns -1 for a link error, which is not a `DPSTAT_*` code (`DPSTAT_ERROR` is 4).  The host handles it through its default branch with the same effect as `DPSTAT_ERROR`; a host that switches on `DPSTAT_ERROR` only would not.  Library side, by design.
