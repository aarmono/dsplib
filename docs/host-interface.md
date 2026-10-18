# The host interface: what a host program has to provide

The library is a set of data pumps (V.21 / Bell 103, V.22 / V.22bis / Bell 212A, V.23, V.32 / V.32bis, V.8, V.34 / V.90 / V.92 / K56flex, call progress,
class 1 fax, voice) that a modem program drives.  It does not include any header of that program.  The contract has three parts, all owned by the library:

| part | where | what |
|---|---|---|
| shared data | `src/core/host_interface.h` | ids, status codes and records that both sides read and write |
| host functions | `src/core/dsp_host.h` | the functions the library calls and never defines (resolved at link time) |
| library entry points | `src/core/dp_registry.h`, `src/fax/`, `src/voice/`, ... | what the host calls |

`host_interface.h` mirrors the interface of slmodemd (its `modem_defs.h`, `modem_dp.h`, `modem_param.h`), the host the library was written for.  Numeric values,
member order and structure layout are identical to slmodemd's, and `tools/hostiface_check.sh` proves it against a copy of those headers (see the end).

## 1. Shared data (`core/host_interface.h`)

* **Data pump ids** (`enum DP_ID`): `DP_V8`, `DP_V22`, `DP_V32`, `DP_V34`, `DP_V90`, `DP_V92`, ... A session is created for one id; the host chooses it (dial
  string, S-registers, automatic mode).  The numbers are the host's (they are stored in its S-registers): a host with its own numbering changes the enumerator
  values in the header; the library refers to the ids by name (not re-audited for numeric literals).
* **Status codes** (`DPSTAT_*`): what a pump reports in `struct dp::status` (`DPSTAT_CONNECT`, `DPSTAT_ERROR`, `DPSTAT_CHANGEDP` ...).
* **Data pump records**: `struct dp_operations` (name, use count, `create`, `delete`, `process`, `hangup`) is what a pump registers; `struct dp` (id, modem, status,
  operations, private data) is the per-session object.  They are declared for C only; the C++ pumps (`src/vpcm`) keep their own opaque copy (`src/vpcm/dp_vpcm.h`) because
  `delete` is a C++ keyword.
* **`struct dsp_info`**: the connection summary the V.8 / quick-connect glue fills in (connection type, `long clock_deviation`, quick-connect LAPM bit and index).
  It contains a `long`: 16 bytes on i386, 24 on x86-64; the host must use the same declaration.
* **Fax and voice command sets**: `enum FAX_CMD`, `FAX_STATUS_*`, `enum VOICE_CMD`, `VOICE_STATUS_*`, `VOICE_STATE_*`, `struct voice_info`.
* **Parameter ids** (`enum MODEM_PARAMETER_NAMES`): the argument of `modem_get_param()` / `modem_set_param()`.  The values are positions in the enumeration
  (`MDMPRM_DPRUNTIME` is 10, `MDMPRM_VOICEINFO` 12, `GetDTMFHighToneLevel` 20 ...), so a host that numbers its parameters differently reorders the enumeration to match.

The header has no `#include`, so the C++ half of the library (compiled `-fno-exceptions -fno-rtti` with freestanding headers only) can use it.  `tools/hostids_check.sh`
keeps it so.

## 2. Host functions (`core/dsp_host.h`)

Declared once, with the real prototypes; the library archive's undefined symbols are exactly this list plus libc.

| group | functions |
|---|---|
| memory and strings | `sysdep_malloc(unsigned)`, `sysdep_free`, `sysdep_memset`, `sysdep_memcpy`, `sysdep_strlen`, `sysdep_strcpy`, `sysdep_strcat`, `sysdep_sprintf`, `sysdep_vsnprintf` |
| diagnostics | `dsplibs_debug_printf(const char*, ...)`, `unsigned int dsplibs_debug_level` (read afresh at every use) |
| registration | `int modem_dp_register(enum DP_ID, struct dp_operations*)`, `void modem_dp_deregister(enum DP_ID, struct dp_operations*)` (declared in `host_interface.h`) |
| modem object | `long modem_get_param(struct modem*, unsigned)`, `long modem_set_param(struct modem*, unsigned, int)`, `int modem_get_sreg(struct modem*, unsigned)` |
| bit and character streams | `modem_get_bits`, `modem_put_bits` (`unsigned char*` buffers), `modem_send_to_tty`, `modem_recv_from_tty`, `modem_debug_log_data` |

`struct modem` stays an incomplete type: the library only hands the pointer back.  Parameters that are objects (`MDMPRM_DSPINFO`, `MDMPRM_VOICEINFO`, `MDMPRM_DP_ADDR`,
`MDMPRM_DPRUNTIME`) return their address in the `long` result, so on a 64-bit host `long` must be pointer sized (LP64) and the objects must stay valid for the
session.

## 3. Mapping another host

1. **Types.**  Give the host the declarations of `host_interface.h` (include it, or confirm that the host's own definitions agree: same enumerator values, same member
   order and types).  `tools/hostiface_check.sh` with `HOST_INCDIR` pointing at the host's headers does the comparison if they have the same names; otherwise adapt the
   name lists in that script (the names of the host's headers are in its first lines).
2. **Functions.**  Implement the functions of section 2 with those prototypes (or thin wrappers).  The `sysdep_*` family may be the C library; `dsplibs_debug_printf`
   must accept printf formats; `dsplibs_debug_level` is a plain `unsigned int` variable.
3. **Lifecycle.**  At start call `prop_dp_init()` (it calls `dp_call_init()`, `dp_b103_init()`, `dp_v22_init()`, `dp_v23_init()`, `dp_v32_init()`, `dp_v8_init()` and
   `dp_vpcm_init()`, each of which calls `modem_dp_register` for its ids; `core/dp_registry.c`); at exit `prop_dp_exit()`.  A session is `op->create(modem, id, caller, srate, max_frag, op)`, then `op->process(dp, in,
   out, cnt)` per block of 16-bit samples, then `op->delete(dp)`; `hangup` ends it gracefully.  The fax and voice engines have their own entry points
   (`src/fax/fax_api.c`, `src/voice/voice_api.c`).
4. **Parameters.**  `modem_get_param` has to answer every `MDMPRM_*` / `Get*` id the library reads (the tone detectors and the dialer read the `Get*` settings; the
   V.8 / V.34 glue read the rates, `MDMPRM_IODELAY`, `MDMPRM_CODECTYPE`, `MDMPRM_DSPINFO`).  `modem_set_param(MDMPRM_UPDATE_DELAY)` and `MDMPRM_DP_REQUESTED` are
   written by the pumps.
5. **Build.**  `make BITS=32|64` builds `obj$(BITS)/libdsplib.a` with no host header on the include path; `make install` puts the archive and `host_interface.h` under
   `$(PREFIX)`.  Link the archive after the host's objects, with the host's functions, `-lstdc++` and `-lm`.  `dsplib.mk` is the fragment to include from a host makefile
   that builds the objects itself.

## 4. Checking against slmodemd

    HOST_INCDIR=<dir holding modem_defs.h modem_dp.h modem_param.h>  sh tools/hostiface_check.sh

The script is optional: the library never needs the host headers.  It skips with exit 0 and a message when `HOST_INCDIR` is not set or the headers are missing.  Otherwise it builds the same
table program against the host headers and against `core/host_interface.h`, for `-m32` and `-m64`, and diffs: every enumerator and status macro, `sizeof` of the enums
and records, `offsetof` of every member; it also initialises typed function pointers with the host functions' names under `-Werror=incompatible-pointer-types`.
