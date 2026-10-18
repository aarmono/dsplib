# dsplib

A binary-compatible port of the `dsplibs` binary (a closed-source i386 object) that shipped with slmodem, written in C / C++ and built as a
static archive (`libdsplib.a`) for 32- and 64-bit x86.  The primary goal of the port is 64-bit compatibility.

    make BITS=64            # obj64/libdsplib.a
    make install PREFIX=/usr/local

Link it after the host's objects together with `-lstdc++ -lm`.  The host provides the functions listed in `docs/host-interface.md`;
the data structures shared with the host are in `src/core/host_interface.h`.

* `docs/architecture.md` - layout of the sources and modules
* `docs/host-interface.md` - what a host must provide
* `docs/bugs.md` - known defects and behaviours inherited from `dsplibs`
* `tools/` - formatting, source-list checks and the neutrality checks described in `CLAUDE.md`
