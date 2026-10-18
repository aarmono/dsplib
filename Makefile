# Build of the DSP library: the objects of every source in sources.mk, compiled with the recipe of dsplib.mk, and a static archive of them.
#   make [BITS=32|64] [PORT_OPT=-O2]   -> obj$(BITS)/<dir>/<name>.o and obj$(BITS)/libdsplib.a
#   make install [PREFIX=/usr/local] [BITS=..]   -> $(PREFIX)/lib/libdsplib.a (libdsplib$(BITS).a with LIBSUFFIX=$(BITS)) and
#                                                    $(PREFIX)/include/dsplib/host_interface.h
# The library needs no host headers (src/core/host_interface.h is its own) but, at link time, the host's sysdep_* / dsplibs_debug_printf /
# modem_* functions (docs/architecture.md, docs/host-interface.md).  Link the archive AFTER the host's objects, with -lstdc++ -lm.
HOSTARCH:=$(shell uname -m)
ifneq ($(filter x86_64 amd64,$(HOSTARCH)),)
BITS?=64
ARCHFLAG:=-m$(BITS)
else ifneq ($(filter i386 i486 i586 i686,$(HOSTARCH)),)
BITS?=32
ARCHFLAG:=-m$(BITS)
else
BITS?=native
ARCHFLAG:=
endif
CC:= gcc $(ARCHFLAG)
CXX:= g++ $(ARCHFLAG)
OBJDIR:=obj$(BITS)

all: $(OBJDIR)/libdsplib.a

include dsplib.mk

$(OBJDIR)/libdsplib.a: $(PORT_OBJS)
	rm -f $@
	ar rcs $@ $^

clean:
	rm -rf obj32 obj64 objnative

PREFIX?=/usr/local
LIBSUFFIX?=
install: $(OBJDIR)/libdsplib.a
	install -d $(DESTDIR)$(PREFIX)/lib $(DESTDIR)$(PREFIX)/include/dsplib
	install -m 644 $(OBJDIR)/libdsplib.a $(DESTDIR)$(PREFIX)/lib/libdsplib$(LIBSUFFIX).a
	install -m 644 src/core/host_interface.h $(DESTDIR)$(PREFIX)/include/dsplib/host_interface.h

.PHONY: all clean install
