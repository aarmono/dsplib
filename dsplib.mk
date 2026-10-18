# dsplib.mk - the build fragment of the DSP library: its sources, flags and object rules.
#
# Use:   DSPDIR := <path of this directory, relative to the including makefile's directory>   (optional: defaults to where this file is)
#        OBJDIR, CC, CXX (with the word-size flag) and optionally PORT_OPT are the includer's;
#        include $(DSPDIR)/dsplib.mk
#        ... link $(PORT_OBJS) or the archive built by the Makefile.
# The object of $(DSPDIR)/<dir>/<name>.c(pp) is $(OBJDIR)/<dir>/<name>.o.  Include this AFTER the includer's default goal: it defines rules.
# The flags (-O2 + PIN_FLAGS of flags.mk, -w -fno-stack-protector -fno-pie, C++ -fno-exceptions -fno-rtti) are the production recipe;
# tools/o0check.py, objcmp.sh and ident32.sh build the library through it.
DSPDIR ?= $(patsubst %/,%,$(dir $(lastword $(MAKEFILE_LIST))))
include $(DSPDIR)/sources.mk

PORTDIR:=$(DSPDIR)/$(DSP_INCDIR)
include $(DSPDIR)/flags.mk
PORT_OPT?=-O2
PORT_FLAGS:= $(PORT_OPT) $(PIN_FLAGS) -w -fno-stack-protector -fno-pie -I$(PORTDIR) -MMD -MP
PORT_SRCS:= $(addprefix $(DSPDIR)/,$(DSP_SRCS))
dsp_objs = $(patsubst %,$(OBJDIR)/%.o,$(basename $(1)))
PORT_OBJS:= $(call dsp_objs,$(DSP_SRCS))
PORT_C_OBJS:= $(call dsp_objs,$(filter %.c,$(DSP_SRCS)))
# the V.34 "V04" pieces (DSP_V04_SRCS) call helpers whose names clash with the cdecl copies of tier 5d; they are compiled with the real helpers
V04_DEFS:= -DV04_REAL_HELPERS -Dgetbit=v04_getbit_impl -DApplyBulkDelay=v04_apply_bulk_delay_impl
PORT_V04_OBJS:= $(call dsp_objs,$(DSP_V04_SRCS))
PORT_CXX_OBJS:= $(filter-out $(PORT_V04_OBJS),$(call dsp_objs,$(filter %.cpp,$(DSP_SRCS))))

# static pattern rules: the per-source flags come from the lists in sources.mk, not from the file names
$(PORT_V04_OBJS): $(OBJDIR)/%.o: $(DSPDIR)/%.cpp
	@mkdir -p $(@D)
	$(CXX) $(PORT_FLAGS) -fno-exceptions -fno-rtti $(V04_DEFS) -c $< -o $@

$(PORT_CXX_OBJS): $(OBJDIR)/%.o: $(DSPDIR)/%.cpp
	@mkdir -p $(@D)
	$(CXX) $(PORT_FLAGS) -fno-exceptions -fno-rtti -c $< -o $@

$(PORT_C_OBJS): $(OBJDIR)/%.o: $(DSPDIR)/%.c
	@mkdir -p $(@D)
	$(CC) $(PORT_FLAGS) -c $< -o $@

-include $(PORT_OBJS:.o=.d)
