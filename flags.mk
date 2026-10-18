# flags.mk - the pinned compiler flags of the library (included by dsplib.mk; can also be printed: make -s -f flags.mk print-pin).
#
# PIN_FLAGS: -fno-strict-aliasing
#   A safety net, not a fix.  Many sources use raw byte offsets (`*(short *)((char *)p + 0x3e)`), and about half of the objects generate
#   different -m32 -O2 code with this flag, i.e. gcc does use type-based alias analysis in them.  Every known defect has been fixed in the
#   source; the flag stays because tests cannot prove the absence of the same pattern on a path they do not reach.  The hazard is a latent
#   wrong-code risk of the optimiser, not of the algorithm, and the flag removes exactly that class.  Cost: a few percent of speed in code
#   that is not a bottleneck.  Remove it only together with an audit of every object that changes.
#
# -fno-lifetime-dse is deliberately NOT pinned: at -m32 -O2 it changes no function body (only the placement of empty D1 destructors).
#   The one known case (~V92EchoCanceller clears members after the lifetime ends) has an explicit asm barrier in the source.
PIN_FLAGS := -fno-strict-aliasing

print-pin:
	@echo $(PIN_FLAGS)
.PHONY: print-pin
