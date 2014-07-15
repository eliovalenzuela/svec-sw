FMC_BUS ?= $(shell pwd)/fmc-bus
export FMC_BUS

RUNME := $(shell test -d $(FMC_BUS) || git submodule update --init)

DIRS = $(FMC_BUS) kernel tools

all clean modules install modules_install:
	for d in $(DIRS); do $(MAKE) -C $$d $@ || exit 1; done

include scripts/gateware.mk
