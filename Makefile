# include parent_common.mk for buildsystem's defines
#use absolute path for REPO_PARENT
REPO_PARENT=$(shell /bin/pwd)/..
-include $(REPO_PARENT)/parent_common.mk

FMC_BUS ?= $(shell pwd)/fmc-bus
export FMC_BUS
# FMC_BUS_ABS has to be absolut path, due to beeing passed to the Kbuild
FMC_BUS_ABS ?= $(abspath $(FMC_BUS) )
export FMC_BUS_ABS

RUNME := $(shell test -d $(FMC_BUS) || git submodule update --init)

DIRS = $(FMC_BUS) kernel tools

.PHONY: all clean modules install modules_install $(DIRS)

all clean modules install modules_install: $(DIRS)

clean: TARGET = clean
modules: TARGET = modules
install: TARGET = install
modules_install: TARGET = modules_install

$(DIRS):
	$(MAKE) -C $@ $(TARGET)

kernel: $(FMC_BUS)

include scripts/gateware.mk
