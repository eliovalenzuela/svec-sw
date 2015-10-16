# include parent_common.mk for buildsystem's defines
#use absolute path for REPO_PARENT
REPO_PARENT=$(shell /bin/pwd)/..
-include $(REPO_PARENT)/parent_common.mk

FMC_BUS ?= $(shell pwd)/fmc-bus
export FMC_BUS

RUNME := $(shell test -d $(FMC_BUS) || git submodule update --init)

DIRS = $(FMC_BUS) kernel tools

all clean modules install modules_install:
	for d in $(DIRS); do $(MAKE) -C $$d $@ || exit 1; done

include scripts/gateware.mk
