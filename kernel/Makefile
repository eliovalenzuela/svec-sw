LINUX ?= /lib/modules/$(shell uname -r)/build

FMC_BUS ?= $(shell pwd)/../fmc-bus
export FMC_BUS
HTVIC ?= $(shell pwd)/../driver-htvic

GIT_VERSION = $(shell git describe --dirty --long --tags)

# For this CSM_VERSION, please see ohwr.org/csm documentation
ifdef CONFIG_CSM_VERSION
  ccflags-y += -D"CERN_SUPER_MODULE=MODULE_VERSION(\"$(CONFIG_CSM_VERSION)\")"
else
  ccflags-y += -DCERN_SUPER_MODULE=""
endif

ccflags-y += -I$(FMC_BUS)/kernel/include -I$(HTVIC)
ccflags-y += -DDEBUG
ccflags-y += -DGIT_VERSION=\"$(GIT_VERSION)\"

KBUILD_EXTRA_SYMBOLS += $(obj)/Module.symvers.vme
KBUILD_EXTRA_SYMBOLS += $(FMC_BUS)/kernel/Module.symvers
KBUILD_EXTRA_SYMBOLS += $(HTVIC)/Module.symvers

obj-m := svec.o

svec-objs := svec-drv.o
svec-objs += svec-sysfs.o
svec-objs += svec-fmc.o
svec-objs += svec-i2c.o
svec-objs += svec-irq.o

all: modules

.PHONY: all modules clean help install modules_install

modules help install modules_install:
	$(MAKE) -C $(LINUX) M=$(shell pwd) GIT_VERSION=$(GIT_VERSION) $@

# be able to run the "clean" rule even if $(LINUX) is not valid
clean:
	rm -rf *.o *~  .*.cmd *.ko *.mod.c .tmp_versions Module.symvers \
                         Module.markers modules.order
