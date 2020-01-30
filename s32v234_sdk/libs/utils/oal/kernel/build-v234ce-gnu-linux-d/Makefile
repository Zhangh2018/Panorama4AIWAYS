# Copyright (c) 2014-2016 Freescale Semiconductor
# Copyright 2017 NXP
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:
#
# o Redistributions of source code must retain the above copyright notice, this list
#   of conditions and the following disclaimer.
#
# o Redistributions in binary form must reproduce the above copyright notice, this
#   list of conditions and the following disclaimer in the documentation and/or
#   other materials provided with the distribution.
#
# o Neither the name of NXP nor the names of its
#   contributors may be used to endorse or promote products derived from this
#   software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
# ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

check_defined = \
    $(foreach 1,$1,$(__check_defined))
__check_defined = \
    $(if $(value $1),, \
      $(error Please define $1 pointing to kernel directory $(if $(value 2), ($(strip $2) ))))

KERNELDIR ?= $(LINUX_S32V234_DIR)
OSDIR := linux

INCLUDE_DIRS := \
		$(CURDIR)/../include/internal/linux/ \
		$(CURDIR)/../include/ \
		$(CURDIR)/../../../../../include/ \
		$(KERNELDIR)/arch/arm/include \
		$(KERNELDIR)/arm/include/generated \

CFLGS :=	-g				\
		-O0				\
		-D DEBUG			\
		-D __KERNEL_MODULE		\
		-D __ARMV8			\
		-U $(OSDIR) -D OSDIR=$(OSDIR)	\
		$(APEXDRIVER_ADDITIONAL_FLAGS)

CFLGS += $(foreach includedir, $(INCLUDE_DIRS),-I$(includedir))

# module string prefix
MODULE := oal_cma

# defines driver module object & output file
obj-m  := $(MODULE).o

# what object (source) files is the final output composed of
$(MODULE)-objs := 	../src/oal_allocator.o \
			../src/oal_cma_list.o \
			../src/linux/os_oal_allocation_kernel.o \
			../src/oal_allocation_kernel.o  \
			../src/cache.o \
			../src/linux/os_kernel_module.o \
			../../../../utils/log/src/vdb_log.o

KBUILD_EXTRA_SYMBOLS := ${CURDIR}/../../../../utils/oal/kernel/build-v234ce-gnu-linux-d/Module.symvers

### Targets ###

.PHONY: clean all allsub cleansub

all: check-env
	$(MAKE) KCPPFLAGS="$(CFLGS)" ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- -C $(KERNELDIR) M=$(CURDIR) modules

allsub: all

check-env:
	$(call check_defined, LINUX_S32V234_DIR)

clean cleansub:
	$(MAKE) KCPPFLAGS="$(CFLGS)" ARCH=arm64 CROSS_COMPILE=$(CROSS_COMPILE) -C $(KERNELDIR) M=$(CURDIR) clean
	rm -rf $(apex2-objs) ../src/linux/.*.cmd ../src/linux/*.o ../../common/src/*.o ../../common/src/.*.cmd
