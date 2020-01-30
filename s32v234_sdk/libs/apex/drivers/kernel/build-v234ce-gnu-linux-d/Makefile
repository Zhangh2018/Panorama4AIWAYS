# Copyright (c) 2014-2016 Freescale Semiconductor
# Copyright (c) 2017-2018 NXP
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

INCLUDE_DIRS := \
	$(CURDIR)/../include/                                         \
	$(CURDIR)/../../user/include/                                 \
	$(CURDIR)/../../../common/include/                            \
	$(CURDIR)/../../../register_maps/include/                     \
	$(CURDIR)/../../../../../include/                             \
	$(CURDIR)/../../../../utils/oal/kernel/include/internal/linux \

CFLGS := \
	-DDEBUG                                      \
	-D__KERNEL_MODULE                            \
	$(APEXDRIVER_ADDITIONAL_FLAGS)

CFLGS += $(foreach includedir, $(INCLUDE_DIRS),-I$(includedir))

obj-m	:= apex.o
apex-objs := \
		../src/linux/kernel_module.o      \

KERNELDIR ?= $(LINUX_S32V234_DIR)
CROSS_COMPILE ?= aarch64-linux-gnu-

.PHONY: clean cleansub all allsub

all allsub: check-env
	$(MAKE) KCPPFLAGS="$(CFLGS)" ARCH=arm64 CROSS_COMPILE=$(CROSS_COMPILE) -C $(KERNELDIR) M=$(CURDIR)

check-env:
	$(call check_defined, LINUX_S32V234_DIR)

clean cleansub:
	$(MAKE) KCPPFLAGS="$(CFLGS)" ARCH=arm64 CROSS_COMPILE=$(CROSS_COMPILE) -C $(KERNELDIR) M=$(CURDIR) clean
	rm -rf $(apex2-objs) ../src/linux/.*.cmd ../src/linux/*.o ../../common/src/*.o ../../common/src/.*.cmd