##############################################################################
#
# NXP Confidential Proprietary
#
# Copyright (c) 2017 NXP;
# All Rights Reserved
#
##############################################################################
#
# THIS SOFTWARE IS PROVIDED BY NXP "AS IS" AND ANY EXPRESSED OR
# IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
# OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
# IN NO EVENT SHALL NXP OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
# INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
# IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
# THE POSSIBILITY OF SUCH DAMAGE.
#
##############################################################################

SDK_ROOT := ../../../../..

ARM_APP = oal_driver

DRIVER_DIR = $(SDK_ROOT)/libs/utils/oal/kernel

##############################################################################
# oal_driver
##############################################################################

ifeq ($(OSDIR), qnx)

VPATH +=  $(DRIVER_DIR)/src/qnx

ARM_LIB_SRCS +=	os_oal_allocation_kernel.c                           \
		os_kernel_module.c

ARM_INCS += -I$(DRIVER_DIR)/include/internal/qnx/

endif

ARM_LIB_SRCS += oal_allocator.c                                       \
		oal_cma_list.c                                        \
		oal_allocation_kernel.c

ARM_APP_LIBS +=                                                       \
	$(SDK_ROOT)/libs/utils/log/$(ODIR)/liblog.a

VPATH += $(DRIVER_DIR)/src

ARM_INCS +=	-I$(DRIVER_DIR)/include/                              \
		-I$(SDK_ROOT)/include/

##############################################################################
