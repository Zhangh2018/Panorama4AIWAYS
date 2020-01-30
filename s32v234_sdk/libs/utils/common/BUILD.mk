##############################################################################
#
# Freescale Confidential Proprietary
#
# Copyright (c) 2014 Freescale Semiconductor;
# All Rights Reserved
#
##############################################################################
#
# THIS SOFTWARE IS PROVIDED BY FREESCALE "AS IS" AND ANY EXPRESSED OR
# IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
# OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
# IN NO EVENT SHALL FREESCALE OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
# INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
# IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
# THE POSSIBILITY OF SUCH DAMAGE.
#
##############################################################################
SDK_ROOT := $(call path_relative_to,$(CURDIR),$(CURR_SDK_ROOT))
X86_LIB = libcommon.a

ifeq ($(PDIR), x86)
  ifeq ($(OSDIR), linux)
    X86_DEFS += -D__LINUX__
  endif
endif


VPATH += ../src
##############################################################################
# libcommon.a
##############################################################################
X86_LIB_SRCS +=                                                              \
    time_measure.cpp                                                         \
    base_process_init.cpp                                                    \
    helper_message.cpp                                                       \
    swt_arm_report.cpp


X86_INCS +=                                                                  \
    -I../include                                                             \
    -I$(SDK_ROOT)/libs/utils/oal/user/include                                \
    -I$(SDK_ROOT)/libs/utils/oal/kernel/include                              \
    -I$(SDK_ROOT)/libs/utils/common/include                                  \
    -I$(SDK_ROOT)/libs/io/semihost/include                                   \
    -I$(SDK_ROOT)/include                                                    \
    -I$(SDK_ROOT)                                                            \


X86_DEFS += -DAPEX2_EMULATE
X86_CCOPTS += -w 
#X86_CCOPTS += -DDEBUG_OUTPUT

##############################################################################
# S32V244 TARGET VARIABLES
##############################################################################

ARM_SHARED_LIB = $(X86_SHARED_LIB)
ARM_LIB = $(X86_LIB)
ARM_LIB_SRCS += $(X86_LIB_SRCS)
ARM_INCS += $(X86_INCS)
ARM_DEFS += $(X86_DEFS)
ARM_CXOPTS +=  $(X86_CXOPTS)
ARM_CCOPTS += $(X86_CCOPTS)
