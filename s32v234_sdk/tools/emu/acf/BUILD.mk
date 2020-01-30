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

SDK_ROOT := $(call path_relative_to,$(CURDIR),$(CURR_SDK_ROOT))

X86_LIB = libemu_acf.a

VPATH += ../src \
  $(SDK_ROOT)/libs/apex/acf/src \
  $(SDK_ROOT)/libs/apex/icp/src

##############################################################################
# libacf.a
##############################################################################
X86_LIB_SRCS += acf_graph.cpp                        \
                acf_kernel.cpp                       \
                acf_lib.cpp                          \
                acf_process_desc_apu.cpp             \
                acf_datadesc.cpp                     \
                acf_common.cpp                       \
                icp_datatype.cpp                     \


X86_INCS += -I../src                             \
            -I../../apu/src                      \
            -I$(SDK_ROOT)/libs/apex/acf/include  \
            -I$(SDK_ROOT)/include                \
            -I$(OPENCV_ROOT)/include             \
            -I$(SDK_ROOT)/libs/utils/umat/include\

X86_DEFS += -DAPEX2_EMULATE


X86_CCOPTS += -w  
#X86_CCOPTS += -DDEBUG_OUTPUT

##############################################################################
# S32V234 TARGET VARIABLES
##############################################################################
ARM_LIB = $(X86_LIB)
ARM_LIB_SRCS += $(X86_LIB_SRCS)
ARM_INCS += $(X86_INCS)
ARM_CCOPTS += $(X86_CCOPTS)
ARM_DEFS += $(X86_DEFS)
ARM_CXOPTS += $(X86_CXOPTS)
