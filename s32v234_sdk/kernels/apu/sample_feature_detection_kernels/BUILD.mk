##############################################################################
#
# Freescale Confidential Proprietary
#
# Copyright (c) 2016 Freescale Semiconductor;
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

X86_INCS = -I$(SDK_ROOT)/include  \
  -I../src
COMMON_LIB_SRCS =                                              \
  fast9_acf.cpp                                                \
  fast9_apu.cpp                                                \
  harris_acf.cpp                                               \
  harris_apu.cpp                                               \
  sad_acf.cpp                                                  \
  sad_apu.cpp

ifeq ($(PDIR), x86)
# APEX2_EMULATE
X86_CXOPTS += -g
X86_LIB = libfeature_detection_kernels.a
X86_DEFS = -DAPEX2_EMULATE -DACF_KERNEL_IMPLEMENTATION
X86_INCS +=                                                    \
  -I$(SDK_ROOT)/tools/emu/acf/src                              \
  -I$(SDK_ROOT)/tools/emu/apu/src
X86_LIB_SRCS = $(COMMON_LIB_SRCS)
else
APU_LIB = feature_detection_kernels.a
##############################################################################
# APU Kernel library built (code for APU - kernel implementation)
##############################################################################

APU_INCS =                                                    \
    -I./                                                      \
    -I$(SDK_ROOT)/include

APU_LIB_SRCS = $(COMMON_LIB_SRCS)

APU_DEFS = -DACF_KERNEL_IMPLEMENTATION


ifeq (,$(findstring -nxp,$(ODIR)))
  APU_CCOPTS += +W noodle,-Wnone
  APU_CXOPTS += +W noodle,-Wnone
endif

##############################################################################
# Kernel db generator - builds an exe application and generates .h header with
#                       kernel description
# Do not change following code
##############################################################################
X86_LIB = $(addsuffix acf.a,$(basename $(APU_LIB)))
X86_LIB_SRCS = $(APU_LIB_SRCS)

X86_KERNEL_DB = "on"
##############################################################################
endif

