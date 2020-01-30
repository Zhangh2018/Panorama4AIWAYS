##############################################################################
#
# NXP Confidential Proprietary
#
# Copyright (c) 2016 Freescale Semiconductor;
# Copyright 2017 NXP;
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
KERNELS  := $(SDK_ROOT)/kernels/apu

##############################################################################
# acf kernels and graphs
##############################################################################
APU_GRAPH_LIBS =                                                             \
   $(SDK_ROOT)/kernels/apu/apexcv_pro_gftt_corners/$(ODIR)/apexcv_pro_gftt_corners.a \
   $(SDK_ROOT)/kernels/apu/apexcv_pro_harris_corners/$(ODIR)/apexcv_pro_harris_corners.a

   
APU_GRAPH_INCS =                                                            \
   -I..                                                                     \
   -I$(SDK_ROOT)/include                                                    \
   -I$(SDK_ROOT)/libs/utils/common/include                                  \
   -I$(SDK_ROOT)/kernels/apu/apexcv_pro_gftt_corners/src                    \
   -I$(SDK_ROOT)/kernels/apu/apexcv_pro_harris_corners/src

APU_GRAPHS =                                                                \
   ../gftt_b7n5_compute_apu_process_desc.hpp                                \
   ../gftt_b5n5_compute_apu_process_desc.hpp                                \
   ../gftt_b3n5_compute_apu_process_desc.hpp                                \
   ../gftt_b7n5_compute_extract_apu_process_desc.hpp                        \
   ../gftt_b5n5_compute_extract_apu_process_desc.hpp                        \
   ../gftt_b3n5_compute_extract_apu_process_desc.hpp                        \
   ../harris_b7n5_compute_apu_process_desc.hpp                              \
   ../harris_b7n5_compute_extract_apu_process_desc.hpp                      \
   ../harris_b5n5_compute_apu_process_desc.hpp                              \
   ../harris_b5n5_compute_extract_apu_process_desc.hpp                      \
   ../harris_b3n5_compute_apu_process_desc.hpp                              \
   ../harris_b3n5_compute_extract_apu_process_desc.hpp                      \

#######################################################
# this is here to enable __ACF_REDUCE_CODE_SIZE__
#######################################################
ifneq (,$(findstring -tct,$(ODIR)))
override C1_COMMON_DEFS :=      \
  -DCGV_FORCE_INLINE            \
  -D__TCTBUILDER__              \
  -D__ACF_REDUCE_CODE_SIZE__    \
  -include apu2_kernel_port.h   \
  -include acf_autokernel_inc.h
endif

ifneq (,$(findstring -nxp,$(ODIR)))
override C1_COMMON_DEFS :=       \
  -DCGV_FORCE_INLINE            \
  -D__ACF_REDUCE_CODE_SIZE__    \
  -include apu2_kernel_port.h   \
  -include acf_autokernel_inc.h \
  -include apu_microkernel.h  
endif
