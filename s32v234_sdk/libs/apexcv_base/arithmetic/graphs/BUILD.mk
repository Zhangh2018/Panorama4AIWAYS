##############################################################################
#
# Freescale Confidential Proprietary
#
# Copyright (c) 2016 Freescale Semiconductor;
# Copyright (c) 2017-2018 NXP 
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
KERNELS  := $(SDK_ROOT)/kernels/apu


##############################################################################
# acf kernels and graphs
##############################################################################

APU_GRAPH_LIBS =                                                             \
    $(SDK_ROOT)/kernels/apu/apexcv_base_arithmetic/$(ODIR)/apexcv_base_arithmetic.a

APU_GRAPH_INCS =                                                             \
    -I..                                                                     \
    -I../../include                                                          \
    -I$(SDK_ROOT)/libs/utils/umat/include                                    \
    -I$(SDK_ROOT)/libs/utils/oal/user/include                                \
    -I$(SDK_ROOT)/libs/utils/oal/kernel/include                              \
    -I$(SDK_ROOT)/include                                                    \
    -I$(SDK_ROOT)/kernels/apu/apexcv_base_arithmetic/src                     \
    -I$(SDK_ROOT)/kernels/apu/vfxp_math/include                              \
    -I$(SDK_ROOT)/libs/apexcv_base/apexcv_core/include
# apexcv_core is to access graph_template.h

APU_GRAPHS =                                                                 \
    ../abs_08u_apu_process_desc.hpp                                          \
    ../absdiff_08u_apu_process_desc.hpp                                      \
    ../absdiff_16s_apu_process_desc.hpp                                      \
    ../accumulate_16s_apu_process_desc.hpp                                   \
    ../accumulate_squared_16s_apu_process_desc.hpp                           \
    ../accumulate_weighted_08u_apu_process_desc.hpp                          \
    ../add_08u_apu_process_desc.hpp                                          \
    ../add_08u_08u_16s_apu_process_desc.hpp                                  \
    ../add_08u_16s_16s_apu_process_desc.hpp                                  \
    ../add_16s_apu_process_desc.hpp                                          \
    ../and_08u_apu_process_desc.hpp                                          \
    ../and_16u_apu_process_desc.hpp                                          \
    ../and_32u_apu_process_desc.hpp                                          \
    ../clz_08s_apu_process_desc.hpp                                          \
    ../clz_08u_apu_process_desc.hpp                                          \
    ../clz_16s_apu_process_desc.hpp                                          \
    ../clz_16u_apu_process_desc.hpp                                          \
    ../magnitude_16s_apu_process_desc.hpp                                    \
    ../max_08u_apu_process_desc.hpp                                          \
    ../max_16s_apu_process_desc.hpp                                          \
    ../min_08u_apu_process_desc.hpp                                          \
    ../min_16s_apu_process_desc.hpp                                          \
    ../mul_08u_apu_process_desc.hpp                                          \
    ../mul_08u_08u_16s_apu_process_desc.hpp                                  \
    ../mul_08u_16s_16s_apu_process_desc.hpp                                  \
    ../mul_16s_apu_process_desc.hpp                                          \
    ../not_08u_apu_process_desc.hpp                                          \
    ../or_08u_apu_process_desc.hpp                                           \
    ../or_16u_apu_process_desc.hpp                                           \
    ../or_32u_apu_process_desc.hpp                                           \
    ../phase_08u_apu_process_desc.hpp                                        \
    ../sub_08u_apu_process_desc.hpp                                          \
    ../sub_08u_08u_16s_apu_process_desc.hpp                                  \
    ../sub_08u_16s_16s_apu_process_desc.hpp                                  \
    ../sub_16s_apu_process_desc.hpp                                          \
    ../sub_16s_08u_16s_apu_process_desc.hpp                                  \
    ../tablelookup_08u_apu_process_desc.hpp                                  \
    ../threshold_08u_apu_process_desc.hpp                                    \
    ../threshold_16u_apu_process_desc.hpp                                    \
    ../threshold_32u_apu_process_desc.hpp                                    \
    ../threshold_range_08u_apu_process_desc.hpp                              \
    ../threshold_range_16u_apu_process_desc.hpp                              \
    ../threshold_range_32u_apu_process_desc.hpp                              \
    ../xor_08u_apu_process_desc.hpp                                          \
    ../xor_16u_apu_process_desc.hpp                                          \
    ../xor_32u_apu_process_desc.hpp

