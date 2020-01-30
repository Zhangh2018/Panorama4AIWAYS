##############################################################################
#
# Freescale Confidential Proprietary
#
# Copyright (c) 2016 Freescale Semiconductor;
# Copyright (c) 2017 NXP 
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
    $(SDK_ROOT)/kernels/apu/apexcv_base_color_conversion/$(ODIR)/apexcv_base_color_conversion.a

APU_GRAPH_INCS =                                                             \
    -I..                                                                     \
        -I$(SDK_ROOT)/libs/utils/umat/include                                   \
    -I$(SDK_ROOT)/libs/utils/oal/user/include                               \
    -I$(SDK_ROOT)/libs/utils/oal/kernel/include                             \
    -I$(SDK_ROOT)/include                                                    \
    -I$(SDK_ROOT)/kernels/apu/apexcv_base_color_conversion/src               \
    -I$(SDK_ROOT)/libs/apexcv_base/apexcv_core/include                       
# apexcv_core is to access graph_template.h

APU_GRAPHS =                                                                 \
    ../rgb565_to_rgb888x_apu_process_desc.hpp                                \
    ../rgb888x_to_rgb565_apu_process_desc.hpp                                \
    ../rgb888x_to_y_apu_process_desc.hpp                                     \
    ../rgb888x_to_yuv_apu_process_desc.hpp                                   \
    ../rgb888_to_grey_apu_process_desc.hpp                                   \
    ../bgr888_to_grey_apu_process_desc.hpp                                   \
    ../grey_to_rgb888_apu_process_desc.hpp                                   \
    ../extract_channel_2ch_apu_process_desc.hpp                              \
    ../extract_channel_3ch_apu_process_desc.hpp                              \
    ../extract_channel_4ch_apu_process_desc.hpp                              \
    ../insert_channel_2ch_apu_process_desc.hpp                               \
    ../insert_channel_3ch_apu_process_desc.hpp                               \
    ../insert_channel_4ch_apu_process_desc.hpp                               \
    ../split_channel_2ch_apu_process_desc.hpp                                \
    ../split_channel_3ch_apu_process_desc.hpp                                \
    ../split_channel_4ch_apu_process_desc.hpp                                \
    ../merge_channel_2ch_apu_process_desc.hpp                                \
    ../merge_channel_3ch_apu_process_desc.hpp                                \
    ../merge_channel_4ch_apu_process_desc.hpp                                \
    ../ht_rgb888x_to_y_apu_process_desc.hpp                                  \
    ../down_convert_16s_apu_process_desc.hpp                                 \
    ../up_convert_08u_apu_process_desc.hpp
