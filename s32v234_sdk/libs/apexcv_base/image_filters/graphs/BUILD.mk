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
KERNELS  := $(SDK_ROOT)/kernels/apu

##############################################################################
# acf kernels and graphs
##############################################################################
APU_GRAPH_LIBS =                                                             \
    $(SDK_ROOT)/kernels/apu/apexcv_base_image_filters/$(ODIR)/apexcv_base_image_filters.a

APU_GRAPH_INCS =                                                             \
    -I..                                                                     \
    -I$(SDK_ROOT)/libs/utils/umat/include                                    \
    -I$(SDK_ROOT)/libs/utils/oal/user/include                                \
    -I$(SDK_ROOT)/libs/utils/oal/kernel/include                              \
    -I$(SDK_ROOT)/include                                                    \
    -I$(SDK_ROOT)/kernels/apu/apexcv_base_image_filters/src                  \
    -I$(SDK_ROOT)/libs/apexcv_base/apexcv_core/include
# apexcv_core is to access graph_template.h

APU_GRAPHS =                                                                 \
    ../bilateral_5x5_apu_process_desc.hpp                                    \
    ../box_3x3_08u_apu_process_desc.hpp                                      \
    ../box_3x3_16s_apu_process_desc.hpp                                      \
    ../box_5x5_08u_apu_process_desc.hpp                                      \
    ../box_5x5_16s_apu_process_desc.hpp                                      \
    ../box_7x7_08u_apu_process_desc.hpp                                      \
    ../box_9x9_08u_apu_process_desc.hpp                                      \
    ../census_3x3_08u_apu_process_desc.hpp                                   \
    ../convolve_3x3_08u_apu_process_desc.hpp                                 \
    ../convolve_3x3_16s_apu_process_desc.hpp                                 \
    ../convolve_5x5_08u_apu_process_desc.hpp                                 \
    ../convolve_5x5_16s_apu_process_desc.hpp                                 \
    ../convolve_scale_3x3_08u_apu_process_desc.hpp                           \
    ../convolve_scale_3x3_16s_apu_process_desc.hpp                           \
    ../convolve_scale_5x5_08u_apu_process_desc.hpp                           \
    ../convolve_scale_5x5_16s_apu_process_desc.hpp                           \
    ../dilate_3x3_08u_apu_process_desc.hpp                                   \
    ../dilate_3x3_16s_apu_process_desc.hpp                                   \
    ../erode_3x3_08u_apu_process_desc.hpp                                    \
    ../gaussian_3x3_08u_apu_process_desc.hpp                                 \
    ../gaussian_5x5_08u_apu_process_desc.hpp                                 \
    ../gaussian_7x7_08u_apu_process_desc.hpp                                 \
    ../gaussian_9x9_08u_apu_process_desc.hpp                                 \
    ../ht_box_3x3_08u_apu_process_desc.hpp                                   \
    ../ht_centraldx_3x3_16s_apu_process_desc.hpp                             \
    ../ht_centraldy_3x3_16s_apu_process_desc.hpp                             \
    ../ht_generic_3x3_08u_apu_process_desc.hpp                               \
    ../ht_generic_5x5_08u_apu_process_desc.hpp                               \
    ../ht_saturate_08s_apu_process_desc.hpp                                  \
    ../ht_separable_3x3_16s_apu_process_desc.hpp                             \
    ../ht_separable_5x5_16s_apu_process_desc.hpp                             \
    ../ht_sobel_3x3_16s_apu_process_desc.hpp                                 \
    ../ht_sobel_x_3x3_08s_apu_process_desc.hpp                               \
    ../ht_sobel_y_3x3_08s_apu_process_desc.hpp                               \
    ../median_3x3_08u_apu_process_desc.hpp                                   \
    ../median_5x5_08u_apu_process_desc.hpp                                   \
    ../median_7x7_08u_apu_process_desc.hpp                                   \
    ../prewitt_x_3x3_apu_process_desc.hpp                                    \
    ../prewitt_y_3x3_apu_process_desc.hpp                                    \
    ../scharr_3x3_08u_apu_process_desc.hpp                                   \
    ../scharr_x_3x3_08u_apu_process_desc.hpp                                 \
    ../scharr_xy_3x3_08u_apu_process_desc.hpp                                \
    ../scharr_y_3x3_08u_apu_process_desc.hpp                                 \
    ../scharr_5x5_08u_apu_process_desc.hpp                                   \
    ../scharr_x_5x5_08u_apu_process_desc.hpp                                 \
    ../scharr_xy_5x5_08u_apu_process_desc.hpp                                \
    ../scharr_y_5x5_08u_apu_process_desc.hpp                                 \
    ../sobel_3x3_08u_apu_process_desc.hpp                                    \
    ../sobel_5x5_08u_apu_process_desc.hpp                                    \
    ../sobel_x_3x3_08s_apu_process_desc.hpp                                  \
    ../sobel_x_3x3_08u_apu_process_desc.hpp                                  \
    ../sobel_x_5x5_08s_apu_process_desc.hpp                                  \
    ../sobel_x_5x5_08u_apu_process_desc.hpp                                  \
    ../sobel_xy_3x3_08u_apu_process_desc.hpp                                 \
    ../sobel_y_3x3_08s_apu_process_desc.hpp                                  \
    ../sobel_y_3x3_08u_apu_process_desc.hpp                                  \
    ../sobel_y_5x5_08s_apu_process_desc.hpp                                  \
    ../sobel_y_5x5_08u_apu_process_desc.hpp
