##############################################################################
#
# NXP Confidential Proprietary
#
# Copyright (c) 2016-2017 NXP
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
   $(SDK_ROOT)/kernels/apu/apexcv_pro_aggcf/$(ODIR)/apexcv_pro_aggcf.a       \
   $(SDK_ROOT)/kernels/apu/apexcv_pro_resize/$(ODIR)/apexcv_pro_resize.a


APU_GRAPH_INCS =                                                            \
   -I..                                                                     \
   -I$(SDK_ROOT)/include                                                    \
   -I$(SDK_ROOT)/libs/utils/common/include                                  \
   -I$(SDK_ROOT)/kernels/apu/apexcv_pro_aggcf/src                           \
   -I$(SDK_ROOT)/kernels/apu/apexcv_pro_resize/src

APU_GRAPHS =                                                                \
   ../aggcf_apu_process_desc.hpp                                            \
   ../aggcf_hist_nt1_apu_process_desc.hpp                                   \
   ../aggcf_hist_nt2_apu_process_desc.hpp                                   \
   ../aggcf_hist_nt3_apu_process_desc.hpp                                   \
   ../aggcf_hist_nt4_apu_process_desc.hpp                                   \
   ../aggcf_hist_ti2_apu_process_desc.hpp                                   \
   ../aggcf_hist_ti3_apu_process_desc.hpp                                   \
   ../aggcf_hist_ti4_apu_process_desc.hpp                                   \
   ../aggcf_flt_apu_process_desc.hpp                                        \
   ../aggcf_luv_apu_process_desc.hpp                                        \
   ../aggcf_triflt_apu_process_desc.hpp                                     \
   ../aggcf_flt_norm_apu_process_desc.hpp                                   \
   ../resize16_v_h_flt_apu_process_desc.hpp                                 \
   ../resize16_h_v_flt_apu_process_desc.hpp                                 \
   ../resize16_h_flt_apu_process_desc.hpp                                   \
   ../resize16_v_flt_apu_process_desc.hpp                                   \
   ../aggcf_scale_down2_apu_process_desc.hpp                                \
   ../aggcf_scale_down4_apu_process_desc.hpp                                \
   ../aggcf_scale_down8_apu_process_desc.hpp
   
