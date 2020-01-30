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
   $(KERNELS)/sample_feature_detection_kernels/$(ODIR)/feature_detection_kernels.a \
   $(KERNELS)/sample_filtering_kernels/$(ODIR)/filtering_kernels.a                 \

  
APU_GRAPH_INCS =                                                            \
   -I..                                                                     \
   -I$(SDK_ROOT)/include                                                    \
   -I$(SDK_ROOT)/kernels/apu/sample_feature_detection_kernels/src      \
   -I$(SDK_ROOT)/kernels/apu/sample_filtering_kernels/src              \

APU_GRAPHS =                                                            \
  ../apu_fast_nms_apu_process_desc.hpp      \
  ../apu_fast_apu_process_desc.hpp					\
   
