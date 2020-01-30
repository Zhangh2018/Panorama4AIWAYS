##############################################################################
#
# NXP Confidential Proprietary
#
# Copyright (c) 2015 NXP Semiconductor;
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
                                   

##############################################################################
# acf_app
##############################################################################
ARM_LIB = apexcv_pro_lkpyramid.a

ARM_LIB_LIBS = $(SDK_ROOT)/libs/apexcv_pro/image_pyramid_multi/$(ODIR)/apexcv_pro_pyramid_multi.a

ARM_LIB_SRCS  =                                                               \
   apexcv_pro_lkpyramid.cpp                                  \


ARM_INCS +=                                                                  \
   -I$(SDK_ROOT)/libs/apex/icp/include                                       \
   -I$(SDK_ROOT)/libs/apex/acf/include                                       \
   -I$(SDK_ROOT)/libs/utils/common/include                                   \
   -I$(SDK_ROOT)/libs/apex/drivers/user/include                              \
   -I$(SDK_ROOT)/libs/apexcv_pro/image_pyramid_multi/$(ODIR)                 \
   -I$(SDK_ROOT)/libs/apexcv_pro/image_pyramid_multi/graphs/build-apu-$(APU_COMP)-sa-d \
   -I$(SDK_ROOT)/include                                                     \
   -I../src                                                                  \
   -I../graphs                                                               \
   -I../graphs/build-apu-$(APU_COMP)-sa-d



