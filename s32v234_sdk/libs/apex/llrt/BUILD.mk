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

SDK_ROOT := ../../../..

ARM_LIB = libllrt.a

##############################################################################
# libllrt.a
##############################################################################

ARM_LIB_SRCS += data_desc.cpp

ARM_INCS +=                          \
    -I$(SDK_ROOT)/libs/apex/llrt/inc \
    -I$(SDK_ROOT)/include            \

##############################################################################


ifneq (,$(findstring apu3,$(ODIR)))

VPATH += \
   ../../icp/src \
   ../../acf_target/src \
   ../../common/src \
   ../../utils/src

#
# LIB
#
APU_LIB:=libapu3_llrt__hw.a


APU_LIB_SRCS:= \
   buf_desc.cpp                  \
   circ_tile_buf.cpp             \
   circ_tile_smart_buf.cpp       \
   data_desc.cpp                 \
   data_transfer.cpp             \
   data_transfer_method_a.cpp    \
   data_transfer_method_a__hw.cpp\
   kernel_io_desc.cpp            \
   tile_vectorization_desc.cpp   \
   tiling_desc.cpp               \
   transfer_desc.cpp             \
   utilities_desc.cpp            \
   vector_map_desc.cpp           \
   icp_datatype.cpp              \
   apu_test_util.post.cpp        \
   apu_mem_mgmt__iss.cpp         \
   c_malloc.cpp


APU_LIB_DEFS:= \
   __STDC_LIMIT_MACROS \
    APEX_STATIC_LIB

APU_LIB_INCS:= \
   ../inc \
   ../../../../include \
   ../../common/include \
   ../../acf_target/include \
   ../../utils/inc \
   ../../../apexd/drivers/user/include \
   ../../../apexd/drv/platform/base/apex-d/inc \
   ../../../apexd/drv/util/inc \
   ../../../apexd/map/hw/cpp

endif
