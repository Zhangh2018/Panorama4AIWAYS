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

APU_LIB = libapu_common.a

##############################################################################
# acf2_target.a for APU (incl. main function)
##############################################################################
VPATH +=                                                                     \
    $(SDK_ROOT)/libs/apex/common/src                                         \
    $(SDK_ROOT)/libs/apex/common/src/driver                                  \
    $(SDK_ROOT)/libs/apex/acf/src                                            \
    $(SDK_ROOT)/libs/apex/icp/src                                            \
    $(SDK_ROOT)/libs/apex/drivers/user/src

APU_LIB_SRCS +=                                                              \
    apu_mk_lib.cpp                                                           \
    apu_mk_main.cpp                                                          \
    apumk_hal_cmdque.cpp                                                     \
    cmem_if_hal.cpp                                                          \
    hw_reg_user.cpp                                                          \
    icp_datatype.cpp                                                         \
    linked_list_utilities.cpp                                                \
    mcdma_hal.cpp                                                            \
    mce_hal.cpp                                                              \
    seq_hal.cpp                                                              \
    stio_hal.cpp                                                             \
    util.cpp
    
APU_INCS +=                                                                  \
    -I$(SDK_ROOT)/libs/apex/common/include                                   \
    -I$(SDK_ROOT)/libs/apex/common/include/driver                            \
    -I$(SDK_ROOT)/libs/apex/drivers/user/include                             \
    -I$(SDK_ROOT)/libs/apex/acf_target/include                               \
    -I$(SDK_ROOT)/libs/apex/acf/include                                      \
    -I$(SDK_ROOT)/libs/apex/icp/include                                      \
    -I$(SDK_ROOT)/libs/apex/drivers/user/include                             \
    -I$(SDK_ROOT)/include/

ifeq (,$(findstring -nxp,$(ODIR)))
  APU_CCOPTS += +W noodle,-Wnone
  APU_CXOPTS += +W noodle,-Wnone
endif

##############################################################################
