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

ARM_LIB =   libacf.a

VPATH +=    ../test/ ../src/

##############################################################################
# libacf2host.a
##############################################################################

ARM_LIB_SRCS +=                                                              \
    acf_apu_program_params.cpp                                               \
    acf_process.cpp                                                          \
    acf_process_apu.cpp                                                      \
    acf_common.cpp                                                           \
    acf_datadesc.cpp                                                         \
    resource_manager_apex.cpp                                                \
    resource_manager_private.cpp                                             \
    resource_manager_public.cpp                                              \
    resource_manager_types.cpp                                               \
    kernel_manager_host.cpp

ifeq (,$(findstring gnu-sa,$(ODIR)))
    ARM_LIB_SRCS +=                                                          \
        resource_manager_test.cpp

endif
        
ARM_INCS +=                                                                  \
    -I$(SDK_ROOT)/libs/apex/acf/include                                      \
    -I$(SDK_ROOT)/libs/apex/drivers/user/include                             \
    -I$(SDK_ROOT)/libs/apex/register_maps/include                            \
    -I$(SDK_ROOT)/libs/apex/icp/include                                      \
    -I$(SDK_ROOT)/libs/utils/log                                             \
    -I$(SDK_ROOT)/include                                                    \
    -I$(SDK_ROOT)/libs/apex/common/include                                   \
    -I$(SDK_ROOT)/libs/apex/acf_offline/include                              \
    -I$(SDK_ROOT)/libs/utils/common/include                                  \
    -I$(SDK_ROOT)/libs/utils/sumat/include                                   \
    -I$(SDK_ROOT)/libs/utils/oal/user/include                                \
    -I$(SDK_ROOT)/libs/utils/oal/kernel/include                              \
    -I$(SDK_ROOT)/libs/utils/communications/src
        

