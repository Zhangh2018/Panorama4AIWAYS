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

APU_LIB =   libacf_target.a

##############################################################################
# acf2_target.a for APU (incl. main function)
##############################################################################
VPATH +=                                                                     \
    $(SDK_ROOT)/libs/apex/acf_target/src/driver                              \
    $(SDK_ROOT)/libs/apex/acf_target/src/common                              \
    $(SDK_ROOT)/libs/apex/acf/src

APU_LIB_SRCS +=                                                              \
    acf_common.cpp                                                           \
    acf_hwutilities.cpp                                                      \
    acf_memmgmt.cpp                                                          \
    acf_utilities.cpp                                                        \
    mk_main.cpp                                                              \
    apu_test_util.cpp                                                        \
    acf_cbmanager.cpp

APU_INCS +=                                                                  \
    -I$(SDK_ROOT)/libs/apex/acf_target/include                               \
    -I$(SDK_ROOT)/libs/apex/common/include/driver                            \
    -I$(SDK_ROOT)/libs/apex/acf_offline/include                              \
    -I$(SDK_ROOT)/libs/apex/acf/include                                      \
    -I$(SDK_ROOT)/libs/apex/icp/include                                      \
    -I$(SDK_ROOT)/libs/apex/drivers/user/include                             \
    -I$(SDK_ROOT)/libs/apex/register_maps/include                            \
    -I$(SDK_ROOT)/include



##############################################################################
