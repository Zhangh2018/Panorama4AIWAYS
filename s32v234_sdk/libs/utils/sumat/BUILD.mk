##############################################################################
#
# NXP Confidential Proprietary
#
# Copyright 2017 NXP 
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

ARM_LIB =   libsumat.a
X86_LIB =   libsumat.a

##############################################################################
# libgdi.a
##############################################################################

ARM_LIB_SRCS += sumat_statics.cpp                                             \
                sumat_types.cpp                                               \
                sumat_allocator.cpp                                           \
		sumat_mutex.cpp                                               \
		sumat_data.cpp                                                \
		sumat_iterator.cpp                                            \
		sumat_mat.cpp                                                 \
		sumat.cpp

ARM_INCS +=     -I$(SDK_ROOT)/include                                         \
                -I../include
                
       
X86_LIB_SRCS += $(ARM_LIB_SRCS)
X86_INCS += $(ARM_INCS)

##############################################################################
