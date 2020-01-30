##############################################################################
#
# NXP Confidential Proprietary
#
# Copyright (c) 2017-2018 NXP;
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


ifneq (,$(findstring x86,$(ODIR)))
  X86_LIB = liboal.a
  X86_CXOPTS += -g -std=c++11 
  

 ifneq (,$(findstring linux,$(ODIR)))
     VPATH    += $(SDK_ROOT)/libs/utils/oal/user/src/windows
	 X86_DEFS += -D__LINUX__
	 #X86_INCS += -I$(SDK_ROOT)/libs/utils/oal/user/include/internal/posix 
     X86_LIB_SRCS +=      os_oal_memory.cpp                                        \
        
#		os_oal_semaphore.cpp                                                        \
#       os_oal_task.cpp                                                             \
#		oal_lldcmd.cpp                                                              \
#       os_oal_memory_driver_if.cpp                                                 \

  else
#    windows host compiling

	#X86_DEFS += -D_WIN32 -D__CYGWIN__

    VPATH +=    $(SDK_ROOT)/libs/utils/oal/user/src/windows
    X86_LIB_SRCS +=                                                            \
        os_oal_memory.cpp                                                         \

  endif
 

else
  ARM_LIB =	liboal.a
  
  ifneq (,$(findstring v234ce,$(ODIR))) 
    ifneq (,$(findstring linux,$(ODIR)))  
	  OSDIR = linux
    else
	    ifneq (,$(findstring sa,$(ODIR))) 
		  OSDIR = sa

		  ARM_INCS += \
		  -I$(SDK_ROOT)/libs/utils/oal/kernel/include/internal/$(OSDIR)  \
    	  -I$(SDK_ROOT)/libs/utils/oal/user/include/internal/$(OSDIR)    \

		endif   
	endif
  endif
endif

X86_INCS +=                                                                 \
	-I$(SDK_ROOT)/include                                                   \
	-I$(SDK_ROOT)/libs/utils/oal/user/include                               \
	-I$(SDK_ROOT)/libs/utils/oal/kernel/include                             \
    -I$(SDK_ROOT)/libs/utils/oal/user/include/internal/

#############################################################################
# liboal.a
##############################################################################

VPATH += $(SDK_ROOT)/libs/utils/oal/user/src/

ARM_LIB_SRCS +=								\
	oal_memory.cpp							\
	os_oal_memory_driver_if.cpp				\
	os_oal_semaphore.cpp					\
	os_oal_task.cpp


VPATH += $(SDK_ROOT)/libs/utils/oal/kernel/src/$(OSDIR)		\
		 $(SDK_ROOT)/libs/utils/oal/kernel/src


ifeq ($(OSDIR), sa)				#Standalone

	ARM_LIB_SRCS +=							\
		os_oal_memory.cpp					\
		oal_allocator.c						\
		oal_cma_list.c						\
		oal_allocation_kernel.c				\
		os_oal_allocation_kernel.c			\
		os_oal_interrupt.cpp
endif

ifeq ($(OSDIR), linux)
	POSIX := true
	
	ARM_LIB_SRCS +=							\
		oal_lldcmd.cpp						\
	
endif

ifeq ($(OSDIR), qnx)
	POSIX := true
endif

ifeq ($(OSDIR), integrity)
	POSIX := true
endif

ifdef POSIX
	VPATH += $(SDK_ROOT)/libs/utils/oal/user/src/posix

	ARM_INCS +=							\
	 -I$(SDK_ROOT)/libs/utils/oal/user/include/internal/posix

	ARM_LIB_SRCS +=							\
		posix_oal_memory.cpp					\
		posix_oal_task.cpp					\
		posix_oal_interrupt.cpp
endif

ARM_DEFS +=	-D OSNAME='"$(call uc,$(OSDIR))"'


ARM_INCS +=							  	                           \
	-I$(SDK_ROOT)/include					                       \
	-I$(SDK_ROOT)/libs/utils/oal/user/include			           \
	-I$(SDK_ROOT)/libs/utils/oal/kernel/include			           \
	-I$(SDK_ROOT)/libs/utils/oal/kernel/include/internal           \
	-I$(SDK_ROOT)/libs/utils/common/include

VPATH += $(SDK_ROOT)/libs/utils/oal/user/src/$(OSDIR)			\
	 $(SDK_ROOT)/libs/utils/oal/user/src

##############################################################################
