##############################################################################
#
#  Freescale Semiconductor, Inc.
#  (c) Copyright 2011, Freescale Semiconductor, Inc.
#  Copyright 2017-2018 NXP;
#  ALL RIGHTS RESERVED.
#
##############################################################################
#
#  CONTENT
#
#
#  AUTHOR
#    Stefan Schwarz
#
#  DATE
#    2018-06-06
#
#  LANGUAGE
#    make
#
##############################################################################
OS_TYPE := Linux

C2_APP := $(X86_APP)
C2_APP_SRCS := $(X86_APP_SRCS)
C2_APP_LIBS := $(X86_APP_LIBS)

C2_LIB := $(X86_LIB)
C2_LIB_SRCS := $(X86_LIB_SRCS)
C2_LIB_LIBS := $(X86_LIB_LIBS)
C2_LIB_ARCS := $(X86_LIB_ARCS)

C2_DEFS := $(X86_DEFS)
C2_INCS := $(X86_INCS)
C2_CCOPTS = $(X86_CCOPTS)
C2_CXOPTS = $(X86_CXOPTS) -std=c++11

MCU = cortex-a9

##############################################################################
PLATFORM = zc702

C2_CC := $(COMPILER_PREFIX)gcc -c
C2_CX := $(COMPILER_PREFIX)g++ -c
C2_AS := $(COMPILER_PREFIX)gcc -c
C2_LD := $(COMPILER_PREFIX)gcc
C2_AR := $(COMPILER_PREFIX)ar rcs
C2_OD := $(COMPILER_PREFIX)objdump
C2_OC := $(COMPILER_PREFIX)objcopy
C2_NM := $(COMPILER_PREFIX)nm

C2_PROMPT := offline/X86

# compiler specific options for core
C2_INCS += \
  -I$(SDK_ROOT)/libs/utils/sumat/include \
  -I$(SDK_ROOT)/libs/utils/oal/user/include \
  -I$(SDK_ROOT)/libs/utils/oal/kernel/include \
  -I$(OPENCV_ROOT)/include

C2_DEFS += -D_GLIBCXX_USE_CXX11_ABI=0
C2_CCOPTS += -std=c99
C2_CXOPTS += -std=c++11
C2_ASOPTS +=
C2_LDOPTS += -L$(OPENCV_ROOT)/lib -lstdc++ -ldl -lrt $(X86_LDOPTS)
C2_FCLEAN :=

include $(SDK_ROOT)/build/nbuild/.C2.mk
