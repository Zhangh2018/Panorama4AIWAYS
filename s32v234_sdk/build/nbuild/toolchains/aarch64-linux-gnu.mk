##############################################################################
#
#  Freescale Semiconductor, Inc.
#  (c) Copyright 2011, Freescale Semiconductor, Inc.
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
#    2013-08-05
#
#  LANGUAGE
#    make
#
##############################################################################
OS_TYPE := Linux

C0_APP := $(ARM_APP).elf
C0_APP_SRCS := $(ARM_APP_SRCS)
C0_APP_LIBS := $(ARM_APP_LIBS)

C0_LIB := $(ARM_LIB)
C0_LIB_SRCS := $(ARM_LIB_SRCS)
C0_LIB_LIBS := $(ARM_LIB_LIBS)
C0_LIB_ARCS := $(ARM_LIB_ARCS)

C0_DEFS := $(ARM_DEFS)
C0_INCS := $(ARM_INCS)
C0_CCOPTS = $(ARM_CCOPTS)
C0_CXOPTS = $(ARM_CXOPTS)
C0_LDOPTS = -L$(OPENCV_ROOT)/lib/ -L$(FFMPEG_ROOT)/lib/ $(ARM_LDOPTS)

##############################################################################
PLATFORM = s32v234_ce

C0_DEFS += \
  -D__GNU__ \
  -D__LINUX__ \
  -DARM \
  -D__arm64 \
  -D __ARMV8 \
  -D CV220X_$(CPU) \
  -U $(OSDIR) -D OSDIR=$(OSDIR) \
  -D GNUARM#

C0_INCS += -I. \
  -I$(SDK_ROOT)/libs/utils/sumat/include \
  -I$(SDK_ROOT)/libs/utils/oal/user/include \
  -I$(SDK_ROOT)/libs/utils/oal/kernel/include \
  -I$(OPENCV_ROOT)/include

ARM_NO_PEDANTIC_OPT ?= 0
ifeq ($(ARM_NO_PEDANTIC_OPT),0)
C0_PEDANTIC_OPT := -pedantic
else
C0_PEDANTIC_OPT :=
endif

# compiler specific options for platform, not in headdep
C0_CCOPTS += \
  -Wall \
  -Wextra \
  -mcpu=cortex-a53 \
  -fdata-sections -ffunction-sections \
  -std=c99 \
  -Wno-narrowing \
  $(C0_PEDANTIC_OPT)

C0_CXOPTS += \
  -Wall \
  -Wextra \
  -mcpu=cortex-a53 \
  -fdata-sections -ffunction-sections \
  -std=gnu++11 \
  -Wno-narrowing \
  $(C0_PEDANTIC_OPT)

C0_ASOPTS += -mcpu=cortex-a53

C0_LDOPTS += \
  -Wl,-Map=$@.map,--cref,--gc-sections

C0_LDOPTS_PRE_LIBS := -Wl,--start-group
C0_LDOPTS_POST_LIBS := -Wl,--end-group

CROSS_COMPILE_SYSROOT ?=
CROSS_COMPILE ?= aarch64-linux-gnu-

C0_CC := $(CROSS_COMPILE)gcc -c $(CROSS_COMPILE_SYSROOT)
C0_CX := $(CROSS_COMPILE)g++ -c $(CROSS_COMPILE_SYSROOT)
C0_AS := $(CROSS_COMPILE)g++ -c $(CROSS_COMPILE_SYSROOT)
C0_LD := $(CROSS_COMPILE)g++ $(CROSS_COMPILE_SYSROOT)
C0_AR := $(CROSS_COMPILE)ar
C0_OD := $(CROSS_COMPILE)objdump
C0_OC := $(CROSS_COMPILE)objcopy
C0_NM := $(CROSS_COMPILE)nm


C0_AROPTS += rcs
C0_CXOPTS +=
C0_ASOPTS +=
C0_LDOPTS += -lpthread -lrt -lstdc++
C0_FCLEAN := *_log.txt *.map

