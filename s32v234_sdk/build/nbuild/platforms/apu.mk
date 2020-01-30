##############################################################################
#
#  Copyright 2017-2018 NXP;
#  ALL RIGHTS RESERVED.
#
##############################################################################
#
#  CONTENT
#    full featured Makefile for arm gcc using nbuild packages.
#    internal use only
#
#  AUTHOR
#    Mihail Marian Nistor
#
#  DATE
#    2018-08-24
#
#  LANGUAGE
#    make
#
##############################################################################
# nbuild packages

##############################################################################
# check required parameters
##############################################################################

ifeq ($(origin SDK_ROOT),undefined)
  $(error error: SDK_ROOT undefined)
endif

ifeq ($(origin APU_COMP),undefined)
  $(error error: APU_COMP undefined)
endif

VPATH += ../graphs

ifeq ($(X86_KERNEL_DB),"on")
  X86_APP = $(basename $(APU_LIB))
  X86_APP_SRCS = $(APU_LIB_SRCS)

  ifneq ($(findstring msys,$(NBUILD_PLATFORM)),)
    X86_APP_LIBS = $(SDK_ROOT)/libs/apex/acf_offline/build-x86-gnu-windows-d/libacf_offline.a
  else
    X86_APP_LIBS = $(SDK_ROOT)/libs/apex/acf_offline/build-x86-gnu-linux-d/libacf_offline.a
  endif

  X86_DEFS += -DACF_KERNEL_METADATA \
    --include apu2_kernel_db_lib.h
  X86_INCS += -I$(SDK_ROOT)/libs/apex/acf_offline/include \
    -I$(SDK_ROOT)/libs/apex/acf_target/include \
    -I$(SDK_ROOT)/libs/apex/common/include \
    -I$(SDK_ROOT)/libs/apex/common/include/driver

  X86_KERNEL_DB = "on"
endif

ifneq ($(strip $(X86_APP) $(X86_LIB)),)
  ifeq ($(NBUILD_PLATFORM), msys)
    OPENCV_ROOT = $(CURR_SDK_ROOT)/3rdparty/ocv/win32-mingw
    OPENCV_VER = 310
    include $(SDK_ROOT)/build/nbuild/toolchains/x86-windows-gnu.mk
  else
    OPENCV_ROOT = $(CURR_SDK_ROOT)/3rdparty/ocv/linux-x86
    OPENCV_VER =
    include $(SDK_ROOT)/build/nbuild/toolchains/x86-linux-gnu.mk
  endif
endif

ifneq ($(APU_GRAPHS),)
  include $(SDK_ROOT)/build/nbuild/apex_tools/apu-kernel-getgraphs.mk
endif

ifeq ($(X86_KERNEL_DB),"on")
  include $(SDK_ROOT)/build/nbuild/apex_tools/apu-kernel-database.mk
endif

ifneq ($(strip $(APU_APP) $(APU_LIB) $(APU_LOAD_BINS)),)
  include $(SDK_ROOT)/build/nbuild/toolchains/apu-acf-sa-$(APU_COMP).mk
endif
