##############################################################################
#
# Copyright 2018 NXP 
# All Rights Reserved
#
##############################################################################
#
#  CONTENT
#    APU Emulator makefiles
#
#  AUTHOR
#    Dmitry Kudryavtsev
#    Mihail Marian Nistor
#
#  DATE
#    2018-11-27
#
##############################################################################
EMULATOR_DIR = ..
VISION_DIR = ../../../../s32v234_sdk
ifeq ($(shell echo $$OSTYPE),msys)
  OS_STYLE_VISION_DIR := $(shell cygpath -w $(realpath $(VISION_DIR)))
  OPENCV_DIR = $(VISION_DIR)/3rdparty/ocv/win32-mingw/include
else
  OS_STYLE_VISION_DIR := $(realpath $(VISION_DIR))
  OPENCV_DIR = $(VISION_DIR)/3rdparty/ocv/linux-x86/include
endif
ifeq ($(OS_STYLE_VISION_DIR),)
$(error The cygpath cannot convert the current sdk root from Unix-style to Windows-style)
endif
INSTALLDIR = ../libs
ARFLAGS = rv
CXX = g++
CXXOPTS = -std=c++0x -O0 -g -Wall -Wextra -fmessage-length=0 -MMD -MP \
  -D_GLIBCXX_USE_CXX11_ABI=0 -fdebug-prefix-map="$(OS_STYLE_VISION_DIR)"="s32v234_sdk"
RM = rm -rf
export

.PHONY: all clean install uninstall

all:
	@make -f Makefile_emu all
	@make -f Makefile_misc all
	@make -f Makefile_apexcv_base_kernels all
	@make -f Makefile_kernels all
	@make -f Makefile_apexcv_base all

install:
	@make -f Makefile_emu install
	@make -f Makefile_misc install
	@make -f Makefile_apexcv_base_kernels install
	@make -f Makefile_kernels install
	@make -f Makefile_apexcv_base install

uninstall:
	@make -f Makefile_emu uninstall
	@make -f Makefile_misc uninstall
	@make -f Makefile_apexcv_base_kernels uninstall
	@make -f Makefile_kernels uninstall
	@make -f Makefile_apexcv_base uninstall

clean:
	@make -f Makefile_emu clean
	@make -f Makefile_misc clean
	@make -f Makefile_apexcv_base_kernels clean
	@make -f Makefile_kernels clean
	@make -f Makefile_apexcv_base clean
