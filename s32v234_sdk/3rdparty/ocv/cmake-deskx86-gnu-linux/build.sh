#!/bin/bash
##############################################################################
#
#  Freescale Semiconductor, Inc.
#  (c) Copyright 2013-2016 Freescale Semiconductor, Inc.
#  Copyright 2017-2018 NXP
#  ALL RIGHTS RESERVED.
#
##############################################################################
#
#  CONTENT
#    Generator of OpenCV makefiles for Linux desktop compilation
#
#  AUTHOR
#    Tomas Babinec
#    Mihail Marian Nistor
#
#  DATE
#    2018-11-27
#
##############################################################################
SDK_ROOT=../../../../s32v234_sdk
OCV_ROOT=$SDK_ROOT/../opencv_src
NJOBS=$(($(nproc)*2))
. ../APU_Emulator_common.sh || exit 1

cmake 												\
  -D CMAKE_INSTALL_PREFIX=$SDK_ROOT/3rdparty/ocv/linux-x86					\
  $APU_EMU_COMMON_OPTS \
  $OCV_ROOT || exit 1

echo "Generating done, building....."

make -j ${NJOBS} || exit 1

make install || exit 1
echo "Done!"
