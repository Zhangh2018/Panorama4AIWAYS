#!/bin/bash
##############################################################################
#
##############################################################################
#
# Copyright 2018 NXP 
# All Rights Reserved
#
##############################################################################
#
#  CONTENT
#    Common APU Emulator options for OpenCV makefiles
#
#  AUTHOR
#    Anca Dima
#    Mihail Marian Nistor
#
#  DATE
#    2018-11-27
#
##############################################################################

SDK_ROOT=../../../../s32v234_sdk
OCV_ROOT=$SDK_ROOT/../opencv_src/
NJOBS=$(($(nproc)*2))

. ../APU_Emulator_common.sh || exit 1

cmake -G "MSYS Makefiles" \
  -D CMAKE_INSTALL_PREFIX=$SDK_ROOT/3rdparty/ocv/win32-mingw \
  $APU_EMU_COMMON_OPTS \
  $OCV_ROOT || exit 1

echo "Generating done, building....."
make -j ${NJOBS} || exit 1
make install || exit 1
echo "Done!"