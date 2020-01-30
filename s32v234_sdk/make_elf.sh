#!/bin/sh

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

cd $DIR/demos/isp/max9286_96705_4channel_uyvy/build-v234ce-gnu-linux-o

make cleansub

export ARCH=arm64

export CROSS_COMPILE=aarch64-linux-gnu-

export LINUX_S32V234_DIR=$DIR/../../linux_v4.14.34_bsp19.0

export PATH=$DIR/../../../tools/toolchain/gcc-6.3.1-linaro-Earmv7-eabi/i686-linux/bin:$PATH

export PATH=$DIR/../../../tools/toolchain/gcc-linaro-6.3.1-2017.05-x86_64_aarch64-linux-gnu/bin:$PATH

make APU_COMP=nxp allsub

cp $DIR/demos/isp/max9286_96705_4channel_uyvy/build-v234ce-gnu-linux-o/max9286_96705_4channel_uyvy.elf $DIR/../../../modules
