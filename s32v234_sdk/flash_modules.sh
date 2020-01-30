#!/bin/sh

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

#cd /home/intesight/sourcecode/bsp18/modules

#rm * -rf

cd $DIR/libs/apex/drivers/kernel/build-v234ce-gnu-linux-d

make ARCH=arm64 CROSS_COMPILE=$DIR/../../../tools/toolchain/gcc-linaro-6.3.1-2017.05-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu- LINUX_S32V234_DIR=$DIR/../../linux_v4.14.34_bsp19.0 clean
make ARCH=arm64 CROSS_COMPILE=$DIR/../../../tools/toolchain/gcc-linaro-6.3.1-2017.05-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu- LINUX_S32V234_DIR=$DIR/../../linux_v4.14.34_bsp19.0

cp $DIR/libs/apex/drivers/kernel/build-v234ce-gnu-linux-d/apex.ko $DIR/../../../modules

cd $DIR/libs/utils/oal/kernel/build-v234ce-gnu-linux-o

make ARCH=arm64 CROSS_COMPILE=$DIR/../../../tools/toolchain/gcc-linaro-6.3.1-2017.05-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu- LINUX_S32V234_DIR=$DIR/../../linux_v4.14.34_bsp19.0 clean
make ARCH=arm64 CROSS_COMPILE=$DIR/../../../tools/toolchain/gcc-linaro-6.3.1-2017.05-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu- LINUX_S32V234_DIR=$DIR/../../linux_v4.14.34_bsp19.0

cp $DIR/libs/utils/oal/kernel/build-v234ce-gnu-linux-o/oal_cma.ko $DIR/../../../modules

cd $DIR/libs/isp/csi/kernel/build-v234ce-gnu-linux-d

make ARCH=arm64 CROSS_COMPILE=$DIR/../../../tools/toolchain/gcc-linaro-6.3.1-2017.05-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu- LINUX_S32V234_DIR=$DIR/../../linux_v4.14.34_bsp19.0 clean
make ARCH=arm64 CROSS_COMPILE=$DIR/../../../tools/toolchain/gcc-linaro-6.3.1-2017.05-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu- LINUX_S32V234_DIR=$DIR/../../linux_v4.14.34_bsp19.0

cp $DIR/libs/isp/csi/kernel/build-v234ce-gnu-linux-d/csi.ko $DIR/../../../modules

cd $DIR/libs/isp/cam_generic/kernel/build-v234ce-gnu-linux-d

make ARCH=arm64 CROSS_COMPILE=$DIR/../../../tools/toolchain/gcc-linaro-6.3.1-2017.05-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu- LINUX_S32V234_DIR=$DIR/../../linux_v4.14.34_bsp19.0 clean
make ARCH=arm64 CROSS_COMPILE=$DIR/../../../tools/toolchain/gcc-linaro-6.3.1-2017.05-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu- LINUX_S32V234_DIR=$DIR/../../linux_v4.14.34_bsp19.0

cp $DIR/libs/isp/cam_generic/kernel/build-v234ce-gnu-linux-d/cam.ko $DIR/../../../modules

cd $DIR/libs/isp/sequencer/kernel/build-v234ce-gnu-linux-d

make ARCH=arm64 CROSS_COMPILE=$DIR/../../../tools/toolchain/gcc-linaro-6.3.1-2017.05-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu- LINUX_S32V234_DIR=$DIR/../../linux_v4.14.34_bsp19.0 clean
make ARCH=arm64 CROSS_COMPILE=$DIR/../../../tools/toolchain/gcc-linaro-6.3.1-2017.05-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu- LINUX_S32V234_DIR=$DIR/../../linux_v4.14.34_bsp19.0

cp $DIR/libs/isp/sequencer/kernel/build-v234ce-gnu-linux-d/seq.ko $DIR/../../../modules

cd $DIR/libs/isp/fdma/kernel/build-v234ce-gnu-linux-d

make ARCH=arm64 CROSS_COMPILE=$DIR/../../../tools/toolchain/gcc-linaro-6.3.1-2017.05-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu- LINUX_S32V234_DIR=$DIR/../../linux_v4.14.34_bsp19.0 clean
make ARCH=arm64 CROSS_COMPILE=$DIR/../../../tools/toolchain/gcc-linaro-6.3.1-2017.05-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu- LINUX_S32V234_DIR=$DIR/../../linux_v4.14.34_bsp19.0

cp $DIR/libs/isp/fdma/kernel/build-v234ce-gnu-linux-d/fdma.ko $DIR/../../../modules

cd $DIR/libs/isp/h264enc/kernel/build-v234ce-gnu-linux-d

make ARCH=arm64 CROSS_COMPILE=$DIR/../../../tools/toolchain/gcc-linaro-6.3.1-2017.05-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu- LINUX_S32V234_DIR=$DIR/../../linux_v4.14.34_bsp19.0 clean
make ARCH=arm64 CROSS_COMPILE=$DIR/../../../tools/toolchain/gcc-linaro-6.3.1-2017.05-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu- LINUX_S32V234_DIR=$DIR/../../linux_v4.14.34_bsp19.0

cp $DIR/libs/isp/h264enc/kernel/build-v234ce-gnu-linux-d/h264enc.ko $DIR/../../../modules

cd $DIR/libs/isp/h264dec/kernel/build-v234ce-gnu-linux-d

make ARCH=arm64 CROSS_COMPILE=$DIR/../../../tools/toolchain/gcc-linaro-6.3.1-2017.05-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu- LINUX_S32V234_DIR=$DIR/../../linux_v4.14.34_bsp19.0 clean
make ARCH=arm64 CROSS_COMPILE=$DIR/../../../tools/toolchain/gcc-linaro-6.3.1-2017.05-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu- LINUX_S32V234_DIR=$DIR/../../linux_v4.14.34_bsp19.0

cp $DIR/libs/isp/h264dec/kernel/build-v234ce-gnu-linux-d/h264dcd.ko $DIR/../../../modules

cd $DIR/libs/isp/jpegdec/kernel/build-v234ce-gnu-linux-d

make ARCH=arm64 CROSS_COMPILE=$DIR/../../../tools/toolchain/gcc-linaro-6.3.1-2017.05-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu- LINUX_S32V234_DIR=$DIR/../../linux_v4.14.34_bsp19.0 clean
make ARCH=arm64 CROSS_COMPILE=$DIR/../../../tools/toolchain/gcc-linaro-6.3.1-2017.05-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu- LINUX_S32V234_DIR=$DIR/../../linux_v4.14.34_bsp19.0

cp $DIR/libs/isp/jpegdec/kernel/build-v234ce-gnu-linux-d/jpegdcd.ko $DIR/../../../modules

cd $DIR/libs/isp/viu/kernel/build-v234ce-gnu-linux-d

make ARCH=arm64 CROSS_COMPILE=$DIR/../../../tools/toolchain/gcc-linaro-6.3.1-2017.05-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu- LINUX_S32V234_DIR=$DIR/../../linux_v4.14.34_bsp19.0 clean
make ARCH=arm64 CROSS_COMPILE=$DIR/../../../tools/toolchain/gcc-linaro-6.3.1-2017.05-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu- LINUX_S32V234_DIR=$DIR/../../linux_v4.14.34_bsp19.0

cp $DIR/libs/isp/viu/kernel/build-v234ce-gnu-linux-d/viulite.ko $DIR/../../../modules

cd $DIR/libs/isp/sram/kernel/build-v234ce-gnu-linux-d

make ARCH=arm64 CROSS_COMPILE=$DIR/../../../tools/toolchain/gcc-linaro-6.3.1-2017.05-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu- LINUX_S32V234_DIR=$DIR/../../linux_v4.14.34_bsp19.0 clean
make ARCH=arm64 CROSS_COMPILE=$DIR/../../../tools/toolchain/gcc-linaro-6.3.1-2017.05-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu- LINUX_S32V234_DIR=$DIR/../../linux_v4.14.34_bsp19.0

cp $DIR/libs/isp/sram/kernel/build-v234ce-gnu-linux-d/sram.ko $DIR/../../../modules

cd $DIR/libs/io/Galcore


./build.sh KERNEL_DIR=$DIR/../../linux_v4.14.34_bsp19.0 TOOLCHAIN=$DIR/../../../tools/toolchain/gcc-linaro-6.3.1-2017.05-x86_64_aarch64-linux-gnu clean
./build.sh KERNEL_DIR=$DIR/../../linux_v4.14.34_bsp19.0 TOOLCHAIN=$DIR/../../../tools/toolchain/gcc-linaro-6.3.1-2017.05-x86_64_aarch64-linux-gnu

cp $DIR/libs/io/Galcore/galcore.ko $DIR/../../../modules

