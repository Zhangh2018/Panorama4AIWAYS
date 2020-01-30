#!/bin/bash

TARGET=linux-arm64
COMPILER_PREFIX=aarch64-linux-gnu-
VERSION=1.62.0
FOLDER_NAME=boost_1_62_0
EXTENSION=

if [[ ${OSTYPE} == "msys" ]]; then
  EXTENSION=.exe
fi

ORIGIN=$PWD

if [ ! -f "$FOLDER_NAME.tar.gz" ] ; then
  wget -nc https://sourceforge.net/projects/boost/files/boost/$VERSION/$FOLDER_NAME.tar.gz || exit 1
fi

if [ ! -d "${FOLDER_NAME}" ] ; then
  tar xvf $FOLDER_NAME.tar.gz || exit 1
fi

cd $FOLDER_NAME || exit 1

./bootstrap.sh --prefix=$ORIGIN/$TARGET || exit 1
sed -i "s/using gcc ;/using gcc : arm : "$COMPILER_PREFIX"g++$EXTENSION ;/g" project-config.jam
./b2 -q --without-python toolset=gcc-arm abi=aapcs address-model=64 cxxflags=-D__arm64 target-os=linux install || exit 1

cd $ORIGIN/$TARGET/lib || exit 1

for f in *.so.$VERSION; do
mv "$f" "$(basename "$f" .so.$VERSION).so"
done

cp -r $ORIGIN/$TARGET/lib $ORIGIN/stage/ || exit 1
cp -r $ORIGIN/$TARGET/include/boost $ORIGIN/ || exit 1