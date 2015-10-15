#!/bin/bash
export ARCH=arm
export SUBARCH=arm
export CROSS_COMPILE=/media/joe/linux_storage/toolchains/arm-eabi-4.8/bin/arm-eabi-
make shamu_defconfig
make -j6
