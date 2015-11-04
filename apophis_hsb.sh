#!/bin/bash
export ARCH=arm
export CROSS_COMPILE=/home/apophis9283/gcc/arm-eabi-5.2/bin/arm-eabi-
make vortex_defconfig
make -j8
