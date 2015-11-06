#!/bin/bash
export ARCH=arm
export CROSS_COMPILE=/home/apophis9283/gcc/arm-eabi-6.0/bin/arm-eabi-
make HSB_defconfig
make -j8
