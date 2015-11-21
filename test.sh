#!/bin/bash
export ARCH=arm
export CROSS_COMPILE=/home/apophis9283/gcc/arm-eabi-6.0/bin/arm-eabi-
make HSB_defconfig
make -j1


if [ ! -f arch/arm/boot/zImage-dtb ]; then
echo "Build Failed"
else
echo "Copy kernel and build generate anykernel zip."
mv arch/arm/boot/zImage-dtb zip/anykernel/kernel/zImage

cd zip/anykernel
zip -9 -r ../test_kernel.zip .
cd ../..

echo "Finished Zipping."
fi
