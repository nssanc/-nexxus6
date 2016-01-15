#!/bin/bash
export ARCH=arm
export CROSS_COMPILE=/home/apophis9283/gcc/arm-eabi-5.3/bin/arm-eabi-
make Singularity_defconfig
make -j8


if [ ! -f arch/arm/boot/zImage-dtb ]; then
echo "Build Failed"
else
echo "Copy kernel and build generate anykernel zip."
mv arch/arm/boot/zImage-dtb zip/anykernel/kernel/zImage

cd zip/anykernel
zip -9 -r ../SINGULARITY-V-.zip .
cd ../..

echo "Finished Zipping."
fi
