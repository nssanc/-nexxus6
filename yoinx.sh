#!/bin/bash
export CROSS_COMPILE="ccache /media/joe/linux_storage/toolchains/UBERTC-arm-eabi-6.0/bin/arm-eabi-"

export HOST_CC="ccache gcc"
export CC_JOBS="-j5"
export ARCH=arm
export SUBARCH=arm


make Singularity_defconfig 
make $CC_JOBS

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

