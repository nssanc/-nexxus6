#!/bin/bash
export ARCH=arm
export SUBARCH=arm
export CROSS_COMPILE=/media/joe/linux_storage/toolchains/arm-eabi-4.8/bin/arm-eabi-
export LOCALVERSION="PlaceHolder_Name-Alpha"


make shamu_defconfig
make -j5

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
