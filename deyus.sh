#!/bin/bash
export ARCH=arm
export CROSS_COMPILE=/home/deyus/android/gcc/arm-eabi-6.0/arm-linux-gnueabi-6.0/bin/arm-eabi-
make .deyus
make -j1


if [ ! -f arch/arm/boot/zImage-dtb ]; then
echo "Build Failed"
else
echo "Copy kernel and build generate anykernel zip."
mv arch/arm/boot/zImage-dtb zip/anykernel/kernel/zImage

cd zip/anykernel
zip -9 -r ../DeYuS-KeRnEl.zip .
cd ../..
rm /home/deyus/DeYuS-KeRnEl.zip
mv zip/DeYuS-KeRnEl.zip /home/deyus/DeYuS-KeRnEl.zip

echo "Finished Zipping."
fi

#make ARCH=arm CROSS_COMPILE=~/home/deyus/android/gcc/arm-linux-androideabi-6.0/bin/arm-linux-androideabi- menuconfig
