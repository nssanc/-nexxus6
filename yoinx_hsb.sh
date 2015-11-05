#!/bin/bash

<<IMPORTANT

This file has been modified in order to allow for sharing between multiple people.
It now requires a local file called "vortex_local.sh" that should containt local variables
at a miniumum it needs the following 3 lines. This will allow environmental configuring on each 
machine. This "vortex_local.sh" file will be untracked (.gitignore). 

#!/bin/bash
export CROSS_COMPILE=/media/joe/linux_storage/toolchains/arm-eabi-4.8/bin/arm-eabi-
export CC_JOBS="-j5"


IMPORTANT


source hsb_local.sh
export ARCH=arm
export SUBARCH=arm


make HSB_defconfig
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
