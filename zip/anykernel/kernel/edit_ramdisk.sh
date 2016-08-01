#!/sbin/sh
#ramdisk_gov_sed.sh by show-p1984
#Features:
#extracts ramdisk
#Force SystemLess Root install
#finds busbox in /system or sets default location if it cannot be found
#add init.d support if not already supported
#removes forced encryption
#disables mpdecision
#repacks the ramdisk

mkdir /tmp/ramdisk
cp /tmp/boot.img-ramdisk.gz /tmp/ramdisk/
cd /tmp/ramdisk/
gunzip -c /tmp/ramdisk/boot.img-ramdisk.gz | cpio -i
cd /

#remove cmdline parameters we do not want
#maxcpus=2 in this case, which limits smp activation to the first 2 cpus
echo $(cat /tmp/boot.img-cmdline | sed -e 's/maxcpus=[^ ]\+//')>/tmp/boot.img-cmdline
if [[ $(cat /tmp/boot.img-cmdline) != *"enforcing=0 androidboot.selinux=permissive"* ]]
then
echo $(/tmp/cat boot.img-cmdline) enforcing=0 androidboot.selinux=permissive > /tmp/boot.img-cmdline
fi

# Force Permissive on cmdline
sed -ri 's/ enforcing=[0-1]//g' /tmp/boot.img-cmdline
sed -ri 's/ androidboot.selinux=permissive|androidboot.selinux=enforcing|androidboot.selinux=disabled//g' /tmp/boot.img-cmdline
echo $(cat /tmp/boot.img-cmdline) enforcing=0 androidboot.selinux=permissive >/tmp/boot.img-cmdline

#Don't force encryption
if  grep -qr forceencrypt /tmp/ramdisk/fstab.shamu; then
   sed -i "s/forceencrypt/encryptable/" /tmp/ramdisk/fstab.shamu
fi

#Gain write access on /system
if  grep -qr ro.secure=1 /tmp/ramdisk/default.prop; then
   sed -i "s/ro.secure=1/ro.secure=0/" /tmp/ramdisk/default.prop
fi

#remove verity
if  grep -qr verity_load_state /tmp/ramdisk/init.shamu.rc; then
 sed -i "s/verity_load_state/#verity_load_state/" /tmp/ramdisk/init.shamu.rc
fi
if  grep -qr verity_update_state /tmp/ramdisk/init.shamu.rc; then
 sed -i "s/verity_update_state/#verity_update_state/" /tmp/ramdisk/init.shamu.rc
fi

#Force SystemLess Root
Rless=/data/.supersu
if grep "SYSTEMLESS=true" $Rless; then
echo SystemLess Root already forced
else
rm /data/.supersu
echo "SYSTEMLESS=true" >> $Rless
echo SystemLess Root forced
fi

#add init.d support if not already supported
#this is no longer needed as the ramdisk now inserts our modules, but we will
#keep this here for user comfort, since having run-parts init.d support is a
#good idea anyway.
#found=$(find /tmp/ramdisk/init.rc -type f | xargs grep -oh "start /system/bin/sysinit");
#if [ "$found" != 'start /system/bin/sysinit' ]; then
#	echo "Sysinit launcher already created"
#else
#	#append the new lines for this option at the bottom
#        echo "" >> /tmp/ramdisk/init.rc
#        echo "# Sysinit will run scripts in init.d folder" >> /tmp/ramdisk/init.rc
#	#Edit the property to one which is sure to not change so far
#        echo "on property:???" >> /tmp/ramdisk/init.rc
#        echo "    start /system/bin/sysinit" >> /tmp/ramdisk/init.rc
#fi
#Editing the sysinit to match /su/bin or /su/xbin location
if [ -f "/system/bin/sysinit" ]; then
	rm /system/bin/sysinit
	echo "#!system/bin/sh" >> /system/bin/sysinit
	echo "#Support below busybox location to run init.d script, remember to chmod your script as executable" >> /system/bin/sysinit
	echo "export PATH=/sbin:/system/sbin:/system/bin:/system/xbin:/su/bin:/su/xbin" >> /system/bin/sysinit
	echo 'for i in /system/etc/init.d/*; do' >> /system/bin/sysinit
	echo '	if [ -x $i ]; then' >> /system/bin/sysinit
	echo '	log -p i -t Init.d Script $i; $i; fi' >> /system/bin/sysinit
	echo "done" >> /system/bin/sysinit
	chmod 755 /system/bin/sysinit
fi

#Make MPD and T-E as RW only
if [ -f "/system/bin/mpdecision" ]; then
chmod 644 /system/bin/mpdecision
fi
if [ -f "/system/bin/thermal-engine" ]; then
chmod 644 /system/bin/thermal-engine
fi

#copy fstab
cp /tmp/fstab.shamu /tmp/ramdisk/fstab.shamu
chmod 750 /tmp/ramdisk/fstab.shamu

#copy custom init.shamu.power.rc
cp /tmp/init.shamu.power.rc /tmp/ramdisk/init.shamu.power.rc
chmod 750 /tmp/ramdisk/init.shamu.power.rc

rm /tmp/ramdisk/boot.img-ramdisk.gz
rm /tmp/boot.img-ramdisk.gz
cd /tmp/ramdisk/
find . | cpio -o -H newc | gzip > ../boot.img-ramdisk.gz
cd /
rm -rf /tmp/ramdisk

