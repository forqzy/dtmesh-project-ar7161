#!/bin/sh
#IMG_MD5SUM, BLKSZ, IMG_SIZE, TOTAL and IMG_ARCH are filled in by mk_update_img
IMG_MD5SUM=__IMG_MD5SUM__
BLKSZ=__BLKSZ__
IMG_SIZE=__IMG_SIZE__
IMG_ARCH=__IMG_ARCH__
IMGFILE=$0 # this is us
NEW_ROOT=/.update_root
DD="dd bs=$BLKSZ"

echo "Setup tmpfs root file system" > /dev/console
cfg -e | sed -n 's/^export LAN_IPADDR="\([0-9.]*\)"/\1/p'  > /tmp/server_ip
# HTTPD PORT
HTTPD_PORT=$(cfg -e | awk -F"=" '/HTTPD_PORT/{printf $2 ; exit }')
[ "$HTTPD_PORT" != "80" -o "$HTTPD_PORT" != "" ] && echo $HTTPD_PORT > /tmp/httpd_port

mkdir -p $NEW_ROOT
mount -t ramfs -n none $NEW_ROOT -o sync,noatime,nodiratime,rw
cd $NEW_ROOT
#unpack the update rootfs
$DD if=$IMGFILE skip=$(($IMG_SIZE + 1)) 2> /dev/null | zcat | tar -x
mkdir old_root

mount -o move /sys  $NEW_ROOT/sys
mount -o move /tmp  $NEW_ROOT/tmp
mount -o move /proc $NEW_ROOT/proc
ln -sf tmp var
cp -a /dev $NEW_ROOT
echo "IMGFILE $IMGFILE" > $NEW_ROOT/tmp/imgparams
echo "IMG_SIZE $IMG_SIZE" >> $NEW_ROOT/tmp/imgparams
echo "BLKSZ $BLKSZ" >> $NEW_ROOT/tmp/imgparams


echo "Move root file system to $NEW_ROOT" > /dev/console

pivot_root . old_root
( cd / ; /update2.sh /tmp/imgparams ) </dev/console >/dev/console  2>&1 & PID=$!

exit
# end update script
