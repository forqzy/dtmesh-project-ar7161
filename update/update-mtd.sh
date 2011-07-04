#!/bin/sh
PARAMFILE=$1
[ ! -e "$PARAMFILE" ] && echo "Cannot find image parameter file $PARAMFILE" && exit

IMG_SIZE=$(sed -n 's/IMG_SIZE[[:space:]]*//p' $PARAMFILE)
IMGFILE=$(sed -n 's/IMGFILE[[:space:]]*//p' $PARAMFILE)
BLKSZ=$(sed -n 's/BLKSZ[[:space:]]*//p' $PARAMFILE)

UPDLOG=/tmp/update.log
DD="dd bs=$BLKSZ"

echo "Removing dependencies on flash file system"
keeplist="sh init hostapd wpa_supplicant watchdog"
allprocs=$(fuser -m /old_root)
for p in $allprocs ; do
    name=$(sed -n 's/^Name:[[:space:]]*//p' /proc/$p/status)
    for k in $keeplist ; do
	if [ "$k" = "$name" ] ; then
	    echo "skipping" $name
	    break
	fi
    done
    if [ "$k" != "$name" ] ; then
	echo "stopping" $name
	kill -9 $p
    fi
done

echo "Flushing cached disk pages"
# The next steps are important.  The only process keeping /old_root
# busy is init.  But the kernel may have dirty pages in the cache.
# We need to first commit the dirty pages, then drain any remaining
# pages in the cache. This will avoid write accesses after the 
# low level block replacement has started
sync                       # commit dirty pages to disk
sysctl -w vm.drop_caches=3 2> /dev/null > /dev/null # flush page, dentry and inode caches
umount -r /old_root 2> /dev/null > /dev/null       # will remount read-only

cat <<EOF > /www/upgrade.php
<html><head>
<title>Access Point</title>
<META HTTP-EQUIV="refresh" CONTENT="0; URL=/cgi-bin/update.cgi">
</head></html>
EOF

echo "image copy output:" > $UPDLOG

[ -f "/tmp/httpd_port" ] && HTTPD_PORT=$(cat /tmp/httpd_port) || HTTPD_PORT=80

cat <<EOF > /tmp/httpd.conf
.php:text/html
EOF

echo "starting new httpd"
/usr/sbin/httpd -h /www -c /tmp/httpd.conf -p $HTTPD_PORT

$DD if=$IMGFILE skip=1 count=$IMG_SIZE | zcat | tar -x # get vmlinux and filesystem
rm  -f $IMGFILE

ERASE_BLKSZ=$(($(cat /proc/mtd | awk '/vmlinux/ { printf "0x%s", $3}')))
VMLINUX_MTD=$(cat /proc/mtd | awk -F ":" '/vmlinux/ { printf $1}')
VMLINUX_MTD_SIZE=$(($(cat /proc/mtd | awk '/vmlinux/ { printf "0x%s", $2}')/$ERASE_BLKSZ))
KERNEL_SIZE=$(( (__KERNEL_SIZE__ + $ERASE_BLKSZ - 1)/$ERASE_BLKSZ))

ERASE_BLKSZ=$(($(cat /proc/mtd | awk '/filesystem[^_]/ { printf "0x%s", $3}')))
FILESYSTEM_MTD=$(cat /proc/mtd | awk -F ":" '/filesystem[^_]/ { printf $1}')
FILESYSTEM_MTD_SIZE=$(($(cat /proc/mtd | awk '/filesystem[^_]/ { printf "0x%s", $2}')/$ERASE_BLKSZ))
FILESYSTEM_SIZE=$(( (__FILESYSTEM_SIZE__ + $ERASE_BLKSZ - 1)/$ERASE_BLKSZ))

[ $KERNEL_SIZE -gt $VMLINUX_MTD_SIZE ] && \
    echo "kernel size is too large ($KERNEL_SIZE > $VMLINUX_MTD_SIZE), aborting" \
    && exit 1

[ $FILESYSTEM_SIZE -gt $FILESYSTEM_MTD_SIZE ] && \
    echo "kernel size is too large ($FILESYSTEM_SIZE > $FILESYSTEM_MTD_SIZE), aborting" \
    && exit 1

[ ! -x "/sbin/mtd" ] && \
    echo "/sbin/mtd needed for partition erasure, aborting" \
    && exit 1

echo "TOTAL $(($KERNEL_SIZE + $FILESYSTEM_SIZE))" >> $PARAMFILE
echo "SUBTOTAL 0" >>$PARAMFILE 

echo "Starting kernel image update"
/sbin/mtd -ql write /__KERNEL_IMG__ /dev/$VMLINUX_MTD 2>> $UPDLOG & PID1=$!

echo "LOG $UPDLOG" >>$PARAMFILE 
echo "PID $PID1"  >>$PARAMFILE

while [ -n "$(ps | awk '/'$PID1'/ {print $1}')" ] ; do 
    sleep 1
done
 
echo "Starting file system image update"
/sbin/mtd -ql write /__FILESYSTEM_IMG__ /dev/$FILESYSTEM_MTD 2>> $UPDLOG & PID2=$!

DONE=0

sed -i 's/SUBTOTAL .*/SUBTOTAL '$KERNEL_SIZE'/;s/PID .*/PID '$PID2'/' $PARAMFILE

while [ -n "$(ps | awk '/'$PID2'/ {print $1}')" ] ; do 
    sleep 1
done
 
