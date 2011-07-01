#!/bin/sh
make
cp ../images/intellus/intellus_routing-jffs2 /tftpboot/
cp ../images/intellus/vmlinux_routing.bin.gz /tftpboot/
cp ../images/intellus/intellus-update.img /var/lib/samba/share/Individual/tony/
