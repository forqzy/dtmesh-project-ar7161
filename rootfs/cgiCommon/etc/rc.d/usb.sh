#!/bin/sh

if [ "${ACTION}" = "remove" ]; then
    umount  /mnt/usba        
    umount  /mnt/usbb              

#if [ "${ACTION}" = "add" ]; then
else
    sleep 6
    tmp=$(cat /proc/partitions | awk '/sda1/{printf $4}')
    if [ "${tmp}" = "sda1" ]; then
        mount /dev/sda1 /mnt/usba
    fi
    tmp=$(cat /proc/partitions | awk '/sdb1/{printf $4}')
    if [ "${tmp}" = "sdb1" ]; then                                           
        mount /dev/sdb1 /mnt/usbb                                       
    fi
fi


