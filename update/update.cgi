#!/bin/sh 
# jjPlus Corp. Copyright 2008
PARAMFILE=/tmp/imgparams
HOSTNAME=$(cat /proc/sys/kernel/hostname)

[ "$HOSTNAME" = "(none)" ] && HOSTNAME=""
 
#this is a cgi script to update the flash image using dd
if [ -f "$PARAMFILE" ] ; then
    TOTAL=$(awk '/^TOTAL/ { print $2 }' $PARAMFILE)
    SUBTOTAL=$(awk '/^SUBTOTAL/ { print $2 }' $PARAMFILE)
    UPDLOG=$(awk '/^LOG/ { print $2 }' $PARAMFILE)
    UPDPID=$(awk '/^PID/ { print $2 }' $PARAMFILE)
else
    TOTAL=1 # needed below until parent update script writes to $PARAMFILE 
fi

#This is the image update

if [ -f "$UPDLOG" -a -n "$UPDPID" ] ; then 
    [ "$(ps | awk '/'$UPDPID'/ {print $1}')" = "$UPDPID" ] && kill -SIGUSR1 $UPDPID 2> /dev/null > /dev/null
    DONE=$(sed -n '$ s/^\([0-9]*\).*/\1/p' $UPDLOG)
fi

[ -z "$SUBTOTAL" ] && SUBTOTAL=0
[ -z "$DONE" ] && DONE=0

[  "$DONE" -eq "$SUBTOTAL" ] && SUBTOTAL=0

DONE=$(($DONE + $SUBTOTAL))
percent=$(( (($DONE * 20) / $TOTAL ) *5 ))
pixels=$(( $percent * 2 ))

if [ "$DONE" -lt "$TOTAL" ] ; then
    sed  's/__PROGRESS_TEXT__/Upgrade started. Please wait for completion.Do not unplug the power./;s/__PIXELS__/'$pixels'/;s/__PERCENT__/'$percent'/;s/__HOSTNAME__/'$HOSTNAME'/' /www/tmpl/progress.html
else      # show reboot form
    [ -e "/proc/mtd" ] && JFFS2_MTD=$(cat /proc/mtd | awk -F ":" '/rootfs_data/ { printf $1}')
    if [ -n "$JFFS2_MTD" ] ; then
	sed 's/__PROGRESS_TEXT__/Upgrade complete/;s/__PIXELS__/200/;s/__PERCENT__/100/;s/__HOSTNAME__/'$HOSTNAME'/;s/<META.*progress\.html">//;s,</div></div></body>,</div><form method="POST" action="/cgi-bin/apply.cgi"><br><input type="submit" name="reboot" value="Reboot"></form></div></body>,' /www/tmpl/progress.html
    else
	sed 's/__PROGRESS_TEXT__/Upgrade complete/;s/__PIXELS__/200/;s/__PERCENT__/100/;s/__HOSTNAME__/'$HOSTNAME'/;s/<META.*progress\.html">//;s,</div></div></body>,</div><form method="POST" action="/cgi-bin/apply.cgi"><input type="submit" name="reboot" value="Reboot"></form></div></body>,' /www/tmpl/progress.html
    fi
fi
