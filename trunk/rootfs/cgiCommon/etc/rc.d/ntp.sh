#!/bin/sh
. /etc/ath/apcfg

killall -q ntpclient


if [ "${NTP_HOSTNAME}" = "" ]; then
	exit 0
fi

#NTP and TimeZone
if [ "$NETWORK_MODE" = "Router" ]; then
	echo "$TIMEZONE" > /tmp/TZ
	echo "$TIMEZONE" > /etc/tmpTZ
	sed -e 's#.*_\(-*\)0*\(.*\)#GMT-\1\2#' /etc/tmpTZ > /etc/tmpTZ2
	sed -e 's#\(.*\)--\(.*\)#\1\2#' /etc/tmpTZ2 > /etc/TZ	
	rm -rf /etc/tmpTZ
	rm -rf /etc/tmpTZ2
	ntpclient -s -c 0 -h ${NTP_HOSTNAME} -i ${NTPSync} >> /tmp/TZ & 
fi




