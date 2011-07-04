#
# Makefile for Atheros tools.
#

CC=${TOOLPREFIX}gcc

ifeq (${BUILD_STATIC}, y)
LDFLAGS= -static
endif

CGIMAIN_LINKS := APBasicConfig APChannels APRadioConfig APStatistics APStatus VAPcfg \
	APFanConfig APGPIOConfig APTemperature

ifeq ($(ATH_SINGLE_CFG),1)
CFLAGS=-DATH_SINGLE_CFG=1 
endif

$(CGIMAIN_LINKS):
	@cd $(INSTALL_ROOT)/usr/www/cgi-bin && \
	ln -sf cgiMain $@ 

cgiMain: cgiMain.c $(CGIMAIN_LINKS)
	${CC} -o cgiMain cgiMain.c
	cp cgiMain $(INSTALL_ROOT)/usr/www/cgi-bin

upload: upload.c
	${CC} -o upload upload.c
	cp upload $(INSTALL_ROOT)/usr/www/cgi-bin

upgrade: upgrade.c
	${CC} -o upgrade upgrade.c
	cp upgrade $(INSTALL_ROOT)/usr/www/cgi-bin

wlanconfig: wlanconfig.c
	${CC} -o wlanconfig -I ../ -include ../include/compat.h wlanconfig.c
