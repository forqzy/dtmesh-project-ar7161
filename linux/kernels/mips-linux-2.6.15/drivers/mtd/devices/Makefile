#
# linux/drivers/devices/Makefile
#
# $Id: //depot/sw/releases/7.3_AP/linux/kernels/mips-linux-2.6.15/drivers/mtd/devices/Makefile#2 $

#                       *** BIG UGLY NOTE ***
#
# The removal of get_module_symbol() and replacement with
# inter_module_register() et al has introduced a link order dependency
# here where previously there was none.  We now have to ensure that
# doc200[01].o are linked before docprobe.o

obj-$(CONFIG_MTD_DOC2000)	+= doc2000.o
obj-$(CONFIG_MTD_DOC2001)	+= doc2001.o
obj-$(CONFIG_MTD_DOC2001PLUS)	+= doc2001plus.o
obj-$(CONFIG_MTD_DOCPROBE)	+= docprobe.o
obj-$(CONFIG_MTD_DOCECC)	+= docecc.o
obj-$(CONFIG_MTD_SLRAM)		+= slram.o
obj-$(CONFIG_MTD_PHRAM)		+= phram.o
obj-$(CONFIG_MTD_PMC551)	+= pmc551.o
obj-$(CONFIG_MTD_MS02NV)	+= ms02-nv.o
obj-$(CONFIG_MTD_MTDRAM)	+= mtdram.o
obj-$(CONFIG_MTD_LART)		+= lart.o
obj-$(CONFIG_MTD_BLKMTD)	+= blkmtd.o
obj-$(CONFIG_MTD_BLOCK2MTD)	+= block2mtd.o
obj-$(CONFIG_MTD_AR7100_SPI_FLASH)	+= ar7100_flash.o
obj-$(CONFIG_MTD_AR9100_PARALLEL_FLASH)	+= ar9100_pflash.o
obj-$(CONFIG_MTD_AR7240_SPI_FLASH)	+= ar7240_flash.o
ifeq ($(ATH_SST_FLASH), y)
CFLAGS_ar7240_flash.o += -DATH_SST_FLASH=1
endif
