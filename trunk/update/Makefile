ROOT_GZ=$(IMAGEPATH)/$(BOARD_TYPE)$(BUILD_CONFIG)-update-rootfs.gz
ROOTFSDIR:=$(shell mktemp -d)
TMP:=$(shell mktemp)

KERNEL_SIZE:=$(shell stat -L -c %s $(IMAGEPATH)/$(KTFILE).gz)
FILESYSTEM_SIZE:=$(shell stat -L -c %s $(IMAGEPATH)/$(JFFS2FILE))
VERSION=$(shell cat $(INSTALL_ROOT)/etc/firmware)

all: $(IMAGEPATH)/intellus-update.img

$(ROOT_GZ): update-mtd.sh update.cgi progress.html apply.cgi imgupdate_busybox.config
	cp imgupdate_busybox.config $(TOPDIR)/apps/$(BUSYBOX)/.config
	cd $(TOPDIR)/apps/$(BUSYBOX) && $(MAKE) clean && $(MAKE) PREFIX=$(ROOTFSDIR) install
	cd $(ROOTFSDIR) && mkdir -p www/tmpl www/cgi-bin proc sys tmp
	cp update-mtd.sh $(ROOTFSDIR)/update2.sh
	chmod +x $(ROOTFSDIR)/update2.sh
	sed -i 's/__KERNEL_SIZE__/$(KERNEL_SIZE)/g;s/__KERNEL_IMG__/$(KTFILE).gz/g' $(ROOTFSDIR)/update2.sh
	sed -i 's/__FILESYSTEM_SIZE__/$(FILESYSTEM_SIZE)/g;s/__FILESYSTEM_IMG__/$(JFFS2FILE)/g' $(ROOTFSDIR)/update2.sh
	cp progress.html $(ROOTFSDIR)/www/tmpl
	cp apply.cgi update.cgi $(ROOTFSDIR)/www/cgi-bin
	chmod +x  $(ROOTFSDIR)/www/cgi-bin/apply.cgi $(ROOTFSDIR)/www/cgi-bin/update.cgi
	cd $(ROOTFSDIR) && tar -c ./* | gzip -c > $@
	rm -fr $(ROOTFSDIR)

$(IMAGEPATH)/intellus-update.img: ./mk_update_img $(IMAGEPATH)/$(KTFILE).gz $(IMAGEPATH)/$(JFFS2FILE) update.sh $(ROOT_GZ)
	cd $(IMAGEPATH) && tar -chf $(TMP) $(KTFILE).gz $(JFFS2FILE)
	./mk_update_img $(TMP) update.sh $(ROOT_GZ) $(KERNELARCH) $@
	rm -fr $(TMP) $(ROOT_GZ)
