# eCos makefile

# This is a generated file - do not edit

export REPOSITORY := /home/fanjianqi/atheros_work/reference/trunk_11_6_22/boot/redboot/ecos/packages
export PREFIX := /home/fanjianqi/atheros_work/reference/trunk_11_6_22/boot/redboot/rom_bld/install
export COMMAND_PREFIX := mipsisa32-elf-
export CC := $(COMMAND_PREFIX)gcc
export OBJCOPY := $(COMMAND_PREFIX)objcopy
export HOST := UNIX
export AR := $(COMMAND_PREFIX)ar

PACKAGE := compat/linux/current
OBJECT_PREFIX := compat_linux
CFLAGS := -mips32 -EB -msoft-float -Wall -Wpointer-arith -Wstrict-prototypes -Winline -Wundef -Woverloaded-virtual -g -O2 -ffunction-sections -fdata-sections -fno-rtti -fno-exceptions -fvtable-gc -finit-priority -G0 -mlong-calls
LDFLAGS := -msoft-float -EB -g -nostdlib -Wl,--gc-sections -Wl,-static
VPATH := $(REPOSITORY)/$(PACKAGE)
INCLUDE_PATH := $(INCLUDE_PATH) -I$(PREFIX)/include $(foreach dir,$(VPATH),-I$(dir) -I$(dir)/src -I$(dir)/tests) -I.
MLT := $(wildcard $(REPOSITORY)/$(PACKAGE)/include/pkgconf/mlt*.ldi $(REPOSITORY)/$(PACKAGE)/include/pkgconf/mlt*.h)
TESTS := 

build: headers libtarget.a.stamp

LIBRARY := libtarget.a
COMPILE := src/rbtree.c
OBJECTS := $(COMPILE:.cxx=.o.d)
OBJECTS := $(OBJECTS:.c=.o.d)
OBJECTS := $(OBJECTS:.S=.o.d)

$(LIBRARY).stamp: $(OBJECTS)
	$(AR) rcs $(PREFIX)/lib/$(@:.stamp=) $(foreach obj,$?,$(dir $(obj))$(OBJECT_PREFIX)_$(notdir $(obj:.o.d=.o)))
	@cat $^ > $(@:.stamp=.deps)
	@touch $@

headers: mlt_headers $(PREFIX)/include/asm/bug.h $(PREFIX)/include/asm/page.h $(PREFIX)/include/asm/atomic.h $(PREFIX)/include/asm/semaphore.h $(PREFIX)/include/linux/vmalloc.h $(PREFIX)/include/linux/stat.h $(PREFIX)/include/linux/errno.h $(PREFIX)/include/linux/rbtree.h $(PREFIX)/include/linux/slab.h $(PREFIX)/include/linux/zutil.h $(PREFIX)/include/linux/init.h $(PREFIX)/include/linux/version.h $(PREFIX)/include/linux/compiler.h $(PREFIX)/include/linux/workqueue.h $(PREFIX)/include/linux/TODO $(PREFIX)/include/linux/crc32.h $(PREFIX)/include/linux/completion.h $(PREFIX)/include/linux/wait.h $(PREFIX)/include/linux/spinlock.h $(PREFIX)/include/linux/string.h $(PREFIX)/include/linux/list.h $(PREFIX)/include/linux/pagemap.h $(PREFIX)/include/linux/kernel.h $(PREFIX)/include/linux/timer.h $(PREFIX)/include/linux/sched.h $(PREFIX)/include/linux/config.h $(PREFIX)/include/linux/zlib.h $(PREFIX)/include/linux/types.h $(PREFIX)/include/linux/fs.h $(PREFIX)/include/linux/mtd/mtd.h $(PREFIX)/include/linux/mtd/compatmac.h

$(PREFIX)/include/asm/bug.h: $(REPOSITORY)/$(PACKAGE)/include/asm/bug.h
	@mkdir -p $(dir $@)
	@cp $< $@
	@chmod u+w $@

$(PREFIX)/include/asm/page.h: $(REPOSITORY)/$(PACKAGE)/include/asm/page.h
	@mkdir -p $(dir $@)
	@cp $< $@
	@chmod u+w $@

$(PREFIX)/include/asm/atomic.h: $(REPOSITORY)/$(PACKAGE)/include/asm/atomic.h
	@mkdir -p $(dir $@)
	@cp $< $@
	@chmod u+w $@

$(PREFIX)/include/asm/semaphore.h: $(REPOSITORY)/$(PACKAGE)/include/asm/semaphore.h
	@mkdir -p $(dir $@)
	@cp $< $@
	@chmod u+w $@

$(PREFIX)/include/linux/vmalloc.h: $(REPOSITORY)/$(PACKAGE)/include/linux/vmalloc.h
	@mkdir -p $(dir $@)
	@cp $< $@
	@chmod u+w $@

$(PREFIX)/include/linux/stat.h: $(REPOSITORY)/$(PACKAGE)/include/linux/stat.h
	@mkdir -p $(dir $@)
	@cp $< $@
	@chmod u+w $@

$(PREFIX)/include/linux/errno.h: $(REPOSITORY)/$(PACKAGE)/include/linux/errno.h
	@mkdir -p $(dir $@)
	@cp $< $@
	@chmod u+w $@

$(PREFIX)/include/linux/rbtree.h: $(REPOSITORY)/$(PACKAGE)/include/linux/rbtree.h
	@mkdir -p $(dir $@)
	@cp $< $@
	@chmod u+w $@

$(PREFIX)/include/linux/slab.h: $(REPOSITORY)/$(PACKAGE)/include/linux/slab.h
	@mkdir -p $(dir $@)
	@cp $< $@
	@chmod u+w $@

$(PREFIX)/include/linux/zutil.h: $(REPOSITORY)/$(PACKAGE)/include/linux/zutil.h
	@mkdir -p $(dir $@)
	@cp $< $@
	@chmod u+w $@

$(PREFIX)/include/linux/init.h: $(REPOSITORY)/$(PACKAGE)/include/linux/init.h
	@mkdir -p $(dir $@)
	@cp $< $@
	@chmod u+w $@

$(PREFIX)/include/linux/version.h: $(REPOSITORY)/$(PACKAGE)/include/linux/version.h
	@mkdir -p $(dir $@)
	@cp $< $@
	@chmod u+w $@

$(PREFIX)/include/linux/compiler.h: $(REPOSITORY)/$(PACKAGE)/include/linux/compiler.h
	@mkdir -p $(dir $@)
	@cp $< $@
	@chmod u+w $@

$(PREFIX)/include/linux/workqueue.h: $(REPOSITORY)/$(PACKAGE)/include/linux/workqueue.h
	@mkdir -p $(dir $@)
	@cp $< $@
	@chmod u+w $@

$(PREFIX)/include/linux/TODO: $(REPOSITORY)/$(PACKAGE)/include/linux/TODO
	@mkdir -p $(dir $@)
	@cp $< $@
	@chmod u+w $@

$(PREFIX)/include/linux/crc32.h: $(REPOSITORY)/$(PACKAGE)/include/linux/crc32.h
	@mkdir -p $(dir $@)
	@cp $< $@
	@chmod u+w $@

$(PREFIX)/include/linux/completion.h: $(REPOSITORY)/$(PACKAGE)/include/linux/completion.h
	@mkdir -p $(dir $@)
	@cp $< $@
	@chmod u+w $@

$(PREFIX)/include/linux/wait.h: $(REPOSITORY)/$(PACKAGE)/include/linux/wait.h
	@mkdir -p $(dir $@)
	@cp $< $@
	@chmod u+w $@

$(PREFIX)/include/linux/spinlock.h: $(REPOSITORY)/$(PACKAGE)/include/linux/spinlock.h
	@mkdir -p $(dir $@)
	@cp $< $@
	@chmod u+w $@

$(PREFIX)/include/linux/string.h: $(REPOSITORY)/$(PACKAGE)/include/linux/string.h
	@mkdir -p $(dir $@)
	@cp $< $@
	@chmod u+w $@

$(PREFIX)/include/linux/list.h: $(REPOSITORY)/$(PACKAGE)/include/linux/list.h
	@mkdir -p $(dir $@)
	@cp $< $@
	@chmod u+w $@

$(PREFIX)/include/linux/pagemap.h: $(REPOSITORY)/$(PACKAGE)/include/linux/pagemap.h
	@mkdir -p $(dir $@)
	@cp $< $@
	@chmod u+w $@

$(PREFIX)/include/linux/kernel.h: $(REPOSITORY)/$(PACKAGE)/include/linux/kernel.h
	@mkdir -p $(dir $@)
	@cp $< $@
	@chmod u+w $@

$(PREFIX)/include/linux/timer.h: $(REPOSITORY)/$(PACKAGE)/include/linux/timer.h
	@mkdir -p $(dir $@)
	@cp $< $@
	@chmod u+w $@

$(PREFIX)/include/linux/sched.h: $(REPOSITORY)/$(PACKAGE)/include/linux/sched.h
	@mkdir -p $(dir $@)
	@cp $< $@
	@chmod u+w $@

$(PREFIX)/include/linux/config.h: $(REPOSITORY)/$(PACKAGE)/include/linux/config.h
	@mkdir -p $(dir $@)
	@cp $< $@
	@chmod u+w $@

$(PREFIX)/include/linux/zlib.h: $(REPOSITORY)/$(PACKAGE)/include/linux/zlib.h
	@mkdir -p $(dir $@)
	@cp $< $@
	@chmod u+w $@

$(PREFIX)/include/linux/types.h: $(REPOSITORY)/$(PACKAGE)/include/linux/types.h
	@mkdir -p $(dir $@)
	@cp $< $@
	@chmod u+w $@

$(PREFIX)/include/linux/fs.h: $(REPOSITORY)/$(PACKAGE)/include/linux/fs.h
	@mkdir -p $(dir $@)
	@cp $< $@
	@chmod u+w $@

$(PREFIX)/include/linux/mtd/mtd.h: $(REPOSITORY)/$(PACKAGE)/include/linux/mtd/mtd.h
	@mkdir -p $(dir $@)
	@cp $< $@
	@chmod u+w $@

$(PREFIX)/include/linux/mtd/compatmac.h: $(REPOSITORY)/$(PACKAGE)/include/linux/mtd/compatmac.h
	@mkdir -p $(dir $@)
	@cp $< $@
	@chmod u+w $@

include $(REPOSITORY)/pkgconf/rules.mak

