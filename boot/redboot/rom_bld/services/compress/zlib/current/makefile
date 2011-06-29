# eCos makefile

# This is a generated file - do not edit

export REPOSITORY := /home/fanjianqi/atheros_work/reference/trunk_11_6_22/boot/redboot/ecos/packages
export PREFIX := /home/fanjianqi/atheros_work/reference/trunk_11_6_22/boot/redboot/rom_bld/install
export COMMAND_PREFIX := mipsisa32-elf-
export CC := $(COMMAND_PREFIX)gcc
export OBJCOPY := $(COMMAND_PREFIX)objcopy
export HOST := UNIX
export AR := $(COMMAND_PREFIX)ar

PACKAGE := services/compress/zlib/current
OBJECT_PREFIX := services_compress_zlib
CFLAGS := -mips32 -EB -msoft-float -Wall -Wpointer-arith -Winline -Wundef -Woverloaded-virtual -g -O2 -ffunction-sections -fdata-sections -fno-rtti -fno-exceptions -fvtable-gc -finit-priority -G0 -mlong-calls -D__ECOS__ -DNO_ERRNO_H
LDFLAGS := -msoft-float -EB -g -nostdlib -Wl,--gc-sections -Wl,-static
VPATH := $(REPOSITORY)/$(PACKAGE)
INCLUDE_PATH := $(INCLUDE_PATH) -I$(PREFIX)/include $(foreach dir,$(VPATH),-I$(dir) -I$(dir)/src -I$(dir)/tests) -I.
MLT := $(wildcard $(REPOSITORY)/$(PACKAGE)/include/pkgconf/mlt*.ldi $(REPOSITORY)/$(PACKAGE)/include/pkgconf/mlt*.h)
TESTS := tests/zlib1.c tests/zlib2.c

build: headers libtarget.a.stamp

LIBRARY := libtarget.a
COMPILE := src/adler32.c src/compress.c src/uncompr.c src/zutil.c src/trees.c src/deflate.c src/infback.c src/inffast.c src/inflate.c src/inftrees.c
OBJECTS := $(COMPILE:.cxx=.o.d)
OBJECTS := $(OBJECTS:.c=.o.d)
OBJECTS := $(OBJECTS:.S=.o.d)

$(LIBRARY).stamp: $(OBJECTS)
	$(AR) rcs $(PREFIX)/lib/$(@:.stamp=) $(foreach obj,$?,$(dir $(obj))$(OBJECT_PREFIX)_$(notdir $(obj:.o.d=.o)))
	@cat $^ > $(@:.stamp=.deps)
	@touch $@

headers: mlt_headers $(PREFIX)/include/cyg/compress/zconf.h $(PREFIX)/include/cyg/compress/zlib.h

$(PREFIX)/include/cyg/compress/zconf.h: $(REPOSITORY)/$(PACKAGE)/include/zconf.h
	@mkdir -p $(dir $@)
	@cp $< $@
	@chmod u+w $@

$(PREFIX)/include/cyg/compress/zlib.h: $(REPOSITORY)/$(PACKAGE)/include/zlib.h
	@mkdir -p $(dir $@)
	@cp $< $@
	@chmod u+w $@

include $(REPOSITORY)/pkgconf/rules.mak

