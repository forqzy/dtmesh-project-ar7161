# eCos makefile

# This is a generated file - do not edit

export REPOSITORY := /home/fanjianqi/atheros_work/reference/trunk_11_6_22/boot/redboot/ecos/packages
export PREFIX := /home/fanjianqi/atheros_work/reference/trunk_11_6_22/boot/redboot/rom_bld/install
export COMMAND_PREFIX := mipsisa32-elf-
export CC := $(COMMAND_PREFIX)gcc
export OBJCOPY := $(COMMAND_PREFIX)objcopy
export HOST := UNIX
export AR := $(COMMAND_PREFIX)ar

PACKAGE := hal/mips/ar7100/current
OBJECT_PREFIX := hal_mips_ar7100
CFLAGS := -mips32 -EB -msoft-float -Wall -Wpointer-arith -Wstrict-prototypes -Winline -Wundef -Woverloaded-virtual -g -O2 -ffunction-sections -fdata-sections -fno-rtti -fno-exceptions -fvtable-gc -finit-priority -G0 -mlong-calls
LDFLAGS := -msoft-float -EB -g -nostdlib -Wl,--gc-sections -Wl,-static
VPATH := $(REPOSITORY)/$(PACKAGE)
INCLUDE_PATH := $(INCLUDE_PATH) -I$(PREFIX)/include $(foreach dir,$(VPATH),-I$(dir) -I$(dir)/src -I$(dir)/tests) -I.
MLT := $(wildcard $(REPOSITORY)/$(PACKAGE)/include/pkgconf/mlt*.ldi $(REPOSITORY)/$(PACKAGE)/include/pkgconf/mlt*.h)
TESTS := 

build: headers libtarget.a.stamp

LIBRARY := libtarget.a
COMPILE := src/platform.S src/plf_misc.c src/ar7100_serial.c
OBJECTS := $(COMPILE:.cxx=.o.d)
OBJECTS := $(OBJECTS:.c=.o.d)
OBJECTS := $(OBJECTS:.S=.o.d)

$(LIBRARY).stamp: $(OBJECTS)
	$(AR) rcs $(PREFIX)/lib/$(@:.stamp=) $(foreach obj,$?,$(dir $(obj))$(OBJECT_PREFIX)_$(notdir $(obj:.o.d=.o)))
	@cat $^ > $(@:.stamp=.deps)
	@touch $@

headers: mlt_headers $(PREFIX)/include/cyg/hal/hal_diag.h $(PREFIX)/include/cyg/hal/platform.inc $(PREFIX)/include/cyg/hal/ar7100_soc.h $(PREFIX)/include/cyg/hal/plf_stub.h $(PREFIX)/include/cyg/hal/plf_cache.h $(PREFIX)/include/cyg/hal/ar7100_sio.h $(PREFIX)/include/cyg/hal/platform.inc.orig $(PREFIX)/include/cyg/hal/plf_io.h $(PREFIX)/include/cyg/hal/plf_intr.h

$(PREFIX)/include/cyg/hal/hal_diag.h: $(REPOSITORY)/$(PACKAGE)/include/hal_diag.h
	@mkdir -p $(dir $@)
	@cp $< $@
	@chmod u+w $@

$(PREFIX)/include/cyg/hal/platform.inc: $(REPOSITORY)/$(PACKAGE)/include/platform.inc
	@mkdir -p $(dir $@)
	@cp $< $@
	@chmod u+w $@

$(PREFIX)/include/cyg/hal/ar7100_soc.h: $(REPOSITORY)/$(PACKAGE)/include/ar7100_soc.h
	@mkdir -p $(dir $@)
	@cp $< $@
	@chmod u+w $@

$(PREFIX)/include/cyg/hal/plf_stub.h: $(REPOSITORY)/$(PACKAGE)/include/plf_stub.h
	@mkdir -p $(dir $@)
	@cp $< $@
	@chmod u+w $@

$(PREFIX)/include/cyg/hal/plf_cache.h: $(REPOSITORY)/$(PACKAGE)/include/plf_cache.h
	@mkdir -p $(dir $@)
	@cp $< $@
	@chmod u+w $@

$(PREFIX)/include/cyg/hal/ar7100_sio.h: $(REPOSITORY)/$(PACKAGE)/include/ar7100_sio.h
	@mkdir -p $(dir $@)
	@cp $< $@
	@chmod u+w $@

$(PREFIX)/include/cyg/hal/platform.inc.orig: $(REPOSITORY)/$(PACKAGE)/include/platform.inc.orig
	@mkdir -p $(dir $@)
	@cp $< $@
	@chmod u+w $@

$(PREFIX)/include/cyg/hal/plf_io.h: $(REPOSITORY)/$(PACKAGE)/include/plf_io.h
	@mkdir -p $(dir $@)
	@cp $< $@
	@chmod u+w $@

$(PREFIX)/include/cyg/hal/plf_intr.h: $(REPOSITORY)/$(PACKAGE)/include/plf_intr.h
	@mkdir -p $(dir $@)
	@cp $< $@
	@chmod u+w $@

include $(REPOSITORY)/pkgconf/rules.mak

