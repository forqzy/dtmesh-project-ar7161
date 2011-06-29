# eCos makefile

# This is a generated file - do not edit

export REPOSITORY := /home/fanjianqi/atheros_work/reference/trunk_11_6_22/boot/redboot/ecos/packages
export PREFIX := /home/fanjianqi/atheros_work/reference/trunk_11_6_22/boot/redboot/rom_bld/install
export COMMAND_PREFIX := mipsisa32-elf-
export CC := $(COMMAND_PREFIX)gcc
export OBJCOPY := $(COMMAND_PREFIX)objcopy
export HOST := UNIX
export AR := $(COMMAND_PREFIX)ar

PACKAGE := hal/mips/pb44/current
OBJECT_PREFIX := hal_mips_pb44
CFLAGS := -mips32 -EB -msoft-float -Wall -Wpointer-arith -Wstrict-prototypes -Winline -Wundef -Woverloaded-virtual -g -O2 -ffunction-sections -fdata-sections -fno-rtti -fno-exceptions -fvtable-gc -finit-priority -G0 -mlong-calls
LDFLAGS := -msoft-float -EB -g -nostdlib -Wl,--gc-sections -Wl,-static
VPATH := $(REPOSITORY)/$(PACKAGE)
INCLUDE_PATH := $(INCLUDE_PATH) -I$(PREFIX)/include $(foreach dir,$(VPATH),-I$(dir) -I$(dir)/src -I$(dir)/tests) -I.
MLT := $(wildcard $(REPOSITORY)/$(PACKAGE)/include/pkgconf/mlt*.ldi $(REPOSITORY)/$(PACKAGE)/include/pkgconf/mlt*.h)
TESTS := 

build: headers libtarget.a.stamp

LIBRARY := libtarget.a
COMPILE := src/plf_flash.c src/board_misc.c
OBJECTS := $(COMPILE:.cxx=.o.d)
OBJECTS := $(OBJECTS:.c=.o.d)
OBJECTS := $(OBJECTS:.S=.o.d)

$(LIBRARY).stamp: $(OBJECTS)
	$(AR) rcs $(PREFIX)/lib/$(@:.stamp=) $(foreach obj,$?,$(dir $(obj))$(OBJECT_PREFIX)_$(notdir $(obj:.o.d=.o)))
	@cat $^ > $(@:.stamp=.deps)
	@touch $@

$(PREFIX)/include/cyg/hal/plf_defs.inc: $(wildcard $(REPOSITORY)/$(PACKAGE)/src/plf_mk_defs.c)
	$(CC) $(CFLAGS) $(INCLUDE_PATH) -Wp,-MD,plf_defs.tmp -o plf_mk_defs.tmp -S $<
	fgrep .equ plf_mk_defs.tmp | sed s/#// > $@
	@echo $@ ": \\" > $(notdir $@).deps
	@tail -n +2 plf_defs.tmp >> $(notdir $@).deps
	@echo >> $(notdir $@).deps
	@rm plf_defs.tmp plf_mk_defs.tmp
	

$(PREFIX)/bin/redboot.srec: $(wildcard $(PREFIX)/bin/redboot.elf)
	$(OBJCOPY) --strip-all $< $(@:.srec=.img)
	$(OBJCOPY) -O srec $< $@
	

headers: mlt_headers

include $(REPOSITORY)/pkgconf/rules.mak

