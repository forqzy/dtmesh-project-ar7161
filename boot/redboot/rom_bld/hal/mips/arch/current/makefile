# eCos makefile

# This is a generated file - do not edit

export REPOSITORY := /home/fanjianqi/atheros_work/reference/trunk_11_6_22/boot/redboot/ecos/packages
export PREFIX := /home/fanjianqi/atheros_work/reference/trunk_11_6_22/boot/redboot/rom_bld/install
export COMMAND_PREFIX := mipsisa32-elf-
export CC := $(COMMAND_PREFIX)gcc
export OBJCOPY := $(COMMAND_PREFIX)objcopy
export HOST := UNIX
export AR := $(COMMAND_PREFIX)ar

PACKAGE := hal/mips/arch/current
OBJECT_PREFIX := hal_mips_arch
CFLAGS := -mips32 -EB -msoft-float -Wall -Wpointer-arith -Wstrict-prototypes -Winline -Wundef -Woverloaded-virtual -g -O2 -ffunction-sections -fdata-sections -fno-rtti -fno-exceptions -fvtable-gc -finit-priority -G0 -mlong-calls
LDFLAGS := -msoft-float -EB -g -nostdlib -Wl,--gc-sections -Wl,-static
VPATH := $(REPOSITORY)/$(PACKAGE)
INCLUDE_PATH := $(INCLUDE_PATH) -I$(PREFIX)/include $(foreach dir,$(VPATH),-I$(dir) -I$(dir)/src -I$(dir)/tests) -I.
MLT := $(wildcard $(REPOSITORY)/$(PACKAGE)/include/pkgconf/mlt*.ldi $(REPOSITORY)/$(PACKAGE)/include/pkgconf/mlt*.h)
TESTS := 

build: headers libtarget.a.stamp libextras.a.stamp

LIBRARY := libtarget.a
COMPILE := src/hal_misc.c src/context.S src/mips-stub.c src/mipsfp.c src/hal_syscall.c
OBJECTS := $(COMPILE:.cxx=.o.d)
OBJECTS := $(OBJECTS:.c=.o.d)
OBJECTS := $(OBJECTS:.S=.o.d)

$(LIBRARY).stamp: $(OBJECTS)
	$(AR) rcs $(PREFIX)/lib/$(@:.stamp=) $(foreach obj,$?,$(dir $(obj))$(OBJECT_PREFIX)_$(notdir $(obj:.o.d=.o)))
	@cat $^ > $(@:.stamp=.deps)
	@touch $@

LIBRARY := libextras.a
COMPILE := src/redboot_linux_exec.c
OBJECTS := $(COMPILE:.cxx=.o.d)
OBJECTS := $(OBJECTS:.c=.o.d)
OBJECTS := $(OBJECTS:.S=.o.d)

$(LIBRARY).stamp: $(OBJECTS)
	$(AR) rcs $(PREFIX)/lib/$(@:.stamp=) $(foreach obj,$?,$(dir $(obj))$(OBJECT_PREFIX)_$(notdir $(obj:.o.d=.o)))
	@cat $^ > $(@:.stamp=.deps)
	@touch $@

$(PREFIX)/lib/vectors.o: $(wildcard $(REPOSITORY)/$(PACKAGE)/src/vectors.S)
	$(CC) -Wp,-MD,vectors.tmp $(INCLUDE_PATH) $(CFLAGS) -c -o $@ $<
	@echo $@ ": \\" > $(notdir $@).deps
	@tail -n +2 vectors.tmp >> $(notdir $@).deps
	@echo >> $(notdir $@).deps
	@rm vectors.tmp
	

headers: mlt_headers $(PREFIX)/include/cyg/hal/mips-stub.h $(PREFIX)/include/cyg/hal/mips.inc $(PREFIX)/include/cyg/hal/basetype.h $(PREFIX)/include/cyg/hal/hal_intr.h $(PREFIX)/include/cyg/hal/hal_cache.h $(PREFIX)/include/cyg/hal/arch.inc $(PREFIX)/include/cyg/hal/mips-regs.h $(PREFIX)/include/cyg/hal/mips_opcode.h $(PREFIX)/include/cyg/hal/hal_arch.h $(PREFIX)/include/cyg/hal/hal_io.h

$(PREFIX)/include/cyg/hal/mips-stub.h: $(REPOSITORY)/$(PACKAGE)/include/mips-stub.h
	@mkdir -p $(dir $@)
	@cp $< $@
	@chmod u+w $@

$(PREFIX)/include/cyg/hal/mips.inc: $(REPOSITORY)/$(PACKAGE)/include/mips.inc
	@mkdir -p $(dir $@)
	@cp $< $@
	@chmod u+w $@

$(PREFIX)/include/cyg/hal/basetype.h: $(REPOSITORY)/$(PACKAGE)/include/basetype.h
	@mkdir -p $(dir $@)
	@cp $< $@
	@chmod u+w $@

$(PREFIX)/include/cyg/hal/hal_intr.h: $(REPOSITORY)/$(PACKAGE)/include/hal_intr.h
	@mkdir -p $(dir $@)
	@cp $< $@
	@chmod u+w $@

$(PREFIX)/include/cyg/hal/hal_cache.h: $(REPOSITORY)/$(PACKAGE)/include/hal_cache.h
	@mkdir -p $(dir $@)
	@cp $< $@
	@chmod u+w $@

$(PREFIX)/include/cyg/hal/arch.inc: $(REPOSITORY)/$(PACKAGE)/include/arch.inc
	@mkdir -p $(dir $@)
	@cp $< $@
	@chmod u+w $@

$(PREFIX)/include/cyg/hal/mips-regs.h: $(REPOSITORY)/$(PACKAGE)/include/mips-regs.h
	@mkdir -p $(dir $@)
	@cp $< $@
	@chmod u+w $@

$(PREFIX)/include/cyg/hal/mips_opcode.h: $(REPOSITORY)/$(PACKAGE)/include/mips_opcode.h
	@mkdir -p $(dir $@)
	@cp $< $@
	@chmod u+w $@

$(PREFIX)/include/cyg/hal/hal_arch.h: $(REPOSITORY)/$(PACKAGE)/include/hal_arch.h
	@mkdir -p $(dir $@)
	@cp $< $@
	@chmod u+w $@

$(PREFIX)/include/cyg/hal/hal_io.h: $(REPOSITORY)/$(PACKAGE)/include/hal_io.h
	@mkdir -p $(dir $@)
	@cp $< $@
	@chmod u+w $@

include $(REPOSITORY)/pkgconf/rules.mak

