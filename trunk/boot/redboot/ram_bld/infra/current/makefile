# eCos makefile

# This is a generated file - do not edit

export REPOSITORY := /home/fanjianqi/atheros_work/reference/trunk_11_6_22/boot/redboot/ecos/packages
export PREFIX := /home/fanjianqi/atheros_work/reference/trunk_11_6_22/boot/redboot/ram_bld/install
export COMMAND_PREFIX := mipsisa32-elf-
export CC := $(COMMAND_PREFIX)gcc
export OBJCOPY := $(COMMAND_PREFIX)objcopy
export HOST := UNIX
export AR := $(COMMAND_PREFIX)ar

PACKAGE := infra/current
OBJECT_PREFIX := infra
CFLAGS := -mips32 -EB -msoft-float -Wall -Wpointer-arith -Wstrict-prototypes -Winline -Wundef -Woverloaded-virtual -g -O2 -ffunction-sections -fdata-sections -fno-rtti -fno-exceptions -fvtable-gc -finit-priority -G0 -mlong-calls
LDFLAGS := -msoft-float -EB -g -nostdlib -Wl,-static -Wl,--fatal-warnings
VPATH := $(REPOSITORY)/$(PACKAGE)
INCLUDE_PATH := $(INCLUDE_PATH) -I$(PREFIX)/include $(foreach dir,$(VPATH),-I$(dir) -I$(dir)/src -I$(dir)/tests) -I.
MLT := $(wildcard $(REPOSITORY)/$(PACKAGE)/include/pkgconf/mlt*.ldi $(REPOSITORY)/$(PACKAGE)/include/pkgconf/mlt*.h)
TESTS := tests/cxxsupp

build: headers libtarget.a.stamp

LIBRARY := libtarget.a
COMPILE := src/startup.cxx src/prestart.cxx src/pkgstart.cxx src/userstart.cxx src/dummyxxmain.cxx src/null.cxx src/simple.cxx src/fancy.cxx src/buffer.cxx src/diag.cxx src/tcdiag.cxx src/memcpy.c src/memset.c src/delete.cxx src/eprintf.c src/pure.cxx src/abort.cxx
OBJECTS := $(COMPILE:.cxx=.o.d)
OBJECTS := $(OBJECTS:.c=.o.d)
OBJECTS := $(OBJECTS:.S=.o.d)

$(LIBRARY).stamp: $(OBJECTS)
	$(AR) rcs $(PREFIX)/lib/$(@:.stamp=) $(foreach obj,$?,$(dir $(obj))$(OBJECT_PREFIX)_$(notdir $(obj:.o.d=.o)))
	@cat $^ > $(@:.stamp=.deps)
	@touch $@

headers: mlt_headers $(PREFIX)/include/cyg/infra/cyg_type.inc $(PREFIX)/include/cyg/infra/cyg_ass.h $(PREFIX)/include/cyg/infra/testcase.h $(PREFIX)/include/cyg/infra/diag.h $(PREFIX)/include/cyg/infra/cyg_type.h $(PREFIX)/include/cyg/infra/clist.hxx $(PREFIX)/include/cyg/infra/cyg_trac.h

$(PREFIX)/include/cyg/infra/cyg_type.inc: $(REPOSITORY)/$(PACKAGE)/include/cyg_type.inc
	@mkdir -p $(dir $@)
	@cp $< $@
	@chmod u+w $@

$(PREFIX)/include/cyg/infra/cyg_ass.h: $(REPOSITORY)/$(PACKAGE)/include/cyg_ass.h
	@mkdir -p $(dir $@)
	@cp $< $@
	@chmod u+w $@

$(PREFIX)/include/cyg/infra/testcase.h: $(REPOSITORY)/$(PACKAGE)/include/testcase.h
	@mkdir -p $(dir $@)
	@cp $< $@
	@chmod u+w $@

$(PREFIX)/include/cyg/infra/diag.h: $(REPOSITORY)/$(PACKAGE)/include/diag.h
	@mkdir -p $(dir $@)
	@cp $< $@
	@chmod u+w $@

$(PREFIX)/include/cyg/infra/cyg_type.h: $(REPOSITORY)/$(PACKAGE)/include/cyg_type.h
	@mkdir -p $(dir $@)
	@cp $< $@
	@chmod u+w $@

$(PREFIX)/include/cyg/infra/clist.hxx: $(REPOSITORY)/$(PACKAGE)/include/clist.hxx
	@mkdir -p $(dir $@)
	@cp $< $@
	@chmod u+w $@

$(PREFIX)/include/cyg/infra/cyg_trac.h: $(REPOSITORY)/$(PACKAGE)/include/cyg_trac.h
	@mkdir -p $(dir $@)
	@cp $< $@
	@chmod u+w $@

include $(REPOSITORY)/pkgconf/rules.mak

