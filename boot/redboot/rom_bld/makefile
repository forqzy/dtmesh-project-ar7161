# eCos makefile

# This is a generated file - do not edit

export REPOSITORY := /home/fanjianqi/atheros_work/reference/trunk_11_6_22/boot/redboot/ecos/packages
export HOST := UNIX
export PREFIX := /home/fanjianqi/atheros_work/reference/trunk_11_6_22/boot/redboot/rom_bld/install
export COMMAND_PREFIX := mipsisa32-elf-
export CC := $(COMMAND_PREFIX)gcc
export OBJCOPY := $(COMMAND_PREFIX)objcopy
export AR := $(COMMAND_PREFIX)ar

.PHONY: default build clean tests headers

build: headers $(PREFIX)/include/pkgconf/ecos.mak
	$(MAKE) -r -C hal/mips/pb44/current $(PREFIX)/include/cyg/hal/plf_defs.inc
	$(MAKE) -r -C services/memalloc/common/current heapgeninc.tcl
	$(MAKE) -r -C services/memalloc/common/current heaps.cxx
	$(MAKE) -r -C hal/mips/arch/current $@
	$(MAKE) -r -C hal/mips/mips32/current $@
	$(MAKE) -r -C hal/mips/ar7100/current $@
	$(MAKE) -r -C hal/mips/pb44/current $@
	$(MAKE) -r -C devs/eth/mips/ar7100/current $@
	$(MAKE) -r -C devs/flash/mips/ar7100/current $@
	$(MAKE) -r -C hal/common/current $@
	$(MAKE) -r -C infra/current $@
	$(MAKE) -r -C redboot/current $@
	$(MAKE) -r -C isoinfra/current $@
	$(MAKE) -r -C language/c/libc/string/current $@
	$(MAKE) -r -C services/crc/current $@
	$(MAKE) -r -C services/memalloc/common/current $@
	$(MAKE) -r -C services/compress/7zip/current $@
	$(MAKE) -r -C services/compress/zlib/current $@
	$(MAKE) -r -C io/eth/current $@
	$(MAKE) -r -C io/fileio/current $@
	$(MAKE) -r -C io/flash/current $@
	$(MAKE) -r -C io/common/current $@
	$(MAKE) -r -C error/current $@
	$(MAKE) -r -C compat/linux/current $@
	$(MAKE) -r -C hal/common/current $(PREFIX)/lib/extras.o
	$(MAKE) -r -C hal/mips/arch/current $(PREFIX)/lib/vectors.o
	$(MAKE) -r -C hal/mips/mips32/current $(PREFIX)/lib/target.ld
	$(MAKE) -r -C redboot/current $(PREFIX)/bin/redboot.elf
	$(MAKE) -r -C hal/mips/pb44/current $(PREFIX)/bin/redboot.srec
	@echo $@ finished

clean:
	$(MAKE) -r -C hal/mips/arch/current $@
	$(MAKE) -r -C hal/mips/mips32/current $@
	$(MAKE) -r -C hal/mips/ar7100/current $@
	$(MAKE) -r -C hal/mips/pb44/current $@
	$(MAKE) -r -C devs/eth/mips/ar7100/current $@
	$(MAKE) -r -C devs/flash/mips/ar7100/current $@
	$(MAKE) -r -C hal/common/current $@
	$(MAKE) -r -C infra/current $@
	$(MAKE) -r -C redboot/current $@
	$(MAKE) -r -C isoinfra/current $@
	$(MAKE) -r -C language/c/libc/string/current $@
	$(MAKE) -r -C services/crc/current $@
	$(MAKE) -r -C services/memalloc/common/current $@
	$(MAKE) -r -C services/compress/7zip/current $@
	$(MAKE) -r -C services/compress/zlib/current $@
	$(MAKE) -r -C io/eth/current $@
	$(MAKE) -r -C io/fileio/current $@
	$(MAKE) -r -C io/flash/current $@
	$(MAKE) -r -C io/common/current $@
	$(MAKE) -r -C error/current $@
	$(MAKE) -r -C compat/linux/current $@
	@echo $@ finished

tests: build
	$(MAKE) -r -C hal/mips/arch/current $@
	$(MAKE) -r -C hal/mips/mips32/current $@
	$(MAKE) -r -C hal/mips/ar7100/current $@
	$(MAKE) -r -C hal/mips/pb44/current $@
	$(MAKE) -r -C devs/eth/mips/ar7100/current $@
	$(MAKE) -r -C devs/flash/mips/ar7100/current $@
	$(MAKE) -r -C hal/common/current $@
	$(MAKE) -r -C infra/current $@
	$(MAKE) -r -C redboot/current $@
	$(MAKE) -r -C isoinfra/current $@
	$(MAKE) -r -C language/c/libc/string/current $@
	$(MAKE) -r -C services/crc/current $@
	$(MAKE) -r -C services/memalloc/common/current $@
	$(MAKE) -r -C services/compress/7zip/current $@
	$(MAKE) -r -C services/compress/zlib/current $@
	$(MAKE) -r -C io/eth/current $@
	$(MAKE) -r -C io/fileio/current $@
	$(MAKE) -r -C io/flash/current $@
	$(MAKE) -r -C io/common/current $@
	$(MAKE) -r -C error/current $@
	$(MAKE) -r -C compat/linux/current $@
	@echo $@ finished

headers:
	$(MAKE) -r -C hal/mips/arch/current $@
	$(MAKE) -r -C hal/mips/mips32/current $@
	$(MAKE) -r -C hal/mips/ar7100/current $@
	$(MAKE) -r -C hal/mips/pb44/current $@
	$(MAKE) -r -C devs/eth/mips/ar7100/current $@
	$(MAKE) -r -C devs/flash/mips/ar7100/current $@
	$(MAKE) -r -C hal/common/current $@
	$(MAKE) -r -C infra/current $@
	$(MAKE) -r -C redboot/current $@
	$(MAKE) -r -C isoinfra/current $@
	$(MAKE) -r -C language/c/libc/string/current $@
	$(MAKE) -r -C services/crc/current $@
	$(MAKE) -r -C services/memalloc/common/current $@
	$(MAKE) -r -C services/compress/7zip/current $@
	$(MAKE) -r -C services/compress/zlib/current $@
	$(MAKE) -r -C io/eth/current $@
	$(MAKE) -r -C io/fileio/current $@
	$(MAKE) -r -C io/flash/current $@
	$(MAKE) -r -C io/common/current $@
	$(MAKE) -r -C error/current $@
	$(MAKE) -r -C compat/linux/current $@
	@echo $@ finished

$(PREFIX)/include/pkgconf/ecos.mak: makefile
	@echo 'ECOS_GLOBAL_CFLAGS = -mips32 -EB -msoft-float -Wall -Wpointer-arith -Wstrict-prototypes -Winline -Wundef -Woverloaded-virtual -g -O2 -ffunction-sections -fdata-sections -fno-rtti -fno-exceptions -fvtable-gc -finit-priority -G0 -mlong-calls' > $@
	@echo 'ECOS_GLOBAL_LDFLAGS = -msoft-float -EB -g -nostdlib -Wl,--gc-sections -Wl,-static' >> $@
	@echo 'ECOS_COMMAND_PREFIX = $(COMMAND_PREFIX)' >> $@

