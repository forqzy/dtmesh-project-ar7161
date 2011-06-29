ECOS_GLOBAL_CFLAGS = -mips32 -EB -msoft-float -Wall -Wpointer-arith -Wstrict-prototypes -Winline -Wundef -Woverloaded-virtual -g -O2 -ffunction-sections -fdata-sections -fno-rtti -fno-exceptions -fvtable-gc -finit-priority -G0 -mlong-calls
ECOS_GLOBAL_LDFLAGS = -msoft-float -EB -g -nostdlib -Wl,--gc-sections -Wl,-static
ECOS_COMMAND_PREFIX = mipsisa32-elf-
