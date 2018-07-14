ifeq ($(OS),Windows_NT)
	EXE_SUFFIX = .exe
endif

DX1ELF2DFU_C = dx1elf2dfu.c

all: dx1elf2dfu$(EXE_SUFFIX)

dx1elf2dfu$(EXE_SUFFIX): Makefile $(DX1ELF2DFU_C)
	gcc $(DX1ELF2DFU_C) -o $@ $(CFLAGS)
	strip $@

clean:
	rm -f dx1elf2dfu$(EXE_SUFFIX)
