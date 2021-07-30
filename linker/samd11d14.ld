/*
modified from original (which can be found at https://github.com/ataradov/mcu-starter-projects):
+ formal stack declaration (rather than just use the end of RAM)
+ entry point of "Reset_Handler" instead of "irq_handler_reset"
+ used Rowley convention of section names (i.e. "data_load" instead of "etext")
+ flash size reduced to 1kByte (since bootloader must fit in this)
*/

/*
 * Copyright (c) 2016, Alex Taradov <alex@taradov.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

__STACKSIZE__ = 256;

MEMORY
{
  flash (rx) : ORIGIN = 0x00000000, LENGTH = 0x400 /* 1k */
  ram  (rwx) : ORIGIN = 0x20000000, LENGTH = 0x1000 /* 4k */
}

__top_flash = ORIGIN(flash) + LENGTH(flash);
__top_ram = ORIGIN(ram) + LENGTH(ram);

ENTRY(Reset_Handler)

SECTIONS
{
  .text : ALIGN(4)
  {
    FILL(0xff)
    KEEP(*(.vectors))
    *(.text*)
    *(.rodata)
    *(.rodata.*)
    . = ALIGN(4);
  } > flash

  . = ALIGN(4);
  __data_load_start__ = .;

  .uninit_RESERVED : ALIGN(4)
  {
    KEEP(*(.bss.$RESERVED*))
  } > ram

  .dont_move_block (NOLOAD): ALIGN(4)
  {
    *(.vectors_ram)
  }

  .data : ALIGN(4)
  {
    FILL(0xff)
    __data_start__ = .;
    *(.ramfunc .ramfunc.*);
    *(vtable)
    *(.data*)
    . = ALIGN(4);
    __data_end__ = .;
  } > ram AT > flash

  .bss : ALIGN(4)
  {
    __bss_start__ = .;
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    __bss_end__ = .;
    PROVIDE(_end = .);
  } > ram

  .stack : ALIGN(4)
  {
    __stack_start__ = .;
    *(.stack .stack.*)
    . = ALIGN(MAX(__stack_start__ + __STACKSIZE__, .), 8);
    __stack_end__ = .;
  } > ram

  __RAM_segment_used_end__ = .;
}

