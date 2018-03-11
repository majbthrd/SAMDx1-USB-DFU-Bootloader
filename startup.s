/*
 * minimal startup for 1kByte USB DFU bootloader for Atmel SAMD11 microcontrollers
 *
 * Copyright (c) 2018, Peter Lawrence
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

  .syntax unified

  .global Reset_Handler

  .section .vectors, "ax"
  .code 16  
  .global _vectors  

_vectors:
  .word __stack_end__
#ifdef STARTUP_FROM_RESET
  .word Reset_Handler
#else
  .word loop
#endif /* STARTUP_FROM_RESET */
  .word loop
  .word loop

  .section .init, "ax"
  .thumb_func

Reset_Handler:

  /* optimized to assume non-zero sized 'data' section */
  ldr r0, =__data_end__
  ldr r1, =__data_start__
  ldr r2, =__data_load_start__
copy_loop:
  ldm r2!, {r3}
  stm r1!, {r3}
  cmp r1, r0
  bcc copy_loop

  /* optimized to assume non-zero sized 'bss' section */
  ldr r0, =__bss_end__
  ldr r1, =__bss_start__
zero_loop:
  movs r2, #0
  stm r1!, {r2}
  cmp r1, r0
  bcc zero_loop

  bl bootloader

  /* if bootloader returns, we proceed to the user app */

  ldr r1, =0x400 /* origin of user app */
  ldr r0, =0xE000ED08 /* VTOR register */
  str r1, [r0] /* point VTOR to user app */
  ldr r0, [r1] /* load stack pointer from user app */
  msr msp, r0
  msr psp, r0
  ldr r0, [r1, #4] /* load reset address from user app */
  mov pc, r0

loop:
  b .
