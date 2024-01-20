/*
  This provides USB mouse functionality similar to "mouseplay" in the "example-apps" directory of:
  https://github.com/majbthrd/PIC16F1-USB-DFU-Bootloader
*/

/*
 * Copyright (c) 2018, Peter Lawrence
 * Copyright (c) 2017, Alex Taradov <alex@taradov.com>
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

/*- Includes ----------------------------------------------------------------*/
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdalign.h>
#include <string.h>
#include "samd11.h"
#include "hal_gpio.h"
#include "nvm_data.h"
#include "usb.h"
#include "usb_hid.h"

/*- Definitions -------------------------------------------------------------*/

/*- Variables ---------------------------------------------------------------*/
static alignas(4) uint8_t app_hid_report[64];
static bool app_hid_report_free = true;
static unsigned tick_count;

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static void sys_init(void)
{
  uint32_t sn = 0;

#if 1
  /*
  configure oscillator for crystal-free USB operation (USBCRM / USB Clock Recovery Mode)
  */
  uint32_t coarse;

  NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_CACHEDIS | NVMCTRL_CTRLB_RWS(2);

  SYSCTRL->INTFLAG.reg = SYSCTRL_INTFLAG_BOD33RDY | SYSCTRL_INTFLAG_BOD33DET |
      SYSCTRL_INTFLAG_DFLLRDY;

  coarse = NVM_READ_CAL(NVM_DFLL48M_COARSE_CAL);

  SYSCTRL->DFLLCTRL.reg = 0; // See Errata 9905
  while (0 == (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY));

  /* This is the same step size used by MPLAB Harmony; seems reasonable since fstep=10 yields a typical settling time
   * of 200us with an input clock of 32kHz according to datasheet. Presumably we'd divide fstep
   * by 32 when reducing the input clock to 1kHz, but that would make it zero...
   */
  SYSCTRL->DFLLMUL.reg = SYSCTRL_DFLLMUL_MUL(48000) | SYSCTRL_DFLLMUL_CSTEP(1) | SYSCTRL_DFLLMUL_FSTEP(1);
  
  /* Since the DFLL is used in closed-loop, there is no need for a fine calibration from the fuses.
   * (It will be overwritten anyway). Using the middle value (512) follows the example in MPLAB
   * Harmony and saves 8 bytes of shift and mask instructions.
   */
  SYSCTRL->DFLLVAL.reg = SYSCTRL_DFLLVAL_COARSE( coarse ) | SYSCTRL_DFLLVAL_FINE(512);

  SYSCTRL->DFLLCTRL.reg = SYSCTRL_DFLLCTRL_ENABLE | SYSCTRL_DFLLCTRL_USBCRM |
      SYSCTRL_DFLLCTRL_MODE | SYSCTRL_DFLLCTRL_CCDIS;

  while (0 == (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY));

  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(0) | GCLK_GENCTRL_SRC(GCLK_SOURCE_DFLL48M) |
      GCLK_GENCTRL_RUNSTDBY | GCLK_GENCTRL_GENEN;
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);
#else
  /*
  configure oscillator for operation disciplined by external 32k crystal

  This can only be used on PCBs (such as Arduino Zero derived designs) that have these extra components populated.
  It *should* be wholly unnecessary to use this instead of the above USBCRM code.
  However, some problem (Sparkfun?) PCBs experience unreliable USB operation in USBCRM mode.
  */

  NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_RWS_DUAL;

  SYSCTRL->XOSC32K.reg = SYSCTRL_XOSC32K_STARTUP( 0x6u ) | SYSCTRL_XOSC32K_XTALEN | SYSCTRL_XOSC32K_EN32K;
  SYSCTRL->XOSC32K.reg |= SYSCTRL_XOSC32K_ENABLE;

  while (!(SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_XOSC32KRDY));

  GCLK->GENDIV.reg = GCLK_GENDIV_ID( 1u /* XOSC32K */ );

  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID( 1u /* XOSC32K */ ) | GCLK_GENCTRL_SRC_XOSC32K | GCLK_GENCTRL_GENEN;

  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID( 0u /* DFLL48M */ ) | GCLK_CLKCTRL_GEN_GCLK1 | GCLK_CLKCTRL_CLKEN;

  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

  SYSCTRL->DFLLCTRL.reg = 0; // See Errata 9905
  while (!SYSCTRL->PCLKSR.bit.DFLLRDY);

  SYSCTRL->DFLLMUL.reg = SYSCTRL_DFLLMUL_CSTEP( 31 ) | SYSCTRL_DFLLMUL_FSTEP( 511 ) | SYSCTRL_DFLLMUL_MUL(48000000ul / 32768ul);

  SYSCTRL->DFLLCTRL.reg = SYSCTRL_DFLLCTRL_ENABLE | SYSCTRL_DFLLCTRL_MODE | SYSCTRL_DFLLCTRL_WAITLOCK | SYSCTRL_DFLLCTRL_QLDIS;

  while ( !(SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLLCKC) || !(SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLLCKF) || !(SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY) );

  GCLK->GENDIV.reg = GCLK_GENDIV_ID( 0u /* MAIN */ );

  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID( 0u /* MAIN */ ) | GCLK_GENCTRL_SRC_DFLL48M | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN;

  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);
#endif

  sn ^= *(volatile uint32_t *)0x0080a00c;
  sn ^= *(volatile uint32_t *)0x0080a040;
  sn ^= *(volatile uint32_t *)0x0080a044;
  sn ^= *(volatile uint32_t *)0x0080a048;

  for (int i = 0; i < 8; i++)
    usb_serial_number[i] = "0123456789ABCDEF"[(sn >> (i * 4)) & 0xf];

  usb_serial_number[9] = 0;
}

//-----------------------------------------------------------------------------
void usb_hid_send_callback(void)
{
  app_hid_report_free = true;
}

//-----------------------------------------------------------------------------
static void send_buffer(void)
{
  app_hid_report_free = false;

  usb_hid_send(app_hid_report, 3);
}

//-----------------------------------------------------------------------------
void usb_configuration_callback(int config)
{
  (void)config;
  usb_hid_send_callback();
}

//-----------------------------------------------------------------------------
void usb_sof_callback(void)
{
  tick_count++;
}

//-----------------------------------------------------------------------------
static void hid_task(void)
{
  static unsigned table_index = 0;

  /* arbitrary mouse movement pattern to play back */
  const int8_t move_table[]=
  {
          /* 
          X, Y, (at time 0)
          X, Y, (at time 1)
          X, Y, (at time 2)
          ...
          */
          6, -2,
          2, -6,
          -2, -6,
          -6, -2,
          -6, 2,
          -2, 6,
          2, 6,
          6, 2,
  };

  if (tick_count < 64)
    return;

  if (!app_hid_report_free)
    return;

  /* table_index modulus 16 *AND* make table_index an even number */
  table_index &= 0xE;

  app_hid_report[0] = 0;
  app_hid_report[1] = move_table[table_index++];
  app_hid_report[2] = move_table[table_index++];

  send_buffer();
  tick_count = 0;
}

//-----------------------------------------------------------------------------
int main(void)
{
  sys_init();
  usb_init();
  usb_hid_init();

  while (1)
  {
    usb_task();
    hid_task();
  }

  return 0;
}
