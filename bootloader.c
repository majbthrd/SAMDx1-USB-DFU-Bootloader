/*
 * 1kByte USB DFU bootloader for Atmel SAMD11 microcontrollers
 *
 * Copyright (c) 2018-2020, Peter Lawrence
 * derived from https://github.com/ataradov/vcp Copyright (c) 2016, Alex Taradov <alex@taradov.com>
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

/* 
NOTES:
- anything pointed to by udc_mem[*].*.ADDR.reg *MUST* BE IN RAM and be 32-bit aligned... no exceptions
*/


/*- Includes ----------------------------------------------------------------*/
#include <stdbool.h>
#include <string.h>
#include <sam.h>
#include "usb.h"
#include "nvm_data.h"
#include "usb_descriptors.h"

/*- Definitions -------------------------------------------------------------*/
#define USE_DBL_TAP /* comment out to use GPIO input for bootloader entry */
#define REBOOT_AFTER_DOWNLOAD /* comment out to prevent boot into app after it has been downloaded */
#define USB_CMD(dir, rcpt, type) ((USB_##dir##_TRANSFER << 7) | (USB_##type##_REQUEST << 5) | (USB_##rcpt##_RECIPIENT << 0))
#define SIMPLE_USB_CMD(rcpt, type) ((USB_##type##_REQUEST << 5) | (USB_##rcpt##_RECIPIENT << 0))
#define DBL_TAP_MAGIC 0xf02669ef

/*- Types -------------------------------------------------------------------*/
typedef struct
{
    UsbDeviceDescBank  out;
    UsbDeviceDescBank  in;
} udc_mem_t;

typedef struct
{
  uint32_t reserved2;
  uint32_t reserved1;
  uint32_t reserved0;
  uint32_t double_tap_magic;
} bl_info_t;

/*- Variables ---------------------------------------------------------------*/
static uint32_t usb_config = 0;
static uint32_t dfu_status_choices[4] =
{ 
  0x00000000, 0x00000002, /* normal */
  0x00000000, 0x00000005, /* dl */
};

static udc_mem_t udc_mem[USB_EPT_NUM];
static uint32_t udc_ctrl_in_buf[16];
static uint32_t udc_ctrl_out_buf[16];

static volatile bl_info_t __attribute__((section(".bl_info"))) bl_info;

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static void __attribute__((noinline)) udc_control_send(const uint32_t *data, uint32_t size)
{
  /* USB peripheral *only* reads valid data from 32-bit aligned RAM locations */
  udc_mem[0].in.ADDR.reg = (uint32_t)data;

  udc_mem[0].in.PCKSIZE.reg = USB_DEVICE_PCKSIZE_BYTE_COUNT(size) | USB_DEVICE_PCKSIZE_MULTI_PACKET_SIZE(0) | USB_DEVICE_PCKSIZE_SIZE(3 /*64 Byte*/);

  USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_TRCPT1;
  USB->DEVICE.DeviceEndpoint[0].EPSTATUSSET.reg = USB_DEVICE_EPSTATUSSET_BK1RDY;

  while (0 == USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.bit.TRCPT1);
}

//-----------------------------------------------------------------------------
static void __attribute__((noinline)) udc_control_send_zlp(void)
{
  udc_control_send(NULL, 0); /* peripheral can't read from NULL address, but size is zero and this value takes less space to compile */
}

//-----------------------------------------------------------------------------
static void __attribute__((noinline)) USB_Service(void)
{
  static uint32_t dfu_addr;

  if (USB->DEVICE.INTFLAG.bit.EORST) /* End Of Reset */
  {
    USB->DEVICE.INTFLAG.reg = USB_DEVICE_INTFLAG_EORST;
    USB->DEVICE.DADD.reg = USB_DEVICE_DADD_ADDEN;

    udc_mem[0].in.ADDR.reg = (uint32_t)udc_ctrl_in_buf;
    udc_mem[0].in.PCKSIZE.reg = USB_DEVICE_PCKSIZE_BYTE_COUNT(0) | USB_DEVICE_PCKSIZE_MULTI_PACKET_SIZE(0) | USB_DEVICE_PCKSIZE_SIZE(3 /*64 Byte*/);

    udc_mem[0].out.ADDR.reg = (uint32_t)udc_ctrl_out_buf;
    udc_mem[0].out.PCKSIZE.reg = USB_DEVICE_PCKSIZE_BYTE_COUNT(64) | USB_DEVICE_PCKSIZE_MULTI_PACKET_SIZE(0) | USB_DEVICE_PCKSIZE_SIZE(3 /*64 Byte*/);


    USB->DEVICE.DeviceEndpoint[0].EPCFG.reg = USB_DEVICE_EPCFG_EPTYPE0(1 /*CONTROL*/) | USB_DEVICE_EPCFG_EPTYPE1(1 /*CONTROL*/);
    /* Allow SETUP and OUT transactions to be received, but NAK all IN transactions. */
    USB->DEVICE.DeviceEndpoint[0].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSCLR_BK1RDY | USB_DEVICE_EPSTATUSCLR_BK0RDY;
  }

  if (USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.bit.TRCPT0) /* Transmit Complete 0 */
  {
    if (dfu_addr)
    {
      if (0 == ((dfu_addr >> 6) & 0x3))
      {
        NVMCTRL->ADDR.reg = dfu_addr >> 1;
        NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD(NVMCTRL_CTRLA_CMD_ER);
        while (!NVMCTRL->INTFLAG.bit.READY);
      }

      uint16_t *nvm_addr = (uint16_t *)(dfu_addr);
      uint16_t *ram_addr = (uint16_t *)udc_ctrl_out_buf;
      for (unsigned i = 0; i < 32; i++)
        *nvm_addr++ = *ram_addr++;
      while (!NVMCTRL->INTFLAG.bit.READY);

      udc_control_send_zlp();
      dfu_addr = 0;
    }

    USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_TRCPT0;
  }

  if (USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.bit.RXSTP) /* Received Setup */
  {
    USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_RXSTP;
    USB->DEVICE.DeviceEndpoint[0].EPSTATUSCLR.reg = USB_DEVICE_EPSTATUSCLR_BK0RDY;

    usb_request_t *request = (usb_request_t *)udc_ctrl_out_buf;
    uint8_t type = request->wValue >> 8;
    uint16_t length = request->wLength;
    static uint32_t *dfu_status = dfu_status_choices + 0;

    /* for these other USB requests, we must examine all fields in bmRequestType */
    if (USB_CMD(OUT, INTERFACE, STANDARD) == request->bmRequestType)
    {
      udc_control_send_zlp();
      return;
    }

    /* for these "simple" USB requests, we can ignore the direction and use only bRequest */
    switch (request->bmRequestType & 0x7F)
    {
    case SIMPLE_USB_CMD(DEVICE, STANDARD):
    case SIMPLE_USB_CMD(INTERFACE, STANDARD):
      switch (request->bRequest)
      {
        case USB_GET_DESCRIPTOR:
          if (USB_DEVICE_DESCRIPTOR == type)
          {
            udc_control_send((uint32_t *)&usb_device_descriptor, length);
          }
          else if (USB_CONFIGURATION_DESCRIPTOR == type)
          {
            udc_control_send((uint32_t *)&usb_configuration_hierarchy, length);
          }
          else
          {
            USB->DEVICE.DeviceEndpoint[0].EPSTATUSSET.reg = USB_DEVICE_EPSTATUSSET_STALLRQ1;
          }
          break;
        case USB_GET_CONFIGURATION:
          udc_control_send(&usb_config, 1);
          break;
        case USB_GET_STATUS:
          udc_control_send(dfu_status_choices + 0, 2); /* a 32-bit aligned zero in RAM is all we need */
          break;
        case USB_SET_FEATURE:
        case USB_CLEAR_FEATURE:
          USB->DEVICE.DeviceEndpoint[0].EPSTATUSSET.reg = USB_DEVICE_EPSTATUSSET_STALLRQ1;
          break;
        case USB_SET_ADDRESS:
          udc_control_send_zlp();
          USB->DEVICE.DADD.reg = USB_DEVICE_DADD_ADDEN | USB_DEVICE_DADD_DADD(request->wValue);
          break;
        case USB_SET_CONFIGURATION:
          usb_config = request->wValue;
          udc_control_send_zlp();
          break;
      }
      break;
    case SIMPLE_USB_CMD(INTERFACE, CLASS):
      switch (request->bRequest)
      {
        case 0x03: // DFU_GETSTATUS
          udc_control_send(&dfu_status[0], 6);
          break;
        case 0x05: // DFU_GETSTATE
          udc_control_send(&dfu_status[1], 1);
          break;
        case 0x01: // DFU_DNLOAD
          dfu_status = dfu_status_choices + 0;
          if (request->wLength)
          {
            dfu_status = dfu_status_choices + 2;
            dfu_addr = APP_ORIGIN + request->wValue * 64;
          }
#ifdef REBOOT_AFTER_DOWNLOAD
          else
          {
            /* the download has now finished, so now reboot */
            WDT->CONFIG.reg = WDT_CONFIG_PER_8 | WDT_CONFIG_WINDOW_8;
            WDT->CTRL.reg = WDT_CTRL_ENABLE;
          }
#endif
          /* fall through */
        default: // DFU_UPLOAD & others
          /* 0x00 == DFU_DETACH, 0x04 == DFU_CLRSTATUS, 0x06 == DFU_ABORT, and 0x01 == DFU_DNLOAD and 0x02 == DFU_UPLOAD */
          if (!dfu_addr)
            udc_control_send_zlp();
          break;
      }
      break;
    }
  }
}

/* App origin passed from startup to reduce code size */
void bootloader(uint32_t app_origin)
{
#ifndef USE_DBL_TAP
  /* configure PA15 (bootloader entry pin used by SAM-BA) as input pull-up */
  PORT->Group[0].PINCFG[15].reg = PORT_PINCFG_PULLEN | PORT_PINCFG_INEN;
  PORT->Group[0].OUTSET.reg = (1UL << 15);
#endif

  PAC1->WPCLR.reg = 2; /* clear DSU */

  DSU->ADDR.reg = app_origin; /* start CRC check at beginning of user app */
  DSU->LENGTH.reg = *(volatile uint32_t *)(app_origin + 0x10); /* use length encoded into unused vector address in user app */

  /* ask DSU to compute CRC */
  DSU->DATA.reg = 0xFFFFFFFF;
  DSU->CTRL.reg = DSU_CTRL_CRC; /* Strobe bits; no need for read-modify-write */
  while (!DSU->STATUSA.bit.DONE);

  if (DSU->DATA.reg)
    goto run_bootloader; /* CRC failed, so run bootloader */

#ifndef USE_DBL_TAP
  if (!(PORT->Group[0].IN.reg & (1UL << 15)))
    goto run_bootloader; /* pin grounded, so run bootloader */

  return; /* we've checked everything and there is no reason to run the bootloader */
#else
  if (PM->RCAUSE.reg & PM_RCAUSE_POR)
    bl_info.double_tap_magic = 0; /* a power up event should never be considered a 'double tap' */

  if (bl_info.double_tap_magic == DBL_TAP_MAGIC)
  {
    /* a 'double tap' has happened, so run bootloader */
    bl_info.double_tap_magic = 0;
    goto run_bootloader;
  }

  /* postpone boot for a short period of time; if a second reset happens during this window, the "magic" value will remain */
  bl_info.double_tap_magic = DBL_TAP_MAGIC;

  /* Spinning with a volatile counter forces load/store; asm saves 12 bytes. */
  register uint32_t delay;
  __asm__ volatile (
    "\t.syntax unified\n"
    "\tMOVS %0, #1\n"
    "\tLSLS %0, %0, #18\n"
    "reset_wait_loop:\n"
    "\tSUBS %0, #1\n"
    "\tBNE reset_wait_loop\n"
    : "=l" (delay)
    :
    : "cc");

  /* however, if execution reaches this point, the window of opportunity has closed and the "magic" disappears  */
  bl_info.double_tap_magic = 0;
  return;
#endif

run_bootloader:
#if 1
  /*
  configure oscillator for crystal-free USB operation (USBCRM / USB Clock Recovery Mode)
  */

  NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_RWS_DUAL;

  SYSCTRL->DFLLCTRL.reg = 0; // See Errata 9905

  /* This is the same step size used by MPLAB Harmony; seems reasonable since fstep=10 yields a typical settling time
   * of 200us with an input clock of 32kHz according to datasheet. Presumably we'd divide fstep
   * by 32 when reducing the input clock to 1kHz, but that would make it zero...
   */
  SYSCTRL->DFLLMUL.reg = SYSCTRL_DFLLMUL_MUL(48000) | SYSCTRL_DFLLMUL_CSTEP(1) | SYSCTRL_DFLLMUL_FSTEP(1);
  
  /* Since the DFLL is used in closed-loop, there is no need for a fine calibration from the fuses.
   * (It will be overwritten anyway). Using the middle value (512) follows the example in MPLAB
   * Harmony and saves 8 bytes of shift and mask instructions.
   */
  SYSCTRL->DFLLVAL.reg = SYSCTRL_DFLLVAL_COARSE( NVM_READ_CAL(NVM_DFLL48M_COARSE_CAL) ) | SYSCTRL_DFLLVAL_FINE(512);

  SYSCTRL->DFLLCTRL.reg = SYSCTRL_DFLLCTRL_ENABLE | SYSCTRL_DFLLCTRL_USBCRM | SYSCTRL_DFLLCTRL_MODE | SYSCTRL_DFLLCTRL_BPLCKC | SYSCTRL_DFLLCTRL_CCDIS;

  /* No need to wait for DFLL ready signal; GCLK handles the wait in hardware
   * and won't switch until the DFLL signals to the GCLK that it's ready.
   */
  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(0) | GCLK_GENCTRL_SRC(GCLK_SOURCE_DFLL48M) | GCLK_GENCTRL_RUNSTDBY | GCLK_GENCTRL_GENEN;
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

//  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

  SYSCTRL->DFLLCTRL.reg = 0; // See Errata 9905
//  while (!SYSCTRL->PCLKSR.bit.DFLLRDY);

  SYSCTRL->DFLLMUL.reg = SYSCTRL_DFLLMUL_CSTEP( 31 ) | SYSCTRL_DFLLMUL_FSTEP( 511 ) | SYSCTRL_DFLLMUL_MUL(48000000ul / 32768ul);

  SYSCTRL->DFLLCTRL.reg = SYSCTRL_DFLLCTRL_ENABLE | SYSCTRL_DFLLCTRL_MODE | SYSCTRL_DFLLCTRL_WAITLOCK | SYSCTRL_DFLLCTRL_QLDIS;

  while ( !(SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLLCKC) || !(SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLLCKF) || !(SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY) );

  GCLK->GENDIV.reg = GCLK_GENDIV_ID( 0u /* MAIN */ );

  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID( 0u /* MAIN */ ) | GCLK_GENCTRL_SRC_DFLL48M | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN;

  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);
#endif

  /*
  initialize USB
  */

  PORT->Group[0].WRCONFIG.reg = PORT_WRCONFIG_HWSEL | PORT_WRCONFIG_WRPINCFG | PORT_WRCONFIG_WRPMUX | PORT_WRCONFIG_PMUXEN | PORT_WRCONFIG_PMUX(PORT_PMUX_PMUXE_G_Val) | PORT_WRCONFIG_PINMASK(0x0300);

  PM->APBBMASK.reg = PM_APBBMASK_USB | PM_APBBMASK_PORT | PM_APBBMASK_NVMCTRL | PM_APBBMASK_DSU;

  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_ID(USB_GCLK_ID) | GCLK_CLKCTRL_GEN(0);

  USB->DEVICE.PADCAL.reg = USB_PADCAL_TRANSN( NVM_READ_CAL(NVM_USB_TRANSN) ) | USB_PADCAL_TRANSP( NVM_READ_CAL(NVM_USB_TRANSP) ) | USB_PADCAL_TRIM( NVM_READ_CAL(NVM_USB_TRIM) );

  USB->DEVICE.DESCADD.reg = (uint32_t)udc_mem;

  USB->DEVICE.CTRLB.reg = USB_DEVICE_CTRLB_SPDCONF_FS;
  USB->DEVICE.CTRLA.reg = USB_CTRLA_MODE_DEVICE | USB_CTRLA_RUNSTDBY | USB_CTRLA_ENABLE;

  /*
  service USB
  */

  while (1)
    USB_Service();
}
