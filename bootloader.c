/*
 * 1kByte USB DFU bootloader for Atmel SAMD11 microcontrollers
 *
 * Copyright (c) 2018, Peter Lawrence
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

/* 
NOTES:
- anything pointed to by udc_mem[*].*.ADDR.reg *MUST* BE IN RAM and be 32-bit aligned... no exceptions
*/


/*- Includes ----------------------------------------------------------------*/
#include <stdbool.h>
#include <string.h>
#include <samd11.h>
#include "usb.h"
#include "nvm_data.h"
#include "usb_descriptors.h"

/*- Definitions -------------------------------------------------------------*/
#define USB_CMD(dir, rcpt, type) ((USB_##dir##_TRANSFER << 7) | (USB_##type##_REQUEST << 5) | (USB_##rcpt##_RECIPIENT << 0))

/*- Types -------------------------------------------------------------------*/
typedef struct
{
    UsbDeviceDescBank  out;
    UsbDeviceDescBank  in;
} udc_mem_t;

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

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static void udc_control_send(const uint32_t *data, uint32_t size)
{
  /* USB peripheral *only* reads valid data from 32-bit aligned RAM locations */
  udc_mem[0].in.ADDR.reg = (uint32_t)data;

  udc_mem[0].in.PCKSIZE.reg = USB_DEVICE_PCKSIZE_BYTE_COUNT(size) | USB_DEVICE_PCKSIZE_MULTI_PACKET_SIZE(0) | USB_DEVICE_PCKSIZE_SIZE(3 /*64 Byte*/);

  USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_TRCPT1;
  USB->DEVICE.DeviceEndpoint[0].EPSTATUSSET.bit.BK1RDY = 1;

  while (0 == USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.bit.TRCPT1);
}

//-----------------------------------------------------------------------------
static void udc_control_send_zlp(void)
{
  udc_control_send(NULL, 0); /* peripheral can't read from NULL address, but size is zero and this value takes less space to compile */
}

//-----------------------------------------------------------------------------
static void USB_Service(void)
{
  static uint32_t dfu_addr;

  if (USB->DEVICE.INTFLAG.bit.EORST) /* End Of Reset */
  {
    USB->DEVICE.INTFLAG.reg = USB_DEVICE_INTFLAG_EORST;
    USB->DEVICE.DADD.reg = USB_DEVICE_DADD_ADDEN;

    for (int ep = 0; ep < USB_EPT_NUM; ep++)
      USB->DEVICE.DeviceEndpoint[ep].EPCFG.reg = 0;

    USB->DEVICE.DeviceEndpoint[0].EPCFG.reg = USB_DEVICE_EPCFG_EPTYPE0(1 /*CONTROL*/) | USB_DEVICE_EPCFG_EPTYPE1(1 /*CONTROL*/);
    USB->DEVICE.DeviceEndpoint[0].EPSTATUSSET.bit.BK0RDY = 1;
    USB->DEVICE.DeviceEndpoint[0].EPSTATUSCLR.bit.BK1RDY = 1;

    udc_mem[0].in.ADDR.reg = (uint32_t)udc_ctrl_in_buf;
    udc_mem[0].in.PCKSIZE.reg = USB_DEVICE_PCKSIZE_BYTE_COUNT(0) | USB_DEVICE_PCKSIZE_MULTI_PACKET_SIZE(0) | USB_DEVICE_PCKSIZE_SIZE(3 /*64 Byte*/);

    udc_mem[0].out.ADDR.reg = (uint32_t)udc_ctrl_out_buf;
    udc_mem[0].out.PCKSIZE.reg = USB_DEVICE_PCKSIZE_BYTE_COUNT(64) | USB_DEVICE_PCKSIZE_MULTI_PACKET_SIZE(0) | USB_DEVICE_PCKSIZE_SIZE(3 /*64 Byte*/);

    USB->DEVICE.DeviceEndpoint[0].EPSTATUSCLR.bit.BK0RDY = 1;
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
    USB->DEVICE.DeviceEndpoint[0].EPSTATUSCLR.bit.BK0RDY = 1;

    usb_request_t *request = (usb_request_t *)udc_ctrl_out_buf;
    uint8_t type = request->wValue >> 8;
    uint8_t index = request->wValue & 0xff;
    uint16_t length = request->wLength;
    static uint32_t *dfu_status = dfu_status_choices + 0;

    switch (request->bmRequestType)
    {
    case USB_CMD(IN, DEVICE, STANDARD):
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
            USB->DEVICE.DeviceEndpoint[0].EPSTATUSSET.bit.STALLRQ1 = 1;
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
          USB->DEVICE.DeviceEndpoint[0].EPSTATUSSET.bit.STALLRQ1 = 1;
          break;
      }
      break;
    case USB_CMD(OUT, DEVICE, STANDARD):
      switch (request->bRequest)
      {
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
    case USB_CMD(IN, INTERFACE, STANDARD):
      switch (request->bRequest)
      {
        case USB_GET_STATUS:
          udc_control_send(dfu_status_choices + 0, 2); /* a 32-bit aligned zero in RAM is all we need */
          break;
      }
      break;
    case USB_CMD(OUT, INTERFACE, STANDARD):
      udc_control_send_zlp();
      break;
    case USB_CMD(IN, INTERFACE, CLASS):
      switch (request->bRequest)
      {
        case 0x03: // DFU_GETSTATUS
          udc_control_send(&dfu_status[0], 6);
          break;
        case 0x05: // DFU_GETSTATE
          udc_control_send(&dfu_status[1], 1);
          break;
        default: // DFU_UPLOAD & others
          /* no uploads here */
          udc_control_send_zlp();
          break;
      }
      break;
    case USB_CMD(OUT, INTERFACE, CLASS):
      if (0x01 /* DFU_DNLOAD */ == request->bRequest)
      {
        dfu_status = dfu_status_choices + 0;
        if (request->wLength)
        {
          dfu_status = dfu_status_choices + 2;
          dfu_addr = 0x400 + request->wValue * 64;
        }
      }
      /* 0x00 == DFU_DETACH, 0x04 == DFU_CLRSTATUS, 0x06 == DFU_ABORT, and 0x01 == DFU_DNLOAD */
      if (!dfu_addr)
        udc_control_send_zlp();
      break;
    }
  }
}

void bootloader(void)
{
  /* configure PA15 (bootloader entry pin used by SAM-BA) as input pull-up */
  PORT->Group[0].PINCFG[15].reg = PORT_PINCFG_PULLEN | PORT_PINCFG_INEN;
  PORT->Group[0].OUTSET.reg = (1UL << 15);

  if (PORT->Group[0].IN.reg & (1UL << 15))
    return; /* pin not grounded, so run user app */

  /*
  configure oscillator for crystal-free USB operation
  */
  
  SYSCTRL->OSC8M.bit.PRESC = 0;

  SYSCTRL->INTFLAG.reg = SYSCTRL_INTFLAG_BOD33RDY | SYSCTRL_INTFLAG_BOD33DET | SYSCTRL_INTFLAG_DFLLRDY;

  NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_RWS_DUAL;

  SYSCTRL->DFLLCTRL.reg = 0; // See Errata 9905
  while (!SYSCTRL->PCLKSR.bit.DFLLRDY);

  SYSCTRL->DFLLMUL.reg = SYSCTRL_DFLLMUL_MUL(48000);
  SYSCTRL->DFLLVAL.reg = SYSCTRL_DFLLVAL_COARSE( NVM_READ_CAL(NVM_DFLL48M_COARSE_CAL) ) | SYSCTRL_DFLLVAL_FINE( NVM_READ_CAL(NVM_DFLL48M_FINE_CAL) );

  SYSCTRL->DFLLCTRL.reg = SYSCTRL_DFLLCTRL_ENABLE | SYSCTRL_DFLLCTRL_USBCRM | SYSCTRL_DFLLCTRL_MODE | SYSCTRL_DFLLCTRL_BPLCKC | SYSCTRL_DFLLCTRL_CCDIS | SYSCTRL_DFLLCTRL_STABLE;

  while (!SYSCTRL->PCLKSR.bit.DFLLRDY);

  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(0) | GCLK_GENCTRL_SRC(GCLK_SOURCE_DFLL48M) | GCLK_GENCTRL_RUNSTDBY | GCLK_GENCTRL_GENEN;
  while (GCLK->STATUS.bit.SYNCBUSY);

  /*
  initialize USB
  */

  PORT->Group[0].PINCFG[24].reg |= PORT_PINCFG_PMUXEN;
  PORT->Group[0].PINCFG[25].reg |= PORT_PINCFG_PMUXEN;
  PORT->Group[0].PMUX[24>>1].reg = PORT_PMUX_PMUXO(PORT_PMUX_PMUXE_G_Val) | PORT_PMUX_PMUXE(PORT_PMUX_PMUXE_G_Val);

  PM->APBBMASK.reg |= PM_APBBMASK_USB;

  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_ID(USB_GCLK_ID) | GCLK_CLKCTRL_GEN(0);

  USB->DEVICE.CTRLA.reg = USB_CTRLA_SWRST;
  while (USB->DEVICE.SYNCBUSY.bit.SWRST);

  USB->DEVICE.PADCAL.reg = USB_PADCAL_TRANSN( NVM_READ_CAL(NVM_USB_TRANSN) ) | USB_PADCAL_TRANSP( NVM_READ_CAL(NVM_USB_TRANSP) ) | USB_PADCAL_TRIM( NVM_READ_CAL(NVM_USB_TRIM) );

  USB->DEVICE.DESCADD.reg = (uint32_t)udc_mem;

  USB->DEVICE.CTRLA.reg = USB_CTRLA_MODE_DEVICE | USB_CTRLA_RUNSTDBY;
  USB->DEVICE.CTRLB.reg = USB_DEVICE_CTRLB_SPDCONF_FS;
  USB->DEVICE.CTRLA.reg |= USB_CTRLA_ENABLE;

  /*
  service USB
  */

  while (1)
    USB_Service();
}
