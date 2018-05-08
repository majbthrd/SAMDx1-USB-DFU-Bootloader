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


/*- Includes ----------------------------------------------------------------*/
#include "usb.h"
#include "usb_descriptors.h"

/*- Variables ---------------------------------------------------------------*/
usb_device_descriptor_t usb_device_descriptor __attribute__ ((aligned (4))) = /* MUST BE IN RAM for USB peripheral */
{
  .bLength            = sizeof(usb_device_descriptor_t),
  .bDescriptorType    = USB_DEVICE_DESCRIPTOR,

  .bcdUSB                 = 0x0100,
  .bDeviceClass           = 254,
  .bDeviceSubClass        = 1, /* DFU */
  .bDeviceProtocol        = 0,

  .bMaxPacketSize0        = 64,
  .idVendor               = 0x1209,
  .idProduct              = 0x2003,
  .bcdDevice              = 0x0101,

  .iManufacturer          = USB_STR_ZERO,
  .iProduct               = USB_STR_ZERO,
  .iSerialNumber          = USB_STR_ZERO,

  .bNumConfigurations     = 1
};

usb_configuration_hierarchy_t usb_configuration_hierarchy __attribute__ ((aligned (4))) = /* MUST BE IN RAM for USB peripheral */
{
  .configuration =
  {
    .bLength             = sizeof(usb_configuration_descriptor_t),
    .bDescriptorType     = USB_CONFIGURATION_DESCRIPTOR,
    .wTotalLength        = sizeof(usb_configuration_hierarchy_t),
    .bNumInterfaces      = 1,
    .bConfigurationValue = 1,
    .iConfiguration      = USB_STR_ZERO,
    .bmAttributes        = 0x80,
    .bMaxPower           = 50, // 100 mA
  },

  .interface =
  {
    .bLength             = sizeof(usb_interface_descriptor_t),
    .bDescriptorType     = USB_INTERFACE_DESCRIPTOR,
    .bInterfaceNumber    = 0,
    .bAlternateSetting   = 0,
    .bNumEndpoints       = 0,
    .bInterfaceClass     = 254,
    .bInterfaceSubClass  = 1,
    .bInterfaceProtocol  = 0,
    .iInterface          = USB_STR_ZERO,
  },

  .dfu =
  {
    .bLength             = sizeof(usb_dfu_descriptor_t),
    .bDescriptorType     = 33,
    .bmAttributes        = 3,
    .wDetachTimeout      = 0,
    .wTransferSize       = 64,
    .bcdDFU              = 0x100,
  },
};
