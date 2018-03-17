/*
  This source code is an amalgam of:
  usb_cdc.c from https://github.com/ataradov/vcp *AND*
  usb.c     from https://github.com/ataradov/free-dap,
  combined with a custom "usb_hid_report_descriptor" array
*/

/*
 * Copyright (c) 2016,2017, Alex Taradov <alex@taradov.com>
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
#include <stdbool.h>
#include <stdalign.h>
#include <string.h>
#include "utils.h"
#include "usb.h"
#include "usb_std.h"
#include "usb_hid.h"
#include "usb_descriptors.h"

/*- Prototypes --------------------------------------------------------------*/
static void usb_hid_ep_send_callback(int size);

/*- Variables ---------------------------------------------------------------*/
static const alignas(4) uint8_t usb_hid_report_descriptor[USB_HID_REPORT_DESCRIPTOR_LENGTH] =
{
  0x05, 0x01,    // USAGE_PAGE (Generic Desktop)
  0x09, 0x02,    // USAGE (Mouse)
  0xa1, 0x01,    // COLLECTION (Application)
  0x09, 0x01,    //   USAGE (Pointer)
  0xa1, 0x00,    //   COLLECTION (Physical)
  0x05, 0x09,    //     USAGE_PAGE (Button)
  0x19, 0x01,    //     USAGE_MINIMUM (Button 1)
  0x29, 0x03,    //     USAGE_MAXIMUM (Button 3)
  0x15, 0x00,    //     LOGICAL_MINIMUM (0)
  0x25, 0x01,    //     LOGICAL_MAXIMUM (1)
  0x95, 0x03,    //     REPORT_COUNT (3)
  0x75, 0x01,    //     REPORT_SIZE (1)
  0x81, 0x02,    //     INPUT (Data,Var,Abs)
  0x95, 0x01,    //     REPORT_COUNT (1)
  0x75, 0x05,    //     REPORT_SIZE (5)
  0x81, 0x03,    //     INPUT (Cnst,Var,Abs)
  0x05, 0x01,    //     USAGE_PAGE (Generic Desktop)
  0x09, 0x30,    //     USAGE (X)
  0x09, 0x31,    //     USAGE (Y)
  0x15, 0x81,    //     LOGICAL_MINIMUM (-127)
  0x25, 0x7f,    //     LOGICAL_MAXIMUM (127)
  0x75, 0x08,    //     REPORT_SIZE (8)
  0x95, 0x02,    //     REPORT_COUNT (2)
  0x81, 0x06,    //     INPUT (Data,Var,Rel)
  0xc0,          //   END_COLLECTION
  0xc0           // END_COLLECTION
};

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
void usb_hid_init(void)
{
  usb_set_callback(USB_HID_EP_SEND, usb_hid_ep_send_callback);
}

//-----------------------------------------------------------------------------
void usb_hid_send(uint8_t *data, int size)
{
  usb_send(USB_HID_EP_SEND, data, size);
}

//-----------------------------------------------------------------------------
static void usb_hid_ep_send_callback(int size)
{
  usb_hid_send_callback();
  (void)size;
}

//-----------------------------------------------------------------------------
bool usb_class_handle_request(usb_request_t *request)
{
  int length = request->wLength;

  switch ((request->bRequest << 8) | request->bmRequestType)
  {
    case USB_CMD(IN, INTERFACE, STANDARD, GET_DESCRIPTOR):
    {
      length = LIMIT(length, sizeof(usb_hid_report_descriptor));

      usb_control_send((uint8_t *)usb_hid_report_descriptor, length);
    } break;

    default:
      return false;
  }

  return true;
}
