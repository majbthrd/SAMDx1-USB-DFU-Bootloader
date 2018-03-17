mouseplay: example HID mouse
============================

## Introduction

This functions like the example app "mouseplay" included with [USB DFU Bootloader for PIC16F1454/5/9](https://github.com/majbthrd/PIC16F1-USB-DFU-Bootloader).

The USB stack source code owes its origins to both [vcp](https://github.com/ataradov/vcp) and [free-dap](https://github.com/ataradov/free-dap).  The  [vcp](https://github.com/ataradov/vcp) USB stack is newer and more sophisticated, but is not a HID implementation like [free-dap](https://github.com/ataradov/free-dap).  Using the two code bases, I made an educated guess at what sort of USB HID implementation the author might have written after written vcp.

The linker memory maps have been adjusted to exclude the first 0x400 bytes so as allow operation with the [USB DFU Bootloader for SAMD11](https://github.com/majbthrd/SAMD11-USB-DFU-Bootloader/).

## Build Requirements

One approach is to use [Rowley Crossworks for ARM](http://www.rowley.co.uk/arm/) to compile this code.  It is not free software, but has been my favorite go-to ARM development tool for a decade and counting.

*OR*

Use the Makefile in the make subdirectory.  With this approach, the code can be built using only open-source software.  In Ubuntu-derived distributions, this is likely achieved with as little as:

```
sudo apt-get install gcc-arm-none-eabi libnewlib-arm-none-eabi build-essential
```

## Customizing

usb_descriptors.c contains the USB VID:PID.  All unique USB device implementations must have their own unique USB VID:PID identifiers.

If modifying usb_hid_report_descriptor in usb_hid.c, the USB_HID_REPORT_DESCRIPTOR_LENGTH define in usb_hid.h *MUST* exactly match the new size of usb_hid_report_descriptor.

As with any USB HID device, the HID report size (nominally sent by send_buffer() in main.c) *MUST* exactly match the metadata conveyed in the usb_hid_report_descriptor.
