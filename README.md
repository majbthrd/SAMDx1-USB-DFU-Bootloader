USB DFU Bootloader for SAMD11 / SAMD21
======================================

Bootloaders are a dime a dozen, but existing USB bootloaders for the Atmel/Microchip SAMDx1 all seem to be 4kBytes in size.  To spent 25% of the SAMD11's flash on the bootloader seems quite excessive.

This bootloader is only 1kBytes and implements the industry-standard [DFU protocol](http://www.usb.org/developers/docs/devclass_docs/DFU_1.1.pdf) that is supported under multiple Operating Systems via existing tools such as [dfu-util](http://dfu-util.sourceforge.net/).

It is a much more space efficient alternative to the 4kB Atmel/Microchip [AN_42366](http://www.microchip.com//wwwAppNotes/AppNotes.aspx?appnote=en591491) SAM-BA Bootloader.

## Usage

Downloading can be accomplished with the existing [dfu-util](http://dfu-util.sourceforge.net/) utilities:

```
dfu-util -D write.bin
```

## Specifics

The linker memory map of the user application must be modified to have an origin at 0x0000_0400 rather than at 0x0000_0000.  This bootloader resides at 0x0000_0000.

When booting, the bootloader checks whether a GPIO pin (nominally PA15) is connected to ground.  If so, it runs the bootloader instead of the user application.

When branching to the user application, the bootloader includes functionality to update the [VTOR (Vector Table Offset Register)](http://infocenter.arm.com/help/topic/com.arm.doc.dui0662a/Ciheijba.html) and update the stack pointer to suit the value in the user application's vector table.

## Requirements for compiling

[Rowley Crossworks for ARM](http://www.rowley.co.uk/arm/) is presently needed to compile this code.  With Crossworks for ARM v4.1.1, using the Clang 5.0.1 compiler produces an 1010 byte image.  The more mainstream GCC does not appear to be optimized enough to produce an image that comes anywhere close to fitting into 1024 bytes.

There is no dependency in the code on the [Rowley Crossworks for ARM](http://www.rowley.co.uk/arm/) toolchain per se, but at this time I am not aware of any other ready-to-use Clang ARM cross-compiler package that I can readily point users to.
