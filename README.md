USB DFU Bootloader for SAMD11 / SAMD21
======================================

Bootloaders may be a dime a dozen, but existing USB bootloaders for the Atmel/Microchip SAMD11/SAMD21 all seem to be 4kBytes or 8kBytes in size.  To spend 25% or 50% of the SAMD11's flash on the bootloader seems quite excessive.  The SAMD21 may have more flash to spare than the SAMD11, but why be so wasteful with it?

This USB bootloader is only 1kBytes and implements the industry-standard [DFU protocol](http://www.usb.org/developers/docs/devclass_docs/DFU_1.1.pdf) that is supported under multiple Operating Systems via existing tools such as [dfu-util](http://dfu-util.sourceforge.net/) and [webdfu](https://github.com/devanlai/webdfu).

It is a much more space efficient alternative to the 4kB Atmel/Microchip [AN_42366](http://www.microchip.com//wwwAppNotes/AppNotes.aspx?appnote=en591491) SAM-BA Bootloader or the positively gluttonous 8kB Arduino Zero bootloaders.

## Features

Despite the small size, it packs a punch.  Unlike other bootloaders whose integrity check consists of merely sampling the first few bytes to see if they are not erased, this bootloader performs a proper CRC32 check of the application before letting it boot.  It also supports the latest trend of detecting a double-tap of RESET to manually invoke the bootloader.

## Usage

Downloading can be accomplished with any software that supports DFU, which includes [dfu-util](http://dfu-util.sourceforge.net/) and [webdfu](https://github.com/devanlai/webdfu).

Using the provided dx1elf2dfu utility, one can create a .dfu file.  Your DFU software of choice will accept that file.

With [dfu-util](http://dfu-util.sourceforge.net/), downloading is like so:

```
dfu-util -D write.dfu
```

With [webdfu](https://devanlai.github.io/webdfu/dfu-util/), select the Vendor ID as 0x1209 and click Connect.  Verify the transfer size is 64, and choose the .dfu file.  Click Download, and after the download, click Disconnect.

## Specifics

Source code for some example apps is provided in the 'example-apps' subdirectory.

The linker memory map of the user application must be modified to have an origin at 0x0000_0400 rather than at 0x0000_0000.  This bootloader resides at 0x0000_0000.

v1.04 features support for detecting a double-tap of a RESET button.  For earlier versions (or by re-enabling legacy behavior in the source code), the bootloader checks at boot whether a GPIO pin (nominally PA15) is connected to ground.  v1.02+ also computes a CRC32 of the user application.  If the user application is unprogrammed or corrupted, the CRC32 check should fail.  If the CRC32 check fails or a user request is detected, it runs the bootloader instead of the user application.

When branching to the user application, the bootloader includes functionality to update the [VTOR (Vector Table Offset Register)](http://infocenter.arm.com/help/topic/com.arm.doc.dui0662a/Ciheijba.html) and update the stack pointer to suit the value in the user application's vector table.


## Arduino Zero Boards with Problems

This bootloader normally uses the USBCRM mode of the SAMD11/SAMD21.  In this mode, the part disciplines its own 48MHz RC clock using the USB SOF messages.  The advantage of this is simple: it doesn't require optional external components.
 
Until buying a PCB from Sparkfun, I have had zero issues with USBCRM across several hardware designs.  However, the Sparkfun design (derived from Arduino Zero) uses a 32k external crystal and USB will NOT work reliably on it unless this optional crystal is used as the timing source.  Furthermore, the Arduino Zero bootloader source code depends on a 32k external crystal without any explanation as to why.  There is no verbiage in the datasheet to indicate special requirements for USBCRM.  AT07175 (the SAM-BA app note that served as a basis for the "Arduino Zero" bootloader) makes no special requirements.  There are no errata notes on possible scenarios where USBCRM should not function.

The USBCRM mode should be universal, as it doesn't depend on optional external components.  For that reason, the source code uses this mode by default.  However, the source code now has an "#if 1" that can be changed to "#if 0" to cause the optional external 32k crystal (if populated on your design) to be used instead of USBCRM.

## Requirements for compiling

Pre-compiled images are already available via this project's Releases tab.

[Rowley Crossworks for ARM](http://www.rowley.co.uk/arm/) is presently suggested to compile this code, as it includes support for Clang; at this time, I am not aware of any other ready-to-use (and multi-OS to boot) Clang ARM cross-compiler package that I can readily point users to.  With Crossworks for ARM v4.8.1, compiling v1.06 using the Clang 11.1.0 compiler produces a 995 byte image.  The more mainstream GCC lags behind Clang, although more recent GCC versions produce code that is less overweight than in years past.

|bootloader variant|Clang 9.0.1 (-O1) |Clang 11.1.0 (-O1) |GNU Arm 2018-q3 (-Os) |GNU Arm 2019-q4 (-Os) |
|------------------|------------------|-------------------|----------------------|----------------------|
| USE_DBL_TAP      | 1003 bytes       | 995 bytes         | 1044 bytes (too big!)| 1041 bytes (too big!)|
| GPIO input       | 979 bytes        | 975 bytes         | 1008 bytes           | 1006 bytes           |

A Makefile supporting GCC is provided, but even if you have the latest GCC, it may not be able to output a version within the 1024 bytes available.  The latest GCC for ARM can be here: [GNU Arm Embedded Toolchain](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm).  Note that if you are adapting the Makefile for use with clang, try replacing the "-Os" argument in CFLAGS with something like "-O1" if the output size is larger than expected.

## Programming targets with bootloader

[OpenOCD](http://openocd.org/) is open-source and freely available.  Given a debug unit interfaced to the target, it will not only program the flash but it also has built-in support for setting the BOOTPROT bits to provide write-protection of the bootloader.

Testing was done with OpenOCD v0.10.0 using both the debug units built-in to the ATSAMD11-XPRO and ATSAMD21-XPRO as well as the [Dapper Miser CMSIS-DAP](https://github.com/majbthrd/DapperMiser) debug unit.

Your mileage may vary, particularly with earlier releases of OpenOCD.  (The wisdom of the Internet implies earlier versions of OpenOCD had bugs with setting the BOOTPROT bits.)  You don't have to use the BOOTPROT bits, but adding write-protection to the bootloader is generally a desirable attribute.

####Step 1: disable existing BOOTPROT write-protection, if any

```
openocd -f interface/cmsis-dap.cfg -f target/at91samdXX.cfg -c "init; halt; at91samd bootloader 0; exit"
```

####Step 2: program the bootloader (the file Dx1bootloader-v1_01-PA15.srec in this example)

```
openocd -f interface/cmsis-dap.cfg -f target/at91samdXX.cfg -c "program Dx1bootloader-v1_01-PA15.srec verify; exit"
```

####Step 3: enable the BOOTPROT bits for write-protecting a 1kB bootloader

```
openocd -f interface/cmsis-dap.cfg -f target/at91samdXX.cfg -c "init; halt; at91samd bootloader 1024; exit"
```

Once the bootloader is programmed, the user should be able to dispense with having to repeatedly connect the target to a debug unit and use OpenOCD (or equivalent) to program updates.  New user firmware can be downloaded over the target's USB connection as shown in the Usage section above.
