USB DFU Bootloader for SAMD11 / SAMD21
======================================

Bootloaders are a dime a dozen, but existing USB bootloaders for the Atmel/Microchip SAMDx1 all seem to be 4kBytes in size.  To spend 25% of the SAMD11's flash on the bootloader seems quite excessive.

This bootloader is only 1kBytes and implements the industry-standard [DFU protocol](http://www.usb.org/developers/docs/devclass_docs/DFU_1.1.pdf) that is supported under multiple Operating Systems via existing tools such as [dfu-util](http://dfu-util.sourceforge.net/).

It is a much more space efficient alternative to the 4kB Atmel/Microchip [AN_42366](http://www.microchip.com//wwwAppNotes/AppNotes.aspx?appnote=en591491) SAM-BA Bootloader.

## Usage

Downloading can be accomplished with the existing [dfu-util](http://dfu-util.sourceforge.net/) utilities.

Downloading a raw binary file looks like this:

```
dfu-util -D write.bin
```

or by using the provided dx1elf2dfu utility, one can create a .dfu file to be downloaded:

```
dfu-util -D write.dfu
```

## Specifics

The linker memory map of the user application must be modified to have an origin at 0x0000_0400 rather than at 0x0000_0000.  This bootloader resides at 0x0000_0000.

When booting, the bootloader checks whether a GPIO pin (nominally PA15) is connected to ground.  If so, it runs the bootloader instead of the user application.

When branching to the user application, the bootloader includes functionality to update the [VTOR (Vector Table Offset Register)](http://infocenter.arm.com/help/topic/com.arm.doc.dui0662a/Ciheijba.html) and update the stack pointer to suit the value in the user application's vector table.

## Requirements for compiling

[Rowley Crossworks for ARM](http://www.rowley.co.uk/arm/) is presently needed to compile this code.  With Crossworks for ARM v4.1.1, using the Clang 5.0.1 compiler produces an 1010 byte image.  The more mainstream GCC does not appear to be optimized enough to produce an image that comes anywhere close to fitting into 1024 bytes.

There is no dependency in the code on the [Rowley Crossworks for ARM](http://www.rowley.co.uk/arm/) toolchain per se, but at this time I am not aware of any other ready-to-use Clang ARM cross-compiler package that I can readily point users to.

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
