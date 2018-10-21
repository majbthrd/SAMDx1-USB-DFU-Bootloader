dx1elf2dfu
==========

This tool is for users of the Dx1bootloader to generate a DFU image directly from a SAMD11/SAMD21 ELF object file.

This DFU image output includes a CRC32 calculation that is stored inside the user application; this is checked by the bootloader to verify the user application's integrity.

## Sample Usage

```
dx1elf2dfu myapp.elf myapp.dfu
```

## Theory of Operation

The Cortex-M0 vector table has nine unused 32-bit entries marked 'Reserved' that only serve as wasted flash space.  By using two of these entries to store the user application length and its CRC32, the dx1elf2dfu utility can communicate this information to the bootloader without consuming additional space.

By storing the user application length, the bootloader will only compute the CRC32 over a prescribed portion of the flash.  This frees the user application, if it wishes, to store and re-write data in upper portions of the flash without impacting the CRC32 protection.
