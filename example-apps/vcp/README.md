vcp: Virtual COM-Port
=====================

## Introduction

This is a lightly-modified version of [vcp](https://github.com/ataradov/vcp).  Said code provides a CDC-ACM USB-to-UART implementation on pins PA4 and PA5 of the SAMD11.

The linker memory maps have been adjusted to exclude the first 0x400 bytes so as allow operation with the [USB DFU Bootloader for SAMD11](https://github.com/majbthrd/SAMD11-USB-DFU-Bootloader/).

## Build Requirements

One approach is to use [Rowley Crossworks for ARM](http://www.rowley.co.uk/arm/) to compile this code.  It is not free software, but has been my favorite go-to ARM development tool for a decade and counting.

*OR*

Use the Makefile in the make subdirectory.  With this approach, the code can be built using only open-source software.  In Ubuntu-derived distributions, this is likely achieved with as little as:

```
sudo apt-get install gcc-arm-none-eabi libnewlib-arm-none-eabi build-essential
```

## Customizing

Refer to the [original source code](https://github.com/ataradov/vcp) for any details.
