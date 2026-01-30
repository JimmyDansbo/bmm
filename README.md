# Banked Memory Manager
Banked Memory Manager for Commander X16  
Version 0.8  
*Author: Jimmy Dansbo*

**Note that this documentation can change at any time.**

**Table of Contents**

* [Overview](#overview)
* [Memory Bank Header](#memory-bank-header)
* [Memory Area Header](#memory-area-header)
* [Using in your project](#using-in-your-project)

## Overview
## Memory Bank Header

Any memory bank, except bank 0, can be initialized for use with the library.  
When a bank is initialized, the first 36 bytes, starting at $A000 are claimed for the memory bank header.

The header has the following layout.
| Address | Size | Description|
|---------|------|------------|
| $A000   | 2 bytes | Next available memory address |
| $A002   | 2 bytes | First allocatable address |
| $A004   | 32 bytes | Handle bitmap |

This means that the library can use the remaining available space in an otherwise used memory bank, provided the first 36 bytes are free to initialize the bank.
## Memory Area Header

When a memory area is allocated, it takes up 4 bytes more than the requested space.  
These 4 bytes are used for the memory area header:
| Offset | Size | Description |
|--------|------|-------------|
|   $00  | 2 bytes | Address to next memory element |
|  $02 | 1 byte | Handle ID in current bank |
| $03 | 1 byte | Checksum of header |
## Using in your project
## Functions
### Function name: mm_init
Purpose: Initialize Banked Memory Manager  
Communication registers: A, Y & X

**Description** Initialize the banked memory manager library, copy functions to low-ram and initialize the first memory bank for usage by the library.
| Registers | Purpose |
|-----------|---------|
|    A     | First ZeroPage address to use as pointer |
|    Y     | Second ZeroPage address to use as pointer |
|    X     | Memory bank to initialize |

