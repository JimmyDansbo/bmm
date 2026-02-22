# Banked Memory Manager
Banked Memory Manager for Commander X16  
Version 0.12  
*Author: Jimmy Dansbo*

**Note that this documentation can change at any time.**

**Table of Contents**

* [Overview](#overview)
* [Memory Bank Header](#memory-bank-header)
* [Memory Area Header](#memory-area-header)
* [Memory Handle](#memory-handle)
* [Using in your project](#using-in-your-project)
* [Error codes](#error-codes)
* [Functions](#functions)  
	* [mm_init](#function-name-mm_init)  
	* [mm_init_bank](#function-name-mm_init_bank)  
	* [mm_remaining](#function-name-mm_remaining)  
	* [mm_alloc](#function-name-mm_alloc)  
	* [mm_get_ptr](#function-name-mm_get_ptr)  
	* [mm_free](#function-name-mm_free)  
	* [mm_defrag](#function-name-mm_defrag)  
	* [mm_get_size](#function-name-mm_get_size)
	* [mm_set_isr](#function-name-mm_set_isr)  
	* [mm_clear_isr](#function-name-mm_clear_isr)
* [Zero Page](#zero-page)  
	* [mm_update_zp](#function-name-mm_update_zp)  
	* [mm_read_zp1](#function-name-mm_read_zp1)  
	* [mm_read_zp1l](#function-name-mm_read_zp1l)  
	* [mm_read_zp1h](#function-name-mm_read_zp1h)  
	* [mm_read_zp2](#function-name-mm_read_zp2)  
	* [mm_read_zp2l](#function-name-mm_read_zp2l)  
	* [mm_read_zp2h](#function-name-mm_read_zp2h)  
	* [mm_store_zp1](#function-name-mm_store_zp1)  
	* [mm_store_zp1l](#function-name-mm_store_zp1l)  
	* [mm_store_zp1h](#function-name-mm_store_zp1h)  
	* [mm_store_zp2](#function-name-mm_store_zp2)  
	* [mm_store_zp2l](#function-name-mm_store_zp2l)  
	* [mm_store_zp2h](#function-name-mm_store_zp2h)  
* [Lowram Functions](#lowram-functions)  
	* [mm_lda_bank](#function-name-mm_lda_bank)  
	* [mm_lday_bank](#function-name-mm_lday_bank)  
	* [mm_ldayx_bank](#function-name-mm_ldayx_bank)  
	* [mm_sta_bank](#function-name-mm_sta_bank)  
	* [mm_stay_bank](#function-name-mm_stay_bank)  
	* [mm_bank_copy](#function-name-mm_bank_copy)

## Overview

The Banked Memory Manager library is designed to ease using banked memory in programs and libraries designed for the Commander X16.  
The library is designed to handle banked memory even if it is loaded into a memory bank itself. It is able to allocate memory in any RAM bank, except bank 0 and it will keep track of remaining memory in a bank as well as pointers to allocated memory areas.  
The library utilizes handles to let the user easily free an allocated memory area again.  
**NOTE:** When a memory area is freed, it can either be done by marking the freed memory as dirty or by defragmenting the remaining memory. When defragmentation is chosen, it is important to get new pointers with the `mm_get_ptr` function as addresses may have changed.  
The `mm_get_ptr` function will always return the correct address of a memory area identified by handle.   
The library also makes several bank safe functions availabe to load, store and copy inside banked memory.

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

When a memory area is allocated, it takes up 4 bytes more than the requested size.  
These 4 bytes are used for the memory area header:
| Offset | Size | Description |
|--------|------|-------------|
|   $00  | 2 bytes | Address to next memory element |
|  $02 | 1 byte | Handle ID in current bank |
| $03 | 1 byte | Checksum of header |  

If bit 6 is set in high-byte of the address to next element, it means that the current area is marked as dirty.

## Memory Handle

The memory handle is the way the library identifies allocated memory areas. It is a 16 bit value that is always passed in registers A and Y.  
When using the handle with the library, the low-byte of the handle is always passed in register A and the high-byte in register Y.  
The low-byte of the handle is the RAM bank in which the memory is allocated. The high-byte is the ID within that RAM bank.  
This means that it is not possible to allocate more than 256 memory areas in a RAM bank which in turn means that if the average size of allocated memory is less than 28 bytes, the library will run out of handles before an empty RAM bank is filled.  

## Using in your project

The library is designed to be used in CC65 assembler projects. To include the library in your own project, you need `memman.inc` & `memman.o`

When linking your project with the banked memory manager library, you need a custom cc65 configuration file. The configuration file must contain definition of two segments `MEMMAN` & `MMLOWRAM` like this:
```
SEGMENTS {
	...
	MEMMAN:	  load = HIRAM, type = ro;
	MMLOWRAM: load = HIRAM, type = ro, define = yes;
	...
}
```
The MMLOWRAM segment must have the define=yes option to ensure the linker can check the size of the segment.

In order to use the functions and constants in the library, you should include the `memman.inc` file in your own source files.

When linking your project you simply link it against the `memman.o` file as well.
```
ld65 -C yourproject.cfg memman.o yourproject.asm -o yourproject.prg
``` 

## Error Codes
| Code | Name | Description |
|------|------|-------------|
| $01 | MM_ERR_ZERO | Zero bytes allocation requested |
| $02 | MM_ERR_NOSPACE | Not enough available memory in bank |
| $03 | MM_ERR_NOHANDLE | No available handles in current bank |
| $04 | MM_ERR_HANDLE_NOTFOUND | Requested handle was not found |
| $05 | MM_ERR_CORRUPT_HDR | Memory area header has been found corrupted |

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

The two ZeroPage pointers will be used by the library to allocate memory and copy memory within banks.

Before calling `mm_init` the first ZeroPage pointer (zp1) must containt the address to 256 bytes of lowram that will be used by the library.

The second ZeroPage pointer (zp2) must contain the first available address in the current RAM bank. If it is an empty RAM bank, the address should be set to $A000.

```ASM
	; Use golden memory $0400 as lowram
	lda #<$0400
	sta $30		; Store low-byte in zp1-low
	lda #>$0400
	sta $31		; Store high-byte in zp1-high
	; Tell the library that it is an empty bank
	lda #<$A000
	sta $32		; Store low-byte in zp2-low
	lda #>$A000
	sta $33		; Store high-byte in zp2-high
	lda #$30	; Use $30,$31 as ZP1
	ldy #$32	; Use $32,$33 as ZP2
	ldx #$02	; Initialize RAM bank 2
	jsr mm_init 
```
### Function name: mm_init_bank
Purpose: Initialize a memory bank for use by the library.  
Communication registers: A, Y & X  
Preserves: X

**Description** Initialize a memory bank by setting the 4 first bytes of the bank to the first available memory address, followed by 32 bytes for bank handle bitmap. See [Memory Bank Header](#memory-bank-header)
| Registers | Purpose |
|-----------|---------|
| A | Low-byte of next free address |
| Y | High-byte of next free address |
| X | RAM bank to initialize |

### Function name: mm_remaining
Purpose: Return the amount of memory available in a specified memory bank  
Communication registers: A, Y, X  
Preserves: X

**Description** Calculates and returns the amount of free memory in a specified bank. This does not include memory areas that are marked as dirty. In order to ensure an accurate amount is returned, the memory should be defragmented first.

| Input | Purpose |
|-------|---------|
| X | Bank to check free space for |  

| Output | Description |
|--------|-------------|
| A | low-byte of available memory |
| Y | high-byte of available memory

### Function name: mm_alloc
Purpose: Allocate a memory area    
Communication registers: A, Y, X & C  
Preserves: X

**Description** Allocates the specified amount of memory in a bank.  
If a dirty memory area is found to be large enough to accommodate the requested size, it will be re-allocated and the amount of available memory will stay unchanged.
| Input | Purpose |
|-------|---------|
| A | low-byte of bytes to allocate |
| Y | high-byte of bytes to allocate |
| X | RAM bank to allocate memory in

| Output | Description |
|--------|-------------|
| C | Carry set on error |
| A | low-byte of handle or error code on error |
| Y | high-byte of handle |

### Function name: mm_get_ptr
Purpose: Return address of memory area identified by handle  
Communication registers: A, Y, X & C

**Description** Returns the actual address of a memory area identified by handle.  
This function should be called to ensure correct address of a memory area if memory defragmentation has been performed.  
Only the handle of an allocated memory area is guaranteed to stay the same.
| Input | Purpose |
|-------|---------|
| A | low-byte of handle |
| Y | high-byte of handle |

This function can be called in a different way to look for dirty memory areas instead of handles. This is meant to be used internally for defragmenting the memory.
| Input | Purpose |
|-------|---------|
| A | Set to 0 to look for dirty memory |
| X | RAM bank |
| Y | 0 if searching from beginning of RAM bank |

In any case, the output of the function is the same.

| Output | Description |
|--------|-------------|
| C | Carry set on error |
| A | low-byte of address or error code on error |
| Y | high-byte of address |
| X | RAM bank |

### Function name: mm_free
Purpose: Free a memory area  
Communication registers: A, Y & C

**Description** Free a previously allocated memory area.  
The function can defragment the memory right away or simply mark it as dirty.  
Dirty memory may be re-used if it is large enough to contain a new request.  
A request to allocate memory will use the first dirty memory area large enough to hold it, even if it means using a memory area much larger than the requested size.  
When memory is defragmented, it means that allocated memory get's new pointers and it is therefore very important to get the correct pointers through a call to `mm_get_ptr`.
| Input | Purpose |
|-------|---------|
| A | low-byte of handle |
| Y | high-byte of handle |
| C | clear=defrag memory, set=mark only

| Output | Description |
|--------|-------------|
| C | Carry set on error |
| A | Error code on error |

### Function name: mm_defrag
Purpose: Defragment a memory bank  
Communication registers: A, X & C

**Description** Defragment a memory bank, freeing up previously allocated memory and in the process moving still used memory areas.  
**Note:** After defragmentation, it is important to call `mm_get_ptr` to ensure your code has the correct memory addresses.
| Input | Purpose |
|-------|---------|
| X | RAM bank to perform defragmentation on |

| Output | Description |
|--------|-------------|
| C | Set on error |
| A | Error code on error |

### Function name: mm_get_size
Purpose: Get the size of a memory area identified by handle  
Communications registers: A, Y, X & C

**Description** Get the actual size of an allocated memory area, minus the 4 bytes for header.  
This can be used to check if an allocated memory area takes up more space than was actually requested. This could happen if a dirty memory area has been re-allocated.
| Input | Purpose |
|-------|---------|
| A | Low-byte of handle |
| Y | High-byte of handle |

| Output | Description |
|--------|-------------|
| C | Set on error |
| A | Low-byte of size or errorcode if C set |
| Y | High-byte of size |

### Function name: mm_set_isr
Purpose: Install banked ISR  
Communication registers: A, Y & X  
Preserves: X

**Description** Installs an interrupt service routine located in banked RAM. When the banked ISR does a normal `rts`, execution will continue to the default interrupt handler
| Input | Purpose |
|-------|---------|
| A | low-byte of address for banked ISR |
| Y | high-byte of address for banked ISR |
| X | RAM bank of the banked ISR |

### Function name: mm_clear_isr
Purpose: Restore previous ISR  
Communication registers: none  
Uses: A

**Description** Remove the banked interrupt service routine and restore the original interrupt handler.
## Zero Page
A few functions are exported that are only used to manipulate the Zero Page addresses used by the library.

### Function name: mm_update_zp
Purpose: Update the ZeroPage pointers used by the library  
Communication registers: A & Y

**Description** Change the ZeroPage address used by the library. This function is part of the library initialization, but can be called seperately if the user needs to change the ZeroPage addresses used by the library.
| Registers | Purpose |
|-----------|---------|
| A | First ZeroPage address to use for pointer (zp1) |
| Y | Second ZeroPage address to use for pointer (zp2) |

### Function name: mm_read_zp1
Purpose: Read the value/address currently stored in ZP1  
Communication registers: A & Y  

**Description** Returns the values stored in the first Zero Page Pointer that was provided to the library.
| Output | Description |
|------- |-------------|
| A | low-byte of ZP1 |
| Y | high-byte of ZP1 |

### Function name: mm_read_zp1l
Purpose: Read the low-byte of ZP1  
Communication registers: A  

**Description** Returns the low-byte stored in the first Zero Page pointer that was provided to the library
| Output | Description |
|--------|-------------|
| A | low-byte of ZP1 |

### Function name: mm_read_zp1h
Purpose: Read the high-byte of ZP1  
Communication registers: A

**Description** Returns the high-byte stored in the first Zero Page pointer that was provided to the library
| Output | Description |
|--------|-------------|
| A | high-byte of ZP1 |

### Function name: mm_read_zp2
Purpose: Read the value/address currently stored in ZP2  
Communication registers: A & Y  

**Description** Returns the values stored in the second Zero Page Pointer that was provided to the library.
| Output | Description |
|------- |-------------|
| A | low-byte of ZP2 |
| Y | high-byte of ZP2 |

### Function name: mm_read_zp2l
Purpose: Read the low-byte of ZP2  
Communication registers: A  

**Description** Returns the low-byte stored in the second Zero Page pointer that was provided to the library  
| Output | Description |
|--------|-------------|
| A | low-byte of ZP2 |

### Function name: mm_read_zp2h
Purpose: Read the high-byte of ZP2  
Communication registers: A

**Description** Returns the high-byte stored in the second Zero Page pointer that was provided to the library
| Output | Description |
|--------|-------------|
| A | high-byte of ZP2 |

### Function name: mm_store_zp1
Purpose: Store a new value/address in ZP1  
Communication registers: A & Y  

**Description** Update the first Zero Page pointer (zp1) with a new value/address.
| Input | Purpose |
|------- |-------------|
| A | low-byte of new value/address |
| Y | high-byte of new value/address |

### Function name: mm_store_zp1l
Purpose: Store new low-byte in ZP1  
Communication registers: A

**Description** Update the low-byte of the first Zero Page pointer with a new value
| Input | Purpose |
|------- |-------------|
| A | low-byte of new value |

### Function name: mm_store_zp1h
Purpose: Store new high-byte in ZP1
Communication registers: A

**Description** Update the high-byte of the first Zero Page pointer with a new value
| Input | Purpose |
|------- |-------------|
| A | high-byte of new value |

### Function name: mm_store_zp2
Purpose: Store a new value/address in ZP2  
Communication registers: A & Y  

**Description** Update the second Zero Page pointer (zp2) with a new value/address.
| Input | Purpose |
|------- |-------------|
| A | low-byte of new value/address |
| Y | high-byte of new value/address |

### Function name: mm_store_zp2l
Purpose: Store new low-byte in ZP2  
Communication registers: A

**Description** Update the low-byte of the second Zero Page pointer with a new value
| Input | Purpose |
|------- |-------------|
| A | low-byte of new value |

### Function name: mm_store_zp2h
Purpose: Store new high-byte in ZP2
Communication registers: A

**Description** Update the high-byte of the second Zero Page pointer with a new value
| Input | Purpose |
|------- |-------------|
| A | high-byte of new value |

## Lowram functions
There are several lowram functions made available by the library. They are located in the memory area that was made available to the library at initialization.

To call a lowram function, the function offset is simply added to the start address of the lowram area.

```
	jsr	lowram+MM_LDA_BANK_OFS
```

The functions are also exported by the library so if your code is running in the same bank as the library, you can simply call the functions directly.  

### Function name: mm_lda_bank
Purpose: lda from banked address  
Communication registers: A, X & Y  
Depends: zp1 pointer  
Preserves: X, Y and RAM bank  
Offset: $12  
Offset constant: `MM_LDA_BANK_OFFS`

**Description** Reads a single byte from banked memory specified by first ZeroPage pointer into register A
| Inputs | Purpose |
|--------|---------|
| zp1 | ZeroPage pointer to address to read from |
| X | RAM bank to read from |
| Y | Offset from pointer to read from |

| Output | Description |
|--------|-------------|
| A | Value read from banked memory |
| Flags | Set according to A

### Function name: mm_lday_bank
Purpose: lda & ldy from banked address  
Communication registers: A, Y & X  
Depends: zp1 pointer  
Preserves: X & RAM bank before call  
Offset: $21  
Offset constant: `MM_LDAY_BANK_OFFS`

**Description** Reads two bytes from banked memory specified by first ZeroPage pointer into registers A & Y
| Inputs | Purpose |
|--------|---------|
| zp1 | ZeroPage pointer to address to read from |
| X | RAM bank to read from |
| Y | Offset from pointer to read from |

| Output | Description |
|--------|-------------|
| A | low-byte of value read from banked memory |
| Y | high-byte of value read from banked memory |

### Function name: mm_ldayx_bank
Purpose: lda, ldy & ldx from banked address  
Communication registers: A, Y & X  
Depends: zp1 pointer  
Preserves: RAM bank before call  
Offset: $33  
Offset constant: `MM_LDAYX_BANK_OFFS`

**Description** Reads three bytes from banked memory specified by first ZeroPage pointer into registers Y, X & A
| Inputs | Purpose |
|--------|---------|
| zp1 | ZeroPage pointer to address to read from |
| X | RAM bank to read from |
| Y | Offset from pointer to read from |

| Output | Description |
|--------|-------------|
| A | low-byte of value read from banked memory |
| Y | mid-byte of value read from banked memory |
| X | high-byte of value read from banked memory |

### Function name: mm_sta_bank
Purpose: sta to banked address  
Communication registers: A, X, Y
Depends: zp1 pointer
Preserves: A, X, Y and the RAM bank before call  
Offset: $48  
Offset constant: `MM_STA_BANK_OFFS`

**Description** Store a value from register A to banked memory pointed to by first ZeroPage pointer.
| Inputs | Purpose |
|--------|---------|
| zp1 | ZeroPage pointer to address to write to |
| X | RAM bank to write to |
| Y | Offset from pointer to write to |
| A | Value to write |

### Function name: mm_stay_bank
Purpose: sta & sty to banked address  
Communication registers: A, X & Y  
Depends: zp1 pointer  
Preserves: A, X, Y & RAM bank before call
Offset: $56  
Offset constant: `MM_STAY_BANK_OFFS`

**Description** Store two bytes to banked memory specified by first ZeroPage pointer from registers A & Y
| Inputs | Purpose |
|--------|---------|
| zp1 | ZeroPage pointer to address to write to |
| X | RAM bank to write to |
| A | low-byte to write to banked address |
| Y | high-byte to write to banked address |

### Function name: mm_bank_copy
Purpose: Copy memory within banks  
Communication registers: A & X  
Depends: zp1, zp2 pointers and 2 first bytes of lowram area  
Preserves: X & RAM bank
Offset: $6E  
Offset constant: `MM_BANK_COPY_OFFS`

**Description** Copies memory to- and from any RAM bank and conventional RAM. The funciton is NOT aware of IO ports such as VERA data ports.  
The two first bytes of the lowram area provided to the library must be filled with the amount of bytes that needs to be copied prior to calling this function.  
If the source- and destination RAM bank are equal, the RAM bank will be set once before copying data. This enable to function to copy within the same RAM bank or to copy to- and from conventional memory and banked memory.
| Inputs | Purpose |
|--------|---------|
| zp1 | Fist ZeroPage Pointer is source address |
| zp2 | Second ZeroPage Pointer is destination address |
| lowram | The two first bytes of lowram is the number of bytes to copy |
| A | Source RAM bank |
| X | Destination RAM bank |
