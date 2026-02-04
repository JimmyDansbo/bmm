.include "x16.inc"
SKIPIMPORT=1
.include "memman.inc"

MEMMAN_VERSION	= $000A

.import __MMLOWRAM_SIZE__
.export mm_init, mm_set_isr, mm_clear_isr, mm_alloc, mm_remaining, mm_free, mm_init_bank
.export mm_update_zp, mm_get_ptr, mm_defrag, mm_get_size

.segment "MEMMAN"
lowram_addr:	.res	2
orig_isr:	.res	2
scratch:	.res	6

FREE_ADDR	= $A000
FIRST_ITEM	= $A002
ID_BITMAP	= $A004
FREE_ADDR_OFS	= $00
FIRST_ITEM_OFS	= $02

MEM_HANDLE_OFS	= $02
MEM_CRC_OFS	= $03

MEM_HDR_SIZE	= 4
BANK_HDR_SIZE	= 36
BITMAP_SIZE	= BANK_HDR_SIZE-4	; 4 bytes for addresses

; Offsets from low-ram start address
_isr_bank	= 6
_isr_addr	= 10
_isr_orig	= 16

; Internal jump table into lowram functions
; The bank-load and store functions use first ZP pointer for the address and X for bank
lda_bank:		; Return byte in A					X=bank, Y=offset
	jmp	$0000
lday_bank:		; Return lowbyte in A, highbyte in Y			X=bank
	jmp	$0000
ldyxa_bank:		; Return lowbyte in Y, midbyte in X , highbyte in A	X=bank
	jmp	$0000
sta_bank:		; Store value in A					A=val, X=bank, Y=offset
	jmp	$0000
stay_bank:		; Store A to lowbyte, Y to highbyte			X=bank
	jmp	$0000
; Bank copy function uses two ZP pointers
bank_cpy:
	jmp	$0000

;*****************************************************************************
; Get the size of an allocated memory area
;=============================================================================
; Inputs:	.A & .Y = handle_id (bank/cnt)
; Outputs:	.C set on error
;		.A = low-byte of size or errorcode if .C set
;		.Y = high-byte of size
;-----------------------------------------------------------------------------
; Uses:		.A, .Y, .X, scratch+0 & scratch+1
;*****************************************************************************
.proc mm_get_size: near
	jsr	mm_get_ptr	; Find actual address of handle_id
	bcc	:+
	rts
:	sta	scratch+0
	sty	scratch+1
	jsr	lday_bank	; Read address of next element
	; Subtract current element address from next element address to get size
	sec
	sbc	scratch+0
	pha
	tya
	sbc	scratch+1
	tay
	pla
	rts
.endproc

;*****************************************************************************
; Defragment memory by moving memory down over previously dirty areas
;=============================================================================
; Inputs:	.X = RAM bank to defragment
; Outputs:	.C set on error with errorcode in .A
;-----------------------------------------------------------------------------
; Uses:		.A, .Y, .X, both ZP pointers & scratch areas
;*****************************************************************************
.proc mm_defrag: near
	lda	#0
	ldy	#1
	jsr	mm_get_ptr
	bcs	done
	jsr	readzp1		; Get real pointer from zp1
	sta	scratch+0
	sty	scratch+1
	jsr	updzp2		; Destination pointer set
	jsr	lday_bank	; Get starting address of rest of memory
	sta	scratch+2	; Store it temporarily it is going to
	tya			; be the source address
	and	#$BF		; Reset bit6 of high-byte
	sta	scratch+3
	; Calculate the size of the remaining memory
	jsr	calc_remaining_size
	pha
	phy
	; Calculate the size of memory freed. Store in scratch+0 & scratch+1
	jsr	calc_freed_size
	; Calculate new free address by subtracting size from free address
	jsr	store_new_free_addr
	; Set the low-ram scratch area to the number of bytes that needs to be copied.
	lda	lowram_addr
	ldy	lowram_addr+1
	jsr	updzp1
	; Pull size from stack
	ply
	pla
	jsr	stay_bank	; Store size to low-ram
	sty	scratch+5
	; Check if low- and high-byte are 0 (last element)
	ora	scratch+5
	beq	last_element
	; Update source pointer
	lda	scratch+2
	ldy	scratch+3
	jsr	updzp1
	txa			; Set source- & destination ram bank to the same
	jsr	bank_cpy
last_element:
	; Write 0s to last address to show it is the last
	jsr	zero_last_free
	; Update headers of copied memory blocks by subtracting the freed memory
	; stored in scratc+0 & scratch+1
	jsr	readzp2		; address of first memory object that has been moved
	jsr	update_mem_headers
	bra	mm_defrag
done:	cmp	#MM_ERR_HANDLE_NOTFOUND
	beq	end
	sec
	rts
end:	clc
	rts
.endproc

;*****************************************************************************
; Free the block of memory with handle_id and move any following used memory
; or just mark the memory as dirty
;=============================================================================
; Inputs:	.A & .Y = handle_id (bank/cnt)
;		.C = clear=defrag, set=mark unused
; Outputs:	.C set on error with errorcode in .A
;-----------------------------------------------------------------------------
; Uses:		.A, .Y, .X, both ZP pointers & scratch areas
;*****************************************************************************
.proc mm_free: near
	phy			; Save cnt-part of handle_id
	php			; Save status flags - need to check Carry later
	; Find pointer for handle_id or exit with error
	jsr	mm_get_ptr
	bcc	:+
	ply			; Cleanup stack
	sec
	rts
	; Get the real pointer from zp1
:	jsr	readzp1
	sta	scratch+0
	sty	scratch+1
	jsr	updzp2		; Destination pointer updated
	plp
	; If Carry set, we do NOT defragment memory, we just mark it as dirty
	bcc	:+
	ldy	#1
	jsr	lda_bank	; Mark the area as dirty by setting bit6 of high-byte
	ora	#%01000000
	jsr	sta_bank
	jsr	update_checksum
	jmp	handle
	; Save the starting address of rest of memory
:	jsr	lday_bank
	sta	scratch+2	; This is going to be the from-ptr
	sty	scratch+3
	; Calculate the size of the remaining memory
	jsr	calc_remaining_size
	pha
	phy
	; Calculate the size of memory freed. Store in scratch+0 & scratch+1
	jsr	calc_freed_size
	; Calculate new free address by subtracting size from free address
	jsr	store_new_free_addr
	; Set the low-ram scratch area to the number of bytes that needs to be copied.
	lda	lowram_addr
	ldy	lowram_addr+1
	jsr	updzp1
	; Pull size from stack
	ply
	pla
	jsr	stay_bank	; Store size to low-ram
	sty	scratch+5
	; Check if low- and high-byte are 0 (last element)
	ora	scratch+5
	beq	end
	; Update source pointer
	lda	scratch+2
	ldy	scratch+3
	jsr	updzp1
	txa			; Set source- & destination ram bank to the same
	jsr	bank_cpy
end:	; Write 0s to last address to show it is the last
	jsr	zero_last_free
	; Update headers of copied memory blocks by subtracting the freed memory
	; stored in scratc+0 & scratch+1
	jsr	readzp2		; address of first memory object that has been moved
	jsr	update_mem_headers
handle:	pla			; Get cnt-part of handle_id
	jmp	free_handle
.endproc

;*****************************************************************************
; Return an actual memory address to the area assigned to a handle id
;=============================================================================
; Inputs:	.A & .Y = handle_id (bank/cnt)
;		  if .A=0 look for dirty memory instead of handle,
;		  in that case, .X must contain the bank and .Y is set to
;		  non-zero if searching from start of RAM, otherwise it will
;		  continue search from the address in zp1. 
; Outputs:	.A & .Y = Memory address, Carry clear on success
;		.X = Bank
;		.C set on error with errorcode in .A
;-----------------------------------------------------------------------------
; Preserves:	nothing
;*****************************************************************************
.proc mm_get_ptr: near
	pha			; Save bank part of handle
	cmp	#0
	bne	normal
	cpy	#0
	beq	checknext
	bra	dirty
normal:	tax			; .X = Bank
	sty	scratch+0
dirty:	lda	#<FIRST_ITEM
	ldy	#>FIRST_ITEM
	jsr	updzp1
	; Read first memory block address
checknext:
	jsr	lday_bank
	; Check if high-byte of address is zero
	cpy	#0
	bne	:+
	pla			; Clear stack of bank-part of handle
	lda	#MM_ERR_HANDLE_NOTFOUND
	sec
	rts
	; If it is not zero, we can check the memory block for handle id
:	sta	scratch+2
	sty	scratch+3
	; Check if current handle id matches
loop:	lda	scratch+3	; Ensure bit 6 of high-byte is 0 for address
	and	#$BF
	tay
	lda	scratch+2
	jsr	updzp1	; Set ZP to point to memory block
	; Read next address
	jsr	lday_bank
	; Check if the high-byte of address is zero
	cpy	#0
	bne	:+
	pla			; clear stack of bank-part of handle
	lda	#MM_ERR_HANDLE_NOTFOUND
	sec
	rts
	; Store next address for loop iteration
:	sta	scratch+2
	sty	scratch+3
	jsr	check_header	; Uses scratch+5
	bcc	:+
	pla			; clear stack of bank-part of handle
	lda	#MM_ERR_CORRUPT_HDR
	rts
:	pla			; Get bank-part of handle
	pha			; Save it on stack again
	; If bank-part of handle is 0, we check if address is marked
	bne	:+
	lda	scratch+3
	and	#%01000000
	bne	end
	bra	loop
:	lda	scratch+3
	and	#%01000000
	bne	loop
	ldy	#MEM_HANDLE_OFS	; Read handle_id
	jsr	lda_bank
	; compare to the one specified in the call
	cmp	scratch
	beq	end		; If equal, return address otherwise check next
	bra	loop
end:	pla			; clear bank-part of handle from stack
	jsr	readzp1		; Read current address and add MEM_HDR_SIZE
	clc
	adc	#MEM_HDR_SIZE
	pha
	tya
	adc	#$00
	tay
	pla
	clc
	rts
.endproc

;*****************************************************************************
; Return the number of available bytes in specified bank
;=============================================================================
; Inputs:	.X = bank
; Outputs:	.A & .Y = low-byte & high-byte of available memory
;-----------------------------------------------------------------------------
; Preserves:	.X
;*****************************************************************************
.proc mm_remaining: near
	phx			; Preserve .X, scratch+0 & scratch+1
	lda	scratch+0
	pha
	lda	scratch+1
	pha
	lda	#<FREE_ADDR	; Set address $A000 in ZP pointer
	ldy	#>FREE_ADDR
	jsr	updzp1
	jsr	lday_bank	; Read next free address from bank
	sta	scratch+0
	sty	scratch+1
	; Subtract free address from $C000
	lda	#<X16_RAM_WindowEnd	; low-byte
	sec
	sbc	scratch+0
	pha
	lda	#>X16_RAM_WindowEnd	; high-byte
	sbc	scratch+1
	tay
	pla
	plx			; Restore scratch+0, scratch+1 & .X
	stx	scratch+1
	plx
	stx	scratch+0
	plx
	rts
.endproc

;*****************************************************************************
; Allocate the requested number of bytes if they are available
;=============================================================================
; Inputs:	.A & Y = number of bytes (low/high)
; 		.X = bank
; Output:	.A & .Y = Handle ID (bank/cnt), carry clear on success
;		.C = set on error (.A contains error code)
;-----------------------------------------------------------------------------
; Preserves:	.X
; Uses:		ZP pointer
;*****************************************************************************
.proc mm_alloc: near
requested=scratch+2
needed=scratch+4
	sta	requested+0
	sty	requested+1
	; Ensure it is not a request for 0 bytes
	ora	requested+1
	bne	:+
	lda	#MM_ERR_ZERO
	sec
	rts
:	; Calculate space needed including 4 byte header
	clc
	lda	requested
	adc	#MEM_HDR_SIZE
	sta	needed
	lda	requested+1
	adc	#0
	sta	needed+1
	; Check if any dirty blocks can accomodate the needed memory
	ldy	#1
dirtyloop:
	lda	#0
	jsr	mm_get_ptr
	bcs	nodirty		; No dirty memory available
	; Dirty memory found, check if it is large enough
	jsr	readzp1		; Put current address in scratch
	sta	scratch+0
	sty	scratch+1
	jsr	lday_bank	; Get next address
	pha			; Ensure bit6 of high-byte is reset
	tya
	and	#$BF
	tay
	pla
	sec			; Subtract current address from next address
	sbc	scratch+0
	pha
	tya
	sbc	scratch+1
	tay
	pla
	sec			; Subtract the needes space from size of dirty memory
	sbc	needed
	tya
	sbc	needed+1
	ldy	#0
	bcc	dirtyloop	; If dirty memory is not large enough, search again
	; Dirty memory is large enough
	jsr	get_handle
	bcc	:+
	lda	#MM_ERR_NOHANDLE
	rts
:	pha
	lda	scratch+0
	ldy	scratch+1
	jsr	updzp1
	pla
	ldy	#MEM_HANDLE_OFS
	jsr	sta_bank
	jsr	lday_bank	; Read address to reset bit6 of high-byte
	pha
	tya
	and	#$BF		; Reset bit6 of high-byte
	tay
	pla
	jsr	stay_bank	; Update address
	jsr	update_checksum
	ldy	#MEM_HANDLE_OFS	; Read the handle
	jsr	lda_bank
	tay			; Combine with bank for handle_id
	txa
	rts
nodirty:; Check if available space is larger than requested space
	jsr	mm_remaining	; Remaining space in .A/.Y
	sec
	sbc	needed
	tya
	sbc	needed+1
	bcs	:+
	lda	#MM_ERR_NOSPACE
	sec
	rts
:	; --- Here we know that the memory is available
	; --- Ensure there is a handle to use
	jsr	get_handle
	bcc	:+
	lda	#MM_ERR_NOHANDLE
	rts
:	sta	scratch+2	; Store handle for later use
	; Update bank header with address of next available address
	lda	#<FREE_ADDR
	ldy	#>FREE_ADDR
	jsr	updzp1
	jsr	lday_bank	; Free address in .A/.Y
	sta	scratch		; Save in scratch for later use
	sty	scratch+1
	; Calculate new free address
	clc
	adc	needed
	pha
	tya
	adc	needed+1
	tay
	pla
	jsr	stay_bank	; Store in RAM bank header
	; Save new free address on stack
	pha
	phy
	; Set ZP to point to allocated memory block
	lda	scratch
	ldy	scratch+1
	jsr	updzp1
	; Write next available address to memory block header
	ply
	pla
	jsr	stay_bank
	; Write handle to memory block header
	ldy	#MEM_HANDLE_OFS
	lda	scratch+2
	jsr	sta_bank
	; Update checksum of header
	jsr	update_checksum
	; Write zero's as pointer in next memory block
	jsr	lday_bank
	jsr	updzp1
	lda	#0
	ldy	#0
	jsr	stay_bank
	; Combine handle_id from bank and handle counter
	txa
	ldy	scratch+2
	clc
	rts
.endproc

;*****************************************************************************
; Restores the interrupt vector to the one saved by mm_set_isr
;=============================================================================
; No inputs
;-----------------------------------------------------------------------------
; Uses: .A
;*****************************************************************************
.proc mm_clear_isr: near
	sei	; Disable interrupts
	lda	orig_isr
	sta	X16_Vector_IRQ
	lda	orig_isr+1
	sta	X16_Vector_IRQ+1
	cli	; Enable interrupts
	rts
.endproc

;*****************************************************************************
; Updates the ISR in lowram with the correct bank and address of the actual
; ISR, then installs the lowram ISR and e; sta (zp1)nsures it calls the original ISR
;=============================================================================
; Input:	.A & .Y = low- and high-byte of banked ISR
;		.X = bank of ISR
;-----------------------------------------------------------------------------
; Preserves:	.X
;*****************************************************************************
.proc mm_set_isr: near
	phy	; Preserve ISR address on stack
	pha
	; Store lowram_addr in ZP pointer
	lda	lowram_addr
	ldy	lowram_addr+1
	jsr	updzp1
	; Store low-byte of address for banked ISR
	pla
	ldy	#_isr_addr
	jsr	sta_bank
	; Store high-byte of address for banked ISR
	iny
	pla	; Load high-byte of ISR from stack
	jsr	sta_bank
	; Store bank# of banked ISR
	ldy	#_isr_bank
	txa
	jsr	sta_bank
	; Save original interrupt vector
	ldy	#_isr_orig
	lda	X16_Vector_IRQ
	sta	orig_isr
	jsr	sta_bank
	iny
	lda	X16_Vector_IRQ+1
	sta	orig_isr+1
	jsr	sta_bank
	; Install new interrupt vector
	clc
	sei	; Disable interrupts
	lda	lowram_addr	; Two first bytes of lowram is scratch storage
	adc	#2
	sta	X16_Vector_IRQ
	lda	lowram_addr+1
	adc	#0
	sta	X16_Vector_IRQ+1
	cli	; Enable interrupts
	rts
.endproc

;*****************************************************************************
; Update the memory bank header with next free address and zero handle bitmap
;=============================================================================
; Inputs:	.A & .Y = low- and high-byte of next free address
;			  $A000 if it's an empty bank
;		.X = Ram bank
;-----------------------------------------------------------------------------
; Preserves:	.X
;*****************************************************************************
.proc mm_init_bank: near
	phy
	pha
	lda	#<FREE_ADDR
	ldy	#>FREE_ADDR
	jsr	updzp1
	; If address is $A000, write first free address after header
	pla			; low-byte of free address
	; If low-byte is not 0, just write the address ot bank header
	bne	:+
	pla			; high-byte of free address
	pha			; Save on stack again
	cmp	#>FREE_ADDR	; Here we know that low-byte was 0 check if high-byte is $A0
	bne	:+		; If not, just write the address to bank header
	lda	#BANK_HDR_SIZE	; If it was zero, set low-byte to skip bank header
	; Write free address to bank header
:	ldy	#FREE_ADDR_OFS
	jsr	sta_bank
	ldy	#FIRST_ITEM_OFS
	jsr	sta_bank
	pla
	ldy	#FREE_ADDR_OFS+1
	jsr	sta_bank
	ldy	#FIRST_ITEM_OFS+1
	jsr	sta_bank
	; Zero out ID bitmap
	lda	#<ID_BITMAP
	ldy	#>ID_BITMAP
	jsr	updzp1
	lda	#0
	; Zero out bitmap and the address field of the first available memory
	ldy	#BITMAP_SIZE-1
:	jsr	sta_bank
	dey
	bne	:-
	jsr	sta_bank
	lda	#<FREE_ADDR
	ldy	#>FREE_ADDR
	jsr	updzp1
	jsr	lday_bank
	jsr	updzp1
	lda	#0
	ldy	#0
	jmp	stay_bank
.endproc

;*****************************************************************************
; Sets correct ZP pointer in functions, copies functions to lowram
; and updates jumptable 
;=============================================================================
; Inputs:	.A = First ZP address to use as pointer
;		.Y = Second ZP address to use as pointer
;		.X = First RAM bank to allocate memory in
;		Content of first ZP pointer should be the lowram address
;		Content of second ZP pointer should be first free address
;		  in RAM bank. $A000 if it is an empty RAM bank.
;		  Any RAM bank used by the library must reserve the first 3
;		  bytes of the RAM Bank for the library. $A000,$A001&$A002 
;-----------------------------------------------------------------------------
; Preserves:	.X
;*****************************************************************************
.proc mm_init: near
	jsr	mm_update_zp
	; Store low-ram addresses in library and low-ram mem copy function
zp0:	lda	$42
	sta	lowram_addr
	sta	lsl0+1		; low-ram scratch address low-byte
	sta	lsl1+1
zp1:	lda	$42+1
	sta	lowram_addr+1
	sta	lsl0+2
	sta	lsl1+2
	lda	lowram_addr
	inc
	sta	lsh0+1
	sta	lsh1+1
	lda	lowram_addr+1
	bcc	:+
	inc
:	sta	lsh0+2
	sta	lsh1+2

	; Copy routines to lowram
	ldy	#(_end_lowram-_low_scratch-1)	; Size of lowram segment
loop:	lda	_low_scratch,y
zp2:	sta	($42),y
	dey
	bne	loop
	; Update jumptable
	lda	#(_lda_bank-_low_scratch)	; Offset of _lda_bank
	clc
	adc	lowram_addr			; Add offset to lowram address
	sta	lda_bank+1
	lda	#0
	adc	lowram_addr+1
	sta	lda_bank+2

	lda	#(_lday_bank-_low_scratch)
	clc
	adc	lowram_addr
	sta	lday_bank+1
	lda	#0
	adc	lowram_addr+1
	sta	lday_bank+2

	lda	#(_ldyxa_bank-_low_scratch)
	clc
	adc	lowram_addr
	sta	ldyxa_bank+1
	lda	#0
	adc	lowram_addr+1
	sta	ldyxa_bank+2

	lda	#(_sta_bank-_low_scratch)
	clc
	adc	lowram_addr
	sta	sta_bank+1
	lda	#0
	adc	lowram_addr+1
	sta	sta_bank+2

	lda	#(_stay_bank-_low_scratch)
	clc
	adc	lowram_addr
	sta	stay_bank+1
	lda	#0
	adc	lowram_addr+1
	sta	stay_bank+2

	lda	#(_bank_cpy-_low_scratch)
	clc
	adc	lowram_addr
	sta	bank_cpy+1
	lda	#0
	adc	lowram_addr+1
	sta	bank_cpy+2
	; Initialize memory bank 
zp3:	lda	$42+0
zp4:	ldy	$42+1
	jmp	mm_init_bank
.endproc

;*****************************************************************************
; Update ZP pointers in the library
;=============================================================================
; Inputs:	.A = First ZP address to use as pointer
;		.Y = Second ZP address to use as pointer
;-----------------------------------------------------------------------------
; Uses:		.A & .Y
;*****************************************************************************
.proc mm_update_zp: near
	; Update ZP pointers
	sta	mm_init::zp0+1
	sta	mm_init::zp2+1
	sty	mm_init::zp3+1
	sta	updzp1+1
	sta	readzp1+1
	sty	updzp2+1
	sty	readzp2+1
	sta	zp01+1
	sta	zp02+1
	sta	zp03+1
	sta	zp04+1
	sta	zp05+1
	sta	zp06+1
	sta	zp07+1
	sta	zp08+1
	sta	zp09+1
	sta	zp10+1
	sta	zp11+1
	sty	zp20+1
	sty	zp21+1
	inc
	iny
	sta	mm_init::zp1+1
	sty	mm_init::zp4+1
	sta	updzp1+3
	sta	readzp1+3
	sty	updzp2+3
	sty	readzp2+3
	sta	zpp1+1
	sty	zpp2+1
	rts
.endproc

;*****************************************************************************
; Set a specified handle_id as free in the ID bitmap
;=============================================================================
; Inputs:	.A = cnt-part of handle_id
;		.X = bank
; Outputs:	.C clear
;-----------------------------------------------------------------------------
; Uses:		.A, .Y & first ZP pointer
; Preserves:	.X & scratch
;*****************************************************************************
.proc free_handle: near
	ldy	scratch		; Save content of scratch
	phy
	pha			; Save handle_id
	lsr
	lsr
	lsr
	tay			; Byte index in .Y
	pla			; Get handle_id
	phy			; Save byte index
	and	#%00000111	; bit index 0-7
	tay			; .Y = bit index
	; Create bitmask for freed handle
	lda	#1
	cpy	#0
	beq	bitmask_done
bitmask_loop:
	asl
	dey
	bne	bitmask_loop
bitmask_done:
	eor	#$FF		; Invert bitmask
	sta	scratch		; Save bitmask
	; Set ZP pointer
	lda	#<ID_BITMAP
	ldy	#>ID_BITMAP
	jsr	updzp1
	ply			; Get byte index
	jsr	lda_bank
	and	scratch		; zero the bit in the byte
	jsr	sta_bank
	pla			; Restore content of scratch
	sta	scratch
	clc
	rts
.endproc

;*****************************************************************************
; Return an available handle_id for the current bank
;=============================================================================
; Inputs:	.X = bank
; Outputs:	.A = handle_id
;		.C clear on success, otherwise set
;-----------------------------------------------------------------------------
; Uses:		.A, .Y & first ZP pointer
; Preserves	.X
;*****************************************************************************
.proc get_handle: near
	lda	scratch		; Save contents of scratch
	pha
	lda	#<ID_BITMAP
	ldy	#>ID_BITMAP
	jsr	updzp1
	ldy	#0
byte_loop:
	jsr	lda_bank	; Read byte from bitmap
	cmp	#$FF		; Are all bits set?
	bne	bit_available	; If not, there's an ID available
	iny
	cpy	#BITMAP_SIZE	; Have all bytes been checked?
	bne	byte_loop
	pla			; No handles available
	sta	scratch
	sec
	rts
bit_available:	; Here is a byte with an available bit
	phy			; Save byte index
	ldy	#0		; Initialize bit index
bit_loop:
	lsr			; Push bit to carry
	bcc	mark_found	; If carry clear, bit is found
	iny
	bra	bit_loop
mark_found:
	sty	scratch		; Save bit index
	pla			; Get byte index
	pha			; Save byte index again
	; Calculate handle (byte index * 8) + bit index
	asl	; * 2
	asl	; * 4
	asl	; * 8
	clc
	adc	scratch		; Add bit index
	pha			; Save handle_id
	; Set bit as occupied
	lda	#1		; Set right most bit
	cpy	#0		; Is bit index already 0?
	beq	shift_done
shift_loop:
	asl			; Shift bit left
	dey
	bne	shift_loop
shift_done:
	sta	scratch		; Save bitmask
	pla			; Get handle_id
	ply			; Get byte index
	pha			; Save handle_id
	jsr	lda_bank	; Read byte from bitmap again
	ora	scratch		; Combine with bitmask
	jsr	sta_bank	; Save the updated byte
	pla			; Get handle_id
	ply			; Restore scratch
	sty	scratch
	clc			; Success
	rts
.endproc

;*****************************************************************************
; Calculate and update the checksum of a memory block
;=============================================================================
; Inputs:	ZP1 pointing to head of memory block
;		.X = bank
;-----------------------------------------------------------------------------
; Uses:		.A, .Y
; Preserves	.X
;*****************************************************************************
.proc update_checksum: near
	lda	scratch+5	; Preserve scratch+5
	pha
	jsr	lday_bank
	sty	scratch+5
	eor	scratch+5
	sta	scratch+5
	ldy	#MEM_HANDLE_OFS
	jsr	lda_bank
	eor	scratch+5
	eor	#$AA
	iny	;MEM_CRC_OFS
	jsr	sta_bank
	pla			; Restore scratch+5
	sta	scratch+5
.endproc

;*****************************************************************************
; Check memory area header to ensure the checksum is correct
;=============================================================================
; Inputs:	ZP pointer must point to beginning of header
;		.X = bank
; Outputs:	.C = Clear on success otherwise Set
;-----------------------------------------------------------------------------
; Uses:		.A, .Y
; Preserves	.X
;*****************************************************************************
.proc check_header: near
	lda	scratch+5	; Preserve scratch+5
	pha
	jsr	lday_bank
	sty	scratch+5
	eor	scratch+5
	sta	scratch+5
	ldy	#MEM_HANDLE_OFS
	jsr	lda_bank
	eor	scratch+5
	eor	#$AA
	sta	scratch+5
	iny	;MEM_CRC_OFS
	jsr	lda_bank
	cmp	scratch+5
	clc
	beq	:+
	sec
:	pla			; Restore scratch+5
	sta	scratch+5
	rts
.endproc

;*****************************************************************************
; Update address of first ZP pointer
;=============================================================================
; Inputs:	.A & .Y = low- & high-byte of address
;-----------------------------------------------------------------------------
; Preserves:	All except first ZP pointer
;*****************************************************************************
.proc updzp1: near	; sta zp1+0, sty zp1+1
	sta	$42+0
	sty	$42+1
	rts
.endproc

;*****************************************************************************
; Update address of second ZP pointer
;=============================================================================
; Inputs:	.A & .Y = low- & high-byte of address
;-----------------------------------------------------------------------------
; Preserves:	All except second ZP pointer
;*****************************************************************************
.proc updzp2: near	; sta zp2+0, sty zp1+1
	sta	$42+0
	sty	$42+1
	rts
.endproc

;*****************************************************************************
; Get address stored in first ZP pointer
;=============================================================================
; Inputs:	none
; Outputs:	.A & .Y = low- & high-byte of address
;-----------------------------------------------------------------------------
; Preserves:	All except .A & .Y
;*****************************************************************************
.proc readzp1: near	; lda zp1+0, ldy zp1+1
	lda	$42+0
	ldy	$42+1
	rts
.endproc

;*****************************************************************************
; Get address stored in second ZP pointer
;=============================================================================
; Inputs:	none
; Outputs:	.A & .Y = low- & high-byte of address
;-----------------------------------------------------------------------------
; Preserves:	All except .A & .Y
;*****************************************************************************
.proc readzp2: near	; lda zp2+0, ldy zp2+1
	lda	$42+0
	ldy	$42+1
	rts
.endproc

;*****************************************************************************
; The below 5 functions were originally code inside the mm_free function.
; The code has been moved out into separate functions as they are also needed
; by the mm_defrag function.
; NOTE: These functions should not be called by any functions other than
;	* mm_free
;	* mm_defrag
;	They rely on zp pointers and scratch memory being set up before call
;*****************************************************************************
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
.proc calc_remaining_size: near
	lda	#<FREE_ADDR
	ldy	#>FREE_ADDR
	jsr	updzp1
	jsr	lday_bank
	sec			; Subtract the from-address
	sbc	scratch+2
	pha			; Store low-byte of remaining mem size on stack
	tya
	sbc	scratch+3
	tay
	pla
	rts
.endproc
.proc calc_freed_size: near
	sec			; Calculate difference between dest- and source-pointer
	lda	scratch+2	; Source-pointer in scratch+2&scratch+3
	sbc	scratch+0	; Dest-pointer in scratch+0&scratch+1
	sta	scratch+0	; Store size in scratch+0&scratch+1
	lda	scratch+3
	sbc	scratch+1
	sta	scratch+1
	rts
.endproc
.proc store_new_free_addr: near
	jsr	lday_bank	; Get free address
	sec			; Subtract size of freed mem
	sbc	scratch+0	; located in scratch+0&scratch+1
	pha
	tya
	sbc	scratch+1
	tay
	pla
	jmp	stay_bank	; Update memory bank with new free address
.endproc
.proc zero_last_free: near
	lda	#<FREE_ADDR
	ldy	#>FREE_ADDR
	jsr	updzp1
	jsr	lday_bank	; Get last address
	jsr	updzp1
	lda	#0
	ldy	#0
	jmp	stay_bank	; Store 0's to it
.endproc
.proc update_mem_headers: near
	pha
	tya			; Move high-byte to .A to reset bit6
	and	#$BF
	tay
	pla
	jsr	updzp1		; Store address in zp1 pointer
	jsr	lday_bank	; Read address of next memory object
	pha
	sty	scratch+5
	ora	scratch+5
	beq	:+		; If the address is all 0's, we are done
	pla
	sec
	sbc	scratch+0	; Subtract the freed size
	pha
	tya
	; If bit6 of high-byte is set, reset it for calculation
	and	#$40
	bne	isset
	tya
	sbc	scratch+1
	bra	store
isset:	tya
	and	#$BF		; Reset bit6
	sbc	scratch+1
	ora	#$40		; Set bit6
store:	tay
	pla
	jsr	stay_bank	; Store it back to the memory header
	jsr	update_checksum
	jsr	lday_bank	; Get address of next memory object
	bra	update_mem_headers
:	pla			; Cleanup stack
	rts
.endproc
;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

.segment "MMLOWRAM"
_low_scratch:
	.word	0
;*****************************************************************************
; The current RAM bank is saved, RAM bank set correctly and a jsr to ISR
; function is performed before continuing on to previous interrupt handler
;=============================================================================
; ROM has already preserved registers and previous interrupt handler should
; ensure they are restored. RAM bank is restored before call to original
; interrupt handler
;*****************************************************************************
_isr:
	; Save current RAM bank
	lda	X16_RAMBank_Reg
	pha
	; Set new RAM bank
	lda	#$FF	; Must be modified after copy to lowram (offset=4)
	sta	X16_RAMBank_Reg
	; Call banked ISR
	jsr	$FFFF	; Must be modified after copy to lowram (offset=8,9)
	; Restore RAM bank
	pla
	sta	X16_RAMBank_Reg
	; Continue to original ISR
	jmp	$FFFF	; Must be modified after copy to lowram (offset=14,15)

;*****************************************************************************
; Load a byte from banked RAM pointed to by ZP pointer LD_ST_BANK_PTR in bank
; specified by the value in .X
;=============================================================================
; Arguments:	ZP pointer to the memory address that should be read
;		.X = the RAM bank to read from
;		.Y = offset from ZP pointer
;-----------------------------------------------------------------------------
; Preserves:	.X, .Y and the RAM bank before call
; Returns:	.A = the value read from memory
;*****************************************************************************
_lda_bank:
	phx			; Preserve bank
	phy			; Preserve offset
	ldy	X16_RAMBank_Reg	; Save current RAM bank
	stx	X16_RAMBank_Reg	; Set new RAM bank
	phy			; Move original RAM bank from Y to X through stack
	plx
	ply			; Pull offset from stack
zp01:	lda	($42),y		; Load value from address pointed to by ZP pointer
	stx	X16_RAMBank_Reg	; Restore original RAM bank
	plx			; Restore RAM bank from caller
	pha			; Push and restore value to set flags
	pla
	rts

;*****************************************************************************
; Load two bytes from banked RAM pointed to by $42 pointer LD_ST_BANK_PTR in
; bank specified by the value in .X
;=============================================================================
; Arguments:	ZP pointer to the memory address that should be read
;		.X = the RAM bank to read from
;-----------------------------------------------------------------------------
; Preserves:	.X
; Returns:	.A = value from ZP pointer, .Y from ZP pointer + 1
;*****************************************************************************
_lday_bank:
	phx			; Preserve X
	ldy	X16_RAMBank_Reg	; Save original RAM bank
	stx	X16_RAMBank_Reg	; Switch RAM bank
	phy			; Move original RAM bank through stack to X
	plx
	ldy	#1		; Read highbyte into Y
zp02:	lda	($42),y
	tay
zp03:	lda	($42); Read lowbyte into A
	stx	X16_RAMBank_Reg	; Restore original RAM bank
	plx			; Restore X
	rts

;*****************************************************************************
; Load three bytes from banked RAM pointed to by ZP pointer LD_ST_BANK_PTR in00
; bank specified by the value in .X
;=============================================================================
; Arguments:	ZP pointer to the memory address that should be read
;		.X = the RAM bank to read from
;-----------------------------------------------------------------------------
; Preserves:	The RAM bank before call
; Returns:	.Y=ZP pointer, .X=ZP pointer+1, .A=ZP pointer+2
;*****************************************************************************
_ldyxa_bank:
	ldy	X16_RAMBank_Reg	; Save current RAM bank
	stx	X16_RAMBank_Reg	; Set new RAM bank
zp04:	lda	($42)		; Load value from address pointed to by ZP pointer
	pha			; Save value to stack
	phy			; Save RAM bank to stack
	ldy	#1		; Initialize .Y for reading value through ZP pointer
zp05:	lda	($42),y		; Load next value from address pointed to by ZP pointer with .Y
	tax			; Store value in .X
	iny
zp06:	lda	($42),y		; Load next value from address pointed to by ZP pointer with .Y
	ply			; Restore original RAM bank
	sty	X16_RAMBank_Reg
	ply			; Get first read value from stack
	rts

;*****************************************************************************
; Store a byte to banked RAM pointed to by ZP pointer LD_ST_BANK_PTR in bank
; specified by the value in .X
;=============================================================================
; Arguments:	ZP pointer to memory address that should be written
;		.X = the RAM bank to write to
;		.A = the value to write
;		.Y = the offset to the ZP pointer to write to
;-----------------------------------------------------------------------------
; Preserves: 	A, .X, .Y and the RAM bank before call
; Returns:	nothing
;*****************************************************************************
_sta_bank:
	phx			; Preserve .X 
	pha			; Preserve values to write
	lda	X16_RAMBank_Reg	; Save current RAM bank
	stx	X16_RAMBank_Reg	; Set new RAM bank
	tax			; Move original RAM bank to .X
	pla			; Restore value to write
zp07:	sta	($42),y		; Store value in .A to address pointed to by ZP pointer
	stx	X16_RAMBank_Reg	; Restore RAM bank
	plx			; Restore .X
	rts

;*****************************************************************************
; Store two bytes to banked RAM pointed to by ZP pointer LD_ST_BANK_PTR in
; bank specified by the value in .X
;=============================================================================
; Arguments:	ZP pointer to memory address that should be written
;		.X = the RAM bank to write to
;		.A & .Y = the values to write in that order (low-high)
;-----------------------------------------------------------------------------
; Preserves:	.A, .X, .Y & The RAM bank before call
; Returns:	nothing
;*****************************************************************************
_stay_bank:
	pha			; Preserve A register
	pha			; Preserve low byte
	lda	X16_RAMBank_Reg	; Save current RAM bank in .A
	stx	X16_RAMBank_Reg	; Set the new RAM bank
	plx			; Pull .low byte from stack and store it in .X temporarily
	pha			; Push original RAM bank to stack
	txa			; Move low byte from .X back to .A
zp08:	sta	($42)		; Store low byte to address pointed to by ZP pointer
	tya			; Store high byte to address pointed to by ZP pointer with .Y added
	ldy	#1
zp09:	sta	($42),y
	tay			; Restore high byte to Y
	ldx	X16_RAMBank_Reg	; Restore RAM bank value from the call
	pla			; Restore RAM bank to original
	sta	X16_RAMBank_Reg
	pla			; Restore A register
	rts

;*****************************************************************************
; Copy contents of memory region to other memory region
;=============================================================================
; Inputs:	First ZP pointer should point to source memory region
;		Second ZP pointer should point to destination memory region
;		Two first bytes of low-ram = number of bytes to copy
;		.A = Source RAM bank
;		.X = Destination RAM bank
;	- If .A = .X, RAM bank will be set and not changed during copy
;-----------------------------------------------------------------------------
; Preserves:	.X & RAM bank
; Uses:		.A, .Y, ZP pointers & two first bytes of low-ram
;*****************************************************************************
_bank_cpy:
	; Save current RAM bank
	ldy	X16_RAMBank_Reg
	phy
	ldy	#0
	; Set source bank
	sta	X16_RAMBank_Reg
loop:	cpx	X16_RAMBank_Reg
	bne	:+
zp10:	lda	($42),y
zp20:	sta	($44),y
	bra	cont
:	pha
	; Read byte and save on stack
zp11:	lda	($42),y
	stx	X16_RAMBank_Reg
zp21:	sta	($44),y
	; Read source bank from stack and set it
	pla
	sta	X16_RAMBank_Reg
cont:	iny
	bne	:+
zpp1:	inc	42+1
zpp2:	inc	44+1
:	; Decrement counter
lsl0:	lda	_low_scratch+0
	bne	:+
lsh0:	dec	_low_scratch+1
:
lsl1:	dec	_low_scratch+0
	; Check if we have reached 0
	dec
lsh1:	ora	_low_scratch+1
	bne	loop
	ply
	sty	X16_RAMBank_Reg
	rts

_end_lowram:

.assert __MMLOWRAM_SIZE__ <= 255, error, "MMLOWRAM segment may not be larger than 255 bytes"
