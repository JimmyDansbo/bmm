.include "x16.inc"
SKIPIMPORT=1
.include "memman.inc"

.import __LOWRAM_SIZE__
.export mm_init, mm_set_isr, mm_clear_isr, mm_alloc, mm_remaining, mm_free, mm_init_bank
.export mm_update_zp, mm_get_ptr

.segment "MEMMANBSS"
lowram_addr:	.res	2
orig_isr:	.res	2
scratch:	.res	6

FREE_ADDR	= $A000
HANDLE_CNT	= $A002
first_item	= $A003

; Offsets from low-ram start address
_isr_bank	= 6
_isr_addr	= 10
_isr_orig	= 16


.segment "MEMMAN"
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
bank_cpy:
	jmp	$0000

;*****************************************************************************
; Free the block of memory at pointer and move any following used memory
;=============================================================================
;-----------------------------------------------------------------------------
;*****************************************************************************
.proc mm_get_ptr: near
	rts
.endproc

;*****************************************************************************
; Free the block of memory at pointer and move any following used memory
;=============================================================================
;-----------------------------------------------------------------------------
;*****************************************************************************
.proc mm_free: near
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
	lda	#<FREE_ADDR	; Set address $A000 in ZP pointer
	ldy	#>FREE_ADDR
	jsr	updzp1
	jsr	lday_bank	; Read next free address from bank
	sta	scratch
	sty	scratch+1
	; Subtract free address from $C000
	lda	#<X16_RAM_WindowEnd	; low-byte
	sec
	sbc	scratch
	pha
	lda	#>X16_RAM_WindowEnd	; high-byte
	sbc	scratch+1
	tay
	pla
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
newfree=scratch+0
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
	adc	#4
	sta	needed
	lda	requested+1
	adc	#0
	sta	needed+1
	; Check if available space is larger than requested space
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
	; Update bank header with address of next available address
	lda	#<FREE_ADDR
	ldy	#>FREE_ADDR
	jsr	updzp1
	; Read handle counter from bank header
	ldy	#2
	jsr	lda_bank
	sta	scratch+2
	; If next handle counter value is 0, we have run out of handles
	inc
	bne	:+
	lda	#MM_ERR_NOHANDLE
	sec
	rts
:	jsr	sta_bank	; Save new handle counter value
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
	ldy	#2
	lda	scratch+2
	jsr	sta_bank
	; Calculate checksum of header
	jsr	lday_bank
	sty	scratch
	eor	scratch
	sta	scratch
	ldy	#2
	jsr	lda_bank
	eor	scratch
	eor	#$AA
	ldy	#3
	jsr	sta_bank
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
; Update the memory bank header with next free address and zero out handle cnt
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
	; Set handle counter to 0
	ldy	#2
	lda	#0
	jsr	sta_bank
	; Write free address to bank header
	pla			; low-byte of free address
	ldy	#0		; Store as low-byte of next free in bank header
	jsr	sta_bank	
	ldy	#3		; Store as lowb-yte of first item in bank header
	jsr	sta_bank
	pla			; high-byte of free address
	ldy	#1		; Store as high-byte of next free in bank header
	jsr	sta_bank
	ldy	#4		; Store as high-byte of first item in bank header
	jsr	sta_bank
	; Check if address is $A000
	cmp	#>FREE_ADDR
	bne	:+
	dey
	jsr	lda_bank
	cmp	#<FREE_ADDR
	bne	:+
	lda	#$03
	ldy	#$01
	jsr	sta_bank
	ldy	#$04
	jmp	sta_bank
:	rts
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
	sty	updzp2+1
	sta	inczp1+1
	sty	inczp2+1
	sta	zerozp1l+1
	sty	zerozp2l+1
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
	sty	updzp2+3
	sta	updzp1h+1
	sty	updzp2h+1
	sta	inczp1+5
	sty	inczp2+5
	sta	inczp1h+1
	sty	inczp2h+1
	sta	zpp1+1
	sty	zpp2+1
	rts
.endproc

.proc updzp1: near	; sta zp1+0, sty zp1+1
	sta	$42+0
	sty	$42+1
	rts
.endproc
.proc updzp2: near	; sta zp2+0, sty zp2+1
	sta	$42+0
	sty	$42+1
	rts
.endproc
.proc updzp1h: near	; sta zp1+1
	sta	$42+1
	rts
.endproc
.proc updzp2h: near	; sta zp2+1
	sta	$42+1
	rts
.endproc
.proc inczp1: near	; inc zp1, bne :+, inc zp1+1
	inc	$42
	bne	:+
	inc	$42+1
:	rts
.endproc
.proc inczp2: near	; inc zp2, bne :+, inc zp2+1
	inc	$42
	bne	:+
	inc	$42+1
:	rts
.endproc
.proc inczp1h: near	; inc zp1+1
	inc	$42+1
	rts
.endproc
.proc inczp2h: near	; inc zp2+1
	inc	$42+1
	rts
.endproc
.proc zerozp1l: near	; stz zp1
	stz	$42
	rts
.endproc
.proc zerozp2l: near	; stz zp2
	stz	$42
	rts
.endproc

.segment "LOWRAM"
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
loop:	sta	X16_RAMBank_Reg
	cpx	X16_RAMBank_Reg
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

.assert __LOWRAM_SIZE__ <= 255, error, "LOWRAM segment may not be larger than 255 bytes"
