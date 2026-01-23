.include "x16.inc"
SKIPIMPORT=1
.include "memman.inc"

.import __LOWRAM_SIZE__, __LOWRAM_SIZE__
.export mm_init, mm_set_isr, mm_clear_isr, mm_alloc, mm_remaining, mm_free, mm_get_next_handle
.export mm_set_next_handle

.segment "MEMMANBSS"
lowram_addr:	.res	2
orig_isr:	.res	2
handle_id:	.res	2
scratch:	.res	6

REM_SPACE=$A000
FREE_ADDR=$A002

_isr_bank=6
_isr_addr=10
_isr_orig=16

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
	lda	#<REM_SPACE	; Set address $A000 in ZP pointer
	ldy	#>REM_SPACE
	jsr	updzp1
	jmp	lday_bank	; Read available memory from bank
.endproc

;*****************************************************************************
; Allocate the requested number of bytes if they are available
;=============================================================================
; Inputs:	.A = number of bytes
; 		.X = bank
; Output:	.A & .Y = low- and high-byte of address of allocated memory
;		.C = set if memory is available, otherwise clear
;-----------------------------------------------------------------------------
; Preserves:	.X
; Uses:		ZP pointer
;*****************************************************************************
.proc mm_alloc: near
	; Ensure that requested amount is more than 0 bytes
	cmp	#0
	bcs	:+
	clc		; Return carry clear if requested amount is 0
	rts
:	; Ensure that requested amount is less than 255 bytes
	cmp	#$FF
	bne	:+
	clc
	rts
:	inc	; Allocate an extra byte for header/size information
	jsr	check_space
	bcc	end
	; Memory is available here. Calculate the new free space and pointer
	dec	; Save only the amount the user has requested
	pha	; Save the number of bytes being requested
	; Set ZP pointer to FREE_ADDR
	lda	#<FREE_ADDR
	ldy	#>FREE_ADDR
	jsr	updzp1
	jsr	lday_bank	; Read next available address from bank
	jsr	updzp1
	sta	scratch
	sty	scratch+1

	pla			; Restore number of bytes being requested (+1)
	ldy	#0
	jsr	sta_bank	; Store as header of allocated memory

	inc	; We are still allocating 1 byte more than requested for the header
	; Save address on stack as this needs to be returned to caller
	ldy	scratch
	phy			; low-byte
	ldy	scratch+1
	phy			; high-byte

	clc			; Calculate new next available address
	adc	scratch
	sta	scratch
	lda	#0
	adc	scratch+1
	sta	scratch+1
	; Calculate new free space after allocation of memory
	lda	#<X16_ROM_Window
	sec
	sbc	scratch
	pha
	lda	#>X16_ROM_Window
	sbc	scratch+1
	tay
	lda	#>REM_SPACE
	jsr	zerozp1l
	jsr	updzp1h
	pla
	jsr	stay_bank	; Store new free space
	lda	#<FREE_ADDR
	ldy	#>FREE_ADDR
	jsr	updzp1
	lda	scratch
	ldy	scratch+1
	jsr	stay_bank	; Store pointer to next available space
	; Load start of allocated memory back into .A & .Y, add 1 to skip header
	ply
	pla
	inc
	bne	:+
	iny
:	sec
end:	rts
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
	jsr	storzp1y
	; Store high-byte of address for banked ISR
	iny
	pla	; Load high-byte of ISR from stack
	jsr	storzp1y
	; Store bank# of banked ISR
	ldy	#_isr_bank
	txa
	jsr	storzp1y
	; Save original interrupt vector
	ldy	#_isr_orig
	lda	X16_Vector_IRQ
	sta	orig_isr
	jsr	storzp1y
	iny
	lda	X16_Vector_IRQ+1
	sta	orig_isr+1
	jsr	storzp1y
	; Install new interrupt vector
	sei	; Disable interrupts
	lda	lowram_addr
	sta	X16_Vector_IRQ
	lda	lowram_addr+1
	sta	X16_Vector_IRQ+1
	cli	; Enable interrupts
	rts
.endproc

;*****************************************************************************
; Sets correct ZP pointer in functions, copies functions to lowram
; and updates jumptable 
;=============================================================================
; Inputs:	.A = First ZP address to use as pointer
;		.Y = Second ZP address to use as pointer
;		Content of first ZP pointer should be the lowram address
;-----------------------------------------------------------------------------
;*****************************************************************************
.proc mm_init: near
	; Update ZP pointers
	sta	@zp0+1
	sta	readzp1+1
	sta	readzp1y+1
	sta	storzp1+1
	sta	storzp1y+1
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
	sta	@zp1+1
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
	
	; Store low-ram addresses in library and low-ram mem copy function
@zp0:	lda	$42
	sta	lowram_addr
	sta	lsl0+1
	sta	lsl1+1
@zp1:	lda	$42+1
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

	; Reset handle_id
	stz	handle_id

	; Copy routines to lowram
	ldy	#(_end_lowram-_low_scratch-1)	; Size of lowram segment
loop:	lda	_low_scratch,y
	jsr	storzp1y
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
	rts
.endproc

;*****************************************************************************
; Get the handle id that will be given out next time a block of memory is
; allocated. This can be used to save the handle id during a re-initialization
; of the memory manager.
;=============================================================================
; Outputs:	.A & .Y = low- and high-byte of handle id
;-----------------------------------------------------------------------------
; Preserves:	All but .A & .Y
;*****************************************************************************
.proc mm_get_next_handle: near
	lda	handle_id+0
	ldy	handle_id+1
	rts
.endproc

;*****************************************************************************
; Tell the memory manager which handle id to give out next time a block of
; memory is allocated. This is useful if the array has been re-initialized
; to use another ZP pointer as that also resets the next handle id to 0
;=============================================================================
; Inputs:	.A & .Y = low- and high-byte of handle id
;-----------------------------------------------------------------------------
; Preserves:	All
;*****************************************************************************
.proc mm_set_next_handle: near
	sta	handle_id+0
	sty	handle_id+1
	rts
.endproc

;*****************************************************************************
; Check if requested amount of space is available in the specified bank
; Maximum request size is 255 bytes
;=============================================================================
; Inputs:	.A = number of bytes 
;		.X = bank
; Output:	Carry set if the memory is available, otherwise clear
;-----------------------------------------------------------------------------
; Uses:		ZP pointer
; Preserves:	.A & .X
;*****************************************************************************
.proc check_space: near
	pha			; Save the number of bytes being requested
	lda	#>REM_SPACE	; Set address $A000 in ZP pointer
	jsr	zerozp1l
	jsr	updzp1h
	jsr	lday_bank	; Read available memory from bank
	sta	scratch
	sty	scratch+1
	pla			; Restore bytes requested
	ldy	scratch+1	; Check high-byte
	bne	good
	; Here, we know that high-byte is 0
	pha			; Save on stack again
	sta	scratch+1	; Store bytes requested to scratch and load
	lda	scratch		; available bytes into A for CMP
	cmp	scratch+1
	pla			; Restore bytes requested
	rts	; Exit function with C set if mem avail otherwise clear
good:	sec
	rts
.endproc


.proc readzp1: near	; lda (zp1)
	lda	($42)
	rts
.endproc
.proc readzp1y: near	; lda (zp1),y
	lda	($42),y
	rts
.endproc
.proc storzp1: near	; sta (zp1)
	sta	($42)
	rts
.endproc
.proc storzp1y: near	; sta (zp1),y
	sta	($42),y
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
