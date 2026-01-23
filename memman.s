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

_isr_bank=4
_isr_addr=8
_isr_orig=14

.segment "MEMMAN"
; Internal jump table into lowram functions
; The bank-load and store functions use LD_ST_BANK_PTR for the address and X for bank
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
	lda	#>REM_SPACE	; Set address $A000 in ZP pointer
zp18:	stz	$42
zpp2:	sta	$42+1
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
zp19:	sta	$42
	lda	#>FREE_ADDR
zpp3:	sta	$42+1
	jsr	lday_bank	; Read next available address from bank
	sta	scratch
zp23:	sta	$42
	sty	scratch+1
zpp5:	sty	$42+1

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
zp20:	stz	$42
zpp6:	sta	$42+1
	pla
	jsr	stay_bank	; Store new free space
	lda	#<FREE_ADDR
zp21:	sta	$42
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
; ISR, then installs the lowram ISR and ensures it calls the original ISR
;=============================================================================
; Input:	.A & .Y = low- and high-byte of banked ISR
;		.X = bank of ISR
;-----------------------------------------------------------------------------
; Preserves:	.X
;*****************************************************************************
.proc mm_set_isr: near
	phy	; Preserve high-byte of ISR on stack
	; Store lowram_addr in ZP pointer
	ldy	lowram_addr
zp03:	sty	$42
	ldy	lowram_addr+1
zpp1:	sty	$42+1
	; Store low-byte of address for banked ISR
	ldy	#_isr_addr
zp04:	sta	($42),y
	; Store high-byte of address for banked ISR
	iny
	pla	; Load high-byte of ISR from stack
zp05:	sta	($42),y
	; Store bank# of banked ISR
	ldy	#_isr_bank
	txa
zp06:	sta	($42),y
	; Save original interrupt vector
	ldy	#_isr_orig
	lda	X16_Vector_IRQ
	sta	orig_isr
zp07:	sta	($42),y
	iny
	lda	X16_Vector_IRQ+1
	sta	orig_isr+1
zp08:	sta	($42),y
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
; Inputs:	.A & .X = low- & high-byte of lowram address reserved for
;		the x16vision library.
;		.Y = ZP address to use as pointer
;-----------------------------------------------------------------------------
; Preserves:	.X
;*****************************************************************************
.proc mm_init: near
	; Save the lowram address
	sta	lowram_addr
	stx	lowram_addr+1

	stz	handle_id

	jsr	update_zp_pointers

zp00:	sta	$42
zpp0:	stx	$42+1
	; Copy routines to lowram
	ldy	#(_end_lowram-_low_scratch-1)	; Size of lowram segment
loop:	lda	_low_scratch,y
zp01:	sta	($42),y
	dey
	bne	loop
	lda	_low_scratch
zp02:	sta	($42)
	; Update jumptable
	lda	#(_lda_bank-_low_scratch)	; Offset of _lda_bank
	clc
	adc	lowram_addr		; Add offset to lowram address
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
; Copy contents of banked memory region to other memory region in same bank
;=============================================================================
; Inputs:	.X = bank
;		scratch+0 & scratch+1 = Pointer to source
;		scratch+2 & scratch+3 = Pointer to destination
;		scratch+4 & scratch+5 = Number of bytes to copy
;-----------------------------------------------------------------------------
; Preserves:	.X
; Uses:		All 6 bytes of scratch are used as well as .A and .Y
;*****************************************************************************
.proc memcpy: near
	ldy	#0
loop:	; Write source address to ZP
	lda	scratch+0
zp24:	sta	$42
	lda	scratch+1
zpp7:	sta	$42+1
	; Read byte and save on stack
	jsr	lda_bank
	pha
	; Write destination address to ZP
	lda	scratch+2
zp25:	sta	$42
	lda	scratch+3
zpp8:	sta	$42+1
	; Restore byte from stack and write to pointer
	pla
	jsr	sta_bank
	; Increment .Y as offset and high-byte of pointers if necessary
	iny
	bne	:+
	inc	scratch+1
	inc	scratch+3
:	; Decrement counter
	lda	scratch+4
	bne	:+
	dec	scratch+5
:	dec	scratch+4
	; Check if we have reached 0
	dec
	ora	scratch+5
	bne	loop
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
zp22:	stz	$42
zpp4:	sta	$42+1
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

zp_table:
	.word mm_init::zp00+1
	.word mm_init::zp01+1
	.word mm_init::zp02+1
	.word mm_set_isr::zp03+1
	.word mm_set_isr::zp04+1
	.word mm_set_isr::zp05+1
	.word mm_set_isr::zp06+1
	.word mm_set_isr::zp07+1
	.word mm_set_isr::zp08+1
	.word zp09+1
	.word zp10+1
	.word zp11+1
	.word zp12+1
	.word zp13+1
	.word zp14+1
	.word zp15+1
	.word zp16+1
	.word zp17+1
	.word mm_remaining::zp18+1
	.word mm_alloc::zp19+1
	.word mm_alloc::zp20+1
	.word mm_alloc::zp21+1
	.word check_space::zp22+1
	.word mm_alloc::zp23+1
	.word memcpy::zp24+1
	.word memcpy::zp25+1
	.word $0000

;*****************************************************************************
; Update all ZeroPage pointers in the library with the ZeroPage pointer in .Y
;=============================================================================
; Inputs:	.Y = low-byte of ZP pointer to use
;-----------------------------------------------------------------------------
; Preserves:	.A, .X & .Y
;*****************************************************************************
.proc update_zp_pointers: near
	pha
	phx
	; Self-modify to use ZP pointer
	sty	@zp0+1
	sty	@zp1+1
	iny
	sty	@zpp0+1
	dey
	; Go through ZP table and update low-byte of ZP pointer
	ldx	#0
@loop:	lda	zp_table, x
@zp0:	sta	$42
	ora	zp_table+1, x
	beq	@done
	lda	zp_table+1, x
@zpp0:	sta	$42+1
	tya
@zp1:	sta	($42)
	inx
	inx
	bra	@loop
@done:	
	plx
	pla
	; Update high-byte of ZP pointer
	iny
	sty	mm_init::zpp0+1
	sty	mm_set_isr::zpp1+1
	sty	mm_remaining::zpp2+1
	sty	mm_alloc::zpp3+1
	sty	check_space::zpp4+1
	sty	mm_alloc::zpp5+1
	sty	mm_alloc::zpp6+1
	sty	memcpy::zpp7+1
	sty	memcpy::zpp8+1
	rts
.endproc

.segment "LOWRAM"
_low_scratch: .byte $00, $00, $00, $00, $00
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
zp09:	lda	($42),y		; Load value from address pointed to by ZP pointer
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
zp10:	lda	($42),y
	tay
zp11:	lda	($42); Read lowbyte into A
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
zp12:	lda	($42)		; Load value from address pointed to by ZP pointer
	pha			; Save value to stack
	phy			; Save RAM bank to stack
	ldy	#1		; Initialize .Y for reading value through ZP pointer
zp13:	lda	($42),y		; Load next value from address pointed to by ZP pointer with .Y
	tax			; Store value in .X
	iny
zp14:	lda	($42),y		; Load next value from address pointed to by ZP pointer with .Y
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
zp15:	sta	($42),y		; Store value in .A to address pointed to by ZP pointer
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
zp16:	sta	($42)		; Store low byte to address pointed to by ZP pointer
	tya			; Store high byte to address pointed to by ZP pointer with .Y added
	ldy	#1
zp17:	sta	($42),y
	tay			; Restore high byte to Y
	ldx	X16_RAMBank_Reg	; Restore RAM bank value from the call
	pla			; Restore RAM bank to original
	sta	X16_RAMBank_Reg
	pla			; Restore A register
	rts
; X dest bank
; A src bank
_bank_cpy:
	; Save current RAM bank
	ldy	X16_RAMBank_Reg
	phy
	; Set source bank
	sta	X16_RAMBank_Reg
	pha
	ldy	#0
loop:	; Write source address to ZP
	lda	scratch+0
zp24:	sta	$42
	lda	scratch+1
zpp7:	sta	$42+1
	; Read byte and save on stack
	lda	($42)
	pha
	; Write destination address to ZP
	lda	scratch+2
zp25:	sta	$42
	lda	scratch+3
zpp8:	sta	$42+1
	; Restore byte from stack and write to destination address
	pla
	sta	($42)
	; Read source bank from stack and set it
	pla
	sta	X16_RAMBank_Reg
	iny
	bne	:+
	inc	scratch+1
	inc	scratch+3
:	; Decrement counter
	lda	scratch+4
	bne	:+
	dec	scratch+5
:	dec	scratch+4
	; Check if we have reached 0
	dec
	ora	scratch+5
	bne	loop
	ply
	sty	X16_RAMBank_Reg
	rts

_end_lowram:

.assert __LOWRAM_SIZE__ <= 255, error, "LOWRAM segment may not be larger than 255 bytes"
