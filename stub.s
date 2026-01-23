.include "memman.inc"

.segment "HEADER"
rem_space:	.word $C8
free_addr:	.word $BF37
.segment "JUMPTABLE"
	jmp	mm_init		; $A004
	jmp	mm_set_isr	; $A007
	jmp	mm_clear_isr	; $A00A
	jmp	mm_alloc	; $A00D
	jmp	mm_remaining	; $A010
	jmp	mm_free		; $A013
	jmp	mm_get_next_handle ; A016
	jmp	mm_set_next_handle ; $A119