.include "memman.inc"

.segment "HEADER"
free_addr:	.word $BF37
handle_cnt:	.byte $42
first_item:	.word $BF37

.segment "JUMPTABLE"
	jmp	mm_init		; $A005
	jmp	mm_set_isr	; $A008
	jmp	mm_clear_isr	; $A00B
	jmp	mm_alloc	; $A00E
	jmp	mm_remaining	; $A011
	jmp	mm_free		; $A014
	jmp	mm_update_zp	; $A017
	jmp	mm_init_bank	; $A01A
	jmp	mm_get_ptr	; $A01D
