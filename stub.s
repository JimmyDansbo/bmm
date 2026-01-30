.include "memman.inc"

.segment "HEADER"
free_addr:	.word $BF37
first_item:	.word $BF37
id_bitmap:	.res 32

.segment "JUMPTABLE"
	jmp	mm_init		; $A024
	jmp	mm_set_isr
	jmp	mm_clear_isr
	jmp	mm_alloc
	jmp	mm_remaining
	jmp	mm_free
	jmp	mm_update_zp
	jmp	mm_init_bank
	jmp	mm_get_ptr
