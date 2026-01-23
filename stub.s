.include "memman.inc"

.segment "HEADER"
rem_space:	.word 0
free_addr:	.word 0
.segment "JUMPTABLE"
	jmp	mm_init		; $A004
