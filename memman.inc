.ifndef memman_inc
memman_inc=1

MM_ERR_ZERO	= $01
MM_ERR_NOSPACE	= $02
MM_ERR_NOHANDLE = $03

.ifndef SKIPIMPORT
.import mm_init, mm_set_isr, mm_clear_isr, mm_alloc, mm_remaining, mm_free, mm_init_bank
.import mm_update_zp, mm_get_ptr
.endif

.endif