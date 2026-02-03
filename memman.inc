.ifndef memman_inc
memman_inc=1

MM_ERR_ZERO		= $01
MM_ERR_NOSPACE		= $02
MM_ERR_NOHANDLE 	= $03
MM_ERR_HANDLE_NOTFOUND	= $04
MM_ERR_CORRUPT_HDR	= $05

MM_LDA_BANK_OFFS	= $12
MM_LDAY_BANK_OFFS	= $21
MM_LDYXA_BANK_OFFS	= $33
MM_STA_BANK_OFFS	= $48
MM_STAY_BANK_OFFS	= $56
MM_BANK_COPY_OFFS	= $6E

.ifndef SKIPIMPORT
.import mm_init, mm_set_isr, mm_clear_isr, mm_alloc, mm_remaining, mm_free, mm_init_bank
.import mm_update_zp, mm_get_ptr, mm_defrag, mm_get_size
.endif

.endif