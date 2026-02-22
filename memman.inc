.ifndef memman_inc
memman_inc=1

MEMMAN_VERSION	= $000C

MM_ERR_ZERO		= $01
MM_ERR_NOSPACE		= $02
MM_ERR_NOHANDLE		= $03
MM_ERR_HANDLE_NOTFOUND	= $04
MM_ERR_CORRUPT_HDR	= $05

MM_LDA_BANK_OFFS	= $12
MM_LDAY_BANK_OFFS	= $21
MM_LDAYX_BANK_OFFS	= $33
MM_STA_BANK_OFFS	= $48
MM_STAY_BANK_OFFS	= $56
MM_BANK_COPY_OFFS	= $6E

.ifndef SKIPIMPORT
.import mm_init, mm_set_isr, mm_clear_isr, mm_alloc, mm_remaining, mm_free, mm_init_bank
.import mm_update_zp, mm_get_ptr, mm_defrag, mm_get_size
.import mm_lda_bank, mm_lday_bank, mm_ldayx_bank, mm_sta_bank, mm_stay_bank
.import mm_read_zp1, mm_read_zp1l, mm_read_zp1h, mm_read_zp2, mm_read_zp2l, mm_read_zp2h
.import mm_store_zp1, mm_store_zp1l, mm_store_zp1h, mm_store_zp2, mm_store_zp2l, mm_store_zp2h
.endif

.endif
