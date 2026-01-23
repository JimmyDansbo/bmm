.ifndef memman_inc
memman_inc=1

.ifndef SKIPIMPORT
.import mm_init, mm_set_isr, mm_clear_isr, mm_alloc, mm_remaining, mm_free, mm_get_next_handle
.import mm_set_next_handle
.endif

.endif