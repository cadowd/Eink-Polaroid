
set(srcs
    "src/ST25R3911/rfal_rfst25r3911.c"
    "src/ST25R3911/st25r3911.c"
    "src/ST25R3911/st25r3911_com.c"
    "src/ST25R3911/st25r3911_interrupt.c"
    "src/rfal_analogConfig.c"
    "src/rfal_iso15693_2.c"
    "src/rfal_nfca.c"
    "src/rfal_nfcb.c"
    "src/rfal_nfcf.c"
    "src/rfal_nfcv.c"
    "src/rfal_nfcDep.c"
    "src/rfal_crc.c"
    "src/rfal_dpo.c"
    "src/rfal_isoDep.c"
    "src/rfal_st25tb.c"
    "src/rfal_t1t.c"
    "src/rfal_t2t.c"
    )

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "include" "src/ST25R3911"
                       REQUIRES common)
