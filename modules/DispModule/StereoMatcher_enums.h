
#ifndef _SM_ENUMS_H_
#define _SM_ENUMS_H_

enum SM_MATCHING_ALG {
    SGBM_OPENCV = 0,
    SGBM_CUDA,
    LIBELAS
};

enum SM_BLF_FILTER {
    BLF_ORIGINAL = 0,
    BLF_CUDA,
    BLF_DISABLED
};

enum SM_WLS_FILTER {
    WLS_DISABLED = 0,
    WLS_ENABLED,
    WLS_LRCHECK
};

#endif
