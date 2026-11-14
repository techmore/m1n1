/* SPDX-License-Identifier: MIT */
#ifndef M1N1_MILESTONE_H
#define M1N1_MILESTONE_H

#include "types.h"

#define MILESTONE_PAGE_SIZE SZ_16K
#define MILESTONE_RECORD_OFFSET 0x1000
#define MILESTONE_MAGIC 0x41554f5252414d31ULL
#define MILESTONE_VERSION 1

enum milestone_state {
    MILESTONE_PROXY_PREPARED = 0x01,
    MILESTONE_ASM_ENTRY = 0x10,
    MILESTONE_C_ENTRY = 0x20,
    MILESTONE_KERNEL_PREPARED = 0x30,
    MILESTONE_HANDOFF = 0x40,
};

struct milestone_record {
    u64 magic;
    u32 version;
    u32 state;
    u64 page_phys;
    u64 detail;
    u64 sequence;
    u64 state_complement;
};

u64 milestone_page(void);
void milestone_update(u32 state, u64 detail);

#endif
