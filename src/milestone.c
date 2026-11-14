/* SPDX-License-Identifier: MIT */

#include "milestone.h"
#include "memory.h"
#include "utils.h"
#include "xnuboot.h"

u64 milestone_page(void)
{
    u64 base = cur_boot_args.phys_base;
    u64 size = cur_boot_args.mem_size;

    if ((boot_args_addr & (MILESTONE_PAGE_SIZE - 1)) || size < MILESTONE_PAGE_SIZE)
        return 0;
    if (base > UINT64_MAX - size || boot_args_addr < base)
        return 0;
    if (boot_args_addr > base + size - MILESTONE_PAGE_SIZE)
        return 0;
    if (cur_boot_args.top_of_kernel_data < boot_args_addr + MILESTONE_PAGE_SIZE)
        return 0;
    return boot_args_addr;
}

void milestone_update(u32 state, u64 detail)
{
    u64 page = milestone_page();
    if (!page)
        return;

    struct milestone_record *record = (void *)(page + MILESTONE_RECORD_OFFSET);
    u64 sequence = record->magic == MILESTONE_MAGIC ? record->sequence + 1 : 1;

    *record = (struct milestone_record){
        .magic = MILESTONE_MAGIC,
        .version = MILESTONE_VERSION,
        .state = state,
        .page_phys = page,
        .detail = detail,
        .sequence = sequence,
        .state_complement = ~(u64)state,
    };
    dc_cvac_range(record, sizeof(*record));
    __asm__ volatile("dsb sy" ::: "memory");
}
