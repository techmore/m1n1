/* SPDX-License-Identifier: MIT */
/*
 * Mac mini M4 (J773g, T8132) HDMI: hand the running dcpext0 display pipe to the kernel.
 *
 * On the mini the HDMI port sits behind the display crossbar, which lets any of the three display
 * coprocessors drive the DPTX PHY. The kernel drives HDMI from dcpext0 and its display block
 * dispext0, the pipe the boot framebuffer is expected to run on. iBoot leaves dcpext0 running, its
 * firmware mapped through dart-dcpext0 SID 23 and the boot framebuffer scanned out through
 * dart-dispext0 SID 0 (SID 16 is the display's PIODMA). This file describes that state for the
 * kernel, as the T6041 external processors (kboot_display_handoff.c) and the J713 panel
 * (kboot.c) are described: nothing is restarted or reconfigured.
 *
 * Nothing at all happens (no register access, the device tree byte for byte unchanged) unless
 * chip_id is T8132, the root is apple,j773g and the device tree describes the pipe: alias dcpext0
 * naming an apple,t8132-dcpext node. A device tree without that node keeps the boot framebuffer
 * only, as before.
 *
 * Otherwise every check below runs, reading the ADT first, then (only with the dispext0 power
 * domain on) the coprocessor and DART registers and the DART page tables, guarded:
 *  - the device tree: dcpext0, its mailbox, piodma child, both DARTs and the display node (alias
 *    disp0, apple,display-subsystem) are disabled, sit at the J773g addresses, use SIDs 23, 0 and
 *    16, and carry no memory description yet;
 *  - the OS firmware is 26.6.2, the running dcpext0 image is the pinned 26.6.2 image, the ADT says
 *    it is pre-loaded and running, and its four firmware segments (__TEXT, __DATA, __OS_LOG and a
 *    heap) and /vram lie in DRAM, outside the RAM handed to the OS and the memory carveouts;
 *  - dcpext0's CPU runs, and the three streams translate through two-level tables inside their ADT
 *    pt-region;
 *  - SID 23 maps __TEXT, __DATA and the heap in full at their ADT DVAs (and possibly /vram);
 *    SID 0 and SID 16 map only the heap (in full) and /vram, and SID 0 maps the boot framebuffer.
 *    No stream maps the __OS_LOG buffer or anything else.
 *
 * With every check passing, in one transaction on a copy of the device tree:
 *  - /reserved-memory nodes (no-map) for __TEXT, __DATA and the heap (apple,asc-mem), /vram
 *    (compatible framebuffer) and the __OS_LOG buffer (apple,dcp-os-log, no DART mapping), with
 *    iommu-addresses for exactly the mappings found in the page tables, in the 1 TiB form of the
 *    DARTs' apple,dma-range;
 *  - memory-region(-names) on dcpext0, the display and the piodma child;
 *  - apple,firmware-version, apple,firmware-compat and apple,firmware-uuid on dcpext0, and the
 *    marker apple,j773g-hdmi-handoff;
 *  - status "okay" on dcpext0, its mailbox, dart-dcpext0, dart-dispext0 and the display.
 * dart-dcpext0 and dart-dispext0 are then locked (PROTECT TTBR/TCR) and the copy replaces the
 * device tree. The kernel refuses these DARTs unless they are locked, so a locked DART is part of
 * the contract, not an option.
 *
 * Fail-soft: dt_set_j773g_hdmi_handoff() always returns 0. A refused handoff changes no register
 * and publishes nothing but /chosen/aurora,j773g-hdmi-handoff, the step that refused; the Mac
 * keeps the boot framebuffer.
 */

#include "kboot.h"
#include "adt.h"
#include "exception.h"
#include "firmware.h"
#include "malloc.h"
#include "mcc.h"
#include "pmgr.h"
#include "soc.h"
#include "string.h"
#include "utils.h"

#include "libfdt/libfdt.h"

int dt_set_j773g_hdmi_handoff(void *dt);

#define jlog(fmt, ...) printf("DCP: J773g HDMI: " fmt, ##__VA_ARGS__)

#define bail(...)                                                                                  \
    do {                                                                                           \
        jlog(__VA_ARGS__);                                                                         \
        return -1;                                                                                 \
    } while (0)

/* The 26.6.2 T8132 DCP image (iop-dcpext0-nub uuid); the kernel accepts only this one. */
#define J773G_UUID   "81BAF476-679D-3CC0-8C6A-E6B6FA659D5E"
#define J773G_MARKER "apple,j773g-hdmi-handoff"
#define J773G_RESULT "aurora,j773g-hdmi-handoff"
#define J773G_OSLOG  "apple,dcp-os-log"

#define ADT_DCPEXT0   "/arm-io/dcpext0"
#define ADT_NUB       "/arm-io/dcpext0/iop-dcpext0-nub"
#define ADT_DART_DCP  "/arm-io/dart-dcpext0"
#define ADT_DART_DISP "/arm-io/dart-dispext0"

/* J773g ADT reg[0] of dcpext0, dart-dcpext0 and dart-dispext0, after the arm-io ranges */
#define J773G_COPROC    0x286e00000UL
#define J773G_MBOX      0x286e08000UL
#define J773G_DART_DCP  0x286340000UL
#define J773G_DART_DISP 0x286300000UL

#define J773G_SID_DCP    23
#define J773G_SID_DISP   0
#define J773G_SID_PIODMA 16
#define J773G_STREAMS    32

/* Both DARTs translate one 36-bit DVA window at 1 TiB (ADT vm-base, vm-size(-23)) */
#define J773G_VM_BASE 0x10000000000UL
#define J773G_VM_SIZE 0x1000000000UL

#define ASC_CPU_CONTROL     0x44
#define ASC_CPU_STATUS      0x48
#define ASC_CPU_CONTROL_RUN BIT(4)

#define PMGR_AUTO_ENABLE BIT(28)
#define PMGR_PS_ACTUAL   GENMASK(7, 4)
#define PMGR_PS_TARGET   GENMASK(3, 0)

#define DART_T8110_PROTECT          0x200
#define DART_T8110_PROTECT_TTBR_TCR BIT(0)
#define DART_T8110_TCR(sid)         (0x1000 + 4 * (sid))
#define DART_T8110_TCR_TRANSLATE    BIT(0)
#define DART_T8110_TCR_BYPASS_DART  BIT(1)
#define DART_T8110_TCR_FOUR_LEVEL   BIT(3)
#define DART_T8110_TTBR(sid)        (0x1400 + 4 * (sid))
#define DART_T8110_TTBR_VALID       BIT(0)
#define DART_T8110_TTBR_ADDR        GENMASK(29, 2)
#define DART_T8110_TTBR_SHIFT       14

#define DART_PTE_VALID   BIT(0)
#define DART_PTE_ADDR    GENMASK(39, 10)
#define DART_PTE_SHIFT   4
#define DART_PAGE        SZ_16K
#define DART_ENTRIES     2048
#define DART_L1_SHIFT    25
#define DART_L2_SHIFT    14
#define DART_DVA_MASK    ((1UL << 36) - 1)
#define DART_PT_MAX_SIZE SZ_1M

#define SEG_MAX_SIZE   (256 * SZ_1M)
#define OSLOG_MAX_SIZE SZ_1M
#define MAX_RUNS       16
#define MAX_GATES      8
#define PATH_LEN       96

#define SEG_FLAGS_DATA  0
#define SEG_FLAGS_TEXT  1
#define SEG_FLAGS_HEAP  2
#define SEG_FLAGS_OSLOG 0xa

/* The iop-dcpext0-nub segments, in ADT order, then /vram */
enum { R_TEXT, R_DATA, R_OSLOG, R_HEAP, NSEGS, R_VRAM = NSEGS, NREGIONS };

static const struct {
    u32 flags;
    const char *adt_name; /* segment-names entry, or NULL: not checked */
    const char *node;     /* /reserved-memory node name, before the unit address */
    const char *name;     /* memory-region-names entry */
    const char *compat;
} j773g_regions[NREGIONS] = {
    {SEG_FLAGS_TEXT, "__TEXT", "dcpext0-text", "text", "apple,asc-mem"},
    {SEG_FLAGS_DATA, "__DATA", "dcpext0-data", "data", "apple,asc-mem"},
    {SEG_FLAGS_OSLOG, "__OS_LOG", "dcp-oslog", "os_log", NULL},
    {SEG_FLAGS_HEAP, NULL, "dcpext0-heap", "heap", "apple,asc-mem"},
    {0, NULL, "framebuffer", "framebuffer", "framebuffer"},
};

enum { S_DCP, S_DISP, S_PIODMA, NSIDS };

struct j773g_region {
    u64 phys, size, remap;
};

struct j773g_run {
    u64 dva, phys, size;
    int region;
};

struct j773g_sid {
    const char *what;
    u8 sid;
    u64 dart;
    const char *adt_dart;
    int nruns;
    struct j773g_run runs[MAX_RUNS];
};

/* Device tree paths of the nodes the handoff checks and edits (offsets move with every edit) */
struct j773g_dt {
    char dcp[PATH_LEN], disp[PATH_LEN], piodma[PATH_LEN], mbox[PATH_LEN];
    char dart_dcp[PATH_LEN], dart_disp[PATH_LEN];
};

/* Per-call state, reset on every call */
static struct {
    const char *step; /* what the handoff is doing, for the result in /chosen */
    u64 dram_base, dram_size;
    struct j773g_region r[NREGIONS];
    struct j773g_sid s[NSIDS];
    struct j773g_dt n;
} j773g;

/* ---- small helpers ---- */

static bool overlaps(u64 a, u64 a_size, u64 b, u64 b_size)
{
    u64 a_end = a + a_size < a ? ~0UL : a + a_size;
    u64 b_end = b + b_size < b ? ~0UL : b + b_size;

    return a_size && b_size && a < b_end && b < a_end;
}

static bool contains(u64 a, u64 a_size, u64 b, u64 b_size)
{
    return b >= a && b_size <= a_size && b - a <= a_size - b_size;
}

static int adt_u32(int node, const char *name, u32 *val)
{
    u32 len = 0;
    const void *p = adt_getprop(adt, node, name, &len);
    if (!p || len != sizeof(*val))
        return -1;
    memcpy(val, p, sizeof(*val));
    return 0;
}

static int adt_u64(int node, const char *name, u64 *val)
{
    u32 len = 0;
    const void *p = adt_getprop(adt, node, name, &len);
    if (!p || len != sizeof(*val))
        return -1;
    memcpy(val, p, sizeof(*val));
    return 0;
}

static int adt_u64_pair(int node, const char *name, u64 *a, u64 *b)
{
    u64 v[2];
    u32 len = 0;
    const void *p = adt_getprop(adt, node, name, &len);
    if (!p || len != sizeof(v))
        return -1;
    memcpy(v, p, sizeof(v));
    *a = v[0];
    *b = v[1];
    return 0;
}

/* reg[idx] of an ADT node, with the /arm-io ranges applied */
static int adt_reg(const char *path, int idx, u64 *addr, u64 *size)
{
    int adt_path[8];
    int node = adt_path_offset_trace(adt, path, adt_path);
    if (node < 0)
        return -1;
    return adt_get_reg(adt, adt_path, "reg", idx, addr, size);
}

/* Name number idx in a ';'-separated ADT segment-names list */
static bool seg_name_is(const char *names, u32 len, int idx, const char *want)
{
    u32 i = 0;
    while (idx--) {
        while (i < len && names[i] && names[i] != ';')
            i++;
        if (i >= len || names[i] != ';')
            return false;
        i++;
    }
    size_t n = strlen(want);
    return i + n <= len && !memcmp(names + i, want, n) &&
           (i + n == len || names[i + n] == ';' || names[i + n] == 0);
}

static int guard_begin(void)
{
    int before = exc_count;
    exc_guard = GUARD_MARK | GUARD_SILENT;
    return before;
}

/* Returns -1 if a synchronous abort or an SError was taken since guard_begin() */
static int guard_end(int before)
{
    sysop("dsb sy");
    sysop("isb");
    exc_guard = GUARD_OFF;
    return exc_count == before ? 0 : -1;
}

static bool usable_ram(u64 base, u64 size)
{
    return overlaps(base, size, kboot_ram_base(), kboot_ram_size());
}

static int carveout_conflict(u64 base, u64 size, const char *what)
{
    for (size_t i = 0; i < mcc_carveout_count; i++)
        if (overlaps(base, size, mcc_carveouts[i].base, mcc_carveouts[i].size))
            bail("%s 0x%lx+0x%lx overlaps memory carveout 0x%lx+0x%lx\n", what, base, size,
                 mcc_carveouts[i].base, mcc_carveouts[i].size);
    return 0;
}

/* ---- the device tree ---- */

static bool fdt_status_is(void *dt, int node, const char *want)
{
    int len = 0;
    const char *status = fdt_getprop(dt, node, "status", &len);
    return status && len == (int)strlen(want) + 1 && !memcmp(status, want, len);
}

static int fdt_reg0(void *dt, int node, u64 *addr)
{
    int len = 0;
    const fdt64_t *reg = fdt_getprop(dt, node, "reg", &len);
    if (!reg || len < 2 * (int)sizeof(fdt64_t))
        return -1;
    *addr = fdt64_ld(reg);
    return 0;
}

static int fdt_u32(void *dt, int node, const char *name, u32 *val)
{
    int len = 0;
    const fdt32_t *p = fdt_getprop(dt, node, name, &len);
    if (!p || len != sizeof(*p))
        return -1;
    *val = fdt32_ld(p);
    return 0;
}

/* The single <&dart sid> of node's iommus; returns the DART's offset */
static int fdt_iommu(void *dt, int node, u32 *sid)
{
    int len = 0;
    const fdt32_t *iommus = fdt_getprop(dt, node, "iommus", &len);
    if (!iommus || len != 2 * (int)sizeof(fdt32_t))
        return -1;
    *sid = fdt32_ld(&iommus[1]);
    return fdt_node_offset_by_phandle(dt, fdt32_ld(&iommus[0]));
}

static int fdt_path_of(void *dt, int node, char *buf, const char *what)
{
    if (node < 0 || fdt_get_path(dt, node, buf, PATH_LEN))
        bail("device tree: no path for %s\n", what);
    return 0;
}

/* A display DART node: apple,t8110-dart at the J773g address, disabled, with the 1 TiB window */
static int check_dart_node(void *dt, int node, u64 addr, const char *what)
{
    u64 reg;
    int len = 0;
    const fdt64_t *range = node < 0 ? NULL : fdt_getprop(dt, node, "apple,dma-range", &len);

    if (node < 0 || fdt_node_check_compatible(dt, node, "apple,t8110-dart") ||
        fdt_reg0(dt, node, &reg) || reg != addr)
        bail("device tree: %s is not the apple,t8110-dart at 0x%lx\n", what, addr);
    if (!fdt_status_is(dt, node, "disabled"))
        bail("device tree: %s is not status \"disabled\"\n", what);
    if (!range || len != 2 * (int)sizeof(fdt64_t) || fdt64_ld(&range[0]) != J773G_VM_BASE ||
        fdt64_ld(&range[1]) != J773G_VM_SIZE)
        bail("device tree: %s apple,dma-range is not <0x%lx 0x%lx>\n", what, J773G_VM_BASE,
             J773G_VM_SIZE);
    return 0;
}

static int check_no_memory(void *dt, int node, const char *what)
{
    if (fdt_getprop(dt, node, "memory-region", NULL) ||
        fdt_getprop(dt, node, "memory-region-names", NULL))
        bail("device tree: %s already has a memory-region\n", what);
    return 0;
}

static int check_dt(void *dt, int dcp)
{
    struct j773g_dt *n = &j773g.n;
    u64 reg;
    u32 sid, cells;

    j773g.step = "device tree";
    if (!fdt_status_is(dt, dcp, "disabled"))
        bail("device tree: dcpext0 is not status \"disabled\"\n");
    if (fdt_reg0(dt, dcp, &reg) || reg != J773G_COPROC)
        bail("device tree: dcpext0 reg is not 0x%lx\n", J773G_COPROC);
    /* piodma's iommu-addresses are read with dcpext0's cells */
    if (fdt_u32(dt, dcp, "#address-cells", &cells) || cells != 2 ||
        fdt_u32(dt, dcp, "#size-cells", &cells) || cells != 2)
        bail("device tree: dcpext0 does not have #address-cells = #size-cells = 2\n");
    if (check_no_memory(dt, dcp, "dcpext0") || fdt_path_of(dt, dcp, n->dcp, "dcpext0"))
        return -1;
    if (fdt_getprop(dt, dcp, "apple,firmware-uuid", NULL) ||
        fdt_getprop(dt, dcp, "apple,firmware-version", NULL))
        bail("device tree: dcpext0 already has firmware properties\n");

    int dart_dcp = fdt_iommu(dt, dcp, &sid);
    if (dart_dcp < 0 || sid != J773G_SID_DCP)
        bail("device tree: dcpext0 iommus is not <&dart %d>\n", J773G_SID_DCP);
    if (check_dart_node(dt, dart_dcp, J773G_DART_DCP, "dart-dcpext0") ||
        fdt_path_of(dt, dart_dcp, n->dart_dcp, "dart-dcpext0"))
        return -1;

    int len = 0;
    const fdt32_t *mboxes = fdt_getprop(dt, dcp, "mboxes", &len);
    int mbox =
        mboxes && len == sizeof(fdt32_t) ? fdt_node_offset_by_phandle(dt, fdt32_ld(mboxes)) : -1;
    if (mbox < 0 || fdt_node_check_compatible(dt, mbox, "apple,asc-mailbox-v4") ||
        fdt_reg0(dt, mbox, &reg) || reg != J773G_MBOX || !fdt_status_is(dt, mbox, "disabled"))
        bail("device tree: dcpext0 mailbox missing, not at 0x%lx or not disabled\n", J773G_MBOX);
    if (fdt_path_of(dt, mbox, n->mbox, "the mailbox"))
        return -1;

    int piodma = fdt_subnode_offset(dt, dcp, "piodma");
    if (piodma < 0)
        bail("device tree: dcpext0 has no piodma child\n");
    int dart_disp = fdt_iommu(dt, piodma, &sid);
    if (dart_disp < 0 || sid != J773G_SID_PIODMA)
        bail("device tree: piodma iommus is not <&dart %d>\n", J773G_SID_PIODMA);
    if (check_no_memory(dt, piodma, "piodma") || fdt_path_of(dt, piodma, n->piodma, "piodma"))
        return -1;

    const char *disp_path = fdt_get_alias(dt, "disp0");
    int disp = disp_path ? fdt_path_offset(dt, disp_path) : -1;
    if (disp < 0 || fdt_node_check_compatible(dt, disp, "apple,display-subsystem"))
        bail("device tree: alias disp0 is not an apple,display-subsystem node\n");
    if (!fdt_status_is(dt, disp, "disabled"))
        bail("device tree: the display is not status \"disabled\"\n");
    if (check_no_memory(dt, disp, "the display") || fdt_path_of(dt, disp, n->disp, "the display"))
        return -1;
    int dart_disp2 = fdt_iommu(dt, disp, &sid);
    if (dart_disp2 != dart_disp || sid != J773G_SID_DISP)
        bail("device tree: display iommus is not <&dart-dispext0 %d>\n", J773G_SID_DISP);
    if (check_dart_node(dt, dart_disp, J773G_DART_DISP, "dart-dispext0") ||
        fdt_path_of(dt, dart_disp, n->dart_disp, "dart-dispext0"))
        return -1;

    int resv = fdt_path_offset(dt, "/reserved-memory");
    if (resv < 0 || fdt_u32(dt, resv, "#address-cells", &cells) || cells != 2 ||
        fdt_u32(dt, resv, "#size-cells", &cells) || cells != 2)
        bail("device tree: no /reserved-memory with 2+2 cells\n");
    return 0;
}

/* ---- the ADT ---- */

static int check_adt_dart(const char *path, u64 addr, u8 sid)
{
    char prop[24];
    u64 reg, vm_base, vm_size;
    u32 count;
    int node = adt_path_offset(adt, path);

    if (node < 0 || !adt_is_compatible(adt, node, "dart,t8110") || adt_reg(path, 0, &reg, NULL) ||
        reg != addr)
        bail("ADT %s is not the dart,t8110 at 0x%lx\n", path, addr);
    snprintf(prop, sizeof(prop), "vm-size-%d", sid);
    if (adt_u64(node, "vm-base", &vm_base) ||
        (adt_u64(node, prop, &vm_size) && adt_u64(node, "vm-size", &vm_size)) ||
        vm_base != J773G_VM_BASE || vm_size != J773G_VM_SIZE)
        bail("ADT %s vm-base/vm-size is not 0x%lx/0x%lx\n", path, J773G_VM_BASE, J773G_VM_SIZE);
    if (adt_u32(node, "sid-count", &count) || count != J773G_STREAMS)
        bail("ADT %s sid-count is not %d\n", path, J773G_STREAMS);
    return 0;
}

static int check_region(int i, const char *what)
{
    const struct j773g_region *r = &j773g.r[i];
    u64 dram_end = j773g.dram_base + j773g.dram_size;

    if (!r->size || r->size > SEG_MAX_SIZE || (r->phys & (SZ_16K - 1)) ||
        r->phys < j773g.dram_base || r->phys > dram_end || r->size > dram_end - r->phys)
        bail("%s 0x%lx+0x%lx is not a 16K-aligned DRAM range\n", what, r->phys, r->size);
    if (carveout_conflict(r->phys, r->size, what))
        return -1;
    if (i == R_OSLOG) {
        /* A physical buffer: the reservation keeps the OS off it, wherever it is */
        if (r->remap != r->phys || r->size > OSLOG_MAX_SIZE)
            bail("__OS_LOG 0x%lx+0x%lx is not a physical log buffer\n", r->phys, r->size);
        return 0;
    }
    if (usable_ram(r->phys, r->size))
        bail("%s 0x%lx+0x%lx lies in the RAM handed to the OS\n", what, r->phys, r->size);
    if (i == R_VRAM)
        return 0;
    if ((r->remap & (SZ_16K - 1)) || r->remap < J773G_VM_BASE ||
        r->remap - J773G_VM_BASE > J773G_VM_SIZE - r->size)
        bail("%s remap 0x%lx is not in the DART window\n", what, r->remap);
    return 0;
}

static int check_adt(void)
{
    j773g.step = "firmware";
    if (os_firmware.version != V26_6_2)
        bail("OS firmware %s is not 26.6.2\n", os_firmware.string ? os_firmware.string : "?");

    j773g.step = "ADT";
    u64 coproc;
    if (adt_reg(ADT_DCPEXT0, 0, &coproc, NULL) || coproc != J773G_COPROC)
        bail("ADT %s reg is not 0x%lx\n", ADT_DCPEXT0, J773G_COPROC);
    if (check_adt_dart(ADT_DART_DCP, J773G_DART_DCP, J773G_SID_DCP) ||
        check_adt_dart(ADT_DART_DISP, J773G_DART_DISP, J773G_SID_DISP))
        return -1;

    int chosen = adt_path_offset(adt, "/chosen");
    if (chosen < 0 || adt_u64(chosen, "dram-base", &j773g.dram_base) ||
        adt_u64(chosen, "dram-size", &j773g.dram_size) || !j773g.dram_size ||
        j773g.dram_base + j773g.dram_size <= j773g.dram_base)
        bail("no usable /chosen dram-base/dram-size\n");

    int nub = adt_path_offset(adt, ADT_NUB);
    if (nub < 0)
        bail("no %s\n", ADT_NUB);
    u32 len = 0, preloaded, running;
    const char *uuid = adt_getprop(adt, nub, "uuid", &len);
    if (!uuid || len != sizeof(J773G_UUID) || memcmp(uuid, J773G_UUID, sizeof(J773G_UUID)))
        bail("dcpext0 runs another firmware image than %s\n", J773G_UUID);
    if (adt_u32(nub, "pre-loaded", &preloaded) || preloaded != 1 ||
        adt_u32(nub, "running", &running) || running != 1)
        bail("iBoot did not leave the dcpext0 firmware loaded and running\n");

    u32 names_len = 0;
    const struct adt_segment_ranges *seg = adt_getprop(adt, nub, "segment-ranges", &len);
    const char *names = adt_getprop(adt, nub, "segment-names", &names_len);
    if (!seg || len != NSEGS * sizeof(*seg) || !names)
        bail("%s segment-ranges is not %d segments\n", ADT_NUB, NSEGS);
    for (int i = 0; i < NSEGS; i++) {
        struct adt_segment_ranges s;
        memcpy(&s, &seg[i], sizeof(s));
        if (s.unk != j773g_regions[i].flags ||
            (j773g_regions[i].adt_name &&
             !seg_name_is(names, names_len, i, j773g_regions[i].adt_name)))
            bail("dcpext0 segment %d is not %s (flags 0x%x)\n", i, j773g_regions[i].name, s.unk);
        j773g.r[i] = (struct j773g_region){s.phys, ALIGN_UP((u64)s.size, SZ_16K), s.remap};
        if (!s.size || check_region(i, j773g_regions[i].name))
            return -1;
    }

    u64 vram, vram_size;
    if (adt_reg("/vram", 0, &vram, &vram_size))
        bail("no /vram\n");
    j773g.r[R_VRAM] = (struct j773g_region){vram, ALIGN_UP(vram_size, SZ_16K), 0};
    if (check_region(R_VRAM, "/vram"))
        return -1;
    if (!contains(vram, j773g.r[R_VRAM].size, kboot_fb_base(), 1))
        bail("the boot framebuffer 0x%lx is not in /vram 0x%lx+0x%lx\n", kboot_fb_base(), vram,
             j773g.r[R_VRAM].size);

    for (int i = 0; i < NREGIONS; i++)
        for (int k = 0; k < i; k++) {
            if (overlaps(j773g.r[i].phys, j773g.r[i].size, j773g.r[k].phys, j773g.r[k].size))
                bail("%s and %s overlap\n", j773g_regions[i].name, j773g_regions[k].name);
            if (i < NSEGS && i != R_OSLOG && k != R_OSLOG &&
                overlaps(j773g.r[i].remap, j773g.r[i].size, j773g.r[k].remap, j773g.r[k].size))
                bail("%s and %s overlap in the DART\n", j773g_regions[i].name,
                     j773g_regions[k].name);
        }
    return 0;
}

/* ---- the hardware, read only ---- */

/* A power state is on: actually on, or targeted on with automatic power management */
static bool ps_on(u32 ps)
{
    return FIELD_GET(PMGR_PS_ACTUAL, ps) == PMGR_PS_ACTIVE ||
           (FIELD_GET(PMGR_PS_TARGET, ps) == PMGR_PS_ACTIVE && (ps & PMGR_AUTO_ENABLE));
}

/* g's power state register, or, for a virtual device such as DISPEXT0_CPU, its parents' */
static int ps_add(u64 *regs, int *n, const struct pmgr_gate *g, int depth)
{
    if (!g->found)
        return -1;
    if (!g->virt) {
        if (!g->psreg)
            return -1;
        for (int i = 0; i < *n; i++)
            if (regs[i] == g->psreg)
                return 0;
        if (*n >= MAX_GATES)
            return -1;
        regs[(*n)++] = g->psreg;
        return 0;
    }
    if (depth >= MAX_GATES)
        return -1;
    for (int k = 0; k < 2; k++) {
        struct pmgr_gate p;
        if (!g->parent[k])
            continue;
        if (pmgr_device_gate(g->die, g->parent[k], &p) < 0 || ps_add(regs, n, &p, depth + 1))
            return -1;
    }
    return 0;
}

/* dcpext0's ADT clock-gates, resolved through the PMGR table, must all be on */
static int check_power(void)
{
    u64 regs[MAX_GATES];
    int n = 0;

    j773g.step = "power";
    for (u32 i = 0; i < MAX_GATES; i++) {
        struct pmgr_gate g;
        if (pmgr_adt_gate(ADT_DCPEXT0, "clock-gates", i, &g) < 0)
            break;
        if (ps_add(regs, &n, &g, 0))
            bail("%s clock-gates[%u] does not resolve to PMGR power states\n", ADT_DCPEXT0, i);
    }
    if (!n)
        bail("no %s clock-gate has a PMGR power state\n", ADT_DCPEXT0);

    bool on = true;
    for (int i = 0; i < n; i++) {
        int g = guard_begin();
        u32 ps = read32(regs[i]);
        if (guard_end(g))
            bail("reading the dispext0 power state faulted\n");
        if (!ps_on(ps)) {
            jlog("power state 0x%lx = 0x%x is not on\n", regs[i], ps);
            on = false;
        }
    }
    if (!on)
        bail("dispext0 is not powered: iBoot did not light HDMI through dcpext0\n");
    return 0;
}

static int check_running(void)
{
    j773g.step = "coprocessor";
    int g = guard_begin();
    u32 ctl = read32(J773G_COPROC + ASC_CPU_CONTROL);
    u32 prot_dcp = read32(J773G_DART_DCP + DART_T8110_PROTECT);
    u32 prot_disp = read32(J773G_DART_DISP + DART_T8110_PROTECT);
    if (guard_end(g))
        bail("reading the dcpext0/DART registers faulted\n");
    jlog("dcpext0 CPU_CONTROL 0x%x; PROTECT dart-dcpext0 0x%x dart-dispext0 0x%x\n", ctl, prot_dcp,
         prot_disp);
    if (!(ctl & ASC_CPU_CONTROL_RUN))
        bail("dcpext0 is not running\n");
    return 0;
}

/* The region a physical address lies in, or -1 */
static int region_of(u64 pa)
{
    for (int i = 0; i < NREGIONS; i++)
        if (pa >= j773g.r[i].phys && pa - j773g.r[i].phys < j773g.r[i].size)
            return i;
    return -1;
}

/* Record one valid PTE. Runs are contiguous in DVA and PA, and never span two regions. */
static int run_add(struct j773g_sid *s, u64 dva, u64 pa)
{
    int region = region_of(pa);
    bool allowed = region == R_HEAP || region == R_VRAM ||
                   (s == &j773g.s[S_DCP] && (region == R_TEXT || region == R_DATA));
    if (!allowed)
        bail("%s SID %d maps DVA 0x%lx to 0x%lx, %s\n", s->what, s->sid, dva, pa,
             region < 0          ? "outside the firmware segments and /vram"
             : region == R_OSLOG ? "the __OS_LOG buffer"
                                 : "a segment it must not map");

    if (s->nruns) {
        struct j773g_run *r = &s->runs[s->nruns - 1];
        if (r->region == region && r->dva + r->size == dva && r->phys + r->size == pa) {
            r->size += DART_PAGE;
            return 0;
        }
    }
    if (s->nruns >= MAX_RUNS)
        bail("%s SID %d has too many mappings\n", s->what, s->sid);
    s->runs[s->nruns++] = (struct j773g_run){dva, pa, DART_PAGE, region};
    return 0;
}

static int walk_inner(struct j773g_sid *s, u64 pt_lo, u64 pt_hi)
{
    u32 tcr = read32(s->dart + DART_T8110_TCR(s->sid));
    u32 ttbr = read32(s->dart + DART_T8110_TTBR(s->sid));

    if (!(tcr & DART_T8110_TCR_TRANSLATE) ||
        (tcr & (DART_T8110_TCR_BYPASS_DART | DART_T8110_TCR_FOUR_LEVEL)) ||
        !(ttbr & DART_T8110_TTBR_VALID))
        bail("%s SID %d: TCR 0x%x TTBR 0x%x is not a live two-level translation\n", s->what, s->sid,
             tcr, ttbr);

    u64 l1 = (u64)FIELD_GET(DART_T8110_TTBR_ADDR, ttbr) << DART_T8110_TTBR_SHIFT;
    if (l1 < pt_lo || l1 + DART_PAGE > pt_hi)
        bail("%s SID %d: L1 table 0x%lx outside pt-region 0x%lx..0x%lx\n", s->what, s->sid, l1,
             pt_lo, pt_hi);

    for (u64 i = 0; i < DART_ENTRIES; i++) {
        u64 e1 = read64(l1 + 8 * i);
        if (!(e1 & DART_PTE_VALID))
            continue;
        u64 l2 = (e1 & DART_PTE_ADDR) << DART_PTE_SHIFT;
        if (l2 < pt_lo || l2 + DART_PAGE > pt_hi)
            bail("%s SID %d: L2 table 0x%lx outside pt-region\n", s->what, s->sid, l2);
        for (u64 k = 0; k < DART_ENTRIES; k++) {
            u64 e2 = read64(l2 + 8 * k);
            if (!(e2 & DART_PTE_VALID))
                continue;
            if (run_add(s, (i << DART_L1_SHIFT) | (k << DART_L2_SHIFT),
                        (e2 & DART_PTE_ADDR) << DART_PTE_SHIFT))
                return -1;
        }
    }
    return 0;
}

/*
 * Walk one stream's page tables, read only. A table is only read after its address was found
 * inside the stream's ADT pt-region, a 16K-aligned DRAM range outside the OS RAM and carveouts.
 */
static int walk(struct j773g_sid *s)
{
    char prop[24];
    u64 pt_lo, pt_hi;
    int node = adt_path_offset(adt, s->adt_dart);

    s->nruns = 0;
    snprintf(prop, sizeof(prop), "pt-region-%d", s->sid);
    if (node < 0 || adt_u64_pair(node, prop, &pt_lo, &pt_hi))
        bail("%s: no %s/%s\n", s->what, s->adt_dart, prop);
    if (pt_hi <= pt_lo || pt_hi - pt_lo > DART_PT_MAX_SIZE || ((pt_lo | pt_hi) & (SZ_16K - 1)) ||
        pt_lo < j773g.dram_base || pt_hi > j773g.dram_base + j773g.dram_size ||
        usable_ram(pt_lo, pt_hi - pt_lo))
        bail("%s: bad %s 0x%lx..0x%lx\n", s->what, prop, pt_lo, pt_hi);
    if (carveout_conflict(pt_lo, pt_hi - pt_lo, prop))
        return -1;

    int g = guard_begin();
    int ret = walk_inner(s, pt_lo, pt_hi);
    if (guard_end(g))
        bail("%s SID %d: reading the page tables faulted\n", s->what, s->sid);
    return ret;
}

/*
 * dcpext0 (SID 23) maps __TEXT, __DATA and the heap each once, in full, at its ADT DVA, and may
 * map /vram. The display (SID 0) and piodma (SID 16) map at most the whole heap once, and /vram;
 * the display maps the boot framebuffer.
 */
static int check_runs(struct j773g_sid *s)
{
    int seen[NREGIONS] = {0};

    for (int i = 0; i < s->nruns; i++) {
        const struct j773g_run *r = &s->runs[i];
        const struct j773g_region *g = &j773g.r[r->region];
        if (r->region == R_VRAM)
            continue;
        if (seen[r->region]++)
            bail("%s SID %d maps %s twice\n", s->what, s->sid, j773g_regions[r->region].name);
        if (r->phys != g->phys || r->size != g->size)
            bail("%s SID %d maps 0x%lx+0x%lx of %s (0x%lx+0x%lx)\n", s->what, s->sid, r->phys,
                 r->size, j773g_regions[r->region].name, g->phys, g->size);
        if (s == &j773g.s[S_DCP] && r->dva != (g->remap & DART_DVA_MASK))
            bail("dcpext0 SID %d maps %s at DVA 0x%lx, the ADT says 0x%lx\n", s->sid,
                 j773g_regions[r->region].name, r->dva, g->remap & DART_DVA_MASK);
    }

    if (s == &j773g.s[S_DCP]) {
        static const int need[] = {R_TEXT, R_DATA, R_HEAP};
        for (size_t i = 0; i < ARRAY_SIZE(need); i++)
            if (!seen[need[i]])
                bail("dcpext0 SID %d does not map %s\n", s->sid, j773g_regions[need[i]].name);
    } else if (s == &j773g.s[S_DISP]) {
        u64 fb = kboot_fb_base() & ~(u64)(DART_PAGE - 1);
        bool fb_mapped = false;
        for (int i = 0; i < s->nruns; i++)
            fb_mapped |= contains(s->runs[i].phys, s->runs[i].size, fb, DART_PAGE);
        if (!fb_mapped)
            bail("display SID %d does not scan out the boot framebuffer 0x%lx: iBoot does not "
                 "drive HDMI through dispext0\n",
                 s->sid, fb);
    }
    return 0;
}

static int check_streams(void)
{
    static const struct {
        const char *what;
        u8 sid;
        u64 dart;
        const char *adt_dart;
    } sids[NSIDS] = {
        {"dcpext0", J773G_SID_DCP, J773G_DART_DCP, ADT_DART_DCP},
        {"display", J773G_SID_DISP, J773G_DART_DISP, ADT_DART_DISP},
        {"piodma", J773G_SID_PIODMA, J773G_DART_DISP, ADT_DART_DISP},
    };

    j773g.step = "DART mappings";
    for (int i = 0; i < NSIDS; i++) {
        struct j773g_sid *s = &j773g.s[i];
        s->what = sids[i].what;
        s->sid = sids[i].sid;
        s->dart = sids[i].dart;
        s->adt_dart = sids[i].adt_dart;
        if (walk(s) || check_runs(s))
            return -1;
        jlog("%s SID %d: %d mapping(s), all in the firmware segments and /vram\n", s->what, s->sid,
             s->nruns);
    }
    return 0;
}

/* ---- publishing, on a copy of the device tree ---- */

/* Whether [base, base+size) overlaps an enabled /reserved-memory node or a /memreserve/ entry */
static int resv_conflict(void *dt, u64 base, u64 size, const char *what, bool memreserve_ok)
{
    int n = fdt_num_mem_rsv(dt);
    for (int i = 0; i < n; i++) {
        u64 a, s;
        if (fdt_get_mem_rsv(dt, i, &a, &s) || !overlaps(base, size, a, s))
            continue;
        /* a log buffer that a /memreserve/ already keeps whole from the OS is no conflict */
        if (!memreserve_ok || !contains(a, s, base, size))
            bail("%s 0x%lx+0x%lx overlaps /memreserve/ 0x%lx+0x%lx\n", what, base, size, a, s);
    }

    int resv = fdt_path_offset(dt, "/reserved-memory");
    int node;
    if (resv < 0)
        return 0;
    fdt_for_each_subnode(node, dt, resv)
    {
        if (fdt_status_is(dt, node, "disabled"))
            continue;
        int len = 0;
        const fdt64_t *reg = fdt_getprop(dt, node, "reg", &len);
        for (int k = 0; reg && (k + 2) * (int)sizeof(fdt64_t) <= len; k += 2)
            if (overlaps(base, size, fdt64_ld(&reg[k]), fdt64_ld(&reg[k + 1]))) {
                const char *name = fdt_get_name(dt, node, NULL);
                bail("%s 0x%lx+0x%lx overlaps /reserved-memory/%s\n", what, base, size,
                     name ? name : "?");
            }
    }
    return 0;
}

/* Add /reserved-memory/<name> (reg, no-map, phandle, compatible). Returns its phandle or 0. */
static u32 add_resv(void *dt, const char *name, u64 base, u64 size, const char *compat)
{
    fdt64_t reg[2];
    u32 phandle;
    int resv = fdt_path_offset(dt, "/reserved-memory");
    int node = resv < 0 ? resv : fdt_add_subnode(dt, resv, name);

    if (node < 0)
        return 0;
    fdt64_st(&reg[0], base);
    fdt64_st(&reg[1], size);
    int err = fdt_generate_phandle(dt, &phandle);
    if (!err)
        err = fdt_setprop_u32(dt, node, "phandle", phandle);
    if (!err)
        err = fdt_setprop(dt, node, "reg", reg, sizeof(reg));
    if (!err && compat)
        err = fdt_setprop_string(dt, node, "compatible", compat);
    if (!err)
        err = fdt_setprop_empty(dt, node, "no-map");
    return err ? 0 : phandle;
}

static u32 node_phandle(void *dt, const char *path)
{
    int node = fdt_path_offset(dt, path);
    u32 phandle;

    if (node < 0)
        return 0;
    phandle = fdt_get_phandle(dt, node);
    if (phandle)
        return phandle;
    if (fdt_generate_phandle(dt, &phandle) || fdt_setprop_u32(dt, node, "phandle", phandle))
        return 0;
    return phandle;
}

static int set_status_okay(void *dt, const char *path)
{
    int node = fdt_path_offset(dt, path);
    return node < 0 ? node : fdt_setprop_string(dt, node, "status", "okay");
}

static int publish(void *dt)
{
    const char *owners[NSIDS] = {j773g.n.dcp, j773g.n.disp, j773g.n.piodma};
    /* memory-region order per owner: dcpext0 its firmware first, the display its framebuffer */
    static const int order[NSIDS][4] = {
        {R_TEXT, R_DATA, R_HEAP, R_VRAM},
        {R_VRAM, R_HEAP, -1, -1},
        {R_HEAP, R_VRAM, -1, -1},
    };
    u32 owner_ph[NSIDS], region_ph[NREGIONS];
    bool used[NSIDS][NREGIONS] = {{false}};
    char name[64];

    for (int i = 0; i < NSIDS; i++) {
        owner_ph[i] = node_phandle(dt, owners[i]);
        if (!owner_ph[i])
            bail("cannot give %s a phandle\n", j773g.s[i].what);
        for (int k = 0; k < j773g.s[i].nruns; k++)
            used[i][j773g.s[i].runs[k].region] = true;
    }

    for (int r = 0; r < NREGIONS; r++) {
        const struct j773g_region *g = &j773g.r[r];
        if (resv_conflict(dt, g->phys, g->size, j773g_regions[r].name, r == R_OSLOG))
            return -1;
        snprintf(name, sizeof(name), "%s@%lx", j773g_regions[r].node, g->phys);
        region_ph[r] = add_resv(dt, name, g->phys, g->size, j773g_regions[r].compat);
        if (!region_ph[r])
            bail("cannot add /reserved-memory/%s\n", name);
        if (r == R_OSLOG) {
            int node = fdt_node_offset_by_phandle(dt, region_ph[r]);
            if (node < 0 || fdt_setprop_empty(dt, node, J773G_OSLOG))
                bail("cannot mark /reserved-memory/%s\n", name);
        }
    }

    /* One iommu-addresses entry per mapping found in the page tables */
    for (int i = 0; i < NSIDS; i++)
        for (int k = 0; k < j773g.s[i].nruns; k++) {
            const struct j773g_run *run = &j773g.s[i].runs[k];
            int node = fdt_node_offset_by_phandle(dt, region_ph[run->region]);
            fdt32_t entry[5];
            entry[0] = cpu_to_fdt32(owner_ph[i]);
            fdt64_st(&entry[1], J773G_VM_BASE | run->dva);
            fdt64_st(&entry[3], run->size);
            if (node < 0 || fdt_appendprop(dt, node, "iommu-addresses", entry, sizeof(entry)))
                bail("cannot describe the %s mappings\n", j773g.s[i].what);
        }

    for (int i = 0; i < NSIDS; i++)
        for (int k = 0; k < 4; k++) {
            int r = order[i][k];
            if (r < 0 || !used[i][r])
                continue;
            int node = fdt_path_offset(dt, owners[i]);
            if (node < 0 || fdt_appendprop_u32(dt, node, "memory-region", region_ph[r]))
                bail("cannot set the %s memory-region\n", j773g.s[i].what);
            node = fdt_path_offset(dt, owners[i]);
            if (node < 0 ||
                fdt_appendprop_string(dt, node, "memory-region-names", j773g_regions[r].name))
                bail("cannot set the %s memory-region-names\n", j773g.s[i].what);
        }

    int dcp = fdt_path_offset(dt, j773g.n.dcp);
    if (dcp < 0 || firmware_set_fdt(dt, dcp, "apple,firmware-version", &os_firmware))
        bail("cannot publish the firmware version\n");
    dcp = fdt_path_offset(dt, j773g.n.dcp);
    if (dcp < 0 || firmware_set_fdt(dt, dcp, "apple,firmware-compat", &os_firmware))
        bail("cannot publish the firmware compat version\n");
    dcp = fdt_path_offset(dt, j773g.n.dcp);
    if (dcp < 0 || fdt_setprop_string(dt, dcp, "apple,firmware-uuid", J773G_UUID))
        bail("cannot publish the firmware uuid\n");
    dcp = fdt_path_offset(dt, j773g.n.dcp);
    if (dcp < 0 || fdt_setprop_u32(dt, dcp, J773G_MARKER, 1))
        bail("cannot set " J773G_MARKER "\n");

    const char *enable[] = {j773g.n.dart_dcp, j773g.n.dart_disp, j773g.n.mbox, j773g.n.disp,
                            j773g.n.dcp};
    for (size_t i = 0; i < ARRAY_SIZE(enable); i++)
        if (set_status_okay(dt, enable[i]))
            bail("cannot enable %s\n", enable[i]);
    return 0;
}

/* Lock a display DART (PROTECT TTBR/TCR); it stays locked until reset */
static int lock_dart(u64 dart, const char *what)
{
    int g = guard_begin();
    u32 prot = read32(dart + DART_T8110_PROTECT);
    if (!(prot & DART_T8110_PROTECT_TTBR_TCR))
        write32(dart + DART_T8110_PROTECT, prot | DART_T8110_PROTECT_TTBR_TCR);
    u32 after = read32(dart + DART_T8110_PROTECT);
    if (guard_end(g) || !(after & DART_T8110_PROTECT_TTBR_TCR))
        bail("locking %s failed (PROTECT 0x%x)\n", what, after);
    jlog("%s locked (PROTECT 0x%x)\n", what, after);
    return 0;
}

static void set_result(void *dt, const char *result)
{
    int chosen = fdt_path_offset(dt, "/chosen");
    if (chosen < 0 || fdt_setprop_string(dt, chosen, J773G_RESULT, result))
        jlog("cannot record the result in /chosen/" J773G_RESULT "\n");
}

static int handoff(void *dt, int dcp)
{
    if (check_dt(dt, dcp) || check_adt() || check_power() || check_running() || check_streams())
        return -1;

    j773g.step = "device tree update";
    int size = fdt_totalsize(dt);
    void *scratch = malloc(size);
    if (!scratch)
        bail("no memory for a device tree copy (%d bytes)\n", size);
    memcpy(scratch, dt, size);
    if (publish(scratch)) {
        free(scratch);
        bail("device tree update failed (full?); nothing published\n");
    }

    j773g.step = "DART lock";
    if (lock_dart(J773G_DART_DCP, "dart-dcpext0") || lock_dart(J773G_DART_DISP, "dart-dispext0")) {
        free(scratch);
        return -1;
    }
    if (fdt_check_header(scratch) || fdt_totalsize(scratch) != (u32)size) {
        free(scratch);
        bail("device tree copy is not valid; nothing published\n");
    }
    memcpy(dt, scratch, size);
    free(scratch);
    return 0;
}

/* Always returns 0: nothing here may stop the boot */
int dt_set_j773g_hdmi_handoff(void *dt)
{
    memset(&j773g, 0, sizeof(j773g));

    if (chip_id != T8132 || fdt_node_check_compatible(dt, 0, "apple,j773g"))
        return 0;

    const char *path = fdt_get_alias(dt, "dcpext0");
    int dcp = path ? fdt_path_offset(dt, path) : -1;
    if (dcp < 0 || fdt_node_check_compatible(dt, dcp, "apple,t8132-dcpext")) {
        jlog("the device tree does not describe dcpext0: boot framebuffer only\n");
        return 0;
    }
    if (fdt_getprop(dt, dcp, J773G_MARKER, NULL)) {
        jlog("handoff already published\n");
        return 0;
    }

    if (handoff(dt, dcp)) {
        char result[64];
        snprintf(result, sizeof(result), "refused: %s", j773g.step);
        jlog("handoff refused (%s); keeping the boot framebuffer\n", j773g.step);
        set_result(dt, result);
        return 0;
    }
    set_result(dt, "published");
    jlog("dcpext0 firmware %s handed over: dcpext0, dispext0 and their DARTs enabled\n",
         J773G_UUID);
    return 0;
}
