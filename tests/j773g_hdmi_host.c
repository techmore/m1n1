/* SPDX-License-Identifier: MIT */
/*
 * Host test of the Mac mini M4 (J773g) HDMI handoff in src/kboot_display_j773g.c, and of the
 * __OS_LOG pass of src/kboot_display_handoff.c that dt_set_display() runs after it on J773g.
 *
 * A fake ADT, PMGR table, MMIO and DRAM stand in for the Mac (stand-in headers:
 * tests/t8122_display_host/, with tests/j773g_hdmi_host/ over them); libfdt is m1n1's own and
 * the device tree is built here with it, shaped as the kernel's t8132-j773g.dts (branch
 * mini/hdmi) describes the HDMI pipe. Addresses, segments, pt-regions and stream IDs are the
 * J773g ADT's (26.6.2 boot); the mappings are synthetic but follow the ADT segment remaps.
 *
 *   j773g-hdmi-host list
 *   j773g-hdmi-host <scenario>
 *
 * Each scenario runs dt_set_j773g_hdmi_handoff() once (twice for "repeat"), checks the outcome,
 * then runs dt_set_display_handoff() as kboot.c does next and checks the __OS_LOG reservations of
 * dcp, dcpext0 and dcpext1 (by default all three, none on another chip or board), and prints
 * "<scenario>: pass". tests/python/test_j773g_hdmi_handoff.py runs them all.
 *  PUBLISH: the whole contract is in the device tree and both DARTs are locked.
 *  NOTHING: no register access at all and the device tree byte for byte unchanged.
 *  REFUSE:  no register write, and the device tree unchanged but for
 *           /chosen/aurora,j773g-hdmi-handoff = "refused: <step>"; REFUSE_NO_MMIO also no read.
 */

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "adt.h"
#include "exception.h"
#include "firmware.h"
#include "kboot.h"
#include "mcc.h"
#include "pmgr.h"
#include "soc.h"
#include "types.h"
#include "utils.h"

#include "libfdt/libfdt.h"

int dt_set_j773g_hdmi_handoff(void *dt);
int dt_set_display_handoff(void *dt);

void *adt = (void *)1;
u32 chip_id;
volatile enum exc_guard_t exc_guard;
volatile int exc_count;
struct fw_version_info os_firmware;
size_t mcc_carveout_count;
struct mcc_carveout mcc_carveouts[8];
static u64 ram_base, ram_size, fb_base;
static int failures;

u64 kboot_ram_base(void)
{
    return ram_base;
}
u64 kboot_ram_size(void)
{
    return ram_size;
}
u64 kboot_fb_base(void)
{
    return fb_base;
}
void kboot_loaded_file_part(u64 *base, u64 *size)
{
    *base = 0x10010000000ULL;
    *size = 0xe0000;
}

int firmware_set_fdt(void *fdt, int node, const char *prop, const struct fw_version_info *ver)
{
    fdt32_t data[5];
    for (size_t i = 0; i < ver->num_length; i++)
        data[i] = cpu_to_fdt32(ver->num[i]);
    return fdt_setprop(fdt, node, prop, data, ver->num_length * 4) ? -1 : 0;
}

static void check(bool ok, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    printf(ok ? "ok   " : "FAIL ");
    vprintf(fmt, ap);
    printf("\n");
    va_end(ap);
    if (!ok)
        failures++;
}

/* ---- fake ADT ---- */

struct fnode {
    char path[96];
    struct adt_property *props[32];
    int nprops;
    u64 reg[8][2];
    int nreg;
};
static struct fnode nodes[32];
static int nnodes;

static int node_find(const char *path)
{
    for (int i = 0; i < nnodes; i++)
        if (!strcmp(nodes[i].path, path))
            return i;
    return -1;
}

static struct fnode *node_get(const char *path)
{
    int i = node_find(path);
    if (i >= 0)
        return &nodes[i];
    snprintf(nodes[nnodes].path, sizeof(nodes[0].path), "%s", path);
    return &nodes[nnodes++];
}

static void node_drop(const char *path)
{
    size_t n = strlen(path);
    for (int i = 0; i < nnodes; i++)
        if (!strncmp(nodes[i].path, path, n) && (!nodes[i].path[n] || nodes[i].path[n] == '/'))
            nodes[i].path[0] = 0;
}

static void prop_set(const char *path, const char *name, const void *val, u32 len)
{
    struct fnode *n = node_get(path);
    for (int i = 0; i < n->nprops; i++)
        if (!strcmp(n->props[i]->name, name)) {
            free(n->props[i]);
            n->props[i] = n->props[--n->nprops];
            break;
        }
    struct adt_property *p = calloc(1, sizeof(*p) + len + 8);
    snprintf(p->name, sizeof(p->name), "%s", name);
    p->size = len;
    memcpy(p->value, val, len);
    n->props[n->nprops++] = p;
}

static void prop_u32(const char *path, const char *name, u32 v)
{
    prop_set(path, name, &v, 4);
}

static void prop_u64(const char *path, const char *name, u64 v)
{
    prop_set(path, name, &v, 8);
}

static void prop_pair(const char *path, const char *name, u64 a, u64 b)
{
    u64 v[2] = {a, b};
    prop_set(path, name, v, 16);
}

static void prop_str(const char *path, const char *name, const char *s)
{
    prop_set(path, name, s, strlen(s) + 1);
}

static void reg_add(const char *path, u64 a, u64 s)
{
    struct fnode *n = node_get(path);
    n->reg[n->nreg][0] = a;
    n->reg[n->nreg][1] = s;
    n->nreg++;
}

int adt_path_offset(const void *a, const char *path)
{
    UNUSED(a);
    int i = node_find(path);
    return i < 0 ? -1 : i + 1;
}

int adt_path_offset_trace(const void *a, const char *path, int *offsets)
{
    int n = adt_path_offset(a, path);
    if (n < 0)
        return n;
    offsets[0] = n;
    offsets[1] = 0;
    return n;
}

const void *adt_getprop(const void *a, int node, const char *name, u32 *lenp)
{
    UNUSED(a);
    if (node < 1 || node > nnodes)
        return NULL;
    struct fnode *n = &nodes[node - 1];
    for (int i = 0; i < n->nprops; i++)
        if (!strcmp(n->props[i]->name, name)) {
            if (lenp)
                *lenp = n->props[i]->size;
            return n->props[i]->value;
        }
    return NULL;
}

int adt_get_reg(const void *a, int *path, const char *prop, int idx, u64 *addr, u64 *size)
{
    UNUSED(a);
    UNUSED(prop);
    int last = 0;
    for (int i = 0; path[i]; i++)
        last = path[i];
    if (last < 1)
        return -1;
    struct fnode *n = &nodes[last - 1];
    if (idx < 0 || idx >= n->nreg)
        return -1;
    if (addr)
        *addr = n->reg[idx][0];
    if (size)
        *size = n->reg[idx][1];
    return 0;
}

bool adt_is_compatible(const void *a, int node, const char *compat)
{
    u32 len = 0;
    const char *c = adt_getprop(a, node, "compatible", &len);
    for (u32 i = 0; c && i < len; i += strlen(c + i) + 1)
        if (!strcmp(c + i, compat))
            return true;
    return false;
}

int host_adt_nprops(int node)
{
    return node >= 1 && node <= nnodes ? nodes[node - 1].nprops : 0;
}

const struct adt_property *host_adt_prop(int node, int i)
{
    return nodes[node - 1].props[i];
}

int adt_get_property_count(const void *a, int node)
{
    UNUSED(a);
    return host_adt_nprops(node);
}

/* ---- fake PMGR table ---- */

static struct fdev {
    u16 id;
    const char *name;
    bool virt;
    u64 psreg;
    u16 parent;
} fdevs[8];
static int nfdevs;

int pmgr_device_gate(u8 die, u16 id, struct pmgr_gate *g)
{
    if (!id)
        return -1;
    memset(g, 0, sizeof(*g));
    g->entry = ((u32)die << 28) | id;
    g->id = id;
    g->die = die;
    for (int i = 0; i < nfdevs; i++)
        if (fdevs[i].id == id) {
            g->found = true;
            g->virt = fdevs[i].virt;
            g->name = fdevs[i].name;
            g->psreg = fdevs[i].virt ? 0 : fdevs[i].psreg;
            g->parent[0] = fdevs[i].parent;
        }
    return 0;
}

int pmgr_adt_gate(const char *path, const char *prop, u32 index, struct pmgr_gate *g)
{
    u32 len = 0;
    int node = adt_path_offset(adt, path);
    const u32 *e = node < 0 ? NULL : adt_getprop(adt, node, prop, &len);
    if (!e || index >= len / 4)
        return -1;
    pmgr_device_gate(e[index] >> 28, e[index] & 0xffff, g);
    g->entry = e[index];
    return 0;
}

/* ---- fake MMIO and DRAM; every access is counted ---- */

struct region {
    u64 base, size;
    u8 *buf;
};
static struct region regions[16];
static int nregions, mmio_reads, mmio_writes;
static bool protect_sticks = true;

static void region_add(u64 base, u64 size)
{
    regions[nregions++] = (struct region){base, size, calloc(1, size)};
}

static u8 *mem(u64 a, int len)
{
    for (int i = 0; i < nregions; i++)
        if (a >= regions[i].base && a + len <= regions[i].base + regions[i].size)
            return regions[i].buf + (a - regions[i].base);
    return NULL;
}

static void poke32(u64 a, u32 v)
{
    memcpy(mem(a, 4), &v, 4);
}

static u32 peek32(u64 a)
{
    u32 v;
    memcpy(&v, mem(a, 4), 4);
    return v;
}

static void poke64(u64 a, u64 v)
{
    memcpy(mem(a, 8), &v, 8);
}

static u64 peek64(u64 a)
{
    u64 v;
    memcpy(&v, mem(a, 8), 8);
    return v;
}

static void fault(const char *what, u64 a)
{
    if (exc_guard) {
        exc_count++;
        return;
    }
    printf("FAIL unguarded %s of unmapped 0x%llx\n", what, (unsigned long long)a);
    exit(3);
}

u32 read32(u64 a)
{
    u8 *p = mem(a, 4);
    u32 v = 0;
    mmio_reads++;
    if (!exc_guard)
        printf("FAIL unguarded read32 of 0x%llx\n", (unsigned long long)a), failures++;
    if (!p)
        fault("read32", a);
    else
        memcpy(&v, p, 4);
    return v;
}

u64 read64(u64 a)
{
    u8 *p = mem(a, 8);
    u64 v = 0;
    mmio_reads++;
    if (!p)
        fault("read64", a);
    else
        memcpy(&v, p, 8);
    return v;
}

void write32(u64 a, u32 v)
{
    u8 *p = mem(a, 4);
    mmio_writes++;
    if (!exc_guard)
        printf("FAIL unguarded write32 of 0x%llx\n", (unsigned long long)a), failures++;
    if (!p)
        fault("write32", a);
    else if (protect_sticks)
        memcpy(p, &v, 4);
}

/* ---- DART page tables (T8110 two-level format) ---- */

#define PTE_ADDR(x) (((x) >> 4) & 0xffffffffc00ULL)

struct pt {
    u64 lo, next;
};

static void pt_init(struct pt *t, u64 lo, u64 size)
{
    region_add(lo, size);
    t->lo = lo;
    t->next = lo + 0x4000;
}

static void pt_map(struct pt *t, u64 dva, u64 pa, u64 size)
{
    for (u64 o = 0; o < size; o += 0x4000) {
        u64 d = dva + o, i = d >> 25, j = (d >> 14) & 2047;
        u64 e1 = peek64(t->lo + 8 * i);
        if (!(e1 & 1)) {
            e1 = PTE_ADDR(t->next) | 1;
            t->next += 0x4000;
            poke64(t->lo + 8 * i, e1);
        }
        u64 l2 = (e1 & 0xffffffffc00ULL) << 4;
        poke64(l2 + 8 * j, PTE_ADDR(pa + o) | 3);
    }
}

static u32 ttbr_of(u64 l1)
{
    return (u32)(((l1 >> 14) << 2) | 1);
}

/* ---- the J773g world: values of the J773g ADT (26.6.2 boot), arm-io ranges applied ---- */

#define TB         0x10000000000ULL
#define COPROC     0x286e00000ULL
#define MBOX       0x286e08000ULL
#define DART_DCP   0x286340000ULL
#define DART_DISP  0x286300000ULL
#define PMGR_BASE  0x380700000ULL /* synthetic */
#define PS_FE      (PMGR_BASE + 0x1a8)
#define SID_DCP    23
#define SID_DISP   0
#define SID_PIODMA 16
#define UUID       "81BAF476-679D-3CC0-8C6A-E6B6FA659D5E"
#define MARKER     "apple,j773g-hdmi-handoff"
#define RESULT     "aurora,j773g-hdmi-handoff"
#define NUB        "/arm-io/dcpext0/iop-dcpext0-nub"
#define DCP_NUB    "/arm-io/dcp/iop-dcp-nub"
#define EXT1_NUB   "/arm-io/dcpext1/iop-dcpext1-nub"

#define PT_DCP_LO    (TB + 0x3ffe8c000ULL)
#define PT_DCP_HI    (TB + 0x3ffebc000ULL)
#define PT_DISP_LO   (TB + 0x3ffebc000ULL)
#define PT_DISP_HI   (TB + 0x3ffedc000ULL)
#define PT_PIODMA_LO (TB + 0x3ffe7c000ULL)
#define PT_PIODMA_HI (TB + 0x3ffe8c000ULL)
#define VRAM         (TB + 0x3e5300000ULL)
#define VRAM_SIZE    0x854000ULL
#define DISP_FB_DVA  0x4000000ULL /* synthetic */
#define PIODMA_DVA   0x8000000ULL /* synthetic */
#define DCP_VRAM_DVA 0x10000000ULL

struct seg {
    u64 phys, iova, remap;
    u32 size, flags;
};

enum { SEG_TEXT, SEG_DATA, SEG_OSLOG, SEG_HEAP, NSEGS };

/* /arm-io/dcpext0/iop-dcpext0-nub segment-ranges of the J773g ADT */
static struct seg segs[NSEGS] = {
    {0x100002a8000ULL, 0, 0x10001c4c000ULL, 0x806000, 1},
    {0x103e4c6c000ULL, 0x806000, 0x10002454000ULL, 0x325000, 0},
    {0x10003f30000ULL, 0xb2b000, 0x10003f30000ULL, 0x24000, 0xa},
    {0x103e3c6c000ULL, ~0ULL, 0x1000277c000ULL, 0x1000000, 2},
};

/* /arm-io/dcp/iop-dcp-nub and /arm-io/dcpext1/iop-dcpext1-nub of the J773g ADT */
static const struct seg dcp_segs[6] = {
    {0x100002a8000ULL, 0, 0x10001c4c000ULL, 0x806000, 1},
    {0x103e4f94000ULL, 0x806000, 0x10002454000ULL, 0x325000, 0},
    {0x10003f0c000ULL, 0xb2b000, 0x10003f0c000ULL, 0x24000, 0xa},
    {0x103e5c1c000ULL, ~0ULL, 0x10000b70000ULL, 0x1000000, 2},
    {0x103e5c18000ULL, ~0ULL, 0x10001b88000ULL, 0x4000, 2},
    {0x103e5b58000ULL, ~0ULL, 0x10001b8c000ULL, 0xc0000, 2},
};
static const struct seg ext1_segs[4] = {
    {0x100002a8000ULL, 0, 0x10001c4c000ULL, 0x806000, 1},
    {0x103e3944000ULL, 0x806000, 0x10002454000ULL, 0x325000, 0},
    {0x10003f54000ULL, 0xb2b000, 0x10003f54000ULL, 0x24000, 0xa},
    {0x103e2944000ULL, ~0ULL, 0x1000277c000ULL, 0x1000000, 2},
};

/* The three __OS_LOG buffers: dcp, dcpext0, dcpext1 */
static const u64 oslogs[3] = {0x10003f0c000ULL, 0x10003f30000ULL, 0x10003f54000ULL};
#define OSLOG_SIZE 0x24000ULL

static u64 seg_size(int s)
{
    return ALIGN_UP((u64)segs[s].size, SZ_16K);
}

static u64 seg_dva(int s)
{
    return segs[s].remap & ((1ULL << 36) - 1);
}

/* What the scenario changes in the hardware world */
struct opts {
    bool power_off, cpu_stopped, disp_ttbr_invalid, disp_no_fb, disp_heap, dcp_vram, stray_map,
        oslog_mapped, text_wrong_dva, text_partial, pt_outside, piodma_empty, prelocked;
};

static void world(const struct opts *o)
{
    static const struct opts none;
    if (!o)
        o = &none;

    chip_id = T8132;
    os_firmware = (struct fw_version_info){V26_6_2, "26.6.2", {26, 6, 2}, 3, "mBoot-18000.161.10"};
    /* The OS gets DRAM from 16 MiB up to below the display carveouts; __OS_LOG falls inside it */
    ram_base = TB + 0x1000000;
    ram_size = 0x3e0000000ULL - 0x1000000;
    fb_base = VRAM;
    mcc_carveout_count = 0;

    prop_u64("/chosen", "dram-base", TB);
    prop_u64("/chosen", "dram-size", 0x400000000ULL);
    reg_add("/vram", VRAM, VRAM_SIZE);

    reg_add("/arm-io/dcpext0", COPROC, 0x88000);
    prop_str("/arm-io/dcpext0", "compatible", "iop,ascwrap-v6");
    prop_u32("/arm-io/dcpext0", "clock-gates", 42);
    prop_set(NUB, "segment-ranges", segs, sizeof(segs));
    prop_str(NUB, "segment-names", "__TEXT;__DATA;__OS_LOG;unknown");
    prop_str(NUB, "uuid", UUID);
    prop_u32(NUB, "pre-loaded", 1);
    prop_u32(NUB, "running", 1);
    prop_set(DCP_NUB, "segment-ranges", dcp_segs, sizeof(dcp_segs));
    prop_str(DCP_NUB, "segment-names", "__TEXT;__DATA;__OS_LOG;unknown;unknown;unknown");
    prop_str(DCP_NUB, "uuid", UUID);
    prop_set(EXT1_NUB, "segment-ranges", ext1_segs, sizeof(ext1_segs));
    prop_str(EXT1_NUB, "segment-names", "__TEXT;__DATA;__OS_LOG;unknown");
    prop_str(EXT1_NUB, "uuid", UUID);

    const char *dd = "/arm-io/dart-dcpext0", *dp = "/arm-io/dart-dispext0";
    prop_str(dd, "compatible", "dart,t8110");
    reg_add(dd, DART_DCP, 0x20000);
    prop_u64(dd, "vm-base", TB);
    prop_u64(dd, "vm-size", 3298534883328ULL);
    prop_u64(dd, "vm-size-23", 0x1000000000ULL);
    prop_u32(dd, "sid-count", 32);
    prop_pair(dd, "pt-region-23", PT_DCP_LO, PT_DCP_HI);
    prop_str(dp, "compatible", "dart,t8110");
    reg_add(dp, DART_DISP, 0xc000);
    prop_u64(dp, "vm-base", TB);
    prop_u64(dp, "vm-size", 0x1000000000ULL);
    prop_u32(dp, "sid-count", 32);
    prop_pair(dp, "pt-region-0", PT_DISP_LO, PT_DISP_HI);
    prop_pair(dp, "pt-region-16", PT_PIODMA_LO, PT_PIODMA_HI);

    /* DISPEXT0_CPU (42) has no power state of its own; DISPEXT0_FE (41) is its parent */
    nfdevs = 0;
    fdevs[nfdevs++] = (struct fdev){41, "DISPEXT0_FE", false, PS_FE, 40};
    fdevs[nfdevs++] = (struct fdev){42, "DISPEXT0_CPU", true, 0, 41};

    region_add(PMGR_BASE, 0x4000);
    poke32(PS_FE, o->power_off ? 0x0 : 0xff);
    region_add(COPROC, 0x88000);
    poke32(COPROC + 0x44, o->cpu_stopped ? 0 : 0x10);
    region_add(DART_DCP, 0x20000);
    region_add(DART_DISP, 0xc000);
    if (o->prelocked) {
        poke32(DART_DCP + 0x200, 1);
        poke32(DART_DISP + 0x200, 1);
    }

    struct pt dcp, disp, piodma;
    pt_init(&dcp, PT_DCP_LO, PT_DCP_HI - PT_DCP_LO);
    pt_init(&disp, PT_DISP_LO, PT_DISP_HI - PT_DISP_LO);
    pt_init(&piodma, PT_PIODMA_LO, PT_PIODMA_HI - PT_PIODMA_LO);

    for (int s = 0; s < NSEGS; s++) {
        if (s == SEG_OSLOG)
            continue;
        u64 size = seg_size(s);
        u64 dva = seg_dva(s);
        if (s == SEG_TEXT && o->text_wrong_dva)
            dva -= 0x4000;
        if (s == SEG_TEXT && o->text_partial)
            size -= 0x4000;
        pt_map(&dcp, dva, segs[s].phys, size);
    }
    if (o->dcp_vram)
        pt_map(&dcp, DCP_VRAM_DVA, VRAM, VRAM_SIZE);
    if (o->stray_map)
        pt_map(&dcp, 0x20000000, TB + 0x3f0000000ULL, 0x4000);
    if (!o->disp_no_fb)
        pt_map(&disp, DISP_FB_DVA, VRAM, VRAM_SIZE);
    if (o->disp_heap || o->disp_no_fb)
        pt_map(&disp, 0xc000000, segs[SEG_HEAP].phys, seg_size(SEG_HEAP));
    if (o->oslog_mapped)
        pt_map(&disp, 0x14000000, segs[SEG_OSLOG].phys, 0x4000);
    if (!o->piodma_empty)
        pt_map(&piodma, PIODMA_DVA, segs[SEG_HEAP].phys, seg_size(SEG_HEAP));
    if (o->pt_outside)
        poke64(dcp.lo + 8 * 100, PTE_ADDR(TB + 0x3f0000000ULL) | 1);

    poke32(DART_DCP + 0x1000 + 4 * SID_DCP, 1);
    poke32(DART_DCP + 0x1400 + 4 * SID_DCP, ttbr_of(dcp.lo));
    poke32(DART_DISP + 0x1000 + 4 * SID_DISP, 1);
    poke32(DART_DISP + 0x1400 + 4 * SID_DISP, o->disp_ttbr_invalid ? 0 : ttbr_of(disp.lo));
    poke32(DART_DISP + 0x1000 + 4 * SID_PIODMA, 1);
    poke32(DART_DISP + 0x1400 + 4 * SID_PIODMA, ttbr_of(piodma.lo));
    mmio_reads = mmio_writes = 0;
}

/* ---- the device tree, as t8132.dtsi + t8132-j773g.dts (mini/hdmi) describe the pipe ---- */

#define DT_SIZE 0x10000
#define PH_DART_DISP 10
#define PH_DART_DCP  11
#define PH_MBOX      12
#define DCP_PATH     "/soc/dcp@286e00000"
#define DISP_PATH    "/soc/display-subsystem"
#define PIODMA_PATH  "/soc/dcp@286e00000/piodma"

struct dt_opts {
    const char *root_compat, *soc_compat;
    bool no_dcpext0, wrong_dcp_compat, dcp_okay, dart_okay, wrong_sid, wrong_dart_reg, no_mbox,
        no_piodma, no_disp_alias, memory_region, resv_vram, memreserve_oslog,
        memreserve_oslog_straddle, resv_dcp_oslog, resv_ext0_oslog;
    int free_space;
};

static void sets(void *dt, int node, const char *name, const char *v)
{
    if (fdt_setprop_string(dt, node, name, v)) {
        printf("FAIL building the device tree (%s)\n", name);
        exit(2);
    }
}

static void setu32s(void *dt, int node, const char *name, const u32 *v, int n)
{
    fdt32_t b[8];
    for (int i = 0; i < n; i++)
        b[i] = cpu_to_fdt32(v[i]);
    if (fdt_setprop(dt, node, name, b, n * 4)) {
        printf("FAIL building the device tree (%s)\n", name);
        exit(2);
    }
}

static void setu32(void *dt, int node, const char *name, u32 v)
{
    setu32s(dt, node, name, &v, 1);
}

static void setcompat(void *dt, int node, const char *a, const char *b)
{
    char buf[96];
    size_t la = strlen(a) + 1, lb = b ? strlen(b) + 1 : 0;
    memcpy(buf, a, la);
    if (b)
        memcpy(buf + la, b, lb);
    fdt_setprop(dt, node, "compatible", buf, la + lb);
}

static void setreg(void *dt, int node, u64 a, u64 s)
{
    fdt64_t r[2] = {cpu_to_fdt64(a), cpu_to_fdt64(s)};
    fdt_setprop(dt, node, "reg", r, sizeof(r));
}

static int sub(void *dt, const char *parent, const char *name)
{
    int p = fdt_path_offset(dt, parent);
    int n = p < 0 ? p : fdt_add_subnode(dt, p, name);
    if (n < 0) {
        printf("FAIL building the device tree (%s/%s)\n", parent, name);
        exit(2);
    }
    return n;
}

static void *build_dt(const struct dt_opts *d)
{
    static const struct dt_opts none;
    if (!d)
        d = &none;
    void *dt = calloc(1, DT_SIZE);
    fdt_create_empty_tree(dt, DT_SIZE);
    int root = fdt_path_offset(dt, "/");
    const char *rc = d->root_compat ? d->root_compat : "apple,j773g";
    char compat[96];
    int len = snprintf(compat, sizeof(compat), "%s", rc) + 1;
    len += snprintf(compat + len, sizeof(compat) - len, "%s",
                    d->soc_compat ? d->soc_compat : "apple,t8132") + 1;
    fdt_setprop(dt, root, "compatible", compat, len);
    setu32(dt, root, "#address-cells", 2);
    setu32(dt, root, "#size-cells", 2);

    sub(dt, "/", "chosen");
    int aliases = sub(dt, "/", "aliases");
    sets(dt, aliases, "serial0", "/soc/serial@235200000");
    if (!d->no_dcpext0)
        sets(dt, fdt_path_offset(dt, "/aliases"), "dcpext0", DCP_PATH);
    if (!d->no_disp_alias && !d->no_dcpext0)
        sets(dt, fdt_path_offset(dt, "/aliases"), "disp0", DISP_PATH);
    if (!d->no_dcpext0)
        sets(dt, fdt_path_offset(dt, "/aliases"), "dispext0_piodma", PIODMA_PATH);

    int resv = sub(dt, "/", "reserved-memory");
    setu32(dt, resv, "#address-cells", 2);
    setu32(dt, resv, "#size-cells", 2);
    fdt_setprop_empty(dt, resv, "ranges");
    int gpu = sub(dt, "/reserved-memory", "uat-ttbs");
    setreg(dt, gpu, 0, 0);
    fdt_setprop_empty(dt, gpu, "no-map");
    sets(dt, gpu, "status", "disabled");
    if (d->resv_dcp_oslog || d->resv_ext0_oslog) {
        int other = sub(dt, "/reserved-memory", "other@1");
        setreg(dt, other, (d->resv_dcp_oslog ? oslogs[0] : oslogs[1]) + 0x4000, 0x4000);
        fdt_setprop_empty(dt, other, "no-map");
    }
    if (d->resv_vram) {
        int other = sub(dt, "/reserved-memory", "other@0");
        setreg(dt, other, VRAM + 0x100000, 0x4000);
        fdt_setprop_empty(dt, other, "no-map");
    }

    int soc = sub(dt, "/", "soc");
    setu32(dt, soc, "#address-cells", 2);
    setu32(dt, soc, "#size-cells", 2);
    fdt_setprop_empty(dt, soc, "ranges");
    int serial = sub(dt, "/soc", "serial@235200000");
    setreg(dt, serial, 0x235200000ULL, 0x4000);

    /* the kernel DT's empty simple-framebuffer, filled by dt_set_fb() in m1n1 */
    int fb = sub(dt, "/chosen", "framebuffer");
    setcompat(dt, fb, "apple,simple-framebuffer", "simple-framebuffer");
    sets(dt, fb, "status", "disabled");

    if (d->no_dcpext0)
        goto done;

    static const u32 dma_range[4] = {0x100, 0, 0x10, 0};
    int n = sub(dt, "/soc", "iommu@286300000");
    setcompat(dt, n, "apple,t8132-dart", "apple,t8110-dart");
    setreg(dt, fdt_path_offset(dt, "/soc/iommu@286300000"),
           d->wrong_dart_reg ? 0x286310000ULL : DART_DISP, 0xc000);
    n = fdt_path_offset(dt, "/soc/iommu@286300000");
    setu32(dt, n, "#iommu-cells", 1);
    setu32s(dt, n, "apple,dma-range", dma_range, 4);
    sets(dt, n, "status", d->dart_okay ? "okay" : "disabled");
    setu32(dt, n, "phandle", PH_DART_DISP);

    n = sub(dt, "/soc", "iommu@286340000");
    setcompat(dt, n, "apple,t8132-dart", "apple,t8110-dart");
    setreg(dt, n, DART_DCP, 0x20000);
    setu32(dt, n, "#iommu-cells", 1);
    setu32s(dt, n, "apple,dma-range", dma_range, 4);
    sets(dt, n, "status", "disabled");
    setu32(dt, n, "phandle", PH_DART_DCP);

    if (!d->no_mbox) {
        n = sub(dt, "/soc", "mailbox@286e08000");
        setcompat(dt, n, "apple,t8132-asc-mailbox", "apple,asc-mailbox-v4");
        setreg(dt, n, MBOX, 0x4000);
        setu32(dt, n, "#mbox-cells", 0);
        sets(dt, n, "status", "disabled");
        setu32(dt, n, "phandle", PH_MBOX);
    }

    n = sub(dt, "/soc", "dcp@286e00000");
    if (d->wrong_dcp_compat)
        setcompat(dt, n, "apple,t8132-dcp", NULL);
    else
        setcompat(dt, n, "apple,t8132-dcpext", "apple,dcpext");
    setreg(dt, n, COPROC, 0x4000);
    setu32(dt, n, "mboxes", PH_MBOX);
    u32 iommus[2] = {PH_DART_DCP, d->wrong_sid ? 5 : SID_DCP};
    setu32s(dt, n, "iommus", iommus, 2);
    setu32(dt, n, "apple,dcp-index", 1);
    sets(dt, n, "apple,connector-type", "HDMI-A");
    setu32(dt, n, "#address-cells", 2);
    setu32(dt, n, "#size-cells", 2);
    sets(dt, n, "status", d->dcp_okay ? "okay" : "disabled");
    if (d->memory_region)
        setu32(dt, n, "memory-region", PH_MBOX);
    if (!d->no_piodma) {
        n = sub(dt, DCP_PATH, "piodma");
        u32 pio[2] = {PH_DART_DISP, SID_PIODMA};
        setu32s(dt, n, "iommus", pio, 2);
    }

    n = sub(dt, "/soc", "display-subsystem");
    sets(dt, n, "compatible", "apple,display-subsystem");
    u32 disp[2] = {PH_DART_DISP, SID_DISP};
    setu32s(dt, n, "iommus", disp, 2);
    fdt_setprop_empty(dt, n, "dma-noncoherent");
    sets(dt, n, "status", "disabled");

done:
    if (d->memreserve_oslog)
        fdt_add_mem_rsv(dt, segs[SEG_OSLOG].phys, seg_size(SEG_OSLOG));
    if (d->memreserve_oslog_straddle)
        fdt_add_mem_rsv(dt, segs[SEG_OSLOG].phys + 0x4000, 0x40000);
    if (d->free_space) {
        fdt_pack(dt);
        int total = fdt_totalsize(dt) + d->free_space;
        void *small = calloc(1, total);
        fdt_open_into(dt, small, total);
        free(dt);
        return small;
    }
    return dt;
}

/* ---- checks of the outcome ---- */

static const char *prop_str_of(void *dt, const char *path, const char *name)
{
    int node = fdt_path_offset(dt, path);
    return node < 0 ? NULL : fdt_getprop(dt, node, name, NULL);
}

static bool status_okay(void *dt, const char *path)
{
    const char *s = prop_str_of(dt, path, "status");
    return s && !strcmp(s, "okay");
}

static bool names_are(void *dt, const char *path, const char *want, int want_len)
{
    int node = fdt_path_offset(dt, path), len = 0;
    const char *v = node < 0 ? NULL : fdt_getprop(dt, node, "memory-region-names", &len);
    return v && len == want_len && !memcmp(v, want, len);
}

/* /reserved-memory/<name>: reg, no-map, compatible, and its iommu-addresses entries */
static void check_resv(void *dt, const char *name, u64 phys, u64 size, const char *compat,
                       const u32 *owners, const u64 *dvas, const u64 *sizes, int n)
{
    char path[96];
    snprintf(path, sizeof(path), "/reserved-memory/%s", name);
    int node = fdt_path_offset(dt, path), len = 0;
    check(node >= 0, "%s exists", path);
    if (node < 0)
        return;
    const fdt64_t *reg = fdt_getprop(dt, node, "reg", &len);
    check(reg && len == 16 && fdt64_ld(&reg[0]) == phys && fdt64_ld(&reg[1]) == size,
          "%s reg 0x%llx+0x%llx", path, (unsigned long long)phys, (unsigned long long)size);
    check(fdt_getprop(dt, node, "no-map", NULL) != NULL, "%s no-map", path);
    const char *c = fdt_getprop(dt, node, "compatible", NULL);
    check(compat ? c && !strcmp(c, compat) : !c, "%s compatible %s", path, compat ? compat : "none");
    const fdt32_t *ia = fdt_getprop(dt, node, "iommu-addresses", &len);
    check(n ? ia && len == n * 20 : !ia, "%s has %d iommu-addresses entr%s", path, n,
          n == 1 ? "y" : "ies");
    for (int i = 0; ia && i < n && len == n * 20; i++) {
        const fdt32_t *e = ia + 5 * i;
        u32 ph = fdt32_ld(e);
        u64 dva = fdt64_ld((const fdt64_t *)(e + 1)), sz = fdt64_ld((const fdt64_t *)(e + 3));
        check(ph == owners[i] && dva == (TB | dvas[i]) && sz == sizes[i],
              "%s iommu-addresses[%d] = <%u 0x%llx 0x%llx>", path, i, owners[i],
              (unsigned long long)(TB | dvas[i]), (unsigned long long)sizes[i]);
    }
}

static u32 phandle_of(void *dt, const char *path)
{
    int node = fdt_path_offset(dt, path);
    return node < 0 ? 0 : fdt_get_phandle(dt, node);
}

static u32 resv_phandle(void *dt, const char *fmt, u64 phys)
{
    char path[96];
    snprintf(path, sizeof(path), fmt, (unsigned long long)phys);
    return phandle_of(dt, path);
}

static void check_published(void *dt, const struct opts *o)
{
    static const struct opts none;
    if (!o)
        o = &none;
    u32 dcp = phandle_of(dt, DCP_PATH), disp = phandle_of(dt, DISP_PATH),
        pio = phandle_of(dt, PIODMA_PATH);
    check(dcp && disp && pio, "dcpext0, the display and piodma have phandles");

    const char *res = prop_str_of(dt, "/chosen", RESULT);
    check(res && !strcmp(res, "published"), "/chosen/" RESULT " = \"published\"");
    static const char *const enabled[] = {DCP_PATH, DISP_PATH, "/soc/iommu@286300000",
                                          "/soc/iommu@286340000", "/soc/mailbox@286e08000"};
    for (size_t i = 0; i < ARRAY_SIZE(enabled); i++)
        check(status_okay(dt, enabled[i]), "%s status okay", enabled[i]);
    check(prop_str_of(dt, PIODMA_PATH, "status") == NULL, "piodma has no status (available)");

    int node = fdt_path_offset(dt, DCP_PATH), len = 0;
    const fdt32_t *v = fdt_getprop(dt, node, "apple,firmware-version", &len);
    check(v && len == 12 && fdt32_ld(&v[0]) == 26 && fdt32_ld(&v[1]) == 6 && fdt32_ld(&v[2]) == 2,
          "dcpext0 apple,firmware-version = <26 6 2>");
    v = fdt_getprop(dt, node, "apple,firmware-compat", &len);
    check(v && len == 12 && fdt32_ld(&v[0]) == 26 && fdt32_ld(&v[1]) == 6 && fdt32_ld(&v[2]) == 2,
          "dcpext0 apple,firmware-compat = <26 6 2>");
    const char *uuid = prop_str_of(dt, DCP_PATH, "apple,firmware-uuid");
    check(uuid && !strcmp(uuid, UUID), "dcpext0 apple,firmware-uuid = " UUID);
    check(fdt_getprop(dt, fdt_path_offset(dt, DCP_PATH), MARKER, NULL) != NULL,
          "dcpext0 has " MARKER);

    /* dcpext0: text, data, heap (and /vram), at the ADT DVAs */
    u64 text = segs[SEG_TEXT].phys, data = segs[SEG_DATA].phys, heap = segs[SEG_HEAP].phys,
        oslog = segs[SEG_OSLOG].phys;
    u32 owner1[1] = {dcp};
    u64 dva1[1] = {seg_dva(SEG_TEXT)}, size1[1] = {seg_size(SEG_TEXT)};
    char name[64];
    snprintf(name, sizeof(name), "dcpext0-text@%llx", (unsigned long long)text);
    check_resv(dt, name, text, seg_size(SEG_TEXT), "apple,asc-mem", owner1, dva1, size1, 1);
    dva1[0] = seg_dva(SEG_DATA);
    size1[0] = seg_size(SEG_DATA);
    snprintf(name, sizeof(name), "dcpext0-data@%llx", (unsigned long long)data);
    check_resv(dt, name, data, seg_size(SEG_DATA), "apple,asc-mem", owner1, dva1, size1, 1);

    u32 heap_owners[3];
    u64 heap_dvas[3], heap_sizes[3];
    int nheap = 0;
    heap_owners[nheap] = dcp;
    heap_dvas[nheap] = seg_dva(SEG_HEAP);
    heap_sizes[nheap++] = seg_size(SEG_HEAP);
    if (o->disp_heap) {
        heap_owners[nheap] = disp;
        heap_dvas[nheap] = 0xc000000;
        heap_sizes[nheap++] = seg_size(SEG_HEAP);
    }
    if (!o->piodma_empty) {
        heap_owners[nheap] = pio;
        heap_dvas[nheap] = PIODMA_DVA;
        heap_sizes[nheap++] = seg_size(SEG_HEAP);
    }
    snprintf(name, sizeof(name), "dcpext0-heap@%llx", (unsigned long long)heap);
    check_resv(dt, name, heap, seg_size(SEG_HEAP), "apple,asc-mem", heap_owners, heap_dvas,
               heap_sizes, nheap);

    u32 fb_owners[2];
    u64 fb_dvas[2], fb_sizes[2];
    int nfb = 0;
    if (o->dcp_vram) {
        fb_owners[nfb] = dcp;
        fb_dvas[nfb] = DCP_VRAM_DVA;
        fb_sizes[nfb++] = VRAM_SIZE;
    }
    fb_owners[nfb] = disp;
    fb_dvas[nfb] = DISP_FB_DVA;
    fb_sizes[nfb++] = VRAM_SIZE;
    snprintf(name, sizeof(name), "framebuffer@%llx", (unsigned long long)VRAM);
    check_resv(dt, name, VRAM, VRAM_SIZE, "framebuffer", fb_owners, fb_dvas, fb_sizes, nfb);

    snprintf(name, sizeof(name), "dcp-oslog@%llx", (unsigned long long)oslog);
    check_resv(dt, name, oslog, seg_size(SEG_OSLOG), NULL, NULL, NULL, NULL, 0);
    snprintf(name, sizeof(name), "/reserved-memory/dcp-oslog@%llx", (unsigned long long)oslog);
    check(fdt_getprop(dt, fdt_path_offset(dt, name), "apple,dcp-os-log", NULL) != NULL,
          "the __OS_LOG node is marked apple,dcp-os-log");

    /* memory-region lists, in the order the kernel looks them up */
    u32 ph_text = resv_phandle(dt, "/reserved-memory/dcpext0-text@%llx", text);
    u32 ph_data = resv_phandle(dt, "/reserved-memory/dcpext0-data@%llx", data);
    u32 ph_heap = resv_phandle(dt, "/reserved-memory/dcpext0-heap@%llx", heap);
    u32 ph_fb = resv_phandle(dt, "/reserved-memory/framebuffer@%llx", VRAM);
    u32 ph_oslog = resv_phandle(dt, "/reserved-memory/dcp-oslog@%llx", oslog);
    u32 want_dcp[4] = {ph_text, ph_data, ph_heap, ph_fb};
    int ndcp = o->dcp_vram ? 4 : 3;
    v = fdt_getprop(dt, fdt_path_offset(dt, DCP_PATH), "memory-region", &len);
    bool ok = v && len == ndcp * 4;
    for (int i = 0; ok && i < ndcp; i++)
        ok = fdt32_ld(&v[i]) == want_dcp[i] && want_dcp[i] && want_dcp[i] != ph_oslog;
    check(ok, "dcpext0 memory-region = <text data heap%s>", o->dcp_vram ? " framebuffer" : "");
    check(o->dcp_vram ? names_are(dt, DCP_PATH, "text\0data\0heap\0framebuffer", 27)
                      : names_are(dt, DCP_PATH, "text\0data\0heap", 15),
          "dcpext0 memory-region-names");

    v = fdt_getprop(dt, fdt_path_offset(dt, DISP_PATH), "memory-region", &len);
    check(v && len == (o->disp_heap ? 8 : 4) && fdt32_ld(&v[0]) == ph_fb &&
              (!o->disp_heap || fdt32_ld(&v[1]) == ph_heap),
          "display memory-region = <framebuffer%s>", o->disp_heap ? " heap" : "");
    check(o->disp_heap ? names_are(dt, DISP_PATH, "framebuffer\0heap", 17)
                       : names_are(dt, DISP_PATH, "framebuffer", 12),
          "display memory-region-names starts with framebuffer");

    v = fdt_getprop(dt, fdt_path_offset(dt, PIODMA_PATH), "memory-region", &len);
    if (o->piodma_empty)
        check(!v, "piodma has no memory-region");
    else
        check(v && len == 4 && fdt32_ld(&v[0]) == ph_heap && names_are(dt, PIODMA_PATH, "heap", 5),
              "piodma memory-region = <heap>");

    check(peek32(DART_DCP + 0x200) & 1, "dart-dcpext0 locked");
    check(peek32(DART_DISP + 0x200) & 1, "dart-dispext0 locked");
    check(fdt_check_full(dt, fdt_totalsize(dt)) == 0, "the device tree is valid");
}

/* ---- scenarios ---- */

enum outcome { PUBLISH, NOTHING, REFUSE, REFUSE_NO_MMIO };

struct scenario {
    const char *name;
    enum outcome outcome;
    const char *step; /* REFUSE: the step /chosen names */
    struct opts hw;
    struct dt_opts dt;
    void (*adt)(void);
    unsigned oslog; /* __OS_LOG nodes expected after the pass (bit n: oslogs[n]); 0: all three */
};

#define OSLOG_NONE 8

static void adt_bad_uuid(void)
{
    prop_str(NUB, "uuid", "81BAF476-679D-3CC0-8C6A-E6B6FA659D5F");
}
static void adt_not_running(void)
{
    prop_u32(NUB, "running", 0);
}
static void adt_no_nub(void)
{
    node_drop(NUB);
}
static void adt_wrong_dart(void)
{
    node_drop("/arm-io/dart-dispext0");
}
static void adt_seg_flags(void)
{
    struct seg s[NSEGS];
    memcpy(s, segs, sizeof(s));
    s[SEG_DATA].flags = 2;
    prop_set(NUB, "segment-ranges", s, sizeof(s));
}
static void adt_text_in_ram(void)
{
    ram_base = TB + 0x200000;
}
static void adt_fb_outside(void)
{
    fb_base = VRAM + VRAM_SIZE;
}
static void adt_carveout(void)
{
    mcc_carveouts[0] = (struct mcc_carveout){segs[SEG_DATA].phys + 0x4000, 0x4000};
    mcc_carveout_count = 1;
}
static void adt_wrong_os(void)
{
    os_firmware = (struct fw_version_info){V14_7, "14.7", {14, 7}, 2, "iBoot-11881.1.1"};
}
static void adt_wrong_chip(void)
{
    chip_id = T8122;
}
static void adt_lock_fails(void)
{
    protect_sticks = false;
}
static void adt_ext1_oslog_carveout(void)
{
    mcc_carveouts[0] = (struct mcc_carveout){oslogs[2] + 0x8000, 0x4000};
    mcc_carveout_count = 1;
}
static void adt_other_soc(void)
{
    chip_id = 0x8112;
}

static const struct scenario scenarios[] = {
    {"publish", PUBLISH},
    {"publish-prelocked", PUBLISH, NULL, {.prelocked = true}},
    {"publish-disp-heap", PUBLISH, NULL, {.disp_heap = true}},
    {"publish-dcp-vram", PUBLISH, NULL, {.dcp_vram = true}},
    {"publish-piodma-empty", PUBLISH, NULL, {.piodma_empty = true}},
    {"publish-oslog-in-memreserve", PUBLISH, NULL, {}, {.memreserve_oslog = true}},
    {"oslog-dcp-reserved", PUBLISH, NULL, {}, {.resv_dcp_oslog = true}, NULL, .oslog = 6},
    {"oslog-ext1-carveout", PUBLISH, NULL, {}, {}, adt_ext1_oslog_carveout, .oslog = 3},
    {"oslog-ext0-reserved", REFUSE, "device tree update", {}, {.resv_ext0_oslog = true}, NULL,
     .oslog = 5},
    {"repeat", PUBLISH},
    {"wrong-chip", NOTHING, NULL, {}, {}, adt_wrong_chip, .oslog = OSLOG_NONE},
    {"wrong-board", NOTHING, NULL, {}, {.root_compat = "apple,j713"}, NULL, .oslog = OSLOG_NONE},
    {"other-soc", NOTHING, NULL, {}, {.root_compat = "apple,j473", .soc_compat = "apple,t8112"},
     adt_other_soc, .oslog = OSLOG_NONE},
    {"old-dt", NOTHING, NULL, {}, {.no_dcpext0 = true}},
    {"wrong-dcp-compat", NOTHING, NULL, {}, {.wrong_dcp_compat = true}},
    {"dcp-enabled", REFUSE_NO_MMIO, "device tree", {}, {.dcp_okay = true}},
    {"dart-enabled", REFUSE_NO_MMIO, "device tree", {}, {.dart_okay = true}},
    {"wrong-sid", REFUSE_NO_MMIO, "device tree", {}, {.wrong_sid = true}},
    {"wrong-dart-reg", REFUSE_NO_MMIO, "device tree", {}, {.wrong_dart_reg = true}},
    {"no-mbox", REFUSE_NO_MMIO, "device tree", {}, {.no_mbox = true}},
    {"no-piodma", REFUSE_NO_MMIO, "device tree", {}, {.no_piodma = true}},
    {"no-disp-alias", REFUSE_NO_MMIO, "device tree", {}, {.no_disp_alias = true}},
    {"has-memory-region", REFUSE_NO_MMIO, "device tree", {}, {.memory_region = true}},
    {"wrong-os", REFUSE_NO_MMIO, "firmware", {}, {}, adt_wrong_os},
    {"bad-uuid", REFUSE_NO_MMIO, "ADT", {}, {}, adt_bad_uuid},
    {"adt-not-running", REFUSE_NO_MMIO, "ADT", {}, {}, adt_not_running},
    {"no-nub", REFUSE_NO_MMIO, "ADT", {}, {}, adt_no_nub, .oslog = 5},
    {"no-adt-dart", REFUSE_NO_MMIO, "ADT", {}, {}, adt_wrong_dart},
    {"seg-flags", REFUSE_NO_MMIO, "ADT", {}, {}, adt_seg_flags},
    {"text-in-ram", REFUSE_NO_MMIO, "ADT", {}, {}, adt_text_in_ram},
    {"fb-outside-vram", REFUSE_NO_MMIO, "ADT", {}, {}, adt_fb_outside},
    {"carveout", REFUSE_NO_MMIO, "ADT", {}, {}, adt_carveout},
    {"power-off", REFUSE, "power", {.power_off = true}},
    {"cpu-stopped", REFUSE, "coprocessor", {.cpu_stopped = true}},
    {"disp-ttbr-invalid", REFUSE, "DART mappings", {.disp_ttbr_invalid = true}},
    {"disp-no-fb", REFUSE, "DART mappings", {.disp_no_fb = true}},
    {"stray-map", REFUSE, "DART mappings", {.stray_map = true}},
    {"oslog-mapped", REFUSE, "DART mappings", {.oslog_mapped = true}},
    {"text-wrong-dva", REFUSE, "DART mappings", {.text_wrong_dva = true}},
    {"text-partial", REFUSE, "DART mappings", {.text_partial = true}},
    {"pt-outside", REFUSE, "DART mappings", {.pt_outside = true}},
    {"resv-conflict", REFUSE, "device tree update", {}, {.resv_vram = true}},
    {"oslog-memreserve-straddle", REFUSE, "device tree update", {},
     {.memreserve_oslog_straddle = true}, NULL, .oslog = 1},
    {"dt-full", REFUSE, "device tree update", {}, {.free_space = 256}, NULL, .oslog = OSLOG_NONE},
    {"lock-fails", REFUSE, "DART lock", {}, {}, adt_lock_fails},
};

/* After the __OS_LOG pass: exactly the expected dcp-oslog nodes, each once, nothing else */
static void check_oslogs(void *dt, unsigned want)
{
    int resv = fdt_path_offset(dt, "/reserved-memory"), node, count = 0;
    unsigned seen = 0;
    fdt_for_each_subnode(node, dt, resv)
    {
        const char *name = fdt_get_name(dt, node, NULL);
        if (!name || strncmp(name, "dcp-oslog@", 10))
            continue;
        count++;
        int len = 0;
        const fdt64_t *reg = fdt_getprop(dt, node, "reg", &len);
        u64 base = reg && len == 16 ? fdt64_ld(&reg[0]) : 0;
        for (int i = 0; i < 3; i++) {
            char want_name[48];
            snprintf(want_name, sizeof(want_name), "dcp-oslog@%llx", (unsigned long long)oslogs[i]);
            if (base != oslogs[i])
                continue;
            seen |= 1u << i;
            check(!strcmp(name, want_name) && fdt64_ld(&reg[1]) == OSLOG_SIZE &&
                      fdt_getprop(dt, node, "no-map", NULL) &&
                      fdt_getprop(dt, node, "apple,dcp-os-log", NULL) &&
                      !fdt_getprop(dt, node, "iommu-addresses", NULL),
                  "/reserved-memory/%s: 0x%llx+0x%llx, no-map, apple,dcp-os-log", name,
                  (unsigned long long)oslogs[i], (unsigned long long)OSLOG_SIZE);
        }
    }
    int n = !!(want & 1) + !!(want & 2) + !!(want & 4);
    check(count == n && seen == want, "%d __OS_LOG node(s) (mask 0x%x), no duplicate (got %d, 0x%x)",
          n, want, count, seen);
}

/* The __OS_LOG pass kboot.c runs after the HDMI handoff on J773g */
static void oslog_pass(const struct scenario *sc, void *dt, int size)
{
    unsigned want = sc->oslog == OSLOG_NONE ? 0 : sc->oslog ? sc->oslog : 7;
    void *before = malloc(size);
    memcpy(before, dt, size);
    int reads = mmio_reads, writes = mmio_writes;
    check(dt_set_display_handoff(dt) == 0, "dt_set_display_handoff() returns 0");
    check(mmio_reads == reads && mmio_writes == writes, "the __OS_LOG pass touches no register");
    if (sc->oslog == OSLOG_NONE && strcmp(sc->name, "dt-full"))
        check(!memcmp(before, dt, size), "the __OS_LOG pass leaves the device tree alone");
    check_oslogs(dt, want);
    check(fdt_check_full(dt, fdt_totalsize(dt)) == 0, "the device tree is valid after both");
    free(before);
}

static int run(const struct scenario *sc)
{
    world(&sc->hw);
    if (sc->adt)
        sc->adt();
    void *dt = build_dt(&sc->dt);
    int size = fdt_totalsize(dt);
    void *orig = malloc(size);
    memcpy(orig, dt, size);

    check(dt_set_j773g_hdmi_handoff(dt) == 0, "dt_set_j773g_hdmi_handoff() returns 0");
    check(fdt_totalsize(dt) == (u32)size, "the device tree keeps its size");

    switch (sc->outcome) {
        case PUBLISH:
            check_published(dt, &sc->hw);
            if (!strcmp(sc->name, "repeat")) {
                memcpy(orig, dt, size);
                int reads = mmio_reads, writes = mmio_writes;
                check(dt_set_j773g_hdmi_handoff(dt) == 0, "the second call returns 0");
                check(!memcmp(orig, dt, size), "the second call leaves the device tree alone");
                check(mmio_reads == reads && mmio_writes == writes,
                      "the second call touches no register");
            }
            break;
        case NOTHING:
            check(!memcmp(orig, dt, size), "nothing done: the device tree is byte for byte unchanged");
            check(!mmio_reads && !mmio_writes, "nothing done: no MMIO access at all");
            break;
        case REFUSE:
        case REFUSE_NO_MMIO: {
            char want[64];
            snprintf(want, sizeof(want), "refused: %s", sc->step);
            const char *res = prop_str_of(dt, "/chosen", RESULT);
            check(res && !strcmp(res, want), "/chosen/" RESULT " = \"%s\" (got \"%s\")", want,
                  res ? res : "none");
            /* the same result on the original: everything else must be byte for byte the same */
            int chosen = fdt_path_offset(orig, "/chosen");
            fdt_setprop_string(orig, chosen, RESULT, want);
            check(!memcmp(orig, dt, size),
                  "refused: nothing but the result in the device tree (before the __OS_LOG pass)");
            check(sc->dt.dcp_okay || !status_okay(dt, DCP_PATH), "refused: dcpext0 not enabled");
            if (!strcmp(sc->name, "lock-fails"))
                check(mmio_writes == 1 && !(peek32(DART_DISP + 0x200) & 1),
                      "refused: one lock attempt on dart-dcpext0, dart-dispext0 left alone");
            else
                check(!mmio_writes, "refused: no register written (no DART locked)");
            if (sc->outcome == REFUSE_NO_MMIO)
                check(!mmio_reads, "refused before any register access");
            break;
        }
    }
    oslog_pass(sc, dt, size);
    if (!strcmp(sc->name, "repeat")) {
        memcpy(orig, dt, size);
        check(dt_set_j773g_hdmi_handoff(dt) == 0 && dt_set_display_handoff(dt) == 0,
              "a second boot pass returns 0");
        check(!memcmp(orig, dt, size), "a second boot pass leaves the device tree alone");
    }
    free(orig);
    free(dt);
    return failures;
}

int main(int argc, char **argv)
{
    setvbuf(stdout, NULL, _IONBF, 0);
    if (argc != 2) {
        fprintf(stderr, "usage: %s list | <scenario>\n", argv[0]);
        return 2;
    }
    for (size_t i = 0; i < ARRAY_SIZE(scenarios); i++) {
        if (!strcmp(argv[1], "list")) {
            printf("%s\n", scenarios[i].name);
            continue;
        }
        if (strcmp(argv[1], scenarios[i].name))
            continue;
        if (run(&scenarios[i])) {
            printf("%s: %d check(s) failed\n", scenarios[i].name, failures);
            return 1;
        }
        printf("%s: pass\n", scenarios[i].name);
        return 0;
    }
    if (!strcmp(argv[1], "list"))
        return 0;
    fprintf(stderr, "unknown scenario %s\n", argv[1]);
    return 2;
}
