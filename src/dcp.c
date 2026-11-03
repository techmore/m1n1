/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "../config.h"

#include "adt.h"
#include "afk.h"
#include "dcp.h"
#include "dcp_iboot.h"
#include "display.h"
#include "exception.h"
#include "firmware.h"
#include "malloc.h"
#include "pmgr.h"
#include "rtkit.h"
#include "smc.h"
#include "string.h"
#include "utils.h"
#include "vsprintf.h"
#include "xnuboot.h"

#include "dcp/aux_ep.h"
#include "dcp/dptx_phy.h"

struct adt_function_smc_gpio {
    u32 phandle;
    char four_cc[4];
    u32 gpio;
    u32 unk;
};

static char dcp_pmgr_dev[16] = "DISP0_CPU0";
static u32 dcp_die;
static u32 dcp_options;
static int dcp_exc_base;

void dcp_set_exc_base(int count)
{
    dcp_exc_base = count;
}

/* a CPU exception was caught (and skipped) since the display attempt began */
static bool dcp_faulted(void)
{
    if (exc_count == dcp_exc_base)
        return false;
    printf("dcp: CPU exception caught, stopping\n");
    return true;
}

/*
 * Set when a DCP may still be running after a failure: its endpoints or
 * RTKit did not stop. Then its DART tables, RTKit buffers and IOVAs are kept
 * (the firmware may still use them) and nothing touches the DCP again.
 */
static bool dcp_active_after_failure;

bool dcp_left_active(void)
{
    return dcp_active_after_failure;
}

/* Stop what dcp_init() started; release memory only once the IOP has stopped. */
static void dcp_release(dcp_dev_t *dcp, bool stop_iop)
{
    bool stopped = true;

    if (dcp->afk && afk_epic_shutdown(dcp->afk) < 0)
        stopped = false;
    dcp->afk = NULL;
    if (stop_iop && stopped && !rtkit_quiesce(dcp->rtkit))
        stopped = false;

    if (!stopped) {
        printf("dcp: DCP did not stop; keeping its DART tables and buffers\n");
        dcp_active_after_failure = true;
        return;
    }

    if (dcp->compact) {
        /* J773g: DARTs were only read; buffers live in reserved /vram */
        if (dcp->rtkit)
            rtkit_free(dcp->rtkit);
        if (dcp->asc)
            asc_free(dcp->asc);
        dart_free_readonly(dcp->dart_disp);
        dart_free_readonly(dcp->dart_dcp);
        free(dcp);
        return;
    }

    if (dcp->rtkit)
        rtkit_free(dcp->rtkit);
    iovad_shutdown(dcp->iovad_dcp, dcp->dart_dcp);
    dart_shutdown(dcp->dart_disp);
    dart_shutdown(dcp->dart_dcp);
    free(dcp);
}

void dcp_set_options(u32 options)
{
    dcp_options = options;
}

/* SMC GPIO request, as Linux gpio-macsmc sets an output high (CMD_OUTPUT | 1) */
#define SMC_GPIO_OUTPUT_HIGH 0x1000001

/*
 * J773g: switch on the HDMI and DP-to-HDMI converter supplies, like the
 * Gravity Linux DCP driver does through hdmi-pwren-gpios (SMC gP0f) and
 * dp2hdmi-pwren-gpios (SMC gP1a). The ADT gives the SMC keys.
 */
static void dcp_t8132_hdmi_power(const display_config_t *cfg)
{
    struct adt_function_smc_gpio dp2hdmi_pwr, hdmi_pwr;

    int node = adt_path_offset(adt, cfg->dp2hdmi_gpio);
    if (node < 0) {
        printf("dcp: no dp2hdmi-gpio node '%s'\n", cfg->dp2hdmi_gpio);
        return;
    }
    if (adt_getprop_copy(adt, node, "function-hdmi_pwr_en", &hdmi_pwr, sizeof(hdmi_pwr)) < 0 ||
        adt_getprop_copy(adt, node, "function-dp2hdmi_pwr_en", &dp2hdmi_pwr,
                         sizeof(dp2hdmi_pwr)) < 0) {
        printf("dcp: HDMI power GPIOs not found\n");
        return;
    }

    smc_dev_t *smc = smc_init();
    if (!smc) {
        printf("dcp: SMC unavailable, HDMI power left as is\n");
        return;
    }

    int ret = smc_write_u32(smc, hdmi_pwr.gpio, SMC_GPIO_OUTPUT_HIGH);
    if (!ret)
        ret = smc_write_u32(smc, dp2hdmi_pwr.gpio, SMC_GPIO_OUTPUT_HIGH);
    printf("dcp: HDMI power GPIOs %.4s/%.4s on: %s\n", (char *)&hdmi_pwr.gpio,
           (char *)&dp2hdmi_pwr.gpio, ret ? "failed" : "ok");
    smc_shutdown(smc);

    if (!ret)
        mdelay(100);
}

static int dcp_t8132_start_aux(dcp_dev_t *dcp, int endpoint)
{
    for (int i = 0; i < DCP_MAX_AUX_EP; i++) {
        if (!dcp->aux_ep[i]) {
            dcp->aux_ep[i] = dcp_aux_ep_start(dcp->afk, endpoint);
            return dcp->aux_ep[i] ? 0 : -1;
        }
    }
    return -1;
}

/*
 * Bring up the HDMI pipe of J773g (T8132) with macOS 26.6 DCP firmware. The
 * endpoint order follows the Gravity Linux driver's dcp_start() for 26.6,
 * which is known to drive this HDMI port; the iBoot interface is then used
 * for the mode set.
 */
static int dcp_t8132_hdmi_init(dcp_dev_t *dcp, const display_config_t *cfg)
{
    bool minimal = dcp_options & DCP_OPT_MINIMAL;
    u32 stage = DCP_OPT_STAGE(dcp_options);

    if (!(dcp_options & DCP_OPT_NO_SMC))
        dcp_t8132_hdmi_power(cfg);

    /* the only PMGR write in this path, and only if iBoot left the PHY off */
    if (!pmgr_adt_power_is_active(cfg->dptx_phy)) {
        printf("dcp: powering on %s\n", cfg->dptx_phy);
        pmgr_adt_power_enable(cfg->dptx_phy);
        mdelay(25);
    }

    if (stage == 3) {
        printf("dcp: stage 3 (HDMI power) reached, stopping\n");
        return -1;
    }

    dcp->die = cfg->die;
    dcp->phy = dptx_phy_init(cfg->dptx_phy, cfg->dcp_index);
    if (!dcp->phy) {
        printf("dcp: failed to init dptx-phy '%s'\n", cfg->dptx_phy);
        return -1;
    }

    if (!minimal) {
        if (dcp_t8132_start_aux(dcp, DCP_SYSTEM_ENDPOINT))
            return -1;
        /* 26.6 starts the IOMFB transport before it builds the HDMI graph */
        if (!(dcp_options & DCP_OPT_NO_IOMFB) &&
            !rtkit_start_ep(dcp->rtkit, DCP_IOMFB_ENDPOINT)) {
            printf("dcp: failed to start the IOMFB endpoint\n");
            return -1;
        }
        if (dcp_t8132_start_aux(dcp, DCP_DPDEV_ENDPOINT))
            return -1;
    }

    if (dcp_t8132_start_aux(dcp, DCP_DPAVCTRL_ENDPOINT))
        return -1;
    for (int i = 0; i < DCP_MAX_AUX_EP; i++) {
        if (dcp->aux_ep[i] && afk_epic_endpoint(dcp->aux_ep[i]) == DCP_DPAVCTRL_ENDPOINT &&
            afk_epic_wait_services(dcp->aux_ep[i], 4, 1000000) < 4)
            printf("dcp: DPAV controllers incomplete\n");
    }

    if (!minimal) {
        if (dcp_t8132_start_aux(dcp, DCP_DPSAC_ENDPOINT) ||
            dcp_t8132_start_aux(dcp, DCP_AV_ENDPOINT) ||
            dcp_t8132_start_aux(dcp, DCP_REMOTE_ALLOC_ENDPOINT))
            return -1;
    }

    dcp->iboot = dcp_ib_init(dcp);
    if (!dcp->iboot) {
        printf("dcp: failed to start the iBoot interface\n");
        return -1;
    }

    if (!minimal) {
        if (dcp_t8132_start_aux(dcp, DCP_EPIC2C_ENDPOINT) ||
            dcp_t8132_start_aux(dcp, DCP_DPAVSERV_ENDPOINT) ||
            dcp_t8132_start_aux(dcp, DCP_EXPERT_ENDPOINT) ||
            dcp_t8132_start_aux(dcp, DCP_EPIC25_ENDPOINT))
            return -1;
    }

    dcp->dptx_ep = dcp_dptx_init(dcp, cfg->num_dptxports);
    if (!dcp->dptx_ep) {
        printf("dcp: failed to initialize dptx-port endpoint\n");
        return -1;
    }

    if (stage == 4) {
        printf("dcp: stage 4 (endpoints up) reached, stopping\n");
        return -1;
    }

    return 0;
}

/* Read-only checks before m1n1 touches the J773g display hardware at all. */
static bool dcp_t8132_precheck(const display_config_t *cfg)
{
    const char *paths[] = {cfg->dcp, cfg->dcp_dart, cfg->disp_dart, cfg->dptx_phy,
                           cfg->dp2hdmi_gpio};

    for (size_t i = 0; i < ARRAY_SIZE(paths); i++) {
        if (adt_path_offset(adt, paths[i]) < 0) {
            printf("dcp: precheck: %s missing\n", paths[i]);
            return false;
        }
    }
    if (!adt_is_compatible(adt, adt_path_offset(adt, cfg->dptx_phy), "dptx-phy,t8122")) {
        printf("dcp: precheck: unexpected dptx-phy\n");
        return false;
    }
    /* m1n1 must not allocate DART tables of its own for the DCP */
    if (!adt_getprop(adt, adt_path_offset(adt, cfg->dcp_dart), "pt-region-23", NULL) ||
        !adt_getprop(adt, adt_path_offset(adt, cfg->disp_dart), "pt-region-0", NULL)) {
        printf("dcp: precheck: DART page table regions missing\n");
        return false;
    }
    return true;
}

static int dcp_hdmi_dptx_init(dcp_dev_t *dcp, const display_config_t *cfg)
{
    int node = adt_path_offset(adt, cfg->dp2hdmi_gpio);
    if (node < 0) {
        printf("dcp: failed to find dp2hdmi-gpio node '%s'\n", cfg->dp2hdmi_gpio);
        return -1;
    }
    struct adt_function_smc_gpio dp2hdmi_pwr, hdmi_pwr;

    int err =
        adt_getprop_copy(adt, node, "function-dp2hdmi_pwr_en", &dp2hdmi_pwr, sizeof(dp2hdmi_pwr));
    if (err < 0)
        printf("dcp: failed to get dp2hdmi_pwr_en gpio\n");
    else
        dcp->dp2hdmi_pwr_gpio = dp2hdmi_pwr.gpio;
    err = adt_getprop_copy(adt, node, "function-hdmi_pwr_en", &hdmi_pwr, sizeof(hdmi_pwr));
    if (err < 0)
        printf("dcp: failed to get hdmi_pwr_en gpio\n");
    else
        dcp->hdmi_pwr_gpio = hdmi_pwr.gpio;

    if (dcp->dp2hdmi_pwr_gpio && dcp->hdmi_pwr_gpio) {
        smc_dev_t *smc = smc_init();
        if (smc) {
            smc_write_u32(smc, dcp->dp2hdmi_pwr_gpio, 0x800001);
            smc_write_u32(smc, dcp->hdmi_pwr_gpio, 0x800001);
            smc_shutdown(smc);
        }
    }

    dcp->die = cfg->die;

    dcp->phy = dptx_phy_init(cfg->dptx_phy, cfg->dcp_index);
    if (!dcp->phy) {
        printf("dcp: failed to init (lp)dptx-phy '%s'\n", cfg->dptx_phy);
        return -1;
    }

    dcp->dpav_ep = dcp_dpav_init(dcp);
    if (!dcp->dpav_ep) {
        printf("dcp: failed to initialize dpav endpoint\n");
        return -1;
    }

    dcp->dptx_ep = dcp_dptx_init(dcp, cfg->num_dptxports);
    if (!dcp->dptx_ep) {
        printf("dcp: failed to initialize dptx-port endpoint\n");
        dcp_dpav_shutdown(dcp->dpav_ep);
        return -1;
    }

#ifdef RTKIT_SYSLOG
    // start system endpoint when extended logging is requested
    dcp->system_ep = dcp_system_init(dcp);
    if (!dcp->system_ep) {
        printf("dcp: failed to initialize system endpoint\n");
        dcp_dptx_shutdown(dcp->dptx_ep);
        dcp_dpav_shutdown(dcp->dpav_ep);
        return -1;
    }

    dcp_system_set_property_u64(dcp->system_ep, "gAFKConfigLogMask", 0xffff);
#endif

    return 0;
}

int dcp_connect_dptx(dcp_dev_t *dcp)
{
    if (dcp->dptx_ep && dcp->phy) {
        return dcp_dptx_connect(dcp->dptx_ep, dcp->phy, dcp->die, 0);
    }

    return 0;
}

int dcp_work(dcp_dev_t *dcp)
{
    return afk_epic_work(dcp->afk, -1);
}

static int dcp_create_firmware_mappings(const display_config_t *cfg, dcp_dev_t *dcp)
{
    int created = 0;
    int dcp_node = adt_path_offset(adt, cfg->dcp);
    int node = adt_first_child_offset(adt, dcp_node);
    if (node < 0) {
        printf("dcp: iop-dcp*-nub not found!\n");
        return -1;
    }

    u64 asc_dram_mask;
    if (ADT_GETPROP(adt, node, "asc-dram-mask", &asc_dram_mask) < 0)
        asc_dram_mask = 0;

    const struct adt_segment_ranges *seg;
    u32 segments_len;

    seg = adt_getprop(adt, node, "segment-ranges", &segments_len);
    unsigned int count = segments_len / sizeof(*seg);

    for (unsigned int i = 0; i < count; i++) {
        u64 iova = seg[i].remap & ~asc_dram_mask;
        if (dart_translate_silent(dcp->dart_dcp, iova))
            continue;

        size_t len = ALIGN_UP(seg[i].size, SZ_16K);
        u32 flags = i == 0 ? 0b0100 : 0; // TEXT gets this bit set?
        printf("dcp: Mapping segment #%u %lx -> %lx [%lx]\n", i, iova, seg[i].phys, len);
        if (dart_map_flags(dcp->dart_dcp, iova, (void *)seg[i].phys, len, flags)) {
            printf("dcp: Failed to map segment\n");
            return -1;
        }
        created++;
    }

    return created;
}

/* The last 64 KiB of /vram hold the probe record; the buffer pool ends there. */
#define DCP_T8132_RECORD_SIZE 0x10000

/* Read-only: are the DCP firmware segments mapped in its DART, as iBoot left them? */
static int dcp_t8132_check_firmware_mappings(const display_config_t *cfg, dcp_dev_t *dcp,
                                             int *n_segments)
{
    int node = adt_first_child_offset(adt, adt_path_offset(adt, cfg->dcp));
    if (node < 0)
        return -1;

    u64 mask = 0;
    if (ADT_GETPROP(adt, node, "asc-dram-mask", &mask) < 0)
        mask = 0;

    u32 len;
    const struct adt_segment_ranges *seg = adt_getprop(adt, node, "segment-ranges", &len);
    if (!seg)
        return -1;

    int missing = 0;
    *n_segments = len / sizeof(*seg);
    for (int i = 0; i < *n_segments; i++) {
        u64 iova = seg[i].remap & ~mask;
        void *pa = dart_translate_silent(dcp->dart_dcp, iova);
        printf("dcp: segment %d: dva 0x%lx -> %p (ADT phys 0x%lx)%s\n", i, iova, pa,
               seg[i].phys, pa ? "" : " NOT MAPPED");
        if (!pa)
            missing++;
    }
    return missing;
}

/*
 * Buffers for the DCP come from /vram above the framebuffer area: iBoot maps
 * /vram for the DCP, and it is reserved memory that the OS never reuses, so no
 * DART page table needs to be written and nothing is freed at hand-off.
 */
static int dcp_t8132_find_pool(dcp_dev_t *dcp, void **phys, u64 *dva, size_t *size)
{
    u64 pa, vsize;

    if (display_get_vram(&pa, &vsize))
        return -1;

    u64 start = pa + DCP_T8132_FB_MAX;
    u64 end = pa + vsize - DCP_T8132_RECORD_SIZE;
    if (end <= start + SZ_1M) {
        printf("dcp: /vram (0x%lx bytes) too small for a buffer pool\n", vsize);
        return -1;
    }

    u64 d = dart_search(dcp->dart_dcp, (void *)start);
    if (DART_IS_ERR(d)) {
        printf("dcp: /vram+0x%x is not mapped for the DCP\n", DCP_T8132_FB_MAX);
        return -1;
    }

    u64 len = 0;
    while (start + len < end && dart_translate_silent(dcp->dart_dcp, d + len) == (void *)(start + len))
        len += SZ_16K;

    printf("dcp: buffer pool /vram+0x%x: 0x%lx bytes at DCP dva 0x%lx\n", DCP_T8132_FB_MAX, len, d);
    if (len < SZ_1M)
        return -1;

    *phys = (void *)start;
    *dva = d;
    *size = len;
    return 0;
}

static bool dcp_t8132_firmware_ok(void)
{
    /* the compact protocol is known for 26.6 DCP firmware only */
    if (os_firmware.version != V26_6 && os_firmware.version != V26_6_2) {
        printf("dcp: T8132 HDMI needs 26.6 OS firmware (have %s)\n", os_firmware.string);
        return false;
    }
    return true;
}

static u32 dcp_t8132_sid(const display_config_t *cfg)
{
    u32 sid = 0;
    int node = adt_first_child_offset(adt, adt_path_offset(adt, cfg->dcp_dart));
    if (node < 0 || ADT_GETPROP(adt, node, "reg", &sid) < 0)
        return 0xffffffff;
    return sid;
}

/*
 * J773g bring-up without writing any iBoot-owned DART: the DARTs are adopted
 * read-only, the firmware must already be mapped, RTKit/AFK buffers come from
 * /vram, and the framebuffer is the iBoot-mapped /vram. Stages 1 and 2 make no
 * PMGR writes either.
 */
static dcp_dev_t *dcp_init_t8132(const display_config_t *cfg)
{
    u32 stage = DCP_OPT_STAGE(dcp_options);
    int n_segments = 0;
    void *pool;
    u64 pool_dva;
    size_t pool_size;

    if (dcp_active_after_failure || !dcp_t8132_precheck(cfg) || !dcp_t8132_firmware_ok())
        return NULL;

    afk_epic_set_trace(dcp_options & DCP_OPT_TRACE);
    strncpy(dcp_pmgr_dev, cfg->pmgr_dev, sizeof(dcp_pmgr_dev));
    dcp_die = cfg->die;

    if (!pmgr_adt_power_is_active(cfg->dcp)) {
        printf("dcp: DCP is not powered, iBoot did not leave it running; skipping\n");
        return NULL;
    }

    u32 sid = dcp_t8132_sid(cfg);
    if (sid == 0xffffffff) {
        printf("dcp: failed to read dart stream ID!\n");
        return NULL;
    }

    dcp_dev_t *dcp = calloc(1, sizeof(dcp_dev_t));
    if (!dcp)
        return NULL;
    dcp->compact = true;

    dcp->dart_dcp = dart_init_adt_readonly(cfg->dcp_dart, 0, sid);
    if (!dcp->dart_dcp || dcp_faulted())
        goto out_dart_dcp;
    dcp->dart_disp = dart_init_adt_readonly(cfg->disp_dart, 0, 0);
    if (!dcp->dart_disp || dcp_faulted())
        goto out_darts;

    if (dcp_t8132_check_firmware_mappings(cfg, dcp, &n_segments) != 0 || !n_segments) {
        printf("dcp: DCP firmware is not mapped as iBoot leaves it running; skipping\n");
        goto out_darts;
    }
    if (dcp_t8132_find_pool(dcp, &pool, &pool_dva, &pool_size) < 0 || dcp_faulted())
        goto out_darts;

    if (stage == 1) {
        printf("dcp: stage 1 (state checked, nothing written) reached, stopping\n");
        goto out_darts;
    }

    dcp->asc = asc_init(cfg->dcp);
    if (!dcp->asc)
        goto out_darts;

    dcp->rtkit = rtkit_init("dcp", dcp->asc, dcp->dart_dcp, NULL, NULL, false);
    if (!dcp->rtkit)
        goto out_asc;
    rtkit_set_buffer_pool(dcp->rtkit, pool, pool_dva, pool_size);

    if (!rtkit_boot(dcp->rtkit) || dcp_faulted()) {
        printf("dcp: failed to boot RTKit\n");
        goto out_rtkit;
    }

    if (stage == 2) {
        printf("dcp: stage 2 (RTKit handshake) reached, stopping\n");
        goto out_rtkit;
    }

    dcp->afk = afk_epic_init(dcp->rtkit);
    if (!dcp->afk)
        goto out_rtkit;
    afk_epic_set_compact(dcp->afk, true);

    if (dcp_t8132_hdmi_init(dcp, cfg) < 0 || dcp_faulted())
        goto out_rtkit;

    return dcp;

out_rtkit:
    /* the IOP was contacted: stop it; memory is kept if it does not stop */
    dcp_release(dcp, true);
    return NULL;
out_asc:
    asc_free(dcp->asc);
out_darts:
    if (dcp->dart_disp)
        dart_free_readonly(dcp->dart_disp);
out_dart_dcp:
    if (dcp->dart_dcp)
        dart_free_readonly(dcp->dart_dcp);
    free(dcp);
    return NULL;
}

/* Stage A: a read-only look at the J773g display state, kept in /vram too. */
static char probe_record[6144];
static size_t probe_len;

static void plog(const char *fmt, ...)
{
    va_list ap;
    char line[192];

    va_start(ap, fmt);
    vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    printf("%s", line);
    size_t n = strlen(line);
    if (probe_len + n < sizeof(probe_record) - 1) {
        memcpy(probe_record + probe_len, line, n);
        probe_len += n;
    }
}

/* 1920x1080, 4 bytes per pixel, plus m1n1's 24-page slack, in 16 KiB pages */
#define DCP_T8132_FB_1080P_PAGES ((1920 * 1080 * 4 + 24 * 0x4000 + 0x3fff) / 0x4000)

static struct dart_l2_info probe_inv[2][64];
static u8 probe_free[2][0x800];

/* Read-only: L1/L2 layout of one DART and how much of its pt-region is used. */
static int dcp_t8132_inventory(dart_dev_t *dart, const char *name, struct dart_l2_info *inv)
{
    u64 lo, hi;
    int pages, used;

    int n = dart_ro_inventory(dart, inv, 64);
    dart_ro_region_usage(dart, &lo, &hi, &pages, &used);
    plog("%s: pt-region 0x%lx..0x%lx %d pages, %d hold tables; %d valid L1 slots\n", name, lo,
         hi, pages, used, n);
    for (int i = 0; i < n && i < 12; i++)
        plog("  %s t%u l1[%u] -> 0x%lx %s %u leaves\n", name, inv[i].ttbr, inv[i].l1, inv[i].l2,
             inv[i].in_region ? "in-region" : "OUTSIDE", inv[i].valid);
    return n;
}

/*
 * A window for a 1080p framebuffer that persists past m1n1: the same IOVA,
 * free on both DARTs, inside L2 tables that live in the reserved pt-regions.
 */
struct dcp_t8132_window {
    u64 iova;  /* first page, never IOVA 0 */
    int pages; /* free pages from iova on, in both DARTs */
};

static void dcp_t8132_common_window(dcp_dev_t *probe, int na, int nb, int need,
                                    struct dcp_t8132_window *win)
{
    int found = 0, both_unused = 0;

    win->iova = 0;
    win->pages = 0;

    for (int i = 0; i < na; i++) {
        struct dart_l2_info *a = &probe_inv[0][i];
        if (!a->in_region)
            continue;
        for (int j = 0; j < nb; j++) {
            struct dart_l2_info *b = &probe_inv[1][j];
            if (!b->in_region || b->ttbr != a->ttbr || b->l1 != a->l1)
                continue;
            if (dart_ro_l2_free_map(probe->dart_dcp, a->ttbr, a->l1, probe_free[0]) ||
                dart_ro_l2_free_map(probe->dart_disp, b->ttbr, b->l1, probe_free[1]))
                continue;
            int best = 0, best_start = 0, run = 0;
            for (int k = 0; k < 0x800; k++) {
                run = (probe_free[0][k] && probe_free[1][k]) ? run + 1 : 0;
                if (run > best) {
                    best = run;
                    best_start = k - run + 1;
                }
            }
            u64 iova = ((u64)a->ttbr << 36) | ((u64)a->l1 << 25) | ((u64)best_start << 14);
            plog("common t%u l1[%u]: %d free pages at iova 0x%lx%s\n", a->ttbr, a->l1, best, iova,
                 best >= DCP_T8132_FB_1080P_PAGES ? " (fits 1080p)" : "");
            found++;
            /* keep IOVA 0 unused: a zero DMA address reads as "no buffer" */
            int start = best_start ? best_start : 1;
            int len = best - (start - best_start);
            if (need && len >= need && !win->pages) {
                win->iova = ((u64)a->ttbr << 36) | ((u64)a->l1 << 25) | ((u64)start << 14);
                win->pages = len;
            }
        }
    }
    if (!found)
        plog("no reserved-backed L1 slot is shared by both DARTs\n");

    /* slots where a new L2 table would be needed on both DARTs */
    for (u32 l1 = 0; l1 < 0x800; l1++) {
        bool used = false;
        for (int i = 0; i < na && !used; i++)
            used = probe_inv[0][i].ttbr == 0 && probe_inv[0][i].l1 == l1;
        for (int j = 0; j < nb && !used; j++)
            used = probe_inv[1][j].ttbr == 0 && probe_inv[1][j].l1 == l1;
        if (!used)
            both_unused++;
    }
    plog("t0 L1 slots unused by both DARTs: %d (1080p needs %d pages)\n", both_unused,
         DCP_T8132_FB_1080P_PAGES);
}

/*
 * probe,map: map all of /vram at one IOVA in both display DARTs, writing only
 * leaf PTEs into the L2 tables iBoot already keeps in the reserved
 * pt-regions. The DCP stays off. kboot then finds /vram mapped in both DARTs
 * and gives Linux matching iommu-addresses, so the IOVAs stay reserved.
 */
static void dcp_t8132_map_vram(dcp_dev_t *probe, const struct dcp_t8132_window *win, u64 pa,
                               u64 size)
{
    u64 iova = win->iova;

    if (!win->pages || !pa || !size || size % SZ_16K) {
        plog("map: no window for /vram (%d pages free, need %lu), nothing written\n",
             win->pages, size / SZ_16K);
        return;
    }
    plog("map: /vram 0x%lx size 0x%lx (%lu pages) -> iova 0x%lx in both DARTs\n", pa, size,
         size / SZ_16K, iova);

    /* what iBoot's own leaves look like next to the window, for comparison */
    for (u64 k = 0; k < 0x800; k++) {
        u64 v = dart_ro_pte(probe->dart_dcp, (iova & ~0x1ffffffUL) | (k << 14));
        if (v) {
            plog("map: dart-dcp sample leaf [%lu] 0x%lx\n", k, v);
            break;
        }
    }
    for (u64 k = 0; k < 0x800; k++) {
        u64 v = dart_ro_pte(probe->dart_disp, (iova & ~0x1ffffffUL) | (k << 14));
        if (v) {
            plog("map: dart-disp0 sample leaf [%lu] 0x%lx\n", k, v);
            break;
        }
    }

    if (dart_ro_map_existing(probe->dart_disp, iova, pa, size)) {
        plog("map: dart-disp0 refused, nothing written\n");
        return;
    }
    if (dcp_faulted())
        return;
    if (dart_ro_map_existing(probe->dart_dcp, iova, pa, size)) {
        plog("map: dart-dcp refused, undoing dart-disp0\n");
        dart_ro_unmap_existing(probe->dart_disp, iova, size);
        return;
    }
    if (dcp_faulted())
        return;

    plog("map: written; first leaf dcp 0x%lx disp0 0x%lx\n", dart_ro_pte(probe->dart_dcp, iova),
         dart_ro_pte(probe->dart_disp, iova));

    /* read back the way kboot will: search, then translate the last byte */
    u64 d_dcp = dart_search(probe->dart_dcp, (void *)pa);
    u64 d_disp = dart_search(probe->dart_disp, (void *)pa);
    u64 e_dcp = (u64)dart_translate(probe->dart_dcp, iova + size - 1);
    u64 e_disp = (u64)dart_translate(probe->dart_disp, iova + size - 1);
    bool ok = d_dcp == iova && d_disp == iova && e_dcp == pa + size - 1 && e_disp == pa + size - 1;
    plog("map: read back dcp 0x%lx..0x%lx disp0 0x%lx..0x%lx: %s\n", d_dcp, e_dcp, d_disp, e_disp,
         ok ? "OK" : "MISMATCH");
}

int dcp_t8132_probe(const display_config_t *cfg, bool map)
{
    u64 vram_pa = 0, vram_size = 0;
    int n_segments = 0;

    probe_len = 0;
    plog("M1N1-J773G-PROBE v2 (%s)\n", map ? "read-only + map /vram" : "read-only");
    plog("os firmware %s, ok=%d\n", os_firmware.string, dcp_t8132_firmware_ok());
    plog("boot fb 0x%lx %lux%lu stride %lu depth %lu\n", cur_boot_args.video.base,
         cur_boot_args.video.width, cur_boot_args.video.height, cur_boot_args.video.stride,
         cur_boot_args.video.depth);

    bool pre = dcp_t8132_precheck(cfg);
    plog("precheck %s\n", pre ? "ok" : "FAILED");
    if (dcp_faulted())
        goto faulted;

    bool dcp_on = pmgr_adt_power_is_active(cfg->dcp);
    bool phy_on = pmgr_adt_power_is_active(cfg->dptx_phy);
    plog("pmgr: dcp %s, dptx-phy %s\n", dcp_on ? "active" : "NOT active",
         phy_on ? "active" : "NOT active");
    if (dcp_faulted())
        goto faulted;

    if (display_get_vram(&vram_pa, &vram_size) == 0)
        plog("vram 0x%lx size 0x%lx\n", vram_pa, vram_size);
    else
        plog("vram lookup FAILED\n");

    u32 sid = dcp_t8132_sid(cfg);
    plog("dart-dcp sid %u\n", sid);

    dcp_dev_t probe = {0};
    if (pre && sid != 0xffffffff) {
        probe.dart_dcp = dart_init_adt_readonly(cfg->dcp_dart, 0, sid);
        probe.dart_disp = dart_init_adt_readonly(cfg->disp_dart, 0, 0);
    }
    plog("darts adopted read-only: dcp %d disp %d\n", !!probe.dart_dcp, !!probe.dart_disp);
    if (dcp_faulted())
        goto release;

    if (probe.dart_dcp && probe.dart_disp) {
        int na = dcp_t8132_inventory(probe.dart_dcp, "dart-dcp", probe_inv[0]);
        if (dcp_faulted())
            goto release;
        int nb = dcp_t8132_inventory(probe.dart_disp, "dart-disp0", probe_inv[1]);
        if (dcp_faulted())
            goto release;
        struct dcp_t8132_window win;
        dcp_t8132_common_window(&probe, na, nb, map ? (int)(vram_size / SZ_16K) : 0, &win);
        if (dcp_faulted())
            goto release;
        if (map) {
            dcp_t8132_map_vram(&probe, &win, vram_pa, vram_size);
            if (dcp_faulted())
                goto release;
        }
    }

    if (probe.dart_dcp) {
        int missing = dcp_t8132_check_firmware_mappings(cfg, &probe, &n_segments);
        plog("firmware segments %d, unmapped %d\n", n_segments, missing);
    }
    if (vram_pa && probe.dart_disp) {
        u64 d = dart_search(probe.dart_disp, (void *)vram_pa);
        plog("vram in dart-disp0: %s 0x%lx\n", DART_IS_ERR(d) ? "NO" : "dva", d);
    }
    if (vram_pa && probe.dart_dcp) {
        u64 d = dart_search(probe.dart_dcp, (void *)vram_pa);
        plog("vram in dart-dcp: %s 0x%lx\n", DART_IS_ERR(d) ? "NO" : "dva", d);
        void *pool;
        u64 pool_dva;
        size_t pool_size = 0;
        int ret = dcp_t8132_find_pool(&probe, &pool, &pool_dva, &pool_size);
        plog("buffer pool: %s, 0x%lx bytes\n", ret ? "NOT usable" : "usable", pool_size);
    }
    if (dcp_faulted())
        goto release;

    if (dcp_on) {
        asc_dev_t *asc = asc_init(cfg->dcp);
        if (asc) {
            plog("dcp cpu running: %d\n", asc_cpu_running(asc));
            asc_free(asc);
        }
    } else {
        plog("dcp cpu: not read (not powered)\n");
    }

release:
    if (probe.dart_disp)
        dart_free_readonly(probe.dart_disp);
    if (probe.dart_dcp)
        dart_free_readonly(probe.dart_dcp);

faulted:
    if (exc_count != dcp_exc_base)
        plog("CPU exceptions caught: %d (see the m1n1 log for addresses)\n",
             exc_count - dcp_exc_base);
    plog("end of probe\n");

    /* keep a copy at the end of /vram (reserved memory, readable from Linux) */
    if (vram_pa && vram_size > DCP_T8132_RECORD_SIZE + cur_boot_args.video.stride *
                                                           cur_boot_args.video.height) {
        void *rec = (void *)(vram_pa + vram_size - DCP_T8132_RECORD_SIZE);
        memset(rec, 0, sizeof(probe_record) + 64);
        memcpy(rec, probe_record, probe_len);
        dma_wmb();
        printf("display: probe record at 0x%lx\n", (u64)rec);
    }

    return 0;
}

dcp_dev_t *dcp_init(const display_config_t *cfg)
{
    u32 sid;

    if (cfg && cfg->compact)
        return dcp_init_t8132(cfg);

    if (cfg && cfg->dptx_phy[0]) {
        if (os_firmware.version != V13_5) {
            printf("dcp: dtpx-port is only supported with V13_5 OS firmware.\n");
            return NULL;
        }

        strncpy(dcp_pmgr_dev, cfg->pmgr_dev, sizeof(dcp_pmgr_dev));
        dcp_die = cfg->die;
        pmgr_adt_power_enable(cfg->dcp);
        pmgr_adt_power_enable(cfg->dptx_phy);
        mdelay(25);
    }

    int dart_node = adt_path_offset(adt, cfg->dcp_dart);
    int node = adt_first_child_offset(adt, dart_node);
    if (node < 0) {
        printf("dcp: mapper-dcp* not found!\n");
        return NULL;
    }
    if (ADT_GETPROP(adt, node, "reg", &sid) < 0) {
        printf("dcp: failed to read dart stream ID!\n");
        return NULL;
    }

    dcp_dev_t *dcp = calloc(1, sizeof(dcp_dev_t));
    if (!dcp)
        return NULL;

    dcp->dart_dcp = dart_init_adt(cfg->dcp_dart, 0, sid, true);
    if (!dcp->dart_dcp) {
        printf("dcp: failed to initialize DCP DART\n");
        goto out_free;
    }
    u64 vm_base = dart_vm_base(dcp->dart_dcp);
    dart_setup_pt_region(dcp->dart_dcp, cfg->dcp_dart, sid, vm_base);

    dcp->dart_disp = dart_init_adt(cfg->disp_dart, 0, 0, true);
    if (!dcp->dart_disp) {
        printf("dcp: failed to initialize DISP DART\n");
        goto out_dart_dcp;
    }
    // set disp0's page tables at dart-dcp's vm-base
    dart_setup_pt_region(dcp->dart_disp, cfg->disp_dart, 0, vm_base);

    dcp->iovad_dcp = iovad_init(vm_base + 0x10000000, vm_base + 0x20000000);

    int ret = dcp_create_firmware_mappings(cfg, dcp);
    if (ret < 0) {
        printf("dcp: failed to create firmware mappings\n");
        goto out_iovad;
    }
    if (ret > 0) {
        pmgr_reset(dcp_die, dcp_pmgr_dev);
    }

    dcp->asc = asc_init(cfg->dcp);
    if (!dcp->asc) {
        printf("dcp: failed to initialize ASC\n");
        goto out_iovad;
    }

    dcp->rtkit = rtkit_init("dcp", dcp->asc, dcp->dart_dcp, dcp->iovad_dcp, NULL, false);
    if (!dcp->rtkit) {
        printf("dcp: failed to initialize RTKit\n");
        goto out_iovad;
    }

    if (!rtkit_boot(dcp->rtkit)) {
        printf("dcp: failed to boot RTKit\n");
        /* a partial handshake may have handed buffers to the IOP */
        goto out_rtkit;
    }

    dcp->afk = afk_epic_init(dcp->rtkit);
    if (!dcp->afk) {
        printf("dcp: failed to initialize AFK\n");
        goto out_rtkit;
    }

    if (cfg && cfg->dptx_phy[0]) {
        int ret = dcp_hdmi_dptx_init(dcp, cfg);
        if (ret < 0)
            goto out_afk;
    }

    return dcp;

out_afk:
out_rtkit:
    /* the IOP is running: stop it before any of its memory is released */
    dcp_release(dcp, true);
    return NULL;
out_iovad:
    iovad_shutdown(dcp->iovad_dcp, dcp->dart_dcp);
    dart_shutdown(dcp->dart_disp);
out_dart_dcp:
    dart_shutdown(dcp->dart_dcp);
out_free:
    free(dcp);
    return NULL;
}

int dcp_shutdown(dcp_dev_t *dcp, bool sleep)
{
    /* dcp/dcp0 on desktop M2 and M2 Pro/Max devices do not wake from sleep */
    if (dcp->iboot) {
        dcp_ib_shutdown(dcp->iboot);
        dcp->iboot = NULL;
    }
    dcp_system_shutdown(dcp->system_ep);
    dcp_dptx_shutdown(dcp->dptx_ep);
    dcp_dpav_shutdown(dcp->dpav_ep);
    free(dcp->phy);

    if (dcp->compact) {
        /* 26.6: quiesce only, and keep everything if the DCP does not stop */
        dcp_release(dcp, true);
        return dcp_active_after_failure ? -1 : 0;
    }

    afk_epic_shutdown(dcp->afk);
    if (sleep) {
        rtkit_sleep(dcp->rtkit);
        pmgr_reset(dcp_die, dcp_pmgr_dev);
    } else {
        rtkit_quiesce(dcp->rtkit);
    }
    rtkit_free(dcp->rtkit);
    dart_shutdown(dcp->dart_disp);
    iovad_shutdown(dcp->iovad_dcp, dcp->dart_dcp);
    dart_shutdown(dcp->dart_dcp);
    free(dcp);

    return 0;
}
