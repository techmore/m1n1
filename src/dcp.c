/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "../config.h"

#include "adt.h"
#include "afk.h"
#include "dcp.h"
#include "dcp_iboot.h"
#include "firmware.h"
#include "malloc.h"
#include "pmgr.h"
#include "rtkit.h"
#include "smc.h"
#include "string.h"
#include "utils.h"

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

    if (!(dcp_options & DCP_OPT_NO_SMC))
        dcp_t8132_hdmi_power(cfg);

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

    return 0;
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

dcp_dev_t *dcp_init(const display_config_t *cfg)
{
    u32 sid;

    if (cfg && cfg->dptx_phy[0]) {
        if (cfg->compact) {
            /* the compact protocol is known for 26.6 DCP firmware only */
            if (os_firmware.version != V26_6 && os_firmware.version != V26_6_2) {
                printf("dcp: T8132 HDMI needs 26.6 OS firmware (have %s)\n",
                       os_firmware.string);
                return NULL;
            }
            afk_epic_set_trace(dcp_options & DCP_OPT_TRACE);
        } else if (os_firmware.version != V13_5) {
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
        /* A 26.6 DCP left unmapped by iBoot would need a cold restart of its
         * pre-loaded image, which is how stock m1n1 crashed it; do not try. */
        if (cfg && cfg->compact) {
            printf("dcp: firmware was not mapped by iBoot, not resetting a 26.6 DCP\n");
            goto out_iovad;
        }
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
        goto out_iovad;
    }

    dcp->afk = afk_epic_init(dcp->rtkit);
    if (!dcp->afk) {
        printf("dcp: failed to initialize AFK\n");
        goto out_rtkit;
    }

    if (cfg && cfg->compact) {
        dcp->compact = true;
        afk_epic_set_compact(dcp->afk, true);
    }

    if (cfg && cfg->dptx_phy[0]) {
        int ret = cfg->compact ? dcp_t8132_hdmi_init(dcp, cfg) : dcp_hdmi_dptx_init(dcp, cfg);
        if (ret < 0)
            goto out_afk;
    }

    return dcp;

out_afk:
    afk_epic_shutdown(dcp->afk);
out_rtkit:
    rtkit_quiesce(dcp->rtkit);
    rtkit_free(dcp->rtkit);
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
