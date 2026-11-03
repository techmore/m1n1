/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef DCP_H
#define DCP_H

#include "afk.h"
#include "asc.h"
#include "dart.h"
#include "rtkit.h"

#include "dcp/dpav_ep.h"
#include "dcp/dptx_port_ep.h"
#include "dcp/system_ep.h"

typedef struct {
    const char dcp[24];
    const char dcp_dart[24];
    const char disp_dart[24];
    const char dptx_phy[24];
    const char dp2hdmi_gpio[24];
    const char pmgr_dev[24];
    const char dcp_alias[8];
    u32 dcp_index;
    u8 num_dptxports;
    u8 die;
    /* DCP firmware speaks compact EPIC (macOS 26.6 on T8132) */
    bool compact;
} display_config_t;

/* Options for the compact (T8132) HDMI bring-up, from the display= variable. */
#define DCP_OPT_NO_SMC   BIT(0) /* do not switch the HDMI power GPIOs through SMC */
#define DCP_OPT_NO_IOMFB BIT(1) /* do not start the IOMFB transport endpoint */
#define DCP_OPT_MINIMAL  BIT(2) /* start only DPAV, iBoot and DPTX endpoints */
#define DCP_OPT_TRACE    BIT(3) /* log compact EPIC traffic */
/* stop cleanly after this bring-up stage (1..5, 0 = full); for bisecting */
#define DCP_OPT_STAGE_SHIFT 8
#define DCP_OPT_STAGE(opts) (((opts) >> DCP_OPT_STAGE_SHIFT) & 0xf)

#define DCP_MAX_AUX_EP 12

/* J773g: framebuffer area at the start of /vram; DCP buffers follow it */
#define DCP_T8132_FB_MAX 0x400000

struct dcp_iboot_if;

typedef struct dcp_dev {
    dart_dev_t *dart_dcp;
    dart_dev_t *dart_disp;
    iova_domain_t *iovad_dcp;
    asc_dev_t *asc;
    rtkit_dev_t *rtkit;
    afk_epic_t *afk;
    dcp_system_if_t *system_ep;
    dcp_dpav_if_t *dpav_ep;
    dcp_dptx_if_t *dptx_ep;
    dptx_phy_t *phy;
    u32 die;
    u32 dp2hdmi_pwr_gpio;
    u32 hdmi_pwr_gpio;
    /* compact (T8132) bring-up */
    bool compact;
    struct dcp_iboot_if *iboot;
    afk_epic_ep_t *aux_ep[DCP_MAX_AUX_EP];
} dcp_dev_t;

int dcp_connect_dptx(dcp_dev_t *dcp);
int dcp_work(dcp_dev_t *dcp);

void dcp_set_options(u32 options);
void dcp_set_exc_base(int count);
dcp_dev_t *dcp_init(const display_config_t *config);

int dcp_shutdown(dcp_dev_t *dcp, bool sleep);
bool dcp_left_active(void);
int dcp_t8132_probe(const display_config_t *cfg, bool map);

#endif
