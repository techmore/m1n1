/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef DCP_AUX_EP_H
#define DCP_AUX_EP_H

#include "../afk.h"

enum dcp_aux_endpoint {
    DCP_SYSTEM_ENDPOINT = 0x20,
    DCP_EXPERT_ENDPOINT = 0x22,
    DCP_DPAVCTRL_ENDPOINT = 0x24,
    DCP_EPIC25_ENDPOINT = 0x25,
    DCP_DPSAC_ENDPOINT = 0x26,
    DCP_DPDEV_ENDPOINT = 0x27,
    DCP_DPAVSERV_ENDPOINT = 0x28,
    DCP_AV_ENDPOINT = 0x29,
    DCP_EPIC2C_ENDPOINT = 0x2c,
    DCP_REMOTE_ALLOC_ENDPOINT = 0x2d,
    DCP_IOMFB_ENDPOINT = 0x37,
};

afk_epic_ep_t *dcp_aux_ep_start(afk_epic_t *afk, int endpoint);

#endif
