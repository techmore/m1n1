/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "dcp_iboot.h"
#include "afk.h"
#include "assert.h"
#include "firmware.h"
#include "malloc.h"
#include "string.h"
#include "utils.h"

#define DCP_IBOOT_ENDPOINT     0x23
#define DCP_IBOOT_NUM_SERVICES 1

#define TXBUF_LEN 0x4000
#define RXBUF_LEN 0x4000

struct txcmd {
    u32 op;
    u32 len;
    u32 unk1;
    u32 unk2;
    u8 payload[];
};

struct rxcmd {
    u32 op;
    u32 len;
    u8 payload[];
};

struct dcp_iboot_if {
    dcp_dev_t *dcp;
    afk_epic_ep_t *epic;
    int channel;
    bool enabled;
    u32 timing_cnt, color_cnt;

    union {
        u8 txbuf[TXBUF_LEN];
        struct txcmd txcmd;
    };

    union {
        u8 rxbuf[RXBUF_LEN];
        struct rxcmd rxcmd;
    };
};

enum IBootCmd {
    IBOOT_SET_SURFACE = 1,
    IBOOT_SET_POWER = 2,
    IBOOT_GET_HPD = 3,
    IBOOT_GET_TIMING_MODES = 4,
    IBOOT_GET_COLOR_MODES = 5,
    IBOOT_SET_MODE = 6,
    IBOOT_SWAP_BEGIN = 15,
    IBOOT_SWAP_SET_LAYER = 16,
    IBOOT_SWAP_END = 18,
};

struct get_hpd_resp {
    u8 hpd;
    u8 pad[3];
    u32 timing_cnt;
    u32 color_cnt;
};

struct get_tmode_resp {
    u32 count;
    dcp_timing_mode_t modes[];
};

struct get_cmode_resp {
    u32 count;
    dcp_color_mode_t modes[];
};

struct swap_start_resp {
    u32 unk1, unk2, unk3;
    u32 swap_id;
    u32 unk4;
};

struct swap_set_layer_cmd {
    u32 unk;
    u32 layer_id;
    dcp_layer_t layer;
    dcp_rect_t src;
    dcp_rect_t dst;
    u32 unk2;
} PACKED;

struct swap_set_layer_cmd_v13_3 {
    u32 unk;
    u32 layer_id;
    dcp_layer_t layer;
    u32 unk3; // possibly part of layer
    u32 unk4; // possibly part of layer
    dcp_rect_t src;
    dcp_rect_t dst;
    u32 unk2;
} PACKED;

void dcp_ib_service_init(afk_epic_service_t *service, const char *name, const char *eclass,
                         s64 unit)
{
    dcp_iboot_if_t *iboot = service->intf;
    if (!iboot)
        return;
    if (strncmp("disp0-service", eclass, 32) == 0) {
        if (iboot->enabled) {
            printf("dcp-iboot: service init for enabled 'disp0-service' on channel: %d\n",
                   iboot->channel);
            return;
        }
        iboot->enabled = true;
        iboot->channel = service->channel;
    }
    UNUSED(name);
    UNUSED(unit);
}

static const afk_epic_service_ops_t iboot_service_ops[] = {
    {
        .name = "disp0-service",
        .init = dcp_ib_service_init,
    },
    {},
};

dcp_iboot_if_t *dcp_ib_init(dcp_dev_t *dcp)
{
    dcp_iboot_if_t *iboot = calloc(1, sizeof(dcp_iboot_if_t));
    if (!iboot)
        return NULL;

    iboot->dcp = dcp;
    iboot->epic =
        afk_epic_start_ep_intf(dcp->afk, DCP_IBOOT_ENDPOINT, iboot_service_ops, false, iboot);
    if (!iboot->epic) {
        printf("dcp-iboot: failed to initialize EPIC\n");
        goto err_free;
    }

    int err =
        afk_epic_start_interface(iboot->epic, iboot, DCP_IBOOT_NUM_SERVICES, TXBUF_LEN, RXBUF_LEN);

    if (err < 0 || !iboot->enabled) {
        printf("dcp-iboot: failed to initialize disp0 service\n");
        goto err_shutdown;
    }

    return iboot;

err_shutdown:
    afk_epic_shutdown_ep(iboot->epic);
err_free:
    free(iboot);
    return NULL;
}

int dcp_ib_shutdown(dcp_iboot_if_t *iboot)
{
    afk_epic_shutdown_ep(iboot->epic);

    free(iboot);
    return 0;
}

/*
 * Reply capacity for compact (26.6) firmware. Commands are inline there and
 * padded to the reply capacity, so ask for what the reply needs. Setters
 * return transport status only and get no reply buffer, as the Gravity Linux
 * driver sends SET_SURFACE.
 */
static size_t dcp_ib_compact_reply_cap(dcp_iboot_if_t *iboot, int op)
{
    u32 cnt;

    switch (op) {
        case IBOOT_SET_SURFACE:
        case IBOOT_SET_POWER:
        case IBOOT_SET_MODE:
            return 0;
        case IBOOT_GET_HPD:
            return sizeof(struct rxcmd) + sizeof(struct get_hpd_resp);
        case IBOOT_GET_TIMING_MODES:
            cnt = (iboot->timing_cnt && iboot->timing_cnt <= 128) ? iboot->timing_cnt : 64;
            return sizeof(struct rxcmd) + sizeof(struct get_tmode_resp) +
                   cnt * sizeof(dcp_timing_mode_t);
        case IBOOT_GET_COLOR_MODES:
            cnt = (iboot->color_cnt && iboot->color_cnt <= 128) ? iboot->color_cnt : 64;
            return sizeof(struct rxcmd) + sizeof(struct get_cmode_resp) +
                   cnt * sizeof(dcp_color_mode_t);
        default:
            return 0x400;
    }
}

static int dcp_ib_cmd(dcp_iboot_if_t *iboot, int op, size_t in_size)
{
    size_t rxsize = RXBUF_LEN;
    assert(in_size <= TXBUF_LEN - sizeof(struct txcmd));

    iboot->txcmd.op = op;
    iboot->txcmd.len = sizeof(struct txcmd) + in_size;

    if (!afk_epic_is_compact(iboot->epic))
        return afk_epic_command(iboot->epic, iboot->channel, 0xc0, iboot->txbuf,
                                sizeof(struct txcmd) + in_size, iboot->rxbuf, &rxsize);

    rxsize = dcp_ib_compact_reply_cap(iboot, op);
    memset(iboot->rxbuf, 0, rxsize);
    int ret = afk_epic_command(iboot->epic, iboot->channel, 0xc0, iboot->txbuf,
                               sizeof(struct txcmd) + in_size, rxsize ? iboot->rxbuf : NULL,
                               rxsize ? &rxsize : NULL);
    if (ret) {
        printf("dcp-iboot: operation %d failed: %d\n", op, ret);
        return ret < 0 ? ret : -1;
    }

    if (!dcp_ib_compact_reply_cap(iboot, op))
        return 0;

    if (rxsize < sizeof(struct rxcmd) || iboot->rxcmd.len < sizeof(struct rxcmd) ||
        iboot->rxcmd.len > rxsize) {
        printf("dcp-iboot: operation %d: short reply (%zu bytes, len %u)\n", op, rxsize,
               iboot->rxcmd.len);
        return -1;
    }

    return 0;
}

int dcp_ib_set_surface(dcp_iboot_if_t *iboot, dcp_layer_t *layer)
{
    dcp_layer_t *cmd = (void *)iboot->txcmd.payload;
    *cmd = *layer;

    /* 26.6 takes the layer followed by two more (zero) words, as Linux sends. */
    if (afk_epic_is_compact(iboot->epic)) {
        memset(iboot->txcmd.payload + sizeof(*layer), 0, 8);
        return dcp_ib_cmd(iboot, IBOOT_SET_SURFACE, sizeof(*layer) + 8);
    }

    return dcp_ib_cmd(iboot, IBOOT_SET_SURFACE, sizeof(*layer));
}

int dcp_ib_set_power(dcp_iboot_if_t *iboot, bool power)
{
    u32 *pwr = (void *)iboot->txcmd.payload;
    *pwr = power;

    return dcp_ib_cmd(iboot, IBOOT_SET_POWER, 1);
}

int dcp_ib_get_hpd(dcp_iboot_if_t *iboot, int *timing_cnt, int *color_cnt)
{
    struct get_hpd_resp *resp = (void *)iboot->rxcmd.payload;
    int ret = dcp_ib_cmd(iboot, IBOOT_GET_HPD, 0);

    if (ret < 0)
        return ret;

    iboot->timing_cnt = resp->timing_cnt;
    iboot->color_cnt = resp->color_cnt;
    if (timing_cnt)
        *timing_cnt = resp->timing_cnt;
    if (color_cnt)
        *color_cnt = resp->color_cnt;

    return !!resp->hpd;
}

int dcp_ib_get_timing_modes(dcp_iboot_if_t *iboot, dcp_timing_mode_t **modes)
{
    struct get_tmode_resp *resp = (void *)iboot->rxcmd.payload;
    int ret = dcp_ib_cmd(iboot, IBOOT_GET_TIMING_MODES, 0);

    if (ret < 0)
        return ret;

    if (afk_epic_is_compact(iboot->epic) &&
        (iboot->rxcmd.len < sizeof(struct rxcmd) + sizeof(*resp) ||
         resp->count > (iboot->rxcmd.len - sizeof(struct rxcmd) - sizeof(*resp)) /
                           sizeof(resp->modes[0]))) {
        printf("dcp-iboot: mode list does not fit its reply\n");
        return -1;
    }

    *modes = resp->modes;
    return resp->count;
}

int dcp_ib_get_color_modes(dcp_iboot_if_t *iboot, dcp_color_mode_t **modes)
{
    struct get_cmode_resp *resp = (void *)iboot->rxcmd.payload;
    int ret = dcp_ib_cmd(iboot, IBOOT_GET_COLOR_MODES, 0);

    if (ret < 0)
        return ret;

    if (afk_epic_is_compact(iboot->epic) &&
        (iboot->rxcmd.len < sizeof(struct rxcmd) + sizeof(*resp) ||
         resp->count > (iboot->rxcmd.len - sizeof(struct rxcmd) - sizeof(*resp)) /
                           sizeof(resp->modes[0]))) {
        printf("dcp-iboot: mode list does not fit its reply\n");
        return -1;
    }

    *modes = resp->modes;
    return resp->count;
}

int dcp_ib_set_mode(dcp_iboot_if_t *iboot, dcp_timing_mode_t *tmode, dcp_color_mode_t *cmode)
{
    struct {
        dcp_timing_mode_t tmode;
        dcp_color_mode_t cmode;
    } *cmd = (void *)iboot->txcmd.payload;

    cmd->tmode = *tmode;
    cmd->cmode = *cmode;
    return dcp_ib_cmd(iboot, IBOOT_SET_MODE, sizeof(*cmd));
}

int dcp_ib_swap_begin(dcp_iboot_if_t *iboot)
{
    struct swap_start_resp *resp = (void *)iboot->rxcmd.payload;
    int ret = dcp_ib_cmd(iboot, IBOOT_SWAP_BEGIN, 0);
    if (ret < 0)
        return ret;

    return resp->swap_id;
}

static int swap_set_layer_v12_3(dcp_iboot_if_t *iboot, int layer_id, dcp_layer_t *layer,
                                dcp_rect_t *src_rect, dcp_rect_t *dst_rect)
{
    struct swap_set_layer_cmd *cmd = (void *)iboot->txcmd.payload;
    memset(cmd, 0, sizeof(*cmd));
    cmd->layer_id = layer_id;
    cmd->layer = *layer;
    cmd->src = *src_rect;
    cmd->dst = *dst_rect;

    return dcp_ib_cmd(iboot, IBOOT_SWAP_SET_LAYER, sizeof(*cmd));
}

static int swap_set_layer_v13_3(dcp_iboot_if_t *iboot, int layer_id, dcp_layer_t *layer,
                                dcp_rect_t *src_rect, dcp_rect_t *dst_rect)
{
    struct swap_set_layer_cmd_v13_3 *cmd = (void *)iboot->txcmd.payload;
    memset(cmd, 0, sizeof(*cmd));
    cmd->layer_id = layer_id;
    cmd->layer = *layer;
    cmd->src = *src_rect;
    cmd->dst = *dst_rect;

    return dcp_ib_cmd(iboot, IBOOT_SWAP_SET_LAYER, sizeof(*cmd));
}

int dcp_ib_swap_set_layer(dcp_iboot_if_t *iboot, int layer_id, dcp_layer_t *layer,
                          dcp_rect_t *src_rect, dcp_rect_t *dst_rect)
{
    if (os_firmware.version < V13_3)
        return swap_set_layer_v12_3(iboot, layer_id, layer, src_rect, dst_rect);
    else
        return swap_set_layer_v13_3(iboot, layer_id, layer, src_rect, dst_rect);
}

int dcp_ib_swap_end(dcp_iboot_if_t *iboot)
{
    memset(iboot->txcmd.payload, 0, 12);
    return dcp_ib_cmd(iboot, IBOOT_SWAP_END, 12);
}
