/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Auxiliary compact EPIC endpoints of the macOS 26.6 DCP firmware (T8132).
 *
 * The firmware builds its HDMI controller graph only when these endpoints are
 * running and their services are open. They are started and answered the way
 * the Gravity Linux apple DRM driver does it (dcp_start() and
 * dcp_compact_aux_ops, Niklas Sheth): every service is opened unless Linux
 * keeps it closed, and every call gets an empty, successful reply.
 */

#include "aux_ep.h"
#include "string.h"
#include "../utils.h"

static void aux_service_init(afk_epic_service_t *service, const char *name, const char *eclass,
                             s64 unit)
{
    UNUSED(eclass);
    UNUSED(unit);

    switch (afk_epic_endpoint(service->epic)) {
        case DCP_DPAVCTRL_ENDPOINT:
            /* The late DP controller interface stays closed until DPAV link
             * control is activated, which Linux does not do by default. */
            if ((service->channel & 7) == 7)
                service->defer_start = true;
            break;
        case DCP_DPAVSERV_ENDPOINT:
            if (name && !strcmp(name, "dcpdp-service-epic"))
                service->defer_start = true;
            break;
        default:
            break;
    }
}

static int aux_service_call(afk_epic_service_t *service, u16 group, u32 idx, const void *data,
                            size_t data_size, void *reply, size_t reply_size)
{
    UNUSED(service);
    UNUSED(group);
    UNUSED(idx);
    UNUSED(data);
    UNUSED(data_size);

    if (reply && reply_size)
        memset(reply, 0, reply_size);
    return 0;
}

static const afk_epic_service_ops_t aux_ops[] = {
    {
        .name = "*",
        .init = aux_service_init,
        .call_group = aux_service_call,
    },
    {},
};

afk_epic_ep_t *dcp_aux_ep_start(afk_epic_t *afk, int endpoint)
{
    afk_epic_ep_t *epic = afk_epic_start_ep_intf(afk, endpoint, aux_ops, true, NULL);
    if (!epic)
        printf("dcp: failed to start endpoint 0x%02x\n", endpoint);
    return epic;
}
