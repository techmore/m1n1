/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef DCP_AFK_H
#define DCP_AFK_H

#include "rtkit.h"
#include "types.h"

enum EPICMessage {
    SUBTYPE_ANNOUNCE = 0x30,
    SUBTYPE_STD_SERVICE = 0xc0,
};

typedef struct afk_epic afk_epic_t;
typedef struct afk_epic_ep afk_epic_ep_t;

typedef struct afk_epic_service_ops afk_epic_service_ops_t;

typedef struct afk_epic_service {
    void *cookie;
    const afk_epic_service_ops_t *ops;
    afk_epic_ep_t *epic;
    void *intf;
    u32 channel;
    u16 seq;
    bool enabled;

    /* compact (AFK v2) services only */
    char name[33];
    s64 unit;
    bool defer_start;
    bool started;
    u8 cmd_tag;
} afk_epic_service_t;

typedef struct afk_epic_service_ops {
    const char name[32];

    void (*init)(afk_epic_service_t *service, const char *name, const char *eclass, s64 unit);
    int (*call)(afk_epic_service_t *service, u32 idx, const void *data, size_t data_size,
                void *reply, size_t reply_size);
    /* Optional: compact endpoints also pass the call group. */
    int (*call_group)(afk_epic_service_t *service, u16 group, u32 idx, const void *data,
                      size_t data_size, void *reply, size_t reply_size);
} afk_epic_service_ops_t;

afk_epic_t *afk_epic_init(rtkit_dev_t *rtkit);
int afk_epic_shutdown(afk_epic_t *afk);

/*
 * Compact EPIC (AFK v2) is the transport of the macOS 26.6 DCP firmware on
 * T8132. Endpoints started after this is set use it.
 */
void afk_epic_set_compact(afk_epic_t *afk, bool compact);
bool afk_epic_is_compact(afk_epic_ep_t *epic);
int afk_epic_endpoint(afk_epic_ep_t *epic);
void afk_epic_set_trace(bool trace);
void afk_epic_set_deadline(u64 deadline);

afk_epic_ep_t *afk_epic_start_ep(afk_epic_t *afk, int endpoint, const afk_epic_service_ops_t *ops,
                                 bool notify);
afk_epic_ep_t *afk_epic_start_ep_intf(afk_epic_t *afk, int endpoint,
                                      const afk_epic_service_ops_t *ops, bool notify, void *intf);
int afk_epic_shutdown_ep(afk_epic_ep_t *epic);

int afk_epic_work(afk_epic_t *afk, int endpoint);
int afk_epic_start_interface(afk_epic_ep_t *epic, void *intf, int expected, size_t insize,
                             size_t outsize);
int afk_epic_command(afk_epic_ep_t *epic, int channel, u16 sub_type, void *txbuf, size_t txsize,
                     void *rxbuf, size_t *rxsize);

/* compact endpoints only */
int afk_epic_open_service(afk_epic_service_t *service);
int afk_epic_wait_services(afk_epic_ep_t *epic, int expected, u32 timeout_us);

#endif
