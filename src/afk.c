/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * The compact EPIC (AFK v2) transport used by the macOS 26.6 DCP firmware on
 * T8132 is ported from the Gravity Linux apple DRM driver
 * (drivers/gpu/drm/apple/afk.c and afk-fragment.h, "drm/apple: bring up T8132
 * DCP hotplug" and "drm/apple: support fragmented compact AFK messages" by
 * Niklas Sheth, GPL-2.0-only OR MIT). m1n1 drives it synchronously and every
 * wait is bounded.
 */

#include "afk.h"
#include "assert.h"
#include "exception.h"
#include "malloc.h"
#include "string.h"
#include "utils.h"

#include "dcp/parser.h"

/* Ring buffers are three header blocks (size, rptr, wptr) followed by data.
 * Classic firmware uses 0x40 byte blocks; the block size is derived from the
 * ring geometry so larger blocks work as well. */
struct afk_rb {
    bool ready;
    volatile u32 *rptr;
    volatile u32 *wptr;
    void *buf;
    size_t bufsz;
    size_t block_size;
};

enum EPICType {
    TYPE_NOTIFY = 0,
    TYPE_COMMAND = 3,
    TYPE_REPLY = 4,
    TYPE_NOTIFY_ACK = 8,
};

enum EPICCategory {
    CAT_REPORT = 0x00,
    CAT_NOTIFY = 0x10,
    CAT_REPLY = 0x20,
    CAT_COMMAND = 0x30,
};

struct afk_qe {
    u32 magic;
    u32 size;
    u32 channel;
    u32 type;
    u8 data[];
};

struct epic_hdr {
    u8 version;
    u16 seq;
    u8 _pad;
    u32 unk;
    u64 timestamp;
} PACKED;

struct epic_sub_hdr {
    u32 length;
    u8 version;
    u8 category;
    u16 type;
    u64 timestamp;
    u16 seq;
    u8 unk;
    u8 flags;
    u32 inline_len;
} PACKED;

struct epic_announce {
    char name[32];
    u8 props[];
} PACKED;

struct epic_cmd {
    u32 retcode;
    u64 rxbuf;
    u64 txbuf;
    u32 rxlen;
    u32 txlen;
    u8 rxcookie;
    u8 txcookie;
} PACKED;

/*
 * Compact messages: an 8 byte header repeated on every fragment, then the
 * logical message: an 8 byte timestamp, the sub header and the payload.
 */
struct epic_compact_hdr {
    u8 seq;
    u8 flags;
    u16 intf_id;
    u32 length;
} PACKED;

struct epic_compact_sub_hdr {
    u8 type;
    u8 category;
    u8 unk;
    u8 flags;
    u32 reserved;
} PACKED;

struct epic_compact_desc {
    u32 status;
    u32 length;
} PACKED;

struct afk_service_call_hdr {
    u8 _pad0[2];
    u16 group;
    u32 command;
    u32 data_len;
    u32 magic;
    u8 _pad1[48];
} PACKED;

#define EPIC_SERVICE_CALL_MAGIC 0x69706378

#define COMPACT_TIMESTAMP_SIZE 8
#define COMPACT_BODY_HDR_SIZE  (COMPACT_TIMESTAMP_SIZE + sizeof(struct epic_compact_sub_hdr))
/* Native 26.6 traffic: at most 1028 bytes of the logical message per fragment. */
#define COMPACT_FRAGMENT_SIZE 1028
#define COMPACT_MESSAGE_MAX   (256 * 1024)

enum compact_category {
    CCAT_REPORT = 0,
    CCAT_COMMAND = 1,
    CCAT_REPLY = 2,
};

#define CTYPE_ANNOUNCE 0x11
#define CTYPE_OPEN     0x12

#define AFK_START_TIMEOUT_US    1000000
#define AFK_SHUTDOWN_TIMEOUT_US 1000000
#define AFK_COMMAND_TIMEOUT_US  5000000
#define AFK_TX_SPACE_TIMEOUT_US 1000000

struct afk_epic {
    rtkit_dev_t *rtk;

    afk_epic_ep_t *endpoint[0x10];

    bool compact;
};

#define AFK_MAX_CHANNEL 8

struct afk_compact_pending {
    bool active;
    bool done;
    u16 channel;
    u8 type;
    u32 status;
    bool service_call;
    u16 group;
    u32 command;
    void *out;
    size_t out_len;
    size_t got;
    u32 value;
};

struct afk_epic_ep {
    int ep;
    afk_epic_t *afk;

    struct rtkit_buffer buf;
    u16 tag;

    struct afk_rb tx;
    struct afk_rb rx;

    struct rtkit_buffer txbuf;
    struct rtkit_buffer rxbuf;

    bool started;
    u16 seq;

    u32 num_channels;

    const afk_epic_service_ops_t *ops;
    afk_epic_service_t services[AFK_MAX_CHANNEL];

    void (*recv_handler)(afk_epic_ep_t *epic);

    /* compact endpoints */
    bool compact;
    bool broken;
    void *intf;
    u8 tx_seq;
    u8 cmd_tag;
    u8 *crx_buf;
    u32 crx_total;
    u32 crx_received;
    u16 crx_intf;
    u8 crx_next_seq;
    struct afk_compact_pending pending;
};

enum RBEP_MSG {
    RBEP_INIT = 0x80,
    RBEP_INIT_ACK = 0xa0,
    RBEP_GETBUF = 0x89,
    RBEP_GETBUF_ACK = 0xa1,
    RBEP_INIT_TX = 0x8a,
    RBEP_INIT_RX = 0x8b,
    RBEP_START = 0xa3,
    RBEP_START_ACK = 0x86,
    RBEP_SEND = 0xa2,
    RBEP_RECV = 0x85,
    RBEP_SPACE_AVAILABLE = 0xa7,
    RBEP_WAIT_FOR_SPACE = 0xa8,
    RBEP_SHUTDOWN = 0xc0,
    RBEP_SHUTDOWN_ACK = 0xc1,
};

#define BLOCK_SHIFT 6
#define QE_MAGIC    ' POI'

#define RBEP_TYPE GENMASK(63, 48)

#define GETBUF_SIZE    GENMASK(31, 16)
#define GETBUF_TAG     GENMASK(15, 0)
#define GETBUF_ACK_DVA GENMASK(47, 0)

#define INITRB_OFFSET GENMASK(47, 32)
#define INITRB_SIZE   GENMASK(31, 16)
#define INITRB_TAG    GENMASK(15, 0)

#define SEND_WPTR GENMASK(31, 0)

#define EPIC_DATA_READY   13
#define EPIC_POLL_TIMEOUT (-110)

static bool afk_trace;
/* Optional overall deadline for compact work (the J773g display budget). */
static u64 afk_deadline;

void afk_epic_set_deadline(u64 deadline)
{
    afk_deadline = deadline;
}

static bool afk_out_of_time(void)
{
    if (afk_deadline && timeout_expired(afk_deadline)) {
        printf("AFK: display time budget used up\n");
        return true;
    }
    return false;
}

#define tprintf(...)                                                                               \
    do {                                                                                           \
        if (afk_trace)                                                                             \
            printf(__VA_ARGS__);                                                                   \
    } while (0)

void afk_epic_set_trace(bool trace)
{
    afk_trace = trace;
}

static void afk_compact_recv_handler(afk_epic_ep_t *epic);

bool afk_rb_init(afk_epic_ep_t *epic, struct afk_rb *rb, u64 base, u64 size)
{
    if (!epic->buf.bfr || base >= epic->buf.sz || size > epic->buf.sz - base) {
        printf("AFK: ring buffer outside of the shared buffer\n");
        return false;
    }

    void *hdr = epic->buf.bfr + base;
    dma_rmb();
    u32 bufsz = *(volatile u32 *)hdr;

    if (!bufsz || bufsz >= size || (size - bufsz) % 3) {
        printf("AFK: ring buffer size mismatch (data 0x%x, total 0x%lx)\n", bufsz, size);
        return false;
    }

    size_t block = (size - bufsz) / 3;
    if (block < (1 << BLOCK_SHIFT) || block % (1 << BLOCK_SHIFT) || bufsz % block) {
        printf("AFK: invalid ring buffer block size 0x%lx\n", block);
        return false;
    }

    rb->rptr = hdr + block;
    rb->wptr = hdr + 2 * block;
    rb->buf = hdr + 3 * block;
    rb->bufsz = bufsz;
    rb->block_size = block;
    rb->ready = true;

    if (block != (1 << BLOCK_SHIFT))
        tprintf("AFK: ring buffer with 0x%lx byte blocks\n", block);

    return true;
}

static bool afk_rb_has_data(struct afk_rb *rb)
{
    if (!rb->ready)
        return false;
    dma_rmb();
    return *rb->rptr != *rb->wptr;
}

static void afk_drain_rx(afk_epic_ep_t *epic)
{
    /* Bounded: never more entries than fit into the ring. */
    size_t limit = epic->rx.ready ? epic->rx.bufsz / epic->rx.block_size : 0;

    while (limit-- && !epic->broken && epic->recv_handler && afk_rb_has_data(&epic->rx))
        epic->recv_handler(epic);
}

/*
 * deadline == 0: do not block. Returns EPIC_DATA_READY if @endpoint has
 * data, EPIC_POLL_TIMEOUT if the deadline expired, < 0 on errors.
 */
static int afk_epic_poll(afk_epic_t *afk, int endpoint, u64 deadline)
{
    int ret;
    struct rtkit_message msg;

    while ((ret = rtkit_recv(afk->rtk, &msg)) == 0) {
        if (!deadline)
            break;
        if (timeout_expired(deadline))
            return EPIC_POLL_TIMEOUT;
    }

    if (ret < 0) {
        printf("EPIC: rtkit_recv failed!\n");
        return ret;
    }

    if (ret == 0) {
        return 0;
    }

    if (msg.ep < 0x20 || msg.ep >= 0x30 || !afk->endpoint[msg.ep - 0x20]) {
        if (afk->compact)
            tprintf("EPIC: ignoring message for endpoint 0x%02x: %lx\n", msg.ep, msg.msg);
        else
            printf("EPIC: received message for unexpected endpoint 0x%02x\n", msg.ep);
        return 0;
    }

    afk_epic_ep_t *epic = afk->endpoint[msg.ep - 0x20];
    if (!epic) {
        printf("EPIC: received message for idle endpoint 0x%02x\n", msg.ep);
        return 0;
    }

    int type = FIELD_GET(RBEP_TYPE, msg.msg);
    u64 base, size, tag;
    switch (type) {
        case RBEP_INIT_ACK:
            break;

        case RBEP_GETBUF:
            if (epic->buf.bfr) {
                printf("EPIC[0x%02x]: GETBUF but buffer already exists\n", epic->ep);
                return 0;
            }
            size = FIELD_GET(GETBUF_SIZE, msg.msg) << BLOCK_SHIFT;
            epic->tag = FIELD_GET(GETBUF_TAG, msg.msg);
            if (!rtkit_alloc_buffer(epic->afk->rtk, &epic->buf, size)) {
                printf("EPIC: failed to allocate buffer\n");
                return -1;
            }
            msg.msg = (FIELD_PREP(RBEP_TYPE, RBEP_GETBUF_ACK) |
                       FIELD_PREP(GETBUF_ACK_DVA, epic->buf.dva));
            if (!rtkit_send(epic->afk->rtk, &msg)) {
                printf("EPIC: failed to send buffer address\n");
                return -1;
            }
            break;

        case RBEP_INIT_TX:
        case RBEP_INIT_RX:
            base = FIELD_GET(INITRB_OFFSET, msg.msg) << BLOCK_SHIFT;
            size = FIELD_GET(INITRB_SIZE, msg.msg) << BLOCK_SHIFT;
            tag = FIELD_GET(INITRB_TAG, msg.msg);
            if (tag != epic->tag) {
                printf("EPIC: wrong tag (0x%x != 0x%lx)\n", epic->tag, tag);
                return -1;
            }

            struct afk_rb *rb;
            if (type == RBEP_INIT_RX)
                rb = &epic->rx;
            else
                rb = &epic->tx;

            if (rb->ready) {
                printf("EPIC[0x%02x]: ring buffer is already initialized\n", epic->ep);
                return 0;
            }

            if (!afk_rb_init(epic, rb, base, size))
                return -1;

            if (epic->rx.ready && epic->tx.ready) {
                msg.msg = FIELD_PREP(RBEP_TYPE, RBEP_START);
                if (!rtkit_send(epic->afk->rtk, &msg)) {
                    printf("EPIC: failed to send start\n");
                    return -1;
                }
            }
            break;

        case RBEP_RECV: {
            if (afk_rb_has_data(&epic->rx)) {
                if (endpoint == epic->ep)
                    return EPIC_DATA_READY;
                else if (epic->recv_handler && !epic->broken)
                    epic->recv_handler(epic);
            }
            break;
        }
        case RBEP_START_ACK:
            epic->started = true;
            break;

        case RBEP_SHUTDOWN_ACK:
            epic->started = false;
            break;

        case RBEP_WAIT_FOR_SPACE:
            /* Our receive ring is full: drain it, then tell the IOP. */
            afk_drain_rx(epic);
            dma_wmb();
            msg.msg = FIELD_PREP(RBEP_TYPE, RBEP_SPACE_AVAILABLE);
            if (!rtkit_send(epic->afk->rtk, &msg)) {
                printf("EPIC: failed to send space available\n");
                return -1;
            }
            break;

        case RBEP_SPACE_AVAILABLE:
            /* The sender polls the shared read pointer. */
            break;

        default:
            printf("EPIC: received unknown message type 0x%x\n", type);
            return 0;
            break;
    }

    return 0;
}

static int afk_epic_rx(afk_epic_ep_t *epic, struct afk_qe **qe)
{
    struct afk_rb *rb = &epic->rx;
    dma_rmb();
    u32 rptr = *rb->rptr;

    if (!rb->ready || rptr > rb->bufsz - sizeof(struct afk_qe)) {
        printf("EPIC: invalid read pointer 0x%x\n", rptr);
        return -1;
    }

    struct afk_qe *hdr = rb->buf + rptr;

    if (hdr->magic != QE_MAGIC) {
        printf("EPIC: bad queue entry magic!\n");
        return -1;
    }

    /* If the payload did not fit, the producer left this entry as a marker
     * and wrote the real one at the start of the ring. */
    if (rptr + sizeof(*hdr) + hdr->size > rb->bufsz) {
        rptr = 0;
        hdr = rb->buf + rptr;
        if (hdr->magic != QE_MAGIC) {
            printf("EPIC: bad queue entry magic!\n");
            return -1;
        }
        *rb->rptr = rptr;
    }

    if (hdr->size > rb->bufsz - rptr - sizeof(*hdr)) {
        printf("EPIC: queue entry too large (0x%x)\n", hdr->size);
        return -1;
    }

    *qe = hdr;

    return 1;
}

static int afk_epic_tx(afk_epic_ep_t *epic, u32 channel, u32 type, void *data, size_t size)
{
    struct afk_rb *rb = &epic->tx;

    u32 rptr = *rb->rptr;
    u32 wptr = *rb->wptr;
    struct afk_qe *hdr = rb->buf + wptr;
    size_t buf_advance = ALIGN_UP(sizeof(struct afk_qe) + size, rb->block_size);

    if (wptr < rptr && buf_advance >= rptr - wptr)
        goto buffer_full;
    if (wptr >= rptr) {
        bool fits_above_wptr =
            (buf_advance < rb->bufsz - wptr) || (buf_advance == rb->bufsz - wptr && rptr != 0);

        if (!fits_above_wptr && buf_advance >= rptr)
            goto buffer_full;
    }

    hdr->magic = QE_MAGIC;
    hdr->channel = channel;
    hdr->type = type;
    hdr->size = size;

    wptr += sizeof(struct afk_qe);

    if (size > rb->bufsz - wptr) {
        *(struct afk_qe *)rb->buf = *hdr;
        hdr = rb->buf;
        wptr = sizeof(struct afk_qe);
    }

    wptr += size;
    wptr = ALIGN_UP(wptr, rb->block_size);
    if (wptr >= rb->bufsz)
        wptr = 0;

    memcpy(hdr + 1, data, size);

    dma_mb();
    *rb->wptr = wptr;
    dma_wmb();

    struct rtkit_message msg = {
        epic->ep,
        FIELD_PREP(RBEP_TYPE, RBEP_SEND) | FIELD_PREP(SEND_WPTR, wptr),
    };

    if (!rtkit_send(epic->afk->rtk, &msg)) {
        printf("EPIC: failed to send TX WPTR message\n");
        return -1;
    }

    return 1;

buffer_full:
    printf("EPIC: TX ring buffer is full\n");
    return -1;
}

static void afk_epic_rx_ack(afk_epic_ep_t *epic)
{
    struct afk_rb *rb = &epic->rx;
    u32 rptr = *rb->rptr;
    struct afk_qe *hdr = rb->buf + rptr;

    if (hdr->magic != QE_MAGIC) {
        printf("EPIC: bad queue entry magic!\n");
    }

    dma_mb();

    rptr = ALIGN_UP(rptr + sizeof(*hdr) + hdr->size, rb->block_size);
    if (rptr > rb->bufsz) {
        printf("EPIC[0x%02x]: queue entry runs past the ring, ignoring endpoint\n", epic->ep);
        epic->broken = true;
        return;
    }
    if (rptr == rb->bufsz)
        rptr = 0;
    *rb->rptr = rptr;
}

int afk_epic_work(afk_epic_t *afk, int endpoint)
{
    int i = 0;
    /* A chatty IOP must not keep us here past the callers' deadlines. */
    int budget = 64;

    while (i < 0x10) {
        afk_epic_ep_t *cur = afk->endpoint[i++];
        if (cur && !cur->broken && afk_rb_has_data(&cur->rx)) {
            if (cur->ep == endpoint) {
                return EPIC_DATA_READY;
            }
            if (cur->recv_handler)
                cur->recv_handler(cur);
            else {
                struct afk_qe *rmsg;
                // will net block
                int ret = afk_epic_rx(cur, &rmsg);
                if (ret < 0) {
                    return ret;
                }
                dprintf("EPIC[0x%02x]: ignoring message type %d\n", cur->ep, rmsg->type);
                afk_epic_rx_ack(cur);
            }
        }
        if (rtkit_can_recv(afk->rtk)) {
            int ret = afk_epic_poll(afk, endpoint, 0);
            if (ret < 0 || ret == EPIC_DATA_READY)
                return ret;
            if (--budget <= 0)
                return 0;
            i = 0;
            continue;
        }
    }

    return 0;
}

static afk_epic_service_t *afk_epic_find_service(afk_epic_ep_t *epic, u32 channel)
{
    for (u32 i = 0; i < epic->num_channels; i++)
        if (epic->services[i].enabled && epic->services[i].channel == channel)
            return &epic->services[i];

    return NULL;
}

/* ---------------------------------------------------------------------------
 * Compact EPIC (AFK v2)
 */

void afk_epic_set_compact(afk_epic_t *afk, bool compact)
{
    afk->compact = compact;
}

bool afk_epic_is_compact(afk_epic_ep_t *epic)
{
    return epic && epic->compact;
}

int afk_epic_endpoint(afk_epic_ep_t *epic)
{
    return epic->ep;
}

static int afk_compact_tx_fragment(afk_epic_ep_t *epic, u16 intf_id, const void *body,
                                   size_t size, u32 total)
{
    struct afk_rb *rb = &epic->tx;
    size_t message_size = sizeof(struct epic_compact_hdr) + size;
    u32 rptr, wptr, old_wptr;
    u64 deadline = 0;

    if (!rb->ready || epic->broken)
        return -1;

    size_t advance = ALIGN_UP(sizeof(struct afk_qe) + message_size, rb->block_size);
    if (advance > rb->bufsz)
        return -1;

    while (true) {
        bool full = false;

        dma_rmb();
        rptr = *rb->rptr;
        wptr = *rb->wptr;
        if (rptr >= rb->bufsz || wptr >= rb->bufsz) {
            printf("AFK[ep:%02x]: invalid TX ring pointers\n", epic->ep);
            return -1;
        }

        if (wptr < rptr && advance >= rptr - wptr)
            full = true;
        if (wptr >= rptr) {
            bool fits_above =
                advance < rb->bufsz - wptr || (advance == rb->bufsz - wptr && rptr != 0);

            if (!fits_above && advance >= rptr)
                full = true;
        }
        if (!full)
            break;

        if (!deadline) {
            struct rtkit_message msg = {
                epic->ep,
                FIELD_PREP(RBEP_TYPE, RBEP_WAIT_FOR_SPACE) | message_size,
            };
            rtkit_send(epic->afk->rtk, &msg);
            deadline = timeout_calculate(AFK_TX_SPACE_TIMEOUT_US);
        }
        /* The IOP advances the shared read pointer on its own. */
        if (timeout_expired(deadline)) {
            printf("AFK[ep:%02x]: TX ring buffer is full\n", epic->ep);
            return -1;
        }
    }

    old_wptr = wptr;

    struct afk_qe *qe = rb->buf + wptr;
    qe->magic = QE_MAGIC;
    qe->size = message_size;
    qe->channel = 0;
    qe->type = 0;
    wptr += sizeof(*qe);

    if (message_size > rb->bufsz - wptr) {
        *(struct afk_qe *)rb->buf = *qe;
        qe = rb->buf;
        wptr = sizeof(*qe);
    }

    struct epic_compact_hdr *hdr = (void *)qe->data;
    hdr->seq = epic->tx_seq++;
    hdr->flags = 0;
    hdr->intf_id = intf_id;
    hdr->length = total;
    memcpy(qe->data + sizeof(*hdr), body, size);

    wptr = ALIGN_UP(wptr + message_size, rb->block_size);
    if (wptr >= rb->bufsz)
        wptr = 0;

    dma_mb();
    *rb->wptr = wptr;
    dma_wmb();

    struct rtkit_message msg = {
        epic->ep,
        FIELD_PREP(RBEP_TYPE, RBEP_RECV) | ((u64)(rptr & 0xffff) << 32) |
            ((u64)(wptr & 0xffff) << 16) | (old_wptr & 0xffff),
    };

    if (!rtkit_send(epic->afk->rtk, &msg)) {
        printf("AFK[ep:%02x]: failed to send TX doorbell\n", epic->ep);
        return -1;
    }

    return 0;
}

/* Send one logical message; fragments are never interleaved. */
static int afk_compact_send(afk_epic_ep_t *epic, u16 intf_id, u8 type, u8 category,
                            const void *payload, size_t payload_size, size_t wire_size)
{
    if (payload_size > wire_size || wire_size > COMPACT_MESSAGE_MAX - COMPACT_BODY_HDR_SIZE)
        return -1;

    size_t total = COMPACT_BODY_HDR_SIZE + wire_size;
    u8 *body = calloc(total, 1);
    if (!body)
        return -1;

    struct epic_compact_sub_hdr *sub = (void *)(body + COMPACT_TIMESTAMP_SIZE);
    sub->type = type;
    sub->category = category;
    sub->unk = type == CTYPE_OPEN;
    if (payload_size)
        memcpy(body + COMPACT_BODY_HDR_SIZE, payload, payload_size);

    int ret = 0;
    for (size_t offset = 0; offset < total;) {
        size_t chunk = min(total - offset, (size_t)COMPACT_FRAGMENT_SIZE);
        ret = afk_compact_tx_fragment(epic, intf_id, body + offset, chunk, total);
        if (ret) {
            /* A partial message cannot be resumed; stop using the endpoint. */
            if (offset)
                epic->broken = true;
            break;
        }
        offset += chunk;
    }

    free(body);
    return ret;
}

int afk_epic_open_service(afk_epic_service_t *service)
{
    if (!service || !service->enabled)
        return -1;
    if (!service->epic->compact || service->started)
        return 0;

    int ret = afk_compact_send(service->epic, service->channel, CTYPE_OPEN, CCAT_REPORT, NULL, 0, 0);
    if (!ret)
        service->started = true;
    else
        printf("AFK[ep:%02x]: failed to open service %s\n", service->epic->ep, service->name);

    return ret;
}

static const afk_epic_service_ops_t *afk_compact_match_service(afk_epic_ep_t *epic,
                                                               const char *wire_name)
{
    const afk_epic_service_ops_t *ops;
    const char *service_name;

    if (!wire_name[0] || !epic->ops)
        return NULL;

    service_name = strchr(wire_name, ':');
    if (service_name)
        service_name++;
    else
        service_name = wire_name;

    for (ops = epic->ops; ops->name[0]; ops++) {
        size_t len = strlen(ops->name);

        if (!strcmp(ops->name, "*"))
            return ops;
        if (strncmp(ops->name, service_name, len))
            continue;
        if (service_name[len] && service_name[len] != ':')
            continue;

        return ops;
    }

    return NULL;
}

static bool afk_parse_unit(const char *s, s64 *unit)
{
    s64 val = 0;

    if (!*s)
        return false;
    for (; *s; s++) {
        if (*s < '0' || *s > '9')
            return false;
        val = val * 10 + (*s - '0');
    }
    *unit = val;
    return true;
}

static void afk_compact_announce(afk_epic_ep_t *epic, u16 channel, const u8 *payload,
                                 size_t payload_size)
{
    char wire_name[33], service_name[33];
    const char *name;
    s64 unit = -1;

    if (payload_size < 32) {
        printf("AFK[ep:%02x]: short compact service announcement\n", epic->ep);
        return;
    }
    if (afk_epic_find_service(epic, channel)) {
        printf("AFK[ep:%02x]: duplicate compact service on interface %u\n", epic->ep, channel);
        return;
    }

    memcpy(wire_name, payload, 32);
    wire_name[32] = '\0';
    name = strchr(wire_name, ':');
    name = name ? name + 1 : wire_name;
    strncpy(service_name, name, sizeof(service_name) - 1);
    service_name[sizeof(service_name) - 1] = '\0';

    char *unit_suffix = strrchr(service_name, ':');
    if (unit_suffix && afk_parse_unit(unit_suffix + 1, &unit))
        *unit_suffix = '\0';
    else
        unit = -1;

    const afk_epic_service_ops_t *ops = afk_compact_match_service(epic, wire_name);
    if (!ops) {
        printf("AFK[ep:%02x]: unable to match compact service %s on interface %u\n", epic->ep,
               wire_name, channel);
        return;
    }

    /* AV services may be republished under new interface IDs (hotplug). */
    u32 idx = epic->num_channels;
    if (epic->ep >= 0x27 && epic->ep <= 0x29) {
        for (u32 i = 0; i < epic->num_channels; i++) {
            afk_epic_service_t *s = &epic->services[i];
            if (s->ops == ops && s->unit == unit && !strcmp(s->name, service_name)) {
                idx = i;
                break;
            }
        }
    }
    if (idx == epic->num_channels) {
        for (u32 i = 0; i < epic->num_channels; i++) {
            if (!epic->services[i].enabled) {
                idx = i;
                break;
            }
        }
    }
    if (idx == epic->num_channels) {
        if (epic->num_channels >= AFK_MAX_CHANNEL) {
            printf("AFK[ep:%02x]: too many compact services\n", epic->ep);
            return;
        }
        epic->num_channels++;
    }

    afk_epic_service_t *service = &epic->services[idx];
    memset(service, 0, sizeof(*service));
    service->enabled = true;
    service->ops = ops;
    service->epic = epic;
    service->intf = epic->intf;
    service->channel = channel;
    service->unit = unit;
    strncpy(service->name, service_name, sizeof(service->name) - 1);

    if (ops->init)
        ops->init(service, service_name, service_name, unit);

    printf("AFK[ep:%02x]: compact service %s on interface %u%s\n", epic->ep, wire_name, channel,
           service->defer_start ? " (deferred)" : "");

    if (!service->defer_start)
        afk_epic_open_service(service);
}

static void afk_compact_call(afk_epic_ep_t *epic, u16 channel, const u8 *payload,
                             size_t payload_size)
{
    const struct epic_compact_desc *request_desc = (const void *)payload;
    const struct afk_service_call_hdr *call;
    int ret;

    if (payload_size < sizeof(*request_desc) + sizeof(*call))
        return;

    call = (const void *)(payload + sizeof(*request_desc));
    size_t inner_size = payload_size - sizeof(*request_desc);
    size_t call_size = call->data_len;
    if (call->magic != EPIC_SERVICE_CALL_MAGIC || inner_size < sizeof(*call) + call_size) {
        printf("AFK[ep:%02x]: malformed compact service call on interface %u\n", epic->ep,
               channel);
        return;
    }

    tprintf("AFK[ep:%02x]: call intf %u group %u command %u size 0x%lx\n", epic->ep, channel,
            call->group, call->command, call_size);

    u8 *reply = calloc(payload_size, 1);
    if (!reply)
        return;

    struct epic_compact_desc *reply_desc = (void *)reply;
    struct afk_service_call_hdr *reply_call = (void *)(reply + sizeof(*reply_desc));
    reply_desc->status = request_desc->status;
    memcpy(reply_call, call, sizeof(*call));

    afk_epic_service_t *service = afk_epic_find_service(epic, channel);
    if (service && service->ops->call_group) {
        ret = service->ops->call_group(service, call->group, call->command, call + 1, call_size,
                                       reply_call + 1, call_size);
    } else if (service && service->ops->call && call->group == 0) {
        ret = service->ops->call(service, call->command, call + 1, call_size, reply_call + 1,
                                 call_size);
    } else {
        printf("AFK[ep:%02x]: no handler for compact call %u:%u on interface %u\n", epic->ep,
               call->group, call->command, channel);
        ret = -38; /* -ENOSYS, as the Linux driver answers */
    }
    if (ret)
        reply_desc->status = (u32)ret;

    if (afk_compact_send(epic, channel, SUBTYPE_STD_SERVICE, CCAT_REPLY, reply, payload_size,
                         payload_size))
        printf("AFK[ep:%02x]: failed to reply to compact call\n", epic->ep);

    free(reply);
}

static void afk_compact_reply(afk_epic_ep_t *epic, u16 channel, u8 type, const u8 *payload,
                              size_t payload_size)
{
    struct afk_compact_pending *p = &epic->pending;
    const struct epic_compact_desc *desc = (const void *)payload;

    if (payload_size < sizeof(*desc)) {
        printf("AFK[ep:%02x]: compact reply without descriptor\n", epic->ep);
        return;
    }

    if (!p->active || p->done || p->channel != channel || p->type != type ||
        p->status != desc->status) {
        printf("AFK[ep:%02x]: unexpected compact reply %02x on interface %u (status 0x%x)\n",
               epic->ep, type, channel, desc->status);
        return;
    }

    size_t data_size = payload_size - sizeof(*desc);
    if (p->service_call && data_size >= sizeof(struct afk_service_call_hdr)) {
        const struct afk_service_call_hdr *call = (const void *)(desc + 1);
        if (call->magic == EPIC_SERVICE_CALL_MAGIC &&
            (call->group != p->group || call->command != p->command)) {
            printf("AFK[ep:%02x]: compact reply for another call\n", epic->ep);
            return;
        }
    }

    size_t copy = min(p->out_len, data_size);
    if (copy && p->out)
        memcpy(p->out, desc + 1, copy);
    p->got = copy;
    p->value = desc->length;
    p->done = true;
}

static void afk_compact_dispatch(afk_epic_ep_t *epic, u16 channel, const u8 *body, size_t size)
{
    const struct epic_compact_sub_hdr *sub = (const void *)(body + COMPACT_TIMESTAMP_SIZE);
    const u8 *payload = body + COMPACT_BODY_HDR_SIZE;
    size_t payload_size = size - COMPACT_BODY_HDR_SIZE;

    if (sub->category == CCAT_REPORT && sub->type == CTYPE_ANNOUNCE)
        afk_compact_announce(epic, channel, payload, payload_size);
    else if (sub->category == CCAT_COMMAND && sub->type == SUBTYPE_STD_SERVICE)
        afk_compact_call(epic, channel, payload, payload_size);
    else if (sub->category == CCAT_REPLY)
        afk_compact_reply(epic, channel, sub->type, payload, payload_size);
    else
        tprintf("AFK[ep:%02x]: compact message %02x:%02x on interface %u (0x%lx bytes)\n",
                epic->ep, sub->category, sub->type, channel, payload_size);
}

static void afk_compact_reset_rx(afk_epic_ep_t *epic)
{
    free(epic->crx_buf);
    epic->crx_buf = NULL;
    epic->crx_total = 0;
    epic->crx_received = 0;
}

static void afk_compact_handle_qe(afk_epic_ep_t *epic, const struct afk_qe *qe)
{
    const struct epic_compact_hdr *hdr = (const void *)qe->data;
    const size_t hdr_size = sizeof(*hdr);

    if (qe->channel || qe->type || qe->size < hdr_size) {
        printf("AFK[ep:%02x]: unexpected non-compact message\n", epic->ep);
        return;
    }

    u32 total = hdr->length;
    u16 intf_id = hdr->intf_id;
    const u8 *frag = qe->data + hdr_size;
    u32 frag_size = qe->size - hdr_size;

    /* Eight byte empty control messages occur in native traces. */
    if (!epic->crx_received && total == 8 && frag_size == 8)
        return;

    if (total < COMPACT_BODY_HDR_SIZE || total > COMPACT_MESSAGE_MAX || !frag_size ||
        frag_size > total)
        goto discard;
    if (epic->crx_received && (epic->crx_total != total || epic->crx_intf != intf_id ||
                               epic->crx_next_seq != hdr->seq))
        goto discard;
    if (frag_size > total - epic->crx_received)
        goto discard;

    if (!epic->crx_received && frag_size == total) {
        afk_compact_dispatch(epic, intf_id, frag, total);
        return;
    }

    if (!epic->crx_received) {
        epic->crx_buf = malloc(total);
        if (!epic->crx_buf)
            goto discard;
        epic->crx_total = total;
        epic->crx_intf = intf_id;
    }
    memcpy(epic->crx_buf + epic->crx_received, frag, frag_size);
    epic->crx_received += frag_size;
    epic->crx_next_seq = hdr->seq + 1;

    if (epic->crx_received == epic->crx_total) {
        u8 *message = epic->crx_buf;
        epic->crx_buf = NULL;
        epic->crx_received = 0;
        afk_compact_dispatch(epic, intf_id, message, total);
        free(message);
    }
    return;

discard:
    printf("AFK[ep:%02x]: invalid compact fragment intf %u seq %u total 0x%x\n", epic->ep,
           intf_id, hdr->seq, total);
    afk_compact_reset_rx(epic);
}

static void afk_compact_recv_handler(afk_epic_ep_t *epic)
{
    struct afk_qe *qe;

    if (afk_epic_rx(epic, &qe) < 0) {
        printf("AFK[ep:%02x]: bad receive queue, ignoring this endpoint\n", epic->ep);
        epic->broken = true;
        return;
    }

    afk_compact_handle_qe(epic, qe);
    afk_epic_rx_ack(epic);
}

static int afk_compact_command(afk_epic_ep_t *epic, int channel, u16 sub_type, void *txbuf,
                               size_t txsize, void *rxbuf, size_t *rxsize)
{
    struct afk_compact_pending *p = &epic->pending;
    afk_epic_service_t *service = afk_epic_find_service(epic, channel);
    size_t cap = rxsize ? *rxsize : 0;
    size_t wire = max(txsize, cap);

    if (epic->broken || channel < 0 || channel > 0xffff || sub_type > 0xff || afk_out_of_time())
        return -1;

    u8 tag = service ? service->cmd_tag++ : epic->cmd_tag++;

    u8 *payload = calloc(sizeof(struct epic_compact_desc) + wire, 1);
    if (!payload)
        return -1;

    struct epic_compact_desc *desc = (void *)payload;
    desc->status = (u32)tag << 8;
    desc->length = wire;
    memcpy(payload + sizeof(*desc), txbuf, txsize);

    memset(p, 0, sizeof(*p));
    p->active = true;
    p->channel = channel;
    p->type = sub_type;
    p->status = desc->status;
    p->out = rxbuf;
    p->out_len = cap;
    if (txsize >= sizeof(struct afk_service_call_hdr)) {
        const struct afk_service_call_hdr *call = txbuf;
        if (call->magic == EPIC_SERVICE_CALL_MAGIC) {
            p->service_call = true;
            p->group = call->group;
            p->command = call->command;
        }
    }

    int ret = afk_compact_send(epic, channel, sub_type, CCAT_COMMAND, payload,
                               sizeof(*desc) + txsize, sizeof(*desc) + wire);
    free(payload);
    if (ret) {
        p->active = false;
        return -1;
    }

    u64 deadline = timeout_calculate(AFK_COMMAND_TIMEOUT_US);
    int exc_before = exc_count;
    while (!p->done) {
        if (exc_count != exc_before) {
            printf("AFK[ep:%02x]: CPU exception while waiting, giving up\n", epic->ep);
            p->active = false;
            return -1;
        }
        if (timeout_expired(deadline)) {
            printf("AFK[ep:%02x]: timed out waiting for reply on interface %d\n", epic->ep,
                   channel);
            p->active = false;
            return -1;
        }
        ret = afk_epic_work(epic->afk, -1);
        if (ret < 0 || epic->broken) {
            p->active = false;
            return -1;
        }
    }
    p->active = false;

    if (rxsize)
        *rxsize = p->got;

    /* Replies carry an IOReturn here; errors have the top bit set. */
    if (p->value & 0x80000000) {
        printf("AFK[ep:%02x]: command on interface %d returned 0x%x\n", epic->ep, channel,
               p->value);
        return -1;
    }

    return 0;
}

int afk_epic_wait_services(afk_epic_ep_t *epic, int expected, u32 timeout_us)
{
    u64 deadline = timeout_calculate(timeout_us);
    int count;

    while (true) {
        count = 0;
        for (u32 i = 0; i < epic->num_channels; i++)
            if (epic->services[i].enabled)
                count++;
        if (count >= expected || timeout_expired(deadline) || epic->broken ||
            (afk_deadline && timeout_expired(afk_deadline)))
            break;
        if (afk_epic_work(epic->afk, -1) < 0)
            break;
    }

    return count;
}

/* ------------------------------------------------------------------------- */

struct epic_std_service_ap_call {
    u32 unk0;
    u32 unk1;
    u32 type;
    u32 len;
    u32 magic;
    u8 _unk[48];
} PACKED;

static int afk_epic_handle_std_service(afk_epic_ep_t *epic, int channel, u8 category, u16 sub_seq,
                                       void *payload, size_t payload_size)
{
    afk_epic_service_t *service = afk_epic_find_service(epic, channel);

    if (service && service->ops->call && category == CAT_NOTIFY) {
        struct epic_std_service_ap_call *call = payload;
        size_t call_size;
        void *reply;
        int ret;

        if (payload_size < sizeof(*call))
            return -1;

        call_size = call->len;
        if (payload_size < sizeof(*call) + call_size)
            return -1;

        if (!service->ops->call)
            return 0;
        reply = calloc(payload_size, 1);
        if (!reply)
            return -1;

        ret = service->ops->call(service, call->type, payload + sizeof(*call), call_size,
                                 reply + sizeof(*call), call_size);
        if (ret) {
            free(reply);
            return ret;
        }

        memcpy(reply, call, sizeof(*call));

        size_t tx_size = sizeof(struct epic_hdr) + sizeof(struct epic_sub_hdr) + payload_size;
        void *msg = calloc(tx_size, 1);

        struct epic_hdr *hdr = msg;
        struct epic_sub_hdr *sub = msg + sizeof(struct epic_hdr);

        hdr->version = 2;
        hdr->seq = epic->seq++;

        sub->length = payload_size;
        sub->version = 4;
        sub->category = CAT_REPLY;
        sub->type = SUBTYPE_STD_SERVICE;
        sub->seq = sub_seq; // service->seq++;
        sub->flags = 0x08;
        sub->inline_len = payload_size - 4;

        memcpy(msg + sizeof(struct epic_hdr) + sizeof(struct epic_sub_hdr), reply, payload_size);

        afk_epic_tx(epic, channel, TYPE_NOTIFY_ACK, msg, tx_size);
        free(reply);
        free(msg);

        return 0;
    }

    dprintf("AFK: channel %d received unhandled standard service message: %x\n", channel, category);

    return -1;
}

int afk_epic_command(afk_epic_ep_t *epic, int channel, u16 sub_type, void *txbuf, size_t txsize,
                     void *rxbuf, size_t *rxsize)
{
    if (epic->compact)
        return afk_compact_command(epic, channel, sub_type, txbuf, txsize, rxbuf, rxsize);

    struct {
        struct epic_hdr hdr;
        struct epic_sub_hdr sub;
        struct epic_cmd cmd;
    } PACKED msg;

    assert(txsize <= epic->txbuf.sz);
    assert(!rxsize || *rxsize <= epic->rxbuf.sz);

    memset(&msg, 0, sizeof(msg));

    msg.hdr.version = 2;
    msg.hdr.seq = epic->seq++;
    msg.sub.length = sizeof(msg.cmd);
    msg.sub.version = 4;
    msg.sub.category = CAT_COMMAND;
    msg.sub.type = sub_type;
    msg.sub.seq = 0;
    msg.cmd.txbuf = epic->txbuf.dva;
    msg.cmd.txlen = txsize;
    msg.cmd.rxbuf = epic->rxbuf.dva;
    msg.cmd.rxlen = rxsize ? *rxsize : 0;

    memcpy(epic->txbuf.bfr, txbuf, txsize);

    int ret = afk_epic_tx(epic, channel, TYPE_COMMAND, &msg, sizeof msg);
    if (ret < 0) {
        printf("EPIC: failed to transmit command\n");
        return ret;
    }

    struct afk_qe *rmsg;
    struct epic_cmd *rcmd;
    u64 deadline = timeout_calculate(AFK_COMMAND_TIMEOUT_US);

    while (true) {
        if (timeout_expired(deadline)) {
            printf("EPIC: timed out waiting for command reply\n");
            return -1;
        }
        ret = afk_epic_work(epic->afk, epic->ep);
        if (ret < 0)
            return ret;
        else if (ret != EPIC_DATA_READY)
            continue;
        // will not block
        ret = afk_epic_rx(epic, &rmsg);
        if (ret < 0)
            return ret;

        if (rmsg->type != TYPE_REPLY && rmsg->type != TYPE_NOTIFY) {
            printf("EPIC: got unexpected message type %d during command\n", rmsg->type);
            afk_epic_rx_ack(epic);
            continue;
        }

        struct epic_hdr *hdr = (void *)(rmsg + 1);
        struct epic_sub_hdr *sub = (void *)(hdr + 1);

        if (sub->category == CAT_NOTIFY && sub->type == SUBTYPE_STD_SERVICE) {
            void *payload = rmsg->data + sizeof(struct epic_hdr) + sizeof(struct epic_sub_hdr);
            size_t payload_size =
                rmsg->size - sizeof(struct epic_hdr) - sizeof(struct epic_sub_hdr);
            afk_epic_rx_ack(epic);
            afk_epic_handle_std_service(epic, channel, sub->category, sub->seq, payload,
                                        payload_size);
            continue;
        } else if (sub->category != CAT_REPLY || sub->type != sub_type) {
            printf("EPIC: got unexpected message %02x:%04x during command\n", sub->category,
                   sub->type);
            afk_epic_rx_ack(epic);
            continue;
        }

        rcmd = (void *)(sub + 1);
        break;
    }

    if (rcmd->retcode != 0) {
        printf("EPIC: IOP returned 0x%x\n", rcmd->retcode);
        afk_epic_rx_ack(epic);
        return rcmd->retcode; // should be negative already
    }

    if (rxsize) {
        assert(*rxsize >= rcmd->rxlen);
        *rxsize = rcmd->rxlen;

        if (*rxsize && rcmd->rxbuf)
            memcpy(rxbuf, epic->rxbuf.bfr, *rxsize);
    }

    afk_epic_rx_ack(epic);

    return 0;
}

static void afk_epic_notify_handler(afk_epic_ep_t *epic)
{
    struct afk_qe *rmsg;
    // will not block
    int ret = afk_epic_rx(epic, &rmsg);
    if (ret < 0)
        return;

    if (rmsg->type != TYPE_NOTIFY) {
        dprintf("EPIC[0x%02x]: got unexpected message type %d in %s\n", epic->ep, rmsg->type,
                __func__);
        afk_epic_rx_ack(epic);
        return;
    }

    struct epic_hdr *hdr = (void *)(rmsg + 1);
    struct epic_sub_hdr *sub = (void *)(hdr + 1);

    if (sub->category == CAT_NOTIFY && sub->type == SUBTYPE_STD_SERVICE) {
        void *payload = rmsg->data + sizeof(struct epic_hdr) + sizeof(struct epic_sub_hdr);
        size_t payload_size = rmsg->size - sizeof(struct epic_hdr) - sizeof(struct epic_sub_hdr);
        afk_epic_handle_std_service(epic, rmsg->channel, sub->category, sub->seq, payload,
                                    payload_size);
    } else {
        dprintf("EPIC[0x%02x]: %s: rx: Ch %u, Type:0x%02x sub cat:%x type:%x \n", epic->ep,
                __func__, rmsg->channel, rmsg->type, sub->category, sub->type);
    }

    afk_epic_rx_ack(epic);
}

afk_epic_ep_t *afk_epic_start_ep_intf(afk_epic_t *afk, int endpoint,
                                      const afk_epic_service_ops_t *ops, bool notify, void *intf)
{
    if (endpoint < 0x20 || endpoint >= 0x30 || afk->endpoint[endpoint - 0x20]) {
        printf("EPIC: invalid or busy endpoint 0x%02x\n", endpoint);
        return NULL;
    }

    if (afk->compact && afk_out_of_time())
        return NULL;

    afk_epic_ep_t *epic = calloc(1, sizeof(afk_epic_ep_t));
    if (!epic)
        return NULL;

    epic->ep = endpoint;
    epic->afk = afk;
    epic->ops = ops;
    epic->intf = intf;
    epic->compact = afk->compact;
    afk->endpoint[endpoint - 0x20] = epic;

    if (epic->compact)
        epic->recv_handler = afk_compact_recv_handler;
    else if (notify)
        epic->recv_handler = afk_epic_notify_handler;

    if (!rtkit_start_ep(epic->afk->rtk, endpoint)) {
        printf("EPIC: failed to start endpoint %d\n", endpoint);
        goto err;
    }

    struct rtkit_message msg = {endpoint, FIELD_PREP(RBEP_TYPE, RBEP_INIT)};
    if (!rtkit_send(epic->afk->rtk, &msg)) {
        printf("EPIC: failed to send init message\n");
        goto err;
    }

    u64 deadline = timeout_calculate(AFK_START_TIMEOUT_US);
    while (!epic->started) {
        int ret = afk_epic_poll(epic->afk, epic->compact ? -1 : endpoint, deadline);
        if (ret == EPIC_POLL_TIMEOUT || (!epic->started && timeout_expired(deadline))) {
            printf("EPIC: endpoint 0x%02x did not start\n", endpoint);
            goto err;
        } else if (ret < 0) {
            goto err;
        } else if (ret > 0)
            printf("EPIC: received unexpected message during init\n");
    }

    return epic;

err:
    /* The IOP may already own the shared buffer; never hand it back. */
    afk->endpoint[endpoint - 0x20] = NULL;
    free(epic->crx_buf);
    free(epic);
    return NULL;
}

afk_epic_ep_t *afk_epic_start_ep(afk_epic_t *afk, int endpoint, const afk_epic_service_ops_t *ops,
                                 bool notify)
{
    return afk_epic_start_ep_intf(afk, endpoint, ops, notify, NULL);
}

int afk_epic_shutdown_ep(afk_epic_ep_t *epic)
{
    int ret = 0;
    struct rtkit_message msg = {epic->ep, FIELD_PREP(RBEP_TYPE, RBEP_SHUTDOWN)};
    if (!rtkit_send(epic->afk->rtk, &msg)) {
        printf("EPIC: failed to send shutdown message\n");
        ret = -1;
    }

    u64 deadline = timeout_calculate(AFK_SHUTDOWN_TIMEOUT_US);
    while (!ret && epic->started) {
        int r = afk_epic_poll(epic->afk, epic->compact ? -1 : epic->ep, deadline);
        if (r == EPIC_POLL_TIMEOUT || (epic->started && timeout_expired(deadline))) {
            printf("EPIC: endpoint 0x%02x did not stop\n", epic->ep);
            ret = -1;
        } else if (r < 0)
            ret = -1;
    }

    epic->afk->endpoint[epic->ep - 0x20] = NULL;

    if (ret) {
        /* The IOP may still use the shared buffers; keep them. */
        free(epic->crx_buf);
        free(epic);
        return ret;
    }

    rtkit_free_buffer(epic->afk->rtk, &epic->buf);
    rtkit_free_buffer(epic->afk->rtk, &epic->rxbuf);
    rtkit_free_buffer(epic->afk->rtk, &epic->txbuf);

    free(epic->crx_buf);
    free(epic);
    return 0;
}

static const afk_epic_service_ops_t *afk_match_service(afk_epic_ep_t *ep, const char *name)
{
    const afk_epic_service_ops_t *ops;

    if (!name[0])
        return NULL;
    if (!ep->ops)
        return NULL;

    for (ops = ep->ops; ops->name[0]; ops++) {
        if (strcmp(ops->name, name))
            continue;

        return ops;
    }

    return NULL;
}

int afk_epic_start_interface(afk_epic_ep_t *epic, void *intf, int expected, size_t txsize,
                             size_t rxsize)
{
    int services = 0;
    struct afk_qe *msg;
    struct epic_announce *announce;

    if (epic->compact) {
        /* Services announce themselves while the endpoint starts and later;
         * commands are inline, so no command buffers are needed. */
        epic->intf = intf;
        for (u32 i = 0; i < epic->num_channels; i++)
            epic->services[i].intf = intf;

        services = afk_epic_wait_services(epic, expected, 1000000);
        if (!services) {
            printf("AFK[ep:%02x]: no compact services announced\n", epic->ep);
            return -1;
        }
        return 0;
    }

    /* consume messages for other endpoints, syslog or ioreport might be noisy
     * at startup */
    u64 deadline = timeout_calculate(2000000);
    while (1) {
        int ret = afk_epic_work(epic->afk, epic->ep);
        if (ret < 0)
            return ret;
        if (ret == EPIC_DATA_READY)
            break;
        if (timeout_expired(deadline)) {
            printf("AFK[ep:%02x]: no service announcement\n", epic->ep);
            return -1;
        }
    }

    u64 timeout = timeout_calculate(500000);

    while (!timeout_expired(timeout)) {
        s64 epic_unit = -1;
        char *epic_name = NULL;
        char *epic_class = NULL;
        const char *service_name = NULL;

        int ret = afk_epic_work(epic->afk, epic->ep);
        if (ret < 0)
            return ret;
        else if (ret != EPIC_DATA_READY)
            continue;

        ret = afk_epic_rx(epic, &msg);
        if (ret < 0)
            return ret;

        if (msg->type != TYPE_NOTIFY && msg->type != TYPE_REPLY) {
            dprintf("AFK[ep:%02x]: got unexpected message type %d during iface start\n", epic->ep,
                    msg->type);
            afk_epic_rx_ack(epic);
            continue;
        }

        struct epic_hdr *hdr = (void *)(msg + 1);
        struct epic_sub_hdr *sub = (void *)(hdr + 1);

        if (sub->category != CAT_REPORT || sub->type != SUBTYPE_ANNOUNCE) {
            dprintf("AFK[ep:%02x]: got unexpected message %02x:%04x during iface start\n", epic->ep,
                    sub->category, sub->type);
            afk_epic_rx_ack(epic);
            continue;
        }

        if (epic->num_channels >= AFK_MAX_CHANNEL) {
            printf("AFK[ep:%02x]: Out of free service for service on channel %d\n", epic->ep,
                   msg->channel);
            afk_epic_rx_ack(epic);
            continue;
        }

        announce = (void *)(sub + 1);

        size_t props_size = sub->length - offsetof(struct epic_announce, props);

        if (props_size > 36) {
            struct dcp_parse_ctx ctx;

            int ret = parse(announce->props, props_size, &ctx);
            if (ret) {
                printf("AFK[ep:%02x]: Failed to parse service init props (len=%zu) for %s\n",
                       epic->ep, props_size, announce->name);
                afk_epic_rx_ack(epic);
                continue;
            }
            ret = parse_epic_service_init(&ctx, &epic_name, &epic_class, &epic_unit);
            if (ret) {
                printf("AFK[ep:%02x]: failed to extract init props (len=%zu): %d\n", epic->ep,
                       props_size, ret);
                hexdump(announce->props, props_size);
                afk_epic_rx_ack(epic);
                continue;
            }
            service_name = epic_class;
        } else {
            service_name = announce->name;
        }

        const afk_epic_service_ops_t *ops = afk_match_service(epic, service_name);
        if (!ops) {
            printf("AFK[ep:%02x]: unable to match service %s on channel %d\n", epic->ep,
                   service_name, msg->channel);
            afk_epic_rx_ack(epic);
            continue;
        }

        afk_epic_service_t *service = &epic->services[epic->num_channels++];
        service->enabled = true;
        service->ops = ops;
        service->intf = intf;
        service->epic = epic;
        service->channel = msg->channel;
        service->seq = 0;

        ops->init(service, epic_name, service_name, epic_unit);
        dprintf("AFK[ep:%02x]: new service %s on channel %d\n", epic->ep, service_name,
                msg->channel);
        free(epic_name);
        free(epic_class);

        afk_epic_rx_ack(epic);
        if (++services >= expected)
            break;
    }

    if (!services) {
        printf("AFK[ep:%02x]: too many unexpected messages, giving up\n", epic->ep);
        return -1;
    }

    if (!rtkit_alloc_buffer(epic->afk->rtk, &epic->rxbuf, rxsize)) {
        printf("AFK[ep:%02x]: failed to allocate rx buffer\n", epic->ep);
        return -1;
    }

    if (!rtkit_alloc_buffer(epic->afk->rtk, &epic->txbuf, txsize)) {
        printf("AFK[ep:%02x]: failed to allocate tx buffer\n", epic->ep);
        return -1;
    }

    dprintf("AFK[ep:%02x]: started interface with %d services\n", epic->ep, services);

    return 0;
}

afk_epic_t *afk_epic_init(rtkit_dev_t *rtkit)
{
    afk_epic_t *afk = calloc(sizeof(*afk), 1);

    if (!afk)
        return NULL;

    afk->rtk = rtkit;

    return afk;
}

int afk_epic_shutdown(afk_epic_t *afk)
{
    int ret = 0;

    for (int i = 0; i < 0x10; i++)
        if (afk->endpoint[i] && afk_epic_shutdown_ep(afk->endpoint[i]) < 0)
            ret = -1;

    free(afk);

    return ret;
}
