/**
 * @file canard.c
 * @brief Lightweight UAVCAN v0 stack implementation.
 *
 * Handles single-frame and multi-frame transfer reassembly.
 * For multi-frame transfers the first two payload bytes carry a
 * CRC-16/MCRF4XX computed over the entire payload and the
 * data-type signature.
 */
#include "canard.h"
#include <string.h>

/* ------------------------------------------------------------------ */
/*  Internal constants                                                  */
/* ------------------------------------------------------------------ */

#define TAIL_START_OF_TRANSFER  (1U << 7U)
#define TAIL_END_OF_TRANSFER    (1U << 6U)
#define TAIL_TOGGLE             (1U << 5U)
#define TAIL_TRANSFER_ID_MASK   (0x1FU)

/* Bits in the 29-bit CAN ID */
#define CANID_SERVICE_FLAG      (1UL << 7U)
#define CANID_DTID_SHIFT        8U
#define CANID_DTID_MASK         0xFFFFUL
#define CANID_SRC_MASK          0x7FUL
#define CANID_PRIORITY_SHIFT    24U
#define CANID_PRIORITY_MASK     0x1FUL

/* ------------------------------------------------------------------ */
/*  CRC-16/MCRF4XX                                                      */
/* ------------------------------------------------------------------ */

static uint16_t crc16Add(uint16_t crc, uint8_t byte)
{
    crc ^= (uint16_t)byte;
    for (int i = 0; i < 8; i++) {
        if (crc & 1U) {
            crc = (uint16_t)((crc >> 1U) ^ 0x8408U);
        } else {
            crc >>= 1U;
        }
    }
    return crc;
}

/**
 * Compute CRC-16/MCRF4XX over a data-type signature (8 bytes, little-endian)
 * followed by @p len payload bytes.  Used to verify multi-frame transfers.
 */
static uint16_t crc16WithSignature(const uint8_t *data, size_t len,
                                    uint64_t data_type_signature)
{
    uint16_t crc = 0xFFFFU;
    for (int i = 0; i < 8; i++) {
        crc = crc16Add(crc, (uint8_t)(data_type_signature >> (i * 8U)));
    }
    for (size_t i = 0; i < len; i++) {
        crc = crc16Add(crc, data[i]);
    }
    return crc;
}

/* ------------------------------------------------------------------ */
/*  Helper: find or create an RX session                               */
/* ------------------------------------------------------------------ */

static CanardRxSession *findRxSession(CanardInstance *ins,
                                       uint8_t  source_node_id,
                                       uint16_t data_type_id)
{
    CanardRxSession *free_slot = NULL;
    for (size_t i = 0; i < CANARD_RX_SESSIONS_MAX; i++) {
        CanardRxSession *s = &ins->rx_sessions[i];
        if (s->in_use &&
            s->source_node_id == source_node_id &&
            s->data_type_id   == data_type_id) {
            return s;
        }
        if (!s->in_use && free_slot == NULL) {
            free_slot = s;
        }
    }
    return free_slot; /* NULL when all slots are occupied */
}

static void resetRxSession(CanardRxSession *s)
{
    memset(s, 0, sizeof(*s));
}

/* ------------------------------------------------------------------ */
/*  Public API                                                          */
/* ------------------------------------------------------------------ */

void canardInit(CanardInstance             *out_ins,
                CanardOnTransferReceived    on_reception,
                CanardShouldAcceptTransfer  should_accept,
                void                       *user_reference)
{
    memset(out_ins, 0, sizeof(*out_ins));
    out_ins->on_transfer_received = on_reception;
    out_ins->should_accept        = should_accept;
    out_ins->user_reference       = user_reference;
}

bool canardIsMessageTransfer(uint32_t frame_id)
{
    /* Strip the EFF flag first */
    frame_id &= ~CANARD_CAN_FRAME_EFF;
    return (frame_id & CANID_SERVICE_FLAG) == 0U;
}

uint16_t canardExtractDataTypeID(uint32_t frame_id)
{
    frame_id &= ~CANARD_CAN_FRAME_EFF;
    if (!canardIsMessageTransfer(frame_id)) {
        return 0U;
    }
    return (uint16_t)((frame_id >> CANID_DTID_SHIFT) & CANID_DTID_MASK);
}

int canardHandleRxFrame(CanardInstance       *ins,
                         const CanardCANFrame *frame,
                         uint64_t              timestamp_usec)
{
    /* Only accept extended (29-bit) frames */
    if ((frame->id & CANARD_CAN_FRAME_EFF) == 0U) {
        return -1;
    }
    /* Reject error/remote frames */
    if ((frame->id & (CANARD_CAN_FRAME_RTR | CANARD_CAN_FRAME_ERR)) != 0U) {
        return -1;
    }
    /* Need at least the tail byte */
    if (frame->data_len < 1U) {
        return -1;
    }
    /* We handle only message broadcasts */
    if (!canardIsMessageTransfer(frame->id)) {
        return -1;
    }

    const uint32_t raw_id  = frame->id & ~CANARD_CAN_FRAME_EFF;
    const uint16_t dtid    = (uint16_t)((raw_id >> CANID_DTID_SHIFT) & CANID_DTID_MASK);
    const uint8_t  src     = (uint8_t)(raw_id & CANID_SRC_MASK);
    const uint8_t  priority= (uint8_t)((raw_id >> CANID_PRIORITY_SHIFT) & CANID_PRIORITY_MASK);

    const uint8_t  tail       = frame->data[frame->data_len - 1U];
    const uint8_t  transfer_id = tail & TAIL_TRANSFER_ID_MASK;
    const bool     sot         = (tail & TAIL_START_OF_TRANSFER) != 0U;
    const bool     eot         = (tail & TAIL_END_OF_TRANSFER)   != 0U;
    const bool     toggle      = (tail & TAIL_TOGGLE)             != 0U;
    const uint8_t  payload_len = frame->data_len - 1U; /* exclude tail byte */

    /* ----------------------------------------------------------------
     * Single-frame transfer (SOT=1, EOT=1)
     * ---------------------------------------------------------------- */
    if (sot && eot) {
        /* Ask the application whether it wants this data type */
        uint64_t sig = 0U;
        if (ins->should_accept != NULL &&
            !ins->should_accept(ins, &sig, dtid, CanardTransferTypeBroadcast, src)) {
            return 0;
        }

        CanardRxTransfer transfer;
        transfer.timestamp_usec       = timestamp_usec;
        transfer.payload              = frame->data;
        transfer.payload_len          = payload_len;
        transfer.data_type_signature  = sig;
        transfer.data_type_id         = dtid;
        transfer.transfer_type        = CanardTransferTypeBroadcast;
        transfer.transfer_id          = transfer_id;
        transfer.priority             = priority;
        transfer.source_node_id       = src;

        if (ins->on_transfer_received != NULL) {
            ins->on_transfer_received(ins, &transfer);
        }
        return 1;
    }

    /* ----------------------------------------------------------------
     * Multi-frame transfer
     * ---------------------------------------------------------------- */
    CanardRxSession *s = findRxSession(ins, src, dtid);
    if (s == NULL) {
        return -2; /* No session slots available */
    }

    if (sot) {
        /* First frame of a multi-frame transfer */
        uint64_t sig = 0U;
        if (ins->should_accept != NULL &&
            !ins->should_accept(ins, &sig, dtid, CanardTransferTypeBroadcast, src)) {
            return 0;
        }
        resetRxSession(s);
        s->in_use              = true;
        s->data_type_id        = dtid;
        s->data_type_signature = sig;
        s->source_node_id      = src;
        s->transfer_id         = transfer_id;
        s->timestamp_usec      = timestamp_usec;
        s->next_toggle         = 1U; /* after the SOT frame, toggle flips */

        /* First two bytes of the first frame carry the transfer CRC
           (little-endian, CRC-16/MCRF4XX seeded with the data-type signature). */
        if (payload_len < 2U) {
            resetRxSession(s);
            return -3;
        }
        s->received_crc = (uint16_t)((uint16_t)frame->data[0] |
                                     ((uint16_t)frame->data[1] << 8U));

        /* Copy actual payload bytes (skip the 2-byte CRC prefix) */
        if (payload_len > 2U) {
            size_t copy = payload_len - 2U;
            if (copy > CANARD_TRANSFER_PAYLOAD_LEN_MAX) {
                resetRxSession(s);
                return -3;
            }
            memcpy(s->payload, frame->data + 2U, copy);
            s->payload_len = copy;
        }
        return 0;
    }

    /* Continuation or last frame */
    if (!s->in_use || s->transfer_id != transfer_id) {
        /* Out-of-order or stale — drop */
        return -4;
    }
    if ((uint8_t)toggle != s->next_toggle) {
        /* Toggle mismatch — drop and invalidate session */
        resetRxSession(s);
        return -5;
    }
    s->next_toggle ^= 1U;

    /* Append payload */
    if (s->payload_len + payload_len > CANARD_TRANSFER_PAYLOAD_LEN_MAX) {
        resetRxSession(s);
        return -3;
    }
    memcpy(s->payload + s->payload_len, frame->data, payload_len);
    s->payload_len += payload_len;

    if (eot) {
        /* Transfer complete — verify CRC-16/MCRF4XX */
        uint16_t computed_crc = crc16WithSignature(s->payload,
                                                    s->payload_len,
                                                    s->data_type_signature);
        if (computed_crc != s->received_crc) {
            resetRxSession(s);
            return -6; /* CRC mismatch */
        }

        CanardRxTransfer transfer;
        transfer.timestamp_usec       = s->timestamp_usec;
        transfer.payload              = s->payload;
        transfer.payload_len          = s->payload_len;
        transfer.data_type_signature  = s->data_type_signature;
        transfer.data_type_id         = dtid;
        transfer.transfer_type        = CanardTransferTypeBroadcast;
        transfer.transfer_id          = s->transfer_id;
        transfer.priority             = priority;
        transfer.source_node_id       = src;

        if (ins->on_transfer_received != NULL) {
            ins->on_transfer_received(ins, &transfer);
        }
        resetRxSession(s);
        return 1;
    }

    return 0;
}
