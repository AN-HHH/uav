/**
 * @file canard.h
 * @brief Lightweight UAVCAN v0 stack (libcanard-compatible subset).
 *
 * Implements multi-frame transfer reassembly, CAN ID parsing, and
 * transfer dispatch for UAVCAN v0 message-broadcast transfers.
 *
 * UAVCAN v0 CAN frame ID layout (29-bit extended):
 *   bits[28:24] – priority        (5 bits, 0 = highest)
 *   bits[23:8]  – data type ID    (16 bits)
 *   bit[7]      – service flag    (0 = message broadcast, 1 = service)
 *   bits[6:0]   – source node ID  (7 bits)
 *
 * Tail byte (last byte of each CAN frame):
 *   bit[7] – start of transfer
 *   bit[6] – end of transfer
 *   bit[5] – toggle
 *   bits[4:0] – transfer ID
 */
#ifndef CANARD_H
#define CANARD_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/*  Configuration                                                       */
/* ------------------------------------------------------------------ */

/** Maximum reassembled transfer payload (bytes). */
#define CANARD_TRANSFER_PAYLOAD_LEN_MAX 256U

/** Number of simultaneous in-progress (multi-frame) RX transfers. */
#define CANARD_RX_SESSIONS_MAX 8U

/* ------------------------------------------------------------------ */
/*  Public types                                                        */
/* ------------------------------------------------------------------ */

/** Raw CAN frame as passed to/from the driver. */
typedef struct {
    uint32_t id;            /**< 29-bit extended CAN ID (CANARD_CAN_FRAME_EFF set). */
    uint8_t  data[8];
    uint8_t  data_len;
} CanardCANFrame;

/** Extended-frame flag (mirrors libcanard convention). */
#define CANARD_CAN_FRAME_EFF (1UL << 31U)
/** Remote-frame flag. */
#define CANARD_CAN_FRAME_RTR (1UL << 30U)
/** Error-frame flag. */
#define CANARD_CAN_FRAME_ERR (1UL << 29U)

/** Transfer types. */
typedef enum {
    CanardTransferTypeResponse  = 0,
    CanardTransferTypeRequest   = 1,
    CanardTransferTypeBroadcast = 2
} CanardTransferType;

/** Describes a fully-reassembled received transfer. */
typedef struct {
    uint64_t         timestamp_usec;
    const uint8_t   *payload;           /**< Points into internal buffer; valid only during callback. */
    size_t           payload_len;
    uint64_t         data_type_signature;
    uint16_t         data_type_id;
    CanardTransferType transfer_type;
    uint8_t          transfer_id;
    uint8_t          priority;
    uint8_t          source_node_id;
} CanardRxTransfer;

/** Forward declaration for callback signatures. */
typedef struct CanardInstance CanardInstance;

/**
 * Called when a complete transfer has been reassembled.
 * @p transfer is only valid for the duration of the callback.
 */
typedef void (*CanardOnTransferReceived)(CanardInstance *ins,
                                         CanardRxTransfer *transfer);

/**
 * Called before starting reassembly of an incoming transfer.
 * Return true to accept; also write the data-type signature to
 * *out_data_type_signature so the CRC can be verified.
 */
typedef bool (*CanardShouldAcceptTransfer)(const CanardInstance *ins,
                                            uint64_t *out_data_type_signature,
                                            uint16_t  data_type_id,
                                            CanardTransferType transfer_type,
                                            uint8_t   source_node_id);

/* ------------------------------------------------------------------ */
/*  Internal types (do not access directly)                            */
/* ------------------------------------------------------------------ */

/** State of one in-progress multi-frame RX session. */
typedef struct {
    uint64_t timestamp_usec;
    uint8_t  payload[CANARD_TRANSFER_PAYLOAD_LEN_MAX];
    size_t   payload_len;
    uint16_t data_type_id;
    uint64_t data_type_signature; /**< For multi-frame CRC verification. */
    uint16_t received_crc;        /**< CRC extracted from the first frame. */
    uint8_t  source_node_id;
    uint8_t  transfer_id;
    uint8_t  next_toggle;
    bool     in_use;
} CanardRxSession;

/** Instance of the UAVCAN stack. */
struct CanardInstance {
    CanardOnTransferReceived    on_transfer_received;
    CanardShouldAcceptTransfer  should_accept;
    void                       *user_reference;   /**< Not interpreted by the library. */

    /* Internal state */
    CanardRxSession rx_sessions[CANARD_RX_SESSIONS_MAX];
};

/* ------------------------------------------------------------------ */
/*  API                                                                 */
/* ------------------------------------------------------------------ */

/**
 * Initialise a CanardInstance.  Must be called before any other function.
 */
void canardInit(CanardInstance             *out_ins,
                CanardOnTransferReceived    on_reception,
                CanardShouldAcceptTransfer  should_accept,
                void                       *user_reference);

/**
 * Process one received CAN frame.
 *
 * @param ins            Library instance.
 * @param frame          Received frame (id must have CANARD_CAN_FRAME_EFF set).
 * @param timestamp_usec Microsecond timestamp of reception.
 * @return  0  – frame consumed, no complete transfer yet.
 *          1  – transfer completed and callback was invoked.
 *         <0  – error (frame rejected / malformed).
 */
int canardHandleRxFrame(CanardInstance    *ins,
                        const CanardCANFrame *frame,
                        uint64_t           timestamp_usec);

/**
 * Helper: extract the 16-bit data-type ID from a UAVCAN message-broadcast frame ID.
 * Returns 0 if the frame is a service transfer (bit 7 set).
 */
uint16_t canardExtractDataTypeID(uint32_t frame_id);

/**
 * Helper: return true if frame_id represents a message broadcast (bit 7 == 0).
 */
bool canardIsMessageTransfer(uint32_t frame_id);

#ifdef __cplusplus
}
#endif

#endif /* CANARD_H */
