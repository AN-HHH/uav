/**
 * @file can_bridge.c
 * @brief UAVCAN → CAN ESC bridge implementation.
 *
 * Converts uavcan.equipment.esc.RawCommand (DTID 1030) received from
 * the PX4 flight controller into standard CAN ESC command frames
 * (DJI/FLAMEWHEEL protocol) and forwards them on the ESC ring bus.
 *
 * Output frame encoding
 * ─────────────────────
 * Motors 1–4:  CAN ID 0x1FF, DLC = 8
 *   Byte 0–1: motor 1 throttle (uint13, big-endian, bits 12:0)
 *   Byte 2–3: motor 2 throttle
 *   Byte 4–5: motor 3 throttle
 *   Byte 6–7: motor 4 throttle
 *
 * Motors 5–8:  CAN ID 0x2FF, DLC = 8  (same layout)
 *
 * The throttle value sent over the output bus is in range [0, 8191].
 * Negative UAVCAN RawCommand values are clamped to 0 (disarmed / stopped).
 */
#include "can_bridge.h"
#include "uavcan_esc.h"
#include <string.h>

/* ------------------------------------------------------------------ */
/*  Private state                                                       */
/* ------------------------------------------------------------------ */

static CanardInstance g_canard;

/** Last known throttle values (0..1000) per channel. */
static uint16_t g_throttle[CAN_BRIDGE_MAX_ESC_CHANNELS];

/* ------------------------------------------------------------------ */
/*  Internal helpers                                                    */
/* ------------------------------------------------------------------ */

/**
 * Map a UAVCAN int14 raw value to a 13-bit output throttle [0, 8191].
 * Negative values are clamped to 0.
 */
static uint16_t rawToOutputThrottle(int16_t raw)
{
    if (raw <= 0) {
        return 0U;
    }
    /* UAVCAN max = 8191, output max = 8191: 1:1 mapping */
    if (raw > (int16_t)CAN_BRIDGE_ESC_MAX_RAW) {
        return (uint16_t)CAN_BRIDGE_ESC_MAX_RAW;
    }
    return (uint16_t)raw;
}

/**
 * Build and transmit the two CAN ESC command frames (0x1FF and 0x2FF).
 *
 * @param values     Decoded int14 throttle values from UAVCAN.
 * @param num_values Number of valid values (1..8).
 */
static void sendEscFrames(const int16_t *values, int num_values)
{
    uint8_t frame_a[8] = {0};  /* Motors 1–4  (CAN ID 0x1FF) */
    uint8_t frame_b[8] = {0};  /* Motors 5–8  (CAN ID 0x2FF) */

    for (int i = 0; i < num_values && i < (int)CAN_BRIDGE_MAX_ESC_CHANNELS; i++) {
        uint16_t thr = rawToOutputThrottle(values[i]);

        /* Update the publicly readable throttle cache (0..1000 scale) */
        g_throttle[i] = uavcan_esc_raw_to_throttle(values[i]);

        if (i < 4) {
            /* Motors 1–4 in frame_a */
            int slot = i * 2;
            frame_a[slot]     = (uint8_t)((thr >> 8U) & 0xFFU);
            frame_a[slot + 1] = (uint8_t)( thr        & 0xFFU);
        } else {
            /* Motors 5–8 in frame_b */
            int slot = (i - 4) * 2;
            frame_b[slot]     = (uint8_t)((thr >> 8U) & 0xFFU);
            frame_b[slot + 1] = (uint8_t)( thr        & 0xFFU);
        }
    }

    /* Always send frame_a; only send frame_b if we have > 4 motors */
    bsp_can2_transmit(CAN_BRIDGE_ESC_FRAME_A_ID, frame_a, 8U);
    if (num_values > 4) {
        bsp_can2_transmit(CAN_BRIDGE_ESC_FRAME_B_ID, frame_b, 8U);
    }
}

/* ------------------------------------------------------------------ */
/*  libcanard callbacks                                                 */
/* ------------------------------------------------------------------ */

static bool shouldAccept(const CanardInstance *ins,
                          uint64_t             *out_data_type_signature,
                          uint16_t              data_type_id,
                          CanardTransferType    transfer_type,
                          uint8_t               source_node_id)
{
    (void)ins;
    (void)transfer_type;
    (void)source_node_id;

    if (data_type_id == UAVCAN_ESC_RAWCOMMAND_DTID) {
        *out_data_type_signature = UAVCAN_ESC_RAWCOMMAND_SIGNATURE;
        return true;
    }
    return false;
}

static void onTransferReceived(CanardInstance   *ins,
                                CanardRxTransfer *transfer)
{
    (void)ins;

    if (transfer->data_type_id != UAVCAN_ESC_RAWCOMMAND_DTID) {
        return;
    }

    int16_t values[UAVCAN_ESC_RAWCOMMAND_MAX_CH];
    int     count = uavcan_esc_decode_raw_command(transfer->payload,
                                                   transfer->payload_len,
                                                   values,
                                                   UAVCAN_ESC_RAWCOMMAND_MAX_CH);
    if (count <= 0) {
        return;
    }

    /* Clamp to the supported output channels */
    if (count > (int)CAN_BRIDGE_MAX_ESC_CHANNELS) {
        count = (int)CAN_BRIDGE_MAX_ESC_CHANNELS;
    }

    sendEscFrames(values, count);
}

/* ------------------------------------------------------------------ */
/*  Public API                                                          */
/* ------------------------------------------------------------------ */

void can_bridge_init(void)
{
    memset(g_throttle, 0, sizeof(g_throttle));
    canardInit(&g_canard, onTransferReceived, shouldAccept, NULL);
}

void can_bridge_feed(const CanardCANFrame *frame, uint64_t timestamp_us)
{
    canardHandleRxFrame(&g_canard, frame, timestamp_us);
}

uint16_t can_bridge_get_throttle(uint8_t ch)
{
    if (ch >= CAN_BRIDGE_MAX_ESC_CHANNELS) {
        return 0U;
    }
    return g_throttle[ch];
}
