/**
 * @file can_bridge.h
 * @brief UAVCAN → CAN ESC bridge interface.
 *
 * This module sits between the UAVCAN receiver (libcanard) and the
 * CAN2 output driver.  It translates the UAVCAN v0
 * uavcan.equipment.esc.RawCommand message into two standard CAN frames
 * compatible with the DJI/FLAMEWHEEL ESC protocol:
 *
 *   Frame A — motors 1–4:  CAN ID 0x1FF  (11-bit standard)
 *   Frame B — motors 5–8:  CAN ID 0x2FF  (11-bit standard)
 *
 * Payload encoding (big-endian, 2 bytes per motor, 0..8191 throttle):
 *   [M_H, M_L, ...]  where M = uint13 throttle value
 *
 * Negative UAVCAN throttle values (reverse / disarmed) are mapped to 0.
 */
#ifndef CAN_BRIDGE_H
#define CAN_BRIDGE_H

#include "canard.h"
#include "bsp.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/*  Configuration                                                       */
/* ------------------------------------------------------------------ */

/** Number of ESC output channels supported. */
#define CAN_BRIDGE_MAX_ESC_CHANNELS  8U

/** CAN IDs for the two ESC command frames. */
#define CAN_BRIDGE_ESC_FRAME_A_ID    0x1FFU   /**< Motors 1–4 */
#define CAN_BRIDGE_ESC_FRAME_B_ID    0x2FFU   /**< Motors 5–8 */

/** Maximum raw throttle value sent on the output bus. */
#define CAN_BRIDGE_ESC_MAX_RAW       8191U

/* ------------------------------------------------------------------ */
/*  Public API                                                          */
/* ------------------------------------------------------------------ */

/**
 * Initialise the bridge.
 *
 * Sets up the libcanard instance to accept only
 * uavcan.equipment.esc.RawCommand transfers.
 */
void can_bridge_init(void);

/**
 * Feed one raw CAN frame received on CAN1 into the bridge.
 *
 * Internally calls canardHandleRxFrame(); if a complete
 * uavcan.equipment.esc.RawCommand transfer is assembled the ESC
 * throttle values are decoded and forwarded via bsp_can2_transmit().
 *
 * @param frame        Frame received from CAN1.
 * @param timestamp_us Reception timestamp (microseconds).
 */
void can_bridge_feed(const CanardCANFrame *frame, uint64_t timestamp_us);

/**
 * Return the last decoded throttle value for channel @p ch.
 *
 * @param ch  Channel index 0..CAN_BRIDGE_MAX_ESC_CHANNELS-1.
 * @return    Throttle in range [0, 1000], or 0 if not yet received.
 */
uint16_t can_bridge_get_throttle(uint8_t ch);

#ifdef __cplusplus
}
#endif

#endif /* CAN_BRIDGE_H */
