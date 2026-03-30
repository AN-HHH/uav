/**
 * @file uavcan_esc.h
 * @brief UAVCAN v0 ESC message definitions and decoder.
 *
 * uavcan.equipment.esc.RawCommand  (DTID = 1030 / 0x0406)
 * -------------------------------------------------------
 * Encoding: array of int14 values packed LSB-first into the payload.
 * Each value is in the range [-8192, 8191]:
 *   -8192 = full reverse (or disarmed)
 *       0 = stopped
 *    8191 = full forward (100% throttle)
 *
 * Maximum 20 values per message; PX4 typically sends up to 8.
 *
 * Data type signature: 0x217F5DE238CC0A2E
 */
#ifndef UAVCAN_ESC_H
#define UAVCAN_ESC_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/*  Data type constants                                                 */
/* ------------------------------------------------------------------ */

/** UAVCAN data type ID for uavcan.equipment.esc.RawCommand. */
#define UAVCAN_ESC_RAWCOMMAND_DTID      1030U

/** Data type signature for uavcan.equipment.esc.RawCommand. */
#define UAVCAN_ESC_RAWCOMMAND_SIGNATURE UINT64_C(0x217F5DE238CC0A2E)

/** Maximum number of ESC channels in one RawCommand message. */
#define UAVCAN_ESC_RAWCOMMAND_MAX_CH    20U

/** Minimum (full-reverse / disarmed) throttle value. */
#define UAVCAN_ESC_RAW_MIN  (-8192)

/** Maximum (full-forward) throttle value. */
#define UAVCAN_ESC_RAW_MAX  ( 8191)

/* ------------------------------------------------------------------ */
/*  Decoder                                                             */
/* ------------------------------------------------------------------ */

/**
 * Decode a uavcan.equipment.esc.RawCommand payload.
 *
 * @param[in]  payload       Raw payload bytes (no tail byte).
 * @param[in]  payload_len   Number of valid bytes in @p payload.
 * @param[out] values        Output buffer for decoded int14 values
 *                           (sign-extended to int16_t).
 * @param[in]  max_values    Capacity of @p values array.
 * @return Number of decoded ESC values (0..UAVCAN_ESC_RAWCOMMAND_MAX_CH),
 *         or -1 if the payload is invalid.
 */
int uavcan_esc_decode_raw_command(const uint8_t *payload,
                                   size_t         payload_len,
                                   int16_t       *values,
                                   size_t         max_values);

/**
 * Map a UAVCAN int14 throttle value to a normalised 0..1000 range.
 *
 * Negative UAVCAN values (reverse / disarmed) are clamped to 0.
 * Positive values are linearly scaled: 0 → 0, 8191 → 1000.
 *
 * @param raw  Signed 14-bit value from the RawCommand message.
 * @return     Throttle in range [0, 1000].
 */
uint16_t uavcan_esc_raw_to_throttle(int16_t raw);

#ifdef __cplusplus
}
#endif

#endif /* UAVCAN_ESC_H */
