/**
 * @file uavcan_esc.c
 * @brief UAVCAN v0 ESC RawCommand decoder.
 */
#include "uavcan_esc.h"

/**
 * Extract a 14-bit signed integer from a packed bit-stream.
 *
 * UAVCAN packs values LSB-first; each int14 occupies exactly 14 bits.
 */
static int16_t extractInt14(const uint8_t *data, size_t bit_offset)
{
    uint16_t value = 0U;
    for (size_t b = 0U; b < 14U; b++) {
        size_t  byte_idx = (bit_offset + b) / 8U;
        size_t  bit_idx  = (bit_offset + b) % 8U;
        if (data[byte_idx] & (uint8_t)(1U << bit_idx)) {
            value |= (uint16_t)(1U << b);
        }
    }
    /* Sign-extend from 14 bits to 16 bits */
    if (value & 0x2000U) {
        value |= 0xC000U;
    }
    return (int16_t)value;
}

int uavcan_esc_decode_raw_command(const uint8_t *payload,
                                   size_t         payload_len,
                                   int16_t       *values,
                                   size_t         max_values)
{
    if (payload == NULL || values == NULL || payload_len == 0U) {
        return -1;
    }

    size_t bit_capacity = payload_len * 8U;
    size_t count        = 0U;
    size_t bit_offset   = 0U;

    while ((bit_offset + 14U) <= bit_capacity &&
           count < max_values &&
           count < UAVCAN_ESC_RAWCOMMAND_MAX_CH) {
        values[count] = extractInt14(payload, bit_offset);
        count++;
        bit_offset += 14U;
    }

    return (int)count;
}

uint16_t uavcan_esc_raw_to_throttle(int16_t raw)
{
    if (raw <= 0) {
        return 0U;
    }
    if (raw >= UAVCAN_ESC_RAW_MAX) {
        return 1000U;
    }
    /* Linear mapping: [0, 8191] -> [0, 1000] */
    return (uint16_t)(((uint32_t)(uint16_t)raw * 1000UL) / (uint32_t)UAVCAN_ESC_RAW_MAX);
}
