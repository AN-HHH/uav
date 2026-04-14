/**
 * @file test_can_bridge.c
 * @brief Unit tests for the CAN bridge logic (UAVCAN → CAN ESC frames).
 *
 * Uses a stub implementation of the BSP to capture CAN2 output
 * without any STM32 hardware.
 *
 *   make -C tests test_can_bridge && ./tests/test_can_bridge
 */
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>

/* ------------------------------------------------------------------ */
/*  BSP stub — captures CAN2 transmit calls                           */
/* ------------------------------------------------------------------ */

#include "../firmware/Core/Inc/bsp.h"
#include "../firmware/Core/Inc/canard.h"

typedef struct {
    uint16_t id;
    uint8_t  data[8];
    uint8_t  data_len;
} CapturedFrame;

static CapturedFrame g_tx_frames[32];
static int           g_tx_count = 0;

/* Stub implementations of BSP functions */
void     bsp_init(void)                                          { }
uint64_t bsp_micros(void)                                        { return 0; }
uint32_t bsp_millis(void)                                        { return 0; }
bool     bsp_can1_receive(CanardCANFrame *f, uint64_t *ts)
{
    (void)f; (void)ts;
    return false;
}

bool bsp_can2_transmit(uint16_t id, const uint8_t *data, uint8_t data_len)
{
    if (g_tx_count < 32) {
        g_tx_frames[g_tx_count].id       = id;
        g_tx_frames[g_tx_count].data_len = data_len;
        memcpy(g_tx_frames[g_tx_count].data, data, data_len);
        g_tx_count++;
    }
    return true;
}

/* ------------------------------------------------------------------ */
/*  Modules under test                                                  */
/* ------------------------------------------------------------------ */

#include "../firmware/Core/Inc/uavcan_esc.h"
#include "../firmware/Core/Src/uavcan_esc.c"

#include "../firmware/Core/Src/canard.c"

#include "../firmware/Core/Inc/can_bridge.h"
#include "../firmware/Core/Src/can_bridge.c"

/* ------------------------------------------------------------------ */
/*  Test helpers                                                        */
/* ------------------------------------------------------------------ */

static int g_passed = 0;
static int g_failed = 0;

#define TEST(name)      static void name(void)
#define RUN(name)       do { printf("  %-50s", #name); name(); } while (0)
#define PASS()          do { printf("PASS\n"); g_passed++; } while (0)
#define FAIL(msg)       do { printf("FAIL — %s\n", msg); g_failed++; } while (0)
#define CHECK(cond, msg) \
    do { if (!(cond)) { FAIL(msg); return; } } while (0)

/** Tail byte constants (mirror of canard.c internals). */
#define TAIL_SOT  (1U << 7U)
#define TAIL_EOT  (1U << 6U)

/**
 * Pack int14 array into payload bytes.
 */
static size_t packRawCommand(const int16_t *values, int n, uint8_t *out)
{
    size_t bit = 0;
    memset(out, 0, (n * 14 + 7) / 8);
    for (int i = 0; i < n; i++) {
        uint16_t v = (uint16_t)(values[i] & 0x3FFF);
        for (int b = 0; b < 14; b++) {
            if (v & (1U << b)) {
                out[bit / 8] |= (uint8_t)(1U << (bit % 8));
            }
            bit++;
        }
    }
    return (bit + 7) / 8;
}

/**
 * CRC-16/MCRF4XX (matches canard.c implementation).
 * Used by the test multi-frame encoder to produce correct CRCs.
 */
static uint16_t testCrc16Add(uint16_t crc, uint8_t b)
{
    crc ^= (uint16_t)b;
    for (int i = 0; i < 8; i++) {
        crc = (crc & 1U) ? (uint16_t)((crc >> 1U) ^ 0x8408U) : (uint16_t)(crc >> 1U);
    }
    return crc;
}

static uint16_t testCrc16WithSig(const uint8_t *data, size_t len, uint64_t sig)
{
    uint16_t crc = 0xFFFFU;
    for (int i = 0; i < 8; i++) {
        crc = testCrc16Add(crc, (uint8_t)(sig >> (i * 8U)));
    }
    for (size_t i = 0; i < len; i++) {
        crc = testCrc16Add(crc, data[i]);
    }
    return crc;
}

/**
 * Build and feed a UAVCAN RawCommand to the bridge.
 *
 * Automatically selects single-frame or multi-frame encoding:
 *   - payload <= 7 bytes: single CAN frame (SOT=1, EOT=1)
 *   - payload  > 7 bytes: multi-frame transfer with 2-byte CRC-16/MCRF4XX prefix
 *
 * UAVCAN v0 multi-frame layout
 * ───────────────────────────
 *  Frame 0  (SOT=1, EOT=0, toggle=0): [CRC_L][CRC_H][payload[0..4]] + tail
 *  Frame k  (SOT=0, EOT=0, toggle alternates starting at 1):
 *                                    [payload[5+7*(k-1)..]] + tail
 *  Last frame (SOT=0, EOT=1):        [remaining payload bytes] + tail
 */
static void feedRawCommand(const int16_t *values, int n)
{
    uint8_t payload[64];
    size_t  plen = packRawCommand(values, n, payload);

    uint32_t base_id = ((uint32_t)16U << 24U) |
                       ((uint32_t)UAVCAN_ESC_RAWCOMMAND_DTID << 8U) |
                       1U; /* src node 1 */
    uint32_t eff_id  = CANARD_CAN_FRAME_EFF | base_id;

    if (plen <= 7U) {
        /* ----- Single-frame transfer ----- */
        CanardCANFrame frame;
        frame.id = eff_id;
        memcpy(frame.data, payload, plen);
        frame.data[plen] = TAIL_SOT | TAIL_EOT; /* SOT=1 EOT=1 toggle=0 tid=0 */
        frame.data_len   = (uint8_t)(plen + 1U);
        can_bridge_feed(&frame, 0U);
    } else {
        /* ----- Multi-frame transfer ----- */
        uint16_t crc = testCrc16WithSig(payload, plen,
                                         UAVCAN_ESC_RAWCOMMAND_SIGNATURE);

        CanardCANFrame frame;
        frame.id        = eff_id;
        frame.data[0]   = (uint8_t)(crc & 0xFFU);        /* CRC low  */
        frame.data[1]   = (uint8_t)((crc >> 8U) & 0xFFU); /* CRC high */
        size_t first_payload = (plen < 5U) ? plen : 5U;
        memcpy(&frame.data[2], payload, first_payload);
        /* Tail: SOT=1, EOT=0, toggle=0, tid=0 */
        frame.data[2U + first_payload] = TAIL_SOT;
        frame.data_len = (uint8_t)(2U + first_payload + 1U);
        can_bridge_feed(&frame, 0U);

        /* Continuation frames: 7 payload bytes each */
        size_t  offset = first_payload;
        uint8_t toggle = 1U;
        while (offset < plen) {
            size_t chunk = plen - offset;
            if (chunk > 7U) { chunk = 7U; }

            frame.id = eff_id;
            memcpy(frame.data, payload + offset, chunk);
            bool is_last = (offset + chunk >= plen);
            /* Tail: SOT=0, EOT=is_last, toggle alternates */
            frame.data[chunk] = (uint8_t)((is_last ? TAIL_EOT : 0U) |
                                           (toggle ? (1U << 5U) : 0U));
            frame.data_len = (uint8_t)(chunk + 1U);
            can_bridge_feed(&frame, 0U);

            offset += chunk;
            toggle ^= 1U;
        }
    }
}

/* ------------------------------------------------------------------ */
/*  Tests                                                               */
/* ------------------------------------------------------------------ */

TEST(test_single_motor_frame_a)
{
    can_bridge_init();
    g_tx_count = 0;

    int16_t motors[1] = {4096};
    feedRawCommand(motors, 1);

    /* Frame A (0x1FF) must have been sent */
    CHECK(g_tx_count >= 1, "at least one CAN frame transmitted");
    CHECK(g_tx_frames[0].id == CAN_BRIDGE_ESC_FRAME_A_ID,
          "first frame ID = 0x1FF");
    CHECK(g_tx_frames[0].data_len == 8U, "DLC = 8");

    /* Motor 1 throttle = 4096, output = 4096 (1:1 below 8191) */
    uint16_t thr = ((uint16_t)g_tx_frames[0].data[0] << 8U) |
                    (uint16_t)g_tx_frames[0].data[1];
    CHECK(thr == 4096U, "motor 1 throttle = 4096");

    /* Motors 2–4 should be 0 */
    for (int i = 2; i < 8; i++) {
        CHECK(g_tx_frames[0].data[i] == 0, "unused motor bytes = 0");
    }
    PASS();
}

TEST(test_four_motors_frame_a_only)
{
    can_bridge_init();
    g_tx_count = 0;

    int16_t motors[4] = {100, 200, 300, 400};
    feedRawCommand(motors, 4);

    /* Only frame A should be sent (<=4 motors) */
    CHECK(g_tx_count == 1, "exactly 1 CAN frame sent for 4 motors");
    CHECK(g_tx_frames[0].id == CAN_BRIDGE_ESC_FRAME_A_ID, "ID = 0x1FF");

    uint16_t m1 = ((uint16_t)g_tx_frames[0].data[0] << 8U) | g_tx_frames[0].data[1];
    uint16_t m2 = ((uint16_t)g_tx_frames[0].data[2] << 8U) | g_tx_frames[0].data[3];
    uint16_t m3 = ((uint16_t)g_tx_frames[0].data[4] << 8U) | g_tx_frames[0].data[5];
    uint16_t m4 = ((uint16_t)g_tx_frames[0].data[6] << 8U) | g_tx_frames[0].data[7];
    CHECK(m1 == 100U, "motor 1 = 100");
    CHECK(m2 == 200U, "motor 2 = 200");
    CHECK(m3 == 300U, "motor 3 = 300");
    CHECK(m4 == 400U, "motor 4 = 400");
    PASS();
}

TEST(test_eight_motors_both_frames)
{
    can_bridge_init();
    g_tx_count = 0;

    int16_t motors[8] = {10, 20, 30, 40, 50, 60, 70, 80};
    feedRawCommand(motors, 8);

    /* Both frames A and B must be sent */
    CHECK(g_tx_count == 2, "2 CAN frames sent for 8 motors");
    CHECK(g_tx_frames[0].id == CAN_BRIDGE_ESC_FRAME_A_ID, "frame A ID = 0x1FF");
    CHECK(g_tx_frames[1].id == CAN_BRIDGE_ESC_FRAME_B_ID, "frame B ID = 0x2FF");

    /* Check frame A motors 1–4 */
    for (int i = 0; i < 4; i++) {
        uint16_t thr = ((uint16_t)g_tx_frames[0].data[i*2] << 8U) |
                        (uint16_t)g_tx_frames[0].data[i*2+1];
        CHECK(thr == (uint16_t)motors[i], "frame A motor mismatch");
    }
    /* Check frame B motors 5–8 */
    for (int i = 0; i < 4; i++) {
        uint16_t thr = ((uint16_t)g_tx_frames[1].data[i*2] << 8U) |
                        (uint16_t)g_tx_frames[1].data[i*2+1];
        CHECK(thr == (uint16_t)motors[i+4], "frame B motor mismatch");
    }
    PASS();
}

TEST(test_negative_throttle_clamped_to_zero)
{
    can_bridge_init();
    g_tx_count = 0;

    int16_t motors[4] = {-100, -1, -8192, 0};
    feedRawCommand(motors, 4);

    CHECK(g_tx_count >= 1, "frame sent");
    for (int i = 0; i < 8; i++) {
        CHECK(g_tx_frames[0].data[i] == 0,
              "negative/zero throttle must output 0");
    }
    PASS();
}

TEST(test_max_throttle)
{
    can_bridge_init();
    g_tx_count = 0;

    int16_t motors[1] = {8191};
    feedRawCommand(motors, 1);

    CHECK(g_tx_count >= 1, "frame sent");
    uint16_t thr = ((uint16_t)g_tx_frames[0].data[0] << 8U) |
                    (uint16_t)g_tx_frames[0].data[1];
    CHECK(thr == 8191U, "max throttle = 8191");
    PASS();
}

TEST(test_throttle_cache)
{
    can_bridge_init();
    g_tx_count = 0;

    int16_t motors[2] = {4096, 8191};
    feedRawCommand(motors, 2);

    /* get_throttle returns 0..1000 scale */
    uint16_t t0 = can_bridge_get_throttle(0);
    uint16_t t1 = can_bridge_get_throttle(1);
    CHECK(t0 >= 498 && t0 <= 502, "motor 0 throttle ~500");
    CHECK(t1 == 1000U, "motor 1 throttle = 1000");
    PASS();
}

TEST(test_out_of_range_channel_returns_zero)
{
    can_bridge_init();
    uint16_t t = can_bridge_get_throttle(CAN_BRIDGE_MAX_ESC_CHANNELS);
    CHECK(t == 0U, "out-of-range channel = 0");
    PASS();
}

TEST(test_non_esc_frame_ignored)
{
    can_bridge_init();
    g_tx_count = 0;

    /* Send a non-ESC UAVCAN message (DTID = 341, Node Status) */
    CanardCANFrame frame;
    uint32_t base_id = ((uint32_t)16U << 24U) |
                       ((uint32_t)341U << 8U) |
                       5U;
    frame.id = CANARD_CAN_FRAME_EFF | base_id;
    frame.data[0] = 0xDE;
    frame.data[1] = 0xAD;
    frame.data[2] = TAIL_SOT | TAIL_EOT;
    frame.data_len = 3U;

    can_bridge_feed(&frame, 0U);

    CHECK(g_tx_count == 0, "non-ESC frame must not trigger CAN2 output");
    PASS();
}

/* ------------------------------------------------------------------ */
/*  Main                                                                */
/* ------------------------------------------------------------------ */

int main(void)
{
    printf("=== CAN bridge tests ===\n");
    RUN(test_single_motor_frame_a);
    RUN(test_four_motors_frame_a_only);
    RUN(test_eight_motors_both_frames);
    RUN(test_negative_throttle_clamped_to_zero);
    RUN(test_max_throttle);
    RUN(test_throttle_cache);
    RUN(test_out_of_range_channel_returns_zero);
    RUN(test_non_esc_frame_ignored);

    printf("\nResults: %d passed, %d failed\n", g_passed, g_failed);
    return g_failed == 0 ? 0 : 1;
}
