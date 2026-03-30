/**
 * @file test_uavcan_esc.c
 * @brief Unit tests for the UAVCAN ESC RawCommand decoder and
 *        UAVCAN CAN frame helpers.
 *
 * Compiled and run on the host PC (no STM32 hardware required).
 *
 *   make -C tests test_uavcan_esc && ./tests/test_uavcan_esc
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>

/* Pull in the modules under test */
#include "../firmware/Core/Inc/uavcan_esc.h"
#include "../firmware/Core/Src/uavcan_esc.c"   /* single-TU build */

#include "../firmware/Core/Inc/canard.h"
#include "../firmware/Core/Src/canard.c"        /* single-TU build */

/* ------------------------------------------------------------------ */
/*  Helpers                                                             */
/* ------------------------------------------------------------------ */

static int g_passed = 0;
static int g_failed = 0;

#define TEST(name)      static void name(void)
#define RUN(name)       do { printf("  %-45s", #name); name(); } while (0)
#define PASS()          do { printf("PASS\n"); g_passed++; } while (0)
#define FAIL(msg)       do { printf("FAIL — %s\n", msg); g_failed++; } while (0)
#define CHECK(cond, msg) \
    do { if (!(cond)) { FAIL(msg); return; } } while (0)

/* ------------------------------------------------------------------ */
/*  Encode helpers (used to build test payloads)                       */
/* ------------------------------------------------------------------ */

/**
 * Pack an array of int14 values into a byte buffer (UAVCAN encoding).
 * Returns the number of bytes written.
 */
static size_t packRawCommand(const int16_t *values, int n, uint8_t *out)
{
    size_t bit = 0;
    memset(out, 0, (n * 14 + 7) / 8);
    for (int i = 0; i < n; i++) {
        uint16_t v = (uint16_t)(values[i] & 0x3FFF); /* 14 bits */
        for (int b = 0; b < 14; b++) {
            if (v & (1U << b)) {
                out[bit / 8] |= (uint8_t)(1U << (bit % 8));
            }
            bit++;
        }
    }
    return (bit + 7) / 8;
}

/* ------------------------------------------------------------------ */
/*  Tests: uavcan_esc_decode_raw_command                               */
/* ------------------------------------------------------------------ */

TEST(test_decode_single_zero)
{
    int16_t  values[8];
    uint8_t  payload[4] = {0};
    /* One zero value (14 bits = 0) packed into 2 bytes (rounded up) */
    int n = uavcan_esc_decode_raw_command(payload, 2, values, 8);
    CHECK(n == 1, "expected 1 value");
    CHECK(values[0] == 0, "expected 0");
    PASS();
}

TEST(test_decode_single_max)
{
    int16_t  values[8];
    uint8_t  payload[2];
    int16_t  src[1] = {8191};
    size_t   len = packRawCommand(src, 1, payload);
    int n = uavcan_esc_decode_raw_command(payload, len, values, 8);
    CHECK(n == 1, "expected 1 value");
    CHECK(values[0] == 8191, "expected 8191");
    PASS();
}

TEST(test_decode_single_negative)
{
    int16_t  values[8];
    uint8_t  payload[4];
    int16_t  src[1] = {-1};
    size_t   len = packRawCommand(src, 1, payload);
    int n = uavcan_esc_decode_raw_command(payload, len, values, 8);
    CHECK(n >= 1, "expected at least 1 value");
    CHECK(values[0] == -1, "expected -1");
    PASS();
}

TEST(test_decode_min_raw)
{
    int16_t  values[8];
    uint8_t  payload[4];
    int16_t  src[1] = {-8192};
    size_t   len = packRawCommand(src, 1, payload);
    int n = uavcan_esc_decode_raw_command(payload, len, values, 8);
    CHECK(n >= 1, "expected at least 1 value");
    CHECK(values[0] == -8192, "expected -8192");
    PASS();
}

TEST(test_decode_four_motors)
{
    int16_t src[4]    = {0, 2048, 4096, 8191};
    int16_t values[8] = {0};
    uint8_t payload[8];
    size_t  len = packRawCommand(src, 4, payload);
    int n = uavcan_esc_decode_raw_command(payload, len, values, 8);
    CHECK(n == 4, "expected 4 values");
    CHECK(values[0] == 0,    "motor 1 = 0");
    CHECK(values[1] == 2048, "motor 2 = 2048");
    CHECK(values[2] == 4096, "motor 3 = 4096");
    CHECK(values[3] == 8191, "motor 4 = 8191");
    PASS();
}

TEST(test_decode_eight_motors)
{
    int16_t src[8]     = {100, 200, 300, 400, 500, 600, 700, 800};
    int16_t values[10] = {0};
    uint8_t payload[16];
    size_t  len = packRawCommand(src, 8, payload);
    int n = uavcan_esc_decode_raw_command(payload, len, values, 10);
    CHECK(n == 8, "expected 8 values");
    for (int i = 0; i < 8; i++) {
        CHECK(values[i] == src[i], "motor value mismatch");
    }
    PASS();
}

TEST(test_decode_null_payload)
{
    int16_t values[4];
    int n = uavcan_esc_decode_raw_command(NULL, 4, values, 4);
    CHECK(n == -1, "expected -1 for NULL payload");
    PASS();
}

TEST(test_decode_empty_payload)
{
    int16_t values[4];
    uint8_t buf[2] = {0};
    int n = uavcan_esc_decode_raw_command(buf, 0, values, 4);
    CHECK(n == -1, "expected -1 for zero-length payload");
    PASS();
}

/* ------------------------------------------------------------------ */
/*  Tests: uavcan_esc_raw_to_throttle                                  */
/* ------------------------------------------------------------------ */

TEST(test_throttle_zero)
{
    CHECK(uavcan_esc_raw_to_throttle(0) == 0, "0 -> 0");
    PASS();
}

TEST(test_throttle_max)
{
    CHECK(uavcan_esc_raw_to_throttle(8191) == 1000, "8191 -> 1000");
    PASS();
}

TEST(test_throttle_negative_clamped)
{
    CHECK(uavcan_esc_raw_to_throttle(-1) == 0, "-1 -> 0");
    CHECK(uavcan_esc_raw_to_throttle(-8192) == 0, "-8192 -> 0");
    PASS();
}

TEST(test_throttle_midpoint)
{
    /* ~4096/8191 * 1000 ≈ 500 */
    uint16_t t = uavcan_esc_raw_to_throttle(4096);
    CHECK(t >= 498 && t <= 502, "midpoint throttle ~500");
    PASS();
}

TEST(test_throttle_quarter)
{
    /* ~2048/8191 * 1000 ≈ 250 */
    uint16_t t = uavcan_esc_raw_to_throttle(2048);
    CHECK(t >= 249 && t <= 251, "quarter throttle ~250");
    PASS();
}

/* ------------------------------------------------------------------ */
/*  Tests: canard CAN ID helpers                                        */
/* ------------------------------------------------------------------ */

TEST(test_canard_is_message_transfer)
{
    /* Build a message-broadcast CAN ID for DTID=1030, src=1, prio=16 */
    uint32_t id = CANARD_CAN_FRAME_EFF |
                  ((uint32_t)16U << 24U) |
                  ((uint32_t)1030U << 8U) |
                  0U |   /* service bit = 0 */
                  1U;    /* source node ID */

    CHECK(canardIsMessageTransfer(id) == true, "should be message transfer");
    PASS();
}

TEST(test_canard_extract_dtid)
{
    uint32_t id = CANARD_CAN_FRAME_EFF |
                  ((uint32_t)16U << 24U) |
                  ((uint32_t)1030U << 8U) |
                  1U;

    CHECK(canardExtractDataTypeID(id) == 1030U, "DTID should be 1030");
    PASS();
}

TEST(test_canard_service_flag)
{
    /* Set the service flag (bit 7) */
    uint32_t id = CANARD_CAN_FRAME_EFF |
                  ((uint32_t)16U << 24U) |
                  ((uint32_t)1030U << 8U) |
                  (1U << 7U) | /* service bit = 1 */
                  1U;

    CHECK(canardIsMessageTransfer(id) == false, "service frame is not message");
    CHECK(canardExtractDataTypeID(id) == 0U, "service frame DTID = 0");
    PASS();
}

/* ------------------------------------------------------------------ */
/*  Tests: canardHandleRxFrame — single-frame transfer                 */
/* ------------------------------------------------------------------ */

static CanardRxTransfer g_last_transfer;
static int              g_cb_calls = 0;

static bool testShouldAccept(const CanardInstance *ins,
                               uint64_t *out_sig,
                               uint16_t dtid,
                               CanardTransferType ttype,
                               uint8_t src)
{
    (void)ins; (void)ttype; (void)src;
    if (dtid == UAVCAN_ESC_RAWCOMMAND_DTID) {
        *out_sig = UAVCAN_ESC_RAWCOMMAND_SIGNATURE;
        return true;
    }
    return false;
}

static void testOnTransfer(CanardInstance *ins, CanardRxTransfer *t)
{
    (void)ins;
    g_last_transfer = *t;
    /* Copy payload to a stable buffer so we can inspect it after the CB */
    static uint8_t buf[CANARD_TRANSFER_PAYLOAD_LEN_MAX];
    memcpy(buf, t->payload, t->payload_len);
    g_last_transfer.payload = buf;
    g_cb_calls++;
}

TEST(test_single_frame_transfer)
{
    CanardInstance canard;
    canardInit(&canard, testOnTransfer, testShouldAccept, NULL);
    g_cb_calls = 0;

    /* Build a single-frame UAVCAN transfer for ESC RawCommand
       with one motor value = 4000 */
    int16_t src[1] = {4000};
    uint8_t payload[4];
    size_t  plen = packRawCommand(src, 1, payload);

    CanardCANFrame frame;
    uint32_t base_id = ((uint32_t)16U << 24U) |
                       ((uint32_t)UAVCAN_ESC_RAWCOMMAND_DTID << 8U) |
                       2U; /* source node ID 2 */
    frame.id = CANARD_CAN_FRAME_EFF | base_id;

    /* Copy payload, append tail byte: SOT=1, EOT=1, toggle=0, tid=0 */
    memcpy(frame.data, payload, plen);
    frame.data[plen] = TAIL_START_OF_TRANSFER | TAIL_END_OF_TRANSFER;
    frame.data_len   = (uint8_t)(plen + 1U);

    int rc = canardHandleRxFrame(&canard, &frame, 1000U);
    CHECK(rc == 1, "expected rc=1 (transfer complete)");
    CHECK(g_cb_calls == 1, "callback called once");
    CHECK(g_last_transfer.data_type_id == UAVCAN_ESC_RAWCOMMAND_DTID,
          "DTID mismatch");
    CHECK(g_last_transfer.source_node_id == 2U, "source node ID mismatch");

    /* Verify decoded value */
    int16_t out[4];
    int n = uavcan_esc_decode_raw_command(g_last_transfer.payload,
                                           g_last_transfer.payload_len,
                                           out, 4);
    CHECK(n >= 1, "decoded at least 1 value");
    CHECK(out[0] == 4000, "decoded value = 4000");
    PASS();
}

TEST(test_rejected_unknown_dtid)
{
    CanardInstance canard;
    canardInit(&canard, testOnTransfer, testShouldAccept, NULL);
    g_cb_calls = 0;

    CanardCANFrame frame;
    /* Use DTID 999 (not ESC RawCommand) */
    uint32_t base_id = ((uint32_t)16U << 24U) |
                       ((uint32_t)999U << 8U) |
                       3U;
    frame.id = CANARD_CAN_FRAME_EFF | base_id;
    frame.data[0] = 0xAA;
    frame.data[1] = TAIL_START_OF_TRANSFER | TAIL_END_OF_TRANSFER;
    frame.data_len = 2U;

    int rc = canardHandleRxFrame(&canard, &frame, 2000U);
    CHECK(rc == 0, "unknown DTID should return 0 (silently discarded)");
    CHECK(g_cb_calls == 0, "callback not called for rejected DTID");
    PASS();
}

TEST(test_standard_frame_rejected)
{
    CanardInstance canard;
    canardInit(&canard, testOnTransfer, testShouldAccept, NULL);
    g_cb_calls = 0;

    CanardCANFrame frame;
    /* Standard (11-bit) frame — no EFF flag */
    frame.id = 0x1FFU;
    frame.data[0] = 0x55;
    frame.data[1] = TAIL_START_OF_TRANSFER | TAIL_END_OF_TRANSFER;
    frame.data_len = 2U;

    int rc = canardHandleRxFrame(&canard, &frame, 3000U);
    CHECK(rc < 0, "standard frame should be rejected (<0)");
    CHECK(g_cb_calls == 0, "callback not called");
    PASS();
}

/* ------------------------------------------------------------------ */
/*  Main                                                                */
/* ------------------------------------------------------------------ */

int main(void)
{
    printf("=== UAVCAN ESC decoder tests ===\n");
    printf("--- uavcan_esc_decode_raw_command ---\n");
    RUN(test_decode_single_zero);
    RUN(test_decode_single_max);
    RUN(test_decode_single_negative);
    RUN(test_decode_min_raw);
    RUN(test_decode_four_motors);
    RUN(test_decode_eight_motors);
    RUN(test_decode_null_payload);
    RUN(test_decode_empty_payload);

    printf("--- uavcan_esc_raw_to_throttle ---\n");
    RUN(test_throttle_zero);
    RUN(test_throttle_max);
    RUN(test_throttle_negative_clamped);
    RUN(test_throttle_midpoint);
    RUN(test_throttle_quarter);

    printf("--- canard CAN ID helpers ---\n");
    RUN(test_canard_is_message_transfer);
    RUN(test_canard_extract_dtid);
    RUN(test_canard_service_flag);

    printf("--- canardHandleRxFrame (single-frame) ---\n");
    RUN(test_single_frame_transfer);
    RUN(test_rejected_unknown_dtid);
    RUN(test_standard_frame_rejected);

    printf("\nResults: %d passed, %d failed\n", g_passed, g_failed);
    return g_failed == 0 ? 0 : 1;
}
