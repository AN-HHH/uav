/**
 * @file bsp.h
 * @brief Board Support Package — hardware abstraction interface.
 *
 * The CAN bridge application calls only these functions.  A concrete
 * implementation for STM32F4 with HAL is provided in bsp_stm32f4.c.
 * For unit-tests on a host PC a stub implementation can be provided.
 *
 * Physical mapping assumed by bsp_stm32f4.c:
 *   CAN1 — connected to the PX4 flight controller (receives UAVCAN v0)
 *   CAN2 — connected to the ESC ring bus (transmits standard CAN ESC frames)
 *
 * Both buses run at 1 Mbit/s (UAVCAN mandates 1 Mbit/s).
 */
#ifndef BSP_H
#define BSP_H

#include "canard.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/*  Initialisation                                                      */
/* ------------------------------------------------------------------ */

/**
 * Initialise all hardware peripherals (clocks, GPIO, CAN1, CAN2).
 * Must be called once at startup before any other BSP function.
 */
void bsp_init(void);

/* ------------------------------------------------------------------ */
/*  CAN receive (CAN1 — from PX4)                                      */
/* ------------------------------------------------------------------ */

/**
 * Attempt to read one CAN frame from the CAN1 receive FIFO.
 *
 * @param[out] out_frame  Populated with the received frame on success.
 * @param[out] out_ts_us  Receive timestamp in microseconds.
 * @return true  if a frame was available and written to *out_frame.
 *         false if the FIFO was empty.
 */
bool bsp_can1_receive(CanardCANFrame *out_frame, uint64_t *out_ts_us);

/* ------------------------------------------------------------------ */
/*  CAN transmit (CAN2 — to ESC ring bus)                              */
/* ------------------------------------------------------------------ */

/**
 * Transmit one CAN frame on CAN2 (standard 11-bit ID, data frame).
 *
 * Blocks until the frame is queued in a transmit mailbox or until a
 * short timeout expires.
 *
 * @param id       Standard 11-bit CAN ID (bits 10:0 used).
 * @param data     Pointer to payload bytes.
 * @param data_len Payload length in bytes (0..8).
 * @return true  on success.
 *         false if no transmit mailbox was available.
 */
bool bsp_can2_transmit(uint16_t id, const uint8_t *data, uint8_t data_len);

/* ------------------------------------------------------------------ */
/*  Timing                                                              */
/* ------------------------------------------------------------------ */

/**
 * Return the current system time in microseconds (wraps after ~71 min).
 */
uint64_t bsp_micros(void);

/**
 * Return the current system time in milliseconds.
 */
uint32_t bsp_millis(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_H */
