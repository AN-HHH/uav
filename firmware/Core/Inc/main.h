/**
 * @file main.h
 * @brief Application-level definitions shared across source files.
 */
#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/*  Build-time configuration                                            */
/* ------------------------------------------------------------------ */

/**
 * Node ID of this device on the UAVCAN bus.
 * PX4 must be configured to broadcast ESC commands to this bus.
 * Value range: 1..127.  Change to match your network topology.
 */
#define APP_UAVCAN_NODE_ID      10U

/**
 * Watchdog period in milliseconds.
 * If no UAVCAN ESC command is received within this window the bridge
 * sends zero-throttle frames to all ESCs.
 */
#define APP_ESC_WATCHDOG_MS     200U

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H */
