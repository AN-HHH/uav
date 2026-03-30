# UAV CAN Bridge — UAVCAN v0 to CAN ESC converter

将 PX4 飞控发出的 UAVCAN v0 电调命令转换为标准 CAN ESC 帧的 STM32F4 固件。

Firmware for an STM32F4 microcontroller that receives
`uavcan.equipment.esc.RawCommand` messages from a PX4 flight controller
and re-transmits them as standard CAN ESC command frames on a second CAN bus.

---

## Architecture

```
  PX4 Flight Controller
        │  UAVCAN v0 @ 1 Mbit/s
        │  uavcan.equipment.esc.RawCommand (DTID 1030)
        ▼
  ┌─────────────┐
  │   CAN1 RX   │  STM32F4 bxCAN peripheral (extended frames, 29-bit IDs)
  │  (PD0/PD1)  │
  └──────┬──────┘
         │  libcanard reassembly + DSDL decode
         ▼
  ┌─────────────────────┐
  │    CAN Bridge       │  maps int14 throttle → uint13 output throttle
  │  (can_bridge.c)     │  clamps negative values to 0
  └──────┬──────────────┘
         │  two standard CAN frames per update
         ▼
  ┌─────────────┐
  │   CAN2 TX   │  standard 11-bit CAN frames @ 1 Mbit/s
  │  (PB5/PB6)  │
  └─────────────┘
        │
  ┌─────┴──────────────────────────────┐
  │  ESC Ring Bus (DJI/FLAMEWHEEL)     │
  │  Frame A (0x1FF): motors 1–4       │
  │  Frame B (0x2FF): motors 5–8       │
  └────────────────────────────────────┘
```

---

## Output CAN ESC Protocol

Two standard 11-bit CAN frames are sent on every received UAVCAN command:

| CAN ID  | Motors | DLC | Bytes |
|---------|--------|-----|-------|
| `0x1FF` | 1 – 4  |  8  | `[M1_H, M1_L, M2_H, M2_L, M3_H, M3_L, M4_H, M4_L]` |
| `0x2FF` | 5 – 8  |  8  | `[M5_H, M5_L, M6_H, M6_L, M7_H, M7_L, M8_H, M8_L]` |

Each motor value is a **13-bit unsigned integer (0–8191)** in big-endian order.
Frame `0x2FF` is only sent when more than 4 motors are present.

This format is compatible with DJI SNAIL / E-series / FLAMEWHEEL CAN ESCs.

### Throttle mapping

| UAVCAN RawCommand | Output throttle |
|-------------------|-----------------|
| -8192 … 0         | 0 (stopped)     |
| 1                 | 1               |
| 8191              | 8191 (full)     |

---

## File structure

```
firmware/
  Core/
    Inc/
      main.h              Application configuration (node ID, watchdog)
      bsp.h               Hardware abstraction interface
      canard.h            UAVCAN v0 library API
      uavcan_esc.h        ESC RawCommand DSDL definitions & decoder
      can_bridge.h        UAVCAN → CAN ESC bridge API
    Src/
      main.c              Application entry point & main loop
      bsp_stm32f4.c       STM32F4 HAL implementation of BSP
      canard.c            UAVCAN v0 transfer reassembly (libcanard subset)
      uavcan_esc.c        ESC RawCommand payload decoder
      can_bridge.c        Bridge logic + CAN2 output
  Makefile                Cross-compile with arm-none-eabi-gcc
  STM32F405RGTx_FLASH.ld  Linker script

tests/
  test_uavcan_esc.c       Unit tests: DSDL decoder + libcanard helpers
  test_can_bridge.c       Unit tests: bridge conversion (with BSP stub)
  Makefile                Build & run on host PC (no hardware needed)
```

---

## Building

### Prerequisites

| Tool | Version |
|------|---------|
| `arm-none-eabi-gcc` | ≥ 10 |
| STM32CubeF4 | ≥ 1.27 |
| `openocd` (optional, for flashing) | any |

### Build firmware

```bash
# Set the path to your STM32CubeF4 installation
export CUBE_PATH=$HOME/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1

cd firmware
make
```

The build produces `build/uav_can_bridge.elf` and `build/uav_can_bridge.bin`.

### Flash

```bash
cd firmware
make flash          # uses OpenOCD + ST-Link
```

### Run host-side unit tests (no hardware)

```bash
cd tests
make                # compiles and runs both test binaries
```

Expected output:
```
Results: 19 passed, 0 failed   (test_uavcan_esc)
Results:  8 passed, 0 failed   (test_can_bridge)
```

---

## Hardware

### Target MCU

STM32F405RGT6 / STM32F407VGT6 (Cortex-M4, 168 MHz)

### Pin assignment (default, see `bsp_stm32f4.c` to change)

| Signal   | Pin  | Direction |
|----------|------|-----------|
| CAN1_RX  | PD0  | Input     |
| CAN1_TX  | PD1  | Output    |
| CAN2_RX  | PB5  | Input     |
| CAN2_TX  | PB6  | Output    |

Both buses use standard 120 Ω termination resistors at each end.

### CAN bit-timing

1 Mbit/s with APB1 = 42 MHz (PCLK1):
- Prescaler = 3, BS1 = 11 tq, BS2 = 2 tq, SJW = 1 tq

---

## PX4 configuration

1. Enable UAVCAN in PX4: `UAVCAN_ENABLE = 1`
2. Set the output mode for the ESC group to UAVCAN:
   `UAVCAN_ESC_IDLT = 1` (use idle throttle if desired)
3. The bridge auto-accepts any source node ID.
4. Node ID of this device: `APP_UAVCAN_NODE_ID` in `Core/Inc/main.h` (default 10).

---

## Watchdog

If no UAVCAN ESC command is received within `APP_ESC_WATCHDOG_MS`
(default 200 ms), the bridge sends zero-throttle frames on both
`0x1FF` and `0x2FF` so all ESCs return to idle.

---

## Module responsibilities

| Module | Responsibility |
|--------|----------------|
| `canard.c` | CAN ID parsing, multi-frame transfer reassembly, CRC-16/MCRF4XX verification |
| `uavcan_esc.c` | Bit-unpacking of int14 array from UAVCAN v0 DSDL payload |
| `can_bridge.c` | Throttle mapping, output frame building, libcanard callback glue |
| `bsp_stm32f4.c` | STM32 HAL CAN init, GPIO, receive FIFO drain, transmit |
| `main.c` | Startup, main loop, watchdog |
