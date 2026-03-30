/**
 * @file main.c
 * @brief Application entry point — UAVCAN-to-CAN ESC bridge.
 *
 * Start-up sequence
 * -----------------
 * 1. HAL_Init()  + SystemClock_Config()   (MCU HAL boilerplate)
 * 2. bsp_init()  — enables CAN1 (RX from PX4) and CAN2 (TX to ESCs)
 * 3. can_bridge_init()  — initialises libcanard
 * 4. Main loop:
 *      a. Drain CAN1 FIFO, feed frames to can_bridge_feed()
 *      b. Watchdog: if no frame received within APP_ESC_WATCHDOG_MS,
 *         output zero-throttle frames so ESCs go idle
 *
 * The bridge converts uavcan.equipment.esc.RawCommand (DTID 1030)
 * received from the PX4 flight controller into DJI/FLAMEWHEEL
 * CAN ESC command frames (IDs 0x1FF and 0x2FF) on the output bus.
 */
#include "main.h"
#include "bsp.h"
#include "can_bridge.h"

/* ------------------------------------------------------------------ */
/*  SystemClock_Config                                                  */
/* ------------------------------------------------------------------ */

/**
 * Configure the system clock for STM32F405/407:
 *   HSE = 8 MHz crystal → PLL → SYSCLK = 168 MHz
 *   AHB = 168 MHz, APB1 = 42 MHz, APB2 = 84 MHz
 *
 * Adjust HSE_VALUE and PLL parameters to match your board.
 */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState       = RCC_HSE_ON;
    osc.PLL.PLLState   = RCC_PLL_ON;
    osc.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM       = 8U;   /* HSE / 8  = 1 MHz PLL input */
    osc.PLL.PLLN       = 336U; /* 1 × 336 = 336 MHz VCO     */
    osc.PLL.PLLP       = RCC_PLLP_DIV2; /* 336 / 2 = 168 MHz SYSCLK */
    osc.PLL.PLLQ       = 7U;   /* 336 / 7 = 48 MHz USB       */
    HAL_RCC_OscConfig(&osc);

    RCC_ClkInitTypeDef clk = {0};
    clk.ClockType      = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                         RCC_CLOCKTYPE_PCLK1  | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV4;  /* 168/4 = 42 MHz */
    clk.APB2CLKDivider = RCC_HCLK_DIV2;  /* 168/2 = 84 MHz */
    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5);
}

/* ------------------------------------------------------------------ */
/*  Entry point                                                         */
/* ------------------------------------------------------------------ */

int main(void)
{
    /* --- MCU and HAL initialisation -------------------------------- */
    HAL_Init();
    SystemClock_Config();

    /* --- Peripheral initialisation --------------------------------- */
    bsp_init();

    /* --- Application initialisation -------------------------------- */
    can_bridge_init();

    /* Timestamp of the last successfully received ESC command */
    uint32_t last_cmd_ms = bsp_millis();

    /* ----------------------------------------------------------------
     * Main loop
     * ---------------------------------------------------------------- */
    while (1) {
        CanardCANFrame frame;
        uint64_t ts_us;

        /* Drain all pending CAN1 frames this iteration */
        while (bsp_can1_receive(&frame, &ts_us)) {
            can_bridge_feed(&frame, ts_us);
            last_cmd_ms = bsp_millis();
        }

        /* Watchdog: if PX4 stops sending, zero all ESC throttles */
        if ((bsp_millis() - last_cmd_ms) > APP_ESC_WATCHDOG_MS) {
            static const uint8_t zero8[8] = {0};
            bsp_can2_transmit(CAN_BRIDGE_ESC_FRAME_A_ID, zero8, 8U);
            bsp_can2_transmit(CAN_BRIDGE_ESC_FRAME_B_ID, zero8, 8U);
            last_cmd_ms = bsp_millis(); /* Reset to avoid flooding */
        }
    }

    /* Never reached */
    return 0;
}
