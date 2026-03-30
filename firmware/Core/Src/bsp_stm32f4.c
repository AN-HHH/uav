/**
 * @file bsp_stm32f4.c
 * @brief STM32F4 HAL implementation of the BSP interface.
 *
 * Tested on STM32F405RGT6 / STM32F407VGT6.
 *
 * Pin assignment (adjust to your PCB):
 *   CAN1_RX — PD0    CAN1_TX — PD1
 *   CAN2_RX — PB5    CAN2_TX — PB6
 *
 * CAN bit-timing for 1 Mbit/s with APB1 = 42 MHz:
 *   Prescaler = 3, BS1 = 11 tq, BS2 = 2 tq, SJW = 1 tq
 *   Bit time = (1 + 11 + 2) * (1/42e6) * 3 = 1 µs  ✓
 *
 * Note: this file includes STM32 HAL headers.  You must add
 *   -I<path-to-STM32Cube_FW_F4>/Drivers/STM32F4xx_HAL_Driver/Inc
 *   -I<path-to-STM32Cube_FW_F4>/Drivers/CMSIS/Device/ST/STM32F4xx/Include
 *   -I<path-to-STM32Cube_FW_F4>/Drivers/CMSIS/Include
 * to your compiler include path and link the HAL driver sources.
 */
#include "bsp.h"

/* ---- STM32 HAL --------------------------------------------------- */
#include "stm32f4xx_hal.h"

/* ------------------------------------------------------------------ */
/*  Private handles                                                     */
/* ------------------------------------------------------------------ */

static CAN_HandleTypeDef hcan1;
static CAN_HandleTypeDef hcan2;

/* Millisecond tick provided by HAL_GetTick(). */

/* ------------------------------------------------------------------ */
/*  Internal helpers                                                    */
/* ------------------------------------------------------------------ */

/**
 * Configure one bxCAN peripheral at 1 Mbit/s.
 * APB1 clock is assumed to be 42 MHz.
 */
static HAL_StatusTypeDef initCAN(CAN_HandleTypeDef *hcan, CAN_TypeDef *instance)
{
    hcan->Instance                  = instance;
    hcan->Init.Prescaler            = 3U;
    hcan->Init.Mode                 = CAN_MODE_NORMAL;
    hcan->Init.SyncJumpWidth        = CAN_SJW_1TQ;
    hcan->Init.TimeSeg1             = CAN_BS1_11TQ;
    hcan->Init.TimeSeg2             = CAN_BS2_2TQ;
    hcan->Init.TimeTriggeredMode    = DISABLE;
    hcan->Init.AutoBusOff           = ENABLE;
    hcan->Init.AutoWakeUp           = DISABLE;
    hcan->Init.AutoRetransmission   = ENABLE;
    hcan->Init.ReceiveFifoLocked    = DISABLE;
    hcan->Init.TransmitFifoPriority = DISABLE;

    return HAL_CAN_Init(hcan);
}

/**
 * Set an accept-all receive filter on FIFO 0 of the given CAN handle.
 * Filter bank 0 is used for CAN1; filter bank 14 for CAN2 (bxCAN shared
 * filter bank split).
 */
static void configRxFilter(CAN_HandleTypeDef *hcan, uint32_t filter_bank)
{
    CAN_FilterTypeDef f;
    f.FilterBank           = filter_bank;
    f.FilterMode           = CAN_FILTERMODE_IDMASK;
    f.FilterScale          = CAN_FILTERSCALE_32BIT;
    f.FilterIdHigh         = 0x0000U;
    f.FilterIdLow          = 0x0000U;
    f.FilterMaskIdHigh     = 0x0000U;  /* mask = 0 → accept everything */
    f.FilterMaskIdLow      = 0x0000U;
    f.FilterFIFOAssignment = CAN_RX_FIFO0;
    f.FilterActivation     = ENABLE;
    f.SlaveStartFilterBank = 14U;      /* CAN2 uses banks 14..27 */
    HAL_CAN_ConfigFilter(hcan, &f);
}

/* ------------------------------------------------------------------ */
/*  HAL MSP callbacks (called internally by HAL_CAN_Init)             */
/* ------------------------------------------------------------------ */

/**
 * HAL_CAN_MspInit is a weak symbol in the HAL; we override it here to
 * configure GPIO and clocks for both CAN peripherals.
 */
void HAL_CAN_MspInit(CAN_HandleTypeDef *hcan)
{
    GPIO_InitTypeDef gpio = {0};

    if (hcan->Instance == CAN1) {
        __HAL_RCC_CAN1_CLK_ENABLE();
        __HAL_RCC_GPIOD_CLK_ENABLE();

        /* PD0 = CAN1_RX, PD1 = CAN1_TX */
        gpio.Pin       = GPIO_PIN_0 | GPIO_PIN_1;
        gpio.Mode      = GPIO_MODE_AF_PP;
        gpio.Pull      = GPIO_NOPULL;
        gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio.Alternate = GPIO_AF9_CAN1;
        HAL_GPIO_Init(GPIOD, &gpio);

    } else if (hcan->Instance == CAN2) {
        __HAL_RCC_CAN2_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        /* PB5 = CAN2_RX, PB6 = CAN2_TX */
        gpio.Pin       = GPIO_PIN_5 | GPIO_PIN_6;
        gpio.Mode      = GPIO_MODE_AF_PP;
        gpio.Pull      = GPIO_NOPULL;
        gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio.Alternate = GPIO_AF9_CAN2;
        HAL_GPIO_Init(GPIOB, &gpio);
    }
}

/* ------------------------------------------------------------------ */
/*  BSP API implementation                                              */
/* ------------------------------------------------------------------ */

void bsp_init(void)
{
    /* HAL must already be initialised (HAL_Init() + SystemClock_Config()
       called from main before bsp_init).                               */

    /* CAN1 — receive from PX4 */
    initCAN(&hcan1, CAN1);
    configRxFilter(&hcan1, 0U);
    HAL_CAN_Start(&hcan1);

    /* CAN2 — transmit to ESCs */
    initCAN(&hcan2, CAN2);
    /* CAN2 does not need an RX filter in this application, but the
       bxCAN hardware requires at least one active filter to transmit.
       Use filter bank 14 (first bank assigned to CAN2). */
    configRxFilter(&hcan2, 14U);
    HAL_CAN_Start(&hcan2);
}

bool bsp_can1_receive(CanardCANFrame *out_frame, uint64_t *out_ts_us)
{
    if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) == 0U) {
        return false;
    }

    CAN_RxHeaderTypeDef hdr;
    uint8_t data[8];
    if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &hdr, data) != HAL_OK) {
        return false;
    }

    /* We are only interested in extended (29-bit) frames */
    if (hdr.IDE != CAN_ID_EXT) {
        return false;
    }

    out_frame->id = hdr.ExtId | CANARD_CAN_FRAME_EFF;
    out_frame->data_len = (uint8_t)hdr.DLC;
    for (uint8_t i = 0; i < hdr.DLC && i < 8U; i++) {
        out_frame->data[i] = data[i];
    }
    *out_ts_us = bsp_micros();
    return true;
}

bool bsp_can2_transmit(uint16_t id, const uint8_t *data, uint8_t data_len)
{
    /* Wait for a free transmit mailbox (short busy-wait) */
    uint32_t t0 = bsp_millis();
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0U) {
        if ((bsp_millis() - t0) > 2U) {
            return false; /* Timeout */
        }
    }

    CAN_TxHeaderTypeDef hdr;
    hdr.StdId              = id & 0x7FFU;
    hdr.ExtId              = 0U;
    hdr.IDE                = CAN_ID_STD;
    hdr.RTR                = CAN_RTR_DATA;
    hdr.DLC                = data_len;
    hdr.TransmitGlobalTime = DISABLE;

    uint32_t mailbox;
    uint8_t  buf[8] = {0};
    for (uint8_t i = 0; i < data_len && i < 8U; i++) {
        buf[i] = data[i];
    }
    return HAL_CAN_AddTxMessage(&hcan2, &hdr, buf, &mailbox) == HAL_OK;
}

uint64_t bsp_micros(void)
{
    /* HAL_GetTick() returns milliseconds; multiply by 1000 for µs.
       For higher resolution use a hardware timer (e.g., TIM2 at 1 MHz). */
    return (uint64_t)HAL_GetTick() * 1000ULL;
}

uint32_t bsp_millis(void)
{
    return HAL_GetTick();
}
