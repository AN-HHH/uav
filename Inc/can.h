/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "adc.h"
#include "foc.h"
#include "PositionSensor.h"
#include "MotorConfig.h"
#include "usart.h"
#include "observer.h"
/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN Private defines */

/* DroneCAN ESC RawCommand 消息定义 */
/* ============ DroneCAN 29位ID格式定义 ============ */
/* 
 * DroneCAN 29位ID格式:
 * Bit 28-26: Priority (3bits)     = 0x1 (001)
 * Bit 25-21: DataType (5bits)     = 0x0E (ESC RawCommand)
 * Bit 20-13: SourceNodeID (8bits) = varies
 * Bit 12-5:  DestNodeID (8bits)   = varies
 * Bit 4-0:   Reserved (5bits)     = 0x00
 * 
 * ESC RawCommand ID 范围: 0x18EA0000 ~ 0x18EAFFFF
 */

#define DRONECAN_DATA_TYPE_ESC_RAW_CMD      0x0E    /* 数据类型: ESC RawCommand */
#define DRONECAN_PRIORITY_DEFAULT           0x01    /* 优先级: 高优先级 */
#define DRONECAN_RESERVED_BITS              0x00    /* 保留位 */

/* DroneCAN ESC RawCommand ID识别 - 检查数据类型字段 (bits 24-28) */
#define IS_DRONECAN_ESC_CMD(canid) \
    ((((canid) >> 24) & 0x1F) == (((DRONECAN_PRIORITY_DEFAULT << 4) | DRONECAN_DATA_TYPE_ESC_RAW_CMD) & 0x1F))

/* 验证是否为有效DroneCAN ESC RawCommand ID (0x18EA0000-0x18EAFFFF) */
#define DRONECAN_ESC_RAWCMD_ID_MASK         0xFFFF0000
#define DRONECAN_ESC_RAWCMD_ID_VALUE        0x18EA0000
#define IS_DRONECAN_ESC_CMD_ALT(canid) \
    (((canid) & DRONECAN_ESC_RAWCMD_ID_MASK) == DRONECAN_ESC_RAWCMD_ID_VALUE)



#define DRIVER_CLIENT_BASE_ID		0x280
#define DRIVER_SERVER_BASE_ID		0x300

#define DRIVER_CLIENT_CAN_ID		(DRIVER_CLIENT_BASE_ID + CAN_ID_NUM)
#define DRIVER_SERVER_CAN_ID		(DRIVER_SERVER_BASE_ID + CAN_ID_NUM)
#define DRIVER_BROADCAST_ID			(DRIVER_SERVER_BASE_ID + 0x40 + GROUP_NUM)

/*各标识符宏定义需与主控同�?*/
/*控制标识�?*/
#define IDENTIFIER_DRIVER_STATE				0x01
#define IDENTIFIER_CURR_KP_Q			  	0x02
#define IDENTIFIER_CURR_KI_Q			   	0x03
#define IDENTIFIER_SPD_KP					    0x04
#define IDENTIFIER_SPD_KI					    0x05
#define IDENTIFIER_POS_KP					    0x06
#define IDENTIFIER_POS_KD				    	0x07
#define IDENTIFIER_TORQUE_CTRL				0x08
#define IDENTIFIER_VEL_CTRL					  0x09
#define IDENTIFIER_POS_CTRL_ABS				0x0A
#define IDENTIFIER_POS_CTRL_REL				0x0B
#define IDENTIFIER_SET_CTRL_MODE			0x0C
#define IDENTIFIER_SET_ACC				    	0x0D
#define IDENTIFIER_SET_DEC					    0x0E
#define IDENTIFIER_SET_TORQUE_LIMIT			0x0F
#define IDENTIFIER_SET_VEL_LIMIT			  0x10
#define IDENTIFIER_SET_POS_LIMIT_UP			0x11
#define IDENTIFIER_SET_POS_LIMIT_LOW		0x12
#define IDENTIFIER_CORRECT_POS_OFFSET		0x13
#define IDENTIFIER_SET_INTEGRAL_CLEARED	0x14
#define IDENTIFIER_SET_LOAD_GAIN_1      0x17
#define IDENTIFIER_SET_LOAD_GAIN_2      0x18
#define IDENTIFIER_SET_CONTROL_MODE		  0x19
#define IDENTIFIER_SET_CLEAR_INTEGRAL		0x1A

/*读取标识�?*/	
#define IDENTIFIER_ENABLE_DONE			0x50
#define IDENTIFIER_READ_EXVEL       0x18
#define IDENTIFIER_READ_ACC				  0x19
#define IDENTIFIER_READ_TORQUE			0x20
#define IDENTIFIER_READ_VEL					0x21
#define IDENTIFIER_READ_POS					0x22
#define IDENTIFIER_READ_ENCODER_POS	0x23
#define IDENTIFIER_READ_VOL_D				0x24
#define IDENTIFIER_READ_CURR_D			0x25
#define IDENTIFIER_READ_VOL_Q				0x26
#define IDENTIFIER_READ_CURR_Q			0x27
#define IDENTIFIER_READ_SPD_LOOP_OUTPUT		0x28
#define IDENTIFIER_READ_POS_LOOP_OUTPUT		0x29
#define IDENTIFIER_READ_LOAD_OBSERVER   	0x60

/*错误标识�??*/
#define IDENTIFIER_ENCODER_ERROR		0xEE
#define IDENTIFIER_HARD_FAULT				0xFF

/*霍尔标识�?*/
#define IDENTIFIER_HALL_CATCH     0x31 

/*寻找舵轮磁铁位置指令*/
#define IDENTIFIER_HALL_TEST_ON      0x32
#define IDENTIFIER_HALL_TEST_OFF     0x33

/* DroneCAN ESC RawCommand PWM数据缓冲 */
typedef struct {
    uint16_t pwm_values[8];      /* 最多支持8个电机的PWM值 */
    uint8_t motor_count;          /* 实际电机数量 */
    uint8_t data_valid;           /* 数据有效标志 */
} DroneCAN_ESC_RawCmd_t;


typedef union
{
	uint32_t  data_uint32[2];
	int32_t   data_int32[2];
	uint8_t   data_uint8[8];
} CAN_Data_t;

 struct CAN_t
 {
     uint32_t StdID;              // ✅ 保留标准ID
     uint32_t ExtID;              // ✅ 新增：扩展帧ID
     uint8_t IDE;                 // ✅ 新增：帧类型标志 (0=标准帧, 1=扩展帧)
     uint32_t MailBox;
     uint8_t Identifier;
     int32_t ReceiveData;
     uint32_t RecieveStatus;
     int32_t TransmitData;
     CAN_Data_t Receive;
     CAN_Data_t Transmit;
 };

/* USER CODE END Private defines */


void MX_CAN1_Init(void);

/* USER CODE BEGIN Prototypes */
void CAN_Respond(void);
void CAN_Transmit(uint8_t identifier, int32_t transmitData, uint8_t length, uint32_t StdId);
void CAN_Receive(uint32_t *stdId, uint8_t *identifier, int32_t *receiveData);
void CAN_Enable(void);


/* DroneCAN消息处理函数 */

//uint8_t DRONECAN_ParseRawCommand(uint8_t *data, uint8_t dlc, DroneCAN_ESC_RawCmd_t *cmd);
//void DRONECAN_ProcessESCCommand(DroneCAN_ESC_RawCmd_t *cmd);
//uint32_t DRONECAN_GetCanID(uint8_t msg_type, uint8_t node_id);
//DRONECAN_ParseRawCommand()负责解析多字节PWM数据
//DRONECAN_ProcessESCCommand()负责执行电机命令
//DRONECAN_GetCanID()用于计算或验证DroneCAN CAN ID

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
