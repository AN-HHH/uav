/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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

/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
#ifndef CAN_ID_NUM
 #error "CAN_ID_NUMBER is not undefined."
#endif

CAN_TxHeaderTypeDef TxMessage = { 0 };
CAN_RxHeaderTypeDef RxMessage0 = { 0 };

struct CAN_t CAN;
extern struct Driver_t Driver;
extern struct CurrLoop_t CurrLoop;
extern struct SpdLoop_t SpdLoop;
extern struct PosLoop_t PosLoop;
extern struct TorqueCtrl_t TorqueCtrl;
extern struct MainCtrl_t MainCtrl;
extern struct CoordTrans_t CoordTrans;
extern struct PosSensor_t PosSensor;
extern struct LoadObserver_t LoadOb; 

/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 5;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_4TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 1, 1);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 1, 1);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/*
 * Extract a signed 14-bit (int14) value from a UAVCAN DSDL bit-packed payload.
 * Values are packed LSB-first with no padding between elements.
 * @param data  Pointer to the raw CAN frame payload bytes
 * @param index Zero-based index of the int14 value to extract
 * @return      Sign-extended 16-bit representation of the int14 value
 */
static int16_t DroneCAN_ExtractInt14(const uint8_t *data, uint8_t index)
{
	uint32_t bit_offset = (uint32_t)index * 14u;
	uint32_t byte_idx   = bit_offset >> 3u;
	uint32_t bit_shift  = bit_offset & 7u;
	uint32_t raw;

	raw = (uint32_t)data[byte_idx] | ((uint32_t)data[byte_idx + 1u] << 8u);
	if (bit_shift > 2u)
	{
		raw |= (uint32_t)data[byte_idx + 2u] << 16u;
	}

	raw >>= bit_shift;
	raw &= 0x3FFFu;

	if ((raw & 0x2000u) != 0u) /* sign-extend int14 to int16 */
	{
		raw |= 0xFFFFC000u;
	}

	return (int16_t)raw;
}

/*
 * Process a DroneCAN ESC RawCommand frame for this ESC node.
 * Extracts the int14 command at DRONECAN_ESC_INDEX and applies it
 * to the active control loop (torque or speed).
 * Sets CAN.RecieveStatus to trigger velocity feedback via CAN_Respond().
 */
static void DroneCAN_ESC_ProcessRawCommand(void)
{
	if (RxMessage0.DLC < 2u)
	{
		return;
	}

	/* Payload excludes the UAVCAN transfer tail byte at the end */
	uint8_t num_cmds = (uint8_t)(((uint16_t)(RxMessage0.DLC - 1u) * 8u) / 14u);
	if (DRONECAN_ESC_INDEX >= num_cmds)
	{
		return;
	}

	int16_t raw_cmd = DroneCAN_ExtractInt14((uint8_t *)&CAN.Receive, DRONECAN_ESC_INDEX);

	if (Driver.ControlMode == TORQUE_CTRL_MODE)
	{
		TorqueCtrl.ExptTorque_Nm = (float)raw_cmd * TorqueCtrl.MaxTorque_Nm
		                           / (float)DRONECAN_ESC_INT14_MAX;
		Saturation_float(&TorqueCtrl.ExptTorque_Nm,
		                  TorqueCtrl.MaxTorque_Nm, -TorqueCtrl.MaxTorque_Nm);
	}
	else
	{
		SpdLoop.ExptMecAngularSpeed_rad = (float)raw_cmd
		                                  * SpdLoop.MaxExptMecAngularSpeed_rad
		                                  / (float)DRONECAN_ESC_INT14_MAX;
		Saturation_float(&SpdLoop.ExptMecAngularSpeed_rad,
		                  SpdLoop.MaxExptMecAngularSpeed_rad,
		                  -SpdLoop.MaxExptMecAngularSpeed_rad);
	}

	/* Trigger velocity status feedback via CAN_Respond() */
	CAN.RecieveStatus = (0x40u + IDENTIFIER_READ_VEL);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{	
	first_line:
	
	CAN_Receive(&CAN.StdID, &CAN.Identifier, &CAN.ReceiveData);
	
	/*DroneCAN ESC RawCommand (extended 29-bit ID frame)*/
	if (RxMessage0.IDE == CAN_ID_EXT &&
	    (RxMessage0.ExtId & DRONECAN_MSG_DTID_MASK) == DRONECAN_ESC_RAW_CMD_FILTER)
	{
		DroneCAN_ESC_ProcessRawCommand();
	}
	/*ACTION指令*/
	/*点对点模�?*/
	else if (CAN.StdID == DRIVER_SERVER_CAN_ID)
	{
		switch(CAN.Identifier)
		{
			case IDENTIFIER_DRIVER_STATE:

				if (CAN.ReceiveData == 0x00000001)
				{
					/*PWM输出使能*/
					SpdLoop.ExptMecAngularSpeed_rad = 0.f;
					CurrLoop.ErrD = 0;
					CurrLoop.ErrQ = 0;
					CurrLoop.ExptCurrD = 0;
					CurrLoop.ExptCurrQ = 0;
					CurrLoop.IntegralErrD = 0;
					CurrLoop.IntegralErrQ = 0;
					SpdLoop.Err = 0;
					SpdLoop.IntegralErr = 0;
					PWM_CMD(ENABLE);
					
					CAN.Identifier = IDENTIFIER_ENABLE_DONE;
					CAN.TransmitData = 0x01;
					CAN_Transmit(CAN.Identifier, CAN.TransmitData, 4, DRIVER_CLIENT_CAN_ID);
					
				}
				else if(CAN.ReceiveData == 0x00000000)
				{
					/*PWM输出失能*/
					SpdLoop.ExptMecAngularSpeed_rad = 0.f;
					PWM_CMD(DISABLE);
				}
				
				break;
	
			case IDENTIFIER_CURR_KP_Q:
				
				/*设置q轴Kp, 主控乘以1000后发给驱动器*/
				CurrLoop.Kp_Q = CAN.ReceiveData * 1e-3;
			
				break;
			
			case IDENTIFIER_CURR_KI_Q:
				
				/*设置q轴Ki, 主控乘以1000后发给驱动器*/
				CurrLoop.Ki_Q = CAN.ReceiveData * 1e-3;
			
				break;
			
			case IDENTIFIER_SPD_KP:
				
				/*设置速度环Kp, 主控乘以1000后发给驱动器*/
				SpdLoop.Kp = CAN.ReceiveData * 1e-3;
			
				break;
			
			case IDENTIFIER_SPD_KI:
			
				/*设置速度环Ki, 主控乘以1000后发给驱动器*/			
				SpdLoop.Ki = CAN.ReceiveData * 1e-3;
			
				break;
			
			case IDENTIFIER_POS_KP:
				
				/*设置位置环Kp, 主控乘以1000后发给驱动器*/
				PosLoop.Kp = CAN.ReceiveData * 1e-3;
			
				break;
			
			case IDENTIFIER_POS_KD:

				/*设置位置环Kd, 主控乘以1000后发给驱动器*/
				PosLoop.Kd = CAN.ReceiveData * 1e-3;
			
				break;
			
			case IDENTIFIER_TORQUE_CTRL:
				
				/*转矩控制模式, 期望转矩, 主控以mN*M为单位发给驱动器*/
				TorqueCtrl.ExptTorque_Nm =  (float)CAN.ReceiveData * 1e-3;
			
				break;
				
			case IDENTIFIER_VEL_CTRL:
				
				/*速度控制模式, 期望机械角�?�度*/
				MainCtrl.ExptMecAngularSpeed_pulse =  MC_PULSE_TO_DRV_PULSE(CAN.ReceiveData);
				SpdLoop.ExptMecAngularSpeed_rad = DRV_PULSE_TO_RAD(MainCtrl.ExptMecAngularSpeed_pulse);
			
				break;
			
			case IDENTIFIER_POS_CTRL_ABS:
			
				/*位置控制模式, 绝对位置模式, 期望机械角度*/
				MainCtrl.ExptMecAngle_pulse = MC_PULSE_TO_DRV_PULSE(CAN.ReceiveData);
			
				break;

			case IDENTIFIER_POS_CTRL_REL:
				
				/*位置控制模式, 相对位置模式, 期望机械角度*/
				MainCtrl.ExptMecAngle_pulse = MC_PULSE_TO_DRV_PULSE(CAN.ReceiveData);
			
				break;
						
			case IDENTIFIER_SET_CTRL_MODE:
				
				/*设置控制模式*/
				if(CAN.ReceiveData == SPD_CURR_CTRL_MODE)
				{
					Driver.ControlMode = SPD_CURR_CTRL_MODE;
				}
				else if(CAN.ReceiveData == POS_SPD_CURR_CTRL_MODE)
				{
					Driver.ControlMode = POS_SPD_CURR_CTRL_MODE;
				}
				else if(CAN.ReceiveData == POS_CURR_CTRL_MODE)
				{
					Driver.ControlMode = POS_SPD_CURR_CTRL_MODE;
				}
				else if(CAN.ReceiveData == TORQUE_CTRL_MODE)
				{
					Driver.ControlMode = TORQUE_CTRL_MODE;
				}
				
				DriverCtrlModeInit();
				
				break;

			case IDENTIFIER_SET_ACC:
				
				/*设置加�?�度*/
				MainCtrl.Acceleration_pulse = MC_PULSE_TO_DRV_PULSE(CAN.ReceiveData);
				SpdLoop.Acceleration = DRV_PULSE_TO_RAD(MainCtrl.Acceleration_pulse);
				
				break;

			case IDENTIFIER_SET_DEC:

				/*设置减�?�度*/
				MainCtrl.Deceleration_pulse = MC_PULSE_TO_DRV_PULSE(CAN.ReceiveData);
				SpdLoop.Deceleration = DRV_PULSE_TO_RAD(MainCtrl.Deceleration_pulse);

				break;
					
			case IDENTIFIER_SET_TORQUE_LIMIT:
				
				/*设置转矩限幅*/
				TorqueCtrl.MaxTorque_Nm = (float)CAN.ReceiveData * 1e-3;
				CurrLoop.LimitCurrQ = TorqueCtrl.MaxTorque_Nm / (1.5f * MOTOR_POLE_PAIRS_NUM * ROTATOR_FLUX_LINKAGE);
			
				break;
			case IDENTIFIER_SET_VEL_LIMIT:
				
				/*设置速度限幅*/
				MainCtrl.MaxMecAngularSpeed_pulse =  MC_PULSE_TO_DRV_PULSE(CAN.ReceiveData);
			
				/*速度控制模式和位置控制模式下速度限幅*/
				if(Driver.ControlMode == SPD_CURR_CTRL_MODE || Driver.ControlMode == POS_SPD_CURR_CTRL_MODE)
				{
					SpdLoop.MaxExptMecAngularSpeed_rad = DRV_PULSE_TO_RAD(MainCtrl.MaxMecAngularSpeed_pulse);
					Saturation_float(&SpdLoop.MaxExptMecAngularSpeed_rad,  MAX_SPD(PosSensor.Generatrix_Vol), - MAX_SPD(PosSensor.Generatrix_Vol));
				}
				/*转矩控制模式下�?�度限幅*/
				else if(Driver.ControlMode == TORQUE_CTRL_MODE)
				{
					TorqueCtrl.MaxMecSpd_rad = DRV_PULSE_TO_RAD(MainCtrl.MaxMecAngularSpeed_pulse);
					Saturation_float(&TorqueCtrl.MaxMecSpd_rad, MAX_SPD(PosSensor.Generatrix_Vol), - MAX_SPD(PosSensor.Generatrix_Vol));
				}
				
				break;
			
			case IDENTIFIER_SET_POS_LIMIT_UP:
				
				/*设置机械角度上限*/
				MainCtrl.MecAngleUpperLimit_pulse =  MC_PULSE_TO_DRV_PULSE(CAN.ReceiveData);
				PosLoop.MecAngleUpperLimit_rad = (float)DRV_PULSE_TO_RAD(MainCtrl.MecAngleUpperLimit_pulse);
			
				break;
			
			case IDENTIFIER_SET_POS_LIMIT_LOW:
				
				/*设置机械角度下限*/
				MainCtrl.MecAngleLowerLimit_pulse =  MC_PULSE_TO_DRV_PULSE(CAN.ReceiveData);
				PosLoop.MecAngleLowerLimit_rad = (float)DRV_PULSE_TO_RAD(MainCtrl.MecAngleLowerLimit_pulse);
			
				break;

			case IDENTIFIER_CORRECT_POS_OFFSET:
				
				/*失能ADC, 停止进入ADC中断*/
				ADC_CMD(DISABLE);

				/*矫正编码器偏移量, 在中断中执行以阻塞程�?*/
				Driver.UnitMode = CORRECT_POS_OFFSET_MODE;
				CorrectPosOffset_Encoder(0.8f);
			
			    PWM_CMD(ENABLE);
				/*进入工作模式*/
				Driver.UnitMode = WORK_MODE;
				ADC_CMD(ENABLE);
			
				break;
			
			case IDENTIFIER_HALL_TEST_ON:
					
				EXTI ->IMR = 0x00000008;
				
				break;
			
			/*关闭霍尔外部中断*/
			case IDENTIFIER_HALL_TEST_OFF:
					
				EXTI ->IMR = 0x00000000;
				break;
			
		  case IDENTIFIER_SET_CLEAR_INTEGRAL:
				
				SpdLoop.IntegralErr = 0.f;
				
				break;
			
			case (0x40 + IDENTIFIER_READ_TORQUE):
				
				/*读取电磁转矩*/
				CAN.RecieveStatus = (0x40 + IDENTIFIER_READ_TORQUE);
				
				break;
						
			case (0x40 + IDENTIFIER_READ_VEL):
				
				/*读取机械角�?�度*/
				CAN.RecieveStatus = (0x40 + IDENTIFIER_READ_VEL);
			
				break;

			case (0x40 + IDENTIFIER_READ_POS):
				
				/*读取绝对机械角度*/
				CAN.RecieveStatus = (0x40 + IDENTIFIER_READ_POS);
		
				break; 
			
			case (0x40 + IDENTIFIER_READ_ENCODER_POS):
				
				/*读取编码�?*/
				CAN.RecieveStatus = (0x40 + IDENTIFIER_READ_ENCODER_POS);
		
				break; 
						
			case (0x40 + IDENTIFIER_READ_VOL_D):
				
				/*读取Vd*/
				CAN.RecieveStatus = (0x40 + IDENTIFIER_READ_VOL_D);
			
				break;
			
			case (0x40 + IDENTIFIER_READ_CURR_D):
				
				/*读取Id*/
				CAN.RecieveStatus = (0x40 + IDENTIFIER_READ_CURR_D);
				
				break;
			
			case (0x40 + IDENTIFIER_READ_VOL_Q):
				
				/*读取Vq*/
				CAN.RecieveStatus = (0x40 + IDENTIFIER_READ_VOL_Q);
			
				break;
			
			case (0x40 + IDENTIFIER_READ_CURR_Q):
				
				/*读取Iq*/
				CAN.RecieveStatus = (0x40 + IDENTIFIER_READ_CURR_Q);
				
				break;
			
			case (0x40 + IDENTIFIER_READ_SPD_LOOP_OUTPUT):
				
				/*读取速度环输�?*/
				CAN.RecieveStatus = (0x40 + IDENTIFIER_READ_SPD_LOOP_OUTPUT);
			
				break;
			
			case (0x40 + IDENTIFIER_READ_POS_LOOP_OUTPUT):
				
				/*读取位置环输�?*/
				CAN.RecieveStatus = (0x40 + IDENTIFIER_READ_POS_LOOP_OUTPUT);
				
				break;
			case (0x40 + IDENTIFIER_READ_LOAD_OBSERVER):
				
				/*读取位置环输�?*/
				CAN.RecieveStatus = (0x40 + IDENTIFIER_READ_LOAD_OBSERVER);
				
				break;		
	
			case (0x40 + IDENTIFIER_READ_ACC):
				
				/*读取加速度*/
				CAN.RecieveStatus = (0x40 + IDENTIFIER_READ_ACC);
			

						
			default:
				
				break;
		}	
	}
	/*广播模式*/
	else if(CAN.StdID == DRIVER_BROADCAST_ID)
	{
		switch(CAN.Identifier)
		{
			case IDENTIFIER_DRIVER_STATE:
				
				if (CAN.ReceiveData == 0x00000001)
				{
					/*PWM输出使能*/
					SpdLoop.ExptMecAngularSpeed_rad = 0.f;
					PWM_CMD(ENABLE);
				}
				else if(CAN.ReceiveData == 0x00000000)
				{
					/*PWM输出失能*/
					SpdLoop.ExptMecAngularSpeed_rad = 0.f;
					PWM_CMD(DISABLE);
				}
				
				break;
	
			case IDENTIFIER_CURR_KP_Q:
				
				/*设置q轴Kp, 主控乘以1000后发给驱动器*/
				CurrLoop.Kp_Q = CAN.ReceiveData * 1e-3;
			
				break;
			
			case IDENTIFIER_CURR_KI_Q:
				
				/*设置q轴Ki, 主控乘以1000后发给驱动器*/
				CurrLoop.Ki_Q = CAN.ReceiveData * 1e-3;
			
				break;
			
			case IDENTIFIER_SPD_KP:
				
				/*设置速度环Kp, 主控乘以1000后发给驱动器*/
				SpdLoop.Kp = CAN.ReceiveData * 1e-3;
			
				break;
			
			case IDENTIFIER_SPD_KI:
			
				/*设置速度环Ki, 主控乘以1000后发给驱动器*/			
				SpdLoop.Ki = CAN.ReceiveData * 1e-3;
			
				break;
			
			case IDENTIFIER_POS_KP:
				
				/*设置位置环Kp, 主控乘以1000后发给驱动器*/
				PosLoop.Kp = CAN.ReceiveData * 1e-3;
			
				break;
			
			case IDENTIFIER_POS_KD:

				/*设置位置环Kd, 主控乘以1000后发给驱动器*/
				PosLoop.Kd = CAN.ReceiveData * 1e-3;
			
				break;
			
			case IDENTIFIER_TORQUE_CTRL:
				
				/*转矩控制模式, 期望转矩, 主控以毫牛米为单位发给驱动器*/
				TorqueCtrl.ExptTorque_Nm =  (float)CAN.ReceiveData * 1e-3;
			
				break;
				
			case IDENTIFIER_VEL_CTRL:
				
				/*速度控制模式, 期望机械角�?�度*/
				MainCtrl.ExptMecAngularSpeed_pulse =  MC_PULSE_TO_DRV_PULSE(CAN.ReceiveData);
				SpdLoop.ExptMecAngularSpeed_rad = DRV_PULSE_TO_RAD(MainCtrl.ExptMecAngularSpeed_pulse);
			
				break;
			
			case IDENTIFIER_POS_CTRL_ABS:
			
				/*位置控制模式, 绝对位置模式, 期望机械角度*/
				MainCtrl.ExptMecAngle_pulse = MC_PULSE_TO_DRV_PULSE(CAN.ReceiveData);
			
				break;

			case IDENTIFIER_POS_CTRL_REL:
				
				/*位置控制模式, 相对位置模式，期望机械角�?*/
				MainCtrl.ExptMecAngle_pulse = MC_PULSE_TO_DRV_PULSE(CAN.ReceiveData);
			
				break;
						
			case IDENTIFIER_SET_CTRL_MODE:
				
				/*设置控制模式*/
				if(CAN.ReceiveData == SPD_CURR_CTRL_MODE)
				{
					Driver.ControlMode = SPD_CURR_CTRL_MODE;
				}
				else if(CAN.ReceiveData == POS_SPD_CURR_CTRL_MODE)
				{
					Driver.ControlMode = POS_SPD_CURR_CTRL_MODE;
				}
				else if(CAN.ReceiveData == POS_CURR_CTRL_MODE)
				{
					Driver.ControlMode = POS_SPD_CURR_CTRL_MODE;
				}
				else if(CAN.ReceiveData == TORQUE_CTRL_MODE)
				{
					Driver.ControlMode = TORQUE_CTRL_MODE;
				}
				
				DriverCtrlModeInit();
				
				break;

			case IDENTIFIER_SET_ACC:
				
				/*设置加�?�度*/
				MainCtrl.Acceleration_pulse = MC_PULSE_TO_DRV_PULSE(CAN.ReceiveData);
				SpdLoop.Acceleration = DRV_PULSE_TO_RAD(MainCtrl.Acceleration_pulse);
				
				break;

			case IDENTIFIER_SET_DEC:

				/*设置减�?�度*/
				MainCtrl.Deceleration_pulse = MC_PULSE_TO_DRV_PULSE(CAN.ReceiveData);
				SpdLoop.Deceleration = DRV_PULSE_TO_RAD(MainCtrl.Deceleration_pulse);

				break;
					
			case IDENTIFIER_SET_TORQUE_LIMIT:
				
				/*设置转矩限幅, 在转矩控制模式用*/
				TorqueCtrl.MaxTorque_Nm = (float)CAN.ReceiveData * 1e-3;
				CurrLoop.LimitCurrQ = TorqueCtrl.MaxTorque_Nm / (1.5f * MOTOR_POLE_PAIRS_NUM * ROTATOR_FLUX_LINKAGE);
			
				break;
						
			case IDENTIFIER_SET_VEL_LIMIT:
				
				/*设置速度限幅*/
				MainCtrl.MaxMecAngularSpeed_pulse =  MC_PULSE_TO_DRV_PULSE(CAN.ReceiveData);
			
				/*速度控制模式和位置控制模式下速度限幅*/
				if(Driver.ControlMode == SPD_CURR_CTRL_MODE || Driver.ControlMode == POS_SPD_CURR_CTRL_MODE)
				{
					SpdLoop.MaxExptMecAngularSpeed_rad = DRV_PULSE_TO_RAD(MainCtrl.MaxMecAngularSpeed_pulse);
					Saturation_float(&SpdLoop.MaxExptMecAngularSpeed_rad,  MAX_SPD(PosSensor.Generatrix_Vol), - MAX_SPD(PosSensor.Generatrix_Vol));
				}
				/*转矩控制模式下�?�度限幅*/
				else if(Driver.ControlMode == TORQUE_CTRL_MODE)
				{
					TorqueCtrl.MaxMecSpd_rad = DRV_PULSE_TO_RAD(MainCtrl.MaxMecAngularSpeed_pulse);
					Saturation_float(&TorqueCtrl.MaxMecSpd_rad,  MAX_SPD(PosSensor.Generatrix_Vol), - MAX_SPD(PosSensor.Generatrix_Vol));
				}
				
				break;
			
			case IDENTIFIER_SET_POS_LIMIT_UP:
				
				/*设置机械角度上限*/
				MainCtrl.MecAngleUpperLimit_pulse =  MC_PULSE_TO_DRV_PULSE(CAN.ReceiveData);
				PosLoop.MecAngleUpperLimit_rad = (float)DRV_PULSE_TO_RAD(MainCtrl.MecAngleUpperLimit_pulse);
			
				break;
			
			case IDENTIFIER_SET_POS_LIMIT_LOW:
				
				/*设置机械角度下限*/
				MainCtrl.MecAngleLowerLimit_pulse =  MC_PULSE_TO_DRV_PULSE(CAN.ReceiveData);
				PosLoop.MecAngleLowerLimit_rad = (float)DRV_PULSE_TO_RAD(MainCtrl.MecAngleLowerLimit_pulse);
			
				break;

			case IDENTIFIER_CORRECT_POS_OFFSET:
				
				/*失能ADC, 停止进入ADC中断*/
				ADC_CMD(DISABLE);

				/*矫正编码器偏移量, 在中断中执行以阻塞程�?*/
				Driver.UnitMode = CORRECT_POS_OFFSET_MODE;
				CorrectPosOffset_Encoder(ENCODER_CORRECT_VOL_D);
			
				/*进入工作模式*/
				Driver.UnitMode = WORK_MODE;
				ADC_CMD(ENABLE);
			
				break;		
			
			case (0x40 + IDENTIFIER_READ_TORQUE):
				
				/*读取电磁转矩*/
				CAN.RecieveStatus = (0x40 + IDENTIFIER_READ_TORQUE);
				
				break;
						
			case (0x40 + IDENTIFIER_READ_VEL):
				
				/*读取机械角�?�度*/
				CAN.RecieveStatus = (0x40 + IDENTIFIER_READ_VEL);
			
				break;

			case (0x40 + IDENTIFIER_READ_POS):
				
				/*读取绝对机械角度*/
				CAN.RecieveStatus = (0x40 + IDENTIFIER_READ_POS);
		
				break; 
			
			case (0x40 + IDENTIFIER_READ_ENCODER_POS):
				
				/*读取编码�?*/
				CAN.RecieveStatus = (0x40 + IDENTIFIER_READ_ENCODER_POS);
		
				break; 
						
			case (0x40 + IDENTIFIER_READ_VOL_D):
				
				/*读取Vd*/
				CAN.RecieveStatus = (0x40 + IDENTIFIER_READ_VOL_D);
			
				break;
			
			case (0x40 + IDENTIFIER_READ_CURR_D):
				
				/*读取Id*/
				CAN.RecieveStatus = (0x40 + IDENTIFIER_READ_CURR_D);
				
				break;
			
			case (0x40 + IDENTIFIER_READ_VOL_Q):
				
				/*读取Vq*/
				CAN.RecieveStatus = (0x40 + IDENTIFIER_READ_VOL_Q);
			
				break;
			
			case (0x40 + IDENTIFIER_READ_CURR_Q):
				
				/*读取Iq*/
				CAN.RecieveStatus = (0x40 + IDENTIFIER_READ_CURR_Q);
				
				break;
			
			case (0x40 + IDENTIFIER_READ_SPD_LOOP_OUTPUT):
				
				/*读取速度环输�?*/
				CAN.RecieveStatus = (0x40 + IDENTIFIER_READ_SPD_LOOP_OUTPUT);
			
				break;
			
			case (0x40 + IDENTIFIER_READ_POS_LOOP_OUTPUT):
				
				/*读取位置环输�?*/
				CAN.RecieveStatus = (0x40 + IDENTIFIER_READ_POS_LOOP_OUTPUT);
				
				break;
			case (0x40 + IDENTIFIER_READ_LOAD_OBSERVER):
				
				/*读取位置环输�?*/
				CAN.RecieveStatus = (0x40 + IDENTIFIER_READ_LOAD_OBSERVER);
				
				break;	
			
			case (0x40 + IDENTIFIER_READ_ACC):
				
				/*读取加速度*/
				CAN.RecieveStatus = (0x40 + IDENTIFIER_READ_ACC);
				
				break;
			

						
			default:
				
				break;
		}	
	}
	
	CAN_Respond();
	
	MainCtrl.CAN_FaultSign = 0;
	
	/*判断RX FIFO0中是否还有消�?*/
	if(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0))
	{
		goto first_line;
	}
}

void CAN_Respond(void)
{
	switch (CAN.RecieveStatus)
	{
		/*发�?�电磁转�?*/	
		case (0x40 + IDENTIFIER_READ_TORQUE):
			
			CAN.Identifier = IDENTIFIER_READ_TORQUE;
			CAN.TransmitData = (int32_t)(TorqueCtrl.EleTorque_Nm * 1e3);
		
			CAN_Transmit(CAN.Identifier, CAN.TransmitData, 4, DRIVER_CLIENT_CAN_ID);
		
			CAN.RecieveStatus = 0;
		
			break;
			
		/*发�?�机械角速度*/			
		case (0x40 + IDENTIFIER_READ_VEL):
			
			CAN.Identifier = IDENTIFIER_READ_VEL;
			CAN.TransmitData = (int32_t)RAD_TO_MC_PULSE(PosSensor.MecAngularSpeed_rad);
			CAN_Transmit(CAN.Identifier, CAN.TransmitData, 4, DRIVER_CLIENT_CAN_ID);
			CAN.RecieveStatus = 0;
		
			break;		
		
		/*发�?�机械角�?*/
		case (0x40 + IDENTIFIER_READ_POS):
			
			CAN.Identifier = IDENTIFIER_READ_POS;
			CAN.TransmitData = (int32_t)DRV_PULSE_TO_MC_PULSE(MainCtrl.RefMecAngle_pulse);
		
			CAN_Transmit(CAN.Identifier, CAN.TransmitData, 4, DRIVER_CLIENT_CAN_ID);
		
			CAN.RecieveStatus = 0;
		
			break;
		
		/*发�?�编码器位置*/
		case (0x40 + IDENTIFIER_READ_ENCODER_POS):
			
			CAN.Identifier = IDENTIFIER_READ_ENCODER_POS;
			CAN.TransmitData = (int32_t)DRV_PULSE_TO_MC_PULSE(PosSensor.MecAngle_15bit);
		
			CAN_Transmit(CAN.Identifier, CAN.TransmitData, 4, DRIVER_CLIENT_CAN_ID);
		
			CAN.RecieveStatus = 0;
		
			break;

		/*发�?�Vd*/		
		case (0x40 + IDENTIFIER_READ_VOL_D):
			
			CAN.Identifier = IDENTIFIER_READ_VOL_D;
			CAN.TransmitData = (int32_t)(CurrLoop.CtrlVolD * 1e3);
		
			CAN_Transmit(CAN.Identifier, CAN.TransmitData, 4, DRIVER_CLIENT_CAN_ID);
		
			CAN.RecieveStatus = 0;
		
			break;

		/*发�?�Id*/
		case (0x40 + IDENTIFIER_READ_CURR_D):
			
			CAN.Identifier = IDENTIFIER_READ_CURR_D;
			CAN.TransmitData = (int32_t)(CoordTrans.CurrD * 1e3);
		
			CAN_Transmit(CAN.Identifier, CAN.TransmitData, 4, DRIVER_CLIENT_CAN_ID);
		
			CAN.RecieveStatus = 0;
		
			break;
		
		/*发�?�Vq*/		
		case (0x40 + IDENTIFIER_READ_VOL_Q):
			
			CAN.Identifier = IDENTIFIER_READ_VOL_Q;
			CAN.TransmitData = (int32_t)(CurrLoop.CtrlVolQ * 1e3);
		
			CAN_Transmit(CAN.Identifier, CAN.TransmitData, 4, DRIVER_CLIENT_CAN_ID);
		
			CAN.RecieveStatus = 0;
		
			break;

		/*发�?�Iq*/
		case (0x40 + IDENTIFIER_READ_CURR_Q):
			
			CAN.Identifier = IDENTIFIER_READ_CURR_Q;
			CAN.TransmitData = (int32_t)(CoordTrans.CurrQ * 1e3);
		
			CAN_Transmit(CAN.Identifier, CAN.TransmitData, 4, DRIVER_CLIENT_CAN_ID);
		
			CAN.RecieveStatus = 0;
		
			break;
		
		/*发�?��?�度环输�?*/		
		case (0x40 + IDENTIFIER_READ_SPD_LOOP_OUTPUT):
			
			CAN.Identifier = IDENTIFIER_READ_SPD_LOOP_OUTPUT;
			CAN.TransmitData = (int32_t)(CurrLoop.ExptCurrQ * 1e3);
		
			CAN_Transmit(CAN.Identifier, CAN.TransmitData, 4, DRIVER_CLIENT_CAN_ID);
		
			CAN.RecieveStatus = 0;
		
			break;

		/*发�?�位置环输出*/
		case (0x40 + IDENTIFIER_READ_POS_LOOP_OUTPUT):
			
			CAN.Identifier = IDENTIFIER_READ_POS_LOOP_OUTPUT;
			CAN.TransmitData = (int32_t)(SpdLoop.ExptMecAngularSpeed_rad * 1e3);
		
			CAN_Transmit(CAN.Identifier, CAN.TransmitData, 4, DRIVER_CLIENT_CAN_ID);
		
			CAN.RecieveStatus = 0;
		
			break;
        
		case (0x40 +IDENTIFIER_HALL_TEST_OFF):
			
			CAN.Identifier = IDENTIFIER_HALL_TEST_OFF;
		
			CAN_Transmit(CAN.Identifier, CAN.TransmitData, 1, DRIVER_CLIENT_CAN_ID - 1);
		
			CAN.RecieveStatus = 0;
		
			break;

		case (0x40 +IDENTIFIER_READ_ACC):
			
			CAN.Identifier = IDENTIFIER_READ_ACC;
			CAN.TransmitData = (int32_t)DRV_PULSE_TO_MC_PULSE(MainCtrl.Acceleration_pulse);
			CAN_Transmit(CAN.Identifier, CAN.TransmitData, 4, DRIVER_CLIENT_CAN_ID);
		
			CAN.RecieveStatus = 0;

			break;	
		

		
		default:
			
			break;
	}
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
	CAN_Respond();
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
	CAN_Respond();
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
	CAN_Respond();
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
	static uint16_t errTimes = 0;
	
	UART_Transmit_DMA("CAN ERROR: %d\r\n", (uint32_t)hcan->ErrorCode);
	SendBuf();
	
	if(errTimes++ <= 500)           //100
	{
		errTimes++;
		
		HAL_CAN_DeInit(&hcan1);
		HAL_CAN_Init(&hcan1);
		
		__HAL_CAN_CLEAR_FLAG(&hcan1, CAN_FLAG_FOV0);
		
		HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	}
	else
	{
//		PWM_CMD(DISABLE);
	}
}

void CAN_Transmit(uint8_t identifier, int32_t transmitData, uint8_t length, uint32_t StdId)
{
	uint32_t absData = abs(transmitData);
	
	TxMessage.StdId = StdId;
	TxMessage.ExtId = 0u;
	TxMessage.IDE   = CAN_ID_STD;
	TxMessage.RTR   = CAN_RTR_DATA;
	TxMessage.DLC   = length;
	
	CAN.Transmit.data_uint8[0] = (identifier>>0)&0xFF;
	
	if(transmitData >= 0)
	{
		for(uint8_t bytes = 1; bytes <= (length - 1); bytes++)
		{
			CAN.Transmit.data_uint8[bytes] = (absData>>((bytes - 1) * 8))&0xFF;
		}	
	}
	else if(transmitData < 0)
	{
		for(uint8_t bytes = 1; bytes <= (length - 1); bytes++)
		{

			CAN.Transmit.data_uint8[bytes] = (absData>>((bytes - 1) * 8))&0xFF;
			if(bytes == (length - 1))
			{
				CAN.Transmit.data_uint8[bytes] = ((absData>>((bytes - 1) * 8))&0xFF) | 0x80; //加负号
			}
		}	
	}
	
	HAL_CAN_AddTxMessage(&hcan1, &TxMessage, (uint8_t *)&CAN.Transmit, &CAN.MailBox);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY);
}


void CAN_Receive(uint32_t *stdId, uint8_t *identifier, int32_t *receiveData)
{	
	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxMessage0, (uint8_t *)&CAN.Receive);
	
	*stdId = RxMessage0.StdId;
	
	*identifier = CAN.Receive.data_uint8[0];
	
	if(((CAN.Receive.data_uint8[3]&0x80)>>7) == 0)
	{
		/*当数据为正数�?*/
		*receiveData = (int32_t)((CAN.Receive.data_uint8[3]<<16) | (CAN.Receive.data_uint8[2]<<8) | (CAN.Receive.data_uint8[1]<<0));
	}
	else if(((CAN.Receive.data_uint8[3]&0x80)>>7) == 1)
	{
		/*当数据为负数�?*/
		*receiveData = -(int32_t)(((CAN.Receive.data_uint8[3]&0x7F)<<16) | (CAN.Receive.data_uint8[2]<<8) | (CAN.Receive.data_uint8[1]<<0));
	}
}

/*设置双ID滤波器，只接收两种ID的消息（点对点的ID与广播模式的ID�?*/	
void CAN_Enable(void)
{
	CAN_FilterTypeDef CAN1_FilerConf    = {0};
	/*设置CAN滤波器为16位宽的列表模�?*/
	CAN1_FilerConf.FilterBank           = 0;
	CAN1_FilerConf.FilterMode           = CAN_FILTERMODE_IDLIST ;
	CAN1_FilerConf.FilterScale          = CAN_FILTERSCALE_16BIT;
	CAN1_FilerConf.FilterActivation     = ENABLE;
	CAN1_FilerConf.SlaveStartFilterBank = 14;
	CAN1_FilerConf.FilterIdHigh         = DRIVER_SERVER_CAN_ID << 5;
	CAN1_FilerConf.FilterIdLow          = DRIVER_SERVER_CAN_ID << 5;
	CAN1_FilerConf.FilterMaskIdHigh     = DRIVER_BROADCAST_ID << 5;
	CAN1_FilerConf.FilterMaskIdLow      = DRIVER_BROADCAST_ID << 5;
	CAN1_FilerConf.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	CAN1_FilerConf.FilterActivation     = ENABLE;

	if(HAL_CAN_ConfigFilter(&hcan1,&CAN1_FilerConf) != HAL_OK) 
	{
		Error_Handler();
	}

	/* Filter bank 1: 32-bit mask mode for DroneCAN ESC RawCommand (extended ID) */
	/* Matches any broadcast frame with DTID=1030 (0x406) and service bit=0     */
	CAN_FilterTypeDef CAN1_FilterDroneCAN = {0};
	CAN1_FilterDroneCAN.FilterBank           = 1;
	CAN1_FilterDroneCAN.FilterMode           = CAN_FILTERMODE_IDMASK;
	CAN1_FilterDroneCAN.FilterScale          = CAN_FILTERSCALE_32BIT;
	CAN1_FilterDroneCAN.FilterActivation     = ENABLE;
	CAN1_FilterDroneCAN.SlaveStartFilterBank = 14;
	CAN1_FilterDroneCAN.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	/* FilterId = (DRONECAN_ESC_RAW_CMD_FILTER << 3) | CAN_ID_EXT = 0x00203004 */
	CAN1_FilterDroneCAN.FilterIdHigh         = 0x0020u;
	CAN1_FilterDroneCAN.FilterIdLow          = 0x3004u;
	/* FilterMask = (DRONECAN_MSG_DTID_MASK << 3) | CAN_ID_EXT = 0x07FFFC04   */
	CAN1_FilterDroneCAN.FilterMaskIdHigh     = 0x07FFu;
	CAN1_FilterDroneCAN.FilterMaskIdLow      = 0xFC04u;
	if (HAL_CAN_ConfigFilter(&hcan1, &CAN1_FilterDroneCAN) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_LAST_ERROR_CODE);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_BUSOFF);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_ERROR);
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
