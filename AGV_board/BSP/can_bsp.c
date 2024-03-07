#include "can_bsp.h"
#include "briter_encoder.h"
#include "M3508_gear.h"
#include "SW_control_task.h"
#if defined(STM32F105) | defined(STM32F407)
	#include "can.h"
#endif

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef CAN_RxHeaderStruct;
	uint8_t rxdata[8];
	int16_t speed,*gdata,current;
	float angle;
	if(hcan==&hcan1)
	{
		HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&CAN_RxHeaderStruct,rxdata);

		//briter_encoder_feedback_handler(&briter_encoder, rxdata);
		M3508_feedback_handler(&M3508_bus_1, CAN_RxHeaderStruct.StdId, rxdata);
		M3508_gear_feedback_handler(&steering_wheel.directive_part.motor.M3508_kit);
		M3508_gear_feedback_handler(&steering_wheel.motion_part.motor.M3508_kit);
		
		
		// 因为switch只能用常量，所以改用If
		if (CAN_RxHeaderStruct.StdId == steering_wheel.directive_part.encoder.briter_encoder.parameter.CAN_ID)
		{
			briter_encoder_feedback_handler(&steering_wheel.directive_part.encoder.briter_encoder, rxdata);
		}

	}
	if (hcan==&hcan2)
	{
		HAL_CAN_GetRxMessage(&hcan2,CAN_RX_FIFO0,&CAN_RxHeaderStruct,rxdata);
	if (CAN_RxHeaderStruct.IDE == CAN_ID_EXT)
			steering_communication_rx_handler(CAN_RxHeaderStruct.ExtId, rxdata);
	}

}

void can_filter_set(CAN_FilterTypeDef *CAN_Filter,uint16_t equipment_id,uint8_t cmd_id,uint16_t data2_part,CAN_FILTER_IMPROVE_E 	improve)
{
	if(improve	&	EXT_ID_MODE)
	{
		CAN_Filter->FilterIdHigh = data2_part>>5|cmd_id<<5;
		CAN_Filter->FilterIdLow = equipment_id<<3|data2_part<<11;
		if(improve & EQUIPMENT_ID_IMPROVE)
		{
		CAN_Filter->FilterMaskIdHigh = 0x0000;
		CAN_Filter->FilterMaskIdLow =  0xFF<<3;
		}
		else if(improve & EQUIPMENT_ID_NORMAL)
		{
		CAN_Filter->FilterMaskIdHigh = 0x0000;
		CAN_Filter->FilterMaskIdLow =  0x0000;
		}
		if(improve &	DATA2_IMPROVE)
		{
		CAN_Filter->FilterMaskIdHigh = 0x07FF|CAN_Filter->FilterMaskIdHigh;
		CAN_Filter->FilterMaskIdLow =  0x1F<<11|CAN_Filter->FilterMaskIdLow;
		}
		else	if	(improve & DATA2_NORMAL)
		{
		CAN_Filter->FilterMaskIdHigh = 0x0000|CAN_Filter->FilterMaskIdHigh;
		CAN_Filter->FilterMaskIdLow =  0x0000<<11|CAN_Filter->FilterMaskIdLow;
		}
		
			if(improve &	CMD_ID_IMPROVE)
		{
		CAN_Filter->FilterMaskIdHigh = 0x1F<<11|CAN_Filter->FilterMaskIdHigh;
		CAN_Filter->FilterMaskIdLow =CAN_Filter->FilterMaskIdLow;
		}
		else	if	(improve & CMD_ID_NORMAL)
		{
		CAN_Filter->FilterMaskIdHigh = 0x0000|CAN_Filter->FilterMaskIdHigh;
		CAN_Filter->FilterMaskIdLow =CAN_Filter->FilterMaskIdLow;
		}
	}
	else if(improve	& STD_ID_MODE)
	{
		CAN_Filter->FilterIdHigh = equipment_id<<5;
		CAN_Filter->FilterIdLow = 0x0000;
		if(improve & EQUIPMENT_ID_IMPROVE)
		{
		
		CAN_Filter->FilterMaskIdHigh = 0x7FF<<5;
		CAN_Filter->FilterMaskIdLow = 0x0000;
		}
		else if(improve & EQUIPMENT_ID_NORMAL)
		{
		CAN_Filter->FilterMaskIdHigh = 0x0000<<5;
		CAN_Filter->FilterMaskIdLow = 0x0000;
		}
	}
	CAN_Filter->FilterBank++;
}

void platform_filtter_config_setting(void)
{
	#if defined(STM32F105) | defined(STM32F407)
		CAN_FilterTypeDef CAN_FilterStructure;
		CAN_FilterStructure.FilterActivation = ENABLE;
		CAN_FilterStructure.FilterBank = 0;
		CAN_FilterStructure.FilterMode = CAN_FILTERMODE_IDMASK;
		CAN_FilterStructure.FilterScale = CAN_FILTERSCALE_32BIT;
		CAN_FilterStructure.FilterFIFOAssignment = CAN_FILTER_FIFO0;
		can_filter_set(&CAN_FilterStructure,0x201,0,0,STD_ID_MODE);
		can_filter_set(&CAN_FilterStructure,0x202,0,0,STD_ID_MODE);
//	CAN_FilterStructure.FilterIdHigh = 0x0000;
//		CAN_FilterStructure.FilterIdLow = 0x0000;
//	
//		CAN_FilterStructure.FilterMaskIdHigh = 0x0000;
//		CAN_FilterStructure.FilterMaskIdLow =  0x0000;
		HAL_CAN_ConfigFilter(&hcan1,&CAN_FilterStructure);
	  CAN_FilterStructure.FilterBank = 15;
		#ifdef	AGV_BOARD_A
		can_filter_set(&CAN_FilterStructure,A_STEERING_CAN_ID,0,0,EXT_ID_MODE);
		#endif
		#ifdef	AGV_BOARD_B
		can_filter_set(&CAN_FilterStructure,B_STEERING_CAN_ID,0,0,EXT_ID_MODE);
		#endif
		#ifdef	AGV_BOARD_C
		can_filter_set(&CAN_FilterStructure,C_STEERING_CAN_ID,0,0,EXT_ID_MODE);
		#endif
		#ifdef	AGV_BOARD_D
		can_filter_set(&CAN_FilterStructure,D_STEERING_CAN_ID,0,0,EXT_ID_MODE);
		#endif
		HAL_CAN_ConfigFilter(&hcan2,&CAN_FilterStructure);
	#endif
}

