#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "can.h"
#include "chassis_move.h"
#include "trigger_turn.h"
#include "yaw_turn.h"
#include "Briter_Encoder.h"

#define MODE_SWITCH        1
#define MODE_NO_SWITCH     0



#define YAW_ID    				  0x206

#define CAN_GIMBAL_Y_ID           	  0x20E
#define LOOP_BACK_ID              	  0x003
#define YAW_CONTROL_ID            	  0x006
#define GIMBAL_CONTROL_ID         	  0x007
#define SHOOT_MOTOR_TRIGGER_ID    	  0x208
#define MODE_RECEIVE_ID           	  0X009
#define UI_ID                     	  0x010
#define CAN_RefereeData_ID        	  0x011
#define Chassis_Shoot_Task_Rx_ID       	  0x021
#define Chassis_Motor_Speed_ID	  	  0x013
#define TRIGGER_CONTROL_ID		  	  0x014
#define CAN_Yaw_Invert_Flag_Trans_ID  0x015
#define CAN_Beta_Power_Limit_ID		  0x016
#define Relay_Mode_Set_ID			  0x017
#define CAN_YAW_RX_ID				0x018
#define CAN_YAW_TX_ID				0x019

typedef enum
{
	EXT_ID_MODE	=	0x00u,
	STD_ID_MODE	=	0x01u,
	EQUIPMENT_ID_IMPROVE	=0X02U,
	EQUIPMENT_ID_NORMAL		=0X04U,
	CMD_ID_IMPROVE				=	0X08U,
	CMD_ID_NORMAL					=	0X10U,
	DATA2_IMPROVE					=	0x20U,
	DATA2_NORMAL					=	0X40U,
}CAN_FILTER_IMPROVE_E;


uint8_t bsp_can_init(void);
uint8_t Can_Tx_Message(CAN_HandleTypeDef *hcan,uint8_t *mdata);

void Encoder_Data_Process(uint8_t *rxdata, Briter_Encoder_t *Encoder, MOTOR_t *Motor);
void canTX_To_Gimbal_Yaw_Callback(int16_t Angle,int16_t Speed,int16_t shoot_flag_t);
void send_gimbal_data_2(void);
void canTX_Briter_Encoder(Briter_Encoder_t Paramater,CAN_HandleTypeDef *hcan);
void UartTX_To_BetaBoard_Briter_Encoder(Briter_Encoder_t Encoder,int Typecode);
void canTX_To_BetaBoard_WheelVel(void);
void canTX_AGV_Chassis_Motor_Current(void);
void record_yaw_callback(float angle,float speed);
void canTX_To_Beta_Power_Limit(int Power_Mode);
	
extern float pich_angle;
extern int mode_now;
extern char M_DAta[5];

#endif

	
	

