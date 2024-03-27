#ifndef GIMBAL_CONNECTION_H_
#define GIMBAL_CONNECTION_H_

#include "drv_can.h"
#include "chassis.h"

typedef enum
{
	INVERT_ON	=	0x01,
	INVERT_OFF=	0x00,
}INVERT_FLAG_E;

typedef enum
{
	FOLLOW_ON	=	0x01,
	FOLLOW_OFF=	0x00,
}FOLLOW_FLAG_E;

typedef enum
{
	FRIC_ON	=	0x01,
	FRIC_OFF=	0x00,
}FRIC_FLAG_E;

typedef enum
{
	VISION_ON	=	0x01,
	VISION_OFF=	0x00,
}VISION_FLAG_E;

typedef struct 
{
   float remp;
}GIMBAL_TX_T;

typedef struct
{
	bool flag;
	bool last_flag;
}CHASSIS_MODE_STATE_T;//1Ϊ������0Ϊ�ر�

typedef struct 
{
   CHASSIS_MODE_E mode;
   CHASSIS_MODE_STATE_T invert;
   CHASSIS_MODE_STATE_T follow;
	 CHASSIS_MODE_STATE_T Graphic_Init;
	 CHASSIS_MODE_STATE_T	fric;
	 CHASSIS_MODE_STATE_T vision;
	 float pitch_angle;
	 int16_t fric_speed;
   int16_t  vx;
   int16_t vy;
   int16_t vw;

}GIMBAL_RX_T;

typedef struct 
{
    GIMBAL_RX_T connection_rx;
    GIMBAL_TX_T connection_tx;

}GIMBAL_CONNECTION_T;
void Chassis_Flag_Update(GIMBAL_CONNECTION_T *connection);
void Fric_Speed_And_Pitch_Angle_Update(GIMBAL_CONNECTION_T *connection,uint8_t	data[]);
void Chassis_Speed_Command_Update(GIMBAL_CONNECTION_T *connection,uint8_t	data[]);
void Chassis_Control_Mode_Update(GIMBAL_CONNECTION_T *connection,uint8_t	data[]);
extern GIMBAL_CONNECTION_T connection;
#endif