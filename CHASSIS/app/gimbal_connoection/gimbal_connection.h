#ifndef GIMBAL_CONNECTION_H_
#define GIMBAL_CONNECTION_H_

#include "drv_can.h"
#include "chassis.h"
typedef struct 
{
   float remp;
}GIMBAL_TX_T;

typedef struct
{
	bool flag;
	bool last_flag;
}CHASSIS_MODE_STATE_T;//1为开启，0为关闭

typedef struct 
{
   CHASSIS_MODE_E mode;
   CHASSIS_MODE_STATE_T invert;
   CHASSIS_MODE_STATE_T follow;
	 CHASSIS_MODE_STATE_T Graphic_Init;
   float  vx;
   float vy;
   float vw;

}GIMBAL_RX_T;

typedef struct 
{
    GIMBAL_RX_T connection_rx;
    GIMBAL_TX_T connection_tx;

}GIMBAL_CONNECTION_T;

void Chassis_Speed_Command_Update(GIMBAL_CONNECTION_T *connection,uint8_t	data[]);
void Chassis_Control_Mode_Update(GIMBAL_CONNECTION_T *connection,uint8_t	data[]);
extern GIMBAL_CONNECTION_T connection;
#endif