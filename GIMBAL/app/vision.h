#ifndef VISION_H_
#define VISION_H_


#include "stdint.h"
typedef __packed enum{
	
	VISION_OFF		=	0X00U,
	VISION_ON	=	0X01U,
}VISION_MODE;

typedef __packed struct {

	float yaw_angle;
	float pitch_angle;
	float x;
	float y;
	float z;
	uint16_t receive_time;
	uint16_t last_receive_time;	
    
}VISION_COMMAND_T;

typedef __packed struct VISION_T{
	
	VISION_MODE mode;
	VISION_COMMAND_T command;
}VISION_T;

void Vision_Angle_Task(float target_yaw_angle,float target_pitch_angle);
void Vision_Aim_Data_Task(float x,float y,float z);
void Vision_Send_Task(void);
extern VISION_T vision_control;
#endif

