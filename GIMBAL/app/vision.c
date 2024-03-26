#include "vision.h"


VISION_T vision_control;

void Vision_Angle_Task(float target_yaw_angle,float target_pitch_angle)
{

	vision_control.command.yaw_angle	=	-target_yaw_angle;
	vision_control.command.pitch_angle	=	-target_pitch_angle;

};

void Vision_Aim_Data_Task(float x,float y,float z)
{
	vision_control.command.x	=	x;
	vision_control.command.y	=	y;
	vision_control.command.z	=	z;
	
}

void Vision_Send_Task(void)
{
	DMA_Send();

}
