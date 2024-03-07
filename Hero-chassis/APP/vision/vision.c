/**
  ******************************************************************************
  * @file    Project/APP/vision.c 
  * @author  NEUQRM
  * @version V1.0.0
  * @date    12.2021
  * @brief   是否进入视觉模式（因为仅需要接收云台的数据，所以如果上位机不直接接到底盘上该文件就可以忽略）
  ******************************************************************************
  * @attention
  ******************************************************************************
  ****************************(C) COPYRIGHT 2021 NEUQRM****************************

  ...........  革命尚未成功，同志仍需努力  ...........
*/

#include "vision.h"

VISION_t vision_mode=VISION_OFF;
VISION_GET_t vision_sent;

//void Vision_Task(void)
//{
//	if(vision_mode==VISION_OFF)
//	{
//		gimbal_y.add_angle=rc_sent.yaw.Target_Angle;
//		gimbal_y.Target_Speed=rc_sent.yaw.Target_Speed*0.5f;
//	}
//	else
//	{
//		gimbal_y.add_angle=vision_sent.yaw.Target_Angle;
//	}
//}

