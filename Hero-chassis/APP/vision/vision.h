#ifndef __VISION_H
#define __VISION_H

#include "yaw_turn.h"


typedef enum{
    VISION_ON= 0, //自瞄开
    VISION_OFF,    //
}VISION_t;

typedef struct{
	
	float Target_Angle;
	float Target_Speed;
}GIMBAL_VI_t;

//上位机发送的值
typedef struct{
	GIMBAL_VI_t yaw;
	GIMBAL_VI_t pitch;
}VISION_GET_t;

#endif
