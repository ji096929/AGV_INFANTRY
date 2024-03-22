/*
 * @Author: ShenYuJie 1647262812@qq.com
 * @Date: 2024-02-27 21:59:21
 * @LastEditors: ShenYuJie 1647262812@qq.com
 * @LastEditTime: 2024-03-11 03:28:03
 * @FilePath: \MDK-ARMd:\EVENTS\2024_Hero\GIMBAL\app\fric.h
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
#ifndef FRIC_H_
#define FRIC_H_

#include "motor.h"
#include "PID.h"
#include "drv_can.h"

#define FRIC_HIGH_SPEED 5200.f
#define FRIC_NONE_SPEED 0.f

typedef __packed enum
{
    FRIC_STOP = 0,
    FRIC_RUNNING,
    FRIC_ERROR,
}FRIC_MODE_E;

typedef  struct
{
	PID_TypeDef speed_loop;
	PID_TypeDef angle_loop;
}FRIC_PID_T;

typedef  struct 
{
   FRIC_MODE_E mode;
	 int8_t	lr_error;
}FRIC_PARAMETER_T;

typedef  struct 
{
    int16_t target_speed;
	
}FRIC_COMMAND_T;

typedef  struct 
{
    float actual_speed;
    float given_current;
    float temperature;
    
}FRIC_STATUE_T;

typedef  struct
{
    FRIC_COMMAND_T command;
    FRIC_PID_T pid;
    M3508_T motor;
    FRIC_STATUE_T status;
}FRIC_MOTOR_T;

typedef  struct 
{
    FRIC_PARAMETER_T parameter;
    FRIC_MOTOR_T left_motor;
    FRIC_MOTOR_T right_motor;
}FRIC_T;



extern FRIC_T  fric;
void Fric_Init(void);
void Fric_Task(void);

#endif // !FRIC_H_

