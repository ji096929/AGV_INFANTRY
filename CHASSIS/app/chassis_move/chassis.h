/*
 * @Author: ShenYuJie 1647262812@qq.com
 * @Date: 2024-03-09 20:29:16
 * @LastEditors: ShenYuJie 1647262812@qq.com
 * @LastEditTime: 2024-03-10 16:32:16
 * @FilePath: \MDK-ARMd:\EVENTS\2024_Hero\CHASSIS\CHASSIS\app\chassis_move\chassis.h
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
#ifndef CHASSIS_H_
#define CHASSIS_H_

#include "stdbool.h"
#include "motor.h"
#include "PID.h"

#define GIMBAL_HEAD_ANGLE -28.5f

#define CHASSIS_SPEED_X_CHANGE_MAX 20.f
#define CHASSIS_SPEED_Y_CHANGE_MAX 20.f
typedef enum	
{
	CHASSIS_REMOTE_CLOSE	=	0x00u,
	CHASSIS_NORMAL	=		0x01u,
	CHASSIS_SPIN		=		0x02u,
	CHASSIS_PRECISIOUS		=		0x03u,
	
}CHASSIS_MODE_E;

typedef struct
{
	CHASSIS_MODE_E mode;
	bool invert_flag;
	float relative_angle;
	bool break_mode;
	bool follow_switch;
	bool speed_slow;
}CHASSIS_PARAMETER_T;

typedef struct
{
	float vx;
	float set_vx;
	float vy;
	float set_vy;
	float vw;
	float set_vw;
}CHASSIS_COMMAND_T;

typedef struct
{
	float linear_vel;			
	float rpm;	
	int output;					
}Speed_t;

typedef struct
{
	float temp;
	float ChassisCoordinate_Angle;
	float target_angle;
	float last_target_angle;
	int zero_position;
	Speed_t target_speed;
	uint8_t ID;
}CHASSIS_MOTOR_T;

typedef struct
{
	CHASSIS_PARAMETER_T parameter;
	CHASSIS_COMMAND_T	command;
	CHASSIS_MOTOR_T	 A_motor;
	CHASSIS_MOTOR_T	 B_motor;
	CHASSIS_MOTOR_T	 C_motor;
	CHASSIS_MOTOR_T	 D_motor;
}CHASSIS_T;


typedef struct
{
	float number_ratio;
}YAW_PARAMETER_T;

typedef struct
{
	int16_t rounds;

	float total_angle;
	float actual_angle;
	
}YAW_STATUS_T;

typedef struct
{
	GM6020_T	motor;
	YAW_STATUS_T	status;
	YAW_PARAMETER_T	parameter;

}YAW_T;

typedef struct
{
	PID_TypeDef speed_loop;
	PID_TypeDef angle_loop;
}PID_T;

extern CHASSIS_T chassis;
extern YAW_T   yaw;
extern PID_T   yaw_pid;

void Chassis_Move(void);
void Yaw_Angle_Process(YAW_T  *yaw);
void Yaw_Init(void);
void Chassis_Init(void);

#endif