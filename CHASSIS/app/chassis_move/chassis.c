/*
 * @Author: ShenYuJie 1647262812@qq.com
 * @Date: 2024-03-09 20:29:02
 * @LastEditors: ShenYuJie 1647262812@qq.com
 * @LastEditTime: 2024-03-10 16:23:53
 * @FilePath: \MDK-ARMd:\EVENTS\2024_Hero\CHASSIS\CHASSIS\app\chassis_move\chassis.c
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
#include "chassis.h"

#include "gimbal_connection.h"
#include "kinematic.h"



CHASSIS_T chassis;
YAW_T   yaw;
PID_T   yaw_pid;
float yaw_position_loop_data[10]= {0.07f,0.0f,0.f,2.5f,0.0f,1.0f,0.f,0.f,0.f,0.f};

void Chassis_Speed_Slow_Motion(CHASSIS_T *chassis)
{
		if(chassis->parameter.speed_slow)
		{
			if(chassis->command.set_vx-chassis->command.vx	>CHASSIS_SPEED_X_CHANGE_MAX)
			{
				chassis->command.vx+=CHASSIS_SPEED_X_CHANGE_MAX;
			}
			else if(chassis->command.set_vx-chassis->command.vx	<-CHASSIS_SPEED_X_CHANGE_MAX)
			{
				chassis->command.vx-=CHASSIS_SPEED_X_CHANGE_MAX;
			}
			else 
			{
				chassis->command.vx=chassis->command.set_vx;
			}
			if(chassis->command.set_vy-chassis->command.vy	>CHASSIS_SPEED_Y_CHANGE_MAX)
			{
				chassis->command.vy+=CHASSIS_SPEED_Y_CHANGE_MAX;
			}
			else if(chassis->command.set_vy-chassis->command.vy	<-CHASSIS_SPEED_Y_CHANGE_MAX)
			{
				chassis->command.vy-=CHASSIS_SPEED_Y_CHANGE_MAX;
			}
			else 
			{
				chassis->command.vy=chassis->command.set_vy;
			}
				chassis->command.vw=chassis->command.set_vw;
		}
		else
		{
			chassis->command.vw=chassis->command.set_vw;
			chassis->command.vx=chassis->command.set_vx;
			chassis->command.vy=chassis->command.set_vy;
		}
}

void Chassis_Inveter_Judge(void)
{
		chassis.parameter.invert_flag	=	connection.connection_rx.invert.flag;
		chassis.parameter.follow_switch	=	connection.connection_rx.follow.flag;
    if(chassis.parameter.invert_flag)
    {
        chassis.command.set_vx = -connection.connection_rx.vx;
        chassis.command.set_vy = -connection.connection_rx.vy;
        chassis.command.set_vw = -connection.connection_rx.vw/10.0f;
    }
    else
    {
        chassis.command.set_vx = connection.connection_rx.vx;
        chassis.command.set_vy = connection.connection_rx.vy;
        chassis.command.set_vw = connection.connection_rx.vw/10.0f;
    }
}

void Yaw_Angle_Process(YAW_T  *yaw)
{
		
    yaw->status.total_angle =	yaw->motor.status.total_position_degree/yaw->parameter.number_ratio;
		while(yaw->status.total_angle-yaw->status.rounds*360.0f>360.0f) yaw->status.rounds++;
		while(yaw->status.total_angle-yaw->status.rounds*360.0f<-360.0f) yaw->status.rounds--;
    yaw->status.actual_angle    =   yaw->status.total_angle - yaw->status.rounds*360.0f;
}

void Gimbal_To_Chassis_Relative_Angle_Update(void)
{
    float gimbal_angle,chassis_angle;
    chassis_angle   =   yaw.status.actual_angle;
    gimbal_angle    =   GIMBAL_HEAD_ANGLE+180.0f*chassis.parameter.invert_flag;
    chassis.parameter.relative_angle =   chassis_angle-gimbal_angle;
    if(chassis.parameter.relative_angle>180.0f) chassis.parameter.relative_angle-=360.0f;
    if(chassis.parameter.relative_angle<-180.0f) chassis.parameter.relative_angle+=360.0f;
}

static float chassis_fllow(void)
{
    float gimbal_angle,chassis_angle;
    chassis_angle   =   yaw.status.actual_angle;
    gimbal_angle    =   GIMBAL_HEAD_ANGLE+180.0f*chassis.parameter.invert_flag;
    PID_Calculate(&yaw_pid.angle_loop,chassis_angle,gimbal_angle);
    return yaw_pid.angle_loop.Output;
    
}

void Chassis_Mode_Command_Update(void)
{
		chassis.parameter.mode=connection.connection_rx.mode;
    switch(chassis.parameter.mode)
    {
        case    CHASSIS_REMOTE_CLOSE:
				case	CHASSIS_PRECISIOUS:
        chassis.command.vx =  0;
        chassis.command.vy =  0;
        chassis.command.vw =  0;
        break;
        case    CHASSIS_NORMAL:
        if(chassis.parameter.follow_switch	==	1)
				{
				chassis.command.vw =  chassis.command.vw-1.0f*chassis_fllow();
				}
				else
				{
				chassis.command.vw =  0;
				}
				
        break;
        case    CHASSIS_SPIN:
        if(chassis.command.vx==0&&chassis.command.vy==0)
        chassis.command.vw =  3.0f;
        else chassis.command.vw =  2.0f;
        break;

    }
}

void Chassis_Init(void)
{
    chassis.parameter.mode =   CHASSIS_NORMAL;
    chassis.parameter.invert_flag =  1;//1:正转，0：反转
    chassis.parameter.break_mode    =   1;
		chassis.parameter.speed_slow	=	1;
    chassis.parameter.relative_angle    =   0.f;
		chassis.A_motor.zero_position = 0x1202;
		chassis.B_motor.zero_position = 0x174c;
		chassis.C_motor.zero_position = 0x0ACE;
		chassis.D_motor.zero_position = 0x19CB;
		chassis.A_motor.ID	=	0x1a;
		chassis.B_motor.ID	=	0x1b;
		chassis.C_motor.ID	=	0x1c;
		chassis.D_motor.ID	=	0x1d;
	
}

void Yaw_Init(void)
{
    yaw.motor.parameter.calibrate_state	=	1;
		yaw.parameter.number_ratio = 2.0f;
    PID_Init(&yaw_pid.angle_loop,yaw_position_loop_data,NONE);
}

void Chassis_Move(void)
{   
    Chassis_Inveter_Judge();
    Gimbal_To_Chassis_Relative_Angle_Update();
    Chassis_Mode_Command_Update();
    Chassis_Speed_Control(&chassis);
}