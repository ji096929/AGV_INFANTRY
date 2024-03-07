/**
  ************************************* Copyright ******************************   
  *                 (C) Copyright 2022,Hebei,China, NEUQRM.
  *                            All Rights Reserved
  *                              
  *                     By DGYin,
  *                     https://--
  *      
  * FileName   : chassis_move.h
  * Version    : v1.1.0-alpha.1
  * Author     : NEUQRM, DGYin
  * Date       : 2022-11-07         
  * Description: 头文件   
  ******************************************************************************
**/
#ifndef __MOTOR_H
#define __MOTOR_H

#include "pid.h"

#include "can.h"
#include <math.h>
#include "referee.h"


/*******************************************************************************  
                                  在这里配置底盘类型
*******************************************************************************/
#define Mecanum_Wheel_Chassis 0	//麦轮底盘
#define AGV_Chassis						1	//舵轮底盘
#define Chassis_Type AGV_Chassis

//也可由云台直接传输指令
#define CHASSIS_REMOTE_CLOSE		1	//关闭遥控器                                左高位
#define CHASSIS_NORMAL				3	//正常模式                                  左中位
#define CHASSIS_SPIN				2	//小陀螺模式                                    左低位

#define Chassis_Follow_ON		1
#define Chassis_Follow_OFF		0

#define GIMBAL_HEAD_ANGLE  7

#define POWER_LIMIT         80.0f
#define WARNING_POWER       40.0f   
#define WARNING_POWER_BUFF  50.0f  

/*Motor_Power_Prosumption、Motor_Power_Limitation函数相关的宏定义*/
#define Tau_Coefficient 0.3 //请查阅3508手册 https://rm-static.djicdn.com/tem/17348/RoboMaster%20M3508%E7%9B%B4%E6%B5%81%E6%97%A0%E5%88%B7%E5%87%8F%E9%80%9F%E7%94%B5%E6%9C%BA%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8EV1.0%EF%BC%88%E4%B8%AD%E8%8B%B1%E6%97%A5%EF%BC%89.pdf
#define Power_K1 0
#define Power_K2 0
#define Power_Prosumption_Converting_Coefficient 0

/*Target_Velocity_Smoothen函数相关的宏定义*/
#define Uniform_Acceleration 0 //匀加速模式，用于平滑速度曲线
#define Acceleration 0 //加速度，用于平滑速度曲线的匀加速模式

#define Directive_Motor_Max_Err_Angle 80000

#define follow_pid_realize() \
				do{ \
						switch_flag=FOLLOW;      \
						pid_realize(&(chassis_center.pid));   \
					  switch_flag=NUL;  \
				}while(0)
				
#define vpid_chassis_realize() \
				do{ \
						switch_flag=CHASSIS;      \
						pid_realize(&(Chassis_Motor1.pid));   \
						pid_realize(&(Chassis_Motor2.pid));   \
						pid_realize(&(Chassis_Motor3.pid));   \
						pid_realize(&(Chassis_Motor4.pid));   \
					  switch_flag=NUL;  \
				}while(0)
				

			
typedef struct{
	
	float start_angle;			//电机初始角度值
	int Zero_Position;
	int start_angle_flag;	//记录电机初始角度值的flag
	int switch_mode_flag;  //记录模式转换角度值的flag
	int stop_angle;				//发送停止命令时候的角度值
	float Last_Target_Angle;
	float Target_Angle;
	float Find_Min;
	
	int Start_Angle;
	float actual_angle;			//当前真实角度值
	float last_angle;				//上一次返回的角度值
	float switch_mode_angle;  //记录模式转换角度值
	int round_cnt;				//相对开机时转过的圈数
	int Total_Angle;			//总共转过的计数
	int Last_Total_Angle;
	float ChassisCoordinate_Angle;
	float AGV_Heading_Angle; //
	
	float actual_speed;			//电机真实速度,rpm
	int Target_Speed;			//电机目标速度,rpm  转/min
	int last_speed;       //电机上一次回传的速度值
	int actual_current;		//电机真实电流
	int target_current;		//电机目标电流
	float Actual_Torque;
	float Target_Torque;
	//int temp;							//电机温度（2006电机不支持，3508支持）
	
	int Invert_Flag;
	float Power_Prosumption;
	
	PID_t pid;
}MOTOR_t;

typedef struct
{
	int16_t vx_set;
	int16_t vy_set;
	int16_t wz_set;
	uint8_t chassis_mode;
	int Precision_Mode;
	uint8_t last_chassis_mode;
	uint16_t gimbal_6020_angle;
}CHASSIS_CONTROL_ORDER_t;

typedef struct
{
	int16_t real_vx;
	int16_t real_vy;
}REAl_CHASSIS_SPEED_t;

typedef enum
{
	NO_STEP=0,
	X_STEP,
	Y_STEP,	
	XY_STEP,
}STEPSTAR;


extern MOTOR_t Chassis_Motor1,Chassis_Motor2,Chassis_Motor3,Chassis_Motor4,chassis_center;
extern MOTOR_t Chassis_MotorA,Chassis_MotorB,Chassis_MotorC,Chassis_MotorD;
extern CHASSIS_CONTROL_ORDER_t chassis_control_order;
extern POWER_PID_t p_pid;
extern BUFFER_PID_t b_pid;
extern int shoot_flag;
extern int Chassis_Follow_Switch;
void AGV_connoection(int ms_cnt);
float Square(float Input);
void vpid_chassis_realize_F(void);
void chassis_move(void);
int All_Directive_Motor_Angle_Ready(void);
#endif

