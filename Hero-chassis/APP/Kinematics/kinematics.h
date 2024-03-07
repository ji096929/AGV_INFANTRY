#ifndef __KINEMATICS_H
#define __KINEMATICS_H

#include "chassis_move.h"
#include <stdlib.h>
#define wheel_diameter  10.000000f			//轮子直径
#define half_width    17.745000f                           //25.000000f		//底盘半宽
#define half_length   21.815000f                           //35.000000f		//底盘半长

#define PI 			3.141593f
#define PI2     2*PI
#define RPM2RAD 0.104720f										//转速转角速度		1 rpm = 2pi/60 rad/s 
#define RPM2VEL 0.523599f										//转速转线速度		vel = rpn*pi*D/60  cm/s
#define VEL2RPM 1.909859f										//线速度转转度
#define M2006_REDUCTION_RATIO 36.000000f		//齿轮箱减速比
#define M3508_REDUCTION_RATIO 19.000000f		//齿轮箱减速比
#define GM6020_ENCODER_ANGLE  8192.0f


#define MAX_MOTOR_SPEED   300				//电机最大转速，宏定义方便修改   范围0 - 10000   15336   
#define MAX_BASE_LINEAR_SPEED    120.817f    //底盘最大平移速度，单位cm/s   
#define MAX_BASE_ROTATIONAL_SPEED    7.260570f    //底盘最大旋转速度，单位rad/s    
#define NORMAL_LINEAR_SPEED          70.0f
#define NORMAL_ROTATIONAL_SPEED      0.5f


#define set_chassis_speed(motor1_speed,motor2_speed,motor3_speed,motor4_speed,motorA_speed,motorB_speed,motorC_speed,motorD_speed) \
        do{                                                                    \
			Chassis_Motor1.pid.speed_loop.vpid.Target_Speed = motor1_speed;		                         \
	        Chassis_Motor2.pid.speed_loop.vpid.Target_Speed = motor2_speed;                             \
	        Chassis_Motor3.pid.speed_loop.vpid.Target_Speed = motor3_speed;                             \
	        Chassis_Motor4.pid.speed_loop.vpid.Target_Speed = motor4_speed;                             \
	        Chassis_Motor1.Target_Speed = motor1_speed;		                               \
	        Chassis_Motor2.Target_Speed = motor2_speed;                                  \
	        Chassis_Motor3.Target_Speed = motor3_speed;                                  \
	        Chassis_Motor4.Target_Speed = motor4_speed;                                  \
            if (Chassis_Type == AGV_Chassis) \
            { \
                Chassis_MotorA.pid.speed_loop.vpid.Target_Speed = motorA_speed;		                         \
                Chassis_MotorB.pid.speed_loop.vpid.Target_Speed = motorB_speed;                             \
                Chassis_MotorC.pid.speed_loop.vpid.Target_Speed = motorC_speed;                             \
                Chassis_MotorD.pid.speed_loop.vpid.Target_Speed = motorD_speed;                             \
                Chassis_MotorA.Target_Speed = motorA_speed;		                               \
                Chassis_MotorB.Target_Speed = motorB_speed;                                  \
                Chassis_MotorC.Target_Speed = motorC_speed;                                  \
                Chassis_MotorD.Target_Speed = motorD_speed;                                  \
            }    \
		}while(0)                                                              \

				
				
typedef struct
{
	float linear_vel;			//线速度
	float rpm;						//转速圈每分钟
}Speed_t;

typedef struct
{
	float Angle;			//角度值
}Pos_t;

typedef struct
{
	Speed_t Target_Speed;			
	Speed_t actual_speed;
	Pos_t   Target_Angle;
	Pos_t   actual_angle;
}Wheel_t;

//底盘几何中心的线/角速度
typedef struct
{
	float linear_x;	//m/s
	float linear_y;
	float angular_z; //角速度rpm
}Velocities_t;

typedef struct
{
	float target_angular;
	float actual_angular;
	float Target_Angle;
	float actual_angle;
}Application_t;

typedef struct
{
	Wheel_t Wheel_1;
	Wheel_t Wheel_2;
	Wheel_t Wheel_3;
	Wheel_t Wheel_4;
	
	Wheel_t Wheel_A;
	Wheel_t Wheel_B;
	Wheel_t Wheel_C;
	Wheel_t Wheel_D;
	
	Velocities_t target_velocities;		//目标线速度
	Velocities_t actual_velocities; 	//实际线速度
}Kinematics_t;

extern Kinematics_t Kinematics;


void Mecanum_Wheel_BaseVel_To_WheelVel(float linear_x, float linear_y, float angular_z);
void AGV_Vector_Composition_In_ChassisCoordinate(float linear_x, float linear_y, float angular_z, float Theta, float Motion_Angle);
float AGV_DirectiveMotor_RobotMotion_To_TargetStatus(float linear_x, float linear_y, float Theta);
void Directive_Motor_Angle_Optimize(MOTOR_t *Motor);
void AGV_DirectiveMotor_TargetStatus_To_MotorAngle_In_ChassisCoordinate(int break_mode);
void AAA_AGV_DirectiveMotor_TargetStatus_To_MotorAngle(MOTOR_t *Motor, float Target_Motion_Angle, int Spining_Mode);

void Get_Base_Velocities(void);
int find_max(void);
void Speed_Limitation(void);

#endif
