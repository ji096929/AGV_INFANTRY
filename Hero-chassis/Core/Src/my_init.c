/**
  ******************************************************************************
  * @file    Project/Application/User/Core/my_init.c 
  * @author  NEUQRM
  * @version V1.0.0
  * @date    12.2021
  * @brief   所有初始化函数调用，一并进行初始化
  ******************************************************************************
  * @attention
  ******************************************************************************
  ****************************(C) COPYRIGHT 2021 NEUQRM****************************

  ...........  革命尚未成功，同志仍需努力  ...........
*/
#include "my_init.h"

steering_communication_pack_t pack_init;

void mode_init(void)
{
	chassis_center.pid.loop_flag=POSITION_LOOP;
	Chassis_Motor1.pid.loop_flag=SPEED_LOOP;
	Chassis_Motor2.pid.loop_flag=SPEED_LOOP;
	Chassis_Motor3.pid.loop_flag=SPEED_LOOP;
	Chassis_Motor4.pid.loop_flag=SPEED_LOOP;
    Chassis_MotorA.pid.loop_flag=POSITION_LOOP;
	Chassis_MotorB.pid.loop_flag=POSITION_LOOP;
	Chassis_MotorC.pid.loop_flag=POSITION_LOOP;
	Chassis_MotorD.pid.loop_flag=POSITION_LOOP;//POSITION_LOOP
	
	Chassis_MotorA.Invert_Flag = 1;
	Chassis_MotorB.Invert_Flag = 1;
	Chassis_MotorC.Invert_Flag = 1;
	Chassis_MotorD.Invert_Flag = 1;
}
void all_init(void)
{
	motor_pid_init(&(chassis_center.pid));
	motor_pid_init(&(Chassis_Motor1.pid));
	motor_pid_init(&(Chassis_Motor2.pid));
	motor_pid_init(&(Chassis_Motor3.pid));
	motor_pid_init(&(Chassis_Motor4.pid));
    
    motor_pid_init(&(Chassis_MotorA.pid));
	motor_pid_init(&(Chassis_MotorB.pid));
	motor_pid_init(&(Chassis_MotorC.pid));
	motor_pid_init(&(Chassis_MotorD.pid));
	pid_init();
	steering_communication_init();
	trigger_all_init();
	referee_usart_fifo_init();
	POWER_PID_Init(&p_pid);//目前无用
	remote_control_init();
	bsp_can_init();
	steering_communication_SET_VELOCITY_VECTOR(A_MscB, Chassis_MotorA.Target_Angle, 0, chassis_power_control.scaled_power_32[0], pack_init);
	HAL_Delay(1);
  steering_communication_SET_VELOCITY_VECTOR(B_MscB, Chassis_MotorB.Target_Angle, 0, chassis_power_control.scaled_power_32[1], pack_init);
	HAL_Delay(1);
	steering_communication_SET_VELOCITY_VECTOR(C_MscB, Chassis_MotorC.Target_Angle, 0, chassis_power_control.scaled_power_32[2], pack_init);
	HAL_Delay(1);
  steering_communication_SET_VELOCITY_VECTOR(D_MscB, Chassis_MotorD.Target_Angle, 0, chassis_power_control.scaled_power_32[3], pack_init);
	HAL_Delay(1);
	init_referee_struct_data();
	Yaw_PIDinit();
	uart_init();
	HAL_TIM_Base_Start_IT(&htim3);
	
	
////	Gimbal_Init();
}

void Motor_Zero_Position_Init(void)//CW增大，CCW减小
{
	//适用于布瑞特编码器的转向舵零点位置
	Chassis_MotorA.Zero_Position = 0x1202;
	Chassis_MotorB.Zero_Position = 0x174c;
	Chassis_MotorC.Zero_Position = 0x0ACE;
	Chassis_MotorD.Zero_Position = 0x19CB;
}