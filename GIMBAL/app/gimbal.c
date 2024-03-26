/*
 * @Author: ShenYuJie 1647262812@qq.com
 * @Date: 2024-02-23 16:39:45
 * @LastEditors: ShenYuJie 1647262812@qq.com
 * @LastEditTime: 2024-03-02 16:22:17
 * @FilePath: \MDK-ARMd:\EVENTS\2024_Hero\GIMBAL\Core\gimbal\gimbal.c
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
#include "gimbal.h"

#include 	"remote_control.h"

#include 	"bsp_imu.h"

#include 	"task_schedule.h"

#include    "drv_can.h"

#include 	"vision.h"

#include "math.h"

#define pitch_test



GIMBAL_T gimbal;

/*****************************************pitch轴PID*****************************************************/
float gimbal_pitch_encoder_speed_data[PID_DATA_LEN]
	={700.0f,0.1f,0.0f,20000.0f,15000.0f,0.0f,50.0f,10.0f,0.7f,0.0f};
float gimbal_pitch_encoder_position_data[PID_DATA_LEN]
	={9.5f,0.0f,0.2f,150.0f,0.0f,0.00f,0.5f,0.1f,0.5f,0.0f};
float gimbal_pitch_imu_speed_data[PID_DATA_LEN]
	={7000.0f,28.0f,0.0f,25000.0f,10000.0f,0.0f,1000.0f,100.0f,0.7f,0.0f};
float gimbal_pitch_imu_position_data[PID_DATA_LEN]
	={1.2f,0.0f,15.0f,4.0f,0.0f,0.01f,0.5f,0.1f,0.5f,0.0f};

//Kp,Ki,Kd,MaxOut,Inter_limition,Deadband,Change_I_A,Change_I_B,OUTPUT_FILTER,DOUTPUT_FILTER
/*****************************************yaw轴PID*****************************************************/
float gimbal_yaw_encoder_speed_data[PID_DATA_LEN]
	={430.0f,0.3f,0.0f,25000.0f,7500.0f,0.0f,50.0f,10.0f,0.5f,0.0f};
float gimbal_yaw_encoder_position_data[PID_DATA_LEN]
	={10.0f,0.2f,1.0f,150.0f,20.0f,0.00f,10.0f,2.0f,0.5f,0.0f};
float gimbal_yaw_imu_speed_data[PID_DATA_LEN]
	={18000.0f,20.0f,0.0f,25000.0f,1000.0f,0.01f,0.5f,0.1f,0.5f,0.0f};
float gimbal_yaw_imu_position_data[PID_DATA_LEN]
	={0.35f,0.1f,1.0f,10.0f,0.01f,0.04f,1.0f,0.5f,0.5f,0.0f};


/*******************************质心补偿参数******************************************/
#define L_gravity 	0.03791	//m
#define PI_DIV_180 (0.017453292519943296)//π/180
#define DegToRad(x)	((x)*PI_DIV_180)//角度转换为弧度
#define gravity	9.8f
#define M_gravity 2.831f
#define KT_GM	0.00019836
#define KB_GM	0.10979

int16_t Pitch_Gravity_Compensation(void)
{
	//计算重力校正后的俯仰角
	double T_gravity,Theate_change,Theate_gravity,L_actual_gravity;
	int16_t V_GM;
	//计算俯仰角变化量
	Theate_change	=	DegToRad(gimbal.pitch.imu.status.actual_angle);
	//计算重力角度
	Theate_gravity	=	DegToRad(61.23);
	//计算实际重力角度
	L_actual_gravity	=	L_gravity*cos(Theate_gravity-Theate_change);
	//计算重力校正所需力矩
	T_gravity	=L_actual_gravity	*M_gravity*gravity -KB_GM;
	//将计算重力校正所需力矩转换为电机电压
	V_GM	=	(int16_t)(T_gravity/KT_GM);
	//返回电机电压
	return	V_GM;
	
}
//云台状态更新
void Gimbal_Statue_Update(void)
{
    //pitch轴数据多圈处理
	GM6020_Status_Update(&gimbal.pitch.motor);
    gimbal.pitch.imu.status.actual_angle =   1.0f * INS_angle[1] / (2 * 3.141590f) * 360.0f;
    gimbal.pitch.imu.status.actual_speed = -1.0f * INS_gyro[0] +0.023f;	    
    gimbal.pitch.status.total_angle =	gimbal.pitch.motor.status.total_position_degree/gimbal.pitch.parameter.number_ratio;
	while(gimbal.pitch.status.total_angle-gimbal.pitch.status.rounds*360.0f>180.0f) gimbal.pitch.status.rounds++;
	while(gimbal.pitch.status.total_angle-gimbal.pitch.status.rounds*360.0f<-180.0f) gimbal.pitch.status.rounds--;
    //pitch轴状态更新
	switch(gimbal.pitch.parameter.mode)
    {
        case   ENCODER_MODE :
       
        gimbal.pitch.status.actual_angle    =   gimbal.pitch.status.total_angle - gimbal.pitch.status.rounds*360.0f;
		gimbal.pitch.status.actual_speed	=	gimbal.pitch.motor.status.velocity_rpm;

        break;
        case  IMU_MODE :
            
            gimbal.pitch.status.actual_angle = gimbal.pitch.imu.status.actual_angle;
            gimbal.pitch.status.actual_speed = gimbal.pitch.imu.status.actual_speed;

        break;
    }
		chassis.send.pitch_angle=gimbal.pitch.status.actual_angle;
    //yaw轴数据多圈处理
			GM6020_Status_Update(&gimbal.yaw.motor);
			gimbal.yaw.imu.status.last_actual_angle = gimbal.yaw.imu.status.actual_angle;
            gimbal.yaw.imu.status.actual_angle = -INS_angle[0] / (2 * 3.141590f) * 360.0f;
            if(gimbal.yaw.imu.status.actual_angle-gimbal.yaw.imu.status.last_actual_angle>180.0f) gimbal.yaw.imu.status.rounds--;
            if(gimbal.yaw.imu.status.actual_angle-gimbal.yaw.imu.status.last_actual_angle<-180.0f)gimbal.yaw.imu.status.rounds++;

            gimbal.yaw.imu.status.total_angle = gimbal.yaw.imu.status.rounds*360.0f + gimbal.yaw.imu.status.actual_angle;
            gimbal.yaw.imu.status.actual_speed = -1.0f * INS_gyro[2]+0.09f;
			gimbal.yaw.status.total_angle =	gimbal.yaw.motor.status.total_position_degree/gimbal.yaw.parameter.number_ratio;
		    while(gimbal.yaw.status.total_angle-gimbal.yaw.status.rounds*360.0f>180.0f) gimbal.yaw.status.rounds++;
		    while(gimbal.yaw.status.total_angle-gimbal.yaw.status.rounds*360.0f<-180.0f) gimbal.yaw.status.rounds--;
    //yaw轴数据更新
		switch(gimbal.yaw.parameter.mode)
    {
         case   ENCODER_MODE :
            
           
            gimbal.yaw.status.actual_angle    =   gimbal.yaw.status.total_angle - gimbal.yaw.status.rounds*360.0f;
			gimbal.yaw.status.actual_speed	=	gimbal.yaw.motor.status.velocity_rpm;

        break;
        case  IMU_MODE :
            

            gimbal.yaw.status.actual_angle = gimbal.yaw.imu.status.actual_angle;
						
            gimbal.yaw.status.actual_speed = gimbal.yaw.imu.status.actual_speed;
				
			gimbal.yaw.status.total_angle	=	gimbal.yaw.imu.status.total_angle;

        break;
    }
}


//云台控制指令更新
void Gimbal_Command_Update(void)
{
    //判断自瞄状态
    switch(vision_control.mode)
    {
        case VISION_OFF :
            switch (gimbal.parameter.mode)
            {
            case GIMBAL_MODE_PRECISION :
                gimbal.yaw.command.add_angle=RC.rc_sent.yaw.target_angle/6.0f;
                gimbal.pitch.command.add_angle=RC.rc_sent.pitch.target_angle/4.0f;
                break;
            case GIMBAL_MODE_TOPANGLE :
                case GIMBAL_MODE_ABSOLUTE :
                gimbal.yaw.command.add_angle=RC.rc_sent.yaw.target_angle/9.0f;
                gimbal.pitch.command.add_angle=RC.rc_sent.pitch.target_angle/6.0f;
                break;
            default:
                gimbal.yaw.command.add_angle=0.0f;
                gimbal.pitch.command.add_angle=0.0f;
                break;
            }
						gimbal.yaw.command.target_angle+=gimbal.yaw.command.add_angle;
			
						gimbal.pitch.command.target_angle+=gimbal.pitch.command.add_angle;
        break;
        case VISION_ON  :
            switch (gimbal.parameter.mode)
            {
            case GIMBAL_MODE_PRECISION :
                gimbal.yaw.command.target_angle=vision_control.command.yaw_angle;
                gimbal.pitch.command.target_angle=vision_control.command.pitch_angle;
                break;
            case GIMBAL_MODE_TOPANGLE :
                case GIMBAL_MODE_ABSOLUTE :
               gimbal.yaw.command.target_angle=vision_control.command.yaw_angle;
                gimbal.pitch.command.target_angle=vision_control.command.pitch_angle;
                break;
            default:

                break;
            }
        break;
    }

}

float test;
void Gimbal_Motor_Command_Update(void)
{
    //根据参数模式，计算出目标角度和速度
    switch(gimbal.pitch.parameter.mode)
    {
				if(gimbal.pitch.command.target_angle<-39.5f) gimbal.pitch.command.target_angle=-39.5f;
				if(gimbal.pitch.command.target_angle>18.5f)	gimbal.pitch.command.target_angle=18.5f;
			case IMU_MODE :
				
				gimbal.pitch.motor.command.grivity_voltage_lsb=Pitch_Gravity_Compensation();		
        PID_Calculate(&gimbal.pitch.pid.imu_angle_loop,gimbal.pitch.status.actual_angle,gimbal.pitch.command.target_angle);
        //设置目标速度
        gimbal.pitch.command.target_speed = -gimbal.pitch.pid.imu_angle_loop.Output;
        //计算出输出速度
        PID_Calculate(&gimbal.pitch.pid.imu_speed_loop,gimbal.pitch.status.actual_speed,gimbal.pitch.command.target_speed);
        //设置输出力矩
        GM6020_Command_Update(&gimbal.pitch.motor,gimbal.pitch.pid.imu_speed_loop.Output);
				gimbal.pitch.motor.command.give_voltage_lsb	+=gimbal.pitch.motor.command.grivity_voltage_lsb;
				
								

        break;
    case ENCODER_MODE  :
			
				gimbal.pitch.motor.command.grivity_voltage_lsb=Pitch_Gravity_Compensation();
       PID_Calculate(&gimbal.pitch.pid.encoder_angle_loop,gimbal.pitch.status.actual_angle,gimbal.pitch.command.target_angle);
        //设置目标速度
        gimbal.pitch.command.target_speed = gimbal.pitch.pid.encoder_angle_loop.Output;
        //计算出输出速度
        PID_Calculate(&gimbal.pitch.pid.encoder_speed_loop,gimbal.pitch.status.actual_speed,gimbal.pitch.command.target_speed);
        //设置输出力矩
        GM6020_Command_Update(&gimbal.pitch.motor,-gimbal.pitch.pid.encoder_speed_loop.Output);
				gimbal.pitch.motor.command.give_voltage_lsb	+=gimbal.pitch.motor.command.grivity_voltage_lsb;
				
				
        break;
				if(gimbal.pitch.motor.command.give_voltage_lsb>25000)	gimbal.pitch.motor.command.give_voltage_lsb=25000;
				if(gimbal.pitch.motor.command.give_voltage_lsb<-25000)	gimbal.pitch.motor.command.give_voltage_lsb=-25000;
    }

    //根据参数模式，计算出目标角度和速度
    switch(gimbal.yaw.parameter.mode)
    {
				while(gimbal.yaw.command.target_angle-gimbal.yaw.status.total_angle>360.0f)	gimbal.yaw.command.target_angle-=360.0f;
				while(gimbal.yaw.command.target_angle-gimbal.yaw.status.total_angle<-360.0f)	gimbal.yaw.command.target_angle+=360.0f;
			case IMU_MODE :
       
        PID_Calculate(&gimbal.yaw.pid.imu_angle_loop,gimbal.yaw.status.total_angle,gimbal.yaw.command.target_angle);
        //设置目标速度
        gimbal.yaw.command.target_speed = gimbal.yaw.pid.imu_angle_loop.Output;
        //计算出输出速度
        PID_Calculate(&gimbal.yaw.pid.imu_speed_loop,gimbal.yaw.status.actual_speed,gimbal.yaw.command.target_speed);
        //设置输出力矩
        GM6020_Command_Update(&gimbal.yaw.motor,-gimbal.yaw.pid.imu_speed_loop.Output);

        break;
    case ENCODER_MODE  :
       PID_Calculate(&gimbal.yaw.pid.encoder_angle_loop,gimbal.yaw.status.total_angle,gimbal.yaw.command.target_angle);
        //设置目标速度
        gimbal.yaw.command.target_speed = gimbal.yaw.pid.encoder_angle_loop.Output;
        //计算出输出速度
        PID_Calculate(&gimbal.yaw.pid.encoder_speed_loop,gimbal.yaw.status.actual_speed,gimbal.yaw.command.target_speed);
        //设置输出力矩
        GM6020_Command_Update(&gimbal.yaw.motor,-gimbal.yaw.pid.encoder_speed_loop.Output);
        break;
    }
};

void Gimbal_Motor_Mode_Update(void)
{
    //根据gimbal模式设置电机模式
    switch (gimbal.parameter.mode)
    {
    case GIMBAL_MODE_NO_FORCE :
        //无力模式
        case GIMBAL_MODE_TOPANGLE :
            //小陀螺模式
            case GIMBAL_MODE_ABSOLUTE :
                //绝对模式
                case GIMBAL_MODE_CALI :
                //校准模式
                gimbal.yaw.parameter.mode = IMU_MODE;
                gimbal.pitch.parameter.mode = IMU_MODE;
        break;
    case GIMBAL_MODE_PRECISION :
        //精度模式
        gimbal.yaw.parameter.mode = ENCODER_MODE;
        gimbal.pitch.parameter.mode = ENCODER_MODE;


        break;

    }
};

void Gimbal_Cali_Task(void)
{
         gimbal.pitch.command.target_angle = 0.0f;
		gimbal.pitch.motor.command.grivity_voltage_lsb=Pitch_Gravity_Compensation();
        //计算出输出角度
        PID_Calculate(&gimbal.pitch.pid.imu_angle_loop,gimbal.pitch.status.actual_angle,gimbal.pitch.command.target_angle);
        //设置目标速度
        gimbal.pitch.command.target_speed = -gimbal.pitch.pid.imu_angle_loop.Output;
        //计算出输出速度
        PID_Calculate(&gimbal.pitch.pid.imu_speed_loop,gimbal.pitch.status.actual_speed,gimbal.pitch.command.target_speed);
        //设置输出力矩
        GM6020_Command_Update(&gimbal.pitch.motor,gimbal.pitch.pid.imu_speed_loop.Output);
				gimbal.pitch.motor.command.give_voltage_lsb	+=gimbal.pitch.motor.command.grivity_voltage_lsb;
				if(gimbal.pitch.motor.command.give_voltage_lsb>25000)	gimbal.pitch.motor.command.give_voltage_lsb=25000;
				if(gimbal.pitch.motor.command.give_voltage_lsb<-25000)	gimbal.pitch.motor.command.give_voltage_lsb=-25000;

};


//云台电机缓冲区更新
void Gimbal_Send_command_Update(void)
{
    if(gimbal.parameter.mode != GIMBAL_MODE_NO_FORCE)
		{
	memset(&CAN1_0x1ff_Tx_Data,0,8);
		CAN1_0x1ff_Tx_Data[0]=-gimbal.pitch.motor.command.give_voltage_lsb>>8;
		CAN1_0x1ff_Tx_Data[1]=-gimbal.pitch.motor.command.give_voltage_lsb;
    memset(&CAN2_0x1ff_Tx_Data,0,8);
        CAN2_0x1ff_Tx_Data[2]=-gimbal.yaw.motor.command.give_voltage_lsb>>8;
		CAN2_0x1ff_Tx_Data[3]=-gimbal.yaw.motor.command.give_voltage_lsb;
		}
		else
		{
			memset(&CAN1_0x1ff_Tx_Data,0,8);
			memset(&CAN2_0x1ff_Tx_Data,0,8);
		}
		
}

//云台初始化
void Gimbal_Init(void)
{
    GM6020_Init(&gimbal.pitch.motor,1,0.741f);
    GM6020_Init(&gimbal.yaw.motor,2,0.741f);

    gimbal.parameter.calibration_state = NO_CALIBRATION;
    gimbal.parameter.mode = GIMBAL_MODE_NO_FORCE;
		

    gimbal.pitch.parameter.number_ratio = 1.0f;
    gimbal.yaw.parameter.number_ratio = 2.0f;
	gimbal.pitch.motor.parameter.calibrate_state    =   MOTOR_NO_CALIBRATE;
    gimbal.yaw.motor.parameter.calibrate_state    =   MOTOR_NO_CALIBRATE;
    gimbal.pitch.parameter.mode =   IMU_MODE;
    gimbal.yaw.parameter.mode =     IMU_MODE;

    PID_Init(&gimbal.pitch.pid.encoder_angle_loop,gimbal_pitch_encoder_position_data,Integral_Limit|ChangingIntegralRate|Trapezoid_Intergral|Trapezoid_Intergral);
    PID_Init(&gimbal.pitch.pid.encoder_speed_loop,gimbal_pitch_encoder_speed_data,Integral_Limit|ChangingIntegralRate|Trapezoid_Intergral|Trapezoid_Intergral);
    PID_Init(&gimbal.pitch.pid.imu_angle_loop,gimbal_pitch_imu_position_data,Integral_Limit|ChangingIntegralRate|Trapezoid_Intergral|Trapezoid_Intergral);
    PID_Init(&gimbal.pitch.pid.imu_speed_loop,gimbal_pitch_imu_speed_data,Integral_Limit|ChangingIntegralRate|Trapezoid_Intergral|Trapezoid_Intergral);
    
    PID_Init(&gimbal.yaw.pid.encoder_angle_loop,gimbal_yaw_encoder_position_data,Integral_Limit|ChangingIntegralRate|Trapezoid_Intergral|Trapezoid_Intergral);
    PID_Init(&gimbal.yaw.pid.encoder_speed_loop,gimbal_yaw_encoder_speed_data,Integral_Limit|ChangingIntegralRate|Trapezoid_Intergral|Trapezoid_Intergral);
    PID_Init(&gimbal.yaw.pid.imu_angle_loop,gimbal_yaw_imu_position_data,Integral_Limit|ChangingIntegralRate|Trapezoid_Intergral|Trapezoid_Intergral);
    PID_Init(&gimbal.yaw.pid.imu_speed_loop,gimbal_yaw_imu_speed_data,Integral_Limit|ChangingIntegralRate|Trapezoid_Intergral|Trapezoid_Intergral);
}

void Gimbal_Task(void)
{
    switch (gimbal.parameter.calibration_state)
    {
    case NO_CALIBRATION :
        delay_time.gimbal_cali_cnt=2000;
        gimbal.parameter.mode =GIMBAL_MODE_CALI;
        Gimbal_Motor_Mode_Update();
        gimbal.parameter.calibration_state =    CALIBRATING;
        break;
    case CALIBRATING :
        delay_time.gimbal_cali_cnt--;
        Gimbal_Cali_Task();
				if(delay_time.gimbal_cali_cnt==100)
        chassis.send.mode	=	CHASSIS_MODE_ABSOLUTE;
        break;
    case CALIBRATED :
        
        gimbal.parameter.calibration_state =    NORMAL;
				delay_time.gimbal_cali_cnt=0;
				gimbal.parameter.mode = GIMBAL_MODE_ABSOLUTE;
				
        gimbal.pitch.motor.parameter.calibrate_state    =   MOTOR_CALIBRATED;
        gimbal.yaw.motor.parameter.calibrate_state    =   MOTOR_CALIBRATED;
        break;
    case NORMAL :
		if(gimbal.parameter.mode != GIMBAL_MODE_NO_FORCE)
			{
			if(gimbal_time.ms_count%5==1)
			{
				Gimbal_Motor_Mode_Update();
				Gimbal_Command_Update();
			}
        Gimbal_Motor_Command_Update();
			}
        
        break;
    }
		Gimbal_Send_command_Update();
    Gimbal_Statue_Update();

}; 

	
