/*
 * @Author: ShenYuJie 1647262812@qq.com
 * @Date: 2024-03-11 00:37:34
 * @LastEditors: ShenYuJie 1647262812@qq.com
 * @LastEditTime: 2024-03-11 01:21:05
 * @FilePath: \MDK-ARMd:\EVENTS\2024_Hero\GIMBAL\app\can_connection.c
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
#include "can_connection.h"
#include "fric.h"
#include "remote_control.h"
#include "gimbal.h"
#include "trigger.h"
CHASSIS_T chassis;

//底盘通信数据初始化
void Chassis_Connection_Init(void)
{
	chassis.send.follow_flag	=	1;
	chassis.send.invert_flag	=	0;
	chassis.send.mode	=	CHASSIS_MODE_NOFORCE;
}
//地盘速度指令更新
void Chassis_Speed_Update(void)
{
    switch(chassis.send.mode)
    {
        case CHASSIS_MODE_TOPANGLE:
        case    CHASSIS_MODE_ABSOLUTE:
        case    CHASSIS_MODE_PRECISE:
        chassis.send.velocity.vx   =   RC.rc_sent.x_speed;
        chassis.send.velocity.vy   =   RC.rc_sent.y_speed;
        chassis.send.velocity.vw   =   RC.rc_sent.r_speed;
        break;
        case CHASSIS_MODE_NOFORCE:
         chassis.send.velocity.vx   =   0;
        chassis.send.velocity.vy   =   0;
        chassis.send.velocity.vw   =   0;
        break;

    }
}
//底盘发送内容缓冲区更新
void Chassis_Send_data_Update( void)
{
    memset(CAN2_0x150_Tx_Data,0,8);
    memset(CAN2_0x152_Tx_Data,0,8);

    CAN2_0x150_Tx_Data[0] = (int16_t)chassis.send.velocity.vx >> 8;
    CAN2_0x150_Tx_Data[1] = (int16_t)chassis.send.velocity.vx;
    CAN2_0x150_Tx_Data[2] = (int16_t)chassis.send.velocity.vy >> 8;
    CAN2_0x150_Tx_Data[3] = (int16_t)chassis.send.velocity.vy;
    CAN2_0x150_Tx_Data[4] = (int16_t)chassis.send.velocity.vw >> 8;
    CAN2_0x150_Tx_Data[5] = (int16_t)chassis.send.velocity.vw;

    CAN2_0x152_Tx_Data[0] = chassis.send.mode;
//		CAN2_0x152_Tx_Data[0] =0	;
    CAN2_0x152_Tx_Data[1] = chassis.send.invert_flag;
    CAN2_0x152_Tx_Data[2] = chassis.send.follow_flag;
		CAN2_0x152_Tx_Data[3]	=	gimbal.yaw.motor.parameter.calibrate_state;
    

}
//can发送任务
void Can_Send_Task(int8_t ms_count)
{
  

		

	CAN_Send_Data(&hcan1,0x1ff,CAN1_0x1ff_Tx_Data,8);
	CAN_Send_Data(&hcan2,0x1ff,CAN2_0x1ff_Tx_Data,8);
		
	CAN_Send_Data(&hcan1,0x200,CAN1_0x200_Tx_Data,8);
		
    if(ms_count	%5	==1)
		{
	CAN_Send_Data(&hcan2,0x150,CAN2_0x150_Tx_Data,8);
    CAN_Send_Data(&hcan2,0x152,CAN2_0x152_Tx_Data,8);
		}
		
    CAN_Send_Data(&hcan2,0x200,CAN2_0x200_Tx_Data,8);
    
}
//底盘通信任务
void Chassis_Connection_Task(void)
{
    Chassis_Speed_Update();
    Chassis_Send_data_Update();
}
//can1中断回调
void CAN1_Call_Back(struct Struct_CAN_Rx_Buffer *rx)
{
    switch(rx->Header.StdId)
    {
        case 0x202:
        M3508_Feedback_Update(&fric.left_motor.motor,rx->Data);
        break;

        case 0x201:
        M3508_Feedback_Update(&fric.right_motor.motor,rx->Data);
        break;

        case 0x205:
       
        GM6020_Feedback_Update(&gimbal.pitch.motor,rx->Data);
        break;
    }
     switch(rx->Header.ExtId)
    {
        
    }
}
//can2中断回调
void CAN2_Call_Back(struct Struct_CAN_Rx_Buffer *rx)
{
    switch(rx->Header.StdId)
    {
        case 0x206:
        GM6020_Feedback_Update(&gimbal.yaw.motor,rx->Data);
				
        break;

        case 0x201:
        M3508_Feedback_Update(&trigger.motor,rx->Data);
        break;

        case 0x151:
       
        
        break;
        case 0x153:
       
        
        break;
    }
    switch(rx->Header.ExtId&0xff)
    {

    }
}

//can配置初始化
void Can_Connection_Init(void)
{
    CAN_Init(&hcan1, CAN1_Call_Back);
    CAN_Init(&hcan2, CAN2_Call_Back); 

}