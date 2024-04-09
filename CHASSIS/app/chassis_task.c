/*
 * @Author: ShenYuJie 1647262812@qq.com
 * @Date: 2024-03-10 15:58:37
 * @LastEditors: ShenYuJie 1647262812@qq.com
 * @LastEditTime: 2024-03-10 16:48:03
 * @FilePath: \MDK-ARMd:\EVENTS\2024_Hero\CHASSIS\CHASSIS\app\chassis_task.c
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
#include "chassis_task.h"
#include "GraphicsSendTask.h"
TIME_T time;

void Task_Init(void)
{
    Yaw_Init();
    Chassis_Init();
    Can_Connection_Init();
		Chassis_Power_Control_Init();
		Referee_Init();
	//buzzer_init_example();
}
   

void Time_Count_Task(TIME_T *time)
{
    time->ms_count++;
    if(time->ms_count>999)
    {
        time->ms_count=0;
        time->s_count++;
    }
    time->total_count=time->s_count*60+time->ms_count;
}

void Chassis_Task()
{
    if(htim->Instance==TIM3)
    {
    AGV_connoection(time.ms_count);   
 GraphicSendtask();	    
    if(time.ms_count%5==0)
     {
       Chassis_Move();			
     }		 
		if(time.ms_count%20==17)
		{
		calculate_true_power();
		}
		if(time.ms_count%5==1)
		{
			GM6020_Status_Update(&yaw.motor);
      Yaw_Angle_Process(&yaw);
		}     
    if(time.ms_count%10==4)
    {
      referee_unpack_fifo_data();
    }
    if (time.ms_count % 10 == 5)
    {
      CAN_Chassis_TxCpltCallback();
    }
    if(time.ms_count%10==6)
    {
    TIM_CAN_PeriodElapsedCallback();
    }
		if(time.ms_count%10==8)
    {

			Chassis_Flag_Update(&connection);
    }

        Time_Count_Task(&time);
    }
}

