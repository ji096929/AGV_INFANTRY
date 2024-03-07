/**
  ******************************************************************************
  * @file    Project/APP/chassis_task.c 
  * @author  NEUQRM
  * @version V1.0.0
  * @date    12.2021
  * @brief   该文件是战车输出任务汇总
  ******************************************************************************
  * @attention
  ******************************************************************************
  ****************************(C) COPYRIGHT 2021 NEUQRM****************************

  ...........  革命尚未成功，同志仍需努力  ...........
*/

#include "chassis_task.h"
#include "bsp_can.h"
#include "supercap.h"
#include "referee_UI.h"
#include "bsp_uart.h"
#include "relay_task.h"

int MS_Count=0;
int S_Count=0;

void Time_Service_Task(void);
/***********************************************************
*@fuction	:CHASSIS_TASK
*@brief		:
*@param		:--
*@return	:void
*@author	:--
*@date		:2023-04-04
***********************************************************/

void CHASSIS_TASK()//TIM3定时器中断控制战车（已经过宏定义）
{

	Time_Service_Task();
	if(MS_Count%5==0)
	{
		remote_control(); 				//遥控数据接收
		chassis_move();   				//底盘移动任务
		referee_unpack_fifo_data();	//裁判系统数据接收
	}
	if(MS_Count%5==2)
	{
		referee_usart_task();
		
	}	
	if(MS_Count%10==3)
	{
	trigger_turn();
	}
	
	 AGV_connoection(MS_Count);

//	Relay_Task(S_Count, MS_Count);	
	
	supercap(S_Count, MS_Count);

}

void Time_Service_Task(void)
{
	MS_Count++;
	if(MS_Count>999)			
	{
		MS_Count=0;
		S_Count++;
	}
}
