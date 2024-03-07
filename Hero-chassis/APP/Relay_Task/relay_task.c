#include "relay_task.h"
#include "gpio.h"

/***********************************************************
*@Brief	供外部调用的变量
***********************************************************/
int Relay_Set_State = Supercap_Power_Supply;

/***********************************************************
*@Brief	relay_task.c内部的变量
***********************************************************/
int Relay_State = Supercap_Power_Supply;

/***********************************************************
*@Brief	relay_task.c函数顺序重定义
***********************************************************/
void Supercap_Power_Mode(void);
void Battery_Power_Mode(void);

void Relay_Task(int S_Cnt, int MS_Cnt)
{
	//时间服务
	int Global_Time;
	Global_Time = S_Cnt*1000 + MS_Cnt;
	
	//继电器控制，100ms进行一次
	if (Global_Time%300 == 0)  //确保继电器模式切换间隔大于300ms，防止误触
	{
		Relay_State = Relay_Set_State;
		switch(Relay_State)
		{
			case Supercap_Power_Supply:
				Supercap_Power_Mode();
				break;
			case Direct_Power_Supply:
				Battery_Power_Mode();
				break;
		}
	}
}
	
void Supercap_Power_Mode(void)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
}

void Battery_Power_Mode(void)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
}