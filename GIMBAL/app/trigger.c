#include "trigger.h"
#include "math.h"
TRIGGER_T trigger;

float Trigger_Angle_Loop_Data[PID_DATA_LEN]
={30.0f,1.0f,100.0f,50.0f,5.0f,0.1f,1.0f,0.1f,0.6f,0.0f};

float Trigger_Speed_Loop_Data[PID_DATA_LEN]
={0.60f,0.2f,0.0f,20.0f,5.0f,0.0f,0.4f,0.05f,0.0f,0.0f};
//拨弹轮初始化
void Trigger_Init(void)
{
	M3508_Init(&trigger.motor,0x201,50.895f,0.3f);
	trigger.parameter.state	=	TRIGGER_STOP;
	trigger.parameter.shoot_num	=	0;
	trigger.parameter.last_shoot_num	=	trigger.parameter.shoot_num;
	PID_Init(&trigger.pid.angle_loop,Trigger_Angle_Loop_Data,Integral_Limit|ChangingIntegralRate|OutputFilter);
	PID_Init(&trigger.pid.speed_loop,Trigger_Speed_Loop_Data,Integral_Limit|ChangingIntegralRate);
}
//拨弹轮状态更新
void Trigger_Status_Update(void)
{
	 M3508_Status_Update(&trigger.motor);
	trigger.status.actual_speed	=	trigger.motor.status.velocity_rpm/trigger.motor.parameter.reduction_rate;
	trigger.status.total_angle =	trigger.motor.status.total_position_degree/trigger.motor.parameter.reduction_rate;
	while(trigger.status.total_angle-trigger.status.rounds*360.0f>360.0f)trigger.status.rounds++;
	while(trigger.status.total_angle-trigger.status.rounds*360.0f<-360.0f) trigger.status.rounds--;
    trigger.status.actual_angle    =   trigger.status.total_angle - trigger.status.rounds*360.0f;
}


//拨弹轮指令更新
void Trigger_Command_Update(void)
{
	trigger.command.target_total_position	=	trigger.parameter.shoot_num*(60.0f);
	while(trigger.command.target_position	-	trigger.command.rounds*360.0f>360.0f)	trigger.command.rounds++;
	while(trigger.command.target_position	-	trigger.command.rounds*360.0f<-360.0f)	trigger.command.rounds--;
	trigger.command.target_position	=	trigger.command.target_total_position	-	trigger.command.rounds*360.0f	;
	PID_Calculate(&trigger.pid.angle_loop,trigger.status.total_angle,-trigger.command.target_total_position);
	trigger.command.target_speed	=	trigger.pid.angle_loop.Output;
	PID_Calculate(&trigger.pid.speed_loop,trigger.status.actual_speed,trigger.command.target_speed);
	M3508_Command_Update(&trigger.motor,trigger.pid.speed_loop.Output);
	
}
//拨弹轮缓冲区数据更新
void Trigger_Send_Command_Update(void)
{
	switch(trigger.parameter.state)
	{
		case TRIGGER_STOP:
			memset(&CAN2_0x200_Tx_Data,0,8);
			break;
		case TRIGGER_RUNNING	:

			CAN2_0x200_Tx_Data[0]=trigger.motor.command.give_current_lsb>>8;
			CAN2_0x200_Tx_Data[1]=trigger.motor.command.give_current_lsb;
			break;
	}
}

void	Trigger_Task(void)
{
	// 更新拨弹轮状态
	Trigger_Status_Update();
	// 更新拨弹轮命令
	Trigger_Command_Update();
	// 更新拨弹轮发送命令
	Trigger_Send_Command_Update();
}