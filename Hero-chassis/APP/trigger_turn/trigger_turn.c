/**
  ******************************************************************************
  * @file    Project/APP/trigger_turn.c 
  * @author  NEUQRM
  * @version V1.0.0
  * @date    12.2021
  * @brief   该文件包含了拨弹轮的pid参数值，赋值，pid运算，输出等一系列跟拨弹轮有关的函数
  ******************************************************************************
  * @attention
  ******************************************************************************
  ****************************(C) COPYRIGHT 2021 NEUQRM****************************

  ...........  革命尚未成功，同志仍需努力  ...........
*/

#include "trigger_turn.h"



trigger_t trigger;

float TriggerSpeed[5] 	= {10.0f,			0.7f,			0.0f,		30000,			9000};  //拨弹轮速度环
float TriggerAngle[5] 	= {0.20f,			0.7f,			0.000f,		1500,			0.0f};  //拨弹轮角度环

//更新发子弹个数
void shoot_angle_clc(void);
//拨弹轮PID计算
static void trigger_pid(void);
//拨弹轮附带yaw轴任务
void trigger_turn(void)
{
	//shoot_angle_clc();    //在接收云台对应指令时执行该函数（这里不执行）
	if(shoot_flag>=1) //shoot_flag用于延时
	{
		shoot_flag++;
	}
	if(shoot_flag>=25)
	{
//	   trigger.Target_Angle-=((Reduction_Ratio*360/6.f)*8191/360);
	   shoot_flag=0;
		trigger.last_shoot_flag=shoot_flag;
	}
	trigger_pid();
	canTX_trigger(trigger.set_currunt);
}

void trigger_all_init(void)//拨弹轮pid参数及状态初始化
{
	PID_Init(&trigger.speed_pid,TriggerSpeed);
	PID_Init(&trigger.angle_pid,TriggerAngle);
	
	trigger.actual_angle = 0.0f;
	trigger.actual_speed = 0.0f;
	
	trigger.last_shoot_flag = 0;
	
	trigger.Target_Angle = 13.0f;
	trigger.Target_Speed = 0.0f;
}
float Target_Angle;
void shoot_angle_clc(void)
{
	
	if(trigger.last_shoot_flag!=shoot_flag)
	{
		if(trigger.rounds>=400)
		{
				trigger.Target_Angle-=(trigger.rounds-3)*8192;
				if(trigger.Target_Angle<=0) trigger.Target_Angle=1;
				trigger.Total_Angle -=(trigger.rounds-3)*8192 ;
				trigger.begin_angle=trigger.Total_Angle;
				trigger.rounds=3;
		}
		Target_Angle=((SHOOT_NUM*Reduction_Ratio*360/6.f)*8191/360);          //发射1发；19.2是3508的减速比
//		Target_Angle+=((9.5*360/7.f)*8191/360);
		trigger.Target_Angle-=Target_Angle;
	}
	trigger.last_shoot_flag=shoot_flag;
}

int16_t sped;
static void trigger_pid(void)//拨弹轮pid赋值计算
{
	apid_vpid_realize(&trigger.angle_pid,trigger.Target_Angle,trigger.Total_Angle);
//trigger.Target_Speed = trigger.angle_pid.out;
	trigger.Target_Speed = trigger.angle_pid.out*sinf(((trigger.Target_Angle-trigger.Total_Angle)/(Reduction_Ratio*60*8191/360.f))*0.5f);
//	trigger.Target_Speed = sped;

	apid_vpid_realize(&trigger.speed_pid,trigger.Target_Speed,trigger.actual_speed);
	
	trigger.set_currunt = trigger.speed_pid.out;
}


uint8_t canTX_trigger(int16_t trigger)//拨弹轮电流输出
{
	CAN_TxHeaderTypeDef canFrame;
	uint8_t data[8]={0};
	uint32_t temp=0;
	
	canFrame.IDE=CAN_ID_STD;
	canFrame.StdId=0x1ff;
	canFrame.RTR=CAN_RTR_DATA;
	canFrame.DLC=8;
	canFrame.TransmitGlobalTime=DISABLE;
	data[0]=0;
	data[1]=0;
	data[2]=0;
	data[3]=0;
	data[4]=0;
	data[5]=0;
	data[6]=trigger>>8;
	data[7]=trigger&0xff;
	HAL_CAN_AddTxMessage(&hcan1, &canFrame, data,&temp);
	
	return temp;
}

void Trigger_Motor_Callback(trigger_t *motor,uint16_t angle, int16_t speed)//拨弹轮电调回调
{
	motor->last_angle = motor->actual_angle;
	motor->actual_angle = angle;
	motor->actual_speed = 0.5*(speed + motor->last_speed);
	motor->last_speed = speed;

	if(motor->record_begin_angle_status==0)
	{
		motor->begin_angle = angle;
		motor->record_begin_angle_status++;
	}
	if(motor->actual_angle - motor->last_angle > 4096)
		motor->rounds --;
	else if (motor->actual_angle - motor->last_angle < -4096)
		motor->rounds ++;
	motor->Total_Angle = motor->rounds * 8192 + motor->actual_angle;

	
}
