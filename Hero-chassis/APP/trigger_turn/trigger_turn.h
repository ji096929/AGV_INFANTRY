#ifndef __TRIGGER_TURN_H
#define __TRIGGER_TURN_H

#include "pid.h"
#include "remote_control.h"
#include "stm32f4xx.h"
#include "yaw_turn.h"


#define SHOOT_NUM 1          //发射1发
#define Reduction_Ratio 50.895
//#define Reduction_Ratio 19.2
typedef struct{
	
	float Target_Speed;
	float actual_speed;
	
	float Target_Angle;
	float actual_angle;
	
	int last_shoot_flag;
	
	float set_currunt;
	
	float last_angle;			//上一次机械转子角度
	int rounds;				    //转过的圈数
	int Total_Angle;			//总共转过的角度
	int last_speed;       		//上一次真实转速
	int record_begin_angle_status;//是否记录了电机初始角度 0代表没有记录，1代表记录成功
	int begin_angle;            //电机初始角度
	
	PidTypeDef speed_pid;
	PidTypeDef angle_pid;
}trigger_t;

extern trigger_t trigger;
void trigger_turn(void);
void trigger_all_init(void);
uint8_t canTX_trigger(int16_t trigger);
void Trigger_Motor_Callback(trigger_t *motor,uint16_t angle, int16_t speed);
void shoot_angle_clc(void);
#endif
