/*
 * @Author: ShenYuJie 1647262812@qq.com
 * @Date: 2024-03-07 16:09:19
 * @LastEditors: ShenYuJie 1647262812@qq.com
 * @LastEditTime: 2024-03-17 19:55:42
 * @FilePath: \MDK-ARMd:\EVENTS\2024_Hero\CHASSIS\CHASSIS\app\referee\referee.h
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
#ifndef REFREE_H_
#define REFREE_H_

#include "algorithmOfCRC.h"
#include "stdbool.h"

#define JudgeBufBiggestSize 45
#define USART_RX_BUF_LENGHT     512
#define REFEREE_FIFO_BUF_LENGTH 1024
typedef struct
{
  char HeatUpdate_NoUpdate;
	char SpeedUpdate_NoUpdate;
	
	//0x0201
	uint8_t robot_id;
	uint8_t RobotLevel;
	uint16_t remainHP;
	uint16_t maxHP;
	uint16_t HeatCool42;		
	uint16_t HeatMax42;			
	uint16_t BulletSpeedMax42;	
	uint16_t MaxPower;			

	//0x0202
	uint16_t realChassisOutV;
	uint16_t realChassisOutA;
	float realChassispower;
	uint16_t remainEnergy;       
	short shooterHeat42;
	
	//0x0207
	uint8_t bulletFreq;		
	uint8_t ShootCpltFlag; 
	
	//flag
	short HeatUpdateFlag;	
	
	//not in use
	uint8_t cardType;
	uint8_t CardIdx;

	float bulletSpeed;		
	float LastbulletSpeed;
	
	bool receive_flag;
}
JudgeReceive_t;
extern JudgeReceive_t JudgeReceive;
extern unsigned char JudgeReceiveBuffer[JudgeBufBiggestSize];
void Referee_Init(void);
void Judge_Buffer_Receive_Task(unsigned char ReceiveBuffer[],uint16_t DataLen);

#endif