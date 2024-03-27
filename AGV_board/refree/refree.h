/*
 * @Author: ShenYuJie 1647262812@qq.com
 * @Date: 2024-03-07 16:09:19
 * @LastEditors: ShenYuJie 1647262812@qq.com
 * @LastEditTime: 2024-03-07 16:11:39
 * @FilePath: \refree\refree.h
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
#ifndef REFREE_H_
#define REFREE_H_

#include "algorithmOfCRC.h"

#define JudgeBufBiggestSize 45

typedef struct
{
  char HeatUpdate_NoUpdate;
	char SpeedUpdate_NoUpdate;
	
	//0x0201
	uint8_t robot_id;
	uint8_t RobotLevel;
	uint16_t remainHP;
	uint16_t maxHP;
	uint16_t HeatCool42;		//17mm枪口每秒冷却值
	uint16_t HeatMax42;			//17mm枪口热量上限
	uint16_t BulletSpeedMax42;	//17mm枪口上限速度
	uint16_t MaxPower;			//底盘功率限制上限

	//0x0202
	uint16_t realChassisOutV;
	uint16_t realChassisOutA;
	float realChassispower;
	uint16_t remainEnergy;       //剩余能量
	short shooterHeat42;
	
	//0x0207
	uint8_t bulletFreq;		//射击频率
	uint8_t ShootCpltFlag; //已射出一发子弹标志位
	
	//flag
	short HeatUpdateFlag;	
	
	//not in use
	uint8_t cardType;
	uint8_t CardIdx;

	float bulletSpeed;		//当前射速
	float LastbulletSpeed;
}
JudgeReceive_t;

void JudgeBuffReceive(unsigned char ReceiveBuffer[],uint16_t DataLen);

#endif