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
	uint16_t HeatCool42;		//17mmǹ��ÿ����ȴֵ
	uint16_t HeatMax42;			//17mmǹ����������
	uint16_t BulletSpeedMax42;	//17mmǹ�������ٶ�
	uint16_t MaxPower;			//���̹�����������

	//0x0202
	uint16_t realChassisOutV;
	uint16_t realChassisOutA;
	float realChassispower;
	uint16_t remainEnergy;       //ʣ������
	short shooterHeat42;
	
	//0x0207
	uint8_t bulletFreq;		//���Ƶ��
	uint8_t ShootCpltFlag; //�����һ���ӵ���־λ
	
	//flag
	short HeatUpdateFlag;	
	
	//not in use
	uint8_t cardType;
	uint8_t CardIdx;

	float bulletSpeed;		//��ǰ����
	float LastbulletSpeed;
}
JudgeReceive_t;

void JudgeBuffReceive(unsigned char ReceiveBuffer[],uint16_t DataLen);

#endif