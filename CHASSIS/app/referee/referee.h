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

#define HEADER_SOF 0xA5
#define REF_PROTOCOL_FRAME_MAX_SIZE 128
#define REF_PROTOCOL_HEADER_SIZE sizeof(frame_header_struct_t)
#define REF_PROTOCOL_CMD_SIZE 2
#define REF_PROTOCOL_CRC16_SIZE 2
#define REF_HEADER_CRC_LEN (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE)
#define REF_HEADER_CRC_CMDID_LEN (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + sizeof(uint16_t))
#define REF_HEADER_CMDID_LEN (REF_PROTOCOL_HEADER_SIZE + sizeof(uint16_t))

#define JudgeBufBiggestSize 45
#define USART_RX_BUF_LENGHT 512
#define REFEREE_FIFO_BUF_LENGTH 1024
//新加的数据都放在最后
typedef struct
{
	char HeatUpdate_NoUpdate;
	char SpeedUpdate_NoUpdate;


	// 0x0201
	uint8_t robot_id;
	uint8_t RobotLevel;
	uint16_t remainHP;
	uint16_t maxHP;
	uint16_t HeatCool17;
	uint16_t HeatMax17;
	uint16_t MaxPower;
	uint8_t PowerOutPut;

	// 0x0202
	uint16_t realChassisOutV;
	uint16_t realChassisOutA;
	float realChassispower;
	uint16_t remainEnergy;
	short shooterHeat17;

	// 0x0207
	uint8_t bullet_type;
	uint8_t shooter_number;
	uint8_t launching_frequency;
	float initial_speed;
	
		//0x0001
	uint8_t game_type;
	uint8_t game_progress;

	// flag
	short HeatUpdateFlag;

	// not in use
	uint8_t cardType;
	uint8_t CardIdx;

	float bulletSpeed;
	float LastbulletSpeed;

	bool receive_flag;
} JudgeReceive_t;

typedef enum
{
	GAME_STATE_CMD_ID = 0x0001,
	GAME_RESULT_CMD_ID = 0x0002,
	GAME_ROBOT_HP_CMD_ID = 0x0003,
	FIELD_EVENTS_CMD_ID = 0x0101,
	SUPPLY_PROJECTILE_ACTION_CMD_ID = 0x0102,
	SUPPLY_PROJECTILE_BOOKING_CMD_ID = 0x0103,
	REFEREE_WARNING_CMD_ID = 0x0104,
	ROBOT_STATE_CMD_ID = 0x0201,
	POWER_HEAT_DATA_CMD_ID = 0x0202,
	ROBOT_POS_CMD_ID = 0x0203,
	BUFF_MUSK_CMD_ID = 0x0204,
	AERIAL_ROBOT_ENERGY_CMD_ID = 0x0205,
	ROBOT_HURT_CMD_ID = 0x0206,
	SHOOT_DATA_CMD_ID = 0x0207,
	BULLET_REMAINING_CMD_ID = 0x0208,
	STUDENT_INTERACTIVE_DATA_CMD_ID = 0x0301,
	IDCustomData,
} referee_cmd_id_t;

typedef __packed struct
{
	uint8_t SOF;
	uint16_t data_length;
	uint8_t seq;
	uint8_t CRC8;
} frame_header_struct_t;

typedef enum
{
	STEP_HEADER_SOF = 0,
	STEP_LENGTH_LOW = 1,
	STEP_LENGTH_HIGH = 2,
	STEP_FRAME_SEQ = 3,
	STEP_HEADER_CRC8 = 4,
	STEP_DATA_CRC16 = 5,
} unpack_step_e;

typedef struct
{
	frame_header_struct_t *p_header;
	uint16_t data_len;
	uint8_t protocol_packet[REF_PROTOCOL_FRAME_MAX_SIZE];
	unpack_step_e unpack_step;
	uint16_t index;
} unpack_data_t;

extern JudgeReceive_t JudgeReceive;
void Referee_Init(void);
void Judge_Buffer_Receive_Task(uint8_t *frame);
void referee_unpack_fifo_data(void);

#endif