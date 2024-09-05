#ifndef GIMBAL_CONNECTION_H_
#define GIMBAL_CONNECTION_H_

#include "drv_can.h"
#include "chassis.h"

typedef enum
{
	INVERT_ON = 0x01,
	INVERT_OFF = 0x00,
} INVERT_FLAG_E;

enum Enum_Booster_Jamming_Type
{
	Booster_Not_Jamming = 0,
	Booster_Jamming,
};

typedef enum
{
	FOLLOW_ON = 0x01,
	FOLLOW_OFF = 0x00,
} FOLLOW_FLAG_E;

typedef enum
{
	Booster_User_Control_Type_DISABLE = 0,
	Booster_User_Control_Type_SINGLE = 1,
	Booster_User_Control_Type_MULTI = 2,
} FRIC_FLAG_E;

typedef enum
{
	Gimbal_Control_Type_DISABLE = 0,
	Gimbal_Control_Type_NORMAL,
	Gimbal_Control_Type_MINIPC,
} GIMBAL_FLAG_E;

typedef struct
{
	float remp;
} GIMBAL_TX_T;

typedef struct
{
	uint8_t flag;
	uint8_t last_flag;
} CHASSIS_MODE_STATE_T; // 1Ϊ������0Ϊ�ر�

typedef struct
{
	CHASSIS_MODE_E mode;
	CHASSIS_MODE_STATE_T invert;
	CHASSIS_MODE_STATE_T follow;
	CHASSIS_MODE_STATE_T supercap;
	CHASSIS_MODE_STATE_T Graphic_Init;
	CHASSIS_MODE_STATE_T fric;
	CHASSIS_MODE_STATE_T vision;
	CHASSIS_MODE_STATE_T vision_mode;
	CHASSIS_MODE_STATE_T gimbal;
	CHASSIS_MODE_STATE_T jamming;
	float pitch_angle;
	int16_t fric_speed;
	int16_t vx;
	int16_t vy;
	int16_t vw;

} GIMBAL_RX_T;

typedef struct
{
	GIMBAL_RX_T connection_rx;
	GIMBAL_TX_T connection_tx;

} GIMBAL_CONNECTION_T;
void Chassis_Flag_Update(GIMBAL_CONNECTION_T *connection);
void Fric_Speed_And_Pitch_Angle_Update(GIMBAL_CONNECTION_T *connection, uint8_t data[]);
void Chassis_Speed_Command_Update(GIMBAL_CONNECTION_T *connection, uint8_t data[]);
void Chassis_Control_Mode_Update(GIMBAL_CONNECTION_T *connection, uint8_t data[]);
void CAN_Chassis_TxCpltCallback();
extern GIMBAL_CONNECTION_T connection;
#endif