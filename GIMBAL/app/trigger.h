#ifndef TRIGGER_H_
#define TRIGGER_H_

#include "motor.h"
#include "pid.h"
#include "drv_can.h"
typedef enum
{
	TRIGGER_STOP	=	0x00,
	TRIGGER_RUNNING,
}TRIGGER_STATAE_E;

typedef	struct
{
	PID_TypeDef angle_loop;
	PID_TypeDef speed_loop;
}TRIGGER_PID_T;

typedef struct
{
	float target_position;
	float target_total_position;
	int16_t rounds;
	float target_speed;
}TRIGGER_COMMAND_T;

typedef struct
{
	TRIGGER_STATAE_E state;
	uint16_t shoot_num;
	uint16_t last_shoot_num;
}TRIGGER_PAPAMETER_T;

typedef struct
{
	int16_t rounds;
	float total_angle;
	float actual_angle;

	float actual_speed;
}TRIGGER_STATUS_T;

typedef struct
{
	TRIGGER_STATUS_T status;
	TRIGGER_PAPAMETER_T parameter;
	TRIGGER_COMMAND_T command;
	M3508_T motor;
	TRIGGER_PID_T pid;
}TRIGGER_T;

extern TRIGGER_T trigger;

void Trigger_Init(void);
void	Trigger_Task(void);

#endif