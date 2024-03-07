#ifndef __MY_INIT_H
#define __MY_INIT_H

#include "pid.h"
#include "chassis_move.h"
#include "trigger_turn.h"
#include "tim.h"
#include "remote_control.h"
#include "bsp_referee.h"
#include "yaw_turn.h"
#include "bsp_uart.h"
#include "steering_communication.h"
void all_init(void);
void mode_init(void);
void Motor_Zero_Position_Init(void);
#endif

