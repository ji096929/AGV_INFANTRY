/*
 * @Author: ShenYuJie 1647262812@qq.com
 * @Date: 2024-02-26 15:39:49
 * @LastEditors: ShenYuJie 1647262812@qq.com
 * @LastEditTime: 2024-03-11 00:59:48
 * @FilePath: \MDK-ARMd:\EVENTS\2024_Hero\GIMBAL\app\task_schedule.h
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
#ifndef TASK_SCHEDULE_H_
#define TASK_SCHEDULE_H_

#include "tim.h"
#include "bsp_imu.h"
#include "fric.h"
#include "remote_control.h"
#include "gimbal.h"
#include "can_connection.h"
#include "trigger.h"
#include "usbd_cdc_if.h"


#define task_schedule()    HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)

void Init_Task(void);

typedef enum 
{
	INIT_STATE	=	0x00u,
	RUNNING_STATE	=	0x01u,
}SYSTEM_STATE_T;

typedef __packed struct
{
	uint16_t s_count;
	uint16_t ms_count;
	SYSTEM_STATE_T state;
}TASK_TIME_T;

extern TASK_TIME_T gimbal_time;

#endif
