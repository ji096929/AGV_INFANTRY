#ifndef CHASSIS_TASK_H_
#define CHASSIS_TASK_H_

#include "stdint.h"
#include "tim.h"
#include "chassis.h"
#include "can_connection.h"
#include "agv_control.h"
#include "referee.h"
#include "ui.h"
typedef struct 
{
    uint16_t ms_count;
    uint16_t s_count;
    uint32_t total_count;
}TIME_T;

extern TIME_T time;


void Task_Init(void);

#define Chassis_Task()    HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)

#endif
