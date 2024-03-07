/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SW_CONTROL_TASK_H
#define SW_CONTROL_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#include "steering_wheel.h"
#include "steering_communication.h"

void SW_control_task(void);
void SW_control_task_init(void);
void TASK_SCHEDULER(void);
void SW_subscribe_task(void);
	
extern steering_wheel_t steering_wheel;
#ifdef __cplusplus
}
#endif
#endif
