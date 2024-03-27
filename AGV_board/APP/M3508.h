/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef M3508_H
#define M3508_H

#ifdef __cplusplus
extern "C" {
#endif

/* USER Settings -------------------------------------------------------------*/
#define STM32F105

#if defined(STM32F105) | (STM32F407)
	#define MOTOR_BUS_1 hcan1
	#define MOTOR_BUS_2 hcan2
#endif


/* Includes ------------------------------------------------------------------*/
#include "M3508_bsp.h"


typedef enum
{
	M3508_OK,
	M3508_ERROR
}M3508_RETURN_T;
	



void M3508_Init(M3508_motor_bus_t *M3508_bus, void *bus);
void M3508_set_single_motor_current(M3508_motor_bus_t *M3508_bus, uint8_t ESC_ID, uint16_t current, M3508_SINGLE_COMMAND_HOLD_t hold);
void M3508_set_four_motor_current(M3508_motor_bus_t *M3508_bus, uint8_t half, uint16_t current[]);
void M3508_set_all_motor_current(M3508_motor_bus_t *M3508_bus, uint16_t current[]);

M3508_RETURN_T M3508_feedback_handler(M3508_motor_bus_t *M3508_bus, uint16_t CAN_ID, uint8_t data[]);

#if defined(STM32F105) | (STM32F407)
    extern M3508_motor_bus_t M3508_bus_1;
	//extern M3508_motor_bus_t M3508_bus_2;
#endif



#ifdef __cplusplus
}
#endif
#endif