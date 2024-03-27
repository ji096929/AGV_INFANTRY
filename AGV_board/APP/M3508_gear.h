/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef M3508_GEAR_H
#define M3508_GEAR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Settings ------------------------------------------------------------------*/
#define EXAMPLE_M3508_GEAR

/* Includes ------------------------------------------------------------------*/
#include "M3508_gear_bsp.h"


void M3508_gear_init(M3508_gear_t *kit, M3508_gear_parameter_t *init_struct);
void M3508_gear_feedback_handler(M3508_gear_t *kit);
void M3508_gear_set_torque_current_lsb(M3508_gear_t *kit, int16_t torque_current_lsb, M3508_SINGLE_COMMAND_HOLD_t hold);

void M3508_gear_parameter_init(M3508_gear_t *M3508_gear, M3508_gear_parameter_t *init_struct);

#if defined(EXAMPLE_M3508_GEAR)
	extern M3508_gear_t example_M3508_gear;
#endif



#ifdef __cplusplus
}
#endif
#endif
