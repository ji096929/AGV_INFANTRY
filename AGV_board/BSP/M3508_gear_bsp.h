/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef M3508_GEAR_BSP_H
#define M3508_GEAR_BSP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Settings ------------------------------------------------------------------*/


/* Includes ------------------------------------------------------------------*/
#include "M3508.h"

typedef struct
{
	void *handle;
	float				resistance_torque;
	M3508_motor_bus_t	*bus;
	uint8_t 			ESC_ID;
	float				reduction_rate; // Output / Input
	
} M3508_gear_parameter_t;

typedef struct
{
	int		rotor_total_lsb;
	int		rotor_total_round;
	int		rotor_torque;
	
	float	output_speed_rpm;
	int		output_angle_lsb;
	float	output_angle_rad;
	float	output_angle_deg;
	float	output_torque;
	float actual_current;
} M3508_gear_status_t;

typedef struct
{
	int16_t	current_rotor_position_lsb;
	int16_t	last_rotor_position_lsb;
	
	int16_t	current_rotor_rpm;
	int16_t	rotor_torque_current;
} M3508_gear_feedback_t;


typedef struct
{
	float		torque;
	float		torque_current;
	int16_t	torque_current_lsb;
} M3508_gear_command_t;



typedef struct
{
	
	M3508_gear_parameter_t	parameter;
	M3508_gear_command_t	command;
	M3508_gear_feedback_t	feedback;
	M3508_gear_status_t		status;
	uint32_t flag;
	uint32_t preflag;
} M3508_gear_t;

void M3508_gear_command_transmit(M3508_gear_t *kit, M3508_SINGLE_COMMAND_HOLD_t hold);


#ifdef __cplusplus
}
#endif
#endif
