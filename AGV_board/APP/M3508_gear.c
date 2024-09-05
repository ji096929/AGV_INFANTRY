#include "M3508_gear.h"

#if defined(EXAMPLE_M3508_GEAR)
	M3508_gear_t example_M3508_gear;
#endif


void M3508_gear_feedback_handler(M3508_gear_t *kit)
{
	M3508_gear_feedback_process(kit);
	kit->flag++;
	kit->status.output_speed_rpm = kit->feedback.current_rotor_rpm / kit->parameter.reduction_rate;
}

void M3508_gear_init(M3508_gear_t *kit, M3508_gear_parameter_t *init_struct)
{
	M3508_gear_parameter_init(kit, init_struct);	
	
}

void M3508_gear_parameter_init(M3508_gear_t *kit, M3508_gear_parameter_t *init_struct)
{
	kit->parameter.ESC_ID 				= init_struct->ESC_ID;
	kit->parameter.bus					= init_struct->bus;
	kit->parameter.reduction_rate		= init_struct->reduction_rate;
	kit->parameter.resistance_torque	= init_struct->resistance_torque;
	kit->parameter.handle				= init_struct->handle;
	M3508_Init(kit->parameter.bus, init_struct->handle);
}

void M3508_gear_set_torque_current_lsb(M3508_gear_t *kit, int16_t torque_current_lsb, M3508_SINGLE_COMMAND_HOLD_t hold)
{
	kit->command.torque_current_lsb	= torque_current_lsb;
	kit->command.torque_current		= M3508_from_lsb_to_torque_current(kit->command.torque_current_lsb);
	kit->command.torque				= M3508_from_torque_current_to_torque(kit->command.torque_current) * kit->parameter.reduction_rate;
	M3508_gear_command_transmit(kit, hold);
}

