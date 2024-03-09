#include "M3508_gear_bsp.h"

void M3508_gear_feedback_process(M3508_gear_t *kit)
{
	// 电机转子数据获取
	kit->feedback.current_rotor_position_lsb	= kit->parameter.bus->motor[kit->parameter.ESC_ID].feedback.LSB_rotor_position;
	kit->feedback.current_rotor_rpm				= kit->parameter.bus->motor[kit->parameter.ESC_ID].feedback.LSB_rotor_rpm;
	kit->feedback.rotor_torque_current			= kit->parameter.bus->motor[kit->parameter.ESC_ID].feedback.LSB_torque_current;
	
	// 电机转子力矩转换
	kit->status.rotor_torque = kit->feedback.rotor_torque_current * M3508_TORQUE_COEFFICIENT;
	kit->status.actual_current	=	kit->feedback.rotor_torque_current/16384.f*20.0f;
	// 单圈角度与多圈角度转换
	if (kit->feedback.current_rotor_position_lsb - kit->feedback.last_rotor_position_lsb < -8191/2) 
		kit->status.rotor_total_round++;
	if (kit->feedback.current_rotor_position_lsb - kit->feedback.last_rotor_position_lsb >  8191/2)
		kit->status.rotor_total_round--;
	kit->status.rotor_total_lsb = kit->status.rotor_total_round * 8192 + kit->feedback.current_rotor_position_lsb;
	
	// 电机输出轴角速度换算
	kit->status.output_speed_rpm = kit->feedback.current_rotor_rpm / kit->parameter.reduction_rate;
	
	// 电机输出轴力矩换算，为输入轴力矩放大后减去减速箱阻力
	kit->status.output_torque = kit->status.rotor_torque * kit->parameter.reduction_rate - kit->parameter.resistance_torque;
	
	// 存储上次的角度，以便多圈解算
	kit->feedback.last_rotor_position_lsb		= kit->feedback.current_rotor_position_lsb;
}

void M3508_gear_command_transmit(M3508_gear_t *kit, M3508_SINGLE_COMMAND_HOLD_t hold)
{
	M3508_set_single_motor_current(kit->parameter.bus, kit->parameter.ESC_ID, kit->command.torque_current_lsb, hold);
}