
/* Includes ------------------------------------------------------------------*/
#include "M3508_bsp.h"


/**
 * @brief 处理M3508电机的反馈数据
 *
 * 该函数用于处理来自M3508电机的反馈数据，将其存储到电机总线结构体中，并进行单位转换（如角度和电流）。
 *
 * @param  motor_bus 电机所在的总线
 * @param  data      返回的数据
 * @param  CAN_ID    CAN通信ID，用于确定是哪个电机发送的数据
 */
M3508_FEEDBACK_PROCESS_RETURN_T M3508_feedback_process(M3508_motor_bus_t *motor_bus, uint16_t CAN_ID, uint8_t data[])
{
	// 判断是哪个电机
	uint8_t ESC_ID = CAN_ID - 0x200;
	
	// 防止混入别的数据
	if (ESC_ID<1 | ESC_ID>8) return M3508_FEEDBACK_PROCESS_NOT_3508_ID;
	
	// 先把数据存起来
	motor_bus->motor[ESC_ID].feedback.LSB_rotor_position	= data[0]<<8 | data[1];
	motor_bus->motor[ESC_ID].feedback.LSB_rotor_rpm			= data[2]<<8 | data[3];
	motor_bus->motor[ESC_ID].feedback.LSB_torque_current	= data[4]<<8 | data[5];
	motor_bus->motor[ESC_ID].feedback.LSB_tempreture		= data[6];
	
 	// 需要单位转换的
	motor_bus->motor[ESC_ID].status.rotor_position_deg	= M3508_from_lsb_to_position_deg(motor_bus->motor[ESC_ID].feedback.LSB_rotor_position);
	motor_bus->motor[ESC_ID].status.rotor_position_rad	= M3508_from_lsb_to_position_rad(motor_bus->motor[ESC_ID].feedback.LSB_rotor_position);
	motor_bus->motor[ESC_ID].status.torque_current		= M3508_from_lsb_to_torque_current(motor_bus->motor[ESC_ID].feedback.LSB_torque_current);
	// 不需要单位转换的
	
	motor_bus->motor[ESC_ID].status.rotor_rpm	= motor_bus->motor[ESC_ID].feedback.LSB_rotor_rpm;
	motor_bus->motor[ESC_ID].status.tempreture	= motor_bus->motor[ESC_ID].feedback.LSB_tempreture;
	
	return M3508_FEEDBACK_PROCESS_OK;
}


/**
 * @brief 通过M3508电机总线发送电机命令
 *
 * 该函数用于将电机命令发送到M3508电机总线。它将电机的扭矩电流命令分成两个数据包（dataA和dataB）并发送到电机总线上。
 *
 * @param  ctx           发送用结构体
 * @param  motor_bus     电机所在的总线
 * @param  trans_setting 传输设置，可以是0、1或2
 *                      - 1：发送到总线上的1~4号电机
 *                      - 2：发送到总线上的5~8号电机
 *                      - 0：发送到总线上的所有电机
 */
void M3508_command_transmit(M3508_ctx_t *ctx, M3508_motor_bus_t *motor_bus, uint8_t trans_setting)
{
	if (trans_setting == 0 | trans_setting == 1)
	{
		uint8_t dataA[8];
		dataA[0] = motor_bus->motor[1].command.torque_current_lsb >> 8;
		dataA[1] = motor_bus->motor[1].command.torque_current_lsb & 0xFF;
		dataA[2] = motor_bus->motor[2].command.torque_current_lsb >> 8;
		dataA[3] = motor_bus->motor[2].command.torque_current_lsb & 0xFF;
		dataA[4] = motor_bus->motor[3].command.torque_current_lsb >> 8;
		dataA[5] = motor_bus->motor[3].command.torque_current_lsb & 0xFF;
		dataA[6] = motor_bus->motor[4].command.torque_current_lsb >> 8;
		dataA[7] = motor_bus->motor[4].command.torque_current_lsb & 0xFF;
		ctx->tx_cmd(motor_bus->handle, LOWER_IDENTIFIER, dataA);
	}

	if (trans_setting == 0 | trans_setting == 2)
	{
		uint8_t dataB[8];
		dataB[0] = motor_bus->motor[5].command.torque_current_lsb >> 8;
		dataB[1] = motor_bus->motor[5].command.torque_current_lsb & 0xFF;
		dataB[2] = motor_bus->motor[6].command.torque_current_lsb >> 8;
		dataB[3] = motor_bus->motor[6].command.torque_current_lsb & 0xFF;
		dataB[4] = motor_bus->motor[7].command.torque_current_lsb >> 8;
		dataB[5] = motor_bus->motor[7].command.torque_current_lsb & 0xFF;
		dataB[6] = motor_bus->motor[8].command.torque_current_lsb >> 8;
		dataB[7] = motor_bus->motor[8].command.torque_current_lsb & 0xFF;
		ctx->tx_cmd(motor_bus->handle, UPPER_IDENTIFIER, dataB);
	}
}

float M3508_from_lsb_to_position_deg(uint16_t lsb)
{
	return ( lsb / 8191.f * 360.f );
}

float M3508_from_lsb_to_position_rad(uint16_t lsb)
{
	return ( lsb / 8191.f * PI*2 );
}

float M3508_from_lsb_to_torque_current(int16_t lsb)
{
	return ( lsb / 16384.f * 20.f );
}

float M3508_from_torque_current_to_torque(float current)
{
	return ( current * M3508_TORQUE_COEFFICIENT);
}
