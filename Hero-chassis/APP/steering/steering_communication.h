#ifndef STEERING_COMMUNICATION_H
#define STEERING_COMMUNICATION_H

#ifdef __cplusplus
extern "C" {
#endif

/* USER Settings -------------------------------------------------------------*/
#define STM32F105
#define STEERING_COMMUNICATION_QUEUE_LENGTH 10 // ʵ��ֵ��LENGTH+1��LENGTH���ֵ255������ֵ��ռ�ô����ڴ�ռ�

#if defined(STM32F105) | (STM32F407)
	#define STEERING_COMMUNICATION_HANDLE hcan2
#endif

#define DEFAULT_STEERING_CAN_ID 0x1AU
	#define A_STEERING_CAN_ID 0x1AU
	#define B_STEERING_CAN_ID 0x1BU
	#define C_STEERING_CAN_ID 0x1CU
	#define D_STEERING_CAN_ID 0x1DU
/* SYSTEM Settings, DONT CHANGE EASILY! --------------------------------------*/
#define SUBSCRIBE_LIST_MAXIMUM_LENGTH 8
/* Includes ------------------------------------------------------------------*/

/** @addtogroup stdint
  * @{
  */
#include <stdint.h>
#include "steering_communication_bsp.h"
#include "chassis_power_control.h"
	typedef enum
	{
		STEERING_COMMUNICATION_OK,
		STEERING_COMMUNICATION_ERROR,
		STEERING_COMMUNICATION_WRONG_PARAM,

		STEERING_COMMUNICATION_SUBCRIBE_LIST_FULL,
		STEERING_COMMUNICATION_SUBCRIBE_LIST_OVERWRITED,
		STEERING_COMMUNICATION_SUBCRIBE_LIST_NOT_SUBSCRIBE_YET,
	} STEERING_COMMUNICATION_RETURN_T;

	typedef enum
	{

		DISABLE_CONTROLLING = 0x00U, // Ĭ��ǿ��ֹͣ����
		ENABLE_CONTROLLING = 0x01U,
		SET_VELOCITY_VECTOR = 0x03U,

		GET_PID_PARAMETER = 0x11U, // ��ȡ PID ����ֵ
		SET_PID_PARAMETER = 0x12U, // ���� PID ����ֵ

		ADD_SUBSCRIBE_VALUE = 0x0AU,
		DELETE_SUBSCRIBED_VALUE = 0x0BU,
		CHECK_SUBSCRIBE_LIST = 0x0CU,

		SUBSCRIBE_RETURN_CMD_ID = 0x1EU,
		RETURN_CMD_ID = 0x1FU,
	} steering_communication_command_id_t;

	// 新增
	typedef enum
	{

		A_MscB = 0x1AU,
		B_MscB = 0x1BU,
		C_MscB = 0x1CU,
		D_MscB = 0x1DU,
	} steering_communication_MscB_id_t;

	typedef struct
	{
		uint16_t motion_motor_rpm[4];
		uint16_t directive_motor_angle_lsb[4];
		int16_t motion_motor_torque_current_lsb[4];
		int16_t directive_motor_torque_current_lsb[4];
	} subscribed_data_t;

	extern subscribed_data_t subscribed_data;
	typedef enum
	{

		motion_motor_rpm = 0x01U,
		directive_motor_angle_lsb = 0x02U,
		motion_motor_torque_current_lsb = 0x03U,
		directive_motor_torque_current_lsb = 0x04U,
	} steering_communication_subscribe_content_offset_id_t;

	void steering_communication_init(void);
	extern steering_communication_pack_t steering_pack;
	
	STEERING_COMMUNICATION_RETURN_T steering_communication_rx_handler(uint32_t extid, uint8_t data1[]);
STEERING_COMMUNICATION_RETURN_T steering_communication_SET_VELOCITY_VECTOR(steering_communication_MscB_id_t MscB,
																			   int16_t ProtocolPosition, int16_t ProtocolSpeed, float scaled_power_coefficient_32, steering_communication_pack_t tx_pack);

	STEERING_COMMUNICATION_RETURN_T steering_communication_ADD_SUBSCRIBE_VALUE(steering_communication_MscB_id_t MscB,
																			   uint8_t subscribe_content_2_offset_id, uint8_t subscribe_content_1_offset_id, uint16_t callback_cnt2, uint16_t callback_term2, uint16_t callback_cnt1, uint16_t callback_term1, steering_communication_pack_t tx_pack);

#endif
