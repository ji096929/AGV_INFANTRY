/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef STEERING_COMMUNICATION_BSP_H
#define STEERING_COMMUNICATION_BSP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/** @addtogroup stdint
  * @{
  */
#include <stdint.h>

/* Public typedef ------------------------------------------------------------*/
typedef uint8_t (*steering_communication_tx_ptr)(void *, uint32_t , uint8_t[]); // handle, extended frame, data frame
typedef uint8_t (*steering_communication_rx_ptr)(void *, uint32_t , uint8_t[]); // handle, extended frame, data frame



typedef enum
{
	OFFSET_DIRECTIVE_POSITION_LOOP = 0x01U,
	OFFSET_DIRECTIVE_VELOCITY_LOOP,
	OFFSET_MOTION_POSITION_LOOP,
	OFFSET_MOTION_VELOCITY_LOOP
}GET_PID_PARAMETER_LOOP_OFFSET_ID_t;

typedef enum
{
	OFFSET_OUTPUT_TERM = 0x01U,
	OFFSET_PROPOSITIONAL_TERM,
	OFFSET_INTEGRAL_TERM,
	OFFSET_DIFFERENTIAL_TERM
}GET_PID_PARAMETER_LIMIT_OFFSET_ID_t;

typedef enum
{
	OFFSET_MOTION_PART_RPM						= 0x01U,
	OFFSET_DIRECTIVE_POSITION_LSB				= 0x02U,
	OFFSET_MOTION_MOTOR_TORQUE_CURRENT_LSB		= 0x03U,
	OFFSET_DIRECTIVE_MOTOR_TORQUE_CURRENT_LSB	= 0x04U,
}SUBSCRIBE_CONTENT_X_OFFSET_ID_t;

typedef struct
{
	/** essential  **/
	steering_communication_tx_ptr	tx_cmd;
	
	/** customizable  **/
	void *handle;
} steering_communication_ctx_t;


typedef struct
{
	int32_t *pValueAdd;
	int32_t wValue;
	SUBSCRIBE_CONTENT_X_OFFSET_ID_t	content_id;
	uint16_t						subscribe_times;
	uint16_t						subscribe_period;
}subscribe_param_t;

typedef struct
{
	uint16_t			remained_times;
	uint16_t			remained_ticks;
	uint8_t				subscribe_times_infinite_flag;
	subscribe_param_t	param;
}steering_communication_subscribe_list_unit_t;

typedef struct  __PACKED
{
	uint8_t		treated_flag; // 判断本数据包是否已经发送或接收。
	uint8_t		steering_id;
	uint16_t	data2;
	uint8_t		cmd_id;
	int8_t		data1[8];
}steering_communication_pack_t;



uint8_t steering_communication_transmit(steering_communication_ctx_t *ctx, steering_communication_pack_t *pack);
steering_communication_pack_t steering_communication_receive_unpack(uint32_t extid, uint8_t *data1);

#ifdef __cplusplus
}
#endif
#endif
