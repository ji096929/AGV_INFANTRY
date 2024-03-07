#ifndef BRITER_ENCODER_BSP_H
#define BRITER_ENCODER_BSP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Public typedef ------------------------------------------------------------*/
typedef uint8_t (*briter_encoder_tx_ptr)(void *, uint16_t , uint8_t[]);
typedef uint8_t (*briter_encoder_rx_ptr)(void *, uint16_t *, uint8_t[]);

typedef enum
{
	BRITER_ENCODER_DATA_LENGTH_4	= 0x04,
	BRITER_ENCODER_DATA_LENGTH_5	= 0x05,
	BRITER_ENCODER_DATA_LENGTH_7	= 0x07,
} BRITER_ENCODER_TRANS_LENGTH_t;

typedef enum
{
	BRITER_ENCODER_FEEDBACK_PROCESS_OK = 0x00,
	BRITER_ENCODER_FEEDBACK_PROCESS_RETURN_ERR,
	BRITER_ENCODER_FEEDBACK_PROCESS_NO_SUCH_CODE,

} BRITER_ENCODER_FEEDBACK_PROCESS_RETURN_t;

typedef enum
{
	BRITER_ENCODER_COMMAND_OK = 0x00,
	BRITER_ENCODER_COMMAND_ERROR,
	BRITER_ENCODER_COMMAND_WRONG_PARAMETER,
} BRITER_ENCODER_COMMAND_RETURN_t;

typedef enum
{
	GET_TOTAL_ANGLE					= 0x01,
	SET_CAN_ID						= 0x02,
	SET_CAN_BAUD_RATE				= 0x03,
	SET_CALLBACK_MODE				= 0x04,
	SET_CALLBACK_PERIOD				= 0x05,
	SET_CURRENT_POS_ZERO_POS		= 0x06,
	SET_INCREMENT_DIRECTION			= 0x07,
	SET_CURRENT_POS_MID_POS			= 0x0C,
	SET_CURRENT_POS_SPECIFIC_VALUE	= 0x0D,
	SET_CURRENT_POS_5ROUND_VALUE	= 0x0F

} BRITER_ENCODER_COMMAND_CODE_t;

typedef enum
{
	BRITER_ENCODER_SET_CAN_BAUD_RATE_500K = 0x00,
	BRITER_ENCODER_SET_CAN_BAUD_RATE_1M,
	BRITER_ENCODER_SET_CAN_BAUD_RATE_250K,
	BRITER_ENCODER_SET_CAN_BAUD_RATE_125K,
	BRITER_ENCODER_SET_CAN_BAUD_RATE_100K

} BRITER_ENCODER_CAN_BAUD_RATE_t;

typedef enum
{
	BRITER_ENCODER_SET_CALLBACK_REQUEST	= 0x00,
	BRITER_ENCODER_SET_CALLBACK_PERIODICAL	= 0xAA

} BRITER_ENCODER_CALLBACK_MODE_t;

typedef enum
{
	BRITER_ENCODER_INCREMENT_DIRECTION_CW = 0x00,
	BRITER_ENCODER_INCREMENT_DIRECTION_CCW

} BRITER_ENCODER_INCREMENT_DIRECTION_t;

typedef struct
{
	uint8_t length;
	uint8_t encoder_address;
	uint8_t command_code;
	uint8_t data[5];

} briter_encoder_command_t;

typedef struct
{
	uint8_t length;
	uint8_t encoder_address;
	uint8_t command_code;
	uint8_t data[5];

} briter_encoder_feedback_t;

typedef struct
{
	uint32_t	total_angle;
	uint32_t	last_total_angle;

} briter_encoder_status_t;

typedef struct
{
	uint32_t								CAN_ID;
	BRITER_ENCODER_CAN_BAUD_RATE_t			baud_rate;
	BRITER_ENCODER_CALLBACK_MODE_t			call_back_mode;
	uint16_t								call_back_period;
	BRITER_ENCODER_INCREMENT_DIRECTION_t	increment_direction;
	/** customizable  **/
	void *handle;

} birter_encoder_parameter_t;

typedef struct
{
	birter_encoder_parameter_t	parameter;
	briter_encoder_command_t	command;
	briter_encoder_feedback_t	feedback;
	briter_encoder_status_t		status;
}briter_encoder_t;

typedef struct
{
	/** essential  **/
	briter_encoder_tx_ptr	tx_cmd;

	/** customizable  **/
	void *handle;
} briter_encoder_ctx_t;

uint8_t briter_encoder_command_transmit(briter_encoder_ctx_t *ctx, briter_encoder_t *encoder);
uint8_t briter_encoder_feedback_process(briter_encoder_t *encoder, uint8_t data[]);

#ifdef __cplusplus
}
#endif

#endif
