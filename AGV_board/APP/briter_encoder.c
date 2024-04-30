
/* Includes ------------------------------------------------------------------*/
#include "briter_encoder.h"

#if defined(STM32F105) | (STM32F407)
    #include "can.h"
#endif

/* CAN Bus Settings ----------------------------------------------------------*/
#if defined(STM32F105) | (STM32F407)
	#define BRITER_ENCODER_BUS_1 hcan1
    #define BRITER_ENCODER_BUS_2 hcan2
	
    CAN_TxHeaderTypeDef briter_encoder_CAN_TxHeaderStruct;
    uint32_t  briter_encoder_pTxMailbox;
#endif

/* Encoder Settings ----------------------------------------------------------*/
#if defined(briter_encoder_num_1)
	briter_encoder_t briter_encoder;
#endif



static uint8_t platform_trans(void *handle, uint16_t CAN_ID, uint8_t aData[]);

briter_encoder_ctx_t briter_encoder_ctx;
void briter_encoder_Init(briter_encoder_t *encoder)
{
    /* Initialize transmission functions */
    briter_encoder_ctx.tx_cmd = platform_trans;
    
    /* Initialize CAN driver interface */
    #if defined(STM32F105) | (STM32F407)
        encoder->parameter.handle = &BRITER_ENCODER_BUS_1;

        briter_encoder_CAN_TxHeaderStruct.ExtId = 0;
        briter_encoder_CAN_TxHeaderStruct.DLC = 8;
        briter_encoder_CAN_TxHeaderStruct.IDE = CAN_ID_STD;
        briter_encoder_CAN_TxHeaderStruct.RTR = CAN_RTR_DATA;
        briter_encoder_CAN_TxHeaderStruct.TransmitGlobalTime = DISABLE;
    #endif
}

/**
 * @brief 把预先设定好的编码器参数导入结构体中
 */
BRITER_ENCODER_COMMAND_RETURN_t briter_encoder_parameter_init(briter_encoder_t *encoder, birter_encoder_parameter_t *init_struct)
{
	#if defined(briter_encoder_num_1)
		encoder->parameter.baud_rate			= init_struct->baud_rate;
		encoder->parameter.call_back_mode		= init_struct->call_back_mode;
		encoder->parameter.increment_direction	= init_struct->increment_direction;
		encoder->parameter.CAN_ID				= init_struct->CAN_ID;
	#endif
	return BRITER_ENCODER_COMMAND_OK;
}

BRITER_ENCODER_COMMAND_RETURN_t briter_encoder_request_tatal_angle(briter_encoder_t *encoder)
{
    // Pass the bus where the encoder is located
    briter_encoder_ctx.handle = encoder->parameter.handle;

	encoder->command.length = BRITER_ENCODER_DATA_LENGTH_4;
	encoder->command.encoder_address = encoder->parameter.CAN_ID;
	encoder->command.command_code = GET_TOTAL_ANGLE;
	
	uint8_t trans_data[5];
	memcpy(&encoder->command.data, &trans_data, 5);
	
    // Call the CAN transmission function to send the encoder command
    return briter_encoder_command_transmit(&briter_encoder_ctx, encoder);
}

BRITER_ENCODER_COMMAND_RETURN_t briter_encoder_set_CAN_ID(briter_encoder_t *encoder, uint8_t CAN_ID)
{
    // Pass the bus where the encoder is located
    briter_encoder_ctx.handle = encoder->parameter.handle;
    // 检查参数是否正确
    if (CAN_ID>=0x00 && CAN_ID<=0xFF)
	{
		encoder->command.length = BRITER_ENCODER_DATA_LENGTH_4;
		encoder->command.encoder_address = encoder->parameter.CAN_ID;
		encoder->command.command_code = SET_CAN_ID;
		uint8_t trans_data[5];
		trans_data[0] = CAN_ID;
		memcpy(&encoder->command.data, &trans_data, 5);
		encoder->parameter.CAN_ID = CAN_ID;
	}
	else return BRITER_ENCODER_COMMAND_WRONG_PARAMETER;
    // Call the CAN transmission function to send the encoder command
    return briter_encoder_command_transmit(&briter_encoder_ctx, encoder);
}

BRITER_ENCODER_COMMAND_RETURN_t briter_encoder_set_baud_rate(briter_encoder_t *encoder, BRITER_ENCODER_CAN_BAUD_RATE_t rate)
{
    // Pass the bus where the encoder is located
    briter_encoder_ctx.handle = encoder->parameter.handle;
    // 检查参数是否正确
    if (rate>=0 && rate<=4)
	{
		encoder->command.length = BRITER_ENCODER_DATA_LENGTH_4;
		encoder->command.encoder_address = encoder->parameter.CAN_ID;
		encoder->command.command_code = SET_CAN_BAUD_RATE;
		uint8_t trans_data[5];
		trans_data[0] = rate;
		memcpy(&encoder->command.data, &trans_data, 5);
		encoder->parameter.baud_rate = rate;
	}
	else return BRITER_ENCODER_COMMAND_WRONG_PARAMETER;
    // Call the CAN transmission function to send the encoder command
    return briter_encoder_command_transmit(&briter_encoder_ctx, encoder);
}

BRITER_ENCODER_COMMAND_RETURN_t briter_encoder_set_callback_mode(briter_encoder_t *encoder, BRITER_ENCODER_CALLBACK_MODE_t mode)
{
    // Pass the bus where the encoder is located
    briter_encoder_ctx.handle = encoder->parameter.handle;
    // 检查参数是否正确
    if (mode==BRITER_ENCODER_SET_CALLBACK_REQUEST | mode==BRITER_ENCODER_SET_CALLBACK_PERIODICAL)
	{
		encoder->command.length = BRITER_ENCODER_DATA_LENGTH_4;
		encoder->command.encoder_address = encoder->parameter.CAN_ID;
		encoder->command.command_code = SET_CALLBACK_MODE;
		uint8_t trans_data[5];
		trans_data[0] = mode;
		memcpy(&encoder->command.data, &trans_data, 5);
		encoder->parameter.call_back_mode = mode;
	}
	else return BRITER_ENCODER_COMMAND_WRONG_PARAMETER;
    // Call the CAN transmission function to send the encoder command
    return briter_encoder_command_transmit(&briter_encoder_ctx, encoder);
}

BRITER_ENCODER_COMMAND_RETURN_t briter_encoder_set_callback_period(briter_encoder_t *encoder, uint16_t period)
{
    // Pass the bus where the encoder is located
    briter_encoder_ctx.handle = encoder->parameter.handle;
    // 检查参数是否正确
    if (period>=50 | period<=65535)
	{
		encoder->command.length = BRITER_ENCODER_DATA_LENGTH_5;
		encoder->command.encoder_address = encoder->parameter.CAN_ID;
		encoder->command.command_code = SET_CALLBACK_MODE;
		uint8_t trans_data[5];
		memcpy(&trans_data[0], &period, 2);
		memcpy(&encoder->command.data, &trans_data, 5);
		encoder->parameter.call_back_period = period;
	}
	else return BRITER_ENCODER_COMMAND_WRONG_PARAMETER;
    // Call the CAN transmission function to send the encoder command
    return briter_encoder_command_transmit(&briter_encoder_ctx, encoder);
}

BRITER_ENCODER_COMMAND_RETURN_t briter_encoder_set_current_pos_zero_pos(briter_encoder_t *encoder)
{
    // Pass the bus where the encoder is located
    briter_encoder_ctx.handle = encoder->parameter.handle;

	encoder->command.length = BRITER_ENCODER_DATA_LENGTH_4;
	encoder->command.encoder_address = encoder->parameter.CAN_ID;
	encoder->command.command_code = SET_CURRENT_POS_ZERO_POS;
	uint8_t trans_data[5];
	memcpy(&encoder->command.data, &trans_data, 5);
    // Call the CAN transmission function to send the encoder command
    return briter_encoder_command_transmit(&briter_encoder_ctx, encoder);
}

BRITER_ENCODER_COMMAND_RETURN_t briter_encoder_set_increment_direction(briter_encoder_t *encoder, BRITER_ENCODER_INCREMENT_DIRECTION_t direction)
{
    // Pass the bus where the encoder is located
    briter_encoder_ctx.handle = encoder->parameter.handle;
    // 检查参数是否正确
    if (direction==BRITER_ENCODER_INCREMENT_DIRECTION_CW | direction==BRITER_ENCODER_INCREMENT_DIRECTION_CCW)
	{
		encoder->command.length = BRITER_ENCODER_DATA_LENGTH_4;
		encoder->command.encoder_address = encoder->parameter.CAN_ID;
		encoder->command.command_code = SET_INCREMENT_DIRECTION;
		uint8_t trans_data[5];
		trans_data[0] = direction;
		memcpy(&encoder->command.data, &trans_data, 5);
		encoder->parameter.increment_direction = direction;
	}
	else return BRITER_ENCODER_COMMAND_WRONG_PARAMETER;
    // Call the CAN transmission function to send the encoder command
    return briter_encoder_command_transmit(&briter_encoder_ctx, encoder);
}

BRITER_ENCODER_COMMAND_RETURN_t briter_encoder_set_current_pos_mid_pos(briter_encoder_t *encoder)
{
    // Pass the bus where the encoder is located
    briter_encoder_ctx.handle = encoder->parameter.handle;
    // 检查参数是否正确
	encoder->command.length = BRITER_ENCODER_DATA_LENGTH_4;
	encoder->command.encoder_address = encoder->parameter.CAN_ID;
	encoder->command.command_code = SET_CURRENT_POS_MID_POS;
	uint8_t trans_data[5];
	memcpy(&encoder->command.data, &trans_data, 5);
    // Call the CAN transmission function to send the encoder command
    return briter_encoder_command_transmit(&briter_encoder_ctx, encoder);
}

BRITER_ENCODER_COMMAND_RETURN_t briter_encoder_set_current_pos_specific_value(briter_encoder_t *encoder, uint32_t value)
{
    // Pass the bus where the encoder is located
    briter_encoder_ctx.handle = encoder->parameter.handle;
    // 检查参数是否正确
    if (value<=0xFFFFFFFF)
	{
		encoder->command.length = BRITER_ENCODER_DATA_LENGTH_7;
		encoder->command.encoder_address = encoder->parameter.CAN_ID;
		encoder->command.command_code = SET_CURRENT_POS_SPECIFIC_VALUE;
		memcpy(&encoder->command.data, &value , 5);
	}
	else return BRITER_ENCODER_COMMAND_WRONG_PARAMETER;
    // Call the CAN transmission function to send the encoder command
    return briter_encoder_command_transmit(&briter_encoder_ctx, encoder);
}

BRITER_ENCODER_COMMAND_RETURN_t briter_encoder_set_current_pos_5round_value(briter_encoder_t *encoder)
{
    // Pass the bus where the encoder is located
    briter_encoder_ctx.handle = encoder->parameter.handle;
    // 检查参数是否正确
	encoder->command.length = BRITER_ENCODER_DATA_LENGTH_4;
	encoder->command.encoder_address = encoder->parameter.CAN_ID;
	encoder->command.command_code = SET_CURRENT_POS_5ROUND_VALUE;
	uint8_t trans_data[5];
	memcpy(&encoder->command.data, &trans_data, 5);
    // Call the CAN transmission function to send the encoder command
    return briter_encoder_command_transmit(&briter_encoder_ctx, encoder);
}

void briter_encoder_feedback_handler(briter_encoder_t *encoder, uint8_t data[])
{
	encoder->flag++;
	briter_encoder_feedback_process(encoder, data);
}

/**
 * @brief Transmit data via CAN bus
 *
 * This function is used to transmit data via the CAN bus.
 *
 * @param  handle   Pointer to the CAN handle
 * @param  CAN_ID   CAN ID for the message
 * @param  aData    Array of data to transmit
 *
 * @return Status of the transmission (0 if successful, non-zero otherwise)
 */
static uint8_t platform_trans(void *handle, uint16_t CAN_ID, uint8_t aData[])
{
    #if defined(STM32F105) | (STM32F407)
        briter_encoder_CAN_TxHeaderStruct.StdId = CAN_ID;
        return HAL_CAN_AddTxMessage(handle, &briter_encoder_CAN_TxHeaderStruct, aData, &briter_encoder_pTxMailbox);
    #endif
}
