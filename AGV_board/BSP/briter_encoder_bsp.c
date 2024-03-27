#include "briter_encoder_bsp.h"
briter_encoder_t test_t;

void test(void)
{
	test_t.command.command_code = GET_TOTAL_ANGLE;
}

uint8_t briter_encoder_feedback_process(briter_encoder_t *encoder, uint8_t data[])
{
	//先把数据存起来
	memcpy(&encoder->feedback, &data[0], 8);
	uint8_t cpy_length;
	cpy_length = encoder->feedback.length - 3;
	//通过指令码接收数据
	switch( encoder->feedback.command_code)
	{
		case GET_TOTAL_ANGLE:
			memcpy(&encoder->status.last_total_angle, &encoder->status.total_angle,		sizeof(encoder->status.total_angle)); // 存好上次的角度
			memcpy(&encoder->status.total_angle		, &encoder->feedback.data, 			sizeof(encoder->status.total_angle)); // 写入当前的角度
			return BRITER_ENCODER_FEEDBACK_PROCESS_OK;
			break;
		default:
			if (encoder->feedback.data[0] != 0)
				return BRITER_ENCODER_FEEDBACK_PROCESS_RETURN_ERR;
			else return BRITER_ENCODER_FEEDBACK_PROCESS_OK;
			break;
	}
	return BRITER_ENCODER_FEEDBACK_PROCESS_NO_SUCH_CODE;
}

uint8_t briter_encoder_command_transmit(briter_encoder_ctx_t *ctx, briter_encoder_t *encoder)
{
	uint8_t dataA[8];
	memcpy(&dataA, &encoder->command, 8);
	return ctx->tx_cmd(encoder->parameter.handle, encoder->parameter.CAN_ID, dataA);

}
