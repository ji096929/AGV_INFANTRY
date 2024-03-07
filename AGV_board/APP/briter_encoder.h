#ifndef BRITER_ENCODER_H
#define BRITER_ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

/* USER Settings -------------------------------------------------------------*/
#define STM32F105
#define briter_encoder_num_1

/* Includes ------------------------------------------------------------------*/
#include "briter_encoder_bsp.h"


void briter_encoder_Init(briter_encoder_t *encoder);
void briter_encoder_feedback_handler(briter_encoder_t *encoder, uint8_t data[]);
BRITER_ENCODER_COMMAND_RETURN_t briter_encoder_request_tatal_angle(briter_encoder_t *encoder);
BRITER_ENCODER_COMMAND_RETURN_t briter_encoder_set_CAN_ID(briter_encoder_t *encoder, uint8_t CAN_ID);
BRITER_ENCODER_COMMAND_RETURN_t briter_encoder_set_baud_rate(briter_encoder_t *encoder, BRITER_ENCODER_CAN_BAUD_RATE_t rate);
BRITER_ENCODER_COMMAND_RETURN_t briter_encoder_set_callback_mode(briter_encoder_t *encoder, BRITER_ENCODER_CALLBACK_MODE_t mode);
BRITER_ENCODER_COMMAND_RETURN_t briter_encoder_set_callback_period(briter_encoder_t *encoder, uint16_t period);
BRITER_ENCODER_COMMAND_RETURN_t briter_encoder_set_current_pos_zero_pos(briter_encoder_t *encoder);
BRITER_ENCODER_COMMAND_RETURN_t briter_encoder_set_increment_direction(briter_encoder_t *encoder, BRITER_ENCODER_INCREMENT_DIRECTION_t direction);
BRITER_ENCODER_COMMAND_RETURN_t briter_encoder_set_current_pos_mid_pos(briter_encoder_t *encoder);
BRITER_ENCODER_COMMAND_RETURN_t briter_encoder_set_current_pos_specific_value(briter_encoder_t *encoder, uint32_t value);
BRITER_ENCODER_COMMAND_RETURN_t briter_encoder_set_current_pos_5round_value(briter_encoder_t *encoder);

BRITER_ENCODER_COMMAND_RETURN_t briter_encoder_parameter_init(briter_encoder_t *encoder, birter_encoder_parameter_t *init_struct);

#if defined(briter_encoder_num_1)
	extern briter_encoder_t briter_encoder;
#endif

#ifdef __cplusplus
}
#endif

#endif
