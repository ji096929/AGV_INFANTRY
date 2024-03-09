/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef M3508_BSP_H
#define M3508_BSP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Public typedef ------------------------------------------------------------*/
typedef uint8_t (*M3508_tx_ptr)(void *, uint16_t , uint8_t[]);
typedef uint8_t (*M3508_rx_ptr)(void *, uint16_t , uint8_t[]);

typedef enum
{
	SEND_COMMAND_NOW = 0x00,
	HOLD_FOR_AGGREGATION_TRANS
} M3508_SINGLE_COMMAND_HOLD_t;
typedef struct
{
	int16_t	LSB_rotor_position;
	int16_t	LSB_rotor_rpm;
	int16_t	LSB_torque_current;
	uint8_t	LSB_tempreture;
	
} M3508_feedback_t;

typedef struct
{
	float	rotor_position_rad;
	float	rotor_position_deg;
	int16_t		rotor_rpm;
	int		torque_current;
	int		tempreture;
	
} M3508_status_t;

typedef struct
{
	float	torque;
	float	torque_current;
	int16_t	torque_current_lsb;
} M3508_command_t;

typedef struct
{
	uint8_t	ESC_ID	:3;	// 电调ID
	
} M3508_parameter_t;

typedef struct
{
	M3508_parameter_t	parameter;
	M3508_feedback_t	feedback;
	M3508_status_t		status;
	M3508_command_t		command;

} M3508_t;

typedef struct
{
	M3508_t	motor[9];
	void	*handle;
	
} M3508_motor_bus_t;


typedef struct
{
	/** essential  **/
	M3508_tx_ptr	tx_cmd;
	
	/** customizable  **/
	void *handle;
} M3508_ctx_t;

typedef enum
{
	M3508_BSP_OK,
	M3508_BSP_ERROR
}M3508_BSP_RETURN_T;

typedef enum
{
	M3508_FEEDBACK_PROCESS_OK,
	M3508_FEEDBACK_PROCESS_NOT_3508_ID,
} M3508_FEEDBACK_PROCESS_RETURN_T;

/* Default Parameters --------------------------------------------------------*/
#define M3508_TORQUE_COEFFICIENT	0.3
#define	PI	3.14

#define LOWER_IDENTIFIER 0x200
#define UPPER_IDENTIFIER 0x1FF

float M3508_from_lsb_to_position_deg(uint16_t lsb);
float M3508_from_lsb_to_position_rad(uint16_t lsb);
float M3508_from_lsb_to_torque_current(int16_t lsb);
float M3508_from_torque_current_to_torque(float current);

M3508_FEEDBACK_PROCESS_RETURN_T M3508_feedback_process(M3508_motor_bus_t *motor_bus, uint16_t CAN_ID, uint8_t data[]);
void M3508_command_transmit(M3508_ctx_t *ctx, M3508_motor_bus_t *motor_bus, uint8_t trans_setting);

#ifdef __cplusplus
}
#endif
#endif
