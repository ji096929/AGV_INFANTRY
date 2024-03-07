
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef STEERING_COMMUNICATION_H
#define STEERING_COMMUNICATION_H

#ifdef __cplusplus
extern "C" {
#endif

/* USER Settings -------------------------------------------------------------*/
#define STM32F105
#define STEERING_COMMUNICATION_QUEUE_LENGTH 10 // 实际值是LENGTH+1。LENGTH最大值255，过大值会占用大量内存空间

#if defined(STM32F105) | (STM32F407)
	#define STEERING_COMMUNICATION_HANDLE hcan2
#endif


/* SYSTEM Settings, DONT CHANGE EASILY! --------------------------------------*/
#define SUBSCRIBE_LIST_MAXIMUM_LENGTH 8
/* Includes ------------------------------------------------------------------*/

/** @addtogroup stdint
  * @{
  */
#include <stdint.h>
#include "steering_communication_bsp.h"
#include "steering_wheel.h"
typedef enum
{
	STEERING_COMMUNICATION_OK,
	STEERING_COMMUNICATION_ERROR,
	STEERING_COMMUNICATION_WRONG_PARAM,
	
	STEERING_COMMUNICATION_SUBCRIBE_LIST_FULL,
	STEERING_COMMUNICATION_SUBCRIBE_LIST_OVERWRITED,
	STEERING_COMMUNICATION_SUBCRIBE_LIST_NOT_SUBSCRIBE_YET,
}STEERING_COMMUNICATION_RETURN_T;

typedef enum
{
	
	DISABLE_CONTROLLING		= 0x00U, // 默认强制停止运行
	ENABLE_CONTROLLING		= 0x01U, 
	SET_VELOCITY_VECTOR  	= 0x03U,
	
	GET_PID_PARAMETER		= 0x11U, // 获取 PID 参数值
	SET_PID_PARAMETER		= 0x12U, // 设置 PID 参数值
	

	ADD_SUBSCRIBE_VALUE		= 0x0AU,
	DELETE_SUBSCRIBED_VALUE	= 0x0BU,
	CHECK_SUBSCRIBE_LIST	= 0x0CU,
	
	SUBSCRIBE_RETURN_CMD_ID	= 0x1EU,
	RETURN_CMD_ID			= 0x1FU,
}steering_communication_command_id_t;


typedef struct
{
	steering_communication_pack_t element[STEERING_COMMUNICATION_QUEUE_LENGTH+1];
	uint8_t head, tail; // 不要轻易更改!
	uint8_t boundary;
}steering_communication_queue_t;


void steering_communication_init(void);
STEERING_COMMUNICATION_RETURN_T steering_communication_rx_handler(uint32_t extid, uint8_t data1[]);
STEERING_COMMUNICATION_RETURN_T steering_communication_SubscribeList_Scheduler(steering_wheel_t *steering);

#ifdef __cplusplus
}
#endif

#endif