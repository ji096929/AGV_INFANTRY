/* USER Settings -------------------------------------------------------------*/
#define STM32F105

/* Includes ------------------------------------------------------------------*/
#include "M3508_bsp.h"

#if defined(STM32F105) | (STM32F407)
	#include "can.h"
#endif

/* Motor Bus Settings --------------------------------------------------------*/
#if defined(STM32F105) | (STM32F407)
	//F105, F407支持两条CAN总线，所以开两个结构体
	M3508_motor_bus_t M3508_bus_1;
	M3508_motor_bus_t M3508_bus_2;
#endif

/* CAN Bus Settings ----------------------------------------------------------*/
#if defined(STM32F105) | (STM32F407)
	#define MOTOR_BUS_1 hcan1
	#define MOTOR_BUS_2 hcan2
	
	CAN_TxHeaderTypeDef CAN_TxHeaderStruct;
	uint32_t  pTxMailbox;
#endif

static int32_t platform_trans(void *handle, uint16_t CAN_ID, uint8_t aData[]);
static int32_t platform_get_msg(void *handle, uint16_t CAN_ID, uint8_t aData[]);


M3508_ctx_t dev_ctx;
void M3508_Init(void)
{
	/* Initialize mems driver interface */
	dev_ctx.rx_feedback	= platform_get_msg;
	dev_ctx.tx_cmd		= platform_trans;
	
	/* Initialize CAN driver interface */
	#if defined(STM32F105) | (STM32F407)
		M3508_bus_1.handle = &MOTOR_BUS_1;
		M3508_bus_2.handle = &MOTOR_BUS_2;
		
		CAN_TxHeaderStruct.ExtId=0;
		CAN_TxHeaderStruct.DLC=8;
		CAN_TxHeaderStruct.IDE=CAN_ID_STD;
		CAN_TxHeaderStruct.RTR=CAN_RTR_DATA;
		CAN_TxHeaderStruct.TransmitGlobalTime=DISABLE;
	#endif
}

void M3508_set_single_current(M3508_motor_bus_t *M3508_bus, uint8_t ESC_ID, uint16_t current)
{
	//传递电机所在的总线
	dev_ctx.handle = M3508_bus->handle;
	//传递电流值
	M3508_bus->motor[ESC_ID].command.torque_current = current;
	//调用CAN发送
	M3508_command_transmit(&dev_ctx, M3508_bus, (ESC_ID%4 + 1));
}

void M3508_set_half_current(M3508_motor_bus_t *M3508_bus, uint8_t half, uint16_t current[])
{
	//传递电机所在的总线
	dev_ctx.handle = M3508_bus->handle;
	uint8_t offset = 0;
	if (half) offset = 4;
	M3508_bus->motor[1+offset].command.torque_current = current[0];
	M3508_bus->motor[2+offset].command.torque_current = current[1];
	M3508_bus->motor[3+offset].command.torque_current = current[2];
	M3508_bus->motor[4+offset].command.torque_current = current[3];
	
	
}

static int32_t platform_trans(void *handle, uint16_t CAN_ID, uint8_t aData[])
{
	#if defined(STM32F105) | (STM32F407)
		CAN_TxHeaderStruct.StdId=CAN_ID;
		return HAL_CAN_AddTxMessage(handle, &CAN_TxHeaderStruct, aData, &pTxMailbox);
	#endif
}

static int32_t platform_get_msg(void *handle, uint16_t CAN_ID, uint8_t aData[])
{
	#if defined(STM32F105) | (STM32F407)
		CAN_RxHeaderTypeDef CAN_RxHeaderStruct;
		return HAL_CAN_GetRxMessage(handle, CAN_RX_FIFO0, &CAN_RxHeaderStruct, aData);
	#endif
}